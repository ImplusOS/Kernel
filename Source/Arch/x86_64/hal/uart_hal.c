#include "interfaces/uart_hal.h"

#include <stdbool.h>
#include <stdint.h>

#include "interfaces/hal_io.h"
#include "kernel/config.h"

#define COM1_BASE 0x3F8u

#define COM_REG_DATA       0u
#define COM_REG_INTERRUPT  1u
#define COM_REG_FIFO       2u   /* write: FCR   read: IIR */
#define COM_REG_LINE       3u
#define COM_REG_MODEM      4u
#define COM_REG_STATUS     5u
#define COM_REG_SCRATCH    7u

#define COM_LINE_DLAB      0x80u
#define COM_STATUS_TX_EMPTY 0x20u
#define COM_IIR_FIFO_MASK   0xC0u

/* How long to wait for the transmit holding register to drain before giving
 * up on a byte. One character at 115200 baud is ~87us and an `in` on the
 * legacy 0x3F8 decode costs on the order of a microsecond, so a few thousand
 * polls already covers a character time with a wide margin -- and covers
 * 9600 baud too. The old value was 0x100000 (1,048,576) polls *per byte*:
 * on a machine whose LSR reads back as 0 (a chipset that decodes the port
 * but has no UART behind it, which is the common case on modern hardware)
 * every single serial_write_char() burned a full second of pure port I/O.
 * That alone made the whole system crawl, KVM or not. */
#define COM_TX_POLL_LIMIT   20000u

/* 16550A transmit FIFO depth. When the FIFO is enabled, LSR.THRE means "the
 * holding register/FIFO is empty", i.e. there is room for a whole FIFO load,
 * so re-polling THRE for every byte just leaves the line idle between
 * characters. Feeding a burst per poll keeps the shift register busy and
 * gets the port to its actual line rate. */
#define COM_TX_FIFO_DEPTH   16u

/* 0 = not probed yet, 1 = a UART answered, 2 = nothing there (writes become
 * no-ops rather than spinning out COM_TX_POLL_LIMIT per byte forever).
 *
 * Only ever written by x86_uart_init(), which runs once on the boot CPU
 * before any other CPU is up. The probe must not run from the write path:
 * it works by writing a value to the scratch register and reading it back,
 * so two CPUs probing at once each read the other's value, both conclude
 * "no UART", and all kernel serial output disappears for the rest of the
 * boot -- intermittently, and looking exactly like a hang. Until init has
 * run, writes go out unprobed. */
static uint8_t g_uart_state = 0u;
static uint8_t g_uart_burst = 1u;

static void x86_uart_write_reg(uint16_t reg, uint8_t value)
{
    hal_io_out8((uint16_t)(COM1_BASE + reg), value);
}

static uint8_t x86_uart_read_reg(uint16_t reg)
{
    return hal_io_in8((uint16_t)(COM1_BASE + reg));
}

/* Presence probe. The scratch register (offset 7) is plain read/write RAM on
 * every 16450 and later, so a value written to it must read back. A port that
 * nothing drives answers 0xFF, and one that is decoded but empty answers 0x00
 * -- both fail this, and both are exactly the cases where waiting on THRE can
 * never succeed. This is the same test Linux's 8250 driver uses.
 *
 * Deliberately non-destructive and deliberately *not* an MCR loopback test:
 * losing all serial output to an over-eager probe would be worse than the
 * spin it avoids, so anything that answers gets treated as a real UART. Set
 * OS_CONFIG_SERIAL_ASSUME_PRESENT to skip the probe entirely. */
static uint8_t x86_uart_probe(void)
{
#if defined(OS_CONFIG_SERIAL_ASSUME_PRESENT) && OS_CONFIG_SERIAL_ASSUME_PRESENT
    return 1u;
#else
    uint8_t saved = x86_uart_read_reg(COM_REG_SCRATCH);
    x86_uart_write_reg(COM_REG_SCRATCH, 0x5Au);
    bool ok = (x86_uart_read_reg(COM_REG_SCRATCH) == 0x5Au);
    if (ok) {
        x86_uart_write_reg(COM_REG_SCRATCH, 0xA5u);
        ok = (x86_uart_read_reg(COM_REG_SCRATCH) == 0xA5u);
    }
    x86_uart_write_reg(COM_REG_SCRATCH, saved);
    return ok ? 1u : 2u;
#endif
}

static void x86_uart_init(uint32_t baud)
{
    uint16_t divisor = 1u;
    if (baud != 0u) {
        uint32_t raw_divisor = 115200u / baud;
        if (raw_divisor == 0u) {
            raw_divisor = 1u;
        }
        divisor = (uint16_t)raw_divisor;
    }

    x86_uart_write_reg(COM_REG_INTERRUPT, 0x00);
    x86_uart_write_reg(COM_REG_LINE, COM_LINE_DLAB);
    x86_uart_write_reg(COM_REG_DATA, (uint8_t)(divisor & 0xFFu));
    x86_uart_write_reg(COM_REG_INTERRUPT, (uint8_t)((divisor >> 8) & 0xFFu));
    x86_uart_write_reg(COM_REG_LINE, 0x03);
    x86_uart_write_reg(COM_REG_FIFO, 0xC7);
    x86_uart_write_reg(COM_REG_MODEM, 0x0B);

    /* IIR[7:6] == 11b means the FIFOs are present *and* enabled. */
    g_uart_burst =
        ((x86_uart_read_reg(COM_REG_FIFO) & COM_IIR_FIFO_MASK) == COM_IIR_FIFO_MASK)
            ? (uint8_t)COM_TX_FIFO_DEPTH : 1u;

    g_uart_state = x86_uart_probe();
}

/* True when there is room to push a byte. Returns false only if the port
 * never drained, in which case the byte is dropped rather than blocking. */
static bool x86_uart_wait_tx(void)
{
    for (uint32_t t = COM_TX_POLL_LIMIT; t != 0u; --t) {
        if ((x86_uart_read_reg(COM_REG_STATUS) & COM_STATUS_TX_EMPTY) != 0u) {
            return true;
        }
    }
    return false;
}

static void x86_uart_write_char(char c)
{
    if (g_uart_state == 2u) {
        return;
    }
    if (!x86_uart_wait_tx()) {
        return;
    }
    x86_uart_write_reg(COM_REG_DATA, (uint8_t)c);
}

static void x86_uart_write_string(const char *s)
{
    if (!s || g_uart_state == 2u) {
        return;
    }

    uint32_t room = 0u;
    while (*s) {
        char c = *s++;
        /* One THRE poll per FIFO load instead of per byte. */
        if (room == 0u) {
            if (!x86_uart_wait_tx()) {
                return;
            }
            room = g_uart_burst;
        }
        if (c == '\n') {
            x86_uart_write_reg(COM_REG_DATA, (uint8_t)'\r');
            if (--room == 0u) {
                if (!x86_uart_wait_tx()) {
                    return;
                }
                room = g_uart_burst;
            }
        }
        x86_uart_write_reg(COM_REG_DATA, (uint8_t)c);
        --room;
    }
}

static int x86_uart_read_char(void)
{
    return -1;
}

static const uart_hal_t g_x86_uart_hal = {
    .init = x86_uart_init,
    .write_char = x86_uart_write_char,
    .write_string = x86_uart_write_string,
    .read_char = x86_uart_read_char,
};

const uart_hal_t *uart_hal_get(void)
{
    return &g_x86_uart_hal;
}
