#include "interfaces/uart_hal.h"

#include <stdint.h>

#include "interfaces/hal_io.h"

#define COM1_BASE 0x3F8u

#define COM_REG_DATA       0u
#define COM_REG_INTERRUPT  1u
#define COM_REG_FIFO       2u
#define COM_REG_LINE       3u
#define COM_REG_MODEM      4u
#define COM_REG_STATUS     5u

#define COM_LINE_DLAB      0x80u
#define COM_STATUS_TX_EMPTY 0x20u

static void x86_uart_write_reg(uint16_t reg, uint8_t value)
{
    hal_io_out8((uint16_t)(COM1_BASE + reg), value);
}

static uint8_t x86_uart_read_reg(uint16_t reg)
{
    return hal_io_in8((uint16_t)(COM1_BASE + reg));
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
}

static void x86_uart_write_char(char c)
{
    uint32_t timeout = 0x100000u;
    while ((x86_uart_read_reg(COM_REG_STATUS) & COM_STATUS_TX_EMPTY) == 0u) {
        if (--timeout == 0u) {
            return;
        }
    }

    x86_uart_write_reg(COM_REG_DATA, (uint8_t)c);
}

static void x86_uart_write_string(const char *s)
{
    if (!s) {
        return;
    }

    while (*s) {
        if (*s == '\n') {
            x86_uart_write_char('\r');
        }
        x86_uart_write_char(*s++);
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
