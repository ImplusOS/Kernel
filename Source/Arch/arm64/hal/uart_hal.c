#include "interfaces/uart_hal.h"

#include "interfaces/mmio.h"

#include <stdint.h>

#define UART0_BASE 0x09000000UL

/* PL011 registers, accessed through mmio.h (see there for why). */
#define UART_REG(off) ((volatile void *)(uintptr_t)(UART0_BASE + (off)))
#define UART_DR   0x00u
#define UART_FR   0x18u
#define UART_IBRD 0x24u
#define UART_FBRD 0x28u
#define UART_LCRH 0x2Cu
#define UART_CR   0x30u

#define UART_FR_TXFF (1u << 5)

static void arm64_uart_write_char(char c)
{
    uint32_t timeout = 0x100000u;
    while ((mmio_read32(UART_REG(UART_FR)) & UART_FR_TXFF) != 0u) {
        if (--timeout == 0u) {
            return;
        }
    }

    mmio_write32(UART_REG(UART_DR), (uint32_t)(uint8_t)c);
}

static void arm64_uart_init(uint32_t baud)
{
    (void)baud;

    mmio_write32(UART_REG(UART_CR), 0x00000000u);
    mmio_write32(UART_REG(UART_IBRD), 26u);
    mmio_write32(UART_REG(UART_FBRD), 3u);
    mmio_write32(UART_REG(UART_LCRH), 3u << 5);
    mmio_write32(UART_REG(UART_CR), (1u << 0) | (1u << 8) | (1u << 9));
}

static void arm64_uart_write_string(const char *s)
{
    if (!s) {
        return;
    }

    while (*s) {
        if (*s == '\n') {
            arm64_uart_write_char('\r');
        }
        arm64_uart_write_char(*s++);
    }
}

static int arm64_uart_read_char(void)
{
    return -1;
}

static const uart_hal_t g_arm64_uart_hal = {
    .init = arm64_uart_init,
    .write_char = arm64_uart_write_char,
    .write_string = arm64_uart_write_string,
    .read_char = arm64_uart_read_char,
};

const uart_hal_t *uart_hal_get(void)
{
    return &g_arm64_uart_hal;
}
