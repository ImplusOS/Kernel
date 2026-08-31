#include "interfaces/uart_hal.h"

#include <stdint.h>

#define UART0_BASE 0x09000000UL

#define UART_DR   (*(volatile uint32_t *)(UART0_BASE + 0x00))
#define UART_FR   (*(volatile uint32_t *)(UART0_BASE + 0x18))
#define UART_IBRD (*(volatile uint32_t *)(UART0_BASE + 0x24))
#define UART_FBRD (*(volatile uint32_t *)(UART0_BASE + 0x28))
#define UART_LCRH (*(volatile uint32_t *)(UART0_BASE + 0x2C))
#define UART_CR   (*(volatile uint32_t *)(UART0_BASE + 0x30))

#define UART_FR_TXFF (1u << 5)

static void arm64_uart_write_char(char c)
{
    uint32_t timeout = 0x100000u;
    while ((UART_FR & UART_FR_TXFF) != 0u) {
        if (--timeout == 0u) {
            return;
        }
    }

    UART_DR = (uint32_t)c;
}

static void arm64_uart_init(uint32_t baud)
{
    (void)baud;

    UART_CR = 0x00000000u;
    UART_IBRD = 26u;
    UART_FBRD = 3u;
    UART_LCRH = (3u << 5);
    UART_CR = (1u << 0) | (1u << 8) | (1u << 9);
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
