#ifndef IMPLUSOS_UART_HAL_H
#define IMPLUSOS_UART_HAL_H

#include <stdint.h>

typedef struct {
    void (*init)(uint32_t baud);
    void (*write_char)(char c);
    void (*write_string)(const char *s);
    int  (*read_char)(void);
} uart_hal_t;

const uart_hal_t *uart_hal_get(void);

#endif
