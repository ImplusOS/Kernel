#pragma once

#include <stdint.h>

typedef struct {
    uint8_t (*inb)(uint16_t port);
    void (*outb)(uint16_t port, uint8_t value);
    uint16_t (*inw)(uint16_t port);
    void (*outw)(uint16_t port, uint16_t value);
    uint32_t (*inl)(uint16_t port);
    void (*outl)(uint16_t port, uint32_t value);
} io_ops_t;

const io_ops_t *io_ops_get(void);
