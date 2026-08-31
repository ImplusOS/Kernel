#pragma once

#include <stdint.h>

void hal_io_out8(uint16_t port, uint8_t value);
uint8_t hal_io_in8(uint16_t port);
void hal_io_out16(uint16_t port, uint16_t value);
uint16_t hal_io_in16(uint16_t port);
void hal_io_out32(uint16_t port, uint32_t value);
uint32_t hal_io_in32(uint16_t port);
void hal_io_outsw(uint16_t port, const void *addr, uint32_t count);
