#include "interfaces/hal_io.h"
#include <stdint.h>

void hal_io_out8(uint16_t port, uint8_t value)
{
    __asm__ volatile("outb %0, %1" :: "a"(value), "Nd"(port));
}

uint8_t hal_io_in8(uint16_t port)
{
    uint8_t value;
    __asm__ volatile("inb %1, %0" : "=a"(value) : "Nd"(port));
    return value;
}

void hal_io_out16(uint16_t port, uint16_t value)
{
    __asm__ volatile("outw %0, %1" :: "a"(value), "Nd"(port));
}

uint16_t hal_io_in16(uint16_t port)
{
    uint16_t value;
    __asm__ volatile("inw %1, %0" : "=a"(value) : "Nd"(port));
    return value;
}

void hal_io_out32(uint16_t port, uint32_t value)
{
    __asm__ volatile("outl %0, %1" :: "a"(value), "Nd"(port));
}

uint32_t hal_io_in32(uint16_t port)
{
    uint32_t value;
    __asm__ volatile("inl %1, %0" : "=a"(value) : "Nd"(port));
    return value;
}

void hal_io_outsw(uint16_t port, const void *addr, uint32_t count)
{
    __asm__ volatile(
        "rep outsw"
        : "+S"(addr), "+c"(count)
        : "d"(port)
        : "memory"
    );
}
