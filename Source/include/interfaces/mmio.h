#pragma once

#include <stdint.h>

/*
 * Device-register accessors.
 *
 * On AArch64 an MMIO access must be a single plain LDR/STR with an immediate
 * or register offset. Under a hypervisor (HVF, KVM) a guest access to an
 * emulated device traps as a data abort, and the hypervisor can only replay
 * it when the syndrome is valid (ESR.ISV = 1). Load/store pair, writeback
 * addressing ("str w1, [x0], #4") and SIMD forms leave ISV = 0 -- QEMU on HVF
 * aborts on them ("Assertion failed: (isv)"). A plain `volatile` dereference
 * does not prevent GCC from picking a writeback form inside a loop, so the
 * instruction is spelled out here. TCG emulates every form, which is why this
 * only shows up on real virtualization.
 *
 * x86_64 has no such restriction; there these are ordinary volatile accesses.
 */

static inline uint8_t mmio_read8(const volatile void *addr)
{
#if defined(__aarch64__)
    uint32_t v;
    __asm__ volatile("ldrb %w0, [%1]" : "=r"(v) : "r"(addr) : "memory");
    return (uint8_t)v;
#else
    return *(const volatile uint8_t *)addr;
#endif
}

static inline uint16_t mmio_read16(const volatile void *addr)
{
#if defined(__aarch64__)
    uint32_t v;
    __asm__ volatile("ldrh %w0, [%1]" : "=r"(v) : "r"(addr) : "memory");
    return (uint16_t)v;
#else
    return *(const volatile uint16_t *)addr;
#endif
}

static inline uint32_t mmio_read32(const volatile void *addr)
{
#if defined(__aarch64__)
    uint32_t v;
    __asm__ volatile("ldr %w0, [%1]" : "=r"(v) : "r"(addr) : "memory");
    return v;
#else
    return *(const volatile uint32_t *)addr;
#endif
}

static inline uint64_t mmio_read64(const volatile void *addr)
{
#if defined(__aarch64__)
    uint64_t v;
    __asm__ volatile("ldr %0, [%1]" : "=r"(v) : "r"(addr) : "memory");
    return v;
#else
    return *(const volatile uint64_t *)addr;
#endif
}

static inline void mmio_write8(volatile void *addr, uint8_t value)
{
#if defined(__aarch64__)
    __asm__ volatile("strb %w0, [%1]" :: "r"((uint32_t)value), "r"(addr) : "memory");
#else
    *(volatile uint8_t *)addr = value;
#endif
}

static inline void mmio_write16(volatile void *addr, uint16_t value)
{
#if defined(__aarch64__)
    __asm__ volatile("strh %w0, [%1]" :: "r"((uint32_t)value), "r"(addr) : "memory");
#else
    *(volatile uint16_t *)addr = value;
#endif
}

static inline void mmio_write32(volatile void *addr, uint32_t value)
{
#if defined(__aarch64__)
    __asm__ volatile("str %w0, [%1]" :: "r"(value), "r"(addr) : "memory");
#else
    *(volatile uint32_t *)addr = value;
#endif
}

static inline void mmio_write64(volatile void *addr, uint64_t value)
{
#if defined(__aarch64__)
    __asm__ volatile("str %0, [%1]" :: "r"(value), "r"(addr) : "memory");
#else
    *(volatile uint64_t *)addr = value;
#endif
}
