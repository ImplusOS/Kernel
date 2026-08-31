#pragma once

#include <stdint.h>
#include <stddef.h>
#include "interfaces/hal_cpu.h"

typedef struct {
    volatile uint32_t value;
} spinlock_t;

static inline uint64_t irq_save_disable(void)
{
    return hal_cpu_save_interrupts();
}

static inline void irq_restore(uint64_t flags)
{
    hal_cpu_restore_interrupts(flags);
}

static inline void spinlock_init(spinlock_t *lock)
{
    if (lock != NULL) {
        lock->value = 0;
    }
}

static inline void memory_barrier_full(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

static inline void spinlock_lock(spinlock_t *lock)
{
    if (lock == NULL) {
        return;
    }

    while (__atomic_exchange_n(&lock->value, 1u, __ATOMIC_ACQUIRE) != 0u) {
        while (__atomic_load_n(&lock->value, __ATOMIC_RELAXED) != 0u) {
            hal_cpu_pause();
        }
    }
}

static inline void spinlock_unlock(spinlock_t *lock)
{
    if (lock == NULL) {
        return;
    }
    __atomic_store_n(&lock->value, 0u, __ATOMIC_RELEASE);
}
