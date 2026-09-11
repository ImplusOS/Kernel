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

/*
 * Serve any TLB shootdown another CPU is waiting on us for. Defined in
 * Arch/<arch>/smp/SMP_Main.c; a cheap no-op when nothing is in flight (and
 * always, before SMP is up).
 *
 * Declared here rather than pulled in from the SMP header to keep this file
 * free of arch includes -- it is included almost everywhere.
 *
 * Why a spinlock cares: most spinlocks in this kernel are taken with
 * interrupts disabled, so a CPU waiting here cannot take the shootdown IPI.
 * The CPU that sent it blocks until every other CPU acknowledges, so without
 * this the two sit and wait for each other until the sender's timeout expires
 * -- once per lock acquisition, which is enough to stall the whole system.
 */
void smp_tlb_poll(void);

static inline void spinlock_lock(spinlock_t *lock)
{
    if (lock == NULL) {
        return;
    }

    uint32_t spins = 0u;
    while (__atomic_exchange_n(&lock->value, 1u, __ATOMIC_ACQUIRE) != 0u) {
        while (__atomic_load_n(&lock->value, __ATOMIC_RELAXED) != 0u) {
            /* Not every iteration: smp_tlb_poll() reads the local APIC to find
             * out which CPU it is on, and a lock held for a while would spend
             * the whole wait doing that. Once in a while is enough to keep the
             * sender moving. */
            if ((++spins & 0x3Fu) == 0u) {
                smp_tlb_poll();
            }
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
