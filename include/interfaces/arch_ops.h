#pragma once

#include <stdint.h>

#include "interfaces/timer_hal.h"

typedef struct {
    void (*early_init)(void);
    void (*init_cpu_tables)(void);
    void (*enable_interrupts)(void);
    void (*disable_interrupts)(void);
    uint64_t (*irq_save_disable)(void);
    void (*irq_restore)(uint64_t flags);
    void (*enter_user_mode)(uint64_t saved_rsp, uint64_t user_rsp, uint64_t address_space);
    int (*virtualization_init)(void);
    /* Added under Docs/Others/TODO_OS_Refactor.md phase P4 (8.1): the
     * per-arch timer_hal_t the kernel should hand to timer_init(), so
     * kernel_main.c no longer needs an `#ifdef PLATFORM_X86_64 ... #elif
     * PLATFORM_ARM64` to pick between `&lapic_timer_hal` and
     * `&generic_timer_hal`. Every arch_ops_t implementation must set this
     * (non-NULL) -- there is no sensible "no timer" default. */
    const timer_hal_t *(*get_timer_hal)(void);
} arch_ops_t;

const arch_ops_t *arch_ops_get(void);
