#pragma once

#include <stdint.h>

typedef struct {
    int (*configure)(const void *firmware_info);
    void (*eoi)(uint32_t vector);
    int (*route_irq)(uint32_t irq, uint32_t vector);
    void (*mask_irq)(uint32_t irq);
    void (*unmask_irq)(uint32_t irq);
    int (*using_local_timer)(void);
} interrupt_ops_t;

typedef struct {
    void (*init)(void);
    void (*enable_irq)(uint32_t irq);
    void (*disable_irq)(uint32_t irq);
    void (*set_handler)(uint32_t irq, void (*handler)(void));
    void (*send_eoi)(uint32_t irq);
    void (*send_ipi)(uint32_t target_cpu, uint32_t vector);
} interrupt_controller_t;

const interrupt_ops_t *interrupt_ops_get(void);
