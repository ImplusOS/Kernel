#pragma once

#include <stdint.h>

void arm64_cpu_early_init(void);
uint64_t arm64_current_el(void);
void arm64_set_exception_vector(void *vector_base);
void arm64_enable_interrupts(void);
void arm64_disable_interrupts(void);
uint64_t arm64_irq_save_disable(void);
void arm64_irq_restore(uint64_t flags);
void arm64_wait_forever(void) __attribute__((noreturn));

