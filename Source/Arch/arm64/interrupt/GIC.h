#pragma once

#include <stdint.h>

int arm64_gic_init(uint64_t gicd_base, uint64_t gicr_base, uint64_t gicc_base);
void arm64_gic_eoi(uint32_t irq);
uint32_t arm64_gic_read_iar(void);
int arm64_gic_route_irq(uint32_t irq, uint32_t vector);
void arm64_gic_mask_irq(uint32_t irq);
void arm64_gic_unmask_irq(uint32_t irq);

