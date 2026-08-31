#pragma once

#include <stdint.h>

int  ioapic_init(uint64_t phys_base, uint32_t gsi_base);
int  ioapic_is_present(void);
void ioapic_mask_all(void);
int  ioapic_route_irq(uint8_t irq, uint8_t vector, uint8_t dest_apic_id, int level_trigger, int active_low);
int  ioapic_mask_irq(uint32_t irq);
int  ioapic_unmask_irq(uint32_t irq);
