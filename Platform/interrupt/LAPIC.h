#pragma once

#include <stdint.h>

int  lapic_init(uint64_t phys_base);
int  lapic_is_present(void);
void lapic_ap_init(void);
void lapic_eoi(void);
uint32_t lapic_get_id(void);

uint32_t lapic_timer_current(void);
int  lapic_timer_start(uint8_t vector, uint32_t initial_count, int periodic, uint32_t divide);
void lapic_timer_stop(void);
void lapic_timer_ap_init(void);

void lapic_send_ipi(uint8_t apic_id, uint32_t icr_low);
