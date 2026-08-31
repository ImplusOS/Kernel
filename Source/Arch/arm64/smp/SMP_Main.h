#pragma once

#include <stdint.h>

void smp_init(void);
uint32_t smp_get_cpu_count(void);
uint32_t smp_get_possible_cpu_count(void);
uint32_t smp_get_current_cpu_id(void);
void smp_tlb_shootdown(uint64_t vaddr, uint64_t pages);
void smp_tlb_shootdown_handler(void);
int32_t smp_get_current_pid(void);
void smp_set_current_pid(int32_t pid);

