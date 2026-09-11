#pragma once

#include <stdint.h>

void smp_init(void);
uint32_t smp_get_cpu_count(void);
uint32_t smp_get_possible_cpu_count(void);
uint32_t smp_get_current_cpu_id(void);
void smp_tlb_shootdown(uint64_t vaddr, uint64_t pages);

/* x86_64 narrows a shootdown to the CPUs running one address space (see
 * Arch/x86_64/smp/SMP_Main.h). AArch64 broadcasts TLBI by ASID in hardware, so
 * there is nothing to wait for and nothing to narrow: these are here so
 * architecture-independent callers compile, and ignore the address space. */
void smp_tlb_shootdown_cr3(uint64_t cr3, uint64_t vaddr, uint64_t pages);
void smp_note_cr3(uint64_t cr3);
void smp_tlb_poll(void);
void smp_tlb_shootdown_handler(void);
int32_t smp_get_current_pid(void);
void smp_set_current_pid(int32_t pid);

