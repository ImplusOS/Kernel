#pragma once

#include <stdint.h>

#define VECTOR_TLB_SHOOTDOWN 0xFE

void     smp_init(void);
uint32_t smp_get_cpu_count(void);
uint32_t smp_get_possible_cpu_count(void);
uint32_t smp_get_current_cpu_id(void);
/*
 * Invalidate `pages` pages starting at `vaddr` in the address space `cr3`, on
 * every CPU that could be holding a translation for it, and do not return
 * until they have. `pages == 0` means "flush the whole TLB". `cr3 == 0` means
 * "every CPU", for a change to something every address space shares.
 *
 * Naming the address space is what makes the wait affordable. Without PCID an
 * x86 CR3 load flushes the entire TLB, so a CPU that is not currently running
 * `cr3` cannot be holding a stale translation for it and does not have to be
 * waited on -- which on a 16-vCPU guest is nearly every CPU.
 */
void     smp_tlb_shootdown_cr3(uint64_t cr3, uint64_t vaddr, uint64_t pages);

/* Shorthands. Both name no address space, so both wait for every CPU; prefer
 * smp_tlb_shootdown_cr3() wherever the caller knows which one it is changing. */
void     smp_tlb_shootdown(uint64_t vaddr, uint64_t pages);
void     smp_tlb_shootdown_all(void);
void     smp_tlb_shootdown_handler(void);

/* Publish the address space this CPU has just loaded, so shootdowns for other
 * address spaces need not wait for it. Called from paging_switch_cr3(). */
void     smp_note_cr3(uint64_t cr3);

/* Service any shootdown a peer is waiting on this CPU for. Cheap when there is
 * nothing outstanding. Called from the spinlock wait loop, because most locks
 * here are taken with interrupts disabled and a CPU spinning on one cannot
 * take the shootdown IPI. */
void     smp_tlb_poll(void);
int32_t  smp_get_current_pid(void);
void     smp_set_current_pid(int32_t pid);
