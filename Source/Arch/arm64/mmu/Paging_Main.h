#pragma once
#ifndef ARM64_PAGING_MAIN_H
#define ARM64_PAGING_MAIN_H

#include <stddef.h>
#include <stdint.h>

#define PAGE_PRESENT  (1ULL << 0)
#define PAGE_RW       (1ULL << 1)
#define PAGE_USER     (1ULL << 2)
#define PAGE_PWT      (1ULL << 3)
#define PAGE_PCD      (1ULL << 4)
#define PAGE_PS       (1ULL << 7)
#define PAGE_EXTERNAL (1ULL << 10)
#define PAGE_NX       (1ULL << 54)
#define PAGE_SIZE     4096ULL
#define PAGE_MASK     0xFFFFFFFFFFFFF000ULL
#define PAGE_FRAME_MASK 0x0000FFFFFFFFF000ULL

void init_paging(void);
void *map_mmio_virt(uint64_t phys_addr);
void *map_mmio_wc_virt(uint64_t phys_addr);
uint64_t paging_get_kernel_cr3(void);
uint64_t paging_get_active_cr3(void);
void paging_switch_cr3(uint64_t address_space);
uint64_t paging_create_process_space(void);
void paging_destroy_process_space(uint64_t address_space);
int paging_set_user_access(uint64_t address_space, uint64_t start, uint64_t size, int enable_user);
int paging_protect_user_range(uint64_t address_space, uint64_t start, uint64_t size,
                              uint64_t flags);
int paging_unmap_range(uint64_t address_space, uint64_t start, uint64_t size);
int paging_is_user_range_mapped(uint64_t address_space, uint64_t start, uint64_t size);
int paging_map_user_page(uint64_t address_space, uint64_t virt_addr, uint64_t phys_addr, uint64_t flags);
int paging_map_user_range_alloc(uint64_t address_space, uint64_t start, uint64_t size, uint64_t flags);
int paging_copy_present_user_range(uint64_t child_ttbr0, uint64_t parent_ttbr0,
                                   uint64_t start, uint64_t end);
int paging_map_kernel_range(uint64_t start, uint64_t size, uint64_t flags);
void paging_swap_set_enabled(int enable);
int paging_swap_reclaim_one_page(void);
int paging_handle_swap_fault(uint64_t address_space, uint64_t fault_addr);
void *pmm_alloc_pages(size_t num_pages);
void pmm_free_pages(void *virt, size_t num_pages);
uint64_t get_phys_base(void);
uint64_t get_virt_base(void);
uint64_t paging_virt_to_phys(uint64_t address_space, uint64_t virt_addr);

/* Copy-on-write fork. The arm64 port does eager copies: the clone reports
 * failure so callers fall back to paging_copy_present_user_range(), and
 * there are never COW pages to break. */
int paging_cow_clone_user_range(uint64_t child_ttbr0, uint64_t parent_ttbr0,
                                uint64_t start, uint64_t end);
int paging_handle_cow_fault(uint64_t address_space, uint64_t fault_addr);
int paging_user_range_break_cow(uint64_t address_space, uint64_t start, uint64_t len);

/* 1 if a user-mode write to every present page of [start, start+len) is
 * permitted by the current tables (absent pages pass). */
int paging_user_range_is_writable(uint64_t address_space, uint64_t start, uint64_t len);

/* Diagnostics: an x86-style synthesized leaf entry (frame | PAGE_PRESENT |
 * PAGE_RW | PAGE_USER | PAGE_NX as applicable), 0 if unmapped. Never sets
 * PAGE_PS: the frame is always the exact 4 KiB frame of virt_addr. */
uint64_t paging_debug_leaf_pte(uint64_t address_space, uint64_t virt_addr);

int paging_addr_is_user(uint64_t virt_addr);
void paging_invalidate_page(uint64_t vaddr);
/* MAIR/TCR/SCTLR and the kernel TTBR the boot CPU runs with, for a
 * secondary CPU to enable its MMU with (Arch/arm64/smp). */
void paging_get_secondary_config(uint64_t *ttbr, uint64_t *tcr,
                                 uint64_t *mair, uint64_t *sctlr);
/* 1 if the current tables already allow the access an x86-style page-fault
 * error code describes (bit1 write, bit2 user, bit4 instruction fetch). */
int paging_access_is_now_permitted(uint64_t address_space, uint64_t vaddr,
                                   uint64_t error_code);

/* x86_64 shadows parts of its low identity map per process (see the x86_64
 * Paging_Main.h); arm64 user space has no such overlap. */
static inline int paging_is_user_shadow_window(uint64_t va)
{
    (void)va;
    return 0;
}

#endif
