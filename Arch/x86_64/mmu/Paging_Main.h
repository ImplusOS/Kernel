#pragma once
#ifndef PAGING_MAIN_H
#define PAGING_MAIN_H

#include <stdint.h>
#include <stddef.h>

#define PAGE_PRESENT (1ULL << 0)
#define PAGE_RW      (1ULL << 1)
#define PAGE_USER    (1ULL << 2)
#define PAGE_PWT     (1ULL << 3)
#define PAGE_PCD     (1ULL << 4)
#define PAGE_PS      (1ULL << 7)
#define PAGE_EXTERNAL (1ULL << 10)
/* OS-available PTE bit: marks a copy-on-write page shared with another
 * address space after fork (TODO_Chromium_LinuxABI.md bucket B). Such a
 * page is mapped read-only in every owner; a write fault is serviced by
 * paging_handle_cow_fault(). Bit 9 is PAGE_SWAP, bit 10 is PAGE_EXTERNAL. */
#define PAGE_COW     (1ULL << 11)
#define PAGE_NX      (1ULL << 63)
#define PAGE_SIZE 4096ULL
#define PAGE_MASK 0xFFFFFFFFFFFFF000ULL
#define PAGE_FRAME_MASK 0x000FFFFFFFFFF000ULL

#define PML4_INDEX(x) (((x) >> 39) & 0x1FF)
#define PDPT_INDEX(x) (((x) >> 30) & 0x1FF)
#define PD_INDEX(x)   (((x) >> 21) & 0x1FF)
#define PT_INDEX(x)   (((x) >> 12) & 0x1FF)

void init_paging(void);
void *map_mmio_virt(uint64_t phys_addr);
uint64_t paging_get_kernel_cr3(void);
uint64_t paging_get_active_cr3(void);
void paging_switch_cr3(uint64_t cr3);
uint64_t paging_create_process_space(void);
void paging_destroy_process_space(uint64_t cr3);
int paging_set_user_access(uint64_t cr3, uint64_t start, uint64_t size, int enable_user);
int paging_protect_user_range(uint64_t cr3, uint64_t start, uint64_t size,
                              uint64_t flags);
int paging_unmap_range(uint64_t cr3, uint64_t start, uint64_t size);
int paging_is_user_range_mapped(uint64_t cr3, uint64_t start, uint64_t size);
int paging_copy_present_user_range(uint64_t child_cr3, uint64_t parent_cr3,
                                   uint64_t start, uint64_t end);
/* Copy-on-write clone of [start,end): instead of duplicating every present
 * user page, share the parent's frames read-only into the child, bump their
 * refcount, and downgrade the parent's writable PTEs to read-only + PAGE_COW.
 * PAGE_EXTERNAL (shared-memory / MMIO) pages are still deep-copied. Returns 0
 * on success, <0 on failure (caller should fall back / abort the fork). */
int paging_cow_clone_user_range(uint64_t child_cr3, uint64_t parent_cr3,
                                uint64_t start, uint64_t end);
/* Write-fault handler for PAGE_COW pages. Returns 1 if it resolved the fault
 * (caller resumes the faulting instruction), 0 if the address was not a COW
 * page (caller continues to swap / SIGSEGV handling). */
int paging_handle_cow_fault(uint64_t cr3, uint64_t fault_addr);
int paging_map_user_page(uint64_t cr3,
                         uint64_t virt_addr,
                         uint64_t phys_addr,
                         uint64_t flags);
int paging_map_user_range_alloc(uint64_t cr3,
                                uint64_t start,
                                uint64_t size,
                                uint64_t flags);
void paging_swap_set_enabled(int enable);
int paging_swap_reclaim_one_page(void);
int paging_handle_swap_fault(uint64_t cr3, uint64_t fault_addr);
void *pmm_alloc_pages(size_t num_pages);
void pmm_free_pages(void *virt, size_t num_pages);
uint64_t get_phys_base(void);
uint64_t get_virt_base(void);
uint64_t paging_virt_to_phys(uint64_t cr3, uint64_t virt_addr);

#endif
