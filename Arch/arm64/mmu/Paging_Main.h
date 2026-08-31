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

void init_paging(void);
void *map_mmio_virt(uint64_t phys_addr);
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

#endif
