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
/* Whether some CPU is still running on these tables (its TTBR0). Tracked,
 * so an exiting process's address space can be freed at exit rather than
 * when it is reaped (ProcessManager_Create.c). */
int paging_space_active_on_any_cpu(uint64_t address_space);
#define PAGING_TRACKS_ACTIVE_SPACE 1
int paging_set_user_access(uint64_t address_space, uint64_t start, uint64_t size, int enable_user);
int paging_protect_user_range(uint64_t address_space, uint64_t start, uint64_t size,
                              uint64_t flags);
int paging_unmap_range(uint64_t address_space, uint64_t start, uint64_t size);
int paging_is_user_range_mapped(uint64_t address_space, uint64_t start, uint64_t size);
int paging_map_user_page(uint64_t address_space, uint64_t virt_addr, uint64_t phys_addr, uint64_t flags);
/* Map a frame other mappings share (FileMap's page cache): read-only, and
 * copy-on-write when `flags` asks for PAGE_RW, so the first write gets a
 * private copy (paging_handle_cow_fault). The caller has already counted
 * this mapping in the frame's refcount. User windows only; -1 elsewhere. */
int paging_map_user_page_cow(uint64_t address_space, uint64_t virt_addr,
                             uint64_t phys_addr, uint64_t flags);
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

/* Diagnostics (the out-of-memory report): frames the address space holds of
 * its own -- EL0 leaf pages that are not PAGE_EXTERNAL -- with how many of
 * them a fork still shares (refcount > 1) in *shared_out, and the page-table
 * pages in *tables_out. Unlocked and approximate: a table freed under it is
 * skipped, never followed. */
/* Diagnostics: pages copied by the copy-on-write fault handler so far. */
uint64_t paging_debug_cow_copies(void);
uint64_t paging_debug_count_user_pages(uint64_t address_space,
                                       uint64_t *shared_out,
                                       uint64_t *tables_out);

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

/* Low memory for Windows programs under Wine.
 *
 * Wine (x86_64, translated) puts Windows' KUSER_SHARED_DATA at 0x7FFE0000
 * and its PEB and first TEBs below 2 GiB, and refuses to start without
 * them. Here all of that is the kernel's identity map of RAM (QEMU virt: RAM
 * from 0x40000000). As on x86_64 (see its Paging_Main.h), the frames behind
 * a window there are withheld from the allocator at boot -- when they are
 * free RAM -- and a process may then map private pages over that part of the
 * identity map in its own tables (the 1 GiB block is split per process):
 * 0x70000000..0x80000000 when it can be had, otherwise just the
 * 0x7FFE0000..0x80000000 tail with the user shared data (Wine also places
 * its hypervisor shared page right after it). */
#define PAGING_WIN_LOW_BASE          0x70000000ULL
#define PAGING_WIN_SHARED_DATA_BASE  0x7FFE0000ULL
#define PAGING_WIN_LOW_LIMIT         0x80000000ULL

/* Start of the window in use (0: none). */
extern volatile uint64_t g_paging_win_low_base;
void paging_win_low_window_set(uint64_t base);

/* Identity-mapped ranges a process may shadow with private pages. */
static inline int paging_is_user_shadow_window(uint64_t va)
{
    const uint64_t base = g_paging_win_low_base;
    return base != 0u && va >= base && va < PAGING_WIN_LOW_LIMIT;
}

/* May a process place a fixed-address mapping at [addr, addr + len)?
 * Outside the user windows (code/heap/stack, mmap arena) and the shadow
 * windows the address space is the kernel's identity map, which a process
 * mapping must never replace: the kernel reaches its devices and memory
 * through those very addresses in every address space. */
int paging_user_fixed_range_allowed(uint64_t addr, uint64_t len);

/* The Linux i386 window is an x86_64 thing (32-bit compatibility mode). */
static inline int paging_compat32_window_available(void)
{
    return 0;
}

static inline int paging_prepare_compat32_window(uint64_t address_space)
{
    (void)address_space;
    return -1;
}

#endif
