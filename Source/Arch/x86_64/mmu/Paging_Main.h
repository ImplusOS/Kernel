#pragma once
#ifndef PAGING_MAIN_H
#define PAGING_MAIN_H

#include <stdint.h>
#include <stddef.h>
#include "kernel/config.h"

#define PAGE_PRESENT (1ULL << 0)
#define PAGE_RW      (1ULL << 1)
#define PAGE_USER    (1ULL << 2)
#define PAGE_PWT     (1ULL << 3)
#define PAGE_PCD     (1ULL << 4)
#define PAGE_PS      (1ULL << 7)
#define PAGE_PAT     (1ULL << 12)
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

/* The x86_64 kernel is linked at 0x01800000, while fixed-address static
 * Linux ET_EXEC binaries such as BusyBox use 0x00400000. Reserve this small
 * identity-map window before kernel heap initialization so per-process
 * mappings can shadow it without aliasing any kernel allocation. */
#define PAGING_LEGACY_ET_EXEC_BASE  0x00400000ULL
#define PAGING_LEGACY_ET_EXEC_SIZE  0x00400000ULL
#define PAGING_LEGACY_ET_EXEC_LIMIT (PAGING_LEGACY_ET_EXEC_BASE + PAGING_LEGACY_ET_EXEC_SIZE)

/* The 2 MiB below it: LLVM lld links a non-PIE x86_64 executable at
 * 0x00200000 by default, and Firefox's launcher stub (Resource/firefox, the
 * program that execs firefox-bin) is one. Without a reserved window such an
 * image could only borrow the identity frames when they happened to be free
 * -- the first launch after boot worked, a later one failed with "paging map
 * failed (load segment)" once the allocator had handed those frames out.
 * Withheld at boot like the window above, but only when every frame of it is
 * still free RAM (the firmware or the boot manager may hold some), so it is
 * shadowable only once kernel_main has called
 * paging_low_et_exec_window_set_available(1). */
#define PAGING_LOW_ET_EXEC_BASE  0x00200000ULL
#define PAGING_LOW_ET_EXEC_SIZE  0x00200000ULL
#define PAGING_LOW_ET_EXEC_LIMIT (PAGING_LOW_ET_EXEC_BASE + PAGING_LOW_ET_EXEC_SIZE)
extern volatile int g_paging_low_et_exec_ready;
void paging_low_et_exec_window_set_available(int available);

/* Windows' KUSER_SHARED_DATA lives at the fixed address 0x7FFE0000 in every
 * process, and Wine's ntdll exits ("failed to map the shared user data")
 * unless it can map it exactly there. That is inside the low identity map,
 * so it gets the same treatment as the ET_EXEC window: the physical frames
 * are withheld before heap init and each process may shadow the range with
 * private pages. The whole 0x7FFE0000..0x80000000 tail is covered (Wine also
 * places its hypervisor shared page right after the user shared data). */
#define PAGING_WIN_SHARED_DATA_BASE  0x7FFE0000ULL
#define PAGING_WIN_SHARED_DATA_SIZE  0x00020000ULL
#define PAGING_WIN_SHARED_DATA_LIMIT (PAGING_WIN_SHARED_DATA_BASE + PAGING_WIN_SHARED_DATA_SIZE)

/* The Linux i386 window (OS_CONFIG_LINUX_I386_WINDOW_* in kernel/config.h):
 * shadowable only once kernel_main has withheld its frames and called
 * paging_compat32_window_set_available(1). */
extern volatile int g_paging_compat32_window_ready;
int paging_compat32_window_available(void);
void paging_compat32_window_set_available(int available);
/* Remove the window's identity entries from `cr3` (an i386 process's
 * address space), leaving a hole the process fills with private pages,
 * demand-zero (paging_handle_swap_fault). User pages already mapped there
 * are kept. 0 on success. */
int paging_prepare_compat32_window(uint64_t cr3);

/* Low identity-mapped ranges whose physical frames are never handed to the
 * kernel allocator, so a process may map private pages over them. */
static inline int paging_is_user_shadow_window(uint64_t va)
{
    return (va >= PAGING_LEGACY_ET_EXEC_BASE && va < PAGING_LEGACY_ET_EXEC_LIMIT) ||
           (g_paging_low_et_exec_ready &&
            va >= PAGING_LOW_ET_EXEC_BASE && va < PAGING_LOW_ET_EXEC_LIMIT) ||
           (va >= PAGING_WIN_SHARED_DATA_BASE && va < PAGING_WIN_SHARED_DATA_LIMIT) ||
           (g_paging_compat32_window_ready &&
            va >= OS_CONFIG_LINUX_I386_WINDOW_BASE &&
            va < OS_CONFIG_LINUX_I386_WINDOW_BASE + OS_CONFIG_LINUX_I386_WINDOW_SIZE);
}

/* May a process place a fixed-address mapping at [addr, addr + len)? On
 * x86_64 the low identity map is policed per page by the leaf mapper (only
 * free identity frames and the shadow windows can be taken). */
static inline int paging_user_fixed_range_allowed(uint64_t addr, uint64_t len)
{
    (void)addr;
    (void)len;
    return 1;
}

#define PML4_INDEX(x) (((x) >> 39) & 0x1FF)
#define PDPT_INDEX(x) (((x) >> 30) & 0x1FF)
#define PD_INDEX(x)   (((x) >> 21) & 0x1FF)
#define PT_INDEX(x)   (((x) >> 12) & 0x1FF)

void init_paging(void);
void *map_mmio_virt(uint64_t phys_addr);
void *map_mmio_wc_virt(uint64_t phys_addr);
/* Program this CPU's PAT for the WC/UC page-table flag convention used by
 * map_mmio_virt()/map_mmio_wc_virt(): WC pages carry PWT=1,PCD=0,PAT=1
 * (PAT entry 5) and UC carries PCD=1,PWT=1 (power-on entry 3). Runs on the
 * BSP from init_paging() and must be repeated on every AP in ap_entry_c(),
 * because IA32_PAT_MSR is per-CPU while the kernel page tables are shared.
 * Idempotent; also prints the PAT value over serial. */
void paging_setup_pat(void);
uint64_t paging_get_kernel_cr3(void);
uint64_t paging_get_active_cr3(void);
void paging_switch_cr3(uint64_t cr3);
uint64_t paging_create_process_space(void);
void paging_destroy_process_space(uint64_t cr3);
/* Whether some CPU may still be running on this address space. Not tracked
 * here yet: always "yes", so a deferred teardown (ProcessManager_Create.c)
 * keeps the space, as reaping did before. */
static inline int paging_space_active_on_any_cpu(uint64_t cr3)
{
    (void)cr3;
    return 1;
}
#define PAGING_TRACKS_ACTIVE_SPACE 0
/* Diagnostic: count resident user pages of an address space. Returns the
 * total; *priv_out gets frames only this address space maps, *ext_out gets
 * PAGE_EXTERNAL (shared/COW) frames it merely participates in. Either output
 * pointer may be NULL. */
uint64_t paging_count_user_pages(uint64_t cr3, uint64_t *priv_out,
                                 uint64_t *ext_out);
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
/* Unshare every copy-on-write page in a user range so the kernel may write
 * into it. See the definition: CR0.WP is clear, so a kernel write would
 * otherwise go through to the frame the parent and child still share. */
int paging_user_range_break_cow(uint64_t cr3, uint64_t start, uint64_t len);
int paging_map_user_page(uint64_t cr3,
                         uint64_t virt_addr,
                         uint64_t phys_addr,
                         uint64_t flags);
/* Map a frame other mappings share, read-only and (when `flags` has
 * PAGE_RW) PAGE_COW. The caller has counted the mapping in the frame's
 * refcount. Note CR0.WP is clear: kernel writes go through unless
 * paging_user_range_break_cow() ran first -- which is why FileMap does not
 * share pages on this architecture yet (see FILEMAP_SHARE_PAGES). */
int paging_map_user_page_cow(uint64_t cr3, uint64_t virt_addr,
                             uint64_t phys_addr, uint64_t flags);
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

/* 1 if a user-mode write to every page of [start, start+len) is allowed by the
 * current tables (absent pages pass: they fault in writable). See the
 * definition for why the kernel has to ask. */
int paging_user_range_is_writable(uint64_t cr3, uint64_t start, uint64_t len);

/* Diagnostics: the leaf PTE for an address, 0 if unmapped. See the .c. */
uint64_t paging_debug_leaf_pte(uint64_t cr3, uint64_t virt_addr);

/* Diagnostics (the out-of-memory report): present user frames the address
 * space owns (not PAGE_EXTERNAL), how many of them fork still shares in
 * *shared_out, and its page-table pages in *tables_out. Approximate. */
/* Diagnostics: pages copied by the copy-on-write fault handler so far. */
uint64_t paging_debug_cow_copies(void);
uint64_t paging_debug_count_user_pages(uint64_t cr3, uint64_t *shared_out,
                                       uint64_t *tables_out);

/* Non-zero when the address falls in a user address-space window. */
int paging_addr_is_user(uint64_t virt_addr);

/* Drop this CPU's TLB entry for the page containing `vaddr`. */
void paging_invalidate_page(uint64_t vaddr);

/* 1 if the page tables already permit the access described by `error_code` at
 * `vaddr` -- i.e. the #PF was spurious (a stale TLB entry on another CPU).
 * See the definition: on SMP the OS has to tolerate these. */
int paging_access_is_now_permitted(uint64_t cr3, uint64_t vaddr,
                                   uint64_t error_code);

#endif
