#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "Paging_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "kernel/config.h"
#include "Debug/serial/Serial.h"

#include <stddef.h>
#include <stdint.h>

#define GB (1024ULL * 1024ULL * 1024ULL)
#define MB2 (2ULL * 1024ULL * 1024ULL)
#define MAX_PDPT_ENTRIES (((USER_STACK_TOP >> 30) & 0x1FF) + 1)
#define MMIO_WINDOW_BASE 0xFFFF800000000000ULL
#define MMIO_WINDOW_SLOTS 512
#define PAGING_BOOT_IDENTITY_GB 64ULL
#define SWAP_TRACK_MAX 4096
#define PAGE_SWAP (1ULL << 9)

#define PAGE_SIZE_BYTES 4096ULL

typedef struct {
    uint8_t used;
    uint64_t cr3;
    uint64_t *pml4;
    uint64_t *pdpt;
    uint64_t *pd_tables[MAX_PDPT_ENTRIES];
} paging_space_t;

static uint64_t g_kernel_pml4[512] __attribute__((aligned(4096)));
static uint64_t g_kernel_pdpt[512] __attribute__((aligned(4096)));
static uint64_t g_kernel_pd[MAX_PDPT_ENTRIES][512] __attribute__((aligned(4096)));
static uint64_t g_mmio_pdpt[512] __attribute__((aligned(4096)));
static uint64_t g_mmio_pd[512] __attribute__((aligned(4096)));
static uint64_t g_mmio_phys_base[MMIO_WINDOW_SLOTS];
static uint32_t g_mmio_slots_used = 0;
static uint64_t g_kernel_identity_entries = PAGING_BOOT_IDENTITY_GB;
static paging_space_t g_process_spaces_static[OS_CONFIG_PROCESS_MAX_COUNT];
static paging_space_t *g_process_spaces = g_process_spaces_static;
static uint32_t g_process_spaces_capacity = OS_CONFIG_PROCESS_MAX_COUNT;
static spinlock_t g_paging_space_lock;

typedef struct {
    uint8_t  used;
    uint8_t  reserved[7];
    void    *phys_page;
} swap_slot_t;

typedef struct {
    uint8_t used;
    uint8_t swapped;
    uint16_t reserved0;
    uint32_t slot_index;
    uint64_t cr3;
    uint64_t virt_addr;
} swap_track_t;

static swap_track_t g_swap_tracks[SWAP_TRACK_MAX];
static uint8_t g_swap_enabled = 1;
static spinlock_t g_swap_lock;

#define SWAP_SLOT_COUNT 128
static swap_slot_t g_swap_slots[SWAP_SLOT_COUNT];

#include "interfaces/hal_cpu.h"

static inline uint64_t read_cr3(void)
{
    return hal_cpu_read_cr(3);
}

static inline void write_cr3(uint64_t value)
{
    hal_cpu_write_cr(3, value);
}

static inline void enable_paging(void)
{
    uint64_t cr4 = hal_cpu_read_cr(4);
    cr4 |= (1ULL << 5);
    hal_cpu_write_cr(4, cr4);

    uint64_t cr0 = hal_cpu_read_cr(0);
    cr0 |= (1ULL << 31);
    hal_cpu_write_cr(0, cr0);
}

static inline void invlpg_addr(uint64_t addr)
{
    hal_mmu_invalidate_tlb(addr);
}

#include "smp/SMP_Main.h"


static void copy_page_entries(uint64_t *dst, const uint64_t *src)
{
    for (uint32_t i = 0; i < 512; ++i) {
        dst[i] = src[i];
    }
}

static void copy_page_bytes(uint8_t *dst, const uint8_t *src)
{
    for (uint64_t i = 0; i < PAGE_SIZE_BYTES; ++i) {
        dst[i] = src[i];
    }
}

/*
 * Serialises structural changes to a page table: installing a PDPT/PD/PT,
 * freeing one, splitting a huge page, and writing leaf entries.
 *
 * Every demand fault ends up here, and the two fault paths that lead to it --
 * filemap_handle_fault() and the demand-zero branch of
 * paging_handle_swap_fault() -- hold different locks from each other, so they
 * were free to walk and edit the same tables at the same time. Two CPUs
 * finding the same PML4 slot empty would each allocate a PDPT and each store
 * it; the loser's table, and every mapping already installed through it, just
 * disappeared. What the process saw afterwards was memory that had quietly
 * reverted to demand-zero -- pointers reading back as NULL, structures full of
 * zeroes -- which is the shape of the corruption Chromium died of under
 * `-smp >1`.
 *
 * Taken with interrupts off, and never while holding it is anything else
 * acquired, so it cannot deadlock against the fault locks above it.
 */
#ifndef PAGING_LOST_MAPPING_TRACE
#define PAGING_LOST_MAPPING_TRACE 0
#endif

static spinlock_t g_page_table_lock;

static uint64_t *alloc_zeroed_page_table(void)
{
    uint64_t *table = (uint64_t *)alloc_page();
    if (table == NULL) {
        return NULL;
    }
    for (uint32_t i = 0; i < 512; ++i) {
        table[i] = 0;
    }
    return table;
}

static paging_space_t *find_space_by_cr3(uint64_t cr3)
{
    for (uint32_t i = 0; i < g_process_spaces_capacity; ++i) {
        if (g_process_spaces[i].used && g_process_spaces[i].cr3 == cr3) {
            return &g_process_spaces[i];
        }
    }
    return NULL;
}

static paging_space_t *find_free_space_slot(void)
{
    for (uint32_t i = 0; i < g_process_spaces_capacity; ++i) {
        if (!g_process_spaces[i].used) {
            return &g_process_spaces[i];
        }
    }
    return NULL;
}

static int ensure_kernel_pdpt_entry(uint64_t pdpt_index)
{
    if (pdpt_index >= MAX_PDPT_ENTRIES) {
        return -1;
    }

    if ((g_kernel_pdpt[pdpt_index] & PAGE_PRESENT) != 0) {
        return 0;
    }

    uint64_t base = pdpt_index * GB;
    for (uint64_t j = 0; j < 512; ++j) {
        g_kernel_pd[pdpt_index][j] = (base + (j * MB2)) | PAGE_PRESENT | PAGE_RW | PAGE_PS;
    }
    g_kernel_pdpt[pdpt_index] = ((uint64_t)g_kernel_pd[pdpt_index]) | PAGE_PRESENT | PAGE_RW;

    if (pdpt_index + 1 > g_kernel_identity_entries) {
        g_kernel_identity_entries = pdpt_index + 1;
    }
    return 0;
}

static int pd_table_has_user_pages(const uint64_t *pd_table)
{
    for (uint32_t i = 0; i < 512; ++i) {
        if ((pd_table[i] & PAGE_PRESENT) != 0 &&
            (pd_table[i] & PAGE_USER) != 0) {
            return 1;
        }
    }
    return 0;
}



/*
 * The page directory that maps `virt_addr` in `cr3`, read out of the live
 * tables.
 *
 * This replaces a version that indexed a per-address-space cache
 * (paging_space_t::pd_tables) with bits 30-38 of the address alone, which
 * quietly folded every PML4 slot onto one 512-entry array. That was survivable while every user mapping lived below
 * 512 GiB, and stopped being survivable when the mmap arena moved to
 * USER_MMAP_BASE (1 TiB, PML4 slot 2): an arena address resolved to the page
 * directory of an unrelated address in PML4 slot 0. munmap() in the arena then
 * walked that directory instead -- freeing the frames the executable had
 * mapped there, and finally clearing the PD entry outright, which is why 2 MiB
 * of Chromium's image would disappear and fault back in as demand-zero pages.
 * PartitionAlloc unmaps from the arena continuously, so the damage tracked how
 * far the browser got rather than anything about the addresses involved.
 */
static uint64_t *walk_pd_table(uint64_t cr3, uint64_t virt_addr)
{
    if (cr3 == 0) {
        return NULL;
    }
    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    uint64_t e4 = pml4[PML4_INDEX(virt_addr)];
    if ((e4 & PAGE_PRESENT) == 0) {
        return NULL;
    }
    uint64_t *pdpt = (uint64_t *)(uintptr_t)(e4 & PAGE_FRAME_MASK);
    uint64_t e3 = pdpt[PDPT_INDEX(virt_addr)];
    if ((e3 & PAGE_PRESENT) == 0 || (e3 & PAGE_PS) != 0) {
        return NULL;
    }
    return (uint64_t *)(uintptr_t)(e3 & PAGE_FRAME_MASK);
}

/* Rewrite the PDPT entry covering `virt_addr` to point at `pd_table`, with the
 * user bit set only if the directory still has user pages under it. The
 * Addressed by virtual address rather than by a bits-30-38 index, for the
 * reason spelled out on walk_pd_table(). */
static int refresh_pdpt_entry(uint64_t cr3, uint64_t virt_addr,
                              uint64_t *pd_table)
{
    if (cr3 == 0 || pd_table == NULL) {
        return -1;
    }
    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    uint64_t e4 = pml4[PML4_INDEX(virt_addr)];
    if ((e4 & PAGE_PRESENT) == 0) {
        return -1;
    }
    uint64_t *pdpt = (uint64_t *)(uintptr_t)(e4 & PAGE_FRAME_MASK);
    uint64_t flags = PAGE_PRESENT | PAGE_RW;
    if (pd_table_has_user_pages(pd_table)) {
        flags |= PAGE_USER;
    }
    pdpt[PDPT_INDEX(virt_addr)] =
        ((uint64_t)(uintptr_t)pd_table) | flags;
    return 0;
}

static int is_kernel_table(const void *table)
{
    if (table == (const void *)g_kernel_pdpt) {
        return 1;
    }
    for (uint32_t i = 0; i < MAX_PDPT_ENTRIES; ++i) {
        if (table == (const void *)g_kernel_pd[i]) {
            return 1;
        }
    }
    return 0;
}

static int split_huge_page(uint64_t *pd, uint64_t pd_index)
{
    uint64_t pde = pd[pd_index];

    if ((pde & PAGE_PRESENT) == 0 || (pde & PAGE_PS) == 0) {
        return 0;
    }

    uint64_t *pt = alloc_zeroed_page_table();
    if (pt == NULL) {
        return -1;
    }

    uint64_t phys_base = pde & PAGE_FRAME_MASK & ~(MB2 - 1ULL);

    uint64_t inherit = pde & (PAGE_RW | PAGE_USER |
                              (1ULL << 3) |
                              (1ULL << 4) |
                              (1ULL << 8));

    for (uint64_t i = 0; i < 512; ++i) {
        pt[i] = (phys_base + i * PAGE_SIZE_BYTES) | PAGE_PRESENT | inherit;
    }

    pd[pd_index] = ((uint64_t)pt) | PAGE_PRESENT | PAGE_RW |
                   (pde & PAGE_USER);

    return 0;
}

static int resolve_user_pte_slot(uint64_t cr3,
                                 uint64_t virt_addr,
                                 uint64_t **pte_out)
{
    if (cr3 == 0 || pte_out == NULL) {
        return -1;
    }

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    uint64_t pml4e = pml4[PML4_INDEX(virt_addr)];
    if ((pml4e & PAGE_PRESENT) == 0 || (pml4e & PAGE_USER) == 0) {
        return -1;
    }

    uint64_t *pdpt = (uint64_t *)(uintptr_t)(pml4e & PAGE_FRAME_MASK);
    uint64_t pdpte = pdpt[PDPT_INDEX(virt_addr)];
    if ((pdpte & PAGE_PRESENT) == 0 || (pdpte & PAGE_USER) == 0) {
        return -1;
    }

    uint64_t *pd = (uint64_t *)(uintptr_t)(pdpte & PAGE_FRAME_MASK);
    uint64_t pde = pd[PD_INDEX(virt_addr)];
    if ((pde & PAGE_PRESENT) == 0) {
        return -1;
    }

    if ((pde & PAGE_USER) == 0) {
        return -1;
    }

    if ((pde & PAGE_PS) != 0) {
        *pte_out = &pd[PD_INDEX(virt_addr)];
        return 0;
    }

    uint64_t *pt = (uint64_t *)(uintptr_t)(pde & PAGE_FRAME_MASK);
    *pte_out = &pt[PT_INDEX(virt_addr)];
    return 0;
}

static int is_user_virtual_address(uint64_t virt_addr)
{
    return ((virt_addr >= USER_CODE_BASE && virt_addr < USER_CODE_LIMIT) ||
            (virt_addr >= USER_HEAP_BASE && virt_addr < USER_HEAP_LIMIT) ||
            (virt_addr >= USER_STACK_BASE && virt_addr < USER_STACK_TOP) ||
            (virt_addr >= USER_MMAP_BASE  && virt_addr < USER_MMAP_LIMIT));
}

static int copy_present_page(uint64_t child_cr3, uint64_t vaddr,
                             uint64_t parent_phys)
{
    void *child_page = pmm_alloc_pages(1);
    if (!child_page) return -1;
    memcpy(child_page, (void *)(uintptr_t)parent_phys, PAGE_SIZE);
    int ret = paging_map_user_page(child_cr3, vaddr,
                                    (uint64_t)(uintptr_t)child_page,
                                    PAGE_PRESENT | PAGE_RW | PAGE_USER);
    if (ret < 0) {
        pmm_free_pages(child_page, 1);
    }
    return ret;
}

/* Copy the USER pages present in [start, end) from one address space to
 * another (fork).
 *
 * Every level is filtered on PAGE_USER, not just PAGE_PRESENT. A process's
 * page tables also carry the kernel's own mappings, and the kernel
 * identity-maps PAGING_BOOT_IDENTITY_GB (64 GB) of physical memory at low
 * virtual addresses -- squarely inside the range fork walks. Without the
 * filter this function walked into those kernel tables and copied physical
 * memory into the child until the machine ran out: a measured fork spent
 * 345 s and died with 94 MB free, and the cost tracked installed RAM rather
 * than anything about the process being forked.
 * (paging_destroy_process_space() has always applied the same filter.) */
int paging_copy_present_user_range(uint64_t child_cr3, uint64_t parent_cr3,
                                   uint64_t start, uint64_t end)
{
    if (child_cr3 == 0 || parent_cr3 == 0 || end <= start) {
        return -1;
    }

    uint64_t first = start & PAGE_MASK;
    uint64_t last = (end - 1) & PAGE_MASK;

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(parent_cr3 & PAGE_FRAME_MASK);
    uint64_t pml4_first = PML4_INDEX(first);
    uint64_t pml4_last = PML4_INDEX(last);

    for (uint64_t i = pml4_first; i <= pml4_last; ++i) {
        uint64_t pml4e = pml4[i];
        if ((pml4e & (PAGE_PRESENT | PAGE_USER)) != (PAGE_PRESENT | PAGE_USER)) continue;

        uint64_t *pdpt = (uint64_t *)(uintptr_t)(pml4e & PAGE_FRAME_MASK);
        uint64_t pdpt_first = (i == pml4_first) ? PDPT_INDEX(first) : 0;
        uint64_t pdpt_last = (i == pml4_last) ? PDPT_INDEX(last) : 511;

        for (uint64_t j = pdpt_first; j <= pdpt_last; ++j) {
            uint64_t pdpte = pdpt[j];
            if ((pdpte & (PAGE_PRESENT | PAGE_USER)) != (PAGE_PRESENT | PAGE_USER)) continue;

            uint64_t *pd = (uint64_t *)(uintptr_t)(pdpte & PAGE_FRAME_MASK);
            uint64_t pd_first = (i == pml4_first && j == pdpt_first)
                                    ? PD_INDEX(first) : 0;
            uint64_t pd_last = (i == pml4_last && j == pdpt_last)
                                    ? PD_INDEX(last) : 511;

            for (uint64_t k = pd_first; k <= pd_last; ++k) {
                uint64_t pde = pd[k];
                if ((pde & (PAGE_PRESENT | PAGE_USER)) != (PAGE_PRESENT | PAGE_USER)) continue;

                uint64_t region_base =
                    (i << 39) | (j << 30) | (k << 21);

                if ((pde & PAGE_PS) != 0) {
                    for (uint64_t off = 0; off < 0x200000; off += PAGE_SIZE) {
                        uint64_t v = region_base + off;
                        if (v < first || v > last) continue;
                        if (copy_present_page(child_cr3, v,
                                                   (pde & PAGE_FRAME_MASK) + off) < 0) {
                            return -1;
                        }
                    }
                    continue;
                }

                uint64_t *pt = (uint64_t *)(uintptr_t)(pde & PAGE_FRAME_MASK);
                uint64_t pt_first = (i == pml4_first && j == pdpt_first &&
                                     k == pd_first) ? PT_INDEX(first) : 0;
                uint64_t pt_last = (i == pml4_last && j == pdpt_last &&
                                    k == pd_last) ? PT_INDEX(last) : 511;

                for (uint64_t l = pt_first; l <= pt_last; ++l) {
                    uint64_t pte = pt[l];
                    if ((pte & (PAGE_PRESENT | PAGE_USER)) != (PAGE_PRESENT | PAGE_USER)) continue;
                    uint64_t v = region_base + (l << 12);
                    if (v < first || v > last) continue;
                    if (copy_present_page(child_cr3, v,
                                               pte & PAGE_FRAME_MASK) < 0) {
                        return -1;
                    }
                }
            }
        }
    }

    return 0;
}

/* --- Copy-on-write fork support (TODO_Chromium_LinuxABI.md bucket B) --- */

static int resolve_fault_leaf_entry(uint64_t cr3, uint64_t virt_addr,
                                    uint64_t **pml4e_out, uint64_t **pdpte_out,
                                    uint64_t **pde_out, uint64_t **entry_out);

static int cow_share_one_page(uint64_t child_cr3, uint64_t v,
                              uint64_t *parent_pte)
{
    uint64_t pte = *parent_pte;
    uint64_t phys = pte & PAGE_FRAME_MASK;

    /* Externally-owned frames (shared memory / MMIO mapped into user space)
     * have their own lifecycle - never fold them into COW, deep-copy as the
     * old path did. */
    if ((pte & PAGE_EXTERNAL) != 0u) {
        return copy_present_page(child_cr3, v, phys);
    }

    int was_writable = (pte & PAGE_RW) != 0u;
    uint64_t child_flags = PAGE_PRESENT | PAGE_USER | (pte & PAGE_NX);
    if (was_writable) {
        child_flags |= PAGE_COW; /* read-only in the child until it writes */
    }

    /* Account for the extra mapping BEFORE publishing it, so a concurrent
     * free on the parent side can never drop the frame while the child
     * references it. */
    pmm_page_ref_inc(phys);

    if (paging_map_user_page(child_cr3, v, phys, child_flags) < 0) {
        /* undo the ref we just took */
        (void)pmm_page_ref_dec(phys);
        return -1;
    }

    if (was_writable) {
        /* Downgrade the parent's mapping to read-only + COW so its next
         * write also traps into paging_handle_cow_fault(). */
        *parent_pte = (pte & ~PAGE_RW) | PAGE_COW;
    }
    return 0;
}

int paging_cow_clone_user_range(uint64_t child_cr3, uint64_t parent_cr3,
                                uint64_t start, uint64_t end)
{
    if (child_cr3 == 0 || parent_cr3 == 0 || end <= start) {
        return -1;
    }

    uint64_t first = start & PAGE_MASK;
    uint64_t last = (end - 1) & PAGE_MASK;

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(parent_cr3 & PAGE_FRAME_MASK);
    uint64_t pml4_first = PML4_INDEX(first);
    uint64_t pml4_last = PML4_INDEX(last);

    for (uint64_t i = pml4_first; i <= pml4_last; ++i) {
        uint64_t pml4e = pml4[i];
        if ((pml4e & (PAGE_PRESENT | PAGE_USER)) != (PAGE_PRESENT | PAGE_USER)) continue;
        uint64_t *pdpt = (uint64_t *)(uintptr_t)(pml4e & PAGE_FRAME_MASK);
        uint64_t pdpt_first = (i == pml4_first) ? PDPT_INDEX(first) : 0;
        uint64_t pdpt_last = (i == pml4_last) ? PDPT_INDEX(last) : 511;

        for (uint64_t j = pdpt_first; j <= pdpt_last; ++j) {
            uint64_t pdpte = pdpt[j];
            if ((pdpte & (PAGE_PRESENT | PAGE_USER)) != (PAGE_PRESENT | PAGE_USER)) continue;
            uint64_t *pd = (uint64_t *)(uintptr_t)(pdpte & PAGE_FRAME_MASK);
            uint64_t pd_first = (i == pml4_first && j == pdpt_first)
                                    ? PD_INDEX(first) : 0;
            uint64_t pd_last = (i == pml4_last && j == pdpt_last)
                                    ? PD_INDEX(last) : 511;

            for (uint64_t k = pd_first; k <= pd_last; ++k) {
                uint64_t pde = pd[k];
                if ((pde & (PAGE_PRESENT | PAGE_USER)) != (PAGE_PRESENT | PAGE_USER)) continue;
                uint64_t region_base = (i << 39) | (j << 30) | (k << 21);

                if ((pde & PAGE_PS) != 0) {
                    /* 2 MiB huge page: fall back to per-4K deep copy (rare in
                     * user space here, and splitting it for COW is not worth
                     * the complexity). */
                    for (uint64_t off = 0; off < 0x200000; off += PAGE_SIZE) {
                        uint64_t v = region_base + off;
                        if (v < first || v > last) continue;
                        if (copy_present_page(child_cr3, v,
                                              (pde & PAGE_FRAME_MASK) + off) < 0) {
                            return -1;
                        }
                    }
                    continue;
                }

                uint64_t *pt = (uint64_t *)(uintptr_t)(pde & PAGE_FRAME_MASK);
                uint64_t pt_first = (i == pml4_first && j == pdpt_first &&
                                     k == pd_first) ? PT_INDEX(first) : 0;
                uint64_t pt_last = (i == pml4_last && j == pdpt_last &&
                                    k == pd_last) ? PT_INDEX(last) : 511;

                for (uint64_t l = pt_first; l <= pt_last; ++l) {
                    if ((pt[l] & PAGE_PRESENT) == 0) continue;
                    uint64_t v = region_base + (l << 12);
                    if (v < first || v > last) continue;
                    if (cow_share_one_page(child_cr3, v, &pt[l]) < 0) {
                        return -1;
                    }
                }
            }
        }
    }

    /* The parent may be running threads on other CPUs with stale writable
     * TLB entries for the pages just downgraded - flush every CPU that is on
     * the parent's address space, and wait for them. */
    smp_tlb_shootdown_cr3(parent_cr3, 0u, 0u);
    return 0;
}

int paging_handle_cow_fault(uint64_t cr3, uint64_t fault_addr)
{
    if (cr3 == 0) return 0;

    uint64_t v = fault_addr & PAGE_MASK;
    if (!is_user_virtual_address(v)) {
        return 0;
    }

    uint64_t *pml4e = NULL, *pdpte = NULL, *pde = NULL, *pte = NULL;
    if (resolve_fault_leaf_entry(cr3, v, &pml4e, &pdpte, &pde, &pte) < 0 ||
        pte == NULL) {
        return 0;
    }
    uint64_t entry = *pte;
    if ((entry & PAGE_PRESENT) == 0 || (entry & PAGE_COW) == 0 ||
        (entry & PAGE_USER) == 0 || (entry & PAGE_RW) != 0) {
        return 0; /* not a writable-intent COW page we downgraded */
    }

    uint64_t old_phys = entry & PAGE_FRAME_MASK;
    uint64_t keep_flags = entry & (PAGE_USER | PAGE_NX | PAGE_PWT | PAGE_PCD);

    if (pmm_page_ref_get(old_phys) <= 1u) {
        /* Sole remaining owner: just take write access back, no copy. */
        *pte = (old_phys | keep_flags | PAGE_PRESENT | PAGE_RW);
        if (cr3 == read_cr3()) invlpg_addr(v);
        return 1;
    }

    void *fresh = alloc_page();
    if (fresh == NULL) {
        return 0; /* OOM: let it become a fault/SIGSEGV */
    }
    memcpy(fresh, (const void *)(uintptr_t)old_phys, PAGE_SIZE_BYTES);
    *pte = (((uint64_t)(uintptr_t)fresh) & PAGE_FRAME_MASK) |
           keep_flags | PAGE_PRESENT | PAGE_RW;
    if (cr3 == read_cr3()) invlpg_addr(v);

    /* Drop this address space's reference to the old shared frame. free_page()
     * consults the refcount: it retains the frame for the other owner(s) and
     * only releases it when the last reference goes away. */
    free_page((void *)(uintptr_t)old_phys);
    return 1;
}

static int resolve_fault_leaf_entry(uint64_t cr3,
                                    uint64_t virt_addr,
                                    uint64_t **pml4e_out,
                                    uint64_t **pdpte_out,
                                    uint64_t **pde_out,
                                    uint64_t **entry_out)
{
    if (cr3 == 0 || pml4e_out == NULL || pdpte_out == NULL ||
        pde_out == NULL || entry_out == NULL) {
        return -1;
    }

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    uint64_t *pml4e = &pml4[PML4_INDEX(virt_addr)];
    if ((*pml4e & PAGE_PRESENT) == 0) {
        return -1;
    }

    uint64_t *pdpt = (uint64_t *)(uintptr_t)(*pml4e & PAGE_FRAME_MASK);
    uint64_t *pdpte = &pdpt[PDPT_INDEX(virt_addr)];
    if ((*pdpte & PAGE_PRESENT) == 0) {
        return -1;
    }

    uint64_t *pd = (uint64_t *)(uintptr_t)(*pdpte & PAGE_FRAME_MASK);
    uint64_t *pde = &pd[PD_INDEX(virt_addr)];
    if ((*pde & PAGE_PRESENT) == 0) {
        return -1;
    }

    uint64_t *entry = NULL;
    if ((*pde & PAGE_PS) != 0) {
        entry = pde;
    } else {
        uint64_t *pt = (uint64_t *)(uintptr_t)(*pde & PAGE_FRAME_MASK);
        entry = &pt[PT_INDEX(virt_addr)];
    }

    *pml4e_out = pml4e;
    *pdpte_out = pdpte;
    *pde_out = pde;
    *entry_out = entry;
    return 0;
}

static int swap_alloc_slot(void)
{
    spinlock_lock(&g_swap_lock);
    for (uint32_t i = 0; i < SWAP_SLOT_COUNT; ++i) {
        if (!g_swap_slots[i].used) {
            void *page = alloc_page();
            if (page == NULL) {
                spinlock_unlock(&g_swap_lock);
                return -1;
            }
            g_swap_slots[i].used       = 1;
            g_swap_slots[i].phys_page  = page;
            spinlock_unlock(&g_swap_lock);
            return (int)i;
        }
    }
    spinlock_unlock(&g_swap_lock);
    return -1;
}

static void swap_free_slot(uint32_t slot)
{
    if (slot >= SWAP_SLOT_COUNT) {
        return;
    }
    spinlock_lock(&g_swap_lock);
    if (g_swap_slots[slot].used && g_swap_slots[slot].phys_page != NULL) {
        free_page(g_swap_slots[slot].phys_page);
        g_swap_slots[slot].phys_page = NULL;
    }
    g_swap_slots[slot].used = 0;
    spinlock_unlock(&g_swap_lock);
}

static swap_track_t *swap_find_track(uint64_t cr3, uint64_t virt_addr)
{
    for (uint32_t i = 0; i < SWAP_TRACK_MAX; ++i) {
        if (g_swap_tracks[i].used &&
            g_swap_tracks[i].cr3 == cr3 &&
            g_swap_tracks[i].virt_addr == virt_addr) {
            return &g_swap_tracks[i];
        }
    }
    return NULL;
}

static void swap_forget_track(uint64_t cr3, uint64_t virt_addr)
{
    swap_track_t *track = swap_find_track(cr3, virt_addr);
    if (track == NULL) {
        return;
    }
    if (track->swapped) {
        swap_free_slot(track->slot_index);
    }
    track->used = 0;
    track->swapped = 0;
    track->slot_index = 0;
    track->cr3 = 0;
    track->virt_addr = 0;
}

static void swap_track_page(uint64_t cr3, uint64_t virt_addr)
{
    virt_addr &= PAGE_MASK;
    swap_track_t *existing = swap_find_track(cr3, virt_addr);
    if (existing != NULL) {
        if (existing->swapped) {
            swap_free_slot(existing->slot_index);
            existing->swapped = 0;
            existing->slot_index = 0;
        }
        existing->used = 1;
        return;
    }

    for (uint32_t i = 0; i < SWAP_TRACK_MAX; ++i) {
        if (!g_swap_tracks[i].used) {
            g_swap_tracks[i].used = 1;
            g_swap_tracks[i].swapped = 0;
            g_swap_tracks[i].slot_index = 0;
            g_swap_tracks[i].cr3 = cr3;
            g_swap_tracks[i].virt_addr = virt_addr;
            return;
        }
    }
}

static void paging_identity_mark_2mb_uncached(uint64_t phys_addr)
{
    uint64_t base = phys_addr & ~(MB2 - 1ULL);
    uint64_t pdpt_index = base / GB;
    uint64_t pd_index = (base - pdpt_index * GB) / MB2;

    if (pdpt_index >= MAX_PDPT_ENTRIES) {
        return;
    }
    if (ensure_kernel_pdpt_entry(pdpt_index) < 0) {
        return;
    }
    g_kernel_pd[pdpt_index][pd_index] |= PAGE_PCD | PAGE_PWT;
    invlpg_addr(base);
}

void *map_mmio_virt(uint64_t phys_addr)
{
    if ((phys_addr & ~(PAGE_FRAME_MASK | (PAGE_SIZE_BYTES - 1ULL))) != 0u) {
        return NULL;
    }

    if (phys_addr < (PAGING_BOOT_IDENTITY_GB * GB)) {
        paging_identity_mark_2mb_uncached(phys_addr);
        return (void *)(uintptr_t)phys_addr;
    }

    uint64_t phys_base = phys_addr & ~(MB2 - 1ULL);
    uint64_t offset = phys_addr - phys_base;

    for (uint32_t i = 0; i < g_mmio_slots_used; ++i) {
        if (g_mmio_phys_base[i] == phys_base) {
            uint64_t virt_base = MMIO_WINDOW_BASE + ((uint64_t)i * MB2);
            return (void *)(uintptr_t)(virt_base + offset);
        }
    }

    if (g_mmio_slots_used >= MMIO_WINDOW_SLOTS) {
        return NULL;
    }

    uint64_t virt_base = MMIO_WINDOW_BASE + ((uint64_t)g_mmio_slots_used * MB2);
    uint64_t pdpt_index = (virt_base >> 30) & 0x1FFULL;
    uint64_t pd_index = (virt_base >> 21) & 0x1FFULL;

    if (PML4_INDEX(virt_base) != PML4_INDEX(MMIO_WINDOW_BASE) ||
        pdpt_index != PDPT_INDEX(MMIO_WINDOW_BASE)) {
        return NULL;
    }

    if ((g_kernel_pml4[PML4_INDEX(MMIO_WINDOW_BASE)] & PAGE_PRESENT) == 0) {
        g_kernel_pml4[PML4_INDEX(MMIO_WINDOW_BASE)] =
            ((uint64_t)g_mmio_pdpt) | PAGE_PRESENT | PAGE_RW;
    }
    if ((g_mmio_pdpt[pdpt_index] & PAGE_PRESENT) == 0) {
        g_mmio_pdpt[pdpt_index] = ((uint64_t)g_mmio_pd) | PAGE_PRESENT | PAGE_RW;
    }

    g_mmio_pd[pd_index] = phys_base | PAGE_PRESENT | PAGE_RW | PAGE_PS | PAGE_PCD | PAGE_PWT;
    
    invlpg_addr(virt_base);

    g_mmio_phys_base[g_mmio_slots_used] = phys_base;
    g_mmio_slots_used++;

    return (void *)(uintptr_t)(virt_base + offset);
}

void init_paging(void)
{
    uint32_t msr = 0xC0000080;
    uint64_t efer = hal_cpu_read_msr(msr);
    efer |= (1 << 11); 
    hal_cpu_write_msr(msr, efer);

    memset(g_kernel_pml4, 0, sizeof(g_kernel_pml4));
    memset(g_kernel_pdpt, 0, sizeof(g_kernel_pdpt));
    memset(g_kernel_pd, 0, sizeof(g_kernel_pd));
    memset(g_mmio_pdpt, 0, sizeof(g_mmio_pdpt));
    memset(g_mmio_pd, 0, sizeof(g_mmio_pd));
    memset(g_mmio_phys_base, 0, sizeof(g_mmio_phys_base));
    memset(g_swap_slots, 0, sizeof(g_swap_slots));
    memset(g_swap_tracks, 0, sizeof(g_swap_tracks));
    spinlock_init(&g_paging_space_lock);
    spinlock_init(&g_page_table_lock);
    spinlock_init(&g_swap_lock);

    for (uint32_t i = 0; i < g_process_spaces_capacity; ++i) {
        g_process_spaces[i].used = 0;
        g_process_spaces[i].cr3  = 0;
        g_process_spaces[i].pml4 = NULL;
        g_process_spaces[i].pdpt = NULL;
        for (uint32_t j = 0; j < MAX_PDPT_ENTRIES; ++j) {
            g_process_spaces[i].pd_tables[j] = NULL;
        }
    }
    g_mmio_slots_used = 0;
    g_kernel_identity_entries = PAGING_BOOT_IDENTITY_GB;
    uint64_t total_pages = get_total_memory_pages();
    if (total_pages > 0) {
        uint64_t total_bytes = total_pages * PAGE_SIZE_BYTES;
        uint64_t required_entries = (total_bytes + GB - 1ULL) / GB;
        if (required_entries > g_kernel_identity_entries) {
            g_kernel_identity_entries = required_entries;
        }
    }
    if (g_kernel_identity_entries > 256) {
        g_kernel_identity_entries = 256;
    }

    g_kernel_pml4[0] = ((uint64_t)g_kernel_pdpt) | PAGE_PRESENT | PAGE_RW;
    g_kernel_pml4[PML4_INDEX(MMIO_WINDOW_BASE)] =
        ((uint64_t)g_mmio_pdpt) | PAGE_PRESENT | PAGE_RW;
    g_mmio_pdpt[PDPT_INDEX(MMIO_WINDOW_BASE)] =
        ((uint64_t)g_mmio_pd) | PAGE_PRESENT | PAGE_RW;

    for (uint64_t i = 0; i < g_kernel_identity_entries; ++i) {
        if (ensure_kernel_pdpt_entry(i) < 0) {
            break;
        }
    }

    write_cr3((uint64_t)g_kernel_pml4);
}

uint64_t paging_get_kernel_cr3(void)
{
    return (uint64_t)g_kernel_pml4;
}

uint64_t paging_get_active_cr3(void)
{
    return read_cr3();
}

/*
 * 1 if the page tables, as they stand right now, already allow the access that
 * just faulted -- in other words the fault was spurious.
 *
 * This is required on SMP, not a nicety. When one CPU makes a page present (or
 * widens its permissions), the other CPUs may still hold a stale TLB entry for
 * that address and fault on it; x86 permits that and expects the OS to notice
 * and resume. Chromium is where it shows: dozens of threads share one address
 * space and hammer the same freshly-mapped PartitionAlloc slots, so what looks
 * like a wild pointer is often a translation from a moment ago. Terminating on
 * one of these is why the browser died only under -smp >1.
 *
 * `error_code` is the #PF error code: bit0 present, bit1 write, bit2 user,
 * bit4 instruction fetch.
 */
/*
 * 1 if a user-mode write to every page of [start, start+len) would be allowed
 * by the current tables.
 *
 * Pages that are not present pass: they fault in writable through the
 * demand-zero path, so refusing them would break every write into a
 * lazily-committed buffer. Present pages must carry PAGE_RW | PAGE_USER.
 *
 * The kernel needs this because it can write through a read-only user mapping
 * without trapping, and Linux does not: read(2) into a PROT_READ page returns
 * EFAULT. Chromium checks exactly that -- base/memory/protected_memory_posix.cc
 * reads /dev/zero into memory it has just mprotect()ed read-only and CHECKs
 * that the call fails with EFAULT -- and takes the process down when it does
 * not.
 */
int paging_user_range_is_writable(uint64_t cr3, uint64_t start, uint64_t len)
{
    if (cr3 == 0 || len == 0) {
        return 1;
    }
    uint64_t first = start & PAGE_MASK;
    uint64_t last  = (start + len - 1ULL) & PAGE_MASK;

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    for (uint64_t page = first; ; page += PAGE_SIZE_BYTES) {
        uint64_t e = pml4[PML4_INDEX(page)];
        if ((e & PAGE_PRESENT) != 0) {
            uint64_t *pdpt = (uint64_t *)(uintptr_t)(e & PAGE_FRAME_MASK);
            e = pdpt[PDPT_INDEX(page)];
            if ((e & PAGE_PRESENT) != 0 && (e & PAGE_PS) == 0) {
                uint64_t *pd = (uint64_t *)(uintptr_t)(e & PAGE_FRAME_MASK);
                e = pd[PD_INDEX(page)];
                if ((e & PAGE_PRESENT) != 0 && (e & PAGE_PS) == 0) {
                    uint64_t *pt = (uint64_t *)(uintptr_t)(e & PAGE_FRAME_MASK);
                    uint64_t pte = pt[PT_INDEX(page)];
                    /* Only the leaf, and only its write bit. The upper levels
                     * carry a user bit this kernel installs lazily (the fault
                     * handlers repair it on the way past), so folding those in
                     * would reject writes that are perfectly legal. */
                    if ((pte & PAGE_PRESENT) != 0 && (pte & PAGE_RW) == 0) {
                        return 0;
                    }
                }
            }
        }
        if (page == last) {
            break;
        }
    }
    return 1;
}

/* Drop this CPU's TLB entry for one page. */
void paging_invalidate_page(uint64_t vaddr)
{
    invlpg_addr(vaddr & PAGE_MASK);
}

int paging_access_is_now_permitted(uint64_t cr3, uint64_t vaddr,
                                   uint64_t error_code)
{
    if (cr3 == 0) {
        return 0;
    }

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    uint64_t entry = pml4[PML4_INDEX(vaddr)];
    if ((entry & PAGE_PRESENT) == 0) return 0;
    uint64_t path = entry;

    uint64_t *pdpt = (uint64_t *)(uintptr_t)(entry & PAGE_FRAME_MASK);
    entry = pdpt[PDPT_INDEX(vaddr)];
    if ((entry & PAGE_PRESENT) == 0) return 0;
    path &= entry;

    if ((entry & PAGE_PS) == 0) {
        uint64_t *pd = (uint64_t *)(uintptr_t)(entry & PAGE_FRAME_MASK);
        entry = pd[PD_INDEX(vaddr)];
        if ((entry & PAGE_PRESENT) == 0) return 0;
        path &= entry;

        if ((entry & PAGE_PS) == 0) {
            uint64_t *pt = (uint64_t *)(uintptr_t)(entry & PAGE_FRAME_MASK);
            entry = pt[PT_INDEX(vaddr)];
            if ((entry & PAGE_PRESENT) == 0) return 0;
            path &= entry;
        }
    }

    /* Permission bits are the AND down the walk, except NX which is the OR. */
    if ((error_code & 0x2ULL) != 0u && (path & PAGE_RW) == 0) return 0;
    if ((error_code & 0x4ULL) != 0u && (path & PAGE_USER) == 0) return 0;
    if ((error_code & 0x10ULL) != 0u && (entry & PAGE_NX) != 0) return 0;
    /* A copy-on-write page still owes the write a private copy; that is the COW
     * handler's job, not a stale translation. */
    if ((error_code & 0x2ULL) != 0u && (entry & PAGE_COW) != 0) return 0;

    return 1;
}

uint64_t paging_virt_to_phys(uint64_t cr3, uint64_t virt_addr)
{
    if (cr3 == 0) {
        return 0;
    }

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    uint64_t pml4e = pml4[PML4_INDEX(virt_addr)];
    if ((pml4e & PAGE_PRESENT) == 0) {
        return 0;
    }

    uint64_t *pdpt = (uint64_t *)(uintptr_t)(pml4e & PAGE_FRAME_MASK);
    uint64_t pdpte = pdpt[PDPT_INDEX(virt_addr)];
    if ((pdpte & PAGE_PRESENT) == 0) {
        return 0;
    }
    
    if ((pdpte & PAGE_PS) != 0) {
        uint64_t phys_base = pdpte & PAGE_FRAME_MASK & ~(GB - 1ULL);
        return phys_base | (virt_addr & (GB - 1ULL));
    }

    uint64_t *pd = (uint64_t *)(uintptr_t)(pdpte & PAGE_FRAME_MASK);
    uint64_t pde = pd[PD_INDEX(virt_addr)];
    if ((pde & PAGE_PRESENT) == 0) {
        return 0;
    }

    if ((pde & PAGE_PS) != 0) {
        uint64_t phys_base = pde & PAGE_FRAME_MASK & ~(MB2 - 1ULL);
        return phys_base | (virt_addr & (MB2 - 1ULL));
    }

    uint64_t *pt = (uint64_t *)(uintptr_t)(pde & PAGE_FRAME_MASK);
    uint64_t pte = pt[PT_INDEX(virt_addr)];
    if ((pte & PAGE_PRESENT) == 0) {
        return 0;
    }

    return (pte & PAGE_FRAME_MASK) | (virt_addr & (PAGE_SIZE_BYTES - 1ULL));
}

void paging_switch_cr3(uint64_t cr3)
{
    if (cr3 == 0) {
        return;
    }
    write_cr3(cr3);
    /* Tell the shootdown code which address space this CPU is on, so a
     * shootdown for any other one need not wait for it. */
    smp_note_cr3(cr3);
}

uint64_t paging_create_process_space(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_paging_space_lock);
    paging_space_t *space = find_free_space_slot();
    if (space == NULL) {
        spinlock_unlock(&g_paging_space_lock);
        irq_restore(irq_flags);
        return 0;
    }
    memset(space, 0, sizeof(*space));

    space->used = 1;
    space->cr3 = UINT64_MAX;
    paging_space_t *space_ptr = space;
    spinlock_unlock(&g_paging_space_lock);
    irq_restore(irq_flags);

    space_ptr->pml4 = alloc_zeroed_page_table();
    if (!space_ptr->pml4) {
        uint64_t err_irq = irq_save_disable();
        spinlock_lock(&g_paging_space_lock);
        space_ptr->used = 0;
        space_ptr->cr3  = 0;
        spinlock_unlock(&g_paging_space_lock);
        irq_restore(err_irq);
        return 0;
    }

    copy_page_entries(space_ptr->pml4, g_kernel_pml4);

    space_ptr->pdpt = alloc_zeroed_page_table(); if (!space_ptr->pdpt) { uint64_t err_irq = irq_save_disable(); spinlock_lock(&g_paging_space_lock); free_page(space_ptr->pml4); space_ptr->used = 0; space_ptr->cr3 = 0; spinlock_unlock(&g_paging_space_lock); irq_restore(err_irq); return 0; } copy_page_entries(space_ptr->pdpt, g_kernel_pdpt);
    space_ptr->pml4[0] = ((uint64_t)(uintptr_t)space_ptr->pdpt) | PAGE_PRESENT | PAGE_RW | PAGE_USER;

    for (uint64_t i = 0; i < MAX_PDPT_ENTRIES; ++i) {
        space_ptr->pd_tables[i] = NULL;
    }

    uint64_t irq_flags_final = irq_save_disable();
    spinlock_lock(&g_paging_space_lock);
    space_ptr->cr3 = (uint64_t)space_ptr->pml4;
    spinlock_unlock(&g_paging_space_lock);
    irq_restore(irq_flags_final);
    return space_ptr->cr3;
}

void paging_destroy_process_space(uint64_t cr3)
{
    if (cr3 == 0 || cr3 == (uint64_t)g_kernel_pml4) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_paging_space_lock);
    paging_space_t *space = find_space_by_cr3(cr3);
    if (space == NULL) {
        spinlock_unlock(&g_paging_space_lock);
        irq_restore(irq_flags);
        return;
    }
    paging_space_t space_copy = *space;
    space->used = 0;
    space->cr3 = 0;
    spinlock_unlock(&g_paging_space_lock);
    irq_restore(irq_flags);

    if (read_cr3() == cr3) {
        write_cr3((uint64_t)g_kernel_pml4);
        smp_note_cr3((uint64_t)g_kernel_pml4);
    }

    /* The page tables below are about to be freed. Anyone still on this
     * address space has to have dropped its translations first. */
    smp_tlb_shootdown_cr3(cr3, 0u, 0u);

    for (uint64_t i = 0; i < MAX_PDPT_ENTRIES; ++i) {
        if (space_copy.pd_tables[i] == NULL) {
            continue;
        }

        for (uint64_t j = 0; j < 512; ++j) {
            uint64_t pde = space_copy.pd_tables[i][j];
            if ((pde & PAGE_PRESENT) == 0 || (pde & PAGE_PS) != 0) {
                continue;
            }

            uint64_t *pt = (uint64_t *)(uintptr_t)(pde & PAGE_FRAME_MASK);
            for (uint64_t k = 0; k < 512; ++k) {
                uint64_t pte = pt[k];
                if ((pte & PAGE_USER) == 0) {
                    continue;
                }
                uint64_t virt_addr = (i << 30) | (j << 21) | (k << 12);
                if ((pte & PAGE_PRESENT) != 0) {
                    if ((pte & PAGE_EXTERNAL) == 0) {
                        free_page((void *)(uintptr_t)(pte & PAGE_FRAME_MASK));
                    }
                } else if ((pte & PAGE_SWAP) != 0) {
                    uint32_t slot = (uint32_t)((pte & PAGE_FRAME_MASK) >> 12);
                    swap_free_slot(slot);
                }
                swap_forget_track(cr3, virt_addr);
                pt[k] = 0;
            }
            free_page(pt);
        }

        free_page(space_copy.pd_tables[i]);
    }
    if (space_copy.pdpt != NULL && space_copy.pdpt != g_kernel_pdpt) {
        free_page(space_copy.pdpt);
    }
    if (space_copy.pml4 != NULL) {
        free_page(space_copy.pml4);
    }

    for (uint32_t i = 0; i < SWAP_TRACK_MAX; ++i) {
        if (g_swap_tracks[i].used && g_swap_tracks[i].cr3 == cr3) {
            swap_forget_track(cr3, g_swap_tracks[i].virt_addr);
        }
    }
}

int paging_set_user_access(uint64_t cr3,
                           uint64_t start,
                           uint64_t size,
                           int enable_user)
{
    if (cr3 == 0 || size == 0)
        return -1;

    uint64_t end = start + size;
    if (end <= start)
        return -1;

    uint64_t aligned_start = start & ~(PAGE_SIZE_BYTES - 1ULL);
    uint64_t aligned_end   = (end + PAGE_SIZE_BYTES - 1ULL) & ~(PAGE_SIZE_BYTES - 1ULL);
    if (aligned_end < end) return -1;

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);

    for (uint64_t addr = aligned_start; addr < aligned_end; addr += PAGE_SIZE_BYTES)
    {
        uint64_t pml4_index = (addr >> 39) & 0x1FFULL;
        uint64_t pdpt_index = (addr >> 30) & 0x1FFULL;
        uint64_t pd_index   = (addr >> 21) & 0x1FFULL;
        uint64_t pt_index   = (addr >> 12) & 0x1FFULL;

        if ((pml4[pml4_index] & PAGE_PRESENT) == 0) {
            return -1;
        }
        if (enable_user) {
            pml4[pml4_index] |= PAGE_USER;
        }

        uint64_t *pd_table = walk_pd_table(cr3, addr);
        if (pd_table == NULL) {
            if (cr3 == (uint64_t)g_kernel_pml4) {
                if (ensure_kernel_pdpt_entry(pdpt_index) < 0) {
                    return -1;
                }
                pd_table = g_kernel_pd[pdpt_index];
            } else {
                uint64_t irq_f = irq_save_disable();
                spinlock_lock(&g_paging_space_lock);
                paging_space_t *space = find_space_by_cr3(cr3);
                if (space == NULL) {
                    spinlock_unlock(&g_paging_space_lock);
                    irq_restore(irq_f);
                    return -1;
                }
                pd_table = alloc_zeroed_page_table();
                if (pd_table == NULL) {
                    spinlock_unlock(&g_paging_space_lock);
                    irq_restore(irq_f);
                    return -1;
                }
                /* Install into the live tables, addressed properly. The
                 * cache below is only meaningful for PML4 slot 0, which is
                 * what paging_destroy_process_space() walks. */
                {
                    uint64_t *live_pml4 =
                        (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
                    uint64_t e4 = live_pml4[PML4_INDEX(addr)];
                    if ((e4 & PAGE_PRESENT) == 0) {
                        free_page(pd_table);
                        spinlock_unlock(&g_paging_space_lock);
                        irq_restore(irq_f);
                        return -1;
                    }
                    uint64_t *live_pdpt =
                        (uint64_t *)(uintptr_t)(e4 & PAGE_FRAME_MASK);
                    uint64_t ent = ((uint64_t)(uintptr_t)pd_table) |
                                   PAGE_PRESENT | PAGE_RW;
                    if (enable_user) {
                        ent |= PAGE_USER;
                    }
                    live_pdpt[PDPT_INDEX(addr)] = ent;
                    if (PML4_INDEX(addr) == 0u &&
                        pdpt_index < MAX_PDPT_ENTRIES) {
                        space->pd_tables[pdpt_index] = pd_table;
                    }
                }
                spinlock_unlock(&g_paging_space_lock);
                irq_restore(irq_f);
            }
        }

        if ((pd_table[pd_index] & PAGE_PRESENT) != 0 &&
            (pd_table[pd_index] & PAGE_PS) != 0) {
            if (split_huge_page(pd_table, pd_index) < 0) {
                return -1;
            }
        }

        if ((pd_table[pd_index] & PAGE_PRESENT) == 0) {
            if (!enable_user) {
                if (refresh_pdpt_entry(cr3, addr, pd_table) < 0) {
                    return -1;
                }
                continue;
            }
            
            uint64_t *pt = alloc_zeroed_page_table();
            if (pt == NULL) {
                return -1;
            }

            void *phys_page = alloc_page();
            if (phys_page == NULL) {
                free_page(pt);
                return -1;
            }
            memset(phys_page, 0, PAGE_SIZE_BYTES);

            pt[pt_index] = ((uint64_t)(uintptr_t)phys_page) |
                           PAGE_PRESENT |
                           PAGE_RW |
                           PAGE_USER;

            pd_table[pd_index] = ((uint64_t)pt) |
                                 PAGE_PRESENT |
                                 PAGE_RW |
                                 PAGE_USER;

        } else {
            uint64_t *pt = (uint64_t *)(uintptr_t)(pd_table[pd_index] & PAGE_FRAME_MASK);
            uint64_t pte = pt[pt_index];

            if ((pte & PAGE_PRESENT) == 0 && (pte & PAGE_SWAP) == 0) {
                if (enable_user) {
                    void *phys_page = alloc_page();
                    if (phys_page == NULL) {
                        return -1;
                    }
                    memset(phys_page, 0, PAGE_SIZE_BYTES);
                    pte = ((uint64_t)(uintptr_t)phys_page) |
                          PAGE_PRESENT |
                          PAGE_RW |
                          PAGE_USER;
                    pt[pt_index] = pte;
                }
            } else {
                if (enable_user) {
                    pte |= PAGE_USER;
                    pte |= PAGE_RW;
                } else {
                    pte &= ~PAGE_USER;
                }
                pt[pt_index] = pte;
            }

            if (enable_user) {
                pd_table[pd_index] |= PAGE_RW;
                pd_table[pd_index] |= PAGE_USER;
            } else {
                int any_user = 0;
                for (uint32_t k = 0; k < 512; ++k) {
                    if (((pt[k] & PAGE_PRESENT) != 0 ||
                         (pt[k] & PAGE_SWAP) != 0) &&
                        (pt[k] & PAGE_USER) != 0) {
                        any_user = 1;
                        break;
                    }
                }
                if (!any_user) {
                    pd_table[pd_index] &= ~PAGE_USER;
                }
            }
        }

        if (refresh_pdpt_entry(cr3, addr, pd_table) < 0) {
            return -1;
        }
    }

    if (read_cr3() == cr3)
        write_cr3(cr3);

    return 0;
}

static int paging_protect_user_range_locked(uint64_t cr3, uint64_t start,
                                           uint64_t size, uint64_t flags,
                                           uint64_t *shoot_start,
                                           uint64_t *shoot_end)
{
    if (cr3 == 0 || size == 0 || (start & (PAGE_SIZE_BYTES - 1ULL)) != 0)
        return -1;
    uint64_t end = (start + size + PAGE_SIZE_BYTES - 1ULL) &
                   ~(PAGE_SIZE_BYTES - 1ULL);
    if (end <= start) return -1;

    /* mprotect() over a lazily-committed anonymous range (USER_MMAP arena, or
     * any not-yet-faulted user page) must not fail just because pages are not
     * resident: the protection request is advisory until the demand-zero
     * fault, which always brings the page in RW. Skip absent pages for user
     * addresses; a genuinely bogus (non-user) address still errors. */
    const int lenient = is_user_virtual_address(start);
    uint64_t narrowed_start = 0, narrowed_end = 0;
    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    for (uint64_t address = start; address < end; address += PAGE_SIZE_BYTES) {
        uint64_t i4 = PML4_INDEX(address);
        uint64_t i3 = PDPT_INDEX(address);
        uint64_t i2 = PD_INDEX(address);
        uint64_t i1 = PT_INDEX(address);
        if ((pml4[i4] & PAGE_PRESENT) == 0) {
            if (!lenient) return -1;
            address = ((address >> 39) << 39) + 0x8000000000ULL - PAGE_SIZE_BYTES;
            continue;
        }
        uint64_t *pdpt = (uint64_t *)(uintptr_t)(pml4[i4] & PAGE_FRAME_MASK);
        if ((pdpt[i3] & PAGE_PRESENT) == 0) {
            if (!lenient) return -1;
            address = ((address >> 30) << 30) + 0x40000000ULL - PAGE_SIZE_BYTES;
            continue;
        }
        uint64_t *pd = (uint64_t *)(uintptr_t)(pdpt[i3] & PAGE_FRAME_MASK);
        if ((pd[i2] & PAGE_PRESENT) == 0) {
            if (!lenient) return -1;
            address = ((address >> 21) << 21) + 0x200000ULL - PAGE_SIZE_BYTES;
            continue;
        }
        if ((pd[i2] & PAGE_PS) != 0 && split_huge_page(pd, i2) < 0) return -1;
        uint64_t *pt = (uint64_t *)(uintptr_t)(pd[i2] & PAGE_FRAME_MASK);
        uint64_t entry = pt[i1];
        if ((entry & PAGE_PRESENT) == 0) { if (lenient) continue; return -1; }
        uint64_t old_entry = entry;
        uint64_t keep_cow = entry & PAGE_COW;
        uint64_t new_flags = flags & (PAGE_RW | PAGE_USER | PAGE_NX);
        /* A still-shared copy-on-write page must stay read-only even when the
         * caller asks for write access: the first write then traps into
         * paging_handle_cow_fault(), which makes the private copy and grants
         * PAGE_RW. Clearing RW here without that would let two address spaces
         * write the same frame. */
        if (keep_cow != 0u) {
            new_flags &= ~PAGE_RW;
        }
        entry &= ~(PAGE_RW | PAGE_USER | PAGE_NX);
        entry |= new_flags | keep_cow;
        pt[i1] = entry;
        if ((flags & PAGE_USER) != 0u) {
            pml4[i4] |= PAGE_USER | PAGE_RW;
            pdpt[i3] |= PAGE_USER | PAGE_RW;
            pd[i2] |= PAGE_USER | PAGE_RW;
        }
        invlpg_addr(address);
        if (entry != old_entry) {
            if (narrowed_start == 0u || address < narrowed_start) {
                narrowed_start = address;
            }
            if (address + PAGE_SIZE_BYTES > narrowed_end) {
                narrowed_end = address + PAGE_SIZE_BYTES;
            }
        }
    }

    *shoot_start = narrowed_start;
    *shoot_end = narrowed_end;
    return 0;
}

int paging_protect_user_range(uint64_t cr3, uint64_t start, uint64_t size,
                              uint64_t flags)
{
    uint64_t shoot_start = 0, shoot_end = 0;
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_page_table_lock);
    int rc = paging_protect_user_range_locked(cr3, start, size, flags,
                                              &shoot_start, &shoot_end);
    spinlock_unlock(&g_page_table_lock);
    irq_restore(irq);

    /* One shootdown for the whole changed span rather than one per page, and
     * outside the lock. A PartitionAlloc decommit mprotects megabytes at a
     * time, and a synchronous IPI round trip per 4 KiB page is enough on its
     * own to stop a browser from making progress. */
    if (shoot_end > shoot_start) {
        uint64_t pages = (shoot_end - shoot_start) / PAGE_SIZE_BYTES;
        /* Past a certain size, telling each CPU to drop everything beats
         * walking it through hundreds of individual invalidations. */
        smp_tlb_shootdown_cr3(cr3, shoot_start, pages > 64u ? 0u : pages);
    }
    return rc;
}

static int paging_unmap_range_locked(uint64_t cr3, uint64_t start, uint64_t size)
{
    if (cr3 == 0 || size == 0) {
        return -1;
    }
#if PAGING_LOST_MAPPING_TRACE
    if (start < 0x4080000000ULL && start + size > 0x4000000000ULL) {
        static volatile uint32_t seen;
        if (__atomic_fetch_add(&seen, 1u, __ATOMIC_RELAXED) < 16u) {
            serial_write_string("[lostmap] unmap_range start=");
            serial_write_uint64(start);
            serial_write_string(" size=");
            serial_write_uint64(size);
            serial_write_string(" cr3=");
            serial_write_uint64(cr3);
            serial_write_string("\n");
        }
    }
#endif

    uint64_t end = start + size;
    if (end <= start) {
        return -1;
    }

    uint64_t aligned_start = start & ~(PAGE_SIZE_BYTES - 1ULL);
    uint64_t aligned_end = (end + PAGE_SIZE_BYTES - 1ULL) & ~(PAGE_SIZE_BYTES - 1ULL);
    if (aligned_end < end) return -1;

    for (uint64_t addr = aligned_start; addr < aligned_end; addr += PAGE_SIZE_BYTES) {
        uint64_t pdpt_index = (addr >> 30) & 0x1FFULL;
        uint64_t pd_index = (addr >> 21) & 0x1FFULL;
        uint64_t pt_index = (addr >> 12) & 0x1FFULL;

        uint64_t *pd_table = walk_pd_table(cr3, addr);
        if (pd_table == NULL) {
            /* Whole 1 GiB not mapped (unmap of a lazily-reserved mmap arena
             * range that was never touched). Jump to the next 1 GiB boundary
             * instead of a million per-page no-ops. */
            addr = ((addr >> 30) << 30) + 0x40000000ULL - PAGE_SIZE_BYTES;
            continue;
        }

        uint64_t pde = pd_table[pd_index];
        if ((pde & PAGE_PRESENT) == 0) {
            addr = ((addr >> 21) << 21) + 0x200000ULL - PAGE_SIZE_BYTES;
            continue;
        }

        if ((pde & PAGE_PS) != 0) {
            if (split_huge_page(pd_table, pd_index) < 0) {
                return -1;
            }
            pde = pd_table[pd_index];
        }

        if ((pde & PAGE_PRESENT) == 0 || (pde & PAGE_PS) != 0) {
            continue;
        }

        uint64_t *pt = (uint64_t *)(uintptr_t)(pde & PAGE_FRAME_MASK);
        uint64_t old_pte = pt[pt_index];
        if ((old_pte & PAGE_SWAP) != 0 && (old_pte & PAGE_PRESENT) == 0) {
            uint32_t slot = (uint32_t)((old_pte & PAGE_FRAME_MASK) >> 12);
            swap_free_slot(slot);
        } else if ((old_pte & PAGE_PRESENT) != 0 && (old_pte & PAGE_USER) != 0) {
            if ((old_pte & PAGE_EXTERNAL) == 0) {
                free_page((void *)(uintptr_t)(old_pte & PAGE_FRAME_MASK));
            }
        }
        swap_forget_track(cr3, addr);
        pt[pt_index] = 0;

        int any_present = 0;
        for (uint64_t i = 0; i < 512; ++i) {
            if ((pt[i] & PAGE_PRESENT) != 0 || (pt[i] & PAGE_SWAP) != 0) {
                any_present = 1;
                break;
            }
        }

        if (!any_present) {
#if PAGING_LOST_MAPPING_TRACE
            /* Clearing a PD entry drops 2 MiB of mappings at once. Record the
             * address that asked for it and the table it landed in: if the
             * table is shared with another address space, this is where the
             * other one's memory disappears. */
            {
                static volatile uint32_t seen;
                if (__atomic_fetch_add(&seen, 1u, __ATOMIC_RELAXED) < 24u) {
                    serial_write_string("[lostmap] clear pde addr=");
                    serial_write_uint64(addr);
                    serial_write_string(" cr3=");
                    serial_write_uint64(cr3);
                    serial_write_string(" pd=");
                    serial_write_uint64((uint64_t)(uintptr_t)pd_table);
                    serial_write_string(" idx=");
                    serial_write_uint64(pd_index);
                    serial_write_string("\n");
                }
            }
#endif
            free_page(pt);
            pd_table[pd_index] = 0;
        }
        if (refresh_pdpt_entry(cr3, addr, pd_table) < 0) {
            return -1;
        }
    }

    return 0;
}

int paging_unmap_range(uint64_t cr3, uint64_t start, uint64_t size)
{
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_page_table_lock);
    int rc = paging_unmap_range_locked(cr3, start, size);
    spinlock_unlock(&g_page_table_lock);
    irq_restore(irq);

    /* Every CPU on this address space, not just this one, and not until they
     * have actually done it: the frames just freed go back to the allocator
     * immediately, so a sibling thread still holding a translation would read
     * and write memory that now belongs to somebody else. Done outside the
     * lock, since the CPUs being waited on may want it themselves. */
    smp_tlb_shootdown_cr3(cr3, 0u, 0u);
    return rc;
}

int paging_is_user_range_mapped(uint64_t cr3, uint64_t start, uint64_t size)
{
    if (size == 0) {
        return 1;
    }
    if (cr3 == 0) {
        return 0;
    }

    uint64_t end = start + size;
    if (end <= start) {
        return 0;
    }

    uint64_t aligned_start = start & ~(PAGE_SIZE_BYTES - 1ULL);
    uint64_t aligned_end = (end + PAGE_SIZE_BYTES - 1ULL) & ~(PAGE_SIZE_BYTES - 1ULL);
    if (aligned_end < end) return 0;

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    uint64_t *pdpt = NULL;
    uint64_t *pd = NULL;
    uint16_t cached_i3 = 0xFFFFu;
    uint16_t cached_i2 = 0xFFFFu;

    for (uint64_t addr = aligned_start; addr < aligned_end; addr += PAGE_SIZE_BYTES) {
        uint64_t pml4e = pml4[PML4_INDEX(addr)];
        if ((pml4e & PAGE_PRESENT) == 0 || (pml4e & PAGE_USER) == 0) {
            return 0;
        }

        uint16_t i3 = PDPT_INDEX(addr);
        if (pdpt == NULL || i3 != cached_i3) {
            pdpt = (uint64_t *)(uintptr_t)(pml4e & PAGE_FRAME_MASK);
            cached_i3 = i3;
            pd = NULL;
            cached_i2 = 0xFFFFu;
        }
        uint64_t pdpte = pdpt[i3];
        if ((pdpte & PAGE_PRESENT) == 0 || (pdpte & PAGE_USER) == 0) {
            return 0;
        }

        uint16_t i2 = PD_INDEX(addr);
        if (pd == NULL || i2 != cached_i2) {
            pd = (uint64_t *)(uintptr_t)(pdpte & PAGE_FRAME_MASK);
            cached_i2 = i2;
        }
        uint64_t pde = pd[i2];
        if ((pde & PAGE_PRESENT) == 0 || (pde & PAGE_USER) == 0) {
            return 0;
        }

        if ((pde & PAGE_PS) != 0) {
            continue;
        }

        uint64_t *pt = (uint64_t *)(uintptr_t)(pde & PAGE_FRAME_MASK);
        uint64_t pte = pt[PT_INDEX(addr)];
        if ((pte & PAGE_PRESENT) == 0) {
            if (((pte & PAGE_SWAP) == 0) || ((pte & PAGE_USER) == 0)) {
                return 0;
            }
        } else if ((pte & PAGE_USER) == 0) {
            return 0;
        }
    }

    return 1;
}

void *pmm_alloc_pages(size_t num_pages)
{
    return alloc_contiguous_pages(num_pages, 1);
}

static int paging_map_user_page_locked(uint64_t cr3,
                                      uint64_t virt_addr,
                                      uint64_t phys_addr,
                                      uint64_t flags,
                                      int *replaced_out)
{
    if (cr3 == 0) {
        return -1;
    }
    *replaced_out = 0;

    uint64_t saved_cr3 = read_cr3();
    uint64_t kernel_cr3 = paging_get_kernel_cr3();

    if (saved_cr3 != kernel_cr3) {
        write_cr3(kernel_cr3);
    }

    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    uint16_t i4 = PML4_INDEX(virt_addr);
    uint16_t i3 = PDPT_INDEX(virt_addr);
    uint16_t i2 = PD_INDEX(virt_addr);
    uint16_t i1 = PT_INDEX(virt_addr);

    uint64_t *pdpt = NULL;
    uint64_t *pd   = NULL;
    uint64_t *pt   = NULL;
    paging_space_t *space = find_space_by_cr3(cr3);

    if ((pml4[i4] & PAGE_PRESENT) == 0) {
        pdpt = alloc_zeroed_page_table();
        if (pdpt == NULL) {
            if (saved_cr3 != kernel_cr3) write_cr3(saved_cr3);
            return -1;
        }
        pml4[i4] = ((uint64_t)pdpt) | PAGE_PRESENT | PAGE_RW | PAGE_USER;
        if (space != NULL) {
            space->pdpt = pdpt;
        }
    } else {
        pdpt = (uint64_t *)(uintptr_t)(pml4[i4] & PAGE_FRAME_MASK);

        if (is_kernel_table(pdpt)) {
            uint64_t *cloned = alloc_zeroed_page_table();
            if (cloned == NULL) {
                if (saved_cr3 != kernel_cr3) write_cr3(saved_cr3);
                return -1;
            }
            copy_page_entries(cloned, pdpt);
            pdpt = cloned;
            pml4[i4] = ((uint64_t)pdpt) | PAGE_PRESENT | PAGE_RW | PAGE_USER;

            if (space != NULL) {
                space->pdpt = cloned;
            }
        } else {
            pml4[i4] |= PAGE_USER;
        }
    }

    if ((pdpt[i3] & PAGE_PRESENT) == 0) {
        pd = alloc_zeroed_page_table();
        if (pd == NULL) {
            if (saved_cr3 != kernel_cr3) write_cr3(saved_cr3);
            return -1;
        }
        pdpt[i3] = ((uint64_t)pd) | PAGE_PRESENT | PAGE_RW | PAGE_USER;
    } else {
        pd = (uint64_t *)(uintptr_t)(pdpt[i3] & PAGE_FRAME_MASK);

        if (is_kernel_table(pd)) {
            uint64_t *cloned = alloc_zeroed_page_table();
            if (cloned == NULL) {
                if (saved_cr3 != kernel_cr3) write_cr3(saved_cr3);
                return -1;
            }
            copy_page_entries(cloned, pd);
            pd = cloned;
            pdpt[i3] = ((uint64_t)pd) | PAGE_PRESENT | PAGE_RW | PAGE_USER;

            if (space != NULL && i3 < MAX_PDPT_ENTRIES) {
                space->pd_tables[i3] = cloned;
            }
        } else {
            pdpt[i3] |= PAGE_USER;
        }
    }
    if (space != NULL && i3 < MAX_PDPT_ENTRIES) {
        space->pd_tables[i3] = pd;
    }

    if ((pd[i2] & PAGE_PRESENT) != 0 && (pd[i2] & PAGE_PS) != 0) {
        if (split_huge_page(pd, i2) < 0) {
            if (saved_cr3 != kernel_cr3) write_cr3(saved_cr3);
            return -1;
        }
        pd[i2] |= PAGE_USER;
    }

    if ((pd[i2] & PAGE_PRESENT) == 0) {
        pt = alloc_zeroed_page_table();
        if (pt == NULL) {
            if (saved_cr3 != kernel_cr3) write_cr3(saved_cr3);
            return -1;
        }
        pd[i2] = ((uint64_t)pt) | PAGE_PRESENT | PAGE_RW | PAGE_USER;
    } else {
        pd[i2] |= PAGE_USER;
        pt = (uint64_t *)(uintptr_t)(pd[i2] & PAGE_FRAME_MASK);
    }

    uint64_t old_pte = pt[i1];
    /* Replacing a live translation, not filling a hole: every other CPU may
     * still hold the old one, and the frame under it is freed just below and
     * handed straight back to the allocator. A thread on another CPU then
     * keeps reading (and writing) a page that now belongs to something else --
     * which is how a fully populated GL dispatch table read back as a NULL
     * entry in one thread while the page tables said otherwise. */
    int replaced_live_mapping = (old_pte & PAGE_PRESENT) != 0;
    if ((old_pte & PAGE_PRESENT) != 0 && (old_pte & PAGE_USER) != 0) {
        if ((old_pte & PAGE_EXTERNAL) == 0) {
            free_page((void *)(uintptr_t)(old_pte & PAGE_FRAME_MASK));
        }
    } else if ((old_pte & PAGE_SWAP) != 0 && (old_pte & PAGE_USER) != 0) {
        uint32_t slot = (uint32_t)((old_pte & PAGE_FRAME_MASK) >> 12);
        swap_free_slot(slot);
    }

    pt[i1] = (phys_addr & PAGE_FRAME_MASK) |
             PAGE_PRESENT |
             PAGE_USER |
             (flags & (PAGE_RW | PAGE_PWT | PAGE_PCD | PAGE_EXTERNAL | PAGE_NX |
                       PAGE_COW));
    swap_track_page(cr3, virt_addr);

    if (saved_cr3 != kernel_cr3) {
        write_cr3(saved_cr3);
    }
    
    if (cr3 == read_cr3()) {
        invlpg_addr(virt_addr & PAGE_MASK);
    }
    *replaced_out = replaced_live_mapping;
    return 0;
}

/* 1 if every level above the leaf already exists and belongs to this address
 * space, so mapping the page is a single store into an existing table.
 * `out_pt` receives the leaf table. */
static int paging_leaf_table_ready(uint64_t cr3, uint64_t virt_addr,
                                   uint64_t **out_pt)
{
    uint64_t *pml4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
    /* PAGE_RW as well as PAGE_USER at every level: the CPU ANDs the write bit
     * down the walk, so a leaf marked writable under a read-only directory
     * still faults -- forever, since nothing in the fast path repairs it. */
    uint64_t e4 = pml4[PML4_INDEX(virt_addr)];
    if ((e4 & (PAGE_PRESENT | PAGE_USER | PAGE_RW)) !=
        (PAGE_PRESENT | PAGE_USER | PAGE_RW)) return 0;
    uint64_t *pdpt = (uint64_t *)(uintptr_t)(e4 & PAGE_FRAME_MASK);
    if (is_kernel_table(pdpt)) return 0;

    uint64_t e3 = pdpt[PDPT_INDEX(virt_addr)];
    if ((e3 & PAGE_PS) != 0 ||
        (e3 & (PAGE_PRESENT | PAGE_USER | PAGE_RW)) !=
        (PAGE_PRESENT | PAGE_USER | PAGE_RW)) return 0;
    uint64_t *pd = (uint64_t *)(uintptr_t)(e3 & PAGE_FRAME_MASK);
    if (is_kernel_table(pd)) return 0;

    uint64_t e2 = pd[PD_INDEX(virt_addr)];
    if ((e2 & PAGE_PS) != 0 ||
        (e2 & (PAGE_PRESENT | PAGE_USER | PAGE_RW)) !=
        (PAGE_PRESENT | PAGE_USER | PAGE_RW)) return 0;
    uint64_t *pt = (uint64_t *)(uintptr_t)(e2 & PAGE_FRAME_MASK);
    if (is_kernel_table(pt)) return 0;

    *out_pt = pt;
    return 1;
}

int paging_map_user_page(uint64_t cr3,
                         uint64_t virt_addr,
                         uint64_t phys_addr,
                         uint64_t flags)
{
    if (cr3 == 0) {
        return -1;
    }

    /* Fast path: nothing structural to change, so the whole operation is one
     * aligned 64-bit store into a table that already exists and cannot be
     * replaced under us. Taking the global page-table lock here instead put
     * every demand fault in the system through one lock with interrupts off,
     * and Chromium faults from four CPUs continuously -- it cost roughly five
     * times the throughput. Concurrent maps of the *same* page are still
     * settled by the callers, which re-check presence under their own lock
     * (filemap_handle_fault(), the demand-zero branch of
     * paging_handle_swap_fault()). */
#ifndef PAGING_LEAF_FASTPATH
#define PAGING_LEAF_FASTPATH 1
#endif
#if PAGING_LEAF_FASTPATH
    {
        uint64_t *pt = NULL;
        if (paging_leaf_table_ready(cr3, virt_addr, &pt)) {
            uint64_t i1 = PT_INDEX(virt_addr);
            uint64_t old = pt[i1];
            if ((old & PAGE_PRESENT) == 0) {
                pt[i1] = (phys_addr & PAGE_FRAME_MASK) | PAGE_PRESENT |
                         PAGE_USER |
                         (flags & (PAGE_RW | PAGE_PWT | PAGE_PCD |
                                   PAGE_EXTERNAL | PAGE_NX | PAGE_COW));
                swap_track_page(cr3, virt_addr);
                if (cr3 == read_cr3()) {
                    invlpg_addr(virt_addr & PAGE_MASK);
                }
                return 0;
            }
            /* Replacing a live mapping frees a frame: that needs the lock and
             * the shootdown, so fall through to the slow path. */
        }
    }
#endif

    int replaced = 0;
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_page_table_lock);
    int rc = paging_map_user_page_locked(cr3, virt_addr, phys_addr, flags,
                                         &replaced);
    spinlock_unlock(&g_page_table_lock);
    irq_restore(irq);

    /* Outside the lock: the shootdown waits for other CPUs, and one of them
     * may be waiting for this very lock to service its own fault. */
    if (rc == 0 && replaced) {
        /* The frame that was here may go straight back to the allocator, so
         * the other CPUs running this address space have to drop it before we
         * return -- but only this page, not their whole TLB. */
        smp_tlb_shootdown_cr3(cr3, virt_addr & PAGE_MASK, 1ULL);
    }
    return rc;
}

int paging_map_user_range_alloc(uint64_t cr3,
                                uint64_t start,
                                uint64_t size,
                                uint64_t flags)
{
    if (cr3 == 0 || size == 0) {
        return -1;
    }

    uint64_t end = start + size;
    if (end <= start) {
        return -1;
    }

    uint64_t aligned_start = start & ~(PAGE_SIZE_BYTES - 1ULL);
    uint64_t aligned_end = (end + PAGE_SIZE_BYTES - 1ULL) & ~(PAGE_SIZE_BYTES - 1ULL);
    if (aligned_end < end) return -1;

    for (uint64_t addr = aligned_start; addr < aligned_end; addr += PAGE_SIZE_BYTES) {
        void *phys_page = alloc_page();
        if (phys_page == NULL) {
            return -1;
        }

        memset(phys_page, 0, PAGE_SIZE_BYTES);

        if (paging_map_user_page(cr3,
                                 addr,
                                 (uint64_t)(uintptr_t)phys_page,
                                 flags) < 0) {
            free_page(phys_page);
            return -1;
        }
    }

    return 0;
}

void paging_swap_set_enabled(int enable)
{
    g_swap_enabled = (enable != 0) ? 1u : 0u;
}

int paging_swap_reclaim_one_page(void)
{
    return 0;
}

/* Demand faulting is not per-address-space serialised anywhere else, and two
 * threads of one process (llvmpipe's worker pool, say) fault on the same page
 * routinely. Without this lock both allocate a zeroed page and both map it:
 * the second map overwrites the first one's PTE, so every write the first
 * thread already made lands in a page nothing points at any more, and the
 * other CPUs keep a TLB entry for it. That is how a fully populated glvnd GL
 * dispatch table read back as a NULL entry and Doom jumped to address 0. */
static spinlock_t g_demand_fault_lock;

int paging_handle_swap_fault(uint64_t cr3, uint64_t fault_addr)
{
    if (cr3 == 0) return 0;

    uint64_t virt_addr = fault_addr & PAGE_MASK;

    if (!is_user_virtual_address(virt_addr)) {
        return 0;
    }

    uint64_t *pml4e = NULL, *pdpte = NULL, *pde = NULL, *pte = NULL;
    int leaf_res = resolve_fault_leaf_entry(cr3, virt_addr, &pml4e, &pdpte, &pde, &pte);

    if (leaf_res < 0 || pte == NULL || ((*pte & PAGE_PRESENT) == 0 && (*pte & PAGE_SWAP) == 0)) {
        uint64_t dz_irq = irq_save_disable();
        spinlock_lock(&g_demand_fault_lock);

        /* Re-walk under the lock: another CPU may have filled this page in
         * while we were on our way here, in which case there is nothing to do
         * but drop our own stale translation. */
        pml4e = pdpte = pde = pte = NULL;
        leaf_res = resolve_fault_leaf_entry(cr3, virt_addr, &pml4e, &pdpte,
                                            &pde, &pte);
        if (leaf_res >= 0 && pte != NULL &&
            ((*pte & PAGE_PRESENT) != 0 || (*pte & PAGE_SWAP) != 0)) {
            spinlock_unlock(&g_demand_fault_lock);
            irq_restore(dz_irq);
            if (cr3 == read_cr3()) {
                invlpg_addr(virt_addr);
            }
            return 1;
        }

#if PAGING_LOST_MAPPING_TRACE
        /* The main executable's PT_LOADs are mapped eagerly by the ELF loader,
         * so nothing in the code window should ever reach demand-zero. If it
         * does, a mapping that existed has gone missing -- which is the
         * signature of the SMP corruption being chased. Rate limited. */
        if (virt_addr >= 0x4000000000ULL && virt_addr < 0x4080000000ULL) {
            static volatile uint32_t lost_reported;
            if (__atomic_fetch_add(&lost_reported, 1u, __ATOMIC_RELAXED) < 16u) {
                /* Walk it by hand: whether the leaf table is gone or merely
                 * has a hole says whether a page table was reused or a single
                 * entry was cleared. */
                uint64_t *l4 = (uint64_t *)(uintptr_t)(cr3 & PAGE_FRAME_MASK);
                uint64_t e4 = l4[PML4_INDEX(virt_addr)];
                uint64_t e3 = 0, e2 = 0;
                uint32_t present_in_pt = 0xFFFFFFFFu;
                if ((e4 & PAGE_PRESENT) != 0) {
                    uint64_t *l3 = (uint64_t *)(uintptr_t)(e4 & PAGE_FRAME_MASK);
                    e3 = l3[PDPT_INDEX(virt_addr)];
                    if ((e3 & PAGE_PRESENT) != 0 && (e3 & PAGE_PS) == 0) {
                        uint64_t *l2 = (uint64_t *)(uintptr_t)(e3 & PAGE_FRAME_MASK);
                        e2 = l2[PD_INDEX(virt_addr)];
                        if ((e2 & PAGE_PRESENT) != 0 && (e2 & PAGE_PS) == 0) {
                            uint64_t *l1 =
                                (uint64_t *)(uintptr_t)(e2 & PAGE_FRAME_MASK);
                            present_in_pt = 0u;
                            for (uint32_t k = 0; k < 512u; ++k) {
                                if ((l1[k] & PAGE_PRESENT) != 0) ++present_in_pt;
                            }
                        }
                    }
                }
                serial_write_string("[lostmap] demand-zero code va=");
                serial_write_uint64(virt_addr);
                serial_write_string(" pml4e=");
                serial_write_uint64(e4);
                serial_write_string(" pdpte=");
                serial_write_uint64(e3);
                serial_write_string(" pde=");
                serial_write_uint64(e2);
                serial_write_string(" ptes_present=");
                serial_write_uint64((uint64_t)present_in_pt);
                /* Are the tables themselves still owned? A "free" here means
                 * the frame was handed back to the allocator while the level
                 * above still pointed at it. */
                serial_write_string(" pdpt_alloc=");
                serial_write_uint64((uint64_t)(int64_t)pmm_page_is_allocated(
                    e4 & PAGE_FRAME_MASK));
                serial_write_string(" pd_alloc=");
                serial_write_uint64((uint64_t)(int64_t)pmm_page_is_allocated(
                    e3 & PAGE_FRAME_MASK));
                serial_write_string("\n");
            }
        }
#endif
        void *phys_page = alloc_page();
        if (phys_page == NULL) {
            spinlock_unlock(&g_demand_fault_lock);
            irq_restore(dz_irq);
            serial_write_string("[PF] demand-zero: alloc_page failed (PMM OOM) va=");
            serial_write_uint64(virt_addr);
            serial_write_string("\n");
            return 0;
        }

        memset(phys_page, 0, PAGE_SIZE_BYTES);

        if (paging_map_user_page(cr3, virt_addr, (uint64_t)(uintptr_t)phys_page, PAGE_RW) < 0) {
            free_page(phys_page);
            spinlock_unlock(&g_demand_fault_lock);
            irq_restore(dz_irq);
            serial_write_string("[PF] demand-zero: map_user_page failed (page-table OOM) va=");
            serial_write_uint64(virt_addr);
            serial_write_string("\n");
            return 0;
        }

        spinlock_unlock(&g_demand_fault_lock);
        irq_restore(dz_irq);

        if (cr3 == read_cr3()) {
            invlpg_addr(virt_addr);
        }

        return 1;
    }

    int repaired_permissions = 0;
    if ((*pml4e & PAGE_USER) == 0) { *pml4e |= PAGE_USER; repaired_permissions = 1; }
    if ((*pdpte & PAGE_USER) == 0) { *pdpte |= PAGE_USER; repaired_permissions = 1; }
    if ((*pde & PAGE_USER) == 0)   { *pde |= PAGE_USER; repaired_permissions = 1; }

    uint64_t entry = *pte;
    if ((entry & PAGE_PRESENT) != 0) {
        if ((entry & PAGE_USER) == 0) { 
            *pte = entry | PAGE_USER; 
            repaired_permissions = 1; 
        }

        if (repaired_permissions) {
            if (cr3 == read_cr3()) invlpg_addr(virt_addr);
            return 1;
        }
        return 0;
    }

    if ((entry & PAGE_SWAP) == 0 || !g_swap_enabled) {
        return 0;
    }

    spinlock_lock(&g_swap_lock);
    uint32_t slot = (uint32_t)((entry & PAGE_FRAME_MASK) >> 12);
    if (slot >= SWAP_SLOT_COUNT || !g_swap_slots[slot].used) {
        spinlock_unlock(&g_swap_lock);
        return 0;
    }

    void *phys_page = alloc_page();
    if (phys_page == NULL) {
        spinlock_unlock(&g_swap_lock);
        return 0;
    }

    copy_page_bytes((uint8_t *)phys_page, g_swap_slots[slot].phys_page);
    swap_free_slot(slot);

    // Clear the PTE so that subsequent attempts to access this page
    // MUST go through paging_handle_swap_fault again until swap-in completes.
    // However, if we're setting it to a phys_page immediately, we can
    // directly update the PTE.
    
    // Ensure all modifications to PTE are atomic with respect to the swap state
    // AND synchronized across TLB.
    
    uint64_t flags = entry & PAGE_RW;
    *pte = ((uint64_t)(uintptr_t)phys_page) | PAGE_PRESENT | PAGE_USER | flags;

    spinlock_lock(&g_swap_lock);
    swap_track_t *track = swap_find_track(cr3, virt_addr);
    if (track != NULL) {
        track->swapped = 0;
        track->slot_index = 0;
    } else {
        swap_track_page(cr3, virt_addr);
    }
    spinlock_unlock(&g_swap_lock);

    /* The page just became present: no other CPU can hold a translation that
     * is now wrong, only one that says "not present", and that costs at most a
     * spurious fault. A local invalidation is all this needs. */
    if (cr3 == read_cr3()) {
        invlpg_addr(virt_addr);
    }
    return 1;
}

void pmm_free_pages(void *virt, size_t num_pages)
{
    if (virt == NULL || num_pages == 0) return;

    uint8_t *p = (uint8_t *)virt;
    for (size_t i = 0; i < num_pages; ++i) {
        free_page(p + i * PAGE_SIZE_BYTES);
    }
}

uint64_t get_phys_base(void)
{
    return 0ULL;
}

uint64_t get_virt_base(void)
{
    return 0ULL;
}
