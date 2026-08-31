#include "Paging_Main.h"

#include <string.h>

#include "Core/sync/Spinlock.h"
#include "MemoryManagement/Memory_Main.h"
#include "interfaces/hal_cpu.h"
#include "interfaces/mmu_ops.h"
#include "kernel/config.h"

#define ARM64_DESC_VALID            (1ULL << 0)
#define ARM64_DESC_TABLE_OR_PAGE    (1ULL << 1)
#define ARM64_DESC_AF               (1ULL << 10)
#define ARM64_DESC_SH_INNER         (3ULL << 8)
#define ARM64_DESC_AP_EL1_RW        (0ULL << 6)
#define ARM64_DESC_AP_EL0_RW        (1ULL << 6)
#define ARM64_DESC_AP_EL1_RO        (2ULL << 6)
#define ARM64_DESC_AP_EL0_RO        (3ULL << 6)
#define ARM64_DESC_ATTRIDX_DEVICE   (0ULL << 2)
#define ARM64_DESC_ATTRIDX_NORMAL   (1ULL << 2)
#define ARM64_DESC_PXN              (1ULL << 53)
#define ARM64_DESC_UXN              (1ULL << 54)
#define ARM64_DESC_EXTERNAL         (1ULL << 55)
#define ARM64_DESC_ADDR_MASK        0x0000FFFFFFFFF000ULL

#define ARM64_L0_SHIFT              39U
#define ARM64_L1_SHIFT              30U
#define ARM64_L2_SHIFT              21U
#define ARM64_L3_SHIFT              12U
#define ARM64_INDEX_MASK            0x1FFULL
#define ARM64_TABLE_ENTRIES         512U
#define ARM64_L1_BLOCK_SIZE         (1ULL << ARM64_L1_SHIFT)
#define ARM64_L2_BLOCK_SIZE         (1ULL << ARM64_L2_SHIFT)

typedef struct {
    uint8_t used;
    uint64_t ttbr0;
    uint64_t *l0;
    uint64_t *l1;
} arm64_space_t;

static uint64_t g_kernel_l0[ARM64_TABLE_ENTRIES] __attribute__((aligned(4096)));
static uint64_t g_kernel_l1[ARM64_TABLE_ENTRIES] __attribute__((aligned(4096)));
static uint64_t g_kernel_ttbr = 0;
static arm64_space_t g_spaces_static[OS_CONFIG_PROCESS_MAX_COUNT];
static spinlock_t g_space_lock;

static inline void write_ttbr0(uint64_t value)
{
    __asm__ volatile("msr TTBR0_EL1, %0; dsb ish; isb" :: "r"(value) : "memory");
}

static inline void write_ttbr1(uint64_t value)
{
    __asm__ volatile("msr TTBR1_EL1, %0; dsb ish; isb" :: "r"(value) : "memory");
}

static inline void write_tcr(uint64_t value)
{
    __asm__ volatile("msr TCR_EL1, %0" :: "r"(value) : "memory");
}

static inline void write_mair(uint64_t value)
{
    __asm__ volatile("msr MAIR_EL1, %0" :: "r"(value) : "memory");
}

static inline uint64_t read_sctlr(void)
{
    uint64_t value;
    __asm__ volatile("mrs %0, SCTLR_EL1" : "=r"(value));
    return value;
}

static inline void write_sctlr(uint64_t value)
{
    __asm__ volatile("msr SCTLR_EL1, %0; isb" :: "r"(value) : "memory");
}

static inline uint64_t level_index(uint64_t va, unsigned shift)
{
    return (va >> shift) & ARM64_INDEX_MASK;
}

static inline int desc_is_valid(uint64_t desc)
{
    return (desc & ARM64_DESC_VALID) != 0;
}

static inline int desc_is_table(uint64_t desc, unsigned level)
{
    if (!desc_is_valid(desc)) {
        return 0;
    }
    if (level == 3) {
        return 0;
    }
    return (desc & ARM64_DESC_TABLE_OR_PAGE) != 0;
}

static inline uint64_t *desc_to_table(uint64_t desc)
{
    return (uint64_t *)(uintptr_t)(desc & ARM64_DESC_ADDR_MASK);
}

static inline uint64_t table_desc(uint64_t table)
{
    return (table & ARM64_DESC_ADDR_MASK) | ARM64_DESC_VALID | ARM64_DESC_TABLE_OR_PAGE;
}

static inline uint64_t block_or_page_desc(uint64_t phys, uint64_t flags, int user)
{
    uint64_t ap = ARM64_DESC_AP_EL1_RW;
    uint64_t xn = ARM64_DESC_PXN | ARM64_DESC_UXN;

    if ((flags & PAGE_RW) == 0) {
        ap = user ? ARM64_DESC_AP_EL0_RO : ARM64_DESC_AP_EL1_RO;
    } else if (user) {
        ap = ARM64_DESC_AP_EL0_RW;
    }

    if ((flags & PAGE_NX) == 0) {
        xn &= ~ARM64_DESC_UXN;
        if (!user) {
            xn &= ~ARM64_DESC_PXN;
        }
    }
    if (user) {
        xn |= ARM64_DESC_PXN;
    }

    uint64_t attr = (flags & PAGE_PCD) ? ARM64_DESC_ATTRIDX_DEVICE
                                       : ARM64_DESC_ATTRIDX_NORMAL;
    uint64_t sh = (flags & PAGE_PCD) ? 0 : ARM64_DESC_SH_INNER;

    return (phys & ARM64_DESC_ADDR_MASK) |
           ARM64_DESC_VALID |
           ARM64_DESC_TABLE_OR_PAGE |
           ARM64_DESC_AF |
           sh |
           ap |
           attr |
           xn |
           ((flags & PAGE_EXTERNAL) ? ARM64_DESC_EXTERNAL : 0u);
}

static inline uint64_t kernel_block_desc(uint64_t phys, int device)
{
    return (phys & ARM64_DESC_ADDR_MASK & ~(ARM64_L1_BLOCK_SIZE - 1ULL)) |
           ARM64_DESC_VALID |
           ARM64_DESC_AF |
           ARM64_DESC_SH_INNER |
           ARM64_DESC_AP_EL1_RW |
           (device ? ARM64_DESC_ATTRIDX_DEVICE : ARM64_DESC_ATTRIDX_NORMAL) |
           (device ? ARM64_DESC_PXN : 0) |
           ARM64_DESC_UXN;
}

static uint64_t *alloc_zeroed_table(void)
{
    uint64_t *table = (uint64_t *)alloc_page();
    if (table == NULL) {
        return NULL;
    }
    memset(table, 0, PAGE_SIZE);
    return table;
}

static uint64_t *ensure_l1_table(uint64_t *l0, uint64_t l0_index)
{
    uint64_t desc = l0[l0_index];
    if (desc_is_table(desc, 0)) {
        return desc_to_table(desc);
    }

    uint64_t *l1 = alloc_zeroed_table();
    if (l1 == NULL) {
        return NULL;
    }

    l0[l0_index] = table_desc((uint64_t)(uintptr_t)l1);
    return l1;
}

static arm64_space_t *find_space(uint64_t ttbr0)
{
    for (uint32_t i = 0; i < OS_CONFIG_PROCESS_MAX_COUNT; ++i) {
        if (g_spaces_static[i].used && g_spaces_static[i].ttbr0 == ttbr0) {
            return &g_spaces_static[i];
        }
    }
    return NULL;
}

static arm64_space_t *find_free_space(void)
{
    for (uint32_t i = 0; i < OS_CONFIG_PROCESS_MAX_COUNT; ++i) {
        if (!g_spaces_static[i].used) {
            return &g_spaces_static[i];
        }
    }
    return NULL;
}

static uint64_t *ensure_l2_table(uint64_t *l1, uint64_t l1_index)
{
    uint64_t desc = l1[l1_index];
    if (desc_is_table(desc, 1)) {
        return desc_to_table(desc);
    }

    uint64_t *l2 = alloc_zeroed_table();
    if (l2 == NULL) {
        return NULL;
    }

    l1[l1_index] = table_desc((uint64_t)(uintptr_t)l2);
    return l2;
}

static uint64_t *ensure_l2_table_preserve_block(uint64_t *l1, uint64_t l1_index)
{
    uint64_t desc = l1[l1_index];
    if (desc_is_table(desc, 1)) {
        return desc_to_table(desc);
    }

    uint64_t *l2 = alloc_zeroed_table();
    if (l2 == NULL) {
        return NULL;
    }

    if (desc_is_valid(desc)) {
        uint64_t block_base = desc & ARM64_DESC_ADDR_MASK & ~(ARM64_L1_BLOCK_SIZE - 1ULL);
        uint64_t attrs = desc & ~ARM64_DESC_ADDR_MASK;
        for (uint32_t i = 0; i < ARM64_TABLE_ENTRIES; ++i) {
            uint64_t phys = block_base + ((uint64_t)i * ARM64_L2_BLOCK_SIZE);
            l2[i] = (phys & ~(ARM64_L2_BLOCK_SIZE - 1ULL)) | attrs;
        }
    }

    l1[l1_index] = table_desc((uint64_t)(uintptr_t)l2);
    return l2;
}

static uint64_t *ensure_l3_table(uint64_t *l2, uint64_t l2_index)
{
    uint64_t desc = l2[l2_index];
    if (desc_is_table(desc, 2)) {
        return desc_to_table(desc);
    }

    uint64_t *l3 = alloc_zeroed_table();
    if (l3 == NULL) {
        return NULL;
    }

    l2[l2_index] = table_desc((uint64_t)(uintptr_t)l3);
    return l3;
}

static uint64_t *ensure_l3_table_preserve_block(uint64_t *l2, uint64_t l2_index)
{
    uint64_t desc = l2[l2_index];
    if (desc_is_table(desc, 2)) {
        return desc_to_table(desc);
    }

    uint64_t *l3 = alloc_zeroed_table();
    if (l3 == NULL) {
        return NULL;
    }

    if (desc_is_valid(desc)) {
        uint64_t block_base = desc & ARM64_DESC_ADDR_MASK & ~(ARM64_L2_BLOCK_SIZE - 1ULL);
        uint64_t attrs = desc & ~ARM64_DESC_ADDR_MASK;
        for (uint32_t i = 0; i < ARM64_TABLE_ENTRIES; ++i) {
            uint64_t phys = block_base + ((uint64_t)i * PAGE_SIZE);
            l3[i] = (phys & ARM64_DESC_ADDR_MASK) | attrs | ARM64_DESC_TABLE_OR_PAGE;
        }
    }

    l2[l2_index] = table_desc((uint64_t)(uintptr_t)l3);
    return l3;
}

static uint64_t *walk_l3_preserve_blocks(uint64_t ttbr0, uint64_t va, int create)
{
    uint64_t *l0 = (uint64_t *)(uintptr_t)(ttbr0 & PAGE_MASK);
    if (l0 == NULL) {
        return NULL;
    }

    uint64_t l0_index = level_index(va, ARM64_L0_SHIFT);
    uint64_t l1_index = level_index(va, ARM64_L1_SHIFT);
    uint64_t l2_index = level_index(va, ARM64_L2_SHIFT);
    uint64_t l3_index = level_index(va, ARM64_L3_SHIFT);

    uint64_t *l1 = create ? ensure_l1_table(l0, l0_index) : NULL;
    if (!create) {
        if (!desc_is_table(l0[l0_index], 0)) {
            return NULL;
        }
        l1 = desc_to_table(l0[l0_index]);
    }
    if (l1 == NULL) {
        return NULL;
    }

    uint64_t *l2 = create ? ensure_l2_table_preserve_block(l1, l1_index) : NULL;
    if (!create) {
        if (!desc_is_table(l1[l1_index], 1)) {
            return NULL;
        }
        l2 = desc_to_table(l1[l1_index]);
    }
    if (l2 == NULL) {
        return NULL;
    }

    uint64_t *l3 = create ? ensure_l3_table_preserve_block(l2, l2_index) : NULL;
    if (!create) {
        if (!desc_is_table(l2[l2_index], 2)) {
            return NULL;
        }
        l3 = desc_to_table(l2[l2_index]);
    }
    if (l3 == NULL) {
        return NULL;
    }

    return &l3[l3_index];
}

static uint64_t *walk_l3(uint64_t ttbr0, uint64_t va, int create)
{
    uint64_t *l0 = (uint64_t *)(uintptr_t)(ttbr0 & PAGE_MASK);
    if (l0 == NULL) {
        return NULL;
    }

    uint64_t l0_index = level_index(va, ARM64_L0_SHIFT);
    uint64_t l1_index = level_index(va, ARM64_L1_SHIFT);
    uint64_t l2_index = level_index(va, ARM64_L2_SHIFT);
    uint64_t l3_index = level_index(va, ARM64_L3_SHIFT);

    uint64_t *l1 = create ? ensure_l1_table(l0, l0_index) : NULL;
    if (!create) {
        if (!desc_is_table(l0[l0_index], 0)) {
            return NULL;
        }
        l1 = desc_to_table(l0[l0_index]);
    }
    if (l1 == NULL) {
        return NULL;
    }

    uint64_t *l2 = create ? ensure_l2_table(l1, l1_index) : NULL;
    if (!create) {
        if (!desc_is_table(l1[l1_index], 1)) {
            return NULL;
        }
        l2 = desc_to_table(l1[l1_index]);
    }
    if (l2 == NULL) {
        return NULL;
    }

    uint64_t *l3 = create ? ensure_l3_table(l2, l2_index) : NULL;
    if (!create) {
        if (!desc_is_table(l2[l2_index], 2)) {
            return NULL;
        }
        l3 = desc_to_table(l2[l2_index]);
    }
    if (l3 == NULL) {
        return NULL;
    }

    return &l3[l3_index];
}

static void free_user_tables(uint64_t *l1)
{
    for (uint32_t i = 0; i < ARM64_TABLE_ENTRIES; ++i) {
        uint64_t l1_desc = l1[i];
        if (!desc_is_table(l1_desc, 1)) {
            continue;
        }

        uint64_t *l2 = desc_to_table(l1_desc);
        for (uint32_t j = 0; j < ARM64_TABLE_ENTRIES; ++j) {
            uint64_t l2_desc = l2[j];
            if (!desc_is_table(l2_desc, 2)) {
                continue;
            }
            uint64_t *l3 = desc_to_table(l2_desc);
            for (uint32_t k = 0; k < ARM64_TABLE_ENTRIES; ++k) {
                if (desc_is_valid(l3[k]) &&
                    (l3[k] & ARM64_DESC_EXTERNAL) == 0u) {
                    free_page((void *)(uintptr_t)
                              (l3[k] & ARM64_DESC_ADDR_MASK));
                }
            }
            free_page(l3);
        }
        free_page(l2);
    }
}

void init_paging(void)
{
    memset(g_kernel_l0, 0, sizeof(g_kernel_l0));
    memset(g_kernel_l1, 0, sizeof(g_kernel_l1));
    memset(g_spaces_static, 0, sizeof(g_spaces_static));
    spinlock_init(&g_space_lock);

    for (uint64_t i = 0; i < ARM64_TABLE_ENTRIES; ++i) {
        g_kernel_l1[i] = kernel_block_desc(i * ARM64_L1_BLOCK_SIZE, i == 0 ? 1 : 0);
    }

    g_kernel_l0[0] = table_desc((uint64_t)(uintptr_t)g_kernel_l1);
    g_kernel_ttbr = (uint64_t)(uintptr_t)g_kernel_l0;

    write_mair(0xFF00ULL);
    write_tcr((16ULL << 0) | (16ULL << 16) |
              (0ULL << 14) | (2ULL << 30) |
              (3ULL << 12) | (3ULL << 28) |
              (1ULL << 8) | (1ULL << 24) |
              (5ULL << 32));
    write_ttbr0(g_kernel_ttbr);
    write_ttbr1(g_kernel_ttbr);
    __asm__ volatile("dsb ish; tlbi vmalle1; dsb ish; isb" ::: "memory");
    write_sctlr(read_sctlr() | 1ULL | (1ULL << 2) | (1ULL << 12));
}

void *map_mmio_virt(uint64_t phys_addr)
{
    const uint64_t mmio_window_size = 1024ULL * 1024ULL;
    const uint64_t high_mmio_base = 0x4000000000ULL;
    uint64_t base = phys_addr & ~(PAGE_SIZE - 1ULL);
    uint64_t flags = PAGE_RW | PAGE_NX;
    if (phys_addr >= high_mmio_base) {
        flags |= PAGE_PCD;
    }
    if (paging_map_kernel_range(base, mmio_window_size, flags) < 0) {
        return NULL;
    }
    return (void *)(uintptr_t)phys_addr;
}

uint64_t paging_get_kernel_cr3(void)
{
    return g_kernel_ttbr;
}

uint64_t paging_get_active_cr3(void)
{
    return hal_cpu_read_cr(3);
}

void paging_switch_cr3(uint64_t address_space)
{
    if (address_space == 0) {
        return;
    }
    write_ttbr0(address_space);
}

uint64_t paging_create_process_space(void)
{
    uint64_t irq_flags = hal_cpu_save_interrupts();
    spinlock_lock(&g_space_lock);
    arm64_space_t *space = find_free_space();
    if (space != NULL) {
        memset(space, 0, sizeof(*space));
        space->used = 1;
    }
    spinlock_unlock(&g_space_lock);
    hal_cpu_restore_interrupts(irq_flags);

    if (space == NULL) {
        return 0;
    }

    space->l0 = alloc_zeroed_table();
    if (space->l0 == NULL) {
        space->used = 0;
        return 0;
    }

    space->l1 = alloc_zeroed_table();
    if (space->l1 == NULL) {
        free_page(space->l0);
        space->l0 = NULL;
        space->used = 0;
        return 0;
    }
    
    memcpy(space->l0, g_kernel_l0, sizeof(g_kernel_l0));
    memcpy(space->l1, g_kernel_l1, sizeof(g_kernel_l1));
    space->l0[0] = table_desc((uint64_t)(uintptr_t)space->l1);
    space->ttbr0 = (uint64_t)(uintptr_t)space->l0;
    return space->ttbr0;
}

void paging_destroy_process_space(uint64_t address_space)
{
    if (address_space == 0 || address_space == g_kernel_ttbr) {
        return;
    }

    uint64_t irq_flags = hal_cpu_save_interrupts();
    spinlock_lock(&g_space_lock);
    arm64_space_t *space = find_space(address_space);
    if (space != NULL) {
        space->used = 0;
    }
    spinlock_unlock(&g_space_lock);
    hal_cpu_restore_interrupts(irq_flags);

    if (space == NULL) {
        return;
    }

    if (paging_get_active_cr3() == address_space) {
        write_ttbr0(g_kernel_ttbr);
    }

    if (space->l1 != NULL) {
        free_user_tables(space->l1);
        free_page(space->l1);
    }
    if (space->l0 != NULL) {
        free_page(space->l0);
    }

    memset(space, 0, sizeof(*space));
}

int paging_set_user_access(uint64_t address_space, uint64_t start, uint64_t size, int enable_user)
{
    if (address_space == 0 || size == 0) {
        return -1;
    }

    uint64_t end = (start + size + PAGE_SIZE - 1ULL) & ~(PAGE_SIZE - 1ULL);
    if (end < start) {
        return -1;
    }

    for (uint64_t va = start & ~(PAGE_SIZE - 1ULL); va < end; va += PAGE_SIZE) {
        uint64_t *pte = walk_l3_preserve_blocks(address_space, va, 1);
        if (pte == NULL || !desc_is_valid(*pte)) {
            return -1;
        }

        uint64_t phys = *pte & ARM64_DESC_ADDR_MASK;
        uint64_t ap = *pte & (3ULL << 6);
        uint64_t flags = ((*pte & ARM64_DESC_UXN) != 0) ? PAGE_NX : 0;
        if ((*pte & ARM64_DESC_EXTERNAL) != 0) flags |= PAGE_EXTERNAL;
        if (ap != ARM64_DESC_AP_EL1_RO && ap != ARM64_DESC_AP_EL0_RO) {
            flags |= PAGE_RW;
        }
        if (enable_user) {
            flags |= PAGE_USER;
        }
        *pte = block_or_page_desc(phys, flags, enable_user != 0);
    }

    __asm__ volatile("dsb ish; tlbi vmalle1is; dsb ish; isb" ::: "memory");
    return 0;
}

int paging_protect_user_range(uint64_t address_space, uint64_t start, uint64_t size,
                              uint64_t flags)
{
    if (address_space == 0 || size == 0 || (start & (PAGE_SIZE - 1ULL)) != 0)
        return -1;
    uint64_t end = (start + size + PAGE_SIZE - 1ULL) & ~(PAGE_SIZE - 1ULL);
    if (end <= start) return -1;

    for (uint64_t va = start; va < end; va += PAGE_SIZE) {
        uint64_t *pte = walk_l3_preserve_blocks(address_space, va, 0);
        if (pte == NULL || !desc_is_valid(*pte)) return -1;
        uint64_t physical = *pte & ARM64_DESC_ADDR_MASK;
        int user = (flags & PAGE_USER) != 0u;
        if ((*pte & ARM64_DESC_EXTERNAL) != 0) flags |= PAGE_EXTERNAL;
        *pte = block_or_page_desc(physical, flags, user);
    }
    __asm__ volatile("dsb ish; tlbi vmalle1is; dsb ish; isb" ::: "memory");
    return 0;
}

int paging_unmap_range(uint64_t address_space, uint64_t start, uint64_t size)
{
    if (address_space == 0 || size == 0) {
        return -1;
    }

    uint64_t end = (start + size + PAGE_SIZE - 1ULL) & ~(PAGE_SIZE - 1ULL);
    if (end < start) {
        return -1;
    }

    for (uint64_t va = start & ~(PAGE_SIZE - 1ULL); va < end; va += PAGE_SIZE) {
        uint64_t *pte = walk_l3(address_space, va, 0);
        if (pte == NULL || !desc_is_valid(*pte)) {
            continue;
        }
        if ((*pte & ARM64_DESC_EXTERNAL) == 0u) {
            free_page((void *)(uintptr_t)(*pte & ARM64_DESC_ADDR_MASK));
        }
        *pte = 0;
    }

    __asm__ volatile("dsb ish; tlbi vmalle1is; dsb ish; isb" ::: "memory");
    return 0;
}

int paging_is_user_range_mapped(uint64_t address_space, uint64_t start, uint64_t size)
{
    if (address_space == 0 || size == 0) {
        return 0;
    }

    uint64_t end = (start + size + PAGE_SIZE - 1ULL) & ~(PAGE_SIZE - 1ULL);
    if (end < start) {
        return 0;
    }

    for (uint64_t va = start & ~(PAGE_SIZE - 1ULL); va < end; va += PAGE_SIZE) {
        uint64_t *pte = walk_l3(address_space, va, 0);
        if (pte == NULL || !desc_is_valid(*pte)) {
            return 0;
        }
        uint64_t ap = *pte & (3ULL << 6);
        if (ap != ARM64_DESC_AP_EL0_RW && ap != ARM64_DESC_AP_EL0_RO) {
            return 0;
        }
    }
    return 1;
}

int paging_map_user_page(uint64_t address_space, uint64_t virt_addr, uint64_t phys_addr, uint64_t flags)
{
    if (address_space == 0) {
        return -1;
    }

    uint64_t *pte = walk_l3(address_space, virt_addr, 1);
    if (pte == NULL) {
        return -1;
    }

    if (desc_is_valid(*pte) && (*pte & ARM64_DESC_ADDR_MASK) != 0 &&
        (*pte & ARM64_DESC_EXTERNAL) == 0u) {
        free_page((void *)(uintptr_t)(*pte & ARM64_DESC_ADDR_MASK));
    }

    *pte = block_or_page_desc(phys_addr, flags, 1);
    hal_mmu_invalidate_tlb(virt_addr & PAGE_MASK);
    return 0;
}

int paging_map_user_range_alloc(uint64_t address_space, uint64_t start, uint64_t size, uint64_t flags)
{
    if (address_space == 0 || size == 0) {
        return -1;
    }

    uint64_t end = (start + size + PAGE_SIZE - 1ULL) & ~(PAGE_SIZE - 1ULL);
    if (end < start) {
        return -1;
    }

    for (uint64_t va = start & ~(PAGE_SIZE - 1ULL); va < end; va += PAGE_SIZE) {
        void *page = alloc_page();
        if (page == NULL) {
            return -1;
        }
        memset(page, 0, PAGE_SIZE);
        if (paging_map_user_page(address_space, va, (uint64_t)(uintptr_t)page, flags) < 0) {
            free_page(page);
            return -1;
        }
    }

    return 0;
}

static int copy_present_page(uint64_t child_ttbr0, uint64_t vaddr,
                             uint64_t parent_phys)
{
    void *child_page = alloc_page();
    if (child_page == NULL) {
        return -1;
    }
    memcpy(child_page, (void *)(uintptr_t)parent_phys, PAGE_SIZE);
    int ret = paging_map_user_page(child_ttbr0, vaddr,
                                   (uint64_t)(uintptr_t)child_page,
                                   PAGE_RW | PAGE_USER);
    if (ret < 0) {
        free_page(child_page);
    }
    return ret;
}

int paging_copy_present_user_range(uint64_t child_ttbr0, uint64_t parent_ttbr0,
                                   uint64_t start, uint64_t end)
{
    if (child_ttbr0 == 0 || parent_ttbr0 == 0 || end <= start) {
        return -1;
    }

    uint64_t first = start & PAGE_MASK;
    uint64_t last = (end - 1) & PAGE_MASK;

    uint64_t *l0 = (uint64_t *)(uintptr_t)(parent_ttbr0 & PAGE_MASK);
    if (l0 == NULL) {
        return -1;
    }

    uint64_t l0_first = level_index(first, ARM64_L0_SHIFT);
    uint64_t l0_last = level_index(last, ARM64_L0_SHIFT);
    uint64_t l1_first = level_index(first, ARM64_L1_SHIFT);
    uint64_t l1_last = level_index(last, ARM64_L1_SHIFT);
    uint64_t l2_first = level_index(first, ARM64_L2_SHIFT);
    uint64_t l2_last = level_index(last, ARM64_L2_SHIFT);
    uint64_t l3_first = level_index(first, ARM64_L3_SHIFT);
    uint64_t l3_last = level_index(last, ARM64_L3_SHIFT);

    for (uint64_t i = l0_first; i <= l0_last; ++i) {
        if (!desc_is_table(l0[i], 0)) {
            continue;
        }
        uint64_t *l1 = desc_to_table(l0[i]);

        uint64_t j_first = (i == l0_first) ? l1_first : 0;
        uint64_t j_last = (i == l0_last) ? l1_last : ARM64_TABLE_ENTRIES - 1;

        for (uint64_t j = j_first; j <= j_last; ++j) {
            uint64_t l1_desc = l1[j];
            if (!desc_is_valid(l1_desc) || !desc_is_table(l1_desc, 1)) {
                continue;
            }
            uint64_t *l2 = desc_to_table(l1_desc);

            uint64_t k_first = (i == l0_first && j == j_first) ? l2_first : 0;
            uint64_t k_last = (i == l0_last && j == j_last) ? l2_last
                                                            : ARM64_TABLE_ENTRIES - 1;

            for (uint64_t k = k_first; k <= k_last; ++k) {
                uint64_t l2_desc = l2[k];
                if (!desc_is_valid(l2_desc)) {
                    continue;
                }
                uint64_t region_base = (i << ARM64_L0_SHIFT) |
                                       (j << ARM64_L1_SHIFT) |
                                       (k << ARM64_L2_SHIFT);

                uint64_t l_first = (i == l0_first && j == j_first && k == k_first)
                                       ? l3_first : 0;
                uint64_t l_last = (i == l0_last && j == j_last && k == k_last)
                                      ? l3_last : ARM64_TABLE_ENTRIES - 1;

                if (!desc_is_table(l2_desc, 2)) {
                    uint64_t block_base = l2_desc & ARM64_DESC_ADDR_MASK;
                    for (uint64_t l = l_first; l <= l_last; ++l) {
                        uint64_t v = region_base + (l << ARM64_L3_SHIFT);
                        if (v < first || v > last) {
                            continue;
                        }
                        if (copy_present_page(child_ttbr0, v,
                                              block_base + (l << ARM64_L3_SHIFT)) < 0) {
                            return -1;
                        }
                    }
                    continue;
                }

                uint64_t *l3 = desc_to_table(l2_desc);
                for (uint64_t l = l_first; l <= l_last; ++l) {
                    uint64_t l3_desc = l3[l];
                    if (!desc_is_valid(l3_desc)) {
                        continue;
                    }
                    uint64_t v = region_base + (l << ARM64_L3_SHIFT);
                    if (v < first || v > last) {
                        continue;
                    }
                    if (copy_present_page(child_ttbr0, v,
                                          l3_desc & ARM64_DESC_ADDR_MASK) < 0) {
                        return -1;
                    }
                }
            }
        }
    }

    return 0;
}

int paging_map_kernel_range(uint64_t start, uint64_t size, uint64_t flags)
{
    if (size == 0) {
        return -1;
    }

    uint64_t end = (start + size + PAGE_SIZE - 1ULL) & ~(PAGE_SIZE - 1ULL);
    if (end < start) {
        return -1;
    }

    for (uint64_t va = start & ~(PAGE_SIZE - 1ULL); va < end; va += PAGE_SIZE) {
        uint64_t *pte = walk_l3_preserve_blocks(g_kernel_ttbr, va, 1);
        if (pte == NULL) {
            return -1;
        }
        *pte = block_or_page_desc(va, flags, 0);
    }

    uint64_t first_l0 = level_index(start, ARM64_L0_SHIFT);
    uint64_t last_l0 = level_index(end - 1ULL, ARM64_L0_SHIFT);
    if (last_l0 >= first_l0) {
        uint64_t irq_flags = hal_cpu_save_interrupts();
        spinlock_lock(&g_space_lock);
        for (uint32_t i = 0; i < OS_CONFIG_PROCESS_MAX_COUNT; ++i) {
            arm64_space_t *space = &g_spaces_static[i];
            if (!space->used || space->l0 == NULL) {
                continue;
            }
            for (uint64_t l0i = first_l0; l0i <= last_l0 && l0i < ARM64_TABLE_ENTRIES; ++l0i) {
                if (l0i != 0) {
                    space->l0[l0i] = g_kernel_l0[l0i];
                }
            }
        }
        spinlock_unlock(&g_space_lock);
        hal_cpu_restore_interrupts(irq_flags);
    }

    __asm__ volatile("dsb ish; tlbi vmalle1is; dsb ish; isb" ::: "memory");
    return 0;
}

void paging_swap_set_enabled(int enable)
{
    (void)enable;
}

int paging_swap_reclaim_one_page(void)
{
    return 0;
}

int paging_handle_swap_fault(uint64_t address_space, uint64_t fault_addr)
{
    (void)address_space;
    (void)fault_addr;
    return -1;
}

void *pmm_alloc_pages(size_t num_pages)
{
    return alloc_contiguous_pages((uint32_t)num_pages, 1);
}

void pmm_free_pages(void *virt, size_t num_pages)
{
    free_contiguous_pages(virt, (uint32_t)num_pages);
}

uint64_t get_phys_base(void)
{
    return 0;
}

uint64_t get_virt_base(void)
{
    return 0;
}

uint64_t paging_virt_to_phys(uint64_t address_space, uint64_t virt_addr)
{
    if (address_space == 0) {
        return 0;
    }

    uint64_t *l0 = (uint64_t *)(uintptr_t)(address_space & PAGE_MASK);
    if (l0 == NULL) {
        return 0;
    }

    uint64_t l0_desc = l0[level_index(virt_addr, ARM64_L0_SHIFT)];
    if (!desc_is_table(l0_desc, 0)) {
        return 0;
    }

    uint64_t *l1 = desc_to_table(l0_desc);
    uint64_t l1_desc = l1[level_index(virt_addr, ARM64_L1_SHIFT)];
    if (!desc_is_valid(l1_desc)) {
        return 0;
    }
    if (!desc_is_table(l1_desc, 1)) {
        return (l1_desc & ARM64_DESC_ADDR_MASK & ~(ARM64_L1_BLOCK_SIZE - 1ULL)) |
               (virt_addr & (ARM64_L1_BLOCK_SIZE - 1ULL));
    }

    uint64_t *l2 = desc_to_table(l1_desc);
    uint64_t l2_desc = l2[level_index(virt_addr, ARM64_L2_SHIFT)];
    if (!desc_is_table(l2_desc, 2)) {
        return 0;
    }

    uint64_t *l3 = desc_to_table(l2_desc);
    uint64_t l3_desc = l3[level_index(virt_addr, ARM64_L3_SHIFT)];
    if (!desc_is_valid(l3_desc)) {
        return 0;
    }

    return (l3_desc & ARM64_DESC_ADDR_MASK) | (virt_addr & (PAGE_SIZE - 1ULL));
}

static const mmu_ops_t g_arm64_mmu_ops = {
    .init = init_paging,
    .map_mmio = map_mmio_virt,
    .kernel_address_space = paging_get_kernel_cr3,
    .active_address_space = paging_get_active_cr3,
    .switch_address_space = paging_switch_cr3,
    .create_address_space = paging_create_process_space,
    .destroy_address_space = paging_destroy_process_space,
    .map_user_page = paging_map_user_page,
    .map_user_range_alloc = paging_map_user_range_alloc,
    .unmap_range = paging_unmap_range,
    .virt_to_phys = paging_virt_to_phys,
};

const mmu_ops_t *mmu_ops_get(void)
{
    return &g_arm64_mmu_ops;
}
