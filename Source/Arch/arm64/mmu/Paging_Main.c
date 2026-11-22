#include "Paging_Main.h"

#include <string.h>

#include "Core/sync/Spinlock.h"
#include "MemoryManagement/Memory_Main.h"
#include "Debug/serial/Serial.h"
#include "Arch/arm64/smp/SMP_Main.h"
#include "interfaces/hal_cpu.h"
#include "interfaces/mmu_ops.h"
#include "kernel/config.h"
#include "Core/process/ProcessManager.h"
#include "Core/memory/FileMap.h"

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
#define ARM64_DESC_ATTRIDX_NORMAL_NC (2ULL << 2)
#define ARM64_DESC_PXN              (1ULL << 53)
#define ARM64_DESC_UXN              (1ULL << 54)
#define ARM64_DESC_EXTERNAL         (1ULL << 55)
/* Software bit: a page fork shared copy-on-write. It is mapped read-only;
 * the first write gets a private copy (paging_handle_cow_fault). */
#define ARM64_DESC_COW              (1ULL << 56)
#define ARM64_DESC_ADDR_MASK        0x0000FFFFFFFFF000ULL

#define ARM64_L0_SHIFT              39U
#define ARM64_L1_SHIFT              30U
#define ARM64_L2_SHIFT              21U
#define ARM64_L3_SHIFT              12U
#define ARM64_INDEX_MASK            0x1FFULL
#define ARM64_TABLE_ENTRIES         512U
#define ARM64_L1_BLOCK_SIZE         (1ULL << ARM64_L1_SHIFT)
#define ARM64_L2_BLOCK_SIZE         (1ULL << ARM64_L2_SHIFT)
/* L1 slots (1 GiB each, under L0[0]) spanned by USER_CODE_BASE ..
 * USER_DISPLAY_FB_LIMIT (0x40_0000_0000 .. 0x80_0000_0000): code, heap and
 * stack, then the window SYSCALL_GET_DISPLAY_FRAMEBUFFER maps the
 * framebuffer into. */
#define ARM64_USER_L1_FIRST         256U
#define ARM64_USER_L1_LIMIT         512U
_Static_assert(((uint64_t)ARM64_USER_L1_FIRST << ARM64_L1_SHIFT) == USER_CODE_BASE,
               "ARM64_USER_L1_FIRST must start at USER_CODE_BASE");
_Static_assert(((uint64_t)ARM64_USER_L1_LIMIT << ARM64_L1_SHIFT) == USER_DISPLAY_FB_LIMIT,
               "ARM64_USER_L1_LIMIT must end at USER_DISPLAY_FB_LIMIT");

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
static spinlock_t g_demand_fault_lock;
/* The translation regime init_paging() switched to, for secondary CPUs to
 * load verbatim (paging_get_secondary_config). */
static uint64_t g_mmu_mair;
static uint64_t g_mmu_tcr;
static uint64_t g_mmu_sctlr;

/* Every address space is tagged ASID 0 (TCR_EL1.A1 = 0, TTBR0.ASID = 0), so
 * the TLB cannot tell one process's translations from another's. x86 drops
 * non-global entries on every CR3 write; do the equivalent here, or the next
 * process runs through whatever of the previous one's mappings the TLB still
 * caches. Local flush only: other CPUs keep their own address space. */
/* The tables each CPU is on (TTBR0_EL1, as last written by write_ttbr0):
 * the kernel's identity map lives in every process's own copy, so tables
 * stay in use for as long as any CPU runs on them, kernel code included.
 * See paging_space_active_on_any_cpu(). */
static volatile uint64_t g_cpu_ttbr0[OS_CONFIG_SMP_MAX_CPUS];

static inline void write_ttbr0(uint64_t value)
{
    __asm__ volatile("dsb ishst\n"
                     "msr TTBR0_EL1, %0\n"
                     "isb\n"
                     "tlbi vmalle1\n"
                     "dsb nsh\n"
                     "isb" :: "r"(value) : "memory");
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        __atomic_store_n(&g_cpu_ttbr0[cpu], value, __ATOMIC_RELEASE);
    }
}

int paging_space_active_on_any_cpu(uint64_t address_space)
{
    if (address_space == 0u) {
        return 0;
    }
    for (uint32_t cpu = 0; cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS; ++cpu) {
        if (__atomic_load_n(&g_cpu_ttbr0[cpu], __ATOMIC_ACQUIRE) == address_space) {
            return 1;
        }
    }
    return 0;
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
                                       : ((flags & PAGE_PWT) ?
                                          ARM64_DESC_ATTRIDX_NORMAL_NC :
                                          ARM64_DESC_ATTRIDX_NORMAL);
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

/* Free the L2/L3 tables under l1[first, limit) and every page they map
 * that the address space owns (PAGE_EXTERNAL pages belong to someone else).
 * Callers pass only ranges the process owns outright: the rest of a process
 * L1 is a copy of the kernel's, and its tables are the kernel's own. */
static void free_l1_subtree(uint64_t *l1, uint32_t first, uint32_t limit)
{
    for (uint32_t i = first; i < limit; ++i) {
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
        l1[i] = 0;
    }
}

extern char _kernel_start[];
extern char _kernel_end[];

static void dcache_clean_poc_range(uintptr_t start, uintptr_t end)
{
    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    uintptr_t line = (uintptr_t)4u << ((ctr >> 16) & 0xFu);
    for (uintptr_t p = start & ~(line - 1u); p < end; p += line) {
        __asm__ volatile("dc cvac, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

void init_paging(void)
{
    memset(g_kernel_l0, 0, sizeof(g_kernel_l0));
    memset(g_kernel_l1, 0, sizeof(g_kernel_l1));
    memset(g_spaces_static, 0, sizeof(g_spaces_static));
    spinlock_init(&g_space_lock);
    spinlock_init(&g_demand_fault_lock);

    for (uint64_t i = 0; i < ARM64_TABLE_ENTRIES; ++i) {
        g_kernel_l1[i] = kernel_block_desc(i * ARM64_L1_BLOCK_SIZE, i == 0 ? 1 : 0);
    }

    g_kernel_l0[0] = table_desc((uint64_t)(uintptr_t)g_kernel_l1);
    g_kernel_ttbr = (uint64_t)(uintptr_t)g_kernel_l0;

    /* AttrIdx 0: Device-nGnRnE, 1: Normal WB, 2: Normal non-cacheable. */
    const uint64_t mair = 0x0044FF00ULL;
    const uint64_t tcr = (16ULL << 0) | (16ULL << 16) |
                         (0ULL << 14) | (2ULL << 30) |
                         (3ULL << 12) | (3ULL << 28) |
                         (1ULL << 8) | (1ULL << 24) |
                         (5ULL << 32);
    /* M | C | I on, WXN off: the identity map keeps kernel text writable.
     * UCT and UCI let EL0 read CTR_EL0 and run DC CVAU / IC IVAU, as on
     * Linux: a program that writes code (a JIT, the x86_64 translator)
     * makes it visible to instruction fetch itself. */
    const uint64_t sctlr_on = (read_sctlr() | 1ULL | (1ULL << 2) | (1ULL << 12) |
                               (1ULL << 15) | (1ULL << 26)) &
                              ~(1ULL << 19);
    g_mmu_mair = mair;
    g_mmu_tcr = tcr;
    g_mmu_sctlr = sctlr_on;

    /* The firmware's tables are still live here, and they index MAIR with
     * their own attribute layout. Rewriting MAIR/TCR under them turns the
     * kernel's own text into Device memory (or an unmapped VA range) and the
     * very next instruction fetch faults -- into a vector table that faults
     * the same way. So switch with the MMU off, and do the whole sequence in
     * registers: with SCTLR.M clear every data access is non-cacheable and
     * would miss dirty lines (the stack included) still sitting in the cache.
     * The new tables were written through the cache; the walker sees them
     * because TCR selects inner-shareable write-back walks. */
    /* With SCTLR.M clear, instruction fetches and the table walk bypass the
     * caches. The boot manager copied (and relocated) this image through the
     * D-cache and the tables above were just written through it: push both
     * to the point of coherency first, or on real hardware (HVF) the CPU
     * would fetch the switch sequence below from stale RAM. */
    dcache_clean_poc_range((uintptr_t)_kernel_start, (uintptr_t)_kernel_end);

    __asm__ volatile("dsb ish\n"
                     "mrs x9, sctlr_el1\n"
                     "bic x9, x9, #1\n"
                     "msr sctlr_el1, x9\n"
                     "isb\n"
                     "msr mair_el1, %0\n"
                     "msr tcr_el1, %1\n"
                     "msr ttbr0_el1, %2\n"
                     "msr ttbr1_el1, %2\n"
                     "isb\n"
                     "tlbi vmalle1\n"
                     "dsb ish\n"
                     "isb\n"
                     "msr sctlr_el1, %3\n"
                     "isb\n"
                     "ic iallu\n"
                     "dsb ish\n"
                     "isb\n"
                     :
                     : "r"(mair), "r"(tcr), "r"(g_kernel_ttbr), "r"(sctlr_on)
                     : "x9", "memory");
}

/* Kernel view of MMIO that sits inside the user windows.
 *
 * The kernel identity-maps physical memory through TTBR0, and so does every
 * process address space (a copy of the kernel's L0/L1). That is fine below
 * 256 GiB, but QEMU virt puts the high PCIe ECAM at 0x40_1000_0000, inside
 * USER_CODE_BASE..USER_STACK_TOP, and its 64-bit BAR window at 0x80_0000_0000.
 * Mapping those through the identity map turned the kernel's L1[256] into a
 * table that every process then copied -- one shared set of L2/L3 tables for
 * every process's user image, so spawning a program overwrote its parent.
 *
 * Instead, MMIO at or above ARM64_HIGH_MMIO_BASE is handed out at its alias
 * in the upper half: TTBR1 always holds the kernel's own tables (it is never
 * switched per process) and the same L0 walk, so 0xFFFF_0000_0000_0000 + pa
 * reaches pa with the kernel's attributes in every address space, and the
 * user windows of a process stay its own (paging_create_process_space). */
#define ARM64_HIGH_MMIO_BASE   0x0000004000000000ULL
#define ARM64_KERNEL_ALIAS     0xFFFF000000000000ULL

static void *map_mmio_with_flags(uint64_t phys_addr, uint64_t flags)
{
    const uint64_t mmio_window_size = 1024ULL * 1024ULL;
    uint64_t base = phys_addr & ~(PAGE_SIZE - 1ULL);
    if (phys_addr >= ARM64_HIGH_MMIO_BASE) {
        flags |= PAGE_PCD;
    }
    if (paging_map_kernel_range(base, mmio_window_size, flags) < 0) {
        return NULL;
    }
    if (phys_addr >= ARM64_HIGH_MMIO_BASE) {
        return (void *)(uintptr_t)(ARM64_KERNEL_ALIAS | phys_addr);
    }
    return (void *)(uintptr_t)phys_addr;
}

void *map_mmio_virt(uint64_t phys_addr)
{
    return map_mmio_with_flags(phys_addr, PAGE_RW | PAGE_NX);
}

/* AArch64 uses the normal mapping attributes selected by map_mmio_virt() for
 * the boot framebuffer.  Keep this separate from the generic MMIO entry point
 * so the display driver can request the intended scanout policy on every arch. */
void *map_mmio_wc_virt(uint64_t phys_addr)
{
    return map_mmio_with_flags(phys_addr, PAGE_RW | PAGE_NX | PAGE_PWT);
}

void paging_get_secondary_config(uint64_t *ttbr, uint64_t *tcr,
                                 uint64_t *mair, uint64_t *sctlr)
{
    *ttbr = g_kernel_ttbr;
    *tcr = g_mmu_tcr;
    *mair = g_mmu_mair;
    *sctlr = g_mmu_sctlr;
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

volatile uint64_t g_paging_win_low_base;

void paging_win_low_window_set(uint64_t base)
{
    g_paging_win_low_base = base;
}

static int va_in_user_window(uint64_t va)
{
    /* The code/heap/stack window with the display framebuffer window above
     * it (ARM64_USER_L1_FIRST..LIMIT), and from 1 TiB the mmap arena plus
     * the rest of the 47-bit user range (USER_MMAP_FIXED_LIMIT). */
    return (va >= USER_CODE_BASE && va < USER_DISPLAY_FB_LIMIT) ||
           (va >= USER_MMAP_BASE && va < USER_MMAP_FIXED_LIMIT);
}

int paging_user_fixed_range_allowed(uint64_t addr, uint64_t len)
{
    if (len == 0u) {
        return 1;
    }
    uint64_t end = addr + len;
    if (end < addr) {
        return 0;
    }
    uint64_t first = addr & PAGE_MASK;
    uint64_t last = (end - 1u) & PAGE_MASK;
    /* Either window is one contiguous range: checking every page is only
     * needed where a shadow window and the identity map meet. */
    if (va_in_user_window(first) && va_in_user_window(last) &&
        (first >= 0x0000010000000000ULL) == (last >= 0x0000010000000000ULL)) {
        return 1;
    }
    for (uint64_t va = first;; va += PAGE_SIZE) {
        if (!va_in_user_window(va) && !paging_is_user_shadow_window(va)) {
            return 0;
        }
        if (va == last) {
            break;
        }
    }
    return 1;
}

static uint64_t *kernel_identity_l2(uint64_t i1);

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
    /* The user windows below 512 GiB (code, heap, stack, framebuffer)
     * belong to the process alone: never inherit the kernel's identity
     * entries there -- least of all a kernel table, which would be shared
     * with every other process. See map_mmio_with_flags(). */
    for (uint64_t i = ARM64_USER_L1_FIRST; i < ARM64_USER_L1_LIMIT; ++i) {
        space->l1[i] = 0;
    }
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
    /* Take the slot's contents and free the slot in one step, under the
     * lock. Marking it unused and reading l0/l1 from it afterwards let a
     * paging_create_process_space() on another CPU claim the slot in between
     * and store its new tables there -- which this then freed, leaving the
     * new process on freed tables (a kernel abort in walk_l3 at a garbage
     * address once Chromium started processes while others exited). */
    arm64_space_t dying_copy;
    arm64_space_t *space = NULL;
    spinlock_lock(&g_space_lock);
    arm64_space_t *slot = find_space(address_space);
    if (slot != NULL) {
        dying_copy = *slot;
        memset(slot, 0, sizeof(*slot));
        space = &dying_copy;
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
        free_l1_subtree(space->l1, ARM64_USER_L1_FIRST, ARM64_USER_L1_LIMIT);
        /* Identity 1 GiB blocks this process split to shadow part of them
         * (paging_is_user_shadow_window): the tables are its own, and of the
         * pages they map only the EL0 ones -- the identity entries are the
         * kernel's memory. */
        for (uint32_t i = 0; i < ARM64_USER_L1_FIRST; ++i) {
            uint64_t d1 = space->l1[i];
            if (d1 == g_kernel_l1[i] || !desc_is_table(d1, 1)) {
                continue;
            }
            uint64_t *l2 = desc_to_table(d1);
            /* A private copy of a kernel L2 (shadow_unshare_path) still
             * points at the kernel's own L3s off the copied path. */
            const uint64_t *k2 = kernel_identity_l2(i);
            for (uint32_t j = 0; j < ARM64_TABLE_ENTRIES; ++j) {
                if (!desc_is_table(l2[j], 2) || (k2 != NULL && l2[j] == k2[j])) {
                    continue;
                }
                uint64_t *l3 = desc_to_table(l2[j]);
                for (uint32_t k = 0; k < ARM64_TABLE_ENTRIES; ++k) {
                    uint64_t ap = l3[k] & (3ULL << 6);
                    if (desc_is_valid(l3[k]) &&
                        (ap == ARM64_DESC_AP_EL0_RW || ap == ARM64_DESC_AP_EL0_RO) &&
                        (l3[k] & ARM64_DESC_EXTERNAL) == 0u) {
                        free_page((void *)(uintptr_t)(l3[k] & ARM64_DESC_ADDR_MASK));
                    }
                }
                free_page(l3);
            }
            free_page(l2);
            space->l1[i] = g_kernel_l1[i];
        }
        free_page(space->l1);
    }
    if (space->l0 != NULL) {
        /* L0 slots the process filled itself (the mmap arena, 1..16 TiB):
         * anything that is not the kernel's entry is private to it. */
        for (uint32_t i = 1; i < ARM64_TABLE_ENTRIES; ++i) {
            uint64_t desc = space->l0[i];
            if (!desc_is_table(desc, 0) || desc == g_kernel_l0[i]) {
                continue;
            }
            uint64_t *l1 = desc_to_table(desc);
            free_l1_subtree(l1, 0, ARM64_TABLE_ENTRIES);
            free_page(l1);
        }
        free_page(space->l0);
    }

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
        const uint64_t cow = *pte & ARM64_DESC_COW;
        if (cow != 0u) {
            flags &= ~(uint64_t)PAGE_RW;    /* the write must still fault */
        }
        *pte = block_or_page_desc(phys, flags, enable_user != 0) | cow;
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
    /* Below the user windows the tables are the kernel's, shared by every
     * address space: never a process's to change -- except the shadow
     * windows, where only the process's own (EL0) pages are touched. */
    const int low = start < (uint64_t)ARM64_USER_L1_FIRST * ARM64_L1_BLOCK_SIZE;
    if (low && !paging_user_fixed_range_allowed(start, end - start)) return -1;

    for (uint64_t va = start; va < end; va += PAGE_SIZE) {
        uint64_t *pte = walk_l3_preserve_blocks(address_space, va, 0);
        if (low && pte != NULL && desc_is_valid(*pte)) {
            uint64_t ap0 = *pte & (3ULL << 6);
            if (ap0 != ARM64_DESC_AP_EL0_RW && ap0 != ARM64_DESC_AP_EL0_RO) continue;
        }
        /* A page not faulted in yet (an anonymous mmap is committed lazily)
         * takes its protection at the demand-zero fault: skip it, as the
         * x86_64 implementation does. Failing here made every mprotect()
         * of a fresh PROT_NONE reservation EFAULT -- glibc's thread stacks
         * among them, so pthread_create() never worked. */
        if (pte == NULL || !desc_is_valid(*pte)) continue;
        uint64_t physical = *pte & ARM64_DESC_ADDR_MASK;
        int user = (flags & PAGE_USER) != 0u;
        uint64_t page_flags = flags;
        const int external = (*pte & ARM64_DESC_EXTERNAL) != 0;
        if (external) page_flags |= PAGE_EXTERNAL;
        /* Write access to a frame fork shared with another address space is
         * granted copy-on-write: read-only plus ARM64_DESC_COW, so the write
         * faults and gets a private copy. Read-only drops the intent (a
         * write then faults for real, as the program asked). */
        uint64_t cow = 0;
        if ((page_flags & PAGE_RW) != 0u && user && !external &&
            pmm_page_ref_get(physical) > 1u) {
            page_flags &= ~(uint64_t)PAGE_RW;
            cow = ARM64_DESC_COW;
        }
        *pte = block_or_page_desc(physical, page_flags, user) | cow;
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
        if (!va_in_user_window(va)) {
            /* A split identity block: the kernel's entries stay; a page the
             * process mapped over a shadow window goes back to the
             * identity frame (withheld, so never in use). */
            uint64_t ap = *pte & (3ULL << 6);
            if (ap != ARM64_DESC_AP_EL0_RW && ap != ARM64_DESC_AP_EL0_RO) {
                continue;
            }
            if ((*pte & ARM64_DESC_EXTERNAL) == 0u) {
                free_page((void *)(uintptr_t)(*pte & ARM64_DESC_ADDR_MASK));
            }
            *pte = block_or_page_desc(va, PAGE_RW | PAGE_NX, 0);
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

/* The kernel's L2 table for identity L1 slot i1, or NULL when that slot is
 * a block in the kernel's own tables. */
static uint64_t *kernel_identity_l2(uint64_t i1)
{
    return desc_is_table(g_kernel_l1[i1], 1) ? desc_to_table(g_kernel_l1[i1]) : NULL;
}

/* Before a process maps a page of its own into a shadow window, the tables
 * on the way there must be its own. Its L1 is a copy of the kernel's, so an
 * identity slot the kernel itself keeps as a table (the first GiB of RAM on
 * QEMU virt, say) still points at the kernel's L2 -- shared by every
 * process. Splitting and filling that one put Wine's shared user data page
 * into every address space and left it there after Wine exited (the next
 * Wine's MAP_FIXED_NOREPLACE then failed with EEXIST). Copy the shared L2,
 * and the shared L3 below it on this path, into private tables first.
 * Returns 1 if anything changed (TLBs then need flushing), 0 if not, -1 if
 * out of memory. */
static int shadow_unshare_path(uint64_t *l1, uint64_t va)
{
    const uint64_t i1 = level_index(va, ARM64_L1_SHIFT);
    const uint64_t i2 = level_index(va, ARM64_L2_SHIFT);
    uint64_t *k2 = kernel_identity_l2(i1);
    if (k2 == NULL || !desc_is_table(l1[i1], 1)) {
        return 0;               /* a block: the split makes private tables */
    }
    int changed = 0;
    if (l1[i1] == g_kernel_l1[i1]) {
        uint64_t *n2 = alloc_zeroed_table();
        if (n2 == NULL) {
            return -1;
        }
        memcpy(n2, k2, PAGE_SIZE);
        __asm__ volatile("dsb ishst" ::: "memory");
        l1[i1] = table_desc((uint64_t)(uintptr_t)n2);
        changed = 1;
    }
    uint64_t *l2 = desc_to_table(l1[i1]);
    if (desc_is_table(l2[i2], 2) && l2[i2] == k2[i2]) {
        uint64_t *n3 = alloc_zeroed_table();
        if (n3 == NULL) {
            return -1;
        }
        memcpy(n3, desc_to_table(k2[i2]), PAGE_SIZE);
        __asm__ volatile("dsb ishst" ::: "memory");
        l2[i2] = table_desc((uint64_t)(uintptr_t)n3);
        changed = 1;
    }
    return changed;
}

int paging_map_user_page(uint64_t address_space, uint64_t virt_addr, uint64_t phys_addr, uint64_t flags)
{
    if (address_space == 0) {
        return -1;
    }

    if (!va_in_user_window(virt_addr)) {
        /* The kernel's identity map: only a shadow window, and only in the
         * process's own copy of the tables (the block is split, keeping
         * every other identity entry as it was). Anything else would take
         * the kernel's devices or memory away from it in this address
         * space -- a user page at 0x10000 used to replace the 1 GiB device
         * block holding the UART and the GIC. */
        if (!paging_is_user_shadow_window(virt_addr) ||
            address_space == g_kernel_ttbr) {
            return -1;
        }
        uint64_t *l0 = (uint64_t *)(uintptr_t)(address_space & PAGE_MASK);
        uint64_t *l1 = desc_to_table(l0[0]);
        const uint64_t i1 = level_index(virt_addr, ARM64_L1_SHIFT);
        const int unshared = shadow_unshare_path(l1, virt_addr);
        if (unshared < 0) {
            return -1;
        }
        const int split = unshared > 0 || !desc_is_table(l1[i1], 1) ||
            !desc_is_table(desc_to_table(l1[i1])[level_index(virt_addr, ARM64_L2_SHIFT)], 2);
        uint64_t *spte = walk_l3_preserve_blocks(address_space, virt_addr, 1);
        if (spte == NULL) {
            return -1;
        }
        if (split) {
            __asm__ volatile("dsb ish; tlbi vmalle1is; dsb ish; isb" ::: "memory");
        }
        uint64_t ap = *spte & (3ULL << 6);
        if (desc_is_valid(*spte) &&
            (ap == ARM64_DESC_AP_EL0_RW || ap == ARM64_DESC_AP_EL0_RO) &&
            (*spte & ARM64_DESC_EXTERNAL) == 0u) {
            free_page((void *)(uintptr_t)(*spte & ARM64_DESC_ADDR_MASK));
        }
        *spte = block_or_page_desc(phys_addr, flags, 1);
        hal_mmu_invalidate_tlb(virt_addr & PAGE_MASK);
        return 0;
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

int paging_map_user_page_cow(uint64_t address_space, uint64_t virt_addr,
                             uint64_t phys_addr, uint64_t flags)
{
    if (address_space == 0 || !va_in_user_window(virt_addr)) {
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
    /* One store: read-only and COW together, so another thread's write
     * never meets the page read-only without the COW mark (which would be
     * a genuine protection fault). */
    uint64_t desc = block_or_page_desc(phys_addr, flags & ~(uint64_t)PAGE_RW, 1);
    if ((flags & PAGE_RW) != 0u) {
        desc |= ARM64_DESC_COW;
    }
    *pte = desc;
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
                    /* Only what EL0 can reach is the process's: below the
                     * user windows these are the kernel's identity blocks,
                     * which a fork must neither read nor hand to the child. */
                    if ((l2_desc & ARM64_DESC_AP_EL0_RW) == 0u) {
                        continue;
                    }
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
                    if (!desc_is_valid(l3_desc) ||
                        (l3_desc & ARM64_DESC_AP_EL0_RW) == 0u) {
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

/* Demand-zero fill for an absent page in a user window (the mmap arena is
 * reserved lazily; see USER_MMAP_BASE in ProcessManager.h). There is no swap
 * on arm64, so this is the whole of what the x86 handler of the same name
 * does for a never-touched page. Returns 1 when the page is (now) present,
 * 0 when the fault is not one this handler services. */
int paging_handle_swap_fault(uint64_t address_space, uint64_t fault_addr)
{
    if (address_space == 0) {
        return 0;
    }
    uint64_t va = fault_addr & PAGE_MASK;
    if (!paging_addr_is_user(va)) {
        return 0;
    }
    int reclaimed = 0;

again:;
    uint64_t irq_flags = hal_cpu_save_interrupts();
    spinlock_lock(&g_demand_fault_lock);

    uint64_t *pte = walk_l3(address_space, va, 0);
    if (pte != NULL && desc_is_valid(*pte)) {
        uint64_t ap = *pte & (3ULL << 6);
        if (ap != ARM64_DESC_AP_EL0_RW && ap != ARM64_DESC_AP_EL0_RO) {
            /* The kernel's identity entry in a shadow window the process
             * has not mapped: not a page of its own to fill. */
            spinlock_unlock(&g_demand_fault_lock);
            hal_cpu_restore_interrupts(irq_flags);
            return 0;
        }
        /* Another CPU filled it in first: only our TLB entry is stale. */
        spinlock_unlock(&g_demand_fault_lock);
        hal_cpu_restore_interrupts(irq_flags);
        hal_mmu_invalidate_tlb(va);
        return 1;
    }

    void *page = alloc_page();
    if (page == NULL) {
        spinlock_unlock(&g_demand_fault_lock);
        hal_cpu_restore_interrupts(irq_flags);
        /* File pages only the shared page cache still holds are free for
         * the taking; then start over (another CPU may have filled the page
         * meanwhile). */
        if (!reclaimed && filemap_reclaim_shared_pages() > 0u) {
            reclaimed = 1;
            goto again;
        }
        serial_write_string("[OS] [PF] demand-zero: no free page, free=");
        serial_write_uint64(memory_free_pages());
        serial_write_string("\n");
        process_debug_memory_report("demand-zero fault", 0);
        return 0;
    }
    memset(page, 0, PAGE_SIZE);
    if (paging_map_user_page(address_space, va, (uint64_t)(uintptr_t)page,
                             PAGE_RW | PAGE_USER) < 0) {
        free_page(page);
        spinlock_unlock(&g_demand_fault_lock);
        hal_cpu_restore_interrupts(irq_flags);
        serial_write_string("[OS] [PF] demand-zero: map failed va=");
        serial_write_uint64(va);
        serial_write_string(" free=");
        serial_write_uint64(memory_free_pages());
        serial_write_string("\n");
        return 0;
    }
    spinlock_unlock(&g_demand_fault_lock);
    hal_cpu_restore_interrupts(irq_flags);
    return 1;
}

/* x86 page-fault error-code bits, which the generic fault path speaks. */
#define PF_ERR_WRITE (1ULL << 1)
#define PF_ERR_USER  (1ULL << 2)
#define PF_ERR_INSTR (1ULL << 4)

int paging_access_is_now_permitted(uint64_t address_space, uint64_t vaddr,
                                   uint64_t error_code)
{
    if (address_space == 0) {
        return 0;
    }
    uint64_t *pte = walk_l3(address_space, vaddr & PAGE_MASK, 0);
    if (pte == NULL || !desc_is_valid(*pte)) {
        return 0;
    }
    uint64_t desc = *pte;
    uint64_t ap = desc & (3ULL << 6);
    int el0 = (ap == ARM64_DESC_AP_EL0_RW || ap == ARM64_DESC_AP_EL0_RO);
    int ro = (ap == ARM64_DESC_AP_EL0_RO || ap == ARM64_DESC_AP_EL1_RO);
    if ((error_code & PF_ERR_USER) != 0 && !el0) {
        return 0;
    }
    if ((error_code & PF_ERR_WRITE) != 0 && ro) {
        return 0;
    }
    if ((error_code & PF_ERR_INSTR) != 0) {
        uint64_t xn = ((error_code & PF_ERR_USER) != 0) ? ARM64_DESC_UXN
                                                         : ARM64_DESC_PXN;
        if ((desc & xn) != 0) {
            return 0;
        }
    }
    return 1;
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
    if (!desc_is_valid(l2_desc)) {
        return 0;
    }
    if (!desc_is_table(l2_desc, 2)) {
        /* 2 MiB block (a split kernel 1 GiB block, see
         * ensure_l2_table_preserve_block). */
        return (l2_desc & ARM64_DESC_ADDR_MASK & ~(ARM64_L2_BLOCK_SIZE - 1ULL)) |
               (virt_addr & (ARM64_L2_BLOCK_SIZE - 1ULL));
    }

    uint64_t *l3 = desc_to_table(l2_desc);
    uint64_t l3_desc = l3[level_index(virt_addr, ARM64_L3_SHIFT)];
    if (!desc_is_valid(l3_desc)) {
        return 0;
    }

    return (l3_desc & ARM64_DESC_ADDR_MASK) | (virt_addr & (PAGE_SIZE - 1ULL));
}

/* --- Copy-on-write fork ----------------------------------------------------
 *
 * fork() shares the parent's pages with the child instead of copying them:
 * both map the frame read-only with ARM64_DESC_COW set, and the frame's
 * refcount (pmm_page_ref_*) counts the address spaces holding it. The first
 * write either way faults into paging_handle_cow_fault(), which copies the
 * page -- or, for the last owner, just gives write access back. The eager
 * copy this replaces made every fork of a translated Chromium zygote copy
 * hundreds of megabytes (the translator's code caches included), and a
 * headless run peaked near 6 GiB.
 *
 * Kernel writes to user memory through its virtual address fault like user
 * writes (AP read-only binds EL1 too) and are served by the same handler;
 * writes through the physical frame must call paging_user_range_break_cow()
 * first, as on x86_64. */

static inline uint64_t desc_user_page_flags(uint64_t desc)
{
    uint64_t flags = PAGE_USER;
    if ((desc & ARM64_DESC_UXN) != 0u) flags |= PAGE_NX;
    const uint64_t attr = desc & (7ULL << 2);
    if (attr == ARM64_DESC_ATTRIDX_DEVICE) flags |= PAGE_PCD;
    else if (attr == ARM64_DESC_ATTRIDX_NORMAL_NC) flags |= PAGE_PWT;
    return flags;
}

static int cow_share_one_page(uint64_t child_ttbr0, uint64_t va, uint64_t *parent_pte)
{
    const uint64_t desc = *parent_pte;
    const uint64_t phys = desc & ARM64_DESC_ADDR_MASK;
    if ((desc & ARM64_DESC_EXTERNAL) != 0u) {
        /* Frames with a life of their own (shared memory): as before. */
        return copy_present_page(child_ttbr0, va, phys);
    }
    const uint64_t ap = desc & (3ULL << 6);
    const int writable = ap == ARM64_DESC_AP_EL0_RW;
    /* Read-only only because an earlier fork shared it: still writable to
     * the program. */
    const int cow = writable || (desc & ARM64_DESC_COW) != 0u;

    /* The child's reference first: a parent unmapping the page meanwhile
     * must not drop the frame from under it. */
    pmm_page_ref_inc(phys);
    if (paging_map_user_page(child_ttbr0, va, phys, desc_user_page_flags(desc)) < 0) {
        (void)pmm_page_ref_dec(phys);
        return -1;
    }
    if (cow) {
        uint64_t *cpte = walk_l3(child_ttbr0, va, 0);
        if (cpte != NULL && desc_is_valid(*cpte)) {
            *cpte |= ARM64_DESC_COW;
        }
    }
    if (writable) {
        *parent_pte = (desc & ~(3ULL << 6)) | ARM64_DESC_AP_EL0_RO | ARM64_DESC_COW;
    }
    return 0;
}

int paging_cow_clone_user_range(uint64_t child_ttbr0, uint64_t parent_ttbr0,
                                uint64_t start, uint64_t end)
{
    if (child_ttbr0 == 0 || parent_ttbr0 == 0 || end <= start) {
        return -1;
    }
    const uint64_t first = start & PAGE_MASK;
    const uint64_t last = (end - 1) & PAGE_MASK;
    uint64_t *l0 = (uint64_t *)(uintptr_t)(parent_ttbr0 & PAGE_MASK);
    int rc = 0;

    uint64_t irq_flags = hal_cpu_save_interrupts();
    spinlock_lock(&g_demand_fault_lock);
    for (uint64_t i = level_index(first, ARM64_L0_SHIFT);
         i <= level_index(last, ARM64_L0_SHIFT) && rc == 0; ++i) {
        if (!desc_is_table(l0[i], 0)) continue;
        uint64_t *l1 = desc_to_table(l0[i]);
        for (uint64_t j = 0; j < ARM64_TABLE_ENTRIES && rc == 0; ++j) {
            const uint64_t base1 = (i << ARM64_L0_SHIFT) | (j << ARM64_L1_SHIFT);
            if (base1 + ARM64_L1_BLOCK_SIZE <= first || base1 > last) continue;
            if (!desc_is_valid(l1[j]) || !desc_is_table(l1[j], 1)) continue;
            uint64_t *l2 = desc_to_table(l1[j]);
            for (uint64_t k = 0; k < ARM64_TABLE_ENTRIES && rc == 0; ++k) {
                const uint64_t base2 = base1 | (k << ARM64_L2_SHIFT);
                if (base2 + (1ULL << ARM64_L2_SHIFT) <= first || base2 > last) continue;
                const uint64_t d2 = l2[k];
                if (!desc_is_valid(d2)) continue;
                if (!desc_is_table(d2, 2)) {
                    /* A block: only an EL0 one is the process's; copy it. */
                    if ((d2 & ARM64_DESC_AP_EL0_RW) == 0u) continue;
                    for (uint64_t l = 0; l < ARM64_TABLE_ENTRIES && rc == 0; ++l) {
                        const uint64_t v = base2 | (l << ARM64_L3_SHIFT);
                        if (v < first || v > last) continue;
                        rc = copy_present_page(child_ttbr0, v,
                                               (d2 & ARM64_DESC_ADDR_MASK) + (l << ARM64_L3_SHIFT));
                    }
                    continue;
                }
                uint64_t *l3 = desc_to_table(d2);
                for (uint64_t l = 0; l < ARM64_TABLE_ENTRIES && rc == 0; ++l) {
                    const uint64_t v = base2 | (l << ARM64_L3_SHIFT);
                    if (v < first || v > last) continue;
                    const uint64_t d3 = l3[l];
                    if (!desc_is_valid(d3)) continue;
                    const uint64_t ap = d3 & (3ULL << 6);
                    if (ap != ARM64_DESC_AP_EL0_RW && ap != ARM64_DESC_AP_EL0_RO) continue;
                    rc = cow_share_one_page(child_ttbr0, v, &l3[l]);
                }
            }
        }
    }
    spinlock_unlock(&g_demand_fault_lock);
    hal_cpu_restore_interrupts(irq_flags);
    /* The parent's writable entries just became read-only: every CPU running
     * one of its threads must stop writing through the old translation. */
    __asm__ volatile("dsb ish; tlbi vmalle1is; dsb ish; isb" ::: "memory");
    return rc;
}

static uint64_t g_cow_copies;

uint64_t paging_debug_cow_copies(void)
{
    return __atomic_load_n(&g_cow_copies, __ATOMIC_RELAXED);
}

int paging_handle_cow_fault(uint64_t address_space, uint64_t fault_addr)
{
    if (address_space == 0) {
        return 0;
    }
    const uint64_t va = fault_addr & PAGE_MASK;
    if (!paging_addr_is_user(va)) {
        return 0;
    }
    int reclaimed = 0;
again:;
    uint64_t irq_flags = hal_cpu_save_interrupts();
    spinlock_lock(&g_demand_fault_lock);
    uint64_t *pte = walk_l3(address_space, va, 0);
    if (pte == NULL || !desc_is_valid(*pte)) {
        spinlock_unlock(&g_demand_fault_lock);
        hal_cpu_restore_interrupts(irq_flags);
        return 0;
    }
    const uint64_t desc = *pte;
    const uint64_t ap = desc & (3ULL << 6);
    if (ap == ARM64_DESC_AP_EL0_RW) {
        /* Another thread broke it first: only our TLB entry is stale. */
        spinlock_unlock(&g_demand_fault_lock);
        hal_cpu_restore_interrupts(irq_flags);
        hal_mmu_invalidate_tlb(va);
        return 1;
    }
    if ((desc & ARM64_DESC_COW) == 0u || ap != ARM64_DESC_AP_EL0_RO) {
        spinlock_unlock(&g_demand_fault_lock);
        hal_cpu_restore_interrupts(irq_flags);
        return 0;               /* a genuinely read-only page */
    }
    const uint64_t old_phys = desc & ARM64_DESC_ADDR_MASK;
    const uint64_t rw = (desc & ~(3ULL << 6) & ~ARM64_DESC_COW) | ARM64_DESC_AP_EL0_RW;
    if (pmm_page_ref_get(old_phys) <= 1u) {
        /* The last owner: write access back, no copy. */
        *pte = rw;
        spinlock_unlock(&g_demand_fault_lock);
        hal_cpu_restore_interrupts(irq_flags);
        hal_mmu_invalidate_tlb(va);
        return 1;
    }
    void *fresh = alloc_page();
    if (fresh == NULL) {
        spinlock_unlock(&g_demand_fault_lock);
        hal_cpu_restore_interrupts(irq_flags);
        if (!reclaimed && filemap_reclaim_shared_pages() > 0u) {
            reclaimed = 1;
            goto again;
        }
        serial_write_string("[OS] [PF] copy-on-write: no free page\n");
        process_debug_memory_report("copy-on-write fault", 0);
        return 0;
    }
    memcpy(fresh, (const void *)(uintptr_t)old_phys, PAGE_SIZE);
    __atomic_add_fetch(&g_cow_copies, 1u, __ATOMIC_RELAXED);
    __asm__ volatile("dsb ishst" ::: "memory");
    *pte = (rw & ~ARM64_DESC_ADDR_MASK) | ((uint64_t)(uintptr_t)fresh & ARM64_DESC_ADDR_MASK);
    spinlock_unlock(&g_demand_fault_lock);
    hal_cpu_restore_interrupts(irq_flags);
    /* Every CPU running this address space may still translate the page to
     * the shared frame: flush them before this space's reference goes. */
    hal_mmu_invalidate_tlb(va);
    free_page((void *)(uintptr_t)old_phys);     /* drops one reference */
    return 1;
}

int paging_user_range_break_cow(uint64_t address_space, uint64_t start, uint64_t len)
{
    if (address_space == 0 || len == 0) {
        return 0;
    }
    const uint64_t end = start + len;
    if (end < start) {
        return -1;
    }
    for (uint64_t va = start & PAGE_MASK; va < end; va += PAGE_SIZE) {
        uint64_t *pte = walk_l3(address_space, va, 0);
        if (pte != NULL && desc_is_valid(*pte) && (*pte & ARM64_DESC_COW) != 0u) {
            if (paging_handle_cow_fault(address_space, va) <= 0) {
                return -1;
            }
        }
    }
    return 0;
}

int paging_user_range_is_writable(uint64_t address_space, uint64_t start, uint64_t len)
{
    if (address_space == 0 || len == 0) {
        return 1;
    }
    uint64_t first = start & PAGE_MASK;
    uint64_t last = (start + len - 1ULL) & PAGE_MASK;
    for (uint64_t page = first; ; page += PAGE_SIZE) {
        uint64_t *pte = walk_l3(address_space, page, 0);
        if (pte != NULL && desc_is_valid(*pte)) {
            uint64_t ap = *pte & (3ULL << 6);
            if (ap == ARM64_DESC_AP_EL0_RO || ap == ARM64_DESC_AP_EL1_RO) {
                return 0;
            }
        }
        if (page == last) {
            break;
        }
    }
    return 1;
}

uint64_t paging_debug_leaf_pte(uint64_t address_space, uint64_t virt_addr)
{
    if (address_space == 0) {
        return 0;
    }
    uint64_t desc = 0;
    uint64_t frame = 0;
    {
        /* Through 1 GiB and 2 MiB blocks too: the identity map is made of
         * them, and a caller asking "is this address taken?" must see it. */
        uint64_t *l0 = (uint64_t *)(uintptr_t)(address_space & PAGE_MASK);
        uint64_t d = l0[level_index(virt_addr, ARM64_L0_SHIFT)];
        if (!desc_is_table(d, 0)) return 0;
        d = desc_to_table(d)[level_index(virt_addr, ARM64_L1_SHIFT)];
        if (!desc_is_valid(d)) return 0;
        if (!desc_is_table(d, 1)) {
            desc = d;
            frame = ((d & ARM64_DESC_ADDR_MASK) & ~(ARM64_L1_BLOCK_SIZE - 1ULL)) |
                    (virt_addr & (ARM64_L1_BLOCK_SIZE - 1ULL) & PAGE_MASK);
        } else {
            d = desc_to_table(d)[level_index(virt_addr, ARM64_L2_SHIFT)];
            if (!desc_is_valid(d)) return 0;
            if (!desc_is_table(d, 2)) {
                desc = d;
                frame = ((d & ARM64_DESC_ADDR_MASK) & ~(ARM64_L2_BLOCK_SIZE - 1ULL)) |
                        (virt_addr & (ARM64_L2_BLOCK_SIZE - 1ULL) & PAGE_MASK);
            } else {
                d = desc_to_table(d)[level_index(virt_addr, ARM64_L3_SHIFT)];
                if (!desc_is_valid(d)) return 0;
                desc = d;
                frame = d & ARM64_DESC_ADDR_MASK;
            }
        }
    }
    uint64_t ap = desc & (3ULL << 6);
    uint64_t out = frame | PAGE_PRESENT;
    if (ap == ARM64_DESC_AP_EL0_RW || ap == ARM64_DESC_AP_EL1_RW) {
        out |= PAGE_RW;
    }
    if (ap == ARM64_DESC_AP_EL0_RW || ap == ARM64_DESC_AP_EL0_RO) {
        out |= PAGE_USER;
    }
    if ((desc & ARM64_DESC_UXN) != 0) {
        out |= PAGE_NX;
    }
    if ((desc & ARM64_DESC_EXTERNAL) != 0) {
        out |= PAGE_EXTERNAL;
    }
    return out;
}

/* A table pointer the report may follow: an allocated RAM frame. Anything
 * else (a table freed and reused under the unlocked walk) is skipped. */
static int debug_table_ok(uint64_t desc)
{
    return pmm_page_is_allocated(desc & ARM64_DESC_ADDR_MASK) == 1;
}

static void debug_count_l3(const uint64_t *l3, int el0_only,
                           uint64_t *own, uint64_t *shared)
{
    for (uint32_t k = 0; k < ARM64_TABLE_ENTRIES; ++k) {
        const uint64_t d = l3[k];
        if (!desc_is_valid(d) || (d & ARM64_DESC_EXTERNAL) != 0u) continue;
        const uint64_t ap = d & (3ULL << 6);
        if (el0_only && ap != ARM64_DESC_AP_EL0_RW && ap != ARM64_DESC_AP_EL0_RO) continue;
        ++*own;
        if (pmm_page_ref_get(d & ARM64_DESC_ADDR_MASK) > 1u) ++*shared;
    }
}

/* L2 tables under l1[first, limit) and their pages. `kernel_l1` (or NULL)
 * marks entries that are the kernel's own and not the process's. */
static void debug_count_l1(const uint64_t *l1, uint32_t first, uint32_t limit,
                           int identity, uint64_t *own, uint64_t *shared,
                           uint64_t *tables)
{
    for (uint32_t i = first; i < limit; ++i) {
        const uint64_t d1 = l1[i];
        if (!desc_is_table(d1, 1) || !debug_table_ok(d1)) continue;
        if (identity && d1 == g_kernel_l1[i]) continue;
        const uint64_t *l2 = desc_to_table(d1);
        const uint64_t *k2 = identity ? kernel_identity_l2(i) : NULL;
        ++*tables;
        for (uint32_t j = 0; j < ARM64_TABLE_ENTRIES; ++j) {
            const uint64_t d2 = l2[j];
            if (!desc_is_table(d2, 2) || !debug_table_ok(d2)) continue;
            if (k2 != NULL && d2 == k2[j]) continue;
            ++*tables;
            debug_count_l3(desc_to_table(d2), identity, own, shared);
        }
    }
}

uint64_t paging_debug_count_user_pages(uint64_t address_space,
                                       uint64_t *shared_out,
                                       uint64_t *tables_out)
{
    uint64_t own = 0, shared = 0, tables = 0;
    if (address_space != 0u && address_space != g_kernel_ttbr &&
        debug_table_ok(address_space)) {
        const uint64_t *l0 = (const uint64_t *)(uintptr_t)(address_space & PAGE_MASK);
        tables = 1;
        if (desc_is_table(l0[0], 0) && debug_table_ok(l0[0])) {
            const uint64_t *l1 = desc_to_table(l0[0]);
            ++tables;
            debug_count_l1(l1, ARM64_USER_L1_FIRST, ARM64_USER_L1_LIMIT, 0,
                           &own, &shared, &tables);
            debug_count_l1(l1, 0, ARM64_USER_L1_FIRST, 1, &own, &shared, &tables);
        }
        for (uint32_t i = 1; i < ARM64_TABLE_ENTRIES; ++i) {
            const uint64_t d0 = l0[i];
            if (!desc_is_table(d0, 0) || d0 == g_kernel_l0[i] || !debug_table_ok(d0)) {
                continue;
            }
            ++tables;
            debug_count_l1(desc_to_table(d0), 0, ARM64_TABLE_ENTRIES, 0,
                           &own, &shared, &tables);
        }
    }
    if (shared_out != NULL) *shared_out = shared;
    if (tables_out != NULL) *tables_out = tables;
    return own;
}

int paging_addr_is_user(uint64_t virt_addr)
{
    return va_in_user_window(virt_addr) || paging_is_user_shadow_window(virt_addr);
}

void paging_invalidate_page(uint64_t vaddr)
{
    hal_mmu_invalidate_tlb(vaddr & PAGE_MASK);
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
