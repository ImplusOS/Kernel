#include "IntelGen9_VM.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#define IG9_VM_ENTRIES_PER_TABLE 512u
#define IG9_VM_PAGE_MASK          0xFFFFFFFFFFFFF000ull
#define IG9_VM_PHYS_MASK          0x0000FFFFFFFFF000ull
#define IG9_VM_PHYS_MAX           0x0000FFFFFFFFFFFFull
#define IG9_VM_PTE_PRESENT        (1ull << 0)
#define IG9_VM_PTE_WRITE          (1ull << 1)
#define IG9_VM_PTE_PWT            (1ull << 3)
#define IG9_VM_PTE_PCD            (1ull << 4)
#define IG9_VM_PTE_PAT            (1ull << 7)
#define IG9_VM_PTE_UNCACHED       (IG9_VM_PTE_PWT | IG9_VM_PTE_PCD)
#define IG9_VM_VA_START           0x0000000000010000ull
#define IG9_VM_LEVELS              4u

typedef struct ig9_vm_pt_page {
    void *allocation;
    uint64_t dma_address;
    uint64_t *entries;
    uint8_t level;
    struct ig9_vm_pt_page *next;
} ig9_vm_pt_page_t;

typedef struct ig9_vm_va_range {
    uint64_t region_start;
    uint64_t region_size;
    uint64_t gpu_va;
    uint64_t mapped_size;
    struct ig9_vm_va_range *next;
} ig9_vm_va_range_t;

struct ig9_vm_space {
    volatile uint32_t lock;
    ig9_vm_pt_page_t *root;
    ig9_vm_pt_page_t *page_tables;
    ig9_vm_va_range_t *va_ranges;
    uint64_t generation;
    uint64_t invalidated_generation;
    uint32_t object_count;
};

struct ig9_gem_object {
    ig9_vm_space_t *vm;
    void *cpu_address;
    uint64_t physical_address;
    uint64_t gpu_address;
    uint64_t size;
    uint64_t allocation_size;
    uint32_t references;
    uint32_t map_flags;
};

static const driver_binary_t *g_ig9_vm_api;

static void ig9_vm_lock(ig9_vm_space_t *vm)
{
    while (__atomic_exchange_n(&vm->lock, 1u, __ATOMIC_ACQUIRE) != 0u) {
        while (__atomic_load_n(&vm->lock, __ATOMIC_RELAXED) != 0u) {
            __atomic_signal_fence(__ATOMIC_SEQ_CST);
        }
    }
}

static void ig9_vm_unlock(ig9_vm_space_t *vm)
{
    __atomic_store_n(&vm->lock, 0u, __ATOMIC_RELEASE);
}

static bool ig9_vm_add_overflow_u64(uint64_t a, uint64_t b, uint64_t *out)
{
    if (out == NULL || a > UINT64_MAX - b) {
        return true;
    }
    *out = a + b;
    return false;
}

static bool ig9_vm_align_up_u64(uint64_t value,
                                uint64_t alignment,
                                uint64_t *out)
{
    uint64_t mask;
    if (out == NULL || alignment == 0u ||
        (alignment & (alignment - 1u)) != 0u) {
        return false;
    }
    mask = alignment - 1u;
    if (value > UINT64_MAX - mask) {
        return false;
    }
    *out = (value + mask) & ~mask;
    return true;
}

static bool ig9_vm_round_size(uint64_t size, uint64_t *rounded)
{
    if (size == 0u || rounded == NULL ||
        size > UINT64_MAX - (IG9_VM_PAGE_SIZE - 1u)) {
        return false;
    }
    *rounded = (size + IG9_VM_PAGE_SIZE - 1u) &
               ~(uint64_t)(IG9_VM_PAGE_SIZE - 1u);
    return *rounded != 0u;
}

static void *ig9_vm_alloc_metadata(size_t bytes)
{
    if (g_ig9_vm_api == NULL || g_ig9_vm_api->mem.malloc == NULL ||
        bytes == 0u) {
        return NULL;
    }
    return g_ig9_vm_api->mem.malloc((uint64_t)bytes);
}

static void ig9_vm_free_metadata(void *ptr)
{
    if (ptr != NULL && g_ig9_vm_api != NULL &&
        g_ig9_vm_api->mem.free != NULL) {
        g_ig9_vm_api->mem.free(ptr);
    }
}

static void ig9_vm_zero(void *ptr, size_t bytes)
{
    if (ptr != NULL && g_ig9_vm_api != NULL &&
        g_ig9_vm_api->mem.memset != NULL) {
        (void)g_ig9_vm_api->mem.memset(ptr, 0, bytes);
    }
}

static ig9_vm_pt_page_t *ig9_vm_find_pt_locked(ig9_vm_space_t *vm,
                                                uint64_t dma_address)
{
    for (ig9_vm_pt_page_t *page = vm->page_tables;
         page != NULL; page = page->next) {
        if (page->dma_address == dma_address) {
            return page;
        }
    }
    return NULL;
}

static ig9_vm_pt_page_t *ig9_vm_alloc_pt_locked(ig9_vm_space_t *vm,
                                                 uint8_t level)
{
    uint64_t dma_address = 0u;
    void *allocation;
    ig9_vm_pt_page_t *page;

    if (level == 0u || level > IG9_VM_LEVELS ||
        g_ig9_vm_api == NULL ||
        g_ig9_vm_api->mem.dma_alloc_ex == NULL ||
        g_ig9_vm_api->mem.dma_free == NULL) {
        return NULL;
    }

    allocation = g_ig9_vm_api->mem.dma_alloc_ex(
        IG9_VM_PAGE_SIZE, IG9_VM_PAGE_SIZE, IG9_VM_PHYS_MAX,
        &dma_address);
    if (allocation == NULL || dma_address == 0u ||
        (((uintptr_t)allocation & (IG9_VM_PAGE_SIZE - 1u)) != 0u) ||
        ((dma_address & (IG9_VM_PAGE_SIZE - 1u)) != 0u) ||
        ((dma_address & ~IG9_VM_PHYS_MASK) != 0u)) {
        if (allocation != NULL) {
            g_ig9_vm_api->mem.dma_free(allocation, IG9_VM_PAGE_SIZE);
        }
        return NULL;
    }

    page = (ig9_vm_pt_page_t *)ig9_vm_alloc_metadata(sizeof(*page));
    if (page == NULL) {
        g_ig9_vm_api->mem.dma_free(allocation, IG9_VM_PAGE_SIZE);
        return NULL;
    }

    ig9_vm_zero(allocation, IG9_VM_PAGE_SIZE);
    page->allocation = allocation;
    page->dma_address = dma_address;
    page->entries = (uint64_t *)allocation;
    page->level = level;
    page->next = vm->page_tables;
    vm->page_tables = page;
    return page;
}

/*
 * Intel Gen8/Gen9 48-bit PPGTT uses PML4E[47:39], PDPE[38:30],
 * PDE[29:21], and PTE[20:12]. Non-leaf entries carry PRESENT|RW and
 * the uncached cache-policy bits. Leaf mappings default to uncached too:
 * this avoids relying on a PPAT programming sequence until that sequence is
 * implemented and validated as part of GT bring-up.
 */
static volatile uint64_t *ig9_vm_leaf_slot_locked(ig9_vm_space_t *vm,
                                                   uint64_t gpu_va,
                                                   bool create)
{
    static const uint8_t shifts[3] = { 39u, 30u, 21u };
    ig9_vm_pt_page_t *parent = vm->root;

    if (gpu_va >= IG9_VM_ADDRESS_LIMIT || parent == NULL) {
        return NULL;
    }

    for (uint32_t level_index = 0u; level_index < 3u; ++level_index) {
        uint32_t index = (uint32_t)((gpu_va >> shifts[level_index]) & 0x1FFu);
        uint64_t entry = __atomic_load_n(&parent->entries[index],
                                         __ATOMIC_ACQUIRE);
        ig9_vm_pt_page_t *child;

        if ((entry & IG9_VM_PTE_PRESENT) == 0u) {
            if (!create) {
                return NULL;
            }
            child = ig9_vm_alloc_pt_locked(
                vm, (uint8_t)(parent->level - 1u));
            if (child == NULL) {
                return NULL;
            }
            entry = child->dma_address | IG9_VM_PTE_PRESENT |
                    IG9_VM_PTE_WRITE | IG9_VM_PTE_UNCACHED;
            __atomic_store_n(&parent->entries[index], entry, __ATOMIC_RELEASE);
            (void)__atomic_add_fetch(&vm->generation, 1u, __ATOMIC_RELEASE);
        } else {
            uint64_t child_dma = entry & IG9_VM_PHYS_MASK;
            child = ig9_vm_find_pt_locked(vm, child_dma);
            if (child == NULL ||
                child->level != (uint8_t)(parent->level - 1u)) {
                return NULL;
            }
        }
        parent = child;
    }

    return &parent->entries[(gpu_va >> 12) & 0x1FFu];
}

static bool ig9_vm_va_alloc_locked(ig9_vm_space_t *vm,
                                   uint64_t size,
                                   uint64_t alignment,
                                   uint32_t guard_pages,
                                   uint64_t *gpu_va_out)
{
    uint64_t mapped_size;
    uint64_t guard_bytes = (uint64_t)guard_pages * IG9_VM_PAGE_SIZE;
    uint64_t total_size;
    uint64_t cursor = IG9_VM_VA_START;
    ig9_vm_va_range_t **insert_at = &vm->va_ranges;
    ig9_vm_va_range_t *range;
    uint64_t candidate_va = 0u;
    uint64_t candidate_start = 0u;
    uint64_t candidate_end = 0u;

    if (gpu_va_out == NULL || alignment < IG9_VM_PAGE_SIZE ||
        (alignment & (alignment - 1u)) != 0u ||
        !ig9_vm_round_size(size, &mapped_size) ||
        guard_bytes > UINT64_MAX / 2u ||
        ig9_vm_add_overflow_u64(mapped_size, guard_bytes * 2u,
                                &total_size)) {
        return false;
    }

    while (*insert_at != NULL) {
        range = *insert_at;
        uint64_t aligned_after_guard;
        if (ig9_vm_add_overflow_u64(cursor, guard_bytes,
                                    &aligned_after_guard) ||
            !ig9_vm_align_up_u64(aligned_after_guard, alignment,
                                 &candidate_va) ||
            candidate_va < guard_bytes) {
            return false;
        }
        candidate_start = candidate_va - guard_bytes;
        if (ig9_vm_add_overflow_u64(candidate_start, total_size,
                                    &candidate_end)) {
            return false;
        }
        if (candidate_start >= cursor &&
            candidate_end <= range->region_start) {
            break;
        }

        if (ig9_vm_add_overflow_u64(range->region_start,
                                    range->region_size, &cursor)) {
            return false;
        }
        insert_at = &range->next;
    }

    if (*insert_at == NULL) {
        uint64_t aligned_after_guard;
        if (ig9_vm_add_overflow_u64(cursor, guard_bytes,
                                    &aligned_after_guard) ||
            !ig9_vm_align_up_u64(aligned_after_guard, alignment,
                                 &candidate_va) ||
            candidate_va < guard_bytes) {
            return false;
        }
        candidate_start = candidate_va - guard_bytes;
        if (ig9_vm_add_overflow_u64(candidate_start, total_size,
                                    &candidate_end) ||
            candidate_end > IG9_VM_ADDRESS_LIMIT) {
            return false;
        }
    }

    range = (ig9_vm_va_range_t *)ig9_vm_alloc_metadata(sizeof(*range));
    if (range == NULL) {
        return false;
    }
    range->region_start = candidate_start;
    range->region_size = total_size;
    range->gpu_va = candidate_va;
    range->mapped_size = mapped_size;
    range->next = *insert_at;
    *insert_at = range;
    *gpu_va_out = candidate_va;
    return true;
}

static bool ig9_vm_va_free_locked(ig9_vm_space_t *vm,
                                  uint64_t gpu_va,
                                  uint64_t size)
{
    uint64_t mapped_size;
    ig9_vm_va_range_t **link = &vm->va_ranges;
    ig9_vm_va_range_t *range;

    if (!ig9_vm_round_size(size, &mapped_size)) {
        return false;
    }

    while (*link != NULL) {
        range = *link;
        if (range->gpu_va == gpu_va && range->mapped_size == mapped_size) {
            uint64_t pages = mapped_size / IG9_VM_PAGE_SIZE;
            for (uint64_t i = 0u; i < pages; ++i) {
                volatile uint64_t *slot = ig9_vm_leaf_slot_locked(
                    vm, gpu_va + i * IG9_VM_PAGE_SIZE, false);
                if (slot != NULL &&
                    (__atomic_load_n(slot, __ATOMIC_ACQUIRE) &
                     IG9_VM_PTE_PRESENT) != 0u) {
                    return false;
                }
            }
            *link = range->next;
            ig9_vm_free_metadata(range);
            return true;
        }
        link = &range->next;
    }
    return false;
}

static bool ig9_vm_map_contiguous_locked(ig9_vm_space_t *vm,
                                         uint64_t gpu_va,
                                         uint64_t physical_base,
                                         uint64_t size,
                                         uint32_t flags)
{
    uint64_t end;
    uint64_t physical_end;
    uint64_t pages;

    if (size == 0u || (gpu_va & (IG9_VM_PAGE_SIZE - 1u)) != 0u ||
        (physical_base & (IG9_VM_PAGE_SIZE - 1u)) != 0u ||
        (size & (IG9_VM_PAGE_SIZE - 1u)) != 0u ||
        (flags & ~(IG9_VM_MAP_READ | IG9_VM_MAP_WRITE)) != 0u ||
        (flags & IG9_VM_MAP_READ) == 0u ||
        ig9_vm_add_overflow_u64(gpu_va, size, &end) ||
        end > IG9_VM_ADDRESS_LIMIT ||
        ig9_vm_add_overflow_u64(physical_base, size - 1u, &physical_end) ||
        physical_base == 0u || physical_end > IG9_VM_PHYS_MAX ||
        (physical_base & ~IG9_VM_PHYS_MASK) != 0u) {
        return false;
    }

    pages = size / IG9_VM_PAGE_SIZE;

    /* First create any missing intermediate tables and verify that every
     * target leaf is empty. No leaf PTE is installed during this pass. */
    for (uint64_t i = 0u; i < pages; ++i) {
        volatile uint64_t *slot = ig9_vm_leaf_slot_locked(
            vm, gpu_va + i * IG9_VM_PAGE_SIZE, true);
        if (slot == NULL ||
            (__atomic_load_n(slot, __ATOMIC_ACQUIRE) &
             IG9_VM_PTE_PRESENT) != 0u) {
            return false;
        }
    }

    uint64_t pte_flags = IG9_VM_PTE_PRESENT | IG9_VM_PTE_UNCACHED;
    if ((flags & IG9_VM_MAP_WRITE) != 0u) {
        pte_flags |= IG9_VM_PTE_WRITE;
    }

    for (uint64_t i = 0u; i < pages; ++i) {
        volatile uint64_t *slot = ig9_vm_leaf_slot_locked(
            vm, gpu_va + i * IG9_VM_PAGE_SIZE, false);
        uint64_t pte = (physical_base + i * IG9_VM_PAGE_SIZE) | pte_flags;
        if (slot == NULL) {
            return false;
        }
        __atomic_store_n(slot, pte, __ATOMIC_RELEASE);
    }

    (void)__atomic_add_fetch(&vm->generation, 1u, __ATOMIC_RELEASE);
    return true;
}

static bool ig9_vm_unmap_locked(ig9_vm_space_t *vm,
                                uint64_t gpu_va,
                                uint64_t size)
{
    uint64_t end;
    uint64_t pages;

    if (size == 0u || (gpu_va & (IG9_VM_PAGE_SIZE - 1u)) != 0u ||
        (size & (IG9_VM_PAGE_SIZE - 1u)) != 0u ||
        ig9_vm_add_overflow_u64(gpu_va, size, &end) ||
        end > IG9_VM_ADDRESS_LIMIT) {
        return false;
    }

    pages = size / IG9_VM_PAGE_SIZE;

    /* Verify the complete range before clearing anything. */
    for (uint64_t i = 0u; i < pages; ++i) {
        volatile uint64_t *slot = ig9_vm_leaf_slot_locked(
            vm, gpu_va + i * IG9_VM_PAGE_SIZE, false);
        if (slot == NULL ||
            (__atomic_load_n(slot, __ATOMIC_ACQUIRE) &
             IG9_VM_PTE_PRESENT) == 0u) {
            return false;
        }
    }

    for (uint64_t i = 0u; i < pages; ++i) {
        volatile uint64_t *slot = ig9_vm_leaf_slot_locked(
            vm, gpu_va + i * IG9_VM_PAGE_SIZE, false);
        __atomic_store_n(slot, 0u, __ATOMIC_RELEASE);
    }

    (void)__atomic_add_fetch(&vm->generation, 1u, __ATOMIC_RELEASE);
    return true;
}

bool ig9_vm_system_init(const driver_binary_t *api)
{
    if (api == NULL || api->mem.malloc == NULL ||
        api->mem.free == NULL || api->mem.dma_alloc_ex == NULL ||
        api->mem.dma_free == NULL || api->mem.memset == NULL) {
        return false;
    }
    g_ig9_vm_api = api;
    return true;
}

ig9_vm_space_t *ig9_vm_create(void)
{
    ig9_vm_space_t *vm;
    ig9_vm_pt_page_t *root;

    if (g_ig9_vm_api == NULL) {
        return NULL;
    }
    vm = (ig9_vm_space_t *)ig9_vm_alloc_metadata(sizeof(*vm));
    if (vm == NULL) {
        return NULL;
    }
    ig9_vm_zero(vm, sizeof(*vm));
    __atomic_store_n(&vm->generation, 1u, __ATOMIC_RELAXED);
    __atomic_store_n(&vm->invalidated_generation, 1u, __ATOMIC_RELAXED);

    root = ig9_vm_alloc_pt_locked(vm, IG9_VM_LEVELS);
    if (root == NULL) {
        ig9_vm_free_metadata(vm);
        return NULL;
    }
    vm->root = root;
    return vm;
}

bool ig9_vm_destroy(ig9_vm_space_t *vm)
{
    ig9_vm_pt_page_t *page;
    ig9_vm_va_range_t *range;

    if (vm == NULL) {
        return false;
    }

    ig9_vm_lock(vm);
    if (vm->object_count != 0u) {
        ig9_vm_unlock(vm);
        return false;
    }

    page = vm->page_tables;
    vm->page_tables = NULL;
    range = vm->va_ranges;
    vm->va_ranges = NULL;
    vm->root = NULL;
    ig9_vm_unlock(vm);

    while (range != NULL) {
        ig9_vm_va_range_t *next = range->next;
        ig9_vm_free_metadata(range);
        range = next;
    }
    while (page != NULL) {
        ig9_vm_pt_page_t *next = page->next;
        if (g_ig9_vm_api != NULL &&
            g_ig9_vm_api->mem.dma_free != NULL) {
            g_ig9_vm_api->mem.dma_free(page->allocation, IG9_VM_PAGE_SIZE);
        }
        ig9_vm_free_metadata(page);
        page = next;
    }
    ig9_vm_free_metadata(vm);
    return true;
}

bool ig9_vm_va_alloc(ig9_vm_space_t *vm,
                     uint64_t size,
                     uint64_t alignment,
                     uint32_t guard_pages,
                     uint64_t *gpu_va_out)
{
    bool result;
    if (vm == NULL) {
        return false;
    }
    ig9_vm_lock(vm);
    result = ig9_vm_va_alloc_locked(vm, size, alignment,
                                    guard_pages, gpu_va_out);
    ig9_vm_unlock(vm);
    return result;
}

bool ig9_vm_va_free(ig9_vm_space_t *vm,
                    uint64_t gpu_va,
                    uint64_t size)
{
    bool result;
    if (vm == NULL) {
        return false;
    }
    ig9_vm_lock(vm);
    result = ig9_vm_va_free_locked(vm, gpu_va, size);
    ig9_vm_unlock(vm);
    return result;
}

bool ig9_vm_map_contiguous(ig9_vm_space_t *vm,
                           uint64_t gpu_va,
                           uint64_t physical_base,
                           uint64_t size,
                           uint32_t flags)
{
    bool result;
    if (vm == NULL) {
        return false;
    }
    ig9_vm_lock(vm);
    result = ig9_vm_map_contiguous_locked(vm, gpu_va, physical_base,
                                          size, flags);
    ig9_vm_unlock(vm);
    return result;
}

bool ig9_vm_unmap(ig9_vm_space_t *vm,
                  uint64_t gpu_va,
                  uint64_t size)
{
    bool result;
    if (vm == NULL) {
        return false;
    }
    ig9_vm_lock(vm);
    result = ig9_vm_unmap_locked(vm, gpu_va, size);
    ig9_vm_unlock(vm);
    return result;
}

bool ig9_vm_translate(ig9_vm_space_t *vm,
                      uint64_t gpu_va,
                      uint64_t *physical_out)
{
    volatile uint64_t *slot;
    uint64_t pte;
    bool result = false;

    if (vm == NULL || physical_out == NULL ||
        gpu_va >= IG9_VM_ADDRESS_LIMIT) {
        return false;
    }

    ig9_vm_lock(vm);
    slot = ig9_vm_leaf_slot_locked(vm, gpu_va, false);
    if (slot != NULL) {
        pte = __atomic_load_n(slot, __ATOMIC_ACQUIRE);
        if ((pte & IG9_VM_PTE_PRESENT) != 0u) {
            *physical_out = (pte & IG9_VM_PHYS_MASK) |
                            (gpu_va & (IG9_VM_PAGE_SIZE - 1u));
            result = true;
        }
    }
    ig9_vm_unlock(vm);
    return result;
}

uint64_t ig9_vm_root_dma(const ig9_vm_space_t *vm)
{
    return (vm != NULL && vm->root != NULL) ? vm->root->dma_address : 0u;
}

uint64_t ig9_vm_generation(const ig9_vm_space_t *vm)
{
    return (vm != NULL) ? __atomic_load_n(&vm->generation, __ATOMIC_ACQUIRE) : 0u;
}

bool ig9_vm_tlb_invalidate_pending(const ig9_vm_space_t *vm)
{
    if (vm == NULL) {
        return false;
    }
    return __atomic_load_n(&vm->generation, __ATOMIC_ACQUIRE) !=
           __atomic_load_n(&vm->invalidated_generation, __ATOMIC_ACQUIRE);
}

void ig9_vm_note_tlb_invalidate(ig9_vm_space_t *vm, uint64_t generation)
{
    if (vm == NULL) {
        return;
    }
    ig9_vm_lock(vm);
    if (__atomic_load_n(&vm->generation, __ATOMIC_ACQUIRE) == generation) {
        __atomic_store_n(&vm->invalidated_generation, generation,
                         __ATOMIC_RELEASE);
    }
    ig9_vm_unlock(vm);
}

ig9_gem_object_t *ig9_gem_create(ig9_vm_space_t *vm,
                                uint64_t size,
                                uint32_t map_flags)
{
    uint64_t allocation_size;
    uint64_t physical = 0u;
    uint64_t gpu_va = 0u;
    void *cpu;
    ig9_gem_object_t *object;
    bool va_allocated = false;
    bool mapped = false;

    if (vm == NULL || (map_flags & ~(IG9_VM_MAP_READ | IG9_VM_MAP_WRITE)) != 0u ||
        (map_flags & IG9_VM_MAP_READ) == 0u ||
        !ig9_vm_round_size(size, &allocation_size) ||
        allocation_size > (uint64_t)SIZE_MAX ||
        g_ig9_vm_api == NULL ||
        g_ig9_vm_api->mem.dma_alloc_ex == NULL ||
        g_ig9_vm_api->mem.dma_free == NULL) {
        return NULL;
    }

    object = (ig9_gem_object_t *)ig9_vm_alloc_metadata(sizeof(*object));
    if (object == NULL) {
        return NULL;
    }
    ig9_vm_zero(object, sizeof(*object));

    cpu = g_ig9_vm_api->mem.dma_alloc_ex(
        (size_t)allocation_size, IG9_VM_PAGE_SIZE, IG9_VM_PHYS_MAX,
        &physical);
    if (cpu == NULL || physical == 0u ||
        (((uintptr_t)cpu & (IG9_VM_PAGE_SIZE - 1u)) != 0u) ||
        ((physical & (IG9_VM_PAGE_SIZE - 1u)) != 0u) ||
        ((physical & ~IG9_VM_PHYS_MASK) != 0u) ||
        allocation_size - 1u > IG9_VM_PHYS_MAX - physical) {
        if (cpu != NULL) {
            g_ig9_vm_api->mem.dma_free(cpu, (size_t)allocation_size);
        }
        ig9_vm_free_metadata(object);
        return NULL;
    }

    ig9_vm_lock(vm);
    if (ig9_vm_va_alloc_locked(vm, allocation_size, IG9_VM_PAGE_SIZE,
                               1u, &gpu_va)) {
        va_allocated = true;
        mapped = ig9_vm_map_contiguous_locked(vm, gpu_va, physical,
                                              allocation_size, map_flags);
    }
    if (mapped) {
        object->vm = vm;
        object->cpu_address = cpu;
        object->physical_address = physical;
        object->gpu_address = gpu_va;
        object->size = size;
        object->allocation_size = allocation_size;
        object->references = 1u;
        object->map_flags = map_flags;
        ++vm->object_count;
    } else if (va_allocated) {
        (void)ig9_vm_va_free_locked(vm, gpu_va, allocation_size);
    }
    ig9_vm_unlock(vm);

    if (!mapped) {
        g_ig9_vm_api->mem.dma_free(cpu, (size_t)allocation_size);
        ig9_vm_free_metadata(object);
        return NULL;
    }

    return object;
}

bool ig9_gem_get(ig9_gem_object_t *object)
{
    bool result = false;
    ig9_vm_space_t *vm;
    if (object == NULL || object->vm == NULL) {
        return false;
    }
    vm = object->vm;
    ig9_vm_lock(vm);
    if (object->references != 0u && object->references != UINT32_MAX) {
        ++object->references;
        result = true;
    }
    ig9_vm_unlock(vm);
    return result;
}

bool ig9_gem_put(ig9_gem_object_t *object)
{
    ig9_vm_space_t *vm;
    void *cpu;
    uint64_t size;

    if (object == NULL || object->vm == NULL) {
        return false;
    }
    vm = object->vm;

    ig9_vm_lock(vm);
    if (object->references == 0u) {
        ig9_vm_unlock(vm);
        return false;
    }
    if (object->references > 1u) {
        --object->references;
        ig9_vm_unlock(vm);
        return true;
    }

    /* A GEM allocation is never returned to the DMA pool unless its PPGTT
     * mapping is confirmed removed. The later engine path must also wait for
     * all requests referencing this object before calling the final put. */
    if (!ig9_vm_unmap_locked(vm, object->gpu_address,
                             object->allocation_size) ||
        !ig9_vm_va_free_locked(vm, object->gpu_address,
                               object->allocation_size)) {
        ig9_vm_unlock(vm);
        return false;
    }

    object->references = 0u;
    if (vm->object_count != 0u) {
        --vm->object_count;
    }
    cpu = object->cpu_address;
    size = object->allocation_size;
    ig9_vm_unlock(vm);

    g_ig9_vm_api->mem.dma_free(cpu, (size_t)size);
    ig9_vm_free_metadata(object);
    return true;
}

void *ig9_gem_cpu_address(const ig9_gem_object_t *object)
{
    return (object != NULL) ? object->cpu_address : NULL;
}

uint64_t ig9_gem_gpu_address(const ig9_gem_object_t *object)
{
    return (object != NULL) ? object->gpu_address : 0u;
}

uint64_t ig9_gem_physical_address(const ig9_gem_object_t *object)
{
    return (object != NULL) ? object->physical_address : 0u;
}

uint64_t ig9_gem_size(const ig9_gem_object_t *object)
{
    return (object != NULL) ? object->size : 0u;
}

bool ig9_vm_software_selftest(const driver_binary_t *api)
{
    ig9_vm_space_t *vm;
    ig9_gem_object_t *object;
    uint64_t physical0 = 0u;
    uint64_t physical1 = 0u;
    uint64_t generation;
    uint64_t old_gpu_va = 0u;
    uint64_t ignored = 0u;
    bool ok = false;

    if (!ig9_vm_system_init(api)) {
        return false;
    }
    vm = ig9_vm_create();
    if (vm == NULL || ig9_vm_root_dma(vm) == 0u ||
        (ig9_vm_root_dma(vm) & (IG9_VM_PAGE_SIZE - 1u)) != 0u) {
        if (vm != NULL) {
            (void)ig9_vm_destroy(vm);
        }
        return false;
    }

    object = ig9_gem_create(vm, (uint64_t)IG9_VM_PAGE_SIZE * 2u,
                            IG9_VM_MAP_READ | IG9_VM_MAP_WRITE);
    if (object == NULL) {
        (void)ig9_vm_destroy(vm);
        return false;
    }

    old_gpu_va = ig9_gem_gpu_address(object);
    generation = ig9_vm_generation(vm);
    ok = generation > 1u &&
         ig9_vm_tlb_invalidate_pending(vm) &&
         ig9_vm_translate(vm, ig9_gem_gpu_address(object), &physical0) &&
         ig9_vm_translate(vm,
                          ig9_gem_gpu_address(object) + IG9_VM_PAGE_SIZE,
                          &physical1) &&
         physical0 == ig9_gem_physical_address(object) &&
         physical1 == ig9_gem_physical_address(object) + IG9_VM_PAGE_SIZE;

    if (ok) {
        ok = !ig9_vm_translate(vm,
                               ig9_gem_gpu_address(object) - IG9_VM_PAGE_SIZE,
                               &ignored) &&
             !ig9_vm_translate(vm,
                               ig9_gem_gpu_address(object) +
                               ((ig9_gem_size(object) + IG9_VM_PAGE_SIZE - 1u) &
                                ~(uint64_t)(IG9_VM_PAGE_SIZE - 1u)),
                               &ignored);
    }

    if (!ig9_gem_put(object)) {
        ok = false;
    }
    if (ig9_vm_translate(vm, old_gpu_va, &ignored)) {
        ok = false;
    }

    /* Reuse the released VA range and check that the stale translation did
     * not survive object destruction. */
    ig9_gem_object_t *second = ig9_gem_create(
        vm, IG9_VM_PAGE_SIZE, IG9_VM_MAP_READ | IG9_VM_MAP_WRITE);
    if (second == NULL ||
        !ig9_vm_translate(vm, ig9_gem_gpu_address(second), &ignored) ||
        ignored != ig9_gem_physical_address(second)) {
        ok = false;
    }
    if (second != NULL && !ig9_gem_put(second)) {
        ok = false;
    }

    if (!ig9_vm_destroy(vm)) {
        ok = false;
    }
    return ok;
}
