#include "Memory_Main.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Debug/serial/Serial.h"
#include "smp/SMP_Main.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PAGE_SIZE 4096
#define MAX_PAGES_HARD_LIMIT 8388608ULL
#define MAX_ALLOC_PAGE_RECURSION_DEPTH 5

static uint8_t  *g_page_bitmap     = NULL;
static uint64_t  g_max_pages       = 0;
static uint32_t  g_alloc_page_recursion_depth[OS_CONFIG_SMP_MAX_CPUS] = {0};
int paging_swap_reclaim_one_page(void);

/* COW physical-page refcounts (see Memory_Main.h). NULL until
 * memory_init_page_refcounts() runs; one byte per physical frame. */
static uint8_t  *g_page_refcount   = NULL;
static uint64_t  g_page_refcount_pages = 0;
static spinlock_t g_page_refcount_lock;

extern char _kernel_start[];
extern char _kernel_end[];

#define PAGE_BITMAP_STATIC_SIZE 1048576U
#define PAGE_BITMAP_CAPACITY_PAGES ((uint64_t)PAGE_BITMAP_STATIC_SIZE * 8ULL)
static uint8_t g_page_bitmap_static[PAGE_BITMAP_STATIC_SIZE] __attribute__((aligned(8)));

enum {
    EFI_LOADER_CODE        = 1,
    EFI_LOADER_DATA        = 2,
    EFI_BOOT_SERVICES_CODE = 3,
    EFI_BOOT_SERVICES_DATA = 4,
    EFI_CONVENTIONAL_MEMORY = 7
};

#define HEAP_MAGIC      0x1BADB002
#define HEAP_MAGIC_FREE 0x2BADB002

typedef struct __attribute__((aligned(16))) memory_block {
    uint32_t magic;
    uint64_t size;
    uint8_t  is_free;
    uint8_t  is_sensitive;
    uint16_t reserved;
    struct memory_block *prev;
    struct memory_block *next;
} memory_block_t;

_Static_assert((sizeof(memory_block_t) % 16u) == 0u,
               "heap metadata must preserve malloc alignment");

static memory_block_t *heap_start       = NULL;
static memory_block_t *heap_search_hint = NULL;
static uint32_t        heap_initialized = 0;
static uint64_t        used_memory      = 0;

#define HEAP_PAGE_COUNT_MIN  4096U
#define HEAP_PAGE_COUNT_MAX  32768U

static uint32_t heap_page_count = HEAP_PAGE_COUNT_MIN;

static spinlock_t heap_lock;
static spinlock_t page_lock;
static uint32_t   page_alloc_hint = 0;
static uint32_t   g_oom_total     = 0;
static uint32_t   g_oom_malloc    = 0;
static uint32_t   g_oom_pages     = 0;

#define MIN_ALLOC_ALIGN    16u 
#define MIN_SPLIT_REMAINDER 64u

static inline uint64_t align_up(uint64_t value, uint64_t align)
{
    return (value + align - 1ull) & ~(align - 1ull);
}

static inline uint64_t align_page_index_up(uint64_t value, uint32_t align_pages)
{
    uint64_t align = (align_pages == 0) ? 1ULL : (uint64_t)align_pages;
    uint64_t rem = value % align;
    return (rem == 0) ? value : (value + align - rem);
}

static inline uint8_t page_bitmap_get(uint64_t page_index)
{
    uint64_t byte_index = page_index >> 3;
    uint8_t bit_mask = (uint8_t)(1u << (page_index & 7u));
    return (uint8_t)((g_page_bitmap[byte_index] & bit_mask) != 0u);
}

static inline void page_bitmap_set(uint64_t page_index, uint8_t used)
{
    uint64_t byte_index = page_index >> 3;
    uint8_t bit_mask = (uint8_t)(1u << (page_index & 7u));
    if (used != 0u) {
        g_page_bitmap[byte_index] |= bit_mask;
    } else {
        g_page_bitmap[byte_index] &= (uint8_t)~bit_mask;
    }
}

/* Physical pages currently free. */
uint64_t memory_free_pages(void)
{
    if (g_page_bitmap == NULL || g_max_pages == 0) return 0;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&page_lock);
    uint64_t free_pages = 0;
    uint64_t words = (g_max_pages + 63ULL) >> 6;
    for (uint64_t w = 0; w < words; ++w) {
        /* SWAR popcount: __builtin_popcountll lowers to libgcc's
         * __popcountdi2, which a freestanding kernel does not link. */
        uint64_t v = ~((const uint64_t *)g_page_bitmap)[w];
        v = v - ((v >> 1) & 0x5555555555555555ULL);
        v = (v & 0x3333333333333333ULL) + ((v >> 2) & 0x3333333333333333ULL);
        v = (v + (v >> 4)) & 0x0F0F0F0F0F0F0F0FULL;
        free_pages += (v * 0x0101010101010101ULL) >> 56;
    }
    spinlock_unlock(&page_lock);
    irq_restore(irq_flags);
    return free_pages > g_max_pages ? g_max_pages : free_pages;
}

/* Running out of physical memory used to be completely silent: this counted
 * the event and threw the site and size away, so every caller that degraded on
 * a NULL page (execve building a new address space, for one) failed with no
 * trace at all. Report it, but only the first few times -- an OOM tends to
 * arrive in storms and the serial console is the only channel there is. */
static void memory_report_oom(const char *site, uint64_t request)
{
    ++g_oom_total;
    if (g_oom_total <= 8u) {
        serial_write_string("[mem] OOM at ");
        serial_write_string(site ? site : "?");
        serial_write_string(" request=");
        serial_write_uint64(request);
        serial_write_string(" free_pages=");
        serial_write_uint64(memory_free_pages());
        serial_write_string(" total_pages=");
        serial_write_uint64(g_max_pages);
        serial_write_char('\n');
    }
}

static memory_block_t *split_block_if_needed(memory_block_t *block, uint64_t size)
{
    if (block->size >= size + sizeof(memory_block_t) + MIN_SPLIT_REMAINDER) {
        memory_block_t *new_block =
            (memory_block_t *)((uint8_t *)block + sizeof(memory_block_t) + size);
        new_block->magic     = HEAP_MAGIC_FREE;
        new_block->size      = block->size - size - sizeof(memory_block_t);
        new_block->is_free   = 1;
        new_block->is_sensitive = 0;
        new_block->prev      = block;
        new_block->next      = block->next;
        if (new_block->next != NULL) {
            new_block->next->prev = new_block;
        }

        block->size = size;
        block->next = new_block;
    }
    return block;
}

static void *malloc_locked(uint64_t size)
{
    if (heap_search_hint == NULL) {
        heap_search_hint = heap_start;
    }

    memory_block_t *start   = heap_search_hint;
    memory_block_t *current = start;
    int wrapped = 0;

    while (current != NULL) {
        if (current->is_free && current->size >= size) {
            current = split_block_if_needed(current, size);
            current->is_free      = 0;
            current->is_sensitive = 0;
            current->magic        = HEAP_MAGIC;
            used_memory += current->size;
            heap_search_hint = (current->next != NULL) ? current->next : heap_start;
            return (void *)((uint8_t *)current + sizeof(memory_block_t));
        }

        current = current->next;
        if (current == NULL && !wrapped) {
            current = heap_start;
            wrapped = 1;
        }
        if (wrapped && current == start) {
            break;
        }
    }
    return NULL;
}

static void mark_pages(uint64_t start_page, uint64_t page_count, uint8_t used)
{
    if (g_page_bitmap == NULL || start_page >= g_max_pages) return;
    uint64_t end_page = start_page + page_count;
    if (end_page > g_max_pages) end_page = g_max_pages;
    for (uint64_t i = start_page; i < end_page; i++) {
        page_bitmap_set(i, used);
    }
}

static uint64_t page_bitmap_find_free_from_hint(uint64_t hint)
{
    if (g_page_bitmap == NULL || g_max_pages == 0) {
        return UINT64_MAX;
    }

    uint64_t total_words = (g_max_pages + 63ULL) >> 6;
    uint64_t start_word = hint >> 6;

    for (uint64_t pass = 0; pass < total_words; ++pass) {
        uint64_t word_index = (start_word + pass) % total_words;
        uint64_t base_page = word_index << 6;
        uint64_t word = ((const uint64_t *)g_page_bitmap)[word_index];
        if (word == UINT64_MAX) {
            continue;
        }

        if (word_index == start_word) {
            uint64_t start_bit = hint & 63ULL;
            if (start_bit != 0) {
                uint64_t low_mask = (1ULL << start_bit) - 1ULL;
                word |= low_mask;
            }
        }

        uint64_t free_bits = ~word;
        if (free_bits != 0ULL) {
            uint32_t bit = (uint32_t)__builtin_ctzll(free_bits);
            uint64_t page = base_page + (uint64_t)bit;
            if (page < g_max_pages) {
                return page;
            }
        }
    }

    return UINT64_MAX;
}

static int is_usable_memory_type(uint32_t type)
{
    return (type == EFI_CONVENTIONAL_MEMORY);
}

void init_physical_memory(void *memory_map, size_t map_size, size_t desc_size,
                          uint64_t detected_pages)
{
    uint64_t max_pages = detected_pages;

    if ((max_pages == 0) && memory_map != NULL && desc_size != 0) {
        uint8_t *map = (uint8_t *)memory_map;
        for (size_t offset = 0; offset + desc_size <= map_size; offset += desc_size) {
            typedef struct {
                uint32_t Type; uint32_t Pad;
                uint64_t PhysicalStart;
                uint64_t VirtualStart;
                uint64_t NumberOfPages;
                uint64_t Attribute;
            } EFI_MEM_DESC;
            EFI_MEM_DESC *desc = (EFI_MEM_DESC *)(map + offset);
            uint64_t end_page = (desc->PhysicalStart / PAGE_SIZE) + desc->NumberOfPages;
            if (end_page > max_pages) max_pages = end_page;
        }
    }

    if (max_pages == 0 || max_pages > MAX_PAGES_HARD_LIMIT) {
        max_pages = MAX_PAGES_HARD_LIMIT;
    }
    if (max_pages > PAGE_BITMAP_CAPACITY_PAGES) {
        max_pages = PAGE_BITMAP_CAPACITY_PAGES;
    }

    g_max_pages   = max_pages;
    g_page_bitmap = g_page_bitmap_static;

    memset(g_page_bitmap, 0xFF, PAGE_BITMAP_STATIC_SIZE);

    if (memory_map != NULL && desc_size != 0) {
        uint8_t *map = (uint8_t *)memory_map;
        for (size_t offset = 0; offset + desc_size <= map_size; offset += desc_size) {
            typedef struct {
                uint32_t Type; uint32_t Pad;
                uint64_t PhysicalStart;
                uint64_t VirtualStart;
                uint64_t NumberOfPages;
                uint64_t Attribute;
            } EFI_MEM_DESC;
            EFI_MEM_DESC *desc = (EFI_MEM_DESC *)(map + offset);
            if (is_usable_memory_type(desc->Type)) {
                uint64_t start_page = desc->PhysicalStart / PAGE_SIZE;
                mark_pages(start_page, desc->NumberOfPages, 0);
            }
        }
    }

    uint64_t kernel_start_page = ((uint64_t)(uintptr_t)_kernel_start) / PAGE_SIZE;
    uint64_t kernel_end_page   = (((uint64_t)(uintptr_t)_kernel_end) + PAGE_SIZE - 1) / PAGE_SIZE;
    mark_pages(kernel_start_page, kernel_end_page - kernel_start_page, 1);

    uint64_t free_pages = 0;
    for (uint64_t i = 0; i < g_max_pages; i++) {
        if (page_bitmap_get(i) == 0u) free_pages++;
    }
    
    if (g_max_pages > 0) {
        page_bitmap_set(0, 1u);
    }

    heap_page_count = HEAP_PAGE_COUNT_MIN;
    if (free_pages / 4 > (uint64_t)HEAP_PAGE_COUNT_MIN) {
        uint64_t candidate = free_pages / 4;
        if (candidate > (uint64_t)HEAP_PAGE_COUNT_MAX) candidate = (uint64_t)HEAP_PAGE_COUNT_MAX;
        heap_page_count = (uint32_t)candidate;
    }
}

void physical_memory_reserve_region(uint64_t base, uint64_t size)
{
    if (g_page_bitmap == NULL || size == 0) {
        return;
    }

    uint64_t start_page = base / PAGE_SIZE;
    uint64_t end_page = (base + size + PAGE_SIZE - 1ULL) / PAGE_SIZE;
    if (end_page <= start_page) {
        return;
    }

    mark_pages(start_page, end_page - start_page, 1);
}

void memory_init(void)
{
    if (heap_initialized) {
        return;
    }

    spinlock_init(&heap_lock);
    spinlock_init(&page_lock);

    heap_start = (memory_block_t *)alloc_contiguous_pages(heap_page_count, 1);
    if (heap_start == NULL) {
        if (heap_page_count > 16384U) {
            heap_page_count = 16384U;
            heap_start = (memory_block_t *)alloc_contiguous_pages(heap_page_count, 1);
        }
        if (heap_start == NULL && heap_page_count > 8192U) {
            heap_page_count = 8192U;
            heap_start = (memory_block_t *)alloc_contiguous_pages(heap_page_count, 1);
        }
        if (heap_start == NULL) {
            heap_page_count = HEAP_PAGE_COUNT_MIN;
            heap_start = (memory_block_t *)alloc_contiguous_pages(heap_page_count, 1);
            if (heap_start == NULL) {
                return;
            }
        }
    }
 
    heap_start->magic       = HEAP_MAGIC_FREE;
    heap_start->size        = ((uint64_t)heap_page_count * PAGE_SIZE) - sizeof(memory_block_t);
    heap_start->is_free     = 1;
    heap_start->is_sensitive = 0;
    heap_start->prev        = NULL;
    heap_start->next        = NULL;
    heap_search_hint        = heap_start;

    heap_initialized  = 1;
    used_memory       = 0;
    g_oom_total       = 0;
    g_oom_malloc      = 0;
    g_oom_pages       = 0;
    for (uint32_t i = 0; i < OS_CONFIG_SMP_MAX_CPUS; ++i) {
        g_alloc_page_recursion_depth[i] = 0;
    }
}

void *malloc(uint64_t size)
{
    if (!heap_initialized) return NULL;
    if (size == 0) return NULL;
    size = align_up(size, MIN_ALLOC_ALIGN);
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&heap_lock);
    void *ptr = malloc_locked(size);
    spinlock_unlock(&heap_lock);
    irq_restore(irq_flags);
    if (ptr != NULL) return ptr;
    ++g_oom_malloc;
    memory_report_oom("malloc", size);
    return NULL;
}

void free(void *ptr)
{
    if (ptr == NULL) return;
    if (!heap_initialized || heap_start == NULL) return;
    uintptr_t addr           = (uintptr_t)ptr;
    uintptr_t heap_start_addr = (uintptr_t)heap_start;
    uintptr_t heap_end_addr   = heap_start_addr + ((uint64_t)heap_page_count * PAGE_SIZE);
    if (addr < heap_start_addr + sizeof(memory_block_t) || addr >= heap_end_addr) return;
    memory_block_t *block = (memory_block_t *)(addr - sizeof(memory_block_t));
    if (block->magic != HEAP_MAGIC) return;
    if (block->is_free) return;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&heap_lock);
    block->is_free = 1;
    block->magic   = HEAP_MAGIC_FREE;
    used_memory   -= block->size;
    if (block->is_sensitive) {
        uint8_t *payload = (uint8_t *)ptr;
        for (uint64_t i = 0; i < block->size; i++) payload[i] = 0;
        block->is_sensitive = 0;
    }
    if (block->next != NULL && block->next->is_free) {
        block->size += sizeof(memory_block_t) + block->next->size;
        block->next  = block->next->next;
        if (block->next != NULL) {
            block->next->prev = block;
        }
    }
    if (block->prev != NULL && block->prev->is_free) {
        block->prev->size += sizeof(memory_block_t) + block->size;
        block->prev->next = block->next;
        if (block->next != NULL) {
            block->next->prev = block->prev;
        }
        block = block->prev;
    }
    heap_search_hint = block;
    spinlock_unlock(&heap_lock);
    irq_restore(irq_flags);
}

void *malloc_sensitive(uint64_t size) {
    void *ptr = malloc(size);
    if (ptr == NULL) return NULL;
    memory_block_t *block = (memory_block_t *)((uintptr_t)ptr - sizeof(memory_block_t));
    block->is_sensitive = 1;
    return ptr;
}

void free_sensitive(void *ptr) {
    if (ptr == NULL) return;
    memory_block_t *block = (memory_block_t *)((uintptr_t)ptr - sizeof(memory_block_t));
    if (block->magic == HEAP_MAGIC) block->is_sensitive = 1;
    free(ptr);
}

void *calloc(uint64_t num, uint64_t size) {
    if (num != 0 && size > UINT64_MAX / num) return NULL;
    uint64_t total_size = num * size;
    void *ptr = malloc(total_size);
    if (ptr != NULL) {
        memset(ptr, 0, total_size);
    }
    return ptr;
}

void *realloc(void *ptr, uint64_t new_size) {
    if (ptr == NULL) return malloc(new_size);
    if (new_size == 0) { free(ptr); return NULL; }
    new_size = align_up(new_size, MIN_ALLOC_ALIGN);
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&heap_lock);
    memory_block_t *block = (memory_block_t *)((uintptr_t)ptr - sizeof(memory_block_t));
    if (block->magic != HEAP_MAGIC || block->is_free) {
        spinlock_unlock(&heap_lock);
        irq_restore(irq_flags);
        return NULL;
    }
    uint64_t old_size    = block->size;
    uint8_t  was_sensitive = block->is_sensitive;
    if (new_size <= old_size) {
        split_block_if_needed(block, new_size);
        spinlock_unlock(&heap_lock);
        irq_restore(irq_flags);
        return ptr;
    }
    if (block->next != NULL && block->next->is_free && block->size + sizeof(memory_block_t) + block->next->size >= new_size) {
        uint64_t added = sizeof(memory_block_t) + block->next->size;
        block->size += added;
        block->next  = block->next->next;
        used_memory += added;
        split_block_if_needed(block, new_size);
        spinlock_unlock(&heap_lock);
        irq_restore(irq_flags);
        return ptr;
    }
    spinlock_unlock(&heap_lock);
    irq_restore(irq_flags);
    void *new_ptr = malloc(new_size);
    if (new_ptr == NULL) return NULL;
    memcpy(new_ptr, ptr, old_size);
    if (was_sensitive) {
        memory_block_t *nb = (memory_block_t *)((uintptr_t)new_ptr - sizeof(memory_block_t));
        nb->is_sensitive = 1;
    }
    free(ptr);
    return new_ptr;
}

uint64_t get_total_memory_pages(void) { return g_max_pages; }

uint64_t get_free_memory(void) {
    if (!heap_initialized) return 0;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&heap_lock);
    uint64_t free_memory = 0;
    memory_block_t *current = heap_start;
    while (current != NULL) {
        if (current->is_free) free_memory += current->size;
        current = current->next;
    }
    spinlock_unlock(&heap_lock);
    irq_restore(irq_flags);
    return free_memory;
}

uint64_t get_used_memory(void) {
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&heap_lock);
    uint64_t used = used_memory;
    spinlock_unlock(&heap_lock);
    irq_restore(irq_flags);
    return used;
}

void *alloc_page(void) {
    if (g_page_bitmap == NULL) return NULL;
    uint32_t cpu_id = smp_get_current_cpu_id();
    if (cpu_id >= OS_CONFIG_SMP_MAX_CPUS) cpu_id = 0;
    if (g_alloc_page_recursion_depth[cpu_id] >= MAX_ALLOC_PAGE_RECURSION_DEPTH) {
        ++g_oom_pages;
        memory_report_oom("alloc_page (recursion limit)", PAGE_SIZE);
        return NULL;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&page_lock);
    uint64_t page = page_bitmap_find_free_from_hint((uint64_t)page_alloc_hint);
    if (page != UINT64_MAX) {
        page_bitmap_set(page, 1u);
        page_alloc_hint  = (uint32_t)((page + 1ULL) % g_max_pages);
        spinlock_unlock(&page_lock);
        irq_restore(irq_flags);
        return (void *)(page * PAGE_SIZE);
    }
    spinlock_unlock(&page_lock);
    irq_restore(irq_flags);
    ++g_alloc_page_recursion_depth[cpu_id];
    int reclaim_result = paging_swap_reclaim_one_page();
    --g_alloc_page_recursion_depth[cpu_id];
    if (reclaim_result > 0) return alloc_page();
    ++g_oom_pages;
    memory_report_oom("alloc_page", PAGE_SIZE);
    return NULL;
}

void *alloc_contiguous_pages(uint32_t page_count, uint32_t align_pages) {
    if (g_page_bitmap == NULL || page_count == 0) {
        return NULL;
    }
    if ((uint64_t)page_count > g_max_pages) {
        return NULL;
    }
    uint32_t cpu_id = smp_get_current_cpu_id();
    if (cpu_id >= OS_CONFIG_SMP_MAX_CPUS) cpu_id = 0;
    if (g_alloc_page_recursion_depth[cpu_id] >= MAX_ALLOC_PAGE_RECURSION_DEPTH) {
        ++g_oom_pages;
        memory_report_oom("alloc_contiguous_pages (recursion limit)", (uint64_t)page_count * PAGE_SIZE);
        return NULL;
    }
    if (align_pages == 0) align_pages = 1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&page_lock);
    uint64_t limit = g_max_pages - (uint64_t)page_count;
    uint64_t start = align_page_index_up((uint64_t)page_alloc_hint, align_pages);
    if (start > limit) start = 0;
    uint64_t wrapped_limit = start;
    int wrapped = 0;
    while (start <= limit) {
        if (wrapped && start >= wrapped_limit) break;
        if (page_bitmap_get(start) != 0u) {
            uint64_t next = page_bitmap_find_free_from_hint(start + 1ULL);
            if (next == UINT64_MAX || next <= start) {
                if (wrapped) break;
                start = 0;
                wrapped = 1;
                continue;
            }
            start = align_page_index_up(next, align_pages);
            if (start > limit) {
                if (wrapped) break;
                start = 0;
                wrapped = 1;
            }
            continue;
        }
        uint32_t run = 1;
        while (run < page_count && page_bitmap_get(start + run) == 0u) ++run;
        if (run != page_count) { start += run; continue; }
        for (uint32_t i = 0; i < page_count; ++i) page_bitmap_set(start + i, 1u);
        page_alloc_hint = (uint32_t)((start + page_count) % g_max_pages);
        spinlock_unlock(&page_lock);
        irq_restore(irq_flags);
        return (void *)((uintptr_t)start * PAGE_SIZE);
    }
    spinlock_unlock(&page_lock);
    irq_restore(irq_flags);
    ++g_alloc_page_recursion_depth[cpu_id];
    int reclaim_result = paging_swap_reclaim_one_page();
    --g_alloc_page_recursion_depth[cpu_id];
    if (reclaim_result > 0) return alloc_contiguous_pages(page_count, align_pages);
    ++g_oom_pages;
    memory_report_oom("alloc_contiguous_pages", (uint64_t)page_count * PAGE_SIZE);
    return NULL;
}

void memory_init_page_refcounts(void)
{
    if (g_page_refcount != NULL || g_max_pages == 0) {
        return;
    }
    spinlock_init(&g_page_refcount_lock);
    uint64_t bytes = g_max_pages; /* one uint8 per frame */
    uint64_t need_pages = (bytes + PAGE_SIZE - 1ULL) / PAGE_SIZE;
    if (need_pages == 0 || need_pages > 0xFFFFFFFFULL) {
        return;
    }
    void *mem = alloc_contiguous_pages((uint32_t)need_pages, 1);
    if (mem == NULL) {
        serial_write_string("[mem] COW refcount table alloc failed; COW disabled\n");
        return;
    }
    memset(mem, 0, (size_t)(need_pages * PAGE_SIZE));
    g_page_refcount = (uint8_t *)mem;
    g_page_refcount_pages = g_max_pages;
}

void pmm_page_ref_inc(uint64_t phys_addr)
{
    if (g_page_refcount == NULL) return;
    uint64_t idx = phys_addr / PAGE_SIZE;
    if (idx >= g_page_refcount_pages) return;
    uint64_t f = irq_save_disable();
    spinlock_lock(&g_page_refcount_lock);
    uint8_t v = g_page_refcount[idx];
    if (v < 2u) {
        g_page_refcount[idx] = 2u; /* 0/1 (single owner) -> 2 (now shared) */
    } else if (v < 255u) {
        g_page_refcount[idx] = (uint8_t)(v + 1u);
    }
    spinlock_unlock(&g_page_refcount_lock);
    irq_restore(f);
}

uint32_t pmm_page_ref_dec(uint64_t phys_addr)
{
    if (g_page_refcount == NULL) return 0u;
    uint64_t idx = phys_addr / PAGE_SIZE;
    if (idx >= g_page_refcount_pages) return 0u;
    uint64_t f = irq_save_disable();
    spinlock_lock(&g_page_refcount_lock);
    uint8_t v = g_page_refcount[idx];
    uint32_t out;
    if (v <= 1u) {
        out = 0u;                     /* was single owner already */
    } else if (v == 255u) {
        out = 255u;                   /* saturated: never reclaim */
    } else {
        v = (uint8_t)(v - 1u);
        g_page_refcount[idx] = v;
        if (v == 1u) {
            g_page_refcount[idx] = 0u; /* back to single owner */
        }
        out = v;
    }
    spinlock_unlock(&g_page_refcount_lock);
    irq_restore(f);
    return out;
}

uint32_t pmm_page_ref_get(uint64_t phys_addr)
{
    if (g_page_refcount == NULL) return 0u;
    uint64_t idx = phys_addr / PAGE_SIZE;
    if (idx >= g_page_refcount_pages) return 0u;
    return g_page_refcount[idx];
}

void free_page(void *addr) {
    if (addr == NULL || g_page_bitmap == NULL) return;
    uint64_t page_num = (uint64_t)(uintptr_t)addr / PAGE_SIZE;
    /* If this frame is COW-shared by more than one address space, just drop a
     * reference and keep it allocated for the remaining owner(s). */
    if (g_page_refcount != NULL && page_num < g_page_refcount_pages) {
        uint64_t rf = irq_save_disable();
        spinlock_lock(&g_page_refcount_lock);
        uint8_t v = g_page_refcount[page_num];
        int keep = 0;
        if (v >= 2u) {
            if (v == 255u) {
                keep = 1;                 /* saturated: leak rather than corrupt */
            } else {
                v = (uint8_t)(v - 1u);
                g_page_refcount[page_num] = (v == 1u) ? 0u : v;
                keep = (v >= 1u);
            }
        }
        spinlock_unlock(&g_page_refcount_lock);
        irq_restore(rf);
        if (keep) {
            return;
        }
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&page_lock);
    if (page_num < g_max_pages) {
        page_bitmap_set(page_num, 0u);
        if (page_num < (uint64_t)page_alloc_hint) page_alloc_hint = (uint32_t)page_num;
    }
    spinlock_unlock(&page_lock);
    irq_restore(irq_flags);
}

void free_contiguous_pages(void *addr, uint32_t page_count) {
    if (addr == NULL || page_count == 0 || g_page_bitmap == NULL) return;
    uint64_t start_page = (uint64_t)(uintptr_t)addr / PAGE_SIZE;
    if (start_page >= g_max_pages) return;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&page_lock);
    uint64_t max_pages_left = g_max_pages - start_page;
    if ((uint64_t)page_count > max_pages_left) page_count = (uint32_t)max_pages_left;
    for (uint32_t i = 0; i < page_count; ++i) page_bitmap_set(start_page + i, 0u);
    if (start_page < (uint64_t)page_alloc_hint) page_alloc_hint = (uint32_t)start_page;
    spinlock_unlock(&page_lock);
    irq_restore(irq_flags);
}

void memory_dump_virtual(const void *addr, uint32_t bytes) {
    if (addr == NULL || bytes == 0) return;
    (void)addr; (void)bytes;
}

void memory_dump_physical(uint64_t phys_addr, uint32_t bytes) {
    if (bytes == 0) return;
    memory_dump_virtual((const void *)(uintptr_t)phys_addr, bytes);
}
