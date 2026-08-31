#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "Core/sync/Spinlock.h"
#include "DMA_Memory.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"

#ifndef DMA_POOL_SIZE
#define DMA_POOL_SIZE   (8u * 1024u * 1024u)
#endif
#define DMA_MIN_ALIGN       64u
#define DMA_MAX_BLOCKS      8192u
#define DMA_MAX_SEGMENTS    16u
#define DMA_GROWTH_CHUNK    (4u * 1024u * 1024u)
#ifndef VIRT_TO_PHYS_OFFSET
#define VIRT_TO_PHYS_OFFSET  0ULL
#endif

typedef struct {
    uint8_t *base_virt;
    uint64_t base_phys;
    size_t total;
    size_t offset;
    bool initialized;
} dma_segment_t;

typedef struct {
    uintptr_t virt;
    uint64_t  phys;
    size_t    size;
    uint16_t  segment_index;
    bool      used;
} dma_block_t;

typedef struct {
    dma_segment_t segments[DMA_MAX_SEGMENTS];
    dma_block_t   blocks[DMA_MAX_BLOCKS];
    uint32_t      segment_count;
    uint32_t      block_cnt;
    bool          initialized;
} dma_pool_t;

static dma_pool_t g_pool;
static spinlock_t g_lock;
static volatile uint32_t g_lock_initialized = 0;

static size_t dma_align_up(size_t value, size_t align)
{
    return (value + align - 1u) & ~(align - 1u);
}

static bool dma_add_segment_locked(size_t min_bytes)
{
    if (g_pool.segment_count >= DMA_MAX_SEGMENTS) {
        return false;
    }

    size_t target_size = DMA_POOL_SIZE;
    if (target_size < DMA_GROWTH_CHUNK) {
        target_size = DMA_GROWTH_CHUNK;
    }
    if (target_size < min_bytes) {
        target_size = dma_align_up(min_bytes, PAGE_SIZE);
    }

    size_t num_pages = (target_size + PAGE_SIZE - 1u) / PAGE_SIZE;
    void *virt = pmm_alloc_pages(num_pages);
    if (virt == NULL) {
        return false;
    }

    dma_segment_t *segment = &g_pool.segments[g_pool.segment_count++];
    segment->base_virt = (uint8_t *)virt;
    segment->base_phys = (uint64_t)(uintptr_t)virt + VIRT_TO_PHYS_OFFSET;
    segment->total = num_pages * PAGE_SIZE;
    segment->offset = 0;
    segment->initialized = true;
    g_pool.initialized = true;
    return true;
}

static bool dma_init_locked(void)
{
    if (g_pool.initialized) {
        return true;
    }
    memset(&g_pool, 0, sizeof(g_pool));
    return dma_add_segment_locked(DMA_POOL_SIZE);
}

bool dma_init(void)
{
    if (__atomic_exchange_n(&g_lock_initialized, 1u, __ATOMIC_ACQ_REL) == 0u) {
        spinlock_init(&g_lock);
    }
    spinlock_lock(&g_lock);
    bool ok = dma_init_locked();
    spinlock_unlock(&g_lock);
    return ok;
}

static int dma_block_compare(const void *lhs, const void *rhs)
{
    const dma_block_t *a = (const dma_block_t *)lhs;
    const dma_block_t *b = (const dma_block_t *)rhs;
    if (a->segment_index != b->segment_index) {
        return (a->segment_index < b->segment_index) ? -1 : 1;
    }
    if (a->virt < b->virt) {
        return -1;
    }
    if (a->virt > b->virt) {
        return 1;
    }
    return 0;
}

static void dma_coalesce_locked(void)
{
    if (g_pool.block_cnt < 2u) {
        return;
    }

    qsort(g_pool.blocks, g_pool.block_cnt, sizeof(g_pool.blocks[0]), dma_block_compare);

    uint32_t write_index = 0u;
    for (uint32_t read_index = 0u; read_index < g_pool.block_cnt; ++read_index) {
        dma_block_t current = g_pool.blocks[read_index];
        if (write_index == 0u) {
            g_pool.blocks[write_index++] = current;
            continue;
        }

        dma_block_t *prev = &g_pool.blocks[write_index - 1u];
        if (!prev->used && !current.used &&
            prev->segment_index == current.segment_index &&
            prev->virt + prev->size == current.virt) {
            prev->size += current.size;
            continue;
        }

        g_pool.blocks[write_index++] = current;
    }
    g_pool.block_cnt = write_index;
}

static dma_block_t *dma_find_reusable_block_locked(size_t aligned_bytes,
                                                   size_t need_align,
                                                   uint64_t max_address)
{
    dma_block_t *best = NULL;
    size_t best_diff = SIZE_MAX;

    for (uint32_t i = 0; i < g_pool.block_cnt; ++i) {
        dma_block_t *block = &g_pool.blocks[i];
        if (block->used || block->size < aligned_bytes) {
            continue;
        }
        if ((block->virt & (need_align - 1u)) != 0u) {
            continue;
        }
        if (max_address != 0u &&
            (block->phys > max_address ||
             aligned_bytes - 1u > max_address - block->phys)) {
            continue;
        }

        size_t diff = block->size - aligned_bytes;
        if (diff < best_diff) {
            best_diff = diff;
            best = block;
        }
    }

    return best;
}

static dma_block_t *dma_append_block_locked(uint16_t segment_index,
                                            uintptr_t virt,
                                            uint64_t phys,
                                            size_t aligned_bytes)
{
    if (g_pool.block_cnt >= DMA_MAX_BLOCKS) {
        return NULL;
    }

    dma_block_t *block = &g_pool.blocks[g_pool.block_cnt++];
    block->virt = virt;
    block->phys = phys;
    block->size = aligned_bytes;
    block->segment_index = segment_index;
    block->used = true;
    return block;
}

void *dma_alloc_ex(size_t bytes, size_t alignment, uint64_t max_address,
                   uint64_t *phys_out)
{
    if (bytes == 0u) {
        return NULL;
    }
    if (alignment < DMA_MIN_ALIGN) {
        alignment = DMA_MIN_ALIGN;
    }
    if ((alignment & (alignment - 1u)) != 0u) {
        return NULL;
    }

    spinlock_lock(&g_lock);

    if (!dma_init_locked()) {
        spinlock_unlock(&g_lock);
        return NULL;
    }

    size_t need_align = alignment;
    if (bytes >= (size_t)PAGE_SIZE && need_align < (size_t)PAGE_SIZE) {
        need_align = (size_t)PAGE_SIZE;
    }
    size_t aligned_bytes = dma_align_up(bytes, need_align);

    dma_block_t *reuse =
        dma_find_reusable_block_locked(aligned_bytes, need_align, max_address);
    if (reuse != NULL) {
        reuse->used = true;
        if (phys_out != NULL) {
            *phys_out = reuse->phys;
        }
        spinlock_unlock(&g_lock);
        memset((void *)reuse->virt, 0, reuse->size);
        return (void *)reuse->virt;
    }

    for (uint16_t segment_index = 0; segment_index < g_pool.segment_count; ++segment_index) {
        dma_segment_t *segment = &g_pool.segments[segment_index];
        size_t aligned_offset = dma_align_up(segment->offset, need_align);
        if (aligned_offset + aligned_bytes > segment->total) {
            continue;
        }

        uintptr_t virt = (uintptr_t)(segment->base_virt + aligned_offset);
        uint64_t phys = segment->base_phys + aligned_offset;
        if (max_address != 0u &&
            (phys > max_address ||
             aligned_bytes - 1u > max_address - phys)) {
            continue;
        }
        dma_block_t *block = dma_append_block_locked(segment_index, virt, phys, aligned_bytes);
        if (block == NULL) {
            spinlock_unlock(&g_lock);
            return NULL;
        }

        segment->offset = aligned_offset + aligned_bytes;
        if (phys_out != NULL) {
            *phys_out = phys;
        }
        spinlock_unlock(&g_lock);
        memset((void *)virt, 0, aligned_bytes);
        return (void *)virt;
    }

    if (!dma_add_segment_locked(aligned_bytes)) {
        spinlock_unlock(&g_lock);
        return NULL;
    }

    dma_segment_t *segment = &g_pool.segments[g_pool.segment_count - 1u];
    size_t aligned_offset = dma_align_up(segment->offset, need_align);
    uintptr_t virt = (uintptr_t)(segment->base_virt + aligned_offset);
    uint64_t phys = segment->base_phys + aligned_offset;
    if (aligned_offset + aligned_bytes > segment->total ||
        (max_address != 0u &&
         (phys > max_address ||
          aligned_bytes - 1u > max_address - phys))) {
        spinlock_unlock(&g_lock);
        return NULL;
    }
    dma_block_t *block = dma_append_block_locked((uint16_t)(g_pool.segment_count - 1u),
                                                 virt,
                                                 phys,
                                                 aligned_bytes);
    if (block == NULL) {
        spinlock_unlock(&g_lock);
        return NULL;
    }

    segment->offset = aligned_offset + aligned_bytes;
    if (phys_out != NULL) {
        *phys_out = phys;
    }
    spinlock_unlock(&g_lock);
    memset((void *)virt, 0, aligned_bytes);
    return (void *)virt;
}

void *dma_alloc(size_t bytes, uint64_t *phys_out)
{
    size_t alignment =
        (bytes >= (size_t)PAGE_SIZE) ? (size_t)PAGE_SIZE : (size_t)DMA_MIN_ALIGN;
    return dma_alloc_ex(bytes, alignment, 0u, phys_out);
}

void dma_free(void *virt, size_t bytes)
{
    if (virt == NULL) {
        return;
    }

    spinlock_lock(&g_lock);
    for (uint32_t i = 0; i < g_pool.block_cnt; ++i) {
        dma_block_t *block = &g_pool.blocks[i];
        if (block->virt != (uintptr_t)virt || !block->used) {
            continue;
        }
        if (bytes != 0u && bytes > block->size) {
            spinlock_unlock(&g_lock);
            return;
        }

        uintptr_t zero_virt = block->virt;
        size_t zero_size = block->size;
        block->used = false;
        dma_coalesce_locked();
        spinlock_unlock(&g_lock);
        memset((void *)zero_virt, 0, zero_size);
        return;
    }
    spinlock_unlock(&g_lock);
}

uint64_t virt_to_phys(void *virt)
{
    if (virt == NULL) {
        return 0;
    }

    uintptr_t virt_addr = (uintptr_t)virt;
    for (uint32_t i = 0; i < g_pool.segment_count; ++i) {
        dma_segment_t *segment = &g_pool.segments[i];
        uintptr_t base = (uintptr_t)segment->base_virt;
        if (virt_addr >= base && virt_addr < base + segment->total) {
            return segment->base_phys + (virt_addr - base);
        }
    }

    return virt_addr + VIRT_TO_PHYS_OFFSET;
}

void dma_dump_stats(void)
{
    if (!g_pool.initialized) {
        return;
    }

    spinlock_lock(&g_lock);
    uint32_t used_blocks = 0;
    uint32_t free_blocks = 0;
    size_t used_bytes = 0;
    size_t free_bytes = 0;

    for (uint32_t i = 0; i < g_pool.block_cnt; ++i) {
        dma_block_t *block = &g_pool.blocks[i];
        if (block->used) {
            ++used_blocks;
            used_bytes += block->size;
        } else {
            ++free_blocks;
            free_bytes += block->size;
        }
    }

    (void)used_blocks;
    (void)free_blocks;
    (void)used_bytes;
    (void)free_bytes;
    spinlock_unlock(&g_lock);
}
