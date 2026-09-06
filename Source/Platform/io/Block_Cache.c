#include "Block_Cache.h"

#include <stddef.h>
#include <string.h>

#include "Core/sync/Spinlock.h"
#include "MemoryManagement/Memory_Main.h"
#include "kernel/config.h"

#define BLOCK_CACHE_WAYS      4u
#define BLOCK_CACHE_MIN_LINES 512u    /*  2 MiB */
#define BLOCK_CACHE_MAX_LINES 8192u   /* 32 MiB */

typedef struct {
    uint8_t *data;      /* NULL until this line has been filled once */
    uint64_t tag;       /* line index = lba / BLOCK_CACHE_LINE_SECTORS */
    uint64_t device;    /* opaque key from the caller; see Block_Cache.h */
    uint8_t  valid;
} block_cache_line_t;

static block_cache_line_t *g_lines;
static uint32_t            g_line_count;
static uint32_t            g_set_count;
static uint8_t            *g_victim;    /* round-robin cursor per set */
static spinlock_t          g_cache_lock;
static uint64_t            g_hits;
static uint64_t            g_misses;
/* Lines that have been given a page. Once this reaches g_line_count no fill
 * can ever need to allocate again, which is the steady state. */
static uint32_t            g_lines_with_page;

/* Sequential read-ahead. A miss whose line is exactly where the previous fill
 * stopped is part of a sweep -- a directory being scanned record by record, a
 * file being paged in -- and on this hardware the command turnaround dominates
 * the transfer, so pulling the full run costs almost nothing and saves the
 * next several commands outright. A miss anywhere else only fills what the
 * request needs, so genuinely random access does not drag 32 KiB per sector.
 * Tracked per device; the table is tiny and a wrong guess is only ever a
 * performance question, never a correctness one. */
#define BLOCK_CACHE_MAX_DEVICES 8u
static uint64_t            g_readahead_key[BLOCK_CACHE_MAX_DEVICES];
static uint64_t            g_readahead_next_tag[BLOCK_CACHE_MAX_DEVICES];
static uint8_t             g_readahead_valid[BLOCK_CACHE_MAX_DEVICES];

/* Hashed into a slot rather than indexed: the key is opaque and sparse. A
 * collision only mispredicts read-ahead, so the slot is confirmed against the
 * stored key before it is trusted. */
static inline uint32_t readahead_slot(uint64_t key)
{
    return (uint32_t)((key ^ (key >> 32)) % BLOCK_CACHE_MAX_DEVICES);
}

/* How many lines one miss pulls in a single transfer. 8 lines = 32 KiB, which
 * is exactly the AHCI DMA window (AHCI_DMA_SECTORS) and 16 ATAPI blocks, so a
 * run of this size is one command on every transport here. Filling a run
 * rather than a line is what keeps the mmap read-ahead path (64 KiB at a
 * time) down to two commands instead of sixteen. */
#define BLOCK_CACHE_FILL_MAX_LINES 8u

/* Staging for a line fill. Not on the stack: the kernel stack is 32 KiB
 * (Arch/x86_64/linker/linker.ld) and this sits at the bottom of a
 * syscall -> VFS -> filesystem driver -> block layer chain. Safe as a single
 * shared buffer because disk_read() serialises the whole block layer on
 * g_disk_lock before calling in here. */
static uint8_t             g_staging[BLOCK_CACHE_FILL_MAX_LINES *
                                      BLOCK_CACHE_LINE_BYTES];

static inline uint32_t set_of(uint64_t tag)
{
    return (uint32_t)(tag % g_set_count);
}

void block_cache_flush(void)
{
    if (g_lines == NULL) {
        return;
    }
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_cache_lock);
    for (uint32_t i = 0; i < g_line_count; ++i) {
        g_lines[i].valid = 0u;
    }
    for (uint32_t i = 0; i < BLOCK_CACHE_MAX_DEVICES; ++i) {
        g_readahead_valid[i] = 0u;
    }
    spinlock_unlock(&g_cache_lock);
    irq_restore(irq);
}

void block_cache_init(void)
{
#if defined(OS_CONFIG_BLOCK_CACHE) && OS_CONFIG_BLOCK_CACHE == 0
    return; /* every read falls through to the medium */
#else
    if (g_lines != NULL) {
        block_cache_flush();
        return;
    }

    spinlock_init(&g_cache_lock);

    /* Take a 64th of physical memory, clamped. On the 4 GiB reference QEMU
     * machine that is 16 MiB; a 512 MiB machine gets the 2 MiB floor, which
     * still holds every directory the loader walks. */
    uint64_t pages = get_total_memory_pages();
    uint64_t want  = pages / 64u;
    if (want < BLOCK_CACHE_MIN_LINES) want = BLOCK_CACHE_MIN_LINES;
    if (want > BLOCK_CACHE_MAX_LINES) want = BLOCK_CACHE_MAX_LINES;

    /* Round down to a whole number of sets. */
    uint32_t lines = (uint32_t)(want / BLOCK_CACHE_WAYS) * BLOCK_CACHE_WAYS;
    if (lines == 0u) {
        return;
    }

    block_cache_line_t *table =
        (block_cache_line_t *)calloc(lines, sizeof(block_cache_line_t));
    if (table == NULL) {
        return; /* stays disabled: every caller falls through to a real read */
    }
    uint8_t *victim = (uint8_t *)calloc(lines / BLOCK_CACHE_WAYS, 1u);
    if (victim == NULL) {
        free(table);
        return;
    }

    g_lines      = table;
    g_line_count = lines;
    g_set_count  = lines / BLOCK_CACHE_WAYS;
    g_victim     = victim;
    g_hits       = 0u;
    g_misses     = 0u;
#endif
}

void block_cache_stats(uint64_t *out_hits, uint64_t *out_misses,
                       uint32_t *out_lines)
{
    if (out_hits)   *out_hits   = g_hits;
    if (out_misses) *out_misses = g_misses;
    if (out_lines)  *out_lines  = g_line_count;
}

/* Caller holds the cache lock. Returns the line holding `tag`, or NULL. */
static block_cache_line_t *lookup_locked(uint64_t device, uint64_t tag)
{
    block_cache_line_t *set = &g_lines[(uint64_t)set_of(tag) * BLOCK_CACHE_WAYS];
    for (uint32_t w = 0; w < BLOCK_CACHE_WAYS; ++w) {
        if (set[w].valid && set[w].tag == tag && set[w].device == device) {
            return &set[w];
        }
    }
    return NULL;
}

/* Caller holds the cache lock. Picks a line to reuse and points it at `tag`,
 * leaving it invalid; the caller fills it and sets valid.
 *
 * `spare` is a page the caller allocated *outside* the lock, used only if the
 * chosen line has never had one. It is deliberately not allocated here:
 * alloc_page() can fall through to paging_swap_reclaim_one_page(), and this
 * runs underneath g_disk_lock (disk_read holds it), so an allocator that ever
 * reclaims by writing to the disc would deadlock against the very read that
 * triggered it. Returns NULL when the line needs a page and `spare` is NULL;
 * *spare_used says whether the page was taken. */
static block_cache_line_t *claim_locked(uint64_t device, uint64_t tag,
                                        uint8_t *spare, int *spare_used)
{
    uint32_t s = set_of(tag);
    block_cache_line_t *set = &g_lines[(uint64_t)s * BLOCK_CACHE_WAYS];

    *spare_used = 0;

    /* Prefer a way that is free. */
    block_cache_line_t *victim = NULL;
    for (uint32_t w = 0; w < BLOCK_CACHE_WAYS; ++w) {
        if (!set[w].valid) {
            victim = &set[w];
            break;
        }
    }
    if (victim == NULL) {
        victim = &set[g_victim[s] % BLOCK_CACHE_WAYS];
        g_victim[s] = (uint8_t)((g_victim[s] + 1u) % BLOCK_CACHE_WAYS);
    }

    if (victim->data == NULL) {
        if (spare == NULL) {
            return NULL;
        }
        victim->data = spare;
        *spare_used = 1;
        ++g_lines_with_page;
    }
    victim->valid  = 0u;
    victim->tag    = tag;
    victim->device = device;
    return victim;
}

void block_cache_invalidate(uint64_t device, uint64_t lba, uint32_t sectors)
{
    if (g_lines == NULL || sectors == 0u) {
        return;
    }
    uint64_t first = lba / BLOCK_CACHE_LINE_SECTORS;
    uint64_t last  = (lba + sectors - 1u) / BLOCK_CACHE_LINE_SECTORS;

    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_cache_lock);
    for (uint64_t tag = first; tag <= last; ++tag) {
        block_cache_line_t *line = lookup_locked(device, tag);
        if (line != NULL) {
            line->valid = 0u;
        }
    }
    spinlock_unlock(&g_cache_lock);
    irq_restore(irq);
}

bool block_cache_read(uint64_t device, uint64_t lba, uint8_t *buffer,
                      uint32_t sectors,
                      bool (*fill)(uint64_t lba, uint8_t *buf, uint32_t n))
{
    if (fill == NULL || buffer == NULL) {
        return false;
    }
    if (sectors == 0u) {
        return true;
    }
    if (g_lines == NULL || sectors >= BLOCK_CACHE_BYPASS_SECTORS) {
        return fill(lba, buffer, sectors);
    }

    uint64_t tag = lba / BLOCK_CACHE_LINE_SECTORS;
    uint32_t done = 0u;

    while (done < sectors) {
        uint32_t offset = (uint32_t)((lba + done) % BLOCK_CACHE_LINE_SECTORS);
        uint32_t take   = BLOCK_CACHE_LINE_SECTORS - offset;
        if (take > sectors - done) {
            take = sectors - done;
        }
        tag = (lba + done) / BLOCK_CACHE_LINE_SECTORS;

        uint64_t irq = irq_save_disable();
        spinlock_lock(&g_cache_lock);
        block_cache_line_t *line = lookup_locked(device, tag);
        if (line != NULL) {
            memcpy(buffer + (uint64_t)done * 512u,
                   line->data + (uint64_t)offset * 512u,
                   (size_t)take * 512u);
            ++g_hits;
            spinlock_unlock(&g_cache_lock);
            irq_restore(irq);
            done += take;
            continue;
        }
        spinlock_unlock(&g_cache_lock);
        irq_restore(irq);

        /* Miss. Pull an aligned *run* of lines, not just the one: reading
         * the sectors around the one actually asked for is what makes the
         * next record in the same directory -- and the next page of a
         * mapped shared object -- free. The run stops at the end of the
         * request, at the fill cap, or at a line that is already cached. */
        uint32_t run = 1u;
        {
            uint32_t ra = readahead_slot(device);
            int sequential = g_readahead_valid[ra] &&
                             g_readahead_key[ra] == device &&
                             g_readahead_next_tag[ra] == tag;

            uint64_t irq2 = irq_save_disable();
            spinlock_lock(&g_cache_lock);
            while (run < BLOCK_CACHE_FILL_MAX_LINES &&
                   (sequential ||
                    (uint64_t)(done + take) +
                        (uint64_t)(run - 1u) * BLOCK_CACHE_LINE_SECTORS < sectors) &&
                   lookup_locked(device, tag + run) == NULL) {
                ++run;
            }
            spinlock_unlock(&g_cache_lock);
            irq_restore(irq2);
        }

        ++g_misses;
        uint64_t line_lba = tag * BLOCK_CACHE_LINE_SECTORS;
        uint32_t run_sectors = run * BLOCK_CACHE_LINE_SECTORS;
        if (!fill(line_lba, g_staging, run_sectors)) {
            /* Past the end of the medium, or a transport that refuses the
             * wider request: fall back to exactly what was asked for and do
             * not cache it. */
            if (!fill(lba + done, buffer + (uint64_t)done * 512u, take)) {
                return false;
            }
            done += take;
            continue;
        }

        /* Hand back everything of this request the run covers. */
        uint32_t copied = 0u;
        {
            uint32_t avail = run_sectors - offset;
            copied = (avail < sectors - done) ? avail : (sectors - done);
            memcpy(buffer + (uint64_t)done * 512u,
                   g_staging + (uint64_t)offset * 512u,
                   (size_t)copied * 512u);
        }

        /* Pages for lines that have never held one, taken before the lock.
         * Once the cache is warm g_lines_with_page == g_line_count and this
         * whole step disappears. */
        uint8_t *spares[BLOCK_CACHE_FILL_MAX_LINES] = { 0 };
        if (g_lines_with_page < g_line_count) {
            for (uint32_t i = 0; i < run; ++i) {
                spares[i] = (uint8_t *)alloc_page();
            }
        }

        uint64_t irq3 = irq_save_disable();
        spinlock_lock(&g_cache_lock);
        for (uint32_t i = 0; i < run; ++i) {
            /* Another CPU may have filled a line while the read was in
             * flight; either copy is equally good, so just install ours. */
            int spare_used = 0;
            block_cache_line_t *slot =
                claim_locked(device, tag + i, spares[i], &spare_used);
            if (spare_used) {
                spares[i] = NULL;
            }
            if (slot == NULL) {
                continue; /* out of pages: this line simply stays uncached */
            }
            memcpy(slot->data,
                   g_staging + (uint64_t)i * BLOCK_CACHE_LINE_BYTES,
                   BLOCK_CACHE_LINE_BYTES);
            slot->valid = 1u;
        }
        spinlock_unlock(&g_cache_lock);
        irq_restore(irq3);

        for (uint32_t i = 0; i < run; ++i) {
            if (spares[i] != NULL) {
                free_page(spares[i]);
            }
        }

        {
            uint32_t ra = readahead_slot(device);
            g_readahead_key[ra] = device;
            g_readahead_next_tag[ra] = tag + run;
            g_readahead_valid[ra] = 1u;
        }

        done += copied;
        continue;
    }

    return true;
}