#include "FileMap.h"

#include "Core/syscall/Syscall_File.h"
#include "Core/sync/Spinlock.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "interfaces/hal_cpu.h"
#include "Core/process/ProcessManager.h"
#include "Debug/serial/Serial.h"
#include "Core/timer/Timer.h"

#include <stddef.h>
#include <string.h>

/*
 * How many lazy file mappings can exist system-wide at once.
 *
 * Only mappings at or above FILEMAP_MIN_BYTES are registered here (see
 * linux_mmap), so this counts big library images rather than every segment
 * glibc's loader lays down. Xorg pulls in ~180 shared objects but only a
 * couple of dozen are large enough to qualify, so a shared pool this size has
 * ample headroom without giving every process a multi-kilobyte table.
 */
/* One entry per file-backed mapping, for every process at once. A single
 * dynamically linked GTK3 client registers ~290 of them (about 60 shared
 * objects, each contributing the loader's whole-object reservation plus one
 * record per PT_LOAD segment plus a zero-fill record for its .bss), and the
 * compositor and window manager hold entries at the same time. At 256 the
 * table filled part-way through gtk3-demo's startup and every later mapping
 * was dropped without a word -- those pages then fell through to the
 * demand-zero path, so a library's data read back as zeros. That is silent,
 * and it surfaces far away: GTK found its icons in libgtk-3.so's embedded
 * GResource but the bytes were zeros, and it aborted with "Unrecognized
 * image file format". */
/* 2048: once the table is full a big file mapping is read in whole instead
 * (see linux_mmap), which for a translated Chromium image is ~350 MB per
 * process -- exactly how memory ran out. Xorg x3 + Wine + Chromium peak
 * near 620 records, and every further Chromium process adds ~30. The scan
 * below walks g_filemap_owner1 (4 bytes a slot) and touches a record only
 * when its owner matches, so the larger table costs little per fault. */
#define FILEMAP_MAX 2048

/* Pages faulted in per file-backed fault. See filemap_handle_fault(). */
/* 64 pages (256 KiB) per fault. Sixteen was chosen when concurrent block reads
 * overlapped -- which they did by corrupting each other, since the USB
 * transport has one bounce buffer and no locking. Serialising those reads (see
 * usb_storage_lock() in Drivers/Bus/USB/USB_Main.c) made the per-command cost
 * matter, so fetch more per command. pmm_alloc_pages() halves the run down
 * when a contiguous one that big is not available. */
#define FILEMAP_READAHEAD_PAGES 64

typedef struct {
    uint8_t  used;
    int32_t  owner_pid;
    uint64_t start;        /* page-aligned user VA */
    uint64_t length;       /* page-aligned byte count */
    int32_t  file_handle;  /* open-file reference, or -1 for a zero-fill hole */
    uint64_t file_offset;  /* page-aligned offset of `start` within the file */
    uint64_t page_flags;   /* PAGE_* bits to install on a faulted-in page */
    uint64_t seq;          /* registration order; higher wins on overlap */
} filemap_t;

static filemap_t  g_filemaps[FILEMAP_MAX];
/* Owner pid + 1 of each slot, 0 when unused -- the field every scan filters
 * on first, kept apart from the records so a scan reads 8 KiB, not 112. */
static uint32_t   g_filemap_owner1[FILEMAP_MAX];
static spinlock_t g_filemap_lock;
/* Serialises the page-table installation at the end of filemap_handle_fault();
 * separate from g_filemap_lock, which only guards the registration table and
 * must not be held across the file read. */
static spinlock_t g_filemap_fault_lock;
static uint8_t    g_filemap_ready;
static uint64_t   g_filemap_seq;

static void filemap_init_once(void)
{
    if (g_filemap_ready == 0u) {
        spinlock_init(&g_filemap_lock);
        g_filemap_ready = 1u;
    }
}

/* ---- Shared page cache ----------------------------------------------------
 *
 * Without it every process reads its own copy of every file page it touches:
 * three Xorg servers hold three copies of libLLVM's hot pages, and each of a
 * dozen Chromium processes (each exec'd one, and every renderer the zygote
 * forks for the pages it reaches after the fork) its own copy of the
 * browser's 350 MB image. With Xorg, Wine and Chromium up together that
 * duplication ran an 8 GiB arm64 machine out of memory.
 *
 * A file on read-only media cannot change under a mapping, so one physical
 * copy of each of its pages can serve every private mapping of it: the
 * frame is entered here once, keyed on (mount, file id, page), and mapped
 * into each address space read-only + copy-on-write with the frame's
 * refcount counting the cache and every mapping. A write gets a private
 * copy (paging_handle_cow_fault: refcount > 1), so the cached frame never
 * changes. A frame only the cache still holds (refcount <= 1) is reclaimed
 * when a process exits or execs, and whenever a fault runs out of memory
 * (filemap_reclaim_shared_pages()).
 *
 * arm64 only for now: a kernel write to a user page there faults on the
 * read-only descriptor like a user write and gets its private copy. x86_64
 * runs with CR0.WP clear, so a kernel write that skipped
 * paging_user_range_break_cow() would land in the frame every process
 * shares; that needs an audit before it is turned on there. */
#ifndef FILEMAP_SHARE_PAGES
#if defined(PLATFORM_ARM64)
#define FILEMAP_SHARE_PAGES 1
#else
#define FILEMAP_SHARE_PAGES 0
#endif
#endif

/* 256 Ki entries (8 MiB, allocated on first use), filled to at most 3/4:
 * up to 768 MiB of shared file pages. When it is full a fault falls back to
 * a private page, as before. */
#define FMC_BITS 18u
#define FMC_SIZE (1u << FMC_BITS)
#define FMC_MASK (FMC_SIZE - 1u)
#define FMC_LIMIT (FMC_SIZE / 4u * 3u)

typedef struct {
    uint64_t mount;     /* 0: empty slot */
    uint64_t id;
    uint64_t page;      /* page index within the file */
    uint64_t phys;
} fmc_entry_t;

static fmc_entry_t *g_fmc;
static uint32_t     g_fmc_used;
/* Statistics for the memory report. */
static uint64_t     g_fmc_stat_hits, g_fmc_stat_inserts, g_fmc_stat_full;
static uint64_t     g_fmc_stat_sweeps, g_fmc_stat_sweep_ns, g_fmc_stat_freed;
static uint64_t     g_fm_stat_faults, g_fm_stat_read_pages, g_fm_stat_read_ns;
static uint8_t      g_fmc_failed;
static spinlock_t   g_fmc_lock;

static inline uint32_t fmc_hash(uint64_t mount, uint64_t id, uint64_t page)
{
    uint64_t h = mount * 0x9E3779B97F4A7C15ull ^ id * 0xC2B2AE3D27D4EB4Full ^
                 page * 0x165667B19E3779F9ull;
    h ^= h >> 29;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 32;
    return (uint32_t)h & FMC_MASK;
}

/* Caller holds g_fmc_lock. Linear probing: the slot holding the key, or the
 * empty slot ending its probe sequence. */
static uint32_t fmc_slot_locked(uint64_t mount, uint64_t id, uint64_t page)
{
    uint32_t i = fmc_hash(mount, id, page);
    while (g_fmc[i].mount != 0u &&
           (g_fmc[i].mount != mount || g_fmc[i].id != id || g_fmc[i].page != page)) {
        i = (i + 1u) & FMC_MASK;
    }
    return i;
}

/* Caller holds g_fmc_lock. Backward-shift deletion keeps every remaining
 * key reachable from its home slot without tombstones. */
static void fmc_delete_locked(uint32_t i)
{
    uint32_t j = i;
    for (;;) {
        j = (j + 1u) & FMC_MASK;
        if (g_fmc[j].mount == 0u) {
            break;
        }
        const uint32_t k = fmc_hash(g_fmc[j].mount, g_fmc[j].id, g_fmc[j].page);
        /* Entry j may move into the hole at i unless its home k lies
         * cyclically in (i, j]. */
        const int stays = (i <= j) ? (k > i && k <= j) : (k > i || k <= j);
        if (!stays) {
            g_fmc[i] = g_fmc[j];
            i = j;
        }
    }
    g_fmc[i].mount = 0u;
    --g_fmc_used;
}

/* Caller holds g_fmc_lock. Frees every frame no mapping holds any more. A
 * new reference is only ever taken under this lock (fmc_get/fmc_put), and a
 * frame with no mapping cannot gain one through fork, so the refcount read
 * here cannot be overtaken. */
static uint64_t fmc_reclaim_locked(void)
{
    const uint64_t t0 = timer_monotonic_ns();
    uint64_t freed = 0;
    for (uint32_t i = 0; i < FMC_SIZE;) {
        if (g_fmc[i].mount != 0u && pmm_page_ref_get(g_fmc[i].phys) <= 1u) {
            free_page((void *)(uintptr_t)g_fmc[i].phys);
            fmc_delete_locked(i);
            ++freed;
            continue;           /* a later entry may have moved into slot i */
        }
        ++i;
    }
    ++g_fmc_stat_sweeps;
    g_fmc_stat_freed += freed;
    g_fmc_stat_sweep_ns += timer_monotonic_ns() - t0;
    return freed;
}

static int fmc_ready(void)
{
    if (g_fmc != NULL) {
        return 1;
    }
    if (g_fmc_failed) {
        return 0;
    }
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_fmc_lock);
    if (g_fmc == NULL && !g_fmc_failed) {
        const uint32_t pages = (uint32_t)((FMC_SIZE * sizeof(fmc_entry_t)) / PAGE_SIZE);
        fmc_entry_t *t = (fmc_entry_t *)alloc_contiguous_pages(pages, 1u);
        if (t == NULL) {
            g_fmc_failed = 1u;
        } else {
            memset(t, 0, (size_t)pages * PAGE_SIZE);
            __atomic_store_n(&g_fmc, t, __ATOMIC_RELEASE);
        }
    }
    spinlock_unlock(&g_fmc_lock);
    irq_restore(irq);
    return g_fmc != NULL;
}

/* The cached frame for a file page with one more reference taken for the
 * caller's mapping, or 0. */
static uint64_t fmc_get(uint64_t mount, uint64_t id, uint64_t page)
{
    uint64_t phys = 0;
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_fmc_lock);
    const uint32_t i = fmc_slot_locked(mount, id, page);
    if (g_fmc[i].mount != 0u) {
        phys = g_fmc[i].phys;
        pmm_page_ref_inc(phys);
        ++g_fmc_stat_hits;
    }
    spinlock_unlock(&g_fmc_lock);
    irq_restore(irq);
    return phys;
}

/* Offer `frame` (freshly read, owned by the caller) as the cached copy of a
 * file page. Returns the frame the caller should map, with a reference
 * taken for that mapping: `frame` itself once the cache has adopted it, or
 * the copy another CPU entered first (then `frame` is the caller's to free).
 * 0 when the table is full: map `frame` privately. */
static uint64_t fmc_put(uint64_t mount, uint64_t id, uint64_t page, uint64_t frame)
{
    uint64_t out = 0;
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_fmc_lock);
    uint32_t i = fmc_slot_locked(mount, id, page);
    if (g_fmc[i].mount == 0u && g_fmc_used >= FMC_LIMIT &&
        fmc_reclaim_locked() > 0u) {
        i = fmc_slot_locked(mount, id, page);
    }
    if (g_fmc[i].mount != 0u) {
        out = g_fmc[i].phys;
        pmm_page_ref_inc(out);
    } else if (g_fmc_used < FMC_LIMIT) {
        g_fmc[i].mount = mount;
        g_fmc[i].id = id;
        g_fmc[i].page = page;
        g_fmc[i].phys = frame;
        ++g_fmc_used;
        pmm_page_ref_inc(frame);    /* the cache's reference + the mapping's */
        out = frame;
        ++g_fmc_stat_inserts;
    } else {
        ++g_fmc_stat_full;
    }
    spinlock_unlock(&g_fmc_lock);
    irq_restore(irq);
    return out;
}

uint64_t filemap_reclaim_shared_pages(void)
{
    if (g_fmc == NULL) {
        return 0;
    }
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_fmc_lock);
    const uint64_t freed = fmc_reclaim_locked();
    spinlock_unlock(&g_fmc_lock);
    irq_restore(irq);
    return freed;
}

void filemap_debug_read_stats(uint64_t out[3])
{
    out[0] = g_fm_stat_faults;
    out[1] = g_fm_stat_read_pages;
    out[2] = g_fm_stat_read_ns;
}

void filemap_debug_cache_stats(uint64_t out[6])
{
    out[0] = g_fmc_stat_hits;
    out[1] = g_fmc_stat_inserts;
    out[2] = g_fmc_stat_full;
    out[3] = g_fmc_stat_sweeps;
    out[4] = g_fmc_stat_sweep_ns;
    out[5] = g_fmc_stat_freed;
}

uint32_t filemap_debug_shared_pages(uint32_t *unmapped_out)
{
    uint32_t used = 0, unmapped = 0;
    if (g_fmc != NULL) {
        for (uint32_t i = 0; i < FMC_SIZE; ++i) {
            if (g_fmc[i].mount == 0u) continue;
            ++used;
            if (pmm_page_ref_get(g_fmc[i].phys) <= 1u) ++unmapped;
        }
    }
    if (unmapped_out != NULL) *unmapped_out = unmapped;
    return used;
}

/* Install frame `phys` (one reference already taken for this mapping) at
 * `va` unless another thread mapped the page meanwhile. Returns 1 when the
 * page is now present either way, 0 when mapping failed. */
static int filemap_install(uint64_t cr3, uint64_t va, uint64_t phys,
                           uint64_t page_flags, int shared)
{
    int ok = 1;
    uint64_t map_irq = irq_save_disable();
    spinlock_lock(&g_filemap_fault_lock);
    if (paging_virt_to_phys(cr3, va) != 0u) {
        free_page((void *)(uintptr_t)phys);     /* drops our reference */
    } else if ((shared ? paging_map_user_page_cow(cr3, va, phys, page_flags)
                       : paging_map_user_page(cr3, va, phys, page_flags)) < 0) {
        free_page((void *)(uintptr_t)phys);
        ok = 0;
    } else {
        hal_mmu_invalidate_tlb((uintptr_t)va);
    }
    spinlock_unlock(&g_filemap_fault_lock);
    irq_restore(map_irq);
    return ok;
}

int filemap_register(int32_t pid, uint64_t start, uint64_t length,
                     int32_t file_handle, uint64_t file_offset,
                     uint64_t page_flags)
{
    if (pid < 0 || length == 0u) {
        return -1;
    }
    filemap_init_once();

    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_filemap_lock);
    int result = -1;
    for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
        if (g_filemaps[i].used == 0u) {
            g_filemap_owner1[i]       = (uint32_t)pid + 1u;
            g_filemaps[i].used        = 1u;
            g_filemaps[i].owner_pid   = pid;
            g_filemaps[i].start       = start;
            g_filemaps[i].length      = length;
            g_filemaps[i].file_handle = file_handle;
            g_filemaps[i].file_offset = file_offset;
            g_filemaps[i].page_flags  = page_flags;
            g_filemaps[i].seq         = ++g_filemap_seq;
            result = 0;
            break;
        }
    }
    spinlock_unlock(&g_filemap_lock);
    irq_restore(irq);
    return result;
}

int filemap_register_zero(int32_t pid, uint64_t start, uint64_t length)
{
    if (g_filemap_ready == 0u) {
        return 0; /* nothing has ever been file-mapped: nothing to shadow */
    }
    /* Only worth a slot if a file mapping actually covers this range. */
    int overlaps = 0;
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_filemap_lock);
    for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
        const filemap_t *m = &g_filemaps[i];
        if (g_filemap_owner1[i] != (uint32_t)pid + 1u || m->file_handle < 0) continue;
        if (start + length <= m->start || start >= m->start + m->length) continue;
        overlaps = 1;
        break;
    }
    spinlock_unlock(&g_filemap_lock);
    irq_restore(irq);

    if (!overlaps) {
        return 0;
    }
    return filemap_register(pid, start, length, -1, 0u, 0u);
}

int filemap_addr_is_file_backed(int32_t pid, uint64_t addr)
{
    if (g_filemap_ready == 0u) {
        return 0;
    }
    int file_backed = 0;
    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_filemap_lock);
    for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
        const filemap_t *m = &g_filemaps[i];
        if (g_filemap_owner1[i] != (uint32_t)pid + 1u) continue;
        if (addr < m->start || addr >= m->start + m->length) continue;
        if (m->file_handle < 0) {
            /* A zero-fill hole punched over a file mapping wins: those pages
             * really are anonymous. */
            file_backed = 0;
            break;
        }
        file_backed = 1;
    }
    spinlock_unlock(&g_filemap_lock);
    irq_restore(irq);
    return file_backed;
}

int filemap_handle_fault(int32_t pid, uint64_t cr3, uint64_t fault_addr)
{
    if (pid < 0 || cr3 == 0u || g_filemap_ready == 0u) {
        return 0;
    }

    uint64_t page = fault_addr & ~0xFFFULL;

    /* Locate the mapping. The lock is dropped before the read: this runs in
     * the #PF handler, and the file read below reaches into the VFS and the
     * (polling) block driver, which take locks of their own.
     *
     * Most recent registration wins where mappings overlap. glibc's loader
     * first maps a whole shared object, then lays each PT_LOAD segment over
     * it with MAP_FIXED at a different file offset; taking the first match
     * would serve those segments the span mapping's bytes instead. */
    int32_t  handle = -1;
    uint64_t file_offset = 0;
    uint64_t page_flags = 0;
    uint64_t best_seq = 0;
    uint64_t window_end = 0;

    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_filemap_lock);
    for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
        const filemap_t *m = &g_filemaps[i];
        if (g_filemap_owner1[i] != (uint32_t)pid + 1u) continue;
        if (page < m->start || page >= m->start + m->length) continue;
        if (m->seq < best_seq) continue;
        best_seq    = m->seq;
        handle      = m->file_handle;
        file_offset = m->file_offset + (page - m->start);
        page_flags  = m->page_flags;
        window_end  = m->start + m->length;
    }
    /* Read-ahead must not run past a record registered later than the winner:
     * those pages belong to that record (a segment, or a .bss zero hole), not
     * to this one. */
    if (handle >= 0) {
        uint64_t cap = page + (uint64_t)FILEMAP_READAHEAD_PAGES * PAGE_SIZE;
        if (window_end > cap) window_end = cap;
        for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
            const filemap_t *m = &g_filemaps[i];
            if (g_filemap_owner1[i] != (uint32_t)pid + 1u) continue;
            if (m->seq <= best_seq) continue;
            if (m->start > page && m->start < window_end) {
                window_end = m->start;
            }
        }
    }
    spinlock_unlock(&g_filemap_lock);
    irq_restore(irq);

    if (handle < 0) {
        /* Either no record covers this page, or the winning record is a
         * zero-fill hole punched by a later MAP_ANONYMOUS|MAP_FIXED (a shared
         * object's .bss lands on top of the file mapping of its own image).
         * Both cases belong to the demand-zero path. */
        return 0;
    }

    /* Pages of a file on read-only media come from the shared cache: map the
     * cached run that starts here, if there is one. */
    uint64_t share_mount = 0, share_id = 0;
    int share = FILEMAP_SHARE_PAGES && fmc_ready() &&
                syscall_file_mmap_identity(handle, &share_mount, &share_id);
    const uint64_t first_index = file_offset / PAGE_SIZE;
    if (share) {
        uint64_t phys = fmc_get(share_mount, share_id, first_index);
        if (phys != 0u) {
            if (!filemap_install(cr3, page, phys, page_flags, 1)) {
                return 0;
            }
            for (uint64_t va = page + PAGE_SIZE; va < window_end; va += PAGE_SIZE) {
                if (paging_virt_to_phys(cr3, va) != 0u) break;
                phys = fmc_get(share_mount, share_id, first_index + (va - page) / PAGE_SIZE);
                if (phys == 0u || !filemap_install(cr3, va, phys, page_flags, 1)) break;
            }
            return 1;
        }
    }

    /* Fault in a run of pages, not just one. Each fault otherwise costs a
     * separate ATAPI command against the boot ISO, and a process like Xorg
     * touches tens of thousands of pages; ISO9660 reads whole sector runs in
     * one transfer, so a contiguous batch is close to the cost of a single
     * page. The run stops at the first page that is already present. */
    uint64_t pages = 1;
    while (page + pages * PAGE_SIZE < window_end &&
           pages < (uint64_t)FILEMAP_READAHEAD_PAGES &&
           paging_virt_to_phys(cr3, page + pages * PAGE_SIZE) == 0u) {
        ++pages;
    }

    /* Halve down rather than dropping straight to a single page: giving up the
     * whole read-ahead because a 16-page run was not available turns one
     * transfer into sixteen, and the block device is the slowest thing in
     * this path. */
    uint8_t *frames = (uint8_t *)pmm_alloc_pages((size_t)pages);
    while (frames == NULL && pages > 1u) {
        pages /= 2u;
        frames = (uint8_t *)pmm_alloc_pages((size_t)pages);
    }
    if (frames == NULL && filemap_reclaim_shared_pages() > 0u) {
        frames = (uint8_t *)pmm_alloc_pages(1u);
        pages = 1u;
    }
    if (frames == NULL) {
        serial_write_string("[OS] [PF] file map: no free page\n");
        process_debug_memory_report("file-backed fault", 0);
        return 0;
    }

    /* Zero first: a mapping may extend past end-of-file, and those bytes must
     * read as zero (the .bss tail of a shared object's last segment). */
    memset(frames, 0, (size_t)(pages * PAGE_SIZE));

    const uint64_t read_t0 = timer_monotonic_ns();
    int64_t got = syscall_file_mmap_read(handle, file_offset, frames,
                                         (uint32_t)(pages * PAGE_SIZE));
    __atomic_add_fetch(&g_fm_stat_faults, 1u, __ATOMIC_RELAXED);
    __atomic_add_fetch(&g_fm_stat_read_pages, pages, __ATOMIC_RELAXED);
    __atomic_add_fetch(&g_fm_stat_read_ns, timer_monotonic_ns() - read_t0, __ATOMIC_RELAXED);
    if (got < 0) {
        pmm_free_pages(frames, (size_t)pages);
        return 0;
    }

    /* Shared: hand each frame to the cache (or take the copy another CPU
     * entered first) and map it copy-on-write. filemap_install() re-checks
     * presence under the lock, as below. */
    if (share) {
        int serviced = 0;
        for (uint64_t i = 0; i < pages; ++i) {
            uint64_t frame = (uint64_t)(uintptr_t)(frames + i * PAGE_SIZE);
            uint64_t phys = fmc_put(share_mount, share_id, first_index + i, frame);
            int shared = phys != 0u;
            if (!shared) {
                phys = frame;                   /* table full: a private page */
            } else if (phys != frame) {
                pmm_free_pages((void *)(uintptr_t)frame, 1);
            }
            if (filemap_install(cr3, page + i * PAGE_SIZE, phys, page_flags, shared)) {
                if (i == 0u) serviced = 1;
            } else if (i == 0u) {
                /* Nothing mapped; the rest of the run is still ours to
                 * hand back or keep cached. */
                for (uint64_t j = 1; j < pages; ++j) {
                    uint64_t f = (uint64_t)(uintptr_t)(frames + j * PAGE_SIZE);
                    pmm_free_pages((void *)(uintptr_t)f, 1);
                }
                return 0;
            }
        }
        return serviced;
    }

    /* The presence check above happened before the (slow, blocking) file read,
     * so another thread of this process may have faulted any of these pages in
     * meanwhile -- and written to it. Mapping our fresh frame over the top
     * would strand those writes on a page nothing points at any more while
     * other CPUs keep a translation to it, which reads back as zeroes long
     * after the fact. Re-check every page under the lock and yield to whoever
     * got there first. */
    uint64_t map_irq = irq_save_disable();
    spinlock_lock(&g_filemap_fault_lock);
    int serviced = 0;
    for (uint64_t i = 0; i < pages; ++i) {
        uint64_t va = page + i * PAGE_SIZE;
        uint8_t *frame = frames + i * PAGE_SIZE;
        if (paging_virt_to_phys(cr3, va) != 0u) {
            pmm_free_pages(frame, 1);
            if (i == 0u) serviced = 1; /* already present: the retry will hit */
            continue;
        }
        if (paging_map_user_page(cr3, va, (uint64_t)(uintptr_t)frame,
                                 page_flags) < 0) {
            /* Hand back the frames of the run that went unused. */
            pmm_free_pages(frame, (size_t)(pages - i));
            break;
        }
        /* The faulting access is about to be retried; drop any stale
         * translation for this page first, matching the demand-zero path. */
        hal_mmu_invalidate_tlb((uintptr_t)va);
        if (i == 0u) serviced = 1;
    }
    spinlock_unlock(&g_filemap_fault_lock);
    irq_restore(map_irq);
    return serviced;
}

void filemap_unregister_range(int32_t pid, uint64_t start, uint64_t length)
{
    if (pid < 0 || length == 0u || g_filemap_ready == 0u) {
        return;
    }
    uint64_t end = start + length;

    uint64_t irq = irq_save_disable();
    spinlock_lock(&g_filemap_lock);
    for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
        filemap_t *m = &g_filemaps[i];
        if (g_filemap_owner1[i] != (uint32_t)pid + 1u) continue;
        /* Only whole-mapping removal is handled; a partial munmap leaves the
         * record in place so the remaining pages keep faulting in correctly.
         * Nothing in tree partially unmaps a library image. */
        if (m->start < start || m->start + m->length > end) continue;
        int32_t handle = m->file_handle;
        memset(m, 0, sizeof(*m));
        g_filemap_owner1[i] = 0u;
        spinlock_unlock(&g_filemap_lock);
        irq_restore(irq);
        syscall_file_mmap_release(handle);
        irq = irq_save_disable();
        spinlock_lock(&g_filemap_lock);
    }
    spinlock_unlock(&g_filemap_lock);
    irq_restore(irq);
}

uint32_t filemap_debug_count(int32_t pid)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
        if (g_filemap_owner1[i] != 0u &&
            (pid < 0 || g_filemap_owner1[i] == (uint32_t)pid + 1u)) {
            ++n;
        }
    }
    return n;
}

uint32_t filemap_debug_capacity(void)
{
    return FILEMAP_MAX;
}

void filemap_release_pid(int32_t pid)
{
    if (pid < 0 || g_filemap_ready == 0u) {
        return;
    }
    for (;;) {
        int32_t handle = -1;
        int found = 0;
        uint64_t irq = irq_save_disable();
        spinlock_lock(&g_filemap_lock);
        for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
            if (g_filemap_owner1[i] == (uint32_t)pid + 1u) {
                handle = g_filemaps[i].file_handle;
                memset(&g_filemaps[i], 0, sizeof(g_filemaps[i]));
                g_filemap_owner1[i] = 0u;
                found = 1;
                break;
            }
        }
        spinlock_unlock(&g_filemap_lock);
        irq_restore(irq);
        /* Keyed on `found`, not on the handle: a zero-fill hole carries -1 and
         * must not be mistaken for "no records left". */
        if (!found) {
            break;
        }
        if (handle >= 0) {
            syscall_file_mmap_release(handle);
        }
    }
    /* The process's pages are gone (or about to go) with its address space;
     * cached copies nobody maps any more can go too. */
    (void)filemap_reclaim_shared_pages();
}

int filemap_clone_for_fork(int32_t parent_pid, int32_t child_pid)
{
    if (parent_pid < 0 || child_pid < 0 || g_filemap_ready == 0u) {
        return 0;
    }
    /* Copied one record at a time rather than snapshotted -- a whole-table
     * copy would be several kilobytes of kernel stack -- and in the parent's
     * registration order, so the child's freshly assigned seq numbers keep the
     * same relative order and overlapping mappings still resolve the same way.
     * Rows added along the way belong to the child, so the owner test skips
     * them and no copy is ever copied. */
    uint64_t after_seq = 0;
    for (;;) {
        filemap_t entry;
        entry.used = 0u;

        uint64_t irq = irq_save_disable();
        spinlock_lock(&g_filemap_lock);
        uint64_t best = 0;
        for (uint32_t i = 0; i < FILEMAP_MAX; ++i) {
            const filemap_t *m = &g_filemaps[i];
            if (g_filemap_owner1[i] != (uint32_t)parent_pid + 1u) continue;
            if (m->seq <= after_seq) continue;
            if (best != 0u && m->seq >= best) continue;
            best = m->seq;
            entry = *m;
        }
        spinlock_unlock(&g_filemap_lock);
        irq_restore(irq);

        if (entry.used == 0u) {
            return 0;
        }
        after_seq = entry.seq;

        /* A zero-fill hole holds no file reference; carry it across as-is so
         * the child keeps the same overlay ordering. */
        int32_t handle = -1;
        if (entry.file_handle >= 0) {
            handle = syscall_file_mmap_reacquire(entry.file_handle);
            if (handle < 0) {
                filemap_release_pid(child_pid);
                return -1;
            }
        }
        if (filemap_register(child_pid, entry.start, entry.length,
                             handle, entry.file_offset,
                             entry.page_flags) < 0) {
            if (handle >= 0) syscall_file_mmap_release(handle);
            filemap_release_pid(child_pid);
            return -1;
        }
    }
    return 0;
}
