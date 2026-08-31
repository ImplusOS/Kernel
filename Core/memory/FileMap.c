#include "FileMap.h"

#include "Core/syscall/Syscall_File.h"
#include "Core/sync/Spinlock.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "interfaces/hal_cpu.h"
#include "Debug/serial/Serial.h"

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
#define FILEMAP_MAX 256

/* Pages faulted in per file-backed fault. See filemap_handle_fault(). */
#define FILEMAP_READAHEAD_PAGES 16

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
static spinlock_t g_filemap_lock;
static uint8_t    g_filemap_ready;
static uint64_t   g_filemap_seq;

static void filemap_init_once(void)
{
    if (g_filemap_ready == 0u) {
        spinlock_init(&g_filemap_lock);
        g_filemap_ready = 1u;
    }
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
        if (m->used == 0u || m->owner_pid != pid || m->file_handle < 0) continue;
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
        if (m->used == 0u || m->owner_pid != pid) continue;
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
            if (m->used == 0u || m->owner_pid != pid) continue;
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

    uint8_t *frames = (uint8_t *)pmm_alloc_pages((size_t)pages);
    if (frames == NULL && pages > 1u) {
        pages = 1u; /* no contiguous run available: settle for the one page */
        frames = (uint8_t *)pmm_alloc_pages(1);
    }
    if (frames == NULL) {
        return 0;
    }

    /* Zero first: a mapping may extend past end-of-file, and those bytes must
     * read as zero (the .bss tail of a shared object's last segment). */
    memset(frames, 0, (size_t)(pages * PAGE_SIZE));

    int64_t got = syscall_file_mmap_read(handle, file_offset, frames,
                                         (uint32_t)(pages * PAGE_SIZE));
    if (got < 0) {
        pmm_free_pages(frames, (size_t)pages);
        return 0;
    }

    for (uint64_t i = 0; i < pages; ++i) {
        uint64_t va = page + i * PAGE_SIZE;
        uint64_t pa = (uint64_t)(uintptr_t)(frames + i * PAGE_SIZE);
        if (paging_map_user_page(cr3, va, pa, page_flags) < 0) {
            if (i == 0u) {
                pmm_free_pages(frames, (size_t)pages);
                return 0;
            }
            /* The faulting page is mapped, so the access can be retried;
             * hand back the frames of the run that went unused. */
            pmm_free_pages(frames + i * PAGE_SIZE, (size_t)(pages - i));
            break;
        }
        /* The faulting access is about to be retried; drop any stale
         * translation for this page first, matching the demand-zero path. */
        hal_mmu_invalidate_tlb((uintptr_t)va);
    }
    return 1;
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
        if (m->used == 0u || m->owner_pid != pid) continue;
        /* Only whole-mapping removal is handled; a partial munmap leaves the
         * record in place so the remaining pages keep faulting in correctly.
         * Nothing in tree partially unmaps a library image. */
        if (m->start < start || m->start + m->length > end) continue;
        int32_t handle = m->file_handle;
        memset(m, 0, sizeof(*m));
        spinlock_unlock(&g_filemap_lock);
        irq_restore(irq);
        syscall_file_mmap_release(handle);
        irq = irq_save_disable();
        spinlock_lock(&g_filemap_lock);
    }
    spinlock_unlock(&g_filemap_lock);
    irq_restore(irq);
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
            if (g_filemaps[i].used != 0u && g_filemaps[i].owner_pid == pid) {
                handle = g_filemaps[i].file_handle;
                memset(&g_filemaps[i], 0, sizeof(g_filemaps[i]));
                found = 1;
                break;
            }
        }
        spinlock_unlock(&g_filemap_lock);
        irq_restore(irq);
        /* Keyed on `found`, not on the handle: a zero-fill hole carries -1 and
         * must not be mistaken for "no records left". */
        if (!found) {
            return;
        }
        if (handle >= 0) {
            syscall_file_mmap_release(handle);
        }
    }
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
            if (m->used == 0u || m->owner_pid != parent_pid) continue;
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
