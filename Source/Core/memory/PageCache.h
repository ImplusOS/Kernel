#pragma once

/*
 * PageCache -- shared, read-only copies of file pages from immutable media.
 *
 * Every Chromium process maps the same 500 MB `chrome` binary, and before this
 * cache each of them pulled the pages it touched off the boot medium itself:
 * the execve() of a zygote or GPU process re-read ~345 MB over USB mass
 * storage, and every page fault in every process read its 256 KiB run again,
 * with interrupts off. The block cache underneath is 32 MiB at most, so none
 * of that was ever a hit.
 *
 * A file qualifies when its filesystem declares itself immutable (the
 * read-only boot media -- vfs_driver_t.immutable) and it is large enough to
 * be worth tracking. Its pages are read once, kept, and handed out two ways:
 *   - copied: pagecache_read() serves any byte range from RAM;
 *   - shared: a demand-paging fault maps the cached frame itself into the
 *     process, read-only (copy-on-write for a writable mapping). The cache
 *     keeps its own reference on every frame, so a write always gets a
 *     private copy and the cached bytes never change.
 *
 * Nothing is ever evicted: the contents cannot go stale (the medium does not
 * change while mounted) and the total is capped (PAGECACHE_MAX_BYTES and a
 * free-memory floor); past the cap callers simply fall back to reading the
 * file themselves.
 */

#include <stdint.h>

#include "kernel/interfaces/vfs_file.h"

/* 1 if pages of `file` may be served from the cache. */
int pagecache_eligible(const vfs_file_t *file);

/* Kernel-virtual (identity-mapped) address of the cached page holding file
 * offset `page_offset` (page-aligned), filling it -- and up to `readahead`
 * following pages in the same transfer -- on a miss. NULL when the page
 * cannot be cached (not eligible, past EOF, cache full, read error); the
 * caller then reads the file itself. The page stays valid for the life of the
 * kernel and must never be written. */
void *pagecache_get_page(const vfs_file_t *file, uint64_t page_offset,
                         uint32_t readahead);

/* vfs_read_at() through the cache. Returns 1 when all `size` bytes were
 * served (bytes past EOF read as zero), 0 when the caller must fall back to
 * reading the file directly. */
int pagecache_read(const vfs_file_t *file, uint64_t offset, uint8_t *buffer,
                   uint64_t size);
