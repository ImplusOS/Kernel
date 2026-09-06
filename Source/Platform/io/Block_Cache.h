#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Read cache for the block layer (Platform/io/IO_Main.c).
 *
 * Why this exists: nothing between disk_read() and the media used to remember
 * anything, so every ISO9660/UDF path walk re-read the same directory sectors
 * off the boot medium. Resolving one shared object means walking "/usr", then
 * "lib", then "x86_64-linux-gnu", and each of those directories is scanned a
 * 2 KiB sector at a time until the name matches. glibc's ld.so opens (or
 * stats, and misses) hundreds of paths before a Linux program's first
 * instruction runs, so the *same* few hundred KiB of directory data was being
 * fetched from the disc tens of thousands of times. On optical media or USB
 * BOT each of those is a real command with real latency, which is why the
 * startup cost did not move when KVM was switched on and did not move on real
 * hardware either -- it was never CPU-bound.
 *
 * The cache is deliberately plain: fixed 4 KiB lines, 4-way set associative,
 * round-robin victim within a set, read-only (writes invalidate and go
 * straight through), one page allocated per line on first use so an idle
 * system pays nothing.
 */

/* Cache line, in 512-byte sectors. One page. */
#define BLOCK_CACHE_LINE_SECTORS 8u
#define BLOCK_CACHE_LINE_BYTES   (BLOCK_CACHE_LINE_SECTORS * 512u)

/* Requests at least this large bypass the cache entirely: they are streaming
 * reads (an ELF image, a firmware blob) that will not be asked for twice, and
 * letting them through the cache would evict exactly the metadata the cache
 * is there to keep. */
#define BLOCK_CACHE_BYPASS_SECTORS 256u

/* Sizes and allocates the tag table. Safe to call more than once; a second
 * call only flushes. Silently leaves the cache disabled if memory is short --
 * every entry point degrades to a direct read. */
void block_cache_init(void);

/* Drop everything (media or device change). */
void block_cache_flush(void);

/* `device_key` identifies the medium a line belongs to. It must distinguish
 * every device the system can read, including two devices that share a
 * per-protocol index (AHCI unit 0 and USB unit 0 are both "0"), because a
 * collision hands one device's sectors to another. IO_Main.c builds it from
 * the block_device_t pointer and the unit index. */

/* Serve [lba, lba+sectors) for `device_key` out of the cache, filling from
 * `fill` (which reads straight from the medium) on a miss. Returns false only
 * if the underlying read failed. */
bool block_cache_read(uint64_t device_key, uint64_t lba, uint8_t *buffer,
                      uint32_t sectors,
                      bool (*fill)(uint64_t lba, uint8_t *buf, uint32_t n));

/* Invalidate any line overlapping [lba, lba+sectors) on `device_key`. Every
 * path that writes to a medium must call this, including the ones that reach
 * a device directly rather than through disk_write(). */
void block_cache_invalidate(uint64_t device_key, uint64_t lba,
                            uint32_t sectors);

/* Hit/miss counters for the boot profile line. */
void block_cache_stats(uint64_t *out_hits, uint64_t *out_misses,
                       uint32_t *out_lines);
