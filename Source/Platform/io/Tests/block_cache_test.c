/*
 * Host harness for Platform/io/Block_Cache.c.
 *
 * The cache sits under every disk_read() in the system, so a bug in it
 * corrupts filesystem metadata rather than merely being slow. This drives the
 * real file (no copy, no #ifdef TEST) against a synthetic disc whose contents
 * are a known function of the sector number, so any wrong byte is caught by
 * value and not just by count.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Block_Cache.h"

uint64_t g_stub_total_pages = 1024u * 1024u;   /* 4 GiB of 4 KiB pages */
int64_t  g_stub_pages_left  = 1000000;

void *alloc_page(void)
{
    if (g_stub_pages_left <= 0) return NULL;
    --g_stub_pages_left;
    return calloc(1u, 4096u);
}

void free_page(void *addr)
{
    ++g_stub_pages_left;
    free(addr);
}

/* ---- synthetic medium ---------------------------------------------------- */

#define DISK_SECTORS 20000u

static uint32_t g_fill_calls;
static uint64_t g_fill_sectors;
static uint32_t g_fail_beyond_disk = 1u;   /* reads past the end fail */

static void sector_bytes(uint64_t lba, uint8_t *out)
{
    for (uint32_t i = 0; i < 512u; ++i) {
        out[i] = (uint8_t)((lba * 31u + i * 7u + (lba >> 8)) & 0xFFu);
    }
}

static bool disk_fill(uint64_t lba, uint8_t *buf, uint32_t sectors)
{
    if (g_fail_beyond_disk && (lba + sectors) > DISK_SECTORS) {
        return false;
    }
    ++g_fill_calls;
    g_fill_sectors += sectors;
    for (uint32_t i = 0; i < sectors; ++i) {
        sector_bytes(lba + i, buf + (size_t)i * 512u);
    }
    return true;
}

/* ---- checks -------------------------------------------------------------- */

static int g_failures;

static void check(const char *what, int ok)
{
    printf("%-5s %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) ++g_failures;
}

static int read_and_verify(uint64_t device, uint64_t lba, uint32_t sectors)
{
    uint8_t *got = malloc((size_t)sectors * 512u);
    uint8_t *want = malloc((size_t)sectors * 512u);
    int ok = block_cache_read(device, lba, got, sectors, disk_fill) ? 1 : 0;
    if (ok) {
        for (uint32_t i = 0; i < sectors; ++i) {
            sector_bytes(lba + i, want + (size_t)i * 512u);
        }
        ok = memcmp(got, want, (size_t)sectors * 512u) == 0;
    }
    free(got);
    free(want);
    return ok;
}

int main(void)
{
    block_cache_init();

    uint64_t hits = 0, misses = 0;
    uint32_t lines = 0;
    block_cache_stats(&hits, &misses, &lines);
    check("cache allocated", lines > 0u);

    /* 1. Every alignment and length up to two lines, twice (cold then warm). */
    int ok = 1;
    for (uint64_t lba = 0; lba < 40u && ok; ++lba) {
        for (uint32_t n = 1; n <= 20u && ok; ++n) {
            ok = read_and_verify(0u, lba, n) && read_and_verify(0u, lba, n);
        }
    }
    check("aligned and unaligned reads of every length", ok);

    /* 2. Scattered reads across the whole medium. */
    ok = 1;
    for (uint32_t i = 0; i < 400u && ok; ++i) {
        uint64_t lba = ((uint64_t)i * 4099u) % (DISK_SECTORS - 64u);
        ok = read_and_verify(0u, lba, 1u + (i % 40u));
    }
    check("scattered reads", ok);

    /* 3. The point of the thing: re-reading the same run must not touch the
     *    medium again. This is the ISO9660 directory-walk pattern. */
    (void)read_and_verify(0u, 100u, 8u);
    g_fill_calls = 0;
    for (uint32_t i = 0; i < 100u; ++i) {
        (void)read_and_verify(0u, 100u, 8u);
    }
    check("repeated reads are served from the cache", g_fill_calls == 0u);

    /* 4. One miss pulls a run, so a sequential sweep costs far fewer commands
     *    than it does sectors. */
    block_cache_flush();
    g_fill_calls = 0;
    ok = 1;
    for (uint64_t lba = 1000u; lba < 1000u + 256u && ok; lba += 4u) {
        ok = read_and_verify(0u, lba, 4u);
    }
    check("sequential sweep batches into runs", ok && g_fill_calls <= 64u / 8u + 2u);

    /* 5. Different devices must not alias each other. */
    /* Keys are opaque and sparse in practice (a pointer in the high bits),
     * so use two that hash to the same read-ahead slot as well. */
    check("device 1 reads correctly", read_and_verify(0x1234500u, 100u, 8u));
    check("device 0 unaffected", read_and_verify(0u, 100u, 8u));
    check("a third device is independent too",
          read_and_verify(0x1234508u, 100u, 8u));

    /* 6. A write invalidates, so the next read goes back to the medium. */
    (void)read_and_verify(0u, 500u, 8u);
    block_cache_invalidate(0u, 500u, 8u);
    g_fill_calls = 0;
    check("write invalidates the line",
          read_and_verify(0u, 500u, 8u) && g_fill_calls > 0u);

    /* 7. Large requests bypass the cache entirely rather than evicting it. */
    g_fill_calls = 0;
    ok = read_and_verify(0u, 2000u, BLOCK_CACHE_BYPASS_SECTORS);
    check("large request bypasses", ok && g_fill_calls == 1u);

    /* 8. The last sectors of the medium: the run fill runs off the end and
     *    must fall back to the exact request rather than failing. */
    check("read at end of medium",
          read_and_verify(0u, DISK_SECTORS - 3u, 3u));
    check("read at end of medium again (warm)",
          read_and_verify(0u, DISK_SECTORS - 3u, 3u));

    /* 9. A genuine medium error must be reported, not papered over. */
    {
        uint8_t buf[512];
        check("failed read is reported",
              !block_cache_read(0u, DISK_SECTORS + 100u, buf, 1u, disk_fill));
    }

    /* 10. Out of pages: correctness must not depend on the cache succeeding. */
    block_cache_flush();
    g_stub_pages_left = 0;
    ok = 1;
    for (uint64_t lba = 3000u; lba < 3040u && ok; ++lba) {
        ok = read_and_verify(0u, lba, 5u);
    }
    check("correct with no pages available", ok);
    g_stub_pages_left = 1000000;

    /* 11. The workload this exists for: glibc's ld.so resolving shared
     *     objects walks /usr, then lib, then x86_64-linux-gnu for every
     *     candidate path, scanning each directory a 2 KiB ISO sector (4
     *     512-byte sectors) at a time. Model 200 such lookups over three
     *     directories and count what reaches the medium. */
    block_cache_flush();
    g_fill_calls = 0;
    {
        const uint64_t dir_lba[3]   = { 6000u, 6400u, 7000u };
        const uint32_t dir_sectors[3] = { 40u, 80u, 320u }; /* 20/40/160 KiB */
        int walk_ok = 1;
        for (uint32_t lookup = 0; lookup < 200u && walk_ok; ++lookup) {
            for (uint32_t d = 0; d < 3u && walk_ok; ++d) {
                /* Scan until the (pseudo-random) matching record. */
                uint32_t scan = 4u + ((lookup * 13u + d * 7u) % dir_sectors[d]);
                scan &= ~3u;
                for (uint32_t off = 0; off < scan && walk_ok; off += 4u) {
                    walk_ok = read_and_verify(0u, dir_lba[d] + off, 4u);
                }
            }
        }
        check("directory-walk workload stays correct", walk_ok);
        /* Uncached, this pattern is tens of thousands of commands. Every
         * directory here fits in the cache, so after the first pass nothing
         * should reach the medium at all. */
        printf("      (%u medium commands for 200 path walks)\n", g_fill_calls);
        check("directory walk collapses to the first pass",
              g_fill_calls < 100u);
    }

    block_cache_stats(&hits, &misses, &lines);
    printf("\n%u lines, %llu hits, %llu misses, %u fills of %llu sectors\n",
           lines, (unsigned long long)hits, (unsigned long long)misses,
           g_fill_calls, (unsigned long long)g_fill_sectors);
    printf("%s\n", g_failures == 0 ? "all block-cache checks passed"
                                   : "BLOCK CACHE CHECKS FAILED");
    return g_failures == 0 ? 0 : 1;
}
