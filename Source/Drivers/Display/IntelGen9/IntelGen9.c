/*
 * IntelGen9.c - display driver for Intel 9th generation ("Gen9.5") integrated
 * graphics (Coffee Lake and its Whiskey Lake / Comet Lake siblings).
 *
 * WHAT THIS DOES
 *
 * The firmware has already programmed a display pipeline and handed the
 * kernel a linear framebuffer (the same one ImplusOS_Generic_Display_Driver
 * presents through), so this driver does NOT modeset: it never touches a
 * pipe, plane, PLL or port register. All it takes from the hardware is the
 * blitter engine (BCS), which it uses to move the window manager's composed
 * frame into the scanout buffer. Everything the GPU cannot be trusted to do
 * - and everything on a machine that is not one of the parts in
 * g_ig9_devices[] - stays exactly as it is on the generic driver: CPU
 * copies, same double-buffer protocol, same framebuffer.
 *
 * WHY IT IS BUILT THIS WAY
 *
 * Nothing here can be exercised in the environment this OS is developed in
 * (QEMU has no Intel GPU), so the code assumes any single register or
 * command encoding below might be wrong:
 *
 *   - Only known Gen9.5 device ids are ever touched.
 *   - Every wait on the hardware is deadline-bounded, and the one register
 *     write that could plausibly wedge a machine if it were wrong
 *     (forcewake) is checked against its acknowledge register before
 *     anything else is issued.
 *   - Acceleration is staged behind self-tests that run on memory this
 *     driver owns and can read back:
 *       stage 0 (always)   two MI_NOOPs and a MI_USER_INTERRUPT are queued
 *                          and the ring's HEAD is watched to advance past
 *                          them: forcewake, ring registers and the doorbell.
 *       stage 1            a scratch page is mapped into the GGTT, an
 *                          XY_COLOR_BLT fills it, a MI_STORE_DWORD_IMM fence
 *                          lands in it, and the CPU compares the result.
 *                          Nothing outside our own scratch is written.
 *       stage 2            an XY_SRC_COPY_BLT copies a CPU-written pattern
 *                          from a second scratch page into the first: the
 *                          exact direction present_rects() needs.
 *     Present acceleration additionally needs the scanout framebuffer's
 *     graphics address. That address is read out of the display plane's
 *     surface register when that is possible (each candidate is validated
 *     against the GGTT entry it names), and only otherwise looked up by
 *     scanning the GGTT for the framebuffer's physical address. A failure
 *     at any point logs over serial and leaves the CPU path in place.
 *   - A present whose fence does not confirm within its deadline falls back
 *     to a CPU copy of the same rectangles *and* disables GPU use for the
 *     rest of the boot, so a wedged engine costs one frame, not a session.
 *
 * WHAT IS VERIFIED
 *
 *   - Compiles for x86_64 and arm64 driver-module builds.
 *   - On machines without an allowlisted Intel GPU (QEMU included) probe()
 *     returns false before any MMIO access, so the generic driver stays
 *     selected - verified by booting the live image with this module staged.
 *   - MMIO/ring/command encodings follow the Gen9 programming model as used
 *     by i915's blitter paths and the Intel DDX's 2D blit sequences:
 *     BCS ring registers at 0x22000 + 0x30/0x34/0x38/0x3C, XY_COLOR_BLT and
 *     XY_SRC_COPY_BLT with the BR13 dword, MI_STORE_DWORD_IMM GEN4 fences.
 *     *Not* verified on real silicon; see Docs/Architecture/IntelGen9.md.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef IMPLUS_DRIVER_MODULE

#include "Drivers/Module/DriverBinary.h"

/* ------------------------------------------------------------------ *
 * 0. Driver-module glue
 * ------------------------------------------------------------------ */

static const driver_binary_t *g_api = NULL;

static void *ig9_memset(void *dst, int value, size_t n)
{
    uint8_t *p = (uint8_t *)dst;
    for (size_t i = 0; i < n; ++i) {
        p[i] = (uint8_t)value;
    }
    return dst;
}

#define api_malloc(size)              g_api->malloc((size))
#define api_free(ptr)                 g_api->free((ptr))
#define api_dma_alloc(size, phys_out) g_api->dma_alloc((size), (phys_out))
#define api_dma_free(ptr, size)       g_api->dma_free((ptr), (size))
#define api_virt_to_phys(ptr)         g_api->virt_to_phys((ptr))
#define api_map_mmio(phys, size)      g_api->hw.map_mmio_range((phys), (size))
#define api_memset                    ig9_memset

static void ig9_log(const char *str)
{
    if (g_api != NULL && g_api->serial_write_string != NULL) {
        g_api->serial_write_string(str);
    }
}

static void ig9_log_hex(uint32_t value)
{
    if (g_api != NULL && g_api->serial_write_uint32 != NULL) {
        g_api->serial_write_uint32(value);
    }
}

static void ig9_msleep(uint32_t ms)
{
    if (g_api != NULL && g_api->timer_msleep != NULL) {
        g_api->timer_msleep(ms);
    }
}

static void ig9_cache_flush(const void *base, uint64_t bytes)
{
#if defined(__x86_64__)
    const uint8_t *p = (const uint8_t *)base;
    for (uint64_t offset = 0; offset < bytes; offset += 64u) {
        __asm__ __volatile__("clflush (%0)" : : "r"((const void *)(p + offset))
                             : "memory");
    }
    __asm__ __volatile__("mfence" ::: "memory");
#else
    (void)base;
    (void)bytes;
#endif
}

/* ------------------------------------------------------------------ *
 * 1. Devices this driver will touch
 * ------------------------------------------------------------------ */

typedef struct {
    uint16_t device_id;
    const char *name;
} ig9_device_t;

/*
 * Gen9.5 parts only. Every id here shares the same blitter programming model
 * (2D blitter with XY_COLOR_BLT/XY_SRC_COPY_BLT, MI_STORE_DWORD_IMM fences,
 * GT forcewake at 0xA188). Deliberately absent: Gen9 parts (Skylake/Kaby
 * Lake) - they would very likely work with the same code, but "very likely"
 * is not the standard for a driver that cannot be tested here, and a wrong
 * guess means a wedged command stream on someone's laptop. Add an id only
 * with a self-test report from that machine.
 */
static const ig9_device_t g_ig9_devices[] = {
    { 0x3E90u, "UHD 610 (Coffee Lake)" },
    { 0x3E91u, "UHD 630 (Coffee Lake)" },
    { 0x3E92u, "UHD 630 (Coffee Lake)" },
    { 0x3E93u, "UHD 610 (Coffee Lake)" },
    { 0x3E98u, "UHD 630 (Coffee Lake)" },
    { 0x3E9Bu, "UHD 630 (Coffee Lake-H)" },
    { 0x3EA0u, "UHD 620 (Whiskey Lake)" },
    { 0x3EA5u, "UHD 655 (Coffee Lake)" },
    { 0x9B41u, "UHD 620 (Comet Lake)" },
    { 0x9BC5u, "UHD 630 (Comet Lake)" },
    { 0x9BC8u, "UHD 630 (Comet Lake)" },
};

#define IG9_DEVICE_COUNT (sizeof(g_ig9_devices) / sizeof(g_ig9_devices[0]))

#define IG9_PCI_VENDOR_INTEL  0x8086u
#define IG9_PCI_CLASS_DISPLAY 0x03u

typedef struct {
    uint8_t  bus;
    uint8_t  device;
    uint8_t  function;
    uint16_t device_id;
    const char *name;
} ig9_match_t;

static ig9_match_t g_match;
static int g_have_match = 0;
static int g_probe_attempted = 0;

/* ------------------------------------------------------------------ *
 * 2. MMIO register map (Gen9)
 * ------------------------------------------------------------------ */

/* BAR0 (GTTMMADR) is 16 MiB on Gen8+: a 4 MiB register window followed by
 * the GGTT at +8 MiB. The whole BAR is mapped so both are reachable. */
#define IG9_MMIO_MAP_BYTES       (16u * 1024u * 1024u)
#define IG9_PAGE_BYTES           4096u

/* GT forcewake: the GT power well must be awake before engine registers in
 * the 0x2xxxx block can be written. The acknowledge register reports the
 * same bit back once the well is up. */
#define IG9_FORCEWAKE_MT         0xA188u
#define IG9_FORCEWAKE_MT_ACK     0x130044u
#define IG9_FORCEWAKE_KERNEL_BIT 0x1u

/* Blitter (BCS) ring, Gen9. TAIL/HEAD/START/CTL sit at +0x30/+0x34/+0x38/
 * +0x3C; HEAD and TAIL also carry wrap counts above their address field. */
#define IG9_BLT_RING_BASE        0x22000u
#define IG9_RING_TAIL            (IG9_BLT_RING_BASE + 0x30u)
#define IG9_RING_HEAD            (IG9_BLT_RING_BASE + 0x34u)
#define IG9_RING_START           (IG9_BLT_RING_BASE + 0x38u)
#define IG9_RING_CTL             (IG9_BLT_RING_BASE + 0x3Cu)
#define IG9_RING_CTL_VALID       (1u << 0)
#define IG9_RING_CTL_SIZE_SHIFT  12u
#define IG9_RING_HEAD_ADDR_MASK  0x001FFFFCu

/* GGTT: the GTTMMADR BAR is 16 MiB; the register window occupies its first
 * 4 MiB and Gen8+ places the GTT of 8-byte PTEs (4 KiB pages) at offset
 * 8 MiB, covering up to 4 GiB of graphics address space. */
#define IG9_GGTT_OFFSET      (8u * 1024u * 1024u)
#define IG9_GGTT_PTE_BYTES   8u
#define IG9_GGTT_MAX_ENTRIES ((8u * 1024u * 1024u) / IG9_GGTT_PTE_BYTES)

/* Bits [38:12] of a PTE hold the physical page address on Gen9; everything
 * else (present, read/write, cache/pat bits) is left exactly as found in
 * the firmware's own entries. */
#define IG9_GGTT_PTE_ADDR_MASK ((uint64_t)0x7FFFFFF000ull)

/* How large the firmware actually made the GTT is reported by the host
 * bridge in GMCH_CTRL bits [7:6] (PCI config offset 0x50, Gen8+ layout:
 * a value N >= 1 means 2^N MiB of page table). */
#define IG9_PCI_GMCH_CTRL    0x50u
#define IG9_GMCH_GGMS_SHIFT  6u
#define IG9_GMCH_GGMS_MASK   0x3u

/* Display plane surface registers (pipe A/B/C, primary and sprite planes).
 * The firmware leaves the scanout buffer's graphics address in one of
 * these; the address is bits [31:12]. */
#define IG9_PLANE_SURF_1_A       0x7019Cu
#define IG9_PLANE_SURF_1_B       0x7119Cu
#define IG9_PLANE_SURF_1_C       0x7219Cu
#define IG9_PLANE_SURF_2_A       0x7029Cu
#define IG9_PLANE_SURF_2_B       0x7129Cu
#define IG9_PLANE_SURF_2_C       0x7229Cu
#define IG9_PLANE_SURF_ADDR_MASK 0xFFFFF000u

/* Our own GGTT pages come from the top of the table, which the firmware does
 * not use; each candidate entry is required to read back as empty before it
 * is written, and the window is placed inside the table size the firmware
 * programmed (see ig9_ggtt_detect_size()). */
#define IG9_GGTT_PRIVATE_ENTRIES 128u

static volatile uint8_t *g_mmio = NULL;
static int g_forcewake_on = 0;

static uint32_t ig9_mmio_read32(uint32_t offset)
{
    if (g_mmio == NULL) {
        return 0u;
    }
    return *(volatile uint32_t *)(g_mmio + offset);
}

static void ig9_mmio_write32(uint32_t offset, uint32_t value)
{
    if (g_mmio == NULL) {
        return;
    }
    *(volatile uint32_t *)(g_mmio + offset) = value;
}

static uint64_t ig9_ggtt_read_pte(uint32_t index)
{
    uint32_t lo = ig9_mmio_read32(IG9_GGTT_OFFSET + index * IG9_GGTT_PTE_BYTES);
    uint32_t hi = ig9_mmio_read32(IG9_GGTT_OFFSET + index * IG9_GGTT_PTE_BYTES + 4u);
    return (uint64_t)lo | ((uint64_t)hi << 32);
}

static void ig9_ggtt_write_pte(uint32_t index, uint64_t value)
{
    ig9_mmio_write32(IG9_GGTT_OFFSET + index * IG9_GGTT_PTE_BYTES,
                     (uint32_t)(value & 0xFFFFFFFFu));
    ig9_mmio_write32(IG9_GGTT_OFFSET + index * IG9_GGTT_PTE_BYTES + 4u,
                     (uint32_t)(value >> 32));
}

/* PTE template: the flags of an entry the firmware installed and is
 * currently scanning out of, so this driver never has to guess at Gen9's
 * present/cache bit layout. */
static uint64_t g_ggtt_template = 0;
static int g_ggtt_template_ready = 0;
static uint32_t g_ggtt_entries = IG9_GGTT_MAX_ENTRIES;
static uint32_t g_ggtt_private_first =
    IG9_GGTT_MAX_ENTRIES - IG9_GGTT_PRIVATE_ENTRIES;
static uint32_t g_ggtt_private_next =
    IG9_GGTT_MAX_ENTRIES - IG9_GGTT_PRIVATE_ENTRIES;

/*
 * Size the table the way the firmware programmed it, so the private window
 * at its top is always inside the real thing. A GMCH_CTRL value that makes
 * no sense (0 or garbage) leaves the full-size assumption in place.
 */
static void ig9_ggtt_detect_size(void)
{
    uint32_t entries = IG9_GGTT_MAX_ENTRIES;

    if (g_api->pci.read_config != NULL) {
        uint32_t gmch = g_api->pci.read_config(g_match.bus, g_match.device,
                                               g_match.function,
                                               IG9_PCI_GMCH_CTRL);
        uint32_t ggms = (gmch >> IG9_GMCH_GGMS_SHIFT) & IG9_GMCH_GGMS_MASK;
        if (ggms >= 1u && ggms <= 3u) {
            entries = ((1u << ggms) * 1024u * 1024u) / IG9_GGTT_PTE_BYTES;
        }
    }

    if (entries < IG9_GGTT_PRIVATE_ENTRIES * 2u) {
        entries = IG9_GGTT_MAX_ENTRIES;
    }

    g_ggtt_entries = entries;
    g_ggtt_private_first = entries - IG9_GGTT_PRIVATE_ENTRIES;
    g_ggtt_private_next = g_ggtt_private_first;

    ig9_log("[IntelGen9] ggtt entries 0x");
    ig9_log_hex(g_ggtt_entries);
    ig9_log("\n");
}

/*
 * Read-only search for the entry mapping `phys`. This is how the scanout
 * framebuffer - whose graphics address we are never told - becomes something
 * the blitter can be pointed at. The first match is also kept as the PTE
 * template used below.
 */
static bool ig9_ggtt_find_mapping(uint64_t phys, uint32_t *entry_out)
{
    uint64_t want = phys & ~(uint64_t)(IG9_PAGE_BYTES - 1u);

    /* The firmware maps the framebuffer low in the table; scanning the
     * first 64 MiB of graphics address space is generous and cheap. */
    for (uint32_t i = 0; i < 16384u && i < g_ggtt_entries; ++i) {
        uint64_t pte = ig9_ggtt_read_pte(i);
        if ((pte & 0x1u) == 0u) {
            continue;
        }
        if ((pte & IG9_GGTT_PTE_ADDR_MASK) == want) {
            if (entry_out != NULL) {
                *entry_out = i;
            }
            if (g_ggtt_template_ready == 0) {
                g_ggtt_template = pte;
                g_ggtt_template_ready = 1;
            }
            return true;
        }
    }
    return false;
}

/*
 * Point consecutive private GGTT entries at the pages of `virt`..`virt+bytes`
 * (one entry per page, in order). Walking pages instead of assuming one
 * contiguous physical span means this works for any allocation the kernel
 * hands out - the back buffer, the ring, scratch - whether or not it is
 * physically contiguous; the only requirement is a page-aligned start.
 * Every entry is verified empty before the write and non-empty after it; a
 * failure leaves the caller on the CPU path.
 */
static bool ig9_ggtt_map_pages(const void *virt, uint64_t bytes,
                               uint32_t *ggtt_offset_out)
{
    if (g_ggtt_template_ready == 0 || virt == NULL || ggtt_offset_out == NULL ||
        bytes == 0u) {
        return false;
    }
    if (((uintptr_t)virt & (uintptr_t)(IG9_PAGE_BYTES - 1u)) != 0u) {
        return false;
    }

    uint64_t pages64 = (bytes + IG9_PAGE_BYTES - 1u) / IG9_PAGE_BYTES;
    if (pages64 == 0u || pages64 > IG9_GGTT_PRIVATE_ENTRIES) {
        return false;
    }
    uint32_t pages = (uint32_t)pages64;
    if (g_ggtt_private_next + pages >
        g_ggtt_private_first + IG9_GGTT_PRIVATE_ENTRIES) {
        return false;
    }

    uint32_t first = g_ggtt_private_next;

    for (uint32_t i = 0; i < pages; ++i) {
        if (ig9_ggtt_read_pte(first + i) != 0u) {
            ig9_log("[IntelGen9] ggtt: private entry in use\n");
            return false;
        }
    }

    for (uint32_t i = 0; i < pages; ++i) {
        const uint8_t *page = (const uint8_t *)virt +
                              (uint64_t)i * IG9_PAGE_BYTES;
        uint64_t page_phys = api_virt_to_phys((void *)(uintptr_t)page);
        if (page_phys == 0u ||
            (page_phys & (uint64_t)(IG9_PAGE_BYTES - 1u)) != 0u) {
            ig9_log("[IntelGen9] ggtt: unmappable page\n");
            return false;
        }
        /* Same flags as the firmware's own entries, new address. */
        uint64_t pte = (g_ggtt_template & ~IG9_GGTT_PTE_ADDR_MASK) |
                       (page_phys & IG9_GGTT_PTE_ADDR_MASK);
        ig9_ggtt_write_pte(first + i, pte);
    }

    for (uint32_t i = 0; i < pages; ++i) {
        if (ig9_ggtt_read_pte(first + i) == 0u) {
            ig9_log("[IntelGen9] ggtt: write did not stick\n");
            return false;
        }
    }

    g_ggtt_private_next += pages;
    *ggtt_offset_out = first * IG9_PAGE_BYTES;
    return true;
}

/* ------------------------------------------------------------------ *
 * 3. Ring, commands, fences
 * ------------------------------------------------------------------ */

#define IG9_RING_BYTES    (64u * 1024u)
/* Eight 8-dword copies plus a 4-dword fence fit in one present batch. */
#define IG9_MAX_BATCH_DW  128u

/* Command encodings, kept together so they can be checked against the Gen9
 * PRM without reading the logic around them. */
#define IG9_MI_NOOP                  (0x00u << 23)
#define IG9_MI_USER_INTERRUPT        (0x02u << 23)
/* MI_STORE_DWORD_IMM, GEN4+ format: four dwords (header, address low,
 * address high, data) with the address interpreted as a GGTT address. */
#define IG9_MI_STORE_DWORD_IMM_GEN4  ((0x20u << 23) | 2u)
#define IG9_MI_USE_GGTT              (1u << 22)
#define IG9_XY_COLOR_BLT_CMD         ((2u << 29) | (0x50u << 22))
#define IG9_XY_SRC_COPY_BLT_CMD      ((2u << 29) | (0x53u << 22))
#define IG9_BLT_WRITE_ALPHA          (1u << 21)
#define IG9_BLT_WRITE_RGB            (1u << 20)
#define IG9_BLT_DEPTH_32             (3u << 24)  /* 8888 destination */
#define IG9_BLT_ROP_SRC_COPY         0xCCu
#define IG9_BLT_ROP_PAT_COPY         0xF0u
/* The 2D blit commands state "total dwords - 2" in bits [7:0]. */
#define IG9_BLT_DW_LEN(dwords)       ((dwords) - 2u)

typedef struct {
    volatile uint32_t *base;   /* CPU view of the ring */
    uint32_t ggtt_offset;      /* its graphics address */
    uint32_t size_bytes;
    uint32_t tail_bytes;       /* where the next command goes (8-byte aligned) */
    uint32_t seqno;
    int ready;
} ig9_ring_t;

static ig9_ring_t g_ring;

/* Scratch page: GGTT-mapped, holds the fence dword and the blitter's own
 * self-test surface. */
static uint8_t *g_scratch_virt = NULL;
static uint32_t g_scratch_ggtt = 0;
static int g_scratch_ready = 0;

static uint8_t *g_copy_src_virt = NULL;
static uint32_t g_copy_src_ggtt = 0;
static int g_copy_src_ready = 0;

static int g_accel_ok = 0;

#define IG9_FORCEWAKE_TIMEOUT_MS  50u
#define IG9_RING_DRAIN_TIMEOUT_MS 50u
#define IG9_FENCE_TIMEOUT_MS      50u
/* The fence owns the last dword of the scratch page so the blitter's
 * self-test surfaces can use the front of it. */
#define IG9_FENCE_DWORD_OFFSET    (IG9_PAGE_BYTES - sizeof(uint32_t))
#define IG9_MAX_RECTS_PER_BATCH   8u

static volatile uint32_t *ig9_fence_slot(void)
{
    return (volatile uint32_t *)(void *)(g_scratch_virt + IG9_FENCE_DWORD_OFFSET);
}

/*
 * Free space between the engine (HEAD) and the next command (TAIL). HEAD is
 * masked to its address field (its wrap count lives above it) and one 8-byte
 * slot is always left unused, which is what makes HEAD == TAIL mean "empty".
 * The mask arithmetic works because the ring size is a power of two.
 */
static uint32_t ig9_ring_free_dwords(void)
{
    uint32_t head = ig9_mmio_read32(IG9_RING_HEAD) & IG9_RING_HEAD_ADDR_MASK;
    uint32_t tail = g_ring.tail_bytes;
    uint32_t size = g_ring.size_bytes;

    if (size == 0u) {
        return 0u;
    }
    return ((tail - head - 8u) & (size - 1u)) / 4u;
}

/*
 * Copy a prepared batch into the ring and ring the doorbell. Batches are
 * padded to 8 bytes because that is the alignment the engine reads its
 * buffer at. Returns false without touching the hardware when there is not
 * enough room; callers drain first rather than blocking in here.
 */
static bool ig9_ring_submit(const uint32_t *cmds, uint32_t dwords)
{
    if (g_ring.ready == 0 || cmds == NULL || dwords == 0u) {
        return false;
    }

    uint32_t padded = (dwords + 1u) & ~1u;
    if (padded + 4u > ig9_ring_free_dwords()) {
        return false;
    }

    uint32_t start = g_ring.tail_bytes;
    for (uint32_t i = 0; i < padded; ++i) {
        uint32_t offset = g_ring.tail_bytes / 4u;
        g_ring.base[offset] = (i < dwords) ? cmds[i] : IG9_MI_NOOP;
        g_ring.tail_bytes += 4u;
        if (g_ring.tail_bytes >= g_ring.size_bytes) {
            g_ring.tail_bytes = 0u;
        }
    }

    /* The engine reads memory, not the CPU's caches: write the batch back
     * before the doorbell rings, then order the tail store after it. */
    const uint8_t *ring_bytes = (const uint8_t *)(const void *)g_ring.base;
    uint32_t written = padded * 4u;
    if (start + written <= g_ring.size_bytes) {
        ig9_cache_flush(ring_bytes + start, written);
    } else {
        ig9_cache_flush(ring_bytes + start, g_ring.size_bytes - start);
        ig9_cache_flush(ring_bytes, start + written - g_ring.size_bytes);
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);

    ig9_mmio_write32(IG9_RING_TAIL, g_ring.tail_bytes);
    /* Posting read: make sure the doorbell write reached the device. */
    (void)ig9_mmio_read32(IG9_RING_TAIL);
    return true;
}

static bool ig9_ring_drain(uint32_t timeout_ms)
{
    for (uint32_t waited = 0; waited <= timeout_ms; ++waited) {
        if ((ig9_mmio_read32(IG9_RING_HEAD) & IG9_RING_HEAD_ADDR_MASK) ==
            g_ring.tail_bytes) {
            return true;
        }
        ig9_msleep(1u);
    }
    return false;
}

/* Append a fence: the engine stores an incrementing seqno into the scratch
 * page. The engines execute in order, so a landed fence means the work
 * queued ahead of it has executed. */
static bool ig9_emit_fence(uint32_t *batch, uint32_t *dw_inout)
{
    if (g_scratch_ready == 0) {
        return false;
    }
    uint32_t dw = *dw_inout;
    batch[dw++] = IG9_MI_STORE_DWORD_IMM_GEN4 | IG9_MI_USE_GGTT;
    batch[dw++] = g_scratch_ggtt + IG9_FENCE_DWORD_OFFSET;
    batch[dw++] = 0u;   /* address high dword */
    batch[dw++] = ++g_ring.seqno;
    *dw_inout = dw;
    return true;
}

static bool ig9_wait_fence(uint32_t seqno, uint32_t timeout_ms)
{
    if (g_scratch_ready == 0) {
        return false;
    }
    volatile uint32_t *slot = ig9_fence_slot();
    for (uint32_t waited = 0; waited <= timeout_ms; ++waited) {
        /* Drop the CPU's cached copy so the engine's store is visible. */
        ig9_cache_flush((const void *)slot, sizeof(uint32_t));
        if (__atomic_load_n(slot, __ATOMIC_ACQUIRE) >= seqno) {
            return true;
        }
        ig9_msleep(1u);
    }
    return false;
}

/* ------------------------------------------------------------------ *
 * 4. Blitter commands
 * ------------------------------------------------------------------ */

/* XY_COLOR_BLT: six dwords (command, BR13 with destination pitch, rect
 * top-left, rect bottom-right, destination graphics address, colour). */
static uint32_t ig9_emit_fill(uint32_t *batch, uint32_t dst_ggtt,
                              uint32_t dst_pitch, int32_t x, int32_t y,
                              uint32_t w, uint32_t h, uint32_t color)
{
    uint32_t dw = 0u;

    batch[dw++] = IG9_XY_COLOR_BLT_CMD | IG9_BLT_DW_LEN(6u) |
                  IG9_BLT_WRITE_ALPHA | IG9_BLT_WRITE_RGB;
    batch[dw++] = IG9_BLT_DEPTH_32 |
                  ((uint32_t)IG9_BLT_ROP_PAT_COPY << 16) |
                  (dst_pitch & 0xFFFFu);
    batch[dw++] = ((uint32_t)y << 16) | ((uint32_t)x & 0xFFFFu);
    batch[dw++] = ((uint32_t)(y + (int32_t)h) << 16) |
                  ((uint32_t)(x + (int32_t)w) & 0xFFFFu);
    batch[dw++] = dst_ggtt;
    batch[dw++] = color;
    return dw;
}

/* XY_SRC_COPY_BLT: eight dwords (command, BR13 with destination pitch,
 * destination rect, destination address, source position, source pitch,
 * source address). The destination rectangle's size sets the copy size;
 * clipping is the caller's job. */
static uint32_t ig9_emit_copy(uint32_t *batch,
                              uint32_t dst_ggtt, uint32_t dst_pitch,
                              uint32_t src_ggtt, uint32_t src_pitch,
                              int32_t x, int32_t y, uint32_t w, uint32_t h)
{
    uint32_t dw = 0u;

    batch[dw++] = IG9_XY_SRC_COPY_BLT_CMD | IG9_BLT_DW_LEN(8u) |
                  IG9_BLT_WRITE_ALPHA | IG9_BLT_WRITE_RGB;
    batch[dw++] = IG9_BLT_DEPTH_32 |
                  ((uint32_t)IG9_BLT_ROP_SRC_COPY << 16) |
                  (dst_pitch & 0xFFFFu);
    batch[dw++] = ((uint32_t)y << 16) | ((uint32_t)x & 0xFFFFu);
    batch[dw++] = ((uint32_t)(y + (int32_t)h) << 16) |
                  ((uint32_t)(x + (int32_t)w) & 0xFFFFu);
    batch[dw++] = dst_ggtt;
    batch[dw++] = ((uint32_t)y << 16) | ((uint32_t)x & 0xFFFFu);
    batch[dw++] = src_pitch & 0xFFFFu;
    batch[dw++] = src_ggtt;
    return dw;
}

/* ------------------------------------------------------------------ *
 * 5. Bring-up: forcewake, ring, self-tests
 * ------------------------------------------------------------------ */

static bool ig9_forcewake(void)
{
    ig9_mmio_write32(IG9_FORCEWAKE_MT, IG9_FORCEWAKE_KERNEL_BIT);
    for (uint32_t waited = 0; waited <= IG9_FORCEWAKE_TIMEOUT_MS; ++waited) {
        if ((ig9_mmio_read32(IG9_FORCEWAKE_MT_ACK) &
             IG9_FORCEWAKE_KERNEL_BIT) != 0u) {
            /* Write again so this driver, not just the firmware, holds a
             * reference for as long as it is loaded. */
            ig9_mmio_write32(IG9_FORCEWAKE_MT, IG9_FORCEWAKE_KERNEL_BIT);
            g_forcewake_on = 1;
            return true;
        }
        ig9_msleep(1u);
    }

    /* Nothing else is touched if the acknowledge never comes: the machine
     * then behaves exactly as if this driver did not exist. */
    ig9_mmio_write32(IG9_FORCEWAKE_MT, 0u);
    return false;
}

static void ig9_forcewake_release(void)
{
    if (g_forcewake_on != 0) {
        ig9_mmio_write32(IG9_FORCEWAKE_MT, 0u);
        g_forcewake_on = 0;
    }
}

static bool ig9_ring_init(void)
{
    uint64_t ring_phys = 0;
    uint8_t *ring = (uint8_t *)api_dma_alloc(IG9_RING_BYTES, &ring_phys);
    if (ring == NULL || ring_phys == 0u) {
        ig9_log("[IntelGen9] ring: dma_alloc failed\n");
        return false;
    }
    api_memset(ring, 0, IG9_RING_BYTES);

    if ((ring_phys & (uint64_t)(IG9_PAGE_BYTES - 1u)) != 0u) {
        ig9_log("[IntelGen9] ring: not page aligned\n");
        api_dma_free(ring, IG9_RING_BYTES);
        return false;
    }

    if (!ig9_ggtt_map_pages(ring, IG9_RING_BYTES, &g_ring.ggtt_offset)) {
        ig9_log("[IntelGen9] ring: no GGTT mapping\n");
        api_dma_free(ring, IG9_RING_BYTES);
        return false;
    }

    g_ring.base = (volatile uint32_t *)(void *)ring;
    g_ring.size_bytes = IG9_RING_BYTES;
    g_ring.tail_bytes = 0u;
    g_ring.seqno = 0u;

    ig9_log("[IntelGen9] ring: ggtt 0x");
    ig9_log_hex(g_ring.ggtt_offset);
    ig9_log("\n");

    ig9_mmio_write32(IG9_RING_CTL, 0u);
    ig9_mmio_write32(IG9_RING_HEAD, 0u);
    ig9_mmio_write32(IG9_RING_TAIL, 0u);
    ig9_mmio_write32(IG9_RING_START, g_ring.ggtt_offset);
    /* Size is expressed in pages-minus-one at bit 12, with the valid bit. */
    ig9_mmio_write32(IG9_RING_CTL,
                     (((IG9_RING_BYTES / IG9_PAGE_BYTES) - 1u) <<
                      IG9_RING_CTL_SIZE_SHIFT) | IG9_RING_CTL_VALID);

    g_ring.ready = 1;
    return true;
}

/* Stage 0: does the ring move at all? */
static bool ig9_selftest_ring(void)
{
    uint32_t batch[4];
    uint32_t dw = 0u;

    batch[dw++] = IG9_MI_NOOP;
    batch[dw++] = IG9_MI_NOOP;
    batch[dw++] = IG9_MI_USER_INTERRUPT;

    if (!ig9_ring_submit(batch, dw)) {
        ig9_log("[IntelGen9] selftest: submit failed\n");
        return false;
    }
    if (!ig9_ring_drain(IG9_RING_DRAIN_TIMEOUT_MS)) {
        ig9_log("[IntelGen9] selftest: ring did not drain\n");
        return false;
    }
    return true;
}

/* Stage 1: can the blitter write memory this driver owns? */
static bool ig9_selftest_fill(void)
{
    uint32_t batch[IG9_MAX_BATCH_DW];
    uint32_t dw = 0u;
    const uint32_t pattern = 0xA5C3E1FFu;
    uint32_t seqno;

    dw += ig9_emit_fill(batch, g_scratch_ggtt, 256u, 0, 0, 32u, 8u, pattern);
    if (!ig9_emit_fence(batch, &dw)) {
        return false;
    }
    seqno = g_ring.seqno;

    volatile uint32_t *words = (volatile uint32_t *)(void *)g_scratch_virt;
    for (uint32_t i = 0; i < 64u; ++i) {
        words[i] = 0u;
    }
    ig9_cache_flush(g_scratch_virt, 256u);

    if (!ig9_ring_submit(batch, dw)) {
        ig9_log("[IntelGen9] selftest: fill submit failed\n");
        return false;
    }
    if (!ig9_wait_fence(seqno, IG9_FENCE_TIMEOUT_MS)) {
        ig9_log("[IntelGen9] selftest: fill fence timeout\n");
        return false;
    }

    ig9_cache_flush(g_scratch_virt, 256u);
    for (uint32_t i = 0; i < 5u; ++i) {
        if (words[i] != pattern) {
            ig9_log("[IntelGen9] selftest: fill wrong at ");
            ig9_log_hex(i);
            ig9_log("\n");
            return false;
        }
    }
    return true;
}

/* Stage 2: the copy direction present_rects() needs - source in system
 * memory written by the CPU, destination in GGTT-mapped memory. */
static bool ig9_selftest_copy(void)
{
    uint32_t batch[IG9_MAX_BATCH_DW];
    uint32_t dw = 0u;
    uint32_t seqno;

    if (g_copy_src_ready == 0 || g_scratch_ready == 0) {
        return false;
    }

    volatile uint32_t *src = (volatile uint32_t *)(void *)g_copy_src_virt;
    for (uint32_t i = 0; i < 64u; ++i) {
        src[i] = 0x11223344u + i;
    }
    ig9_cache_flush(g_copy_src_virt, 256u);

    volatile uint32_t *dst = (volatile uint32_t *)(void *)g_scratch_virt;
    for (uint32_t i = 0; i < 64u; ++i) {
        dst[i] = 0u;
    }
    ig9_cache_flush(g_scratch_virt, 256u);

    dw += ig9_emit_copy(batch, g_scratch_ggtt, 256u, g_copy_src_ggtt, 256u,
                        0, 0, 64u, 1u);
    if (!ig9_emit_fence(batch, &dw)) {
        return false;
    }
    seqno = g_ring.seqno;

    if (!ig9_ring_submit(batch, dw)) {
        ig9_log("[IntelGen9] selftest: copy submit failed\n");
        return false;
    }
    if (!ig9_wait_fence(seqno, IG9_FENCE_TIMEOUT_MS)) {
        ig9_log("[IntelGen9] selftest: copy fence timeout\n");
        return false;
    }

    ig9_cache_flush(g_scratch_virt, 256u);
    for (uint32_t i = 0; i < 64u; ++i) {
        if (dst[i] != 0x11223344u + i) {
            ig9_log("[IntelGen9] selftest: copy wrong at ");
            ig9_log_hex(i);
            ig9_log("\n");
            return false;
        }
    }
    return true;
}

/* ------------------------------------------------------------------ *
 * 6. Display surface (same protocol as the generic driver)
 * ------------------------------------------------------------------ */

#define IG9_BYTES_PER_PIXEL 4u

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t pixels_per_scan_line;
    uint64_t size_bytes;
    uint64_t phys_base;
    volatile uint8_t *mapped;
    uint32_t ggtt_offset;   /* graphics address of the first mapped page */
    int ggtt_found;
} ig9_fb_t;

static ig9_fb_t g_fb;
static uint32_t *g_back_buffer = NULL;
static uint8_t *g_back_buffer_raw = NULL;
static uint64_t g_back_buffer_bytes = 0;
static uint32_t g_back_ggtt = 0;
static int g_back_ggtt_ready = 0;
static int g_ready = 0;
static uint32_t g_generation = 1u;

static void ig9_copy_string(char *dst, uint32_t dst_size, const char *src)
{
    uint32_t i = 0u;
    if (dst == NULL || dst_size == 0u) {
        return;
    }
    if (src != NULL) {
        while (i + 1u < dst_size && src[i] != '\0') {
            dst[i] = src[i];
            ++i;
        }
    }
    dst[i] = '\0';
}

static bool ig9_fb_set(const driver_boot_framebuffer_t *framebuffer)
{
    g_ready = 0;
    g_back_ggtt_ready = 0;
    ++g_generation;

    if (g_back_buffer != NULL) {
        api_free(g_back_buffer_raw);
        g_back_buffer = NULL;
        g_back_buffer_raw = NULL;
        g_back_buffer_bytes = 0u;
    }
    api_memset(&g_fb, 0, sizeof(g_fb));

    if (framebuffer == NULL || framebuffer->addr == NULL ||
        framebuffer->width == 0u || framebuffer->height == 0u ||
        framebuffer->pixels_per_scan_line < framebuffer->width) {
        return false;
    }

    uint32_t bpp = framebuffer->bytes_per_pixel;
    if (bpp == 0u) {
        bpp = IG9_BYTES_PER_PIXEL;
    }
    if (bpp != IG9_BYTES_PER_PIXEL) {
        return false;
    }

    uint64_t required = (uint64_t)framebuffer->pixels_per_scan_line *
                        (uint64_t)framebuffer->height * (uint64_t)bpp;
    if ((framebuffer->size_bytes != 0u) &&
        (required > (uint64_t)framebuffer->size_bytes)) {
        return false;
    }

    uint64_t phys = (uint64_t)(uintptr_t)framebuffer->addr;
    volatile uint8_t *mapped =
        (volatile uint8_t *)api_map_mmio(phys, (size_t)required);
    if (mapped == NULL) {
        ig9_log("[IntelGen9] fb: map failed\n");
        return false;
    }

    uint64_t buffer_size = (uint64_t)framebuffer->pixels_per_scan_line *
                           (uint64_t)framebuffer->height * sizeof(uint32_t);
    /* One extra page so the buffer the blitter sees can start on a page
     * boundary whatever the allocator's alignment is. */
    uint8_t *buffer_raw = (uint8_t *)api_malloc(buffer_size + IG9_PAGE_BYTES);
    if (buffer_raw == NULL) {
        return false;
    }
    uint32_t *buffer = (uint32_t *)(void *)
        (((uintptr_t)buffer_raw + (IG9_PAGE_BYTES - 1u)) &
         ~(uintptr_t)(IG9_PAGE_BYTES - 1u));
    api_memset(buffer, 0, (size_t)buffer_size);
    g_back_buffer_raw = buffer_raw;

    g_fb.width = framebuffer->width;
    g_fb.height = framebuffer->height;
    g_fb.pixels_per_scan_line = framebuffer->pixels_per_scan_line;
    g_fb.size_bytes = required;
    g_fb.phys_base = phys;
    g_fb.mapped = mapped;

    /* The graphics address of the framebuffer is only discoverable once the
     * MMIO window is mapped; ig9_hw_bringup() does that and fills in
     * g_fb.ggtt_offset/ggtt_found. */

    g_back_buffer = buffer;
    g_back_buffer_bytes = buffer_size;
    g_ready = 1;
    return true;
}

static void ig9_hw_bringup(void);

/*
 * BAR0 reads on a function parked in D3 can hang the bus on real hardware,
 * so before the first MMIO touch: turn on memory decoding and bus mastering,
 * and if the function has a power-management capability and is not in D0,
 * ask it to wake up. Both are no-ops on the firmware setups this project
 * normally sees; this runs only for a device that already matched the
 * display allowlist, and a failure here just leaves the CPU path.
 */
static bool ig9_pci_prepare(void)
{
    uint8_t bus = g_match.bus;
    uint8_t dev = g_match.device;
    uint8_t fn = g_match.function;

    if (g_api->pci.read_config == NULL || g_api->pci.write_config == NULL) {
        return false;
    }

    uint32_t command = g_api->pci.read_config(bus, dev, fn, 0x04u);
    if ((command & 0x6u) != 0x6u) {
        g_api->pci.write_config(bus, dev, fn, 0x04u, command | 0x6u);
    }

    if (g_api->pci.find_capability == NULL) {
        return true;
    }
    int32_t pm_cap = g_api->pci.find_capability(bus, dev, fn, 0x01u);
    if (pm_cap < 0) {
        return true; /* no PM capability: nothing that can be in D3 */
    }
    uint16_t pmcsr_offset = (uint16_t)((uint32_t)pm_cap + 4u);

    uint32_t pmcsr = g_api->pci.read_config(bus, dev, fn, pmcsr_offset);
    if ((pmcsr & 0x3u) != 0u) {
        g_api->pci.write_config(bus, dev, fn, pmcsr_offset, pmcsr & ~0x3u);
        ig9_msleep(10u); /* a wake from D3 takes a moment on real silicon */
        pmcsr = g_api->pci.read_config(bus, dev, fn, pmcsr_offset);
        if ((pmcsr & 0x3u) != 0u) {
            ig9_log("[IntelGen9] bringup: device will not enter D0\n");
            return false;
        }
    }
    return true;
}

static void ig9_pci_find_device(void);

/*
 * Find the scanout framebuffer's graphics address in the display plane's
 * surface register. A candidate is only accepted when the GGTT entry it
 * names is present and points at a page inside the framebuffer's own
 * physical range, so a misread register can never become a blit target.
 * The first accepted entry also teaches ig9_ggtt_map_pages() the PTE flags
 * the firmware itself uses.
 */
static bool ig9_fb_find_via_plane(void)
{
    static const uint32_t surf_regs[] = {
        IG9_PLANE_SURF_1_A, IG9_PLANE_SURF_1_B, IG9_PLANE_SURF_1_C,
        IG9_PLANE_SURF_2_A, IG9_PLANE_SURF_2_B, IG9_PLANE_SURF_2_C,
    };

    if (g_fb.mapped == NULL || g_fb.phys_base == 0u || g_fb.size_bytes == 0u) {
        return false;
    }

    uint64_t first_page = g_fb.phys_base & ~(uint64_t)(IG9_PAGE_BYTES - 1u);
    uint64_t last_page = g_fb.phys_base + g_fb.size_bytes;

    for (uint32_t i = 0u; i < sizeof(surf_regs) / sizeof(surf_regs[0]); ++i) {
        uint32_t value = ig9_mmio_read32(surf_regs[i]);
        uint32_t ggtt = value & IG9_PLANE_SURF_ADDR_MASK;
        if (ggtt == 0u) {
            continue;
        }

        uint64_t pte = ig9_ggtt_read_pte(ggtt / IG9_PAGE_BYTES);
        if ((pte & 0x1u) == 0u) {
            continue;
        }
        uint64_t page = pte & IG9_GGTT_PTE_ADDR_MASK;
        if (page < first_page || page >= last_page) {
            continue;
        }

        if (g_ggtt_template_ready == 0) {
            g_ggtt_template = pte;
            g_ggtt_template_ready = 1;
        }
        g_fb.ggtt_offset = ggtt;
        g_fb.ggtt_found = 1;
        ig9_log("[IntelGen9] fb: plane 0x");
        ig9_log_hex(surf_regs[i]);
        ig9_log(" -> ggtt 0x");
        ig9_log_hex(ggtt);
        ig9_log("\n");
        return true;
    }
    return false;
}

static bool ig9_probe(void)
{
    if (g_have_match == 0 && g_probe_attempted == 0) {
        /* driver_module_init() runs before any module is activated, so the
         * PCI client is not attached yet at that point; probe() runs once
         * the bus driver is live. The device list is static, so search the
         * first time only. */
        g_probe_attempted = 1;
        ig9_pci_find_device();
    }
    return g_have_match != 0;
}

/* The display manager calls set_framebuffer() on every candidate driver
 * before probe()/init(), so the hardware bring-up (which needs the
 * framebuffer's physical address to find its GGTT mapping) runs here, on
 * the first init(), once the framebuffer is known. The return value stays
 * "is the framebuffer ready": a bring-up that fails only means the CPU
 * present path is used, not that the driver cannot drive the display. */
static bool ig9_init(void)
{
    if (g_ready == 0) {
        return false;
    }
    ig9_hw_bringup();
    return true;
}

static bool ig9_is_ready(void)
{
    return g_ready != 0;
}

static uint32_t ig9_width(void)
{
    return g_fb.width;
}

static uint32_t ig9_height(void)
{
    return g_fb.height;
}

static uint32_t ig9_generation(void)
{
    return g_generation;
}

static bool ig9_poll_config(void)
{
    return false;
}

static uint32_t ig9_monitor_count(void)
{
    return g_ready ? 1u : 0u;
}

static bool ig9_get_topology(display_topology_t *out_topology)
{
    if (g_ready == 0 || out_topology == NULL) {
        return false;
    }
    api_memset(out_topology, 0, sizeof(*out_topology));
    out_topology->generation = g_generation;
    out_topology->monitor_count = 1u;
    out_topology->primary_monitor = 0u;
    out_topology->width = g_fb.width;
    out_topology->height = g_fb.height;
    return true;
}

static bool ig9_get_monitor_info(uint32_t monitor_index,
                                 display_monitor_info_t *out_info)
{
    if (g_ready == 0 || out_info == NULL || monitor_index != 0u) {
        return false;
    }
    api_memset(out_info, 0, sizeof(*out_info));
    out_info->index = 0u;
    out_info->flags = DISPLAY_MONITOR_FLAG_CONNECTED |
                      DISPLAY_MONITOR_FLAG_PRIMARY |
                      DISPLAY_MONITOR_FLAG_SYNTHETIC_MODE;
    out_info->output_type = DISPLAY_OUTPUT_FRAMEBUFFER;
    out_info->width = g_fb.width;
    out_info->height = g_fb.height;
    out_info->refresh_millihz = 60000u;
    out_info->mode_count = 1u;
    out_info->generation = g_generation;
    ig9_copy_string(out_info->name, sizeof(out_info->name),
                    (g_have_match != 0 && g_match.name != NULL) ?
                        g_match.name : "Intel Gen9.5 Graphics");
    ig9_copy_string(out_info->output_name, sizeof(out_info->output_name),
                    "framebuffer0");
    return true;
}

static bool ig9_get_mode_info(uint32_t monitor_index, uint32_t mode_index,
                              display_mode_info_t *out_info)
{
    if (g_ready == 0 || out_info == NULL || monitor_index != 0u ||
        mode_index != 0u) {
        return false;
    }
    api_memset(out_info, 0, sizeof(*out_info));
    out_info->flags = DISPLAY_MODE_FLAG_CURRENT |
                      DISPLAY_MODE_FLAG_PREFERRED |
                      DISPLAY_MODE_FLAG_SYNTHETIC;
    out_info->width = g_fb.width;
    out_info->height = g_fb.height;
    out_info->stride = g_fb.pixels_per_scan_line;
    out_info->bits_per_pixel = 32u;
    out_info->refresh_millihz = 60000u;
    ig9_copy_string(out_info->name, sizeof(out_info->name), "Current");
    return true;
}

/* Deliberately no modesetting: the firmware's mode is the mode, and this
 * driver must not risk the only working display configuration. */
static bool ig9_set_mode(uint32_t monitor_index, uint32_t mode_index)
{
    (void)monitor_index;
    (void)mode_index;
    return false;
}

static void ig9_draw_pixel(uint32_t x, uint32_t y, uint32_t color)
{
    if (g_ready == 0 || g_back_buffer == NULL) {
        return;
    }
    if (x >= g_fb.width || y >= g_fb.height) {
        return;
    }
    uint64_t index = (uint64_t)y * g_fb.pixels_per_scan_line + x;
    if (index < g_back_buffer_bytes / sizeof(uint32_t)) {
        g_back_buffer[index] = color;
    }
}

static void ig9_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                          uint32_t color)
{
    if (g_ready == 0 || g_back_buffer == NULL || w == 0u || h == 0u) {
        return;
    }
    if (x >= g_fb.width || y >= g_fb.height) {
        return;
    }
    uint64_t x_end = (uint64_t)x + w;
    uint64_t y_end = (uint64_t)y + h;
    if (x_end > g_fb.width)  { x_end = g_fb.width; }
    if (y_end > g_fb.height) { y_end = g_fb.height; }

    for (uint32_t py = y; py < (uint32_t)y_end; ++py) {
        uint64_t row = (uint64_t)py * g_fb.pixels_per_scan_line;
        for (uint64_t idx = row + x; idx < row + x_end; ++idx) {
            if (idx >= g_back_buffer_bytes / sizeof(uint32_t)) {
                break;
            }
            g_back_buffer[idx] = color;
        }
    }
}

static bool ig9_clip_rect(const display_rect_t *in, uint32_t *x, uint32_t *y,
                          uint32_t *w, uint32_t *h)
{
    if (in == NULL || in->w == 0u || in->h == 0u ||
        g_fb.width == 0u || g_fb.height == 0u) {
        return false;
    }
    int64_t x0 = in->x;
    int64_t y0 = in->y;
    int64_t x1 = x0 + (int64_t)in->w;
    int64_t y1 = y0 + (int64_t)in->h;
    if (x1 <= 0 || y1 <= 0 || x0 >= (int64_t)g_fb.width ||
        y0 >= (int64_t)g_fb.height) {
        return false;
    }
    if (x0 < 0) { x0 = 0; }
    if (y0 < 0) { y0 = 0; }
    if (x1 > (int64_t)g_fb.width)  { x1 = (int64_t)g_fb.width; }
    if (y1 > (int64_t)g_fb.height) { y1 = (int64_t)g_fb.height; }
    if (x1 <= x0 || y1 <= y0) {
        return false;
    }
    *x = (uint32_t)x0;
    *y = (uint32_t)y0;
    *w = (uint32_t)(x1 - x0);
    *h = (uint32_t)(y1 - y0);
    return true;
}

static void ig9_cpu_present_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    if (g_fb.mapped == NULL || g_back_buffer == NULL || w == 0u || h == 0u) {
        return;
    }

    uint64_t pitch = (uint64_t)g_fb.pixels_per_scan_line * sizeof(uint32_t);
    const uint8_t *src_base = (const uint8_t *)g_back_buffer;

    for (uint32_t row = 0; row < h; ++row) {
        const uint8_t *src = src_base + (uint64_t)(y + row) * pitch +
                             (uint64_t)x * sizeof(uint32_t);
        volatile uint8_t *dst = g_fb.mapped + (uint64_t)(y + row) * pitch +
                                (uint64_t)x * sizeof(uint32_t);
        for (uint32_t col = 0; col < w * sizeof(uint32_t); ++col) {
            dst[col] = src[col];
        }
    }
}

/* Hand the back buffer's pages to the engine, once, the first time a
 * present wants them. */
static bool ig9_back_buffer_map(void)
{
    if (g_back_ggtt_ready != 0) {
        return true;
    }
    if (g_back_buffer == NULL) {
        return false;
    }

    uint64_t bytes64 = (g_back_buffer_bytes + IG9_PAGE_BYTES - 1u) &
                       ~(uint64_t)(IG9_PAGE_BYTES - 1u);
    if (bytes64 == 0u || bytes64 > 0xFFFFFFFFu) {
        return false;
    }
    /* ig9_ggtt_map_pages() handles a non-contiguous buffer page by page, so
     * all that is required here is that the buffer starts on a page. */
    if (!ig9_ggtt_map_pages((const void *)g_back_buffer, bytes64,
                            &g_back_ggtt)) {
        return false;
    }
    g_back_ggtt_ready = 1;
    return true;
}

/*
 * Accelerated present: up to IG9_MAX_RECTS_PER_BATCH rectangles per batch,
 * each with the source rows flushed first (the engine reads physical
 * memory, not the CPU's caches), then one fence for the whole batch. A
 * fence that does not confirm in time disables GPU use for the rest of the
 * boot; the caller then CPU-copies the same rectangles, which is idempotent.
 */
static bool ig9_gpu_present_rects(const display_rect_t *rects, uint32_t count)
{
    uint32_t batch[IG9_MAX_BATCH_DW];
    uint32_t dw = 0u;
    uint32_t seqno = 0u;
    uint32_t batched_rects = 0u;
    int submitted = 0;

    if (g_accel_ok == 0 || g_fb.ggtt_found == 0 ||
        !ig9_back_buffer_map()) {
        return false;
    }

    uint32_t dst_pitch = g_fb.pixels_per_scan_line * sizeof(uint32_t);
    uint32_t src_pitch = g_fb.pixels_per_scan_line * sizeof(uint32_t);

    for (uint32_t i = 0; i < count; ++i) {
        uint32_t x = 0u;
        uint32_t y = 0u;
        uint32_t w = 0u;
        uint32_t h = 0u;
        if (!ig9_clip_rect(&rects[i], &x, &y, &w, &h)) {
            continue;
        }

        if (batched_rects == IG9_MAX_RECTS_PER_BATCH) {
            if (!ig9_emit_fence(batch, &dw) || !ig9_ring_submit(batch, dw)) {
                return false;
            }
            seqno = g_ring.seqno;
            submitted = 1;
            dw = 0u;
            batched_rects = 0u;
        }

        for (uint32_t row = 0; row < h; ++row) {
            ig9_cache_flush((const uint8_t *)g_back_buffer +
                                ((uint64_t)(y + row) * src_pitch +
                                 (uint64_t)x * sizeof(uint32_t)),
                            (uint64_t)w * sizeof(uint32_t));
        }

        dw += ig9_emit_copy(batch,
                            g_fb.ggtt_offset + y * dst_pitch + x * 4u,
                            dst_pitch,
                            g_back_ggtt + y * src_pitch + x * 4u,
                            src_pitch, (int32_t)x, (int32_t)y, w, h);
        ++batched_rects;
    }

    if (batched_rects != 0u) {
        if (!ig9_emit_fence(batch, &dw) || !ig9_ring_submit(batch, dw)) {
            return false;
        }
        seqno = g_ring.seqno;
        submitted = 1;
    }

    if (submitted == 0) {
        return true;    /* nothing to present */
    }
    if (!ig9_wait_fence(seqno, IG9_FENCE_TIMEOUT_MS)) {
        ig9_log("[IntelGen9] present: fence timeout, GPU disabled\n");
        g_accel_ok = 0;
        return false;
    }
    return true;
}

static void ig9_present_rects(const display_rect_t *rects, uint32_t count);

static void ig9_present(void)
{
    if (g_ready == 0) {
        return;
    }
    display_rect_t full = { 0, 0, g_fb.width, g_fb.height };
    ig9_present_rects(&full, 1u);
}

static void ig9_present_rects(const display_rect_t *rects, uint32_t count)
{
    if (g_ready == 0 || g_back_buffer == NULL) {
        return;
    }
    if (rects == NULL || count == 0u) {
        display_rect_t full = { 0, 0, g_fb.width, g_fb.height };
        ig9_present_rects(&full, 1u);
        return;
    }

    if (ig9_gpu_present_rects(rects, count)) {
        return;
    }

    for (uint32_t i = 0; i < count; ++i) {
        uint32_t x = 0u;
        uint32_t y = 0u;
        uint32_t w = 0u;
        uint32_t h = 0u;
        if (ig9_clip_rect(&rects[i], &x, &y, &w, &h)) {
            ig9_cpu_present_rect(x, y, w, h);
        }
    }
}

static void *ig9_get_framebuffer(void)
{
    return g_back_buffer;
}

static const driver_display_t g_ig9_display = {
    .name = "Intel Gen9.5 Graphics",
    .probe = ig9_probe,
    .init = ig9_init,
    .is_ready = ig9_is_ready,
    .width = ig9_width,
    .height = ig9_height,
    .draw_pixel = ig9_draw_pixel,
    .fill_rect = ig9_fill_rect,
    .present = ig9_present,
    .present_rects = ig9_present_rects,
    .set_framebuffer = ig9_fb_set,
    .get_framebuffer = ig9_get_framebuffer,
    .get_generation = ig9_generation,
    .poll_config = ig9_poll_config,
    .get_monitor_count = ig9_monitor_count,
    .get_topology = ig9_get_topology,
    .get_monitor_info = ig9_get_monitor_info,
    .get_mode_info = ig9_get_mode_info,
    .set_mode = ig9_set_mode,
};

/* ------------------------------------------------------------------ *
 * 7. Hardware bring-up and module entry
 * ------------------------------------------------------------------ */

static void ig9_hw_bringup(void)
{
    if (g_have_match == 0 || g_mmio != NULL) {
        return;
    }

    driver_pci_bar_t bar;
    api_memset(&bar, 0, sizeof(bar));
    if (g_api->pci.get_bar == NULL ||
        !g_api->pci.get_bar(g_match.bus, g_match.device, g_match.function,
                            0u, &bar) ||
        bar.address == 0u || bar.is_io != 0u ||
        bar.size < IG9_MMIO_MAP_BYTES) {
        ig9_log("[IntelGen9] bringup: BAR0 unusable\n");
        return;
    }
    ig9_log("[IntelGen9] bringup: BAR0 0x");
    ig9_log_hex((uint32_t)bar.address);
    ig9_log(" size 0x");
    ig9_log_hex((uint32_t)bar.size);
    ig9_log("\n");

    if (g_api->pci.enable_bus_master != NULL) {
        (void)g_api->pci.enable_bus_master(g_match.bus, g_match.device,
                                           g_match.function);
    }
    if (!ig9_pci_prepare()) {
        ig9_log("[IntelGen9] bringup: PCI prepare failed\n");
        return;
    }

    volatile uint8_t *mmio =
        (volatile uint8_t *)api_map_mmio(bar.address, IG9_MMIO_MAP_BYTES);
    if (mmio == NULL) {
        ig9_log("[IntelGen9] bringup: MMIO map failed\n");
        return;
    }
    g_mmio = mmio;
    ig9_log("[IntelGen9] bringup: MMIO mapped\n");

    ig9_ggtt_detect_size();

    /* The PTE template comes from the firmware's own mapping of the
     * framebuffer: whatever is found there is known to be a working entry.
     * The plane register is authoritative; the table scan is kept for the
     * case where the plane register cannot be read. */
    if (!ig9_fb_find_via_plane()) {
        uint32_t fb_entry = 0u;
        if (g_fb.phys_base != 0u &&
            ig9_ggtt_find_mapping(g_fb.phys_base, &fb_entry)) {
            g_fb.ggtt_offset = fb_entry * IG9_PAGE_BYTES;
            g_fb.ggtt_found = 1;
            ig9_log("[IntelGen9] fb: ggtt entry 0x");
            ig9_log_hex(fb_entry);
            ig9_log("\n");
        } else {
            ig9_log("[IntelGen9] bringup: framebuffer not in GGTT\n");
        }
    }

    if (g_fb.ggtt_found == 0) {
        ig9_log("[IntelGen9] bringup: no framebuffer address, CPU path only\n");
        return;
    }

    if (!ig9_forcewake()) {
        ig9_log("[IntelGen9] bringup: no forcewake ack, CPU path only\n");
        return;
    }
    if (!ig9_ring_init()) {
        ig9_log("[IntelGen9] bringup: ring setup failed\n");
        ig9_forcewake_release();
        return;
    }
    if (!ig9_selftest_ring()) {
        ig9_log("[IntelGen9] bringup: ring selftest failed\n");
        ig9_forcewake_release();
        return;
    }
    ig9_log("[IntelGen9] ring alive\n");

    uint64_t scratch_phys = 0;
    uint8_t *scratch = (uint8_t *)api_dma_alloc(IG9_PAGE_BYTES, &scratch_phys);
    if (scratch == NULL || scratch_phys == 0u) {
        ig9_log("[IntelGen9] bringup: scratch alloc failed\n");
        return;
    }
    api_memset(scratch, 0, IG9_PAGE_BYTES);
    if (!ig9_ggtt_map_pages(scratch, IG9_PAGE_BYTES, &g_scratch_ggtt)) {
        ig9_log("[IntelGen9] bringup: scratch GGTT map failed\n");
        api_dma_free(scratch, IG9_PAGE_BYTES);
        return;
    }
    g_scratch_virt = scratch;
    g_scratch_ready = 1;

    uint64_t src_phys = 0;
    uint8_t *src = (uint8_t *)api_dma_alloc(IG9_PAGE_BYTES, &src_phys);
    if (src != NULL && src_phys != 0u) {
        api_memset(src, 0, IG9_PAGE_BYTES);
        if (ig9_ggtt_map_pages(src, IG9_PAGE_BYTES, &g_copy_src_ggtt)) {
            g_copy_src_virt = src;
            g_copy_src_ready = 1;
        }
    }

    if (!ig9_selftest_fill()) {
        ig9_log("[IntelGen9] bringup: fill selftest failed\n");
        return;
    }
    if (!ig9_selftest_copy()) {
        ig9_log("[IntelGen9] bringup: copy selftest failed\n");
        return;
    }

    g_accel_ok = 1;
    ig9_log("[IntelGen9] blitter verified, accelerated present enabled\n");
}

static void ig9_pci_find_device(void)
{
    if (g_api == NULL || g_api->pci.get_device_count == NULL ||
        g_api->pci.get_device == NULL) {
        ig9_log("[IntelGen9] probe: no PCI client\n");
        return;
    }

    uint32_t count = g_api->pci.get_device_count();
    if (count == 0u) {
        ig9_log("[IntelGen9] probe: PCI device list empty\n");
        return;
    }

    for (uint32_t i = 0; i < count; ++i) {
        driver_pci_device_t dev;
        if (!g_api->pci.get_device(i, &dev)) {
            continue;
        }
        if (dev.vendor_id != IG9_PCI_VENDOR_INTEL) {
            continue;
        }
        /* The allowlist decides, not the class code: firmware has been seen
         * to enumerate the integrated GPU as 0x038000 ("other display
         * controller") instead of the usual 0x030000. */
        for (uint32_t d = 0; d < IG9_DEVICE_COUNT; ++d) {
            if (g_ig9_devices[d].device_id != dev.device_id) {
                continue;
            }
            g_match.bus = dev.bus;
            g_match.device = dev.device;
            g_match.function = dev.function;
            g_match.device_id = dev.device_id;
            g_match.name = g_ig9_devices[d].name;
            g_have_match = 1;
            ig9_log("[IntelGen9] found ");
            ig9_log(g_ig9_devices[d].name);
            ig9_log(" id 0x");
            ig9_log_hex(dev.device_id);
            ig9_log("\n");
            return;
        }
        if (dev.class_code == IG9_PCI_CLASS_DISPLAY) {
            ig9_log("[IntelGen9] display device id 0x");
            ig9_log_hex(dev.device_id);
            ig9_log(" class 0x");
            ig9_log_hex(((uint32_t)dev.class_code << 16) |
                        ((uint32_t)dev.subclass << 8) |
                        (uint32_t)dev.prog_if);
            ig9_log(" not in the allowlist\n");
        }
    }

    ig9_log("[IntelGen9] probe: no supported GPU\n");
}

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    static const driver_module_descriptor_t descriptor = {
        .magic = DRIVER_DESCRIPTOR_MAGIC,
        .version = DRIVER_DESCRIPTOR_VERSION,
        .kind = DEVICE_TYPE_DISPLAY,
        /* Ahead of the generic framebuffer driver (90) in load order: on a
         * machine this driver supports, probe() answers true and its display
         * api is selected; everywhere else probe() is false before any
         * hardware is touched and the generic driver is picked exactly as
         * before. */
        .load_priority = 60u,
        /* Same reason the storage and network drivers take this dependency:
         * the device search needs the PCI client attached, and this module
         * has to be activated after it. */
        .deps = { "PCI_Driver.ELF", NULL },
        .driver_api = &g_ig9_display,
        .shutdown = NULL,
    };

    if (api == NULL || api->malloc == NULL || api->free == NULL ||
        api->dma_alloc == NULL || api->dma_free == NULL ||
        api->virt_to_phys == NULL || api->hw.map_mmio_range == NULL ||
        api->pci.get_device == NULL) {
        return NULL;
    }

    g_api = api;
    /* This runs during the prepare pass, before any module is activated and
     * therefore before the PCI client can answer; the search itself is
     * deferred to probe(). The log stays unconditional so a staged module
     * that never matches is still visible on the serial console. */
    ig9_log("[IntelGen9] module init\n");

    return &descriptor;
}

#undef api_malloc
#undef api_free
#undef api_dma_alloc
#undef api_dma_free
#undef api_virt_to_phys
#undef api_map_mmio
#undef api_memset

#endif /* IMPLUS_DRIVER_MODULE */
