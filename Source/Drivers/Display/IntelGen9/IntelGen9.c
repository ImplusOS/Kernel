/*
 * IntelGen9.c - display driver for Intel Gen9 / Gen9.5 integrated graphics
 * (Skylake, Kaby Lake, Amber Lake, Coffee Lake, Whiskey Lake, Comet Lake,
 * Apollo Lake, Gemini Lake): module entry, device matching, and the
 * driver_display_t implementation.
 *
 * WHAT IT DOES
 *
 * The firmware (UEFI GOP) has already lit a pipe and left a linear 32-bpp
 * framebuffer on it. This driver keeps that mode - it never touches a PLL,
 * port, transcoder or plane register - and takes over everything else:
 *
 *   - It reads the display engine's state (which pipe/plane scans out the
 *     framebuffer, its tiling/format/stride, which DDI port and protocol the
 *     transcoder drives, the real refresh rate measured off the pipe's
 *     frame counter) and reports it through the monitor/mode queries.
 *   - The window manager composes into a driver-owned back buffer; each
 *     present is copied into scanout by the GPU blitter (BCS) as one
 *     XY_SRC_COPY_BLT per damaged rectangle, submitted through execlists
 *     (IG9_Engine.c). If the engine cannot be brought up, fails its
 *     self-tests or hangs, the same rectangles are copied by the CPU
 *     through a write-combining mapping, exactly like the generic driver.
 *
 * Source layout: IG9_Regs.h (registers/encodings), IG9_Uncore.c (MMIO,
 * forcewake, PCI power), IG9_Ggtt.c (GGTT), IG9_Engine.c (BCS), this file.
 * Design notes and the bring-up log to expect on hardware:
 * Docs/Architecture/IntelGen9.md.
 */

#ifdef IMPLUS_DRIVER_MODULE

#include "IG9_Private.h"

/* ------------------------------------------------------------------ *
 * 1. Supported devices (i915 i915_pciids.h: SKL, KBL, AML, CFL, WHL, CML,
 *    BXT/APL, GLK)
 * ------------------------------------------------------------------ */

static const ig9_device_t g_ig9_devices[] = {
    /* --- Skylake (Gen9) --- */
    { 0x1902u, "HD Graphics 510 (Skylake GT1 Desktop)",      IG9_GEN_SKYLAKE },
    { 0x1906u, "HD Graphics 510 (Skylake GT1 Mobile)",       IG9_GEN_SKYLAKE },
    { 0x190Bu, "HD Graphics 510 (Skylake GT1 Server)",       IG9_GEN_SKYLAKE },
    { 0x190Eu, "HD Graphics (Skylake GT1 Desktop)",          IG9_GEN_SKYLAKE },
    { 0x1912u, "HD Graphics 530 (Skylake GT2 Desktop)",      IG9_GEN_SKYLAKE },
    { 0x1916u, "HD Graphics 520 (Skylake GT2 Mobile)",       IG9_GEN_SKYLAKE },
    { 0x191Bu, "HD Graphics 530 (Skylake GT2 Mobile)",       IG9_GEN_SKYLAKE },
    { 0x191Du, "HD Graphics P530 (Skylake GT2 Workstation)", IG9_GEN_SKYLAKE },
    { 0x191Eu, "HD Graphics 515 (Skylake GT2 Mobile Y)",     IG9_GEN_SKYLAKE },
    { 0x1921u, "HD Graphics 520 (Skylake GT2F)",             IG9_GEN_SKYLAKE },
    { 0x1926u, "Iris Graphics 540 (Skylake GT3 Mobile)",     IG9_GEN_SKYLAKE },
    { 0x1927u, "Iris Graphics 550 (Skylake GT3 28W)",        IG9_GEN_SKYLAKE },
    { 0x192Bu, "Iris Graphics 550 (Skylake GT3 Mobile)",     IG9_GEN_SKYLAKE },
    { 0x192Du, "Iris Graphics P555 (Skylake GT3 Server)",    IG9_GEN_SKYLAKE },
    { 0x1932u, "Iris Pro Graphics 580 (Skylake GT4 Desktop)",IG9_GEN_SKYLAKE },
    { 0x193Au, "Iris Pro Graphics P580 (Skylake GT4 Server)", IG9_GEN_SKYLAKE },
    { 0x193Bu, "Iris Pro Graphics 580 (Skylake GT4 Mobile)", IG9_GEN_SKYLAKE },
    { 0x193Du, "Iris Pro Graphics P580 (Skylake GT4 WS)",    IG9_GEN_SKYLAKE },

    /* --- Kaby Lake & Amber Lake (Gen9.5) --- */
    { 0x5902u, "HD Graphics 610 (Kaby Lake GT1 Desktop)",    IG9_GEN_KABYLAKE },
    { 0x5906u, "HD Graphics 610 (Kaby Lake GT1 Mobile)",     IG9_GEN_KABYLAKE },
    { 0x5908u, "HD Graphics (Kaby Lake GT1 Halo)",           IG9_GEN_KABYLAKE },
    { 0x590Au, "HD Graphics (Kaby Lake GT1 Server)",         IG9_GEN_KABYLAKE },
    { 0x590Bu, "HD Graphics (Kaby Lake GT1 Server)",         IG9_GEN_KABYLAKE },
    { 0x590Eu, "HD Graphics (Kaby Lake GT1 Desktop)",        IG9_GEN_KABYLAKE },
    { 0x5912u, "HD Graphics 630 (Kaby Lake GT2 Desktop)",    IG9_GEN_KABYLAKE },
    { 0x5916u, "HD Graphics 620 (Kaby Lake GT2 Mobile)",     IG9_GEN_KABYLAKE },
    { 0x5917u, "UHD Graphics 620 (Kaby Lake-R GT2 Mobile)",  IG9_GEN_KABYLAKE },
    { 0x591Bu, "HD Graphics 630 (Kaby Lake GT2 Mobile H)",   IG9_GEN_KABYLAKE },
    { 0x591Cu, "HD Graphics 615 (Amber Lake Y)",             IG9_GEN_AMBERLAKE },
    { 0x591Du, "HD Graphics P630 (Kaby Lake GT2 WS)",        IG9_GEN_KABYLAKE },
    { 0x591Eu, "HD Graphics 615 (Kaby Lake GT2 Mobile Y)",   IG9_GEN_KABYLAKE },
    { 0x5921u, "Iris Plus Graphics 640 (Kaby Lake GT3)",     IG9_GEN_KABYLAKE },
    { 0x5923u, "Iris Plus Graphics 650 (Kaby Lake GT3 28W)", IG9_GEN_KABYLAKE },
    { 0x5926u, "Iris Plus Graphics 640 (Kaby Lake GT3)",     IG9_GEN_KABYLAKE },
    { 0x5927u, "Iris Plus Graphics 650 (Kaby Lake GT3)",     IG9_GEN_KABYLAKE },
    { 0x593Bu, "Iris Pro Graphics (Kaby Lake GT4 Mobile)",   IG9_GEN_KABYLAKE },
    { 0x87C0u, "UHD Graphics 617 (Amber Lake Y)",            IG9_GEN_AMBERLAKE },
    { 0x87CAu, "UHD Graphics (Amber Lake Y)",                IG9_GEN_AMBERLAKE },

    /* --- Coffee Lake & Whiskey Lake (Gen9.5) --- */
    { 0x3E90u, "UHD Graphics 610 (Coffee Lake)",             IG9_GEN_COFFEELAKE },
    { 0x3E91u, "UHD Graphics 630 (Coffee Lake)",             IG9_GEN_COFFEELAKE },
    { 0x3E92u, "UHD Graphics 630 (Coffee Lake)",             IG9_GEN_COFFEELAKE },
    { 0x3E93u, "UHD Graphics 610 (Coffee Lake)",             IG9_GEN_COFFEELAKE },
    { 0x3E94u, "UHD Graphics 620 (Coffee Lake Mobile)",      IG9_GEN_COFFEELAKE },
    { 0x3E96u, "UHD Graphics 630 (Coffee Lake Server)",      IG9_GEN_COFFEELAKE },
    { 0x3E98u, "UHD Graphics 630 (Coffee Lake)",             IG9_GEN_COFFEELAKE },
    { 0x3E99u, "UHD Graphics 610 (Coffee Lake)",             IG9_GEN_COFFEELAKE },
    { 0x3E9Au, "UHD Graphics P630 (Coffee Lake WS)",         IG9_GEN_COFFEELAKE },
    { 0x3E9Bu, "UHD Graphics 630 (Coffee Lake-H)",           IG9_GEN_COFFEELAKE },
    { 0x3E9Cu, "UHD Graphics 615 (Coffee Lake Y)",           IG9_GEN_COFFEELAKE },
    { 0x3EA0u, "UHD Graphics 620 (Whiskey Lake)",            IG9_GEN_COFFEELAKE },
    { 0x3EA1u, "UHD Graphics 620 (Whiskey Lake U)",          IG9_GEN_COFFEELAKE },
    { 0x3EA2u, "UHD Graphics 620 (Whiskey Lake U)",          IG9_GEN_COFFEELAKE },
    { 0x3EA3u, "UHD Graphics 620 (Whiskey Lake U)",          IG9_GEN_COFFEELAKE },
    { 0x3EA4u, "UHD Graphics 620 (Whiskey Lake U)",          IG9_GEN_COFFEELAKE },
    { 0x3EA5u, "UHD Graphics 655 (Coffee Lake)",             IG9_GEN_COFFEELAKE },
    { 0x3EA6u, "Iris Plus Graphics 655 (Coffee Lake)",       IG9_GEN_COFFEELAKE },
    { 0x3EA7u, "Iris Plus Graphics 655 (Coffee Lake)",       IG9_GEN_COFFEELAKE },
    { 0x3EA8u, "Iris Plus Graphics 655 (Coffee Lake)",       IG9_GEN_COFFEELAKE },

    /* --- Comet Lake (Gen9.5) --- */
    { 0x9B21u, "UHD Graphics 610 (Comet Lake Mobile)",       IG9_GEN_COMETLAKE },
    { 0x9B41u, "UHD Graphics 620 (Comet Lake Mobile)",       IG9_GEN_COMETLAKE },
    { 0x9BA0u, "UHD Graphics 630 (Comet Lake Desktop)",      IG9_GEN_COMETLAKE },
    { 0x9BA2u, "UHD Graphics 630 (Comet Lake Desktop)",      IG9_GEN_COMETLAKE },
    { 0x9BA4u, "UHD Graphics 630 (Comet Lake Desktop)",      IG9_GEN_COMETLAKE },
    { 0x9BA5u, "UHD Graphics 630 (Comet Lake Desktop)",      IG9_GEN_COMETLAKE },
    { 0x9BA8u, "UHD Graphics 630 (Comet Lake Desktop)",      IG9_GEN_COMETLAKE },
    { 0x9BAAu, "UHD Graphics 630 (Comet Lake Desktop)",      IG9_GEN_COMETLAKE },
    { 0x9BC0u, "UHD Graphics 630 (Comet Lake)",              IG9_GEN_COMETLAKE },
    { 0x9BC2u, "UHD Graphics 630 (Comet Lake)",              IG9_GEN_COMETLAKE },
    { 0x9BC4u, "UHD Graphics 630 (Comet Lake)",              IG9_GEN_COMETLAKE },
    { 0x9BC5u, "UHD Graphics 630 (Comet Lake)",              IG9_GEN_COMETLAKE },
    { 0x9BC8u, "UHD Graphics 630 (Comet Lake)",              IG9_GEN_COMETLAKE },
    { 0x9BCAu, "UHD Graphics 630 (Comet Lake)",              IG9_GEN_COMETLAKE },
    { 0x9BCCu, "UHD Graphics 630 (Comet Lake)",              IG9_GEN_COMETLAKE },

    /* --- Broxton / Apollo Lake & Gemini Lake (Gen9 LP) --- */
    { 0x1A84u, "HD Graphics (Broxton)",                      IG9_GEN_BROXTON },
    { 0x1A85u, "HD Graphics (Broxton)",                      IG9_GEN_BROXTON },
    { 0x5A84u, "HD Graphics 505 (Apollo Lake)",              IG9_GEN_BROXTON },
    { 0x5A85u, "HD Graphics 500 (Apollo Lake)",              IG9_GEN_BROXTON },
    { 0x3184u, "UHD Graphics 605 (Gemini Lake)",             IG9_GEN_GEMINILAKE },
    { 0x3185u, "UHD Graphics 600 (Gemini Lake)",             IG9_GEN_GEMINILAKE },
};

#define IG9_DEVICE_COUNT (sizeof(g_ig9_devices) / sizeof(g_ig9_devices[0]))

static bool g_probe_attempted = false;

static void ig9_pci_find_device(void)
{
    const driver_binary_t *api = g_ig9_api;
    if (api == NULL || api->pci.get_device_count == NULL ||
        api->pci.get_device == NULL) {
        ig9_log("[IntelGen9] probe: no PCI client\n");
        return;
    }

    uint32_t count = api->pci.get_device_count();
    for (uint32_t i = 0; i < count; ++i) {
        driver_pci_device_t dev;
        if (!api->pci.get_device(i, &dev) ||
            dev.vendor_id != IG9_PCI_VENDOR_INTEL) {
            continue;
        }
        /* The allowlist decides, not the class code: firmware has been
         * seen to enumerate the iGPU as 0x038000 instead of 0x030000. */
        for (uint32_t d = 0; d < IG9_DEVICE_COUNT; ++d) {
            if (g_ig9_devices[d].device_id != dev.device_id) {
                continue;
            }
            g_ig9_match.bus = dev.bus;
            g_ig9_match.device = dev.device;
            g_ig9_match.function = dev.function;
            g_ig9_match.revision = dev.revision;
            g_ig9_match.device_id = dev.device_id;
            g_ig9_match.name = g_ig9_devices[d].name;
            g_ig9_match.gen = g_ig9_devices[d].gen;
            g_ig9_match.has_llc = g_ig9_devices[d].gen != IG9_GEN_BROXTON &&
                                  g_ig9_devices[d].gen != IG9_GEN_GEMINILAKE;
            g_ig9_have_match = true;
            ig9_log("[IntelGen9] found ");
            ig9_log(g_ig9_devices[d].name);
            ig9_log(" id 0x");
            ig9_log_hex(dev.device_id);
            ig9_log(" rev 0x");
            ig9_log_hex(dev.revision);
            ig9_log("\n");
            return;
        }
        if (dev.class_code == IG9_PCI_CLASS_DISPLAY) {
            ig9_log("[IntelGen9] display device id 0x");
            ig9_log_hex(dev.device_id);
            ig9_log(" class 0x");
            ig9_log_hex(((uint32_t)dev.class_code << 16) |
                        ((uint32_t)dev.subclass << 8) | (uint32_t)dev.prog_if);
            ig9_log(" not in the allowlist\n");
        }
    }
    ig9_log("[IntelGen9] probe: no supported GPU\n");
}

/* ------------------------------------------------------------------ *
 * 2. Display state
 * ------------------------------------------------------------------ */

#define IG9_BPP 4u

typedef struct {
    /* boot framebuffer */
    uint32_t width;
    uint32_t height;
    uint32_t stride_px;
    uint64_t size_bytes;
    uint64_t phys;
    volatile uint8_t *cpu;          /* WC (or UC) CPU mapping of scanout */

    /* driver-owned back buffer the WM draws into */
    uint32_t *back;
    void *back_raw;
    uint64_t back_bytes;
    uint32_t back_ggtt;
    bool back_mapped;

    /* scanout as seen by the GPU */
    uint32_t fb_ggtt;
    bool fb_ggtt_ok;
} ig9_fb_t;

typedef struct {
    bool found;
    uint32_t pipe;
    uint32_t plane;
    uint32_t plane_ctl;
    uint32_t pipe_w;
    uint32_t pipe_h;
    uint32_t output_type;
    char output_name[16];
    uint32_t refresh_millihz;
} ig9_scanout_t;

static ig9_fb_t g_fb;
static ig9_scanout_t g_scanout;
static bool g_ready = false;         /* a framebuffer is set up */
static bool g_initialized = false;   /* probe() matched and init() ran */
static bool g_hw_attempted = false;
static uint32_t g_generation = 1u;
static ig9_spinlock_t g_present_lock;

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

/* ------------------------------------------------------------------ *
 * 3. Reading the firmware's display configuration (read-only)
 * ------------------------------------------------------------------ */

static uint32_t ig9_planes_per_pipe(void)
{
    /* SKL/KBL/CFL: 2 sprite + 1 primary universal planes per pipe;
     * BXT/GLK: 3 + 1. */
    return (g_ig9_match.gen == IG9_GEN_BROXTON ||
            g_ig9_match.gen == IG9_GEN_GEMINILAKE) ? 4u : 3u;
}

/* Which transcoder feeds `pipe`, and what kind of port does it drive? */
static void ig9_detect_output(uint32_t pipe)
{
    static const char ports[] = "ABCDE";
    uint32_t ctl = 0u;
    bool edp = false;

    uint32_t edp_ctl = ig9_read32(IG9_TRANS_DDI_FUNC_CTL_EDP);
    if ((edp_ctl & IG9_TRANS_DDI_ENABLE) != 0u) {
        uint32_t in = (edp_ctl >> IG9_TRANS_DDI_EDP_INPUT_SHIFT) &
                      IG9_TRANS_DDI_EDP_INPUT_MASK;
        uint32_t edp_pipe = (in == 5u) ? 1u : (in == 6u) ? 2u : 0u;
        if (edp_pipe == pipe) {
            ctl = edp_ctl;
            edp = true;
        }
    }
    if (!edp) {
        ctl = ig9_read32(IG9_TRANS_DDI_FUNC_CTL(pipe));
    }

    g_scanout.output_type = DISPLAY_OUTPUT_UNKNOWN;
    ig9_copy_string(g_scanout.output_name, sizeof(g_scanout.output_name),
                    "pipe");
    if ((ctl & IG9_TRANS_DDI_ENABLE) == 0u) {
        return;
    }

    uint32_t port = (ctl >> IG9_TRANS_DDI_PORT_SHIFT) & IG9_TRANS_DDI_PORT_MASK;
    uint32_t mode = (ctl >> IG9_TRANS_DDI_MODE_SHIFT) & IG9_TRANS_DDI_MODE_MASK;
    const char *proto;
    if (edp) {
        proto = "eDP-";
        g_scanout.output_type = DISPLAY_OUTPUT_DISPLAYPORT;
    } else if (mode == IG9_TRANS_DDI_MODE_HDMI) {
        proto = "HDMI-";
        g_scanout.output_type = DISPLAY_OUTPUT_HDMI;
    } else if (mode == IG9_TRANS_DDI_MODE_DVI) {
        proto = "DVI-";
        g_scanout.output_type = DISPLAY_OUTPUT_HDMI;
    } else if (mode == IG9_TRANS_DDI_MODE_DP_SST ||
               mode == IG9_TRANS_DDI_MODE_DP_MST) {
        proto = "DP-";
        g_scanout.output_type = DISPLAY_OUTPUT_DISPLAYPORT;
    } else {
        proto = "DDI-";
    }

    char name[16];
    uint32_t n = 0u;
    for (const char *p = proto; *p != '\0' && n < sizeof(name) - 2u; ++p) {
        name[n++] = *p;
    }
    name[n++] = (port < 5u) ? ports[port] : '?';
    name[n] = '\0';
    ig9_copy_string(g_scanout.output_name, sizeof(g_scanout.output_name), name);
}

/* Measure the refresh rate off the pipe's hardware frame counter. */
static uint32_t ig9_measure_refresh(uint32_t pipe)
{
    static const uint32_t common[] = {
        23976u, 24000u, 25000u, 29970u, 30000u, 48000u, 50000u, 59940u,
        60000u, 72000u, 75000u, 85000u, 90000u, 100000u, 119880u, 120000u,
        144000u, 165000u, 240000u,
    };
    const uint32_t frames = 6u;
    uint64_t start = ig9_now_ns();
    if (start == 0u) {
        return 0u;
    }

    uint32_t reg = IG9_PIPE_FRMCOUNT(pipe);
    uint32_t c0 = ig9_read32(reg);
    uint64_t t0 = 0u;
    uint32_t cs = c0;
    /* Busy-poll: a 1 ms sleep granule would cost ~1% accuracy. Bounded to
     * ~0.4 s even if the counter is frozen. */
    for (;;) {
        uint64_t now = ig9_now_ns();
        cs = ig9_read32(reg);
        if (cs != c0) {
            t0 = now;
            break;
        }
        if (now - start > 60000000u) {
            return 0u;
        }
    }
    uint64_t t1 = 0u;
    for (;;) {
        uint64_t now = ig9_now_ns();
        uint32_t c = ig9_read32(reg);
        if (c - cs >= frames) {
            t1 = now;
            break;
        }
        if (now - start > 400000000u) {
            return 0u;
        }
    }
    if (t1 <= t0) {
        return 0u;
    }
    uint64_t mhz = ((uint64_t)frames * 1000000000000ull) / (t1 - t0);
    if (mhz < 20000u || mhz > 400000u) {
        return 0u;
    }
    /* Snap to the nearest standard rate within 0.5 %. */
    uint64_t best = mhz;
    uint64_t best_diff = UINT64_MAX;
    for (uint32_t i = 0; i < sizeof(common) / sizeof(common[0]); ++i) {
        uint64_t c = common[i];
        uint64_t diff = (mhz > c) ? mhz - c : c - mhz;
        if (diff * 200u <= c && diff < best_diff) {
            best = c;
            best_diff = diff;
        }
    }
    return (uint32_t)best;
}

/*
 * Find the enabled universal plane whose surface is the framebuffer, and
 * check that what it scans out is what this driver will be blitting:
 * linear, XRGB8888, same stride.
 */
static bool ig9_match_plane(uint32_t fb_ggtt, uint32_t pitch)
{
    g_scanout.found = false;
    for (uint32_t pipe = 0; pipe < IG9_PIPE_COUNT; ++pipe) {
        if ((ig9_read32(IG9_PIPECONF(pipe)) & IG9_PIPECONF_ENABLE) == 0u) {
            continue;
        }
        for (uint32_t plane = 1u; plane <= ig9_planes_per_pipe(); ++plane) {
            uint32_t ctl = ig9_read32(IG9_PLANE_CTL(pipe, plane));
            if ((ctl & IG9_PLANE_CTL_ENABLE) == 0u) {
                continue;
            }
            uint32_t surf = ig9_read32(IG9_PLANE_SURF(pipe, plane)) &
                            IG9_PLANE_SURF_ADDR_MASK;
            uint32_t off = ig9_read32(IG9_PLANE_OFFSET(pipe, plane));
            uint32_t stride = (ig9_read32(IG9_PLANE_STRIDE(pipe, plane)) &
                               0x3FFu) * 64u;
            uint64_t start = (uint64_t)surf +
                             (uint64_t)(off >> 16) * stride +
                             (uint64_t)(off & 0xFFFFu) * IG9_BPP;
            if (start != fb_ggtt) {
                continue;
            }

            uint32_t src = ig9_read32(IG9_PIPESRC(pipe));
            g_scanout.found = true;
            g_scanout.pipe = pipe;
            g_scanout.plane = plane;
            g_scanout.plane_ctl = ctl;
            g_scanout.pipe_w = (src >> 16) + 1u;
            g_scanout.pipe_h = (src & 0xFFFFu) + 1u;

            ig9_log("[IntelGen9] scanout: pipe ");
            ig9_log_dec(pipe);
            ig9_log(" plane ");
            ig9_log_dec(plane);
            ig9_log(" ctl 0x");
            ig9_log_hex(ctl);
            ig9_log(" stride ");
            ig9_log_dec(stride);
            ig9_log(" pipe-src ");
            ig9_log_dec(g_scanout.pipe_w);
            ig9_log("x");
            ig9_log_dec(g_scanout.pipe_h);
            ig9_log("\n");

            if ((ctl & IG9_PLANE_CTL_TILED_MASK) != IG9_PLANE_CTL_TILED_LINEAR) {
                ig9_log("[IntelGen9] scanout: plane is tiled; blitter disabled\n");
                return false;
            }
            if ((ctl & IG9_PLANE_CTL_FORMAT_MASK) != IG9_PLANE_CTL_FORMAT_XRGB8888) {
                ig9_log("[IntelGen9] scanout: plane is not 8888; blitter disabled\n");
                return false;
            }
            if (stride != pitch) {
                ig9_log("[IntelGen9] scanout: plane stride differs from GOP pitch; blitter disabled\n");
                return false;
            }
            return true;
        }
    }
    ig9_log("[IntelGen9] scanout: no enabled plane scans out the GOP framebuffer\n");
    return true;   /* not fatal: the fence check below still guards tiling */
}

/* A fence register over the framebuffer would mean the CPU sees a detiled
 * view of a tiled surface; a blit into it would then be garbage. */
static bool ig9_fb_fenced(uint32_t ggtt, uint64_t bytes)
{
    uint64_t lo_addr = ggtt;
    uint64_t hi_addr = (uint64_t)ggtt + bytes;
    for (uint32_t i = 0; i < IG9_FENCE_COUNT; ++i) {
        uint32_t lo = ig9_read32(IG9_FENCE_REG_GEN6_LO(i));
        if ((lo & IG9_FENCE_VALID) == 0u) {
            continue;
        }
        uint32_t hi = ig9_read32(IG9_FENCE_REG_GEN6_HI(i));
        uint64_t start = lo & 0xFFFFF000u;
        uint64_t end = (uint64_t)(hi & 0xFFFFF000u) + IG9_PAGE_BYTES;
        if (start < hi_addr && lo_addr < end) {
            ig9_log_val("scanout: framebuffer covered by fence", i);
            return true;
        }
    }
    return false;
}

/*
 * Where is the framebuffer in the GPU's address space?
 *
 * UEFI GOP on Intel hands out the CPU aperture address (BAR2/GMADR) of the
 * buffer it pinned in the GGTT, so for that case the answer is plain
 * arithmetic: aperture offset == GGTT offset. A framebuffer given by its
 * physical (stolen-memory) address is looked up in the GGTT instead.
 */
static bool ig9_fb_locate(void)
{
    g_fb.fb_ggtt_ok = false;
    uint64_t phys = g_fb.phys;
    uint64_t bytes = g_fb.size_bytes;
    uint32_t pitch = g_fb.stride_px * IG9_BPP;
    uint32_t ggtt = 0u;

    if (g_ig9_bars.gmadr_size != 0u && phys >= g_ig9_bars.gmadr_phys &&
        phys - g_ig9_bars.gmadr_phys + bytes <= g_ig9_bars.gmadr_size) {
        ggtt = (uint32_t)(phys - g_ig9_bars.gmadr_phys);
        if (!ig9_ggtt_range_present(ggtt, bytes)) {
            ig9_log("[IntelGen9] scanout: aperture range not fully mapped\n");
            return false;
        }
        ig9_log_val("scanout: GOP framebuffer via aperture, ggtt", ggtt);
    } else {
        uint32_t index = 0u;
        if (!ig9_ggtt_find_phys(phys, &index)) {
            ig9_log("[IntelGen9] scanout: framebuffer is neither in the aperture nor in the GGTT\n");
            return false;
        }
        ggtt = index * IG9_PAGE_BYTES + (uint32_t)(phys & (IG9_PAGE_BYTES - 1u));
        if (!ig9_ggtt_range_maps(ggtt, phys, bytes)) {
            ig9_log("[IntelGen9] scanout: framebuffer not linearly mapped in the GGTT\n");
            return false;
        }
        ig9_log_val("scanout: framebuffer found in GGTT at", ggtt);
    }

    ig9_log_val("scanout: first PTE (low)",
                (uint32_t)ig9_ggtt_read_pte(ggtt / IG9_PAGE_BYTES));

    if ((ggtt & 63u) != 0u) {
        ig9_log("[IntelGen9] scanout: framebuffer not 64-byte aligned\n");
        return false;
    }
    if (pitch > IG9_BLT_MAX_PITCH || g_fb.width > IG9_BLT_MAX_COORD ||
        g_fb.height > IG9_BLT_MAX_COORD) {
        ig9_log("[IntelGen9] scanout: beyond blitter limits\n");
        return false;
    }
    if (!ig9_match_plane(ggtt, pitch) || ig9_fb_fenced(ggtt, bytes)) {
        return false;
    }

    g_fb.fb_ggtt = ggtt;
    g_fb.fb_ggtt_ok = true;
    return true;
}

static bool ig9_back_buffer_map(void)
{
    if (g_fb.back_mapped) {
        return true;
    }
    if (g_fb.back == NULL) {
        return false;
    }
    if (!ig9_ggtt_map(g_fb.back, 0u, g_fb.back_bytes, &g_fb.back_ggtt)) {
        ig9_log("[IntelGen9] back buffer: GGTT map failed\n");
        return false;
    }
    g_fb.back_mapped = true;
    return true;
}

static void ig9_hw_bringup(void)
{
    if (!g_ig9_have_match || g_hw_attempted) {
        return;
    }
    g_hw_attempted = true;

    if (!ig9_uncore_init() || !ig9_ggtt_init()) {
        ig9_log("[IntelGen9] bringup: CPU present path only\n");
        return;
    }
    if (!ig9_forcewake_get(IG9_FW_GT)) {
        ig9_log("[IntelGen9] bringup: no GT forcewake, CPU present path only\n");
        return;
    }

    bool fb_ok = ig9_fb_locate();
    if (g_scanout.found) {
        ig9_detect_output(g_scanout.pipe);
        g_scanout.refresh_millihz = ig9_measure_refresh(g_scanout.pipe);
        ig9_log("[IntelGen9] output ");
        ig9_log(g_scanout.output_name);
        ig9_log(" refresh ");
        ig9_log_dec(g_scanout.refresh_millihz);
        ig9_log(" mHz\n");
    }

    if (fb_ok && ig9_engine_init() && ig9_back_buffer_map()) {
        ig9_log("[IntelGen9] blitter present enabled (");
        ig9_log(ig9_engine_mode_name());
        ig9_log(")\n");
    } else {
        ig9_log("[IntelGen9] bringup: CPU present path only\n");
    }
    ig9_forcewake_put(IG9_FW_GT);
}

/* ------------------------------------------------------------------ *
 * 4. driver_display_t
 * ------------------------------------------------------------------ */

static void ig9_back_buffer_release(void)
{
    if (g_fb.back == NULL) {
        return;
    }
    bool can_free = true;
    if (g_fb.back_mapped) {
        /* Presents are synchronous and serialized by g_present_lock, so
         * the engine cannot be reading these pages now. */
        can_free = ig9_ggtt_unmap(g_fb.back_ggtt);
    }
    if (can_free) {
        g_ig9_api->free(g_fb.back_raw);
    }
    g_fb.back = NULL;
    g_fb.back_raw = NULL;
    g_fb.back_bytes = 0u;
    g_fb.back_mapped = false;
    g_fb.back_ggtt = 0u;
}

static bool ig9_fb_set(const driver_boot_framebuffer_t *fb)
{
    bool ok = false;
    ig9_spin_lock(&g_present_lock);
    g_ready = false;
    ++g_generation;
    ig9_back_buffer_release();
    ig9_memset(&g_fb, 0, sizeof(g_fb));

    if (fb == NULL || fb->addr == NULL || fb->width == 0u || fb->height == 0u ||
        fb->pixels_per_scan_line < fb->width ||
        (fb->bytes_per_pixel != 0u && fb->bytes_per_pixel != IG9_BPP)) {
        goto out;
    }

    uint64_t pitch = (uint64_t)fb->pixels_per_scan_line * IG9_BPP;
    uint64_t bytes = pitch * fb->height;
    if (bytes == 0u || bytes > 0x40000000ull ||
        (fb->size_bytes != 0u && bytes > fb->size_bytes)) {
        goto out;
    }

    uint64_t phys = (uint64_t)(uintptr_t)fb->addr;
    void *cpu = NULL;
    if (g_ig9_api->hw.map_mmio_wc_range != NULL) {
        cpu = g_ig9_api->hw.map_mmio_wc_range(phys, (size_t)bytes);
    }
    if (cpu == NULL) {
        cpu = g_ig9_api->hw.map_mmio_range(phys, (size_t)bytes);
    }
    if (cpu == NULL) {
        ig9_log("[IntelGen9] fb: map failed\n");
        goto out;
    }

    uint64_t map_bytes = (bytes + IG9_PAGE_BYTES - 1u) &
                         ~(uint64_t)(IG9_PAGE_BYTES - 1u);
    void *raw = g_ig9_api->malloc(map_bytes + IG9_PAGE_BYTES);
    if (raw == NULL) {
        ig9_log("[IntelGen9] fb: back buffer allocation failed\n");
        goto out;
    }
    uint32_t *back = (uint32_t *)(void *)
        (((uintptr_t)raw + IG9_PAGE_BYTES - 1u) &
         ~(uintptr_t)(IG9_PAGE_BYTES - 1u));
    ig9_memset(back, 0, (size_t)map_bytes);

    g_fb.width = fb->width;
    g_fb.height = fb->height;
    g_fb.stride_px = fb->pixels_per_scan_line;
    g_fb.size_bytes = bytes;
    g_fb.phys = phys;
    g_fb.cpu = (volatile uint8_t *)cpu;
    g_fb.back = back;
    g_fb.back_raw = raw;
    g_fb.back_bytes = map_bytes;

    if (g_hw_attempted && ig9_uncore_ready()) {
        /* A new framebuffer after bring-up: re-validate it for the GPU. */
        if (ig9_forcewake_get(IG9_FW_GT)) {
            if (ig9_fb_locate() && ig9_engine_ready()) {
                (void)ig9_back_buffer_map();
            }
            ig9_forcewake_put(IG9_FW_GT);
        }
    }
    g_ready = true;
    ok = true;

out:
    ig9_spin_unlock(&g_present_lock);
    return ok;
}

static bool ig9_probe(void)
{
    if (!g_ig9_have_match && !g_probe_attempted) {
        /* driver_module_init() runs before the PCI client is attached;
         * search the (static) device list the first time probe() runs. */
        g_probe_attempted = true;
        ig9_pci_find_device();
    }
    return g_ig9_have_match;
}

/* The display manager calls set_framebuffer() before probe()/init(), so the
 * hardware bring-up (which needs the framebuffer's address) runs here. A
 * failed bring-up only means the CPU present path is used. */
static bool ig9_init(void)
{
    if (!g_ready || !ig9_probe()) {
        return false;
    }
    ig9_hw_bringup();
    g_initialized = true;
    return true;
}

/* The display selector adopts any driver whose is_ready() is true right
 * after set_framebuffer(), without calling probe(); so "ready" must mean
 * "this machine has a supported GPU and init() ran", not merely "a
 * framebuffer was handed over". */
static bool ig9_is_ready(void)
{
    return g_ready && g_initialized;
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

static uint32_t ig9_refresh(void)
{
    return g_scanout.refresh_millihz != 0u ? g_scanout.refresh_millihz : 60000u;
}

static bool ig9_get_topology(display_topology_t *out)
{
    if (!g_ready || out == NULL) {
        return false;
    }
    ig9_memset(out, 0, sizeof(*out));
    out->generation = g_generation;
    out->monitor_count = 1u;
    out->primary_monitor = 0u;
    out->width = g_fb.width;
    out->height = g_fb.height;
    return true;
}

static bool ig9_get_monitor_info(uint32_t index, display_monitor_info_t *out)
{
    if (!g_ready || out == NULL || index != 0u) {
        return false;
    }
    ig9_memset(out, 0, sizeof(*out));
    out->index = 0u;
    out->flags = DISPLAY_MONITOR_FLAG_CONNECTED | DISPLAY_MONITOR_FLAG_PRIMARY;
    if (!g_scanout.found || g_scanout.refresh_millihz == 0u) {
        out->flags |= DISPLAY_MONITOR_FLAG_SYNTHETIC_MODE;
    }
    out->output_type = g_scanout.found ? g_scanout.output_type
                                       : DISPLAY_OUTPUT_FRAMEBUFFER;
    out->width = g_fb.width;
    out->height = g_fb.height;
    out->refresh_millihz = ig9_refresh();
    out->current_mode = 0u;
    out->mode_count = 1u;
    out->generation = g_generation;
    ig9_copy_string(out->name, sizeof(out->name),
                    g_ig9_have_match ? g_ig9_match.name : "Intel Gen9 Graphics");
    ig9_copy_string(out->output_name, sizeof(out->output_name),
                    g_scanout.found ? g_scanout.output_name : "framebuffer0");
    return true;
}

static bool ig9_get_mode_info(uint32_t monitor, uint32_t mode,
                              display_mode_info_t *out)
{
    if (!g_ready || out == NULL || monitor != 0u || mode != 0u) {
        return false;
    }
    ig9_memset(out, 0, sizeof(*out));
    out->flags = DISPLAY_MODE_FLAG_CURRENT | DISPLAY_MODE_FLAG_PREFERRED;
    if (!g_scanout.found || g_scanout.refresh_millihz == 0u) {
        out->flags |= DISPLAY_MODE_FLAG_SYNTHETIC;
    }
    out->width = g_fb.width;
    out->height = g_fb.height;
    out->stride = g_fb.stride_px;
    out->bits_per_pixel = 32u;
    out->refresh_millihz = ig9_refresh();
    ig9_copy_string(out->name, sizeof(out->name), "Current");
    return true;
}

/* Modesetting (DPLL, DDI buffer, link training) is deliberately not done:
 * the firmware's mode is kept. */
static bool ig9_set_mode(uint32_t monitor, uint32_t mode)
{
    (void)monitor;
    (void)mode;
    return false;
}

static void ig9_draw_pixel(uint32_t x, uint32_t y, uint32_t color)
{
    if (!g_ready || g_fb.back == NULL || x >= g_fb.width || y >= g_fb.height) {
        return;
    }
    g_fb.back[(uint64_t)y * g_fb.stride_px + x] = color;
}

static void ig9_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                          uint32_t color)
{
    if (!g_ready || g_fb.back == NULL || w == 0u || h == 0u ||
        x >= g_fb.width || y >= g_fb.height) {
        return;
    }
    uint32_t x_end = (w > g_fb.width - x) ? g_fb.width : x + w;
    uint32_t y_end = (h > g_fb.height - y) ? g_fb.height : y + h;
    for (uint32_t py = y; py < y_end; ++py) {
        uint32_t *row = g_fb.back + (uint64_t)py * g_fb.stride_px;
        for (uint32_t px = x; px < x_end; ++px) {
            row[px] = color;
        }
    }
}

static bool ig9_clip(const display_rect_t *in, ig9_blit_rect_t *out)
{
    if (in->w == 0u || in->h == 0u) {
        return false;
    }
    int64_t x0 = in->x;
    int64_t y0 = in->y;
    int64_t x1 = x0 + (int64_t)in->w;
    int64_t y1 = y0 + (int64_t)in->h;
    if (x0 < 0) { x0 = 0; }
    if (y0 < 0) { y0 = 0; }
    if (x1 > (int64_t)g_fb.width)  { x1 = (int64_t)g_fb.width; }
    if (y1 > (int64_t)g_fb.height) { y1 = (int64_t)g_fb.height; }
    if (x1 <= x0 || y1 <= y0) {
        return false;
    }
    out->x = (uint16_t)x0;
    out->y = (uint16_t)y0;
    out->w = (uint16_t)(x1 - x0);
    out->h = (uint16_t)(y1 - y0);
    return true;
}

static void ig9_cpu_copy(const ig9_blit_rect_t *r)
{
    uint64_t pitch = (uint64_t)g_fb.stride_px * IG9_BPP;
    for (uint32_t row = 0; row < r->h; ++row) {
        uint64_t off = (uint64_t)(r->y + row) * pitch + (uint64_t)r->x * IG9_BPP;
        ig9_memcpy((void *)(uintptr_t)(g_fb.cpu + off),
                   (const uint8_t *)g_fb.back + off, (size_t)r->w * IG9_BPP);
    }
}

static bool ig9_gpu_usable(void)
{
    return g_fb.fb_ggtt_ok && g_fb.back_mapped && ig9_engine_ready();
}

#define IG9_PRESENT_CHUNK 64u

static void ig9_present_chunk(const ig9_blit_rect_t *rects, uint32_t n)
{
    if (ig9_gpu_usable()) {
        uint64_t pitch = (uint64_t)g_fb.stride_px * IG9_BPP;
        /* The blitter reads memory, not the CPU caches. */
        for (uint32_t i = 0; i < n; ++i) {
            for (uint32_t row = 0; row < rects[i].h; ++row) {
                uint64_t off = (uint64_t)(rects[i].y + row) * pitch +
                               (uint64_t)rects[i].x * IG9_BPP;
                ig9_clflush_range((const uint8_t *)g_fb.back + off,
                                  (uint64_t)rects[i].w * IG9_BPP);
            }
        }
        ig9_surface_t dst = { g_fb.fb_ggtt, (uint32_t)pitch };
        ig9_surface_t src = { g_fb.back_ggtt, (uint32_t)pitch };
        if (ig9_engine_copy_rects(&dst, &src, rects, n)) {
            return;
        }
        ig9_log("[IntelGen9] present: GPU copy failed, CPU fallback\n");
    }
    for (uint32_t i = 0; i < n; ++i) {
        ig9_cpu_copy(&rects[i]);
    }
}

static void ig9_present_rects(const display_rect_t *rects, uint32_t count)
{
    display_rect_t full;
    if (rects == NULL || count == 0u) {
        full.x = 0;
        full.y = 0;
        full.w = g_fb.width;
        full.h = g_fb.height;
        rects = &full;
        count = 1u;
    }

    ig9_spin_lock(&g_present_lock);
    if (!g_ready || g_fb.back == NULL || g_fb.cpu == NULL) {
        ig9_spin_unlock(&g_present_lock);
        return;
    }
    ig9_blit_rect_t chunk[IG9_PRESENT_CHUNK];
    uint32_t n = 0u;
    for (uint32_t i = 0; i < count; ++i) {
        if (!ig9_clip(&rects[i], &chunk[n])) {
            continue;
        }
        if (++n == IG9_PRESENT_CHUNK) {
            ig9_present_chunk(chunk, n);
            n = 0u;
        }
    }
    if (n != 0u) {
        ig9_present_chunk(chunk, n);
    }
    ig9_spin_unlock(&g_present_lock);
}

static void ig9_present(void)
{
    ig9_present_rects(NULL, 0u);
}

static void *ig9_get_framebuffer(void)
{
    return g_fb.back;
}

static void ig9_shutdown(void)
{
    ig9_spin_lock(&g_present_lock);
    ig9_engine_shutdown();
    ig9_spin_unlock(&g_present_lock);
}

static const driver_display_t g_ig9_display = {
    .name = "Intel Gen9 Graphics",
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
 * 5. Module entry
 * ------------------------------------------------------------------ */

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    static const driver_module_descriptor_t descriptor = {
        .magic = DRIVER_DESCRIPTOR_MAGIC,
        .version = DRIVER_DESCRIPTOR_VERSION,
        .kind = DEVICE_TYPE_DISPLAY,
        /* Ahead of the generic framebuffer driver (90): on a supported
         * machine probe() answers true; everywhere else it answers false
         * before any MMIO and the generic driver is picked as before. */
        .load_priority = 60u,
        .deps = { "PCI_Driver.ELF", NULL },
        .driver_api = &g_ig9_display,
        .shutdown = ig9_shutdown,
    };

    if (api == NULL || api->malloc == NULL || api->free == NULL ||
        api->dma_alloc == NULL || api->dma_free == NULL ||
        api->virt_to_phys == NULL || api->hw.map_mmio_range == NULL ||
        api->pci.get_device == NULL) {
        return NULL;
    }

    g_ig9_api = api;
    ig9_log("[IntelGen9] module init\n");
    return &descriptor;
}

#endif /* IMPLUS_DRIVER_MODULE */
