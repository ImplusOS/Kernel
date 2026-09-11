#include "BootAnim.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "kernel/config.h"
#include "MemoryManagement/Memory_Main.h"
#include "Core/timer/Timer.h"

/* 100% -> 150% over BOOT_ANIM_DURATION_MS, paced at ~60 Hz. The loop is
 * driven by the monotonic clock rather than by a frame counter, so a slow
 * blit (a 4K panel on a cold cache) costs frames, never wall time. */
#define BOOT_ANIM_DURATION_MS   OS_CONFIG_BOOT_FADE_MS
#define BOOT_ANIM_FRAME_MS      16u
#define BOOT_ANIM_SCALE_END_Q16 98304u   /* 1.5 in Q16 */
#define BOOT_ANIM_PAGE_SIZE     4096u
#define Q16_ONE                 65536u

/* Cubic ease-in-out, Q16 in / Q16 out:
 *   t < 0.5 : 4t^3
 *   t >= 0.5: 1 - 4(1-t)^3
 * Both halves are evaluated in 64-bit so the cubing cannot overflow. */
static uint32_t boot_anim_ease_in_out(uint32_t t_q16)
{
    if (t_q16 >= Q16_ONE) {
        return Q16_ONE;
    }

    uint64_t v = (t_q16 < (Q16_ONE / 2u)) ? (uint64_t)t_q16
                                          : (uint64_t)(Q16_ONE - t_q16);
    uint64_t cube = (v * v) >> 16;
    cube = (cube * v) >> 16;
    cube *= 4u;

    if (t_q16 < (Q16_ONE / 2u)) {
        return (uint32_t)cube;
    }
    return (uint32_t)(Q16_ONE - cube);
}

/* Per-channel opacity table for one frame: lut[v] = v * alpha / 255.
 *
 * The old fade multiplied every channel of every pixel out of line; a 256-entry
 * table turns that into three loads per pixel and keeps the rounding identical
 * across the whole frame (no banding drift between rows). */
static void boot_anim_build_alpha_lut(uint8_t lut[256], uint32_t alpha)
{
    for (uint32_t v = 0; v < 256u; ++v) {
        lut[v] = (uint8_t)((v * alpha + 127u) / 255u);
    }
}

static void boot_anim_fill_black(uint32_t *fb,
                                 uint32_t stride,
                                 uint32_t width,
                                 uint32_t height)
{
    for (uint32_t y = 0; y < height; ++y) {
        uint32_t *row = fb + (uint64_t)y * stride;
        for (uint32_t x = 0; x < width; ++x) {
            row[x] = 0u;
        }
    }
}

/* Draw the snapshot scaled about the screen centre by scale_q16 and dimmed to
 * `alpha`/255. Nearest-neighbour sampling: the source is only ever magnified,
 * so there is nothing to filter away, and the per-row source address plus the
 * shared x -> source-x table keep the inner loop to a load, three table
 * lookups and a store. */
static void boot_anim_draw_frame(uint32_t *fb,
                                 uint32_t stride,
                                 const uint32_t *snapshot,
                                 int32_t *x_map,
                                 uint32_t width,
                                 uint32_t height,
                                 uint32_t scale_q16,
                                 uint32_t alpha)
{
    if (alpha == 0u || scale_q16 == 0u) {
        boot_anim_fill_black(fb, stride, width, height);
        return;
    }

    uint32_t inv_q16 = (uint32_t)(((uint64_t)Q16_ONE << 16) / scale_q16);
    int32_t center_x = (int32_t)(width / 2u);
    int32_t center_y = (int32_t)(height / 2u);

    for (int32_t x = 0; x < (int32_t)width; ++x) {
        int64_t src = (int64_t)center_x +
                      (((int64_t)(x - center_x) * (int64_t)inv_q16) >> 16);
        if (src < 0) {
            src = 0;
        } else if (src >= (int64_t)width) {
            src = (int64_t)width - 1;
        }
        x_map[x] = (int32_t)src;
    }

    uint8_t lut[256];
    boot_anim_build_alpha_lut(lut, alpha);

    for (int32_t y = 0; y < (int32_t)height; ++y) {
        int64_t src_y = (int64_t)center_y +
                        (((int64_t)(y - center_y) * (int64_t)inv_q16) >> 16);
        if (src_y < 0) {
            src_y = 0;
        } else if (src_y >= (int64_t)height) {
            src_y = (int64_t)height - 1;
        }

        const uint32_t *src_row = snapshot + (uint64_t)src_y * width;
        uint32_t *dst_row = fb + (uint64_t)y * stride;

        if (alpha == 255u) {
            for (uint32_t x = 0; x < width; ++x) {
                dst_row[x] = src_row[x_map[x]];
            }
            continue;
        }

        for (uint32_t x = 0; x < width; ++x) {
            uint32_t p = src_row[x_map[x]];
            dst_row[x] = ((uint32_t)lut[(p >> 16) & 0xFFu] << 16) |
                         ((uint32_t)lut[(p >> 8) & 0xFFu] << 8) |
                         ((uint32_t)lut[p & 0xFFu]);
        }
    }
}

bool boot_anim_play_handoff(const BOOT_INFO *boot_info)
{
    if (boot_info == NULL || boot_info->FrameBufferBase == 0) {
        return false;
    }

    uint32_t width = boot_info->HorizontalResolution;
    uint32_t height = boot_info->VerticalResolution;
    uint32_t stride = boot_info->PixelsPerScanLine;
    if (width == 0u || height == 0u) {
        return false;
    }
    if (stride < width) {
        stride = width;
    }
    if ((uint64_t)stride * height * sizeof(uint32_t) > boot_info->FrameBufferSize) {
        return false;
    }

    uint32_t *fb = (uint32_t *)(uintptr_t)boot_info->FrameBufferBase;

    /* The snapshot is stored packed (stride == width) so that a panel with a
     * long scanline padding does not cost us the padding twice, and it plus
     * the x-map share one run of whole pages taken straight from the physical
     * allocator.
     *
     * Deliberately not malloc(): a panel-sized buffer is several megabytes,
     * and this runs in the window between the last kernel service coming up
     * and process_register_boot_process() allocating the init process's
     * 128 KiB kernel stack. Cycling that much through the kernel heap right
     * there re-shapes the free list the very next allocation is carved out
     * of, for a buffer that is one-shot, page-sized by nature and wanted for
     * under half a second. The page allocator hands it back untouched. */
    uint64_t snapshot_bytes = (uint64_t)width * height * sizeof(uint32_t);
    uint64_t x_map_bytes = (uint64_t)width * sizeof(int32_t);
    uint64_t total_bytes = snapshot_bytes + x_map_bytes;
    uint64_t page_count64 = (total_bytes + BOOT_ANIM_PAGE_SIZE - 1u) /
                            BOOT_ANIM_PAGE_SIZE;
    if (page_count64 == 0u || page_count64 > UINT32_MAX) {
        return false;
    }

    uint32_t page_count = (uint32_t)page_count64;
    uint32_t *snapshot = (uint32_t *)alloc_contiguous_pages(page_count, 1u);
    if (snapshot == NULL) {
        return false;
    }

    /* x_map lives in the tail of the same run, past the snapshot's own bytes. */
    int32_t *x_map = (int32_t *)(void *)((uint8_t *)snapshot + snapshot_bytes);

    for (uint32_t y = 0; y < height; ++y) {
        memcpy(snapshot + (uint64_t)y * width,
               fb + (uint64_t)y * stride,
               (size_t)width * sizeof(uint32_t));
    }

    uint64_t start_ns = timer_monotonic_ns();
    uint64_t duration_ns = (uint64_t)BOOT_ANIM_DURATION_MS * 1000000ull;

    for (;;) {
        uint64_t frame_start_ns = timer_monotonic_ns();
        uint64_t elapsed_ns = (frame_start_ns > start_ns)
                                  ? (frame_start_ns - start_ns)
                                  : 0ull;

        uint32_t t_q16 = (elapsed_ns >= duration_ns)
                             ? Q16_ONE
                             : (uint32_t)((elapsed_ns << 16) / duration_ns);
        uint32_t eased = boot_anim_ease_in_out(t_q16);

        uint32_t scale_q16 =
            Q16_ONE + (uint32_t)(((uint64_t)(BOOT_ANIM_SCALE_END_Q16 - Q16_ONE) *
                                  eased) >> 16);
        uint32_t alpha = 255u - (uint32_t)(((uint64_t)eased * 255u) >> 16);

        boot_anim_draw_frame(fb, stride, snapshot, x_map,
                             width, height, scale_q16, alpha);

        if (t_q16 >= Q16_ONE) {
            break;
        }

        uint64_t spent_ns = timer_monotonic_ns() - frame_start_ns;
        uint64_t budget_ns = (uint64_t)BOOT_ANIM_FRAME_MS * 1000000ull;
        if (spent_ns < budget_ns) {
            timer_apic_sleep_ms((uint32_t)((budget_ns - spent_ns) / 1000000ull));
        }
    }

    /* Leave the panel black: the init process picks the transition up from
     * there with its own 50% -> 100% half. */
    boot_anim_fill_black(fb, stride, width, height);

    free_contiguous_pages(snapshot, page_count);
    return true;
}
