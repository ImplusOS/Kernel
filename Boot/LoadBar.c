#include "LoadBar.h"
#include "kernel/boot_info.h"
#include <stdint.h>
#include <stdbool.h>

#define SPINNER_RADIUS_MIN   10u
#define SPINNER_RADIUS_MAX   15u
#define SPINNER_THICKNESS    5u
#define SPINNER_MARGIN_PX    48u
#define SPINNER_UPDATE_TICKS 1u
#define SPINNER_MAX_DRAIN_TICKS 4u


#define COLOR_BG       0x000000u
#define COLOR_TRACK    0x1A1A1Au
#define COLOR_PRIMARY  0x00A3FFu
#define COLOR_SECONDARY 0xFFFFFFu

static uint32_t g_center_x = 0;
static uint32_t g_center_y = 0;
static uint32_t g_radius   = 0;
static uint32_t g_box_x = 0;
static uint32_t g_box_y = 0;
static uint32_t g_box_w = 0;
static uint32_t g_box_h = 0;

#define MAX_BOX_SIZE 64
static uint32_t g_spinner_backbuffer[MAX_BOX_SIZE * MAX_BOX_SIZE];

static volatile uint8_t  g_ready = 0;
static volatile uint32_t g_tick_accum = 0;
static volatile uint32_t g_pending_ticks = 0;
static volatile uint32_t g_draw_busy = 0;

static uint32_t g_anim_tick = 0;
static uint32_t g_head = 2800;
static uint32_t g_tail = 2700;

static uint32_t g_screen_w = 0;
static uint32_t g_screen_h = 0;
static uint32_t* g_framebuffer = NULL;
static uint32_t g_pixels_per_scanline = 0;

static const uint16_t k_sin_90[91] = {
    0, 571, 1143, 1714, 2285, 2855, 3425, 3993, 4560, 5126,
    5690, 6252, 6812, 7370, 7927, 8480, 9031, 9580, 10125, 10668,
    11207, 11743, 12275, 12803, 13327, 13848, 14364, 14876, 15383, 15886,
    16383, 16876, 17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621,
    21062, 21497, 21925, 22347, 22762, 23169, 23571, 23964, 24351, 24730,
    25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 28087,
    28377, 28659, 28932, 29196, 29451, 29697, 29934, 30163, 30381, 30591,
    30791, 30982, 31163, 31335, 31498, 31650, 31794, 31927, 32051, 32165,
    32269, 32364, 32448, 32523, 32587, 32642, 32687, 32722, 32747, 32763,
    32768
};




static int32_t get_sin(uint32_t deg) {
    deg = deg % 360;
    if (deg <= 90) return k_sin_90[deg];
    if (deg <= 180) return k_sin_90[180 - deg];
    if (deg <= 270) return -k_sin_90[deg - 180];
    return -k_sin_90[360 - deg];
}

static int32_t get_cos(uint32_t deg) {
    return get_sin(deg + 90);
}

static void spinner_clear_box(void) {
    for (uint32_t i = 0; i < g_box_w * g_box_h; ++i) {
        g_spinner_backbuffer[i] = COLOR_BG;
    }
}

static void flush_backbuffer(void) {
    uint32_t* fb = g_framebuffer;

    if (fb != NULL) {
        for (uint32_t j = 0; j < g_box_h; ++j) {
            uint32_t* dst_row = fb + (g_box_y + j) * g_pixels_per_scanline + g_box_x;
            uint32_t* src_row = g_spinner_backbuffer + j * g_box_w;
            for (uint32_t i = 0; i < g_box_w; ++i) {
                dst_row[i] = src_row[i];
            }
        }
    }
}

static void fill_circle_fast(int32_t cx, int32_t cy, int32_t r, uint32_t color, uint32_t intensity) {
    if (intensity == 0) return;
    
    int32_t r2 = r * r;
    uint32_t r_comp = (color >> 16) & 0xFF;
    uint32_t g_comp = (color >> 8) & 0xFF;
    uint32_t b_comp = color & 0xFF;

    for (int32_t y = -r; y <= r; y++) {
        int32_t py = cy + y;
        if (py < (int32_t)g_box_y || py >= (int32_t)(g_box_y + g_box_h)) continue;
        
        int32_t y2 = y * y;
        for (int32_t x = -r; x <= r; x++) {
            int32_t d2 = x * x + y2;
            if (d2 <= r2) {
                int32_t px = cx + x;
                if (px >= (int32_t)g_box_x && px < (int32_t)(g_box_x + g_box_w)) {
                    uint32_t alpha = 255;
                    if (d2 > (r2 * 64) / 100) { 
                        alpha = 255 - (255 * (d2 - (r2 * 64) / 100)) / (r2 - (r2 * 64) / 100);
                    }
                    alpha = (alpha * intensity) / 255;
                    
                    uint32_t buf_idx = (py - g_box_y) * g_box_w + (px - g_box_x);
                    if (alpha > 200) {
                        g_spinner_backbuffer[buf_idx] = color;
                    } else if (alpha > 0) {
                        uint32_t fr = (r_comp * alpha) >> 8;
                        uint32_t fg = (g_comp * alpha) >> 8;
                        uint32_t fb = (b_comp * alpha) >> 8;
                        g_spinner_backbuffer[buf_idx] = (fr << 16) | (fg << 8) | fb;
                    }
                }
            }
        }
    }
}

static void draw_track_fast(void) {
    uint32_t r_outer = g_radius + SPINNER_THICKNESS / 2;
    uint32_t r_inner = g_radius - SPINNER_THICKNESS / 2;
    uint32_t r_outer2 = r_outer * r_outer;
    uint32_t r_inner2 = r_inner * r_inner;

    for (int32_t y = -(int32_t)r_outer; y <= (int32_t)r_outer; y++) {
        int32_t py = (int32_t)g_center_y + y;
        if (py < (int32_t)g_box_y || py >= (int32_t)(g_box_y + g_box_h)) continue;
        
        int32_t y2 = y * y;
        for (int32_t x = -(int32_t)r_outer; x <= (int32_t)r_outer; x++) {
            int32_t d2 = x * x + y2;
            if (d2 >= r_inner2 && d2 <= r_outer2) {
                int32_t px = (int32_t)g_center_x + x;
                if (px >= (int32_t)g_box_x && px < (int32_t)(g_box_x + g_box_w)) {
                    uint32_t buf_idx = (py - g_box_y) * g_box_w + (px - g_box_x);
                    g_spinner_backbuffer[buf_idx] = COLOR_TRACK;
                }
            }
        }
    }
}

static void spinner_draw(void) {
    if (!g_ready) return;

    spinner_clear_box();
    draw_track_fast();

    uint32_t dist = (g_head >= g_tail) ? (g_head - g_tail) : (36000 + g_head - g_tail);
    if (dist == 0) {
        dist = 1;
    }
    int32_t cr_base = SPINNER_THICKNESS / 2;

    uint32_t step = 120; 
    for (uint32_t offset = 0; offset <= dist; offset += step) {
        uint32_t a = (g_tail + offset) % 36000;
        uint32_t deg = a / 100;
        
        int32_t px = (int32_t)g_center_x + ((int32_t)g_radius * get_cos(deg)) / 32768;
        int32_t py = (int32_t)g_center_y + ((int32_t)g_radius * get_sin(deg)) / 32768;
        
        uint32_t intensity = (255 * offset) / dist;
        if (intensity < 40) intensity = 40; 

        uint32_t cr = (cr_base * (128 + (127 * offset) / dist)) / 255;
        if (cr < 2) cr = 2;

        uint32_t r_p = (COLOR_PRIMARY >> 16) & 0xFF;
        uint32_t g_p = (COLOR_PRIMARY >> 8) & 0xFF;
        uint32_t b_p = COLOR_PRIMARY & 0xFF;
        
        uint32_t r_s = (COLOR_SECONDARY >> 16) & 0xFF;
        uint32_t g_s = (COLOR_SECONDARY >> 8) & 0xFF;
        uint32_t b_s = COLOR_SECONDARY & 0xFF;

        uint32_t r = (r_p * (dist - offset) + r_s * offset) / dist;
        uint32_t g = (g_p * (dist - offset) + g_s * offset) / dist;
        uint32_t b = (b_p * (dist - offset) + b_s * offset) / dist;
        uint32_t color = (r << 16) | (g << 8) | b;

        fill_circle_fast(px, py, cr, color, intensity);
    }

    flush_backbuffer();
}

static bool spinner_step(void) {
    if (!g_ready) return false;

    g_tick_accum++;
    if (g_tick_accum < SPINNER_UPDATE_TICKS) return false;
    g_tick_accum = 0;

    uint32_t phase = g_anim_tick % 180;

    uint32_t base_spd = 120;
    g_head += base_spd;
    g_tail += base_spd;

    int32_t extra = get_sin(phase * 2);
    if (extra > 0) {
        g_head += (extra / 64);
    } else {
        g_tail -= (extra / 64);
    }

    g_head %= 36000;
    g_tail %= 36000;
    g_anim_tick++;
    return true;
}

static void spinner_update_once(void) {
    if (!g_ready) {
        return;
    }

    if (__atomic_exchange_n(&g_draw_busy, 1u, __ATOMIC_ACQUIRE) != 0u) {
        return;
    }

    if (spinner_step()) {
        spinner_draw();
    }

    __atomic_store_n(&g_draw_busy, 0u, __ATOMIC_RELEASE);
}

void load_bar_init(BOOT_INFO* boot_info) {
    if (!boot_info || boot_info->FrameBufferBase == 0) return;

    g_framebuffer = (uint32_t*)(uintptr_t)boot_info->FrameBufferBase;
    g_screen_w = boot_info->HorizontalResolution;
    g_screen_h = boot_info->VerticalResolution;
    g_pixels_per_scanline = boot_info->PixelsPerScanLine;

    if (g_screen_w == 0u || g_screen_h == 0u) return;

    uint32_t max_r = (g_screen_w < g_screen_h ? g_screen_w : g_screen_h) / 8u;
    if (max_r < SPINNER_RADIUS_MIN) max_r = SPINNER_RADIUS_MIN;
    if (max_r > SPINNER_RADIUS_MAX) max_r = SPINNER_RADIUS_MAX;

    g_radius = max_r;
    g_center_x = g_screen_w / 2u;
    g_center_y = (g_screen_h * 3u) / 4u;
    if (g_center_y + g_radius + SPINNER_MARGIN_PX > g_screen_h) {
        g_center_y = g_screen_h - g_radius - SPINNER_MARGIN_PX;
    }

    uint32_t diameter = g_radius * 2u + SPINNER_THICKNESS * 2u + 4u;
    g_box_w = diameter;
    g_box_h = diameter;
    g_box_x = g_center_x - (diameter / 2u);
    g_box_y = g_center_y - (diameter / 2u);

    g_anim_tick = 0;
    g_head = 28000;
    g_tail = 27000;
    g_tick_accum = 0;
    g_pending_ticks = 0;
    g_draw_busy = 0;
    g_ready = 1;

    spinner_draw();
}

void load_bar_set_target(uint32_t percent) {
    (void)percent;
}

void load_bar_update(void) {
    uint32_t ticks = __atomic_exchange_n(&g_pending_ticks, 0u, __ATOMIC_ACQUIRE);
    if (ticks == 0) {
        ticks = 1;
    } else if (ticks > SPINNER_MAX_DRAIN_TICKS) {
        ticks = SPINNER_MAX_DRAIN_TICKS;
    }

    while (ticks-- != 0u) {
        spinner_update_once();
    }
}

void load_bar_tick(uint64_t tick) {
    (void)tick;
    __atomic_add_fetch(&g_pending_ticks, 1u, __ATOMIC_RELEASE);
}

void load_bar_finish(void) {
    if (!g_ready) return;
    if (__atomic_exchange_n(&g_draw_busy, 1u, __ATOMIC_ACQUIRE) != 0u) {
        g_ready = 0;
        return;
    }
    spinner_clear_box();
    flush_backbuffer();
    g_ready = 0;
    __atomic_store_n(&g_draw_busy, 0u, __ATOMIC_RELEASE);
}
