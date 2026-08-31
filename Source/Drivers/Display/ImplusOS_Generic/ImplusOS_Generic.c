#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "Platform/io/IO_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include "../Display_Driver.h"
#ifndef IMPLUS_DRIVER_MODULE
#include "mmu/Paging_Main.h"
#include "Drivers/Module/DriverSelect.h"
#endif

#include "Drivers/Module/DriverBinary.h"

#define GENERIC_FB_DEVICE_NAME "Generic Framebuffer"
#define GENERIC_FB_BYTES_PER_PIXEL 4U
#define GENERIC_FB_MAP_GRANULE (2ULL * 1024ULL * 1024ULL)
#define GENERIC_FB_MAX_MAPPINGS 1028

#ifdef IMPLUS_DRIVER_MODULE
static const driver_binary_t *g_driver_api = NULL;

#define malloc g_driver_api->malloc
#define free g_driver_api->free
#define map_mmio_virt g_driver_api->map_mmio_virt
#define memcpy g_driver_api->memcpy
#define memset g_driver_api->memset

static int driver_module_strcmp(const char *a, const char *b) {
    while (*a && *a == *b) {
        ++a;
        ++b;
    }
    return (int)(unsigned char)(*a) - (int)(unsigned char)(*b);
}
#define strcmp driver_module_strcmp

#else
#include <string.h>
#endif

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t pixels_per_scan_line;
    uint64_t size_bytes;
    uint64_t map_offset_bytes;
    uint32_t mapped_chunks;
    uint8_t *mapped_bases[GENERIC_FB_MAX_MAPPINGS];
} framebuffer_t;

typedef struct {
    uint32_t *buffer;
    uint64_t buffer_size;
} double_buffer_t;

static framebuffer_t g_fb;
static double_buffer_t g_double_buffer = {
    .buffer = NULL,
    .buffer_size = 0u,
};
static int g_ready = 0;
static uint32_t g_generation = 1u;

static void generic_copy_string(char *dst, uint32_t dst_size, const char *src) {
    if (dst == NULL || dst_size == 0u) {
        return;
    }
    uint32_t i = 0u;
    if (src != NULL) {
        while (i + 1u < dst_size && src[i] != '\0') {
            dst[i] = src[i];
            ++i;
        }
    }
    dst[i] = '\0';
}

static void generic_publish_display_event(uint16_t action, const char *detail) {
#ifdef IMPLUS_DRIVER_MODULE
    if (g_driver_api == NULL || g_driver_api->pnp_notify == NULL) {
        return;
    }

    pnp_event_t event;
    pnp_event_init(&event,
                   action,
                   PNP_BUS_DISPLAY,
                   PNP_CLASS_DISPLAY,
                   "ImplusOS_Generic_Display_Driver.ELF",
                   "ImplusOS Generic Display",
                   detail);
    event.location0 = g_fb.width;
    event.location1 = g_fb.height;
    g_driver_api->pnp_notify(&event);
#else
    (void)action;
    (void)detail;
#endif
}

static bool map_framebuffer_chunks(uint64_t phys_addr, uint64_t size_bytes) {
    if (size_bytes == 0) {
        return false;
    }

    uint64_t base_phys = phys_addr & ~(GENERIC_FB_MAP_GRANULE - 1ULL);
    uint64_t base_offset = phys_addr - base_phys;
    uint64_t total_span = base_offset + size_bytes;
    uint64_t chunks_u64 =
        (total_span + GENERIC_FB_MAP_GRANULE - 1ULL) / GENERIC_FB_MAP_GRANULE;

    if (chunks_u64 == 0 || chunks_u64 > GENERIC_FB_MAX_MAPPINGS) {
        return false;
    }

    for (uint32_t i = 0; i < GENERIC_FB_MAX_MAPPINGS; ++i) {
        g_fb.mapped_bases[i] = NULL;
    }

    for (uint32_t i = 0; i < (uint32_t)chunks_u64; ++i) {
        uint64_t chunk_phys = base_phys + ((uint64_t)i * GENERIC_FB_MAP_GRANULE);
        void *chunk_virt = map_mmio_virt(chunk_phys);
        if (chunk_virt == NULL) {
            return false;
        }
        g_fb.mapped_bases[i] = (uint8_t *)chunk_virt;
    }

    g_fb.map_offset_bytes = base_offset;
    g_fb.mapped_chunks = (uint32_t)chunks_u64;
    return true;
}

static inline volatile uint8_t *resolve_pixel_addr(uint64_t pixel_index) {
    uint64_t byte_offset =
        g_fb.map_offset_bytes + (pixel_index * (uint64_t)GENERIC_FB_BYTES_PER_PIXEL);
    uint64_t chunk_index = byte_offset / GENERIC_FB_MAP_GRANULE;
    if (chunk_index >= g_fb.mapped_chunks) {
        return NULL;
    }

    uint64_t chunk_offset = byte_offset % GENERIC_FB_MAP_GRANULE;
    return (volatile uint8_t *)(g_fb.mapped_bases[chunk_index] + chunk_offset);
}

bool generic_fb_set(const driver_boot_framebuffer_t *framebuffer) {
    int was_ready = g_ready;

    if (framebuffer == NULL && was_ready) {
        generic_publish_display_event(PNP_EVENT_DEVICE_REMOVED,
                                      "boot framebuffer detached");
    }

    g_ready = 0;
    g_fb.mapped_chunks = 0;
    ++g_generation;

    if (g_double_buffer.buffer != NULL) {
        free(g_double_buffer.buffer);
        g_double_buffer.buffer = NULL;
        g_double_buffer.buffer_size = 0u;
    }

    if (!framebuffer || !framebuffer->addr ||
        framebuffer->width == 0 || framebuffer->height == 0 ||
        framebuffer->pixels_per_scan_line < framebuffer->width) {
        if (framebuffer == NULL) {
            memset(&g_fb, 0, sizeof(g_fb));
        }
        return false;
    }

    uint32_t bytes_per_pixel = framebuffer->bytes_per_pixel;
    if (bytes_per_pixel == 0) {
        bytes_per_pixel = GENERIC_FB_BYTES_PER_PIXEL;
    }
    if (bytes_per_pixel != GENERIC_FB_BYTES_PER_PIXEL) {
        return false;
    }

    uint64_t required_bytes =
        (uint64_t)framebuffer->pixels_per_scan_line *
        (uint64_t)framebuffer->height *
        (uint64_t)bytes_per_pixel;
    if (framebuffer->size_bytes != 0 &&
        required_bytes > framebuffer->size_bytes) {
        return false;
    }

    if ((((uint64_t)(uintptr_t)framebuffer->addr) &
         (GENERIC_FB_BYTES_PER_PIXEL - 1ULL)) != 0ULL) {
        return false;
    }

    if (!map_framebuffer_chunks((uint64_t)(uintptr_t)framebuffer->addr, required_bytes)) {
        return false;
    }

    g_fb.width = framebuffer->width;
    g_fb.height = framebuffer->height;
    g_fb.pixels_per_scan_line = framebuffer->pixels_per_scan_line;
    g_fb.size_bytes = required_bytes;

    uint64_t buffer_size = (uint64_t)g_fb.pixels_per_scan_line * g_fb.height * sizeof(uint32_t);
    uint32_t *buffer = (uint32_t *)malloc(buffer_size);
    if (buffer == NULL) {
        return false;
    }

    for (uint64_t i = 0; i < buffer_size / sizeof(uint32_t); ++i) {
        buffer[i] = 0;
    }

    g_double_buffer.buffer = buffer;
    g_double_buffer.buffer_size = buffer_size;

    g_ready = 1;
    generic_publish_display_event(was_ready ?
                                  PNP_EVENT_DEVICE_CHANGED :
                                  PNP_EVENT_DEVICE_ADDED,
                                  was_ready ?
                                  "boot framebuffer changed" :
                                  "boot framebuffer ready");

    return true;
}

bool fb_probe(void) {
    return g_ready != 0;
}

bool fb_init(void) {
    return g_ready != 0;
}

bool fb_is_ready(void) {
    return g_ready != 0;
}

uint32_t fb_width(void) {
    return g_fb.width;
}

uint32_t fb_height(void) {
    return g_fb.height;
}

uint32_t fb_generation(void) {
    return g_generation;
}

bool fb_poll_config(void) {
    return false;
}

uint32_t fb_monitor_count(void) {
    return g_ready ? 1u : 0u;
}

bool fb_get_topology(display_topology_t *out_topology) {
    if (!g_ready || out_topology == NULL) {
        return false;
    }
    memset(out_topology, 0, sizeof(*out_topology));
    out_topology->generation = g_generation;
    out_topology->monitor_count = 1u;
    out_topology->primary_monitor = 0u;
    out_topology->origin_x = 0;
    out_topology->origin_y = 0;
    out_topology->width = g_fb.width;
    out_topology->height = g_fb.height;
    return true;
}

bool fb_get_monitor_info(uint32_t monitor_index, display_monitor_info_t *out_info) {
    if (!g_ready || out_info == NULL || monitor_index != 0u) {
        return false;
    }
    memset(out_info, 0, sizeof(*out_info));
    out_info->index = 0u;
    out_info->id = 0u;
    out_info->flags = DISPLAY_MONITOR_FLAG_CONNECTED |
                      DISPLAY_MONITOR_FLAG_PRIMARY |
                      DISPLAY_MONITOR_FLAG_SYNTHETIC_MODE;
    out_info->output_type = DISPLAY_OUTPUT_FRAMEBUFFER;
    out_info->x = 0;
    out_info->y = 0;
    out_info->width = g_fb.width;
    out_info->height = g_fb.height;
    out_info->refresh_millihz = 60000u;
    out_info->current_mode = 0u;
    out_info->mode_count = 1u;
    out_info->generation = g_generation;
    generic_copy_string(out_info->name, sizeof(out_info->name),
                        "ImplusOS Generic Display");
    generic_copy_string(out_info->output_name, sizeof(out_info->output_name),
                        "framebuffer0");
    return true;
}

bool fb_get_mode_info(uint32_t monitor_index, uint32_t mode_index,
                      display_mode_info_t *out_info) {
    if (!g_ready || out_info == NULL ||
        monitor_index != 0u || mode_index != 0u) {
        return false;
    }
    memset(out_info, 0, sizeof(*out_info));
    out_info->monitor_index = 0u;
    out_info->mode_index = 0u;
    out_info->flags = DISPLAY_MODE_FLAG_CURRENT |
                      DISPLAY_MODE_FLAG_PREFERRED |
                      DISPLAY_MODE_FLAG_SYNTHETIC;
    out_info->width = g_fb.width;
    out_info->height = g_fb.height;
    out_info->stride = g_fb.pixels_per_scan_line;
    out_info->bits_per_pixel = 32u;
    out_info->refresh_millihz = 60000u;
    generic_copy_string(out_info->name, sizeof(out_info->name), "Current");
    return true;
}

bool fb_set_mode(uint32_t monitor_index, uint32_t mode_index) {
    (void)monitor_index;
    (void)mode_index;
    return false;
}

void fb_draw_pixel(uint32_t x, uint32_t y, uint32_t color) {
    if (!g_ready || !g_double_buffer.buffer) return;
    if (x >= g_fb.width || y >= g_fb.height) return;

    uint64_t pixel_index = (uint64_t)y * g_fb.pixels_per_scan_line + x;
    if (pixel_index < g_double_buffer.buffer_size / sizeof(uint32_t)) {
        g_double_buffer.buffer[pixel_index] = color;
    }
}

void fb_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color) {
    if (!g_ready || !g_double_buffer.buffer || w == 0 || h == 0) {
        return;
    }
    if (x >= g_fb.width || y >= g_fb.height) {
        return;
    }

    uint64_t x_end64 = (uint64_t)x + (uint64_t)w;
    uint64_t y_end64 = (uint64_t)y + (uint64_t)h;
    if (x_end64 > g_fb.width) {
        x_end64 = g_fb.width;
    }
    if (y_end64 > g_fb.height) {
        y_end64 = g_fb.height;
    }

    uint32_t x_end = (uint32_t)x_end64;
    uint32_t y_end = (uint32_t)y_end64;
    if (x_end <= x || y_end <= y) {
        return;
    }

    uint64_t buffer_pixels = g_double_buffer.buffer_size / sizeof(uint32_t);
    for (uint32_t py = y; py < y_end; ++py) {
        uint64_t row_start = (uint64_t)py * g_fb.pixels_per_scan_line;
        uint64_t paint_begin = row_start + (uint64_t)x;
        uint64_t paint_end = row_start + (uint64_t)x_end;
        if (paint_begin >= buffer_pixels) {
            break;
        }
        if (paint_end > buffer_pixels) {
            paint_end = buffer_pixels;
        }

        for (uint64_t idx = paint_begin; idx < paint_end; ++idx) {
            g_double_buffer.buffer[idx] = color;
        }
    }
}

static void copy_row_to_framebuffer(uint64_t pixel_index,
                                    const uint8_t *src,
                                    uint64_t row_bytes)
{
    uint64_t byte_offset = g_fb.map_offset_bytes + (pixel_index * sizeof(uint32_t));
    uint64_t chunk_index = byte_offset / GENERIC_FB_MAP_GRANULE;

    if (chunk_index >= g_fb.mapped_chunks) {
        return;
    }

    uint64_t chunk_offset = byte_offset % GENERIC_FB_MAP_GRANULE;
    uint8_t *dst = g_fb.mapped_bases[chunk_index] + chunk_offset;
    uint64_t remaining_in_chunk = GENERIC_FB_MAP_GRANULE - chunk_offset;

    if (row_bytes <= remaining_in_chunk) {
        memcpy((void *)dst, (const void *)src, row_bytes);
        return;
    }

    uint64_t part1 = remaining_in_chunk;
    uint64_t part2 = row_bytes - part1;
    memcpy((void *)dst, (const void *)src, part1);
    if (chunk_index + 1 < g_fb.mapped_chunks) {
        memcpy((void *)g_fb.mapped_bases[chunk_index + 1],
               (const void *)(src + part1),
               part2);
    }
}

static bool clip_rect_to_framebuffer(const display_rect_t *input,
                                     uint32_t *x_out,
                                     uint32_t *y_out,
                                     uint32_t *w_out,
                                     uint32_t *h_out)
{
    if (input == NULL || input->w == 0u || input->h == 0u ||
        g_fb.width == 0u || g_fb.height == 0u) {
        return false;
    }

    int64_t x0 = input->x;
    int64_t y0 = input->y;
    int64_t x1 = x0 + (int64_t)input->w;
    int64_t y1 = y0 + (int64_t)input->h;
    if (x1 <= 0 || y1 <= 0 ||
        x0 >= (int64_t)g_fb.width ||
        y0 >= (int64_t)g_fb.height) {
        return false;
    }

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > (int64_t)g_fb.width) x1 = (int64_t)g_fb.width;
    if (y1 > (int64_t)g_fb.height) y1 = (int64_t)g_fb.height;
    if (x1 <= x0 || y1 <= y0) {
        return false;
    }

    *x_out = (uint32_t)x0;
    *y_out = (uint32_t)y0;
    *w_out = (uint32_t)(x1 - x0);
    *h_out = (uint32_t)(y1 - y0);
    return true;
}

void fb_present(void) {
    if (!g_ready || !g_double_buffer.buffer) {
        return;
    }

    uint8_t *src_bytes = (uint8_t *)g_double_buffer.buffer;
    uint64_t row_bytes = (uint64_t)g_fb.width * sizeof(uint32_t);
    uint64_t scan_line_bytes = (uint64_t)g_fb.pixels_per_scan_line * sizeof(uint32_t);

    for (uint32_t y = 0; y < g_fb.height; ++y) {
        uint64_t pixel_index = (uint64_t)y * g_fb.pixels_per_scan_line;
        uint8_t *src = src_bytes + (y * scan_line_bytes);
        copy_row_to_framebuffer(pixel_index, src, row_bytes);
    }
}

void fb_present_rects(const display_rect_t *rects, uint32_t count)
{
    if (!g_ready || !g_double_buffer.buffer || rects == NULL || count == 0u) {
        fb_present();
        return;
    }

    uint8_t *src_bytes = (uint8_t *)g_double_buffer.buffer;
    uint64_t scan_line_bytes = (uint64_t)g_fb.pixels_per_scan_line * sizeof(uint32_t);

    for (uint32_t i = 0u; i < count; ++i) {
        uint32_t x = 0u;
        uint32_t y = 0u;
        uint32_t w = 0u;
        uint32_t h = 0u;
        if (!clip_rect_to_framebuffer(&rects[i], &x, &y, &w, &h)) {
            continue;
        }

        uint64_t row_bytes = (uint64_t)w * sizeof(uint32_t);
        for (uint32_t row = 0u; row < h; ++row) {
            uint64_t pixel_index =
                (uint64_t)(y + row) * g_fb.pixels_per_scan_line + x;
            const uint8_t *src =
                src_bytes + ((uint64_t)(y + row) * scan_line_bytes) +
                ((uint64_t)x * sizeof(uint32_t));
            copy_row_to_framebuffer(pixel_index, src, row_bytes);
        }
    }
}

static void *generic_fb_get_framebuffer(void) {
    return g_double_buffer.buffer;
}

static const driver_display_t g_generic_fb_driver = {
    .name = GENERIC_FB_DEVICE_NAME,
    .probe = fb_probe,
    .init = fb_init,
    .is_ready = fb_is_ready,
    .width = fb_width,
    .height = fb_height,
    .draw_pixel = fb_draw_pixel,
    .fill_rect = fb_fill_rect,
    .present = fb_present,
    .present_rects = fb_present_rects,
    .set_framebuffer = generic_fb_set,
    .get_framebuffer = generic_fb_get_framebuffer,
    .get_generation = fb_generation,
    .poll_config = fb_poll_config,
    .get_monitor_count = fb_monitor_count,
    .get_topology = fb_get_topology,
    .get_monitor_info = fb_get_monitor_info,
    .get_mode_info = fb_get_mode_info,
    .set_mode = fb_set_mode,
};

#ifdef IMPLUS_DRIVER_MODULE

static void generic_fb_driver_shutdown(void)
{
    (void)generic_fb_set(NULL);
    g_driver_api = NULL;
}

static const driver_module_descriptor_t g_generic_fb_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_DISPLAY,
    .load_priority = 90u,
    .deps = { NULL },
    .driver_api = &g_generic_fb_driver,
    .shutdown = generic_fb_driver_shutdown,
};

#undef malloc
#undef free
#undef map_mmio_virt
#undef memcpy
#undef memset
#undef strcmp

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api) {
    if (!api) return NULL;

    if (!api->malloc || !api->free ||
        !api->map_mmio_virt || !api->memcpy || !api->memset) {
        return NULL;
    }

    g_driver_api = api;
    return &g_generic_fb_module;
}

#endif
