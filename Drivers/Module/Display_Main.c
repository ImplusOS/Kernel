#include <stddef.h>

#include <string.h>

#include "Display_Main.h"
#include "Drivers/Module/DriverBinary.h"
#include "Drivers/Module/DeviceRegistry.h"
#include "Drivers/Module/DriverManager.h"
#include "Drivers/Module/DriverSelect.h"
#include "MemoryManagement/Memory_Main.h"

const driver_display_t *g_active_display_driver = NULL;
static const char *g_active_display_module_name = NULL;

static uint32_t *g_framebuffer = NULL;
static uint32_t  g_fb_width    = 0;
static uint32_t  g_fb_height   = 0;
static uint64_t  g_framebuffer_pixels = 0;
static uint32_t  g_display_generation = 1u;

static bool display_ensure_shadow_buffer(void)
{
    uint64_t total_pixels = (uint64_t)g_fb_width * (uint64_t)g_fb_height;
    if (total_pixels == 0u) {
        return false;
    }
    if (total_pixels > 0xFFFFFFFFULL / sizeof(uint32_t)) {
        return false;
    }

    if (g_framebuffer != NULL && g_framebuffer_pixels == total_pixels) {
        return true;
    }

    if (g_framebuffer != NULL) {
        free(g_framebuffer);
        g_framebuffer = NULL;
        g_framebuffer_pixels = 0u;
    }

    g_framebuffer = (uint32_t *)malloc((uint32_t)(total_pixels * sizeof(uint32_t)));
    if (g_framebuffer == NULL) {
        return false;
    }
    g_framebuffer_pixels = total_pixels;
    memset(g_framebuffer, 0xFF, (size_t)(total_pixels * sizeof(uint32_t)));
    return true;
}

static bool display_sync_geometry(void)
{
    if (g_active_display_driver == NULL) {
        return false;
    }

    uint32_t width = 0;
    uint32_t height = 0;
    if (g_active_display_driver->width != NULL) {
        width = g_active_display_driver->width();
    }
    if (g_active_display_driver->height != NULL) {
        height = g_active_display_driver->height();
    }

    if (width == 0 || height == 0) {
        return false;
    }

    if (width != g_fb_width || height != g_fb_height) {
        g_fb_width = width;
        g_fb_height = height;
        ++g_display_generation;
    }

    return display_ensure_shadow_buffer();
}

static bool display_refresh_driver(void)
{
    if (g_active_display_driver != NULL && g_active_display_module_name != NULL) {
        const driver_display_t *current =
            driver_manager_get_display_driver(g_active_display_module_name);
        if (current == g_active_display_driver) {
            return display_sync_geometry();
        }
    }

    g_active_display_driver = driver_select_pick_display_driver();
    if (g_active_display_driver == NULL) {
        g_active_display_module_name = NULL;
        g_fb_width = 0;
        g_fb_height = 0;
        return false;
    }

    g_active_display_module_name = NULL;
    for (uint32_t i = 0;; ++i) {
        const device_t *dev = device_registry_find_by_index(DEVICE_TYPE_DISPLAY, i);
        if (dev == NULL) {
            break;
        }
        if (dev->ops == g_active_display_driver) {
            g_active_display_module_name = dev->name;
            break;
        }
    }

    return display_sync_geometry();
}

bool display_init(void) {
    g_active_display_driver = NULL;
    g_active_display_module_name = NULL;
    if (!display_refresh_driver()) {
        return false;
    }

    if (g_fb_width == 0 || g_fb_height == 0) {
        return false;
    }

    return display_ensure_shadow_buffer();
}

bool display_is_ready(void) {
    if (!display_refresh_driver()) {
        return false;
    }

    if (g_active_display_driver->is_ready == NULL) {
        return false;
    }

    return g_active_display_driver->is_ready();
}

uint32_t display_width(void) {
    if (!display_is_ready() || g_active_display_driver->width == NULL) {
        return 0;
    }

    return g_active_display_driver->width();
}

uint32_t display_height(void) {
    if (!display_is_ready() || g_active_display_driver->height == NULL) {
        return 0;
    }

    return g_active_display_driver->height();
}

void display_draw_pixel(uint32_t x, uint32_t y, uint32_t color) {
    if (!display_is_ready() || g_active_display_driver->draw_pixel == NULL) {
        return;
    }

    if (x >= g_fb_width || y >= g_fb_height || !g_framebuffer) {
        return;
    }

    uint32_t idx = y * g_fb_width + x;

    if (g_framebuffer[idx] == color) {
        return;
    }

    g_framebuffer[idx] = color;
    g_active_display_driver->draw_pixel(x, y, color);
}

uint32_t display_get_pixel(uint32_t x, uint32_t y) {
    if (!display_is_ready()) {
        return 0;
    }

    if (x >= g_fb_width || y >= g_fb_height || !g_framebuffer) {
        return 0;
    }

    return g_framebuffer[y * g_fb_width + x];
}

void display_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color) {
    if (!display_is_ready()) {
        return;
    }

    if (g_active_display_driver->fill_rect == NULL) {
        return;
    }

    if (!g_framebuffer || g_fb_width == 0 || g_fb_height == 0) {
        return;
    }

    if (x >= g_fb_width || y >= g_fb_height) {
        return;
    }
    if ((uint64_t)x + w > g_fb_width)  w = (uint32_t)(g_fb_width  - x);
    if ((uint64_t)y + h > g_fb_height) h = (uint32_t)(g_fb_height - y);
    if (w == 0 || h == 0) {
        return;
    }

    for (uint32_t yy = y; yy < y + h; yy++) {
        for (uint32_t xx = x; xx < x + w; xx++) {
            g_framebuffer[yy * g_fb_width + xx] = color;
        }
    }

    g_active_display_driver->fill_rect(x, y, w, h, color);
}

void display_present(void) {
    if (!display_is_ready() || g_active_display_driver->present == NULL) {
        return;
    }

    g_active_display_driver->present();
}

void display_present_rects(const display_rect_t *rects, uint32_t count) {
    if (!display_is_ready()) {
        return;
    }

    if (rects != NULL && count != 0u &&
        g_active_display_driver->present_rects != NULL) {
        g_active_display_driver->present_rects(rects, count);
        return;
    }

    if (g_active_display_driver->present != NULL) {
        g_active_display_driver->present();
    }
}

void *display_get_framebuffer(void) {
    if (!display_is_ready() || g_active_display_driver->get_framebuffer == NULL) {
        return NULL;
    }

    return g_active_display_driver->get_framebuffer();
}

uint32_t display_generation(void)
{
    if (!display_is_ready()) {
        return 0;
    }
    if (g_active_display_driver->get_generation != NULL) {
        uint32_t generation = g_active_display_driver->get_generation();
        return generation != 0u ? generation : g_display_generation;
    }
    return g_display_generation;
}

bool display_poll_config(void)
{
    if (!display_refresh_driver()) {
        return false;
    }

    bool changed = false;
    if (g_active_display_driver->poll_config != NULL) {
        changed = g_active_display_driver->poll_config();
    }
    if (display_sync_geometry() && changed) {
        ++g_display_generation;
    }
    return changed;
}

bool display_get_topology(display_topology_t *out_topology)
{
    if (out_topology == NULL || !display_is_ready()) {
        return false;
    }
    (void)display_poll_config();
    if (g_active_display_driver->get_topology != NULL &&
        g_active_display_driver->get_topology(out_topology)) {
        return true;
    }

    memset(out_topology, 0, sizeof(*out_topology));
    out_topology->generation = display_generation();
    out_topology->monitor_count = 1u;
    out_topology->primary_monitor = 0u;
    out_topology->origin_x = 0;
    out_topology->origin_y = 0;
    out_topology->width = display_width();
    out_topology->height = display_height();
    return out_topology->width != 0u && out_topology->height != 0u;
}

bool display_get_monitor_info(uint32_t monitor_index,
                              display_monitor_info_t *out_info)
{
    if (out_info == NULL || !display_is_ready()) {
        return false;
    }
    (void)display_poll_config();
    if (g_active_display_driver->get_monitor_info != NULL &&
        g_active_display_driver->get_monitor_info(monitor_index, out_info)) {
        return true;
    }
    if (monitor_index != 0u) {
        return false;
    }

    memset(out_info, 0, sizeof(*out_info));
    out_info->index = 0u;
    out_info->id = 0u;
    out_info->flags = DISPLAY_MONITOR_FLAG_CONNECTED |
                      DISPLAY_MONITOR_FLAG_PRIMARY |
                      DISPLAY_MONITOR_FLAG_SYNTHETIC_MODE;
    out_info->output_type = DISPLAY_OUTPUT_UNKNOWN;
    out_info->width = display_width();
    out_info->height = display_height();
    out_info->refresh_millihz = 60000u;
    out_info->mode_count = 1u;
    out_info->generation = display_generation();
    strcpy(out_info->name, "Display 0");
    strcpy(out_info->output_name, "display0");
    return out_info->width != 0u && out_info->height != 0u;
}

bool display_get_monitor_mode_info(uint32_t monitor_index,
                                   uint32_t mode_index,
                                   display_mode_info_t *out_info)
{
    if (out_info == NULL || !display_is_ready()) {
        return false;
    }
    (void)display_poll_config();
    if (g_active_display_driver->get_mode_info != NULL &&
        g_active_display_driver->get_mode_info(monitor_index, mode_index,
                                               out_info)) {
        return true;
    }
    if (monitor_index != 0u || mode_index != 0u) {
        return false;
    }

    memset(out_info, 0, sizeof(*out_info));
    out_info->monitor_index = 0u;
    out_info->mode_index = 0u;
    out_info->flags = DISPLAY_MODE_FLAG_CURRENT |
                      DISPLAY_MODE_FLAG_PREFERRED |
                      DISPLAY_MODE_FLAG_SYNTHETIC;
    out_info->width = display_width();
    out_info->height = display_height();
    out_info->stride = out_info->width;
    out_info->bits_per_pixel = 32u;
    out_info->refresh_millihz = 60000u;
    strcpy(out_info->name, "Current");
    return out_info->width != 0u && out_info->height != 0u;
}

bool display_set_monitor_mode(uint32_t monitor_index, uint32_t mode_index)
{
    if (!display_is_ready() || g_active_display_driver->set_mode == NULL) {
        return false;
    }
    if (!g_active_display_driver->set_mode(monitor_index, mode_index)) {
        return false;
    }
    ++g_display_generation;
    (void)display_poll_config();
    return display_sync_geometry();
}

void display_driver_detached(const char *module_name)
{
    if (module_name == NULL || g_active_display_module_name == NULL) {
        return;
    }
    if (strcmp(module_name, g_active_display_module_name) != 0) {
        return;
    }

    g_active_display_driver = NULL;
    g_active_display_module_name = NULL;
    if (g_framebuffer != NULL) {
        free(g_framebuffer);
        g_framebuffer = NULL;
    }
    g_framebuffer_pixels = 0u;
    g_fb_width = 0;
    g_fb_height = 0;
}
