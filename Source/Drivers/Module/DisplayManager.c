#include "DisplayManager.h"

#include "Drivers/Module/Display_Main.h"

bool display_manager_init(void)
{
    return display_init();
}

bool display_manager_is_ready(void)
{
    return display_is_ready();
}

uint32_t display_manager_width(void)
{
    return display_width();
}

uint32_t display_manager_height(void)
{
    return display_height();
}

void display_manager_draw_pixel(uint32_t x, uint32_t y, uint32_t color)
{
    display_draw_pixel(x, y, color);
}

uint32_t display_manager_get_pixel(uint32_t x, uint32_t y)
{
    return display_get_pixel(x, y);
}

void display_manager_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color)
{
    display_fill_rect(x, y, w, h, color);
}

void display_manager_present(void)
{
    display_present();
}

void display_manager_present_rects(const display_rect_t *rects, uint32_t count)
{
    display_present_rects(rects, count);
}

void *display_manager_get_framebuffer(void)
{
    return display_get_framebuffer();
}

uint32_t display_manager_generation(void)
{
    return display_generation();
}

bool display_manager_poll_config(void)
{
    return display_poll_config();
}

bool display_manager_get_topology(display_topology_t *out_topology)
{
    return display_get_topology(out_topology);
}

bool display_manager_get_monitor_info(uint32_t monitor_index,
                                      display_monitor_info_t *out_info)
{
    return display_get_monitor_info(monitor_index, out_info);
}

bool display_manager_get_monitor_mode_info(uint32_t monitor_index,
                                           uint32_t mode_index,
                                           display_mode_info_t *out_info)
{
    return display_get_monitor_mode_info(monitor_index, mode_index, out_info);
}

bool display_manager_set_monitor_mode(uint32_t monitor_index,
                                      uint32_t mode_index)
{
    return display_set_monitor_mode(monitor_index, mode_index);
}

void display_manager_on_device_detached(const char *name)
{
    display_driver_detached(name);
}
