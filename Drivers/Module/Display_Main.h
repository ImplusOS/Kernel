#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "Drivers/Module/DriverBinary.h"

bool display_init(void);
bool display_is_ready(void);
uint32_t display_width(void);
uint32_t display_height(void);
void display_draw_pixel(uint32_t x, uint32_t y, uint32_t color);
uint32_t display_get_pixel(uint32_t x, uint32_t y);
void display_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color);
void display_present(void);
void display_present_rects(const display_rect_t *rects, uint32_t count);
void *display_get_framebuffer(void);
uint32_t display_generation(void);
bool display_poll_config(void);
bool display_get_topology(display_topology_t *out_topology);
bool display_get_monitor_info(uint32_t monitor_index,
                              display_monitor_info_t *out_info);
bool display_get_monitor_mode_info(uint32_t monitor_index,
                                   uint32_t mode_index,
                                   display_mode_info_t *out_info);
bool display_set_monitor_mode(uint32_t monitor_index, uint32_t mode_index);
void display_driver_detached(const char *module_name);
