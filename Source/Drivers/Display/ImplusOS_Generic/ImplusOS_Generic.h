#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "Drivers/Module/DriverBinary.h"

bool generic_fb_set(const driver_boot_framebuffer_t *framebuffer);
bool fb_init(void);
bool fb_is_ready(void);
uint32_t fb_width(void);
uint32_t fb_height(void);
void fb_draw_pixel(uint32_t x, uint32_t y, uint32_t color);
void fb_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color);
void fb_present(void);
bool fb_probe(void);
