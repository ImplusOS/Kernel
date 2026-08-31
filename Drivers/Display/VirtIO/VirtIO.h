#pragma once

#include <stdbool.h>
#include <stdint.h>

bool virtio_gpu_init(void);
bool virtio_gpu_is_ready(void);
uint32_t virtio_gpu_width(void);
uint32_t virtio_gpu_height(void);
void virtio_gpu_draw_pixel(uint32_t x, uint32_t y, uint32_t color);
void virtio_gpu_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color);
void virtio_gpu_present(void);
