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

/* ---- 3D (virgl) API ---- */
bool virtio_gpu_has_3d(void);

/* Create a rendering context (virgl). Returns context_id (>0) or 0 on error. */
uint32_t virtio_gpu_ctx_create(uint32_t context_spec, const char *name);
bool virtio_gpu_ctx_destroy(uint32_t context_id);

/* Submit a raw virgl command blob to the context. */
bool virtio_gpu_submit_3d(uint32_t context_id, const void *blob, uint32_t blob_size);

/* Create a 3D resource (texture/buffer). */
bool virtio_gpu_resource_create_3d(uint32_t resource_id, uint32_t format,
                                   uint32_t width, uint32_t height,
                                   uint32_t depth, uint32_t target,
                                   uint32_t bind);

/* Transfer host->guest for 3D resources. */
bool virtio_gpu_transfer_to_host_3d(uint32_t context_id, uint32_t resource_id,
                                    uint32_t x, uint32_t y,
                                    uint32_t w, uint32_t h);
