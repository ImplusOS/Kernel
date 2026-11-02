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

/* ---- 3D (virgl) API ----
 * These functions implement the Linux virtio_gpu.h control-queue wire ABI.
 * context_spec is a capset id (VIRGL == 1) for devices negotiating
 * VIRTIO_GPU_F_CONTEXT_INIT; it is ignored by older devices. */
bool virtio_gpu_has_3d(void);
bool virtio_gpu_has_context_init(void);
uint32_t virtio_gpu_resource_id_alloc(void);
bool virtio_gpu_get_capset_info(uint32_t index, uint32_t *out_id,
                                uint32_t *out_version, uint32_t *out_size);
bool virtio_gpu_get_capset(uint32_t capset_id, uint32_t version,
                           void *out_caps, uint32_t out_size);
uint32_t virtio_gpu_ctx_create(uint32_t context_spec, const char *name);
bool virtio_gpu_ctx_destroy(uint32_t context_id);
bool virtio_gpu_submit_3d(uint32_t context_id, const void *blob, uint32_t blob_size);

bool virtio_gpu_resource_create_3d(uint32_t resource_id, uint32_t format,
                                   uint32_t width, uint32_t height,
                                   uint32_t depth, uint32_t target,
                                   uint32_t bind);
bool virtio_gpu_resource_unref(uint32_t resource_id);
bool virtio_gpu_resource_detach_backing(uint32_t resource_id);
bool virtio_gpu_resource_create_3d_ex(uint32_t resource_id,
                                      uint32_t target, uint32_t format,
                                      uint32_t bind, uint32_t width,
                                      uint32_t height, uint32_t depth,
                                      uint32_t array_size,
                                      uint32_t last_level,
                                      uint32_t nr_samples, uint32_t flags);

/* physical_address must identify a contiguous DMA-addressable range. */
bool virtio_gpu_resource_attach_backing(uint32_t resource_id,
                                        uint64_t physical_address,
                                        uint32_t bytes);
bool virtio_gpu_ctx_attach_resource(uint32_t context_id, uint32_t resource_id);
bool virtio_gpu_ctx_detach_resource(uint32_t context_id, uint32_t resource_id);

bool virtio_gpu_transfer_to_host_3d(uint32_t context_id, uint32_t resource_id,
                                    uint32_t x, uint32_t y,
                                    uint32_t w, uint32_t h);
bool virtio_gpu_transfer_from_host_3d(uint32_t context_id, uint32_t resource_id,
                                      uint32_t x, uint32_t y,
                                      uint32_t w, uint32_t h);
bool virtio_gpu_transfer_3d_ex(uint32_t context_id, uint32_t resource_id,
                               bool to_host, uint32_t x, uint32_t y,
                               uint32_t z, uint32_t width, uint32_t height,
                               uint32_t depth, uint64_t offset,
                               uint32_t level, uint32_t stride,
                               uint32_t layer_stride);
