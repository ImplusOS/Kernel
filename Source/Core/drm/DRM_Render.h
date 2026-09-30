#pragma once
#include <stdint.h>

/*
 * DRM_Render — render node (/dev/dri/renderD128) implementation.
 *
 * The render node is a DRM device that exposes ONLY GPU compute/rendering
 * capabilities without any display/modesetting functionality. It allows
 * unprivileged clients (like Mesa) to allocate buffers, submit commands,
 * and share buffers via PRIME/dma-buf without needing master (CAP_SYS_ADMIN).
 *
 * Key differences from card node:
 * - NO modesetting ioctls: SETCRTC, PAGE_FLIP, SETPLANE, ATOMIC, SET_MASTER, DROP_MASTER
 * - NO scanout/connector/encoder/CRTC resources
 * - YES: GEM buffer management (CREATE_DUMB, ADDFB2, GEM_CLOSE, GEM_FLINK, PRIME_HANDLE_TO_FD, PRIME_FD_TO_HANDLE)
 * - YES: Synchronization (SYNCOBJ, TIMELINE_SYNCOBJ)
 * - YES: Buffer sharing via dma-buf (PRIME)
 * - YES: DRM_CAP_PRIME, DRM_CAP_SYNCOBJ, DRM_CAP_TIMELINE_SYNCOBJ
 */

void drm_render_init(void);

/* Same ioctl entry point signature as card node, but different allowlist */
int64_t drm_render_ioctl(uint64_t request, uint64_t arg);

/* Render nodes don't have page-flip events, but may have syncobj wait events */
int64_t drm_render_read(uint8_t *user_buf, uint64_t len, uint32_t nonblock);
uint32_t drm_render_poll(uint32_t events);

/* mmap for dumb buffers / GEM objects */
int64_t drm_render_mmap(uint64_t offset, uint64_t length, uint64_t prot, uint64_t flags);

void drm_render_close(void);

/* Internal: called when a process exits to clean up its render node state */
void drm_render_notify_process_exit(int32_t pid);

/* Buffer sharing helpers for PRIME/dma-buf */
int drm_render_prime_handle_to_fd(int32_t handle, uint32_t flags, int32_t *out_fd);
int drm_render_prime_fd_to_handle(int32_t fd, uint32_t *out_handle);

/* Syncobj operations */
int drm_render_syncobj_create(uint32_t flags, uint32_t *out_handle);
int drm_render_syncobj_destroy(uint32_t handle);
int drm_render_syncobj_wait(uint32_t *handles, uint32_t count, int64_t timeout_ns,
                            uint32_t flags, uint32_t *first_signaled);
int drm_render_syncobj_signal(uint32_t *handles, uint32_t count);
int drm_render_syncobj_reset(uint32_t *handles, uint32_t count);
int drm_render_syncobj_timeline_wait(uint64_t *handles, uint64_t *points, uint32_t count,
                                     int64_t timeout_ns, uint32_t flags, uint32_t *first_signaled);
int drm_render_syncobj_timeline_signal(uint64_t *handles, uint64_t *points, uint32_t count);
int drm_render_syncobj_transfer(uint32_t dst_handle, uint32_t src_handle, uint64_t src_point,
                                uint32_t flags);

/* GEM buffer object management (extended for render node) */
typedef struct drm_render_gem_object {
    uint32_t handle;
    uint32_t size;
    uint32_t refcount;
    void *kva;           /* kernel virtual address */
    uint64_t phys;       /* physical base */
    uint32_t npages;
    uint64_t map_offset; /* mmap offset token */
    int32_t prime_fd;    /* dma-buf fd if exported, -1 otherwise */
    /* For future: dma-buf attachment, import/export ops */
} drm_render_gem_object_t;