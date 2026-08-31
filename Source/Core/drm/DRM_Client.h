#pragma once
#include <stdint.h>

#define DRM_IOCTL_VERSION          0xC0
#define DRM_IOCTL_GET_CAP          0xC1
#define DRM_IOCTL_MODE_GETRESOURCES 0xA0
#define DRM_IOCTL_MODE_GETCRTC     0xA1
#define DRM_IOCTL_MODE_SETCRTC     0xA2
#define DRM_IOCTL_MODE_GETCONNECTOR 0xA3
#define DRM_IOCTL_MODE_CREATE_DUMB 0xB2
#define DRM_IOCTL_MODE_MAP_DUMB    0xB3
#define DRM_IOCTL_MODE_DESTROY_DUMB 0xB4
#define DRM_IOCTL_MODE_ADDFB      0xAE
#define DRM_IOCTL_MODE_RMFB       0xAF
#define DRM_IOCTL_MODE_PAGE_FLIP  0xB0
#define DRM_IOCTL_MODE_GETENCODER 0xA6
#define DRM_IOCTL_SET_MASTER      0xC2
#define DRM_IOCTL_DROP_MASTER     0xC3

#define DRM_CAP_DUMB_BUFFER       0x1
#define DRM_CAP_PRIME             0x5

#define DRM_DEV_FD_BASE 0x6000

typedef struct {
    uint32_t handle;
    uint32_t width;
    uint32_t height;
    uint32_t bpp;
    uint32_t pitch;
    uint64_t size;
} drm_create_dumb_t;

typedef struct {
    uint32_t handle;
    uint32_t pad;
    uint64_t offset;
} drm_map_dumb_t;

typedef struct {
    uint32_t fb_id;
    uint32_t width;
    uint32_t height;
    uint32_t pitch;
    uint32_t bpp;
    uint32_t depth;
    uint32_t handle;
} drm_mode_fb_cmd_t;

void drm_client_init(void);
int64_t drm_client_open(void);
int64_t drm_client_ioctl(int32_t fd, uint64_t request, uint64_t arg);
int64_t drm_client_close(int32_t fd);
void *drm_client_mmap(int32_t fd, uint64_t offset, uint64_t size);
