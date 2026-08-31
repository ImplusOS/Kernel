#include "DRM_Client.h"
#include "Drivers/Module/Display_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include <stddef.h>
#include <string.h>

#define DRM_MAX_DUMB 16
#define DRM_MAX_FB   16

typedef struct {
    uint8_t used;
    uint32_t handle;
    uint32_t width, height, bpp, pitch;
    uint64_t size;
    void *buffer;
} drm_dumb_t;

typedef struct {
    uint8_t used;
    uint32_t fb_id;
    uint32_t handle;
    uint32_t width, height, pitch;
} drm_fb_t;

static drm_dumb_t g_dumbs[DRM_MAX_DUMB];
static drm_fb_t g_fbs[DRM_MAX_FB];
static uint32_t g_next_handle = 1;
static uint32_t g_next_fb_id = 1;
static spinlock_t g_drm_lock;
static int g_drm_init_done = 0;

void drm_client_init(void) {
    spinlock_init(&g_drm_lock);
    memset(g_dumbs, 0, sizeof(g_dumbs));
    memset(g_fbs, 0, sizeof(g_fbs));
    g_drm_init_done = 1;
}

int64_t drm_client_open(void) {
    if (!g_drm_init_done) drm_client_init();
    return DRM_DEV_FD_BASE;
}

static int64_t drm_ioctl_create_dumb(uint64_t arg) {
    drm_create_dumb_t *d = (drm_create_dumb_t*)(uintptr_t)arg;
    if (!d) return -14;
    spinlock_lock(&g_drm_lock);
    for (int i = 0; i < DRM_MAX_DUMB; i++) {
        if (!g_dumbs[i].used) {
            g_dumbs[i].used = 1;
            g_dumbs[i].handle = g_next_handle++;
            g_dumbs[i].width = d->width;
            g_dumbs[i].height = d->height;
            g_dumbs[i].bpp = d->bpp;
            g_dumbs[i].pitch = d->width * (d->bpp / 8);
            g_dumbs[i].size = (uint64_t)g_dumbs[i].pitch * d->height;
            g_dumbs[i].buffer = process_user_alloc((uint32_t)g_dumbs[i].size);
            if (!g_dumbs[i].buffer) { g_dumbs[i].used = 0; spinlock_unlock(&g_drm_lock); return -12; }
            memset(g_dumbs[i].buffer, 0, (size_t)g_dumbs[i].size);
            d->handle = g_dumbs[i].handle;
            d->pitch = g_dumbs[i].pitch;
            d->size = g_dumbs[i].size;
            spinlock_unlock(&g_drm_lock);
            return 0;
        }
    }
    spinlock_unlock(&g_drm_lock);
    return -12;
}

static int64_t drm_ioctl_map_dumb(uint64_t arg) {
    drm_map_dumb_t *m = (drm_map_dumb_t*)(uintptr_t)arg;
    if (!m) return -14;
    spinlock_lock(&g_drm_lock);
    for (int i = 0; i < DRM_MAX_DUMB; i++) {
        if (g_dumbs[i].used && g_dumbs[i].handle == m->handle) {
            m->offset = (uint64_t)(uintptr_t)g_dumbs[i].buffer;
            spinlock_unlock(&g_drm_lock);
            return 0;
        }
    }
    spinlock_unlock(&g_drm_lock);
    return -22;
}

static int64_t drm_ioctl_addfb(uint64_t arg) {
    drm_mode_fb_cmd_t *f = (drm_mode_fb_cmd_t*)(uintptr_t)arg;
    if (!f) return -14;
    spinlock_lock(&g_drm_lock);
    for (int i = 0; i < DRM_MAX_FB; i++) {
        if (!g_fbs[i].used) {
            g_fbs[i].used = 1;
            g_fbs[i].fb_id = g_next_fb_id++;
            g_fbs[i].handle = f->handle;
            g_fbs[i].width = f->width;
            g_fbs[i].height = f->height;
            g_fbs[i].pitch = f->pitch;
            f->fb_id = g_fbs[i].fb_id;
            spinlock_unlock(&g_drm_lock);
            return 0;
        }
    }
    spinlock_unlock(&g_drm_lock);
    return -12;
}

static int64_t drm_ioctl_page_flip(uint64_t arg) {
    struct { uint32_t crtc_id; uint32_t fb_id; uint32_t flags; uint32_t reserved; uint64_t user_data; } *pf = (void*)(uintptr_t)arg;
    if (!pf) return -14;
    spinlock_lock(&g_drm_lock);
    for (int fi = 0; fi < DRM_MAX_FB; fi++) {
        if (g_fbs[fi].used && g_fbs[fi].fb_id == pf->fb_id) {
            for (int di = 0; di < DRM_MAX_DUMB; di++) {
                if (g_dumbs[di].used && g_dumbs[di].handle == g_fbs[fi].handle) {
                    void *hw_fb = display_get_framebuffer();
                    if (hw_fb && g_dumbs[di].buffer) {
                        uint32_t hw_w = display_width();
                        uint32_t hw_h = display_height();
                        uint32_t copy_w = g_dumbs[di].width < hw_w ? g_dumbs[di].width : hw_w;
                        uint32_t copy_h = g_dumbs[di].height < hw_h ? g_dumbs[di].height : hw_h;
                        uint32_t hw_pitch = hw_w * 4;
                        for (uint32_t y = 0; y < copy_h; y++) {
                            memcpy((uint8_t*)hw_fb + y * hw_pitch,
                                   (uint8_t*)g_dumbs[di].buffer + y * g_dumbs[di].pitch,
                                   copy_w * 4);
                        }
                        display_present();
                    }
                    break;
                }
            }
            break;
        }
    }
    spinlock_unlock(&g_drm_lock);
    return 0;
}

int64_t drm_client_ioctl(int32_t fd, uint64_t request, uint64_t arg) {
    (void)fd;
    switch (request) {
        case DRM_IOCTL_VERSION: {
            struct { int32_t major, minor, patch;
                     uint64_t name_len; char *name;
                     uint64_t date_len; char *date;
                     uint64_t desc_len; char *desc; } *v = (void*)(uintptr_t)arg;
            if (!v) return -14;
            v->major = 1; v->minor = 0; v->patch = 0;
            v->name_len = 6; v->date_len = 0; v->desc_len = 0;
            return 0;
        }
        case DRM_IOCTL_GET_CAP: {
            struct { uint64_t cap; uint64_t val; } *c = (void*)(uintptr_t)arg;
            if (!c) return -14;
            c->val = (c->cap == DRM_CAP_DUMB_BUFFER) ? 1 : 0;
            return 0;
        }
        case DRM_IOCTL_MODE_GETRESOURCES: {
            struct { uint64_t fb_id_ptr; uint64_t crtc_id_ptr; uint64_t conn_id_ptr; uint64_t enc_id_ptr;
                     uint32_t count_fbs, count_crtcs, count_conns, count_encs;
                     uint32_t min_w, max_w, min_h, max_h; } *r = (void*)(uintptr_t)arg;
            if (!r) return -14;
            r->count_fbs = 0; r->count_crtcs = 1; r->count_conns = 1; r->count_encs = 1;
            r->min_w = 1; r->max_w = 4096; r->min_h = 1; r->max_h = 4096;
            if (r->crtc_id_ptr) { uint32_t *p = (uint32_t*)(uintptr_t)r->crtc_id_ptr; p[0] = 1; }
            if (r->conn_id_ptr) { uint32_t *p = (uint32_t*)(uintptr_t)r->conn_id_ptr; p[0] = 1; }
            if (r->enc_id_ptr)  { uint32_t *p = (uint32_t*)(uintptr_t)r->enc_id_ptr; p[0] = 1; }
            return 0;
        }
        case DRM_IOCTL_MODE_GETCRTC: {
            struct { uint32_t crtc_id, fb_id, x, y, gamma_size, mode_valid; } *c = (void*)(uintptr_t)arg;
            if (c) { c->fb_id=0; c->x=0; c->y=0; c->gamma_size=0; c->mode_valid=1; }
            return 0;
        }
        case DRM_IOCTL_MODE_GETCONNECTOR: {
            struct { uint64_t enc; uint32_t conn_id, conn_type, conn_type_id, connection, mm_w, mm_h, subpixel,
                     count_modes; uint64_t modes_ptr; uint32_t count_props; uint64_t props_ptr, prop_values_ptr;
                     uint32_t count_encoders; uint64_t encoders_ptr; uint32_t encoder_id; } *cn = (void*)(uintptr_t)arg;
            if (cn) { cn->connection=1; cn->count_modes=1; cn->encoder_id=1; cn->mm_w=530; cn->mm_h=300; }
            return 0;
        }
        case DRM_IOCTL_MODE_CREATE_DUMB: return drm_ioctl_create_dumb(arg);
        case DRM_IOCTL_MODE_MAP_DUMB:    return drm_ioctl_map_dumb(arg);
        case DRM_IOCTL_MODE_ADDFB:       return drm_ioctl_addfb(arg);
        case DRM_IOCTL_MODE_PAGE_FLIP:   return drm_ioctl_page_flip(arg);
        case DRM_IOCTL_SET_MASTER:
        case DRM_IOCTL_DROP_MASTER: return 0;
        default: return 0;
    }
}

void *drm_client_mmap(int32_t fd, uint64_t offset, uint64_t size) {
    (void)fd; (void)size;
    return (void*)(uintptr_t)offset;
}

int64_t drm_client_close(int32_t fd) {
    (void)fd;
    return 0;
}
