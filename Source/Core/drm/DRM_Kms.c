/*
 * DRM_Kms.c — see DRM_Kms.h. Minimal Linux DRM/KMS for an unmodified Xorg
 * "modesetting" DDX + Mesa software GL. Single CRTC/connector/encoder, dumb
 * buffers blitted to the ImplusOS framebuffer on PAGE_FLIP / DIRTYFB.
 *
 * NOTE (TODO_Doom_Xorg_MethodA.md): this is written but has NOT been brought
 * up under QEMU. Struct layouts follow linux/drm.h + drm_mode.h for x86-64.
 */
#include "DRM_Kms.h"

#include "Drivers/Module/Display_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Core/usercopy/Usercopy.h"
#include "Core/timer/Timer.h"
#include "Core/vfs/DevFS.h"
#include "Core/vfs/SysFS.h"
#include "Core/syscall/Syscall_File.h"
#include "mmu/Paging_Main.h"
#include "Debug/serial/Serial.h"

#include <stddef.h>
#include <string.h>

/* ---- errno (negated) ---------------------------------------------------- */
#define E_INVAL   (-22)
#define E_NOMEM   (-12)
#define E_FAULT   (-14)
#define E_AGAIN   (-11)
#define E_NOTTY   (-25)
#define E_NODEV   (-19)

/* ---- Linux _IOC decode ------------------------------------------------- */
#define IOC_NR(cmd)   ((uint32_t)((cmd) >> 0) & 0xffu)
#define IOC_TYPE(cmd) ((uint32_t)((cmd) >> 8) & 0xffu)
#define DRM_IOCTL_BASE 'd'

/* command numbers (nr) */
#define DRM_NR_VERSION            0x00
#define DRM_NR_SET_VERSION        0x07
#define DRM_NR_GET_MAGIC          0x02
#define DRM_NR_AUTH_MAGIC         0x11 /* DRM_IOCTL_AUTH_MAGIC, *not* 0x03 */
#define DRM_NR_GET_CAP            0x0c
#define DRM_NR_SET_CLIENT_CAP     0x0d
#define DRM_NR_GEM_CLOSE          0x09
#define DRM_NR_GEM_FLINK          0x0a
#define DRM_NR_GEM_OPEN           0x0b
#define DRM_NR_SET_MASTER         0x1e
#define DRM_NR_DROP_MASTER        0x1f
#define DRM_NR_MODE_GETRESOURCES  0xA0
#define DRM_NR_MODE_GETCRTC       0xA1
#define DRM_NR_MODE_SETCRTC       0xA2
#define DRM_NR_MODE_CURSOR        0xA3
#define DRM_NR_MODE_GETGAMMA      0xA4
#define DRM_NR_MODE_SETGAMMA      0xA5
#define DRM_NR_MODE_GETENCODER    0xA6
#define DRM_NR_MODE_GETCONNECTOR  0xA7
#define DRM_NR_MODE_GETPROPERTY   0xAA
#define DRM_NR_MODE_SETPROPERTY   0xAB
#define DRM_NR_MODE_GETPROPBLOB   0xAC
#define DRM_NR_MODE_GETFB         0xAD
#define DRM_NR_MODE_ADDFB         0xAE
#define DRM_NR_MODE_RMFB          0xAF
#define DRM_NR_MODE_PAGE_FLIP     0xB0
#define DRM_NR_MODE_DIRTYFB       0xB1
#define DRM_NR_MODE_CREATE_DUMB   0xB2
#define DRM_NR_MODE_MAP_DUMB      0xB3
#define DRM_NR_MODE_DESTROY_DUMB  0xB4
#define DRM_NR_MODE_GETPLANERES   0xB5
#define DRM_NR_MODE_GETPLANE      0xB6
#define DRM_NR_MODE_SETPLANE      0xB7
#define DRM_NR_MODE_ADDFB2        0xB8
#define DRM_NR_MODE_OBJ_GETPROPS  0xB9
#define DRM_NR_MODE_OBJ_SETPROP   0xBA
#define DRM_NR_MODE_CURSOR2       0xBB
#define DRM_NR_MODE_ATOMIC        0xBC
#define DRM_NR_MODE_CREATEPROPBLOB  0xBD
#define DRM_NR_MODE_DESTROYPROPBLOB 0xBE
/* Lease enumeration. ImplusOS issues no leases, so the answer is always an
 * empty list -- but it has to *be* an empty list: Xorg's DDX calls
 * drmModeListLessees() while taking master and treats the ENOTTY we used to
 * answer as a failure to establish the lease machinery at all. */
#define DRM_NR_MODE_LIST_LESSEES  0xC7

/* PRIME fd sharing (ioctl base 'd', so _IOWR('d', 0x2D, ...) etc.) */
#define DRM_NR_PRIME_HANDLE_TO_FD  0x2D
#define DRM_NR_PRIME_FD_TO_HANDLE  0x2E

/* GEM names / open (legacy flink sharing) */

/* DRM_IOCTL_GET_CAP capabilities */
#define DRM_CAP_DUMB_BUFFER              0x1
#define DRM_CAP_VBLANK_HIGH_CRTC        0x2
#define DRM_CAP_DUMB_PREFERRED_DEPTH    0x3
#define DRM_CAP_DUMB_PREFER_SHADOW      0x4
#define DRM_CAP_PRIME                   0x5
#define DRM_CAP_TIMESTAMP_MONOTONIC     0x6
#define DRM_CAP_ASYNC_PAGE_FLIP        0x7
#define DRM_CAP_CURSOR_WIDTH           0x8
#define DRM_CAP_CURSOR_HEIGHT          0x9
#define DRM_CAP_ADDFB2_MODIFIERS       0x10
#define DRM_CAP_CRTC_IN_VBLANK_EVENT   0x12
#define DRM_CAP_SYNCOBJ               0x13

#define DRM_MODE_PAGE_FLIP_EVENT 0x01
#define DRM_EVENT_FLIP_COMPLETE  0x02

/* Distinct object IDs — atomic mode setting queries OBJ_GETPROPERTIES per
 * object, so CRTC/PLANE/CONNECTOR must not collide. */
#define DRM_CRTC_ID       1u
#define DRM_PLANE_ID      2u
#define DRM_CONNECTOR_ID  3u
#define DRM_ENCODER_ID    4u
#define DRM_MODE_CONNECTED 1u

/* ---- Atomic property IDs (must match the ATOMIC handler below) --------- */
#define DRM_PROP_CRTC_ACTIVE    0x100u  /* bool   */
#define DRM_PROP_CRTC_MODE_ID   0x101u  /* blob   */
#define DRM_PROP_CONN_CRTC_ID   0x102u  /* object */
#define DRM_PROP_PLANE_FB_ID    0x103u  /* object */
#define DRM_PROP_PLANE_CRTC_ID  0x104u  /* object */
#define DRM_PROP_PLANE_SRC_X    0x105u  /* range  */
#define DRM_PROP_PLANE_SRC_Y    0x106u  /* range  */
#define DRM_PROP_PLANE_SRC_W    0x107u  /* range  */
#define DRM_PROP_PLANE_SRC_H    0x108u  /* range  */
#define DRM_PROP_PLANE_CRTC_X   0x109u  /* range  */
#define DRM_PROP_PLANE_CRTC_Y   0x10Au  /* range  */
#define DRM_PROP_PLANE_CRTC_W   0x10Bu  /* range  */
#define DRM_PROP_PLANE_CRTC_H   0x10Cu  /* range  */
#define DRM_PROP_PLANE_TYPE     0x10Du  /* enum   */

/* DRM_MODE_PROP_* type bits — drm_mode.h. A wrong type bit makes the DDX
 * mis-read a property (e.g. treat an object ID as a range), so mirror the
 * kernel's values exactly: */
#define DRM_MODE_PROP_ENUM      0x00000008u /* (1 << 3) */
#define DRM_MODE_PROP_RANGE     0x00000002u /* (1 << 1) */
#define DRM_MODE_PROP_BLOB      0x00000010u /* (1 << 4) */
#define DRM_MODE_PROP_OBJECT    0x00000040u /* DRM_MODE_PROP_TYPE(1) */

/* drm_plane.type values — must match DRM_PLANE_TYPE_* in drm_mode.h and the
 * names modesetting looks for ("Primary"/"Cursor"/"Overlay"). Without this
 * property the DDX rejects every plane, leaves crtc->plane_id == 0 and then
 * every drmModeAtomicAddProperty() call returns -EINVAL, so no modeset ever
 * reaches the driver. */
#define DRM_PLANE_TYPE_PRIMARY  0u
#define DRM_PLANE_TYPE_CURSOR   1u
#define DRM_PLANE_TYPE_OVERLAY  2u

/* Simple monotonic blob-id allocator for CREATEPROPBLOB. */
static uint32_t g_next_blob_id = 1u;

/* ---- Linux struct layouts (x86-64) ----------------------------------- */
struct drm_version {
    int32_t  version_major;
    int32_t  version_minor;
    int32_t  version_patchlevel;
    uint32_t _pad;
    uint64_t name_len;   uint64_t name;   /* char __user * */
    uint64_t date_len;   uint64_t date;
    uint64_t desc_len;   uint64_t desc;
};

struct drm_get_cap { uint64_t capability; uint64_t value; };
struct drm_set_client_cap { uint64_t capability; uint64_t value; };

struct drm_mode_card_res {
    uint64_t fb_id_ptr;
    uint64_t crtc_id_ptr;
    uint64_t connector_id_ptr;
    uint64_t encoder_id_ptr;
    uint32_t count_fbs;
    uint32_t count_crtcs;
    uint32_t count_connectors;
    uint32_t count_encoders;
    uint32_t min_width, max_width;
    uint32_t min_height, max_height;
};

struct drm_mode_modeinfo {
    uint32_t clock;
    uint16_t hdisplay, hsync_start, hsync_end, htotal, hskew;
    uint16_t vdisplay, vsync_start, vsync_end, vtotal, vscan;
    uint32_t vrefresh;
    uint32_t flags;
    uint32_t type;
    char     name[32];
};

struct drm_mode_get_connector {
    uint64_t encoders_ptr;
    uint64_t modes_ptr;
    uint64_t props_ptr;
    uint64_t prop_values_ptr;
    uint32_t count_modes;
    uint32_t count_props;
    uint32_t count_encoders;
    uint32_t encoder_id;
    uint32_t connector_id;
    uint32_t connector_type;
    uint32_t connector_type_id;
    uint32_t connection;
    uint32_t mm_width;
    uint32_t mm_height;
    uint32_t subpixel;
    uint32_t pad;
};

struct drm_mode_get_encoder {
    uint32_t encoder_id;
    uint32_t encoder_type;
    uint32_t crtc_id;
    uint32_t possible_crtcs;
    uint32_t possible_clones;
};

struct drm_mode_crtc {
    uint64_t set_connectors_ptr;
    uint32_t count_connectors;
    uint32_t crtc_id;
    uint32_t fb_id;
    uint32_t x, y;
    uint32_t gamma_size;
    uint32_t mode_valid;
    struct drm_mode_modeinfo mode;
};

struct drm_mode_create_dumb {
    uint32_t height, width, bpp, flags;
    uint32_t handle, pitch;
    uint64_t size;
};
struct drm_mode_map_dumb { uint32_t handle; uint32_t pad; uint64_t offset; };
struct drm_mode_destroy_dumb { uint32_t handle; };

struct drm_mode_fb_cmd {
    uint32_t fb_id, width, height, pitch, bpp, depth, handle;
};
struct drm_mode_fb_cmd2 {
    uint32_t fb_id, width, height, pixel_format, flags;
    uint32_t handles[4], pitches[4], offsets[4];
    uint64_t modifier[4];
};

struct drm_mode_crtc_page_flip {
    uint32_t crtc_id, fb_id, flags, reserved;
    uint64_t user_data;
};

struct drm_clip_rect { uint16_t x1, y1, x2, y2; };
struct drm_mode_fb_dirty_cmd {
    uint32_t fb_id, flags, color, num_clips;
    uint64_t clips_ptr;
};

struct drm_mode_get_plane_res { uint64_t plane_id_ptr; uint32_t count_planes; };
struct drm_mode_get_plane {
    uint32_t plane_id, crtc_id, fb_id, possible_crtcs, gamma_size;
    uint32_t count_format_types;
    uint64_t format_type_ptr;
};
struct drm_mode_obj_get_properties {
    uint64_t props_ptr, prop_values_ptr;
    uint32_t count_props, obj_id, obj_type;
};
struct drm_mode_atomic {
    uint32_t flags, count_objs;
    uint64_t objs_ptr, count_props_ptr, props_ptr, prop_values_ptr;
    uint64_t reserved, user_data;
};
struct drm_gem_close { uint32_t handle, pad; };

/* PRIME fd passing */
struct drm_prime_handle {
    uint32_t handle;
    uint32_t flags;
    int32_t  fd;
};
#define DRM_PRIME_FLAG_IMPORT (1 << 0)
#define DRM_PRIME_FLAG_EXPORT (1 << 1)

/* Event structures */
struct drm_event { uint32_t type, length; };
struct drm_event_vblank {
    struct drm_event base;
    uint64_t user_data;
    uint32_t tv_sec, tv_usec, sequence, crtc_id;
};

/* ---- state ---------------------------------------------------------------- */
#define DRM_MAX_DUMB 16
#define DRM_MAX_FB   16
#define DRM_EVQ_MAX  16
#define DRM_MMAP_OFFSET_BASE 0x100000000ull  /* fake mmap offset space */

/* Declared with the rest of the state rather than down by the ioctl switch:
 * the PRIME block below shares both. Serialises every session, framebuffer,
 * dumb buffer and dma-buf block -- one lock, because they reference each
 * other (a framebuffer names a dumb buffer, a dumb buffer may name a prime
 * block) and split locking would let a free run between the two lookups. */
static spinlock_t g_lock;
static int g_inited;

typedef struct {
    uint8_t  used;
    uint32_t handle;
    uint32_t width, height, bpp, pitch;
    uint64_t size;
    uint32_t npages;
    void    *kva;        /* kernel virtual (pmm_alloc_pages) */
    uint64_t phys;       /* physical base */
    uint64_t map_offset; /* token returned by MAP_DUMB */
    /* >= 0 once this buffer has been exported: the g_primes slot that now
     * owns the pages, which this handle holds one reference on. < 0 means the
     * handle owns them itself and freeing means pmm_free_pages() directly.
     * Must be initialised to -1 on creation -- memset(0) would otherwise make
     * every buffer look exported into slot 0. */
    int32_t  prime;
} drm_dumb_t;

/* ---- PRIME (dma-buf) ----------------------------------------------------
 *
 * The moment a dumb buffer is exported, its pages are no longer private to
 * the session that allocated them: drmPrimeHandleToFD() hands a descriptor to
 * somebody else, who may well outlive the handle -- Mesa exports, then
 * DESTROY_DUMBs, then passes the fd across DRI3 to the X server. So from that
 * point the pages belong to a small refcounted block instead. The handle
 * holds one reference and every exported descriptor another; importing it
 * back into a session adds one per imported handle. The last reference out
 * frees the pages, exactly once -- which is also what stops a close of
 * /dev/dri/card0 from yanking memory out from under a dma-buf fd that is
 * still mapped somewhere. */
#define DRM_MAX_PRIME 128

typedef struct {
    uint8_t  used;
    uint32_t refs;
    void    *kva;
    uint32_t npages;
    uint64_t phys;
    uint64_t size;
    uint32_t width, height, bpp, pitch;
} drm_prime_t;

static drm_prime_t g_primes[DRM_MAX_PRIME];

/* Take `bo`'s pages into a dma-buf block and hand the handle its reference.
 * A handle whose `prime` is < 0 owns its pages outright, so there is never an
 * existing block to collide with -- no lookup needed, and none wanted: a
 * match on kva alone could mistake a recycled page frame for a live buffer.
 * Caller holds g_lock. Returns the slot, or -1 when the table is full (in
 * which case nothing changed and export simply fails). */
static int32_t prime_adopt_locked(const drm_dumb_t *bo)
{
    for (int i = 0; i < DRM_MAX_PRIME; i++) {
        drm_prime_t *p = &g_primes[i];
        if (p->used) continue;
        p->used = 1u;
        p->refs = 1u;              /* the handle adopting it */
        p->kva = bo->kva;
        p->npages = bo->npages;
        p->phys = bo->phys;
        p->size = bo->size;
        p->width = bo->width;
        p->height = bo->height;
        p->bpp = bo->bpp;
        p->pitch = bo->pitch;
        return i;
    }
    return -1;
}

/* Caller holds g_lock. */
static void prime_put_locked(int32_t idx)
{
    if (idx < 0 || idx >= DRM_MAX_PRIME) return;
    drm_prime_t *p = &g_primes[idx];
    if (!p->used) return;
    if (p->refs > 0u) p->refs--;
    if (p->refs != 0u) return;
    if (p->kva != NULL) pmm_free_pages(p->kva, p->npages);
    memset(p, 0, sizeof(*p));
}

/* Drop one reference on the pages `bo` holds -- through the dma-buf block
 * when it has been exported, directly when it has not -- and clear the slot.
 * Caller holds g_lock. The single funnel every DESTROY_DUMB / GEM_CLOSE /
 * session teardown path goes through. */
static void dumb_release_locked(drm_dumb_t *bo)
{
    if (bo == NULL || !bo->used) return;
    if (bo->prime >= 0) {
        prime_put_locked(bo->prime);
    } else if (bo->kva != NULL) {
        pmm_free_pages(bo->kva, bo->npages);
    }
    memset(bo, 0, sizeof(*bo));
}

/* Handle lookup over a bare session buffer table (card and render sessions
 * differ only in what surrounds theirs). Caller holds g_lock. */
static drm_dumb_t *dumb_find(drm_dumb_t *dumbs, uint32_t handle)
{
    for (int i = 0; i < DRM_MAX_DUMB; i++) {
        if (dumbs[i].used && dumbs[i].handle == handle) return &dumbs[i];
    }
    return NULL;
}

/* DRM_IOCTL_PRIME_HANDLE_TO_FD -- export a GEM handle as a dma-buf descriptor.
 *
 * Not a stub any more: the descriptor is a real object in the caller's fd
 * table whose mmap/ioctl/close hooks reach back into g_primes. The first
 * export moves the pages into a prime block (the handle keeping a reference),
 * and this descriptor takes one of its own, so the buffer outlives
 * DESTROY_DUMB and the close of /dev/dri/card0 -- which is precisely the
 * window Mesa works in: allocate, export, destroy, hand the fd to the X
 * server over DRI3. Before this it returned -EOPNOTSUPP and every one of
 * those handoffs turned into gbm_wrapper's "Failed to export buffer to
 * dma_buf" plus a slow path. */
static int64_t prime_handle_to_fd(drm_dumb_t *dumbs, void *uarg)
{
    struct drm_prime_handle p;
    if (!uarg || copy_from_user(&p, uarg, sizeof(p)) != 0u) return E_FAULT;

    int32_t slot = -1;
    uint32_t size = 0u;

    spinlock_lock(&g_lock);
    drm_dumb_t *bo = dumb_find(dumbs, p.handle);
    if (bo == NULL || bo->kva == NULL) { spinlock_unlock(&g_lock); return E_INVAL; }
    if (bo->prime < 0) {
        bo->prime = prime_adopt_locked(bo);
        if (bo->prime < 0) {
            spinlock_unlock(&g_lock);
            return E_NOMEM;
        }
    }
    slot = bo->prime;
    g_primes[slot].refs++;           /* the descriptor about to be installed */
    size = (uint32_t)g_primes[slot].size;
    spinlock_unlock(&g_lock);

    /* Deliberately outside g_lock: installing an fd takes the file table lock
     * with interrupts off, and DRM's lock is not in that ordering. The
     * reference taken above keeps `slot` alive across the gap. */
    int32_t fd = devfs_prime_fd(slot, size);
    if (fd < 0) {
        spinlock_lock(&g_lock);
        prime_put_locked(slot);
        spinlock_unlock(&g_lock);
        return (int64_t)fd;
    }

    p.fd = fd;
    return copy_to_user(uarg, &p, sizeof(p)) == 0u ? 0 : E_FAULT;
}

/* DRM_IOCTL_PRIME_FD_TO_HANDLE -- import a dma-buf descriptor as a GEM handle
 * in this session. Shares the pages rather than copying them; the imported
 * handle takes a reference on the same prime block. */
static int64_t prime_fd_to_handle(drm_dumb_t *dumbs, void *uarg,
                                  uint32_t *next_handle, uint64_t *next_map_off)
{
    struct drm_prime_handle p;
    if (!uarg || copy_from_user(&p, uarg, sizeof(p)) != 0u) return E_FAULT;

    /* The descriptor belongs to the calling process; resolve it there, before
     * taking g_lock (syscall_file_get_file_info() takes the file table lock). */
    vfs_file_t vf;
    if (syscall_file_get_file_info(p.fd, &vf, NULL) != 0) return E_INVAL;
    int32_t slot = devfs_prime_slot(&vf);
    if (slot < 0 || slot >= DRM_MAX_PRIME) return E_INVAL;

    spinlock_lock(&g_lock);
    if (!g_primes[slot].used) { spinlock_unlock(&g_lock); return E_INVAL; }
    drm_dumb_t *bo = NULL;
    for (int i = 0; i < DRM_MAX_DUMB; i++) {
        if (!dumbs[i].used) { bo = &dumbs[i]; break; }
    }
    if (bo == NULL) { spinlock_unlock(&g_lock); return E_NOMEM; }

    drm_prime_t *bp = &g_primes[slot];
    bp->refs++;                       /* the importing handle */
    memset(bo, 0, sizeof(*bo));
    bo->used = 1u;
    bo->handle = (*next_handle)++;
    bo->width = bp->width;
    bo->height = bp->height;
    bo->bpp = bp->bpp;
    bo->pitch = bp->pitch;
    bo->size = bp->size;
    bo->npages = bp->npages;
    bo->kva = bp->kva;
    bo->phys = bp->phys;
    bo->prime = slot;
    bo->map_offset = *next_map_off;
    *next_map_off += (uint64_t)bp->npages * 4096u;
    p.handle = bo->handle;
    spinlock_unlock(&g_lock);

    return copy_to_user(uarg, &p, sizeof(p)) == 0u ? 0 : E_FAULT;
}

typedef struct {
    uint8_t  used;
    uint32_t fb_id;
    uint32_t handle;   /* backing dumb handle */
    uint32_t width, height, pitch;
} drm_fb_t;

/*
 * One DRM session per X server.
 *
 * Everything below used to be a single set of file-scope globals, which
 * meant exactly one Xorg could ever be alive: a second server opening
 * /dev/dri/card0 would collide with the first on dumb-buffer handles (both
 * start at 1), on the framebuffer table, on the scanout id and on the event
 * queue. That is why a second Linux application had to join the running
 * server and draw inside the first one's window instead of getting a window
 * of its own.
 *
 * A session is claimed by the launcher that registers a redirection surface
 * (drm_kms_set_mirror), and bound to the first DRM client that is a
 * descendant of that launcher -- which is the Xorg it goes on to spawn. Every
 * ioctl is then served out of that server's own state, so N launchers run N
 * servers that never see each other.
 */
/* 8 MiB of redirection surface: 1600x1200x4 with room over. */
#define DRM_MIRROR_MAX_PAGES 2048u

#define DRM_MAX_SESSIONS 4

/* Each session's mmap tokens live in their own slice of the fake offset
 * space, so an offset identifies a session as well as a buffer. */
#define DRM_MMAP_OFFSET_STRIDE 0x40000000ull

typedef struct {
    uint8_t  used;
    int32_t  owner_pid;      /* launcher that registered the mirror, or -1  */
    int32_t  client_pid;     /* the DRM client (Xorg) bound to it, or -1    */

    drm_dumb_t dumbs[DRM_MAX_DUMB];
    drm_fb_t   fbs[DRM_MAX_FB];
    uint32_t   next_handle;
    uint32_t   next_fb_id;
    uint64_t   next_map_off;
    uint32_t   scanout_fb_id;
    uint32_t   flip_seq;

    struct drm_event_vblank evq[DRM_EVQ_MAX];
    uint32_t evq_head, evq_tail;

    /* Scanout redirection, described by physical pages rather than by a
     * virtual address: the blit runs in whatever process issued the flip
     * (Xorg), while the buffer belongs to the launcher, and the two do not
     * share an address space. The pages come from a shared-memory object
     * whose frames are allocated once and never moved, so caching them is
     * sound. */
    uint64_t mirror_pages[DRM_MIRROR_MAX_PAGES];
    uint32_t mirror_page_count;
    uint32_t mirror_w, mirror_h;
    volatile uint32_t mirror_dirty;
    /* Set once a redirection surface has been withdrawn, cleared when a new
     * one is registered. While set, flips are dropped instead of falling
     * back to the panel: the X server behind a closing session does not stop
     * the instant its mirror is released -- it regenerates when its last
     * client leaves and keeps presenting until it is killed -- and each of
     * those frames would otherwise be blitted straight over the window
     * manager. */
    uint8_t  mirror_released;
} drm_session_t;

static drm_session_t g_sessions[DRM_MAX_SESSIONS];

void drm_kms_init(void)
{
    if (g_inited) return;
    spinlock_init(&g_lock);
    memset(g_sessions, 0, sizeof(g_sessions));
    /* g_primes holds pages that exported descriptors may still be mapping,
     * so it is cleared only on the very first init -- a re-entry must not
     * orphan blocks whose dma-buf fds are alive. BSS already starts it zero. */
    memset(g_primes, 0, sizeof(g_primes));
    for (int i = 0; i < DRM_MAX_SESSIONS; i++) {
        g_sessions[i].owner_pid = -1;
        g_sessions[i].client_pid = -1;
    }
    /* The /sys side of both DRM nodes: libdrm rebuilds a device from
     * /sys/dev/char/<maj>:<min>... and drmGetDevice2() -- which Mesa's
     * loader_is_device_render_capable() is a wrapper around -- fails outright
     * without it. Runs here rather than from the sysfs driver so the PCI bus
     * has definitely been enumerated (fs_init is after driver_module_critical)
     * and the slot it reports is the machine's real display controller. */
    sysfs_publish_drm_nodes();
    g_inited = 1;
}

static void ensure_init(void) { if (!g_inited) drm_kms_init(); }

static drm_dumb_t *dumb_by_handle(drm_session_t *s, uint32_t h)
{
    for (int i = 0; i < DRM_MAX_DUMB; i++)
        if (s->dumbs[i].used && s->dumbs[i].handle == h) return &s->dumbs[i];
    return NULL;
}
static drm_dumb_t *dumb_by_offset(drm_session_t *s, uint64_t off)
{
    for (int i = 0; i < DRM_MAX_DUMB; i++)
        if (s->dumbs[i].used && s->dumbs[i].map_offset == off) return &s->dumbs[i];
    return NULL;
}
static drm_fb_t *fb_by_id(drm_session_t *s, uint32_t id)
{
    for (int i = 0; i < DRM_MAX_FB; i++)
        if (s->fbs[i].used && s->fbs[i].fb_id == id) return &s->fbs[i];
    return NULL;
}

/* ---- session lookup ----------------------------------------------------- */

static drm_session_t *session_alloc(int32_t owner_pid)
{
    for (int i = 0; i < DRM_MAX_SESSIONS; i++) {
        if (g_sessions[i].used) continue;
        drm_session_t *s = &g_sessions[i];
        memset(s, 0, sizeof(*s));
        s->used = 1u;
        s->owner_pid = owner_pid;
        s->client_pid = -1;
        s->next_handle = 1u;
        s->next_fb_id = 1u;
        s->next_map_off = DRM_MMAP_OFFSET_BASE +
                          (uint64_t)i * DRM_MMAP_OFFSET_STRIDE;
        return s;
    }
    return NULL;
}

static void session_free(drm_session_t *s)
{
    if (!s || !s->used) return;
    for (int i = 0; i < DRM_MAX_DUMB; i++) {
        dumb_release_locked(&s->dumbs[i]);
    }
    memset(s, 0, sizeof(*s));
    s->owner_pid = -1;
    s->client_pid = -1;
}

/* The session an unowned client (no launcher above it) draws through: one
 * shared slot that scans out to the panel, which is what a bare Xorg started
 * outside a launcher used to get and still does. */
static drm_session_t *session_panel(void)
{
    for (int i = 0; i < DRM_MAX_SESSIONS; i++)
        if (g_sessions[i].used && g_sessions[i].owner_pid < 0)
            return &g_sessions[i];
    return session_alloc(-1);
}

/*
 * Which server is calling.
 *
 * A bound client is matched by pid. An unbound one is matched by ancestry:
 * the launcher registered its mirror before it spawned Xorg, so the first
 * DRM client whose parent chain reaches that launcher is that launcher's
 * server, and claims the session. Everything else falls back to the panel
 * session.
 */
static drm_session_t *session_for_current(void)
{
    int32_t pid = process_get_current_pid();
    if (pid <= 0) return session_panel();

    for (int i = 0; i < DRM_MAX_SESSIONS; i++)
        if (g_sessions[i].used && g_sessions[i].client_pid == pid)
            return &g_sessions[i];

    /* Walk up the process tree, bounded: a cycle in parent pids would
     * otherwise hang the ioctl path. */
    int32_t ancestor = pid;
    for (uint32_t depth = 0u; depth < 16u && ancestor > 0; ++depth) {
        for (int i = 0; i < DRM_MAX_SESSIONS; i++) {
            drm_session_t *s = &g_sessions[i];
            if (!s->used || s->client_pid >= 0) continue;
            if (s->owner_pid != ancestor) continue;
            s->client_pid = pid;
            return s;
        }
        ancestor = process_get_parent_pid(ancestor);
    }
    return session_panel();
}

/* The session a launcher owns, for the calls it makes itself. */
static drm_session_t *session_for_owner(int32_t owner_pid)
{
    for (int i = 0; i < DRM_MAX_SESSIONS; i++)
        if (g_sessions[i].used && g_sessions[i].owner_pid == owner_pid)
            return &g_sessions[i];
    return NULL;
}

/* ---- render node (renderD128) ------------------------------------------
 *
 * The render node has NO modesetting capability: it exposes only GEM
 * buffer-object operations, PRIME import/export, and syncobj. Mesa opens
 * this node for GL contexts; Xorg's DDX opens card0 for scanout. Keeping
 * them separate matches Linux and prevents a GPU client from accidentally
 * hijacking the display.
 *
 * Render nodes have their OWN session slot (not shared with card0's scanout
 * session). The session is keyed by client pid only (no launcher binding).
 */

#define DRM_MAX_RENDER_SESSIONS 4

typedef struct {
    uint8_t  used;
    int32_t  client_pid;

    drm_dumb_t dumbs[DRM_MAX_DUMB];
    uint32_t   next_handle;
    uint64_t   next_map_off;
} drm_render_session_t;

static drm_render_session_t g_render_sessions[DRM_MAX_RENDER_SESSIONS];

static drm_render_session_t *render_session_alloc(int32_t pid)
{
    for (int i = 0; i < DRM_MAX_RENDER_SESSIONS; i++) {
        if (g_render_sessions[i].used) continue;
        drm_render_session_t *s = &g_render_sessions[i];
        memset(s, 0, sizeof(*s));
        s->used = 1u;
        s->client_pid = pid;
        s->next_handle = 1u;
        /* Separate mmap-offset space from card sessions (0x100000000 base,
         * sessions at +0x40000000 stride; render uses 0x80000000000). */
        s->next_map_off = 0x800000000ull + (uint64_t)i * DRM_MMAP_OFFSET_STRIDE;
        return s;
    }
    return NULL;
}

static drm_render_session_t *render_session_for_current(void)
{
    int32_t pid = process_get_current_pid();
    if (pid <= 0) return NULL;

    for (int i = 0; i < DRM_MAX_RENDER_SESSIONS; i++)
        if (g_render_sessions[i].used &&
            g_render_sessions[i].client_pid == pid)
            return &g_render_sessions[i];

    return render_session_alloc(pid);
}

static void render_session_free(drm_render_session_t *s)
{
    if (!s || !s->used) return;
    for (int i = 0; i < DRM_MAX_DUMB; i++) {
        dumb_release_locked(&s->dumbs[i]);
    }
    memset(s, 0, sizeof(*s));
}

static drm_dumb_t *render_dumb_by_handle(drm_render_session_t *s, uint32_t h)
{
    for (int i = 0; i < DRM_MAX_DUMB; i++)
        if (s->dumbs[i].used && s->dumbs[i].handle == h)
            return &s->dumbs[i];
    return NULL;
}

static drm_dumb_t *render_dumb_by_offset(drm_render_session_t *s, uint64_t off)
{
    for (int i = 0; i < DRM_MAX_DUMB; i++)
        if (s->dumbs[i].used && s->dumbs[i].map_offset == off)
            return &s->dumbs[i];
    return NULL;
}

int64_t drm_render_ioctl(uint64_t request, uint64_t arg)
{
    ensure_init();
    drm_render_session_t *sess = render_session_for_current();
    if (!sess) return E_NOMEM;
    if (IOC_TYPE(request) != (uint32_t)DRM_IOCTL_BASE) return E_NOTTY;
    uint32_t nr = IOC_NR(request);
    void *uarg = (void *)(uintptr_t)arg;

    /* Modeset ioctls are rejected on the render node (Linux returns EACCES). */
    switch (nr) {
    case DRM_NR_MODE_SETCRTC:
    case DRM_NR_MODE_PAGE_FLIP:
    case DRM_NR_MODE_DIRTYFB:
    case DRM_NR_MODE_SETPLANE:
    case DRM_NR_MODE_ATOMIC:
    case DRM_NR_MODE_CURSOR:
    case DRM_NR_MODE_CURSOR2:
    case DRM_NR_MODE_SETGAMMA:
    case DRM_NR_MODE_SETPROPERTY:
    case DRM_NR_MODE_OBJ_SETPROP:
    case DRM_NR_MODE_LIST_LESSEES:
        return -13; /* -EACCES */
    default:
        break;
    }

    switch (nr) {
    case DRM_NR_VERSION: {
        struct drm_version v;
        if (!uarg || copy_from_user(&v, uarg, sizeof(v)) != 0u) return E_FAULT;
        static const char nm[] = "implusdrm";
        static const char dt[] = "20261001";
        static const char ds[] = "ImplusOS render node";
        uint64_t nl = sizeof(nm) - 1, dl = sizeof(dt) - 1, sl = sizeof(ds) - 1;
        if (v.name && v.name_len >= nl)
            if (copy_to_user((void *)(uintptr_t)v.name, nm, nl) != 0u) return E_FAULT;
        if (v.date && v.date_len >= dl)
            if (copy_to_user((void *)(uintptr_t)v.date, dt, dl) != 0u) return E_FAULT;
        if (v.desc && v.desc_len >= sl)
            if (copy_to_user((void *)(uintptr_t)v.desc, ds, sl) != 0u) return E_FAULT;
        v.version_major = 1; v.version_minor = 0; v.version_patchlevel = 0;
        v.name_len = nl; v.date_len = dl; v.desc_len = sl;
        if (copy_to_user(uarg, &v, sizeof(v)) != 0u) return E_FAULT;
        return 0;
    }
    case DRM_NR_GET_MAGIC: {
        uint32_t magic = 1;
        if (uarg && copy_to_user(uarg, &magic, sizeof(magic)) != 0u) return E_FAULT;
        return 0;
    }
    case DRM_NR_AUTH_MAGIC:
        /* drmAuthMagic(): authentication is implicit on a render node (and
         * Xorg's DRI3 open path needs this to *not* fail on card0 either),
         * so accept whatever magic GET_MAGIC handed out. */
        return 0;
    case DRM_NR_GET_CAP: {
        struct drm_get_cap c;
        if (!uarg || copy_from_user(&c, uarg, sizeof(c)) != 0u) return E_FAULT;
        switch (c.capability) {
        case DRM_CAP_DUMB_BUFFER:            c.value = 1; break;
        case DRM_CAP_DUMB_PREFERRED_DEPTH:   c.value = 24; break;
        case DRM_CAP_DUMB_PREFER_SHADOW:     c.value = 1; break;
        case DRM_CAP_TIMESTAMP_MONOTONIC:    c.value = 1; break;
        case DRM_CAP_PRIME:                  c.value = 1; break;
        case DRM_CAP_ADDFB2_MODIFIERS:       c.value = 1; break;
        case DRM_CAP_SYNCOBJ:                c.value = 1; break;
        default:                             c.value = 0; break;
        }
        return copy_to_user(uarg, &c, sizeof(c)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_SET_CLIENT_CAP: {
        /* Render node accepts UNIVERSAL_PLANES and ATOMIC caps (they only
         * matter for modesetting, which is rejected above anyway). */
        struct drm_set_client_cap cap;
        if (!uarg || copy_from_user(&cap, uarg, sizeof(cap)) != 0u) return E_FAULT;
        return 0;
    }
    case DRM_NR_SET_MASTER:
    case DRM_NR_DROP_MASTER:
        return 0;

    case DRM_NR_MODE_CREATE_DUMB: {
        struct drm_mode_create_dumb d;
        if (!uarg || copy_from_user(&d, uarg, sizeof(d)) != 0u) return E_FAULT;
        /* Linux accepts any non-zero bpp here and rounds the bytes-per-pixel
         * up, so an 8-bit (R8) allocation is as valid as a 32-bit scanout one.
         * Refusing bpp=8 made Mesa's gbm fail its 64x64 luminance buffer. */
        if (!d.width || !d.height || !d.bpp || d.bpp > 64u) {
            return E_INVAL;
        }
        uint32_t bpp = d.bpp;
        uint32_t pitch = d.width * ((d.bpp + 7u) / 8u);
        uint64_t size = (uint64_t)pitch * d.height;
        uint32_t npages = (uint32_t)((size + 4095u) / 4096u);
        spinlock_lock(&g_lock);
        drm_dumb_t *slot = NULL;
        for (int i = 0; i < DRM_MAX_DUMB; i++)
            if (!sess->dumbs[i].used) { slot = &sess->dumbs[i]; break; }
        if (!slot) { spinlock_unlock(&g_lock); return E_NOMEM; }
        void *kva = pmm_alloc_pages(npages);
        if (!kva) { spinlock_unlock(&g_lock); return E_NOMEM; }
        memset(kva, 0, (size_t)npages * 4096u);
        slot->used = 1;
        slot->handle = sess->next_handle++;
        slot->width = d.width; slot->height = d.height;
        slot->bpp = bpp; slot->pitch = pitch;
        slot->size = size; slot->npages = npages;
        slot->kva = kva;
        slot->phys = paging_virt_to_phys(paging_get_kernel_cr3(),
                                         (uint64_t)(uintptr_t)kva);
        slot->map_offset = sess->next_map_off;
        slot->prime = -1;
        sess->next_map_off += (uint64_t)npages * 4096u;
        d.handle = slot->handle;
        d.pitch = pitch;
        d.size = size;
        spinlock_unlock(&g_lock);
        return copy_to_user(uarg, &d, sizeof(d)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_MAP_DUMB: {
        struct drm_mode_map_dumb m;
        if (!uarg || copy_from_user(&m, uarg, sizeof(m)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_dumb_t *bo = render_dumb_by_handle(sess, m.handle);
        if (!bo) { spinlock_unlock(&g_lock); return E_INVAL; }
        m.offset = bo->map_offset;
        spinlock_unlock(&g_lock);
        return copy_to_user(uarg, &m, sizeof(m)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_DESTROY_DUMB: {
        struct drm_mode_destroy_dumb d;
        if (!uarg || copy_from_user(&d, uarg, sizeof(d)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_dumb_t *bo = render_dumb_by_handle(sess, d.handle);
        if (bo) dumb_release_locked(bo);
        spinlock_unlock(&g_lock);
        return 0;
    }
    case DRM_NR_GEM_CLOSE: {
        struct drm_gem_close g;
        if (uarg && copy_from_user(&g, uarg, sizeof(g)) == 0u) {
            spinlock_lock(&g_lock);
            drm_dumb_t *bo = render_dumb_by_handle(sess, g.handle);
            if (bo) dumb_release_locked(bo);
            spinlock_unlock(&g_lock);
        }
        return 0;
    }
    case DRM_NR_GEM_FLINK:
    case DRM_NR_GEM_OPEN:
        return E_INVAL; /* no flink name sharing yet */
    case DRM_NR_PRIME_HANDLE_TO_FD:
        /* The render node exports exactly as the KMS node does: the buffer's
         * pages move into a refcounted dma-buf block and a descriptor naming
         * them is installed in this process. */
        return prime_handle_to_fd(sess->dumbs, uarg);
    case DRM_NR_PRIME_FD_TO_HANDLE:
        return prime_fd_to_handle(sess->dumbs, uarg, &sess->next_handle,
                                  &sess->next_map_off);
    default:
        return E_NOTTY;
    }
}

int64_t drm_render_read(uint8_t *user_buf, uint64_t len, uint32_t nonblock)
{
    (void)user_buf; (void)len; (void)nonblock;
    return E_AGAIN; /* render node has no event queue */
}

uint32_t drm_render_poll(uint32_t events)
{
    (void)events;
    return 0; /* no pollable events on render node */
}

int64_t drm_render_mmap(uint64_t offset, uint64_t length, uint64_t prot,
                        uint64_t flags)
{
    ensure_init();
    drm_render_session_t *sess = render_session_for_current();
    if (!sess) return E_NOMEM;
    (void)prot; (void)flags;
    spinlock_lock(&g_lock);
    drm_dumb_t *bo = render_dumb_by_offset(sess, offset);
    if (!bo || !bo->kva) { spinlock_unlock(&g_lock); return E_INVAL; }
    uint64_t need = bo->size;
    uint64_t bo_phys = bo->phys;
    uint32_t np = bo->npages;
    spinlock_unlock(&g_lock);

    if (length > (uint64_t)np * 4096u) length = (uint64_t)np * 4096u;
    if (length == 0) length = need;

    uint64_t cr3 = process_get_current_cr3();
    if (!cr3) return E_NODEV;
    void *va = process_user_reserve((uint64_t)np * 4096u);
    if (!va) return E_NOMEM;
    uint64_t uva = (uint64_t)(uintptr_t)va;
    for (uint32_t i = 0; i < np; i++) {
        /* PAGE_EXTERNAL: these frames belong to the dumb buffer, not to this
         * address space.  Without it the client's munmap -- and, on every
         * exit, the sweep in paging_destroy_process_space() -- called
         * free_page() on frames the session still owns, and DESTROY_DUMB /
         * drm_kms_close() then freed them a second time.  That second free
         * put a still-in-use frame back on the PMM free list, so the next
         * two alloc_page() callers were handed the SAME frame and whichever
         * lost the race had its data overwritten: Xorg died on
         * "malloc(): corrupted top size" moments after Chromium exited.
         * The session is the single owner and frees them exactly once. */
        if (paging_map_user_page(cr3, uva + (uint64_t)i * 4096u,
                                 bo_phys + (uint64_t)i * 4096u,
                                 PAGE_PRESENT | PAGE_RW | PAGE_USER |
                                 PAGE_EXTERNAL) < 0) {
            (void)process_user_munmap(va, (uint64_t)np * 4096u);
            return E_NOMEM;
        }
    }
    return (int64_t)uva;
}

void drm_render_close(void)
{
    ensure_init();
    drm_render_session_t *sess = render_session_for_current();
    if (!sess) return;
    {
        uint32_t n = 0;
        for (int i = 0; i < DRM_MAX_DUMB; i++)
            if (sess->dumbs[i].used && sess->dumbs[i].kva) n++;
        serial_write_string("[drm] render close pid=");
        serial_write_uint32((uint32_t)process_get_current_pid());
        serial_write_string(" sess=");
        serial_write_uint64((uint64_t)(uintptr_t)sess);
        serial_write_string(" dumbs=");
        serial_write_uint32(n);
        serial_write_string("\n");
    }
    spinlock_lock(&g_lock);
    render_session_free(sess);
    spinlock_unlock(&g_lock);
}

int drm_kms_set_mirror(uint64_t pixels, uint32_t width, uint32_t height)
{
    ensure_init();
    int32_t owner = process_get_current_pid();
    drm_session_t *s = session_for_owner(owner);

    if (pixels == 0u) {
        if (!s) return 0;
        if (s->mirror_page_count != 0u) s->mirror_released = 1u;
        s->mirror_page_count = 0u;
        s->mirror_w = s->mirror_h = 0u;
        return 0;
    }
    if (!s) s = session_alloc(owner);
    if (!s) return E_NOMEM;
    if (width == 0u || height == 0u) return E_INVAL;
    if ((pixels & (PAGE_SIZE - 1u)) != 0u) return E_INVAL;

    uint64_t bytes = (uint64_t)width * (uint64_t)height * 4u;
    uint64_t pages = (bytes + PAGE_SIZE - 1u) / PAGE_SIZE;
    if (pages > DRM_MIRROR_MAX_PAGES) return E_NOMEM;

    uint64_t cr3 = process_get_current_cr3();
    if (cr3 == 0u) return E_INVAL;

    /* Resolve every page up front. A partially resolved mirror is worse than
     * none: the blit would write the rows it can and leave the rest holding
     * an older frame, which reads as corruption rather than as a failure. */
    for (uint64_t i = 0; i < pages; ++i) {
        uint64_t phys = paging_virt_to_phys(cr3, pixels + i * PAGE_SIZE);
        if (phys == 0u) {
            s->mirror_page_count = 0u;
            return E_FAULT;
        }
        s->mirror_pages[i] = phys & ~((uint64_t)PAGE_SIZE - 1u);
    }
    s->mirror_w = width;
    s->mirror_h = height;
    s->mirror_page_count = (uint32_t)pages;
    s->mirror_released = 0u;
    return 0;
}

void drm_kms_notify_process_exit(int32_t pid)
{
    if (pid < 0) return;
    for (int i = 0; i < DRM_MAX_SESSIONS; i++) {
        drm_session_t *s = &g_sessions[i];
        if (!s->used) continue;
        /* The launcher going away takes the whole session with it: the
         * surface it registered belongs to a shared-memory object the
         * window manager frees with the window, and a flip landing in
         * those pages afterwards would write over whatever now owns them. */
        if (s->owner_pid == pid) { session_free(s); continue; }
        /* The server going away leaves the session for the next one the
         * launcher starts, but unbinds it so that one can claim it. */
        if (s->client_pid == pid) s->client_pid = -1;
    }
    /* Clean up render-node sessions for this pid. */
    for (int i = 0; i < DRM_MAX_RENDER_SESSIONS; i++) {
        if (g_render_sessions[i].used &&
            g_render_sessions[i].client_pid == pid) {
            render_session_free(&g_render_sessions[i]);
        }
    }
}

int drm_kms_mirror_take_dirty(void)
{
    drm_session_t *s = session_for_owner(process_get_current_pid());
    if (!s) return 0;
    uint32_t d = s->mirror_dirty;
    s->mirror_dirty = 0u;
    return d != 0u;
}

/* memcpy into the page-scattered mirror at a byte offset. */
static void mirror_write(drm_session_t *s, uint64_t offset,
                         const uint8_t *src, uint32_t len)
{
    while (len > 0u) {
        uint32_t page = (uint32_t)(offset / PAGE_SIZE);
        if (page >= s->mirror_page_count) return;
        uint32_t in_page = (uint32_t)(offset % PAGE_SIZE);
        uint32_t chunk = (uint32_t)PAGE_SIZE - in_page;
        if (chunk > len) chunk = len;
        memcpy((uint8_t *)(uintptr_t)s->mirror_pages[page] + in_page, src, chunk);
        offset += chunk;
        src += chunk;
        len -= chunk;
    }
}

/* Blit a dumb buffer to the hardware framebuffer (XRGB8888, 32bpp assumed),
 * or into the redirection surface when one is registered. */
static void blit_fb_to_display(drm_session_t *s, drm_fb_t *fb)
{
    if (!fb) return;
    drm_dumb_t *bo = dumb_by_handle(s, fb->handle);
    if (!bo || !bo->kva) return;
    /* A framebuffer only ever reaches this as 32bpp (the copies below are
     * fixed 4-byte rows). Mesa allocates non-32bpp buffers too -- an 8-bit R8
     * texture, say -- so check the layout instead of assuming it: every row
     * copied has to exist inside the buffer, or the last row reads past it. */
    uint32_t src_pitch = fb->pitch ? fb->pitch : (bo->pitch ? bo->pitch : fb->width * 4u);
    if ((uint64_t)src_pitch < (uint64_t)fb->width * 4u ||
        (uint64_t)src_pitch * (uint64_t)fb->height > bo->size) {
        return;
    }

    if (s->mirror_page_count != 0u) {
        uint32_t cw = (fb->width  < s->mirror_w) ? fb->width  : s->mirror_w;
        uint32_t ch = (fb->height < s->mirror_h) ? fb->height : s->mirror_h;
        uint32_t dst_pitch = s->mirror_w * 4u;
        for (uint32_t y = 0; y < ch; y++) {
            mirror_write(s, (uint64_t)y * dst_pitch,
                         (const uint8_t *)bo->kva + (size_t)y * src_pitch,
                         cw * 4u);
        }
        s->mirror_dirty = 1u;
        /* No display_present(): the window manager owns the panel now and
         * will composite this surface on its own schedule. */
        return;
    }

    /* A server that never had a mirror drives the panel as before; one
     * whose mirror has been withdrawn draws nowhere. */
    if (s->mirror_released) return;

    void *hw = display_get_framebuffer();
    if (!hw) return;
    uint32_t hw_w = display_width();
    uint32_t hw_h = display_height();
    uint32_t hw_pitch = hw_w * 4u;
    uint32_t cw = (fb->width  < hw_w) ? fb->width  : hw_w;
    uint32_t ch = (fb->height < hw_h) ? fb->height : hw_h;
    for (uint32_t y = 0; y < ch; y++) {
        memcpy((uint8_t *)hw + (size_t)y * hw_pitch,
               (uint8_t *)bo->kva + (size_t)y * src_pitch,
               (size_t)cw * 4u);
    }
    display_present();
}

static void queue_flip_event(drm_session_t *s, uint64_t user_data,
                             uint32_t crtc_id)
{
    uint32_t next = (s->evq_head + 1u) % DRM_EVQ_MAX;
    if (next == s->evq_tail) return; /* drop on overflow */
    struct drm_event_vblank *e = &s->evq[s->evq_head];
    memset(e, 0, sizeof(*e));
    e->base.type = DRM_EVENT_FLIP_COMPLETE;
    e->base.length = (uint32_t)sizeof(*e);
    e->user_data = user_data;
    uint32_t hz = timer_hz(); if (!hz) hz = 60u;
    uint64_t ms = (timer_ticks() * 1000ull) / hz;
    e->tv_sec = (uint32_t)(ms / 1000ull);
    e->tv_usec = (uint32_t)((ms % 1000ull) * 1000ull);
    e->sequence = ++s->flip_seq;
    e->crtc_id = crtc_id;
    s->evq_head = next;
}

/* Append decimal `v` to buf at *pos (buf is >= 32); no NUL. */
static void append_u32(char *buf, int *pos, uint32_t v)
{
    char rev[10];
    int ri = 0;
    if (v == 0u) {
        rev[ri++] = '0';
    }
    while (v != 0u) {
        rev[ri++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (ri > 0 && *pos < 30) {
        buf[(*pos)++] = rev[--ri];
    }
}

/* The size the server should drive. With scanout redirected this is the
 * client surface, not the panel: X reads the connector's mode once at
 * startup, so advertising the window size here is what makes it render at
 * exactly that size instead of full-screen and clipped. */
static uint32_t mode_width(const drm_session_t *s)
{
    return s->mirror_page_count != 0u ? s->mirror_w : display_width();
}
static uint32_t mode_height(const drm_session_t *s)
{
    return s->mirror_page_count != 0u ? s->mirror_h : display_height();
}

/* Fill a single 60Hz mode sized to the current display. */
static void fill_mode(const drm_session_t *s, struct drm_mode_modeinfo *m)
{
    uint32_t w = mode_width(s);
    uint32_t h = mode_height(s);
    if (w == 0u) {
        w = 1024u;
    }
    if (h == 0u) {
        h = 768u;
    }
    memset(m, 0, sizeof(*m));
    m->clock = (uint32_t)(((uint64_t)w * h * 60ull) / 1000ull);
    m->hdisplay = (uint16_t)w;
    m->hsync_start = (uint16_t)(w + 8u);
    m->hsync_end = (uint16_t)(w + 16u);
    m->htotal = (uint16_t)(w + 32u);
    m->vdisplay = (uint16_t)h;
    m->vsync_start = (uint16_t)(h + 2u);
    m->vsync_end = (uint16_t)(h + 4u);
    m->vtotal = (uint16_t)(h + 8u);
    m->vrefresh = 60u;
    m->type = 1u << 3; /* DRM_MODE_TYPE_PREFERRED */
    int p = 0;
    append_u32(m->name, &p, w);
    if (p < 30) {
        m->name[p++] = 'x';
    }
    append_u32(m->name, &p, h);
    m->name[p] = '\0';
}

/* Write up to `cap` u32 IDs to user array `uptr`; always return real count. */
static int64_t write_id_array(uint64_t uptr, uint32_t cap, const uint32_t *ids,
                              uint32_t count)
{
    if (uptr && cap >= count && count) {
        if (copy_to_user((void *)(uintptr_t)uptr, ids,
                         (uint64_t)count * sizeof(uint32_t)) != 0u)
            return E_FAULT;
    }
    return 0;
}

/* ---- ioctl ------------------------------------------------------------- */
int64_t drm_kms_ioctl(uint64_t request, uint64_t arg)
{
    ensure_init();
    drm_session_t *sess = session_for_current();
    if (!sess) return E_NOMEM;
    if (IOC_TYPE(request) != (uint32_t)DRM_IOCTL_BASE) return E_NOTTY;
    uint32_t nr = IOC_NR(request);
    void *uarg = (void *)(uintptr_t)arg;

    switch (nr) {
    case DRM_NR_VERSION: {
        struct drm_version v;
        if (!uarg || copy_from_user(&v, uarg, sizeof(v)) != 0u) return E_FAULT;
        /* Advertise "kms_swrast" so Mesa's loader maps this device to its
         * kms_swrast DRI driver (dumb-buffer KMS + software rasterizer).
         * Mesa's loader tries <kernel-driver-name>_dri.so; without a known
         * name glamor's eglGetPlatformDisplay(GBM) has no vendor driver to
         * bind and reports "couldn't get display device". */
        static const char nm[] = "kms_swrast";
        static const char dt[] = "20261001";
        static const char ds[] = "ImplusOS KMS shim";
        uint64_t nl = sizeof(nm) - 1, dl = sizeof(dt) - 1, sl = sizeof(ds) - 1;
        if (v.name && v.name_len >= nl)
            if (copy_to_user((void *)(uintptr_t)v.name, nm, nl) != 0u) return E_FAULT;
        if (v.date && v.date_len >= dl)
            if (copy_to_user((void *)(uintptr_t)v.date, dt, dl) != 0u) return E_FAULT;
        if (v.desc && v.desc_len >= sl)
            if (copy_to_user((void *)(uintptr_t)v.desc, ds, sl) != 0u) return E_FAULT;
        v.version_major = 1; v.version_minor = 0; v.version_patchlevel = 0;
        v.name_len = nl; v.date_len = dl; v.desc_len = sl;
        if (copy_to_user(uarg, &v, sizeof(v)) != 0u) return E_FAULT;
        return 0;
    }
    case DRM_NR_GET_MAGIC: {
        uint32_t magic = 1;
        if (uarg && copy_to_user(uarg, &magic, sizeof(magic)) != 0u) return E_FAULT;
        return 0;
    }
    case DRM_NR_SET_VERSION: {
        /* drmSetVersion(): Mesa/libdrm calls this during device init. */
        struct { int32_t di_major, di_minor, dd_major, dd_minor; } sv;
        if (!uarg || copy_from_user(&sv, uarg, sizeof(sv)) != 0u) return E_FAULT;
        sv.di_major = 1; sv.di_minor = 4;
        sv.dd_major = 1; sv.dd_minor = 0;
        return copy_to_user(uarg, &sv, sizeof(sv)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_AUTH_MAGIC: {
        (void)uarg;
        return 0;
    }
    case DRM_NR_GET_CAP: {
        struct drm_get_cap c;
        if (!uarg || copy_from_user(&c, uarg, sizeof(c)) != 0u) return E_FAULT;
        switch (c.capability) {
        case DRM_CAP_DUMB_BUFFER:            c.value = 1; break;
        case DRM_CAP_DUMB_PREFERRED_DEPTH:   c.value = 24; break;
        case DRM_CAP_DUMB_PREFER_SHADOW:     c.value = 1; break;
        case DRM_CAP_TIMESTAMP_MONOTONIC:    c.value = 1; break;
        case DRM_CAP_CRTC_IN_VBLANK_EVENT:   c.value = 1; break;
        case DRM_CAP_CURSOR_WIDTH:
        case DRM_CAP_CURSOR_HEIGHT:          c.value = 64; break;
        case DRM_CAP_PRIME:                  c.value = 1; break; /* dma-buf */
        case DRM_CAP_ADDFB2_MODIFIERS:       c.value = 1; break;
        case DRM_CAP_SYNCOBJ:                c.value = 1; break;
        default:                             c.value = 0; break;
        }
        return copy_to_user(uarg, &c, sizeof(c)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_SET_CLIENT_CAP: {
        /* Accept UNIVERSAL_PLANES and ATOMIC caps. The DDX/Mesa will then
         * attempt atomic modesetting; we implement a simplified atomic path
         * below (MODE_ATOMIC) that treats the whole state as one blob. */
        struct drm_set_client_cap cap;
        if (!uarg || copy_from_user(&cap, uarg, sizeof(cap)) != 0u) return E_FAULT;
        /* DRM_CLIENT_CAP_UNIVERSAL_PLANES = 2
         * DRM_CLIENT_CAP_ATOMIC = 3
         * DRM_CLIENT_CAP_ASPECT_RATIO = 4 (accepted, informational) */
        if (cap.capability == 2 || cap.capability == 3 || cap.capability == 4)
            return 0;
        /* DRM_CLIENT_CAP_STEREO_3D = 1: not supported. */
        return -95; /* -EOPNOTSUPP */
    }
    case DRM_NR_SET_MASTER:
    case DRM_NR_DROP_MASTER:
        return 0;

    case DRM_NR_MODE_GETRESOURCES: {
        struct drm_mode_card_res r;
        if (!uarg || copy_from_user(&r, uarg, sizeof(r)) != 0u) return E_FAULT;
        uint32_t crtc = DRM_CRTC_ID, conn = DRM_CONNECTOR_ID, enc = DRM_ENCODER_ID;
        int64_t e;
        if ((e = write_id_array(r.crtc_id_ptr, r.count_crtcs, &crtc, 1)) < 0) return e;
        if ((e = write_id_array(r.connector_id_ptr, r.count_connectors, &conn, 1)) < 0) return e;
        if ((e = write_id_array(r.encoder_id_ptr, r.count_encoders, &enc, 1)) < 0) return e;
        r.count_fbs = 0;
        r.count_crtcs = 1;
        r.count_connectors = 1;
        r.count_encoders = 1;
        r.min_width = 320;  r.max_width = 8192;
        r.min_height = 200; r.max_height = 8192;
        return copy_to_user(uarg, &r, sizeof(r)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_GETCONNECTOR: {
        struct drm_mode_get_connector c;
        if (!uarg || copy_from_user(&c, uarg, sizeof(c)) != 0u) return E_FAULT;
        uint32_t enc = DRM_ENCODER_ID;
        int64_t e;
        if ((e = write_id_array(c.encoders_ptr, c.count_encoders, &enc, 1)) < 0) return e;
        if (c.modes_ptr && c.count_modes >= 1u) {
            struct drm_mode_modeinfo m;
            fill_mode(sess, &m);
            if (copy_to_user((void *)(uintptr_t)c.modes_ptr, &m, sizeof(m)) != 0u)
                return E_FAULT;
        }
        c.count_encoders = 1;
        c.count_modes = 1;
        c.count_props = 0;
        c.encoder_id = DRM_ENCODER_ID;
        c.connector_id = DRM_CONNECTOR_ID;
        c.connector_type = 2;      /* DVID-ish; any nonzero */
        c.connector_type_id = 1;
        c.connection = DRM_MODE_CONNECTED;
        c.mm_width = 520;
        c.mm_height = 320;
        c.subpixel = 1;
        return copy_to_user(uarg, &c, sizeof(c)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_GETENCODER: {
        struct drm_mode_get_encoder en;
        if (!uarg || copy_from_user(&en, uarg, sizeof(en)) != 0u) return E_FAULT;
        en.encoder_type = 2;
        en.crtc_id = DRM_CRTC_ID;
        en.possible_crtcs = 1u;
        en.possible_clones = 0u;
        en.encoder_id = DRM_ENCODER_ID;
        return copy_to_user(uarg, &en, sizeof(en)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_GETCRTC: {
        struct drm_mode_crtc cc;
        if (!uarg || copy_from_user(&cc, uarg, sizeof(cc)) != 0u) return E_FAULT;
        cc.crtc_id = DRM_CRTC_ID;
        cc.fb_id = sess->scanout_fb_id;
        cc.x = cc.y = 0;
        cc.gamma_size = 0;
        cc.mode_valid = sess->scanout_fb_id ? 1u : 0u;
        if (sess->scanout_fb_id) fill_mode(sess, &cc.mode);
        else memset(&cc.mode, 0, sizeof(cc.mode));
        return copy_to_user(uarg, &cc, sizeof(cc)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_SETCRTC: {
        struct drm_mode_crtc cc;
        if (!uarg || copy_from_user(&cc, uarg, sizeof(cc)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        sess->scanout_fb_id = cc.fb_id;
        drm_fb_t *fb = fb_by_id(sess, cc.fb_id);
        if (fb) blit_fb_to_display(sess, fb);
        spinlock_unlock(&g_lock);
        return 0;
    }
    case DRM_NR_MODE_CREATE_DUMB: {
        struct drm_mode_create_dumb d;
        if (!uarg || copy_from_user(&d, uarg, sizeof(d)) != 0u) return E_FAULT;
        /* Linux accepts any non-zero bpp here and rounds the bytes-per-pixel
         * up, so an 8-bit (R8) allocation is as valid as a 32-bit scanout one.
         * Refusing bpp=8 made Mesa's gbm fail its 64x64 luminance buffer. */
        if (!d.width || !d.height || !d.bpp || d.bpp > 64u) {
            return E_INVAL;
        }
        uint32_t bpp = d.bpp;
        uint32_t pitch = d.width * ((d.bpp + 7u) / 8u);
        uint64_t size = (uint64_t)pitch * d.height;
        uint32_t npages = (uint32_t)((size + 4095u) / 4096u);
        spinlock_lock(&g_lock);
        drm_dumb_t *slot = NULL;
        for (int i = 0; i < DRM_MAX_DUMB; i++) if (!sess->dumbs[i].used) { slot = &sess->dumbs[i]; break; }
        if (!slot) { spinlock_unlock(&g_lock); return E_NOMEM; }
        void *kva = pmm_alloc_pages(npages);
        if (!kva) { spinlock_unlock(&g_lock); return E_NOMEM; }
        memset(kva, 0, (size_t)npages * 4096u);
        slot->used = 1;
        slot->handle = sess->next_handle++;
        slot->width = d.width; slot->height = d.height;
        slot->bpp = bpp; slot->pitch = pitch;
        slot->size = size; slot->npages = npages;
        slot->kva = kva;
        slot->phys = paging_virt_to_phys(paging_get_kernel_cr3(), (uint64_t)(uintptr_t)kva);
        slot->map_offset = sess->next_map_off;
        slot->prime = -1;
        sess->next_map_off += (uint64_t)npages * 4096u;
        d.handle = slot->handle;
        d.pitch = pitch;
        d.size = size;
        spinlock_unlock(&g_lock);
        return copy_to_user(uarg, &d, sizeof(d)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_MAP_DUMB: {
        struct drm_mode_map_dumb m;
        if (!uarg || copy_from_user(&m, uarg, sizeof(m)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_dumb_t *bo = dumb_by_handle(sess, m.handle);
        if (!bo) { spinlock_unlock(&g_lock); return E_INVAL; }
        m.offset = bo->map_offset;
        spinlock_unlock(&g_lock);
        return copy_to_user(uarg, &m, sizeof(m)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_DESTROY_DUMB: {
        struct drm_mode_destroy_dumb d;
        if (!uarg || copy_from_user(&d, uarg, sizeof(d)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_dumb_t *bo = dumb_by_handle(sess, d.handle);
        if (bo) dumb_release_locked(bo);
        spinlock_unlock(&g_lock);
        return 0;
    }
    case DRM_NR_MODE_ADDFB: {
        struct drm_mode_fb_cmd f;
        if (!uarg || copy_from_user(&f, uarg, sizeof(f)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_fb_t *slot = NULL;
        for (int i = 0; i < DRM_MAX_FB; i++) if (!sess->fbs[i].used) { slot = &sess->fbs[i]; break; }
        if (!slot) { spinlock_unlock(&g_lock); return E_NOMEM; }
        slot->used = 1;
        slot->fb_id = sess->next_fb_id++;
        slot->handle = f.handle;
        slot->width = f.width; slot->height = f.height;
        slot->pitch = f.pitch ? f.pitch : f.width * 4u;
        f.fb_id = slot->fb_id;
        spinlock_unlock(&g_lock);
        return copy_to_user(uarg, &f, sizeof(f)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_ADDFB2: {
        struct drm_mode_fb_cmd2 f;
        if (!uarg || copy_from_user(&f, uarg, sizeof(f)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_fb_t *slot = NULL;
        for (int i = 0; i < DRM_MAX_FB; i++) if (!sess->fbs[i].used) { slot = &sess->fbs[i]; break; }
        if (!slot) { spinlock_unlock(&g_lock); return E_NOMEM; }
        slot->used = 1;
        slot->fb_id = sess->next_fb_id++;
        slot->handle = f.handles[0];
        slot->width = f.width; slot->height = f.height;
        slot->pitch = f.pitches[0] ? f.pitches[0] : f.width * 4u;
        f.fb_id = slot->fb_id;
        spinlock_unlock(&g_lock);
        return copy_to_user(uarg, &f, sizeof(f)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_RMFB: {
        uint32_t id = 0;
        if (uarg && copy_from_user(&id, uarg, sizeof(id)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_fb_t *fb = fb_by_id(sess, id);
        if (fb) memset(fb, 0, sizeof(*fb));
        spinlock_unlock(&g_lock);
        return 0;
    }
    case DRM_NR_MODE_GETFB: {
        struct drm_mode_fb_cmd f;
        if (!uarg || copy_from_user(&f, uarg, sizeof(f)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_fb_t *fb = fb_by_id(sess, f.fb_id);
        if (fb) {
            f.width = fb->width; f.height = fb->height;
            f.pitch = fb->pitch; f.bpp = 32; f.depth = 24; f.handle = fb->handle;
        }
        spinlock_unlock(&g_lock);
        return copy_to_user(uarg, &f, sizeof(f)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_PAGE_FLIP: {
        struct drm_mode_crtc_page_flip pf;
        if (!uarg || copy_from_user(&pf, uarg, sizeof(pf)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        sess->scanout_fb_id = pf.fb_id;
        drm_fb_t *fb = fb_by_id(sess, pf.fb_id);
        if (fb) blit_fb_to_display(sess, fb);
        if (pf.flags & DRM_MODE_PAGE_FLIP_EVENT)
            queue_flip_event(sess, pf.user_data, pf.crtc_id ? pf.crtc_id : DRM_CRTC_ID);
        spinlock_unlock(&g_lock);
        return 0;
    }
    case DRM_NR_MODE_DIRTYFB: {
        struct drm_mode_fb_dirty_cmd d;
        if (!uarg || copy_from_user(&d, uarg, sizeof(d)) != 0u) return E_FAULT;
        spinlock_lock(&g_lock);
        drm_fb_t *fb = fb_by_id(sess, d.fb_id);
        if (fb) blit_fb_to_display(sess, fb);
        spinlock_unlock(&g_lock);
        return 0;
    }
    case DRM_NR_MODE_GETPLANERES: {
        struct drm_mode_get_plane_res r;
        if (!uarg || copy_from_user(&r, uarg, sizeof(r)) != 0u) return E_FAULT;
        uint32_t pid = DRM_PLANE_ID;
        int64_t e = write_id_array(r.plane_id_ptr, r.count_planes, &pid, 1);
        if (e < 0) return e;
        r.count_planes = 1;
        return copy_to_user(uarg, &r, sizeof(r)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_GETPLANE: {
        struct drm_mode_get_plane p;
        if (!uarg || copy_from_user(&p, uarg, sizeof(p)) != 0u) return E_FAULT;
        p.plane_id = DRM_PLANE_ID;
        p.crtc_id = DRM_CRTC_ID;
        p.fb_id = sess->scanout_fb_id;
        p.possible_crtcs = 1u;
        p.gamma_size = 0;
        p.count_format_types = 0;
        return copy_to_user(uarg, &p, sizeof(p)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_OBJ_GETPROPS: {
        struct drm_mode_obj_get_properties o;
        if (!uarg || copy_from_user(&o, uarg, sizeof(o)) != 0u) return E_FAULT;
        /* Return the property-ID list for the requested object. */
        static const uint32_t crtc_props[] = { DRM_PROP_CRTC_ACTIVE,
                                               DRM_PROP_CRTC_MODE_ID };
        static const uint32_t conn_props[] = { DRM_PROP_CONN_CRTC_ID };
        static const uint32_t plane_props[] = {
            DRM_PROP_PLANE_TYPE,
            DRM_PROP_PLANE_FB_ID, DRM_PROP_PLANE_CRTC_ID,
            DRM_PROP_PLANE_SRC_X, DRM_PROP_PLANE_SRC_Y,
            DRM_PROP_PLANE_SRC_W, DRM_PROP_PLANE_SRC_H,
            DRM_PROP_PLANE_CRTC_X, DRM_PROP_PLANE_CRTC_Y,
            DRM_PROP_PLANE_CRTC_W, DRM_PROP_PLANE_CRTC_H };
        const uint32_t *plist = NULL;
        uint32_t pcount = 0;
        uint32_t obj_id = (uint32_t)o.obj_id;
        if (obj_id == DRM_CRTC_ID) {
            plist = crtc_props; pcount = 2;
        } else if (obj_id == DRM_CONNECTOR_ID) {
            plist = conn_props; pcount = 1;
        } else if (obj_id == DRM_PLANE_ID) {
            plist = plane_props; pcount = 11;
        }
        /* Write out property IDs if there is room, always report the count. */
        if (pcount && o.props_ptr && o.count_props >= pcount) {
            if (copy_to_user((void *)(uintptr_t)o.props_ptr, plist,
                             (uint64_t)pcount * sizeof(uint32_t)) != 0u)
                return E_FAULT;
        }
        o.count_props = pcount;
        return copy_to_user(uarg, &o, sizeof(o)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_GETPROPERTY: {
        /* Return property metadata by property-ID. The DDX looks up names to
         * map "ACTIVE" / "MODE_ID" / "FB_ID" / ... to numeric IDs.
         *
         * uapi struct drm_mode_get_property (x86-64, 64 bytes):
         *     u64 values_ptr;            u64 enum_blob_ptr;
         *     u32 prop_id;               u32 flags;
         *     char name[32];   <-- the name lives INLINE, there is no name_ptr
         *     u32 count_values;          u32 count_enum_blobs;
         * An empty name here silently breaks atomic modeset: the DDX finds no
         * prop_id, crtc_add_prop() returns -1, and the atomic commit is
         * skipped entirely (leaving "failed to set mode" with a stale errno).
         */
        struct {
            uint64_t values_ptr;
            uint64_t enum_blob_ptr;
            uint32_t prop_id;
            uint32_t flags;
            char     name[32];
            uint32_t count_values;
            uint32_t count_enum_blobs;
        } gp;
        if (!uarg || copy_from_user(&gp, uarg, sizeof(gp)) != 0u) return E_FAULT;
        const char *pname = NULL;
        uint32_t pflags = 0;
        switch (gp.prop_id) {
        case DRM_PROP_CRTC_ACTIVE:   pname = "ACTIVE";   pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_CRTC_MODE_ID:  pname = "MODE_ID";  pflags = DRM_MODE_PROP_BLOB;   break;
        case DRM_PROP_CONN_CRTC_ID:  pname = "CRTC_ID";  pflags = DRM_MODE_PROP_OBJECT; break;
        case DRM_PROP_PLANE_FB_ID:   pname = "FB_ID";    pflags = DRM_MODE_PROP_OBJECT; break;
        case DRM_PROP_PLANE_CRTC_ID: pname = "CRTC_ID";  pflags = DRM_MODE_PROP_OBJECT; break;
        case DRM_PROP_PLANE_SRC_X:   pname = "SRC_X";    pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_PLANE_SRC_Y:   pname = "SRC_Y";    pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_PLANE_SRC_W:   pname = "SRC_W";    pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_PLANE_SRC_H:   pname = "SRC_H";    pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_PLANE_CRTC_X:  pname = "CRTC_X";   pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_PLANE_CRTC_Y:  pname = "CRTC_Y";   pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_PLANE_CRTC_W:  pname = "CRTC_W";   pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_PLANE_CRTC_H:  pname = "CRTC_H";   pflags = DRM_MODE_PROP_RANGE;  break;
        case DRM_PROP_PLANE_TYPE:    pname = "type";     pflags = DRM_MODE_PROP_ENUM;   break;
        default: break;
        }
        /* Enum properties also carry their enum table (drm_mode_property_enum),
         * delivered through enum_blob_ptr on a second pass.  modesetting
         * matches the *names* below against its own table to learn that our
         * primary plane is DRMMODE_PLANE_TYPE_PRIMARY. */
        static const struct { uint64_t value; char name[32]; } plane_enums[3] = {
            { DRM_PLANE_TYPE_PRIMARY, "Primary" },
            { DRM_PLANE_TYPE_CURSOR,  "Cursor"  },
            { DRM_PLANE_TYPE_OVERLAY, "Overlay" },
        };
        uint32_t n_enum = (gp.prop_id == DRM_PROP_PLANE_TYPE) ? 3u : 0u;
        memset(gp.name, 0, sizeof(gp.name));
        if (pname) {
            uint32_t len = 0;
            while (pname[len] != '\0') len++;
            if (len >= (uint32_t)sizeof(gp.name))
                len = (uint32_t)sizeof(gp.name) - 1u;
            for (uint32_t i = 0; i < len; i++) gp.name[i] = pname[i];
            gp.flags = pflags;
        } else {
            gp.flags = 0;
            n_enum = 0;
        }
        if (n_enum && gp.enum_blob_ptr && gp.count_enum_blobs >= n_enum) {
            if (copy_to_user((void *)(uintptr_t)gp.enum_blob_ptr, plane_enums,
                             (uint64_t)n_enum * sizeof(plane_enums[0])) != 0u)
                return E_FAULT;
        }
        gp.count_values = 0;
        gp.count_enum_blobs = n_enum;
        return copy_to_user(uarg, &gp, sizeof(gp)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_ATOMIC: {
        /* Simplified atomic commit: parse the atomic request, find the
         * framebuffer ID from any ACTIVE CRTC property, and apply it like
         * a legacy SETCRTC. Real atomic would validate state transitions
         * across all objects; this handles the common single-CRTC case. */
        struct drm_mode_atomic a;
        if (!uarg || copy_from_user(&a, uarg, sizeof(a)) != 0u) return E_FAULT;
        if (a.count_objs == 0) return 0;

        uint32_t new_fb_id = 0;
        /* Walk objects: each has obj_id, then count_props entries of
         * (prop_id, value). We look for FB_ID (prop id 4) on CRTC (id 1)
         * or plane (id 1 simplified). */
        uint64_t objs_ptr = a.objs_ptr;
        uint64_t counts_ptr = a.count_props_ptr;
        uint64_t props_ptr = a.props_ptr;
        uint64_t values_ptr = a.prop_values_ptr;

        uint32_t prop_cursor = 0;
        for (uint32_t i = 0; i < a.count_objs; i++) {
            uint32_t obj_id = 0, prop_count = 0;
            if (copy_from_user(&obj_id, (void *)(uintptr_t)(objs_ptr +
                                (uint64_t)i * 4), 4) != 0u) return E_FAULT;
            if (copy_from_user(&prop_count, (void *)(uintptr_t)(counts_ptr +
                                (uint64_t)i * 4), 4) != 0u) return E_FAULT;

            for (uint32_t j = 0; j < prop_count; j++) {
                uint32_t prop_id = 0;
                uint64_t value = 0;
                uint64_t poff = (uint64_t)(prop_cursor + j) * 4;
                uint64_t voff = (uint64_t)(prop_cursor + j) * 8;
                if (copy_from_user(&prop_id, (void *)(uintptr_t)(props_ptr + poff), 4) != 0u)
                    return E_FAULT;
                if (copy_from_user(&value, (void *)(uintptr_t)(values_ptr + voff), 8) != 0u)
                    return E_FAULT;

                /* Match against the property IDs defined above. */
                if (prop_id == DRM_PROP_PLANE_FB_ID) {
                    new_fb_id = (uint32_t)value;
                } else if (prop_id == DRM_PROP_CRTC_ACTIVE && value == 0) {
                    spinlock_lock(&g_lock);
                    sess->scanout_fb_id = 0;
                    spinlock_unlock(&g_lock);
                }
            }
            prop_cursor += prop_count;
        }

        if (new_fb_id != 0) {
            spinlock_lock(&g_lock);
            sess->scanout_fb_id = new_fb_id;
            drm_fb_t *fb = fb_by_id(sess, new_fb_id);
            if (fb) blit_fb_to_display(sess, fb);
            spinlock_unlock(&g_lock);

            /* Queue flip event if requested. */
            if (a.flags & DRM_MODE_PAGE_FLIP_EVENT) {
                queue_flip_event(sess, a.user_data, DRM_CRTC_ID);
            }
        }
        return 0;
    }
    case DRM_NR_MODE_CREATEPROPBLOB: {
        struct { uint64_t data; uint32_t length, blob_id, pad; } cb;
        if (!uarg || copy_from_user(&cb, uarg, sizeof(cb)) != 0u) return E_FAULT;
        cb.blob_id = g_next_blob_id++;
        if (g_next_blob_id == 0u) g_next_blob_id = 1u;
        return copy_to_user(uarg, &cb, sizeof(cb)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_DESTROYPROPBLOB: {
        struct { uint64_t pad; uint32_t blob_id, pad2; } db;
        (void)db;
        (void)uarg;
        return 0;
    }
    case DRM_NR_MODE_GETPROPBLOB: {
        /* Return blob metadata: length=0, data=NULL (mode blob content is
         * carried in the MODE_ID property value in practice). */
        struct { uint64_t blob_id, length, pad; uint64_t data; } gb;
        if (!uarg || copy_from_user(&gb, uarg, sizeof(gb)) != 0u) return E_FAULT;
        gb.length = 0;
        gb.data = 0;
        return copy_to_user(uarg, &gb, sizeof(gb)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_CURSOR:
    case DRM_NR_MODE_CURSOR2:
        /* struct drm_mode_cursor[2] -- no hardware cursor, and the surface is
         * a plain framebuffer blit anyway, so the pointer is drawn in
         * software. Accept the call rather than make the DDX retry with the
         * older CURSOR (0xA3) and log a mode-setting failure. */
        return 0;
    case DRM_NR_MODE_LIST_LESSEES: {
        /* uint32 pad, uint32 count, uint64 *lessees -- 16 bytes. We create no
         * leases, so report none rather than ENOTTY. */
        struct { uint32_t pad, count; uint64_t lessees; } ll;
        if (!uarg || copy_from_user(&ll, uarg, sizeof(ll)) != 0u) return E_FAULT;
        ll.count = 0;
        ll.lessees = 0;
        return copy_to_user(uarg, &ll, sizeof(ll)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_GETGAMMA:
    case DRM_NR_MODE_SETGAMMA:
        return 0; /* no gamma LUT; benign no-op */
    case DRM_NR_MODE_SETPROPERTY:
    case DRM_NR_MODE_OBJ_SETPROP:
        return 0; /* legacy property set; accept as no-op */
    case DRM_NR_MODE_SETPLANE:
        return 0; /* single-plane config; no-op */
    case DRM_NR_GEM_CLOSE: {
        struct drm_gem_close g;
        if (uarg && copy_from_user(&g, uarg, sizeof(g)) == 0u) {
            spinlock_lock(&g_lock);
            drm_dumb_t *bo = dumb_by_handle(sess, g.handle);
            if (bo) dumb_release_locked(bo);
            spinlock_unlock(&g_lock);
        }
        return 0;
    }
    case DRM_NR_GEM_FLINK:
    case DRM_NR_GEM_OPEN:
        return E_INVAL; /* no flink name sharing */
    case DRM_NR_PRIME_HANDLE_TO_FD:
        return prime_handle_to_fd(sess->dumbs, uarg);
    case DRM_NR_PRIME_FD_TO_HANDLE:
        return prime_fd_to_handle(sess->dumbs, uarg, &sess->next_handle,
                                  &sess->next_map_off);
    default:
        serial_write_string("[drm] unhandled ioctl nr=");
        serial_write_uint64(nr);
        serial_write_string(" type=");
        serial_write_uint64(IOC_TYPE(request));
        serial_write_string("\n");
        return E_NOTTY;
    }
}

/* ---- read / poll ----------------------------------------------------- */
int64_t drm_kms_read(uint8_t *user_buf, uint64_t len, uint32_t nonblock)
{
    ensure_init();
    drm_session_t *sess = session_for_current();
    if (!sess) return E_NOMEM;
    ensure_init();
    (void)nonblock;
    spinlock_lock(&g_lock);
    if (sess->evq_tail == sess->evq_head) {
        spinlock_unlock(&g_lock);
        return E_AGAIN; /* caller (drmHandleEvent) polls first anyway */
    }
    uint64_t written = 0;
    while (sess->evq_tail != sess->evq_head) {
        struct drm_event_vblank *e = &sess->evq[sess->evq_tail];
        uint64_t need = e->base.length;
        if (written + need > len) break;
        if (copy_to_user(user_buf + written, e, need) != 0u) {
            spinlock_unlock(&g_lock);
            return written ? (int64_t)written : E_FAULT;
        }
        written += need;
        sess->evq_tail = (sess->evq_tail + 1u) % DRM_EVQ_MAX;
    }
    spinlock_unlock(&g_lock);
    return written ? (int64_t)written : E_AGAIN;
}

uint32_t drm_kms_poll(uint32_t events)
{
    ensure_init();
    drm_session_t *sess = session_for_current();
    if (!sess) return 0;
    ensure_init();
    uint32_t r = 0;
    spinlock_lock(&g_lock);
    if (sess->evq_tail != sess->evq_head) r |= 0x1u; /* POLLIN */
    spinlock_unlock(&g_lock);
    return r & (events | 0x1u);
}

/* ---- mmap ------------------------------------------------------------- */
int64_t drm_kms_mmap(uint64_t offset, uint64_t length, uint64_t prot,
                     uint64_t flags)
{
    ensure_init();
    drm_session_t *sess = session_for_current();
    if (!sess) return E_NOMEM;
    ensure_init();
    (void)prot; (void)flags;
    spinlock_lock(&g_lock);
    drm_dumb_t *bo = dumb_by_offset(sess, offset);
    if (!bo || !bo->kva) { spinlock_unlock(&g_lock); return E_INVAL; }
    uint64_t need = bo->size;
    uint64_t bo_phys = bo->phys;
    uint32_t np = bo->npages;
    spinlock_unlock(&g_lock);

    if (length > (uint64_t)np * 4096u) length = (uint64_t)np * 4096u;
    if (length == 0) length = need;

    uint64_t cr3 = process_get_current_cr3();
    if (!cr3) return E_NODEV;
    void *va = process_user_reserve((uint64_t)np * 4096u);
    if (!va) return E_NOMEM;
    uint64_t uva = (uint64_t)(uintptr_t)va;
    for (uint32_t i = 0; i < np; i++) {
        /* PAGE_EXTERNAL: these frames belong to the dumb buffer, not to this
         * address space.  Without it the client's munmap -- and, on every
         * exit, the sweep in paging_destroy_process_space() -- called
         * free_page() on frames the session still owns, and DESTROY_DUMB /
         * drm_kms_close() then freed them a second time.  That second free
         * put a still-in-use frame back on the PMM free list, so the next
         * two alloc_page() callers were handed the SAME frame and whichever
         * lost the race had its data overwritten: Xorg died on
         * "malloc(): corrupted top size" moments after Chromium exited.
         * The session is the single owner and frees them exactly once. */
        if (paging_map_user_page(cr3, uva + (uint64_t)i * 4096u,
                                 bo_phys + (uint64_t)i * 4096u,
                                 PAGE_PRESENT | PAGE_RW | PAGE_USER |
                                 PAGE_EXTERNAL) < 0) {
            (void)process_user_munmap(va, (uint64_t)np * 4096u);
            return E_NOMEM;
        }
    }
    return (int64_t)uva;
}

void drm_kms_close(void)
{
    ensure_init();
    drm_session_t *sess = session_for_current();
    if (!sess) return ;
    /* Xorg is the only DRM client; on its exit reclaim everything. */
    ensure_init();
    spinlock_lock(&g_lock);
    for (int i = 0; i < DRM_MAX_DUMB; i++) {
        /* Releases the handle's reference. A buffer that was exported keeps
         * its pages -- and its mapping -- alive until the dma-buf fd that
         * owns them closes too; only an unexported one is freed outright. */
        dumb_release_locked(&sess->dumbs[i]);
    }
    memset(sess->fbs, 0, sizeof(sess->fbs));
    sess->evq_head = sess->evq_tail = 0;
    sess->scanout_fb_id = 0;
    sess->mirror_released = 0u;
    spinlock_unlock(&g_lock);
}

/* ---- dma-buf descriptor ------------------------------------------------
 *
 * A PRIME descriptor is not a device node: no path, no minor, no open(2)
 * behind it, so DevFS does nothing but dispatch. Its identity is the prime
 * slot packed into the descriptor's driver_data, exactly the way a pty pair
 * carries its index, and these four are its entire surface. */
int64_t drm_prime_ioctl(int32_t slot, uint64_t request, uint64_t arg)
{
    ensure_init();
    (void)arg;
    if (slot < 0 || slot >= DRM_MAX_PRIME) return E_INVAL;
    /* 'b' is DMA_BUF_BASE; anything else is a caller bug. */
    if (IOC_TYPE(request) != 0x62u) return E_NOTTY;

    int used;
    spinlock_lock(&g_lock);
    used = g_primes[slot].used != 0u;
    spinlock_unlock(&g_lock);
    if (!used) return E_INVAL;

    switch (IOC_NR(request)) {
    case 0:  /* DMA_BUF_IOCTL_SYNC: these mappings are coherent, nothing to
              * flush; answering 0 is what keeps Mesa's explicit flush quiet. */
    case 1:  /* DMA_BUF_SET_NAME_A / DMA_BUF_SET_NAME_B: a debug label. */
        return 0;
    default: /* sync_file export/import (nr 2/3): no sync objects on these. */
        return -95;
    }
}

int64_t drm_prime_mmap(int32_t slot, uint64_t offset, uint64_t length,
                       uint64_t prot, uint64_t flags)
{
    ensure_init();
    (void)prot; (void)flags;
    if (slot < 0 || slot >= DRM_MAX_PRIME) return E_INVAL;
    if ((offset & 4095u) != 0u) return E_INVAL;

    spinlock_lock(&g_lock);
    drm_prime_t *p = &g_primes[slot];
    if (!p->used || p->kva == NULL || offset >= p->size) {
        spinlock_unlock(&g_lock);
        return E_INVAL;
    }
    uint64_t bo_phys = p->phys + offset;
    uint32_t np = (uint32_t)(((p->size - offset) + 4095u) / 4096u);
    spinlock_unlock(&g_lock);

    if (length > (uint64_t)np * 4096u) length = (uint64_t)np * 4096u;
    if (length == 0u) length = (uint64_t)np * 4096u;

    uint64_t cr3 = process_get_current_cr3();
    if (!cr3) return E_NODEV;
    void *va = process_user_reserve((uint64_t)np * 4096u);
    if (!va) return E_NOMEM;
    uint64_t uva = (uint64_t)(uintptr_t)va;
    for (uint32_t i = 0; i < np; i++) {
        /* PAGE_EXTERNAL, for the same reason as drm_kms_mmap(): the frames
         * belong to the prime block, which frees them once its last
         * reference goes -- not to the address space being torn down. */
        if (paging_map_user_page(cr3, uva + (uint64_t)i * 4096u,
                                 bo_phys + (uint64_t)i * 4096u,
                                 PAGE_PRESENT | PAGE_RW | PAGE_USER |
                                 PAGE_EXTERNAL) < 0) {
            (void)process_user_munmap(va, (uint64_t)np * 4096u);
            return E_NOMEM;
        }
    }
    return (int64_t)uva;
}

void drm_prime_close(int32_t slot)
{
    ensure_init();
    if (slot < 0 || slot >= DRM_MAX_PRIME) return;
    spinlock_lock(&g_lock);
    prime_put_locked(slot);
    spinlock_unlock(&g_lock);
}

uint32_t drm_prime_size(int32_t slot)
{
    ensure_init();
    if (slot < 0 || slot >= DRM_MAX_PRIME) return 0u;
    spinlock_lock(&g_lock);
    uint32_t n = g_primes[slot].used ? (uint32_t)g_primes[slot].size : 0u;
    spinlock_unlock(&g_lock);
    return n;
}
