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
#define DRM_NR_GET_MAGIC          0x02
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
#define DRM_NR_MODE_ATOMIC        0xBC
#define DRM_NR_MODE_CREATEPROPBLOB  0xBD
#define DRM_NR_MODE_DESTROYPROPBLOB 0xBE

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

#define DRM_CONNECTOR_ID  1u
#define DRM_ENCODER_ID    1u
#define DRM_CRTC_ID       1u
#define DRM_PLANE_ID      1u
#define DRM_MODE_CONNECTED 1u

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

typedef struct {
    uint8_t  used;
    uint32_t handle;
    uint32_t width, height, bpp, pitch;
    uint64_t size;
    uint32_t npages;
    void    *kva;        /* kernel virtual (pmm_alloc_pages) */
    uint64_t phys;       /* physical base */
    uint64_t map_offset; /* token returned by MAP_DUMB */
} drm_dumb_t;

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

static spinlock_t g_lock;
static int g_inited;

void drm_kms_init(void)
{
    spinlock_init(&g_lock);
    memset(g_sessions, 0, sizeof(g_sessions));
    for (int i = 0; i < DRM_MAX_SESSIONS; i++) {
        g_sessions[i].owner_pid = -1;
        g_sessions[i].client_pid = -1;
    }
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
        if (s->dumbs[i].used && s->dumbs[i].kva)
            pmm_free_pages(s->dumbs[i].kva, s->dumbs[i].npages);
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
    uint32_t src_pitch = fb->pitch ? fb->pitch : (bo->pitch ? bo->pitch : fb->width * 4u);

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
        static const char nm[] = "implusdrm";
        static const char dt[] = "20260829";
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
        default:                             c.value = 0; break;
        }
        return copy_to_user(uarg, &c, sizeof(c)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_SET_CLIENT_CAP:
        /* Refuse every client cap so the DDX stays on the fully legacy
         * modeset path. Saying yes to UNIVERSAL_PLANES would make modesetting
         * drive real planes through GETPLANERESOURCES/SETPLANE, and yes to
         * ATOMIC would make it build atomic commits -- neither of which this
         * shim implements. Failing here is a normal, handled case for the DDX
         * (it is what a pre-3.15 kernel does). */
        return -95; /* -EOPNOTSUPP */
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
        if (!d.width || !d.height || (d.bpp != 32 && d.bpp != 24 && d.bpp != 16))
            return E_INVAL;
        uint32_t bpp = d.bpp < 24 ? d.bpp : 32;
        uint32_t pitch = d.width * (bpp / 8u);
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
        if (bo) {
            if (bo->kva) pmm_free_pages(bo->kva, bo->npages);
            memset(bo, 0, sizeof(*bo));
        }
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
        o.count_props = 0;
        return copy_to_user(uarg, &o, sizeof(o)) == 0u ? 0 : E_FAULT;
    }
    case DRM_NR_MODE_GETPROPERTY:
    case DRM_NR_MODE_GETPROPBLOB:
    case DRM_NR_MODE_OBJ_SETPROP:
    case DRM_NR_MODE_SETPROPERTY:
    case DRM_NR_MODE_GETGAMMA:
    case DRM_NR_MODE_SETGAMMA:
    case DRM_NR_MODE_CURSOR:
    case DRM_NR_MODE_SETPLANE:
    case DRM_NR_MODE_CREATEPROPBLOB:
    case DRM_NR_MODE_DESTROYPROPBLOB:
        return 0; /* benign no-op for the legacy modeset path */
    case DRM_NR_MODE_ATOMIC:
        return -95; /* -EOPNOTSUPP: force the DDX onto the legacy path */
    case DRM_NR_GEM_CLOSE: {
        struct drm_gem_close g;
        if (uarg && copy_from_user(&g, uarg, sizeof(g)) == 0u) {
            spinlock_lock(&g_lock);
            drm_dumb_t *bo = dumb_by_handle(sess, g.handle);
            if (bo) {
                if (bo->kva) pmm_free_pages(bo->kva, bo->npages);
                memset(bo, 0, sizeof(*bo));
            }
            spinlock_unlock(&g_lock);
        }
        return 0;
    }
    case DRM_NR_GEM_FLINK:
    case DRM_NR_GEM_OPEN:
        return E_INVAL; /* no PRIME / flink sharing */
    default:
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
        if (paging_map_user_page(cr3, uva + (uint64_t)i * 4096u,
                                 bo_phys + (uint64_t)i * 4096u,
                                 PAGE_PRESENT | PAGE_RW | PAGE_USER) < 0) {
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
        if (sess->dumbs[i].used && sess->dumbs[i].kva)
            pmm_free_pages(sess->dumbs[i].kva, sess->dumbs[i].npages);
        memset(&sess->dumbs[i], 0, sizeof(sess->dumbs[i]));
    }
    memset(sess->fbs, 0, sizeof(sess->fbs));
    sess->evq_head = sess->evq_tail = 0;
    sess->scanout_fb_id = 0;
    sess->mirror_released = 0u;
    spinlock_unlock(&g_lock);
}
