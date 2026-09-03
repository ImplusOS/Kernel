#pragma once
#include <stdint.h>

/*
 * DRM_Kms — minimal Linux DRM/KMS emulation for an unmodified Xorg
 * "modesetting" DDX + Mesa software GL (llvmpipe / kms_swrast), exposed at
 * /dev/dri/card0. Unlike the older native-ABI DRM_Client.c this speaks the
 * real Linux ioctl ABI: _IOC-encoded request numbers and <drm/drm_mode.h>
 * struct layouts. One CRTC / one connector / one encoder, single dumb-buffer
 * scanout that is blitted to display_get_framebuffer() on PAGE_FLIP/DIRTYFB.
 * See Docs/Others/TODO_Doom_Xorg_MethodA.md M3.
 *
 * Entry points are called from the devfs /dev/dri/card0 node (Kernel/Core/vfs/
 * DevFS.c) via the vfs_driver_t dev_* hooks.
 */

void    drm_kms_init(void);

/* Linux _IOC-encoded request. Returns 0 / >=0 on success, -errno on failure. */
int64_t drm_kms_ioctl(uint64_t request, uint64_t arg);

/* read(2) on the DRM fd: drains completed page-flip events as struct
 * drm_event_vblank records. nonblock != 0 -> return -11 (EAGAIN) when empty. */
int64_t drm_kms_read(uint8_t *user_buf, uint64_t len, uint32_t nonblock);

/* poll(2): returns POLLIN (0x1) when a flip-complete event is queued. */
uint32_t drm_kms_poll(uint32_t events);

/* mmap(2) on the DRM fd at the offset a prior MODE_MAP_DUMB handed back:
 * maps the dumb buffer's pages into the current process. Returns user VA or
 * -errno. */
int64_t drm_kms_mmap(uint64_t offset, uint64_t length, uint64_t prot,
                     uint64_t flags);

void    drm_kms_close(void);

/* ---- Scanout redirection ("mirror") -------------------------------------
 *
 * By default a flip is blitted straight to display_get_framebuffer(), i.e.
 * Xorg owns the panel. That cannot coexist with the ImplusOS window manager,
 * which owns the same framebuffer: whoever presented last wins and the screen
 * tears between two unrelated desktops.
 *
 * A native client (the Doom launcher) instead creates an ordinary WM window,
 * hands the pixels of its backing store here, and X scans out into that
 * surface; the compositor then draws it like any other window. `pixels` is a
 * user VA in the CALLING process -- its physical pages are resolved once and
 * pinned by the shared-memory object behind them, so the blit works from any
 * process's context (Xorg's, when its ioctl runs).
 *
 * Passing pixels == NULL clears the redirection and restores direct scanout.
 * Returns 0 on success, -errno otherwise. */
int drm_kms_set_mirror(uint64_t pixels, uint32_t width, uint32_t height);

/* Non-zero once at least one flip/dirty has been blitted into the mirror, so
 * the client knows there is something to damage. Clears the flag. */
int drm_kms_mirror_take_dirty(void);

/* Called when a process goes away: drops the redirection if that process owned
 * it, so scanout returns to the panel instead of writing into freed pages. */
void drm_kms_notify_process_exit(int32_t pid);
