#pragma once
#include <stdint.h>

/*
 * DRM_Kms — Linux DRM/KMS emulation for Xorg's "modesetting" DDX and a
 * separate VirtIO-GPU virgl render node. card0 exposes one CRTC / connector /
 * encoder and dumb-buffer scanout blitted to display_get_framebuffer().
 * renderD128 exposes the Linux VirtIO-GPU UAPI subset used by Mesa's virgl
 * Gallium winsys: context init, capsets, resources, execbuffer, transfers,
 * BO mapping/WAIT, and PRIME sharing. It intentionally does not claim blob,
 * syncobj, or sync_file fence-FD support.
 * See Docs/Others/TODO_Doom_Xorg_MethodA.md M3.
 *
 * Entry points are called from the devfs /dev/dri/card0 node (Kernel/Core/vfs/
 * DevFS.c) via the vfs_driver_t dev_* hooks.
 *
 * Two device nodes are exposed:
 *   /dev/dri/card0      — full KMS: modeset, scanout, atomic, prime
 *   /dev/dri/renderD128 — render-only: VirtIO virgl GEM resources, exec,
 *                         transfer, mmap, WAIT, and PRIME. No modesetting,
 *                         scanout, syncobj, or sync_file fence-FD support.
 *   Mesa opens renderD128 for GL contexts; Xorg opens card0 for display.
 */

void    drm_kms_init(void);

/* ---- card0 (full KMS) ------------------------------------------------- */
/* Linux _IOC-encoded request. Returns 0 / >=0 on success, -errno on failure. */
int64_t drm_kms_ioctl(int32_t slot, uint64_t request, uint64_t arg);

/* read(2) on the DRM fd: drains completed page-flip events as struct
 * drm_event_vblank records. nonblock != 0 -> return -11 (EAGAIN) when empty. */
int64_t drm_kms_read(uint8_t *user_buf, uint64_t len, uint32_t nonblock);

/* poll(2): returns POLLIN (0x1) when a flip-complete event is queued. */
uint32_t drm_kms_poll(uint32_t events);

/* mmap(2) on the DRM fd at the offset a prior MODE_MAP_DUMB handed back:
 * maps the dumb buffer's pages into the current process. Returns user VA or
 * -errno. */
int64_t drm_kms_mmap(int32_t slot, uint64_t offset, uint64_t length,
                     uint64_t prot, uint64_t flags);

void    drm_kms_close(void);

/* ---- renderD128 (render-only) ----------------------------------------- */
/* Separate namespace: GEM buffers, VirtIO virgl resources, exec/transfer,
 * and PRIME descriptors. Rejects modeset ioctls. Unsupported blob, syncobj,
 * and sync_file fence-FD paths return errors rather than being advertised. */
int64_t drm_render_ioctl(int32_t slot, uint64_t request, uint64_t arg);

/* read(2) on render fd: no events on render node, always EAGAIN. */
int64_t drm_render_read(uint8_t *user_buf, uint64_t len, uint32_t nonblock);

/* poll(2) on render fd: no event sources. */
uint32_t drm_render_poll(uint32_t events);

/* mmap(2) on render fd: maps dumb buffer pages (same as card0 path). */
int64_t drm_render_mmap(int32_t slot, uint64_t offset, uint64_t length,
                        uint64_t prot, uint64_t flags);

/* open(2) of renderD128 or card0: gives the new open file its own render
 * session -- GEM handle namespace plus host GL context, Linux's drm_file --
 * and returns its slot, which the file keeps and passes to every call below
 * (ioctl/mmap), or -1 when the table is full. `slot` is that value. */
int32_t drm_render_open(void);

/* Last close of that open file (whichever process it ends up in): frees
 * the session's buffers and its GPU context. */
void    drm_render_close(int32_t slot);

/* ---- PRIME dma-buf descriptors -----------------------------------------
 *
 * drmPrimeHandleToFD() does not open a node -- there is no path behind the
 * descriptor it hands back -- so DevFS cannot build the vfs_file_t and the
 * fd layer cannot open one either. DevFS makes the descriptor with
 * syscall_file_install_dev() and then routes its three operations here,
 * identifying the buffer by the prime slot packed into the descriptor's
 * driver_data (the same carrier a pty pair uses for its index).
 *
 * The buffer itself is refcounted from the moment it is first exported: the
 * GEM handle holds one reference, every descriptor another, and importing it
 * back into a session one more. The last one out frees the pages. That is
 * what lets a dma-buf fd outlive both DESTROY_DUMB and the close of
 * /dev/dri/card0, which is exactly the order Mesa writes them in before
 * passing the fd to the X server over DRI3. */
int64_t  drm_prime_ioctl(int32_t slot, uint64_t request, uint64_t arg);
int64_t  drm_prime_mmap(int32_t slot, uint64_t offset, uint64_t length,
                        uint64_t prot, uint64_t flags);
void     drm_prime_close(int32_t slot);
uint32_t drm_prime_size(int32_t slot);

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
