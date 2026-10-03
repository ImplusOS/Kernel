#pragma once

/*
 * DevFS - minimal /dev pseudo-filesystem.
 *
 * Provides the handful of character device nodes that a Linux-ABI
 * userland (glibc, Chromium) expects to be able to open() directly:
 * /dev/null, /dev/zero, /dev/full, /dev/urandom, /dev/random, /dev/tty.
 *
 * Mounted at prefix "/dev" via vfs_mount() so ordinary open()/read()/
 * write()/stat() go through the regular VFS path. See TODO_Chromium_LinuxABI.md
 * section 3.3.
 */

#include "kernel/interfaces/vfs_types.h"

void devfs_init(void);
const vfs_driver_t *devfs_vfs_get_driver(void);

/* A descriptor in the current process for the dma-buf in PRIME slot `slot`,
 * of `size` bytes -- the fd half of drmPrimeHandleToFD(). DevFS builds the
 * vfs_file_t (there is no path to look up) and the fd layer slots it in;
 * DevFS's close/ioctl/mmap/poll hooks then dispatch on the slot packed into
 * its driver_data. Returns the fd, or a negative os_status_t. The caller must
 * already hold the reference this descriptor owns. */
int32_t devfs_prime_fd(int32_t slot, uint32_t size);

/* The PRIME slot a vfs_file_t names, or -1 if it is not a dma-buf. */
int32_t devfs_prime_slot(const vfs_file_t *file);

/* True if `path` names one of the fixed /dev character-device nodes
 * (used by stat()/fstat() in the Linux ABI layer to report S_IFCHR
 * instead of the generic S_IFREG mode). */
bool devfs_path_is_device(const char *path);

/* The Linux DRM major and the two minors Linux hands out for it. stat()
 * reports them in st_rdev (see below) and SysFS builds
 * /sys/dev/char/<maj>:<min> out of the very same numbers -- libdrm joins the
 * two sides by formatting them back into a path, so they live next to the
 * nodes they name and cannot drift apart. The minors are fixed by
 * drmGetMinorType(), which stats "/dev/dri/card<minor>" and
 * "/dev/dri/renderD<minor>" to work out which kind of node it is looking at. */
#define DEVFS_DRM_MAJOR        226u
#define DEVFS_DRM_CARD_MINOR   0u
#define DEVFS_DRM_RENDER_MINOR 128u

/* Linux INPUT major (13) and evdev minors (64 + N), matching Linux's
 * EVDEV_MINOR_BASE=64, EVDEV_MINORS=32. Distinct minors per event node so
 * evdev's duplicate check (fstat -> st_rdev compare) works. */
#define DEVFS_INPUT_MAJOR       13u
#define DEVFS_INPUT_EVENT0_MINOR 64u
#define DEVFS_INPUT_EVENT1_MINOR 65u

/* Linux dev_t (glibc's makedev() encoding) that stat() should report in
 * st_rdev for `path` / `file`. The DRM nodes carry the real Linux DRM major
 * (226) plus the minors Linux assigns them -- card0 -> 0, renderD128 -> 128 --
 * because libdrm rebuilds both the sysfs path (/sys/dev/char/<maj>:<min>) and
 * the node name (/dev/dri/card<minor>, /dev/dri/renderD<minor>) from st_rdev:
 * a placeholder value makes drmGetDevice2() look up "1:5" and fail, which is
 * what kept Mesa's glamor on "failed to get compatible render device".
 * Every other node keeps the historical 1:5 placeholder. The path and fd forms
 * must agree -- drm_device_has_rdev() stats the node path and compares it
 * against the fstat() of the fd it was handed. */
uint64_t devfs_path_rdev(const char *path);
uint64_t devfs_file_rdev(const vfs_file_t *file);

/* True if `file` is one end of a pseudo-terminal (/dev/ptmx or /dev/pts/N).
 * The fd layer needs it to route terminal ioctls (see syscall_file_is_pty). */
bool devfs_file_is_pty(const vfs_file_t *file);
