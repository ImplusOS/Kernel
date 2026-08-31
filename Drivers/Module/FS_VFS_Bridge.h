#pragma once

/*
 * FS_VFS_Bridge -- the single kernel-resident glue between the loadable
 * filesystem driver modules and Core/vfs/VFS.c, replacing the three
 * near-identical FAT32_VFS_Bridge.c / exFAT_VFS_Bridge.c /
 * ISO9660_VFS_Bridge.c files. Each module exports one common
 * fs_module_ops_t (see kernel/interfaces/fs_module_ops.h) instead of its
 * own bespoke vtable, and this file is the only place that knows how to
 * turn that into the vfs_driver_t vfs_mount() wants -- allocating the
 * opaque per-open file handle, converting write results back into
 * vfs_file_t.size, and lazily (re-)resolving + init'ing the module.
 *
 * The kernel no longer names any filesystem. fs_bridge_mount_all()
 * discovers every driver that registered itself as DEVICE_TYPE_FILESYSTEM
 * in DeviceRegistry (whatever its module filename), wraps each in a
 * vfs_driver_t, mounts them at the catch-all prefix, and picks the default
 * root filesystem purely from vfs_media_kind_t (optical media wins when
 * present; otherwise a writable disk filesystem is preferred over a
 * read-only one). There is no compile-time list of filesystem names or
 * ids anywhere in the kernel.
 *
 * Adding a filesystem: give its module an fs_module_ops_t and have it
 * register as DEVICE_TYPE_FILESYSTEM. Nothing in the kernel changes.
 */

#include <stdbool.h>

/*
 * Discover, initialise and mount every DEVICE_TYPE_FILESYSTEM driver known
 * to DeviceRegistry, then select the default root filesystem by media
 * kind. Idempotent: already-mounted filesystems are skipped, so it is safe
 * to call again after more filesystem modules have been loaded. Returns
 * true once at least one filesystem is mounted.
 */
bool fs_bridge_mount_all(void);
