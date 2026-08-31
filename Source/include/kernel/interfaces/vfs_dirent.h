#pragma once

/*
 * vfs_dirent_t and vfs_media_kind_t, deliberately split out of vfs_types.h
 * for the same reason vfs_file.h was: vfs_types.h also declares
 * vnode_t/file_t, which pull in Core/sync/Mutex.h (and, through it,
 * WaitQueue.h/Spinlock.h) -- headers that are not safe to drag into a
 * freestanding driver module's translation unit. A filesystem driver
 * module describes its directory entries and media kind through
 * kernel/interfaces/fs_module_ops.h, which includes only this header and
 * vfs_file.h. vfs_types.h re-includes this file, so kernel-side code that
 * has always got these types from vfs_types.h is unaffected.
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct vfs_dirent {
    char name[256];
    bool is_directory;
    uint32_t size;
} vfs_dirent_t;

/*
 * vfs_media_kind_t -- what a mounted filesystem is backed by, in terms
 * generic enough that VFS.c never needs to know a specific filesystem's
 * name or driver identity to make a boot-time policy decision (e.g. "prefer
 * the optical-media filesystem as the default root when one is mounted" --
 * see kernel_main.c's all_fs_initialize() and vfs_set_default_fs_by_kind()).
 * Every filesystem fills this in when it registers with vfs_mount(); it is
 * orthogonal to `fs_type`, which stays around only as a human-readable
 * label and for name-based lookups a caller explicitly asks for
 * (vfs_set_default_fs()), not for VFS-internal policy.
 * See Docs/Others/TODO_OS_Refactor.md 6.1 for the background.
 */
typedef enum {
    VFS_MEDIA_KIND_UNKNOWN = 0,
    VFS_MEDIA_KIND_OPTICAL,  /* read-only optical media (ISO9660, ...) */
    VFS_MEDIA_KIND_DISK,     /* writable disk/removable media (FAT32, exFAT, ...) */
    VFS_MEDIA_KIND_PSEUDO,   /* in-memory pseudo filesystems (devfs, tmpfs, procfs, etcfs) */
} vfs_media_kind_t;
