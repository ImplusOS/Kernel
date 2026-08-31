#pragma once

/*
 * vfs_file_t alone, deliberately split out of vfs_types.h: vfs_types.h also
 * declares vnode_t/file_t, which need Core/sync/Mutex.h (and, through it,
 * WaitQueue.h/Spinlock.h) -- headers that assume a real kernel-linked
 * hal_cpu_* and are not safe to pull into a freestanding driver module's
 * translation unit (Kernel/Drivers/Module/DriverBinary.h includes this
 * header to describe driver_api_fs_t; it must not drag Spinlock.h's
 * `irq_save_disable`/`spinlock_lock` inline definitions in, since most
 * driver modules define their own local equivalents backed by
 * driver_binary_t's hal vtable instead).
 */

#include <stdint.h>

struct vfs_driver;

typedef struct vfs_file {
    uint64_t internal_id;
    uint32_t size;
    void *driver_data;
    const struct vfs_driver *fs_driver;
} vfs_file_t;
