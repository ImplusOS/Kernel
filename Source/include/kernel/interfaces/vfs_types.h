#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "Core/sync/Mutex.h"
#include "kernel/interfaces/vfs_file.h"
#include "kernel/interfaces/vfs_dirent.h"

struct vfs_driver;

typedef struct vnode {
    uint64_t inode_num;
    uint32_t size;
    uint32_t type;
    const struct vfs_driver *fs;
    void *fs_priv;
    uint32_t refcount;
    mutex_t lock;
} vnode_t;

typedef struct file {
    vnode_t *vnode;
    uint64_t offset;
    uint32_t mode;
    uint32_t refcount;
    vfs_file_t legacy_file;
    mutex_t lock;
} file_t;

/* vfs_dirent_t and vfs_media_kind_t now live in
 * kernel/interfaces/vfs_dirent.h (included above) so a filesystem driver
 * module can see them without dragging in vnode_t/file_t's Mutex.h. */

typedef struct vfs_driver {
    const char *fs_type;
    const char *prefix;
    vfs_media_kind_t media_kind;
    bool (*find_file)(const char *path, vfs_file_t *out_file);
    bool (*read_file)(vfs_file_t *file, uint8_t *buffer);
    bool (*write_file)(vfs_file_t *file, const uint8_t *buffer);
    bool (*read_at)(vfs_file_t *file, uint32_t offset, uint8_t *buffer, uint32_t size);
    bool (*write_at)(vfs_file_t *file, uint32_t offset, const uint8_t *buffer, uint32_t size);
    bool (*truncate)(vfs_file_t *file, uint32_t new_size);
    uint32_t (*get_file_size)(vfs_file_t *file);
    bool (*creat)(const char *path);
    bool (*mkdir)(const char *path);
    int32_t (*opendir)(const char *path);
    int32_t (*readdir)(int32_t handle, vfs_dirent_t *out_entry);
    int32_t (*closedir)(int32_t handle);
    bool (*close_file)(vfs_file_t *file);
    bool (*unlink)(const char *path);
    void (*list_root)(void);
    void (*set_case_sensitive)(bool enabled);
    bool (*get_case_sensitive)(void);

    /* Optional character-device hooks (devfs only, all may be NULL). A driver
     * that sets any of these is telling the syscall file layer that fds opened
     * on its nodes are character devices: read()/poll() prefer dev_read/dev_poll
     * over read_at, and ioctl()/mmap() are routed to dev_ioctl/dev_mmap instead
     * of failing. Used by /dev/dri/card0 (DRM/KMS) and /dev/input/event* (evdev)
     * for the foreign-X-server path -- see TODO_Doom_Xorg_MethodA.md M2/M3.
     *   dev_ioctl : Linux _IOC-encoded request; returns >=0 or -errno.
     *   dev_read  : like read(2); returns byte count or -errno (-11 = EAGAIN).
     *   dev_poll  : returns POLLIN(0x1)/POLLOUT(0x4)/... bits currently ready.
     *   dev_mmap  : map device memory for `length` bytes at file `offset` into
     *               the caller's address space; returns user VA or -errno. */
    int64_t (*dev_ioctl)(vfs_file_t *file, uint64_t request, uint64_t arg);
    int64_t (*dev_read)(vfs_file_t *file, uint8_t *buffer, uint64_t length,
                        uint32_t nonblock);
    uint32_t (*dev_poll)(vfs_file_t *file, uint32_t events);
    int64_t (*dev_mmap)(vfs_file_t *file, uint64_t offset, uint64_t length,
                        uint64_t prot, uint64_t flags);
} vfs_driver_t;
