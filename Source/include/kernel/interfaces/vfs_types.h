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
    /*   dev_write : like write(2); returns byte count or -errno. Unlike
     *               dev_read's user pointer, `buffer` is KERNEL memory --
     *               syscall_file_write() stages the user bytes first, the
     *               same contract write_at has. A device that can accept a
     *               short write (a pty whose output ring is full) needs this
     *               instead of write_at, which is all-or-nothing. */
    int64_t (*dev_write)(vfs_file_t *file, const uint8_t *buffer,
                         uint64_t length, uint32_t nonblock);

    /* Optional. Called by the fd layer once find_file() has been accepted for
     * a real open(2), and paired with close_file(). Filesystems whose nodes
     * are stateless do not need it; /dev/ptmx does, because "look this path
     * up" (stat, access, a path-exists probe -- none of which close what they
     * find) must not be what allocates a pseudo-terminal.
     *   flags : the open(2) flags, so a device can honour O_NOCTTY.
     *   false : refuse the open; close_file() is NOT called. */
    bool (*open_file)(vfs_file_t *file, uint64_t flags);

    /* Optional symbolic links (both NULL for a filesystem without them, which
     * is how the read-only boot media and the generated pseudo-filesystems
     * answer). Only tmpfs implements these today, which is enough for the
     * writable trees (/tmp, /run, /var) real Linux programs put links in --
     * Chromium's ProcessSingleton refuses to start a browser when it cannot
     * create <user-data-dir>/SingletonLock.
     *   symlink  : create `linkpath` pointing at `target`; false if it exists
     *              or the path is not ours.
     *   readlink : copy the target into `buf` (NOT NUL-terminated, same as
     *              readlink(2)); returns the byte count, or -1 when `path` is
     *              not a symlink on this filesystem. */
    bool (*symlink)(const char *target, const char *linkpath);
    int32_t (*readlink)(const char *path, char *buf, uint32_t size);

    /* Optional POSIX permission bits (both NULL for a filesystem that has no
     * per-node mode, which is every read-only and generated one here). The
     * VFS mkdir/creat hooks take no mode, so the syscall layer applies the
     * caller's mode with set_mode() right after creating the node.
     *   set_mode : store `mode` (the low 12 bits) for an existing node.
     *   get_mode : the stored bits, or <0 when this filesystem has no node
     *              there -- the caller then falls back to its default.
     * Chromium needs this: mkdtemp() creates its ProcessSingleton socket
     * directory 0700 and then CHECK()s that stat() reports exactly 0700. */
    bool (*set_mode)(const char *path, uint32_t mode);
    int32_t (*get_mode)(const char *path);
} vfs_driver_t;
