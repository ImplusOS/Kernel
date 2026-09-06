#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "kernel/interfaces/vfs_types.h"

bool vfs_init(void);
/* Returns false if the mount table is full --
 * see Docs/Others/TODO_OS_Refactor.md 7.1/7.5. Existing callers that don't
 * check the boot-time filesystems they know are essential may still ignore
 * it, same as before this return value existed; new callers should check
 * it, particularly for any mount that isn't a fixed, small, known-at-boot
 * set (this is the concrete first step of the "adopt an error-reporting
 * return value in new/touched code" staged os_status_t migration -- a full
 * os_status_t is not warranted here since VFS.h's public API is otherwise
 * entirely bool-based already and mixing the two return conventions in one
 * header would be its own inconsistency). */
bool vfs_mount(const char *prefix, const vfs_driver_t *driver);
bool vfs_set_default_fs_for_file(const char *path,
                                 uint32_t min_size,
                                 const uint8_t *magic,
                                 uint32_t magic_size);
/* Selects the default (catch-all-prefix) filesystem by what kind of media
 * it is, rather than by driver name/fs_type -- see vfs_media_kind_t in
 * kernel/interfaces/vfs_types.h. This is the boot-time selection path
 * kernel_main.c should use; vfs_set_default_fs(name) below remains only
 * for callers that already have a specific driver name in hand (e.g. a
 * POSIX-style mount(2) by filesystem name). Returns false if no mounted
 * driver reports that media kind. */
bool vfs_set_default_fs_by_kind(vfs_media_kind_t kind);
bool vfs_find_file(const char *path, vfs_file_t *out_file);
bool vfs_read_file(vfs_file_t *file, uint8_t *buffer);
bool vfs_write_file(vfs_file_t *file, const uint8_t *buffer);
bool vfs_read_at(vfs_file_t *file, uint32_t offset, uint8_t *buffer, uint32_t size);
bool vfs_write_at(vfs_file_t *file, uint32_t offset, const uint8_t *buffer, uint32_t size);

/* Character-device hooks (see vfs_driver_t). vfs_file_is_chardev() is true iff
 * the backing driver exposes any of them. The *_ioctl/_mmap wrappers return
 * -ENOTTY (-25) when the driver has no such hook. */
bool     vfs_file_is_chardev(const vfs_file_t *file);
int64_t  vfs_dev_ioctl(vfs_file_t *file, uint64_t request, uint64_t arg);
int64_t  vfs_dev_read(vfs_file_t *file, uint8_t *buffer, uint64_t length,
                      uint32_t nonblock);
uint32_t vfs_dev_poll(vfs_file_t *file, uint32_t events);
int64_t  vfs_dev_mmap(vfs_file_t *file, uint64_t offset, uint64_t length,
                      uint64_t prot, uint64_t flags);
int64_t  vfs_dev_write(vfs_file_t *file, const uint8_t *buffer,
                       uint64_t length, uint32_t nonblock);
bool     vfs_file_has_dev_write(const vfs_file_t *file);
/* Second half of an open(2): run the driver's open_file hook, if it has one.
 * True (and a no-op) for every driver without one. Paired with
 * vfs_close_file() -- see the hook's comment in vfs_types.h. */
bool     vfs_open_file(vfs_file_t *file, uint64_t flags);
bool vfs_truncate(vfs_file_t *file, uint32_t new_size);
uint32_t vfs_get_file_size(vfs_file_t *file);
bool vfs_close_file(vfs_file_t *file);
void vfs_list_root(void);
bool vfs_creat(const char *path);
bool vfs_mkdir(const char *path);
int32_t vfs_opendir(const char *path);
/* Whether new files can be created under `path` (a directory). Resolves the
 * same way vfs_opendir() does -- the first driver that can open the directory
 * decides -- and reports writable iff that driver implements creat(). Needed
 * by access(dir, W_OK): callers use it to choose a scratch directory, so
 * answering "no" for every directory silently degrades them. */
bool vfs_dir_is_writable(const char *path);
int32_t vfs_readdir(int32_t handle, vfs_dirent_t *out_entry);
int32_t vfs_closedir(int32_t handle);
bool vfs_unlink(const char *path);
/* Symbolic links, on the filesystems that have them (tmpfs today: /tmp, /run,
 * /var, /dev/shm). vfs_symlink() returns false when the link already exists or
 * no mounted filesystem under `linkpath` can hold links; vfs_readlink() fills
 * `buf` WITHOUT a NUL and returns the byte count, or -1 when `path` is not a
 * symlink. */
bool vfs_symlink(const char *target, const char *linkpath);
int32_t vfs_readlink(const char *path, char *buf, uint32_t size);
/* POSIX permission bits, on the filesystems that keep them (tmpfs today).
 * vfs_get_mode() returns -1 when nothing stores a mode for `path`, and the
 * caller should then use its own default. */
bool vfs_set_mode(const char *path, uint32_t mode);
int32_t vfs_get_mode(const char *path);
bool vfs_rename(const char *old_path, const char *new_path);
void vfs_set_case_sensitive(bool enabled);
bool vfs_get_case_sensitive(void);
bool vfs_set_default_fs(const char *fs_type);
int vfs_driver_count_get(void);
