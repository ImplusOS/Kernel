#pragma once

#include <stdint.h>
#include "Core/vfs/VFS.h"

void syscall_file_init(void);
int32_t syscall_file_open(const char *path, uint64_t flags);
int32_t syscall_file_creat(const char *path);

/* creat() the file, then open it with `flags` -- the create half of
 * open(path, ...|O_CREAT) on a path that does not exist yet. */
int32_t syscall_file_creat_ex(const char *path, uint64_t flags);

/* fcntl(F_ADD_SEALS) / fcntl(F_GET_SEALS) on a memfd. */
int32_t syscall_memfd_add_seals(int32_t fd, uint32_t seals);
int32_t syscall_memfd_get_seals(int32_t fd);
int64_t syscall_file_read(int32_t fd, uint8_t *buffer, uint64_t len);
int64_t syscall_file_write(int32_t fd, const uint8_t *buffer, uint64_t len);
int64_t syscall_file_seek(int32_t fd, int64_t offset, int32_t whence);
/* Positional read/write (pread/pwrite): the descriptor's offset is left
 * alone, and concurrent callers on one open file are serialised. */
int64_t syscall_file_pread(int32_t fd, uint8_t *buffer, uint64_t len,
                           uint64_t position);
int64_t syscall_file_pwrite(int32_t fd, const uint8_t *buffer, uint64_t len,
                            uint64_t position);
int32_t syscall_file_close(int32_t fd);
int32_t syscall_file_mkdir(const char *path);
int32_t syscall_file_opendir(const char *path);
int32_t syscall_file_readdir(int32_t dir_handle, vfs_dirent_t *out_entry);
int32_t syscall_file_closedir(int32_t dir_handle);
int32_t syscall_file_unlink(const char *path);
int32_t syscall_file_rename(const char *old_path, const char *new_path);
/* link(2) emulated as a content copy (pseudo-fs only). See Syscall_File.c. */
int32_t syscall_file_link(const char *old_path, const char *new_path);
void syscall_file_close_all_for_pid(int32_t pid, uint32_t *closed_fds_out, uint32_t *closed_dirs_out);
/* execve(2) path: close only the descriptors marked FD_CLOEXEC (O_CLOEXEC /
 * fcntl(F_SETFD)), leaving inherited fds (stdin/out/err, explicitly-passed
 * pipes and sockets) open across the exec. Directory handles are always
 * dropped (POSIX opendir() is implicitly close-on-exec). */
void syscall_file_close_cloexec_for_pid(int32_t pid);
int32_t syscall_file_pipe(int32_t fds_out[2]);
int32_t syscall_file_dup(int32_t oldfd);

/* A value identifying the file behind `fd` -- equal for every open of the
 * same file -- or 0 when it cannot be told (not a regular file, no backing
 * object). Used to find a file's memory mappings from a write to it. */
uint64_t syscall_file_identity(int32_t fd);

/* Called after every successful write to a regular file, with the offset the
 * write started at and the bytes written (`data` is whatever buffer the
 * caller passed in). One observer; NULL removes it. */
typedef void (*file_write_observer_t)(int32_t fd, uint32_t offset,
                                      const uint8_t *data, uint64_t len);
void syscall_file_set_write_observer(file_write_observer_t observer);
int32_t syscall_file_dup2(int32_t oldfd, int32_t newfd);
int32_t syscall_file_dup_at_least(int32_t oldfd, int32_t minimum_fd);

/* Reopen an existing descriptor under a new fd with `flags` as its access
 * mode -- the meaning of open() on /proc/self/fd/<n>. See the definition. */
int32_t syscall_file_reopen_fd(int32_t oldfd, uint64_t flags);
int32_t syscall_file_truncate(int32_t fd, uint64_t length);
/* True for either end of a pipe (see syscall_file_pipe()). The Linux compat
 * layer needs it to tell a pipe's EAGAIN -- which a blocking fd must never
 * see -- apart from a socket's. */
int syscall_file_is_pipe(int32_t fd);
/* True when `fd` is one end of a pseudo-terminal (/dev/ptmx or /dev/pts/N).
 * The Linux compat layer routes terminal ioctls on such an fd to the pty
 * instead of answering ENOTTY for everything. */
int32_t syscall_file_is_pty(int32_t fd);
/* Bitmask of which syscall_file_is_pty() checks passed; see the definition. */
uint32_t syscall_file_pty_debug(int32_t fd);
int32_t syscall_file_get_status_flags(int32_t fd);
int32_t syscall_file_set_status_flags(int32_t fd, uint32_t flags);
int32_t syscall_file_get_descriptor_flags(int32_t fd);
int32_t syscall_file_set_descriptor_flags(int32_t fd, uint32_t flags);
int64_t syscall_file_available(int32_t fd);
uint32_t syscall_file_poll(int32_t fd, uint32_t events);
int32_t syscall_file_register_dir(const char *path);
int32_t syscall_file_get_dir_dirent(int32_t fd, vfs_dirent_t *out_entry);
/* The path a directory fd was opened on, so the Linux compat layer can turn
 * openat(dirfd, "relative", ...) into an absolute path. 0 on success. */
int32_t syscall_file_get_dir_path(int32_t fd, char *out, uint32_t size);
int32_t syscall_file_get_file_info(int32_t fd, vfs_file_t *file_out,
                                   uint32_t *writable_out);
int32_t syscall_file_create_timerfd(void);
int32_t syscall_file_timerfd_settime(int32_t fd,
                                     uint64_t it_value_sec,
                                     uint64_t it_value_nsec,
                                     uint64_t it_interval_sec,
                                     uint64_t it_interval_nsec);
int32_t syscall_file_timerfd_gettime(int32_t fd,
                                     uint64_t *it_value_sec_out,
                                     uint64_t *it_value_nsec_out,
                                     uint64_t *it_interval_sec_out,
                                     uint64_t *it_interval_nsec_out);
int32_t syscall_file_create_memfd(const char *name);
/* Shared-memory handle backing an shm-promoted memfd, or -1. */
int32_t syscall_memfd_shm_handle(int32_t fd);
/* Install a memfd fd in the current process wrapping an existing shared
 * memory handle (adopts one reference). Returns fd or negative os_status_t. */
int32_t syscall_memfd_install_shm(int32_t handle, uint32_t status_flags);
/* Install a memfd fd wrapping a shared-memory object the caller already
 * owns, taking a reference of its own. This is the send-side counterpart of
 * syscall_memfd_shm_handle(): a native process that has no memfd_create()
 * needs a memfd to hand a shared buffer to a foreign client over SCM_RIGHTS
 * (the Wayland compositor's wl_keyboard.keymap). Returns fd or negative. */
int32_t syscall_memfd_from_shm(int32_t handle);
int32_t syscall_file_create_signalfd(uint64_t mask);
int32_t syscall_file_signalfd_set_mask(int32_t fd, uint64_t mask);
int64_t syscall_timerfd_read(int32_t fd, uint8_t *buffer, uint64_t len);
int64_t syscall_memfd_read(int32_t fd, uint8_t *buffer, uint64_t len);
int64_t syscall_memfd_write(int32_t fd, const uint8_t *buffer, uint64_t len);
int64_t syscall_memfd_pread(int32_t fd, uint8_t *buffer, uint64_t len,
                            uint64_t position);
int64_t syscall_memfd_pwrite(int32_t fd, const uint8_t *buffer, uint64_t len,
                             uint64_t position);
int64_t syscall_signalfd_read(int32_t fd, uint8_t *buffer, uint64_t len);

/* mmap(2) references on the open file description. A mapping must survive the
 * caller closing its fd, so demand-paged file mappings (Core/memory/FileMap.c)
 * hold one of these instead. `acquire` takes a reference from an fd,
 * `reacquire` takes another on a handle already held (fork), `read` fills a
 * KERNEL buffer from the mapped file, `release` drops one reference. */
int32_t syscall_file_mmap_acquire(int32_t fd);
/* Shared-memory handle backing a tmpfs file for mmap(MAP_SHARED), or <= 0
 * when `fd` is not such a file. See tmpfs_share_mapping(). */
int syscall_file_is_tmpfs(int32_t fd);
int32_t syscall_file_tmpfs_share(int32_t fd, uint64_t length);
int32_t syscall_file_mmap_reacquire(int32_t handle);
int64_t syscall_file_mmap_read(int32_t handle, uint64_t offset,
                               uint8_t *kernel_buffer, uint32_t length);
void    syscall_file_mmap_release(int32_t handle);

/* Returns 1 if `fd` is a directory handle (opendir/O_DIRECTORY open) owned by
 * the calling process. Used by the Linux compat fstat/statx path so opendir()
 * -- which fstat()s its own fd and bails unless S_ISDIR -- works for foreign
 * binaries (Xorg's module loader walks modules/ with opendir/readdir). */
int syscall_file_is_dir(int32_t fd);
/* Give another process a claim on a file-table descriptor (SCM_RIGHTS). */
int32_t syscall_file_grant(int32_t fd, int32_t pid);
/* Ascending walk of a process's open descriptors; -1 to start, -1 when done.
 * Backs /proc/<pid>/fd -- see the definition. */
int32_t syscall_file_next_open_fd(int32_t pid, int32_t after);

/* Character-device fds (devfs /dev/dri/card0, /dev/input/event*). Returns 1 if
 * `fd` is an open on a devfs node exposing the vfs_driver_t dev_* hooks. */
int32_t syscall_file_is_chardev(int32_t fd);
/* ioctl(2) on such an fd: Linux _IOC-encoded request. -25 (ENOTTY) if the
 * backing driver has no dev_ioctl. */
int64_t syscall_file_ioctl(int32_t fd, uint64_t request, uint64_t arg);
/* mmap(2) on such an fd: map device memory at file `offset` for `length`
 * bytes into the caller. Returns user VA or -errno. */
int64_t syscall_file_dev_mmap(int32_t fd, uint64_t offset, uint64_t length,
                              uint64_t prot, uint64_t flags);

/* fork(2): give `child_pid` the same descriptors as `parent_pid`. fd numbers
 * are global here, so the child is recorded as an additional owner of each
 * slot rather than getting copies; the backing object survives until the last
 * owner closes it. Without this a forked child cannot use any inherited fd --
 * dup2() on the pipe Popen() hands it fails with EFAULT. */
void syscall_file_fork_inherit(int32_t parent_pid, int32_t child_pid);
