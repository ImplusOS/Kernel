#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "Syscall_File.h"

#include "kernel/status.h"
#include "Core/vfs/VFS.h"
#include "kernel/config.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "Core/memory/SharedMemory.h"
#include "Debug/serial/Serial.h"

enum {
    FILE_MAX_FD = FILE_MAX_FD_CONFIG,
    FILE_MAX_DIR_HANDLE = FILE_MAX_DIR_HANDLE_CONFIG
};

#define FILE_IO_CHUNK_SIZE   (128U * 1024U)
#define FILE_READ_CACHE_SIZE 4096U
#define FILE_READ_DIRECT_THRESHOLD FILE_READ_CACHE_SIZE
#define FILE_SEEK_SET        0
#define FILE_SEEK_CUR        1
#define FILE_SEEK_END        2
#define FILE_O_ACCMODE       0x0003u
#define FILE_O_WRONLY        0x0001u
#define FILE_O_RDWR          0x0002u
#define FILE_O_TRUNC         0x0200u
#define FILE_O_APPEND        0x0400u
#define FILE_O_NONBLOCK      0x0800u
#define FILE_FD_CLOEXEC      0x0001u

#define PIPE_MAX_COUNT       16
#define PIPE_BUF_SIZE        4096u

/* Extra owners of a descriptor slot, beyond `owner_pid`.
 *
 * fd numbers are global in this table, so a forked child cannot be given its
 * own copy at the same number -- yet POSIX requires it to see exactly the same
 * fds. Instead the slot records every process that holds it: fork sets the
 * child's bit, close/exit clears one bit, and the backing object is torn down
 * only when the last owner lets go. Parent and child therefore share the open
 * file description (offset included), which is what POSIX specifies anyway.
 *
 * The bitmap is empty for every descriptor that was never inherited, so the
 * behaviour of everything that does not fork is bit-for-bit unchanged. */
#define FD_OWNER_WORDS ((OS_CONFIG_PROCESS_MAX_COUNT + 63) / 64)

typedef struct {
    uint8_t used;
    int32_t owner_pid;
    int32_t open_index;
    uint32_t status_flags;
    uint32_t descriptor_flags;
    uint64_t extra_owners[FD_OWNER_WORDS];
} kernel_file_t;

typedef struct {
    uint8_t used;
    uint8_t writable;
    vfs_file_t file;
    uint32_t offset;
    uint32_t refcount;
    uint8_t cache_valid;
    uint32_t cache_offset;
    uint32_t cache_size;
    uint8_t cache_data[FILE_READ_CACHE_SIZE];
} kernel_open_file_t;

typedef struct {
    uint8_t used;
    int32_t owner_pid;
    int32_t vfs_handle;
} kernel_dir_t;

typedef struct {
    uint8_t  in_use;
    uint8_t  data[PIPE_BUF_SIZE];
    uint16_t read_pos;
    uint16_t write_pos;
    uint16_t count;
    uint16_t reader_count;
    uint16_t writer_count;
    spinlock_t lock;
} kernel_pipe_t;

enum {
    FILE_USED_FILE     = 1,
    FILE_USED_PIPE_R   = 2,
    FILE_USED_PIPE_W   = 3,
    FILE_USED_DIR      = 4,
    FILE_USED_TIMERFD  = 5,
    FILE_USED_MEMFD    = 6,
    FILE_USED_SIGNALFD = 7,
};

typedef struct {
    uint8_t used;
    int32_t owner_pid;
    uint64_t next_deadline_ms;
    uint64_t interval_ms;
} kernel_timerfd_t;

typedef struct {
    uint8_t used;
    int32_t owner_pid;
    uint8_t *data;         /* legacy heap backing (only if shm_handle < 0) */
    uint32_t size;
    uint32_t capacity;
    uint32_t offset;
    /* When >= 0 the memfd is backed by a cross-process shared-memory object
     * (Kernel/Core/memory/SharedMemory.c) instead of `data`. This is what
     * makes mmap(MAP_SHARED) coherent between processes and lets the fd be
     * handed to another process via SCM_RIGHTS - the path Wayland's wl_shm
     * needs. Promoted on the first non-zero ftruncate(). Each fd referencing
     * the object holds one shared_memory reference (create/addref on dup,
     * release on close). */
    int32_t shm_handle;
} kernel_memfd_t;

typedef struct {
    uint8_t used;
    int32_t owner_pid;
    uint64_t mask;
} kernel_signalfd_t;

static kernel_file_t g_files[FILE_MAX_FD];
static kernel_open_file_t g_open_files[FILE_MAX_FD];
static kernel_dir_t g_dirs[FILE_MAX_DIR_HANDLE];
static kernel_pipe_t g_pipes[PIPE_MAX_COUNT];
static kernel_timerfd_t g_timerfds[FILE_MAX_FD];
static kernel_memfd_t g_memfds[FILE_MAX_FD];
static kernel_signalfd_t g_signalfds[FILE_MAX_FD];
static spinlock_t g_file_table_lock;
static spinlock_t g_dir_table_lock;

static kernel_pipe_t *find_pipe_for_fd(int32_t fd, int *is_read_end);
static void release_fd_locked(int32_t fd);
static int  fd_extra_owner_test(const kernel_file_t *f, int32_t pid);
static void fd_extra_owner_clear(kernel_file_t *f, int32_t pid);
static int  fd_promote_owner_locked(kernel_file_t *f);
static int64_t syscall_pipe_read(int32_t fd, uint8_t *buffer, uint64_t len);
static int64_t syscall_pipe_write(int32_t fd, const uint8_t *buffer, uint64_t len);

static uint64_t timer_ms_now(void)
{
    uint32_t hz = timer_hz();
    if (hz == 0) {
        hz = 60;
    }
    return (timer_ticks() * 1000u) / hz;
}

__attribute__((unused))
static uint32_t count_used_file_slots(void)
{
    uint32_t used = 0;
    for (int32_t fd = 0; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used != 0) {
            ++used;
        }
    }
    return used;
}

__attribute__((unused))
static uint32_t count_used_dir_slots(void)
{
    uint32_t used = 0;
    for (int32_t i = 0; i < FILE_MAX_DIR_HANDLE; ++i) {
        if (g_dirs[i].used != 0) {
            ++used;
        }
    }
    return used;
}

static void open_file_cache_invalidate(kernel_open_file_t *file)
{
    if (file == NULL) {
        return;
    }

    file->cache_valid = 0;
    file->cache_offset = 0;
    file->cache_size = 0;
}

static int open_file_cache_refill(kernel_open_file_t *file, uint32_t offset)
{
    if (file == NULL) {
        return 0;
    }

    if (offset >= file->file.size) {
        open_file_cache_invalidate(file);
        return 1;
    }

    uint32_t remaining = file->file.size - offset;
    uint32_t to_cache = remaining;
    if (to_cache > FILE_READ_CACHE_SIZE) {
        to_cache = FILE_READ_CACHE_SIZE;
    }

    if (!vfs_read_at(&file->file, offset, file->cache_data, to_cache)) {
        open_file_cache_invalidate(file);
        return 0;
    }

    file->cache_valid = 1;
    file->cache_offset = offset;
    file->cache_size = to_cache;
    return 1;
}

static kernel_open_file_t *fd_open_file(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != 1) {
        return NULL;
    }

    int32_t open_index = g_files[fd].open_index;
    if (open_index < 0 || open_index >= FILE_MAX_FD ||
        g_open_files[open_index].used == 0) {
        return NULL;
    }

    return &g_open_files[open_index];
}

static int32_t allocate_open_file_locked(void)
{
    for (int32_t i = 0; i < FILE_MAX_FD; ++i) {
        if (g_open_files[i].used == 0) {
            return i;
        }
    }
    return -1;
}

/* Drop one owner's claim on `fd`. The backing object is released only when
 * nobody else holds the slot -- after fork both parent and child own it, and
 * either closing must not pull it out from under the other. */
static void release_fd_locked_for(int32_t fd, int32_t pid)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0) {
        return;
    }
    if (g_files[fd].owner_pid == pid) {
        if (fd_promote_owner_locked(&g_files[fd])) {
            return; /* another process still holds this descriptor */
        }
    } else if (fd_extra_owner_test(&g_files[fd], pid)) {
        fd_extra_owner_clear(&g_files[fd], pid);
        return;
    }
    release_fd_locked(fd);
}

static void release_fd_locked(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0) {
        return;
    }

    if (g_files[fd].used == 1) {
        int32_t open_index = g_files[fd].open_index;
        if (open_index >= 0 && open_index < FILE_MAX_FD &&
            g_open_files[open_index].used != 0) {
            if (g_open_files[open_index].refcount > 0) {
                --g_open_files[open_index].refcount;
            }
            if (g_open_files[open_index].refcount == 0) {
                vfs_close_file(&g_open_files[open_index].file);
                memset(&g_open_files[open_index], 0, sizeof(g_open_files[open_index]));
            }
        }
    } else if (g_files[fd].used == 2 || g_files[fd].used == 3) {
        int32_t pipe_index = g_files[fd].open_index;
        if (pipe_index >= 0 && pipe_index < PIPE_MAX_COUNT &&
            g_pipes[pipe_index].in_use != 0u) {
            kernel_pipe_t *pipe = &g_pipes[pipe_index];
            if (g_files[fd].used == 2 && pipe->reader_count > 0u) {
                --pipe->reader_count;
            } else if (g_files[fd].used == 3 && pipe->writer_count > 0u) {
                --pipe->writer_count;
            }
            if (pipe->reader_count == 0u && pipe->writer_count == 0u) {
                memset(pipe, 0, sizeof(*pipe));
            }
        }
    } else if (g_files[fd].used == 4) {
        int32_t dir_index = g_files[fd].open_index;
        if (dir_index >= 0 && dir_index < FILE_MAX_DIR_HANDLE &&
            g_dirs[dir_index].used != 0u) {
            (void)vfs_closedir(g_dirs[dir_index].vfs_handle);
            memset(&g_dirs[dir_index], 0, sizeof(g_dirs[dir_index]));
        }
    } else if (g_files[fd].used == 5) {
        memset(&g_timerfds[fd], 0, sizeof(g_timerfds[fd]));
    } else if (g_files[fd].used == 6) {
        kernel_memfd_t *memfd = &g_memfds[fd];
        if (memfd->shm_handle >= 0) {
            (void)shared_memory_release(memfd->shm_handle);
        } else if (memfd->data != NULL) {
            free(memfd->data);
        }
        memset(memfd, 0, sizeof(*memfd));
        memfd->shm_handle = -1;
    } else if (g_files[fd].used == 7) {
        memset(&g_signalfds[fd], 0, sizeof(g_signalfds[fd]));
    }

    memset(&g_files[fd], 0, sizeof(g_files[fd]));
    g_files[fd].open_index = -1;
}

static int fd_extra_owner_test(const kernel_file_t *f, int32_t pid)
{
    if (pid < 0 || pid >= OS_CONFIG_PROCESS_MAX_COUNT) return 0;
    return (f->extra_owners[pid >> 6] >> (pid & 63)) & 1u ? 1 : 0;
}

static void fd_extra_owner_set(kernel_file_t *f, int32_t pid)
{
    if (pid < 0 || pid >= OS_CONFIG_PROCESS_MAX_COUNT) return;
    f->extra_owners[pid >> 6] |= (1ULL << (pid & 63));
}

static void fd_extra_owner_clear(kernel_file_t *f, int32_t pid)
{
    if (pid < 0 || pid >= OS_CONFIG_PROCESS_MAX_COUNT) return;
    f->extra_owners[pid >> 6] &= ~(1ULL << (pid & 63));
}

/* Hands `owner_pid` over to one of the extra owners, clearing that bit.
 * Returns 1 if a successor was found, 0 if this was the last owner. */
static int fd_promote_owner_locked(kernel_file_t *f)
{
    for (int32_t pid = 0; pid < OS_CONFIG_PROCESS_MAX_COUNT; ++pid) {
        if (fd_extra_owner_test(f, pid)) {
            fd_extra_owner_clear(f, pid);
            f->owner_pid = pid;
            return 1;
        }
    }
    return 0;
}

static int fd_is_owned_by_current_process(int32_t fd)
{
    int32_t current_pid = process_get_current_pid();
    if (current_pid < 0) {
        return 0;
    }
    return (g_files[fd].owner_pid == current_pid) ||
           fd_extra_owner_test(&g_files[fd], current_pid);
}

static int dir_is_owned_by_current_process(int32_t dir_handle)
{
    int32_t current_pid = process_get_current_pid();
    if (current_pid < 0) {
        return 0;
    }
    return (g_dirs[dir_handle].owner_pid == current_pid);
}

void syscall_file_init(void)
{
    spinlock_init(&g_file_table_lock);
    spinlock_init(&g_dir_table_lock);
    memset(g_files, 0, sizeof(g_files));
    memset(g_open_files, 0, sizeof(g_open_files));
    memset(g_dirs, 0, sizeof(g_dirs));
    memset(g_pipes, 0, sizeof(g_pipes));
    for (int32_t fd = 0; fd < FILE_MAX_FD; ++fd) {
        g_files[fd].open_index = -1;
    }
}

int32_t syscall_file_open(const char *path, uint64_t flags)
{
    vfs_file_t file;
    int32_t current_pid = process_get_current_pid();

    if (path == NULL || path[0] == '\0' || current_pid < 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }

    if (!vfs_find_file(path, &file)) {
        return (int32_t)OS_STATUS_NOT_FOUND;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    /* fd 0/1/2 are implicitly reserved as console fds; allocate from 3. */
    for (int32_t fd = 3; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) {
            int32_t open_index = allocate_open_file_locked();
            if (open_index < 0) {
                spinlock_unlock(&g_file_table_lock);
                irq_restore(irq_flags);
                return (int32_t)OS_STATUS_LIMIT_REACHED;
            }

            memset(&g_open_files[open_index], 0, sizeof(g_open_files[open_index]));
            g_open_files[open_index].used = 1;
            uint32_t access_mode = (uint32_t)flags & FILE_O_ACCMODE;
            g_open_files[open_index].writable =
                (access_mode == FILE_O_WRONLY || access_mode == FILE_O_RDWR) ? 1u : 0u;
            g_open_files[open_index].file = file;
            g_open_files[open_index].offset = 0;
            g_open_files[open_index].refcount = 1;
            open_file_cache_invalidate(&g_open_files[open_index]);

            g_files[fd].used = 1;
            g_files[fd].owner_pid = current_pid;
            g_files[fd].open_index = open_index;
            g_files[fd].status_flags = (uint32_t)flags;
            int do_truncate = ((flags & FILE_O_TRUNC) != 0u &&
                               g_open_files[open_index].writable != 0u &&
                               g_open_files[open_index].file.size != 0u);
            spinlock_unlock(&g_file_table_lock);
            irq_restore(irq_flags);
            /* O_TRUNC was accepted and then ignored, so fopen(path, "w") over
             * an existing file kept whatever the old contents were past the
             * end of the new ones. Xorg retries its keymap compile, and the
             * second xkbcomp run rewriting /var/lib/xkb/server-0.xkm left the
             * longer first attempt's tail glued on -- an .xkm the server then
             * could not load. Done outside the table lock: vfs_truncate() can
             * reach a filesystem driver that blocks. */
            if (do_truncate) {
                (void)syscall_file_truncate(fd, 0u);
            }
            return fd;
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return (int32_t)OS_STATUS_LIMIT_REACHED;
}

int32_t syscall_file_creat(const char *path)
{
    if (path == NULL || path[0] == '\0' || process_get_current_pid() < 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (!vfs_creat(path)) {
        return (int32_t)OS_STATUS_IO_ERROR;
    }
    return syscall_file_open(path, 1ULL);
}

int64_t syscall_file_read(int32_t fd, uint8_t *buffer, uint64_t len)
{
    if (fd < 0 || fd >= FILE_MAX_FD || buffer == NULL || g_files[fd].used == 0) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    if (!fd_is_owned_by_current_process(fd)) {
        return (int64_t)OS_STATUS_ACCESS_DENIED;
    }
    if (len == 0) {
        return 0;
    }
    if (g_files[fd].used == 2) {
        return syscall_pipe_read(fd, buffer, len);
    }
    if (g_files[fd].used == 5) {
        return syscall_timerfd_read(fd, buffer, len);
    }
    if (g_files[fd].used == 6) {
        return syscall_memfd_read(fd, buffer, len);
    }
    if (g_files[fd].used == 7) {
        return syscall_signalfd_read(fd, buffer, len);
    }
    if (g_files[fd].used != 1) {
        return (int64_t)OS_STATUS_ACCESS_DENIED;
    }

    kernel_open_file_t *file = fd_open_file(fd);
    if (file == NULL) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }

    /* Character devices (/dev/dri/card0, /dev/input/event*) manage their own
     * variable-length record streams; the size/EOF model below does not fit. */
    if (file->file.fs_driver != NULL && file->file.fs_driver->dev_read != NULL) {
        uint32_t nonblock =
            (g_files[fd].status_flags & FILE_O_NONBLOCK) ? 1u : 0u;
        return vfs_dev_read(&file->file, buffer, len, nonblock);
    }

    if (file->offset >= file->file.size) {
        return 0;
    }

    uint64_t remaining = (uint64_t)file->file.size - (uint64_t)file->offset;
    uint64_t to_read = (len < remaining) ? len : remaining;
    uint64_t read_total = 0;
    uint32_t cursor = file->offset;

    while (read_total < to_read) {
        uint32_t cache_end = file->cache_offset + file->cache_size;
        if (file->cache_valid != 0 &&
            cursor >= file->cache_offset &&
            cursor < cache_end) {
            uint32_t cache_index = cursor - file->cache_offset;
            uint32_t available = cache_end - cursor;
            uint64_t remaining_request = to_read - read_total;
            uint32_t chunk = available;
            if (remaining_request < (uint64_t)chunk) {
                chunk = (uint32_t)remaining_request;
            }

            memcpy(buffer + (size_t)read_total,
                   file->cache_data + cache_index,
                   (size_t)chunk);

            read_total += (uint64_t)chunk;
            cursor += chunk;
            continue;
        }

        uint64_t remaining_request = to_read - read_total;
        if (remaining_request >= FILE_READ_DIRECT_THRESHOLD) {
            uint64_t chunk64 = remaining_request;
            if (chunk64 > FILE_IO_CHUNK_SIZE) {
                chunk64 = FILE_IO_CHUNK_SIZE;
            }

            uint32_t chunk = (uint32_t)chunk64;
            if (!vfs_read_at(&file->file,
                             cursor,
                             buffer + (size_t)read_total,
                             chunk)) {
                return (int64_t)OS_STATUS_IO_ERROR;
            }
            open_file_cache_invalidate(file);
            read_total += (uint64_t)chunk;
            cursor += chunk;
            continue;
        }

        if (!open_file_cache_refill(file, cursor)) {
            return (int64_t)OS_STATUS_IO_ERROR;
        }
        if (file->cache_size == 0) {
            break;
        }
    }

    file->offset = cursor;
    return (int64_t)read_total;
}

int64_t syscall_file_write(int32_t fd, const uint8_t *buffer, uint64_t len)
{
    if (fd < 0 || fd >= FILE_MAX_FD || buffer == NULL || g_files[fd].used == 0) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    if (!fd_is_owned_by_current_process(fd)) {
        return (int64_t)OS_STATUS_ACCESS_DENIED;
    }
    if (g_files[fd].used == 3) {
        return syscall_pipe_write(fd, buffer, len);
    }
    if (g_files[fd].used == 6) {
        return syscall_memfd_write(fd, buffer, len);
    }
    if (g_files[fd].used != 1) {
        return (int64_t)OS_STATUS_ACCESS_DENIED;
    }

    kernel_open_file_t *file = fd_open_file(fd);
    if (file == NULL) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }

    if (file->writable == 0) {
        return (int64_t)OS_STATUS_ACCESS_DENIED;
    }
    if (len == 0) {
        return 0;
    }
    if ((g_files[fd].status_flags & FILE_O_APPEND) != 0u) {
        file->offset = file->file.size;
    }

    uint64_t write_total = 0;

    while (write_total < len) {
        uint64_t chunk64 = len - write_total;
        if (chunk64 > FILE_IO_CHUNK_SIZE) {
            chunk64 = FILE_IO_CHUNK_SIZE;
        }

        uint32_t chunk = (uint32_t)chunk64;
        uint32_t write_offset = file->offset + (uint32_t)write_total;
        if (!vfs_write_at(&file->file,
                            write_offset,
                            buffer + (size_t)write_total,
                            chunk)) {
            return (int64_t)OS_STATUS_IO_ERROR;
        }

        write_total += (uint64_t)chunk;
    }

    file->offset += (uint32_t)len;
    open_file_cache_invalidate(file);
    return (int64_t)len;
}

int64_t syscall_file_seek(int32_t fd, int64_t offset, int32_t whence)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    if (!fd_is_owned_by_current_process(fd)) {
        return (int64_t)OS_STATUS_ACCESS_DENIED;
    }

    if (g_files[fd].used == 6) {
        kernel_memfd_t *memfd = &g_memfds[fd];
        int64_t base = 0;
        switch (whence) {
            case FILE_SEEK_SET:
                base = 0;
                break;
            case FILE_SEEK_CUR:
                base = (int64_t)memfd->offset;
                break;
            case FILE_SEEK_END:
                base = (int64_t)memfd->size;
                break;
            default:
                return (int64_t)OS_STATUS_INVALID_ARG;
        }
        int64_t next = base + offset;
        if (next < 0 || (uint64_t)next > (uint64_t)memfd->size) {
            return (int64_t)OS_STATUS_INVALID_ARG;
        }
        memfd->offset = (uint32_t)next;
        return next;
    }

    kernel_open_file_t *file = fd_open_file(fd);
    if (file == NULL) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }

    int64_t base = 0;
    switch (whence) {
        case FILE_SEEK_SET:
            base = 0;
            break;
        case FILE_SEEK_CUR:
            base = (int64_t)file->offset;
            break;
        case FILE_SEEK_END:
            base = (int64_t)file->file.size;
            break;
        default:
            return (int64_t)OS_STATUS_INVALID_ARG;
    }

    int64_t next = base + offset;
    if (next < 0 || (uint64_t)next > (uint64_t)file->file.size) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }

    file->offset = (uint32_t)next;
    return next;
}

int32_t syscall_file_close(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (!fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_ACCESS_DENIED;
    }

    int32_t self = process_get_current_pid();
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    release_fd_locked_for(fd, self);
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int32_t syscall_file_mkdir(const char *path)
{
    if (path == NULL || path[0] == '\0' || process_get_current_pid() < 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }

    return vfs_mkdir(path) ? 0 : (int32_t)OS_STATUS_IO_ERROR;
}

int32_t syscall_file_opendir(const char *path)
{
    if (path == NULL || path[0] == '\0' || process_get_current_pid() < 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }

    int32_t vfs_handle = vfs_opendir(path);
    if (vfs_handle < 0) {
        return (int32_t)OS_STATUS_NOT_FOUND;
    }

    int32_t current_pid = process_get_current_pid();
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_dir_table_lock);
    for (int32_t i = 0; i < FILE_MAX_DIR_HANDLE; ++i) {
        if (g_dirs[i].used == 0) {
            g_dirs[i].used = 1;
            g_dirs[i].owner_pid = current_pid;
            g_dirs[i].vfs_handle = vfs_handle;
            spinlock_unlock(&g_dir_table_lock);
            irq_restore(irq_flags);
            return i;
        }
    }
    spinlock_unlock(&g_dir_table_lock);
    irq_restore(irq_flags);

    (void)vfs_closedir(vfs_handle);
    return (int32_t)OS_STATUS_LIMIT_REACHED;
}

int32_t syscall_file_readdir(int32_t dir_handle, vfs_dirent_t *out_entry)
{
    if (dir_handle < 0 || dir_handle >= FILE_MAX_DIR_HANDLE || out_entry == NULL ||
        g_dirs[dir_handle].used == 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (!dir_is_owned_by_current_process(dir_handle)) {
        return (int32_t)OS_STATUS_ACCESS_DENIED;
    }

    return vfs_readdir(g_dirs[dir_handle].vfs_handle, out_entry);
}

int32_t syscall_file_closedir(int32_t dir_handle)
{
    if (dir_handle < 0 || dir_handle >= FILE_MAX_DIR_HANDLE || g_dirs[dir_handle].used == 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (!dir_is_owned_by_current_process(dir_handle)) {
        return (int32_t)OS_STATUS_ACCESS_DENIED;
    }

    (void)vfs_closedir(g_dirs[dir_handle].vfs_handle);
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_dir_table_lock);
    memset(&g_dirs[dir_handle], 0, sizeof(g_dirs[dir_handle]));
    spinlock_unlock(&g_dir_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int32_t syscall_file_unlink(const char *path)
{
    if (path == NULL || path[0] == '\0' || process_get_current_pid() < 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    return vfs_unlink(path) ? 0 : (int32_t)OS_STATUS_IO_ERROR;
}

int32_t syscall_file_rename(const char *old_path, const char *new_path)
{
    if (old_path == NULL || new_path == NULL ||
        old_path[0] == '\0' || new_path[0] == '\0' ||
        process_get_current_pid() < 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    return vfs_rename(old_path, new_path) ?
        0 : (int32_t)OS_STATUS_IO_ERROR;
}

/* link(2): no true hard links in the pseudo filesystems, so implement it as a
 * content copy. Sufficient for the one real consumer -- the X server's
 * LockServer() (os/utils.c) writes /tmp/.tXn-lock then link()s it to
 * /tmp/.Xn-lock and re-reads the pid. Fails with EXDEV for anything the VFS
 * cannot both read and (re)create. */
#define SYSCALL_LINK_MAX_BYTES 65536u

int32_t syscall_file_link(const char *old_path, const char *new_path)
{
    if (old_path == NULL || new_path == NULL ||
        old_path[0] == '\0' || new_path[0] == '\0' ||
        process_get_current_pid() < 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }

    vfs_file_t src;
    if (!vfs_find_file(old_path, &src)) {
        return (int32_t)OS_STATUS_NOT_FOUND;
    }
    vfs_file_t probe;
    if (vfs_find_file(new_path, &probe)) {
        return (int32_t)OS_STATUS_INVALID_ARG; /* EEXIST */
    }

    uint32_t size = src.size;
    if (size > SYSCALL_LINK_MAX_BYTES) {
        return (int32_t)OS_STATUS_IO_ERROR; /* -EXDEV-ish: too big to fake */
    }

    uint8_t *buf = NULL;
    if (size > 0u) {
        buf = (uint8_t *)malloc(size);
        if (buf == NULL) {
            return (int32_t)OS_STATUS_LIMIT_REACHED;
        }
        if (!vfs_read_at(&src, 0u, buf, size)) {
            free(buf);
            return (int32_t)OS_STATUS_IO_ERROR;
        }
    }

    if (!vfs_creat(new_path)) {
        free(buf);
        return (int32_t)OS_STATUS_IO_ERROR;
    }
    if (size > 0u) {
        vfs_file_t dst;
        if (!vfs_find_file(new_path, &dst) ||
            !vfs_write_at(&dst, 0u, buf, size)) {
            free(buf);
            (void)vfs_unlink(new_path);
            return (int32_t)OS_STATUS_IO_ERROR;
        }
    }
    free(buf);
    return 0;
}

void syscall_file_close_all_for_pid(int32_t pid, uint32_t *closed_fds_out, uint32_t *closed_dirs_out)
{
    if (closed_fds_out != NULL) {
        *closed_fds_out = 0;
    }
    if (closed_dirs_out != NULL) {
        *closed_dirs_out = 0;
    }

    if (pid < 0) {
        return;
    }

    uint32_t closed_fds = 0;
    uint32_t closed_dirs = 0;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    for (int32_t fd = 0; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used != 0 &&
            (g_files[fd].owner_pid == pid ||
             fd_extra_owner_test(&g_files[fd], pid))) {
            release_fd_locked_for(fd, pid);
            ++closed_fds;
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);

    irq_flags = irq_save_disable();
    spinlock_lock(&g_dir_table_lock);
    for (int32_t i = 0; i < FILE_MAX_DIR_HANDLE; ++i) {
        if (g_dirs[i].used != 0 && g_dirs[i].owner_pid == pid) {
            (void)vfs_closedir(g_dirs[i].vfs_handle);
            memset(&g_dirs[i], 0, sizeof(g_dirs[i]));
            ++closed_dirs;
        }
    }
    spinlock_unlock(&g_dir_table_lock);
    irq_restore(irq_flags);

    if (closed_fds_out != NULL) {
        *closed_fds_out = closed_fds;
    }
    if (closed_dirs_out != NULL) {
        *closed_dirs_out = closed_dirs;
    }
}

void syscall_file_close_cloexec_for_pid(int32_t pid)
{
    if (pid < 0) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    for (int32_t fd = 0; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used != 0 &&
            (g_files[fd].owner_pid == pid ||
             fd_extra_owner_test(&g_files[fd], pid)) &&
            (g_files[fd].descriptor_flags & FILE_FD_CLOEXEC) != 0u) {
            release_fd_locked_for(fd, pid);
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);

    irq_flags = irq_save_disable();
    spinlock_lock(&g_dir_table_lock);
    for (int32_t i = 0; i < FILE_MAX_DIR_HANDLE; ++i) {
        if (g_dirs[i].used != 0 && g_dirs[i].owner_pid == pid) {
            (void)vfs_closedir(g_dirs[i].vfs_handle);
            memset(&g_dirs[i], 0, sizeof(g_dirs[i]));
        }
    }
    spinlock_unlock(&g_dir_table_lock);
    irq_restore(irq_flags);
}

int32_t syscall_file_pipe(int32_t fds_out[2])
{
    if (fds_out == NULL) {
        return (int32_t)OS_STATUS_FAULT;
    }

    int32_t pipe_idx = -1;
    for (int32_t i = 0; i < PIPE_MAX_COUNT; ++i) {
        if (g_pipes[i].in_use == 0) {
            pipe_idx = i;
            break;
        }
    }
    if (pipe_idx < 0) {
        return (int32_t)OS_STATUS_LIMIT_REACHED;
    }

    int32_t read_fd = -1, write_fd = -1;
    int32_t current_pid = process_get_current_pid();

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    for (int32_t fd = 3; fd < FILE_MAX_FD && (read_fd < 0 || write_fd < 0); ++fd) {
        if (g_files[fd].used == 0) {
            if (read_fd < 0) {
                read_fd = fd;
            } else {
                write_fd = fd;
            }
        }
    }

    if (read_fd < 0 || write_fd < 0) {
        spinlock_unlock(&g_file_table_lock);
        irq_restore(irq_flags);
        return (int32_t)OS_STATUS_LIMIT_REACHED;
    }

    memset(&g_files[read_fd], 0, sizeof(g_files[read_fd]));
    g_files[read_fd].used = 2;
    g_files[read_fd].owner_pid = current_pid;
    g_files[read_fd].open_index = pipe_idx;
    g_files[read_fd].status_flags = 0u;

    memset(&g_files[write_fd], 0, sizeof(g_files[write_fd]));
    g_files[write_fd].used = 3;
    g_files[write_fd].owner_pid = current_pid;
    g_files[write_fd].open_index = pipe_idx;
    g_files[write_fd].status_flags = FILE_O_WRONLY;

    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);

    kernel_pipe_t *pipe = &g_pipes[pipe_idx];
    memset(pipe, 0, sizeof(*pipe));
    spinlock_init(&pipe->lock);
    pipe->in_use = 1;
    pipe->reader_count = 1u;
    pipe->writer_count = 1u;

    fds_out[0] = read_fd;
    fds_out[1] = write_fd;

    return 0;
}

static kernel_pipe_t *find_pipe_for_fd(int32_t fd, int *is_read_end)
{
    if (fd < 0 || fd >= FILE_MAX_FD ||
        (g_files[fd].used != 2 && g_files[fd].used != 3)) {
        return NULL;
    }
    int32_t index = g_files[fd].open_index;
    if (index < 0 || index >= PIPE_MAX_COUNT || g_pipes[index].in_use == 0u) {
        return NULL;
    }
    if (is_read_end) *is_read_end = (g_files[fd].used == 2);
    return &g_pipes[index];
}

/* Longest a blocking pipe read waits for a writer before giving up. Only a
 * backstop against wedging the caller forever if the writer dies without
 * closing; real transfers complete in milliseconds. */
#define PIPE_READ_WAIT_MAX_MS 30000u

static int64_t syscall_pipe_read(int32_t fd, uint8_t *buffer, uint64_t len)
{
    int is_read = 0;
    kernel_pipe_t *pipe = find_pipe_for_fd(fd, &is_read);
    if (pipe == NULL || !is_read) return (int64_t)OS_STATUS_IO_ERROR;

    uint64_t waited_ms = 0;
    for (;;) {
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&pipe->lock);

        uint64_t bytes_read = 0;
        while (bytes_read < len && pipe->count > 0) {
            buffer[bytes_read++] = pipe->data[pipe->read_pos];
            pipe->read_pos = (uint16_t)((pipe->read_pos + 1u) % PIPE_BUF_SIZE);
            pipe->count--;
        }
        uint16_t writers = pipe->writer_count;

        spinlock_unlock(&pipe->lock);
        irq_restore(irq_flags);

        if (bytes_read > 0u) {
            return (int64_t)bytes_read;
        }
        /* An empty pipe is only end-of-file once every writer has closed.
         * Returning 0 regardless meant a reader that got there first saw EOF
         * on a pipe that was about to be filled -- which is how xkbcomp, run
         * by Xorg through Popen(), read an "empty" stdin and reported
         * "syntax error: line 1 of stdin". */
        if (writers == 0u) {
            return 0;
        }
        if ((g_files[fd].status_flags & FILE_O_NONBLOCK) != 0u) {
            return -11; /* EAGAIN */
        }
        if (waited_ms >= PIPE_READ_WAIT_MAX_MS) {
            return -11;
        }
        (void)process_sleep_current_ms(1);
        waited_ms += 1u;
    }
}

/* Blocks until the pipe has room for at least one byte, then writes what
 * fits. Returns EAGAIN for a non-blocking fd, or if the reader never drains
 * within PIPE_READ_WAIT_MAX_MS, or once every reader has gone (SIGPIPE). */
static int64_t syscall_pipe_write_wait(int32_t fd, kernel_pipe_t *pipe,
                                       const uint8_t *buffer, uint64_t len)
{
    if ((g_files[fd].status_flags & FILE_O_NONBLOCK) != 0u) {
        return -11; /* EAGAIN */
    }

    uint64_t waited_ms = 0;
    for (;;) {
        if (waited_ms >= PIPE_READ_WAIT_MAX_MS) {
            return -11; /* EAGAIN */
        }
        (void)process_sleep_current_ms(1);
        waited_ms += 1u;

        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&pipe->lock);

        if (pipe->reader_count == 0u) {
            spinlock_unlock(&pipe->lock);
            irq_restore(irq_flags);
            int32_t self = process_get_current_pid();
            if (self >= 0) {
                (void)process_signal_deliver(self, 13 /* SIGPIPE */);
            }
            return (int64_t)OS_STATUS_BROKEN_PIPE;
        }

        uint64_t bytes_written = 0;
        while (bytes_written < len && pipe->count < PIPE_BUF_SIZE) {
            pipe->data[pipe->write_pos] = buffer[bytes_written++];
            pipe->write_pos = (uint16_t)((pipe->write_pos + 1u) % PIPE_BUF_SIZE);
            pipe->count++;
        }

        spinlock_unlock(&pipe->lock);
        irq_restore(irq_flags);

        if (bytes_written > 0u) {
            return (int64_t)bytes_written;
        }
    }
}

static int64_t syscall_pipe_write(int32_t fd, const uint8_t *buffer, uint64_t len)
{
    int is_read = 0;
    kernel_pipe_t *pipe = find_pipe_for_fd(fd, &is_read);
    if (pipe == NULL || is_read) return (int64_t)OS_STATUS_IO_ERROR;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&pipe->lock);

    if (pipe->reader_count == 0u) {
        /* POSIX: write to a pipe with no readers raises SIGPIPE (default
         * action terminates; if caught or ignored the write fails with
         * EPIPE). Post the signal to the current process and return EPIPE -
         * the pending-signal path applies the correct disposition. */
        spinlock_unlock(&pipe->lock);
        irq_restore(irq_flags);
        int32_t self = process_get_current_pid();
        if (self >= 0) {
            (void)process_signal_deliver(self, 13 /* SIGPIPE */);
        }
        return (int64_t)OS_STATUS_BROKEN_PIPE;
    }

    uint64_t bytes_written = 0;
    while (bytes_written < len && pipe->count < PIPE_BUF_SIZE) {
        pipe->data[pipe->write_pos] = buffer[bytes_written++];
        pipe->write_pos = (uint16_t)((pipe->write_pos + 1u) % PIPE_BUF_SIZE);
        pipe->count++;
    }

    spinlock_unlock(&pipe->lock);
    irq_restore(irq_flags);

    if (bytes_written > 0u) {
        return (int64_t)bytes_written;
    }

    /* The buffer was full, so not one byte went in. Returning 0 is not an
     * option: a blocking write(2) never reports 0 for a non-empty request, and
     * glibc's _IO_new_file_write loops `while (to_do > 0)` on the result --
     * a 0 leaves to_do unchanged and spins the caller forever. Wait for the
     * reader to make room, exactly as syscall_pipe_read() waits for a writer.
     * Xorg hits this every time it feeds a keymap (tens of KB) to xkbcomp
     * through a 4 KiB pipe. */
    return syscall_pipe_write_wait(fd, pipe, buffer, len);
}

int syscall_file_is_pipe(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD) return 0;
    return (g_files[fd].used == 2 || g_files[fd].used == 3);
}

/* ---- mmap(2) references on the open file description -------------------
 * A mapping outlives the fd it was made from (glibc's loader closes the fd as
 * soon as the shared object is mapped), so demand-paged file mappings hold a
 * reference on the kernel_open_file_t rather than on the fd. See
 * Core/memory/FileMap.c. */

int32_t syscall_file_mmap_acquire(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != FILE_USED_FILE ||
        !fd_is_owned_by_current_process(fd)) {
        return -1;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    int32_t open_index = g_files[fd].open_index;
    int32_t result = -1;
    if (open_index >= 0 && open_index < FILE_MAX_FD &&
        g_open_files[open_index].used != 0) {
        ++g_open_files[open_index].refcount;
        result = open_index;
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return result;
}

int32_t syscall_file_mmap_reacquire(int32_t handle)
{
    if (handle < 0 || handle >= FILE_MAX_FD) {
        return -1;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    int32_t result = -1;
    if (g_open_files[handle].used != 0) {
        ++g_open_files[handle].refcount;
        result = handle;
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return result;
}

int64_t syscall_file_mmap_read(int32_t handle, uint64_t offset,
                               uint8_t *kernel_buffer, uint32_t length)
{
    /* Deliberately lock-free: this is called from the #PF handler, and the
     * only fields touched are the immutable `file` descriptor of an entry the
     * caller holds a reference on. */
    if (handle < 0 || handle >= FILE_MAX_FD || kernel_buffer == NULL) {
        return -1;
    }
    kernel_open_file_t *open_file = &g_open_files[handle];
    if (open_file->used == 0) {
        return -1;
    }
    if (offset >= (uint64_t)open_file->file.size) {
        return 0; /* wholly past EOF: caller keeps the zero-filled page */
    }
    uint64_t remaining = (uint64_t)open_file->file.size - offset;
    if ((uint64_t)length > remaining) {
        length = (uint32_t)remaining;
    }
    if (length == 0u) {
        return 0;
    }
    if (!vfs_read_at(&open_file->file, (uint32_t)offset, kernel_buffer, length)) {
        return -1;
    }
    return (int64_t)length;
}

void syscall_file_mmap_release(int32_t handle)
{
    if (handle < 0 || handle >= FILE_MAX_FD) {
        return;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    if (g_open_files[handle].used != 0) {
        if (g_open_files[handle].refcount > 0) {
            --g_open_files[handle].refcount;
        }
        if (g_open_files[handle].refcount == 0) {
            vfs_close_file(&g_open_files[handle].file);
            memset(&g_open_files[handle], 0, sizeof(g_open_files[handle]));
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
}

int syscall_file_is_dir(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD) return 0;
    return (g_files[fd].used == FILE_USED_DIR &&
            fd_is_owned_by_current_process(fd)) ? 1 : 0;
}

uint32_t syscall_file_poll(int32_t fd, uint32_t events)
{
    enum {
        POLL_IN = 0x0001u,
        POLL_OUT = 0x0004u,
        POLL_ERROR = 0x0008u,
        POLL_INVALID = 0x0020u
    };
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0 ||
        !fd_is_owned_by_current_process(fd))
        return POLL_INVALID;

    if (g_files[fd].used == 1) {
        kernel_open_file_t *file = fd_open_file(fd);
        if (file == NULL) return POLL_ERROR;
        if (file->file.fs_driver != NULL && file->file.fs_driver->dev_poll != NULL) {
            return vfs_dev_poll(&file->file, events);
        }
        uint32_t ready = 0u;
        if ((events & POLL_IN) != 0u) ready |= POLL_IN;
        if ((events & POLL_OUT) != 0u && file->writable != 0u)
            ready |= POLL_OUT;
        return ready;
    }

    if (g_files[fd].used == 4) {
        uint32_t ready = 0u;
        if ((events & POLL_IN) != 0u) ready |= POLL_IN;
        if ((events & POLL_OUT) != 0u) ready |= POLL_OUT;
        return ready;
    }
    if (g_files[fd].used == 5) {
        kernel_timerfd_t *timerfd = &g_timerfds[fd];
        uint64_t now_ms = timer_ms_now();
        uint32_t ready = 0u;
        if ((events & POLL_IN) != 0u && timerfd->interval_ms != 0u &&
            timerfd->next_deadline_ms <= now_ms) {
            ready |= POLL_IN;
        }
        if ((events & POLL_OUT) != 0u) ready |= POLL_OUT;
        return ready;
    }
    if (g_files[fd].used == 6) {
        uint32_t ready = 0u;
        if ((events & POLL_IN) != 0u) ready |= POLL_IN;
        if ((events & POLL_OUT) != 0u) ready |= POLL_OUT;
        return ready;
    }
    if (g_files[fd].used == 7) {
        uint32_t ready = 0u;
        if ((events & POLL_IN) != 0u) {
            kernel_signalfd_t *signalfd = &g_signalfds[fd];
            if (signalfd->mask != 0u &&
                ((uint64_t)process_get_current_pending_signals() & signalfd->mask) != 0u) {
                ready |= POLL_IN;
            }
        }
        return ready;
    }

    int is_read_end = 0;
    kernel_pipe_t *pipe = find_pipe_for_fd(fd, &is_read_end);
    if (pipe == NULL) return POLL_ERROR;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&pipe->lock);
    uint32_t ready = 0u;
    if (is_read_end && (events & POLL_IN) != 0u && pipe->count != 0u)
        ready |= POLL_IN;
    if (!is_read_end && (events & POLL_OUT) != 0u && pipe->count < PIPE_BUF_SIZE)
        ready |= POLL_OUT;
    spinlock_unlock(&pipe->lock);
    irq_restore(irq_flags);
    return ready;
}

int32_t syscall_file_dup(int32_t oldfd)
{
    if (oldfd < 0 || oldfd >= FILE_MAX_FD) {
        return (int32_t)OS_STATUS_FAULT;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);

    if (g_files[oldfd].used == 0 || !fd_is_owned_by_current_process(oldfd)) {
        spinlock_unlock(&g_file_table_lock);
        irq_restore(irq_flags);
        return (int32_t)OS_STATUS_FAULT;
    }

    kernel_open_file_t *open_file = NULL;
    if (g_files[oldfd].used == 1) {
        open_file = fd_open_file(oldfd);
    }
    if (g_files[oldfd].used == 1 && open_file == NULL) {
        spinlock_unlock(&g_file_table_lock);
        irq_restore(irq_flags);
        return (int32_t)OS_STATUS_FAULT;
    }

    int32_t newfd = -1;
    for (int32_t fd = 3; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) {
            newfd = fd;
            break;
        }
    }

    if (newfd < 0) {
        spinlock_unlock(&g_file_table_lock);
        irq_restore(irq_flags);
        return (int32_t)OS_STATUS_LIMIT_REACHED;
    }

    memcpy(&g_files[newfd], &g_files[oldfd], sizeof(g_files[newfd]));
    /* A dup is a new descriptor in the CALLING process, so it starts with
     * exactly one owner. Carrying the source's inherited-owner set over would
     * leave the copy attributed to whoever else held the original -- after
     * which the caller's own close could not release it, and the next process
     * to want the slot would be refused it. */
    g_files[newfd].owner_pid = process_get_current_pid();
    memset(g_files[newfd].extra_owners, 0, sizeof(g_files[newfd].extra_owners));
    if (g_files[newfd].used == 5) g_timerfds[newfd] = g_timerfds[oldfd];
    else if (g_files[newfd].used == 6) {
        g_memfds[newfd] = g_memfds[oldfd];
        if (g_memfds[newfd].shm_handle >= 0) {
            (void)shared_memory_addref(g_memfds[newfd].shm_handle);
        }
    }
    else if (g_files[newfd].used == 7) g_signalfds[newfd] = g_signalfds[oldfd];
    if (open_file != NULL) {
        open_file->refcount++;
    } else {
        kernel_pipe_t *pipe = find_pipe_for_fd(newfd, NULL);
        if (pipe != NULL) {
            if (g_files[newfd].used == 2) ++pipe->reader_count;
            else ++pipe->writer_count;
        }
    }

    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);

    return newfd;
}

int32_t syscall_file_dup2(int32_t oldfd, int32_t newfd)
{
    if (oldfd < 0 || oldfd >= FILE_MAX_FD || newfd < 0 || newfd >= FILE_MAX_FD) {
        return (int32_t)OS_STATUS_FAULT;
    }
    if (oldfd == newfd) return newfd;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);

    if (g_files[oldfd].used == 0 || !fd_is_owned_by_current_process(oldfd)) {
        spinlock_unlock(&g_file_table_lock);
        irq_restore(irq_flags);
        return (int32_t)OS_STATUS_FAULT;
    }

    kernel_open_file_t *open_file = NULL;
    if (g_files[oldfd].used == 1) {
        open_file = fd_open_file(oldfd);
    }
    if (g_files[oldfd].used == 1 && open_file == NULL) {
        spinlock_unlock(&g_file_table_lock);
        irq_restore(irq_flags);
        return (int32_t)OS_STATUS_FAULT;
    }
    
    /* Owners of the slot we are about to rebind. A forked child inherits its
     * parent's descriptors as extra owners, so `owner_pid` alone is the wrong
     * test: it rejected the one case dup2 exists for, the Popen()/fork() child
     * doing dup2(pipe, 0) before exec, with EACCES. Test the full owner set.
     *
     * fd numbers are global in this table, so a rebind is visible to every
     * co-owner -- the slot cannot hold two bindings at once. That is accepted
     * rather than worked around: the reference counts below are per-slot, so
     * carrying the co-owners onto the new binding keeps the accounting exact,
     * and fds 0/1/2 (the only realistic targets, and the only ones open() and
     * pipe() never hand out) are console descriptors with no table entry to
     * lose. Anything not owned by the caller is still refused outright. */
    int32_t self_pid = process_get_current_pid();
    uint64_t inherited_owners[FD_OWNER_WORDS];
    int32_t inherited_primary = -1;
    memset(inherited_owners, 0, sizeof(inherited_owners));

    if (g_files[newfd].used != 0) {
        if (!fd_is_owned_by_current_process(newfd)) {
            spinlock_unlock(&g_file_table_lock);
            irq_restore(irq_flags);
            return (int32_t)OS_STATUS_ACCESS_DENIED;
        }
        memcpy(inherited_owners, g_files[newfd].extra_owners,
               sizeof(inherited_owners));
        inherited_primary = g_files[newfd].owner_pid;
        /* Drops the slot's claim on the old backing object for all of them. */
        release_fd_locked(newfd);
    }

    memcpy(&g_files[newfd], &g_files[oldfd], sizeof(g_files[newfd]));
    /* A dup is a new descriptor in the CALLING process, so it starts with
     * exactly one owner. Carrying the source's inherited-owner set over would
     * leave the copy attributed to whoever else held the original -- after
     * which the caller's own close could not release it, and the next process
     * to want the slot would be refused it. */
    g_files[newfd].owner_pid = self_pid;
    memcpy(g_files[newfd].extra_owners, inherited_owners,
           sizeof(g_files[newfd].extra_owners));
    fd_extra_owner_clear(&g_files[newfd], self_pid);
    if (inherited_primary >= 0 && inherited_primary != self_pid) {
        fd_extra_owner_set(&g_files[newfd], inherited_primary);
    }
    if (g_files[newfd].used == 5) g_timerfds[newfd] = g_timerfds[oldfd];
    else if (g_files[newfd].used == 6) {
        g_memfds[newfd] = g_memfds[oldfd];
        if (g_memfds[newfd].shm_handle >= 0) {
            (void)shared_memory_addref(g_memfds[newfd].shm_handle);
        }
    }
    else if (g_files[newfd].used == 7) g_signalfds[newfd] = g_signalfds[oldfd];
    if (open_file != NULL) {
        open_file->refcount++;
    } else {
        kernel_pipe_t *pipe = find_pipe_for_fd(newfd, NULL);
        if (pipe != NULL) {
            if (g_files[newfd].used == 2) ++pipe->reader_count;
            else ++pipe->writer_count;
        }
    }

    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);

    return newfd;
}

int32_t syscall_file_dup_at_least(int32_t oldfd, int32_t minimum_fd)
{
    if (oldfd < 0 || oldfd >= FILE_MAX_FD ||
        minimum_fd < 0 || minimum_fd >= FILE_MAX_FD) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    if (g_files[oldfd].used == 0 || !fd_is_owned_by_current_process(oldfd)) {
        spinlock_unlock(&g_file_table_lock);
        irq_restore(irq_flags);
        return (int32_t)OS_STATUS_FAULT;
    }

    int32_t newfd = -1;
    for (int32_t fd = minimum_fd; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) {
            newfd = fd;
            break;
        }
    }
    if (newfd < 0) {
        spinlock_unlock(&g_file_table_lock);
        irq_restore(irq_flags);
        return (int32_t)OS_STATUS_LIMIT_REACHED;
    }

    memcpy(&g_files[newfd], &g_files[oldfd], sizeof(g_files[newfd]));
    /* A dup is a new descriptor in the CALLING process, so it starts with
     * exactly one owner. Carrying the source's inherited-owner set over would
     * leave the copy attributed to whoever else held the original -- after
     * which the caller's own close could not release it, and the next process
     * to want the slot would be refused it. */
    g_files[newfd].owner_pid = process_get_current_pid();
    memset(g_files[newfd].extra_owners, 0, sizeof(g_files[newfd].extra_owners));
    if (g_files[newfd].used == 1) {
        kernel_open_file_t *open_file = fd_open_file(newfd);
        if (open_file == NULL) {
            memset(&g_files[newfd], 0, sizeof(g_files[newfd]));
            g_files[newfd].open_index = -1;
            spinlock_unlock(&g_file_table_lock);
            irq_restore(irq_flags);
            return (int32_t)OS_STATUS_FAULT;
        }
        ++open_file->refcount;
    } else {
        kernel_pipe_t *pipe = find_pipe_for_fd(newfd, NULL);
        if (pipe != NULL) {
            if (g_files[newfd].used == 2) ++pipe->reader_count;
            else ++pipe->writer_count;
        }
    }

    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return newfd;
}

int32_t syscall_file_truncate(int32_t fd, uint64_t length)
{
    if (length > UINT32_MAX || fd < 0 || fd >= FILE_MAX_FD ||
        g_files[fd].used == 0 || !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (g_files[fd].used == 6) {
        kernel_memfd_t *memfd = &g_memfds[fd];
        uint32_t new_size = (uint32_t)length;
        if (memfd->shm_handle >= 0) {
            /* Backed by a shared-memory object, which has no resize. Grow is
             * rejected (the Wayland client must recreate the pool); shrink /
             * same is a no-op. */
            if (new_size > memfd->size) {
                serial_write_string(
                    "[memfd] ftruncate grow on shm-backed memfd unsupported\n");
                return (int32_t)OS_STATUS_NOT_SUPPORTED;
            }
            return 0;
        }
        if (new_size == 0u) {
            memfd->size = 0u;
            memfd->offset = 0u;
            return 0;
        }
        /* First non-zero size: promote to a shared-memory backing so the
         * mapping is cross-process coherent and the fd is SCM_RIGHTS-able. */
        int32_t handle = shared_memory_create(new_size);
        if (handle < 0) {
            return (int32_t)OS_STATUS_LIMIT_REACHED;
        }
        if (memfd->data != NULL) {
            free(memfd->data);
            memfd->data = NULL;
            memfd->capacity = 0u;
        }
        memfd->shm_handle = handle;
        memfd->size = new_size;
        memfd->offset = 0u;
        return 0;
    }
    if (g_files[fd].used != 1) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    kernel_open_file_t *file = fd_open_file(fd);
    if (file == NULL || file->writable == 0u) {
        return (int32_t)OS_STATUS_ACCESS_DENIED;
    }
    if (!vfs_truncate(&file->file, (uint32_t)length)) {
        return (int32_t)OS_STATUS_IO_ERROR;
    }
    file->file.size = (uint32_t)length;
    if (file->offset > file->file.size) file->offset = file->file.size;
    open_file_cache_invalidate(file);
    return 0;
}

int32_t syscall_file_get_status_flags(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0 ||
        !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_FAULT;
    }
    return (int32_t)g_files[fd].status_flags;
}

int32_t syscall_file_set_status_flags(int32_t fd, uint32_t flags)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0 ||
        !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_FAULT;
    }
    uint32_t preserved = g_files[fd].status_flags & FILE_O_ACCMODE;
    g_files[fd].status_flags =
        preserved | (flags & (FILE_O_APPEND | FILE_O_NONBLOCK));
    return 0;
}

int32_t syscall_file_get_descriptor_flags(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0 ||
        !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_FAULT;
    }
    return (int32_t)g_files[fd].descriptor_flags;
}

int32_t syscall_file_set_descriptor_flags(int32_t fd, uint32_t flags)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0 ||
        !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_FAULT;
    }
    g_files[fd].descriptor_flags = flags & FILE_FD_CLOEXEC;
    return 0;
}

int64_t syscall_file_available(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0 ||
        !fd_is_owned_by_current_process(fd)) {
        return (int64_t)OS_STATUS_FAULT;
    }
    if (g_files[fd].used == 1) {
        kernel_open_file_t *file = fd_open_file(fd);
        if (file == NULL) return (int64_t)OS_STATUS_FAULT;
        return file->offset < file->file.size ?
            (int64_t)(file->file.size - file->offset) : 0;
    }
    if (g_files[fd].used == 5) {
        kernel_timerfd_t *timerfd = &g_timerfds[fd];
        uint64_t now_ms = timer_ms_now();
        if (timerfd->interval_ms != 0u && timerfd->next_deadline_ms <= now_ms) {
            uint64_t extra = (now_ms - timerfd->next_deadline_ms) / timerfd->interval_ms + 1u;
            return (int64_t)extra;
        }
        return 0;
    }
    if (g_files[fd].used == 6) {
        kernel_memfd_t *memfd = &g_memfds[fd];
        return memfd->offset < memfd->size ? (int64_t)(memfd->size - memfd->offset) : 0;
    }
    if (g_files[fd].used == 7) {
        kernel_signalfd_t *signalfd = &g_signalfds[fd];
        if (signalfd->mask != 0u &&
            ((uint64_t)process_get_current_pending_signals() & signalfd->mask) != 0u) {
            return 1;
        }
        return 0;
    }
    int is_read_end = 0;
    kernel_pipe_t *pipe = find_pipe_for_fd(fd, &is_read_end);
    if (pipe == NULL || !is_read_end) return 0;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&pipe->lock);
    int64_t available = (int64_t)pipe->count;
    spinlock_unlock(&pipe->lock);
    irq_restore(irq_flags);
    return available;
}

int64_t syscall_timerfd_read(int32_t fd, uint8_t *buffer, uint64_t len)
{
    kernel_timerfd_t *timerfd = &g_timerfds[fd];
    uint64_t now_ms = timer_ms_now();
    uint64_t expirations = 0;

    if (timerfd->interval_ms != 0u && timerfd->next_deadline_ms <= now_ms) {
        expirations = (now_ms - timerfd->next_deadline_ms) / timerfd->interval_ms + 1u;
        timerfd->next_deadline_ms =
            timerfd->next_deadline_ms + expirations * timerfd->interval_ms;
        if (timerfd->next_deadline_ms <= now_ms) {
            timerfd->next_deadline_ms = now_ms + timerfd->interval_ms;
        }
    }

    if (expirations == 0u || len < sizeof(uint64_t)) {
        return (int64_t)OS_STATUS_WOULD_BLOCK;
    }
    uint64_t raw_value = expirations;
    memcpy(buffer, &raw_value, sizeof(raw_value));
    return (int64_t)sizeof(raw_value);
}

int64_t syscall_memfd_read(int32_t fd, uint8_t *buffer, uint64_t len)
{
    kernel_memfd_t *memfd = &g_memfds[fd];
    if (memfd->shm_handle >= 0) {
        /* shm-backed memfds are meant to be mmap'd, not read()/write()'n.
         * No consumer does this today (Wayland's wl_shm always mmaps). */
        return (int64_t)OS_STATUS_NOT_SUPPORTED;
    }
    if (memfd->offset >= memfd->size) {
        return 0;
    }
    uint64_t avail = (uint64_t)(memfd->size - memfd->offset);
    if (len > avail) len = avail;
    memcpy(buffer, memfd->data + memfd->offset, (size_t)len);
    memfd->offset += (uint32_t)len;
    return (int64_t)len;
}

int64_t syscall_memfd_write(int32_t fd, const uint8_t *buffer, uint64_t len)
{
    kernel_memfd_t *memfd = &g_memfds[fd];
    if (memfd->shm_handle >= 0) {
        return (int64_t)OS_STATUS_NOT_SUPPORTED;
    }
    uint64_t end = (uint64_t)memfd->offset + len;
    if (end > (uint64_t)memfd->capacity) {
        uint32_t new_capacity = memfd->capacity == 0u ? 4096u : memfd->capacity;
        while ((uint64_t)new_capacity < end) {
            new_capacity *= 2u;
        }
        uint8_t *resized = malloc(new_capacity);
        if (resized == NULL) {
            return (int64_t)OS_STATUS_LIMIT_REACHED;
        }
        if (memfd->data != NULL) {
            memcpy(resized, memfd->data, memfd->size);
            free(memfd->data);
        }
        memfd->data = resized;
        memfd->capacity = new_capacity;
    }
    memcpy(memfd->data + memfd->offset, buffer, (size_t)len);
    memfd->offset += (uint32_t)len;
    if (memfd->offset > memfd->size) {
        memfd->size = memfd->offset;
    }
    return (int64_t)len;
}

int64_t syscall_signalfd_read(int32_t fd, uint8_t *buffer, uint64_t len)
{
    kernel_signalfd_t *signalfd = &g_signalfds[fd];
    if (len < 128u) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }

    uint64_t pending = process_get_current_pending_signals() & signalfd->mask;
    int32_t signum = -1;
    for (uint32_t bit = 1; bit < 64u; ++bit) {
        if ((pending & (1u << bit)) != 0u) {
            signum = (int32_t)bit;
            break;
        }
    }
    if (signum < 0) {
        return (int64_t)OS_STATUS_WOULD_BLOCK;
    }
    (void)process_consume_pending_signal(signum);

    memset(buffer, 0, 128u);
    uint32_t *fields = (uint32_t *)(void *)buffer;
    fields[0] = (uint32_t)signum;
    fields[1] = 0;
    fields[2] = 1;
    fields[3] = (uint32_t)process_get_current_pid();
    fields[4] = 0;
    fields[6] = (uint32_t)process_get_current_tid();
    return 128;
}

int32_t syscall_file_register_dir(const char *path)
{
    if (path == NULL || path[0] == '\0' || process_get_current_pid() < 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    int32_t vfs_handle = vfs_opendir(path);
    if (vfs_handle < 0) {
        return (int32_t)OS_STATUS_NOT_FOUND;
    }

    int32_t current_pid = process_get_current_pid();
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    int32_t result = (int32_t)OS_STATUS_LIMIT_REACHED;
    for (int32_t fd = 3; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) {
            g_files[fd].used = FILE_USED_DIR;
            g_files[fd].owner_pid = current_pid;
            g_files[fd].open_index = -1;
            g_files[fd].status_flags = 0;
            g_files[fd].descriptor_flags = 0;
            result = fd;
            break;
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);

    if (result >= 0) {
        uint64_t dir_irq = irq_save_disable();
        spinlock_lock(&g_dir_table_lock);
        for (int32_t i = 0; i < FILE_MAX_DIR_HANDLE; ++i) {
            if (g_dirs[i].used == 0) {
                g_dirs[i].used = 1;
                g_dirs[i].owner_pid = current_pid;
                g_dirs[i].vfs_handle = vfs_handle;
                spinlock_lock(&g_file_table_lock);
                g_files[result].open_index = i;
                spinlock_unlock(&g_file_table_lock);
                spinlock_unlock(&g_dir_table_lock);
                irq_restore(dir_irq);
                return result;
            }
        }
        spinlock_unlock(&g_dir_table_lock);
        irq_restore(dir_irq);
        (void)syscall_file_close(result);
        result = (int32_t)OS_STATUS_LIMIT_REACHED;
    }

    if (result < 0) {
        (void)vfs_closedir(vfs_handle);
    }
    return result;
}

int32_t syscall_file_get_dir_dirent(int32_t fd, vfs_dirent_t *out_entry)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != FILE_USED_DIR ||
        out_entry == NULL || !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    int32_t dir_index = g_files[fd].open_index;
    if (dir_index < 0 || dir_index >= FILE_MAX_DIR_HANDLE ||
        g_dirs[dir_index].used == 0) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    return vfs_readdir(g_dirs[dir_index].vfs_handle, out_entry);
}

int32_t syscall_file_get_file_info(int32_t fd, vfs_file_t *file_out,
                                   uint32_t *writable_out)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used == 0 ||
        !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (g_files[fd].used == FILE_USED_MEMFD) {
        kernel_memfd_t *memfd = &g_memfds[fd];
        if (file_out != NULL) {
            memset(file_out, 0, sizeof(*file_out));
            file_out->size = memfd->size;
        }
        if (writable_out != NULL) *writable_out = 1u;
        return 0;
    }
    if (g_files[fd].used != FILE_USED_FILE) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    kernel_open_file_t *file = fd_open_file(fd);
    if (file == NULL) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (file_out != NULL) *file_out = file->file;
    if (writable_out != NULL) *writable_out = file->writable;
    return 0;
}

int32_t syscall_file_create_timerfd(void)
{
    int32_t current_pid = process_get_current_pid();
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    int32_t result = (int32_t)OS_STATUS_LIMIT_REACHED;
    for (int32_t fd = 3; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) {
            g_files[fd].used = FILE_USED_TIMERFD;
            g_files[fd].owner_pid = current_pid;
            g_files[fd].open_index = -1;
            g_files[fd].status_flags = 0;
            g_files[fd].descriptor_flags = 0;
            memset(&g_timerfds[fd], 0, sizeof(g_timerfds[fd]));
            g_timerfds[fd].used = 1;
            g_timerfds[fd].owner_pid = current_pid;
            result = fd;
            break;
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return result;
}

int32_t syscall_file_timerfd_settime(int32_t fd,
                                     uint64_t it_value_sec,
                                     uint64_t it_value_nsec,
                                     uint64_t it_interval_sec,
                                     uint64_t it_interval_nsec)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != FILE_USED_TIMERFD ||
        !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    kernel_timerfd_t *timerfd = &g_timerfds[fd];
    uint64_t interval_ms = it_interval_sec * 1000u + it_interval_nsec / 1000000u;
    if (it_value_sec == 0u && it_value_nsec == 0u) {
        timerfd->next_deadline_ms = 0;
        timerfd->interval_ms = 0;
        return 0;
    }
    timerfd->next_deadline_ms = timer_ms_now() +
        (it_value_sec * 1000u + it_value_nsec / 1000000u);
    timerfd->interval_ms = interval_ms;
    return 0;
}

int32_t syscall_file_timerfd_gettime(int32_t fd,
                                     uint64_t *it_value_sec_out,
                                     uint64_t *it_value_nsec_out,
                                     uint64_t *it_interval_sec_out,
                                     uint64_t *it_interval_nsec_out)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != FILE_USED_TIMERFD ||
        !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    kernel_timerfd_t *timerfd = &g_timerfds[fd];
    uint64_t now_ms = timer_ms_now();
    uint64_t remain_ms = 0;
    if (timerfd->interval_ms != 0u && timerfd->next_deadline_ms > now_ms) {
        remain_ms = timerfd->next_deadline_ms - now_ms;
    }
    if (it_value_sec_out != NULL) *it_value_sec_out = remain_ms / 1000u;
    if (it_value_nsec_out != NULL) *it_value_nsec_out = (remain_ms % 1000u) * 1000000u;
    if (it_interval_sec_out != NULL) *it_interval_sec_out = timerfd->interval_ms / 1000u;
    if (it_interval_nsec_out != NULL) {
        *it_interval_nsec_out = (timerfd->interval_ms % 1000u) * 1000000u;
    }
    return 0;
}

int32_t syscall_file_create_memfd(const char *name)
{
    (void)name;
    int32_t current_pid = process_get_current_pid();
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    int32_t result = (int32_t)OS_STATUS_LIMIT_REACHED;
    for (int32_t fd = 3; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) {
            g_files[fd].used = FILE_USED_MEMFD;
            g_files[fd].owner_pid = current_pid;
            g_files[fd].open_index = -1;
            g_files[fd].status_flags = FILE_O_RDWR;
            g_files[fd].descriptor_flags = 0;
            memset(&g_memfds[fd], 0, sizeof(g_memfds[fd]));
            g_memfds[fd].used = 1;
            g_memfds[fd].owner_pid = current_pid;
            g_memfds[fd].shm_handle = -1;
            result = fd;
            break;
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return result;
}

/* Handle of the shared-memory object backing memfd `fd` (owned by the
 * caller), or -1 if `fd` is not an shm-backed memfd. */
int32_t syscall_memfd_shm_handle(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD) {
        return -1;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    int32_t handle = -1;
    if (g_files[fd].used == FILE_USED_MEMFD &&
        g_files[fd].owner_pid == process_get_current_pid()) {
        handle = g_memfds[fd].shm_handle;
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return handle;
}

/* Install a fresh memfd fd in the current process wrapping an existing
 * shared-memory `handle`. The caller must have already ensured a
 * shared_memory reference is available for this fd to adopt (the SCM_RIGHTS
 * receiver path transfers the in-flight reference). Returns the new fd or a
 * negative os_status_t. */
int32_t syscall_memfd_install_shm(int32_t handle, uint32_t status_flags)
{
    int32_t current_pid = process_get_current_pid();
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    int32_t result = (int32_t)OS_STATUS_LIMIT_REACHED;
    for (int32_t fd = 3; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) {
            g_files[fd].used = FILE_USED_MEMFD;
            g_files[fd].owner_pid = current_pid;
            g_files[fd].open_index = -1;
            g_files[fd].status_flags = status_flags ? status_flags : FILE_O_RDWR;
            g_files[fd].descriptor_flags = 0;
            memset(&g_memfds[fd], 0, sizeof(g_memfds[fd]));
            g_memfds[fd].used = 1;
            g_memfds[fd].owner_pid = current_pid;
            g_memfds[fd].shm_handle = handle;
            g_memfds[fd].size = shared_memory_size(handle);
            result = fd;
            break;
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return result;
}

int32_t syscall_file_create_signalfd(uint64_t mask)
{
    int32_t current_pid = process_get_current_pid();
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    int32_t result = (int32_t)OS_STATUS_LIMIT_REACHED;
    for (int32_t fd = 3; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) {
            g_files[fd].used = FILE_USED_SIGNALFD;
            g_files[fd].owner_pid = current_pid;
            g_files[fd].open_index = -1;
            g_files[fd].status_flags = 0;
            g_files[fd].descriptor_flags = 0;
            memset(&g_signalfds[fd], 0, sizeof(g_signalfds[fd]));
            g_signalfds[fd].used = 1;
            g_signalfds[fd].owner_pid = current_pid;
            g_signalfds[fd].mask = mask;
            result = fd;
            break;
        }
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);
    return result;
}

int32_t syscall_file_signalfd_set_mask(int32_t fd, uint64_t mask)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != FILE_USED_SIGNALFD ||
        !fd_is_owned_by_current_process(fd)) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    g_signalfds[fd].mask = mask;
    return 0;
}

/* ---- character-device fds (/dev/dri/card0, /dev/input/event*) ------------ */

int32_t syscall_file_is_chardev(int32_t fd)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != 1 ||
        !fd_is_owned_by_current_process(fd)) {
        return 0;
    }
    kernel_open_file_t *file = fd_open_file(fd);
    return (file != NULL && vfs_file_is_chardev(&file->file)) ? 1 : 0;
}

int64_t syscall_file_ioctl(int32_t fd, uint64_t request, uint64_t arg)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != 1 ||
        !fd_is_owned_by_current_process(fd)) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    kernel_open_file_t *file = fd_open_file(fd);
    if (file == NULL) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    return vfs_dev_ioctl(&file->file, request, arg);
}

int64_t syscall_file_dev_mmap(int32_t fd, uint64_t offset, uint64_t length,
                              uint64_t prot, uint64_t flags)
{
    if (fd < 0 || fd >= FILE_MAX_FD || g_files[fd].used != 1 ||
        !fd_is_owned_by_current_process(fd)) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    kernel_open_file_t *file = fd_open_file(fd);
    if (file == NULL) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    return vfs_dev_mmap(&file->file, offset, length, prot, flags);
}

void syscall_file_fork_inherit(int32_t parent_pid, int32_t child_pid)
{
    if (parent_pid < 0 || child_pid < 0) {
        return;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_file_table_lock);
    for (int32_t fd = 0; fd < FILE_MAX_FD; ++fd) {
        if (g_files[fd].used == 0) continue;
        if (g_files[fd].owner_pid != parent_pid &&
            !fd_extra_owner_test(&g_files[fd], parent_pid)) {
            continue;
        }
        fd_extra_owner_set(&g_files[fd], child_pid);
    }
    spinlock_unlock(&g_file_table_lock);
    irq_restore(irq_flags);

    /* Directory handles are per-process and not shared: POSIX opendir() state
     * is not inherited in a useful way here, and nothing in tree relies on it. */
}
