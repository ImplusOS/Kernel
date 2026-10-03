#pragma once

#include <stdint.h>
#include "Core/vfs/VFS.h"
#include "Core/sync/Spinlock.h"

/* Process ABI modes (from ProcessScheduler.h). */
#define PROCESS_ABI_IMPLUS 0
#define PROCESS_ABI_LINUX  1

/* File table types - shared between Syscall_File.c, ProcessScheduler.h, ProcessManager_Create.c */

typedef struct {
    uint8_t used;
    int32_t owner_pid;
    int32_t open_index;
    uint32_t status_flags;
    uint32_t descriptor_flags;
    uint64_t extra_owners[32];
} kernel_file_t;

typedef struct {
    uint8_t used;
    uint8_t writable;
    vfs_file_t file;
    uint32_t offset;
    uint32_t refcount;
    uint8_t cache_valid;
    uint8_t *cache_data;
    uint32_t cache_size;
    uint32_t cache_offset;
    spinlock_t io_lock;
} kernel_open_file_t;

#define FILE_DIR_PATH_MAX 256u
typedef struct {
    uint8_t used;
    int32_t owner_pid;
    int32_t vfs_handle;
    char path[FILE_DIR_PATH_MAX];
} kernel_dir_t;

#define PIPE_BUF_SIZE 4096u
typedef struct {
    uint8_t in_use;
    uint8_t data[PIPE_BUF_SIZE];
    uint16_t read_pos;
    uint16_t write_pos;
    uint16_t count;
    uint16_t reader_count;
    uint16_t writer_count;
    spinlock_t lock;
} kernel_pipe_t;

typedef struct {
    uint8_t used;
    int32_t owner_pid;
    uint64_t next_deadline_ms;
    uint64_t interval_ms;
} kernel_timerfd_t;

typedef struct {
    uint8_t used;
    uint8_t *data;
    uint32_t size;
    uint32_t capacity;
    uint32_t offset;
    int32_t shm_handle;
    uint32_t seals;
    uint32_t refs;
} kernel_memfd_t;

typedef struct {
    uint8_t used;
    int32_t owner_pid;
    uint64_t mask;
} kernel_signalfd_t;

/* FILE_USED_* constants for g_files[].used */
enum {
    FILE_USED_FILE     = 1,
    FILE_USED_PIPE_R   = 2,
    FILE_USED_PIPE_W   = 3,
    FILE_USED_DIR      = 4,
    FILE_USED_TIMERFD  = 5,
    FILE_USED_MEMFD    = 6,
    FILE_USED_SIGNALFD = 7,
};