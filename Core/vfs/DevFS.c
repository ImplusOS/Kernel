#include "DevFS.h"

#include <string.h>

#include "Core/process/ProcessManager.h"
#include "Core/timer/Timer.h"
#include "Crypto/Crypto.h"
#include "Debug/serial/Serial.h"
#include "Core/drm/DRM_Kms.h"
#include "Core/usercopy/Usercopy.h"
#include "Drivers/Module/Evdev_Client.h"

/* Devices exposed by this driver. Sizes are deliberately bounded (rather
 * than "infinite") because the generic VFS read path in Syscall_File.c
 * treats vfs_file_t.size as a hard EOF boundary (offset is a uint32_t).
 * A few hundred MB is far more than any realistic single fd lifetime will
 * consume (glibc/Chromium read getrandom()/tiny chunks from these, not
 * gigabytes), so this is a practical approximation of "unbounded". */
typedef enum {
    DEVFS_KIND_NULL = 0,
    DEVFS_KIND_ZERO,
    DEVFS_KIND_FULL,
    DEVFS_KIND_URANDOM,
    DEVFS_KIND_RANDOM,
    DEVFS_KIND_TTY,
    /* character devices for the foreign X-server path (Method A). These use
     * the vfs_driver_t dev_* hooks rather than read_at/write_at. */
    DEVFS_KIND_DRM_CARD,      /* /dev/dri/card0  -> DRM/KMS shim */
    DEVFS_KIND_DRM_RENDER,    /* /dev/dri/renderD128 -> same shim, render-only */
    DEVFS_KIND_INPUT_EVENT0,  /* /dev/input/event0 -> evdev keyboard */
    DEVFS_KIND_INPUT_EVENT1,  /* /dev/input/event1 -> evdev pointer */
    DEVFS_KIND_COUNT
} devfs_kind_t;

typedef struct {
    const char *name; /* full path, e.g. "/dev/null" */
    devfs_kind_t kind;
    uint32_t size;
    uint8_t is_char_device;
} devfs_entry_t;

#define DEVFS_STREAM_SIZE (64u * 1024u * 1024u)

static const devfs_entry_t g_devfs_entries[] = {
    { "/dev/null",    DEVFS_KIND_NULL,    0u,                1u },
    { "/dev/zero",    DEVFS_KIND_ZERO,    DEVFS_STREAM_SIZE, 1u },
    { "/dev/full",    DEVFS_KIND_FULL,    DEVFS_STREAM_SIZE, 1u },
    { "/dev/urandom", DEVFS_KIND_URANDOM, DEVFS_STREAM_SIZE, 1u },
    { "/dev/random",  DEVFS_KIND_RANDOM,  DEVFS_STREAM_SIZE, 1u },
    { "/dev/tty",     DEVFS_KIND_TTY,     0u,                1u },
    { "/dev/dri/card0",        DEVFS_KIND_DRM_CARD,     0u, 1u },
    { "/dev/dri/renderD128",   DEVFS_KIND_DRM_RENDER,   0u, 1u },
    { "/dev/input/event0",     DEVFS_KIND_INPUT_EVENT0, 0u, 1u },
    { "/dev/input/event1",     DEVFS_KIND_INPUT_EVENT1, 0u, 1u },
};

#define DEVFS_ENTRY_COUNT (sizeof(g_devfs_entries) / sizeof(g_devfs_entries[0]))

static const devfs_entry_t *devfs_lookup(const char *path)
{
    if (path == NULL) {
        return NULL;
    }
    for (uint32_t i = 0; i < DEVFS_ENTRY_COUNT; ++i) {
        if (strcmp(path, g_devfs_entries[i].name) == 0) {
            return &g_devfs_entries[i];
        }
    }
    return NULL;
}

static void devfs_fill_random(uint8_t *buffer, uint32_t length)
{
    static volatile uint64_t generation;
    uint32_t produced = 0;
    while (produced < length) {
        struct {
            uint64_t ticks;
            uint64_t generation;
            uint64_t cr3;
            const void *stack_address;
            int32_t pid;
            int32_t tid;
        } seed;
        seed.ticks = timer_ticks();
        seed.generation = __sync_add_and_fetch(&generation, 1u);
        seed.cr3 = process_get_current_cr3();
        seed.stack_address = &seed;
        seed.pid = process_get_current_pid();
        seed.tid = process_get_current_tid();

        uint8_t digest[32];
        crypto_sha256((const uint8_t *)&seed, sizeof(seed), digest);
        uint32_t chunk = length - produced;
        if (chunk > sizeof(digest)) {
            chunk = sizeof(digest);
        }
        memcpy(buffer + produced, digest, chunk);
        produced += chunk;
    }
}

static bool devfs_vfs_find_file(const char *path, vfs_file_t *out_file)
{
    const devfs_entry_t *entry = devfs_lookup(path);
    if (entry == NULL) {
        return false;
    }
    out_file->internal_id = (uint64_t)(uintptr_t)entry;
    out_file->size = entry->size;
    out_file->driver_data = (void *)(uintptr_t)entry->kind;
    return true;
}

static bool devfs_vfs_read_at(vfs_file_t *file, uint32_t offset,
                              uint8_t *buffer, uint32_t size)
{
    (void)offset;
    if (file == NULL || buffer == NULL) {
        return false;
    }
    devfs_kind_t kind = (devfs_kind_t)(uintptr_t)file->driver_data;
    switch (kind) {
        case DEVFS_KIND_NULL:
            return true; /* Reader already stops at size==0 (EOF). */
        case DEVFS_KIND_ZERO:
        case DEVFS_KIND_FULL:
            memset(buffer, 0, size);
            return true;
        case DEVFS_KIND_URANDOM:
        case DEVFS_KIND_RANDOM:
            devfs_fill_random(buffer, size);
            return true;
        case DEVFS_KIND_TTY:
            return true; /* No pending input; size==0 already yields EOF. */
        default:
            return false;
    }
}

static bool devfs_vfs_write_at(vfs_file_t *file, uint32_t offset,
                               const uint8_t *buffer, uint32_t size)
{
    (void)offset;
    if (file == NULL) {
        return false;
    }
    devfs_kind_t kind = (devfs_kind_t)(uintptr_t)file->driver_data;
    switch (kind) {
        case DEVFS_KIND_NULL:
        case DEVFS_KIND_ZERO:
        case DEVFS_KIND_URANDOM:
        case DEVFS_KIND_RANDOM:
            return true; /* Discard silently, like Linux. */
        case DEVFS_KIND_FULL:
            return false; /* Always ENOSPC, like Linux. */
        case DEVFS_KIND_TTY:
            if (buffer != NULL) {
                for (uint32_t i = 0; i < size; ++i) {
                    serial_write_char((char)buffer[i]);
                }
            }
            return true;
        default:
            return false;
    }
}

static bool devfs_vfs_read_file(vfs_file_t *file, uint8_t *buffer)
{
    return devfs_vfs_read_at(file, 0, buffer, file != NULL ? file->size : 0u);
}

static bool devfs_vfs_write_file(vfs_file_t *file, const uint8_t *buffer)
{
    return devfs_vfs_write_at(file, 0, buffer, file != NULL ? file->size : 0u);
}

static bool devfs_vfs_truncate(vfs_file_t *file, uint32_t new_size)
{
    (void)file;
    (void)new_size;
    return true;
}

static uint32_t devfs_vfs_get_file_size(vfs_file_t *file)
{
    return file != NULL ? file->size : 0u;
}

static bool devfs_vfs_creat(const char *path)
{
    (void)path;
    return false; /* Device nodes are fixed; no dynamic creation. */
}

static bool devfs_vfs_mkdir(const char *path)
{
    (void)path;
    return false;
}

typedef struct {
    uint8_t in_use;
    uint32_t cursor;
} devfs_dir_handle_t;

#define DEVFS_DIR_HANDLE_MAX 8
static devfs_dir_handle_t g_devfs_dir_handles[DEVFS_DIR_HANDLE_MAX];

static int32_t devfs_vfs_opendir(const char *path)
{
    if (path == NULL || (strcmp(path, "/dev") != 0 && strcmp(path, "/dev/") != 0)) {
        return -1;
    }
    for (int32_t i = 0; i < DEVFS_DIR_HANDLE_MAX; ++i) {
        if (!g_devfs_dir_handles[i].in_use) {
            g_devfs_dir_handles[i].in_use = 1;
            g_devfs_dir_handles[i].cursor = 0;
            return i;
        }
    }
    return -1;
}

static int32_t devfs_vfs_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    if (handle < 0 || handle >= DEVFS_DIR_HANDLE_MAX ||
        !g_devfs_dir_handles[handle].in_use || out_entry == NULL) {
        return -1;
    }
    uint32_t cursor = g_devfs_dir_handles[handle].cursor;
    if (cursor >= DEVFS_ENTRY_COUNT) {
        return 0;
    }
    const devfs_entry_t *entry = &g_devfs_entries[cursor];
    const char *base_name = entry->name + 5; /* skip "/dev/" */
    strncpy(out_entry->name, base_name, sizeof(out_entry->name) - 1);
    out_entry->name[sizeof(out_entry->name) - 1] = '\0';
    out_entry->size = entry->size;
    out_entry->is_directory = false;
    g_devfs_dir_handles[handle].cursor = cursor + 1u;
    return 1;
}

static int32_t devfs_vfs_closedir(int32_t handle)
{
    if (handle < 0 || handle >= DEVFS_DIR_HANDLE_MAX) {
        return -1;
    }
    g_devfs_dir_handles[handle].in_use = 0;
    return 0;
}

static bool devfs_vfs_close_file(vfs_file_t *file)
{
    if (file != NULL) {
        devfs_kind_t kind = (devfs_kind_t)(uintptr_t)file->driver_data;
        if (kind == DEVFS_KIND_DRM_CARD || kind == DEVFS_KIND_DRM_RENDER) {
            drm_kms_close();
        }
    }
    return true; /* No per-open allocation to release. */
}

/* ---- character-device hooks (DRM / evdev) --------------------------------- */

static int32_t devfs_evdev_fd(devfs_kind_t kind)
{
    if (kind == DEVFS_KIND_INPUT_EVENT0) return EVDEV_FD_BASE + 0;
    if (kind == DEVFS_KIND_INPUT_EVENT1) return EVDEV_FD_BASE + 1;
    return -1;
}

static int64_t devfs_vfs_dev_ioctl(vfs_file_t *file, uint64_t request, uint64_t arg)
{
    if (file == NULL) return -25;
    devfs_kind_t kind = (devfs_kind_t)(uintptr_t)file->driver_data;
    switch (kind) {
        case DEVFS_KIND_DRM_CARD:
        case DEVFS_KIND_DRM_RENDER:
            return drm_kms_ioctl(request, arg);
        case DEVFS_KIND_INPUT_EVENT0:
        case DEVFS_KIND_INPUT_EVENT1:
            return evdev_ioctl(devfs_evdev_fd(kind), request, arg);
        default:
            return -25; /* ENOTTY */
    }
}

static int64_t devfs_vfs_dev_read(vfs_file_t *file, uint8_t *buffer,
                                  uint64_t length, uint32_t nonblock)
{
    if (file == NULL || buffer == NULL) return -14;
    devfs_kind_t kind = (devfs_kind_t)(uintptr_t)file->driver_data;
    switch (kind) {
        case DEVFS_KIND_DRM_CARD:
        case DEVFS_KIND_DRM_RENDER:
            return drm_kms_read(buffer, length, nonblock);
        case DEVFS_KIND_INPUT_EVENT0:
        case DEVFS_KIND_INPUT_EVENT1: {
            /* evdev_read() writes its dest directly; `buffer` here is a user
             * pointer (linux_read passes it straight through). Bounce. */
            uint8_t tmp[24 * 32];
            uint64_t want = length < sizeof(tmp) ? length : sizeof(tmp);
            int64_t n = evdev_read(devfs_evdev_fd(kind), tmp, want);
            if (n <= 0) return n;
            if (copy_to_user(buffer, tmp, (uint64_t)n) != 0u) return -14;
            return n;
        }
        default:
            return -14;
    }
}

static uint32_t devfs_vfs_dev_poll(vfs_file_t *file, uint32_t events)
{
    if (file == NULL) return 0;
    devfs_kind_t kind = (devfs_kind_t)(uintptr_t)file->driver_data;
    switch (kind) {
        case DEVFS_KIND_DRM_CARD:
        case DEVFS_KIND_DRM_RENDER:
            return drm_kms_poll(events);
        case DEVFS_KIND_INPUT_EVENT0:
        case DEVFS_KIND_INPUT_EVENT1:
            /* Only readable when the ring actually holds an event. Claiming
             * POLLIN unconditionally makes every select()/poll() loop that
             * watches an input device spin: the caller is told there is data,
             * reads, gets nothing, and immediately polls again. X's main loop
             * (WaitForSomething) and its input thread both do exactly that. */
            return (evdev_has_events(devfs_evdev_fd(kind)) != 0)
                       ? (events & 0x1u) : 0u;
        default:
            return 0;
    }
}

static int64_t devfs_vfs_dev_mmap(vfs_file_t *file, uint64_t offset,
                                  uint64_t length, uint64_t prot, uint64_t flags)
{
    if (file == NULL) return -25;
    devfs_kind_t kind = (devfs_kind_t)(uintptr_t)file->driver_data;
    if (kind == DEVFS_KIND_DRM_CARD || kind == DEVFS_KIND_DRM_RENDER) {
        return drm_kms_mmap(offset, length, prot, flags);
    }
    return -25;
}

static bool devfs_vfs_unlink(const char *path)
{
    (void)path;
    return false;
}

static void devfs_vfs_list_root(void)
{
}

static void devfs_vfs_set_case_sensitive(bool enabled)
{
    (void)enabled;
}

static bool devfs_vfs_get_case_sensitive(void)
{
    return true;
}

static const vfs_driver_t g_devfs_vfs_driver = {
    .fs_type = "devfs",
    .media_kind = VFS_MEDIA_KIND_PSEUDO,
    .prefix = NULL,
    .find_file = devfs_vfs_find_file,
    .read_file = devfs_vfs_read_file,
    .write_file = devfs_vfs_write_file,
    .read_at = devfs_vfs_read_at,
    .write_at = devfs_vfs_write_at,
    .truncate = devfs_vfs_truncate,
    .get_file_size = devfs_vfs_get_file_size,
    .creat = devfs_vfs_creat,
    .mkdir = devfs_vfs_mkdir,
    .opendir = devfs_vfs_opendir,
    .readdir = devfs_vfs_readdir,
    .closedir = devfs_vfs_closedir,
    .close_file = devfs_vfs_close_file,
    .unlink = devfs_vfs_unlink,
    .list_root = devfs_vfs_list_root,
    .set_case_sensitive = devfs_vfs_set_case_sensitive,
    .get_case_sensitive = devfs_vfs_get_case_sensitive,
    .dev_ioctl = devfs_vfs_dev_ioctl,
    .dev_read = devfs_vfs_dev_read,
    .dev_poll = devfs_vfs_dev_poll,
    .dev_mmap = devfs_vfs_dev_mmap,
};

void devfs_init(void)
{
    memset(g_devfs_dir_handles, 0, sizeof(g_devfs_dir_handles));
}

const vfs_driver_t *devfs_vfs_get_driver(void)
{
    return &g_devfs_vfs_driver;
}

bool devfs_path_is_device(const char *path)
{
    return devfs_lookup(path) != NULL;
}
