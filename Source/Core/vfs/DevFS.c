#include "DevFS.h"

#include <string.h>

#include "Core/process/ProcessManager.h"
#include "Core/timer/Timer.h"
#include "Crypto/Crypto.h"
#include "Debug/serial/Serial.h"
#include "Core/drm/DRM_Kms.h"
#include "Core/tty/Pty.h"
#include "kernel/config.h"
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
    /* pseudo-terminals. The pair index travels in the upper bits of
     * driver_data (see devfs_pack/devfs_kind_of) because, unlike every other
     * node here, these are per-open objects rather than one fixed device. */
    DEVFS_KIND_PTMX,          /* /dev/ptmx      -> pty master */
    DEVFS_KIND_PTS,           /* /dev/pts/N     -> pty slave */
    /* Kernel log. Like the pty nodes this is a per-open object: each reader
     * carries its own position in the log, packed into driver_data. */
    DEVFS_KIND_KMSG,          /* /dev/kmsg      -> Debug/serial ring */
    DEVFS_KIND_APPLOG,        /* /dev/applog    -> launch-log ring only */
    DEVFS_KIND_COUNT
} devfs_kind_t;

/* driver_data layout: kind in the low byte, (pty index + 1) above it, so 0
 * still means "no pty attached yet" for a /dev/ptmx node that has been looked
 * up but not opened. */
#define DEVFS_KIND_MASK   0xFFu
#define DEVFS_INDEX_SHIFT 8u

static void *devfs_pack(devfs_kind_t kind, int32_t pty_index)
{
    uintptr_t packed = (uintptr_t)kind & DEVFS_KIND_MASK;
    if (pty_index >= 0) {
        packed |= ((uintptr_t)pty_index + 1u) << DEVFS_INDEX_SHIFT;
    }
    return (void *)packed;
}

static devfs_kind_t devfs_kind_of(const vfs_file_t *file)
{
    if (file == NULL) {
        return DEVFS_KIND_COUNT;
    }
    return (devfs_kind_t)((uintptr_t)file->driver_data & DEVFS_KIND_MASK);
}

static int32_t devfs_pty_index_of(const vfs_file_t *file)
{
    if (file == NULL) {
        return -1;
    }
    uintptr_t packed = (uintptr_t)file->driver_data >> DEVFS_INDEX_SHIFT;
    return packed == 0u ? -1 : (int32_t)(packed - 1u);
}

static void devfs_set_pty_index(vfs_file_t *file, devfs_kind_t kind,
                                int32_t pty_index)
{
    if (file != NULL) {
        file->driver_data = devfs_pack(kind, pty_index);
    }
}

typedef struct {
    const char *name; /* full path, e.g. "/dev/null" */
    devfs_kind_t kind;
    uint32_t size;
    uint8_t is_char_device;
} devfs_entry_t;

#define DEVFS_STREAM_SIZE (64u * 1024u * 1024u)

/* O_NOCTTY, in the flag space open(2) hands the file layer. */
#define DEVFS_O_NOCTTY 0x0100u

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
    { "/dev/ptmx",             DEVFS_KIND_PTMX,         0u, 1u },
    { "/dev/kmsg",             DEVFS_KIND_KMSG,         0u, 1u },
    { "/dev/applog",           DEVFS_KIND_APPLOG,       0u, 1u },
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

/* Stand-in entry for the slave nodes, which are not in the static table:
 * "/dev/pts/N" is matched by prefix, and /dev/tty resolves onto it whenever
 * the caller has a controlling terminal. */
static const devfs_entry_t g_devfs_pts_entry = {
    "/dev/pts", DEVFS_KIND_PTS, 0u, 1u
};

static bool devfs_vfs_find_file(const char *path, vfs_file_t *out_file)
{
    /* Slave nodes first: they are dynamic, so the static table cannot hold
     * them. Existence is the pair being allocated and unlocked -- the same
     * test open("/dev/pts/N") makes on Linux. */
    int32_t pty_index = pty_index_from_path(path);
    if (pty_index >= 0) {
        if (!pty_slave_is_openable(pty_index)) {
            return false;
        }
        out_file->internal_id = (uint64_t)(uintptr_t)&g_devfs_pts_entry +
                                (uint64_t)pty_index;
        out_file->size = 0u;
        out_file->driver_data = devfs_pack(DEVFS_KIND_PTS, pty_index);
        return true;
    }

    const devfs_entry_t *entry = devfs_lookup(path);
    if (entry == NULL) {
        return false;
    }

    /* /dev/tty is whichever terminal the caller is sitting at: its pty when it
     * has one (a shell under xterm), the serial console otherwise. Resolved at
     * lookup time, which is when Linux resolves it too. */
    if (entry->kind == DEVFS_KIND_TTY) {
        int32_t ctty = pty_ctty_index_for_current();
        if (ctty >= 0) {
            out_file->internal_id = (uint64_t)(uintptr_t)&g_devfs_pts_entry +
                                    (uint64_t)ctty;
            out_file->size = 0u;
            out_file->driver_data = devfs_pack(DEVFS_KIND_PTS, ctty);
            return true;
        }
    }

    out_file->internal_id = (uint64_t)(uintptr_t)entry;
    out_file->size = entry->size;
    out_file->driver_data = devfs_pack(entry->kind, -1);
    return true;
}


/* ---- /dev/kmsg ------------------------------------------------------------
 *
 * Streams a kernel log ring (Debug/serial/Serial.c) to userland, so what
 * normally only reaches COM1 can be read from inside the OS -- the only way
 * to see it on a machine with no serial cable attached.
 *
 * Two nodes, same mechanics:
 *   /dev/kmsg    the whole kernel log
 *   /dev/applog  only the "[app] ..." Linux-program launch lines
 *
 * The split is not cosmetic. A terminal that streams the whole log into a
 * window feeds back on itself -- drawing a line is X traffic, X traffic is
 * logged, the log is drawn -- so the window that shows the log is the one
 * thing that must not read /dev/kmsg. Launch lines come only from exec and
 * exit, so nothing a terminal does can produce them.
 *
 * Each open gets its own position, so two readers do not steal bytes from
 * each other and a reader that starts late still sees the backlog. Reads are
 * "tail -f" shaped: return what is new, and when nothing is, wait rather than
 * report EOF. The wait is a plain timed sleep and deliberately *not* wired
 * into Poll_Wait.h -- serial_write_string() runs from inside interrupt
 * handlers and from under the process table lock, so waking schedulable
 * work from it would invert a lock order. 20 ms is well under the rate a
 * human reads at.
 */
#define DEVFS_KMSG_MAX_READERS 8u
#define DEVFS_KMSG_POLL_MS     20u
#define DEVFS_KMSG_BACKLOG_BYTES 4096u

typedef struct {
    uint8_t  used;
    uint8_t  applog;   /* 0 = whole kernel log, 1 = launch lines only */
    uint64_t cursor;
} devfs_kmsg_reader_t;

static devfs_kmsg_reader_t g_kmsg_readers[DEVFS_KMSG_MAX_READERS];

static int32_t devfs_kmsg_open(uint8_t applog)
{
    for (uint32_t i = 0; i < DEVFS_KMSG_MAX_READERS; ++i) {
        if (!g_kmsg_readers[i].used) {
            g_kmsg_readers[i].used = 1u;
            g_kmsg_readers[i].applog = applog;
            /* Open tail-first, with a page of backlog for context. A reader
             * is almost always opened *before* the thing it wants to watch
             * (the terminal is up, then an app is launched), so replaying the
             * whole 64 KiB boot log would push the interesting lines off the
             * screen before they arrive. Pass the whole ring by seeking to 0
             * -- lseek is not wired up for character devices, so in practice
             * this is the one policy, and it is the useful one. */
            uint64_t total = applog ? serial_applog_total() : serial_log_total();
            g_kmsg_readers[i].cursor =
                (total > DEVFS_KMSG_BACKLOG_BYTES)
                    ? (total - DEVFS_KMSG_BACKLOG_BYTES) : 0u;
            return (int32_t)i;
        }
    }
    return -1;
}

static void devfs_kmsg_close(int32_t index)
{
    if (index >= 0 && (uint32_t)index < DEVFS_KMSG_MAX_READERS) {
        g_kmsg_readers[index].used = 0u;
    }
}

static int devfs_kmsg_signal_is_pending(void)
{
    uint64_t pending = process_get_current_pending_signals();
    uint64_t masked = process_signal_get_mask();
    return (pending & ~masked) != 0u ? 1 : 0;
}

static int64_t devfs_kmsg_read(int32_t index, uint8_t *user_buffer,
                               uint64_t length, uint32_t nonblock)
{
    if (index < 0 || (uint32_t)index >= DEVFS_KMSG_MAX_READERS ||
        !g_kmsg_readers[index].used) {
        return -9; /* EBADF */
    }
    if (length == 0u) {
        return 0;
    }

    for (;;) {
        char staged[512];
        uint32_t want = (length < sizeof(staged)) ? (uint32_t)length
                                                  : (uint32_t)sizeof(staged);
        uint32_t got = g_kmsg_readers[index].applog
            ? serial_applog_read(&g_kmsg_readers[index].cursor, staged, want)
            : serial_read_log(&g_kmsg_readers[index].cursor, staged, want);
        if (got > 0u) {
            if (copy_to_user(user_buffer, staged, got) != 0u) {
                return -14; /* EFAULT */
            }
            return (int64_t)got;
        }
        if (nonblock != 0u) {
            return -11; /* EAGAIN */
        }
        if (devfs_kmsg_signal_is_pending()) {
            return -4; /* EINTR */
        }
        (void)process_sleep_current_ms(DEVFS_KMSG_POLL_MS);
    }
}

static uint32_t devfs_kmsg_poll(int32_t index, uint32_t events)
{
    if (index < 0 || (uint32_t)index >= DEVFS_KMSG_MAX_READERS ||
        !g_kmsg_readers[index].used) {
        return 0x20u; /* POLLNVAL */
    }
    uint64_t total = g_kmsg_readers[index].applog ? serial_applog_total()
                                                  : serial_log_total();
    uint32_t revents = 0u;
    if (g_kmsg_readers[index].cursor < total) {
        revents |= (events & 0x1u); /* POLLIN */
    }
    return revents;
}

/* Second half of open(2). A /dev/ptmx open allocates the pair here rather than
 * in find_file() so that stat()/access() probes -- which never close what they
 * look up -- cannot leak pairs. */
static void devfs_pty_trace(const char *what, int32_t value)
{
    if (!OS_CONFIG_FOREIGN_TRACE) {
        return;
    }
    serial_write_string("[pty] ");
    serial_write_string(what);
    serial_write_string(" ");
    serial_write_uint32((uint32_t)value);
    serial_write_char('\n');
}

static bool devfs_vfs_open_file(vfs_file_t *file, uint64_t flags)
{
    switch (devfs_kind_of(file)) {
        case DEVFS_KIND_PTMX: {
            int32_t index = pty_allocate();
            devfs_pty_trace("ptmx open -> pair", index);
            if (index < 0) {
                return false;
            }
            devfs_set_pty_index(file, DEVFS_KIND_PTMX, index);
            return true;
        }
        case DEVFS_KIND_PTS: {
            int32_t index = devfs_pty_index_of(file);
            devfs_pty_trace("pts open -> pair", index);
            if (index < 0 || pty_slave_acquire(index) < 0) {
                devfs_pty_trace("pts open REFUSED for pair", index);
                return false;
            }
            /* Opening a terminal without O_NOCTTY makes it the caller's
             * controlling terminal when it has none. xterm's child does this
             * explicitly with TIOCSCTTY, but a shell started any other way
             * relies on the open itself. */
            if ((flags & DEVFS_O_NOCTTY) == 0u &&
                pty_ctty_index_for_current() < 0) {
                pty_ctty_bind_current(index);
            }
            return true;
        }
        case DEVFS_KIND_KMSG:
        case DEVFS_KIND_APPLOG: {
            devfs_kind_t k = devfs_kind_of(file);
            int32_t index = devfs_kmsg_open(k == DEVFS_KIND_APPLOG ? 1u : 0u);
            if (index < 0) {
                return false; /* all reader slots taken */
            }
            devfs_set_pty_index(file, k, index);
            return true;
        }
        default:
            return true;
    }
}

static bool devfs_vfs_read_at(vfs_file_t *file, uint32_t offset,
                              uint8_t *buffer, uint32_t size)
{
    (void)offset;
    if (file == NULL || buffer == NULL) {
        return false;
    }
    devfs_kind_t kind = devfs_kind_of(file);
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
                               const uint8_t *buffer, uint32_t size);

static int64_t devfs_vfs_dev_write(vfs_file_t *file, const uint8_t *buffer,
                                   uint64_t length, uint32_t nonblock)
{
    if (file == NULL || buffer == NULL) return -14;
    devfs_kind_t kind = devfs_kind_of(file);
    if (kind == DEVFS_KIND_PTMX || kind == DEVFS_KIND_PTS) {
        return pty_write(devfs_pty_index_of(file), kind == DEVFS_KIND_PTMX,
                         buffer, length, nonblock);
    }
    /* Everything else keeps the all-or-nothing write_at path. */
    return devfs_vfs_write_at(file, 0u, buffer, (uint32_t)length) ?
        (int64_t)length : -5;
}

static bool devfs_vfs_write_at(vfs_file_t *file, uint32_t offset,
                               const uint8_t *buffer, uint32_t size)
{
    (void)offset;
    if (file == NULL) {
        return false;
    }
    devfs_kind_t kind = devfs_kind_of(file);
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
        devfs_kind_t kind = devfs_kind_of(file);
        if (kind == DEVFS_KIND_DRM_CARD || kind == DEVFS_KIND_DRM_RENDER) {
            drm_kms_close();
        } else if (kind == DEVFS_KIND_PTMX) {
            pty_master_release(devfs_pty_index_of(file));
        } else if (kind == DEVFS_KIND_PTS) {
            pty_slave_release(devfs_pty_index_of(file));
        } else if (kind == DEVFS_KIND_KMSG || kind == DEVFS_KIND_APPLOG) {
            devfs_kmsg_close(devfs_pty_index_of(file));
        }
    }
    return true;
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
    devfs_kind_t kind = devfs_kind_of(file);
    switch (kind) {
        case DEVFS_KIND_DRM_CARD:
        case DEVFS_KIND_DRM_RENDER:
            return drm_kms_ioctl(request, arg);
        case DEVFS_KIND_INPUT_EVENT0:
        case DEVFS_KIND_INPUT_EVENT1:
            return evdev_ioctl(devfs_evdev_fd(kind), request, arg);
        case DEVFS_KIND_PTMX:
        case DEVFS_KIND_PTS:
            return pty_ioctl(devfs_pty_index_of(file), kind == DEVFS_KIND_PTMX,
                             request, arg);
        default:
            return -25; /* ENOTTY */
    }
}

static int64_t devfs_vfs_dev_read(vfs_file_t *file, uint8_t *buffer,
                                  uint64_t length, uint32_t nonblock)
{
    if (file == NULL || buffer == NULL) return -14;
    devfs_kind_t kind = devfs_kind_of(file);
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
        case DEVFS_KIND_PTMX:
        case DEVFS_KIND_PTS:
            return pty_read(devfs_pty_index_of(file), kind == DEVFS_KIND_PTMX,
                            buffer, length, nonblock);
        case DEVFS_KIND_KMSG:
        case DEVFS_KIND_APPLOG:
            return devfs_kmsg_read(devfs_pty_index_of(file), buffer, length,
                                   nonblock);
        default:
            return -14;
    }
}

static uint32_t devfs_vfs_dev_poll(vfs_file_t *file, uint32_t events)
{
    if (file == NULL) return 0;
    devfs_kind_t kind = devfs_kind_of(file);
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
        case DEVFS_KIND_PTMX:
        case DEVFS_KIND_PTS:
            return pty_poll(devfs_pty_index_of(file), kind == DEVFS_KIND_PTMX,
                            events);
        case DEVFS_KIND_KMSG:
        case DEVFS_KIND_APPLOG:
            return devfs_kmsg_poll(devfs_pty_index_of(file), events);
        default:
            return 0;
    }
}

static int64_t devfs_vfs_dev_mmap(vfs_file_t *file, uint64_t offset,
                                  uint64_t length, uint64_t prot, uint64_t flags)
{
    if (file == NULL) return -25;
    devfs_kind_t kind = devfs_kind_of(file);
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
    .dev_write = devfs_vfs_dev_write,
    .open_file = devfs_vfs_open_file,
};

void devfs_init(void)
{
    memset(g_devfs_dir_handles, 0, sizeof(g_devfs_dir_handles));
    pty_init();
}

bool devfs_file_is_pty(const vfs_file_t *file)
{
    /* Identify devfs by one of its own hooks rather than by the address of
     * g_devfs_vfs_driver: vfs_mount() stores a *copy* of the driver struct
     * (so it can stamp the mount prefix into it), so a mounted file's
     * fs_driver never equals the address of the original -- an identity test
     * here silently answers "not a pty" for every pty in the system. */
    if (file == NULL || file->fs_driver == NULL ||
        file->fs_driver->dev_ioctl != devfs_vfs_dev_ioctl) {
        return false;
    }
    devfs_kind_t kind = devfs_kind_of(file);
    return kind == DEVFS_KIND_PTMX || kind == DEVFS_KIND_PTS;
}

const vfs_driver_t *devfs_vfs_get_driver(void)
{
    return &g_devfs_vfs_driver;
}

bool devfs_path_is_device(const char *path)
{
    return devfs_lookup(path) != NULL || pty_index_from_path(path) >= 0;
}
