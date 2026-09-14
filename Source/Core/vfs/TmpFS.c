#include "TmpFS.h"

#include <stdlib.h>
#include <string.h>

#include "Core/sync/Spinlock.h"

/* One table serves all four mounts below, so it has to hold everything at
 * once: Xorg's sockets and compiled keymaps, the fontconfig cache, the login
 * session files, and a whole browser profile. 256 was exhausted while
 * Chromium created its profile: the next file creation failed, the syscall
 * layer reported EIO, and whichever database was being opened at that moment
 * broke ("GCM Store/LOCK: OS or hardware error", "Failed to load tokens
 * (invalid SQL statement)"), which is the "Something went wrong when opening
 * your profile" dialog. A slot is ~272 bytes with file data allocated
 * separately, so 4096 costs about 1.1 MB, and lookups only compare slots in
 * use. */
#define TMPFS_MAX_FILES 4096u
#define TMPFS_PATH_MAX  256u
#define TMPFS_PREFIX    "/dev/shm"

/* TmpFS backs several mount points with one flat, path-keyed namespace:
 *   /dev/shm  - POSIX shm (wl_shm, glibc)
 *   /tmp      - X server needs this writable and unchangeable: the X11 unix
 *               socket dir /tmp/.X11-unix and lock files /tmp/.X{n}-lock are
 *               compile-time paths in libX11/Xserver. Also HOME / XDG_* point
 *               here (fontconfig cache, xkb cache, ...).
 *   /run      - misc runtime state some Linux libs insist on.
 * Directories are implicit (mkdir is a no-op success); a path is a valid
 * tmpfs path iff it starts with one of these prefixes followed by '/' or end. */
/* "/var" is here for the same reason a diskless/live Linux keeps /var on
 * tmpfs: the boot medium is read-only ISO9660, but programs still expect a
 * writable /var. Xorg specifically picks its compiled-keymap output directory
 * by probing access("/var/lib/xkb", W_OK|X_OK) (ddxLoad.c OutputDirectory),
 * and without a writable one it cannot run xkbcomp at all -- which is fatal
 * ("Failed to activate virtual core keyboard"). Directories are implicit in
 * this flat namespace, so /var/lib/xkb needs no mkdir. */
static const char *const TMPFS_PREFIXES[] = { "/dev/shm", "/tmp", "/run", "/var" };

static bool tmpfs_path_ok(const char *path)
{
    if (path == NULL) {
        return false;
    }
    for (uint32_t i = 0; i < sizeof(TMPFS_PREFIXES) / sizeof(TMPFS_PREFIXES[0]); ++i) {
        size_t plen = strlen(TMPFS_PREFIXES[i]);
        if (strncmp(path, TMPFS_PREFIXES[i], plen) == 0 &&
            (path[plen] == '\0' || path[plen] == '/')) {
            return true;
        }
    }
    return false;
}

/* Default permission bits, matching what the syscall layer reported before
 * tmpfs kept modes of its own. */
#define TMPFS_MODE_FILE 0644u
#define TMPFS_MODE_DIR  0755u

typedef struct {
    uint8_t in_use;
    /* A symlink is an ordinary slot whose content is the target path. Kept in
     * the same table (rather than a second one) so unlink/readdir/find_file
     * need no special case, and so a link and a file can never share a name. */
    uint8_t is_symlink;
    /* mkdir() records the directory rather than succeeding blindly, because a
     * mode has to live somewhere: mkdtemp() creates 0700 and Chromium then
     * CHECK()s stat() reports 0700. Lookups stay independent of this -- a path
     * under a tmpfs prefix is still a valid directory whether or not anybody
     * mkdir'ed it -- so this only adds modes and readdir visibility. */
    uint8_t is_dir;
    uint32_t mode;
    char path[TMPFS_PATH_MAX];
    uint8_t *data;
    uint32_t size;
    uint32_t capacity;
} tmpfs_slot_t;

static tmpfs_slot_t g_tmpfs_slots[TMPFS_MAX_FILES];

/* How full the shared table gets. Logged at each new multiple-of-32 high-water
 * mark, and once if it ever fills, so a boot says whether the limit is still
 * in reach. Callers hold g_tmpfs_lock. */
extern void serial_write_string(const char *str);
extern void serial_write_uint32(uint32_t value);
static uint32_t g_tmpfs_slots_used;
static uint32_t g_tmpfs_slots_peak;
static bool g_tmpfs_full_reported;

static void tmpfs_note_claim_locked(void)
{
    ++g_tmpfs_slots_used;
    if (g_tmpfs_slots_used > g_tmpfs_slots_peak) {
        g_tmpfs_slots_peak = g_tmpfs_slots_used;
        if ((g_tmpfs_slots_peak % 32u) == 0u) {
            serial_write_string("[tmpfs] slots in use reached ");
            serial_write_uint32(g_tmpfs_slots_peak);
            serial_write_string(" of ");
            serial_write_uint32(TMPFS_MAX_FILES);
            serial_write_string("\n");
        }
    }
}

static void tmpfs_note_release_locked(void)
{
    if (g_tmpfs_slots_used > 0u) {
        --g_tmpfs_slots_used;
    }
}

static void tmpfs_note_full_locked(const char *path)
{
    if (g_tmpfs_full_reported) {
        return;
    }
    g_tmpfs_full_reported = true;
    serial_write_string("[tmpfs] table full (");
    serial_write_uint32(TMPFS_MAX_FILES);
    serial_write_string(" slots) creating ");
    serial_write_string(path);
    serial_write_string("\n");
}
static spinlock_t g_tmpfs_lock;

static tmpfs_slot_t *tmpfs_find_locked(const char *path)
{
    for (uint32_t i = 0; i < TMPFS_MAX_FILES; ++i) {
        if (g_tmpfs_slots[i].in_use &&
            strncmp(g_tmpfs_slots[i].path, path, TMPFS_PATH_MAX) == 0) {
            return &g_tmpfs_slots[i];
        }
    }
    return NULL;
}

static bool tmpfs_vfs_find_file(const char *path, vfs_file_t *out_file)
{
    if (!tmpfs_path_ok(path)) {
        return false;
    }
    spinlock_lock(&g_tmpfs_lock);
    tmpfs_slot_t *slot = tmpfs_find_locked(path);
    spinlock_unlock(&g_tmpfs_lock);
    if (slot == NULL || slot->is_dir) {
        return false; /* a directory is not openable as a file */
    }
    out_file->internal_id = (uint64_t)(uintptr_t)slot;
    out_file->size = slot->size;
    out_file->driver_data = slot;
    return true;
}

static bool tmpfs_ensure_capacity_locked(tmpfs_slot_t *slot, uint32_t required)
{
    if (required <= slot->capacity) {
        return true;
    }
    uint32_t new_capacity = slot->capacity == 0u ? 4096u : slot->capacity;
    while (new_capacity < required) {
        new_capacity *= 2u;
    }
    uint8_t *resized = (uint8_t *)realloc(slot->data, new_capacity);
    if (resized == NULL) {
        return false;
    }
    memset(resized + slot->capacity, 0, new_capacity - slot->capacity);
    slot->data = resized;
    slot->capacity = new_capacity;
    return true;
}

static bool tmpfs_vfs_read_at(vfs_file_t *file, uint32_t offset,
                              uint8_t *buffer, uint32_t size)
{
    if (file == NULL || file->driver_data == NULL || buffer == NULL) {
        return false;
    }
    tmpfs_slot_t *slot = (tmpfs_slot_t *)file->driver_data;
    spinlock_lock(&g_tmpfs_lock);
    if (offset > slot->size || size > slot->size - offset) {
        spinlock_unlock(&g_tmpfs_lock);
        return false;
    }
    memcpy(buffer, slot->data + offset, size);
    spinlock_unlock(&g_tmpfs_lock);
    return true;
}

static bool tmpfs_vfs_write_at(vfs_file_t *file, uint32_t offset,
                               const uint8_t *buffer, uint32_t size)
{
    if (file == NULL || file->driver_data == NULL) {
        return false;
    }
    tmpfs_slot_t *slot = (tmpfs_slot_t *)file->driver_data;
    spinlock_lock(&g_tmpfs_lock);
    uint64_t end = (uint64_t)offset + (uint64_t)size;
    if (end > 0xFFFFFFFFu || !tmpfs_ensure_capacity_locked(slot, (uint32_t)end)) {
        spinlock_unlock(&g_tmpfs_lock);
        return false;
    }
    if (buffer != NULL && size != 0u) {
        memcpy(slot->data + offset, buffer, size);
    }
    if ((uint32_t)end > slot->size) {
        slot->size = (uint32_t)end;
    }
    file->size = slot->size;
    spinlock_unlock(&g_tmpfs_lock);
    return true;
}

static bool tmpfs_vfs_read_file(vfs_file_t *file, uint8_t *buffer)
{
    return tmpfs_vfs_read_at(file, 0, buffer, file != NULL ? file->size : 0u);
}

static bool tmpfs_vfs_write_file(vfs_file_t *file, const uint8_t *buffer)
{
    return tmpfs_vfs_write_at(file, 0, buffer, file != NULL ? file->size : 0u);
}

static bool tmpfs_vfs_truncate(vfs_file_t *file, uint32_t new_size)
{
    if (file == NULL || file->driver_data == NULL) {
        return false;
    }
    tmpfs_slot_t *slot = (tmpfs_slot_t *)file->driver_data;
    spinlock_lock(&g_tmpfs_lock);
    if (new_size > slot->capacity &&
        !tmpfs_ensure_capacity_locked(slot, new_size)) {
        spinlock_unlock(&g_tmpfs_lock);
        return false;
    }
    if (new_size > slot->size) {
        memset(slot->data + slot->size, 0, new_size - slot->size);
    }
    slot->size = new_size;
    file->size = new_size;
    spinlock_unlock(&g_tmpfs_lock);
    return true;
}

static uint32_t tmpfs_vfs_get_file_size(vfs_file_t *file)
{
    return file != NULL ? file->size : 0u;
}

static bool tmpfs_vfs_creat(const char *path)
{
    if (!tmpfs_path_ok(path) || strlen(path) >= TMPFS_PATH_MAX) {
        return false;
    }
    spinlock_lock(&g_tmpfs_lock);
    if (tmpfs_find_locked(path) != NULL) {
        spinlock_unlock(&g_tmpfs_lock);
        return true; /* Already exists: O_CREAT without O_EXCL is fine. */
    }
    for (uint32_t i = 0; i < TMPFS_MAX_FILES; ++i) {
        if (!g_tmpfs_slots[i].in_use) {
            memset(&g_tmpfs_slots[i], 0, sizeof(g_tmpfs_slots[i]));
            g_tmpfs_slots[i].in_use = 1;
            tmpfs_note_claim_locked();
            g_tmpfs_slots[i].mode = TMPFS_MODE_FILE;
            strncpy(g_tmpfs_slots[i].path, path, TMPFS_PATH_MAX - 1u);
            spinlock_unlock(&g_tmpfs_lock);
            return true;
        }
    }
    tmpfs_note_full_locked(path);
    spinlock_unlock(&g_tmpfs_lock);
    return false;
}

static bool tmpfs_vfs_mkdir(const char *path)
{
    /* Flat namespace: lookups treat directories as implicit, so success here
     * never depended on a node existing -- X's mkdir("/tmp/.X11-unix", 01777)
     * and friends must not abort. A node is still recorded when there is room,
     * so the directory has somewhere to keep its mode and shows up in a
     * readdir() of its parent; running out of slots degrades to the old
     * behaviour rather than failing the call. */
    if (!tmpfs_path_ok(path) || strlen(path) >= TMPFS_PATH_MAX) {
        return false;
    }
    spinlock_lock(&g_tmpfs_lock);
    if (tmpfs_find_locked(path) == NULL) {
        bool recorded = false;
        for (uint32_t i = 0; i < TMPFS_MAX_FILES; ++i) {
            if (g_tmpfs_slots[i].in_use) {
                continue;
            }
            memset(&g_tmpfs_slots[i], 0, sizeof(g_tmpfs_slots[i]));
            g_tmpfs_slots[i].in_use = 1;
            tmpfs_note_claim_locked();
            g_tmpfs_slots[i].is_dir = 1;
            g_tmpfs_slots[i].mode = TMPFS_MODE_DIR;
            strncpy(g_tmpfs_slots[i].path, path, TMPFS_PATH_MAX - 1u);
            recorded = true;
            break;
        }
        if (!recorded) {
            tmpfs_note_full_locked(path);
        }
    }
    spinlock_unlock(&g_tmpfs_lock);
    return true;
}

/* chmod(2), and the mode mkdir(2)/open(O_CREAT) asked for -- the VFS create
 * hooks carry no mode, so the syscall layer applies it here afterwards. */
static bool tmpfs_vfs_set_mode(const char *path, uint32_t mode)
{
    if (!tmpfs_path_ok(path)) {
        return false;
    }
    spinlock_lock(&g_tmpfs_lock);
    tmpfs_slot_t *slot = tmpfs_find_locked(path);
    if (slot != NULL) {
        slot->mode = mode & 07777u;
    }
    spinlock_unlock(&g_tmpfs_lock);
    return slot != NULL;
}

static int32_t tmpfs_vfs_get_mode(const char *path)
{
    if (!tmpfs_path_ok(path)) {
        return -1;
    }
    spinlock_lock(&g_tmpfs_lock);
    tmpfs_slot_t *slot = tmpfs_find_locked(path);
    int32_t mode = slot != NULL ? (int32_t)slot->mode : -1;
    spinlock_unlock(&g_tmpfs_lock);
    return mode;
}

#define TMPFS_DIR_HANDLE_MAX 8u
static uint8_t g_tmpfs_dir_in_use[TMPFS_DIR_HANDLE_MAX];
static uint32_t g_tmpfs_dir_cursor[TMPFS_DIR_HANDLE_MAX];
static char g_tmpfs_dir_path[TMPFS_DIR_HANDLE_MAX][TMPFS_PATH_MAX];

/* Is `path` a directory in this flat namespace? One of the mount roots, a
 * node mkdir() recorded, or an ancestor of something that exists (a file can
 * be created at /tmp/a/b without anyone mkdir'ing /tmp/a).
 *
 * Answering "yes" for every path under a tmpfs prefix -- which is what this
 * used to do -- is not a harmless simplification: opendir() is how the syscall
 * layer decides a path exists at all when find_file() misses, so stat() of any
 * absent /tmp path reported a directory, readlink() of a missing SingletonLock
 * reported EINVAL instead of ENOENT, and unlink() of one reported EIO. */
static bool tmpfs_is_dir_locked(const char *path)
{
    size_t len = strlen(path);
    while (len > 1u && path[len - 1u] == '/') {
        --len;
    }
    for (uint32_t i = 0; i < sizeof(TMPFS_PREFIXES) / sizeof(TMPFS_PREFIXES[0]); ++i) {
        size_t plen = strlen(TMPFS_PREFIXES[i]);
        if (plen == len && strncmp(path, TMPFS_PREFIXES[i], plen) == 0) {
            return true;
        }
    }
    for (uint32_t i = 0; i < TMPFS_MAX_FILES; ++i) {
        if (!g_tmpfs_slots[i].in_use) {
            continue;
        }
        const char *p = g_tmpfs_slots[i].path;
        if (strncmp(p, path, len) != 0) {
            continue;
        }
        if (p[len] == '\0') {
            return g_tmpfs_slots[i].is_dir != 0u; /* the node itself */
        }
        if (p[len] == '/') {
            return true; /* something lives under it */
        }
    }
    return false;
}

static int32_t tmpfs_vfs_opendir(const char *path)
{
    if (!tmpfs_path_ok(path) || strlen(path) >= TMPFS_PATH_MAX) {
        return -1;
    }
    spinlock_lock(&g_tmpfs_lock);
    if (!tmpfs_is_dir_locked(path)) {
        spinlock_unlock(&g_tmpfs_lock);
        return -1;
    }
    for (uint32_t i = 0; i < TMPFS_DIR_HANDLE_MAX; ++i) {
        if (!g_tmpfs_dir_in_use[i]) {
            g_tmpfs_dir_in_use[i] = 1;
            g_tmpfs_dir_cursor[i] = 0;
            strncpy(g_tmpfs_dir_path[i], path, TMPFS_PATH_MAX - 1u);
            g_tmpfs_dir_path[i][TMPFS_PATH_MAX - 1u] = '\0';
            /* strip a single trailing '/' for uniform prefix matching */
            size_t l = strlen(g_tmpfs_dir_path[i]);
            if (l > 1u && g_tmpfs_dir_path[i][l - 1u] == '/') {
                g_tmpfs_dir_path[i][l - 1u] = '\0';
            }
            spinlock_unlock(&g_tmpfs_lock);
            return (int32_t)i;
        }
    }
    spinlock_unlock(&g_tmpfs_lock);
    return -1;
}

static int32_t tmpfs_vfs_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    if (handle < 0 || (uint32_t)handle >= TMPFS_DIR_HANDLE_MAX || out_entry == NULL) {
        return -1;
    }
    spinlock_lock(&g_tmpfs_lock);
    if (!g_tmpfs_dir_in_use[handle]) {
        spinlock_unlock(&g_tmpfs_lock);
        return -1;
    }
    const char *dir = g_tmpfs_dir_path[handle];
    size_t dirlen = strlen(dir);
    uint32_t cursor = g_tmpfs_dir_cursor[handle];
    for (; cursor < TMPFS_MAX_FILES; ++cursor) {
        if (!g_tmpfs_slots[cursor].in_use) {
            continue;
        }
        const char *p = g_tmpfs_slots[cursor].path;
        /* direct child of `dir` only: dir + '/' + name, name has no '/' */
        if (strncmp(p, dir, dirlen) != 0 || p[dirlen] != '/') {
            continue;
        }
        const char *base_name = p + dirlen + 1u;
        if (base_name[0] == '\0' || strchr(base_name, '/') != NULL) {
            continue;
        }
        strncpy(out_entry->name, base_name, sizeof(out_entry->name) - 1u);
        out_entry->name[sizeof(out_entry->name) - 1u] = '\0';
        out_entry->size = g_tmpfs_slots[cursor].size;
        out_entry->is_directory = g_tmpfs_slots[cursor].is_dir != 0u;
        g_tmpfs_dir_cursor[handle] = cursor + 1u;
        spinlock_unlock(&g_tmpfs_lock);
        return 1;
    }
    g_tmpfs_dir_cursor[handle] = cursor;
    spinlock_unlock(&g_tmpfs_lock);
    return 0;
}

static int32_t tmpfs_vfs_closedir(int32_t handle)
{
    if (handle < 0 || (uint32_t)handle >= TMPFS_DIR_HANDLE_MAX) {
        return -1;
    }
    spinlock_lock(&g_tmpfs_lock);
    g_tmpfs_dir_in_use[handle] = 0;
    spinlock_unlock(&g_tmpfs_lock);
    return 0;
}

static bool tmpfs_vfs_close_file(vfs_file_t *file)
{
    (void)file;
    return true; /* Backing slot persists across opens. */
}

static bool tmpfs_vfs_unlink(const char *path)
{
    spinlock_lock(&g_tmpfs_lock);
    tmpfs_slot_t *slot = tmpfs_find_locked(path);
    if (slot == NULL) {
        spinlock_unlock(&g_tmpfs_lock);
        return false;
    }
    if (slot->data != NULL) {
        free(slot->data);
    }
    memset(slot, 0, sizeof(*slot));
    tmpfs_note_release_locked();
    spinlock_unlock(&g_tmpfs_lock);
    return true;
}

/* symlink(2). Chromium's ProcessSingleton creates <user-data-dir>/SingletonLock
 * (-> "hostname-pid"), SingletonCookie and SingletonSocket with symlink() and
 * aborts the whole browser when that fails, so the writable tmpfs trees have
 * to carry real links rather than pretend. */
static bool tmpfs_vfs_symlink(const char *target, const char *linkpath)
{
    if (target == NULL || linkpath == NULL ||
        !tmpfs_path_ok(linkpath) || strlen(linkpath) >= TMPFS_PATH_MAX) {
        return false;
    }
    size_t target_len = strlen(target);
    if (target_len == 0u || target_len >= TMPFS_PATH_MAX) {
        return false;
    }
    spinlock_lock(&g_tmpfs_lock);
    if (tmpfs_find_locked(linkpath) != NULL) {
        spinlock_unlock(&g_tmpfs_lock); /* EEXIST -- symlink(2) never replaces */
        return false;
    }
    for (uint32_t i = 0; i < TMPFS_MAX_FILES; ++i) {
        if (g_tmpfs_slots[i].in_use) {
            continue;
        }
        memset(&g_tmpfs_slots[i], 0, sizeof(g_tmpfs_slots[i]));
        if (!tmpfs_ensure_capacity_locked(&g_tmpfs_slots[i],
                                          (uint32_t)target_len)) {
            spinlock_unlock(&g_tmpfs_lock);
            return false;
        }
        g_tmpfs_slots[i].in_use = 1;
        tmpfs_note_claim_locked();
        g_tmpfs_slots[i].is_symlink = 1;
        g_tmpfs_slots[i].mode = 0777u; /* symlinks are lrwxrwxrwx on Linux */
        strncpy(g_tmpfs_slots[i].path, linkpath, TMPFS_PATH_MAX - 1u);
        memcpy(g_tmpfs_slots[i].data, target, target_len);
        g_tmpfs_slots[i].size = (uint32_t)target_len;
        spinlock_unlock(&g_tmpfs_lock);
        return true;
    }
    tmpfs_note_full_locked(linkpath);
    spinlock_unlock(&g_tmpfs_lock);
    return false;
}

/* readlink(2): the target is NOT NUL-terminated and the count is the return
 * value. -1 means "not a symlink here", which lets the VFS try the next
 * candidate driver rather than reporting EINVAL for someone else's path. */
static int32_t tmpfs_vfs_readlink(const char *path, char *buf, uint32_t size)
{
    if (path == NULL || buf == NULL || size == 0u || !tmpfs_path_ok(path)) {
        return -1;
    }
    spinlock_lock(&g_tmpfs_lock);
    tmpfs_slot_t *slot = tmpfs_find_locked(path);
    if (slot == NULL || !slot->is_symlink) {
        spinlock_unlock(&g_tmpfs_lock);
        return -1;
    }
    uint32_t n = slot->size < size ? slot->size : size;
    memcpy(buf, slot->data, n);
    spinlock_unlock(&g_tmpfs_lock);
    return (int32_t)n;
}

static void tmpfs_vfs_list_root(void)
{
}

static void tmpfs_vfs_set_case_sensitive(bool enabled)
{
    (void)enabled;
}

static bool tmpfs_vfs_get_case_sensitive(void)
{
    return true;
}

static const vfs_driver_t g_tmpfs_vfs_driver = {
    .fs_type = "tmpfs",
    .media_kind = VFS_MEDIA_KIND_PSEUDO,
    .prefix = NULL,
    .find_file = tmpfs_vfs_find_file,
    .read_file = tmpfs_vfs_read_file,
    .write_file = tmpfs_vfs_write_file,
    .read_at = tmpfs_vfs_read_at,
    .write_at = tmpfs_vfs_write_at,
    .truncate = tmpfs_vfs_truncate,
    .get_file_size = tmpfs_vfs_get_file_size,
    .creat = tmpfs_vfs_creat,
    .mkdir = tmpfs_vfs_mkdir,
    .opendir = tmpfs_vfs_opendir,
    .readdir = tmpfs_vfs_readdir,
    .closedir = tmpfs_vfs_closedir,
    .close_file = tmpfs_vfs_close_file,
    .unlink = tmpfs_vfs_unlink,
    .list_root = tmpfs_vfs_list_root,
    .set_case_sensitive = tmpfs_vfs_set_case_sensitive,
    .get_case_sensitive = tmpfs_vfs_get_case_sensitive,
    .symlink = tmpfs_vfs_symlink,
    .readlink = tmpfs_vfs_readlink,
    .set_mode = tmpfs_vfs_set_mode,
    .get_mode = tmpfs_vfs_get_mode,
};

/* Directories that have to be there before anyone creates anything in them.
 * The mount roots themselves are handled by tmpfs_is_dir_locked(); these are
 * the deeper ones a program probes rather than creates -- Xorg picks its
 * compiled-keymap output directory with access("/var/lib/xkb", W_OK|X_OK) and
 * cannot run xkbcomp without one, which is a fatal "Failed to activate virtual
 * core keyboard". */
static const char *const TMPFS_SEED_DIRS[] = {
    "/tmp/.X11-unix",
    "/var/lib",
    "/var/lib/xkb",
    "/var/tmp",
    "/run/user",
};

void tmpfs_init(void)
{
    spinlock_init(&g_tmpfs_lock);
    memset(g_tmpfs_slots, 0, sizeof(g_tmpfs_slots));
    memset(g_tmpfs_dir_in_use, 0, sizeof(g_tmpfs_dir_in_use));
    memset(g_tmpfs_dir_cursor, 0, sizeof(g_tmpfs_dir_cursor));
    memset(g_tmpfs_dir_path, 0, sizeof(g_tmpfs_dir_path));
    for (uint32_t i = 0;
         i < sizeof(TMPFS_SEED_DIRS) / sizeof(TMPFS_SEED_DIRS[0]); ++i) {
        (void)tmpfs_vfs_mkdir(TMPFS_SEED_DIRS[i]);
    }
}

const vfs_driver_t *tmpfs_vfs_get_driver(void)
{
    return &g_tmpfs_vfs_driver;
}
