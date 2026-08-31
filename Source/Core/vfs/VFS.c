#include "VFS.h"
#include <string.h>
#include "Core/sync/Spinlock.h"
#include "Debug/serial/Serial.h"

static vfs_driver_t g_vfs_drivers[16];
static int g_vfs_driver_count = 0;
static const vfs_driver_t *g_default_fs = NULL;

typedef struct {
    const vfs_driver_t *drv;
    int32_t driver_handle;
} vfs_directory_handle_t;

static vfs_directory_handle_t g_vfs_directory_handles[32];
static spinlock_t g_vfs_lock;

bool vfs_mount(const char *prefix, const vfs_driver_t *driver) {
    if (g_vfs_driver_count < 16) {
        g_vfs_drivers[g_vfs_driver_count] = *driver;
        g_vfs_drivers[g_vfs_driver_count].prefix = prefix;
        g_vfs_driver_count++;
        return true;
    }
    /* Mount table full: silently drop the driver.
     * (see Docs/Others/TODO_OS_Refactor.md 7.1) and let the caller know via
     * the return value. */
    return false;
}

/* Note: selection here is purely by driver identity/name, requested
 * explicitly by a caller that already has a specific name in hand (e.g. a
 * POSIX-style mount(2)). It intentionally does NOT drive any VFS-internal
 * boot-time policy -- see vfs_set_default_fs_by_kind() for that. */
bool vfs_set_default_fs(const char *fs_type) {
    for (int i = 0; i < g_vfs_driver_count; i++) {
        if (strcmp(g_vfs_drivers[i].fs_type, fs_type) == 0) {
            g_default_fs = &g_vfs_drivers[i];
            return true;
        }
    }
    return false;
}

bool vfs_set_default_fs_by_kind(vfs_media_kind_t kind) {
    for (int i = 0; i < g_vfs_driver_count; i++) {
        if (g_vfs_drivers[i].media_kind == kind) {
            g_default_fs = &g_vfs_drivers[i];
            return true;
        }
    }
    return false;
}

bool vfs_init(void) {
    spinlock_init(&g_vfs_lock);
    g_vfs_driver_count = 0;
    g_default_fs = NULL;
    memset(g_vfs_drivers, 0, sizeof(g_vfs_drivers));
    memset(g_vfs_directory_handles, 0, sizeof(g_vfs_directory_handles));
    return true;
}

static bool vfs_probe_file_on_driver(const vfs_driver_t *driver,
                                     const char *path,
                                     uint32_t min_size,
                                     const uint8_t *magic,
                                     uint32_t magic_size)
{
    if (driver == NULL || driver->find_file == NULL || path == NULL) {
        return false;
    }
    if (magic_size > 16u) {
        return false;
    }

    vfs_file_t file;
    memset(&file, 0, sizeof(file));
    if (!driver->find_file(path, &file)) {
        return false;
    }
    file.fs_driver = driver;

    bool ok = file.size >= min_size;
    if (ok && magic != NULL && magic_size != 0u) {
        uint8_t probe[16];
        memset(probe, 0, sizeof(probe));
        ok = driver->read_at != NULL &&
             driver->read_at(&file, 0u, probe, magic_size) &&
             memcmp(probe, magic, magic_size) == 0;
    }

    if (driver->close_file != NULL) {
        (void)driver->close_file(&file);
    }
    return ok;
}

bool vfs_set_default_fs_for_file(const char *path,
                                 uint32_t min_size,
                                 const uint8_t *magic,
                                 uint32_t magic_size)
{
    if (path == NULL || path[0] == '\0') {
        return false;
    }

    if (g_default_fs != NULL) {
        if (vfs_probe_file_on_driver(g_default_fs, path, min_size,
                                     magic, magic_size)) {
            return true;
        }
    }

    for (int i = 0; i < g_vfs_driver_count; i++) {
        const vfs_driver_t *driver = &g_vfs_drivers[i];
        if (driver == g_default_fs) {
            continue;
        }
        if (vfs_probe_file_on_driver(driver, path, min_size,
                                     magic, magic_size)) {
            g_default_fs = driver;
            return true;
        }
    }

    return false;
}

/*
 * vfs_resolve_candidates() -- the one place that implements "which mounted
 * driver(s) should handle this path", replacing five near-identical copies
 * of the same prefix-matching logic that used to live separately in
 * vfs_find_file/vfs_creat/vfs_mkdir/vfs_opendir/vfs_unlink (see
 * Docs/Others/TODO_OS_Refactor.md 7.2). Rule, unchanged from before this
 * refactor apart from one fix: the longest matching non-empty prefix wins
 * and is tried FIRST (possibly more than one driver mounted at exactly that
 * prefix, tried in mount order), but is no longer the *only* candidate -- the
 * default/catch-all drivers are appended so a prefix-mounted pseudo
 * filesystem cannot hide real files on the boot medium. With no prefix
 * match, the default filesystem
 * (see vfs_set_default_fs_by_kind()) is tried first, then every
 * empty-prefix ("catch-all") driver as a fallback. Each caller still owns
 * calling its own per-driver operation and deciding what "success" means
 * for that operation (a driver leaving a field NULL is a legitimate
 * "doesn't support this op", not a fault -- callers must NULL-check before
 * calling through a candidate's function pointer).
 */
#define VFS_MAX_CANDIDATES 16
#define VFS_PATH_MAX 256

/* Collapse duplicate '/' and drop trailing '/' (except for bare "/"). POSIX
 * treats "/a/b/" and "/a/b" as the same object for a directory, and callers
 * rely on it: Xorg's module loader probes candidate subdirectories with
 * stat("<dir>/<name>/") -- a trailing slash -- and recurses only when that
 * reports S_ISDIR. Without normalization every filesystem's path walker ran
 * off the end of the final component and returned "not found", so
 * modules/drivers, modules/extensions and modules/input were never searched
 * ("Failed to load module ... module does not exist").
 * Returns `path` itself when no rewrite is needed, else `buf`. */
static const char *vfs_normalize_path(const char *path, char *buf, size_t cap)
{
    if (path == NULL) {
        return NULL;
    }
    size_t len = strlen(path);
    if (len == 0u || len >= cap) {
        return path; /* nothing to do, or too long to rewrite safely */
    }
    /* Fast path: no "//" and no trailing '/' (or it is exactly "/"). */
    bool needs_fix = (len > 1u && path[len - 1u] == '/');
    if (!needs_fix) {
        for (size_t i = 1; i < len; ++i) {
            if (path[i] == '/' && path[i - 1u] == '/') {
                needs_fix = true;
                break;
            }
        }
    }
    if (!needs_fix) {
        return path;
    }

    size_t out = 0;
    for (size_t i = 0; i < len; ++i) {
        if (path[i] == '/' && out > 0u && buf[out - 1u] == '/') {
            continue; /* collapse runs of '/' */
        }
        buf[out++] = path[i];
    }
    while (out > 1u && buf[out - 1u] == '/') {
        --out; /* strip trailing '/', but keep a bare root "/" */
    }
    buf[out] = '\0';
    return buf;
}

static int vfs_resolve_candidates(const char *path,
                                  const vfs_driver_t *out[VFS_MAX_CANDIDATES])
{
    int n = 0;
    size_t best_match_len = 0;

    for (int i = 0; i < g_vfs_driver_count; i++) {
        size_t len = strlen(g_vfs_drivers[i].prefix);
        if (len > 0 && strncmp(path, g_vfs_drivers[i].prefix, len) == 0) {
            if (len > best_match_len) {
                best_match_len = len;
            }
        }
    }

    if (best_match_len > 0) {
        for (int i = 0; i < g_vfs_driver_count && n < VFS_MAX_CANDIDATES; i++) {
            if (strlen(g_vfs_drivers[i].prefix) == best_match_len &&
                strncmp(path, g_vfs_drivers[i].prefix, best_match_len) == 0) {
                out[n++] = &g_vfs_drivers[i];
            }
        }
        /* Deliberately NO early return: a prefix-mounted pseudo filesystem
         * takes priority for the names it owns but must not *hide* real files
         * living under the same prefix on the boot medium. EtcFS is mounted at
         * "/etc" and only knows a handful of synthesized files, yet the image
         * also ships /etc/X11/xorg.conf and /etc/fonts/fonts.conf -- with an
         * early return those were unreachable and Xorg fell back to its
         * built-in config ("Unable to locate/open config file").
         * See Docs/Others/TODO_Doom_Xorg_MethodA.md M6. */
    }

    if (g_default_fs && n < VFS_MAX_CANDIDATES) {
        out[n++] = g_default_fs;
    }
    for (int i = 0; i < g_vfs_driver_count && n < VFS_MAX_CANDIDATES; i++) {
        if (&g_vfs_drivers[i] == g_default_fs) continue;
        if (g_vfs_drivers[i].prefix[0] == '\0') {
            out[n++] = &g_vfs_drivers[i];
        }
    }
    return n;
}

bool vfs_find_file(const char *path, vfs_file_t *out_file) {
    char norm[VFS_PATH_MAX];
    path = vfs_normalize_path(path, norm, sizeof(norm));
    const vfs_driver_t *candidates[VFS_MAX_CANDIDATES];
    int n = vfs_resolve_candidates(path, candidates);
    for (int i = 0; i < n; i++) {
        if (candidates[i]->find_file && candidates[i]->find_file(path, out_file)) {
            out_file->fs_driver = candidates[i];
            return true;
        }
    }
    return false;
}

bool vfs_read_file(vfs_file_t *file, uint8_t *buffer) {
    if (!file || !file->fs_driver) return false;
    return file->fs_driver->read_file(file, buffer);
}

bool vfs_write_file(vfs_file_t *file, const uint8_t *buffer) {
    if (!file || !file->fs_driver) return false;
    return file->fs_driver->write_file(file, buffer);
}

bool vfs_read_at(vfs_file_t *file, uint32_t offset, uint8_t *buffer, uint32_t size) {
    if (!file || !file->fs_driver) return false;
    bool (*read_at)(vfs_file_t *, uint32_t, uint8_t *, uint32_t) = file->fs_driver->read_at;
    if (!read_at) {
        return false;
    }
    return read_at(file, offset, buffer, size);
}

bool vfs_file_is_chardev(const vfs_file_t *file) {
    if (!file || !file->fs_driver) return false;
    return file->fs_driver->dev_ioctl || file->fs_driver->dev_read ||
           file->fs_driver->dev_poll || file->fs_driver->dev_mmap;
}

int64_t vfs_dev_ioctl(vfs_file_t *file, uint64_t request, uint64_t arg) {
    if (!file || !file->fs_driver || !file->fs_driver->dev_ioctl) return -25;
    return file->fs_driver->dev_ioctl(file, request, arg);
}

int64_t vfs_dev_read(vfs_file_t *file, uint8_t *buffer, uint64_t length,
                     uint32_t nonblock) {
    if (!file || !file->fs_driver || !file->fs_driver->dev_read) return -25;
    return file->fs_driver->dev_read(file, buffer, length, nonblock);
}

uint32_t vfs_dev_poll(vfs_file_t *file, uint32_t events) {
    if (!file || !file->fs_driver || !file->fs_driver->dev_poll) return 0;
    return file->fs_driver->dev_poll(file, events);
}

int64_t vfs_dev_mmap(vfs_file_t *file, uint64_t offset, uint64_t length,
                     uint64_t prot, uint64_t flags) {
    if (!file || !file->fs_driver || !file->fs_driver->dev_mmap) return -25;
    return file->fs_driver->dev_mmap(file, offset, length, prot, flags);
}

bool vfs_write_at(vfs_file_t *file, uint32_t offset, const uint8_t *buffer, uint32_t size) {
    if (!file || !file->fs_driver) return false;
    return file->fs_driver->write_at(file, offset, buffer, size);
}

bool vfs_truncate(vfs_file_t *file, uint32_t new_size) {
    if (!file || !file->fs_driver || !file->fs_driver->truncate) return false;
    return file->fs_driver->truncate(file, new_size);
}

uint32_t vfs_get_file_size(vfs_file_t *file) {
    if (!file || !file->fs_driver) return 0;
    return file->fs_driver->get_file_size(file);
}

bool vfs_close_file(vfs_file_t *file) {
    if (!file || !file->fs_driver || !file->fs_driver->close_file) {
        return false;
    }
    bool result = file->fs_driver->close_file(file);
    file->driver_data = NULL;
    file->internal_id = 0;
    file->size = 0;
    file->fs_driver = NULL;
    return result;
}

bool vfs_creat(const char *path) {
    char norm[VFS_PATH_MAX];
    path = vfs_normalize_path(path, norm, sizeof(norm));
    const vfs_driver_t *candidates[VFS_MAX_CANDIDATES];
    int n = vfs_resolve_candidates(path, candidates);
    for (int i = 0; i < n; i++) {
        if (candidates[i]->creat && candidates[i]->creat(path)) return true;
    }
    return false;
}

bool vfs_mkdir(const char *path) {
    char norm[VFS_PATH_MAX];
    path = vfs_normalize_path(path, norm, sizeof(norm));
    const vfs_driver_t *candidates[VFS_MAX_CANDIDATES];
    int n = vfs_resolve_candidates(path, candidates);
    for (int i = 0; i < n; i++) {
        if (candidates[i]->mkdir && candidates[i]->mkdir(path)) return true;
    }
    return false;
}

int32_t vfs_opendir(const char *path) {
    const vfs_driver_t *drv = NULL;
    int32_t handle = -1;
    char norm[VFS_PATH_MAX];
    path = vfs_normalize_path(path, norm, sizeof(norm));

    const vfs_driver_t *candidates[VFS_MAX_CANDIDATES];
    int n = vfs_resolve_candidates(path, candidates);
    for (int i = 0; i < n; i++) {
        if (!candidates[i]->opendir) continue;
        handle = candidates[i]->opendir(path);
        if (handle >= 0) {
            drv = candidates[i];
            break;
        }
    }

    if (handle >= 0 && drv) {
        spinlock_lock(&g_vfs_lock);
        for (int i = 0; i < 32; i++) {
            if (g_vfs_directory_handles[i].drv == NULL) {
                g_vfs_directory_handles[i].drv = drv;
                g_vfs_directory_handles[i].driver_handle = handle;
                spinlock_unlock(&g_vfs_lock);
                return i;
            }
        }
        spinlock_unlock(&g_vfs_lock);
        drv->closedir(handle);
    }
    return -1;
}

bool vfs_dir_is_writable(const char *path) {
    char norm[VFS_PATH_MAX];
    path = vfs_normalize_path(path, norm, sizeof(norm));

    const vfs_driver_t *candidates[VFS_MAX_CANDIDATES];
    int n = vfs_resolve_candidates(path, candidates);
    for (int i = 0; i < n; i++) {
        if (!candidates[i]->opendir) continue;
        int32_t handle = candidates[i]->opendir(path);
        if (handle < 0) continue;
        /* Close through the driver, not vfs_closedir(): `handle` is the
         * driver's own handle, never registered in g_vfs_directory_handles. */
        if (candidates[i]->closedir) (void)candidates[i]->closedir(handle);
        return candidates[i]->creat != NULL;
    }
    return false;
}

int32_t vfs_readdir(int32_t handle, vfs_dirent_t *out_entry) {
    if (handle < 0 || handle >= 32) return -1;
    spinlock_lock(&g_vfs_lock);
    vfs_directory_handle_t *h = &g_vfs_directory_handles[handle];
    if (!h->drv) {
        spinlock_unlock(&g_vfs_lock);
        return -1;
    }
    const vfs_driver_t *drv = h->drv;
    int32_t driver_handle = h->driver_handle;
    spinlock_unlock(&g_vfs_lock);

    return drv->readdir(driver_handle, out_entry);
}

int32_t vfs_closedir(int32_t handle) {
    if (handle < 0 || handle >= 32) return -1;
    spinlock_lock(&g_vfs_lock);
    vfs_directory_handle_t *h = &g_vfs_directory_handles[handle];
    if (!h->drv) {
        spinlock_unlock(&g_vfs_lock);
        return -1;
    }
    const vfs_driver_t *drv = h->drv;
    int32_t driver_handle = h->driver_handle;
    h->drv = NULL;
    h->driver_handle = 0;
    spinlock_unlock(&g_vfs_lock);

    return drv->closedir(driver_handle);
}

bool vfs_unlink(const char *path) {
    const vfs_driver_t *candidates[VFS_MAX_CANDIDATES];
    int n = vfs_resolve_candidates(path, candidates);
    for (int i = 0; i < n; i++) {
        if (candidates[i]->unlink && candidates[i]->unlink(path)) return true;
    }
    return false;
}

bool vfs_rename(const char *old_path, const char *new_path)
{
    if (!old_path || !new_path || old_path[0] == '\0' ||
        new_path[0] == '\0') return false;
    if (strcmp(old_path, new_path) == 0) return true;

    vfs_file_t source;
    if (!vfs_find_file(old_path, &source)) return false;
    uint32_t size = vfs_get_file_size(&source);

    vfs_file_t existing;
    if (vfs_find_file(new_path, &existing)) {
        if (existing.fs_driver == source.fs_driver &&
            existing.internal_id == source.internal_id) {
            (void)vfs_close_file(&existing);
            (void)vfs_close_file(&source);
            return true;
        }
        (void)vfs_close_file(&existing);
        if (!vfs_unlink(new_path)) {
            (void)vfs_close_file(&source);
            return false;
        }
    }
    if (!vfs_creat(new_path)) {
        (void)vfs_close_file(&source);
        return false;
    }

    vfs_file_t destination;
    if (!vfs_find_file(new_path, &destination)) {
        (void)vfs_close_file(&source);
        (void)vfs_unlink(new_path);
        return false;
    }

    uint8_t buffer[4096];
    bool ok = true;
    for (uint32_t offset = 0; offset < size;) {
        uint32_t chunk = size - offset;
        if (chunk > sizeof(buffer)) chunk = sizeof(buffer);
        if (!vfs_read_at(&source, offset, buffer, chunk) ||
            !vfs_write_at(&destination, offset, buffer, chunk)) {
            ok = false;
            break;
        }
        offset += chunk;
    }
    if (ok) ok = vfs_truncate(&destination, size);
    (void)vfs_close_file(&source);
    (void)vfs_close_file(&destination);
    if (!ok) {
        (void)vfs_unlink(new_path);
        return false;
    }
    if (!vfs_unlink(old_path)) {
        (void)vfs_unlink(new_path);
        return false;
    }
    return true;
}

int vfs_driver_count_get(void) {
    return g_vfs_driver_count;
}

void vfs_list_root(void) {
    if (g_default_fs && g_default_fs->list_root) {
        g_default_fs->list_root();
    }
}

void vfs_set_case_sensitive(bool enabled) {
    if (g_default_fs && g_default_fs->set_case_sensitive) {
        g_default_fs->set_case_sensitive(enabled);
    }
}

bool vfs_get_case_sensitive(void) {
    if (g_default_fs && g_default_fs->get_case_sensitive) {
        return g_default_fs->get_case_sensitive();
    }
    return false;
}
