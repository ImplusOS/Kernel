#pragma once

/*
 * fs_module_ops_t -- the single structure every filesystem driver module
 * (FAT32_Driver.ELF, exFAT_Driver.ELF, ISO9660_Driver.ELF, ...) exports to
 * the kernel as its driver_module_descriptor_t.driver_api, replacing the
 * three bespoke per-filesystem vtables (fat32_driver_t / exfat_driver_t /
 * iso9660_driver_t) and the three near-identical kernel-resident
 * *_VFS_Bridge.c files that used to translate each of them into a
 * vfs_driver_t. There is now one kernel-side bridge,
 * Kernel/Drivers/Module/FS_VFS_Bridge.c, driven entirely by this struct.
 * See Docs/Others/TODO_OS_Refactor.md 6.2.
 *
 * Design points that let a single generic bridge do the job the per-fs
 * bridges used to:
 *
 *  - A file is an opaque `void *handle`. The module never sees vfs_file_t.
 *    The bridge allocates `handle_size` bytes before calling find_file()
 *    and frees them (after the optional release() hook) on close; the
 *    module just fills that buffer with whatever its private per-open file
 *    state is (the former FAT32_FILE / exFAT_FILE / ISO9660_FILE).
 *
 *  - readdir() fills a vfs_dirent_t directly, so the FAT-attribute /
 *    is_directory-flag / 64-bit-size-clamp conversions that used to live in
 *    the bridges move into the module, next to the on-disk format they
 *    describe.
 *
 *  - Every hook except find_file/init may be NULL. A read-only filesystem
 *    leaves the write side (write_file/write_at/truncate/creat/mkdir/unlink)
 *    NULL; the bridge reports failure for those exactly as the old
 *    read-only stubs did.
 *
 *  - `fs_type` / `media_kind` are copied into the vfs_driver_t the bridge
 *    hands to vfs_mount(), so VFS boot-time policy (vfs_set_default_fs* )
 *    keeps working unchanged.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "kernel/interfaces/vfs_dirent.h"

typedef struct fs_module_ops {
    const char      *fs_type;      /* human-readable label, e.g. "fat32" */
    vfs_media_kind_t media_kind;   /* VFS boot-time policy hint */
    uint32_t         handle_size;  /* sizeof the module's private per-open file handle */

    /* Resolve/mount the backing partition. Called (once, lazily) by the
     * bridge before any other hook; re-run if the module is reloaded. */
    bool (*init)(void);

    /* ---- path-based ---- */
    /* Populate the caller-provided `handle` (handle_size bytes) and report a
     * stable per-file identity in *out_id (used as st_ino by the Linux
     * compat layer -- must not collide between distinct files) and the file
     * size in *out_size. */
    bool (*find_file)(const char *path, void *handle,
                      uint64_t *out_id, uint32_t *out_size);
    bool (*creat)(const char *path);
    bool (*mkdir)(const char *path);
    bool (*unlink)(const char *path);

    /* ---- handle-based (handle == the buffer find_file filled) ---- */
    bool     (*read_file)(void *handle, uint8_t *buffer);
    bool     (*write_file)(void *handle, const uint8_t *buffer);
    bool     (*read_at)(void *handle, uint32_t offset, uint8_t *buffer, uint32_t size);
    bool     (*write_at)(void *handle, uint32_t offset, const uint8_t *buffer, uint32_t size);
    bool     (*truncate)(void *handle, uint32_t new_size);
    uint32_t (*get_file_size)(void *handle);
    void     (*release)(void *handle);  /* optional teardown before the bridge free()s handle */

    /* ---- directories ---- */
    int32_t (*opendir)(const char *path);
    int32_t (*readdir)(int32_t handle, vfs_dirent_t *out_entry);
    int32_t (*closedir)(int32_t handle);

    /* ---- misc (all optional) ---- */
    void (*list_root)(void);
    void (*set_case_sensitive)(bool enabled);
    bool (*get_case_sensitive)(void);
} fs_module_ops_t;

/*
 * Small helper for a module's readdir() shim: copy an on-disk name (which
 * may be longer than vfs_dirent_t.name and is not guaranteed to fit) into
 * out_entry->name, always NUL-terminated. Lives here rather than in each
 * module because driver modules link -nostdlib and only get memset/memcpy
 * from the kernel API -- no strncpy.
 */
static inline void fs_dirent_set_name(vfs_dirent_t *out_entry, const char *name)
{
    size_t i = 0;
    for (; i + 1u < sizeof(out_entry->name) && name[i] != '\0'; ++i) {
        out_entry->name[i] = name[i];
    }
    out_entry->name[i] = '\0';
}
