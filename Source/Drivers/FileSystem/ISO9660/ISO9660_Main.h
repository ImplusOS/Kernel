#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "kernel/interfaces/fs_module_ops.h"

#define ISO9660_SECTOR_SIZE                 2048u
#define ISO9660_PATH_MAX                    512u
#define ISO9660_NAME_MAX                    260u
#define ISO9660_DIR_HANDLE_MAX              256u
#define ISO9660_SECTOR_BUFFER_SIZE          8192u

typedef struct {
    uint32_t extent;
    uint32_t size;
    uint8_t  is_dir;
    uint8_t  is_symlink;
    uint32_t dir_extent;
    uint32_t dir_offset;
    char     name[ISO9660_NAME_MAX];
} ISO9660_FILE;

typedef struct {
    char     name[ISO9660_NAME_MAX];
    uint32_t size;
    uint32_t extent;
    uint8_t  is_directory;
} ISO9660_DIRENT;

typedef struct {
    uint32_t pvd_extent;
    uint32_t root_extent;
    uint32_t root_size;
    uint32_t vol_space_size;
    uint16_t logical_block_size;
    bool     has_joliet;
    uint32_t joliet_root_extent;
    uint32_t joliet_root_size;
    bool     has_rock_ridge;
    uint8_t  rr_susp_skip;
} ISO9660_CONTEXT;

bool     iso9660_init(void);
bool     iso9660_find_file(const char *path, ISO9660_FILE *out);
uint32_t iso9660_get_file_size(ISO9660_FILE *file);
bool     iso9660_read_file(ISO9660_FILE *file, uint8_t *buf);
bool     iso9660_read_at(ISO9660_FILE *file, uint32_t offset, uint8_t *buf, uint32_t size);
int32_t  iso9660_opendir(const char *path);
int32_t  iso9660_readdir(int32_t handle, ISO9660_DIRENT *out);
int32_t  iso9660_closedir(int32_t handle);
void     iso9660_list_root_files(void);

/* The module exports a generic fs_module_ops_t (see
 * kernel/interfaces/fs_module_ops.h) as its driver_module_descriptor_t
 * .driver_api; FS_VFS_Bridge.c is the sole consumer. */
