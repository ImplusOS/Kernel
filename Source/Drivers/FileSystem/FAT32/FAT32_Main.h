#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "kernel/interfaces/fs_module_ops.h"

#define FAT32_MAX_SECTOR_SIZE       4096u
#define FAT32_CLUSTER_BUFFER_SIZE   65536u
#define FAT32_PATH_MAX              512u
#define FAT32_NAME_MAX              260u

#pragma pack(push, 1)
typedef struct {
    uint16_t bytes_per_sector;
    uint8_t  sectors_per_cluster;
    uint8_t  _reserved0;
    uint16_t reserved_sectors;
    uint8_t  num_fats;
    uint8_t  _reserved1;
    uint32_t fat_size_sectors;
    uint32_t root_cluster;
    uint16_t fs_info_sector;
    uint16_t backup_boot_sector;
    uint32_t total_sectors;
} FAT32_BPB;
#pragma pack(pop)

typedef struct {
    uint32_t first_cluster;
    uint32_t size;
    uint32_t dir_entry_sector;
    uint32_t dir_cluster;
    uint16_t dir_entry_offset;
    uint8_t  attributes;
    uint8_t  lfn_entry_count;
    char     name[FAT32_NAME_MAX];
    uint32_t cached_cluster_index;
    uint32_t cached_cluster_value;
} FAT32_FILE;

typedef struct {
    char     name[FAT32_NAME_MAX];
    uint32_t size;
    uint32_t first_cluster;
    uint8_t  attributes;
} FAT32_DIRENT;

bool     fat32_init();

bool     fat32_find_file(const char *path, FAT32_FILE *out);
uint32_t fat32_get_file_size(FAT32_FILE *file);

bool     fat32_read_file(FAT32_FILE *file, uint8_t *buf);
bool     fat32_write_file(FAT32_FILE *file, const uint8_t *buf);
bool     fat32_read_at(FAT32_FILE *file, uint32_t offset, uint8_t *buf, uint32_t size);
bool     fat32_write_at(FAT32_FILE *file, uint32_t offset, const uint8_t *buf, uint32_t size);
bool     fat32_truncate(FAT32_FILE *file, uint32_t new_size);

bool     fat32_creat(const char *path);
bool     fat32_mkdir(const char *path);
bool     fat32_unlink(const char *path);

int32_t  fat32_opendir(const char *path);
int32_t  fat32_readdir(int32_t handle, FAT32_DIRENT *out);
int32_t  fat32_closedir(int32_t handle);

void     fat32_list_root_files(void);

void     fat32_set_case_sensitive_lookup(bool enabled);
bool     fat32_get_case_sensitive_lookup(void);

typedef struct {
    uint32_t cluster_index;
    uint32_t cluster_value;
} fat32_cache_t;

extern fat32_cache_t g_cluster_cache;

/* The module exports a generic fs_module_ops_t (see
 * kernel/interfaces/fs_module_ops.h) as its driver_module_descriptor_t
 * .driver_api; FS_VFS_Bridge.c is the sole consumer. */
