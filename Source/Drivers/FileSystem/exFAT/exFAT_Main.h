#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "kernel/interfaces/fs_module_ops.h"

/* exFAT (Extended File Allocation Table) -- Microsoft specification
 * "exFAT File System Specification" (August 28, 2019). Read-only for this
 * first implementation; see Docs/Others/TODO_OS_Refactor.md 4. for the
 * staged rollout rationale (mirrors how ISO9660 shipped read-only-first,
 * see Kernel/Drivers/FileSystem/ISO9660/ISO9660_Main.h) and the write-support
 * follow-up phase. */

#define EXFAT_SECTOR_BUFFER_SIZE   4096u
#define EXFAT_PATH_MAX             512u
#define EXFAT_NAME_MAX             260u
#define EXFAT_DIR_HANDLE_MAX       256u

typedef struct {
    uint32_t first_cluster;
    uint64_t size;
    bool     no_fat_chain;      /* stream extension GeneralSecondaryFlags bit1 */
    bool     is_directory;
    uint32_t dir_cluster;       /* cluster the file's entry set starts in */
    uint32_t dir_entry_offset;  /* byte offset within dir_cluster of the 0x85 entry */
    uint8_t  secondary_count;
    char     name[EXFAT_NAME_MAX];
} exFAT_FILE;

typedef struct {
    char     name[EXFAT_NAME_MAX];
    uint64_t size;
    uint32_t first_cluster;
    uint8_t  is_directory;
} exFAT_DIRENT;

bool     exfat_init(void);

bool     exfat_find_file(const char *path, exFAT_FILE *out);
uint64_t exfat_get_file_size(exFAT_FILE *file);

bool     exfat_read_file(exFAT_FILE *file, uint8_t *buf);
bool     exfat_read_at(exFAT_FILE *file, uint32_t offset, uint8_t *buf, uint32_t size);

int32_t  exfat_opendir(const char *path);
int32_t  exfat_readdir(int32_t handle, exFAT_DIRENT *out);
int32_t  exfat_closedir(int32_t handle);

void     exfat_list_root_files(void);

/* The module exports a generic fs_module_ops_t (see
 * kernel/interfaces/fs_module_ops.h) as its driver_module_descriptor_t
 * .driver_api; FS_VFS_Bridge.c is the sole consumer. */
