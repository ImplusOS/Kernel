#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "kernel/interfaces/fs_module_ops.h"

/*
 * UDF (ECMA-167 / OSTA UDF) read-only filesystem driver module.
 *
 * Targets the "UDF bridge" images the top-level Makefile now masters
 * (genisoimage -udf): an ISO9660 volume that also carries a full UDF
 * structure describing the same files. The kernel prefers this driver over
 * the ISO9660 one for optical media (lower load_priority), and falls back
 * to ISO9660 automatically if udf_init() cannot find a valid UDF volume.
 *
 * Scope: enough of UDF 1.02 / 2.01 to walk directories and read regular
 * files produced by a mastering tool -- Type 1 (physical) partition maps,
 * ICB strategy 4, short_ad / long_ad allocation, embedded (in-ICB) file
 * data, Latin-1 and UCS-2 file identifiers. No write side, no metadata
 * partition / sparable / virtual maps, no named streams.
 */

#define UDF_SECTOR_SIZE        2048u
#define UDF_NAME_MAX           256u
#define UDF_MAX_EXTENTS        96u
#define UDF_EMBED_MAX          2048u
#define UDF_DIR_HANDLE_MAX     32u
#define UDF_MAX_PARTITIONS     4u

typedef struct {
    uint32_t lba;   /* volume-relative LBA, 2048-byte units */
    uint32_t len;   /* byte length of this extent                */
} udf_extent_t;

typedef struct {
    uint64_t     size;
    uint32_t     fe_lba;        /* volume-relative LBA of the File Entry -- stable id */
    uint8_t      is_dir;
    uint8_t      embedded;      /* file data stored inline in the File Entry */
    uint32_t     extent_count;
    udf_extent_t extents[UDF_MAX_EXTENTS];
    uint32_t     embed_len;
    uint8_t      embed[UDF_EMBED_MAX];
} UDF_FILE;

typedef struct {
    char     name[UDF_NAME_MAX];
    uint64_t size;
    uint8_t  is_directory;
} UDF_DIRENT;

bool     udf_init(void);
bool     udf_find_file(const char *path, UDF_FILE *out);
bool     udf_read_at(UDF_FILE *file, uint32_t offset, uint8_t *buf, uint32_t size);
bool     udf_read_file(UDF_FILE *file, uint8_t *buf);
uint32_t udf_get_file_size(UDF_FILE *file);
int32_t  udf_opendir(const char *path);
int32_t  udf_readdir(int32_t handle, UDF_DIRENT *out);
int32_t  udf_closedir(int32_t handle);
void     udf_list_root(void);
