#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Host stand-in: the harness exercises the in-RAM log ring only, so file
 * logging is stubbed out to "there is no filesystem". */
typedef struct { uint64_t internal_id; uint32_t size; void *driver_data;
                 void *fs_driver; } vfs_file_t;
static inline bool vfs_creat(const char *path) { (void)path; return false; }
static inline bool vfs_find_file(const char *p, vfs_file_t *f)
{ (void)p; (void)f; return false; }
static inline bool vfs_write_at(vfs_file_t *f, uint32_t off,
                                const uint8_t *b, uint32_t n)
{ (void)f; (void)off; (void)b; (void)n; return false; }
