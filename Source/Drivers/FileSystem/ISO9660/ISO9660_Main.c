#include "Drivers/FileSystem/ISO9660/ISO9660_Main.h"
#include "Debug/serial/Serial.h"
#include "kernel/config.h"
#include "Platform/io/IO_Main.h"
#ifndef IMPLUS_DRIVER_MODULE
#include "interfaces/hal_cpu.h"
#endif
#include <string.h>
#include <stddef.h>

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"

static const driver_binary_t *g_driver_api = NULL;

#define hal_cpu_pause               g_driver_api->hal.cpu_pause
#define disk_read                   g_driver_api->hw.disk_read
#define disk_write                  g_driver_api->hw.disk_write
#define disk_get_partition_lba      g_driver_api->hw.disk_get_partition_lba
#define serial_write_string          g_driver_api->dbg.write_string
#define serial_write_uint32             g_driver_api->dbg.write_uint32

typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_init(spinlock_t *l)   { l->locked = 0; }
static inline void spinlock_lock(spinlock_t *l)   {
    while (__sync_lock_test_and_set(&l->locked, 1)) {
        while (l->locked) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->locked); }

void *memcpy(void *dest, const void *src, size_t n)
{
    uint8_t *d = (uint8_t *)dest;
    const uint8_t *s = (const uint8_t *)src;
    while (n--) *d++ = *s++;
    return dest;
}

void *memset(void *s, int c, size_t n)
{
    uint8_t *p = (uint8_t *)s;
    while (n--) *p++ = (uint8_t)c;
    return s;
}

int memcmp(const void *s1, const void *s2, size_t n) {
    const unsigned char *p1 = s1, *p2 = s2;
    for (size_t i = 0; i < n; i++)
        if (p1[i] != p2[i]) return p1[i] - p2[i];
    return 0;
}
int strcmp(const char *s1, const char *s2) {
    while (*s1 && (*s1 == *s2)) { s1++; s2++; }
    return *(const unsigned char*)s1 - *(const unsigned char*)s2;
}
char *strcpy(char *dest, const char *src) {
    char *d = dest;
    while ((*d++ = *src++) != '\0');
    return dest;
}

#else
#include "Core/sync/Spinlock.h"
#endif

#define ISO9660_SIGNATURE   "CD001"
#define ISO9660_VERSION     1
#define ISO9660_ATTR_DIR    0x02u

#define VD_TYPE_PVD         1u
#define VD_TYPE_SVD         2u
#define VD_TYPE_TERMINATOR  255u

static const uint8_t k_joliet_esc[3][3] = {
    {0x25, 0x2F, 0x40},
    {0x25, 0x2F, 0x43},
    {0x25, 0x2F, 0x45},
};

#define SUSP_SP_CHECK1  0xBEu
#define SUSP_SP_CHECK2  0xEFu

static ISO9660_CONTEXT g_iso_context;

static uint8_t g_iso_sector_buffer[ISO9660_SECTOR_BUFFER_SIZE] __attribute__((aligned(4096)));
static uint8_t g_iso_read_buffer  [ISO9660_SECTOR_BUFFER_SIZE] __attribute__((aligned(4096)));
static uint8_t g_iso_ce_buffer    [ISO9660_SECTOR_BUFFER_SIZE] __attribute__((aligned(4096)));

static spinlock_t g_iso_lock;
static uint64_t g_iso_partition_lba = 0;

typedef struct {
    uint8_t  used;
    uint32_t current_extent;
    uint32_t remaining_size;
    uint32_t entry_offset;
    bool     joliet;
} iso9660_dir_handle_t;

static iso9660_dir_handle_t g_dir_handles[ISO9660_DIR_HANDLE_MAX];

static inline uint16_t iso9660_read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t iso9660_read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline uint32_t iso9660_read_u32_both(const uint8_t *p) {
    return iso9660_read_u32_le(p);
}

static bool iso9660_read_sector(uint32_t lba, uint8_t *buffer) {
    uint64_t base = g_iso_partition_lba +
                    (uint64_t)lba * (ISO9660_SECTOR_SIZE / 512u);
    return disk_read(base, buffer, ISO9660_SECTOR_SIZE / 512u);
}

static void ucs2be_to_utf8(const uint8_t *src, uint32_t src_bytes,
                             char *dst, uint32_t dst_max) {
    uint32_t di = 0;
    for (uint32_t i = 0; i + 1 < src_bytes && di + 4 < dst_max; i += 2) {
        uint16_t cp = ((uint16_t)src[i] << 8) | src[i + 1];
        if (cp == 0x0000u) break;
        if (cp >= 0xD800u && cp <= 0xDFFFu) continue;
        if (cp < 0x0080u) {
            dst[di++] = (char)(cp & 0x7Fu);
        } else if (cp < 0x0800u) {
            dst[di++] = (char)(0xC0u | (cp >> 6));
            dst[di++] = (char)(0x80u | (cp & 0x3Fu));
        } else {
            dst[di++] = (char)(0xE0u | (cp >> 12));
            dst[di++] = (char)(0x80u | ((cp >> 6) & 0x3Fu));
            dst[di++] = (char)(0x80u | (cp & 0x3Fu));
        }
    }
    dst[di] = '\0';
}

static void iso9660_strip_version(char *name) {
    if (!name || !*name) return;
    char *p = name;
    while (*p) p++;
    char *end = p;
    while (end > name && (unsigned char)(end[-1] - '0') <= 9u) end--;
    if (end > name && end[-1] == ';') {
        end--;
        if (end > name && end[-1] == '.') end--;
        *end = '\0';
    }
}

static inline uint32_t susp_area_start(uint8_t name_len) {
    return 33u + name_len + ((name_len & 1u) == 0u ? 1u : 0u);
}

static bool iso9660_parse_susp_fields(const uint8_t *data, uint32_t data_len,
                                       ISO9660_FILE *file,
                                       char *nm_accum, uint32_t *nm_len,
                                       bool *nm_cont_out) {
    bool any = false;
    const uint8_t *p   = data;
    const uint8_t *end = data + data_len;

    while (p + 4 <= end) {
        uint8_t s0  = p[0];
        uint8_t s1  = p[1];
        uint8_t len = p[2];

        if (len < 4u || p + len > end) break;
        if (s0 == 'N' && s1 == 'M' && len >= 5u) {
            uint8_t  flags = p[4];
            uint32_t npart = (uint32_t)(len - 5u);
            if (!(flags & 0x02u) && !(flags & 0x04u)
                    && nm_accum && nm_len
                    && *nm_len + npart < (uint32_t)(ISO9660_NAME_MAX - 1u)) {
                memcpy(nm_accum + *nm_len, p + 5, npart);
                *nm_len += npart;
                if (nm_cont_out) *nm_cont_out = (flags & 0x01u) != 0u;
            }
            any = true;
        }
        else if (s0 == 'S' && s1 == 'L' && file) {
            file->is_symlink = 1;
            any = true;
        }
        else if ((s0 == 'P' && s1 == 'X') ||
                 (s0 == 'T' && s1 == 'F') ||
                 (s0 == 'R' && s1 == 'R')) {
            any = true;
        }

        p += len;
    }
    return any;
}

static bool iso9660_parse_rock_ridge(const uint8_t *rec, uint8_t rec_len,
                                      ISO9660_FILE *file, uint8_t susp_skip) {
    if (!rec || !file || rec_len < 34u) return false;

    uint8_t  name_len   = rec[32];
    uint32_t susp_start = susp_area_start(name_len) + (uint32_t)susp_skip;
    if (susp_start >= rec_len) return false;

    char     nm_buf[ISO9660_NAME_MAX];
    uint32_t nm_len  = 0;
    bool     nm_cont = false;
    memset(nm_buf, 0, sizeof(nm_buf));

    const uint8_t *susp = rec + susp_start;
    uint32_t       slen = (uint32_t)rec_len - susp_start;

    bool found = iso9660_parse_susp_fields(susp, slen,
                                            file, nm_buf, &nm_len, &nm_cont);

    {
        const uint8_t *p   = susp;
        const uint8_t *end = susp + slen;
        while (p + 4 <= end) {
            uint8_t s0  = p[0], s1 = p[1], len = p[2];
            if (len < 4u || p + len > end) break;
            if (s0 == 'C' && s1 == 'E' && len >= 28u) {
                uint32_t cb_val = iso9660_read_u32_le(p + 4);
                uint32_t co_val = iso9660_read_u32_le(p + 12);
                uint32_t cl_val = iso9660_read_u32_le(p + 20);
                if (cl_val > 0u &&
                    cl_val <= (uint32_t)ISO9660_SECTOR_SIZE &&
                    iso9660_read_sector(cb_val, g_iso_ce_buffer) &&
                    co_val + cl_val <= (uint32_t)ISO9660_SECTOR_SIZE) {
                    bool ce_ok = iso9660_parse_susp_fields(
                        g_iso_ce_buffer + co_val, cl_val,
                        file, nm_buf, &nm_len, &nm_cont);
                    if (ce_ok) found = true;
                }
                break;
            }
            p += len;
        }
    }

    if (nm_len > 0u && !nm_cont) {
        nm_buf[nm_len] = '\0';
        memcpy(file->name, nm_buf, nm_len + 1u);
    }

    return found;
}

static bool iso9660_check_susp_sp(const uint8_t *rec, uint8_t rec_len,
                                   uint8_t *skip_out) {
    if (!rec || rec_len < 34u) return false;
    uint8_t  name_len   = rec[32];
    uint32_t susp_start = susp_area_start(name_len);
    if (susp_start + 7u > (uint32_t)rec_len) return false;

    const uint8_t *p   = rec + susp_start;
    const uint8_t *end = rec + rec_len;
    while (p + 4 <= end) {
        uint8_t s0  = p[0], s1 = p[1], len = p[2];
        if (len < 4u || p + len > end) break;
        if (s0 == 'S' && s1 == 'P' && len >= 7u &&
            p[4] == SUSP_SP_CHECK1 && p[5] == SUSP_SP_CHECK2) {
            if (skip_out) *skip_out = p[6];
            return true;
        }
        p += len;
    }
    return false;
}

static bool iso9660_validate_pvd(const uint8_t *sector) {
    return sector[0] == VD_TYPE_PVD &&
           memcmp(&sector[1], ISO9660_SIGNATURE, 5) == 0 &&
           sector[6] == ISO9660_VERSION;
}

static bool iso9660_parse_pvd(const uint8_t *sector, ISO9660_CONTEXT *ctx) {
    if (!iso9660_validate_pvd(sector)) return false;

    ctx->root_extent       = iso9660_read_u32_both(&sector[156 + 2]);
    ctx->root_size         = iso9660_read_u32_both(&sector[156 + 10]);
    ctx->vol_space_size    = iso9660_read_u32_both(&sector[80]);
    ctx->logical_block_size= iso9660_read_u16_le(&sector[128]);
    return ctx->logical_block_size == ISO9660_SECTOR_SIZE;
}

static bool iso9660_scan_descriptors(ISO9660_CONTEXT *ctx) {
    bool found_pvd = false;
    
    for (uint32_t lba = 16u; lba < 32u; lba++) {
        if (!iso9660_read_sector(lba, g_iso_sector_buffer)) {
            break;
        }

        uint8_t type = g_iso_sector_buffer[0];
        if (type == VD_TYPE_TERMINATOR) break;
        if (memcmp(&g_iso_sector_buffer[1], ISO9660_SIGNATURE, 5) != 0) continue;

        if (type == VD_TYPE_PVD && !found_pvd) {
            if (iso9660_parse_pvd(g_iso_sector_buffer, ctx)) {
                ctx->pvd_extent = lba;
                found_pvd = true;
            }
        } else if (type == VD_TYPE_SVD && !ctx->has_joliet) {
            const uint8_t *esc = &g_iso_sector_buffer[88];
            for (int j = 0; j < 3; j++) {
                if (memcmp(esc, k_joliet_esc[j], 3) == 0) {
                    ctx->joliet_root_extent = iso9660_read_u32_both(
                        &g_iso_sector_buffer[156 + 2]);
                    ctx->joliet_root_size   = iso9660_read_u32_both(
                        &g_iso_sector_buffer[156 + 10]);
                    ctx->has_joliet = true;
                    break;
                }
            }
        }
    }

    return found_pvd;
}

static void iso9660_detect_rock_ridge(ISO9660_CONTEXT *ctx) {
    if (!iso9660_read_sector(ctx->root_extent, g_iso_sector_buffer)) return;

    const uint8_t *rec     = g_iso_sector_buffer;
    uint8_t        rec_len = rec[0];
    if (rec_len == 0u || (uint32_t)rec_len > ISO9660_SECTOR_SIZE) return;

    uint8_t skip = 0u;
    if (iso9660_check_susp_sp(rec, rec_len, &skip)) {
        ctx->has_rock_ridge = true;
        ctx->rr_susp_skip   = skip;
    }
}

static bool iso9660_read_dir_record(const uint8_t *data, ISO9660_FILE *file,
                                     uint32_t dir_extent, uint32_t dir_offset,
                                     bool joliet) {
    if (!data || !file) return false;

    uint8_t rec_len  = data[0];
    if (rec_len == 0u) return false;

    uint8_t name_len = data[32];
    if (name_len == 0u) return false;

    file->extent     = iso9660_read_u32_both(&data[2]);
    file->size       = iso9660_read_u32_both(&data[10]);
    file->is_dir     = (data[25] & ISO9660_ATTR_DIR) ? 1u : 0u;
    file->is_symlink = 0u;
    file->dir_extent = dir_extent;
    file->dir_offset = dir_offset;

    if (name_len == 1u) {
        if (data[33] == 0x00u) {
            file->name[0] = '.';  file->name[1] = '\0';
            return true;
        } else if (data[33] == 0x01u) {
            file->name[0] = '.';  file->name[1] = '.';  file->name[2] = '\0';
            return true;
        }
    }

    if (joliet) {
        ucs2be_to_utf8(&data[33], name_len,
                        file->name, (uint32_t)(ISO9660_NAME_MAX - 1u));
        iso9660_strip_version(file->name);
    } else {
        if (name_len > (uint32_t)(ISO9660_NAME_MAX - 1u)) return false;
        memcpy(file->name, &data[33], name_len);
        file->name[name_len] = '\0';

        if (g_iso_context.has_rock_ridge) {
            iso9660_parse_rock_ridge(data, rec_len, file,
                                     g_iso_context.rr_susp_skip);
        }
    }

    return true;
}

static bool iso9660_name_match(const char *iso_name, const char *target) {
    while (*target) {
        char c1 = *iso_name, c2 = *target;
        if (c1 >= 'a' && c1 <= 'z') c1 -= 32;
        if (c2 >= 'a' && c2 <= 'z') c2 -= 32;
        if (c1 != c2) return false;
        iso_name++;
        target++;
    }
    if (*iso_name == '\0' || *iso_name == ';')           return true;
    if (*iso_name == '.' &&
        (*(iso_name + 1) == ';' || *(iso_name + 1) == '\0')) return true;
    return false;
}

static bool iso9660_lookup_in_dir(uint32_t dir_extent, uint32_t dir_size,
                                    const char *target_name,
                                    ISO9660_FILE *out_file, bool joliet) {
    if (!target_name || !out_file) return false;

    uint32_t bytes_read = 0u;
    uint32_t extent     = dir_extent;

    while (bytes_read < dir_size) {
        if (!iso9660_read_sector(extent, g_iso_sector_buffer)) return false;

        uint32_t offset = 0u;
        while (offset < ISO9660_SECTOR_SIZE && bytes_read < dir_size) {
            const uint8_t *rec     = &g_iso_sector_buffer[offset];
            uint8_t        rec_len = rec[0];

            if (rec_len == 0u) {
                bytes_read += (ISO9660_SECTOR_SIZE - offset);
                offset = ISO9660_SECTOR_SIZE;
                break;
            }

            ISO9660_FILE temp;
            if (iso9660_read_dir_record(rec, &temp, dir_extent,
                                         bytes_read + offset, joliet)) {
                if (iso9660_name_match(temp.name, target_name)) {
                    memcpy(out_file, &temp, sizeof(*out_file));
                    return true;
                }
            }

            offset     += rec_len;
            bytes_read += rec_len;
        }

        extent++;
    }

    return false;
}

static bool iso9660_lookup_path(const char *path, ISO9660_FILE *out_entry,
                                  bool joliet) {
    if (!path || !out_entry || path[0] != '/') return false;

    uint32_t current_extent = joliet ? g_iso_context.joliet_root_extent
                                      : g_iso_context.root_extent;
    uint32_t current_size   = joliet ? g_iso_context.joliet_root_size
                                      : g_iso_context.root_size;
    const char *cursor = path + 1;

    if (*cursor == '\0') {
        out_entry->extent  = current_extent;
        out_entry->size    = current_size;
        out_entry->is_dir  = 1u;
        out_entry->name[0] = '/';
        out_entry->name[1] = '\0';
        return true;
    }

    while (*cursor) {
        const char *end = cursor;
        while (*end && *end != '/') end++;

        uint32_t name_len = (uint32_t)(end - cursor);
        if (name_len == 0u || name_len >= ISO9660_NAME_MAX) return false;

        char component[ISO9660_NAME_MAX];
        memcpy(component, cursor, name_len);
        component[name_len] = '\0';

        ISO9660_FILE found;
        if (!iso9660_lookup_in_dir(current_extent, current_size,
                                    component, &found, joliet))
            return false;

        if (*end == '\0') {
            memcpy(out_entry, &found, sizeof(*out_entry));
            return true;
        }

        if (!found.is_dir) return false;
        current_extent = found.extent;
        current_size   = found.size;
        cursor = end + 1;
        while (*cursor == '/') cursor++;   /* collapse "//" runs */
        if (*cursor == '\0') {
            /* Trailing '/': the component just matched IS the target. Callers
             * legitimately probe directories as "<path>/" (Xorg's module
             * loader does exactly this via stat()); without this the walker
             * fell out of the loop and reported "not found". */
            memcpy(out_entry, &found, sizeof(*out_entry));
            return true;
        }
    }

    return false;
}

static bool _iso9660_init(void) {
    g_iso_partition_lba = disk_get_partition_lba();
    memset(&g_iso_context, 0, sizeof(g_iso_context));

    if (iso9660_scan_descriptors(&g_iso_context)) {
        iso9660_detect_rock_ridge(&g_iso_context);
        return true;
    }

    if (g_iso_partition_lba != 0) {
        g_iso_partition_lba = 0;
        memset(&g_iso_context, 0, sizeof(g_iso_context));
        if (iso9660_scan_descriptors(&g_iso_context)) {
            iso9660_detect_rock_ridge(&g_iso_context);
            return true;
        }
    }

    return false;
}

bool iso9660_init(void) {
    spinlock_lock(&g_iso_lock);
    bool ret = _iso9660_init();
    spinlock_unlock(&g_iso_lock);
    return ret;
}

static bool _iso9660_find_file(const char *path, ISO9660_FILE *file) {
    if (!path || !file) return false;

    if (g_iso_context.has_joliet) {
        if (iso9660_lookup_path(path, file, true) && !file->is_dir)
            return true;
    }

    if (!iso9660_lookup_path(path, file, false)) return false;
    return !file->is_dir;
}

bool iso9660_find_file(const char *path, ISO9660_FILE *file) {
    spinlock_lock(&g_iso_lock);
    bool ret = _iso9660_find_file(path, file);
    spinlock_unlock(&g_iso_lock);
    return ret;
}

static bool _iso9660_read_at(ISO9660_FILE *file, uint32_t offset,
                               uint8_t *buf, uint32_t size) {
    if (!file || !buf || offset > file->size) return false;
    if (size == 0u) return true;
    if (size > file->size - offset) size = file->size - offset;
    if (size == 0u) return true;

    uint32_t bytes_done      = 0u;
    uint32_t current_extent  = file->extent + (offset / ISO9660_SECTOR_SIZE);
    uint32_t offset_in_sector= offset % ISO9660_SECTOR_SIZE;

    while (bytes_done < size) {
        uint32_t remaining = size - bytes_done;
        if (offset_in_sector == 0u && remaining >= ISO9660_SECTOR_SIZE) {
            uint32_t full_sectors = remaining / ISO9660_SECTOR_SIZE;
            uint64_t base = g_iso_partition_lba +
                            (uint64_t)current_extent *
                            (ISO9660_SECTOR_SIZE / 512u);
            if (!disk_read(base,
                           &buf[bytes_done],
                           full_sectors * (ISO9660_SECTOR_SIZE / 512u))) {
                return false;
            }
            bytes_done += full_sectors * ISO9660_SECTOR_SIZE;
            current_extent += full_sectors;
            continue;
        }

        spinlock_lock(&g_iso_lock);

        bool read_ok = iso9660_read_sector(current_extent, g_iso_read_buffer);

        uint32_t can_read = ISO9660_SECTOR_SIZE - offset_in_sector;
        uint32_t to_read  = (size - bytes_done) < can_read
                            ? (size - bytes_done) : can_read;

        if (read_ok) {
            memcpy(&buf[bytes_done], &g_iso_read_buffer[offset_in_sector], to_read);
        }

        spinlock_unlock(&g_iso_lock);

        if (!read_ok) {
            return false;
        }

        bytes_done      += to_read;
        offset_in_sector = 0u;
        current_extent++;
    }

    return true;
}

bool iso9660_read_at(ISO9660_FILE *file, uint32_t offset,
                      uint8_t *buf, uint32_t size) {
    return _iso9660_read_at(file, offset, buf, size);
}

bool iso9660_read_file(ISO9660_FILE *file, uint8_t *buf) {
    if (!file || !buf) return false;
    return iso9660_read_at(file, 0u, buf, file->size);
}

uint32_t iso9660_get_file_size(ISO9660_FILE *file) {
    return file ? file->size : 0u;
}

void iso9660_list_root_files(void) {
    spinlock_lock(&g_iso_lock);

    bool     joliet = g_iso_context.has_joliet;
    uint32_t extent = joliet ? g_iso_context.joliet_root_extent
                              : g_iso_context.root_extent;
    uint32_t size   = joliet ? g_iso_context.joliet_root_size
                              : g_iso_context.root_size;
    uint32_t bytes_read = 0u;

    while (bytes_read < size) {
        if (!iso9660_read_sector(extent, g_iso_sector_buffer)) break;

        uint32_t offset = 0u;
        while (offset < ISO9660_SECTOR_SIZE && bytes_read < size) {
            const uint8_t *rec     = &g_iso_sector_buffer[offset];
            uint8_t        rec_len = rec[0];

            if (rec_len == 0u) {
                bytes_read += (ISO9660_SECTOR_SIZE - offset);
                break;
            }

            offset     += rec_len;
            bytes_read += rec_len;
        }

        extent++;
    }

    spinlock_unlock(&g_iso_lock);
}

static int32_t _iso9660_opendir(const char *path) {
    const char  *dir_path = (path && *path) ? path : "/";
    bool         joliet   = g_iso_context.has_joliet;
    ISO9660_FILE dir_file;

    if (!iso9660_lookup_path(dir_path, &dir_file, joliet)) {
        joliet = !joliet;
        if (!iso9660_lookup_path(dir_path, &dir_file, joliet))
            return -1;
    }
    if (!dir_file.is_dir) return -1;

    for (int32_t i = 0; i < ISO9660_DIR_HANDLE_MAX; i++) {
        if (!g_dir_handles[i].used) {
            g_dir_handles[i].used           = 1u;
            g_dir_handles[i].current_extent = dir_file.extent;
            g_dir_handles[i].remaining_size = dir_file.size;
            g_dir_handles[i].entry_offset   = 0u;
            g_dir_handles[i].joliet         = joliet;
            return i;
        }
    }
    return -1;
}

int32_t iso9660_opendir(const char *path) {
    spinlock_lock(&g_iso_lock);
    int32_t ret = _iso9660_opendir(path);
    spinlock_unlock(&g_iso_lock);
    return ret;
}

static int32_t _iso9660_readdir(int32_t handle, ISO9660_DIRENT *out) {
    if (handle < 0 || handle >= ISO9660_DIR_HANDLE_MAX || !out) return -1;

    iso9660_dir_handle_t *h = &g_dir_handles[handle];
    if (!h->used)             return -1;
    if (!h->remaining_size)   return  0;

    if (!iso9660_read_sector(h->current_extent, g_iso_sector_buffer))
        return -1;

    while (h->remaining_size > 0u) {
        uint32_t       sector_off = h->entry_offset % ISO9660_SECTOR_SIZE;
        const uint8_t *rec        = &g_iso_sector_buffer[sector_off];
        uint8_t        rec_len    = rec[0];

        if (rec_len == 0u) {
            uint32_t skip = ISO9660_SECTOR_SIZE - sector_off;
            if (skip > h->remaining_size) skip = h->remaining_size;
            h->entry_offset   += skip;
            h->remaining_size -= skip;
            if (h->remaining_size == 0u) break;
            h->current_extent++;
            if (!iso9660_read_sector(h->current_extent, g_iso_sector_buffer))
                return -1;
            continue;
        }

        ISO9660_FILE temp;
        bool valid = iso9660_read_dir_record(rec, &temp,
                                              h->current_extent,
                                              sector_off, h->joliet);

        if (rec_len > h->remaining_size) rec_len = (uint8_t)h->remaining_size;
        h->entry_offset   += rec_len;
        h->remaining_size -= rec_len;

        if (h->remaining_size > 0u &&
            (h->entry_offset % ISO9660_SECTOR_SIZE) == 0u) {
            h->current_extent++;
            if (!iso9660_read_sector(h->current_extent, g_iso_sector_buffer))
                return -1;
        }

        if (!valid) continue;

        if ((temp.name[0] == '.' && temp.name[1] == '\0') ||
            (temp.name[0] == '.' && temp.name[1] == '.' &&
             temp.name[2] == '\0'))
            continue;

        strcpy(out->name, temp.name);
        out->size         = temp.size;
        out->extent       = temp.extent;
        out->is_directory = temp.is_dir;
        return 1;
    }

    return 0;
}

int32_t iso9660_readdir(int32_t handle, ISO9660_DIRENT *out) {
    spinlock_lock(&g_iso_lock);
    int32_t ret = _iso9660_readdir(handle, out);
    spinlock_unlock(&g_iso_lock);
    return ret;
}

static int32_t _iso9660_closedir(int32_t handle) {
    if (handle < 0 || handle >= ISO9660_DIR_HANDLE_MAX) return -1;
    memset(&g_dir_handles[handle], 0, sizeof(g_dir_handles[handle]));
    return 0;
}

int32_t iso9660_closedir(int32_t handle) {
    spinlock_lock(&g_iso_lock);
    int32_t ret = _iso9660_closedir(handle);
    spinlock_unlock(&g_iso_lock);
    return ret;
}

#ifdef IMPLUS_DRIVER_MODULE

/* ---- fs_module_ops_t shims: opaque void *handle == ISO9660_FILE *, and
 * ISO9660_DIRENT -> vfs_dirent_t. ISO9660 is read-only media, so the whole
 * write side stays NULL below. ---- */

static bool iso9660_ops_find_file(const char *path, void *handle,
                                  uint64_t *out_id, uint32_t *out_size)
{
    ISO9660_FILE *file = (ISO9660_FILE *)handle;
    if (!iso9660_find_file(path, file)) {
        return false;
    }
    /* Stable per-file identity: the extent LBA is unique per file and
     * constant across opens (the malloc'd handle pointer is not). The Linux
     * compat layer maps this to st_ino, which glibc's ld.so uses to dedup
     * already-mapped shared objects. */
    *out_id = (uint64_t)file->extent;
    *out_size = file->size;
    return true;
}

static bool iso9660_ops_read_file(void *handle, uint8_t *buffer)
{
    return iso9660_read_file((ISO9660_FILE *)handle, buffer);
}

static bool iso9660_ops_read_at(void *handle, uint32_t offset,
                                uint8_t *buffer, uint32_t size)
{
    return iso9660_read_at((ISO9660_FILE *)handle, offset, buffer, size);
}

static uint32_t iso9660_ops_get_file_size(void *handle)
{
    return iso9660_get_file_size((ISO9660_FILE *)handle);
}

static int32_t iso9660_ops_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    ISO9660_DIRENT dirent;
    memset(&dirent, 0, sizeof(dirent));
    int32_t result = iso9660_readdir(handle, &dirent);
    if (result <= 0) {
        return result;
    }
    fs_dirent_set_name(out_entry, dirent.name);
    out_entry->size = dirent.size;
    out_entry->is_directory = dirent.is_directory != 0;
    return result;
}

static const fs_module_ops_t g_iso9660_driver = {
    .fs_type       = "iso9660",
    .media_kind    = VFS_MEDIA_KIND_OPTICAL,
    .handle_size   = (uint32_t)sizeof(ISO9660_FILE),
    .init          = iso9660_init,
    .find_file     = iso9660_ops_find_file,
    .read_file     = iso9660_ops_read_file,
    .read_at       = iso9660_ops_read_at,
    .get_file_size = iso9660_ops_get_file_size,
    .opendir       = iso9660_opendir,
    .readdir       = iso9660_ops_readdir,
    .closedir      = iso9660_closedir,
    .list_root     = iso9660_list_root_files,
};

static void iso9660_driver_shutdown(void) {
    g_driver_api = NULL;
    g_iso_partition_lba = 0;
    memset(&g_iso_context, 0, sizeof(g_iso_context));
}

static const driver_module_descriptor_t g_iso9660_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_FILESYSTEM,
    .load_priority = 100u,
    .deps = { NULL },
    .driver_api = &g_iso9660_driver,
    .shutdown   = iso9660_driver_shutdown,
};

#undef hal_cpu_pause
#undef disk_read
#undef disk_write
#undef disk_get_partition_lba
#undef serial_write_string
#undef serial_write_uint32
#undef memset
#undef memcpy

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api) {
    if (!api || !api->disk_read || !api->disk_get_partition_lba ||
        !api->memset || !api->memcpy)
        return NULL;
    g_driver_api = api;
    g_iso_partition_lba = api->disk_get_partition_lba();
    spinlock_init(&g_iso_lock);
    return &g_iso9660_module;
}
#endif
