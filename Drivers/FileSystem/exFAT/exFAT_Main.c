#include "Drivers/FileSystem/exFAT/exFAT_Main.h"
#include "Debug/serial/Serial.h"
#include "kernel/config.h"
#include <string.h>
#include <stddef.h>

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"

static const driver_binary_t *g_driver_api = NULL;

#define hal_cpu_pause               g_driver_api->hal.cpu_pause
#define hal_cpu_save_interrupts     g_driver_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts  g_driver_api->hal.cpu_restore_interrupts
#define disk_read                   g_driver_api->hw.disk_read
#define disk_get_partition_lba      g_driver_api->hw.disk_get_partition_lba

typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_init(spinlock_t *l)   { l->locked = 0; }
static inline void spinlock_lock(spinlock_t *l)   {
    while (__sync_lock_test_and_set(&l->locked, 1)) {
        while (l->locked) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->locked); }

void *memcpy(void *dest, const void *src, size_t n) {
    if (!g_driver_api || !g_driver_api->memcpy) {
        char *d = dest;
        const char *s = src;
        for (size_t i = 0; i < n; i++) d[i] = s[i];
        return dest;
    }
    return g_driver_api->memcpy(dest, src, n);
}

void *memset(void *s, int c, size_t n) {
    if (!g_driver_api || !g_driver_api->memset) {
        char *p = s;
        for (size_t i = 0; i < n; i++) p[i] = (char)c;
        return s;
    }
    return g_driver_api->memset(s, c, n);
}

#else
#include "Core/sync/Spinlock.h"
#include "Platform/io/IO_Main.h"
#endif

/* ---- exFAT on-disk layout constants --------------------------------------
 * "exFAT File System Specification" (Microsoft, 2019-08-28), section 3
 * (Main and Backup Boot Sector) and section 7 (Directory Entries). Offsets
 * below are all relative to the start of a 32-byte directory entry, or the
 * start of the 512-byte boot sector, as noted. */

#define EXFAT_ENTRY_SIZE               32u
#define EXFAT_ENTRY_TYPE_UNUSED        0x00u
#define EXFAT_ENTRY_TYPE_FILE          0x85u  /* File Directory Entry */
#define EXFAT_ENTRY_TYPE_STREAM        0xC0u  /* Stream Extension */
#define EXFAT_ENTRY_TYPE_FILENAME      0xC1u  /* File Name Entry */
#define EXFAT_ENTRY_INUSE_BIT          0x80u

#define EXFAT_ATTR_DIRECTORY           0x0010u

#define EXFAT_STREAM_FLAG_NOFATCHAIN   0x02u

#define EXFAT_FAT_ENTRY_BAD            0xFFFFFFF7u
#define EXFAT_FAT_ENTRY_EOF            0xFFFFFFFFu
#define EXFAT_FIRST_DATA_CLUSTER       2u

#define EXFAT_NAME_CHARS_PER_ENTRY     15u
#define EXFAT_MAX_SECONDARY_ENTRIES    18u  /* 1 stream + up to 17 name entries (255 chars) */

#define EXFAT_DIR_HANDLE_COUNT         FILE_MAX_DIR_HANDLE_CONFIG

typedef struct {
    uint64_t partition_offset;
    uint64_t volume_length;
    uint32_t fat_offset;          /* sectors, from start of partition */
    uint32_t fat_length;          /* sectors */
    uint32_t cluster_heap_offset; /* sectors, from start of partition */
    uint32_t cluster_count;
    uint32_t root_cluster;
    uint8_t  bytes_per_sector_shift;
    uint8_t  sectors_per_cluster_shift;
    uint8_t  num_fats;
} exFAT_SB;

static exFAT_SB g_sb;
static uint32_t g_exfat_partition_lba = 0;
static uint8_t  g_sector_buffer[EXFAT_SECTOR_BUFFER_SIZE] __attribute__((aligned(4096)));
static spinlock_t g_exfat_lock;

typedef struct {
    uint8_t  used;
    uint32_t first_cluster;
    uint8_t  no_fat_chain;
    uint32_t current_cluster;
    uint32_t entry_in_cluster;
} exfat_dir_handle_t;

static exfat_dir_handle_t g_dir_handles[EXFAT_DIR_HANDLE_COUNT];

/* Cursor used to walk one directory's (or file's) 32-byte entries, one at a
 * time, transparently crossing sector and cluster boundaries. Shared by
 * find_file()'s path-component scan and by opendir()/readdir(). */
typedef struct {
    uint32_t first_cluster;
    uint8_t  no_fat_chain;
    uint32_t current_cluster;
    uint32_t entry_in_cluster;
    uint32_t cached_lba;   /* last sector loaded into g_sector_buffer, or 0xFFFFFFFF */
} exfat_cursor_t;

static uint16_t read_u16(const uint8_t *p) {
    if (!p) return 0;
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u32(const uint8_t *p) {
    if (!p) return 0;
    return (uint32_t)p[0]        |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16)|
           ((uint32_t)p[3] << 24);
}

static uint64_t read_u64(const uint8_t *p) {
    if (!p) return 0;
    return (uint64_t)p[0]         |
           ((uint64_t)p[1] << 8)  |
           ((uint64_t)p[2] << 16) |
           ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) |
           ((uint64_t)p[5] << 40) |
           ((uint64_t)p[6] << 48) |
           ((uint64_t)p[7] << 56);
}

static inline uint32_t exfat_bytes_per_sector(void) {
    return 1u << g_sb.bytes_per_sector_shift;
}

static inline uint32_t exfat_sectors_per_cluster(void) {
    return 1u << g_sb.sectors_per_cluster_shift;
}

static inline uint32_t exfat_cluster_size_bytes(void) {
    return exfat_sectors_per_cluster() * exfat_bytes_per_sector();
}

static inline uint32_t exfat_entries_per_cluster(void) {
    return exfat_cluster_size_bytes() / EXFAT_ENTRY_SIZE;
}

static uint32_t exfat_cluster_to_lba(uint32_t cluster) {
    if (cluster < EXFAT_FIRST_DATA_CLUSTER) return 0u;
    return g_exfat_partition_lba + g_sb.cluster_heap_offset +
           (cluster - EXFAT_FIRST_DATA_CLUSTER) * exfat_sectors_per_cluster();
}

/* Walks the FAT: reads the 32-bit entry for `cluster` and returns the next
 * cluster in its chain, or an EXFAT_FAT_ENTRY_* sentinel. Only used for
 * streams that do NOT have the NoFatChain optimization flag set (see
 * exFAT_FILE.no_fat_chain / EXFAT_STREAM_FLAG_NOFATCHAIN above) -- for a
 * NoFatChain stream the FAT entries are not maintained by the formatter and
 * must never be consulted. */
static uint32_t exfat_fat_get_next_cluster(uint32_t cluster) {
    uint32_t byte_offset = cluster * 4u;
    uint32_t sector = g_sb.fat_offset + (byte_offset / exfat_bytes_per_sector());
    uint32_t offset = byte_offset % exfat_bytes_per_sector();

    if (!disk_read(g_exfat_partition_lba + sector, g_sector_buffer, 1)) {
        return EXFAT_FAT_ENTRY_BAD;
    }
    return read_u32(&g_sector_buffer[offset]);
}

static void exfat_cursor_init(exfat_cursor_t *c, uint32_t first_cluster, bool no_fat_chain) {
    c->first_cluster    = first_cluster;
    c->no_fat_chain      = no_fat_chain ? 1u : 0u;
    c->current_cluster  = first_cluster;
    c->entry_in_cluster = 0u;
    c->cached_lba       = 0xFFFFFFFFu;
}

/* Reads the next 32-byte directory entry, advancing the cursor. Returns
 * false once the stream's cluster chain is exhausted (end of directory). */
static bool exfat_cursor_read_entry(exfat_cursor_t *c, uint8_t out[EXFAT_ENTRY_SIZE]) {
    uint32_t entries_per_cluster = exfat_entries_per_cluster();
    if (entries_per_cluster == 0u || c->current_cluster < EXFAT_FIRST_DATA_CLUSTER) {
        return false;
    }

    if (c->entry_in_cluster >= entries_per_cluster) {
        uint32_t next;
        if (c->no_fat_chain) {
            next = c->current_cluster + 1u;
        } else {
            next = exfat_fat_get_next_cluster(c->current_cluster);
        }
        if (next < EXFAT_FIRST_DATA_CLUSTER || next >= EXFAT_FAT_ENTRY_BAD) {
            return false;
        }
        c->current_cluster  = next;
        c->entry_in_cluster = 0u;
    }

    uint32_t byte_offset  = c->entry_in_cluster * EXFAT_ENTRY_SIZE;
    uint32_t sector_index = byte_offset / exfat_bytes_per_sector();
    uint32_t sector_off   = byte_offset % exfat_bytes_per_sector();
    uint32_t lba = exfat_cluster_to_lba(c->current_cluster) + sector_index;

    if (lba != c->cached_lba) {
        if (!disk_read(lba, g_sector_buffer, 1)) {
            return false;
        }
        c->cached_lba = lba;
    }

    memcpy(out, &g_sector_buffer[sector_off], EXFAT_ENTRY_SIZE);
    c->entry_in_cluster++;
    return true;
}

/* UTF-16LE -> best-effort ASCII. exFAT names are always UTF-16LE; ImplusOS
 * paths elsewhere in the tree (FAT32, ISO9660) are plain ASCII, so anything
 * outside 0x20-0x7E is substituted with '?' rather than pulled in as a
 * dependency on a full Unicode codec here. */
static void exfat_utf16_to_ascii(const uint16_t *utf16, uint32_t count, char *out, uint32_t out_size) {
    uint32_t n = count < (out_size - 1u) ? count : (out_size - 1u);
    uint32_t i;
    for (i = 0; i < n; i++) {
        uint16_t ch = utf16[i];
        out[i] = (ch >= 0x20u && ch <= 0x7Eu) ? (char)ch : '?';
    }
    out[i] = '\0';
}

static bool exfat_ascii_ieq(const char *a, const char *b) {
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca = (char)(ca - 'a' + 'A');
        if (cb >= 'a' && cb <= 'z') cb = (char)(cb - 'a' + 'A');
        if (ca != cb) return false;
        a++; b++;
    }
    return *a == '\0' && *b == '\0';
}

/* Reads one File-Directory-Entry set (the 0x85 primary entry plus its
 * 0xC0 Stream Extension and 0xC1 File Name secondary entries) starting at
 * the cursor's current position, and fills `out`. Skips over unused/deleted
 * (in-use bit clear) and unrecognized entries. Returns false at end of
 * directory. */
static bool exfat_scan_next(exfat_cursor_t *c, exFAT_FILE *out) {
    uint8_t entry[EXFAT_ENTRY_SIZE];

    for (;;) {
        uint32_t set_cluster = c->current_cluster;
        uint32_t set_offset  = c->entry_in_cluster * EXFAT_ENTRY_SIZE;

        if (!exfat_cursor_read_entry(c, entry)) {
            return false;
        }

        uint8_t entry_type = entry[0];
        if (entry_type == EXFAT_ENTRY_TYPE_UNUSED) {
            return false; /* first unused entry marks end of directory */
        }
        if (entry_type != EXFAT_ENTRY_TYPE_FILE) {
            continue; /* deleted / bitmap / upcase / label / stray secondary */
        }

        uint8_t  secondary_count = entry[1];
        uint16_t file_attrs      = read_u16(&entry[4]);
        bool     is_dir          = (file_attrs & EXFAT_ATTR_DIRECTORY) != 0u;

        if (secondary_count == 0u || secondary_count > EXFAT_MAX_SECONDARY_ENTRIES) {
            continue; /* malformed entry set, skip */
        }

        bool     stream_found = false;
        bool     no_fat_chain = false;
        uint32_t first_cluster = 0u;
        uint64_t data_length = 0u;
        uint16_t name_utf16[EXFAT_NAME_MAX];
        uint32_t name_len = 0u;
        uint8_t  name_len_expected = 0u;
        bool     truncated = false;

        for (uint8_t i = 0; i < secondary_count; i++) {
            uint8_t sec[EXFAT_ENTRY_SIZE];
            if (!exfat_cursor_read_entry(c, sec)) {
                truncated = true;
                break;
            }
            uint8_t sec_type = sec[0];
            if (sec_type == EXFAT_ENTRY_TYPE_STREAM && !stream_found) {
                stream_found      = true;
                no_fat_chain      = (sec[1] & EXFAT_STREAM_FLAG_NOFATCHAIN) != 0u;
                name_len_expected = sec[3];
                first_cluster     = read_u32(&sec[20]);
                data_length       = read_u64(&sec[24]);
            } else if (sec_type == EXFAT_ENTRY_TYPE_FILENAME) {
                for (uint32_t j = 0; j < EXFAT_NAME_CHARS_PER_ENTRY &&
                                      name_len < (EXFAT_NAME_MAX - 1u); j++) {
                    name_utf16[name_len++] = read_u16(&sec[2u + j * 2u]);
                }
            }
        }

        if (truncated || !stream_found) {
            continue; /* corrupt entry set, skip to whatever follows */
        }
        if (name_len_expected < name_len) {
            name_len = name_len_expected;
        }

        exfat_utf16_to_ascii(name_utf16, name_len, out->name, sizeof(out->name));
        out->first_cluster    = first_cluster;
        out->size             = data_length;
        out->no_fat_chain     = no_fat_chain;
        out->is_directory     = is_dir;
        out->dir_cluster       = set_cluster;
        out->dir_entry_offset = set_offset;
        out->secondary_count  = secondary_count;
        return true;
    }
}

/* ---- boot sector parsing / partition detection ---------------------- */

static bool exfat_parse_boot_sector(const uint8_t *sector, exFAT_SB *out) {
    if (!sector || !out) return false;
    if (memcmp(&sector[3], "EXFAT   ", 8) != 0) return false;
    if (sector[510] != 0x55u || sector[511] != 0xAAu) return false;

    uint8_t bps_shift = sector[108];
    uint8_t spc_shift = sector[109];
    uint8_t num_fats  = sector[110];

    if (bps_shift < 9u || bps_shift > 12u) return false;
    if ((uint32_t)bps_shift + spc_shift > 25u) return false;
    if (num_fats != 1u && num_fats != 2u) return false;

    out->partition_offset          = read_u64(&sector[64]);
    out->volume_length              = read_u64(&sector[72]);
    out->fat_offset                = read_u32(&sector[80]);
    out->fat_length                = read_u32(&sector[84]);
    out->cluster_heap_offset       = read_u32(&sector[88]);
    out->cluster_count             = read_u32(&sector[92]);
    out->root_cluster              = read_u32(&sector[96]);
    out->bytes_per_sector_shift    = bps_shift;
    out->sectors_per_cluster_shift = spc_shift;
    out->num_fats                  = num_fats;

    if (out->root_cluster < EXFAT_FIRST_DATA_CLUSTER) return false;
    if (out->cluster_count == 0u) return false;
    return true;
}

static bool exfat_try_init_at_lba(uint64_t lba) {
    if (lba > 0xFFFFFFFFULL) return false;
    if (!disk_read(lba, g_sector_buffer, 1)) return false;

    exFAT_SB sb;
    if (!exfat_parse_boot_sector(g_sector_buffer, &sb)) return false;

    g_sb = sb;
    g_exfat_partition_lba = (uint32_t)lba;
    return true;
}

/* MBR/GPT scan, mirroring FAT32's fat32_detect_partition_start() -- see
 * Kernel/Drivers/FileSystem/FAT32/FAT32_Main.c. exFAT partitions are
 * identified by MBR partition type 0x07 (shared with NTFS -- disambiguated
 * by the "EXFAT   " boot-sector signature) or, in a GPT, by simply trying
 * every entry and accepting the first one whose boot sector parses. */
static bool exfat_detect_partition_start(uint64_t *out_lba) {
    if (!out_lba) return false;
    if (!disk_read(0, g_sector_buffer, 1)) return false;

    if (g_sector_buffer[510] != 0x55u || g_sector_buffer[511] != 0xAAu) {
        return false;
    }

    bool protective_mbr = false;
    for (uint32_t i = 0; i < 4u; ++i) {
        uint32_t offset = 446u + i * 16u;
        uint8_t  type   = g_sector_buffer[offset + 4u];
        uint32_t start  = read_u32(&g_sector_buffer[offset + 8u]);

        if (type == 0xEEu) {
            protective_mbr = true;
            continue;
        }
        if (type == 0x07u && start != 0u) {
            if (exfat_try_init_at_lba(start)) {
                *out_lba = start;
                return true;
            }
        }
    }

    if (!protective_mbr) {
        return false;
    }

    if (!disk_read(1, g_sector_buffer, 1)) return false;
    if (memcmp(g_sector_buffer, "EFI PART", 8) != 0) return false;

    uint32_t entries_lba = read_u32(&g_sector_buffer[72]);
    uint32_t num_entries = read_u32(&g_sector_buffer[80]);
    uint32_t entry_size  = read_u32(&g_sector_buffer[84]);
    if (entry_size < 128u || entry_size > 4096u) return false;

    uint32_t entries_per_sector = 512u / entry_size;
    if (entries_per_sector == 0u) return false;

    uint32_t sectors_to_read = (num_entries + entries_per_sector - 1u) / entries_per_sector;
    for (uint32_t sector = 0u; sector < sectors_to_read; ++sector) {
        if (!disk_read(entries_lba + sector, g_sector_buffer, 1)) return false;

        for (uint32_t entry_index = 0u;
             entry_index < entries_per_sector && entry_index * entry_size + 40u <= 512u;
             ++entry_index) {
            uint8_t *entry = g_sector_buffer + entry_index * entry_size;

            bool all_zero = true;
            for (uint32_t g = 0; g < 16u; g++) {
                if (entry[g] != 0u) { all_zero = false; break; }
            }
            if (all_zero) continue;

            uint64_t first_lba = read_u64(entry + 32u);
            if (first_lba == 0u || first_lba > 0xFFFFFFFFULL) continue;

            if (exfat_try_init_at_lba(first_lba)) {
                *out_lba = first_lba;
                return true;
            }
        }
    }

    return false;
}

static bool _exfat_init(void) {
    uint64_t hint_lba = disk_get_partition_lba();
    if (hint_lba != 0u && exfat_try_init_at_lba(hint_lba)) {
        return true;
    }

    uint64_t fallback_lba = 0;
    return exfat_detect_partition_start(&fallback_lba);
}

bool exfat_init(void) {
    uint64_t flags = hal_cpu_save_interrupts();
    spinlock_lock(&g_exfat_lock);
    bool ret = _exfat_init();
    spinlock_unlock(&g_exfat_lock);
    hal_cpu_restore_interrupts(flags);
    return ret;
}

/* ---- path resolution -------------------------------------------------- */

static bool exfat_next_component(const char **path_ptr, char *out, uint32_t out_size) {
    const char *p = *path_ptr;
    while (*p == '/') p++;
    if (*p == '\0') { *path_ptr = p; return false; }

    uint32_t n = 0;
    while (*p != '\0' && *p != '/' && n < out_size - 1u) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    while (*p != '\0' && *p != '/') p++; /* skip any overflow remainder */
    *path_ptr = p;
    return true;
}

/* Walks `path` component by component from the root directory, returning
 * the final component's entry (file or directory) in `out`. `stop_before_last`
 * finds the *containing directory* stream instead (used by opendir()). */
static bool exfat_resolve(const char *path, exFAT_FILE *out, bool want_directory) {
    if (!path) return false;

    uint32_t stream_cluster   = g_sb.root_cluster;
    bool     stream_no_chain  = false;
    char     component[EXFAT_NAME_MAX];
    const char *p = path;

    bool have_result = false;
    exFAT_FILE result;
    memset(&result, 0, sizeof(result));
    result.first_cluster = g_sb.root_cluster;
    result.is_directory  = true;
    result.name[0]       = '\0';

    while (exfat_next_component(&p, component, sizeof(component))) {
        exfat_cursor_t cursor;
        exfat_cursor_init(&cursor, stream_cluster, stream_no_chain);

        exFAT_FILE entry;
        bool found = false;
        while (exfat_scan_next(&cursor, &entry)) {
            if (exfat_ascii_ieq(entry.name, component)) {
                found = true;
                break;
            }
        }
        if (!found) return false;

        if (!entry.is_directory) {
            /* Only valid as the final path component. */
            const char *rest = p;
            while (*rest == '/') rest++;
            if (*rest != '\0') return false; /* file used as a directory in the middle of the path */
        }

        result          = entry;
        have_result     = true;
        stream_cluster  = entry.first_cluster;
        stream_no_chain = entry.no_fat_chain;
    }

    if (!have_result) {
        /* path was "" or "/": the root directory itself. */
        if (!want_directory) return false;
        result.first_cluster = g_sb.root_cluster;
        result.no_fat_chain  = false;
        result.is_directory  = true;
    }

    if (want_directory && !result.is_directory) return false;
    if (out) *out = result;
    return true;
}

static bool _exfat_find_file(const char *path, exFAT_FILE *file) {
    if (!path || !file) return false;
    if (!exfat_resolve(path, file, false)) return false;
    return !file->is_directory;
}

bool exfat_find_file(const char *path, exFAT_FILE *file) {
    spinlock_lock(&g_exfat_lock);
    bool ret = _exfat_find_file(path, file);
    spinlock_unlock(&g_exfat_lock);
    return ret;
}

uint64_t exfat_get_file_size(exFAT_FILE *file) {
    if (!file) return 0;
    return file->size;
}

/* ---- file data reads ---------------------------------------------------
 * Random-access read: for a NoFatChain stream the cluster holding `offset`
 * is a direct O(1) computation; otherwise the FAT chain is walked from the
 * first cluster (O(cluster index) -- acceptable here since exFAT's own
 * NoFatChain optimization is precisely what lets a real-world formatter
 * avoid this cost for the common case of a freshly-written, unfragmented
 * file). */
static bool exfat_cluster_at_index(uint32_t first_cluster, bool no_fat_chain,
                                    uint32_t index, uint32_t *cluster_out) {
    if (no_fat_chain) {
        *cluster_out = first_cluster + index;
        return true;
    }

    uint32_t cluster = first_cluster;
    for (uint32_t i = 0; i < index; i++) {
        cluster = exfat_fat_get_next_cluster(cluster);
        if (cluster < EXFAT_FIRST_DATA_CLUSTER || cluster >= EXFAT_FAT_ENTRY_BAD) {
            return false;
        }
    }
    *cluster_out = cluster;
    return true;
}

static bool _exfat_read_at(exFAT_FILE *file, uint32_t offset, uint8_t *buffer, uint32_t size) {
    if (!file || !buffer || file->is_directory) return false;
    if ((uint64_t)offset >= file->size) return false;

    uint32_t cluster_size = exfat_cluster_size_bytes();
    if (cluster_size == 0u) return false;

    uint64_t remaining64 = file->size - offset;
    uint32_t remaining = (remaining64 < (uint64_t)size) ? (uint32_t)remaining64 : size;
    uint32_t total_read = 0u;

    while (total_read < remaining) {
        uint32_t abs_offset     = offset + total_read;
        uint32_t cluster_index  = abs_offset / cluster_size;
        uint32_t offset_in_clus = abs_offset % cluster_size;

        uint32_t cluster;
        if (!exfat_cluster_at_index(file->first_cluster, file->no_fat_chain, cluster_index, &cluster)) {
            return total_read > 0u;
        }

        uint32_t base_lba = exfat_cluster_to_lba(cluster);
        uint32_t sector_in_clus = offset_in_clus / exfat_bytes_per_sector();
        uint32_t sector_off     = offset_in_clus % exfat_bytes_per_sector();

        if (!disk_read(base_lba + sector_in_clus, g_sector_buffer, 1)) {
            return total_read > 0u;
        }

        uint32_t chunk = exfat_bytes_per_sector() - sector_off;
        uint32_t want  = remaining - total_read;
        if (chunk > want) chunk = want;

        memcpy(buffer + total_read, &g_sector_buffer[sector_off], chunk);
        total_read += chunk;
    }

    return total_read == remaining;
}

bool exfat_read_at(exFAT_FILE *file, uint32_t offset, uint8_t *buffer, uint32_t size) {
    spinlock_lock(&g_exfat_lock);
    bool ret = _exfat_read_at(file, offset, buffer, size);
    spinlock_unlock(&g_exfat_lock);
    return ret;
}

static bool _exfat_read_file(exFAT_FILE *file, uint8_t *buffer) {
    if (!file || !buffer) return false;
    if (file->size > 0xFFFFFFFFu) return false; /* whole-file read API is 32-bit sized */
    return _exfat_read_at(file, 0u, buffer, (uint32_t)file->size);
}

bool exfat_read_file(exFAT_FILE *file, uint8_t *buffer) {
    spinlock_lock(&g_exfat_lock);
    bool ret = _exfat_read_file(file, buffer);
    spinlock_unlock(&g_exfat_lock);
    return ret;
}

/* ---- directory enumeration --------------------------------------------- */

static int32_t _exfat_opendir(const char *path) {
    exFAT_FILE dir;
    if (!exfat_resolve(path ? path : "/", &dir, true)) return -1;

    for (int32_t i = 0; i < EXFAT_DIR_HANDLE_COUNT; ++i) {
        if (!g_dir_handles[i].used) {
            g_dir_handles[i].used             = 1u;
            g_dir_handles[i].first_cluster    = dir.first_cluster;
            g_dir_handles[i].no_fat_chain     = dir.no_fat_chain ? 1u : 0u;
            g_dir_handles[i].current_cluster  = dir.first_cluster;
            g_dir_handles[i].entry_in_cluster = 0u;
            return i;
        }
    }
    return -1;
}

int32_t exfat_opendir(const char *path) {
    spinlock_lock(&g_exfat_lock);
    int32_t ret = _exfat_opendir(path);
    spinlock_unlock(&g_exfat_lock);
    return ret;
}

static int32_t _exfat_readdir(int32_t handle, exFAT_DIRENT *out_entry) {
    if (handle < 0 || handle >= EXFAT_DIR_HANDLE_COUNT || !out_entry) return -1;
    exfat_dir_handle_t *h = &g_dir_handles[handle];
    if (!h->used) return -1;

    exfat_cursor_t cursor;
    cursor.first_cluster    = h->first_cluster;
    cursor.no_fat_chain      = h->no_fat_chain;
    cursor.current_cluster  = h->current_cluster;
    cursor.entry_in_cluster = h->entry_in_cluster;
    cursor.cached_lba       = 0xFFFFFFFFu;

    exFAT_FILE entry;
    if (!exfat_scan_next(&cursor, &entry)) {
        return 0;
    }

    h->current_cluster  = cursor.current_cluster;
    h->entry_in_cluster = cursor.entry_in_cluster;

    strncpy(out_entry->name, entry.name, sizeof(out_entry->name) - 1u);
    out_entry->name[sizeof(out_entry->name) - 1u] = '\0';
    out_entry->size          = entry.size;
    out_entry->first_cluster = entry.first_cluster;
    out_entry->is_directory  = entry.is_directory ? 1u : 0u;
    return 1;
}

int32_t exfat_readdir(int32_t handle, exFAT_DIRENT *out_entry) {
    spinlock_lock(&g_exfat_lock);
    int32_t ret = _exfat_readdir(handle, out_entry);
    spinlock_unlock(&g_exfat_lock);
    return ret;
}

static int32_t _exfat_closedir(int32_t handle) {
    if (handle < 0 || handle >= EXFAT_DIR_HANDLE_COUNT) return -1;
    memset(&g_dir_handles[handle], 0, sizeof(g_dir_handles[handle]));
    return 0;
}

int32_t exfat_closedir(int32_t handle) {
    spinlock_lock(&g_exfat_lock);
    int32_t ret = _exfat_closedir(handle);
    spinlock_unlock(&g_exfat_lock);
    return ret;
}

static void _exfat_list_root_files(void) {
    exfat_cursor_t cursor;
    exfat_cursor_init(&cursor, g_sb.root_cluster, false);

    exFAT_FILE entry;
    while (exfat_scan_next(&cursor, &entry)) {
#ifdef IMPLUS_DRIVER_MODULE
        if (g_driver_api && g_driver_api->dbg.write_string) {
            g_driver_api->dbg.write_string(entry.name);
            g_driver_api->dbg.write_string(entry.is_directory ? " <DIR>\n" : "\n");
        }
#else
        serial_write_string(entry.name);
        serial_write_string(entry.is_directory ? " <DIR>\n" : "\n");
#endif
    }
}

void exfat_list_root_files(void) {
    spinlock_lock(&g_exfat_lock);
    _exfat_list_root_files();
    spinlock_unlock(&g_exfat_lock);
}

#ifdef IMPLUS_DRIVER_MODULE

/* ---- fs_module_ops_t shims: opaque void *handle == exFAT_FILE *, and
 * exFAT_DIRENT -> vfs_dirent_t. exFAT_FILE.size is 64-bit (the on-disk
 * format allows > 4 GiB files); vfs_file_t / get_file_size are 32-bit, so
 * sizes are clamped to UINT32_MAX here, at the module boundary. exFAT is
 * read-only for now, so the write side stays NULL. ---- */

static uint32_t exfat_clamp_size(uint64_t size)
{
    return (size > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)size;
}

static bool exfat_ops_find_file(const char *path, void *handle,
                                uint64_t *out_id, uint32_t *out_size)
{
    exFAT_FILE *file = (exFAT_FILE *)handle;
    if (!exfat_find_file(path, file)) {
        return false;
    }
    *out_id = (uint64_t)(uintptr_t)file;
    *out_size = exfat_clamp_size(file->size);
    return true;
}

static bool exfat_ops_read_file(void *handle, uint8_t *buffer)
{
    return exfat_read_file((exFAT_FILE *)handle, buffer);
}

static bool exfat_ops_read_at(void *handle, uint32_t offset,
                              uint8_t *buffer, uint32_t size)
{
    return exfat_read_at((exFAT_FILE *)handle, offset, buffer, size);
}

static uint32_t exfat_ops_get_file_size(void *handle)
{
    return exfat_clamp_size(exfat_get_file_size((exFAT_FILE *)handle));
}

static int32_t exfat_ops_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    exFAT_DIRENT dirent;
    memset(&dirent, 0, sizeof(dirent));
    int32_t result = exfat_readdir(handle, &dirent);
    if (result <= 0) {
        return result;
    }
    fs_dirent_set_name(out_entry, dirent.name);
    out_entry->size = exfat_clamp_size(dirent.size);
    out_entry->is_directory = dirent.is_directory != 0;
    return result;
}

static const fs_module_ops_t g_exfat_driver = {
    .fs_type       = "exfat",
    .media_kind    = VFS_MEDIA_KIND_DISK,
    .handle_size   = (uint32_t)sizeof(exFAT_FILE),
    .init          = exfat_init,
    .find_file     = exfat_ops_find_file,
    .read_file     = exfat_ops_read_file,
    .read_at       = exfat_ops_read_at,
    .get_file_size = exfat_ops_get_file_size,
    .opendir       = exfat_opendir,
    .readdir       = exfat_ops_readdir,
    .closedir      = exfat_closedir,
    .list_root     = exfat_list_root_files,
};

static void exfat_driver_shutdown(void) {
    g_driver_api = NULL;
    memset(&g_sb, 0, sizeof(g_sb));
    g_exfat_partition_lba = 0;
}

static const driver_module_descriptor_t g_exfat_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_FILESYSTEM,
    .load_priority = 100u,
    .deps = { NULL },
    .driver_api = &g_exfat_driver,
    .shutdown = exfat_driver_shutdown,
};

#undef hal_cpu_pause
#undef hal_cpu_save_interrupts
#undef hal_cpu_restore_interrupts
#undef disk_read
#undef disk_get_partition_lba
#undef memset
#undef memcpy

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (!api) return NULL;

    if (!api->disk_read || !api->disk_get_partition_lba ||
        !api->memset || !api->memcpy)
        return NULL;

    g_driver_api = api;
    spinlock_init(&g_exfat_lock);

    return &g_exfat_module;
}
#endif
