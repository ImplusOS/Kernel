#include "Drivers/FileSystem/FAT32/FAT32_Main.h"
#include "Debug/serial/Serial.h"
#include "kernel/config.h"
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"

fat32_cache_t g_cluster_cache;

static const driver_binary_t *g_driver_api = NULL;

#define hal_cpu_pause       g_driver_api->hal.cpu_pause
#define hal_cpu_save_interrupts     g_driver_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts  g_driver_api->hal.cpu_restore_interrupts
#define disk_read           g_driver_api->hw.disk_read
#define disk_write          g_driver_api->hw.disk_write
#define disk_get_partition_lba g_driver_api->hw.disk_get_partition_lba
#define serial_write_string g_driver_api->dbg.write_string
#define serial_write_uint32 g_driver_api->dbg.write_uint32

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

#define FAT32_ATTR_VOLUME_ID        0x08u
#define FAT32_ATTR_DIRECTORY        0x10u
#define FAT32_ATTR_ARCHIVE          0x20u
#define FAT32_ATTR_LFN              0x0Fu
#define FAT32_DIR_ENTRY_SIZE        32u

#define FAT32_MAX_NAME_LEN          260u
#define FAT32_PATH_MAX              512u
#define FAT32_LFN_CHARS_PER_ENTRY   13u
#define FAT32_MAX_LFN_ORDER \
    ((FAT32_MAX_NAME_LEN + FAT32_LFN_CHARS_PER_ENTRY - 1u) / FAT32_LFN_CHARS_PER_ENTRY)

#define FAT32_MIN_SECTOR_SIZE           512u
#define FAT32_MAX_SECTORS_PER_CLUSTER   128u
#define FAT32_MAX_RESERVED_SECTORS      32768u
#define FAT32_MAX_NUM_FATS              2u
#define FAT32_MAX_FAT_SIZE_SECTORS      0x10000000UL
#define FAT32_EOC_MARKER                0x0FFFFFFFu
#define FAT32_DIR_HANDLE_MAX            FILE_MAX_DIR_HANDLE_CONFIG

static FAT32_BPB bpb;
static uint8_t g_sector_buffer[FAT32_MAX_SECTOR_SIZE] __attribute__((aligned(4096)));
static uint8_t g_read_buffer[FAT32_CLUSTER_BUFFER_SIZE] __attribute__((aligned(4096)));
static bool    g_case_sensitive_lookup = false;
static spinlock_t g_fat32_lock;

static uint32_t g_cached_fat_sector = 0xFFFFFFFFu;
static uint8_t  g_fat_cache_buf[FAT32_MAX_SECTOR_SIZE];
static uint32_t g_fat32_partition_lba = 0;
static uint32_t g_next_free_cluster_hint = 2u;
static bool     g_fsinfo_valid = false;
static uint32_t g_fsinfo_free_count = 0xFFFFFFFFu;

typedef struct {
    uint8_t  used;
    uint32_t directory_cluster;
    uint32_t current_cluster;
    uint32_t entry_offset;
} fat32_dir_handle_t;

static fat32_dir_handle_t g_dir_handles[FAT32_DIR_HANDLE_MAX];

static bool _fat32_truncate(FAT32_FILE *file, uint32_t new_size);
static bool _fat32_write_at(FAT32_FILE *file, uint32_t offset, const uint8_t *buffer, uint32_t size);

static uint32_t fat_get_next_cluster(uint32_t cluster);
static bool     fat_set_next_cluster(uint32_t cluster, uint32_t next);
static bool     fat32_free_cluster_chain(uint32_t first_cluster);
static bool     fat32_ensure_cluster_count(FAT32_FILE *file, uint32_t required_clusters);
static bool     fat32_get_cluster_at_index_cached(const FAT32_FILE *file, uint32_t index, uint32_t *cluster_out);
static bool     fat32_update_file_directory_entry(const FAT32_FILE *file);
static uint32_t cluster_to_lba(uint32_t cluster);
static uint32_t fat32_cluster_size_bytes(void);
static bool     fat32_zero_fill_range(FAT32_FILE *file, uint32_t offset, uint32_t size);
static bool     fat32_read_boot_sector_bpb(FAT32_BPB *out_bpb);
static bool     fat32_read_boot_sector_bpb_at_lba(uint64_t lba, FAT32_BPB *out_bpb);
static bool     fat32_detect_partition_start(uint64_t *out_lba);
static bool     fat32_try_init_at_lba(uint64_t lba);
static void     fat32_parse_bpb_from_sector(const uint8_t *sector, FAT32_BPB *out_bpb);
static bool     fat32_validate_bpb(void);
static bool     fat32_load_fsinfo(void);
static bool     fat32_store_fsinfo(void);

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

static bool is_guid_zero(const uint8_t *guid) {
    if (!guid) return true;
    for (uint32_t i = 0; i < 16u; ++i) {
        if (guid[i] != 0u) return false;
    }
    return true;
}

static uint32_t fat32_read_u32_unaligned(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static bool fat32_read_boot_sector_bpb_at_lba(uint64_t lba, FAT32_BPB *out_bpb) {
    if (!out_bpb || lba > 0xFFFFFFFFULL) return false;
    if (!disk_read((uint32_t)lba, g_sector_buffer, 1)) {
        return false;
    }
    fat32_parse_bpb_from_sector(g_sector_buffer, out_bpb);
    return true;
}

static bool fat32_try_init_at_lba(uint64_t lba) {
    FAT32_BPB temp_bpb;
    if (!fat32_read_boot_sector_bpb_at_lba(lba, &temp_bpb)) {
        return false;
    }

    FAT32_BPB previous_bpb = bpb;
    uint32_t previous_partition_lba = g_fat32_partition_lba;

    bpb = temp_bpb;
    if (!fat32_validate_bpb()) {
        bpb = previous_bpb;
        g_fat32_partition_lba = previous_partition_lba;
        return false;
    }

    g_fat32_partition_lba = (uint32_t)lba;
    g_cached_fat_sector = 0xFFFFFFFFu;
    g_cluster_cache.cluster_value = 0;
    g_next_free_cluster_hint = 2u;
    g_fsinfo_valid = false;
    g_fsinfo_free_count = 0xFFFFFFFFu;
    (void)fat32_load_fsinfo();
    return true;
}

static bool fat32_detect_partition_start(uint64_t *out_lba) {
    if (!out_lba) return false;
    if (!disk_read(0, g_sector_buffer, 1)) return false;

    if (g_sector_buffer[510] != 0x55u || g_sector_buffer[511] != 0xAAu) {
        return false;
    }

    bool protective_mbr = false;
    for (uint32_t i = 0; i < 4u; ++i) {
        uint32_t offset = 446u + i * 16u;
        uint8_t  type   = g_sector_buffer[offset + 4u];
        uint64_t start  = read_u32(&g_sector_buffer[offset + 8u]);

        if (type == 0xEEu) {
            protective_mbr = true;
            continue;
        }

        if ((type == 0x0Bu || type == 0x0Cu) && start != 0u) {
            if (fat32_try_init_at_lba(start)) {
                *out_lba = start;
                return true;
            }
        }
    }

    if (!protective_mbr) {
        return false;
    }

    if (!disk_read(1, g_sector_buffer, 1)) {
        return false;
    }

    if (memcmp(g_sector_buffer, "EFI PART", 8) != 0) {
        return false;
    }

    uint32_t entries_lba      = read_u32(&g_sector_buffer[72]);
    uint32_t num_entries      = read_u32(&g_sector_buffer[80]);
    uint32_t entry_size       = read_u32(&g_sector_buffer[84]);
    if (entry_size < 128u || entry_size > 4096u) {
        return false;
    }

    uint32_t entries_per_sector = 512u / entry_size;
    if (entries_per_sector == 0u) {
        return false;
    }

    uint32_t sectors_to_read = (num_entries + entries_per_sector - 1u) / entries_per_sector;
    for (uint32_t sector = 0u; sector < sectors_to_read; ++sector) {
        if (!disk_read(entries_lba + sector, g_sector_buffer, 1)) {
            return false;
        }

        for (uint32_t entry_index = 0u;
             entry_index < entries_per_sector && entry_index * entry_size + 40u <= 512u;
             ++entry_index) {
            uint8_t *entry = g_sector_buffer + entry_index * entry_size;
            if (is_guid_zero(entry)) {
                continue;
            }

            uint64_t first_lba = read_u64(entry + 32u);
            if (first_lba == 0u || first_lba > 0xFFFFFFFFULL) {
                continue;
            }

            if (fat32_try_init_at_lba(first_lba)) {
                *out_lba = first_lba;
                return true;
            }
        }
    }

    return false;
}

static void fat32_write_u32_unaligned(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void fat32_parse_bpb_from_sector(const uint8_t *sector, FAT32_BPB *out_bpb) {
    if (!sector || !out_bpb) return;

    uint16_t total16 = read_u16(&sector[19]);
    uint32_t total32 = read_u32(&sector[32]);

    out_bpb->bytes_per_sector    = read_u16(&sector[11]);
    out_bpb->sectors_per_cluster = sector[13];
    out_bpb->reserved_sectors    = read_u16(&sector[14]);
    out_bpb->num_fats            = sector[16];
    out_bpb->fat_size_sectors    = read_u32(&sector[36]);
    out_bpb->root_cluster        = read_u32(&sector[44]);
    out_bpb->fs_info_sector      = read_u16(&sector[48]);
    out_bpb->backup_boot_sector  = read_u16(&sector[50]);
    out_bpb->total_sectors       = (total16 != 0u) ? (uint32_t)total16 : total32;
}

static bool fat32_read_boot_sector_bpb(FAT32_BPB *out_bpb) {
    if (!out_bpb) return false;
    if (!disk_read(g_fat32_partition_lba + 0, g_sector_buffer, 1)) {
        return false;
    }
    fat32_parse_bpb_from_sector(g_sector_buffer, out_bpb);
    return true;
}

static bool fat32_load_fsinfo(void)
{
    g_fsinfo_valid = false;
    g_fsinfo_free_count = 0xFFFFFFFFu;
    g_next_free_cluster_hint = 2u;

    if (bpb.fs_info_sector == 0u || bpb.fs_info_sector >= bpb.reserved_sectors) {
        return false;
    }

    uint32_t lba = g_fat32_partition_lba + bpb.fs_info_sector;
    if (!disk_read(lba, g_sector_buffer, 1)) {
        return false;
    }

    if (read_u32(&g_sector_buffer[0]) != 0x41615252u ||
        read_u32(&g_sector_buffer[484]) != 0x61417272u) {
        return false;
    }

    g_fsinfo_free_count = read_u32(&g_sector_buffer[488]);
    g_next_free_cluster_hint = read_u32(&g_sector_buffer[492]);
    if (g_next_free_cluster_hint < 2u) {
        g_next_free_cluster_hint = 2u;
    }
    g_fsinfo_valid = true;
    return true;
}

static bool fat32_store_fsinfo(void)
{
    if (!g_fsinfo_valid ||
        bpb.fs_info_sector == 0u ||
        bpb.fs_info_sector >= bpb.reserved_sectors) {
        return false;
    }

    uint32_t lba = g_fat32_partition_lba + bpb.fs_info_sector;
    if (!disk_read(lba, g_sector_buffer, 1)) {
        return false;
    }

    fat32_write_u32_unaligned(&g_sector_buffer[0], 0x41615252u);
    fat32_write_u32_unaligned(&g_sector_buffer[484], 0x61417272u);
    fat32_write_u32_unaligned(&g_sector_buffer[488], g_fsinfo_free_count);
    fat32_write_u32_unaligned(&g_sector_buffer[492], g_next_free_cluster_hint);
    g_sector_buffer[508] = 0x00u;
    g_sector_buffer[509] = 0x00u;
    g_sector_buffer[510] = 0x55u;
    g_sector_buffer[511] = 0xAAu;
    return disk_write(lba, g_sector_buffer, 1);
}

static uint32_t fat32_strlen(const char *s)
{
    if (!s) return 0;
    uint32_t n = 0;
    while (s[n]) ++n;
    return n;
}

static char to_upper_ascii(char c) {
    if (c >= 'a' && c <= 'z') return (char)(c - ('a' - 'A'));
    return c;
}

static bool string_case_equal(const char *a, const char *b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (to_upper_ascii(*a) != to_upper_ascii(*b)) return false;
        ++a; ++b;
    }
    return (*a == '\0' && *b == '\0');
}

static bool string_exact_equal(const char *a, const char *b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        ++a; ++b;
    }
    return (*a == '\0' && *b == '\0');
}

static bool fat32_name_equal(const char *a, const char *b) {
    return g_case_sensitive_lookup ? string_exact_equal(a, b)
                                   : string_case_equal(a, b);
}

static bool _fat32_truncate(FAT32_FILE *file, uint32_t new_size) {
    if (!file) return false;
    if (new_size == file->size) return true;

    uint32_t cluster_size = (uint32_t)bpb.sectors_per_cluster * bpb.bytes_per_sector;

    if (new_size == 0u) {
        if (file->first_cluster >= 2u) {
            fat32_free_cluster_chain(file->first_cluster);
            file->first_cluster = 0u;
        }
    } else {
        uint32_t needed = (new_size + cluster_size - 1u) / cluster_size;
        if (!fat32_ensure_cluster_count(file, needed)) return false;

        if (new_size < file->size) {
            uint32_t last_keep;
            if (fat32_get_cluster_at_index_cached(file, needed - 1, &last_keep)) {
                uint32_t tail = fat_get_next_cluster(last_keep);
                fat_set_next_cluster(last_keep, FAT32_EOC_MARKER);
                if (tail >= 2u && tail < 0x0FFFFFF8u) fat32_free_cluster_chain(tail);
            }
        }
    }

    file->size = new_size;
    g_cluster_cache.cluster_value = 0;
    bool ok = fat32_update_file_directory_entry(file);
    return ok;
}

bool fat32_truncate(FAT32_FILE *file, uint32_t new_size) {
    spinlock_lock(&g_fat32_lock);
    bool ret = _fat32_truncate(file, new_size);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

static void string_copy_limit(char *dst, uint32_t dst_size, const char *src) {
    if (!dst || dst_size == 0) return;
    uint32_t i = 0;
    if (src) {
        while (src[i] != '\0' && i + 1 < dst_size) {
            dst[i] = src[i]; ++i;
        }
    }
    dst[i] = '\0';
}

static const char *skip_path_separators(const char *path) {
    if (!path) return NULL;
    while (*path == '/' || *path == '\\') ++path;
    return path;
}

static const char *extract_path_component(const char *path,
                                           char *component,
                                           uint32_t component_size,
                                           bool *has_more) {
    if (!path || !component || component_size == 0 || !has_more) return NULL;
    const char *p = skip_path_separators(path);
    uint32_t len = 0;
    while (*p != '\0' && *p != '/' && *p != '\\') {
        if (len + 1 >= component_size) return NULL;
        component[len++] = *p++;
    }
    component[len] = '\0';
    p = skip_path_separators(p);
    *has_more = (*p != '\0');
    return p;
}

static void fat32_short_name_to_string(const uint8_t *entry, char out_name[13]) {
    uint32_t n = 0;
    uint8_t first_char = entry[0];
    if (first_char == 0x05) first_char = 0xE5;

    for (int i = 0; i < 8; i++) {
        uint8_t ch = (i == 0) ? first_char : entry[i];
        if (ch == ' ') break;
        out_name[n++] = (char)ch;
    }

    bool has_ext = false;
    for (int i = 8; i < 11; i++) { if (entry[i] != ' ') { has_ext = true; break; } }

    if (has_ext) {
        out_name[n++] = '.';
        for (int i = 8; i < 11; i++) {
            if (entry[i] == ' ') break;
            out_name[n++] = (char)entry[i];
        }
    }
    out_name[n] = '\0';
}

static bool fat32_name_is_simple_83(const char *name) {
    if (!name || name[0] == '\0') return false;
    uint32_t base_len = 0, ext_len = 0;
    int seen_dot = 0;
    for (const char *p = name; *p != '\0'; ++p) {
        char ch = *p;
        if (ch == '.' && !seen_dot) { seen_dot = 1; continue; }
        if (ch == '.' || ch == '/' || ch == '\\' || ch == ' ') return false;
        if (seen_dot) ++ext_len; else ++base_len;
    }
    return (base_len > 0 && base_len <= 8 && ext_len <= 3);
}

static bool fat32_build_short_name_83(const char *name, uint8_t out_name[11]) {
    if (!fat32_name_is_simple_83(name) || !out_name) return false;
    for (uint32_t i = 0; i < 11; ++i) out_name[i] = ' ';
    uint32_t base_idx = 0, ext_idx = 0;
    int seen_dot = 0;
    for (const char *p = name; *p != '\0'; ++p) {
        char ch = *p;
        if (ch == '.') { seen_dot = 1; continue; }
        if (ch >= 'a' && ch <= 'z') ch = (char)(ch - ('a' - 'A'));
        if (!seen_dot) out_name[base_idx++] = (uint8_t)ch;
        else           out_name[8u + ext_idx++] = (uint8_t)ch;
    }
    return true;
}

static void fat32_lfn_store_char(char *lfn_name, uint32_t lfn_size,
                                  uint32_t index, uint16_t value) {
    if (!lfn_name || lfn_size == 0 || index >= lfn_size - 1u) return;
    if (value == 0x0000u || value == 0xFFFFu) {
        if (lfn_name[index] == '\0') return;
        lfn_name[index] = '\0';
        return;
    }
    lfn_name[index] = (value < 0x80u) ? (char)value : '?';
}

static bool fat32_decode_lfn_entry(const uint8_t *entry,
                                    char *lfn_name, uint32_t lfn_size) {
    uint8_t order = entry[0] & 0x1Fu;
    if (order == 0 || order > FAT32_MAX_LFN_ORDER) return false;

    uint32_t base = (uint32_t)(order - 1u) * FAT32_LFN_CHARS_PER_ENTRY;
    static const uint8_t offsets[FAT32_LFN_CHARS_PER_ENTRY] = {
        1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30
    };
    for (uint32_t i = 0; i < FAT32_LFN_CHARS_PER_ENTRY; ++i) {
        if (base + i >= lfn_size - 1u) break;
        fat32_lfn_store_char(lfn_name, lfn_size,
                             base + i, read_u16(&entry[offsets[i]]));
    }
    return true;
}

static uint8_t fat32_lfn_checksum(const uint8_t name11[11])
{
    uint8_t sum = 0;
    for (int i = 0; i < 11; ++i)
        sum = (uint8_t)(((sum & 1u) ? 0x80u : 0u) + (sum >> 1) + name11[i]);
    return sum;
}

static uint8_t fat32_lfn_count_needed(const char *name)
{
    uint32_t len = fat32_strlen(name);
    if (len == 0) return 0;
    return (uint8_t)((len + FAT32_LFN_CHARS_PER_ENTRY - 1u) / FAT32_LFN_CHARS_PER_ENTRY);
}

static void fat32_build_lfn_entry(uint8_t *buf,
                                   uint8_t  order,
                                   bool     is_last,
                                   uint8_t  checksum,
                                   const char *name,
                                   uint32_t    name_len)
{
    static const uint8_t char_offsets[FAT32_LFN_CHARS_PER_ENTRY] = {
        1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30
    };

    memset(buf, 0xFF, FAT32_DIR_ENTRY_SIZE);
    buf[0]  = order | (is_last ? 0x40u : 0u);
    buf[11] = FAT32_ATTR_LFN;
    buf[12] = 0x00u;
    buf[13] = checksum;
    buf[26] = 0x00u;
    buf[27] = 0x00u;

    uint32_t base = (uint32_t)(order - 1u) * FAT32_LFN_CHARS_PER_ENTRY;
    bool end_reached = false;

    for (uint32_t i = 0; i < FAT32_LFN_CHARS_PER_ENTRY; ++i) {
        uint32_t pos = base + i;
        uint16_t val;
        if (end_reached) {
            val = 0xFFFFu;
        } else if (pos < name_len) {
            val = (uint8_t)name[pos];
        } else if (pos == name_len) {
            val = 0x0000u;
            end_reached = true;
        } else {
            val = 0xFFFFu;
            end_reached = true;
        }
        buf[char_offsets[i]]     = (uint8_t)(val & 0xFFu);
        buf[char_offsets[i] + 1] = (uint8_t)(val >> 8);
    }
}

static bool fat32_is_valid_83_char(char c)
{
    if ((unsigned char)c < 0x21u) return false;
    switch (c) {
        case '"': case '*': case '+': case ',': case '.':
        case '/': case ':': case ';': case '<': case '=':
        case '>': case '?': case '[': case '\\': case ']':
        case '|':
            return false;
    }
    return true;
}

static uint8_t fat32_sanitize_83_char(char c)
{
    c = to_upper_ascii(c);
    return fat32_is_valid_83_char(c) ? (uint8_t)c : (uint8_t)'_';
}

static uint32_t fat32_u32_to_str(uint32_t n, char buf[10])
{
    if (n == 0) { buf[0] = '0'; return 1u; }
    uint32_t len = 0;
    char tmp[10];
    while (n > 0) { tmp[len++] = (char)('0' + (n % 10u)); n /= 10u; }
    for (uint32_t i = 0; i < len; ++i) buf[i] = tmp[len - 1u - i];
    return len;
}

static uint32_t fat_start_lba(void) { return bpb.reserved_sectors; }

static uint32_t data_start_lba(void) {
    return bpb.reserved_sectors + (bpb.num_fats * bpb.fat_size_sectors);
}

static uint32_t cluster_to_lba(uint32_t cluster) {
    if (cluster < 2u) return 0u;
    uint64_t lba = (uint64_t)data_start_lba() + (uint64_t)(cluster - 2u) * bpb.sectors_per_cluster;
    if (lba > 0xFFFFFFFFULL) return 0u;
    return (uint32_t)lba;
}

static uint32_t fat_get_next_cluster(uint32_t cluster) {
    if (cluster < 2u || cluster >= 0x0FFFFFF8u) return FAT32_EOC_MARKER;

    uint32_t fat_offset = cluster * 4u;
    uint32_t sector = g_fat32_partition_lba + fat_start_lba() + (fat_offset / bpb.bytes_per_sector);
    uint32_t offset = fat_offset % bpb.bytes_per_sector;

    if (g_cached_fat_sector != sector) {
        if (!disk_read(sector, g_fat_cache_buf, 1)) {
            return FAT32_EOC_MARKER;
        }
        g_cached_fat_sector = sector;
    }

    uint32_t val = fat32_read_u32_unaligned(&g_fat_cache_buf[offset]);
    uint32_t next_cluster = val & 0x0FFFFFFFu;

    if (next_cluster == cluster) return FAT32_EOC_MARKER;
    return next_cluster;
}

static uint32_t fat_get_next_cluster_uncached(uint32_t cluster,
                                              uint32_t *cached_sector_io,
                                              uint8_t *fat_sector_buf)
{
    if (cluster < 2u || cluster >= 0x0FFFFFF8u || cached_sector_io == NULL || fat_sector_buf == NULL) {
        return FAT32_EOC_MARKER;
    }

    uint32_t fat_offset = cluster * 4u;
    uint32_t sector = g_fat32_partition_lba + fat_start_lba() + (fat_offset / bpb.bytes_per_sector);
    uint32_t offset = fat_offset % bpb.bytes_per_sector;

    if (*cached_sector_io != sector) {
        if (!disk_read(sector, fat_sector_buf, 1)) {
            return FAT32_EOC_MARKER;
        }
        *cached_sector_io = sector;
    }

    uint32_t val = fat32_read_u32_unaligned(&fat_sector_buf[offset]);
    uint32_t next_cluster = val & 0x0FFFFFFFu;
    if (next_cluster == cluster) return FAT32_EOC_MARKER;
    return next_cluster;
}

static bool fat_set_next_cluster(uint32_t cluster, uint32_t next_val) {
    if (cluster < 2u) return false;

    uint32_t fat_offset = cluster * 4u;
    uint32_t sector_offset = fat_offset / bpb.bytes_per_sector;
    uint32_t offset = fat_offset % bpb.bytes_per_sector;

    next_val &= 0x0FFFFFFFu;

    for (uint32_t i = 0; i < bpb.num_fats; i++) {
        uint32_t sector = g_fat32_partition_lba + fat_start_lba() + (i * bpb.fat_size_sectors) + sector_offset;
        if (!disk_read(sector, g_sector_buffer, 1)) {
            return false;
        }

        uint32_t val = fat32_read_u32_unaligned(&g_sector_buffer[offset]);
        uint32_t tmp = val & 0xF0000000u;
        fat32_write_u32_unaligned(&g_sector_buffer[offset], tmp | next_val);

        if (!disk_write(sector, g_sector_buffer, 1)) {
            return false;
        }
        g_cached_fat_sector = 0xFFFFFFFFu;
    }
    if (next_val == 0u && cluster < g_next_free_cluster_hint) {
        g_next_free_cluster_hint = cluster;
        (void)fat32_store_fsinfo();
    }
    return true;
}

static uint32_t fat32_cluster_size_bytes(void) {
    return (uint32_t)bpb.sectors_per_cluster * bpb.bytes_per_sector;
}

static uint32_t fat32_total_clusters(void) {
    uint32_t fat_entries = (bpb.fat_size_sectors * bpb.bytes_per_sector) / 4u;
    if (bpb.total_sectors <= data_start_lba()) return 0u;
    uint32_t data_sectors = bpb.total_sectors - data_start_lba();
    if (bpb.sectors_per_cluster == 0) return 0u;
    uint32_t data_clusters = data_sectors / bpb.sectors_per_cluster;
    return (fat_entries < data_clusters) ? fat_entries : data_clusters;
}

static bool fat32_zero_cluster(uint32_t cluster) {
    uint32_t lba = g_fat32_partition_lba + cluster_to_lba(cluster);
    if (lba == g_fat32_partition_lba) return false;
    uint32_t cluster_size = fat32_cluster_size_bytes();
    if (cluster_size == 0u || cluster_size > FAT32_CLUSTER_BUFFER_SIZE) return false;
    memset(g_read_buffer, 0, cluster_size);
    return disk_write(lba, g_read_buffer, bpb.sectors_per_cluster);
}

static uint32_t fat32_find_free_cluster(void) {
    uint32_t total = fat32_total_clusters();
    if (total <= 2u) return 0u;

    uint32_t start = g_next_free_cluster_hint;
    if (start < 2u || start >= total) start = 2u;

    for (uint32_t c = start; c < total; ++c) {
        if (fat_get_next_cluster(c) == 0u) {
            g_next_free_cluster_hint = c;
            return c;
        }
    }
    for (uint32_t c = 2u; c < start; ++c) {
        if (fat_get_next_cluster(c) == 0u) {
            g_next_free_cluster_hint = c;
            return c;
        }
    }
    return 0u;
}

static uint32_t fat32_allocate_cluster_zeroed(void) {
    uint32_t cluster = fat32_find_free_cluster();
    if (cluster < 2u) return 0u;
    if (!fat_set_next_cluster(cluster, FAT32_EOC_MARKER)) return 0u;
    if (!fat32_zero_cluster(cluster)) {
        (void)fat_set_next_cluster(cluster, 0u);
        return 0u;
    }
    g_next_free_cluster_hint = cluster + 1u;
    if (g_fsinfo_valid && g_fsinfo_free_count != 0xFFFFFFFFu && g_fsinfo_free_count > 0u) {
        --g_fsinfo_free_count;
    }
    (void)fat32_store_fsinfo();
    return cluster;
}

static bool fat32_free_cluster_chain(uint32_t first_cluster) {
    if (first_cluster < 2u) return true;
    uint32_t cluster = first_cluster;
    uint32_t guard   = fat32_total_clusters();
    uint32_t freed_count = 0u;
    for (uint32_t i = 0u; i < guard; ++i) {
        uint32_t next = fat_get_next_cluster(cluster);
        if (!fat_set_next_cluster(cluster, 0u)) return false;
        ++freed_count;
        if (next < 2u || next >= FAT32_EOC_MARKER) {
            if (g_fsinfo_valid && g_fsinfo_free_count != 0xFFFFFFFFu) {
                g_fsinfo_free_count += freed_count;
            }
            (void)fat32_store_fsinfo();
            return true;
        }
        cluster = next;
    }
    if (g_fsinfo_valid && g_fsinfo_free_count != 0xFFFFFFFFu) {
        g_fsinfo_free_count += freed_count;
    }
    (void)fat32_store_fsinfo();
    return false;
}

static bool fat32_get_last_cluster(const FAT32_FILE *file,
                                    uint32_t *cluster_out, uint32_t *count_out) {
    if (!file || !cluster_out || !count_out || file->first_cluster < 2u) return false;
    uint32_t cluster = file->first_cluster, count = 1u;
    uint32_t guard = fat32_total_clusters();
    for (uint32_t i = 0u; i < guard; ++i) {
        uint32_t next = fat_get_next_cluster(cluster);
        if (next < 2u || next >= FAT32_EOC_MARKER) {
            *cluster_out = cluster; *count_out = count; return true;
        }
        cluster = next; ++count;
    }
    return false;
}

static bool fat32_ensure_cluster_count(FAT32_FILE *file, uint32_t required_clusters) {
    if (!file) return false;
    if (required_clusters == 0u) return true;
    if (file->first_cluster < 2u) {
        uint32_t c = fat32_allocate_cluster_zeroed();
        if (c < 2u) return false;
        file->first_cluster = c;
    }
    uint32_t last = 0u, current = 0u;
    if (!fat32_get_last_cluster(file, &last, &current)) return false;
    while (current < required_clusters) {
        uint32_t nc = fat32_allocate_cluster_zeroed();
        if (nc < 2u) return false;
        if (!fat_set_next_cluster(last, nc)) { (void)fat_set_next_cluster(nc, 0u); return false; }
        last = nc; ++current;
    }
    return true;
}

static bool fat32_get_cluster_at_index_cached(const FAT32_FILE *file_const, uint32_t index, uint32_t *cluster_out) {
    if (!file_const || !cluster_out || file_const->first_cluster < 2u) return false;

    FAT32_FILE *file = (FAT32_FILE *)file_const;
    uint32_t cluster = file->first_cluster;
    uint32_t start_idx = 0;

    if (file->cached_cluster_value >= 2u && file->cached_cluster_index <= index) {
        cluster = file->cached_cluster_value;
        start_idx = file->cached_cluster_index;
    }

    if (cluster >= 0x0FFFFFF8u) return false;

    for (uint32_t i = start_idx; i < index; i++) {
        cluster = fat_get_next_cluster(cluster);
        if (cluster >= 0x0FFFFFF8u) return false;
    }

    file->cached_cluster_index = index;
    file->cached_cluster_value = cluster;
    *cluster_out = cluster;
    return true;
}

static bool fat32_get_cluster_at_index_uncached(const FAT32_FILE *file,
                                                uint32_t index,
                                                uint32_t *cluster_out,
                                                uint32_t *cached_sector_io,
                                                uint8_t *fat_sector_buf)
{
    if (!file || !cluster_out || file->first_cluster < 2u) return false;

    uint32_t cluster = file->first_cluster;
    for (uint32_t i = 0u; i < index; ++i) {
        cluster = fat_get_next_cluster_uncached(cluster, cached_sector_io, fat_sector_buf);
        if (cluster < 2u || cluster >= FAT32_EOC_MARKER) {
            return false;
        }
    }

    *cluster_out = cluster;
    return true;
}

static uint32_t fat32_count_contiguous_clusters_uncached(uint32_t start_cluster,
                                                         uint32_t max_clusters,
                                                         uint32_t *cached_sector_io,
                                                         uint8_t *fat_sector_buf)
{
    if (start_cluster < 2u || max_clusters == 0u) return 0u;

    uint32_t count = 1u;
    uint32_t cluster = start_cluster;
    while (count < max_clusters) {
        uint32_t next = fat_get_next_cluster_uncached(cluster, cached_sector_io, fat_sector_buf);
        if (next != cluster + 1u || next < 2u || next >= FAT32_EOC_MARKER) {
            break;
        }
        cluster = next;
        ++count;
    }
    return count;
}

static bool fat32_dir_pos_advance(uint32_t *sector, uint16_t *offset)
{
    if (!sector || !offset) return false;

    uint16_t next_off = (uint16_t)(*offset + FAT32_DIR_ENTRY_SIZE);
    if (next_off < (uint16_t)bpb.bytes_per_sector) {
        *offset = next_off;
        return true;
    }

    *offset = 0u;
    uint32_t data_lba      = g_fat32_partition_lba + data_start_lba();
    if (*sector < data_lba) return false;

    uint32_t rel           = *sector - data_lba;
    uint32_t sec_in_cluster = rel % bpb.sectors_per_cluster;

    if (sec_in_cluster + 1u < bpb.sectors_per_cluster) {
        (*sector)++;
        return true;
    }

    uint32_t cluster = rel / bpb.sectors_per_cluster + 2u;
    uint32_t next_c  = fat_get_next_cluster(cluster);
    if (next_c < 2u || next_c >= FAT32_EOC_MARKER) return false;
    *sector = g_fat32_partition_lba + cluster_to_lba(next_c);
    return true;
}

static bool fat32_dir_pos_prev(uint32_t dir_cluster,
                                uint32_t *sector, uint16_t *offset)
{
    if (!sector || !offset) return false;

    if (*offset >= FAT32_DIR_ENTRY_SIZE) {
        *offset = (uint16_t)(*offset - FAT32_DIR_ENTRY_SIZE);
        return true;
    }

    uint32_t data_lba = g_fat32_partition_lba + data_start_lba();
    if (*sector <= data_lba) return false;

    uint32_t rel            = *sector - data_lba;
    uint32_t cur_cluster    = rel / bpb.sectors_per_cluster + 2u;
    uint32_t sec_in_cluster = rel % bpb.sectors_per_cluster;

    if (sec_in_cluster > 0u) {
        (*sector)--;
        *offset = (uint16_t)(bpb.bytes_per_sector - FAT32_DIR_ENTRY_SIZE);
        return true;
    }

    if (cur_cluster == dir_cluster) return false;

    uint32_t prev = dir_cluster;
    uint32_t guard = fat32_total_clusters();
    for (uint32_t i = 0u; i < guard; ++i) {
        uint32_t nc = fat_get_next_cluster(prev);
        if (nc == cur_cluster) {
            *sector = g_fat32_partition_lba + cluster_to_lba(prev) + bpb.sectors_per_cluster - 1u;
            *offset = (uint16_t)(bpb.bytes_per_sector - FAT32_DIR_ENTRY_SIZE);
            return true;
        }
        if (nc < 2u || nc >= FAT32_EOC_MARKER) break;
        prev = nc;
    }
    return false;
}

static bool fat32_short_name_exists_impl(uint32_t dir_cluster,
                                          const uint8_t name11[11])
{
    uint32_t cluster = dir_cluster;
    uint32_t guard   = fat32_total_clusters();

    for (uint32_t g = 0u; g < guard; ++g) {
        if (cluster < 2u) break;
        uint32_t lba = g_fat32_partition_lba + cluster_to_lba(cluster);
        if (lba == g_fat32_partition_lba) return false;

        for (uint8_t sec = 0u; sec < bpb.sectors_per_cluster; ++sec) {
            if (!disk_read(lba + sec, g_sector_buffer, 1)) return false;
            for (uint32_t off = 0u; off < bpb.bytes_per_sector; off += FAT32_DIR_ENTRY_SIZE) {
                uint8_t first = g_sector_buffer[off];
                if (first == 0x00u) goto next_cluster;
                if (first == 0xE5u) continue;
                uint8_t attr = g_sector_buffer[off + 11u];
                if (attr == FAT32_ATTR_LFN) continue;
                if (attr & FAT32_ATTR_VOLUME_ID) continue;
                bool match = true;
                for (int i = 0; i < 11; ++i) {
                    if (g_sector_buffer[off + (uint32_t)i] != name11[i]) { match = false; break; }
                }
                if (match) return true;
            }
        }
next_cluster:
        uint32_t next = fat_get_next_cluster(cluster);
        if (next < 2u || next >= FAT32_EOC_MARKER) break;
        cluster = next;
    }
    return false;
}

static bool fat32_gen_short_name(const char *long_name,
                                  uint32_t    dir_cluster,
                                  uint8_t     out11[11])
{
    if (!long_name || !out11) return false;

    if (fat32_name_is_simple_83(long_name))
        return fat32_build_short_name_83(long_name, out11);

    const char *last_dot = NULL;
    for (const char *p = long_name; *p; ++p)
        if (*p == '.') last_dot = p;

    uint8_t ext[3] = {' ', ' ', ' '};
    if (last_dot) {
        uint32_t ei = 0u;
        for (const char *p = last_dot + 1u; *p && ei < 3u; ++p) {
            if (*p == ' ' || *p == '.') continue;
            ext[ei++] = fat32_sanitize_83_char(*p);
        }
    }

    uint8_t base_chars[6];
    uint32_t base_len = 0u;
    for (const char *p = long_name;
         (last_dot ? (p < last_dot) : (*p != '\0')) && base_len < 6u;
         ++p) {
        if (*p == ' ' || *p == '.') continue;
        base_chars[base_len++] = fat32_sanitize_83_char(*p);
    }
    if (base_len == 0u) base_chars[base_len++] = '_';

    for (uint32_t n = 1u; n <= 9999u; ++n) {
        char     num_str[10];
        uint32_t num_len    = fat32_u32_to_str(n, num_str);
        uint32_t suffix_len = 1u + num_len;
        uint32_t base_part  = (8u > suffix_len) ? (8u - suffix_len) : 0u;
        if (base_part > base_len) base_part = base_len;

        uint8_t candidate[11];
        for (int i = 0; i < 11; ++i) candidate[i] = ' ';

        for (uint32_t i = 0u; i < base_part; ++i) candidate[i] = base_chars[i];

        candidate[base_part] = '~';
        for (uint32_t i = 0u; i < num_len; ++i)
            candidate[base_part + 1u + i] = (uint8_t)num_str[i];

        for (uint32_t i = 0u; i < 3u; ++i) candidate[8u + i] = ext[i];

        if (!fat32_short_name_exists_impl(dir_cluster, candidate)) {
            memcpy(out11, candidate, 11u);
            return true;
        }
    }
    return false;
}

static bool fat32_find_consecutive_free(uint32_t  dir_cluster,
                                         uint32_t  count,
                                         uint32_t *first_sector_out,
                                         uint16_t *first_offset_out)
{
    if (!first_sector_out || !first_offset_out || count == 0u || dir_cluster < 2u)
        return false;

    uint32_t run_sector = 0u;
    uint16_t run_offset = 0u;
    uint32_t run_len    = 0u;

    uint32_t cluster = dir_cluster;
    uint32_t guard   = fat32_total_clusters();
    uint32_t last_cluster = dir_cluster;

    for (uint32_t g = 0u; g <= guard; ++g) {
        if (cluster < 2u) break;
        last_cluster = cluster;

        uint32_t lba = g_fat32_partition_lba + cluster_to_lba(cluster);
        if (lba == g_fat32_partition_lba) return false;

        for (uint8_t sec = 0u; sec < bpb.sectors_per_cluster; ++sec) {
            if (!disk_read(lba + sec, g_sector_buffer, 1)) return false;

            for (uint32_t off = 0u; off < bpb.bytes_per_sector; off += FAT32_DIR_ENTRY_SIZE) {
                uint8_t first = g_sector_buffer[off];
                bool is_free  = (first == 0x00u || first == 0xE5u);

                if (is_free) {
                    if (run_len == 0u) {
                        run_sector = lba + sec;
                        run_offset = (uint16_t)off;
                    }
                    ++run_len;
                    if (run_len >= count) {
                        *first_sector_out = run_sector;
                        *first_offset_out = run_offset;
                        return true;
                    }

                    if (first == 0x00u) {
                        uint32_t entries_left_in_sector =
                            (bpb.bytes_per_sector - off) / FAT32_DIR_ENTRY_SIZE - 1u;
                        uint32_t entries_left =
                            entries_left_in_sector +
                            (uint32_t)(bpb.sectors_per_cluster - sec - 1u) *
                            (bpb.bytes_per_sector / FAT32_DIR_ENTRY_SIZE);
                        if (run_len + entries_left >= count) {
                            *first_sector_out = run_sector;
                            *first_offset_out = run_offset;
                            return true;
                        }
                        goto need_new_cluster;
                    }
                } else {
                    run_len = 0u;
                }
            }
        }

        uint32_t next = fat_get_next_cluster(cluster);
        if (next < 2u || next >= FAT32_EOC_MARKER) goto need_new_cluster;
        cluster = next;
    }

need_new_cluster:;
    uint32_t new_cluster = fat32_allocate_cluster_zeroed();
    if (new_cluster < 2u) return false;

    if (!fat_set_next_cluster(last_cluster, new_cluster)) {
        (void)fat_set_next_cluster(new_cluster, 0u);
        return false;
    }

    uint32_t entries_in_new =
        ((uint32_t)bpb.sectors_per_cluster * bpb.bytes_per_sector) / FAT32_DIR_ENTRY_SIZE;

    if (run_len + entries_in_new >= count) {
        *first_sector_out = (run_len > 0u) ? run_sector : (g_fat32_partition_lba + cluster_to_lba(new_cluster));
        *first_offset_out = (run_len > 0u) ? run_offset : 0u;
        return (*first_sector_out != g_fat32_partition_lba);
    }
    return false;
}

static bool fat32_write_entry_sequence(uint32_t    first_sector,
                                        uint16_t    first_offset,
                                        const char *long_name,
                                        const uint8_t short_name83[11],
                                        uint8_t     attr,
                                        uint32_t    first_cluster,
                                        uint32_t    file_size)
{
    uint32_t sector = first_sector;
    uint16_t offset = first_offset;

    uint32_t name_len  = fat32_strlen(long_name);
    bool     needs_lfn = !fat32_name_is_simple_83(long_name);
    uint8_t  lfn_count = needs_lfn ? fat32_lfn_count_needed(long_name) : 0u;
    uint8_t  checksum  = fat32_lfn_checksum(short_name83);

    for (uint8_t k = 0u; k < lfn_count; ++k) {
        uint8_t order   = (uint8_t)(lfn_count - k);
        bool    is_last = (k == 0u);

        uint8_t lfn_buf[FAT32_DIR_ENTRY_SIZE];
        fat32_build_lfn_entry(lfn_buf, order, is_last, checksum, long_name, name_len);

        if (!disk_read(sector, g_sector_buffer, 1)) return false;
        memcpy(&g_sector_buffer[offset], lfn_buf, FAT32_DIR_ENTRY_SIZE);
        if (!disk_write(sector, g_sector_buffer, 1)) return false;

        if (k + 1u < lfn_count) {
            if (!fat32_dir_pos_advance(&sector, &offset)) return false;
        }
    }

    if (lfn_count > 0u) {
        if (!fat32_dir_pos_advance(&sector, &offset)) return false;
    }

    if (!disk_read(sector, g_sector_buffer, 1)) return false;
    uint8_t *entry = &g_sector_buffer[offset];
    memset(entry, 0, FAT32_DIR_ENTRY_SIZE);
    memcpy(entry, short_name83, 11u);
    entry[11] = attr;

    entry[20] = (uint8_t)((first_cluster >> 16) & 0xFFu);
    entry[21] = (uint8_t)((first_cluster >> 24) & 0xFFu);
    entry[26] = (uint8_t)( first_cluster        & 0xFFu);
    entry[27] = (uint8_t)((first_cluster >>  8) & 0xFFu);

    entry[28] = (uint8_t)( file_size        & 0xFFu);
    entry[29] = (uint8_t)((file_size >>  8) & 0xFFu);
    entry[30] = (uint8_t)((file_size >> 16) & 0xFFu);
    entry[31] = (uint8_t)((file_size >> 24) & 0xFFu);
    return disk_write(sector, g_sector_buffer, 1);
}

static bool fat32_delete_preceding_lfn(uint32_t dir_cluster,
                                        uint32_t short_name_sector,
                                        uint16_t short_name_offset,
                                        uint8_t  lfn_count)
{
    if (lfn_count == 0u) return true;

    uint32_t sector = short_name_sector;
    uint16_t offset = short_name_offset;

    for (uint8_t i = 0u; i < lfn_count; ++i) {
        if (!fat32_dir_pos_prev(dir_cluster, &sector, &offset)) {
            return false;
        }
        if (!disk_read(sector, g_sector_buffer, 1)) return false;
        g_sector_buffer[offset] = 0xE5u;
        if (!disk_write(sector, g_sector_buffer, 1)) return false;
    }
    return true;
}

static void fat32_fill_file_from_entry(const uint8_t *entry,
                                        const char    *resolved_name,
                                        FAT32_FILE    *file,
                                        uint32_t       entry_sector,
                                        uint16_t       entry_offset,
                                        uint8_t        lfn_entry_count,
                                        uint32_t       dir_cluster)
{
    uint16_t high = read_u16(&entry[20]);
    uint16_t low  = read_u16(&entry[26]);
    file->first_cluster    = ((uint32_t)high << 16) | low;
    file->size             = read_u32(&entry[28]);
    file->attributes       = entry[11];
    file->dir_entry_sector = entry_sector;
    file->dir_entry_offset = entry_offset;
    file->lfn_entry_count  = lfn_entry_count;
    file->dir_cluster      = dir_cluster;
    file->cached_cluster_index = 0;
    file->cached_cluster_value = 0;
    string_copy_limit(file->name, sizeof(file->name), resolved_name);
}

static bool fat32_update_file_directory_entry(const FAT32_FILE *file) {
    if (!file || file->dir_entry_sector == 0u ||
        file->dir_entry_offset + FAT32_DIR_ENTRY_SIZE > bpb.bytes_per_sector)
        return false;
    if (!disk_read(file->dir_entry_sector, g_sector_buffer, 1)) return false;

    uint8_t *entry = &g_sector_buffer[file->dir_entry_offset];
    entry[20] = (uint8_t)((file->first_cluster >> 16) & 0xFFu);
    entry[21] = (uint8_t)((file->first_cluster >> 24) & 0xFFu);
    entry[26] = (uint8_t)( file->first_cluster        & 0xFFu);
    entry[27] = (uint8_t)((file->first_cluster >>  8) & 0xFFu);
    entry[28] = (uint8_t)( file->size        & 0xFFu);
    entry[29] = (uint8_t)((file->size >>  8) & 0xFFu);
    entry[30] = (uint8_t)((file->size >> 16) & 0xFFu);
    entry[31] = (uint8_t)((file->size >> 24) & 0xFFu);
    return disk_write(file->dir_entry_sector, g_sector_buffer, 1);
}

static bool fat32_lookup_entry_in_directory(uint32_t    dir_cluster,
                                             const char *target_name,
                                             FAT32_FILE *out_file)
{
    if (dir_cluster < 2u || !target_name || !out_file || target_name[0] == '\0')
        return false;

    char    lfn_name[FAT32_MAX_NAME_LEN];
    int     lfn_valid       = 0;
    uint8_t lfn_entry_count = 0u;
    uint8_t lfn_expected_order = 0u;
    uint8_t lfn_expected_checksum = 0u;
    memset(lfn_name, 0, sizeof(lfn_name));

    uint32_t cluster      = dir_cluster;
    uint32_t total        = fat32_total_clusters();
    uint32_t max_clusters = (total > 65536u) ? 65536u : total;

    for (uint32_t c = 0u; c < max_clusters; ++c) {
        uint32_t lba = g_fat32_partition_lba + cluster_to_lba(cluster);
        if (lba == g_fat32_partition_lba) {
            return false;
        }
        for (uint8_t sec = 0u; sec < bpb.sectors_per_cluster; ++sec) {
            if (!disk_read(lba + sec, g_sector_buffer, 1)) {
                return false;
            }

            for (uint32_t i = 0u; i < bpb.bytes_per_sector; i += FAT32_DIR_ENTRY_SIZE) {
                const uint8_t *entry = &g_sector_buffer[i];
                uint8_t first = entry[0];

                if (first == 0x00u) {
                    return false;
                }
                if (first == 0xE5u) {
                    lfn_valid = 0; lfn_entry_count = 0;
                    lfn_expected_order = 0u;
                    lfn_expected_checksum = 0u;
                    memset(lfn_name, 0, sizeof(lfn_name));
                    continue;
                }

                uint8_t attr = entry[11];

                if (attr == FAT32_ATTR_LFN) {
                    uint8_t seq   = entry[0];
                    uint8_t order = seq & 0x1Fu;
                    uint8_t checksum = entry[13];
                    if (order == 0u || order > FAT32_MAX_LFN_ORDER) {
                        lfn_valid = 0; lfn_entry_count = 0;
                        lfn_expected_order = 0u;
                        lfn_expected_checksum = 0u;
                        memset(lfn_name, 0, sizeof(lfn_name));
                        continue;
                    }
                    if (seq & 0x40u) {
                        memset(lfn_name, 0, sizeof(lfn_name));
                        lfn_valid = 1;
                        lfn_entry_count = 0;
                        lfn_expected_order = order;
                        lfn_expected_checksum = checksum;
                    } else {
                        if (!lfn_valid ||
                            lfn_expected_order == 0u ||
                            (uint8_t)(order + 1u) != lfn_expected_order ||
                            checksum != lfn_expected_checksum) {
                            lfn_valid = 0; lfn_entry_count = 0;
                            lfn_expected_order = 0u;
                            lfn_expected_checksum = 0u;
                            memset(lfn_name, 0, sizeof(lfn_name));
                            continue;
                        }
                        lfn_expected_order = order;
                    }
                    if (!lfn_valid ||
                        !fat32_decode_lfn_entry(entry, lfn_name, sizeof(lfn_name))) {
                        lfn_valid = 0; lfn_entry_count = 0;
                        lfn_expected_order = 0u;
                        lfn_expected_checksum = 0u;
                        memset(lfn_name, 0, sizeof(lfn_name));
                    } else if (lfn_entry_count < 0xFFu) {
                        ++lfn_entry_count;
                    }
                    continue;
                }

                if (attr & FAT32_ATTR_VOLUME_ID) {
                    lfn_valid = 0; lfn_entry_count = 0;
                    lfn_expected_order = 0u;
                    lfn_expected_checksum = 0u;
                    memset(lfn_name, 0, sizeof(lfn_name));
                    continue;
                }

                char short_name[13];
                fat32_short_name_to_string(entry, short_name);
                uint8_t short_checksum = fat32_lfn_checksum(entry);
                int lfn_chain_ok =
                    lfn_valid &&
                    lfn_expected_order == 1u &&
                    lfn_entry_count > 0u &&
                    lfn_name[0] != '\0' &&
                    short_checksum == lfn_expected_checksum;
                const char *candidate =
                    lfn_chain_ok ? lfn_name : short_name;

                if (fat32_name_equal(candidate, target_name)) {
                    fat32_fill_file_from_entry(entry, candidate, out_file,
                                               lba + sec, (uint16_t)i,
                                               lfn_chain_ok ? lfn_entry_count : 0u,
                                               dir_cluster);
                    return true;
                }

                lfn_valid = 0; lfn_entry_count = 0;
                lfn_expected_order = 0u;
                lfn_expected_checksum = 0u;
                memset(lfn_name, 0, sizeof(lfn_name));
            }
        }
        uint32_t next = fat_get_next_cluster(cluster);
        if (next < 2u || next >= 0x0FFFFFF8u) break;
        cluster = next;
    }
    if (dir_cluster == bpb.root_cluster) {
        uint32_t dump_lba = g_fat32_partition_lba + cluster_to_lba(bpb.root_cluster);
        uint8_t *dump_buf = g_read_buffer;
        if (disk_read(dump_lba, dump_buf, 1)) {
            for (uint32_t i = 0u; i < 4u; ++i) {
                const uint8_t *entry = dump_buf + i * 32u;
                if (entry[0] == 0x00u || entry[0] == 0xE5u) continue;
                if ((entry[11] & 0x0Fu) == 0x0Fu) continue;
                if (entry[11] & 0x08u) continue;
                char ename[13];
                fat32_short_name_to_string(entry, ename);
            }
        }
    }
    return false;
}

static bool fat32_validate_bpb(void) {
    if (bpb.bytes_per_sector < FAT32_MIN_SECTOR_SIZE ||
        bpb.bytes_per_sector > FAT32_MAX_SECTOR_SIZE) {
        return false;
    }
    if (bpb.sectors_per_cluster == 0 ||
        bpb.sectors_per_cluster > FAT32_MAX_SECTORS_PER_CLUSTER) {
        return false;
    }
    if (bpb.reserved_sectors == 0 ||
        bpb.reserved_sectors > FAT32_MAX_RESERVED_SECTORS) {
        return false;
    }
    if (bpb.num_fats == 0 || bpb.num_fats > FAT32_MAX_NUM_FATS) {
        return false;
    }
    if (bpb.fat_size_sectors == 0 ||
        bpb.fat_size_sectors > FAT32_MAX_FAT_SIZE_SECTORS) {
        return false;
    }
    if (bpb.num_fats > FAT32_MAX_FAT_SIZE_SECTORS / bpb.fat_size_sectors) {
        return false;
    }
    if (bpb.root_cluster < 2u) {
        return false;
    }
    uint32_t cluster_size = (uint32_t)bpb.sectors_per_cluster * bpb.bytes_per_sector;
    if (cluster_size == 0u || cluster_size > FAT32_CLUSTER_BUFFER_SIZE) {
        return false;
    }
    if (data_start_lba() == 0u &&
        bpb.reserved_sectors + bpb.num_fats * bpb.fat_size_sectors > 0u) {
        return false;
    }
    uint16_t boot_sig = (uint16_t)g_sector_buffer[510] |
                        ((uint16_t)g_sector_buffer[511] << 8);
    if (boot_sig != 0xAA55u) {
        return false;
    }
    bool has_fat_string = false;
    if (memcmp(g_sector_buffer + 82u, "FAT32   ", 8u) == 0) {
        has_fat_string = true;
    }
    if (memcmp(g_sector_buffer + 54u, "FAT16   ", 8u) == 0) {
        has_fat_string = true;
    }
    if (memcmp(g_sector_buffer + 54u, "FAT12   ", 8u) == 0) {
        has_fat_string = true;
    }
    if (!has_fat_string) {
        return false;
    }
    return true;
}

static bool fat32_lookup_path(const char *path, FAT32_FILE *out_entry) {
    if (!path || !out_entry) return false;
    const char *cursor = skip_path_separators(path);
    if (!cursor || *cursor == '\0') return false;

    uint32_t   current_dir_cluster = bpb.root_cluster;
    FAT32_FILE current_entry;
    char       component[FAT32_MAX_NAME_LEN];
    bool       has_more = false;

    while (1) {
        const char *next = extract_path_component(cursor, component,
                                                   sizeof(component), &has_more);
        if (!next || component[0] == '\0') return false;
        if (!fat32_lookup_entry_in_directory(current_dir_cluster, component,
                                              &current_entry)) return false;
        if (!has_more) { *out_entry = current_entry; return true; }
        if ((current_entry.attributes & FAT32_ATTR_DIRECTORY) == 0) return false;
        if (current_entry.first_cluster < 2u) return false;
        current_dir_cluster = current_entry.first_cluster;
        cursor = next;
    }
}

uint32_t fat32_get_file_size(FAT32_FILE *file) {
    return file ? file->size : 0u;
}

static bool fat32_seek_to_offset(const FAT32_FILE *file, uint32_t offset,
                                  uint32_t cluster_size,
                                  uint32_t *cluster_out,
                                  uint32_t *offset_in_cluster_out)
{
    if (!file || !cluster_out || !offset_in_cluster_out || cluster_size == 0u)
        return false;

    uint32_t index = offset / cluster_size;

    if (!fat32_get_cluster_at_index_cached(file, index, cluster_out))
        return false;

    *offset_in_cluster_out = offset % cluster_size;
    return true;
}

static bool fat32_write_existing_range(FAT32_FILE *file, uint32_t offset,
                                        const uint8_t *buffer, uint32_t size) {
    if (!file || !buffer) return false;
    if (size == 0u) return true;

    uint32_t cluster_size = fat32_cluster_size_bytes();
    if (cluster_size == 0u || cluster_size > FAT32_CLUSTER_BUFFER_SIZE) {
        return false;
    }
    if (file->first_cluster < 2u) {
        return false;
    }

    uint32_t cluster = 0u, offset_in_cluster = 0u;
    if (!fat32_seek_to_offset(file, offset, cluster_size, &cluster, &offset_in_cluster)) {
        return false;
    }

    uint8_t fat_sector_buf[FAT32_MAX_SECTOR_SIZE];
    uint32_t cached_fat_sector = 0xFFFFFFFFu;
    uint32_t bytes_left = size;
    while (bytes_left) {
        uint32_t lba = g_fat32_partition_lba + cluster_to_lba(cluster);
        if (lba == g_fat32_partition_lba) {
            return false;
        }

        uint32_t bytes_in_cluster = cluster_size - offset_in_cluster;
        uint32_t n_write = bytes_left > bytes_in_cluster ? bytes_in_cluster : bytes_left;

        if (offset_in_cluster == 0u && n_write == cluster_size) {
            uint32_t full_clusters = bytes_left / cluster_size;
            if (full_clusters > 512u) full_clusters = 512u;

            uint32_t contiguous_clusters =
                fat32_count_contiguous_clusters_uncached(cluster,
                                                         full_clusters,
                                                         &cached_fat_sector,
                                                         fat_sector_buf);
            if (contiguous_clusters == 0u) {
                return false;
            }

            uint32_t sectors_to_write =
                contiguous_clusters * (uint32_t)bpb.sectors_per_cluster;
            uint32_t bytes_to_write = contiguous_clusters * cluster_size;
            if (!disk_write(lba, buffer, sectors_to_write)) {
                return false;
            }
            buffer += bytes_to_write;
            bytes_left -= bytes_to_write;
            offset_in_cluster = 0u;
            if (bytes_left != 0u) {
                uint32_t last_cluster = cluster + contiguous_clusters - 1u;
                uint32_t next = fat_get_next_cluster_uncached(last_cluster,
                                                              &cached_fat_sector,
                                                              fat_sector_buf);
                if (next < 2u || next >= FAT32_EOC_MARKER) {
                    return false;
                }
                cluster = next;
            }
            continue;
        } else {
            if (!disk_read(lba, g_read_buffer, bpb.sectors_per_cluster)) {
                return false;
            }
            memcpy(g_read_buffer + offset_in_cluster, buffer, n_write);
            if (!disk_write(lba, g_read_buffer, bpb.sectors_per_cluster)) {
                return false;
            }
        }
        buffer += n_write; bytes_left -= n_write; offset_in_cluster = 0u;
        if (bytes_left) {
            uint32_t next = fat_get_next_cluster_uncached(cluster,
                                                          &cached_fat_sector,
                                                          fat_sector_buf);
            if (next < 2u || next >= FAT32_EOC_MARKER) {
                return false;
            }
            cluster = next;
        }
    }
    return true;
}

static bool fat32_zero_fill_range(FAT32_FILE *file, uint32_t offset, uint32_t size) {
    if (!file || size == 0u) return true;
    static uint8_t zero_buf[FAT32_CLUSTER_BUFFER_SIZE] __attribute__((aligned(4096)));
    uint32_t max_chunk = fat32_cluster_size_bytes();
    if (max_chunk == 0u || max_chunk > (uint32_t)sizeof(zero_buf)) {
        max_chunk = (uint32_t)sizeof(zero_buf);
    }
    uint32_t remaining = size, cursor = offset;
    while (remaining > 0u) {
        uint32_t chunk = remaining > max_chunk ? max_chunk : remaining;
        if (!fat32_write_existing_range(file, cursor, zero_buf, chunk)) return false;
        cursor += chunk; remaining -= chunk;
    }
    return true;
}

bool fat32_read_at(FAT32_FILE *file, uint32_t offset, uint8_t *buffer, uint32_t size) {
    if (!file || !buffer) return false;

    spinlock_lock(&g_fat32_lock);
    FAT32_FILE local_file = *file;
    spinlock_unlock(&g_fat32_lock);

    if (offset > local_file.size) return false;
    if (size == 0u) return true;
    if (size > local_file.size - offset) size = local_file.size - offset;
    if (size == 0u) return true;

    uint32_t cluster_size = fat32_cluster_size_bytes();
    if (cluster_size == 0u || cluster_size > FAT32_CLUSTER_BUFFER_SIZE) return false;

    uint8_t fat_sector_buf[FAT32_MAX_SECTOR_SIZE];
    uint32_t cached_fat_sector = 0xFFFFFFFFu;
    uint8_t *cluster_buf = NULL;

    uint32_t bytes_left = size;
    uint32_t in_cluster_off = offset % cluster_size;
    uint32_t cluster = 0u;
    bool ok = true;

    if (!fat32_get_cluster_at_index_uncached(&local_file,
                                             offset / cluster_size,
                                             &cluster,
                                             &cached_fat_sector,
                                             fat_sector_buf)) {
        return false;
    }

    while (bytes_left > 0u) {
        uint32_t lba = g_fat32_partition_lba + cluster_to_lba(cluster);
        if (lba == g_fat32_partition_lba) {
            ok = false;
            break;
        }

        uint32_t can_read = cluster_size - in_cluster_off;
        uint32_t chunk = (bytes_left < can_read) ? bytes_left : can_read;

        if (in_cluster_off == 0u && chunk == cluster_size) {
            uint32_t full_clusters = bytes_left / cluster_size;
            if (full_clusters > 512u) full_clusters = 512u;

            uint32_t contiguous_clusters =
                fat32_count_contiguous_clusters_uncached(cluster,
                                                         full_clusters,
                                                         &cached_fat_sector,
                                                         fat_sector_buf);
            if (contiguous_clusters > 0u) {
                uint32_t sectors_to_read = contiguous_clusters * (uint32_t)bpb.sectors_per_cluster;
                uint32_t bytes_to_read = contiguous_clusters * cluster_size;
                if (!disk_read(lba, buffer, sectors_to_read)) {
                    ok = false;
                    break;
                }
                buffer += bytes_to_read;
                bytes_left -= bytes_to_read;
                if (bytes_left == 0u) {
                    break;
                }

                uint32_t last_cluster = cluster + contiguous_clusters - 1u;
                cluster = fat_get_next_cluster_uncached(last_cluster,
                                                        &cached_fat_sector,
                                                        fat_sector_buf);
                if (cluster < 2u || cluster >= FAT32_EOC_MARKER) {
                    ok = false;
                    break;
                }
                in_cluster_off = 0u;
                continue;
            }
        }

        if (cluster_buf == NULL) {
            cluster_buf = (uint8_t *)malloc(cluster_size);
            if (cluster_buf == NULL) {
                ok = false;
                break;
            }
        }
        if (!disk_read(lba, cluster_buf, bpb.sectors_per_cluster)) {
            ok = false;
            break;
        }
        memcpy(buffer, cluster_buf + in_cluster_off, chunk);
        buffer += chunk;
        bytes_left -= chunk;
        in_cluster_off += chunk;
        if (bytes_left != 0u && in_cluster_off == cluster_size) {
            cluster = fat_get_next_cluster_uncached(cluster,
                                                    &cached_fat_sector,
                                                    fat_sector_buf);
            if (cluster < 2u || cluster >= FAT32_EOC_MARKER) {
                ok = false;
                break;
            }
            in_cluster_off = 0u;
        }
    }

    if (cluster_buf != NULL) {
        free(cluster_buf);
    }
    return ok;
}

static bool _fat32_write_at(FAT32_FILE *file, uint32_t offset, const uint8_t *buffer, uint32_t size) {
    if (!file) return false;
    if (size == 0u) return _fat32_truncate(file, offset);
    if (!buffer) return false;
    if (offset > 0xFFFFFFFFu - size) return false;

    uint32_t final_size = offset + size;
    uint32_t old_size   = file->size;

    if (final_size > file->size) {
        if (!_fat32_truncate(file, final_size)) return false;
        if (offset > old_size) {
            if (!fat32_zero_fill_range(file, old_size, offset - old_size)) return false;
        }
    }
    if (offset > file->size || size > file->size - offset) return false;
    return fat32_write_existing_range(file, offset, buffer, size);
}

bool fat32_write_at(FAT32_FILE *file, uint32_t offset, const uint8_t *buffer, uint32_t size) {
    spinlock_lock(&g_fat32_lock);
    bool ret = _fat32_write_at(file, offset, buffer, size);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

bool fat32_write_file(FAT32_FILE *file, const uint8_t *buffer) {
    if (!file || !buffer) return false;
    return fat32_write_at(file, 0u, buffer, file->size);
}

bool fat32_read_file(FAT32_FILE *file, uint8_t *buffer) {
    if (!file || !buffer) return false;
    return fat32_read_at(file, 0u, buffer, file->size);
}

static bool fat32_split_parent_and_name(const char *path,
                                         char *parent_out, uint32_t parent_size,
                                         char *name_out,   uint32_t name_size) {
    if (!path || !parent_out || !name_out || parent_size == 0u || name_size == 0u) return false;
    const char *clean = skip_path_separators(path);
    if (!clean || clean[0] == '\0') return false;

    uint32_t len = fat32_strlen(clean);
    if (len + 1u > FAT32_PATH_MAX) return false;

    char tmp[FAT32_PATH_MAX];
    string_copy_limit(tmp, sizeof(tmp), clean);

    int32_t sep = -1;
    for (int32_t i = (int32_t)len - 1; i >= 0; --i) {
        if (tmp[i] == '/' || tmp[i] == '\\') { sep = i; break; }
    }
    if (sep < 0) {
        parent_out[0] = '/'; parent_out[1] = '\0';
        string_copy_limit(name_out, name_size, tmp);
        return name_out[0] != '\0';
    }
    tmp[sep] = '\0';
    string_copy_limit(name_out, name_size, tmp + sep + 1);
    if (name_out[0] == '\0') return false;
    if (tmp[0] == '\0') { parent_out[0] = '/'; parent_out[1] = '\0'; }
    else string_copy_limit(parent_out, parent_size, tmp);
    return true;
}

static bool fat32_resolve_directory_cluster(const char *path, uint32_t *cluster_out) {
    if (!cluster_out || !path) return false;
    const char *clean = skip_path_separators(path);
    if (!clean || clean[0] == '\0') { *cluster_out = bpb.root_cluster; return true; }

    FAT32_FILE dir;
    if (!fat32_lookup_path(path, &dir)) return false;
    if ((dir.attributes & FAT32_ATTR_DIRECTORY) == 0u) return false;
    if (dir.first_cluster < 2u) return false;
    *cluster_out = dir.first_cluster;
    return true;
}

static void fat32_write_cluster_to_entry(uint8_t *entry, uint32_t cluster) {
    if (!entry) return;
    entry[20] = (uint8_t)((cluster >> 16) & 0xFFu);
    entry[21] = (uint8_t)((cluster >> 24) & 0xFFu);
    entry[26] = (uint8_t)( cluster        & 0xFFu);
    entry[27] = (uint8_t)((cluster >>  8) & 0xFFu);
}

static bool _fat32_init() {
    g_fat32_partition_lba = disk_get_partition_lba();
    if (fat32_read_boot_sector_bpb(&bpb) && fat32_validate_bpb()) {
        g_cached_fat_sector = 0xFFFFFFFFu;
        g_cluster_cache.cluster_value = 0;
        g_fsinfo_valid = false;
        g_fsinfo_free_count = 0xFFFFFFFFu;
        (void)fat32_load_fsinfo();
        return true;
    }

    uint64_t fallback_lba = 0;
    if (fat32_detect_partition_start(&fallback_lba) && fat32_try_init_at_lba(fallback_lba)) {
        return true;
    }

    return false;
}

bool fat32_init() {
    uint64_t flags = hal_cpu_save_interrupts();
    spinlock_lock(&g_fat32_lock);
    g_next_free_cluster_hint = 2u;
    bool ret = _fat32_init();
    spinlock_unlock(&g_fat32_lock);
    hal_cpu_restore_interrupts(flags);
    return ret;
}

static bool _fat32_find_file(const char *filename, FAT32_FILE *file) {
    if (!filename || !file) return false;
    if (!fat32_lookup_path(filename, file)) return false;
    if (file->attributes & FAT32_ATTR_DIRECTORY) return false;
    return true;
}

bool fat32_find_file(const char *filename, FAT32_FILE *file) {
    spinlock_lock(&g_fat32_lock);
    bool ret = _fat32_find_file(filename, file);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

static bool _fat32_creat(const char *path)
{
    if (!path || path[0] == '\0') return false;

    char parent[FAT32_PATH_MAX], name[FAT32_PATH_MAX];
    if (!fat32_split_parent_and_name(path, parent, sizeof(parent),
                                      name,   sizeof(name)))  return false;

    uint32_t name_len = fat32_strlen(name);
    if (name_len == 0u || name_len >= FAT32_MAX_NAME_LEN) {
        return false;
    }

    uint32_t parent_cluster = 0u;
    if (!fat32_resolve_directory_cluster(parent, &parent_cluster)) return false;

    FAT32_FILE existing;
    if (fat32_lookup_entry_in_directory(parent_cluster, name, &existing)) {
        if (existing.attributes & FAT32_ATTR_DIRECTORY) return false;
        return _fat32_truncate(&existing, 0u);
    }

    uint8_t short_name83[11];
    if (!fat32_gen_short_name(name, parent_cluster, short_name83)) {
        return false;
    }

    bool    needs_lfn = !fat32_name_is_simple_83(name);
    uint8_t lfn_count = needs_lfn ? fat32_lfn_count_needed(name) : 0u;
    uint32_t total_slots = (uint32_t)lfn_count + 1u;

    uint32_t first_sector = 0u;
    uint16_t first_offset = 0u;
    if (!fat32_find_consecutive_free(parent_cluster, total_slots,
                                      &first_sector, &first_offset)) {
        return false;
    }

    return fat32_write_entry_sequence(first_sector, first_offset,
                                       name, short_name83,
                                       FAT32_ATTR_ARCHIVE, 0u, 0u);
}

bool fat32_creat(const char *path) {
    spinlock_lock(&g_fat32_lock);
    bool ret = _fat32_creat(path);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

static bool _fat32_mkdir(const char *path)
{
    if (!path || path[0] == '\0') return false;

    char parent[FAT32_PATH_MAX], name[FAT32_PATH_MAX];
    if (!fat32_split_parent_and_name(path, parent, sizeof(parent),
                                      name,   sizeof(name)))  return false;

    uint32_t name_len = fat32_strlen(name);
    if (name_len == 0u || name_len >= FAT32_MAX_NAME_LEN) {
        return false;
    }

    uint32_t parent_cluster = 0u;
    if (!fat32_resolve_directory_cluster(parent, &parent_cluster)) return false;

    FAT32_FILE existing;
    if (fat32_lookup_entry_in_directory(parent_cluster, name, &existing)) return false;

    uint8_t short_name83[11];
    if (!fat32_gen_short_name(name, parent_cluster, short_name83)) {
        return false;
    }

    uint32_t new_dir_cluster = fat32_allocate_cluster_zeroed();
    if (new_dir_cluster < 2u) return false;

    uint32_t dot_dot_cluster =
        (parent_cluster == bpb.root_cluster) ? 0u : parent_cluster;

    uint32_t lba = cluster_to_lba(new_dir_cluster);
    if (lba == 0u || !disk_read(lba, g_sector_buffer, 1)) {
        (void)fat32_free_cluster_chain(new_dir_cluster); return false;
    }
    memset(&g_sector_buffer[0],                    0, FAT32_DIR_ENTRY_SIZE * 2u);
    for (uint32_t i = 0u; i < 11u; ++i) {
        g_sector_buffer[i]                    = ' ';
        g_sector_buffer[FAT32_DIR_ENTRY_SIZE + i] = ' ';
    }
    g_sector_buffer[0]  = '.';
    g_sector_buffer[11] = FAT32_ATTR_DIRECTORY;
    fat32_write_cluster_to_entry(&g_sector_buffer[0], new_dir_cluster);

    g_sector_buffer[FAT32_DIR_ENTRY_SIZE + 0] = '.';
    g_sector_buffer[FAT32_DIR_ENTRY_SIZE + 1] = '.';
    g_sector_buffer[FAT32_DIR_ENTRY_SIZE + 11] = FAT32_ATTR_DIRECTORY;
    fat32_write_cluster_to_entry(&g_sector_buffer[FAT32_DIR_ENTRY_SIZE], dot_dot_cluster);

    if (!disk_write(lba, g_sector_buffer, 1)) {
        (void)fat32_free_cluster_chain(new_dir_cluster); return false;
    }

    bool    needs_lfn  = !fat32_name_is_simple_83(name);
    uint8_t lfn_count  = needs_lfn ? fat32_lfn_count_needed(name) : 0u;
    uint32_t total_slots = (uint32_t)lfn_count + 1u;

    uint32_t first_sector = 0u;
    uint16_t first_offset = 0u;
    if (!fat32_find_consecutive_free(parent_cluster, total_slots,
                                      &first_sector, &first_offset)) {
        (void)fat32_free_cluster_chain(new_dir_cluster);
        return false;
    }

    if (!fat32_write_entry_sequence(first_sector, first_offset,
                                     name, short_name83,
                                     FAT32_ATTR_DIRECTORY,
                                     new_dir_cluster, 0u)) {
        (void)fat32_free_cluster_chain(new_dir_cluster); return false;
    }
    return true;
}

bool fat32_mkdir(const char *path) {
    spinlock_lock(&g_fat32_lock);
    bool ret = _fat32_mkdir(path);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

static bool _fat32_unlink(const char *path)
{
    FAT32_FILE file;
    if (!path || !fat32_lookup_path(path, &file)) return false;
    if (file.attributes & FAT32_ATTR_DIRECTORY)   return false;

    if (!fat32_free_cluster_chain(file.first_cluster)) return false;

    if (file.dir_entry_sector == 0u ||
        file.dir_entry_offset + FAT32_DIR_ENTRY_SIZE > bpb.bytes_per_sector) return false;

    if (file.lfn_entry_count > 0u) {
        if (!fat32_delete_preceding_lfn(file.dir_cluster,
                                         file.dir_entry_sector,
                                         file.dir_entry_offset,
                                         file.lfn_entry_count)) {
            return false;
        }
    }

    if (!disk_read(file.dir_entry_sector, g_sector_buffer, 1)) return false;
    uint32_t off = file.dir_entry_offset;
    g_sector_buffer[off]      = 0xE5u;
    g_sector_buffer[off + 20] = 0u;
    g_sector_buffer[off + 21] = 0u;
    g_sector_buffer[off + 26] = 0u;
    g_sector_buffer[off + 27] = 0u;
    g_sector_buffer[off + 28] = 0u;
    g_sector_buffer[off + 29] = 0u;
    g_sector_buffer[off + 30] = 0u;
    g_sector_buffer[off + 31] = 0u;
    return disk_write(file.dir_entry_sector, g_sector_buffer, 1);
}

bool fat32_unlink(const char *path) {
    spinlock_lock(&g_fat32_lock);
    bool ret = _fat32_unlink(path);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

static void _fat32_list_root_files(void) {
    uint32_t cluster = bpb.root_cluster;
    char     lfn_name[FAT32_MAX_NAME_LEN];
    int      lfn_valid = 0;
    memset(lfn_name, 0, sizeof(lfn_name));
    uint32_t max_clusters = fat32_total_clusters();

    for (uint32_t c = 0u; c < max_clusters; ++c) {
        uint32_t lba = cluster_to_lba(cluster);
        if (!lba) return;

        for (uint8_t sec = 0u; sec < bpb.sectors_per_cluster; ++sec) {
            if (!disk_read(lba + sec, g_sector_buffer, 1)) return;

            for (uint32_t i = 0u; i < bpb.bytes_per_sector; i += FAT32_DIR_ENTRY_SIZE) {
                uint8_t first = g_sector_buffer[i];
                if (first == 0x00u) return;
                if (first == 0xE5u) { lfn_valid = 0; memset(lfn_name, 0, sizeof(lfn_name)); continue; }

                uint8_t attr = g_sector_buffer[i + 11];
                if (attr == FAT32_ATTR_LFN) {
                    uint8_t seq   = g_sector_buffer[i];
                    uint8_t order = seq & 0x1Fu;
                    if (order == 0u || order > FAT32_MAX_LFN_ORDER) {
                        lfn_valid = 0; memset(lfn_name, 0, sizeof(lfn_name)); continue;
                    }
                    if (seq & 0x40u) { memset(lfn_name, 0, sizeof(lfn_name)); lfn_valid = 1; }
                    if (!lfn_valid ||
                        !fat32_decode_lfn_entry(&g_sector_buffer[i], lfn_name, sizeof(lfn_name))) {
                        lfn_valid = 0; memset(lfn_name, 0, sizeof(lfn_name));
                    }
                    continue;
                }
                if (attr & FAT32_ATTR_VOLUME_ID) { lfn_valid = 0; memset(lfn_name, 0, sizeof(lfn_name)); continue; }

                char short_name[13];
                fat32_short_name_to_string(&g_sector_buffer[i], short_name);

                lfn_valid = 0;
                memset(lfn_name, 0, sizeof(lfn_name));
            }
        }
        uint32_t next = fat_get_next_cluster(cluster);
        if (next < 2u || next >= 0x0FFFFFF8u) break;
        cluster = next;
    }
}

void fat32_list_root_files(void) {
    spinlock_lock(&g_fat32_lock);
    _fat32_list_root_files();
    spinlock_unlock(&g_fat32_lock);
}

static int32_t _fat32_opendir(const char *path)
{
    uint32_t directory_cluster = 0u;
    if (!fat32_resolve_directory_cluster(path ? path : "/", &directory_cluster)) return -1;
    for (int32_t i = 0; i < FAT32_DIR_HANDLE_MAX; ++i) {
        if (!g_dir_handles[i].used) {
            g_dir_handles[i].used              = 1u;
            g_dir_handles[i].directory_cluster = directory_cluster;
            g_dir_handles[i].current_cluster   = directory_cluster;
            g_dir_handles[i].entry_offset      = 0u;
            return i;
        }
    }
    return -1;
}

int32_t fat32_opendir(const char *path) {
    spinlock_lock(&g_fat32_lock);
    int32_t ret = _fat32_opendir(path);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

static int32_t _fat32_readdir(int32_t dir_handle, FAT32_DIRENT *out_entry)
{
    if (dir_handle < 0 || dir_handle >= FAT32_DIR_HANDLE_MAX || !out_entry) return -1;
    fat32_dir_handle_t *h = &g_dir_handles[dir_handle];
    if (!h->used || h->current_cluster < 2u) return -1;

    uint32_t cluster_size = fat32_cluster_size_bytes();
    if (cluster_size == 0u) return -1;

    char lfn_name[FAT32_MAX_NAME_LEN];
    int  lfn_valid = 0;
    uint8_t lfn_entry_count = 0u;
    uint8_t lfn_expected_order = 0u;
    uint8_t lfn_expected_checksum = 0u;
    memset(lfn_name, 0, sizeof(lfn_name));

    while (1) {
        if (h->entry_offset >= cluster_size) {
            uint32_t next = fat_get_next_cluster(h->current_cluster);
            if (next < 2u || next >= FAT32_EOC_MARKER) return 0;
            h->current_cluster = next;
            h->entry_offset    = 0u;
        }

        uint32_t lba = cluster_to_lba(h->current_cluster);
        if (lba == 0u) return -1;

        uint32_t sec = h->entry_offset / bpb.bytes_per_sector;
        uint32_t off = h->entry_offset % bpb.bytes_per_sector;
        h->entry_offset += FAT32_DIR_ENTRY_SIZE;

        if (!disk_read(lba + sec, g_sector_buffer, 1)) return -1;

        const uint8_t *entry = &g_sector_buffer[off];
        uint8_t first = entry[0];

        if (first == 0x00u) return 0;
        if (first == 0xE5u) {
            lfn_valid = 0;
            lfn_entry_count = 0u;
            lfn_expected_order = 0u;
            lfn_expected_checksum = 0u;
            memset(lfn_name, 0, sizeof(lfn_name));
            continue;
        }

        uint8_t attr = entry[11];
        if (attr == FAT32_ATTR_LFN) {
            uint8_t seq = entry[0];
            uint8_t order = seq & 0x1Fu;
            uint8_t checksum = entry[13];
            if (order == 0u || order > FAT32_MAX_LFN_ORDER) {
                lfn_valid = 0;
                lfn_entry_count = 0u;
                lfn_expected_order = 0u;
                lfn_expected_checksum = 0u;
                memset(lfn_name, 0, sizeof(lfn_name));
                continue;
            }
            if ((seq & 0x40u) != 0u) {
                lfn_valid = 1;
                lfn_entry_count = 0u;
                lfn_expected_order = order;
                lfn_expected_checksum = checksum;
                memset(lfn_name, 0, sizeof(lfn_name));
            } else if (!lfn_valid ||
                       lfn_expected_order == 0u ||
                       (uint8_t)(order + 1u) != lfn_expected_order ||
                       checksum != lfn_expected_checksum) {
                lfn_valid = 0;
                lfn_entry_count = 0u;
                lfn_expected_order = 0u;
                lfn_expected_checksum = 0u;
                memset(lfn_name, 0, sizeof(lfn_name));
                continue;
            } else {
                lfn_expected_order = order;
            }

            if (!lfn_valid || !fat32_decode_lfn_entry(entry, lfn_name, sizeof(lfn_name))) {
                lfn_valid = 0;
                lfn_entry_count = 0u;
                lfn_expected_order = 0u;
                lfn_expected_checksum = 0u;
                memset(lfn_name, 0, sizeof(lfn_name));
            } else if (lfn_entry_count < 0xFFu) {
                ++lfn_entry_count;
            }
            continue;
        }
        if (attr & FAT32_ATTR_VOLUME_ID) {
            lfn_valid = 0;
            lfn_entry_count = 0u;
            lfn_expected_order = 0u;
            lfn_expected_checksum = 0u;
            memset(lfn_name, 0, sizeof(lfn_name));
            continue;
        }

        char short_name[13];
        fat32_short_name_to_string(entry, short_name);
        uint8_t short_checksum = fat32_lfn_checksum(entry);
        const char *candidate =
            (lfn_valid && lfn_name[0] != '\0' &&
             lfn_expected_order == 1u &&
             lfn_entry_count > 0u &&
             short_checksum == lfn_expected_checksum)
                ? lfn_name
                : short_name;

        lfn_valid = 0;
        lfn_entry_count = 0u;
        lfn_expected_order = 0u;
        lfn_expected_checksum = 0u;
        memset(lfn_name, 0, sizeof(lfn_name));

        if ((candidate[0] == '.' && candidate[1] == '\0') ||
            (candidate[0] == '.' && candidate[1] == '.' && candidate[2] == '\0')) continue;

        string_copy_limit(out_entry->name, sizeof(out_entry->name), candidate);
        out_entry->attributes   = attr;
        out_entry->size         = read_u32(&entry[28]);
        out_entry->first_cluster = ((uint32_t)read_u16(&entry[20]) << 16) | read_u16(&entry[26]);
        return 1;
    }
}

int32_t fat32_readdir(int32_t dir_handle, FAT32_DIRENT *out_entry) {
    spinlock_lock(&g_fat32_lock);
    int32_t ret = _fat32_readdir(dir_handle, out_entry);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

static int32_t _fat32_closedir(int32_t dir_handle)
{
    if (dir_handle < 0 || dir_handle >= FAT32_DIR_HANDLE_MAX) return -1;
    memset(&g_dir_handles[dir_handle], 0, sizeof(g_dir_handles[dir_handle]));
    return 0;
}

int32_t fat32_closedir(int32_t dir_handle) {
    spinlock_lock(&g_fat32_lock);
    int32_t ret = _fat32_closedir(dir_handle);
    spinlock_unlock(&g_fat32_lock);
    return ret;
}

void fat32_set_case_sensitive_lookup(bool enabled) { g_case_sensitive_lookup = enabled; }
bool fat32_get_case_sensitive_lookup(void)          { return g_case_sensitive_lookup; }

#ifdef IMPLUS_DRIVER_MODULE

/* ---- fs_module_ops_t shims: opaque void *handle == FAT32_FILE *, and
 * FAT32_DIRENT -> vfs_dirent_t (directory bit from the FAT attribute
 * byte). ---- */

static bool fat32_ops_find_file(const char *path, void *handle,
                                uint64_t *out_id, uint32_t *out_size)
{
    FAT32_FILE *file = (FAT32_FILE *)handle;
    if (!fat32_find_file(path, file)) {
        return false;
    }
    /* Per-open handle pointer, matching the identity the old bridge used. */
    *out_id = (uint64_t)(uintptr_t)file;
    *out_size = file->size;
    return true;
}

static bool fat32_ops_read_file(void *handle, uint8_t *buffer)
{
    return fat32_read_file((FAT32_FILE *)handle, buffer);
}

static bool fat32_ops_write_file(void *handle, const uint8_t *buffer)
{
    return fat32_write_file((FAT32_FILE *)handle, buffer);
}

static bool fat32_ops_read_at(void *handle, uint32_t offset,
                              uint8_t *buffer, uint32_t size)
{
    return fat32_read_at((FAT32_FILE *)handle, offset, buffer, size);
}

static bool fat32_ops_write_at(void *handle, uint32_t offset,
                               const uint8_t *buffer, uint32_t size)
{
    return fat32_write_at((FAT32_FILE *)handle, offset, buffer, size);
}

static bool fat32_ops_truncate(void *handle, uint32_t new_size)
{
    return fat32_truncate((FAT32_FILE *)handle, new_size);
}

static uint32_t fat32_ops_get_file_size(void *handle)
{
    return fat32_get_file_size((FAT32_FILE *)handle);
}

static int32_t fat32_ops_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    FAT32_DIRENT dirent;
    memset(&dirent, 0, sizeof(dirent));
    int32_t result = fat32_readdir(handle, &dirent);
    if (result <= 0) {
        return result;
    }
    fs_dirent_set_name(out_entry, dirent.name);
    out_entry->size = dirent.size;
    out_entry->is_directory = (dirent.attributes & 0x10u) != 0u;
    return result;
}

static const fs_module_ops_t g_fat32_driver = {
    .fs_type            = "fat32",
    .media_kind         = VFS_MEDIA_KIND_DISK,
    .handle_size        = (uint32_t)sizeof(FAT32_FILE),
    .init               = fat32_init,
    .find_file          = fat32_ops_find_file,
    .read_file          = fat32_ops_read_file,
    .write_file         = fat32_ops_write_file,
    .read_at            = fat32_ops_read_at,
    .write_at           = fat32_ops_write_at,
    .truncate           = fat32_ops_truncate,
    .get_file_size      = fat32_ops_get_file_size,
    .creat              = fat32_creat,
    .mkdir              = fat32_mkdir,
    .unlink             = fat32_unlink,
    .opendir            = fat32_opendir,
    .readdir            = fat32_ops_readdir,
    .closedir          = fat32_closedir,
    .list_root          = fat32_list_root_files,
    .set_case_sensitive = fat32_set_case_sensitive_lookup,
    .get_case_sensitive = fat32_get_case_sensitive_lookup,
};

static void fat32_driver_shutdown(void)
{
    g_driver_api = NULL;
    g_case_sensitive_lookup = false;
    g_cached_fat_sector = 0xFFFFFFFFu;
}

static const driver_module_descriptor_t g_fat32_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_FILESYSTEM,
    .load_priority = 100u,
    .deps = { NULL },
    .driver_api = &g_fat32_driver,
    .shutdown = fat32_driver_shutdown,
};

#undef hal_cpu_pause
#undef hal_cpu_save_interrupts
#undef hal_cpu_restore_interrupts
#undef disk_read
#undef disk_write
#undef disk_get_partition_lba
#undef memset
#undef memcpy
#undef serial_write_string
#undef serial_write_uint32

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (!api) return NULL;
    
    if (!api->disk_read || !api->disk_write || !api->disk_get_partition_lba ||
         !api->memset || !api->memcpy)
        return NULL;
    
    g_driver_api = api;
    spinlock_init(&g_fat32_lock);
    
    return &g_fat32_module;
}
#endif
