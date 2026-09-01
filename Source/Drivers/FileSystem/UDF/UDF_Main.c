#include "Drivers/FileSystem/UDF/UDF_Main.h"
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
#define disk_get_partition_lba      g_driver_api->hw.disk_get_partition_lba
#define serial_write_string         g_driver_api->dbg.write_string
#define serial_write_uint32         g_driver_api->dbg.write_uint32
#define udf_heap_alloc(sz)          g_driver_api->malloc((uint64_t)(sz))
#define udf_heap_free(p)            g_driver_api->free((p))

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

#else
#include "Core/sync/Spinlock.h"
#include <stdlib.h>
#define udf_heap_alloc(sz)          malloc((size_t)(sz))
#define udf_heap_free(p)            free((p))
#endif

/* ---- ECMA-167 descriptor tag identifiers ---- */
#define UDF_TAG_PVD          1u
#define UDF_TAG_AVDP         2u
#define UDF_TAG_IUVD         4u
#define UDF_TAG_PD           5u
#define UDF_TAG_LVD          6u
#define UDF_TAG_TD           8u
#define UDF_TAG_LVID         9u
#define UDF_TAG_FSD        256u
#define UDF_TAG_FID        257u
#define UDF_TAG_AED        258u
#define UDF_TAG_IE         259u
#define UDF_TAG_FE         261u
#define UDF_TAG_EFE        266u

/* ICBTag.FileType */
#define UDF_FT_DIRECTORY    4u
#define UDF_FT_REGULAR      5u

/* FID FileCharacteristics bits */
#define UDF_FC_HIDDEN      0x01u
#define UDF_FC_DIRECTORY   0x02u
#define UDF_FC_DELETED     0x04u
#define UDF_FC_PARENT      0x08u

/* Extent type held in the top 2 bits of an allocation-descriptor length. */
#define UDF_EXT_RECORDED       0u
#define UDF_EXT_ALLOC_NOTREC   1u
#define UDF_EXT_NOT_ALLOC      2u
#define UDF_EXT_CONTINUATION   3u

#define UDF_HOLE_LBA          0xFFFFFFFFu
#define UDF_DIR_MAX_BYTES     (8u * 1024u * 1024u)
#define UDF_ICB_MAX_DEPTH     8u

typedef struct {
    bool     valid;
    uint16_t ref_partnum[UDF_MAX_PARTITIONS]; /* partition number per map/ref index  */
    uint32_t ref_start[UDF_MAX_PARTITIONS];   /* resolved volume-relative LBA per ref */
    uint32_t ref_count;
    uint16_t pd_num[UDF_MAX_PARTITIONS];
    uint32_t pd_start[UDF_MAX_PARTITIONS];
    uint32_t pd_count;
    uint32_t block_size;
    uint32_t root_fe_lba;
    uint16_t root_ref;
} udf_context_t;

static udf_context_t g_udf;

static uint8_t g_udf_sector_buffer[UDF_SECTOR_SIZE] __attribute__((aligned(4096)));
static uint8_t g_udf_read_buffer  [UDF_SECTOR_SIZE] __attribute__((aligned(4096)));
static uint8_t g_udf_meta_buffer  [UDF_SECTOR_SIZE] __attribute__((aligned(4096)));

static spinlock_t g_udf_lock;
static uint64_t g_udf_partition_lba = 0; /* 512-byte units, from disk_get_partition_lba() */

typedef struct {
    uint8_t  used;
    uint8_t *data;
    uint32_t size;
    uint32_t offset;
} udf_dir_handle_t;

static udf_dir_handle_t g_udf_dirs[UDF_DIR_HANDLE_MAX];

/* ---------------------------------------------------------------- helpers */

static inline uint16_t rd16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline uint64_t rd64(const uint8_t *p) {
    return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32);
}

static bool udf_read_sector(uint32_t lba, uint8_t *buffer) {
    uint64_t base = g_udf_partition_lba + (uint64_t)lba * (UDF_SECTOR_SIZE / 512u);
    return disk_read(base, buffer, UDF_SECTOR_SIZE / 512u);
}

static bool udf_tag_is(const uint8_t *sector, uint16_t want) {
    if (rd16(sector) != want) return false;
    /* ECMA-167 3/7.2 tag checksum: sum modulo 256 of bytes 0..3 and 5..15. */
    uint8_t sum = 0;
    for (int i = 0; i < 16; i++) {
        if (i == 4) continue;
        sum = (uint8_t)(sum + sector[i]);
    }
    return sum == sector[4];
}

static uint32_t udf_part_base(uint16_t ref) {
    if (ref < g_udf.ref_count) return g_udf.ref_start[ref];
    return g_udf.ref_count ? g_udf.ref_start[0] : 0u;
}

static bool udf_name_eq_ci(const char *a, const char *b) {
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca = (char)(ca + 32);
        if (cb >= 'A' && cb <= 'Z') cb = (char)(cb + 32);
        if (ca != cb) return false;
        ++a; ++b;
    }
    return *a == '\0' && *b == '\0';
}

/* Decode a UDF d-characters run (already past the compression-id byte). */
static void udf_decode_dchars(uint8_t comp, const uint8_t *src, uint32_t nbytes,
                              char *dst, uint32_t dst_max) {
    uint32_t di = 0;
    if (comp == 16u) {
        for (uint32_t i = 0; i + 1 < nbytes && di + 4 < dst_max; i += 2) {
            uint16_t cp = (uint16_t)((uint16_t)src[i] << 8 | src[i + 1]);
            if (cp == 0) break;
            if (cp < 0x80u) {
                dst[di++] = (char)cp;
            } else if (cp < 0x800u) {
                dst[di++] = (char)(0xC0u | (cp >> 6));
                dst[di++] = (char)(0x80u | (cp & 0x3Fu));
            } else {
                dst[di++] = (char)(0xE0u | (cp >> 12));
                dst[di++] = (char)(0x80u | ((cp >> 6) & 0x3Fu));
                dst[di++] = (char)(0x80u | (cp & 0x3Fu));
            }
        }
    } else {
        /* compression id 8 (or anything else) -> treat as Latin-1 */
        for (uint32_t i = 0; i < nbytes && di + 3 < dst_max; ++i) {
            uint8_t c = src[i];
            if (c == 0) break;
            if (c < 0x80u) {
                dst[di++] = (char)c;
            } else {
                dst[di++] = (char)(0xC0u | (c >> 6));
                dst[di++] = (char)(0x80u | (c & 0x3Fu));
            }
        }
    }
    dst[di] = '\0';
}

/* -------------------------------------------------- volume structure parse */

static bool udf_parse_lvd(const uint8_t *s, uint32_t *fsd_lbn, uint16_t *fsd_ref) {
    g_udf.block_size = rd32(&s[212]);
    uint32_t num_maps = rd32(&s[268]);
    if (num_maps > UDF_MAX_PARTITIONS) num_maps = UDF_MAX_PARTITIONS;

    /* File Set Descriptor pointer lives in LogicalVolumeContentsUse (a long_ad). */
    *fsd_lbn = rd32(&s[248 + 4]);
    *fsd_ref = rd16(&s[248 + 8]);

    const uint8_t *map = &s[440];
    const uint8_t *end = &s[UDF_SECTOR_SIZE];
    g_udf.ref_count = 0;
    for (uint32_t m = 0; m < num_maps && map + 2 <= end; ++m) {
        uint8_t type = map[0];
        uint8_t mlen = map[1];
        if (mlen < 2u || map + mlen > end) break;
        if (type == 1u && mlen >= 6u) {
            g_udf.ref_partnum[g_udf.ref_count++] = rd16(&map[4]);
        } else {
            /* Type 2 (virtual / sparable / metadata) is unsupported; record a
             * placeholder so the ref index still lines up, but it will not
             * resolve to a partition below. */
            g_udf.ref_partnum[g_udf.ref_count++] = 0xFFFFu;
        }
        map += mlen;
    }
    return g_udf.ref_count > 0u;
}

static void udf_parse_pd(const uint8_t *s) {
    if (g_udf.pd_count >= UDF_MAX_PARTITIONS) return;
    g_udf.pd_num[g_udf.pd_count]   = rd16(&s[22]);
    g_udf.pd_start[g_udf.pd_count] = rd32(&s[188]);
    g_udf.pd_count++;
}

static bool udf_scan_vds(uint32_t loc, uint32_t len_bytes) {
    uint32_t count = len_bytes / UDF_SECTOR_SIZE;
    if (count == 0u || count > 64u) count = 64u;

    bool have_lvd = false;
    uint32_t fsd_lbn = 0;
    uint16_t fsd_ref = 0;

    for (uint32_t i = 0; i < count; ++i) {
        if (!udf_read_sector(loc + i, g_udf_sector_buffer)) break;
        uint16_t tag = rd16(g_udf_sector_buffer);
        if (tag == 0u) continue;
        if (tag == UDF_TAG_TD) break;
        if (!udf_tag_is(g_udf_sector_buffer, tag)) continue;

        if (tag == UDF_TAG_LVD && !have_lvd) {
            have_lvd = udf_parse_lvd(g_udf_sector_buffer, &fsd_lbn, &fsd_ref);
        } else if (tag == UDF_TAG_PD) {
            udf_parse_pd(g_udf_sector_buffer);
        }
    }

    if (!have_lvd || g_udf.pd_count == 0u) return false;

    /* Resolve each partition reference index to a volume-relative LBA. */
    for (uint32_t r = 0; r < g_udf.ref_count; ++r) {
        g_udf.ref_start[r] = 0u;
        for (uint32_t p = 0; p < g_udf.pd_count; ++p) {
            if (g_udf.pd_num[p] == g_udf.ref_partnum[r]) {
                g_udf.ref_start[r] = g_udf.pd_start[p];
                break;
            }
        }
    }
    if (fsd_ref >= g_udf.ref_count && g_udf.ref_count > 0u) fsd_ref = 0u;

    /* File Set Descriptor -> Root Directory ICB. */
    uint32_t fsd_lba = udf_part_base(fsd_ref) + fsd_lbn;
    if (!udf_read_sector(fsd_lba, g_udf_sector_buffer)) return false;
    if (!udf_tag_is(g_udf_sector_buffer, UDF_TAG_FSD)) return false;

    g_udf.root_ref    = rd16(&g_udf_sector_buffer[400 + 8]);
    if (g_udf.root_ref >= g_udf.ref_count) g_udf.root_ref = 0u;
    g_udf.root_fe_lba = udf_part_base(g_udf.root_ref) + rd32(&g_udf_sector_buffer[400 + 4]);
    return true;
}

static bool udf_find_nsr(void) {
    for (uint32_t lba = 16u; lba < 64u; ++lba) {
        if (!udf_read_sector(lba, g_udf_sector_buffer)) return false;
        const uint8_t *id = &g_udf_sector_buffer[1];
        if (memcmp(id, "NSR0", 4) == 0) return true;
        if (memcmp(id, "BEA01", 5) == 0) continue;
        if (memcmp(id, "TEA01", 5) == 0) return false;
        if (memcmp(id, "CD001", 5) == 0) continue; /* ISO9660 primary VD terminator area */
        if (g_udf_sector_buffer[0] == 0u && id[0] == 0u) return false;
    }
    return false;
}

static bool udf_parse_anchor(void) {
    static const uint32_t kAnchorLbas[] = { 256u, 512u };
    for (uint32_t i = 0; i < sizeof(kAnchorLbas) / sizeof(kAnchorLbas[0]); ++i) {
        if (!udf_read_sector(kAnchorLbas[i], g_udf_sector_buffer)) continue;
        if (!udf_tag_is(g_udf_sector_buffer, UDF_TAG_AVDP)) continue;
        uint32_t main_len = rd32(&g_udf_sector_buffer[16]);
        uint32_t main_loc = rd32(&g_udf_sector_buffer[20]);
        if (main_loc == 0u) continue;
        if (udf_scan_vds(main_loc, main_len)) return true;
    }
    return false;
}

static bool _udf_init(void) {
    memset(&g_udf, 0, sizeof(g_udf));
    memset(g_udf_dirs, 0, sizeof(g_udf_dirs));
    g_udf_partition_lba = disk_get_partition_lba();

    if (udf_find_nsr() && udf_parse_anchor()) {
        g_udf.valid = true;
        serial_write_string("[UDF] mounted, root FE LBA ");
        serial_write_uint32(g_udf.root_fe_lba);
        serial_write_string("\n");
        return true;
    }

    if (g_udf_partition_lba != 0) {
        g_udf_partition_lba = 0;
        memset(&g_udf, 0, sizeof(g_udf));
        if (udf_find_nsr() && udf_parse_anchor()) {
            g_udf.valid = true;
            return true;
        }
    }

    serial_write_string("[UDF] no UDF volume found\n");
    return false;
}

bool udf_init(void) {
    spinlock_lock(&g_udf_lock);
    bool ret = _udf_init();
    spinlock_unlock(&g_udf_lock);
    return ret;
}

/* --------------------------------------------------------- File Entry parse */

static void udf_add_extent(UDF_FILE *out, uint32_t lba, uint32_t len) {
    if (len == 0u) return;
    if (out->extent_count >= UDF_MAX_EXTENTS) return;
    out->extents[out->extent_count].lba = lba;
    out->extents[out->extent_count].len = len;
    out->extent_count++;
}

/* Parse a run of allocation descriptors into out->extents. `ad_kind` is the
 * ICBTag flags AD-type (0 short_ad, 1 long_ad). `home_ref` is the partition
 * the owning File Entry lives in (short_ad has no partition field). */
static void udf_parse_ads(UDF_FILE *out, const uint8_t *ad, uint32_t ad_len,
                          uint8_t ad_kind, uint16_t home_ref, uint32_t depth) {
    uint32_t step = (ad_kind == 1u) ? 16u : 8u;
    for (uint32_t off = 0; off + step <= ad_len; off += step) {
        const uint8_t *p = ad + off;
        uint32_t raw   = rd32(p);
        uint32_t etype = raw >> 30;
        uint32_t elen  = raw & 0x3FFFFFFFu;
        uint32_t lbn;
        uint16_t ref;
        if (ad_kind == 1u) {
            lbn = rd32(p + 4);
            ref = rd16(p + 8);
        } else {
            lbn = rd32(p + 4);
            ref = home_ref;
        }

        if (etype == UDF_EXT_CONTINUATION) {
            if (depth >= UDF_ICB_MAX_DEPTH) return;
            uint32_t aed_lba = udf_part_base(ref) + lbn;
            if (!udf_read_sector(aed_lba, g_udf_meta_buffer)) return;
            if (!udf_tag_is(g_udf_meta_buffer, UDF_TAG_AED)) return;
            uint32_t aed_ad_len = rd32(&g_udf_meta_buffer[20]);
            if (24u + aed_ad_len > UDF_SECTOR_SIZE) aed_ad_len = UDF_SECTOR_SIZE - 24u;
            /* g_udf_meta_buffer is reused by the recursion; copy the AD run out. */
            static uint8_t s_aed_scratch[UDF_SECTOR_SIZE];
            memcpy(s_aed_scratch, &g_udf_meta_buffer[24], aed_ad_len);
            udf_parse_ads(out, s_aed_scratch, aed_ad_len, ad_kind, home_ref, depth + 1u);
            return;
        }
        if (elen == 0u) continue;
        if (etype == UDF_EXT_RECORDED) {
            udf_add_extent(out, udf_part_base(ref) + lbn, elen);
        } else {
            /* allocated-not-recorded / not-allocated -> sparse hole */
            udf_add_extent(out, UDF_HOLE_LBA, elen);
        }
    }
}

static bool udf_parse_file_entry(uint32_t fe_lba, uint16_t home_ref,
                                 UDF_FILE *out, uint32_t depth) {
    if (depth >= UDF_ICB_MAX_DEPTH) return false;
    if (!udf_read_sector(fe_lba, g_udf_meta_buffer)) return false;

    uint16_t tag = rd16(g_udf_meta_buffer);
    if (tag == UDF_TAG_IE) {
        uint32_t lbn = rd32(&g_udf_meta_buffer[16 + 4]);
        uint16_t ref = rd16(&g_udf_meta_buffer[16 + 8]);
        return udf_parse_file_entry(udf_part_base(ref) + lbn, ref, out, depth + 1u);
    }
    if (tag != UDF_TAG_FE && tag != UDF_TAG_EFE) return false;
    if (!udf_tag_is(g_udf_meta_buffer, tag)) return false;

    bool efe = (tag == UDF_TAG_EFE);
    const uint8_t *b = g_udf_meta_buffer;

    uint8_t  file_type = b[16 + 11];
    uint8_t  ad_kind   = (uint8_t)(rd16(&b[16 + 18]) & 0x7u);
    uint64_t info_len  = rd64(&b[56]);

    uint32_t l_ea, l_ad, ad_base;
    if (efe) {
        l_ea    = rd32(&b[208]);
        l_ad    = rd32(&b[212]);
        ad_base = 216u;
    } else {
        l_ea    = rd32(&b[168]);
        l_ad    = rd32(&b[172]);
        ad_base = 176u;
    }
    if ((uint64_t)ad_base + l_ea + l_ad > UDF_SECTOR_SIZE) return false;

    memset(out, 0, sizeof(*out));
    out->size   = info_len;
    out->fe_lba = fe_lba;
    out->is_dir = (file_type == UDF_FT_DIRECTORY) ? 1u : 0u;

    const uint8_t *ad = b + ad_base + l_ea;
    if (ad_kind == 3u) {
        /* File data stored directly in the allocation-descriptor area. */
        uint32_t n = l_ad;
        if (n > info_len)       n = (uint32_t)info_len;
        if (n > UDF_EMBED_MAX)  n = UDF_EMBED_MAX;
        memcpy(out->embed, ad, n);
        out->embed_len = n;
        out->embedded  = 1u;
        return true;
    }
    if (ad_kind == 0u || ad_kind == 1u) {
        udf_parse_ads(out, ad, l_ad, ad_kind, home_ref, 0u);
        return true;
    }
    return false; /* extended_ad unsupported */
}

/* ------------------------------------------------------------- directory IO */

static bool _udf_read_at(UDF_FILE *file, uint32_t offset, uint8_t *buf, uint32_t size);

/* Slurp an entire directory's contents into a freshly allocated buffer. */
static uint8_t *udf_load_dir(UDF_FILE *dir, uint32_t *out_len) {
    if (!dir->is_dir) return NULL;
    uint32_t len = (dir->size > UDF_DIR_MAX_BYTES) ? UDF_DIR_MAX_BYTES
                                                   : (uint32_t)dir->size;
    if (len == 0u) { *out_len = 0u; return (uint8_t *)udf_heap_alloc(1); }

    uint8_t *data = (uint8_t *)udf_heap_alloc(len);
    if (!data) return NULL;
    if (!_udf_read_at(dir, 0u, data, len)) {
        udf_heap_free(data);
        return NULL;
    }
    *out_len = len;
    return data;
}

/* Walk FIDs in `data` starting at *cursor. Fills name/child on a real entry,
 * returns 1; returns 0 at end of directory, -1 on malformed data. */
static int udf_next_fid(const uint8_t *data, uint32_t len, uint32_t *cursor,
                        char *name, uint32_t name_max,
                        uint32_t *child_fe_lba, uint8_t *is_dir) {
    while (*cursor + 38u <= len) {
        const uint8_t *fid = data + *cursor;
        if (rd16(fid) != UDF_TAG_FID) return 0;

        uint16_t l_iu  = rd16(&fid[36]);
        uint8_t  l_fi  = fid[19];
        uint8_t  chars = fid[18];
        uint32_t lbn   = rd32(&fid[20 + 4]);
        uint16_t ref   = rd16(&fid[20 + 8]);

        uint32_t fid_len = 38u + l_iu + l_fi;
        uint32_t padded  = (fid_len + 3u) & ~3u;
        if (padded == 0u || *cursor + fid_len > len) return -1;

        const uint8_t *nm = fid + 38u + l_iu;
        bool skip = (chars & (UDF_FC_PARENT | UDF_FC_DELETED)) != 0u || l_fi == 0u;

        *cursor += padded;

        if (!skip) {
            udf_decode_dchars(nm[0], nm + 1, (uint32_t)(l_fi - 1u), name, name_max);
            if (name[0] != '\0') {
                *child_fe_lba = udf_part_base(ref) + lbn;
                *is_dir = (chars & UDF_FC_DIRECTORY) ? 1u : 0u;
                return 1;
            }
        }
    }
    return 0;
}

/* ------------------------------------------------------------- path lookup */

static bool _udf_lookup_path(const char *path, UDF_FILE *out) {
    if (!g_udf.valid || !path || !out || path[0] != '/') return false;

    UDF_FILE cur;
    if (!udf_parse_file_entry(g_udf.root_fe_lba, g_udf.root_ref, &cur, 0u)) return false;

    const char *cursor = path + 1;
    if (*cursor == '\0') { memcpy(out, &cur, sizeof(*out)); return true; }

    while (*cursor) {
        const char *end = cursor;
        while (*end && *end != '/') ++end;
        uint32_t clen = (uint32_t)(end - cursor);
        if (clen == 0u || clen >= UDF_NAME_MAX) return false;

        char want[UDF_NAME_MAX];
        memcpy(want, cursor, clen);
        want[clen] = '\0';

        if (!cur.is_dir) return false;

        uint32_t dlen = 0;
        uint8_t *data = udf_load_dir(&cur, &dlen);
        if (!data) return false;

        uint32_t walk = 0;
        char nm[UDF_NAME_MAX];
        uint32_t child_lba = 0;
        uint8_t child_dir = 0;
        bool found = false;
        int r;
        while ((r = udf_next_fid(data, dlen, &walk, nm, sizeof(nm),
                                 &child_lba, &child_dir)) == 1) {
            if (udf_name_eq_ci(nm, want)) { found = true; break; }
        }
        udf_heap_free(data);
        if (!found) return false;

        UDF_FILE child;
        if (!udf_parse_file_entry(child_lba, g_udf.root_ref, &child, 0u)) return false;

        while (*end == '/') ++end;
        if (*end == '\0') { memcpy(out, &child, sizeof(*out)); return true; }
        cur = child;
        cursor = end;
    }
    return false;
}

static bool _udf_find_file(const char *path, UDF_FILE *file) {
    if (!_udf_lookup_path(path, file)) return false;
    return !file->is_dir;
}

bool udf_find_file(const char *path, UDF_FILE *file) {
    spinlock_lock(&g_udf_lock);
    bool ret = _udf_find_file(path, file);
    spinlock_unlock(&g_udf_lock);
    return ret;
}

/* --------------------------------------------------------------- file read */

static bool _udf_read_at(UDF_FILE *file, uint32_t offset, uint8_t *buf, uint32_t size) {
    if (!file || !buf) return false;
    if (offset >= file->size) return size == 0u;
    if (size > file->size - offset) size = (uint32_t)(file->size - offset);
    if (size == 0u) return true;

    if (file->embedded) {
        uint32_t n = (offset < file->embed_len) ? (file->embed_len - offset) : 0u;
        if (n > size) n = size;
        memcpy(buf, file->embed + offset, n);
        if (n < size) memset(buf + n, 0, size - n);
        return true;
    }

    uint32_t done = 0;
    uint64_t file_pos = 0;
    for (uint32_t e = 0; e < file->extent_count && done < size; ++e) {
        uint32_t ext_len = file->extents[e].len;
        uint64_t ext_end = file_pos + ext_len;

        if ((uint64_t)(offset + done) < ext_end) {
            uint32_t within = (uint32_t)((uint64_t)(offset + done) - file_pos);
            uint32_t avail  = ext_len - within;
            uint32_t want   = size - done;
            uint32_t chunk  = (want < avail) ? want : avail;

            if (file->extents[e].lba == UDF_HOLE_LBA) {
                memset(buf + done, 0, chunk);
                done += chunk;
            } else {
                uint32_t sec = file->extents[e].lba + within / UDF_SECTOR_SIZE;
                uint32_t soff = within % UDF_SECTOR_SIZE;
                uint32_t left = chunk;
                while (left > 0u) {
                    if (soff == 0u && left >= UDF_SECTOR_SIZE) {
                        uint32_t nsec = left / UDF_SECTOR_SIZE;
                        uint64_t base = g_udf_partition_lba +
                                        (uint64_t)sec * (UDF_SECTOR_SIZE / 512u);
                        if (!disk_read(base, buf + done, nsec * (UDF_SECTOR_SIZE / 512u)))
                            return false;
                        uint32_t got = nsec * UDF_SECTOR_SIZE;
                        done += got; left -= got; sec += nsec;
                        continue;
                    }
                    if (!udf_read_sector(sec, g_udf_read_buffer)) return false;
                    uint32_t can = UDF_SECTOR_SIZE - soff;
                    uint32_t cp  = (left < can) ? left : can;
                    memcpy(buf + done, g_udf_read_buffer + soff, cp);
                    done += cp; left -= cp; soff = 0u; sec++;
                }
            }
        }
        file_pos = ext_end;
    }

    if (done < size) {
        memset(buf + done, 0, size - done);
    }
    return true;
}

bool udf_read_at(UDF_FILE *file, uint32_t offset, uint8_t *buf, uint32_t size) {
    spinlock_lock(&g_udf_lock);
    bool ret = _udf_read_at(file, offset, buf, size);
    spinlock_unlock(&g_udf_lock);
    return ret;
}

bool udf_read_file(UDF_FILE *file, uint8_t *buf) {
    if (!file || !buf) return false;
    return udf_read_at(file, 0u, buf, (uint32_t)file->size);
}

uint32_t udf_get_file_size(UDF_FILE *file) {
    return file ? (uint32_t)file->size : 0u;
}

/* ---------------------------------------------------------- directory API */

static int32_t _udf_opendir(const char *path) {
    const char *dir_path = (path && *path) ? path : "/";
    UDF_FILE dir;
    if (!_udf_lookup_path(dir_path, &dir) || !dir.is_dir) return -1;

    for (int32_t i = 0; i < (int32_t)UDF_DIR_HANDLE_MAX; ++i) {
        if (g_udf_dirs[i].used) continue;
        uint32_t len = 0;
        uint8_t *data = udf_load_dir(&dir, &len);
        if (!data) return -1;
        g_udf_dirs[i].used   = 1u;
        g_udf_dirs[i].data   = data;
        g_udf_dirs[i].size   = len;
        g_udf_dirs[i].offset = 0u;
        return i;
    }
    return -1;
}

int32_t udf_opendir(const char *path) {
    spinlock_lock(&g_udf_lock);
    int32_t ret = _udf_opendir(path);
    spinlock_unlock(&g_udf_lock);
    return ret;
}

static int32_t _udf_readdir(int32_t handle, UDF_DIRENT *out) {
    if (handle < 0 || handle >= (int32_t)UDF_DIR_HANDLE_MAX || !out) return -1;
    udf_dir_handle_t *h = &g_udf_dirs[handle];
    if (!h->used) return -1;

    char nm[UDF_NAME_MAX];
    uint32_t child_lba = 0;
    uint8_t child_dir = 0;
    int r = udf_next_fid(h->data, h->size, &h->offset, nm, sizeof(nm),
                         &child_lba, &child_dir);
    if (r != 1) return r == 0 ? 0 : -1;

    memcpy(out->name, nm, UDF_NAME_MAX);
    out->is_directory = child_dir;
    out->size = 0u;

    UDF_FILE child;
    if (udf_parse_file_entry(child_lba, g_udf.root_ref, &child, 0u)) {
        out->size = child.size;
        out->is_directory = child.is_dir;
    }
    return 1;
}

int32_t udf_readdir(int32_t handle, UDF_DIRENT *out) {
    spinlock_lock(&g_udf_lock);
    int32_t ret = _udf_readdir(handle, out);
    spinlock_unlock(&g_udf_lock);
    return ret;
}

static int32_t _udf_closedir(int32_t handle) {
    if (handle < 0 || handle >= (int32_t)UDF_DIR_HANDLE_MAX) return -1;
    udf_dir_handle_t *h = &g_udf_dirs[handle];
    if (h->used && h->data) udf_heap_free(h->data);
    memset(h, 0, sizeof(*h));
    return 0;
}

int32_t udf_closedir(int32_t handle) {
    spinlock_lock(&g_udf_lock);
    int32_t ret = _udf_closedir(handle);
    spinlock_unlock(&g_udf_lock);
    return ret;
}

void udf_list_root(void) {
    (void)0;
}

/* ------------------------------------------------------ fs_module_ops shim */
#ifdef IMPLUS_DRIVER_MODULE

static bool udf_ops_find_file(const char *path, void *handle,
                              uint64_t *out_id, uint32_t *out_size) {
    UDF_FILE *file = (UDF_FILE *)handle;
    if (!udf_find_file(path, file)) return false;
    *out_id   = (uint64_t)file->fe_lba;
    *out_size = (uint32_t)file->size;
    return true;
}

static bool udf_ops_read_file(void *handle, uint8_t *buffer) {
    return udf_read_file((UDF_FILE *)handle, buffer);
}

static bool udf_ops_read_at(void *handle, uint32_t offset,
                            uint8_t *buffer, uint32_t size) {
    return udf_read_at((UDF_FILE *)handle, offset, buffer, size);
}

static uint32_t udf_ops_get_file_size(void *handle) {
    return udf_get_file_size((UDF_FILE *)handle);
}

static int32_t udf_ops_readdir(int32_t handle, vfs_dirent_t *out_entry) {
    UDF_DIRENT dirent;
    memset(&dirent, 0, sizeof(dirent));
    int32_t result = udf_readdir(handle, &dirent);
    if (result <= 0) return result;
    fs_dirent_set_name(out_entry, dirent.name);
    out_entry->size = (uint32_t)dirent.size;
    out_entry->is_directory = dirent.is_directory != 0;
    return result;
}

static const fs_module_ops_t g_udf_driver = {
    .fs_type       = "udf",
    .media_kind    = VFS_MEDIA_KIND_OPTICAL,
    .handle_size   = (uint32_t)sizeof(UDF_FILE),
    .init          = udf_init,
    .find_file     = udf_ops_find_file,
    .read_file     = udf_ops_read_file,
    .read_at       = udf_ops_read_at,
    .get_file_size = udf_ops_get_file_size,
    .opendir       = udf_opendir,
    .readdir       = udf_ops_readdir,
    .closedir      = udf_closedir,
    .list_root     = udf_list_root,
};

static void udf_driver_shutdown(void) {
    for (uint32_t i = 0; i < UDF_DIR_HANDLE_MAX; ++i) {
        if (g_udf_dirs[i].used && g_udf_dirs[i].data) udf_heap_free(g_udf_dirs[i].data);
    }
    memset(g_udf_dirs, 0, sizeof(g_udf_dirs));
    memset(&g_udf, 0, sizeof(g_udf));
    g_udf_partition_lba = 0;
    g_driver_api = NULL;
}

static const driver_module_descriptor_t g_udf_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_FILESYSTEM,
    /* Lower than ISO9660 (100) so, on a UDF-bridge disc, UDF is discovered
     * and mounted first and wins the "first optical filesystem" tie-break in
     * FS_VFS_Bridge.c. If udf_init() fails the bridge marks this slot
     * init-failed and ISO9660 becomes the default automatically. */
    .load_priority = 99u,
    .deps = { NULL },
    .driver_api = &g_udf_driver,
    .shutdown   = udf_driver_shutdown,
};

#undef hal_cpu_pause
#undef disk_read
#undef disk_get_partition_lba
#undef serial_write_string
#undef serial_write_uint32
#undef memset
#undef memcpy

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api) {
    if (!api || !api->disk_read || !api->disk_get_partition_lba ||
        !api->malloc || !api->free || !api->memset || !api->memcpy)
        return NULL;
    g_driver_api = api;
    g_udf_partition_lba = api->disk_get_partition_lba();
    spinlock_init(&g_udf_lock);
    return &g_udf_module;
}
#endif
