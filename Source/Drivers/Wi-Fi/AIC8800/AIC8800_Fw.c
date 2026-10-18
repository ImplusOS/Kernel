#include "AIC8800.h"

/*
 * AIC8800_Fw.c -- the loader stage: get the vendor firmware running.
 *
 * A cold AIC8800D80 enumerates as a69c:8d80 and runs nothing but a ROM
 * monitor that speaks the TASK_DBG message set (raw memory reads/writes).
 * Everything the chip will ever be able to do has to be pushed into its
 * RAM from here, in the vendor's own order:
 *
 *   system_config_8800d80()      read the chip revision out of 0x40500000,
 *                                program the instruction-cache window, run
 *                                the (empty on D80) syscfg tables.
 *   fw_patch_table_*.bin         read and parse it: it is a container of
 *                                (name, type, pairs-of-words) blocks, and
 *                                its type-0 "INF" block is what says where
 *                                the adid/patch images go and how many
 *                                extra patch images there are.
 *   fw_adid_*.bin                -> patch_info.addr_adid
 *   fw_patch_*.bin               -> patch_info.addr_patch
 *   fw_patch_*_ext<N>.bin        -> one per ext entry in INF
 *   (the patch table itself)     every block of it is a list of
 *                                (address, value) pairs to write
 *   fmacfw_*.bin                 -> 0x00120000, the actual firmware
 *   aicwf_patch_config_8800d80() the firmware's patch descriptor, written
 *                                into RAM the image just loaded
 *   DBG_START_APP_REQ            run it.
 *
 * After that the chip resets its USB side and commonly comes back as
 * a69c:8d83 (some revisions use 8d81),
 * where AIC8800_Bring.c takes over.
 *
 * Two things keep this cheap on the syscall path: the whole ladder is
 * driven as a phase machine (one step per aic_tick(), so no single call
 * ever runs to completion), and the big images are streamed 1 KiB at a
 * time against the vendor's DBG_MEM_BLOCK_WRITE_REQ instead of being read
 * into memory first.
 *
 * The firmware blobs are proprietary and deliberately not in the
 * repository: the driver reads them from AIC_FW_DIR on the OS filesystem.
 * When they are missing, every step simply fails and the ladder retries a
 * few times before giving up, so a machine without them boots normally
 * with the adapter reporting DRIVER_WIFI_STATE_FIRMWARE_FAILED.
 */

/* ---- loader-stage constants (aic_compat_8800d80.c / aicbluetooth.c) ---- */

#define AIC_SYSCFG_CHIPID_ADDR      0x40500000u /* chip id / mcu id */
#define AIC_SYSCFG_CACHE_ADDR       0x40100020u /* instruction cache window */
#define AIC_RAM_FMAC_FW_ADDR        0x00120000u

#define AIC_CHIP_REV_U01            0x01u
#define AIC_CHIP_REV_U02            0x03u
#define AIC_CHIP_REV_U03            0x07u
#define AIC_CHIP_ID_H_MASK          0xC0u
#define AIC_CHIP_ID_H(chip_id)      (((chip_id) & AIC_CHIP_ID_H_MASK) == AIC_CHIP_ID_H_MASK)

#define AIC_PATCH_CONFIG_OFS        0x0198u /* fmacfw image: patch base */
#define AIC_PATCH_STR_OFS           0x01A0u /* fmacfw image: patch descriptor */
#define AIC_PATCH_BUFF_OFS          0x01A4u /* fmacfw image: alternate buffer */
#define AIC_PATCH_VERSION_OFS       0x001Cu
#define AIC_PATCH_VERSION_NEW       0x06090100u
#define AIC_PATCH_MAGIC_NUM         0x48435450u /* "PTCH" */
#define AIC_PATCH_MAGIC_NUM_2       0x50544348u /* "HCTP" */
#define AIC_PATCH_START_ADDR        0x001D7000u
#define AIC_PATCH_PAIR_COUNT        3u
#define AIC_PATCH_BLOCK_MAX         4u

/* Patch table block types. An entry whose type is >= AICBT_PT_SKIP_TYPE
 * (or whose length is zero) carries no data at all -- the vendor
 * "workaround" for entries that only mark a resource. */
#define AICBT_PT_INF                0u
#define AICBT_PT_TRAP               1u
#define AICBT_PT_B4                 2u
#define AICBT_PT_BTMODE             3u
#define AICBT_PT_PWRON              4u
#define AICBT_PT_AF                 5u
#define AICBT_PT_VER                6u
#define AICBT_PT_SKIP_TYPE          1000u

/* Bluetooth coexistence defaults the BTMODE block receives: the vendor
 * applies aicbsp_info (hwinfo = -1 default, cpmode = work) and the
 * aicbt_info[PRODUCT_ID_AIC8800D80] row before writing it. */
#define AICBT_HWINFO_DEFAULT        0xFFFFFFFFu /* (u32)(-1) */
#define AICBT_HWINFO_IS_DEFAULT     1u
#define AICBT_CPMODE_WORK           0u
#define AICBT_BTMODE_D80            5u /* BT_ONLY_COANT */
#define AICBT_BTPORT_MB             1u
#define AICBT_UART_BAUD_1_5M        1500000u
#define AICBT_UART_FC_ENABLE        1u
#define AICBT_LPM_ENABLE            0u
#define AICBT_TXPWR_LVL_D80         0x00006F2Fu

#define AIC_HOST_START_APP_AUTO     1u

/* The vendor compares sizeof(AICBT_PT_TAG) - 1 bytes of the 16-byte tag
 * field ("AICBT_PT_TAG" is 12 characters). */
#define AIC_FW_PT_TAG               "AICBT_PT_TAG"
#define AIC_FW_PT_TAG_LEN           12u
#define AIC_FW_PT_HDR_LEN           16u
#define AIC_FW_PT_BLOCK_HDR_LEN     24u /* name[16] + type + pair count */
#define AIC_FW_PT_PAIR_LEN          8u  /* { u32 addr; u32 value; } */

#define AIC_FW_PATH_MAX             112u
#define AIC_FW_EXT_NAME_MAX         48u
#define AIC_FW_EXT_SLEEP_MS         100u /* after a PWRON block */

/* ---- DBG message helpers ---- */

static bool fw_mem_read(uint32_t addr, uint32_t *out_value)
{
    aic_dbg_mem_read_req_t req;
    aic_dbg_mem_read_cfm_t cfm;

    req.memaddr = addr;
    aic_memset(&cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_DBG_MEM_READ_REQ, AIC_DBG_MEM_READ_CFM,
                 (uint16_t)AIC_TASK_DBG, &req, (uint16_t)sizeof(req),
                 &cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS)) {
        return false;
    }
    if (cfm.memaddr != addr) {
        return false; /* a reply to a different read: do not trust it */
    }
    *out_value = cfm.memdata;
    return true;
}

static bool fw_mem_write(uint32_t addr, uint32_t value)
{
    aic_dbg_mem_write_req_t req;
    aic_dbg_mem_write_cfm_t cfm;

    req.memaddr = addr;
    req.memdata = value;
    aic_memset(&cfm, 0u, sizeof(cfm));
    return aic_cmd(AIC_DBG_MEM_WRITE_REQ, AIC_DBG_MEM_WRITE_CFM,
                   (uint16_t)AIC_TASK_DBG, &req, (uint16_t)sizeof(req),
                   &cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS);
}

/* One vendor DBG_MEM_BLOCK_WRITE_REQ: at most AIC_FW_BLOCK_SIZE bytes to
 * `addr`. The message is a fixed-size struct (4 + 4 + 1024 bytes) whatever
 * `len` is, exactly like the vendor's rwnx_send_dbg_mem_block_write_req(),
 * which sends sizeof(struct dbg_mem_block_write_req) and only fills the
 * first `len` bytes of memdata. */
static bool fw_block_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    static aic_dbg_mem_block_write_req_t req;
    aic_dbg_mem_block_write_cfm_t cfm;

    if (len == 0u || len > AIC_FW_BLOCK_SIZE) {
        return false;
    }
    req.memaddr = addr;
    req.memsize = len;
    aic_memset(req.memdata, 0u, (uint32_t)sizeof(req.memdata));
    aic_memcpy(req.memdata, data, len);

    aic_memset(&cfm, 0u, sizeof(cfm));
    return aic_cmd(AIC_DBG_MEM_BLOCK_WRITE_REQ, AIC_DBG_MEM_BLOCK_WRITE_CFM,
                   (uint16_t)AIC_TASK_DBG, &req, (uint16_t)sizeof(req),
                   &cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS);
}

/* ---- file access ---- */

static bool fw_build_path(char *out, uint32_t cap, const char *name)
{
    uint32_t dir_len = aic_strnlen(AIC_FW_DIR, cap);
    uint32_t name_len = aic_strnlen(name, cap);

    if (dir_len + name_len + 1u > cap) {
        return false;
    }
    aic_memcpy(out, AIC_FW_DIR, dir_len);
    aic_memcpy(out + dir_len, name, name_len + 1u);
    return true;
}

static void fw_upload_abort(void)
{
    if (g_aic.file_open && g_aic_api != NULL &&
        g_aic_api->fs.close_file != NULL) {
        (void)g_aic_api->fs.close_file(&g_aic.file);
    }
    g_aic.file_open = false;
    g_aic.file_off = 0u;
    g_aic.file_size = 0u;
}

/* Open `name` from AIC_FW_DIR and prepare to stream it to `addr`. */
static bool fw_upload_begin(const char *name, uint32_t addr)
{
    char path[AIC_FW_PATH_MAX];
    uint32_t size;

    if (g_aic.file_open || g_aic_api == NULL) {
        return false;
    }
    if (!fw_build_path(path, (uint32_t)sizeof(path), name)) {
        return false;
    }
    if (!g_aic_api->fs.find_file(path, &g_aic.file)) {
        return false;
    }

    size = g_aic_api->fs.get_file_size(&g_aic.file);
    if (size == 0u || size > AIC_FW_FILE_MAX) {
        (void)g_aic_api->fs.close_file(&g_aic.file);
        return false;
    }

    g_aic.file_off = 0u;
    g_aic.file_size = size;
    g_aic.file_addr = addr;
    g_aic.file_open = true;

    aic_log("[AIC8800] flashing ");
    aic_log(name);
    aic_log(" at ");
    aic_log_hex32(addr);
    aic_log(" (");
    aic_log_hex32(size);
    aic_log(" bytes)\n");
    return true;
}

typedef enum {
    AIC_UP_MORE = 0, /* keep calling */
    AIC_UP_DONE,
    AIC_UP_FAIL,
} aic_upload_result_t;

/* Stream the open file, one 1 KiB block per DBG round-trip, at most
 * AIC_FW_CHUNKS_PER_STEP blocks per call. Mirrors the vendor's chunking:
 * full blocks while more than a block is left, then one last short one. */
static aic_upload_result_t fw_upload_pump(void)
{
    static uint8_t chunk[AIC_FW_BLOCK_SIZE];
    uint32_t budget = AIC_FW_CHUNKS_PER_STEP;

    if (!g_aic.file_open) {
        return AIC_UP_FAIL;
    }
    while (budget != 0u) {
        uint32_t remaining;
        uint32_t len;

        if (g_aic.file_off >= g_aic.file_size) {
            (void)g_aic_api->fs.close_file(&g_aic.file);
            g_aic.file_open = false;
            return AIC_UP_DONE;
        }
        remaining = g_aic.file_size - g_aic.file_off;
        len = (remaining > AIC_FW_BLOCK_SIZE) ? AIC_FW_BLOCK_SIZE : remaining;

        if (!g_aic_api->fs.read_at(&g_aic.file, g_aic.file_off, chunk, len)) {
            return AIC_UP_FAIL;
        }
        if (!fw_block_write(g_aic.file_addr + g_aic.file_off, chunk, len)) {
            return AIC_UP_FAIL;
        }
        g_aic.file_off += len;
        --budget;
    }
    return (g_aic.file_off >= g_aic.file_size) ? AIC_UP_DONE : AIC_UP_MORE;
}

/* Drive the upload the current phase wants: open it the first time, then
 * pump it. */
static aic_upload_result_t fw_upload_file(const char *name, uint32_t addr)
{
    if (!g_aic.file_open) {
        return fw_upload_begin(name, addr) ? AIC_UP_MORE : AIC_UP_FAIL;
    }
    return fw_upload_pump();
}

/* ---- patch table ---- */

typedef struct {
    uint32_t type;
    uint32_t pairs;      /* number of { u32 addr; u32 value; } entries */
    uint8_t *data;       /* pairs * 8 bytes, NULL when there are none */
} fw_pt_block_t;

/* Walk one block starting at *off; advance *off past it. Returns false when
 * the buffer ends or a block claims more data than the file holds. */
static bool fw_pt_next(uint32_t *off, fw_pt_block_t *out)
{
    uint8_t *base = g_aic.patch_table;
    uint32_t remaining;
    uint32_t pairs;

    if (*off >= g_aic.patch_table_len) {
        return false;
    }
    remaining = g_aic.patch_table_len - *off;
    if (remaining < AIC_FW_PT_BLOCK_HDR_LEN) {
        return false;
    }

    out->data = NULL;
    out->pairs = 0u;
    out->type = aic_rd32(base + *off + 16u);
    pairs = aic_rd32(base + *off + 20u);

    if (out->type >= AICBT_PT_SKIP_TYPE || pairs == 0u) {
        /* Address-only entry: the header is all there is. */
        *off += AIC_FW_PT_BLOCK_HDR_LEN;
        return true;
    }
    if (pairs > (remaining - AIC_FW_PT_BLOCK_HDR_LEN) / AIC_FW_PT_PAIR_LEN) {
        return false;
    }
    out->pairs = pairs;
    out->data = base + *off + AIC_FW_PT_BLOCK_HDR_LEN;
    *off += AIC_FW_PT_BLOCK_HDR_LEN + (pairs * AIC_FW_PT_PAIR_LEN);
    return true;
}

/* Apply the vendor's aicbt_patch_table_load() overrides to a BTMODE block,
 * in place (the buffer is ours). Word 1/3/5 are the host-provided hwinfo
 * and cpmode slots, 7..17 the Bluetooth coexistence settings; the D80 row
 * of aicbt_info[] is what the vendor fills in. */
static void fw_pt_apply_btmode(fw_pt_block_t *blk)
{
    if (blk->pairs < 9u) {
        return; /* malformed; write it as-is rather than past the block */
    }
    aic_wr32(blk->data + 4u, AICBT_HWINFO_IS_DEFAULT);
    aic_wr32(blk->data + 12u, AICBT_HWINFO_DEFAULT);
    aic_wr32(blk->data + 20u, AICBT_CPMODE_WORK);
    aic_wr32(blk->data + 28u, AICBT_BTMODE_D80);
    aic_wr32(blk->data + 36u, AICBT_BTPORT_MB);
    aic_wr32(blk->data + 44u, AICBT_UART_BAUD_1_5M);
    aic_wr32(blk->data + 52u, AICBT_UART_FC_ENABLE);
    aic_wr32(blk->data + 60u, AICBT_LPM_ENABLE);
    aic_wr32(blk->data + 68u, AICBT_TXPWR_LVL_D80);
}

/* "fw_patch_8800d80_u02_ext" + the decimal id + ".bin" (the vendor builds
 * this with sprintf("%s%d.bin", FW_PATCH_BASE_NAME_8800D80_U02_EXT, id)). */
static bool fw_ext_name(char *out, uint32_t cap, uint32_t id)
{
    static const char suffix[] = ".bin";
    uint32_t pos;
    uint32_t len = aic_strnlen(AIC_FW_FILE_PATCH_EXT, cap);
    char digits[8];
    uint32_t ndigits = 0u;

    if (len == 0u || len + 1u + 4u + 1u > cap) {
        return false;
    }
    for (pos = 0u; pos < len; ++pos) {
        out[pos] = AIC_FW_FILE_PATCH_EXT[pos];
    }
    do {
        digits[ndigits] = (char)('0' + (id % 10u));
        ++ndigits;
        id /= 10u;
    } while (id != 0u && ndigits < sizeof(digits));
    if (id != 0u) {
        return false; /* more digits than the vendor's %d would ever print */
    }
    while (ndigits != 0u) {
        --ndigits;
        out[pos] = digits[ndigits];
        ++pos;
    }
    for (uint32_t i = 0u; i < sizeof(suffix); ++i) { /* includes the NUL */
        out[pos] = suffix[i];
        ++pos;
    }
    return true;
}

/* Find and unpack the INF block (type 0) into the state. */
static bool fw_pt_parse_inf(void)
{
    uint32_t off = AIC_FW_PT_HDR_LEN;
    fw_pt_block_t blk;
    bool found = false;

    while (fw_pt_next(&off, &blk)) {
        uint32_t words;
        uint32_t copied;

        if (blk.type != AICBT_PT_INF || blk.data == NULL || blk.pairs == 0u) {
            continue;
        }
        words = blk.pairs * 2u;
        /* The vendor reads base_len (4) pairs, or the whole block when it
         * is shorter, and takes one more pair when there is one -- the
         * last field it copies is ext_patch_nb at word 9. */
        copied = (blk.pairs > 4u) ? 10u : words;
        if (copied > words) {
            copied = words;
        }

        if (copied >= 2u) {
            g_aic.addr_adid = aic_rd32(blk.data + 4u);
        }
        if (copied >= 4u) {
            g_aic.addr_patch = aic_rd32(blk.data + 12u);
        }
        if (copied >= 6u) {
            g_aic.reset_addr = aic_rd32(blk.data + 16u);
            g_aic.reset_val = aic_rd32(blk.data + 20u);
        }
        if (copied >= 8u) {
            g_aic.adid_flag_addr = aic_rd32(blk.data + 24u);
            g_aic.adid_flag = aic_rd32(blk.data + 28u);
        }
        if (blk.pairs > 4u && words >= 10u) {
            uint32_t nb = aic_rd32(blk.data + 36u); /* word 9 */
            uint32_t ext_words = words - 10u;
            uint32_t i;

            for (i = 0u; i < nb && i < AIC_FW_EXT_PATCH_MAX; ++i) {
                if ((i * 2u + 2u) > ext_words) {
                    break;
                }
                g_aic.ext_patch_id[i] = aic_rd32(blk.data + 40u + (i * 8u));
                g_aic.ext_patch_addr[i] = aic_rd32(blk.data + 44u + (i * 8u));
            }
            g_aic.ext_patch_nb = i;
            if (nb > i) {
                aic_log("[AIC8800] patch table lists more ext images than "
                        "this driver supports; ignoring the rest\n");
            }
        }
        found = true;
        break;
    }
    return found;
}

/* Read the whole patch table into memory and unpack it. */
static bool fw_pt_load(void)
{
    char path[AIC_FW_PATH_MAX];
    uint32_t size;
    uint32_t off;
    uint32_t walk;
    fw_pt_block_t blk;

    if (!fw_build_path(path, (uint32_t)sizeof(path), AIC_FW_FILE_PATCH_TABLE)) {
        return false;
    }
    if (!g_aic_api->fs.find_file(path, &g_aic.file)) {
        aic_log("[AIC8800] " AIC_FW_FILE_PATCH_TABLE " not found in " AIC_FW_DIR "\n");
        return false;
    }

    size = g_aic_api->fs.get_file_size(&g_aic.file);
    if (size <= AIC_FW_PT_HDR_LEN + AIC_FW_PT_BLOCK_HDR_LEN ||
        size > AIC_FW_PATCH_TABLE_MAX) {
        (void)g_aic_api->fs.close_file(&g_aic.file);
        return false;
    }

    g_aic.patch_table = (uint8_t *)g_aic_api->malloc(size);
    if (g_aic.patch_table == NULL) {
        (void)g_aic_api->fs.close_file(&g_aic.file);
        return false;
    }

    off = 0u;
    while (off < size) {
        uint32_t chunk = size - off;
        if (chunk > AIC_FW_BLOCK_SIZE) {
            chunk = AIC_FW_BLOCK_SIZE;
        }
        if (!g_aic_api->fs.read_at(&g_aic.file, off, g_aic.patch_table + off,
                                   chunk)) {
            (void)g_aic_api->fs.close_file(&g_aic.file);
            return false;
        }
        off += chunk;
    }
    (void)g_aic_api->fs.close_file(&g_aic.file);
    g_aic.patch_table_len = size;

    if (!aic_memequal(g_aic.patch_table, AIC_FW_PT_TAG, AIC_FW_PT_TAG_LEN)) {
        aic_log("[AIC8800] " AIC_FW_FILE_PATCH_TABLE " is not a patch table\n");
        return false;
    }

    /* Validate the whole chain before anything is written to the chip. */
    walk = AIC_FW_PT_HDR_LEN;
    while (walk < size) {
        if (!fw_pt_next(&walk, &blk)) {
            aic_log("[AIC8800] malformed patch table\n");
            return false;
        }
    }
    if (!fw_pt_parse_inf()) {
        aic_log("[AIC8800] patch table has no INF block\n");
        return false;
    }

    aic_log("[AIC8800] patch table: adid ");
    aic_log_hex32(g_aic.addr_adid);
    aic_log(", patch ");
    aic_log_hex32(g_aic.addr_patch);
    aic_log(", ext images ");
    aic_log_hex32(g_aic.ext_patch_nb);
    aic_log("\n");
    return true;
}

static void fw_pt_free(void)
{
    if (g_aic.patch_table != NULL) {
        if (g_aic_api != NULL && g_aic_api->free != NULL) {
            g_aic_api->free(g_aic.patch_table);
        }
        g_aic.patch_table = NULL;
    }
    g_aic.patch_table_len = 0u;
    g_aic.pt_off = 0u;
    g_aic.pt_pair = 0u;
}

/* Write the patch table's blocks, one (address, value) pair at a time,
 * bounded per call. The version block is skipped (the vendor only logs
 * its text), a PWRON block is followed by the vendor's 100 ms settle
 * delay, and BTMODE gets its coexistence overrides first. */
static aic_upload_result_t fw_pt_write(void)
{
    uint32_t budget = AIC_FW_WRITES_PER_STEP;

    if (g_aic.patch_table == NULL) {
        return AIC_UP_FAIL;
    }
    while (budget != 0u) {
        uint32_t off = g_aic.pt_off;
        fw_pt_block_t blk;

        if (off >= g_aic.patch_table_len) {
            return AIC_UP_DONE;
        }
        if (!fw_pt_next(&off, &blk)) {
            return AIC_UP_FAIL;
        }
        if (blk.data == NULL || blk.pairs == 0u) {
            g_aic.pt_off = off;
            continue;
        }
        if (g_aic.pt_pair == 0u && blk.type == AICBT_PT_BTMODE) {
            fw_pt_apply_btmode(&blk);
        }
        if (g_aic.pt_pair < blk.pairs && blk.type != AICBT_PT_VER) {
            uint8_t *pair = blk.data + (g_aic.pt_pair * AIC_FW_PT_PAIR_LEN);
            if (!fw_mem_write(aic_rd32(pair), aic_rd32(pair + 4u))) {
                return AIC_UP_FAIL;
            }
            --budget;
        }
        ++g_aic.pt_pair;

        if (g_aic.pt_pair >= blk.pairs) {
            g_aic.pt_off = off;
            g_aic.pt_pair = 0u;
            if (blk.type == AICBT_PT_PWRON) {
                aic_msleep(AIC_FW_EXT_SLEEP_MS);
            }
        }
    }
    return AIC_UP_MORE;
}

/* ---- the ladder ---- */

static bool fw_sys_config(void)
{
    uint32_t value = 0u;

    /* The loader's own idea of which chip it is on: bit 25 of the boot
     * status word is clear on the "MCU owns the cache" revision, and the
     * revision itself is in bits 16..23. */
    if (!fw_mem_read(AIC_SYSCFG_CHIPID_ADDR, &value)) {
        return false;
    }
    if (((value >> 25) & 1u) == 0u) {
        g_aic.chip_mcu_id = 1u;
    }
    g_aic.chip_id = (uint8_t)((value >> 16) & 0xFFu);

    aic_log("[AIC8800] chip id ");
    aic_log_hex32((uint32_t)g_aic.chip_id);
    aic_log(g_aic.chip_mcu_id != 0u ? " (mcu) " : " (rom) ");
    aic_log(AIC_CHIP_ID_H(g_aic.chip_id) ? "H-variant\n" : "standard\n");

    if (g_aic.chip_mcu_id != 0u) {
        if (!fw_mem_read(AIC_SYSCFG_CACHE_ADDR, &value)) {
            return false;
        }
        if (!fw_mem_write(AIC_SYSCFG_CACHE_ADDR, value | 1u)) {
            return false;
        }
    }
    /* syscfg_tbl_8800d80[] and syscfg_tbl_masked_8800d80[] are both empty
     * in the vendor build (CONFIG_PMIC_SETTING, CONFIG_RADAR_OR_IR_DETECT
     * and CONFIG_WOWLAN are all off for USB), so the tables have nothing
     * to program on this part. */
    return true;
}

/* The firmware's own patch descriptor (aicwf_patch_config_8800d80): the
 * downloaded image has just told us where its patch area lives (0x120198),
 * so write the descriptor -- magic words, the pair table's address and
 * count, and the three (offset, value) pairs that switch on the D80's
 * per-chip behaviour -- plus a zeroed block-descriptor array. */
static bool fw_patch_config(void)
{
    static const uint32_t pair_table[AIC_PATCH_PAIR_COUNT][2] = {
        { 0x00B4u, 0xF3010000u }, /* front-end config: 2.4 GHz only */
        { 0x0170u, 0x0001000Au }, /* RX aggregation counter */
        { 0x0188u, 0x00000001u }, /* user_ext_flags: PWROFST cover calib */
    };
    uint32_t config_base = 0u;
    uint32_t str_base = 0u;
    uint32_t version = 0u;
    uint32_t start_addr = AIC_PATCH_START_ADDR;
    uint32_t i;

    if (!fw_mem_read(AIC_RAM_FMAC_FW_ADDR + AIC_PATCH_CONFIG_OFS, &config_base)) {
        return false;
    }
    if (!fw_mem_read(AIC_RAM_FMAC_FW_ADDR + AIC_PATCH_STR_OFS, &str_base)) {
        return false;
    }
    if (!fw_mem_read(AIC_RAM_FMAC_FW_ADDR + AIC_PATCH_VERSION_OFS, &version)) {
        return false;
    }
    g_aic.fw_version = version;
    if (version > AIC_PATCH_VERSION_NEW) {
        uint32_t buff_base = 0u;
        if (!fw_mem_read(AIC_RAM_FMAC_FW_ADDR + AIC_PATCH_BUFF_OFS, &buff_base)) {
            return false;
        }
        start_addr = buff_base;
    }

    /* struct aic_patch, laid out at str_base. */
    if (!fw_mem_write(str_base + 0u, AIC_PATCH_MAGIC_NUM) ||
        !fw_mem_write(str_base + 8u, AIC_PATCH_MAGIC_NUM_2) ||
        !fw_mem_write(str_base + 4u, start_addr) ||
        !fw_mem_write(str_base + 12u, AIC_PATCH_PAIR_COUNT)) {
        return false;
    }
    for (i = 0u; i < AIC_PATCH_PAIR_COUNT; ++i) {
        if (!fw_mem_write(start_addr + (8u * i),
                          pair_table[i][0] + config_base) ||
            !fw_mem_write(start_addr + (8u * i) + 4u, pair_table[i][1])) {
            return false;
        }
    }
    for (i = 0u; i < AIC_PATCH_BLOCK_MAX; ++i) {
        if (!fw_mem_write(str_base + 48u + (4u * i), 0u)) {
            return false;
        }
    }

    aic_log("[AIC8800] firmware version ");
    aic_log_hex32(version);
    aic_log(", patch descriptor at ");
    aic_log_hex32(str_base);
    aic_log("\n");
    return true;
}

static bool fw_start_app(void)
{
    aic_dbg_start_app_req_t req;

    req.bootaddr = AIC_RAM_FMAC_FW_ADDR;
    req.boottype = AIC_HOST_START_APP_AUTO;
    /* Fire and forget: the chip resets its USB side to run the new image,
     * so no confirmation is ever going to come back on this endpoint. */
    return aic_cmd_nowait(AIC_DBG_START_APP_REQ, (uint16_t)AIC_TASK_DBG,
                          &req, (uint16_t)sizeof(req));
}

/* ---- failure and retry ---- */

static void fw_ladder_reset(void)
{
    fw_pt_free();
    fw_upload_abort();
    g_aic.addr_adid = 0u;
    g_aic.addr_patch = 0u;
    g_aic.ext_patch_nb = 0u;
    g_aic.ext_patch_index = 0u;
    g_aic.fw_version = 0u;
    g_aic.fw_phase = AIC_FW_SYS_CONFIG;
}

static void fw_fail(const char *what)
{
    aic_log("[AIC8800] firmware download: ");
    aic_log(what);
    aic_log(" failed (attempt ");
    aic_log_hex32(g_aic.fw_attempts + 1u);
    aic_log("/");
    aic_log_hex32(AIC_MAX_ATTEMPTS);
    aic_log(")\n");

    fw_pt_free();
    fw_upload_abort();

    g_aic.fw_phase = AIC_FW_IDLE;
    ++g_aic.fw_attempts;
    if (g_aic.fw_attempts >= AIC_MAX_ATTEMPTS) {
        g_aic.fw_phase = AIC_FW_FAILED;
        aic_log("[AIC8800] giving up: are the firmware blobs installed in "
                AIC_FW_DIR "?\n");
    }
}

void aic_fw_reset(void)
{
    /* A fresh instance (probe() already zeroed the state): only the
     * allocation and the file handle can survive a re-attach. */
    fw_pt_free();
    fw_upload_abort();
    g_aic.fw_phase = AIC_FW_IDLE;
    g_aic.fw_attempts = 0u;
    g_aic.fw_last_attempt = 0u;
    g_aic.fw_done = false;
}

/* ---- entry point ---- */

void aic_fw_step(void)
{
    aic_upload_result_t r;

    if (!g_aic.attached || g_aic_api == NULL) {
        return;
    }
    if (g_aic.fw_phase == AIC_FW_FAILED || g_aic.fw_phase == AIC_FW_DONE) {
        return;
    }

    if (g_aic.fw_phase == AIC_FW_IDLE) {
        /* Retry pacing: the blobs may only become readable once the
         * filesystems are mounted, and this must cost nothing when they
         * are never installed at all. */
        if (g_aic.fw_attempts != 0u &&
            aic_ms_since(g_aic.fw_last_attempt, AIC_RETRY_INTERVAL_MS) <
                AIC_RETRY_INTERVAL_MS) {
            return;
        }
        g_aic.fw_last_attempt = aic_now_ticks();
        fw_ladder_reset();
    }

    switch (g_aic.fw_phase) {
    case AIC_FW_SYS_CONFIG:
        if (!fw_sys_config()) {
            fw_fail("chip configuration");
            return;
        }
        g_aic.fw_phase = AIC_FW_PATCH_TABLE;
        break;

    case AIC_FW_PATCH_TABLE:
        if (!fw_pt_load()) {
            fw_fail("patch table read");
            return;
        }
        g_aic.fw_phase = AIC_FW_LOAD_ADID;
        break;

    case AIC_FW_LOAD_ADID:
        r = fw_upload_file(AIC_FW_FILE_ADID, g_aic.addr_adid);
        if (r == AIC_UP_DONE) {
            g_aic.fw_phase = AIC_FW_LOAD_PATCH;
        } else if (r == AIC_UP_FAIL) {
            fw_fail("adid image");
        }
        break;

    case AIC_FW_LOAD_PATCH:
        r = fw_upload_file(AIC_FW_FILE_PATCH, g_aic.addr_patch);
        if (r == AIC_UP_DONE) {
            g_aic.fw_phase = (g_aic.ext_patch_nb != 0u) ? AIC_FW_LOAD_EXT
                                                        : AIC_FW_PATCH_TABLE_LOAD;
        } else if (r == AIC_UP_FAIL) {
            fw_fail("patch image");
        }
        break;

    case AIC_FW_LOAD_EXT: {
        char name[AIC_FW_EXT_NAME_MAX];

        if (g_aic.ext_patch_index >= g_aic.ext_patch_nb) {
            g_aic.fw_phase = AIC_FW_PATCH_TABLE_LOAD;
            break;
        }
        if (!fw_ext_name(name, (uint32_t)sizeof(name),
                         g_aic.ext_patch_id[g_aic.ext_patch_index])) {
            fw_fail("ext patch name");
            return;
        }
        r = fw_upload_file(name, g_aic.ext_patch_addr[g_aic.ext_patch_index]);
        if (r == AIC_UP_DONE) {
            ++g_aic.ext_patch_index;
        } else if (r == AIC_UP_FAIL) {
            fw_fail("ext patch image");
        }
        break;
    }

    case AIC_FW_PATCH_TABLE_LOAD:
        r = fw_pt_write();
        if (r == AIC_UP_DONE) {
            g_aic.fw_phase = AIC_FW_LOAD_FMAC;
        } else if (r == AIC_UP_FAIL) {
            fw_fail("patch table write");
        }
        break;

    case AIC_FW_LOAD_FMAC:
        r = fw_upload_file(AIC_CHIP_ID_H(g_aic.chip_id) ? AIC_FW_FILE_FMAC_H
                                                        : AIC_FW_FILE_FMAC,
                           AIC_RAM_FMAC_FW_ADDR);
        if (r == AIC_UP_DONE) {
            g_aic.fw_phase = AIC_FW_PATCH_CONFIG;
        } else if (r == AIC_UP_FAIL) {
            fw_fail("firmware image");
        }
        break;

    case AIC_FW_PATCH_CONFIG:
        if (!fw_patch_config()) {
            fw_fail("patch descriptor");
            return;
        }
        g_aic.fw_phase = AIC_FW_START_APP;
        break;

    case AIC_FW_START_APP:
        if (!fw_start_app()) {
            fw_fail("start app");
            return;
        }
        aic_log("[AIC8800] firmware running; waiting for the device to "
                "come back as 8d81\n");
        g_aic.fw_done = true;
        g_aic.fw_phase = AIC_FW_DONE;
        break;

    default:
        break;
    }
}
