#include "AX900.h"
#include "AX900_Protocol.h"

#include "Drivers/Module/DriverBinary.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * AX900.c -- see AX900.h/AX900_Protocol.h for what this is and where its
 * constants come from.
 *
 * What is real and exercised in this build:
 *   - USB packet framing (build + parse), ported from aicwf_process_rxframes()
 *     / aicwf_usb_bus_txmsg() in the reference driver.
 *   - The ax900_msg_hdr_t command/response envelope and message id scheme.
 *   - The DBG_MEM_READ_REQ chip-id bring-up step, ax900_bring_up()'s
 *     MM_RESET_REQ/MM_ADD_IF_REQ station-interface bring-up, a real
 *     2.4GHz 14-channel SCANU_START_REQ scan (with SCANU_RESULT_IND
 *     parsing into driver_wifi_scan_result_t -- SSID + open/secured
 *     guess from the beacon/probe-resp IEs), and SM_CONNECT_REQ/
 *     SM_DISCONNECT_REQ station commands, all built to the real wire
 *     layout. Exposed to userland end-to-end via the driver_nic_t.wifi_*
 *     vtable (DriverBinary.h) -> NicManager/DriverManager ->
 *     SYSCALL_WIFI_* (Syscall_Dispatch.c) -> Userland/API/WiFi.h -> the
 *     WindowManager's Wi-Fi panel (Userland/Application/
 *     com.ImplusOS.windowmanager/UI/WM_WifiPanel.c).
 *   - ax900_download_firmware(): reads real firmware bytes via g_api->fs
 *     (driver_binary_t API 2.2) from /Kernel/Driver/Firmware/AX900/ and
 *     pushes them to the chip via DBG_MEM_WRITE_REQ. This is retried
 *     opportunistically from ax900_poll() (not blocked on from ax900_probe())
 *     so it tolerates VFS not being mounted yet at USB enumeration time.
 *     It still won't succeed in this repository, because AICSemi's
 *     firmware .bin files are proprietary and were never available to
 *     embed here -- find_file() simply returns false until someone
 *     places the real files at that path, same relationship Linux has
 *     with the separate linux-firmware package. Everything downstream
 *     (bring-up, scan, connect) is consequently untestable without real
 *     hardware + those files, however protocol-correct it is.
 *   - Once associated, Kernel/Network/network_main.c automatically kicks
 *     off DHCP (dhcp_discover(), retried every ~2s until leased) --
 *     nothing did that for *any* NIC before this, wired or wireless; the
 *     stack previously only ever ran on the static QEMU-NAT-shaped
 *     address in OS_CONFIG_NET_IPV4_*.
 *
 * What is still an honest stub, and why:
 *   - ax900_send_msg()'s wait for a reply times out whenever the chip isn't
 *     actually running firmware yet (i.e. today, absent real firmware
 *     files); the wait/signal plumbing itself (g_api->event) is real.
 *   - WPA2-PSK: ax900_build_wpa2_ies() builds a real, spec-correct RSN IE,
 *     but how the passphrase itself reaches AICSemi's firmware over this
 *     transport was never captured from source (the reference driver's
 *     own struct sm_connect_req.ie_buf comment doesn't say either) -- see
 *     AX900_Protocol.h's "WPA2-PSK key material handoff" comment.
 *     ax900_connect() sends its best-effort guess (raw passphrase in a
 *     private vendor IE) rather than refusing outright; open networks are
 *     unaffected either way and were already fully correct.
 *   - fw_patch_table_8800d80_u02.bin and fw_adid_8800d80_u02.bin are not
 *     blob-copied by ax900_download_firmware() -- see the comment there.
 *   - No real MAC address (efuse read never captured, see
 *     ax900_generate_local_mac()): a stable locally-administered address
 *     is generated instead.
 *
 * Module shape: this is a standalone driver module (its own .ELF, kind =
 * DEVICE_TYPE_NIC), *not* compiled into USB_Driver.ELF the way HID/
 * MassStorage still are. USB_Main.c has no direct visibility into this
 * file at all -- it enumerates a vendor-specific interface, builds a
 * bus_device_t (+ usb_device_context_t bus_context carrying the addr/
 * interface/endpoints and its own usb_submit_bulk so this module can move
 * data despite being a separate link unit), and calls
 * g_api->bus.report_device(). BusRegistry.c (Kernel/Drivers/Module/) then
 * matches it against g_ax900_bus_matches[] below and calls ax900_probe().
 */

#include "kernel/interfaces/vfs_file.h"

static const driver_binary_t *g_api = NULL;

/* Bounded so wifi_get_scan_results() copies fit comfortably on the
 * syscall dispatcher's stack (see SYSCALL_WIFI_GET_SCAN_RESULTS) -- plenty
 * for any real 2.4GHz neighborhood (14 channels total). */
#define AX900_MAX_SCAN_RESULTS 24u

#define hal_cpu_save_interrupts    g_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts g_api->hal.cpu_restore_interrupts
#define hal_cpu_pause              g_api->hal.cpu_pause

typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_lock(spinlock_t *l)
{
    while (__sync_lock_test_and_set(&l->locked, 1)) {
        while (l->locked) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->locked); }

static inline uint64_t irq_save_disable(void) { return hal_cpu_save_interrupts(); }
static inline void irq_restore(uint64_t flags) { hal_cpu_restore_interrupts(flags); }

static size_t ax900_strnlen(const char *s, size_t max)
{
    size_t n = 0;
    while (n < max && s[n] != '\0') { n++; }
    return n;
}

/* driver_binary_t has no memcmp -- see DriverBinary.h -- only memset/memcpy. */
static bool ax900_bytes_equal(const void *a, const void *b, size_t n)
{
    const uint8_t *pa = (const uint8_t *)a;
    const uint8_t *pb = (const uint8_t *)b;
    for (size_t i = 0u; i < n; ++i) {
        if (pa[i] != pb[i]) return false;
    }
    return true;
}

/* ---- device state ---- */

typedef struct {
    bool     attached;
    uint8_t  addr;
    uint8_t  interface;
    uint8_t  ep_in;
    uint8_t  ep_out;
    uint16_t ep_in_mps;
    uint16_t ep_out_mps;
    bool (*submit_bulk)(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                        uint8_t pid, void *data, uint32_t length);

    bool     fw_loaded; /* always false today -- see ax900_download_firmware() */
    uint8_t  chip_id;
    uint8_t  chip_sub_id;

    /* Post-firmware bring-up (MM_RESET_REQ / MM_ADD_IF_REQ, see
     * ax900_bring_up()): gates scan/connect the same way fw_loaded gates
     * everything before it. */
    bool     bringup_done;
    uint8_t  vif_idx;

    uint8_t  mac[6];
    uint16_t mtu;
    bool     associated;
    bool     connect_failed;   /* last ax900_connect() attempt didn't reach SM_CONNECT_IND success */
    char     target_ssid[DRIVER_WIFI_SSID_MAX + 1u]; /* set by ax900_connect(), for status reporting */

    bool     scanning;
    uint64_t scan_started_ticks;
    driver_wifi_scan_result_t scan_results[AX900_MAX_SCAN_RESULTS];
    uint32_t scan_result_count;

    spinlock_t lock;
    ax900_rx_callback_t rx_callback;
} ax900_state_t;

static ax900_state_t g_ax900 = {0};

/* One outstanding request/response wait at a time -- adequate for the
 * bring-up/scan/connect command sequence, which is inherently serial. */
typedef struct {
    bool     waiting;
    uint16_t expect_msg_id;
    void    *out_buf;
    uint16_t out_cap;
    uint16_t out_len;
    void    *event;
} ax900_pending_cfm_t;

static ax900_pending_cfm_t g_pending = {0};

/* ---- USB packet framing (AX900_Protocol.h) ---- */

static bool ax900_usb_send_packet(uint8_t type, const void *payload, uint16_t payload_len)
{
    if (!g_ax900.attached || g_ax900.ep_out == 0u || g_ax900.submit_bulk == NULL) {
        return false;
    }
    if (payload_len > AX900_CMD_BUF_MAX - AX900_USB_HDR_LEN) {
        return false;
    }

    uint8_t buf[AX900_CMD_BUF_MAX];
    buf[0] = (uint8_t)(payload_len & 0xFFu);
    buf[1] = (uint8_t)((payload_len >> 8) & 0xFFu);
    buf[2] = type;
    buf[3] = 0u;
    if (payload_len > 0u && payload != NULL) {
        g_api->memcpy(buf + AX900_USB_HDR_LEN, payload, payload_len);
    }

    /* The reference driver sends the CFG/message channel over a dedicated
     * bulk pipe only when CONFIG_USB_MSG_EP selects one; otherwise it
     * shares the data OUT pipe, exactly as done here (ax900_probe() below
     * only captures a single bulk pair) -- see aicwf_usb_bus_txmsg() in
     * aic8800_fdrv/aicwf_usb.c. A dedicated msg pipe is a real TODO
     * enhancement, not a functional requirement. */
    uint16_t total = (uint16_t)(AX900_USB_HDR_LEN + payload_len);
    return g_ax900.submit_bulk(g_ax900.addr, g_ax900.ep_out, g_ax900.ep_out_mps, 0u, buf, total);
}

/* ---- message send + wait for the matching *_CFM ---- */

static bool ax900_send_msg(uint16_t msg_id, uint16_t expect_cfm_id,
                           const void *param, uint16_t param_len,
                           void *cfm_out, uint16_t cfm_cap, uint32_t timeout_ms)
{
    if (g_api->event.create == NULL || g_api->event.wait == NULL ||
        g_api->event.signal == NULL || g_api->event.destroy == NULL) {
        return false;
    }
    if ((uint32_t)sizeof(ax900_msg_hdr_t) + param_len > AX900_CMD_BUF_MAX - AX900_USB_HDR_LEN) {
        return false;
    }

    uint8_t body[AX900_CMD_BUF_MAX - AX900_USB_HDR_LEN];
    ax900_msg_hdr_t hdr;
    g_api->memset(&hdr, 0, sizeof(hdr));
    hdr.id = msg_id;
    hdr.param_len = param_len;
    hdr.pattern = AX900_MSG_VALID_PATTERN;
    g_api->memcpy(body, &hdr, sizeof(hdr));
    if (param_len > 0u && param != NULL) {
        g_api->memcpy(body + sizeof(hdr), param, param_len);
    }

    void *event = g_api->event.create();
    if (event == NULL) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    g_pending.waiting = true;
    g_pending.expect_msg_id = expect_cfm_id;
    g_pending.out_buf = cfm_out;
    g_pending.out_cap = cfm_cap;
    g_pending.out_len = 0u;
    g_pending.event = event;
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);

    bool sent = ax900_usb_send_packet(AX900_PKT_TYPE_CFG, body,
                                      (uint16_t)(sizeof(hdr) + param_len));

    /* NOTE: nothing here spawns a thread to pump ax900_poll() -- driver
     * modules have no thread-create primitive in driver_binary_t. Whatever
     * drives NIC/USB polling elsewhere in the system must keep calling
     * ax900_poll() for this wait to ever succeed; on its own this call
     * will simply time out, which today it always does anyway because
     * firmware was never loaded (see file header). */
    bool got_cfm = sent && g_api->event.wait(event, timeout_ms);

    irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    g_pending.waiting = false;
    g_pending.event = NULL;
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);

    g_api->event.destroy(event);
    return got_cfm;
}

/* Forward declaration: defined further down alongside ax900_bring_up()
 * (post-firmware bring-up), called from ax900_handle_cfg_rsp_locked()
 * below (RX dispatch, defined first). */
static void ax900_parse_scan_payload(const ax900_scanu_result_ind_hdr_t *hdr,
                                     const uint8_t *payload, uint16_t payload_len);

/* ---- RX: CFG_CMD_RSP dispatch (called with g_ax900.lock held) ---- */

static void ax900_handle_cfg_rsp_locked(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(ax900_msg_hdr_t)) {
        return;
    }

    ax900_msg_hdr_t hdr;
    g_api->memcpy(&hdr, payload, sizeof(hdr));
    if (hdr.pattern != AX900_MSG_VALID_PATTERN) {
        return; /* not a genuine message buffer */
    }

    const uint8_t *param = payload + sizeof(ax900_msg_hdr_t);
    uint16_t avail = (uint16_t)(len - sizeof(ax900_msg_hdr_t));
    uint16_t param_len = (hdr.param_len <= avail) ? hdr.param_len : avail;

    if (g_pending.waiting && hdr.id == g_pending.expect_msg_id) {
        uint16_t copy_len = (param_len <= g_pending.out_cap) ? param_len : g_pending.out_cap;
        if (g_pending.out_buf != NULL && copy_len > 0u) {
            g_api->memcpy(g_pending.out_buf, param, copy_len);
        }
        g_pending.out_len = copy_len;
        g_api->event.signal(g_pending.event);
    }

    if (hdr.id == AX900_SM_CONNECT_IND && param_len >= sizeof(ax900_sm_connect_ind_t)) {
        ax900_sm_connect_ind_t ind;
        g_api->memcpy(&ind, param, sizeof(ind));
        g_ax900.associated = (ind.status_code == 0u);
        g_ax900.connect_failed = !g_ax900.associated;
    } else if (hdr.id == AX900_SM_DISCONNECT_IND) {
        g_ax900.associated = false;
    } else if (hdr.id == AX900_SCANU_RESULT_IND &&
              param_len >= sizeof(ax900_scanu_result_ind_hdr_t) && g_ax900.scanning) {
        ax900_scanu_result_ind_hdr_t ind_hdr;
        g_api->memcpy(&ind_hdr, param, sizeof(ind_hdr));
        const uint8_t *ie_payload = param + sizeof(ind_hdr);
        uint16_t ie_avail = (uint16_t)(param_len - sizeof(ind_hdr));
        uint16_t ie_len = (ind_hdr.length <= ie_avail) ? ind_hdr.length : ie_avail;
        ax900_parse_scan_payload(&ind_hdr, ie_payload, ie_len);
    }
}

/* ---- module-level init (called once from driver_module_init() below) ---- */

void ax900_init(void)
{
    g_api->memset(&g_ax900, 0, sizeof(g_ax900));
    g_api->memset(&g_pending, 0, sizeof(g_pending));
}

static bool ax900_read_chip_id(void)
{
    ax900_dbg_mem_read_req_t req_id = { .mem_addr = AX900_REG_CHIP_ID_ADDR };
    ax900_dbg_mem_read_cfm_t cfm_id = {0};
    if (!ax900_send_msg(AX900_DBG_MEM_READ_REQ, AX900_DBG_MEM_READ_CFM,
                        &req_id, sizeof(req_id), &cfm_id, sizeof(cfm_id), 1000u)) {
        return false;
    }
    g_ax900.chip_id = (uint8_t)((cfm_id.mem_data >> 16) & 0xFFu);

    ax900_dbg_mem_read_req_t req_sub = { .mem_addr = 0x00000004u };
    ax900_dbg_mem_read_cfm_t cfm_sub = {0};
    if (!ax900_send_msg(AX900_DBG_MEM_READ_REQ, AX900_DBG_MEM_READ_CFM,
                        &req_sub, sizeof(req_sub), &cfm_sub, sizeof(cfm_sub), 1000u)) {
        return false;
    }
    g_ax900.chip_sub_id = (uint8_t)((cfm_sub.mem_data >> 4) & 0xFFu);
    return true;
}

#define AX900_FW_DIR              "/Kernel/Driver/Firmware/AX900/"
#define AX900_FW_MAX_BLOB_BYTES   (512u * 1024u)

/* Reads `path` in full via g_api->fs and pushes it to the chip's RAM at
 * `ram_addr` one 32-bit word at a time via DBG_MEM_WRITE_REQ. That message
 * is confirmed from source for small config-table pokes (see
 * system_config_8800() in the reference driver's aic_load_fw/aicwf_usb.c);
 * a real bulk firmware image is very likely pushed through a dedicated,
 * faster bulk-load path in that driver which this research did not
 * capture (TODO). Word-at-a-time DBG_MEM_WRITE_REQ is protocol-correct but
 * would be far too slow for a real multi-hundred-KB image if the chip were
 * actually responding -- acceptable for now since the loop bails on the
 * first failed write, and in this repository (no proprietary .bin files
 * present) find_file() below fails immediately every time regardless. */
static bool ax900_load_firmware_blob(const char *path, uint32_t ram_addr)
{
    if (g_api->fs.find_file == NULL || g_api->fs.read_at == NULL ||
        g_api->fs.get_file_size == NULL || g_api->fs.close_file == NULL) {
        return false;
    }

    vfs_file_t file;
    g_api->memset(&file, 0, sizeof(file));
    if (!g_api->fs.find_file(path, &file)) {
        return false; /* not present yet (proprietary blob not installed), or VFS not mounted yet */
    }

    uint32_t size = g_api->fs.get_file_size(&file);
    if (size == 0u || size > AX900_FW_MAX_BLOB_BYTES) {
        g_api->fs.close_file(&file);
        return false;
    }

    uint8_t *buf = (uint8_t *)g_api->mem.malloc(size);
    if (buf == NULL) {
        g_api->fs.close_file(&file);
        return false;
    }

    bool read_ok = g_api->fs.read_at(&file, 0u, buf, size);
    g_api->fs.close_file(&file);
    if (!read_ok) {
        g_api->mem.free(buf);
        return false;
    }

    bool write_ok = true;
    for (uint32_t off = 0u; (off + 4u) <= size && write_ok; off += 4u) {
        uint32_t word = (uint32_t)buf[off] |
                        ((uint32_t)buf[off + 1u] << 8) |
                        ((uint32_t)buf[off + 2u] << 16) |
                        ((uint32_t)buf[off + 3u] << 24);
        ax900_dbg_mem_write_req_t req = { .mem_addr = ram_addr + off, .mem_data = word };
        ax900_dbg_mem_write_cfm_t cfm = {0};
        write_ok = ax900_send_msg(AX900_DBG_MEM_WRITE_REQ, AX900_DBG_MEM_WRITE_CFM,
                                  &req, sizeof(req), &cfm, sizeof(cfm), 200u);
    }

    g_api->mem.free(buf);
    return write_ok;
}

static bool ax900_download_firmware(void)
{
    /* Firmware images confirmed (by the RAM_*_ADDR_U02 naming/pairing in
     * the reference driver's aic_compat_8800d80.h) to be a straight blob
     * copy to a fixed RAM address:
     *   fmacfw_8800d80_u02.bin    -> AX900_RAM_FMAC_FW_ADDR_U02
     *   lmacfw_rf_8800d80_u02.bin -> AX900_RAM_FMAC_RF_FW_ADDR_U02
     *   fw_patch_8800d80_u02.bin  -> AX900_RAM_PATCH_BASE_ADDR_U02
     *
     * fw_patch_table_8800d80_u02.bin and fw_adid_8800d80_u02.bin are NOT
     * blob-copied by the reference driver: the patch table is parsed
     * host-side into individual (address, value) writes (see
     * system_config_8800() in aic_load_fw/aicwf_usb.c), and ADID is
     * revision-specific calibration data with its own handling. Neither
     * was captured precisely enough by this research to reproduce
     * correctly, so both are deliberately left out rather than guessed --
     * TODO once that handling is confirmed from source. */
    if (!ax900_load_firmware_blob(AX900_FW_DIR AX900_FW_FMAC_U02, AX900_RAM_FMAC_FW_ADDR_U02)) {
        return false;
    }
    if (!ax900_load_firmware_blob(AX900_FW_DIR AX900_FW_LMAC_RF_U02, AX900_RAM_FMAC_RF_FW_ADDR_U02)) {
        return false;
    }
    if (!ax900_load_firmware_blob(AX900_FW_DIR AX900_FW_PATCH_U02, AX900_RAM_PATCH_BASE_ADDR_U02)) {
        return false;
    }

    ax900_dbg_mem_write_req_t reboot1 = { .mem_addr = AX900_REG_REBOOT_CTRL_ADDR,
                                          .mem_data = AX900_REG_REBOOT_CTRL_VALUE };
    ax900_dbg_mem_write_cfm_t cfm1 = {0};
    if (!ax900_send_msg(AX900_DBG_MEM_WRITE_REQ, AX900_DBG_MEM_WRITE_CFM,
                        &reboot1, sizeof(reboot1), &cfm1, sizeof(cfm1), 1000u)) {
        return false;
    }

    ax900_dbg_mem_write_req_t reboot2 = { .mem_addr = AX900_REG_REBOOT_TRIGGER_ADDR,
                                          .mem_data = AX900_REG_REBOOT_TRIGGER_VALUE };
    ax900_dbg_mem_write_cfm_t cfm2 = {0};
    if (!ax900_send_msg(AX900_DBG_MEM_WRITE_REQ, AX900_DBG_MEM_WRITE_CFM,
                        &reboot2, sizeof(reboot2), &cfm2, sizeof(cfm2), 1000u)) {
        return false;
    }

    return true;
}

/* No real MAC is available: the reference driver reads one out of the
 * chip's efuse via DBG_EF_USRDATA_READ_REQ, a message this research never
 * captured (see AX900_Protocol.h's DBG task -- only MEM_READ/WRITE are
 * reproduced). Rather than block station bring-up on that, this driver
 * generates a locally-administered address (the standard "no globally
 * unique address available" escape hatch, IEEE 802-2014 8.2.2: U/L bit
 * set, I/G bit clear) seeded from the chip id/sub-id this driver *does*
 * read, so it is at least stable across reboots of the same chip. TODO:
 * replace with a real efuse read once DBG_EF_USRDATA_READ_REQ's wire
 * layout is confirmed from source. */
static void ax900_generate_local_mac(uint8_t mac_out[6])
{
    mac_out[0] = 0x02u; /* locally administered, unicast */
    mac_out[1] = 0x41u; /* 'A' -- ax900 marker, arbitrary */
    mac_out[2] = 0x58u; /* 'X' */
    mac_out[3] = g_ax900.chip_id;
    mac_out[4] = g_ax900.chip_sub_id;
    mac_out[5] = (uint8_t)(g_api->timer_ticks() & 0xFFu);
}

/* Post-firmware-boot bring-up: MM_RESET_REQ (mirrors the reference
 * driver's rwnx_send_reset(), a bare message with no params) then
 * MM_ADD_IF_REQ to create the STA-mode virtual interface scan/connect
 * need. MM_START_REQ / MM_VERSION_REQ (also part of the reference
 * sequence) are deliberately skipped: MM_START_REQ's struct mm_start_req
 * carries an opaque struct phy_cfg_tag this research never captured, and
 * guessing its layout risks corrupting a real firmware's PHY bring-up
 * far worse than simply not sending it -- ADD_IF_REQ does not depend on
 * it having been sent. */
static bool ax900_bring_up(void)
{
    if (!ax900_send_msg(AX900_MM_RESET_REQ, AX900_MM_RESET_CFM,
                        NULL, 0u, NULL, 0u, 1000u)) {
        return false;
    }

    ax900_mm_add_if_req_t req;
    g_api->memset(&req, 0, sizeof(req));
    req.type = AX900_VIF_TYPE_STA;
    ax900_generate_local_mac(req.addr);
    ax900_mm_add_if_cfm_t cfm = {0};
    if (!ax900_send_msg(AX900_MM_ADD_IF_REQ, AX900_MM_ADD_IF_CFM,
                        &req, sizeof(req), &cfm, sizeof(cfm), 1000u)) {
        return false;
    }
    if (cfm.status != 0u) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    g_api->memcpy(g_ax900.mac, req.addr, 6u);
    g_ax900.vif_idx = cfm.inst_nbr;
    g_ax900.bringup_done = true;
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);
    return true;
}

/* ---- SCANU_RESULT_IND parsing: extract SSID + a rough open/secured
 * guess from the tagged IEs of one discovered AP. See
 * AX900_Protocol.h's ax900_scanu_result_ind_hdr_t comment for why the IE
 * search is defensive about where exactly the tag run starts. `payload`
 * points at the bytes following that 10-byte header; `payload_len` is
 * hdr.length clamped to what's actually available in the URB. */
static void ax900_parse_scan_payload(const ax900_scanu_result_ind_hdr_t *hdr,
                                     const uint8_t *payload, uint16_t payload_len)
{
    if (g_ax900.scan_result_count >= AX900_MAX_SCAN_RESULTS) {
        return; /* keep the first N seen this scan; good enough for a picker UI */
    }

    /* Beacon/probe-resp fixed fields (timestamp 8 + interval 2 + cap 2)
     * conventionally precede the IE run; try that offset first, and fall
     * back to offset 0 (already-stripped payload) if it doesn't look like
     * a valid tag/len run. */
    uint16_t offsets[2] = {12u, 0u};
    char ssid[DRIVER_WIFI_SSID_MAX + 1u];
    bool have_ssid = false;
    bool secured = false;

    for (uint32_t attempt = 0u; attempt < 2u && !have_ssid; ++attempt) {
        uint16_t pos = offsets[attempt];
        if (pos >= payload_len) {
            continue;
        }
        bool attempt_ssid = false;
        bool attempt_secured = false;
        bool attempt_valid = true;
        uint16_t p = pos;
        while (p + 2u <= payload_len) {
            uint8_t tag = payload[p];
            uint8_t len = payload[p + 1u];
            if ((uint32_t)p + 2u + len > payload_len) {
                attempt_valid = (p == pos) ? false : attempt_valid;
                break;
            }
            if (tag == AX900_IE_TAG_SSID && len <= DRIVER_WIFI_SSID_MAX) {
                g_api->memcpy(ssid, payload + p + 2u, len);
                ssid[len] = '\0';
                attempt_ssid = true;
            } else if (tag == AX900_IE_TAG_RSN) {
                attempt_secured = true;
            } else if (tag == AX900_IE_TAG_VENDOR && len >= 4u &&
                      payload[p + 2u] == 0x00u && payload[p + 3u] == 0x50u &&
                      payload[p + 4u] == 0xF2u && payload[p + 5u] == 0x01u) {
                attempt_secured = true; /* WPA1 OUI 00:50:F2, type 1 */
            }
            p = (uint16_t)(p + 2u + len);
        }
        if (attempt_valid && attempt_ssid) {
            have_ssid = true;
            secured = attempt_secured;
        }
    }

    if (!have_ssid || ssid[0] == '\0') {
        return; /* hidden/unparseable SSID -- skip rather than show a blank entry */
    }

    /* De-dupe by BSSID (repeated beacons for an AP already seen this
     * scan) -- refresh RSSI in place instead of growing the list. */
    for (uint32_t i = 0u; i < g_ax900.scan_result_count; ++i) {
        if (ax900_bytes_equal(g_ax900.scan_results[i].ssid, ssid, sizeof(ssid))) {
            g_ax900.scan_results[i].rssi_dbm = hdr->rssi;
            return;
        }
    }

    driver_wifi_scan_result_t *slot = &g_ax900.scan_results[g_ax900.scan_result_count];
    g_api->memset(slot, 0, sizeof(*slot));
    g_api->memcpy(slot->ssid, ssid, sizeof(ssid));
    slot->rssi_dbm = hdr->rssi;
    slot->security = secured ? DRIVER_WIFI_SECURITY_WPA_PSK : DRIVER_WIFI_SECURITY_OPEN;
    g_ax900.scan_result_count++;
}

/* driver_module_descriptor_t.probe -- called by BusRegistry.c once
 * g_ax900_bus_matches[] hits a vendor-specific AX900 interface USB_Main.c
 * enumerated. `dev->bus_context` is a `usb_device_context_t` (DriverBinary.h)
 * built on USB_Main.c's stack -- only valid for this call, so everything
 * needed from it is copied out before returning. */
static bool ax900_probe(const bus_device_t *dev)
{
    if (g_api == NULL || dev == NULL || dev->bus_context == NULL) {
        return false;
    }
    const usb_device_context_t *ctx = (const usb_device_context_t *)dev->bus_context;
    if (ctx->ep_in == 0u || ctx->ep_out == 0u || ctx->submit_bulk == NULL) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    g_api->memset(&g_ax900, 0, sizeof(g_ax900));
    g_ax900.attached = true;
    g_ax900.addr = ctx->addr;
    g_ax900.interface = ctx->interface;
    g_ax900.ep_in = ctx->ep_in;
    g_ax900.ep_out = ctx->ep_out;
    g_ax900.ep_in_mps = ctx->ep_in_mps;
    g_ax900.ep_out_mps = ctx->ep_out_mps;
    g_ax900.submit_bulk = ctx->submit_bulk;
    g_ax900.mtu = 1500u;
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);

    if (g_api->serial_write_string != NULL) {
        g_api->serial_write_string("[AX900] UGREEN AX900 (AIC8800D80) attached\n");
    }

    /* Best-effort chip-id read (a pre-firmware bootloader-level command in
     * the reference driver, matching its own bring-up order). Firmware
     * download is deliberately NOT attempted here: VFS may not be mounted
     * yet at USB-enumeration time (drivers are loaded before filesystems
     * in kernel_main.c's boot order). ax900_poll() retries it instead,
     * every poll, until it succeeds -- see ax900_download_firmware(). */
    (void)ax900_read_chip_id();
    return true;
}

/* driver_module_descriptor_t.remove */
static void ax900_remove(const bus_device_t *dev)
{
    if (dev == NULL || dev->bus_context == NULL) {
        return;
    }
    const usb_device_context_t *ctx = (const usb_device_context_t *)dev->bus_context;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    if (g_ax900.attached && g_ax900.addr == ctx->addr) {
        g_api->memset(&g_ax900, 0, sizeof(g_ax900));
    }
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);
}

/* driver_nic_t.init -- reflects whether a real AX900 is currently attached
 * (via ax900_probe() above), not any hardware bring-up of its own; that
 * already happened in probe(). TODO: nic_init() (Drivers/Module/NIC.c)
 * only tries each DEVICE_TYPE_NIC candidate once and never automatically
 * retries later, so this driver only becomes the active NIC if it happens
 * to already be attached the first time nic_init() runs -- a pre-existing
 * limitation of that selection logic, not something new here. */
static bool ax900_nic_init(void)
{
    return g_ax900.attached;
}

void ax900_poll(void)
{
    if (!g_ax900.attached || g_ax900.ep_in == 0u || g_ax900.submit_bulk == NULL) {
        return;
    }

    /* Retried every poll while not yet loaded: cheap when the firmware
     * files simply aren't present (find_file() fails immediately, see
     * ax900_load_firmware_blob()), bounded even if they are present but
     * the chip never responds (the write loop bails on the first failed
     * DBG_MEM_WRITE_REQ). Stops being attempted forever once it succeeds. */
    if (!g_ax900.fw_loaded) {
        g_ax900.fw_loaded = ax900_download_firmware();
    }

    /* Same retry-every-poll shape as firmware download above, and for the
     * same reason: cheap (one RESET_REQ + one ADD_IF_REQ) when it fails,
     * stops being attempted once bringup_done is true. Split from the
     * fw_loaded branch above because the reboot triggered by
     * ax900_download_firmware() needs real wall-clock time to take effect
     * before the chip will answer a fresh MM_RESET_REQ -- retrying here on
     * a later poll rather than immediately after reboot gives it that
     * window for free. */
    if (g_ax900.fw_loaded && !g_ax900.bringup_done) {
        (void)ax900_bring_up();
    }

    if (g_ax900.scanning) {
        uint64_t elapsed_ticks = g_api->timer_ticks() - g_ax900.scan_started_ticks;
        uint32_t hz = g_api->timer_hz();
        if (hz != 0u && elapsed_ticks * 1000u / hz >= 3000u) {
            g_ax900.scanning = false; /* no explicit "scan complete" message in this
                                       * trimmed protocol -- see AX900_Protocol.h's
                                       * SCANU task comment. Bounded window instead. */
        }
    }

    uint8_t rx_buf[AX900_DATA_BUF_MAX];
    if (!g_ax900.submit_bulk(g_ax900.addr, g_ax900.ep_in, g_ax900.ep_in_mps, 1u, rx_buf, sizeof(rx_buf))) {
        return; /* nothing available / transfer failed -- normal while idle */
    }

    /* TODO(hw): ImplusOS's usb_submit_bulk() has no actual-length-
     * transferred out-parameter, so this cannot yet tell how many of
     * sizeof(rx_buf) bytes are genuinely new vs. stale from a previous
     * poll. The sub-packet walk below trusts each embedded length field
     * exactly as the reference driver's aicwf_process_rxframes() does, but
     * (unlike that driver, which gets a real URB actual_length from the
     * USB core) has no reliable way to know where the whole transfer
     * legitimately ends -- needs usb_submit_bulk() extended with an
     * actual-length out-param before this is safe against real hardware. */

    uint32_t pos = 0u;
    uint32_t processed = 0u;
    while (pos + AX900_USB_HDR_LEN <= sizeof(rx_buf) && processed < 32u) {
        uint16_t pkt_len = (uint16_t)(rx_buf[pos] | ((uint16_t)rx_buf[pos + 1u] << 8));
        uint8_t  type    = rx_buf[pos + 2u];

        if (pkt_len == 0u) {
            break; /* no more aggregated sub-packets in this URB */
        }

        uint32_t chunk;
        if ((type & AX900_PKT_TYPE_CFG) == 0u) {
            /* DATA packet: HW RX status header, then the frame itself.
             * FullMAC: firmware already hands up an 802.3-shaped frame
             * (the reference driver's ieee80211_amsdu_to_8023s() runs
             * against firmware-delivered A-MSDUs, not raw 802.11 -- unlike
             * the generic SoftMAC foundation in Server/NIC/WiFi/), so no
             * 802.11 decapsulation belongs here. */
            chunk = AX900_USB_HDR_LEN + AX900_RX_HWHDR_LEN + pkt_len;
            if (pos + chunk > sizeof(rx_buf)) {
                break;
            }

            if (g_ax900.associated && g_ax900.rx_callback != NULL) {
                const uint8_t *eth_frame = rx_buf + pos + AX900_USB_HDR_LEN + AX900_RX_HWHDR_LEN;
                g_ax900.rx_callback(eth_frame, pkt_len);
            }
        } else {
            /* CFG packet: 4-byte header, then an ax900_msg_hdr_t + params. */
            chunk = AX900_USB_HDR_LEN + pkt_len;
            if (pos + chunk > sizeof(rx_buf)) {
                break;
            }

            if ((type & AX900_PKT_TYPE_MASK) == AX900_PKT_TYPE_CFG_CMD_RSP) {
                uint64_t irq_flags = irq_save_disable();
                spinlock_lock(&g_ax900.lock);
                ax900_handle_cfg_rsp_locked(rx_buf + pos + AX900_USB_HDR_LEN, pkt_len);
                spinlock_unlock(&g_ax900.lock);
                irq_restore(irq_flags);
            }
            /* AX900_PKT_TYPE_CFG_DATA_CFM (tx completion) intentionally
             * not handled: ax900_send() has no in-flight tx queue yet for
             * it to update -- see ax900_send(). */
        }

        uint32_t aligned = (chunk + (AX900_ALIGN - 1u)) & ~(AX900_ALIGN - 1u);
        pos += aligned;
        processed++;
    }
}

/* ---- public: NIC-shaped surface (not yet wired into NicManager, see AX900.h) ---- */

bool ax900_is_ready(void)
{
    return g_ax900.associated;
}

uint16_t ax900_mtu(void)
{
    return g_ax900.mtu;
}

void ax900_get_mac(uint8_t mac_out[6])
{
    if (mac_out == NULL) {
        return;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    g_api->memcpy(mac_out, g_ax900.mac, 6u);
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);
}

bool ax900_send(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len == 0u || frame_len > (uint16_t)(g_ax900.mtu + 14u)) {
        return false;
    }
    if (!g_ax900.fw_loaded || !g_ax900.associated) {
        return false; /* honest: fw_loaded is never true today, see ax900_download_firmware() */
    }
    /* TODO: no in-flight TX queue/host-id tracking yet to correlate a
     * later CFG_DATA_CFM against this specific frame (see ax900_poll()) --
     * fine for a fire-and-forget foundation, needed for real flow control. */
    return ax900_usb_send_packet(AX900_PKT_TYPE_DATA, frame, frame_len);
}

void ax900_set_rx_callback(ax900_rx_callback_t cb)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    g_ax900.rx_callback = cb;
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);
}

/* ---- public: Wi-Fi management plane ---- */

/* 2.4GHz channels 1-14, center frequencies per IEEE 802.11-2020 Table
 * 19-6 (channel 14 uses the Japan-only 2484MHz spacing). 5/6GHz scanning
 * is out of scope until AX900_SCAN_CHANNEL_MAX grows to cover it -- see
 * AX900_Protocol.h. `band` is left 0 (unconfirmed encoding, see
 * ax900_chan_def_t) since nothing here reads it back. */
static const uint16_t ax900_2g4_channel_freq_mhz[14] = {
    2412u, 2417u, 2422u, 2427u, 2432u, 2437u, 2442u,
    2447u, 2452u, 2457u, 2462u, 2467u, 2472u, 2484u,
};

bool ax900_scan_start(void)
{
    if (!g_ax900.attached || !g_ax900.bringup_done) {
        return false;
    }

    ax900_scanu_start_req_t req;
    g_api->memset(&req, 0, sizeof(req));
    for (uint32_t i = 0u; i < AX900_SCAN_CHANNEL_MAX; ++i) {
        req.chan[i].freq_mhz = ax900_2g4_channel_freq_mhz[i];
    }
    req.vif_idx = g_ax900.vif_idx;
    req.chan_cnt = AX900_SCAN_CHANNEL_MAX;
    req.ssid_cnt = 0u; /* wildcard scan -- list every AP, not just one SSID */
    req.duration_us = 100000u; /* 100ms/channel: a conventional FullMAC default, unconfirmed against this firmware */

    ax900_scanu_start_cfm_t cfm = {0};
    bool sent = ax900_send_msg(AX900_SCANU_START_REQ, AX900_SCANU_START_CFM,
                               &req, sizeof(req), &cfm, sizeof(cfm), 2000u);
    if (sent) {
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_ax900.lock);
        g_ax900.scan_result_count = 0u;
        g_ax900.scanning = true;
        g_ax900.scan_started_ticks = g_api->timer_ticks();
        spinlock_unlock(&g_ax900.lock);
        irq_restore(irq_flags);
    }
    return sent;
}

/* Appends a standard WPA2-PSK/CCMP RSN IE (tag 48) to `ie_buf`, then --
 * best-effort, see AX900_Protocol.h's "WPA2-PSK key material handoff"
 * comment -- the raw ASCII passphrase as a private vendor IE (tag 221).
 * Returns the total bytes written, or 0 if it doesn't fit. `ie_buf` is
 * ax900_sm_connect_req_t.ie_buf, AX900_CMD_BUF_MAX bytes of headroom in
 * the outer USB packet notwithstanding -- the reference struct sizes it
 * at 64 u32 words (256 bytes), which comfortably fits both IEs. */
static uint16_t ax900_build_wpa2_ies(uint8_t *ie_buf, uint16_t ie_buf_cap,
                                     const char *psk, size_t psk_len)
{
    uint16_t off = 0u;
    if ((uint32_t)off + AX900_RSN_IE_TOTAL_LEN > ie_buf_cap) {
        return 0u;
    }
    ie_buf[off++] = AX900_IE_TAG_RSN;
    ie_buf[off++] = (uint8_t)AX900_RSN_IE_BODY_LEN;
    ie_buf[off++] = 0x01u; ie_buf[off++] = 0x00u; /* RSN version 1 */
    /* group cipher suite: 00-0F-AC-04 (CCMP) */
    ie_buf[off++] = 0x00u; ie_buf[off++] = 0x0Fu; ie_buf[off++] = 0xACu; ie_buf[off++] = 0x04u;
    ie_buf[off++] = 0x01u; ie_buf[off++] = 0x00u; /* pairwise cipher count = 1 */
    ie_buf[off++] = 0x00u; ie_buf[off++] = 0x0Fu; ie_buf[off++] = 0xACu; ie_buf[off++] = 0x04u; /* CCMP */
    ie_buf[off++] = 0x01u; ie_buf[off++] = 0x00u; /* AKM count = 1 */
    ie_buf[off++] = 0x00u; ie_buf[off++] = 0x0Fu; ie_buf[off++] = 0xACu; ie_buf[off++] = 0x02u; /* PSK */
    ie_buf[off++] = 0x00u; ie_buf[off++] = 0x00u; /* RSN capabilities = 0 */

    uint16_t psk_ie_len = (uint16_t)(4u + psk_len); /* OUI(3) + type(1) + passphrase */
    if ((uint32_t)off + 2u + psk_ie_len > ie_buf_cap || psk_len > 63u) {
        return off; /* RSN IE alone is still valid to send */
    }
    ie_buf[off++] = AX900_IE_TAG_VENDOR;
    ie_buf[off++] = (uint8_t)psk_ie_len;
    ie_buf[off++] = AX900_PRIV_PSK_VENDOR_OUI_0;
    ie_buf[off++] = AX900_PRIV_PSK_VENDOR_OUI_1;
    ie_buf[off++] = AX900_PRIV_PSK_VENDOR_OUI_2;
    ie_buf[off++] = AX900_PRIV_PSK_VENDOR_TYPE;
    g_api->memcpy(ie_buf + off, psk, psk_len);
    off = (uint16_t)(off + psk_len);
    return off;
}

bool ax900_connect(const char *ssid, const char *psk)
{
    if (!g_ax900.attached || !g_ax900.bringup_done || ssid == NULL) {
        return false;
    }

    size_t ssid_len = ax900_strnlen(ssid, 32u + 1u);
    if (ssid_len == 0u || ssid_len > 32u) {
        return false;
    }
    size_t psk_len = 0u;
    if (psk != NULL) {
        psk_len = ax900_strnlen(psk, 63u + 1u);
        if (psk_len < 8u || psk_len > 63u) {
            return false; /* WPA2-PSK passphrases are 8-63 ASCII chars, IEEE 802.11-2020 */
        }
    }

    ax900_sm_connect_req_t req;
    g_api->memset(&req, 0, sizeof(req));
    req.ssid.length = (uint8_t)ssid_len;
    g_api->memcpy(req.ssid.ssid, ssid, ssid_len);
    req.chan.freq_mhz = 0u; /* 0: let firmware pick the channel from its scan cache */
    req.auth_type = AX900_SM_AUTH_TYPE_AUTO; /* let the IEs below (or their absence) say open vs WPA2 */
    req.vif_idx = g_ax900.vif_idx;
    if (psk != NULL) {
        req.ie_len = ax900_build_wpa2_ies((uint8_t *)req.ie_buf, (uint16_t)sizeof(req.ie_buf),
                                          psk, psk_len);
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    size_t copy_len = ssid_len < DRIVER_WIFI_SSID_MAX ? ssid_len : DRIVER_WIFI_SSID_MAX;
    g_api->memcpy(g_ax900.target_ssid, ssid, copy_len);
    g_ax900.target_ssid[copy_len] = '\0';
    g_ax900.connect_failed = false;
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);

    ax900_sm_connect_cfm_t cfm = {0};
    bool sent = ax900_send_msg(AX900_SM_CONNECT_REQ, AX900_SM_CONNECT_CFM,
                               &req, sizeof(req), &cfm, sizeof(cfm), 5000u);
    bool started = sent && cfm.status == 0u;
    if (!started) {
        g_ax900.connect_failed = true;
    }
    return started;
}

void ax900_disconnect(void)
{
    if (!g_ax900.attached) {
        return;
    }

    (void)ax900_send_msg(AX900_SM_DISCONNECT_REQ, AX900_SM_DISCONNECT_CFM,
                         NULL, 0u, NULL, 0u, 1000u);

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    g_ax900.associated = false;
    g_ax900.connect_failed = false;
    g_ax900.target_ssid[0] = '\0';
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);
}

uint32_t ax900_get_scan_results(driver_wifi_scan_result_t *out, uint32_t max_count)
{
    if (out == NULL || max_count == 0u) {
        return 0u;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    uint32_t count = g_ax900.scan_result_count;
    if (count > max_count) {
        count = max_count;
    }
    if (count > AX900_MAX_SCAN_RESULTS) {
        count = AX900_MAX_SCAN_RESULTS; /* defensive; scan_result_count is never allowed past this */
    }
    for (uint32_t i = 0u; i < count; ++i) {
        out[i] = g_ax900.scan_results[i];
    }
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);
    return count;
}

void ax900_get_wifi_status(driver_wifi_status_t *out_status)
{
    if (out_status == NULL) {
        return;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ax900.lock);
    driver_wifi_state_t state;
    if (!g_ax900.attached) {
        state = DRIVER_WIFI_STATE_NO_ADAPTER;
    } else if (!g_ax900.fw_loaded) {
        state = DRIVER_WIFI_STATE_FIRMWARE_LOADING; /* also covers "files missing" -- see
                                                      * ax900_load_firmware_blob(), it retries
                                                      * forever rather than reporting FAILED */
    } else if (!g_ax900.bringup_done) {
        state = DRIVER_WIFI_STATE_FIRMWARE_LOADING;
    } else if (g_ax900.associated) {
        state = DRIVER_WIFI_STATE_ASSOCIATED;
    } else if (g_ax900.connect_failed) {
        state = DRIVER_WIFI_STATE_CONNECT_FAILED;
    } else if (g_ax900.target_ssid[0] != '\0') {
        state = DRIVER_WIFI_STATE_CONNECTING;
    } else if (g_ax900.scanning) {
        state = DRIVER_WIFI_STATE_SCANNING;
    } else {
        state = DRIVER_WIFI_STATE_READY;
    }
    out_status->state = state;
    g_api->memcpy(out_status->ssid, g_ax900.target_ssid, sizeof(out_status->ssid));
    g_api->memcpy(out_status->mac, g_ax900.mac, 6u);
    spinlock_unlock(&g_ax900.lock);
    irq_restore(irq_flags);
}

/* ---- standalone driver module wiring ---- */

/* Matches the vendor-specific USB interface AX900_Protocol.h and USB_Main.c
 * agree on (class/subclass/protocol == 0xFF, either AICSemi USB VID). Two
 * entries because match_flags can't OR two vendor ids together. */
static const driver_bus_match_t g_ax900_bus_matches[] = {
    {
        .bus_type = DEVICE_TYPE_USB,
        .vendor_id = (uint16_t)AX900_USB_VENDOR_ID_AIC,
        .class_code = AX900_USB_IFACE_CLASS_VENDOR,
        .subclass = AX900_USB_IFACE_SUBCLASS_VENDOR,
        .protocol = AX900_USB_IFACE_PROTOCOL_VENDOR,
        .match_flags = DRIVER_BUS_MATCH_VENDOR | DRIVER_BUS_MATCH_CLASS |
                       DRIVER_BUS_MATCH_SUBCLASS | DRIVER_BUS_MATCH_PROTOCOL,
    },
    {
        .bus_type = DEVICE_TYPE_USB,
        .vendor_id = (uint16_t)AX900_USB_VENDOR_ID_AIC_V2,
        .class_code = AX900_USB_IFACE_CLASS_VENDOR,
        .subclass = AX900_USB_IFACE_SUBCLASS_VENDOR,
        .protocol = AX900_USB_IFACE_PROTOCOL_VENDOR,
        .match_flags = DRIVER_BUS_MATCH_VENDOR | DRIVER_BUS_MATCH_CLASS |
                       DRIVER_BUS_MATCH_SUBCLASS | DRIVER_BUS_MATCH_PROTOCOL,
    },
};

static const driver_nic_t g_ax900_nic_driver = {
    .init = ax900_nic_init,
    .is_ready = ax900_is_ready,
    .mtu = ax900_mtu,
    .get_mac = ax900_get_mac,
    .send_frame = ax900_send,
    .poll = ax900_poll,
    .set_rx_callback = ax900_set_rx_callback,

    .wifi_scan_start = ax900_scan_start,
    .wifi_get_scan_results = ax900_get_scan_results,
    .wifi_connect = ax900_connect,
    .wifi_disconnect = ax900_disconnect,
    .wifi_get_status = ax900_get_wifi_status,
};

static void ax900_shutdown(void)
{
    if (g_api != NULL) {
        g_api->memset(&g_ax900, 0, sizeof(g_ax900));
    }
    g_api = NULL;
}

static const driver_module_descriptor_t g_ax900_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_NIC,
    .load_priority = 55u,
    .deps = { "USB_Driver.ELF", NULL },
    .driver_api = &g_ax900_nic_driver,
    .shutdown = ax900_shutdown,
    .bus_matches = g_ax900_bus_matches,
    .bus_match_count = (uint32_t)(sizeof(g_ax900_bus_matches) / sizeof(g_ax900_bus_matches[0])),
    .probe = ax900_probe,
    .remove = ax900_remove,
};

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL ||
        api->memset == NULL ||
        api->memcpy == NULL ||
        api->serial_write_string == NULL ||
        api->mem.malloc == NULL ||
        api->mem.free == NULL ||
        api->fs.find_file == NULL ||
        api->fs.read_at == NULL ||
        api->fs.get_file_size == NULL ||
        api->fs.close_file == NULL ||
        api->event.create == NULL ||
        api->event.destroy == NULL ||
        api->event.signal == NULL ||
        api->event.wait == NULL ||
        api->hal.cpu_save_interrupts == NULL ||
        api->hal.cpu_restore_interrupts == NULL ||
        api->hal.cpu_pause == NULL) {
        return NULL;
    }

    g_api = api;
    ax900_init();
    return &g_ax900_module;
}
