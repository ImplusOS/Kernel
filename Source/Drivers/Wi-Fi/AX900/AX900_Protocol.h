#pragma once

/*
 * AX900_Protocol.h -- wire protocol for the UGREEN AX900 USB Wi-Fi 6 adapter.
 *
 * The AX900 (UGREEN model CM762 / CM845) uses AICSemi's AIC8800D80 chipset
 * (USB VID 0xA69C, "AIC Wlan"): a RivieraWaves "nX MAC" based, FullMAC
 * 802.11ax Wi-Fi+BT combo part. It has no official Linux support and no
 * public datasheet -- UGREEN ships Windows-only drivers. Every constant in
 * this file is taken from AICSemi's own out-of-tree "aic8800" Linux driver
 * (as packaged for Radxa boards) and cross-checked against community
 * reports of specifically the AX900:
 *
 *   - https://github.com/radxa-pkg/aic8800
 *     (src/USB/driver_fw/drivers/aic8800/aic_load_fw/{aicwf_usb.c,aicwf_usb.h,
 *      aic_compat_8800d80.h,aic_txrxif.c} and
 *      aic8800_fdrv/{aicwf_usb.c,usb_host.c,hal_desc.h,ipc_shared.h,lmac_msg.h})
 *   - https://github.com/saarp/CM762_Driver          (AX900-specific packaging)
 *   - https://github.com/shenmintao/aic8800d80/issues/11  (AX900 firmware names, chip/mcu id)
 *   - https://blog.nero.gay/ugreen/ax900/            (AX900 bring-up notes, kernel 6.4+ patch)
 *   - Arch Linux / Linux Mint forum threads reporting the AX900 as "a69c:8d80 AIC Wlan"
 *
 * "FullMAC" means the firmware handles 802.11 timing, retries, and the
 * WPA2/WPA3 handshake itself; the host only exchanges high-level commands
 * (scan, connect-with-SSID, ...) and already-802.3-shaped data frames. This
 * is a different division of labour than the generic 802.11 SoftMAC
 * foundation in Kernel/Drivers/NIC/WiFi/ -- the two drivers are
 * intentionally independent (see AX900.c for why this one lives under
 * Server/USB/ instead).
 *
 * Fields/values marked "unconfirmed" below are inferred from the message
 * struct layout rather than read directly off an AX900 unit; verify them
 * against the reference driver before relying on them for real bring-up.
 */

#include <stdint.h>

/* ---- USB identification ----
 * Vendor-specific interface: bInterfaceClass/SubClass/Protocol = 0xFF (the
 * AX900 exposes no HID/mass-storage/CDC personality at all). Confirmed by
 * aicwf_parse_usb() in aic8800_fdrv/aicwf_usb.c.
 */
#define AX900_USB_VENDOR_ID_AIC      0xA69Cu  /* AICSemi */
#define AX900_USB_VENDOR_ID_AIC_V2   0x368Bu  /* newer AICSemi USB VID, some SKUs */

#define AX900_USB_IFACE_CLASS_VENDOR    0xFFu
#define AX900_USB_IFACE_SUBCLASS_VENDOR 0xFFu
#define AX900_USB_IFACE_PROTOCOL_VENDOR 0xFFu

/* Product IDs seen for the AIC8800D80 family. AX900 units in the wild
 * enumerate directly as 0xA69C:0x8D80 ("AIC Wlan"). The 0x1111:0x1111 ->
 * 0xA69C:0x8D80 "bootloader stage" transition (triggered by a vendor
 * control sequence informally described as "send F3, then F2") is
 * documented for a related AIC8800-family board (Pandora 88M80), not
 * confirmed on the AX900 itself -- kept here as a known family behaviour,
 * not an AX900-specific fact. */
#define AX900_PID_D80_WLAN        0x8D80u /* AX900, WLAN function (confirmed) */
#define AX900_PID_D80_BT          0x8D81u /* same silicon, Bluetooth function */
#define AX900_PID_D80_ALT         0x8D83u /* seen on some AIC8800D80 units/newer fw */
#define AX900_PID_PRE_FW_BOOT     0x1111u /* generic pre-firmware "unprogrammed" ID, unconfirmed on AX900 */

/* ---- Chip revision (read from a boot-time register, see AX900_REG_* below) ---- */
#define AX900_CHIP_REV_U01      0x1u
#define AX900_CHIP_REV_U02      0x3u  /* AX900 units reported working use the "_u02" firmware set */
#define AX900_CHIP_REV_U03      0x7u
#define AX900_CHIP_REV_U04      0xFu
#define AX900_CHIP_REV_U05      0x1Fu
#define AX900_CHIP_SUB_REV_U04  0x20u

/* Registers read during early bring-up, before firmware is even loaded, to
 * identify the chip revision and pick the matching firmware file set.
 * c.f. system_config_8800() in aic_load_fw/aicwf_usb.c. */
#define AX900_REG_CHIP_ID_ADDR     0x40500000u /* chip_id     = (value >> 16) & 0xFF */
#define AX900_REG_CHIP_SUBID_ADDR  0x00000004u /* chip_sub_id = (value >> 4)  & 0xFF */

/* Register writes that trigger a full chip reboot once firmware download
 * is complete. c.f. sys_reboot_tbl[] in aic_load_fw/aic_compat_8800d80.c. */
#define AX900_REG_REBOOT_CTRL_ADDR   0x50017000u
#define AX900_REG_REBOOT_CTRL_VALUE  0x0001FFFFu
#define AX900_REG_REBOOT_TRIGGER_ADDR  0x50017008u
#define AX900_REG_REBOOT_TRIGGER_VALUE 0x00000002u

/* ---- Firmware file set (chip revision U02 -- matches confirmed-working
 * AX900 units, c.f. shenmintao/aic8800d80 issue #11 and
 * aic_load_fw/aic_compat_8800d80.h). ImplusOS driver modules have no VFS/
 * file-read primitive today (see AX900.c), so these names are recorded for
 * when that primitive exists, not yet used to actually load anything. */
#define AX900_FW_PATCH_TABLE_U02  "fw_patch_table_8800d80_u02.bin"
#define AX900_FW_FMAC_U02         "fmacfw_8800d80_u02.bin"
#define AX900_FW_LMAC_RF_U02      "lmacfw_rf_8800d80_u02.bin"
#define AX900_FW_PATCH_U02        "fw_patch_8800d80_u02.bin"
#define AX900_FW_ADID_U02         "fw_adid_8800d80_u02.bin"
#define AX900_FW_USERCONFIG       "aic_userconfig_8800d80.txt"

/* RAM addresses firmware segments are downloaded to (chip revision U02). */
#define AX900_RAM_FMAC_FW_ADDR_U02     0x120000u
#define AX900_RAM_FMAC_RF_FW_ADDR_U02  0x120000u
#define AX900_RAM_ADID_BASE_ADDR_U02   0x00201940u
#define AX900_RAM_PATCH_BASE_ADDR_U02  0x0020B43Cu

/* ---- USB packet framing ----
 * Every logical sub-packet moved over a bulk pipe (data or "msg"/control)
 * starts with this 4-byte header; several sub-packets can be aggregated
 * into one URB, each one padded up to AX900_ALIGN before the next header.
 * c.f. aicwf_process_rxframes() in aic_load_fw/aic_txrxif.c (RX side) and
 * aicwf_usb_bus_txmsg() in aic8800_fdrv/aicwf_usb.c (TX side).
 *
 *   offset 0..1 : payload length, u16 little-endian
 *                   - DATA packets : length of the 802.11/802.3 payload
 *                     that follows the AX900_RX_HWHDR_LEN-byte HW header
 *                   - CFG packets  : length of the message body that
 *                     follows this 4-byte header (no separate HW header)
 *   offset 2    : type byte (AX900_PKT_TYPE_*, low 7 bits significant)
 *   offset 3    : reserved / padding
 */
#define AX900_USB_HDR_LEN   4u

#define AX900_PKT_TYPE_DATA         0x00u
#define AX900_PKT_TYPE_CFG          0x10u /* bit flag: set on every control-channel packet */
#define AX900_PKT_TYPE_CFG_CMD_RSP  0x11u /* device -> host: synchronous response to a CFG request */
#define AX900_PKT_TYPE_CFG_DATA_CFM 0x12u /* device -> host: TX completion confirmation */
#define AX900_PKT_TYPE_MASK         0x7Fu /* bit 7 is reserved; mask it off before comparing */

#define AX900_RX_HWHDR_LEN     60u   /* HW RX status header prepended to DATA packets (internal layout not published) */
#define AX900_ALIGN             4u   /* both TX and RX sub-packets are padded to this */
#define AX900_CMD_BUF_MAX    1536u   /* max size of one CFG message buffer */
#define AX900_DATA_BUF_MAX   2048u   /* max size of one bulk data buffer (AICWF_USB_MAX_PKT_SIZE) */
#define AX900_TXPKT_BLOCKSIZE 512u

/* ---- Host <-> firmware message header ----
 * Every CFG/CFG_CMD_RSP payload (i.e. what follows the 4-byte USB header
 * above) is an ipc_e2a_msg-shaped record: a fixed 12-byte header followed
 * by a parameter block. c.f. struct ipc_e2a_msg in aic8800_fdrv/ipc_shared.h.
 * dest_id/src_id are "dummy" fields on this transport (only meaningful on
 * the SDIO/PCIe IPC ring variant of this driver) -- kept here for layout
 * fidelity, always 0 over USB.
 */
typedef struct __attribute__((packed)) {
    uint16_t id;          /* message id -- see the AX900_TASK_* / AX900_xxx_REQ defines below */
    uint16_t dest_id;     /* dummy on USB transport */
    uint16_t src_id;      /* dummy on USB transport */
    uint16_t param_len;   /* length in bytes of the parameter block that follows */
    uint32_t pattern;     /* AX900_MSG_VALID_PATTERN stamps a genuine message buffer */
} ax900_msg_hdr_t;

/* c.f. #define IPC_MSGE2A_VALID_PATTERN in aic8800_fdrv/ipc_shared.h */
#define AX900_MSG_VALID_PATTERN  0xADDEDE2Au

/* ---- Task / message id scheme ----
 * c.f. lmac_msg.h: task ids are assigned 0, 1, 2, ... in declaration
 * order, and each task's first message id is (task_id << 10); message ids
 * within a task increment by 1 from there (LMAC_FIRST_MSG/MSG_T/MSG_I).
 */
#define AX900_TASK_MM     0u /* MAC management: reset, interface add/remove, keys, ... */
#define AX900_TASK_DBG    1u /* debug: raw register/memory read-write (used pre-firmware too) */
#define AX900_TASK_SCAN   2u
#define AX900_TASK_TDLS   3u
#define AX900_TASK_SCANU  4u /* "scan unified" -- the FullMAC scan API actually used */
#define AX900_TASK_ME     5u
#define AX900_TASK_SM     6u /* station mode: connect/disconnect */
#define AX900_TASK_APM    7u /* AP mode */

#define AX900_LMAC_FIRST_MSG(task)  ((uint16_t)((task) << 10))
#define AX900_MSG_TASK(msg_id)      ((uint16_t)((msg_id) >> 10))
#define AX900_MSG_INDEX(msg_id)     ((uint16_t)((msg_id) & 0x3FFu))

/* MM task (bring-up: reset, start, interface add). */
#define AX900_MM_RESET_REQ    AX900_LMAC_FIRST_MSG(AX900_TASK_MM)      /* + 0 */
#define AX900_MM_RESET_CFM    (AX900_MM_RESET_REQ + 1u)
#define AX900_MM_START_REQ    (AX900_MM_RESET_REQ + 2u)
#define AX900_MM_START_CFM    (AX900_MM_RESET_REQ + 3u)
#define AX900_MM_VERSION_REQ  (AX900_MM_RESET_REQ + 4u)
#define AX900_MM_VERSION_CFM  (AX900_MM_RESET_REQ + 5u)
#define AX900_MM_ADD_IF_REQ   (AX900_MM_RESET_REQ + 6u)
#define AX900_MM_ADD_IF_CFM   (AX900_MM_RESET_REQ + 7u)

/* DBG task (raw register access, used before firmware is even running). */
#define AX900_DBG_MEM_READ_REQ   AX900_LMAC_FIRST_MSG(AX900_TASK_DBG)  /* + 0 */
#define AX900_DBG_MEM_READ_CFM   (AX900_DBG_MEM_READ_REQ + 1u)
#define AX900_DBG_MEM_WRITE_REQ  (AX900_DBG_MEM_READ_REQ + 2u)
#define AX900_DBG_MEM_WRITE_CFM  (AX900_DBG_MEM_READ_REQ + 3u)

/* SCANU task (the scan API the FullMAC firmware actually implements). */
#define AX900_SCANU_START_REQ    AX900_LMAC_FIRST_MSG(AX900_TASK_SCANU) /* + 0 */
#define AX900_SCANU_START_CFM    (AX900_SCANU_START_REQ + 1u)
#define AX900_SCANU_JOIN_REQ     (AX900_SCANU_START_REQ + 2u)
#define AX900_SCANU_JOIN_CFM     (AX900_SCANU_START_REQ + 3u)
#define AX900_SCANU_RESULT_IND   (AX900_SCANU_START_REQ + 4u)

/* SM task (station connect/disconnect). */
#define AX900_SM_CONNECT_REQ     AX900_LMAC_FIRST_MSG(AX900_TASK_SM)   /* + 0 */
#define AX900_SM_CONNECT_CFM     (AX900_SM_CONNECT_REQ + 1u)
#define AX900_SM_CONNECT_IND     (AX900_SM_CONNECT_REQ + 2u)
#define AX900_SM_DISCONNECT_REQ  (AX900_SM_CONNECT_REQ + 3u)
#define AX900_SM_DISCONNECT_CFM  (AX900_SM_CONNECT_REQ + 4u)
#define AX900_SM_DISCONNECT_IND  (AX900_SM_CONNECT_REQ + 5u)

/* ---- Selected message parameter structs (project-native fixed-width
 * types; the reference driver's u8_l/u16_l/u32_l are plain little-endian
 * wrappers of the same widths). Only the bring-up/scan/connect subset
 * needed by AX900.c is reproduced -- see lmac_msg.h for the full set. */

typedef struct __attribute__((packed)) {
    uint32_t mem_addr;
} ax900_dbg_mem_read_req_t;

typedef struct __attribute__((packed)) {
    uint32_t mem_addr;
    uint32_t mem_data;
} ax900_dbg_mem_read_cfm_t;

typedef struct __attribute__((packed)) {
    uint32_t mem_addr;
    uint32_t mem_data;
} ax900_dbg_mem_write_req_t;

/* c.f. struct dbg_mem_write_cfm -- same layout as the request (echoes back
 * what was written). */
typedef struct __attribute__((packed)) {
    uint32_t mem_addr;
    uint32_t mem_data;
} ax900_dbg_mem_write_cfm_t;

/* c.f. struct mm_version_cfm -- only the fields AX900.c actually reads. */
typedef struct __attribute__((packed)) {
    uint32_t version_lmac;
    uint32_t version_machw_1;
    uint32_t version_machw_2;
    uint32_t version_phy_1;
    uint32_t version_phy_2;
} ax900_mm_version_cfm_t;

/* c.f. struct mm_add_if_req. `type` -- enum mac_vif_type from lmac_mac.h:
 * { VIF_STA = 0, VIF_IBSS, VIF_AP, VIF_MESH_POINT, VIF_MONITOR, VIF_UNKNOWN }.
 * AX900_VIF_TYPE_STA below is confirmed against that enum (declaration
 * order, not read off real hardware). */
#define AX900_VIF_TYPE_STA 0u

typedef struct __attribute__((packed)) {
    uint8_t  type;      /* AX900_VIF_TYPE_STA */
    uint8_t  addr[6];   /* MAC address to bring the interface up with */
    uint8_t  p2p;        /* bool: P2P interface */
} ax900_mm_add_if_req_t;

typedef struct __attribute__((packed)) {
    uint8_t status;      /* 0 == success */
    uint8_t inst_nbr;    /* interface index assigned by firmware */
} ax900_mm_add_if_cfm_t;

#define AX900_SCAN_CHANNEL_MAX 14u /* 2.4GHz channels 1-14; 5/6GHz scanning needs a larger table -- TODO */
#define AX900_SCAN_SSID_MAX     1u

typedef struct __attribute__((packed)) {
    uint16_t freq_mhz;   /* channel centre frequency */
    uint8_t  band;        /* unconfirmed encoding, see mac_chan_def in lmac_mac.h */
    uint8_t  flags;
} ax900_chan_def_t;

typedef struct __attribute__((packed)) {
    uint8_t length;
    uint8_t ssid[32];
} ax900_ssid_t;

/* c.f. struct scanu_start_req, trimmed to what AX900.c populates. */
typedef struct __attribute__((packed)) {
    ax900_chan_def_t chan[AX900_SCAN_CHANNEL_MAX];
    ax900_ssid_t     ssid[AX900_SCAN_SSID_MAX];
    uint8_t          bssid[6];
    uint32_t         add_ies;      /* host pointer to extra probe-req IEs -- unused (0) */
    uint16_t         add_ie_len;
    uint8_t          vif_idx;
    uint8_t          chan_cnt;
    uint8_t          ssid_cnt;
    uint8_t          no_cck;
    uint32_t         duration_us;
} ax900_scanu_start_req_t;

typedef struct __attribute__((packed)) {
    uint8_t vif_idx;
    uint8_t status;
    uint8_t result_cnt;
} ax900_scanu_start_cfm_t;

#define AX900_SM_AUTH_TYPE_OPEN      0u /* unconfirmed exact value -- see lmac_mac.h mac_auth_type */
#define AX900_SM_AUTH_TYPE_AUTO      4u /* "auto" is a common convention in this driver family, unconfirmed */

/* c.f. struct sm_connect_req. The trailing ie_buf[64] (u32 words, 256
 * bytes) is confirmed present and sized from the reference struct's own
 * declaration -- what's NOT confirmed is what firmware expects packed
 * into it for a WPA2-PSK connect beyond a standard 802.11 RSN IE; see the
 * "WPA2-PSK key material handoff" comment further up this file and
 * ax900_build_wpa2_ies() in AX900.c. */
typedef struct __attribute__((packed)) {
    ax900_ssid_t ssid;
    uint8_t      bssid[6];
    ax900_chan_def_t chan;
    uint32_t     flags;
    uint16_t     ctrl_port_ethertype;
    uint16_t     ie_len;
    uint16_t     listen_interval;
    uint8_t      dont_wait_bcmc;
    uint8_t      auth_type;
    uint8_t      uapsd_queues;
    uint8_t      vif_idx;
    uint32_t     ie_buf[64];
} ax900_sm_connect_req_t;

typedef struct __attribute__((packed)) {
    uint8_t status; /* 0: connection procedure started, wait for SM_CONNECT_IND */
} ax900_sm_connect_cfm_t;

typedef struct __attribute__((packed)) {
    uint16_t status_code;
    uint8_t  bssid[6];
    uint8_t  roamed;
    uint8_t  vif_idx;
} ax900_sm_connect_ind_t;

/* ---- SCANU_RESULT_IND: one discovered AP per indication ----
 * c.f. struct scanu_result_ind in lmac_msg.h: { length, framectrl,
 * center_freq, band, sta_idx, inst_nbr, rssi, u32_l payload[] }. The
 * fixed portion is 10 bytes (matches this struct exactly, no padding
 * since packed); AX900.c only byte-walks payload[] so it's addressed here
 * as a flexible uint8_t region following this header rather than u32_l
 * words. Whether payload[] still carries the 24-byte 802.11 MAC header
 * ahead of the beacon/probe-resp fixed fields (timestamp+interval+cap,
 * 12 bytes) and tagged IEs, or firmware has already stripped it, is
 * unconfirmed -- ax900_parse_scan_payload() (AX900.c) searches for a
 * plausible IE run rather than assuming one fixed offset. */
typedef struct __attribute__((packed)) {
    uint16_t length;
    uint16_t framectrl;
    uint16_t center_freq;
    uint8_t  band;
    uint8_t  sta_idx;
    uint8_t  inst_nbr;
    int8_t   rssi;
} ax900_scanu_result_ind_hdr_t;

/* ---- 802.11 tagged information elements (public spec, IEEE 802.11-2020
 * 9.4.2 -- unlike the vendor message framing above, this part is not
 * AICSemi-specific and is reproduced with full confidence). Used both to
 * read scan results (SSID/RSN presence) and to build the RSN IE this
 * driver offers firmware in ax900_sm_connect_req_t.ie_buf for a WPA2-PSK
 * ax900_connect(). */
#define AX900_IE_TAG_SSID  0u
#define AX900_IE_TAG_RSN   48u
#define AX900_IE_TAG_VENDOR 221u

/* Minimal WPA2-PSK/CCMP RSN IE this driver offers: version=1, group=CCMP,
 * 1 pairwise cipher (CCMP), 1 AKM (PSK), capabilities=0. 20 bytes total
 * (2-byte tag+len header + 18-byte body). */
#define AX900_RSN_IE_BODY_LEN 18u
#define AX900_RSN_IE_TOTAL_LEN (2u + AX900_RSN_IE_BODY_LEN)

/*
 * WPA2-PSK key material handoff (ax900_connect() -> firmware): NOT
 * confirmed against AICSemi source. The reference driver's sm_connect_req
 * carries a raw association-request IE buffer (ie_buf[64] u32 words) that
 * this driver fills with a real, spec-correct RSN IE (see above -- that
 * part is solid). What is NOT captured anywhere in the available
 * reference source is *how the passphrase/PMK reaches firmware* over this
 * USB transport -- FullMAC parts vary (some want a raw PMK the host
 * derives via PBKDF2, some want the ASCII passphrase and derive PMK
 * firmware-side, some use an entirely separate SET_KEY-style message this
 * driver has never seen). Absent that, this driver appends the raw ASCII
 * passphrase as a private vendor IE (tag 221, a locally-assigned "not a
 * real OUI" prefix) immediately after the RSN IE -- a plausible, common
 * convention, but genuinely experimental. If it doesn't match what a
 * specific firmware build expects, WPA2 connect attempts will time out
 * exactly like an unimplemented one did before; open networks are
 * unaffected either way. */
#define AX900_PRIV_PSK_VENDOR_OUI_0  0x00u
#define AX900_PRIV_PSK_VENDOR_OUI_1  0x00u
#define AX900_PRIV_PSK_VENDOR_OUI_2  0x00u
#define AX900_PRIV_PSK_VENDOR_TYPE   0xACu
