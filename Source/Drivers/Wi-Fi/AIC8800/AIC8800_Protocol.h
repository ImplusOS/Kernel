#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * AIC8800_Protocol.h -- on-wire protocol for AICSemi's AIC8800D80 USB
 * Wi-Fi 6 chipset (the AIC8800D80/SDIO-USB family of dongles).
 *
 * Every structure, id and constant here was transcribed from AICSemi's
 * vendor driver sources:
 *
 *   aic_load_fw/     -- the "loader" image (PID 8d80): DBG_MEM_* messages
 *                       used to bring the chip out of ROM and push the
 *                       three firmware blobs (adid, patch, fmacfw) into
 *                       its RAM, then DBG_START_APP_REQ to run them.
 *   aic8800_fdrv/    -- the "fdrv" image (PID 8d83/8d81 once running): the
 *                       full-MAC firmware's own message set (TASK_MM,
 *                       TASK_SCAN/TASK_SCANU, TASK_SM, TASK_ME, ...).
 *                       lmac_msg.h / lmac_mac.h / ipc_shared.h over
 *                       there are the authority for the struct layouts
 *                       below, and the two #defines below that pin the
 *                       layouts (CONFIG_RWNX_TL4 and the USB transport)
 *                       were checked against the vendor Makefiles: the
 *                       USB build does NOT define CONFIG_RWNX_TL4, so
 *                       bool_l is one byte and every u16_l/u32_l is a
 *                       plain little-endian uint16_t/uint32_t.
 *
 * The USB transport framing (also from the vendor driver) is:
 *
 *   OUT (host -> chip), all packets:
 *     [u16 len][u8 type][u8 0]
 *     len counts everything after this 4-byte header.
 *
 *   type 0x11 = CFG command message (aicwf_set_cmd_tx()):
 *     [u16 len][0x11][0][4 dummy bytes][lmac_msg][param]
 *     lmac_msg = {u16 id, u16 dest_id, u16 src_id, u16 param_len}
 *     and len == sizeof(lmac_msg) + param_len + 4, so the number of
 *     bytes pushed over the bulk pipe is param_len + 16.
 *
 *   type 0x01 = DATA frame (aicwf_usb_bus_txdata()):
 *     [u16 len][0x01][0][struct hostdesc][802.3 payload]
 *     "payload" excludes the 14-byte Ethernet header -- the vendor
 *     driver strips it off and re-packs the addresses/ethertype into
 *     the 28-byte hostdesc -- so len == 28 + payload_len and the
 *     transfer pushes len + 4 bytes. (A data frame whose pushed length
 *     is a multiple of 512 gets one extra 0x00 byte, to avoid the
 *     transfer looking like a short ZLP-terminated one.)
 *
 *   IN (chip -> host):
 *     [u16 pkt_len][u8 type][u8 0][payload...]
 *     type 0x11 (CFG_CMD_RSP): payload is an ipc_e2a_msg --
 *       {u16 id, u16 dummy_dest, u16 dummy_src, u16 param_len,
 *        u32 pattern, param...} with pattern == 0xADDEDE2A; the
 *       transfer is rounded up with ALIGN(pkt_len + 4, 4) total bytes.
 *     type 0x12 (CFG_DATA_CFM): fixed 12-byte packet, no payload.
 *     type 0x01 (DATA): payload is a 56-byte struct hw_rxhdr followed
 *       by pkt_len bytes of 802.11 MPDU. (RX_HWHRD_LEN in the vendor
 *       driver is 60, i.e. 4 bytes of the header above + 56. The
 *       driver only looks at the MPDU, so the hw_rxhdr body is treated
 *       as an opaque 56-byte prefix here -- see AIC8800_Data.c.)
 *
 * Message ids are LMAC_FIRST_MSG(task) | n, i.e. task << 10 | n, with
 * TASK_MM = 0, TASK_DBG = 1, TASK_SCAN = 2, TASK_SCANU = 4, TASK_ME = 5,
 * TASK_SM = 6.
 */

/* ---- USB packet framing ---- */

#define AIC_USB_HDR_LEN             4u
/* Packet kinds. A host -> chip CFG command and the chip's reply to it are
 * both kind 0x11 (the vendor names it for the reply), which is why the
 * vendor's RX walk can classify with a mask: kinds with bit 4 set are CFG
 * traffic, 0x01 is an 802.11 data frame, 0x12 is a TX confirmation. */
#define AIC_USB_TYPE_DATA           0x01u
#define AIC_USB_TYPE_CFG            0x10u /* CFG class mask, not a kind */
#define AIC_USB_TYPE_CFG_CMD        0x11u /* host -> chip: aicwf_set_cmd_tx() */
#define AIC_USB_TYPE_CFG_CMD_RSP    0x11u /* chip -> host: same kind */
#define AIC_USB_TYPE_CFG_DATA_CFM   0x12u /* chip -> host: data confirmation */

#define AIC_IPC_E2A_MSG_PATTERN     0xADDEDE2Au
#define AIC_IPC_E2A_MSG_HDR_LEN     12u  /* id, dummy_dest, dummy_src, param_len, pattern */
#define AIC_RX_HWHRD_LEN            60u  /* AIC_USB_HDR_LEN + sizeof(struct hw_rxhdr) */
#define AIC_TX_HOSTDESC_LEN         28u

/* ---- task ids ---- */

#define AIC_TASK_MM                 0u
#define AIC_TASK_DBG                1u
#define AIC_TASK_SCAN               2u
#define AIC_TASK_TDLS               3u
#define AIC_TASK_SCANU              4u
#define AIC_TASK_ME                 5u
#define AIC_TASK_SM                 6u

#define AIC_FIRST_MSG(task)         ((uint16_t)((task) << 10))
#define AIC_DRV_TASK_ID             100u  /* DRV_TASK_ID: "from the driver" */

/* ---- TASK_DBG (loader image) ---- */

#define AIC_DBG_MEM_READ_REQ            (AIC_FIRST_MSG(AIC_TASK_DBG) + 0u)   /* 0x400 */
#define AIC_DBG_MEM_READ_CFM            (AIC_FIRST_MSG(AIC_TASK_DBG) + 1u)   /* 0x401 */
#define AIC_DBG_MEM_WRITE_REQ           (AIC_FIRST_MSG(AIC_TASK_DBG) + 2u)   /* 0x402 */
#define AIC_DBG_MEM_WRITE_CFM           (AIC_FIRST_MSG(AIC_TASK_DBG) + 3u)   /* 0x403 */
#define AIC_DBG_MEM_BLOCK_WRITE_REQ     (AIC_FIRST_MSG(AIC_TASK_DBG) + 11u)  /* 0x40b */
#define AIC_DBG_MEM_BLOCK_WRITE_CFM     (AIC_FIRST_MSG(AIC_TASK_DBG) + 12u)  /* 0x40c */
#define AIC_DBG_START_APP_REQ           (AIC_FIRST_MSG(AIC_TASK_DBG) + 13u)  /* 0x40d */
#define AIC_DBG_START_APP_CFM           (AIC_FIRST_MSG(AIC_TASK_DBG) + 14u)  /* 0x40e */

#define AIC_HOST_START_APP_AUTO         1u
#define AIC_HOST_START_APP_FUNC         2u

/* DBG_MEM_BLOCK_WRITE_REQ carries a fixed 1 KiB window; the vendor
 * driver always sends the full struct (memsize only says how much of
 * memdata is meaningful), which is what this driver does too. */
#define AIC_FW_BLOCK_SIZE               1024u

/* ---- TASK_MM (fdrv image) ---- */

#define AIC_MM_RESET_REQ                (AIC_FIRST_MSG(AIC_TASK_MM) + 0u)    /* 0x000 */
#define AIC_MM_RESET_CFM                (AIC_FIRST_MSG(AIC_TASK_MM) + 1u)    /* 0x001 */
#define AIC_MM_START_REQ                (AIC_FIRST_MSG(AIC_TASK_MM) + 2u)    /* 0x002 */
#define AIC_MM_START_CFM                (AIC_FIRST_MSG(AIC_TASK_MM) + 3u)    /* 0x003 */
#define AIC_MM_VERSION_REQ              (AIC_FIRST_MSG(AIC_TASK_MM) + 4u)    /* 0x004 */
#define AIC_MM_VERSION_CFM              (AIC_FIRST_MSG(AIC_TASK_MM) + 5u)    /* 0x005 */
#define AIC_MM_ADD_IF_REQ               (AIC_FIRST_MSG(AIC_TASK_MM) + 6u)    /* 0x006 */
#define AIC_MM_ADD_IF_CFM               (AIC_FIRST_MSG(AIC_TASK_MM) + 7u)    /* 0x007 */
#define AIC_MM_SET_FILTER_REQ           (AIC_FIRST_MSG(AIC_TASK_MM) + 14u)   /* 0x00e */
#define AIC_MM_SET_FILTER_CFM           (AIC_FIRST_MSG(AIC_TASK_MM) + 15u)   /* 0x00f */
#define AIC_MM_KEY_ADD_REQ              (AIC_FIRST_MSG(AIC_TASK_MM) + 36u)   /* 0x024 */
#define AIC_MM_KEY_ADD_CFM              (AIC_FIRST_MSG(AIC_TASK_MM) + 37u)   /* 0x025 */
#define AIC_MM_SET_PS_MODE_REQ          (AIC_FIRST_MSG(AIC_TASK_MM) + 49u)   /* 0x031 */
#define AIC_MM_SET_PS_MODE_CFM          (AIC_FIRST_MSG(AIC_TASK_MM) + 50u)   /* 0x032 */
#define AIC_MM_SET_RF_CALIB_REQ         (AIC_FIRST_MSG(AIC_TASK_MM) + 105u)  /* 0x069 */
#define AIC_MM_SET_RF_CALIB_CFM         (AIC_FIRST_MSG(AIC_TASK_MM) + 106u)  /* 0x06a */
#define AIC_MM_GET_MAC_ADDR_REQ         (AIC_FIRST_MSG(AIC_TASK_MM) + 115u)  /* 0x073 */
#define AIC_MM_GET_MAC_ADDR_CFM         (AIC_FIRST_MSG(AIC_TASK_MM) + 116u)  /* 0x074 */
#define AIC_MM_SET_TXPWR_IDX_LVL_REQ    (AIC_FIRST_MSG(AIC_TASK_MM) + 119u)  /* 0x077 */
#define AIC_MM_SET_TXPWR_IDX_LVL_CFM    (AIC_FIRST_MSG(AIC_TASK_MM) + 120u)  /* 0x078 */

/* ---- TASK_SCANU ---- */

#define AIC_SCANU_START_REQ             (AIC_FIRST_MSG(AIC_TASK_SCANU) + 0u)  /* 0x1000 */
#define AIC_SCANU_START_CFM             (AIC_FIRST_MSG(AIC_TASK_SCANU) + 1u)  /* 0x1001 */
#define AIC_SCANU_JOIN_REQ              (AIC_FIRST_MSG(AIC_TASK_SCANU) + 2u)  /* 0x1002 */
#define AIC_SCANU_JOIN_CFM              (AIC_FIRST_MSG(AIC_TASK_SCANU) + 3u)  /* 0x1003 */
#define AIC_SCANU_RESULT_IND            (AIC_FIRST_MSG(AIC_TASK_SCANU) + 4u)  /* 0x1004 */
#define AIC_SCANU_FAST_REQ              (AIC_FIRST_MSG(AIC_TASK_SCANU) + 5u)  /* 0x1005 */
#define AIC_SCANU_FAST_CFM              (AIC_FIRST_MSG(AIC_TASK_SCANU) + 6u)  /* 0x1006 */
#define AIC_SCANU_VENDOR_IE_REQ         (AIC_FIRST_MSG(AIC_TASK_SCANU) + 7u)  /* 0x1007 */
#define AIC_SCANU_VENDOR_IE_CFM         (AIC_FIRST_MSG(AIC_TASK_SCANU) + 8u)  /* 0x1008 */
#define AIC_SCANU_START_CFM_ADDTIONAL   (AIC_FIRST_MSG(AIC_TASK_SCANU) + 9u)  /* 0x1009 */
#define AIC_SCANU_CANCEL_REQ            (AIC_FIRST_MSG(AIC_TASK_SCANU) + 10u) /* 0x100a */
#define AIC_SCANU_CANCEL_CFM            (AIC_FIRST_MSG(AIC_TASK_SCANU) + 11u) /* 0x100b */

/* ---- TASK_SM ---- */

#define AIC_SM_CONNECT_REQ              (AIC_FIRST_MSG(AIC_TASK_SM) + 0u)     /* 0x1800 */
#define AIC_SM_CONNECT_CFM              (AIC_FIRST_MSG(AIC_TASK_SM) + 1u)     /* 0x1801 */
#define AIC_SM_CONNECT_IND              (AIC_FIRST_MSG(AIC_TASK_SM) + 2u)     /* 0x1802 */
#define AIC_SM_DISCONNECT_REQ           (AIC_FIRST_MSG(AIC_TASK_SM) + 3u)     /* 0x1803 */
#define AIC_SM_DISCONNECT_CFM           (AIC_FIRST_MSG(AIC_TASK_SM) + 4u)     /* 0x1804 */
#define AIC_SM_DISCONNECT_IND           (AIC_FIRST_MSG(AIC_TASK_SM) + 5u)     /* 0x1805 */

/* ---- limits ---- */

#define AIC_MAC_ADDR_LEN                6u
#define AIC_MAC_SSID_LEN                32u
#define AIC_MAC_SEC_KEY_LEN             32u
#define AIC_SCAN_SSID_MAX               3u
#define AIC_SCAN_CHANNEL_MAX            42u  /* MAC_DOMAINCHANNEL_24G_MAX(14) + 5G_MAX(28) */
#define AIC_SM_ASSOC_IE_LEN             800u
#define AIC_PHY_CFG_BUF_SIZE            16u
#define AIC_AC_MAX                      4u

/* ---- small common structures ---- */

typedef struct {
    uint16_t array[3]; /* MAC_ADDR_LEN / 2 words */
} aic_mac_addr_t;

typedef struct {
    uint8_t length;
    uint8_t array[AIC_MAC_SSID_LEN];
} aic_mac_ssid_t;

/* Primary channel definition. Natural alignment: 2 + 1 + 1 + 1 -> 6. */
typedef struct {
    uint16_t freq;
    uint8_t  band;   /* 0 = 2.4 GHz, 1 = 5 GHz (enum mac_chan_band) */
    uint8_t  flags;  /* enum mac_chan_flags */
    int8_t   tx_power;
} aic_mac_chan_def_t;

/* Key material. `length` is followed by 3 bytes of padding so that the
 * 32-byte word array is 4-byte aligned -- sizeof == 36. */
typedef struct {
    uint8_t  length;
    uint32_t array[AIC_MAC_SEC_KEY_LEN / 4u];
} aic_mac_sec_key_t;

/* ---- DBG (loader) messages ---- */

typedef struct {
    uint32_t memaddr;
} aic_dbg_mem_read_req_t;

typedef struct {
    uint32_t memaddr;
    uint32_t memdata;
} aic_dbg_mem_read_cfm_t;

typedef struct {
    uint32_t memaddr;
    uint32_t memdata;
} aic_dbg_mem_write_req_t;

/* Both write confirmations are the read CFM's shape: the vendor's
 * struct dbg_mem_write_cfm / dbg_mem_block_write_cfm. */
typedef struct {
    uint32_t memaddr;
    uint32_t memdata;
} aic_dbg_mem_write_cfm_t;

typedef struct {
    uint32_t memaddr;
    uint32_t memsize;
    uint32_t memdata[AIC_FW_BLOCK_SIZE / 4u];
} aic_dbg_mem_block_write_req_t;

typedef struct {
    uint32_t memaddr;
    uint32_t memsize;
} aic_dbg_mem_block_write_cfm_t;

typedef struct {
    uint32_t bootaddr;
    uint32_t boottype; /* HOST_START_APP_AUTO / _FUNC */
} aic_dbg_start_app_req_t;

/* ---- MM messages ---- */

typedef struct {
    uint32_t get; /* 1 = read the MAC out of efuse */
} aic_mm_get_mac_addr_req_t;

typedef struct {
    uint8_t mac_addr[AIC_MAC_ADDR_LEN];
} aic_mm_get_mac_addr_cfm_t;

typedef struct {
    uint32_t phy_cfg[AIC_PHY_CFG_BUF_SIZE];
    uint32_t uapsd_timeout;
    uint16_t lp_clk_accuracy;
} aic_mm_start_req_t; /* sizeof == 72 */

typedef struct {
    uint8_t type; /* enum: MM_STA == 0 */
    aic_mac_addr_t addr;
    uint8_t p2p;  /* bool_l */
} aic_mm_add_if_req_t; /* sizeof == 10: the vendor's struct is laid out the
                         * same way (u16-aligned mac_addr, so one pad byte
                         * before it and one after p2p) -- the firmware was
                         * built against that header too. */

typedef struct {
    uint8_t status;
    uint8_t inst_nbr;
} aic_mm_add_if_cfm_t;

typedef struct {
    uint32_t cal_cfg_24g;
    uint32_t cal_cfg_5g;
    uint32_t param_alpha;
    uint32_t bt_calib_en;
    uint32_t bt_calib_param;
    uint8_t  xtal_cap;
    uint8_t  xtal_cap_fine;
} aic_mm_set_rf_calib_req_t; /* sizeof == 24 (2 bytes of tail padding) */

typedef struct {
    uint32_t rxgain_24g_addr;
    uint32_t rxgain_5g_addr;
    uint32_t txgain_24g_addr;
    uint32_t txgain_5g_addr;
} aic_mm_set_rf_calib_cfm_t;

typedef struct {
    uint8_t  enable;
    int8_t   pwrlvl_11b_11ag_2g4[12];
    int8_t   pwrlvl_11n_11ac_2g4[10];
    int8_t   pwrlvl_11ax_2g4[12];
    int8_t   pwrlvl_11a_5g[12];
    int8_t   pwrlvl_11n_11ac_5g[10];
    int8_t   pwrlvl_11ax_5g[12];
} aic_txpwr_lvl_conf_v3_t; /* 69 bytes, alignment 1 */

/* mm_set_txpwr_lvl_req: the vendor fills this message from a union of the
 * v1..v4 power tables and always sends sizeof(the union) -- 95 bytes in the
 * D80 build. Only the v3 member is ever filled in (the D80 bring-up path is
 * what sends this), every other byte staying zero, so the message is
 * modelled as the raw union-sized buffer: aic_txpwr_lvl_conf_v3_t above
 * lays the v3 fields over its front, and the tail is the zero padding a
 * firmware built against a wider table may still look at. */
#define AIC_TXPWR_LVL_REQ_LEN 95u
typedef struct {
    uint8_t raw[AIC_TXPWR_LVL_REQ_LEN];
} aic_mm_set_txpwr_lvl_req_t;

typedef struct {
    uint8_t key_idx;
    uint8_t sta_idx; /* 0xFF (INVALID_STA_IDX) for a default/group key */
    aic_mac_sec_key_t key;
    uint8_t cipher_suite; /* 2 == CCMP */
    uint8_t inst_nbr;
    uint8_t spp;
    uint8_t pairwise; /* bool_l */
} aic_mm_key_add_req_t; /* sizeof == 44 */

typedef struct {
    uint8_t status;
    uint8_t hw_key_idx;
} aic_mm_key_add_cfm_t;

/* ---- SCANU messages ---- */

typedef struct {
    aic_mac_chan_def_t chan[AIC_SCAN_CHANNEL_MAX];
    aic_mac_ssid_t     ssid[AIC_SCAN_SSID_MAX];
    aic_mac_addr_t     bssid;
    uint32_t add_ies;
    uint16_t add_ie_len;
    uint8_t  vif_idx;
    uint8_t  chan_cnt;
    uint8_t  ssid_cnt;
    uint8_t  no_cck;
    uint32_t duration; /* us */
} aic_scanu_start_req_t; /* sizeof == 376 */

typedef struct {
    uint8_t vif_idx;
    uint8_t status;
    uint8_t result_cnt;
} aic_scanu_start_cfm_t;

/* Followed by `length` bytes of 802.11 management frame (the beacon or
 * probe response, starting with its 24-byte MAC header). */
typedef struct {
    uint16_t length;
    uint16_t framectrl;
    uint16_t center_freq;
    uint8_t  band;
    uint8_t  sta_idx;
    uint8_t  inst_nbr;
    int8_t   rssi;
} aic_scanu_result_ind_t;

/* ---- SM (station manager) messages ---- */

/* mac_connection_flags */
#define AIC_SM_CONTROL_PORT_HOST    (1u << 0)
#define AIC_SM_CONTROL_PORT_NO_ENC  (1u << 1)
#define AIC_SM_DISABLE_HT           (1u << 2)
#define AIC_SM_WPA_WPA2_IN_USE      (1u << 3)
#define AIC_SM_MFP_IN_USE           (1u << 4)
#define AIC_SM_REASSOCIATION        (1u << 5)

#define AIC_SM_STATUS_SUCCESS       0u

typedef struct {
    aic_mac_ssid_t     ssid;
    aic_mac_addr_t     bssid;
    aic_mac_chan_def_t chan;
    uint32_t flags;
    uint16_t ctrl_port_ethertype; /* in network byte order, as a host u16 */
    uint16_t ie_len;
    uint16_t listen_interval;
    uint8_t  dont_wait_bcmc; /* bool_l */
    uint8_t  auth_type;      /* 0 == WLAN_AUTH_OPEN */
    uint8_t  uapsd_queues;
    uint8_t  vif_idx;
    uint32_t ie_buf[64];
} aic_sm_connect_req_t; /* sizeof == 320 */

typedef struct {
    uint8_t status;
} aic_sm_connect_cfm_t;

/* Only the leading fields are used by this driver; the firmware may send
 * a shorter or longer IE buffer than the full union. */
typedef struct {
    uint16_t status_code;
    aic_mac_addr_t bssid;
    uint8_t roamed;
    uint8_t vif_idx;
    uint8_t ap_idx;
    uint8_t ch_idx;
} aic_sm_connect_ind_t;

typedef struct {
    uint16_t reason_code;
    uint8_t  vif_idx;
} aic_sm_disconnect_req_t;

/* ---- data path ---- */

/* struct hostdesc, 28 bytes, no padding (ipc_shared.h in the vendor
 * driver). packet_len is the length of the 802.3 *payload*: the driver
 * moves the 14-byte Ethernet header's contents into eth_dest/eth_src/
 * ethertype and sends only what follows the header. */
typedef struct __attribute__((packed)) {
    uint16_t packet_len;
    uint16_t flags_ext;
    uint32_t status_desc_addr;
    uint8_t  eth_dest[AIC_MAC_ADDR_LEN];
    uint8_t  eth_src[AIC_MAC_ADDR_LEN];
    uint16_t ethertype; /* copied raw from the 802.3 header (network order in memory) */
    uint8_t  ac;        /* access category: 0 == BE/BK? (vendor: txq hwq id: BK 0, BE 1, VI 2, VO 3) */
    uint8_t  tid;
    uint8_t  vif_idx;
    uint8_t  staid;
    uint16_t flags;
} aic_hostdesc_t;

/* ---- layout checks ---- */

_Static_assert(sizeof(aic_mac_addr_t) == 6, "mac_addr must be 6 bytes");
_Static_assert(sizeof(aic_mac_ssid_t) == 33, "mac_ssid must be 33 bytes");
_Static_assert(sizeof(aic_mac_chan_def_t) == 6, "mac_chan_def must be 6 bytes");
_Static_assert(sizeof(aic_mac_sec_key_t) == 36, "mac_sec_key must be 36 bytes");
_Static_assert(sizeof(aic_mm_start_req_t) == 72, "mm_start_req must be 72 bytes");
_Static_assert(sizeof(aic_mm_add_if_req_t) == 10, "mm_add_if_req must be 10 bytes");
_Static_assert(sizeof(aic_mm_set_rf_calib_req_t) == 24, "mm_set_rf_calib_req must be 24 bytes");
_Static_assert(sizeof(aic_mm_set_txpwr_lvl_req_t) == AIC_TXPWR_LVL_REQ_LEN, "mm_set_txpwr_lvl_req");
_Static_assert(sizeof(aic_mm_key_add_req_t) == 44, "mm_key_add_req must be 44 bytes");
_Static_assert(sizeof(aic_scanu_start_req_t) == 376, "scanu_start_req must be 376 bytes");
_Static_assert(sizeof(aic_sm_connect_req_t) == 320, "sm_connect_req must be 320 bytes");
_Static_assert(sizeof(aic_hostdesc_t) == 28, "hostdesc must be 28 bytes");
