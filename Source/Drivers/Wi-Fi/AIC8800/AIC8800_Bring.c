#include "AIC8800.h"

/*
 * AIC8800_Bring.c -- the fdrv stage: station bring-up, scanning, connect.
 *
 * Once the loader stage has run the firmware, the chip comes back as
 * a69c:8d81 speaking the full TASK_MM/TASK_SCANU/TASK_SM message set.
 * probe() has already classified it and aic_tick() calls aic_bring_step()
 * until `ready` is set, then the steady state takes over:
 *
 *   bring-up   rwnx_ic_rf_init()      calibrated TX power + RF calibration
 *              MM_GET_MAC_ADDR        read the efuse MAC (logged only: the
 *                                     kernel already cached the one this
 *                                     driver handed it at boot)
 *              MM_RESET, MM_VERSION   reset the MAC, log the firmware rev
 *              MM_START               start the MAC with the default PHY
 *                                     config
 *              MM_ADD_IF              add the station interface, which
 *                                     yields the vif index every later
 *                                     message carries
 *   scan       SCANU_START            sweep 2.4 GHz ch1..13 for
 *                                     AIC_SCAN_WINDOW_MS; the results
 *                                     arrive as SCANU_RESULT_IND
 *   connect    SM_CONNECT_REQ         BSSID/channel taken from our own
 *                                     scan results (the Wi-Fi panel only
 *                                     hands us an SSID and a passphrase)
 *              SM_CONNECT_IND         associated; a passphrase turns this
 *                                     into the 4-way handshake of
 *                                     AIC8800_Wpa.c, and the connection
 *                                     only counts as up once its keys are
 *                                     in the firmware (is_ready() and
 *                                     therefore DHCP wait for that)
 *
 * Every message here follows the layouts in AIC8800_Protocol.h and the
 * vendor's own field values (see the comments on each constant).
 */

/* ---- constants ---- */

/* MM_SET_TXPWR_IDX_LVL: the v3 table is what this chip's path sends, with
 * enable = 1 and every power limit left at the firmware's own default
 * (zero: the vendor driver only ever fills them from module parameters
 * that default to zero). The v1/v2/v4 members of the union are never
 * touched. */
#define AIC_TXPWR_LVL_V3_ENABLE     1u

/* MM_SET_RF_CALIB, D80 values (rwnx_ic_rf_init). */
#define AIC_RF_CAL_CFG_24G          0x00000F8Fu
#define AIC_RF_CAL_CFG_5G           0x00000F0Fu
#define AIC_RF_CAL_PARAM_ALPHA      0x0C34C008u
#define AIC_RF_CAL_BT_ENABLE        0u
#define AIC_RF_CAL_BT_PARAM         0x00264203u
#define AIC_RF_CAL_XTAL_CAP         0u
#define AIC_RF_CAL_XTAL_CAP_FINE    0u

/* MM_START: the vendor sends a zeroed PHY config with these two settings. */
#define AIC_MM_START_UAPSD_TIMEOUT  300u
#define AIC_MM_START_LP_CLK_ACC     20u

#define AIC_MM_STA_TYPE             0u /* enum mm_sta_type: MM_STA */
#define AIC_KEY_CIPHER_CCMP         2u
#define AIC_STA_IDX_INVALID         0xFFu /* INVALID_STA_IDX, for ap_idx */
#define AIC_REASON_DEAUTH_LEAVING   3u

#define AIC_SCAN_CHAN_COUNT         42u

/* The firmware expects center frequencies, not channel numbers.  Keep the
 * table explicit: 5 GHz has gaps (DFS/non-DFS and regional channels), so a
 * simple arithmetic sequence would ask the radio to scan invalid channels.
 * This covers the channels exposed by the D80 firmware's 2.4/5 GHz domain. */
static const uint16_t g_scan_frequencies[AIC_SCAN_CHAN_COUNT] = {
    2412u, 2417u, 2422u, 2427u, 2432u, 2437u, 2442u,
    2447u, 2452u, 2457u, 2462u, 2467u, 2472u, 2484u,
    5180u, 5200u, 5220u, 5240u, 5260u, 5280u, 5300u, 5320u,
    5500u, 5520u, 5540u, 5560u, 5580u, 5600u, 5620u, 5640u,
    5660u, 5680u, 5700u, 5745u, 5765u, 5785u, 5805u, 5825u,
    5845u, 5865u, 5885u, 5905u
};

/* Wait for the reply to a request the firmware answers before it starts
 * the work it describes (the association itself is asynchronous: its
 * result is SM_CONNECT_IND). */
#define AIC_CONNECT_CFM_TIMEOUT_MS  500u

/* 802.11 information elements. */
#define AIC_IE_SSID                 0u
#define AIC_IE_VENDOR               221u
#define AIC_IE_RSN                  48u

/* An SCANU_RESULT_IND payload: the fixed report header followed by the
 * complete management frame (24-byte MAC header + 12 bytes of beacon fixed
 * fields + IEs). */
#define AIC_SCAN_IND_HDR_LEN        10u
#define AIC_MGMT_HDR_LEN            24u
#define AIC_BEACON_FIXED_LEN        12u
#define AIC_BSSID_OFS               16u /* addr3 of a beacon/probe response */
#define AIC_IE_OFS                  (AIC_MGMT_HDR_LEN + AIC_BEACON_FIXED_LEN)

/* ---- helpers ---- */

/* Walk an IE blob. Returns the element (its id byte) or NULL. */
static const uint8_t *ie_find(const uint8_t *ies, uint32_t len, uint8_t id,
                              uint32_t *out_len)
{
    uint32_t pos = 0u;

    while (pos + 2u <= len) {
        uint8_t ie_id = ies[pos];
        uint8_t ie_len = ies[pos + 1u];

        if (pos + 2u + (uint32_t)ie_len > len) {
            break;
        }
        if (ie_id == id) {
            if (out_len != NULL) {
                *out_len = ie_len;
            }
            return ies + pos;
        }
        pos += 2u + (uint32_t)ie_len;
    }
    return NULL;
}

/* The WPA1 case: a vendor-specific element carrying the Microsoft OUI and
 * type 1. Anything else in an IE list is not a security claim. */
static bool ie_has_wpa1(const uint8_t *ies, uint32_t len)
{
    uint32_t pos = 0u;

    while (pos + 2u <= len) {
        uint8_t ie_id = ies[pos];
        uint8_t ie_len = ies[pos + 1u];

        if (pos + 2u + (uint32_t)ie_len > len) {
            break;
        }
        if (ie_id == AIC_IE_VENDOR && ie_len >= 4u &&
            ies[pos + 2u] == 0x00u && ies[pos + 3u] == 0x50u &&
            ies[pos + 4u] == 0xF2u && ies[pos + 5u] == 0x01u) {
            return true;
        }
        pos += 2u + (uint32_t)ie_len;
    }
    return false;
}

static void bring_log_status(const char *what, uint32_t value)
{
    aic_log("[AIC8800] ");
    aic_log(what);
    aic_log(" = ");
    aic_log_hex32(value);
    aic_log("\n");
}

/* ---- bring-up steps ---- */

/* TX power table + RF calibration (the vendor's rwnx_ic_rf_init). The two
 * optional sub-requests (per-rate offset adjustment and the 2x offset
 * table) are only sent when their enable flag is set, and the D80 path
 * leaves both disabled. */
static bool bring_rf_init(void)
{
    aic_mm_set_txpwr_lvl_req_t txpwr;
    aic_mm_set_rf_calib_req_t calib;
    uint8_t cfm[8];

    aic_memset(&txpwr, 0u, sizeof(txpwr));
    aic_memset(&cfm, 0u, sizeof(cfm));
    txpwr.raw[0] = AIC_TXPWR_LVL_V3_ENABLE;
    if (!aic_cmd(AIC_MM_SET_TXPWR_IDX_LVL_REQ, AIC_MM_SET_TXPWR_IDX_LVL_CFM,
                 (uint16_t)AIC_TASK_MM, &txpwr, (uint16_t)sizeof(txpwr),
                 cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS)) {
        return false;
    }

    aic_memset(&calib, 0u, sizeof(calib));
    calib.cal_cfg_24g = AIC_RF_CAL_CFG_24G;
    calib.cal_cfg_5g = AIC_RF_CAL_CFG_5G;
    calib.param_alpha = AIC_RF_CAL_PARAM_ALPHA;
    calib.bt_calib_en = AIC_RF_CAL_BT_ENABLE;
    calib.bt_calib_param = AIC_RF_CAL_BT_PARAM;
    calib.xtal_cap = AIC_RF_CAL_XTAL_CAP;
    calib.xtal_cap_fine = AIC_RF_CAL_XTAL_CAP_FINE;
    aic_memset(&cfm, 0u, sizeof(cfm));
    return aic_cmd(AIC_MM_SET_RF_CALIB_REQ, AIC_MM_SET_RF_CALIB_CFM,
                   (uint16_t)AIC_TASK_MM, &calib, (uint16_t)sizeof(calib),
                   cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS);
}

/* The efuse MAC is only readable once the firmware runs, which is long
 * after the kernel cached the address this driver reported at boot; so
 * read it for the log only (see the note on the MAC in AIC8800.c). */
static bool bring_read_mac(void)
{
    aic_mm_get_mac_addr_req_t req;
    aic_mm_get_mac_addr_cfm_t cfm;

    req.get = 1u;
    aic_memset(&cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_MM_GET_MAC_ADDR_REQ, AIC_MM_GET_MAC_ADDR_CFM,
                 (uint16_t)AIC_TASK_MM, &req, (uint16_t)sizeof(req),
                 &cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS)) {
        return false;
    }
    aic_log("[AIC8800] efuse mac ");
    for (uint32_t i = 0u; i < 6u; ++i) {
        aic_log_hex32(cfm.mac_addr[i]);
    }
    aic_log(" (not used: this station keeps its locally administered one)\n");
    return true;
}

static bool bring_mm_reset(void)
{
    uint8_t cfm[8];

    aic_memset(cfm, 0u, sizeof(cfm));
    return aic_cmd(AIC_MM_RESET_REQ, AIC_MM_RESET_CFM, (uint16_t)AIC_TASK_MM,
                   NULL, 0u, cfm, (uint16_t)sizeof(cfm),
                   AIC_CMD_TIMEOUT_MS);
}

static bool bring_mm_version(void)
{
    uint8_t cfm[16];

    aic_memset(cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_MM_VERSION_REQ, AIC_MM_VERSION_CFM,
                 (uint16_t)AIC_TASK_MM, NULL, 0u, cfm, (uint16_t)sizeof(cfm),
                 AIC_CMD_TIMEOUT_MS)) {
        return false;
    }
    /* The first word of the reply is the LMAC version. */
    aic_log("[AIC8800] firmware version ");
    aic_log_hex32(aic_rd32(cfm));
    aic_log("\n");
    return true;
}

static bool bring_mm_start(void)
{
    aic_mm_start_req_t req;
    uint8_t cfm[8];

    aic_memset(&req, 0u, sizeof(req));
    req.uapsd_timeout = AIC_MM_START_UAPSD_TIMEOUT;
    req.lp_clk_accuracy = AIC_MM_START_LP_CLK_ACC;
    aic_memset(cfm, 0u, sizeof(cfm));
    return aic_cmd(AIC_MM_START_REQ, AIC_MM_START_CFM, (uint16_t)AIC_TASK_MM,
                   &req, (uint16_t)sizeof(req), cfm, (uint16_t)sizeof(cfm),
                   AIC_CMD_TIMEOUT_MS);
}

static bool bring_mm_add_if(void)
{
    aic_mm_add_if_req_t req;
    aic_mm_add_if_cfm_t cfm;

    aic_memset(&req, 0u, sizeof(req));
    req.type = AIC_MM_STA_TYPE;
    aic_memcpy(&req.addr, g_aic.mac, 6u);
    req.p2p = 0u;

    aic_memset(&cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_MM_ADD_IF_REQ, AIC_MM_ADD_IF_CFM, (uint16_t)AIC_TASK_MM,
                 &req, (uint16_t)sizeof(req), &cfm, (uint16_t)sizeof(cfm),
                 AIC_CMD_TIMEOUT_MS)) {
        return false;
    }
    /* A non-zero status with a plausible interface index is what a retry
     * after a partially completed bring-up looks like (the firmware still
     * holds the interface); anything else is a real failure. */
    if (cfm.status != 0u && cfm.inst_nbr >= 4u) {
        bring_log_status("MM_ADD_IF status", cfm.status);
        return false;
    }
    if (cfm.status != 0u) {
        bring_log_status("MM_ADD_IF status (kept the existing interface)",
                         cfm.status);
    }
    g_aic.vif_idx = cfm.inst_nbr;
    return true;
}

static void bring_fail(const char *what)
{
    aic_log("[AIC8800] bring-up: ");
    aic_log(what);
    aic_log(" failed (attempt ");
    aic_log_hex32(g_aic.bring_attempts + 1u);
    aic_log("/");
    aic_log_hex32(AIC_MAX_ATTEMPTS);
    aic_log(")\n");

    ++g_aic.bring_attempts;
    if (g_aic.bring_attempts >= AIC_MAX_ATTEMPTS) {
        g_aic.bring_phase = AIC_BRING_FAILED;
        aic_log("[AIC8800] giving up on the station bring-up\n");
    } else {
        g_aic.bring_phase = AIC_BRING_IDLE; /* retry the ladder later */
    }
}

void aic_bring_reset(void)
{
    g_aic.bring_phase = AIC_BRING_IDLE;
    g_aic.bring_attempts = 0u;
    g_aic.bring_last_attempt = 0u;
    g_aic.ready = false;
    g_aic.vif_idx = 0u;
    g_aic.ap_idx = AIC_STA_IDX_INVALID;
}

void aic_bring_step(void)
{
    if (!g_aic.attached || g_aic_api == NULL) {
        return;
    }
    if (g_aic.ready || g_aic.bring_phase == AIC_BRING_FAILED) {
        return;
    }

    if (g_aic.bring_phase == AIC_BRING_IDLE) {
        if (g_aic.bring_attempts != 0u &&
            aic_ms_since(g_aic.bring_last_attempt, AIC_RETRY_INTERVAL_MS) <
                AIC_RETRY_INTERVAL_MS) {
            return;
        }
        g_aic.bring_last_attempt = aic_now_ticks();
        g_aic.bring_phase = AIC_BRING_RF;
    }

    switch (g_aic.bring_phase) {
    case AIC_BRING_RF:
        if (!bring_rf_init()) {
            bring_fail("rf calibration");
            return;
        }
        g_aic.bring_phase = AIC_BRING_MAC_ADDR;
        break;

    case AIC_BRING_MAC_ADDR:
        if (!bring_read_mac()) {
            bring_fail("mac address read");
            return;
        }
        g_aic.bring_phase = AIC_BRING_RESET;
        break;

    case AIC_BRING_RESET:
        if (!bring_mm_reset()) {
            bring_fail("mm reset");
            return;
        }
        g_aic.bring_phase = AIC_BRING_VERSION;
        break;

    case AIC_BRING_VERSION:
        if (!bring_mm_version()) {
            bring_fail("version read");
            return;
        }
        g_aic.bring_phase = AIC_BRING_START;
        break;

    case AIC_BRING_START:
        if (!bring_mm_start()) {
            bring_fail("mm start");
            return;
        }
        g_aic.bring_phase = AIC_BRING_ADD_IF;
        break;

    case AIC_BRING_ADD_IF:
        if (!bring_mm_add_if()) {
            bring_fail("add interface");
            return;
        }
        g_aic.bring_phase = AIC_BRING_DONE;
        g_aic.ready = true;
        g_aic.conn = AIC_CONN_NONE;
        aic_log("[AIC8800] station ready on vif ");
        aic_log_hex32((uint32_t)g_aic.vif_idx);
        aic_log("\n");
        break;

    default:
        break;
    }
}

/* ---- scanning ---- */

bool aic_scan_start(void)
{
    aic_scanu_start_req_t req;

    if (!g_aic.ready) {
        return false;
    }

    aic_memset(&req, 0u, sizeof(req));
    for (uint32_t ch = 0u; ch < AIC_SCAN_CHAN_COUNT; ++ch) {
        req.chan[ch].freq = g_scan_frequencies[ch];
        req.chan[ch].band = (g_scan_frequencies[ch] >= 5000u) ? 1u : 0u;
        req.chan[ch].flags = 0u;  /* active scan on every channel */
        req.chan[ch].tx_power = 0;
    }
    req.chan_cnt = (uint8_t)AIC_SCAN_CHAN_COUNT;
    req.ssid_cnt = 0u; /* wildcard: report every AP */
    req.vif_idx = g_aic.vif_idx;
    req.no_cck = 0u;

    /* A new sweep replaces the previous results. */
    aic_memset(g_aic.scan_results, 0u, sizeof(g_aic.scan_results));
    aic_memset(g_aic.scan_freq, 0u, sizeof(g_aic.scan_freq));
    g_aic.scan_result_count = 0u;
    g_aic.scanning = true;
    g_aic.scan_deadline = aic_now_ticks();

    /* Fire and forget: the firmware answers SCANU_START_CFM (0x1001) and
     * then one SCANU_RESULT_IND per AP, but the window below is what ends
     * the scan either way. */
    if (!aic_cmd_nowait(AIC_SCANU_START_REQ, (uint16_t)AIC_TASK_SCANU,
                        &req, (uint16_t)sizeof(req))) {
        g_aic.scanning = false;
        return false;
    }
    return true;
}

uint32_t aic_scan_results_copy(driver_wifi_scan_result_t *out, uint32_t max_count)
{
    uint32_t count = g_aic.scan_result_count;

    if (out == NULL || max_count == 0u) {
        return count;
    }
    if (count > max_count) {
        count = max_count;
    }
    aic_memcpy(out, g_aic.scan_results,
               count * (uint32_t)sizeof(driver_wifi_scan_result_t));
    return count;
}

void aic_handle_scan_result(const uint8_t *payload, uint16_t len)
{
    const uint8_t *frame;
    const uint8_t *bssid;
    const uint8_t *ies;
    const uint8_t *ssid_ie;
    uint32_t ies_len;
    uint32_t ssid_len = 0u;
    uint32_t frame_len;
    uint16_t center_freq;
    uint32_t security = (uint32_t)DRIVER_WIFI_SECURITY_OPEN;
    int8_t rssi;

    if (!g_aic.scanning || payload == NULL ||
        len < AIC_SCAN_IND_HDR_LEN + AIC_IE_OFS) {
        return;
    }

    frame_len = aic_rd16(payload);
    center_freq = aic_rd16(payload + 4u);
    rssi = (int8_t)payload[9u];

    frame = payload + AIC_SCAN_IND_HDR_LEN;
    if (frame_len > (uint32_t)len - AIC_SCAN_IND_HDR_LEN) {
        frame_len = (uint32_t)len - AIC_SCAN_IND_HDR_LEN;
    }
    if (frame_len < AIC_IE_OFS) {
        return;
    }

    bssid = frame + AIC_BSSID_OFS;
    ies = frame + AIC_IE_OFS;
    ies_len = frame_len - AIC_IE_OFS;

    ssid_ie = ie_find(ies, ies_len, AIC_IE_SSID, &ssid_len);
    if (ssid_ie == NULL || ssid_len == 0u) {
        return; /* hidden SSID: nothing the panel could show */
    }
    if (ssid_len > DRIVER_WIFI_SSID_MAX) {
        ssid_len = DRIVER_WIFI_SSID_MAX;
    }
    if (ie_find(ies, ies_len, AIC_IE_RSN, NULL) != NULL ||
        ie_has_wpa1(ies, ies_len)) {
        security = (uint32_t)DRIVER_WIFI_SECURITY_WPA_PSK;
    }

    /* One entry per BSSID: a later report of the same AP only upgrades the
     * signal strength (a roaming AP pair would otherwise fill the list). */
    for (uint32_t i = 0u; i < g_aic.scan_result_count; ++i) {
        if (aic_memequal(g_aic.scan_results[i].bssid, bssid, 6u)) {
            if (rssi > g_aic.scan_results[i].rssi_dbm) {
                g_aic.scan_results[i].rssi_dbm = rssi;
                g_aic.scan_results[i].security = security;
            }
            return;
        }
    }
    if (g_aic.scan_result_count >= AIC_MAX_SCAN_RESULTS) {
        return;
    }

    driver_wifi_scan_result_t *out =
        &g_aic.scan_results[g_aic.scan_result_count];
    aic_memset(out, 0u, sizeof(*out));
    aic_memcpy(out->ssid, ssid_ie + 2u, ssid_len);
    aic_memcpy(out->bssid, bssid, 6u);
    out->rssi_dbm = rssi;
    out->security = security;
    g_aic.scan_freq[g_aic.scan_result_count] = center_freq;
    ++g_aic.scan_result_count;
}

/* ---- connect / disconnect ---- */

/* Find the AP the caller asked for in the last scan's results: the Wi-Fi
 * panel hands us an SSID, and the connect request needs the BSSID and the
 * channel to aim at. Returns the result index or -1. */
static int32_t scan_find_ssid(const char *ssid, uint32_t ssid_len)
{
    for (uint32_t i = 0u; i < g_aic.scan_result_count; ++i) {
        const char *cand = g_aic.scan_results[i].ssid;
        if (aic_strnlen(cand, DRIVER_WIFI_SSID_MAX + 1u) == ssid_len &&
            aic_memequal(cand, ssid, ssid_len)) {
            return (int32_t)i;
        }
    }
    return -1;
}

bool aic_connect(const char *ssid, const char *psk)
{
    aic_sm_connect_req_t req;
    uint8_t cfm[8];
    uint32_t ssid_len;
    uint32_t psk_len = 0u;
    int32_t index;

    if (!g_aic.ready || ssid == NULL) {
        return false;
    }
    ssid_len = aic_strnlen(ssid, DRIVER_WIFI_SSID_MAX + 1u);
    if (ssid_len == 0u || ssid_len > DRIVER_WIFI_SSID_MAX) {
        return false;
    }
    if (psk != NULL) {
        psk_len = aic_strnlen(psk, (uint32_t)sizeof(g_aic.psk));
        if (psk_len < 8u || psk_len > 63u) {
            aic_log("[AIC8800] passphrase must be 8..63 characters\n");
            return false;
        }
    }

    index = scan_find_ssid(ssid, ssid_len);
    if (index < 0) {
        aic_log("[AIC8800] connect: no scan result for that network\n");
        return false;
    }

    /* A connection that is already up (or still running) is torn down
     * first: the firmware will not accept a second connect on the same
     * interface. */
    if (g_aic.conn != AIC_CONN_NONE && g_aic.conn != AIC_CONN_FAILED) {
        aic_disconnect();
    }
    aic_wpa_stop();

    aic_memset(&req, 0u, sizeof(req));
    req.ssid.length = (uint8_t)ssid_len;
    aic_memcpy(req.ssid.array, ssid, ssid_len);
    aic_memcpy(&req.bssid, g_aic.scan_results[index].bssid, 6u);
    req.chan.freq = (uint16_t)g_aic.scan_freq[index];
    req.chan.band = (g_aic.scan_freq[index] >= 5000u) ? 1u : 0u;
    req.chan.flags = 0u;
    req.chan.tx_power = 0;
    req.ctrl_port_ethertype = 0x8E88u; /* 0x888e as it sits in the frame */
    req.listen_interval = 0u;
    req.dont_wait_bcmc = 0u;
    req.auth_type = 0u; /* WLAN_AUTH_OPEN: WPA2-PSK authenticates in RSN */
    req.uapsd_queues = 1u; /* AC_VO */
    req.vif_idx = g_aic.vif_idx;
    req.flags = AIC_SM_CONTROL_PORT_HOST | AIC_SM_CONTROL_PORT_NO_ENC;

    if (psk != NULL) {
        uint32_t ie_len = 0u;
        const uint8_t *ie = aic_wpa_rsn_ie(&ie_len);
        if (ie == NULL || ie_len == 0u ||
            ie_len > (uint32_t)sizeof(req.ie_buf)) {
            return false;
        }
        aic_memcpy(req.ie_buf, ie, ie_len);
        req.ie_len = (uint16_t)ie_len;
        req.flags |= AIC_SM_WPA_WPA2_IN_USE;
    }

    /* Remember the target *before* the request goes out: the firmware can
     * report the association (SM_CONNECT_IND) before this call returns. */
    aic_memcpy(g_aic.ssid, ssid, ssid_len + 1u);
    if (psk != NULL) {
        aic_memcpy(g_aic.psk, psk, psk_len + 1u);
    } else {
        g_aic.psk[0] = '\0';
    }
    aic_memcpy(g_aic.bssid, g_aic.scan_results[index].bssid, 6u);
    g_aic.center_freq = (uint16_t)g_aic.scan_freq[index];
    g_aic.ap_idx = AIC_STA_IDX_INVALID;
    g_aic.wpa_enabled = (psk != NULL);
    g_aic.connect_open = (psk == NULL);
    g_aic.conn = AIC_CONN_CONNECTING;
    g_aic.connect_deadline = aic_now_ticks();

    aic_memset(cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_SM_CONNECT_REQ, AIC_SM_CONNECT_CFM,
                 (uint16_t)AIC_TASK_SM, &req, (uint16_t)sizeof(req),
                 cfm, (uint16_t)sizeof(cfm), AIC_CONNECT_CFM_TIMEOUT_MS) ||
        cfm[0] != AIC_SM_STATUS_SUCCESS) {
        g_aic.conn = AIC_CONN_FAILED;
        aic_log("[AIC8800] connect request rejected\n");
        return false;
    }

    aic_log("[AIC8800] connecting to ");
    aic_log(g_aic.ssid);
    aic_log(psk != NULL ? " (WPA2-PSK)\n" : " (open)\n");
    return true;
}

void aic_mm_disconnect(void)
{
    aic_sm_disconnect_req_t req;
    uint8_t cfm[4];

    if (!g_aic.ready) {
        return;
    }
    aic_memset(&req, 0u, sizeof(req));
    aic_memset(cfm, 0u, sizeof(cfm));
    req.reason_code = AIC_REASON_DEAUTH_LEAVING;
    req.vif_idx = g_aic.vif_idx;
    (void)aic_cmd(AIC_SM_DISCONNECT_REQ, AIC_SM_DISCONNECT_CFM,
                  (uint16_t)AIC_TASK_SM, &req, (uint16_t)sizeof(req),
                  cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS);
}

void aic_disconnect(void)
{
    if (!g_aic.ready) {
        return;
    }
    aic_mm_disconnect();
    aic_wpa_stop();
    g_aic.conn = AIC_CONN_NONE;
    g_aic.ssid[0] = '\0';
    g_aic.psk[0] = '\0';
    g_aic.ap_idx = AIC_STA_IDX_INVALID;
    g_aic.wpa_enabled = false;
    g_aic.connect_open = false;
}

/* ---- key installation (called by the supplicant, never from the RX
 * dispatch -- see the note in AIC8800_Wpa.h) ---- */

bool aic_mm_key_add(uint8_t key_idx, uint8_t sta_idx, const uint8_t *key,
                    uint8_t key_len, uint8_t cipher_suite, bool pairwise)
{
    aic_mm_key_add_req_t req;
    aic_mm_key_add_cfm_t cfm;

    if (!g_aic.ready || key == NULL || key_len == 0u ||
        key_len > AIC_MAC_SEC_KEY_LEN) {
        return false;
    }

    aic_memset(&req, 0u, sizeof(req));
    req.key_idx = key_idx;
    req.sta_idx = sta_idx;
    req.key.length = key_len;
    aic_memcpy(&req.key.array[0], key, key_len); /* raw key material */
    req.cipher_suite = cipher_suite;
    req.inst_nbr = g_aic.vif_idx;
    req.spp = 0u;
    req.pairwise = pairwise ? 1u : 0u;

    aic_memset(&cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_MM_KEY_ADD_REQ, AIC_MM_KEY_ADD_CFM, (uint16_t)AIC_TASK_MM,
                 &req, (uint16_t)sizeof(req), &cfm, (uint16_t)sizeof(cfm),
                 AIC_CMD_TIMEOUT_MS)) {
        return false;
    }
    if (cfm.status != 0u) {
        bring_log_status("MM_KEY_ADD status", cfm.status);
        return false;
    }
    return true;
}

/* ---- asynchronous events ---- */

void aic_handle_connect_ind(const uint8_t *payload, uint16_t len)
{
    uint16_t status;

    if (payload == NULL || len < 12u) {
        return;
    }
    if (g_aic.conn != AIC_CONN_CONNECTING) {
        return; /* a stale indication from an earlier attempt */
    }

    status = aic_rd16(payload);
    if (status != AIC_SM_STATUS_SUCCESS) {
        bring_log_status("connect status", status);
        g_aic.conn = AIC_CONN_FAILED;
        return;
    }

    aic_memcpy(g_aic.bssid, payload + 2u, 6u);
    g_aic.ap_idx = payload[10u];

    if (g_aic.wpa_enabled) {
        /* The handshake starts now: the 300 ms EAPOL-Start retry in
         * aic_wpa_tick() is timed from here, i.e. from association. */
        if (!aic_wpa_start(g_aic.psk, (const uint8_t *)g_aic.ssid,
                           aic_strnlen(g_aic.ssid, DRIVER_WIFI_SSID_MAX + 1u),
                           g_aic.bssid, g_aic.mac)) {
            g_aic.conn = AIC_CONN_FAILED;
            return;
        }
        g_aic.conn = AIC_CONN_4WAY;
        aic_log("[AIC8800] associated; running the 4-way handshake\n");
    } else {
        g_aic.conn = AIC_CONN_ASSOCIATED;
        aic_log("[AIC8800] associated (open network)\n");
    }
}

void aic_handle_disconnect_ind(const uint8_t *payload, uint16_t len)
{
    if (payload == NULL || len < 3u) {
        return;
    }
    if (g_aic.conn == AIC_CONN_NONE) {
        return; /* our own disconnect's indication */
    }
    /* The link went down (the AP dropped us, or the firmware gave up):
     * report a failed connection either way -- is_ready() follows. */
    bring_log_status("disconnected, reason", aic_rd16(payload));
    aic_wpa_stop();
    g_aic.conn = AIC_CONN_FAILED;
}

/* ---- steady-state bookkeeping ---- */

void aic_conn_tick(void)
{
    if (!g_aic.ready) {
        return;
    }

    if (g_aic.scanning &&
        aic_ms_since(g_aic.scan_deadline, AIC_SCAN_WINDOW_MS) >= AIC_SCAN_WINDOW_MS) {
        g_aic.scanning = false;
    }

    switch (g_aic.conn) {
    case AIC_CONN_CONNECTING:
        if (aic_ms_since(g_aic.connect_deadline, AIC_CONNECT_TIMEOUT_MS) >=
            AIC_CONNECT_TIMEOUT_MS) {
            g_aic.conn = AIC_CONN_FAILED;
            aic_log("[AIC8800] association timed out\n");
        }
        break;

    case AIC_CONN_4WAY:
        if (aic_wpa_state() == AIC_WPA_FAILED) {
            g_aic.conn = AIC_CONN_FAILED;
            aic_log("[AIC8800] 4-way handshake failed\n");
        } else if (aic_wpa_keys_installed()) {
            /* Only now is the data path really usable: the network stack
             * sees link-up through is_ready() and starts DHCP. */
            g_aic.conn = AIC_CONN_ASSOCIATED;
            aic_log("[AIC8800] WPA2 keys installed; link is up\n");
        }
        break;

    default:
        break;
    }
}
