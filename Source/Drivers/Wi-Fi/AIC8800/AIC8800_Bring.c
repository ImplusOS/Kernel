#include "AIC8800.h"

/*
 * AIC8800_Bring.c -- the fdrv stage: station bring-up, scanning, connect.
 *
 * Once the loader stage has run the firmware, the chip comes back as
 * a69c:8d81 (or a branded id, see AIC8800.c) speaking the full TASK_MM/
 * ME/SCANU/SM message set. probe() has already classified it and the
 * Wi-Fi entry points call aic_bring_step() until `ready` is set; then the
 * steady state takes over. The ladder is the vendor's, in its order --
 * rwnx_cfg80211_init() followed by rwnx_open() for the first interface:
 *
 *   (reboot)     a unit this boot did not load firmware into is sent back
 *                to its ROM first (aic_load_fw's rwnx_send_reboot()), so
 *                every session starts from firmware this driver configured
 *   MM_SET_STACK_START   start the firmware's UMAC stack; says whether the
 *                        unit has a 5 GHz radio
 *   rf init      MM_SET_TXPWR_IDX_LVL (the vendor's default v3 table) and
 *                MM_SET_RF_CALIB with the D80 calibration values
 *   MM_GET_MAC_ADDR      read the efuse MAC (logged; see AIC8800.c)
 *   MM_RESET, MM_VERSION
 *   ME_CONFIG    the station's capabilities (HT), TX lifetime, power save
 *   ME_CHAN_CONFIG       the channel list the firmware may scan and use --
 *                        without it the UMAC has no channels at all
 *   MM_START     start the LMAC
 *   MM_SET_COEX  Wi-Fi/BT coexistence defaults (vendor CONFIG_COEX)
 *   MM_ADD_IF    add the station interface -> vif index
 *
 *   scan         SCANU_START over every channel; SCANU_RESULT_IND per BSS,
 *                SCANU_START_CFM when the sweep is over
 *   connect      SM_CONNECT_REQ; SM_CONNECT_IND reports the association.
 *                For WPA2 the controlled port stays closed (CONTROL_PORT_HOST)
 *                until AIC8800_Wpa.c has installed the keys and this file
 *                has sent ME_SET_CONTROL_PORT_REQ -- exactly what cfg80211's
 *                "authorized" station flag does in the vendor driver.
 *
 * Every message follows the layouts in AIC8800_Protocol.h, which were
 * checked against the vendor's lmac_msg.h compiled on the host.
 */

/* ---- constants ---- */

#define AIC_TXPWR_LVL_V3_ENABLE     1u

/* MM_SET_RF_CALIB, D81 values (rwnx_send_rf_calib_req()). */
#define AIC_RF_CAL_CFG_24G          0x00000F8Fu
#define AIC_RF_CAL_CFG_5G           0x00000F0Fu
#define AIC_RF_CAL_PARAM_ALPHA      0x0C34C008u
#define AIC_RF_CAL_BT_ENABLE        0u
#define AIC_RF_CAL_BT_PARAM         0x00264203u
#define AIC_RF_CAL_XTAL_CAP         0u
#define AIC_RF_CAL_XTAL_CAP_FINE    0u

/* MM_SET_STACK_START for D81 in the vendor's (non-USE_5G) build. */
#define AIC_STACK_SET_VENDOR_INFO   0x20u /* CO_BIT(5) */

/* MM_START: the vendor sends a zeroed PHY config with these two settings
 * (mod_params uapsd_timeout / lp_clk_ppm). */
#define AIC_MM_START_UAPSD_TIMEOUT  300u
#define AIC_MM_START_LP_CLK_ACC     20u

/* ME_CONFIG. HT only: the D80's single stream at 20/40 MHz with short GI,
 * LDPC and RX STBC, which is what rwnx_set_ht_capa() computes for D81
 * (nss = 1). VHT and HE are left off on purpose: a mis-encoded HE
 * capability makes an 802.11ax AP refuse the association, whereas HT is
 * understood by every AP this driver will meet. */
#define AIC_HT_CAP_LDPC             0x0001u
#define AIC_HT_CAP_SUP_WIDTH_20_40  0x0002u
#define AIC_HT_CAP_SGI_20           0x0020u
#define AIC_HT_CAP_SGI_40           0x0040u
#define AIC_HT_CAP_RX_STBC_1        0x0100u
#define AIC_HT_AMPDU_FACTOR_64K     3u
#define AIC_HT_AMPDU_DENSITY_16US   7u
#define AIC_HT_RX_HIGHEST_1SS_40SGI 150u
#define AIC_HT_MCS_TX_DEFINED       0x01u
#define AIC_ME_TX_LIFETIME_MS       1000u /* RWNX_TX_LIFETIME_MS */
#define AIC_PHY_CHNL_BW_40          1u

/* Channel TX power ceiling handed to the firmware (dBm). The vendor uses
 * the regulatory maximum here; 20 dBm is the common 2.4/5 GHz EIRP limit,
 * and the per-rate table from MM_SET_TXPWR_IDX_LVL stays the real cap. */
#define AIC_CHAN_TX_POWER_DBM       20

#define AIC_MM_STA_TYPE             0u /* enum mm_sta_type: MM_STA */
#define AIC_STA_IDX_INVALID         0xFFu /* INVALID_STA_IDX, for ap_idx */
#define AIC_REASON_DEAUTH_LEAVING   3u
#define AIC_UAPSD_QUEUES_VO         1u /* IEEE80211_WMM_IE_STA_QOSINFO_AC_VO */

/* How many times the controlled port is asked to open before the link is
 * declared failed. */
#define AIC_PORT_MAX_ATTEMPTS       5u

/* 2.4 GHz: channels 1..14. 5 GHz: the 28 channels of the vendor's
 * rwnx_channels_5 table (UNII-1/2/2e/3 and 169..177). */
static const uint16_t g_freq_24[AIC_DOMAIN_CHAN_24G_MAX] = {
    2412u, 2417u, 2422u, 2427u, 2432u, 2437u, 2442u,
    2447u, 2452u, 2457u, 2462u, 2467u, 2472u, 2484u,
};
static const uint16_t g_freq_5[AIC_DOMAIN_CHAN_5G_MAX] = {
    5180u, 5200u, 5220u, 5240u, 5260u, 5280u, 5300u, 5320u,
    5500u, 5520u, 5540u, 5560u, 5580u, 5600u, 5620u, 5640u,
    5660u, 5680u, 5700u, 5720u, 5745u, 5765u, 5785u, 5805u,
    5825u, 5845u, 5865u, 5885u,
};

/* Wait for the reply to a request the firmware answers before it starts
 * the work it describes (the association itself is asynchronous: its
 * result is SM_CONNECT_IND). */
#define AIC_CONNECT_CFM_TIMEOUT_MS  1000u

/* 802.11 information elements. */
#define AIC_IE_SSID                 0u
#define AIC_IE_VENDOR               221u
#define AIC_IE_RSN                  48u

/* A beacon/probe response: 24-byte MAC header + 12 bytes of fixed fields
 * (timestamp, interval, capability) + IEs. */
#define AIC_MGMT_HDR_LEN            24u
#define AIC_BEACON_FIXED_LEN        12u
#define AIC_BSSID_OFS               16u /* addr3 */
#define AIC_CAPINFO_OFS             34u
#define AIC_CAPINFO_PRIVACY         0x0010u
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
 * type 1. */
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

static void chan_fill(aic_mac_chan_def_t *chan, uint16_t freq)
{
    chan->freq = freq;
    chan->band = (freq >= 5000u) ? 1u : 0u;
    chan->flags = 0u;
    chan->tx_power = (int8_t)AIC_CHAN_TX_POWER_DBM;
}

/* ---- bring-up steps ---- */

static bool bring_stack_start(void)
{
    aic_mm_set_stack_start_req_t req;
    aic_mm_set_stack_start_cfm_t cfm;

    aic_memset(&req, 0u, sizeof(req));
    req.is_stack_start = 1u;
    req.efuse_valid = 0u;
    req.set_vendor_info = AIC_STACK_SET_VENDOR_INFO;
    req.fwtrace_redir = 0u;
    aic_memset(&cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_MM_SET_STACK_START_REQ, AIC_MM_SET_STACK_START_CFM,
                 (uint16_t)AIC_TASK_MM, &req, (uint16_t)sizeof(req),
                 &cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS)) {
        return false;
    }
    g_aic.band_5g = (cfm.is_5g_support != 0u);
    return true;
}

/* TX power table + RF calibration (rwnx_ic_rf_init() ->
 * aicwf_set_rf_config_8800d80()). The per-rate adjustment and the 2x offset
 * table are only sent when a userconfig file enables them, which the
 * default D80 configuration does not. */
static bool bring_rf_init(void)
{
    aic_mm_set_txpwr_lvl_req_t txpwr;
    aic_mm_set_rf_calib_req_t calib;
    uint8_t cfm[16];

    /* The vendor's built-in userconfig defaults (rwnx_platform.c's
     * userconfig_info.txpwr_lvl_v3). 0x80 in the 5 GHz 11a entries is the
     * vendor's "not applicable" marker for the CCK slots. */
    static const aic_txpwr_lvl_conf_v3_t txpwr_defaults = {
        .enable = AIC_TXPWR_LVL_V3_ENABLE,
        .pwrlvl_11b_11ag_2g4 = { 20, 20, 20, 20, 20, 20, 20, 20,
                                 18, 18, 16, 16 },
        .pwrlvl_11n_11ac_2g4 = { 20, 20, 20, 20, 18, 18, 16, 16, 16, 16 },
        .pwrlvl_11ax_2g4 = { 20, 20, 20, 20, 18, 18, 16, 16, 16, 16, 15, 15 },
        .pwrlvl_11a_5g = { -128, -128, -128, -128, 20, 20, 20, 20,
                           18, 18, 16, 16 },
        .pwrlvl_11n_11ac_5g = { 20, 20, 20, 20, 18, 18, 16, 16, 16, 15 },
        .pwrlvl_11ax_5g = { 20, 20, 20, 20, 18, 18, 16, 16, 16, 15, 14, 14 },
    };

    aic_memset(&txpwr, 0u, sizeof(txpwr));
    aic_memcpy(txpwr.raw, &txpwr_defaults, sizeof(txpwr_defaults));
    aic_memset(cfm, 0u, sizeof(cfm));
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
    aic_memset(cfm, 0u, sizeof(cfm));
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
    uint8_t cfm[32];

    aic_memset(cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_MM_VERSION_REQ, AIC_MM_VERSION_CFM,
                 (uint16_t)AIC_TASK_MM, NULL, 0u, cfm, (uint16_t)sizeof(cfm),
                 AIC_CMD_TIMEOUT_MS)) {
        return false;
    }
    /* The first word of the reply is the LMAC version. */
    return true;
}

static bool bring_me_config(void)
{
    aic_me_config_req_t req;
    uint8_t cfm[4];
    uint16_t ht_cap = (uint16_t)(AIC_HT_CAP_LDPC | AIC_HT_CAP_SUP_WIDTH_20_40 |
                                 AIC_HT_CAP_SGI_20 | AIC_HT_CAP_SGI_40 |
                                 AIC_HT_CAP_RX_STBC_1);

    aic_memset(&req, 0u, sizeof(req));
    req.ht_cap.ht_capa_info = ht_cap;
    req.ht_cap.a_mpdu_param = (uint8_t)(AIC_HT_AMPDU_FACTOR_64K |
                                        (AIC_HT_AMPDU_DENSITY_16US << 2));
    /* struct ieee80211_mcs_info: rx_mask[10], le16 rx_highest, tx_params */
    req.ht_cap.mcs_rate[0] = 0xFFu; /* MCS 0..7 */
    req.ht_cap.mcs_rate[4] = 0x01u; /* MCS 32 (40 MHz duplicate) */
    aic_wr16(&req.ht_cap.mcs_rate[10], AIC_HT_RX_HIGHEST_1SS_40SGI);
    req.ht_cap.mcs_rate[12] = AIC_HT_MCS_TX_DEFINED;
    req.ht_supp = 1u;
    req.vht_supp = 0u;
    req.he_supp = 0u;
    req.he_ul_on = 0u;
    /* Power save off: this host keeps no PS state of its own, and an awake
     * station is what makes DHCP and the handshake reliable. */
    req.ps_on = 0u;
    req.dpsm = 0u;
    req.ant_div_on = 0u;
    req.tx_lft = AIC_ME_TX_LIFETIME_MS;
    req.phy_bw_max = AIC_PHY_CHNL_BW_40;

    aic_memset(cfm, 0u, sizeof(cfm));
    return aic_cmd(AIC_ME_CONFIG_REQ, AIC_ME_CONFIG_CFM, (uint16_t)AIC_TASK_ME,
                   &req, (uint16_t)sizeof(req), cfm, (uint16_t)sizeof(cfm),
                   AIC_CMD_TIMEOUT_MS);
}

static bool bring_me_chan_config(void)
{
    aic_me_chan_config_req_t req;
    uint8_t cfm[4];

    aic_memset(&req, 0u, sizeof(req));
    for (uint32_t i = 0u; i < AIC_DOMAIN_CHAN_24G_MAX; ++i) {
        chan_fill(&req.chan2G4[i], g_freq_24[i]);
    }
    req.chan2G4_cnt = (uint8_t)AIC_DOMAIN_CHAN_24G_MAX;
    if (g_aic.band_5g) {
        for (uint32_t i = 0u; i < AIC_DOMAIN_CHAN_5G_MAX; ++i) {
            chan_fill(&req.chan5G[i], g_freq_5[i]);
        }
        req.chan5G_cnt = (uint8_t)AIC_DOMAIN_CHAN_5G_MAX;
    }

    aic_memset(cfm, 0u, sizeof(cfm));
    return aic_cmd(AIC_ME_CHAN_CONFIG_REQ, AIC_ME_CHAN_CONFIG_CFM,
                   (uint16_t)AIC_TASK_ME, &req, (uint16_t)sizeof(req),
                   cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS);
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

/* rwnx_send_coex_req(hw, 0, 1) from rwnx_open(). Best effort: a firmware
 * that does not answer it is still usable. */
static void bring_mm_coex(void)
{
    aic_mm_set_coex_req_t req;
    uint8_t cfm[4];

    aic_memset(&req, 0u, sizeof(req));
    req.bt_on = 1u;
    req.disable_coexnull = 0u;
    req.enable_nullcts = 1u;
    req.enable_periodic_timer = 0u;
    req.coex_timeslot_set = 0u;
    aic_memset(cfm, 0u, sizeof(cfm));
    (void)aic_cmd(AIC_MM_SET_COEX_REQ, AIC_MM_SET_COEX_CFM,
                  (uint16_t)AIC_TASK_MM, &req, (uint16_t)sizeof(req),
                  cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS);
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
    if (cfm.status != 0u) {
        return false;
    }
    g_aic.vif_idx = cfm.inst_nbr;
    return true;
}

static void bring_fail(const char *what)
{
    g_aic.bring_last_error = what;

    ++g_aic.bring_attempts;
    if (g_aic.bring_attempts >= AIC_MAX_ATTEMPTS) {
        g_aic.bring_phase = AIC_BRING_FAILED;
    } else {
        g_aic.bring_phase = AIC_BRING_IDLE; /* retry the ladder later */
    }
}

void aic_bring_reset(void)
{
    g_aic.bring_phase = AIC_BRING_IDLE;
    g_aic.bring_attempts = 0u;
    g_aic.bring_last_attempt = 0u;
    g_aic.reboot_ticks = 0u;
    g_aic.ready = false;
    g_aic.band_5g = false;
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

        if (g_aic.bring_attempts == 0u && !aic_fw_loaded_this_boot()) {
            /* Firmware of unknown provenance (a warm reboot, another OS):
             * reboot the chip into its ROM so the loader stage configures a
             * fresh image, as the vendor's aic_load_fw does. */
            if (aic_fw_send_reboot()) {
                g_aic.reboot_ticks = aic_now_ticks();
                g_aic.bring_phase = AIC_BRING_REBOOT_WAIT;
                return;
            }
        }
        g_aic.bring_phase = AIC_BRING_STACK_START;
    }

    switch (g_aic.bring_phase) {
    case AIC_BRING_REBOOT_WAIT:
        /* The re-enumeration as 8d80 arrives as a new probe(), which resets
         * all of this. Still here after the window: the unit did not
         * reboot, so bring up whatever is running. */
        if (aic_ms_since(g_aic.reboot_ticks, AIC_REBOOT_WAIT_MS) >=
            AIC_REBOOT_WAIT_MS) {
            g_aic.bring_phase = AIC_BRING_STACK_START;
        }
        break;

    case AIC_BRING_STACK_START:
        if (!bring_stack_start()) {
            bring_fail("stack start");
            return;
        }
        g_aic.bring_phase = AIC_BRING_RF;
        break;

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
        g_aic.bring_phase = AIC_BRING_ME_CONFIG;
        break;

    case AIC_BRING_ME_CONFIG:
        if (!bring_me_config()) {
            bring_fail("me config");
            return;
        }
        g_aic.bring_phase = AIC_BRING_CHAN_CONFIG;
        break;

    case AIC_BRING_CHAN_CONFIG:
        if (!bring_me_chan_config()) {
            bring_fail("channel config");
            return;
        }
        g_aic.bring_phase = AIC_BRING_START;
        break;

    case AIC_BRING_START:
        if (!bring_mm_start()) {
            bring_fail("mm start");
            return;
        }
        g_aic.bring_phase = AIC_BRING_COEX;
        break;

    case AIC_BRING_COEX:
        bring_mm_coex();
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
        break;

    default:
        break;
    }
}

/* ---- scanning ---- */

bool aic_scan_start(void)
{
    aic_scanu_start_req_t req;
    uint32_t n = 0u;

    if (!g_aic.ready) {
        return false;
    }
    if (g_aic.conn == AIC_CONN_CONNECTING || g_aic.conn == AIC_CONN_4WAY) {
        return false; /* the firmware cannot scan while it associates */
    }

    aic_memset(&req, 0u, sizeof(req));
    for (uint32_t i = 0u; i < AIC_DOMAIN_CHAN_24G_MAX; ++i) {
        chan_fill(&req.chan[n++], g_freq_24[i]);
    }
    if (g_aic.band_5g) {
        for (uint32_t i = 0u; i < AIC_DOMAIN_CHAN_5G_MAX; ++i) {
            chan_fill(&req.chan[n++], g_freq_5[i]);
        }
    }
    req.chan_cnt = (uint8_t)n;
    /* One wildcard (zero-length) SSID: an active scan that reports every
     * network, which is what cfg80211 asks for. */
    req.ssid_cnt = 1u;
    req.ssid[0].length = 0u;
    /* The BSSID filter must be the broadcast address, or the firmware only
     * reports a BSS whose BSSID equals it. */
    aic_memset(&req.bssid, 0xFFu, sizeof(req.bssid));
    req.vif_idx = g_aic.vif_idx;
    req.no_cck = 0u;
    req.add_ies = 0u;
    req.add_ie_len = 0u;

    /* A new sweep replaces the previous results. */
    aic_memset(g_aic.scan_results, 0u, sizeof(g_aic.scan_results));
    aic_memset(g_aic.scan_freq, 0u, sizeof(g_aic.scan_freq));
    aic_memset(g_aic.scan_rsn_group, 0u, sizeof(g_aic.scan_rsn_group));
    aic_memset(g_aic.scan_rsn_ok, 0u, sizeof(g_aic.scan_rsn_ok));
    g_aic.scan_result_count = 0u;
    g_aic.scanning = true;
    g_aic.scan_deadline = aic_now_ticks();

    /* The vendor waits for SCANU_START_CFM_ADDTIONAL, the firmware's
     * immediate acknowledgement; SCANU_START_CFM follows when the sweep is
     * over (aic_handle_scan_done()). Older firmware may skip the early
     * acknowledgement: the results still arrive, and the window below ends
     * the scan either way. */
    (void)aic_cmd(AIC_SCANU_START_REQ, AIC_SCANU_START_CFM_ADDTIONAL,
                  (uint16_t)AIC_TASK_SCANU, &req, (uint16_t)sizeof(req),
                  NULL, 0u, AIC_CMD_TIMEOUT_MS);
    return true;
}

void aic_handle_scan_done(void)
{
    if (g_aic.scanning) {
        g_aic.scanning = false;
    }
}

uint32_t aic_scan_results_copy(driver_wifi_scan_result_t *out, uint32_t max_count)
{
    uint32_t count = g_aic.scan_result_count;

    if (out == NULL || max_count == 0u) {
        return count;
    }
    /* Strongest first, so a caller with a smaller table (the panel keeps
     * 24) gets the networks it can most likely join. */
    uint8_t order[AIC_MAX_SCAN_RESULTS];
    for (uint32_t i = 0u; i < count; ++i) {
        uint32_t j = i;
        while (j > 0u && g_aic.scan_results[order[j - 1u]].rssi_dbm <
                             g_aic.scan_results[i].rssi_dbm) {
            order[j] = order[j - 1u];
            --j;
        }
        order[j] = (uint8_t)i;
    }
    if (count > max_count) {
        count = max_count;
    }
    for (uint32_t i = 0u; i < count; ++i) {
        aic_memcpy(&out[i], &g_aic.scan_results[order[i]],
                   (uint32_t)sizeof(driver_wifi_scan_result_t));
    }
    return count;
}

void aic_handle_scan_result(const uint8_t *payload, uint16_t len)
{
    const uint8_t *frame;
    const uint8_t *bssid;
    const uint8_t *ies;
    const uint8_t *ssid_ie;
    const uint8_t *rsn_ie;
    uint32_t ies_len;
    uint32_t ssid_len = 0u;
    uint32_t rsn_len = 0u;
    uint32_t frame_len;
    uint16_t center_freq;
    uint16_t capinfo;
    uint32_t security = (uint32_t)DRIVER_WIFI_SECURITY_OPEN;
    uint8_t rsn_group = AIC_RSN_GROUP_NONE;
    bool rsn_ok = false;
    int8_t rssi;

    if (payload == NULL ||
        len < AIC_SCANU_RESULT_FRAME_OFS + AIC_IE_OFS) {
        return;
    }

    frame_len = aic_rd16(payload);
    center_freq = aic_rd16(payload + 4u);
    rssi = (int8_t)payload[9u];

    frame = payload + AIC_SCANU_RESULT_FRAME_OFS;
    if (frame_len > (uint32_t)len - AIC_SCANU_RESULT_FRAME_OFS) {
        frame_len = (uint32_t)len - AIC_SCANU_RESULT_FRAME_OFS;
    }
    if (frame_len < AIC_IE_OFS) {
        return;
    }
    /* Beacon (0x80) or probe response (0x50) only. */
    if (frame[0] != 0x80u && frame[0] != 0x50u) {
        return;
    }

    bssid = frame + AIC_BSSID_OFS;
    capinfo = aic_rd16(frame + AIC_CAPINFO_OFS);
    ies = frame + AIC_IE_OFS;
    ies_len = frame_len - AIC_IE_OFS;

    ssid_ie = ie_find(ies, ies_len, AIC_IE_SSID, &ssid_len);
    if (ssid_ie == NULL || ssid_len == 0u || ssid_ie[2] == 0u) {
        return; /* hidden SSID: nothing the panel could show */
    }
    if (ssid_len > DRIVER_WIFI_SSID_MAX) {
        ssid_len = DRIVER_WIFI_SSID_MAX;
    }
    rsn_ie = ie_find(ies, ies_len, AIC_IE_RSN, &rsn_len);
    if (rsn_ie != NULL) {
        security = (uint32_t)DRIVER_WIFI_SECURITY_WPA_PSK;
        rsn_ok = aic_wpa_rsn_usable(rsn_ie, rsn_len + 2u, &rsn_group);
    } else if (ie_has_wpa1(ies, ies_len) ||
               (capinfo & AIC_CAPINFO_PRIVACY) != 0u) {
        /* WPA1-only or WEP: shown as secured, but not joinable here. */
        security = (uint32_t)DRIVER_WIFI_SECURITY_WPA_PSK;
    }

    /* One entry per BSSID: a later report of the same AP only upgrades the
     * signal strength. */
    for (uint32_t i = 0u; i < g_aic.scan_result_count; ++i) {
        if (aic_memequal(g_aic.scan_results[i].bssid, bssid, 6u)) {
            if (rssi > g_aic.scan_results[i].rssi_dbm) {
                g_aic.scan_results[i].rssi_dbm = rssi;
            }
            return;
        }
    }
    uint32_t slot;
    if (g_aic.scan_result_count < AIC_MAX_SCAN_RESULTS) {
        slot = g_aic.scan_result_count++;
    } else {
        /* Full: keep the strongest, replacing the weakest entry if this
         * one beats it. */
        slot = 0u;
        for (uint32_t i = 1u; i < AIC_MAX_SCAN_RESULTS; ++i) {
            if (g_aic.scan_results[i].rssi_dbm <
                g_aic.scan_results[slot].rssi_dbm) {
                slot = i;
            }
        }
        if (rssi <= g_aic.scan_results[slot].rssi_dbm) {
            return;
        }
    }

    driver_wifi_scan_result_t *out = &g_aic.scan_results[slot];
    aic_memset(out, 0u, sizeof(*out));
    aic_memcpy(out->ssid, ssid_ie + 2u, ssid_len);
    aic_memcpy(out->bssid, bssid, 6u);
    out->rssi_dbm = rssi;
    out->security = security;
    g_aic.scan_freq[slot] = center_freq;
    g_aic.scan_rsn_group[slot] = rsn_group;
    g_aic.scan_rsn_ok[slot] = rsn_ok ? 1u : 0u;
}

/* ---- connect / disconnect ---- */

/* Find the AP the caller asked for in the last scan's results: the Wi-Fi
 * panel hands us an SSID, and the connect request needs the BSSID and the
 * channel to aim at. The strongest BSS of that name wins. Returns the
 * result index or -1. */
static int32_t scan_find_ssid(const char *ssid, uint32_t ssid_len,
                              bool want_secure)
{
    int32_t best = -1;

    for (uint32_t i = 0u; i < g_aic.scan_result_count; ++i) {
        const char *cand = g_aic.scan_results[i].ssid;
        bool secure = g_aic.scan_results[i].security !=
                      (uint32_t)DRIVER_WIFI_SECURITY_OPEN;
        if (aic_strnlen(cand, DRIVER_WIFI_SSID_MAX + 1u) != ssid_len ||
            !aic_memequal(cand, ssid, ssid_len) || secure != want_secure) {
            continue;
        }
        if (best < 0 || g_aic.scan_results[i].rssi_dbm >
                            g_aic.scan_results[best].rssi_dbm) {
            best = (int32_t)i;
        }
    }
    return best;
}

bool aic_connect(const char *ssid, const char *psk)
{
    aic_sm_connect_req_t req;
    uint8_t cfm[4];
    uint32_t ssid_len;
    uint32_t psk_len = 0u;
    int32_t index;

    if (!g_aic.ready || ssid == NULL) {
        return false;
    }
    if (psk != NULL && psk[0] == '\0') {
        psk = NULL; /* an empty passphrase means an open network */
    }
    ssid_len = aic_strnlen(ssid, DRIVER_WIFI_SSID_MAX + 1u);
    if (ssid_len == 0u || ssid_len > DRIVER_WIFI_SSID_MAX) {
        return false;
    }
    if (psk != NULL) {
        psk_len = aic_strnlen(psk, (uint32_t)sizeof(g_aic.psk));
        if (psk_len < 8u || psk_len > 63u) {
            return false;
        }
    }

    index = scan_find_ssid(ssid, ssid_len, psk != NULL);
    if (index < 0) {
        return false;
    }
    if (psk != NULL && g_aic.scan_rsn_ok[index] == 0u) {
        return false;
    }

    /* A connection that is already up (or still running) is torn down
     * first: the firmware will not accept a second connect on the same
     * interface. */
    if (g_aic.conn != AIC_CONN_NONE && g_aic.conn != AIC_CONN_FAILED) {
        aic_disconnect();
    }
    aic_wpa_stop();
    g_aic.scanning = false;

    aic_memset(&req, 0u, sizeof(req));
    req.ssid.length = (uint8_t)ssid_len;
    aic_memcpy(req.ssid.array, ssid, ssid_len);
    aic_memcpy(&req.bssid, g_aic.scan_results[index].bssid, 6u);
    chan_fill(&req.chan, g_aic.scan_freq[index]);
    req.listen_interval = 0u; /* rwnx_mod_params.listen_itv */
    req.dont_wait_bcmc = 0u;  /* !listen_bcmc */
    req.auth_type = 0u;       /* WLAN_AUTH_OPEN: WPA2-PSK authenticates in RSN */
    req.uapsd_queues = AIC_UAPSD_QUEUES_VO;
    req.vif_idx = g_aic.vif_idx;
    req.flags = 0u;

    if (psk != NULL) {
        uint32_t ie_len = 0u;
        const uint8_t *ie;

        aic_wpa_set_group_cipher(g_aic.scan_rsn_group[index]);
        ie = aic_wpa_rsn_ie(&ie_len);
        if (ie == NULL || ie_len == 0u ||
            ie_len > (uint32_t)sizeof(req.ie_buf)) {
            return false;
        }
        aic_memcpy(req.ie_buf, ie, ie_len);
        req.ie_len = (uint16_t)ie_len;
        /* The host runs the 802.1X port: the firmware passes EAPOL and
         * holds everything else until ME_SET_CONTROL_PORT_REQ opens it.
         * (cfg80211's crypto.control_port for WPA, as the vendor maps it.) */
        req.flags = AIC_SM_CONTROL_PORT_HOST | AIC_SM_WPA_WPA2_IN_USE;
        req.ctrl_port_ethertype = 0x8E88u; /* htons(0x888e) */
    }

    /* Remember the target *before* the request goes out: the firmware can
     * report the association (SM_CONNECT_IND) before this call returns. */
    aic_memcpy(g_aic.ssid, ssid, ssid_len);
    g_aic.ssid[ssid_len] = '\0';
    if (psk != NULL) {
        aic_memcpy(g_aic.psk, psk, psk_len);
        g_aic.psk[psk_len] = '\0';
    } else {
        g_aic.psk[0] = '\0';
    }
    aic_memcpy(g_aic.bssid, g_aic.scan_results[index].bssid, 6u);
    g_aic.center_freq = g_aic.scan_freq[index];
    g_aic.ap_idx = AIC_STA_IDX_INVALID;
    g_aic.wpa_enabled = (psk != NULL);
    g_aic.connect_open = (psk == NULL);
    g_aic.port_opened = false;
    g_aic.port_attempts = 0u;
    g_aic.conn = AIC_CONN_CONNECTING;
    g_aic.connect_deadline = aic_now_ticks();

    aic_memset(cfm, 0u, sizeof(cfm));
    if (!aic_cmd(AIC_SM_CONNECT_REQ, AIC_SM_CONNECT_CFM,
                 (uint16_t)AIC_TASK_SM, &req, (uint16_t)sizeof(req),
                 cfm, (uint16_t)sizeof(cfm), AIC_CONNECT_CFM_TIMEOUT_MS) ||
        cfm[0] != AIC_SM_STATUS_SUCCESS) {
        g_aic.conn = AIC_CONN_FAILED;
        return false;
    }

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
    /* Mark the link down first: the SM_DISCONNECT_IND the request below
     * triggers then reads as our own. */
    aic_conn_state_t was = g_aic.conn;
    g_aic.conn = AIC_CONN_NONE;
    if (was != AIC_CONN_NONE && was != AIC_CONN_FAILED) {
        aic_mm_disconnect();
    }
    aic_wpa_stop();
    g_aic.ssid[0] = '\0';
    aic_memset(g_aic.psk, 0u, sizeof(g_aic.psk));
    g_aic.ap_idx = AIC_STA_IDX_INVALID;
    g_aic.wpa_enabled = false;
    g_aic.connect_open = false;
    g_aic.port_opened = false;
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
    /* rwnx_send_key_add(): the key index only matters for a default
     * (group) key; a pairwise key is addressed by its station. */
    req.key_idx = (sta_idx == AIC_STA_IDX_INVALID) ? key_idx : 0u;
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
        aic_memset(&req, 0u, sizeof(req));
        return false;
    }
    aic_memset(&req, 0u, sizeof(req));
    if (cfm.status != 0u) {
        return false;
    }
    return true;
}

/* ME_SET_CONTROL_PORT_REQ(open): the vendor sends it when cfg80211 marks
 * the AP's station authorized, i.e. once wpa_supplicant installed the
 * keys. Until then the firmware drops every non-EAPOL data frame. */
static bool aic_open_control_port(void)
{
    aic_me_set_control_port_req_t req;
    uint8_t cfm[4];

    aic_memset(&req, 0u, sizeof(req));
    req.sta_idx = g_aic.ap_idx;
    req.control_port_open = 1u;
    aic_memset(cfm, 0u, sizeof(cfm));
    return aic_cmd(AIC_ME_SET_CONTROL_PORT_REQ, AIC_ME_SET_CONTROL_PORT_CFM,
                   (uint16_t)AIC_TASK_ME, &req, (uint16_t)sizeof(req),
                   cfm, (uint16_t)sizeof(cfm), AIC_CMD_TIMEOUT_MS);
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
        g_aic.conn = AIC_CONN_FAILED;
        return;
    }

    aic_memcpy(g_aic.bssid, payload + 2u, 6u);
    g_aic.ap_idx = payload[10u];

    if (g_aic.wpa_enabled) {
        /* The handshake starts now: the EAPOL-Start retry in
         * aic_wpa_tick() is timed from here, i.e. from association. */
        if (!aic_wpa_start(g_aic.psk, (const uint8_t *)g_aic.ssid,
                           aic_strnlen(g_aic.ssid, DRIVER_WIFI_SSID_MAX + 1u),
                           g_aic.bssid, g_aic.mac)) {
            g_aic.conn = AIC_CONN_FAILED;
            return;
        }
        g_aic.conn = AIC_CONN_4WAY;
    } else {
        g_aic.conn = AIC_CONN_ASSOCIATED;
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
    aic_wpa_stop();
    g_aic.port_opened = false;
    g_aic.conn = AIC_CONN_FAILED;
}

/* ---- steady-state bookkeeping ---- */

void aic_conn_tick(void)
{
    if (!g_aic.ready) {
        return;
    }

    /* SCANU_START_CFM normally ends the scan; the window is the fallback. */
    if (g_aic.scanning &&
        aic_ms_since(g_aic.scan_deadline, AIC_SCAN_WINDOW_MS) >=
            AIC_SCAN_WINDOW_MS * 3u) {
        g_aic.scanning = false;
    }

    switch (g_aic.conn) {
    case AIC_CONN_CONNECTING:
        if (aic_ms_since(g_aic.connect_deadline, AIC_CONNECT_TIMEOUT_MS) >=
            AIC_CONNECT_TIMEOUT_MS) {
            g_aic.conn = AIC_CONN_FAILED;
            aic_mm_disconnect();
        }
        break;

    case AIC_CONN_4WAY:
        if (aic_wpa_state() == AIC_WPA_FAILED) {
            g_aic.conn = AIC_CONN_FAILED;
            aic_mm_disconnect();
        } else if (aic_wpa_keys_installed()) {
            /* Keys are in; open the controlled port, after which (and only
             * after which) data flows and DHCP may start. */
            if (aic_open_control_port()) {
                g_aic.port_opened = true;
                g_aic.conn = AIC_CONN_ASSOCIATED;
            } else if (++g_aic.port_attempts >= AIC_PORT_MAX_ATTEMPTS) {
                g_aic.conn = AIC_CONN_FAILED;
                aic_mm_disconnect();
            }
        }
        break;

    case AIC_CONN_ASSOCIATED:
        if (g_aic.wpa_enabled && aic_wpa_state() == AIC_WPA_FAILED) {
            /* A rekey failed: the AP will drop us anyway. */
            g_aic.conn = AIC_CONN_FAILED;
            aic_mm_disconnect();
        }
        break;

    default:
        break;
    }
}
