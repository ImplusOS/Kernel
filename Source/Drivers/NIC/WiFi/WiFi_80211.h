#pragma once

/*
 * WiFi_80211.h -- IEEE 802.11 wire-format definitions.
 *
 * This header only describes on-air frame layout and well-known constants;
 * it has no dependency on the driver API and performs no hardware access,
 * so it can be reused by any future 802.11 management-plane code (driver,
 * userland scan tooling, etc).
 *
 * This is a *foundation*: it covers what is needed to recognise/build the
 * management frames used during scan and open/PSK association (beacon,
 * probe, auth, assoc) plus RFC 1042 (SNAP) data encapsulation. 802.11n/ac/ax
 * information elements, rate negotiation, and the WPA2 4-way handshake are
 * out of scope here -- see the TODOs in WiFi.c for where they plug in.
 */

#include <stdint.h>

/* ---- Frame Control field (IEEE 802.11-2020, 9.2.4) ---- */
#define IEEE80211_FC_TYPE_MGMT   0x00u
#define IEEE80211_FC_TYPE_CTRL   0x01u
#define IEEE80211_FC_TYPE_DATA   0x02u

#define IEEE80211_FC_STYPE_ASSOC_REQ    0x00u
#define IEEE80211_FC_STYPE_ASSOC_RESP   0x01u
#define IEEE80211_FC_STYPE_REASSOC_REQ  0x02u
#define IEEE80211_FC_STYPE_REASSOC_RESP 0x03u
#define IEEE80211_FC_STYPE_PROBE_REQ    0x04u
#define IEEE80211_FC_STYPE_PROBE_RESP   0x05u
#define IEEE80211_FC_STYPE_BEACON       0x08u
#define IEEE80211_FC_STYPE_DISASSOC     0x0Au
#define IEEE80211_FC_STYPE_AUTH         0x0Bu
#define IEEE80211_FC_STYPE_DEAUTH       0x0Cu
#define IEEE80211_FC_STYPE_ACTION       0x0Du

#define IEEE80211_FC_STYPE_DATA         0x00u
#define IEEE80211_FC_STYPE_QOS_DATA     0x08u
#define IEEE80211_FC_STYPE_QOS_FLAG     0x08u /* set on any QoS-subtype data frame */

#define IEEE80211_FCTL_TYPE(fc)   (((fc) >> 2) & 0x3u)
#define IEEE80211_FCTL_STYPE(fc)  (((fc) >> 4) & 0xFu)
#define IEEE80211_FCTL_TODS       0x0100u
#define IEEE80211_FCTL_FROMDS     0x0200u
#define IEEE80211_FCTL_PROTECTED  0x4000u

/* Generic 3-address MAC header. Present at the start of every management
 * and (non-4-address) data frame. */
typedef struct __attribute__((packed)) {
    uint16_t frame_control;
    uint16_t duration_id;
    uint8_t  addr1[6]; /* RA / DA   */
    uint8_t  addr2[6]; /* TA / SA   */
    uint8_t  addr3[6]; /* BSSID (role depends on ToDS/FromDS) */
    uint16_t seq_control;
} ieee80211_mac_hdr_t;

/* QoS data frames append a 2-byte QoS Control field after the MAC header. */
typedef struct __attribute__((packed)) {
    ieee80211_mac_hdr_t hdr;
    uint16_t             qos_control;
} ieee80211_qos_hdr_t;

/* ---- Management-frame fixed fields (IEs follow immediately after) ---- */
typedef struct __attribute__((packed)) {
    uint64_t timestamp;
    uint16_t beacon_interval;
    uint16_t capability_info;
} ieee80211_beacon_fixed_t;

typedef struct __attribute__((packed)) {
    uint16_t algorithm;
    uint16_t seq_number;
    uint16_t status_code;
    /* optional IEs (e.g. challenge text) follow for shared-key auth */
} ieee80211_auth_fixed_t;

typedef struct __attribute__((packed)) {
    uint16_t capability_info;
    uint16_t listen_interval;
} ieee80211_assoc_req_fixed_t;

typedef struct __attribute__((packed)) {
    uint16_t capability_info;
    uint16_t status_code;
    uint16_t association_id;
} ieee80211_assoc_resp_fixed_t;

#define IEEE80211_AUTH_ALGO_OPEN       0u
#define IEEE80211_AUTH_ALGO_SHARED_KEY 1u
#define IEEE80211_AUTH_SEQ_REQUEST     1u
#define IEEE80211_AUTH_SEQ_RESPONSE    2u
#define IEEE80211_STATUS_SUCCESS       0u

/* ---- Information elements (IEEE 802.11-2020, 9.4.2) ---- */
#define IEEE80211_EID_SSID            0u
#define IEEE80211_EID_SUPP_RATES      1u
#define IEEE80211_EID_DS_PARAMS       3u
#define IEEE80211_EID_TIM             5u
#define IEEE80211_EID_RSN             48u
#define IEEE80211_EID_EXT_SUPP_RATES  50u
#define IEEE80211_EID_VENDOR_SPECIFIC 221u

#define IEEE80211_MAX_SSID_LEN 32u

/* ---- RSN (WPA2/WPA3) suite selectors (IEEE 802.11-2020, 9.4.2.24) ---- */
#define IEEE80211_RSN_CIPHER_TKIP  2u
#define IEEE80211_RSN_CIPHER_CCMP  4u
#define IEEE80211_RSN_AKM_PSK      2u
#define IEEE80211_RSN_AKM_SAE      8u /* WPA3-Personal */

/* ---- Capability information bits (subset) ---- */
#define IEEE80211_CAPINFO_ESS     0x0001u
#define IEEE80211_CAPINFO_IBSS    0x0002u
#define IEEE80211_CAPINFO_PRIVACY 0x0010u

/* ---- RFC 1042 (SNAP) encapsulation used to carry Ethernet payloads over
 * an 802.11 data frame. Layout: DSAP, SSAP, Control, OUI(3), EtherType(2). */
#define IEEE80211_SNAP_DSAP     0xAAu
#define IEEE80211_SNAP_SSAP     0xAAu
#define IEEE80211_SNAP_CTRL     0x03u
#define IEEE80211_SNAP_HDR_LEN  8u
