#pragma once

#include <stdbool.h>
#include <stdint.h>

#define WIFI_MAX_SSID_LEN     32u
#define WIFI_MAX_PSK_LEN      64u
#define WIFI_MAX_SCAN_RESULTS 32u

typedef enum {
    WIFI_LINK_DOWN = 0,       /* radio present but not associated to any BSS */
    WIFI_LINK_SCANNING,
    WIFI_LINK_AUTHENTICATING,
    WIFI_LINK_ASSOCIATING,
    WIFI_LINK_ASSOCIATED,     /* usable as a NIC: send/poll pass real traffic */
    WIFI_LINK_DISCONNECTING,
} wifi_link_state_t;

typedef enum {
    WIFI_SECURITY_OPEN = 0,
    WIFI_SECURITY_WEP,
    WIFI_SECURITY_WPA_PSK,
    WIFI_SECURITY_WPA2_PSK,
    WIFI_SECURITY_WPA3_SAE,
    WIFI_SECURITY_UNKNOWN,
} wifi_security_t;

typedef struct {
    uint8_t         bssid[6];
    char            ssid[WIFI_MAX_SSID_LEN + 1u];
    uint8_t         ssid_len;
    uint8_t         channel;
    int8_t          rssi_dbm;
    wifi_security_t security;
} wifi_scan_result_t;

typedef void (*wifi_rx_callback_t)(const uint8_t *frame, uint16_t frame_len);

/* --- NIC-compatible surface ---
 * Mirrors driver_nic_t (Drivers/Module/DriverBinary.h) so this driver can
 * register itself as a DEVICE_TYPE_NIC device and be picked up
 * transparently by NicManager / the network stack, exactly like a wired
 * NIC. send_frame() and poll() only move real traffic once
 * wifi_get_link_state() == WIFI_LINK_ASSOCIATED.
 */
bool wifi_init(void);
bool wifi_is_ready(void);
uint16_t wifi_mtu(void);
void wifi_get_mac(uint8_t mac_out[6]);
bool wifi_send(const uint8_t *frame, uint16_t frame_len);
void wifi_poll(void);
void wifi_set_rx_callback(wifi_rx_callback_t cb);

/* --- 802.11 management plane (foundation) ---
 * These drive the link state machine. The current build implements the
 * state transitions, the scan-result table, and 802.11 <-> Ethernet frame
 * (de)capsulation in full; only the actual over-the-air exchange (probe/
 * auth/assoc transmission, channel stepping, the WPA2 4-way handshake, and
 * the hardware TX/RX descriptor rings) are left as TODOs in WiFi.c for the
 * target chipset's command/firmware interface.
 */
bool wifi_scan_start(void);
bool wifi_scan_is_active(void);
uint32_t wifi_scan_get_results(wifi_scan_result_t *out_results, uint32_t max_results);

bool wifi_connect(const char *ssid, const char *psk);
void wifi_disconnect(void);

wifi_link_state_t wifi_get_link_state(void);
int8_t wifi_get_rssi(void);
bool wifi_get_ssid(char *out_ssid, uint8_t *out_len);
