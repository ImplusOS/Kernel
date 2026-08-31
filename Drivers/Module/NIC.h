#pragma once

#include "Drivers/Module/DriverBinary.h"

#include <stdbool.h>
#include <stdint.h>

typedef void (*nic_rx_callback_t)(const uint8_t *frame, uint16_t frame_len);

bool nic_init(void);
bool nic_is_ready(void);
uint16_t nic_mtu(void);
void nic_get_mac(uint8_t mac_out[6]);

bool nic_send_frame(const uint8_t *frame, uint16_t frame_len);
void nic_poll(void);

void nic_set_rx_callback(nic_rx_callback_t cb);

void nic_schedule_poll(void);
bool nic_check_poll(void);

/* ---- Wi-Fi management plane (see driver_nic_t in DriverBinary.h) ----
 * Every entry point here returns a harmless "unsupported" value (false/0)
 * when no active NIC exposes the Wi-Fi vtable -- either no NIC is active
 * yet, or the active one (e.g. a wired VirtIONet/I219V card) simply isn't
 * Wi-Fi. Callers (NicManager.c, then the syscall layer) don't need to know
 * which case they're in. */
bool nic_wifi_scan_start(void);
uint32_t nic_wifi_get_scan_results(driver_wifi_scan_result_t *out, uint32_t max_count);
bool nic_wifi_connect(const char *ssid, const char *psk);
void nic_wifi_disconnect(void);
void nic_wifi_get_status(driver_wifi_status_t *out_status);
