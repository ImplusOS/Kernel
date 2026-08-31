#pragma once

#include "Drivers/Module/DriverBinary.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * AX900.h -- driver for the UGREEN AX900 USB Wi-Fi 6 adapter (AICSemi
 * AIC8800D80 chipset). See AX900_Protocol.h for source citations and the
 * on-wire protocol this implementation is built from.
 *
 * This is a *separate* driver from the generic 802.11 SoftMAC foundation in
 * Kernel/Drivers/NIC/WiFi/: the AX900 is FullMAC (firmware does
 * 802.11 framing/timing/handshake; the host only exchanges high-level
 * commands), a fundamentally different driver shape than the raw-802.11
 * SoftMAC model WiFi.c is built around.
 *
 * Despite being a USB device, this is a standalone driver module (its own
 * .ELF, kind = DEVICE_TYPE_NIC) -- *not* compiled into USB_Driver.ELF the
 * way HID/MassStorage still are. It binds to a USB device dynamically via
 * BusRegistry (Kernel/Drivers/Module/BusRegistry.h): this file declares a
 * driver_bus_match_t table and a probe()/remove() pair in its
 * driver_module_descriptor_t, and USB_Main.c reports every vendor-specific
 * interface it enumerates via g_api->bus.report_device() instead of
 * dispatching to this module directly (it has no way to -- separate .ELF,
 * separate address space).
 */

/* ---- module-level init (called once from driver_module_init(), AX900.c) ---- */
void ax900_init(void);

/* Drains pending bulk IN data and drives the bring-up/command state
 * machine. Reachable once this module is the active NIC (Drivers/Module/
 * NIC.c calls driver_nic_t.poll periodically) -- no interrupt-driven
 * RX path exists yet. */
void ax900_poll(void);

/* ---- NIC-shaped surface (driver_nic_t, Drivers/Module/DriverBinary.h) ---- */
typedef void (*ax900_rx_callback_t)(const uint8_t *frame, uint16_t frame_len);

bool ax900_is_ready(void);       /* true once associated (mirrors WIFI_LINK_ASSOCIATED) */
uint16_t ax900_mtu(void);
void ax900_get_mac(uint8_t mac_out[6]);
bool ax900_send(const uint8_t *frame, uint16_t frame_len);
void ax900_set_rx_callback(ax900_rx_callback_t cb);

/* ---- Wi-Fi management plane --------------------------------------------
 * Builds and sends the real SCANU_START_REQ/SM_CONNECT_REQ messages
 * (AX900_Protocol.h), but everything downstream of "firmware must be
 * running" depends on ax900_download_firmware() (see AX900.c) succeeding.
 * That reads real bytes via g_api->fs (driver_binary_t API 2.2) from
 * /Kernel/Driver/Firmware/AX900/ and pushes them to the chip -- the read
 * path is real, but AICSemi's firmware .bin files are proprietary and were
 * never available to embed in this repository, so find_file() fails and
 * these calls still validate their arguments, build correctly-framed
 * messages, and then fail at the "send to un-booted firmware" step.
 */
bool ax900_scan_start(void);
bool ax900_connect(const char *ssid, const char *psk);
void ax900_disconnect(void);

/* Snapshot of AP(s) seen since the last ax900_scan_start(); results
 * accumulate as SCANU_RESULT_IND messages arrive in ax900_poll() and are
 * cleared at the start of the next scan. Returns the number of entries
 * written (<= max_count). */
uint32_t ax900_get_scan_results(driver_wifi_scan_result_t *out, uint32_t max_count);
void ax900_get_wifi_status(driver_wifi_status_t *out_status);
