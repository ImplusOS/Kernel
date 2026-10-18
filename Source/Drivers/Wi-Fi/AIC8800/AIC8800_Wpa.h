#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * AIC8800_Wpa.h -- in-driver WPA2-PSK supplicant.
 *
 * The AIC8800D80 firmware is full-MAC: it associates, but it does not do
 * the RSN 4-way handshake, so the host has to. Because a driver module
 * cannot link against a userland supplicant (there is no wpa_supplicant
 * in this repository either), this driver speaks the handshake itself.
 *
 * The implementation follows wpa_supplicant 2.10's IEEE 802.11i/RSN
 * state machine (hostap): the exact EAPOL-Key layout, MIC coverage
 * (the whole EAPOL frame with the MIC field zeroed, not the "802.11i
 * D10" subset), the PTK derivation, and the AES key-unwrap of message
 * 3's key data.
 *
 * Interface with the rest of the driver:
 *   - aic_wpa_recv() is called from the RX dispatch with a complete
 *     802.3 frame; it consumes EAPOL frames and returns true.
 *   - It transmits its own message 2 / 4 frames through aic_tx_eth().
 *   - Any key installation it needs is *deferred* to aic_wpa_tick(),
 *     which runs outside the RX dispatch: sending an MM_KEY_ADD message
 *     from inside the dispatch would need to drain the same endpoint the
 *     dispatch is already reading.
 */

typedef enum {
    AIC_WPA_IDLE = 0,
    AIC_WPA_WAIT_M1, /* associated, waiting for message 1 of 4 */
    AIC_WPA_WAIT_M3, /* message 2 sent, waiting for message 3 */
    AIC_WPA_DONE,    /* PTK (and GTK) installed */
    AIC_WPA_FAILED,
} aic_wpa_state_t;

/* Arm the supplicant for a new connection.
 *   psk      -- WPA2 passphrase, 8..63 characters (NUL-terminated)
 *   ssid/len -- the SSID (PMK salt)
 *   bssid    -- AP MAC (address 1 of the PTK derivation)
 *   own_mac  -- our station MAC (address 2 of the PTK derivation)
 * Returns false when the passphrase is too short to be a WPA2 PSK. */
bool aic_wpa_start(const char *psk, const uint8_t *ssid, uint32_t ssid_len,
                   const uint8_t *bssid, const uint8_t *own_mac);

void aic_wpa_stop(void);
aic_wpa_state_t aic_wpa_state(void);

/* Consume one received 802.3 frame. Returns true when it was EAPOL. */
bool aic_wpa_recv(const uint8_t *eth_frame, uint16_t frame_len);

/* Periodic work: EAPOL-Start retry, deferred MM_KEY_ADD installs,
 * handshake timeout. Call outside the RX dispatch. */
void aic_wpa_tick(void);

/* The RSN IE this driver puts in the association request, and the exact
 * same bytes it echoes in message 2 (required by the standard). NULL
 * when the target is an open network. */
const uint8_t *aic_wpa_rsn_ie(uint32_t *out_len);
