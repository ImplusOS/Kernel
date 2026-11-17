#include "AIC8800_Wpa.h"
#include "AIC8800.h"
#include "AIC8800_Crypto.h"

/*
 * AIC8800_Wpa.c -- WPA2-PSK supplicant, see AIC8800_Wpa.h.
 *
 * References, in order of authority:
 *   - IEEE 802.11i-2004 clause 8.5 (EAPOL-Key frames, 4-way handshake)
 *   - wpa_supplicant 2.10 (src/rsn_supp/wpa.c, src/common/wpa_common.c)
 *     for every byte-level detail the standard leaves ambiguous --
 *     which key_info bits message 2 and 4 carry, the exact MIC
 *     coverage, the GTK KDE layout, the AES-KW unwrap of message 3's
 *     key data, and the "reject a GTK in unencrypted key data" rule.
 */

/* ---- EAPOL ---- */

#define WPA_EAPOL_VERSION              2u
#define EAPOL_TYPE_EAPOL_START         1u
#define EAPOL_TYPE_EAPOL_KEY           3u
#define EAPOL_KEY_TYPE_RSN             2u
#define EAPOL_ETHERTYPE                0x888eu

/* key_info bits */
#define WPA_KEY_INFO_TYPE_MASK         0x0007u
#define WPA_KEY_INFO_TYPE_HMAC_SHA1_AES 0x0002u
#define WPA_KEY_INFO_KEY_TYPE          0x0008u /* 1 = pairwise */
#define WPA_KEY_INFO_INSTALL           0x0040u
#define WPA_KEY_INFO_ACK               0x0080u
#define WPA_KEY_INFO_MIC               0x0100u
#define WPA_KEY_INFO_SECURE            0x0200u
#define WPA_KEY_INFO_ENCR_KEY_DATA     0x1000u

/* EAPOL-Key body offsets (see struct wpa_eapol_key in hostap) */
#define KEY_INFO_OFF                   1u
#define KEY_LENGTH_OFF                 3u
#define REPLAY_OFF                     5u
#define NONCE_OFF                      13u
#define MIC_OFF                        77u
#define KEY_DATA_LEN_OFF               93u
#define KEY_DATA_OFF                   95u
#define EAPOL_HDR_LEN                  4u
#define ETH_HDR_LEN                    14u

#define WPA_NONCE_LEN                  32u
#define WPA_REPLAY_LEN                 8u
#define WPA_MIC_LEN                    16u

#define AIC_WPA_MAX_FRAME              384u
#define AIC_WPA_GTK_MAX                32u

/* MM_KEY_ADD cipher_suite values (enum mac_cipher_suite). */
#define AIC_MAC_CIPHER_TKIP            1u
#define AIC_MAC_CIPHER_CCMP            2u

/* RSN capabilities: management frame protection required / capable. */
#define AIC_RSN_CAP_MFPR               0x0040u

/* How long the whole 4-way handshake may take before the connection is
 * declared failed. Without this a silent AP (or one whose message 1 never
 * arrives) would leave the panel spinning on "Connecting..." forever,
 * because nothing else bounds the WAIT_M1/WAIT_M3 states. */
#define AIC_WPA_HANDSHAKE_TIMEOUT_MS   15000u

/* MM_KEY_ADD is a blocking command (AIC_CMD_TIMEOUT_MS per attempt), so a
 * firmware that keeps rejecting a key must not be retried forever from the
 * poll path. */
#define AIC_WPA_INSTALL_MAX_ATTEMPTS   5u

typedef struct {
    aic_wpa_state_t state;
    bool     armed;
    bool     pmk_valid;
    bool     ptk_valid;

    uint8_t  pmk[32];
    uint8_t  ptk[64];    /* KCK[0:16] | KEK[16:32] | TK[32:48] | MIC keys */
    uint8_t  snonce[WPA_NONCE_LEN];
    uint8_t  anonce[WPA_NONCE_LEN];
    uint8_t  replay[WPA_REPLAY_LEN];
    uint8_t  bssid[6];
    uint8_t  own_mac[6];

    uint8_t  rsn_ie[22];
    uint32_t rsn_ie_len;
    bool     replay_valid; /* `replay` holds the last accepted counter */

    uint64_t start_ticks;
    bool     eapol_start_sent;

    /* Key material to hand to the firmware, installed from aic_wpa_tick()
     * so the MM_KEY_ADD command engine never runs inside the RX
     * dispatch. */
    bool     install_ptk;
    bool     install_gtk;
    bool     keys_installed;
    uint8_t  install_attempts;
    uint8_t  gtk[AIC_WPA_GTK_MAX];
    uint32_t gtk_len;
    uint8_t  gtk_idx;
} aic_wpa_t;

static aic_wpa_t g_wpa;

/* The group cipher of the next association. Lives outside g_wpa because
 * aic_connect() chooses it before aic_wpa_start() resets the rest. */
static uint8_t g_wpa_group = AIC_RSN_GROUP_CCMP;

/* WPA2-PSK RSN IE: version 1, the AP's group cipher, one pairwise CCMP,
 * one PSK AKM, no RSN capabilities. Used both in the association request
 * (SM_CONNECT_REQ's ie_buf) and in message 2/4 -- the two must be the same
 * bytes, which is why it is built once per connection. */
static uint8_t g_rsn_ie[22] = {
    0x30u, 0x14u,
    0x01u, 0x00u,                    /* version 1 */
    0x00u, 0x0fu, 0xacu, 0x04u,      /* group cipher (patched below) */
    0x01u, 0x00u,                    /* pairwise count: 1 */
    0x00u, 0x0fu, 0xacu, 0x04u,      /* pairwise cipher: CCMP */
    0x01u, 0x00u,                    /* AKM count: 1 */
    0x00u, 0x0fu, 0xacu, 0x02u,      /* AKM: PSK */
    0x00u, 0x00u                     /* RSN capabilities */
};

/* ---- tiny helpers (the driver API's memcpy/memset are available, but
 *      these keep the crypto-adjacent code dependency-free) ---- */

static void wpa_memcpy(void *dst, const void *src, uint32_t n)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    for (uint32_t i = 0u; i < n; ++i) {
        d[i] = s[i];
    }
}

static void wpa_memset(void *dst, uint8_t v, uint32_t n)
{
    uint8_t *d = (uint8_t *)dst;
    for (uint32_t i = 0u; i < n; ++i) {
        d[i] = v;
    }
}

static bool wpa_equal(const void *a, const void *b, uint32_t n)
{
    const uint8_t *pa = (const uint8_t *)a;
    const uint8_t *pb = (const uint8_t *)b;
    for (uint32_t i = 0u; i < n; ++i) {
        if (pa[i] != pb[i]) {
            return false;
        }
    }
    return true;
}

static uint16_t wpa_get_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void wpa_put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

static uint32_t wpa_strlen(const char *s)
{
    uint32_t n = 0u;
    while (s[n] != '\0') {
        ++n;
    }
    return n;
}

/* ---- key derivation ---- */

/* PRF-n as used by 802.11i: HMAC-SHA1(key, label || 0x00 || data || i)
 * for i = 0, 1, ... , concatenated and truncated to out_len. The
 * derivation buffers here are tiny (<= 100 bytes), so one flat scratch
 * buffer is enough. */
static void wpa_prf(const uint8_t *key, uint32_t key_len, const char *label,
                    const uint8_t *data, uint32_t data_len,
                    uint8_t *out, uint32_t out_len)
{
    uint8_t msg[128];
    uint32_t label_len = wpa_strlen(label) + 1u; /* include the NUL */
    uint32_t msg_len;
    uint32_t done = 0u;
    uint8_t counter = 0u;

    if (label_len + data_len + 1u > sizeof(msg)) {
        return;
    }
    for (uint32_t i = 0u; i < label_len; ++i) {
        msg[i] = (uint8_t)label[i];
    }
    wpa_memcpy(msg + label_len, data, data_len);
    msg_len = label_len + data_len;

    while (done < out_len) {
        uint8_t h[20];
        uint32_t take;

        /* The counter byte starts at 0 (IEEE 802.11i H.3 test vectors,
         * wpa_supplicant's sha1_prf()). */
        msg[msg_len] = counter;
        ++counter;
        aic_hmac_sha1(key, key_len, msg, msg_len + 1u, h);

        take = out_len - done;
        if (take > 20u) {
            take = 20u;
        }
        wpa_memcpy(out + done, h, take);
        done += take;
    }
}

/* PTK = PRF-512(PMK, "Pairwise key expansion",
 *               min(AA,SPA) || max(AA,SPA) || min(ANonce,SNonce) ||
 *               max(ANonce,SNonce))
 * producing 64 bytes: KCK(16) | KEK(16) | TK(16) | MIC keys(16). CCMP
 * only consumes KCK/KEK/TK, but wpa_supplicant derives all 64 and the
 * pairwise key in message 3 is verified against the first 16 of them. */
static bool wpa_derive_ptk(void)
{
    uint8_t data[2u * 6u + 2u * WPA_NONCE_LEN];
    const uint8_t *aa = g_wpa.bssid;   /* authenticator */
    const uint8_t *spa = g_wpa.own_mac; /* supplicant */

    if (!g_wpa.pmk_valid) {
        return false;
    }

    /* min/max on the MAC addresses (byte-wise) */
    {
        uint32_t i = 0u;
        bool aa_first = true;
        while (i < 6u && aa[i] == spa[i]) {
            ++i;
        }
        if (i < 6u) {
            aa_first = (aa[i] < spa[i]);
        }
        if (aa_first) {
            wpa_memcpy(data, aa, 6u);
            wpa_memcpy(data + 6u, spa, 6u);
        } else {
            wpa_memcpy(data, spa, 6u);
            wpa_memcpy(data + 6u, aa, 6u);
        }
    }

    /* min/max on the nonces (byte-wise) */
    {
        uint32_t i = 0u;
        bool anonce_first = true;
        while (i < WPA_NONCE_LEN && g_wpa.anonce[i] == g_wpa.snonce[i]) {
            ++i;
        }
        if (i < WPA_NONCE_LEN) {
            anonce_first = (g_wpa.anonce[i] < g_wpa.snonce[i]);
        }
        if (anonce_first) {
            wpa_memcpy(data + 12u, g_wpa.anonce, WPA_NONCE_LEN);
            wpa_memcpy(data + 12u + WPA_NONCE_LEN, g_wpa.snonce, WPA_NONCE_LEN);
        } else {
            wpa_memcpy(data + 12u, g_wpa.snonce, WPA_NONCE_LEN);
            wpa_memcpy(data + 12u + WPA_NONCE_LEN, g_wpa.anonce, WPA_NONCE_LEN);
        }
    }

    wpa_prf(g_wpa.pmk, 32u, "Pairwise key expansion",
            data, sizeof(data), g_wpa.ptk, 64u);
    g_wpa.ptk_valid = true;
    return true;
}

/* ---- transmission ---- */

static bool wpa_send_eapol_key(const uint8_t *body, uint16_t body_len, bool with_mic)
{
    uint8_t frame[AIC_WPA_MAX_FRAME];

    if ((uint32_t)(ETH_HDR_LEN + EAPOL_HDR_LEN) + body_len > sizeof(frame)) {
        return false;
    }

    /* 802.3 header: to the AP, from us, EAPOL. */
    wpa_memcpy(frame, g_wpa.bssid, 6u);
    wpa_memcpy(frame + 6u, g_wpa.own_mac, 6u);
    frame[12] = 0x88u;
    frame[13] = 0x8Eu;

    /* IEEE 802.1X header. */
    frame[ETH_HDR_LEN + 0u] = WPA_EAPOL_VERSION;
    frame[ETH_HDR_LEN + 1u] = EAPOL_TYPE_EAPOL_KEY;
    wpa_put_be16(frame + ETH_HDR_LEN + 2u, body_len);

    wpa_memcpy(frame + ETH_HDR_LEN + EAPOL_HDR_LEN, body, body_len);

    if (with_mic) {
        uint8_t mic[20];
        /* The MIC covers the whole EAPOL frame (version/type/length
         * included) with the MIC field itself zeroed. */
        wpa_memset(frame + ETH_HDR_LEN + EAPOL_HDR_LEN + MIC_OFF, 0u, WPA_MIC_LEN);
        aic_hmac_sha1(g_wpa.ptk, 16u, frame + ETH_HDR_LEN,
                      EAPOL_HDR_LEN + body_len, mic);
        wpa_memcpy(frame + ETH_HDR_LEN + EAPOL_HDR_LEN + MIC_OFF, mic, WPA_MIC_LEN);
    }

    return aic_tx_eth(frame, (uint16_t)(ETH_HDR_LEN + EAPOL_HDR_LEN + body_len));
}

static bool wpa_send_eapol_start(void)
{
    uint8_t frame[ETH_HDR_LEN + EAPOL_HDR_LEN];

    wpa_memcpy(frame, g_wpa.bssid, 6u);
    wpa_memcpy(frame + 6u, g_wpa.own_mac, 6u);
    frame[12] = 0x88u;
    frame[13] = 0x8Eu;
    frame[ETH_HDR_LEN + 0u] = WPA_EAPOL_VERSION;
    frame[ETH_HDR_LEN + 1u] = EAPOL_TYPE_EAPOL_START;
    wpa_put_be16(frame + ETH_HDR_LEN + 2u, 0u);

    return aic_tx_eth(frame, (uint16_t)sizeof(frame));
}

/* ---- message handlers ---- */

/* Replay counters are 64-bit big-endian; a new message must carry a larger
 * one than the last message this supplicant accepted. */
static bool wpa_replay_newer(const uint8_t *candidate, const uint8_t *last)
{
    for (uint32_t i = 0u; i < WPA_REPLAY_LEN; ++i) {
        if (candidate[i] != last[i]) {
            return candidate[i] > last[i];
        }
    }
    return false;
}

static void wpa_handle_m1(const uint8_t *key, uint16_t body_len)
{
    uint16_t key_info;
    uint8_t body[KEY_DATA_OFF + sizeof(g_wpa.rsn_ie)];
    uint16_t ki;
    uint32_t ie_len = 0u;
    const uint8_t *rsn_ie;

    /* Message 1 may also arrive as a retransmission while we are in
     * WAIT_M3: the AP resends it when it did not see our message 2, and
     * the right answer is to rebuild and resend message 2 (with the same
     * SNonce and whatever ANonce the AP just handed us). */
    if (g_wpa.state != AIC_WPA_WAIT_M1 && g_wpa.state != AIC_WPA_WAIT_M3 &&
        g_wpa.state != AIC_WPA_DONE) {
        return;
    }
    if (body_len < KEY_DATA_OFF) {
        return;
    }
    key_info = wpa_get_be16(key + KEY_INFO_OFF);
    if (g_wpa.state == AIC_WPA_DONE) {
        /* A PTK rekey: a fresh 4-way handshake on the live link. It needs a
         * fresh SNonce; the keys stay installed until the new ones are. */
        if (g_wpa.replay_valid &&
            !wpa_replay_newer(key + REPLAY_OFF, g_wpa.replay)) {
            return;
        }
        aic_random_bytes(g_wpa.snonce, WPA_NONCE_LEN);
        g_wpa.start_ticks = aic_now_ticks();
    }

    /* Message 1: pairwise, ACK, no MIC, not an install, no error. */
    if ((key_info & WPA_KEY_INFO_KEY_TYPE) == 0u ||
        (key_info & WPA_KEY_INFO_ACK) == 0u ||
        (key_info & WPA_KEY_INFO_MIC) != 0u ||
        (key_info & WPA_KEY_INFO_INSTALL) != 0u) {
        return;
    }

    wpa_memcpy(g_wpa.anonce, key + NONCE_OFF, WPA_NONCE_LEN);
    wpa_memcpy(g_wpa.replay, key + REPLAY_OFF, WPA_REPLAY_LEN);

    if (!wpa_derive_ptk()) {
        g_wpa.state = AIC_WPA_FAILED;
        return;
    }

    /* Message 2: version | pairwise | MIC (secure stays 0 -- the port is
     * not yet authorized), our SNonce, the association RSN IE as key
     * data. Every other field (key length, IV, RSC, key ID) is zero. */
    wpa_memset(body, 0u, sizeof(body));
    body[0] = EAPOL_KEY_TYPE_RSN;
    ki = (uint16_t)(WPA_KEY_INFO_TYPE_HMAC_SHA1_AES |
                    WPA_KEY_INFO_KEY_TYPE | WPA_KEY_INFO_MIC);
    wpa_put_be16(body + KEY_INFO_OFF, ki);
    wpa_put_be16(body + KEY_LENGTH_OFF, 0u);
    wpa_memcpy(body + REPLAY_OFF, g_wpa.replay, WPA_REPLAY_LEN);
    wpa_memcpy(body + NONCE_OFF, g_wpa.snonce, WPA_NONCE_LEN);

    rsn_ie = aic_wpa_rsn_ie(&ie_len);
    if (rsn_ie == NULL || ie_len > sizeof(g_wpa.rsn_ie)) {
        g_wpa.state = AIC_WPA_FAILED;
        return;
    }
    wpa_put_be16(body + KEY_DATA_LEN_OFF, (uint16_t)ie_len);
    wpa_memcpy(body + KEY_DATA_OFF, rsn_ie, ie_len);

    if (!wpa_send_eapol_key(body, (uint16_t)(KEY_DATA_OFF + ie_len), true)) {
        g_wpa.state = AIC_WPA_FAILED;
        return;
    }

    g_wpa.state = AIC_WPA_WAIT_M3;
}

/* Find the GTK inside (unwrapped) key data. Layout of the vendor
 * specific element, per 802.11i figure 43x:
 *   dd <len> 00 0f ac 01 <keyid|tx> <reserved> <gtk...>
 * where <len> = 6 + sizeof(gtk). */
static bool wpa_parse_gtk_kde(const uint8_t *ies, uint32_t len,
                              uint8_t *gtk_out, uint32_t *gtk_len_out,
                              uint8_t *key_idx_out)
{
    uint32_t pos = 0u;

    while (pos + 2u <= len) {
        uint8_t id = ies[pos];
        uint8_t elen = ies[pos + 1u];

        if (id == 0u && elen == 0u) {
            break; /* padding */
        }
        if (pos + 2u + elen > len) {
            break;
        }
        if (id == 0xDDu && elen >= 6u &&
            ies[pos + 2u] == 0x00u && ies[pos + 3u] == 0x0Fu &&
            ies[pos + 4u] == 0xACu && ies[pos + 5u] == 0x01u) {
            uint32_t gtk_len = (uint32_t)elen - 6u;
            if (gtk_len == 0u || gtk_len > AIC_WPA_GTK_MAX) {
                return false;
            }
            *key_idx_out = (uint8_t)(ies[pos + 6u] & 0x03u);
            wpa_memcpy(gtk_out, ies + pos + 8u, gtk_len);
            *gtk_len_out = gtk_len;
            return true;
        }
        pos += 2u + (uint32_t)elen;
    }
    return false;
}

static bool wpa_key_data_has_gtk_kde(const uint8_t *ies, uint32_t len)
{
    uint8_t gtk[AIC_WPA_GTK_MAX];
    uint32_t gtk_len = 0u;
    uint8_t idx = 0u;
    return wpa_parse_gtk_kde(ies, len, gtk, &gtk_len, &idx);
}

/* GTK as MM_KEY_ADD wants it. For TKIP the KDE carries TK | Tx-MIC |
 * Rx-MIC from the authenticator's point of view; the station swaps the two
 * Michael keys (wpa_supplicant_install_gtk()). */
static void wpa_stage_gtk(const uint8_t *gtk, uint32_t gtk_len, uint8_t idx)
{
    if (g_wpa_group == AIC_RSN_GROUP_TKIP && gtk_len == 32u) {
        wpa_memcpy(g_wpa.gtk, gtk, 16u);
        wpa_memcpy(g_wpa.gtk + 16u, gtk + 24u, 8u);
        wpa_memcpy(g_wpa.gtk + 24u, gtk + 16u, 8u);
    } else {
        wpa_memcpy(g_wpa.gtk, gtk, gtk_len);
    }
    g_wpa.gtk_len = gtk_len;
    g_wpa.gtk_idx = idx;
    g_wpa.install_gtk = true;
}

static bool wpa_verify_mic(uint8_t *key, uint8_t *eapol, uint16_t eapol_len)
{
    uint8_t saved_mic[WPA_MIC_LEN];
    uint8_t calc_mic[20];

    wpa_memcpy(saved_mic, key + MIC_OFF, WPA_MIC_LEN);
    wpa_memset(key + MIC_OFF, 0u, WPA_MIC_LEN);
    aic_hmac_sha1(g_wpa.ptk, 16u, eapol, eapol_len, calc_mic);
    wpa_memcpy(key + MIC_OFF, saved_mic, WPA_MIC_LEN);
    return wpa_equal(calc_mic, saved_mic, WPA_MIC_LEN);
}

static void wpa_handle_m3(uint8_t *key, uint16_t body_len,
                          uint8_t *eapol, uint16_t eapol_len)
{
    uint16_t key_info;
    uint16_t key_length;
    uint16_t kd_len;
    uint8_t body[KEY_DATA_OFF];

    if (g_wpa.state != AIC_WPA_WAIT_M3) {
        return;
    }
    if (body_len < KEY_DATA_OFF || !g_wpa.ptk_valid) {
        return;
    }
    key_info = wpa_get_be16(key + KEY_INFO_OFF);

    /* Message 3: pairwise, ACK+MIC+INSTALL, teaches us the GTK. */
    if ((key_info & WPA_KEY_INFO_KEY_TYPE) == 0u ||
        (key_info & WPA_KEY_INFO_MIC) == 0u ||
        (key_info & WPA_KEY_INFO_ACK) == 0u) {
        return;
    }
    if (!wpa_equal(key + NONCE_OFF, g_wpa.anonce, WPA_NONCE_LEN)) {
        return; /* ANonce mismatch: another handshake instance? */
    }
    key_length = wpa_get_be16(key + KEY_LENGTH_OFF);
    if (key_length != 16u) {
        g_wpa.state = AIC_WPA_FAILED;
        return;
    }

    kd_len = wpa_get_be16(key + KEY_DATA_LEN_OFF);
    if ((uint32_t)KEY_DATA_OFF + kd_len > body_len) {
        return; /* truncated */
    }

    /* Verify the key MIC over the whole EAPOL frame. The frame lives in
     * the RX scratch buffer, which is ours to modify. */
    if (!wpa_verify_mic(key, eapol, eapol_len)) {
        /* Wrong KCK: the PSK is wrong. Report as a failure rather than
         * silently retrying -- the AP will keep retransmitting M1. */
        g_wpa.state = AIC_WPA_FAILED;
        return;
    }
    wpa_memcpy(g_wpa.replay, key + REPLAY_OFF, WPA_REPLAY_LEN);
    g_wpa.replay_valid = true;

    /* Key data: wrapped with the KEK when the encrypted-key-data bit is
     * set (the normal WPA2 case -- unwrap it and pull the GTK out). */
    if ((key_info & WPA_KEY_INFO_ENCR_KEY_DATA) != 0u) {
        uint8_t unwrapped[AIC_AES_KW_MAX_BLOCKS * 8u];
        uint32_t unwrapped_len = 0u;
        uint8_t gtk[AIC_WPA_GTK_MAX];
        uint32_t gtk_len = 0u;
        uint8_t idx = 0u;

        if (kd_len < 16u ||
            !aic_aes128_kw_unwrap(g_wpa.ptk + 16u, key + KEY_DATA_OFF, kd_len,
                                  unwrapped, sizeof(unwrapped), &unwrapped_len)) {
            g_wpa.state = AIC_WPA_FAILED;
            return;
        }
        if (wpa_parse_gtk_kde(unwrapped, unwrapped_len, gtk, &gtk_len, &idx)) {
            wpa_stage_gtk(gtk, gtk_len, idx);
        }
        wpa_memset(unwrapped, 0u, sizeof(unwrapped));
    } else if (wpa_key_data_has_gtk_kde(key + KEY_DATA_OFF, kd_len)) {
        /* 802.11i forbids the GTK in unencrypted key data. */
        g_wpa.state = AIC_WPA_FAILED;
        return;
    }

    /* Message 4: same key_info as M3 minus ACK/INSTALL/ENCR, MIC over an
     * empty key data. wpa_supplicant copies the SECURE bit from M3 and
     * leaves the nonce field zeroed -- match it exactly. */
    wpa_memset(body, 0u, sizeof(body));
    body[0] = EAPOL_KEY_TYPE_RSN;
    {
        uint16_t ki4 = (uint16_t)(key_info & WPA_KEY_INFO_SECURE);
        ki4 = (uint16_t)(ki4 | WPA_KEY_INFO_TYPE_HMAC_SHA1_AES |
                         WPA_KEY_INFO_KEY_TYPE | WPA_KEY_INFO_MIC);
        wpa_put_be16(body + KEY_INFO_OFF, ki4);
    }
    wpa_put_be16(body + KEY_LENGTH_OFF, 0u);
    wpa_memcpy(body + REPLAY_OFF, key + REPLAY_OFF, WPA_REPLAY_LEN);
    wpa_put_be16(body + KEY_DATA_LEN_OFF, 0u);

    if (!wpa_send_eapol_key(body, KEY_DATA_OFF, true)) {
        g_wpa.state = AIC_WPA_FAILED;
        return;
    }

    /* The PTK is installed from aic_wpa_tick() (deferred out of the RX
     * dispatch), after which the data path is live. */
    g_wpa.install_ptk = true;
    g_wpa.state = AIC_WPA_DONE;
}

/* Group Key Handshake, message 1 of 2 (IEEE 802.11-2016 12.7.7): the AP
 * hands out a new GTK, wrapped with the KEK. Answering it matters: an AP
 * whose group rekey goes unanswered deauthenticates the station. */
static void wpa_handle_group_m1(uint8_t *key, uint16_t body_len,
                                uint8_t *eapol, uint16_t eapol_len)
{
    uint16_t key_info;
    uint16_t kd_len;
    uint8_t unwrapped[AIC_AES_KW_MAX_BLOCKS * 8u];
    uint32_t unwrapped_len = 0u;
    uint8_t gtk[AIC_WPA_GTK_MAX];
    uint32_t gtk_len = 0u;
    uint8_t idx = 0u;
    uint8_t body[KEY_DATA_OFF];

    if (g_wpa.state != AIC_WPA_DONE || !g_wpa.ptk_valid ||
        body_len < KEY_DATA_OFF) {
        return;
    }
    key_info = wpa_get_be16(key + KEY_INFO_OFF);
    if ((key_info & WPA_KEY_INFO_MIC) == 0u ||
        (key_info & WPA_KEY_INFO_ACK) == 0u ||
        (key_info & WPA_KEY_INFO_ENCR_KEY_DATA) == 0u) {
        return;
    }
    if (g_wpa.replay_valid &&
        !wpa_replay_newer(key + REPLAY_OFF, g_wpa.replay)) {
        return; /* a replay of an older message */
    }
    kd_len = wpa_get_be16(key + KEY_DATA_LEN_OFF);
    if ((uint32_t)KEY_DATA_OFF + kd_len > body_len || kd_len < 16u) {
        return;
    }
    if (!wpa_verify_mic(key, eapol, eapol_len)) {
        return;
    }
    if (!aic_aes128_kw_unwrap(g_wpa.ptk + 16u, key + KEY_DATA_OFF, kd_len,
                              unwrapped, sizeof(unwrapped), &unwrapped_len) ||
        !wpa_parse_gtk_kde(unwrapped, unwrapped_len, gtk, &gtk_len, &idx)) {
        wpa_memset(unwrapped, 0u, sizeof(unwrapped));
        return;
    }
    wpa_memset(unwrapped, 0u, sizeof(unwrapped));
    wpa_memcpy(g_wpa.replay, key + REPLAY_OFF, WPA_REPLAY_LEN);
    g_wpa.replay_valid = true;

    /* Message 2 of 2: MIC | SECURE, the same replay counter, no key data
     * (wpa_supplicant_send_2_of_2() for RSN). */
    wpa_memset(body, 0u, sizeof(body));
    body[0] = EAPOL_KEY_TYPE_RSN;
    wpa_put_be16(body + KEY_INFO_OFF,
                 (uint16_t)(WPA_KEY_INFO_TYPE_HMAC_SHA1_AES |
                            WPA_KEY_INFO_MIC | WPA_KEY_INFO_SECURE));
    wpa_memcpy(body + REPLAY_OFF, key + REPLAY_OFF, WPA_REPLAY_LEN);
    if (!wpa_send_eapol_key(body, KEY_DATA_OFF, true)) {
        return; /* the AP retransmits message 1 */
    }
    wpa_stage_gtk(gtk, gtk_len, idx);
    g_wpa.install_attempts = 0u;
}

/* ---- public interface ---- */

const uint8_t *aic_wpa_rsn_ie(uint32_t *out_len)
{
    g_rsn_ie[7] = (g_wpa_group == AIC_RSN_GROUP_TKIP) ? AIC_RSN_GROUP_TKIP
                                                      : AIC_RSN_GROUP_CCMP;
    *out_len = sizeof(g_rsn_ie);
    return g_rsn_ie;
}

void aic_wpa_set_group_cipher(uint8_t group)
{
    g_wpa_group = (group == AIC_RSN_GROUP_TKIP) ? AIC_RSN_GROUP_TKIP
                                                : AIC_RSN_GROUP_CCMP;
}

/* A cipher/AKM selector list entry is 4 bytes: OUI 00-0F-AC + type. */
static bool rsn_suite_is(const uint8_t *s, uint8_t type)
{
    return s[0] == 0x00u && s[1] == 0x0Fu && s[2] == 0xACu && s[3] == type;
}

bool aic_wpa_rsn_usable(const uint8_t *ie, uint32_t ie_len, uint8_t *group_out)
{
    uint32_t pos;
    uint32_t end;
    uint16_t count;
    bool ccmp = false;
    bool psk = false;
    uint8_t group;

    if (ie == NULL || ie_len < 2u || ie[0] != 0x30u) {
        return false;
    }
    end = 2u + (uint32_t)ie[1];
    if (end > ie_len || end < 8u) {
        return false;
    }
    if (ie[2] != 0x01u || ie[3] != 0x00u) {
        return false; /* RSN version 1 only */
    }
    /* Group cipher. */
    if (rsn_suite_is(ie + 4u, AIC_RSN_GROUP_CCMP)) {
        group = AIC_RSN_GROUP_CCMP;
    } else if (rsn_suite_is(ie + 4u, AIC_RSN_GROUP_TKIP)) {
        group = AIC_RSN_GROUP_TKIP;
    } else {
        return false;
    }
    pos = 8u;
    /* Pairwise ciphers (absent = CCMP by default). */
    if (pos + 2u > end) {
        ccmp = true;
        psk = true; /* absent AKM list defaults to 802.1X, but be lenient */
    } else {
        count = (uint16_t)(ie[pos] | (ie[pos + 1u] << 8));
        pos += 2u;
        for (uint16_t i = 0u; i < count; ++i, pos += 4u) {
            if (pos + 4u > end) {
                return false;
            }
            ccmp = ccmp || rsn_suite_is(ie + pos, 4u);
        }
        /* AKM suites. */
        if (pos + 2u <= end) {
            count = (uint16_t)(ie[pos] | (ie[pos + 1u] << 8));
            pos += 2u;
            for (uint16_t i = 0u; i < count; ++i, pos += 4u) {
                if (pos + 4u > end) {
                    return false;
                }
                psk = psk || rsn_suite_is(ie + pos, 2u);
            }
            /* RSN capabilities: refuse mandatory MFP (no BIP here). */
            if (pos + 2u <= end) {
                uint16_t caps = (uint16_t)(ie[pos] | (ie[pos + 1u] << 8));
                if ((caps & AIC_RSN_CAP_MFPR) != 0u) {
                    return false;
                }
            }
        }
    }
    if (group_out != NULL) {
        *group_out = group;
    }
    return ccmp && psk;
}

bool aic_wpa_start(const char *psk, const uint8_t *ssid, uint32_t ssid_len,
                   const uint8_t *bssid, const uint8_t *own_mac)
{
    uint32_t psk_len;

    wpa_memset(&g_wpa, 0u, sizeof(g_wpa));

    if (psk == NULL) {
        return false;
    }
    psk_len = wpa_strlen(psk);
    if (psk_len < 8u || psk_len > 63u) {
        return false;
    }
    if (ssid_len > 32u) {
        ssid_len = 32u;
    }

    wpa_memcpy(g_wpa.bssid, bssid, 6u);
    wpa_memcpy(g_wpa.own_mac, own_mac, 6u);
    {
        uint32_t ie_len = 0u;
        const uint8_t *ie = aic_wpa_rsn_ie(&ie_len);
        wpa_memcpy(g_wpa.rsn_ie, ie, ie_len);
        g_wpa.rsn_ie_len = ie_len;
    }

    /* PMK = PBKDF2(HMAC-SHA1, passphrase, SSID, 4096, 256 bits) */
    aic_pbkdf2_sha1((const uint8_t *)psk, psk_len, ssid, ssid_len,
                    4096u, g_wpa.pmk, 32u);
    g_wpa.pmk_valid = true;

    /* Our SNonce for this handshake instance. */
    aic_random_bytes(g_wpa.snonce, WPA_NONCE_LEN);

    g_wpa.armed = true;
    g_wpa.state = AIC_WPA_WAIT_M1;
    g_wpa.start_ticks = aic_now_ticks();
    g_wpa.eapol_start_sent = false;

    return true;
}

void aic_wpa_stop(void)
{
    wpa_memset(&g_wpa, 0u, sizeof(g_wpa));
    g_wpa.state = AIC_WPA_IDLE;
}

aic_wpa_state_t aic_wpa_state(void)
{
    return g_wpa.state;
}

bool aic_wpa_keys_installed(void)
{
    return g_wpa.keys_installed;
}

bool aic_wpa_recv(const uint8_t *eth_frame, uint16_t frame_len)
{
    const uint8_t *eapol;
    uint16_t eapol_len;
    const uint8_t *key;
    uint16_t body_len;
    uint8_t eapol_type;

    if (!g_wpa.armed || frame_len < (ETH_HDR_LEN + EAPOL_HDR_LEN)) {
        return false;
    }
    if (eth_frame[12] != 0x88u || eth_frame[13] != 0x8Eu) {
        return false;
    }

    eapol = eth_frame + ETH_HDR_LEN;
    eapol_len = (uint16_t)(frame_len - ETH_HDR_LEN);
    if (eapol[0] != WPA_EAPOL_VERSION) {
        return false; /* EAPOL v1 (WPA1/TKIP) is not supported */
    }
    eapol_type = eapol[1];
    if (eapol_type == EAPOL_TYPE_EAPOL_START || eapol_type == 0x00u) {
        /* EAPOL-Start (AP side) or EAP-Logoff: nothing to do. */
        return true;
    }
    if (eapol_type != EAPOL_TYPE_EAPOL_KEY) {
        return true; /* consumed: this is an EAPOL frame for the port */
    }

    body_len = wpa_get_be16(eapol + 2u);
    if ((uint32_t)EAPOL_HDR_LEN + body_len > eapol_len) {
        return true; /* truncated */
    }
    key = eapol + EAPOL_HDR_LEN;
    if (body_len < KEY_DATA_OFF || key[0] != EAPOL_KEY_TYPE_RSN) {
        return true;
    }

    /* Pairwise: message 1 carries no MIC, message 3 does. Group: the group
     * key handshake. The handlers reject frames that do not belong to the
     * current step. */
    {
        uint16_t ki = wpa_get_be16(key + KEY_INFO_OFF);
        if ((ki & WPA_KEY_INFO_KEY_TYPE) == 0u) {
            wpa_handle_group_m1((uint8_t *)key, body_len, (uint8_t *)eapol,
                                eapol_len);
        } else if ((ki & WPA_KEY_INFO_MIC) == 0u) {
            wpa_handle_m1(key, body_len);
        } else {
            wpa_handle_m3((uint8_t *)key, body_len, (uint8_t *)eapol, eapol_len);
        }
    }
    return true;
}

void aic_wpa_tick(void)
{
    if (!g_wpa.armed) {
        return;
    }

    /* Nothing else bounds WAIT_M1/WAIT_M3: an AP that never sends message 1
     * (or never follows up with message 3) would leave the connection
     * "in progress" forever, with the panel spinning on "Connecting...".
     * aic_ms_since() returns the fallback when the timer HAL cannot
     * answer, and 0 means "no timeout" -- never a spurious failure. */
    if ((g_wpa.state == AIC_WPA_WAIT_M1 || g_wpa.state == AIC_WPA_WAIT_M3) &&
        aic_ms_since(g_wpa.start_ticks, 0u) >= AIC_WPA_HANDSHAKE_TIMEOUT_MS) {
        g_wpa.state = AIC_WPA_FAILED;
        return;
    }

    switch (g_wpa.state) {
    case AIC_WPA_WAIT_M1:
        /* Some APs wait for the station to start the exchange. Send one
         * EAPOL-Start shortly after association and then keep waiting. */
        if (!g_wpa.eapol_start_sent &&
            aic_ms_since(g_wpa.start_ticks, 100000u) >= AIC_EAPOL_START_MS) {
            (void)wpa_send_eapol_start();
            g_wpa.eapol_start_sent = true;
        }
        break;

    case AIC_WPA_DONE:
        /* MM_KEY_ADD is a blocking command (AIC_CMD_TIMEOUT_MS when the
         * firmware does not answer), so a firmware that keeps rejecting a
         * key must not be retried forever from the poll path -- after a few
         * tries the handshake is over and the connection is reported
         * failed rather than stalling every tick. */
        if (g_wpa.install_attempts >= AIC_WPA_INSTALL_MAX_ATTEMPTS) {
            g_wpa.state = AIC_WPA_FAILED;
            break;
        }
        if (g_wpa.install_ptk) {
            /* Pairwise key: key index 0, CCMP, held by the AP's station
             * entry (the firmware assigns hw_key_idx itself). */
            if (aic_mm_key_add(0u, g_aic.ap_idx != 0xFFu ? g_aic.ap_idx : 0u,
                               g_wpa.ptk + 32u, 16u, AIC_MAC_CIPHER_CCMP,
                               true)) {
                g_wpa.install_ptk = false;
            } else {
                ++g_wpa.install_attempts;
            }
        }
        if (!g_wpa.install_ptk && g_wpa.install_gtk) {
            /* Group key: default key index, not tied to a station. */
            if (aic_mm_key_add(g_wpa.gtk_idx, 0xFFu, g_wpa.gtk,
                               (uint8_t)g_wpa.gtk_len,
                               (g_wpa_group == AIC_RSN_GROUP_TKIP)
                                   ? AIC_MAC_CIPHER_TKIP
                                   : AIC_MAC_CIPHER_CCMP,
                               false)) {
                g_wpa.install_gtk = false;
                wpa_memset(g_wpa.gtk, 0u, sizeof(g_wpa.gtk));
                g_wpa.gtk_len = 0u;
            } else {
                ++g_wpa.install_attempts;
            }
        }
        if (!g_wpa.install_ptk && !g_wpa.install_gtk) {
            g_wpa.keys_installed = true;
        }
        break;

    case AIC_WPA_FAILED:
    case AIC_WPA_IDLE:
    default:
        break;
    }
}
