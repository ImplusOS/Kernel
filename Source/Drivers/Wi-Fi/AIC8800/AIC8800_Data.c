#include "AIC8800.h"

/*
 * AIC8800_Data.c -- the 802.11 <-> 802.3 data path.
 *
 * TX: the kernel's Ethernet stack hands aic_tx_eth() a complete 802.3
 * frame. The firmware wants the 14-byte header split out into the 28-byte
 * struct hostdesc (addresses, ethertype and the queue selection) with only
 * the payload behind it, wrapped in the USB "DATA" framing of
 * AIC8800_Protocol.h.
 *
 * RX: one USB DATA packet carries a 56-byte hw_rxhdr (opaque to this
 * driver) followed by one complete 802.11 MPDU. The MPDU is decapsulated
 * -- LLC/SNAP out of the MAC header and, when the AP aggregated several
 * MSDUs, out of each A-MSDU subframe -- and handed to the network stack
 * as an 802.3 frame. EAPOL frames never reach the stack: they belong to
 * the in-driver supplicant.
 *
 * Only one thread ever runs this code (the kernel's syscall/network-poll
 * path) and it never sends a command that waits for a reply: message 2/4
 * of the handshake are plain bulk transfers, which is what makes it safe
 * to do this from inside the RX dispatch (aic_cmd() must not be re-entered
 * there -- see the note in AIC8800.h).
 */

/* EtherType of EAPOL (802.1X): these go to the supplicant, not the stack */
#define AIC_ETH_P_EAPOL     0x888Eu

/* Access categories as the firmware's hardware queues number them
 * (BK 0, BE 1, VI 2, VO 3). Everything this driver sends is best effort:
 * the kernel's stack sets no priority on its frames. */
#define AIC_HOSTDESC_AC_BE  1u

/* The one A-MSDU-subframe-length field is big-endian on the wire. */
static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ---- TX ---- */

bool aic_tx_eth(const uint8_t *frame, uint16_t frame_len)
{
    static uint8_t tx[4u + AIC_TX_HOSTDESC_LEN + AIC_MAX_MTU];
    aic_hostdesc_t *desc;
    uint32_t payload_len;
    uint32_t total;

    if (!g_aic.attached || g_aic.ready == false || frame == NULL) {
        return false;
    }
    if (frame_len < 14u) {
        return false;
    }
    payload_len = (uint32_t)frame_len - 14u;
    if (payload_len > AIC_MAX_MTU) {
        return false;
    }

    /* [u16 len][0x01][0][hostdesc][payload] */
    desc = (aic_hostdesc_t *)(void *)(tx + 4u);
    aic_memset(tx, 0u, 4u + AIC_TX_HOSTDESC_LEN);
    aic_memset(desc, 0u, sizeof(*desc));

    desc->packet_len = (uint16_t)payload_len;
    desc->flags_ext = 0u;
    desc->status_desc_addr = 0u;
    aic_memcpy(desc->eth_dest, frame, 6u);
    aic_memcpy(desc->eth_src, frame + 6u, 6u);
    desc->ethertype = (uint16_t)(((uint16_t)frame[12] << 8) | (uint16_t)frame[13]);
    desc->ac = AIC_HOSTDESC_AC_BE;
    desc->tid = 0u;
    desc->vif_idx = g_aic.vif_idx;
    desc->staid = g_aic.ap_idx;
    desc->flags = 0u;
    aic_memcpy(tx + 4u + AIC_TX_HOSTDESC_LEN, frame + 14u, payload_len);

    total = 4u + AIC_TX_HOSTDESC_LEN + payload_len;
    /* The length field counts everything after the 4-byte USB header. */
    tx[0] = (uint8_t)((AIC_TX_HOSTDESC_LEN + payload_len) & 0xFFu);
    tx[1] = (uint8_t)(((AIC_TX_HOSTDESC_LEN + payload_len) >> 8) & 0x0Fu);
    tx[2] = AIC_USB_TYPE_DATA;
    tx[3] = 0u;

    /* A transfer that is an exact multiple of the max packet size would
     * look short to the firmware's aggregator; the vendor appends one
     * byte in that case, and the length field stays as it is. */
    if (g_aic.ep_out_data_mps != 0u &&
        (total % g_aic.ep_out_data_mps) == 0u && total + 1u <= sizeof(tx)) {
        tx[total] = 0x00u;
        ++total;
    }
    return aic_usb_bulk(g_aic.ep_out_data, g_aic.ep_out_data_mps,
                        AIC_USB_PID_OUT,
                        tx, total);
}

/* ---- RX ---- */

/* Build the 802.3 frame for one MSDU and hand it on. `llc` is the start of
 * the LLC/SNAP header inside the (sub)frame, `msdu_len` its length, and
 * `dst`/`src` the addresses to put in the Ethernet header. */
static void rx_deliver(const uint8_t *dst, const uint8_t *src,
                       const uint8_t *llc, uint32_t msdu_len)
{
    static uint8_t eth[AIC_MAX_MTU + 14u];
    uint32_t out_len;
    uint16_t ethertype;

    if (msdu_len < 8u) {
        return;
    }
    /* LLC/SNAP: AA AA 03 OUI(3) ethertype(2). */
    if (llc[0] != 0xAAu || llc[1] != 0xAAu || llc[2] != 0x03u) {
        return;
    }
    ethertype = be16(llc + 6u);

    out_len = msdu_len - 8u;
    if (out_len > AIC_MAX_MTU) {
        return;
    }

    aic_memcpy(eth, dst, 6u);
    aic_memcpy(eth + 6u, src, 6u);
    eth[12] = llc[6];
    eth[13] = llc[7];
    if (out_len != 0u) {
        aic_memcpy(eth + 14u, llc + 8u, out_len);
    }

    /* EAPOL belongs to the supplicant: the IP stack has no business with
     * it (and the kernel has no 802.1X layer). */
    if (ethertype == AIC_ETH_P_EAPOL) {
        (void)aic_wpa_recv(eth, (uint16_t)(14u + out_len));
        return;
    }
    if (g_aic.rx_callback != NULL) {
        g_aic.rx_callback(eth, (uint16_t)(14u + out_len));
    }
}

void aic_rx_data_packet(const uint8_t *usb_packet, uint16_t usb_len)
{
    const uint8_t *mpdu;
    uint32_t mpdu_len;
    uint16_t fc;
    uint8_t to_ds;
    uint8_t from_ds;
    uint8_t subtype;
    uint32_t hdr_len;
    const uint8_t *addr1;
    const uint8_t *addr2;
    const uint8_t *addr3;
    const uint8_t *addr4 = NULL;
    const uint8_t *dst;
    const uint8_t *src;
    const uint8_t *body;
    uint32_t body_len;
    bool amsdu = false;

    if (usb_packet == NULL || usb_len <= AIC_RX_HWHRD_LEN || !g_aic.ready) {
        return;
    }
    mpdu = usb_packet + AIC_RX_HWHRD_LEN;
    mpdu_len = (uint32_t)usb_len - AIC_RX_HWHRD_LEN;
    if (mpdu_len < 24u) {
        return;
    }

    fc = (uint16_t)((uint16_t)mpdu[0] | ((uint16_t)mpdu[1] << 8));
    if (((fc >> 2) & 0x3u) != 0x2u) {
        return; /* not a data frame (this driver only takes data) */
    }
    subtype = (uint8_t)((fc >> 4) & 0xFu);
    to_ds = (uint8_t)((fc >> 8) & 0x1u);
    from_ds = (uint8_t)((fc >> 9) & 0x1u);
    if (((fc >> 10) & 0x1u) != 0u) {
        return; /* fragmented: the driver never enables fragmentation */
    }

    addr1 = mpdu + 4u;
    addr2 = mpdu + 10u;
    addr3 = mpdu + 16u;
    hdr_len = 24u;
    if (to_ds != 0u && from_ds != 0u) {
        addr4 = mpdu + 24u;
        hdr_len += 6u;
    }
    if ((subtype & 0x8u) != 0u) { /* QoS data: 2-byte QoS control */
        if ((mpdu[hdr_len] & 0x80u) != 0u) {
            amsdu = true;
        }
        hdr_len += 2u;
    }
    if ((fc & 0x8000u) != 0u) { /* HTC field present */
        hdr_len += 4u;
    }
    if (mpdu_len < hdr_len + 8u) {
        return;
    }

    if (to_ds != 0u) {
        dst = addr3;
        src = (addr4 != NULL) ? addr4 : addr2;
    } else {
        dst = addr1;
        src = addr3;
    }

    body = mpdu + hdr_len;
    body_len = mpdu_len - hdr_len;

    if (!amsdu) {
        rx_deliver(dst, src, body, body_len);
        return;
    }

    /* A-MSDU: a chain of DA(6) SA(6) length(2, big-endian) subframes, each
     * padded to a 4-byte boundary, carrying an LLC/SNAP-encapsulated MSDU. */
    while (body_len >= 14u) {
        const uint8_t *sub_da = body;
        const uint8_t *sub_sa = body + 6u;
        uint32_t sub_len = be16(body + 12u);
        uint32_t padded = (sub_len + 3u) & ~3u;

        if (sub_len < 8u || sub_len > (body_len - 14u)) {
            break; /* malformed: stop rather than walk out of the MPDU */
        }
        rx_deliver(sub_da, sub_sa, body + 14u, sub_len);
        if (padded > (body_len - 14u)) {
            break;
        }
        body += 14u + padded;
        body_len -= 14u + padded;
    }
}
