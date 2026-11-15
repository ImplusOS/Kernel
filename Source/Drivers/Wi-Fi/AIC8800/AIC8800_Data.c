#include "AIC8800.h"

/*
 * AIC8800_Data.c -- the 802.11 <-> 802.3 data path.
 *
 * TX: the kernel's Ethernet stack hands aic_tx_eth() a complete 802.3
 * frame. The firmware wants the 14-byte header split out into the 28-byte
 * struct hostdesc (addresses, ethertype and the queue selection) with only
 * the payload behind it, wrapped in the USB "DATA" framing of
 * AIC8800_Protocol.h (aicwf_usb_bus_txdata() in the vendor fdrv).
 *
 * RX: one USB DATA packet carries the RX descriptor (struct hw_rxhdr)
 * followed by one 802.11 MPDU in which the hardware has already decrypted
 * the payload but left the security header in place. The MPDU is turned
 * into 802.3 the way rwnx_rxdataind_aicwf() does it -- MAC header, security
 * header and LLC/SNAP out, or the A-MSDU subframes split apart -- and
 * handed to the network stack. EAPOL frames never reach the stack: they
 * belong to the in-driver supplicant.
 *
 * Only one thread runs this code at a time (the caller holds the driver
 * lock) and it never sends a command that waits for a reply: message 2/4
 * of the handshake are plain bulk transfers, which is what makes it safe
 * to do this from inside the RX dispatch.
 */

/* EtherType of EAPOL (802.1X): these go to the supplicant, not the stack */
#define AIC_ETH_P_EAPOL     0x888Eu

/* Hardware queue (hostdesc.ac, which the vendor fills with the txq's hw
 * queue id: BK 0, BE 1, VI 2, VO 3). The kernel's stack sets no priority
 * on its frames, so everything goes best effort, TID 0 -- what the vendor
 * does for skb->priority 0. */
#define AIC_HOSTDESC_AC_BE  1u

/* The vendor appends one byte to a transfer whose length is a multiple of
 * 512 (instead of relying on a zero-length packet). */
#define AIC_TX_ZLP_UNIT     512u

/* The one A-MSDU-subframe-length field is big-endian on the wire. */
static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ---- TX ---- */

bool aic_tx_eth(const uint8_t *frame, uint16_t frame_len)
{
    static uint8_t tx[4u + AIC_TX_HOSTDESC_LEN + AIC_MAX_MTU + 4u];
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

    desc->packet_len = (uint16_t)payload_len;
    desc->flags_ext = 0u;
    desc->status_desc_addr = 0u; /* no TX confirmation wanted */
    aic_memcpy(desc->eth_dest, frame, 6u);
    aic_memcpy(desc->eth_src, frame + 6u, 6u);
    /* ethertype is copied as it sits in the frame (network byte order),
     * exactly like the vendor's desc->host.ethertype = eth_t.h_proto. */
    aic_memcpy(&desc->ethertype, frame + 12u, 2u);
    desc->ac = AIC_HOSTDESC_AC_BE;
    desc->tid = 0u;
    desc->vif_idx = g_aic.vif_idx;
    desc->staid = g_aic.ap_idx;
    desc->flags = 0u;
    aic_memcpy(tx + 4u + AIC_TX_HOSTDESC_LEN, frame + 14u, payload_len);

    /* The DATA header's length counts the whole transfer, its own four
     * bytes included (the vendor pushes the header onto the skb and then
     * stores skb->len). */
    total = 4u + AIC_TX_HOSTDESC_LEN + payload_len;
    tx[0] = (uint8_t)(total & 0xFFu);
    tx[1] = (uint8_t)((total >> 8) & 0x0Fu);
    tx[2] = AIC_USB_TYPE_DATA;
    tx[3] = 0u;

    if ((total % AIC_TX_ZLP_UNIT) == 0u) {
        tx[total] = 0x00u; /* not counted in the length field */
        ++total;
    }
    return aic_usb_bulk(g_aic.ep_out_data, g_aic.ep_out_data_mps,
                        AIC_USB_PID_OUT, tx, total);
}

/* ---- RX ---- */

/* Hand one 802.3 frame (dst, src, ethertype, payload) on. */
static void rx_deliver_eth(const uint8_t *dst, const uint8_t *src,
                           const uint8_t ethertype[2],
                           const uint8_t *payload, uint32_t payload_len)
{
    static uint8_t eth[AIC_MAX_MTU + 14u];
    uint16_t type = be16(ethertype);

    if (payload_len > AIC_MAX_MTU) {
        return;
    }
    aic_memcpy(eth, dst, 6u);
    aic_memcpy(eth + 6u, src, 6u);
    eth[12] = ethertype[0];
    eth[13] = ethertype[1];
    if (payload_len != 0u) {
        aic_memcpy(eth + 14u, payload, payload_len);
    }

    /* EAPOL belongs to the supplicant: the IP stack has no business with
     * it (and the kernel has no 802.1X layer). */
    if (type == AIC_ETH_P_EAPOL) {
        (void)aic_wpa_recv(eth, (uint16_t)(14u + payload_len));
        return;
    }
    if (g_aic.conn != AIC_CONN_ASSOCIATED) {
        return; /* nothing but the handshake may pass before the link is up */
    }
    if (g_aic.rx_callback != NULL) {
        g_aic.rx_callback(eth, (uint16_t)(14u + payload_len));
    }
}

/* One MSDU that starts with its LLC/SNAP header. */
static void rx_deliver_llc(const uint8_t *dst, const uint8_t *src,
                           const uint8_t *llc, uint32_t msdu_len)
{
    if (msdu_len < 8u) {
        return;
    }
    /* LLC/SNAP: AA AA 03 OUI(3) ethertype(2). (Bridge-tunnel OUI
     * 00:00:F8 is handled the same way: the ethertype follows.) */
    if (llc[0] != 0xAAu || llc[1] != 0xAAu || llc[2] != 0x03u) {
        return;
    }
    rx_deliver_eth(dst, src, llc + 6u, llc + 8u, msdu_len - 8u);
}

/* Security header the hardware left in a decrypted MPDU
 * (rwnx_rxdataind_aicwf(): pull_len per decr_status). */
static uint32_t rx_sec_hdr_len(uint8_t decr_status)
{
    switch (decr_status) {
    case AIC_RX_DECR_CCMP128:
    case AIC_RX_DECR_CCMP256:
    case AIC_RX_DECR_GCMP128:
    case AIC_RX_DECR_GCMP256:
    case AIC_RX_DECR_TKIP:
        return 8u;
    case AIC_RX_DECR_WEP:
        return 4u;
    case AIC_RX_DECR_WAPI:
        return 18u;
    default:
        return 0u;
    }
}

void aic_rx_data_packet(const uint8_t *usb_packet, uint32_t usb_len)
{
    const uint8_t *mpdu;
    uint32_t mpdu_len;
    uint16_t fc;
    uint8_t flags;
    uint8_t decr;
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
    flags = usb_packet[AIC_RXHDR_FLAGS_OFS];
    if ((flags & AIC_RXHDR_FLAG_UPLOAD) == 0u) {
        return; /* the firmware did not mean this one for the host */
    }
    if ((flags & AIC_RXHDR_FLAG_80211_MPDU) != 0u) {
        return; /* a management frame for cfg80211: none of ours */
    }
    decr = (uint8_t)((usb_packet[AIC_RXHDR_DECR_OFS] >> AIC_RXHDR_DECR_SHIFT) &
                     AIC_RXHDR_DECR_MASK);

    mpdu = usb_packet + AIC_RX_HWHRD_LEN;
    mpdu_len = usb_len - AIC_RX_HWHRD_LEN;
    if (mpdu_len < 24u) {
        return;
    }

    fc = (uint16_t)((uint16_t)mpdu[0] | ((uint16_t)mpdu[1] << 8));
    if (((fc >> 2) & 0x3u) != 0x2u) {
        return; /* not a data frame */
    }
    subtype = (uint8_t)((fc >> 4) & 0xFu);
    if ((subtype & 0x4u) != 0u) {
        return; /* null / QoS-null: no payload */
    }
    to_ds = (uint8_t)((fc >> 8) & 0x1u);
    from_ds = (uint8_t)((fc >> 9) & 0x1u);
    if (((fc >> 10) & 0x1u) != 0u || (mpdu[22] & 0x0Fu) != 0u) {
        /* A fragment. APs essentially never fragment (the threshold
         * defaults to off), and this station never asks for it. */
        return;
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
        if (mpdu_len < hdr_len + 2u) {
            return;
        }
        if ((mpdu[hdr_len] & 0x80u) != 0u) {
            amsdu = true;
        }
        hdr_len += 2u;
        if ((fc & 0x8000u) != 0u) { /* +HTC (order bit on a QoS frame) */
            hdr_len += 4u;
        }
    }
    hdr_len += rx_sec_hdr_len(decr);
    if (mpdu_len < hdr_len + 8u) {
        return;
    }

    if (to_ds == 0u && from_ds != 0u) {
        dst = addr1;            /* from the AP: DA = addr1, SA = addr3 */
        src = addr3;
    } else if (to_ds != 0u && from_ds == 0u) {
        dst = addr3;
        src = addr2;
    } else if (addr4 != NULL) {
        dst = addr3;
        src = addr4;
    } else {
        dst = addr1;            /* IBSS-style */
        src = addr2;
    }

    body = mpdu + hdr_len;
    body_len = mpdu_len - hdr_len;

    if (amsdu) {
        /* The vendor's workaround for routers (NETGEAR R7000) that set the
         * A-MSDU bit on a plain MSDU: if the body starts like an LLC/SNAP
         * header rather than a subframe header, treat it as one MSDU. */
        if (body[0] == 0xAAu && body[1] == 0xAAu && body[6] > 0x06u) {
            amsdu = false;
        }
    }
    if (!amsdu) {
        rx_deliver_llc(dst, src, body, body_len);
        return;
    }

    /* A-MSDU: a chain of DA(6) SA(6) length(2, big-endian) subframes, each
     * padded to a 4-byte boundary except the last, carrying an
     * LLC/SNAP-encapsulated MSDU. */
    while (body_len >= 14u) {
        const uint8_t *sub_da = body;
        const uint8_t *sub_sa = body + 6u;
        uint32_t sub_len = be16(body + 12u);
        uint32_t padded = (14u + sub_len + 3u) & ~3u;

        if (sub_len < 8u || sub_len > (body_len - 14u)) {
            break; /* malformed: stop rather than walk out of the MPDU */
        }
        rx_deliver_llc(sub_da, sub_sa, body + 14u, sub_len);
        if (padded >= body_len) {
            break;
        }
        body += padded;
        body_len -= padded;
    }
}
