#include "AIC8800.h"
#include "AIC8800_Wpa.h"

/*
 * AIC8800.c -- USB transport, message engine, RX dispatch and the
 * driver_nic_t surface of the AIC8800D80 driver module.
 *
 * The module is one .ELF (DEVICE_TYPE_NIC) matched against the two USB
 * identities an AIC8800D80 unit presents during its life:
 *
 *   a69c:8d80 (also 368b:8d80)  the ROM's "loader" image. It speaks only
 *                               the TASK_DBG message set -- raw memory
 *                               reads/writes -- which AIC8800_Fw.c uses to
 *                               push the vendor firmware into RAM.
 *   a69c:8d83 (also 368b:8d81/8d83) the same chip after DBG_START_APP_REQ has
 *                               run the downloaded firmware: a full-MAC
 *                               Wi-Fi device speaking TASK_MM/SCANU/SM.
 *
 * Both identities are handled by this one module so the hand-over needs no
 * second .ELF and no state outside this driver. probe() classifies the unit
 * by product id, and since USB_Main.c reports neither a remove nor a
 * re-add for vendor-specific interfaces, a probe() for the 8d81 identity
 * simply re-initializes the transport for the new address/endpoints and
 * starts the station bring-up over there (see AIC8800_Bring.c).
 *
 * Everything here was written against AICSemi's vendor driver (aic_load_fw/
 * and aic8800_fdrv/) with the wx_supplicant 2.10 reference for the RSN part
 * -- see AIC8800_Protocol.h for the message/framing authority and
 * AIC8800_Wpa.h for the handshake one.
 *
 * Threading: every entry point runs on the kernel's syscall/network-poll
 * thread. The only entry point the kernel calls from an interrupt is
 * is_ready() (network_main.c asks it on every timer tick), which is
 * therefore a pure state read. Nothing here locks: the state is touched by
 * one thread, and the RX path guards itself against re-entry instead
 * (g_aic.rx_busy) because aic_cmd() services the endpoint while it waits.
 */

/* ---- USB identities ---- */

#define AIC_USB_VENDOR_ID_AIC       0xA69Cu
#define AIC_USB_VENDOR_ID_AIC_V2    0x368Bu
#define AIC_USB_PID_LOADER          0x8D80u
#define AIC_USB_PID_FDRV            0x8D83u
#define AIC_USB_IFACE_CLASS_VENDOR  0xFFu

/* usb_submit_bulk()'s pid argument (AIC_USB_PID_IN/OUT) lives in AIC8800.h,
 * shared with AIC8800_Data.c's TX path. */

/* How long an opportunistic sync read of the IN endpoint may spin before it
 * gives up. Only the *fallback* path pays this -- see aic_rx_service(). */
#define AIC_RX_SYNC_TIMEOUT_MS      2u
/* Inside aic_cmd()'s wait the reply is expected, so a longer window is
 * right: the transfer still returns the moment data arrives. */
#define AIC_CMD_RX_TIMEOUT_MS       50u

/* Latency between completion checks while aic_cmd() waits for its reply. */
#define AIC_CMD_WAIT_SLEEP_MS       1u

aic_state_t g_aic;
const driver_binary_t *g_aic_api = NULL;

/* Static, not on the syscall dispatcher's stack (see the size note on
 * AIC_CMD_BUF_MAX): the sync fallback's landing buffer, and the scratch the
 * RX parse walks. */
static uint8_t g_rx_buf[AIC_RX_BUF_MAX];

/* ---- logging ---- */

void aic_log(const char *msg)
{
    if (g_aic_api != NULL && g_aic_api->serial_write_string != NULL) {
        g_aic_api->serial_write_string(msg);
    }
}

void aic_log_hex32(uint32_t value)
{
    if (g_aic_api != NULL && g_aic_api->serial_write_uint32 != NULL) {
        g_aic_api->serial_write_uint32(value);
    }
}

/* ---- time ---- */

uint64_t aic_now_ticks(void)
{
    if (g_aic_api == NULL || g_aic_api->timer_ticks == NULL) {
        return 0u;
    }
    return g_aic_api->timer_ticks();
}

/* Milliseconds elapsed since `since_ticks`, or `fallback` when the timer HAL
 * cannot answer -- callers pass their own timeout there, so a system with no
 * usable clock fails its waits instead of spinning forever. */
uint32_t aic_ms_since(uint64_t since_ticks, uint32_t fallback)
{
    if (g_aic_api == NULL || g_aic_api->timer_ticks == NULL ||
        g_aic_api->timer_hz == NULL) {
        return fallback;
    }
    uint32_t hz = g_aic_api->timer_hz();
    if (hz == 0u) {
        return fallback;
    }
    uint64_t now = g_aic_api->timer_ticks();
    uint64_t delta = (now >= since_ticks) ? (now - since_ticks) : 0u;
    /* Clamp before the multiply so a bogus tick value cannot wrap. */
    if (delta > 0x00000000FFFFFFFFu) {
        return fallback;
    }
    return (uint32_t)((delta * 1000u) / (uint64_t)hz);
}

void aic_msleep(uint32_t ms)
{
    if (g_aic_api != NULL && g_aic_api->timer_msleep != NULL && ms != 0u) {
        g_aic_api->timer_msleep(ms);
    }
}

/* ---- transport ---- */

bool aic_usb_bulk(uint8_t endpoint, uint16_t mps, uint8_t pid,
                  void *data, uint32_t length)
{
    if (!g_aic.attached ||
        (g_aic.submit_bulk == NULL && g_aic.submit_bulk_timeout == NULL)) {
        return false;
    }
    if (g_aic.submit_bulk_timeout != NULL) {
        return g_aic.submit_bulk_timeout(g_aic.addr, endpoint, mps, pid,
                                         data, length,
                                         (pid == AIC_USB_PID_IN)
                                             ? AIC_RX_SYNC_TIMEOUT_MS
                                             : AIC_CMD_TIMEOUT_MS);
    }
    if (g_aic.submit_bulk != NULL) {
        return g_aic.submit_bulk(g_aic.addr, endpoint, mps, pid, data, length);
    }
    return false;
}

/* ---- command engine ---- */

/* aic_cmd_nowait()'s framing is described at the top of AIC8800_Protocol.h:
 * [u16 len][0x11][0][4 pad][lmac_msg][param], where len counts everything
 * after the first four bytes. */
bool aic_cmd_nowait(uint16_t msg_id, uint16_t dest_id,
                    const void *param, uint16_t param_len)
{
    static uint8_t tx[AIC_CMD_BUF_MAX];

    if (!g_aic.attached || g_aic.ep_out == 0u) {
        return false;
    }
    if ((uint32_t)param_len + 16u > sizeof(tx)) {
        return false;
    }

    /* The header's length field counts the bytes after the 4-byte USB
     * header (the same convention the RX side uses): 4 filler + 8 bytes of
     * lmac_msg + the parameter block. The transfer itself pushes
     * param_len + 16 bytes, which is 4 more (AIC8800_Protocol.h). */
    tx[0] = (uint8_t)((param_len + 12u) & 0xFFu);
    tx[1] = (uint8_t)(((param_len + 12u) >> 8) & 0x0Fu);
    tx[2] = AIC_USB_TYPE_CFG_CMD;
    tx[3] = 0u;
    aic_memset(tx + 4u, 0u, 4u);
    aic_wr16(tx + 8u, msg_id);
    aic_wr16(tx + 10u, dest_id);
    aic_wr16(tx + 12u, (uint16_t)AIC_DRV_TASK_ID);
    aic_wr16(tx + 14u, param_len);
    if (param_len != 0u) {
        aic_memcpy(tx + 16u, param, param_len);
    }

    return aic_usb_bulk(g_aic.ep_out, g_aic.ep_out_mps, AIC_USB_PID_OUT,
                        tx, (uint32_t)param_len + 16u);
}

/* ---- RX ---- */

/* Defined below, next to aic_rx_service(): aic_cmd() has to run it while it
 * waits for a reply, and the dispatch pushes data frames up into the
 * kernel's network stack through g_aic.rx_callback. */
static bool aic_rx_service(void);

/* Everything the IN endpoint can hand up, in the framing AIC8800_Protocol.h
 * documents: a run of sub-packets, each with its own [u16 len][u8 type]
 * header. Bounded in both directions -- at most AIC_RX_MAX_SUBPKTS per
 * transfer, and any sub-packet whose length does not fit in the buffer ends
 * the walk (there is no length-of-transfer report to fall back on; see the
 * note on aic_rx_service()). */
#define AIC_RX_MAX_SUBPKTS          16u

static void aic_handle_cfg_rsp(const uint8_t *payload, uint16_t payload_len)
{
    uint16_t id;
    uint16_t param_len;
    const uint8_t *param;

    if (payload_len < AIC_IPC_E2A_MSG_HDR_LEN) {
        return;
    }
    if (aic_rd32(payload + 8u) != AIC_IPC_E2A_MSG_PATTERN) {
        /* ipc_e2a_msg's own validity marker: a truncated or misaligned
         * packet can never be mistaken for a reply. */
        return;
    }
    id = aic_rd16(payload + 0u);
    param_len = aic_rd16(payload + 6u);
    if ((uint32_t)AIC_IPC_E2A_MSG_HDR_LEN + param_len > payload_len) {
        return;
    }
    param = payload + AIC_IPC_E2A_MSG_HDR_LEN;

    if (g_aic.wait_cfm_id != 0u && id == g_aic.wait_cfm_id &&
        g_aic.cfm_received == false) {
        uint32_t copy = param_len;
        if (copy > g_aic.cfm_cap) {
            copy = g_aic.cfm_cap;
        }
        if (copy != 0u && g_aic.cfm_buf != NULL) {
            aic_memcpy(g_aic.cfm_buf, param, copy);
        }
        g_aic.cfm_len = (uint16_t)copy;
        g_aic.cfm_received = true;
        return;
    }

    /* Unsolicited messages: the async events this driver acts on. Anything
     * else (CFMs for fire-and-forget requests, ME/TDLS chatter) is dropped
     * on purpose. */
    switch (id) {
    case AIC_SCANU_RESULT_IND:
        aic_handle_scan_result(param, param_len);
        break;
    case AIC_SM_CONNECT_IND:
        aic_handle_connect_ind(param, param_len);
        break;
    case AIC_SM_DISCONNECT_IND:
        aic_handle_disconnect_ind(param, param_len);
        break;
    default:
        break;
    }
}

/* One RX packet buffer (starting at the 4-byte USB header). DATA packets
 * have their MPDU at offset AIC_RX_HWHRD_LEN; CFG packets carry an
 * ipc_e2a_msg right after the header. */
static void aic_handle_rx_packet(const uint8_t *pkt, uint32_t pkt_total_len)
{
    uint8_t type = pkt[2];

    if (type == AIC_USB_TYPE_CFG_CMD_RSP) {
        uint32_t avail = pkt_total_len - AIC_USB_HDR_LEN;
        uint16_t pkt_len = aic_rd16(pkt);
        /* The header's length excludes the 4-byte USB header and the
         * ALIGN(,4) padding this walk added, so it is the payload bound
         * (and the truncation detector: a short value is what a truncated
         * transfer looks like). */
        uint16_t payload_len = (pkt_len <= avail) ? pkt_len : (uint16_t)avail;
        aic_handle_cfg_rsp(pkt + AIC_USB_HDR_LEN, payload_len);
        return;
    }
    if (type == AIC_USB_TYPE_CFG_DATA_CFM) {
        return; /* TX credit, see aic_tx_eth(); nothing to correlate yet */
    }
    /* DATA */
    aic_rx_data_packet(pkt, (uint16_t)pkt_total_len);
}

static void aic_rx_parse(const uint8_t *buf, uint32_t buf_len)
{
    uint32_t pos = 0u;
    uint32_t subpkts = 0u;

    while (pos + AIC_USB_HDR_LEN <= buf_len && subpkts < AIC_RX_MAX_SUBPKTS) {
        uint16_t pkt_len = aic_rd16(buf + pos);
        uint8_t type = buf[pos + 2u];
        uint32_t chunk;

        if (pkt_len == 0u) {
            break; /* nothing more in this transfer */
        }
        if ((type & AIC_USB_TYPE_CFG) != 0u) {
            /* CFG_CMD_RSP/CFG_DATA_CFM: ALIGN(pkt_len + 4, 4) bytes. */
            chunk = ((uint32_t)pkt_len + 4u + 3u) & ~3u;
        } else {
            /* DATA: hw_rxhdr + pkt_len bytes of MPDU. */
            chunk = (uint32_t)pkt_len + (uint32_t)AIC_RX_HWHRD_LEN;
        }
        if (chunk > buf_len - pos) {
            /* Truncated: a transfer has no length report in this kernel
             * (usb_submit_bulk() returns a bool, not an actual_length), so
             * a sub-packet that claims more bytes than the buffer holds is
             * simply dropped. The next transfer resynchronizes on the next
             * header, which is what the vendor driver does too. */
            break;
        }

        aic_handle_rx_packet(buf + pos, chunk);
        pos += chunk;
        ++subpkts;
    }
}

static bool aic_rx_submit_async(void)
{
    if (!g_aic.rx_async || g_aic.submit_in_async == NULL ||
        g_aic.rx_dma == NULL || g_aic.rx_dma_phys == 0u) {
        return false;
    }
    return g_aic.submit_in_async(g_aic.addr, g_aic.ep_in, g_aic.ep_in_mps,
                                 g_aic.rx_dma, g_aic.rx_dma_phys,
                                 (uint16_t)AIC_RX_BUF_MAX);
}

/* Service the IN endpoint once. Returns true when a transfer completed, i.e.
 * when the device had something to say.
 *
 * Two modes, never mixed on one endpoint (the completion bookkeeping in
 * XHCI.c is per endpoint, so interleaving an async transfer with a sync one
 * could attribute a completion to the wrong caller):
 *
 *   async (preferred) -- a DMA transfer is posted and each call only tests
 *     for its completion. Posting costs one ring doorbell, checking costs a
 *     flag read, so polling an idle link is free: this is what keeps the
 *     syscall path from paying a bulk-transfer timeout on every poll.
 *   sync -- one bounded bulk read into g_rx_buf. Used until the endpoint's
 *     transfer ring exists (a probe's very first reads) and whenever the
 *     async API or a DMA buffer is unavailable.
 */
static bool aic_rx_service(void)
{
    bool got = false;

    if (!g_aic.attached || g_aic.ep_in == 0u ||
        (g_aic.submit_bulk == NULL && g_aic.submit_bulk_timeout == NULL)) {
        return false;
    }
    if (g_aic.rx_busy) {
        return false; /* the dispatch below may already be running */
    }
    g_aic.rx_busy = true;

    if (g_aic.rx_async) {
        if (g_aic.rx_pending) {
            int ev = g_aic.check_in_event(g_aic.addr, g_aic.ep_in);
            if (ev > 0) {
                g_aic.rx_pending = false;
                aic_rx_parse(g_aic.rx_dma, AIC_RX_BUF_MAX);
                /* There is no transfer-length report, so the walk relies on
                 * each packet's own length field and on the zero terminator
                 * a short transfer leaves behind. Clear the buffer before
                 * re-arming it, or the previous transfer's bytes would sit
                 * behind the last packet of a shorter one and be replayed
                 * as duplicates. */
                aic_memset(g_aic.rx_dma, 0u, AIC_RX_BUF_MAX);
                got = true;
            } else if (ev < 0) {
                /* The endpoint was reset (or the device unplugged);
                 * check_interrupt_event() already tried to recover it. */
                g_aic.rx_pending = false;
            }
        }
        if (!g_aic.rx_pending) {
            g_aic.rx_pending = aic_rx_submit_async();
        }
        if (got) {
            g_aic.rx_busy = false;
            return true;
        }
        if (g_aic.rx_pending) {
            g_aic.rx_busy = false;
            return false; /* armed and still in flight */
        }
        /* Falling through means the async submit keeps failing: fall back
         * to sync reads for good. */
        g_aic.rx_async = false;
    }

    /* Same truncation story as the async path: clear before the read so a
     * short transfer cannot expose the previous one's bytes to the walk. */
    aic_memset(g_rx_buf, 0u, sizeof(g_rx_buf));
    if (aic_usb_bulk(g_aic.ep_in, g_aic.ep_in_mps, AIC_USB_PID_IN,
                     g_rx_buf, sizeof(g_rx_buf))) {
        /* The ring now exists, so a later async transfer lands on the same
         * (bulk) endpoint configuration instead of re-configuring it. */
        g_aic.rx_ring_ready = true;
        aic_rx_parse(g_rx_buf, sizeof(g_rx_buf));
        got = true;
    }

    g_aic.rx_busy = false;
    return got;
}

/* Enable the async pump once the endpoint is known-good (one sync transfer
 * has run) and a DMA buffer exists. Called from aic_tick(). */
static void aic_rx_enable_async(void)
{
    if (g_aic.rx_async || !g_aic.attached || !g_aic.rx_ring_ready) {
        return;
    }
    if (g_aic.submit_in_async == NULL || g_aic.check_in_event == NULL) {
        return;
    }
    if (g_aic.rx_dma == NULL) {
        if (g_aic_api->dma_alloc == NULL) {
            return;
        }
        g_aic.rx_dma = (uint8_t *)g_aic_api->dma_alloc(AIC_RX_BUF_MAX,
                                                       &g_aic.rx_dma_phys);
        if (g_aic.rx_dma == NULL || g_aic.rx_dma_phys == 0u) {
            if (g_aic.rx_dma != NULL && g_aic_api->dma_free != NULL) {
                g_aic_api->dma_free(g_aic.rx_dma, AIC_RX_BUF_MAX);
            }
            g_aic.rx_dma = NULL;
            g_aic.rx_dma_phys = 0u;
            return;
        }
    }
    g_aic.rx_async = true;
}

static void aic_rx_free_dma(void)
{
    /* An armed async transfer writes into the buffer when it completes, so
     * the buffer may only be released once nothing is posted against it.
     * check_interrupt_event() is the only way to ask (the endpoint is the
     * old one, but the query is by address/endpoint and harmless): a
     * completion or an error must have happened for the transfer to be
     * gone. A transfer that is still in flight at probe() time leaks the
     * buffer on purpose -- 2 KiB, on a path that only runs when the device
     * re-enumerates under a live transfer, is a far smaller problem than a
     * controller writing into memory the DMA allocator has since handed to
     * something else. */
    bool armed = g_aic.rx_pending;
    if (armed && g_aic.check_in_event != NULL && g_aic.attached) {
        int ev = g_aic.check_in_event(g_aic.addr, g_aic.ep_in);
        armed = (ev == 0);
    }
    if (g_aic.rx_dma != NULL && g_aic_api != NULL &&
        g_aic_api->dma_free != NULL && !armed) {
        g_aic_api->dma_free(g_aic.rx_dma, AIC_RX_BUF_MAX);
    } else if (g_aic.rx_dma != NULL && armed) {
        aic_log("[AIC8800] rx DMA buffer kept: transfer still in flight\n");
    }
    g_aic.rx_dma = NULL;
    g_aic.rx_dma_phys = 0u;
    g_aic.rx_async = false;
    g_aic.rx_pending = false;
    g_aic.rx_ring_ready = false;
}

bool aic_cmd(uint16_t msg_id, uint16_t expect_cfm_id, uint16_t dest_id,
             const void *param, uint16_t param_len,
             void *cfm_out, uint16_t cfm_cap, uint32_t timeout_ms)
{
    bool sent;
    bool ok = false;
    uint64_t started;

    if (expect_cfm_id == 0u || timeout_ms == 0u) {
        return aic_cmd_nowait(msg_id, dest_id, param, param_len);
    }
    /* Never from inside the RX dispatch (aic_cmd would have to service the
     * same endpoint the dispatch is already reading), and never nested. */
    if (!g_aic.attached || g_aic.rx_busy || g_aic.cmd_busy) {
        return false;
    }

    g_aic.cmd_busy = true;
    g_aic.wait_cfm_id = expect_cfm_id;
    g_aic.cfm_buf = cfm_out;
    g_aic.cfm_cap = cfm_cap;
    g_aic.cfm_len = 0u;
    g_aic.cfm_received = false;

    sent = aic_cmd_nowait(msg_id, dest_id, param, param_len);
    if (sent) {
        started = aic_now_ticks();
        while (!g_aic.cfm_received) {
            (void)aic_rx_service();
            if (g_aic.cfm_received) {
                break;
            }
            if (aic_ms_since(started, timeout_ms) >= timeout_ms) {
                break;
            }
            aic_msleep(AIC_CMD_WAIT_SLEEP_MS);
        }
        ok = g_aic.cfm_received;
    }

    g_aic.cmd_busy = false;
    g_aic.wait_cfm_id = 0u;
    g_aic.cfm_buf = NULL;
    g_aic.cfm_cap = 0u;
    return ok;
}

void aic_pump(uint32_t max_transfers)
{
    if (!g_aic.attached || g_aic.cmd_busy || max_transfers == 0u) {
        return;
    }
    for (uint32_t done = 0u; done < max_transfers; ++done) {
        if (!aic_rx_service()) {
            break; /* nothing left in the endpoint */
        }
    }
}

/* ---- probe / remove ---- */

/* A stable-but-not-vendor MAC would be ideal here (one that survives
 * reboots), and the chip's efuse MAC would be it -- but the efuse is only
 * readable once the firmware runs, and the kernel caches the MAC it reads
 * from get_mac() at boot, before the firmware has been downloaded. So the
 * driver picks one locally-administered address at attach time and uses it
 * consistently everywhere (get_mac(), MM_ADD_IF, the supplicant's PTK
 * derivation), which is what keeps the host stack and the firmware
 * agreeing on who this station is. */
static void aic_make_local_mac(uint8_t mac[6])
{
    uint64_t seed = aic_now_ticks();
    /* Mix in the attach instance so two dongles (and two boots) differ. */
    seed ^= ((uint64_t)g_aic.addr << 32) | (uint64_t)g_aic.interface;
    seed ^= (uint64_t)(g_aic.ep_in | ((uint16_t)g_aic.ep_out << 8));

    for (uint32_t i = 0u; i < 6u; ++i) {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        mac[i] = (uint8_t)(seed >> 24);
    }
    mac[0] = (uint8_t)((mac[0] & 0xFCu) | 0x02u); /* locally administered, unicast */
    if (mac[0] == 0u && mac[1] == 0u && mac[2] == 0u &&
        mac[3] == 0u && mac[4] == 0u && mac[5] == 0u) {
        mac[0] = 0x02u;
        mac[5] = 0x01u;
    }
}

static bool aic_probe(const bus_device_t *dev)
{
    const usb_device_context_t *ctx;
    aic_stage_t stage;

    if (g_aic_api == NULL || dev == NULL || dev->bus_context == NULL) {
        return false;
    }

    /* Stage from the product id: the ROM's loader image is 8d80, and any
     * other product id on this vendor's USB interface is a unit that is
     * already running firmware -- either the vendor's factory image or the
     * one this driver downloaded on a previous attach (the chip keeps its
     * RAM until it loses power, so a warm reboot comes back as 8d81). The
     * rest of the vendor's product-id range (8d82/8d83/8d8x, on either
     * vendor id) lands in the same bucket on purpose: the two ladders are
     * distinguishable by the messages the device answers, and the one that
     * risks harm is pushing firmware into a *live* chip -- which only ever
     * happens on the 8d80 path, i.e. exactly the case this classification
     * is sure about. */
    stage = (dev->device_id == AIC_USB_PID_LOADER) ? AIC_STAGE_LOADER
                                                   : AIC_STAGE_FDRV;

    /* An AIC8800D80 presents more than one vendor-specific interface (the
     * Wi-Fi one and, once its Bluetooth side is running, a second with its
     * own bulk pair), and USB_Main.c reports each of them. Only the first
     * one is ours: a probe for the same device address while this driver
     * is already attached to it is the second interface (or an enumeration
     * replay), and re-initializing the transport under a live association
     * would break it. A probe that moves the attach from the loader stage
     * to the firmware stage, or that carries a different address (a
     * re-plug, which USB_Main.c's allocator gives a fresh address), is the
     * real hand-over and is taken. */
    ctx = (const usb_device_context_t *)dev->bus_context;
    if (ctx->ep_in == 0u || ctx->ep_out == 0u ||
        (ctx->submit_bulk == NULL && ctx->submit_bulk_timeout == NULL)) {
        return false;
    }

    if (g_aic.attached && ctx->addr == g_aic.addr &&
        !(g_aic.stage == AIC_STAGE_LOADER && stage == AIC_STAGE_FDRV)) {
        aic_log("[AIC8800] another vendor interface at addr ");
        aic_log_hex32((uint32_t)ctx->addr);
        aic_log(" (iface ");
        aic_log_hex32((uint32_t)ctx->interface);
        aic_log(") ignored -- already driving this device\n");
        return false;
    }

    /* Release anything the previous instance owned before overwriting the
     * state (the loader identity's DMA buffer, if a re-probe happens while
     * the async pump is running). */
    aic_rx_free_dma();

    aic_memset(&g_aic, 0u, sizeof(g_aic));
    g_aic.attached = true;
    g_aic.pid = dev->device_id;
    g_aic.addr = ctx->addr;
    g_aic.interface = ctx->interface;
    g_aic.ep_in = ctx->ep_in;
    g_aic.ep_out = ctx->ep_out;
    g_aic.ep_in_mps = ctx->ep_in_mps;
    g_aic.ep_out_mps = ctx->ep_out_mps;
    g_aic.ep_out_data = (ctx->ep_out_data != 0u) ? ctx->ep_out_data
                                                  : ctx->ep_out;
    g_aic.ep_out_data_mps = (ctx->ep_out_data_mps != 0u)
                                ? ctx->ep_out_data_mps
                                : ctx->ep_out_mps;
    g_aic.submit_bulk = ctx->submit_bulk;
    g_aic.submit_bulk_timeout = ctx->submit_bulk_timeout;
    g_aic.submit_in_async = ctx->submit_interrupt_in_async;
    g_aic.check_in_event = ctx->check_interrupt_event;
    g_aic.stage = stage;
    g_aic.conn = AIC_CONN_NONE;
    aic_make_local_mac(g_aic.mac);

    aic_fw_reset();
    aic_bring_reset();

    aic_log("[AIC8800] ");
    aic_log(stage == AIC_STAGE_LOADER ? "loader-stage device at usb addr "
                                      : "firmware-stage device at usb addr ");
    aic_log_hex32((uint32_t)g_aic.addr);
    aic_log(" iface ");
    aic_log_hex32((uint32_t)g_aic.interface);
    aic_log(" pid 0x");
    aic_log_hex32((uint32_t)g_aic.pid);
    aic_log("\n");
    return true;
}

static void aic_remove(const bus_device_t *dev)
{
    (void)dev;
    aic_rx_free_dma();
    if (g_aic_api != NULL) {
        aic_memset(&g_aic, 0u, sizeof(g_aic));
    }
}

/* ---- state machine ---- */

/* One step of whatever the driver is currently doing. Every kernel entry
 * point runs this first (see aic_tick()). */
void aic_tick(void)
{
    if (!g_aic.attached) {
        return;
    }

    switch (g_aic.stage) {
    case AIC_STAGE_LOADER:
        /* aic_fw_step() services the IN endpoint through aic_cmd() while it
         * waits for each DBG_MEM_* confirmation. */
        aic_fw_step();
        break;
    case AIC_STAGE_FDRV:
        if (!g_aic.ready) {
            aic_bring_step();
        } else {
            aic_rx_enable_async();
            aic_pump(AIC_RX_PER_TICK);
            aic_conn_tick();
            aic_wpa_tick();
        }
        break;
    default:
        break;
    }
}

/* ---- driver_nic_t ---- */

static bool aic_nic_init(void)
{
    /* "Is there a device to drive": the kernel asks this once, at
     * ethernet_init(), and picks the first NIC that says yes. The firmware
     * is not up yet at that point -- aic_tick() below and the driver's own
     * poll() take it from there. */
    return g_aic.attached;
}

static bool aic_nic_is_ready(void)
{
    /* Called from the timer interrupt (network_main.c's DHCP/link check),
     * so it must not touch the USB endpoint. Association counts as link-up
     * only once the WPA2 keys are installed -- DHCP must not start before
     * the data path can carry its frames. */
    return g_aic.attached && g_aic.conn == AIC_CONN_ASSOCIATED;
}

static uint16_t aic_nic_mtu(void)
{
    return AIC_MAX_MTU;
}

static void aic_nic_get_mac(uint8_t mac_out[6])
{
    if (mac_out == NULL) {
        return;
    }
    aic_memcpy(mac_out, g_aic.mac, 6u);
}

static bool aic_nic_send_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len == 0u) {
        return false;
    }
    return aic_tx_eth(frame, frame_len);
}

static void aic_nic_poll(void)
{
    aic_tick();
}

static void aic_nic_set_rx_callback(driver_nic_rx_callback_t cb)
{
    g_aic.rx_callback = cb;
}

static bool aic_wifi_scan_start(void)
{
    aic_tick();
    return aic_scan_start();
}

static uint32_t aic_wifi_get_scan_results(driver_wifi_scan_result_t *out,
                                          uint32_t max_count)
{
    aic_tick();
    return aic_scan_results_copy(out, max_count);
}

static bool aic_wifi_connect(const char *ssid, const char *psk)
{
    aic_tick();
    return aic_connect(ssid, psk);
}

static void aic_wifi_disconnect(void)
{
    aic_tick();
    aic_disconnect();
}

static void aic_wifi_get_status(driver_wifi_status_t *out_status)
{
    if (out_status == NULL) {
        return;
    }
    aic_tick();

    aic_memset(out_status, 0u, sizeof(*out_status));
    switch (g_aic.stage) {
    case AIC_STAGE_LOADER:
        out_status->state =
            (g_aic.fw_phase == AIC_FW_FAILED)
                ? (uint32_t)DRIVER_WIFI_STATE_FIRMWARE_FAILED
                : (uint32_t)DRIVER_WIFI_STATE_FIRMWARE_LOADING;
        break;
    case AIC_STAGE_FDRV:
        if (!g_aic.ready) {
            out_status->state =
                (g_aic.bring_phase == AIC_BRING_FAILED)
                    ? (uint32_t)DRIVER_WIFI_STATE_FIRMWARE_FAILED
                    : (uint32_t)DRIVER_WIFI_STATE_ADAPTER_ATTACHED;
        } else if (g_aic.conn == AIC_CONN_ASSOCIATED) {
            out_status->state = (uint32_t)DRIVER_WIFI_STATE_ASSOCIATED;
        } else if (g_aic.conn == AIC_CONN_FAILED) {
            out_status->state = (uint32_t)DRIVER_WIFI_STATE_CONNECT_FAILED;
        } else if (g_aic.ssid[0] != '\0') {
            out_status->state = (uint32_t)DRIVER_WIFI_STATE_CONNECTING;
        } else if (g_aic.scanning) {
            out_status->state = (uint32_t)DRIVER_WIFI_STATE_SCANNING;
        } else {
            out_status->state = (uint32_t)DRIVER_WIFI_STATE_READY;
        }
        break;
    default:
        out_status->state = (uint32_t)DRIVER_WIFI_STATE_NO_ADAPTER;
        break;
    }

    aic_memcpy(out_status->ssid, g_aic.ssid, sizeof(out_status->ssid));
    aic_memcpy(out_status->mac, g_aic.mac, 6u);
}

/* ---- module wiring ---- */

/* A vendor-specific (class 0xFF) interface on either AICSemi USB vendor id.
 * The product id is deliberately NOT part of the match: one unit presents
 * 8d80 before DBG_START_APP_REQ and 8d81 after, and possibly 8d83/8d84/8d85
 * on other SKUs -- probe() above classifies all of them. Two entries because
 * match_flags cannot OR two vendor ids together. */
static const driver_bus_match_t g_aic_bus_matches[] = {
    {
        .bus_type = DEVICE_TYPE_USB,
        .vendor_id = (uint16_t)AIC_USB_VENDOR_ID_AIC,
        .class_code = AIC_USB_IFACE_CLASS_VENDOR,
        .match_flags = DRIVER_BUS_MATCH_VENDOR | DRIVER_BUS_MATCH_CLASS,
    },
    {
        .bus_type = DEVICE_TYPE_USB,
        .vendor_id = (uint16_t)AIC_USB_VENDOR_ID_AIC_V2,
        .class_code = AIC_USB_IFACE_CLASS_VENDOR,
        .match_flags = DRIVER_BUS_MATCH_VENDOR | DRIVER_BUS_MATCH_CLASS,
    },
};

static const driver_nic_t g_aic_nic_driver = {
    .init = aic_nic_init,
    .is_ready = aic_nic_is_ready,
    .mtu = aic_nic_mtu,
    .get_mac = aic_nic_get_mac,
    .send_frame = aic_nic_send_frame,
    .poll = aic_nic_poll,
    .set_rx_callback = aic_nic_set_rx_callback,

    .wifi_scan_start = aic_wifi_scan_start,
    .wifi_get_scan_results = aic_wifi_get_scan_results,
    .wifi_connect = aic_wifi_connect,
    .wifi_disconnect = aic_wifi_disconnect,
    .wifi_get_status = aic_wifi_get_status,
};

static void aic_shutdown(void)
{
    aic_rx_free_dma();
    if (g_aic_api != NULL) {
        aic_memset(&g_aic, 0u, sizeof(g_aic));
    }
    g_aic_api = NULL;
}

static const driver_module_descriptor_t g_aic_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_NIC,
    .load_priority = 55u,
    .deps = { "USB_Driver.ELF", NULL },
    .driver_api = &g_aic_nic_driver,
    .shutdown = aic_shutdown,
    .bus_matches = g_aic_bus_matches,
    .bus_match_count = (uint32_t)(sizeof(g_aic_bus_matches) /
                                  sizeof(g_aic_bus_matches[0])),
    .probe = aic_probe,
    .remove = aic_remove,
};

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL || api->memcpy == NULL || api->memset == NULL ||
        api->timer_ticks == NULL || api->timer_hz == NULL ||
        api->fs.find_file == NULL || api->fs.read_at == NULL ||
        api->fs.get_file_size == NULL || api->fs.close_file == NULL) {
        return NULL;
    }
    g_aic_api = api;
    return &g_aic_module;
}
