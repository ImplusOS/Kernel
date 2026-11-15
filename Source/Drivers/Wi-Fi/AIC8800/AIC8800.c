#include "AIC8800.h"
#include "AIC8800_Wpa.h"

/*
 * AIC8800.c -- USB transport, message engine, RX dispatch and the
 * driver_nic_t surface of the AIC8800D80 driver module.
 *
 * The module is one .ELF (DEVICE_TYPE_NIC) matched against the USB
 * identities an AIC8800D80 unit presents during its life:
 *
 *   a69c:8d80 (also 8d40)       the ROM's "loader" image. It speaks only
 *                               the TASK_DBG message set -- raw memory
 *                               reads/writes -- which AIC8800_Fw.c uses to
 *                               push the vendor firmware into RAM.
 *   a69c/368b:8d81 and friends  the same chip after DBG_START_APP_REQ has
 *                               run the downloaded firmware: a full-MAC
 *                               Wi-Fi device speaking TASK_MM/ME/SCANU/SM.
 *                               Branded units re-enumerate under their own
 *                               vendor id here (TP-Link 2357, Tenda 2604);
 *                               aic_is_fdrv_identity() has the full list,
 *                               which is the vendor fdrv's PRODUCT_ID_AIC8800D81
 *                               set.
 *
 * Both identities are handled by this one module so the hand-over needs no
 * second .ELF and no state outside this driver. probe() classifies the unit
 * by vendor/product id, and since USB_Main.c reports neither a remove nor a
 * re-add for vendor-specific interfaces, a probe() for the firmware identity
 * simply re-initializes the transport for the new address/endpoints and
 * starts the station bring-up over there (see AIC8800_Bring.c).
 *
 * Everything here was written against AICSemi's vendor driver (aic_load_fw/
 * and aic8800_fdrv/) with the wpa_supplicant 2.10 reference for the RSN part
 * -- see AIC8800_Protocol.h for the message/framing authority and
 * AIC8800_Wpa.h for the handshake one.
 *
 * Threading: the kernel reaches this driver from the syscall path of any
 * CPU (the Wi-Fi syscalls, the network poll the syscall dispatcher runs,
 * send_frame from the IP stack). The command engine and the RX pump are not
 * re-entrant, so every entry point takes aic_lock_try() first and skips its
 * work when another CPU holds it. is_ready() is a pure state read (the
 * timer tick asks it) and takes no lock.
 */

/* ---- USB identities ---- */

#define AIC_USB_VENDOR_ID_AIC       0xA69Cu
#define AIC_USB_VENDOR_ID_AIC_V2    0x368Bu
#define AIC_USB_VENDOR_ID_TP        0x2357u
#define AIC_USB_VENDOR_ID_TENDA     0x2604u
#define AIC_USB_PID_LOADER          0x8D80u
#define AIC_USB_PID_LOADER_D40      0x8D40u
#define AIC_USB_IFACE_CLASS_VENDOR  0xFFu

/* usb_submit_bulk()'s pid argument (AIC_USB_PID_IN/OUT) lives in AIC8800.h,
 * shared with AIC8800_Data.c's TX path. */

/* How long an opportunistic sync read of an IN endpoint may spin before it
 * gives up. Only the *fallback* path pays this -- see
 * aic_rx_service_pipe(). */
#define AIC_RX_SYNC_TIMEOUT_MS      2u

/* Latency between completion checks while aic_cmd() waits for its reply. */
#define AIC_CMD_WAIT_SLEEP_MS       1u

/* A bound on the packets one transfer can carry (the firmware aggregates
 * at most ten; the rest is slack for CFG chatter). */
#define AIC_RX_MAX_SUBPKTS          64u

/* An xHCI TRB buffer must not cross a 64 KiB boundary, and the async IN
 * path posts the whole landing buffer as one TRB. */
#define AIC_DMA_BOUNDARY            0x10000u

aic_state_t g_aic;
const driver_binary_t *g_aic_api = NULL;

/* Static, not on the syscall dispatcher's stack: the sync fallback's
 * landing buffers (one per bulk IN pipe) and the cross-transfer carry
 * buffers. */
static uint8_t g_rx_buf[AIC_RX_BUF_MAX];
static uint8_t g_rx_data_buf[AIC_RX_BUF_MAX];
static uint8_t g_rx_carry[AIC_RX_CARRY_MAX];
static uint8_t g_rx_data_carry[AIC_RX_CARRY_MAX];

static volatile int g_aic_lock;

/* ---- locking ---- */

bool aic_lock_try(void)
{
    return __sync_lock_test_and_set(&g_aic_lock, 1) == 0;
}

void aic_unlock(void)
{
    __sync_lock_release(&g_aic_lock);
}

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

static bool aic_usb_bulk_to(uint8_t endpoint, uint16_t mps, uint8_t pid,
                            void *data, uint32_t length, uint32_t timeout_ms);

bool aic_usb_bulk(uint8_t endpoint, uint16_t mps, uint8_t pid,
                  void *data, uint32_t length)
{
    return aic_usb_bulk_to(endpoint, mps, pid, data, length,
                           (pid == AIC_USB_PID_IN) ? AIC_RX_SYNC_TIMEOUT_MS
                                                   : AIC_CMD_TIMEOUT_MS);
}

/* Same, with an explicit timeout. aic_rx_service_pipe() uses this so a
 * command wait can keep its IN transfer posted for the whole reply window
 * instead of asking XHCI.c for a 2 ms transfer and making it recover the
 * endpoint on every miss; the endpoint recovery is a Stop/Reset Endpoint
 * sequence, and doing that every couple of milliseconds while a ROM is
 * still preparing its reply is how confirmations get lost. */
static bool aic_usb_bulk_to(uint8_t endpoint, uint16_t mps, uint8_t pid,
                            void *data, uint32_t length, uint32_t timeout_ms)
{
    if (!g_aic.attached || g_aic.submit_bulk_timeout == NULL) {
        /* Never fall back to submit_bulk(): its controller default is 30 s
         * on xHCI, which turns a NAKing endpoint into a boot hang. probe()
         * refuses a bus without the bounded variant. */
        return false;
    }
    return g_aic.submit_bulk_timeout(g_aic.addr, endpoint, mps, pid,
                                     data, length, timeout_ms);
}

/* ---- command engine ---- */

/* aic_cmd_nowait()'s framing is described at the top of AIC8800_Protocol.h:
 * [u16 len][0x11][0][4 pad][lmac_msg][param], where len counts everything
 * after the first four bytes (aicwf_set_cmd_tx(): len + 4 with len =
 * sizeof(lmac_msg) + param_len, and len + 8 bytes pushed). */
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

static bool aic_rx_service_pipe(aic_rx_pipe_t *pipe, uint32_t timeout_ms);

static void aic_handle_cfg_rsp(const uint8_t *payload, uint16_t payload_len)
{
    uint16_t id;
    uint16_t param_len;
    const uint8_t *param;

    if (payload_len < AIC_IPC_E2A_MSG_HDR_LEN) {
        return;
    }
    id = aic_rd16(payload + 0u);
    param_len = aic_rd16(payload + 6u);
    if ((uint32_t)AIC_IPC_E2A_MSG_HDR_LEN + param_len > payload_len) {
        /* The header's own length is the authority (the vendor does not
         * look at `pattern` either): a short message is a truncated one. */
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
    case AIC_SCANU_START_CFM:
        aic_handle_scan_done();
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

/* One RX packet (starting at its 4-byte USB header, `pkt_total_len` bytes
 * including any CFG alignment padding). */
static void aic_handle_rx_packet(const uint8_t *pkt, uint32_t pkt_total_len)
{
    uint8_t type = (uint8_t)(pkt[2] & 0x7Fu);

    if ((type & AIC_USB_TYPE_CFG) != 0u) {
        uint32_t avail = pkt_total_len - AIC_USB_HDR_LEN;
        uint16_t pkt_len = aic_rd16(pkt);
        uint16_t payload_len = (pkt_len <= avail) ? pkt_len : (uint16_t)avail;

        if (type == AIC_USB_TYPE_CFG_CMD_RSP) {
            aic_handle_cfg_rsp(pkt + AIC_USB_HDR_LEN, payload_len);
        }
        /* CFG_DATA_CFM: TX status for a frame sent with a status descriptor;
         * this driver never asks for one. CFG_PRINT: firmware log text. */
        return;
    }
    aic_rx_data_packet(pkt, pkt_total_len);
}

/* Size of the packet whose header is at `hdr`, padding included. */
static uint32_t aic_rx_packet_size(const uint8_t *hdr)
{
    uint16_t pkt_len = aic_rd16(hdr);
    uint8_t type = hdr[2];

    if ((type & AIC_USB_TYPE_CFG) != 0u) {
        /* CFG_CMD_RSP/CFG_DATA_CFM/CFG_PRINT: ALIGN(pkt_len + 4, 4). */
        return ((uint32_t)pkt_len + 4u + 3u) & ~3u;
    }
    /* DATA: hw_rxhdr (whose first word aliases this header) + MPDU. */
    return (uint32_t)pkt_len + (uint32_t)AIC_RX_HWHRD_LEN;
}

static void aic_rx_carry_reset(aic_rx_pipe_t *pipe)
{
    pipe->carry_len = 0u;
    pipe->carry_need = 0u;
}

/* Walk one completed transfer. There is no transfer-length report from the
 * bus (submit_bulk returns a bool), so the walk relies on each packet's own
 * length field and on the zero terminator a short transfer leaves behind
 * (the landing buffer is zeroed before every transfer). A packet whose
 * length runs past the end of the buffer was cut by a *full* transfer --
 * the device never ends a transfer in the middle of a packet -- so its head
 * is carried over and completed from the next transfer, which is what the
 * vendor's rx_reassemble path does too.
 *
 * Returns how many bytes of `buf` were touched, so the caller can clear
 * exactly those before re-arming the buffer. */
static uint32_t aic_rx_consume(aic_rx_pipe_t *pipe, const uint8_t *buf,
                               uint32_t buf_len)
{
    uint32_t pos = 0u;
    uint32_t subpkts = 0u;

    /* 1. Finish a packet carried over from the previous transfer. */
    if (pipe->carry_len != 0u) {
        uint32_t take;

        if (pipe->carry_need == 0u) {
            /* Not even the 4-byte header made it last time. */
            take = AIC_USB_HDR_LEN - pipe->carry_len;
            if (take > buf_len) {
                take = buf_len;
            }
            aic_memcpy(pipe->carry + pipe->carry_len, buf, take);
            pipe->carry_len += take;
            pos += take;
            if (pipe->carry_len < AIC_USB_HDR_LEN) {
                return pos;
            }
            pipe->carry_need = aic_rx_packet_size(pipe->carry);
            if (aic_rd16(pipe->carry) == 0u ||
                pipe->carry_need > AIC_RX_CARRY_MAX) {
                aic_rx_carry_reset(pipe);
                return pos; /* resynchronize on the next transfer */
            }
        }
        take = pipe->carry_need - pipe->carry_len;
        if (take > buf_len - pos) {
            take = buf_len - pos;
        }
        aic_memcpy(pipe->carry + pipe->carry_len, buf + pos, take);
        pipe->carry_len += take;
        pos += take;
        if (pipe->carry_len < pipe->carry_need) {
            return pos; /* still incomplete: this transfer was full too */
        }
        aic_handle_rx_packet(pipe->carry, pipe->carry_need);
        aic_rx_carry_reset(pipe);
    }

    /* 2. The packets that start in this transfer. */
    while (pos < buf_len && subpkts < AIC_RX_MAX_SUBPKTS) {
        uint32_t left = buf_len - pos;
        uint32_t chunk;

        if (left < AIC_USB_HDR_LEN) {
            /* Only a header fragment is left. Non-zero bytes there mean the
             * transfer was full and the header continues next time. */
            bool any = false;
            for (uint32_t i = 0u; i < left; ++i) {
                any = any || (buf[pos + i] != 0u);
            }
            if (any) {
                aic_memcpy(pipe->carry, buf + pos, left);
                pipe->carry_len = left;
                pipe->carry_need = 0u;
                pos += left;
            }
            break;
        }
        if (aic_rd16(buf + pos) == 0u) {
            break; /* the zero terminator: nothing more in this transfer */
        }
        chunk = aic_rx_packet_size(buf + pos);
        if (chunk > left) {
            if (chunk <= AIC_RX_CARRY_MAX) {
                aic_memcpy(pipe->carry, buf + pos, left);
                pipe->carry_len = left;
                pipe->carry_need = chunk;
            } else {
                aic_log("[AIC8800] rx: oversized packet dropped\n");
            }
            pos = buf_len;
            break;
        }

        aic_handle_rx_packet(buf + pos, chunk);
        pos += chunk;
        ++subpkts;
    }
    return pos;
}

static bool aic_rx_submit_async(aic_rx_pipe_t *pipe)
{
    if (!pipe->present || !pipe->use_async || g_aic.submit_in_async == NULL ||
        pipe->dma == NULL || pipe->dma_phys == 0u) {
        return false;
    }
    return g_aic.submit_in_async(g_aic.addr, pipe->ep, pipe->mps,
                                 pipe->dma, pipe->dma_phys,
                                 (uint16_t)AIC_RX_BUF_MAX);
}

/* Service one IN pipe once. Returns true when a transfer completed, i.e.
 * when the device had something to say on that pipe.
 *
 * Two modes, never mixed on one endpoint (the completion bookkeeping in
 * XHCI.c is per endpoint, so interleaving an async transfer with a sync one
 * could attribute a completion to the wrong caller):
 *
 *   async (preferred) -- a DMA transfer is posted and each call only tests
 *     for its completion. Posting costs one ring doorbell, checking costs a
 *     flag read, so polling an idle link is free.
 *   sync -- one bounded bulk read into the pipe's fallback buffer. Used
 *     until the endpoint's transfer ring exists (a probe's very first reads)
 *     and whenever the async API or a DMA buffer is unavailable.
 */
static bool aic_rx_service_pipe(aic_rx_pipe_t *pipe, uint32_t timeout_ms)
{
    bool got = false;

    if (!g_aic.attached || pipe == NULL || !pipe->present || pipe->ep == 0u ||
        g_aic.submit_bulk_timeout == NULL) {
        return false;
    }
    if (g_aic.rx_busy) {
        return false; /* the dispatch below may already be running */
    }
    g_aic.rx_busy = true;

    if (pipe->use_async) {
        if (pipe->pending) {
            int ev = g_aic.check_in_event(g_aic.addr, pipe->ep);
            if (ev > 0) {
                uint32_t used;
                pipe->pending = false;
                used = aic_rx_consume(pipe, pipe->dma, AIC_RX_BUF_MAX);
                /* Clear what the transfer wrote before re-arming it, or the
                 * previous transfer's bytes would sit behind the last packet
                 * of a shorter one and be replayed as duplicates. */
                aic_memset(pipe->dma, 0u, used);
                got = true;
            } else if (ev < 0) {
                /* The endpoint was reset (or the device unplugged): the
                 * transfer and whatever it carried are gone. */
                pipe->pending = false;
                aic_rx_carry_reset(pipe);
                aic_memset(pipe->dma, 0u, AIC_RX_BUF_MAX);
            }
        }
        if (!pipe->pending) {
            pipe->pending = aic_rx_submit_async(pipe);
        }
        if (got) {
            g_aic.rx_busy = false;
            return true;
        }
        if (pipe->pending) {
            g_aic.rx_busy = false;
            return false; /* armed and still in flight */
        }
        /* Falling through means the async submit keeps failing: fall back
         * to sync reads for good. */
        pipe->use_async = false;
        aic_log("[AIC8800] rx ep ");
        aic_log_hex32((uint32_t)pipe->ep);
        aic_log(": async submit failed, using sync reads\n");
    }

    /* Same truncation story as the async path: the buffer is all zeroes
     * here (cleared after the previous parse), so a short transfer cannot
     * expose the previous one's bytes to the walk. */
    if (aic_usb_bulk_to(pipe->ep, pipe->mps, AIC_USB_PID_IN,
                        pipe->sync_buf, AIC_RX_BUF_MAX, timeout_ms)) {
        uint32_t used = aic_rx_consume(pipe, pipe->sync_buf, AIC_RX_BUF_MAX);
        aic_memset(pipe->sync_buf, 0u, used);
        got = true;
    }
    /* Whatever the outcome, the controller has now created this endpoint's
     * ring as a bulk endpoint (it does so before queueing the transfer). */
    pipe->ring_ready = true;

    g_aic.rx_busy = false;
    return got;
}

/* A landing buffer for the async IN path that does not straddle a 64 KiB
 * boundary. dma_alloc_ex() can align it outright; plain dma_alloc() gets
 * twice the size and the window is placed inside it. */
static bool aic_rx_pipe_alloc_dma(aic_rx_pipe_t *pipe)
{
    uint64_t phys = 0u;
    uint8_t *base;
    uint32_t size;
    uint64_t off = 0u;

    if (g_aic_api->mem.dma_alloc_ex != NULL) {
        size = 0x8000u; /* 32 KiB, 32 KiB aligned: never crosses 64 KiB */
        base = (uint8_t *)g_aic_api->mem.dma_alloc_ex(size, size, 0u, &phys);
    } else {
        size = 2u * AIC_RX_BUF_MAX;
        base = (uint8_t *)g_aic_api->dma_alloc(size, &phys);
    }
    if (base == NULL || phys == 0u) {
        if (base != NULL && g_aic_api->dma_free != NULL) {
            g_aic_api->dma_free(base, size);
        }
        return false;
    }
    if ((phys / AIC_DMA_BOUNDARY) !=
        ((phys + AIC_RX_BUF_MAX - 1u) / AIC_DMA_BOUNDARY)) {
        /* Start the window at the boundary instead; it is less than one
         * window into the allocation, so the window still fits. */
        off = AIC_DMA_BOUNDARY - (phys % AIC_DMA_BOUNDARY);
        if (off + AIC_RX_BUF_MAX > size) {
            if (g_aic_api->dma_free != NULL) {
                g_aic_api->dma_free(base, size);
            }
            return false;
        }
    }
    pipe->dma_base = base;
    pipe->dma_base_size = size;
    pipe->dma = base + off;
    pipe->dma_phys = phys + off;
    /* The parser depends on a zero terminator after the last packet. */
    aic_memset(pipe->dma, 0u, AIC_RX_BUF_MAX);
    return true;
}

/* Enable the async pump for one pipe. Only once a sync transfer has
 * created the endpoint's ring: xhci_submit_interrupt_in() would otherwise
 * configure the bulk endpoint as an interrupt one. Called from the steady
 * state (aic_tick()/poll()). */
static void aic_rx_pipe_enable_async(aic_rx_pipe_t *pipe)
{
    if (!pipe->present || pipe->use_async || pipe->ep == 0u) {
        return;
    }
    if (g_aic.submit_in_async == NULL || g_aic.check_in_event == NULL ||
        g_aic_api == NULL || g_aic_api->dma_alloc == NULL) {
        return;
    }
    if (!pipe->ring_ready) {
        /* One short sync read creates the ring as bulk (and drains
         * anything already waiting). */
        (void)aic_rx_service_pipe(pipe, AIC_RX_SYNC_TIMEOUT_MS);
        if (!pipe->ring_ready) {
            return;
        }
    }
    if (pipe->dma == NULL && !aic_rx_pipe_alloc_dma(pipe)) {
        return;
    }
    pipe->use_async = true;
    aic_log("[AIC8800] rx ep ");
    aic_log_hex32((uint32_t)pipe->ep);
    aic_log(" now async\n");
}

static void aic_rx_enable_async(void)
{
    if (!g_aic.attached) {
        return;
    }
    aic_rx_pipe_enable_async(&g_aic.rx_cfg);
    aic_rx_pipe_enable_async(&g_aic.rx_data);
}

static void aic_rx_pipe_free_dma(aic_rx_pipe_t *pipe)
{
    /* An armed async transfer writes into the buffer when it completes, so
     * the buffer may only be released once nothing is posted against it.
     * A transfer that is still in flight at probe() time leaks the buffer
     * on purpose -- far less harmful than a controller writing into memory
     * the DMA allocator has since handed to something else. */
    bool armed = pipe->pending;
    if (armed && g_aic.check_in_event != NULL && g_aic.attached &&
        pipe->ep != 0u) {
        int ev = g_aic.check_in_event(g_aic.addr, pipe->ep);
        armed = (ev == 0);
    }
    if (pipe->dma_base != NULL && g_aic_api != NULL &&
        g_aic_api->dma_free != NULL && !armed) {
        g_aic_api->dma_free(pipe->dma_base, pipe->dma_base_size);
    } else if (pipe->dma_base != NULL && armed) {
        aic_log("[AIC8800] rx DMA buffer kept: transfer still in flight\n");
    }
    pipe->dma = NULL;
    pipe->dma_base = NULL;
    pipe->dma_base_size = 0u;
    pipe->dma_phys = 0u;
    pipe->use_async = false;
    pipe->pending = false;
}

static void aic_rx_free_dma(void)
{
    aic_rx_pipe_free_dma(&g_aic.rx_cfg);
    aic_rx_pipe_free_dma(&g_aic.rx_data);
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
            uint32_t elapsed = aic_ms_since(started, timeout_ms);
            if (elapsed >= timeout_ms) {
                break;
            }
            /* The reply is expected on the CFG pipe, so keep that transfer
             * posted for the rest of the command window: it completes the
             * instant the reply arrives, and a miss costs one endpoint
             * recovery instead of hundreds of them. */
            (void)aic_rx_service_pipe(&g_aic.rx_cfg, timeout_ms - elapsed);
            if (g_aic.cfm_received) {
                break;
            }
            /* Once the link is up, data shares the wait (an idle async pipe
             * costs nothing to look at). */
            if (g_aic.ready && g_aic.rx_data.present) {
                (void)aic_rx_service_pipe(&g_aic.rx_data,
                                          AIC_RX_SYNC_TIMEOUT_MS);
            }
            if (g_aic.cfm_received) {
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
        bool got_cfg = aic_rx_service_pipe(&g_aic.rx_cfg,
                                           AIC_RX_SYNC_TIMEOUT_MS);
        bool got_data = aic_rx_service_pipe(&g_aic.rx_data,
                                            AIC_RX_SYNC_TIMEOUT_MS);
        if (!got_cfg && !got_data) {
            break; /* nothing left in either endpoint */
        }
    }
}

/* ---- probe / remove ---- */

/* The vendor fdrv's PRODUCT_ID_AIC8800D81 set (aicwf_usb_chipmatch()): the
 * identities an AIC8800D80 takes once its full-MAC firmware runs. Other
 * AICSemi product ids are different chips (8800DC/DW/D80X2/...) that speak
 * another dialect, so they are left alone. */
static bool aic_is_fdrv_identity(uint16_t vid, uint16_t pid)
{
    if (vid == AIC_USB_VENDOR_ID_AIC || vid == AIC_USB_VENDOR_ID_AIC_V2) {
        switch (pid) {
        case 0x8D81u: /* AIC8800D81 */
        case 0x8D41u: /* AIC8800D41 */
        case 0x8D83u: case 0x8D84u: case 0x8D85u: case 0x8D86u: /* M80 CUSx */
        case 0x8D88u: /* UGREEN */
        case 0x8D89u: case 0x8D8Au: case 0x8D8Bu: case 0x8D8Cu: case 0x8D8Du:
            return true;
        default:
            return false;
        }
    }
    if (vid == AIC_USB_VENDOR_ID_TP) {
        return pid == 0x014Eu /* TP-Link */ || pid == 0x014Bu /* Mercury */;
    }
    if (vid == AIC_USB_VENDOR_ID_TENDA) {
        return pid == 0x001Fu /* U11 */ || pid == 0x0020u /* U11 Pro */;
    }
    return false;
}

static bool aic_is_loader_identity(uint16_t vid, uint16_t pid)
{
    return vid == AIC_USB_VENDOR_ID_AIC &&
           (pid == AIC_USB_PID_LOADER || pid == AIC_USB_PID_LOADER_D40);
}

/* A stable-but-not-vendor MAC would be ideal here (one that survives
 * reboots), and the chip's efuse MAC would be it -- but the efuse is only
 * readable once the firmware runs, and the kernel caches the MAC it reads
 * from get_mac() at boot, before the firmware has been downloaded. So the
 * driver picks one locally-administered address per boot and uses it
 * consistently everywhere (get_mac(), MM_ADD_IF, the supplicant's PTK
 * derivation), which is what keeps the host stack and the firmware
 * agreeing on who this station is. It survives the loader -> firmware
 * re-probe (the kernel cached it before that happened). */
static uint8_t g_boot_mac[6];
static bool g_boot_mac_valid;

static void aic_make_local_mac(uint8_t mac[6])
{
    if (!g_boot_mac_valid) {
        uint64_t seed = aic_now_ticks();
        if (g_aic_api != NULL && g_aic_api->timer.monotonic_ns != NULL) {
            seed ^= g_aic_api->timer.monotonic_ns() << 7;
        }
        seed ^= ((uint64_t)g_aic.addr << 32) | (uint64_t)g_aic.interface;
        seed |= 1u;

        for (uint32_t i = 0u; i < 6u; ++i) {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            g_boot_mac[i] = (uint8_t)(seed >> 24);
        }
        /* locally administered, unicast */
        g_boot_mac[0] = (uint8_t)((g_boot_mac[0] & 0xFCu) | 0x02u);
        g_boot_mac_valid = true;
    }
    aic_memcpy(mac, g_boot_mac, 6u);
}

static bool aic_probe(const bus_device_t *dev)
{
    const usb_device_context_t *ctx;
    aic_stage_t stage;
    bool locked;

    if (g_aic_api == NULL || dev == NULL || dev->bus_context == NULL) {
        return false;
    }

    if (aic_is_loader_identity(dev->vendor_id, dev->device_id)) {
        stage = AIC_STAGE_LOADER;
    } else if (aic_is_fdrv_identity(dev->vendor_id, dev->device_id)) {
        stage = AIC_STAGE_FDRV;
    } else {
        return false; /* another AICSemi chip, or another TP-Link/Tenda device */
    }

    ctx = (const usb_device_context_t *)dev->bus_context;
    if (ctx->ep_in == 0u || ctx->ep_out == 0u ||
        ctx->submit_bulk_timeout == NULL) {
        /* AIC firmware probing is not allowed to use the controller's
         * unbounded/default bulk wait; rejecting an older bus ABI is safer
         * than turning a dongle attach into a multi-second boot stall. */
        return false;
    }

    /* Both vendor ladders bind only to the WLAN interface, which is
     * class/subclass/protocol ff/ff/ff; the Bluetooth side is e0/01/01
     * (and a vendor-specific BT interface is not ff/ff/ff either). */
    if (dev->subclass != AIC_USB_IFACE_CLASS_VENDOR ||
        dev->protocol != AIC_USB_IFACE_CLASS_VENDOR) {
        return false;
    }

    /* The firmware stage has two bulk pairs: the *first* carries 802.11 data
     * and the *second* is the message pipe (vendor fdrv: bulk_in/out are the
     * first IN/OUT, msg_in/out the second). The ROM loader speaks TASK_DBG
     * on the first pair only. A single-pair interface collapses both roles
     * onto the same endpoints; rx_data then stays absent and the CFG pump's
     * parser handles data frames too. */
    uint8_t  first_in = (ctx->ep_in_data != 0u) ? ctx->ep_in_data : ctx->ep_in;
    uint16_t first_in_mps = (ctx->ep_in_data != 0u) ? ctx->ep_in_data_mps
                                                    : ctx->ep_in_mps;
    uint8_t  first_out = (ctx->ep_out_data != 0u) ? ctx->ep_out_data
                                                  : ctx->ep_out;
    uint16_t first_out_mps = (ctx->ep_out_data != 0u) ? ctx->ep_out_data_mps
                                                      : ctx->ep_out_mps;
    uint8_t  cfg_in;
    uint16_t cfg_in_mps;
    uint8_t  cfg_out;
    uint16_t cfg_out_mps;
    if (stage == AIC_STAGE_LOADER) {
        cfg_in = first_in;
        cfg_in_mps = first_in_mps;
        cfg_out = first_out;
        cfg_out_mps = first_out_mps;
    } else {
        cfg_in = ctx->ep_in;
        cfg_in_mps = ctx->ep_in_mps;
        cfg_out = ctx->ep_out;
        cfg_out_mps = ctx->ep_out_mps;
    }
    if (cfg_in_mps == 0u) {
        cfg_in_mps = 512u;
    }
    if (cfg_out_mps == 0u) {
        cfg_out_mps = 512u;
    }
    if (first_in_mps == 0u) {
        first_in_mps = 512u;
    }
    if (first_out_mps == 0u) {
        first_out_mps = 512u;
    }

    /* A probe whose stage differs from the attached one is the same unit
     * re-enumerating (loader -> firmware after START_APP, firmware -> loader
     * after a reboot), even if the bus handed out the same address again. */
    if (g_aic.attached && ctx->addr == g_aic.addr && stage == g_aic.stage) {
        /* A second vendor interface at the same address is normally the
         * Bluetooth side or an enumeration replay. One exception is worth
         * taking: a firmware-stage probe that has the second bulk pair
         * while the attached one did not is the real Wi-Fi interface. */
        bool probe_has_data_pair =
            (stage == AIC_STAGE_FDRV) && (first_in != cfg_in);
        if (!(probe_has_data_pair && !g_aic.rx_data.present) || g_aic.ready) {
            aic_log("[AIC8800] another vendor interface at addr ");
            aic_log_hex32((uint32_t)ctx->addr);
            aic_log(" (iface ");
            aic_log_hex32((uint32_t)ctx->interface);
            aic_log(") ignored -- already driving this device\n");
            return false;
        }
        aic_log("[AIC8800] switching to the interface with a data bulk pair\n");
    }

    /* probe() runs on the hotplug path, possibly while another CPU is in a
     * Wi-Fi syscall: wait for that to finish before replacing the state.
     * The holder is bounded by the command timeout. */
    locked = false;
    for (uint32_t spins = 0u; spins < 5000u; ++spins) {
        if (aic_lock_try()) {
            locked = true;
            break;
        }
        aic_msleep(1u);
    }
    if (!locked) {
        aic_log("[AIC8800] probe: driver busy, deferring\n");
        return false;
    }

    /* Release anything the previous instance owned before overwriting the
     * state (the loader identity's DMA buffer, the patch table, an open
     * firmware file). */
    aic_rx_free_dma();
    aic_fw_reset();
    aic_wpa_stop();

    driver_nic_rx_callback_t keep_cb = g_aic.rx_callback;
    aic_memset(&g_aic, 0u, sizeof(g_aic));
    g_aic.rx_callback = keep_cb; /* set once by the kernel at ethernet_init() */
    g_aic.attached = true;
    g_aic.pid = dev->device_id;
    g_aic.addr = ctx->addr;
    g_aic.interface = ctx->interface;
    g_aic.ep_out = cfg_out;
    g_aic.ep_out_mps = cfg_out_mps;
    g_aic.ep_out_data = first_out;
    g_aic.ep_out_data_mps = first_out_mps;
    g_aic.rx_cfg.ep = cfg_in;
    g_aic.rx_cfg.mps = cfg_in_mps;
    g_aic.rx_cfg.present = (cfg_in != 0u);
    g_aic.rx_cfg.sync_buf = g_rx_buf;
    g_aic.rx_cfg.carry = g_rx_carry;
    g_aic.rx_data.ep = first_in;
    g_aic.rx_data.mps = first_in_mps;
    g_aic.rx_data.present = (stage == AIC_STAGE_FDRV) &&
                            (first_in != 0u) && (first_in != cfg_in);
    g_aic.rx_data.sync_buf = g_rx_data_buf;
    g_aic.rx_data.carry = g_rx_data_carry;
    g_aic.submit_bulk = ctx->submit_bulk;
    g_aic.submit_bulk_timeout = ctx->submit_bulk_timeout;
    g_aic.submit_in_async = ctx->submit_interrupt_in_async;
    g_aic.check_in_event = ctx->check_interrupt_event;
    g_aic.stage = stage;
    g_aic.conn = AIC_CONN_NONE;
    aic_make_local_mac(g_aic.mac);
    aic_memset(g_rx_buf, 0u, sizeof(g_rx_buf));
    aic_memset(g_rx_data_buf, 0u, sizeof(g_rx_data_buf));

    aic_fw_reset();
    aic_bring_reset();

    aic_log("[AIC8800] ");
    aic_log(stage == AIC_STAGE_LOADER ? "loader-stage device at usb addr "
                                      : "firmware-stage device at usb addr ");
    aic_log_hex32((uint32_t)g_aic.addr);
    aic_log(" iface ");
    aic_log_hex32((uint32_t)g_aic.interface);
    aic_log(" id ");
    aic_log_hex32(((uint32_t)dev->vendor_id << 16) | (uint32_t)g_aic.pid);
    aic_log(" cfg-in 0x");
    aic_log_hex32((uint32_t)cfg_in);
    aic_log(" cfg-out 0x");
    aic_log_hex32((uint32_t)cfg_out);
    aic_log(" data-in 0x");
    aic_log_hex32((uint32_t)(g_aic.rx_data.present ? first_in : 0u));
    aic_log(" data-out 0x");
    aic_log_hex32((uint32_t)first_out);
    aic_log("\n");

    aic_unlock();
    return true;
}

static void aic_remove(const bus_device_t *dev)
{
    (void)dev;
    while (!aic_lock_try()) {
        aic_msleep(1u);
    }
    aic_rx_free_dma();
    aic_fw_reset();
    aic_wpa_stop();
    if (g_aic_api != NULL) {
        driver_nic_rx_callback_t keep_cb = g_aic.rx_callback;
        aic_memset(&g_aic, 0u, sizeof(g_aic));
        g_aic.rx_callback = keep_cb;
    }
    aic_unlock();
}

/* ---- state machine ---- */

/* The steady state once the station interface exists: keep the RX pipes
 * drained (scan results, connect/disconnect events, data, EAPOL) and move
 * the connection along. Everything here is non-blocking except the rare
 * key/port commands the supplicant and the connection logic issue. */
static void aic_steady_tick(void)
{
    aic_rx_enable_async();
    aic_pump(AIC_RX_PER_TICK);
    aic_conn_tick();
    aic_wpa_tick();
}

/* One step of whatever the driver is currently doing. The caller holds the
 * entry lock. */
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
            aic_steady_tick();
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
     * is not up yet at that point -- the Wi-Fi entry points take it from
     * there. */
    return g_aic.attached;
}

static bool aic_nic_is_ready(void)
{
    /* Called from the timer tick (network_main.c's DHCP/link check), so it
     * must not touch the USB endpoint. The link counts as up only once
     * traffic can actually flow: keys installed and the controlled port
     * opened for WPA2, association for an open network. DHCP must not
     * start before that. */
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
    if (!g_aic.attached) {
        aic_make_local_mac(g_aic.mac);
    }
    aic_memcpy(mac_out, g_aic.mac, 6u);
}

static bool aic_nic_send_frame(const uint8_t *frame, uint16_t frame_len)
{
    bool ok;

    if (frame == NULL || frame_len == 0u) {
        return false;
    }
    /* Data only flows on an established link; refusing early also keeps
     * the IP stack's ARP/DHCP retries from contending for the lock while
     * the bring-up runs a command. */
    if (!g_aic.ready || g_aic.conn != AIC_CONN_ASSOCIATED) {
        return false;
    }
    if (!aic_lock_try()) {
        return false; /* another CPU is in the driver; the stack retries */
    }
    ok = aic_tx_eth(frame, frame_len);
    aic_unlock();
    return ok;
}

static void aic_nic_poll(void)
{
    /* The generic network poll (scheduled every timer tick, run on the
     * syscall path with interrupts enabled -- the same place the USB hotplug
     * poll runs) drives the whole driver:
     *
     *   - before the station is up: one step of the firmware download or of
     *     the bring-up per poll. Each step is bounded (at most
     *     AIC_FW_CHUNKS_PER_STEP block writes, or one command), so no single
     *     syscall is held for long. This used to be left to the Wi-Fi
     *     panel's status poll alone, which meant an adapter nobody opened
     *     the panel for never got past probe() -- on real hardware the log
     *     simply stopped after the "loader-stage device" line.
     *   - afterwards: the steady state (RX pipes, connection, supplicant).
     */
    if (!g_aic.attached) {
        return;
    }
    if (!g_aic.ready && aic_ms_since(g_aic.last_bring_poll, AIC_BRING_POLL_MS) <
                            AIC_BRING_POLL_MS) {
        return; /* pace the bring-up: it does not need every timer tick */
    }
    if (!aic_lock_try()) {
        return;
    }
    if (!g_aic.ready) {
        g_aic.last_bring_poll = aic_now_ticks();
        aic_tick();
    } else {
        aic_steady_tick();
    }
    aic_unlock();
}

static void aic_nic_set_rx_callback(driver_nic_rx_callback_t cb)
{
    g_aic.rx_callback = cb;
}

/* User actions (scan, connect, disconnect) wait for whoever is in the
 * driver -- normally the network poll on another CPU, which can hold the
 * lock for a bounded USB read -- instead of failing on the first try: a
 * Scan button that does nothing because a poll happened to be running is
 * indistinguishable from a broken adapter. */
static bool aic_lock_wait(uint32_t max_ms)
{
    for (uint32_t waited = 0u; !aic_lock_try(); ++waited) {
        if (waited >= max_ms) {
            return false;
        }
        aic_msleep(1u);
    }
    return true;
}

static bool aic_wifi_scan_start(void)
{
    bool ok = false;
    if (!aic_lock_wait(AIC_USER_LOCK_WAIT_MS)) {
        return false;
    }
    aic_tick();
    ok = aic_scan_start();
    aic_unlock();
    return ok;
}

static uint32_t aic_wifi_get_scan_results(driver_wifi_scan_result_t *out,
                                          uint32_t max_count)
{
    uint32_t n;
    /* Polled by the panel every few hundred ms: a short wait is enough, and
     * returning 0 would blank the list for one refresh. */
    if (!aic_lock_wait(AIC_RESULTS_LOCK_WAIT_MS)) {
        return 0u;
    }
    aic_tick();
    n = aic_scan_results_copy(out, max_count);
    aic_unlock();
    return n;
}

static bool aic_wifi_connect(const char *ssid, const char *psk)
{
    bool ok;
    if (!aic_lock_wait(AIC_USER_LOCK_WAIT_MS)) {
        return false;
    }
    aic_tick();
    ok = aic_connect(ssid, psk);
    aic_unlock();
    return ok;
}

static void aic_wifi_disconnect(void)
{
    if (!aic_lock_wait(AIC_USER_LOCK_WAIT_MS)) {
        return;
    }
    aic_tick();
    aic_disconnect();
    aic_unlock();
}

static void aic_wifi_get_status(driver_wifi_status_t *out_status)
{
    if (out_status == NULL) {
        return;
    }
    if (aic_lock_try()) {
        aic_tick();
        aic_unlock();
    }

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
            if (g_aic.bring_phase == AIC_BRING_FAILED) {
                out_status->state = (uint32_t)DRIVER_WIFI_STATE_FIRMWARE_FAILED;
            } else if (g_aic.bring_phase == AIC_BRING_REBOOT_WAIT) {
                out_status->state = (uint32_t)DRIVER_WIFI_STATE_FIRMWARE_LOADING;
            } else {
                out_status->state = (uint32_t)DRIVER_WIFI_STATE_ADAPTER_ATTACHED;
            }
        } else if (g_aic.conn == AIC_CONN_ASSOCIATED) {
            out_status->state = (uint32_t)DRIVER_WIFI_STATE_ASSOCIATED;
        } else if (g_aic.conn == AIC_CONN_FAILED) {
            out_status->state = (uint32_t)DRIVER_WIFI_STATE_CONNECT_FAILED;
        } else if (g_aic.conn != AIC_CONN_NONE) {
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

    /* A firmware failure has two very different causes -- the loader ladder
     * (blob download) and the firmware-stage bring-up -- and the panel's
     * single "firmware failed" line cannot tell them apart. Carry the step
     * that failed so the user sees it without a serial console. */
    if (out_status->state == (uint32_t)DRIVER_WIFI_STATE_FIRMWARE_FAILED) {
        const char *why = (g_aic.stage == AIC_STAGE_LOADER)
                              ? g_aic.fw_last_error
                              : g_aic.bring_last_error;
        if (why != NULL) {
            uint32_t n = aic_strnlen(why, DRIVER_WIFI_DETAIL_MAX);
            aic_memcpy(out_status->detail, why, n);
            out_status->detail[n] = '\0';
        }
    }
}

/* ---- module wiring ---- */

/* A vendor-specific (class 0xFF) interface on each vendor id an AIC8800D80
 * can enumerate under. The product id is deliberately NOT part of the match
 * (match_flags cannot express a list): probe() classifies it, and rejects
 * everything that is not an AIC8800D80 identity. */
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
    {
        .bus_type = DEVICE_TYPE_USB,
        .vendor_id = (uint16_t)AIC_USB_VENDOR_ID_TP,
        .class_code = AIC_USB_IFACE_CLASS_VENDOR,
        .match_flags = DRIVER_BUS_MATCH_VENDOR | DRIVER_BUS_MATCH_CLASS,
    },
    {
        .bus_type = DEVICE_TYPE_USB,
        .vendor_id = (uint16_t)AIC_USB_VENDOR_ID_TENDA,
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
    while (!aic_lock_try()) {
        aic_msleep(1u);
    }
    aic_rx_free_dma();
    aic_fw_reset();
    aic_wpa_stop();
    if (g_aic_api != NULL) {
        aic_memset(&g_aic, 0u, sizeof(g_aic));
    }
    g_aic_api = NULL;
    aic_unlock();
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
        api->malloc == NULL || api->free == NULL || api->dma_alloc == NULL ||
        api->fs.find_file == NULL || api->fs.read_at == NULL ||
        api->fs.get_file_size == NULL || api->fs.close_file == NULL) {
        return NULL;
    }
    g_aic_api = api;
    return &g_aic_module;
}
