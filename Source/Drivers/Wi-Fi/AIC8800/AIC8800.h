#pragma once

#include "Drivers/Module/DriverBinary.h"
#include "kernel/interfaces/vfs_file.h"

#include "AIC8800_Protocol.h"
#include "AIC8800_Wpa.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * AIC8800.h -- shared state for the AIC8800D80 driver module.
 *
 * The driver is split across five translation units (all built into the
 * single AIC8800_Driver.ELF by Kernel/Source/Drivers/module.mk):
 *
 *   AIC8800.c        USB transport, command/response engine, RX
 *                    dispatch, probe/remove, the poll() state machine,
 *                    and the driver_nic_t vtable handed to the kernel.
 *   AIC8800_Fw.c     the *loader* stage: DBG_MEM_* bring-up, parsing and
 *                    downloading the vendor firmware blobs.
 *   AIC8800_Bring.c  the *fdrv* stage: MM_/SCANU_/SM_ station bring-up,
 *                    scanning, connect/disconnect and key installation.
 *   AIC8800_Data.c   the 802.11 <-> 802.3 data path (TX hostdesc
 *                    framing, RX MPDU decapsulation).
 *   AIC8800_Wpa.c    the in-driver WPA2-PSK supplicant (4-way
 *                    handshake) -- the firmware is full-MAC, so the host
 *                    is responsible for the handshake.
 *   AIC8800_Crypto.c SHA-1/HMAC/PBKDF2 and AES-128 (for the NIST key
 *                    unwrap of the GTK in message 3 of 4).
 *
 * All state lives in one process-wide struct (a driver module is loaded
 * once) and every entry point runs on the same thread -- the kernel's
 * syscall/network-poll path -- so no locking beyond an interrupt-safe
 * guard around the USB endpoint access is needed.
 */

/* usb_submit_bulk()'s pid argument: 1 == IN, 0 == OUT (see xhci_submit_bulk
 * in Kernel/Drivers/Bus/USB/XHCI/XHCI.c). */
#define AIC_USB_PID_IN              1u
#define AIC_USB_PID_OUT             0u

/* ---- limits ---- */

/* Firmware blobs live on the OS filesystem, not in the repository (they
 * are proprietary): drop the vendor's own .bin files from the driver
 * tarball into this directory. See Firmware/AIC8800/README.md. */
#define AIC_FW_DIR              "/Kernel/Driver/Firmware/AIC8800/"

#define AIC_FW_FILE_ADID        "fw_adid_8800d80_u02.bin"
#define AIC_FW_FILE_PATCH       "fw_patch_8800d80_u02.bin"
#define AIC_FW_FILE_PATCH_EXT   "fw_patch_8800d80_u02_ext"
#define AIC_FW_FILE_FMAC        "fmacfw_8800d80_u02.bin"
#define AIC_FW_FILE_FMAC_H      "fmacfw_8800d80_h_u02.bin"
#define AIC_FW_FILE_PATCH_TABLE "fw_patch_table_8800d80_u02.bin"

/* The patch table file is tiny (a couple of KiB); the cap only exists
 * so a corrupt/mismatched file cannot make the driver allocate an
 * arbitrary amount out of the kernel heap. */
#define AIC_FW_PATCH_TABLE_MAX  16384u

/* Sanity cap on one firmware image. The D80's fmacfw image is ~300 KiB;
 * anything past this is not the file we are looking for. */
#define AIC_FW_FILE_MAX         0x00100000u /* 1 MiB */

/* Firmware download pace: at most this many 1 KiB DBG_MEM_BLOCK_WRITE
 * chunks per poll() call, so a ~300 KiB image never blocks the syscall
 * path for more than a few milliseconds at a time. */
#define AIC_FW_CHUNKS_PER_STEP  16u

/* At most this many (addr,value) patch-table writes per poll() call. */
#define AIC_FW_WRITES_PER_STEP  64u

/* Bounds on what one fw_patch_table_*.bin's INF block may list (the D80
 * file lists one external patch image). */
#define AIC_FW_EXT_PATCH_MAX    4u

/* Command/response engine */
#define AIC_CMD_BUF_MAX         2048u
#define AIC_RX_BUF_MAX          2048u
#define AIC_USB_XFER_TIMEOUT_MS 100u
#define AIC_CMD_TIMEOUT_MS      300u

/* How many completed bulk transfers aic_pump()/aic_tick() will process in
 * one visit. Each one re-arms immediately, so this only bounds how long a
 * single syscall can spend draining a burst. */
#define AIC_RX_PER_TICK         16u

/* The station interface's MTU: 802.3 payload, i.e. the 1500 bytes the
 * firmware's own MAX_AMSDU/MPDU budget is sized around. The kernel's IP
 * stack fragments above this. */
#define AIC_MAX_MTU             1500u

/* Scanning: how long a scan stays "open" collecting SCANU_RESULT_IND
 * messages. Matches the window the window manager's Wi-Fi panel uses
 * (WIFI_SCAN_WINDOW_MS). */
#define AIC_SCAN_WINDOW_MS      3000u
#define AIC_MAX_SCAN_RESULTS    24u

/* Connection: how long to wait for SM_CONNECT_IND after SM_CONNECT_REQ
 * was accepted, and the EAPOL-Start retry period once associated. */
#define AIC_CONNECT_TIMEOUT_MS  8000u
#define AIC_EAPOL_START_MS      300u

/* Bring-up retry pacing (the firmware blobs may simply not be installed
 * yet, and this must not cost anything on such boots). */
#define AIC_RETRY_INTERVAL_MS   2000u
#define AIC_MAX_ATTEMPTS        6u

/* ---- state ---- */

typedef enum {
    AIC_STAGE_NONE = 0,
    AIC_STAGE_LOADER, /* PID 8d80: ROM, needs firmware */
    AIC_STAGE_FDRV,   /* PID 8d83/8d81: firmware running */
} aic_stage_t;

typedef enum {
    AIC_FW_IDLE = 0,
    AIC_FW_SYS_CONFIG,      /* DBG_MEM_READ chip id + cache config */
    AIC_FW_PATCH_TABLE,     /* read + parse fw_patch_table_*.bin */
    AIC_FW_LOAD_ADID,       /* fw_adid_*.bin -> patch_info.addr_adid */
    AIC_FW_LOAD_PATCH,      /* fw_patch_*.bin -> patch_info.addr_patch */
    AIC_FW_LOAD_EXT,        /* fw_patch_*_ext<N>.bin -> ext addr */
    AIC_FW_PATCH_TABLE_LOAD,/* write every patch table block */
    AIC_FW_LOAD_FMAC,       /* fmacfw_*.bin -> RAM_FMAC_FW_ADDR */
    AIC_FW_PATCH_CONFIG,    /* aicwf_patch_config_8800d80() */
    AIC_FW_START_APP,       /* DBG_START_APP_REQ */
    AIC_FW_DONE,
    AIC_FW_FAILED,
} aic_fw_phase_t;

typedef enum {
    AIC_BRING_IDLE = 0,
    AIC_BRING_RF,        /* MM_SET_TXPWR_IDX_LVL, MM_SET_RF_CALIB */
    AIC_BRING_MAC_ADDR,  /* MM_GET_MAC_ADDR (logged only) */
    AIC_BRING_RESET,     /* MM_RESET */
    AIC_BRING_VERSION,   /* MM_VERSION */
    AIC_BRING_START,     /* MM_START */
    AIC_BRING_ADD_IF,    /* MM_ADD_IF -> vif index, station ready */
    AIC_BRING_DONE,
    AIC_BRING_FAILED,
} aic_bring_phase_t;

typedef enum {
    AIC_CONN_NONE = 0,
    AIC_CONN_CONNECTING, /* SM_CONNECT_REQ sent, waiting for SM_CONNECT_IND */
    AIC_CONN_4WAY,       /* SM_CONNECT_IND ok, WPA2 4-way handshake running */
    AIC_CONN_ASSOCIATED, /* keys installed, data path live */
    AIC_CONN_FAILED,
} aic_conn_state_t;

typedef struct {
    /* ---- USB transport (set by probe(), cleared by remove()) ---- */
    bool     attached;
    uint16_t pid;
    uint8_t  addr;
    uint8_t  interface;
    uint8_t  ep_in;
    uint8_t  ep_out;
    uint16_t ep_in_mps;
    uint16_t ep_out_mps;
    uint8_t  ep_out_data;
    uint16_t ep_out_data_mps;
    bool (*submit_bulk)(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                        uint8_t pid, void *data, uint32_t length);
    /* Bounded variant (see usb_device_context_t in DriverBinary.h). Every
     * transfer this driver makes uses it when available: the unbounded
     * call gives xHCI 30 s per transfer and the chip answers nothing at
     * all until its firmware is running. */
    bool (*submit_bulk_timeout)(uint8_t addr, uint8_t endpoint,
                                uint16_t max_packet_size, uint8_t pid,
                                void *data, uint32_t length,
                                uint32_t timeout_ms);
    /* Non-blocking IN pair, also copied out of usb_device_context_t (see
     * there for the contract): the RX pump posts one transfer into a DMA
     * buffer and only tests for its completion on each poll. */
    bool (*submit_in_async)(uint8_t addr, uint8_t ep_num,
                            uint16_t max_packet_size, void *dma_buf,
                            uint64_t dma_phys, uint16_t length);
    int  (*check_in_event)(uint8_t addr, uint8_t ep_num);

    aic_stage_t stage;

    /* Command/response engine: aic_cmd() parks the expected confirmation
     * id here and lets the RX dispatch fill in the reply. */
    bool     cmd_busy;
    uint16_t wait_cfm_id;
    uint16_t cfm_len;
    uint16_t cfm_cap;
    void    *cfm_buf;
    bool     cfm_received;

    /* Re-entrancy guard: aic_cmd() drains the IN endpoint while waiting,
     * and the RX dispatch must therefore never recursively drain (or,
     * worse, run a second command engine instance). Handlers called from
     * the dispatch defer any command they need to a later poll(). */
    bool     rx_busy;

    /* ---- RX pump ----
     *
     * Preferred mode is an asynchronously posted transfer into a DMA
     * buffer (driver_binary_t.usb.submit_interrupt_in_async on the bulk IN
     * endpoint): an idle link then costs nothing per poll, because each
     * poll only tests a completion flag instead of busy-waiting through a
     * bulk transfer timeout. The sync bulk read stays as the fallback, and
     * is what creates the endpoint's transfer ring in the first place. */
    bool     rx_pending;    /* an async transfer is posted right now */
    bool     rx_async;      /* DMA buffer + async API available */
    bool     rx_ring_ready; /* one sync transfer has already run (ring exists) */
    uint8_t *rx_dma;
    uint64_t rx_dma_phys;

    /* ---- loader stage: firmware download ---- */
    aic_fw_phase_t fw_phase;
    uint32_t fw_attempts;
    uint64_t fw_last_attempt; /* ticks at the last attempt (for retry pacing) */
    bool     fw_done;
    uint8_t  chip_id;
    uint8_t  chip_mcu_id;
    uint32_t fw_version;
    uint32_t addr_adid;
    uint32_t addr_patch;
    /* Patch table contents (the vendor files list one "ext" patch image per
     * entry; the D80 files have exactly one). */
    uint32_t ext_patch_nb;
    uint32_t ext_patch_id[AIC_FW_EXT_PATCH_MAX];
    uint32_t ext_patch_addr[AIC_FW_EXT_PATCH_MAX];
    uint32_t ext_patch_index;
    /* INF block fields kept for the log; the vendor's D80 download path
     * parses but never writes reset_addr/reset_val/adid_flag. */
    uint32_t reset_addr;
    uint32_t reset_val;
    uint32_t adid_flag_addr;
    uint32_t adid_flag;
    uint8_t *patch_table;     /* malloc'd raw file contents */
    uint32_t patch_table_len;
    /* The patch table is written straight out of that buffer, so the walk
     * only needs its position: the byte offset of the block being written
     * and the index of the next (addr,value) pair inside it. */
    uint32_t pt_off;
    uint32_t pt_pair;

    /* Current file being uploaded, chunk by chunk. */
    vfs_file_t file;
    bool     file_open;
    uint32_t file_off;
    uint32_t file_size;
    uint32_t file_addr;

    /* ---- fdrv stage: station bring-up ---- */
    aic_bring_phase_t bring_phase;
    uint32_t bring_attempts;
    uint64_t bring_last_attempt; /* ticks at the last attempt (for retry pacing) */
    uint8_t  vif_idx;
    uint8_t  mac[AIC_MAC_ADDR_LEN];
    bool     ready; /* MM_ADD_IF_CFM came back */

    /* ---- scan ---- */
    bool     scanning;
    uint64_t scan_deadline;
    driver_wifi_scan_result_t scan_results[AIC_MAX_SCAN_RESULTS];
    uint16_t scan_freq[AIC_MAX_SCAN_RESULTS]; /* parallel: MHz of each result */
    uint32_t scan_result_count;

    /* ---- connection ---- */
    aic_conn_state_t conn;
    char     ssid[DRIVER_WIFI_SSID_MAX + 1u];
    char     psk[64];      /* WPA2 passphrase (8..63 chars + NUL) */
    uint8_t  bssid[AIC_MAC_ADDR_LEN];
    uint8_t  ap_idx;       /* firmware station index from SM_CONNECT_IND */
    uint16_t center_freq;
    bool     wpa_enabled;  /* a PSK was supplied: run the 4-way handshake */
    bool     connect_open; /* target is an open network */
    uint64_t connect_deadline;

    /* ---- data path ---- */
    driver_nic_rx_callback_t rx_callback;
} aic_state_t;

extern aic_state_t g_aic;

/* The driver_binary_t handed to driver_module_init(); NULL until then. */
extern const driver_binary_t *g_aic_api;

/* ---- AIC8800.c: transport + engine ---- */

/* Serial logging, all no-ops when the kernel has no serial console. */
void aic_log(const char *msg);
void aic_log_hex32(uint32_t value);

/* Sleep for at most `ms` (no-op when the kernel exposes no msleep). */
void aic_msleep(uint32_t ms);

/* Little-endian accessors -- the firmware's wire format is LE throughout,
 * and every structure in AIC8800_Protocol.h assumes LE field order. */
static inline uint16_t aic_rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static inline uint32_t aic_rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void aic_wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline void aic_wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline void aic_memcpy(void *dst, const void *src, uint32_t n)
{
    if (g_aic_api != NULL && g_aic_api->memcpy != NULL) {
        (void)g_aic_api->memcpy(dst, src, (size_t)n);
        return;
    }
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    for (uint32_t i = 0u; i < n; ++i) {
        d[i] = s[i];
    }
}

static inline void aic_memset(void *dst, uint8_t value, uint32_t n)
{
    if (g_aic_api != NULL && g_aic_api->memset != NULL) {
        (void)g_aic_api->memset(dst, value, (size_t)n);
        return;
    }
    uint8_t *d = (uint8_t *)dst;
    for (uint32_t i = 0u; i < n; ++i) {
        d[i] = value;
    }
}

static inline bool aic_memequal(const void *a, const void *b, uint32_t n)
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

static inline uint32_t aic_strnlen(const char *s, uint32_t max)
{
    uint32_t n = 0u;
    while (n < max && s[n] != '\0') {
        ++n;
    }
    return n;
}

/* One bounded bulk transfer over the device's bulk pipe. */
bool aic_usb_bulk(uint8_t endpoint, uint16_t mps, uint8_t pid,
                  void *data, uint32_t length);

/* Send an lmac_msg without waiting for a response (START_APP and
 * friends). dest_id is a TASK_* value; src_id is always DRV_TASK_ID. */
bool aic_cmd_nowait(uint16_t msg_id, uint16_t dest_id,
                    const void *param, uint16_t param_len);

/* Send an lmac_msg and wait (draining the IN endpoint) for the *_CFM.
 * On success `cfm_out` holds the first min(cfm_cap, param_len) bytes of
 * the response parameter block. A zero `expect_cfm_id` means "do not
 * wait" (equivalent to aic_cmd_nowait). */
bool aic_cmd(uint16_t msg_id, uint16_t expect_cfm_id, uint16_t dest_id,
             const void *param, uint16_t param_len,
             void *cfm_out, uint16_t cfm_cap, uint32_t timeout_ms);

/* Millisecond clock helpers (0 on failure to read the timer). */
uint32_t aic_ms_since(uint64_t since_ticks, uint32_t fallback);
uint64_t aic_now_ticks(void);

/* Drain and dispatch up to `max_transfers` completed bulk transfers.
 * Safe to call from any entry point; a nested call is a no-op. */
void aic_pump(uint32_t max_transfers);

/* Run the whole state machine once (firmware download -> bring-up ->
 * scan/handshake bookkeeping) plus one bounded RX pump. This is what every
 * entry point the kernel can reach calls first, so the driver makes
 * progress no matter which of them the kernel happens to poll: the
 * kernel's own network poll is gated on network_stack_is_ready(), which is
 * decided once at boot, and the Wi-Fi syscalls are not. */
void aic_tick(void);

/* ---- AIC8800_Fw.c: loader stage ---- */

void aic_fw_reset(void);
/* One paced step of the firmware download: a single DBG round-trip, or a
 * bounded run of 1 KiB block writes. The stage's phase carries the
 * progress across calls. */
void aic_fw_step(void);

/* ---- AIC8800_Bring.c: fdrv stage ---- */

void aic_bring_reset(void);
void aic_bring_step(void);
/* Steady-state bookkeeping: closes the scan window when its deadline passes
 * and moves the connection state machine along (SM_CONNECT_IND seen,
 * 4-way handshake finished or timed out). */
void aic_conn_tick(void);
bool aic_scan_start(void);
uint32_t aic_scan_results_copy(driver_wifi_scan_result_t *out, uint32_t max_count);
bool aic_connect(const char *ssid, const char *psk);
void aic_disconnect(void);
bool aic_mm_key_add(uint8_t key_idx, uint8_t sta_idx, const uint8_t *key,
                    uint8_t key_len, uint8_t cipher_suite, bool pairwise);
void aic_mm_disconnect(void);

/* RX-side handlers (called from the dispatch in AIC8800.c). */
void aic_handle_scan_result(const uint8_t *payload, uint16_t len);
void aic_handle_connect_ind(const uint8_t *payload, uint16_t len);
void aic_handle_disconnect_ind(const uint8_t *payload, uint16_t len);

/* ---- AIC8800_Data.c: 802.11/802.3 data path ---- */

/* Transmit a complete 802.3 frame (14-byte Ethernet header included). */
bool aic_tx_eth(const uint8_t *frame, uint16_t frame_len);
/* Handle one RX USB packet whose payload begins with a 56-byte hw_rxhdr
 * followed by an 802.11 MPDU. */
void aic_rx_data_packet(const uint8_t *usb_packet, uint16_t usb_len);

/* ---- AIC8800_Wpa.c: WPA2-PSK supplicant (see AIC8800_Wpa.h) ---- */

/* The supplicant installs its own PTK/GTK through aic_wpa_tick(); the
 * station bring-up uses this to decide when the link is really usable
 * (the window manager must not report ASSOCIATED -- and the network
 * stack must not start DHCP -- before the keys are in the firmware). */
bool aic_wpa_keys_installed(void);
