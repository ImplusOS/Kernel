#include "WiFi.h"
#include "WiFi_80211.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * WiFi.c -- foundation for an ImplusOS 802.11 driver module.
 *
 * Scope of this build:
 *   - PCI discovery (known-ID table + generic network-controller class scan)
 *     and BAR mapping, following the same pattern as VirtIONet / I219V.
 *   - The full link state machine (scan -> auth -> assoc -> associated ->
 *     disconnect), the scan-result table, and 802.11 <-> Ethernet frame
 *     (de)capsulation (RFC 1042 SNAP) are implemented in software.
 *   - Registers as a DEVICE_TYPE_NIC driver_nic_t, so NicManager / the
 *     network stack pick it up exactly like a wired NIC once associated.
 *
 * NOT in scope yet (see the "TODO(hw):" markers below): the concrete
 * chipset's firmware/command interface, the TX/RX descriptor rings, over-
 * the-air transmission of management frames, and the WPA2/WPA3 handshake.
 * hw_dequeue_rx()/hw_enqueue_tx() are the two integration points where a
 * real chipset backend plugs in; every function above them is already
 * exercised and does not need to change when that backend is added.
 *
 * Driver modules are freestanding and link with no C library at all (see
 * Kernel/Drivers/module.mk), so this file avoids libc calls other than the
 * primitives handed in through driver_binary_t; small helpers below cover
 * what's missing (string length, byte compare).
 */

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"
#endif

#ifdef IMPLUS_DRIVER_MODULE
static const driver_binary_t *g_api = NULL;

#define memset               g_api->memset
#define memcpy               g_api->memcpy
#define map_mmio_range       g_api->hw.map_mmio_range
#define pci_read_config      g_api->pci_read_config
#define pci_write_config     g_api->pci_write_config
#define serial_write_string  g_api->serial_write_string
#define timer_ticks          g_api->timer_ticks
#define timer_hz             g_api->timer_hz

#define hal_cpu_save_interrupts    g_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts g_api->hal.cpu_restore_interrupts
#define hal_cpu_pause              g_api->hal.cpu_pause

typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_lock(spinlock_t *l)
{
    while (__sync_lock_test_and_set(&l->locked, 1)) {
        while (l->locked) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->locked); }

static inline uint64_t irq_save_disable(void) { return hal_cpu_save_interrupts(); }
static inline void irq_restore(uint64_t flags) { hal_cpu_restore_interrupts(flags); }
#endif

/* ---- small libc-free helpers ---- */

static size_t wifi_strnlen_local(const char *s, size_t max)
{
    size_t n = 0;
    while (n < max && s[n] != '\0') { n++; }
    return n;
}

static bool wifi_bssid_eq(const uint8_t a[6], const uint8_t b[6])
{
    for (int i = 0; i < 6; ++i) {
        if (a[i] != b[i]) { return false; }
    }
    return true;
}

/* ---- PCI identification ----
 * 802.11 controllers are not guaranteed a dedicated PCI subclass the way
 * Ethernet NICs are (class 0x02 / subclass 0x00); most report class 0x02
 * ("network controller") / subclass 0x80 ("other"). find_wifi_controller()
 * matches that generic signature, biased by a short list of known vendor/
 * device IDs. Extend g_known_wifi_ids with the IDs of the chipset you are
 * bringing up.
 */
#define PCI_CLASS_NETWORK          0x02u
#define PCI_SUBCLASS_NETWORK_OTHER 0x80u

typedef struct {
    uint16_t    vendor_id;
    uint16_t    device_id;
    const char *name;
} wifi_pci_id_t;

static const wifi_pci_id_t g_known_wifi_ids[] = {
    { 0x8086u, 0x2723u, "Intel AX200" },
    { 0x8086u, 0x2725u, "Intel AX210" },
    { 0x8086u, 0x24FDu, "Intel 7265" },
    { 0x10ECu, 0x8179u, "Realtek RTL8188EE" },
    { 0x14E4u, 0x43A3u, "Broadcom BCM4350" },
};

typedef struct {
    uint8_t  bus;
    uint8_t  device;
    uint8_t  func;
    uint64_t bar_addr[6];
    uint8_t  bar_is_mem[6];
} wifi_pci_dev_t;

static bool wifi_pci_id_is_known(uint16_t vendor_id, uint16_t device_id)
{
    for (size_t i = 0; i < sizeof(g_known_wifi_ids) / sizeof(g_known_wifi_ids[0]); ++i) {
        if (g_known_wifi_ids[i].vendor_id == vendor_id && g_known_wifi_ids[i].device_id == device_id) {
            return true;
        }
    }
    return false;
}

static void wifi_read_bar_addrs(uint8_t bus, uint8_t device, uint8_t func,
                                 uint64_t out_bar[6], uint8_t out_is_mem[6])
{
    for (uint8_t j = 0; j < 6u; j++) {
        out_bar[j] = 0;
        out_is_mem[j] = 0;
    }

    uint8_t i = 0;
    while (i < 6u) {
        uint32_t bar = pci_read_config(bus, device, func, (uint8_t)(0x10u + i * 4u));

        if (bar == 0u || bar == 0xFFFFFFFFu) {
            i++;
            continue;
        }

        if ((bar & 0x1u) != 0u) {
            /* I/O-space BAR -- not supported by this foundation yet. */
            i++;
            continue;
        }

        out_is_mem[i] = 1;
        if (((bar >> 1) & 0x3u) == 0x2u && i < 5u) {
            uint32_t bar_hi = pci_read_config(bus, device, func, (uint8_t)(0x10u + (i + 1u) * 4u));
            out_bar[i] = (((uint64_t)bar_hi) << 32) | (uint64_t)(bar & ~0xFu);
            i = (uint8_t)(i + 2u);
            continue;
        }

        out_bar[i] = (uint64_t)(bar & ~0xFu);
        i++;
    }
}

static bool find_wifi_controller(wifi_pci_dev_t *out_dev)
{
    if (out_dev == NULL) {
        return false;
    }

    for (uint16_t bus = 0; bus < 256u; ++bus) {
        for (uint8_t device = 0; device < 32u; ++device) {
            for (uint8_t func = 0; func < 8u; ++func) {
                uint32_t vd = pci_read_config((uint8_t)bus, device, func, 0x00u);
                uint16_t vendor_id = (uint16_t)(vd & 0xFFFFu);
                uint16_t device_id = (uint16_t)((vd >> 16) & 0xFFFFu);

                if (vendor_id == 0xFFFFu) {
                    if (func == 0u) {
                        break;
                    }
                    continue;
                }

                uint32_t class_reg = pci_read_config((uint8_t)bus, device, func, 0x08u);
                uint8_t class_code = (uint8_t)((class_reg >> 24) & 0xFFu);
                uint8_t subclass = (uint8_t)((class_reg >> 16) & 0xFFu);

                bool match = wifi_pci_id_is_known(vendor_id, device_id) ||
                             (class_code == PCI_CLASS_NETWORK && subclass == PCI_SUBCLASS_NETWORK_OTHER);

                if (match) {
                    out_dev->bus = (uint8_t)bus;
                    out_dev->device = device;
                    out_dev->func = func;

                    uint32_t cmd = pci_read_config((uint8_t)bus, device, func, 0x04u);
                    cmd |= (1u << 1) | (1u << 2); /* memory space + bus master */
                    pci_write_config((uint8_t)bus, device, func, 0x04u, cmd);

                    wifi_read_bar_addrs((uint8_t)bus, device, func, out_dev->bar_addr, out_dev->bar_is_mem);
                    return true;
                }

                if (func == 0u) {
                    uint32_t header_type = pci_read_config((uint8_t)bus, device, func, 0x0Cu);
                    if (((header_type >> 16) & 0x80u) == 0u) {
                        break;
                    }
                }
            }
        }
    }

    return false;
}

/* ---- driver state ---- */

#define WIFI_DEFAULT_MTU            1500u
/* TODO(hw): probe the real BAR size (write all-1s, read back, restore)
 * instead of mapping a fixed placeholder window. */
#define WIFI_MMIO_WINDOW_BYTES      0x4000u
#define WIFI_MAX_80211_FRAME_BYTES  1560u /* qos hdr(26) + snap(8) + 1500 payload, rounded up */
#define WIFI_MAX_ETH_FRAME_BYTES    1514u /* 6+6+2 header + 1500 payload */
#define WIFI_SCAN_TIMEOUT_FALLBACK_TICKS 50u

typedef struct {
    spinlock_t lock;
    uint8_t    hw_ready;

    wifi_pci_dev_t     pci;
    volatile uint8_t  *mmio_base;

    uint8_t  mac[6];
    uint16_t mtu;

    wifi_link_state_t link_state;
    uint8_t            bssid[6];
    char               target_ssid[WIFI_MAX_SSID_LEN + 1u];
    uint8_t            target_ssid_len;
    uint8_t            target_psk[WIFI_MAX_PSK_LEN + 1u];
    uint8_t            target_psk_len;
    int8_t             rssi_dbm;

    bool     scanning;
    uint64_t scan_start_tick;
    uint32_t scan_count;
    wifi_scan_result_t scan_results[WIFI_MAX_SCAN_RESULTS];

    wifi_rx_callback_t rx_callback;
} wifi_state_t;

static wifi_state_t g_wifi = {0};

/* ---- 802.11 <-> Ethernet (RFC 1042 / SNAP) ---- */

static uint16_t wifi_encapsulate_8023_to_80211(const uint8_t *eth_frame, uint16_t eth_len,
                                                const uint8_t bssid[6], const uint8_t sta_mac[6],
                                                uint8_t *out_80211, uint16_t out_cap)
{
    if (eth_frame == NULL || out_80211 == NULL || eth_len < 14u) {
        return 0u;
    }

    uint16_t data_len = (uint16_t)(eth_len - 14u);
    uint16_t total = (uint16_t)(sizeof(ieee80211_qos_hdr_t) + IEEE80211_SNAP_HDR_LEN + data_len);
    if (total > out_cap) {
        return 0u;
    }

    ieee80211_qos_hdr_t *hdr = (ieee80211_qos_hdr_t *)out_80211;
    memset(hdr, 0, sizeof(*hdr));
    hdr->hdr.frame_control = (uint16_t)((IEEE80211_FC_TYPE_DATA << 2) |
                                         (IEEE80211_FC_STYPE_QOS_DATA << 4) |
                                         IEEE80211_FCTL_TODS);
    memcpy(hdr->hdr.addr1, bssid, 6u);        /* RA = AP   */
    memcpy(hdr->hdr.addr2, sta_mac, 6u);      /* TA = us   */
    memcpy(hdr->hdr.addr3, eth_frame, 6u);    /* DA, from the Ethernet header */
    /* TODO(hw): real per-TID sequence numbering (seq_control) once a
     * chipset backend exists; a static value is harmless for a frame that
     * is never actually transmitted yet. */

    uint8_t *snap = out_80211 + sizeof(ieee80211_qos_hdr_t);
    snap[0] = IEEE80211_SNAP_DSAP;
    snap[1] = IEEE80211_SNAP_SSAP;
    snap[2] = IEEE80211_SNAP_CTRL;
    snap[3] = 0x00u;
    snap[4] = 0x00u;
    snap[5] = 0x00u;
    snap[6] = eth_frame[12]; /* EtherType, copied from the original Ethernet header */
    snap[7] = eth_frame[13];

    memcpy(out_80211 + sizeof(ieee80211_qos_hdr_t) + IEEE80211_SNAP_HDR_LEN,
           eth_frame + 14u, data_len);

    return total;
}

static uint16_t wifi_decapsulate_80211_to_8023(const uint8_t *frame, uint16_t len,
                                                uint8_t *out_eth, uint16_t out_cap)
{
    if (frame == NULL || out_eth == NULL || len < sizeof(ieee80211_mac_hdr_t)) {
        return 0u;
    }

    const ieee80211_mac_hdr_t *hdr = (const ieee80211_mac_hdr_t *)frame;
    uint16_t fc = hdr->frame_control;

    uint16_t hdr_len = (uint16_t)sizeof(ieee80211_mac_hdr_t);
    if ((IEEE80211_FCTL_STYPE(fc) & IEEE80211_FC_STYPE_QOS_FLAG) != 0u) {
        hdr_len = (uint16_t)(hdr_len + 2u); /* QoS Control field */
    }
    if (len < (uint16_t)(hdr_len + IEEE80211_SNAP_HDR_LEN)) {
        return 0u;
    }

    const uint8_t *snap = frame + hdr_len;
    if (snap[0] != IEEE80211_SNAP_DSAP || snap[1] != IEEE80211_SNAP_SSAP) {
        return 0u; /* not RFC1042-encapsulated -- TODO(hw): handle other encapsulations */
    }

    uint16_t data_len = (uint16_t)(len - hdr_len - IEEE80211_SNAP_HDR_LEN);
    uint16_t total = (uint16_t)(14u + data_len);
    if (total > out_cap) {
        return 0u;
    }

    memcpy(out_eth, hdr->addr1, 6u);       /* DA */
    memcpy(out_eth + 6u, hdr->addr3, 6u);  /* SA (from-DS: addr3 carries the original sender) */
    out_eth[12] = snap[6];
    out_eth[13] = snap[7];
    memcpy(out_eth + 14u, snap + IEEE80211_SNAP_HDR_LEN, data_len);

    return total;
}

/* ---- scan-result table (locked by the caller) ---- */

static void wifi_scan_note_bss_locked(const uint8_t bssid[6], const char *ssid, uint8_t ssid_len,
                                       uint8_t channel, int8_t rssi_dbm, wifi_security_t security)
{
    for (uint32_t i = 0; i < g_wifi.scan_count; ++i) {
        if (wifi_bssid_eq(g_wifi.scan_results[i].bssid, bssid)) {
            memcpy(g_wifi.scan_results[i].ssid, ssid, (size_t)ssid_len);
            g_wifi.scan_results[i].ssid[ssid_len] = '\0';
            g_wifi.scan_results[i].ssid_len = ssid_len;
            g_wifi.scan_results[i].channel = channel;
            g_wifi.scan_results[i].rssi_dbm = rssi_dbm;
            g_wifi.scan_results[i].security = security;
            return;
        }
    }

    if (g_wifi.scan_count < WIFI_MAX_SCAN_RESULTS) {
        wifi_scan_result_t *slot = &g_wifi.scan_results[g_wifi.scan_count];
        memcpy(slot->bssid, bssid, 6u);
        memcpy(slot->ssid, ssid, (size_t)ssid_len);
        slot->ssid[ssid_len] = '\0';
        slot->ssid_len = ssid_len;
        slot->channel = channel;
        slot->rssi_dbm = rssi_dbm;
        slot->security = security;
        g_wifi.scan_count++;
    }
}

static void wifi_process_beacon_or_probe_resp_locked(const uint8_t *frame, uint16_t len, int8_t rssi_dbm)
{
    uint16_t min_len = (uint16_t)(sizeof(ieee80211_mac_hdr_t) + sizeof(ieee80211_beacon_fixed_t));
    if (len < min_len) {
        return;
    }

    const ieee80211_mac_hdr_t *hdr = (const ieee80211_mac_hdr_t *)frame;
    const ieee80211_beacon_fixed_t *fixed =
        (const ieee80211_beacon_fixed_t *)(frame + sizeof(ieee80211_mac_hdr_t));

    const uint8_t *ie = frame + min_len;
    uint16_t ie_remaining = (uint16_t)(len - min_len);

    char    ssid[WIFI_MAX_SSID_LEN + 1u];
    uint8_t ssid_len = 0u;
    uint8_t channel = 0u;
    bool    has_rsn = false;

    ssid[0] = '\0';

    while (ie_remaining >= 2u) {
        uint8_t id = ie[0];
        uint8_t elen = ie[1];
        uint16_t consumed = (uint16_t)(elen + 2u);
        if (consumed > ie_remaining) {
            break;
        }

        if (id == IEEE80211_EID_SSID && elen <= WIFI_MAX_SSID_LEN) {
            memcpy(ssid, ie + 2, (size_t)elen);
            ssid[elen] = '\0';
            ssid_len = elen;
        } else if (id == IEEE80211_EID_DS_PARAMS && elen >= 1u) {
            channel = ie[2];
        } else if (id == IEEE80211_EID_RSN) {
            has_rsn = true;
        }

        ie += consumed;
        ie_remaining = (uint16_t)(ie_remaining - consumed);
    }

    bool privacy = (fixed->capability_info & IEEE80211_CAPINFO_PRIVACY) != 0u;

    /* TODO(hw): parse the RSN IE's AKM suite list to distinguish WPA2-PSK
     * from WPA3-SAE instead of assuming WPA2-PSK whenever an RSN IE is
     * present. */
    wifi_security_t security = WIFI_SECURITY_OPEN;
    if (has_rsn) {
        security = WIFI_SECURITY_WPA2_PSK;
    } else if (privacy) {
        security = WIFI_SECURITY_WEP;
    }

    wifi_scan_note_bss_locked(hdr->addr3, ssid, ssid_len, channel, rssi_dbm, security);
}

static void wifi_advance_scan_timeout_locked(void)
{
    if (!g_wifi.scanning) {
        return;
    }

    uint64_t hz = (uint64_t)timer_hz();
    uint64_t elapsed = timer_ticks() - g_wifi.scan_start_tick;
    uint64_t timeout_ticks = (hz > 0u) ? (hz / 2u) : (uint64_t)WIFI_SCAN_TIMEOUT_FALLBACK_TICKS;

    if (elapsed >= timeout_ticks) {
        g_wifi.scanning = false;
        if (g_wifi.link_state == WIFI_LINK_SCANNING) {
            g_wifi.link_state = WIFI_LINK_DOWN;
        }
    }
}

/* ---- hardware integration seam (TODO(hw): implement for a real chipset) ----
 * Everything above this point is chipset-agnostic and already exercised.
 * A concrete backend only needs to fill in these two functions: pull a
 * completed 802.11 frame + its RSSI out of the RX ring, and push an
 * encapsulated 802.11 frame onto the TX ring.
 */

static bool hw_dequeue_rx(uint8_t *out_frame, uint16_t cap, uint16_t *out_len, int8_t *out_rssi)
{
    (void)out_frame;
    (void)cap;
    (void)out_len;
    (void)out_rssi;
    /* No chipset backend is wired in yet, so there is nothing to dequeue. */
    return false;
}

static bool hw_enqueue_tx(const uint8_t *frame80211, uint16_t len)
{
    (void)frame80211;
    (void)len;
    /* No chipset backend is wired in yet; drop and let the upper layer
     * retransmit, exactly as a full TX ring would signal back-pressure. */
    return false;
}

/* ---- RX dispatch (locked by the caller; never invokes the rx callback) ---- */

static void wifi_process_rx_frame_locked(const uint8_t *frame, uint16_t len, int8_t rssi_dbm,
                                          uint8_t *out_eth, uint16_t out_eth_cap,
                                          uint16_t *out_eth_len, bool *out_got_data)
{
    *out_got_data = false;

    if (frame == NULL || len < sizeof(ieee80211_mac_hdr_t)) {
        return;
    }

    const ieee80211_mac_hdr_t *hdr = (const ieee80211_mac_hdr_t *)frame;
    uint16_t fc = hdr->frame_control;
    uint8_t type = (uint8_t)IEEE80211_FCTL_TYPE(fc);
    uint8_t stype = (uint8_t)IEEE80211_FCTL_STYPE(fc);

    if (type == IEEE80211_FC_TYPE_MGMT) {
        if (stype == IEEE80211_FC_STYPE_BEACON || stype == IEEE80211_FC_STYPE_PROBE_RESP) {
            wifi_process_beacon_or_probe_resp_locked(frame, len, rssi_dbm);
        } else if (stype == IEEE80211_FC_STYPE_AUTH && g_wifi.link_state == WIFI_LINK_AUTHENTICATING) {
            /* TODO(hw): verify status_code == IEEE80211_STATUS_SUCCESS, then
             * transmit an Association Request and move to
             * WIFI_LINK_ASSOCIATING. */
        } else if (stype == IEEE80211_FC_STYPE_ASSOC_RESP && g_wifi.link_state == WIFI_LINK_ASSOCIATING) {
            /* TODO(hw): on status == IEEE80211_STATUS_SUCCESS: if
             * target security is WPA2_PSK/WPA3_SAE, run the EAPOL 4-way
             * handshake and install the resulting CCMP key before
             * declaring WIFI_LINK_ASSOCIATED; for WIFI_SECURITY_OPEN,
             * associate immediately. */
        }
        return;
    }

    if (type == IEEE80211_FC_TYPE_DATA && g_wifi.link_state == WIFI_LINK_ASSOCIATED) {
        uint16_t eth_len = wifi_decapsulate_80211_to_8023(frame, len, out_eth, out_eth_cap);
        if (eth_len > 0u) {
            *out_eth_len = eth_len;
            *out_got_data = true;
        }
    }
}

/* ---- public API ---- */

bool wifi_init(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);

    if (g_wifi.hw_ready != 0u) {
        spinlock_unlock(&g_wifi.lock);
        irq_restore(irq_flags);
        return true;
    }

    wifi_pci_dev_t dev;
    if (!find_wifi_controller(&dev)) {
        spinlock_unlock(&g_wifi.lock);
        irq_restore(irq_flags);
        return false;
    }

    uint8_t bar_index = 6u;
    for (uint8_t i = 0; i < 6u; ++i) {
        if (dev.bar_is_mem[i] != 0u && dev.bar_addr[i] != 0u) {
            bar_index = i;
            break;
        }
    }
    if (bar_index >= 6u) {
        spinlock_unlock(&g_wifi.lock);
        irq_restore(irq_flags);
        return false;
    }

    void *mmio = map_mmio_range(dev.bar_addr[bar_index], WIFI_MMIO_WINDOW_BYTES);
    if (mmio == NULL) {
        spinlock_unlock(&g_wifi.lock);
        irq_restore(irq_flags);
        return false;
    }

    g_wifi.pci = dev;
    g_wifi.mmio_base = (volatile uint8_t *)mmio;

    /* TODO(hw): real chipset bring-up belongs here -- firmware/microcode
     * load, radio reset, and reading the station MAC address out of
     * hardware (EEPROM/OTP/NVM). Until a concrete backend exists, a
     * locally-administered placeholder address is used instead. */
    static const uint8_t placeholder_mac[6] = { 0x02u, 0x49u, 0x4Du, 0x50u, 0x4Cu, 0x00u };
    memcpy(g_wifi.mac, placeholder_mac, 6u);

    g_wifi.mtu = WIFI_DEFAULT_MTU;
    g_wifi.link_state = WIFI_LINK_DOWN;
    g_wifi.scanning = false;
    g_wifi.scan_count = 0u;
    g_wifi.hw_ready = 1u;

    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);

    serial_write_string("[WiFi] init: controller found; hardware bring-up not yet implemented "
                         "(driver foundation build)\n");
    return true;
}

bool wifi_is_ready(void)
{
    return g_wifi.link_state == WIFI_LINK_ASSOCIATED;
}

uint16_t wifi_mtu(void)
{
    return g_wifi.mtu;
}

void wifi_get_mac(uint8_t mac_out[6])
{
    if (mac_out == NULL) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);
    memcpy(mac_out, g_wifi.mac, 6u);
    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);
}

bool wifi_send(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < 14u || frame_len > (uint16_t)(g_wifi.mtu + 14u)) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);

    if (g_wifi.link_state != WIFI_LINK_ASSOCIATED) {
        spinlock_unlock(&g_wifi.lock);
        irq_restore(irq_flags);
        return false;
    }

    uint8_t frame80211[WIFI_MAX_80211_FRAME_BYTES];
    uint16_t frame80211_len = wifi_encapsulate_8023_to_80211(frame, frame_len,
                                                              g_wifi.bssid, g_wifi.mac,
                                                              frame80211, sizeof(frame80211));
    bool ok = false;
    if (frame80211_len > 0u) {
        ok = hw_enqueue_tx(frame80211, frame80211_len);
    }

    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);
    return ok;
}

void wifi_poll(void)
{
    uint32_t processed = 0u;

    for (;;) {
        if (processed >= 16u) {
            return;
        }

        uint8_t  rx_buf[WIFI_MAX_80211_FRAME_BYTES];
        uint16_t rx_len = 0u;
        int8_t   rssi = 0;
        uint8_t  eth_frame[WIFI_MAX_ETH_FRAME_BYTES];
        uint16_t eth_len = 0u;
        bool     got_data = false;
        wifi_rx_callback_t cb = NULL;

        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_wifi.lock);

        if (g_wifi.hw_ready == 0u) {
            spinlock_unlock(&g_wifi.lock);
            irq_restore(irq_flags);
            return;
        }

        wifi_advance_scan_timeout_locked();

        if (!hw_dequeue_rx(rx_buf, sizeof(rx_buf), &rx_len, &rssi)) {
            spinlock_unlock(&g_wifi.lock);
            irq_restore(irq_flags);
            return;
        }

        wifi_process_rx_frame_locked(rx_buf, rx_len, rssi, eth_frame, sizeof(eth_frame), &eth_len, &got_data);
        cb = g_wifi.rx_callback;

        spinlock_unlock(&g_wifi.lock);
        irq_restore(irq_flags);

        if (got_data && eth_len > 0u && cb != NULL) {
            cb(eth_frame, eth_len);
        }

        processed++;
    }
}

void wifi_set_rx_callback(wifi_rx_callback_t cb)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);
    g_wifi.rx_callback = cb;
    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);
}

bool wifi_scan_start(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);

    if (g_wifi.hw_ready == 0u ||
        g_wifi.link_state == WIFI_LINK_AUTHENTICATING ||
        g_wifi.link_state == WIFI_LINK_ASSOCIATING) {
        spinlock_unlock(&g_wifi.lock);
        irq_restore(irq_flags);
        return false;
    }

    g_wifi.scan_count = 0u;
    g_wifi.scanning = true;
    g_wifi.scan_start_tick = timer_ticks();
    g_wifi.link_state = WIFI_LINK_SCANNING;

    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);

    /* TODO(hw): transmit a broadcast Probe Request on each supported
     * channel. wifi_poll() -> wifi_process_beacon_or_probe_resp_locked()
     * already parses any Beacon/Probe Response frames handed back by
     * hw_dequeue_rx() into g_wifi.scan_results; only the transmit side and
     * channel stepping need a chipset backend. Until then the scan simply
     * times out (see wifi_advance_scan_timeout_locked()) with an empty
     * result set. */

    return true;
}

bool wifi_scan_is_active(void)
{
    return g_wifi.scanning;
}

uint32_t wifi_scan_get_results(wifi_scan_result_t *out_results, uint32_t max_results)
{
    if (out_results == NULL || max_results == 0u) {
        return 0u;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);

    uint32_t count = g_wifi.scan_count;
    if (count > max_results) {
        count = max_results;
    }
    for (uint32_t i = 0u; i < count; ++i) {
        out_results[i] = g_wifi.scan_results[i];
    }

    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);
    return count;
}

bool wifi_connect(const char *ssid, const char *psk)
{
    if (ssid == NULL) {
        return false;
    }

    size_t ssid_len = wifi_strnlen_local(ssid, WIFI_MAX_SSID_LEN + 1u);
    if (ssid_len == 0u || ssid_len > WIFI_MAX_SSID_LEN) {
        return false;
    }

    size_t psk_len = 0u;
    if (psk != NULL) {
        psk_len = wifi_strnlen_local(psk, WIFI_MAX_PSK_LEN + 1u);
        if (psk_len > WIFI_MAX_PSK_LEN) {
            return false;
        }
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);

    if (g_wifi.hw_ready == 0u) {
        spinlock_unlock(&g_wifi.lock);
        irq_restore(irq_flags);
        return false;
    }

    memcpy(g_wifi.target_ssid, ssid, ssid_len);
    g_wifi.target_ssid[ssid_len] = '\0';
    g_wifi.target_ssid_len = (uint8_t)ssid_len;

    memset(g_wifi.target_psk, 0, sizeof(g_wifi.target_psk));
    if (psk_len > 0u) {
        memcpy(g_wifi.target_psk, psk, psk_len);
    }
    g_wifi.target_psk_len = (uint8_t)psk_len;

    g_wifi.link_state = WIFI_LINK_AUTHENTICATING;
    g_wifi.rssi_dbm = 0;

    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);

    /* TODO(hw): locate target_ssid among the scan results (or issue a
     * directed probe), then transmit an Open/Shared-Key 802.11
     * Authentication frame to its BSSID.
     * wifi_process_rx_frame_locked() advances the state machine from the
     * Authentication/Association responses; on WPA2/WPA3 networks a full
     * EAPOL 4-way handshake and CCMP key install must complete before
     * link_state becomes WIFI_LINK_ASSOCIATED. None of that is wired to a
     * chipset backend yet, so the state machine will remain in
     * WIFI_LINK_AUTHENTICATING until this is implemented. */

    return true;
}

void wifi_disconnect(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);

    /* TODO(hw): transmit a Deauthentication frame to the current BSSID and
     * tear down any installed CCMP key before dropping the association. */
    g_wifi.link_state = WIFI_LINK_DOWN;
    g_wifi.scanning = false;
    memset(g_wifi.bssid, 0, sizeof(g_wifi.bssid));
    g_wifi.rssi_dbm = 0;

    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);
}

wifi_link_state_t wifi_get_link_state(void)
{
    return g_wifi.link_state;
}

int8_t wifi_get_rssi(void)
{
    return g_wifi.rssi_dbm;
}

bool wifi_get_ssid(char *out_ssid, uint8_t *out_len)
{
    if (out_ssid == NULL || out_len == NULL) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_wifi.lock);

    bool connected = (g_wifi.link_state == WIFI_LINK_ASSOCIATED) ||
                      (g_wifi.link_state == WIFI_LINK_AUTHENTICATING) ||
                      (g_wifi.link_state == WIFI_LINK_ASSOCIATING);
    if (connected) {
        memcpy(out_ssid, g_wifi.target_ssid, (size_t)g_wifi.target_ssid_len + 1u);
        *out_len = g_wifi.target_ssid_len;
    }

    spinlock_unlock(&g_wifi.lock);
    irq_restore(irq_flags);
    return connected;
}

#ifdef IMPLUS_DRIVER_MODULE
static const driver_nic_t g_wifi_nic_driver = {
    .init = wifi_init,
    .is_ready = wifi_is_ready,
    .mtu = wifi_mtu,
    .get_mac = wifi_get_mac,
    .send_frame = wifi_send,
    .poll = wifi_poll,
    .set_rx_callback = wifi_set_rx_callback,
};

static void wifi_shutdown(void)
{
    g_wifi.hw_ready = 0u;
    g_wifi.link_state = WIFI_LINK_DOWN;
    g_api = NULL;
}

static const driver_module_descriptor_t g_wifi_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_NIC,
    .load_priority = 55u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_wifi_nic_driver,
    .shutdown = wifi_shutdown,
};

#undef memset
#undef memcpy
#undef map_mmio_range
#undef pci_read_config
#undef pci_write_config
#undef serial_write_string
#undef timer_ticks
#undef timer_hz

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL ||
        api->memset == NULL ||
        api->memcpy == NULL ||
        api->hw.map_mmio_range == NULL ||
        api->pci_read_config == NULL ||
        api->pci_write_config == NULL ||
        api->serial_write_string == NULL ||
        api->timer_ticks == NULL ||
        api->timer_hz == NULL ||
        api->hal.cpu_save_interrupts == NULL ||
        api->hal.cpu_restore_interrupts == NULL ||
        api->hal.cpu_pause == NULL) {
        return NULL;
    }

    g_api = api;
    return &g_wifi_module;
}
#endif
