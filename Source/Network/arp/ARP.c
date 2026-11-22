#include "ARP.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Network/ethernet/Ethernet.h"
#include "Network/network_utils.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "Debug/serial/Serial.h"

#define ARP_HTYPE_ETHERNET 1u
#define ARP_PTYPE_IPV4     0x0800u
#define ARP_HLEN_ETHERNET  6u
#define ARP_PLEN_IPV4      4u
#define ARP_OP_REQUEST     1u
#define ARP_OP_REPLY       2u

#define ARP_ENTRY_TTL_SECONDS 300u
/* Past its TTL an entry stays usable while it is re-probed (once a second);
 * only an entry nobody answered for this long is dropped. Dropping it at the
 * TTL sent the next packet to the gateway through a blocking resolve in the
 * middle of the TCP stack (see arp_resolve()). */
#define ARP_ENTRY_STALE_GRACE_SECONDS 60u

typedef struct __attribute__((packed)) {
    uint16_t htype;
    uint16_t ptype;
    uint8_t hlen;
    uint8_t plen;
    uint16_t oper;
    uint8_t sha[6];
    uint32_t spa;
    uint8_t tha[6];
    uint32_t tpa;
} arp_packet_t;

typedef struct {
    uint8_t valid;
    uint8_t mac[6];
    uint32_t ipv4_addr;
    uint64_t last_updated_tick;
    uint64_t last_probe_tick;
} arp_entry_t;

static spinlock_t g_arp_lock = {0};
static arp_entry_t g_arp_table[ARP_TABLE_MAX_ENTRIES];

static uint32_t g_local_ipv4_addr = 0;
/* The last address a non-blocking arp_resolve() asked for, and when: a burst
 * of packets to an unresolved next hop sends one request, not one each. */
static uint32_t g_arp_pending_ip = 0;
static uint64_t g_arp_pending_tick = 0;

static void arp_table_update(uint32_t ipv4_addr, const uint8_t mac[6])
{
    if (mac == NULL || ipv4_addr == 0u) {
        return;
    }

    uint64_t now = timer_ticks();

    for (uint32_t i = 0; i < ARP_TABLE_MAX_ENTRIES; ++i) {
        if (g_arp_table[i].valid != 0u && g_arp_table[i].ipv4_addr == ipv4_addr) {
            memcpy(g_arp_table[i].mac, mac, sizeof(g_arp_table[i].mac));
            g_arp_table[i].last_updated_tick = now;
            return;
        }
    }

    uint32_t victim = 0u;
    uint64_t oldest = UINT64_MAX;

    for (uint32_t i = 0; i < ARP_TABLE_MAX_ENTRIES; ++i) {
        if (g_arp_table[i].valid == 0u) {
            victim = i;
            oldest = 0u;
            break;
        }
        if (g_arp_table[i].last_updated_tick < oldest) {
            oldest = g_arp_table[i].last_updated_tick;
            victim = i;
        }
    }

    g_arp_table[victim].valid = 1u;
    g_arp_table[victim].ipv4_addr = ipv4_addr;
    memcpy(g_arp_table[victim].mac, mac, sizeof(g_arp_table[victim].mac));
    g_arp_table[victim].last_updated_tick = now;
}

static void arp_on_ethernet(const uint8_t src_mac[6],
                            const uint8_t dst_mac[6],
                            const uint8_t *payload,
                            uint16_t payload_len)
{
    (void)dst_mac;

    if (src_mac == NULL || payload == NULL || payload_len < (uint16_t)sizeof(arp_packet_t)) {
        return;
    }

    const arp_packet_t *packet = (const arp_packet_t *)payload;

    uint16_t htype = net_ntohs(packet->htype);
    uint16_t ptype = net_ntohs(packet->ptype);
    uint16_t oper = net_ntohs(packet->oper);

    if (htype != ARP_HTYPE_ETHERNET ||
        ptype != ARP_PTYPE_IPV4 ||
        packet->hlen != ARP_HLEN_ETHERNET ||
        packet->plen != ARP_PLEN_IPV4) {
        return;
    }

    uint32_t sender_ip = net_ntohl(packet->spa);
    uint32_t target_ip = net_ntohl(packet->tpa);

    uint32_t local_ip = 0u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_arp_lock);

    local_ip = g_local_ipv4_addr;
    arp_table_update(sender_ip, packet->sha);

    spinlock_unlock(&g_arp_lock);
    irq_restore(irq_flags);

    if (oper == ARP_OP_REQUEST && local_ip != 0u && target_ip == local_ip) {
        (void)arp_send_reply(packet->sha, sender_ip);
    }
}

void arp_init(uint32_t local_ipv4_addr,
              uint32_t subnet_mask,
              uint32_t gateway_ipv4_addr)
{
    (void)subnet_mask;
    (void)gateway_ipv4_addr;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_arp_lock);

    memset(g_arp_table, 0, sizeof(g_arp_table));
    g_local_ipv4_addr = local_ipv4_addr;

    spinlock_unlock(&g_arp_lock);
    irq_restore(irq_flags);

    (void)ethernet_register_handler(ETHERNET_TYPE_ARP, arp_on_ethernet);
}

void arp_set_local_ipv4(uint32_t local_ipv4_addr)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_arp_lock);
    g_local_ipv4_addr = local_ipv4_addr;
    spinlock_unlock(&g_arp_lock);
    irq_restore(irq_flags);
}

bool arp_lookup(uint32_t ipv4_addr, uint8_t mac_out[6])
{
    if (mac_out == NULL || ipv4_addr == 0u) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_arp_lock);

    for (uint32_t i = 0; i < ARP_TABLE_MAX_ENTRIES; ++i) {
        if (g_arp_table[i].valid != 0u && g_arp_table[i].ipv4_addr == ipv4_addr) {
            memcpy(mac_out, g_arp_table[i].mac, 6u);
            spinlock_unlock(&g_arp_lock);
            irq_restore(irq_flags);
            return true;
        }
    }

    spinlock_unlock(&g_arp_lock);
    irq_restore(irq_flags);
    return false;
}

bool arp_send_request(uint32_t target_ipv4_addr)
{
    if (target_ipv4_addr == 0u) {
        return false;
    }

    uint8_t local_mac[6];
    ethernet_get_local_mac(local_mac);

    uint32_t local_ip = 0u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_arp_lock);
    local_ip = g_local_ipv4_addr;
    spinlock_unlock(&g_arp_lock);
    irq_restore(irq_flags);

    if (local_ip == 0u) {
        return false;
    }

    arp_packet_t packet;
    memset(&packet, 0, sizeof(packet));

    packet.htype = net_htons(ARP_HTYPE_ETHERNET);
    packet.ptype = net_htons(ARP_PTYPE_IPV4);
    packet.hlen = ARP_HLEN_ETHERNET;
    packet.plen = ARP_PLEN_IPV4;
    packet.oper = net_htons(ARP_OP_REQUEST);
    memcpy(packet.sha, local_mac, sizeof(packet.sha));
    packet.spa = net_htonl(local_ip);
    memset(packet.tha, 0, sizeof(packet.tha));
    packet.tpa = net_htonl(target_ipv4_addr);

    static const uint8_t k_broadcast_mac[6] = {
        0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu
    };

    return ethernet_send(ETHERNET_TYPE_ARP,
                         k_broadcast_mac,
                         &packet,
                         (uint16_t)sizeof(packet));
}

bool arp_send_reply(const uint8_t target_mac[6], uint32_t target_ipv4_addr)
{
    if (target_mac == NULL || target_ipv4_addr == 0u) {
        return false;
    }

    uint8_t local_mac[6];
    ethernet_get_local_mac(local_mac);

    uint32_t local_ip = 0u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_arp_lock);
    local_ip = g_local_ipv4_addr;
    spinlock_unlock(&g_arp_lock);
    irq_restore(irq_flags);

    if (local_ip == 0u) {
        return false;
    }

    arp_packet_t packet;
    memset(&packet, 0, sizeof(packet));

    packet.htype = net_htons(ARP_HTYPE_ETHERNET);
    packet.ptype = net_htons(ARP_PTYPE_IPV4);
    packet.hlen = ARP_HLEN_ETHERNET;
    packet.plen = ARP_PLEN_IPV4;
    packet.oper = net_htons(ARP_OP_REPLY);
    memcpy(packet.sha, local_mac, sizeof(packet.sha));
    packet.spa = net_htonl(local_ip);
    memcpy(packet.tha, target_mac, sizeof(packet.tha));
    packet.tpa = net_htonl(target_ipv4_addr);

    return ethernet_send(ETHERNET_TYPE_ARP,
                         target_mac,
                         &packet,
                         (uint16_t)sizeof(packet));
}

static bool arp_irq_state_enabled(uint64_t state)
{
#if defined(__aarch64__)
    return (state & (1ull << 7)) == 0u; /* DAIF.I clear */
#else
    return (state & (1ull << 9)) != 0u; /* RFLAGS.IF */
#endif
}

bool arp_resolve(uint32_t ipv4_addr, uint8_t mac_out[6], uint32_t timeout_ms)
{
    if (mac_out == NULL || ipv4_addr == 0u) {
        return false;
    }

    if (arp_lookup(ipv4_addr, mac_out)) {
        return true;
    }

    uint32_t hz = timer_hz();
    if (hz == 0u) {
        hz = 60u;
    }

    /* Waiting below means polling the NIC and halting until the next
     * interrupt. A caller running with interrupts off holds a spinlock --
     * every TCP transmission runs under the TCP lock, the retransmission
     * timer in the timer interrupt -- and must not do either: the poll
     * dispatches received segments straight into tcp_on_ipv4(), which spins
     * forever on the lock this CPU already holds, and "sti; hlt" lets the
     * timer interrupt in to do the same. Such a caller only starts the
     * lookup; its packet is dropped, and TCP's retransmission sends it once
     * the reply has filled the table. */
    uint64_t irq_state = hal_cpu_save_interrupts();
    hal_cpu_restore_interrupts(irq_state);
    if (!arp_irq_state_enabled(irq_state)) {
        uint64_t now = timer_ticks();
        int send = 0;
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_arp_lock);
        if (g_arp_pending_ip != ipv4_addr ||
            (now - g_arp_pending_tick) >= (uint64_t)hz / 4u + 1u) {
            g_arp_pending_ip = ipv4_addr;
            g_arp_pending_tick = now;
            send = 1;
        }
        spinlock_unlock(&g_arp_lock);
        irq_restore(irq_flags);
        if (send) {
            (void)arp_send_request(ipv4_addr);
        }
        return false;
    }

    (void)arp_send_request(ipv4_addr);

    uint64_t timeout_ticks = ((uint64_t)timeout_ms * (uint64_t)hz + 999u) / 1000u;
    if (timeout_ticks == 0u) {
        timeout_ticks = 1u;
    }

    uint64_t retry_ticks = (uint64_t)hz / 4u;
    if (retry_ticks == 0u) {
        retry_ticks = 1u;
    }

    uint64_t start = timer_ticks();
    uint64_t next_retry = start + retry_ticks;

    while ((timer_ticks() - start) < timeout_ticks) {
        ethernet_poll();

        if (arp_lookup(ipv4_addr, mac_out)) {
            hal_cpu_restore_interrupts(irq_state); /* undo the cli below */
            return true;
        }

        uint64_t now = timer_ticks();
        if (now >= next_retry) {
            (void)arp_send_request(ipv4_addr);
            next_retry = now + retry_ticks;
        }

        // Halt until next interrupt (timer or NIC IRQ) instead of busy-spinning.
        // This drops CPU to near-zero while waiting for the ARP reply.
#if defined(__aarch64__)
        __asm__ volatile("wfi" ::: "memory");
#else
        __asm__ volatile("sti; hlt; cli" ::: "memory");
#endif
    }

    hal_cpu_restore_interrupts(irq_state);
    return false;
}

void arp_process_timer(void)
{
    uint32_t hz = timer_hz();
    if (hz == 0u) {
        hz = 60u;
    }

    uint64_t ttl_ticks = (uint64_t)ARP_ENTRY_TTL_SECONDS * (uint64_t)hz;
    uint64_t grace_ticks = (uint64_t)ARP_ENTRY_STALE_GRACE_SECONDS * (uint64_t)hz;
    uint64_t now = timer_ticks();
    uint32_t probe[8];
    uint32_t probe_count = 0u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_arp_lock);

    for (uint32_t i = 0; i < ARP_TABLE_MAX_ENTRIES; ++i) {
        arp_entry_t *e = &g_arp_table[i];
        if (e->valid == 0u) {
            continue;
        }
        uint64_t age = now - e->last_updated_tick;
        if (age > ttl_ticks + grace_ticks) {
            e->valid = 0u;
            e->ipv4_addr = 0u;
            memset(e->mac, 0, sizeof(e->mac));
            e->last_updated_tick = 0u;
            e->last_probe_tick = 0u;
        } else if (age > ttl_ticks &&
                   (now - e->last_probe_tick) >= (uint64_t)hz &&
                   probe_count < 8u) {
            /* Stale: still used, and asked about again. The reply refreshes
             * it through arp_on_ethernet(). */
            e->last_probe_tick = now;
            probe[probe_count++] = e->ipv4_addr;
        }
    }

    spinlock_unlock(&g_arp_lock);
    irq_restore(irq_flags);

    for (uint32_t i = 0; i < probe_count; ++i) {
        (void)arp_send_request(probe[i]);
    }
}
