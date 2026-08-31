#include "ipv4.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Network/arp/ARP.h"
#include "Network/ethernet/Ethernet.h"
#include "network_utils.h"
#include "Core/sync/Spinlock.h"

#define IPV4_HEADER_BYTES_MIN 20u
#define IPV4_VERSION 4u
#define IPV4_DEFAULT_TTL 64u
#define IPV4_MAX_HANDLERS 16u

typedef struct {
    uint8_t protocol;
    ipv4_protocol_handler_t handler;
} ipv4_handler_entry_t;

static spinlock_t g_ipv4_lock = {0};
static uint32_t g_ipv4_local_addr = 0;
static uint32_t g_ipv4_subnet_mask = 0;
static uint32_t g_ipv4_gateway = 0;
static uint16_t g_ipv4_next_ident = 1u;
static ipv4_handler_entry_t g_ipv4_handlers[IPV4_MAX_HANDLERS];

static inline uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline uint32_t read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static inline void write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)((value >> 8) & 0xFFu);
    p[1] = (uint8_t)(value & 0xFFu);
}

static inline void write_be32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)((value >> 24) & 0xFFu);
    p[1] = (uint8_t)((value >> 16) & 0xFFu);
    p[2] = (uint8_t)((value >> 8) & 0xFFu);
    p[3] = (uint8_t)(value & 0xFFu);
}

static int ipv4_is_local_destination(uint32_t dst_ipv4_addr)
{
    if (dst_ipv4_addr == 0xFFFFFFFFu) {
        return 1;
    }

    uint32_t local_addr = 0u;
    uint32_t mask = 0u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipv4_lock);
    local_addr = g_ipv4_local_addr;
    mask = g_ipv4_subnet_mask;
    spinlock_unlock(&g_ipv4_lock);
    irq_restore(irq_flags);

    if (local_addr == 0u) {
        return 0;
    }

    if (dst_ipv4_addr == local_addr) {
        return 1;
    }

    uint32_t subnet_broadcast = (local_addr & mask) | (~mask);
    if (dst_ipv4_addr == subnet_broadcast) {
        return 1;
    }

    return 0;
}

static void ipv4_on_ethernet(const uint8_t src_mac[6],
                             const uint8_t dst_mac[6],
                             const uint8_t *payload,
                             uint16_t payload_len)
{
    (void)src_mac;
    (void)dst_mac;

    if (payload == NULL || payload_len < IPV4_HEADER_BYTES_MIN) {
        return;
    }

    uint8_t version = (uint8_t)((payload[0] >> 4) & 0x0Fu);
    uint8_t ihl_words = (uint8_t)(payload[0] & 0x0Fu);
    uint16_t header_bytes = (uint16_t)ihl_words * 4u;

    if (version != IPV4_VERSION || ihl_words < 5u || header_bytes > payload_len) {
        return;
    }

    uint16_t total_len = read_be16(payload + 2u);
    if (total_len < header_bytes || total_len > payload_len) {
        return;
    }

    uint16_t frag_field = read_be16(payload + 6u);
    if ((frag_field & 0x1FFFu) != 0u || (frag_field & 0x2000u) != 0u) {
        return;
    }

    if (net_checksum16(payload, header_bytes) != 0u) {
        return;
    }

    uint32_t src_ip = read_be32(payload + 12u);
    uint32_t dst_ip = read_be32(payload + 16u);

    if (!ipv4_is_local_destination(dst_ip)) {
        return;
    }

    uint8_t protocol = payload[9];
    const uint8_t *l4_payload = payload + header_bytes;
    uint16_t l4_len = (uint16_t)(total_len - header_bytes);

    ipv4_protocol_handler_t handler = NULL;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipv4_lock);

    for (uint32_t i = 0; i < IPV4_MAX_HANDLERS; ++i) {
        if (g_ipv4_handlers[i].handler != NULL && g_ipv4_handlers[i].protocol == protocol) {
            handler = g_ipv4_handlers[i].handler;
            break;
        }
    }

    spinlock_unlock(&g_ipv4_lock);
    irq_restore(irq_flags);

    if (handler != NULL) {
        handler(src_ip, dst_ip, l4_payload, l4_len);
    }
}

void ipv4_init(uint32_t local_ipv4_addr,
               uint32_t subnet_mask,
               uint32_t gateway_ipv4_addr)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipv4_lock);

    memset(g_ipv4_handlers, 0, sizeof(g_ipv4_handlers));
    g_ipv4_local_addr = local_ipv4_addr;
    g_ipv4_subnet_mask = subnet_mask;
    g_ipv4_gateway = gateway_ipv4_addr;
    g_ipv4_next_ident = 1u;

    spinlock_unlock(&g_ipv4_lock);
    irq_restore(irq_flags);

    (void)ethernet_register_handler(ETHERNET_TYPE_IPV4, ipv4_on_ethernet);
    arp_set_local_ipv4(local_ipv4_addr);
}

uint32_t ipv4_local_address(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipv4_lock);
    uint32_t local = g_ipv4_local_addr;
    spinlock_unlock(&g_ipv4_lock);
    irq_restore(irq_flags);
    return local;
}

bool ipv4_register_protocol(uint8_t protocol, ipv4_protocol_handler_t handler)
{
    if (handler == NULL) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipv4_lock);

    for (uint32_t i = 0; i < IPV4_MAX_HANDLERS; ++i) {
        if (g_ipv4_handlers[i].handler == handler && g_ipv4_handlers[i].protocol == protocol) {
            spinlock_unlock(&g_ipv4_lock);
            irq_restore(irq_flags);
            return true;
        }
    }

    for (uint32_t i = 0; i < IPV4_MAX_HANDLERS; ++i) {
        if (g_ipv4_handlers[i].handler == NULL) {
            g_ipv4_handlers[i].protocol = protocol;
            g_ipv4_handlers[i].handler = handler;
            spinlock_unlock(&g_ipv4_lock);
            irq_restore(irq_flags);
            return true;
        }
    }

    spinlock_unlock(&g_ipv4_lock);
    irq_restore(irq_flags);
    return false;
}

bool ipv4_send(uint32_t dst_ipv4_addr,
               uint8_t protocol,
               const void *payload,
               uint16_t payload_len)
{
    if ((payload == NULL && payload_len != 0u) || dst_ipv4_addr == 0u) {
        return false;
    }

    uint8_t dst_mac[6];
    memset(dst_mac, 0, sizeof(dst_mac));

    uint32_t local_ip = 0u;
    uint32_t mask = 0u;
    uint32_t gateway = 0u;
    uint16_t ident = 0u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipv4_lock);

    local_ip = g_ipv4_local_addr;
    mask = g_ipv4_subnet_mask;
    gateway = g_ipv4_gateway;
    ident = g_ipv4_next_ident++;

    spinlock_unlock(&g_ipv4_lock);
    irq_restore(irq_flags);

    if (local_ip == 0u) {
        return false;
    }

    if (dst_ipv4_addr == 0xFFFFFFFFu) {
        memset(dst_mac, 0xFF, sizeof(dst_mac));
    } else {
        uint32_t next_hop = dst_ipv4_addr;
        if ((dst_ipv4_addr & mask) != (local_ip & mask) && gateway != 0u) {
            next_hop = gateway;
        }

        if (!arp_resolve(next_hop, dst_mac, 1000u)) {
            return false;
        }
    }

    uint32_t total_len32 = (uint32_t)IPV4_HEADER_BYTES_MIN + (uint32_t)payload_len;
    if (total_len32 > 1500u || total_len32 > 0xFFFFu) {
        return false;
    }
    uint16_t total_len = (uint16_t)total_len32;

    uint8_t packet[1520];
    memset(packet, 0, sizeof(packet));

    packet[0] = (uint8_t)((IPV4_VERSION << 4) | 5u);
    packet[1] = 0u;
    write_be16(packet + 2u, total_len);
    write_be16(packet + 4u, ident);
    write_be16(packet + 6u, 0x4000u);
    packet[8] = IPV4_DEFAULT_TTL;
    packet[9] = protocol;
    write_be16(packet + 10u, 0u);
    write_be32(packet + 12u, local_ip);
    write_be32(packet + 16u, dst_ipv4_addr);

    uint16_t header_checksum = net_checksum16(packet, IPV4_HEADER_BYTES_MIN);
    write_be16(packet + 10u, header_checksum);

    if (payload_len > 0u) {
        memcpy(packet + IPV4_HEADER_BYTES_MIN, payload, payload_len);
    }

    return ethernet_send(ETHERNET_TYPE_IPV4, dst_mac, packet, total_len);
}

void ipv4_process_timer(void)
{
    arp_process_timer();
}
