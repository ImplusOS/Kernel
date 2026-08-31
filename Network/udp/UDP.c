#include "UDP.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Network/ipv4.h"
#include "Network/network_utils.h"
#include "Core/sync/Spinlock.h"
#include "Core/process/ProcessManager.h"

#define UDP_PROTOCOL_NUMBER 17u
#define UDP_HEADER_BYTES 8u
#define UDP_MAX_BINDINGS 32u
#define UDP_MAX_PAYLOAD 1472u

#define UDP_USER_MAX_BINDINGS 16u
#define UDP_USER_QUEUE_DEPTH  8u

typedef struct {
    uint32_t src_ip;
    uint16_t src_port;
    uint16_t len;
    uint8_t  data[UDP_MAX_PAYLOAD];
} udp_user_pkt_t;

typedef struct {
    uint8_t  used;
    int32_t  owner_pid;
    uint16_t port;
    uint32_t head;
    uint32_t count;
    udp_user_pkt_t queue[UDP_USER_QUEUE_DEPTH];
} udp_user_binding_t;

static spinlock_t g_udp_user_lock = {0};
static udp_user_binding_t g_udp_user_bindings[UDP_USER_MAX_BINDINGS];

static void udp_user_dispatcher(uint32_t src_ipv4_addr,
                                uint16_t src_port,
                                uint32_t dst_ipv4_addr,
                                uint16_t dst_port,
                                const uint8_t *payload,
                                uint16_t payload_len);

typedef struct {
    uint8_t used;
    uint16_t port;
    udp_rx_handler_t handler;
} udp_binding_entry_t;

static spinlock_t g_udp_lock = {0};
static udp_binding_entry_t g_udp_bindings[UDP_MAX_BINDINGS];

static inline uint16_t udp_read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline void udp_write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)((value >> 8) & 0xFFu);
    p[1] = (uint8_t)(value & 0xFFu);
}

static uint32_t checksum_accumulate(uint32_t sum, const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return sum;
    }

    uint16_t i = 0;
    while ((uint16_t)(i + 1u) < len) {
        uint16_t word = (uint16_t)(((uint16_t)data[i] << 8) | (uint16_t)data[i + 1u]);
        sum += (uint32_t)word;
        i = (uint16_t)(i + 2u);
    }

    if ((len & 1u) != 0u) {
        sum += (uint32_t)((uint16_t)data[len - 1u] << 8);
    }

    return sum;
}

static uint16_t checksum_finalize(uint32_t sum)
{
    while ((sum >> 16) != 0u) {
        sum = (sum & 0xFFFFu) + (sum >> 16);
    }
    return (uint16_t)~(sum & 0xFFFFu);
}

static uint16_t udp_checksum(uint32_t src_ip,
                             uint32_t dst_ip,
                             const uint8_t *udp_datagram,
                             uint16_t udp_len)
{
    uint32_t sum = 0u;

    uint8_t pseudo[12];
    pseudo[0] = (uint8_t)((src_ip >> 24) & 0xFFu);
    pseudo[1] = (uint8_t)((src_ip >> 16) & 0xFFu);
    pseudo[2] = (uint8_t)((src_ip >> 8) & 0xFFu);
    pseudo[3] = (uint8_t)(src_ip & 0xFFu);

    pseudo[4] = (uint8_t)((dst_ip >> 24) & 0xFFu);
    pseudo[5] = (uint8_t)((dst_ip >> 16) & 0xFFu);
    pseudo[6] = (uint8_t)((dst_ip >> 8) & 0xFFu);
    pseudo[7] = (uint8_t)(dst_ip & 0xFFu);

    pseudo[8] = 0u;
    pseudo[9] = UDP_PROTOCOL_NUMBER;
    udp_write_be16(&pseudo[10], udp_len);

    sum = checksum_accumulate(sum, pseudo, (uint16_t)sizeof(pseudo));
    sum = checksum_accumulate(sum, udp_datagram, udp_len);

    return checksum_finalize(sum);
}

static void udp_on_ipv4(uint32_t src_ipv4_addr,
                        uint32_t dst_ipv4_addr,
                        const uint8_t *payload,
                        uint16_t payload_len)
{
    if (payload == NULL || payload_len < UDP_HEADER_BYTES) {
        return;
    }

    uint16_t src_port = udp_read_be16(payload + 0u);
    uint16_t dst_port = udp_read_be16(payload + 2u);
    uint16_t udp_len = udp_read_be16(payload + 4u);
    uint16_t checksum = udp_read_be16(payload + 6u);

    if (udp_len < UDP_HEADER_BYTES || udp_len > payload_len) {
        return;
    }

    if (checksum != 0u) {
        if (udp_checksum(src_ipv4_addr, dst_ipv4_addr, payload, udp_len) != 0u) {
            return;
        }
    }

    const uint8_t *udp_payload = payload + UDP_HEADER_BYTES;
    uint16_t udp_payload_len = (uint16_t)(udp_len - UDP_HEADER_BYTES);

    udp_rx_handler_t handler = NULL;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_lock);

    for (uint32_t i = 0; i < UDP_MAX_BINDINGS; ++i) {
        if (g_udp_bindings[i].used != 0u && g_udp_bindings[i].port == dst_port) {
            handler = g_udp_bindings[i].handler;
            break;
        }
    }

    spinlock_unlock(&g_udp_lock);
    irq_restore(irq_flags);

    if (handler != NULL) {
        handler(src_ipv4_addr, src_port, dst_ipv4_addr, dst_port, udp_payload, udp_payload_len);
    }
}

void udp_init(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_lock);
    memset(g_udp_bindings, 0, sizeof(g_udp_bindings));
    spinlock_unlock(&g_udp_lock);
    irq_restore(irq_flags);

    (void)ipv4_register_protocol(UDP_PROTOCOL_NUMBER, udp_on_ipv4);
}

bool udp_bind(uint16_t port, udp_rx_handler_t handler)
{
    if (port == 0u || handler == NULL) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_lock);

    for (uint32_t i = 0; i < UDP_MAX_BINDINGS; ++i) {
        if (g_udp_bindings[i].used != 0u && g_udp_bindings[i].port == port) {
            if (g_udp_bindings[i].handler == handler) {
                spinlock_unlock(&g_udp_lock);
                irq_restore(irq_flags);
                return true;
            }
            spinlock_unlock(&g_udp_lock);
            irq_restore(irq_flags);
            return false;
        }
    }

    for (uint32_t i = 0; i < UDP_MAX_BINDINGS; ++i) {
        if (g_udp_bindings[i].used == 0u) {
            g_udp_bindings[i].used = 1u;
            g_udp_bindings[i].port = port;
            g_udp_bindings[i].handler = handler;
            spinlock_unlock(&g_udp_lock);
            irq_restore(irq_flags);
            return true;
        }
    }

    spinlock_unlock(&g_udp_lock);
    irq_restore(irq_flags);
    return false;
}

void udp_unbind(uint16_t port)
{
    if (port == 0u) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_lock);

    for (uint32_t i = 0; i < UDP_MAX_BINDINGS; ++i) {
        if (g_udp_bindings[i].used != 0u && g_udp_bindings[i].port == port) {
            g_udp_bindings[i].used = 0u;
            g_udp_bindings[i].port = 0u;
            g_udp_bindings[i].handler = NULL;
            break;
        }
    }

    spinlock_unlock(&g_udp_lock);
    irq_restore(irq_flags);
}

bool udp_send(uint32_t dst_ipv4_addr,
              uint16_t src_port,
              uint16_t dst_port,
              const void *payload,
              uint16_t payload_len)
{
    if (dst_ipv4_addr == 0u || src_port == 0u || dst_port == 0u) {
        return false;
    }
    if (payload == NULL && payload_len != 0u) {
        return false;
    }
    if (payload_len > UDP_MAX_PAYLOAD) {
        return false;
    }

    uint16_t udp_len = (uint16_t)(UDP_HEADER_BYTES + payload_len);
    uint8_t datagram[UDP_HEADER_BYTES + UDP_MAX_PAYLOAD];
    memset(datagram, 0, sizeof(datagram));

    udp_write_be16(datagram + 0u, src_port);
    udp_write_be16(datagram + 2u, dst_port);
    udp_write_be16(datagram + 4u, udp_len);
    udp_write_be16(datagram + 6u, 0u);

    if (payload_len > 0u) {
        memcpy(datagram + UDP_HEADER_BYTES, payload, payload_len);
    }

    uint32_t src_ip = ipv4_local_address();
    if (src_ip == 0u) {
        return false;
    }

    uint16_t checksum = udp_checksum(src_ip, dst_ipv4_addr, datagram, udp_len);
    if (checksum == 0u) {
        checksum = 0xFFFFu;
    }
    udp_write_be16(datagram + 6u, checksum);

    return ipv4_send(dst_ipv4_addr, UDP_PROTOCOL_NUMBER, datagram, udp_len);
}

bool udp_syscall_send(uint32_t dst_ipv4_addr,
                      uint16_t src_port,
                      uint16_t dst_port,
                      const void *payload,
                      uint16_t payload_len)
{
    if (!process_user_buffer_is_valid(payload, payload_len)) {
        return false;
    }
    return udp_send(dst_ipv4_addr, src_port, dst_port, payload, payload_len);
}

static udp_user_binding_t *udp_user_find_locked(uint16_t port)
{
    for (uint32_t i = 0; i < UDP_USER_MAX_BINDINGS; ++i) {
        if (g_udp_user_bindings[i].used != 0u &&
            g_udp_user_bindings[i].port == port) {
            return &g_udp_user_bindings[i];
        }
    }
    return NULL;
}

static void udp_user_dispatcher(uint32_t src_ipv4_addr,
                                uint16_t src_port,
                                uint32_t dst_ipv4_addr,
                                uint16_t dst_port,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
    (void)dst_ipv4_addr;

    if (payload == NULL || payload_len == 0u ||
        payload_len > UDP_MAX_PAYLOAD) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_user_lock);

    udp_user_binding_t *b = udp_user_find_locked(dst_port);
    if (b == NULL) {
        spinlock_unlock(&g_udp_user_lock);
        irq_restore(irq_flags);
        return;
    }

    if (b->count >= UDP_USER_QUEUE_DEPTH) {
        b->head = (b->head + 1u) % UDP_USER_QUEUE_DEPTH;
        b->count--;
    }

    uint32_t tail = (b->head + b->count) % UDP_USER_QUEUE_DEPTH;
    udp_user_pkt_t *slot = &b->queue[tail];
    slot->src_ip = src_ipv4_addr;
    slot->src_port = src_port;
    slot->len = payload_len;
    memcpy(slot->data, payload, payload_len);
    b->count++;

    spinlock_unlock(&g_udp_user_lock);
    irq_restore(irq_flags);
}

int32_t udp_user_bind(int32_t owner_pid, uint16_t port)
{
    if (port == 0u) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_user_lock);

    for (uint32_t i = 0; i < UDP_USER_MAX_BINDINGS; ++i) {
        if (g_udp_user_bindings[i].used != 0u &&
            g_udp_user_bindings[i].port == port) {
            spinlock_unlock(&g_udp_user_lock);
            irq_restore(irq_flags);
            return -1;
        }
    }

    udp_user_binding_t *slot = NULL;
    for (uint32_t i = 0; i < UDP_USER_MAX_BINDINGS; ++i) {
        if (g_udp_user_bindings[i].used == 0u) {
            slot = &g_udp_user_bindings[i];
            break;
        }
    }

    if (slot == NULL) {
        spinlock_unlock(&g_udp_user_lock);
        irq_restore(irq_flags);
        return -1;
    }

    slot->used = 1u;
    slot->owner_pid = owner_pid;
    slot->port = port;
    slot->head = 0u;
    slot->count = 0u;

    spinlock_unlock(&g_udp_user_lock);
    irq_restore(irq_flags);

    if (!udp_bind(port, udp_user_dispatcher)) {
        irq_flags = irq_save_disable();
        spinlock_lock(&g_udp_user_lock);
        slot->used = 0u;
        slot->owner_pid = 0;
        slot->port = 0u;
        spinlock_unlock(&g_udp_user_lock);
        irq_restore(irq_flags);
        return -1;
    }

    return 0;
}

int32_t udp_user_unbind(int32_t owner_pid, uint16_t port)
{
    if (port == 0u) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_user_lock);

    udp_user_binding_t *b = udp_user_find_locked(port);
    if (b == NULL || b->owner_pid != owner_pid) {
        spinlock_unlock(&g_udp_user_lock);
        irq_restore(irq_flags);
        return -1;
    }

    b->used = 0u;
    b->owner_pid = 0;
    b->port = 0u;
    b->head = 0u;
    b->count = 0u;

    spinlock_unlock(&g_udp_user_lock);
    irq_restore(irq_flags);

    udp_unbind(port);
    return 0;
}

int32_t udp_user_recv(int32_t owner_pid,
                      uint16_t port,
                      uint8_t *out_buf,
                      uint32_t out_buf_len)
{
    if (out_buf == NULL || out_buf_len < UDP_USER_HEADER_BYTES) {
        return -1;
    }

    uint32_t src_ip = 0u;
    uint16_t src_port = 0u;
    uint16_t payload_len = 0u;
    uint8_t local_copy[UDP_MAX_PAYLOAD];

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_user_lock);

    udp_user_binding_t *b = udp_user_find_locked(port);
    if (b == NULL || b->owner_pid != owner_pid) {
        spinlock_unlock(&g_udp_user_lock);
        irq_restore(irq_flags);
        return -1;
    }
    if (b->count == 0u) {
        spinlock_unlock(&g_udp_user_lock);
        irq_restore(irq_flags);
        return 0;
    }

    udp_user_pkt_t *slot = &b->queue[b->head];
    src_ip = slot->src_ip;
    src_port = slot->src_port;
    payload_len = slot->len;
    if (payload_len > UDP_MAX_PAYLOAD) {
        payload_len = UDP_MAX_PAYLOAD;
    }
    memcpy(local_copy, slot->data, payload_len);

    b->head = (b->head + 1u) % UDP_USER_QUEUE_DEPTH;
    b->count--;

    spinlock_unlock(&g_udp_user_lock);
    irq_restore(irq_flags);

    uint32_t max_payload = out_buf_len - UDP_USER_HEADER_BYTES;
    uint16_t copy_len = payload_len;
    if ((uint32_t)copy_len > max_payload) {
        copy_len = (uint16_t)max_payload;
    }
    
    out_buf[0] = (uint8_t)((src_ip >> 0) & 0xFFu);
    out_buf[1] = (uint8_t)((src_ip >> 8) & 0xFFu);
    out_buf[2] = (uint8_t)((src_ip >> 16) & 0xFFu);
    out_buf[3] = (uint8_t)((src_ip >> 24) & 0xFFu);
    out_buf[4] = (uint8_t)((src_port >> 0) & 0xFFu);
    out_buf[5] = (uint8_t)((src_port >> 8) & 0xFFu);
    out_buf[6] = (uint8_t)((copy_len >> 0) & 0xFFu);
    out_buf[7] = (uint8_t)((copy_len >> 8) & 0xFFu);

    if (copy_len > 0u) {
        memcpy(out_buf + UDP_USER_HEADER_BYTES, local_copy, copy_len);
    }

    return (int32_t)(UDP_USER_HEADER_BYTES + (uint32_t)copy_len);
}

int32_t udp_user_available(int32_t owner_pid, uint16_t port)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_user_lock);
    udp_user_binding_t *b = udp_user_find_locked(port);
    int32_t count = (b != NULL && b->owner_pid == owner_pid) ?
        (int32_t)b->count : -1;
    spinlock_unlock(&g_udp_user_lock);
    irq_restore(irq_flags);
    return count;
}

void udp_user_release_all(int32_t owner_pid)
{
    uint16_t released_ports[UDP_USER_MAX_BINDINGS];
    uint32_t released_count = 0u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_udp_user_lock);

    for (uint32_t i = 0; i < UDP_USER_MAX_BINDINGS; ++i) {
        if (g_udp_user_bindings[i].used != 0u &&
            g_udp_user_bindings[i].owner_pid == owner_pid) {
            released_ports[released_count++] = g_udp_user_bindings[i].port;
            g_udp_user_bindings[i].used = 0u;
            g_udp_user_bindings[i].owner_pid = 0;
            g_udp_user_bindings[i].port = 0u;
            g_udp_user_bindings[i].head = 0u;
            g_udp_user_bindings[i].count = 0u;
        }
    }

    spinlock_unlock(&g_udp_user_lock);
    irq_restore(irq_flags);

    for (uint32_t i = 0; i < released_count; ++i) {
        udp_unbind(released_ports[i]);
    }
}
