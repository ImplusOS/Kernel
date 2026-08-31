#include "TCP.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Network/ipv4.h"
#include "Network/network_utils.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "Debug/serial/Serial.h"

#define TCP_HEADER_BYTES_MIN 20u
#define TCP_MAX_PAYLOAD      1460u

typedef struct __attribute__((packed)) {
    uint16_t src_port;
    uint16_t dst_port;
    uint32_t seq_num;
    uint32_t ack_num;
    uint8_t  data_offset_reserved;
    uint8_t  flags;
    uint16_t window;
    uint16_t checksum;
    uint16_t urgent_ptr;
} tcp_header_t;

static spinlock_t g_tcp_lock = {0};
static tcp_connection_t g_tcp_connections[TCP_MAX_CONNECTIONS];
static uint32_t g_tcp_seq_counter = 0x01000000u;

static inline uint16_t tcp_read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline uint32_t tcp_read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
           (uint32_t)p[3];
}

static inline void tcp_write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)((value >> 8) & 0xFFu);
    p[1] = (uint8_t)(value & 0xFFu);
}

static inline void tcp_write_be32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)((value >> 24) & 0xFFu);
    p[1] = (uint8_t)((value >> 16) & 0xFFu);
    p[2] = (uint8_t)((value >> 8)  & 0xFFu);
    p[3] = (uint8_t)(value & 0xFFu);
}

static uint32_t tcp_generate_isn(void)
{
    uint32_t ticks = (uint32_t)(timer_ticks() & 0xFFFFFFFFu);
    uint32_t hash = ticks;
    hash ^= hash << 13;
    hash ^= hash >> 17;
    hash ^= hash << 5;
    hash += 0xdeadbeefu;
    hash ^= g_tcp_seq_counter;
    g_tcp_seq_counter += 64000u + (hash & 0xFFFFu);
    return g_tcp_seq_counter ^ (hash << 16);
}

static uint32_t checksum_accumulate(uint32_t sum, const uint8_t *data, uint16_t len)
{
    if (data == NULL) return sum;
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

static uint16_t tcp_checksum(uint32_t src_ip, uint32_t dst_ip,
                             const uint8_t *tcp_segment, uint16_t tcp_len)
{
    uint32_t sum = 0u;
    uint8_t pseudo[12];
    pseudo[0]  = (uint8_t)((src_ip >> 24) & 0xFFu);
    pseudo[1]  = (uint8_t)((src_ip >> 16) & 0xFFu);
    pseudo[2]  = (uint8_t)((src_ip >> 8)  & 0xFFu);
    pseudo[3]  = (uint8_t)(src_ip & 0xFFu);
    pseudo[4]  = (uint8_t)((dst_ip >> 24) & 0xFFu);
    pseudo[5]  = (uint8_t)((dst_ip >> 16) & 0xFFu);
    pseudo[6]  = (uint8_t)((dst_ip >> 8)  & 0xFFu);
    pseudo[7]  = (uint8_t)(dst_ip & 0xFFu);
    pseudo[8]  = 0u;
    pseudo[9]  = TCP_PROTOCOL_NUMBER;
    tcp_write_be16(&pseudo[10], tcp_len);

    sum = checksum_accumulate(sum, pseudo, (uint16_t)sizeof(pseudo));
    sum = checksum_accumulate(sum, tcp_segment, tcp_len);
    return checksum_finalize(sum);
}

static bool tcp_send_segment(uint32_t src_ip, uint32_t dst_ip,
                             uint16_t src_port, uint16_t dst_port,
                             uint32_t seq, uint32_t ack,
                             uint8_t flags, uint16_t window,
                             const void *data, uint16_t data_len)
{
    uint16_t header_len = TCP_HEADER_BYTES_MIN;
    uint16_t total_len = (uint16_t)(header_len + data_len);
    uint8_t segment[TCP_HEADER_BYTES_MIN + TCP_MAX_PAYLOAD];

    if (total_len > sizeof(segment)) return false;
    memset(segment, 0, sizeof(segment));
    
    tcp_write_be16(segment + 0, src_port);
    tcp_write_be16(segment + 2, dst_port);
    tcp_write_be32(segment + 4, seq);
    tcp_write_be32(segment + 8, ack);
    segment[12] = (uint8_t)((header_len / 4u) << 4);
    segment[13] = flags;
    tcp_write_be16(segment + 14, window);
    tcp_write_be16(segment + 16, 0u);
    tcp_write_be16(segment + 18, 0u);

    if (data_len > 0u && data != NULL) {
        memcpy(segment + header_len, data, data_len);
    }

    if (src_ip == 0u) src_ip = ipv4_local_address();
    uint16_t cksum = tcp_checksum(src_ip, dst_ip, segment, total_len);
    if (cksum == 0u) cksum = 0xFFFFu;
    tcp_write_be16(segment + 16, cksum);

    return ipv4_send(dst_ip, TCP_PROTOCOL_NUMBER, segment, total_len);
}

static void tcp_send_rst(uint32_t src_ip, uint32_t dst_ip,
                         uint16_t src_port, uint16_t dst_port,
                         uint32_t seq, uint32_t ack)
{
    tcp_send_segment(src_ip, dst_ip, src_port, dst_port,
                     seq, ack, TCP_FLAG_RST | TCP_FLAG_ACK,
                     0, NULL, 0);
}

static int32_t tcp_find_connection(uint32_t local_ip, uint16_t local_port,
                                   uint32_t remote_ip, uint16_t remote_port)
{
    for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
        if (g_tcp_connections[i].in_use == 0u) continue;
        tcp_connection_t *c = &g_tcp_connections[i];
        if (c->local_port == local_port && c->remote_port == remote_port &&
            c->remote_ip == remote_ip &&
            (c->local_ip == local_ip || c->local_ip == 0u)) {
            return (int32_t)i;
        }
    }
    return -1;
}

static int32_t tcp_find_listener(uint16_t local_port)
{
    for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
        if (g_tcp_connections[i].in_use == 0u) continue;
        if (g_tcp_connections[i].state == TCP_STATE_LISTEN &&
            g_tcp_connections[i].local_port == local_port) {
            return (int32_t)i;
        }
    }
    return -1;
}

static int tcp_local_port_in_use_locked(uint16_t local_port)
{
    if (local_port == 0u) {
        return 1;
    }

    for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
        const tcp_connection_t *c = &g_tcp_connections[i];
        if (c->in_use != 0u &&
            c->state != TCP_STATE_CLOSED &&
            c->local_port == local_port) {
            return 1;
        }
    }
    return 0;
}

int tcp_local_port_in_use(uint16_t local_port)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);
    int in_use = tcp_local_port_in_use_locked(local_port);
    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return in_use;
}

static int32_t tcp_alloc_connection(void)
{
    for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
        if (g_tcp_connections[i].in_use == 0u) {
            memset(&g_tcp_connections[i], 0, sizeof(tcp_connection_t));
            g_tcp_connections[i].in_use = 1u;
            return (int32_t)i;
        }
    }
    return -1;
}

static uint16_t recv_buf_free(const tcp_connection_t *c)
{
    return (uint16_t)(TCP_RECV_BUF_SIZE - c->recv_count);
}

static uint16_t ring_write_partial(uint8_t *ring, uint16_t ring_size,
                                   uint16_t *index, const uint8_t *data,
                                   uint16_t len)
{
    uint16_t written = 0;
    uint16_t first = (uint16_t)(ring_size - *index);
    if (first > len) first = len;
    if (first > 0u) {
        memcpy(ring + *index, data, first);
        *index = (uint16_t)((*index + first) % ring_size);
        written = first;
    }
    if (len > written) {
        uint16_t rest = (uint16_t)(len - written);
        memcpy(ring, data + written, rest);
        *index = rest;
        written = len;
    }
    return written;
}

static uint16_t ring_read_partial(uint8_t *ring, uint16_t ring_size,
                                  uint16_t *index, uint8_t *dst,
                                  uint16_t len)
{
    uint16_t read = 0;
    uint16_t first = (uint16_t)(ring_size - *index);
    if (first > len) first = len;
    if (first > 0u) {
        memcpy(dst, ring + *index, first);
        *index = (uint16_t)((*index + first) % ring_size);
        read = first;
    }
    if (len > read) {
        uint16_t rest = (uint16_t)(len - read);
        memcpy(dst + read, ring, rest);
        *index = rest;
        read = len;
    }
    return read;
}

static void recv_buf_write(tcp_connection_t *c, const uint8_t *data, uint16_t len)
{
    uint16_t space = (uint16_t)(TCP_RECV_BUF_SIZE - c->recv_count);
    if (len > space) len = space;
    if (len == 0u) return;
    uint16_t written = ring_write_partial(c->recv_buf, TCP_RECV_BUF_SIZE,
                                          &c->recv_head, data, len);
    c->recv_count = (uint16_t)(c->recv_count + written);
}

static uint16_t recv_buf_read(tcp_connection_t *c, uint8_t *buf, uint16_t len)
{
    if (len > c->recv_count) len = c->recv_count;
    if (len == 0u) return 0;
    uint16_t read = ring_read_partial(c->recv_buf, TCP_RECV_BUF_SIZE,
                                      &c->recv_tail, buf, len);
    c->recv_count = (uint16_t)(c->recv_count - read);
    return read;
}

static void tcp_on_ipv4(uint32_t src_ip, uint32_t dst_ip,
                        const uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL || payload_len < TCP_HEADER_BYTES_MIN) return;

    uint16_t src_port  = tcp_read_be16(payload + 0);
    uint16_t dst_port  = tcp_read_be16(payload + 2);
    uint32_t seq_num   = tcp_read_be32(payload + 4);
    uint32_t ack_num   = tcp_read_be32(payload + 8);
    uint8_t  data_off  = (uint8_t)((payload[12] >> 4) & 0x0Fu);
    uint8_t  flags     = payload[13];
    uint16_t window    = tcp_read_be16(payload + 14);
    uint16_t hdr_bytes = (uint16_t)(data_off * 4u);

    if (hdr_bytes < TCP_HEADER_BYTES_MIN || hdr_bytes > payload_len) return;

    if (tcp_checksum(src_ip, dst_ip, payload, payload_len) != 0u) return;

    const uint8_t *seg_data = payload + hdr_bytes;
    uint16_t seg_data_len = (uint16_t)(payload_len - hdr_bytes);

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    int32_t conn_id = tcp_find_connection(dst_ip, dst_port, src_ip, src_port);

    if (conn_id < 0) {
        int32_t listen_id = tcp_find_listener(dst_port);

        if (listen_id >= 0 && (flags & TCP_FLAG_SYN) && !(flags & TCP_FLAG_ACK)) {
            uint16_t queued = 0u;
            for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
                const tcp_connection_t *candidate = &g_tcp_connections[i];
                if (candidate->in_use != 0u &&
                    candidate->parent_conn_id == listen_id &&
                    candidate->state != TCP_STATE_CLOSED) {
                    ++queued;
                }
            }
            uint16_t backlog = g_tcp_connections[listen_id].listen_backlog;
            if (backlog == 0u) backlog = 1u;
            if (queued >= backlog) {
                tcp_send_rst(dst_ip, src_ip, dst_port, src_port,
                             0u, seq_num + 1u);
                spinlock_unlock(&g_tcp_lock);
                irq_restore(irq_flags);
                return;
            }
            int32_t child_id = tcp_alloc_connection();
            if (child_id >= 0) {
                tcp_connection_t *child = &g_tcp_connections[child_id];
                child->local_ip    = dst_ip;
                child->remote_ip   = src_ip;
                child->local_port  = dst_port;
                child->remote_port = src_port;
                child->state       = TCP_STATE_SYN_RECEIVED;
                child->irs         = seq_num;
                child->rcv_nxt     = seq_num + 1u;
                child->rcv_wnd     = TCP_DEFAULT_WINDOW;
                child->iss         = tcp_generate_isn();
                child->snd_una     = child->iss;
                child->snd_nxt     = child->iss + 1u;
                child->snd_wnd     = window;
                child->parent_conn_id = listen_id;
                child->accept_pending = 1u;
                child->last_send_tick = timer_ticks();

                tcp_send_segment(dst_ip, src_ip, dst_port, src_port,
                                 child->iss, child->rcv_nxt,
                                 TCP_FLAG_SYN | TCP_FLAG_ACK,
                                 child->rcv_wnd, NULL, 0);
            }
        } else if (!(flags & TCP_FLAG_RST)) {
            uint32_t rst_seq = 0u;
            uint32_t rst_ack = seq_num + (uint32_t)seg_data_len;
            if (flags & TCP_FLAG_SYN) rst_ack++;
            tcp_send_rst(dst_ip, src_ip, dst_port, src_port, rst_seq, rst_ack);
        }

        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return;
    }

    tcp_connection_t *conn = &g_tcp_connections[conn_id];

    if (flags & TCP_FLAG_RST) {
        conn->state = TCP_STATE_CLOSED;
        conn->in_use = 0u;
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return;
    }

    switch (conn->state) {
    case TCP_STATE_SYN_SENT:
        if ((flags & (TCP_FLAG_SYN | TCP_FLAG_ACK)) == (TCP_FLAG_SYN | TCP_FLAG_ACK)) {
            if (ack_num == conn->snd_nxt) {
                conn->irs     = seq_num;
                conn->rcv_nxt = seq_num + 1u;
                conn->snd_una = ack_num;
                conn->snd_wnd = window;
                conn->state   = TCP_STATE_ESTABLISHED;
                conn->retransmit_count = 0;

                tcp_send_segment(conn->local_ip, conn->remote_ip,
                                 conn->local_port, conn->remote_port,
                                 conn->snd_nxt, conn->rcv_nxt,
                                 TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
            }
        }
        break;

    case TCP_STATE_SYN_RECEIVED:
        if ((flags & TCP_FLAG_ACK) && ack_num == conn->snd_nxt) {
            conn->snd_una = ack_num;
            conn->snd_wnd = window;
            conn->state   = TCP_STATE_ESTABLISHED;
            conn->retransmit_count = 0;
        }
        break;

    case TCP_STATE_ESTABLISHED:
        if (flags & TCP_FLAG_ACK) {
            if (ack_num > conn->snd_una && ack_num <= conn->snd_nxt) {
                conn->snd_una = ack_num;
                conn->retransmit_count = 0;
            }
            conn->snd_wnd = window;
        }

        if (seg_data_len > 0u && seq_num == conn->rcv_nxt) {
            uint16_t space = recv_buf_free(conn);
            uint16_t to_copy = (seg_data_len < space) ? seg_data_len : space;
            if (to_copy > 0u) {
                recv_buf_write(conn, seg_data, to_copy);
                conn->rcv_nxt += to_copy;
            }
            conn->rcv_wnd = recv_buf_free(conn);
            tcp_send_segment(conn->local_ip, conn->remote_ip,
                             conn->local_port, conn->remote_port,
                             conn->snd_nxt, conn->rcv_nxt,
                             TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
        }

        if (flags & TCP_FLAG_FIN) {
            conn->rcv_nxt = seq_num + (uint32_t)seg_data_len + 1u;
            conn->state = TCP_STATE_CLOSE_WAIT;
            tcp_send_segment(conn->local_ip, conn->remote_ip,
                             conn->local_port, conn->remote_port,
                             conn->snd_nxt, conn->rcv_nxt,
                             TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
        }
        break;

    case TCP_STATE_FIN_WAIT_1:
        if (flags & TCP_FLAG_ACK) {
            if (ack_num == conn->snd_nxt) {
                conn->snd_una = ack_num;
                if (flags & TCP_FLAG_FIN) {
                    conn->rcv_nxt = seq_num + 1u;
                    conn->state = TCP_STATE_TIME_WAIT;
                    conn->time_wait_start = timer_ticks();
                    tcp_send_segment(conn->local_ip, conn->remote_ip,
                                     conn->local_port, conn->remote_port,
                                     conn->snd_nxt, conn->rcv_nxt,
                                     TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
                } else {
                    conn->state = TCP_STATE_FIN_WAIT_2;
                }
            }
        }
        if (conn->state == TCP_STATE_FIN_WAIT_1 && (flags & TCP_FLAG_FIN)) {
            conn->rcv_nxt = seq_num + 1u;
            conn->state = TCP_STATE_CLOSING;
            tcp_send_segment(conn->local_ip, conn->remote_ip,
                             conn->local_port, conn->remote_port,
                             conn->snd_nxt, conn->rcv_nxt,
                             TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
        }
        break;

    case TCP_STATE_FIN_WAIT_2:
        if (seg_data_len > 0u && seq_num == conn->rcv_nxt) {
            uint16_t space = recv_buf_free(conn);
            uint16_t to_copy = (seg_data_len < space) ? seg_data_len : space;
            if (to_copy > 0u) {
                recv_buf_write(conn, seg_data, to_copy);
                conn->rcv_nxt += to_copy;
            }
            conn->rcv_wnd = recv_buf_free(conn);
            tcp_send_segment(conn->local_ip, conn->remote_ip,
                             conn->local_port, conn->remote_port,
                             conn->snd_nxt, conn->rcv_nxt,
                             TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
        }
        if (flags & TCP_FLAG_FIN) {
            conn->rcv_nxt = seq_num + (uint32_t)seg_data_len + 1u;
            conn->state = TCP_STATE_TIME_WAIT;
            conn->time_wait_start = timer_ticks();
            tcp_send_segment(conn->local_ip, conn->remote_ip,
                             conn->local_port, conn->remote_port,
                             conn->snd_nxt, conn->rcv_nxt,
                             TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
        }
        break;

    case TCP_STATE_CLOSING:
        if ((flags & TCP_FLAG_ACK) && ack_num == conn->snd_nxt) {
            conn->state = TCP_STATE_TIME_WAIT;
            conn->time_wait_start = timer_ticks();
        }
        break;

    case TCP_STATE_LAST_ACK:
        if ((flags & TCP_FLAG_ACK) && ack_num == conn->snd_nxt) {
            conn->state = TCP_STATE_CLOSED;
            conn->in_use = 0u;
        }
        break;

    case TCP_STATE_CLOSE_WAIT:
        if (flags & TCP_FLAG_ACK) {
            if (ack_num > conn->snd_una && ack_num <= conn->snd_nxt) {
                conn->snd_una = ack_num;
            }
        }
        break;

    case TCP_STATE_TIME_WAIT:
        if (flags & TCP_FLAG_FIN) {
            tcp_send_segment(conn->local_ip, conn->remote_ip,
                             conn->local_port, conn->remote_port,
                             conn->snd_nxt, conn->rcv_nxt,
                             TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
            conn->time_wait_start = timer_ticks();
        }
        break;

    default:
        break;
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
}

void tcp_init(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);
    memset(g_tcp_connections, 0, sizeof(g_tcp_connections));
    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);

    (void)ipv4_register_protocol(TCP_PROTOCOL_NUMBER, tcp_on_ipv4);
}

int32_t tcp_connect(uint32_t remote_ip, uint16_t remote_port, uint16_t local_port)
{
    if (remote_ip == 0u || remote_port == 0u || local_port == 0u) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    if (tcp_local_port_in_use_locked(local_port)) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    int32_t conn_id = tcp_alloc_connection();
    if (conn_id < 0) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    tcp_connection_t *conn = &g_tcp_connections[conn_id];
    conn->local_ip    = ipv4_local_address();
    conn->remote_ip   = remote_ip;
    conn->local_port  = local_port;
    conn->remote_port = remote_port;
    conn->iss         = tcp_generate_isn();
    conn->snd_una     = conn->iss;
    conn->snd_nxt     = conn->iss + 1u;
    conn->rcv_wnd     = TCP_DEFAULT_WINDOW;
    conn->snd_wnd     = TCP_DEFAULT_WINDOW;
    conn->state       = TCP_STATE_SYN_SENT;
    conn->last_send_tick = timer_ticks();
    conn->parent_conn_id = -1;
    
    tcp_send_segment(conn->local_ip, conn->remote_ip,
                     conn->local_port, conn->remote_port,
                     conn->iss, 0u,
                     TCP_FLAG_SYN, conn->rcv_wnd, NULL, 0);

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return conn_id;
}

int32_t tcp_listen(uint16_t port)
{
    if (port == 0u) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    
    if (tcp_find_listener(port) >= 0) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    int32_t conn_id = tcp_alloc_connection();
    if (conn_id < 0) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    tcp_connection_t *conn = &g_tcp_connections[conn_id];
    conn->local_ip    = 0u;  
    conn->local_port  = port;
    conn->state       = TCP_STATE_LISTEN;
    conn->parent_conn_id = -1;
    conn->listen_backlog = 1u;

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return conn_id;
}

int tcp_set_listen_backlog(int32_t conn_id, uint16_t backlog)
{
    if (conn_id < 0 || conn_id >= (int32_t)TCP_MAX_CONNECTIONS ||
        backlog == 0u) {
        return -1;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);
    tcp_connection_t *connection = &g_tcp_connections[conn_id];
    if (connection->in_use == 0u ||
        connection->state != TCP_STATE_LISTEN) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }
    connection->listen_backlog = backlog;
    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return 0;
}

int32_t tcp_accept(int32_t listen_conn_id)
{
    if (listen_conn_id < 0 || listen_conn_id >= (int32_t)TCP_MAX_CONNECTIONS) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    tcp_connection_t *listener = &g_tcp_connections[listen_conn_id];
    if (listener->in_use == 0u || listener->state != TCP_STATE_LISTEN) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    
    for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
        tcp_connection_t *c = &g_tcp_connections[i];
        if (c->in_use != 0u &&
            c->parent_conn_id == listen_conn_id &&
            c->accept_pending != 0u &&
            c->state == TCP_STATE_ESTABLISHED) {
            c->accept_pending = 0u;
            spinlock_unlock(&g_tcp_lock);
            irq_restore(irq_flags);
            return (int32_t)i;
        }
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return -1;
}

int32_t tcp_send(int32_t conn_id, const void *data, uint16_t len)
{
    if (conn_id < 0 || conn_id >= (int32_t)TCP_MAX_CONNECTIONS) return -1;
    if (data == NULL && len != 0u) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    tcp_connection_t *conn = &g_tcp_connections[conn_id];
    if (conn->in_use == 0u || conn->state != TCP_STATE_ESTABLISHED) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    const uint8_t *ptr = (const uint8_t *)data;
    uint16_t sent = 0u;

    while (sent < len) {
        uint16_t chunk = (uint16_t)(len - sent);
        if (chunk > TCP_MAX_SEGMENT_DATA) chunk = TCP_MAX_SEGMENT_DATA;

        uint16_t buf_avail = (uint16_t)(TCP_SEND_BUF_SIZE - conn->send_count);
        if (chunk > buf_avail) chunk = buf_avail;
        if (chunk == 0u) break;

        ring_write_partial(conn->send_buf, TCP_SEND_BUF_SIZE,
                           &conn->send_tail, ptr + sent, chunk);
        conn->send_count = (uint16_t)(conn->send_count + chunk);

        bool ok = tcp_send_segment(conn->local_ip, conn->remote_ip,
                                   conn->local_port, conn->remote_port,
                                   conn->snd_nxt, conn->rcv_nxt,
                                   TCP_FLAG_ACK | TCP_FLAG_PSH,
                                   conn->rcv_wnd,
                                   ptr + sent, chunk);
        if (!ok) break;

        conn->snd_nxt += chunk;
        conn->last_send_tick = timer_ticks();
        conn->retransmit_count = 0;
        sent += chunk;
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return (int32_t)sent;
}

int32_t tcp_recv(int32_t conn_id, void *buf, uint16_t buf_len)
{
    if (conn_id < 0 || conn_id >= (int32_t)TCP_MAX_CONNECTIONS) return -1;
    if (buf == NULL && buf_len != 0u) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    tcp_connection_t *conn = &g_tcp_connections[conn_id];
    if (conn->in_use == 0u) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    
    if (conn->state != TCP_STATE_ESTABLISHED &&
        conn->state != TCP_STATE_CLOSE_WAIT &&
        conn->state != TCP_STATE_FIN_WAIT_1 &&
        conn->state != TCP_STATE_FIN_WAIT_2) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    uint16_t n = recv_buf_read(conn, (uint8_t *)buf, buf_len);

    
    conn->rcv_wnd = recv_buf_free(conn);

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return (int32_t)n;
}

int32_t tcp_close(int32_t conn_id)
{
    if (conn_id < 0 || conn_id >= (int32_t)TCP_MAX_CONNECTIONS) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    tcp_connection_t *conn = &g_tcp_connections[conn_id];
    if (conn->in_use == 0u) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    switch (conn->state) {
    case TCP_STATE_LISTEN:
    case TCP_STATE_SYN_SENT:
        conn->state = TCP_STATE_CLOSED;
        conn->in_use = 0u;
        break;

    case TCP_STATE_SYN_RECEIVED:
    case TCP_STATE_ESTABLISHED:
        
        tcp_send_segment(conn->local_ip, conn->remote_ip,
                         conn->local_port, conn->remote_port,
                         conn->snd_nxt, conn->rcv_nxt,
                         TCP_FLAG_FIN | TCP_FLAG_ACK,
                         conn->rcv_wnd, NULL, 0);
        conn->snd_nxt++;
        conn->state = TCP_STATE_FIN_WAIT_1;
        conn->last_send_tick = timer_ticks();
        break;

    case TCP_STATE_CLOSE_WAIT:
        
        tcp_send_segment(conn->local_ip, conn->remote_ip,
                         conn->local_port, conn->remote_port,
                         conn->snd_nxt, conn->rcv_nxt,
                         TCP_FLAG_FIN | TCP_FLAG_ACK,
                         conn->rcv_wnd, NULL, 0);
        conn->snd_nxt++;
        conn->state = TCP_STATE_LAST_ACK;
        conn->last_send_tick = timer_ticks();
        break;

    default:
        break;
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return 0;
}

tcp_state_t tcp_get_state(int32_t conn_id)
{
    if (conn_id < 0 || conn_id >= (int32_t)TCP_MAX_CONNECTIONS) return TCP_STATE_CLOSED;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    tcp_state_t s = TCP_STATE_CLOSED;
    if (g_tcp_connections[conn_id].in_use != 0u) {
        s = g_tcp_connections[conn_id].state;
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return s;
}

int tcp_get_connection_info(int32_t conn_id, tcp_connection_info_t *info_out)
{
    if (conn_id < 0 || conn_id >= (int32_t)TCP_MAX_CONNECTIONS ||
        info_out == NULL) {
        return -1;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);
    const tcp_connection_t *connection = &g_tcp_connections[conn_id];
    if (connection->in_use == 0u) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }
    info_out->local_ip = connection->local_ip;
    info_out->remote_ip = connection->remote_ip;
    info_out->local_port = connection->local_port;
    info_out->remote_port = connection->remote_port;
    info_out->state = connection->state;
    info_out->receive_available = connection->recv_count;
    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return 0;
}

uint32_t tcp_poll(int32_t conn_id, uint32_t events)
{
    if (conn_id < 0 || conn_id >= (int32_t)TCP_MAX_CONNECTIONS) return 0x0008u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);
    tcp_connection_t *connection = &g_tcp_connections[conn_id];
    uint32_t ready = 0u;
    if (connection->in_use == 0u || connection->state == TCP_STATE_CLOSED) {
        ready = 0x0010u;
    } else {
        if ((events & 0x0001u) != 0u &&
            (connection->recv_count != 0u ||
             connection->state == TCP_STATE_CLOSE_WAIT))
            ready |= 0x0001u;
        if ((events & 0x0004u) != 0u &&
            connection->state == TCP_STATE_ESTABLISHED &&
            connection->send_count < TCP_SEND_BUF_SIZE)
            ready |= 0x0004u;
    }
    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return ready;
}

void tcp_process_timer(void)
{
    uint32_t hz = timer_hz();
    if (hz == 0u) hz = 60u;

    uint64_t now = timer_ticks();
    uint64_t retransmit_ticks = ((uint64_t)TCP_RETRANSMIT_MS * (uint64_t)hz + 999u) / 1000u;
    uint64_t time_wait_ticks  = ((uint64_t)TCP_TIME_WAIT_MS * (uint64_t)hz + 999u) / 1000u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);

    for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
        tcp_connection_t *c = &g_tcp_connections[i];
        if (c->in_use == 0u) continue;

        
        if (c->state == TCP_STATE_TIME_WAIT) {
            if ((now - c->time_wait_start) >= time_wait_ticks) {
                c->state = TCP_STATE_CLOSED;
                c->in_use = 0u;
            }
            continue;
        }

        
        if (c->state == TCP_STATE_SYN_SENT || c->state == TCP_STATE_SYN_RECEIVED) {
            if ((now - c->last_send_tick) >= retransmit_ticks) {
                c->retransmit_count++;
                if (c->retransmit_count >= TCP_MAX_RETRANSMITS) {
                    c->state = TCP_STATE_CLOSED;
                    c->in_use = 0u;
                    continue;
                }
                c->last_send_tick = now;
                if (c->state == TCP_STATE_SYN_SENT) {
                    tcp_send_segment(c->local_ip, c->remote_ip,
                                     c->local_port, c->remote_port,
                                     c->iss, 0u,
                                     TCP_FLAG_SYN, c->rcv_wnd, NULL, 0);
                } else {
                    tcp_send_segment(c->local_ip, c->remote_ip,
                                     c->local_port, c->remote_port,
                                     c->iss, c->rcv_nxt,
                                     TCP_FLAG_SYN | TCP_FLAG_ACK,
                                     c->rcv_wnd, NULL, 0);
                }
            }
        }

        
        if (c->state == TCP_STATE_FIN_WAIT_1 || c->state == TCP_STATE_LAST_ACK) {
            if ((now - c->last_send_tick) >= retransmit_ticks) {
                c->retransmit_count++;
                if (c->retransmit_count >= TCP_MAX_RETRANSMITS) {
                    c->state = TCP_STATE_CLOSED;
                    c->in_use = 0u;
                    continue;
                }
                c->last_send_tick = now;
                tcp_send_segment(c->local_ip, c->remote_ip,
                                 c->local_port, c->remote_port,
                                 c->snd_nxt - 1u, c->rcv_nxt,
                                 TCP_FLAG_FIN | TCP_FLAG_ACK,
                                 c->rcv_wnd, NULL, 0);
            }
        }

        if (c->state == TCP_STATE_ESTABLISHED && c->send_count > 0 &&
            c->snd_una < c->snd_nxt) {
            if ((now - c->last_send_tick) >= retransmit_ticks) {
                c->retransmit_count++;
                if (c->retransmit_count >= TCP_MAX_RETRANSMITS) {
                    c->state = TCP_STATE_CLOSED;
                    c->in_use = 0u;
                    continue;
                }
                uint32_t unacked = c->snd_nxt - c->snd_una;
                if (unacked > TCP_MAX_SEGMENT_DATA) unacked = TCP_MAX_SEGMENT_DATA;
                if (unacked > c->send_count) unacked = c->send_count;

                uint8_t retx_buf[TCP_MAX_SEGMENT_DATA];
                uint16_t first = (uint16_t)(TCP_SEND_BUF_SIZE - c->send_head);
                if (first > (uint16_t)unacked) first = (uint16_t)unacked;
                memcpy(retx_buf, c->send_buf + c->send_head, first);
                if ((uint16_t)unacked > first) {
                    memcpy(retx_buf + first, c->send_buf,
                           (uint16_t)unacked - first);
                }

                c->last_send_tick = now;
                tcp_send_segment(c->local_ip, c->remote_ip,
                                 c->local_port, c->remote_port,
                                 c->snd_una, c->rcv_nxt,
                                 TCP_FLAG_ACK | TCP_FLAG_PSH,
                                 c->rcv_wnd, retx_buf, (uint16_t)unacked);
            }
        }
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
}
