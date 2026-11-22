#include "TCP.h"
#include "Core/syscall/Poll_Wait.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Network/ipv4.h"
#include "Network/network_utils.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"

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

#define TCP_OPT_MSS_BYTES 4u
#define TCP_ERR_ECONNRESET   104u
#define TCP_ERR_ETIMEDOUT    110u
#define TCP_ERR_ECONNREFUSED 111u
/* RFC 879: the MSS to assume when the peer's SYN carries none. */
#define TCP_DEFAULT_PEER_MSS 536u
/* Smallest MSS taken from a peer (Linux's TCP_MIN_MSS). */
#define TCP_MIN_PEER_MSS     88u

static bool tcp_send_segment_opts(uint32_t src_ip, uint32_t dst_ip,
                                  uint16_t src_port, uint16_t dst_port,
                                  uint32_t seq, uint32_t ack,
                                  uint8_t flags, uint16_t window,
                                  const uint8_t *opts, uint8_t opts_len,
                                  const void *data, uint16_t data_len)
{
    uint16_t header_len = (uint16_t)(TCP_HEADER_BYTES_MIN + opts_len);
    uint16_t total_len = (uint16_t)(header_len + data_len);
    uint8_t segment[TCP_HEADER_BYTES_MIN + TCP_OPT_MSS_BYTES + TCP_MAX_PAYLOAD];

    if ((opts_len & 3u) != 0u) return false;

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

    if (opts_len > 0u && opts != NULL) {
        memcpy(segment + TCP_HEADER_BYTES_MIN, opts, opts_len);
    }
    if (data_len > 0u && data != NULL) {
        memcpy(segment + header_len, data, data_len);
    }

    if (src_ip == 0u) src_ip = ipv4_local_address();
    uint16_t cksum = tcp_checksum(src_ip, dst_ip, segment, total_len);
    if (cksum == 0u) cksum = 0xFFFFu;
    tcp_write_be16(segment + 16, cksum);

    return ipv4_send(dst_ip, TCP_PROTOCOL_NUMBER, segment, total_len);
}

static bool tcp_send_segment(uint32_t src_ip, uint32_t dst_ip,
                             uint16_t src_port, uint16_t dst_port,
                             uint32_t seq, uint32_t ack,
                             uint8_t flags, uint16_t window,
                             const void *data, uint16_t data_len)
{
    return tcp_send_segment_opts(src_ip, dst_ip, src_port, dst_port, seq, ack,
                                 flags, window, NULL, 0u, data, data_len);
}

/* SYN and SYN-ACK carry our MSS. Without it the peer must assume 536 bytes
 * (RFC 879) and sends everything in small segments. */
static bool tcp_send_syn(uint32_t src_ip, uint32_t dst_ip,
                         uint16_t src_port, uint16_t dst_port,
                         uint32_t seq, uint32_t ack,
                         uint8_t flags, uint16_t window)
{
    const uint8_t mss[TCP_OPT_MSS_BYTES] = {
        2u, 4u, (uint8_t)(TCP_MAX_SEGMENT_DATA >> 8),
        (uint8_t)(TCP_MAX_SEGMENT_DATA & 0xFFu),
    };
    return tcp_send_segment_opts(src_ip, dst_ip, src_port, dst_port, seq, ack,
                                 flags, window, mss, TCP_OPT_MSS_BYTES,
                                 NULL, 0u);
}

/* The MSS a SYN asks us to respect (its option 2), capped at what we send
 * at most. Routers clamp it to the path: on an uplink with a smaller MTU
 * (PPPoE, IPv4-over-IPv6) a 1460-byte segment never arrives, so a request
 * larger than the clamped MSS -- a TLS ClientHello with a post-quantum key
 * share is ~1.9 KB -- was retransmitted unchanged until the connection gave
 * up. QEMU's SLIRP re-segments on the host, which hid this. */
static uint16_t tcp_peer_mss(const uint8_t *tcp, uint16_t hdr_bytes)
{
    uint16_t mss = TCP_DEFAULT_PEER_MSS;
    uint16_t o = TCP_HEADER_BYTES_MIN;
    while (o < hdr_bytes) {
        uint8_t kind = tcp[o];
        if (kind == 0u) break;
        if (kind == 1u) { ++o; continue; }
        if ((uint16_t)(o + 1u) >= hdr_bytes) break;
        uint8_t len = tcp[o + 1u];
        if (len < 2u || (uint16_t)(o + len) > hdr_bytes) break;
        if (kind == 2u && len == 4u) {
            mss = (uint16_t)(((uint16_t)tcp[o + 2u] << 8) | tcp[o + 3u]);
        }
        o = (uint16_t)(o + len);
    }
    if (mss > TCP_MAX_SEGMENT_DATA) mss = TCP_MAX_SEGMENT_DATA;
    if (mss < TCP_MIN_PEER_MSS) mss = TCP_MIN_PEER_MSS;
    return mss;
}

static uint16_t tcp_conn_mss(const tcp_connection_t *c)
{
    return c->snd_mss != 0u ? c->snd_mss : (uint16_t)TCP_MAX_SEGMENT_DATA;
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
        if (c->state == TCP_STATE_CLOSED) continue;
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

/* The connection is finished. Its slot is recycled only once no socket
 * refers to it any more (see tcp_connection_t.user_ref). */
static void tcp_release_locked(tcp_connection_t *c)
{
    c->state = TCP_STATE_CLOSED;
    if (c->user_ref == 0u) {
        c->in_use = 0u;
    }
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

static int tcp_in_fin_state(const tcp_connection_t *c)
{
    return c->state == TCP_STATE_FIN_WAIT_1 ||
           c->state == TCP_STATE_LAST_ACK ||
           c->state == TCP_STATE_CLOSING;
}

/* Resend the oldest unacknowledged segment: up to one MSS of data from
 * snd_una (send_buf[send_head] is the byte at snd_una), or -- with no data
 * outstanding in a state that has sent its FIN -- the FIN. */
static void tcp_retransmit_head_locked(tcp_connection_t *c, uint64_t now)
{
    c->last_send_tick = now;
    if (c->send_count == 0u) {
        if (tcp_in_fin_state(c)) {
            tcp_send_segment(c->local_ip, c->remote_ip,
                             c->local_port, c->remote_port,
                             c->snd_nxt - 1u, c->rcv_nxt,
                             TCP_FLAG_FIN | TCP_FLAG_ACK,
                             (uint16_t)c->rcv_wnd, NULL, 0);
        }
        return;
    }

    uint32_t len = c->snd_nxt - c->snd_una;
    if (len > c->send_count) len = c->send_count;
    if (len > tcp_conn_mss(c)) len = tcp_conn_mss(c);
    if (len == 0u) return;

    uint8_t retx_buf[TCP_MAX_SEGMENT_DATA];
    uint16_t first = (uint16_t)(TCP_SEND_BUF_SIZE - c->send_head);
    if (first > (uint16_t)len) first = (uint16_t)len;
    memcpy(retx_buf, c->send_buf + c->send_head, first);
    if ((uint16_t)len > first) {
        memcpy(retx_buf + first, c->send_buf, (uint16_t)len - first);
    }
    tcp_send_segment(c->local_ip, c->remote_ip,
                     c->local_port, c->remote_port,
                     c->snd_una, c->rcv_nxt,
                     TCP_FLAG_ACK | TCP_FLAG_PSH,
                     (uint16_t)c->rcv_wnd, retx_buf, (uint16_t)len);
}

/* The peer acknowledged everything below ack_num: advance snd_una and drop
 * the acknowledged bytes from the retransmission buffer. Nothing used to
 * release them -- send_count only ever grew -- so every connection could
 * send TCP_SEND_BUF_SIZE bytes in its whole life and then no more: an
 * HTTP/2 connection carrying request after request (and the DevTools
 * socket) went silent after its first 16 KiB of requests, and every
 * response behind them waited forever. */
static void tcp_ack_advance(tcp_connection_t *conn, uint32_t ack_num)
{
    int32_t ahead = (int32_t)(ack_num - conn->snd_una);
    int32_t beyond = (int32_t)(ack_num - conn->snd_nxt);
    if (ahead <= 0 || beyond > 0) {
        return;
    }
    uint32_t acked = (uint32_t)ahead;
    uint32_t data = acked < conn->send_count ? acked : conn->send_count;
    conn->send_head = (uint16_t)((conn->send_head + data) % TCP_SEND_BUF_SIZE);
    conn->send_count = (uint16_t)(conn->send_count - data);
    conn->snd_una = ack_num;
    conn->retransmit_count = 0;
    conn->dup_acks = 0u;
    /* New data acknowledged: the retransmission timer restarts (RFC 6298
     * 5.3) -- a transfer that is slow but moving is not a lost segment. */
    uint64_t now = timer_ticks();
    conn->last_send_tick = now;
    if (conn->snd_una == conn->snd_nxt) {
        conn->recovering = 0u;
    } else if (conn->recovering != 0u) {
        /* Partial ACK after a retransmission: the next segment was lost
         * too (we keep no out-of-order data, and neither may the peer).
         * Send it now rather than one retransmission timeout per segment. */
        tcp_retransmit_head_locked(conn, now);
    }
}

/* An ACK in a state that can have data in flight. Three duplicates of the
 * same ACK while data is outstanding mean the segment at snd_una was lost
 * and the ones after it arrived: retransmit it at once (RFC 5681 fast
 * retransmit) instead of waiting for the retransmission timer. */
static void tcp_on_ack_locked(tcp_connection_t *conn, uint32_t ack_num,
                              uint16_t window, uint8_t flags,
                              uint16_t seg_data_len)
{
    if (ack_num == conn->snd_una && seg_data_len == 0u &&
        (flags & (TCP_FLAG_SYN | TCP_FLAG_FIN)) == 0u &&
        conn->send_count > 0u && conn->snd_una != conn->snd_nxt &&
        window == conn->snd_wnd) {
        if (++conn->dup_acks == TCP_DUPACK_THRESHOLD) {
            conn->recovering = 1u;
            tcp_retransmit_head_locked(conn, timer_ticks());
        }
    } else {
        tcp_ack_advance(conn, ack_num);
    }
    conn->snd_wnd = window;
}

/* Take a data segment into the receive buffer: the part of it that starts
 * at rcv_nxt. A retransmission that overlaps bytes we already have (senders
 * merge segments when they retransmit) is trimmed to its new tail instead
 * of being dropped whole. Returns 1 when all of the segment's data is in, so
 * a FIN on it is in sequence. */
static int tcp_take_data_locked(tcp_connection_t *conn, uint32_t seq_num,
                                const uint8_t *data, uint16_t len)
{
    int32_t skip = (int32_t)(conn->rcv_nxt - seq_num);
    if (skip < 0 || (uint32_t)skip > (uint32_t)len) {
        return 0; /* past a gap, or entirely old */
    }
    uint16_t fresh = (uint16_t)(len - (uint16_t)skip);
    uint16_t space = recv_buf_free(conn);
    uint16_t take = (fresh < space) ? fresh : space;
    if (take > 0u) {
        recv_buf_write(conn, data + skip, take);
        conn->rcv_nxt += take;
    }
    return take == fresh;
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
                child->snd_mss     = tcp_peer_mss(payload, hdr_bytes);

                tcp_send_syn(dst_ip, src_ip, dst_port, src_port,
                             child->iss, child->rcv_nxt,
                             TCP_FLAG_SYN | TCP_FLAG_ACK, child->rcv_wnd);
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
        conn->error = (uint8_t)(conn->state == TCP_STATE_SYN_SENT ?
                                TCP_ERR_ECONNREFUSED : TCP_ERR_ECONNRESET);
        tcp_release_locked(conn);
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
                conn->snd_mss = tcp_peer_mss(payload, hdr_bytes);
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
            tcp_on_ack_locked(conn, ack_num, window, flags, seg_data_len);
        }

        {
            /* Data. Only the in-sequence segment is taken (there is no
             * reassembly queue), but every data segment -- early, late or
             * duplicate -- is answered with an ACK of what we do have: that
             * duplicate ACK is what makes the sender retransmit the missing
             * piece at once. Dropping them silently left the sender waiting
             * for its retransmission timer, which backs off to many seconds;
             * a large response (YouTube's watch page) then sat half-received
             * with the tab spinning. */
            int fin_in_sequence = 0;
            if (seg_data_len > 0u) {
                fin_in_sequence = tcp_take_data_locked(conn, seq_num,
                                                       seg_data, seg_data_len);
                conn->rcv_wnd = recv_buf_free(conn);
                if (!(fin_in_sequence && (flags & TCP_FLAG_FIN))) {
                    tcp_send_segment(conn->local_ip, conn->remote_ip,
                                     conn->local_port, conn->remote_port,
                                     conn->snd_nxt, conn->rcv_nxt,
                                     TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
                }
            } else {
                fin_in_sequence = (seq_num == conn->rcv_nxt);
            }

            /* A FIN counts only in sequence and once all the data before it
             * is in: taking one that arrived early (or whose segment did not
             * fit) jumped rcv_nxt past bytes never received -- a response
             * silently cut short. */
            if ((flags & TCP_FLAG_FIN) && fin_in_sequence) {
                conn->rcv_nxt += 1u;
                conn->state = TCP_STATE_CLOSE_WAIT;
                tcp_send_segment(conn->local_ip, conn->remote_ip,
                                 conn->local_port, conn->remote_port,
                                 conn->snd_nxt, conn->rcv_nxt,
                                 TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
            }
        }
        break;

    case TCP_STATE_FIN_WAIT_1:
        if (flags & TCP_FLAG_ACK) {
            tcp_ack_advance(conn, ack_num);
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

    case TCP_STATE_FIN_WAIT_2: {
        int fin_ok = (seq_num == conn->rcv_nxt);
        if (seg_data_len > 0u) {
            fin_ok = tcp_take_data_locked(conn, seq_num, seg_data, seg_data_len);
            conn->rcv_wnd = recv_buf_free(conn);
            tcp_send_segment(conn->local_ip, conn->remote_ip,
                             conn->local_port, conn->remote_port,
                             conn->snd_nxt, conn->rcv_nxt,
                             TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
        }
        if ((flags & TCP_FLAG_FIN) && fin_ok) {
            conn->rcv_nxt += 1u;
            conn->state = TCP_STATE_TIME_WAIT;
            conn->time_wait_start = timer_ticks();
            tcp_send_segment(conn->local_ip, conn->remote_ip,
                             conn->local_port, conn->remote_port,
                             conn->snd_nxt, conn->rcv_nxt,
                             TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
        }
        break;
    }

    case TCP_STATE_CLOSING:
        if ((flags & TCP_FLAG_ACK) && ack_num == conn->snd_nxt) {
            conn->state = TCP_STATE_TIME_WAIT;
            conn->time_wait_start = timer_ticks();
        }
        break;

    case TCP_STATE_LAST_ACK:
        if ((flags & TCP_FLAG_ACK) && ack_num == conn->snd_nxt) {
            tcp_release_locked(conn);
        }
        break;

    case TCP_STATE_CLOSE_WAIT:
        if (flags & TCP_FLAG_ACK) {
            tcp_on_ack_locked(conn, ack_num, window, flags, seg_data_len);
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

    /* A segment landed: whatever is parked in poll()/select() on this
     * socket should see it now rather than at the end of its slice. */
    poll_wait_notify();
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
    conn->user_ref    = 1u;
    
    tcp_send_syn(conn->local_ip, conn->remote_ip,
                 conn->local_port, conn->remote_port,
                 conn->iss, 0u, TCP_FLAG_SYN, conn->rcv_wnd);

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
    conn->user_ref    = 1u;
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
            c->user_ref = 1u;
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
    if (conn->in_use == 0u || (conn->state != TCP_STATE_ESTABLISHED &&
                               conn->state != TCP_STATE_CLOSE_WAIT)) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    const uint8_t *ptr = (const uint8_t *)data;
    uint16_t sent = 0u;

    while (sent < len) {
        uint16_t chunk = (uint16_t)(len - sent);
        if (chunk > tcp_conn_mss(conn)) chunk = tcp_conn_mss(conn);

        uint16_t buf_avail = (uint16_t)(TCP_SEND_BUF_SIZE - conn->send_count);
        if (chunk > buf_avail) chunk = buf_avail;
        if (chunk == 0u) break;

        bool ok = tcp_send_segment(conn->local_ip, conn->remote_ip,
                                   conn->local_port, conn->remote_port,
                                   conn->snd_nxt, conn->rcv_nxt,
                                   TCP_FLAG_ACK | TCP_FLAG_PSH,
                                   conn->rcv_wnd,
                                   ptr + sent, chunk);
        /* Only a transmitted chunk joins the retransmission buffer. It used
         * to be queued before the attempt and left there when the attempt
         * failed (no ARP entry, NIC ring full) while snd_nxt stayed put, so
         * the caller's retry of the same bytes queued them a second time:
         * from then on send_buf[send_head] no longer held the byte at
         * snd_una, and every retransmission put stale bytes under the new
         * sequence numbers -- a corrupted stream the peer's TLS rejects. */
        if (!ok) break;

        ring_write_partial(conn->send_buf, TCP_SEND_BUF_SIZE,
                           &conn->send_tail, ptr + sent, chunk);
        conn->send_count = (uint16_t)(conn->send_count + chunk);
        conn->snd_nxt += chunk;
        conn->last_send_tick = timer_ticks();
        conn->retransmit_count = 0;
        sent += chunk;
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return (int32_t)sent;
}

static int32_t tcp_recv_common(int32_t conn_id, void *buf, uint16_t buf_len,
                               int peek)
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

    /* Bytes already received stay readable whatever happened to the
     * connection since, as on Linux: a reset (or the end of TIME_WAIT under
     * a socket that shut down its write side) used to discard them, and a
     * server that answers and then resets -- an error page for a request
     * whose body it did not read -- lost its answer. Only once the buffer is
     * empty does the state decide: end-of-file after the peer's FIN, an
     * error (tcp_connection_error() says which) otherwise. */
    if (conn->state != TCP_STATE_ESTABLISHED &&
        conn->state != TCP_STATE_CLOSE_WAIT &&
        conn->state != TCP_STATE_FIN_WAIT_1 &&
        conn->state != TCP_STATE_FIN_WAIT_2 &&
        conn->recv_count == 0u) {
        int eof = conn->state == TCP_STATE_TIME_WAIT ||
                  conn->state == TCP_STATE_CLOSING ||
                  conn->state == TCP_STATE_LAST_ACK ||
                  (conn->state == TCP_STATE_CLOSED && conn->error == 0u);
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return eof ? 0 : -1;
    }
    if (conn->state == TCP_STATE_LISTEN ||
        conn->state == TCP_STATE_SYN_SENT ||
        conn->state == TCP_STATE_SYN_RECEIVED) {
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return -1;
    }

    if (peek) {
        uint16_t n = buf_len < conn->recv_count ? buf_len : conn->recv_count;
        uint16_t index = conn->recv_tail; /* a copy: nothing is consumed */
        if (n > 0u) {
            (void)ring_read_partial(conn->recv_buf, TCP_RECV_BUF_SIZE,
                                    &index, (uint8_t *)buf, n);
        }
        spinlock_unlock(&g_tcp_lock);
        irq_restore(irq_flags);
        return (int32_t)n;
    }

    uint32_t old_wnd = conn->rcv_wnd;
    uint16_t n = recv_buf_read(conn, (uint8_t *)buf, buf_len);

    conn->rcv_wnd = recv_buf_free(conn);
    /* Window update. The sender stops once our advertised window closes and
     * only learns it reopened from an ACK -- which, with nothing more
     * arriving, nobody sent: the connection sat idle until the peer's
     * persist timer probed it (seconds, growing). Announce the reopened
     * window as soon as it is worth a full segment again (receiver-side
     * silly-window avoidance). */
    if (n > 0u && conn->state == TCP_STATE_ESTABLISHED &&
        old_wnd < (uint32_t)TCP_RECV_BUF_SIZE / 2u &&
        conn->rcv_wnd >= old_wnd + TCP_MAX_SEGMENT_DATA) {
        tcp_send_segment(conn->local_ip, conn->remote_ip,
                         conn->local_port, conn->remote_port,
                         conn->snd_nxt, conn->rcv_nxt,
                         TCP_FLAG_ACK, conn->rcv_wnd, NULL, 0);
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return (int32_t)n;
}

int32_t tcp_recv(int32_t conn_id, void *buf, uint16_t buf_len)
{
    return tcp_recv_common(conn_id, buf, buf_len, 0);
}

/* Chromium asks "is this connection still alive?" with recv(fd, &c, 1,
 * MSG_PEEK) (SocketPosix::IsConnected / IsConnectedAndIdle): before it hands
 * out a pooled or preconnected socket, and before it opens another HTTP/2
 * stream on a live session whose read loop is between reads. The flag used
 * to be ignored, so every such check consumed a byte of the TLS stream: the
 * record it belonged to failed to decrypt (bad_record_mac), and the session
 * failed every request it carried with ERR_SSL_PROTOCOL_ERROR -- hundreds
 * of them on a page like Discord's, which opens a few hundred streams at
 * once. */
int32_t tcp_peek(int32_t conn_id, void *buf, uint16_t buf_len)
{
    return tcp_recv_common(conn_id, buf, buf_len, 1);
}

/* Sends our FIN for a synchronized connection. release_ref: the socket is
 * closing (tcp_close) rather than only shutting down its write side. */
static int32_t tcp_finish_sending(int32_t conn_id, int release_ref)
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

    if (release_ref) {
        conn->user_ref = 0u;
    }
    switch (conn->state) {
    case TCP_STATE_LISTEN:
    case TCP_STATE_SYN_SENT:
    case TCP_STATE_CLOSED:
        if (!release_ref) {
            break; /* nothing sent yet / already dead: shutdown is a no-op */
        }
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

int32_t tcp_close(int32_t conn_id)
{
    return tcp_finish_sending(conn_id, 1);
}

/* shutdown(SHUT_WR) used to call tcp_close(), which also dropped the
 * socket's reference: once the connection finished, its slot was recycled
 * while the socket still named it, and the next connect() got the same id --
 * two sockets on one byte stream, and closing the first sent a FIN on the
 * second. */
int32_t tcp_shutdown_write(int32_t conn_id)
{
    return tcp_finish_sending(conn_id, 0);
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

int32_t tcp_connection_error(int32_t conn_id)
{
    if (conn_id < 0 || conn_id >= (int32_t)TCP_MAX_CONNECTIONS) return 0;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);
    const tcp_connection_t *c = &g_tcp_connections[conn_id];
    int32_t error = c->in_use != 0u ? (int32_t)c->error : 0;
    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
    return error;
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
        if (connection->in_use != 0u && connection->error != 0u) {
            ready |= 0x0008u; /* POLLERR: SO_ERROR has the reason */
        }
        if (connection->in_use != 0u && connection->recv_count != 0u) {
            ready |= 0x0001u & events; /* still readable (tcp_recv) */
        }
    } else if (connection->state == TCP_STATE_LISTEN) {
        /* A listening socket is readable when accept() has a connection to
         * hand out -- the same condition tcp_accept() looks for. Without
         * this an event-driven server (epoll/poll before accept: Chromium's
         * DevTools server, any net::TCPServerSocket) never accepted. */
        if ((events & 0x0001u) != 0u) {
            for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
                const tcp_connection_t *c = &g_tcp_connections[i];
                if (c->in_use != 0u && c->parent_conn_id == conn_id &&
                    c->accept_pending != 0u &&
                    c->state == TCP_STATE_ESTABLISHED) {
                    ready |= 0x0001u;
                    break;
                }
            }
        }
    } else {
        /* Readable: data, or end-of-file once the peer's FIN is in
         * (CLOSE_WAIT, and CLOSING/LAST_ACK/TIME_WAIT after it). */
        if ((events & 0x0001u) != 0u &&
            (connection->recv_count != 0u ||
             connection->state == TCP_STATE_CLOSE_WAIT ||
             connection->state == TCP_STATE_CLOSING ||
             connection->state == TCP_STATE_LAST_ACK ||
             connection->state == TCP_STATE_TIME_WAIT))
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
    uint64_t fin_wait2_ticks  = ((uint64_t)TCP_FIN_WAIT2_MS * (uint64_t)hz + 999u) / 1000u;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_tcp_lock);


    for (uint32_t i = 0; i < TCP_MAX_CONNECTIONS; ++i) {
        tcp_connection_t *c = &g_tcp_connections[i];
        if (c->in_use == 0u) continue;

        
        if (c->state == TCP_STATE_TIME_WAIT) {
            if ((now - c->time_wait_start) >= time_wait_ticks) {
                tcp_release_locked(c);
            }
            continue;
        }

        
        if (c->state == TCP_STATE_SYN_SENT || c->state == TCP_STATE_SYN_RECEIVED) {
            if ((now - c->last_send_tick) >= retransmit_ticks) {
                c->retransmit_count++;
                if (c->retransmit_count >= TCP_MAX_RETRANSMITS) {
                    c->error = (uint8_t)TCP_ERR_ETIMEDOUT;
                    tcp_release_locked(c);
                    continue;
                }
                c->last_send_tick = now;
                if (c->state == TCP_STATE_SYN_SENT) {
                    tcp_send_syn(c->local_ip, c->remote_ip,
                                 c->local_port, c->remote_port,
                                 c->iss, 0u, TCP_FLAG_SYN, c->rcv_wnd);
                } else {
                    tcp_send_syn(c->local_ip, c->remote_ip,
                                 c->local_port, c->remote_port,
                                 c->iss, c->rcv_nxt,
                                 TCP_FLAG_SYN | TCP_FLAG_ACK, c->rcv_wnd);
                }
            }
        }

        /* An orphaned connection (its socket is closed) whose peer never
         * sends the FIN that ends FIN_WAIT_2 is dropped after a while. */
        if (c->state == TCP_STATE_FIN_WAIT_2) {
            if (c->user_ref == 0u &&
                (now - c->last_send_tick) >= fin_wait2_ticks) {
                tcp_release_locked(c);
            }
            continue;
        }

        /* Unacknowledged data -- in any state that can still have some,
         * not only ESTABLISHED: data written just before close() (FIN_WAIT_1,
         * LAST_ACK) or after the peer's FIN (CLOSE_WAIT) was never resent,
         * and a FIN retransmitted ahead of lost data is out of sequence for
         * the peer -- and then our FIN. tcp_retransmit_head_locked() sends
         * whichever comes first. The timeout doubles per attempt (RFC 6298
         * 5.5): with a fixed 1 s, five lost retransmissions -- a 5 s stall
         * anywhere on the path -- killed the connection. */
        int data_pending = (c->send_count > 0u && c->snd_una != c->snd_nxt);
        int can_send = (c->state == TCP_STATE_ESTABLISHED ||
                        c->state == TCP_STATE_CLOSE_WAIT ||
                        tcp_in_fin_state(c));
        if (can_send && (data_pending || tcp_in_fin_state(c))) {
            uint32_t shift = c->retransmit_count < 4u ? c->retransmit_count : 4u;
            if ((now - c->last_send_tick) >= (retransmit_ticks << shift)) {
                c->retransmit_count++;
                if (c->retransmit_count >= TCP_MAX_RETRANSMITS) {
                    c->error = (uint8_t)TCP_ERR_ETIMEDOUT;
                    tcp_release_locked(c);
                    continue;
                }
                c->recovering = 1u;
                c->dup_acks = 0u;
                tcp_retransmit_head_locked(c, now);
            }
        }
    }

    spinlock_unlock(&g_tcp_lock);
    irq_restore(irq_flags);
}
