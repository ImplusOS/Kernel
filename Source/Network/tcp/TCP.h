#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TCP_PROTOCOL_NUMBER  6u
/* Chromium alone keeps dozens of connections open (a YouTube page talks to
 * a dozen hosts), and each closed one sits in TIME_WAIT for a while; 32 ran
 * out, and connect() then failed with no obvious cause. */
#define TCP_MAX_CONNECTIONS  128u
#define TCP_RECV_BUF_SIZE    32768u
#define TCP_SEND_BUF_SIZE    16384u
#define TCP_MAX_SEGMENT_DATA 1460u
#define TCP_DEFAULT_WINDOW   32768u
#define TCP_RETRANSMIT_MS    1000u
#define TCP_MAX_RETRANSMITS  5u
#define TCP_TIME_WAIT_MS     2000u
/* How long a connection whose socket is already closed may sit in
 * FIN_WAIT_2 waiting for the peer's FIN (Linux's tcp_fin_timeout). A peer
 * that never sends one used to keep the slot forever; enough of them and
 * TCP_MAX_CONNECTIONS ran out. */
#define TCP_FIN_WAIT2_MS     60000u
/* Duplicate ACKs that trigger a fast retransmit (RFC 5681). */
#define TCP_DUPACK_THRESHOLD 3u


typedef enum {
    TCP_STATE_CLOSED = 0,
    TCP_STATE_LISTEN,
    TCP_STATE_SYN_SENT,
    TCP_STATE_SYN_RECEIVED,
    TCP_STATE_ESTABLISHED,
    TCP_STATE_FIN_WAIT_1,
    TCP_STATE_FIN_WAIT_2,
    TCP_STATE_CLOSE_WAIT,
    TCP_STATE_CLOSING,
    TCP_STATE_LAST_ACK,
    TCP_STATE_TIME_WAIT,
} tcp_state_t;


#define TCP_FLAG_FIN  0x01u
#define TCP_FLAG_SYN  0x02u
#define TCP_FLAG_RST  0x04u
#define TCP_FLAG_PSH  0x08u
#define TCP_FLAG_ACK  0x10u
#define TCP_FLAG_URG  0x20u

typedef struct {
    uint32_t local_ip;
    uint32_t remote_ip;
    uint16_t local_port;
    uint16_t remote_port;
    tcp_state_t state;

    
    uint32_t snd_una;    
    uint32_t snd_nxt;    
    uint32_t snd_wnd;    
    uint32_t rcv_nxt;    
    uint32_t rcv_wnd;    
    uint32_t iss;        
    uint32_t irs;        

    
    uint8_t  recv_buf[TCP_RECV_BUF_SIZE];
    uint16_t recv_head;
    uint16_t recv_tail;
    uint16_t recv_count;

    
    uint8_t  send_buf[TCP_SEND_BUF_SIZE];
    uint16_t send_head;
    uint16_t send_tail;
    uint16_t send_count;

    
    uint64_t last_send_tick;
    uint32_t retransmit_count;

    
    uint64_t time_wait_start;

    
    uint8_t  in_use;
    uint8_t  accept_pending;
    /* Set while a socket holds this connection id (from tcp_connect/listen/
     * accept until tcp_close). A connection that dies under an open socket
     * (RST, retransmit give-up) keeps its slot in TCP_STATE_CLOSED until the
     * socket closes; freeing it at once let tcp_connect hand the same id to a
     * new socket, and two sockets then shared one byte stream. */
    uint8_t  user_ref;
    /* Why the connection died, as a positive errno (0 while it is alive):
     * ECONNREFUSED (RST answering our SYN), ECONNRESET (RST later) or
     * ETIMEDOUT (retransmissions exhausted). recv/send/SO_ERROR/poll report
     * it; without it every dead connection read back as a generic EIO, which
     * Chromium maps to ERR_FAILED. */
    uint8_t  error;
    /* Largest payload the peer takes per segment: the MSS option of its
     * SYN, capped at TCP_MAX_SEGMENT_DATA (RFC 879's 536 without one). */
    uint16_t snd_mss;
    /* Duplicate ACKs seen for snd_una (fast retransmit at
     * TCP_DUPACK_THRESHOLD). */
    uint8_t  dup_acks;
    /* Set once a segment had to be retransmitted, until everything sent is
     * acknowledged: each ACK that moves snd_una but not to snd_nxt (a
     * "partial ACK") then resends the next segment at once (RFC 6582)
     * instead of leaving it to the next retransmission timeout. */
    uint8_t  recovering;
    int32_t  parent_conn_id;
    uint16_t listen_backlog;
} tcp_connection_t;

typedef struct {
    uint32_t local_ip;
    uint32_t remote_ip;
    uint16_t local_port;
    uint16_t remote_port;
    tcp_state_t state;
    uint16_t receive_available;
} tcp_connection_info_t;


typedef void (*tcp_accept_callback_t)(int32_t conn_id);

void tcp_init(void);


int32_t tcp_connect(uint32_t remote_ip, uint16_t remote_port, uint16_t local_port);

/* The positive errno a dead connection failed with (tcp_connection_t.error),
 * or 0. */
int32_t tcp_connection_error(int32_t conn_id);
int tcp_local_port_in_use(uint16_t local_port);


int32_t tcp_listen(uint16_t port);
int tcp_set_listen_backlog(int32_t conn_id, uint16_t backlog);


int32_t tcp_accept(int32_t listen_conn_id);


int32_t tcp_send(int32_t conn_id, const void *data, uint16_t len);


int32_t tcp_recv(int32_t conn_id, void *buf, uint16_t buf_len);

/* recv(MSG_PEEK): copies what tcp_recv() would return without taking it out
 * of the receive buffer. */
int32_t tcp_peek(int32_t conn_id, void *buf, uint16_t buf_len);


int32_t tcp_close(int32_t conn_id);

/* shutdown(SHUT_WR): sends our FIN like tcp_close(), but the socket still
 * holds the connection (it can go on reading, and its slot is not recycled
 * under it). */
int32_t tcp_shutdown_write(int32_t conn_id);


tcp_state_t tcp_get_state(int32_t conn_id);
int tcp_get_connection_info(int32_t conn_id, tcp_connection_info_t *info_out);
uint32_t tcp_poll(int32_t conn_id, uint32_t events);


void tcp_process_timer(void);
