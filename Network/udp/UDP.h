#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*udp_rx_handler_t)(uint32_t src_ipv4_addr,
                                 uint16_t src_port,
                                 uint32_t dst_ipv4_addr,
                                 uint16_t dst_port,
                                 const uint8_t *payload,
                                 uint16_t payload_len);

void udp_init(void);

bool udp_bind(uint16_t port, udp_rx_handler_t handler);
void udp_unbind(uint16_t port);

bool udp_send(uint32_t dst_ipv4_addr,
              uint16_t src_port,
              uint16_t dst_port,
              const void *payload,
              uint16_t payload_len);

bool udp_syscall_send(uint32_t dst_ipv4_addr,
                      uint16_t src_port,
                      uint16_t dst_port,
                      const void *payload,
                      uint16_t payload_len);

#define UDP_USER_HEADER_BYTES 8u

int32_t udp_user_bind(int32_t owner_pid, uint16_t port);
int32_t udp_user_unbind(int32_t owner_pid, uint16_t port);
int32_t udp_user_recv(int32_t owner_pid,
                      uint16_t port,
                      uint8_t *out_buf,
                      uint32_t out_buf_len);
/* Number of queued datagrams waiting for `port` (owned by `owner_pid`),
 * or -1 if not bound / not owned. Used for FIONREAD/poll. */
int32_t udp_user_available(int32_t owner_pid, uint16_t port);
void    udp_user_release_all(int32_t owner_pid);
