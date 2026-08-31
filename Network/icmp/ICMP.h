#pragma once

#include <stdint.h>
#include <stdbool.h>

#define ICMP_PROTOCOL_NUMBER 1u

#define ICMP_TYPE_ECHO_REPLY    0u
#define ICMP_TYPE_ECHO_REQUEST  8u
#define ICMP_TYPE_DEST_UNREACH  3u

void icmp_init(void);
bool icmp_send_echo_request(uint32_t dst_ip, uint16_t id, uint16_t seq,
                            const void *data, uint16_t data_len);
