#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*ipv4_protocol_handler_t)(uint32_t src_ipv4_addr,
                                        uint32_t dst_ipv4_addr,
                                        const uint8_t *payload,
                                        uint16_t payload_len);

void ipv4_init(uint32_t local_ipv4_addr,
               uint32_t subnet_mask,
               uint32_t gateway_ipv4_addr);

/* Change the local address, mask and gateway without touching the protocol
 * handlers registered since ipv4_init() (use this for DHCP). */
void ipv4_set_address(uint32_t local_ipv4_addr,
                      uint32_t subnet_mask,
                      uint32_t gateway_ipv4_addr);

uint32_t ipv4_local_address(void);

bool ipv4_register_protocol(uint8_t protocol, ipv4_protocol_handler_t handler);

bool ipv4_send(uint32_t dst_ipv4_addr,
               uint8_t protocol,
               const void *payload,
               uint16_t payload_len);

/* ipv4_send() with the source address chosen by the caller: IPV4_SRC_LOCAL
 * for the configured address, or any value (0.0.0.0 for DHCP before a lease)
 * when dst is the limited broadcast 255.255.255.255. */
#define IPV4_SRC_LOCAL 0x100000000ull
bool ipv4_send_from(uint64_t src_ipv4_addr,
                    uint32_t dst_ipv4_addr,
                    uint8_t protocol,
                    const void *payload,
                    uint16_t payload_len);

void ipv4_process_timer(void);
