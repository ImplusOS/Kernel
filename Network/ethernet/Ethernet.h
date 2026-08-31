#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ETHERNET_ADDR_LEN 6u
#define ETHERNET_HEADER_BYTES 14u

#define ETHERNET_TYPE_IPV4 0x0800u
#define ETHERNET_TYPE_ARP  0x0806u

typedef void (*ethernet_type_handler_t)(const uint8_t src_mac[ETHERNET_ADDR_LEN],
                                        const uint8_t dst_mac[ETHERNET_ADDR_LEN],
                                        const uint8_t *payload,
                                        uint16_t payload_len);

bool ethernet_init(void);
bool ethernet_is_ready(void);
void ethernet_poll(void);

void ethernet_get_local_mac(uint8_t mac_out[ETHERNET_ADDR_LEN]);

bool ethernet_send(uint16_t ether_type,
                   const uint8_t dst_mac[ETHERNET_ADDR_LEN],
                   const void *payload,
                   uint16_t payload_len);

bool ethernet_register_handler(uint16_t ether_type, ethernet_type_handler_t handler);
