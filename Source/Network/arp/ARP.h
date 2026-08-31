#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ARP_TABLE_MAX_ENTRIES 64u

void arp_init(uint32_t local_ipv4_addr,
              uint32_t subnet_mask,
              uint32_t gateway_ipv4_addr);

void arp_set_local_ipv4(uint32_t local_ipv4_addr);

bool arp_lookup(uint32_t ipv4_addr, uint8_t mac_out[6]);
bool arp_resolve(uint32_t ipv4_addr, uint8_t mac_out[6], uint32_t timeout_ms);

bool arp_send_request(uint32_t target_ipv4_addr);
bool arp_send_reply(const uint8_t target_mac[6], uint32_t target_ipv4_addr);

void arp_process_timer(void);
