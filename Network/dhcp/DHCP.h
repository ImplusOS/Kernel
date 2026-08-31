#pragma once

#include <stdint.h>
#include <stdbool.h>

void dhcp_init(void);
bool dhcp_discover(void);
uint32_t dhcp_get_assigned_ip(void);
uint32_t dhcp_get_gateway(void);
uint32_t dhcp_get_subnet_mask(void);
uint32_t dhcp_get_dns_server(void);
