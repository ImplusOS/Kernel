#pragma once

#include <stdbool.h>
#include <stdint.h>

bool network_stack_init(void);
bool network_stack_is_ready(void);

void network_stack_poll(void);
void network_stack_schedule_poll(void);
bool network_stack_check_poll(void);

void network_stack_on_timer_tick(void);

uint32_t network_stack_local_ipv4(void);
