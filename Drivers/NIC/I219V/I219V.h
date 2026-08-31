#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*i219v_rx_callback_t)(const uint8_t *frame, uint16_t frame_len);

bool i219v_init(void);
bool i219v_is_ready(void);
uint16_t i219v_mtu(void);
void i219v_get_mac(uint8_t mac_out[6]);

bool i219v_send(const uint8_t *frame, uint16_t frame_len);
void i219v_poll(void);

void i219v_set_rx_callback(i219v_rx_callback_t cb);
