#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*virtio_net_rx_callback_t)(const uint8_t *frame, uint16_t frame_len);

bool virtio_net_init(void);
bool virtio_net_is_ready(void);
uint16_t virtio_net_mtu(void);
void virtio_net_get_mac(uint8_t mac_out[6]);

bool virtio_net_send(const uint8_t *frame, uint16_t frame_len);
void virtio_net_poll(void);

void virtio_net_set_rx_callback(virtio_net_rx_callback_t cb);
