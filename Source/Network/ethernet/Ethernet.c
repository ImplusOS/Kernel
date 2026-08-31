#include "Ethernet.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Drivers/Module/DriverManager.h"
#include "Network/network_utils.h"
#include "Core/sync/Spinlock.h"
#include "Debug/serial/Serial.h"

#define ETHERNET_MAX_HANDLERS 8u
#define ETHERNET_MAX_FRAME_BYTES 1600u
#define ETHERNET_RX_QUEUE_SIZE 64u

typedef struct {
    uint16_t ether_type;
    ethernet_type_handler_t handler;
} ethernet_handler_entry_t;

typedef struct {
    uint16_t len;
    uint8_t data[ETHERNET_MAX_FRAME_BYTES];
} ethernet_rx_frame_t;

static spinlock_t g_ethernet_lock = {0};
static int g_ethernet_ready = 0;
static uint8_t g_local_mac[ETHERNET_ADDR_LEN] = {0};
static ethernet_handler_entry_t g_handlers[ETHERNET_MAX_HANDLERS];
static ethernet_rx_frame_t g_rx_queue[ETHERNET_RX_QUEUE_SIZE];
static uint32_t g_rx_head = 0;
static uint32_t g_rx_tail = 0;
static uint32_t g_rx_count = 0;

static void ethernet_dispatch_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < ETHERNET_HEADER_BYTES) {
        return;
    }

    const uint8_t *dst_mac = frame;
    const uint8_t *src_mac = frame + 6u;
    uint16_t ether_type = (uint16_t)(((uint16_t)frame[12] << 8) | (uint16_t)frame[13]);
    const uint8_t *payload = frame + ETHERNET_HEADER_BYTES;
    uint16_t payload_len = (uint16_t)(frame_len - ETHERNET_HEADER_BYTES);

    ethernet_type_handler_t handler = NULL;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ethernet_lock);

    if (g_ethernet_ready != 0) {
        for (uint32_t i = 0; i < ETHERNET_MAX_HANDLERS; ++i) {
            if (g_handlers[i].handler != NULL && g_handlers[i].ether_type == ether_type) {
                handler = g_handlers[i].handler;
                break;
            }
        }
    }

    spinlock_unlock(&g_ethernet_lock);
    irq_restore(irq_flags);

    if (handler != NULL) {
        handler(src_mac, dst_mac, payload, payload_len);
    }
}

static void ethernet_receive_from_nic(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < ETHERNET_HEADER_BYTES ||
        frame_len > ETHERNET_MAX_FRAME_BYTES) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ethernet_lock);

    if (g_rx_count < ETHERNET_RX_QUEUE_SIZE) {
        ethernet_rx_frame_t *slot = &g_rx_queue[g_rx_head];
        slot->len = frame_len;
        memcpy(slot->data, frame, frame_len);
        g_rx_head = (g_rx_head + 1u) % ETHERNET_RX_QUEUE_SIZE;
        g_rx_count++;
    }

    spinlock_unlock(&g_ethernet_lock);
    irq_restore(irq_flags);
}

static void ethernet_drain_rx_queue(void)
{
    ethernet_rx_frame_t batch[16];
    uint32_t batch_count = 0;

    for (;;) {
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_ethernet_lock);

        if (g_rx_count == 0u) {
            spinlock_unlock(&g_ethernet_lock);
            irq_restore(irq_flags);
            break;
        }

        while (g_rx_count > 0u && batch_count < 16u) {
            batch[batch_count] = g_rx_queue[g_rx_tail];
            g_rx_tail = (g_rx_tail + 1u) % ETHERNET_RX_QUEUE_SIZE;
            g_rx_count--;
            batch_count++;
        }

        spinlock_unlock(&g_ethernet_lock);
        irq_restore(irq_flags);

        for (uint32_t i = 0u; i < batch_count; i++) {
            ethernet_dispatch_frame(batch[i].data, batch[i].len);
        }
        batch_count = 0;
    }
}

bool ethernet_init(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ethernet_lock);

    if (g_ethernet_ready != 0) {
        spinlock_unlock(&g_ethernet_lock);
        irq_restore(irq_flags);
        return true;
    }

    memset(g_handlers, 0, sizeof(g_handlers));

    spinlock_unlock(&g_ethernet_lock);
    irq_restore(irq_flags);

    if (!driver_manager_nic_init()) {
        return false;
    }

    driver_manager_nic_get_mac(g_local_mac);
    driver_manager_nic_set_rx_callback(ethernet_receive_from_nic);

    irq_flags = irq_save_disable();
    spinlock_lock(&g_ethernet_lock);
    g_ethernet_ready = 1;
    spinlock_unlock(&g_ethernet_lock);
    irq_restore(irq_flags);

    return true;
}

bool ethernet_is_ready(void)
{
    return g_ethernet_ready != 0;
}

void ethernet_poll(void)
{
    driver_manager_nic_poll();
    ethernet_drain_rx_queue();
}

void ethernet_get_local_mac(uint8_t mac_out[ETHERNET_ADDR_LEN])
{
    if (mac_out == NULL) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ethernet_lock);

    for (uint8_t i = 0; i < ETHERNET_ADDR_LEN; ++i) {
        mac_out[i] = g_local_mac[i];
    }

    spinlock_unlock(&g_ethernet_lock);
    irq_restore(irq_flags);
}

bool ethernet_send(uint16_t ether_type,
                   const uint8_t dst_mac[ETHERNET_ADDR_LEN],
                   const void *payload,
                   uint16_t payload_len)
{
    if (dst_mac == NULL || (payload == NULL && payload_len != 0u)) {
        return false;
    }
    if (payload_len > driver_manager_nic_mtu()) {
        return false;
    }

    uint16_t frame_len = (uint16_t)(ETHERNET_HEADER_BYTES + payload_len);
    if (frame_len > ETHERNET_MAX_FRAME_BYTES) {
        return false;
    }

    uint8_t frame[ETHERNET_MAX_FRAME_BYTES];
    memset(frame, 0, (size_t)frame_len);

    for (uint8_t i = 0; i < ETHERNET_ADDR_LEN; ++i) {
        frame[i] = dst_mac[i];
        frame[6u + i] = g_local_mac[i];
    }

    frame[12] = (uint8_t)((ether_type >> 8) & 0xFFu);
    frame[13] = (uint8_t)(ether_type & 0xFFu);

    if (payload_len > 0u) {
        memcpy(frame + ETHERNET_HEADER_BYTES, payload, payload_len);
    }

    return driver_manager_nic_send_frame(frame, frame_len);
}

bool ethernet_register_handler(uint16_t ether_type, ethernet_type_handler_t handler)
{
    if (handler == NULL) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ethernet_lock);

    for (uint32_t i = 0; i < ETHERNET_MAX_HANDLERS; ++i) {
        if (g_handlers[i].handler == handler && g_handlers[i].ether_type == ether_type) {
            spinlock_unlock(&g_ethernet_lock);
            irq_restore(irq_flags);
            return true;
        }
    }

    for (uint32_t i = 0; i < ETHERNET_MAX_HANDLERS; ++i) {
        if (g_handlers[i].handler == NULL) {
            g_handlers[i].ether_type = ether_type;
            g_handlers[i].handler = handler;
            spinlock_unlock(&g_ethernet_lock);
            irq_restore(irq_flags);
            return true;
        }
    }

    spinlock_unlock(&g_ethernet_lock);
    irq_restore(irq_flags);
    return false;
}
