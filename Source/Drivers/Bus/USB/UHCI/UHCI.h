#pragma once
#include <stdint.h>
#include <stdbool.h>

struct usb_device_request;

#define UHCI_NUM_FRAMES 1024

#define UHCI_TD_CTRL_ACTIVE    (1u << 23)
#define UHCI_TD_CTRL_STALLED   (1u << 22)
#define UHCI_TD_CTRL_DBUFERR   (1u << 21)
#define UHCI_TD_CTRL_BABBLE    (1u << 20)
#define UHCI_TD_CTRL_NAK       (1u << 19)
#define UHCI_TD_CTRL_CRCTIMEO  (1u << 18)
#define UHCI_TD_CTRL_BITSTUFF  (1u << 17)
#define UHCI_TD_CTRL_IOC       (1u << 24)
#define UHCI_TD_CTRL_ISO       (1u << 25)
#define UHCI_TD_CTRL_LS        (1u << 26)
#define UHCI_TD_CTRL_SPD       (1u << 29)

#define UHCI_PID_OUT   0xE1
#define UHCI_PID_IN    0x69
#define UHCI_PID_SETUP 0x2D

typedef struct __attribute__((packed, aligned(16))) uhci_td {
    volatile uint32_t link;
    volatile uint32_t ctrl_status;
    volatile uint32_t token;
    volatile uint32_t buffer[5];
} uhci_td_t;

typedef struct __attribute__((packed, aligned(16))) uhci_qh {
    volatile uint32_t head_link;
    volatile uint32_t elem_link;
} uhci_qh_t;

void uhci_init(void);
uint32_t uhci_get_num_ports(void);
bool uhci_reset_port(uint32_t port);
bool uhci_port_connected(uint32_t port);
bool uhci_port_valid(uint32_t port);

bool uhci_submit_control(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                         struct usb_device_request *req, void *data);
bool uhci_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                      uint8_t pid, void *data, uint32_t length);

bool uhci_submit_interrupt_in(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                              void *data, uint16_t length);

bool uhci_is_ready(void);
