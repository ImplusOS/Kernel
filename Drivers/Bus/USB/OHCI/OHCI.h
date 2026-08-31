#pragma once
#include <stdint.h>
#include <stdbool.h>

struct usb_device_request;

#define OHCI_ED_SKIP    (1u << 14)
#define OHCI_ED_HALT    (1u << 0)
#define OHCI_TD_ROUND      (1u << 18)
#define OHCI_TD_DP_SETUP   (0u << 19)
#define OHCI_TD_DP_OUT     (1u << 19)
#define OHCI_TD_DP_IN      (2u << 19)
#define OHCI_TD_DI_MASK    (7u << 21)
#define OHCI_TD_TOGGLE_0   (2u << 24)
#define OHCI_TD_TOGGLE_1   (3u << 24)
#define OHCI_TD_CC_MASK    0xF0000000u
#define OHCI_TD_CC_NOERR   0x00000000u

typedef struct __attribute__((packed, aligned(16))) ohci_hcca {
    volatile uint32_t int_table[32];
    volatile uint16_t frame_number;
    volatile uint16_t pad1;
    volatile uint32_t done_head;
    volatile uint8_t  reserved[116];
} ohci_hcca_t;

typedef struct __attribute__((packed, aligned(16))) ohci_ed {
    volatile uint32_t control;
    volatile uint32_t tailp;
    volatile uint32_t headp;
    volatile uint32_t next;
} ohci_ed_t;

typedef struct __attribute__((packed, aligned(16))) ohci_td {
    volatile uint32_t control;
    volatile uint32_t curr_buf;
    volatile uint32_t next_td;
    volatile uint32_t buf_end;
} ohci_td_t;

void ohci_init(void);
bool ohci_is_ready(void);
uint32_t ohci_get_num_ports(void);
bool ohci_reset_port(uint32_t port);
bool ohci_port_connected(uint32_t port);
bool ohci_port_valid(uint32_t port);

bool ohci_submit_control(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                         struct usb_device_request *req, void *data);
bool ohci_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                      uint8_t pid, void *data, uint32_t length);

bool ohci_submit_interrupt_in(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                              void *data, uint16_t length);
