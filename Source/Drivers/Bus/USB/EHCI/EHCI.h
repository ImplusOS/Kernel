#pragma once
#include <stdint.h>
#include <stdbool.h>

struct usb_device_request;

typedef struct ehci_qtd {
    volatile uint32_t next_qtd;
    volatile uint32_t alt_next_qtd;
    volatile uint32_t token;
    volatile uint32_t buffer[5];
    volatile uint32_t padding[8];
} __attribute__((packed, aligned(32))) ehci_qtd_t;

typedef struct ehci_qh {
    volatile uint32_t horiz_link;
    volatile uint32_t ep_chars;
    volatile uint32_t ep_caps;
    volatile uint32_t current_qtd;
    volatile uint32_t next_qtd;
    volatile uint32_t alt_next_qtd;
    volatile uint32_t token;
    volatile uint32_t buffer[5];
    volatile uint32_t padding[4];
} __attribute__((packed, aligned(32))) ehci_qh_t;

typedef struct {
    volatile uint32_t usbcmd;
    volatile uint32_t usbsts;
    volatile uint32_t usbintr;
    volatile uint32_t frindex;
    volatile uint32_t ctrldssegment;
    volatile uint32_t periodiclistbase;
    volatile uint32_t asynclistaddr;
    volatile uint32_t reserved[9];
    volatile uint32_t configflag;
    volatile uint32_t portsc[];
} __attribute__((packed)) ehci_op_regs_t;

typedef struct {
    volatile uint8_t  caplength;
    volatile uint8_t  reserved;
    volatile uint16_t hciversion;
    volatile uint32_t hcsparams;
    volatile uint32_t hccparams;
    volatile uint64_t hcsp_portroute;
} __attribute__((packed)) ehci_cap_regs_t;

void ehci_init(void);

bool ehci_submit_control(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                         struct usb_device_request *req, void *data);
bool ehci_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                      uint8_t pid, void *data, uint32_t length);

uint32_t ehci_get_num_ports(void);
bool ehci_reset_port(uint32_t port);
void ehci_delay_ms(uint32_t ms);
bool ehci_port_connected(uint32_t port);
bool ehci_port_valid(uint32_t port);

bool ehci_submit_interrupt_in(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                              void *data, uint16_t length);
