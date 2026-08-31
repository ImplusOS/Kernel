#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "Drivers/Module/DriverBinary.h"

struct usb_device_request;

typedef struct xhci_trb {
    volatile uint64_t parameter;
    volatile uint32_t status;
    volatile uint32_t control;
} __attribute__((packed, aligned(16))) xhci_trb_t;

#define TRB_TYPE_NORMAL          1
#define TRB_TYPE_SETUP_STAGE     2
#define TRB_TYPE_DATA_STAGE      3
#define TRB_TYPE_STATUS_STAGE    4
#define TRB_TYPE_LINK            6
#define TRB_TYPE_EVENT_DATA      7
#define TRB_TYPE_TRANSFER_EVENT  32
#define TRB_TYPE_CMD_COMPLETION  33
#define TRB_TYPE_PORT_STATUS_CHG 34
#define TRB_TYPE_ENABLE_SLOT     9
#define TRB_TYPE_DISABLE_SLOT    10
#define TRB_TYPE_ADDRESS_DEVICE  11
#define TRB_TYPE_CONFIG_ENDPOINT 12
#define TRB_TYPE_EVALUATE_CTX    13
#define TRB_TYPE_RESET_ENDPOINT  14
#define TRB_TYPE_STOP_ENDPOINT   15
#define TRB_TYPE_SET_TR_DEQUEUE  16
#define TRB_TYPE_NO_OP_CMD       23

#define TRB_CTRL_CYCLE  (1u << 0)
#define TRB_CTRL_TC     (1u << 1)
#define TRB_CTRL_ENT    (1u << 1)
#define TRB_CTRL_ISP    (1u << 2)
#define TRB_CTRL_NS     (1u << 3)
#define TRB_CTRL_CH     (1u << 4)
#define TRB_CTRL_IOC    (1u << 5)
#define TRB_CTRL_IDT    (1u << 6)
#define TRB_CTRL_DIR_IN (1u << 16)
#define TRB_CTRL_BSR    (1u << 9)
#define TRB_TYPE_SHIFT  10

#define XHCI_SPEED_SUPER_SPEED      4u
#define XHCI_SPEED_SUPER_SPEED_PLUS 5u

typedef struct {
    volatile uint32_t dw0, dw1, dw2, dw3;
    volatile uint32_t rsvd[4];
} __attribute__((packed)) xhci_slot_ctx_t;

typedef struct {
    volatile uint32_t dw0, dw1;
    volatile uint64_t deq;
    volatile uint32_t dw4;
    volatile uint32_t rsvd[3];
} __attribute__((packed)) xhci_ep_ctx_t;

typedef struct {
    xhci_slot_ctx_t slot;
    xhci_ep_ctx_t   ep[31];
} __attribute__((packed)) xhci_dev_ctx_t;

typedef struct {
    volatile uint32_t drop_flags;
    volatile uint32_t add_flags;
    volatile uint32_t rsvd[6];
} __attribute__((packed)) xhci_input_ctrl_ctx_t;

typedef struct {
    xhci_input_ctrl_ctx_t ctrl;
    xhci_dev_ctx_t        dev;
} __attribute__((packed)) xhci_input_ctx_t;

typedef struct {
    volatile uint8_t  caplength;
    volatile uint8_t  rsvd;
    volatile uint16_t hciversion;
    volatile uint32_t hcsparams1;
    volatile uint32_t hcsparams2;
    volatile uint32_t hcsparams3;
    volatile uint32_t hccparams1;
    volatile uint32_t dboff;
    volatile uint32_t rtsoff;
    volatile uint32_t hccparams2;
} xhci_cap_regs_t;

typedef struct {
    volatile uint32_t portsc;
    volatile uint32_t portpmsc;
    volatile uint32_t portli;
    volatile uint32_t porthlpmc;
} xhci_port_regs_t;

typedef struct {
    volatile uint32_t usbcmd;
    volatile uint32_t usbsts;
    volatile uint32_t pagesize;
    volatile uint32_t rsvd1[2];
    volatile uint32_t dnctrl;
    volatile uint64_t crcr;
    volatile uint32_t rsvd2[4];
    volatile uint64_t dcbaap;
    volatile uint32_t config;
    volatile uint8_t  rsvd3[0x400 - 0x3C];
    xhci_port_regs_t  ports[];
} xhci_op_regs_t;

#define XHCI_CMD_RUN   (1u << 0)
#define XHCI_CMD_HCRST (1u << 1)
#define XHCI_CMD_INTE  (1u << 2)
#define XHCI_CMD_HSEE  (1u << 3)

#define XHCI_STS_HCH (1u << 0)
#define XHCI_STS_CNR (1u << 11)

#define XHCI_IMAN_IP (1u << 0)
#define XHCI_IMAN_IE (1u << 1)

#define XHCI_PORTSC_CCS (1u << 0)
#define XHCI_PORTSC_PED (1u << 1)
#define XHCI_PORTSC_PR  (1u << 4)
#define XHCI_PORTSC_PRS (1u << 4)
#define XHCI_PORTSC_PP  (1u << 9)
#define XHCI_PORTSC_CSC (1u << 17)
#define XHCI_PORTSC_PEC (1u << 18)
#define XHCI_PORTSC_WRC (1u << 19)
#define XHCI_PORTSC_PRC (1u << 21)
#define XHCI_PORTSC_PLC (1u << 22)
#define XHCI_PORTSC_WPR (1u << 31)

#define XHCI_PORTSC_PLS_SHIFT 5
#define XHCI_PORTSC_PLS_MASK  (0xFu << XHCI_PORTSC_PLS_SHIFT)
#define XHCI_PORTSC_PLS_U0    (0u << XHCI_PORTSC_PLS_SHIFT)

#define XHCI_PORTSC_SPEED_SHIFT 10
#define XHCI_PORTSC_SPEED_MASK  0xFu

typedef struct {
    volatile uint32_t iman;
    volatile uint32_t imod;
    volatile uint32_t erstsz;
    volatile uint32_t rsvd;
    volatile uint64_t erstba;
    volatile uint64_t erdp;
} xhci_intr_regs_t;

typedef struct {
    volatile uint64_t base;
    volatile uint16_t size;
    volatile uint16_t rsvd[3];
} __attribute__((packed)) xhci_erst_entry_t;

extern uint8_t g_xhci_pci_bus;
extern uint8_t g_xhci_pci_dev;
extern uint8_t g_xhci_pci_func;

void xhci_init(void);

bool xhci_submit_control(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                         struct usb_device_request *req, void *data);
bool xhci_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                      uint8_t pid, void *data, uint32_t length);
uint32_t xhci_get_max_bulk_transfer_size(void);

uint32_t xhci_get_num_ports(void);
bool     xhci_reset_port(uint32_t port);
void     xhci_delay_ms(uint32_t ms);
bool     xhci_is_ready(void);
bool     xhci_port_connected(uint32_t port);
bool     xhci_port_valid(uint32_t port);
bool     xhci_evaluate_ep0_mps(uint8_t addr, uint16_t new_mps);

bool xhci_submit_interrupt_in(uint8_t addr, uint8_t ep_num,
                                     uint16_t max_packet_size,
                                     void *dma_buf, uint64_t dma_phys,
                                     uint16_t length);
bool xhci_submit_interrupt_in_sync(uint8_t addr, uint8_t ep_num,
                                   uint16_t max_packet_size,
                                   void *data, uint16_t length);
int  xhci_check_interrupt_event(uint8_t addr, uint8_t ep_num);
bool xhci_recover_interrupt_endpoint(uint8_t addr, uint8_t ep_num);
void xhci_disable_slot(uint8_t addr);
