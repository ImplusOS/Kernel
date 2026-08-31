#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define USB_REQ_TYPE_STANDARD 0x00
#define USB_REQ_TYPE_CLASS    0x20
#define USB_REQ_TYPE_VENDOR   0x40

#define USB_REQ_RCPT_DEVICE    0x00
#define USB_REQ_RCPT_INTERFACE 0x01
#define USB_REQ_RCPT_ENDPOINT  0x02
#define USB_REQ_RCPT_OTHER     0x03

#define USB_REQ_DIR_OUT 0x00
#define USB_REQ_DIR_IN  0x80

#define USB_REQ_GET_STATUS        0x00
#define USB_REQ_CLEAR_FEATURE     0x01
#define USB_REQ_SET_FEATURE       0x03
#define USB_REQ_SET_ADDRESS       0x05
#define USB_REQ_GET_DESCRIPTOR    0x06
#define USB_REQ_SET_DESCRIPTOR    0x07
#define USB_REQ_GET_CONFIGURATION 0x08
#define USB_REQ_SET_CONFIGURATION 0x09

#define USB_DESC_DEVICE        0x01
#define USB_DESC_CONFIGURATION 0x02
#define USB_DESC_STRING        0x03
#define USB_DESC_INTERFACE     0x04
#define USB_DESC_ENDPOINT      0x05

typedef struct usb_device_request {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((packed)) usb_device_request_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} __attribute__((packed)) usb_device_descriptor_t;

typedef enum {
    USB_HC_NONE = 0,
    USB_HC_OHCI,
    USB_HC_UHCI,
    USB_HC_EHCI,
    USB_HC_XHCI,
} usb_hc_type_t;

void usb_core_init(void);
usb_hc_type_t usb_get_hc_type(void);
usb_hc_type_t usb_get_device_hc_type(uint8_t addr);
void usb_set_hc_type(usb_hc_type_t type);

bool usb_control_transfer(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                          usb_device_request_t *req, void *data_buffer);

bool usb_submit_control(uint8_t addr,
                        uint8_t bmRequestType, uint8_t bRequest,
                        uint16_t wValue, uint16_t wIndex, uint16_t wLength,
                        void *data);

bool usb_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                     uint8_t pid, void *data, uint32_t length);

bool usb_set_address(uint8_t old_addr, uint8_t new_addr);
bool usb_get_device_descriptor(uint8_t addr, usb_device_descriptor_t *desc);

bool usb_submit_interrupt_in_async(uint8_t addr, uint8_t ep_num,
                                    uint16_t max_packet_size,
                                    void *dma_buf, uint64_t dma_phys,
                                    uint16_t length);
int  usb_check_interrupt_event(uint8_t addr, uint8_t ep_num);
bool usb_recover_interrupt_in(uint8_t addr, uint8_t ep_num);

bool usb_submit_interrupt_in_sync(uint8_t addr, uint8_t endpoint,
                                   uint16_t max_packet_size,
                                   void *data, uint16_t length);
