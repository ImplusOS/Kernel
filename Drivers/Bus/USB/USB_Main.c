#include "USB_Main.h"
#include "EHCI/EHCI.h"
#include "OHCI/OHCI.h"
#include "UHCI/UHCI.h"
#include "XHCI/XHCI.h"
#include "MassStorage/MassStorage.h"
#include "HID/USB_HID.h"
#include "Drivers/Module/DriverBinary.h"

const driver_binary_t *g_api = NULL;

uint8_t  g_mass_storage_addr      = 0;
uint8_t  g_mass_storage_ep_in     = 0;
uint8_t  g_mass_storage_ep_out    = 0;
uint8_t  g_mass_storage_interface = 0;
uint16_t g_mass_storage_ep_in_mps = 0;
uint16_t g_mass_storage_ep_out_mps= 0;

uint8_t  g_enum_speed = 0;
uint8_t  g_enum_parent_hub_addr = 0;
uint8_t  g_enum_parent_port = 0;
uint8_t  g_dev_root_port[256] = {0};
uint32_t g_dev_route_string[256] = {0};

static uint8_t  g_hid_kbd_addr      = 0;
static uint8_t  g_hid_kbd_interface = 0;
static uint8_t  g_hid_kbd_ep_in     = 0;
static uint16_t g_hid_kbd_ep_mps    = 0;
static uint16_t g_hid_kbd_vendor_id = 0;
static uint16_t g_hid_kbd_device_id = 0;

static uint8_t  g_hid_mouse_addr      = 0;
static uint8_t  g_hid_mouse_interface = 0;
static uint8_t  g_hid_mouse_ep_in     = 0;
static uint16_t g_hid_mouse_ep_mps    = 0;
static uint16_t g_hid_mouse_vendor_id = 0;
static uint16_t g_hid_mouse_device_id = 0;

static usb_hc_type_t g_hc_type = USB_HC_NONE;
static bool g_ohci_ready = false;
static usb_hc_type_t g_dev_hc[256];
static uint16_t g_dev_ep0_mps[256] = {0};
static usb_hc_type_t g_default_hc = USB_HC_NONE;
static bool g_uhci_ready = false;
static bool g_usb_initialized = false;
static uint8_t g_usb_next_addr = 1;

#define USB_HC_TABLE_SIZE 5u
#define USB_MAX_TRACKED_ROOT_PORTS 32u
#define USB_MAX_TRACKED_HUB_PORTS 16u
#define USB_MAX_TRACKED_HUBS 16u

typedef struct {
    bool connected;
    uint8_t addr;
    usb_hc_type_t hc;
    uint32_t port;
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t has_keyboard;
    uint8_t has_mouse;
    uint8_t keyboard_addr;
    uint8_t keyboard_interface;
    uint8_t keyboard_endpoint;
    uint16_t keyboard_vendor_id;
    uint16_t keyboard_device_id;
    uint8_t mouse_addr;
    uint8_t mouse_interface;
    uint8_t mouse_endpoint;
    uint16_t mouse_vendor_id;
    uint16_t mouse_device_id;
} usb_root_port_state_t;

static usb_root_port_state_t g_root_ports[USB_HC_TABLE_SIZE][USB_MAX_TRACKED_ROOT_PORTS];

typedef struct {
    bool connected;
    uint8_t addr;
} usb_hub_port_state_t;

static usb_hub_port_state_t g_hub_ports[256][USB_MAX_TRACKED_HUB_PORTS];
static uint8_t g_usb_hub_addrs[USB_MAX_TRACKED_HUBS];
static uint8_t g_usb_hub_count = 0;

usb_hc_type_t usb_get_hc_type(void) { return g_hc_type; }
usb_hc_type_t usb_get_device_hc_type(uint8_t addr) { return g_dev_hc[addr]; }

static void usb_publish_hid_event(uint32_t device_class,
                                  uint16_t action,
                                  const char *device_name,
                                  const char *detail,
                                  uint8_t addr,
                                  uint8_t interface,
                                  uint8_t endpoint,
                                  uint16_t vendor_id,
                                  uint16_t device_id)
{
    if (g_api == NULL || g_api->pnp_notify == NULL) {
        return;
    }

    pnp_event_t event;
    pnp_event_init(&event,
                   action,
                   PNP_BUS_USB,
                   device_class,
                   "USB_Driver.ELF",
                   device_name,
                   detail);
    event.vendor_id = vendor_id;
    event.device_id = device_id;
    event.location0 = ((uint32_t)addr << 16u) |
                      ((uint32_t)interface << 8u) |
                      (uint32_t)endpoint;
    event.location1 = (uint32_t)usb_get_device_hc_type(addr);
    g_api->pnp_notify(&event);
}

static void usb_wait_ms(usb_hc_type_t hc, uint32_t ms)
{
    if (ms == 0) return;
    if (hc == USB_HC_EHCI) ehci_delay_ms(ms);
    else                   xhci_delay_ms(ms);
}

bool usb_submit_control(uint8_t addr,
                        uint8_t bmRequestType, uint8_t bRequest,
                        uint16_t wValue, uint16_t wIndex, uint16_t wLength,
                        void *data)
{
    usb_device_request_t req;
    req.bmRequestType = bmRequestType;
    req.bRequest      = bRequest;
    req.wValue        = wValue;
    req.wIndex        = wIndex;
    req.wLength       = wLength;
    
    usb_hc_type_t hc = (addr == 0) ? g_default_hc : g_dev_hc[addr];
    if (hc == USB_HC_NONE) hc = usb_get_hc_type();
    uint16_t mps = (addr == 0) ? ((hc == USB_HC_EHCI || hc == USB_HC_XHCI) ? 64 : 8) : g_dev_ep0_mps[addr];
    if (mps == 0) mps = 8;
    
    return usb_control_transfer(addr, 0, mps, &req, data);
}

#define USB_DESC_HUB 0x29

#define USB_PORT_STATUS_CONNECTION   (1u << 0)
#define USB_PORT_STATUS_ENABLE       (1u << 1)
#define USB_PORT_STATUS_SUSPEND      (1u << 2)
#define USB_PORT_STATUS_RESET        (1u << 4)
#define USB_PORT_STATUS_POWER        (1u << 8)

#define USB_FEATURE_PORT_CONNECTION  0
#define USB_FEATURE_PORT_ENABLE      1
#define USB_FEATURE_PORT_SUSPEND     2
#define USB_FEATURE_PORT_OVER_CURRENT 3
#define USB_FEATURE_PORT_RESET       4
#define USB_FEATURE_PORT_POWER       8
#define USB_FEATURE_C_PORT_CONNECTION 16
#define USB_FEATURE_C_PORT_RESET     20

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bNbrPorts;
    uint16_t wHubCharacteristics;
    uint8_t  bPwrOn2PwrGood;
    uint8_t  bHubContrCurrent;
    uint8_t  bDeviceRemovable;
    uint8_t  bPortPwrCtrlMask;
} __attribute__((packed)) usb_hub_descriptor_t;

static uint16_t usb_le16(const uint8_t *buffer)
{
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

static uint16_t usb_device_max_packet_size(uint8_t packet_size)
{
    return (packet_size == 9) ? 512 : packet_size;
}

static bool usb_get_configuration_value(uint8_t addr,
                                        uint16_t max_packet_size,
                                        uint8_t *out_config)
{
    if (out_config == NULL) return false;
    usb_device_request_t req;
    req.bmRequestType = USB_REQ_DIR_IN | USB_REQ_TYPE_STANDARD | USB_REQ_RCPT_DEVICE;
    req.bRequest      = USB_REQ_GET_CONFIGURATION;
    req.wValue        = 0;
    req.wIndex        = 0;
    req.wLength       = 1;
    uint8_t value = 0;
    if (!usb_control_transfer(addr, 0, max_packet_size, &req, &value)) {
        return false;
    }
    *out_config = value;
    return true;
}

static bool usb_hub_get_descriptor(uint8_t addr, uint16_t max_packet_size, usb_hub_descriptor_t *hub_desc)
{
    usb_device_request_t req;
    req.bmRequestType = USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS | USB_REQ_RCPT_DEVICE;
    req.bRequest      = USB_REQ_GET_DESCRIPTOR;
    req.wValue        = (USB_DESC_HUB << 8) | 0;
    req.wIndex        = 0;
    req.wLength       = sizeof(usb_hub_descriptor_t);
    return usb_control_transfer(addr, 0, max_packet_size, &req, hub_desc);
}

static bool usb_hub_get_port_status(uint8_t addr, uint8_t port, uint16_t max_packet_size,
                                   uint16_t *status, uint16_t *change)
{
    uint8_t buffer[4];
    usb_device_request_t req;
    req.bmRequestType = USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS | USB_REQ_RCPT_OTHER;
    req.bRequest      = USB_REQ_GET_STATUS;
    req.wValue        = 0;
    req.wIndex        = port;
    req.wLength       = sizeof(buffer);
    if (!usb_control_transfer(addr, 0, max_packet_size, &req, buffer)) {
        return false;
    }
    
    *status = usb_le16(buffer);
    *change = usb_le16(buffer + 2);
    return true;
}
static bool usb_hub_set_port_feature(uint8_t addr, uint8_t port, uint16_t feature, uint16_t max_packet_size)
{
    usb_device_request_t req;
    req.bmRequestType = USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS | USB_REQ_RCPT_OTHER;
    req.bRequest      = USB_REQ_SET_FEATURE;
    req.wValue        = feature;
    req.wIndex        = port;
    req.wLength       = 0;
    return usb_control_transfer(addr, 0, max_packet_size, &req, NULL);
}

static bool usb_hub_clear_port_feature(uint8_t addr, uint8_t port, uint16_t feature, uint16_t max_packet_size)
{
    usb_device_request_t req;
    req.bmRequestType = USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS | USB_REQ_RCPT_OTHER;
    req.bRequest      = USB_REQ_CLEAR_FEATURE;
    req.wValue        = feature;
    req.wIndex        = port;
    req.wLength       = 0;
    return usb_control_transfer(addr, 0, max_packet_size, &req, NULL);
}

static bool usb_hub_power_on_port(uint8_t addr, uint8_t port, uint16_t max_packet_size)
{
    if (!usb_hub_set_port_feature(addr, port, USB_FEATURE_PORT_POWER, max_packet_size)) {
        return false;
    }
    for (int i = 0; i < 50; ++i) {
        uint16_t status, change;
        if (usb_hub_get_port_status(addr, port, max_packet_size, &status, &change)) {
            if (status & USB_PORT_STATUS_POWER) {
                return true;
            }
        }
        usb_wait_ms(g_dev_hc[addr], 10);
    }
    return false;
}

static bool usb_hub_reset_port(uint8_t addr, uint8_t port, uint16_t max_packet_size)
{
    if (!usb_hub_set_port_feature(addr, port, USB_FEATURE_PORT_RESET, max_packet_size)) {
        return false;
    }

    for (int i = 0; i < 100; ++i) {
        uint16_t status, change;
        if (!usb_hub_get_port_status(addr, port, max_packet_size, &status, &change)) {
            return false;
        }
        if ((status & USB_PORT_STATUS_RESET) == 0) {
            break;
        }
        usb_wait_ms(g_dev_hc[addr], 10);
    }

    uint16_t status, change;
    if (!usb_hub_get_port_status(addr, port, max_packet_size, &status, &change)) {
        return false;
    }
    usb_hub_clear_port_feature(addr, port, USB_FEATURE_C_PORT_RESET, max_packet_size);
    usb_hub_clear_port_feature(addr, port, USB_FEATURE_C_PORT_CONNECTION, max_packet_size);
    return (status & USB_PORT_STATUS_CONNECTION) != 0;
}

static bool usb_enumerate_hub(uint8_t hub_addr, uint16_t max_packet_size, uint8_t *next_addr);
static void usb_track_hub(uint8_t hub_addr);
static void usb_rescan_known_hubs(uint8_t *next_addr);

static bool usb_enumerate_device(uint8_t current_addr, usb_hc_type_t port_hc,
                                 uint16_t max_packet_size, uint8_t *next_addr)
{
    usb_device_descriptor_t desc = {0};
    bool desc_ok = false;

    for (int retry_count = 8; retry_count-- > 0;) {
        if (usb_get_device_descriptor(current_addr, &desc)) {
            desc_ok = true;
            break;
        }
        usb_wait_ms(port_hc, 50);
    }
    if (!desc_ok) {
        return false;
    }

    uint16_t detected_mps = usb_device_max_packet_size(desc.bMaxPacketSize0);
    if (detected_mps == 0u || detected_mps > 512u) {
        detected_mps = max_packet_size;
    }
    if (port_hc == USB_HC_XHCI &&
        detected_mps != max_packet_size) {
        if (xhci_evaluate_ep0_mps(current_addr, detected_mps)) {
            max_packet_size = detected_mps;
            usb_wait_ms(port_hc, 20);
        }
    } else {
        max_packet_size = detected_mps;
    }
    g_dev_ep0_mps[current_addr] = max_packet_size;

    uint8_t conf_buf[2048];
    usb_device_request_t req;
    req.bmRequestType = USB_REQ_DIR_IN | USB_REQ_TYPE_STANDARD | USB_REQ_RCPT_DEVICE;
    req.bRequest      = USB_REQ_GET_DESCRIPTOR;
    req.wValue        = (USB_DESC_CONFIGURATION << 8) | 0;
    req.wIndex        = 0;
    req.wLength       = 9;
    uint8_t config_value = 1;
    bool config_header_ok = false;

    bool is_hub = (desc.bDeviceClass == 9);
    uint8_t current_iface = 0xFF;

    uint8_t mass_iface = 0xFF;
    uint8_t mass_ep_in = 0;
    uint8_t mass_ep_out = 0;
    uint16_t mass_ep_in_mps = 0;
    uint16_t mass_ep_out_mps = 0;
    bool mass_bot_scsi = false;

    uint8_t kbd_iface = 0xFF;
    uint8_t kbd_ep_in = 0;
    uint16_t kbd_ep_mps = 0;

    uint8_t mouse_iface = 0xFF;
    uint8_t mouse_ep_in = 0;
    uint16_t mouse_ep_mps = 0;

    /* A single vendor-specific (class 0xFF) interface, reported generically
     * to BusRegistry via g_api->bus.report_device() below instead of being
     * dispatched to a specific driver by name/VID here -- see AX900.c
     * (Kernel/Drivers/Wi-Fi/AX900/) for the driver-side half of this
     * contract. Only one such interface per device is tracked; a device
     * exposing more than one vendor-specific interface only gets the last
     * one reported (a real limitation, not exercised by any device this
     * driver has been tested against). */
    uint8_t vendor_iface = 0xFF;
    uint8_t vendor_subclass = 0;
    uint8_t vendor_protocol = 0;
    uint8_t vendor_ep_in = 0;
    uint8_t vendor_ep_out = 0;
    uint16_t vendor_ep_in_mps = 0;
    uint16_t vendor_ep_out_mps = 0;

    uint16_t total_len = 0u;
    for (uint32_t retry = 0u; retry < 5u; ++retry) {
            req.wLength = 9;
            if (usb_control_transfer(current_addr, 0, max_packet_size, &req, conf_buf)) {
                total_len = usb_le16(&conf_buf[2]);
                if (total_len >= 9u) {
                    config_header_ok = true;
                    config_value = conf_buf[5];
                break;
            }
        }
        usb_wait_ms(port_hc, 50);
    }

    bool config_full_ok = false;
    if (config_header_ok) {
        if (total_len > sizeof(conf_buf)) total_len = sizeof(conf_buf);

        for (uint32_t retry = 0u; retry < 5u; ++retry) {
            req.wLength = total_len;
            if (usb_control_transfer(current_addr, 0, max_packet_size, &req, conf_buf)) {
                config_full_ok = true;
                break;
            }
            usb_wait_ms(port_hc, 50);
        }
    }

    if (config_full_ok) {
        for (uint32_t pos = 0; pos + 2 <= total_len;) {
            uint8_t len = conf_buf[pos];
            uint8_t type = conf_buf[pos + 1];

            if (len == 0 || pos + len > total_len) {
                break;
            }

            if (type == USB_DESC_INTERFACE && len >= 9) {
                current_iface = conf_buf[pos + 2];
                uint8_t iface_class = conf_buf[pos + 5];
                uint8_t iface_subclass = conf_buf[pos + 6];
                uint8_t iface_protocol = conf_buf[pos + 7];
                uint8_t effective_class =
                    (iface_class != 0u) ? iface_class : desc.bDeviceClass;
                uint8_t effective_subclass =
                    (iface_subclass != 0u) ? iface_subclass : desc.bDeviceSubClass;
                uint8_t effective_protocol =
                    (iface_protocol != 0u) ? iface_protocol : desc.bDeviceProtocol;

                if (effective_class == 9) {
                    is_hub = true;
                } else if (effective_class == 0x08 &&
                           effective_subclass == 0x06) {
                    mass_iface = current_iface;
                    mass_bot_scsi =
                        (effective_protocol == 0x50u || effective_protocol == 0x00u);
                } else if (effective_class == 0x03) {
                    if (effective_protocol == 1) {
                        kbd_iface = current_iface;
                    } else if (effective_protocol == 2) {
                        mouse_iface = current_iface;
                    }
                } else if (effective_class == 0xFFu) {
                    vendor_iface = current_iface;
                    vendor_subclass = effective_subclass;
                    vendor_protocol = effective_protocol;
                }
            } else if (type == USB_DESC_ENDPOINT && len >= 7 && current_iface != 0xFF) {
                uint8_t ep_addr = conf_buf[pos + 2];
                uint8_t attributes = conf_buf[pos + 3];
                uint16_t mps = usb_le16(&conf_buf[pos + 4]);

                if (mass_iface != 0xFF && current_iface == mass_iface && (attributes & 0x03u) == 2) {
                    if (ep_addr & 0x80u) {
                        mass_ep_in = ep_addr & 0x7Fu;
                        mass_ep_in_mps = mps;
                    } else {
                        mass_ep_out = ep_addr & 0x7Fu;
                        mass_ep_out_mps = mps;
                    }
                } else if (kbd_iface != 0xFF && current_iface == kbd_iface && (attributes & 0x03u) == 3 && (ep_addr & 0x80u)) {
                    kbd_ep_in = ep_addr & 0x7Fu;
                    kbd_ep_mps = mps;
                } else if (mouse_iface != 0xFF && current_iface == mouse_iface && (attributes & 0x03u) == 3 && (ep_addr & 0x80u)) {
                    mouse_ep_in = ep_addr & 0x7Fu;
                    mouse_ep_mps = mps;
                } else if (vendor_iface != 0xFF && current_iface == vendor_iface && (attributes & 0x03u) == 2) {
                    if (ep_addr & 0x80u) {
                        vendor_ep_in = ep_addr & 0x7Fu;
                        vendor_ep_in_mps = mps;
                    } else {
                        vendor_ep_out = ep_addr & 0x7Fu;
                        vendor_ep_out_mps = mps;
                    }
                }
            }

            pos += len;
        }
    }

    if (!config_header_ok || !config_full_ok) {
        return false;
    }

    req.bmRequestType = USB_REQ_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_REQ_RCPT_DEVICE;
    req.bRequest      = USB_REQ_SET_CONFIGURATION;
    req.wValue        = config_value;
    req.wIndex        = 0;
    req.wLength       = 0;
    bool configured = false;
    for (uint32_t retry = 0u; retry < 5u; ++retry) {
        if (usb_control_transfer(current_addr, 0, max_packet_size, &req, NULL)) {
            configured = true;
            break;
        }
        usb_wait_ms(port_hc, 100u + retry * 100u);
    }
    if (!configured) {
        uint8_t active_config = 0;
        if (usb_get_configuration_value(current_addr, max_packet_size, &active_config) &&
            active_config == config_value) {
            configured = true;
        }
    }

    if (mass_bot_scsi && mass_iface != 0xFF &&
        mass_ep_in != 0 && mass_ep_out != 0) {
        bot_add_device(current_addr, mass_iface, mass_ep_in, mass_ep_out, mass_ep_in_mps, mass_ep_out_mps);
        usb_wait_ms(port_hc, configured ? 150u : 300u);
    }

    if (configured && vendor_iface != 0xFF && vendor_ep_in != 0 && vendor_ep_out != 0 &&
        g_api != NULL && g_api->bus.report_device != NULL) {
        usb_device_context_t usb_ctx;
        usb_ctx.addr = current_addr;
        usb_ctx.interface = vendor_iface;
        usb_ctx.ep_in = vendor_ep_in;
        usb_ctx.ep_out = vendor_ep_out;
        usb_ctx.ep_in_mps = vendor_ep_in_mps;
        usb_ctx.ep_out_mps = vendor_ep_out_mps;
        usb_ctx.submit_bulk = usb_submit_bulk;

        bus_device_t bus_dev;
        bus_dev.bus_type = DEVICE_TYPE_USB;
        bus_dev.vendor_id = desc.idVendor;
        bus_dev.device_id = desc.idProduct;
        bus_dev.class_code = 0xFFu;
        bus_dev.subclass = vendor_subclass;
        bus_dev.protocol = vendor_protocol;
        bus_dev.bus_context = &usb_ctx;

        /* No match today just means no loaded driver claims this device
         * (e.g. AX900_Driver.ELF, Kernel/Drivers/Wi-Fi/AX900/, isn't
         * present) -- not an error. Dynamic loading of a not-yet-loaded
         * matching module on a report_device() miss is a further step this
         * doesn't attempt yet. */
        (void)g_api->bus.report_device(&bus_dev);
    }

    if (configured) {
        usb_wait_ms(port_hc, 100u);

        if (kbd_iface != 0xFF && kbd_ep_in != 0) {
            g_hid_kbd_addr = current_addr;
            g_hid_kbd_interface = kbd_iface;
            g_hid_kbd_ep_in = kbd_ep_in;
            g_hid_kbd_ep_mps = kbd_ep_mps ? kbd_ep_mps : 8;
            g_hid_kbd_vendor_id = desc.idVendor;
            g_hid_kbd_device_id = desc.idProduct;
        }

        if (mouse_iface != 0xFF && mouse_ep_in != 0) {
            g_hid_mouse_addr = current_addr;
            g_hid_mouse_interface = mouse_iface;
            g_hid_mouse_ep_in = mouse_ep_in;
            g_hid_mouse_ep_mps = mouse_ep_mps ? mouse_ep_mps : 8;
            g_hid_mouse_vendor_id = desc.idVendor;
            g_hid_mouse_device_id = desc.idProduct;
        }
    }

    if (g_hid_kbd_addr == current_addr && g_hid_kbd_ep_in != 0) {
        usb_hid_add_keyboard(g_hid_kbd_addr, g_hid_kbd_interface, g_hid_kbd_ep_in, g_hid_kbd_ep_mps);
        usb_publish_hid_event(PNP_CLASS_KEYBOARD,
                              PNP_EVENT_DEVICE_ADDED,
                              "USB HID keyboard",
                              "USB HID device ready",
                              g_hid_kbd_addr,
                              g_hid_kbd_interface,
                              g_hid_kbd_ep_in,
                              desc.idVendor,
                              desc.idProduct);
    }

    if (g_hid_mouse_addr == current_addr && g_hid_mouse_ep_in != 0) {
        usb_hid_add_mouse(g_hid_mouse_addr, g_hid_mouse_interface, g_hid_mouse_ep_in, g_hid_mouse_ep_mps);
        usb_publish_hid_event(PNP_CLASS_MOUSE,
                              PNP_EVENT_DEVICE_ADDED,
                              "USB HID mouse",
                              "USB HID device ready",
                              g_hid_mouse_addr,
                              g_hid_mouse_interface,
                              g_hid_mouse_ep_in,
                              desc.idVendor,
                              desc.idProduct);
    }

    if (is_hub) {
        usb_track_hub(current_addr);
        usb_enumerate_hub(current_addr, max_packet_size, next_addr);
    }
    return true;
}

static bool usb_enumerate_hub_port(uint8_t hub_addr, uint8_t hub_port, uint16_t max_packet_size, uint8_t *next_addr)
{
    usb_hub_port_state_t *state = NULL;
    if (hub_port != 0u && hub_port <= USB_MAX_TRACKED_HUB_PORTS) {
        state = &g_hub_ports[hub_addr][hub_port - 1u];
    }

    if (!usb_hub_power_on_port(hub_addr, hub_port, max_packet_size)) {
        return false;
    }

    uint16_t status, change;
    if (!usb_hub_get_port_status(hub_addr, hub_port, max_packet_size, &status, &change)) {
        return false;
    }

    if ((status & USB_PORT_STATUS_CONNECTION) == 0) {
        if (state != NULL) {
            state->connected = false;
            state->addr = 0u;
        }
        return false;
    }

    if (state != NULL && state->connected) {
        return true;
    }

    if (!usb_hub_reset_port(hub_addr, hub_port, max_packet_size)) {
        return false;
    }

    if (!usb_hub_get_port_status(hub_addr, hub_port, max_packet_size, &status, &change)) {
        return false;
    }
    if ((status & USB_PORT_STATUS_CONNECTION) == 0) {
        if (state != NULL) {
            state->connected = false;
            state->addr = 0u;
        }
        return false;
    }

    g_default_hc = g_dev_hc[hub_addr];
    g_dev_hc[0] = g_dev_hc[hub_addr];

    g_enum_parent_hub_addr = hub_addr;
    g_enum_parent_port = hub_port;

    uint8_t speed = 1;
    if (status & (1u << 9)) speed = 2;
    else if (status & (1u << 10)) speed = 3;
    g_enum_speed = speed;

    uint32_t route = g_dev_route_string[hub_addr];
    for (int i = 0; i < 20; i += 4) {
        if (((route >> i) & 0xF) == 0) {
            route |= ((hub_port & 0xF) << i);
            break;
        }
    }
    g_dev_route_string[0] = route;

    uint8_t current_addr = *next_addr;

    if (!usb_set_address(0, current_addr)) {
        g_enum_parent_hub_addr = 0;
        return false;
    }

    g_enum_parent_hub_addr = 0;

    (*next_addr)++;
    usb_wait_ms(g_dev_hc[hub_addr], 10);
    g_dev_hc[current_addr] = g_dev_hc[hub_addr];
    g_dev_root_port[current_addr] = g_dev_root_port[hub_addr];
    g_dev_route_string[current_addr] = g_dev_route_string[0];
    if (!usb_enumerate_device(current_addr, g_dev_hc[hub_addr], max_packet_size, next_addr)) {
        usb_hc_type_t hc = g_dev_hc[hub_addr];
        g_dev_hc[current_addr] = USB_HC_NONE;
        g_dev_ep0_mps[current_addr] = 0u;
        if (hc == USB_HC_XHCI) {
            xhci_disable_slot(current_addr);
        }
        return false;
    }
    if (state != NULL) {
        state->connected = true;
        state->addr = current_addr;
    }
    return true;
}

static bool usb_enumerate_hub(uint8_t hub_addr, uint16_t max_packet_size, uint8_t *next_addr)
{
    usb_hub_descriptor_t hub_desc = {0};
    if (!usb_hub_get_descriptor(hub_addr, max_packet_size, &hub_desc)) {
        return false;
    }
    if (hub_desc.bNbrPorts == 0) {
        return false;
    }

    for (uint8_t port = 1; port <= hub_desc.bNbrPorts; ++port) {
        usb_hub_clear_port_feature(hub_addr, port, USB_FEATURE_C_PORT_CONNECTION, max_packet_size);
        if (usb_enumerate_hub_port(hub_addr, port, max_packet_size, next_addr)) {
        }
    }
    return true;
}

static void usb_track_hub(uint8_t hub_addr)
{
    if (hub_addr == 0u) {
        return;
    }
    for (uint8_t i = 0u; i < g_usb_hub_count; ++i) {
        if (g_usb_hub_addrs[i] == hub_addr) {
            return;
        }
    }
    if (g_usb_hub_count < USB_MAX_TRACKED_HUBS) {
        g_usb_hub_addrs[g_usb_hub_count++] = hub_addr;
    }
}

static void usb_rescan_known_hubs(uint8_t *next_addr)
{
    for (uint8_t i = 0u; i < g_usb_hub_count; ++i) {
        uint8_t hub_addr = g_usb_hub_addrs[i];
        uint16_t mps = g_dev_ep0_mps[hub_addr];
        if (hub_addr == 0u || g_dev_hc[hub_addr] == USB_HC_NONE) {
            continue;
        }
        if (mps == 0u) {
            mps = 8u;
        }
        (void)usb_enumerate_hub(hub_addr, mps, next_addr);
    }
}

static uint8_t usb_alloc_address(void)
{
    if (g_usb_next_addr == 0u || g_usb_next_addr >= 255u) {
        g_usb_next_addr = 1u;
    }
    return g_usb_next_addr++;
}

static uint32_t usb_hc_port_count(usb_hc_type_t hc)
{
    if (hc == USB_HC_XHCI && xhci_is_ready()) {
        return xhci_get_num_ports();
    }
    if (hc == USB_HC_EHCI && ehci_get_num_ports() > 0u) {
        return ehci_get_num_ports();
    }
    if (hc == USB_HC_OHCI && g_ohci_ready) {
        return ohci_get_num_ports();
    }
    if (hc == USB_HC_UHCI && g_uhci_ready) {
        return uhci_get_num_ports();
    }
    return 0u;
}

static bool usb_hc_port_valid(usb_hc_type_t hc, uint32_t port)
{
    if (hc == USB_HC_XHCI) {
        return xhci_port_valid(port);
    }
    if (hc == USB_HC_EHCI) {
        return ehci_port_valid(port);
    }
    if (hc == USB_HC_OHCI) {
        return ohci_port_valid(port);
    }
    if (hc == USB_HC_UHCI) {
        return uhci_port_valid(port);
    }
    return false;
}

static bool usb_hc_port_connected(usb_hc_type_t hc, uint32_t port)
{
    if (hc == USB_HC_XHCI) {
        return xhci_port_connected(port);
    }
    if (hc == USB_HC_EHCI) {
        return ehci_port_connected(port);
    }
    if (hc == USB_HC_OHCI) {
        return ohci_port_connected(port);
    }
    if (hc == USB_HC_UHCI) {
        return uhci_port_connected(port);
    }
    return false;
}

static bool usb_hc_reset_port(usb_hc_type_t hc, uint32_t port)
{
    if (hc == USB_HC_XHCI) {
        return xhci_reset_port(port);
    }
    if (hc == USB_HC_EHCI) {
        return ehci_reset_port(port);
    }
    if (hc == USB_HC_OHCI) {
        return ohci_reset_port(port);
    }
    if (hc == USB_HC_UHCI) {
        return uhci_reset_port(port);
    }
    return false;
}

static usb_root_port_state_t *usb_root_port_state(usb_hc_type_t hc, uint32_t port)
{
    if ((uint32_t)hc >= USB_HC_TABLE_SIZE ||
        port >= USB_MAX_TRACKED_ROOT_PORTS) {
        return NULL;
    }
    return &g_root_ports[(uint32_t)hc][port];
}

static void usb_clear_root_port_states(void)
{
    for (uint32_t hc = 0u; hc < USB_HC_TABLE_SIZE; ++hc) {
        for (uint32_t port = 0u; port < USB_MAX_TRACKED_ROOT_PORTS; ++port) {
            g_root_ports[hc][port].connected = false;
            g_root_ports[hc][port].addr = 0u;
            g_root_ports[hc][port].hc = USB_HC_NONE;
            g_root_ports[hc][port].port = 0u;
            g_root_ports[hc][port].vendor_id = 0u;
            g_root_ports[hc][port].device_id = 0u;
            g_root_ports[hc][port].has_keyboard = 0u;
            g_root_ports[hc][port].has_mouse = 0u;
            g_root_ports[hc][port].keyboard_addr = 0u;
            g_root_ports[hc][port].keyboard_interface = 0u;
            g_root_ports[hc][port].keyboard_endpoint = 0u;
            g_root_ports[hc][port].keyboard_vendor_id = 0u;
            g_root_ports[hc][port].keyboard_device_id = 0u;
            g_root_ports[hc][port].mouse_addr = 0u;
            g_root_ports[hc][port].mouse_interface = 0u;
            g_root_ports[hc][port].mouse_endpoint = 0u;
            g_root_ports[hc][port].mouse_vendor_id = 0u;
            g_root_ports[hc][port].mouse_device_id = 0u;
        }
    }

    for (uint32_t addr = 0u; addr < 256u; ++addr) {
        for (uint32_t port = 0u; port < USB_MAX_TRACKED_HUB_PORTS; ++port) {
            g_hub_ports[addr][port].connected = false;
            g_hub_ports[addr][port].addr = 0u;
        }
    }
    for (uint32_t i = 0u; i < USB_MAX_TRACKED_HUBS; ++i) {
        g_usb_hub_addrs[i] = 0u;
    }
    g_usb_hub_count = 0u;
}

static void usb_disconnect_root_device(usb_root_port_state_t *state)
{
    if (state == NULL || !state->connected) {
        return;
    }

    if (state->has_keyboard != 0u) {
        usb_publish_hid_event(PNP_CLASS_KEYBOARD,
                              PNP_EVENT_DEVICE_REMOVED,
                              "USB HID keyboard",
                              "USB HID device removed",
                              state->keyboard_addr,
                              state->keyboard_interface,
                              state->keyboard_endpoint,
                              state->keyboard_vendor_id,
                              state->keyboard_device_id);
        usb_hid_remove_keyboard(state->keyboard_addr);
        if (g_hid_kbd_addr == state->keyboard_addr) {
            g_hid_kbd_addr = 0u;
            g_hid_kbd_interface = 0u;
            g_hid_kbd_ep_in = 0u;
            g_hid_kbd_ep_mps = 0u;
            g_hid_kbd_vendor_id = 0u;
            g_hid_kbd_device_id = 0u;
        }
        if (state->keyboard_addr != 0u) {
            g_dev_hc[state->keyboard_addr] = USB_HC_NONE;
            g_dev_ep0_mps[state->keyboard_addr] = 0u;
            g_dev_root_port[state->keyboard_addr] = 0u;
            g_dev_route_string[state->keyboard_addr] = 0u;
        }
    }

    if (state->has_mouse != 0u) {
        usb_publish_hid_event(PNP_CLASS_MOUSE,
                              PNP_EVENT_DEVICE_REMOVED,
                              "USB HID mouse",
                              "USB HID device removed",
                              state->mouse_addr,
                              state->mouse_interface,
                              state->mouse_endpoint,
                              state->mouse_vendor_id,
                              state->mouse_device_id);
        usb_hid_remove_mouse(state->mouse_addr);
        if (g_hid_mouse_addr == state->mouse_addr) {
            g_hid_mouse_addr = 0u;
            g_hid_mouse_interface = 0u;
            g_hid_mouse_ep_in = 0u;
            g_hid_mouse_ep_mps = 0u;
            g_hid_mouse_vendor_id = 0u;
            g_hid_mouse_device_id = 0u;
        }
        if (state->mouse_addr != 0u) {
            g_dev_hc[state->mouse_addr] = USB_HC_NONE;
            g_dev_ep0_mps[state->mouse_addr] = 0u;
            g_dev_root_port[state->mouse_addr] = 0u;
            g_dev_route_string[state->mouse_addr] = 0u;
        }
    }

    if (state->addr != 0u) {
        g_dev_hc[state->addr] = USB_HC_NONE;
        g_dev_ep0_mps[state->addr] = 0u;
        g_dev_root_port[state->addr] = 0u;
        g_dev_route_string[state->addr] = 0u;
    }

    state->connected = false;
    state->addr = 0u;
    state->hc = USB_HC_NONE;
    state->port = 0u;
    state->vendor_id = 0u;
    state->device_id = 0u;
    state->has_keyboard = 0u;
    state->has_mouse = 0u;
    state->keyboard_addr = 0u;
    state->keyboard_interface = 0u;
    state->keyboard_endpoint = 0u;
    state->keyboard_vendor_id = 0u;
    state->keyboard_device_id = 0u;
    state->mouse_addr = 0u;
    state->mouse_interface = 0u;
    state->mouse_endpoint = 0u;
    state->mouse_vendor_id = 0u;
    state->mouse_device_id = 0u;
}

static void usb_record_root_device(usb_root_port_state_t *state,
                                   usb_hc_type_t hc,
                                   uint32_t port,
                                   uint8_t addr,
                                   const usb_device_descriptor_t *desc)
{
    if (state == NULL) {
        return;
    }

    state->connected = true;
    state->addr = addr;
    state->hc = hc;
    state->port = port;
    state->vendor_id = desc ? desc->idVendor : 0u;
    state->device_id = desc ? desc->idProduct : 0u;
    uint8_t root_port = (uint8_t)(port + 1u);
    state->has_keyboard =
        (g_hid_kbd_addr != 0u &&
         g_hid_kbd_ep_in != 0u &&
         g_dev_hc[g_hid_kbd_addr] == hc &&
         g_dev_root_port[g_hid_kbd_addr] == root_port) ? 1u : 0u;
    state->has_mouse =
        (g_hid_mouse_addr != 0u &&
         g_hid_mouse_ep_in != 0u &&
         g_dev_hc[g_hid_mouse_addr] == hc &&
         g_dev_root_port[g_hid_mouse_addr] == root_port) ? 1u : 0u;
    if (state->has_keyboard != 0u) {
        state->keyboard_addr = g_hid_kbd_addr;
        state->keyboard_interface = g_hid_kbd_interface;
        state->keyboard_endpoint = g_hid_kbd_ep_in;
        state->keyboard_vendor_id = g_hid_kbd_vendor_id;
        state->keyboard_device_id = g_hid_kbd_device_id;
    }
    if (state->has_mouse != 0u) {
        state->mouse_addr = g_hid_mouse_addr;
        state->mouse_interface = g_hid_mouse_interface;
        state->mouse_endpoint = g_hid_mouse_ep_in;
        state->mouse_vendor_id = g_hid_mouse_vendor_id;
        state->mouse_device_id = g_hid_mouse_device_id;
    }
}

static bool usb_enumerate_root_port(usb_hc_type_t hc, uint32_t port)
{
    usb_root_port_state_t *state = usb_root_port_state(hc, port);
    if (state == NULL || !usb_hc_port_valid(hc, port)) {
        return false;
    }

    if (!usb_hc_port_connected(hc, port)) {
        return false;
    }

    if (!usb_hc_reset_port(hc, port)) {
        return false;
    }

    usb_wait_ms(hc, 50);
    if (!usb_hc_port_connected(hc, port)) {
        return false;
    }

    uint8_t current_addr = usb_alloc_address();

    g_default_hc = hc;
    g_dev_hc[0] = hc;
    g_enum_parent_hub_addr = 0u;
    g_enum_parent_port = 0u;
    g_enum_speed = 0u;
    g_dev_route_string[0] = 0u;

    if (!usb_set_address(0, current_addr)) {
        return false;
    }

    usb_wait_ms(hc, 50);
    g_dev_hc[current_addr] = hc;
    g_dev_root_port[current_addr] = (uint8_t)(port + 1u);
    g_dev_route_string[current_addr] = 0u;

    if (!usb_enumerate_device(current_addr, hc, 64u, &g_usb_next_addr)) {
        g_dev_hc[current_addr] = USB_HC_NONE;
        g_dev_ep0_mps[current_addr] = 0u;
        if (hc == USB_HC_XHCI) {
            xhci_disable_slot(current_addr);
        }
        return false;
    }

    usb_record_root_device(state, hc, port, current_addr, NULL);
    return true;
}

extern bool xhci_is_ready(void);

void usb_core_init(void)
{
    usb_wait_ms(g_hc_type, 500);

    g_usb_next_addr = 1u;
    g_mass_storage_addr = 0;
    g_mass_storage_ep_in = 0;
    g_mass_storage_ep_out = 0;
    g_mass_storage_interface = 0;
    g_mass_storage_ep_in_mps = 0;
    g_mass_storage_ep_out_mps = 0;
    bot_reset_devices();

    g_hid_kbd_addr = 0;
    g_hid_kbd_interface = 0;
    g_hid_kbd_ep_in = 0;
    g_hid_kbd_ep_mps = 0;
    g_hid_kbd_vendor_id = 0;
    g_hid_kbd_device_id = 0;

    g_hid_mouse_addr = 0;
    g_hid_mouse_interface = 0;
    g_hid_mouse_ep_in = 0;
    g_hid_mouse_ep_mps = 0;
    g_hid_mouse_vendor_id = 0;
    g_hid_mouse_device_id = 0;

    for (uint32_t i = 0; i < 256u; ++i) {
        g_dev_hc[i] = USB_HC_NONE;
        g_dev_ep0_mps[i] = 8;
    }
    usb_clear_root_port_states();

    usb_hc_type_t hc_types[] = { USB_HC_XHCI, USB_HC_EHCI, USB_HC_OHCI, USB_HC_UHCI };

    for (int pass = 0; pass < 3; pass++) {
        if (pass > 0) {
            usb_wait_ms(g_hc_type, 150);
        }

        for (int t = 0; t < 4; t++) {
            usb_hc_type_t cur_hc = hc_types[t];
            uint32_t num_ports = usb_hc_port_count(cur_hc);

            if (num_ports == 0) {
                continue;
            }
            if (num_ports > USB_MAX_TRACKED_ROOT_PORTS) {
                num_ports = USB_MAX_TRACKED_ROOT_PORTS;
            }

            for (uint32_t i = 0; i < num_ports; i++) {
                usb_root_port_state_t *state = usb_root_port_state(cur_hc, i);
                if (state != NULL && state->connected) continue;
                (void)usb_enumerate_root_port(cur_hc, i);
            }
        }

        if (g_usb_hub_count != 0u) {
            usb_rescan_known_hubs(&g_usb_next_addr);
        }

        if (bot_get_device_count() != 0) break;
    }
}

static bool usb_poll_hc_hotplug(usb_hc_type_t hc)
{
    uint32_t num_ports = usb_hc_port_count(hc);
    bool changed = false;

    if (num_ports > USB_MAX_TRACKED_ROOT_PORTS) {
        num_ports = USB_MAX_TRACKED_ROOT_PORTS;
    }

    for (uint32_t port = 0u; port < num_ports; ++port) {
        usb_root_port_state_t *state = usb_root_port_state(hc, port);
        bool valid = usb_hc_port_valid(hc, port);
        bool connected = valid && usb_hc_port_connected(hc, port);

        if (state == NULL) {
            continue;
        }

        if (state->connected && !connected) {
            usb_disconnect_root_device(state);
            changed = true;
            continue;
        }

        if (!state->connected && connected) {
            if (usb_enumerate_root_port(hc, port)) {
                changed = true;
            }
        }
    }

    return changed;
}

static bool usb_poll_hotplug(void)
{
    if (!g_usb_initialized) {
        return false;
    }

    bool changed = false;
    usb_hc_type_t hc_types[] = {
        USB_HC_XHCI,
        USB_HC_EHCI,
        USB_HC_OHCI,
        USB_HC_UHCI,
    };

    for (uint32_t i = 0u; i < sizeof(hc_types) / sizeof(hc_types[0]); ++i) {
        if (usb_poll_hc_hotplug(hc_types[i])) {
            changed = true;
        }
    }

    return changed;
}

bool usb_control_transfer(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                          usb_device_request_t *req, void *data_buffer) {
    usb_hc_type_t hc = (addr == 0) ? g_default_hc : g_dev_hc[addr];
    if (hc == USB_HC_NONE) hc = g_hc_type;
    if (hc == USB_HC_OHCI)
        return ohci_submit_control(addr, endpoint, max_packet_size, req, data_buffer);
    else if (hc == USB_HC_UHCI)
        return uhci_submit_control(addr, endpoint, max_packet_size, req, data_buffer);
    else if (hc == USB_HC_EHCI)
        return ehci_submit_control(addr, endpoint, max_packet_size, req, data_buffer);
    else if (hc == USB_HC_XHCI)
        return xhci_submit_control(addr, endpoint, max_packet_size, req, data_buffer);
    return false;
}

bool usb_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                     uint8_t pid, void *data, uint32_t length) {
    usb_hc_type_t hc = g_dev_hc[addr];
    if (hc == USB_HC_NONE) hc = g_hc_type;
    if (hc == USB_HC_OHCI)
        return ohci_submit_bulk(addr, endpoint, max_packet_size, pid, data, length);
    else if (hc == USB_HC_UHCI)
        return uhci_submit_bulk(addr, endpoint, max_packet_size, pid, data, length);
    else if (hc == USB_HC_EHCI)
        return ehci_submit_bulk(addr, endpoint, max_packet_size, pid, data, length);
    else if (hc == USB_HC_XHCI)
        return xhci_submit_bulk(addr, endpoint, max_packet_size, pid, data, length);
    return false;
}

bool usb_set_address(uint8_t old_addr, uint8_t new_addr) {
    usb_device_request_t req;
    req.bmRequestType = USB_REQ_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_REQ_RCPT_DEVICE;
    req.bRequest      = USB_REQ_SET_ADDRESS;
    req.wValue        = new_addr;
    req.wIndex        = 0;
    req.wLength       = 0;
    
    usb_hc_type_t hc = (old_addr == 0) ? g_default_hc : g_dev_hc[old_addr];
    if (hc == USB_HC_NONE) hc = usb_get_hc_type();
    uint16_t mps = (old_addr == 0) ? ((hc == USB_HC_EHCI || hc == USB_HC_XHCI) ? 64 : 8) : g_dev_ep0_mps[old_addr];
    if (mps == 0) mps = 8;
    
    return usb_control_transfer(old_addr, 0, mps, &req, NULL);
}

bool usb_submit_interrupt_in_async(uint8_t addr, uint8_t ep_num,
                                    uint16_t max_packet_size,
                                    void *dma_buf, uint64_t dma_phys,
                                    uint16_t length)
{
    usb_hc_type_t hc = g_dev_hc[addr];
    if (hc == USB_HC_NONE) hc = g_hc_type;
    if (hc == USB_HC_XHCI)
        return xhci_submit_interrupt_in(addr, ep_num, max_packet_size, dma_buf, dma_phys, length);
    return false;
}

int usb_check_interrupt_event(uint8_t addr, uint8_t ep_num)
{
    usb_hc_type_t hc = g_dev_hc[addr];
    if (hc == USB_HC_NONE) hc = g_hc_type;
    if (hc == USB_HC_XHCI)
        return xhci_check_interrupt_event(addr, ep_num);
    return -1;
}

bool usb_recover_interrupt_in(uint8_t addr, uint8_t ep_num)
{
    usb_hc_type_t hc = g_dev_hc[addr];
    if (hc == USB_HC_NONE) hc = g_hc_type;
    if (hc == USB_HC_XHCI)
        return xhci_recover_interrupt_endpoint(addr, ep_num);
    return false;
}

bool usb_submit_interrupt_in_sync(uint8_t addr, uint8_t endpoint,
                                   uint16_t max_packet_size,
                                   void *data, uint16_t length)
{
    usb_hc_type_t hc = g_dev_hc[addr];
    if (hc == USB_HC_NONE) hc = g_hc_type;
    if (hc == USB_HC_EHCI)
        return ehci_submit_interrupt_in(addr, endpoint, max_packet_size, data, length);
    else if (hc == USB_HC_OHCI)
        return ohci_submit_interrupt_in(addr, endpoint, max_packet_size, data, length);
    else if (hc == USB_HC_UHCI)
        return uhci_submit_interrupt_in(addr, endpoint, max_packet_size, data, length);
    else if (hc == USB_HC_XHCI) {
        return xhci_submit_interrupt_in_sync(addr, endpoint, max_packet_size,
                                             data, length);
    }
    return false;
}

bool usb_get_device_descriptor(uint8_t addr, usb_device_descriptor_t *desc) {
    usb_device_request_t req;
    req.bmRequestType = USB_REQ_DIR_IN | USB_REQ_TYPE_STANDARD | USB_REQ_RCPT_DEVICE;
    req.bRequest      = USB_REQ_GET_DESCRIPTOR;
    req.wValue        = (USB_DESC_DEVICE << 8);
    req.wIndex        = 0;
    req.wLength       = sizeof(usb_device_descriptor_t);
    
    usb_hc_type_t hc = (addr == 0) ? g_default_hc : g_dev_hc[addr];
    if (hc == USB_HC_NONE) hc = usb_get_hc_type();
    uint16_t mps = (addr == 0) ? ((hc == USB_HC_EHCI || hc == USB_HC_XHCI) ? 64 : 8) : g_dev_ep0_mps[addr];
    if (mps == 0) mps = 8;
    
    return usb_control_transfer(addr, 0, mps, &req, desc);
}

void usb_set_hc_type(usb_hc_type_t type) {
    g_hc_type = type;
}

static void usb_master_init(void)
{
    if (g_usb_initialized) {
        return;
    }
    g_usb_initialized = true;

    if (g_api) g_api->memset(g_dev_hc, 0, sizeof(g_dev_hc));
    g_default_hc = USB_HC_NONE;

    xhci_init();
    if (xhci_is_ready()) {
        usb_set_hc_type(USB_HC_XHCI);
    }

    ehci_init();
    if (g_hc_type == USB_HC_NONE && ehci_get_num_ports() > 0) {
        usb_set_hc_type(USB_HC_EHCI);
    }

    ohci_init();
    g_ohci_ready = ohci_is_ready();

    uhci_init();
    g_uhci_ready = uhci_is_ready();

    if (g_hc_type == USB_HC_NONE && g_ohci_ready) {
        usb_set_hc_type(USB_HC_OHCI);
    }
    if (g_hc_type == USB_HC_NONE && g_uhci_ready) {
        usb_set_hc_type(USB_HC_UHCI);
    }

    usb_hid_init();
    usb_core_init();
}

static bool usb_storage_init(void)
{
    usb_master_init();

    for (uint32_t attempt = 0u; attempt < 3u; ++attempt) {
        if (attempt != 0u) {
            usb_core_init();
            usb_wait_ms(g_hc_type, 250u);
        }
        if (bot_get_device_count() != 0u) {
            usb_wait_ms(g_hc_type, 250u);
        }
        bool ok = bot_init();
        if (ok) {
            return true;
        }

        usb_wait_ms(g_hc_type, 250u);
    }

    return false;
}

static bool usb_storage_is_ready(void)
{
    return bot_get_device_count() != 0u;
}

static bool usb_storage_get_info(uint32_t device_index,
                                 driver_block_info_t *out_info)
{
    if (out_info == NULL || !bot_select_device(device_index)) {
        return false;
    }
    uint32_t block_size = bot_get_block_size();
    uint64_t total_bytes = bot_get_total_bytes();
    if (block_size == 0u || total_bytes < block_size) {
        return false;
    }
    g_api->memset(out_info, 0, sizeof(*out_info));
    out_info->block_count = total_bytes / block_size;
    out_info->logical_block_size = block_size;
    out_info->physical_block_size = block_size;
    out_info->flags = DRIVER_BLOCK_FLAG_REMOVABLE;
    if (!bot_is_read_only()) {
        out_info->flags |= DRIVER_BLOCK_FLAG_WRITABLE;
    }
    out_info->transport = DRIVER_BLOCK_TRANSPORT_USB;
    if (xhci_is_ready()) {
        out_info->identity_flags = DRIVER_BLOCK_IDENTITY_PCI_VALID;
        out_info->pci_bus = g_xhci_pci_bus;
        out_info->pci_device = g_xhci_pci_dev;
        out_info->pci_function = g_xhci_pci_func;
    }
    const char model[] = "USB Mass Storage";
    g_api->memcpy(out_info->model, model, sizeof(model));
    return true;
}

static bool usb_storage_read(uint32_t device_index, uint64_t lba,
                             void *buffer, uint32_t block_count)
{
    if (!bot_select_device(device_index)) {
        return false;
    }
    uint32_t block_size = bot_get_block_size();
    uint64_t factor = block_size / 512u;
    uint64_t sector_lba = lba * factor;
    uint64_t sectors = (uint64_t)block_count * factor;
    if (factor == 0u || sector_lba > UINT32_MAX || sectors > UINT32_MAX ||
        sector_lba + sectors > (uint64_t)UINT32_MAX + 1u) {
        return false;
    }
    return bot_read_sectors((uint32_t)sector_lba, (uint8_t *)buffer,
                            (uint32_t)sectors);
}

static bool usb_storage_write(uint32_t device_index, uint64_t lba,
                              const void *buffer, uint32_t block_count)
{
    if (!bot_select_device(device_index)) {
        return false;
    }
    uint32_t block_size = bot_get_block_size();
    uint64_t factor = block_size / 512u;
    uint64_t sector_lba = lba * factor;
    uint64_t sectors = (uint64_t)block_count * factor;
    if (factor == 0u || sector_lba > UINT32_MAX || sectors > UINT32_MAX ||
        sector_lba + sectors > (uint64_t)UINT32_MAX + 1u) {
        return false;
    }
    return bot_write_sectors((uint32_t)sector_lba,
                             (const uint8_t *)buffer, (uint32_t)sectors);
}

static bool usb_storage_flush(uint32_t device_index)
{
    return bot_select_device(device_index) && bot_flush();
}

static const usb_master_vtable_t g_usb_vtable = {
    .input = {
        .init           = usb_master_init,
        .poll           = usb_hid_poll,
        .read_keyboard  = usb_hid_read_keyboard,
        .read_mouse     = usb_hid_read_mouse,
        .drain_keyboard = usb_hid_drain_keyboard,
        .drain_mouse    = usb_hid_drain_mouse,
    },
    .storage = {
        .name             = "usb",
        .priority         = 40u,
        .init             = usb_storage_init,
        .is_ready         = usb_storage_is_ready,
        .get_device_count = bot_get_device_count,
        .get_info         = usb_storage_get_info,
        .read_blocks      = usb_storage_read,
        .write_blocks     = usb_storage_write,
        .flush            = usb_storage_flush,
    },
    .usb = {
        .submit_interrupt_in_async = usb_submit_interrupt_in_async,
        .check_interrupt_event      = usb_check_interrupt_event,
        .submit_interrupt_in_sync  = usb_submit_interrupt_in_sync,
        .poll_hotplug              = usb_poll_hotplug,
    }
};

static void usb_driver_shutdown(void)
{
    g_usb_initialized = false;
    g_hid_kbd_addr = 0;
    g_hid_kbd_interface = 0;
    g_hid_kbd_ep_in = 0;
    g_hid_kbd_ep_mps = 0;
    g_hid_kbd_vendor_id = 0;
    g_hid_kbd_device_id = 0;
    g_hid_mouse_addr = 0;
    g_hid_mouse_interface = 0;
    g_hid_mouse_ep_in = 0;
    g_hid_mouse_ep_mps = 0;
    g_hid_mouse_vendor_id = 0;
    g_hid_mouse_device_id = 0;
    g_hc_type = USB_HC_NONE;
    g_ohci_ready = false;
    g_default_hc = USB_HC_NONE;
    g_uhci_ready = false;
    for (uint32_t i = 0; i < 256u; ++i) {
        g_dev_hc[i] = USB_HC_NONE;
        g_dev_ep0_mps[i] = 0;
    }
    usb_clear_root_port_states();
    g_api = NULL;
}

static const driver_module_descriptor_t g_usb_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_USB,
    .load_priority = 30u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_usb_vtable,
    .shutdown = usb_driver_shutdown,
};

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL) {
        return NULL;
    }
    g_api = api;
    return &g_usb_module;
}
