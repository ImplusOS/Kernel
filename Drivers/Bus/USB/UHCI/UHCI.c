#include "UHCI.h"
#include "Drivers/Module/DriverBinary.h"
#include "../USB_Main.h"
#include <stddef.h>

extern const driver_binary_t *g_api;

static uint16_t g_io_base = 0;
static bool     g_ready   = false;
static uint32_t *g_frame_list = NULL;
static uint64_t  g_frame_phys = 0;
static uhci_qh_t *g_qh_ctrl = NULL;
static uhci_qh_t *g_qh_bulk = NULL;
static uint64_t   g_qh_phys = 0;
static uhci_td_t *g_td_pool = NULL;

#define UHCI_REG_USBCMD    0x00
#define UHCI_REG_USBSTS    0x02
#define UHCI_REG_USBINTR   0x04
#define UHCI_REG_FRNUM     0x06
#define UHCI_REG_FRBASEADD 0x08
#define UHCI_REG_SOFMOD    0x0C
#define UHCI_REG_PORTSC1   0x10

static inline uint8_t io_inb(uint16_t port) { return g_api->inb(port); }
static inline void io_outb(uint16_t port, uint8_t v) { g_api->outb(port, v); }
static uint16_t io_inw(uint16_t port) {
    uint8_t lo = io_inb(port);
    uint8_t hi = io_inb(port + 1);
    return (uint16_t)(lo | ((uint16_t)hi << 8));
}
static void io_outw(uint16_t port, uint16_t v) {
    io_outb(port, (uint8_t)(v & 0xFFu));
    io_outb(port + 1, (uint8_t)((v >> 8) & 0xFFu));
}

static void uhci_delay_ms(uint32_t ms) { if (g_api) g_api->timer_msleep(ms); }

bool uhci_is_ready(void) { return g_ready; }

static uint32_t phys32(void *p) { return (uint32_t)g_api->virt_to_phys(p); }

bool uhci_port_valid(uint32_t port) {
    if (!g_ready) return false;
    if (port >= uhci_get_num_ports()) return false;
    return true;
}

static void uhci_global_reset(void) {
    uint16_t cmd = io_inw(g_io_base + UHCI_REG_USBCMD);
    cmd |= (1u << 1);
    io_outw(g_io_base + UHCI_REG_USBCMD, cmd);
    for (int i = 0; i < 1000; i++) {
        if ((io_inw(g_io_base + UHCI_REG_USBCMD) & (1u << 1)) == 0) break;
        uhci_delay_ms(1);
    }
}

static void uhci_stop(void) {
    uint16_t cmd = io_inw(g_io_base + UHCI_REG_USBCMD);
    cmd &= (uint16_t)~(1u << 0);
    io_outw(g_io_base + UHCI_REG_USBCMD, cmd);
}

static bool uhci_td_active(uhci_td_t *td) { return (td->ctrl_status & UHCI_TD_CTRL_ACTIVE) != 0; }

static bool uhci_wait_td(uhci_td_t *td, uint32_t timeout_ms) {
    while (uhci_td_active(td) && timeout_ms--) {
        uhci_delay_ms(1);
    }
    return !uhci_td_active(td);
}

void uhci_init(void) {
    g_ready = false;
    uint8_t uhci_bus = 0, uhci_dev = 0, uhci_fn = 0;
    bool found = false;
    for (uint16_t b = 0; b < 256 && !found; b++) {
        for (uint8_t d = 0; d < 32 && !found; d++) {
            for (uint8_t f = 0; f < 8 && !found; f++) {
                uint32_t id = g_api->pci_read_config((uint8_t)b, d, f, 0x00);
                if ((id & 0xFFFF) == 0xFFFF) { if (f == 0) break; continue; }
                uint32_t cls = g_api->pci_read_config((uint8_t)b, d, f, 0x08);
                if (((cls >> 24) & 0xFF) == 0x0C && ((cls >> 16) & 0xFF) == 0x03 && ((cls >> 8) & 0xFF) == 0x00) {
                    uhci_bus = (uint8_t)b; uhci_dev = d; uhci_fn = f; found = true;
                }
                if (f == 0) {
                    uint32_t hdr = g_api->pci_read_config((uint8_t)b, d, f, 0x0C);
                    if (((hdr >> 16) & 0x80) == 0) break;
                }
            }
        }
    }
    if (!found) {
        return;
    }

    uint32_t cmd = g_api->pci_read_config(uhci_bus, uhci_dev, uhci_fn, 0x04);
    cmd |= (1u << 0) | (1u << 2);
    g_api->pci_write_config(uhci_bus, uhci_dev, uhci_fn, 0x04, cmd);

    uint32_t bar4 = g_api->pci_read_config(uhci_bus, uhci_dev, uhci_fn, 0x20);
    if ((bar4 & 0x1) == 0) {
        return;
    }
    g_io_base = (uint16_t)(bar4 & ~0x1Fu);
    if (g_io_base == 0) {
        return;
    }

    uhci_stop();
    uhci_global_reset();

    g_frame_list = (uint32_t *)g_api->dma_alloc(4096, &g_frame_phys);
    if (!g_frame_list) return;
    for (int i = 0; i < UHCI_NUM_FRAMES; i++) g_frame_list[i] = 0x00000001;

    g_qh_ctrl = (uhci_qh_t *)g_api->dma_alloc(4096, &g_qh_phys);
    if (!g_qh_ctrl) return;
    g_qh_bulk = g_qh_ctrl + 1;
    g_td_pool = (uhci_td_t *)(g_qh_bulk + 1);

    uint64_t qh_ctrl_phys = g_qh_phys;
    uint64_t qh_bulk_phys = qh_ctrl_phys + sizeof(uhci_qh_t);

    g_qh_ctrl->head_link = (uint32_t)qh_bulk_phys | 0x00000002;
    g_qh_ctrl->elem_link = 0x00000001;

    g_qh_bulk->head_link = 0x00000001;
    g_qh_bulk->elem_link = 0x00000001;

    for (int i = 0; i < UHCI_NUM_FRAMES; i++) {
        g_frame_list[i] = (uint32_t)qh_ctrl_phys | 0x00000002;
    }

    io_outw(g_io_base + UHCI_REG_FRNUM, 0);
    g_api->outl(g_io_base + UHCI_REG_FRBASEADD, (uint32_t)g_frame_phys);
    io_outb(g_io_base + UHCI_REG_SOFMOD, 0x40);
    io_outw(g_io_base + UHCI_REG_USBINTR, 0);

    uint16_t usbcmd = (1u << 7) | (1u << 6) | (1u << 0);
    io_outw(g_io_base + UHCI_REG_USBCMD, usbcmd);

    g_ready = true;
}

uint32_t uhci_get_num_ports(void) {
    return g_ready ? 2u : 0u;
}

bool uhci_reset_port(uint32_t port) {
    if (!g_ready || !uhci_port_valid(port)) return false;
    uint16_t portsc = (uint16_t)(g_io_base + UHCI_REG_PORTSC1 + (port * 2));
    uint16_t v = 0;
    
    for (int i = 0; i < 50; i++) {
        v = io_inw(portsc);
        if (v & 1u) break;
        uhci_delay_ms(1);
    }
    
    if ((v & 1u) == 0) return false;

    io_outw(portsc, v | (1u << 9));
    uhci_delay_ms(50);
    v = io_inw(portsc);
    v &= (uint16_t)~(1u << 9);
    io_outw(portsc, v);

    for (int i = 0; i < 100; i++) {
        if ((io_inw(portsc) & (1u << 9)) == 0) break;
        uhci_delay_ms(1);
    }

    for (int i = 0; i < 100; i++) {
        v = io_inw(portsc);
        if (v & (1u << 2)) return true;
        uhci_delay_ms(1);
    }
    return false;
}

bool uhci_port_connected(uint32_t port) {
    if (!g_ready || !uhci_port_valid(port)) return false;
    uint16_t portsc = (uint16_t)(g_io_base + UHCI_REG_PORTSC1 + (port * 2));
    return (io_inw(portsc) & 1u) != 0;
}

static uhci_td_t *alloc_td(uint32_t index) { return g_td_pool + index; }

static void td_fill(uhci_td_t *td, uint32_t link, uint32_t ctrl, uint32_t token, uint32_t buf_phys) {
    td->link = link;
    td->ctrl_status = ctrl;
    td->token = token;
    td->buffer[0] = buf_phys;
    for (int i = 1; i < 5; i++) td->buffer[i] = 0;
}

static bool uhci_control_xfer(uint8_t addr, uint8_t endpoint, uint16_t mps,
                              struct usb_device_request *req, void *data) {
    (void)mps;
    bool dir_in = (req->bmRequestType & 0x80) != 0;
    uint16_t wlen = req->wLength;

    uhci_td_t *td_setup  = alloc_td(0);
    uhci_td_t *td_data   = alloc_td(1);
    uhci_td_t *td_status = alloc_td(2);

    uint32_t phys_setup  = phys32(td_setup);
    uint32_t phys_data   = phys32(td_data);
    uint32_t phys_status = phys32(td_status);

    uint64_t req_phys = g_api->virt_to_phys(req);

    uint32_t token_setup = (UHCI_PID_SETUP) |
                           ((uint32_t)addr << 8) |
                           ((uint32_t)endpoint << 15) |
                           (0u << 19) |
                           ((uint32_t)(8 - 1) << 21);
    uint32_t ctrl_setup = UHCI_TD_CTRL_ACTIVE | UHCI_TD_CTRL_SPD | (3u << 27);
    td_fill(td_setup, (wlen ? phys_data : phys_status), ctrl_setup, token_setup, (uint32_t)req_phys);

    if (wlen) {
        uint64_t data_phys_buf = g_api->virt_to_phys(data);
        uint32_t token_data = (dir_in ? UHCI_PID_IN : UHCI_PID_OUT) |
                              ((uint32_t)addr << 8) |
                              ((uint32_t)endpoint << 15) |
                              (1u << 19) |
                              ((uint32_t)(wlen - 1) << 21);
        uint32_t ctrl_data = UHCI_TD_CTRL_ACTIVE | UHCI_TD_CTRL_SPD | (3u << 27);
        td_fill(td_data, phys_status, ctrl_data, token_data, (uint32_t)data_phys_buf);
    } else {
        td_data->link = 0x00000001;
    }

    uint32_t token_status = (dir_in ? UHCI_PID_OUT : UHCI_PID_IN) |
                            ((uint32_t)addr << 8) |
                            ((uint32_t)endpoint << 15) |
                            (1u << 19) |
                            ((uint32_t)(0x7FF) << 21);
    uint32_t ctrl_status = UHCI_TD_CTRL_ACTIVE | UHCI_TD_CTRL_SPD | (3u << 27);
    td_fill(td_status, 0x00000001, ctrl_status, token_status, 0);

    g_qh_ctrl->elem_link = phys_setup;

    bool ok = uhci_wait_td(td_status, 5000);
    if (ok && (td_status->ctrl_status & (UHCI_TD_CTRL_STALLED | UHCI_TD_CTRL_BABBLE | UHCI_TD_CTRL_CRCTIMEO))) ok = false;
    g_qh_ctrl->elem_link = 0x00000001;
    return ok;
}

static bool uhci_bulk_xfer(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                           uint8_t pid, void *data, uint32_t length) {
    (void)max_packet_size;
    uhci_td_t *td = alloc_td(3);
    uint32_t token = (pid ? UHCI_PID_IN : UHCI_PID_OUT) |
                     ((uint32_t)addr << 8) |
                     ((uint32_t)endpoint << 15) |
                     (1u << 19) |
                     ((length ? (length - 1) : 0x7FFu) << 21);
    uint64_t buf_phys = g_api->virt_to_phys(data);
    uint32_t ctrl = UHCI_TD_CTRL_ACTIVE | UHCI_TD_CTRL_SPD | (3u << 27);
    td_fill(td, 0x00000001, ctrl, token, (uint32_t)buf_phys);

    g_qh_bulk->elem_link = phys32(td);

    bool ok = uhci_wait_td(td, 5000);
    if (ok && (td->ctrl_status & (UHCI_TD_CTRL_STALLED | UHCI_TD_CTRL_BABBLE | UHCI_TD_CTRL_CRCTIMEO))) ok = false;
    g_qh_bulk->elem_link = 0x00000001;
    return ok;
}

bool uhci_submit_control(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                         struct usb_device_request *req, void *data) {
    (void)endpoint; (void)max_packet_size;
    if (!g_ready) return false;
    return uhci_control_xfer(addr, 0, max_packet_size, req, data);
}

bool uhci_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                      uint8_t pid, void *data, uint32_t length) {
    (void)max_packet_size;
    if (!g_ready) return false;
    return uhci_bulk_xfer(addr, endpoint, max_packet_size, pid, data, length);
}

bool uhci_submit_interrupt_in(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                              void *data, uint16_t length) {
    (void)max_packet_size;
    if (!g_ready || length == 0 || !data) return false;

    uhci_td_t *td = alloc_td(5);
    uint32_t token = UHCI_PID_IN |
                     ((uint32_t)addr << 8) |
                     ((uint32_t)endpoint << 15) |
                     (1u << 19) |
                     ((uint32_t)(length - 1) << 21);
    uint64_t buf_phys = g_api->virt_to_phys(data);
    uint32_t ctrl = UHCI_TD_CTRL_ACTIVE | UHCI_TD_CTRL_SPD | (3u << 27);
    td_fill(td, 0x00000001, ctrl, token, (uint32_t)buf_phys);

    g_qh_bulk->elem_link = phys32(td);

    bool ok = uhci_wait_td(td, 2);
    if (!ok) {
        td->ctrl_status &= ~UHCI_TD_CTRL_ACTIVE;
    } else if (td->ctrl_status & (UHCI_TD_CTRL_STALLED | UHCI_TD_CTRL_BABBLE | UHCI_TD_CTRL_CRCTIMEO)) {
        ok = false;
    }
    g_qh_bulk->elem_link = 0x00000001;
    return ok;
}