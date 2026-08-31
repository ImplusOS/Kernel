#include "EHCI.h"
#include "Drivers/Module/DriverBinary.h"
#include "Drivers/Module/PCI_Main.h"
#include "../USB_Main.h"

extern const driver_binary_t *g_api;

static ehci_cap_regs_t *g_cap_regs = NULL;
static ehci_op_regs_t  *g_op_regs  = NULL;
static uint32_t        *g_framelist = NULL;
static ehci_qh_t *g_qh_control  = NULL;
static ehci_qh_t *g_qh_bulk_in  = NULL;
static ehci_qh_t *g_qh_bulk_out = NULL;

#define EHCI_EPS_HIGH_SPEED (2u << 12)
#define EHCI_QTD_PID_OUT    (0u << 8)
#define EHCI_QTD_PID_IN     (1u << 8)
#define EHCI_QTD_PID_SETUP  (2u << 8)
#define EHCI_QTD_CERR_3     (3u << 10)
#define EHCI_QTD_TOGGLE_1   (1u << 31)

void ehci_delay_ms(uint32_t ms) {
    if (g_api) g_api->timer_msleep(ms);
}

static inline void ehci_ring_async_doorbell(void) {
    if (!g_op_regs) return;
    g_op_regs->usbcmd |= (1u << 6);
    g_op_regs->usbsts = (1u << 5);
}

static void ehci_bios_handoff(uint8_t bus, uint8_t dev, uint8_t func) {
    if (!g_api || !g_cap_regs) return;

    uint32_t hcc = g_cap_regs->hccparams;
    uint8_t eecp = (uint8_t)((hcc >> 8) & 0xFFu);
    int guard = 48;

    while (eecp && guard-- > 0) {
        uint32_t cap = g_api->pci_read_config(bus, dev, func, eecp);
        uint8_t cap_id = (uint8_t)(cap & 0xFFu);
        uint8_t next   = (uint8_t)((cap >> 8) & 0xFFu);

        if (cap_id == 1u) {
            uint32_t legsup = cap;

            if ((legsup & (1u << 24)) == 0) {
                g_api->pci_write_config(bus, dev, func, eecp, legsup | (1u << 24));
            }

            for (int i = 0; i < 1000; i++) {
                legsup = g_api->pci_read_config(bus, dev, func, eecp);
                if ((legsup & (1u << 16)) == 0) break;
                ehci_delay_ms(1);
            }

            uint32_t legctl = g_api->pci_read_config(bus, dev, func, eecp + 4);
            uint32_t new_legctl = (legctl & ~0x7Fu) | (legctl & 0x00FF0000u);
            g_api->pci_write_config(bus, dev, func, eecp + 4, new_legctl);
            return;
        }
        eecp = next;
    }
}

void ehci_init(void) {
    pci_device_t ehci_dev;
    bool found = false;
    for (uint16_t b = 0; b < 256; b++) {
        for (uint8_t d = 0; d < 32; d++) {
            for (uint8_t f = 0; f < 8; f++) {
                uint32_t class_reg = g_api->pci_read_config((uint8_t)b, d, f, 0x08);
                uint8_t class_code = (uint8_t)((class_reg >> 24) & 0xFFu);
                uint8_t subclass   = (uint8_t)((class_reg >> 16) & 0xFFu);
                uint8_t prog_if    = (uint8_t)((class_reg >>  8) & 0xFFu);
                if (class_code == 0x0C && subclass == 0x03 && prog_if == 0x20) {
                    ehci_dev.bus    = (uint8_t)b;
                    ehci_dev.device = d;
                    ehci_dev.func   = f;
                    found = true;
                    goto pci_done;
                }
            }
        }
    }
pci_done:
    if (!found) {
        return;
    }

    uint32_t cmd = g_api->pci_read_config(ehci_dev.bus, ehci_dev.device, ehci_dev.func, 0x04);
    cmd |= (1 << 1) | (1 << 2);
    g_api->pci_write_config(ehci_dev.bus, ehci_dev.device, ehci_dev.func, 0x04, cmd);

    uint32_t bar0_raw = g_api->pci_read_config(ehci_dev.bus, ehci_dev.device, ehci_dev.func, 0x10);
    uint64_t phys_addr;
    if ((bar0_raw & 0x1u) == 0 && (bar0_raw & 0x6u) == 0x4u) {
        uint32_t bar1 = g_api->pci_read_config(ehci_dev.bus, ehci_dev.device, ehci_dev.func, 0x14);
        phys_addr = ((uint64_t)bar1 << 32) | (bar0_raw & ~0xFu);
    } else {
        phys_addr = bar0_raw & ~0xFu;
    }

    if (phys_addr == 0 || phys_addr >= 0x10000000000ULL) {
        return;
    }

    g_cap_regs = (ehci_cap_regs_t *)g_api->map_mmio_virt(phys_addr);
    if (!g_cap_regs) {
        return;
    }

    g_op_regs = (ehci_op_regs_t *)((uint8_t *)g_cap_regs + g_cap_regs->caplength);

    ehci_bios_handoff(ehci_dev.bus, ehci_dev.device, ehci_dev.func);

    uint32_t usb2pr = g_api->pci_read_config(ehci_dev.bus, ehci_dev.device, ehci_dev.func, 0xD0);
    uint32_t port_mask = (g_cap_regs->hcsparams & 0xFu);
    if (port_mask >= 32) port_mask = 32;
    if (port_mask) {
        uint32_t mask = (port_mask == 32) ? 0xFFFFFFFFu : ((1u << port_mask) - 1u);
        usb2pr |= mask;
        g_api->pci_write_config(ehci_dev.bus, ehci_dev.device, ehci_dev.func, 0xD0, usb2pr);
    }

    if (g_cap_regs->hcsparams & (1u << 4)) {
        uint32_t ports = g_cap_regs->hcsparams & 0xFu;
        for (uint32_t i = 0; i < ports; i++) {
            uint32_t p = g_op_regs->portsc[i];
            if (!(p & (1u << 12))) {
                g_op_regs->portsc[i] = p | (1u << 12);
                ehci_delay_ms(1);
            }
        }
    }

    g_op_regs->usbcmd &= ~(1u);
    {
        uint32_t n;
        for (n = 0; n < 10000u && !(g_op_regs->usbsts & (1u << 12)); n++) {
            ehci_delay_ms(1);
        }
        if (!(g_op_regs->usbsts & (1u << 12))) {
            return;
        }
    }

    g_op_regs->usbcmd |= 2u;
    {
        uint32_t n;
        for (n = 0; n < 10000u && (g_op_regs->usbcmd & 2u); n++) {
            ehci_delay_ms(1);
        }
        if (g_op_regs->usbcmd & 2u) {
            return;
        }
    }

    uint64_t framelist_phys = 0;
    g_framelist = g_api->dma_alloc(4096, &framelist_phys);
    for (int i = 0; i < 1024; i++) g_framelist[i] = 1;

    g_op_regs->ctrldssegment  = 0;
    g_op_regs->periodiclistbase = (uint32_t)framelist_phys;

    g_op_regs->usbcmd = (g_op_regs->usbcmd & 0xFF00FFFFu) | (8u << 16) | 1u;
    {
        uint32_t n;
        for (n = 0; n < 10000u && (g_op_regs->usbsts & (1u << 12)); n++) {
            ehci_delay_ms(1);
        }
        if (g_op_regs->usbsts & (1u << 12)) {
            return;
        }
    }

    if (!g_qh_control) {
        uint64_t phys = 0;
        g_qh_control = g_api->dma_alloc(4096, &phys);
        g_qh_bulk_in  = g_qh_control + 1;
        g_qh_bulk_out = g_qh_bulk_in  + 1;

        uint32_t p_ctrl = (uint32_t)phys;
        uint32_t p_in   = (uint32_t)phys + sizeof(ehci_qh_t);
        uint32_t p_out  = (uint32_t)phys + 2 * sizeof(ehci_qh_t);

        g_qh_control->horiz_link  = p_in | 2u;
        g_qh_control->ep_chars    = (1u << 15) | (1u << 14) | EHCI_EPS_HIGH_SPEED;
        g_qh_control->ep_caps     = 0;
        g_qh_control->current_qtd = 1;
        g_qh_control->next_qtd    = 1;
        g_qh_control->alt_next_qtd= 1;
        g_qh_control->token       = 0;
        for (int i = 0; i < 5; i++) g_qh_control->buffer[i] = 0;

        g_qh_bulk_in->horiz_link  = p_out | 2u;
        g_qh_bulk_in->ep_chars    = EHCI_EPS_HIGH_SPEED;
        g_qh_bulk_in->ep_caps     = 0;
        g_qh_bulk_in->current_qtd = 0;
        g_qh_bulk_in->next_qtd    = 1;
        g_qh_bulk_in->alt_next_qtd= 1;
        g_qh_bulk_in->token       = 0;
        for (int i = 0; i < 5; i++) g_qh_bulk_in->buffer[i] = 0;

        g_qh_bulk_out->horiz_link  = p_ctrl | 2u;
        g_qh_bulk_out->ep_chars    = EHCI_EPS_HIGH_SPEED;
        g_qh_bulk_out->ep_caps     = 0;
        g_qh_bulk_out->current_qtd = 0;
        g_qh_bulk_out->next_qtd    = 1;
        g_qh_bulk_out->alt_next_qtd= 1;
        g_qh_bulk_out->token       = 0;
        for (int i = 0; i < 5; i++) g_qh_bulk_out->buffer[i] = 0;

        g_op_regs->asynclistaddr = p_ctrl;
        g_op_regs->usbcmd |= (1u << 5);
        for (uint32_t n = 0; n < 10000u; n++) {
            if (g_op_regs->usbsts & (1u << 15)) break;
            ehci_delay_ms(1);
        }
    }

    g_op_regs->configflag = 1;
}

uint32_t ehci_get_num_ports(void) {
    if (!g_cap_regs) return 0;
    return g_cap_regs->hcsparams & 0xFu;
}

bool ehci_reset_port(uint32_t port) {
    if (!g_op_regs) return false;
    if (!ehci_port_valid(port)) return false;
    uint32_t psc = g_op_regs->portsc[port];
    if ((psc & 1u) == 0) return false;

    const uint32_t RW1C_MASK = (1u << 1) | (1u << 3) | (1u << 5);
    #define PORTSC_WRITE(port, val) \
        g_op_regs->portsc[(port)] = (val) & ~RW1C_MASK

    g_op_regs->portsc[port] = (psc & ~RW1C_MASK) | RW1C_MASK;

    psc = g_op_regs->portsc[port];
    PORTSC_WRITE(port, psc | (1u << 8));
    ehci_delay_ms(50);

    psc = g_op_regs->portsc[port];
    PORTSC_WRITE(port, psc & ~(1u << 8));

    for (uint32_t n = 0; n < 500; n++) {
        if (!(g_op_regs->portsc[port] & (1u << 8))) break;
        ehci_delay_ms(1);
    }

    psc = g_op_regs->portsc[port];
    g_op_regs->portsc[port] = (psc & ~RW1C_MASK) | RW1C_MASK;

    ehci_delay_ms(10);

    uint32_t wait_en_ms = 100;
    while (wait_en_ms--) {
        psc = g_op_regs->portsc[port];
        if (psc & (1u << 2)) break;
        ehci_delay_ms(1);
    }

    uint32_t final_psc = g_op_regs->portsc[port];

    if (final_psc & (1u << 13)) {
        return false;
    }

    if (!(final_psc & (1u << 2))) {
        PORTSC_WRITE(port, final_psc | (1u << 13));
        return false;
    }

    #undef PORTSC_WRITE
    return true;
}

bool ehci_port_connected(uint32_t port) {
    if (!g_op_regs) return false;
    uint32_t p = g_op_regs->portsc[port];
    if (p & (1u << 13)) return false;
    return (p & 1u) != 0;
}

bool ehci_port_valid(uint32_t port) {
    if (!g_op_regs) return false;
    uint32_t v = g_op_regs->portsc[port];
    return v != 0 && v != 0xFFFFFFFFu;
}

bool ehci_submit_control(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                         struct usb_device_request *req, void *data) {
    if (!g_qh_control) return false;

    ehci_qtd_t *setup_qtd  = (ehci_qtd_t *)(g_qh_bulk_out + 1);
    ehci_qtd_t *data_qtd   = setup_qtd + 1;
    ehci_qtd_t *status_qtd = data_qtd  + 1;
    usb_device_request_t *req_buf = (usb_device_request_t *)(status_qtd + 1);

    uint64_t setup_phys  = g_api->virt_to_phys(setup_qtd);
    uint64_t data_phys   = g_api->virt_to_phys(data_qtd);
    uint64_t status_phys = g_api->virt_to_phys(status_qtd);
    uint64_t req_phys    = g_api->virt_to_phys(req_buf);

    *req_buf = *(usb_device_request_t *)req;

    g_qh_control->ep_chars = ((uint32_t)max_packet_size << 16) |
                              (1u << 15) | (1u << 14) |
                              EHCI_EPS_HIGH_SPEED |
                              ((uint32_t)endpoint << 8) | addr;
    g_qh_control->ep_caps  = (1u << 30);
    g_qh_control->token    = 0;

    bool has_data = (((usb_device_request_t *)req)->wLength > 0 && data != NULL);

    setup_qtd->next_qtd    = has_data ? (uint32_t)data_phys : (uint32_t)status_phys;
    setup_qtd->alt_next_qtd= 1;
    setup_qtd->token       = (8u << 16) | EHCI_QTD_CERR_3 | EHCI_QTD_PID_SETUP | 0x80u;
    setup_qtd->buffer[0]   = (uint32_t)req_phys;
    for (int i = 1; i < 5; i++) setup_qtd->buffer[i] = 0;

    if (has_data) {
        uint64_t d_phys = g_api->virt_to_phys(data);
        data_qtd->next_qtd    = (uint32_t)status_phys;
        data_qtd->alt_next_qtd= 1;
        uint8_t pid = (((usb_device_request_t *)req)->bmRequestType & 0x80) ? 1 : 0;
        data_qtd->token       = ((uint32_t)(((usb_device_request_t *)req)->wLength) << 16) |
                                 EHCI_QTD_CERR_3 |
                                 (pid ? EHCI_QTD_PID_IN : EHCI_QTD_PID_OUT) |
                                 EHCI_QTD_TOGGLE_1 | 0x80u;
        data_qtd->buffer[0]   = (uint32_t)d_phys;
        data_qtd->buffer[1]   = (uint32_t)((d_phys & ~0xFFFULL) + 0x1000ULL);
        data_qtd->buffer[2]   = (uint32_t)((d_phys & ~0xFFFULL) + 0x2000ULL);
        data_qtd->buffer[3]   = (uint32_t)((d_phys & ~0xFFFULL) + 0x3000ULL);
        data_qtd->buffer[4]   = (uint32_t)((d_phys & ~0xFFFULL) + 0x4000ULL);
    }

    uint8_t status_pid = (((usb_device_request_t *)req)->bmRequestType & 0x80) ? 0 : 1;
    status_qtd->next_qtd    = 1;
    status_qtd->alt_next_qtd= 1;
    status_qtd->token       = EHCI_QTD_CERR_3 |
                               (status_pid ? EHCI_QTD_PID_IN : EHCI_QTD_PID_OUT) |
                               EHCI_QTD_TOGGLE_1 | 0x80u;
    status_qtd->buffer[0]   = 0;
    for (int i = 1; i < 5; i++) status_qtd->buffer[i] = 0;

    g_qh_control->next_qtd = (uint32_t)setup_phys;
    __asm__ volatile("" ::: "memory");

    ehci_ring_async_doorbell();

    uint32_t timeout_ms = 5000;
    while (status_qtd->token & 0x80u) {
        ehci_delay_ms(1);
        if (timeout_ms-- == 0) {
            return false;
        }
    }

    if (status_qtd->token & 0x7Cu) {
        return false;
    }
    return true;
}

bool ehci_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                      uint8_t pid, void *data, uint32_t length) {
    if (!g_qh_control) return false;

    ehci_qh_t  *qh       = (pid == 1) ? g_qh_bulk_in : g_qh_bulk_out;
    ehci_qtd_t *data_qtd = (ehci_qtd_t *)(g_qh_bulk_out + 1);

    if ((qh->ep_chars & 0xFFu) != addr ||
        ((qh->ep_chars >> 8) & 0xFu) != endpoint) {
        qh->ep_chars = ((uint32_t)max_packet_size << 16) |
                       (1u << 14) |
                       EHCI_EPS_HIGH_SPEED |
                       ((uint32_t)endpoint << 8) | addr;
        qh->token    = 0;
    }
    qh->ep_caps = (1u << 30);

    uint64_t dma_phys = 0;
    void *dma_buf = NULL;
    if (length > 0) {
        dma_buf = g_api->dma_alloc(length, &dma_phys);
        if (!dma_buf) return false;
        if (pid == 0 && data) {
            g_api->memcpy(dma_buf, data, length);
        }
    }

    uint64_t data_qtd_phys = g_api->virt_to_phys(data_qtd);

    data_qtd->next_qtd    = 1;
    data_qtd->alt_next_qtd= 1;
    data_qtd->token       = (length << 16) |
                             EHCI_QTD_CERR_3 |
                             (pid ? EHCI_QTD_PID_IN : EHCI_QTD_PID_OUT) |
                             EHCI_QTD_TOGGLE_1 | 0x80u;

    if (length > 0) {
        data_qtd->buffer[0]   = (uint32_t)dma_phys;
        data_qtd->buffer[1]   = (uint32_t)((dma_phys & ~0xFFFULL) + 0x1000ULL);
        data_qtd->buffer[2]   = (uint32_t)((dma_phys & ~0xFFFULL) + 0x2000ULL);
        data_qtd->buffer[3]   = (uint32_t)((dma_phys & ~0xFFFULL) + 0x3000ULL);
        data_qtd->buffer[4]   = (uint32_t)((dma_phys & ~0xFFFULL) + 0x4000ULL);
    } else {
        for (int i = 0; i < 5; i++) data_qtd->buffer[i] = 0;
    }

    qh->token &= ~0xFFu;

    qh->next_qtd = (uint32_t)data_qtd_phys;
    __asm__ volatile("" ::: "memory");

    ehci_ring_async_doorbell();

    uint32_t timeout_ms = 5000;
    bool success = true;
    while (data_qtd->token & 0x80u) {
        ehci_delay_ms(1);
        if (timeout_ms-- == 0) {
            success = false;
            break;
        }
    }

    if (success && (data_qtd->token & 0x7Cu)) {
        success = false;
    }

    if (success && length > 0 && pid == 1 && data) {
        g_api->memcpy(data, dma_buf, length);
    }
    
    if (dma_buf && g_api->dma_free) {
        g_api->dma_free(dma_buf, length);
    }

    return success;
}

bool ehci_submit_interrupt_in(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                              void *data, uint16_t length) {
    if (!g_qh_control || !g_api || length == 0 || !data) return false;

    ehci_qh_t  *qh      = g_qh_bulk_in;
    ehci_qtd_t *in_qtd   = (ehci_qtd_t *)(g_qh_bulk_out + 1);

    qh->ep_chars = ((uint32_t)max_packet_size << 16) |
                   (1u << 14) |
                   EHCI_EPS_HIGH_SPEED |
                   ((uint32_t)endpoint << 8) | addr;
    qh->ep_caps  = (1u << 30);
    qh->token    = 0;

    uint64_t dma_phys = 0;
    void *dma_buf = g_api->dma_alloc(length, &dma_phys);
    if (!dma_buf) return false;
    g_api->memset(dma_buf, 0, length);

    uint64_t in_qtd_phys = g_api->virt_to_phys(in_qtd);

    in_qtd->next_qtd     = 1;
    in_qtd->alt_next_qtd = 1;
    in_qtd->token        = ((uint32_t)length << 16) |
                           EHCI_QTD_CERR_3 |
                           EHCI_QTD_PID_IN |
                           EHCI_QTD_TOGGLE_1 | 0x80u;
    in_qtd->buffer[0]    = (uint32_t)dma_phys;
    in_qtd->buffer[1]    = (uint32_t)((dma_phys & ~0xFFFULL) + 0x1000ULL);
    for (int i = 2; i < 5; i++) in_qtd->buffer[i] = 0;

    qh->token &= ~0xFFu;
    qh->next_qtd = (uint32_t)in_qtd_phys;
    __asm__ volatile("" ::: "memory");

    ehci_ring_async_doorbell();

    uint32_t timeout_ms = 2;
    bool success = true;
    while (in_qtd->token & 0x80u) {
        ehci_delay_ms(1);
        if (timeout_ms-- == 0) {
            success = false;
            break;
        }
    }

    if (!success) {
        qh->next_qtd = 1;
        __asm__ volatile("" ::: "memory");
        in_qtd->token &= ~0x80u;
    } else if (in_qtd->token & 0x7Cu) {
        success = false;
    }

    if (success) {
        g_api->memcpy(data, dma_buf, length);
    }

    if (g_api->dma_free) {
        g_api->dma_free(dma_buf, length);
    }

    return success;
}