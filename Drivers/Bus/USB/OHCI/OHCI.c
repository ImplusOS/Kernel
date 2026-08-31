#include "OHCI.h"
#include "Drivers/Module/DriverBinary.h"
#include "../USB_Main.h"
#include <stddef.h>

extern const driver_binary_t *g_api;

static volatile uint32_t *g_regs = NULL;
static ohci_hcca_t *g_hcca = NULL;      
static uint64_t     g_hcca_phys = 0;
static ohci_ed_t   *g_ed_ctrl = NULL;   
static ohci_ed_t   *g_ed_bulk = NULL;   
static ohci_td_t   *g_td_pool = NULL;   
static uint64_t     g_ed_phys = 0;
static bool         g_ready = false;

#define OHCI_REG_REVISION     0x00
#define OHCI_REG_CONTROL      0x04
#define OHCI_REG_CMD_STATUS   0x08
#define OHCI_REG_INT_ENABLE   0x10
#define OHCI_REG_HCCA         0x18
#define OHCI_REG_CTRL_HEAD    0x20
#define OHCI_REG_BULK_HEAD    0x28
#define OHCI_REG_FM_INTERVAL  0x34
#define OHCI_REG_PERIOD_START 0x40
#define OHCI_REG_RH_DESC      0x48
#define OHCI_REG_RH_STATUS    0x4C
#define OHCI_REG_RH_PORT1     0x50

static inline uint32_t mmio_read(uint32_t off)  { return g_regs[off/4]; }
static inline void     mmio_write(uint32_t off, uint32_t v) { g_regs[off/4] = v; }

static void ohci_delay_ms(uint32_t ms) { if (g_api) g_api->timer_msleep(ms); }

bool ohci_is_ready(void) { return g_ready; }

static void ohci_reset(void) {
    mmio_write(OHCI_REG_CMD_STATUS, 1u << 0);
    for (int i = 0; i < 100; i++) {
        if ((mmio_read(OHCI_REG_CMD_STATUS) & 1u) == 0) break;
        ohci_delay_ms(1);
    }
}

void ohci_init(void) {
    g_ready = false;
    g_regs = NULL;

    uint8_t bus=0, dev=0, fn=0; bool found=false;
    for (uint16_t b=0; b<256 && !found; b++) {
        for (uint8_t d=0; d<32 && !found; d++) {
            for (uint8_t f=0; f<8 && !found; f++) {
                uint32_t id = g_api->pci_read_config((uint8_t)b,d,f,0x00);
                if ((id & 0xFFFF)==0xFFFF) { if (f==0) break; continue; }
                uint32_t cls = g_api->pci_read_config((uint8_t)b,d,f,0x08);
                if (((cls>>24)&0xFF)==0x0C && ((cls>>16)&0xFF)==0x03 && ((cls>>8)&0xFF)==0x10) {
                    bus=(uint8_t)b; dev=d; fn=f; found=true; break;
                }
                if (f==0) {
                    uint32_t hdr=g_api->pci_read_config((uint8_t)b,d,f,0x0C);
                    if (((hdr>>16)&0x80)==0) break;
                }
            }
        }
    }
    if (!found) {
        return;
    }

    uint32_t cmd = g_api->pci_read_config(bus,dev,fn,0x04);
    cmd |= (1u<<1) | (1u<<2);
    g_api->pci_write_config(bus,dev,fn,0x04,cmd);

    uint32_t bar = g_api->pci_read_config(bus,dev,fn,0x10);
    uint64_t phys = (uint64_t)(bar & ~0xFu);
    if (phys == 0 || phys >= 0x10000000000ULL) {
        return;
    }
    g_regs = (uint32_t *)g_api->map_mmio_virt(phys);
    if (!g_regs) return;

    ohci_reset();

    g_hcca = (ohci_hcca_t *)g_api->dma_alloc(sizeof(ohci_hcca_t), &g_hcca_phys);
    g_ed_ctrl = (ohci_ed_t *)g_api->dma_alloc(4096, &g_ed_phys);
    if (!g_hcca || !g_ed_ctrl) return;
    g_ed_bulk = g_ed_ctrl + 1;
    g_td_pool = (ohci_td_t *)(g_ed_bulk + 1);

    g_api->memset(g_hcca, 0, sizeof(ohci_hcca_t));
    g_api->memset(g_ed_ctrl, 0, 4096);

    mmio_write(OHCI_REG_HCCA, (uint32_t)g_hcca_phys);
    mmio_write(OHCI_REG_CTRL_HEAD, (uint32_t)g_ed_phys);
    mmio_write(OHCI_REG_BULK_HEAD, (uint32_t)(g_ed_phys + sizeof(ohci_ed_t)));

    uint32_t fi = 0x2EDF;
    uint32_t fsmps = ((fi - 210) * 6) / 7;
    mmio_write(OHCI_REG_FM_INTERVAL, fi | (fsmps << 16));
    mmio_write(OHCI_REG_PERIOD_START, (fi * 9) / 10);

    mmio_write(OHCI_REG_INT_ENABLE, 0);

    uint32_t rhda = mmio_read(OHCI_REG_RH_DESC);
    uint32_t ports = rhda & 0xFFu;
    if (rhda & (1u<<8)) {
        for (uint32_t p = 0; p < ports; p++) {
            uint32_t reg = OHCI_REG_RH_PORT1 + 4*p;
            mmio_write(reg, (1u<<8));
        }
    } else {
        mmio_write(OHCI_REG_RH_STATUS, (1u<<16));
    }

    uint32_t ctrl = mmio_read(OHCI_REG_CONTROL);
    ctrl &= ~(3u<<6);
    ctrl |= (2u<<6);
    ctrl |= (1u<<4) | (1u<<5);
    mmio_write(OHCI_REG_CONTROL, ctrl);

    mmio_write(OHCI_REG_CMD_STATUS, (1u<<1)|(1u<<2));

    g_ready = true;
}

uint32_t ohci_get_num_ports(void) {
    if (!g_ready) return 0;
    return mmio_read(OHCI_REG_RH_DESC) & 0xFFu;
}

bool ohci_reset_port(uint32_t port) {
    if (!g_ready) return false;
    uint32_t num = ohci_get_num_ports();
    if (port >= num) return false;
    uint32_t reg = OHCI_REG_RH_PORT1 + 4*port;
    uint32_t v = mmio_read(reg);

    for (int i = 0; i < 500 && (v & 1u) == 0; i++) {
        ohci_delay_ms(1);
        v = mmio_read(reg);
    }
    if ((v & 1u)==0) {
        return false;
    }

    if ((v & (1u<<8)) == 0) {
        mmio_write(reg, (1u<<8));
        ohci_delay_ms(20);
        v = mmio_read(reg);
    }

    mmio_write(reg, v | (1u<<4));
    ohci_delay_ms(50);
    mmio_write(reg, (1u<<20) | (1u<<16));
    for (int i=0;i<200;i++) {
        v = mmio_read(reg);
        if (v & (1u<<1)) break;
        ohci_delay_ms(1);
    }
    bool ok = (mmio_read(reg) & (1u<<1)) != 0;
    return ok;
}

bool ohci_port_connected(uint32_t port) {
    if (!g_ready) return false;
    if (port >= ohci_get_num_ports()) return false;
    uint32_t v = mmio_read(OHCI_REG_RH_PORT1 + 4*port);
    return (v & 1u) != 0;
}

bool ohci_port_valid(uint32_t port) {
    if (!g_ready) return false;
    return port < ohci_get_num_ports();
}

static ohci_td_t* alloc_td(int idx) { return g_td_pool + idx; }

static bool wait_td_done(ohci_td_t *td, uint32_t timeout_ms) {
    while (((td->control) & OHCI_TD_CC_MASK) == OHCI_TD_CC_MASK && timeout_ms--) {
        ohci_delay_ms(1);
    }
    return ((td->control & OHCI_TD_CC_MASK) == OHCI_TD_CC_NOERR);
}

static void ed_fill(ohci_ed_t *ed, uint8_t addr, uint8_t ep, uint16_t mps, uint32_t head, uint32_t tail, bool low_speed) {
    uint32_t ctrl = (uint32_t)addr | ((uint32_t)ep << 7) | ((uint32_t)mps << 16);
    if (low_speed) ctrl |= (1u<<13);
    ed->control = ctrl;
    ed->tailp = tail;
    ed->headp = head;
    ed->next = 0;
}

static bool ohci_control_xfer(uint8_t addr, uint8_t ep, uint16_t mps, struct usb_device_request *req, void *data) {
    bool dir_in = (req->bmRequestType & 0x80)!=0;
    uint16_t wlen = req->wLength;

    ohci_td_t *td_setup  = alloc_td(0);
    ohci_td_t *td_data   = alloc_td(1);
    ohci_td_t *td_status = alloc_td(2);
    ohci_td_t *td_dummy  = alloc_td(3);

    uint32_t phys_setup  = (uint32_t)g_api->virt_to_phys(td_setup);
    uint32_t phys_data   = (uint32_t)g_api->virt_to_phys(td_data);
    uint32_t phys_status = (uint32_t)g_api->virt_to_phys(td_status);
    uint32_t phys_dummy  = (uint32_t)g_api->virt_to_phys(td_dummy);

    uint32_t req_phys = (uint32_t)g_api->virt_to_phys(req);
    uint32_t buf_phys = (uint32_t)(wlen ? g_api->virt_to_phys(data) : 0);

    td_setup->control   = OHCI_TD_CC_MASK | OHCI_TD_DP_SETUP | OHCI_TD_TOGGLE_0 | (7u<<21);
    td_setup->curr_buf  = req_phys;
    td_setup->next_td   = wlen ? phys_data : phys_status;
    td_setup->buf_end   = req_phys + 7;

    if (wlen) {
        td_data->control  = OHCI_TD_CC_MASK | (dir_in ? OHCI_TD_DP_IN : OHCI_TD_DP_OUT) | OHCI_TD_TOGGLE_1 | (7u<<21);
        td_data->curr_buf = buf_phys;
        td_data->next_td  = phys_status;
        td_data->buf_end  = buf_phys + wlen - 1;
    }

    td_status->control  = OHCI_TD_CC_MASK | (dir_in ? OHCI_TD_DP_OUT : OHCI_TD_DP_IN) | OHCI_TD_TOGGLE_1 | (7u<<21);
    td_status->curr_buf = 0;
    td_status->next_td  = phys_dummy;
    td_status->buf_end  = 0;

    td_dummy->control = OHCI_TD_CC_MASK | OHCI_TD_DP_IN | OHCI_TD_TOGGLE_1 | (7u<<21);
    td_dummy->curr_buf = 0;
    td_dummy->next_td  = 0;
    td_dummy->buf_end  = 0;

    ed_fill(g_ed_ctrl, addr, ep, mps, phys_setup, phys_dummy, false);

    mmio_write(OHCI_REG_CTRL_HEAD, (uint32_t)g_ed_phys);
    mmio_write(OHCI_REG_CMD_STATUS, (1u<<1));

    bool ok = wait_td_done(td_status, 5000);
    g_ed_ctrl->headp = 0;
    g_ed_ctrl->tailp = 0;
    return ok;
}

static bool ohci_bulk_xfer(uint8_t addr, uint8_t ep, uint16_t mps, uint8_t pid, void *buf, uint32_t len) {
    ohci_td_t *td = alloc_td(4);
    ohci_td_t *td_dummy = alloc_td(5);
    uint32_t phys_td = (uint32_t)g_api->virt_to_phys(td);
    uint32_t phys_dummy = (uint32_t)g_api->virt_to_phys(td_dummy);
    uint32_t buf_phys = (uint32_t)g_api->virt_to_phys(buf);

    td->control  = OHCI_TD_CC_MASK | (pid ? OHCI_TD_DP_IN : OHCI_TD_DP_OUT) | OHCI_TD_TOGGLE_1 | (7u<<21);
    td->curr_buf = len ? buf_phys : 0;
    td->next_td  = phys_dummy;
    td->buf_end  = len ? (buf_phys + len - 1) : 0;

    td_dummy->control = OHCI_TD_CC_MASK | (pid ? OHCI_TD_DP_OUT : OHCI_TD_DP_IN) | OHCI_TD_TOGGLE_1 | (7u<<21);
    td_dummy->curr_buf = 0;
    td_dummy->next_td  = 0;
    td_dummy->buf_end  = 0;

    ed_fill(g_ed_bulk, addr, ep, mps, phys_td, phys_dummy, false);
    mmio_write(OHCI_REG_BULK_HEAD, (uint32_t)(g_ed_phys + sizeof(ohci_ed_t)));
    mmio_write(OHCI_REG_CMD_STATUS, (1u<<2));

    bool ok = wait_td_done(td, 5000);
    g_ed_bulk->headp = 0;
    g_ed_bulk->tailp = 0;
    return ok;
}

bool ohci_submit_control(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                         struct usb_device_request *req, void *data) {
    if (!g_ready) return false;
    uint16_t mps = (addr == 0) ? 8 : max_packet_size;
    return ohci_control_xfer(addr, endpoint, mps, req, data);
}

bool ohci_submit_bulk(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                      uint8_t pid, void *data, uint32_t length) {
    if (!g_ready) return false;
    return ohci_bulk_xfer(addr, endpoint, max_packet_size, pid, data, length);
}

bool ohci_submit_interrupt_in(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                              void *data, uint16_t length) {
    if (!g_ready || length == 0 || !data) return false;

    ohci_td_t *td       = alloc_td(6);
    ohci_td_t *td_dummy = alloc_td(7);
    uint32_t phys_td    = (uint32_t)g_api->virt_to_phys(td);
    uint32_t phys_dummy = (uint32_t)g_api->virt_to_phys(td_dummy);
    uint32_t buf_phys   = (uint32_t)g_api->virt_to_phys(data);

    td->control  = OHCI_TD_CC_MASK | OHCI_TD_DP_IN | OHCI_TD_TOGGLE_1 | (7u<<21);
    td->curr_buf = length ? buf_phys : 0;
    td->next_td  = phys_dummy;
    td->buf_end  = length ? (buf_phys + length - 1) : 0;

    td_dummy->control  = OHCI_TD_CC_MASK | OHCI_TD_DP_IN | OHCI_TD_TOGGLE_1 | (7u<<21);
    td_dummy->curr_buf = 0;
    td_dummy->next_td  = 0;
    td_dummy->buf_end  = 0;

    ed_fill(g_ed_bulk, addr, endpoint, max_packet_size, phys_td, phys_dummy, false);
    mmio_write(OHCI_REG_BULK_HEAD, (uint32_t)(g_ed_phys + sizeof(ohci_ed_t)));
    mmio_write(OHCI_REG_CMD_STATUS, (1u<<2));

    bool ok = wait_td_done(td, 2);
    if (!ok) {
        td->control = OHCI_TD_CC_MASK;
    }
    g_ed_bulk->headp = 0;
    g_ed_bulk->tailp = 0;
    return ok;
}