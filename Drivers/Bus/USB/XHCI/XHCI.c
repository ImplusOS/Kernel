#include "XHCI.h"
#include "Drivers/Module/DriverBinary.h"
#include "../USB_Main.h"
#ifndef IMPLUS_DRIVER_MODULE
#include "interfaces/hal_cpu.h"
#endif
#include <stddef.h>

extern const driver_binary_t *g_api;

#define XHCI_MAX_PORTS      16
#define XHCI_MAX_SLOTS      32
#define CMD_RING_SIZE       64
#define EVT_RING_SIZE       256
#define TRANSFER_RING_SIZE  128
#define XHCI_CONTROL_TIMEOUT_MS 30000u
#define XHCI_BULK_TIMEOUT_MS    30000u
#define XHCI_BULK_MAX_TRANSFER_SIZE (1024u * 1024u)
#define XHCI_NORMAL_TRB_MAX_LENGTH  65536u
#define XHCI_STALE_COMPLETION_TRACE 0u
#define XHCI_STALE_COMPLETION_DEBUG_LIMIT 8u

static volatile xhci_cap_regs_t   *g_cap    = NULL;
static volatile xhci_op_regs_t    *g_op     = NULL;
static volatile uint32_t          *g_db     = NULL;
static volatile xhci_intr_regs_t  *g_intr   = NULL;
static volatile xhci_trb_t        *g_evt_ring = NULL;
static bool                        g_ready  = false;
static volatile uint32_t g_cmd_cc = 0;
static volatile uint8_t  g_cmd_slot = 0;
static volatile bool     g_cmd_ready = false;
static volatile uint32_t g_last_cmd_cc = 0;
static volatile uint8_t  g_last_cmd_slot = 0;
static uint32_t g_last_reset_port = 0;

static uint8_t g_slot_map[256];

uint8_t g_xhci_pci_bus = 0;
uint8_t g_xhci_pci_dev = 0;
uint8_t g_xhci_pci_func = 0;
static uint8_t g_addr_from_slot[XHCI_MAX_SLOTS + 1];

static volatile uint64_t *g_dcbaa      = NULL;
static uint64_t           g_dcbaa_phys = 0;

#ifdef IMPLUS_DRIVER_MODULE
#define hal_cpu_pause               g_api->hal.cpu_pause
#define hal_cpu_enable_interrupts   g_api->hal.cpu_enable_interrupts
#define hal_cpu_halt                g_api->hal.cpu_halt
#define hal_cpu_disable_interrupts  g_api->hal.cpu_disable_interrupts
#define hal_cpu_memory_barrier      g_api->hal.cpu_memory_barrier
#define hal_cpu_save_interrupts     g_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts  g_api->hal.cpu_restore_interrupts
typedef struct { volatile uint32_t value; } spinlock_t;
static inline void spinlock_init(spinlock_t *l) { l->value = 0; }
static inline void spinlock_lock(spinlock_t *l) {
    while (__sync_lock_test_and_set(&l->value, 1)) {
        while (l->value) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->value); }
#else
#include "Core/sync/Spinlock.h"
#endif

static spinlock_t g_xhci_lock;

static volatile xhci_trb_t *g_cmd_ring      = NULL;
static uint64_t             g_cmd_ring_phys = 0;
static uint32_t             g_cmd_enq       = 0;
static uint8_t              g_cmd_cycle     = 1;

static uint64_t                     g_evt_ring_phys = 0;
static volatile xhci_erst_entry_t  *g_erst          = NULL;
static uint64_t                     g_erst_phys     = 0;
static uint32_t                     g_evt_deq       = 0;
static uint8_t                      g_evt_cycle     = 1;

typedef struct {
    volatile xhci_trb_t *ring;
    uint64_t  phys;
    uint32_t  enq;
    uint8_t   cycle;
} xhci_xfer_ring_t;

#define XHCI_MAX_EP_PER_SLOT 32
static xhci_xfer_ring_t g_xfer[XHCI_MAX_SLOTS][XHCI_MAX_EP_PER_SLOT];

static void    *g_dev_ctx[XHCI_MAX_SLOTS];
static uint64_t g_dev_ctx_phys[XHCI_MAX_SLOTS];
static void    *g_inp_ctx[XHCI_MAX_SLOTS];
static uint64_t g_inp_ctx_phys[XHCI_MAX_SLOTS];
static uint8_t g_completion_flags[256][XHCI_MAX_EP_PER_SLOT];
static uint8_t g_completion_error_count[256][XHCI_MAX_EP_PER_SLOT];
static uint32_t g_completion_cc[256][XHCI_MAX_EP_PER_SLOT];
static uint64_t g_completion_trb[256][XHCI_MAX_EP_PER_SLOT];
static uint8_t g_completion_match_required[256][XHCI_MAX_EP_PER_SLOT];
static uint32_t g_completion_stale_total = 0u;
static uint32_t g_completion_stale_logged_total = 0u;
static uint8_t g_completion_stale_last_slot = 0u;
static uint8_t g_completion_stale_last_ep = 0u;
static uint32_t g_completion_stale_last_cc = 0u;
static uint32_t g_completion_stale_last_expected = 0u;
static uint32_t g_completion_stale_last_actual = 0u;

#define XHCI_DMA_BOUNCE_SIZE  XHCI_BULK_MAX_TRANSFER_SIZE
static void    *g_dma_bounce     = NULL;
static uint64_t g_dma_bounce_phys = 0;
static spinlock_t g_dma_bounce_lock;
static uint64_t g_dma_max_address = 0;

static uint32_t xhci_td_size_field(uint32_t length, uint16_t max_packet_size)
{
    if (length == 0u) return 0u;
    if (max_packet_size == 0u) max_packet_size = 8u;

    uint32_t packets = (length + (uint32_t)max_packet_size - 1u) /
                       (uint32_t)max_packet_size;
    if (packets == 0u) return 0u;
    if (packets > 32u) return 31u;
    return packets - 1u;
}

static uint32_t xhci_normal_trb_chunk_len(uint64_t phys, uint32_t remaining)
{
    uint32_t boundary_left = 0x10000u - (uint32_t)(phys & 0xFFFFu);
    uint32_t chunk = remaining;

    if (chunk > boundary_left) chunk = boundary_left;
    if (chunk > XHCI_NORMAL_TRB_MAX_LENGTH) {
        chunk = XHCI_NORMAL_TRB_MAX_LENGTH;
    }
    return chunk;
}

static uint32_t xhci_bulk_trb_count(uint64_t phys, uint32_t length)
{
    if (length == 0u) return 1u;

    uint32_t count = 0u;
    uint32_t remaining = length;
    uint64_t current_phys = phys;

    while (remaining != 0u) {
        uint32_t chunk = xhci_normal_trb_chunk_len(current_phys, remaining);
        if (chunk == 0u) return 0u;
        remaining -= chunk;
        current_phys += chunk;
        ++count;
    }

    return count;
}

static void *xhci_dma_alloc(size_t size, size_t alignment, uint64_t *phys_out)
{
    if (g_api == NULL) return NULL;
    if (g_api->mem.dma_alloc_ex != NULL) {
        return g_api->mem.dma_alloc_ex(size, alignment, g_dma_max_address, phys_out);
    }
    return g_api->dma_alloc(size, phys_out);
}

static bool xhci_ensure_bounce_buf(void)
{
    if (g_dma_bounce) return true;
    if (!g_api) return false;
    g_dma_bounce = xhci_dma_alloc(XHCI_DMA_BOUNCE_SIZE, 4096u, &g_dma_bounce_phys);
    return g_dma_bounce != NULL;
}

static uint64_t g_tsc_per_ms = 5000000ULL;
void xhci_delay_ms(uint32_t ms);
static inline bool xhci_cc_success(uint32_t cc)
{
    return cc == 1u || cc == 13u;
}

static void xhci_debug_string(const char *str)
{
    if (str == NULL || g_api == NULL) {
        return;
    }
    if (g_api->dbg.write_string != NULL) {
        g_api->dbg.write_string(str);
    } else if (g_api->serial_write_string != NULL) {
        g_api->serial_write_string(str);
    }
}

static void xhci_debug_u32(uint32_t value)
{
    if (g_api == NULL) {
        return;
    }
    if (g_api->dbg.write_uint32 != NULL) {
        g_api->dbg.write_uint32(value);
    } else if (g_api->serial_write_uint32 != NULL) {
        g_api->serial_write_uint32(value);
    }
}

static inline uint64_t xhci_event_trb_pointer(uint64_t value)
{
    return value & ~0xFULL;
}

static inline bool xhci_completion_trb_matches(uint64_t expected,
                                               uint64_t actual)
{
    if (expected == 0u || actual == 0u) {
        return false;
    }
    return xhci_event_trb_pointer(expected) ==
           xhci_event_trb_pointer(actual);
}

static void xhci_note_stale_completion(uint8_t slot_id, uint8_t ep_idx,
                                       uint32_t cc, uint64_t expected,
                                       uint64_t actual)
{
    ++g_completion_stale_total;
    g_completion_stale_last_slot = slot_id;
    g_completion_stale_last_ep = ep_idx;
    g_completion_stale_last_cc = cc;
    g_completion_stale_last_expected = (uint32_t)expected;
    g_completion_stale_last_actual = (uint32_t)actual;
}

static void xhci_maybe_log_stale_completion(void)
{
    uint32_t total = g_completion_stale_total;
    if (total == g_completion_stale_logged_total) {
        return;
    }
    if (XHCI_STALE_COMPLETION_TRACE == 0u) {
        g_completion_stale_logged_total = total;
        return;
    }
    if (g_completion_stale_logged_total >= XHCI_STALE_COMPLETION_DEBUG_LIMIT &&
        (total & 0x7Fu) != 0u) {
        return;
    }
    g_completion_stale_logged_total = total;
    xhci_debug_string("[xhci] completion trb mismatch slot=");
    xhci_debug_u32(g_completion_stale_last_slot);
    xhci_debug_string(" ep=");
    xhci_debug_u32(g_completion_stale_last_ep);
    xhci_debug_string(" cc=");
    xhci_debug_u32(g_completion_stale_last_cc);
    xhci_debug_string(" exp=");
    xhci_debug_u32(g_completion_stale_last_expected);
    xhci_debug_string(" act=");
    xhci_debug_u32(g_completion_stale_last_actual);
    xhci_debug_string(" total=");
    xhci_debug_u32(total);
    xhci_debug_string("\n");
}

static inline uint64_t xhci_read_tsc(void);

static inline void xhci_relax_poll(uint32_t *spins)
{
    if ((*spins & 0x3FFu) == 0) {
        xhci_delay_ms(1);
    } else {
        hal_cpu_pause();
    }
    (*spins)++;
}

static bool xhci_wait_for_portsc(uint32_t port, uint32_t mask,
                                 uint32_t expected, uint32_t timeout_ms)
{
#if defined(__aarch64__)
    for (uint32_t elapsed = 0; elapsed < timeout_ms; ++elapsed) {
        if ((g_op->ports[port].portsc & mask) == expected) {
            return true;
        }
        xhci_delay_ms(1);
    }
    return ((g_op->ports[port].portsc & mask) == expected);
#else
    uint64_t start = xhci_read_tsc();
    uint64_t limit = (uint64_t)timeout_ms * g_tsc_per_ms;
    uint32_t spins = 0;

    while ((xhci_read_tsc() - start) < limit) {
        if ((g_op->ports[port].portsc & mask) == expected) {
            return true;
        }
        xhci_relax_poll(&spins);
    }
    return ((g_op->ports[port].portsc & mask) == expected);
#endif
}

static bool xhci_wait_for_usbsts(uint32_t mask, uint32_t expected, uint32_t timeout_ms)
{
#if defined(__aarch64__)
    for (uint32_t elapsed = 0; elapsed < timeout_ms; ++elapsed) {
        if ((g_op->usbsts & mask) == expected) {
            return true;
        }
        xhci_delay_ms(1);
    }
    return ((g_op->usbsts & mask) == expected);
#else
    uint64_t start = xhci_read_tsc();
    uint64_t limit = (uint64_t)timeout_ms * g_tsc_per_ms;
    uint32_t spins = 0;

    while ((xhci_read_tsc() - start) < limit) {
        if ((g_op->usbsts & mask) == expected) {
            return true;
        }
        xhci_relax_poll(&spins);
    }
    return ((g_op->usbsts & mask) == expected);
#endif
}

static bool xhci_wait_for_usbcmd(uint32_t mask, uint32_t expected, uint32_t timeout_ms)
{
#if defined(__aarch64__)
    for (uint32_t elapsed = 0; elapsed < timeout_ms; ++elapsed) {
        if ((g_op->usbcmd & mask) == expected) {
            return true;
        }
        xhci_delay_ms(1);
    }
    return ((g_op->usbcmd & mask) == expected);
#else
    uint64_t start = xhci_read_tsc();
    uint64_t limit = (uint64_t)timeout_ms * g_tsc_per_ms;
    uint32_t spins = 0;

    while ((xhci_read_tsc() - start) < limit) {
        if ((g_op->usbcmd & mask) == expected) {
            return true;
        }
        xhci_relax_poll(&spins);
    }
    return ((g_op->usbcmd & mask) == expected);
#endif
}

static void xhci_store_completion(uint8_t slot_id, uint8_t ep_idx,
                                  uint32_t cc, uint64_t trb_phys)
{
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return;
    if (ep_idx >= XHCI_MAX_EP_PER_SLOT) return;

    uint8_t addr = g_addr_from_slot[slot_id];
    if (addr == 0) {
        for (uint32_t a = 1; a < 256; a++) {
            if (g_slot_map[a] == slot_id) {
                addr = (uint8_t)a;
                g_addr_from_slot[slot_id] = addr;
                break;
            }
        }
    }
    if (addr == 0) return;
    uint64_t expected_trb = g_completion_trb[addr][ep_idx];
    if (g_completion_match_required[addr][ep_idx] != 0u &&
        !xhci_completion_trb_matches(expected_trb, trb_phys)) {
        xhci_note_stale_completion(slot_id, ep_idx, cc,
                                   expected_trb, trb_phys);
    } else if (expected_trb != 0u && trb_phys != 0u &&
               !xhci_completion_trb_matches(expected_trb, trb_phys)) {
        xhci_note_stale_completion(slot_id, ep_idx, cc,
                                   expected_trb, trb_phys);
    }
    if (xhci_cc_success(cc)) {
        if (g_completion_flags[addr][ep_idx] != 0xFFu) {
            ++g_completion_flags[addr][ep_idx];
        }
    } else if (g_completion_error_count[addr][ep_idx] != 0xFFu) {
        ++g_completion_error_count[addr][ep_idx];
    }
    g_completion_cc[addr][ep_idx] = cc;
}

static inline bool xhci_interrupt_pending(void)
{
    if (!g_intr) return false;
    return (g_intr->iman & XHCI_IMAN_IP) != 0;
}

static bool xhci_wait_for_work(uint64_t rflags, uint32_t *idle_spins)
{
    (void)rflags;

    if (xhci_interrupt_pending()) {
        hal_cpu_pause();
        return false;
    }

    if ((*idle_spins & 0x7u) != 0) {
        hal_cpu_pause();
        (*idle_spins)++;
        return false;
    }

#if defined(__x86_64__)
    __asm__ volatile("sti\n"
                     "hlt\n"
                     "cli"
                     ::: "memory");
#else
    xhci_delay_ms(1);
#endif
    (*idle_spins)++;
    return true;
}

static uint8_t xhci_take_completion(uint8_t slot_id, uint8_t ep_idx, uint32_t *cc_out)
{
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return 0;
    if (ep_idx >= XHCI_MAX_EP_PER_SLOT) return 0;

    uint8_t addr = g_addr_from_slot[slot_id];
    if (addr == 0) return 0;

    uint8_t state = 0u;
    if (cc_out != NULL) {
        *cc_out = g_completion_cc[addr][ep_idx];
    }
    if (g_completion_flags[addr][ep_idx] > 0u) {
        --g_completion_flags[addr][ep_idx];
        state = 1u;
    } else if (g_completion_error_count[addr][ep_idx] > 0u) {
        --g_completion_error_count[addr][ep_idx];
        state = 2u;
    }
    if (g_completion_flags[addr][ep_idx] == 0u &&
        g_completion_error_count[addr][ep_idx] == 0u) {
        g_completion_cc[addr][ep_idx] = 0u;
        g_completion_trb[addr][ep_idx] = 0u;
        g_completion_match_required[addr][ep_idx] = 0u;
    }
    return state;
}

static inline uint32_t max_ports(void) { return (g_cap->hcsparams1 >> 24) & 0xFFu; }
static inline uint32_t max_slots(void) { return  g_cap->hcsparams1        & 0xFFu; }

static inline uint32_t xhci_get_ctx_size(void) {
    if (!g_cap) return 32u;
    return (g_cap->hccparams1 & (1u << 2)) ? 64u : 32u;
}

static inline void xhci_mmio_write64_lo_hi(volatile uint32_t *lo_word, uint64_t val)
{
    lo_word[0] = (uint32_t)(val & 0xFFFFFFFFu);
    __asm__ volatile("" ::: "memory");
    lo_word[1] = (uint32_t)(val >> 32);
}

static uint32_t xhci_pagesize_bytes(void) {
    if (!g_op) return 4096u;
    uint32_t ps = g_op->pagesize;
    if (ps == 0) return 4096u;
    unsigned bit = (unsigned)__builtin_ctz(ps);
    return 1u << (bit + 12);
}

static void ring_doorbell(uint8_t slot, uint8_t target) {
    g_db[slot] = (uint32_t)target;
    __asm__ volatile("" ::: "memory");
}

void xhci_delay_ms(uint32_t ms) {
    if (g_api) g_api->timer_msleep(ms);
}

bool xhci_is_ready(void) { return g_ready; }

static inline uint64_t xhci_read_tsc(void) {
#if defined(__aarch64__)
    uint64_t val;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(val));
    return val;
#else
    uint32_t lo, hi;
    __asm__ volatile("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | (uint64_t)lo;
#endif
}

static void xhci_calibrate_tsc(void) {
#if defined(__aarch64__)
    g_tsc_per_ms = 1;
#else
    if (g_api && g_api->timer_msleep) {
        uint64_t t0 = xhci_read_tsc();
        g_api->timer_msleep(10);
        uint64_t t1 = xhci_read_tsc();
        if (t1 > t0 && (t1 - t0) > 10000) {
            g_tsc_per_ms = (t1 - t0) / 10;
        } else {
            g_tsc_per_ms = 2500000ULL;
        }
    }
#endif
}

static void xhci_tsc_busy_ms(uint32_t ms) {
    if (g_api) { g_api->timer_msleep(ms); return; }
    uint64_t t0 = xhci_read_tsc();
    uint64_t span = (uint64_t)ms * g_tsc_per_ms;
    while (xhci_read_tsc() - t0 < span)
        hal_cpu_pause();
}

static inline uint64_t pci_read_bar_fixed(uint8_t bus, uint8_t dev,
                                           uint8_t func, uint8_t offset)
{
    uint32_t bar_low = g_api->pci_read_config(bus, dev, func, offset);
    if (bar_low & 0x1u) return (uint64_t)(bar_low & ~0x3u);
    uint32_t bar_type = bar_low & 0x6u;
    if (bar_type == 0x4u) {
        uint32_t bar_high = g_api->pci_read_config(bus, dev, func, offset + 4);
        return ((uint64_t)bar_high << 32) | (bar_low & ~0xFu);
    }
    return (uint64_t)(bar_low & ~0xFu);
}

static void xhci_pci_try_power_d0(uint8_t bus, uint8_t dev, uint8_t fn) {
    uint32_t d34 = g_api->pci_read_config(bus, dev, fn, 0x34);
    uint8_t cap_ptr = (uint8_t)(d34 & 0xFCu);
    if (cap_ptr == 0) return;
    for (int n = 0; n < 64; n++) {
        uint32_t capdw = g_api->pci_read_config(bus, dev, fn, cap_ptr);
        uint8_t id   = (uint8_t)(capdw & 0xFFu);
        uint8_t next = (uint8_t)((capdw >> 8) & 0xFFu) & 0xFCu;
        if (id == 0x01u) {
            uint32_t pmcsr = g_api->pci_read_config(bus, dev, fn, cap_ptr + 4);
            pmcsr &= ~3u;
            g_api->pci_write_config(bus, dev, fn, cap_ptr + 4, pmcsr);
            return;
        }
        if (next == 0) return;
        cap_ptr = next;
    }
}

static bool xhci_take_ownership(void)
{
    if (!g_cap) return false;

    uint32_t hcc  = g_cap->hccparams1;
    uint32_t xecp = (hcc >> 16) & 0xFFFFu;
    if (xecp == 0) return true;

    volatile uint32_t *ext =
        (volatile uint32_t *)((uint8_t *)g_cap + (xecp << 2));

    while (ext) {
        uint32_t cap = ext[0];
        uint8_t  id   = (uint8_t)(cap & 0xFFu);
        uint8_t  next = (uint8_t)((cap >> 8) & 0xFFu);

        if (id == 1u) {
            ext[1] = 0;
            hal_cpu_memory_barrier();

            uint32_t val = ext[0];
            val |= (1u << 24);
            ext[0] = val;
            hal_cpu_memory_barrier();

            int timeout = 1000000;
            while ((ext[0] & (1u << 16)) && --timeout)
                hal_cpu_pause();

            if (timeout == 0) {
                ext[0] = (ext[0] & ~(1u << 16)) | (1u << 24);
                hal_cpu_memory_barrier();
            }

            ext[1] = 0;
            hal_cpu_memory_barrier();
            return true;
        }

        if (next == 0) break;
        ext = (volatile uint32_t *)((uint8_t *)ext + ((uint32_t)next << 2));
    }
    return true;
}

static bool xhci_setup_scratchpads(uint32_t num_sp, uint32_t page_bytes)
{
    if (num_sp == 0) return true;

    uint64_t  table_phys = 0;
    uint64_t *table = (uint64_t *)xhci_dma_alloc(
                          (size_t)num_sp * sizeof(uint64_t), 4096u, &table_phys);
    if (!table) return false;

    for (uint32_t i = 0; i < num_sp; i++) {
        uint64_t buf_phys = 0;
        void *buf = xhci_dma_alloc((size_t)page_bytes, (size_t)page_bytes, &buf_phys);
        if (!buf) return false;
        table[i] = buf_phys;
    }

    g_dcbaa[0] = table_phys;
    hal_cpu_memory_barrier();
    return true;
}

static bool init_xfer_ring(uint8_t slot_idx, uint8_t ep_idx)
{
    xhci_xfer_ring_t *r = &g_xfer[slot_idx][ep_idx];
    r->ring = (volatile xhci_trb_t *)xhci_dma_alloc(
                  sizeof(xhci_trb_t) * TRANSFER_RING_SIZE, 4096u, &r->phys);
    if (!r->ring) return false;

    for (int i = 0; i < TRANSFER_RING_SIZE; i++) {
        r->ring[i].parameter = 0;
        r->ring[i].status    = 0;
        r->ring[i].control   = 0;
    }

    r->ring[TRANSFER_RING_SIZE - 1].parameter = r->phys;
    r->ring[TRANSFER_RING_SIZE - 1].status    = 0;
    r->ring[TRANSFER_RING_SIZE - 1].control   =
        (TRB_TYPE_LINK << TRB_TYPE_SHIFT) |
        TRB_CTRL_TC |
        TRB_CTRL_CYCLE;
    hal_cpu_memory_barrier();

    r->enq   = 0;
    r->cycle = 1;
    return true;
}

static void xhci_reset_xfer_ring(uint8_t slot_idx, uint8_t ep_idx)
{
    if (slot_idx >= XHCI_MAX_SLOTS || ep_idx >= XHCI_MAX_EP_PER_SLOT) {
        return;
    }

    xhci_xfer_ring_t *r = &g_xfer[slot_idx][ep_idx];
    if (r->ring == NULL) {
        return;
    }

    for (int i = 0; i < TRANSFER_RING_SIZE; i++) {
        r->ring[i].parameter = 0;
        r->ring[i].status    = 0;
        r->ring[i].control   = 0;
    }

    r->ring[TRANSFER_RING_SIZE - 1].parameter = r->phys;
    r->ring[TRANSFER_RING_SIZE - 1].status    = 0;
    r->ring[TRANSFER_RING_SIZE - 1].control   =
        (TRB_TYPE_LINK << TRB_TYPE_SHIFT) |
        TRB_CTRL_TC |
        TRB_CTRL_CYCLE;
    hal_cpu_memory_barrier();

    r->enq   = 0;
    r->cycle = 1;
}

static bool xfer_enqueue(xhci_xfer_ring_t *r, xhci_trb_t *trb)
{
    volatile xhci_trb_t *slot = &r->ring[r->enq];
    slot->parameter = trb->parameter;
    slot->status    = trb->status;
    hal_cpu_memory_barrier();
    slot->control   = (trb->control & ~1u) | r->cycle;
    hal_cpu_memory_barrier();

    r->enq++;
    if (r->enq >= TRANSFER_RING_SIZE - 1) {
        volatile xhci_trb_t *link = &r->ring[TRANSFER_RING_SIZE - 1];
        uint32_t lctrl = link->control;
        link->control = (lctrl & ~1u) | (1u << 1) | r->cycle;
        hal_cpu_memory_barrier();
        r->enq    = 0;
        r->cycle ^= 1;
    }
    return true;
}

static bool xhci_wait_event(uint32_t expected_type,
                             uint8_t  expected_slot,
                             uint8_t  expected_ep,
                             uint32_t *status_out,
                             uint32_t timeout_ms)
{
#if !defined(__aarch64__)
    uint64_t tsc_per_ms = g_tsc_per_ms;
    uint64_t tsc_abs0 = xhci_read_tsc();
#endif

    bool result = false;
    uint64_t rflags;
    uint32_t idle_spins = 0;
#if defined(__aarch64__)
    uint32_t elapsed_ms = 0;
#endif
    
    rflags = hal_cpu_save_interrupts();

    while (1) {
#if defined(__aarch64__)
        if (elapsed_ms >= timeout_ms) {
            break;
        }
#else
        if (xhci_read_tsc() - tsc_abs0 >= (uint64_t)timeout_ms * tsc_per_ms) {
            break;
        }
#endif

        spinlock_lock(&g_xhci_lock);

        if (expected_type == 32u && expected_slot != 0) {
            uint32_t stored_cc = 0;
            uint8_t state = xhci_take_completion(expected_slot, expected_ep, &stored_cc);
            if (state != 0) {
                if (status_out) *status_out = stored_cc;
                result = (state == 1u);
                spinlock_unlock(&g_xhci_lock);
                hal_cpu_restore_interrupts(rflags);
                xhci_maybe_log_stale_completion();
                return result;
            }
        } else if (expected_type == 33u) {
            if (g_cmd_ready && (expected_slot == 0 || g_cmd_slot == expected_slot)) {
                if (status_out) *status_out = g_cmd_cc;
                result = (g_cmd_cc == 1u || g_cmd_cc == 13u);
                g_cmd_ready = false;
                spinlock_unlock(&g_xhci_lock);
                hal_cpu_restore_interrupts(rflags);
                xhci_maybe_log_stale_completion();
                return result;
            }
        }

        volatile xhci_trb_t *trb = &g_evt_ring[g_evt_deq];
        if ((trb->control & 1u) != g_evt_cycle) {
            spinlock_unlock(&g_xhci_lock);
            if (xhci_wait_for_work(rflags, &idle_spins)) {
#if defined(__aarch64__)
                ++elapsed_ms;
#endif
            }
            continue;
        }

        uint64_t ev_param = trb->parameter;
        uint32_t trb_type = (trb->control >> 10) & 0x3Fu;
        
        uint32_t cc       = (trb->status  >> 24) & 0xFFu;
        uint8_t ev_slot   = (uint8_t)((trb->control >> 24) & 0xFFu);
        uint8_t ev_ep     = (uint8_t)((trb->control >> 16) & 0xFFu);
        
        int match = (expected_type == 0) || (trb_type == expected_type);

        if (match && expected_slot != 0) {
            if (ev_slot != expected_slot) match = 0;
        }
        if (match && expected_ep != 0 && trb_type == 32u) {
            if (ev_ep != expected_ep) match = 0;
        }
        if (match && trb_type == 32u &&
            expected_slot != 0u && expected_ep != 0u) {
            uint8_t addr = g_addr_from_slot[expected_slot];
            if (addr != 0u &&
                g_completion_match_required[addr][expected_ep] != 0u) {
                uint64_t expected_trb = g_completion_trb[addr][expected_ep];
                if (!xhci_completion_trb_matches(expected_trb, ev_param)) {
                    xhci_note_stale_completion(ev_slot, ev_ep, cc,
                                               expected_trb, ev_param);
                }
            }
        }
        
        g_evt_deq++;
        if (g_evt_deq >= EVT_RING_SIZE) {
            g_evt_deq   = 0;
            g_evt_cycle ^= 1;
        }
        xhci_mmio_write64_lo_hi(
            (volatile uint32_t *)((uint8_t *)g_intr +
                                  offsetof(xhci_intr_regs_t, erdp)),
            (g_evt_ring_phys +
             (uint64_t)g_evt_deq * sizeof(xhci_trb_t)) | (1u << 3));
             
        if (!match) {
            if (trb_type == 32u) {
                xhci_store_completion(ev_slot, ev_ep, cc, ev_param);
            } else if (trb_type == 33u) {
                g_cmd_cc = cc;
                g_cmd_slot = ev_slot;
                g_cmd_ready = true;
                g_last_cmd_cc = cc;
                g_last_cmd_slot = ev_slot;
            }
        }

        spinlock_unlock(&g_xhci_lock);
        idle_spins = 0;

        if (match) {
            if (trb_type == 33u) {
                g_last_cmd_cc = cc;
                g_last_cmd_slot = ev_slot;
            }
            if (status_out) *status_out = cc;
            result = xhci_cc_success(cc);
            break;
        }
    }

    hal_cpu_restore_interrupts(rflags);
    xhci_maybe_log_stale_completion();

    return result;
}

static bool xhci_issue_command(xhci_trb_t *cmd,
                                uint8_t expected_slot,
                                uint32_t *cc_out)
{
    uint64_t rflags;
    rflags = hal_cpu_save_interrupts();
    spinlock_lock(&g_xhci_lock);

    cmd->control &= ~1u;
    cmd->control |= g_cmd_cycle;

    volatile xhci_trb_t *slot = &g_cmd_ring[g_cmd_enq];
    slot->parameter = cmd->parameter;
    slot->status    = cmd->status;
    hal_cpu_memory_barrier();
    slot->control   = (cmd->control & ~1u) | g_cmd_cycle;
    hal_cpu_memory_barrier();

    g_cmd_enq++;
    if (g_cmd_enq >= CMD_RING_SIZE - 1) {
        volatile xhci_trb_t *link = &g_cmd_ring[CMD_RING_SIZE - 1];
        link->parameter = g_cmd_ring_phys;
        link->status    = 0;
        hal_cpu_memory_barrier();
        link->control   = (6u << 10) | (1u << 1) | g_cmd_cycle;
        hal_cpu_memory_barrier();
        g_cmd_enq    = 0;
        g_cmd_cycle ^= 1;
    }

    ring_doorbell(0, 0);
    spinlock_unlock(&g_xhci_lock);
    
    hal_cpu_restore_interrupts(rflags);

    return xhci_wait_event(33u, expected_slot, 0u, cc_out, 500u);
}

static uint8_t xhci_get_port_speed(uint8_t port) {
    if (!g_op) return 3;
    uint32_t portsc = g_op->ports[port].portsc;
    uint8_t  speed  = (uint8_t)((portsc >> XHCI_PORTSC_SPEED_SHIFT) &
                                 XHCI_PORTSC_SPEED_MASK);
    return (speed == 0) ? 3u : speed;
}

uint32_t xhci_get_num_ports(void) {
    if (!g_cap) return 0;
    return max_ports();
}

bool xhci_port_connected(uint32_t port) {
    if (!g_op) return false;
    if (!xhci_port_valid(port)) return false;

    uint32_t portsc = g_op->ports[port].portsc;
    if ((portsc & XHCI_PORTSC_PP) == 0u) {
        g_op->ports[port].portsc = (portsc & 0x0E000200u) | XHCI_PORTSC_PP;
        hal_cpu_memory_barrier();
        (void)xhci_wait_for_portsc(port, XHCI_PORTSC_PP, XHCI_PORTSC_PP, 100u);
    }
    return (g_op->ports[port].portsc & XHCI_PORTSC_CCS) != 0;
}

bool xhci_port_valid(uint32_t port) {
    if (!g_op) return false;
    uint32_t maxp = max_ports();
    if (maxp == 0 || port >= maxp) return false;
    return true;
}

bool xhci_reset_port(uint32_t port)
{
    g_last_reset_port = port;
    if (!g_op || !g_ready) {
        return false;
    }
    if (!xhci_port_valid(port)) {
        return false;
    }

    uint32_t rw1c_mask = XHCI_PORTSC_CSC | XHCI_PORTSC_PEC |
                          XHCI_PORTSC_PRC | XHCI_PORTSC_PLC | XHCI_PORTSC_WRC;
    uint32_t initial_p = g_op->ports[port].portsc;
    g_op->ports[port].portsc = (initial_p & 0x0E000200u) |
                                (initial_p & rw1c_mask);
    hal_cpu_memory_barrier();

    uint32_t p = g_op->ports[port].portsc;

    if (!(p & XHCI_PORTSC_PP)) {
        g_op->ports[port].portsc = (p & 0x0E000200u) | XHCI_PORTSC_PP;
        if (!xhci_wait_for_portsc(port, XHCI_PORTSC_PP, XHCI_PORTSC_PP, 20u)) {
            p = g_op->ports[port].portsc;
        } else {
            p = g_op->ports[port].portsc;
        }
    }

    if (!(p & XHCI_PORTSC_CCS)) {
        if (!xhci_wait_for_portsc(port, XHCI_PORTSC_CCS, XHCI_PORTSC_CCS, 1000u)) {
            return false;
        }
    }
    p = g_op->ports[port].portsc;

    uint8_t speed = (uint8_t)((p >> XHCI_PORTSC_SPEED_SHIFT) & XHCI_PORTSC_SPEED_MASK);

    g_op->ports[port].portsc = (p & 0x0E000200u) | XHCI_PORTSC_PR;
    if (!xhci_wait_for_portsc(port, XHCI_PORTSC_PR, 0u, 3000u)) {
        return false;
    }

    {
        uint32_t sc     = g_op->ports[port].portsc;
        uint32_t sc_rw1c = sc & rw1c_mask;
        g_op->ports[port].portsc =
            (sc & 0x0E000200u) | sc_rw1c | XHCI_PORTSC_PRC;
    }
    xhci_wait_for_portsc(port, XHCI_PORTSC_PRC, 0u, 20u);

    uint32_t final_p = g_op->ports[port].portsc;
    speed = (uint8_t)((final_p >> XHCI_PORTSC_SPEED_SHIFT) &
                               XHCI_PORTSC_SPEED_MASK);

    if (speed >= XHCI_SPEED_SUPER_SPEED) {
        if ((final_p & XHCI_PORTSC_PLS_MASK) == XHCI_PORTSC_PLS_U0) {
            return true;
        }

        uint32_t wp = g_op->ports[port].portsc;
        g_op->ports[port].portsc = (wp & 0x0E000200u) | XHCI_PORTSC_WPR;
        if (!xhci_wait_for_portsc(port, XHCI_PORTSC_WRC, XHCI_PORTSC_WRC, 3000u)) {
            return false;
        }

        {
            uint32_t sc      = g_op->ports[port].portsc;
            uint32_t sc_rw1c = sc & rw1c_mask;
            g_op->ports[port].portsc =
                (sc & 0x0E000200u) | sc_rw1c |
                XHCI_PORTSC_WRC | XHCI_PORTSC_PRC;
        }
        xhci_wait_for_portsc(port, XHCI_PORTSC_WRC, 0u, 20u);
        final_p = g_op->ports[port].portsc;
        bool u0 = (final_p & XHCI_PORTSC_PLS_MASK) == XHCI_PORTSC_PLS_U0;
        return u0;
    }

    bool ped = (final_p & XHCI_PORTSC_PED) != 0;
    uint32_t pls = (final_p & XHCI_PORTSC_PLS_MASK) >> XHCI_PORTSC_PLS_SHIFT;
    bool u0 = (pls == 0u);
    if (!u0) {
        return false;
    }
    return ped;
}

void xhci_init(void)
{
    if (!g_api) {
        return;
    }

    spinlock_init(&g_xhci_lock);
    spinlock_init(&g_dma_bounce_lock);
    g_ready = false;

    xhci_calibrate_tsc();

    g_cap = NULL;
    g_op = NULL;
    g_db = NULL;
    g_intr = NULL;
    g_evt_ring = NULL;
    g_cmd_ring = NULL;
    g_erst = NULL;
    g_dcbaa = NULL;
    g_cmd_cc = 0;
    g_cmd_slot = 0;
    g_cmd_ready = false;
    g_last_cmd_cc = 0;
    g_last_cmd_slot = 0;
    g_evt_deq = 0;
    g_evt_cycle = 1;
    g_cmd_enq = 0;
    g_cmd_cycle = 1;
    g_last_reset_port = 0;
    g_dma_max_address = 0u;

    uint8_t xhci_bus = 0, xhci_dev = 0, xhci_func = 0;
    bool    found = false;

#if defined(__aarch64__)
    const uint16_t pci_bus_limit = 1;
#else
    const uint16_t pci_bus_limit = 256;
#endif

    for (uint16_t b = 0; b < pci_bus_limit && !found; b++) {
        for (uint8_t d = 0; d < 32 && !found; d++) {
            for (uint8_t f = 0; f < 8 && !found; f++) {
                uint32_t id = g_api->pci_read_config((uint8_t)b, d, f, 0x00);
                if ((id & 0xFFFFu) == 0xFFFFu) { if (f == 0) break; continue; }

                uint32_t cls = g_api->pci_read_config((uint8_t)b, d, f, 0x08);

                if (((cls >> 24) & 0xFFu) == 0x0Cu &&
                    ((cls >> 16) & 0xFFu) == 0x03u &&
                    ((cls >>  8) & 0xFFu) == 0x30u)
                {
                    xhci_bus  = (uint8_t)b;
                    xhci_dev  = d;
                    xhci_func = f;
                    g_xhci_pci_bus = xhci_bus;
                    g_xhci_pci_dev = xhci_dev;
                    g_xhci_pci_func = xhci_func;
                    found = true;
                }

                if (f == 0) {
                    uint32_t hdr = g_api->pci_read_config((uint8_t)b, d, f, 0x0C);
                    if (((hdr >> 16) & 0x80u) == 0) break;
                }
            }
        }
    }

    if (!found) {
        return;
    }

    uint32_t cmd = g_api->pci_read_config(xhci_bus, xhci_dev, xhci_func, 0x04);
    cmd |= (1u << 1) | (1u << 2);
    g_api->pci_write_config(xhci_bus, xhci_dev, xhci_func, 0x04, cmd);

    xhci_pci_try_power_d0(xhci_bus, xhci_dev, xhci_func);
    xhci_tsc_busy_ms(10);

    uint64_t bar0 = pci_read_bar_fixed(xhci_bus, xhci_dev, xhci_func, 0x10);

    if (bar0 == 0 || bar0 >= 0x10000000000ULL) {
        return;
    }

    g_cap = (xhci_cap_regs_t *)g_api->map_mmio_virt(bar0);
    if (!g_cap) {
        return;
    }

    if ((g_cap->hccparams1 & 1u) == 0u) {
        g_dma_max_address = 0xFFFFFFFFULL;
    }

    uint32_t caplen = (uint32_t)g_cap->caplength;
    if (caplen < 0x20u || caplen > 0x200u) caplen = 0x20u;
    g_op = (xhci_op_regs_t *)((uint8_t *)g_cap + caplen);

    xhci_take_ownership();

    if (!xhci_wait_for_usbsts(XHCI_STS_CNR, 0u, 5000u)) {
        return;
    }

    g_op->usbcmd &= ~(1u << 0);
    if (!xhci_wait_for_usbsts(XHCI_STS_HCH, XHCI_STS_HCH, 5000u)) {
        return;
    }

    g_op->usbcmd |= (1u << 1);
    if (!xhci_wait_for_usbcmd(XHCI_CMD_HCRST, 0u, 5000u)) {
        return;
    }
    if (!xhci_wait_for_usbsts(XHCI_STS_CNR, 0u, 5000u)) {
        return;
    }

    uint32_t db_offset  = g_cap->dboff  & ~0x3u;
    uint32_t rts_offset = g_cap->rtsoff & ~0x1Fu;

    g_db   = (volatile uint32_t *)((uint8_t *)g_cap + db_offset);
    g_intr = (volatile xhci_intr_regs_t *)
                 ((uint8_t *)g_cap + rts_offset + 0x20);

    g_op->usbsts = 0x0000011Cu;

    uint32_t slots = max_slots();
    if (slots > XHCI_MAX_SLOTS) slots = XHCI_MAX_SLOTS;
    g_op->config = slots;

    g_dcbaa = (volatile uint64_t *)xhci_dma_alloc(4096u, 4096u, &g_dcbaa_phys);
    if (!g_dcbaa) {
        return;
    }
    g_api->memset((void *)g_dcbaa, 0, 4096);
    hal_cpu_memory_barrier();

    uint32_t sp_lo   = (g_cap->hcsparams2 >> 27) & 0x1Fu;
    uint32_t sp_hi   = (g_cap->hcsparams2 >> 21) & 0x1Fu;
    uint32_t num_sp  = (sp_hi << 5) | sp_lo;
    uint32_t pg_byte = xhci_pagesize_bytes();

    if (num_sp > 0) {
        if (!xhci_setup_scratchpads(num_sp, pg_byte)) {
            return;
        }
    }

    xhci_mmio_write64_lo_hi(
        (volatile uint32_t *)((uint8_t *)g_op + offsetof(xhci_op_regs_t, dcbaap)),
        g_dcbaa_phys);

    g_cmd_ring = (volatile xhci_trb_t *)
                     xhci_dma_alloc(4096u, 4096u, &g_cmd_ring_phys);
    if (!g_cmd_ring) {
        return;
    }
    g_api->memset((void *)g_cmd_ring, 0, 4096);
    hal_cpu_memory_barrier();

    g_cmd_ring[CMD_RING_SIZE - 1].parameter = g_cmd_ring_phys;
    g_cmd_ring[CMD_RING_SIZE - 1].status    = 0;
    g_cmd_ring[CMD_RING_SIZE - 1].control   = (6u << 10) | (1u << 1) | 1u;

    g_cmd_enq   = 0;
    g_cmd_cycle = 1;

    xhci_mmio_write64_lo_hi(
        (volatile uint32_t *)((uint8_t *)g_op + offsetof(xhci_op_regs_t, crcr)),
        g_cmd_ring_phys | 1u);

    g_evt_ring = (volatile xhci_trb_t *)
                     xhci_dma_alloc(sizeof(xhci_trb_t) * EVT_RING_SIZE,
                                    4096u, &g_evt_ring_phys);
    g_erst     = (volatile xhci_erst_entry_t *)
                     xhci_dma_alloc(4096u, 4096u, &g_erst_phys);

    if (!g_evt_ring || !g_erst) {
        return;
    }
    g_api->memset((void *)g_evt_ring, 0,
                  sizeof(xhci_trb_t) * EVT_RING_SIZE);
    g_api->memset((void *)g_erst,     0, 4096);
    hal_cpu_memory_barrier();

    g_erst->base = g_evt_ring_phys;
    g_erst->size = EVT_RING_SIZE;

    g_evt_deq   = 0;
    g_evt_cycle = 1;
    hal_cpu_memory_barrier();

    (void)g_intr->iman;
    g_intr->erstsz = 1;

    xhci_mmio_write64_lo_hi(
        (volatile uint32_t *)((uint8_t *)g_intr + offsetof(xhci_intr_regs_t, erstba)),
        g_erst_phys);

    xhci_mmio_write64_lo_hi(
        (volatile uint32_t *)((uint8_t *)g_intr + offsetof(xhci_intr_regs_t, erdp)),
        g_evt_ring_phys | (1u << 3));

    g_intr->iman  |= XHCI_IMAN_IE;
    g_op->usbcmd  |= XHCI_CMD_INTE | XHCI_CMD_HSEE;
    hal_cpu_memory_barrier();

    (void)g_op->usbsts;

    g_op->usbcmd |= (1u << 0);
    if (!xhci_wait_for_usbsts(XHCI_STS_HCH, 0u, 1000u)) {
        return;
    }

    g_api->memset(g_slot_map, 0, sizeof(g_slot_map));
    g_api->memset(g_addr_from_slot, 0, sizeof(g_addr_from_slot));
    g_api->memset(g_completion_flags, 0, sizeof(g_completion_flags));
    g_api->memset(g_completion_error_count, 0,
                  sizeof(g_completion_error_count));
    g_api->memset(g_completion_cc, 0, sizeof(g_completion_cc));
    g_api->memset(g_completion_trb, 0, sizeof(g_completion_trb));
    g_api->memset(g_completion_match_required, 0,
                  sizeof(g_completion_match_required));
    g_completion_stale_total = 0u;
    g_completion_stale_logged_total = 0u;
    g_completion_stale_last_slot = 0u;
    g_completion_stale_last_ep = 0u;
    g_completion_stale_last_cc = 0u;
    g_completion_stale_last_expected = 0u;
    g_completion_stale_last_actual = 0u;
    g_ready = true;
}

static bool xhci_enable_slot(uint8_t *slot_id_out)
{
    xhci_trb_t cmd = {0};
    cmd.control = (9u << 10);

    uint32_t cc = 0;
    if (!xhci_issue_command(&cmd, 0u, &cc)) {
        return false;
    }

    uint8_t slot = g_last_cmd_slot;

    if (slot == 0 || slot > XHCI_MAX_SLOTS) return false;
    *slot_id_out = slot;
    return true;
}

static bool xhci_disable_slot_command(uint8_t slot_id)
{
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return false;

    xhci_trb_t cmd = {0};
    cmd.control = (10u << 10) | ((uint32_t)slot_id << 24);

    uint32_t cc = 0;
    return xhci_issue_command(&cmd, slot_id, &cc);
}

void xhci_disable_slot(uint8_t addr)
{
    if (addr == 0) return;
    uint8_t slot_id = g_slot_map[addr];
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return;

    xhci_disable_slot_command(slot_id);

    g_addr_from_slot[slot_id] = 0;
    g_slot_map[addr] = 0;
    for (uint8_t ep = 0u; ep < XHCI_MAX_EP_PER_SLOT; ++ep) {
        g_completion_flags[addr][ep] = 0u;
        g_completion_error_count[addr][ep] = 0u;
        g_completion_cc[addr][ep] = 0u;
        g_completion_trb[addr][ep] = 0u;
        g_completion_match_required[addr][ep] = 0u;
    }
}

extern uint8_t g_enum_speed;
extern uint8_t g_enum_parent_hub_addr;
extern uint8_t g_enum_parent_port;
extern uint8_t g_dev_root_port[256];
extern uint32_t g_dev_route_string[256];

static bool xhci_address_device(uint8_t port, uint8_t speed, uint8_t dev_addr)
{
    if (!g_ready)  { if (!g_api) return false; }

    uint8_t slot_id = 0;
    if (!xhci_enable_slot(&slot_id)) {
        return false;
    }
    uint8_t si = (uint8_t)(slot_id - 1);
    if (si >= XHCI_MAX_SLOTS) {
        xhci_disable_slot_command(slot_id);
        return false;
    }

    g_slot_map[0] = slot_id;

    if (!g_dev_ctx[si]) {
        g_dev_ctx[si] = xhci_dma_alloc(4096u, 4096u, &g_dev_ctx_phys[si]);
        if (!g_dev_ctx[si]) {
            g_slot_map[0] = 0;
            xhci_disable_slot_command(slot_id);
            return false;
        }
    }
    g_api->memset(g_dev_ctx[si], 0, 4096);

    if (!g_inp_ctx[si]) {
        g_inp_ctx[si] = xhci_dma_alloc(4096u, 4096u, &g_inp_ctx_phys[si]);
        if (!g_inp_ctx[si]) {
            g_slot_map[0] = 0;
            xhci_disable_slot_command(slot_id);
            return false;
        }
    }
    g_api->memset(g_inp_ctx[si], 0, 4096);

    if (!g_xfer[si][1].ring) {
        if (!init_xfer_ring(si, 1)) {
            g_slot_map[0] = 0;
            xhci_disable_slot_command(slot_id);
            return false;
        }
    }
    xhci_reset_xfer_ring(si, 1);

    g_dcbaa[slot_id] = g_dev_ctx_phys[si];
    hal_cpu_memory_barrier();

    uint32_t ctx_size = xhci_get_ctx_size();
    volatile uint32_t *ic_ptr   = (volatile uint32_t *)g_inp_ctx[si];
    volatile uint32_t *slot_ctx = ic_ptr + (ctx_size / 4);
    volatile uint32_t *ep0_ctx  = ic_ptr + 2 * (ctx_size / 4);

    uint32_t route_string = 0;
    uint8_t root_port = port + 1;
    uint8_t parent_slot = 0;
    uint8_t parent_port = 0;

    if (g_enum_parent_hub_addr != 0) {
        parent_slot = g_slot_map[g_enum_parent_hub_addr];
        parent_port = g_enum_parent_port;
        root_port   = g_dev_root_port[g_enum_parent_hub_addr];
        route_string = g_dev_route_string[0];
    }

    g_dev_root_port[dev_addr] = root_port;
    g_dev_route_string[dev_addr] = route_string;
    
    uint8_t actual_speed = (g_enum_parent_hub_addr != 0) ? g_enum_speed : speed;
    if (actual_speed == 0u) {
        actual_speed = 3u;
    }
    uint32_t mps;
    switch (actual_speed) {
    case 2:  mps = 8;   break;
    case 1:  mps = 8;   break;
    case 3:  mps = 64;  break;
    default: mps = 512; break;
    }

    g_api->memset(g_inp_ctx[si], 0, 4096);

    ic_ptr[0] = 0;
    ic_ptr[1] = (1u << 0) | (1u << 1);

    slot_ctx[0] = route_string | (1u << 27) | ((uint32_t)(actual_speed & 0xFu) << 20);
    slot_ctx[1] = ((uint32_t)root_port << 16);
    slot_ctx[2] = 0;
    if (actual_speed == 1 || actual_speed == 2) {
        if (g_enum_parent_hub_addr != 0) {
            slot_ctx[2] = ((uint32_t)parent_port << 8) | parent_slot;
        }
    }
    slot_ctx[3] = 0;

    ep0_ctx[0] = 0;
    ep0_ctx[1] = (mps << 16) | (4u << 3) | (3u << 1);
    ep0_ctx[2] = (uint32_t)(g_xfer[si][1].phys & 0xFFFFFFFFu) | 1u;
    ep0_ctx[3] = (uint32_t)(g_xfer[si][1].phys >> 32);
    ep0_ctx[4] = 8;
    hal_cpu_memory_barrier();

    bool ok = false;
    for (int retry = 0; retry < 3; retry++) {
        xhci_trb_t cmd2 = {0};
        cmd2.parameter = g_inp_ctx_phys[si];
        cmd2.status    = 0;
        cmd2.control   = (11u << 10) | ((uint32_t)slot_id << 24);

        uint32_t cc2 = 0;
        if (xhci_issue_command(&cmd2, slot_id, &cc2)) { ok = true; break; }

        xhci_delay_ms(10);
    }

    if (!ok) {
        g_slot_map[0] = 0;
        xhci_disable_slot_command(slot_id);
        return false;
    }

    g_slot_map[0] = 0;
    if (g_slot_map[dev_addr] != 0 && g_slot_map[dev_addr] <= XHCI_MAX_SLOTS) {
        g_addr_from_slot[g_slot_map[dev_addr]] = 0;
    }
    g_slot_map[dev_addr] = slot_id;
    g_addr_from_slot[slot_id] = dev_addr;
    return true;
}

bool xhci_evaluate_ep0_mps(uint8_t addr, uint16_t new_mps)
{
    if (!g_ready || !g_api) return false;

    uint8_t slot_id = g_slot_map[addr];
    if (slot_id == 0) return false;

    uint8_t si = slot_id - 1;
    if (si >= XHCI_MAX_SLOTS) return false;
    if (!g_inp_ctx[si]) return false;

    uint32_t ctx_size = xhci_get_ctx_size();
    volatile uint32_t *ic_ptr  = (volatile uint32_t *)g_inp_ctx[si];
    volatile uint32_t *ep0_ctx = ic_ptr + 2 * (ctx_size / 4);

    g_api->memset((void *)ic_ptr, 0, 4096);

    ic_ptr[0] = 0;
    ic_ptr[1] = (1u << 1);

    ep0_ctx[0] = 0;
    ep0_ctx[1] = ((uint32_t)new_mps << 16) | (4u << 3) | (3u << 1);
    ep0_ctx[2] = (uint32_t)(g_xfer[si][1].phys & 0xFFFFFFFFu) | 1u;
    ep0_ctx[3] = (uint32_t)(g_xfer[si][1].phys >> 32);
    ep0_ctx[4] = 8;
    hal_cpu_memory_barrier();

    xhci_trb_t cmd = {0};
    cmd.parameter = g_inp_ctx_phys[si];
    cmd.control   = (13u << 10) | ((uint32_t)slot_id << 24);

    uint32_t cc = 0;
    bool ok = xhci_issue_command(&cmd, slot_id, &cc);
    return ok;
}

static bool xhci_stop_endpoint_command(uint8_t slot_id, uint8_t ep_idx, uint32_t *cc_out)
{
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return false;
    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) return false;

    xhci_trb_t cmd = {0};
    cmd.control = (TRB_TYPE_STOP_ENDPOINT << TRB_TYPE_SHIFT) |
                  ((uint32_t)ep_idx << 16) |
                  ((uint32_t)slot_id << 24);

    return xhci_issue_command(&cmd, slot_id, cc_out);
}

static bool xhci_reset_endpoint_command(uint8_t slot_id, uint8_t ep_idx, uint32_t *cc_out)
{
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return false;
    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) return false;

    xhci_trb_t cmd = {0};
    cmd.control = (TRB_TYPE_RESET_ENDPOINT << TRB_TYPE_SHIFT) |
                  ((uint32_t)ep_idx << 16) |
                  ((uint32_t)slot_id << 24);

    return xhci_issue_command(&cmd, slot_id, cc_out);
}

static bool xhci_set_tr_dequeue_command(uint8_t slot_id, uint8_t ep_idx,
                                         uint64_t dequeue, uint32_t *cc_out)
{
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return false;
    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) return false;

    xhci_trb_t cmd = {0};
    cmd.parameter = dequeue;
    cmd.control = (TRB_TYPE_SET_TR_DEQUEUE << TRB_TYPE_SHIFT) |
                  ((uint32_t)ep_idx << 16) |
                  ((uint32_t)slot_id << 24);

    return xhci_issue_command(&cmd, slot_id, cc_out);
}

static bool xhci_recover_endpoint(uint8_t addr, uint8_t ep_idx)
{
    uint8_t slot_id = g_slot_map[addr];
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return false;
    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) return false;

    uint8_t si = (uint8_t)(slot_id - 1u);
    if (si >= XHCI_MAX_SLOTS || g_xfer[si][ep_idx].ring == NULL) {
        return false;
    }

    uint32_t stop_cc = 0;
    uint32_t reset_cc = 0;
    uint32_t deq_cc = 0;

    (void)xhci_stop_endpoint_command(slot_id, ep_idx, &stop_cc);
    (void)xhci_reset_endpoint_command(slot_id, ep_idx, &reset_cc);

    xhci_reset_xfer_ring(si, ep_idx);
    g_completion_flags[addr][ep_idx] = 0u;
    g_completion_error_count[addr][ep_idx] = 0u;
    g_completion_cc[addr][ep_idx] = 0u;
    g_completion_trb[addr][ep_idx] = 0u;
    g_completion_match_required[addr][ep_idx] = 0u;

    bool deq_ok = xhci_set_tr_dequeue_command(slot_id, ep_idx,
                                              g_xfer[si][ep_idx].phys | 1u,
                                              &deq_cc);
    return deq_ok;
}

bool xhci_recover_interrupt_endpoint(uint8_t addr, uint8_t ep_num)
{
    uint8_t ep_idx = (uint8_t)(ep_num * 2u + 1u);
    if (ep_idx == 0u || ep_idx >= XHCI_MAX_EP_PER_SLOT) {
        return false;
    }
    return xhci_recover_endpoint(addr, ep_idx);
}

static uint64_t xhci_pack_setup_packet(const usb_device_request_t *setup)
{
    if (setup == NULL) {
        return 0u;
    }

    return (uint64_t)setup->bmRequestType |
           ((uint64_t)setup->bRequest << 8) |
           ((uint64_t)(setup->wValue & 0xFFu) << 16) |
           ((uint64_t)((setup->wValue >> 8) & 0xFFu) << 24) |
           ((uint64_t)(setup->wIndex & 0xFFu) << 32) |
           ((uint64_t)((setup->wIndex >> 8) & 0xFFu) << 40) |
           ((uint64_t)(setup->wLength & 0xFFu) << 48) |
           ((uint64_t)((setup->wLength >> 8) & 0xFFu) << 56);
}

static bool xhci_configure_ep(uint8_t addr, uint8_t ep_addr, uint8_t ep_type,
                       uint16_t max_packet_size, uint8_t interval)
{
    static const uint8_t usb_to_xhci_type[4][2] = {
        { 4, 4 },
        { 1, 5 },
        { 2, 6 },
        { 3, 7 },
    };

    uint8_t slot_id = g_slot_map[addr];
    if (slot_id == 0) return false;
    uint8_t si = slot_id - 1;
    if (si >= XHCI_MAX_SLOTS) return false;

    uint8_t ep_num  = ep_addr & 0x7Fu;
    uint8_t dir     = (ep_addr >> 7) & 1u;
    uint8_t ep_idx  = (uint8_t)(ep_num * 2u + (dir ? 1u : 0u));
    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) return false;

    uint8_t xhci_ep_type = usb_to_xhci_type[ep_type & 3u][dir & 1u];

    if (!g_xfer[si][ep_idx].ring) {
        if (!init_xfer_ring(si, ep_idx)) return false;
    }
    if (!g_inp_ctx[si]) return false;

    uint32_t ctx_size = xhci_get_ctx_size();
    volatile uint32_t *ic_ptr = (volatile uint32_t *)g_inp_ctx[si];
    g_api->memset((void *)ic_ptr, 0, 4096);

    ic_ptr[0] = 0;
    ic_ptr[1] = (1u << 0) | (1u << ep_idx);

    volatile uint32_t *slot_ctx     = ic_ptr + (ctx_size / 4);
    volatile uint32_t *out_slot_ctx = (volatile uint32_t *)g_dev_ctx[si];

    uint32_t cur_entries = (out_slot_ctx[0] >> 27) & 0x1Fu;
    uint32_t new_entries = (ep_idx > cur_entries) ? ep_idx : cur_entries;

    slot_ctx[0] = (out_slot_ctx[0] & ~(0x1Fu << 27)) | (new_entries << 27);
    slot_ctx[1] = out_slot_ctx[1];
    slot_ctx[2] = out_slot_ctx[2];
    slot_ctx[3] = out_slot_ctx[3];

    volatile uint32_t *ep_ctx = ic_ptr + (uint32_t)(1u + ep_idx) * (ctx_size / 4);
    ep_ctx[0] = ((uint32_t)interval << 16);
    ep_ctx[1] = ((uint32_t)max_packet_size << 16) |
                ((uint32_t)(xhci_ep_type & 0x7u) << 3) |
                (3u << 1);
    ep_ctx[2] = (uint32_t)(g_xfer[si][ep_idx].phys | 1u);
    ep_ctx[3] = (uint32_t)(g_xfer[si][ep_idx].phys >> 32);
    uint32_t max_esit_payload = 0;
    if (xhci_ep_type != 4) {
        max_esit_payload = max_packet_size;
    }
    if (xhci_ep_type == 1 || xhci_ep_type == 5 || xhci_ep_type == 2 || xhci_ep_type == 6) {
        max_esit_payload = max_packet_size;
    }
    ep_ctx[4] = 8 | (max_esit_payload << 16);
    hal_cpu_memory_barrier();

    xhci_trb_t cmd = {0};
    cmd.parameter = g_inp_ctx_phys[si];
    cmd.control   = (12u << 10) | ((uint32_t)slot_id << 24);

    uint32_t cc = 0;
    return xhci_issue_command(&cmd, slot_id, &cc);
}

bool xhci_submit_control(uint8_t addr, uint8_t endpoint,
                          uint16_t max_packet_size,
                          struct usb_device_request *req, void *data)
{
    (void)endpoint;
    if (!g_ready) {
        return false;
    }

    usb_device_request_t *setup = (usb_device_request_t *)req;

    if (addr == 0 && setup->bRequest == 0x05) {
        uint8_t dev_addr = (uint8_t)setup->wValue;
        uint32_t port = g_last_reset_port;
        uint8_t speed = xhci_get_port_speed((uint8_t)port);
        return xhci_address_device((uint8_t)port, speed, dev_addr);
    }

    uint8_t slot_id = g_slot_map[addr];
    if (slot_id == 0) {
        return false;
    }

    uint8_t si = slot_id - 1;
    if (!g_dev_ctx[si] || !g_xfer[si][1].ring) {
        return false;
    }

    xhci_xfer_ring_t *r = &g_xfer[si][1];

    bool     dir_in = (setup->bmRequestType & USB_REQ_DIR_IN) != 0;
    uint16_t wlen   = setup->wLength;
    uint16_t ep0_mps = max_packet_size;
    if (ep0_mps == 0u) {
        ep0_mps = 8u;
    }

    uint64_t rflags;
    rflags = hal_cpu_save_interrupts();
    spinlock_lock(&g_xhci_lock);

    xhci_trb_t setup_trb = {0};
    setup_trb.parameter = xhci_pack_setup_packet(setup);
    setup_trb.status    = 8;
    uint32_t trt = (wlen == 0) ? 0u : (dir_in ? 3u : 2u);
    setup_trb.control = (TRB_TYPE_SETUP_STAGE << TRB_TYPE_SHIFT) |
                        TRB_CTRL_IDT |
                        (trt << 16);
    xfer_enqueue(r, &setup_trb);

    uint64_t data_phys = 0;
    void    *data_buf  = NULL;
    bool     ctrl_use_bounce = false;
    if (wlen > 0 && data != NULL) {
        if (xhci_ensure_bounce_buf()) {
            spinlock_lock(&g_dma_bounce_lock);
            data_buf  = g_dma_bounce;
            data_phys = g_dma_bounce_phys;
            ctrl_use_bounce = true;
        } else {
            data_buf = xhci_dma_alloc(wlen, 64u, &data_phys);
        }
        if (!data_buf) {
            spinlock_unlock(&g_xhci_lock);
            hal_cpu_restore_interrupts(rflags);
            return false;
        }

        if (!dir_in) {
            if (g_api && g_api->memcpy)
                g_api->memcpy(data_buf, data, wlen);
            else {
                uint8_t *src = (uint8_t *)data;
                uint8_t *dst = (uint8_t *)data_buf;
                for (uint16_t i = 0; i < wlen; i++) dst[i] = src[i];
            }
        }

        xhci_trb_t data_trb = {0};
        data_trb.parameter = data_phys;
        data_trb.status    = (uint32_t)wlen |
                              (xhci_td_size_field((uint32_t)wlen, ep0_mps) << 17);
        data_trb.control   = (TRB_TYPE_DATA_STAGE << TRB_TYPE_SHIFT) |
                              (dir_in ? TRB_CTRL_DIR_IN : 0u);
        xfer_enqueue(r, &data_trb);
    }

    xhci_trb_t status_trb = {0};
    uint32_t status_dir = (!dir_in || wlen == 0) ? (1u << 16) : 0u;
    status_trb.control  = (TRB_TYPE_STATUS_STAGE << TRB_TYPE_SHIFT) |
                          TRB_CTRL_IOC |
                          status_dir;
    xfer_enqueue(r, &status_trb);

    ring_doorbell(slot_id, 1);

    spinlock_unlock(&g_xhci_lock);
    hal_cpu_restore_interrupts(rflags);

    uint32_t cc = 0;
    bool ok = xhci_wait_event(32u, slot_id, 1u, &cc, XHCI_CONTROL_TIMEOUT_MS);

    if (ok && dir_in && wlen > 0 && data != NULL && data_buf != NULL) {
        if (g_api && g_api->memcpy)
            g_api->memcpy(data, data_buf, wlen);
        else {
            uint8_t *src = (uint8_t *)data_buf;
            uint8_t *dst = (uint8_t *)data;
            for (uint16_t i = 0; i < wlen; i++) dst[i] = src[i];
        }
    }

    if (ctrl_use_bounce) {
        spinlock_unlock(&g_dma_bounce_lock);
    } else if (data_buf != NULL) {
        g_api->dma_free(data_buf, wlen);
    }

    if (!ok) {
        (void)xhci_recover_endpoint(addr, 1u);
    }

    return ok;
}

bool xhci_submit_bulk(uint8_t addr, uint8_t endpoint,
                       uint16_t max_packet_size,
                       uint8_t pid, void *data, uint32_t length)
{
    if (!g_ready) {
        return false;
    }

    uint8_t slot_id = g_slot_map[addr];
    if (slot_id == 0) {
        return false;
    }
    uint8_t si = slot_id - 1;

    bool    dir_in   = (pid == 1);
    uint8_t ep_idx   = (uint8_t)(endpoint * 2u + (dir_in ? 1u : 0u));

    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) {
        return false;
    }

    if (!g_xfer[si][ep_idx].ring) {
        uint16_t mps = max_packet_size ? max_packet_size : 512u;
        if (!xhci_configure_ep(addr,
                               (uint8_t)(endpoint | (dir_in ? 0x80u : 0)),
                               2, mps, 0)) {
            return false;
        }
    }

    if (!g_xfer[si][ep_idx].ring) {
        return false;
    }

    xhci_xfer_ring_t *r = &g_xfer[si][ep_idx];
    uint32_t trb_count = 0u;

    uint64_t dma_phys = 0;
    void    *dma_buf  = NULL;
    bool     use_bounce = false;

    if (length > XHCI_BULK_MAX_TRANSFER_SIZE) {
        return false;
    }

    if (length <= XHCI_DMA_BOUNCE_SIZE && xhci_ensure_bounce_buf()) {
        spinlock_lock(&g_dma_bounce_lock);
        dma_buf   = g_dma_bounce;
        dma_phys  = g_dma_bounce_phys;
        use_bounce = true;
    } else {
        dma_buf = xhci_dma_alloc(length, 64u, &dma_phys);
    }
    if (!dma_buf) {
        return false;
    }

    trb_count = xhci_bulk_trb_count(dma_phys, length);
    if (trb_count == 0u || trb_count >= TRANSFER_RING_SIZE - 1u) {
        if (use_bounce) {
            spinlock_unlock(&g_dma_bounce_lock);
        } else {
            g_api->dma_free(dma_buf, length);
        }
        return false;
    }

    if (!dir_in) {
        if (g_api && g_api->memcpy)
            g_api->memcpy(dma_buf, data, length);
        else {
            uint8_t *src = (uint8_t *)data;
            uint8_t *dst = (uint8_t *)dma_buf;
            for (uint32_t i = 0; i < length; i++) dst[i] = src[i];
        }
    }

    uint64_t rflags;
    rflags = hal_cpu_save_interrupts();
    spinlock_lock(&g_xhci_lock);

    uint64_t current_phys = dma_phys;
    uint32_t remaining = length;
    do {
        uint32_t chunk = xhci_normal_trb_chunk_len(current_phys, remaining);
        if (length == 0u) {
            chunk = 0u;
        }

        xhci_trb_t trb = {0};
        trb.parameter = current_phys;
        trb.status    = chunk |
                        (xhci_td_size_field(remaining, max_packet_size) << 17);
        trb.control   = (TRB_TYPE_NORMAL << TRB_TYPE_SHIFT) |
                        TRB_CTRL_ISP;

        if (remaining > chunk) {
            trb.control |= TRB_CTRL_CH;
        } else {
            trb.control |= TRB_CTRL_IOC;
        }

        xfer_enqueue(r, &trb);

        current_phys += chunk;
        remaining -= chunk;
    } while (remaining != 0u);

    ring_doorbell(slot_id, ep_idx);

    spinlock_unlock(&g_xhci_lock);
    hal_cpu_restore_interrupts(rflags);

    uint32_t cc = 0;
    bool ok = xhci_wait_event(32u, slot_id, ep_idx, &cc, XHCI_BULK_TIMEOUT_MS);

    if (ok && dir_in) {
        if (g_api && g_api->memcpy)
            g_api->memcpy(data, dma_buf, length);
        else {
            uint8_t *src = (uint8_t *)dma_buf;
            uint8_t *dst = (uint8_t *)data;
            for (uint32_t i = 0; i < length; i++) dst[i] = src[i];
        }
    }

    if (use_bounce) {
        spinlock_unlock(&g_dma_bounce_lock);
    } else {
        g_api->dma_free(dma_buf, length);
    }

    if (!ok) {
        (void)xhci_recover_endpoint(addr, ep_idx);
    }
    return ok;
}

uint32_t xhci_get_max_bulk_transfer_size(void)
{
    return XHCI_BULK_MAX_TRANSFER_SIZE;
}

bool xhci_submit_interrupt_in(uint8_t addr, uint8_t ep_num,
                                     uint16_t max_packet_size,
                                     void *dma_buf, uint64_t dma_phys,
                                     uint16_t length)
{
    (void)dma_buf;
    if (!g_ready) return false;

    uint8_t slot_id = g_slot_map[addr];
    if (slot_id == 0) return false;
    uint8_t si = slot_id - 1;
    if (si >= XHCI_MAX_SLOTS) return false;

    uint8_t ep_idx = (uint8_t)(ep_num * 2u + 1u);
    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) return false;
    if (dma_phys == 0u || length == 0u) return false;

    if (!g_xfer[si][ep_idx].ring) {
        xhci_configure_ep(addr, ep_num | 0x80u, 3, max_packet_size, 6);
    }

    if (!g_xfer[si][ep_idx].ring) return false;

    xhci_xfer_ring_t *r = &g_xfer[si][ep_idx];
    
    uint64_t rflags;
    rflags = hal_cpu_save_interrupts();
    spinlock_lock(&g_xhci_lock);

    xhci_trb_t trb = {0};
    uint64_t trb_phys = r->phys + (uint64_t)r->enq * sizeof(xhci_trb_t);
    g_completion_flags[addr][ep_idx] = 0u;
    g_completion_error_count[addr][ep_idx] = 0u;
    g_completion_cc[addr][ep_idx] = 0u;
    g_completion_trb[addr][ep_idx] = trb_phys;
    g_completion_match_required[addr][ep_idx] = 1u;
    trb.parameter = dma_phys;
    trb.status    = (uint32_t)length |
                    (xhci_td_size_field((uint32_t)length, max_packet_size) << 17);
    trb.control   = (1u << 10) | TRB_CTRL_IOC | TRB_CTRL_ISP;
    xfer_enqueue(r, &trb);

    ring_doorbell(slot_id, ep_idx);

    spinlock_unlock(&g_xhci_lock);
    hal_cpu_restore_interrupts(rflags);

    return true;
}

bool xhci_submit_interrupt_in_sync(uint8_t addr, uint8_t ep_num,
                                   uint16_t max_packet_size,
                                   void *data, uint16_t length)
{
    if (!g_ready || data == NULL || length == 0u) return false;

    uint8_t slot_id = g_slot_map[addr];
    if (slot_id == 0) return false;

    uint8_t ep_idx = (uint8_t)(ep_num * 2u + 1u);
    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) return false;

    uint64_t dma_phys = 0u;
    void *dma_buf = xhci_dma_alloc(length, 64u, &dma_phys);
    if (dma_buf == NULL || dma_phys == 0u) {
        if (dma_buf != NULL && g_api != NULL && g_api->dma_free != NULL) {
            g_api->dma_free(dma_buf, length);
        }
        return false;
    }

    if (g_api != NULL && g_api->memset != NULL) {
        g_api->memset(dma_buf, 0, length);
    }

    bool submitted = xhci_submit_interrupt_in(addr, ep_num, max_packet_size,
                                              dma_buf, dma_phys, length);
    if (!submitted) {
        g_api->dma_free(dma_buf, length);
        return false;
    }

    uint32_t cc = 0u;
    bool ok = xhci_wait_event(32u, slot_id, ep_idx, &cc, 25u);
    uint64_t clear_flags = hal_cpu_save_interrupts();
    spinlock_lock(&g_xhci_lock);
    g_completion_flags[addr][ep_idx] = 0u;
    g_completion_error_count[addr][ep_idx] = 0u;
    g_completion_cc[addr][ep_idx] = 0u;
    g_completion_trb[addr][ep_idx] = 0u;
    g_completion_match_required[addr][ep_idx] = 0u;
    spinlock_unlock(&g_xhci_lock);
    hal_cpu_restore_interrupts(clear_flags);
    if (ok) {
        if (g_api != NULL && g_api->memcpy != NULL) {
            g_api->memcpy(data, dma_buf, length);
        } else {
            uint8_t *dst = (uint8_t *)data;
            uint8_t *src = (uint8_t *)dma_buf;
            for (uint16_t i = 0u; i < length; ++i) {
                dst[i] = src[i];
            }
        }
    } else {
        (void)xhci_recover_endpoint(addr, ep_idx);
    }

    g_api->dma_free(dma_buf, length);
    return ok;
}

static void xhci_drain_event_ring(void)
{
    uint64_t rflags;
    rflags = hal_cpu_save_interrupts();
    spinlock_lock(&g_xhci_lock);

    hal_cpu_memory_barrier();
    
    for (;;) {
        volatile xhci_trb_t *trb = &g_evt_ring[g_evt_deq];
        uint32_t cycle_bit = trb->control & 1u;
        if (cycle_bit != g_evt_cycle) {
            break;
        }

        uint64_t ev_param = trb->parameter;
        uint32_t trb_type = (trb->control >> 10) & 0x3Fu;
        
        uint32_t cc       = (trb->status  >> 24) & 0xFFu;
        uint8_t ev_slot   = (uint8_t)((trb->control >> 24) & 0xFFu);
        uint8_t ev_ep     = (uint8_t)((trb->control >> 16) & 0xFFu);

        g_evt_deq++;
        if (g_evt_deq >= EVT_RING_SIZE) {
            g_evt_deq   = 0;
            g_evt_cycle ^= 1;
        }
        xhci_mmio_write64_lo_hi(
            (volatile uint32_t *)((uint8_t *)g_intr +
                                  offsetof(xhci_intr_regs_t, erdp)),
            (g_evt_ring_phys +
             (uint64_t)g_evt_deq * sizeof(xhci_trb_t)) | (1u << 3));

        if (trb_type == 32u) {
            xhci_store_completion(ev_slot, ev_ep, cc, ev_param);
        } else if (trb_type == 33u) {
            g_cmd_cc = cc;
            g_cmd_slot = ev_slot;
            g_cmd_ready = true;
            g_last_cmd_cc = cc;
            g_last_cmd_slot = ev_slot;
        }
    }

    spinlock_unlock(&g_xhci_lock);
    hal_cpu_restore_interrupts(rflags);
    xhci_maybe_log_stale_completion();
}

int xhci_check_interrupt_event(uint8_t addr, uint8_t ep_num)
{
    if (!g_ready) return -1;
    if (g_slot_map[addr] == 0) return -1;

    xhci_drain_event_ring();
    xhci_maybe_log_stale_completion();

    uint8_t ep_idx = (uint8_t)(ep_num * 2u + 1u);
    if (ep_idx == 0 || ep_idx >= XHCI_MAX_EP_PER_SLOT) return -1;

    uint8_t result = 0u;
    uint64_t rflags = hal_cpu_save_interrupts();
    spinlock_lock(&g_xhci_lock);
    if (g_completion_flags[addr][ep_idx] > 0u) {
        --g_completion_flags[addr][ep_idx];
        result = 1u;
    } else if (g_completion_error_count[addr][ep_idx] > 0u) {
        --g_completion_error_count[addr][ep_idx];
        result = 2u;
    }
    if (g_completion_flags[addr][ep_idx] == 0u &&
        g_completion_error_count[addr][ep_idx] == 0u) {
        g_completion_cc[addr][ep_idx] = 0u;
        g_completion_trb[addr][ep_idx] = 0u;
        g_completion_match_required[addr][ep_idx] = 0u;
    }
    spinlock_unlock(&g_xhci_lock);
    hal_cpu_restore_interrupts(rflags);

    if (result == 1u) {
        return 1;
    } else if (result == 2u) {
        (void)xhci_recover_endpoint(addr, ep_idx);
        return -1;
    }
    return 0;
}
