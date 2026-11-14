#include "USB_HID.h"
#include "../USB_Main.h"
#include "Drivers/Module/DriverBinary.h"

extern const driver_binary_t *g_api;

#ifdef IMPLUS_DRIVER_MODULE
#define hal_cpu_save_interrupts    g_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts g_api->hal.cpu_restore_interrupts
#define hal_cpu_pause             g_api->hal.cpu_pause

typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_init(spinlock_t *l)   { l->locked = 0; }
static inline void spinlock_lock(spinlock_t *l)   {
    while (__sync_lock_test_and_set(&l->locked, 1)) {
        while (l->locked) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->locked); }

static inline uint64_t irq_save_disable(void) { return hal_cpu_save_interrupts(); }
static inline void irq_restore(uint64_t flags) { hal_cpu_restore_interrupts(flags); }
#else
#include "Core/sync/Spinlock.h"
#include "interfaces/hal_cpu.h"
#include "Debug/serial/Serial.h"
#endif

#include "kernel/keycodes.h"
#include "kernel/input_utils.h"

#define USB_HID_QUEUE_SIZE 64
/* SET_IDLE / SET_PROTOCOL, issued once per HID interface at enumeration
 * time. 0 would mean "the controller's default", which on xHCI is 30 s: a
 * device that stalls here would hold the boot sweep (input_init) for a
 * minute on its own. */
#define USB_HID_CTRL_TIMEOUT_MS 500u
#define USB_HID_ENABLE_IDLE_PENDING_RECOVER 0u
#define USB_HID_PENDING_RECOVER_NS (250ULL * 1000ULL * 1000ULL)
#define USB_HID_PENDING_RECOVER_POLLS 64u
#define USB_HID_PENDING_RECOVER_MAX_NS (1000ULL * 1000ULL * 1000ULL)
#define USB_HID_PENDING_RECOVER_MAX_POLLS 256u

static const uint16_t hid_to_ps2_set1[256] = {
    [0x04] = KEY_A, [0x05] = KEY_B, [0x06] = KEY_C, [0x07] = KEY_D,
    [0x08] = KEY_E, [0x09] = KEY_F, [0x0A] = KEY_G, [0x0B] = KEY_H,
    [0x0C] = KEY_I, [0x0D] = KEY_J, [0x0E] = KEY_K, [0x0F] = KEY_L,
    [0x10] = KEY_M, [0x11] = KEY_N, [0x12] = KEY_O, [0x13] = KEY_P,
    [0x14] = KEY_Q, [0x15] = KEY_R, [0x16] = KEY_S, [0x17] = KEY_T,
    [0x18] = KEY_U, [0x19] = KEY_V, [0x1A] = KEY_W, [0x1B] = KEY_X,
    [0x1C] = KEY_Y, [0x1D] = KEY_Z,
    [0x1E] = KEY_1, [0x1F] = KEY_2, [0x20] = KEY_3, [0x21] = KEY_4,
    [0x22] = KEY_5, [0x23] = KEY_6, [0x24] = KEY_7, [0x25] = KEY_8,
    [0x26] = KEY_9, [0x27] = KEY_0,
    [0x28] = KEY_ENTER, [0x29] = KEY_ESC, [0x2A] = KEY_BACKSPACE,
    [0x2B] = KEY_TAB, [0x2C] = KEY_SPACE, [0x2D] = KEY_MINUS,
    [0x2E] = KEY_EQUAL, [0x2F] = KEY_LEFTBRACE, [0x30] = KEY_RIGHTBRACE,
    [0x31] = KEY_BACKSLASH, [0x33] = KEY_SEMICOLON, [0x34] = KEY_APOSTROPHE,
    [0x35] = KEY_GRAVE, [0x36] = KEY_COMMA, [0x37] = KEY_DOT,
    [0x38] = KEY_SLASH, [0x39] = KEY_CAPSLOCK,
    [0x3A] = KEY_F1, [0x3B] = KEY_F2, [0x3C] = KEY_F3, [0x3D] = KEY_F4,
    [0x3E] = KEY_F5, [0x3F] = KEY_F6, [0x40] = KEY_F7, [0x41] = KEY_F8,
    [0x42] = KEY_F9, [0x43] = KEY_F10, [0x44] = KEY_F11, [0x45] = KEY_F12,
    [0x49] = KEY_INSERT, [0x4A] = KEY_HOME, [0x4B] = KEY_PAGEUP,
    [0x4C] = KEY_DELETE, [0x4D] = KEY_END, [0x4E] = KEY_PAGEDOWN,
    [0x4F] = KEY_RIGHT, [0x50] = KEY_LEFT, [0x51] = KEY_DOWN, [0x52] = KEY_UP,
    [0x53] = KEY_NUMLOCK, [0x54] = KEY_KPSLASH, [0x55] = KEY_KPASTERISK,
    [0x56] = KEY_KPMINUS, [0x57] = KEY_KPPLUS, [0x58] = KEY_KPENTER,
    [0x59] = KEY_KP1, [0x5A] = KEY_KP2, [0x5B] = KEY_KP3, [0x5C] = KEY_KP4,
    [0x5D] = KEY_KP5, [0x5E] = KEY_KP6, [0x5F] = KEY_KP7, [0x60] = KEY_KP8,
    [0x61] = KEY_KP9, [0x62] = KEY_KP0, [0x63] = KEY_KPDOT,
    [0x65] = KEY_COMPOSE,
    [0x87] = KEY_RO, [0x88] = KEY_KATAKANAHIRAGANA, [0x89] = KEY_YEN,
    [0x8A] = KEY_HENKAN, [0x8B] = KEY_MUHENKAN,
    /* Modifiers. A boot-protocol report never lists these in its key array;
     * they are the bits of byte 0, turned into these usages by
     * process_kbd_report() so the modifier keys themselves reach the input
     * stack as presses and releases. */
    [0xE0] = KEY_LEFTCTRL,  [0xE1] = KEY_LEFTSHIFT,
    [0xE2] = KEY_LEFTALT,   [0xE3] = KEY_LEFTMETA,
    [0xE4] = KEY_RIGHTCTRL, [0xE5] = KEY_RIGHTSHIFT,
    [0xE6] = KEY_RIGHTALT,  [0xE7] = KEY_RIGHTMETA,
};

typedef struct {
    uint8_t dev_addr;
    uint8_t interface;
    uint8_t ep_in;
    uint16_t mps;
    bool valid;
    void     *dma_buf;
    uint64_t  dma_phys;
    bool      pending;
    uint32_t  pending_polls;
    uint64_t  pending_started_ns;
    bool      recover_armed;
    uint8_t   hc_type;
} usb_hid_device_t;

static usb_hid_device_t g_usd_kbd   = {0};
static usb_hid_device_t g_usd_mouse = {0};

static driver_keyboard_event_t g_kbd_queue[USB_HID_QUEUE_SIZE];
static uint32_t g_kbd_head = 0, g_kbd_tail = 0, g_kbd_count = 0;
static spinlock_t g_kbd_lock = {0};

static driver_mouse_event_t g_mouse_queue[USB_HID_QUEUE_SIZE];
static uint32_t g_mouse_head = 0, g_mouse_tail = 0, g_mouse_count = 0;
static spinlock_t g_mouse_lock = {0};

static uint8_t  g_last_kbd_report[8] = {0};
static uint8_t  g_last_mouse_buttons = 0;
static uint16_t g_last_abs_x = 0xFFFFu;
static uint16_t g_last_abs_y = 0xFFFFu;

static uint32_t g_mouse_recover_polls = USB_HID_PENDING_RECOVER_POLLS;
static uint64_t g_mouse_recover_ns = USB_HID_PENDING_RECOVER_NS;

static uint32_t g_kbd_poll_timer = 0;
static uint32_t g_mouse_poll_timer = 0;

extern usb_hc_type_t usb_get_device_hc_type(uint8_t addr);

static uint64_t hid_monotonic_ns(void)
{
#ifdef IMPLUS_DRIVER_MODULE
    if (g_api != NULL && g_api->timer.monotonic_ns != NULL) {
        return g_api->timer.monotonic_ns();
    }
    if (g_api != NULL && g_api->timer_ticks != NULL &&
        g_api->timer_hz != NULL) {
        uint32_t hz = g_api->timer_hz();
        if (hz != 0u) {
            uint64_t ticks = g_api->timer_ticks();
            return (ticks / hz) * 1000000000ULL +
                   ((ticks % hz) * 1000000000ULL) / hz;
        }
    }
    return 0u;
#else
    extern uint64_t timer_monotonic_ns(void);
    return timer_monotonic_ns();
#endif
}

static void hid_mark_pending(usb_hid_device_t *dev)
{
    if (dev == NULL) {
        return;
    }
    dev->pending = true;
    dev->pending_polls = 0u;
    dev->pending_started_ns = hid_monotonic_ns();
}

static void hid_clear_pending(usb_hid_device_t *dev)
{
    if (dev == NULL) {
        return;
    }
    dev->pending = false;
    dev->pending_polls = 0u;
    dev->pending_started_ns = 0u;
}

static bool hid_pending_timed_out(const usb_hid_device_t *dev,
                                  uint64_t recover_ns,
                                  uint32_t recover_polls)
{
    if (dev == NULL) {
        return false;
    }

    uint64_t now = hid_monotonic_ns();
    if (now != 0u && dev->pending_started_ns != 0u &&
        now >= dev->pending_started_ns &&
        now - dev->pending_started_ns >= recover_ns) {
        return true;
    }
    return dev->pending_polls >= recover_polls;
}

static void hid_reset_mouse_recover_interval(void)
{
    g_mouse_recover_polls = USB_HID_PENDING_RECOVER_POLLS;
    g_mouse_recover_ns = USB_HID_PENDING_RECOVER_NS;
}

static void hid_backoff_mouse_recover_interval(void)
{
    if (g_mouse_recover_polls < USB_HID_PENDING_RECOVER_MAX_POLLS) {
        g_mouse_recover_polls *= 2u;
        if (g_mouse_recover_polls > USB_HID_PENDING_RECOVER_MAX_POLLS) {
            g_mouse_recover_polls = USB_HID_PENDING_RECOVER_MAX_POLLS;
        }
    }
    if (g_mouse_recover_ns < USB_HID_PENDING_RECOVER_MAX_NS) {
        g_mouse_recover_ns *= 2ULL;
        if (g_mouse_recover_ns > USB_HID_PENDING_RECOVER_MAX_NS) {
            g_mouse_recover_ns = USB_HID_PENDING_RECOVER_MAX_NS;
        }
    }
}

static void hid_mark_mouse_pending(void)
{
    hid_mark_pending(&g_usd_mouse);
    g_usd_mouse.recover_armed = true;
}


/* Pointer report layout. Boot-protocol mice use the fixed 3/4-byte boot
 * report; anything else (QEMU's usb-tablet, touch screens, KVM dongles) is
 * described by its HID report descriptor, which hid_parse_pointer_layout()
 * reads once at attach time. */
typedef struct {
    bool     absolute;      /* X/Y are absolute positions */
    uint8_t  report_id;     /* 0 = the device does not use report IDs */
    uint16_t report_bytes;  /* bytes to request per interrupt IN */
    uint16_t btn_bit[3];    /* bit offsets of buttons 1..3 (0xFFFF = none) */
    uint16_t x_bit, x_size; /* bit offset / size; offsets exclude report ID */
    uint16_t y_bit, y_size;
    uint16_t wheel_bit, wheel_size;
    int32_t  x_min, x_max, y_min, y_max;
} hid_pointer_layout_t;

static hid_pointer_layout_t g_mouse_layout;

#define HID_FIELD_NONE 0xFFFFu

static void hid_zero(void *p, uint32_t n)
{
    volatile uint8_t *b = (volatile uint8_t *)p;
    for (uint32_t i = 0u; i < n; ++i) b[i] = 0u;
}

static void hid_layout_set_boot(hid_pointer_layout_t *l)
{
    hid_zero(l, (uint32_t)sizeof(*l));
    l->absolute = false;
    l->report_bytes = 4u;
    l->btn_bit[0] = 0u; l->btn_bit[1] = 1u; l->btn_bit[2] = 2u;
    l->x_bit = 8u;  l->x_size = 8u;
    l->y_bit = 16u; l->y_size = 8u;
    l->wheel_bit = 24u; l->wheel_size = 8u;
}

static uint32_t hid_item_udata(const uint8_t *p, uint32_t size)
{
    uint32_t v = 0u;
    for (uint32_t i = 0u; i < size; ++i) v |= (uint32_t)p[i] << (8u * i);
    return v;
}

static int32_t hid_item_sdata(const uint8_t *p, uint32_t size)
{
    uint32_t v = hid_item_udata(p, size);
    if (size == 1u) return (int32_t)(int8_t)v;
    if (size == 2u) return (int32_t)(int16_t)v;
    return (int32_t)v;
}

/* Minimal HID report-descriptor walk: find the first input report that
 * carries Generic Desktop X and Y, and record where X, Y, wheel and buttons
 * 1..3 live in it. Returns false when no such report exists. */
static bool hid_parse_pointer_layout(const uint8_t *d, uint32_t len,
                                     hid_pointer_layout_t *out)
{
    enum { MAX_USAGES = 16, MAX_IDS = 8 };
    uint32_t usage_page = 0u, report_size = 0u, report_count = 0u;
    int32_t  logical_min = 0, logical_max = 0;
    uint8_t  report_id = 0u;
    uint32_t usages[MAX_USAGES];
    uint32_t usage_count = 0u, usage_min = 0u, usage_max = 0u;
    bool     have_usage_range = false;
    uint8_t  ids[MAX_IDS];
    uint32_t bits[MAX_IDS];
    uint32_t id_count = 0u;
    hid_pointer_layout_t l;
    bool found_x = false, found_y = false;
    uint8_t found_id = 0u;

    hid_zero(&l, (uint32_t)sizeof(l));
    l.btn_bit[0] = l.btn_bit[1] = l.btn_bit[2] = HID_FIELD_NONE;
    l.wheel_bit = HID_FIELD_NONE;

    for (uint32_t pos = 0u; pos < len;) {
        uint8_t prefix = d[pos];
        if (prefix == 0xFEu) {                   /* long item: skip */
            if (pos + 2u >= len) break;
            pos += 3u + d[pos + 1u];
            continue;
        }
        uint32_t size = prefix & 0x3u;
        if (size == 3u) size = 4u;
        uint8_t type = (uint8_t)((prefix >> 2) & 0x3u);
        uint8_t tag  = (uint8_t)(prefix >> 4);
        if (pos + 1u + size > len) break;
        const uint8_t *data = &d[pos + 1u];
        pos += 1u + size;

        if (type == 1u) {                        /* global */
            switch (tag) {
            case 0x0: usage_page = hid_item_udata(data, size); break;
            case 0x1: logical_min = hid_item_sdata(data, size); break;
            case 0x2: logical_max = hid_item_sdata(data, size); break;
            case 0x7: report_size = hid_item_udata(data, size); break;
            case 0x8: report_id = (uint8_t)hid_item_udata(data, size); break;
            case 0x9: report_count = hid_item_udata(data, size); break;
            default: break;
            }
        } else if (type == 2u) {                 /* local */
            uint32_t u = hid_item_udata(data, size);
            if (size < 4u) u |= usage_page << 16;
            if (tag == 0x0) {
                if (usage_count < MAX_USAGES) usages[usage_count++] = u;
            } else if (tag == 0x1) {
                usage_min = u; have_usage_range = true;
            } else if (tag == 0x2) {
                usage_max = u; have_usage_range = true;
            }
        } else if (type == 0u) {                 /* main */
            if (tag == 0x8) {                    /* Input */
                uint32_t flags = hid_item_udata(data, size);
                uint32_t slot = 0u;
                while (slot < id_count && ids[slot] != report_id) ++slot;
                if (slot == id_count && id_count < MAX_IDS) {
                    ids[id_count] = report_id; bits[id_count] = 0u; ++id_count;
                }
                uint32_t base = (slot < id_count) ? bits[slot] : 0u;
                bool constant = (flags & 0x1u) != 0u;
                bool variable = (flags & 0x2u) != 0u;
                bool relative = (flags & 0x4u) != 0u;
                if (!constant && variable &&
                    (!found_x || found_id == report_id)) {
                    for (uint32_t i = 0u; i < report_count; ++i) {
                        uint32_t u;
                        if (i < usage_count) {
                            u = usages[i];
                        } else if (have_usage_range &&
                                   usage_min + i <= usage_max) {
                            u = usage_min + i;
                        } else if (usage_count > 0u) {
                            u = usages[usage_count - 1u];
                        } else {
                            continue;
                        }
                        uint32_t bit = base + i * report_size;
                        if (bit > 0xFFF0u) break;
                        if (u == 0x00010030u && !found_x) {
                            l.x_bit = (uint16_t)bit;
                            l.x_size = (uint16_t)report_size;
                            l.x_min = logical_min; l.x_max = logical_max;
                            l.absolute = !relative;
                            found_x = true; found_id = report_id;
                        } else if (u == 0x00010031u && !found_y) {
                            l.y_bit = (uint16_t)bit;
                            l.y_size = (uint16_t)report_size;
                            l.y_min = logical_min; l.y_max = logical_max;
                            found_y = true;
                        } else if (u == 0x00010038u &&
                                   l.wheel_bit == HID_FIELD_NONE) {
                            l.wheel_bit = (uint16_t)bit;
                            l.wheel_size = (uint16_t)report_size;
                        } else if ((u >> 16) == 0x0009u &&
                                   (u & 0xFFFFu) >= 1u &&
                                   (u & 0xFFFFu) <= 3u) {
                            l.btn_bit[(u & 0xFFFFu) - 1u] = (uint16_t)bit;
                        }
                    }
                }
                if (slot < id_count) bits[slot] += report_size * report_count;
            }
            /* Every main item clears the local state. */
            usage_count = 0u;
            have_usage_range = false;
            usage_min = usage_max = 0u;
        }
    }

    if (!found_x || !found_y || l.x_size == 0u || l.x_size > 32u ||
        l.y_size == 0u || l.y_size > 32u) {
        return false;
    }
    l.report_id = found_id;
    uint32_t total_bits = 0u;
    for (uint32_t i = 0u; i < id_count; ++i) {
        if (ids[i] == found_id) total_bits = bits[i];
    }
    uint32_t bytes = (total_bits + 7u) / 8u + (found_id != 0u ? 1u : 0u);
    if (bytes == 0u || bytes > 64u) return false;
    l.report_bytes = (uint16_t)bytes;
    *out = l;
    return true;
}

static uint32_t hid_get_bits(const uint8_t *r, uint32_t bit, uint32_t size)
{
    uint32_t v = 0u;
    for (uint32_t i = 0u; i < size; ++i) {
        uint32_t b = bit + i;
        if ((r[b >> 3] >> (b & 7u)) & 1u) v |= 1u << i;
    }
    return v;
}

static int32_t hid_get_signed(const uint8_t *r, uint32_t bit, uint32_t size)
{
    uint32_t v = hid_get_bits(r, bit, size);
    if (size < 32u && (v & (1u << (size - 1u))) != 0u) {
        v |= ~((1u << size) - 1u);
    }
    return (int32_t)v;
}

/* Scale an absolute coordinate to 0..DRIVER_MOUSE_ABS_MAX. */
static uint16_t hid_scale_abs(int32_t v, int32_t lo, int32_t hi)
{
    if (hi <= lo) return 0u;
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    int64_t num = (int64_t)(v - lo) * (int64_t)DRIVER_MOUSE_ABS_MAX;
    return (uint16_t)(num / (int64_t)(hi - lo));
}

void usb_hid_init(void)
{
    g_usd_kbd.valid     = false;
    g_usd_kbd.dma_buf   = NULL;
    g_usd_kbd.pending   = false;
    g_usd_kbd.pending_polls = 0u;
    g_usd_kbd.pending_started_ns = 0u;
    g_usd_kbd.recover_armed = false;
    g_usd_mouse.valid   = false;
    g_usd_mouse.dma_buf = NULL;
    g_usd_mouse.pending = false;
    g_usd_mouse.pending_polls = 0u;
    g_usd_mouse.pending_started_ns = 0u;
    g_usd_mouse.recover_armed = false;
    hid_reset_mouse_recover_interval();
    
    spinlock_init(&g_kbd_lock);
    spinlock_init(&g_mouse_lock);
}

static bool hid_alloc_dma(usb_hid_device_t *dev)
{
    if (dev->dma_buf) return true;
    if (!g_api || !g_api->dma_alloc) return false;
    dev->dma_buf = g_api->dma_alloc(64, &dev->dma_phys);
    return dev->dma_buf != NULL;
}

static bool hid_sync_interrupt_in(usb_hid_device_t *dev, uint16_t len)
{
    if (!dev->valid || !dev->dma_buf) return false;
    if (g_api && g_api->memset)
        g_api->memset(dev->dma_buf, 0, 64);
    return usb_submit_interrupt_in_sync(dev->dev_addr, dev->ep_in,
                                         dev->mps, dev->dma_buf, len);
}

void usb_hid_add_keyboard(uint8_t dev_addr, uint8_t interface,
                           uint8_t ep_in, uint16_t mps)
{
    g_usd_kbd.dev_addr  = dev_addr;
    g_usd_kbd.interface = interface;
    g_usd_kbd.ep_in     = ep_in;
    g_usd_kbd.mps       = mps;
    g_usd_kbd.hc_type   = usb_get_device_hc_type(dev_addr);
    if (g_usd_kbd.hc_type == USB_HC_NONE) g_usd_kbd.hc_type = usb_get_hc_type();

    usb_submit_control_timeout(dev_addr, 0x21, 0x0B, 0, interface, 0, NULL,
                               USB_HID_CTRL_TIMEOUT_MS);
    usb_submit_control_timeout(dev_addr, 0x21, 0x0A, 0, interface, 0, NULL,
                               USB_HID_CTRL_TIMEOUT_MS);

    if (!hid_alloc_dma(&g_usd_kbd)) {
        return;
    }

    g_usd_kbd.valid = true;
    g_usd_kbd.pending = false;
    g_usd_kbd.pending_polls = 0u;
    g_usd_kbd.pending_started_ns = 0u;
    g_usd_kbd.recover_armed = false;
}

void usb_hid_add_mouse(uint8_t dev_addr, uint8_t interface,
                        uint8_t ep_in, uint16_t mps, bool boot_protocol)
{
    hid_pointer_layout_t layout;
    hid_layout_set_boot(&layout);
    if (boot_protocol) {
        /* SET_PROTOCOL(boot): the fixed report hid_layout_set_boot() expects. */
        usb_submit_control_timeout(dev_addr, 0x21, 0x0B, 0, interface, 0, NULL,
                                   USB_HID_CTRL_TIMEOUT_MS);
    } else {
        /* No boot protocol (e.g. an absolute tablet): learn the report
         * format from the HID report descriptor. Not a pointer -> ignore. */
        uint8_t rdesc[512];
        hid_zero(rdesc, (uint32_t)sizeof(rdesc));
        if (!usb_submit_control_timeout(dev_addr, 0x81, 0x06, 0x2200u,
                                        interface, (uint16_t)sizeof(rdesc),
                                        rdesc, USB_HID_CTRL_TIMEOUT_MS) ||
            !hid_parse_pointer_layout(rdesc, (uint32_t)sizeof(rdesc),
                                      &layout)) {
            return;
        }
        if (mps != 0u && layout.report_bytes > mps) {
            layout.report_bytes = mps;
        }
    }
    usb_submit_control_timeout(dev_addr, 0x21, 0x0A, 0, interface, 0, NULL,
                               USB_HID_CTRL_TIMEOUT_MS);

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_mouse_lock);
    g_usd_mouse.valid     = false;
    g_usd_mouse.dev_addr  = dev_addr;
    g_usd_mouse.interface = interface;
    g_usd_mouse.ep_in     = ep_in;
    g_usd_mouse.mps       = mps;
    g_usd_mouse.hc_type   = usb_get_device_hc_type(dev_addr);
    if (g_usd_mouse.hc_type == USB_HC_NONE) g_usd_mouse.hc_type = usb_get_hc_type();
    g_mouse_layout = layout;
    g_last_abs_x = 0xFFFFu;
    g_last_abs_y = 0xFFFFu;
    spinlock_unlock(&g_mouse_lock);
    irq_restore(flags);

    if (g_api != NULL && g_api->serial_write_string != NULL) {
        g_api->serial_write_string(layout.absolute ?
                                   "[USB-HID] absolute pointer\n" :
                                   "[USB-HID] relative pointer\n");
    }

    if (!hid_alloc_dma(&g_usd_mouse)) {
        return;
    }

    g_usd_mouse.valid = true;
    g_usd_mouse.pending = false;
    g_usd_mouse.pending_polls = 0u;
    g_usd_mouse.pending_started_ns = 0u;
    g_usd_mouse.recover_armed = false;
    hid_reset_mouse_recover_interval();
}

void usb_hid_remove_keyboard(uint8_t dev_addr)
{
    if (!g_usd_kbd.valid || g_usd_kbd.dev_addr != dev_addr) {
        return;
    }

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_kbd_lock);
    g_usd_kbd.valid = false;
    g_usd_kbd.pending = false;
    g_usd_kbd.pending_polls = 0u;
    g_usd_kbd.pending_started_ns = 0u;
    g_usd_kbd.recover_armed = false;
    g_kbd_head = 0;
    g_kbd_tail = 0;
    g_kbd_count = 0;
    for (uint32_t i = 0u; i < sizeof(g_last_kbd_report); ++i) {
        g_last_kbd_report[i] = 0u;
    }
    spinlock_unlock(&g_kbd_lock);
    irq_restore(flags);
}

void usb_hid_remove_mouse(uint8_t dev_addr)
{
    if (!g_usd_mouse.valid || g_usd_mouse.dev_addr != dev_addr) {
        return;
    }

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_mouse_lock);
    g_usd_mouse.valid = false;
    g_usd_mouse.pending = false;
    g_usd_mouse.pending_polls = 0u;
    g_usd_mouse.pending_started_ns = 0u;
    g_usd_mouse.recover_armed = false;
    hid_reset_mouse_recover_interval();
    g_mouse_head = 0;
    g_mouse_tail = 0;
    g_mouse_count = 0;
    g_last_mouse_buttons = 0u;
    spinlock_unlock(&g_mouse_lock);
    irq_restore(flags);
}

static void push_kbd_event(uint16_t hid_keycode, uint8_t pressed, uint8_t modifiers)
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_kbd_lock);
    
    if (g_kbd_count >= USB_HID_QUEUE_SIZE) {
        spinlock_unlock(&g_kbd_lock);
        irq_restore(flags);
        return;
    }

    uint16_t keycode = 0;
    if (hid_keycode < 256) {
        keycode = hid_to_ps2_set1[hid_keycode];
    }
    if (keycode == 0) {
        spinlock_unlock(&g_kbd_lock);
        irq_restore(flags);
        return;
    }

    driver_keyboard_event_t evt = {0};
    evt.keycode   = keycode;
    evt.pressed   = pressed;
    evt.modifiers = modifiers;
    if (pressed) {
        evt.ascii = (uint8_t)keycode_to_ascii(keycode, modifiers);
    }
    g_kbd_queue[g_kbd_head] = evt;
    g_kbd_head = (g_kbd_head + 1) % USB_HID_QUEUE_SIZE;
    g_kbd_count++;
    
    spinlock_unlock(&g_kbd_lock);
    irq_restore(flags);
}

static void process_kbd_report(uint8_t *report)
{
    bool changed = false;
    for (int i = 0; i < 8; i++) {
        if (report[i] != g_last_kbd_report[i]) { changed = true; break; }
    }
    if (!changed) return;

    uint8_t mods = report[0];
    uint8_t ps2_mods = 0;
    if (mods & 0x02) ps2_mods |= DRIVER_KBD_MOD_SHIFT;
    if (mods & 0x20) ps2_mods |= DRIVER_KBD_MOD_SHIFT;
    if (mods & 0x01) ps2_mods |= DRIVER_KBD_MOD_CTRL;
    if (mods & 0x10) ps2_mods |= DRIVER_KBD_MOD_CTRL;
    if (mods & 0x04) ps2_mods |= DRIVER_KBD_MOD_ALT;
    if (mods & 0x40) ps2_mods |= DRIVER_KBD_MOD_ALT;

    /* Modifier transitions first, so a chord's Ctrl is already down when its
     * letter arrives. Only reporting them in `modifiers` was not enough: a
     * consumer that tracks key state itself -- the X server behind XSession --
     * never saw Ctrl pressed, and USB Ctrl+L typed a plain "l". */
    uint8_t mod_changes = (uint8_t)(mods ^ g_last_kbd_report[0]);
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
        if ((mod_changes & (1u << bit)) != 0u) {
            push_kbd_event((uint16_t)(0xE0u + bit),
                           (uint8_t)((mods >> bit) & 1u), ps2_mods);
        }
    }

    for (int i = 2; i < 8; i++) {
        if (report[i] != 0) {
            bool found = false;
            for (int j = 2; j < 8; j++)
                if (g_last_kbd_report[j] == report[i]) { found = true; break; }
            if (!found) push_kbd_event(report[i], 1, ps2_mods);
        }
    }

    for (int i = 2; i < 8; i++) {
        if (g_last_kbd_report[i] != 0) {
            bool found = false;
            for (int j = 2; j < 8; j++)
                if (report[j] == g_last_kbd_report[i]) { found = true; break; }
            if (!found) push_kbd_event(g_last_kbd_report[i], 0, ps2_mods);
        }
    }
    for (int i = 0; i < 8; i++) g_last_kbd_report[i] = report[i];
}

static void poll_keyboard(void)
{
    if (!g_usd_kbd.valid || !g_usd_kbd.dma_buf) return;

    if (g_usd_kbd.hc_type == USB_HC_XHCI) {
        if (!g_usd_kbd.pending) {
            if (usb_submit_interrupt_in_async(g_usd_kbd.dev_addr, g_usd_kbd.ep_in,
                                              g_usd_kbd.mps,
                                              g_usd_kbd.dma_buf, g_usd_kbd.dma_phys, 8)) {
                g_usd_kbd.pending = true;
            }
            return;
        }

        int event_code = usb_check_interrupt_event(g_usd_kbd.dev_addr, g_usd_kbd.ep_in);
        
        if (event_code != 0) {
            if (event_code > 0) {
                process_kbd_report((uint8_t *)g_usd_kbd.dma_buf);
            }
            if (usb_submit_interrupt_in_async(g_usd_kbd.dev_addr, g_usd_kbd.ep_in,
                                              g_usd_kbd.mps,
                                              g_usd_kbd.dma_buf, g_usd_kbd.dma_phys, 8)) {
                g_usd_kbd.pending = true;
            } else {
                g_usd_kbd.pending = false;
            }
        }
    } else {
        g_kbd_poll_timer++;
        if (g_kbd_poll_timer >= 10) {
            g_kbd_poll_timer = 0;
            if (hid_sync_interrupt_in(&g_usd_kbd, 8)) {
                process_kbd_report((uint8_t *)g_usd_kbd.dma_buf);
            }
        }
    }
}

static void process_mouse_report(uint8_t *report)
{
    const hid_pointer_layout_t *l = &g_mouse_layout;
    driver_mouse_event_t evt = {0};
    uint8_t buttons = 0u;

    if (l->report_id != 0u) {
        if (report[0] != l->report_id) {
            return;
        }
        ++report;
    } else if (!l->absolute && (report[0] & 0xC0u) != 0u) {
        return;   /* boot report with X/Y overflow: drop it */
    }

    for (uint32_t i = 0u; i < 3u; ++i) {
        if (l->btn_bit[i] != HID_FIELD_NONE &&
            hid_get_bits(report, l->btn_bit[i], 1u) != 0u) {
            buttons |= (uint8_t)(1u << i);
        }
    }
    int32_t wheel = 0;
    if (l->wheel_bit != HID_FIELD_NONE && l->wheel_size != 0u) {
        wheel = hid_get_signed(report, l->wheel_bit, l->wheel_size);
    }

    if (l->absolute) {
        evt.x = hid_scale_abs((int32_t)hid_get_bits(report, l->x_bit, l->x_size),
                              l->x_min, l->x_max);
        evt.y = hid_scale_abs((int32_t)hid_get_bits(report, l->y_bit, l->y_size),
                              l->y_min, l->y_max);
        evt.flags = DRIVER_MOUSE_FLAG_ABSOLUTE;
        if (evt.x == g_last_abs_x && evt.y == g_last_abs_y && wheel == 0 &&
            buttons == g_last_mouse_buttons) {
            return;
        }
        g_last_abs_x = evt.x;
        g_last_abs_y = evt.y;
    } else {
        int32_t dx = hid_get_signed(report, l->x_bit, l->x_size);
        int32_t dy = hid_get_signed(report, l->y_bit, l->y_size);
        if (dx == 0 && dy == 0 && wheel == 0 && buttons == g_last_mouse_buttons) {
            return;
        }
        evt.x = (uint16_t)(int16_t)dx;
        evt.y = (uint16_t)(int16_t)dy;
    }
    evt.buttons = buttons;
    evt.wheel = (int8_t)(wheel > 127 ? 127 : (wheel < -127 ? -127 : wheel));

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_mouse_lock);

    g_last_mouse_buttons = buttons;

    if (g_mouse_count >= USB_HID_QUEUE_SIZE) {
        g_mouse_tail = (g_mouse_tail + 1u) % USB_HID_QUEUE_SIZE;
        --g_mouse_count;
    }
    g_mouse_queue[g_mouse_head] = evt;
    g_mouse_head = (g_mouse_head + 1u) % USB_HID_QUEUE_SIZE;
    ++g_mouse_count;

    spinlock_unlock(&g_mouse_lock);
    irq_restore(flags);
}

static void poll_mouse(void)
{
    if (!g_usd_mouse.valid || !g_usd_mouse.dma_buf) return;

    if (g_usd_mouse.hc_type == USB_HC_XHCI) {
        if (!g_usd_mouse.pending) {
            if (usb_submit_interrupt_in_async(g_usd_mouse.dev_addr, g_usd_mouse.ep_in,
                                              g_usd_mouse.mps,
                                              g_usd_mouse.dma_buf, g_usd_mouse.dma_phys, g_mouse_layout.report_bytes)) {
                hid_mark_mouse_pending();
            } else {
                hid_clear_pending(&g_usd_mouse);
            }
            return;
        }

        int event_code = usb_check_interrupt_event(g_usd_mouse.dev_addr, g_usd_mouse.ep_in);

        if (event_code != 0) {
            g_usd_mouse.pending_polls = 0u;
            g_usd_mouse.pending_started_ns = 0u;

            if (event_code > 0) {
                process_mouse_report((uint8_t *)g_usd_mouse.dma_buf);
                hid_reset_mouse_recover_interval();
            }
            if (usb_submit_interrupt_in_async(g_usd_mouse.dev_addr, g_usd_mouse.ep_in,
                                              g_usd_mouse.mps,
                                              g_usd_mouse.dma_buf, g_usd_mouse.dma_phys, g_mouse_layout.report_bytes)) {
                hid_mark_mouse_pending();
            } else {
                hid_clear_pending(&g_usd_mouse);
            }
        } else {
            ++g_usd_mouse.pending_polls;
            if (USB_HID_ENABLE_IDLE_PENDING_RECOVER != 0u &&
                g_usd_mouse.recover_armed &&
                hid_pending_timed_out(&g_usd_mouse,
                                      g_mouse_recover_ns,
                                      g_mouse_recover_polls)) {
                (void)usb_recover_interrupt_in(g_usd_mouse.dev_addr,
                                                g_usd_mouse.ep_in);
                hid_backoff_mouse_recover_interval();
                hid_clear_pending(&g_usd_mouse);
                g_usd_mouse.recover_armed = false;
                if (usb_submit_interrupt_in_async(g_usd_mouse.dev_addr,
                                                  g_usd_mouse.ep_in,
                                                  g_usd_mouse.mps,
                                                  g_usd_mouse.dma_buf,
                                                  g_usd_mouse.dma_phys,
                                                  g_mouse_layout.report_bytes)) {
                    hid_mark_mouse_pending();
                }
            }
        }
    } else {
        g_mouse_poll_timer++;
        if (g_mouse_poll_timer >= 10) {
            g_mouse_poll_timer = 0;
            if (hid_sync_interrupt_in(&g_usd_mouse, g_mouse_layout.report_bytes)) {
                process_mouse_report((uint8_t *)g_usd_mouse.dma_buf);
            }
        }
    }
}

void usb_hid_poll(void)
{
    poll_keyboard();
    poll_mouse();
}

int32_t usb_hid_read_keyboard(driver_keyboard_event_t *out_event)
{
    /* No usb_hid_poll() here: InputManager polls every driver once per timer
     * tick (input_manager_schedule_poll) and callers reach the queue only
     * after that poll ran -- the dispatcher consumes the pending tick before
     * dispatching any syscall. Polling per read meant several full controller
     * sweeps per window-manager loop iteration for no fresher data. */

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_kbd_lock);
    
    if (g_kbd_count == 0) {
        spinlock_unlock(&g_kbd_lock);
        irq_restore(flags);
        return 0;
    }
    *out_event = g_kbd_queue[g_kbd_tail];
    g_kbd_tail = (g_kbd_tail + 1) % USB_HID_QUEUE_SIZE;
    g_kbd_count--;
    
    spinlock_unlock(&g_kbd_lock);
    irq_restore(flags);
    return 1;
}

int32_t usb_hid_read_mouse(driver_mouse_event_t *out_event)
{
    /* Polled by InputManager on the timer tick; see read_keyboard above. */

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_mouse_lock);
    
    if (g_mouse_count == 0) {
        spinlock_unlock(&g_mouse_lock);
        irq_restore(flags);
        return 0;
    }
    
    *out_event = g_mouse_queue[g_mouse_tail];
    g_mouse_tail = (g_mouse_tail + 1) % USB_HID_QUEUE_SIZE;
    g_mouse_count--;
    
    spinlock_unlock(&g_mouse_lock);
    irq_restore(flags);
    return 1;
}

void usb_hid_drain_keyboard(driver_keyboard_event_t *tmp,
                             void (*forward)(driver_keyboard_event_t *))
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_kbd_lock);
    
    while (g_kbd_count > 0) {
        *tmp = g_kbd_queue[g_kbd_tail];
        g_kbd_tail = (g_kbd_tail + 1) % USB_HID_QUEUE_SIZE;
        g_kbd_count--;
        
        spinlock_unlock(&g_kbd_lock);
        irq_restore(flags);
        
        if (forward) forward(tmp);
        
        flags = irq_save_disable();
        spinlock_lock(&g_kbd_lock);
    }
    
    spinlock_unlock(&g_kbd_lock);
    irq_restore(flags);
}

void usb_hid_drain_mouse(driver_mouse_event_t *tmp,
                          void (*forward)(driver_mouse_event_t *))
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_mouse_lock);
    
    while (g_mouse_count > 0) {
        *tmp = g_mouse_queue[g_mouse_tail];
        g_mouse_tail = (g_mouse_tail + 1) % USB_HID_QUEUE_SIZE;
        g_mouse_count--;
        
        spinlock_unlock(&g_mouse_lock);
        irq_restore(flags);
        
        if (forward) forward(tmp);
        
        flags = irq_save_disable();
        spinlock_lock(&g_mouse_lock);
    }
    
    spinlock_unlock(&g_mouse_lock);
    irq_restore(flags);
}
