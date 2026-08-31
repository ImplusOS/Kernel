#include "Evdev_Client.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "Core/usercopy/Usercopy.h"
#include <stddef.h>
#include <string.h>

typedef struct {
    uint8_t used;
    input_event_t ring[EVDEV_RING_SIZE];
    uint32_t head, tail;
    spinlock_t lock;
} evdev_device_t;

static evdev_device_t g_devs[EVDEV_MAX_DEVICES];
static int g_evdev_init_done = 0;

void evdev_init(void) {
    memset(g_devs, 0, sizeof(g_devs));
    for (int i = 0; i < EVDEV_MAX_DEVICES; i++) spinlock_init(&g_devs[i].lock);
    g_devs[0].used = 1;
    g_devs[1].used = 1;
    g_evdev_init_done = 1;
}

static evdev_device_t *evdev_get(int32_t fd) {
    int idx = fd - EVDEV_FD_BASE;
    if (idx < 0 || idx >= EVDEV_MAX_DEVICES) return NULL;
    return g_devs[idx].used ? &g_devs[idx] : NULL;
}

static void evdev_push(evdev_device_t *dev, uint16_t type, uint16_t code, int32_t value) {
    spinlock_lock(&dev->lock);
    uint32_t next = (dev->head + 1) % EVDEV_RING_SIZE;
    if (next != dev->tail) {
        uint32_t hz = timer_hz(); if (!hz) hz = 60;
        uint64_t ms = (timer_ticks() * 1000ULL) / hz;
        dev->ring[dev->head].time_sec = ms / 1000;
        dev->ring[dev->head].time_usec = (ms % 1000) * 1000;
        dev->ring[dev->head].type = type;
        dev->ring[dev->head].code = code;
        dev->ring[dev->head].value = value;
        dev->head = next;
    }
    spinlock_unlock(&dev->lock);
}

void evdev_push_key_event(uint16_t code, int32_t value) {
    if (!g_evdev_init_done) evdev_init();
    evdev_push(&g_devs[0], EV_KEY, code, value);
    evdev_push(&g_devs[0], EV_SYN, SYN_REPORT, 0);
}

void evdev_push_rel_event(uint16_t code, int32_t value) {
    if (!g_evdev_init_done) evdev_init();
    evdev_push(&g_devs[1], EV_REL, code, value);
}

void evdev_push_abs_event(uint16_t code, int32_t value) {
    if (!g_evdev_init_done) evdev_init();
    evdev_push(&g_devs[1], EV_ABS, code, value);
}

int64_t evdev_open(const char *path) {
    if (!g_evdev_init_done) evdev_init();
    (void)path;
    for (int i = 0; i < EVDEV_MAX_DEVICES; i++) {
        if (g_devs[i].used) return EVDEV_FD_BASE + i;
    }
    return -19;
}

int64_t evdev_read(int32_t fd, void *buf, uint64_t len) {
    evdev_device_t *dev = evdev_get(fd);
    if (!dev || !buf) return -14;
    uint64_t ev_size = sizeof(input_event_t);
    uint64_t count = 0;
    input_event_t *out = (input_event_t*)buf;
    spinlock_lock(&dev->lock);
    while (count + ev_size <= len && dev->tail != dev->head) {
        out[count / ev_size] = dev->ring[dev->tail];
        dev->tail = (dev->tail + 1) % EVDEV_RING_SIZE;
        count += ev_size;
    }
    spinlock_unlock(&dev->lock);
    /* An empty ring is EAGAIN, never 0. read() returning 0 on a character
     * device means end-of-file, and xf86-input-evdev reads that as the device
     * having been unplugged: it disables the device and stops polling it. */
    if (count == 0u) {
        return -11; /* -EAGAIN */
    }
    return (int64_t)count;
}

int evdev_has_events(int32_t fd) {
    evdev_device_t *dev = evdev_get(fd);
    if (!dev) return 0;
    spinlock_lock(&dev->lock);
    int ready = (dev->tail != dev->head);
    spinlock_unlock(&dev->lock);
    return ready;
}

/* --- Linux evdev ioctl subset for the X "evdev" input driver -------------
 * type 'E' (0x45). We decode nr and (for the variable-length "get" calls)
 * the size field from the _IOC-encoded request. dev 0 = keyboard, dev 1 =
 * relative pointer. Enough for xf86-input-evdev to classify the devices and
 * start reading events; not a complete implementation. */
#define IOC_NR(c)   ((uint32_t)((c) >> 0)  & 0xffu)
#define IOC_TYPE(c) ((uint32_t)((c) >> 8)  & 0xffu)
#define IOC_SIZE(c) ((uint32_t)((c) >> 16) & 0x3fffu)

/* _IOC direction field: bit 30 = read (kernel -> user), bit 31 = write. */
#define IOC_DIR_IS_WRITE(c) ((((uint32_t)((c) >> 30)) & 0x3u) == 0x1u)

#define EVIOC_TYPE 0x45u

/* bit set helper into a little-endian bitmap */
static void bm_set(uint8_t *bm, uint32_t bit) { bm[bit >> 3] |= (uint8_t)(1u << (bit & 7u)); }

static int64_t evdev_fill_bits(uint64_t arg, uint32_t size, uint32_t evtype, int is_kbd)
{
    uint8_t bm[128];
    memset(bm, 0, sizeof(bm));
    if (size > sizeof(bm)) size = sizeof(bm);

    if (evtype == 0u) {
        /* EVIOCGBIT(0): which EV_* types this device reports */
        bm_set(bm, EV_SYN);
        bm_set(bm, EV_KEY);
        /* EV_REP only: the server derives key repeat from EVIOCGREP, which we
         * answer. EV_LED is deliberately NOT claimed -- there is no LED to
         * drive, and claiming it makes the server run its keyboard-LED
         * control path, which writes input_events back to the device node
         * (something this shim does not implement). */
        if (is_kbd) { bm_set(bm, 0x14 /*EV_REP*/); }
        else        { bm_set(bm, EV_REL); }
    } else if (evtype == EV_KEY) {
        if (is_kbd) {
            for (uint32_t k = 1; k < 248; k++) bm_set(bm, k); /* KEY_ESC..KEY_MICMUTE-ish */
        } else {
            bm_set(bm, BTN_LEFT); bm_set(bm, BTN_RIGHT); bm_set(bm, BTN_MIDDLE);
        }
    } else if (evtype == EV_REL && !is_kbd) {
        bm_set(bm, REL_X); bm_set(bm, REL_Y); bm_set(bm, REL_WHEEL);
    } else {
        /* nothing for this type */
    }
    if (copy_to_user((void *)(uintptr_t)arg, bm, size) != 0u) return -14;
    return (int64_t)size;
}

int64_t evdev_ioctl(int32_t fd, uint64_t request, uint64_t arg) {
    int idx = fd - EVDEV_FD_BASE;
    if (idx < 0 || idx >= EVDEV_MAX_DEVICES) return -19;
    int is_kbd = (idx == 0);
    if (IOC_TYPE(request) != EVIOC_TYPE) return -25;
    uint32_t nr = IOC_NR(request);
    uint32_t sz = IOC_SIZE(request);

    if (nr == 0x01) { /* EVIOCGVERSION -> int */
        int32_t v = 0x010001;
        return (arg && copy_to_user((void *)(uintptr_t)arg, &v, sizeof(v)) == 0u) ? 0 : -14;
    }
    if (nr == 0x02) { /* EVIOCGID -> struct input_id {u16 bustype,vendor,product,version} */
        uint16_t id[4] = { 0x0019 /*BUS_HOST*/, 0x1234, (uint16_t)(is_kbd ? 1u : 2u), 1u };
        return (arg && copy_to_user((void *)(uintptr_t)arg, id, sizeof(id)) == 0u) ? 0 : -14;
    }
    if (nr == 0x03) { /* EVIOCGREP / EVIOCSREP -> unsigned int[2] */
        /* libevdev_set_fd() issues this for any device whose EVIOCGBIT(0)
         * advertises EV_REP -- i.e. the keyboard, and only the keyboard. It
         * used to fall through to the ENOTTY below, which failed the whole
         * libevdev_set_fd() and cost us kbd0:
         *   (EE) evdev: kbd0: Unable to query fd: Inappropriate ioctl for device
         *   (EE) PreInit returned 2 for "kbd0"
         * Report Linux's own defaults: 250 ms to first repeat, 33 ms between.
         * Repeat is generated by the X server from these, so a set is simply
         * accepted. */
        uint32_t rep[2] = { 250u, 33u };
        if (IOC_DIR_IS_WRITE(request)) {
            return 0;
        }
        return (arg && copy_to_user((void *)(uintptr_t)arg, rep, sizeof(rep)) == 0u) ? 0 : -14;
    }
    if (nr == 0x06 || nr == 0x07 || nr == 0x08) { /* EVIOCGNAME/PHYS/UNIQ(len) */
        const char *s = (nr == 0x06) ? (is_kbd ? "ImplusOS Keyboard" : "ImplusOS Pointer")
                                     : (nr == 0x07 ? (is_kbd ? "implus/input0" : "implus/input1") : "");
        uint32_t n = 0; while (s[n]) n++; n++;
        if (n > sz) n = sz;
        if (!arg || copy_to_user((void *)(uintptr_t)arg, s, n) != 0u) return -14;
        return (int64_t)n;
    }
    if (nr == 0x09) { /* EVIOCGPROP(len) - no INPUT_PROP_* */
        uint8_t z[16]; memset(z, 0, sizeof(z));
        if (sz > sizeof(z)) sz = sizeof(z);
        if (arg && copy_to_user((void *)(uintptr_t)arg, z, sz) != 0u) return -14;
        return (int64_t)sz;
    }
    if (nr >= 0x20 && nr <= 0x20 + 0x1f) { /* EVIOCGBIT(ev,len) */
        return arg ? evdev_fill_bits(arg, sz, nr - 0x20, is_kbd) : -14;
    }
    if (nr == 0x18 || nr == 0x19 || nr == 0x1b) { /* EVIOCGKEY/LED/SW -> zeroed */
        /* 128 bytes covers the largest of these libevdev asks for (KEY_CNT
         * bits = 96); the old 64 left the caller's buffer tail untouched
         * while still reporting success. */
        uint8_t z[128]; memset(z, 0, sizeof(z));
        if (sz > sizeof(z)) sz = sizeof(z);
        if (arg && copy_to_user((void *)(uintptr_t)arg, z, sz) != 0u) return -14;
        return (int64_t)sz;
    }
    if (nr >= 0x40 && nr <= 0x40 + 0x3f) { /* EVIOCGABS(abs) - we have no abs axes */
        uint8_t z[24]; memset(z, 0, sizeof(z));
        if (arg && copy_to_user((void *)(uintptr_t)arg, z, sizeof(z)) != 0u) return -14;
        return 0;
    }
    if (nr == 0x90 || nr == 0x91 || nr == 0xa0) { /* EVIOCGRAB / REVOKE / SCLOCKID */
        return 0;
    }
    return -25; /* ENOTTY: unhandled */
}

int64_t evdev_close(int32_t fd) {
    (void)fd;
    return 0;
}
