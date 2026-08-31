#pragma once
#include <stdint.h>

#define EVDEV_FD_BASE 0x7000
#define EVDEV_MAX_DEVICES 4
#define EVDEV_RING_SIZE 64

#define EV_SYN 0x00
#define EV_KEY 0x01
#define EV_REL 0x02
#define EV_ABS 0x03

#define SYN_REPORT 0
#define REL_X 0
#define REL_Y 1
#define REL_WHEEL 8
#define BTN_LEFT 0x110
#define BTN_RIGHT 0x111
#define BTN_MIDDLE 0x112

typedef struct {
    uint64_t time_sec;
    uint64_t time_usec;
    uint16_t type;
    uint16_t code;
    int32_t  value;
} input_event_t;

void evdev_init(void);
int64_t evdev_open(const char *path);
/* Returns the queued events, or -EAGAIN when the ring is empty (never 0 --
 * that would mean EOF to an evdev reader). */
int64_t evdev_read(int32_t fd, void *buf, uint64_t len);
/* Non-destructive "is there anything to read?", for poll()/select(). */
int evdev_has_events(int32_t fd);
int64_t evdev_ioctl(int32_t fd, uint64_t request, uint64_t arg);
int64_t evdev_close(int32_t fd);
void evdev_push_key_event(uint16_t code, int32_t value);
void evdev_push_rel_event(uint16_t code, int32_t value);
void evdev_push_abs_event(uint16_t code, int32_t value);
