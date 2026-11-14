#ifndef IMPLUSOS_TIMER_HAL_H
#define IMPLUSOS_TIMER_HAL_H

#include <stdint.h>

typedef struct {
    void     (*init)(uint32_t hz);
    uint64_t (*get_ticks)(void);
    void     (*msleep)(uint32_t ms);
    void     (*set_handler)(void (*handler)(void));
    void     (*disable_irq)(void);
    void     (*switch_to_local)(void);
    /* Optional: make sure the timer fires no later than `due_ns`
     * (timer_monotonic_ns() time) even between periodic ticks. Backends
     * without a one-shot timer leave it NULL; wake-ups then land on the next
     * tick. See timer_hires_request(). */
    void     (*hires_request)(uint64_t due_ns);
} timer_hal_t;

#endif
