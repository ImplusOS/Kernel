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
} timer_hal_t;

#endif
