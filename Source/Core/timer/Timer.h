#pragma once

#include <stdint.h>

#include "interfaces/timer_hal.h"

typedef void (*timer_callback_t)(uint64_t tick);

void timer_init(const timer_hal_t* hal);
void timer_start_clock(void);
void timer_start_services(void);
void timer_set_callback(timer_callback_t cb);
uint64_t timer_ticks(void);
uint32_t timer_hz(void);
uint64_t timer_monotonic_ns(void);
void timer_disable_irq0(void);
void timer_switch_lapic(void);
void timer_apic_sleep_ms(uint32_t ms);

/* ---- high-resolution wake-ups --------------------------------------------
 * The periodic tick is 4 ms. Where the timer backend can do better (x86_64:
 * the BSP's LAPIC runs one-shot, see LAPIC_Timer.c) it also fires at the
 * earliest deadline a client registered, so a vblank event or wait is
 * delivered at its vblank rather than at the next tick.
 *   timer_hires_request()   a client lowered its deadline to `due_ns`
 *   timer_hires_next_due_ns() / timer_hires_service()  called by the backend
 *                           from the timer interrupt */
void timer_hires_request(uint64_t due_ns);
uint64_t timer_hires_next_due_ns(void);
void timer_hires_service(uint64_t now_ns);
