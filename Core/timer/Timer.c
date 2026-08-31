#include "Timer.h"
#include <stddef.h>
#include <stdint.h>

#include "interfaces/timer_hal.h"
#include "Core/process/ProcessManager.h"
#include "Core/syscall/Syscall_Futex.h"
#include "Drivers/Module/DriverManager.h"
#include "Drivers/Module/InputManager.h"
#include "Network/network_main.h"
#include "smp/SMP_Main.h"
#include "Platform/timer/HPET.h"
#include "Debug/serial/Serial.h"

#include <stdio.h>

static const timer_hal_t* g_timer_hal = NULL;
static volatile uint64_t g_tick_count = 0;
static timer_callback_t g_tick_callback = NULL;
static int g_timer_initialized = 0;
static volatile uint32_t g_timer_clock_started = 0;
static volatile uint32_t g_timer_services_started = 0;
#define TIMER_DEFAULT_HZ 250u
#define TIMER_ENABLE_PERIODIC_PERF_LOGS 0u
#define TIMER_DEBUG_LOG_TICK_MULTIPLIER 2u
#define TIMER_RUN_DRIVER_POLLS_IN_IRQ 0u

#if TIMER_RUN_DRIVER_POLLS_IN_IRQ != 0u || TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
static volatile uint32_t g_timer_async_poll_active = 0;
#endif
#if TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
static volatile uint64_t g_timer_async_poll_runs = 0;
static volatile uint64_t g_timer_async_poll_busy = 0;
static volatile uint64_t g_timer_async_poll_total_ns = 0;
static volatile uint64_t g_timer_async_poll_max_ns = 0;
static uint64_t g_timer_cpu_log_last_ns = 0;
#endif
static uint32_t g_requested_hz = 0;

#if TIMER_RUN_DRIVER_POLLS_IN_IRQ != 0u || TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
static void timer_run_async_polls(void)
{
    if (__atomic_exchange_n(&g_timer_async_poll_active, 1u,
                            __ATOMIC_ACQUIRE) != 0u) {
#if TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
        __atomic_fetch_add(&g_timer_async_poll_busy, 1u, __ATOMIC_RELAXED);
#endif
        return;
    }

#if TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
    uint64_t start_ns = timer_monotonic_ns();
#endif
#if TIMER_RUN_DRIVER_POLLS_IN_IRQ != 0u
    input_manager_poll();

    if (network_stack_check_poll()) {
        network_stack_poll();
    }
#endif
#if TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
    uint64_t end_ns = timer_monotonic_ns();
    uint64_t duration_ns = (end_ns >= start_ns) ? (end_ns - start_ns) : 0u;
    __atomic_fetch_add(&g_timer_async_poll_runs, 1u, __ATOMIC_RELAXED);
    __atomic_fetch_add(&g_timer_async_poll_total_ns, duration_ns,
                       __ATOMIC_RELAXED);

    uint64_t max_ns = __atomic_load_n(&g_timer_async_poll_max_ns,
                                      __ATOMIC_RELAXED);
    while (duration_ns > max_ns &&
           !__atomic_compare_exchange_n(&g_timer_async_poll_max_ns,
                                        &max_ns,
                                        duration_ns,
                                        0,
                                        __ATOMIC_RELAXED,
                                        __ATOMIC_RELAXED)) {
    }
#endif

    __atomic_store_n(&g_timer_async_poll_active, 0u, __ATOMIC_RELEASE);
}
#endif

#if TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
static void timer_debug_dump_async_poll(uint64_t interval_ns)
{
    uint64_t runs = __atomic_exchange_n(&g_timer_async_poll_runs, 0u,
                                        __ATOMIC_ACQ_REL);
    uint64_t busy = __atomic_exchange_n(&g_timer_async_poll_busy, 0u,
                                        __ATOMIC_ACQ_REL);
    uint64_t total_ns = __atomic_exchange_n(&g_timer_async_poll_total_ns, 0u,
                                            __ATOMIC_ACQ_REL);
    uint64_t max_ns = __atomic_exchange_n(&g_timer_async_poll_max_ns, 0u,
                                          __ATOMIC_ACQ_REL);
    uint64_t pct_x100 = 0u;
    if (interval_ns != 0u) {
        pct_x100 = (total_ns * 10000ULL) / interval_ns;
    }
    uint64_t avg_us = runs != 0u ? (total_ns / runs) / 1000ULL : 0u;

    char line[192];
    snprintf(line, sizeof(line),
             "[timer:async] interval_ms=%llu runs=%llu busy=%llu total=%llu.%02llu%% avg_us=%llu max_us=%llu\n",
             (unsigned long long)(interval_ns / 1000000ULL),
             (unsigned long long)runs,
             (unsigned long long)busy,
             (unsigned long long)(pct_x100 / 100ULL),
             (unsigned long long)(pct_x100 % 100ULL),
             (unsigned long long)avg_us,
             (unsigned long long)(max_ns / 1000ULL));
    serial_write_string(line);
}
#endif

static void timer_core_handler(void) {
    uint32_t cpu_id = smp_get_current_cpu_id();
    if (cpu_id != 0u) {
        return;
    }

    if (__atomic_load_n(&g_timer_services_started, __ATOMIC_ACQUIRE) != 0u) {
        process_on_timer_tick();
    }

    g_tick_count++;

    if (__atomic_load_n(&g_timer_clock_started, __ATOMIC_ACQUIRE) != 0u) {
        timer_callback_t cb = g_tick_callback;
        if (cb) {
            cb(g_tick_count);
        }
    }

    if (__atomic_load_n(&g_timer_services_started, __ATOMIC_ACQUIRE) != 0u) {
        syscall_futex_on_timer_tick();
        input_manager_schedule_poll();
        uint32_t hotplug_interval = g_requested_hz != 0u ?
                                    g_requested_hz :
                                    TIMER_DEFAULT_HZ;
        if (hotplug_interval != 0u &&
            (g_tick_count % hotplug_interval) == 0u) {
            driver_manager_schedule_hotplug_poll();
        }
        network_stack_on_timer_tick();
#if TIMER_RUN_DRIVER_POLLS_IN_IRQ != 0u || TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
        timer_run_async_polls();
#endif
#if TIMER_ENABLE_PERIODIC_PERF_LOGS != 0u
        uint32_t log_interval = g_requested_hz != 0u ?
                                g_requested_hz :
                                TIMER_DEFAULT_HZ;
        log_interval *= TIMER_DEBUG_LOG_TICK_MULTIPLIER;
        if (log_interval != 0u && (g_tick_count % log_interval) == 0u) {
            uint64_t now_ns = timer_monotonic_ns();
            uint64_t interval_ns = 1000000000ULL;
            if (g_timer_cpu_log_last_ns != 0u &&
                now_ns >= g_timer_cpu_log_last_ns) {
                interval_ns = now_ns - g_timer_cpu_log_last_ns;
            }
            g_timer_cpu_log_last_ns = now_ns;
            process_debug_dump_cpu_usage("periodic", interval_ns);
            timer_debug_dump_async_poll(interval_ns);
        }
#endif
    }
}

void timer_init(const timer_hal_t* hal) {
    if (g_timer_initialized) {
        return;
    }

    g_timer_hal = hal;
    g_requested_hz = TIMER_DEFAULT_HZ;
    (void)hpet_init();

    if (g_timer_hal && g_timer_hal->init) {
        if (g_timer_hal->set_handler) {
            g_timer_hal->set_handler(timer_core_handler);
        }
        g_timer_hal->init(g_requested_hz);
    }

    g_timer_initialized = 1;
}

void timer_start_clock(void) {
    if (!g_timer_initialized) {
        return;
    }
    if (__atomic_load_n(&g_timer_clock_started, __ATOMIC_ACQUIRE) != 0u) {
        return;
    }
    __atomic_store_n(&g_timer_clock_started, 1u, __ATOMIC_RELEASE);
}

void timer_start_services(void) {
    timer_start_clock();
    __atomic_store_n(&g_timer_services_started, 1u, __ATOMIC_RELEASE);
}

void timer_set_callback(timer_callback_t cb) {
    g_tick_callback = cb;
}

uint64_t timer_ticks(void) {
    if (g_timer_hal && g_timer_hal->get_ticks) {
        return g_timer_hal->get_ticks();
    }
    return g_tick_count;
}

uint32_t timer_hz(void) {
    return (g_requested_hz != 0) ? g_requested_hz : TIMER_DEFAULT_HZ;
}

uint64_t timer_monotonic_ns(void) {
    if (hpet_is_available()) {
        return hpet_monotonic_ns();
    }
    uint32_t hz = timer_hz();
    uint64_t ticks = timer_ticks();
    if (hz == 0u) {
        return 0u;
    }
    return (ticks / hz) * 1000000000ULL +
           ((ticks % hz) * 1000000000ULL) / hz;
}

void timer_disable_irq0(void) {
    if (g_timer_hal && g_timer_hal->disable_irq) {
        g_timer_hal->disable_irq();
    }
}

void timer_switch_lapic(void) {
    if (g_timer_hal && g_timer_hal->switch_to_local) {
        g_timer_hal->switch_to_local();
    }
}

void timer_apic_sleep_ms(uint32_t ms) {
    if (g_timer_hal && g_timer_hal->msleep) {
        g_timer_hal->msleep(ms);
    }
}
