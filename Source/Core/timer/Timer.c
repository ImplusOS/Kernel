#include "Core/process/ProcessScheduler.h"
#include "Timer.h"
#include "Core/sound/ALSA.h"
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
#include "kernel/config.h"

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

/* Latency diagnostics: where each CPU's last tick interrupted it, and how
 * long it went without one. A long gap means that CPU ran with interrupts
 * off; the tick that ends it lands just after whatever re-enabled them. */
#ifndef TIMER_GAP_TRACE
#define TIMER_GAP_TRACE 0
#endif
static uint64_t g_irq_rip[OS_CONFIG_SMP_MAX_CPUS];
static uint64_t g_tick_ns[OS_CONFIG_SMP_MAX_CPUS];

void timer_note_irq_rip(uint64_t rip)
{
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu < OS_CONFIG_SMP_MAX_CPUS) {
        g_irq_rip[cpu] = rip;
    }
}

static void timer_gap_check(uint32_t cpu)
{
    if (!TIMER_GAP_TRACE || cpu >= OS_CONFIG_SMP_MAX_CPUS ||
        __atomic_load_n(&g_timer_services_started, __ATOMIC_ACQUIRE) == 0u) {
        return;
    }
    uint64_t now = timer_monotonic_ns();
    uint64_t last = g_tick_ns[cpu];
    g_tick_ns[cpu] = now;
    static volatile uint32_t printed;
    if (last != 0u && now - last > 60000000ull &&
        __atomic_fetch_add(&printed, 1u, __ATOMIC_RELAXED) < 64u) {
        char line[96];
        snprintf(line, sizeof(line), "[tick] cpu%u gap=%llums rip=%llx\n",
                 (unsigned)cpu, (unsigned long long)((now - last) / 1000000ull),
                 (unsigned long long)g_irq_rip[cpu]);
        serial_write_string(line);
    }
}

static void timer_core_handler(void) {
    uint32_t cpu_id = smp_get_current_cpu_id();
    timer_gap_check(cpu_id);
    if (cpu_id != 0u) {
        /* The APs keep no time, but they do run tasks, and those need their
         * slices counted for the timer ISR's preemption check
         * (process_preempt_from_user_irq) to ever fire there. */
        if (__atomic_load_n(&g_timer_services_started, __ATOMIC_ACQUIRE) != 0u) {
            process_scheduler_tick_cpu();
            /* Keep audio fed when CPU0 is late with its own tick. */
            alsa_timer_tick_backup();
        }
        return;
    }

    if (__atomic_load_n(&g_timer_services_started, __ATOMIC_ACQUIRE) != 0u) {
        process_on_timer_tick();
#if PROCESS_STALL_DUMP
        {
            extern void process_stall_dump_tick(void);
            process_stall_dump_tick();
        }
#endif
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
        alsa_timer_tick();
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

#if defined(PLATFORM_X86_64)
/* A monotonic clock that does not leave the guest.
 *
 * hpet_monotonic_ns() is an MMIO read that QEMU emulates in user space, so
 * every call is a full VM exit to the QEMU process. The scheduler and the
 * poll/futex wait paths read the clock on every syscall, usually with the
 * process table lock held, and profiling Chromium starting up found the other
 * vCPUs spinning behind that lock while its holder waited on HPET reads.
 *
 * RDTSC runs natively under KVM. It is calibrated once against HPET here and
 * then used alone. KVM keeps the vCPUs' TSCs in step, but a clock must never
 * run backwards even if two of them disagree by a few ticks, so the result is
 * clamped to the largest value any CPU has returned so far. */
#define TIMER_TSC_CAL_WINDOW_NS 50000000ULL /* 50 ms */

static uint64_t g_tsc_base;
static uint64_t g_tsc_base_ns;
static uint64_t g_tsc_ns_per_tick_q32;
static volatile uint8_t g_tsc_ready;
static volatile uint64_t g_tsc_last_ns;

static inline uint64_t timer_rdtsc(void) {
    uint32_t lo, hi;
    __asm__ volatile("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | lo;
}

static void timer_tsc_calibrate(void) {
    if (!hpet_is_available()) {
        return;
    }
    uint64_t ns0 = hpet_monotonic_ns();
    uint64_t tsc0 = timer_rdtsc();
    uint64_t ns1;
    do {
        ns1 = hpet_monotonic_ns();
    } while (ns1 - ns0 < TIMER_TSC_CAL_WINDOW_NS);
    uint64_t tsc1 = timer_rdtsc();
    if (tsc1 <= tsc0) {
        return;
    }
    /* ns1 - ns0 is about 5e7, so shifting it left by 32 cannot overflow. */
    uint64_t per_tick = ((ns1 - ns0) << 32) / (tsc1 - tsc0);
    if (per_tick == 0u) {
        return;
    }
    g_tsc_ns_per_tick_q32 = per_tick;
    g_tsc_base = tsc1;
    g_tsc_base_ns = ns1;
    g_tsc_last_ns = ns1;
    __atomic_store_n(&g_tsc_ready, 1u, __ATOMIC_RELEASE);
}

static uint64_t timer_tsc_monotonic_ns(void) {
    uint64_t tsc = timer_rdtsc();
    uint64_t delta = tsc > g_tsc_base ? tsc - g_tsc_base : 0u;
    uint64_t ns = g_tsc_base_ns +
                  (uint64_t)(((unsigned __int128)delta * g_tsc_ns_per_tick_q32) >> 32);
    uint64_t last = __atomic_load_n(&g_tsc_last_ns, __ATOMIC_ACQUIRE);
    while (ns > last) {
        if (__atomic_compare_exchange_n(&g_tsc_last_ns, &last, ns, 0,
                                        __ATOMIC_ACQUIRE, __ATOMIC_ACQUIRE)) {
            return ns;
        }
    }
    return last;
}
#endif

void timer_init(const timer_hal_t* hal) {
    if (g_timer_initialized) {
        return;
    }

    g_timer_hal = hal;
    g_requested_hz = TIMER_DEFAULT_HZ;
    (void)hpet_init();
#if defined(PLATFORM_X86_64)
    timer_tsc_calibrate();
#endif

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
#if defined(PLATFORM_X86_64)
    if (__atomic_load_n(&g_tsc_ready, __ATOMIC_RELAXED) != 0u) {
        return timer_tsc_monotonic_ns();
    }
#endif
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
