#include "interfaces/timer_hal.h"
#include <stdint.h>
#include <stddef.h>

#include "cpu/IDT_Main.h"
#include "Platform/io/IO_Main.h"
#include "Platform/interrupt/LAPIC.h"
#include "Platform/interrupt/Interrupts.h"
#include "Arch/x86_64/smp/SMP_Main.h"
#include "Debug/serial/Serial.h"
#include "interfaces/hal_cpu.h"
#include "Platform/timer/HPET.h"

#define PIT_CHANNEL0_DATA 0x40
#define PIT_COMMAND       0x43
#define PIT_BASE_FREQ     1193182U

static uint32_t g_timer_hz = 0;
static void (*g_timer_callback)(void) = NULL;
static volatile uint64_t g_ticks = 0;
static uint32_t g_lapic_timer_initial = 0;

static void pit_set_frequency(uint32_t hz) {
    if (hz == 0) return;

    uint32_t clamped_hz = hz;
    if (clamped_hz < 10u)  clamped_hz = 10u;
    if (clamped_hz > 1000u) clamped_hz = 1000u;

    uint32_t divisor32 = PIT_BASE_FREQ / clamped_hz;
    if (divisor32 == 0u) divisor32 = 1u;
    if (divisor32 > 0xFFFFu) divisor32 = 0xFFFFu;

    uint16_t divisor = (uint16_t)divisor32;

    outb(PIT_COMMAND, 0x36);
    outb(PIT_CHANNEL0_DATA, (uint8_t)(divisor & 0xFF));
    outb(PIT_CHANNEL0_DATA, (uint8_t)((divisor >> 8) & 0xFF));

    g_timer_hz = clamped_hz;
}

/* Is the BSP-only guard below actually holding, and does the tick counter
 * keep real time? Enable with -DTIMER_TICK_TRACE=1: every 2000 ticks it
 * prints the HPET reading (an independent clock), the tick count, and how
 * many times each physical LAPIC id has entered this handler. A tick counter
 * that runs fast makes every guest CLOCK_MONOTONIC reading, every
 * timer_msleep() and every driver settle delay wrong by the same factor. */
#ifndef TIMER_TICK_TRACE
#define TIMER_TICK_TRACE 0
#endif
#if TIMER_TICK_TRACE
#define TIMER_TICK_TRACE_LAPICS 16u
static volatile uint32_t g_tick_by_lapic[TIMER_TICK_TRACE_LAPICS];
#endif

static void lapic_timer_handler(void) {
#if TIMER_TICK_TRACE
    {
        uint32_t lid = lapic_is_present() ? lapic_get_id() : 0u;
        if (lid < TIMER_TICK_TRACE_LAPICS) {
            uint32_t n = __atomic_add_fetch(&g_tick_by_lapic[lid], 1u,
                                            __ATOMIC_RELAXED);
            /* Printed per CPU, outside the BSP guard below, because the
             * failure being chased is "one CPU's LAPIC timer stops firing"
             * and a trace that only runs on the BSP cannot tell that apart
             * from the whole subsystem dying. Lines from different CPUs
             * interleave; the lid= field is what makes them readable. */
            if ((n % 4000u) == 0u) {
                serial_write_string("[tk] lid=");
                serial_write_uint32(lid);
                serial_write_string(" n=");
                serial_write_uint32(n);
                serial_write_string(" hpet_ms=");
                serial_write_uint64(hpet_is_available()
                                        ? (hpet_monotonic_ns() / 1000000ull)
                                        : 0ull);
                serial_write_string("\n");
            }
        }
    }
#endif
    if (smp_get_current_cpu_id() != 0u) {
        return;
    }
    __atomic_fetch_add(&g_ticks, 1u, __ATOMIC_RELAXED);
#if TIMER_TICK_TRACE
    if ((g_ticks % 2000u) == 0u) {
        serial_write_string("[tick] hpet_ms=");
        serial_write_uint64(hpet_is_available() ? (hpet_monotonic_ns() / 1000000ull)
                                                : 0ull);
        serial_write_string(" ticks=");
        serial_write_uint64(g_ticks);
        serial_write_string(" hz=");
        serial_write_uint32(g_timer_hz);
        serial_write_string(" bylapic=");
        for (uint32_t i = 0; i < 8u; ++i) {
            serial_write_uint32(g_tick_by_lapic[i]);
            serial_write_string(",");
        }
        serial_write_string("\n");
    }
#endif
    if (g_timer_callback) {
        g_timer_callback();
    }
}

static void lapic_timer_hal_init(uint32_t hz) {
    g_timer_hz = hz;
    register_interrupt_handler(VECTOR_TIMER, lapic_timer_handler);
    
    pit_set_frequency(hz);

    if (!platform_interrupts_using_lapic()) {
        uint8_t master_mask = inb(0x21);
        master_mask &= (uint8_t)~0x01u;
        outb(0x21, master_mask);
    }

    platform_interrupts_route_pit();
}

static uint64_t lapic_get_ticks(void) {
    return g_ticks;
}

static void lapic_msleep(uint32_t ms) {
    if (ms == 0) return;

    uint64_t wait_ticks = ((uint64_t)ms * g_timer_hz + 999) / 1000;
    uint64_t end = g_ticks + wait_ticks;

    /* g_ticks only advances on a timer interrupt, so this loop can never make
     * progress between ticks anyway. When interrupts are enabled (the common
     * case: driver reset/settle delays of 1-100 ms run in process/kthread
     * context) halt until the next tick instead of spinning `pause`, which
     * otherwise pins a core at 100% for the whole delay. Fall back to `pause`
     * when IF=0 (early boot / IRQ context / spinlock held) where `hlt` would
     * wedge. */
    uint64_t rflags;
    __asm__ volatile("pushfq; popq %0" : "=r"(rflags) :: "memory");
    int irqs_enabled = (rflags & (1ull << 9)) != 0;

    while (g_ticks < end) {
        if (irqs_enabled) {
            __asm__ volatile("hlt");
        } else {
            __asm__ volatile("pause" ::: "memory");
        }
    }
}

static void lapic_set_handler(void (*handler)(void)) {
    g_timer_callback = handler;
}

static void lapic_disable_irq(void) {
    platform_interrupts_mask_pit();

    if (!platform_interrupts_using_lapic()) {
        uint8_t master_mask = inb(0x21);
        master_mask |= 0x01u;
        outb(0x21, master_mask);
    }
}

/* How long to count the LAPIC timer for while working out its frequency, and
 * the narrowest credible answer. */
#define LAPIC_CAL_WINDOW_NS   50000000ull   /* 50 ms */
#define LAPIC_CAL_MIN_TICKS   1000u

/* Hand periodic timing over from the PIT to this CPU's own LAPIC timer.
 *
 * The frequency has to be measured, because the LAPIC timer counts at a
 * bus/core-derived rate nobody reports. It used to be measured against the
 * PIT: start the LAPIC counting, spin until g_ticks has advanced 10, and call
 * that 10 timer periods. Two things made that unsafe, and together they are
 * why the timer sometimes ran at thousands of hertz instead of the requested
 * two hundred and fifty.
 *
 *  - g_ticks was sampled *before* lapic_timer_start(), so part of the window
 *   was already gone when counting began. That alone is a ~10% error.
 *  - far worse, g_ticks advancing 10 does not mean 10 periods of wall time
 *   passed. Boot runs long stretches with interrupts masked, and the PIT
 *   interrupts queued behind them arrive back to back the moment interrupts
 *   come back on. The window then collapses to microseconds, `elapsed` comes
 *   out tiny, and the LAPIC gets programmed with a period that much too
 *   short. The `elapsed < 1000` guard let a 100x error through.
 *
 * Measured in QEMU, the result swung between 250 Hz, 286 Hz and 4439 Hz from
 * one boot to the next, which is most of why Chromium's startup time varied
 * by an order of magnitude between otherwise identical runs.
 *
 * So calibrate against the HPET instead: it is a free-running counter that
 * cannot be delayed or backlogged, and timer_monotonic_ns() already trusts
 * it. Only when there is no HPET does this fall back to the PIT, and then
 * with the tick sampled after the counter is running.
 * See Docs/Others/TODO_Chromium_LinuxABI.md section 10.-5. */
static void lapic_switch_to_local(void) {
    if (!platform_interrupts_using_lapic() || !lapic_is_present()) {
        return;
    }
    if (g_timer_hz == 0u) {
        return;
    }

    uint32_t initial = 0u;

    if (hpet_is_available()) {
        /* One-shot at full count: 2^32 ticks is over a minute even at the
         * fastest plausible rate, so the counter cannot wrap inside the
         * window and cannot fire its vector while we are measuring. */
        if (lapic_timer_start(VECTOR_TIMER, 0xFFFFFFFFu, 0, 16u) != 0) {
            return;
        }
        uint64_t start_ns = hpet_monotonic_ns();
        uint32_t start_count = lapic_timer_current();
        while ((hpet_monotonic_ns() - start_ns) < LAPIC_CAL_WINDOW_NS) {
            hal_cpu_pause();
        }
        uint32_t end_count = lapic_timer_current();
        uint64_t end_ns = hpet_monotonic_ns();
        lapic_timer_stop();

        if (end_count >= start_count || end_ns <= start_ns) {
            return; /* Counter did not move the way it must; leave the PIT. */
        }
        uint64_t counted = (uint64_t)(start_count - end_count);
        uint64_t span_ns = end_ns - start_ns;
        if (counted < LAPIC_CAL_MIN_TICKS) {
            return;
        }
        uint64_t per_second = (counted * 1000000000ull) / span_ns;
        uint64_t count = per_second / (uint64_t)g_timer_hz;
        if (count == 0u || count > 0xFFFFFFFFull) {
            return;
        }
        initial = (uint32_t)count;
    } else {
        if (lapic_timer_start(VECTOR_TIMER, 0xFFFFFFFFu, 0, 16u) != 0) {
            return;
        }
        /* Sampled after the counter is running, so the whole window is
         * counted. Still vulnerable to a PIT backlog -- there is nothing
         * better to compare against on a machine with no HPET. */
        uint64_t start_tick = g_ticks;
        uint64_t target = start_tick + 10u;
        while (g_ticks < target) {
            hal_cpu_pause();
        }
        uint32_t elapsed = 0xFFFFFFFFu - lapic_timer_current();
        lapic_timer_stop();
        if (elapsed < LAPIC_CAL_MIN_TICKS) {
            return;
        }
        initial = elapsed / 10u;
    }

    serial_write_string("[lapic] timer initial=");
    serial_write_uint32(initial);
    serial_write_string(" hz=");
    serial_write_uint32(g_timer_hz);
    serial_write_string("\n");

    g_lapic_timer_initial = initial;
    platform_interrupts_mask_pit();
    (void)lapic_timer_start(VECTOR_TIMER, initial, 1, 16u);
}

void lapic_timer_ap_init(void) {
    while (__atomic_load_n(&g_lapic_timer_initial, __ATOMIC_ACQUIRE) == 0u) {
        hal_cpu_pause();
    }
    (void)lapic_timer_start(VECTOR_TIMER, g_lapic_timer_initial, 1, 16u);
}

const timer_hal_t lapic_timer_hal = {
    .init = lapic_timer_hal_init,
    .get_ticks = lapic_get_ticks,
    .msleep = lapic_msleep,
    .set_handler = lapic_set_handler,
    .disable_irq = lapic_disable_irq,
    .switch_to_local = lapic_switch_to_local
};
