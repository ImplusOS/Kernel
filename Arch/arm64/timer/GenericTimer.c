#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "interfaces/timer_hal.h"
#include "interfaces/hal_cpu.h"
#include "Arch/arm64/interrupt/GIC.h"
#include "Arch/arm64/cpu/IDT_Main.h"
#include "Debug/serial/Serial.h"

#define CNT_CTL_ENABLE  (1u << 0)
#define CNT_CTL_IMASK   (1u << 1)
#define CNT_CTL_ISTATUS (1u << 2)

#define ARM_GENERIC_TIMER_VIRTUAL_IRQ  27u
#define ARM_GENERIC_TIMER_PHYSICAL_IRQ 30u

#define DEFAULT_TIMER_HZ          60u
#define TIMER_FALLBACK_FREQ       1000000u
#define TIMER_MIN_VALID_FREQ      1000u
#define TIMER_MAX_VALID_FREQ      1000000000u
#define TIMER_COUNTER_PROBE_SPINS 100000u
#define TIMER_COUNTER_STALE_SPINS 100000u
#define TIMER_SPIN_FALLBACK_PER_MS 4096u

typedef struct {
    const char *name;
    uint32_t irq;
    uint64_t (*read_count)(void);
    uint32_t (*read_ctl)(void);
    void (*write_tval)(uint32_t value);
    void (*write_ctl)(uint32_t value);
} arm64_timer_bank_t;

typedef enum {
    ARM64_TIMER_COUNTER_NONE = 0,
    ARM64_TIMER_COUNTER_VIRTUAL,
    ARM64_TIMER_COUNTER_PHYSICAL,
} arm64_timer_counter_source_t;

static void (*g_timer_callback)(void) = NULL;
static volatile uint64_t g_ticks = 0;
static uint64_t g_timer_freq = TIMER_FALLBACK_FREQ;
static uint32_t g_interval_count = 1;
static uint32_t g_hz = DEFAULT_TIMER_HZ;
static const arm64_timer_bank_t *g_tick_bank = NULL;
static arm64_timer_counter_source_t g_tick_counter_source = ARM64_TIMER_COUNTER_NONE;
static arm64_timer_counter_source_t g_delay_counter_source = ARM64_TIMER_COUNTER_NONE;
static uint64_t g_counter_base_count = 0;
static uint64_t g_counter_base_ticks = 0;
static volatile uint32_t g_irq_enabled = 0;

static inline uint64_t read_cntfrq_el0(void)
{
    uint64_t val;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(val));
    return val;
}

static uint64_t read_cntvct_el0(void)
{
    uint64_t val;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(val));
    return val;
}

static uint64_t read_cntpct_el0(void)
{
    uint64_t val;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(val));
    return val;
}

static uint32_t read_cntv_ctl_el0(void)
{
    uint64_t val;
    __asm__ volatile("mrs %0, cntv_ctl_el0" : "=r"(val));
    return (uint32_t)val;
}

static uint32_t read_cntp_ctl_el0(void)
{
    uint64_t val;
    __asm__ volatile("mrs %0, cntp_ctl_el0" : "=r"(val));
    return (uint32_t)val;
}

static void write_cntv_tval_el0(uint32_t val)
{
    __asm__ volatile("msr cntv_tval_el0, %0" :: "r"((uint64_t)val));
    __asm__ volatile("isb" ::: "memory");
}

static void write_cntp_tval_el0(uint32_t val)
{
    __asm__ volatile("msr cntp_tval_el0, %0" :: "r"((uint64_t)val));
    __asm__ volatile("isb" ::: "memory");
}

static void write_cntv_ctl_el0(uint32_t val)
{
    __asm__ volatile("msr cntv_ctl_el0, %0" :: "r"((uint64_t)val));
    __asm__ volatile("isb" ::: "memory");
}

static void write_cntp_ctl_el0(uint32_t val)
{
    __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"((uint64_t)val));
    __asm__ volatile("isb" ::: "memory");
}

static const arm64_timer_bank_t g_virtual_timer_bank = {
    .name = "virtual",
    .irq = ARM_GENERIC_TIMER_VIRTUAL_IRQ,
    .read_count = read_cntvct_el0,
    .read_ctl = read_cntv_ctl_el0,
    .write_tval = write_cntv_tval_el0,
    .write_ctl = write_cntv_ctl_el0,
};

static const arm64_timer_bank_t g_physical_timer_bank = {
    .name = "physical",
    .irq = ARM_GENERIC_TIMER_PHYSICAL_IRQ,
    .read_count = read_cntpct_el0,
    .read_ctl = read_cntp_ctl_el0,
    .write_tval = write_cntp_tval_el0,
    .write_ctl = write_cntp_ctl_el0,
};

static uint64_t sanitize_timer_freq(uint64_t freq)
{
    if (freq < TIMER_MIN_VALID_FREQ || freq > TIMER_MAX_VALID_FREQ) {
        return TIMER_FALLBACK_FREQ;
    }
    return freq;
}

static void timer_bank_disable(const arm64_timer_bank_t *bank)
{
    if (bank != NULL) {
        bank->write_ctl(CNT_CTL_IMASK);
    }
}

static bool timer_bank_counter_advances(const arm64_timer_bank_t *bank)
{
    uint64_t start = bank->read_count();
    for (uint32_t i = 0; i < TIMER_COUNTER_PROBE_SPINS; ++i) {
        if (bank->read_count() != start) {
            return true;
        }
        __asm__ volatile("yield" ::: "memory");
    }
    return false;
}

static uint64_t timer_read_counter_source(arm64_timer_counter_source_t source)
{
    switch (source) {
        case ARM64_TIMER_COUNTER_VIRTUAL:
            return read_cntvct_el0();
        case ARM64_TIMER_COUNTER_PHYSICAL:
            return read_cntpct_el0();
        case ARM64_TIMER_COUNTER_NONE:
        default:
            return 0;
    }
}

static const char *timer_counter_source_name(arm64_timer_counter_source_t source)
{
    switch (source) {
        case ARM64_TIMER_COUNTER_VIRTUAL:
            return "virtual-counter";
        case ARM64_TIMER_COUNTER_PHYSICAL:
            return "physical-counter";
        case ARM64_TIMER_COUNTER_NONE:
        default:
            return "fallback";
    }
}

static void select_timer_banks(void)
{
    bool virtual_counter = timer_bank_counter_advances(&g_virtual_timer_bank);
    bool physical_counter = timer_bank_counter_advances(&g_physical_timer_bank);

    if (virtual_counter) {
        g_tick_bank = &g_virtual_timer_bank;
        g_tick_counter_source = ARM64_TIMER_COUNTER_VIRTUAL;
    } else if (physical_counter) {
        g_tick_bank = &g_physical_timer_bank;
        g_tick_counter_source = ARM64_TIMER_COUNTER_PHYSICAL;
    } else {
        g_tick_bank = &g_virtual_timer_bank;
        g_tick_counter_source = ARM64_TIMER_COUNTER_VIRTUAL;
    }

    if (g_tick_counter_source == ARM64_TIMER_COUNTER_VIRTUAL && physical_counter) {
        g_delay_counter_source = ARM64_TIMER_COUNTER_PHYSICAL;
    } else if (g_tick_counter_source == ARM64_TIMER_COUNTER_PHYSICAL && virtual_counter) {
        g_delay_counter_source = ARM64_TIMER_COUNTER_VIRTUAL;
    } else if (virtual_counter) {
        g_delay_counter_source = ARM64_TIMER_COUNTER_VIRTUAL;
    } else if (physical_counter) {
        g_delay_counter_source = ARM64_TIMER_COUNTER_PHYSICAL;
    } else {
        g_delay_counter_source = ARM64_TIMER_COUNTER_NONE;
    }

    timer_bank_disable(&g_virtual_timer_bank);
    timer_bank_disable(&g_physical_timer_bank);
}

static uint64_t timer_counts_for_ms(uint32_t ms)
{
    uint64_t counts = (((uint64_t)ms * g_timer_freq) + 999u) / 1000u;
    return (counts != 0) ? counts : 1u;
}

static uint64_t timer_soft_ticks_for_ms(uint32_t ms)
{
    if (g_hz == 0) {
        return 0;
    }
    uint64_t ticks = (((uint64_t)ms * (uint64_t)g_hz) + 999u) / 1000u;
    return (ticks != 0) ? ticks : 1u;
}

static bool timer_soft_deadline_reached(uint64_t start_ticks,
                                        uint64_t wait_ticks)
{
    return wait_ticks != 0 && (g_ticks - start_ticks) >= wait_ticks;
}

static uint64_t timer_counter_ticks_since_base(void)
{
    if (g_tick_counter_source == ARM64_TIMER_COUNTER_NONE ||
        g_interval_count == 0) {
        return g_ticks;
    }

    uint64_t now = timer_read_counter_source(g_tick_counter_source);
    return g_counter_base_ticks +
           ((now - g_counter_base_count) / (uint64_t)g_interval_count);
}

static void timer_spin_fallback_ms(uint32_t ms)
{
    uint64_t spins = (uint64_t)ms * (uint64_t)TIMER_SPIN_FALLBACK_PER_MS;
    if (spins == 0) {
        spins = TIMER_SPIN_FALLBACK_PER_MS;
    }

    for (uint64_t i = 0; i < spins; ++i) {
        hal_cpu_pause();
    }
}

static void timer_counter_delay_ms(uint32_t ms)
{
    if (g_delay_counter_source == ARM64_TIMER_COUNTER_NONE) {
        timer_spin_fallback_ms(ms);
        return;
    }

    uint64_t wait_counts = timer_counts_for_ms(ms);
    uint64_t start_count = timer_read_counter_source(g_delay_counter_source);
    uint64_t last_count = start_count;
    uint64_t soft_start = g_ticks;
    uint64_t soft_wait = timer_soft_ticks_for_ms(ms);
    uint32_t stale_spins = 0;
    uint32_t total_spins = 0;

    while ((timer_read_counter_source(g_delay_counter_source) - start_count) < wait_counts) {
        if (timer_soft_deadline_reached(soft_start, soft_wait)) {
            break;
        }

        uint64_t now = timer_read_counter_source(g_delay_counter_source);
        if (now != last_count) {
            last_count = now;
            stale_spins = 0;
        } else {
            stale_spins++;
        }
        if (stale_spins >= TIMER_COUNTER_STALE_SPINS || ++total_spins >= (TIMER_COUNTER_STALE_SPINS * 10)) {
            timer_spin_fallback_ms(ms);
            break;
        }

        hal_cpu_pause();
    }
}

static void generic_timer_irq_handler(void)
{
    if (g_tick_bank == NULL) {
        return;
    }

    uint32_t ctl = g_tick_bank->read_ctl();
    if ((ctl & CNT_CTL_ISTATUS) == 0u) {
        return;
    }

    g_tick_bank->write_tval(g_interval_count);
    g_ticks++;

    if (g_timer_callback != NULL) {
        g_timer_callback();
    }
}

static void generic_timer_enable_irq_tick(void)
{
    if (g_tick_bank == NULL || g_irq_enabled != 0u) {
        return;
    }

    arm64_gic_mask_irq(g_tick_bank->irq);
    g_tick_bank->write_ctl(CNT_CTL_IMASK);
    g_tick_bank->write_tval(g_interval_count);
    g_ticks = timer_counter_ticks_since_base();
    g_irq_enabled = 1u;
    arm64_gic_route_irq(g_tick_bank->irq, 0);
    g_tick_bank->write_ctl(CNT_CTL_ENABLE);
}

static void generic_timer_init(uint32_t hz)
{
    g_hz = (hz != 0) ? hz : DEFAULT_TIMER_HZ;
    g_timer_freq = sanitize_timer_freq(read_cntfrq_el0());
    g_interval_count = (uint32_t)(g_timer_freq / g_hz);
    if (g_interval_count == 0) {
        g_interval_count = 1;
    }

    timer_bank_disable(&g_virtual_timer_bank);
    timer_bank_disable(&g_physical_timer_bank);
    select_timer_banks();

    register_interrupt_handler((int)g_tick_bank->irq, generic_timer_irq_handler);

    g_ticks = 0;
    g_irq_enabled = 0u;
    g_counter_base_count = timer_read_counter_source(g_tick_counter_source);
    g_counter_base_ticks = 0;
}

static uint64_t generic_timer_get_ticks(void)
{
    if (g_irq_enabled == 0u) {
        return timer_counter_ticks_since_base();
    }
    return g_ticks;
}

static void generic_timer_msleep(uint32_t ms)
{
    if (ms == 0) {
        return;
    }

    timer_counter_delay_ms(ms);
}

static void generic_timer_set_handler(void (*handler)(void))
{
    g_timer_callback = handler;
}

static void generic_timer_disable_irq(void)
{
    if (g_tick_bank != NULL) {
        arm64_gic_mask_irq(g_tick_bank->irq);
        g_tick_bank->write_ctl(CNT_CTL_IMASK);
        g_counter_base_ticks = g_ticks;
        g_irq_enabled = 0u;
        g_counter_base_count = timer_read_counter_source(g_tick_counter_source);
    }
}

static void generic_timer_switch_to_local(void)
{
    generic_timer_enable_irq_tick();
}

const timer_hal_t generic_timer_hal = {
    .init = generic_timer_init,
    .get_ticks = generic_timer_get_ticks,
    .msleep = generic_timer_msleep,
    .set_handler = generic_timer_set_handler,
    .disable_irq = generic_timer_disable_irq,
    .switch_to_local = generic_timer_switch_to_local
};
