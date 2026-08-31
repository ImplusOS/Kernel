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

static void lapic_timer_handler(void) {
    if (smp_get_current_cpu_id() != 0u) {
        return;
    }
    __atomic_fetch_add(&g_ticks, 1u, __ATOMIC_RELAXED);
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

static void lapic_switch_to_local(void) {
    if (!platform_interrupts_using_lapic() || !lapic_is_present()) {
        return;
    }
    uint64_t start_tick = g_ticks;
    if (lapic_timer_start(VECTOR_TIMER, 0xFFFFFFFFu, 0, 16u) != 0) {
        return;
    }
    uint64_t target = start_tick + 10u;
    while (g_ticks < target) {
        __asm__ volatile("pause" ::: "memory");
    }
    uint32_t current = lapic_timer_current();
    uint32_t elapsed = 0xFFFFFFFFu - current;
    lapic_timer_stop();
    if (elapsed < 1000u) {
        return;
    }
    uint32_t initial = elapsed / 10u;
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
