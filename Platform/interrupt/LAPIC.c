#include "LAPIC.h"
#include <stddef.h>

#include <stddef.h>
#include "mmu/Paging_Main.h"

#define IA32_APIC_BASE_MSR    0x1B
#define IA32_APIC_BASE_ENABLE (1ULL << 11)

#define LAPIC_REG_ID        0x020
#define LAPIC_REG_EOI       0x0B0
#define LAPIC_REG_SVR       0x0F0
#define LAPIC_REG_ICR_LOW   0x300
#define LAPIC_REG_ICR_HIGH  0x310
#define LAPIC_ICR_DELIVERY_STATUS (1u << 12)
#define LAPIC_REG_LVT_TIMER 0x320
#define LAPIC_REG_TIMER_INIT 0x380
#define LAPIC_REG_TIMER_CURR 0x390
#define LAPIC_REG_TIMER_DIV  0x3E0

#define LAPIC_LVT_MASKED   (1u << 16)
#define LAPIC_LVT_PERIODIC (1u << 17)

static volatile uint32_t *g_lapic = NULL;
static uint64_t g_lapic_phys = 0;
static int g_lapic_present = 0;

static inline uint64_t rdmsr(uint32_t msr)
{
    uint32_t lo, hi;
    __asm__ volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(msr));
    return ((uint64_t)hi << 32) | (uint64_t)lo;
}

static inline void wrmsr(uint32_t msr, uint64_t val)
{
    uint32_t lo = (uint32_t)(val & 0xFFFFFFFFu);
    uint32_t hi = (uint32_t)(val >> 32);
    __asm__ volatile("wrmsr" :: "c"(msr), "a"(lo), "d"(hi));
}

static inline void lapic_write(uint32_t reg, uint32_t val)
{
    g_lapic[reg / 4] = val;
    (void)g_lapic[LAPIC_REG_ID / 4];
}

static inline uint32_t lapic_read(uint32_t reg)
{
    return g_lapic[reg / 4];
}

int lapic_init(uint64_t phys_base)
{
    if (phys_base == 0) {
        return -1;
    }

    uint64_t msr = rdmsr(IA32_APIC_BASE_MSR);
    msr &= ~0xFFFFF000ULL;
    msr |= (phys_base & 0xFFFFF000ULL);
    msr |= IA32_APIC_BASE_ENABLE;
    wrmsr(IA32_APIC_BASE_MSR, msr);

    g_lapic_phys = phys_base;
    g_lapic = (volatile uint32_t *)map_mmio_virt(phys_base);
    g_lapic_present = 1;

    lapic_write(LAPIC_REG_SVR, 0xFFu | (1u << 8));
    lapic_write(LAPIC_REG_LVT_TIMER, LAPIC_LVT_MASKED);
    return 0;
}

int lapic_is_present(void)
{
    return g_lapic_present;
}

void lapic_ap_init(void)
{
    if (!g_lapic_present) return;
    uint64_t msr;
    __asm__ volatile("rdmsr" : "=a"(((uint32_t *)&msr)[0]),
                                "=d"(((uint32_t *)&msr)[1])
                              : "c"((uint32_t)0x1BU));
    msr |= (1ULL << 11);
    __asm__ volatile("wrmsr" : : "a"(((uint32_t *)&msr)[0]),
                                 "d"(((uint32_t *)&msr)[1]),
                                 "c"((uint32_t)0x1BU)
                               : "memory");
    lapic_write(LAPIC_REG_SVR, 0xFFu | (1u << 8));
}

void lapic_eoi(void)
{
    if (!g_lapic_present) return;
    lapic_write(LAPIC_REG_EOI, 0);
}

uint32_t lapic_get_id(void)
{
    if (!g_lapic_present) return 0;
    return (lapic_read(LAPIC_REG_ID) >> 24) & 0xFFu;
}

int lapic_timer_start(uint8_t vector, uint32_t initial_count, int periodic, uint32_t divide)
{
    if (!g_lapic_present || g_lapic == NULL || initial_count == 0) {
        return -1;
    }

    if (divide == 0) divide = 1;

    uint32_t div_encoded = 0;
    switch (divide) {
        case 1:   div_encoded = 0b1011; break;
        case 2:   div_encoded = 0b0000; break;
        case 4:   div_encoded = 0b0001; break;
        case 8:   div_encoded = 0b0010; break;
        case 16:  div_encoded = 0b0011; break;
        case 32:  div_encoded = 0b1000; break;
        case 64:  div_encoded = 0b1001; break;
        case 128: div_encoded = 0b1010; break;
        default:  div_encoded = 0b0011; break;
    }

    lapic_write(LAPIC_REG_TIMER_DIV, div_encoded);
    lapic_write(LAPIC_REG_LVT_TIMER,
                (uint32_t)vector | (periodic ? LAPIC_LVT_PERIODIC : 0));
    lapic_write(LAPIC_REG_TIMER_INIT, initial_count);
    return 0;
}

void lapic_timer_stop(void)
{
    if (!g_lapic_present) return;
    lapic_write(LAPIC_REG_LVT_TIMER, LAPIC_LVT_MASKED);
    lapic_write(LAPIC_REG_TIMER_INIT, 0);
}

uint32_t lapic_timer_current(void)
{
    if (!g_lapic_present) return 0;
    return lapic_read(LAPIC_REG_TIMER_CURR);
}

static void lapic_wait_icr_idle(void)
{
    uint32_t spins = 0x100000u;
    while (spins--) {
        if ((lapic_read(LAPIC_REG_ICR_LOW) & LAPIC_ICR_DELIVERY_STATUS) == 0) {
            return;
        }
        __asm__ volatile("pause");
    }
}

void lapic_send_ipi(uint8_t apic_id, uint32_t icr_low)
{
    if (!g_lapic_present) return;
    lapic_wait_icr_idle();
    lapic_write(LAPIC_REG_ICR_HIGH, ((uint32_t)apic_id) << 24);
    lapic_write(LAPIC_REG_ICR_LOW, icr_low);
}
