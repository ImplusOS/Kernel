#include "CPU.h"
#include "Exception.h"
#include "interfaces/hal_cpu.h"

uint64_t arm64_current_el(void)
{
    return hal_cpu_get_current_el();
}

void arm64_set_exception_vector(void *vector_base)
{
    hal_cpu_set_vbar(vector_base);
}

#include "interfaces/hal_cpu.h"

void arm64_disable_interrupts(void)
{
    hal_cpu_disable_interrupts();
}

void arm64_enable_interrupts(void)
{
    hal_cpu_enable_interrupts();
}

uint64_t arm64_irq_save_disable(void)
{
    return hal_cpu_save_interrupts();
}

void arm64_irq_restore(uint64_t flags)
{
    hal_cpu_restore_interrupts(flags);
}

void arm64_cpu_early_init(void)
{
    uint64_t el = arm64_current_el();
    if (el == 1) {
        uint64_t cpacr = (3ULL << 20);
        __asm__ volatile("msr CPACR_EL1, %0\n isb\n" :: "r"(cpacr) : "memory");
    } else if (el == 2) {
        uint64_t cptr = 0;
        __asm__ volatile("msr CPTR_EL2, %0\n isb\n" :: "r"(cptr) : "memory");
    }
    __asm__ volatile("msr daifset, #0xf\n");
    arm64_exception_init();
}

void arm64_wait_forever(void)
{
    for (;;) {
        __asm__ volatile("wfi");
    }
}

