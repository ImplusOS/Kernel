#include "interfaces/arch_ops.h"

#include "cpu/CPU.h"
#include "cpu/Exception.h"
#include "mmu/Paging_Main.h"

static void arm64_init_cpu_tables(void)
{
    arm64_exception_init();
}

static void arm64_enter_user_mode(uint64_t saved_rsp, uint64_t user_rsp, uint64_t address_space)
{
    (void)user_rsp;
    paging_switch_cr3(address_space);
    arm64_enter_user_from_frame(saved_rsp);
    __builtin_unreachable();
}

static int arm64_virtualization_init(void)
{
    return -1;
}

extern const timer_hal_t generic_timer_hal;

static const timer_hal_t *arm64_get_timer_hal(void)
{
    return &generic_timer_hal;
}

static const arch_ops_t g_arm64_arch_ops = {
    .early_init = arm64_cpu_early_init,
    .init_cpu_tables = arm64_init_cpu_tables,
    .enable_interrupts = arm64_enable_interrupts,
    .disable_interrupts = arm64_disable_interrupts,
    .irq_save_disable = arm64_irq_save_disable,
    .irq_restore = arm64_irq_restore,
    .enter_user_mode = arm64_enter_user_mode,
    .virtualization_init = arm64_virtualization_init,
    .get_timer_hal = arm64_get_timer_hal,
};

const arch_ops_t *arch_ops_get(void)
{
    return &g_arm64_arch_ops;
}
