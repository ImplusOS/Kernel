#include "SMP_Main.h"
#include "PSCI.h"

#include "kernel/config.h"
#include "cpu/CPU.h"
#include "cpu/Exception.h"
#include "cpu/IDT_Main.h"
#include "interrupt/GIC.h"
#include "mmu/Paging_Main.h"
#include "Platform/acpi/ACPI.h"
#include "Core/process/ProcessManager.h"
#include "Core/process/ProcessScheduler.h"
#include "Core/syscall/Syscall_Main.h"
#include "Core/timer/Timer.h"
#include "Debug/serial/Serial.h"
#include "interfaces/hal_cpu.h"

#include <stdbool.h>
#include <stdint.h>

/* SGI used to make another CPU look for work (process_scheduler_kick_*). */
#define ARM64_SGI_RESCHED 1u

#define AP_KERNEL_STACK_SIZE (64u * 1024u)
#define SMP_MAX_TRACKED      64u

/* Read by SecondaryEntry.S with the MMU off: keep the layout in step. */
typedef struct {
    uint64_t stack_top;   /*  0 */
    uint64_t ttbr;        /*  8 */
    uint64_t tcr;         /* 16 */
    uint64_t mair;        /* 24 */
    uint64_t sctlr;       /* 32 */
    uint64_t entry;       /* 40 */
    uint64_t cpacr;       /* 48 */
    uint64_t cpu_index;   /* 56 */
} __attribute__((aligned(64))) arm64_ap_boot_t;

extern void arm64_secondary_entry(void);
extern void generic_timer_ap_init(void);
extern char _kernel_start[];
extern char _kernel_end[];

static volatile uint32_t g_cpu_count = 1;     /* CPUs online */
static volatile uint32_t g_cpu_possible = 1;  /* CPUs the firmware lists */
static volatile int32_t g_current_pid[SMP_MAX_TRACKED];
static volatile uint64_t g_cpu_mpidr[SMP_MAX_TRACKED];
static uint32_t g_cpu_iface[SMP_MAX_TRACKED];
static arm64_ap_boot_t g_ap_boot[OS_CONFIG_SMP_MAX_CPUS];
static uint8_t g_ap_stacks[OS_CONFIG_SMP_MAX_CPUS][AP_KERNEL_STACK_SIZE]
    __attribute__((aligned(16)));

static inline uint64_t read_mpidr(void)
{
    uint64_t mpidr;
    __asm__ volatile("mrs %0, MPIDR_EL1" : "=r"(mpidr));
    return mpidr;
}

static inline uint64_t mpidr_affinity(uint64_t mpidr)
{
    return mpidr & 0xFF00FFFFFFULL;
}

/* Clean [start, end) to the point of coherency, so a CPU running with its
 * MMU (and therefore its caches) off reads what this one wrote. */
static void dcache_clean_poc(const void *start, const void *end)
{
    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    uintptr_t line = (uintptr_t)4u << ((ctr >> 16) & 0xFu);
    for (uintptr_t p = (uintptr_t)start & ~(line - 1u); p < (uintptr_t)end; p += line) {
        __asm__ volatile("dc cvac, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

static void smp_delay_spin(uint32_t iterations)
{
    for (volatile uint32_t i = 0; i < iterations; ++i) {
        hal_cpu_pause();
    }
}

static void arm64_resched_sgi_handler(void)
{
    /* Whatever this CPU was doing, have it look for other work. The IRQ
     * exit path (arm64_exception_dispatch) preempts EL0 when it sees it. */
    process_scheduler_request_reschedule();
}

/* C half of a secondary CPU's start-up; SecondaryEntry.S has enabled the
 * MMU and moved onto this CPU's stack. Mirrors x86's ap_entry_c(). */
__attribute__((noreturn)) void arm64_ap_entry_c(arm64_ap_boot_t *boot);
__attribute__((noreturn)) void arm64_ap_entry_c(arm64_ap_boot_t *boot)
{
    uint32_t cpu_idx = (uint32_t)boot->cpu_index;
    (void)cpu_idx;

    arm64_cpu_early_init();          /* CPACR, DAIF masked, VBAR */
    syscall_init_per_cpu();          /* TPIDR_EL1 -> this CPU's state */
    (void)arm64_gic_init_secondary();

    __atomic_fetch_add(&g_cpu_count, 1u, __ATOMIC_SEQ_CST);

    generic_timer_ap_init();
    hal_cpu_enable_interrupts();

    for (;;) {
        if (process_run_next_on_current_cpu() != 0) {
            continue;
        }
        uint64_t idle_start_ns = timer_monotonic_ns();
        process_scheduler_idle_wait();
        process_scheduler_add_idle_ns(timer_monotonic_ns() - idle_start_ns);
    }
}

void smp_init(void)
{
    for (uint32_t i = 0; i < SMP_MAX_TRACKED; ++i) {
        g_current_pid[i] = -1;
        g_cpu_mpidr[i] = 0;
        g_cpu_iface[i] = 0;
    }

    uint64_t self = mpidr_affinity(read_mpidr());
    g_cpu_count = 1;
    g_cpu_possible = 1;
    g_cpu_mpidr[0] = self;

    const acpi_info_t *info = acpi_get_info();
    if (info != NULL) {
        arm64_psci_set_conduit_hvc(info->psci_use_hvc != 0u ||
                                   info->psci_compliant == 0u);
    }

    /* CPU 0 is the boot CPU wherever the firmware lists it; the others keep
     * MADT order. The table is filled before any secondary starts, so
     * smp_get_current_cpu_id() resolves on every CPU from its first
     * instruction of C. */
    uint32_t possible = 1;
    if (info != NULL) {
        for (uint32_t i = 0; i < info->cpu_count && i < ACPI_MAX_CPUS; ++i) {
            uint64_t aff = mpidr_affinity(info->cpu_mpidr[i]);
            if (aff == self) {
                g_cpu_iface[0] = info->cpu_apic_ids[i];
                continue;
            }
            if (possible >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
                break;
            }
            g_cpu_mpidr[possible] = aff;
            g_cpu_iface[possible] = info->cpu_apic_ids[i];
            ++possible;
        }
    }
    if (!OS_CONFIG_SMP_ENABLED) {
        possible = 1;
    }
    __atomic_store_n(&g_cpu_possible, possible, __ATOMIC_SEQ_CST);

    register_interrupt_handler((int)ARM64_SGI_RESCHED, arm64_resched_sgi_handler);

    if (possible <= 1u) {
        return;
    }

    uint64_t ttbr, tcr, mair, sctlr;
    paging_get_secondary_config(&ttbr, &tcr, &mair, &sctlr);
    uint64_t cpacr;
    __asm__ volatile("mrs %0, cpacr_el1" : "=r"(cpacr));

    /* The secondaries fetch their first instructions with the MMU off. */
    dcache_clean_poc(_kernel_start, _kernel_end);

    uint32_t started = 0;
    for (uint32_t cpu = 1; cpu < possible; ++cpu) {
        arm64_ap_boot_t *boot = &g_ap_boot[cpu];
        boot->stack_top = ((uint64_t)(uintptr_t)&g_ap_stacks[cpu][AP_KERNEL_STACK_SIZE]) &
                          ~0xFULL;
        boot->ttbr = ttbr;
        boot->tcr = tcr;
        boot->mair = mair;
        boot->sctlr = sctlr;
        boot->entry = (uint64_t)(uintptr_t)arm64_ap_entry_c;
        boot->cpacr = cpacr;
        boot->cpu_index = cpu;
        dcache_clean_poc(boot, boot + 1);

        uint32_t expected = __atomic_load_n(&g_cpu_count, __ATOMIC_ACQUIRE) + 1u;
        int64_t rc = arm64_psci_cpu_on(g_cpu_mpidr[cpu],
                                       (uint64_t)(uintptr_t)arm64_secondary_entry,
                                       (uint64_t)(uintptr_t)boot);
        if (rc != PSCI_SUCCESS) {
            serial_write_string("[SMP] PSCI CPU_ON failed for cpu ");
            serial_write_uint32(cpu);
            serial_write_string("\n");
            continue;
        }
        for (uint32_t spin = 0; spin < 2000u; ++spin) {
            if (__atomic_load_n(&g_cpu_count, __ATOMIC_ACQUIRE) >= expected) {
                break;
            }
            smp_delay_spin(10000u);
        }
        if (__atomic_load_n(&g_cpu_count, __ATOMIC_ACQUIRE) >= expected) {
            ++started;
        }
    }

    serial_write_string("[SMP] arm64: ");
    serial_write_uint32(__atomic_load_n(&g_cpu_count, __ATOMIC_ACQUIRE));
    serial_write_string(" CPU(s) online of ");
    serial_write_uint32(possible);
    serial_write_string("\n");
    (void)started;
}

uint32_t smp_get_cpu_count(void)
{
    return g_cpu_count;
}

uint32_t smp_get_possible_cpu_count(void)
{
    return g_cpu_possible;
}

uint32_t smp_get_current_cpu_id(void)
{
    uint64_t current = mpidr_affinity(read_mpidr());
    uint32_t limit = g_cpu_possible;
    if (limit > SMP_MAX_TRACKED) {
        limit = SMP_MAX_TRACKED;
    }

    for (uint32_t i = 0; i < limit; ++i) {
        if (g_cpu_mpidr[i] == current) {
            return i;
        }
    }

    return 0;
}

void smp_send_resched_ipi(uint32_t cpu)
{
    if (cpu >= g_cpu_possible || cpu >= SMP_MAX_TRACKED) {
        return;
    }
    arm64_gic_send_sgi(g_cpu_mpidr[cpu], g_cpu_iface[cpu], ARM64_SGI_RESCHED);
}

void smp_tlb_shootdown(uint64_t vaddr, uint64_t pages)
{
    if (pages == 0u) {
        __asm__ volatile("dsb ishst; tlbi vmalle1is; dsb ish; isb" ::: "memory");
        return;
    }

    uint64_t page = vaddr >> 12;
    __asm__ volatile("dsb ishst" ::: "memory");
    for (uint64_t i = 0; i < pages; ++i) {
        uint64_t operand = page + i;
        __asm__ volatile("tlbi vaae1is, %0" :: "r"(operand) : "memory");
    }
    __asm__ volatile("dsb ish; isb" ::: "memory");
}

void smp_tlb_shootdown_handler(void)
{
    __asm__ volatile("dsb ishst; tlbi vmalle1is; dsb ish; isb" ::: "memory");
}

int32_t smp_get_current_pid(void)
{
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu >= SMP_MAX_TRACKED) return -1;
    return g_current_pid[cpu];
}

void smp_set_current_pid(int32_t pid)
{
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu < SMP_MAX_TRACKED) g_current_pid[cpu] = pid;
}

/* pages == 0 means "the whole TLB", as on x86 (SMP_Main.h there). TLBI ...IS
 * is broadcast to every CPU in the inner-shareable domain by hardware, so
 * there is no one to wait for and the address space does not narrow it. */
void smp_tlb_shootdown_cr3(uint64_t cr3, uint64_t vaddr, uint64_t pages)
{
    (void)cr3;
    smp_tlb_shootdown(vaddr, pages);
}

void smp_note_cr3(uint64_t cr3)
{
    (void)cr3;
}

void smp_tlb_poll(void)
{
}
