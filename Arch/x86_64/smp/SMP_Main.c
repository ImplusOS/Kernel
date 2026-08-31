#include "SMP_Main.h"
#include "cpu/GDT_Main.h"
#include "cpu/IDT_Main.h"
#include "mmu/Paging_Main.h"
#include "Debug/serial/Serial.h"

#include "kernel/config.h"
#include "Platform/acpi/ACPI.h"
#include "Platform/interrupt/LAPIC.h"
#include "MemoryManagement/Memory_Main.h"
#include "Core/timer/Timer.h"
#include "Core/process/ProcessManager.h"
#include "cpu/IDT_Main.h"
#include "cpu/GDT_Main.h"
#include "Core/syscall/Syscall_Main.h"
#include "Core/process/ProcessScheduler.h"
#include "Core/timer/Timer.h"
#include <string.h>

#include <stdint.h>
#include <stdbool.h>

#define SMP_TRAMPOLINE_PHYS  0x8000ULL
#define SMP_SHARED_PHYS      0x9000ULL
#define AP_KERNEL_STACK_SIZE (256 * 1024)

#define AP_CR3_OFF   0
#define AP_ENTRY_OFF 8
#define AP_STACK_OFF 16
#define AP_GDTR_OFF  24
#define AP_IDTR_OFF  34

typedef struct __attribute__((packed)) {
    uint64_t cr3;
    uint64_t entry;
    uint64_t stack;
    uint16_t gdtr_limit;
    uint64_t gdtr_base;
    uint16_t idtr_limit;
    uint64_t idtr_base;
    /* Offset 44. The trampoline (SMP_Trampoline.asm) only touches bytes
     * 0..43; the AP raises this from ap_entry_c once it is running on its
     * own stack, i.e. once it has finished consuming everything above.
     * The BSP waits on it before reusing this page for the next AP. */
    volatile uint32_t ap_started;
} smp_shared_t;

typedef struct __attribute__((packed)) {
    uint16_t limit;
    uint64_t base;
} gdtr_t;

static uint32_t g_cpu_online = 1;
static uint32_t g_cpu_possible = 1;
static uint8_t  g_cpu_apic_ids[ACPI_MAX_CPUS];
static uint32_t g_cpu_apic_count = 0;
static int32_t  g_current_pid_per_cpu[OS_CONFIG_SMP_MAX_CPUS];

volatile struct {
    volatile uint64_t vaddr;
    volatile uint64_t pages;
    volatile uint32_t ack_count;
} g_tlb_req;

extern uint8_t smp_trampoline_start[];
extern uint8_t smp_trampoline_end[];

#include "interfaces/hal_cpu.h"

static inline uint32_t read_lapic_id(void)
{
    if (!lapic_is_present()) return 0;
    return lapic_get_id();
}

static inline void io_wait(void)
{
    hal_io_delay();
}

static void smp_delay_ms(uint32_t ms)
{
    uint64_t start = timer_monotonic_ns();
    if (start != 0u) {
        uint64_t duration = (uint64_t)ms * 1000000ULL;
        while (timer_monotonic_ns() - start < duration) {
            hal_cpu_pause();
        }
        return;
    }

    for (uint32_t i = 0; i < ms; ++i) {
        for (volatile uint32_t j = 0; j < 10000u; ++j) {
            hal_cpu_pause();
        }
    }
}

static uint32_t smp_detect_possible_cpus(void)
{
    uint32_t eax = 0, ebx = 0, ecx = 0, edx = 0;

    hal_cpu_get_id(0, 0, &eax, &ebx, &ecx, &edx);

    if (eax >= 0x0Bu) {
        hal_cpu_get_id(0x0Bu, 0, &eax, &ebx, &ecx, &edx);
        uint32_t count = ebx & 0xFFFFu;
        if (count > 0) return count;
    }

    hal_cpu_get_id(1, 0, &eax, &ebx, &ecx, &edx);
    uint32_t logical = (ebx >> 16) & 0xFFu;
    if (logical == 0) logical = 1;
    return logical;
}

void ap_entry_c(void)
{
    uint32_t lapic_id = read_lapic_id();

    uint32_t cpu_idx = 0;
    bool found = false;
    for (uint32_t i = 0; i < g_cpu_apic_count; i++) {
        if (g_cpu_apic_ids[i] == (uint8_t)lapic_id) {
            cpu_idx = i;
            found = true;
            break;
        }
    }

    if (!found) {
        while (1) { hal_cpu_halt(); }
    }

    /* We are past the real-mode trampoline and running on the kernel stack
     * it handed us, so SMP_SHARED_PHYS has been fully consumed and the BSP
     * may now overwrite it for the next AP. */
    {
        volatile smp_shared_t *sh =
            (volatile smp_shared_t *)(uintptr_t)SMP_SHARED_PHYS;
        __atomic_store_n(&sh->ap_started, 1u, __ATOMIC_SEQ_CST);
    }

    g_current_pid_per_cpu[cpu_idx] = -1;

    init_gdt();
    init_idt_per_cpu();
    syscall_init_per_cpu();

    /* Per-CPU MSR setup: EFER (NX), LAPIC */
    {
        uint32_t msr = 0xC0000080U;
        uint32_t lo, hi;
        __asm__ volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(msr));
        lo |= (1u << 11);
        __asm__ volatile("wrmsr" :: "a"(lo), "d"(hi), "c"(msr) : "memory");
    }

    __atomic_fetch_add(&g_cpu_online, 1u, __ATOMIC_SEQ_CST);

    lapic_ap_init();

    lapic_timer_ap_init();
    hal_cpu_enable_interrupts();
    uint64_t ap_iter = 0;
    while (1) {
        int found_proc = process_run_next_on_current_cpu();
        if (found_proc != 0) {
            continue;
        }
        uint64_t idle_start_ns = timer_monotonic_ns();
        hal_cpu_halt();
        uint64_t idle_ns = timer_monotonic_ns() - idle_start_ns;
        process_scheduler_add_idle_ns(idle_ns);
    }
}

static void smp_send_init_sipi_sipi(uint8_t apic_id, uint8_t trampoline_vector)
{
    lapic_send_ipi(apic_id, (1u << 15) | (1u << 14) | (5u << 8));
    smp_delay_ms(10);

    lapic_send_ipi(apic_id, (1u << 14) | (6u << 8) | trampoline_vector);
    smp_delay_ms(1);

    lapic_send_ipi(apic_id, (1u << 14) | (6u << 8) | trampoline_vector);
    smp_delay_ms(2);
}

static void smp_fill_shared(uint64_t ap_cr3, void *ap_entry, uint64_t ap_stack)
{
    smp_shared_t *sh = (smp_shared_t *)(uintptr_t)SMP_SHARED_PHYS;
    sh->cr3   = ap_cr3;
    sh->entry = (uint64_t)(uintptr_t)ap_entry;
    sh->stack = ap_stack;

    IDT_Ptr *idt_ptr = idt_get_ptr();
    sh->idtr_limit = idt_ptr->limit;
    sh->idtr_base  = idt_ptr->base;

    gdtr_t gdtr;
    hal_cpu_get_gdt_ptr(&gdtr);
    sh->gdtr_limit = gdtr.limit;
    sh->gdtr_base  = gdtr.base;
}

void smp_init(void)
{
    for (uint32_t i = 0; i < OS_CONFIG_SMP_MAX_CPUS; i++) {
        g_current_pid_per_cpu[i] = -1;
    }
    g_current_pid_per_cpu[0] = -1;

    const acpi_info_t *info = acpi_get_info();
    if (info && info->cpu_count > 0) {
        g_cpu_possible = info->cpu_count;
        g_cpu_apic_count = info->cpu_count;
        for (uint32_t i = 0; i < info->cpu_count && i < ACPI_MAX_CPUS; ++i) {
            g_cpu_apic_ids[i] = info->cpu_apic_ids[i];
        }
    } else {
        g_cpu_possible = smp_detect_possible_cpus();
        g_cpu_apic_count = 0;
    }

    g_cpu_online = 1;

    if (!OS_CONFIG_SMP_ENABLED || g_cpu_possible <= 1 || !lapic_is_present()) {
        return;
    }

    uint32_t trampoline_size = (uint32_t)(smp_trampoline_end - smp_trampoline_start);
    memcpy((void *)(uintptr_t)SMP_TRAMPOLINE_PHYS, smp_trampoline_start, trampoline_size);
    hal_cpu_memory_barrier();
    memset((void *)(uintptr_t)SMP_SHARED_PHYS, 0, 4096);

    uint32_t bsp_lapic_id = lapic_get_id();
    uint8_t  trampoline_vector = (uint8_t)(SMP_TRAMPOLINE_PHYS >> 12);

    uint64_t bsp_cr3 = hal_cpu_read_cr(3);
    hal_cpu_invalidate_caches();

    uint32_t aps_started = 0;

    for (uint32_t i = 0; i < g_cpu_apic_count && i < (uint32_t)OS_CONFIG_SMP_MAX_CPUS; i++) {
        uint8_t aid = g_cpu_apic_ids[i];

        if (aid == (uint8_t)bsp_lapic_id) continue;

        uint8_t *ap_stack_base = (uint8_t *)malloc(AP_KERNEL_STACK_SIZE);
        if (!ap_stack_base) {
            continue;
        }

        uint64_t ap_stack_top = ((uint64_t)(uintptr_t)(ap_stack_base + AP_KERNEL_STACK_SIZE)) & ~0xFULL;

        volatile smp_shared_t *sh =
            (volatile smp_shared_t *)(uintptr_t)SMP_SHARED_PHYS;

        smp_fill_shared(bsp_cr3, ap_entry_c, ap_stack_top);
        __atomic_store_n(&sh->ap_started, 0u, __ATOMIC_SEQ_CST);

        __atomic_thread_fence(__ATOMIC_SEQ_CST);

        uint32_t expected =
            __atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE) + 1u;
        smp_send_init_sipi_sipi(aid, trampoline_vector);

        /* Do not touch the shared hand-off page again until this AP has
         * left the trampoline and copied it out (it sets ap_started from
         * ap_entry_c). Otherwise a slow AP would pick up the *next* AP's
         * stack pointer here and two CPUs would run on one stack. The
         * cap (matching the online-wait below) only trips for a CPU that
         * never starts at all -- in which case there is no straggler left
         * to corrupt anyway. ap_started is raised before the AP bumps
         * g_cpu_online, so this never waits longer than the online wait. */
        uint32_t consume_timeout = 200;
        while (consume_timeout-- > 0) {
            if (__atomic_load_n(&sh->ap_started, __ATOMIC_ACQUIRE) != 0u) break;
            smp_delay_ms(1);
        }

        uint32_t timeout = 200;
        while (timeout-- > 0) {
            if (__atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE) >= expected) break;
            smp_delay_ms(1);
        }

        if (__atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE) >= expected) {
            aps_started++;
        }
    }
}

uint32_t smp_get_cpu_count(void)
{
    return __atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE);
}

uint32_t smp_get_possible_cpu_count(void)
{
    return g_cpu_possible;
}

uint32_t smp_get_current_cpu_id(void)
{
    if (!lapic_is_present()) return 0;
    uint32_t lid = lapic_get_id();
    for (uint32_t i = 0; i < g_cpu_apic_count; i++) {
        if (g_cpu_apic_ids[i] == (uint8_t)lid) return i;
    }
    return 0;
}

int32_t smp_get_current_pid(void)
{
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) cpu = 0;
    return g_current_pid_per_cpu[cpu];
}

void smp_set_current_pid(int32_t pid)
{
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) cpu = 0;
    g_current_pid_per_cpu[cpu] = pid;
}

void smp_tlb_shootdown(uint64_t vaddr, uint64_t pages)
{
    if (__atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE) <= 1) {
        for (uint64_t i = 0; i < pages; i++) {
            hal_mmu_invalidate_tlb(vaddr + i * 4096ULL);
        }
        return;
    }

    __atomic_store_n(&g_tlb_req.vaddr,     vaddr, __ATOMIC_RELAXED);
    __atomic_store_n(&g_tlb_req.pages,     pages, __ATOMIC_RELAXED);
    __atomic_store_n(&g_tlb_req.ack_count, 0u,    __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);

    lapic_send_ipi(0, (3u << 18) | (uint32_t)VECTOR_TLB_SHOOTDOWN);

    for (uint64_t i = 0; i < pages; i++) {
        hal_mmu_invalidate_tlb(vaddr + i * 4096ULL);
    }
}

void smp_tlb_shootdown_all(void)
{
    if (__atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE) <= 1) {
        paging_switch_cr3(paging_get_active_cr3());
        return;
    }

    __atomic_store_n(&g_tlb_req.vaddr,     0u, __ATOMIC_RELAXED);
    __atomic_store_n(&g_tlb_req.pages,     0u, __ATOMIC_RELAXED);
    __atomic_store_n(&g_tlb_req.ack_count, 0u, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);

    lapic_send_ipi(0, (3u << 18) | (uint32_t)VECTOR_TLB_SHOOTDOWN);

    paging_switch_cr3(paging_get_active_cr3());
}

void smp_tlb_shootdown_handler(void)
{
    uint64_t addr  = __atomic_load_n(&g_tlb_req.vaddr, __ATOMIC_ACQUIRE);
    uint64_t pages = __atomic_load_n(&g_tlb_req.pages, __ATOMIC_ACQUIRE);
    if (pages == 0u) {
        paging_switch_cr3(paging_get_active_cr3());
    } else {
        for (uint64_t i = 0; i < pages; i++) {
            hal_mmu_invalidate_tlb(addr + i * 4096ULL);
        }
    }
    __atomic_fetch_add(&g_tlb_req.ack_count, 1u, __ATOMIC_RELEASE);
    lapic_eoi();
}
