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

/*
 * TLB shootdown: one request slot per CPU, plus an acknowledgement matrix.
 *
 * The previous version had a single shared slot and never waited for anyone to
 * act on it. Both halves of that were wrong. Not waiting means the initiator
 * carries on -- freeing the page, narrowing its permissions, handing the
 * address range back to the allocator -- while other CPUs still hold the old
 * translation and keep writing through it. Sharing one unlocked slot means two
 * CPUs shooting down at the same time overwrite each other's request, so one
 * of the two flushes never happens at all. With a few dozen Chromium threads
 * calling mmap/mprotect/munmap across four CPUs, both happen constantly, and
 * the result is memory that changes under the process: the browser died within
 * a minute under -smp >1 and ran indefinitely under -smp 1.
 *
 * Each CPU now owns g_tlb_slot[cpu] and bumps its `seq` to publish a request.
 * A responder services every slot it has not caught up with and records the
 * sequence it serviced in g_tlb_seen[responder][requester]. The initiator waits
 * until every online CPU has recorded its sequence -- and services other CPUs'
 * slots while it waits, so two CPUs shooting down simultaneously unblock each
 * other instead of deadlocking. A slot stays stable while its request is
 * outstanding because its owner is blocked in that wait.
 */
typedef struct {
    volatile uint64_t cr3;     /* 0 = concerns every address space */
    volatile uint64_t vaddr;
    volatile uint64_t pages;   /* 0 = flush everything */
    volatile uint32_t seq;
} tlb_request_t;

/* The address space each CPU currently has loaded, published by
 * smp_note_cr3(). Without PCID a CR3 load flushes the whole TLB, so a CPU that
 * is not running an address space cannot hold a stale translation for it --
 * which is what lets a shootdown skip it instead of waiting. */
static volatile uint64_t g_cpu_cr3[OS_CONFIG_SMP_MAX_CPUS];

static tlb_request_t g_tlb_slot[OS_CONFIG_SMP_MAX_CPUS];
static volatile uint32_t
    g_tlb_seen[OS_CONFIG_SMP_MAX_CPUS][OS_CONFIG_SMP_MAX_CPUS];

/* Bounded so a CPU that is wedged (or was never really brought up) degrades to
 * the old best-effort behaviour instead of hanging the machine. Sized to be
 * far longer than any legitimate IPI turnaround. */
#define TLB_ACK_SPIN_LIMIT 20000000u

/* Non-zero while at least one shootdown is waiting for acknowledgements. Lets
 * smp_tlb_poll() -- called from every contended spinlock -- cost a single
 * relaxed load in the common case instead of an APIC read. */
static volatile uint32_t g_tlb_inflight;

/* Bring-up instrumentation for the shootdown cost. Enabled with
 * -DTLB_STATS=1; the counters are relaxed atomics and the dump is rate
 * limited, so it costs nothing measurable. */
#ifndef TLB_STATS
#define TLB_STATS 0
#endif
#if TLB_STATS
static volatile uint64_t g_tlb_calls;   /* requests made */
static volatile uint64_t g_tlb_ipis;    /* of those, ones that needed an IPI */
static volatile uint64_t g_tlb_spins;   /* total spin iterations waiting */
static volatile uint64_t g_tlb_maxspin; /* worst single wait */
#endif

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

/* The current CPU's index without a VM exit.
 *
 * smp_get_current_cpu_id() runs several times per syscall -- the scheduler,
 * syscall_set_user_rsp(), the TSS rsp0 update, process_get_current_pid(),
 * and every spinlock that polls for TLB shootdowns -- and reading the local
 * APIC ID is an MMIO access, which is a VM exit under KVM. Profiling Chromium
 * starting up found most busy vCPU samples either inside lapic_get_id() or
 * spinning behind a lock whose holder was.
 *
 * RDTSCP hands back MSR_TSC_AUX in ECX and runs natively under KVM, so each
 * CPU stores (index + 1) there as it comes up. Zero, the reset value, means
 * "not stored yet" and the lookup falls back to the APIC ID. */
#define MSR_TSC_AUX 0xC0000103U
static bool g_cpu_id_rdtscp;

static bool cpu_has_rdtscp(void)
{
    uint32_t eax = 0, ebx = 0, ecx = 0, edx = 0;
    hal_cpu_get_id(0x80000000u, 0, &eax, &ebx, &ecx, &edx);
    if (eax < 0x80000001u) return false;
    hal_cpu_get_id(0x80000001u, 0, &eax, &ebx, &ecx, &edx);
    return (edx & (1u << 27)) != 0u;
}

static void smp_publish_cpu_index(uint32_t cpu_idx)
{
    if (!cpu_has_rdtscp()) return;
    hal_cpu_write_msr(MSR_TSC_AUX, (uint64_t)cpu_idx + 1u);
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

    smp_publish_cpu_index(cpu_idx);

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
    for (uint32_t i = 0; i < g_cpu_apic_count; i++) {
        if (g_cpu_apic_ids[i] == (uint8_t)bsp_lapic_id) {
            if (cpu_has_rdtscp()) {
                smp_publish_cpu_index(i);
                g_cpu_id_rdtscp = true;
            }
            break;
        }
    }
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
    if (g_cpu_id_rdtscp) {
        uint32_t lo, hi, aux;
        __asm__ volatile("rdtscp" : "=a"(lo), "=d"(hi), "=c"(aux));
        (void)lo;
        (void)hi;
        if (aux != 0u) return aux - 1u;
    }
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

/* Act on every other CPU's outstanding request that this CPU has not yet
 * caught up with. Safe to call from the shootdown IPI handler and from inside
 * the initiator's own wait loop.
 *
 * The acknowledgement is unconditional: a CPU that is not running the address
 * space the request names has nothing to invalidate (its last CR3 load already
 * flushed everything), but it still has to say so, or the sender waits for it.
 */
static void tlb_service_peers(uint32_t me)
{
    uint32_t count = __atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE);
    if (count > (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        count = (uint32_t)OS_CONFIG_SMP_MAX_CPUS;
    }
    uint64_t mine = paging_get_active_cr3();
    for (uint32_t r = 0; r < count; ++r) {
        if (r == me) {
            continue;
        }
        uint32_t seq = __atomic_load_n(&g_tlb_slot[r].seq, __ATOMIC_ACQUIRE);
        if (seq == __atomic_load_n(&g_tlb_seen[me][r], __ATOMIC_RELAXED)) {
            continue;
        }
        uint64_t want  = g_tlb_slot[r].cr3;
        uint64_t addr  = g_tlb_slot[r].vaddr;
        uint64_t pages = g_tlb_slot[r].pages;
        if (want == 0u || want == mine) {
            if (pages == 0u) {
                paging_switch_cr3(mine);
            } else {
                for (uint64_t i = 0; i < pages; ++i) {
                    hal_mmu_invalidate_tlb(addr + i * 4096ULL);
                }
            }
        }
        __atomic_store_n(&g_tlb_seen[me][r], seq, __ATOMIC_RELEASE);
    }
}

/* Does CPU `c` still have to answer request `seq` from CPU `me`? */
static int tlb_ack_outstanding(uint32_t c, uint32_t me, uint32_t seq,
                               uint64_t want)
{
    if (__atomic_load_n(&g_tlb_seen[c][me], __ATOMIC_ACQUIRE) == seq) {
        return 0;   /* answered */
    }
    if (want != 0u &&
        __atomic_load_n(&g_cpu_cr3[c], __ATOMIC_ACQUIRE) != want) {
        return 0;   /* not running this address space: nothing stale to hold */
    }
    return 1;
}

void smp_note_cr3(uint64_t cr3)
{
    uint32_t me = smp_get_current_cpu_id();
    if (me >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        me = 0;
    }
    __atomic_store_n(&g_cpu_cr3[me], cr3, __ATOMIC_RELEASE);
}

void smp_tlb_shootdown_cr3(uint64_t cr3, uint64_t vaddr, uint64_t pages)
{
    uint32_t online = __atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE);
    uint32_t me = smp_get_current_cpu_id();
    if (me >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        me = 0;
    }
#if TLB_STATS
    __atomic_fetch_add(&g_tlb_calls, 1u, __ATOMIC_RELAXED);
#endif

    /* This CPU's own flush happens either way. */
    if (pages == 0u) {
        paging_switch_cr3(paging_get_active_cr3());
    } else {
        for (uint64_t i = 0; i < pages; ++i) {
            hal_mmu_invalidate_tlb(vaddr + i * 4096ULL);
        }
    }

    if (online <= 1u) {
        return;
    }
    if (online > (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        online = (uint32_t)OS_CONFIG_SMP_MAX_CPUS;
    }

    /* Nobody else is running this address space: nobody else can hold a
     * translation for it. This is the common case and costs no IPI at all. */
    if (cr3 != 0u) {
        int peers = 0;
        for (uint32_t c = 0; c < online; ++c) {
            if (c != me &&
                __atomic_load_n(&g_cpu_cr3[c], __ATOMIC_ACQUIRE) == cr3) {
                peers = 1;
                break;
            }
        }
        if (!peers) {
            return;
        }
    }

    g_tlb_slot[me].cr3 = cr3;
    g_tlb_slot[me].vaddr = vaddr;
    g_tlb_slot[me].pages = pages;
    uint32_t seq = g_tlb_slot[me].seq + 1u;
    __atomic_fetch_add(&g_tlb_inflight, 1u, __ATOMIC_RELAXED);
    __atomic_store_n(&g_tlb_slot[me].seq, seq, __ATOMIC_RELEASE);

#if TLB_STATS
    __atomic_fetch_add(&g_tlb_ipis, 1u, __ATOMIC_RELAXED);
#endif
    lapic_send_ipi(0, (3u << 18) | (uint32_t)VECTOR_TLB_SHOOTDOWN);

    for (uint32_t spins = 0; spins < TLB_ACK_SPIN_LIMIT; ++spins) {
        int pending = 0;
        for (uint32_t c = 0; c < online; ++c) {
            if (c == me) continue;
            if (tlb_ack_outstanding(c, me, seq, cr3)) {
                pending = 1;
                break;
            }
        }
        if (!pending) {
#if TLB_STATS
            __atomic_fetch_add(&g_tlb_spins, (uint64_t)spins, __ATOMIC_RELAXED);
            if ((uint64_t)spins > __atomic_load_n(&g_tlb_maxspin, __ATOMIC_RELAXED)) {
                __atomic_store_n(&g_tlb_maxspin, (uint64_t)spins, __ATOMIC_RELAXED);
            }
            uint64_t n = __atomic_load_n(&g_tlb_ipis, __ATOMIC_RELAXED);
            if ((n & 0xFFu) == 0u) {
                serial_write_string("[tlbstat] calls=");
                serial_write_uint64(__atomic_load_n(&g_tlb_calls, __ATOMIC_RELAXED));
                serial_write_string(" ipis=");
                serial_write_uint64(n);
                serial_write_string(" spins=");
                serial_write_uint64(__atomic_load_n(&g_tlb_spins, __ATOMIC_RELAXED));
                serial_write_string(" max=");
                serial_write_uint64(__atomic_load_n(&g_tlb_maxspin, __ATOMIC_RELAXED));
                serial_write_string("\n");
            }
#endif
            __atomic_fetch_sub(&g_tlb_inflight, 1u, __ATOMIC_RELAXED);
            return;
        }
        /* Another CPU may be waiting on us at the same time. */
        tlb_service_peers(me);
        __asm__ volatile("pause" ::: "memory");
    }

    /* Timed out. Report it once in a while: reaching here means a CPU stopped
     * answering, and the memory this shootdown was protecting is no longer
     * protected. */
    {
        static volatile uint32_t reported;
        if (__atomic_fetch_add(&reported, 1u, __ATOMIC_RELAXED) < 8u) {
            serial_write_string("[tlb] ack timeout cpu=");
            serial_write_uint32(me);
            serial_write_string(" cr3=");
            serial_write_uint64(cr3);
            serial_write_string("\n");
        }
    }
    __atomic_fetch_sub(&g_tlb_inflight, 1u, __ATOMIC_RELAXED);
}

void smp_tlb_shootdown(uint64_t vaddr, uint64_t pages)
{
    if (pages == 0u) {
        return;
    }
    smp_tlb_shootdown_cr3(0u, vaddr, pages);
}

void smp_tlb_shootdown_all(void)
{
    smp_tlb_shootdown_cr3(0u, 0u, 0u);
}

void smp_tlb_poll(void)
{
    if (__atomic_load_n(&g_tlb_inflight, __ATOMIC_RELAXED) == 0u) {
        return;
    }
    if (__atomic_load_n(&g_cpu_online, __ATOMIC_ACQUIRE) <= 1u) {
        return;
    }
    uint32_t me = smp_get_current_cpu_id();
    if (me >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        me = 0;
    }
    tlb_service_peers(me);
}

void smp_tlb_shootdown_handler(void)
{
    uint32_t me = smp_get_current_cpu_id();
    if (me >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        me = 0;
    }
    tlb_service_peers(me);
    lapic_eoi();
}
