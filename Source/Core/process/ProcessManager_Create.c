#include "ProcessManager.h"
#include "ProcessScheduler.h"
#include "Core/debug/FlightRec.h"

#include "IPC/IPC_Main.h"
#include "IPC/PnP_Notifications.h"
#include "IPC/UnixSocket.h"
#include "Core/vfs/ProcFS.h"
#include "Core/elf/ELF_Loader.h"
#include "Core/memory/SharedMemory.h"
#include "Compat/Linux/SysvShm.h"
#include "Core/memory/FileMap.h"
#include "Core/vfs/TmpFS.h"
#include "Compat/Linux/Syscall_LinuxCompat.h"
#include <stdio.h>
#include <stdarg.h>
#include "Core/vfs/VFS.h"
#include "cpu/GDT_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "Core/sync/Spinlock.h"
#include "Core/syscall/Syscall_File.h"
#include "Core/syscall/Poll_Wait.h"
#include "Core/syscall/Syscall_Socket.h"
#include "Core/syscall/Syscall_Main.h"
#include "Core/syscall/Syscall_Futex.h"
#include "Core/syscall/Syscall_InputOwner.h"
#include "Core/usercopy/Usercopy.h"
#include "Core/timer/Timer.h"
#include "interfaces/hal_cpu.h"
#include "interfaces/arch_ops.h"
#include "smp/SMP_Main.h"
#include <string.h>
#include "Debug/serial/Serial.h"
#if defined(__aarch64__)
#include "cpu/Exception.h"
#endif

#include <stddef.h>
#include <stdint.h>
#include "Core/drm/DRM_Kms.h"
#include "Core/tty/Pty.h"

#define PROCESS_RFLAGS_DEFAULT 0x202ULL
#define PROCESS_GUARD_PAGE_SIZE PAGE_SIZE
#define PROCESS_INITIAL_USER_STACK_SIZE (16ULL * PAGE_SIZE)
#define PROCESS_THREAD_STACK_SIZE (8ULL * 1024ULL * 1024ULL)
#define PROCESS_THREAD_STACK_REGION_SIZE \
    (PROCESS_THREAD_STACK_SIZE + PROCESS_GUARD_PAGE_SIZE)

#if defined(__aarch64__)
#define PROCESS_CONTEXT_QWORDS ((uint32_t)(sizeof(arm64_exception_frame_t) / sizeof(uint64_t)))
#else
#define PROCESS_CONTEXT_QWORDS SYSCALL_FRAME_QWORDS
#endif
/* Big enough for a dynamically-linked prebuilt Chromium (~465 MB, unstripped)
 * plus headroom. The ELF loader streams segment data through a small staging
 * buffer (see ELF_Loader.c), so this bound is about the file, not kernel heap.
 * vfs_file_t.size is uint32_t, so the hard ceiling is 4 GiB regardless. */
#define PROCESS_ELF_MAX_SIZE (768ULL * 1024ULL * 1024ULL)

#define IA32_FS_BASE      0xC0000100U
#define IA32_KERNEL_GS_BASE 0xC0000102U

static inline uint64_t rdmsr_fs_base(void)
{
#if defined(__aarch64__)
    uint64_t val;
    __asm__ volatile("mrs %0, TPIDR_EL0" : "=r"(val));
    return val;
#else
    uint32_t lo, hi;
    __asm__ volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(IA32_FS_BASE));
    return ((uint64_t)hi << 32) | lo;
#endif
}

static inline void wrmsr_fs_base(uint64_t val)
{
#if defined(__aarch64__)
    __asm__ volatile("msr TPIDR_EL0, %0" :: "r"(val) : "memory");
#else
    uint32_t lo = (uint32_t)(val & 0xFFFFFFFFU);
    uint32_t hi = (uint32_t)(val >> 32);
    __asm__ volatile("wrmsr" :: "c"(IA32_FS_BASE), "a"(lo), "d"(hi) : "memory");
#endif
}

static inline void wrmsr_gs_base(uint64_t val)
{
    hal_cpu_write_gs_base(val);
}

static inline uint64_t rdmsr_kernel_gs_base(void)
{
#if defined(__aarch64__)
    return 0;
#else
    uint32_t lo, hi;
    __asm__ volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(IA32_KERNEL_GS_BASE));
    return ((uint64_t)hi << 32) | lo;
#endif
}

static inline void wrmsr_kernel_gs_base(uint64_t val)
{
#if defined(__aarch64__)
    (void)val;
#else
    uint32_t lo = (uint32_t)(val & 0xFFFFFFFFU);
    uint32_t hi = (uint32_t)(val >> 32);
    __asm__ volatile("wrmsr" :: "c"(IA32_KERNEL_GS_BASE), "a"(lo), "d"(hi) : "memory");
#endif
}

static inline void process_cpu_halt(void)
{
#if defined(__aarch64__)
    __asm__ volatile("wfi");
#else
    __asm__ volatile("hlt");
#endif
}

static inline void process_fpu_save(uint8_t *state)
{
    hal_cpu_save_fpu(state);
}

static inline void process_fpu_restore(uint8_t *state)
{
    hal_cpu_restore_fpu(state);
}

typedef struct {
    uint64_t a_type;
    uint64_t a_val;
} process_auxv_t;

enum {
    PROCESS_AT_NULL   = 0,
    PROCESS_AT_PHDR   = 3,
    PROCESS_AT_PHENT  = 4,
    PROCESS_AT_PHNUM  = 5,
    PROCESS_AT_PAGESZ = 6,
    PROCESS_AT_BASE   = 7,
    PROCESS_AT_ENTRY  = 9,
    PROCESS_AT_HWCAP  = 16,
    PROCESS_AT_CLKTCK = 17,
    PROCESS_AT_UID    = 11,
    PROCESS_AT_EUID   = 12,
    PROCESS_AT_GID    = 13,
    PROCESS_AT_EGID   = 14,
    PROCESS_AT_SECURE = 23,
    PROCESS_AT_RANDOM = 25,
    PROCESS_AT_EXECFN = 31,
    PROCESS_AT_PLATFORM = 15,
    PROCESS_AT_HWCAP2 = 26,
};

/* AArch64 AT_HWCAP bits (arch/arm64/include/uapi/asm/hwcap.h). Only what
 * every ARMv8-A core has: glibc picks its string/memory routines from these,
 * and claiming an extension the CPU lacks would make it SIGILL. */
#define PROCESS_ARM64_HWCAP_FP    (1ULL << 0)
#define PROCESS_ARM64_HWCAP_ASIMD (1ULL << 1)

static process_t *g_processes = NULL;
static int32_t g_process_capacity = 0;
static uint64_t *g_sleep_deadline_ns = NULL;
static spinlock_t g_process_table_lock;
static uint32_t g_timeslice_ticks = 6;

static void initialize_fpu_state(uint8_t *fpu_state)
{
    if (fpu_state == NULL) {
        return;
    }

    memset(fpu_state, 0, PROCESS_FPU_STATE_SIZE);
    fpu_state[0] = 0x7F;
    fpu_state[1] = 0x03;
    fpu_state[24] = 0x80;
    fpu_state[25] = 0x1F;
}

int32_t current_pid_get(void)
{
    return process_scheduler_current_pid();
}

static inline void current_pid_set(int32_t pid)
{
    process_scheduler_set_current_pid(pid);
}


/* ---- Parked kernel stacks ------------------------------------------------
 *
 * A CPU that has just run a process off the end of exit_group() is still
 * standing on that process's kernel stack, and it keeps naming the process as
 * its current pid until it finds something else to run. process_waitpid()
 * refuses to reap a zombie any CPU still names, so if nothing else is
 * runnable on this CPU -- the common case with 16 CPUs and three processes --
 * the parent's wait4() waits forever on a child that has already exited. That
 * is what stalled Xorg after every xkbcomp run.
 *
 * Dropping the reference alone is not enough: the reap then frees the very
 * stack this CPU is executing on. So take the stack away from the dying
 * process first and park it against this CPU. The reaper frees everything
 * else; the stack itself is released the next time this CPU is demonstrably
 * running on some other stack. */
typedef struct {
    void    *base;
    uint64_t top;
} parked_stack_t;

static parked_stack_t g_parked_stack[OS_CONFIG_SMP_MAX_CPUS];

static void process_release_parked_stack(void)
{
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        return;
    }
    void *base = g_parked_stack[cpu].base;
    if (base == NULL) {
        return;
    }
    /* An interrupt taken while this CPU idles on the parked stack re-enters
     * the scheduler on that same stack, so prove we have left it before
     * handing it back to the heap. */
    uint64_t here = (uint64_t)(uintptr_t)&base;
    if (here >= (uint64_t)(uintptr_t)base && here < g_parked_stack[cpu].top) {
        return;
    }
    g_parked_stack[cpu].base = NULL;
    g_parked_stack[cpu].top = 0;
    free(base);
}

static int is_valid_pid(int32_t pid);

static void deferred_space_drain(void);

/* Caller holds g_process_table_lock. If this CPU's current process is already
 * dead, park its kernel stack here and stop naming it, so waitpid() can reap.
 * No-op for a live process. */
static void process_detach_dead_current_locked(void)
{
    int32_t pid = current_pid_get();
    if (!is_valid_pid(pid)) {
        return;
    }
    process_t *proc = &g_processes[pid];
    if (proc->state != PROCESS_STATE_ZOMBIE &&
        proc->state != PROCESS_STATE_DEAD &&
        proc->state != PROCESS_STATE_UNUSED) {
        return;
    }

    /* Park the dying process's kernel stack against this CPU and take it away
     * from the slot. This CPU is still standing on that stack: the rest of
     * this scheduling call runs on it, and when nothing is runnable it idles
     * here with interrupts enabled, taking every timer tick and IPI onto it.
     * Meanwhile the parent's wait4() -- woken by poll_wait_notify() on exit --
     * can reap the zombie on another CPU, and a slot keeps its kernel stack for
     * reuse, so the next fork into the slot starts running on the very stack
     * this CPU is still using. Xorg forks and reaps xkbcomp this way twice
     * whenever an X client connects, and the shared stack showed up as a
     * double fault at RIP 0x800C, a triple fault with RSP 0, or a whole-guest
     * hang with a frozen clock, always within a second of Chromium starting.
     *
     * With the stack detached, a reused slot allocates a fresh one, and
     * process_release_parked_stack() frees this one at the next scheduling
     * entry on this CPU -- only once it can see it is running elsewhere. That
     * release always runs before the next detach on the same CPU, so the
     * parking spot is normally empty here; if it is not, leak the older stack
     * rather than free one we cannot prove unused. */
    {
        uint32_t cpu = smp_get_current_cpu_id();
        if (cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS &&
            proc->kernel_stack_base != NULL) {
            g_parked_stack[cpu].base = proc->kernel_stack_base;
            g_parked_stack[cpu].top = proc->kernel_stack_top;
            proc->kernel_stack_base = NULL;
            proc->kernel_stack_top = 0u;
        }
    }
    /* This CPU is also still running on the dying process's page tables, and
     * the reap tears those down (paging_destroy_process_space). Get onto the
     * kernel's own CR3 first, or the next instruction faults with no page
     * tables to fault into -- an instant triple fault and reboot. */
    if (paging_get_active_cr3() != paging_get_kernel_cr3()) {
        /* Also when the slot no longer names its tables: an exited process
         * hands them to the deferred teardown (process_teardown), which
         * frees them once no CPU is on them -- this one included. */
        paging_switch_cr3(paging_get_kernel_cr3());
    }
    current_pid_set(-1);
    /* This CPU has just left a dead task's tables: maybe the last one on
     * them. */
    deferred_space_drain();
}

static void halt_forever(void)
{
    hal_cpu_disable_interrupts();
    while (1) {
        process_cpu_halt();
    }
}

void process_broadcast_shutdown(void)
{
    uint32_t signal = IPC_SIGNAL_SHUTDOWN;
    for (int32_t i = 0; i < g_process_capacity; ++i) {
        if (g_processes[i].state == PROCESS_STATE_RUNNING ||
            g_processes[i].state == PROCESS_STATE_READY ||
            g_processes[i].state == PROCESS_STATE_BLOCKED) {
            if (i != current_pid_get()) {
                ipc_send_message(i, &signal, sizeof(uint32_t));
            }
        }
    }

    
    for (int retry = 0; retry < 100; retry++) {
        int all_dead = 1;
        for (int32_t i = 0; i < g_process_capacity; ++i) {
            if (i == current_pid_get()) continue;
            if (g_processes[i].state != PROCESS_STATE_UNUSED &&
                g_processes[i].state != PROCESS_STATE_DEAD) {
                all_dead = 0;
                break;
            }
        }
        if (all_dead) break;
        
        
        process_cpu_halt();
    }
}

static int align_up_u64_checked(uint64_t value, uint64_t align, uint64_t *result_out)
{
    if (result_out == NULL || align == 0 || (align & (align - 1ULL)) != 0) {
        return -1;
    }

    uint64_t addend = align - 1ULL;
    if (value > (UINT64_MAX - addend)) {
        return -1;
    }

    *result_out = (value + addend) & ~addend;
    return 0;
}

static int is_valid_user_entry(uint64_t entry)
{
    /* A thread/clone entry point is legitimate anywhere executable code can be
     * mapped in the user half:
     *   - the code+heap span [0x1000, USER_STACK_BASE): glibc's pthread_create
     *     passes start_thread() from libc.so.6, and the Linux-ABI loader
     *     file-maps the whole .so closure into the heap window (~0x41_0000_0000,
     *     observed with gtk3-demo), well above USER_CODE_LIMIT;
     *   - the lazy mmap arena [USER_MMAP_BASE, USER_MMAP_LIMIT): anonymous
     *     JIT / trampoline pages, and any .so ld.so places there.
     * The stack region and kernel space stay excluded. Restricting this to
     * USER_CODE_LIMIT (or, before that, to the mmap arena) made every glibc
     * pthread_create() fail with EAGAIN, which GLib's mandatory pool-spawner
     * thread turns into a fatal g_error() ("Resource temporarily unavailable").
     * See Docs/Others/TODO_GTK3_Wayland_LinuxABI.md G6. */
    return (entry >= 0x1000) &&
           ((entry < USER_STACK_BASE) ||
            (entry >= USER_MMAP_BASE && entry < USER_MMAP_LIMIT));
}

static int process_table_ready(void)
{
    return g_processes != NULL && g_process_capacity > 0;
}

static void process_clear_sleep_deadline_for_proc_locked(const process_t *proc)
{
    if (g_sleep_deadline_ns == NULL || g_processes == NULL ||
        proc == NULL || g_process_capacity <= 0) {
        return;
    }

    uintptr_t base = (uintptr_t)g_processes;
    uintptr_t end = base + (uintptr_t)g_process_capacity * sizeof(process_t);
    uintptr_t value = (uintptr_t)proc;
    if (value < base || value >= end) {
        return;
    }

    uintptr_t offset = value - base;
    if ((offset % sizeof(process_t)) != 0u) {
        return;
    }

    size_t index = (size_t)(offset / sizeof(process_t));
    g_sleep_deadline_ns[index] = 0u;
}

static uint64_t process_deadline_after_ns(uint64_t now_ns, uint64_t delay_ns)
{
    if (UINT64_MAX - now_ns < delay_ns) {
        return UINT64_MAX;
    }
    return now_ns + delay_ns;
}

static uint64_t process_perf_now_ns(void)
{
    return timer_monotonic_ns();
}

/* ---- Foreign (Linux-ABI) program launch log -------------------------------
 *
 * "Which Linux program started, when, and how it ended" is worth having in
 * every boot: it is a handful of lines per program rather than
 * thousands, and it is the first thing anyone asks when an app seems slow to
 * come up.
 *
 * Each line is assembled in full and written once. Piecewise
 * serial_write_string() calls from several CPUs interleave character by
 * character -- an Xorg boot log is full of lines spliced into each other --
 * and a half-line is worse than no line. One write also lets the UART FIFO
 * batching in uart_hal.c do its job.
 *
 * Lines go to COM1 and to the small ring behind /dev/applog, which is what a
 * terminal on the desktop reads. Deliberately *not* the whole kernel log:
 * a window streaming that feeds back on itself, because drawing a line is X
 * traffic and X traffic is logged.
 */
static uint64_t g_launch_start_ns[PROCESS_MAX_COUNT_CONFIG];

#define FOREIGN_LOG_LINE_MAX 256u

typedef struct {
    char     buf[FOREIGN_LOG_LINE_MAX];
    uint32_t len;
} foreign_log_line_t;

static void foreign_log_str(foreign_log_line_t *line, const char *text)
{
    if (text == NULL) {
        text = "(null)";
    }
    while (*text != '\0' && line->len + 1u < FOREIGN_LOG_LINE_MAX) {
        line->buf[line->len++] = *text++;
    }
    line->buf[line->len] = '\0';
}

static void foreign_log_dec(foreign_log_line_t *line, uint64_t value)
{
    char digits[24];
    uint32_t n = 0;
    do {
        digits[n++] = (char)('0' + (value % 10u));
        value /= 10u;
    } while (value != 0u && n < sizeof(digits));
    while (n != 0u && line->len + 1u < FOREIGN_LOG_LINE_MAX) {
        line->buf[line->len++] = digits[--n];
    }
    line->buf[line->len] = '\0';
}

static void foreign_log_emit(foreign_log_line_t *line)
{
    foreign_log_str(line, "\n");
    serial_write_string(line->buf);
    serial_applog_write(line->buf);
}

static void foreign_launch_log_start(int32_t pid, const char *what,
                                     const char *path, const char *arg,
                                     uint8_t linux_abi)
{
    uint64_t now_ns = process_perf_now_ns();
    if (pid >= 0 && pid < PROCESS_MAX_COUNT_CONFIG) {
        g_launch_start_ns[pid] = now_ns;
    }
    if (!OS_CONFIG_FOREIGN_LAUNCH_LOG || !linux_abi) {
        return;
    }

    foreign_log_line_t line = { {0}, 0u };
    foreign_log_str(&line, "[app] ");
    foreign_log_str(&line, what);
    foreign_log_str(&line, " pid=");
    foreign_log_dec(&line, (uint64_t)(uint32_t)pid);
    foreign_log_str(&line, " path=");
    foreign_log_str(&line, path);
    if (arg != NULL && arg[0] != '\0') {
        foreign_log_str(&line, " args='");
        foreign_log_str(&line, arg);
        foreign_log_str(&line, "'");
    }
    foreign_log_str(&line, " at=");
    foreign_log_dec(&line, now_ns / 1000000ULL);
    foreign_log_str(&line, "ms");
    foreign_log_emit(&line);
}

static void foreign_launch_log_exit(int32_t pid, int32_t status,
                                    uint8_t linux_abi)
{
    /* Cleared for every process, not just the logged ones: pid slots are
     * reused, and a stale start time left behind by a native process would
     * make the next Linux program to land on that slot report a lifetime
     * measured from someone else's launch. */
    uint64_t started_ns = 0u;
    if (pid >= 0 && pid < PROCESS_MAX_COUNT_CONFIG) {
        started_ns = g_launch_start_ns[pid];
        g_launch_start_ns[pid] = 0u;
    }

    if (!OS_CONFIG_FOREIGN_LAUNCH_LOG || !linux_abi) {
        return;
    }

    foreign_log_line_t line = { {0}, 0u };
    foreign_log_str(&line, "[app] exit pid=");
    foreign_log_dec(&line, (uint64_t)(uint32_t)pid);
    foreign_log_str(&line, " status=");
    foreign_log_dec(&line, (uint64_t)(uint32_t)status);
    if (started_ns != 0u) {
        foreign_log_str(&line, " lived=");
        foreign_log_dec(&line, (process_perf_now_ns() - started_ns) / 1000000ULL);
        foreign_log_str(&line, "ms");
    }
    foreign_log_emit(&line);
}


static void process_perf_reset(process_t *proc)
{
    if (proc == NULL) {
        return;
    }
    proc->runtime_ns = 0;
    proc->ready_wait_ns = 0;
    proc->max_ready_wait_ns = 0;
    proc->context_switches = 0;
    proc->voluntary_switches = 0;
    proc->involuntary_switches = 0;
    proc->syscalls = 0;
    proc->ipc_send = 0;
    proc->ipc_recv = 0;
    proc->block_count = 0;
    proc->wake_count = 0;
    proc->blocked_ns = 0;
    proc->display_present_calls = 0;
    proc->display_rects = 0;
    proc->display_bytes = 0;
    proc->last_scheduled_ns = 0;
    proc->ready_since_ns = 0;
    proc->blocked_since_ns = 0;
    proc->page_fault_count = 0;
    proc->last_pf_addr = 0;
    proc->last_pf_rip = 0;
    proc->last_pf_error = 0;
    proc->has_crashed = 0;
    proc->crash_reason[0] = '\0';
}

static void process_perf_account_runtime_locked(process_t *proc, uint64_t now_ns)
{
    if (proc == NULL || proc->last_scheduled_ns == 0u) {
        return;
    }
    if (now_ns >= proc->last_scheduled_ns) {
        proc->runtime_ns += now_ns - proc->last_scheduled_ns;
    }
    proc->last_scheduled_ns = 0u;
}

static void process_perf_mark_ready_locked(process_t *proc, uint64_t now_ns)
{
    if (proc == NULL) {
        return;
    }
    if (proc->ready_since_ns == 0u) {
        proc->ready_since_ns = now_ns;
    }
}

static void process_perf_prepare_run_locked(process_t *proc,
                                            uint64_t now_ns,
                                            int involuntary)
{
    if (proc == NULL) {
        return;
    }
    if (proc->ready_since_ns != 0u && now_ns >= proc->ready_since_ns) {
        uint64_t wait_ns = now_ns - proc->ready_since_ns;
        proc->ready_wait_ns += wait_ns;
        if (wait_ns > proc->max_ready_wait_ns) {
            proc->max_ready_wait_ns = wait_ns;
        }
    }
    proc->ready_since_ns = 0u;
    proc->last_scheduled_ns = now_ns;
    proc->context_switches++;
    if (involuntary) {
        proc->involuntary_switches++;
    } else {
        proc->voluntary_switches++;
    }
}

static void process_wake_sleepers_locked(uint64_t now_ns)
{
    if (g_sleep_deadline_ns == NULL || !process_table_ready()) {
        return;
    }

    for (int32_t i = 0; i < g_process_capacity; ++i) {
        uint64_t deadline_ns = g_sleep_deadline_ns[i];
        if (deadline_ns == 0u) {
            continue;
        }

        process_t *proc = &g_processes[i];
        if (proc->state != PROCESS_STATE_BLOCKED) {
            g_sleep_deadline_ns[i] = 0u;
            continue;
        }
        if (now_ns < deadline_ns) {
            continue;
        }

        g_sleep_deadline_ns[i] = 0u;
        if (proc->blocked_since_ns != 0u &&
            now_ns >= proc->blocked_since_ns) {
            proc->blocked_ns += now_ns - proc->blocked_since_ns;
        }
        proc->blocked_since_ns = 0u;
        proc->wake_count++;
        proc->state = PROCESS_STATE_READY;
        process_perf_mark_ready_locked(proc, now_ns);
        process_scheduler_request_reschedule();
    }
}

static int process_group_on_cpu_locked(int32_t pid);

static int is_valid_pid(int32_t pid)
{
    return process_table_ready() && pid >= 0 && pid < g_process_capacity;
}

static void save_syscall_frame_to_process(process_t *proc, uint64_t current_saved_rsp)
{
    if (proc == NULL || current_saved_rsp == 0) return;
    proc->saved_rsp = current_saved_rsp;
}

static void reset_process_slot(process_t *proc)
{
    if (proc == NULL) {
        return;
    }

    process_clear_sleep_deadline_for_proc_locked(proc);

    proc->state = PROCESS_STATE_UNUSED;
    proc->full_restore = 0;
    proc->rt_policy = 0;
    proc->rt_priority = 0;
    proc->wake_boost = 0;
    proc->is_thread = 0;
    proc->thread_detached = 0;
    proc->user_stack_exchanged = 0;
    proc->capability_mask = 0;
    proc->entry = 0;
    proc->saved_rsp = 0;
    proc->saved_user_rsp = 0;

    initialize_fpu_state(proc->fpu_state);

    proc->fs_base = 0;
    proc->gs_base = 0;
    proc->full_rcx = 0;
    proc->full_r11 = 0;
    memset(proc->compat_tls, 0, sizeof(proc->compat_tls));
    proc->compat_fs_sel = 0;
    proc->compat_gs_sel = 0;
    if (proc->compat_vm_map != NULL) {
        free(proc->compat_vm_map);
        proc->compat_vm_map = NULL;
    }

    proc->cr3 = 0;
    /* kernel_stack_base / kernel_stack_top are deliberately NOT cleared: the
     * stack belongs to the slot, not to the process that last used it. See
     * the allocation site. */
    proc->user_code_base = 0;
    proc->user_code_limit = 0;
    proc->user_heap_base = 0;
    proc->user_heap_cursor = 0;
    proc->user_heap_limit = 0;
    proc->user_heap_alloc_limit = 0;
    proc->user_brk_base = 0;
    proc->user_brk_cursor = 0;
    proc->user_brk_limit = 0;
    proc->user_heap_guard_page = 0;
    proc->user_stack_base = 0;
    proc->user_stack_top = 0;
    proc->user_mmap_cursor = 0;
    proc->user_stack_guard_page = 0;
    proc->timeslice = 0;
    proc->total_ticks = 0;
    process_perf_reset(proc);
    memset(proc->name, 0, sizeof(proc->name));
    memset(proc->cwd, 0, sizeof(proc->cwd));
    proc->cwd[0] = '/';
    memset(proc->root_path, 0, sizeof(proc->root_path)); /* "" == the real root */
    proc->fs_share_pid = -1;
    proc->pidns_init = -1;
    proc->fork_hold_tid = -1;
    proc->uid = 0u;
    proc->gid = 0u;
    memset(proc->exe_path, 0, sizeof(proc->exe_path));
    memset(proc->launch_argument, 0, sizeof(proc->launch_argument));
    proc->parent_pid = -1;
    proc->pgid = -1;
    proc->sid = -1;
    proc->teardown_pending = 0u;
    proc->memory_owner_pid = -1;
    proc->exit_status = 0;
    proc->exit_by_signal = 0;
    proc->exit_term_signal = 0;
    proc->pdeath_signal = 0;
    proc->thread_stack_region_base = 0;
    proc->thread_stack_region_size = 0;
    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        proc->user_allocs[i].used = 0;
        proc->user_allocs[i].addr = 0;
        proc->user_allocs[i].size = 0;
    }
    for (uint32_t i = 0; i < PROCESS_SIGNAL_MAX; ++i) {
        proc->signal_handlers[i] = 0;
        proc->signal_flags[i] = 0;
        proc->signal_sa_mask[i] = 0;
        proc->signal_restorer[i] = 0;
    }
    proc->altstack_sp = 0;
    proc->altstack_size = 0;
    proc->altstack_flags = 2u; /* SS_DISABLE */
    proc->signal_mask = 0;
    proc->pending_signals = 0;
    proc->clear_child_tid = 0;
    proc->robust_list_head = 0;
    proc->robust_list_length = 0;
    proc->rseq_area = 0;
    proc->rseq_sig = 0;
    proc->wake_pending = 0;
}

/*
 * Address spaces whose owner was reaped while another CPU was still running
 * one of its threads (see release_process_resources). Each is destroyed once
 * no CPU is on its tables any more and no task slot refers to it; every reap
 * retries. Callers hold g_process_table_lock.
 */
#define DEFERRED_SPACE_MAX 256
static uint64_t g_deferred_space[DEFERRED_SPACE_MAX];

static void deferred_space_add(uint64_t cr3)
{
    for (uint32_t i = 0; i < DEFERRED_SPACE_MAX; ++i) {
        if (g_deferred_space[i] == 0u) {
            g_deferred_space[i] = cr3;
            return;
        }
    }
    /* Full: the space is left behind, as every such reap used to do. */
    serial_write_string("[OS] [PROC] deferred address-space queue full: leaking one\n");
}

static void deferred_space_drain(void)
{
    for (uint32_t i = 0; i < DEFERRED_SPACE_MAX; ++i) {
        const uint64_t cr3 = g_deferred_space[i];
        if (cr3 == 0u || paging_space_active_on_any_cpu(cr3)) {
            continue;
        }
        /* A task that can still be scheduled onto it keeps it. Dead and
         * zombie tasks never run again (the scheduler only picks READY). */
        int referenced = 0;
        for (int32_t k = 0; k < g_process_capacity && !referenced; ++k) {
            const process_t *p = &g_processes[k];
            referenced = p->cr3 == cr3 && p->state != PROCESS_STATE_UNUSED &&
                         p->state != PROCESS_STATE_DEAD &&
                         p->state != PROCESS_STATE_ZOMBIE;
        }
        if (referenced) {
            continue;
        }
        /* Nothing may name it once it is gone: the frame can come back as
         * another process's tables, and a slot released later would unmap
         * its thread stack from those. */
        for (int32_t k = 0; k < g_process_capacity; ++k) {
            if (g_processes[k].cr3 == cr3) {
                g_processes[k].cr3 = 0u;
            }
        }
        g_deferred_space[i] = 0u;
        flight_rec(FR_TAG_DESTROY, cr3, 0xFFFFFFFFu);
        paging_destroy_process_space(cr3);
    }
}

static void release_thread_resources(process_t *proc, int unmap_user_stack)
{
    if (proc == NULL || !proc->is_thread) {
        return;
    }

    if (unmap_user_stack && proc->cr3 != 0 &&
        proc->thread_stack_region_base != 0 &&
        proc->thread_stack_region_size != 0) {
        (void)paging_unmap_range(proc->cr3,
                                 proc->thread_stack_region_base,
                                 proc->thread_stack_region_size);
    }
    /* The kernel stack stays with the slot. Freeing it here raced with the
     * CPU that was still running on it: a thread is torn down from whichever
     * CPU reaps it, while the CPU that was executing the thread may not have
     * left its stack yet, and the heap would hand those pages straight to
     * somebody else. That is a reboot with nothing on the serial line, and
     * Chromium -- which creates and destroys threads continuously -- hit it
     * within a minute under -smp 4. Keeping it costs one 128 KiB stack per
     * occupied process slot and removes the window entirely. */
    proc->cr3 = 0;
    proc->thread_stack_region_base = 0;
    proc->thread_stack_region_size = 0;
    deferred_space_drain();
}

static void release_process_resources(process_t *proc)
{
    if (proc == NULL) {
        return;
    }

    if (proc->is_thread) {
        release_thread_resources(proc, 1);
        return;
    }

    int32_t owner_pid = -1;
    if (g_processes != NULL && proc >= g_processes &&
        proc < (g_processes + g_process_capacity)) {
        owner_pid = (int32_t)(proc - g_processes);
    }

    int threads_still_running = 0;
    if (owner_pid >= 0) {
        for (int32_t i = 1; i < g_process_capacity; ++i) {
            process_t *thread = &g_processes[i];
            if (thread != proc && thread->is_thread &&
                thread->memory_owner_pid == owner_pid &&
                thread->state != PROCESS_STATE_UNUSED) {
                if (process_scheduler_pid_in_use_on_any_cpu(i)) {
                    threads_still_running = 1;
                    continue;
                }
                release_thread_resources(thread, 0);
                reset_process_slot(thread);
            }
        }
    }

    /*
     * Only tear the address space down once nothing is executing in it.
     *
     * The loop above deliberately skips threads another CPU is still running,
     * and destroying the page tables anyway pulls them out from under that
     * CPU -- which does not fault, it triple-faults, and the machine resets
     * with nothing on the serial line. Chromium reaches this every run: it
     * exits with dozens of threads live across four CPUs.
     *
     * When that happens the space is queued instead, and destroyed by a
     * later reap once no CPU is on it and no thread refers to it. (Leaving
     * it behind for good, as this used to, leaked the whole address space:
     * every page of it. Chromium lost gigabytes per run that way and the
     * next start ran out of memory.)
     */
    if (proc->cr3 != 0) {
        if (!threads_still_running) {
            flight_rec(FR_TAG_DESTROY, proc->cr3, (uint64_t)(uint32_t)owner_pid);
            paging_destroy_process_space(proc->cr3);
        } else {
            deferred_space_add(proc->cr3);
        }
        proc->cr3 = 0;
    }
    deferred_space_drain();
    /* Kept, not freed -- see the note in the thread teardown above. */
    proc->capability_mask = 0;
    proc->user_code_base = 0;
    proc->user_code_limit = 0;
    proc->user_heap_base = 0;
    proc->user_heap_cursor = 0;
    proc->user_heap_limit = 0;
    proc->user_heap_alloc_limit = 0;
    proc->user_brk_base = 0;
    proc->user_brk_cursor = 0;
    proc->user_brk_limit = 0;
    proc->user_heap_guard_page = 0;
    proc->user_stack_base = 0;
    proc->user_stack_top = 0;
    proc->user_mmap_cursor = 0;
    proc->user_stack_guard_page = 0;
    if (proc->compat_vm_map != NULL) {
        free(proc->compat_vm_map);
        proc->compat_vm_map = NULL;
    }
    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        proc->user_allocs[i].used = 0;
        proc->user_allocs[i].addr = 0;
        proc->user_allocs[i].size = 0;
    }
    for (uint32_t i = 0; i < PROCESS_SIGNAL_MAX; ++i) {
        proc->signal_handlers[i] = 0;
        proc->signal_flags[i] = 0;
        proc->signal_sa_mask[i] = 0;
        proc->signal_restorer[i] = 0;
    }
    proc->altstack_sp = 0;
    proc->altstack_size = 0;
    proc->altstack_flags = 2u; /* SS_DISABLE */
    proc->signal_mask = 0;
    proc->pending_signals = 0;
    proc->clear_child_tid = 0;
    proc->robust_list_head = 0;
    proc->robust_list_length = 0;
    proc->rseq_area = 0;
    proc->rseq_sig = 0;
}

static process_t *process_memory_owner_locked(process_t *proc)
{
    if (proc == NULL) {
        return NULL;
    }
    if (!proc->is_thread) {
        return proc;
    }
    if (!is_valid_pid(proc->memory_owner_pid)) {
        return NULL;
    }
    process_t *owner = &g_processes[proc->memory_owner_pid];
    if (owner->state == PROCESS_STATE_UNUSED || owner->is_thread) {
        return NULL;
    }
    return owner;
}

/* ------------------------------------------------------------------ */
/* Linux x86-64 rt_sigframe construction (TODO_Chromium_LinuxABI.md 3.5) */
/*                                                                      */
/* A signal is delivered to a process from two very different contexts: */
/*  - "pending" signals (kill/tkill/queued) are only actually applied at */
/*    a syscall-return scheduling boundary, where the process's full    */
/*    register state already lives in its SYSCALL_FRAME_* array         */
/*    (see process_push_signal_frame_locked below).                     */
/*  - a synchronous fault (SIGSEGV from a bad user-mode page fault) must */
/*    be applied *immediately*, while the process is still mid-         */
/*    instruction inside the page-fault ISR - there is no "next syscall */
/*    return" to hijack (see process_signal_deliver_fault_now below,    */
/*    called from Arch/x86_64/cpu/IDT_Main.c's page_fault_handler()).   */
/*                                                                      */
/* Both paths build the identical user-visible frame (ucontext_t +      */
/* siginfo_t, matching the real Linux x86-64 ABI layout closely enough  */
/* for glibc's signal trampoline and for a crash handler like Chromium's */
/* Breakpad to read si_addr / uc_mcontext.gregs[REG_RIP] etc.), via the */
/* shared write_signal_frame() below - only how the "current registers" */
/* are read and how "resume here instead" is written back differs.     */
/* ------------------------------------------------------------------ */

#define LINUX_SA_SIGINFO   0x00000004u
#define LINUX_SA_ONSTACK   0x08000000u
#define LINUX_SA_RESTART   0x10000000u
#define LINUX_SA_NODEFER   0x40000000u
#define LINUX_SA_RESETHAND 0x80000000u

#define LINUX_SS_ONSTACK 1u
#define LINUX_SS_DISABLE 2u

#define LINUX_SIGSEGV 11
#define LINUX_SIGBUS  7

/* General-purpose registers relevant to sigcontext, in a layout-neutral
 * struct (translated into the real Linux field order by the frame
 * writer below). rcx/r11 are only meaningful for the fault path - the
 * syscall ABI itself destroys the true RCX/R11 (SYSCALL stores the
 * return RIP in RCX and RFLAGS in R11), so the syscall-boundary path
 * cannot recover their original values and reports 0. */
typedef struct {
    uint64_t r8, r9, r10, r11, r12, r13, r14, r15;
    uint64_t rdi, rsi, rbp, rbx, rdx, rax, rcx;
} linux_gpregs_t;

typedef struct {
    uint64_t r8, r9, r10, r11, r12, r13, r14, r15;
    uint64_t rdi, rsi, rbp, rbx, rdx, rax, rcx, rsp;
    uint64_t rip;
    uint64_t eflags;
    uint16_t cs, gs, fs, ss;
    uint64_t err;
    uint64_t trapno;
    uint64_t oldmask;
    uint64_t cr2;
    uint64_t fpstate; /* user address of the 512-byte FXSAVE image, or 0 */
    uint64_t reserved1[8];
} linux_sigcontext_t; /* 256 bytes, matches Linux asm/sigcontext.h */

typedef struct {
    uint64_t ss_sp;
    int32_t ss_flags;
    int32_t __pad;
    uint64_t ss_size;
} linux_stack_t; /* 24 bytes, matches Linux stack_t */

typedef struct {
    uint64_t uc_flags;
    uint64_t uc_link;
    linux_stack_t uc_stack;
    linux_sigcontext_t uc_mcontext;
    uint8_t uc_sigmask[128]; /* glibc sigset_t; only the low 8 bytes are used */
} linux_ucontext_t; /* 8+8+24+256+128 = 424 bytes */

typedef struct {
    int32_t si_signo;
    int32_t si_errno;
    int32_t si_code;
    int32_t __pad0;
    uint64_t si_addr; /* only sigfault.si_addr is populated */
    uint8_t __pad1[128 - 24];
} linux_siginfo_t; /* 128 bytes, matches Linux siginfo_t */

typedef struct {
    uint64_t pretcode;
    linux_ucontext_t uc;
    linux_siginfo_t info;
} linux_rt_sigframe_t;

/* Builds a Linux-ABI rt_sigframe on `proc`'s user stack (switching to its
 * address space to do so) and reports where execution should resume
 * (`*out_new_rip`/`*out_new_rsp`) to run the handler. Does not touch
 * `proc`'s saved register state itself - callers apply the two output
 * values to whichever register-save area their context uses. Returns 0
 * on success, <0 on failure (bad handler/mask copy, corrupt stack, OOM
 * of stack space) - the caller should then fall back to terminating the
 * process, exactly like the pre-existing "no handler" behavior. */
/* Size of the FXSAVE image stored above the rt_sigframe and pointed to by
 * uc_mcontext.fpstate (Linux's struct _fpstate_64 without the XSAVE tail). */
#define LINUX_SIGFRAME_FPSTATE_SIZE 512u

static int write_signal_frame_locked(process_t *proc, int32_t signum,
                                     uint64_t handler,
                                     const linux_gpregs_t *regs,
                                     uint64_t old_rip, uint64_t old_rsp,
                                     uint64_t old_rflags, uint64_t si_addr,
                                     int32_t si_code,
                                     const uint8_t *fxstate,
                                     uint64_t trapno, uint64_t err,
                                     uint64_t *out_new_rip,
                                     uint64_t *out_new_rsp)
{
    if (proc == NULL || regs == NULL || out_new_rip == NULL ||
        out_new_rsp == NULL) {
        return -1;
    }

    uint64_t flags = proc->signal_flags[(uint32_t)signum];
    uint64_t restorer = proc->signal_restorer[(uint32_t)signum];
    int use_altstack =
        (flags & LINUX_SA_ONSTACK) != 0u &&
        (proc->altstack_flags & LINUX_SS_DISABLE) == 0u &&
        (old_rsp < proc->altstack_sp ||
         old_rsp >= proc->altstack_sp + proc->altstack_size);

    uint64_t base;
    if (use_altstack) {
        base = proc->altstack_sp + proc->altstack_size;
    } else {
        base = old_rsp - 128ULL; /* skip the SysV red zone */
    }

    /* Linux layout: the FPU image sits above the frame, 64-byte aligned.
     * The interrupted code's vector registers and MXCSR have to survive the
     * handler: glibc and the compiler assume a signal never changes them,
     * and Chromium's handlers (and the code they call) use SSE freely. */
    uint64_t fp_addr = 0;
    uint64_t sp = base;
    if (fxstate != NULL) {
        sp -= LINUX_SIGFRAME_FPSTATE_SIZE;
        sp &= ~0x3FULL;
        fp_addr = sp;
    }
    sp -= sizeof(linux_rt_sigframe_t);
    sp &= ~0xFULL;
    sp -= 8ULL; /* land at sp%16==8, matching "just after a call" ABI state */

    uint64_t stack_low = use_altstack ? proc->altstack_sp : proc->user_stack_base;
    uint64_t stack_high = use_altstack ?
        (proc->altstack_sp + proc->altstack_size) : proc->user_stack_top;
    if (sp < stack_low || base > stack_high) {
        return -1; /* Would overflow the target stack. */
    }

    linux_rt_sigframe_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.pretcode = (restorer != 0u) ? restorer : old_rip;

    frame.uc.uc_flags = 0;
    frame.uc.uc_link = 0;
    frame.uc.uc_stack.ss_sp = proc->altstack_sp;
    frame.uc.uc_stack.ss_flags =
        (int32_t)((proc->altstack_flags & LINUX_SS_DISABLE) != 0u ? 2 : 0);
    frame.uc.uc_stack.ss_size = proc->altstack_size;

    linux_sigcontext_t *mc = &frame.uc.uc_mcontext;
    mc->r8 = regs->r8;   mc->r9 = regs->r9;   mc->r10 = regs->r10;
    mc->r11 = regs->r11; mc->r12 = regs->r12; mc->r13 = regs->r13;
    mc->r14 = regs->r14; mc->r15 = regs->r15;
    mc->rdi = regs->rdi; mc->rsi = regs->rsi; mc->rbp = regs->rbp;
    mc->rbx = regs->rbx; mc->rdx = regs->rdx; mc->rax = regs->rax;
    mc->rcx = regs->rcx; mc->rsp = old_rsp;    mc->rip = old_rip;
    mc->eflags = old_rflags;
    mc->cs = 0; mc->gs = 0; mc->fs = 0; mc->ss = 0;
    mc->err = err; mc->trapno = trapno;
    mc->oldmask = proc->signal_mask;
    mc->cr2 = (trapno == 14u) ? si_addr : 0;
    mc->fpstate = fp_addr;

    memcpy(frame.uc.uc_sigmask, &proc->signal_mask, sizeof(proc->signal_mask));

    frame.info.si_signo = signum;
    frame.info.si_errno = 0;
    frame.info.si_code = si_code;
    frame.info.si_addr = si_addr;

    uint64_t old_cr3 = paging_get_active_cr3();
    paging_switch_cr3(proc->cr3);
    int copy_ok = process_user_buffer_is_valid((void *)(uintptr_t)sp,
                                               (size_t)(base - sp));
    /* Make every page of the frame present first. The memcpy below runs
     * with g_process_table_lock held, and a demand fault taken inside it
     * would enter the page-fault handler, which takes that same lock: the
     * CPU deadlocks on itself and the rest follow it (a sigaltstack or a
     * deep thread stack is lazily committed, so this is the common case,
     * not a corner). paging_handle_swap_fault() is what that handler would
     * have done, and it needs only its own lock. */
    if (copy_ok) {
        for (uint64_t pg = sp & ~0xFFFULL; pg < base; pg += 0x1000u) {
            if (paging_virt_to_phys(proc->cr3, pg) == 0u &&
                paging_handle_swap_fault(proc->cr3, pg) <= 0) {
                copy_ok = 0;
                break;
            }
        }
    }
    if (copy_ok) {
#if KERNEL_COW_FORK
        /* The target's stack may still be shared with its fork parent, and
         * this memcpy does not trap. */
        (void)paging_user_range_break_cow(proc->cr3, sp, base - sp);
#endif
        memcpy((void *)(uintptr_t)sp, &frame, sizeof(frame));
        if (fp_addr != 0u) {
            memcpy((void *)(uintptr_t)fp_addr, fxstate,
                   LINUX_SIGFRAME_FPSTATE_SIZE);
        }
    }
    paging_switch_cr3(old_cr3);
    if (!copy_ok) {
        return -1;
    }

    /* Block the signal itself (unless SA_NODEFER) plus whatever sa_mask
     * requested, for the duration of the handler; rt_sigreturn restores
     * uc_sigmask (saved above) once the handler returns. SIGKILL/SIGSTOP
     * remain always-deliverable (mirrors process_signal_blocked_locked). */
    uint64_t new_mask = proc->signal_mask | proc->signal_sa_mask[(uint32_t)signum];
    if ((flags & LINUX_SA_NODEFER) == 0u) {
        new_mask |= (1ULL << ((uint32_t)signum - 1u));
    }
    new_mask &= ~((1ULL << (9u - 1u)) | (1ULL << (19u - 1u)));
    proc->signal_mask = new_mask;

    if ((flags & LINUX_SA_RESETHAND) != 0u) {
        proc->signal_handlers[(uint32_t)signum] = 0;
    }

    *out_new_rip = handler;
    *out_new_rsp = sp;
    return 0;
}

/* A process killed by a signal's default action.
 *
 * This runs in the scheduler, under the process table lock, as the victim is
 * about to be resumed -- not in the victim's own context, so it cannot run
 * process_exit_current(). Marking the slot a zombie used to be all that
 * happened: its descriptors, sockets, pty, shared memory and DRM session
 * stayed open with nobody to close them. A SIGKILLed Xorg went on
 * "listening" on its socket, and a shell killed by SIGHUP kept its pty slave
 * open, so the xterm waiting for EOF on it never closed its window. The
 * teardown is queued instead (process_run_pending_teardowns()), and the
 * parent is told only once it has run -- reaping before it would release a
 * slot whose descriptors were still being closed. */
static volatile uint32_t g_teardown_pending;

static void process_signal_kill_locked(process_t *proc, int32_t signum)
{
    process_t *owner = process_memory_owner_locked(proc);
    if (owner == NULL) owner = proc;
    if (owner->state != PROCESS_STATE_ZOMBIE || owner->teardown_pending) {
        owner->exit_status = 128 + signum;
        owner->exit_by_signal = 1u;
        owner->exit_term_signal = (uint8_t)signum;
    }
    proc->exit_status = owner->exit_status;
    proc->exit_by_signal = owner->exit_by_signal;
    proc->exit_term_signal = owner->exit_term_signal;
    if (owner->state != PROCESS_STATE_ZOMBIE && !owner->teardown_pending) {
        owner->teardown_pending = 1u;
        __atomic_fetch_add(&g_teardown_pending, 1u, __ATOMIC_RELEASE);
    }
    owner->state = PROCESS_STATE_ZOMBIE;
    proc->state = PROCESS_STATE_ZOMBIE;
}

/* ------------------------------------------------------------------ */
/* Signal frames of the other two Linux ABIs.                          */
/*                                                                      */
/* Same rules as the x86-64 frame above -- the interrupted registers,   */
/* signal mask and FP state go on the user stack (or the alternate      */
/* one), the handler runs on top of it, and rt_sigreturn / sigreturn    */
/* puts everything back -- in the layouts the i386 and AArch64 kernels  */
/* use (arch/x86/include/asm/sigframe.h, arch/arm64/kernel/signal.c),   */
/* since libc and debuggers read them field by field.                   */
/* ------------------------------------------------------------------ */

#define LINUX_SA_RESTORER 0x04000000u

/* Offsets of the signal-return trampolines in the page process_map_sigpage()
 * maps into every Linux i386 / AArch64 process. */
#define SIGPAGE_I386_SIGRETURN     0u  /* popl %eax; movl $119,%eax; int $0x80 */
#define SIGPAGE_I386_RT_SIGRETURN 16u  /* movl $173,%eax; int $0x80 */
#define SIGPAGE_ARM64_RT_SIGRETURN 0u  /* mov x8,#139; svc #0 */

#if defined(__x86_64__)
static void compat32_capture_selectors(process_t *proc);
#endif

int linux_signal_interrupts_wait(int32_t tid, uint64_t frame_num,
                                 uint64_t frame_rip);

/* Handler bookkeeping common to every ABI: block the signal (unless
 * SA_NODEFER) and sa_mask for the handler's duration, SA_RESETHAND. */
static void compat_signal_enter_handler_locked(process_t *proc, int32_t signum)
{
    uint64_t flags = proc->signal_flags[(uint32_t)signum];
    uint64_t new_mask = proc->signal_mask | proc->signal_sa_mask[(uint32_t)signum];
    if ((flags & LINUX_SA_NODEFER) == 0u) {
        new_mask |= (1ULL << ((uint32_t)signum - 1u));
    }
    new_mask &= ~((1ULL << (9u - 1u)) | (1ULL << (19u - 1u)));
    proc->signal_mask = new_mask;
    if ((flags & LINUX_SA_RESETHAND) != 0u) {
        proc->signal_handlers[(uint32_t)signum] = 0;
    }
}

/* Where the frame goes: the alternate stack when the handler asked for it
 * and the task is not on it already, else just below `sp` (neither ABI has
 * a red zone). */
static uint64_t compat_signal_stack_top(const process_t *proc, int32_t signum,
                                        uint64_t sp, int *on_altstack)
{
    uint64_t flags = proc->signal_flags[(uint32_t)signum];
    int use_alt = (flags & LINUX_SA_ONSTACK) != 0u &&
                  (proc->altstack_flags & LINUX_SS_DISABLE) == 0u &&
                  proc->altstack_size != 0u &&
                  (sp < proc->altstack_sp ||
                   sp >= proc->altstack_sp + proc->altstack_size);
    *on_altstack = use_alt;
    return use_alt ? proc->altstack_sp + proc->altstack_size : sp;
}

/* Make every page of [lo, hi) of `proc`'s address space present and
 * private before the kernel writes there with the table lock held (see the
 * comment in write_signal_frame_locked() for why a fault there would
 * deadlock). Runs with proc's address space active. */
static int compat_signal_prepare_user_range(process_t *proc, uint64_t lo,
                                            uint64_t hi)
{
    if (!process_user_buffer_is_valid((void *)(uintptr_t)lo, hi - lo)) {
        return -1;
    }
    for (uint64_t pg = lo & ~0xFFFULL; pg < hi; pg += 0x1000u) {
        if (paging_virt_to_phys(proc->cr3, pg) == 0u &&
            paging_handle_swap_fault(proc->cr3, pg) <= 0) {
            return -1;
        }
    }
#if KERNEL_COW_FORK
    (void)paging_user_range_break_cow(proc->cr3, lo, hi - lo);
#endif
    return 0;
}

#if defined(__x86_64__)
/* ---- Linux i386 -------------------------------------------------- */

typedef struct __attribute__((packed)) {
    uint16_t gs, gsh, fs, fsh, es, esh, ds, dsh;
    uint32_t edi, esi, ebp, esp, ebx, edx, ecx, eax;
    uint32_t trapno, err, eip;
    uint16_t cs, csh;
    uint32_t eflags, esp_at_signal;
    uint16_t ss, ssh;
    uint32_t fpstate, oldmask, cr2;
} linux32_sigcontext_t;

typedef struct __attribute__((packed)) {
    uint32_t ss_sp;
    int32_t  ss_flags;
    uint32_t ss_size;
} linux32_stack_t;

typedef struct __attribute__((packed)) {
    uint32_t uc_flags;
    uint32_t uc_link;
    linux32_stack_t uc_stack;
    linux32_sigcontext_t uc_mcontext;
    uint32_t uc_sigmask[2];
} linux32_ucontext_t;

typedef struct __attribute__((packed)) {
    int32_t  si_signo;
    int32_t  si_errno;
    int32_t  si_code;
    uint32_t si_addr;
    uint8_t  __pad[128 - 16];
} linux32_siginfo_t;

/* struct _fpstate_32: the legacy FSAVE header (112 bytes), then the 512-byte
 * FXSAVE image, then reserved words. */
#define LINUX32_FPSTATE_SIZE   624u
#define LINUX32_FPSTATE_FXSAVE 112u
#define LINUX32_FPSTATE_MAGIC  110u  /* u16: 0 = FXSAVE data present */

typedef struct __attribute__((packed)) {
    uint32_t pretcode;
    int32_t  sig;
    linux32_sigcontext_t sc;
    uint8_t  fpstate_unused[LINUX32_FPSTATE_SIZE];
    uint32_t extramask;
    uint8_t  retcode[8];
} linux32_sigframe_t;

typedef struct __attribute__((packed)) {
    uint32_t pretcode;
    int32_t  sig;
    uint32_t pinfo;
    uint32_t puc;
    linux32_siginfo_t info;
    linux32_ucontext_t uc;
    uint8_t  retcode[8];
} linux32_rt_sigframe_t;

_Static_assert(sizeof(linux32_sigcontext_t) == 88, "i386 sigcontext");
_Static_assert(sizeof(linux32_ucontext_t) == 116, "i386 ucontext");
_Static_assert(sizeof(linux32_sigframe_t) == 732, "i386 sigframe");
_Static_assert(sizeof(linux32_rt_sigframe_t) == 268, "i386 rt_sigframe");

typedef struct {
    uint32_t eax, ebx, ecx, edx, esi, edi, ebp, esp, eip, eflags;
} compat32_regs_t;

#define USER32_CS ((uint16_t)(GDT_USER_COMPAT_CODE | 3u))
#define USER32_DS ((uint16_t)(GDT_USER_DATA | 3u))

/* Builds the i386 frame (rt when the handler is SA_SIGINFO) on proc's
 * stack. On success *out has what the handler starts with: EIP, ESP, and
 * EAX/EDX/ECX = signal, &info, &uc (Linux passes them in registers as well
 * as on the stack). */
static int write_signal_frame32_locked(process_t *proc, int32_t signum,
                                       uint64_t handler,
                                       const compat32_regs_t *r,
                                       uint32_t si_addr, int32_t si_code,
                                       const uint8_t *fxstate,
                                       uint32_t trapno, uint32_t err,
                                       compat32_regs_t *out)
{
    uint64_t flags = proc->signal_flags[(uint32_t)signum];
    uint64_t restorer = proc->signal_restorer[(uint32_t)signum];
    int rt = (flags & LINUX_SA_SIGINFO) != 0u;
    int on_alt = 0;
    uint64_t base = compat_signal_stack_top(proc, signum, r->esp, &on_alt);

    uint64_t sp = base - LINUX32_FPSTATE_SIZE;
    sp &= ~0x3FULL;
    uint64_t fp_addr = sp;
    sp -= rt ? sizeof(linux32_rt_sigframe_t) : sizeof(linux32_sigframe_t);
    sp = ((sp + 4u) & ~0xFULL) - 4u; /* ((esp + 4) & 15) == 0 at handler entry */
    if (sp < USER32_WINDOW_BASE || base > USER32_WINDOW_LIMIT || sp >= base) {
        return -1;
    }

    linux32_sigcontext_t sc;
    memset(&sc, 0, sizeof(sc));
    sc.gs = proc->compat_gs_sel;
    sc.fs = proc->compat_fs_sel;
    sc.es = USER32_DS;
    sc.ds = USER32_DS;
    sc.edi = r->edi; sc.esi = r->esi; sc.ebp = r->ebp; sc.esp = r->esp;
    sc.ebx = r->ebx; sc.edx = r->edx; sc.ecx = r->ecx; sc.eax = r->eax;
    sc.trapno = trapno;
    sc.err = err;
    sc.eip = r->eip;
    sc.cs = USER32_CS;
    sc.eflags = r->eflags;
    sc.esp_at_signal = r->esp;
    sc.ss = USER32_DS;
    sc.fpstate = (uint32_t)fp_addr;
    sc.oldmask = (uint32_t)proc->signal_mask;
    sc.cr2 = (trapno == 14u) ? si_addr : 0u;

    uint8_t fp[LINUX32_FPSTATE_SIZE];
    memset(fp, 0, sizeof(fp));
    if (fxstate != NULL) {
        /* Legacy header from the FXSAVE environment, then the image. */
        uint32_t cw = (uint32_t)fxstate[0] | ((uint32_t)fxstate[1] << 8);
        uint32_t sw = (uint32_t)fxstate[2] | ((uint32_t)fxstate[3] << 8);
        memcpy(fp + 0, &cw, 4u);
        memcpy(fp + 4, &sw, 4u);
        uint32_t tag = 0xFFFFu;
        memcpy(fp + 8, &tag, 4u);
        memcpy(fp + 108, &sw, 2u);
        memcpy(fp + LINUX32_FPSTATE_FXSAVE, fxstate, 512u);
        memset(fp + LINUX32_FPSTATE_FXSAVE + 464u, 0, 48u); /* no XSAVE info */
    }
    uint16_t magic = fxstate != NULL ? 0x0000u : 0xFFFFu;
    memcpy(fp + LINUX32_FPSTATE_MAGIC, &magic, 2u);

    uint32_t pretcode = (uint32_t)(USER32_SIGPAGE +
        (rt ? SIGPAGE_I386_RT_SIGRETURN : SIGPAGE_I386_SIGRETURN));
    if ((flags & LINUX_SA_RESTORER) != 0u && restorer != 0u) {
        pretcode = (uint32_t)restorer;
    }

    union {
        linux32_sigframe_t legacy;
        linux32_rt_sigframe_t rt;
    } frame;
    memset(&frame, 0, sizeof(frame));
    uint32_t pinfo = 0, puc = 0;
    if (rt) {
        frame.rt.pretcode = pretcode;
        frame.rt.sig = signum;
        pinfo = (uint32_t)(sp + offsetof(linux32_rt_sigframe_t, info));
        puc = (uint32_t)(sp + offsetof(linux32_rt_sigframe_t, uc));
        frame.rt.pinfo = pinfo;
        frame.rt.puc = puc;
        frame.rt.info.si_signo = signum;
        frame.rt.info.si_code = si_code;
        frame.rt.info.si_addr = si_addr;
        frame.rt.uc.uc_stack.ss_sp = (uint32_t)proc->altstack_sp;
        frame.rt.uc.uc_stack.ss_flags =
            (proc->altstack_flags & LINUX_SS_DISABLE) != 0u ? 2 : (on_alt ? 1 : 0);
        frame.rt.uc.uc_stack.ss_size = (uint32_t)proc->altstack_size;
        frame.rt.uc.uc_mcontext = sc;
        frame.rt.uc.uc_sigmask[0] = (uint32_t)proc->signal_mask;
        frame.rt.uc.uc_sigmask[1] = (uint32_t)(proc->signal_mask >> 32);
        static const uint8_t rc[8] = { 0xB8, 0xAD, 0, 0, 0, 0xCD, 0x80, 0 };
        memcpy(frame.rt.retcode, rc, sizeof(rc));
    } else {
        frame.legacy.pretcode = pretcode;
        frame.legacy.sig = signum;
        frame.legacy.sc = sc;
        frame.legacy.extramask = (uint32_t)(proc->signal_mask >> 32);
        static const uint8_t rc[8] = { 0x58, 0xB8, 0x77, 0, 0, 0, 0xCD, 0x80 };
        memcpy(frame.legacy.retcode, rc, sizeof(rc));
    }
    uint64_t frame_size = rt ? sizeof(linux32_rt_sigframe_t)
                             : sizeof(linux32_sigframe_t);

    uint64_t old_cr3 = paging_get_active_cr3();
    paging_switch_cr3(proc->cr3);
    int ok = compat_signal_prepare_user_range(proc, sp, fp_addr + sizeof(fp)) == 0;
    if (ok) {
        memcpy((void *)(uintptr_t)sp, &frame, (size_t)frame_size);
        memcpy((void *)(uintptr_t)fp_addr, fp, sizeof(fp));
    }
    paging_switch_cr3(old_cr3);
    if (!ok) {
        return -1;
    }

    compat_signal_enter_handler_locked(proc, signum);

    *out = *r;
    out->eip = (uint32_t)handler;
    out->esp = (uint32_t)sp;
    out->eax = (uint32_t)signum;
    out->edx = pinfo;
    out->ecx = puc;
    out->eflags = r->eflags & ~(0x400u | 0x100u); /* DF, TF clear */
    return 0;
}

/* A pending signal for a Linux i386 task, at a scheduling boundary: its
 * registers are the syscall frame plus full_rcx (its real ECX). */
static int push_signal_frame32_locked(process_t *proc, int32_t signum,
                                      uint64_t handler)
{
    uint64_t *frame = (uint64_t *)(uintptr_t)proc->saved_rsp;
    if (frame == NULL) {
        return -1;
    }
    /* A poll/select/epoll wait parked for re-execution: EINTR instead (see
     * linux_signal_interrupts_wait()). int $0x80 is two bytes like SYSCALL. */
    if (linux_signal_interrupts_wait((int32_t)(proc - g_processes),
                                     frame[SYSCALL_FRAME_RAX],
                                     frame[SYSCALL_FRAME_RCX])) {
        frame[SYSCALL_FRAME_RAX] = (uint32_t)-4; /* -EINTR */
        frame[SYSCALL_FRAME_RCX] += 2u;
    }

    compat32_regs_t r;
    r.eax = (uint32_t)frame[SYSCALL_FRAME_RAX];
    r.ebx = (uint32_t)frame[SYSCALL_FRAME_RBX];
    r.ecx = (uint32_t)proc->full_rcx;
    r.edx = (uint32_t)frame[SYSCALL_FRAME_RDX];
    r.esi = (uint32_t)frame[SYSCALL_FRAME_RSI];
    r.edi = (uint32_t)frame[SYSCALL_FRAME_RDI];
    r.ebp = (uint32_t)frame[SYSCALL_FRAME_RBP];
    r.esp = (uint32_t)proc->saved_user_rsp;
    r.eip = (uint32_t)frame[SYSCALL_FRAME_RCX];
    r.eflags = (uint32_t)frame[SYSCALL_FRAME_R11];

    compat32_regs_t out;
    if (write_signal_frame32_locked(proc, signum, handler, &r, 0u, 0 /* SI_USER */,
                                    proc->fpu_state, 0u, 0u, &out) < 0) {
        return -1;
    }
    frame[SYSCALL_FRAME_RCX] = out.eip;
    frame[SYSCALL_FRAME_R11] = out.eflags;
    frame[SYSCALL_FRAME_RAX] = out.eax;
    frame[SYSCALL_FRAME_RDX] = out.edx;
    proc->full_restore = 1;
    proc->full_rcx = out.ecx;
    proc->saved_user_rsp = out.esp;
    return 0;
}
#endif /* __x86_64__ */

#if defined(__aarch64__)
/* ---- Linux AArch64 ----------------------------------------------- */

#define ARM64_SIGFRAME_SIGINFO   0u
#define ARM64_SIGFRAME_UC        128u
#define ARM64_UC_SIGMASK         40u    /* within ucontext */
#define ARM64_UC_MCONTEXT        176u
#define ARM64_MC_FAULT_ADDRESS   0u     /* within sigcontext */
#define ARM64_MC_REGS            8u
#define ARM64_MC_SP              256u
#define ARM64_MC_PC              264u
#define ARM64_MC_PSTATE          272u
#define ARM64_MC_RESERVED        288u
#define ARM64_SIGFRAME_SIZE      (ARM64_SIGFRAME_UC + ARM64_UC_MCONTEXT + \
                                  ARM64_MC_RESERVED + 4096u)
#define ARM64_FPSIMD_MAGIC       0x46508001u
#define ARM64_FPSIMD_SIZE        528u
#define ARM64_PSTATE_USER_MASK   0xF0000000ULL  /* NZCV; EL0t, nothing masked */

/* Builds the AArch64 rt_sigframe for a handler, from `f` (the task's
 * exception frame) and `fpu` (its FP/SIMD state in hal_cpu_save_fpu()
 * layout: q0-q31, then FPSR, then FPCR), and points `f` at the handler. */
static int push_signal_frame_arm64_locked(process_t *proc, int32_t signum,
                                          uint64_t handler,
                                          arm64_exception_frame_t *f,
                                          uint64_t old_sp,
                                          uint64_t si_addr, int32_t si_code,
                                          const uint8_t *fpu)
{
    uint64_t flags = proc->signal_flags[(uint32_t)signum];
    uint64_t restorer = proc->signal_restorer[(uint32_t)signum];
    int on_alt = 0;
    uint64_t top = compat_signal_stack_top(proc, signum, old_sp, &on_alt);

    uint64_t record = (top - 16u) & ~0xFULL;     /* struct frame_record */
    uint64_t sp = record - ARM64_SIGFRAME_SIZE;  /* 16-aligned: size is */
    if (sp >= top) {
        return -1;
    }

    uint64_t old_cr3 = paging_get_active_cr3();
    paging_switch_cr3(proc->cr3);
    if (compat_signal_prepare_user_range(proc, sp, record + 16u) < 0) {
        paging_switch_cr3(old_cr3);
        return -1;
    }
    uint8_t *base = (uint8_t *)(uintptr_t)sp;
    memset(base, 0, ARM64_SIGFRAME_SIZE);

    linux_siginfo_t info;
    memset(&info, 0, sizeof(info));
    info.si_signo = signum;
    info.si_code = si_code;
    info.si_addr = si_addr;
    memcpy(base + ARM64_SIGFRAME_SIGINFO, &info, sizeof(info));

    uint8_t *uc = base + ARM64_SIGFRAME_UC;
    linux_stack_t st;
    st.ss_sp = proc->altstack_sp;
    st.ss_flags = (proc->altstack_flags & LINUX_SS_DISABLE) != 0u ? 2 : (on_alt ? 1 : 0);
    st.__pad = 0;
    st.ss_size = proc->altstack_size;
    memcpy(uc + 16u, &st, sizeof(st));
    memcpy(uc + ARM64_UC_SIGMASK, &proc->signal_mask, sizeof(uint64_t));

    uint8_t *mc = uc + ARM64_UC_MCONTEXT;
    memcpy(mc + ARM64_MC_FAULT_ADDRESS, &si_addr, 8u);
    memcpy(mc + ARM64_MC_REGS, f->x, sizeof(f->x));
    memcpy(mc + ARM64_MC_SP, &old_sp, 8u);
    memcpy(mc + ARM64_MC_PC, &f->elr_el1, 8u);
    memcpy(mc + ARM64_MC_PSTATE, &f->spsr_el1, 8u);

    uint8_t *fp = mc + ARM64_MC_RESERVED;
    uint32_t head[4] = { ARM64_FPSIMD_MAGIC, ARM64_FPSIMD_SIZE, 0u, 0u };
    memcpy(&head[2], fpu + 512u, 4u);     /* fpsr */
    memcpy(&head[3], fpu + 520u, 4u);     /* fpcr */
    memcpy(fp, head, sizeof(head));
    memcpy(fp + 16u, fpu, 512u);          /* v0-v31 */
    /* The terminator record ({0, 0}) right after is already zero. */

    uint64_t rec[2] = { f->x[29], f->x[30] };
    memcpy((void *)(uintptr_t)record, rec, sizeof(rec));
    paging_switch_cr3(old_cr3);

    compat_signal_enter_handler_locked(proc, signum);

    f->x[0] = (uint64_t)(uint32_t)signum;
    if ((flags & LINUX_SA_SIGINFO) != 0u) {
        f->x[1] = sp + ARM64_SIGFRAME_SIGINFO;
        f->x[2] = sp + ARM64_SIGFRAME_UC;
    }
    f->x[29] = record;
    f->x[30] = ((flags & LINUX_SA_RESTORER) != 0u && restorer != 0u) ?
        restorer : (USER_ARM64_SIGPAGE + SIGPAGE_ARM64_RT_SIGRETURN);
    f->elr_el1 = handler;
    f->spsr_el1 &= ARM64_PSTATE_USER_MASK;
    f->sp_el0 = sp;
    proc->saved_user_rsp = sp;
    return 0;
}
#endif /* __aarch64__ */

static int process_push_signal_frame_locked(process_t *proc, int32_t signum)
{
    if (proc == NULL || signum <= 0 || signum >= PROCESS_SIGNAL_MAX) {
        return -1;
    }

    uint64_t handler = proc->signal_handlers[(uint32_t)signum];
    if (handler == 1u) {
        proc->pending_signals &= ~(1u << (uint32_t)signum);
        return 0;
    }
    if (handler == 0) {
        process_signal_kill_locked(proc, signum);
        return -1;
    }

#if defined(__x86_64__)
    if (PROCESS_ABI_IS_COMPAT32(proc->abi_mode)) {
        if (push_signal_frame32_locked(proc, signum, handler) < 0) {
            process_signal_kill_locked(proc, signum);
            return -1;
        }
        proc->pending_signals &= ~(1u << (uint32_t)signum);
        return 0;
    }

    uint64_t *frame = (uint64_t *)(uintptr_t)proc->saved_rsp;
    if (frame == NULL || proc->saved_user_rsp < (proc->user_stack_base + 16ULL)) {
        process_signal_kill_locked(proc, signum);
        return -1;
    }

    /* The task is parked in poll/select/epoll_wait with its frame rewound to
     * re-run the syscall. Linux interrupts those waits with EINTR when a
     * handler runs, so make the frame the handler returns to an EINTR
     * return rather than the re-run (see linux_signal_interrupts_wait()). */
    {
        extern int linux_signal_interrupts_wait(int32_t tid, uint64_t frame_num,
                                                uint64_t frame_rip);
        if (!proc->full_restore &&
            linux_signal_interrupts_wait((int32_t)(proc - g_processes),
                                         frame[SYSCALL_FRAME_RAX],
                                         frame[SYSCALL_FRAME_RCX])) {
            frame[SYSCALL_FRAME_RAX] = (uint64_t)(int64_t)-4; /* -EINTR */
            frame[SYSCALL_FRAME_RCX] += 2u; /* past the 2-byte SYSCALL */
        }
    }

    linux_gpregs_t regs;
    memset(&regs, 0, sizeof(regs));
    regs.r8  = frame[SYSCALL_FRAME_R8];
    regs.r9  = frame[SYSCALL_FRAME_R9];
    regs.r10 = frame[SYSCALL_FRAME_R10];
    regs.r12 = frame[SYSCALL_FRAME_R12];
    regs.r13 = frame[SYSCALL_FRAME_R13];
    regs.r14 = frame[SYSCALL_FRAME_R14];
    regs.r15 = frame[SYSCALL_FRAME_R15];
    regs.rdi = frame[SYSCALL_FRAME_RDI];
    regs.rsi = frame[SYSCALL_FRAME_RSI];
    regs.rbp = frame[SYSCALL_FRAME_RBP];
    regs.rbx = frame[SYSCALL_FRAME_RBX];
    regs.rdx = frame[SYSCALL_FRAME_RDX];
    regs.rax = frame[SYSCALL_FRAME_RAX];
    /* regs.rcx / regs.r11: unrecoverable at a syscall boundary (SYSCALL
     * itself clobbers them with the return RIP/RFLAGS) - left as 0. A task
     * preempted in user mode, though, still has them (full_restore); they
     * go into the frame so rt_sigreturn can put them back. */
    if (proc->full_restore) {
        regs.rcx = proc->full_rcx;
        regs.r11 = proc->full_r11;
        proc->full_restore = 0;
    }

    uint64_t old_rip = frame[SYSCALL_FRAME_RCX];
    uint64_t old_rflags = frame[SYSCALL_FRAME_R11];
    uint64_t old_rsp = proc->saved_user_rsp;

    /* proc is off the CPU (or its registers were just saved by the
     * scheduler), so fpu_state is its live FPU/SSE state. */
    uint64_t new_rip = 0, new_rsp = 0;
    if (write_signal_frame_locked(proc, signum, handler, &regs, old_rip,
                                  old_rsp, old_rflags, 0, 0 /* SI_USER */,
                                  proc->fpu_state, 0, 0,
                                  &new_rip, &new_rsp) < 0) {
        process_signal_kill_locked(proc, signum);
        return -1;
    }

    frame[SYSCALL_FRAME_RCX] = new_rip;
    /* The handler starts with DF and TF clear, as on Linux. */
    frame[SYSCALL_FRAME_R11] = old_rflags & ~(0x400ULL | 0x100ULL);
    frame[SYSCALL_FRAME_RDI] = (uint64_t)(uint32_t)signum;
    if ((proc->signal_flags[(uint32_t)signum] & LINUX_SA_SIGINFO) != 0u) {
        /* &frame->uc / &frame->info within the rt_sigframe just written
         * at new_rsp (linux_rt_sigframe_t: pretcode, then uc, then info). */
        frame[SYSCALL_FRAME_RSI] = new_rsp + offsetof(linux_rt_sigframe_t, info);
        frame[SYSCALL_FRAME_RDX] = new_rsp + offsetof(linux_rt_sigframe_t, uc);
    }
    proc->saved_user_rsp = new_rsp;
    proc->pending_signals &= ~(1u << (uint32_t)signum);
    return 0;
#elif defined(__aarch64__)
    arm64_exception_frame_t *f = (arm64_exception_frame_t *)(uintptr_t)proc->saved_rsp;
    if (proc->abi_mode != PROCESS_ABI_LINUX_ARM64 || f == NULL) {
        process_signal_kill_locked(proc, signum);
        return -1;
    }
    /* A wait parked for re-execution returns EINTR instead: x8 holds the
     * syscall number again and the PC was stepped back over the svc. */
    if (linux_signal_interrupts_wait((int32_t)(proc - g_processes),
                                     f->x[8], f->elr_el1)) {
        f->x[0] = (uint64_t)(int64_t)-4; /* -EINTR */
        f->elr_el1 += 4u;
    }
    if (push_signal_frame_arm64_locked(proc, signum, handler, f,
                                       proc->saved_user_rsp, 0u, 0 /* SI_USER */,
                                       proc->fpu_state) < 0) {
        process_signal_kill_locked(proc, signum);
        return -1;
    }
    proc->pending_signals &= ~(1u << (uint32_t)signum);
    return 0;
#else
    process_signal_kill_locked(proc, signum);
    return -1;
#endif
}

/* Raise SIGCHLD on a terminated process's parent. Caller holds
 * g_process_table_lock. SIGCHLD's POSIX default disposition is "ignore", but
 * our delivery path treats an unhandled signal as fatal, so only post it when
 * the parent installed an actual catcher (disposition > SIG_IGN). Parents that
 * just poll wait4()/waitid() are unaffected. */
static void process_notify_parent_sigchld_locked(const process_t *child)
{
    if (child == NULL) {
        return;
    }
    int32_t ppid = child->parent_pid;
    if (!is_valid_pid(ppid)) {
        return;
    }
    process_t *parent = &g_processes[ppid];
    if (parent->state == PROCESS_STATE_UNUSED ||
        parent->state == PROCESS_STATE_DEAD ||
        parent->state == PROCESS_STATE_ZOMBIE) {
        return;
    }
    if (parent->signal_handlers[17] > 1u) {
        parent->pending_signals |= (1u << 17u); /* SIGCHLD */
    }
}

static void process_deliver_pending_signals_locked(process_t *proc)
{
    if (proc == NULL || proc->pending_signals == 0) {
        return;
    }

    for (uint32_t signum = 1; signum < PROCESS_SIGNAL_MAX; ++signum) {
        uint32_t bit = 1u << signum;
        if ((proc->pending_signals & bit) == 0u) {
            continue;
        }
        if ((proc->signal_mask & (1ULL << (signum - 1u))) != 0u &&
            signum != 9u && signum != 19u) {
            continue;
        }
        if (process_push_signal_frame_locked(proc, (int32_t)signum) < 0) {
            proc->pending_signals &= ~bit;
            process_t *owner = process_memory_owner_locked(proc);
            if (proc->state == PROCESS_STATE_ZOMBIE &&
                (owner == NULL || !owner->teardown_pending)) {
                process_notify_parent_sigchld_locked(proc);
            }
        }
        break;
    }
}

#if defined(__x86_64__)
/* Called synchronously from Arch/x86_64/cpu/IDT_Main.c's page fault ISR
 * handler for a currently-executing user-mode fault, when the faulting
 * process/thread has a real SIGSEGV handler installed. `kernel_regs`
 * points at the base of the ISR's SAVE_REGS array (IDT.asm), in push
 * order rax,rbx,rcx,rdx,rsi,rdi,rbp,r8..r15 (so the lowest address holds
 * the *last* pushed register, r15); `cpu_frame` points at the CPU's own
 * interrupt frame {error_code, rip, cs, rflags, rsp, ss} (only present
 * with rsp/ss when a privilege change occurred, which is always true
 * here since this is only called for PF_USER faults). On success this
 * rewrites both arrays in place so that the ISR's existing "resume via
 * iretq" path (used already for the swap-fault-recovered case) lands
 * directly in the signal handler; the ISR itself needs no changes.
 * Returns 1 if the frame was built (caller should resume), 0 otherwise
 * (caller should fall back to terminating the process as before). */
int process_signal_deliver_trap_now(int32_t pid, int32_t signum,
                                    int32_t si_code, uint64_t si_addr,
                                    uint64_t trapno,
                                    uint64_t *kernel_regs,
                                    uint64_t *cpu_frame)
{
    if (kernel_regs == NULL || cpu_frame == NULL || signum <= 0 ||
        signum >= PROCESS_SIGNAL_MAX) {
        return 0;
    }

    /* SAVE_REGS push order (IDT.asm): rax,rbx,rcx,rdx,rsi,rdi,rbp,r8,r9,
     * r10,r11,r12,r13,r14,r15 - last pushed (r15) ends up at the lowest
     * address, i.e. kernel_regs[0]. */
    enum {
        KR_R15 = 0, KR_R14, KR_R13, KR_R12, KR_R11, KR_R10, KR_R9, KR_R8,
        KR_RBP, KR_RDI, KR_RSI, KR_RDX, KR_RCX, KR_RBX, KR_RAX
    };
    /* cpu_frame layout: [0]=error_code [1]=rip [2]=cs [3]=rflags [4]=rsp [5]=ss */
    enum { CF_ERR = 0, CF_RIP, CF_CS, CF_RFLAGS, CF_RSP, CF_SS };

    /* The faulting thread is the one on this CPU and the kernel never
     * touches the FPU (-mgeneral-regs-only), so the registers are still
     * exactly what the faulting code left in them. */
    uint8_t fxstate[512] __attribute__((aligned(16)));
    hal_cpu_save_fpu(fxstate);

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }
    process_t *proc = &g_processes[pid];
    uint64_t handler = (signum > 0 && signum < PROCESS_SIGNAL_MAX) ?
        proc->signal_handlers[(uint32_t)signum] : 0;
    /* A synchronous fault that is blocked cannot be deferred (the
     * instruction would just fault again): Linux kills the task, and so do
     * we, by reporting "not delivered". */
    int blocked = (proc->signal_mask & (1ULL << ((uint32_t)signum - 1u))) != 0u;
    if (handler == 0 || handler == 1u || blocked) {
        /* No handler (default = terminate) or explicitly ignored: a
         * synchronous fault cannot be ignored (matches Linux - an ignored
         * SIGSEGV still kills the process), so either way the caller should
         * terminate. */
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }

    if (PROCESS_ABI_IS_COMPAT32(proc->abi_mode)) {
        compat32_regs_t r32;
        r32.eax = (uint32_t)kernel_regs[KR_RAX];
        r32.ebx = (uint32_t)kernel_regs[KR_RBX];
        r32.ecx = (uint32_t)kernel_regs[KR_RCX];
        r32.edx = (uint32_t)kernel_regs[KR_RDX];
        r32.esi = (uint32_t)kernel_regs[KR_RSI];
        r32.edi = (uint32_t)kernel_regs[KR_RDI];
        r32.ebp = (uint32_t)kernel_regs[KR_RBP];
        r32.esp = (uint32_t)cpu_frame[CF_RSP];
        r32.eip = (uint32_t)cpu_frame[CF_RIP];
        r32.eflags = (uint32_t)cpu_frame[CF_RFLAGS];
        /* What the program has in its segment registers right now. */
        compat32_capture_selectors(proc);
        compat32_regs_t out;
        if (write_signal_frame32_locked(proc, signum, handler, &r32,
                                        (uint32_t)si_addr, si_code, fxstate,
                                        (uint32_t)trapno,
                                        (uint32_t)cpu_frame[CF_ERR],
                                        &out) < 0) {
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            return 0;
        }
        kernel_regs[KR_RAX] = out.eax;
        kernel_regs[KR_RDX] = out.edx;
        kernel_regs[KR_RCX] = out.ecx;
        cpu_frame[CF_RIP] = out.eip;
        cpu_frame[CF_RSP] = out.esp;
        cpu_frame[CF_RFLAGS] = out.eflags;
        proc->pending_signals &= ~(1u << (uint32_t)signum);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 1;
    }

    linux_gpregs_t regs;
    regs.r8 = kernel_regs[KR_R8];   regs.r9 = kernel_regs[KR_R9];
    regs.r10 = kernel_regs[KR_R10]; regs.r11 = kernel_regs[KR_R11];
    regs.r12 = kernel_regs[KR_R12]; regs.r13 = kernel_regs[KR_R13];
    regs.r14 = kernel_regs[KR_R14]; regs.r15 = kernel_regs[KR_R15];
    regs.rdi = kernel_regs[KR_RDI]; regs.rsi = kernel_regs[KR_RSI];
    regs.rbp = kernel_regs[KR_RBP]; regs.rbx = kernel_regs[KR_RBX];
    regs.rdx = kernel_regs[KR_RDX]; regs.rax = kernel_regs[KR_RAX];
    regs.rcx = kernel_regs[KR_RCX];

    uint64_t old_rip = cpu_frame[CF_RIP];
    uint64_t old_rflags = cpu_frame[CF_RFLAGS];
    uint64_t old_rsp = cpu_frame[CF_RSP];

    uint64_t new_rip = 0, new_rsp = 0;
    if (write_signal_frame_locked(proc, signum, handler, &regs, old_rip,
                                  old_rsp, old_rflags, si_addr, si_code,
                                  fxstate, trapno, cpu_frame[CF_ERR],
                                  &new_rip, &new_rsp) < 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }

    kernel_regs[KR_RDI] = (uint64_t)(uint32_t)signum;
    if ((proc->signal_flags[(uint32_t)signum] & LINUX_SA_SIGINFO) != 0u) {
        kernel_regs[KR_RSI] = new_rsp + offsetof(linux_rt_sigframe_t, info);
        kernel_regs[KR_RDX] = new_rsp + offsetof(linux_rt_sigframe_t, uc);
    }
    cpu_frame[CF_RIP] = new_rip;
    cpu_frame[CF_RSP] = new_rsp;
    /* The handler starts with DF and TF clear, as on Linux. */
    cpu_frame[CF_RFLAGS] = old_rflags & ~(0x400ULL | 0x100ULL);
    proc->pending_signals &= ~(1u << (uint32_t)signum);

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 1;
}

int process_signal_deliver_fault_now(int32_t pid, int32_t signum,
                                     uint64_t fault_addr,
                                     uint64_t *kernel_regs,
                                     uint64_t *cpu_frame)
{
    /* SEGV_MAPERR: we do not distinguish MAPERR/ACCERR. Page fault = 14. */
    return process_signal_deliver_trap_now(pid, signum, 1, fault_addr, 14u,
                                           kernel_regs, cpu_frame);
}
#endif

int64_t process_signal_rt_sigreturn(uint64_t saved_rsp)
{
#if defined(__x86_64__)
    /* IMPORTANT: unlike process_push_signal_frame_locked() (only ever
     * invoked from a scheduling boundary, where proc->saved_rsp/
     * saved_user_rsp have already been refreshed for the syscall that
     * just completed - see process_schedule_on_syscall()'s call order),
     * rt_sigreturn runs *during* the still-in-flight syscall that
     * invoked it. proc->saved_rsp/saved_user_rsp are therefore still
     * stale (left over from this process's *previous* syscall) at this
     * point - process_schedule_on_syscall() only refreshes them after
     * linux_syscall_dispatch() (and thus this function) returns. The
     * live state instead comes from `saved_rsp` (threaded through from
     * the dispatcher, exactly like linux_clone() does for the same
     * reason) and syscall_get_user_rsp()/syscall_set_user_rsp(). */
    uint64_t *frame = (uint64_t *)(uintptr_t)saved_rsp;
    if (frame == NULL) {
        return -1;
    }
    /* The restorer trampoline reached rt_sigreturn via a plain `syscall`
     * right after the handler's `ret` popped pretcode, so the current
     * user RSP is exactly &rt_sigframe.uc (see write_signal_frame_locked:
     * frame = {pretcode, uc, info}). */
    uint64_t uc_ptr = syscall_get_user_rsp();

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    process_t *proc = &g_processes[current_pid_get()];
    uint64_t cr3 = proc->cr3;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    linux_ucontext_t uc;
    uint64_t old_cr3 = paging_get_active_cr3();
    paging_switch_cr3(cr3);
    int copy_ok = process_user_buffer_is_valid((const void *)(uintptr_t)uc_ptr,
                                               sizeof(uc)) &&
                  (memcpy(&uc, (const void *)(uintptr_t)uc_ptr, sizeof(uc)), 1);
    paging_switch_cr3(old_cr3);
    if (!copy_ok) {
        return -1;
    }

    const linux_sigcontext_t *mc = &uc.uc_mcontext;

    /* A handler may point the context anywhere (V8's wasm trap handler
     * redirects RIP to a landing pad), but returning to a non-canonical RIP
     * would fault on the kernel's own IRETQ/SYSRET. Linux delivers SIGSEGV
     * for this; terminating is the same outcome for every real program. */
    {
        uint64_t top = mc->rip >> 47;
        if (top != 0u) {
            process_exit_current_signaled(11 /* SIGSEGV */);
            return -1;
        }
    }

    /* The FPU/SSE state the frame was built with (or the handler's edits to
     * it). The task is the one on this CPU and the kernel does not use the
     * FPU, so loading it here is what the task resumes with. MXCSR's
     * reserved bits make FXRSTOR fault, so they are masked off first. */
    if (mc->fpstate != 0u) {
        uint8_t fx[512] __attribute__((aligned(16)));
        int fp_ok = 0;
        paging_switch_cr3(cr3);
        if (process_user_buffer_is_valid((const void *)(uintptr_t)mc->fpstate,
                                         sizeof(fx))) {
            memcpy(fx, (const void *)(uintptr_t)mc->fpstate, sizeof(fx));
            fp_ok = 1;
        }
        paging_switch_cr3(old_cr3);
        if (fp_ok) {
            uint32_t mxcsr;
            memcpy(&mxcsr, fx + 24, sizeof(mxcsr));
            mxcsr &= 0x0000FFFFu;
            memcpy(fx + 24, &mxcsr, sizeof(mxcsr));
            hal_cpu_restore_fpu(fx);
        }
    }

    /* Linux's FIX_EFLAGS: only the arithmetic/direction/alignment flags come
     * from the frame; IF and the reserved bit 1 are forced on. TF is left
     * out since single-step traps are not supported. */
    uint64_t user_rflags = (mc->eflags & 0x50CD5ULL) | 0x202ULL;

    frame[SYSCALL_FRAME_R8]  = mc->r8;
    frame[SYSCALL_FRAME_R9]  = mc->r9;
    frame[SYSCALL_FRAME_R10] = mc->r10;
    frame[SYSCALL_FRAME_R12] = mc->r12;
    frame[SYSCALL_FRAME_R13] = mc->r13;
    frame[SYSCALL_FRAME_R14] = mc->r14;
    frame[SYSCALL_FRAME_R15] = mc->r15;
    frame[SYSCALL_FRAME_RDI] = mc->rdi;
    frame[SYSCALL_FRAME_RSI] = mc->rsi;
    frame[SYSCALL_FRAME_RBP] = mc->rbp;
    frame[SYSCALL_FRAME_RBX] = mc->rbx;
    frame[SYSCALL_FRAME_RDX] = mc->rdx;
    frame[SYSCALL_FRAME_RAX] = mc->rax;
    frame[SYSCALL_FRAME_RCX] = mc->rip;    /* RCX slot doubles as return RIP */
    frame[SYSCALL_FRAME_R11] = user_rflags; /* R11 slot doubles as RFLAGS */
    syscall_set_user_rsp(mc->rsp);

    uint64_t restored_mask;
    memcpy(&restored_mask, uc.uc_sigmask, sizeof(restored_mask));
    const uint64_t unmaskable = (1ULL << (9u - 1u)) | (1ULL << (19u - 1u));

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        process_t *self = &g_processes[current_pid_get()];
        self->signal_mask = restored_mask & ~unmaskable;
        /* The interrupted code may have been anywhere -- in the middle of
         * its own instructions for a fault or a preemption -- so it gets
         * RCX and R11 back too, through the IRETQ exit. */
        self->full_restore = 1;
        self->full_rcx = mc->rcx;
        self->full_r11 = mc->r11;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    return (int64_t)mc->rax;
#else
    (void)saved_rsp;
    return -1;
#endif
}

/* Restores what a compat signal frame saved: the signal mask (SIGKILL and
 * SIGSTOP stay deliverable). */
static void compat_signal_restore_mask(uint64_t mask)
{
    const uint64_t unmaskable = (1ULL << (9u - 1u)) | (1ULL << (19u - 1u));
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        g_processes[current_pid_get()].signal_mask = mask & ~unmaskable;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

#if defined(__x86_64__)
/* sigreturn(2) (rt == 0) and rt_sigreturn(2) of a Linux i386 program, from
 * the sigreturn trampoline: the handler's `ret` popped pretcode, and the
 * non-rt trampoline popped the signal number as well, so the frame starts
 * 4 (rt) or 8 bytes below the current %esp. Returns the restored EAX. */
int64_t process_signal_sigreturn_compat32(uint64_t saved_rsp, int rt)
{
    uint64_t *frame = (uint64_t *)(uintptr_t)saved_rsp;
    if (frame == NULL) {
        return -1;
    }
    uint64_t esp = syscall_get_user_rsp();
    linux32_sigcontext_t sc;
    uint64_t mask;
    if (rt) {
        linux32_rt_sigframe_t f;
        if (copy_from_user(&f, (const void *)(uintptr_t)(esp - 4u), sizeof(f)) != 0u) {
            process_exit_current_signaled(11 /* SIGSEGV */);
            return -1;
        }
        sc = f.uc.uc_mcontext;
        mask = (uint64_t)f.uc.uc_sigmask[0] | ((uint64_t)f.uc.uc_sigmask[1] << 32);
    } else {
        linux32_sigframe_t f;
        if (copy_from_user(&f, (const void *)(uintptr_t)(esp - 8u), sizeof(f)) != 0u) {
            process_exit_current_signaled(11 /* SIGSEGV */);
            return -1;
        }
        sc = f.sc;
        mask = (uint64_t)sc.oldmask | ((uint64_t)f.extramask << 32);
    }

    /* The FPU/SSE image the frame carried (or the handler's edits to it),
     * loaded into this CPU: the task is the one running, and the kernel
     * itself never touches the FPU. Reserved MXCSR bits would make FXRSTOR
     * fault, so they are masked. */
    if (sc.fpstate != 0u) {
        uint8_t fp[LINUX32_FPSTATE_SIZE];
        uint16_t magic = 0xFFFFu;
        if (copy_from_user(fp, (const void *)(uintptr_t)sc.fpstate, sizeof(fp)) == 0u) {
            memcpy(&magic, fp + LINUX32_FPSTATE_MAGIC, sizeof(magic));
        }
        if (magic == 0x0000u) {
            uint8_t fx[512] __attribute__((aligned(16)));
            memcpy(fx, fp + LINUX32_FPSTATE_FXSAVE, sizeof(fx));
            uint32_t mxcsr;
            memcpy(&mxcsr, fx + 24, sizeof(mxcsr));
            mxcsr &= 0x0000FFFFu;
            memcpy(fx + 24, &mxcsr, sizeof(mxcsr));
            hal_cpu_restore_fpu(fx);
        }
    }

    frame[SYSCALL_FRAME_RAX] = sc.eax;
    frame[SYSCALL_FRAME_RBX] = sc.ebx;
    frame[SYSCALL_FRAME_RDX] = sc.edx;
    frame[SYSCALL_FRAME_RSI] = sc.esi;
    frame[SYSCALL_FRAME_RDI] = sc.edi;
    frame[SYSCALL_FRAME_RBP] = sc.ebp;
    frame[SYSCALL_FRAME_RCX] = sc.eip;
    /* Linux FIX_EFLAGS: arithmetic, direction, alignment, IF forced on. */
    frame[SYSCALL_FRAME_R11] = (sc.eflags & 0x50CD5u) | 0x202u;
    syscall_set_user_rsp(sc.esp);

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        process_t *self = &g_processes[current_pid_get()];
        self->full_restore = 1;
        self->full_rcx = sc.ecx;
        self->full_r11 = 0;
        self->compat_fs_sel = sc.fs;
        self->compat_gs_sel = sc.gs;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    compat_signal_restore_mask(mask);
    return (int64_t)(uint32_t)sc.eax;
}
#endif

#if defined(__aarch64__)
/* rt_sigreturn(2) of a Linux AArch64 program: the trampoline runs with sp at
 * the rt_sigframe the kernel built. Returns the restored x0. */
int64_t process_signal_rt_sigreturn_arm64(uint64_t saved_rsp)
{
    arm64_exception_frame_t *f = (arm64_exception_frame_t *)(uintptr_t)saved_rsp;
    if (f == NULL) {
        return -1;
    }
    uint64_t sp = f->sp_el0;
    uint8_t mc[ARM64_MC_RESERVED + ARM64_FPSIMD_SIZE];
    uint64_t mask = 0;
    if ((sp & 0xFu) != 0u ||
        copy_from_user(&mask, (const void *)(uintptr_t)(sp + ARM64_SIGFRAME_UC +
                                                        ARM64_UC_SIGMASK),
                       sizeof(mask)) != 0u ||
        copy_from_user(mc, (const void *)(uintptr_t)(sp + ARM64_SIGFRAME_UC +
                                                     ARM64_UC_MCONTEXT),
                       sizeof(mc)) != 0u) {
        process_exit_current_signaled(11 /* SIGSEGV */);
        return -1;
    }

    uint64_t pstate, pc, new_sp;
    memcpy(&pstate, mc + ARM64_MC_PSTATE, 8u);
    memcpy(&pc, mc + ARM64_MC_PC, 8u);
    memcpy(&new_sp, mc + ARM64_MC_SP, 8u);
    /* Only EL0 AArch64 state may come back (Linux valid_user_regs()). */
    if ((pstate & 0x1FULL) != 0u) {
        process_exit_current_signaled(11 /* SIGSEGV */);
        return -1;
    }
    memcpy(f->x, mc + ARM64_MC_REGS, sizeof(f->x));
    f->elr_el1 = pc;
    f->spsr_el1 = pstate & ARM64_PSTATE_USER_MASK;
    f->sp_el0 = new_sp;
    syscall_set_user_rsp(new_sp);

    uint32_t head[4];
    memcpy(head, mc + ARM64_MC_RESERVED, sizeof(head));
    if (head[0] == ARM64_FPSIMD_MAGIC && head[1] >= ARM64_FPSIMD_SIZE) {
        uint8_t fpu[PROCESS_FPU_STATE_SIZE] __attribute__((aligned(16)));
        memset(fpu, 0, sizeof(fpu));
        memcpy(fpu, mc + ARM64_MC_RESERVED + 16u, 512u);
        memcpy(fpu + 512u, &head[2], 4u);
        memcpy(fpu + 520u, &head[3], 4u);
        hal_cpu_restore_fpu(fpu);
    }
    compat_signal_restore_mask(mask);
    return (int64_t)f->x[0];
}

/* A synchronous fault in a Linux AArch64 program (data/instruction abort,
 * undefined instruction, ...) with a handler installed: build the frame on
 * the spot from the exception frame and resume into the handler. Returns 1
 * when delivered, 0 when the caller should terminate the task as before. */
int process_signal_deliver_trap_now_arm64(arm64_exception_frame_t *f,
                                          int32_t signum, int32_t si_code,
                                          uint64_t si_addr)
{
    if (f == NULL || signum <= 0 || signum >= PROCESS_SIGNAL_MAX) {
        return 0;
    }
    uint8_t fpu[PROCESS_FPU_STATE_SIZE] __attribute__((aligned(16)));
    hal_cpu_save_fpu(fpu);

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t pid = current_pid_get();
    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }
    process_t *proc = &g_processes[pid];
    uint64_t handler = proc->signal_handlers[(uint32_t)signum];
    int blocked = (proc->signal_mask & (1ULL << ((uint32_t)signum - 1u))) != 0u;
    if (proc->abi_mode != PROCESS_ABI_LINUX_ARM64 || handler <= 1u || blocked) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }
    int rc = push_signal_frame_arm64_locked(proc, signum, handler, f, f->sp_el0,
                                            si_addr, si_code, fpu);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return rc == 0 ? 1 : 0;
}
#endif

static void release_process_table(void)
{
    if (!process_table_ready()) {
        if (g_sleep_deadline_ns != NULL) {
            free(g_sleep_deadline_ns);
            g_sleep_deadline_ns = NULL;
        }
        return;
    }

    for (int32_t i = 0; i < g_process_capacity; ++i) {
        if (g_processes[i].state != PROCESS_STATE_UNUSED) {
            release_process_resources(&g_processes[i]);
            reset_process_slot(&g_processes[i]);
        }
    }

    free(g_processes);
    g_processes = NULL;
    free(g_sleep_deadline_ns);
    g_sleep_deadline_ns = NULL;
    g_process_capacity = 0;
    process_scheduler_init(g_timeslice_ticks);
}

static int32_t find_free_slot(void)
{
    if (!process_table_ready()) {
        return -1;
    }

    /* Reclaim EVERY freeable slot on each call, not just the first one hit.
     * A dead thread/process holds a 128 KiB kernel stack, an 8 MiB eager user
     * stack, its page tables and any committed mmap-arena pages until its slot
     * is reused -- and Chromium churns dozens of threads per attempt, so
     * "free lazily when this exact index is needed" let physical memory leak
     * until demand-zero faults started failing (PMM OOM). A full 256-entry
     * sweep here is cheap and keeps memory bounded to what is actually live. */
    for (int32_t i = 1; i < g_process_capacity; ++i) {
        if (g_processes[i].state == PROCESS_STATE_DEAD &&
            !process_group_on_cpu_locked(i)) {
            release_process_resources(&g_processes[i]);
            reset_process_slot(&g_processes[i]);
        } else if (g_processes[i].state == PROCESS_STATE_ZOMBIE &&
                   !process_group_on_cpu_locked(i)) {
            int32_t pp = g_processes[i].parent_pid;
            if (pp < 0 || !is_valid_pid(pp) ||
                g_processes[pp].state == PROCESS_STATE_UNUSED ||
                g_processes[pp].state == PROCESS_STATE_DEAD) {
                release_process_resources(&g_processes[i]);
                reset_process_slot(&g_processes[i]);
            }
        }
    }

    for (int32_t i = 1; i < g_process_capacity; ++i) {
        if (g_processes[i].state == PROCESS_STATE_UNUSED) {
            return i;
        }
    }
    return -1;
}

/* Write the overflow sentinel to the lowest 16 bytes of a freshly allocated
 * kernel stack. Call once right after kernel_stack_base is assigned. */
static void process_kstack_arm(process_t *proc)
{
    if (proc == NULL || proc->kernel_stack_base == NULL) {
        return;
    }
    volatile uint64_t *bottom = (volatile uint64_t *)(uintptr_t)proc->kernel_stack_base;
    bottom[0] = PROCESS_KSTACK_CANARY;
    bottom[1] = PROCESS_KSTACK_CANARY;
}

/* Panic deterministically if a process's kernel stack has been written past its
 * bottom. Cheap enough (two loads) to run on every context switch; turns what
 * would otherwise be silent heap corruption -> a random later reboot into a
 * clear, immediate diagnosis. */
static void process_kstack_canary_check(const process_t *proc)
{
    if (proc == NULL || proc->kernel_stack_base == NULL) {
        return;
    }
    const volatile uint64_t *bottom =
        (const volatile uint64_t *)(uintptr_t)proc->kernel_stack_base;
    if (bottom[0] != PROCESS_KSTACK_CANARY || bottom[1] != PROCESS_KSTACK_CANARY) {
        extern void kernel_panic(const char *module_name, const char *message);
        serial_write_string("[OS] [KSTACK] kernel stack overflow pid=");
        serial_write_uint64((uint64_t)(uint32_t)(int32_t)(proc - g_processes));
        serial_write_string(" name=");
        serial_write_string(proc->name[0] ? proc->name : "?");
        serial_write_string(" base=");
        serial_write_uint64((uint64_t)(uintptr_t)proc->kernel_stack_base);
        serial_write_string("\n");
        kernel_panic("KSTACK", "kernel stack overflow");
    }
}

#if defined(__x86_64__)
/* ---- Linux i386 segment state (x86_64 only) ---------------------------
 *
 * A 32-bit Linux program finds its thread pointer through a segment: glibc
 * asks set_thread_area() for a GDT descriptor and loads its selector (0x63)
 * into %gs. The descriptor belongs to the thread, the GDT to the CPU, so the
 * thread's three TLS descriptors are copied into the GDT whenever it is
 * switched in, and its FS/GS selectors reloaded -- GS through a SWAPGS pair,
 * because in kernel mode the GS base is the kernel's per-CPU pointer. The
 * selector values themselves are read back whenever the thread leaves the
 * CPU (and on each of its syscalls), since only the program changes them. */
static inline uint16_t x86_read_fs_selector(void)
{
    uint16_t v;
    __asm__ volatile("mov %%fs, %0" : "=r"(v));
    return v;
}

static inline uint16_t x86_read_gs_selector(void)
{
    uint16_t v;
    __asm__ volatile("mov %%gs, %0" : "=r"(v));
    return v;
}

/* 1 (with the segment base) for a selector the thread may have loaded, 0
 * for a null one, -1 for anything that would fault if loaded now. */
static int compat32_selector_base(const process_t *proc, uint16_t sel,
                                  uint64_t *base_out)
{
    *base_out = 0;
    if ((sel & 0xFFFCu) == 0u) {
        return 0;
    }
    if ((sel & 3u) != 3u || (sel & 4u) != 0u) {
        return -1;
    }
    uint16_t idx = (uint16_t)(sel >> 3);
    if (idx == (GDT_USER_DATA >> 3) || idx == (GDT_USER_COMPAT_CODE >> 3)) {
        return 1;
    }
    if (idx >= GDT_TLS_ENTRY_MIN && idx < GDT_TLS_ENTRY_MIN + GDT_TLS_ENTRIES) {
        uint64_t d = proc->compat_tls[idx - GDT_TLS_ENTRY_MIN];
        if ((d & (1ULL << 47)) == 0u ||        /* present */
            ((d >> 45) & 3u) != 3u ||          /* DPL 3 */
            (d & (1ULL << 44)) == 0u) {        /* code/data, not system */
            return -1;
        }
        *base_out = ((d >> 16) & 0xFFFFFFULL) | (((d >> 56) & 0xFFULL) << 24);
        return 1;
    }
    return -1;
}

static void compat32_capture_selectors(process_t *proc)
{
    if (proc != NULL && PROCESS_ABI_IS_COMPAT32(proc->abi_mode)) {
        proc->compat_fs_sel = x86_read_fs_selector();
        proc->compat_gs_sel = x86_read_gs_selector();
    }
}

/* Interrupts are off (every caller holds the process table lock). */
static void compat32_load_segments(process_t *proc)
{
    gdt_set_tls_entries(proc->compat_tls);

    uint64_t base = 0;
    uint16_t fs = proc->compat_fs_sel;
    if (compat32_selector_base(proc, fs, &base) < 0) {
        fs = 0;
        base = 0;
    }
    proc->fs_base = base;
    __asm__ volatile("mov %0, %%fs" :: "r"(fs) : "memory");

    uint16_t gs = proc->compat_gs_sel;
    if (compat32_selector_base(proc, gs, &base) < 0) {
        gs = 0;
        base = 0;
    }
    proc->gs_base = base;
    __asm__ volatile("swapgs\n\t"
                     "mov %0, %%gs\n\t"
                     "swapgs" :: "r"(gs) : "memory");
}
#endif

static void activate_process_context(process_t *proc)
{
    flight_rec(FR_TAG_SWITCH,
               (uint64_t)(uint32_t)(proc != NULL ? (int32_t)(proc - g_processes) : -1),
               proc != NULL ? proc->cr3 : 0u);
    process_kstack_canary_check(proc);
    if (paging_get_active_cr3() != proc->cr3) {
        paging_switch_cr3(proc->cr3);
    }
    syscall_set_kernel_rsp(proc->kernel_stack_top);
    syscall_set_user_rsp(proc->saved_user_rsp);
    gdt_set_kernel_rsp0(proc->kernel_stack_top);

#if defined(__x86_64__)
    const int compat32 = PROCESS_ABI_IS_COMPAT32(proc->abi_mode);
    if (compat32) {
        compat32_load_segments(proc); /* before the base MSRs below */
    }
#endif
    wrmsr_fs_base(proc->fs_base);
    wrmsr_kernel_gs_base(proc->gs_base);
#if defined(__x86_64__)
    /* Always written, so a stale request from a previous task can never be
     * applied to this one. Consumed (cleared) by the exit path. A 32-bit task
     * always leaves through IRETQ into compatibility mode, with its ECX. */
    syscall_set_user_segments(compat32);
    syscall_set_full_restore(proc->full_restore || compat32, proc->full_rcx,
                             proc->full_r11);
    proc->full_restore = 0;
#endif
}

static void mark_process_runnable(process_t *proc, uint64_t entry, int32_t parent_pid)
{
    if (proc == NULL) {
        return;
    }

    uint64_t now_ns = process_perf_now_ns();
    proc->entry = entry;
    proc->parent_pid = parent_pid;
    /* A spawned program is launched, not forked: it leads a process group
     * and a session of its own, the way a program started from a desktop
     * does. */
    proc->pgid = (int32_t)(proc - g_processes);
    proc->sid = proc->pgid;
    /* A spawned program runs as whoever spawned it. */
    if (is_valid_pid(parent_pid)) {
        process_t *parent = process_memory_owner_locked(&g_processes[parent_pid]);
        if (parent != NULL) {
            proc->uid = parent->uid;
            proc->gid = parent->gid;
        }
    }
    proc->timeslice = g_timeslice_ticks;
    proc->state = PROCESS_STATE_READY;
    process_perf_mark_ready_locked(proc, now_ns);
    process_scheduler_request_reschedule();
}

static int initialize_raw_user_stack(process_t *proc)
{
    if (proc == NULL || proc->cr3 == 0 || proc->user_stack_top <= proc->user_stack_base) {
        return -1;
    }

    uint64_t user_rsp = proc->user_stack_top - sizeof(uint64_t);
    uint64_t old_cr3 = paging_get_active_cr3();
    paging_switch_cr3(proc->cr3);
    *(uint64_t *)(uintptr_t)user_rsp = 0;
    paging_switch_cr3(old_cr3);

    proc->saved_user_rsp = user_rsp;
    return 0;
}

#define EXECVE_ARG_MAX 256
/* argv + envp bytes. 8 KiB was not enough for Wine starting wineserver: its
 * environment left no room for the translator path a foreign program's exec
 * adds, and the exec then failed past the point of no return (status 127). */
#define EXECVE_STRTOTAL_MAX 16384

/* Bounds on the environment a launcher may hand a spawn
 * (SYSCALL_PROCESS_SPAWN_ELF_ENV). Smaller than the execve limits on purpose:
 * this is a *merged* buffer -- the launcher's entries plus the kernel's own
 * defaults -- and it lives on the spawning thread's kernel stack, which is
 * also what process_execve's 8 KiB strings_buf sits on. */
#define SPAWN_ENV_MAX 64
#define SPAWN_ENV_STRTOTAL_MAX 4096
/* Room for the launcher's entries plus every default below plus a terminator. */
#define LINUX_ENV_DEFAULT_MAX 24
#define LINUX_ENV_MERGED_MAX (SPAWN_ENV_MAX + LINUX_ENV_DEFAULT_MAX + 2)

/*
 * Foreign-architecture programs (OS_CONFIG_TRANSLATOR_DIR). If `path` is a
 * Linux ELF for the other CPU and its translator is on the image, put the
 * translator's path in `out` and return true; `lib_path` gets the guest's
 * library search path (for LD_LIBRARY_PATH). The translator is then run with
 *     { translator, "-0", argv[0], path, argv[1..] }
 * and loads `path` itself. The process keeps the guest's name and exe path.
 */
#define TRANSLATOR_PATH_MAX 96u

static bool process_foreign_translator(const char *path, char *out,
                                       uint32_t out_size, const char **lib_path)
{
    static const char dir[] = OS_CONFIG_TRANSLATOR_DIR;
    if (dir[0] == '\0' || path == NULL) {
        return false;
    }
    vfs_file_t f;
    memset(&f, 0, sizeof(f));
    if (!vfs_find_file(path, &f)) {
        return false;
    }
    uint8_t h[20];
    bool ok = f.size >= 64u && vfs_read_at(&f, 0u, h, sizeof(h));
    (void)vfs_close_file(&f);
    if (!ok || h[0] != 0x7Fu || h[1] != 'E' || h[2] != 'L' || h[3] != 'F' ||
        h[4] != 2u /* ELFCLASS64 */ || h[5] != 1u /* little-endian */) {
        return false;
    }
    const uint16_t machine = (uint16_t)(h[18] | (h[19] << 8));
#if defined(PLATFORM_ARM64)
    const char *isa = machine == 62u /* EM_X86_64 */ ? "x86_64" : NULL;
    const char *lib = OS_CONFIG_LINUX_RUNTIME_LIB_PATH;
#else
    const char *isa = machine == 183u /* EM_AARCH64 */ ? "aarch64" : NULL;
    const char *lib = OS_CONFIG_LINUX_RUNTIME_LIB_PATH_ARM64;
#endif
    if (isa == NULL) {
        return false;
    }
    size_t dl = strlen(dir), il = strlen(isa);
    if (dl + 1u + il + 5u >= out_size) {
        return false;
    }
    memcpy(out, dir, dl);
    out[dl] = '/';
    memcpy(out + dl + 1u, isa, il);
    memcpy(out + dl + 1u + il, ".ELF", 5u);
    vfs_file_t tf;
    memset(&tf, 0, sizeof(tf));
    if (!vfs_find_file(out, &tf)) {
        return false;
    }
    (void)vfs_close_file(&tf);
    if (lib_path != NULL) {
        *lib_path = lib;
    }
    return true;
}

/* Length of the "NAME" part of a "NAME=VALUE" entry, or (uint32_t)-1 when the
 * entry carries no '=' and so is not a variable at all. */
static uint32_t linux_env_key_length(const char *entry)
{
    uint32_t i = 0;
    while (entry[i] != '\0' && entry[i] != '=') {
        ++i;
    }
    return (entry[i] == '=') ? i : (uint32_t)-1;
}

static int linux_env_same_key(const char *a, const char *b)
{
    uint32_t la = linux_env_key_length(a);
    uint32_t lb = linux_env_key_length(b);
    if (la == (uint32_t)-1 || la != lb) {
        return 0;
    }
    for (uint32_t i = 0; i < la; ++i) {
        if (a[i] != b[i]) {
            return 0;
        }
    }
    return 1;
}

static int count_user_string_array(const char *const *user_array,
                                    uint64_t *count_out,
                                    uint64_t *total_strlen_out)
{
    *count_out = 0;
    *total_strlen_out = 0;
    if (!user_array) return 0;

    for (uint64_t i = 0; i < EXECVE_ARG_MAX; ++i) {
        if (!process_user_buffer_is_valid(&user_array[i], sizeof(const char *)))
            return -1;
        const char *str_ptr = NULL;
        if (copy_from_user(&str_ptr, &user_array[i], sizeof(const char *)) != 0)
            return -1;
        if (!str_ptr) return 0;
        uint64_t len = 0;
        if (process_user_cstring_length(str_ptr, EXECVE_STRTOTAL_MAX, &len) < 0)
            return -1;
        *total_strlen_out += len + 1;
        if (*total_strlen_out > EXECVE_STRTOTAL_MAX) return -1;
        (*count_out)++;
    }
    return -1;
}

static int copy_user_strings(const char *const *user_array,
                              uint64_t count,
                              char *buf, uint64_t buf_size,
                              uint64_t *offsets)
{
    uint64_t pos = 0;
    for (uint64_t i = 0; i < count; ++i) {
        const char *str_ptr = NULL;
        if (copy_from_user(&str_ptr, &user_array[i], sizeof(const char *)) != 0)
            return -1;
        if (!str_ptr) return -1;
        uint64_t len = 0;
        if (process_user_cstring_length(str_ptr, EXECVE_STRTOTAL_MAX, &len) < 0)
            return -1;
        if (pos + len + 1 > buf_size) return -1;
        if (copy_from_user(buf + pos, str_ptr, len + 1) != 0)
            return -1;
        offsets[i] = pos;
        pos += len + 1;
    }
    return 0;
}

static int initialize_elf_user_stack_ex(
    process_t *proc,
    const elf_loaded_image_info_t *image_info,
    const char *exec_path,
    uint64_t argc, const char *const *argv_kernel,
    uint64_t envc, const char *const *envp_kernel);

static int initialize_elf_user_stack(process_t *proc,
                                     const elf_loaded_image_info_t *image_info,
                                     const char *exec_path)
{
    return initialize_elf_user_stack_ex(proc, image_info, exec_path,
                                        0, NULL, 0, NULL);
}

/* The initial stack of a Linux i386 program: the same picture as the 64-bit
 * one below, in 32-bit words -- [argc][argv..][0][envp..][0][auxv..] with
 * %esp at argc. The stack lives in the program's window, which the kernel
 * reaches through the program's own address space. */
static int initialize_elf_user_stack32(
    process_t *proc,
    const elf_loaded_image_info_t *image_info,
    const char *exec_path,
    uint64_t argc, const char *const *argv_kernel,
    uint64_t envc, const char *const *envp_kernel)
{
    uint64_t sp = proc->user_stack_top;
    uint64_t random_seed = image_info->entry ^ proc->cr3 ^ proc->kernel_stack_top;
    size_t exec_path_len = strlen(exec_path) + 1u;
    uint8_t random_bytes[16];
    for (uint32_t i = 0; i < sizeof(random_bytes); ++i) {
        random_seed = (random_seed * 6364136223846793005ULL) + 1442695040888963407ULL;
        random_bytes[i] = (uint8_t)(random_seed >> 56);
    }
    static const char platform[] = "i686";

    uint32_t cpuid_edx = 0;
    hal_cpu_get_id(1u, 0u, NULL, NULL, NULL, &cpuid_edx);
    uint32_t at_clktck = (uint32_t)timer_hz();
    if (at_clktck == 0u) {
        at_clktck = 100u;
    }

    uint64_t total_strings = exec_path_len + sizeof(platform) + 16u + 64u;
    for (uint64_t i = 0; i < argc; ++i) {
        total_strings += strlen(argv_kernel[i]) + 1u;
    }
    for (uint64_t i = 0; i < envc; ++i) {
        total_strings += strlen(envp_kernel[i]) + 1u;
    }
    const uint64_t auxv_pairs = 18u;
    uint64_t words = 1u + argc + 1u + envc + 1u + 2u * auxv_pairs;
    if (total_strings + words * 4u + 64u > PROCESS_INITIAL_USER_STACK_SIZE) {
        return -1; /* everything must fit in the eagerly mapped top */
    }

    uint64_t old_cr3 = paging_get_active_cr3();
    paging_switch_cr3(proc->cr3);

    sp -= exec_path_len;
    memcpy((void *)(uintptr_t)sp, exec_path, exec_path_len);
    uint32_t execfn_addr = (uint32_t)sp;
    sp -= sizeof(platform);
    memcpy((void *)(uintptr_t)sp, platform, sizeof(platform));
    uint32_t platform_addr = (uint32_t)sp;
    sp &= ~0xFULL;
    sp -= sizeof(random_bytes);
    memcpy((void *)(uintptr_t)sp, random_bytes, sizeof(random_bytes));
    uint32_t random_addr = (uint32_t)sp;

    uint32_t envp_addr[LINUX_ENV_MERGED_MAX + EXECVE_ARG_MAX];
    uint32_t argv_addr[EXECVE_ARG_MAX];
    if (argc > EXECVE_ARG_MAX || envc > (uint64_t)(LINUX_ENV_MERGED_MAX + EXECVE_ARG_MAX)) {
        paging_switch_cr3(old_cr3);
        return -1;
    }
    for (uint64_t i = envc; i-- > 0;) {
        size_t len = strlen(envp_kernel[i]) + 1u;
        sp -= len;
        memcpy((void *)(uintptr_t)sp, envp_kernel[i], len);
        envp_addr[i] = (uint32_t)sp;
    }
    for (uint64_t i = argc; i-- > 0;) {
        size_t len = strlen(argv_kernel[i]) + 1u;
        sp -= len;
        memcpy((void *)(uintptr_t)sp, argv_kernel[i], len);
        argv_addr[i] = (uint32_t)sp;
    }

    const uint32_t auxv[2u * 18u] = {
        PROCESS_AT_PHDR,     (uint32_t)image_info->phdr_vaddr,
        PROCESS_AT_PHENT,    (uint32_t)image_info->phent,
        PROCESS_AT_PHNUM,    (uint32_t)image_info->phnum,
        PROCESS_AT_PAGESZ,   (uint32_t)PAGE_SIZE,
        PROCESS_AT_BASE,     (uint32_t)image_info->interp_base,
        PROCESS_AT_ENTRY,    (uint32_t)image_info->main_entry,
        PROCESS_AT_HWCAP,    cpuid_edx,
        PROCESS_AT_HWCAP2,   0u,
        PROCESS_AT_CLKTCK,   at_clktck,
        PROCESS_AT_UID,      proc->uid,
        PROCESS_AT_EUID,     proc->uid,
        PROCESS_AT_GID,      proc->gid,
        PROCESS_AT_EGID,     proc->gid,
        PROCESS_AT_SECURE,   0u,
        PROCESS_AT_RANDOM,   random_addr,
        PROCESS_AT_PLATFORM, platform_addr,
        PROCESS_AT_EXECFN,   execfn_addr,
        PROCESS_AT_NULL,     0u,
    };

    sp -= words * 4u;
    sp &= ~0xFULL; /* %esp 16-aligned at argc, as the i386 kernel leaves it */
    uint32_t *w = (uint32_t *)(uintptr_t)sp;
    uint64_t idx = 0;
    w[idx++] = (uint32_t)argc;
    for (uint64_t i = 0; i < argc; ++i) w[idx++] = argv_addr[i];
    w[idx++] = 0;
    for (uint64_t i = 0; i < envc; ++i) w[idx++] = envp_addr[i];
    w[idx++] = 0;
    for (uint64_t i = 0; i < 2u * auxv_pairs; ++i) w[idx++] = auxv[i];

    paging_switch_cr3(old_cr3);
    proc->saved_user_rsp = sp;
    return 0;
}

static int initialize_elf_user_stack_ex(
    process_t *proc,
    const elf_loaded_image_info_t *image_info,
    const char *exec_path,
    uint64_t argc, const char *const *argv_kernel,
    uint64_t envc, const char *const *envp_kernel)
{
    if (proc == NULL || image_info == NULL || exec_path == NULL || proc->cr3 == 0) {
        return -1;
    }
    if (image_info->abi_mode == PROCESS_ABI_LINUX_I386) {
        return initialize_elf_user_stack32(proc, image_info, exec_path,
                                           argc, argv_kernel, envc, envp_kernel);
    }

    uint64_t old_cr3 = paging_get_active_cr3();
    uint64_t sp = proc->user_stack_top;
    uint64_t random_seed = image_info->entry ^ proc->cr3 ^ proc->kernel_stack_top;
    size_t exec_path_len = 0;
    while (exec_path[exec_path_len] != '\0') {
        ++exec_path_len;
    }
    ++exec_path_len;

    uint8_t random_bytes[16];
    for (uint32_t i = 0; i < sizeof(random_bytes); ++i) {
        random_seed = (random_seed * 6364136223846793005ULL) + 1442695040888963407ULL;
        random_bytes[i] = (uint8_t)(random_seed >> 56);
    }

    /*
     * AT_HWCAP: x86-64 exposes the CPUID leaf 1 EDX feature bits here. glibc's
     * dynamic loader reads it (GLRO(dl_hwcap)); passing the real value keeps
     * IFUNC resolvers and hwcap-based tunables sane. AT_CLKTCK feeds
     * sysconf(_SC_CLK_TCK); without it glibc assumes 100 which would disagree
     * with our timer_hz().
     */
    uint32_t cpuid_edx = 0;
    hal_cpu_get_id(1u, 0u, NULL, NULL, NULL, &cpuid_edx);
    uint64_t at_hwcap = (uint64_t)cpuid_edx;
    if (image_info->abi_mode == PROCESS_ABI_LINUX_ARM64) {
        at_hwcap = PROCESS_ARM64_HWCAP_FP | PROCESS_ARM64_HWCAP_ASIMD;
    }
    uint64_t at_clktck = (uint64_t)timer_hz();
    if (at_clktck == 0u) {
        at_clktck = 100u;
    }

    paging_switch_cr3(proc->cr3);

    sp -= (uint64_t)exec_path_len;
    memcpy((void *)(uintptr_t)sp, exec_path, exec_path_len);
    uint64_t execfn_addr = sp;

    sp &= ~0xFULL;
    sp -= sizeof(random_bytes);
    memcpy((void *)(uintptr_t)sp, random_bytes, sizeof(random_bytes));
    uint64_t random_addr = sp;

    uint64_t total_argv_strlen = 0;
    for (uint64_t i = 0; i < argc; ++i) {
        total_argv_strlen += (argv_kernel ? strlen(argv_kernel[i]) : 0) + 1;
    }
    uint64_t total_envp_strlen = 0;
    for (uint64_t i = 0; i < envc; ++i) {
        total_envp_strlen += (envp_kernel ? strlen(envp_kernel[i]) : 0) + 1;
    }

    sp &= ~0xFULL;
    sp -= total_envp_strlen;
    uint64_t envp_strings_base = sp;
    for (uint64_t i = 0; i < envc; ++i) {
        uint64_t len = strlen(envp_kernel[i]) + 1;
        memcpy((void *)(uintptr_t)sp, envp_kernel[i], len);
        sp += len;
    }
    sp = envp_strings_base;

    sp &= ~0xFULL;
    sp -= total_argv_strlen;
    uint64_t argv_strings_base = sp;
    for (uint64_t i = 0; i < argc; ++i) {
        uint64_t len = strlen(argv_kernel[i]) + 1;
        memcpy((void *)(uintptr_t)sp, argv_kernel[i], len);
        sp += len;
    }
    sp = argv_strings_base;

    process_auxv_t auxv[] = {
        { PROCESS_AT_PHDR,   image_info->phdr_vaddr },
        { PROCESS_AT_PHENT,  image_info->phent },
        { PROCESS_AT_PHNUM,  image_info->phnum },
        { PROCESS_AT_PAGESZ, PAGE_SIZE },
        { PROCESS_AT_BASE,   image_info->interp_base },
        { PROCESS_AT_ENTRY,  image_info->main_entry },
        { PROCESS_AT_HWCAP,  at_hwcap },
        { PROCESS_AT_CLKTCK, at_clktck },
        { PROCESS_AT_UID,    0 },
        { PROCESS_AT_EUID,   0 },
        { PROCESS_AT_GID,    0 },
        { PROCESS_AT_EGID,   0 },
        { PROCESS_AT_SECURE, 0 },
        { PROCESS_AT_RANDOM, random_addr },
        { PROCESS_AT_EXECFN, execfn_addr },
        { PROCESS_AT_NULL,   0 },
    };

    uint64_t auxv_count = sizeof(auxv) / sizeof(auxv[0]);
    uint64_t auxv_words = auxv_count * 2U;

    /* Lay out the initial process stack: [argc][argv..][NULL][envp..][NULL]
     * [auxv..][0], then set the entry %rsp to point at argc.
     *
     * The required entry alignment of %rsp differs by ABI and there is NO
     * single value that works for both:
     *   - Linux ABI (glibc's _start / the ld.so RTLD_START): a real crt stub
     *     that is JUMPed to, so it wants %rsp % 16 == 0 with %rsp pointing
     *     straight at argc. A misaligned %rsp #GPs the dynamic linker on its
     *     first `movaps ...,-0x80(%rbp)`.
     *   - Native ImplusOS apps: _start is a plain `void _start(void)` C
     *     function with no asm crt, so GCC's prologue assumes it was CALLed,
     *     i.e. %rsp % 16 == 8 on entry (a return address had been pushed).
     *
     * So: build the block, align %rsp to 16, and for the native ABI drop it
     * by 8 more. (The historic unconditional `sp -= 8` happened to be right
     * for native and wrong for Linux; making it unconditionally 16-aligned
     * fixed Linux but broke native _start -- e.g. the window manager #GP'd on
     * a `movdqa` off a now-misaligned frame.) */
    uint64_t stack_words_total =
        1ULL + argc + 1ULL + envc + 1ULL + auxv_words + 1ULL;

    sp &= ~0xFULL;
    sp -= stack_words_total * sizeof(uint64_t);
    sp &= ~0xFULL;
    if (!image_info->linux_abi) {
        sp -= 8ULL; /* native C _start expects %rsp % 16 == 8 on entry */
    }

    uint64_t *stack_words = (uint64_t *)(uintptr_t)sp;
    uint64_t idx = 0;

    stack_words[idx++] = argc;

    uint64_t argv_base = argv_strings_base;
    for (uint64_t i = 0; i < argc; ++i) {
        stack_words[idx++] = argv_base;
        argv_base += strlen(argv_kernel[i]) + 1;
    }
    stack_words[idx++] = 0;

    uint64_t envp_base = envp_strings_base;
    for (uint64_t i = 0; i < envc; ++i) {
        stack_words[idx++] = envp_base;
        envp_base += strlen(envp_kernel[i]) + 1;
    }
    stack_words[idx++] = 0;

    for (uint64_t i = 0; i < auxv_count; ++i) {
        stack_words[idx++] = auxv[i].a_type;
        stack_words[idx++] = auxv[i].a_val;
    }
    stack_words[idx] = 0;

    paging_switch_cr3(old_cr3);
    proc->saved_user_rsp = sp;
    return 0;
}


static int initialize_process_memory(process_t *proc,
                                     uint64_t entry,
                                     uint64_t arg1,
                                     uint64_t arg2,
                                     uint64_t arg3,
                                     uint64_t arg4)
{
    if (proc == NULL) {
        return -1;
    }

    /* One kernel stack per process slot, allocated on first use and kept for
     * the life of the boot. Reusing it is what makes teardown safe (nothing is
     * ever handed back to the heap while a CPU might still be standing on it)
     * and it also takes a 128 KiB allocation out of every thread creation. */
    if (proc->kernel_stack_base == NULL) {
        proc->kernel_stack_base = malloc(PROCESS_KERNEL_STACK_SIZE);
        if (!proc->kernel_stack_base) {
            return -1;
        }
        proc->kernel_stack_top =
            ((uint64_t)(uintptr_t)(proc->kernel_stack_base +
                                   PROCESS_KERNEL_STACK_SIZE)) & ~0xFULL;
    }
    process_kstack_arm(proc);

    proc->cr3 = paging_create_process_space();
    if (!proc->cr3) {
        return -1;
    }

    proc->user_code_base = USER_CODE_BASE;
    proc->user_code_limit = USER_CODE_LIMIT;
    proc->user_heap_base = USER_HEAP_BASE;
    proc->user_heap_cursor = USER_HEAP_BASE;
    proc->user_heap_limit = USER_HEAP_LIMIT;
    proc->user_stack_base = USER_STACK_BASE;
    proc->user_stack_top = USER_STACK_TOP;
    proc->user_mmap_cursor = USER_MMAP_BASE;
    proc->capability_mask = PROCESS_CAP_DEFAULT_MASK;

    if (proc->user_code_limit <= proc->user_code_base ||
        proc->user_heap_limit <= proc->user_heap_base ||
        proc->user_stack_top <= proc->user_stack_base ||
        proc->user_code_limit > proc->user_heap_base ||
        proc->user_heap_limit > proc->user_stack_base) {
        return -1;
    }

    if ((proc->user_heap_limit - proc->user_heap_base) <= PROCESS_GUARD_PAGE_SIZE ||
        (proc->user_stack_top - proc->user_stack_base) <= PROCESS_GUARD_PAGE_SIZE) {
        return -1;
    }

    proc->user_heap_guard_page = proc->user_heap_limit - PROCESS_GUARD_PAGE_SIZE;
    proc->user_heap_limit -= PROCESS_GUARD_PAGE_SIZE;
    uint64_t thread_reserve =
        (uint64_t)g_process_capacity * PROCESS_THREAD_STACK_REGION_SIZE;
    if (thread_reserve >= (proc->user_heap_limit - proc->user_heap_base)) {
        return -1;
    }
    proc->user_heap_alloc_limit = proc->user_heap_limit - thread_reserve;

    /* Carve the program break its own window, below anything the bump
     * allocator can reach.
     *
     * brk() used to be served by process_user_alloc() -- the same bump/
     * free-list allocator that backs process_user_mmap(). glibc's malloc
     * treats the break as one contiguous arena it owns outright, so sharing an
     * allocator with mmap() is not survivable: a brk() satisfied out of the
     * free list left user_heap_cursor where it was, and the next dlopen() then
     * mapped a shared object straight over memory malloc had already handed
     * out. It surfaced as function pointers in Xorg's heap reading back as
     * addresses inside libmtdev / modesetting_drv, and as threads returning
     * into string data. See TODO_Doom_Xorg_MethodA.md M20. */
    if (USER_BRK_WINDOW_SIZE >=
        (proc->user_heap_alloc_limit - proc->user_heap_base)) {
        return -1;
    }
    proc->user_brk_limit  = proc->user_heap_alloc_limit;
    proc->user_brk_base   = proc->user_brk_limit - USER_BRK_WINDOW_SIZE;
    proc->user_brk_cursor = proc->user_brk_base;
    proc->user_heap_alloc_limit = proc->user_brk_base;

    proc->user_stack_guard_page = proc->user_stack_base;
    proc->user_stack_base += PROCESS_GUARD_PAGE_SIZE;

    if (proc->user_heap_limit <= proc->user_heap_cursor ||
        proc->user_stack_top <= proc->user_stack_base ||
        proc->user_heap_limit > proc->user_stack_base) {
        return -1;
    }

    uint64_t initial_stack_base = proc->user_stack_top - PROCESS_INITIAL_USER_STACK_SIZE;
    if (initial_stack_base < proc->user_stack_base) {
        return -1;
    }

    if (paging_map_user_range_alloc(proc->cr3,
                                    initial_stack_base,
                                    PROCESS_INITIAL_USER_STACK_SIZE,
                                    PAGE_RW | PAGE_USER) < 0) {
        return -1;
    }

    initialize_fpu_state(proc->fpu_state);

    proc->fs_base = 0;
    proc->gs_base = 0;

    uint64_t *kstack = (uint64_t *)proc->kernel_stack_top;
    kstack -= PROCESS_CONTEXT_QWORDS;

    for (uint32_t i = 0; i < PROCESS_CONTEXT_QWORDS; ++i) {
        kstack[i] = 0;
    }

#if defined(__aarch64__)
    arm64_exception_frame_t *frame = (arm64_exception_frame_t *)kstack;
    frame->x[0] = arg1;
    frame->x[1] = arg2;
    frame->x[2] = arg3;
    frame->x[3] = arg4;
    frame->elr_el1 = entry;
    frame->spsr_el1 = 0;
    proc->saved_rsp = (uint64_t)(uintptr_t)frame;
#else
    kstack[SYSCALL_FRAME_RCX] = entry;
    kstack[SYSCALL_FRAME_RDI] = arg1;
    kstack[SYSCALL_FRAME_RSI] = arg2;
    kstack[SYSCALL_FRAME_RDX] = arg3;
    kstack[SYSCALL_FRAME_R8]  = arg4;
    kstack[SYSCALL_FRAME_R11] = PROCESS_RFLAGS_DEFAULT;
    proc->saved_rsp = (uint64_t)(uintptr_t)kstack;
#endif
    if (initialize_raw_user_stack(proc) < 0) {
        return -1;
    }
#if defined(__aarch64__)
    frame->sp_el0 = proc->saved_user_rsp;
#endif
    proc->timeslice = g_timeslice_ticks;
    
    return 0;
}

void *process_user_mmap(uint64_t length, uint64_t flags)
{
    (void)flags;
    if (!is_valid_pid(current_pid_get()) || length == 0) {
        return NULL;
    }

    uint64_t aligned_len = 0;
    if (align_up_u64_checked(length, PAGE_SIZE, &aligned_len) < 0) {
        return NULL;
    }

    if (aligned_len == 0) {
        return NULL;
    }

    /* Linux i386: from the (reusable) mmap window, committed now -- the
     * 16 MiB bump heap of a 32-bit process would not hold its libraries. */
    if (PROCESS_ABI_IS_COMPAT32(process_get_current_abi_mode())) {
        void *va = process_user_reserve(aligned_len);
        if (va == NULL) {
            return NULL;
        }
        if (paging_map_user_range_alloc(process_get_current_cr3(),
                                        (uint64_t)(uintptr_t)va, aligned_len,
                                        PAGE_RW | PAGE_USER) < 0) {
            process_user_compat_release((uint64_t)(uintptr_t)va, aligned_len);
            return NULL;
        }
        return va;
    }

    return process_user_alloc(aligned_len);
}

/* --- Signal-return trampolines ---------------------------------------------
 *
 * The kernel writes a signal frame whose return address has to reach
 * sigreturn. Every libc on i386 passes its own (SA_RESTORER), glibc on
 * AArch64 does not -- it relies on the kernel's vDSO -- and neither ABI has a
 * vDSO here, so a Linux i386 or AArch64 process gets one read-only executable
 * page holding the trampolines, at USER32_SIGPAGE / USER_ARM64_SIGPAGE.
 * Offsets within the page are part of the frame writers below. */
/* (SIGPAGE_* trampoline offsets: defined with the signal frame writers.) */

static int process_map_sigpage(process_t *proc)
{
    if (proc == NULL || proc->cr3 == 0) {
        return -1;
    }
    uint8_t code[32];
    memset(code, 0, sizeof(code));
    uint64_t va;
    if (PROCESS_ABI_IS_COMPAT32(proc->abi_mode)) {
        static const uint8_t i386_sigreturn[] = {
            0x58, 0xB8, 0x77, 0x00, 0x00, 0x00, 0xCD, 0x80 };
        static const uint8_t i386_rt_sigreturn[] = {
            0xB8, 0xAD, 0x00, 0x00, 0x00, 0xCD, 0x80 };
        memcpy(code + SIGPAGE_I386_SIGRETURN, i386_sigreturn,
               sizeof(i386_sigreturn));
        memcpy(code + SIGPAGE_I386_RT_SIGRETURN, i386_rt_sigreturn,
               sizeof(i386_rt_sigreturn));
        va = USER32_SIGPAGE;
    } else if (proc->abi_mode == PROCESS_ABI_LINUX_ARM64) {
        static const uint32_t arm64_rt_sigreturn[] = {
            0xD2801168u,  /* mov x8, #139 (rt_sigreturn) */
            0xD4000001u,  /* svc #0 */
        };
        memcpy(code + SIGPAGE_ARM64_RT_SIGRETURN, arm64_rt_sigreturn,
               sizeof(arm64_rt_sigreturn));
        va = USER_ARM64_SIGPAGE;
    } else {
        return 0;
    }

    if (paging_map_user_range_alloc(proc->cr3, va, PAGE_SIZE,
                                    PAGE_RW | PAGE_USER) < 0) {
        return -1;
    }
    uint64_t phys = paging_virt_to_phys(proc->cr3, va);
    if (phys == 0u) {
        return -1;
    }
    /* Written through the frame's identity address: the process's address
     * space need not be the active one. */
    memcpy((void *)(uintptr_t)(phys & ~0xFFFULL), code, sizeof(code));
#if defined(__aarch64__)
    __builtin___clear_cache((char *)(uintptr_t)(phys & ~0xFFFULL),
                            (char *)(uintptr_t)(phys & ~0xFFFULL) + sizeof(code));
#endif
    /* Read-only and executable for the program. */
    return paging_protect_user_range(proc->cr3, va, PAGE_SIZE, PAGE_USER) < 0 ? -1 : 0;
}

/* --- Linux i386 address space ----------------------------------------------
 *
 * Applied once the ELF loader has prepared the window (it removed the
 * identity entries and mapped the image and ld.so into it): every other
 * region of the process moves into the window too. See USER32_* in
 * ProcessManager.h for the layout. */
#define COMPAT32_MAP_PAGES ((USER32_MMAP_LIMIT - USER32_MMAP_BASE) / PAGE_SIZE)
#define COMPAT32_MAP_WORDS ((COMPAT32_MAP_PAGES + 63u) / 64u)

static uint64_t *compat32_vm_map_alloc(const uint64_t *copy_from)
{
    uint64_t *map = (uint64_t *)malloc(COMPAT32_MAP_WORDS * sizeof(uint64_t));
    if (map == NULL) {
        return NULL;
    }
    if (copy_from != NULL) {
        memcpy(map, copy_from, COMPAT32_MAP_WORDS * sizeof(uint64_t));
    } else {
        memset(map, 0, COMPAT32_MAP_WORDS * sizeof(uint64_t));
    }
    return map;
}

static void compat32_vm_mark(uint64_t *map, uint64_t first, uint64_t count,
                             int used)
{
    for (uint64_t i = first; i < first + count && i < COMPAT32_MAP_PAGES; ++i) {
        if (used) {
            map[i >> 6] |= 1ULL << (i & 63u);
        } else {
            map[i >> 6] &= ~(1ULL << (i & 63u));
        }
    }
}

/* First fit, a run of `count` free pages; the page index or -1. */
static int64_t compat32_vm_find(const uint64_t *map, uint64_t count)
{
    uint64_t run = 0;
    for (uint64_t i = 0; i < COMPAT32_MAP_PAGES; ++i) {
        if ((i & 63u) == 0u && map[i >> 6] == ~0ULL) {
            run = 0;
            i += 63u;
            continue;
        }
        if ((map[i >> 6] & (1ULL << (i & 63u))) != 0u) {
            run = 0;
            continue;
        }
        if (++run == count) {
            return (int64_t)(i + 1u - count);
        }
    }
    return -1;
}

static int process_apply_compat32_layout(process_t *proc,
                                         const elf_loaded_image_info_t *image)
{
    if (proc == NULL || proc->cr3 == 0 || image == NULL) {
        return -1;
    }

    proc->user_code_base = USER32_WINDOW_BASE;
    proc->user_code_limit = USER32_IMAGE_LIMIT;
    proc->user_heap_base = USER32_HEAP_BASE;
    proc->user_heap_cursor = USER32_HEAP_BASE;
    proc->user_heap_guard_page = USER32_HEAP_LIMIT - PROCESS_GUARD_PAGE_SIZE;
    proc->user_heap_limit = USER32_HEAP_LIMIT - PROCESS_GUARD_PAGE_SIZE;
    proc->user_heap_alloc_limit = proc->user_heap_limit;

    /* The break starts right after the image, as on Linux, and may grow to
     * the end of the image region. */
    uint64_t brk = (image->image_end + PAGE_SIZE - 1u) & ~(PAGE_SIZE - 1u);
    if (brk < USER32_WINDOW_BASE || brk > USER32_IMAGE_LIMIT) {
        brk = USER32_IMAGE_LIMIT;
    }
    proc->user_brk_base = brk;
    proc->user_brk_cursor = brk;
    proc->user_brk_limit = USER32_IMAGE_LIMIT;

    proc->user_stack_guard_page = USER32_STACK_BASE;
    proc->user_stack_base = USER32_STACK_BASE + PROCESS_GUARD_PAGE_SIZE;
    proc->user_stack_top = USER32_STACK_TOP;
    proc->user_mmap_cursor = USER32_MMAP_BASE;
    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        proc->user_allocs[i].used = 0;
        proc->user_allocs[i].addr = 0;
        proc->user_allocs[i].size = 0;
    }

    uint64_t *map = compat32_vm_map_alloc(NULL);
    if (map == NULL) {
        return -1;
    }
    if (proc->compat_vm_map != NULL) {
        free(proc->compat_vm_map);
    }
    proc->compat_vm_map = map;

    memset(proc->compat_tls, 0, sizeof(proc->compat_tls));
    proc->compat_fs_sel = 0;
    proc->compat_gs_sel = 0;
    proc->full_rcx = 0;
    proc->full_r11 = 0;

    /* The 64-bit initial stack the generic setup mapped is of no use to a
     * program that cannot address it; the 32-bit one grows demand-zero below
     * an eagerly mapped top. */
    (void)paging_unmap_range(proc->cr3,
                             USER_STACK_TOP - PROCESS_INITIAL_USER_STACK_SIZE,
                             PROCESS_INITIAL_USER_STACK_SIZE);
    if (paging_map_user_range_alloc(proc->cr3,
                                    USER32_STACK_TOP - PROCESS_INITIAL_USER_STACK_SIZE,
                                    PROCESS_INITIAL_USER_STACK_SIZE,
                                    PAGE_RW | PAGE_USER) < 0) {
        return -1;
    }
    return process_map_sigpage(proc);
}

/* munmap()/mremap() of the i386 mmap window: the address space is free for
 * the next reservation (the caller has dropped the pages). */
void process_user_compat_release(uint64_t addr, uint64_t length)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = is_valid_pid(current_pid_get()) ?
        process_memory_owner_locked(&g_processes[current_pid_get()]) : NULL;
    if (proc != NULL && proc->compat_vm_map != NULL && length != 0u) {
        uint64_t start = addr & ~(PAGE_SIZE - 1u);
        uint64_t end = (addr + length + PAGE_SIZE - 1u) & ~(PAGE_SIZE - 1u);
        if (start < USER32_MMAP_BASE) start = USER32_MMAP_BASE;
        if (end > USER32_MMAP_LIMIT) end = USER32_MMAP_LIMIT;
        if (end > start) {
            compat32_vm_mark(proc->compat_vm_map,
                             (start - USER32_MMAP_BASE) / PAGE_SIZE,
                             (end - start) / PAGE_SIZE, 0);
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

/* MAP_FIXED into the i386 mmap window: those pages are taken. */
void process_user_compat_claim(uint64_t addr, uint64_t length)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = is_valid_pid(current_pid_get()) ?
        process_memory_owner_locked(&g_processes[current_pid_get()]) : NULL;
    if (proc != NULL && proc->compat_vm_map != NULL && length != 0u) {
        uint64_t start = addr & ~(PAGE_SIZE - 1u);
        uint64_t end = (addr + length + PAGE_SIZE - 1u) & ~(PAGE_SIZE - 1u);
        if (start < USER32_MMAP_BASE) start = USER32_MMAP_BASE;
        if (end > USER32_MMAP_LIMIT) end = USER32_MMAP_LIMIT;
        if (end > start) {
            compat32_vm_mark(proc->compat_vm_map,
                             (start - USER32_MMAP_BASE) / PAGE_SIZE,
                             (end - start) / PAGE_SIZE, 1);
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

/* --- USER_MMAP arena: lazily-committed anonymous reservations -------------- */

int process_user_addr_in_mmap_arena(uint64_t addr, uint64_t length)
{
    /* A Linux i386 process: its whole window is demand-zero. */
    if (PROCESS_ABI_IS_COMPAT32(process_get_current_abi_mode())) {
        return addr >= USER32_WINDOW_BASE && addr < USER32_WINDOW_LIMIT &&
               length <= USER32_WINDOW_LIMIT - addr;
    }
    if (length == 0u) {
        return addr >= USER_MMAP_BASE && addr < USER_MMAP_FIXED_LIMIT;
    }
    if (addr < USER_MMAP_BASE) {
        return 0;
    }
    if (addr > (0xFFFFFFFFFFFFFFFFULL - length)) {
        return 0;
    }
    return (addr + length) <= USER_MMAP_FIXED_LIMIT;
}

void *process_user_reserve(uint64_t length)
{
    if (length == 0u) {
        return NULL;
    }

    uint64_t aligned_len = 0;
    if (align_up_u64_checked(length, PAGE_SIZE, &aligned_len) < 0 ||
        aligned_len == 0u) {
        return NULL;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }
    process_t *proc =
        process_memory_owner_locked(&g_processes[current_pid_get()]);
    if (proc == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }

    if (PROCESS_ABI_IS_COMPAT32(proc->abi_mode)) {
        /* Linux i386: page-granular and reusable, inside the window. */
        int64_t first = proc->compat_vm_map != NULL ?
            compat32_vm_find(proc->compat_vm_map, aligned_len / PAGE_SIZE) : -1;
        if (first < 0) {
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            return NULL;
        }
        compat32_vm_mark(proc->compat_vm_map, (uint64_t)first,
                         aligned_len / PAGE_SIZE, 1);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return (void *)(uintptr_t)(USER32_MMAP_BASE + (uint64_t)first * PAGE_SIZE);
    }

    uint64_t cursor = proc->user_mmap_cursor;
    if (cursor < USER_MMAP_BASE || cursor >= USER_MMAP_LIMIT) {
        cursor = USER_MMAP_BASE; /* first use / uninitialised */
    }
    /* 64 KiB granularity keeps things comfortably page-aligned and matches
     * what glibc/PartitionAlloc expect from an mmap() base. */
    cursor = (cursor + 0xFFFFULL) & ~0xFFFFULL;

    if (aligned_len > (USER_MMAP_LIMIT - cursor)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL; /* arena exhausted */
    }

    uint64_t base = cursor;
    proc->user_mmap_cursor = cursor + aligned_len;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    /* Deliberately map nothing: pages fault in demand-zero (RW|USER) via
     * paging_handle_swap_fault() from the #PF handler on first touch. */
    return (void *)(uintptr_t)base;
}

static int range_within(uint64_t addr, uint64_t len, uint64_t start, uint64_t end)
{
    if (len == 0) {
        return 1;
    }
    if (addr > (0xFFFFFFFFFFFFFFFFULL - len)) {
        return 0;
    }
    uint64_t addr_end = addr + len;
    return (addr >= start) && (addr_end <= end);
}

void process_manager_init(void)
{
    int32_t desired_capacity = PROCESS_MAX_COUNT_CONFIG;
    if (desired_capacity < 1) {
        desired_capacity = 1;
    }

    process_scheduler_init(g_timeslice_ticks);

    release_process_table();

    uint64_t table_size_u64 = (uint64_t)desired_capacity * (uint64_t)sizeof(process_t);
    if (table_size_u64 == 0 || table_size_u64 > 0xFFFFFFFFULL) {
        halt_forever();
    }

    g_processes = (process_t *)malloc((size_t)table_size_u64);
    if (g_processes == NULL) {
        halt_forever();
    }
    /* The heap does not zero what it hands out, and reset_process_slot() leaves
     * kernel_stack_base alone on purpose (the stack belongs to the slot). So
     * every slot must start from zero here, or a slot inherits whatever that
     * memory last held. After a warm reset that is the previous boot's own
     * table -- same deterministic heap layout, so a plausible-looking stack
     * pointer -- and initialize_process_memory() skips the allocation and runs
     * the process on memory this boot gave to someone else. That surfaced as
     * "[KSTACK] kernel stack overflow pid=1 name=Userland.ELF" right after any
     * reboot that was not a power-on. */
    memset(g_processes, 0, (size_t)table_size_u64);

    uint64_t sleep_table_size_u64 =
        (uint64_t)desired_capacity * (uint64_t)sizeof(uint64_t);
    if (sleep_table_size_u64 == 0 || sleep_table_size_u64 > 0xFFFFFFFFULL) {
        halt_forever();
    }

    g_sleep_deadline_ns = (uint64_t *)malloc((size_t)sleep_table_size_u64);
    if (g_sleep_deadline_ns == NULL) {
        halt_forever();
    }
    memset(g_sleep_deadline_ns, 0, (size_t)sleep_table_size_u64);

    g_process_capacity = desired_capacity;
    for (int32_t i = 0; i < g_process_capacity; ++i) {
        reset_process_slot(&g_processes[i]);
    }
    
    g_processes[0].state = PROCESS_STATE_RUNNING;
    g_processes[0].parent_pid = -1;
    g_processes[0].memory_owner_pid = 0;
    g_processes[0].capability_mask = PROCESS_CAP_DEFAULT_MASK;
    g_processes[0].last_scheduled_ns = process_perf_now_ns();

    initialize_fpu_state(g_processes[0].fpu_state);
    g_processes[0].fs_base = 0;
    g_processes[0].gs_base = 0;
    g_processes[0].cr3 = paging_get_kernel_cr3();

    current_pid_set(0);
    spinlock_init(&g_process_table_lock);
}

void process_set_current_fs_base(uint64_t fs_base)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (is_valid_pid(current_pid_get())) {
        g_processes[current_pid_get()].fs_base = fs_base;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    wrmsr_fs_base(fs_base);
}

uint64_t process_get_current_fs_base(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    uint64_t val = 0;
    if (is_valid_pid(current_pid_get())) {
        val = g_processes[current_pid_get()].fs_base;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return val;
}

/* GS differs from FS: the syscall/exception entry paths swapgs, so while the
 * kernel runs, the user-visible GS base is parked in IA32_KERNEL_GS_BASE
 * (activate_process_context restores it from proc->gs_base on every switch-in).
 * Writing IA32_GS_BASE here would clobber the kernel's own per-CPU base. */
void process_set_current_gs_base(uint64_t gs_base)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (is_valid_pid(current_pid_get())) {
        g_processes[current_pid_get()].gs_base = gs_base;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    wrmsr_kernel_gs_base(gs_base);
}

uint64_t process_get_current_gs_base(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    uint64_t val = 0;
    if (is_valid_pid(current_pid_get())) {
        val = g_processes[current_pid_get()].gs_base;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return val;
}

/* Lock-free for the same reason as process_get_current_pid(). */
uint8_t process_get_current_abi_mode(void)
{
    int32_t cur = current_pid_get();
    if (!is_valid_pid(cur)) {
        return PROCESS_ABI_IMPLUS;
    }
    return __atomic_load_n(&g_processes[cur].abi_mode, __ATOMIC_RELAXED);
}

void process_set_current_abi_mode(uint8_t mode)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (is_valid_pid(current_pid_get())) {
        g_processes[current_pid_get()].abi_mode = mode;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

uint64_t process_get_current_pending_signals(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    uint64_t pending = 0;
    if (is_valid_pid(current_pid_get())) {
        pending = g_processes[current_pid_get()].pending_signals;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return pending;
}

int process_consume_pending_signal(int32_t signum)
{
    if (signum <= 0 || signum >= PROCESS_SIGNAL_MAX) {
        return 0;
    }
    uint64_t bit = 1u << (uint32_t)signum;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int consumed = 0;
    if (is_valid_pid(current_pid_get())) {
        process_t *proc = &g_processes[current_pid_get()];
        if ((proc->pending_signals & bit) != 0u) {
            proc->pending_signals &= ~bit;
            consumed = 1;
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return consumed;
}

int process_set_current_name(const char *name, uint32_t max_len)
{
    if (name == NULL || max_len == 0u) {
        return -22;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int result = -22;
    if (is_valid_pid(current_pid_get())) {
        process_t *proc = &g_processes[current_pid_get()];
        uint32_t copy_len = max_len;
        if (copy_len >= sizeof(proc->name)) {
            copy_len = sizeof(proc->name) - 1u;
        }
        for (uint32_t i = 0; i < copy_len; ++i) {
            if (name[i] == '\0') {
                copy_len = i;
                break;
            }
        }
        memcpy(proc->name, name, copy_len);
        proc->name[copy_len] = '\0';
        result = 0;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return result;
}

int process_get_current_name(char *out, uint32_t capacity)
{
    if (out == NULL || capacity == 0u) {
        return -22;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int result = -22;
    if (is_valid_pid(current_pid_get())) {
        process_t *proc = &g_processes[current_pid_get()];
        uint32_t len = 0;
        while (len + 1u < capacity && proc->name[len] != '\0') {
            ++len;
        }
        memcpy(out, proc->name, len);
        out[len] = '\0';
        result = 0;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return result;
}

int process_set_current_cwd(const char *cwd)
{
    if (cwd == NULL || cwd[0] == '\0' || cwd[0] != '/') {
        return -22;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int result = -22;
    if (is_valid_pid(current_pid_get())) {
        process_t *proc = &g_processes[current_pid_get()];
        uint32_t len = 0;
        while (cwd[len] != '\0' && len + 1u < sizeof(proc->cwd)) {
            ++len;
        }
        memcpy(proc->cwd, cwd, len);
        proc->cwd[len] = '\0';
        result = 0;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return result;
}

/* chroot(2) for the calling process. The path is stored as given (already
 * resolved against any root already in force by the caller); "/" clears it.
 * Memory-owner scoped, so every thread of a process shares one root, which is
 * what CLONE_FS means and what Chromium's sandbox relies on when it chroots
 * from a helper thread. */
int process_set_current_root(const char *root)
{
    if (root == NULL || root[0] != '/') {
        return -22;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int result = -22;
    if (is_valid_pid(current_pid_get())) {
        process_t *proc =
            process_memory_owner_locked(&g_processes[current_pid_get()]);
        if (proc != NULL) {
            if (root[1] == '\0') {
                proc->root_path[0] = '\0';
            } else {
                uint32_t len = 0;
                while (root[len] != '\0' && len + 1u < sizeof(proc->root_path)) {
                    ++len;
                }
                /* A trailing slash would double up when paths are joined. */
                while (len > 1u && root[len - 1u] == '/') {
                    --len;
                }
                memcpy(proc->root_path, root, len);
                proc->root_path[len] = '\0';
            }
            /* CLONE_FS partner, if any: same root by definition. */
            if (is_valid_pid(proc->fs_share_pid)) {
                process_t *partner =
                    process_memory_owner_locked(&g_processes[proc->fs_share_pid]);
                if (partner != NULL && partner != proc) {
                    memcpy(partner->root_path, proc->root_path,
                           sizeof(partner->root_path));
                }
            }
            result = 0;
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return result;
}

/* Record that `child_pid` was cloned with CLONE_FS from the caller, so a
 * later chroot() by either is seen by both. */
/* The namespace init for `pid`'s process, or -1 (root namespace). Caller
 * holds the table lock. */
static int32_t process_pidns_init_locked(int32_t pid)
{
    if (!is_valid_pid(pid)) {
        return -1;
    }
    process_t *owner = process_memory_owner_locked(&g_processes[pid]);
    return owner != NULL ? owner->pidns_init : -1;
}

/* clone(CLONE_NEWPID): `pid` is the init of a namespace of its own. */
/* Credentials of a process (by any of its thread slots); see process_t.uid. */
int process_get_credentials(int32_t pid, uint32_t *uid, uint32_t *gid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int rc = -1;
    if (is_valid_pid(pid)) {
        process_t *owner = process_memory_owner_locked(&g_processes[pid]);
        if (owner != NULL) {
            if (uid) *uid = owner->uid;
            if (gid) *gid = owner->gid;
            rc = 0;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return rc;
}

/* setuid()/setgid() family: (uint32_t)-1 leaves a value unchanged. */
int process_set_current_credentials(uint32_t uid, uint32_t gid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int rc = -1;
    int32_t pid = current_pid_get();
    if (is_valid_pid(pid)) {
        process_t *owner = process_memory_owner_locked(&g_processes[pid]);
        if (owner != NULL) {
            if (uid != 0xFFFFFFFFu) owner->uid = uid;
            if (gid != 0xFFFFFFFFu) owner->gid = gid;
            rc = 0;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return rc;
}

void process_mark_pidns_init(int32_t pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(pid)) {
        g_processes[pid].pidns_init = pid;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

/* A real pid as the calling process sees it: its namespace's init is 1,
 * everything else keeps its number. */
int32_t process_pid_as_seen_by_current(int32_t real)
{
    if (real <= 0) {
        return real;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t init = process_pidns_init_locked(current_pid_get());
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return (init >= 0 && real == init) ? 1 : real;
}

/* The reverse: a pid the calling process named, as a real pid. */
int32_t process_pid_from_current_view(int32_t seen)
{
    if (seen != 1) {
        return seen;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t init = process_pidns_init_locked(current_pid_get());
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return init >= 0 ? init : seen;
}

/* Is the caller the init of its namespace (getppid() is then 0)? */
int process_current_is_pidns_init(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t pid = current_pid_get();
    int32_t init = process_pidns_init_locked(pid);
    int32_t owner = -1;
    if (is_valid_pid(pid)) {
        process_t *o = process_memory_owner_locked(&g_processes[pid]);
        owner = o ? (int32_t)(o - g_processes) : -1;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return init >= 0 && owner == init;
}

int process_set_fs_share(int32_t child_pid, int32_t partner_pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int result = -1;
    if (is_valid_pid(child_pid) && is_valid_pid(partner_pid)) {
        g_processes[child_pid].fs_share_pid = partner_pid;
        result = 0;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return result;
}

/* The calling process's root, or "" when it is the real one. Returns the
 * length written, or -1. */
int process_get_current_root(char *out, uint32_t capacity)
{
    if (out == NULL || capacity == 0u) {
        return -1;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int result = -1;
    out[0] = '\0';
    if (is_valid_pid(current_pid_get())) {
        process_t *proc =
            process_memory_owner_locked(&g_processes[current_pid_get()]);
        if (proc != NULL) {
            uint32_t len = 0;
            while (proc->root_path[len] != '\0' && len + 1u < capacity) {
                out[len] = proc->root_path[len];
                ++len;
            }
            out[len] = '\0';
            result = (int)len;
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return result;
}

int process_get_current_cwd(char *out, uint32_t capacity)
{
    if (out == NULL || capacity == 0u) {
        return -22;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int result = -22;
    if (is_valid_pid(current_pid_get())) {
        process_t *proc = &g_processes[current_pid_get()];
        uint32_t len = 0;
        while (len + 1u < capacity && proc->cwd[len] != '\0') {
            ++len;
        }
        memcpy(out, proc->cwd, len);
        out[len] = '\0';
        result = 0;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return result;
}

int process_set_clear_child_tid(uint64_t address)
{
    if (address != 0u &&
        !process_user_buffer_is_valid((void *)(uintptr_t)address,
                                      sizeof(uint32_t))) {
        return -14;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    g_processes[current_pid_get()].clear_child_tid = address;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

/* clone(2)'s CLONE_CHILD_CLEARTID names a word in the *child's* TCB that the
 * kernel must zero (and futex-wake) when that child exits -- it is how
 * pthread_join() learns the thread is gone. It is requested by the creating
 * thread, so it cannot be installed with process_set_clear_child_tid(), which
 * always targets the caller: doing that pointed the *creator's* clear-tid at
 * the new thread's TCB, and the creator's own exit then zeroed a live
 * thread's glibc `pd->tid`. glibc's rwlock reads that field to decide whether
 * it is the writer, so the victim thread saw `__cur_writer == tid == 0` and
 * every g_rw_lock_reader_lock() after that failed with EDEADLK -- GObject's
 * type lock stopped working and GTK3 spun re-initialising GSettings forever.
 * Docs/Others/TODO_GTK3_Wayland_LinuxABI.md G5 (W2'). */
int process_set_clear_child_tid_for(int32_t pid, uint64_t address)
{
    if (address != 0u &&
        !process_user_buffer_is_valid((void *)(uintptr_t)address,
                                      sizeof(uint32_t))) {
        return -14;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    g_processes[pid].clear_child_tid = address;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

/* The pid that owns `pid`'s address space -- itself for a process, the
 * creating process for a thread. Threads get their own pid in this kernel,
 * so anything scoped to an address space (shared memory, above all) has to
 * compare this rather than raw pids. */
int32_t process_memory_owner_pid_of(int32_t pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t owner = -1;
    if (is_valid_pid(pid)) {
        owner = g_processes[pid].is_thread &&
                        is_valid_pid(g_processes[pid].memory_owner_pid)
                    ? g_processes[pid].memory_owner_pid
                    : pid;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return owner;
}

int process_set_robust_list(uint64_t head, uint64_t length)
{
    if (length != 24u || head == 0u ||
        !process_user_buffer_is_valid((void *)(uintptr_t)head, length)) {
        return -22;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    process_t *thread = &g_processes[current_pid_get()];
    thread->robust_list_head = head;
    thread->robust_list_length = length;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int process_rseq_register(uint64_t area, uint32_t sig)
{
    if (area == 0u) {
        return -22;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    process_t *proc = &g_processes[current_pid_get()];
    if (proc->rseq_area != 0u) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -16;
    }
    proc->rseq_area = area;
    proc->rseq_sig = sig;
    uint64_t cr3 = proc->cr3;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (cr3 != 0u &&
        paging_is_user_range_mapped(cr3, area, PROCESS_RSEQ_AREA_SIZE)) {
        uint32_t cpu = smp_get_current_cpu_id();
        memcpy((void *)(uintptr_t)(area + PROCESS_RSEQ_CPU_ID_START),
               &cpu, sizeof(cpu));
        memcpy((void *)(uintptr_t)(area + PROCESS_RSEQ_CPU_ID),
               &cpu, sizeof(cpu));
    }
    return 0;
}

int process_rseq_unregister(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    g_processes[current_pid_get()].rseq_area = 0;
    g_processes[current_pid_get()].rseq_sig = 0;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

uint64_t process_get_current_rseq_area(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    uint64_t area = 0;
    if (is_valid_pid(current_pid_get())) {
        area = g_processes[current_pid_get()].rseq_area;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return area;
}

void process_rseq_update_on_preempt_locked(int32_t pid, uint32_t cpu_id)
{
    if (!is_valid_pid(pid)) {
        return;
    }
    process_t *proc = &g_processes[pid];
    if (proc->rseq_area == 0u) {
        return;
    }
    uint64_t area = proc->rseq_area;
    uint64_t cr3 = proc->cr3;
    /* The writes below go through whatever address space is live: if that
     * is not this task's, they would land in another process's memory. */
    if (cr3 == 0u || paging_get_active_cr3() != cr3 ||
        !paging_is_user_range_mapped(cr3, area, PROCESS_RSEQ_AREA_SIZE)) {
        return;
    }

    uint32_t cpu = cpu_id;
    memcpy((void *)(uintptr_t)(area + PROCESS_RSEQ_CPU_ID_START),
           &cpu, sizeof(cpu));
    memcpy((void *)(uintptr_t)(area + PROCESS_RSEQ_CPU_ID),
           &cpu, sizeof(cpu));

    uint64_t rseq_cs = 0;
    memcpy(&rseq_cs, (const void *)(uintptr_t)(area + PROCESS_RSEQ_CS),
           sizeof(rseq_cs));
    if (rseq_cs == 0u) {
        return;
    }
    if (!paging_is_user_range_mapped(cr3, rseq_cs, 8u)) {
        return;
    }

    uint32_t cs_sig = 0;
    uint32_t cs_flags = 0;
    memcpy(&cs_sig, (const void *)(uintptr_t)(rseq_cs + PROCESS_RSEQ_CS_SIG),
           sizeof(cs_sig));
    memcpy(&cs_flags,
           (const void *)(uintptr_t)(rseq_cs + PROCESS_RSEQ_CS_FLAGS),
           sizeof(cs_flags));
    if (cs_sig != proc->rseq_sig) {
        proc->pending_signals |= (1u << 11u);
        return;
    }
    if ((cs_flags & PROCESS_RSEQ_CS_FLAG_NO_RESTART_ON_PREEMPT) != 0u) {
        return;
    }

    uint64_t zero = 0;
    memcpy((void *)(uintptr_t)(area + PROCESS_RSEQ_CS),
           &zero, sizeof(zero));
}

static int32_t process_create_user_internal(uint64_t entry,
                                            uint64_t arg1,
                                            uint64_t arg2,
                                            uint64_t arg3,
                                            uint64_t arg4,
                                            int start_ready)
{
    if (!process_table_ready() || !is_valid_user_entry(entry)) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t pid = find_free_slot();
    if (pid < 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *proc = &g_processes[pid];
    reset_process_slot(proc);

    proc->state = PROCESS_STATE_INIT;
    proc->memory_owner_pid = pid;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (initialize_process_memory(proc, entry, arg1, arg2, arg3, arg4) < 0) {
        irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        release_process_resources(proc);
        reset_process_slot(proc);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    
    int32_t parent_pid = process_get_current_pid();

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (proc->state != PROCESS_STATE_INIT || proc->memory_owner_pid != pid) {
        release_process_resources(proc);
        reset_process_slot(proc);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    proc->entry = entry;
    proc->timeslice = g_timeslice_ticks;
    proc->priority = 1;
    proc->parent_pid = parent_pid;
    proc->pgid = pid;
    proc->sid = pid;
    if (is_valid_pid(parent_pid)) {
        process_t *parent = process_memory_owner_locked(&g_processes[parent_pid]);
        if (parent != NULL) {
            proc->uid = parent->uid;
            proc->gid = parent->gid;
        }
    }
    if (start_ready) {
        proc->state = PROCESS_STATE_READY;
        process_perf_mark_ready_locked(proc, process_perf_now_ns());
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    ipc_init_process_queue(pid);

    return pid;
}

int32_t process_create_user(uint64_t entry)
{
    return process_create_user_internal(entry, 0, 0, 0, 0, 1);
}

int32_t process_create_user_ex(uint64_t entry,
                               uint64_t arg1,
                               uint64_t arg2,
                               uint64_t arg3,
                               uint64_t arg4)
{
    return process_create_user_internal(entry, arg1, arg2, arg3, arg4, 1);
}

int32_t process_create_thread_ex(uint64_t entry,
                                 uint64_t arg1,
                                 uint64_t arg2,
                                 uint64_t arg3,
                                 uint64_t arg4,
                                 int has_tls,
                                 uint64_t tls_fs_base,
                                 uint64_t user_stack)
{
    if (!process_table_ready() || !is_valid_user_entry(entry)) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t current_tid = current_pid_get();
    if (!is_valid_pid(current_tid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *current = &g_processes[current_tid];
    process_t *owner = process_memory_owner_locked(current);
    if (owner == NULL || owner->cr3 == 0 ||
        owner->state == PROCESS_STATE_UNUSED ||
        owner->state == PROCESS_STATE_DEAD ||
        owner->state == PROCESS_STATE_ZOMBIE) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    int32_t tid = find_free_slot();
    if (tid < 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    int32_t owner_pid = (int32_t)(owner - g_processes);
    uint64_t region_top =
        owner->user_heap_limit -
        ((uint64_t)tid * PROCESS_THREAD_STACK_REGION_SIZE);
    uint64_t region_base = region_top - PROCESS_THREAD_STACK_REGION_SIZE;
    if (region_top <= region_base ||
        region_base < owner->user_heap_alloc_limit ||
        region_base < owner->user_heap_cursor) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *thread = &g_processes[tid];
    reset_process_slot(thread);
    thread->state = PROCESS_STATE_INIT;
    thread->is_thread = 1;
    thread->memory_owner_pid = owner_pid;
    thread->parent_pid = owner_pid;
    thread->pgid = owner->pgid;
    thread->sid = owner->sid;
    thread->cr3 = owner->cr3;
    thread->capability_mask = owner->capability_mask;
    thread->user_code_base = owner->user_code_base;
    thread->user_code_limit = owner->user_code_limit;
    thread->user_heap_base = owner->user_heap_base;
    thread->user_heap_cursor = owner->user_heap_cursor;
    thread->user_heap_limit = owner->user_heap_limit;
    thread->user_heap_alloc_limit = owner->user_heap_alloc_limit;
    thread->user_brk_base = owner->user_brk_base;
    thread->user_brk_cursor = owner->user_brk_cursor;
    thread->user_brk_limit = owner->user_brk_limit;
    thread->user_heap_guard_page = owner->user_heap_guard_page;
    thread->user_stack_guard_page = region_base;
    thread->user_stack_base = region_base + PROCESS_GUARD_PAGE_SIZE;
    thread->user_stack_top = region_top;
    thread->user_mmap_cursor = owner->user_mmap_cursor;
    thread->thread_stack_region_base = region_base;
    thread->thread_stack_region_size = PROCESS_THREAD_STACK_REGION_SIZE;
    memcpy(thread->signal_handlers, owner->signal_handlers,
           sizeof(thread->signal_handlers));
    memcpy(thread->signal_flags, owner->signal_flags,
           sizeof(thread->signal_flags));
    memcpy(thread->signal_sa_mask, owner->signal_sa_mask,
           sizeof(thread->signal_sa_mask));
    memcpy(thread->signal_restorer, owner->signal_restorer,
           sizeof(thread->signal_restorer));
    /* sigaltstack is per-thread on real Linux (each thread can register
     * its own); a freshly created thread starts with none configured
     * rather than inheriting the creator's (which would be wrong - two
     * threads must not share one alternate stack). */
    thread->altstack_sp = 0;
    thread->altstack_size = 0;
    thread->altstack_flags = 2u; /* SS_DISABLE */
    thread->signal_mask = current->signal_mask;
    thread->abi_mode = owner->abi_mode;
    thread->priority = owner->priority;
    strncpy(thread->name, owner->name, sizeof(thread->name) - 1);
    thread->name[sizeof(thread->name) - 1] = '\0';

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    thread->kernel_stack_base = malloc(PROCESS_KERNEL_STACK_SIZE);
    if (thread->kernel_stack_base == NULL) {
        goto fail;
    }
    thread->kernel_stack_top =
        ((uint64_t)(uintptr_t)(thread->kernel_stack_base +
                               PROCESS_KERNEL_STACK_SIZE)) & ~0xFULL;
    process_kstack_arm(thread);

    if (paging_map_user_range_alloc(thread->cr3,
                                    thread->user_stack_base,
                                    PROCESS_THREAD_STACK_SIZE,
                                    PAGE_RW | PAGE_USER) < 0) {
        goto fail;
    }

    initialize_fpu_state(thread->fpu_state);
    thread->fs_base = current->fs_base;
    thread->gs_base = current->gs_base;

    uint64_t *kstack = (uint64_t *)thread->kernel_stack_top;
    kstack -= PROCESS_CONTEXT_QWORDS;
    for (uint32_t i = 0; i < PROCESS_CONTEXT_QWORDS; ++i) {
        kstack[i] = 0;
    }

#if defined(__aarch64__)
    arm64_exception_frame_t *frame = (arm64_exception_frame_t *)kstack;
    frame->x[0] = arg1;
    frame->x[1] = arg2;
    frame->x[2] = arg3;
    frame->x[3] = arg4;
    frame->elr_el1 = entry;
    frame->spsr_el1 = 0;
    thread->saved_rsp = (uint64_t)(uintptr_t)frame;
#else
    kstack[SYSCALL_FRAME_RCX] = entry;
    kstack[SYSCALL_FRAME_RDI] = arg1;
    kstack[SYSCALL_FRAME_RSI] = arg2;
    kstack[SYSCALL_FRAME_RDX] = arg3;
    kstack[SYSCALL_FRAME_R8] = arg4;
    kstack[SYSCALL_FRAME_R11] = PROCESS_RFLAGS_DEFAULT;
    thread->saved_rsp = (uint64_t)(uintptr_t)kstack;
#endif

    if (user_stack != 0u) {
        /* Linux clone(2): glibc's __clone has already laid out the child's
         * fn/arg on this stack. Adopt it verbatim - set before the thread is
         * READY so an SMP peer can't dispatch it against a stale raw stack
         * (that race made the child jump through a zero fn pointer -> #PF at
         * RIP=0). Do NOT run initialize_raw_user_stack(), which would both
         * pick a different stack and scribble a 0 into it. */
        thread->saved_user_rsp = user_stack;
    } else if (initialize_raw_user_stack(thread) < 0) {
        goto fail;
    }
#if defined(__aarch64__)
    frame->sp_el0 = thread->saved_user_rsp;
#endif

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (thread->state != PROCESS_STATE_INIT ||
        thread->memory_owner_pid != owner_pid) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        goto fail;
    }
    thread->entry = entry;
    thread->timeslice = g_timeslice_ticks;
    if (has_tls) {
        /* Must happen before PROCESS_STATE_READY is visible, still under
         * g_process_table_lock: once READY, another CPU's scheduler can
         * pick this thread up immediately (SMP is enabled), so setting
         * fs_base afterwards would race against the thread's own first
         * instructions observing the wrong TLS base. */
        thread->fs_base = tls_fs_base;

    }
    thread->state = PROCESS_STATE_READY;
    process_perf_mark_ready_locked(thread, process_perf_now_ns());
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return tid;

fail:
    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    release_thread_resources(thread, 1);
    reset_process_slot(thread);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return -1;
}

int32_t process_create_thread(uint64_t entry,
                              uint64_t arg1,
                              uint64_t arg2,
                              uint64_t arg3,
                              uint64_t arg4)
{
    return process_create_thread_ex(entry, arg1, arg2, arg3, arg4, 0, 0u, 0u);
}

/* clone(CLONE_THREAD) for the Linux ABIs that need the child to come back
 * with the caller's registers -- all of them, as Linux does: AArch64 glibc's
 * clone3 keeps the thread function and its argument in x10/x11 across the
 * svc, i386 glibc pops them off the new stack after `int $0x80`. The child
 * starts from a copy of the caller's syscall frame with the result register
 * zeroed and its stack pointer at `user_sp`, and it gets no kernel-chosen
 * stack region: a Linux thread always runs on the stack its library mapped.
 *
 * TLS: `has_tls` sets the AArch64 thread pointer (TPIDR_EL0) to `tls_value`;
 * for an i386 thread `compat_tls` (if not NULL) replaces the three GDT TLS
 * descriptors it would otherwise inherit from the caller. */
int32_t process_create_thread_clone(uint64_t parent_frame, uint64_t user_sp,
                                    int has_tls, uint64_t tls_value,
                                    const uint64_t *compat_tls)
{
    if (!process_table_ready() || parent_frame == 0u || user_sp == 0u) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t current_tid = current_pid_get();
    if (!is_valid_pid(current_tid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    process_t *current = &g_processes[current_tid];
    process_t *owner = process_memory_owner_locked(current);
    if (owner == NULL || owner->cr3 == 0 ||
        owner->state == PROCESS_STATE_UNUSED ||
        owner->state == PROCESS_STATE_DEAD ||
        owner->state == PROCESS_STATE_ZOMBIE) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    int32_t tid = find_free_slot();
    if (tid < 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    int32_t owner_pid = (int32_t)(owner - g_processes);
    process_t *thread = &g_processes[tid];
    reset_process_slot(thread);
    thread->state = PROCESS_STATE_INIT;
    thread->is_thread = 1;
    thread->memory_owner_pid = owner_pid;
    thread->parent_pid = owner_pid;
    thread->pgid = owner->pgid;
    thread->sid = owner->sid;
    thread->uid = owner->uid;
    thread->gid = owner->gid;
    thread->cr3 = owner->cr3;
    thread->capability_mask = owner->capability_mask;
    thread->user_code_base = owner->user_code_base;
    thread->user_code_limit = owner->user_code_limit;
    thread->user_heap_base = owner->user_heap_base;
    thread->user_heap_cursor = owner->user_heap_cursor;
    thread->user_heap_limit = owner->user_heap_limit;
    thread->user_heap_alloc_limit = owner->user_heap_alloc_limit;
    thread->user_brk_base = owner->user_brk_base;
    thread->user_brk_cursor = owner->user_brk_cursor;
    thread->user_brk_limit = owner->user_brk_limit;
    thread->user_heap_guard_page = owner->user_heap_guard_page;
    thread->user_stack_base = owner->user_stack_base;
    thread->user_stack_top = owner->user_stack_top;
    thread->user_stack_guard_page = owner->user_stack_guard_page;
    thread->user_mmap_cursor = owner->user_mmap_cursor;
    thread->thread_stack_region_base = 0;
    thread->thread_stack_region_size = 0;
    memcpy(thread->signal_handlers, owner->signal_handlers,
           sizeof(thread->signal_handlers));
    memcpy(thread->signal_flags, owner->signal_flags,
           sizeof(thread->signal_flags));
    memcpy(thread->signal_sa_mask, owner->signal_sa_mask,
           sizeof(thread->signal_sa_mask));
    memcpy(thread->signal_restorer, owner->signal_restorer,
           sizeof(thread->signal_restorer));
    thread->altstack_sp = 0;
    thread->altstack_size = 0;
    thread->altstack_flags = 2u; /* SS_DISABLE */
    thread->signal_mask = current->signal_mask;
    thread->abi_mode = owner->abi_mode;
    thread->priority = owner->priority;
    memcpy(thread->compat_tls, compat_tls != NULL ? compat_tls : current->compat_tls,
           sizeof(thread->compat_tls));
    thread->compat_fs_sel = current->compat_fs_sel;
    thread->compat_gs_sel = current->compat_gs_sel;
    thread->full_restore = 1;
    thread->full_rcx = current->full_rcx;
    thread->full_r11 = current->full_r11;
    strncpy(thread->name, owner->name, sizeof(thread->name) - 1);
    thread->name[sizeof(thread->name) - 1] = '\0';
    memcpy(thread->cwd, owner->cwd, sizeof(thread->cwd));
    memcpy(thread->root_path, owner->root_path, sizeof(thread->root_path));
    memcpy(thread->exe_path, owner->exe_path, sizeof(thread->exe_path));

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (thread->kernel_stack_base == NULL) {
        thread->kernel_stack_base = malloc(PROCESS_KERNEL_STACK_SIZE);
        if (thread->kernel_stack_base == NULL) {
            goto fail;
        }
    }
    thread->kernel_stack_top =
        ((uint64_t)(uintptr_t)(thread->kernel_stack_base +
                               PROCESS_KERNEL_STACK_SIZE)) & ~0xFULL;
    process_kstack_arm(thread);

    initialize_fpu_state(thread->fpu_state);
#if defined(__aarch64__)
    thread->fs_base = has_tls ? tls_value : rdmsr_fs_base();
#else
    (void)has_tls;
    (void)tls_value;
    thread->fs_base = current->fs_base;
    thread->gs_base = current->gs_base;
#endif

    uint64_t *kstack = (uint64_t *)thread->kernel_stack_top;
    kstack -= PROCESS_CONTEXT_QWORDS;
    const uint64_t *src = (const uint64_t *)(uintptr_t)parent_frame;
    for (uint32_t i = 0; i < PROCESS_CONTEXT_QWORDS; ++i) {
        kstack[i] = src[i];
    }
    uint64_t return_pc;
#if defined(__aarch64__)
    arm64_exception_frame_t *frame = (arm64_exception_frame_t *)kstack;
    frame->x[0] = 0;
    frame->sp_el0 = user_sp;
    return_pc = frame->elr_el1;
#else
    kstack[SYSCALL_FRAME_RAX] = 0;
    return_pc = kstack[SYSCALL_FRAME_RCX];
#endif
    thread->saved_rsp = (uint64_t)(uintptr_t)kstack;
    thread->saved_user_rsp = user_sp;

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (thread->state != PROCESS_STATE_INIT ||
        thread->memory_owner_pid != owner_pid) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        goto fail;
    }
    thread->entry = return_pc;
    thread->timeslice = g_timeslice_ticks;
    thread->state = PROCESS_STATE_READY;
    process_perf_mark_ready_locked(thread, process_perf_now_ns());
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    process_scheduler_request_reschedule();
    return tid;

fail:
    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    release_thread_resources(thread, 0);
    reset_process_slot(thread);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return -1;
}

/* An `int $0x80` from the running Linux i386 thread: its ECX (a preserved
 * register in that ABI) is what it must get back, and its FS/GS selectors are
 * whatever it last loaded. */
void process_compat32_note_syscall_entry(uint64_t ecx)
{
#if defined(__x86_64__)
    uint64_t irq_flags = irq_save_disable();
    int32_t cur = current_pid_get();
    if (is_valid_pid(cur)) {
        process_t *proc = &g_processes[cur];
        proc->full_restore = 1;
        proc->full_rcx = (uint32_t)ecx;
        proc->full_r11 = 0;
        compat32_capture_selectors(proc);
    }
    irq_restore(irq_flags);
#else
    (void)ecx;
#endif
}

/* set_thread_area()/get_thread_area() storage of the running thread. */
int process_compat32_get_tls(uint64_t out[3])
{
    uint64_t irq_flags = irq_save_disable();
    int32_t cur = current_pid_get();
    int rc = -1;
    if (is_valid_pid(cur)) {
        memcpy(out, g_processes[cur].compat_tls, sizeof(g_processes[cur].compat_tls));
        rc = 0;
    }
    irq_restore(irq_flags);
    return rc;
}

int process_compat32_set_tls(const uint64_t desc[3])
{
    uint64_t irq_flags = irq_save_disable();
    int32_t cur = current_pid_get();
    int rc = -1;
    if (is_valid_pid(cur)) {
        memcpy(g_processes[cur].compat_tls, desc, sizeof(g_processes[cur].compat_tls));
#if defined(__x86_64__)
        /* Into this CPU's GDT right away as well: the exit path reloads the
         * selectors from it before the program's next instruction. */
        gdt_set_tls_entries(g_processes[cur].compat_tls);
#endif
        rc = 0;
    }
    irq_restore(irq_flags);
    return rc;
}

int32_t process_spawn_user_elf_with_arg(const char *path,
                                        const char *launch_argument)
{
    return process_spawn_user_elf_with_env(path, launch_argument, NULL);
}

/* Spawn `path`, giving the child the environment the launcher supplied in
 * `user_envp` (a NULL-terminated array of "NAME=VALUE" pointers in the
 * caller's address space, or NULL for the kernel's defaults alone).
 *
 * The kernel never assembles an application's environment any more: it only
 * appends its own generic defaults for keys the launcher did not name. See
 * the default table further down and Userland/API/Source/LinuxEnv.c. */
int32_t process_spawn_user_elf_with_env(const char *path,
                                        const char *launch_argument,
                                        const char *const *user_envp)
{
    /* Copied out of the caller's address space before anything else happens:
     * initialize_elf_user_stack_ex() switches to the child's CR3 and only
     * then reads these strings, by which time the caller's mappings are gone.
     * Same technique, smaller budget, as process_execve's strings_buf. */
    char env_strings[SPAWN_ENV_STRTOTAL_MAX];
    uint64_t env_offsets[SPAWN_ENV_MAX];
    const char *env_ptrs[SPAWN_ENV_MAX];
    uint64_t user_envc = 0u;
    memset(env_strings, 0, sizeof(env_strings));
    if (user_envp != NULL) {
        uint64_t env_strtotal = 0u;
        if (count_user_string_array(user_envp, &user_envc, &env_strtotal) < 0 ||
            user_envc > (uint64_t)SPAWN_ENV_MAX ||
            env_strtotal > (uint64_t)SPAWN_ENV_STRTOTAL_MAX) {
            return -1;
        }
        if (user_envc > 0u &&
            copy_user_strings(user_envp, user_envc, env_strings,
                              sizeof(env_strings), env_offsets) < 0) {
            return -1;
        }
        for (uint64_t i = 0; i < user_envc; ++i) {
            env_ptrs[i] = env_strings + env_offsets[i];
        }
    }

    if (!path || path[0] == '\0') {
        return -1;
    }
    int32_t pid = process_create_user_internal(USER_CODE_BASE, 0, 0, 0, 0, 0);
    if (pid < 0) {
        return -1;
    }

    process_t *proc = &g_processes[pid];

    const char *name_ptr = path;
    for (const char *p = path; *p; ++p) {
        if (*p == '/' && *(p+1) != '\0') name_ptr = p + 1;
    }
    strncpy(proc->name, name_ptr, sizeof(proc->name) - 1);
    proc->name[sizeof(proc->name) - 1] = '\0';
    strncpy(proc->exe_path, path, sizeof(proc->exe_path) - 1u);
    proc->exe_path[sizeof(proc->exe_path) - 1u] = '\0';

    /* Start the child with cwd = directory component of the executable path.
     * Foreign Linux binaries (Method A: Doom's linuxxdoom) locate sibling data
     * relative to ".", e.g. IdentifyVersion() -> access("./doom1.wad"), and we
     * have no way to pass DOOMWADDIR through process_spawn. Falls back to "/"
     * when path has no directory part. */
    {
        uint32_t slash = 0u;
        for (uint32_t i = 0u; path[i] != '\0' && i < sizeof(proc->cwd) - 1u; ++i) {
            if (path[i] == '/') {
                slash = i;
            }
        }
        if (slash == 0u) {
            proc->cwd[0] = '/';
            proc->cwd[1] = '\0';
        } else {
            memcpy(proc->cwd, path, slash);
            proc->cwd[slash] = '\0';
        }
    }

    if (launch_argument) {
        strncpy(proc->launch_argument, launch_argument,
                sizeof(proc->launch_argument) - 1u);
        proc->launch_argument[sizeof(proc->launch_argument) - 1u] = '\0';
    }

    elf_load_policy_t policy = {
        .max_file_size = PROCESS_ELF_MAX_SIZE,
        .min_vaddr = USER_FOREIGN_BASE,
        .max_vaddr = USER_CODE_LIMIT,
    };
    elf_loaded_image_info_t image_info = {0};

    /* A program for the other CPU runs under its translator (see
     * process_foreign_translator); the process keeps the program's name,
     * exe path and working directory set above. */
    char tpath[TRANSLATOR_PATH_MAX];
    const bool translated = process_foreign_translator(path, tpath, sizeof(tpath), NULL);
    const char *load_path = translated ? tpath : path;

    if (!elf_loader_load_from_path(proc->cr3, pid, load_path, &policy, &image_info)) {
        filemap_release_pid(pid);
        ipc_cleanup_process_queue(pid);
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        release_process_resources(proc);
        reset_process_slot(proc);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    proc->abi_mode = image_info.abi_mode;
    proc->main_phdr_vaddr = image_info.phdr_vaddr;
    proc->main_phent = image_info.phent;
    proc->main_phnum = image_info.phnum;
    if ((PROCESS_ABI_IS_COMPAT32(proc->abi_mode) &&
         process_apply_compat32_layout(proc, &image_info) < 0) ||
        (proc->abi_mode == PROCESS_ABI_LINUX_ARM64 &&
         process_map_sigpage(proc) < 0)) {
        filemap_release_pid(pid);
        ipc_cleanup_process_queue(pid);
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        release_process_resources(proc);
        reset_process_slot(proc);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    foreign_launch_log_start((int32_t)(proc - g_processes),
                             translated ? "spawn (translated)" : "spawn", name_ptr,
                             launch_argument, image_info.linux_abi);
    if (PROCESS_ABI_IS_LINUX(proc->abi_mode)) {
        /* Split launch_argument on whitespace into argv[1..]. A caller that
         * needs to pass several flags to a Linux binary (e.g. Chromium's
         * "--single-process --no-sandbox ...") packs them into the one launch
         * argument string; a value with no spaces still arrives as a single
         * argv[1], matching the old behaviour. */
        #define LINUX_ARGV_MAX 32u
        const char *linux_argv[LINUX_ARGV_MAX];
        char argsplit[sizeof(proc->launch_argument)];
        linux_argv[0] = name_ptr;
        uint64_t linux_argc = 1;
        if (translated) {
            linux_argv[0] = tpath;
            linux_argv[1] = "-0";
            linux_argv[2] = name_ptr;
            linux_argv[3] = path;
            linux_argc = 4;
        }
        if (launch_argument != NULL && launch_argument[0] != '\0') {
            strncpy(argsplit, launch_argument, sizeof(argsplit) - 1u);
            argsplit[sizeof(argsplit) - 1u] = '\0';
            char *s = argsplit;
            while (*s != '\0' && linux_argc < LINUX_ARGV_MAX) {
                while (*s == ' ' || *s == '\t' || *s == '\n') {
                    *s++ = '\0';
                }
                if (*s == '\0') {
                    break;
                }
                linux_argv[linux_argc++] = s;
                while (*s != '\0' && *s != ' ' && *s != '\t' && *s != '\n') {
                    ++s;
                }
            }
        }
        /* Default environment for a Linux-ABI process, read by whichever
           interpreter PT_INTERP named (a static binary ignores it).

           ONLY WHAT IS NOT AN APPLICATION'S BUSINESS IS LISTED HERE. Every
           entry below is either POSIX-generic -- identity, login shell,
           locale, timezone, terminal type -- or one of the XDG base
           directories that glibc's own GUI stacks (GTK, GLib/GIO) and
           Chromium all assume. The single platform-specific entry is the
           library search path, which only the kernel is in a position to
           know: it is what the staged Linux runtime's layout requires, the
           moral equivalent of the rpath a distro's ld.so is built with, and
           it comes from OS_CONFIG_LINUX_RUNTIME_LIB_PATH.

           Anything naming an application, a toolkit or a service -- DISPLAY,
           LD_PRELOAD, DOOMWADDIR, GDK_*, FONTCONFIG_*, ALSOFT_DRIVERS, and
           the in-tree test ld.so's own LD_LIBRARY_PATH -- is assembled by
           the launcher in Userland/API/Source/LinuxEnv.c and handed over
           through SYSCALL_PROCESS_SPAWN_ELF_ENV. A launcher entry and a
           default naming the same key are never both emitted: the default
           is dropped, so the child sees exactly one value regardless of
           whether getenv() or glibc's ld.so reads it first- or last-match.

           History of each line used to live here; it moved with the entries
           it describes (Docs/Others/TODO_Doom_Xorg_MethodA.md,
           TODO_GTK3_Wayland_LinuxABI.md, TODO_Terminal_xterm.md,
           TODO_glibc_Port.md). */
        static const char *linux_envp_defaults[] = {
            "PATH=/bin:/usr/bin",
            "HOME=/tmp",
            "USER=root",
            "LOGNAME=root",
            "SHELL=/bin/sh",
            "LANG=C.UTF-8",
            "LC_ALL=C.UTF-8",
            "TZ=UTC",
            "TERM=xterm",
            "XDG_RUNTIME_DIR=/tmp",
            "XDG_CACHE_HOME=/tmp/.cache",
            "XDG_CONFIG_HOME=/tmp/.config",
            "XDG_DATA_DIRS=/usr/share",
            "XDG_CONFIG_DIRS=/etc/xdg",
            NULL,
        };
        static const char linux_envp_ld_path_x86_64[] =
            "LD_LIBRARY_PATH=" OS_CONFIG_LINUX_RUNTIME_LIB_PATH;
        static const char linux_envp_ld_path_i386[] =
            "LD_LIBRARY_PATH=" OS_CONFIG_LINUX_RUNTIME_LIB_PATH_I386;
        static const char linux_envp_ld_path_arm64[] =
            "LD_LIBRARY_PATH=" OS_CONFIG_LINUX_RUNTIME_LIB_PATH_ARM64;
        const char *linux_envp_ld_path =
            proc->abi_mode == PROCESS_ABI_LINUX_I386 ? linux_envp_ld_path_i386 :
            proc->abi_mode == PROCESS_ABI_LINUX_ARM64 ? linux_envp_ld_path_arm64 :
                                                        linux_envp_ld_path_x86_64;

        /* Only glibc's loader (/lib64/ld-linux-x86-64.so.2, used by
           external Linux binaries) needs the runtime's library directory,
           and only the kernel can tell it apart from any other PT_INTERP --
           a launcher never sees PT_INTERP (Docs/Others/TODO_glibc_Port.md
           G3). */
        const char *family_ld_path = NULL;
        if (translated) {
            /* the translated program's own runtime, not the translator's */
#if defined(PLATFORM_ARM64)
            family_ld_path = linux_envp_ld_path_x86_64;
#else
            family_ld_path = linux_envp_ld_path_arm64;
#endif
        } else {
            const char *ip = image_info.interp_path;
            for (uint32_t i = 0; i + 2 < sizeof(image_info.interp_path) &&
                                 ip[i] != '\0'; ++i) {
                if (ip[i] == 'l' && ip[i + 1] == 'd' && ip[i + 2] == '-') {
                    family_ld_path = linux_envp_ld_path; /* ".../ld-linux-..." */
                    break;
                }
            }
        }

        /* Launcher entries go in first: getenv() returns the first match, so
           this is also how a launcher overrides a default below. */
        const char *merged_env[LINUX_ENV_MERGED_MAX];
        uint64_t merged_envc = 0u;
        for (uint64_t i = 0; i < user_envc &&
                            merged_envc < (uint64_t)LINUX_ENV_MERGED_MAX; ++i) {
            merged_env[merged_envc++] = env_ptrs[i];
        }

        int ld_path_given = 0;
        for (uint64_t u = 0; u < user_envc; ++u) {
            if (family_ld_path != NULL &&
                linux_env_same_key(family_ld_path, env_ptrs[u])) {
                ld_path_given = 1;
            }
        }

        for (uint64_t d = 0; linux_envp_defaults[d] != NULL &&
                            merged_envc < (uint64_t)LINUX_ENV_MERGED_MAX; ++d) {
            const char *candidate = linux_envp_defaults[d];
            for (uint64_t u = 0; u < user_envc; ++u) {
                if (linux_env_same_key(candidate, env_ptrs[u])) {
                    candidate = NULL;
                    break;
                }
            }
            if (candidate != NULL) {
                merged_env[merged_envc++] = candidate;
            }
        }
        if (family_ld_path != NULL && !ld_path_given &&
            merged_envc < (uint64_t)LINUX_ENV_MERGED_MAX) {
            merged_env[merged_envc++] = family_ld_path;
        }
        if (initialize_elf_user_stack_ex(proc, &image_info, path,
                                         linux_argc, linux_argv,
                                         merged_envc, merged_env) < 0) {
            filemap_release_pid(pid);
            ipc_cleanup_process_queue(pid);
            uint64_t irq_flags = irq_save_disable();
            spinlock_lock(&g_process_table_lock);
            release_process_resources(proc);
            reset_process_slot(proc);
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            return -1;
        }
    } else if (initialize_elf_user_stack(proc, &image_info, path) < 0) {
        filemap_release_pid(pid);
        ipc_cleanup_process_queue(pid);
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        release_process_resources(proc);
        reset_process_slot(proc);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    
    uint64_t irq_flags = irq_save_disable();

#if defined(__aarch64__)
    arm64_exception_frame_t *frame =
        (arm64_exception_frame_t *)(uintptr_t)proc->saved_rsp;
    frame->elr_el1 = image_info.entry;
    frame->sp_el0 = proc->saved_user_rsp;
#else
    uint64_t *kstack = (uint64_t *)(uintptr_t)proc->saved_rsp;
    kstack[SYSCALL_FRAME_RCX] = image_info.entry;
#endif

    spinlock_lock(&g_process_table_lock);
    mark_process_runnable(proc, image_info.entry, proc->parent_pid);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    return pid;
}

int32_t process_spawn_user_elf(const char *path)
{
    return process_spawn_user_elf_with_arg(path, NULL);
}

static int process_copy_user_page(uint64_t child_cr3, uint64_t vaddr,
                                   uint64_t parent_phys)
{
    void *child_page = pmm_alloc_pages(1);
    if (!child_page) return -1;
    memcpy(child_page, (void *)(uintptr_t)parent_phys, PAGE_SIZE);
    int ret = paging_map_user_page(child_cr3, vaddr,
                                    (uint64_t)(uintptr_t)child_page,
                                    PAGE_PRESENT | PAGE_RW | PAGE_USER);
    if (ret < 0) {
        pmm_free_pages(child_page, 1);
    }
    return ret;
}

static int process_clone_address_space(process_t *child, process_t *parent)
{
    int clone_rc;

    /* A Linux i386 child's window has to be a hole before the parent's pages
     * are shared into it, like the parent's was at exec. */
    if (PROCESS_ABI_IS_COMPAT32(parent->abi_mode) &&
        paging_prepare_compat32_window(child->cr3) < 0) {
        return -1;
    }

    if (PROCESS_ABI_IS_LINUX(parent->abi_mode)) {
#if KERNEL_COW_FORK
        /* Copy-on-write: share the parent's pages read-only into the child
         * and fault-in private copies on first write. Falls back to the
         * eager copy if the COW clone reports failure.
         *
         * The ranges must be the same ones the eager copy below walks. They
         * were not: this used to stop at USER_STACK_BASE, so the child got
         * the image and the heap but no *stack* and no mmap arena, and
         * faulted on its first instruction back in user space. That is the
         * "COW makes boot unstable" this was turned off for. */
        if (paging_cow_clone_user_range(child->cr3, parent->cr3,
                                        USER_FOREIGN_BASE,
                                        USER_STACK_TOP) == 0 &&
            paging_cow_clone_user_range(child->cr3, parent->cr3,
                                        USER_MMAP_BASE, USER_MMAP_FIXED_LIMIT) == 0) {
            clone_rc = 0;
            goto done;
        }
#endif
#if defined(__aarch64__)
        /* arm64 Linux images are never below the user windows (the ELF
         * loader refuses them); everything lower is the kernel's. */
        clone_rc = paging_copy_present_user_range(child->cr3, parent->cr3,
                                                  USER_CODE_BASE,
                                                  USER_STACK_TOP);
#else
        clone_rc = paging_copy_present_user_range(child->cr3, parent->cr3,
                                                  USER_FOREIGN_BASE,
                                                  USER_STACK_TOP);
#endif
        if (clone_rc == 0) {
            /* The USER_MMAP arena sits far above the stack, so it needs its
             * own pass. Shared objects live there now that large file
             * mappings are demand-paged, and the child would otherwise start
             * with its libraries' writable data absent. The walk skips absent
             * page-table levels, so covering the (mostly empty) arena is
             * cheap, and the demand-paged pages that have never been touched
             * cost nothing here -- FileMap re-registers those separately. */
            clone_rc = paging_copy_present_user_range(child->cr3, parent->cr3,
                                                      USER_MMAP_BASE,
                                                      USER_MMAP_FIXED_LIMIT);
        }
        goto done;
    }

    clone_rc = 0;
    uint64_t parent_cr3 = parent->cr3;
    uint64_t child_cr3 = child->cr3;

    for (uint64_t vaddr = USER_CODE_BASE; vaddr < USER_CODE_LIMIT;
         vaddr += PAGE_SIZE) {
        uint64_t phys = paging_virt_to_phys(parent_cr3, vaddr);
        if (phys == 0) continue;
        if (process_copy_user_page(child_cr3, vaddr, phys) < 0) return -1;
    }

    for (uint64_t vaddr = USER_HEAP_BASE; vaddr < parent->user_heap_cursor;
         vaddr += PAGE_SIZE) {
        uint64_t phys = paging_virt_to_phys(parent_cr3, vaddr);
        if (phys == 0) continue;
        if (process_copy_user_page(child_cr3, vaddr, phys) < 0) return -1;
    }

    for (uint64_t vaddr = parent->user_stack_base;
         vaddr < parent->user_stack_top; vaddr += PAGE_SIZE) {
        uint64_t phys = paging_virt_to_phys(parent_cr3, vaddr);
        if (phys == 0) continue;
        if (process_copy_user_page(child_cr3, vaddr, phys) < 0) return -1;
    }

done:
    return clone_rc;
}

/* Per-ABI state that has to follow a process through fork() and exit --
 * today the Linux layer's per-process descriptor table
 * (Compat/Linux/Linux_FdTable.c). Registered by that layer at init, so the
 * process manager needs no knowledge of it. */
/* Stop the other threads of the caller's process for a fork. Returns 1 when
 * every one of them is off-CPU (and will stay off until
 * process_fork_hold_release()); 0 when some are still running and the caller
 * should come back shortly -- the hold stays set meanwhile, so they get off
 * at their next scheduling point and are not picked again. */
int process_fork_hold_acquire(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t self = current_pid_get();
    int ready = 0;
    if (is_valid_pid(self)) {
        process_t *owner = process_memory_owner_locked(&g_processes[self]);
        if (owner != NULL) {
            int32_t owner_pid = (int32_t)(owner - g_processes);
            if (owner->fork_hold_tid >= 0 && owner->fork_hold_tid != self) {
                ready = 0; /* another thread of ours is forking: wait */
            } else {
                owner->fork_hold_tid = self;
                ready = 1;
                for (int32_t i = 0; i < g_process_capacity; ++i) {
                    process_t *p = &g_processes[i];
                    if (i == self || p->state == PROCESS_STATE_UNUSED ||
                        p->state == PROCESS_STATE_DEAD) {
                        continue;
                    }
                    int32_t eff = p->is_thread ? p->memory_owner_pid : i;
                    /* Only one that is running. A CPU that has nothing
                     * else to run idles still naming the thread that
                     * blocked on it, and keeps naming it -- woken, that
                     * thread is not picked again while the hold is set --
                     * until something else becomes runnable there. Xorg's
                     * input thread, blocked in poll(), held its fork for
                     * xkbcomp up to two minutes that way, and XSession gave
                     * up on the server. Blocked or parked by the hold, a
                     * thread runs no user code. */
                    if (eff == owner_pid && p->state == PROCESS_STATE_RUNNING &&
                        process_scheduler_pid_in_use_on_any_cpu(i)) {
                        ready = 0;
                        break;
                    }
                }
            }
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return ready;
}

void process_fork_hold_release(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t self = current_pid_get();
    if (is_valid_pid(self)) {
        process_t *owner = process_memory_owner_locked(&g_processes[self]);
        if (owner != NULL && owner->fork_hold_tid == self) {
            owner->fork_hold_tid = -1;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

static process_lifecycle_hook_t g_fork_hook;
static process_lifecycle_hook_t g_exit_hook;

void process_register_lifecycle_hooks(process_lifecycle_hook_t on_fork,
                                      process_lifecycle_hook_t on_exit)
{
    g_fork_hook = on_fork;
    g_exit_hook = on_exit;
}

static int32_t process_fork_impl(uint64_t child_user_rsp, uint32_t opts,
                                 uint64_t child_settid,
                                 uint64_t child_cleartid);

int32_t process_fork(void)
{
    return process_fork_impl(0u, 0u, 0u, 0u);
}

/* fork() whose child starts on `child_user_rsp` (clone() with a stack). The
 * stack has to be in place before the child can be scheduled: setting it
 * afterwards raced the other CPUs, and a child that won the race ran on the
 * parent's stack -- Chromium's chroot helper then "returned" into its
 * parent's frame with RBP 0 and faulted, failing ChrootToSafeEmptyDir(). */
int32_t process_fork_with_stack(uint64_t child_user_rsp)
{
    return process_fork_impl(child_user_rsp, 0u, 0u, 0u);
}

/* clone(): stack plus the per-child properties the Linux flags ask for
 * (PROCESS_FORK_NEWPID, PROCESS_FORK_SHARE_FS), all applied before the child
 * can run -- see process_fork_with_stack(). */
int32_t process_fork_ex(uint64_t child_user_rsp, uint32_t opts)
{
    return process_fork_impl(child_user_rsp, opts, 0u, 0u);
}

int32_t process_fork_ex_tid(uint64_t child_user_rsp, uint32_t opts,
                            uint64_t child_settid, uint64_t child_cleartid)
{
    return process_fork_impl(child_user_rsp, opts, child_settid,
                             child_cleartid);
}

/* Store a 32-bit value at user address `addr` in the address space `cr3`,
 * which is not the one running: a fork child's, before it runs. The page is
 * given its own frame first if it is still shared copy-on-write with the
 * parent -- writing the shared frame would change the parent's copy too. */
static int process_poke_user_u32(uint64_t cr3, uint64_t addr, uint32_t value)
{
    if (cr3 == 0u || addr < 0x1000u || (addr & 3u) != 0u) {
        return -1;
    }
    if (paging_virt_to_phys(cr3, addr) == 0u &&
        paging_handle_swap_fault(cr3, addr) <= 0) {
        return -1;
    }
#if KERNEL_COW_FORK
    (void)paging_user_range_break_cow(cr3, addr, sizeof(value));
#endif
    uint64_t phys = paging_virt_to_phys(cr3, addr);
    if (phys == 0u) {
        return -1;
    }
    *(volatile uint32_t *)(uintptr_t)phys = value;
    return 0;
}

static int32_t process_fork_impl(uint64_t child_user_rsp, uint32_t opts,
                                 uint64_t child_settid,
                                 uint64_t child_cleartid)
{
    if (!process_table_ready()) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t current_pid = current_pid_get();
    if (!is_valid_pid(current_pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }

    /* Two different things are "the parent" here, and conflating them is how
     * a fork() issued by a thread produced a child that resumed at the main
     * thread's instruction pointer:
     *   `owner`  owns the address space and everything process-wide (heap and
     *            stack bounds, signal dispositions, name, cwd);
     *   `caller` is the thread that actually called fork(), and its saved
     *            register frame, TLS base and FPU state are what the child
     *            has to come back on -- POSIX gives the child exactly one
     *            thread, the one that forked.
     * They are the same slot whenever the main thread forks. Chromium's
     * process launcher does not run on the main thread, so they usually are
     * not. */
    process_t *caller = &g_processes[current_pid];
    process_t *owner = process_memory_owner_locked(caller);
    if (!owner || owner->state == PROCESS_STATE_UNUSED) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }

    if (owner->is_thread) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -38;
    }
    process_t *parent = owner;

    int32_t child_pid = find_free_slot();
    if (child_pid < 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *child = &g_processes[child_pid];
    reset_process_slot(child);
    child->state = PROCESS_STATE_INIT;
    child->memory_owner_pid = child_pid;

    int32_t parent_pid_saved = (int32_t)(parent - g_processes);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    child->kernel_stack_base = malloc(PROCESS_KERNEL_STACK_SIZE);
    if (!child->kernel_stack_base) goto fail_free;
    child->kernel_stack_top =
        (((uint64_t)(uintptr_t)(child->kernel_stack_base +
                                 PROCESS_KERNEL_STACK_SIZE)) & ~0xFULL);
    process_kstack_arm(child);

    child->cr3 = paging_create_process_space();
    if (!child->cr3) goto fail_free_kstack;

    child->user_code_base = parent->user_code_base;
    child->user_code_limit = parent->user_code_limit;
    child->user_heap_base = parent->user_heap_base;
    child->user_heap_cursor = parent->user_heap_cursor;
    child->user_heap_limit = parent->user_heap_limit;
    child->user_heap_alloc_limit = parent->user_heap_alloc_limit;
    child->user_brk_base = parent->user_brk_base;
    child->user_brk_cursor = parent->user_brk_cursor;
    child->user_brk_limit = parent->user_brk_limit;
    child->user_heap_guard_page = parent->user_heap_guard_page;
    child->user_stack_base = parent->user_stack_base;
    child->user_stack_top = parent->user_stack_top;
    child->user_mmap_cursor = parent->user_mmap_cursor;
    child->user_stack_guard_page = parent->user_stack_guard_page;

    if (process_clone_address_space(child, parent) < 0) goto fail_free_space;

    /* Demand-paged file mappings live outside the page tables, so the child
     * needs its own records (and its own references on the underlying open
     * file descriptions) or its faults in those ranges would be answered with
     * zeroes. */
    if (filemap_clone_for_fork(parent_pid_saved, child_pid) < 0) {
        goto fail_free_space;
    }

    /* POSIX: the child sees the parent's open descriptors at the same numbers.
     * Without this, the very first thing a forked child does with an inherited
     * fd fails -- Popen()'s child cannot dup2() the pipe it was handed. */
    syscall_file_fork_inherit(parent_pid_saved, child_pid);
    /* AF_UNIX endpoints live in their own table and need the same treatment:
     * without it the child could not close its inherited half without
     * destroying the parent's, and the parent could not close its half
     * without destroying the child's. */
    unix_socket_fork_inherit(parent_pid_saved, child_pid);
    /* Still before the child is READY: it must never run with a table it has
     * not been given yet. */
    if (g_fork_hook != NULL) {
        g_fork_hook(parent_pid_saved, child_pid);
    }

    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        child->user_allocs[i] = parent->user_allocs[i];
    }

    child->capability_mask = parent->capability_mask;
    child->abi_mode = parent->abi_mode;
    child->priority = parent->priority;
    /* Per-thread state comes from the calling thread, not from the process. */
    child->rseq_area = caller->rseq_area;
    child->rseq_sig = caller->rseq_sig;
    child->fs_base = caller->fs_base;
    /* hal_cpu_read_gs_base() would read the kernel's per-CPU GS (swapgs is
     * already done in syscall context); inherit the user GS like FS. */
    child->gs_base = caller->gs_base;
    memcpy(child->fpu_state, caller->fpu_state, PROCESS_FPU_STATE_SIZE);
    /* Linux i386: the caller's TLS descriptors and selectors, its ECX, and
     * the occupancy of its mmap window. */
    memcpy(child->compat_tls, caller->compat_tls, sizeof(child->compat_tls));
    child->compat_fs_sel = caller->compat_fs_sel;
    child->compat_gs_sel = caller->compat_gs_sel;
    if (PROCESS_ABI_IS_COMPAT32(parent->abi_mode)) {
        child->full_restore = 1;
        child->full_rcx = caller->full_rcx;
        child->full_r11 = caller->full_r11;
        if (parent->compat_vm_map != NULL) {
            child->compat_vm_map = compat32_vm_map_alloc(parent->compat_vm_map);
            if (child->compat_vm_map == NULL) {
                goto fail_free_space;
            }
        }
    }

    uint64_t *parent_kstack = (uint64_t *)(uintptr_t)caller->saved_rsp;
    uint64_t *child_kstack = (uint64_t *)(uintptr_t)child->kernel_stack_top;
    child_kstack -= PROCESS_CONTEXT_QWORDS;
    for (uint32_t i = 0; i < PROCESS_CONTEXT_QWORDS; ++i) {
        child_kstack[i] = parent_kstack ? parent_kstack[i] : 0;
    }
    child_kstack[SYSCALL_FRAME_RAX] = 0;

    child->saved_rsp = (uint64_t)(uintptr_t)child_kstack;
    child->saved_user_rsp = child_user_rsp != 0u ? child_user_rsp
                                                 : syscall_get_user_rsp();
#if defined(__aarch64__)
    /* The frame is what an arm64 task resumes from when it is switched in
     * by preemption rather than at a syscall return. */
    ((arm64_exception_frame_t *)child_kstack)->sp_el0 = child->saved_user_rsp;
#endif

    /* CLONE_CHILD_SETTID: the child's own TID, written into its memory
     * before it runs. glibc's fork passes &THREAD_SELF->tid here and never
     * sets it any other way, so without this a forked child went on
     * believing it was its parent -- harmless for fork+exec, but Chromium's
     * zygote children never exec, and glibc writes that TID into every PI
     * mutex it takes: the kernel's FUTEX_UNLOCK_PI then saw someone else's
     * lock (EPERM) and glibc aborted the process ("The futex facility
     * returned an unexpected error code." in the audio service). The value is
     * the TID as the child sees it: 1 for the init of a new PID namespace. */
    if (child_settid != 0u) {
        uint32_t tid_seen = (opts & PROCESS_FORK_NEWPID) ? 1u
                                                          : (uint32_t)child_pid;
        (void)process_poke_user_u32(child->cr3, child_settid, tid_seen);
    }

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (child->state != PROCESS_STATE_INIT) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        goto fail_free_space;
    }

    child->entry = parent->entry;
    child->parent_pid = parent_pid_saved;
    child->pdeath_signal = 0;
    child->pgid = parent->pgid >= 0 ? parent->pgid : parent_pid_saved;
    child->sid = parent->sid >= 0 ? parent->sid : parent_pid_saved;
    child->timeslice = g_timeslice_ticks;
    child->state = PROCESS_STATE_READY;
    process_perf_mark_ready_locked(child, process_perf_now_ns());

    memcpy(child->name, parent->name, sizeof(child->name));
    memcpy(child->cwd, parent->cwd, sizeof(child->cwd));
    memcpy(child->root_path, parent->root_path, sizeof(child->root_path));
    child->pidns_init = (opts & PROCESS_FORK_NEWPID) ? child_pid
                                                     : parent->pidns_init;
    child->fs_share_pid = (opts & PROCESS_FORK_SHARE_FS)
                              ? (int32_t)(caller - g_processes) : -1;
    child->uid = parent->uid;
    child->gid = parent->gid;
    /* CLONE_CHILD_CLEARTID: zeroed and futex-woken when the child exits. */
    child->clear_child_tid = child_cleartid;
    memcpy(child->exe_path, parent->exe_path, sizeof(child->exe_path));
    memcpy(child->launch_argument, parent->launch_argument,
           sizeof(child->launch_argument));

    for (uint32_t i = 0; i < PROCESS_SIGNAL_MAX; ++i) {
        child->signal_handlers[i] = parent->signal_handlers[i];
        child->signal_flags[i] = parent->signal_flags[i];
        child->signal_sa_mask[i] = parent->signal_sa_mask[i];
        child->signal_restorer[i] = parent->signal_restorer[i];
    }
    child->altstack_sp = parent->altstack_sp;
    child->altstack_size = parent->altstack_size;
    child->altstack_flags = parent->altstack_flags;
    child->signal_mask = caller->signal_mask;
    child->pending_signals = 0;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    ipc_init_process_queue(child_pid);

    return child_pid;

fail_free_space:
    paging_destroy_process_space(child->cr3);
    child->cr3 = 0;
fail_free_kstack:
    free(child->kernel_stack_base);
    child->kernel_stack_base = NULL;
fail_free:
    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    reset_process_slot(child);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return -1;
}

int32_t process_execve(const char *path, const char *const *argv,
                       const char *const *envp)
{
    if (!path || path[0] == '\0') return -22;

    char path_buf[256];
    uint64_t path_len = 0;
    if (process_user_cstring_length(path, sizeof(path_buf) - 1, &path_len) < 0) {
        return -14;
    }
    if (copy_from_user(path_buf, path, path_len + 1) != 0) {
        return -14;
    }

    /* /proc/self/exe (and /proc/<pid>/exe) is a symlink to the running
     * image, and Chromium re-executes itself through it for every child it
     * does not fork from the zygote (the network service among them). The
     * loader only understands real paths, so follow the link here. */
    if (strncmp(path_buf, "/proc/", 6) == 0) {
        char target[256];
        if (procfs_readlink(path_buf, target, sizeof(target)) == 0 &&
            target[0] == '/') {
            size_t tlen = strlen(target);
            if (tlen < sizeof(path_buf)) {
                memcpy(path_buf, target, tlen + 1u);
                path_len = tlen;
            }
        }
    }

    if (path_buf[0] != '/') {
        char cwd_buf[256];
        if (process_get_current_cwd(cwd_buf, sizeof(cwd_buf)) != 0) {
            return -36;
        }
        uint32_t cwd_len = (uint32_t)strlen(cwd_buf);
        while (cwd_len > 1u && cwd_buf[cwd_len - 1u] == '/') {
            cwd_buf[cwd_len - 1u] = '\0';
            --cwd_len;
        }
        if ((uint64_t)cwd_len + path_len + 2u > sizeof(path_buf)) {
            return -36;
        }
        memmove(path_buf + cwd_len + 1u, path_buf, path_len + 1u);
        memcpy(path_buf, cwd_buf, cwd_len);
        path_buf[cwd_len] = '/';
        path_len += (uint64_t)cwd_len + 1u;
    }
    /* Wine spawns its server as
     * "/usr/lib/x86_64-linux-gnu/wine/../../wine/wineserver"; the
     * filesystems walk components literally and know no "..". */
    vfs_canonicalize_path(path_buf);
    path_len = strlen(path_buf);

    uint64_t argc = 0;
    uint64_t envc = 0;
    uint64_t argv_strtotal = 0;
    uint64_t envp_strtotal = 0;

    if (count_user_string_array(argv, &argc, &argv_strtotal) < 0) {
        return -22;
    }
    if (count_user_string_array(envp, &envc, &envp_strtotal) < 0) {
        return -22;
    }

    (void)argv_strtotal;

    char strings_buf[EXECVE_STRTOTAL_MAX];
    uint64_t offsets[EXECVE_ARG_MAX];
    char *argv_ptrs[EXECVE_ARG_MAX];
    char *envp_ptrs[EXECVE_ARG_MAX];
    /* Raw "#!" line (BINPRM_BUF_SIZE). Both the interpreter path and its
     * optional single argument end up as NUL-terminated substrings *inside*
     * this one buffer, which is function-scoped so they stay valid until the
     * new user stack is built. Kept deliberately small - process_execve's
     * frame is already large (strings_buf[8192] + 3x pointer arrays). */
    char shebang_line[256];

    memset(strings_buf, 0, sizeof(strings_buf));

    if (argc > 0) {
        if (copy_user_strings(argv, argc, strings_buf, sizeof(strings_buf), offsets) < 0) {
            return -14;
        }
        for (uint64_t i = 0; i < argc; ++i)
            argv_ptrs[i] = strings_buf + offsets[i];
    }

    if (envc > 0) {
        uint64_t argv_len = (argc > 0) ? (offsets[argc - 1] + strlen(argv_ptrs[argc - 1]) + 1) : 0;
        if (argv_len + envp_strtotal > sizeof(strings_buf))
            return -22;
        if (copy_user_strings(envp, envc, strings_buf + argv_len, sizeof(strings_buf) - argv_len, offsets) < 0)
            return -14;
        for (uint64_t i = 0; i < envc; ++i)
            envp_ptrs[i] = strings_buf + argv_len + offsets[i];
    }

    /*
     * Shebang ("#!") handling. Linux does this in the kernel exec path: if the
     * target file begins with "#!", the first line names an interpreter (and at
     * most one optional argument, taken verbatim as a single arg), and the
     * kernel re-targets execve at that interpreter with argv rewritten to
     *   [ interp, (optarg,) script_path, original argv[1..] ].
     * Only a single level is resolved here (the common "#!/bin/sh" case where
     * /bin/sh is itself an ELF); a nested script interpreter yields the normal
     * "exec format error" from the ELF loader below. Chromium itself is a plain
     * ELF, so this only matters for busybox/dash-style helper scripts.
     */
    {
        vfs_file_t sb_file;
        memset(&sb_file, 0, sizeof(sb_file));
        if (vfs_find_file(path_buf, &sb_file)) {
            uint32_t want = sb_file.size < sizeof(shebang_line)
                                ? sb_file.size
                                : (uint32_t)sizeof(shebang_line);
            if (want >= 2u &&
                vfs_read_at(&sb_file, 0u, (uint8_t *)shebang_line, want) &&
                shebang_line[0] == '#' && shebang_line[1] == '!') {
                /* NUL-terminate at end of the first line (or buffer end). */
                uint32_t eol = 2u;
                while (eol < want && shebang_line[eol] != '\n' &&
                       shebang_line[eol] != '\0') {
                    ++eol;
                }
                if (eol >= sizeof(shebang_line)) {
                    eol = (uint32_t)sizeof(shebang_line) - 1u;
                }
                shebang_line[eol] = '\0';

                /* interp = first token; optional arg = rest of the line as a
                 * single arg. Both are NUL-terminated in place inside
                 * shebang_line by writing '\0' at their boundaries. */
                uint32_t p = 2u;
                while (shebang_line[p] == ' ' || shebang_line[p] == '\t') ++p;
                uint32_t interp_start = p;
                while (shebang_line[p] != '\0' && shebang_line[p] != ' ' &&
                       shebang_line[p] != '\t') {
                    ++p;
                }
                uint32_t interp_end = p;
                while (shebang_line[p] == ' ' || shebang_line[p] == '\t') ++p;
                uint32_t arg_start = p;
                uint32_t arg_end = (uint32_t)strlen(shebang_line);
                while (arg_end > arg_start &&
                       (shebang_line[arg_end - 1u] == ' ' ||
                        shebang_line[arg_end - 1u] == '\t' ||
                        shebang_line[arg_end - 1u] == '\r')) {
                    --arg_end;
                }
                shebang_line[interp_end] = '\0';
                int have_arg = (arg_end > arg_start);
                if (have_arg) {
                    shebang_line[arg_end] = '\0';
                }

                char *sb_interp = &shebang_line[interp_start];
                uint64_t il = strlen(sb_interp);
                uint64_t extra = 1u + (have_arg ? 1u : 0u); /* interp (+arg) */
                uint64_t orig_rest = (argc > 1u) ? (argc - 1u) : 0u;
                uint64_t new_argc = extra + 1u /* script path */ + orig_rest;

                if (interp_end > interp_start && il < sizeof(path_buf) &&
                    new_argc <= EXECVE_ARG_MAX) {
                    /* Stash the script path in the free tail of strings_buf so
                     * an argv entry can keep pointing at it after path_buf is
                     * re-aimed at the interpreter. Compute the used extent from
                     * the pointer arrays directly - offsets[] was overwritten
                     * with envp offsets by the envc block above. */
                    uint64_t used = 0;
                    for (uint64_t i = 0; i < argc; ++i) {
                        uint64_t e = (uint64_t)(argv_ptrs[i] - strings_buf) +
                                     strlen(argv_ptrs[i]) + 1u;
                        if (e > used) used = e;
                    }
                    for (uint64_t i = 0; i < envc; ++i) {
                        uint64_t e = (uint64_t)(envp_ptrs[i] - strings_buf) +
                                     strlen(envp_ptrs[i]) + 1u;
                        if (e > used) used = e;
                    }
                    /* The interpreter and its argument go into strings_buf
                     * too: every argv entry has to live there, or the next
                     * step (a foreign interpreter run by its translator, e.g.
                     * Wine's #!/bin/sh wineserver wrapper with /bin/sh an
                     * x86_64 dash) measures the buffer from a pointer outside
                     * it and gives up. */
                    uint64_t spl = strlen(path_buf);
                    const char *sb_arg = have_arg ? &shebang_line[arg_start] : "";
                    uint64_t al = have_arg ? strlen(sb_arg) + 1u : 0u;
                    if (used + spl + 1u + il + 1u + al <= sizeof(strings_buf)) {
                        char *script_copy = strings_buf + used;
                        memcpy(script_copy, path_buf, spl + 1u);
                        char *interp_copy = script_copy + spl + 1u;
                        memcpy(interp_copy, sb_interp, il + 1u);
                        char *arg_copy = interp_copy + il + 1u;
                        if (have_arg) {
                            memcpy(arg_copy, sb_arg, al);
                        }

                        /* Shift original argv[1..] right to make room for the
                         * prepended entries, then drop argv[0]. */
                        if (orig_rest > 0) {
                            memmove(&argv_ptrs[extra + 1u], &argv_ptrs[1],
                                    (size_t)orig_rest * sizeof(char *));
                        }
                        argv_ptrs[0] = interp_copy;
                        if (have_arg) {
                            argv_ptrs[1] = arg_copy;
                        }
                        argv_ptrs[extra] = script_copy;
                        argc = new_argc;

                        /* Re-target execve at the interpreter. */
                        memcpy(path_buf, sb_interp, il + 1u);
                        path_len = il;
                    }
                }
            }
        }
    }

    /* A program for the other CPU: exec its translator instead, the program
     * path and its own argv[0] becoming the translator's arguments
     * (process_foreign_translator). After the "#!" step, so a script whose
     * interpreter is a foreign binary works too. */
    char guest_path[256];
    guest_path[0] = '\0';
    {
        char tpath[TRANSLATOR_PATH_MAX];
        if (argc + 3u <= EXECVE_ARG_MAX &&
            process_foreign_translator(path_buf, tpath, sizeof(tpath), NULL)) {
            uint64_t used = 0;
            for (uint64_t k = 0; k < argc; ++k) {
                uint64_t e = (uint64_t)(argv_ptrs[k] - strings_buf) +
                             strlen(argv_ptrs[k]) + 1u;
                if (e > used) used = e;
            }
            for (uint64_t k = 0; k < envc; ++k) {
                uint64_t e = (uint64_t)(envp_ptrs[k] - strings_buf) +
                             strlen(envp_ptrs[k]) + 1u;
                if (e > used) used = e;
            }
            size_t tl = strlen(tpath);
            if (used + tl + 1u <= sizeof(strings_buf)) {
                char *tcopy = strings_buf + used;
                memcpy(tcopy, tpath, tl + 1u);
                memcpy(guest_path, path_buf, path_len + 1u);
                char *argv0 = argc > 0u ? argv_ptrs[0] : guest_path;
                uint64_t rest = argc > 1u ? argc - 1u : 0u;
                if (rest > 0u) {
                    memmove(&argv_ptrs[4], &argv_ptrs[1], (size_t)rest * sizeof(char *));
                }
                argv_ptrs[0] = tcopy;
                argv_ptrs[1] = (char *)"-0";
                argv_ptrs[2] = argv0;
                argv_ptrs[3] = guest_path;
                argc = rest + 4u;
                memcpy(path_buf, tpath, tl + 1u);
                path_len = tl;
            } else {
                /* Carrying on would load the foreign ELF natively, fail past
                 * the point of no return and kill the caller. */
                serial_write_string("[app] exec: no room for the translator path, used=");
                serial_write_uint64(used);
                serial_write_string(" argc=");
                serial_write_uint64(argc);
                serial_write_string(" envc=");
                serial_write_uint64(envc);
                serial_write_string("\n");
                return -7; /* E2BIG */
            }
        }
    }

    /* Everything that can make the exec fail cleanly has to be checked
     * before the point of no return below, because after it the caller has
     * no image left to return to and is killed instead. That included a
     * target that does not exist -- which is the normal case for execvp()
     * and posix_spawnp() walking $PATH: glibc tries /bin/<name>, expects
     * ENOENT, then /usr/bin/<name>. Killing the caller at the first miss
     * meant no Linux program could ever run anything found later in PATH
     * (Chromium's xdg-open for "Open" / "Show in folder" among them). */
    {
        vfs_file_t probe;
        memset(&probe, 0, sizeof(probe));
        if (!vfs_find_file(path_buf, &probe)) {
            return -2; /* ENOENT */
        }
        if (probe.size < 4u) {
            return -8; /* ENOEXEC: not an ELF, not a script */
        }
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t pid = current_pid_get();
    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    process_t *proc = process_memory_owner_locked(&g_processes[pid]);
    if (!proc || proc->state == PROCESS_STATE_UNUSED) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }

    int32_t proc_pid = (int32_t)(proc - g_processes);
    /*
     * execve(2) preserves inherited descriptors: only those explicitly marked
     * FD_CLOEXEC are dropped. Previously this closed *every* fd (including
     * stdin/stdout/stderr and any pipe/socket handed to the child), which
     * broke glibc programs and Chromium's Mojo fd passing after exec. Sockets
     * carry no per-fd CLOEXEC bit yet, so they always inherit (the POSIX
     * default); a failed exec still tears everything down via
     * process_exit_current().
     */
    syscall_file_close_cloexec_for_pid(proc_pid);
    shared_memory_cleanup_process(proc_pid);
    linux_sysv_shm_cleanup_process(proc_pid);
    drm_kms_notify_process_exit(proc_pid);

    /* POSIX: execve resets every caught signal to SIG_DFL. The new image has
     * none of the old one's code, so keeping the handlers meant a process that
     * forked from Xorg and exec'd /bin/sh still ran Xorg's SIGSEGV handler --
     * which faulted itself, at an address that no longer belonged to it, over
     * and over. Ignored dispositions and the blocked mask are inherited, as
     * POSIX requires; only handlers are cleared. */
    /* signal_handlers[]: 0 = SIG_DFL, 1 = SIG_IGN, anything else is a user
     * handler -- only the last of those is meaningless after exec. */
    for (uint32_t sig = 0; sig < PROCESS_SIGNAL_MAX; ++sig) {
        if (proc->signal_handlers[sig] > 1u) {
            proc->signal_handlers[sig] = 0u;
            proc->signal_flags[sig] = 0u;
            proc->signal_sa_mask[sig] = 0u;
            proc->signal_restorer[sig] = 0u;
        }
    }
    proc->pending_signals = 0;
    proc->altstack_sp = 0;
    proc->altstack_size = 0;
    proc->altstack_flags = 0;
    /* execve replaces the address space wholesale, so every demand-paged file
     * mapping (and its reference on the open file description) goes with it. */
    filemap_release_pid(proc_pid);

    uint64_t old_cr3 = proc->cr3;
    proc->cr3 = 0;
    proc->rseq_area = 0;
    proc->rseq_sig = 0;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    paging_destroy_process_space(old_cr3);

    uint64_t new_cr3 = paging_create_process_space();
    if (!new_cr3) {
        /* Past the point of no return: the old image is gone, so there is
         * nothing to return to. Linux kills the process here rather than
         * failing the call, and returning -1 instead left the caller running
         * on a destroyed address space -- which is how a failed exec of
         * xkbcomp turned into busybox faulting at NULL. */
        serial_write_string("[app] exec failed: create_space\n");
        process_exit_current_with_status(127);
        return -1;
    }

    proc->cr3 = new_cr3;
    proc->user_code_base = USER_CODE_BASE;
    proc->user_code_limit = USER_CODE_LIMIT;
    proc->user_heap_base = USER_HEAP_BASE;
    proc->user_heap_cursor = USER_HEAP_BASE;
    proc->user_heap_limit = USER_HEAP_LIMIT;
    proc->user_stack_base = USER_STACK_BASE;
    proc->user_stack_top = USER_STACK_TOP;
    proc->user_mmap_cursor = USER_MMAP_BASE;

    proc->user_heap_guard_page = proc->user_heap_limit - PROCESS_GUARD_PAGE_SIZE;
    proc->user_heap_limit -= PROCESS_GUARD_PAGE_SIZE;
    uint64_t thread_reserve =
        (uint64_t)g_process_capacity * PROCESS_THREAD_STACK_REGION_SIZE;
    if (thread_reserve >= (proc->user_heap_limit - proc->user_heap_base)) {
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        /* Past the point of no return: the old image is gone, so there is
         * nothing to return to. Linux kills the process here rather than
         * failing the call, and returning -1 instead left the caller running
         * on a destroyed address space -- which is how a failed exec of
         * xkbcomp turned into busybox faulting at NULL. */
        serial_write_string("[app] exec failed: thread_reserve\n");
        process_exit_current_with_status(127);
        return -1;
    }
    proc->user_heap_alloc_limit = proc->user_heap_limit - thread_reserve;

    /* Carve the program break its own window, below anything the bump
     * allocator can reach.
     *
     * brk() used to be served by process_user_alloc() -- the same bump/
     * free-list allocator that backs process_user_mmap(). glibc's malloc
     * treats the break as one contiguous arena it owns outright, so sharing an
     * allocator with mmap() is not survivable: a brk() satisfied out of the
     * free list left user_heap_cursor where it was, and the next dlopen() then
     * mapped a shared object straight over memory malloc had already handed
     * out. It surfaced as function pointers in Xorg's heap reading back as
     * addresses inside libmtdev / modesetting_drv, and as threads returning
     * into string data. See TODO_Doom_Xorg_MethodA.md M20. */
    if (USER_BRK_WINDOW_SIZE >=
        (proc->user_heap_alloc_limit - proc->user_heap_base)) {
        return -1;
    }
    proc->user_brk_limit  = proc->user_heap_alloc_limit;
    proc->user_brk_base   = proc->user_brk_limit - USER_BRK_WINDOW_SIZE;
    proc->user_brk_cursor = proc->user_brk_base;
    proc->user_heap_alloc_limit = proc->user_brk_base;

    proc->user_stack_guard_page = proc->user_stack_base;
    proc->user_stack_base += PROCESS_GUARD_PAGE_SIZE;

    uint64_t initial_stack_base =
        proc->user_stack_top - PROCESS_INITIAL_USER_STACK_SIZE;
    if (initial_stack_base < proc->user_stack_base ||
        paging_map_user_range_alloc(proc->cr3, initial_stack_base,
                                     PROCESS_INITIAL_USER_STACK_SIZE,
                                     PAGE_RW | PAGE_USER) < 0) {
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        /* Past the point of no return: the old image is gone, so there is
         * nothing to return to. Linux kills the process here rather than
         * failing the call, and returning -1 instead left the caller running
         * on a destroyed address space -- which is how a failed exec of
         * xkbcomp turned into busybox faulting at NULL. */
        serial_write_string("[app] exec failed: initial_stack\n");
        process_exit_current_with_status(127);
        return -1;
    }

    /* A translated program is known by its own path, not the translator's
     * (ps, /proc/<pid>/exe -- which a re-exec of /proc/self/exe follows). */
    const char *shown_path = guest_path[0] != '\0' ? guest_path : path_buf;
    const char *name_ptr = shown_path;
    for (const char *p = shown_path; *p; ++p) {
        if (*p == '/' && *(p+1) != '\0') name_ptr = p + 1;
    }
    strncpy(proc->name, name_ptr, sizeof(proc->name) - 1);
    proc->name[sizeof(proc->name) - 1] = '\0';
    strncpy(proc->exe_path, shown_path, sizeof(proc->exe_path) - 1u);
    proc->exe_path[sizeof(proc->exe_path) - 1u] = '\0';

    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        proc->user_allocs[i].used = 0;
        proc->user_allocs[i].addr = 0;
        proc->user_allocs[i].size = 0;
    }

    elf_load_policy_t elf_policy = {
        .max_file_size = PROCESS_ELF_MAX_SIZE,
        .min_vaddr = USER_FOREIGN_BASE,
        .max_vaddr = USER_CODE_LIMIT,
    };
    elf_loaded_image_info_t image_info = {0};
    if (!elf_loader_load_from_path(proc->cr3, proc_pid, path_buf,
                                    &elf_policy, &image_info)) {
        filemap_release_pid(proc_pid);
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        /* Past the point of no return: the old image is gone, so there is
         * nothing to return to. Linux kills the process here rather than
         * failing the call, and returning -1 instead left the caller running
         * on a destroyed address space -- which is how a failed exec of
         * xkbcomp turned into busybox faulting at NULL. */
        serial_write_string("[app] exec failed: elf_load\n");
        process_exit_current_with_status(127);
        return -1;
    }

    proc->abi_mode = image_info.abi_mode;
    proc->main_phdr_vaddr = image_info.phdr_vaddr;
    proc->main_phent = image_info.phent;
    proc->main_phnum = image_info.phnum;
    /* The new image may run under a different ABI than the old one (a 64-bit
     * shell exec'ing a 32-bit program, or back). */
    if (!PROCESS_ABI_IS_COMPAT32(proc->abi_mode)) {
        if (proc->compat_vm_map != NULL) {
            free(proc->compat_vm_map);
            proc->compat_vm_map = NULL;
        }
        memset(proc->compat_tls, 0, sizeof(proc->compat_tls));
        proc->compat_fs_sel = 0;
        proc->compat_gs_sel = 0;
    }
    if ((PROCESS_ABI_IS_COMPAT32(proc->abi_mode) &&
         process_apply_compat32_layout(proc, &image_info) < 0) ||
        (proc->abi_mode == PROCESS_ABI_LINUX_ARM64 &&
         process_map_sigpage(proc) < 0)) {
        filemap_release_pid(proc_pid);
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        serial_write_string("[app] exec failed: abi_layout\n");
        process_exit_current_with_status(127);
        return -1;
    }
    if (!image_info.linux_abi) {
        /* A native program reads its arguments through
         * process_get_launch_argument(), not argv: give it argv[1..], joined
         * with spaces, the same string process_spawn_with_arg() would have
         * passed. Otherwise it saw whatever its Linux parent was launched
         * with (e.g. /usr/bin/xdg-open run by Chromium got Chromium's own
         * command line). */
        uint32_t at = 0;
        for (uint32_t ai = 1; ai < (uint32_t)argc && argv_ptrs[ai] != NULL; ++ai) {
            const char *a = argv_ptrs[ai];
            if (ai > 1 && at + 1u < sizeof(proc->launch_argument)) {
                proc->launch_argument[at++] = ' ';
            }
            /* Native programs have no notion of the working directory they
             * were started in, so a relative path is made absolute here.
             * Chromium's "Show in folder" runs `xdg-open .` from inside the
             * folder. Options and URLs are passed through. */
            if (a[0] != '/' && a[0] != '-' && a[0] != '\0' &&
                strstr(a, "://") == NULL && proc->cwd[0] == '/') {
                const char *c = proc->cwd;
                while (*c != '\0' && at + 1u < sizeof(proc->launch_argument)) {
                    proc->launch_argument[at++] = *c++;
                }
                if (strcmp(a, ".") == 0) {
                    a += 1;
                } else {
                    if (at > 0 && proc->launch_argument[at - 1u] != '/' &&
                        at + 1u < sizeof(proc->launch_argument)) {
                        proc->launch_argument[at++] = '/';
                    }
                    if (a[0] == '.' && a[1] == '/') a += 2;
                }
            }
            while (*a != '\0' && at + 1u < sizeof(proc->launch_argument)) {
                proc->launch_argument[at++] = *a++;
            }
        }
        proc->launch_argument[at] = '\0';
    }
    foreign_launch_log_start((int32_t)(proc - g_processes),
                             guest_path[0] != '\0' ? "exec (translated)" : "exec",
                             shown_path, NULL, image_info.linux_abi);

    if (initialize_elf_user_stack_ex(proc, &image_info, path_buf,
                                      argc, (const char *const *)argv_ptrs,
                                      envc, (const char *const *)envp_ptrs) < 0) {
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        process_exit_current_with_status(127);
        return -1;
    }

    uint64_t kstack_saved = proc->saved_rsp;
    if (kstack_saved != 0) {
#if defined(__aarch64__)
        /* The new program starts with clean registers at its entry, on the
         * stack just built (the frame layout is arm64_exception_frame_t, not
         * the x86 syscall frame). */
        arm64_exception_frame_t *frame =
            (arm64_exception_frame_t *)(uintptr_t)kstack_saved;
        memset(frame->x, 0, sizeof(frame->x));
        frame->elr_el1 = image_info.entry;
        frame->sp_el0 = proc->saved_user_rsp;
        frame->spsr_el1 = 0;
#else
        uint64_t *kstack = (uint64_t *)(uintptr_t)kstack_saved;
        if (image_info.linux_abi) {
            /* Linux starts a program with its registers zeroed, and the
             * startup code relies on it: %edx/%rdx is the atexit function
             * the dynamic linker passes, which must be 0 for a static
             * binary -- otherwise exit() calls whatever execve()'s third
             * argument left there. */
            for (uint32_t i = 0; i < SYSCALL_FRAME_QWORDS; ++i) {
                kstack[i] = 0;
            }
            kstack[SYSCALL_FRAME_R11] = PROCESS_RFLAGS_DEFAULT;
            proc->full_rcx = 0;
            proc->full_r11 = 0;
        }
        kstack[SYSCALL_FRAME_RCX] = image_info.entry;
#endif
    }

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    /* RUNNING belongs here: a process calling execve() on itself is, by
     * definition, the one running -- the scheduler stamps PROCESS_STATE_RUNNING
     * on whatever it dispatches. Accepting only INIT/READY made every
     * self-exec fail at the very last step, after the new image had already
     * been loaded and the return RIP pointed at its entry, so the process ran
     * the new program with a stack that was never handed over and died on the
     * first argv access. The check is here to catch a process torn down
     * underneath us, so reject exactly those states instead. */
    if (proc->state != PROCESS_STATE_INIT &&
        proc->state != PROCESS_STATE_READY &&
        proc->state != PROCESS_STATE_RUNNING) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        process_exit_current_with_status(127);
        return -1;
    }
    proc->entry = image_info.entry;
    proc->timeslice = g_timeslice_ticks;
    proc->state = PROCESS_STATE_READY;
    proc->user_stack_exchanged = 1;
    proc->ready_since_ns = process_perf_now_ns();
    for (uint32_t i = 0; i < PROCESS_SIGNAL_MAX; ++i) {
        proc->signal_handlers[i] = 0;
        proc->signal_flags[i] = 0;
        proc->signal_sa_mask[i] = 0;
        proc->signal_restorer[i] = 0;
    }
    proc->altstack_sp = 0;
    proc->altstack_size = 0;
    proc->altstack_flags = 2u; /* SS_DISABLE */
    proc->signal_mask = 0;
    proc->pending_signals = 0;
    proc->fs_base = 0;
    proc->gs_base = 0;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    /* Successful exec, not just the failures: X shells out to xkbcomp through
     * /bin/sh, and telling "the shell never ran" apart from "xkbcomp ran and
     * hung" needs the successful hop logged too. */

    return 0;
}

int32_t process_copy_launch_argument_of(int32_t pid, char *out, uint32_t capacity)
{
    if (!out || capacity == 0u) return -1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = is_valid_pid(pid) ? &g_processes[pid] : NULL;
    proc = process_memory_owner_locked(proc);
    if (!proc) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    uint32_t length = 0u;
    while (length + 1u < capacity &&
           proc->launch_argument[length] != '\0') {
        out[length] = proc->launch_argument[length];
        ++length;
    }
    out[length] = '\0';
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return (int32_t)length;
}

int32_t process_copy_exe_path_of(int32_t pid, char *out, uint32_t capacity)
{
    if (!out || capacity == 0u) return -1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = is_valid_pid(pid) ? &g_processes[pid] : NULL;
    proc = process_memory_owner_locked(proc);
    if (!proc) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    uint32_t length = 0u;
    while (length + 1u < capacity && proc->exe_path[length] != '\0') {
        out[length] = proc->exe_path[length];
        ++length;
    }
    out[length] = '\0';
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return (int32_t)length;
}

/* Current-process wrappers. /proc/<pid>/{cmdline,exe} needs the same two
 * readers for a pid other than the caller's -- the zygote host and the crash
 * handler both introspect their children -- so the bodies above take an
 * explicit pid and these keep the old call sites unchanged. */
int32_t process_copy_launch_argument(char *out, uint32_t capacity)
{
    return process_copy_launch_argument_of(current_pid_get(), out, capacity);
}

int32_t process_copy_exe_path(char *out, uint32_t capacity)
{
    return process_copy_exe_path_of(current_pid_get(), out, capacity);
}

int64_t process_get_main_image_info(uint64_t *phdr_vaddr,
                                    uint64_t *phent,
                                    uint64_t *phnum)
{
    if (phdr_vaddr == NULL || phent == NULL || phnum == NULL) {
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    process_t *proc = is_valid_pid(current_pid_get()) ?
        &g_processes[current_pid_get()] : NULL;
    proc = process_memory_owner_locked(proc);
    if (proc == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return (int64_t)OS_STATUS_FAULT;
    }
    *phdr_vaddr = proc->main_phdr_vaddr;
    *phent = proc->main_phent;
    *phnum = proc->main_phnum;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int32_t process_register_boot_process(const char *path, uint64_t *entry_out)
{    int32_t pid = process_spawn_user_elf(path);
    if (pid < 0) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = &g_processes[pid];
    proc->state = PROCESS_STATE_RUNNING;
    current_pid_set(pid);
    proc->timeslice = g_timeslice_ticks;
    proc->ready_since_ns = 0u;
    proc->last_scheduled_ns = process_perf_now_ns();
    process_scheduler_consume_reschedule();
    if (entry_out) {
        *entry_out = proc->entry;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    return pid;
}

int32_t process_register_boot_process_from_memory(const void *data, uint64_t size, uint64_t *entry_out)
{
    if (!data || size == 0) return -1;

    int32_t pid = process_create_user_internal(USER_CODE_BASE, 0, 0, 0, 0, 0);
    if (pid < 0) {
        return -1;
    }

    process_t *proc = &g_processes[pid];
    strncpy(proc->name, OS_CONFIG_USERLAND_INIT_NAME, sizeof(proc->name) - 1);
    proc->name[sizeof(proc->name) - 1] = '\0';

    elf_load_policy_t policy = {
        .max_file_size = PROCESS_ELF_MAX_SIZE,
        .min_vaddr = USER_CODE_BASE,
        .max_vaddr = USER_CODE_LIMIT,
    };
    elf_loaded_image_info_t image_info = {0};

    if (!elf_loader_load_from_memory(proc->cr3, data, size, &policy, &image_info)) {
        ipc_cleanup_process_queue(pid);
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        release_process_resources(proc);
        reset_process_slot(proc);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    proc->main_phdr_vaddr = image_info.phdr_vaddr;
    proc->main_phent = image_info.phent;
    proc->main_phnum = image_info.phnum;

    if (initialize_elf_user_stack(proc, &image_info,
                                 OS_CONFIG_USERLAND_INIT_PATH) < 0) {
        ipc_cleanup_process_queue(pid);
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        release_process_resources(proc);
        reset_process_slot(proc);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();

#if defined(__aarch64__)
    arm64_exception_frame_t *frame =
        (arm64_exception_frame_t *)(uintptr_t)proc->saved_rsp;
    frame->elr_el1 = image_info.entry;
    frame->sp_el0 = proc->saved_user_rsp;
#else
    uint64_t *kstack = (uint64_t *)(uintptr_t)proc->saved_rsp;
    kstack[SYSCALL_FRAME_RCX] = image_info.entry;
#endif

    spinlock_lock(&g_process_table_lock);
    proc->state = PROCESS_STATE_RUNNING;
    current_pid_set(pid);
    proc->timeslice = g_timeslice_ticks;
    proc->ready_since_ns = 0u;
    proc->last_scheduled_ns = process_perf_now_ns();
    process_scheduler_consume_reschedule();
    if (entry_out) {
        *entry_out = image_info.entry;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    return pid;
}

static int process_group_has_living_member_locked(int32_t leader_pid);

static void process_teardown(int32_t pid_to_exit);

void process_exit_current(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }

    process_t *current = &g_processes[current_pid_get()];
    process_t *owner = process_memory_owner_locked(current);
    if (owner == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }
    int32_t pid_to_exit = (int32_t)(owner - g_processes);
    /* Several threads can run exit_group() at once. The first one tears the
     * group down; a later one must not run the cleanup a second time (every
     * table would be released twice) -- it only has to stop running. */
    if (owner->state == PROCESS_STATE_ZOMBIE) {
        if (current != owner) {
            current->state = PROCESS_STATE_DEAD;
        }
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }
    /* Claim the teardown before dropping the lock. */
    owner->state = PROCESS_STATE_ZOMBIE;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    process_teardown(pid_to_exit);
}

/* Everything a dead process leaves behind except its slot and address space
 * (those go when the parent reaps it). Runs once per process, after it has
 * been marked a zombie: from its own exit, or -- for a process killed by a
 * signal -- from process_run_pending_teardowns(). */
static void process_teardown(int32_t pid_to_exit)
{
    uint64_t irq_flags;
    process_t *owner = &g_processes[pid_to_exit];
    int32_t exit_status_log = owner->exit_status;
    uint8_t abi_mode_log = PROCESS_ABI_IS_LINUX(owner->abi_mode) ? 1u : 0u;

    foreign_launch_log_exit(pid_to_exit, exit_status_log, abi_mode_log);

    uint32_t closed_fds = 0;
    uint32_t closed_dirs = 0;
    syscall_file_close_all_for_pid(pid_to_exit, &closed_fds, &closed_dirs);
    syscall_socket_close_all_for_pid(pid_to_exit);
    unix_socket_close_all_for_pid(pid_to_exit);
    if (g_exit_hook != NULL) {
        g_exit_hook(pid_to_exit, -1);
    }
    shared_memory_cleanup_process(pid_to_exit);
    linux_sysv_shm_cleanup_process(pid_to_exit);
    drm_kms_notify_process_exit(pid_to_exit);
    filemap_release_pid(pid_to_exit);
    /* After the fds: closing the last slave fd is what tells the terminal
     * emulator the session ended, and this only drops the controlling-terminal
     * binding the pid leaves behind. */
    pty_forget_process(pid_to_exit);

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    g_processes[pid_to_exit].state = PROCESS_STATE_ZOMBIE;
    for (int32_t i = 1; i < g_process_capacity; ++i) {
        if (g_processes[i].is_thread &&
            g_processes[i].memory_owner_pid == pid_to_exit &&
            g_processes[i].state != PROCESS_STATE_UNUSED) {
            g_processes[i].state = PROCESS_STATE_DEAD;
        }
    }
    process_notify_parent_sigchld_locked(&g_processes[pid_to_exit]);
    /* Another CPU naming the group leader as current is only stale when the
     * leader itself is the one exiting here. When a secondary thread runs
     * exit_group(), the leader may be legitimately mid-syscall on another
     * CPU (often in its own exit_group()); clearing it there sent that CPU
     * back to user space with no process behind it (a #GP on glibc's
     * post-exit hlt, "no current thread on this CPU"), and let the parent
     * reap the group while it was still running. */
    if (current_pid_get() == pid_to_exit) {
        process_scheduler_release_stale_pid(pid_to_exit);
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    pnp_notifications_cleanup_process(pid_to_exit);
    ipc_cleanup_process_queue(pid_to_exit);
    syscall_input_owner_release(pid_to_exit);

    /* Release any file-backed MAP_SHARED write-back registrations (live ones
     * were already flushed by munmap/msync). */
    extern void linux_compat_mshared_release_pid(int32_t pid);
    linux_compat_mshared_release_pid(pid_to_exit);

    /* Children that asked for a signal when their parent dies
     * (PR_SET_PDEATHSIG) -- Chromium's zygote and helpers do, and without it
     * they outlived a killed browser, holding their memory for good. */
    for (;;) {
        int32_t victims[32];
        uint8_t sigs[32];
        uint32_t nv = 0;
        irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        for (int32_t i = 1; i < g_process_capacity && nv < 32u; ++i) {
            process_t *c = &g_processes[i];
            if (!c->is_thread && c->parent_pid == pid_to_exit && c->pdeath_signal != 0u &&
                c->state != PROCESS_STATE_UNUSED && c->state != PROCESS_STATE_DEAD &&
                c->state != PROCESS_STATE_ZOMBIE) {
                victims[nv] = i;
                sigs[nv] = c->pdeath_signal;
                c->pdeath_signal = 0;       /* once */
                ++nv;
            }
        }
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        for (uint32_t k = 0; k < nv; ++k) {
            (void)process_signal_deliver_group(victims[k], sigs[k]);
        }
        if (nv < 32u) {
            break;
        }
    }

#if PAGING_TRACKS_ACTIVE_SPACE
    /* The address space goes now, not when the parent reaps the zombie (Linux
     * frees a process's memory at exit too). A zombie nobody waits for -- the
     * Chromium browser under a shell that never calls wait, and through it
     * every child process it left behind -- otherwise kept gigabytes until
     * the machine ran out. Every thread of the group is dead by now; the
     * deferred teardown waits for the CPUs still on these tables. */
    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    const uint64_t dead_cr3 = g_processes[pid_to_exit].cr3;
    if (dead_cr3 != 0u && g_processes[pid_to_exit].state == PROCESS_STATE_ZOMBIE) {
        for (int32_t i = 1; i < g_process_capacity; ++i) {
            process_t *p = &g_processes[i];
            if ((i == pid_to_exit || (p->is_thread && p->memory_owner_pid == pid_to_exit)) &&
                p->cr3 == dead_cr3) {
                p->cr3 = 0u;
                p->thread_stack_region_base = 0u;
                p->thread_stack_region_size = 0u;
            }
        }
        deferred_space_add(dead_cr3);
        deferred_space_drain();
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
#endif

    /* A child becoming a zombie is what a parent parked in wait4() is waiting
     * for, and closing this process's fds has just made the other end of every
     * pipe and socket it held readable (EOF). Both of those are readiness
     * events, so cut short whoever is sleeping on them rather than making them
     * serve out the slice: Xorg's Pclose() waits for xkbcomp this way, twice,
     * for every keyboard it configures. */
    poll_wait_notify();
}

int process_set_pdeath_signal(int32_t signum)
{
    if (signum < 0 || signum >= (int32_t)PROCESS_SIGNAL_MAX) {
        return -1;
    }
    int rc = -1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        process_t *owner = process_memory_owner_locked(&g_processes[current_pid_get()]);
        if (owner != NULL) {
            owner->pdeath_signal = (uint8_t)signum;
            rc = 0;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return rc;
}

int32_t process_get_pdeath_signal(void)
{
    int32_t sig = 0;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        process_t *owner = process_memory_owner_locked(&g_processes[current_pid_get()]);
        if (owner != NULL) {
            sig = owner->pdeath_signal;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return sig;
}

/* Run the teardown of every process a signal killed (see
 * process_signal_kill_locked()). Called on the way into every syscall: one
 * atomic load when there is nothing to do. */
void process_run_pending_teardowns(void)
{
    while (__atomic_load_n(&g_teardown_pending, __ATOMIC_ACQUIRE) != 0u) {
        int32_t victim = -1;
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        for (int32_t i = 1; i < g_process_capacity; ++i) {
            process_t *p = &g_processes[i];
            if (p->teardown_pending && !p->is_thread &&
                !process_group_on_cpu_locked(i)) {
                p->teardown_pending = 0u;
                victim = i;
                break;
            }
        }
        if (victim >= 0) {
            __atomic_fetch_sub(&g_teardown_pending, 1u, __ATOMIC_ACQ_REL);
        }
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        if (victim < 0) {
            return; /* still on a CPU somewhere: the next syscall retries */
        }
        process_teardown(victim);
    }
}

void process_exit_current_signaled(int32_t signum)
{
    /* TEMPORARY DIAGNOSTIC (revert): which of the several kill paths took a
     * process down, and as whom it is recorded. */

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        process_t *owner =
            process_memory_owner_locked(&g_processes[current_pid_get()]);
        if (owner != NULL) {
            owner->exit_status = 128 + signum;
            owner->exit_by_signal = 1u;
            owner->exit_term_signal = (uint8_t)signum;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    process_exit_current();
}

void process_exit_current_with_status(int32_t exit_status)
{
    /* TEMPORARY DIAGNOSTIC (revert): a process that leaves through exit_group
     * rather than by signal -- the status is its own argument. */

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        process_t *owner =
            process_memory_owner_locked(&g_processes[current_pid_get()]);
        if (owner != NULL) {
            owner->exit_status = exit_status;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    process_exit_current();
}

/* Linux exit_robust_list() equivalent. Runs in the dying thread's own
 * address space (called from its pthread_exit path) so user memory is
 * directly reachable. For every mutex still on the thread's robust list that
 * it still owns, set FUTEX_OWNER_DIED in the futex word (clearing the owner
 * TID, preserving FUTEX_WAITERS) and wake one waiter, so a sibling thread's
 * glibc pthread_mutex_lock() returns EOWNERDEAD and can recover the mutex. */
static void process_robust_handle_entry_current(uint64_t entry,
                                                int64_t futex_offset,
                                                int32_t tid)
{
    if (entry == 0u) {
        return;
    }
    uint64_t uaddr = entry + (uint64_t)futex_offset;
    uint32_t word = 0;
    if (copy_from_user(&word, (const void *)(uintptr_t)uaddr,
                       sizeof(word)) != 0u) {
        return;
    }
    if ((int32_t)(word & 0x3FFFFFFFu) != tid) {
        return; /* Not (or no longer) owned by the dying thread. */
    }
    uint32_t newword = (word & 0x80000000u) | 0x40000000u; /* WAITERS | OWNER_DIED */
    if (copy_to_user((void *)(uintptr_t)uaddr, &newword, sizeof(newword)) != 0u) {
        return;
    }
    if ((word & 0x80000000u) != 0u) {
        (void)syscall_futex(uaddr, 1u /* FUTEX_WAKE */, 1u, 0u, 0u, 0u);
    }
}

static void process_run_robust_list_current(uint64_t head_ptr, int32_t tid)
{
    if (head_ptr == 0u) {
        return;
    }
    /* struct robust_list_head { void *next; long futex_offset; void *pending; } */
    uint64_t next = 0u;
    uint64_t pending = 0u;
    int64_t futex_offset = 0;
    if (copy_from_user(&next, (const void *)(uintptr_t)head_ptr,
                       sizeof(next)) != 0u ||
        copy_from_user(&futex_offset,
                       (const void *)(uintptr_t)(head_ptr + 8u),
                       sizeof(futex_offset)) != 0u ||
        copy_from_user(&pending,
                       (const void *)(uintptr_t)(head_ptr + 16u),
                       sizeof(pending)) != 0u) {
        return;
    }

    uint32_t limit = 2048u; /* ROBUST_LIST_LIMIT */
    uint64_t entry = next;
    while (entry != head_ptr && entry != 0u && limit-- != 0u) {
        if (entry != pending) {
            process_robust_handle_entry_current(entry, futex_offset, tid);
        }
        uint64_t chain = 0u;
        if (copy_from_user(&chain, (const void *)(uintptr_t)entry,
                           sizeof(chain)) != 0u) {
            break;
        }
        entry = chain;
    }
    if (pending != 0u) {
        process_robust_handle_entry_current(pending, futex_offset, tid);
    }
}

/* exit_group() from a thread must not come back, whatever state the group is
 * in. process_exit_current() retires every thread of a live group, but when
 * the group is already on its way out (another thread got there first, or the
 * owner slot is mid-teardown) it returns early and the caller stayed runnable
 * -- it went back to user space, fell onto the hlt glibc places after the
 * syscall, and took a #GP. Mark the caller dead outright. */
void process_retire_current_thread(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t tid = current_pid_get();
    if (is_valid_pid(tid) && g_processes[tid].is_thread) {
        uint8_t st = g_processes[tid].state;
        if (st == PROCESS_STATE_RUNNING || st == PROCESS_STATE_READY ||
            st == PROCESS_STATE_BLOCKED) {
            g_processes[tid].state = PROCESS_STATE_DEAD;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

void process_thread_exit_current(int32_t exit_status)
{
    uint64_t clear_child_tid = 0;
    uint64_t robust_head = 0;
    int32_t group_leader = -1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t tid = current_pid_get();
    if (is_valid_pid(tid) && g_processes[tid].is_thread) {
        process_t *thread = &g_processes[tid];
        process_t *owner = process_memory_owner_locked(thread);
        if (owner != NULL) {
            group_leader = (int32_t)(owner - g_processes);
        }
        clear_child_tid = thread->clear_child_tid;
        thread->clear_child_tid = 0;
        robust_head = thread->robust_list_head;
        thread->robust_list_head = 0;
        thread->robust_list_length = 0;
        thread->exit_status = exit_status;
        thread->state = thread->thread_detached ?
                        PROCESS_STATE_DEAD : PROCESS_STATE_ZOMBIE;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (robust_head != 0u && is_valid_pid(tid)) {
        /* Mutex words hold TIDs as the process sees them (PID namespace). */
        process_run_robust_list_current(robust_head,
                                        process_pid_as_seen_by_current(tid));
    }

    if (clear_child_tid != 0u) {
        uint32_t zero = 0;
        if (copy_to_user((void *)(uintptr_t)clear_child_tid,
                         &zero, sizeof(zero)) == 0u) {
            (void)syscall_futex(clear_child_tid, 1u, 1u, 0u, 0u, 0u);
        }
    }

    if (group_leader >= 0 &&
        !process_group_has_living_member_locked(group_leader)) {
        process_exit_current();
    }
}

static int process_group_has_living_member_locked(int32_t leader_pid)
{
    if (!is_valid_pid(leader_pid)) {
        return 0;
    }
    process_t *leader = &g_processes[leader_pid];
    if (leader->state != PROCESS_STATE_UNUSED &&
        leader->state != PROCESS_STATE_DEAD &&
        leader->state != PROCESS_STATE_ZOMBIE) {
        return 1;
    }
    for (int32_t i = 1; i < g_process_capacity; ++i) {
        process_t *thread = &g_processes[i];
        if (thread->is_thread && thread->memory_owner_pid == leader_pid &&
            thread->state != PROCESS_STATE_UNUSED &&
            thread->state != PROCESS_STATE_DEAD &&
            thread->state != PROCESS_STATE_ZOMBIE) {
            return 1;
        }
    }
    return 0;
}

/* Lock-free on purpose. It reads only the running task's own slot and its
 * memory owner, neither of which can be torn down while the task runs (a group
 * exit reaps only once every thread is off its CPU). Taking
 * g_process_table_lock here made every caller that already holds another
 * subsystem's lock order that lock before the process table -- the reverse of
 * process_execve(), which holds the process table while it closes
 * close-on-exec descriptors -- and the file table deadlocked against execve
 * (all CPUs spinning, interrupts off). It is also on nearly every syscall. */
int32_t process_get_current_pid(void)
{
    int32_t cur = current_pid_get();
    if (!is_valid_pid(cur)) {
        return -1;
    }
    process_t *owner = process_memory_owner_locked(&g_processes[cur]);
    return owner != NULL ? (int32_t)(owner - g_processes) : -1;
}

int32_t process_get_current_tid(void)
{
    /* Lock-free for the same reason as process_get_current_pid() above: it
     * reads only the running task's own slot, via the per-CPU current-pid
     * word, and that slot cannot be torn down while the task runs. This is on
     * the front of every foreign-ABI syscall (linux_syscall_heartbeat), every
     * futex operation and every poll park; taking g_process_table_lock here
     * serialised all of those on one cacheline with interrupts off. */
    int32_t tid = current_pid_get();
    return is_valid_pid(tid) ? tid : -1;
}

int process_thread_join(int32_t tid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t current_tid = current_pid_get();
    if (!is_valid_pid(current_tid) || !is_valid_pid(tid) ||
        tid == current_tid || !g_processes[tid].is_thread) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -22;
    }

    process_t *current_owner =
        process_memory_owner_locked(&g_processes[current_tid]);
    process_t *thread = &g_processes[tid];
    if (current_owner == NULL ||
        thread->memory_owner_pid != (int32_t)(current_owner - g_processes) ||
        thread->thread_detached) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    if (thread->state != PROCESS_STATE_ZOMBIE) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -11;
    }
    if (process_scheduler_pid_in_use_on_any_cpu(tid)) {
        thread->state = PROCESS_STATE_DEAD;
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }

    release_thread_resources(thread, 1);
    reset_process_slot(thread);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int process_thread_detach(int32_t tid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t current_tid = current_pid_get();
    if (!is_valid_pid(current_tid) || !is_valid_pid(tid) ||
        !g_processes[tid].is_thread) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }

    process_t *current_owner =
        process_memory_owner_locked(&g_processes[current_tid]);
    process_t *thread = &g_processes[tid];
    if (current_owner == NULL ||
        thread->memory_owner_pid != (int32_t)(current_owner - g_processes) ||
        thread->thread_detached) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -22;
    }

    thread->thread_detached = 1;
    if (thread->state == PROCESS_STATE_ZOMBIE) {
        thread->state = PROCESS_STATE_DEAD;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

uint64_t process_get_current_saved_rsp(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }
    uint64_t saved_rsp = g_processes[current_pid_get()].saved_rsp;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return saved_rsp;
}

uint64_t process_get_current_user_rsp(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }
    uint64_t rsp = g_processes[current_pid_get()].saved_user_rsp;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return rsp;
}

uint64_t process_get_current_cr3(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return paging_get_kernel_cr3();
    }
    uint64_t cr3 = g_processes[current_pid_get()].cr3;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return cr3;
}

const char *process_get_current_name_str(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return "none";
    }
    const char *name = g_processes[current_pid_get()].name;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return name;
}

uint64_t process_get_current_kernel_stack_base(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }
    uint64_t base = (uint64_t)(uintptr_t)
        g_processes[current_pid_get()].kernel_stack_base;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return base;
}

void process_debug_dump_current(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t pid = current_pid_get();
    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }
    process_t *proc = &g_processes[pid];
    extern void serial_write_uint64(uint64_t value);
    extern void serial_write_string(const char *str);
    serial_write_string("[PFDBG] pid=");
    serial_write_uint64((uint64_t)(uint32_t)pid);
    serial_write_string(" name=");
    serial_write_string(proc->name);
    serial_write_string(" is_thread=");
    serial_write_uint64((uint64_t)proc->is_thread);
    serial_write_string(" owner=");
    serial_write_uint64((uint64_t)(uint32_t)proc->memory_owner_pid);
    serial_write_string(" saved_rsp=");
    serial_write_uint64(proc->saved_rsp);
    serial_write_string(" saved_user_rsp=");
    serial_write_uint64(proc->saved_user_rsp);
    serial_write_string(" stack_top=");
    serial_write_uint64(proc->user_stack_top);
    serial_write_string(" heap_limit=");
    serial_write_uint64(proc->user_heap_limit);
    if (proc->saved_rsp != 0u) {
        const uint64_t *kf = (const uint64_t *)(uintptr_t)proc->saved_rsp;
        serial_write_string(" kf_rcx=");
        serial_write_string(" ");
        serial_write_uint64(kf[SYSCALL_FRAME_RCX]);
        serial_write_string(" kf_r11=");
        serial_write_uint64(kf[SYSCALL_FRAME_R11]);
        serial_write_string(" kf_rax=");
        serial_write_uint64(kf[SYSCALL_FRAME_RAX]);
    }
    serial_write_string("\n");
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

void process_debug_dump_slot_no_lock(int32_t pid)
{
    extern void serial_write_string(const char *str);
    extern void serial_write_uint64(uint64_t value);
    if (!is_valid_pid(pid)) {
        return;
    }
    process_t *proc = &g_processes[pid];
    serial_write_string("[SLOT] pid=");
    serial_write_uint64((uint64_t)(uint32_t)pid);
    serial_write_string(" name=");
    serial_write_string(proc->name);
    serial_write_string(" is_thread=");
    serial_write_uint64((uint64_t)proc->is_thread);
    serial_write_string(" owner=");
    serial_write_uint64((uint64_t)(uint32_t)proc->memory_owner_pid);
    serial_write_string(" parent=");
    serial_write_uint64((uint64_t)(uint32_t)proc->parent_pid);
    serial_write_string(" state=");
    serial_write_uint64((uint64_t)(uint32_t)proc->state);
    serial_write_string(" saved_rsp=");
    serial_write_uint64(proc->saved_rsp);
    serial_write_string(" saved_user_rsp=");
    serial_write_uint64(proc->saved_user_rsp);
    serial_write_string(" kstack_base=");
    serial_write_uint64((uint64_t)(uintptr_t)proc->kernel_stack_base);
    serial_write_string(" kstack_top=");
    serial_write_uint64(proc->kernel_stack_top);
    serial_write_string(" cr3=");
    serial_write_uint64(proc->cr3);
    serial_write_string("\n");
}

void process_debug_dump_pid(int32_t pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }
    process_t *proc = &g_processes[pid];
    extern void serial_write_uint64(uint64_t value);
    extern void serial_write_string(const char *str);
    serial_write_string("[PFDBG] pid=");
    serial_write_uint64((uint64_t)(uint32_t)pid);
    serial_write_string(" name=");
    serial_write_string(proc->name);
    serial_write_string(" is_thread=");
    serial_write_uint64((uint64_t)proc->is_thread);
    serial_write_string(" owner=");
    serial_write_uint64((uint64_t)(uint32_t)proc->memory_owner_pid);
    serial_write_string(" state=");
    serial_write_uint64((uint64_t)(uint32_t)proc->state);
    serial_write_string(" saved_rsp=");
    serial_write_uint64(proc->saved_rsp);
    serial_write_string(" saved_user_rsp=");
    serial_write_uint64(proc->saved_user_rsp);
    serial_write_string(" stack_top=");
    serial_write_uint64(proc->user_stack_top);
    serial_write_string(" heap_limit=");
    serial_write_uint64(proc->user_heap_limit);
    if (proc->saved_rsp != 0u) {
        const uint64_t *kf = (const uint64_t *)(uintptr_t)proc->saved_rsp;
        serial_write_string(" kf_rcx=");
        serial_write_string(" ");
        serial_write_uint64(kf[SYSCALL_FRAME_RCX]);
        serial_write_string(" kf_r11=");
        serial_write_uint64(kf[SYSCALL_FRAME_R11]);
        serial_write_string(" kf_rax=");
        serial_write_uint64(kf[SYSCALL_FRAME_RAX]);
        serial_write_string(" kf_rdi=");
        serial_write_uint64(kf[SYSCALL_FRAME_RDI]);
        serial_write_string(" kf_rsi=");
        serial_write_uint64(kf[SYSCALL_FRAME_RSI]);
        serial_write_string(" kf_rdx=");
        serial_write_uint64(kf[SYSCALL_FRAME_RDX]);
        serial_write_string(" kf_r8=");
        serial_write_uint64(kf[SYSCALL_FRAME_R8]);
    }
    serial_write_string("\n");
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

uint64_t process_schedule_on_syscall(uint64_t current_saved_rsp,
                                     uint64_t current_user_rsp,
                                     int request_switch,
                                     uint64_t *next_user_rsp_out)
{
    if (next_user_rsp_out != NULL) {
        *next_user_rsp_out = current_user_rsp;
    }

    process_release_parked_stack();

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_scheduler_clear_leaving_pid();
    uint64_t now_ns = process_perf_now_ns();
    int do_switch = (request_switch & PROCESS_SCHEDULE_REQUEST_SWITCH) != 0;
    int involuntary = (request_switch & PROCESS_SCHEDULE_REQUEST_INVOLUNTARY) != 0;

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return current_saved_rsp;
    }

    process_t *current = &g_processes[current_pid_get()];
    if (do_switch) {
        process_perf_account_runtime_locked(current, now_ns);
    }
    if (current->state == PROCESS_STATE_RUNNING ||
        current->state == PROCESS_STATE_READY ||
        current->state == PROCESS_STATE_BLOCKED) {
        uint8_t original_state = current->state;
        save_syscall_frame_to_process(current, current_saved_rsp);
        if (current_user_rsp != 0 && current->user_stack_exchanged == 0) {
            current->saved_user_rsp = current_user_rsp;
        }
        current->user_stack_exchanged = 0;
        process_fpu_save(current->fpu_state);
        
        current->fs_base = rdmsr_fs_base();
        current->gs_base = rdmsr_kernel_gs_base();
#if defined(__x86_64__)
        compat32_capture_selectors(current);
#endif

        if (do_switch && original_state != PROCESS_STATE_BLOCKED) {
            current->state = PROCESS_STATE_READY;
            process_perf_mark_ready_locked(current, now_ns);
        }
    }

    /* A sibling is forking this process: get off the CPU and stay off (the
     * scheduler skips held threads) until it is done. */
    if (!do_switch &&
        scheduler_pid_held_for_fork(g_processes, g_process_capacity,
                                    current_pid_get())) {
        do_switch = 1;
        if (current->state == PROCESS_STATE_RUNNING) {
            current->state = PROCESS_STATE_READY;
            process_perf_mark_ready_locked(current, now_ns);
        }
    }

    if (!do_switch &&
        (current->state == PROCESS_STATE_RUNNING ||
         current->state == PROCESS_STATE_READY)) {
        process_deliver_pending_signals_locked(current);
        uint64_t return_saved_rsp = current->saved_rsp;
        uint64_t return_user_rsp = current->saved_user_rsp;
        current->state = PROCESS_STATE_RUNNING;

        process_fpu_restore(current->fpu_state);
        activate_process_context(current);
        
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        
        if (next_user_rsp_out != NULL) {
            *next_user_rsp_out = return_user_rsp;
        }
        return return_saved_rsp;
    }

    /* exit_group() lands here on the dying process's own kernel stack, and the
     * idle loop below can sit on it indefinitely. Hand the stack to this CPU
     * and stop naming the corpse, so the parent's wait4() can reap it. */
    process_detach_dead_current_locked();

    int32_t next_pid = process_scheduler_pick_next(g_processes,
                                                   g_process_capacity,
                                                   current_pid_get());

    while (next_pid < 0) {
        uint32_t ecpu = smp_get_current_cpu_id();
        spinlock_unlock(&g_process_table_lock);
        uint64_t idle_start_ns = timer_monotonic_ns();
        process_scheduler_idle_wait();
        process_scheduler_add_idle_ns(timer_monotonic_ns() - idle_start_ns);
        hal_cpu_disable_interrupts();
        spinlock_lock(&g_process_table_lock);

        now_ns = process_perf_now_ns();
        process_wake_sleepers_locked(now_ns);
        next_pid = process_scheduler_pick_next(g_processes,
                                               g_process_capacity,
                                               current_pid_get());
    }

    int32_t old_pid = current_pid_get();
    if (next_pid >= 0 && next_pid != old_pid && old_pid >= 0) {
        process_scheduler_set_leaving_pid(old_pid);
    }

    current_pid_set(next_pid);
    process_t *next = &g_processes[current_pid_get()];
    process_deliver_pending_signals_locked(next);
    process_perf_prepare_run_locked(next, now_ns, involuntary);
    process_scheduler_prepare_run(next);

    uint64_t next_saved_rsp = next->saved_rsp;
    uint64_t next_user_rsp = next->saved_user_rsp;
    
    process_fpu_restore(next->fpu_state);
    activate_process_context(next);
    
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (next_user_rsp_out != NULL) {
        *next_user_rsp_out = next_user_rsp;
    }
    return next_saved_rsp;
}

uint64_t process_schedule_after_exit(uint64_t *next_user_rsp_out)
{
    if (next_user_rsp_out != NULL) {
        *next_user_rsp_out = 0;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_scheduler_clear_leaving_pid();
    uint64_t now_ns = process_perf_now_ns();

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        halt_forever();
    }

    process_perf_account_runtime_locked(&g_processes[current_pid_get()], now_ns);

    int32_t next_pid = process_scheduler_pick_next(g_processes,
                                                   g_process_capacity,
                                                   current_pid_get());
    if (next_pid < 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        halt_forever();
    }

    int32_t old_pid = current_pid_get();
    if (next_pid >= 0 && next_pid != old_pid && old_pid >= 0) {
        process_scheduler_set_leaving_pid(old_pid);
    }

    current_pid_set(next_pid);
    process_t *next = &g_processes[current_pid_get()];
    process_deliver_pending_signals_locked(next);
    process_perf_prepare_run_locked(next, now_ns, 0);
    process_scheduler_prepare_run(next);

    uint64_t next_saved_rsp = next->saved_rsp;
    uint64_t next_user_rsp = next->saved_user_rsp;
    process_fpu_restore(next->fpu_state);
    activate_process_context(next);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (next_user_rsp_out != NULL) {
        *next_user_rsp_out = next_user_rsp;
    }
    return next_saved_rsp;
}

void process_on_timer_tick(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_wake_sleepers_locked(process_perf_now_ns());
    process_scheduler_on_tick(g_processes, g_process_capacity);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}

int process_timeslice_expired(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int pending = process_scheduler_consume_reschedule();
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return pending;
}

#if defined(__x86_64__)
/* Timer preemption of user code.
 *
 * Until this existed, a task only ever left the CPU inside a syscall
 * (process_schedule_on_syscall): a thread computing in user mode -- V8
 * running a script, a video decoder, SwiftShader rasterising -- kept its CPU
 * until it happened to make a syscall, however many other threads were
 * ready. With four CPUs and a browser that runs dozens of threads that is
 * the difference between "slow" and "stuck".
 *
 * isr_regs is the timer ISR's register window: SAVE_REGS (r15 at [0] ...
 * rax at [14]) followed by the CPU frame ([15]=rip [16]=cs [17]=rflags
 * [18]=rsp [19]=ss). The interrupted state is rewritten as an ordinary
 * saved syscall frame at the top of the task's own kernel stack -- the same
 * place syscall_entry builds one -- plus the two registers such a frame
 * cannot hold (full_rcx/full_r11). Whichever exit path resumes the task
 * later sees full_restore and leaves through IRETQ with all of them. */
void process_preempt_from_user_irq(uint64_t *isr_regs)
{
    enum {
        IR_R15 = 0, IR_R14, IR_R13, IR_R12, IR_R11, IR_R10, IR_R9, IR_R8,
        IR_RBP, IR_RDI, IR_RSI, IR_RDX, IR_RCX, IR_RBX, IR_RAX,
        IR_RIP, IR_CS, IR_RFLAGS, IR_RSP, IR_SS, IR_COUNT
    };

    if (isr_regs == NULL || !process_table_ready() ||
        (isr_regs[IR_CS] & 3u) != 3u) {
        return;
    }
    if (!process_scheduler_consume_reschedule()) {
        return;
    }

    /* The window is about to be overwritten (it overlaps the frame built
     * below), so take a copy first. */
    uint64_t r[IR_COUNT];
    memcpy(r, isr_regs, sizeof(r));

    /* rseq: a thread preempted inside a registered critical section must
     * resume at its abort handler (Linux rseq_ip_fixup). This touches user
     * memory, which can fault -- and the fault handler takes the process
     * table lock -- so it runs before that lock is taken, on this thread's
     * own address space, through the fault-tolerant user-copy helpers. */
    {
        int32_t self = current_pid_get();
        uint64_t area = is_valid_pid(self) ? g_processes[self].rseq_area : 0u;
        uint64_t cs_ptr = 0;
        if (area != 0u &&
            copy_from_user(&cs_ptr,
                           (const void *)(uintptr_t)(area + PROCESS_RSEQ_CS),
                           sizeof(cs_ptr)) == 0u &&
            cs_ptr != 0u) {
            struct {
                uint32_t version, flags;
                uint64_t start_ip, post_commit_offset, abort_ip;
            } cs;
            if (copy_from_user(&cs, (const void *)(uintptr_t)cs_ptr,
                               sizeof(cs)) == 0u &&
                r[IR_RIP] - cs.start_ip < cs.post_commit_offset) {
                uint32_t sig = 0;
                if (cs.abort_ip >= 4u &&
                    copy_from_user(&sig,
                                   (const void *)(uintptr_t)(cs.abort_ip - 4u),
                                   sizeof(sig)) == 0u &&
                    sig == g_processes[self].rseq_sig) {
                    r[IR_RIP] = cs.abort_ip;
                }
            }
            uint64_t zero = 0;
            (void)copy_to_user((void *)(uintptr_t)(area + PROCESS_RSEQ_CS),
                               &zero, sizeof(zero));
        }
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t cur = current_pid_get();
    if (!is_valid_pid(cur) ||
        g_processes[cur].state != PROCESS_STATE_RUNNING ||
        g_processes[cur].kernel_stack_top == 0u ||
        scheduler_pid_held_for_fork(g_processes, g_process_capacity, cur)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }
    process_t *current = &g_processes[cur];

    int32_t next_pid = process_scheduler_pick_next(g_processes,
                                                   g_process_capacity, cur);
    if (next_pid < 0 || next_pid == cur) {
        /* Nobody else wants this CPU: keep running, with a fresh slice. */
        process_scheduler_prepare_run(current);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }

    uint64_t now_ns = process_perf_now_ns();

    uint64_t *frame = (uint64_t *)(uintptr_t)
        ((current->kernel_stack_top & ~0xFULL) -
         (uint64_t)SYSCALL_FRAME_QWORDS * sizeof(uint64_t));
    frame[SYSCALL_FRAME_RAX] = r[IR_RAX];
    frame[SYSCALL_FRAME_RDX] = r[IR_RDX];
    frame[SYSCALL_FRAME_RSI] = r[IR_RSI];
    frame[SYSCALL_FRAME_RDI] = r[IR_RDI];
    frame[SYSCALL_FRAME_R8]  = r[IR_R8];
    frame[SYSCALL_FRAME_R9]  = r[IR_R9];
    frame[SYSCALL_FRAME_R10] = r[IR_R10];
    frame[SYSCALL_FRAME_R12] = r[IR_R12];
    frame[SYSCALL_FRAME_R13] = r[IR_R13];
    frame[SYSCALL_FRAME_R14] = r[IR_R14];
    frame[SYSCALL_FRAME_R15] = r[IR_R15];
    frame[SYSCALL_FRAME_RBX] = r[IR_RBX];
    frame[SYSCALL_FRAME_RBP] = r[IR_RBP];
    frame[SYSCALL_FRAME_RCX] = r[IR_RIP];
    frame[SYSCALL_FRAME_R11] = r[IR_RFLAGS];

    current->saved_rsp = (uint64_t)(uintptr_t)frame;
    current->saved_user_rsp = r[IR_RSP];
    current->user_stack_exchanged = 0;
    current->full_restore = 1;
    current->full_rcx = r[IR_RCX];
    current->full_r11 = r[IR_R11];
    process_fpu_save(current->fpu_state);
    current->fs_base = rdmsr_fs_base();
    current->gs_base = rdmsr_kernel_gs_base();
    compat32_capture_selectors(current);

    process_perf_account_runtime_locked(current, now_ns);
    current->state = PROCESS_STATE_READY;
    process_perf_mark_ready_locked(current, now_ns);

    /* Still on cur's kernel stack until syscall_enter_user_from_frame
     * switches: keep other CPUs off it until then. */
    process_scheduler_set_leaving_pid(cur);

    current_pid_set(next_pid);
    process_t *next = &g_processes[next_pid];
    process_deliver_pending_signals_locked(next);
    process_perf_prepare_run_locked(next, now_ns, 1);
    process_scheduler_prepare_run(next);

    uint64_t next_saved_rsp = next->saved_rsp;
    uint64_t next_user_rsp = next->saved_user_rsp;
    uint64_t next_cr3 = next->cr3;
    process_fpu_restore(next->fpu_state);
    activate_process_context(next);
    spinlock_unlock(&g_process_table_lock);
    (void)irq_flags; /* interrupts stay off until the IRETQ into next */

    const arch_ops_t *ops = arch_ops_get();
    ops->enter_user_mode(next_saved_rsp, next_user_rsp, next_cr3);
}
#endif

#if defined(__aarch64__)
/* Timer preemption of user code, arm64 flavour of the x86 function above.
 *
 * Simpler than x86: an IRQ taken from EL0 already saved the complete user
 * register file as an arm64_exception_frame_t on this task's kernel stack --
 * at kernel_stack_top - sizeof(frame), the same place a syscall frame goes,
 * because SP_EL1 is left at kernel_stack_top when the task enters EL0 (see
 * arm64_enter_user_from_frame). That frame *is* the task's resumable
 * context, so all there is to do is record it and switch. */
void process_preempt_from_user_irq_arm64(arm64_exception_frame_t *frame)
{
    if (frame == NULL || !process_table_ready()) {
        return;
    }
    if (!process_scheduler_consume_reschedule()) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t cur = current_pid_get();
    if (!is_valid_pid(cur) ||
        g_processes[cur].state != PROCESS_STATE_RUNNING ||
        g_processes[cur].kernel_stack_top == 0u ||
        scheduler_pid_held_for_fork(g_processes, g_process_capacity, cur)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }
    process_t *current = &g_processes[cur];

    int32_t next_pid = process_scheduler_pick_next(g_processes,
                                                   g_process_capacity, cur);
    if (next_pid < 0 || next_pid == cur) {
        process_scheduler_prepare_run(current);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return;
    }

    uint64_t now_ns = process_perf_now_ns();

    current->saved_rsp = (uint64_t)(uintptr_t)frame;
    current->saved_user_rsp = frame->sp_el0;
    current->user_stack_exchanged = 0;
    process_fpu_save(current->fpu_state);
    current->fs_base = rdmsr_fs_base();

    process_perf_account_runtime_locked(current, now_ns);
    current->state = PROCESS_STATE_READY;
    process_perf_mark_ready_locked(current, now_ns);

    /* Still on cur's kernel stack until enter_user_mode switches: keep other
     * CPUs off it until then. */
    process_scheduler_set_leaving_pid(cur);

    current_pid_set(next_pid);
    process_t *next = &g_processes[next_pid];
    process_deliver_pending_signals_locked(next);
    process_perf_prepare_run_locked(next, now_ns, 1);
    process_scheduler_prepare_run(next);

    uint64_t next_saved_rsp = next->saved_rsp;
    uint64_t next_user_rsp = next->saved_user_rsp;
    uint64_t next_cr3 = next->cr3;
    process_fpu_restore(next->fpu_state);
    activate_process_context(next);
    spinlock_unlock(&g_process_table_lock);
    (void)irq_flags; /* interrupts stay masked until the eret into next */

    const arch_ops_t *ops = arch_ops_get();
    ops->enter_user_mode(next_saved_rsp, next_user_rsp, next_cr3);
}
#endif

int process_run_next_on_current_cpu(void)
{
    if (!process_table_ready()) {
        return 0;
    }

    process_release_parked_stack();

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_scheduler_clear_leaving_pid();

    int32_t current_pid = current_pid_get();
    process_rseq_update_on_preempt_locked(current_pid,
                                          smp_get_current_cpu_id());
    int32_t next_pid = process_scheduler_pick_next(g_processes,
                                                   g_process_capacity,
                                                   current_pid);
    if (next_pid < 0) {
        /* Nothing to dispatch: this CPU is going idle. Let go of the process
         * it still names if that process is already dead -- see
         * process_detach_dead_current_locked(). */
        process_detach_dead_current_locked();
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }

    if (next_pid != current_pid && current_pid >= 0) {
        process_scheduler_set_leaving_pid(current_pid);
    }

    current_pid_set(next_pid);
    process_t *next = &g_processes[next_pid];
    process_deliver_pending_signals_locked(next);
    process_perf_prepare_run_locked(next, process_perf_now_ns(), 0);
    process_scheduler_prepare_run(next);

    uint64_t next_saved_rsp = next->saved_rsp;
    uint64_t next_user_rsp = next->saved_user_rsp;
    uint64_t next_cr3 = next->cr3;
    process_fpu_restore(next->fpu_state);

    activate_process_context(next);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    const arch_ops_t *ops = arch_ops_get();
    if (ops != NULL && ops->enter_user_mode != NULL) {
        ops->enter_user_mode(next_saved_rsp, next_user_rsp, next_cr3);
    }
    return 0;
}

/* process_user_buffer_is_valid() plus "and a user-mode write to it would be
 * allowed". Destinations of read(2)-like syscalls need this: the kernel can
 * write through a read-only user mapping without trapping, so without the
 * check a read into PROT_READ memory silently succeeds where Linux returns
 * EFAULT. See paging_user_range_is_writable(). */
int process_user_buffer_is_writable(const void *ptr, uint64_t len)
{
    if (len == 0) {
        return 1;
    }
    if (!process_user_buffer_is_valid(ptr, len)) {
        return 0;
    }
    uint64_t cr3 = process_get_current_cr3();
    if (cr3 == 0) {
        return 1;
    }
#if KERNEL_COW_FORK
    (void)paging_user_range_break_cow(cr3, (uint64_t)(uintptr_t)ptr, len);
#endif
    return paging_user_range_is_writable(cr3, (uint64_t)(uintptr_t)ptr, len);
}

/* Unshare any copy-on-write pages under a user buffer the kernel is about to
 * write. Called from process_user_buffer_is_valid(), which nearly every
 * kernel->user copy passes through, so that a write after fork() lands in
 * this process's own page rather than the one it still shares with its
 * parent. Compiled out entirely when COW fork is off. */
void process_user_break_cow(const void *ptr, uint64_t len)
{
#if KERNEL_COW_FORK
    if (ptr == NULL || len == 0u) {
        return;
    }
    uint64_t cr3 = process_get_current_cr3();
    if (cr3 == 0u) {
        return;
    }
    (void)paging_user_range_break_cow(cr3, (uint64_t)(uintptr_t)ptr, len);
#else
    (void)ptr;
    (void)len;
#endif
}

/* Every page of [addr, addr+len) either mapped or part of a demand-paged
 * file mapping of the current process (FileMap), which copy_to/from_user()
 * fills in from the file through its kernel-mode #PF like a user access
 * would. A large Linux program's own image is demand-paged that way
 * (ELF_Loader, demand_page_image), and the paths, names and tables it hands
 * to system calls straight out of its .rodata are absent until something
 * reads them: Chromium's open("/dev/null") in the child it forks for the
 * zygote came back EFAULT, the child gave up ("Failed to open /dev/null")
 * and the browser died with "Cannot communicate with zygote". */
static int process_user_range_absent_file_backed(uint64_t cr3, uint64_t addr,
                                                 uint64_t len)
{
    const int32_t owner = process_get_current_pid();
    if (owner < 0) {
        return 0;
    }
    const uint64_t first = addr & ~0xFFFULL;
    const uint64_t last = (addr + len - 1u) & ~0xFFFULL;
    for (uint64_t page = first;; page += 0x1000ULL) {
        if (!paging_is_user_range_mapped(cr3, page, 1u) &&
            filemap_addr_is_file_backed(owner, page) <= 0) {
            return 0;
        }
        if (page == last) {
            break;
        }
    }
    return 1;
}

int process_user_buffer_is_valid(const void *ptr, uint64_t len)
{
    if (len == 0) {
        return 1;
    }

    uint64_t irq_flags = irq_save_disable();
    int32_t pid = current_pid_get();
    if (!is_valid_pid(pid)) {
        irq_restore(irq_flags);
        return 0;
    }

    const process_t *proc = &g_processes[pid];
    uint64_t addr           = (uint64_t)(uintptr_t)ptr;
    uint64_t code_base      = proc->user_code_base;
    uint64_t code_limit     = proc->user_code_limit;
    uint64_t heap_base      = proc->user_heap_base;
    uint64_t heap_limit     = proc->user_heap_limit;
    uint64_t stack_base     = proc->user_stack_base;
    uint64_t stack_top      = proc->user_stack_top;
    uint64_t process_cr3    = proc->cr3;
    uint8_t  abi_mode       = proc->abi_mode;
    /* A thread has its own process_t (and its own stack window carved out of
     * the owner's heap area) but shares the owner's address space, so a
     * pointer into the *creator's* stack is perfectly valid from a thread.
     * Remember the owner's stack window so the checks below accept it. */
    uint64_t owner_stack_base = stack_base;
    uint64_t owner_stack_top  = stack_top;
    if (proc->is_thread && is_valid_pid(proc->memory_owner_pid)) {
        const process_t *owner = &g_processes[proc->memory_owner_pid];
        owner_stack_base = owner->user_stack_base;
        owner_stack_top  = owner->user_stack_top;
    }
    irq_restore(irq_flags);

    /* The lazily-committed mmap arena is valid as a syscall buffer even when
     * pages are not yet resident: copy_to/from_user()'s memcpy takes a
     * kernel-mode #PF that the handler services demand-zero (see
     * page_fault_handler / process_user_reserve). Requiring residency here
     * would EFAULT every write into a fresh PartitionAlloc region. */
    if (range_within(addr, len, USER_MMAP_BASE, USER_MMAP_FIXED_LIMIT)) {
        return 1;
    }

    /* A Linux i386 process owns its whole window, demand-zero everywhere
     * (paging_handle_swap_fault), and nothing else. */
    if (PROCESS_ABI_IS_COMPAT32(abi_mode)) {
        return range_within(addr, len, USER32_WINDOW_BASE, USER32_WINDOW_LIMIT);
    }

    if (PROCESS_ABI_IS_LINUX(abi_mode)) {
        if (addr < USER_FOREIGN_BASE) {
            return 0;
        }
        /* One window over everything below the top of the stack area rather
         * than "code/heap" plus "this thread's own stack": the page table is
         * the real authority, and splitting it that way rejected any pointer
         * into another thread's stack. Chromium hit this on its first
         * pthread_create -- the new thread's FUTEX_WAKE and prctl(PR_SET_NAME)
         * both address the creator's stack, both came back EFAULT, and the
         * browser then sat in poll(-1) forever waiting for a thread that had
         * failed to hand itself over. */
        if (range_within(addr, len, USER_FOREIGN_BASE, USER_STACK_TOP)) {
            if (paging_is_user_range_mapped(process_cr3, addr, len)) {
                return 1;
            }
            return process_user_range_absent_file_backed(process_cr3, addr, len);
        }
        return 0;
    }

    if (range_within(addr, len, code_base, code_limit)) {
        return paging_is_user_range_mapped(process_cr3, addr, len);
    }
    if (range_within(addr, len, heap_base, heap_limit)) {
        return paging_is_user_range_mapped(process_cr3, addr, len);
    }
    if (range_within(addr, len, stack_base, stack_top)) {
        return paging_is_user_range_mapped(process_cr3, addr, len);
    }
    if (owner_stack_top != stack_top &&
        range_within(addr, len, owner_stack_base, owner_stack_top)) {
        return paging_is_user_range_mapped(process_cr3, addr, len);
    }
    return 0;
}

int process_user_cstring_length(const char *str, uint64_t max_len, uint64_t *len_out)
{
    if (str == NULL || max_len == 0) {
        return -1;
    }

    uint64_t current_page = (uint64_t)(uintptr_t)str & ~0xFFFULL;
    if (!process_user_buffer_is_valid((const void *)(uintptr_t)current_page, 1)) {
        return -1;
    }

    for (uint64_t i = 0; i < max_len; ++i) {
        uint64_t addr = (uint64_t)(uintptr_t)&str[i];
        if ((addr & ~0xFFFULL) != current_page) {
            current_page = addr & ~0xFFFULL;
            if (!process_user_buffer_is_valid((const void *)(uintptr_t)current_page, 1)) {
                return -1;
            }
        }

        char ch = '\0';
        if (copy_from_user(&ch, &str[i], 1u) != 0u) {
            return -1;
        }

        if (ch == '\0') {
            if (len_out != NULL) {
                *len_out = i;
            }
            return 0;
        }
    }
    return -1;
}

void *process_user_alloc(uint64_t size)
{
    if (size == 0) {
        return NULL;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }

    process_t *proc =
        process_memory_owner_locked(&g_processes[current_pid_get()]);
    if (proc == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }
    if (proc->user_heap_base == 0 ||
        proc->user_heap_limit <= proc->user_heap_base ||
        proc->user_heap_alloc_limit <= proc->user_heap_base ||
        proc->user_heap_cursor < proc->user_heap_base ||
        proc->user_heap_cursor > proc->user_heap_alloc_limit) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }
    /* Page granularity, not 16 bytes: every allocation is backed by page
     * mappings (paging_map_user_range_alloc() maps whole pages and zeroes
     * them). Two allocations whose 16-byte-aligned boundaries share a page
     * would fight over that page: mapping the second one allocated a fresh
     * frame and replaced the first one's last page with zeros, and freeing
     * either one unmapped the page under the other. Sizes that are not page
     * multiples (tmpfs shared mappings use the file's exact size) walked the
     * bump cursor off page alignment, and the next shared mapping then
     * clobbered the previous object's tail page -- Chromium's buffers came
     * back zeroed and a return address read back as garbage (RIP=0x4246).
     * Keeping the cursor and every slot page-aligned makes the page table
     * and the allocator describe the same ranges. */
    uint64_t alloc_size = 0;
    if (align_up_u64_checked((uint64_t)size, PAGE_SIZE, &alloc_size) < 0 ||
        alloc_size == 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }

    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        user_alloc_t *slot = &proc->user_allocs[i];
        if (!slot->used && slot->size != 0 && slot->size >= alloc_size) {
            slot->used = 1;
            uint64_t addr = slot->addr;
            uint64_t slot_size = slot->size;
            uint64_t cr3 = proc->cr3;
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            if (paging_map_user_range_alloc(cr3, addr, alloc_size, PAGE_RW | PAGE_USER) < 0) {
                irq_flags = irq_save_disable();
                spinlock_lock(&g_process_table_lock);
                if (is_valid_pid(current_pid_get())) {
                    process_t *rollback_proc =
                        process_memory_owner_locked(
                            &g_processes[current_pid_get()]);
                    if (rollback_proc == NULL) {
                        spinlock_unlock(&g_process_table_lock);
                        irq_restore(irq_flags);
                        return NULL;
                    }
                    for (uint32_t j = 0; j < PROCESS_USER_ALLOC_MAX; ++j) {
                        if (rollback_proc->user_allocs[j].addr == addr &&
                            rollback_proc->user_allocs[j].size == slot_size) {
                            rollback_proc->user_allocs[j].used = 0;
                            break;
                        }
                    }
                }
                spinlock_unlock(&g_process_table_lock);
                irq_restore(irq_flags);
                return NULL;
            }
            return (void *)(uintptr_t)addr;
        }
    }

    uint32_t new_slot = PROCESS_USER_ALLOC_MAX;
    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        if (proc->user_allocs[i].size == 0) {
            new_slot = i;
            break;
        }
    }
    if (new_slot == PROCESS_USER_ALLOC_MAX) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }

    uint64_t addr = 0;
    if (align_up_u64_checked(proc->user_heap_cursor, PAGE_SIZE, &addr) < 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }
    uint64_t next = addr + alloc_size;
    if (next <= addr || next > proc->user_heap_alloc_limit) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }

    uint64_t cr3 = proc->cr3;
    proc->user_heap_cursor = next;
    proc->user_allocs[new_slot].used = 1;
    proc->user_allocs[new_slot].addr = addr;
    proc->user_allocs[new_slot].size = alloc_size;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (paging_map_user_range_alloc(cr3, addr, alloc_size, PAGE_RW | PAGE_USER) < 0) {
        irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        if (is_valid_pid(current_pid_get())) {
            process_t *rollback_proc =
                process_memory_owner_locked(&g_processes[current_pid_get()]);
            if (rollback_proc == NULL) {
                spinlock_unlock(&g_process_table_lock);
                irq_restore(irq_flags);
                return NULL;
            }
            user_alloc_t *rollback_slot = &rollback_proc->user_allocs[new_slot];
            if (rollback_proc->cr3 == cr3 &&
                rollback_slot->used &&
                rollback_slot->addr == addr &&
                rollback_slot->size == alloc_size) {
                rollback_slot->used = 0;
                rollback_slot->addr = 0;
                rollback_slot->size = 0;
                if (rollback_proc->user_heap_cursor == next) {
                    rollback_proc->user_heap_cursor = addr;
                }
            }
        }
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return NULL;
    }

    return (void *)(uintptr_t)addr;
}

int process_user_free(void *ptr)
{
    if (ptr == NULL) {
        return 0;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *proc =
        process_memory_owner_locked(&g_processes[current_pid_get()]);
    if (proc == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    uint64_t addr = (uint64_t)(uintptr_t)ptr;

    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        user_alloc_t *slot = &proc->user_allocs[i];
        if (slot->used && slot->addr == addr) {
            uint64_t cr3 = proc->cr3;
            uint64_t size = slot->size;
            /* Keep the slot marked used while its pages are being torn down:
             * process_user_alloc() reuses !used slots, and clearing the flag
             * first let another thread map a fresh object at this address,
             * which this unmap then pulled out from under it. */
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            (void)paging_unmap_range(cr3, addr, size);
            irq_flags = irq_save_disable();
            spinlock_lock(&g_process_table_lock);
            if (is_valid_pid(current_pid_get())) {
                process_t *owner =
                    process_memory_owner_locked(&g_processes[current_pid_get()]);
                if (owner != NULL) {
                    for (uint32_t j = 0; j < PROCESS_USER_ALLOC_MAX; ++j) {
                        user_alloc_t *candidate = &owner->user_allocs[j];
                        if (candidate->used && candidate->addr == addr &&
                            candidate->size == size) {
                            candidate->used = 0;
                            break;
                        }
                    }
                }
            }
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            return 0;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return -1;
}

int process_user_munmap(void *ptr, uint64_t length)
{
    if (ptr == NULL || length == 0) {
        return 0;
    }

    uint64_t start = (uint64_t)(uintptr_t)ptr & PAGE_MASK;
    uint64_t end = ((uint64_t)(uintptr_t)ptr + length + PAGE_SIZE - 1ULL) & PAGE_MASK;
    if (end <= start) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *proc =
        process_memory_owner_locked(&g_processes[current_pid_get()]);
    if (proc == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    uint64_t cr3 = proc->cr3;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    int result = paging_unmap_range(cr3, start, end - start);

    /* Clear the slots only after their pages are gone. Clearing first let
     * process_user_alloc() reuse a slot while the old range was still being
     * unmapped, so the fresh mapping was torn down instead. */
    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        process_t *owner =
            process_memory_owner_locked(&g_processes[current_pid_get()]);
        /* Linux i386: the address space is the window's to hand out again. */
        if (owner != NULL && owner->compat_vm_map != NULL) {
            uint64_t lo = start < USER32_MMAP_BASE ? USER32_MMAP_BASE : start;
            uint64_t hi = end > USER32_MMAP_LIMIT ? USER32_MMAP_LIMIT : end;
            if (hi > lo) {
                compat32_vm_mark(owner->compat_vm_map,
                                 (lo - USER32_MMAP_BASE) / PAGE_SIZE,
                                 (hi - lo) / PAGE_SIZE, 0);
            }
        }
        if (owner != NULL) {
            for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
                user_alloc_t *slot = &owner->user_allocs[i];
                if (slot->size == 0) {
                    continue;
                }
                uint64_t slot_start = slot->addr;
                uint64_t slot_end = slot->addr + slot->size;
                if (slot_start >= start && slot_end <= end) {
                    slot->used = 0;
                }
            }
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return result;
}

uint64_t process_get_heap_cursor(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    uint64_t cursor = 0;
    if (is_valid_pid(current_pid_get())) {
        cursor = g_processes[current_pid_get()].user_heap_cursor;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return cursor;
}

uint64_t process_get_brk(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    uint64_t brk = 0;
    if (is_valid_pid(current_pid_get())) {
        process_t *proc =
            process_memory_owner_locked(&g_processes[current_pid_get()]);
        if (proc != NULL) {
            brk = proc->user_brk_cursor;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return brk;
}

/* Move the program break to `addr`, mapping exactly the pages between the old
 * break and the new one. Returns 0 on success.
 *
 * The whole point is that this maps *where the caller asked*, not wherever an
 * allocator felt like: glibc computes chunk addresses from the value brk()
 * reports and never re-reads it, so a break that lands somewhere else is
 * silently handing malloc memory that belongs to someone else. Shrinking just
 * moves the cursor back; the pages stay mapped, which costs a little memory
 * and avoids tearing down mappings the caller may still be using. */
int process_set_brk(uint64_t addr)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    process_t *proc =
        process_memory_owner_locked(&g_processes[current_pid_get()]);
    if (proc == NULL || proc->user_brk_base == 0u ||
        addr < proc->user_brk_base || addr > proc->user_brk_limit) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    uint64_t old_brk = proc->user_brk_cursor;
    uint64_t cr3 = proc->cr3;
    if (addr <= old_brk) {
        proc->user_brk_cursor = addr;
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }

    /* Only whole pages that are not already mapped need backing: a grow that
     * stays inside the page the old break sat in has nothing to do. */
    uint64_t map_start = (old_brk + PAGE_SIZE - 1u) & ~(PAGE_SIZE - 1u);
    uint64_t map_end = (addr + PAGE_SIZE - 1u) & ~(PAGE_SIZE - 1u);
    proc->user_brk_cursor = addr;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (map_end > map_start &&
        paging_map_user_range_alloc(cr3, map_start, map_end - map_start,
                                    PAGE_RW | PAGE_USER) < 0) {
        irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        if (is_valid_pid(current_pid_get())) {
            process_t *rollback =
                process_memory_owner_locked(&g_processes[current_pid_get()]);
            if (rollback != NULL && rollback->cr3 == cr3 &&
                rollback->user_brk_cursor == addr) {
                rollback->user_brk_cursor = old_brk;
            }
        }
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    return 0;
}

int process_set_heap_cursor(uint64_t addr)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    process_t *proc =
        process_memory_owner_locked(&g_processes[current_pid_get()]);
    uint64_t grow = 0;
    if (proc != NULL &&
        addr >= proc->user_heap_base &&
        addr <= proc->user_heap_alloc_limit) {
        if (addr > proc->user_heap_cursor) {
            grow = addr - proc->user_heap_cursor;
        } else {
            proc->user_heap_cursor = addr;
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (grow != 0u) {
        if (process_user_alloc((uint32_t)grow) == NULL) {
            return -1;
        }
    }
    return 0;
}

void process_set_thread_user_rsp(int32_t tid, uint64_t user_rsp)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (is_valid_pid(tid)) {
        g_processes[tid].saved_user_rsp = user_rsp;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
}


uint64_t process_signal_set_handler(int32_t signum, uint64_t handler)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return (uint64_t)-1;
    }
    if (signum <= 0 || signum >= PROCESS_SIGNAL_MAX) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return (uint64_t)-1;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (handler > 1u &&
        !process_user_buffer_is_valid((const void *)(uintptr_t)handler, 1)) {
        return (uint64_t)-1;
    }

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return (uint64_t)-1;
    }
    process_t *proc =
        process_memory_owner_locked(&g_processes[current_pid_get()]);
    if (proc == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return (uint64_t)-1;
    }
    uint64_t previous = proc->signal_handlers[(uint32_t)signum];
    proc->signal_handlers[(uint32_t)signum] = handler;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return previous;
}

uint64_t process_signal_get_handler(int32_t signum)
{
    if (signum <= 0 || signum >= PROCESS_SIGNAL_MAX) return (uint64_t)-1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = NULL;
    if (is_valid_pid(current_pid_get())) {
        proc = process_memory_owner_locked(&g_processes[current_pid_get()]);
    }
    uint64_t handler = proc != NULL ?
        proc->signal_handlers[(uint32_t)signum] : (uint64_t)-1;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return handler;
}

/* Extended sigaction(2) setter carrying sa_flags/sa_mask/sa_restorer
 * alongside the handler (TODO_Chromium_LinuxABI.md 3.5). Also reports
 * the previous full disposition via the out params (all optional). */
int process_signal_set_handler_ex(int32_t signum, uint64_t handler,
                                  uint64_t flags, uint64_t mask,
                                  uint64_t restorer,
                                  uint64_t *old_handler_out,
                                  uint64_t *old_flags_out,
                                  uint64_t *old_mask_out,
                                  uint64_t *old_restorer_out)
{
    if (signum <= 0 || signum >= PROCESS_SIGNAL_MAX) {
        return -1;
    }
    if (handler > 1u &&
        !process_user_buffer_is_valid((const void *)(uintptr_t)handler, 1)) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    process_t *proc =
        process_memory_owner_locked(&g_processes[current_pid_get()]);
    if (proc == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    if (old_handler_out != NULL) *old_handler_out = proc->signal_handlers[(uint32_t)signum];
    if (old_flags_out != NULL) *old_flags_out = proc->signal_flags[(uint32_t)signum];
    if (old_mask_out != NULL) *old_mask_out = proc->signal_sa_mask[(uint32_t)signum];
    if (old_restorer_out != NULL) *old_restorer_out = proc->signal_restorer[(uint32_t)signum];
    proc->signal_handlers[(uint32_t)signum] = handler;
    proc->signal_flags[(uint32_t)signum] = flags;
    proc->signal_sa_mask[(uint32_t)signum] = mask;
    proc->signal_restorer[(uint32_t)signum] = restorer;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

/* sigaltstack(2) (TODO_Chromium_LinuxABI.md 3.5). `new_*` may be NULL to
 * only query the current altstack into `old_*`. Returns -1 if an
 * attempt is made to change the altstack while currently executing on
 * it (matches Linux EPERM), or if `new_size` is absurdly small while
 * enabling. Altstack is per-thread (see the thread-creation copy site
 * above, which resets rather than inherits it). */
int process_sigaltstack(uint64_t new_sp, uint64_t new_size, uint32_t new_flags,
                        int has_new, uint64_t *old_sp_out,
                        uint64_t *old_size_out, uint32_t *old_flags_out)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    process_t *proc = &g_processes[current_pid_get()];

    int on_stack = (proc->altstack_flags & LINUX_SS_DISABLE) == 0u &&
                   proc->saved_user_rsp >= proc->altstack_sp &&
                   proc->saved_user_rsp < proc->altstack_sp + proc->altstack_size;

    if (old_sp_out != NULL) *old_sp_out = proc->altstack_sp;
    if (old_size_out != NULL) *old_size_out = proc->altstack_size;
    if (old_flags_out != NULL) {
        *old_flags_out = proc->altstack_flags |
            (uint32_t)(on_stack ? LINUX_SS_ONSTACK : 0u);
    }

    if (has_new) {
        if (on_stack) {
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            return -1;
        }
        if ((new_flags & LINUX_SS_DISABLE) != 0u) {
            proc->altstack_flags = LINUX_SS_DISABLE;
            proc->altstack_sp = 0;
            proc->altstack_size = 0;
        } else {
            if (new_size < 2048u ||
                !process_user_buffer_is_valid((void *)(uintptr_t)new_sp, new_size)) {
                spinlock_unlock(&g_process_table_lock);
                irq_restore(irq_flags);
                return -1;
            }
            proc->altstack_sp = new_sp;
            proc->altstack_size = new_size;
            proc->altstack_flags = 0;
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

uint64_t process_signal_get_mask(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    uint64_t mask = 0;
    if (is_valid_pid(current_pid_get())) {
        mask = g_processes[current_pid_get()].signal_mask;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return mask;
}

int process_signal_set_mask(uint64_t mask)
{
    const uint64_t unmaskable =
        (1ULL << (9u - 1u)) | (1ULL << (19u - 1u));
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    g_processes[current_pid_get()].signal_mask = mask & ~unmaskable;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

/* signal(7) default disposition: 1 = terminate the process, 0 = ignore/stop.
 * Only "terminate" needs acting on when no user handler is installed. */
static int process_signal_default_terminates(int32_t signum)
{
    switch (signum) {
        case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
        case 9: case 11: case 13: case 14: case 15: case 24: case 25:
        case 26: case 27: case 29: case 30: case 31:
            return 1;
        default:
            return 0;
    }
}

/* If `signum` was just raised at the current process and its disposition is
 * SIG_DFL with a default action of "terminate", exit now. Without this,
 * glibc's abort()/assert()/__stack_chk_fail() -- which raise(SIGABRT) and
 * expect to die -- return from the syscall and fall through to a privileged
 * `hlt` (=> #GP). Returns 1 if the process was terminated. Must be called
 * from syscall context with no locks held. */
int process_signal_maybe_self_terminate(int32_t signum)
{
    if (!process_signal_default_terminates(signum)) {
        return 0;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int is_default = 1;
    if (is_valid_pid(current_pid_get())) {
        process_t *owner =
            process_memory_owner_locked(&g_processes[current_pid_get()]);
        /* signal_handlers[]: 0 = SIG_DFL, 1 = SIG_IGN, else user handler. */
        if (owner != NULL && owner->signal_handlers[signum] != 0u) {
            is_default = 0;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    if (!is_default) {
        return 0;
    }
    process_exit_current_signaled(signum);
    return 1;
}

static int process_signal_blocked_locked(const process_t *proc, int32_t signum);

int process_signal_deliver(int32_t pid, int32_t signum)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    if (signum <= 0 || signum >= PROCESS_SIGNAL_MAX) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *proc = &g_processes[pid];
    if (proc->state == PROCESS_STATE_UNUSED ||
        proc->state == PROCESS_STATE_DEAD ||
        proc->state == PROCESS_STATE_ZOMBIE) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    proc->pending_signals |= (1u << (uint32_t)signum);
    /* A thread asleep somewhere other than a poll park -- a futex wait, a
     * timed sleep -- has to be woken too, or the signal waits until whatever
     * it sleeps on happens on its own. wineserver interrupts a client thread
     * with SIGUSR1 to make it run a system APC and then waits for the
     * result; a thread parked in a pthread condvar never took the signal,
     * and the requester (and everything waiting on it) hung. Woken, the
     * thread takes the signal on its way back to user space and then
     * re-enters its wait (Linux futex/sleep waits restart here). */
    int wake_blocked = proc->state == PROCESS_STATE_BLOCKED &&
                       !process_signal_blocked_locked(proc, signum);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    if (wake_blocked) {
        (void)process_wake_pid(pid);
    }
    /* A signal is not an fd event, and nothing else would tell a process
     * parked in poll()/epoll_wait()/ppoll() that one is waiting for it: it
     * would otherwise sit out its whole park slice before coming back to
     * userspace to take the signal. The park slice is a ceiling on an idle
     * wait (Syscall_Epoll.c), and this is what keeps that ceiling from
     * becoming the latency of every EINTR and every SIGCHLD. After the
     * unlock, because poll_wait_notify() ends by taking the process table
     * lock through process_wake_pid(). */
    poll_wait_notify();
    return 0;
}

static int process_signal_blocked_locked(const process_t *proc, int32_t signum)
{
    if (signum == 9 || signum == 19) {
        return 0;
    }
    return (proc->signal_mask & (1ULL << ((uint32_t)signum - 1u))) != 0u;
}

int process_signal_validate_group(int32_t tgid, int32_t tid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int valid = 0;
    if (is_valid_pid(tgid) && is_valid_pid(tid)) {
        process_t *thread = &g_processes[tid];
        if (thread->state != PROCESS_STATE_UNUSED &&
            thread->state != PROCESS_STATE_DEAD &&
            thread->state != PROCESS_STATE_ZOMBIE) {
            process_t *owner = process_memory_owner_locked(thread);
            if (owner != NULL &&
                (int32_t)(owner - g_processes) == tgid) {
                valid = 1;
            }
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return valid;
}

/* ---- process groups and sessions ---------------------------------- */

static process_t *pg_owner_locked(int32_t pid)
{
    if (!is_valid_pid(pid)) return NULL;
    process_t *p = &g_processes[pid];
    if (p->state == PROCESS_STATE_UNUSED || p->state == PROCESS_STATE_DEAD) {
        return NULL;
    }
    return process_memory_owner_locked(p);
}

/* pid 0: the caller. -1 when there is no such process. */
int32_t process_get_pgid(int32_t pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *p = pg_owner_locked(pid == 0 ? current_pid_get() : pid);
    int32_t r = -1;
    if (p != NULL) r = p->pgid >= 0 ? p->pgid : (int32_t)(p - g_processes);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return r;
}

int32_t process_get_sid(int32_t pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *p = pg_owner_locked(pid == 0 ? current_pid_get() : pid);
    int32_t r = -1;
    if (p != NULL) r = p->sid >= 0 ? p->sid : (int32_t)(p - g_processes);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return r;
}

/* setpgid(2): pid 0 is the caller, pgid 0 means "pid's own". The target must
 * be the caller or a child of it, in the caller's session. */
int32_t process_set_pgid(int32_t pid, int32_t pgid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *self = pg_owner_locked(current_pid_get());
    process_t *p = pg_owner_locked(pid == 0 ? current_pid_get() : pid);
    int32_t r = 0;
    if (self == NULL || p == NULL) {
        r = -3; /* ESRCH */
    } else if (p != self && p->parent_pid != (int32_t)(self - g_processes)) {
        r = -3;
    } else if (pgid < 0) {
        r = -22; /* EINVAL */
    } else {
        int32_t target = (int32_t)(p - g_processes);
        p->pgid = (pgid == 0) ? target : pgid;
        if (p->sid < 0) p->sid = self->sid >= 0 ? self->sid : target;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return r;
}

/* setsid(2). Linux refuses a process-group leader (EPERM); here a program
 * spawned from the desktop already leads both its group and its session,
 * and refusing it would only break programs that call setsid() "to be
 * sure", so a leader simply becomes the leader of a new session. */
int32_t process_setsid(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *self = pg_owner_locked(current_pid_get());
    int32_t r = -1;
    if (self != NULL) {
        r = (int32_t)(self - g_processes);
        self->pgid = r;
        self->sid = r;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return r;
}

/* kill(2) with pid <= 0: 0 = the caller's group, -1 = every process of the
 * caller's session (not the whole system -- the desktop, its services and
 * other programs' processes are not the caller's to signal), < -1 = group
 * -pid. Returns 0 if at least one process was signalled, -1 otherwise. */
int process_signal_deliver_pgrp(int32_t pid, int32_t signum)
{
    int32_t self = current_pid_get();
    int32_t own_pgid = process_get_pgid(0);
    int32_t own_sid = process_get_sid(0);
    int32_t targets[OS_CONFIG_PROCESS_MAX_COUNT];
    int32_t count = 0;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *me = pg_owner_locked(self);
    for (int32_t i = 1; i < g_process_capacity &&
                        count < (int32_t)OS_CONFIG_PROCESS_MAX_COUNT; ++i) {
        process_t *p = &g_processes[i];
        if (p->is_thread || p->state == PROCESS_STATE_UNUSED ||
            p->state == PROCESS_STATE_DEAD || p->state == PROCESS_STATE_ZOMBIE) {
            continue;
        }
        bool match;
        if (pid == 0) {
            match = p->pgid == own_pgid;
        } else if (pid == -1) {
            match = p->sid == own_sid && p != me;
        } else {
            match = p->pgid == -pid;
        }
        if (match) targets[count++] = i;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    int delivered = 0;
    for (int32_t i = 0; i < count; ++i) {
        if (process_signal_deliver_group(targets[i], signum) == 0) delivered = 1;
    }
    return delivered ? 0 : -1;
}

/* Signal every live process whose session or process group is `sid`.
 * 0 if at least one was signalled (or, for signum 0, exists). */
int process_signal_deliver_session(int32_t sid, int32_t signum)
{
    if (sid <= 0) return -1;
    int32_t targets[OS_CONFIG_PROCESS_MAX_COUNT];
    int32_t count = 0;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    for (int32_t i = 1; i < g_process_capacity &&
                        count < (int32_t)OS_CONFIG_PROCESS_MAX_COUNT; ++i) {
        process_t *p = &g_processes[i];
        if (p->is_thread || p->state == PROCESS_STATE_UNUSED ||
            p->state == PROCESS_STATE_DEAD || p->state == PROCESS_STATE_ZOMBIE) {
            continue;
        }
        if (p->sid == sid || p->pgid == sid) targets[count++] = i;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    if (count == 0) return -1;
    if (signum == 0) return 0;
    int delivered = 0;
    for (int32_t i = 0; i < count; ++i) {
        if (process_signal_deliver_group(targets[i], signum) == 0) delivered = 1;
    }
    return delivered ? 0 : -1;
}

int process_signal_deliver_group(int32_t pid, int32_t signum)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (pid == 0) {
        pid = current_pid_get();
        if (is_valid_pid(pid)) {
            process_t *owner =
                process_memory_owner_locked(&g_processes[pid]);
            if (owner != NULL) {
                pid = (int32_t)(owner - g_processes);
            }
        }
    }
    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *leader = &g_processes[pid];
    if (leader->state == PROCESS_STATE_UNUSED ||
        leader->state == PROCESS_STATE_DEAD ||
        leader->state == PROCESS_STATE_ZOMBIE) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    leader = process_memory_owner_locked(leader);
    if (leader == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    int32_t leader_pid = (int32_t)(leader - g_processes);

    if (signum == 0) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }
    if (signum < 0 || signum >= PROCESS_SIGNAL_MAX) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    int32_t wake_target = -1;
    if (!process_signal_blocked_locked(leader, signum)) {
        leader->pending_signals |= (1u << (uint32_t)signum);
        wake_target = leader_pid;
    } else {
        int32_t target = leader_pid;
        for (int32_t i = 1; i < g_process_capacity; ++i) {
            process_t *thread = &g_processes[i];
            if (thread->state == PROCESS_STATE_UNUSED ||
                thread->state == PROCESS_STATE_DEAD ||
                thread->state == PROCESS_STATE_ZOMBIE) {
                continue;
            }
            if (!thread->is_thread || thread->memory_owner_pid != leader_pid) {
                continue;
            }
            if (!process_signal_blocked_locked(thread, signum)) {
                target = i;
                break;
            }
        }
        g_processes[target].pending_signals |= (1u << (uint32_t)signum);
        wake_target = target;
    }
    int wake_blocked = wake_target >= 0 &&
                       g_processes[wake_target].state == PROCESS_STATE_BLOCKED &&
                       !process_signal_blocked_locked(&g_processes[wake_target], signum);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    /* See process_signal_deliver(): a blocked thread must take it now. */
    if (wake_blocked) {
        (void)process_wake_pid(wake_target);
    }
    /* Same reason as process_signal_deliver(): a parked waiter has to be
     * told, or the signal waits out its park slice. Done outside the table
     * lock -- poll_wait_notify() takes it again in process_wake_pid(). */
    poll_wait_notify();
    return 0;
}

int process_is_current_thread(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int is_thread = is_valid_pid(current_pid_get()) &&
                    g_processes[current_pid_get()].is_thread;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return is_thread;
}

int process_is_guard_page_fault(uint64_t fault_addr)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }

    const process_t *proc = &g_processes[current_pid_get()];
    uint64_t fault_page = fault_addr & PAGE_MASK;
    int is_guard = (fault_page == proc->user_heap_guard_page ||
                    fault_page == proc->user_stack_guard_page);
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return is_guard;
}

process_capability_mask_t process_default_capabilities(void)
{
    return PROCESS_CAP_DEFAULT_MASK;
}

process_capability_mask_t process_get_current_capabilities(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(current_pid_get())) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }
    process_capability_mask_t mask = g_processes[current_pid_get()].capability_mask;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return mask;
}

int process_current_has_capability(process_capability_mask_t capability)
{
    if (capability == 0) {
        return 1;
    }

    process_capability_mask_t current = process_get_current_capabilities();
    return ((current & capability) == capability);
}

int process_set_capabilities(int32_t pid, process_capability_mask_t capabilities)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    g_processes[pid].capability_mask = capabilities & PROCESS_CAP_DEFAULT_MASK;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int process_get_capabilities(int32_t pid, process_capability_mask_t *capabilities_out)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(pid) || capabilities_out == NULL) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    *capabilities_out = g_processes[pid].capability_mask;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int process_set_priority(int32_t pid, uint8_t priority)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    g_processes[pid].priority = priority;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int process_is_alive(int32_t pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int alive = 0;
    if (is_valid_pid(pid)) {
        uint8_t state = g_processes[pid].state;
        alive = (state != PROCESS_STATE_UNUSED && state != PROCESS_STATE_DEAD && state != PROCESS_STATE_ZOMBIE);
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return alive;
}

int process_sleep_current_ms(uint64_t ms)
{
    if (ms == 0u) {
        return 0;
    }

    uint64_t delay_ns;
    if (ms > (UINT64_MAX / 1000000ULL)) {
        delay_ns = UINT64_MAX;
    } else {
        delay_ns = ms * 1000000ULL;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t pid = current_pid_get();
    if (!is_valid_pid(pid) || g_sleep_deadline_ns == NULL ||
        g_processes[pid].state == PROCESS_STATE_UNUSED ||
        g_processes[pid].state == PROCESS_STATE_DEAD ||
        g_processes[pid].state == PROCESS_STATE_ZOMBIE) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    /* Deliberately does NOT consume the wake_pending credit that
     * process_block_current() consumes.
     *
     * The credit belongs to that call: a futex waiter checks its word, finds
     * it unchanged, and races towards process_block_current(), and a
     * FUTEX_WAKE landing in that window leaves a credit so the block is
     * skipped. Spending those credits here too was tried and was a mistake in
     * two directions -- it stole wakes the futex path was waiting for, and it
     * let futex traffic (which glibc generates constantly) short-circuit
     * unrelated sleeps, so a pipe or poll loop whose every sleep returned
     * immediately spun a CPU at 100%. A timed sleep has its own deadline and
     * does not need the credit; the waiters that want early wakeups register
     * with Core/syscall/Poll_Wait.h, which closes its own race without
     * touching this one.
     */
    uint64_t now_ns = process_perf_now_ns();
    g_sleep_deadline_ns[pid] = process_deadline_after_ns(now_ns, delay_ns);
    g_processes[pid].state = PROCESS_STATE_BLOCKED;
    g_processes[pid].block_count++;
    g_processes[pid].blocked_since_ns = now_ns;
    g_processes[pid].ready_since_ns = 0u;
    process_scheduler_request_reschedule();

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int process_block_current(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    int32_t pid = current_pid_get();
    if (!is_valid_pid(pid) ||
        g_processes[pid].state == PROCESS_STATE_UNUSED ||
        g_processes[pid].state == PROCESS_STATE_DEAD ||
        g_processes[pid].state == PROCESS_STATE_ZOMBIE) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    /* A wake that raced ahead of us (the waiter checked its condition,
     * found it unmet, and got here before actually flipping to BLOCKED)
     * left a credit in wake_pending. Consume it instead of sleeping: the
     * condition the caller is about to block on may already hold. This
     * check and the credit deposit in process_wake_pid() both happen
     * under g_process_table_lock, so the two can never race past each
     * other -- the lost-wakeup window is closed. */
    if (g_processes[pid].wake_pending > 0u) {
        g_processes[pid].wake_pending--;
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }

    uint64_t now_ns = process_perf_now_ns();
    if (g_sleep_deadline_ns != NULL) {
        g_sleep_deadline_ns[pid] = 0u;
    }
    g_processes[pid].state = PROCESS_STATE_BLOCKED;
    g_processes[pid].block_count++;
    g_processes[pid].blocked_since_ns = now_ns;
    g_processes[pid].ready_since_ns = 0u;
    process_scheduler_request_reschedule();
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int process_wake_pid(int32_t pid)
{
    int kick = 0;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (!is_valid_pid(pid)) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_t *proc = &g_processes[pid];
    if (g_sleep_deadline_ns != NULL) {
        g_sleep_deadline_ns[pid] = 0u;
    }
    if (proc->state == PROCESS_STATE_BLOCKED) {
        uint64_t now_ns = process_perf_now_ns();
        if (proc->blocked_since_ns != 0u && now_ns >= proc->blocked_since_ns) {
            proc->blocked_ns += now_ns - proc->blocked_since_ns;
        }
        proc->blocked_since_ns = 0u;
        proc->wake_count++;
        proc->state = PROCESS_STATE_READY;
        proc->wake_boost = 1u;
        process_perf_mark_ready_locked(proc, now_ns);
        process_scheduler_request_reschedule();
        kick = proc->rt_policy != 0u ? 2 : 1;
    } else if (proc->state != PROCESS_STATE_UNUSED &&
               proc->state != PROCESS_STATE_DEAD &&
               proc->state != PROCESS_STATE_ZOMBIE) {
        /* Target has not reached process_block_current() yet (it is still
         * racing towards it after failing its lock-free condition check).
         * Leave a credit so the upcoming block call short-circuits instead
         * of sleeping through this wake.
         *
         * Deliberately a flag rather than a counter. Callers only ever need
         * to know "at least one wake happened", and a counter is dangerous:
         * a wake aimed at a process that is awake (a stale poll registration,
         * a broadcast wake) banks a credit that is never spent, and a process
         * that has banked thousands consumes one per sleep and so stops
         * sleeping altogether -- a 100% CPU spin whose cause is nowhere near
         * where it shows up. */
        proc->wake_pending = 1u;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    /* The woken task is only READY; if a CPU is idle, wake it now instead of
     * leaving the task for whichever CPU next reaches the scheduler -- up to
     * a timer tick for an idle CPU, a whole slice for a busy one. Chromium's
     * audio renderer has ~10 ms to answer each request before the audio
     * service gives up on it and plays silence. */
    if (kick == 2) {
        process_scheduler_kick_for_rt(g_processes, g_process_capacity);
    } else if (kick) {
        process_scheduler_kick_idle_cpu();
    }
    return 0;
}

/* sched_setscheduler(2) for thread `tid` (a kernel tid). */
int process_set_rt_policy(int32_t tid, uint8_t policy, uint8_t priority)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(tid) || g_processes[tid].state == PROCESS_STATE_UNUSED) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3; /* ESRCH */
    }
    g_processes[tid].rt_policy = policy;
    g_processes[tid].rt_priority = priority;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int process_get_rt_policy(int32_t tid, uint8_t *policy, uint8_t *priority)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (!is_valid_pid(tid) || g_processes[tid].state == PROCESS_STATE_UNUSED) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }
    if (policy) *policy = g_processes[tid].rt_policy;
    if (priority) *priority = g_processes[tid].rt_priority;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int32_t process_waitpid(int32_t pid, int32_t *status_out, int32_t options)
{
    return process_waitpid_ex(pid, status_out, options, NULL);
}

/* Is the process, or any thread of it, still executing on some CPU? Reaping
 * frees the address space, so a zombie is only reapable once every one of its
 * threads is off-CPU. Checking the group leader alone let a parent reap a
 * multi-threaded child while one of its threads was still returning from its
 * own exit_group() on another CPU -- that thread came back to user space with
 * no process behind it and took a #GP on glibc's post-exit hlt. Caller holds
 * the table lock. */
static int process_group_on_cpu_locked(int32_t pid)
{
    if (process_scheduler_pid_in_use_on_any_cpu(pid)) {
        return 1;
    }
    for (int32_t i = 0; i < g_process_capacity; ++i) {
        const process_t *t = &g_processes[i];
        if (t->is_thread && t->memory_owner_pid == pid &&
            t->state != PROCESS_STATE_UNUSED &&
            process_scheduler_pid_in_use_on_any_cpu(i)) {
            return 1;
        }
    }
    return 0;
}

int32_t process_waitpid_ex(int32_t pid, int32_t *status_out, int32_t options,
                           int32_t *term_signal_out)
{
    (void)options;
    if (term_signal_out != NULL) {
        *term_signal_out = 0;
    }
    int32_t my_pid = process_get_current_pid();
    if (my_pid < 0) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (pid > 0) {
        if (!is_valid_pid(pid) || g_processes[pid].is_thread ||
            g_processes[pid].parent_pid != my_pid) {
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            return -1;
        }
        if (g_processes[pid].state == PROCESS_STATE_ZOMBIE &&
            !g_processes[pid].teardown_pending &&
            !process_group_on_cpu_locked(pid)) {
            int32_t exit_code = g_processes[pid].exit_status;
            int32_t term_sig = g_processes[pid].exit_by_signal
                                   ? (int32_t)g_processes[pid].exit_term_signal
                                   : 0;
            release_process_resources(&g_processes[pid]);
            reset_process_slot(&g_processes[pid]);
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            if (status_out != NULL) {
                *status_out = exit_code;
            }
            if (term_signal_out != NULL) {
                *term_signal_out = term_sig;
            }
            return pid;
        }
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
    }

    for (int32_t i = 0; i < g_process_capacity; ++i) {
        if (g_processes[i].state == PROCESS_STATE_ZOMBIE &&
            !g_processes[i].is_thread &&
            !g_processes[i].teardown_pending &&
            g_processes[i].parent_pid == my_pid &&
            !process_group_on_cpu_locked(i)) {
            int32_t exit_code = g_processes[i].exit_status;
            int32_t term_sig = g_processes[i].exit_by_signal
                                   ? (int32_t)g_processes[i].exit_term_signal
                                   : 0;
            int32_t child_pid = i;
            release_process_resources(&g_processes[i]);
            reset_process_slot(&g_processes[i]);
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            if (status_out != NULL) {
                *status_out = exit_code;
            }
            if (term_signal_out != NULL) {
                *term_signal_out = term_sig;
            }
            return child_pid;
        }
    }
    
    int has_children = 0;
    for (int32_t i = 0; i < g_process_capacity; ++i) {
        if (i != my_pid && g_processes[i].state != PROCESS_STATE_UNUSED &&
            !g_processes[i].is_thread &&
            g_processes[i].parent_pid == my_pid) {
            has_children = 1;
            break;
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    return has_children ? 0 : -1;
}

int32_t process_getppid(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t pid = current_pid_get();
    int32_t ppid = -1;
    if (is_valid_pid(pid)) {
        process_t *owner = process_memory_owner_locked(&g_processes[pid]);
        if (owner != NULL) {
            ppid = owner->parent_pid;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return ppid;
}

int32_t process_get_parent_pid(int32_t pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t ppid = -1;
    if (is_valid_pid(pid)) {
        ppid = g_processes[pid].parent_pid;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return ppid;
}
int32_t process_terminate(int32_t pid)
{
    if (pid < 0 || pid >= g_process_capacity) return -1;
    if (pid == current_pid_get()) {
        process_exit_current();
        return 0;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = &g_processes[pid];
    if (proc->state == PROCESS_STATE_UNUSED || proc->state == PROCESS_STATE_DEAD) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    proc->state = PROCESS_STATE_DEAD;
    if (!process_scheduler_pid_in_use_on_any_cpu(pid)) {
        release_process_resources(proc);
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int32_t process_get_full_info(int32_t pid, void *info_out)
{
    if (pid < 0 || pid >= g_process_capacity) return -1;
    
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = &g_processes[pid];
    if (proc->state == PROCESS_STATE_UNUSED) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    struct full_info {
        int32_t pid;
        int32_t parent_pid;
        uint8_t state;
        uint8_t reserved[7];
        char    name[64];
        uint64_t total_ticks;
        uint64_t memory_usage;
    } *info = info_out;

    info->pid = pid;
    info->parent_pid = proc->parent_pid;
    info->state = proc->state;
    /* reserved[0]: scheduling priority (0-3, see process_set_priority) so a
     * task manager can show what it would be changing. */
    info->reserved[0] = proc->priority;
    for (int i = 0; i < 64; i++) info->name[i] = proc->name[i];
    info->total_ticks = proc->total_ticks;
    
    uint64_t usage = 0;
    if (proc->user_heap_cursor > proc->user_heap_base) {
        usage += (proc->user_heap_cursor - proc->user_heap_base);
    }
    if (proc->user_stack_top > proc->user_stack_base) {
        usage += (proc->user_stack_top - proc->user_stack_base);
    }
    info->memory_usage = usage;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

static uint64_t process_memory_usage_locked(const process_t *proc)
{
    if (proc == NULL) {
        return 0;
    }
    uint64_t usage = 0;
    if (proc->user_heap_cursor > proc->user_heap_base) {
        usage += (proc->user_heap_cursor - proc->user_heap_base);
    }
    if (proc->user_stack_top > proc->user_stack_base) {
        usage += (proc->user_stack_top - proc->user_stack_base);
    }
    return usage;
}

static uint64_t g_memreport_last_ns;
static spinlock_t g_memreport_lock;

static void memreport_line(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

static void memreport_line(const char *fmt, ...)
{
    char line[224];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    serial_write_string(line);
}

#define MEMREPORT_KIB(pages) ((unsigned long long)((pages) * (PAGE_SIZE / 1024u)))
#define MEMREPORT_MIB(bytes) ((unsigned long long)((bytes) >> 20))

void process_debug_memory_report(const char *why, int force)
{
    const uint64_t now = timer_monotonic_ns();
    if (!force) {
        const uint64_t last = __atomic_load_n(&g_memreport_last_ns, __ATOMIC_RELAXED);
        if (last != 0u && now - last < 5000000000ull) {
            return;
        }
    }
    /* One report at a time: two CPUs failing together would interleave. */
    if (__atomic_exchange_n(&g_memreport_lock.value, 1u, __ATOMIC_ACQUIRE) != 0u) {
        return;
    }
    __atomic_store_n(&g_memreport_last_ns, now, __ATOMIC_RELAXED);

    uint32_t tmpfs_files = 0, eager_count = 0;
    uint64_t tmpfs_shared = 0, eager_bytes = 0;
    const uint64_t tmpfs_bytes = tmpfs_debug_bytes(&tmpfs_files, &tmpfs_shared);
    linux_mmap_debug_eager_stats(&eager_count, &eager_bytes);
    memreport_line("[mem] report (%s) at %llu ms: free %llu MiB of %llu MiB; "
                   "kernel heap used %llu MiB\n",
                   why != NULL ? why : "-", (unsigned long long)(now / 1000000ull),
                   MEMREPORT_MIB(memory_free_pages() * PAGE_SIZE),
                   MEMREPORT_MIB(get_total_memory_pages() * PAGE_SIZE),
                   MEMREPORT_MIB(get_used_memory()));
    memreport_line("[mem]   tmpfs %llu MiB in %u files (+%llu MiB mapped shared); "
                   "shm objects %llu MiB in %u\n",
                   MEMREPORT_MIB(tmpfs_bytes), tmpfs_files, MEMREPORT_MIB(tmpfs_shared),
                   MEMREPORT_MIB(shared_memory_debug_pages() * PAGE_SIZE),
                   shared_memory_object_count());
    uint32_t cache_unmapped = 0;
    const uint32_t cache_pages = filemap_debug_shared_pages(&cache_unmapped);
    memreport_line("[mem]   filemap %u/%u records; file maps read eagerly for want of "
                   "a record: %u (%llu MiB); shared file pages %llu MiB "
                   "(%llu MiB unmapped)\n",
                   filemap_debug_count(-1), filemap_debug_capacity(),
                   eager_count, MEMREPORT_MIB(eager_bytes),
                   MEMREPORT_MIB((uint64_t)cache_pages * PAGE_SIZE),
                   MEMREPORT_MIB((uint64_t)cache_unmapped * PAGE_SIZE));
    uint64_t rs[3];
    filemap_debug_read_stats(rs);
    memreport_line("[mem]   file faults %llu read %llu MiB in %llu ms; cow copies %llu\n",
                   (unsigned long long)rs[0], MEMREPORT_MIB(rs[1] * PAGE_SIZE),
                   (unsigned long long)(rs[2] / 1000000ull),
                   (unsigned long long)paging_debug_cow_copies());
    uint64_t cs[6];
    filemap_debug_cache_stats(cs);
    memreport_line("[mem]   file page cache: %llu hits, %llu inserts, %llu refused (full); "
                   "%llu sweeps (%llu ms) freed %llu\n",
                   (unsigned long long)cs[0], (unsigned long long)cs[1],
                   (unsigned long long)cs[2], (unsigned long long)cs[3],
                   (unsigned long long)(cs[4] / 1000000ull), (unsigned long long)cs[5]);

    uint64_t irq = irq_save_disable();
    if (__atomic_exchange_n(&g_process_table_lock.value, 1u, __ATOMIC_ACQUIRE) != 0u) {
        irq_restore(irq);
        serial_write_string("[mem]   (process table busy: no per-process breakdown)\n");
        __atomic_store_n(&g_memreport_lock.value, 0u, __ATOMIC_RELEASE);
        return;
    }
    uint64_t total_own = 0, total_shared = 0, total_tables = 0;
    uint32_t spaces = 0;
    for (int32_t i = 0; i < g_process_capacity; ++i) {
        const process_t *p = &g_processes[i];
        if (p->state == PROCESS_STATE_UNUSED || p->cr3 == 0u) {
            continue;
        }
        /* Threads share their leader's tables: one line per address space,
         * under the lowest pid holding it. */
        int seen = 0;
        uint32_t threads = 0;
        for (int32_t j = 0; j < g_process_capacity; ++j) {
            const process_t *q = &g_processes[j];
            if (q->state == PROCESS_STATE_UNUSED || q->cr3 != p->cr3) continue;
            if (j < i) { seen = 1; break; }
            ++threads;
        }
        if (seen) {
            continue;
        }
        uint64_t shared = 0, tables = 0;
        const uint64_t own = paging_debug_count_user_pages(p->cr3, &shared, &tables);
        total_own += own;
        total_shared += shared;
        total_tables += tables;
        ++spaces;
        memreport_line("[mem]   pid %3d %-24.24s st=%u thr=%2u rss %7llu KiB (shared %7llu) "
                       "tables %5llu KiB filemap %u\n",
                       (int)i, p->name, (unsigned)p->state, threads,
                       MEMREPORT_KIB(own), MEMREPORT_KIB(shared), MEMREPORT_KIB(tables),
                       filemap_debug_count(i));
    }
    __atomic_store_n(&g_process_table_lock.value, 0u, __ATOMIC_RELEASE);
    irq_restore(irq);
    memreport_line("[mem]   %u address spaces: rss %llu MiB (of it shared by fork or the "
                   "file page cache %llu MiB, counted once per sharer), page tables %llu MiB\n",
                   spaces, MEMREPORT_MIB(total_own * PAGE_SIZE),
                   MEMREPORT_MIB(total_shared * PAGE_SIZE),
                   MEMREPORT_MIB(total_tables * PAGE_SIZE));
    __atomic_store_n(&g_memreport_lock.value, 0u, __ATOMIC_RELEASE);
}

int32_t process_get_perf_info(int32_t pid, process_perf_info_t *info_out)
{
    if (info_out == NULL || pid < 0 || pid >= g_process_capacity) {
        return -1;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = &g_processes[pid];
    if (proc->state == PROCESS_STATE_UNUSED) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    uint64_t now_ns = process_perf_now_ns();
    uint64_t runtime_ns = proc->runtime_ns;
    if (proc->state == PROCESS_STATE_RUNNING &&
        proc->last_scheduled_ns != 0u &&
        now_ns >= proc->last_scheduled_ns) {
        runtime_ns += now_ns - proc->last_scheduled_ns;
    }
    uint64_t blocked_ns = proc->blocked_ns;
    if (proc->state == PROCESS_STATE_BLOCKED &&
        proc->blocked_since_ns != 0u &&
        now_ns >= proc->blocked_since_ns) {
        blocked_ns += now_ns - proc->blocked_since_ns;
    }

    memset(info_out, 0, sizeof(*info_out));
    info_out->pid = pid;
    info_out->parent_pid = proc->parent_pid;
    info_out->state = proc->state;
    memcpy(info_out->name, proc->name, sizeof(info_out->name));
    info_out->runtime_ns = runtime_ns;
    info_out->ready_wait_ns = proc->ready_wait_ns;
    info_out->max_ready_wait_ns = proc->max_ready_wait_ns;
    info_out->context_switches = proc->context_switches;
    info_out->voluntary_switches = proc->voluntary_switches;
    info_out->involuntary_switches = proc->involuntary_switches;
    info_out->syscalls = proc->syscalls;
    info_out->ipc_send = proc->ipc_send;
    info_out->ipc_recv = proc->ipc_recv;
    info_out->block_count = proc->block_count;
    info_out->wake_count = proc->wake_count;
    info_out->blocked_ns = blocked_ns;
    info_out->display_present_calls = proc->display_present_calls;
    info_out->display_rects = proc->display_rects;
    info_out->display_bytes = proc->display_bytes;
    info_out->memory_usage = process_memory_usage_locked(proc);
    info_out->page_fault_count = proc->page_fault_count;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

void process_perf_note_syscall(int32_t pid)
{
    if (is_valid_pid(pid)) {
        __atomic_fetch_add(&g_processes[pid].syscalls, 1u, __ATOMIC_RELAXED);
    }
}

void process_perf_note_ipc_send(int32_t pid)
{
    if (is_valid_pid(pid)) {
        __atomic_fetch_add(&g_processes[pid].ipc_send, 1u, __ATOMIC_RELAXED);
    }
}

void process_perf_note_ipc_recv(int32_t pid)
{
    if (is_valid_pid(pid)) {
        __atomic_fetch_add(&g_processes[pid].ipc_recv, 1u, __ATOMIC_RELAXED);
    }
}

void process_perf_note_display(int32_t pid, uint32_t rect_count, uint64_t bytes)
{
    if (is_valid_pid(pid)) {
        process_t *proc = &g_processes[pid];
        __atomic_fetch_add(&proc->display_present_calls, 1u, __ATOMIC_RELAXED);
        __atomic_fetch_add(&proc->display_rects, rect_count, __ATOMIC_RELAXED);
        __atomic_fetch_add(&proc->display_bytes, bytes, __ATOMIC_RELAXED);
    }
}

int32_t process_get_capacity(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t capacity = g_process_capacity;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return capacity;
}

int32_t process_count_threads(int32_t owner_pid)
{
    if (owner_pid < 0) {
        return -1;
    }
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (owner_pid >= g_process_capacity) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    int32_t count = 0;
    for (int32_t i = 0; i < g_process_capacity; ++i) {
        process_t *proc = &g_processes[i];
        if (proc->state == PROCESS_STATE_UNUSED ||
            proc->state == PROCESS_STATE_DEAD) {
            continue;
        }
        int32_t effective_owner = proc->is_thread ? proc->memory_owner_pid : i;
        if (effective_owner == owner_pid) {
            ++count;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return count;
}

/* Ascending walk of the slots belonging to one thread group; -1 to start,
 * -1 when done. /proc/<pid>/task is the caller: crash handling and any
 * per-thread accounting a Linux program does starts by listing that
 * directory. Uses the same "effective owner" rule as
 * process_count_threads(). */
int32_t process_next_thread_of(int32_t owner_pid, int32_t after)
{
    if (owner_pid < 0) {
        return -1;
    }
    int32_t start = (after < 0) ? 0 : after + 1;
    if (start < 0) {
        return -1;
    }
    int32_t found = -1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    for (int32_t i = start; i < g_process_capacity; ++i) {
        process_t *proc = &g_processes[i];
        if (proc->state == PROCESS_STATE_UNUSED ||
            proc->state == PROCESS_STATE_DEAD) {
            continue;
        }
        int32_t effective_owner = proc->is_thread ? proc->memory_owner_pid : i;
        if (effective_owner == owner_pid) {
            found = i;
            break;
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return found;
}

/* Ascending walk of every live process slot, used to enumerate /proc. */
int32_t process_next_live_pid(int32_t after)
{
    int32_t start = (after < 0) ? 0 : after + 1;
    if (start < 0) {
        return -1;
    }
    int32_t found = -1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    for (int32_t i = start; i < g_process_capacity; ++i) {
        uint8_t state = g_processes[i].state;
        if (state == PROCESS_STATE_UNUSED || state == PROCESS_STATE_DEAD ||
            state == PROCESS_STATE_ZOMBIE) {
            continue;
        }
        if (g_processes[i].is_thread) {
            continue; /* Threads appear under /proc/<tgid>/task, not at /proc. */
        }
        found = i;
        break;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return found;
}

void process_record_page_fault(int32_t pid, uint64_t fault_addr, uint64_t rip,
                               uint32_t error_code, int is_guard)
{
    if (pid < 0 || pid >= g_process_capacity) return;
    process_t *proc = &g_processes[pid];
    __atomic_fetch_add(&proc->page_fault_count, 1u, __ATOMIC_RELAXED);
    proc->last_pf_addr = fault_addr;
    proc->last_pf_rip = rip;
    proc->last_pf_error = error_code;
    proc->has_crashed = 1;
    if (is_guard) {
        strncpy(proc->crash_reason, "STACK_OVERFLOW", sizeof(proc->crash_reason) - 1);
    } else {
        strncpy(proc->crash_reason, "PAGE_FAULT", sizeof(proc->crash_reason) - 1);
    }
}

typedef struct {
    int32_t pid;
    int32_t parent_pid;
    uint8_t state;
    uint8_t reserved[7];
    char name[64];
    uint64_t page_fault_count;
    uint64_t last_pf_addr;
    uint64_t last_pf_rip;
    uint32_t last_pf_error;
    uint8_t has_crashed;
    char crash_reason[64];
    int32_t exit_status;
} process_debug_info_kernel_t;

typedef struct {
    uint64_t total_page_faults;
    uint32_t process_count;
    uint64_t uptime_ms;
    uint64_t total_memory;
    uint64_t used_memory;
} os_debug_info_kernel_t;

int32_t process_get_debug_info(int32_t pid, void *info_out)
{
    if (info_out == NULL || pid < 0 || pid >= g_process_capacity) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    process_t *proc = &g_processes[pid];
    if (proc->state == PROCESS_STATE_UNUSED) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }

    process_debug_info_kernel_t di = {0};
    di.pid = pid;
    di.parent_pid = proc->parent_pid;
    di.state = proc->state;
    memcpy(di.name, proc->name, sizeof(di.name));
    di.page_fault_count = proc->page_fault_count;
    di.last_pf_addr = proc->last_pf_addr;
    di.last_pf_rip = proc->last_pf_rip;
    di.last_pf_error = proc->last_pf_error;
    di.has_crashed = proc->has_crashed;
    memcpy(di.crash_reason, proc->crash_reason, sizeof(di.crash_reason));
    di.exit_status = proc->exit_status;

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    memcpy(info_out, &di, sizeof(di));
    return 0;
}

int32_t process_get_os_debug(void *info_out)
{
    if (info_out == NULL) return -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    os_debug_info_kernel_t od = {0};
    uint64_t now_ns = process_perf_now_ns();
    for (int32_t i = 0; i < g_process_capacity; ++i) {
        process_t *proc = &g_processes[i];
        if (proc->state == PROCESS_STATE_UNUSED) continue;
        od.total_page_faults += proc->page_fault_count;
        od.process_count++;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    od.uptime_ms = now_ns / 1000000ULL;
    od.total_memory = get_total_memory_pages() * 4096ULL;
    od.used_memory = get_used_memory();

    memcpy(info_out, &od, sizeof(od));
    return 0;
}
