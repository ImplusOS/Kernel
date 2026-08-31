#include "ProcessManager.h"
#include "ProcessScheduler.h"

#include "IPC/IPC_Main.h"
#include "IPC/PnP_Notifications.h"
#include "IPC/UnixSocket.h"
#include "Core/elf/ELF_Loader.h"
#include "Core/memory/SharedMemory.h"
#include "Core/memory/FileMap.h"
#include "Core/vfs/VFS.h"
#include "cpu/GDT_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "Core/sync/Spinlock.h"
#include "Core/syscall/Syscall_File.h"
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
#include <stdio.h>

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
};

static process_t *g_processes = NULL;
static int32_t g_process_capacity = 0;
static uint64_t *g_sleep_deadline_ns = NULL;
static spinlock_t g_process_table_lock;
static uint32_t g_timeslice_ticks = 6;
static uint64_t g_cpu_usage_prev_runtime_ns[OS_CONFIG_PROCESS_MAX_COUNT_MAX];
static uint64_t g_cpu_usage_prev_syscalls[OS_CONFIG_PROCESS_MAX_COUNT_MAX];
static uint64_t g_cpu_usage_prev_display_bytes[OS_CONFIG_PROCESS_MAX_COUNT_MAX];

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

static int is_valid_pid(int32_t pid)
{
    return process_table_ready() && pid >= 0 && pid < g_process_capacity;
}

static void save_syscall_frame_to_process(process_t *proc, uint64_t current_saved_rsp)
{
    if (proc == NULL || current_saved_rsp == 0) return;
    proc->saved_rsp = current_saved_rsp;
    proc->gs_base = hal_cpu_read_gs_base();
}

static void reset_process_slot(process_t *proc)
{
    if (proc == NULL) {
        return;
    }

    process_clear_sleep_deadline_for_proc_locked(proc);

    proc->state = PROCESS_STATE_UNUSED;
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

    proc->cr3 = 0;
    proc->kernel_stack_base = NULL;
    proc->kernel_stack_top = 0;
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
    memset(proc->exe_path, 0, sizeof(proc->exe_path));
    memset(proc->launch_argument, 0, sizeof(proc->launch_argument));
    proc->parent_pid = -1;
    proc->memory_owner_pid = -1;
    proc->exit_status = 0;
    proc->exit_by_signal = 0;
    proc->exit_term_signal = 0;
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
    if (proc->kernel_stack_base != NULL) {
        free(proc->kernel_stack_base);
        proc->kernel_stack_base = NULL;
    }
    proc->cr3 = 0;
    proc->thread_stack_region_base = 0;
    proc->thread_stack_region_size = 0;
    proc->kernel_stack_top = 0;
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

    if (owner_pid >= 0) {
        for (int32_t i = 1; i < g_process_capacity; ++i) {
            process_t *thread = &g_processes[i];
            if (thread != proc && thread->is_thread &&
                thread->memory_owner_pid == owner_pid &&
                thread->state != PROCESS_STATE_UNUSED) {
                if (process_scheduler_pid_in_use_on_any_cpu(i)) {
                    continue;
                }
                release_thread_resources(thread, 0);
                reset_process_slot(thread);
            }
        }
    }

    if (proc->cr3 != 0) {
        paging_destroy_process_space(proc->cr3);
        proc->cr3 = 0;
    }
    if (proc->kernel_stack_base != NULL) {
        free(proc->kernel_stack_base);
        proc->kernel_stack_base = NULL;
    }
    proc->kernel_stack_top = 0;
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
    uint64_t fpstate; /* always 0 here: no FPU/XSAVE state is captured */
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
static int write_signal_frame_locked(process_t *proc, int32_t signum,
                                     uint64_t handler,
                                     const linux_gpregs_t *regs,
                                     uint64_t old_rip, uint64_t old_rsp,
                                     uint64_t old_rflags, uint64_t si_addr,
                                     int32_t si_code,
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

    uint64_t sp = base - sizeof(linux_rt_sigframe_t);
    sp &= ~0xFULL;
    sp -= 8ULL; /* land at sp%16==8, matching "just after a call" ABI state */

    uint64_t stack_low = use_altstack ? proc->altstack_sp : proc->user_stack_base;
    uint64_t stack_high = use_altstack ?
        (proc->altstack_sp + proc->altstack_size) : proc->user_stack_top;
    if (sp < stack_low || sp + sizeof(linux_rt_sigframe_t) > stack_high) {
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
    mc->err = 0; mc->trapno = (uint64_t)(signum == LINUX_SIGSEGV ? 14 : 0);
    mc->oldmask = proc->signal_mask;
    mc->cr2 = (signum == LINUX_SIGSEGV) ? si_addr : 0;
    mc->fpstate = 0;

    memcpy(frame.uc.uc_sigmask, &proc->signal_mask, sizeof(proc->signal_mask));

    frame.info.si_signo = signum;
    frame.info.si_errno = 0;
    frame.info.si_code = si_code;
    frame.info.si_addr = si_addr;

    uint64_t old_cr3 = paging_get_active_cr3();
    paging_switch_cr3(proc->cr3);
    int copy_ok = process_user_buffer_is_valid((void *)(uintptr_t)sp,
                                               sizeof(frame));
    if (copy_ok) {
        memcpy((void *)(uintptr_t)sp, &frame, sizeof(frame));
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
        proc->exit_status = 128 + signum;
        proc->exit_by_signal = 1u;
        proc->exit_term_signal = (uint8_t)signum;
        proc->state = PROCESS_STATE_ZOMBIE;
        return -1;
    }

#if defined(__x86_64__)
    uint64_t *frame = (uint64_t *)(uintptr_t)proc->saved_rsp;
    if (frame == NULL || proc->saved_user_rsp < (proc->user_stack_base + 16ULL)) {
        proc->exit_status = 128 + signum;
        proc->exit_by_signal = 1u;
        proc->exit_term_signal = (uint8_t)signum;
        proc->state = PROCESS_STATE_ZOMBIE;
        return -1;
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
     * itself clobbers them with the return RIP/RFLAGS) - left as 0. */

    uint64_t old_rip = frame[SYSCALL_FRAME_RCX];
    uint64_t old_rflags = frame[SYSCALL_FRAME_R11];
    uint64_t old_rsp = proc->saved_user_rsp;

    uint64_t new_rip = 0, new_rsp = 0;
    if (write_signal_frame_locked(proc, signum, handler, &regs, old_rip,
                                  old_rsp, old_rflags, 0, 0 /* SI_USER */,
                                  &new_rip, &new_rsp) < 0) {
        proc->exit_status = 128 + signum;
        proc->exit_by_signal = 1u;
        proc->exit_term_signal = (uint8_t)signum;
        proc->state = PROCESS_STATE_ZOMBIE;
        return -1;
    }

    frame[SYSCALL_FRAME_RCX] = new_rip;
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
#else
    proc->exit_status = 128 + signum;
    proc->exit_by_signal = 1u;
    proc->exit_term_signal = (uint8_t)signum;
    proc->state = PROCESS_STATE_ZOMBIE;
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
            if (proc->state == PROCESS_STATE_ZOMBIE) {
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
int process_signal_deliver_fault_now(int32_t pid, int32_t signum,
                                     uint64_t fault_addr,
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
    if (handler == 0 || handler == 1u) {
        /* No handler (default = terminate) or explicitly ignored: SIGSEGV
         * cannot be ignored (matches Linux - an ignored SIGSEGV still
         * kills the process), so either way the caller should terminate. */
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return 0;
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
    int32_t si_code = 1; /* SEGV_MAPERR: we do not distinguish MAPERR/ACCERR. */
    if (write_signal_frame_locked(proc, signum, handler, &regs, old_rip,
                                  old_rsp, old_rflags, fault_addr, si_code,
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
    proc->pending_signals &= ~(1u << (uint32_t)signum);

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 1;
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
    frame[SYSCALL_FRAME_R11] = mc->eflags; /* R11 slot doubles as RFLAGS */
    syscall_set_user_rsp(mc->rsp);

    uint64_t restored_mask;
    memcpy(&restored_mask, uc.uc_sigmask, sizeof(restored_mask));
    const uint64_t unmaskable = (1ULL << (9u - 1u)) | (1ULL << (19u - 1u));

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    if (is_valid_pid(current_pid_get())) {
        g_processes[current_pid_get()].signal_mask = restored_mask & ~unmaskable;
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    return (int64_t)mc->rax;
#else
    (void)saved_rsp;
    return -1;
#endif
}

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
            !process_scheduler_pid_in_use_on_any_cpu(i)) {
            release_process_resources(&g_processes[i]);
            reset_process_slot(&g_processes[i]);
        } else if (g_processes[i].state == PROCESS_STATE_ZOMBIE &&
                   !process_scheduler_pid_in_use_on_any_cpu(i)) {
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

static void activate_process_context(process_t *proc)
{
    process_kstack_canary_check(proc);
    if (paging_get_active_cr3() != proc->cr3) {
        paging_switch_cr3(proc->cr3);
    }
    syscall_set_kernel_rsp(proc->kernel_stack_top);
    syscall_set_user_rsp(proc->saved_user_rsp);
    gdt_set_kernel_rsp0(proc->kernel_stack_top);

    wrmsr_fs_base(proc->fs_base);
    wrmsr_kernel_gs_base(proc->gs_base);
}

static void mark_process_runnable(process_t *proc, uint64_t entry, int32_t parent_pid)
{
    if (proc == NULL) {
        return;
    }

    uint64_t now_ns = process_perf_now_ns();
    proc->entry = entry;
    proc->parent_pid = parent_pid;
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
#define EXECVE_STRTOTAL_MAX 8192

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

    proc->kernel_stack_base = malloc(PROCESS_KERNEL_STACK_SIZE);
    if (!proc->kernel_stack_base) {
        return -1;
    }
    proc->kernel_stack_top = ((uint64_t)(uintptr_t)(proc->kernel_stack_base + PROCESS_KERNEL_STACK_SIZE)) & ~0xFULL;
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

    return process_user_alloc(aligned_len);
}

/* --- USER_MMAP arena: lazily-committed anonymous reservations -------------- */

int process_user_addr_in_mmap_arena(uint64_t addr, uint64_t length)
{
    if (length == 0u) {
        return addr >= USER_MMAP_BASE && addr < USER_MMAP_LIMIT;
    }
    if (addr < USER_MMAP_BASE) {
        return 0;
    }
    if (addr > (0xFFFFFFFFFFFFFFFFULL - length)) {
        return 0;
    }
    return (addr + length) <= USER_MMAP_LIMIT;
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

uint8_t process_get_current_abi_mode(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    uint8_t mode = PROCESS_ABI_IMPLUS;
    if (is_valid_pid(current_pid_get())) {
        mode = g_processes[current_pid_get()].abi_mode;
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return mode;
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
    if (cr3 == 0u ||
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

int32_t process_spawn_user_elf_with_arg(const char *path,
                                        const char *launch_argument)
{
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
        .min_vaddr = 0x1000,
        .max_vaddr = USER_CODE_LIMIT,
    };
    elf_loaded_image_info_t image_info = {0};

    if (!elf_loader_load_from_path(proc->cr3, path, &policy, &image_info)) {
        const char *elf_err = elf_loader_last_error();
        extern void serial_write_string(const char *str);
        extern void serial_write_uint64(uint64_t value);
        serial_write_string("[spawn-fail] pid=");
        serial_write_uint64((uint64_t)pid);
        serial_write_string(" path=");
        serial_write_string(path);
        serial_write_string(" cr3=0x");
        serial_write_uint64(proc->cr3);
        serial_write_string(" elf_err=");
        serial_write_string(elf_err ? elf_err : "(null)");
        serial_write_string("\n");
        ipc_cleanup_process_queue(pid);
        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_process_table_lock);
        release_process_resources(proc);
        reset_process_slot(proc);
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -1;
    }
    proc->abi_mode = image_info.linux_abi ? PROCESS_ABI_LINUX : PROCESS_ABI_IMPLUS;
    proc->main_phdr_vaddr = image_info.phdr_vaddr;
    proc->main_phent = image_info.phent;
    proc->main_phnum = image_info.phnum;
    if (proc->abi_mode == PROCESS_ABI_LINUX) {
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
        /* Default environment for Linux-ABI processes (used by the
           ld.so interpreter; ignored by static binaries).

           Two interpreter families reach this path:
             - the in-tree ImplusOS test ld.so (com.ImplusOS.dynmain /
               com.ImplusOS.ldso), exercised by the dynmain test app, which
               wants LD_LIBRARY_PATH pointed at its own lib dir + libpreload;
             - real glibc /lib64/ld-linux-x86-64.so.2 (external Linux
               binaries such as Chromium; Vendor/LinuxRuntime stages it at
               /lib64 with its .so closure under /usr/lib/x86_64-linux-gnu).
           The glibc loader chokes on the dynmain-specific vars, so pick the
           environment from the PT_INTERP path. See
           Docs/Others/TODO_glibc_Port.md G3. */
        static const char *implus_ld_envp[] = {
            "PATH=/bin:/usr/bin",
            "LD_LIBRARY_PATH=/Userland/Service/com.ImplusOS.dynmain/lib",
            "LD_PRELOAD=libpreload.so",
            NULL,
        };
        static const char *glibc_envp[] = {
            "PATH=/bin:/usr/bin",
            "LD_LIBRARY_PATH=/lib64:/usr/lib/x86_64-linux-gnu:/usr/lib",
            /* Xorg's modesetting_drv.so has 12 undefined gbm_* symbols and does
               NOT list libgbm.so.1 in DT_NEEDED -- on a stock distro it only
               resolves them because AccelMethod "glamor" pulls in
               libglamoregl.so (-> libgbm). Our xorg.conf forces AccelMethod
               "none" (no GPU), so nothing loads libgbm and the driver fails
               relocation ("undefined symbol: gbm_bo_get_plane_count",
               "No drivers available"). Preload libgbm.so.1 so its symbols sit
               in the global scope before modesetting_drv.so is dlopen()ed.
               TODO_Doom_Xorg_MethodA.md M6 (7th boot). */
            "LD_PRELOAD=libgbm.so.1",
            /* Bind every relocation at load time. Two reasons:
               (1) boot 7 proved libglx.so resolves cleanly under eager
                   binding but a *lazy* PLT fixup for one of its symbols dies
                   at GlxExtensionInit() time with an un-printable name
                   (process faults mid-message) -- eager binding sidesteps
                   the broken lazy path;
               (2) with LD_WARN=1 a load-time relocation miss is reported by
                   name and made non-fatal, so any genuinely absent symbol is
                   finally legible instead of racing off the console.
               TODO_Doom_Xorg_MethodA.md M6 (9th boot). */
            "LD_BIND_NOW=1",
            /* NOTE: LD_WARN=1 was removed. With it, the unresolved
               R_X86_64_GLOB_DAT for `glxServer` (a data symbol exported only
               by the Xorg PIE) was demoted to a warning and the GOT slot left
               0, so libglx.so later did `call *0x38(NULL)` -> SIGSEGV at 0x38.
               Without LD_WARN the miss is fatal AND named at load time:
               "Xorg: ... undefined symbol: glxServer", which is the real bug
               to chase (dlopen'd modules can't resolve main-executable data
               symbols). TODO_Doom_Xorg_MethodA.md M7. */
            "LANG=C.UTF-8",
            "LC_ALL=C.UTF-8",
            "TZ=UTC",
            "HOME=/tmp",
            /* XDG base dirs: glibc GUI stacks (GTK, GLib/GIO) and Chromium all
               expect these. No real per-user runtime dir exists, so point the
               runtime/cache dirs at /tmp (TmpFS). See
               Docs/Others/TODO_GTK3_Wayland_LinuxABI.md G4. */
            "XDG_RUNTIME_DIR=/tmp",
            "XDG_CACHE_HOME=/tmp/.cache",
            "XDG_CONFIG_HOME=/tmp/.config",
            "XDG_DATA_DIRS=/usr/share",
            "XDG_CONFIG_DIRS=/etc/xdg",
            /* GTK3: try the Wayland backend, then X11. ImplusOS has neither a
               Wayland compositor nor an X server yet, so gdk_display_open()
               is expected to fail with "cannot open display" - reaching that
               point already proves the whole glibc + GTK3 .so closure loaded
               and initialised. Compositor: TODO_GTK3_Wayland_LinuxABI.md G5. */
            "GDK_BACKEND=wayland,x11",
            "GSETTINGS_SCHEMA_DIR=/usr/share/glib-2.0/schemas",
            "GSETTINGS_BACKEND=memory",
            "FONTCONFIG_PATH=/etc/fonts",
            "FONTCONFIG_FILE=/etc/fonts/fonts.conf",
            /* Doom (Method A: TODO_Doom_Xorg_MethodA.md). The Xorg we spawn
               runs modesetting on /dev/dri/card0 (kernel KMS shim); Mesa must
               use the software rasteriser (llvmpipe) since there is no real
               GPU; the X core keyboard needs xkb-data's rules root.

               DISPLAY is NOT optional. Xlib does not fall back to :0 -- with
               no DISPLAY in the environment, XOpenDisplay(NULL) hands a NULL
               display name to xcb_connect(), which returns a connection
               already in the error state, so every X client dies at startup
               ("Couldn't connect to display!"). Name the socket the server
               actually listens on: /tmp/.X11-unix/X0. */
            "DISPLAY=:0",
            "LIBGL_ALWAYS_SOFTWARE=1",
            "GALLIUM_DRIVER=llvmpipe",
            /* Force indirect GLX: GL commands go over the X protocol and the
               server renders them (it reports "GLX: Initialized DRISWRAST GL
               provider" with +iglx). The client's own direct path cannot work
               here -- trixie ships every *_dri.so as the thin libdril shim,
               which has no usable software screen -- so glXCreateContext()
               returned NULL and the following glXMakeCurrent() drew a GLX
               BadMatch. See TODO_Doom_Xorg_MethodA.md M23. */
            "LIBGL_ALWAYS_INDIRECT=1",
            "XKB_CONFIG_ROOT=/usr/share/X11/xkb",
            /* Doom pulls libopenal (+ SDL2 via libfluidsynth). ImplusOS has no
               PipeWire/PulseAudio/ALSA device; letting OpenAL-soft probe them
               makes libpulse's pa_make_fd_cloexec() abort the process. Force
               the null / dummy backends -- audio is silent, Doom runs. */
            "ALSOFT_DRIVERS=null",
            "SDL_AUDIODRIVER=dummy",
            "PULSE_SERVER=none",
            /* Doom locates its IWAD relative to DOOMWADDIR (else "."); its
               soundfont from SOUNDFONT. Both data files live beside the
               binary in /Userland/Doom/Resource. */
            "DOOMWADDIR=/Userland/Doom/Resource",
            "SOUNDFONT=/Userland/Doom/Resource/soundfont.sf2",
            NULL,
        };
        const char **linux_envp = implus_ld_envp;
        {
            const char *ip = image_info.interp_path;
            for (uint32_t i = 0; i + 2 < sizeof(image_info.interp_path) &&
                                 ip[i] != '\0'; ++i) {
                if (ip[i] == 'l' && ip[i + 1] == 'd' && ip[i + 2] == '-') {
                    linux_envp = glibc_envp; /* ".../ld-linux-..." */
                    break;
                }
            }
        }
        uint64_t linux_envc = 0;
        while (linux_envp[linux_envc] != NULL) {
            ++linux_envc;
        }
        if (initialize_elf_user_stack_ex(proc, &image_info, path,
                                         linux_argc, linux_argv,
                                         linux_envc, linux_envp) < 0) {
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

/* Bring-up instrumentation (TODO_Doom_Xorg_MethodA.md M12). Xorg's fork() --
 * its only route to xkbcomp -- was measured taking ~281 s and then failing.
 * Whether that is sheer copy volume (Xorg maps libLLVM at 129 MB, libgallium
 * at 42 MB, ...) or a per-page pathology cannot be told apart from the
 * outside, so report both the elapsed time and the outcome once per fork. */
static int process_clone_address_space(process_t *child, process_t *parent)
{
    const uint64_t clone_start_ns = process_perf_now_ns();
    int clone_rc;

    if (parent->abi_mode == PROCESS_ABI_LINUX) {
#if KERNEL_COW_FORK
        /* Copy-on-write: share the parent's pages read-only into the child
         * and fault-in private copies on first write. Falls back to the
         * eager copy if the COW clone reports failure. */
        if (paging_cow_clone_user_range(child->cr3, parent->cr3,
                                        0x1000, USER_STACK_BASE) == 0) {
            clone_rc = 0;
            goto done;
        }
#endif
        clone_rc = paging_copy_present_user_range(child->cr3, parent->cr3,
                                                  0x1000, USER_STACK_TOP);
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
                                                      USER_MMAP_LIMIT);
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
    {
        uint64_t elapsed_ms =
            (process_perf_now_ns() - clone_start_ns) / 1000000ULL;
        serial_write_string("[fork] clone_address_space rc=");
        serial_write_uint64((uint64_t)(int64_t)clone_rc);
        serial_write_string(" elapsed_ms=");
        serial_write_uint64(elapsed_ms);
        /* Kernel heap, not free physical memory -- get_free_memory() walks the
         * heap free list. Labelled honestly so it is not read as an
         * out-of-RAM signal (it sits at ~94 MB in steady state). */
        serial_write_string(" free_heap=");
        serial_write_uint64(get_free_memory());
        serial_write_string(" free_pages=");
        serial_write_uint64(memory_free_pages());
        serial_write_char('\n');
    }
    return clone_rc;
}

int32_t process_fork(void)
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

    process_t *parent = process_memory_owner_locked(&g_processes[current_pid]);
    if (!parent || parent->state == PROCESS_STATE_UNUSED) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -3;
    }

    if (parent->is_thread) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        return -38;
    }

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

    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        child->user_allocs[i] = parent->user_allocs[i];
    }

    child->capability_mask = parent->capability_mask;
    child->abi_mode = parent->abi_mode;
    child->priority = parent->priority;
    child->rseq_area = parent->rseq_area;
    child->rseq_sig = parent->rseq_sig;
    child->fs_base = parent->fs_base;
    child->gs_base = hal_cpu_read_gs_base();
    memcpy(child->fpu_state, parent->fpu_state, PROCESS_FPU_STATE_SIZE);

    uint64_t *parent_kstack = (uint64_t *)(uintptr_t)parent->saved_rsp;
    uint64_t *child_kstack = (uint64_t *)(uintptr_t)child->kernel_stack_top;
    child_kstack -= PROCESS_CONTEXT_QWORDS;
    for (uint32_t i = 0; i < PROCESS_CONTEXT_QWORDS; ++i) {
        child_kstack[i] = parent_kstack ? parent_kstack[i] : 0;
    }
    child_kstack[SYSCALL_FRAME_RAX] = 0;

    child->saved_rsp = (uint64_t)(uintptr_t)child_kstack;
    child->saved_user_rsp = syscall_get_user_rsp();

    irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    if (child->state != PROCESS_STATE_INIT) {
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        goto fail_free_space;
    }

    child->entry = parent->entry;
    child->parent_pid = parent_pid_saved;
    child->timeslice = g_timeslice_ticks;
    child->state = PROCESS_STATE_READY;
    process_perf_mark_ready_locked(child, process_perf_now_ns());

    memcpy(child->name, parent->name, sizeof(child->name));
    memcpy(child->cwd, parent->cwd, sizeof(child->cwd));
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
    child->signal_mask = parent->signal_mask;
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
                    uint64_t spl = strlen(path_buf);
                    if (used + spl + 1u <= sizeof(strings_buf)) {
                        char *script_copy = strings_buf + used;
                        memcpy(script_copy, path_buf, spl + 1u);

                        /* Shift original argv[1..] right to make room for the
                         * prepended entries, then drop argv[0]. */
                        if (orig_rest > 0) {
                            memmove(&argv_ptrs[extra + 1u], &argv_ptrs[1],
                                    (size_t)orig_rest * sizeof(char *));
                        }
                        argv_ptrs[0] = sb_interp;
                        if (have_arg) {
                            argv_ptrs[1] = &shebang_line[arg_start];
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
        serial_write_string("[execve-fail] paging_create_process_space free_pages=");
        serial_write_uint64(memory_free_pages());
        serial_write_char('\n');
        /* Past the point of no return: the old image is gone, so there is
         * nothing to return to. Linux kills the process here rather than
         * failing the call, and returning -1 instead left the caller running
         * on a destroyed address space -- which is how a failed exec of
         * xkbcomp turned into busybox faulting at NULL. */
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
        serial_write_string("[execve-fail] thread-reserve free_pages=");
        serial_write_uint64(memory_free_pages());
        serial_write_char('\n');
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        /* Past the point of no return: the old image is gone, so there is
         * nothing to return to. Linux kills the process here rather than
         * failing the call, and returning -1 instead left the caller running
         * on a destroyed address space -- which is how a failed exec of
         * xkbcomp turned into busybox faulting at NULL. */
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
        serial_write_string("[execve-fail] initial-stack-map free_pages=");
        serial_write_uint64(memory_free_pages());
        serial_write_char('\n');
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        /* Past the point of no return: the old image is gone, so there is
         * nothing to return to. Linux kills the process here rather than
         * failing the call, and returning -1 instead left the caller running
         * on a destroyed address space -- which is how a failed exec of
         * xkbcomp turned into busybox faulting at NULL. */
        process_exit_current_with_status(127);
        return -1;
    }

    const char *name_ptr = path_buf;
    for (const char *p = path_buf; *p; ++p) {
        if (*p == '/' && *(p+1) != '\0') name_ptr = p + 1;
    }
    strncpy(proc->name, name_ptr, sizeof(proc->name) - 1);
    proc->name[sizeof(proc->name) - 1] = '\0';
    strncpy(proc->exe_path, path_buf, sizeof(proc->exe_path) - 1u);
    proc->exe_path[sizeof(proc->exe_path) - 1u] = '\0';

    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        proc->user_allocs[i].used = 0;
        proc->user_allocs[i].addr = 0;
        proc->user_allocs[i].size = 0;
    }

    elf_load_policy_t elf_policy = {
        .max_file_size = PROCESS_ELF_MAX_SIZE,
        .min_vaddr = 0x1000,
        .max_vaddr = USER_CODE_LIMIT,
    };
    elf_loaded_image_info_t image_info = {0};
    if (!elf_loader_load_from_path(proc->cr3, path_buf,
                                    &elf_policy, &image_info)) {
        const char *elf_err = elf_loader_last_error();
        extern void serial_write_string(const char *str);
        extern void serial_write_uint64(uint64_t value);
        serial_write_string("[execve-fail] pid=");
        serial_write_uint64((uint64_t)(proc - g_processes));
        serial_write_string(" path=");
        serial_write_string(path_buf);
        serial_write_string(" cr3=0x");
        serial_write_uint64(proc->cr3);
        serial_write_string(" elf_err=");
        serial_write_string(elf_err ? elf_err : "(null)");
        serial_write_string("\n");
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        /* Past the point of no return: the old image is gone, so there is
         * nothing to return to. Linux kills the process here rather than
         * failing the call, and returning -1 instead left the caller running
         * on a destroyed address space -- which is how a failed exec of
         * xkbcomp turned into busybox faulting at NULL. */
        process_exit_current_with_status(127);
        return -1;
    }

    proc->abi_mode = image_info.linux_abi ? PROCESS_ABI_LINUX : PROCESS_ABI_IMPLUS;
    proc->main_phdr_vaddr = image_info.phdr_vaddr;
    proc->main_phent = image_info.phent;
    proc->main_phnum = image_info.phnum;

    if (initialize_elf_user_stack_ex(proc, &image_info, path_buf,
                                      argc, (const char *const *)argv_ptrs,
                                      envc, (const char *const *)envp_ptrs) < 0) {
        serial_write_string("[execve-fail] user-stack-init\n");
        paging_destroy_process_space(new_cr3);
        proc->cr3 = 0;
        process_exit_current_with_status(127);
        return -1;
    }

    uint64_t kstack_saved = proc->saved_rsp;
    if (kstack_saved != 0) {
        uint64_t *kstack = (uint64_t *)(uintptr_t)kstack_saved;
        kstack[SYSCALL_FRAME_RCX] = image_info.entry;
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
        serial_write_string("[execve-fail] process state=");
        serial_write_uint64((uint64_t)proc->state);
        serial_write_char('\n');
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

    return 0;
}

int32_t process_copy_launch_argument(char *out, uint32_t capacity)
{
    if (!out || capacity == 0u) return -1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t pid = current_pid_get();
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

int32_t process_copy_exe_path(char *out, uint32_t capacity)
{
    if (!out || capacity == 0u) return -1;
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t pid = current_pid_get();
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
    strncpy(proc->name, "Userland.ELF", sizeof(proc->name) - 1);
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

    if (initialize_elf_user_stack(proc, &image_info, "/Userland/Userland.ELF") < 0) {
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
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    uint32_t closed_fds = 0;
    uint32_t closed_dirs = 0;
    syscall_file_close_all_for_pid(pid_to_exit, &closed_fds, &closed_dirs);
    syscall_socket_close_all_for_pid(pid_to_exit);
    unix_socket_close_all_for_pid(pid_to_exit);
    shared_memory_cleanup_process(pid_to_exit);
    filemap_release_pid(pid_to_exit);

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
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    pnp_notifications_cleanup_process(pid_to_exit);
    ipc_cleanup_process_queue(pid_to_exit);
    syscall_input_owner_release(pid_to_exit);

    /* Release any file-backed MAP_SHARED write-back registrations (live ones
     * were already flushed by munmap/msync). */
    extern void linux_compat_mshared_release_pid(int32_t pid);
    linux_compat_mshared_release_pid(pid_to_exit);
}

void process_exit_current_signaled(int32_t signum)
{
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
        process_run_robust_list_current(robust_head, tid);
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

int32_t process_get_current_pid(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t pid = -1;
    if (is_valid_pid(current_pid_get())) {
        process_t *owner =
            process_memory_owner_locked(&g_processes[current_pid_get()]);
        if (owner != NULL) {
            pid = (int32_t)(owner - g_processes);
        }
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return pid;
}

int32_t process_get_current_tid(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);
    int32_t tid = is_valid_pid(current_pid_get()) ? current_pid_get() : -1;
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return tid;
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

        if (do_switch && original_state != PROCESS_STATE_BLOCKED) {
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
        if (0) {
            extern void serial_write_uint64(uint64_t value);
            extern void serial_write_string(const char *str);
            if (current->is_thread) {
                serial_write_string("[SR> pid=");
                serial_write_uint64((uint64_t)(uint32_t)current_pid_get());
                serial_write_string(" srs=");
                serial_write_uint64(return_saved_rsp);
                serial_write_string(" rcx=");
                if (return_saved_rsp != 0u) {
                    serial_write_uint64(((const uint64_t *)(uintptr_t)return_saved_rsp)[13]);
                } else {
                    serial_write_uint64(0);
                }
                serial_write_string(" ursp=");
                serial_write_uint64(return_user_rsp);
                serial_write_string("]\n");
            }
        }
        (void)return_saved_rsp;
        (void)return_user_rsp;
        
        process_fpu_restore(current->fpu_state);
        activate_process_context(current);
        
        spinlock_unlock(&g_process_table_lock);
        irq_restore(irq_flags);
        
        if (next_user_rsp_out != NULL) {
            *next_user_rsp_out = return_user_rsp;
        }
        return return_saved_rsp;
    }

    int32_t next_pid = process_scheduler_pick_next(g_processes,
                                                   g_process_capacity,
                                                   current_pid_get());

    while (next_pid < 0) {
        uint32_t ecpu = smp_get_current_cpu_id();
        spinlock_unlock(&g_process_table_lock);
        hal_cpu_enable_interrupts();
        uint64_t idle_start_ns = timer_monotonic_ns();
        process_cpu_halt();
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
        if (0) {
            extern void serial_write_uint64(uint64_t value);
            extern void serial_write_string(const char *str);
            if (next->is_thread || g_processes[current_pid_get()].is_thread) {
            serial_write_string("[SW> pid=");
            serial_write_uint64((uint64_t)(uint32_t)current_pid_get());
            serial_write_string(" srs=");
            serial_write_uint64(next_saved_rsp);
            serial_write_string(" srs_rcx=");
            if (next_saved_rsp != 0u) {
                serial_write_uint64(((const uint64_t *)(uintptr_t)next_saved_rsp)[13]);
            } else {
                serial_write_uint64(0);
            }
            serial_write_string(" srs_r11=");
            if (next_saved_rsp != 0u) {
                serial_write_uint64(((const uint64_t *)(uintptr_t)next_saved_rsp)[14]);
            } else {
                serial_write_uint64(0);
            }
            serial_write_string(" ursp=");
            serial_write_uint64(next_user_rsp);
            serial_write_string("]\n");
        }
    }
    (void)next_saved_rsp;
    (void)next_user_rsp;
    
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

int process_run_next_on_current_cpu(void)
{
    if (!process_table_ready()) {
        return 0;
    }

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
    irq_restore(irq_flags);

    /* The lazily-committed mmap arena is valid as a syscall buffer even when
     * pages are not yet resident: copy_to/from_user()'s memcpy takes a
     * kernel-mode #PF that the handler services demand-zero (see
     * page_fault_handler / process_user_reserve). Requiring residency here
     * would EFAULT every write into a fresh PartitionAlloc region. */
    if (range_within(addr, len, USER_MMAP_BASE, USER_MMAP_LIMIT)) {
        return 1;
    }

    if (abi_mode == PROCESS_ABI_LINUX) {
        if (addr < 0x1000) {
            return 0;
        }
        if (range_within(addr, len, 0x1000, USER_STACK_BASE)) {
            return paging_is_user_range_mapped(process_cr3, addr, len);
        }
        if (range_within(addr, len, stack_base, stack_top)) {
            return paging_is_user_range_mapped(process_cr3, addr, len);
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
    uint64_t alloc_size = 0;
    if (align_up_u64_checked((uint64_t)size, 16ULL, &alloc_size) < 0 ||
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
    if (align_up_u64_checked(proc->user_heap_cursor, 16ULL, &addr) < 0) {
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
            slot->used = 0;
            spinlock_unlock(&g_process_table_lock);
            irq_restore(irq_flags);
            (void)paging_unmap_range(cr3, addr, size);
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
    for (uint32_t i = 0; i < PROCESS_USER_ALLOC_MAX; ++i) {
        user_alloc_t *slot = &proc->user_allocs[i];
        if (slot->size == 0) {
            continue;
        }
        uint64_t slot_start = slot->addr;
        uint64_t slot_end = slot->addr + slot->size;
        if (slot_start >= start && slot_end <= end) {
            slot->used = 0;
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    return paging_unmap_range(cr3, start, end - start);
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
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
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

    if (!process_signal_blocked_locked(leader, signum)) {
        leader->pending_signals |= (1u << (uint32_t)signum);
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
    }
    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
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
        process_perf_mark_ready_locked(proc, now_ns);
        process_scheduler_request_reschedule();
    } else if (proc->state != PROCESS_STATE_UNUSED &&
               proc->state != PROCESS_STATE_DEAD &&
               proc->state != PROCESS_STATE_ZOMBIE) {
        /* Target has not reached process_block_current() yet (it is still
         * racing towards it after failing its lock-free condition check).
         * Leave a credit so the upcoming block call short-circuits instead
         * of sleeping through this wake. Saturate rather than overflow --
         * callers only ever need to know "at least one wake happened". */
        if (proc->wake_pending < UINT32_MAX) {
            proc->wake_pending++;
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);
    return 0;
}

int32_t process_waitpid(int32_t pid, int32_t *status_out, int32_t options)
{
    return process_waitpid_ex(pid, status_out, options, NULL);
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
            !process_scheduler_pid_in_use_on_any_cpu(pid)) {
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
            g_processes[i].parent_pid == my_pid &&
            !process_scheduler_pid_in_use_on_any_cpu(i)) {
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


void process_debug_dump_cpu_usage(const char *reason, uint64_t interval_ns)
{
    if (interval_ns == 0u) {
        interval_ns = 1000000000ULL;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_process_table_lock);

    uint64_t now_ns = process_perf_now_ns();
    uint64_t total_delta_ns = 0u;
    uint64_t total_syscall_delta = 0u;
    uint64_t total_display_delta = 0u;
    uint64_t top_delta_ns = 0u;
    uint64_t top_user_delta_ns = 0u;
    uint64_t top_user_syscall_delta = 0u;
    uint64_t top_user_display_delta = 0u;
    uint64_t top_syscall_count = 0u;
    uint64_t top_syscall_runtime_delta = 0u;
    uint64_t user_delta_ns = 0u;
    uint64_t user_syscall_delta = 0u;
    int32_t top_pid = -1;
    int32_t top_user_pid = -1;
    int32_t top_syscall_pid = -1;
    char top_name[64];
    char top_user_name[64];
    char top_syscall_name[64];
    top_name[0] = '\0';
    top_user_name[0] = '\0';
    top_syscall_name[0] = '\0';

    int32_t capacity = g_process_capacity;
    if (capacity > OS_CONFIG_PROCESS_MAX_COUNT_MAX) {
        capacity = OS_CONFIG_PROCESS_MAX_COUNT_MAX;
    }

    for (int32_t i = 0; i < capacity; ++i) {
        process_t *proc = &g_processes[i];
        if (proc->state == PROCESS_STATE_UNUSED ||
            proc->state == PROCESS_STATE_DEAD) {
            g_cpu_usage_prev_runtime_ns[i] = 0u;
            g_cpu_usage_prev_syscalls[i] = 0u;
            g_cpu_usage_prev_display_bytes[i] = 0u;
            continue;
        }

        uint64_t runtime_ns = proc->runtime_ns;
        if (proc->state == PROCESS_STATE_RUNNING &&
            proc->last_scheduled_ns != 0u &&
            now_ns >= proc->last_scheduled_ns) {
            runtime_ns += now_ns - proc->last_scheduled_ns;
        }

        uint64_t runtime_delta = 0u;
        if (runtime_ns >= g_cpu_usage_prev_runtime_ns[i]) {
            runtime_delta = runtime_ns - g_cpu_usage_prev_runtime_ns[i];
        }
        uint64_t syscall_delta = 0u;
        if (proc->syscalls >= g_cpu_usage_prev_syscalls[i]) {
            syscall_delta = proc->syscalls - g_cpu_usage_prev_syscalls[i];
        }
        uint64_t display_delta = 0u;
        if (proc->display_bytes >= g_cpu_usage_prev_display_bytes[i]) {
            display_delta = proc->display_bytes -
                            g_cpu_usage_prev_display_bytes[i];
        }

        g_cpu_usage_prev_runtime_ns[i] = runtime_ns;
        g_cpu_usage_prev_syscalls[i] = proc->syscalls;
        g_cpu_usage_prev_display_bytes[i] = proc->display_bytes;

        total_delta_ns += runtime_delta;
        total_syscall_delta += syscall_delta;
        total_display_delta += display_delta;
        if (i != 0) {
            user_delta_ns += runtime_delta;
            user_syscall_delta += syscall_delta;
            if (runtime_delta > top_user_delta_ns) {
                top_user_delta_ns = runtime_delta;
                top_user_syscall_delta = syscall_delta;
                top_user_display_delta = display_delta;
                top_user_pid = i;
                memcpy(top_user_name, proc->name, sizeof(top_user_name));
                top_user_name[sizeof(top_user_name) - 1u] = '\0';
            }
        }
        if (runtime_delta > top_delta_ns) {
            top_delta_ns = runtime_delta;
            top_pid = i;
            memcpy(top_name, proc->name, sizeof(top_name));
            top_name[sizeof(top_name) - 1u] = '\0';
        }
        if (syscall_delta > top_syscall_count) {
            top_syscall_count = syscall_delta;
            top_syscall_runtime_delta = runtime_delta;
            top_syscall_pid = i;
            memcpy(top_syscall_name, proc->name, sizeof(top_syscall_name));
            top_syscall_name[sizeof(top_syscall_name) - 1u] = '\0';
        }
    }

    spinlock_unlock(&g_process_table_lock);
    irq_restore(irq_flags);

    uint32_t cpu_count = smp_get_cpu_count();
    if (cpu_count == 0u) {
        cpu_count = 1u;
    }
    uint64_t denominator = interval_ns * (uint64_t)cpu_count;
    if (denominator == 0u) {
        denominator = interval_ns;
    }
    uint64_t total_cpu_x100 = (total_delta_ns * 10000ULL) / denominator;
    uint64_t top_cpu_x100 = (top_delta_ns * 10000ULL) / denominator;
    uint64_t user_cpu_x100 = (user_delta_ns * 10000ULL) / denominator;
    uint64_t top_user_cpu_x100 =
        (top_user_delta_ns * 10000ULL) / denominator;
    uint64_t top_syscall_cpu_x100 =
        (top_syscall_runtime_delta * 10000ULL) / denominator;

    char line[384];
    snprintf(line, sizeof(line),
             "[cpu] reason=%s interval_ms=%llu cpus=%u total=%llu.%02llu%% user=%llu.%02llu%% syscalls=%llu user_syscalls=%llu display_kib=%llu top_pid=%d top=%s top_cpu=%llu.%02llu%% top_user_pid=%d top_user=%s top_user_cpu=%llu.%02llu%% top_user_syscalls=%llu top_user_display_kib=%llu syscall_pid=%d syscall_top=%s syscall_count=%llu syscall_cpu=%llu.%02llu%%\n",
             reason ? reason : "periodic",
             (unsigned long long)(interval_ns / 1000000ULL),
             (unsigned int)cpu_count,
             (unsigned long long)(total_cpu_x100 / 100ULL),
             (unsigned long long)(total_cpu_x100 % 100ULL),
             (unsigned long long)(user_cpu_x100 / 100ULL),
             (unsigned long long)(user_cpu_x100 % 100ULL),
             (unsigned long long)total_syscall_delta,
             (unsigned long long)user_syscall_delta,
             (unsigned long long)(total_display_delta / 1024ULL),
             (int)top_pid,
             top_name,
             (unsigned long long)(top_cpu_x100 / 100ULL),
             (unsigned long long)(top_cpu_x100 % 100ULL),
             (int)top_user_pid,
             top_user_name,
             (unsigned long long)(top_user_cpu_x100 / 100ULL),
             (unsigned long long)(top_user_cpu_x100 % 100ULL),
             (unsigned long long)top_user_syscall_delta,
             (unsigned long long)(top_user_display_delta / 1024ULL),
             (int)top_syscall_pid,
             top_syscall_name,
             (unsigned long long)top_syscall_count,
             (unsigned long long)(top_syscall_cpu_x100 / 100ULL),
             (unsigned long long)(top_syscall_cpu_x100 % 100ULL));
    serial_write_string(line);
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
