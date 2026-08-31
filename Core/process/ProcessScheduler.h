#pragma once

#include <stdint.h>
#include "kernel/config.h"
#include "ProcessManager.h"

/* Per-process/-thread kernel stack. This is the stack the CPU switches to on a
 * syscall (SYSCALL_KERNEL_RSP) and on an interrupt/exception from CPL3
 * (TSS.RSP0). It is a plain heap allocation with NO guard page, so an overflow
 * silently corrupts adjacent heap objects and surfaces later as a random panic
 * or triple-fault reboot. The Linux-ABI compat layer plus Chromium's deeply
 * nested glibc syscalls (and fault handlers that now run scheduler/demand-page
 * code on this stack) blew past the old 32 KiB. 128 KiB gives real headroom;
 * process_kstack_canary_check() converts any residual overflow into a clean,
 * deterministic panic instead of heap corruption. */
#define PROCESS_KERNEL_STACK_SIZE (128 * 1024)

/* Sentinel written to the lowest 16 bytes of every kernel stack right after
 * allocation; checked on each context switch. If it is gone, something wrote
 * past the bottom of the stack. */
#define PROCESS_KSTACK_CANARY 0x9E3779B97F4A7C15ULL
#define PROCESS_USER_ALLOC_MAX 4096
#define PROCESS_SIGNAL_MAX 32
#define PROCESS_FPU_STATE_SIZE 544U

#define PROCESS_STATE_UNUSED  0
#define PROCESS_STATE_READY   1
#define PROCESS_STATE_RUNNING 2
#define PROCESS_STATE_DEAD    3
#define PROCESS_STATE_INIT    4
#define PROCESS_STATE_ZOMBIE  5
#define PROCESS_STATE_BLOCKED 6

#define PROCESS_ABI_IMPLUS 0
#define PROCESS_ABI_LINUX  1

typedef struct {
    uint8_t used;
    uint64_t addr;
    uint64_t size;   /* was uint32_t: mmap() reservations can exceed 4 GiB */
} user_alloc_t;

typedef struct __attribute__((aligned(16))) {
    uint8_t state;
    uint8_t is_thread;
    uint8_t thread_detached;
    uint8_t abi_mode;
    uint8_t user_stack_exchanged;
    process_capability_mask_t capability_mask;
    uint64_t entry;
    uint64_t saved_rsp;
    uint64_t saved_user_rsp;
    uint64_t main_phdr_vaddr;
    uint64_t main_phent;
    uint64_t main_phnum;
    uint8_t  fpu_state[PROCESS_FPU_STATE_SIZE] __attribute__((aligned(16)));

    uint64_t fs_base;
    uint64_t gs_base;

    uint64_t cr3;
    uint8_t *kernel_stack_base;
    uint64_t kernel_stack_top;
    uint64_t user_code_base;
    uint64_t user_code_limit;
    uint64_t user_heap_base;
    uint64_t user_heap_cursor;
    uint64_t user_heap_limit;
    uint64_t user_heap_alloc_limit;
    uint64_t user_heap_guard_page;
    /* The program break (brk/sbrk), which glibc's malloc grows as one
     * contiguous arena. It gets a window of its own, disjoint from the
     * user_heap_cursor bump allocator: those two used to share a cursor, so a
     * dlopen()ed shared object could be mapped straight on top of memory
     * malloc had already handed out. See process_set_heap_cursor(). */
    uint64_t user_brk_base;
    uint64_t user_brk_cursor;
    uint64_t user_brk_limit;
    uint64_t user_stack_base;
    uint64_t user_stack_top;
    uint64_t user_stack_guard_page;
    /* Bump cursor into the USER_MMAP anon arena (lazily-committed mmap()).
     * Shared across threads of a process (they share the address space);
     * copied on fork. */
    uint64_t user_mmap_cursor;
    uint32_t timeslice;
    uint8_t  priority;
    uint64_t total_ticks;
    uint64_t runtime_ns;
    uint64_t ready_wait_ns;
    uint64_t max_ready_wait_ns;
    uint64_t context_switches;
    uint64_t voluntary_switches;
    uint64_t involuntary_switches;
    uint64_t syscalls;
    uint64_t ipc_send;
    uint64_t ipc_recv;
    uint64_t block_count;
    uint64_t wake_count;
    uint64_t blocked_ns;
    /* Wake-ahead-of-block credits. process_wake_pid() bumps this when the
     * target has not reached PROCESS_STATE_BLOCKED yet (it is still racing
     * towards process_block_current()); process_block_current() consumes a
     * credit instead of sleeping so a wake can never be lost to that window.
     * Both sides run under g_process_table_lock, so check-and-block and
     * check-and-wake are each atomic with respect to one another. */
    uint32_t wake_pending;
    uint64_t display_present_calls;
    uint64_t display_rects;
    uint64_t display_bytes;
    uint64_t last_scheduled_ns;
    uint64_t ready_since_ns;
    uint64_t blocked_since_ns;
    char     name[64];
    char     cwd[256];
    /* Absolute path the process was exec'd from. Backs /proc/self/exe (glibc /
     * Chromium read it via readlink to locate their own asset directory) and
     * argv[0] for Linux-ABI binaries. Empty for the idle/kernel tasks. */
    char     exe_path[256];
    char     launch_argument[512];
    int32_t parent_pid;
    int32_t memory_owner_pid;
    int32_t exit_status;
    /* Termination cause for POSIX wait-status encoding (wait4/waitid in the
     * Linux ABI). When exit_by_signal != 0 the process died from signal
     * exit_term_signal; otherwise exit_status carries the exit_group() code.
     * The native waitpid path ignores these and keeps its legacy convention. */
    uint8_t  exit_by_signal;
    uint8_t  exit_term_signal;
    uint64_t thread_stack_region_base;
    uint64_t thread_stack_region_size;
    user_alloc_t user_allocs[PROCESS_USER_ALLOC_MAX];
    uint64_t signal_handlers[PROCESS_SIGNAL_MAX];
    /* Extended per-signal sigaction(2) state (TODO_Chromium_LinuxABI.md
     * 3.5): flags (SA_SIGINFO/SA_ONSTACK/SA_RESTART/SA_NODEFER/...),
     * sa_mask (blocked while the handler runs), and sa_restorer (glibc
     * always supplies one; used as the frame's return trampoline so a
     * plain `ret` from the handler reaches rt_sigreturn(2) instead of
     * jumping straight back into interrupted user code). */
    uint64_t signal_flags[PROCESS_SIGNAL_MAX];
    uint64_t signal_sa_mask[PROCESS_SIGNAL_MAX];
    uint64_t signal_restorer[PROCESS_SIGNAL_MAX];
    uint64_t altstack_sp;
    uint64_t altstack_size;
    uint32_t altstack_flags; /* 0, SS_DISABLE(2), or SS_ONSTACK(1, read-only/reported) */
    uint64_t signal_mask;
    uint32_t pending_signals;
    uint64_t clear_child_tid;
    uint64_t robust_list_head;
    uint64_t robust_list_length;
    uint64_t rseq_area;
    uint32_t rseq_sig;

    uint64_t page_fault_count;
    uint64_t last_pf_addr;
    uint64_t last_pf_rip;
    uint32_t last_pf_error;
    uint8_t  has_crashed;
    char     crash_reason[64];
} process_t;

void process_scheduler_init(uint32_t timeslice_ticks);
int32_t process_scheduler_current_pid(void);
void process_scheduler_set_current_pid(int32_t pid);
void process_scheduler_set_leaving_pid(int32_t pid);
void process_scheduler_clear_leaving_pid(void);
int process_scheduler_pid_in_use_on_any_cpu(int32_t pid);
int32_t process_scheduler_pick_next(process_t *processes,
                                    int32_t capacity,
                                    int32_t current_pid);
void process_scheduler_on_tick(process_t *processes, int32_t capacity);
void process_scheduler_request_reschedule(void);
int process_scheduler_consume_reschedule(void);
void process_scheduler_prepare_run(process_t *proc);
void process_scheduler_add_idle_ns(uint64_t ns);
uint64_t process_scheduler_get_idle_ns(uint32_t cpu);
uint32_t process_scheduler_max_cpus(void);
void process_scheduler_debug_dump_cpus(void);
