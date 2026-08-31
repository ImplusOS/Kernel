#pragma once

#include <stdint.h>
#include "kernel/config.h"

#define USER_CODE_BASE    0x0000004000000000ULL
#define USER_CODE_LIMIT   0x0000004080000000ULL
#define USER_HEAP_BASE    0x0000004100000000ULL
#define USER_STACK_SIZE   (32 * 1024 * 1024ULL)
#define USER_STACK_TOP    0x0000004800000000ULL
#define USER_STACK_BASE   (USER_STACK_TOP - USER_STACK_SIZE)
#define USER_HEAP_LIMIT   USER_STACK_BASE

/*
 * Anonymous mmap() arena. Separate from the (eagerly-committed, bump-managed)
 * heap so that mmap() can hand out multi-GiB *lazily-committed* reservations:
 * pages here are not backed by physical frames until first touch, which is
 * serviced demand-zero by paging_handle_swap_fault() from the #PF handler
 * (both user- and kernel-mode faults). Chromium's PartitionAlloc reserves
 * ~48 GiB of PROT_NONE address space up front and commits sub-ranges via
 * mprotect(); that is impossible against the eager heap allocator.
 *
 * 1 TiB..16 TiB -> its own PML4 slots (index 2..31), clear of the code/heap/
 * stack regions which all live in PML4[0] (< 512 GiB). Well inside the 47-bit
 * canonical user half.
 */
/* Size of the per-process brk()/sbrk() window carved out of the user heap
 * region. glibc's main malloc arena lives here and must be contiguous and
 * exclusively owned; nothing else allocates from it. 8 GiB is far more than
 * any foreign binary in tree needs and still leaves the bump allocator the
 * bulk of the ~29 GiB heap region. */
#define USER_BRK_WINDOW_SIZE (8ULL * 1024ULL * 1024ULL * 1024ULL)

#define USER_MMAP_BASE    0x0000010000000000ULL
#define USER_MMAP_LIMIT   0x0000100000000000ULL

#if (USER_CODE_BASE >= USER_CODE_LIMIT)
#error "Invalid user code range"
#endif

#if (USER_HEAP_BASE >= USER_HEAP_LIMIT)
#error "Invalid user heap range"
#endif

#if (USER_STACK_BASE >= USER_STACK_TOP)
#error "Invalid user stack range"
#endif

#if (USER_MMAP_BASE >= USER_MMAP_LIMIT) || (USER_MMAP_BASE < USER_STACK_TOP)
#error "Invalid user mmap range"
#endif

typedef uint64_t process_capability_mask_t;

#define PROCESS_CAP_SERIAL  (1ULL << 0)
#define PROCESS_CAP_PROCESS (1ULL << 1)
#define PROCESS_CAP_FILE    (1ULL << 2)
#define PROCESS_CAP_MEMORY  (1ULL << 3)
#define PROCESS_CAP_INPUT   (1ULL << 4)
#define PROCESS_CAP_SIGNAL  (1ULL << 5)
#define PROCESS_CAP_IPC     (1ULL << 6)
#define PROCESS_CAP_NETWORK (1ULL << 7)
#define PROCESS_CAP_DISPLAY (1ULL << 8)

#define PROCESS_CAP_DEFAULT_MASK \
    (PROCESS_CAP_SERIAL | PROCESS_CAP_PROCESS | PROCESS_CAP_FILE | PROCESS_CAP_MEMORY | PROCESS_CAP_INPUT | PROCESS_CAP_SIGNAL | PROCESS_CAP_IPC | PROCESS_CAP_NETWORK | PROCESS_CAP_DISPLAY)

#define PROCESS_RSEQ_AREA_SIZE          32u
#define PROCESS_RSEQ_CPU_ID_START       0u
#define PROCESS_RSEQ_CPU_ID             4u
#define PROCESS_RSEQ_CS                 8u
#define PROCESS_RSEQ_CS_SIG             0u
#define PROCESS_RSEQ_CS_FLAGS           4u
#define PROCESS_RSEQ_CS_FLAG_NO_RESTART_ON_PREEMPT 1u

typedef struct {
    int32_t pid;
    int32_t parent_pid;
    uint8_t state;
    uint8_t reserved[7];
    char name[64];
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
    uint64_t display_present_calls;
    uint64_t display_rects;
    uint64_t display_bytes;
    uint64_t memory_usage;
    uint64_t page_fault_count;
} process_perf_info_t;

#define PROCESS_SCHEDULE_REQUEST_SWITCH      1
#define PROCESS_SCHEDULE_REQUEST_INVOLUNTARY 2

void process_manager_init(void);
int32_t process_register_boot_process(const char *path, uint64_t *entry_out);
int32_t process_create_user(uint64_t entry);
int32_t process_create_user_ex(uint64_t entry,
                               uint64_t arg1,
                               uint64_t arg2,
                               uint64_t arg3,
                               uint64_t arg4);
int32_t process_create_thread(uint64_t entry,
                              uint64_t arg1,
                              uint64_t arg2,
                              uint64_t arg3,
                              uint64_t arg4);
/* Same as process_create_thread(), but when has_tls is non-zero the new
 * thread's FS base (x86-64 TLS pointer) is set to tls_fs_base atomically
 * with making the thread schedulable (see clone(2) CLONE_SETTLS) -
 * TODO_Chromium_LinuxABI.md section 3.6.
 *
 * user_stack: if non-zero, the new thread's initial user RSP. It is set
 * before the thread becomes schedulable, so an SMP peer that dispatches
 * the thread immediately still sees the caller-provided stack. Used by
 * the Linux clone(2) path, where glibc's __clone has already pushed the
 * child's fn/arg onto this stack - the kernel must not touch its
 * contents. Pass 0 to have the kernel allocate/prepare a fresh raw
 * user stack (native ImplusOS threads). */
int32_t process_create_thread_ex(uint64_t entry,
                                 uint64_t arg1,
                                 uint64_t arg2,
                                 uint64_t arg3,
                                 uint64_t arg4,
                                 int has_tls,
                                 uint64_t tls_fs_base,
                                 uint64_t user_stack);
int32_t process_spawn_user_elf(const char *path);
int32_t process_spawn_user_elf_with_arg(const char *path,
                                        const char *launch_argument);
int32_t process_fork(void);
int32_t process_execve(const char *path, const char *const *argv,
                       const char *const *envp);
int32_t process_copy_launch_argument(char *out, uint32_t capacity);
int32_t process_copy_exe_path(char *out, uint32_t capacity);
void process_exit_current_with_status(int32_t exit_status);
void process_exit_current_signaled(int32_t signum);
void process_exit_current(void);
void process_thread_exit_current(int32_t exit_status);
int32_t process_get_current_pid(void);
int32_t process_get_current_tid(void);
int process_thread_join(int32_t tid);
int process_thread_detach(int32_t tid);
void process_set_current_fs_base(uint64_t fs_base);
uint64_t process_get_current_fs_base(void);
int64_t process_get_main_image_info(uint64_t *phdr_vaddr,
                                    uint64_t *phent,
                                    uint64_t *phnum);
int process_set_clear_child_tid(uint64_t address);
int process_set_robust_list(uint64_t head, uint64_t length);
int process_rseq_register(uint64_t area, uint32_t sig);
int process_rseq_unregister(void);
uint64_t process_get_current_rseq_area(void);
void process_rseq_update_on_preempt_locked(int32_t pid, uint32_t cpu_id);
uint8_t process_get_current_abi_mode(void);
void process_set_current_abi_mode(uint8_t mode);
uint64_t process_get_current_saved_rsp(void);
uint64_t process_get_current_user_rsp(void);
uint64_t process_get_current_cr3(void);
const char *process_get_current_name_str(void);
uint64_t process_get_current_kernel_stack_base(void);
uint64_t process_schedule_on_syscall(uint64_t current_saved_rsp,
                                     uint64_t current_user_rsp,
                                     int request_switch,
                                     uint64_t *next_user_rsp_out);
uint64_t process_schedule_after_exit(uint64_t *next_user_rsp_out);
int process_user_buffer_is_valid(const void *ptr, uint64_t len);
int process_user_cstring_length(const char *str, uint64_t max_len, uint64_t *len_out);
void *process_user_alloc(uint64_t size);
uint64_t process_get_heap_cursor(void);
/* brk(2) support: the current program break, and moving it. Kept separate
 * from the user_heap_cursor bump allocator - see process_set_brk(). */
uint64_t process_get_brk(void);
int process_set_brk(uint64_t addr);
int process_set_heap_cursor(uint64_t addr);
void process_set_thread_user_rsp(int32_t tid, uint64_t user_rsp);
int process_user_free(void *ptr);
void *process_user_mmap(uint64_t length, uint64_t flags);

/* Lazily-committed anonymous reservation from the USER_MMAP arena. Returns a
 * page-aligned base for `length` bytes with NO physical backing; pages fault
 * in demand-zero on first access. `length` is not capped at 4 GiB. NULL on
 * exhaustion. See USER_MMAP_BASE above. */
void *process_user_reserve(uint64_t length);
int   process_user_addr_in_mmap_arena(uint64_t addr, uint64_t length);
int process_user_munmap(void *ptr, uint64_t length);
uint64_t process_signal_set_handler(int32_t signum, uint64_t handler);
uint64_t process_signal_get_handler(int32_t signum);
int process_signal_set_handler_ex(int32_t signum, uint64_t handler,
                                  uint64_t flags, uint64_t mask,
                                  uint64_t restorer,
                                  uint64_t *old_handler_out,
                                  uint64_t *old_flags_out,
                                  uint64_t *old_mask_out,
                                  uint64_t *old_restorer_out);
int process_sigaltstack(uint64_t new_sp, uint64_t new_size, uint32_t new_flags,
                        int has_new, uint64_t *old_sp_out,
                        uint64_t *old_size_out, uint32_t *old_flags_out);
int64_t process_signal_rt_sigreturn(uint64_t saved_rsp);
#if defined(__x86_64__)
int process_signal_deliver_fault_now(int32_t pid, int32_t signum,
                                     uint64_t fault_addr,
                                     uint64_t *kernel_regs,
                                     uint64_t *cpu_frame);
#endif
uint64_t process_signal_get_mask(void);
int process_signal_set_mask(uint64_t mask);
int process_signal_deliver(int32_t pid, int32_t signum);
int process_signal_deliver_group(int32_t pid, int32_t signum);
int process_signal_maybe_self_terminate(int32_t signum);
int process_signal_validate_group(int32_t tgid, int32_t tid);
int process_is_current_thread(void);
uint64_t process_get_current_pending_signals(void);
int process_consume_pending_signal(int32_t signum);
int process_set_current_name(const char *name, uint32_t max_len);
int process_get_current_name(char *out, uint32_t capacity);
int process_set_current_cwd(const char *cwd);
int process_get_current_cwd(char *out, uint32_t capacity);
int32_t process_waitpid(int32_t pid, int32_t *status_out, int32_t options);
/* Like process_waitpid, but also reports how the child terminated so the
 * Linux ABI can build a POSIX wait status. On a successful reap:
 *   *term_signal_out = 0            -> normal exit, *status_out = exit code
 *   *term_signal_out = <signum>     -> killed by that signal
 * term_signal_out may be NULL. */
int32_t process_waitpid_ex(int32_t pid, int32_t *status_out, int32_t options,
                           int32_t *term_signal_out);
int32_t process_getppid(void);
int32_t process_get_parent_pid(int32_t pid);
int process_is_guard_page_fault(uint64_t fault_addr);
process_capability_mask_t process_default_capabilities(void);
process_capability_mask_t process_get_current_capabilities(void);
int process_current_has_capability(process_capability_mask_t capability);
int process_set_capabilities(int32_t pid, process_capability_mask_t capabilities);
int process_get_capabilities(int32_t pid, process_capability_mask_t *capabilities_out);
int process_set_priority(int32_t pid, uint8_t priority);
void process_on_timer_tick(void);
int  process_timeslice_expired(void);
int process_run_next_on_current_cpu(void);
void process_debug_dump_slot_no_lock(int32_t pid);
void process_debug_dump_current(void);
void process_debug_dump_pid(int32_t pid);
int32_t current_pid_get(void);
int process_is_alive(int32_t pid);
int process_sleep_current_ms(uint64_t ms);
int process_block_current(void);
int process_wake_pid(int32_t pid);
int32_t process_terminate(int32_t pid);
int32_t process_get_full_info(int32_t pid, void *info_out);
int32_t process_get_perf_info(int32_t pid, process_perf_info_t *info_out);
int32_t process_get_capacity(void);
int32_t process_count_threads(int32_t owner_pid);
void process_perf_note_syscall(int32_t pid);
void process_perf_note_ipc_send(int32_t pid);
void process_perf_note_ipc_recv(int32_t pid);
void process_perf_note_display(int32_t pid, uint32_t rect_count, uint64_t bytes);
void process_debug_dump_cpu_usage(const char *reason, uint64_t interval_ns);
void process_record_page_fault(int32_t pid, uint64_t fault_addr, uint64_t rip,
                               uint32_t error_code, int is_guard);
int32_t process_get_debug_info(int32_t pid, void *info_out);
int32_t process_get_os_debug(void *info_out);
