/*
 * Linux AArch64 syscall ABI (arm64 kernel).
 *
 * A Linux AArch64 program traps with `svc #0`, the syscall number in x8 and
 * arguments in x0-x5, result in x0 (Arch/arm64/cpu/Exception.c hands those
 * to syscall_dispatch, which routes PROCESS_ABI_LINUX_ARM64 processes here
 * through the compat registry). The numbering is the kernel's generic table
 * (include/uapi/asm-generic/unistd.h), and the program is LP64, so nearly
 * every call is the x86_64 one under another number and goes to the shared
 * implementation unchanged (see Linux_CompatCore.h). The rest is translated
 * here:
 *   - struct stat (fstat, newfstatat) has its own AArch64 layout (struct
 *     epoll_event does too, but the kernel's epoll already uses the natural
 *     16-byte layout when built for arm64 -- Core/syscall/Syscall_Epoll.h);
 *   - four O_* flags have different values (O_DIRECTORY, O_NOFOLLOW,
 *     O_DIRECT, O_LARGEFILE);
 *   - clone() takes tls before child_tid, and a new thread must come back
 *     with all of the caller's registers (glibc keeps the thread function in
 *     x10/x11 across the svc);
 *   - rt_sigreturn restores an AArch64 signal frame;
 *   - PROT_BTI / PROT_MTE are accepted and ignored.
 * Everything the generic table has no x86_64 counterpart for -- or that the
 * shared implementation does not provide -- returns ENOSYS.
 */

#include "Compat/Linux/Linux_FrontEnd.h"
#include "Compat/compat_registry.h"
#include <string.h>

#if defined(__aarch64__)
#include "cpu/Exception.h"

enum {
    A64_fcntl = 25,
    A64_openat = 56, A64_newfstatat = 79, A64_fstat = 80,
    A64_rt_sigreturn = 139, A64_clone = 220, A64_mmap = 222,
    A64_mprotect = 226, A64_rseq = 293, A64_clone3 = 435,
    A64_openat2 = 437,
    A64_NR_MAX = 448,
};

/* asm-generic number -> x86_64 number, for every call whose arguments and
 * structures are identical on the two LP64 ABIs. */
static const uint16_t g_a64_direct[][2] = {
    {17, X64_getcwd}, {19, X64_eventfd2}, {20, X64_epoll_create1},
    {21, X64_epoll_ctl}, {22, X64_epoll_pwait}, {441, X64_epoll_pwait2},
    {23, X64_dup}, {24, X64_dup3}, {26, X64_inotify_init1},
    {27, X64_inotify_add_watch}, {28, X64_inotify_rm_watch},
    {29, X64_ioctl}, {32, X64_flock}, {33, X64_mknodat}, {34, X64_mkdirat},
    {35, X64_unlinkat}, {36, X64_symlinkat}, {37, X64_linkat},
    {38, X64_renameat}, {39, X64_umount2}, {40, X64_mount},
    {41, X64_pivot_root}, {43, X64_statfs}, {44, X64_fstatfs},
    {45, X64_truncate}, {46, X64_ftruncate}, {47, X64_fallocate},
    {48, X64_faccessat}, {49, X64_chdir}, {50, X64_fchdir},
    {51, X64_chroot}, {52, X64_fchmod}, {53, X64_fchmodat},
    {54, X64_fchownat}, {55, X64_fchown}, {57, X64_close},
    {59, X64_pipe2}, {61, X64_getdents64}, {62, X64_lseek},
    {63, X64_read}, {64, X64_write}, {65, X64_readv}, {66, X64_writev},
    {67, X64_pread64}, {68, X64_pwrite64}, {69, X64_preadv},
    {70, X64_pwritev}, {71, X64_sendfile}, {72, X64_pselect6},
    {73, X64_ppoll}, {74, X64_signalfd4}, {78, X64_readlinkat},
    {81, X64_sync}, {82, X64_fsync}, {83, X64_fdatasync},
    {85, X64_timerfd_create}, {86, X64_timerfd_settime},
    {87, X64_timerfd_gettime}, {88, X64_utimensat}, {90, X64_capget},
    {91, X64_capset}, {92, X64_personality}, {93, X64_exit},
    {94, X64_exit_group}, {95, X64_waitid}, {96, X64_set_tid_address},
    {97, X64_unshare}, {98, X64_futex}, {99, X64_set_robust_list},
    {100, X64_get_robust_list}, {101, X64_nanosleep},
    {102, X64_getitimer}, {103, X64_setitimer},
    {113, X64_clock_gettime}, {114, X64_clock_getres},
    {115, X64_clock_nanosleep}, {116, X64_syslog}, {117, X64_ptrace},
    {118, X64_sched_setparam}, {119, X64_sched_setscheduler},
    {120, X64_sched_getscheduler}, {121, X64_sched_getparam},
    {122, X64_sched_setaffinity}, {123, X64_sched_getaffinity},
    {124, X64_sched_yield}, {125, X64_sched_get_priority_max},
    {126, X64_sched_get_priority_min}, {127, X64_sched_rr_get_interval},
    {129, X64_kill}, {130, X64_tkill}, {131, X64_tgkill},
    {132, X64_sigaltstack}, {133, X64_rt_sigsuspend},
    {134, X64_rt_sigaction}, {135, X64_rt_sigprocmask},
    {136, X64_rt_sigpending}, {137, X64_rt_sigtimedwait},
    {138, X64_rt_sigqueueinfo}, {140, X64_setpriority},
    {141, X64_getpriority}, {143, X64_setregid}, {144, X64_setgid},
    {145, X64_setreuid}, {146, X64_setuid}, {147, X64_setresuid},
    {148, X64_getresuid}, {149, X64_setresgid}, {150, X64_getresgid},
    {151, X64_setfsuid}, {152, X64_setfsgid}, {153, X64_times},
    {154, X64_setpgid}, {155, X64_getpgid}, {156, X64_getsid},
    {157, X64_setsid}, {158, X64_getgroups}, {159, X64_setgroups},
    {160, X64_uname}, {163, X64_getrlimit}, {164, X64_setrlimit},
    {165, X64_getrusage}, {166, X64_umask}, {167, X64_prctl},
    {168, X64_getcpu}, {169, X64_gettimeofday}, {170, X64_settimeofday},
    {172, X64_getpid}, {173, X64_getppid}, {174, X64_getuid},
    {175, X64_geteuid}, {176, X64_getgid}, {177, X64_getegid},
    {178, X64_gettid}, {179, X64_sysinfo}, {194, X64_shmget},
    {195, X64_shmctl}, {196, X64_shmat}, {197, X64_shmdt},
    {198, X64_socket}, {199, X64_socketpair}, {200, X64_bind},
    {201, X64_listen}, {202, X64_accept}, {203, X64_connect},
    {204, X64_getsockname}, {205, X64_getpeername}, {206, X64_sendto},
    {207, X64_recvfrom}, {208, X64_setsockopt}, {209, X64_getsockopt},
    {210, X64_shutdown}, {211, X64_sendmsg}, {212, X64_recvmsg},
    {213, X64_readahead}, {214, X64_brk}, {215, X64_munmap},
    {216, X64_mremap}, {221, X64_execve}, {223, X64_fadvise64},
    {227, X64_msync}, {228, X64_mlock}, {229, X64_munlock},
    {230, X64_mlockall}, {231, X64_munlockall}, {232, X64_mincore},
    {233, X64_madvise}, {242, X64_accept4}, {243, X64_recvmmsg},
    {260, X64_wait4}, {261, X64_prlimit64}, {264, X64_name_to_handle_at},
    {267, X64_syncfs}, {268, X64_setns}, {269, X64_sendmmsg},
    {274, X64_sched_setattr}, {275, X64_sched_getattr},
    {276, X64_renameat2}, {277, X64_seccomp}, {278, X64_getrandom},
    {279, X64_memfd_create}, {281, X64_execveat}, {283, X64_membarrier},
    {284, X64_mlock2}, {285, X64_copy_file_range}, {286, X64_preadv2},
    {287, X64_pwritev2}, {291, X64_statx}, {436, X64_close_range},
    {439, X64_faccessat2},
};

#define A64_UNMAPPED 0xFFFFu
static uint16_t g_a64_to_x64[A64_NR_MAX];

/* The four O_* flags whose values differ (arch/arm64/include/uapi/asm/
 * fcntl.h against the generic ones x86_64 uses). */
#define A64_O_DIRECTORY 0040000u
#define A64_O_NOFOLLOW  0100000u
#define A64_O_DIRECT    0200000u
#define A64_O_LARGEFILE 0400000u
#define X64_O_DIRECT    0040000u
#define X64_O_LARGEFILE 0100000u
#define X64_O_DIRECTORY 0200000u
#define X64_O_NOFOLLOW  0400000u
#define O_SWAPPED_MASK  (0040000u | 0100000u | 0200000u | 0400000u)

static uint64_t a64_oflags_in(uint64_t f)
{
    uint64_t out = f & ~(uint64_t)O_SWAPPED_MASK;
    if (f & A64_O_DIRECTORY) out |= X64_O_DIRECTORY;
    if (f & A64_O_NOFOLLOW)  out |= X64_O_NOFOLLOW;
    if (f & A64_O_DIRECT)    out |= X64_O_DIRECT;
    if (f & A64_O_LARGEFILE) out |= X64_O_LARGEFILE;
    return out;
}

static uint64_t a64_oflags_out(uint64_t f)
{
    uint64_t out = f & ~(uint64_t)O_SWAPPED_MASK;
    if (f & X64_O_DIRECTORY) out |= A64_O_DIRECTORY;
    if (f & X64_O_NOFOLLOW)  out |= A64_O_NOFOLLOW;
    if (f & X64_O_DIRECT)    out |= A64_O_DIRECT;
    if (f & X64_O_LARGEFILE) out |= A64_O_LARGEFILE;
    return out;
}

/* include/uapi/asm-generic/stat.h */
typedef struct {
    uint64_t st_dev;
    uint64_t st_ino;
    uint32_t st_mode;
    uint32_t st_nlink;
    uint32_t st_uid;
    uint32_t st_gid;
    uint64_t st_rdev;
    uint64_t __pad1;
    int64_t  st_size;
    int32_t  st_blksize;
    int32_t  __pad2;
    int64_t  st_blocks;
    int64_t  st_atime_sec;
    uint64_t st_atime_nsec;
    int64_t  st_mtime_sec;
    uint64_t st_mtime_nsec;
    int64_t  st_ctime_sec;
    uint64_t st_ctime_nsec;
    uint32_t __unused4;
    uint32_t __unused5;
} a64_stat_t;
_Static_assert(sizeof(a64_stat_t) == 128, "asm-generic struct stat");

typedef struct {
    uint64_t saved_rsp;
    linux_call_t call;
    uint64_t sw;
    lx_scratch_t scratch;
} a64_ctx_t;

/* One call into the shared implementation. Returns its result; *restarted
 * (via ctx->call) tells the caller to leave the frame alone. */
static int64_t a64_core(a64_ctx_t *c, uint64_t x64_num, uint64_t a1,
                        uint64_t a2, uint64_t a3, uint64_t a4, uint64_t a5,
                        uint64_t a6)
{
    c->sw |= linux_syscall_core(c->saved_rsp, &c->call, x64_num,
                                a1, a2, a3, a4, a5, a6);
    return c->call.result;
}

static int64_t a64_stat_out(a64_ctx_t *c, uint64_t x64_num, uint64_t a1,
                            uint64_t a2, uint64_t a3, uint64_t a4,
                            uint64_t user_buf)
{
    uint64_t tmp = lx_scratch_alloc(&c->scratch, sizeof(lx_x64_stat_t));
    if (tmp == 0u) {
        return LX_EFAULT;
    }
    int64_t rc;
    if (x64_num == X64_fstat) {
        rc = a64_core(c, X64_fstat, a1, tmp, 0, 0, 0, 0);
    } else {
        rc = a64_core(c, X64_newfstatat, a1, a2, tmp, a4, 0, 0);
    }
    (void)a3;
    if (c->call.restarted || rc < 0) {
        return rc;
    }
    lx_x64_stat_t in;
    if (copy_from_user(&in, (const void *)(uintptr_t)tmp, sizeof(in)) != 0u) {
        return LX_EFAULT;
    }
    a64_stat_t out;
    memset(&out, 0, sizeof(out));
    out.st_dev = in.st_dev;
    out.st_ino = in.st_ino;
    out.st_mode = in.st_mode;
    out.st_nlink = (uint32_t)in.st_nlink;
    out.st_uid = in.st_uid;
    out.st_gid = in.st_gid;
    out.st_rdev = in.st_rdev;
    out.st_size = in.st_size;
    out.st_blksize = (int32_t)in.st_blksize;
    out.st_blocks = in.st_blocks;
    out.st_atime_sec = (int64_t)in.st_atime_sec;
    out.st_atime_nsec = in.st_atime_nsec;
    out.st_mtime_sec = (int64_t)in.st_mtime_sec;
    out.st_mtime_nsec = in.st_mtime_nsec;
    out.st_ctime_sec = (int64_t)in.st_ctime_sec;
    out.st_ctime_nsec = in.st_ctime_nsec;
    if (copy_to_user((void *)(uintptr_t)user_buf, &out, sizeof(out)) != 0u) {
        return LX_EFAULT;
    }
    return rc;
}

static int64_t a64_clone_thread(a64_ctx_t *c, uint64_t flags, uint64_t stack,
                                uint64_t ptid, uint64_t tls, uint64_t ctid)
{
    if (stack == 0u || (flags & LX_CLONE_VM) == 0u) {
        return LX_EINVAL;
    }
    int32_t tid = process_create_thread_clone(c->saved_rsp, stack,
                                              (flags & LX_CLONE_SETTLS) != 0u,
                                              tls, NULL);
    return lx_thread_clone_finish(tid, flags, ptid, ctid);
}

static uint64_t linux_arm64_syscall_dispatch(uint64_t saved_rsp, uint64_t num,
                                             uint64_t a1, uint64_t a2,
                                             uint64_t a3, uint64_t a4,
                                             uint64_t a5, uint64_t a6)
{
    a64_ctx_t c;
    memset(&c, 0, sizeof(c));
    c.saved_rsp = saved_rsp;
    c.call.abi = PROCESS_ABI_LINUX_ARM64;
    c.call.orig_num = num;
    lx_scratch_init(&c.scratch,
                    ((const arm64_exception_frame_t *)(uintptr_t)saved_rsp)->sp_el0,
                    USER_FOREIGN_BASE);

    int64_t result;
    switch (num) {
    case A64_rt_sigreturn:
        result = process_signal_rt_sigreturn_arm64(saved_rsp);
        break;

    case A64_openat:
        result = a64_core(&c, X64_openat, a1, a2, a64_oflags_in(a3), a4, 0, 0);
        break;

    case A64_openat2: {
        uint64_t how[3];
        if (a4 < sizeof(how) ||
            copy_from_user(how, (const void *)(uintptr_t)a3, sizeof(how)) != 0u) {
            result = LX_EFAULT;
            break;
        }
        how[0] = a64_oflags_in(how[0]);
        uint64_t tmp = lx_scratch_alloc(&c.scratch, sizeof(how));
        if (tmp == 0u ||
            copy_to_user((void *)(uintptr_t)tmp, how, sizeof(how)) != 0u) {
            result = LX_EFAULT;
            break;
        }
        result = a64_core(&c, X64_openat2, a1, a2, tmp, sizeof(how), 0, 0);
        break;
    }

    case A64_fcntl: {
        const uint32_t cmd = (uint32_t)a2;
        if (cmd == 4u /* F_SETFL */) {
            a3 = a64_oflags_in(a3);
        }
        result = a64_core(&c, X64_fcntl, a1, a2, a3, a4, a5, a6);
        if (!c.call.restarted && cmd == 3u /* F_GETFL */ && result >= 0) {
            result = (int64_t)a64_oflags_out((uint64_t)result);
        }
        break;
    }

    case A64_fstat:
        result = a64_stat_out(&c, X64_fstat, a1, 0, 0, 0, a2);
        break;

    case A64_newfstatat:
        result = a64_stat_out(&c, X64_newfstatat, a1, a2, 0, a4, a3);
        break;

    case A64_clone:
        /* clone(flags, stack, parent_tid, tls, child_tid) */
        if ((a1 & LX_CLONE_THREAD) != 0u) {
            result = a64_clone_thread(&c, a1, a2, a3, a4, a5);
        } else {
            result = a64_core(&c, X64_clone, a1, a2, a3, a5, a4, 0);
        }
        break;

    case A64_clone3: {
        struct {
            uint64_t flags, pidfd, child_tid, parent_tid, exit_signal;
            uint64_t stack, stack_size, tls;
        } args;
        if (a2 < sizeof(args) ||
            copy_from_user(&args, (const void *)(uintptr_t)a1, sizeof(args)) != 0u) {
            result = LX_EFAULT;
            break;
        }
        if ((args.flags & LX_CLONE_THREAD) != 0u) {
            uint64_t top = args.stack_size != 0u ? args.stack + args.stack_size
                                                 : args.stack;
            result = a64_clone_thread(&c, args.flags, top, args.parent_tid,
                                      args.tls, args.child_tid);
        } else {
            result = a64_core(&c, X64_clone3, a1, a2, 0, 0, 0, 0);
        }
        break;
    }

    case A64_mmap:
        /* PROT_BTI (0x10) / PROT_MTE (0x20): no such page attributes here. */
        result = a64_core(&c, X64_mmap, a1, a2, a3 & 0x7u, a4, a5, a6);
        break;

    case A64_mprotect:
        result = a64_core(&c, X64_mprotect, a1, a2, a3 & 0x7u, 0, 0, 0);
        break;

    case A64_rseq:
        /* The preemption fixup rseq needs exists only on the x86_64 side;
         * glibc treats ENOSYS as "no rseq". */
        result = LX_ENOSYS;
        break;

    default:
        if (num < A64_NR_MAX && g_a64_to_x64[num] != A64_UNMAPPED) {
            result = a64_core(&c, g_a64_to_x64[num], a1, a2, a3, a4, a5, a6);
        } else {
            result = LX_ENOSYS;
        }
        break;
    }

    if (!c.call.restarted) {
        linux_frame_set_result(saved_rsp, PROCESS_ABI_LINUX_ARM64, result);
    }
    return c.sw;
}

static const compat_layer_t g_linux_arm64_layer = {
    .abi = PROCESS_ABI_LINUX_ARM64,
    .name = "Linux AArch64",
    .dispatch = linux_arm64_syscall_dispatch,
};

void linux_arm64_compat_layer_register(void)
{
    for (uint32_t i = 0; i < A64_NR_MAX; ++i) {
        g_a64_to_x64[i] = A64_UNMAPPED;
    }
    for (uint32_t i = 0; i < sizeof(g_a64_direct) / sizeof(g_a64_direct[0]); ++i) {
        g_a64_to_x64[g_a64_direct[i][0]] = g_a64_direct[i][1];
    }
    (void)compat_registry_register(&g_linux_arm64_layer);
}

#else

void linux_arm64_compat_layer_register(void)
{
}

#endif
