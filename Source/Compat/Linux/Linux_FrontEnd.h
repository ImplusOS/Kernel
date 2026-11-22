#pragma once

/*
 * Helpers shared by the Linux i386 and AArch64 front ends (see
 * Linux_CompatCore.h). Internal to Kernel/Source/Compat/Linux.
 */

#include <stdint.h>
#include <stddef.h>

#include "Compat/Linux/Linux_CompatCore.h"
#include "Core/process/ProcessManager.h"
#include "Core/syscall/Syscall_Main.h"
#include "Core/usercopy/Usercopy.h"
#include "mmu/Paging_Main.h"

#define LX_EPERM      (-1)
#define LX_ENOENT     (-2)
#define LX_ESRCH      (-3)
#define LX_EINTR      (-4)
#define LX_EBADF      (-9)
#define LX_EAGAIN     (-11)
#define LX_ENOMEM     (-12)
#define LX_EFAULT     (-14)
#define LX_EINVAL     (-22)
#define LX_ENOSYS     (-38)
#define LX_EOVERFLOW  (-75)

/* Linux x86_64 syscall numbers the front ends translate into -- the
 * numbering Syscall_LinuxCompat.c implements. */
enum {
    X64_read = 0, X64_write = 1, X64_open = 2, X64_close = 3, X64_stat = 4,
    X64_fstat = 5, X64_lstat = 6, X64_poll = 7, X64_lseek = 8, X64_mmap = 9,
    X64_mprotect = 10, X64_munmap = 11, X64_brk = 12, X64_rt_sigaction = 13,
    X64_rt_sigprocmask = 14, X64_rt_sigreturn = 15, X64_ioctl = 16,
    X64_pread64 = 17, X64_pwrite64 = 18, X64_readv = 19, X64_writev = 20,
    X64_access = 21, X64_pipe = 22, X64_select = 23, X64_sched_yield = 24,
    X64_mremap = 25, X64_msync = 26, X64_mincore = 27, X64_madvise = 28,
    X64_shmget = 29, X64_shmat = 30, X64_shmctl = 31, X64_dup = 32,
    X64_dup2 = 33, X64_pause = 34, X64_nanosleep = 35, X64_getitimer = 36,
    X64_alarm = 37, X64_setitimer = 38, X64_getpid = 39, X64_sendfile = 40,
    X64_socket = 41, X64_connect = 42, X64_accept = 43, X64_sendto = 44,
    X64_recvfrom = 45, X64_sendmsg = 46, X64_recvmsg = 47, X64_shutdown = 48,
    X64_bind = 49, X64_listen = 50, X64_getsockname = 51,
    X64_getpeername = 52, X64_socketpair = 53, X64_setsockopt = 54,
    X64_getsockopt = 55, X64_clone = 56, X64_fork = 57, X64_vfork = 58,
    X64_execve = 59, X64_exit = 60, X64_wait4 = 61, X64_kill = 62,
    X64_uname = 63, X64_shmdt = 67, X64_fcntl = 72, X64_flock = 73,
    X64_fsync = 74, X64_fdatasync = 75, X64_truncate = 76,
    X64_ftruncate = 77, X64_getcwd = 79, X64_chdir = 80, X64_fchdir = 81,
    X64_rename = 82, X64_mkdir = 83, X64_rmdir = 84, X64_creat = 85,
    X64_link = 86, X64_unlink = 87, X64_symlink = 88, X64_readlink = 89,
    X64_chmod = 90, X64_fchmod = 91, X64_chown = 92, X64_fchown = 93,
    X64_lchown = 94, X64_umask = 95, X64_gettimeofday = 96,
    X64_getrlimit = 97, X64_getrusage = 98, X64_sysinfo = 99, X64_times = 100,
    X64_ptrace = 101, X64_getuid = 102, X64_syslog = 103, X64_getgid = 104,
    X64_setuid = 105, X64_setgid = 106, X64_geteuid = 107, X64_getegid = 108,
    X64_setpgid = 109, X64_getppid = 110, X64_getpgrp = 111, X64_setsid = 112,
    X64_setreuid = 113, X64_setregid = 114, X64_getgroups = 115,
    X64_setgroups = 116, X64_setresuid = 117, X64_getresuid = 118,
    X64_setresgid = 119, X64_getresgid = 120, X64_getpgid = 121,
    X64_setfsuid = 122, X64_setfsgid = 123, X64_getsid = 124,
    X64_capget = 125, X64_capset = 126, X64_rt_sigpending = 127,
    X64_rt_sigtimedwait = 128, X64_rt_sigqueueinfo = 129,
    X64_rt_sigsuspend = 130, X64_sigaltstack = 131, X64_mknod = 133,
    X64_personality = 135, X64_statfs = 137, X64_fstatfs = 138,
    X64_getpriority = 140, X64_setpriority = 141, X64_sched_setparam = 142,
    X64_sched_getparam = 143, X64_sched_setscheduler = 144,
    X64_sched_getscheduler = 145, X64_sched_get_priority_max = 146,
    X64_sched_get_priority_min = 147, X64_sched_rr_get_interval = 148,
    X64_mlock = 149, X64_munlock = 150, X64_mlockall = 151,
    X64_munlockall = 152, X64_pivot_root = 155, X64_prctl = 157,
    X64_setrlimit = 160, X64_chroot = 161, X64_sync = 162,
    X64_settimeofday = 164, X64_mount = 165, X64_umount2 = 166,
    X64_gettid = 186, X64_readahead = 187, X64_tkill = 200, X64_time = 201,
    X64_futex = 202, X64_sched_setaffinity = 203,
    X64_sched_getaffinity = 204, X64_getdents64 = 217,
    X64_set_tid_address = 218, X64_fadvise64 = 221, X64_timer_create = 222,
    X64_clock_gettime = 228, X64_clock_getres = 229,
    X64_clock_nanosleep = 230, X64_exit_group = 231, X64_epoll_wait = 232,
    X64_epoll_ctl = 233, X64_tgkill = 234, X64_waitid = 247,
    X64_inotify_init = 253, X64_inotify_add_watch = 254,
    X64_inotify_rm_watch = 255, X64_openat = 257, X64_mkdirat = 258,
    X64_mknodat = 259, X64_fchownat = 260, X64_newfstatat = 262,
    X64_unlinkat = 263, X64_renameat = 264, X64_linkat = 265,
    X64_symlinkat = 266, X64_readlinkat = 267, X64_fchmodat = 268,
    X64_faccessat = 269, X64_pselect6 = 270, X64_ppoll = 271,
    X64_unshare = 272, X64_set_robust_list = 273, X64_get_robust_list = 274,
    X64_utimensat = 280, X64_epoll_pwait = 281, X64_timerfd_create = 283,
    X64_eventfd = 284, X64_fallocate = 285, X64_timerfd_settime = 286,
    X64_timerfd_gettime = 287, X64_accept4 = 288, X64_signalfd4 = 289,
    X64_eventfd2 = 290, X64_epoll_create1 = 291, X64_dup3 = 292,
    X64_pipe2 = 293, X64_inotify_init1 = 294, X64_preadv = 295,
    X64_pwritev = 296, X64_recvmmsg = 299, X64_prlimit64 = 302,
    X64_name_to_handle_at = 303, X64_syncfs = 306, X64_sendmmsg = 307,
    X64_setns = 308, X64_getcpu = 309, X64_sched_setattr = 314,
    X64_sched_getattr = 315, X64_renameat2 = 316, X64_seccomp = 317,
    X64_getrandom = 318, X64_memfd_create = 319, X64_execveat = 322,
    X64_membarrier = 324, X64_mlock2 = 325, X64_copy_file_range = 326,
    X64_preadv2 = 327, X64_pwritev2 = 328, X64_statx = 332, X64_rseq = 334,
    X64_clone3 = 435, X64_close_range = 436, X64_openat2 = 437,
    X64_faccessat2 = 439, X64_epoll_pwait2 = 441,
};

#define LX_CLONE_VM             0x00000100u
#define LX_CLONE_SETTLS         0x00080000u
#define LX_CLONE_PARENT_SETTID  0x00100000u
#define LX_CLONE_CHILD_CLEARTID 0x00200000u
#define LX_CLONE_CHILD_SETTID   0x01000000u
#define LX_CLONE_THREAD         0x00010000u

/* Scratch user memory for translated structures: carved downwards from a
 * little below the caller's stack pointer. Nothing below the stack pointer is
 * live in either ABI (neither has a red zone), the space is gone again when
 * the call returns, and a restarted call rebuilds it from the program's own
 * arguments. Living in the caller's address space, the shared implementation
 * reads and writes it exactly as it would the program's own buffers. */
typedef struct {
    uint64_t cursor;
    uint64_t floor;
} lx_scratch_t;

#define LX_SCRATCH_GAP 256u

static inline void lx_scratch_init(lx_scratch_t *s, uint64_t user_sp,
                                   uint64_t floor)
{
    s->cursor = user_sp > LX_SCRATCH_GAP ? (user_sp - LX_SCRATCH_GAP) & ~0xFULL : 0u;
    s->floor = floor;
}

/* Make every page of [addr, addr+len) present in the caller's address space
 * (demand-zero where it can be), so a buffer just below the stack pointer is
 * one the usercopy checks accept. */
static inline int lx_user_prepare(uint64_t addr, uint64_t len)
{
    uint64_t space = process_get_current_cr3();
    if (space == 0u || len == 0u) {
        return space == 0u ? -1 : 0;
    }
    for (uint64_t pg = addr & ~0xFFFULL; pg < addr + len; pg += 0x1000u) {
        if (paging_virt_to_phys(space, pg) == 0u &&
            paging_handle_swap_fault(space, pg) <= 0) {
            return -1;
        }
    }
    return 0;
}

static inline uint64_t lx_scratch_alloc(lx_scratch_t *s, uint64_t size)
{
    if (s->cursor == 0u || size > s->cursor) {
        return 0u;
    }
    uint64_t addr = (s->cursor - size) & ~0xFULL;
    if (addr < s->floor || addr >= s->cursor) {
        return 0u;
    }
    if (lx_user_prepare(addr, s->cursor - addr) < 0) {
        return 0u;
    }
    s->cursor = addr;
    return addr;
}

/* The CLONE_*SETTID / CLONE_CHILD_CLEARTID part of creating a thread, once
 * process_create_thread_clone() has made it. */
static inline int64_t lx_thread_clone_finish(int32_t tid, uint64_t flags,
                                             uint64_t ptid, uint64_t ctid)
{
    if (tid < 0) {
        return LX_EAGAIN;
    }
    uint32_t tid32 = (uint32_t)tid;
    if ((flags & LX_CLONE_PARENT_SETTID) != 0u && ptid != 0u) {
        (void)copy_to_user((void *)(uintptr_t)ptid, &tid32, sizeof(tid32));
    }
    if ((flags & LX_CLONE_CHILD_SETTID) != 0u && ctid != 0u) {
        (void)copy_to_user((void *)(uintptr_t)ctid, &tid32, sizeof(tid32));
    }
    if ((flags & LX_CLONE_CHILD_CLEARTID) != 0u && ctid != 0u) {
        (void)process_set_clear_child_tid_for(tid, ctid);
    }
    return (int64_t)process_pid_as_seen_by_current(tid);
}

/* struct stat as Linux x86_64 lays it out -- what the shared implementation
 * writes; the front ends repack it. */
typedef struct {
    uint64_t st_dev;
    uint64_t st_ino;
    uint64_t st_nlink;
    uint32_t st_mode;
    uint32_t st_uid;
    uint32_t st_gid;
    uint32_t __pad0;
    uint64_t st_rdev;
    int64_t  st_size;
    int64_t  st_blksize;
    int64_t  st_blocks;
    uint64_t st_atime_sec;
    uint64_t st_atime_nsec;
    uint64_t st_mtime_sec;
    uint64_t st_mtime_nsec;
    uint64_t st_ctime_sec;
    uint64_t st_ctime_nsec;
    int64_t  __unused[3];
} lx_x64_stat_t;

_Static_assert(sizeof(lx_x64_stat_t) == 144, "x86_64 struct stat");
