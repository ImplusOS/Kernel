/*
 * Linux i386 syscall ABI (x86_64 kernel, 32-bit compatibility mode).
 *
 * A Linux i386 program traps with `int $0x80`: number in EAX, arguments in
 * EBX, ECX, EDX, ESI, EDI, EBP, result in EAX, every other register preserved
 * (Arch/x86_64/cpu/Syscall_Entry.asm, isr_int80). Syscall_Dispatch.c routes
 * PROCESS_ABI_LINUX_I386 processes here through the compat registry.
 *
 * The program is ILP32: long and pointers are 4 bytes, time_t and off_t are
 * 4 bytes in the old calls (the *64 / *_time64 calls carry 64-bit ones), and
 * 64-bit arguments arrive split over two registers. Each call is translated
 * into the x86_64 call the shared implementation provides (Linux_CompatCore.h)
 * -- through structures rebuilt in scratch space below the caller's stack
 * pointer where the layouts differ -- and its results back. Calls that only
 * exist on i386 (set_thread_area, socketcall, ipc, sigreturn, mmap2,
 * _llseek, the 16-bit uid family, ...) are implemented here on top of the
 * same calls.
 *
 * The address space (one window of the low 2 GiB, see USER32_* in
 * Core/process/ProcessManager.h), the trap entry, TLS segments and signal
 * frames are the process manager's and the x86_64 architecture code's.
 */

#include "Compat/Linux/Linux_FrontEnd.h"
#include "Compat/compat_registry.h"
#include "Debug/serial/Serial.h"
#include <string.h>

#if defined(__x86_64__)
#include "cpu/GDT_Main.h"

/* An int argument, as a 64-bit program would have passed it. */
#define SX(v) ((uint64_t)(int64_t)(int32_t)(uint32_t)(v))
/* Two halves of a 64-bit argument. */
#define LOHI(lo, hi) (((uint64_t)(uint32_t)(hi) << 32) | (uint64_t)(uint32_t)(lo))

/* ---- i386 syscall numbers that need more than a renumbering ---------- */
enum {
    I386_exit = 1, I386_waitpid = 7, I386_execve = 11, I386_time = 13,
    I386_lchown16 = 16, I386_lseek = 19, I386_umount = 22, I386_setuid16 = 23,
    I386_getuid16 = 24, I386_utime = 30, I386_times = 43, I386_setgid16 = 46,
    I386_getgid16 = 47, I386_signal = 48, I386_geteuid16 = 49,
    I386_getegid16 = 50, I386_fcntl = 55, I386_sigaction = 67,
    I386_setreuid16 = 70, I386_setregid16 = 71, I386_sigsuspend = 72,
    I386_sigpending = 73, I386_setrlimit = 75, I386_getrlimit = 76,
    I386_getrusage = 77, I386_gettimeofday = 78, I386_select = 82,
    I386_old_mmap = 90, I386_truncate = 92, I386_ftruncate = 93,
    I386_fchown16 = 95, I386_statfs = 99, I386_fstatfs = 100,
    I386_socketcall = 102, I386_setitimer = 104, I386_getitimer = 105,
    I386_stat = 106, I386_lstat = 107, I386_fstat = 108, I386_wait4 = 114,
    I386_sysinfo = 116, I386_ipc = 117, I386_sigreturn = 119,
    I386_clone = 120, I386_sigprocmask = 126, I386_llseek = 140,
    I386_getdents = 141, I386_newselect = 142, I386_readv = 145,
    I386_writev = 146, I386_nanosleep = 162, I386_setresuid16 = 164,
    I386_setresgid16 = 170, I386_rt_sigreturn = 173, I386_rt_sigaction = 174,
    I386_rt_sigtimedwait = 177, I386_pread64 = 180, I386_pwrite64 = 181,
    I386_chown16 = 182, I386_sigaltstack = 186, I386_sendfile = 187,
    I386_ugetrlimit = 191, I386_mmap2 = 192, I386_truncate64 = 193,
    I386_ftruncate64 = 194, I386_stat64 = 195, I386_lstat64 = 196,
    I386_fstat64 = 197, I386_fcntl64 = 221, I386_readahead = 225,
    I386_futex = 240, I386_set_thread_area = 243, I386_get_thread_area = 244,
    I386_fadvise64 = 250, I386_epoll_create = 254, I386_clock_gettime = 265,
    I386_clock_getres = 266, I386_clock_nanosleep = 267,
    I386_statfs64 = 268, I386_fstatfs64 = 269, I386_utimes = 271,
    I386_fadvise64_64 = 272, I386_waitid = 284, I386_futimesat = 299,
    I386_fstatat64 = 300, I386_pselect6 = 308, I386_ppoll = 309,
    I386_utimensat = 320, I386_signalfd = 321, I386_fallocate = 324,
    I386_timerfd_settime = 325, I386_timerfd_gettime = 326,
    I386_preadv = 333, I386_pwritev = 334, I386_socket = 359,
    I386_socketpair = 360, I386_bind = 361, I386_connect = 362,
    I386_listen = 363, I386_accept4 = 364, I386_getsockopt = 365,
    I386_setsockopt = 366, I386_getsockname = 367, I386_getpeername = 368,
    I386_sendto = 369, I386_sendmsg = 370, I386_recvfrom = 371,
    I386_recvmsg = 372, I386_shutdown = 373, I386_shmget = 395,
    I386_shmctl = 396, I386_shmat = 397, I386_shmdt = 398,
    I386_pselect6_time64 = 413, I386_rt_sigtimedwait_time64 = 421,
    I386_clone3 = 435,
    I386_NR_MAX = 448,
};

/* i386 number -> x86_64 number, for calls whose arguments mean the same
 * thing in both ABIs once widened. sext: bit n set = argument n+1 is a
 * signed int (descriptor, pid, timeout, offset) and is sign-extended. */
typedef struct {
    uint16_t i386;
    uint16_t x64;
    uint8_t  sext;
} i386_direct_t;

static const i386_direct_t g_i386_direct[] = {
    {2,   X64_fork, 0},           {3,   X64_read, 0x01},
    {4,   X64_write, 0x01},       {5,   X64_open, 0},
    {6,   X64_close, 0x01},       {8,   X64_creat, 0},
    {9,   X64_link, 0},           {10,  X64_unlink, 0},
    {12,  X64_chdir, 0},          {14,  X64_mknod, 0},
    {15,  X64_chmod, 0},          {20,  X64_getpid, 0},
    {21,  X64_mount, 0},          {26,  X64_ptrace, 0x02},
    {27,  X64_alarm, 0},          {29,  X64_pause, 0},
    {33,  X64_access, 0},         {36,  X64_sync, 0},
    {37,  X64_kill, 0x01},        {38,  X64_rename, 0},
    {39,  X64_mkdir, 0},          {40,  X64_rmdir, 0},
    {41,  X64_dup, 0x01},         {42,  X64_pipe, 0},
    {45,  X64_brk, 0},            {52,  X64_umount2, 0},
    {54,  X64_ioctl, 0x01},       {57,  X64_setpgid, 0x03},
    {60,  X64_umask, 0},          {61,  X64_chroot, 0},
    {63,  X64_dup2, 0x03},        {64,  X64_getppid, 0},
    {65,  X64_getpgrp, 0},        {66,  X64_setsid, 0},
    {83,  X64_symlink, 0},        {85,  X64_readlink, 0},
    {91,  X64_munmap, 0},         {94,  X64_fchmod, 0x01},
    {96,  X64_getpriority, 0x02}, {97,  X64_setpriority, 0x06},
    {103, X64_syslog, 0},         {118, X64_fsync, 0x01},
    {122, X64_uname, 0},          {125, X64_mprotect, 0},
    {132, X64_getpgid, 0x01},     {133, X64_fchdir, 0x01},
    {136, X64_personality, 0},    {143, X64_flock, 0x01},
    {144, X64_msync, 0},          {147, X64_getsid, 0x01},
    {148, X64_fdatasync, 0x01},   {150, X64_mlock, 0},
    {151, X64_munlock, 0},        {152, X64_mlockall, 0},
    {153, X64_munlockall, 0},     {154, X64_sched_setparam, 0x01},
    {155, X64_sched_getparam, 0x01}, {156, X64_sched_setscheduler, 0x01},
    {157, X64_sched_getscheduler, 0x01}, {158, X64_sched_yield, 0},
    {159, X64_sched_get_priority_max, 0x01},
    {160, X64_sched_get_priority_min, 0x01},
    {163, X64_mremap, 0},         {168, X64_poll, 0x04},
    {172, X64_prctl, 0},          {175, X64_rt_sigprocmask, 0},
    {176, X64_rt_sigpending, 0},  {179, X64_rt_sigsuspend, 0},
    {183, X64_getcwd, 0},         {184, X64_capget, 0},
    {185, X64_capset, 0},         {190, X64_vfork, 0},
    {198, X64_lchown, 0},         {199, X64_getuid, 0},
    {200, X64_getgid, 0},         {201, X64_geteuid, 0},
    {202, X64_getegid, 0},        {203, X64_setreuid, 0},
    {204, X64_setregid, 0},       {205, X64_getgroups, 0x01},
    {206, X64_setgroups, 0x01},   {207, X64_fchown, 0x01},
    {208, X64_setresuid, 0},      {209, X64_getresuid, 0},
    {210, X64_setresgid, 0},      {211, X64_getresgid, 0},
    {212, X64_chown, 0},          {213, X64_setuid, 0},
    {214, X64_setgid, 0},         {217, X64_pivot_root, 0},
    {218, X64_mincore, 0},        {219, X64_madvise, 0},
    {220, X64_getdents64, 0x01},  {224, X64_gettid, 0},
    {238, X64_tkill, 0x01},       {239, X64_sendfile, 0x03},
    {241, X64_sched_setaffinity, 0x01},
    {242, X64_sched_getaffinity, 0x01},
    {252, X64_exit_group, 0x01},  {255, X64_epoll_ctl, 0x05},
    {256, X64_epoll_wait, 0x09},  {258, X64_set_tid_address, 0},
    {270, X64_tgkill, 0x03},      {291, X64_inotify_init, 0},
    {292, X64_inotify_add_watch, 0x01}, {293, X64_inotify_rm_watch, 0x03},
    {295, X64_openat, 0x01},      {296, X64_mkdirat, 0x01},
    {297, X64_mknodat, 0x01},     {298, X64_fchownat, 0x01},
    {301, X64_unlinkat, 0x01},    {302, X64_renameat, 0x05},
    {303, X64_linkat, 0x05},      {304, X64_symlinkat, 0x02},
    {305, X64_readlinkat, 0x01},  {306, X64_fchmodat, 0x01},
    {307, X64_faccessat, 0x01},   {310, X64_unshare, 0},
    {318, X64_getcpu, 0},         {319, X64_epoll_pwait, 0x09},
    {322, X64_timerfd_create, 0}, {323, X64_eventfd, 0},
    {327, X64_signalfd4, 0x01},   {328, X64_eventfd2, 0},
    {329, X64_epoll_create1, 0},  {330, X64_dup3, 0x03},
    {331, X64_pipe2, 0},          {332, X64_inotify_init1, 0},
    {340, X64_prlimit64, 0x01},   {341, X64_name_to_handle_at, 0x11},
    {344, X64_syncfs, 0x01},      {346, X64_setns, 0x01},
    {351, X64_sched_setattr, 0x01}, {352, X64_sched_getattr, 0x01},
    {353, X64_renameat2, 0x05},   {354, X64_seccomp, 0},
    {355, X64_getrandom, 0},      {356, X64_memfd_create, 0},
    {359, X64_socket, 0},         {360, X64_socketpair, 0},
    {361, X64_bind, 0x01},        {362, X64_connect, 0x01},
    {363, X64_listen, 0x01},      {364, X64_accept4, 0x01},
    {367, X64_getsockname, 0x01}, {368, X64_getpeername, 0x01},
    {369, X64_sendto, 0x01},      {371, X64_recvfrom, 0x01},
    {373, X64_shutdown, 0x01},    {375, X64_membarrier, 0},
    {376, X64_mlock2, 0},         {377, X64_copy_file_range, 0x05},
    {383, X64_statx, 0x01},       {395, X64_shmget, 0},
    {397, X64_shmat, 0},          {398, X64_shmdt, 0},
    /* The *_time64 calls: 64-bit time_t, so the x86_64 structures. */
    {403, X64_clock_gettime, 0x01}, {406, X64_clock_getres, 0x01},
    {407, X64_clock_nanosleep, 0x01}, {410, X64_timerfd_gettime, 0x01},
    {411, X64_timerfd_settime, 0x01}, {412, X64_utimensat, 0x01},
    {414, X64_ppoll, 0},          {422, X64_futex, 0},
    {436, X64_close_range, 0},    {437, X64_openat2, 0x01},
    {439, X64_faccessat2, 0x01},  {441, X64_epoll_pwait2, 0x01},
};

typedef struct {
    uint16_t x64;
    uint8_t  sext;
    uint8_t  valid;
} i386_map_t;

static i386_map_t g_i386_map[I386_NR_MAX];

/* ---- ILP32 structures ------------------------------------------------ */

typedef struct { int32_t tv_sec; int32_t tv_nsec; } ts32_t;
typedef struct { int64_t tv_sec; int64_t tv_nsec; } ts64_t;
typedef struct { int32_t tv_sec; int32_t tv_usec; } tv32_t;
typedef struct { int64_t tv_sec; int64_t tv_usec; } tv64_t;
typedef struct { uint32_t base; uint32_t len; } iovec32_t;
typedef struct { uint64_t base; uint64_t len; } iovec64_t;

typedef struct __attribute__((packed)) {
    uint64_t st_dev;
    uint8_t  __pad0[4];
    uint32_t __st_ino;
    uint32_t st_mode;
    uint32_t st_nlink;
    uint32_t st_uid;
    uint32_t st_gid;
    uint64_t st_rdev;
    uint8_t  __pad3[4];
    int64_t  st_size;
    uint32_t st_blksize;
    uint64_t st_blocks;
    uint32_t st_atime_sec;
    uint32_t st_atime_nsec;
    uint32_t st_mtime_sec;
    uint32_t st_mtime_nsec;
    uint32_t st_ctime_sec;
    uint32_t st_ctime_nsec;
    uint64_t st_ino;
} stat64_32_t;
_Static_assert(sizeof(stat64_32_t) == 96, "i386 struct stat64");

typedef struct {
    uint32_t st_dev;
    uint32_t st_ino;
    uint16_t st_mode;
    uint16_t st_nlink;
    uint16_t st_uid;
    uint16_t st_gid;
    uint32_t st_rdev;
    uint32_t st_size;
    uint32_t st_blksize;
    uint32_t st_blocks;
    uint32_t st_atime_sec;
    uint32_t st_atime_nsec;
    uint32_t st_mtime_sec;
    uint32_t st_mtime_nsec;
    uint32_t st_ctime_sec;
    uint32_t st_ctime_nsec;
    uint32_t __unused4;
    uint32_t __unused5;
} stat32_t;
_Static_assert(sizeof(stat32_t) == 64, "i386 struct stat");

typedef struct {
    int64_t f_type, f_bsize, f_blocks, f_bfree, f_bavail, f_files, f_ffree;
    int32_t f_fsid[2];
    int64_t f_namelen, f_frsize, f_flags, f_spare[4];
} statfs64_x64_t;

typedef struct {
    uint32_t f_type, f_bsize, f_blocks, f_bfree, f_bavail, f_files, f_ffree;
    int32_t  f_fsid[2];
    uint32_t f_namelen, f_frsize, f_flags, f_spare[4];
} statfs32_t;

typedef struct __attribute__((packed)) {
    uint32_t f_type, f_bsize;
    uint64_t f_blocks, f_bfree, f_bavail, f_files, f_ffree;
    int32_t  f_fsid[2];
    uint32_t f_namelen, f_frsize, f_flags, f_spare[4];
} statfs64_32_t;
_Static_assert(sizeof(statfs64_32_t) == 84, "i386 struct statfs64");

typedef struct {
    tv64_t ru_utime, ru_stime;
    int64_t rest[14];
} rusage64_t;

typedef struct {
    tv32_t ru_utime, ru_stime;
    int32_t rest[14];
} rusage32_t;

typedef struct {
    uint64_t handler, flags, restorer, mask;
} sigaction64_t;

typedef struct {
    uint32_t handler, flags, restorer, mask[2];
} sigaction32_t;

typedef struct {
    uint32_t handler, mask, flags, restorer;
} old_sigaction32_t;

typedef struct {
    int16_t l_type, l_whence;
    int32_t __pad;
    int64_t l_start, l_len;
    int32_t l_pid, __pad2;
} flock64_x64_t;

typedef struct {
    int16_t l_type, l_whence;
    int32_t l_start, l_len, l_pid;
} flock32_t;

typedef struct __attribute__((packed)) {
    int16_t l_type, l_whence;
    int64_t l_start, l_len;
    int32_t l_pid;
} flock64_32_t;
_Static_assert(sizeof(flock64_32_t) == 24, "i386 struct flock64 (packed)");

typedef struct {
    uint32_t name, namelen, iov, iovlen, control, controllen, flags;
} msghdr32_t;

typedef struct {
    uint64_t name;
    uint32_t namelen, __pad0;
    uint64_t iov, iovlen, control, controllen;
    uint32_t flags, __pad1;
} msghdr64_t;

/* ---- one call -------------------------------------------------------- */

typedef struct {
    uint64_t saved_rsp;
    linux_call_t call;
    uint64_t sw;
    lx_scratch_t scratch;
} i386_ctx_t;

static int64_t x64(i386_ctx_t *c, uint64_t num, uint64_t a1, uint64_t a2,
                   uint64_t a3, uint64_t a4, uint64_t a5, uint64_t a6)
{
    c->sw |= linux_syscall_core(c->saved_rsp, &c->call, num,
                                a1, a2, a3, a4, a5, a6);
    return c->call.result;
}

/* After a call into the shared implementation: 1 when its result may be
 * looked at (it completed and succeeded). */
#define DONE_OK(c, r) (!(c)->call.restarted && (r) >= 0)

static uint64_t scratch(i386_ctx_t *c, uint64_t size)
{
    return lx_scratch_alloc(&c->scratch, size);
}

static int get32(uint32_t uaddr, void *dst, uint64_t len)
{
    return copy_from_user(dst, (const void *)(uintptr_t)uaddr, len) == 0u ? 0 : -1;
}

static int put32(uint32_t uaddr, const void *src, uint64_t len)
{
    return copy_to_user((void *)(uintptr_t)uaddr, src, len) == 0u ? 0 : -1;
}

static int getk(uint64_t uaddr, void *dst, uint64_t len)
{
    return copy_from_user(dst, (const void *)(uintptr_t)uaddr, len) == 0u ? 0 : -1;
}

static int putk(uint64_t uaddr, const void *src, uint64_t len)
{
    return copy_to_user((void *)(uintptr_t)uaddr, src, len) == 0u ? 0 : -1;
}

/* A 32-bit structure at `uaddr` widened into scratch; *out = 0 for NULL. */
static int ts32_in(i386_ctx_t *c, uint32_t uaddr, uint64_t *out)
{
    *out = 0;
    if (uaddr == 0u) return 0;
    ts32_t t;
    if (get32(uaddr, &t, sizeof(t)) < 0) return LX_EFAULT;
    ts64_t w = { t.tv_sec, t.tv_nsec };
    *out = scratch(c, sizeof(w));
    return (*out != 0u && putk(*out, &w, sizeof(w)) == 0) ? 0 : LX_EFAULT;
}

static int ts32_out(uint64_t kaddr, uint32_t uaddr)
{
    if (kaddr == 0u || uaddr == 0u) return 0;
    ts64_t w;
    if (getk(kaddr, &w, sizeof(w)) < 0) return LX_EFAULT;
    ts32_t t = { (int32_t)w.tv_sec, (int32_t)w.tv_nsec };
    return put32(uaddr, &t, sizeof(t)) == 0 ? 0 : LX_EFAULT;
}

static int tv32_in(i386_ctx_t *c, uint32_t uaddr, uint64_t *out)
{
    *out = 0;
    if (uaddr == 0u) return 0;
    tv32_t t;
    if (get32(uaddr, &t, sizeof(t)) < 0) return LX_EFAULT;
    tv64_t w = { t.tv_sec, t.tv_usec };
    *out = scratch(c, sizeof(w));
    return (*out != 0u && putk(*out, &w, sizeof(w)) == 0) ? 0 : LX_EFAULT;
}

static int tv32_out(uint64_t kaddr, uint32_t uaddr)
{
    if (kaddr == 0u || uaddr == 0u) return 0;
    tv64_t w;
    if (getk(kaddr, &w, sizeof(w)) < 0) return LX_EFAULT;
    tv32_t t = { (int32_t)w.tv_sec, (int32_t)w.tv_usec };
    return put32(uaddr, &t, sizeof(t)) == 0 ? 0 : LX_EFAULT;
}

/* struct itimerspec / itimerval: two of the above back to back. */
static int pair32_in(i386_ctx_t *c, uint32_t uaddr, uint64_t *out)
{
    *out = 0;
    if (uaddr == 0u) return 0;
    int32_t t[4];
    if (get32(uaddr, t, sizeof(t)) < 0) return LX_EFAULT;
    int64_t w[4] = { t[0], t[1], t[2], t[3] };
    *out = scratch(c, sizeof(w));
    return (*out != 0u && putk(*out, w, sizeof(w)) == 0) ? 0 : LX_EFAULT;
}

static int pair32_out(uint64_t kaddr, uint32_t uaddr)
{
    if (kaddr == 0u || uaddr == 0u) return 0;
    int64_t w[4];
    if (getk(kaddr, w, sizeof(w)) < 0) return LX_EFAULT;
    int32_t t[4] = { (int32_t)w[0], (int32_t)w[1], (int32_t)w[2], (int32_t)w[3] };
    return put32(uaddr, t, sizeof(t)) == 0 ? 0 : LX_EFAULT;
}

static uint64_t out_buf(i386_ctx_t *c, uint32_t uaddr, uint64_t size)
{
    return uaddr != 0u ? scratch(c, size) : 0u;
}

/* ---- struct stat --------------------------------------------------- */

static int stat_fetch(uint64_t kaddr, lx_x64_stat_t *st)
{
    return getk(kaddr, st, sizeof(*st));
}

static int64_t stat64_out(uint64_t kaddr, uint32_t uaddr)
{
    lx_x64_stat_t in;
    if (stat_fetch(kaddr, &in) < 0) return LX_EFAULT;
    stat64_32_t o;
    memset(&o, 0, sizeof(o));
    o.st_dev = in.st_dev;
    o.__st_ino = (uint32_t)in.st_ino;
    o.st_mode = in.st_mode;
    o.st_nlink = (uint32_t)in.st_nlink;
    o.st_uid = in.st_uid;
    o.st_gid = in.st_gid;
    o.st_rdev = in.st_rdev;
    o.st_size = in.st_size;
    o.st_blksize = (uint32_t)in.st_blksize;
    o.st_blocks = (uint64_t)in.st_blocks;
    o.st_atime_sec = (uint32_t)in.st_atime_sec;
    o.st_atime_nsec = (uint32_t)in.st_atime_nsec;
    o.st_mtime_sec = (uint32_t)in.st_mtime_sec;
    o.st_mtime_nsec = (uint32_t)in.st_mtime_nsec;
    o.st_ctime_sec = (uint32_t)in.st_ctime_sec;
    o.st_ctime_nsec = (uint32_t)in.st_ctime_nsec;
    o.st_ino = in.st_ino;
    return put32(uaddr, &o, sizeof(o)) == 0 ? 0 : LX_EFAULT;
}

static int64_t stat32_out(uint64_t kaddr, uint32_t uaddr)
{
    lx_x64_stat_t in;
    if (stat_fetch(kaddr, &in) < 0) return LX_EFAULT;
    if (in.st_size > 0x7FFFFFFFLL || in.st_ino > 0xFFFFFFFFULL) {
        return LX_EOVERFLOW;
    }
    stat32_t o;
    memset(&o, 0, sizeof(o));
    o.st_dev = (uint32_t)in.st_dev;
    o.st_ino = (uint32_t)in.st_ino;
    o.st_mode = (uint16_t)in.st_mode;
    o.st_nlink = (uint16_t)in.st_nlink;
    o.st_uid = (uint16_t)(in.st_uid > 0xFFFFu ? 65534u : in.st_uid);
    o.st_gid = (uint16_t)(in.st_gid > 0xFFFFu ? 65534u : in.st_gid);
    o.st_rdev = (uint32_t)in.st_rdev;
    o.st_size = (uint32_t)in.st_size;
    o.st_blksize = (uint32_t)in.st_blksize;
    o.st_blocks = (uint32_t)in.st_blocks;
    o.st_atime_sec = (uint32_t)in.st_atime_sec;
    o.st_atime_nsec = (uint32_t)in.st_atime_nsec;
    o.st_mtime_sec = (uint32_t)in.st_mtime_sec;
    o.st_mtime_nsec = (uint32_t)in.st_mtime_nsec;
    o.st_ctime_sec = (uint32_t)in.st_ctime_sec;
    o.st_ctime_nsec = (uint32_t)in.st_ctime_nsec;
    return put32(uaddr, &o, sizeof(o)) == 0 ? 0 : LX_EFAULT;
}

/* One of the stat calls through x86_64 newfstatat()/fstat(). wide = stat64. */
static int64_t do_stat(i386_ctx_t *c, int fd_form, uint64_t dirfd,
                       uint32_t path, uint32_t buf, uint64_t flags, int wide)
{
    uint64_t k = scratch(c, sizeof(lx_x64_stat_t));
    if (k == 0u) return LX_EFAULT;
    int64_t r = fd_form ? x64(c, X64_fstat, dirfd, k, 0, 0, 0, 0)
                        : x64(c, X64_newfstatat, dirfd, path, k, flags, 0, 0);
    if (!DONE_OK(c, r)) return r;
    return wide ? stat64_out(k, buf) : stat32_out(k, buf);
}

#define AT_FDCWD_64 ((uint64_t)(int64_t)-100)
#define AT_SYMLINK_NOFOLLOW_ 0x100u

/* ---- statfs -------------------------------------------------------- */

static int64_t do_statfs(i386_ctx_t *c, int fd_form, uint64_t target,
                         uint32_t buf, int wide, uint32_t size)
{
    if (wide && size != sizeof(statfs64_32_t)) return LX_EINVAL;
    uint64_t k = scratch(c, sizeof(statfs64_x64_t));
    if (k == 0u) return LX_EFAULT;
    int64_t r = x64(c, fd_form ? X64_fstatfs : X64_statfs, target, k, 0, 0, 0, 0);
    if (!DONE_OK(c, r)) return r;
    statfs64_x64_t in;
    if (getk(k, &in, sizeof(in)) < 0) return LX_EFAULT;
    if (wide) {
        statfs64_32_t o;
        memset(&o, 0, sizeof(o));
        o.f_type = (uint32_t)in.f_type;
        o.f_bsize = (uint32_t)in.f_bsize;
        o.f_blocks = (uint64_t)in.f_blocks;
        o.f_bfree = (uint64_t)in.f_bfree;
        o.f_bavail = (uint64_t)in.f_bavail;
        o.f_files = (uint64_t)in.f_files;
        o.f_ffree = (uint64_t)in.f_ffree;
        o.f_fsid[0] = in.f_fsid[0];
        o.f_fsid[1] = in.f_fsid[1];
        o.f_namelen = (uint32_t)in.f_namelen;
        o.f_frsize = (uint32_t)in.f_frsize;
        o.f_flags = (uint32_t)in.f_flags;
        return put32(buf, &o, sizeof(o)) == 0 ? 0 : LX_EFAULT;
    }
    statfs32_t o;
    memset(&o, 0, sizeof(o));
    o.f_type = (uint32_t)in.f_type;
    o.f_bsize = (uint32_t)in.f_bsize;
    o.f_blocks = (uint32_t)in.f_blocks;
    o.f_bfree = (uint32_t)in.f_bfree;
    o.f_bavail = (uint32_t)in.f_bavail;
    o.f_files = (uint32_t)in.f_files;
    o.f_ffree = (uint32_t)in.f_ffree;
    o.f_fsid[0] = in.f_fsid[0];
    o.f_fsid[1] = in.f_fsid[1];
    o.f_namelen = (uint32_t)in.f_namelen;
    o.f_frsize = (uint32_t)in.f_frsize;
    o.f_flags = (uint32_t)in.f_flags;
    return put32(buf, &o, sizeof(o)) == 0 ? 0 : LX_EFAULT;
}

/* ---- struct rusage, siginfo ------------------------------------------ */

static int rusage_out(uint64_t kaddr, uint32_t uaddr)
{
    if (kaddr == 0u || uaddr == 0u) return 0;
    rusage64_t in;
    if (getk(kaddr, &in, sizeof(in)) < 0) return LX_EFAULT;
    rusage32_t o;
    o.ru_utime.tv_sec = (int32_t)in.ru_utime.tv_sec;
    o.ru_utime.tv_usec = (int32_t)in.ru_utime.tv_usec;
    o.ru_stime.tv_sec = (int32_t)in.ru_stime.tv_sec;
    o.ru_stime.tv_usec = (int32_t)in.ru_stime.tv_usec;
    for (uint32_t i = 0; i < 14u; ++i) {
        o.rest[i] = (int32_t)in.rest[i];
    }
    return put32(uaddr, &o, sizeof(o)) == 0 ? 0 : LX_EFAULT;
}

/* siginfo_t: three ints, then a union that starts at 16 with 8-byte longs
 * and pointers on x86_64, at 12 with 4-byte ones on i386. */
static int siginfo_out(uint64_t kaddr, uint32_t uaddr)
{
    if (kaddr == 0u || uaddr == 0u) return 0;
    uint8_t in[128];
    uint8_t o[128];
    if (getk(kaddr, in, sizeof(in)) < 0) return LX_EFAULT;
    memset(o, 0, sizeof(o));
    memcpy(o, in, 12u);
    int32_t signo;
    memcpy(&signo, in, sizeof(signo));
    if (signo == 11 || signo == 7 || signo == 4 || signo == 8 || signo == 5) {
        uint64_t addr;
        memcpy(&addr, in + 16, sizeof(addr));
        uint32_t a32 = (uint32_t)addr;
        memcpy(o + 12, &a32, sizeof(a32));
    } else {
        memcpy(o + 12, in + 16, 8u);          /* si_pid, si_uid */
        if (signo == 17) {                    /* SIGCHLD */
            int64_t ut, st;
            memcpy(o + 20, in + 24, 4u);      /* si_status */
            memcpy(&ut, in + 32, sizeof(ut));
            memcpy(&st, in + 40, sizeof(st));
            int32_t ut32 = (int32_t)ut, st32 = (int32_t)st;
            memcpy(o + 24, &ut32, 4u);
            memcpy(o + 28, &st32, 4u);
        } else {
            uint64_t val;                     /* si_value */
            memcpy(&val, in + 24, sizeof(val));
            uint32_t v32 = (uint32_t)val;
            memcpy(o + 20, &v32, 4u);
        }
    }
    return put32(uaddr, o, sizeof(o)) == 0 ? 0 : LX_EFAULT;
}

/* ---- iovec / msghdr --------------------------------------------------- */

#define I386_IOV_MAX 1024u

static int iov_in(i386_ctx_t *c, uint32_t uiov, uint32_t cnt, uint64_t *out)
{
    *out = 0;
    if (cnt == 0u) return 0;
    if (cnt > I386_IOV_MAX) return LX_EINVAL;
    uint64_t k = scratch(c, (uint64_t)cnt * sizeof(iovec64_t));
    if (k == 0u) return LX_EFAULT;
    for (uint32_t i = 0; i < cnt; ++i) {
        iovec32_t v;
        if (get32(uiov + i * (uint32_t)sizeof(v), &v, sizeof(v)) < 0) return LX_EFAULT;
        iovec64_t w = { v.base, v.len };
        if (putk(k + (uint64_t)i * sizeof(w), &w, sizeof(w)) < 0) return LX_EFAULT;
    }
    *out = k;
    return 0;
}

#define CMSG32_ALIGN(n) (((n) + 3u) & ~3u)
#define CMSG64_ALIGN(n) (((n) + 7u) & ~(uint64_t)7u)

/* An i386 control buffer rebuilt with 16-byte x86_64 cmsg headers. */
static int cmsg_in(i386_ctx_t *c, uint32_t ctl, uint32_t len,
                   uint64_t *out, uint64_t *out_len)
{
    *out = 0;
    *out_len = 0;
    if (ctl == 0u || len == 0u) return 0;
    if (len > 4096u) return LX_EINVAL;
    uint8_t in[4096];
    uint8_t o[8192];
    if (get32(ctl, in, len) < 0) return LX_EFAULT;
    uint64_t used = 0;
    for (uint32_t pos = 0; pos + 12u <= len;) {
        uint32_t clen;
        int32_t level, type;
        memcpy(&clen, in + pos, 4u);
        memcpy(&level, in + pos + 4u, 4u);
        memcpy(&type, in + pos + 8u, 4u);
        if (clen < 12u || pos + clen > len) break;
        uint64_t data = clen - 12u;
        uint64_t len64 = 16u + data;
        if (used + CMSG64_ALIGN(len64) > sizeof(o)) return LX_EINVAL;
        memset(o + used, 0, (size_t)CMSG64_ALIGN(len64));
        memcpy(o + used, &len64, 8u);
        memcpy(o + used + 8u, &level, 4u);
        memcpy(o + used + 12u, &type, 4u);
        memcpy(o + used + 16u, in + pos + 12u, (size_t)data);
        used += CMSG64_ALIGN(len64);
        pos += CMSG32_ALIGN(clen);
    }
    uint64_t k = scratch(c, used != 0u ? used : 16u);
    if (k == 0u || (used != 0u && putk(k, o, used) < 0)) return LX_EFAULT;
    *out = k;
    *out_len = used;
    return 0;
}

/* Received x86_64 control messages back into the program's buffer;
 * MSG_CTRUNC when they do not all fit. */
static int cmsg_out(uint64_t k, uint64_t klen, uint32_t ctl, uint32_t cap,
                    uint32_t *out_len, uint32_t *flags)
{
    *out_len = 0;
    if (k == 0u || klen == 0u || ctl == 0u) return 0;
    if (klen > 8192u) klen = 8192u;
    uint8_t in[8192];
    uint8_t o[4096];
    if (getk(k, in, klen) < 0) return LX_EFAULT;
    uint32_t used = 0;
    for (uint64_t pos = 0; pos + 16u <= klen;) {
        uint64_t len64;
        memcpy(&len64, in + pos, 8u);
        if (len64 < 16u || pos + len64 > klen) break;
        uint32_t data = (uint32_t)(len64 - 16u);
        uint32_t clen = 12u + data;
        if (used + clen > cap || used + CMSG32_ALIGN(clen) > sizeof(o)) {
            *flags |= 0x08u; /* MSG_CTRUNC */
            break;
        }
        memset(o + used, 0, CMSG32_ALIGN(clen));
        memcpy(o + used, &clen, 4u);
        memcpy(o + used + 4u, in + pos + 8u, 8u);
        memcpy(o + used + 12u, in + pos + 16u, data);
        used += CMSG32_ALIGN(clen);
        pos += CMSG64_ALIGN(len64);
    }
    if (used > cap) used = cap;
    if (used != 0u && put32(ctl, o, used) < 0) return LX_EFAULT;
    *out_len = used;
    return 0;
}

static int64_t do_sendmsg(i386_ctx_t *c, uint32_t fd, uint32_t umsg, uint32_t flags)
{
    msghdr32_t m;
    if (get32(umsg, &m, sizeof(m)) < 0) return LX_EFAULT;
    msghdr64_t w;
    memset(&w, 0, sizeof(w));
    w.name = m.name;
    w.namelen = m.namelen;
    w.iovlen = m.iovlen;
    w.flags = m.flags;
    int rc = iov_in(c, m.iov, m.iovlen, &w.iov);
    if (rc < 0) return rc;
    rc = cmsg_in(c, m.control, m.controllen, &w.control, &w.controllen);
    if (rc < 0) return rc;
    uint64_t k = scratch(c, sizeof(w));
    if (k == 0u || putk(k, &w, sizeof(w)) < 0) return LX_EFAULT;
    return x64(c, X64_sendmsg, SX(fd), k, flags, 0, 0, 0);
}

static int64_t do_recvmsg(i386_ctx_t *c, uint32_t fd, uint32_t umsg, uint32_t flags)
{
    msghdr32_t m;
    if (get32(umsg, &m, sizeof(m)) < 0) return LX_EFAULT;
    msghdr64_t w;
    memset(&w, 0, sizeof(w));
    w.name = m.name;
    w.namelen = m.namelen;
    w.iovlen = m.iovlen;
    w.flags = m.flags;
    int rc = iov_in(c, m.iov, m.iovlen, &w.iov);
    if (rc < 0) return rc;
    if (m.control != 0u && m.controllen != 0u) {
        /* Room for every i386 message once its header has grown by 4 and
         * its padding to 8. */
        uint64_t cap = (uint64_t)m.controllen * 2u + 16u;
        if (cap > 8192u) cap = 8192u;
        w.control = scratch(c, cap);
        if (w.control == 0u) return LX_EFAULT;
        w.controllen = cap;
    }
    uint64_t k = scratch(c, sizeof(w));
    if (k == 0u || putk(k, &w, sizeof(w)) < 0) return LX_EFAULT;
    int64_t r = x64(c, X64_recvmsg, SX(fd), k, flags, 0, 0, 0);
    if (!DONE_OK(c, r)) return r;
    if (getk(k, &w, sizeof(w)) < 0) return LX_EFAULT;
    m.namelen = w.namelen;
    m.flags = w.flags;
    uint32_t clen = 0;
    if (cmsg_out(w.control, w.controllen, m.control, m.controllen, &clen,
                 &m.flags) < 0) {
        return LX_EFAULT;
    }
    m.controllen = clen;
    if (put32(umsg, &m, sizeof(m)) < 0) return LX_EFAULT;
    return r;
}

/* SO_RCVTIMEO / SO_SNDTIMEO carry a struct timeval. */
#define SOL_SOCKET_ 1u
#define SO_RCVTIMEO_OLD 20u
#define SO_SNDTIMEO_OLD 21u
#define SO_RCVTIMEO_NEW 66u
#define SO_SNDTIMEO_NEW 67u

static int64_t do_setsockopt(i386_ctx_t *c, uint32_t fd, uint32_t level,
                             uint32_t name, uint32_t val, uint32_t len)
{
    if (level == SOL_SOCKET_ && (name == SO_RCVTIMEO_NEW || name == SO_SNDTIMEO_NEW)) {
        name = name == SO_RCVTIMEO_NEW ? SO_RCVTIMEO_OLD : SO_SNDTIMEO_OLD;
        return x64(c, X64_setsockopt, SX(fd), level, name, val, len, 0);
    }
    if (level == SOL_SOCKET_ && (name == SO_RCVTIMEO_OLD || name == SO_SNDTIMEO_OLD)) {
        if (len < sizeof(tv32_t)) return LX_EINVAL;
        uint64_t k = 0;
        int rc = tv32_in(c, val, &k);
        if (rc < 0) return rc;
        return x64(c, X64_setsockopt, SX(fd), level, name, k, sizeof(tv64_t), 0);
    }
    return x64(c, X64_setsockopt, SX(fd), level, name, val, len, 0);
}

static int64_t do_getsockopt(i386_ctx_t *c, uint32_t fd, uint32_t level,
                             uint32_t name, uint32_t val, uint32_t ulen)
{
    if (level == SOL_SOCKET_ && (name == SO_RCVTIMEO_NEW || name == SO_SNDTIMEO_NEW)) {
        name = name == SO_RCVTIMEO_NEW ? SO_RCVTIMEO_OLD : SO_SNDTIMEO_OLD;
        return x64(c, X64_getsockopt, SX(fd), level, name, val, ulen, 0);
    }
    if (level == SOL_SOCKET_ && (name == SO_RCVTIMEO_OLD || name == SO_SNDTIMEO_OLD)) {
        uint32_t len = 0;
        if (get32(ulen, &len, sizeof(len)) < 0) return LX_EFAULT;
        if (len < sizeof(tv32_t)) return LX_EINVAL;
        uint64_t kv = scratch(c, sizeof(tv64_t));
        uint64_t kl = scratch(c, sizeof(uint32_t));
        uint32_t klen = sizeof(tv64_t);
        if (kv == 0u || kl == 0u || putk(kl, &klen, sizeof(klen)) < 0) return LX_EFAULT;
        int64_t r = x64(c, X64_getsockopt, SX(fd), level, name, kv, kl, 0);
        if (!DONE_OK(c, r)) return r;
        if (tv32_out(kv, val) < 0) return LX_EFAULT;
        len = sizeof(tv32_t);
        return put32(ulen, &len, sizeof(len)) == 0 ? r : LX_EFAULT;
    }
    return x64(c, X64_getsockopt, SX(fd), level, name, val, ulen, 0);
}

/* ---- signals --------------------------------------------------------- */

static int64_t do_rt_sigaction(i386_ctx_t *c, uint32_t sig, uint32_t act,
                               uint32_t oact, uint32_t setsize)
{
    uint64_t kact = 0, koact = 0;
    if (act != 0u) {
        sigaction32_t a;
        if (get32(act, &a, sizeof(a)) < 0) return LX_EFAULT;
        sigaction64_t w = { a.handler, a.flags, a.restorer,
                            LOHI(a.mask[0], a.mask[1]) };
        kact = scratch(c, sizeof(w));
        if (kact == 0u || putk(kact, &w, sizeof(w)) < 0) return LX_EFAULT;
    }
    koact = out_buf(c, oact, sizeof(sigaction64_t));
    int64_t r = x64(c, X64_rt_sigaction, sig, kact, koact, setsize, 0, 0);
    if (!DONE_OK(c, r) || koact == 0u) return r;
    sigaction64_t w;
    if (getk(koact, &w, sizeof(w)) < 0) return LX_EFAULT;
    sigaction32_t a = { (uint32_t)w.handler, (uint32_t)w.flags,
                        (uint32_t)w.restorer,
                        { (uint32_t)w.mask, (uint32_t)(w.mask >> 32) } };
    return put32(oact, &a, sizeof(a)) == 0 ? r : LX_EFAULT;
}

static int64_t do_old_sigaction(i386_ctx_t *c, uint32_t sig, uint32_t act,
                                uint32_t oact)
{
    uint64_t kact = 0;
    if (act != 0u) {
        old_sigaction32_t a;
        if (get32(act, &a, sizeof(a)) < 0) return LX_EFAULT;
        sigaction64_t w = { a.handler, a.flags, a.restorer, a.mask };
        kact = scratch(c, sizeof(w));
        if (kact == 0u || putk(kact, &w, sizeof(w)) < 0) return LX_EFAULT;
    }
    uint64_t koact = out_buf(c, oact, sizeof(sigaction64_t));
    int64_t r = x64(c, X64_rt_sigaction, sig, kact, koact, 8, 0, 0);
    if (!DONE_OK(c, r) || koact == 0u) return r;
    sigaction64_t w;
    if (getk(koact, &w, sizeof(w)) < 0) return LX_EFAULT;
    old_sigaction32_t a = { (uint32_t)w.handler, (uint32_t)w.mask,
                            (uint32_t)w.flags, (uint32_t)w.restorer };
    return put32(oact, &a, sizeof(a)) == 0 ? r : LX_EFAULT;
}

static int64_t do_sigprocmask(i386_ctx_t *c, uint32_t how, uint32_t set,
                              uint32_t oset)
{
    /* Only the low 32 signals are visible to this call; the others keep
     * their state. */
    uint64_t kcur = scratch(c, 8u);
    if (kcur == 0u) return LX_EFAULT;
    int64_t r = x64(c, X64_rt_sigprocmask, 0 /* SIG_BLOCK */, 0, kcur, 8, 0, 0);
    if (!DONE_OK(c, r)) return r;
    uint64_t cur;
    if (getk(kcur, &cur, sizeof(cur)) < 0) return LX_EFAULT;
    if (set != 0u) {
        uint32_t s32;
        if (get32(set, &s32, sizeof(s32)) < 0) return LX_EFAULT;
        uint64_t s64 = how == 2u /* SIG_SETMASK */ ?
            ((cur & ~0xFFFFFFFFULL) | s32) : (uint64_t)s32;
        uint64_t kset = scratch(c, 8u);
        if (kset == 0u || putk(kset, &s64, sizeof(s64)) < 0) return LX_EFAULT;
        r = x64(c, X64_rt_sigprocmask, how, kset, 0, 8, 0, 0);
        if (!DONE_OK(c, r)) return r;
    }
    if (oset != 0u) {
        uint32_t o32 = (uint32_t)cur;
        if (put32(oset, &o32, sizeof(o32)) < 0) return LX_EFAULT;
    }
    return 0;
}

static int64_t do_sigaltstack(i386_ctx_t *c, uint32_t ss, uint32_t oss)
{
    uint64_t kss = 0, koss = out_buf(c, oss, 24u);
    if (ss != 0u) {
        uint32_t s[3];
        if (get32(ss, s, sizeof(s)) < 0) return LX_EFAULT;
        uint64_t w[3] = { s[0], (uint64_t)(uint32_t)s[1], s[2] };
        kss = scratch(c, sizeof(w));
        if (kss == 0u || putk(kss, w, sizeof(w)) < 0) return LX_EFAULT;
    }
    int64_t r = x64(c, X64_sigaltstack, kss, koss, 0, 0, 0, 0);
    if (!DONE_OK(c, r) || koss == 0u) return r;
    uint64_t w[3];
    if (getk(koss, w, sizeof(w)) < 0) return LX_EFAULT;
    uint32_t s[3] = { (uint32_t)w[0], (uint32_t)w[1], (uint32_t)w[2] };
    return put32(oss, s, sizeof(s)) == 0 ? r : LX_EFAULT;
}

/* ---- fcntl locks ----------------------------------------------------- */

static int64_t do_fcntl(i386_ctx_t *c, uint32_t fd, uint32_t cmd, uint32_t arg)
{
    int wide = 0;
    uint32_t xcmd = cmd;
    switch (cmd) {
    case 12u: xcmd = 5u; wide = 1; break;      /* F_GETLK64 */
    case 13u: xcmd = 6u; wide = 1; break;      /* F_SETLK64 */
    case 14u: xcmd = 7u; wide = 1; break;      /* F_SETLKW64 */
    case 36u: case 37u: case 38u: wide = 1; break; /* F_OFD_* */
    case 5u: case 6u: case 7u: break;
    default:
        /* Every other command takes an int (F_SETOWN's may be negative)
         * or a structure that is the same in both ABIs. */
        return x64(c, X64_fcntl, SX(fd), cmd, SX(arg), 0, 0, 0);
    }
    flock64_x64_t w;
    memset(&w, 0, sizeof(w));
    if (wide) {
        flock64_32_t l;
        if (get32(arg, &l, sizeof(l)) < 0) return LX_EFAULT;
        w.l_type = l.l_type; w.l_whence = l.l_whence;
        w.l_start = l.l_start; w.l_len = l.l_len; w.l_pid = l.l_pid;
    } else {
        flock32_t l;
        if (get32(arg, &l, sizeof(l)) < 0) return LX_EFAULT;
        w.l_type = l.l_type; w.l_whence = l.l_whence;
        w.l_start = l.l_start; w.l_len = l.l_len; w.l_pid = l.l_pid;
    }
    uint64_t k = scratch(c, sizeof(w));
    if (k == 0u || putk(k, &w, sizeof(w)) < 0) return LX_EFAULT;
    int64_t r = x64(c, X64_fcntl, SX(fd), xcmd, k, 0, 0, 0);
    if (!DONE_OK(c, r)) return r;
    if (getk(k, &w, sizeof(w)) < 0) return LX_EFAULT;
    if (wide) {
        flock64_32_t l = { w.l_type, w.l_whence, w.l_start, w.l_len, w.l_pid };
        return put32(arg, &l, sizeof(l)) == 0 ? r : LX_EFAULT;
    }
    if (w.l_start > 0x7FFFFFFFLL || w.l_len > 0x7FFFFFFFLL) return LX_EOVERFLOW;
    flock32_t l = { w.l_type, w.l_whence, (int32_t)w.l_start, (int32_t)w.l_len,
                    w.l_pid };
    return put32(arg, &l, sizeof(l)) == 0 ? r : LX_EFAULT;
}

/* ---- rlimit ---------------------------------------------------------- */

static int64_t do_getrlimit(i386_ctx_t *c, uint32_t res, uint32_t ulim)
{
    uint64_t k = scratch(c, 16u);
    if (k == 0u) return LX_EFAULT;
    int64_t r = x64(c, X64_prlimit64, 0, res, 0, k, 0, 0);
    if (!DONE_OK(c, r)) return r;
    uint64_t w[2];
    if (getk(k, w, sizeof(w)) < 0) return LX_EFAULT;
    uint32_t l[2] = { w[0] >= 0xFFFFFFFFULL ? 0xFFFFFFFFu : (uint32_t)w[0],
                      w[1] >= 0xFFFFFFFFULL ? 0xFFFFFFFFu : (uint32_t)w[1] };
    return put32(ulim, l, sizeof(l)) == 0 ? 0 : LX_EFAULT;
}

static int64_t do_setrlimit(i386_ctx_t *c, uint32_t res, uint32_t ulim)
{
    uint32_t l[2];
    if (get32(ulim, l, sizeof(l)) < 0) return LX_EFAULT;
    uint64_t w[2] = { l[0] == 0xFFFFFFFFu ? ~0ULL : l[0],
                      l[1] == 0xFFFFFFFFu ? ~0ULL : l[1] };
    uint64_t k = scratch(c, sizeof(w));
    if (k == 0u || putk(k, w, sizeof(w)) < 0) return LX_EFAULT;
    return x64(c, X64_prlimit64, 0, res, k, 0, 0, 0);
}

/* ---- set_thread_area ------------------------------------------------- */

typedef struct {
    uint32_t entry_number;
    uint32_t base_addr;
    uint32_t limit;
    uint32_t flags; /* seg_32bit:1 contents:2 read_exec_only:1
                     * limit_in_pages:1 seg_not_present:1 useable:1 */
} user_desc_t;

#define UD_SEG_32BIT(f)      ((f) & 1u)
#define UD_CONTENTS(f)       (((f) >> 1) & 3u)
#define UD_READ_EXEC_ONLY(f) (((f) >> 3) & 1u)
#define UD_LIMIT_IN_PAGES(f) (((f) >> 4) & 1u)
#define UD_NOT_PRESENT(f)    (((f) >> 5) & 1u)
#define UD_USEABLE(f)        (((f) >> 6) & 1u)

static int user_desc_is_empty(const user_desc_t *u)
{
    if (u->base_addr == 0u && u->limit == 0u && (u->flags & 0x7Fu) == 0u) {
        return 1; /* "LDT_zero" */
    }
    return u->base_addr == 0u && u->limit == 0u && UD_CONTENTS(u->flags) == 0u &&
           UD_READ_EXEC_ONLY(u->flags) == 1u && UD_SEG_32BIT(u->flags) == 0u &&
           UD_LIMIT_IN_PAGES(u->flags) == 0u && UD_NOT_PRESENT(u->flags) == 1u &&
           UD_USEABLE(u->flags) == 0u;
}

/* arch/x86/include/asm/desc.h fill_ldt(): a DPL-3 data segment. */
static uint64_t user_desc_encode(const user_desc_t *u)
{
    uint64_t type = ((uint64_t)(UD_READ_EXEC_ONLY(u->flags) ^ 1u) << 1) |
                    ((uint64_t)UD_CONTENTS(u->flags) << 2) | 1u /* accessed */;
    return ((uint64_t)u->limit & 0xFFFFu) |
           (((uint64_t)u->base_addr & 0xFFFFFFu) << 16) |
           (type << 40) | (1ULL << 44) | (3ULL << 45) |
           ((uint64_t)(UD_NOT_PRESENT(u->flags) ^ 1u) << 47) |
           ((((uint64_t)u->limit >> 16) & 0xFu) << 48) |
           ((uint64_t)UD_USEABLE(u->flags) << 52) |
           ((uint64_t)UD_SEG_32BIT(u->flags) << 54) |
           ((uint64_t)UD_LIMIT_IN_PAGES(u->flags) << 55) |
           (((uint64_t)u->base_addr >> 24) << 56);
}

/* Applies `u` (entry -1 = pick a free one, if allowed) to `tls`. */
static int64_t tls_apply(uint64_t tls[3], user_desc_t *u, int can_allocate)
{
    if (u->entry_number == 0xFFFFFFFFu) {
        if (!can_allocate) return LX_EINVAL;
        for (uint32_t i = 0; i < GDT_TLS_ENTRIES; ++i) {
            if (tls[i] == 0u) {
                u->entry_number = GDT_TLS_ENTRY_MIN + i;
                break;
            }
        }
        if (u->entry_number == 0xFFFFFFFFu) return -3; /* ESRCH */
    }
    if (u->entry_number < GDT_TLS_ENTRY_MIN ||
        u->entry_number >= GDT_TLS_ENTRY_MIN + GDT_TLS_ENTRIES) {
        return LX_EINVAL;
    }
    uint32_t idx = u->entry_number - GDT_TLS_ENTRY_MIN;
    if (user_desc_is_empty(u)) {
        tls[idx] = 0;
        return 0;
    }
    /* Only 32-bit data segments (Linux tls_desc_okay()). */
    if (!UD_SEG_32BIT(u->flags) || UD_CONTENTS(u->flags) > 1u) {
        return LX_EINVAL;
    }
    tls[idx] = user_desc_encode(u);
    return 0;
}

static int64_t do_set_thread_area(uint32_t uinfo)
{
    user_desc_t u;
    if (get32(uinfo, &u, sizeof(u)) < 0) return LX_EFAULT;
    uint64_t tls[3];
    if (process_compat32_get_tls(tls) < 0) return LX_ESRCH;
    uint32_t requested = u.entry_number;
    int64_t r = tls_apply(tls, &u, 1);
    if (r < 0) return r;
    if (requested == 0xFFFFFFFFu &&
        put32(uinfo, &u.entry_number, sizeof(u.entry_number)) < 0) {
        return LX_EFAULT;
    }
    return process_compat32_set_tls(tls) == 0 ? 0 : LX_ESRCH;
}

static int64_t do_get_thread_area(uint32_t uinfo)
{
    user_desc_t u;
    if (get32(uinfo, &u.entry_number, sizeof(u.entry_number)) < 0) return LX_EFAULT;
    if (u.entry_number < GDT_TLS_ENTRY_MIN ||
        u.entry_number >= GDT_TLS_ENTRY_MIN + GDT_TLS_ENTRIES) {
        return LX_EINVAL;
    }
    uint64_t tls[3];
    if (process_compat32_get_tls(tls) < 0) return LX_ESRCH;
    uint64_t d = tls[u.entry_number - GDT_TLS_ENTRY_MIN];
    u.base_addr = (uint32_t)(((d >> 16) & 0xFFFFFFu) | (((d >> 56) & 0xFFu) << 24));
    u.limit = (uint32_t)((d & 0xFFFFu) | (((d >> 48) & 0xFu) << 16));
    uint32_t type = (uint32_t)((d >> 40) & 0xFu);
    u.flags = (uint32_t)((d >> 54) & 1u) |                       /* seg_32bit */
              (((type >> 2) & 3u) << 1) |                        /* contents */
              ((((type >> 1) & 1u) ^ 1u) << 3) |                 /* read_exec_only */
              ((uint32_t)((d >> 55) & 1u) << 4) |                /* limit_in_pages */
              (((uint32_t)((d >> 47) & 1u) ^ 1u) << 5) |         /* seg_not_present */
              ((uint32_t)((d >> 52) & 1u) << 6);                 /* useable */
    if (d == 0u) {
        u.flags = (1u << 3) | (1u << 5); /* the "empty" encoding */
    }
    return put32(uinfo, &u, sizeof(u)) == 0 ? 0 : LX_EFAULT;
}

/* ---- clone / execve -------------------------------------------------- */

static int64_t do_clone_thread(i386_ctx_t *c, uint64_t flags, uint64_t stack,
                               uint64_t ptid, uint64_t tls_ptr, uint64_t ctid)
{
    if (stack == 0u || (flags & LX_CLONE_VM) == 0u) {
        return LX_EINVAL;
    }
    uint64_t tls[3];
    if (process_compat32_get_tls(tls) < 0) return LX_ESRCH;
    if ((flags & LX_CLONE_SETTLS) != 0u) {
        user_desc_t u;
        if (get32((uint32_t)tls_ptr, &u, sizeof(u)) < 0) return LX_EFAULT;
        int64_t r = tls_apply(tls, &u, 0);
        if (r < 0) return r;
    }
    int32_t tid = process_create_thread_clone(c->saved_rsp, stack, 0, 0u, tls);
    return lx_thread_clone_finish(tid, flags, ptid, ctid);
}

static int ptr_array_in(i386_ctx_t *c, uint32_t uarr, uint64_t *out)
{
    *out = 0;
    if (uarr == 0u) return 0;
    uint32_t n = 0;
    for (;; ++n) {
        uint32_t p;
        if (n >= 1024u) return -7; /* E2BIG */
        if (get32(uarr + n * 4u, &p, sizeof(p)) < 0) return LX_EFAULT;
        if (p == 0u) break;
    }
    uint64_t k = scratch(c, ((uint64_t)n + 1u) * 8u);
    if (k == 0u) return LX_EFAULT;
    for (uint32_t i = 0; i <= n; ++i) {
        uint32_t p = 0;
        if (i < n && get32(uarr + i * 4u, &p, sizeof(p)) < 0) return LX_EFAULT;
        uint64_t w = p;
        if (putk(k + (uint64_t)i * 8u, &w, sizeof(w)) < 0) return LX_EFAULT;
    }
    *out = k;
    return 0;
}

/* ---- old-style readdir ----------------------------------------------- */

static int64_t do_getdents(i386_ctx_t *c, uint32_t fd, uint32_t ubuf, uint32_t count)
{
    if (count > 65536u) count = 65536u;
    uint64_t k = scratch(c, count);
    if (k == 0u) return LX_EFAULT;
    int64_t r = x64(c, X64_getdents64, SX(fd), k, count, 0, 0, 0);
    if (!DONE_OK(c, r)) return r;
    /* linux_dirent64 { u64 ino; s64 off; u16 reclen; u8 type; char name[]; }
     * -> linux_dirent { u32 ino; u32 off; u16 reclen; char name[]; 0; type } */
    uint32_t out = 0;
    for (uint64_t pos = 0; pos + 19u < (uint64_t)r;) {
        uint8_t hdr[19];
        if (getk(k + pos, hdr, sizeof(hdr)) < 0) return LX_EFAULT;
        uint64_t ino, off;
        uint16_t reclen;
        memcpy(&ino, hdr, 8u);
        memcpy(&off, hdr + 8, 8u);
        memcpy(&reclen, hdr + 16, 2u);
        uint8_t type = hdr[18];
        if (reclen < 20u || pos + reclen > (uint64_t)r) break;
        char name[256];
        uint32_t maxname = reclen - 19u;
        if (maxname > sizeof(name) - 1u) maxname = sizeof(name) - 1u;
        if (getk(k + pos + 19u, name, maxname) < 0) return LX_EFAULT;
        name[maxname] = '\0';
        uint32_t nlen = (uint32_t)strlen(name);
        uint32_t rec32 = (10u + nlen + 2u + 3u) & ~3u;
        if (out + rec32 > count) break;
        uint8_t o[280];
        memset(o, 0, sizeof(o));
        uint32_t ino32 = (uint32_t)ino, off32 = (uint32_t)off;
        uint16_t rl = (uint16_t)rec32;
        memcpy(o, &ino32, 4u);
        memcpy(o + 4, &off32, 4u);
        memcpy(o + 8, &rl, 2u);
        memcpy(o + 10, name, nlen);
        o[rec32 - 1u] = type;
        if (put32(ubuf + out, o, rec32) < 0) return LX_EFAULT;
        out += rec32;
        pos += reclen;
    }
    return out;
}

/* ---- the call table proper ------------------------------------------- */

static int64_t i386_call(i386_ctx_t *c, uint32_t num, const uint32_t a[6]);

/* socketcall(2): the multiplexed socket entry point older libcs use. */
static int64_t do_socketcall(i386_ctx_t *c, uint32_t call, uint32_t uargs)
{
    static const uint8_t nargs[21] = { 0, 3, 3, 3, 2, 3, 3, 3, 4, 4, 4, 6, 6,
                                       2, 5, 5, 3, 3, 4, 5, 4 };
    if (call < 1u || call > 20u) return LX_EINVAL;
    uint32_t v[6] = { 0 };
    if (get32(uargs, v, nargs[call] * 4u) < 0) return LX_EFAULT;
    switch (call) {
    case 1:  return i386_call(c, I386_socket, v);
    case 2:  return i386_call(c, I386_bind, v);
    case 3:  return i386_call(c, I386_connect, v);
    case 4:  return i386_call(c, I386_listen, v);
    case 5:  { uint32_t w[6] = { v[0], v[1], v[2], 0, 0, 0 };
               return i386_call(c, I386_accept4, w); }
    case 6:  return i386_call(c, I386_getsockname, v);
    case 7:  return i386_call(c, I386_getpeername, v);
    case 8:  return i386_call(c, I386_socketpair, v);
    case 9:  { uint32_t w[6] = { v[0], v[1], v[2], v[3], 0, 0 };
               return i386_call(c, I386_sendto, w); }
    case 10: { uint32_t w[6] = { v[0], v[1], v[2], v[3], 0, 0 };
               return i386_call(c, I386_recvfrom, w); }
    case 11: return i386_call(c, I386_sendto, v);
    case 12: return i386_call(c, I386_recvfrom, v);
    case 13: return i386_call(c, I386_shutdown, v);
    case 14: return i386_call(c, I386_setsockopt, v);
    case 15: return i386_call(c, I386_getsockopt, v);
    case 16: return i386_call(c, I386_sendmsg, v);
    case 17: return i386_call(c, I386_recvmsg, v);
    case 18: return i386_call(c, I386_accept4, v);
    default: return LX_ENOSYS; /* recvmmsg, sendmmsg */
    }
}

/* shmctl: i386 libcs add IPC_64 (0x100) to ask for the 64-bit-capable
 * structures; IPC_RMID needs none, and the structure-carrying commands are
 * not translated (MIT-SHM clients only remove their segments). */
static int64_t do_shmctl(i386_ctx_t *c, uint32_t id, uint32_t cmd, uint32_t buf)
{
    cmd &= ~0x100u;
    if (cmd != 0u /* IPC_RMID */) return LX_EINVAL;
    return x64(c, X64_shmctl, id, cmd, buf, 0, 0, 0);
}

/* ipc(2): the multiplexed SysV entry point (shared memory only). */
static int64_t do_ipc(i386_ctx_t *c, const uint32_t a[6])
{
    uint32_t call = a[0] & 0xFFFFu;
    switch (call) {
    case 21: { /* SHMAT: shmat(first, ptr, second), address stored at third */
        int64_t r = x64(c, X64_shmat, a[1], a[4], a[2], 0, 0, 0);
        if (!DONE_OK(c, r)) return r;
        uint32_t addr = (uint32_t)r;
        return put32(a[3], &addr, sizeof(addr)) == 0 ? 0 : LX_EFAULT;
    }
    case 22: return x64(c, X64_shmdt, a[4], 0, 0, 0, 0, 0);
    case 23: return x64(c, X64_shmget, SX(a[1]), a[2], a[3], 0, 0, 0);
    case 24: return do_shmctl(c, a[1], a[2], a[4]);
    default: return LX_ENOSYS;
    }
}

static int64_t do_mmap(i386_ctx_t *c, uint32_t addr, uint32_t len,
                       uint32_t prot, uint32_t flags, uint32_t fd,
                       uint64_t offset)
{
    int64_t r = x64(c, X64_mmap, addr, len, prot, flags, SX(fd), offset);
    if (DONE_OK(c, r) && (flags & 0x10u /* MAP_FIXED */) != 0u) {
        process_user_compat_claim((uint64_t)r, len);
    }
    return r;
}

static int64_t i386_call(i386_ctx_t *c, uint32_t num, const uint32_t a[6])
{
    uint64_t k = 0, k2 = 0;
    int rc;
    int64_t r;

    switch (num) {
    /* -- process ---------------------------------------------------- */
    case I386_exit:
        return x64(c, X64_exit, SX(a[0]), 0, 0, 0, 0, 0);

    case I386_execve: {
        uint64_t argv = 0, envp = 0;
        if ((rc = ptr_array_in(c, a[1], &argv)) < 0) return rc;
        if ((rc = ptr_array_in(c, a[2], &envp)) < 0) return rc;
        return x64(c, X64_execve, a[0], argv, envp, 0, 0, 0);
    }

    case I386_clone:
        /* clone(flags, stack, parent_tid, tls, child_tid) */
        if ((a[0] & LX_CLONE_THREAD) != 0u) {
            return do_clone_thread(c, a[0], a[1], a[2], a[3], a[4]);
        }
        return x64(c, X64_clone, a[0], a[1], a[2], a[4], 0, 0);

    case I386_clone3: {
        struct {
            uint64_t flags, pidfd, child_tid, parent_tid, exit_signal;
            uint64_t stack, stack_size, tls;
        } args;
        if (a[1] < sizeof(args) || get32(a[0], &args, sizeof(args)) < 0) {
            return LX_EFAULT;
        }
        if ((args.flags & LX_CLONE_THREAD) != 0u) {
            uint64_t top = args.stack_size != 0u ? args.stack + args.stack_size
                                                 : args.stack;
            return do_clone_thread(c, args.flags, top, args.parent_tid,
                                   args.tls, args.child_tid);
        }
        return x64(c, X64_clone3, a[0], a[1], 0, 0, 0, 0);
    }

    case I386_waitpid:
        return x64(c, X64_wait4, SX(a[0]), a[1], a[2], 0, 0, 0);

    case I386_wait4:
        k = out_buf(c, a[3], sizeof(rusage64_t));
        r = x64(c, X64_wait4, SX(a[0]), a[1], a[2], k, 0, 0);
        if (DONE_OK(c, r) && rusage_out(k, a[3]) < 0) return LX_EFAULT;
        return r;

    case I386_waitid:
        k = out_buf(c, a[2], 128u);
        k2 = out_buf(c, a[4], sizeof(rusage64_t));
        r = x64(c, X64_waitid, a[0], SX(a[1]), k, a[3], k2, 0);
        if (DONE_OK(c, r) &&
            (siginfo_out(k, a[2]) < 0 || rusage_out(k2, a[4]) < 0)) {
            return LX_EFAULT;
        }
        return r;

    case I386_set_thread_area:
        return do_set_thread_area(a[0]);
    case I386_get_thread_area:
        return do_get_thread_area(a[0]);

    /* -- signals ---------------------------------------------------- */
    case I386_sigreturn:
        return process_signal_sigreturn_compat32(c->saved_rsp, 0);
    case I386_rt_sigreturn:
        return process_signal_sigreturn_compat32(c->saved_rsp, 1);
    case I386_rt_sigaction:
        return do_rt_sigaction(c, a[0], a[1], a[2], a[3]);
    case I386_sigaction:
        return do_old_sigaction(c, a[0], a[1], a[2]);
    case I386_signal: {
        sigaction64_t w = { a[1], 0xC0000000u /* SA_ONESHOT|SA_NOMASK */, 0, 0 };
        k = scratch(c, sizeof(w));
        k2 = scratch(c, sizeof(w));
        if (k == 0u || k2 == 0u || putk(k, &w, sizeof(w)) < 0) return LX_EFAULT;
        r = x64(c, X64_rt_sigaction, a[0], k, k2, 8, 0, 0);
        if (!DONE_OK(c, r)) return r;
        if (getk(k2, &w, sizeof(w)) < 0) return LX_EFAULT;
        return (int64_t)(uint32_t)w.handler;
    }
    case I386_sigprocmask:
        return do_sigprocmask(c, a[0], a[1], a[2]);
    case I386_sigpending: {
        k = scratch(c, 8u);
        if (k == 0u) return LX_EFAULT;
        r = x64(c, X64_rt_sigpending, k, 8, 0, 0, 0, 0);
        uint64_t set;
        if (!DONE_OK(c, r)) return r;
        if (getk(k, &set, sizeof(set)) < 0) return LX_EFAULT;
        uint32_t s32 = (uint32_t)set;
        return put32(a[0], &s32, sizeof(s32)) == 0 ? 0 : LX_EFAULT;
    }
    case I386_sigsuspend: {
        uint64_t set = a[2];
        k = scratch(c, 8u);
        if (k == 0u || putk(k, &set, sizeof(set)) < 0) return LX_EFAULT;
        return x64(c, X64_rt_sigsuspend, k, 8, 0, 0, 0, 0);
    }
    case I386_sigaltstack:
        return do_sigaltstack(c, a[0], a[1]);
    case I386_rt_sigtimedwait:
    case I386_rt_sigtimedwait_time64:
        if (num == I386_rt_sigtimedwait) {
            if ((rc = ts32_in(c, a[2], &k2)) < 0) return rc;
        } else {
            k2 = a[2];
        }
        k = out_buf(c, a[1], 128u);
        r = x64(c, X64_rt_sigtimedwait, a[0], k, k2, a[3], 0, 0);
        if (DONE_OK(c, r) && siginfo_out(k, a[1]) < 0) return LX_EFAULT;
        return r;
    case I386_signalfd:
        return x64(c, X64_signalfd4, SX(a[0]), a[1], a[2], 0, 0, 0);

    /* -- files ------------------------------------------------------ */
    case I386_lseek: {
        r = x64(c, X64_lseek, SX(a[0]), SX(a[1]), a[2], 0, 0, 0);
        if (DONE_OK(c, r) && r > 0x7FFFFFFFLL) return LX_EOVERFLOW;
        return r;
    }
    case I386_llseek: {
        r = x64(c, X64_lseek, SX(a[0]), LOHI(a[2], a[1]), a[4], 0, 0, 0);
        if (!DONE_OK(c, r)) return r;
        uint64_t pos = (uint64_t)r;
        return put32(a[3], &pos, sizeof(pos)) == 0 ? 0 : LX_EFAULT;
    }
    case I386_pread64:
        return x64(c, X64_pread64, SX(a[0]), a[1], a[2], LOHI(a[3], a[4]), 0, 0);
    case I386_pwrite64:
        return x64(c, X64_pwrite64, SX(a[0]), a[1], a[2], LOHI(a[3], a[4]), 0, 0);
    case I386_readv:
    case I386_writev:
        if ((rc = iov_in(c, a[1], a[2], &k)) < 0) return rc;
        return x64(c, num == I386_readv ? X64_readv : X64_writev,
                   SX(a[0]), k, a[2], 0, 0, 0);
    case I386_preadv:
    case I386_pwritev:
        if ((rc = iov_in(c, a[1], a[2], &k)) < 0) return rc;
        return x64(c, num == I386_preadv ? X64_preadv : X64_pwritev,
                   SX(a[0]), k, a[2], LOHI(a[3], a[4]), 0, 0);
    case I386_sendfile: {
        if (a[2] != 0u) {
            int32_t off32;
            if (get32(a[2], &off32, sizeof(off32)) < 0) return LX_EFAULT;
            int64_t off = off32;
            k = scratch(c, 8u);
            if (k == 0u || putk(k, &off, sizeof(off)) < 0) return LX_EFAULT;
        }
        r = x64(c, X64_sendfile, SX(a[0]), SX(a[1]), k, a[3], 0, 0);
        if (DONE_OK(c, r) && k != 0u) {
            int64_t off;
            if (getk(k, &off, sizeof(off)) < 0) return LX_EFAULT;
            int32_t off32 = (int32_t)off;
            if (put32(a[2], &off32, sizeof(off32)) < 0) return LX_EFAULT;
        }
        return r;
    }
    case I386_truncate:
        return x64(c, X64_truncate, a[0], SX(a[1]), 0, 0, 0, 0);
    case I386_ftruncate:
        return x64(c, X64_ftruncate, SX(a[0]), SX(a[1]), 0, 0, 0, 0);
    case I386_truncate64:
        return x64(c, X64_truncate, a[0], LOHI(a[1], a[2]), 0, 0, 0, 0);
    case I386_ftruncate64:
        return x64(c, X64_ftruncate, SX(a[0]), LOHI(a[1], a[2]), 0, 0, 0, 0);
    case I386_fallocate:
        return x64(c, X64_fallocate, SX(a[0]), a[1], LOHI(a[2], a[3]),
                   LOHI(a[4], a[5]), 0, 0);
    case I386_fadvise64:
        return x64(c, X64_fadvise64, SX(a[0]), LOHI(a[1], a[2]), a[3], a[4], 0, 0);
    case I386_fadvise64_64:
        return x64(c, X64_fadvise64, SX(a[0]), LOHI(a[1], a[2]),
                   LOHI(a[3], a[4]), a[5], 0, 0);
    case I386_readahead:
        return x64(c, X64_readahead, SX(a[0]), LOHI(a[1], a[2]), a[3], 0, 0, 0);
    case I386_fcntl:
    case I386_fcntl64:
        return do_fcntl(c, a[0], a[1], a[2]);
    case I386_getdents:
        return do_getdents(c, a[0], a[1], a[2]);
    case I386_epoll_create:
        if ((int32_t)a[0] <= 0) return LX_EINVAL;
        return x64(c, X64_epoll_create1, 0, 0, 0, 0, 0, 0);

    case I386_stat:
        return do_stat(c, 0, AT_FDCWD_64, a[0], a[1], 0, 0);
    case I386_lstat:
        return do_stat(c, 0, AT_FDCWD_64, a[0], a[1], AT_SYMLINK_NOFOLLOW_, 0);
    case I386_fstat:
        return do_stat(c, 1, SX(a[0]), 0, a[1], 0, 0);
    case I386_stat64:
        return do_stat(c, 0, AT_FDCWD_64, a[0], a[1], 0, 1);
    case I386_lstat64:
        return do_stat(c, 0, AT_FDCWD_64, a[0], a[1], AT_SYMLINK_NOFOLLOW_, 1);
    case I386_fstat64:
        return do_stat(c, 1, SX(a[0]), 0, a[1], 0, 1);
    case I386_fstatat64:
        return do_stat(c, 0, SX(a[0]), a[1], a[2], a[3], 1);
    case I386_statfs:
        return do_statfs(c, 0, a[0], a[1], 0, 0);
    case I386_fstatfs:
        return do_statfs(c, 1, SX(a[0]), a[1], 0, 0);
    case I386_statfs64:
        return do_statfs(c, 0, a[0], a[2], 1, a[1]);
    case I386_fstatfs64:
        return do_statfs(c, 1, SX(a[0]), a[2], 1, a[1]);

    case I386_utimensat:
        if (a[2] != 0u) {
            ts32_t t[2];
            if (get32(a[2], t, sizeof(t)) < 0) return LX_EFAULT;
            ts64_t w[2] = { { t[0].tv_sec, t[0].tv_nsec }, { t[1].tv_sec, t[1].tv_nsec } };
            k = scratch(c, sizeof(w));
            if (k == 0u || putk(k, w, sizeof(w)) < 0) return LX_EFAULT;
        }
        return x64(c, X64_utimensat, SX(a[0]), a[1], k, a[3], 0, 0);
    case I386_utimes:
    case I386_futimesat: {
        uint32_t dirfd = num == I386_utimes ? 0xFFFFFF9Cu : a[0];
        uint32_t path = num == I386_utimes ? a[0] : a[1];
        uint32_t tvp = num == I386_utimes ? a[1] : a[2];
        if (tvp != 0u) {
            tv32_t t[2];
            if (get32(tvp, t, sizeof(t)) < 0) return LX_EFAULT;
            ts64_t w[2] = { { t[0].tv_sec, (int64_t)t[0].tv_usec * 1000 },
                            { t[1].tv_sec, (int64_t)t[1].tv_usec * 1000 } };
            k = scratch(c, sizeof(w));
            if (k == 0u || putk(k, w, sizeof(w)) < 0) return LX_EFAULT;
        }
        return x64(c, X64_utimensat, SX(dirfd), path, k, 0, 0, 0);
    }
    case I386_utime:
        if (a[1] != 0u) {
            int32_t t[2];
            if (get32(a[1], t, sizeof(t)) < 0) return LX_EFAULT;
            ts64_t w[2] = { { t[0], 0 }, { t[1], 0 } };
            k = scratch(c, sizeof(w));
            if (k == 0u || putk(k, w, sizeof(w)) < 0) return LX_EFAULT;
        }
        return x64(c, X64_utimensat, AT_FDCWD_64, a[0], k, 0, 0, 0);

    /* 16-bit uid/gid calls: 0xFFFF is the 16-bit "-1", ids that do not fit
     * read back as the overflow id 65534. */
    case I386_getuid16: case I386_getgid16:
    case I386_geteuid16: case I386_getegid16: {
        static const uint16_t map[] = { X64_getuid, X64_getgid, X64_geteuid,
                                        X64_getegid };
        uint32_t i = num == I386_getuid16 ? 0u : num == I386_getgid16 ? 1u :
                     num == I386_geteuid16 ? 2u : 3u;
        r = x64(c, map[i], 0, 0, 0, 0, 0, 0);
        return (DONE_OK(c, r) && r > 0xFFFF) ? 65534 : r;
    }
#define ID16(v) ((uint16_t)(v) == 0xFFFFu ? (uint64_t)(int64_t)-1 : (uint64_t)(uint16_t)(v))
    case I386_setuid16:
        return x64(c, X64_setuid, ID16(a[0]), 0, 0, 0, 0, 0);
    case I386_setgid16:
        return x64(c, X64_setgid, ID16(a[0]), 0, 0, 0, 0, 0);
    case I386_setreuid16:
        return x64(c, X64_setreuid, ID16(a[0]), ID16(a[1]), 0, 0, 0, 0);
    case I386_setregid16:
        return x64(c, X64_setregid, ID16(a[0]), ID16(a[1]), 0, 0, 0, 0);
    case I386_setresuid16:
        return x64(c, X64_setresuid, ID16(a[0]), ID16(a[1]), ID16(a[2]), 0, 0, 0);
    case I386_setresgid16:
        return x64(c, X64_setresgid, ID16(a[0]), ID16(a[1]), ID16(a[2]), 0, 0, 0);
    case I386_chown16:
        return x64(c, X64_chown, a[0], ID16(a[1]), ID16(a[2]), 0, 0, 0);
    case I386_lchown16:
        return x64(c, X64_lchown, a[0], ID16(a[1]), ID16(a[2]), 0, 0, 0);
    case I386_fchown16:
        return x64(c, X64_fchown, SX(a[0]), ID16(a[1]), ID16(a[2]), 0, 0, 0);
#undef ID16
    case I386_umount:
        return x64(c, X64_umount2, a[0], 0, 0, 0, 0, 0);

    /* -- memory ----------------------------------------------------- */
    case I386_mmap2:
        return do_mmap(c, a[0], a[1], a[2], a[3], a[4], (uint64_t)a[5] << 12);
    case I386_old_mmap: {
        uint32_t m[6];
        if (get32(a[0], m, sizeof(m)) < 0) return LX_EFAULT;
        if ((m[5] & 0xFFFu) != 0u) return LX_EINVAL;
        return do_mmap(c, m[0], m[1], m[2], m[3], m[4], m[5]);
    }

    /* -- time ------------------------------------------------------- */
    case I386_time: {
        r = x64(c, X64_time, 0, 0, 0, 0, 0, 0);
        if (!DONE_OK(c, r)) return r;
        int32_t t32 = (int32_t)r;
        if (a[0] != 0u && put32(a[0], &t32, sizeof(t32)) < 0) return LX_EFAULT;
        return t32;
    }
    case I386_gettimeofday:
        k = out_buf(c, a[0], sizeof(tv64_t));
        r = x64(c, X64_gettimeofday, k, a[1], 0, 0, 0, 0);
        if (DONE_OK(c, r) && tv32_out(k, a[0]) < 0) return LX_EFAULT;
        return r;
    case I386_clock_gettime:
    case I386_clock_getres:
        k = out_buf(c, a[1], sizeof(ts64_t));
        r = x64(c, num == I386_clock_gettime ? X64_clock_gettime : X64_clock_getres,
                SX(a[0]), k, 0, 0, 0, 0);
        if (DONE_OK(c, r) && ts32_out(k, a[1]) < 0) return LX_EFAULT;
        return r;
    case I386_nanosleep:
        if ((rc = ts32_in(c, a[0], &k)) < 0) return rc;
        k2 = out_buf(c, a[1], sizeof(ts64_t));
        r = x64(c, X64_nanosleep, k, k2, 0, 0, 0, 0);
        if (!c->call.restarted && r == LX_EINTR) (void)ts32_out(k2, a[1]);
        return r;
    case I386_clock_nanosleep:
        if ((rc = ts32_in(c, a[2], &k)) < 0) return rc;
        k2 = out_buf(c, a[3], sizeof(ts64_t));
        r = x64(c, X64_clock_nanosleep, SX(a[0]), a[1], k, k2, 0, 0);
        if (!c->call.restarted && r == LX_EINTR) (void)ts32_out(k2, a[3]);
        return r;
    case I386_times: {
        k = out_buf(c, a[0], 32u);
        r = x64(c, X64_times, k, 0, 0, 0, 0, 0);
        if (DONE_OK(c, r) && k != 0u) {
            int64_t w[4];
            if (getk(k, w, sizeof(w)) < 0) return LX_EFAULT;
            int32_t t[4] = { (int32_t)w[0], (int32_t)w[1], (int32_t)w[2], (int32_t)w[3] };
            if (put32(a[0], t, sizeof(t)) < 0) return LX_EFAULT;
        }
        return DONE_OK(c, r) ? (int64_t)(uint32_t)r : r;
    }
    case I386_getitimer:
        k = out_buf(c, a[1], 32u);
        r = x64(c, X64_getitimer, a[0], k, 0, 0, 0, 0);
        if (DONE_OK(c, r) && pair32_out(k, a[1]) < 0) return LX_EFAULT;
        return r;
    case I386_setitimer:
        if ((rc = pair32_in(c, a[1], &k)) < 0) return rc;
        k2 = out_buf(c, a[2], 32u);
        r = x64(c, X64_setitimer, a[0], k, k2, 0, 0, 0);
        if (DONE_OK(c, r) && pair32_out(k2, a[2]) < 0) return LX_EFAULT;
        return r;
    case I386_timerfd_settime:
        if ((rc = pair32_in(c, a[2], &k)) < 0) return rc;
        k2 = out_buf(c, a[3], 32u);
        r = x64(c, X64_timerfd_settime, SX(a[0]), a[1], k, k2, 0, 0);
        if (DONE_OK(c, r) && pair32_out(k2, a[3]) < 0) return LX_EFAULT;
        return r;
    case I386_timerfd_gettime:
        k = out_buf(c, a[1], 32u);
        r = x64(c, X64_timerfd_gettime, SX(a[0]), k, 0, 0, 0, 0);
        if (DONE_OK(c, r) && pair32_out(k, a[1]) < 0) return LX_EFAULT;
        return r;

    case I386_futex: {
        uint32_t cmd = a[1] & 0x7Fu;
        if (cmd == 0u || cmd == 6u || cmd == 9u || cmd == 11u || cmd == 13u) {
            if ((rc = ts32_in(c, a[3], &k)) < 0) return rc;
        } else {
            k = a[3]; /* val2: a count, not a pointer */
        }
        return x64(c, X64_futex, a[0], a[1], a[2], k, a[4], a[5]);
    }

    /* -- select / poll ---------------------------------------------- */
    case I386_select: {
        uint32_t s[5];
        if (get32(a[0], s, sizeof(s)) < 0) return LX_EFAULT;
        uint32_t w[6] = { s[0], s[1], s[2], s[3], s[4], 0 };
        return i386_call(c, I386_newselect, w);
    }
    case I386_newselect:
        if ((rc = tv32_in(c, a[4], &k)) < 0) return rc;
        r = x64(c, X64_select, SX(a[0]), a[1], a[2], a[3], k, 0);
        if (!c->call.restarted && k != 0u) (void)tv32_out(k, a[4]);
        return r;
    case I386_pselect6:
    case I386_pselect6_time64: {
        if (num == I386_pselect6) {
            if ((rc = ts32_in(c, a[4], &k)) < 0) return rc;
        } else {
            k = a[4];
        }
        if (a[5] != 0u) {
            uint32_t sig[2];
            if (get32(a[5], sig, sizeof(sig)) < 0) return LX_EFAULT;
            uint64_t w[2] = { sig[0], sig[1] };
            k2 = scratch(c, sizeof(w));
            if (k2 == 0u || putk(k2, w, sizeof(w)) < 0) return LX_EFAULT;
        }
        r = x64(c, X64_pselect6, SX(a[0]), a[1], a[2], a[3], k, k2);
        if (!c->call.restarted && num == I386_pselect6 && k != 0u) {
            (void)ts32_out(k, a[4]);
        }
        return r;
    }
    case I386_ppoll:
        if ((rc = ts32_in(c, a[2], &k)) < 0) return rc;
        r = x64(c, X64_ppoll, a[0], a[1], k, a[3], a[4], 0);
        if (!c->call.restarted && k != 0u) (void)ts32_out(k, a[2]);
        return r;

    /* -- limits / accounting ---------------------------------------- */
    case I386_getrlimit:
    case I386_ugetrlimit:
        return do_getrlimit(c, a[0], a[1]);
    case I386_setrlimit:
        return do_setrlimit(c, a[0], a[1]);
    case I386_getrusage:
        k = out_buf(c, a[1], sizeof(rusage64_t));
        r = x64(c, X64_getrusage, SX(a[0]), k, 0, 0, 0, 0);
        if (DONE_OK(c, r) && rusage_out(k, a[1]) < 0) return LX_EFAULT;
        return r;
    case I386_sysinfo: {
        k = scratch(c, 112u);
        if (k == 0u) return LX_EFAULT;
        r = x64(c, X64_sysinfo, k, 0, 0, 0, 0, 0);
        if (!DONE_OK(c, r)) return r;
        uint8_t in[112];
        if (getk(k, in, sizeof(in)) < 0) return LX_EFAULT;
        uint64_t q[14];
        memcpy(q, in, sizeof(q));        /* uptime, loads[3], 6 ram words */
        uint16_t procs;
        uint32_t unit;
        uint64_t high[2];
        memcpy(&procs, in + 80, 2u);
        memcpy(high, in + 88, 16u);
        memcpy(&unit, in + 104, 4u);
        if (unit == 0u) unit = 1u;
        /* Scale the memory figures until they fit 32 bits, as Linux does. */
        uint64_t mem[8] = { q[4], q[5], q[6], q[7], q[8], q[9], high[0], high[1] };
        for (;;) {
            int fits = 1;
            for (uint32_t i = 0; i < 8u; ++i) {
                if (mem[i] > 0xFFFFFFFFULL) fits = 0;
            }
            if (fits || unit >= 0x40000000u) break;
            for (uint32_t i = 0; i < 8u; ++i) mem[i] >>= 1;
            unit <<= 1;
        }
        uint8_t o[64];
        memset(o, 0, sizeof(o));
        uint32_t w32[10] = { (uint32_t)q[0], (uint32_t)q[1], (uint32_t)q[2],
                             (uint32_t)q[3], (uint32_t)mem[0], (uint32_t)mem[1],
                             (uint32_t)mem[2], (uint32_t)mem[3], (uint32_t)mem[4],
                             (uint32_t)mem[5] };
        memcpy(o, w32, sizeof(w32));
        memcpy(o + 40, &procs, 2u);
        uint32_t h32[2] = { (uint32_t)mem[6], (uint32_t)mem[7] };
        memcpy(o + 44, h32, sizeof(h32));
        memcpy(o + 52, &unit, 4u);
        return put32(a[0], o, sizeof(o)) == 0 ? 0 : LX_EFAULT;
    }

    /* -- sockets ---------------------------------------------------- */
    case I386_socketcall:
        return do_socketcall(c, a[0], a[1]);
    case I386_sendmsg:
        return do_sendmsg(c, a[0], a[1], a[2]);
    case I386_recvmsg:
        return do_recvmsg(c, a[0], a[1], a[2]);
    case I386_setsockopt:
        return do_setsockopt(c, a[0], a[1], a[2], a[3], a[4]);
    case I386_getsockopt:
        return do_getsockopt(c, a[0], a[1], a[2], a[3], a[4]);

    /* -- SysV IPC --------------------------------------------------- */
    case I386_ipc:
        return do_ipc(c, a);
    case I386_shmctl:
        return do_shmctl(c, a[0], a[1], a[2]);

    default:
        break;
    }

    if (num < I386_NR_MAX && g_i386_map[num].valid) {
        uint64_t w[6];
        for (uint32_t i = 0; i < 6u; ++i) {
            w[i] = ((g_i386_map[num].sext >> i) & 1u) ? SX(a[i]) : (uint64_t)a[i];
        }
        return x64(c, g_i386_map[num].x64, w[0], w[1], w[2], w[3], w[4], w[5]);
    }
    return LX_ENOSYS;
}

static uint64_t linux_i386_syscall_dispatch(uint64_t saved_rsp, uint64_t num,
                                            uint64_t a1, uint64_t a2,
                                            uint64_t a3, uint64_t a4,
                                            uint64_t a5, uint64_t a6)
{
    /* ECX is a preserved register here, and the return path hands it back
     * from what this records (with the FS/GS selectors). */
    process_compat32_note_syscall_entry(a2);

    i386_ctx_t c;
    memset(&c, 0, sizeof(c));
    c.saved_rsp = saved_rsp;
    c.call.abi = PROCESS_ABI_LINUX_I386;
    c.call.orig_num = num;
    lx_scratch_init(&c.scratch, syscall_get_user_rsp(), USER32_WINDOW_BASE);

    const uint32_t a[6] = { (uint32_t)a1, (uint32_t)a2, (uint32_t)a3,
                            (uint32_t)a4, (uint32_t)a5, (uint32_t)a6 };
    int64_t result = i386_call(&c, (uint32_t)num, a);

    if (!c.call.restarted) {
        linux_frame_set_result(saved_rsp, PROCESS_ABI_LINUX_I386, result);
    }
    return c.sw;
}

static const compat_layer_t g_linux_i386_layer = {
    .abi = PROCESS_ABI_LINUX_I386,
    .name = "Linux i386",
    .dispatch = linux_i386_syscall_dispatch,
};

void linux_i386_compat_layer_register(void)
{
    memset(g_i386_map, 0, sizeof(g_i386_map));
    for (uint32_t i = 0; i < sizeof(g_i386_direct) / sizeof(g_i386_direct[0]); ++i) {
        const i386_direct_t *d = &g_i386_direct[i];
        g_i386_map[d->i386].x64 = d->x64;
        g_i386_map[d->i386].sext = d->sext;
        g_i386_map[d->i386].valid = 1;
    }
    (void)compat_registry_register(&g_linux_i386_layer);
}

#else

void linux_i386_compat_layer_register(void)
{
}

#endif
