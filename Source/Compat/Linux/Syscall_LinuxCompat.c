#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "Compat/Linux/Syscall_LinuxCompat.h"
#include "Compat/compat_registry.h"
#include "Core/process/ProcessManager.h"
#include "Core/process/ProcessScheduler.h"
#include "Core/syscall/Syscall_File.h"
#include "Core/syscall/Syscall_Socket.h"
#include "Core/syscall/Syscall_Epoll.h"
#include "Core/syscall/Syscall_Clock.h"
#include "Core/syscall/Syscall_Futex.h"
#include "Core/syscall/Poll_Wait.h"
#include "Core/syscall/Syscall_VM.h"
#include "Core/syscall/Syscall_Main.h"
#include "Core/memory/SharedMemory.h"
#include "Core/debug/FlightRec.h"
#include "Core/memory/FileMap.h"
#include "Core/timer/Timer.h"
#include "Core/usercopy/Usercopy.h"
#include "Core/vfs/VFS.h"
#include "Core/vfs/DevFS.h"
#include "Core/vfs/ProcFS.h"
#include "IPC/UnixSocket.h"
#include "Compat/Linux/Linux_FdTable.h"
#include "Crypto/Crypto.h"
#include "Debug/serial/Serial.h"
#include "kernel/config.h"

int64_t write(int fd, const void *buf, uint64_t count);
#include "Platform/rtc/RTC.h"
#include "kernel/status.h"
#include "mmu/Paging_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include "smp/SMP_Main.h"
#include "Core/sync/Spinlock.h"

#ifndef LINUX_CLONE_TRACE
#define LINUX_CLONE_TRACE 0
#endif

#ifndef LINUX_SYSCALL_PROFILE
#define LINUX_SYSCALL_PROFILE 0
#endif
#if LINUX_SYSCALL_PROFILE
/* Which Linux syscalls a foreign program actually spends its life in. Counts
 * only -- the point is to find a syscall being re-run in a loop, and a loop
 * shows up in the count. Dumped over serial every PROFILE_DUMP_EVERY calls,
 * busiest first (and cleared as it prints, so each line is a fresh window). */
#define PROFILE_MAX_NR      512u
#define PROFILE_DUMP_EVERY  100000u
static volatile uint32_t g_prof_count[PROFILE_MAX_NR];
static volatile uint32_t g_prof_total;

static void linux_syscall_profile(uint64_t nr)
{
    if (nr < PROFILE_MAX_NR) {
        __atomic_fetch_add(&g_prof_count[nr], 1u, __ATOMIC_RELAXED);
    }
    uint32_t total = __atomic_add_fetch(&g_prof_total, 1u, __ATOMIC_RELAXED);
    if (total % PROFILE_DUMP_EVERY != 0u) {
        return;
    }
    serial_write_string("[sysprof] total=");
    serial_write_uint64((uint64_t)total);
    for (int rank = 0; rank < 8; ++rank) {
        uint32_t best = 0, best_nr = 0;
        for (uint32_t i = 0; i < PROFILE_MAX_NR; ++i) {
            uint32_t v = __atomic_load_n(&g_prof_count[i], __ATOMIC_RELAXED);
            if (v > best) { best = v; best_nr = i; }
        }
        if (best == 0u) break;
        serial_write_string(" #");
        serial_write_uint64((uint64_t)best_nr);
        serial_write_string("=");
        serial_write_uint64((uint64_t)best);
        __atomic_store_n(&g_prof_count[best_nr], 0u, __ATOMIC_RELAXED);
    }
    serial_write_string("\n");
}
#endif

#ifndef CHROME_SHM_TRACE
#define CHROME_SHM_TRACE 0
#endif
#if CHROME_SHM_TRACE
/* Temporary bring-up trace for Chromium's shared-memory handshake (memfd ->
 * /proc/self/fd reopen -> fcntl(F_GETFL) access-mode check). Capped so a
 * stuck loop cannot bury the rest of the serial log. */
static void chrome_shm_trace3(const char *a, uint64_t av, const char *b,
                              uint64_t bv, const char *c, uint64_t cv)
{
    static uint32_t budget = 400u;
    if (budget == 0u) return;
    --budget;
    serial_write_string("[shmtr] ");
    serial_write_string(a); serial_write_uint64(av);
    if (b[0]) { serial_write_string(b); serial_write_uint64(bv); }
    if (c[0]) { serial_write_string(c); serial_write_uint64(cv); }
    serial_write_string("\n");
}
#endif


#define LINUX_EBADF  (-9LL)
#define LINUX_EFAULT (-14LL)
#define LINUX_EBUSY  (-16LL)
#define LINUX_ENODEV (-19LL)
#define LINUX_EINVAL (-22LL)
#define LINUX_ESRCH  (-3LL)
#define LINUX_ENOTSUP (-95LL)
#define LINUX_ENOTTY (-25LL)
#define LINUX_EINPROGRESS (-115LL)

#define LINUX_RSEQ_FLAG_UNREGISTER 1u

#define LINUX_ARCH_SET_FS 0x1002u
#define LINUX_ARCH_GET_FS 0x1003u

#define LINUX_RLIMIT_CPU     0u
#define LINUX_RLIMIT_FSIZE   1u
#define LINUX_RLIMIT_DATA    2u
#define LINUX_RLIMIT_STACK   3u
#define LINUX_RLIMIT_CORE    4u
#define LINUX_RLIMIT_RSS     5u
#define LINUX_RLIMIT_NPROC   6u
#define LINUX_RLIMIT_NOFILE  7u
#define LINUX_RLIMIT_MEMLOCK 8u
#define LINUX_RLIMIT_AS      9u
#define LINUX_RLIM_INFINITY  (~0ULL)

#define LINUX_F_DUPFD  0
#define LINUX_F_GETFD  1
#define LINUX_F_SETFD  2
#define LINUX_F_GETFL  3
#define LINUX_F_ADD_SEALS 1033
#define LINUX_F_GET_SEALS 1034
#define LINUX_F_SETFL  4
#define LINUX_F_DUPFD_CLOEXEC 1030
/* POSIX advisory record locks, and the open-file-description variants glibc
 * and LevelDB reach for. */
#define LINUX_F_GETLK      5
#define LINUX_F_SETLK      6
#define LINUX_F_SETLKW     7
#define LINUX_F_OFD_GETLK  36
#define LINUX_F_OFD_SETLK  37
#define LINUX_F_OFD_SETLKW 38
#define LINUX_F_RDLCK      0
#define LINUX_F_WRLCK      1
#define LINUX_F_UNLCK      2

/* struct flock as the x86_64 kernel ABI lays it out. */
typedef struct {
    int16_t l_type;
    int16_t l_whence;
    int32_t __pad;
    int64_t l_start;
    int64_t l_len;
    int32_t l_pid;
    int32_t __pad2;
} linux_flock_t;
#define LINUX_FD_CLOEXEC 1u

#define LINUX_FIONREAD 0x541Bu
#define LINUX_FIONBIO  0x5421u

typedef struct {
    uint64_t base;
    uint64_t length;
} linux_iovec_t;

typedef struct {
    uint64_t current;
    uint64_t maximum;
} linux_rlimit64_t;

typedef struct {
    uint64_t handler;
    uint64_t flags;
    uint64_t restorer;
    uint64_t mask;
} linux_sigaction_t;

int64_t syscall_gettid(void)
{
    return (int64_t)process_get_current_tid();
}

int64_t syscall_set_tid_address(uint64_t tidptr)
{
    int rc = process_set_clear_child_tid(tidptr);
    if (rc < 0) return rc;
    return syscall_gettid();
}

int64_t syscall_arch_prctl(uint64_t code, uint64_t addr)
{
#if defined(__x86_64__)
    if (code == LINUX_ARCH_SET_FS) {
        process_set_current_fs_base(addr);
        return 0;
    }
    if (code == LINUX_ARCH_GET_FS) {
        uint64_t value = process_get_current_fs_base();
        return copy_to_user((void *)(uintptr_t)addr, &value, sizeof(value)) == 0u ?
            0 : LINUX_EFAULT;
    }
#else
    (void)addr;
#endif
    return LINUX_EINVAL;
}

/* Defined with the sched_*affinity calls that first needed it; the rule is the
 * same here, because a Linux "pid" in these interfaces is a thread id. */
static int linux_sched_pid_is_self(uint64_t pid);

int64_t syscall_prlimit64(uint64_t pid, uint64_t resource, uint64_t new_limit, uint64_t old_limit)
{
    if (!linux_sched_pid_is_self(pid)) return LINUX_ESRCH;
    if (new_limit != 0u) {
        /* Limits are not enforced yet; accept the request (matching what a
         * setrlimit() with sufficient privilege would do on Linux) instead
         * of failing glibc/Chromium startup paths that lower RLIMIT_CORE,
         * raise RLIMIT_NOFILE, etc. */
        return 0;
    }

    linux_rlimit64_t limit;
    switch (resource) {
        case LINUX_RLIMIT_STACK:
            /* Matches PROCESS_THREAD_STACK_SIZE (ProcessManager_Create.c). */
            limit.current = 8ULL * 1024ULL * 1024ULL;
            limit.maximum = LINUX_RLIM_INFINITY;
            break;
        case LINUX_RLIMIT_NOFILE:
            /* Must exceed the highest fd any range can hand out (AF_UNIX up to
             * ~320, inet sockets up to ~575) or Xtrans rejects sockets with
             * "fd >= sysconf(_SC_OPEN_MAX)". Kept at FD_SETSIZE so select()-
             * based code stays safe too. */
            limit.current = 1024u;
            limit.maximum = 1024u;
            break;
        case LINUX_RLIMIT_AS:
            /* No hard cap is enforced on the user address space today
             * (mmap/brk are only bounded by the fixed user VA layout),
             * so report unlimited rather than an artificially small
             * value that would make glibc's malloc/mmap arena sizing
             * misbehave. */
            limit.current = LINUX_RLIM_INFINITY;
            limit.maximum = LINUX_RLIM_INFINITY;
            break;
        case LINUX_RLIMIT_DATA:
        case LINUX_RLIMIT_RSS:
        case LINUX_RLIMIT_MEMLOCK:
            limit.current = LINUX_RLIM_INFINITY;
            limit.maximum = LINUX_RLIM_INFINITY;
            break;
        case LINUX_RLIMIT_NPROC:
            limit.current = (uint64_t)OS_CONFIG_PROCESS_MAX_COUNT;
            limit.maximum = (uint64_t)OS_CONFIG_PROCESS_MAX_COUNT;
            break;
        case LINUX_RLIMIT_CORE:
            limit.current = 0;
            limit.maximum = 0;
            break;
        default:
            return LINUX_ENOTSUP;
    }
    if (old_limit != 0u &&
        !process_user_buffer_is_writable((void *)(uintptr_t)old_limit,
                                         sizeof(limit)) ||
        copy_to_user((void *)(uintptr_t)old_limit, &limit, sizeof(limit)) != 0u) {
        /* The writability test is not redundant: the kernel can write through
         * a read-only user mapping, and Chromium uses exactly this call to
         * prove that it cannot. base::internal::CheckMemoryReadOnly() passes a
         * page it has just mprotect()ed PROT_READ as the output buffer and
         * CHECK-fails the process unless getrlimit() returns EFAULT. */
        return LINUX_EFAULT;
    }
    return 0;
}

int64_t syscall_getrandom(uint64_t buffer, uint64_t length, uint64_t flags)
{
    if ((flags & ~3u) != 0u) return LINUX_EINVAL;
    if (length == 0u) return 0;
    if (buffer == 0u ||
        !process_user_buffer_is_valid((void *)(uintptr_t)buffer, length)) {
        return LINUX_EFAULT;
    }

    static volatile uint64_t generation;
    uint64_t produced = 0;
    while (produced < length) {
        struct {
            uint64_t ticks;
            uint64_t generation;
            uint64_t cr3;
            uint64_t stack_address;
            int32_t pid;
            int32_t tid;
        } seed;
        seed.ticks = timer_ticks();
        seed.generation = __sync_add_and_fetch(&generation, 1u);
        seed.cr3 = process_get_current_cr3();
        seed.stack_address = (uint64_t)(uintptr_t)&seed;
        seed.pid = process_get_current_pid();
        seed.tid = process_get_current_tid();

        uint8_t digest[32];
        crypto_sha256((const uint8_t *)&seed, sizeof(seed), digest);
        uint64_t chunk = length - produced;
        if (chunk > sizeof(digest)) chunk = sizeof(digest);
        if (copy_to_user_trusted((uint8_t *)(uintptr_t)buffer + produced,
                                 digest, chunk) != 0u) {
            return produced != 0u ? (int64_t)produced : LINUX_EFAULT;
        }
        produced += chunk;
    }
    return (int64_t)produced;
}

int64_t syscall_readv(int32_t fd, uint64_t iov, int32_t iovcnt)
{
    if (iovcnt < 0 || iovcnt > 1024 || (iovcnt != 0 && iov == 0u)) {
        return LINUX_EINVAL;
    }
    uint8_t chunk[4096];
    uint64_t total = 0;

    if (unix_socket_fd_in_range(fd)) {
        for (int32_t index = 0; index < iovcnt; ++index) {
            linux_iovec_t v;
            if (copy_from_user(&v, (const linux_iovec_t *)(uintptr_t)iov + index,
                               sizeof(v)) != 0u) {
                return total ? (int64_t)total : LINUX_EFAULT;
            }
            uint64_t off = 0;
            while (off < v.length) {
                uint64_t want = v.length - off;
                if (want > sizeof(chunk)) want = sizeof(chunk);
                int64_t n = unix_socket_recv(fd, chunk, want);
                if (n < 0) return total ? (int64_t)total : n;
                if (n == 0) return (int64_t)total;
                if (copy_to_user_trusted((uint8_t *)(uintptr_t)v.base + off,
                                         chunk, (uint64_t)n) != 0u) {
                    return total ? (int64_t)total : LINUX_EFAULT;
                }
                off += (uint64_t)n; total += (uint64_t)n;
                if ((uint64_t)n < want) return (int64_t)total;
            }
        }
        return (int64_t)total;
    }
    /* Same rule write() uses: a standard descriptor is the console only while
     * nothing is bound to it. Treating every fd <= 2 as the console is wrong
     * the moment a process puts a real file there, which is exactly what a
     * terminal's child does with its pty -- readv() on such a stdin has to
     * read the terminal, not report end-of-file. */
    int stdio_is_console = (fd <= 2) &&
                           (syscall_file_get_file_info(fd, NULL, NULL) != 0);

    for (int32_t index = 0; index < iovcnt; ++index) {
        linux_iovec_t vector;
        if (copy_from_user(&vector,
                           (const linux_iovec_t *)(uintptr_t)iov + index,
                           sizeof(vector)) != 0u) {
            return total != 0u ? (int64_t)total : LINUX_EFAULT;
        }
        if (vector.length != 0u &&
            !process_user_buffer_is_valid((void *)(uintptr_t)vector.base,
                                          vector.length)) {
            return total != 0u ? (int64_t)total : LINUX_EFAULT;
        }
        uint64_t offset = 0;
        while (offset < vector.length) {
            uint64_t want = vector.length - offset;
            if (want > sizeof(chunk)) want = sizeof(chunk);
            if (stdio_is_console) {
                return (int64_t)total; /* console input is not readable here */
            }
            int64_t count = syscall_file_read(fd, chunk, want);
            if (count < 0) return total != 0u ? (int64_t)total : count;
            if (count == 0) return (int64_t)total;
            if (copy_to_user_trusted((uint8_t *)(uintptr_t)vector.base + offset,
                                     chunk, (uint64_t)count) != 0u) {
                return total != 0u ? (int64_t)total : LINUX_EFAULT;
            }
            offset += (uint64_t)count;
            total += (uint64_t)count;
            if ((uint64_t)count < want) return (int64_t)total;
        }
    }
    return (int64_t)total;
}

int64_t syscall_writev(int32_t fd, uint64_t iov, int32_t iovcnt)
{
    /* See the same test in syscall_readv(): a standard descriptor is only the
     * console while nothing is bound to it. Sending every fd <= 2 to the
     * serial port regardless is why the shell xterm started printed its prompt
     * onto COM1 instead of into the window -- busybox's line editor writes
     * through writev, so write()'s equivalent check never saw it. */
    int stdio_is_console = (fd <= 2) &&
                           (syscall_file_get_file_info(fd, NULL, NULL) != 0);

    if (iovcnt < 0 || iovcnt > 1024 || (iovcnt != 0 && iov == 0u)) {
        return LINUX_EINVAL;
    }
    uint8_t chunk[4096];
    uint64_t total = 0;

    /* AF_UNIX stream writev() -- X's _XSERVTransWritev batches replies here. */
    if (unix_socket_fd_in_range(fd)) {
        for (int32_t index = 0; index < iovcnt; ++index) {
            linux_iovec_t v;
            if (copy_from_user(&v, (const linux_iovec_t *)(uintptr_t)iov + index,
                               sizeof(v)) != 0u) {
                return total ? (int64_t)total : LINUX_EFAULT;
            }
            uint64_t off = 0;
            while (off < v.length) {
                uint64_t want = v.length - off;
                if (want > sizeof(chunk)) want = sizeof(chunk);
                if (copy_from_user_trusted(chunk,
                        (const uint8_t *)(uintptr_t)v.base + off, want) != 0u) {
                    return total ? (int64_t)total : LINUX_EFAULT;
                }
                int64_t w = unix_socket_send(fd, chunk, want);
                if (w < 0) return total ? (int64_t)total : w;
                off += (uint64_t)w; total += (uint64_t)w;
                if ((uint64_t)w < want) return (int64_t)total;
            }
        }
        return (int64_t)total;
    }

    for (int32_t index = 0; index < iovcnt; ++index) {
        linux_iovec_t vector;
        if (copy_from_user(&vector,
                           (const linux_iovec_t *)(uintptr_t)iov + index,
                           sizeof(vector)) != 0u) {
            return total != 0u ? (int64_t)total : LINUX_EFAULT;
        }
        if (vector.length != 0u &&
            !process_user_buffer_is_valid((const void *)(uintptr_t)vector.base,
                                          vector.length)) {
            return total != 0u ? (int64_t)total : LINUX_EFAULT;
        }
        uint64_t offset = 0;
        while (offset < vector.length) {
            uint64_t want = vector.length - offset;
            if (want > sizeof(chunk)) want = sizeof(chunk);
            if (copy_from_user_trusted(chunk,
                                       (const uint8_t *)(uintptr_t)vector.base + offset,
                                       want) != 0u) {
                return total != 0u ? (int64_t)total : LINUX_EFAULT;
            }
            int64_t count;
            if (stdio_is_console) {
                for (uint64_t i = 0; i < want; ++i) {
                    serial_write_char((char)chunk[i]);
                }
                count = (int64_t)want;
            } else {
                count = syscall_file_write(fd, chunk, want);
            }
            if (count < 0) return total != 0u ? (int64_t)total : count;
            offset += (uint64_t)count;
            total += (uint64_t)count;
            if ((uint64_t)count < want) return (int64_t)total;
        }
    }
    return (int64_t)total;
}

int64_t syscall_ftruncate(int32_t fd, int64_t length)
{
    if (length < 0) return LINUX_EINVAL;
    return syscall_file_truncate(fd, (uint64_t)length);
}

/* fallocate(2), enough of it for posix_fallocate(3).
 *
 * libwayland's os_create_anonymous_file() sizes every wl_shm pool this way:
 * memfd_create() gives a zero-length object and posix_fallocate() is what
 * makes it `size` bytes. glibc's userspace fallback for a missing fallocate
 * writes the file out by hand, which a zero-length memfd will not accept, so
 * an ENOSYS here failed every buffer allocation -- and a Wayland client with
 * no buffers draws nothing. GDK hit it first on the cursor theme
 * (wl_cursor_theme_load() returns NULL when it cannot allocate a single
 * cursor) and then aborted on the NULL theme name.
 *
 * Only the default mode is meaningful for the objects that reach us: they
 * have no holes to punch and no way to be sparse, so "make sure the file is
 * at least offset+len long" is the whole of it. */
#define LINUX_FALLOC_FL_KEEP_SIZE 0x01u

static int64_t linux_fallocate(int32_t fd, uint32_t mode, int64_t offset,
                               int64_t len)
{
    if (offset < 0 || len <= 0) return LINUX_EINVAL;
    if ((mode & ~LINUX_FALLOC_FL_KEEP_SIZE) != 0u) return LINUX_ENOTSUP;

    uint64_t need = (uint64_t)offset + (uint64_t)len;

    /* Growing is the only thing to do, so never shrink: find the current end
     * with lseek() and leave the file alone when it is already big enough. */
    int64_t saved = syscall_file_seek(fd, 0, 1 /* SEEK_CUR */);
    if (saved < 0) return saved;
    int64_t end = syscall_file_seek(fd, 0, 2 /* SEEK_END */);
    (void)syscall_file_seek(fd, saved, 0 /* SEEK_SET */);
    if (end < 0) return end;
    if ((uint64_t)end >= need) return 0;

    int32_t rc = syscall_file_truncate(fd, need);
    return (rc < 0) ? (int64_t)rc : 0;
}

/* Terminal ioctls (TODO_Chromium_LinuxABI.md section 4) for everything that is
 * NOT a pseudo-terminal -- syscall_ioctl_ex() peels those off first. None of
 * the fds that reach here are real ttys, so TCGETS/TCSETS* must report ENOTTY
 * - that is exactly the signal isatty()/Chromium's base::IsTerminal() look for.
 * TIOCGWINSZ still hands back a plausible 80x24 for the std fds so code
 * that wants a size (progress bars, `--columns` autodetect) gets one. */
#define LINUX_TCGETS     0x5401u
#define LINUX_TCSETS     0x5402u
#define LINUX_TCSETSW    0x5403u
#define LINUX_TCSETSF    0x5404u
#define LINUX_TCGETA     0x5405u
#define LINUX_TCSETA     0x5406u
#define LINUX_TCSETAW    0x5407u
#define LINUX_TCSETAF    0x5408u
#define LINUX_TCFLSH     0x540Bu
#define LINUX_TIOCGPGRP  0x540Fu
#define LINUX_TIOCSPGRP  0x5410u
#define LINUX_TIOCGWINSZ 0x5413u
#define LINUX_TIOCSWINSZ 0x5414u
#define LINUX_TIOCSCTTY  0x540Eu
#define LINUX_TIOCNOTTY  0x5422u
#define LINUX_TIOCGPTN   0x80045430u
#define LINUX_TIOCSPTLCK 0x40045431u

static int64_t linux_ioctl_tty(int32_t fd, uint64_t request, uint64_t arg)
{
    switch (request) {
        case LINUX_TCGETS:
        case LINUX_TCSETS:
        case LINUX_TCSETSW:
        case LINUX_TCSETSF:
        case LINUX_TCGETA:
        case LINUX_TCSETA:
        case LINUX_TCSETAW:
        case LINUX_TCSETAF:
        case LINUX_TCFLSH:
        case LINUX_TIOCSCTTY:
        case LINUX_TIOCNOTTY:
        case LINUX_TIOCGPTN:
        case LINUX_TIOCSPTLCK:
            return LINUX_ENOTTY;
        case LINUX_TIOCGPGRP:
        case LINUX_TIOCSPGRP:
            /* None of these fds are controlling terminals. */
            return LINUX_ENOTTY;
        case LINUX_TIOCGWINSZ: {
            struct {
                uint16_t ws_row;
                uint16_t ws_col;
                uint16_t ws_xpixel;
                uint16_t ws_ypixel;
            } ws = { 24u, 80u, 0u, 0u };
            if (fd < 0 || fd > 2) {
                return LINUX_ENOTTY;
            }
            if (arg == 0u ||
                copy_to_user((void *)(uintptr_t)arg, &ws, sizeof(ws)) != 0u) {
                return LINUX_EFAULT;
            }
            return 0;
        }
        case LINUX_TIOCSWINSZ:
            return (fd >= 0 && fd <= 2) ? 0 : LINUX_ENOTTY;
        default:
            return LINUX_ENOTSUP;
    }
}

int64_t syscall_ioctl_ex(int32_t fd, uint64_t request, uint64_t arg)
{
    /* A pseudo-terminal answers the terminal ioctls for real -- termios, the
     * window size, the foreground process group, TIOCGPTN/TIOCSPTLCK. It has
     * to be tested before linux_ioctl_tty() below, whose blanket ENOTTY is
     * only correct for the fds that are not terminals. */
    if (syscall_file_is_pty(fd)) {
        return syscall_file_ioctl(fd, request, arg);
    }
    if (OS_CONFIG_FOREIGN_TRACE &&
        ((uint32_t)request == LINUX_TIOCGPTN ||
         (uint32_t)request == LINUX_TIOCSPTLCK)) {
        /* A pty-only ioctl on an fd the file layer does not consider a pty.
         * Left in permanently: it is the difference between "the pty layer
         * refused" and "the fd never became a pty", and nothing else in the
         * system can tell those apart from userland. */
        serial_write_string("[pty] pty ioctl on non-pty fd=");
        serial_write_uint32((uint32_t)fd);
        serial_write_string(" req=");
        serial_write_uint64(request);
        serial_write_string(" checks=");
        serial_write_uint32(syscall_file_pty_debug(fd));
        serial_write_string(" pid=");
        serial_write_uint32((uint32_t)process_get_current_pid());
        serial_write_char('\n');
    }
    int64_t tty_rc = linux_ioctl_tty(fd, request, arg);
    if (tty_rc != LINUX_ENOTSUP) {
        return tty_rc;
    }
    /* Character devices (DRM/KMS, evdev) handle their own arg validation and
     * some requests legitimately pass arg == 0 (DRM_IOCTL_SET_MASTER etc.). */
    if (syscall_file_is_chardev(fd)) {
        return syscall_file_ioctl(fd, request, arg);
    }
    if (arg == 0u) return LINUX_EFAULT;
    if (request == LINUX_FIONBIO) {
        int32_t enabled = 0;
        if (copy_from_user(&enabled, (const void *)(uintptr_t)arg,
                           sizeof(enabled)) != 0u) {
            return LINUX_EFAULT;
        }
        if (syscall_socket_fd_in_range(fd)) {
            return syscall_socket_set_nonblocking(fd, enabled);
        }
        int32_t flags = syscall_file_get_status_flags(fd);
        if (flags < 0) return flags;
        if (enabled) flags |= 0x0800;
        else flags &= ~0x0800;
        return syscall_file_set_status_flags(fd, (uint32_t)flags);
    }
    if (request == LINUX_FIONREAD) {
        int64_t available = syscall_file_available(fd);
        if (available < 0) available = syscall_socket_available(fd);
        if (available < 0) return available;
        int32_t value = available > INT32_MAX ? INT32_MAX : (int32_t)available;
        return copy_to_user((void *)(uintptr_t)arg, &value, sizeof(value)) == 0u ?
            0 : LINUX_EFAULT;
    }
    return LINUX_ENOTSUP;
}

int64_t syscall_fcntl_ex(int32_t fd, int32_t cmd, uint64_t arg)
{
    switch (cmd) {
        case LINUX_F_DUPFD:
            return syscall_file_dup_at_least(fd, (int32_t)arg);
        case LINUX_F_DUPFD_CLOEXEC: {
            int32_t duplicated = syscall_file_dup_at_least(fd, (int32_t)arg);
            if (duplicated >= 0) {
                int32_t rc = syscall_file_set_descriptor_flags(
                    duplicated, LINUX_FD_CLOEXEC);
                if (rc < 0) {
                    (void)syscall_file_close(duplicated);
                    return rc;
                }
            }
            return duplicated;
        }
        case LINUX_F_GETFD:
            /* Socket fds live outside the generic file table. libpulse's
             * pa_make_fd_cloexec() asserts F_GETFD >= 0 on its connection
             * socket and aborts otherwise, so never return an error here. */
            if (syscall_socket_fd_in_range(fd)) {
                return 0;
            }
            {
                int32_t r = syscall_file_get_descriptor_flags(fd);
                return r < 0 ? 0 : r;
            }
        case LINUX_F_SETFD:
            if (syscall_socket_fd_in_range(fd)) {
                return 0; /* accept; CLOEXEC on sockets is a no-op here */
            }
            {
                int32_t r = syscall_file_set_descriptor_flags(fd, (uint32_t)arg);
                return r < 0 ? 0 : r;
            }
        case LINUX_F_GETFL:
#if CHROME_SHM_TRACE
            {
                int64_t tr;
                if (syscall_socket_fd_in_range(fd)) {
                    int32_t sf = syscall_socket_get_status_flags(fd);
                    tr = (sf < 0) ? sf : (0x0002 | sf);
                } else if (unix_socket_fd_in_range(fd)) {
                    tr = 0x0002 | (unix_socket_is_nonblock(fd) ? 0x0800 : 0);
                } else {
                    tr = syscall_file_get_status_flags(fd);
                }
                chrome_shm_trace3("F_GETFL fd=", (uint64_t)fd, " -> ", (uint64_t)tr, "", 0);
            }
#endif
            /* Socket fds are outside the generic file table; O_NONBLOCK for
             * them is tracked in the socket layer. Report O_RDWR|<nonblock>. */
            if (syscall_socket_fd_in_range(fd)) {
                int32_t sflags = syscall_socket_get_status_flags(fd);
                if (sflags < 0) return sflags;
                return 0x0002 | sflags; /* O_RDWR | (O_NONBLOCK?) */
            }
            if (unix_socket_fd_in_range(fd)) {
                return 0x0002 | (unix_socket_is_nonblock(fd) ? 0x0800 : 0);
            }
            return syscall_file_get_status_flags(fd);
        case LINUX_F_ADD_SEALS: {
            /* memfd sealing. Chromium seals every shared-memory region right
             * after ftruncate() and, if the call fails, gives up on memfd and
             * falls back to a temp file -- a path that then trips its own
             * fcntl(F_GETFL) access-mode CHECK. So this has to work. */
            int32_t rc = syscall_memfd_add_seals(fd, (uint32_t)arg);
            if (rc == (int32_t)OS_STATUS_ACCESS_DENIED) return -1LL; /* EPERM */
            return (rc < 0) ? LINUX_EINVAL : 0;
        }
        case LINUX_F_GET_SEALS: {
            int32_t seals = syscall_memfd_get_seals(fd);
            return (seals < 0) ? LINUX_EINVAL : seals;
        }
        case LINUX_F_SETFL:
            if (syscall_socket_fd_in_range(fd)) {
                return syscall_socket_set_nonblocking(
                    fd, ((uint32_t)arg & 0x0800u) != 0u);
            }
            if (unix_socket_fd_in_range(fd)) {
                return unix_socket_set_nonblock(fd,
                                                ((uint32_t)arg & 0x0800u) != 0u);
            }
            return syscall_file_set_status_flags(fd, (uint32_t)arg);

        /* Advisory record locks.
         *
         * Returning ENOTSUP here cost Chromium its whole profile. LevelDB
         * takes a whole-file lock on <db>/LOCK before opening a database, via
         * fcntl(F_SETLK); the failure made every one of the two dozen
         * databases behind a Chrome profile fail to open --
         *
         *   Unable to open /tmp/chromium/Default: IO error:
         *   /tmp/chromium/Default/LOCK
         *
         * -- and the browser came up with "Something went wrong when opening
         * your profile. Some features may be unavailable."
         *
         * The locks are granted unconditionally, and that is a real
         * simplification rather than an implementation: the kernel keeps no
         * lock table, so two processes locking the same range both succeed and
         * F_GETLK always answers "nobody holds it". What it does provide is
         * the contract a *single* locker depends on, which is what every
         * consumer in this runtime is -- LevelDB already refuses a second lock
         * on the same file inside one process from its own bookkeeping. A
         * second process opening the same database would not be stopped; when
         * something in this runtime needs that, this is where the table goes.
         * See Docs/Others/TODO_Chromium_LinuxABI.md section 10.-5. */
        case LINUX_F_SETLK:
        case LINUX_F_SETLKW:
        case LINUX_F_OFD_SETLK:
        case LINUX_F_OFD_SETLKW: {
            linux_flock_t lk;
            if (arg == 0u ||
                copy_from_user(&lk, (const void *)(uintptr_t)arg,
                               sizeof(lk)) != 0u) {
                return LINUX_EFAULT;
            }
            if (lk.l_type != LINUX_F_RDLCK && lk.l_type != LINUX_F_WRLCK &&
                lk.l_type != LINUX_F_UNLCK) {
                return LINUX_EINVAL;
            }
            return 0;
        }
        case LINUX_F_GETLK:
        case LINUX_F_OFD_GETLK: {
            linux_flock_t lk;
            if (arg == 0u ||
                copy_from_user(&lk, (const void *)(uintptr_t)arg,
                               sizeof(lk)) != 0u) {
                return LINUX_EFAULT;
            }
            /* F_UNLCK in l_type is how "the range is free" is reported. */
            lk.l_type = LINUX_F_UNLCK;
            lk.l_pid = 0;
            if (copy_to_user_trusted((void *)(uintptr_t)arg, &lk,
                                     sizeof(lk)) != 0u) {
                return LINUX_EFAULT;
            }
            return 0;
        }
        default:
            return LINUX_ENOTSUP;
    }
}

int64_t syscall_rt_sigaction(uint64_t signum, uint64_t act, uint64_t oldact, uint64_t sigsetsize)
{
    if (signum == 0u || signum >= 32u || signum == 9u || signum == 19u ||
        sigsetsize != sizeof(uint64_t)) {
        return LINUX_EINVAL;
    }

    linux_sigaction_t action = {0};
    int has_new = (act != 0u);
    if (has_new &&
        copy_from_user(&action, (const void *)(uintptr_t)act,
                       sizeof(action)) != 0u) {
        return LINUX_EFAULT;
    }

    uint64_t old_handler = 0, old_flags = 0, old_mask = 0, old_restorer = 0;
    if (has_new) {
        if (process_signal_set_handler_ex((int32_t)signum, action.handler,
                                          action.flags, action.mask,
                                          action.restorer, &old_handler,
                                          &old_flags, &old_mask,
                                          &old_restorer) < 0) {
            return LINUX_EINVAL;
        }
    } else {
        old_handler = process_signal_get_handler((int32_t)signum);
        if (old_handler == (uint64_t)-1) return LINUX_EINVAL;
    }

    if (oldact != 0u) {
        linux_sigaction_t old_action = {0};
        old_action.handler = old_handler;
        old_action.flags = old_flags;
        old_action.mask = old_mask;
        old_action.restorer = old_restorer;
        if (copy_to_user((void *)(uintptr_t)oldact,
                         &old_action, sizeof(old_action)) != 0u) {
            return LINUX_EFAULT;
        }
    }
    return 0;
}

int64_t syscall_rt_sigprocmask(uint64_t how, uint64_t set, uint64_t oldset, uint64_t sigsetsize)
{
    if (sigsetsize != sizeof(uint64_t)) return LINUX_EINVAL;
    uint64_t current = process_signal_get_mask();
    if (oldset != 0u &&
        copy_to_user((void *)(uintptr_t)oldset,
                     &current, sizeof(current)) != 0u) {
        return LINUX_EFAULT;
    }
    if (set != 0u) {
        uint64_t requested;
        if (copy_from_user(&requested, (const void *)(uintptr_t)set,
                           sizeof(requested)) != 0u) {
            return LINUX_EFAULT;
        }
        switch (how) {
            case 0u: current |= requested; break;
            case 1u: current &= ~requested; break;
            case 2u: current = requested; break;
            default: return LINUX_EINVAL;
        }
        return process_signal_set_mask(current);
    }
    return 0;
}

int64_t syscall_access(const char *path, int32_t mode)
{
    if (path == NULL || (mode & ~7) != 0) return LINUX_EINVAL;
    vfs_file_t file;
    if (vfs_find_file(path, &file)) {
        /* Existing regular file. There is no execute-permission model here,
         * so X_OK on anything that exists succeeds (xkbcommon / ld.so probe
         * their data directories with access(dir, R_OK|X_OK) and treat
         * EACCES as "missing", which previously crashed libxkbcommon). */
        if ((mode & 2) != 0 &&
            (file.fs_driver == NULL || file.fs_driver->write_at == NULL)) {
            return -13; /* W_OK on a read-only filesystem */
        }
        return 0;
    }
    int32_t dir_handle = vfs_opendir(path);
    if (dir_handle >= 0) {
        (void)vfs_closedir(dir_handle);
        /* W_OK on a directory asks "can I create files here?", so answer from
         * the backing filesystem rather than refusing outright: a blanket
         * EACCES made every writable mount (tmpfs on /tmp, /run, /var) look
         * read-only. Xorg picks its compiled-keymap output directory with
         * exactly this probe and cannot run xkbcomp without one, which is a
         * fatal "Failed to activate virtual core keyboard". */
        if ((mode & 2) != 0 && !vfs_dir_is_writable(path)) {
            return -13; /* EACCES: directory on a read-only filesystem */
        }
        return 0; /* F_OK / R_OK / X_OK on a dir */
    }
    return -2; /* ENOENT */
}

static int64_t linux_socket_sendto(uint64_t fd, uint64_t buf, uint64_t len,
                                   uint64_t flags, uint64_t addr_ptr,
                                   uint64_t addr_len);

int64_t write(int fd, const void *buf, uint64_t count)
{
    if (buf == NULL) {
        return -1;
    }

    if (count != 0u && !process_user_buffer_is_valid(buf, count)) {
        return -1;
    }

    if (syscall_eventfd_is_valid(fd)) {
        uint8_t staged[8];
        if (count < sizeof(staged)) {
            return -1; /* Real Linux: EINVAL for a write() under 8 bytes. */
        }
        if (copy_from_user_trusted(staged, buf, sizeof(staged)) != 0u) {
            return -1;
        }
        return syscall_eventfd_write(fd, staged, sizeof(staged));
    }

    uint8_t chunk[512];
    uint64_t total = 0;

    /* AF_UNIX stream write() (X's _XSERVTransWrite). sendmsg is routed
     * elsewhere; plain write() previously fell through to the file table. */
    if (unix_socket_fd_in_range(fd)) {
        while (total < count) {
            uint64_t n = count - total;
            if (n > sizeof(chunk)) n = sizeof(chunk);
            if (copy_from_user_trusted(chunk, (const uint8_t *)buf + total, n) != 0u) {
                return total != 0u ? (int64_t)total : -1;
            }
            int64_t w = unix_socket_send(fd, chunk, n);
            if (w < 0) return total != 0u ? (int64_t)total : w;
            total += (uint64_t)w;
            if ((uint64_t)w < n) break;
        }
        return (int64_t)total;
    }

    /* AF_INET write(): the same gap read() had -- socket fds are outside the
     * file table, so this failed instead of sending. */
    if (syscall_socket_fd_in_range(fd)) {
        return linux_socket_sendto((uint64_t)fd, (uint64_t)(uintptr_t)buf,
                                   count, 0u, 0u, 0u);
    }

    if ((fd == 1 || fd == 2) &&
        syscall_file_get_file_info(fd, NULL, NULL) != 0) {
        /* fd 1/2 not allocated to this process: treat as console. */
        while (total < count) {
            uint64_t n = count - total;
            if (n > sizeof(chunk)) {
                n = sizeof(chunk);
            }
            if (copy_from_user_trusted(chunk, (const uint8_t *)buf + total, n) != 0u) {
                return total != 0u ? (int64_t)total : -1;
            }
            serial_write_buffer((const char *)chunk, (uint32_t)n);
            total += n;
        }
        return (int64_t)count;
    }

    while (total < count) {
        uint64_t n = count - total;
        if (n > sizeof(chunk)) {
            n = sizeof(chunk);
        }
        if (copy_from_user_trusted(chunk, (const uint8_t *)buf + total, n) != 0u) {
            return total != 0u ? (int64_t)total : -1;
        }
        int64_t written = syscall_file_write(fd, chunk, n);
        if (written < 0) {
            return total != 0u ? (int64_t)total : written;
        }
        total += (uint64_t)written;
        if ((uint64_t)written < n) {
            break;
        }
    }

    return (int64_t)total;
}

/* ------------------------------------------------------------------ */
/* Linux x86_64 syscall number table (phase 2: compatibility layer)   */
/* ------------------------------------------------------------------ */

#define LINUX_SYS_READ           0u
#define LINUX_SYS_WRITE          1u
#define LINUX_SYS_OPEN           2u
#define LINUX_SYS_CLOSE          3u
#define LINUX_SYS_FSTAT          5u
#define LINUX_SYS_LSEEK          8u
#define LINUX_SYS_MMAP           9u
#define LINUX_SYS_MPROTECT      10u
#define LINUX_SYS_MUNMAP        11u
#define LINUX_SYS_BRK           12u
#define LINUX_SYS_RT_SIGACTION  13u
#define LINUX_SYS_RT_SIGPROCMASK 14u
#define LINUX_SYS_RT_SIGRETURN  15u
#define LINUX_SYS_IOCTL         16u
#define LINUX_SYS_READV         19u
#define LINUX_SYS_WRITEV        20u
#define LINUX_SYS_SELECT        23u
#define LINUX_SYS_PSELECT6     270u
#define LINUX_SYS_ACCESS        21u
#define LINUX_SYS_PIPE          22u
#define LINUX_SYS_SCHED_YIELD   24u
#define LINUX_SYS_MREMAP        25u
#define LINUX_SYS_MINCORE       27u
#define LINUX_SYS_MADVISE       28u
#define LINUX_SYS_DUP           32u
#define LINUX_SYS_DUP2          33u
#define LINUX_SYS_NANOSLEEP     35u
#define LINUX_SYS_GETITIMER     36u
#define LINUX_SYS_SETITIMER     38u
#define LINUX_SYS_GETPID        39u
#define LINUX_SYS_SENDFILE      40u
#define LINUX_SYS_CLONE         56u
#define LINUX_SYS_FORK          57u
#define LINUX_SYS_VFORK         58u
#define LINUX_SYS_EXECVE        59u
#define LINUX_SYS_EXIT          60u
#define LINUX_SYS_WAIT4         61u
#define LINUX_SYS_KILL          62u
#define LINUX_SYS_UNAME         63u
#define LINUX_SYS_FCNTL         72u
#define LINUX_SYS_FTRUNCATE     77u
#define LINUX_SYS_FALLOCATE     285u
#define LINUX_SYS_GETCWD        79u
#define LINUX_SYS_CHDIR         80u
#define LINUX_SYS_RENAME        82u
#define LINUX_SYS_MKDIR         83u
#define LINUX_SYS_RMDIR         84u
#define LINUX_SYS_CREAT         85u
#define LINUX_SYS_LINK          86u
#define LINUX_SYS_UNLINK        87u
#define LINUX_SYS_READLINK      89u
#define LINUX_SYS_SYMLINK       88u
#define LINUX_SYS_SYMLINKAT    266u
#define LINUX_SYS_LINKAT       265u
#define LINUX_SYS_GETTIMEOFDAY  96u
#define LINUX_SYS_GETRLIMIT     97u
#define LINUX_SYS_SYSINFO       99u
#define LINUX_SYS_GETUID       102u
#define LINUX_SYS_GETGID       104u
#define LINUX_SYS_GETEUID      107u
#define LINUX_SYS_GETEGID      108u
#define LINUX_SYS_GETPPID      110u
#define LINUX_SYS_STATFS       137u
#define LINUX_SYS_FSTATFS      138u
#define LINUX_SYS_SIGALTSTACK  131u
#define LINUX_SYS_PRCTL        157u
#define LINUX_SYS_ARCH_PRCTL   158u
#define LINUX_SYS_SETRLIMIT    160u
#define LINUX_SYS_GETTID       186u
#define LINUX_SYS_TKILL        200u
#define LINUX_SYS_TIME         201u
#define LINUX_SYS_FUTEX        202u
#define LINUX_SYS_SCHED_SETAFFINITY 203u
#define LINUX_SYS_SCHED_GETAFFINITY 204u
#define LINUX_SYS_GETDENTS64   217u
#define LINUX_SYS_SET_TID_ADDRESS 218u
#define LINUX_SYS_CLOCK_GETTIME 228u
#define LINUX_SYS_CLOCK_GETRES  229u
#define LINUX_SYS_EXIT_GROUP   231u
#define LINUX_SYS_EPOLL_WAIT   232u
#define LINUX_SYS_EPOLL_CTL    233u
#define LINUX_SYS_TGKILL       234u
#define LINUX_SYS_OPENAT       257u
#define LINUX_SYS_NEWFSTATAT   262u
#define LINUX_SYS_SET_ROBUST_LIST 273u
#define LINUX_SYS_READLINKAT   267u
#define LINUX_SYS_TIMERFD_CREATE 283u
#define LINUX_SYS_EVENTFD      284u
#define LINUX_SYS_TIMERFD_SETTIME 286u
#define LINUX_SYS_TIMERFD_GETTIME 287u
#define LINUX_SYS_SIGNALFD4    289u
#define LINUX_SYS_EVENTFD2     290u
#define LINUX_SYS_EPOLL_CREATE1 291u
#define LINUX_SYS_PRLIMIT64    302u
#define LINUX_SYS_GETCPU       309u
#define LINUX_SYS_GETRANDOM    318u
#define LINUX_SYS_MEMFD_CREATE 319u
#define LINUX_SYS_STATX        332u
/* Namespaces, chroot and seccomp -- the calls Chromium's sandbox makes.
 * See linux_unshare()/linux_chroot()/linux_seccomp() for what each one
 * actually does here. */
#define LINUX_SYS_PTRACE       101u
#define LINUX_SYS_PIVOT_ROOT   155u
#define LINUX_SYS_CHROOT       161u
#define LINUX_SYS_MOUNT        165u
#define LINUX_SYS_UMOUNT2      166u
#define LINUX_SYS_UNSHARE      272u
#define LINUX_SYS_SETNS        308u
#define LINUX_SYS_SECCOMP      317u
#define LINUX_SYS_NAME_TO_HANDLE_AT 303u
#define LINUX_SYS_RSEQ         334u

/* Additional Linux x86_64 syscall numbers - self-contained glibc/Chromium
 * support (TODO_Chromium_LinuxABI.md section 3.8 / section 4). */
#define LINUX_SYS_PREAD64       17u
#define LINUX_SYS_PWRITE64      18u
#define LINUX_SYS_SYNC          162u
#define LINUX_SYS_FSYNC         74u
#define LINUX_SYS_FDATASYNC     75u
#define LINUX_SYS_SYNCFS        306u
#define LINUX_SYS_FLOCK         73u
#define LINUX_SYS_FADVISE64     221u
#define LINUX_SYS_GETRUSAGE     98u
#define LINUX_SYS_UMASK         95u
#define LINUX_SYS_GETPGID       121u
#define LINUX_SYS_SETPGID       109u
#define LINUX_SYS_GETPGRP       111u
#define LINUX_SYS_SETSID        112u
#define LINUX_SYS_GETSID        124u
#define LINUX_SYS_PERSONALITY   135u
#define LINUX_SYS_GETPRIORITY   140u
#define LINUX_SYS_SETPRIORITY   141u
#define LINUX_SYS_SCHED_GETPARAM        143u
#define LINUX_SYS_SCHED_SETPARAM        142u
#define LINUX_SYS_SCHED_SETSCHEDULER    144u
#define LINUX_SYS_SCHED_GETSCHEDULER    145u
#define LINUX_SYS_SCHED_GET_PRIORITY_MAX 146u
#define LINUX_SYS_SCHED_GET_PRIORITY_MIN 147u
#define LINUX_SYS_MLOCK        149u
#define LINUX_SYS_MUNLOCK      150u
#define LINUX_SYS_MLOCKALL     151u
#define LINUX_SYS_MUNLOCKALL   152u
#define LINUX_SYS_MLOCK2       325u
#define LINUX_SYS_MEMBARRIER   324u
#define LINUX_SYS_GETSOCKNAME  51u
#define LINUX_SYS_GETPEERNAME  52u
#define LINUX_SYS_CLOCK_NANOSLEEP 230u
#define LINUX_SYS_EPOLL_PWAIT  281u
#define LINUX_SYS_EPOLL_PWAIT2 441u
#define LINUX_SYS_POLL          7u
#define LINUX_SYS_PPOLL        271u
#define LINUX_SYS_FACCESSAT    269u
#define LINUX_SYS_FACCESSAT2   439u
#define LINUX_SYS_PIPE2        293u
#define LINUX_SYS_DUP3         292u
#define LINUX_SYS_WAITID       247u
#define LINUX_SYS_INOTIFY_INIT  253u
#define LINUX_SYS_INOTIFY_ADD_WATCH 254u
#define LINUX_SYS_INOTIFY_RM_WATCH  255u
#define LINUX_SYS_INOTIFY_INIT1 294u
#define LINUX_SYS_SENDMMSG     307u
#define LINUX_SYS_RECVMMSG     299u
#define LINUX_SYS_UTIMENSAT    280u
#define LINUX_SYS_MKDIRAT     258u
#define LINUX_SYS_UNLINKAT    263u
#define LINUX_SYS_RENAMEAT    264u
#define LINUX_SYS_RENAMEAT2   316u
#define LINUX_SYS_FCHMOD      91u
#define LINUX_SYS_FCHMODAT    268u
#define LINUX_SYS_CHMOD       90u
#define LINUX_SYS_FCHOWN      93u
#define LINUX_SYS_CHOWN       92u
#define LINUX_SYS_LCHOWN      94u
#define LINUX_SYS_FCHOWNAT    260u
#define LINUX_SYS_RT_SIGSUSPEND 130u
#define LINUX_SYS_RT_SIGPENDING 127u
#define LINUX_SYS_PAUSE       34u
#define LINUX_SYS_ALARM       37u
#define LINUX_SYS_SETUID      105u
#define LINUX_SYS_SETGID      106u
#define LINUX_SYS_SETREUID    113u
#define LINUX_SYS_SETREGID    114u
#define LINUX_SYS_SETRESUID   117u
#define LINUX_SYS_SETRESGID   119u
#define LINUX_SYS_GETRESUID   118u
#define LINUX_SYS_GETRESGID   120u
#define LINUX_SYS_SETGROUPS   116u
#define LINUX_SYS_GETGROUPS   115u
#define LINUX_SYS_CAPGET      125u
#define LINUX_SYS_CAPSET      126u
#define LINUX_SYS_SYSLOG      103u
#define LINUX_SYS_PRLIMIT64_  302u

#define LINUX_EPERM   (-1LL)
#define LINUX_ECHILD  (-10LL)
#define LINUX_EEXIST  (-17LL)
#define LINUX_ERANGE  (-34LL)

#define LINUX_SYS_SOCKET        41u
#define LINUX_SYS_CONNECT       42u
#define LINUX_SYS_ACCEPT        43u
#define LINUX_SYS_SENDTO        44u
#define LINUX_SYS_RECVFROM      45u
#define LINUX_SYS_SENDMSG       46u
#define LINUX_SYS_RECVMSG       47u
#define LINUX_SYS_SHUTDOWN      48u
#define LINUX_SYS_BIND          49u
#define LINUX_SYS_LISTEN        50u
#define LINUX_SYS_SOCKETPAIR    53u
#define LINUX_SYS_SETSOCKOPT    54u
#define LINUX_SYS_GETSOCKOPT    55u

#define LINUX_SYS_STAT          4u
#define LINUX_SYS_LSTAT         6u

#define LINUX_MAP_SHARED       0x01u
#define LINUX_MAP_PRIVATE      0x02u
#define LINUX_MAP_FIXED        0x10u
#define LINUX_MAP_ANONYMOUS    0x20u
/* File mappings at or above this size are demand-paged (Core/memory/FileMap.c)
 * instead of read in up front. Set to catch shared-object images -- the ones
 * that made resident sets explode -- while leaving small mappings on the
 * long-proven eager path. */
#define LINUX_MMAP_LAZY_MIN_BYTES (1024u * 1024u)

#define LINUX_PROT_READ        0x1u
#define LINUX_PROT_WRITE       0x2u
#define LINUX_PROT_EXEC        0x4u

#define LINUX_SEEK_SET         0u
#define LINUX_SEEK_CUR         1u
#define LINUX_SEEK_END         2u

#define LINUX_O_CREAT          0x40u
#define LINUX_O_DIRECTORY      0x10000u

#define LINUX_AT_FDCWD         (-100)
#define LINUX_AT_SYMLINK_NOFOLLOW 0x100u

#define LINUX_WNOHANG          1u

/* Bounded slice a blocking wait4()/waitid() parks the calling thread for when
 * a child exists but has not exited yet. This kernel has no "wake the parent
 * on child exit" primitive, so - exactly like syscall_epoll_wait_ex() - the
 * blocking wait degrades to a slow timed poll instead of returning 0
 * immediately (which made glibc's wait loop spin at 100% CPU until the child
 * died). WNOHANG callers never reach this path. */
#define LINUX_WAIT_POLL_SLICE_MS 10u

#define LINUX_PR_SET_NAME      15u
#define LINUX_PR_GET_NAME      16u

#define LINUX_CLONE_VM              0x00000100u
#define LINUX_CLONE_FS              0x00000200u
/* 0x00008000 is CLONE_PARENT ("share my parent"), not CLONE_PARENT_SETTID.
 * With the wrong value every pthread_create() looked like it had not asked
 * for the child's TID to be stored, so glibc's `pd->tid` stayed 0 in every
 * thread but the first. glibc's rwlocks compare that field against the
 * lock's __cur_writer to detect self-deadlock, and a zeroed tid matches the
 * zero of an unheld lock, so g_rw_lock_reader_lock() failed with EDEADLK on
 * every GObject type lookup made off the main thread. */
#define LINUX_CLONE_PARENT_SETTID   0x00100000u
#define LINUX_CLONE_CHILD_CLEARTID  0x00200000u
#define LINUX_CLONE_THREAD          0x00010000u
#define LINUX_CLONE_CHILD_SETTID    0x01000000u
#define LINUX_CLONE_SETTLS          0x00080000u

#define LINUX_S_IFMT   0xF000u
#define LINUX_S_IFDIR  0x4000u
#define LINUX_S_IFCHR  0x2000u
#define LINUX_S_IFREG  0x8000u
#define LINUX_S_IFIFO  0x1000u

#define LINUX_AF_UNIX       1u
#define LINUX_AF_INET       2u
#define LINUX_SOCK_STREAM   1u
#define LINUX_SOCK_DGRAM    2u
#define LINUX_SOL_SOCKET    1u
#define LINUX_SO_REUSEADDR  2u
#define LINUX_SO_TYPE       3u
#define LINUX_SO_ERROR      4u
#define LINUX_SO_BROADCAST  6u
#define LINUX_SO_SNDBUF     7u
#define LINUX_SO_RCVBUF     8u
#define LINUX_SO_KEEPALIVE  9u
#define LINUX_SO_LINGER     13u
#define LINUX_SO_REUSEPORT  15u
#define LINUX_SO_PASSCRED   16u
#define LINUX_SO_PEERCRED   17u
#define LINUX_SO_RCVTIMEO   20u
#define LINUX_SO_SNDTIMEO   21u
#define LINUX_SO_PROTOCOL   38u
#define LINUX_SO_PASSSEC    34u
#define LINUX_SHUT_RD       0
#define LINUX_SHUT_WR       1
#define LINUX_SHUT_RDWR     2
#define LINUX_EAFNOSUPPORT  (-97LL)
#define LINUX_EPROTONOSUPPORT (-93LL)
#define LINUX_ENOPROTOOPT   (-92LL)

#define LINUX_ENOSYS (-38LL)
#define LINUX_ENOMEM (-12LL)
#define LINUX_ENOENT (-2LL)
#define LINUX_EACCES (-13LL)
#define LINUX_EAGAIN (-11LL)
#define LINUX_EBUSY  (-16LL)
#define LINUX_ENOTDIR (-20LL)
#define LINUX_EISDIR (-21LL)
#define LINUX_ENOTEMPTY (-39LL)
#define LINUX_ENAMETOOLONG (-36LL)

#define LINUX_UTSNAME_LEN 65u

#define LINUX_MAX_IO_BYTES (4ULL * 1024ULL * 1024ULL)

static void linux_syscall_result(uint64_t saved_rsp, int64_t value)
{
#if defined(__x86_64__)
    uint64_t *frame = (uint64_t *)(uintptr_t)saved_rsp;
    frame[SYSCALL_FRAME_RAX] = (uint64_t)value;
#else
    (void)saved_rsp;
    (void)value;
#endif
}

/* Re-arm the trap so the caller executes the very same syscall again the next
 * time it is scheduled, instead of returning a value to userspace. This is how
 * Linux restarts an interrupted blocking call (ERESTARTSYS): put the syscall
 * number back in rax and step rip back over the two-byte `syscall` (0f 05).
 * Every argument register is restored from this frame on the way out, so the
 * re-executed call sees exactly the arguments it was given. */
static void linux_syscall_restart(uint64_t saved_rsp, uint64_t num)
{
#if defined(__x86_64__)
    uint64_t *frame = (uint64_t *)(uintptr_t)saved_rsp;
    frame[SYSCALL_FRAME_RAX] = num;
    frame[SYSCALL_FRAME_RCX] -= 2u;
#else
    (void)saved_rsp;
    (void)num;
#endif
}

static int64_t linux_copy_cstring(char *out, uint64_t capacity,
                                  const char *user_ptr)
{
    if (user_ptr == NULL) {
        return LINUX_EFAULT;
    }
    uint64_t len = 0;
    if (process_user_cstring_length(user_ptr, capacity - 1u, &len) < 0) {
        return LINUX_EFAULT;
    }
    if (copy_from_user_trusted(out, user_ptr, len + 1u) != 0u) {
        return LINUX_EFAULT;
    }
#ifdef LINUX_SYSCALL_TRACE
    /* Every path-taking syscall funnels through here, so this is the one place
     * that can name the file a traced syscall is about. Without it a trace is
     * a list of pointers and a bring-up boot cannot tell which path failed.
     * Silent until the trace arms, for the reason given at linux_trace_gate(). */
    extern bool linux_trace_is_armed(void);
    if (!linux_trace_is_armed()) return 0;
    serial_write_string("[lxstr] '");
    serial_write_string(out);
    serial_write_string("'\n");
#endif
    return 0;
}

static int64_t linux_realtime_seconds(void)
{
    return clock_realtime_ns() / 1000000000LL;
}

static int64_t linux_brk(uint64_t addr)
{
    /* The break has its own window now (process_set_brk). It used to run
     * through process_set_heap_cursor() -> process_user_alloc(), i.e. the same
     * bump/free-list allocator that hands out addresses for mmap()ed shared
     * objects -- so a break could be "granted" at an address the kernel had
     * mapped somewhere else entirely, and a later dlopen() could be placed on
     * top of glibc's malloc arena. See TODO_Doom_Xorg_MethodA.md M20.
     *
     * brk(2) reports the resulting break either way: on failure the caller
     * sees the break unchanged and falls back to mmap(). */
    uint64_t cursor = process_get_brk();
    if (addr == 0u) {
        return (int64_t)cursor;
    }
    if (process_set_brk(addr) < 0) {
        return (int64_t)cursor;
    }
    return (int64_t)addr;
}

/* ---- file-backed MAP_SHARED write-back (TODO_Chromium_LinuxABI.md bucket B) ----
 *
 * The kernel VFS has no page cache, so a file MAP_SHARED cannot give live
 * cross-process coherency. What this does provide: a single writer's changes
 * to a MAP_SHARED file mapping are flushed back to the file on msync(2) and on
 * munmap(2)/exit - i.e. mmap()-as-file-write works. A dup of the fd is held so
 * the mapping outlives a user close(). memfd / /dev/shm shared memory (the
 * path Chromium actually relies on) is unaffected - it is tmpfs-backed and
 * already coherent.
 *
 * The other direction matters just as much: a file written with write() or
 * pwrite() while it is mapped. The mapping is a copy taken at mmap() time, so
 * without help it keeps showing the old bytes. SQLite does exactly this -- it
 * writes pages with pwrite() and reads them back through a read-only
 * MAP_SHARED mapping -- and every database Chromium created read back a
 * schema from before its own tables existed, failed with "invalid SQL
 * statement", and was razed ("Something went wrong when opening your
 * profile"). linux_mshared_note_write() copies each write into the mappings
 * of that file in the writer's address space. And only writable mappings are
 * written back: flushing a read-only snapshot on munmap() put stale pages
 * back over the file every time SQLite remapped a growing database. */
#define LINUX_MSHARED_MAX 96
typedef struct {
    int32_t  in_use;
    int32_t  owner_pid;
    int32_t  fd;          /* our dup'd copy */
    uint64_t uaddr;
    uint64_t length;
    uint64_t file_offset;
    uint64_t writeback_len; /* min(length, file bytes from offset at map time) */
    uint64_t file_key;      /* syscall_file_identity() of the mapped file */
    uint64_t cr3;           /* address space the mapping lives in */
    int32_t  writable;      /* PROT_WRITE: flush back to the file */
} linux_mshared_t;

static linux_mshared_t g_linux_mshared[LINUX_MSHARED_MAX];
static spinlock_t g_linux_mshared_lock;
static int g_linux_mshared_ready;

static int64_t linux_pwrite64(uint64_t fd, uint64_t buf, uint64_t count,
                              uint64_t offset);

static void linux_mshared_init_once(void)
{
    if (!g_linux_mshared_ready) {
        spinlock_init(&g_linux_mshared_lock);
        g_linux_mshared_ready = 1;
    }
}

/* A regular file was written: bring every MAP_SHARED copy of that range in
 * the writer's address space up to date. */
static void linux_mshared_note_write(int32_t fd, uint32_t offset,
                                     const uint8_t *data, uint64_t len)
{
    if (!g_linux_mshared_ready || data == NULL || len == 0u) {
        return;
    }
    uint64_t key = syscall_file_identity(fd);
    uint64_t cr3 = process_get_current_cr3();
    if (key == 0u || cr3 == 0u) {
        return;
    }
    uint64_t w_lo = (uint64_t)offset;
    uint64_t w_hi = w_lo + len;
    for (int i = 0; i < LINUX_MSHARED_MAX; ++i) {
        uint64_t uaddr = 0u, foff = 0u, mlen = 0u;
        int match = 0;
        spinlock_lock(&g_linux_mshared_lock);
        const linux_mshared_t *e = &g_linux_mshared[i];
        if (e->in_use && e->file_key == key && e->cr3 == cr3) {
            uaddr = e->uaddr;
            foff = e->file_offset;
            mlen = e->length;
            match = 1;
        }
        spinlock_unlock(&g_linux_mshared_lock);
        if (!match) {
            continue;
        }
        uint64_t lo = w_lo > foff ? w_lo : foff;
        uint64_t hi = w_hi < foff + mlen ? w_hi : foff + mlen;
        if (lo >= hi) {
            continue;
        }
        (void)copy_to_user_trusted((void *)(uintptr_t)(uaddr + (lo - foff)),
                                   data + (lo - w_lo), hi - lo);
    }
}

static void linux_mshared_register(int32_t owner_pid, int32_t src_fd,
                                   uint64_t uaddr, uint64_t length,
                                   uint64_t file_offset, uint64_t writeback_len,
                                   int32_t writable)
{
    linux_mshared_init_once();
    syscall_file_set_write_observer(linux_mshared_note_write);
    uint64_t file_key = syscall_file_identity(src_fd);
    int32_t dup_fd = syscall_file_dup(src_fd);
    if (dup_fd < 0) {
        return; /* best effort: fall back to private-copy semantics */
    }
    spinlock_lock(&g_linux_mshared_lock);
    for (int i = 0; i < LINUX_MSHARED_MAX; ++i) {
        if (!g_linux_mshared[i].in_use) {
            g_linux_mshared[i].in_use = 1;
            g_linux_mshared[i].owner_pid = owner_pid;
            g_linux_mshared[i].fd = dup_fd;
            g_linux_mshared[i].uaddr = uaddr;
            g_linux_mshared[i].length = length;
            g_linux_mshared[i].file_offset = file_offset;
            g_linux_mshared[i].writeback_len = writeback_len;
            g_linux_mshared[i].file_key = file_key;
            g_linux_mshared[i].cr3 = process_get_current_cr3();
            g_linux_mshared[i].writable = writable;
            spinlock_unlock(&g_linux_mshared_lock);
            return;
        }
    }
    spinlock_unlock(&g_linux_mshared_lock);
    (void)syscall_file_close(dup_fd);
}

/* Flush (and optionally unregister) every registered MAP_SHARED file mapping
 * owned by owner_pid that overlaps [lo,hi). Runs in the owner's context, so
 * the mapped user pages are directly readable. */
static void linux_mshared_flush_range(int32_t owner_pid, uint64_t lo,
                                      uint64_t hi, int unregister)
{
    if (!g_linux_mshared_ready) {
        return;
    }
    for (int i = 0; i < LINUX_MSHARED_MAX; ++i) {
        int32_t fd = -1;
        uint64_t uaddr = 0, wlen = 0, foff = 0;
        int do_unreg = 0;
        int writable = 0;

        spinlock_lock(&g_linux_mshared_lock);
        linux_mshared_t *e = &g_linux_mshared[i];
        if (e->in_use && e->owner_pid == owner_pid &&
            e->uaddr < hi && (e->uaddr + e->length) > lo) {
            fd = e->fd;
            uaddr = e->uaddr;
            wlen = e->writeback_len;
            foff = e->file_offset;
            writable = e->writable;
            if (unregister) {
                e->in_use = 0;
                do_unreg = 1;
            }
        }
        spinlock_unlock(&g_linux_mshared_lock);

        if (fd < 0) {
            continue;
        }
        uint64_t done = 0;
        while (writable && done < wlen) {
            uint64_t want = wlen - done;
            if (want > LINUX_MAX_IO_BYTES) want = LINUX_MAX_IO_BYTES;
            int64_t w = linux_pwrite64((uint64_t)fd, uaddr + done, want,
                                       foff + done);
            if (w <= 0) break;
            done += (uint64_t)w;
        }
        if (do_unreg) {
            (void)syscall_file_close(fd);
        }
    }
}

/* Called from the exit path (declared in the header) to drop this process's
 * MAP_SHARED file mappings - flush is skipped here because the address space
 * is already being torn down; munmap/msync already flushed the live ones. */
void linux_compat_mshared_release_pid(int32_t pid)
{
    if (!g_linux_mshared_ready) {
        return;
    }
    for (int i = 0; i < LINUX_MSHARED_MAX; ++i) {
        int32_t fd = -1;
        spinlock_lock(&g_linux_mshared_lock);
        if (g_linux_mshared[i].in_use && g_linux_mshared[i].owner_pid == pid) {
            fd = g_linux_mshared[i].fd;
            g_linux_mshared[i].in_use = 0;
        }
        spinlock_unlock(&g_linux_mshared_lock);
        if (fd >= 0) {
            (void)syscall_file_close(fd);
        }
    }
}

/* Module map trace. A foreign binary's crash reports a raw RIP, and every
 * shared object it runs from was placed by us -- so without this the only way
 * to turn "RIP 0x410087B004" into "libevdev.so.2+0x1234" is to guess the load
 * base. Records which fd a .so was opened on, then the base each file-backed
 * mmap of that fd got. Two lines per shared object, only for .so files. */
#define LINUX_MODULE_MAP_TRACE 1

#if LINUX_MODULE_MAP_TRACE
static int linux_path_is_shared_object(const char *path)
{
    uint64_t n = strlen(path);
    for (uint64_t i = 0; i + 2 < n; ++i) {
        if (path[i] == '.' && path[i + 1] == 's' && path[i + 2] == 'o' &&
            (path[i + 3] == '\0' || path[i + 3] == '.')) {
            return 1;
        }
    }
    return 0;
}

/* Keyed by (pid, fd): Xorg and the Doom client run at the same time and their
 * fd numbers collide, so an fd-only table hands out the wrong module name. */
static int32_t g_module_map_fds[128];
static int32_t g_module_map_pids[128];
static char    g_module_map_names[128][64];

static void linux_module_map_note_open(int32_t fd, const char *path)
{
    if (fd < 0 || !linux_path_is_shared_object(path)) return;
    int32_t pid = process_get_current_pid();
    uint64_t n = strlen(path);
    uint64_t start = 0;
    for (uint64_t i = 0; i < n; ++i) if (path[i] == '/') start = i + 1u;
    uint32_t slot = 128u;
    for (uint32_t i = 0; i < 128u; ++i) {
        if (g_module_map_fds[i] == fd + 1 && g_module_map_pids[i] == pid) {
            slot = i;
            break;
        }
        if (slot == 128u && g_module_map_fds[i] == 0) slot = i;
    }
    if (slot == 128u) return;
    g_module_map_fds[slot] = fd + 1;
    g_module_map_pids[slot] = pid;
    uint64_t j = 0;
    for (; j < 63u && path[start + j]; ++j) {
        g_module_map_names[slot][j] = path[start + j];
    }
    g_module_map_names[slot][j] = '\0';
}

static const char *linux_module_map_name(int32_t fd)
{
    int32_t pid = process_get_current_pid();
    for (uint32_t i = 0; i < 128u; ++i) {
        if (g_module_map_fds[i] == fd + 1 && g_module_map_pids[i] == pid) {
            return g_module_map_names[i];
        }
    }
    return NULL;
}

static void linux_module_map_note_mmap(int32_t fd, uint64_t base, uint64_t len,
                                       uint64_t offset)
{
    if (!OS_CONFIG_FOREIGN_TRACE && !CHROME_SHM_TRACE) {
        (void)base; (void)len; (void)offset;
        return;
    }
    const char *name = linux_module_map_name(fd);
    if (name == NULL) return;
    /* pid= is essential here: Xorg and the Doom client load their modules
     * concurrently onto the same shared COM1, and a faulting RIP can only be
     * resolved against the map of the process that faulted. */
    serial_write_string("[lxmap] pid=");
    serial_write_uint32((uint32_t)process_get_current_pid());
    serial_write_char(' ');
    serial_write_string(name);
    serial_write_string(" base=");
    serial_write_uint64(base);
    serial_write_string(" len=");
    serial_write_uint64(len);
    serial_write_string(" off=");
    serial_write_uint64(offset);
    serial_write_char('\n');
}
#endif

#ifndef PAGING_LOST_MAPPING_TRACE
#define PAGING_LOST_MAPPING_TRACE 0
#endif
#if PAGING_LOST_MAPPING_TRACE
/* Anything that touches the window the main executable is mapped into is
 * suspect while chasing pages of that image reverting to demand-zero. */
static void lostmap_note(const char *what, uint64_t addr, uint64_t len,
                         uint64_t extra)
{
    if (addr + len <= 0x4000000000ULL || addr >= 0x4080000000ULL) {
        return;
    }
    static volatile uint32_t seen;
    if (__atomic_fetch_add(&seen, 1u, __ATOMIC_RELAXED) >= 32u) {
        return;
    }
    serial_write_string("[lostmap] ");
    serial_write_string(what);
    serial_write_string(" addr=");
    serial_write_uint64(addr);
    serial_write_string(" len=");
    serial_write_uint64(len);
    serial_write_string(" x=");
    serial_write_uint64(extra);
    serial_write_string("\n");
}
#endif

static int64_t linux_mmap(uint64_t addr, uint64_t length, uint64_t prot,
                          uint64_t flags, uint64_t fd, uint64_t offset)
{
#if PAGING_LOST_MAPPING_TRACE
    lostmap_note("mmap", addr, length, flags);
#endif
    (void)prot;
    if (length == 0u) {
        return LINUX_EINVAL;
    }
    if ((flags & LINUX_MAP_ANONYMOUS) != 0u) {
        if ((flags & LINUX_MAP_FIXED) != 0u) {
            if (addr == 0u || addr < 0x1000u ||
                (addr & (PAGE_SIZE - 1u)) != 0u) {
                return LINUX_EINVAL;
            }
            /* MAP_FIXED into the lazily-committed mmap arena (typically a
             * PartitionAlloc pool sub-range): accept the address, commit
             * nothing -- demand-zero on first touch. Elsewhere (glibc placing
             * something into the heap) keep the eager mapping. */
            if (process_user_addr_in_mmap_arena(addr, length)) {
                /* MAP_FIXED replaces whatever is mapped there with fresh zero
                 * pages. Committing nothing is only half of that: pages
                 * already present kept their old contents, and PartitionAlloc
                 * decommits-and-zeroes a span exactly this way
                 * (mmap(MAP_FIXED, PROT_NONE)) and then builds allocator
                 * metadata on it believing it is zero -- which is how
                 * Chromium's heap came back full of its own pixels and CSS
                 * text. Drop them, so the next touch demand-zeroes. */
                uint64_t cr3 = process_get_current_cr3();
                if (cr3 != 0u) {
                    (void)paging_unmap_range(cr3, addr, length);
                }
                /* Anonymous pages must read as zero even where they land on
                 * top of a demand-paged file mapping -- this is precisely how
                 * a shared object's .bss is placed over the mapping of its own
                 * image. Shadow the range so its faults go to demand-zero
                 * instead of being filled from the file. */
                int32_t self = process_get_current_pid();
                if (self >= 0) {
                    (void)filemap_register_zero(self, addr, length);
                }
                return (int64_t)addr;
            }
            if (length > 0x4000000000ULL - addr) {
                return LINUX_EINVAL;
            }
            uint64_t cr3 = process_get_current_cr3();
            if (cr3 == 0u ||
                paging_map_user_range_alloc(cr3, addr, length,
                                            PAGE_RW | PAGE_USER) < 0) {
                return LINUX_ENOMEM;
            }
            return (int64_t)addr;
        }
        /* Anonymous, kernel picks the address: hand out a lazily-committed
         * reservation from the mmap arena (no 4 GiB cap, no physical backing
         * until touched). */
        void *mapped = process_user_reserve(length);
        if (mapped == NULL) {
            return LINUX_ENOMEM;
        }
        return (int64_t)(uintptr_t)mapped;
    }

    /* memfd promoted to a shared-memory backing (Wayland wl_shm pool): map the
     * real shared pages so the client's drawing is visible to whoever else
     * maps the same object (the compositor, after SCM_RIGHTS). The generic
     * file path below only makes a one-time snapshot copy, which is useless
     * for shared memory. offset is expected to be 0 for a pool mmap. */
    {
        int32_t shm_handle = syscall_memfd_shm_handle((int32_t)fd);
        if (shm_handle >= 0) {
            if (offset != 0u) {
                return LINUX_EINVAL;
            }
            /* Whoever holds a descriptor for a memfd may map it -- that is
             * the whole access model on Linux, and the descriptor layer has
             * already checked this caller holds one. The per-object grant
             * (one pid) cannot describe a region handed through the zygote
             * to a child it then forked: the child's mmap failed with ENOMEM
             * and Chromium died on "Pseudonymization salt must be
             * initialized in child processes". */
            (void)shared_memory_set_public(shm_handle);
            /* A fresh mapping per mmap(): Chromium keys its shared-memory
             * bookkeeping on the returned address. */
            void *p = shared_memory_map_new(shm_handle);
            if (p == NULL) {
                return LINUX_ENOMEM;
            }
            return (int64_t)(uintptr_t)p;
        }
    }

    /* MAP_SHARED of a tmpfs file (Chromium's shared memory lives in /dev/shm
     * files): map the file's shared pages, so every mapping -- in this process
     * or another -- sees the same bytes. The snapshot copy below made each
     * mapping private, so a buffer one side wrote was never seen by the other
     * (the GPU thread crashed on its first new frame after a click, and pages
     * could stay blank for good). */
#ifndef LINUX_TMPFS_SHARED_MMAP
/* Off: mapping tmpfs files through shared-memory objects deadlocked
 * Chromium at startup (every thread parked in futex before the profile
 * loaded). Kept switchable while that is investigated. */
#define LINUX_TMPFS_SHARED_MMAP 0
#endif
    if (LINUX_TMPFS_SHARED_MMAP &&
        (flags & LINUX_MAP_SHARED) != 0u && offset == 0u &&
        (flags & LINUX_MAP_FIXED) == 0u) {
        int32_t tmpfs_handle = syscall_file_tmpfs_share((int32_t)fd, length);
        if (tmpfs_handle > 0) {
            void *p = shared_memory_map_new(tmpfs_handle);
            if (p != NULL) {
                return (int64_t)(uintptr_t)p;
            }
        }
    }

    /* Character device (DRM/KMS): map the dumb buffer whose fake mmap offset
     * a prior DRM_IOCTL_MODE_MAP_DUMB handed back. See TODO_Doom_Xorg_MethodA.md
     * M2/M4. */
    if (syscall_file_is_chardev((int32_t)fd)) {
        return syscall_file_dev_mmap((int32_t)fd, offset, length, prot, flags);
    }

    vfs_file_t vf;
    if (syscall_file_get_file_info((int32_t)fd, &vf, NULL) < 0) {
        return LINUX_EBADF;
    }
    if ((offset & (PAGE_SIZE - 1u)) != 0u) {
        return LINUX_EINVAL;
    }
    int64_t saved_offset = -1;
    if (offset < (uint64_t)vf.size) {
        saved_offset = syscall_file_seek((int32_t)fd, 0, LINUX_SEEK_CUR);
        if (saved_offset < 0 ||
            syscall_file_seek((int32_t)fd, (int64_t)offset, LINUX_SEEK_SET) < 0) {
            return LINUX_ENODEV;
        }
    }
    uint64_t read_len = 0;
    if (offset < (uint64_t)vf.size) {
        read_len = length;
        uint64_t remaining = (uint64_t)vf.size - offset;
        if (read_len > remaining) {
            read_len = remaining;
        }
    }

    /* Large file mappings are demand-paged rather than read in up front.
     * glibc's loader maps every shared object twice over (a mapping spanning
     * the whole image, then MAP_FIXED segment mappings on top of it), so
     * eagerly backing each one made Xorg's resident set multi-gigabyte and
     * fork() unusable -- see Core/memory/FileMap.c. Below the threshold the
     * proven eager path is kept: small mappings are cheap, and it bounds how
     * many records the (shared) filemap table has to hold. */
    if (length >= LINUX_MMAP_LAZY_MIN_BYTES &&
        (flags & LINUX_MAP_SHARED) == 0u) {
        int32_t pid = process_get_current_pid();
        int32_t handle = syscall_file_mmap_acquire((int32_t)fd);
        if (handle >= 0) {
            void *reserved;
            if ((flags & LINUX_MAP_FIXED) != 0u) {
                if (addr == 0u || addr < 0x1000u ||
                    length > 0x4000000000ULL - addr) {
                    syscall_file_mmap_release(handle);
                    return LINUX_EINVAL;
                }
                reserved = (void *)(uintptr_t)addr;
                /* As for anonymous MAP_FIXED: pages already present would
                 * hide the file's contents. Drop them so this range faults in
                 * from the file. */
                if (process_user_addr_in_mmap_arena(addr, length)) {
                    uint64_t fixed_cr3 = process_get_current_cr3();
                    if (fixed_cr3 != 0u) {
                        (void)paging_unmap_range(fixed_cr3, addr, length);
                    }
                }
            } else {
                reserved = process_user_reserve(length);
            }
            if (reserved != NULL &&
                filemap_register(pid, (uint64_t)(uintptr_t)reserved, length,
                                 handle, offset,
                                 PAGE_RW | PAGE_USER) == 0) {
#if LINUX_MODULE_MAP_TRACE
                linux_module_map_note_mmap((int32_t)fd,
                                           (uint64_t)(uintptr_t)reserved,
                                           length, offset);
#endif
                if (saved_offset >= 0) {
                    (void)syscall_file_seek((int32_t)fd, saved_offset,
                                            LINUX_SEEK_SET);
                }
                return (int64_t)(uintptr_t)reserved;
            }
            /* Table full or no address space: fall through to the eager path
             * rather than failing the mmap outright. */
            syscall_file_mmap_release(handle);
        }
    }

    void *mapped;
    if ((flags & LINUX_MAP_FIXED) != 0u) {
        if (addr == 0u || addr < 0x1000u || length > 0x4000000000ULL - addr) {
            return LINUX_EINVAL;
        }
        uint64_t cr3 = process_get_current_cr3();
        if (cr3 == 0u ||
            paging_map_user_range_alloc(cr3, addr, length,
                                        PAGE_RW | PAGE_USER) < 0) {
            return LINUX_ENOMEM;
        }
        mapped = (void *)(uintptr_t)addr;
    } else {
        mapped = process_user_mmap(length, flags);
        if (mapped == NULL) {
            return LINUX_ENOMEM;
        }
    }

    uint64_t total = 0;
    uint8_t chunk[4096];
    while (total < read_len) {
        uint64_t want = read_len - total;
        if (want > sizeof(chunk)) want = sizeof(chunk);
        int64_t count = syscall_file_read((int32_t)fd, chunk, want);
        if (count <= 0) break;
        if (copy_to_user_trusted((uint8_t *)(uintptr_t)mapped + total,
                                 chunk, (uint64_t)count) != 0u) {
            if (saved_offset >= 0) {
                (void)syscall_file_seek((int32_t)fd, saved_offset,
                                        LINUX_SEEK_SET);
            }
            return LINUX_EFAULT;
        }
        total += (uint64_t)count;
    }
    if (saved_offset >= 0) {
        (void)syscall_file_seek((int32_t)fd, saved_offset, LINUX_SEEK_SET);
    }

    /* A writable MAP_SHARED file mapping: remember it so its contents are
     * flushed back to the file on msync()/munmap()/exit. */
    if ((flags & LINUX_MAP_SHARED) != 0u) {
        int32_t self = process_get_current_pid();
        if (self >= 0) {
            linux_mshared_register(self, (int32_t)fd,
                                   (uint64_t)(uintptr_t)mapped, length,
                                   offset, read_len,
                                   (prot & 0x2u /* PROT_WRITE */) != 0u);
        }
    }
#if LINUX_MODULE_MAP_TRACE
    /* Also on the eager path: an X input/video driver is well under the
     * lazy-mapping threshold, so it never reaches the branch above. */
    linux_module_map_note_mmap((int32_t)fd, (uint64_t)(uintptr_t)mapped,
                               length, offset);
#endif
    return (int64_t)(uintptr_t)mapped;
}

static int64_t linux_epoll_ctl(uint64_t epfd, uint64_t op, uint64_t fd,
                               uint64_t event_ptr)
{
    if (event_ptr == 0u &&
        (op == 1u || op == 3u)) {
        return LINUX_EFAULT;
    }
    if (event_ptr != 0u &&
        !process_user_buffer_is_valid((const void *)(uintptr_t)event_ptr,
                                      sizeof(epoll_event_t))) {
        return LINUX_EFAULT;
    }
    return (int64_t)syscall_epoll_ctl((int32_t)epfd, (int32_t)op,
                                      (int32_t)fd,
                                      (const epoll_event_t *)(uintptr_t)event_ptr);
}

static int64_t linux_epoll_wait(uint64_t epfd, uint64_t events,
                                uint64_t maxevents, uint64_t timeout_ms,
                                int *should_switch_out, int *restart_out)
{
    if (maxevents == 0u || maxevents > 4096u) {
        return LINUX_EINVAL;
    }
    if (!process_user_buffer_is_valid((const void *)(uintptr_t)events,
                                      maxevents * sizeof(epoll_event_t))) {
        return LINUX_EFAULT;
    }
    int64_t rc = (int64_t)syscall_epoll_wait_ex((int32_t)epfd,
                                                (epoll_event_t *)(uintptr_t)events,
                                                (int32_t)maxevents,
                                                (int32_t)timeout_ms,
                                                should_switch_out);
    /* A wait with no deadline may not report a timeout. syscall_epoll_wait_ex()
     * degrades "block" to "sleep a slice and report nothing ready", which is a
     * legal spurious wakeup for a finite timeout but a lie for timeout < 0 --
     * and the X server's WaitForSomething() passes -1 whenever no timer is
     * pending, so it spun a whole CPU re-entering epoll_wait forever. Run the
     * call again instead (same mechanism as linux_wait4). */
    if (rc == 0 && (int32_t)timeout_ms < 0 && restart_out != NULL) {
        *restart_out = 1;
    }
    return rc;
}

/* poll(2) / ppoll(2).
 *
 * glibc's GMainContext (and much other Linux code) drives its event loop
 * with ppoll(); returning -ENOSYS made GLib fall back to a busy loop that
 * printed "poll(2) failed: Function not implemented" on every iteration and
 * pinned the CPU. This does one non-blocking readiness pass over the pollfd
 * set (reusing the same per-fd probe epoll uses). If nothing is ready and a
 * non-zero timeout was requested it parks the caller for a short slice and
 * returns 0 - the same "blocking degrades to a timed poll" compromise
 * syscall_epoll_wait_ex() uses, since this path cannot block internally: see
 * the long comment at the top of Syscall_Epoll.c for why a park-and-rescan
 * loop in here spins instead of sleeping. */
#define LINUX_POLLIN   0x0001
#define LINUX_POLLPRI  0x0002
#define LINUX_POLLOUT  0x0004
#define LINUX_POLLERR  0x0008
#define LINUX_POLLHUP  0x0010
#define LINUX_POLLNVAL 0x0020
#define LINUX_POLL_MAX_FDS  256u
#define LINUX_POLL_SLICE_MS 1u

typedef struct {
    int32_t fd;
    int16_t events;
    int16_t revents;
} linux_pollfd_t;

#ifndef PROCESS_STALL_DUMP
#define PROCESS_STALL_DUMP 0
#endif
#if PROCESS_STALL_DUMP
/* The fd set a thread was last left waiting on, for the stall dump.
 *
 * "Everything is idle and nobody is making progress" is not diagnosable from
 * syscall counts: the question is always which descriptor a thread is waiting
 * on and whether the kernel thinks it is ready. Recorded only on the path
 * where the wait gave up with nothing ready, which is exactly the stalled
 * case. */
#define LX_POLLTRACE_MAX_FDS 8u
#define LX_POLLTRACE_MAX_PID 64u
static uint8_t  g_lx_poll_n[LX_POLLTRACE_MAX_PID];
static int32_t  g_lx_poll_fd[LX_POLLTRACE_MAX_PID][LX_POLLTRACE_MAX_FDS];
static uint16_t g_lx_poll_ev[LX_POLLTRACE_MAX_PID][LX_POLLTRACE_MAX_FDS];

static void linux_poll_trace_note(const linux_pollfd_t *pfds, uint64_t nfds)
{
    int32_t tid = process_get_current_tid();
    if (tid < 0 || (uint32_t)tid >= LX_POLLTRACE_MAX_PID) {
        return;
    }
    uint32_t n = (nfds > LX_POLLTRACE_MAX_FDS) ? LX_POLLTRACE_MAX_FDS
                                               : (uint32_t)nfds;
    for (uint32_t i = 0; i < n; ++i) {
        g_lx_poll_fd[tid][i] = pfds[i].fd;
        g_lx_poll_ev[tid][i] = (uint16_t)pfds[i].events;
    }
    g_lx_poll_n[tid] = (uint8_t)n;
}

void linux_poll_trace_dump_all(void);
void linux_poll_trace_dump_all(void)
{
    for (uint32_t tid = 0; tid < LX_POLLTRACE_MAX_PID; ++tid) {
        if (g_lx_poll_n[tid] == 0u) {
            continue;
        }
        serial_write_string("[poll] ");
        serial_write_uint32(tid);
        for (uint32_t i = 0; i < g_lx_poll_n[tid]; ++i) {
            int32_t fd = g_lx_poll_fd[tid][i];
            serial_write_string(" ");
            serial_write_uint32((uint32_t)fd);
            serial_write_string("/w");
            serial_write_uint32(g_lx_poll_ev[tid][i]);
            serial_write_string("/r");
            serial_write_uint32(syscall_poll_one_fd(fd, 0x1u | 0x4u));
        }
        serial_write_char('\n');
    }
}
#endif

static int64_t linux_poll_common(uint64_t fds_ptr, uint64_t nfds,
                                 int64_t timeout_ms, int *should_switch_out,
                                 int *restart_out)
{
    /* Same rule as linux_epoll_wait(): a negative timeout means "no deadline",
     * so reporting a timeout is not an option. Set at the points below that
     * would otherwise return 0. */
    if (nfds > LINUX_POLL_MAX_FDS) {
        return LINUX_EINVAL;
    }

    uint32_t slice_ms = LINUX_POLL_SLICE_MS;
    if (timeout_ms > 0 && (uint64_t)timeout_ms < slice_ms) {
        slice_ms = (uint32_t)timeout_ms;
    }

    /* Taken before the readiness scan below so an event that lands during the
     * scan cancels the sleep instead of being missed -- see Poll_Wait.h. */
    uint64_t generation = poll_wait_generation();

    if (nfds == 0u) {
        if (timeout_ms != 0 &&
            poll_wait_park(generation, slice_ms) != 0 &&
            should_switch_out != NULL) {
            *should_switch_out = 1;
        }
        if (timeout_ms < 0 && restart_out != NULL) {
            *restart_out = 1;
        }
        return 0;
    }

    uint64_t bytes = nfds * sizeof(linux_pollfd_t);
    if (fds_ptr == 0u ||
        !process_user_buffer_is_valid((void *)(uintptr_t)fds_ptr, bytes)) {
        return LINUX_EFAULT;
    }

    linux_pollfd_t pfds[LINUX_POLL_MAX_FDS];
    if (copy_from_user(pfds, (const void *)(uintptr_t)fds_ptr, bytes) != 0u) {
        return LINUX_EFAULT;
    }

    int64_t ready_count = 0;
    for (uint64_t i = 0; i < nfds; ++i) {
        pfds[i].revents = 0;
        if (pfds[i].fd < 0) {
            continue;
        }
        uint32_t want = 0u;
        if ((pfds[i].events & LINUX_POLLIN) != 0)  want |= 0x1u;
        if ((pfds[i].events & LINUX_POLLOUT) != 0) want |= 0x4u;
        /* The caller's own descriptor numbers (Linux_FdTable.c). */
        int32_t global = lxfd_get(pfds[i].fd);
        uint32_t r = (global < 0) ? 0x20u /* POLLNVAL */
                                  : syscall_poll_one_fd(global,
                                        want != 0u ? want : (0x1u | 0x4u));
        uint16_t rev = 0;
        if ((r & 0x1u) != 0u)  rev |= LINUX_POLLIN;
        if ((r & 0x4u) != 0u)  rev |= LINUX_POLLOUT;
        if ((r & 0x8u) != 0u)  rev |= LINUX_POLLERR;
        if ((r & 0x10u) != 0u) rev |= LINUX_POLLHUP;
        if ((r & 0x20u) != 0u) rev |= LINUX_POLLNVAL;
        rev &= (uint16_t)(pfds[i].events |
                          LINUX_POLLERR | LINUX_POLLHUP | LINUX_POLLNVAL);
        if (rev != 0) {
            pfds[i].revents = (int16_t)rev;
            ++ready_count;
        }
    }

    /* Write revents back regardless (Linux updates the array on timeout too). */
    if (copy_to_user((void *)(uintptr_t)fds_ptr, pfds, bytes) != 0u) {
        return LINUX_EFAULT;
    }
    if (ready_count > 0 || timeout_ms == 0) {
        return ready_count;
    }
#if PROCESS_STALL_DUMP
    linux_poll_trace_note(pfds, nfds);
#endif
    if (poll_wait_park(generation, slice_ms) != 0 && should_switch_out != NULL) {
        *should_switch_out = 1;
    }
    if (timeout_ms < 0 && restart_out != NULL) {
        *restart_out = 1;
    }
    return 0;
}

static int64_t linux_ppoll(uint64_t fds_ptr, uint64_t nfds, uint64_t tmo_ptr,
                           int *should_switch_out, int *restart_out)
{
    int64_t timeout_ms = -1; /* NULL timespec -> block (degrades to a slice) */
    if (tmo_ptr != 0u) {
        struct { int64_t sec; int64_t nsec; } ts;
        if (copy_from_user(&ts, (const void *)(uintptr_t)tmo_ptr,
                           sizeof(ts)) != 0u) {
            return LINUX_EFAULT;
        }
        timeout_ms = (ts.sec >= 0 && ts.nsec >= 0)
                         ? (ts.sec * 1000 + (ts.nsec + 999999) / 1000000)
                         : 0;
    }
    return linux_poll_common(fds_ptr, nfds, timeout_ms, should_switch_out,
                             restart_out);
}

static int64_t linux_wait4(uint64_t pid, uint64_t status_ptr,
                           uint64_t options, uint64_t rusage,
                           int *should_switch_out, int *restart_out)
{
    (void)rusage;
    /* __WNOTHREAD (0x20000000), __WALL (0x40000000), __WCLONE (0x80000000)
     * select which children are eligible; every child here is an ordinary
     * one, so they change nothing. Chromium waits for its clone() helpers
     * with __WALL. */
    options &= ~(uint64_t)0xE0000000u;
    if ((options & ~(LINUX_WNOHANG | 2u)) != 0u) {
        return LINUX_EINVAL;
    }
    if (status_ptr != 0u &&
        !process_user_buffer_is_valid((const void *)(uintptr_t)status_ptr,
                                      sizeof(int32_t))) {
        return LINUX_EFAULT;
    }
    int32_t child_code = 0;
    int32_t term_signal = 0;
    int32_t result = process_waitpid_ex((int32_t)pid, &child_code,
                                        (int32_t)options, &term_signal);
    if (result == 0 && (options & LINUX_WNOHANG) == 0u) {
        /* A child exists but has not exited. Park the caller for a slice, then
         * run the wait again from the top when it is next scheduled.
         *
         * Returning 0 here (what this used to do) is not a thing a blocking
         * wait4() may do -- on Linux 0 means "WNOHANG and nothing to report",
         * so callers read it as "the child is still running" and stop waiting.
         * The X server's Popen()/Pclose() is exactly that shape:
         *     do { pid = waitpid(cur->pid, &pstat, 0); }
         *     while (pid == -1 && errno == EINTR);
         *     return pid == -1 ? -1 : pstat;
         * With 0 the loop falls straight through and returns an UNINITIALISED
         * pstat, so xkbcomp was reported as having succeeded while it was in
         * fact still running -- and the keymap it had not written yet came
         * back as "Couldn't open compiled keymap file", which is fatal. */
        if (process_sleep_current_ms(LINUX_WAIT_POLL_SLICE_MS) == 0 &&
            should_switch_out != NULL) {
            *should_switch_out = 1;
        }
        if (restart_out != NULL) {
            *restart_out = 1;
        }
        return 0;
    }
    if (result < 0) {
        /* No such child. -1 would surface as EPERM. */
        return LINUX_ECHILD;
    }
    if (result > 0 && status_ptr != 0u) {
        /* Encode a POSIX wait status: killed-by-signal -> low 7 bits carry the
         * signal number (WIFSIGNALED/WTERMSIG); normal exit -> exit code in
         * bits 8..15 with the low byte zero (WIFEXITED/WEXITSTATUS). */
        int32_t wstatus = (term_signal != 0)
                              ? (term_signal & 0x7f)
                              : ((child_code & 0xff) << 8);
        if (copy_to_user_trusted((void *)(uintptr_t)status_ptr,
                                 &wstatus, sizeof(wstatus)) != 0u) {
            return LINUX_EFAULT;
        }
    }
    return (int64_t)result;
}

static int64_t linux_rt_sigaction(uint64_t signum, uint64_t act,
                                  uint64_t oldact, uint64_t sigsetsize)
{
    return syscall_rt_sigaction(signum, act, oldact, sigsetsize);
}

static int64_t linux_rt_sigprocmask(uint64_t how, uint64_t set,
                                    uint64_t oldset, uint64_t sigsetsize)
{
    return syscall_rt_sigprocmask(how, set, oldset, sigsetsize);
}

typedef struct {
    uint64_t ss_sp;
    int32_t ss_flags;
    int32_t __pad;
    uint64_t ss_size;
} linux_stack_arg_t;

static int64_t linux_sigaltstack(uint64_t new_ss_ptr, uint64_t old_ss_ptr)
{
    linux_stack_arg_t new_ss = {0};
    int has_new = (new_ss_ptr != 0u);
    if (has_new &&
        copy_from_user(&new_ss, (const void *)(uintptr_t)new_ss_ptr,
                       sizeof(new_ss)) != 0u) {
        return LINUX_EFAULT;
    }
    uint64_t old_sp = 0, old_size = 0;
    uint32_t old_flags = 0;
    if (process_sigaltstack(new_ss.ss_sp, new_ss.ss_size,
                            (uint32_t)new_ss.ss_flags, has_new,
                            &old_sp, &old_size, &old_flags) < 0) {
        return LINUX_EINVAL;
    }
    if (old_ss_ptr != 0u) {
        linux_stack_arg_t old_ss = {0};
        old_ss.ss_sp = old_sp;
        old_ss.ss_flags = (int32_t)old_flags;
        old_ss.ss_size = old_size;
        if (copy_to_user((void *)(uintptr_t)old_ss_ptr, &old_ss,
                         sizeof(old_ss)) != 0u) {
            return LINUX_EFAULT;
        }
    }
    return 0;
}

static int64_t linux_gettimeofday(uint64_t tv_ptr)
{
    if (tv_ptr == 0u) {
        return 0;
    }
    if (!process_user_buffer_is_valid((const void *)(uintptr_t)tv_ptr,
                                      sizeof(int64_t) * 2u)) {
        return LINUX_EFAULT;
    }
    int64_t now_ns = clock_realtime_ns();
    int64_t tv[2];
    tv[0] = now_ns / 1000000000LL;
    tv[1] = (now_ns % 1000000000LL) / 1000LL;
    if (copy_to_user_trusted((void *)(uintptr_t)tv_ptr, tv, sizeof(tv)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t linux_uname(uint64_t uts_ptr)
{
    const uint64_t uts_size = 6u * LINUX_UTSNAME_LEN;
    if (!process_user_buffer_is_valid((const void *)(uintptr_t)uts_ptr,
                                      uts_size)) {
        return LINUX_EFAULT;
    }
    char uts[6 * LINUX_UTSNAME_LEN];
    memset(uts, 0, sizeof(uts));
    memcpy(uts + 0u, "Linux", 5u);
    memcpy(uts + 1u * LINUX_UTSNAME_LEN, "implus", 6u);
    memcpy(uts + 2u * LINUX_UTSNAME_LEN, "6.1.0-implus", 12u);
    memcpy(uts + 3u * LINUX_UTSNAME_LEN, "#1 SMP ImplusOS", 15u);
    memcpy(uts + 4u * LINUX_UTSNAME_LEN, "x86_64", 6u);
    if (copy_to_user_trusted((void *)(uintptr_t)uts_ptr, uts, sizeof(uts)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t linux_getcpu(uint64_t cpu_ptr, uint64_t node_ptr,
                            uint64_t tcache_ptr)
{
    (void)tcache_ptr;
    uint32_t zero = 0;
    if (cpu_ptr != 0u &&
        copy_to_user_trusted((void *)(uintptr_t)cpu_ptr, &zero, sizeof(zero)) != 0u) {
        return LINUX_EFAULT;
    }
    if (node_ptr != 0u &&
        copy_to_user_trusted((void *)(uintptr_t)node_ptr, &zero, sizeof(zero)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

/*
 * Process-lifecycle trace.
 *
 * fork/exec/exit are rare enough that naming each one costs nothing, and
 * every multi-process question -- which slot a child landed in, whether it
 * exec'd or exited, in what order -- is unanswerable without it.
 *
 * Each line is assembled in a local buffer and handed to the serial port in
 * one call. serial_write_string() is not atomic against another CPU writing
 * at the same time, and a Chromium boot has four CPUs logging at once: a
 * line emitted field-by-field comes back interleaved character-by-character
 * with someone else's, which is exactly the state in which fork traces stop
 * being readable.
 */
typedef struct {
    char buf[224];
    uint32_t len;
} lx_trace_line_t;

static void lx_trace_str(lx_trace_line_t *line, const char *text)
{
    while (*text != '\0' && line->len + 1u < sizeof(line->buf)) {
        line->buf[line->len++] = *text++;
    }
    line->buf[line->len] = '\0';
}

static void lx_trace_hex(lx_trace_line_t *line, uint64_t value)
{
    char digits[17];
    int i = 16;
    digits[16] = '\0';
    if (value == 0u) {
        lx_trace_str(line, "0");
        return;
    }
    while (value != 0u && i > 0) {
        uint8_t nibble = (uint8_t)(value & 0xFu);
        digits[--i] = (char)(nibble < 10u ? (uint8_t)('0' + nibble)
                                          : (uint8_t)('a' + (nibble - 10u)));
        value >>= 4;
    }
    lx_trace_str(line, "0x");
    lx_trace_str(line, &digits[i]);
}

static void lx_trace_dec(lx_trace_line_t *line, uint64_t value)
{
    char digits[21];
    int i = 20;
    digits[20] = '\0';
    if (value == 0u) {
        lx_trace_str(line, "0");
        return;
    }
    while (value != 0u && i > 0) {
        digits[--i] = (char)('0' + (uint8_t)(value % 10u));
        value /= 10u;
    }
    lx_trace_str(line, &digits[i]);
}

static void lx_trace_emit(lx_trace_line_t *line)
{
    lx_trace_str(line, "\n");
    serial_write_string(line->buf);
}

__attribute__((unused))
static void lx_proc_trace(const char *what, uint64_t a, uint64_t b)
{
    lx_trace_line_t line = { {0}, 0u };
    lx_trace_str(&line, "[lxproc] ");
    lx_trace_str(&line, what);
    lx_trace_str(&line, " pid=");
    lx_trace_dec(&line, (uint64_t)(uint32_t)process_get_current_pid());
    lx_trace_str(&line, " tid=");
    lx_trace_dec(&line, (uint64_t)(uint32_t)process_get_current_tid());
    lx_trace_str(&line, " a=");
    lx_trace_hex(&line, a);
    lx_trace_str(&line, " b=");
    lx_trace_hex(&line, b);
    lx_trace_emit(&line);
}

/* Off by default: one line per fork/exec/exit plus a prefix of every new
 * process's syscalls is exactly what a multi-process bring-up needs and
 * exactly what nobody wants on a normal boot. Build with
 * -DLINUX_PROC_TRACE=1 to turn it on. */
#ifndef LINUX_PROC_TRACE
#define LINUX_PROC_TRACE 0
#endif

#if LINUX_PROC_TRACE
#define LX_PROC_TRACE(what, a, b) lx_proc_trace((what), (uint64_t)(a), (uint64_t)(b))
#else
#define LX_PROC_TRACE(what, a, b) ((void)0)
#endif

/*
 * The first syscalls a forked child makes.
 *
 * A child that gets its resume state wrong says nothing about it: it simply
 * does something other than what the program's child branch does, and the
 * only visible consequence is the parent reporting a failure much later.
 * Recording a short prefix of each newborn's syscalls -- the child branch of
 * fork() is only a few dozen calls before execve() -- makes that first
 * divergence legible, and costs nothing for every other process.
 */
#ifndef LINUX_NEWBORN_TRACE
#define LINUX_NEWBORN_TRACE 160u
#endif
static uint8_t g_lx_newborn_left[OS_CONFIG_PROCESS_MAX_COUNT];

static void lx_newborn_arm(int32_t pid, uint32_t calls)
{
    if (!LINUX_PROC_TRACE) {
        return;
    }
    if (pid >= 0 && pid < (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) {
        if (calls > 255u) {
            calls = 255u;
        }
        g_lx_newborn_left[pid] = (uint8_t)calls;
    }
}

/* Logged at syscall *exit* so the result is in the line: "which call went
 * wrong" is the question, and an argument list without a return value cannot
 * answer it. */
static void lx_newborn_note(uint64_t num, uint64_t a1, uint64_t a2,
                            int64_t result)
{
    if (!LINUX_PROC_TRACE) {
        return;
    }
    int32_t pid = process_get_current_pid();
    if (pid < 0 || pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT ||
        g_lx_newborn_left[pid] == 0u) {
        return;
    }
    --g_lx_newborn_left[pid];
    lx_trace_line_t line = { {0}, 0u };
    lx_trace_str(&line, "[lxnew] pid=");
    lx_trace_dec(&line, (uint64_t)(uint32_t)pid);
    lx_trace_str(&line, " nr=");
    lx_trace_dec(&line, num);
    lx_trace_str(&line, " a1=");
    lx_trace_hex(&line, a1);
    lx_trace_str(&line, " a2=");
    lx_trace_hex(&line, a2);
    lx_trace_str(&line, " -> ");
    if (result < 0) {
        lx_trace_str(&line, "-");
        lx_trace_dec(&line, (uint64_t)(-result));
    } else {
        lx_trace_hex(&line, (uint64_t)result);
    }
    lx_trace_emit(&line);
}

static int64_t linux_clone(uint64_t saved_rsp, uint64_t flags, uint64_t stack,
                           uint64_t parent_tid, uint64_t child_tid,
                           uint64_t tls, int *should_switch)
{
    /* fork()/vfork() reach the kernel as clone(), not as SYS_fork: glibc's
     * arch_fork() issues
     *   clone(CLONE_CHILD_SETTID|CLONE_CHILD_CLEARTID|SIGCHLD, 0, NULL, ctid, 0)
     * -- no CLONE_VM, and a NULL stack because the child keeps the parent's.
     * Rejecting stack==0 outright therefore failed *every* fork() from a
     * Linux binary with EINVAL. That is what stopped Xorg from compiling a
     * keymap: its only route to xkbcomp is Popen(), i.e. fork() +
     * execl("/bin/sh", ...), and the failure surfaced only as the opaque
     * "XKB: Could not invoke xkbcomp" before being fatal. */
    /*
     * Anything without CLONE_THREAD is a new *process*, however much of the
     * caller it is asked to share.
     *
     * CLONE_VM used to be routed to the thread path, and that is wrong for
     * the one caller that matters: vfork(2) and glibc's posix_spawn() issue
     * clone(CLONE_VM|CLONE_VFORK|SIGCHLD, <stack>), and Crashpad's
     * SpawnSubprocess() reaches exec through exactly that. Treating it as a
     * thread ran the child branch -- close the descriptors, execve the
     * handler -- inside the *caller's* process, so the caller's following
     * _exit(EXIT_SUCCESS) tore down the thread that was about to exec, and
     * Chromium reported "Check failed: client.StartHandler(...)".
     *
     * The address space is copied rather than shared. Real CLONE_VM would
     * have the child write into the parent's memory, which is how
     * posix_spawn() reports an exec failure back through its shared
     * `args.err`; with a copy the parent reads the initial 0 and reports
     * success, and the failure surfaces one step later as the child exiting
     * 127. Everything else these callers do between clone and execve touches
     * only their own stack.
     */
    if ((flags & LINUX_CLONE_THREAD) == 0u) {
        LX_PROC_TRACE("fork-enter", flags, stack);
        uint32_t fork_opts = 0u;
        if ((flags & 0x20000000u) != 0u) fork_opts |= PROCESS_FORK_NEWPID;
        if ((flags & LINUX_CLONE_FS) != 0u) fork_opts |= PROCESS_FORK_SHARE_FS;
        int32_t child_pid = process_fork_ex(stack, fork_opts);
        if (child_pid < 0) {
            return LINUX_EAGAIN;
        }
        /* clone() with an explicit stack: the child resumed at the same
         * instruction as the parent but on the stack the caller prepared,
         * where glibc's wrapper has already pushed fn and arg -- installed
         * by process_fork_with_stack() before the child could run. */
        /* CLONE_FS (one filesystem root between the two -- Chromium's
         * sandbox chroots from a CLONE_FS helper) and CLONE_NEWPID (the
         * child is a namespace init and sees itself as pid 1) were applied
         * inside process_fork_ex(), before the child could run. */
        {
            /* fork() is rare and every multi-process bring-up question starts
             * here, so this line is unconditional. It names the slot the child
             * landed in and the user-mode state it will resume with -- a child
             * that comes back at RIP 0 is the signature of a frame that was
             * copied from the wrong place. */
            const uint64_t *f = (const uint64_t *)(uintptr_t)saved_rsp;
            lx_trace_line_t line = { {0}, 0u };
            if (!LINUX_PROC_TRACE) {
                goto arm_newborn;
            }
            lx_trace_str(&line, "[lxfork] pid=");
            lx_trace_dec(&line, (uint64_t)(uint32_t)process_get_current_pid());
            lx_trace_str(&line, " child=");
            lx_trace_dec(&line, (uint64_t)(uint32_t)child_pid);
            lx_trace_str(&line, " rip=");
            lx_trace_hex(&line, f[SYSCALL_FRAME_RCX]);
            lx_trace_str(&line, " ursp=");
            lx_trace_hex(&line, process_get_current_user_rsp());
            lx_trace_str(&line, " flags=");
            lx_trace_hex(&line, flags);
            lx_trace_emit(&line);
        arm_newborn:
            /* Both sides: the child's divergence and the parent's handling of
             * it (wait4, the socket handshake) are the same story. */
            lx_newborn_arm(child_pid, LINUX_NEWBORN_TRACE);
            lx_newborn_arm(process_get_current_pid(), LINUX_NEWBORN_TRACE);
        }
        if ((flags & LINUX_CLONE_PARENT_SETTID) != 0u && parent_tid != 0u) {
            int32_t pid32 = child_pid;
            (void)copy_to_user_trusted((void *)(uintptr_t)parent_tid,
                                       &pid32, sizeof(pid32));
        }
        /* CLONE_CHILD_SETTID / CLONE_CHILD_CLEARTID are deliberately NOT
         * honoured on this path. Both write through `child_tid` in the
         * *child's* address space, and process_fork() returns with us still
         * running as the parent -- writing here would clobber the parent's own
         * TCB instead. glibc only uses them to refresh THREAD_SELF->tid, which
         * the child does not consult before it execs (and execve clears both
         * on Linux too). Doing it properly needs a cross-address-space write;
         * see Docs/Others/TODO_Doom_Xorg_MethodA.md M9. */
        if (should_switch != NULL) {
            *should_switch = 1;
        }
        return (int64_t)child_pid;
    }

    /* CLONE_THREAD: a real thread in the caller's address space. glibc always
     * allocates the new thread's stack itself, so a NULL one is a malformed
     * request rather than "share mine". */
    if (stack == 0u) {
        return LINUX_EINVAL;
    }
    uint64_t *frame = (uint64_t *)(uintptr_t)saved_rsp;
    uint64_t return_rip = frame[SYSCALL_FRAME_RCX];
    if (return_rip < 0x1000u) {
        return LINUX_EFAULT;
    }
    int has_tls = (flags & LINUX_CLONE_SETTLS) != 0u;
    /* Pass `stack` as the child's user RSP through the create call so it is
     * installed before the thread is schedulable. Setting it afterwards
     * (process_set_thread_user_rsp) raced the SMP scheduler: the child could
     * start on the raw kernel-picked stack, read a 0 fn pointer and #PF at
     * RIP=0. glibc's __clone already prepared this stack. */
    int32_t tid = process_create_thread_ex(return_rip, flags, stack,
                                           parent_tid, child_tid,
                                           has_tls, tls, stack);
    if (tid < 0) {
        /* EAGAIN is the errno glibc/Chromium expect for "couldn't spawn a
         * thread" (resource exhaustion); -1/EPERM sent them down a fatal
         * path. */
        serial_write_string("[lx] clone/thread create failed\n");
        return LINUX_EAGAIN;
    }
#if LINUX_CLONE_TRACE
    /* What glibc handed us for the new thread: its initial user RSP (which is
     * the top of the stack block glibc allocated and recorded in the TCB) and
     * its TLS base. V8 asks glibc for the running thread's stack bounds and
     * CHECKs that its own stack pointer lies inside them
     * (Isolate::IsOnCentralStack); when that fails, the question is whether
     * the stack is where glibc thinks it is, so print both and compare
     * against the RSP in the crash report. */
    serial_write_string("[clone] tid=");
    serial_write_uint32((uint32_t)tid);
    serial_write_string(" stack=");
    serial_write_uint64(stack);
    serial_write_string(" tls=");
    serial_write_uint64(has_tls ? tls : 0u);
    serial_write_string(" flags=");
    serial_write_uint64(flags);
    serial_write_char('\n');
#endif
    if ((flags & LINUX_CLONE_PARENT_SETTID) != 0u && parent_tid != 0u) {
        int32_t tid32 = tid;
        if (copy_to_user_trusted((void *)(uintptr_t)parent_tid,
                                 &tid32, sizeof(tid32)) != 0u) {
            (void)process_terminate(tid);
            return LINUX_EFAULT;
        }
    }
    if ((flags & LINUX_CLONE_CHILD_SETTID) != 0u && child_tid != 0u) {
        int32_t tid32 = tid;
        if (copy_to_user_trusted((void *)(uintptr_t)child_tid,
                                 &tid32, sizeof(tid32)) != 0u) {
            (void)process_terminate(tid);
            return LINUX_EFAULT;
        }
    }
    if ((flags & LINUX_CLONE_CHILD_CLEARTID) != 0u && child_tid != 0u) {
        /* On the new thread, not on us: see process_set_clear_child_tid_for(). */
        (void)process_set_clear_child_tid_for(tid, child_tid);
    }
    return (int64_t)tid;
}

static int64_t linux_socket_recvfrom(uint64_t fd, uint64_t buf, uint64_t len,
                                     uint64_t flags, uint64_t addr_ptr,
                                     uint64_t addr_len_ptr);

static int64_t linux_read(uint64_t fd, uint64_t buf, uint64_t count)
{
    if (count == 0u) {
        return 0;
    }
    if (count > LINUX_MAX_IO_BYTES) {
        count = LINUX_MAX_IO_BYTES;
    }
    /* read(2) writes into `buf`: a read-only destination is EFAULT, and
     * Chromium's protected-memory self-check depends on it being so. */
    if (!process_user_buffer_is_writable((const void *)(uintptr_t)buf, count)) {
        return LINUX_EFAULT;
    }
    if (syscall_eventfd_is_valid((int32_t)fd)) {
        uint8_t staged[8];
        if (count < sizeof(staged)) {
            return LINUX_EINVAL;
        }
        int64_t rc = syscall_eventfd_read((int32_t)fd, staged, sizeof(staged));
        if (rc < 0) {
            return rc;
        }
        if (copy_to_user_trusted((void *)(uintptr_t)buf, staged, sizeof(staged)) != 0u) {
            return LINUX_EFAULT;
        }
        return rc;
    }
    /* AF_UNIX stream read() (X's _XSERVTransRead). recvmsg is routed
     * elsewhere; plain read() previously fell through to the file table and
     * failed. */
    if (unix_socket_fd_in_range((int32_t)fd)) {
        return unix_socket_recv((int32_t)fd, (void *)(uintptr_t)buf, count);
    }
    /* AF_INET read(): socket fds live outside the file table, so this used to
     * fail with EINVAL. Chromium reads DNS replies from connected UDP sockets
     * and HTTP responses from TCP sockets with plain read(), so every reply
     * that arrived was dropped and the page failed with
     * DNS_PROBE_FINISHED_NO_INTERNET while the answers sat in the queue. */
    if (syscall_socket_fd_in_range((int32_t)fd)) {
        return linux_socket_recvfrom(fd, buf, count, 0u, 0u, 0u);
    }
    return (int64_t)syscall_file_read((int32_t)fd,
                                      (uint8_t *)(uintptr_t)buf, count);
}

/* Resolve `path` the way the *at() syscalls do: absolute paths are taken as
 * they are, AT_FDCWD resolves against the process cwd, and anything else
 * resolves against the directory `dirfd` was opened on. Every *at() call used
 * to answer ENOTSUP for a real dirfd, which is a gap ordinary Linux code walks
 * into constantly -- base::DeleteFile() in Chromium opens a directory and then
 * unlinkat()s its entries, and openat(dirfd, name) is how glibc's ftw/nftw and
 * fts walk a tree. Declared before linux_resolve_path() defines it. */
static int64_t linux_resolve_path(char *path, uint64_t capacity);

/*
 * Rewrite a user-supplied absolute path to sit under the process's chroot.
 *
 * Paths that come from the cwd or from a directory descriptor are already
 * real kernel paths (both are stored as resolved), so only a path the caller
 * wrote as absolute needs this. Chromium's namespace sandbox chroots to an
 * empty directory and then proves it worked by checking that /proc is gone,
 * so a chroot that does not actually redirect lookups is worse than none:
 * the check passes, the sandbox believes it is confined, and it is not.
 */
static int64_t linux_apply_root(char *path, uint64_t capacity)
{
    char root[256];
    int root_len = process_get_current_root(root, sizeof(root));
    if (root_len <= 0 || root[0] == '\0') {
        return 0; /* the ordinary case: no chroot in force */
    }
    uint64_t path_len = strlen(path);
    /* "/" alone becomes the root itself, with no trailing slash. */
    if (path_len == 1u) {
        if ((uint64_t)root_len + 1u > capacity) {
            return LINUX_ENAMETOOLONG;
        }
        memcpy(path, root, (uint64_t)root_len + 1u);
        return 0;
    }
    if ((uint64_t)root_len + path_len + 1u > capacity) {
        return LINUX_ENAMETOOLONG;
    }
    memmove(path + root_len, path, path_len + 1u);
    memcpy(path, root, (uint64_t)root_len);
    return 0;
}

static int64_t linux_resolve_at(uint64_t dirfd, char *path, uint64_t capacity)
{
    if (path[0] == '/') {
        return linux_apply_root(path, capacity);
    }
    if ((int64_t)dirfd == LINUX_AT_FDCWD) {
        return linux_resolve_path(path, capacity);
    }
    char dir[256];
    if (syscall_file_get_dir_path((int32_t)dirfd, dir, sizeof(dir)) != 0) {
        return LINUX_EBADF;
    }
    uint64_t dir_len = strlen(dir);
    while (dir_len > 1u && dir[dir_len - 1u] == '/') {
        dir[--dir_len] = '\0';
    }
    uint64_t path_len = strlen(path);
    /* "." means the directory itself. */
    if (path_len == 1u && path[0] == '.') {
        if (dir_len + 1u > capacity) {
            return LINUX_ENAMETOOLONG;
        }
        memcpy(path, dir, dir_len + 1u);
        return 0;
    }
    if (dir_len + 1u + path_len + 1u > capacity) {
        return LINUX_ENAMETOOLONG;
    }
    memmove(path + dir_len + 1u, path, path_len + 1u);
    memcpy(path, dir, dir_len);
    path[dir_len] = '/';
    return 0;
}

static int64_t linux_resolve_path(char *path, uint64_t capacity)
{
    if (path[0] == '/') {
        return linux_apply_root(path, capacity);
    }
    char cwd[256];
    if (process_get_current_cwd(cwd, sizeof(cwd)) != 0) {
        cwd[0] = '/';
        cwd[1] = '\0';
    }
    uint64_t cwd_len = strlen(cwd);
    while (cwd_len > 1u && cwd[cwd_len - 1u] == '/') {
        cwd[cwd_len - 1u] = '\0';
        --cwd_len;
    }
    uint64_t path_len = strlen(path);
    if (cwd_len == 1u && cwd[0] == '/') {
        if (path_len + 2u > capacity) {
            return LINUX_ENAMETOOLONG;
        }
        memmove(path + 1u, path, path_len + 1u);
        path[0] = '/';
        return 0;
    }
    if (cwd_len + path_len + 2u > capacity) {
        return LINUX_ENAMETOOLONG;
    }
    memmove(path + cwd_len + 1u, path, path_len + 1u);
    memcpy(path, cwd, cwd_len);
    path[cwd_len] = '/';
    return 0;
}

static int64_t linux_open_resolved(char *path, uint64_t flags);

static int64_t linux_open_path(uint64_t path_ptr, uint64_t flags)
{
    char path[256];
    int64_t rc = linux_copy_cstring(path, sizeof(path),
                                    (const char *)(uintptr_t)path_ptr);
    if (rc < 0) {
        return rc;
    }
    rc = linux_resolve_path(path, sizeof(path));
    if (rc < 0) {
        return rc;
    }
    return linux_open_resolved(path, flags);
}

/* open(2)/openat(2) on a path the caller has already made absolute. Split out
 * so openat() can resolve against a real dirfd first. */
static int64_t linux_open_resolved(char *path, uint64_t flags)
{
    int64_t result;
    if ((flags & LINUX_O_DIRECTORY) != 0u) {
        result = (int64_t)syscall_file_register_dir(path);
    } else {
        result = (int64_t)syscall_file_open(path, flags);
#if CHROME_SHM_TRACE
        if (path[0] == '/' && path[1] == 'p' && path[2] == 'r' &&
            path[3] == 'o' && path[4] == 'c') {
            serial_write_string("[shmtr] open '");
            serial_write_string(path);
            serial_write_string("' flags=");
            serial_write_uint64(flags);
            serial_write_string(" -> ");
            serial_write_uint64((uint64_t)result);
            serial_write_string("\n");
        }
#endif
        if (result == LINUX_ENOENT && (flags & LINUX_O_CREAT) != 0u) {
            /* Create it, then open it with the caller's flags: the access mode
             * open() reports back has to be the one that was asked for. */
            result = (int64_t)syscall_file_creat_ex(path, flags);
        } else if (result == LINUX_ENOENT && (flags & 3u) == 0u) {
            /* open(dir, O_RDONLY) without O_DIRECTORY is valid on Linux, and
             * the file table has no entry for a directory, so it came back
             * ENOENT. LevelDB's SyncParent does exactly this before fsync()ing
             * the directory of every database it creates; the failure made
             * Chromium report "Unable to open directory (... SyncParent::4)"
             * for GCM Store and put up the "Something went wrong when opening
             * your profile" dialog. Only read-only opens qualify: opening a
             * directory for writing is EISDIR on Linux too. */
            int32_t dir_fd = syscall_file_register_dir(path);
            if (dir_fd >= 0) {
                result = (int64_t)dir_fd;
            }
        }
#if LINUX_MODULE_MAP_TRACE
        if (result >= 0) {
            linux_module_map_note_open((int32_t)result, path);
        }
#endif
    }
#if defined(XKB_READ_PROBE)
    /* Focused probe: for any path containing "xkb", dump the fd, the file
     * size the VFS reports, and the raw bytes syscall_file_read() returns at
     * offset 300..360 - the region where libxkbcommon reports a parse error
     * on keycodes/evdev even though the ISO copy is byte-identical to stock. */
    {
        int has_xkb = 0;
        for (uint32_t i = 0; path[i] && i + 2 < sizeof(path); ++i) {
            if (path[i] == 'x' && path[i+1] == 'k' && path[i+2] == 'b') { has_xkb = 1; break; }
        }
        if (has_xkb && result >= 0) {
            vfs_file_t pvf;
            int64_t szinfo = syscall_file_get_file_info((int32_t)result, &pvf, NULL);
            serial_write_string("[xkbprobe] '");
            serial_write_string(path);
            serial_write_string("' fd=");
            serial_write_uint64((uint64_t)result);
            serial_write_string(" info_rc=");
            serial_write_uint64((uint64_t)szinfo);
            serial_write_string(" vf.size=");
            serial_write_uint64(szinfo >= 0 ? (uint64_t)pvf.size : 0u);
            int64_t sv = syscall_file_seek((int32_t)result, 0, LINUX_SEEK_CUR);
            (void)syscall_file_seek((int32_t)result, 300, LINUX_SEEK_SET);
            uint8_t region[64] = {0};
            int64_t rn = syscall_file_read((int32_t)result, region, sizeof(region));
            (void)syscall_file_seek((int32_t)result, sv >= 0 ? sv : 0, LINUX_SEEK_SET);
            serial_write_string(" read@300 n=");
            serial_write_uint64((uint64_t)rn);
            serial_write_string(" [");
            for (int i = 0; i < 48 && i < (int)rn; ++i) {
                char c = (char)region[i];
                serial_write_char((c >= 32 && c < 127) ? c : '.');
            }
            serial_write_string("]\n");
        }
    }
#endif
#ifdef LINUX_SYSCALL_TRACE
    serial_write_string("[lx] open '");
    serial_write_string(path);
    serial_write_string("' -> ");
    serial_write_uint64((uint64_t)result);
    if (result >= 0) {
        /* peek the first 8 bytes so we can tell which file actually got
         * opened / whether the content is an ELF at all */
        uint8_t hdr[8] = {0};
        int64_t saved = syscall_file_seek((int32_t)result, 0, LINUX_SEEK_CUR);
        (void)syscall_file_seek((int32_t)result, 0, LINUX_SEEK_SET);
        int64_t n = syscall_file_read((int32_t)result, hdr, sizeof(hdr));
        if (saved >= 0) {
            (void)syscall_file_seek((int32_t)result, saved, LINUX_SEEK_SET);
        }
        serial_write_string(" hdr[");
        serial_write_uint64((uint64_t)n);
        serial_write_string("]=");
        for (int i = 0; i < 8; ++i) {
            serial_write_uint64((uint64_t)hdr[i]);
            serial_write_char(' ');
        }
    }
    serial_write_char('\n');
#endif
    return result;
}

static int64_t linux_execve(uint64_t path_ptr, uint64_t argv_ptr,
                            uint64_t envp_ptr)
{
    return (int64_t)process_execve(
        (const char *)(uintptr_t)path_ptr,
        (const char *const *)(uintptr_t)argv_ptr,
        (const char *const *)(uintptr_t)envp_ptr);
}

typedef struct {
    uint64_t st_dev;
    uint64_t st_ino;
    uint64_t st_nlink;
    uint32_t st_mode;
    uint32_t st_uid;
    uint32_t st_gid;
    uint32_t __pad0;
    uint64_t st_rdev;
    int64_t st_size;
    int64_t st_blksize;
    int64_t st_blocks;
    int64_t st_atime;
    int64_t st_atime_nsec;
    int64_t st_mtime;
    int64_t st_mtime_nsec;
    int64_t st_ctime;
    int64_t st_ctime_nsec;
    int64_t __unused[3];
} linux_stat64_t;

/* Distinct per-file st_ino: glibc's ld.so dedups already-loaded shared objects
 * by (st_dev, st_ino), so a constant inode makes every .so alias the first one
 * loaded. Callers pass a stable file identity (vfs_file_t.internal_id -- the
 * ISO9660 extent LBA etc.); 0 means "unknown", keep the legacy 1. */
static void linux_stat_fill_common_ino(linux_stat64_t *st, uint64_t size,
                                       uint64_t ino)
{
    st->st_dev = 0x8200u;
    st->st_ino = ino != 0u ? ino : 1u;
    st->st_nlink = 1;
    /* Single-user system: every file belongs to whoever is asking. Chromium
     * checks that its profile and socket directories are owned by the
     * current user before it uses them. */
    {
        uint32_t cu = 0, cg = 0;
        (void)process_get_credentials(process_get_current_pid(), &cu, &cg);
        st->st_uid = cu;
        st->st_gid = cg;
    }
    st->st_size = (int64_t)size;
    st->st_blksize = 512;
    st->st_blocks = (size + 511u) / 512u;
    st->st_atime = linux_realtime_seconds();
    st->st_mtime = st->st_atime;
    st->st_ctime = st->st_atime;
}

static void linux_stat_fill_common(linux_stat64_t *st, uint64_t size)
{
    linux_stat_fill_common_ino(st, size, 0u);
}

/* Fill *st for a path. Returns 0 or a negative LINUX_E* code. Shared by
 * stat/lstat/newfstatat and statx, which used to carry two copies that had
 * drifted apart -- statx reported a fixed 0755 for every directory and never
 * asked vfs_get_mode(), so whichever of the two a program happened to use
 * decided whether it saw the real mode. */
static int64_t linux_fill_stat_for_fd(int32_t fd, linux_stat64_t *st);

/* If `path` is "/proc/<pid>/task" (or ".../task/"), the pid it names;
 * otherwise -1. */
static int32_t linux_proc_task_dir_pid(const char *path)
{
    if (path == NULL || strncmp(path, "/proc/", 6) != 0) {
        return -1;
    }
    const char *rest = path + 6;
    int32_t pid;
    if (strncmp(rest, "self/", 5) == 0) {
        pid = process_get_current_pid();
        rest += 5;
    } else {
        pid = 0;
        int digits = 0;
        while (*rest >= '0' && *rest <= '9') {
            pid = pid * 10 + (*rest - '0');
            ++rest;
            ++digits;
        }
        if (digits == 0 || *rest != '/') {
            return -1;
        }
        ++rest;
    }
    if (strcmp(rest, "task") != 0 && strcmp(rest, "task/") != 0) {
        return -1;
    }
    return pid;
}

static int64_t linux_fill_stat_for_path(const char *path, linux_stat64_t *st)
{
    memset(st, 0, sizeof(*st));

    vfs_file_t vf;
    if (vfs_find_file(path, &vf)) {
        linux_stat_fill_common_ino(st, vf.size, vf.internal_id);
        if (devfs_path_is_device(path)) {
            st->st_mode = LINUX_S_IFCHR | 0x1B6u; /* crw-rw-rw- */
            st->st_size = 0;
            st->st_rdev = 0x0105u; /* arbitrary but stable device number */
        } else {
            int32_t stored = vfs_get_mode(path);
            st->st_mode = LINUX_S_IFREG |
                          (uint32_t)(stored >= 0 ? stored : 0x1A4);
        }
        return 0;
    }

    /* /proc/self/fd/<n> is a symlink to whatever <n> is open on, and stat()
     * follows it. Chromium's sandbox walks that directory and fstatat()s
     * every entry to prove no directory descriptor is still open before it
     * locks itself down, so the answer has to be the descriptor's own type
     * rather than ENOENT. */
    {
        int32_t proc_fd = procfs_parse_fd_path(path);
        if (proc_fd >= 0) {
            return linux_fill_stat_for_fd(proc_fd, st);
        }
    }

    /* Not a file, so: is it a directory? Asking costs a directory handle,
     * because no filesystem here exposes a cheaper "is this a directory"
     * query -- which also means a filesystem that has run out of handles
     * answers this with "no such path". See TMPFS_DIR_HANDLE_MAX. */
    int32_t dir_handle = vfs_opendir(path);
    if (dir_handle < 0) {
        return LINUX_ENOENT;
    }
    (void)vfs_closedir(dir_handle);
    linux_stat_fill_common(st, 0);
    /* The stored mode matters: mkdtemp() makes its directory 0700 and
       Chromium's ProcessSingleton CHECK()s that stat() says 0700 --
       reporting a fixed 0755 aborted the browser. */
    int32_t stored = vfs_get_mode(path);
    st->st_mode = LINUX_S_IFDIR | (uint32_t)(stored >= 0 ? stored : 0x1ED);
    /* /proc/<pid>/task counts threads through its link count, and that is how
     * Chromium's sandbox decides whether it is safe to lock itself down:
     *   fstatat(proc_fd, "self/task/", &st, 0);
     *   CHECK_LE(3UL, st.st_nlink);        // ".", "..", one thread
     *   return st.st_nlink == 3;           // single-threaded
     * With the default nlink of 1 the zygote died on that CHECK the moment it
     * started. Report ".", ".." plus one entry per thread. */
    {
        int32_t task_pid = linux_proc_task_dir_pid(path);
        if (task_pid >= 0) {
            int32_t threads = process_count_threads(task_pid);
            if (threads < 1) {
                threads = 1;
            }
            st->st_nlink = 2u + (uint64_t)(uint32_t)threads;
        }
    }
    return 0;
}

static int64_t linux_stat_path(const char *path, uint64_t statbuf_ptr)
{
    if (statbuf_ptr == 0u ||
        !process_user_buffer_is_valid((const void *)(uintptr_t)statbuf_ptr,
                                      sizeof(linux_stat64_t))) {
        return LINUX_EFAULT;
    }
    linux_stat64_t st;
    int64_t rc = linux_fill_stat_for_path(path, &st);
    if (rc != 0) {
        return rc;
    }
    if (copy_to_user_trusted((void *)(uintptr_t)statbuf_ptr, &st, sizeof(st)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

/* Fill *st for an already-open fd (fstat / statx+AT_EMPTY_PATH). Returns 0 or a
 * negative LINUX_E* code. Directory fds must report S_ISDIR: glibc's opendir()
 * fstat()s the fd it just opened and returns NULL for anything that is not a
 * directory, so without this every opendir() from a foreign binary failed --
 * which is why Xorg found no loadable modules ("No drivers available" ->
 * "no screens found") even though modules/{drivers,extensions,input} were on
 * the medium. */
static int64_t linux_fill_stat_for_fd(int32_t fd, linux_stat64_t *st)
{
    memset(st, 0, sizeof(*st));

    vfs_file_t vf;
    uint32_t writable = 0;
    if (syscall_file_get_file_info(fd, &vf, &writable) == 0) {
        linux_stat_fill_common_ino(st, vf.size, vf.internal_id);
        st->st_mode = LINUX_S_IFREG | (writable != 0u ? 0x1A4u : 0x124u);
        return 0;
    }
    if (syscall_file_is_dir(fd)) {
        linux_stat_fill_common(st, 0);
        st->st_mode = LINUX_S_IFDIR | 0x1EDu; /* drwxr-xr-x */
        st->st_blksize = 4096;
        return 0;
    }
    syscall_socket_info_t info;
    if (syscall_socket_get_info(fd, &info) == 0) {
        linux_stat_fill_common(st, 0);
        st->st_mode = LINUX_S_IFREG | 0x1A4u;
        return 0;
    }
    return LINUX_EBADF;
}

static int64_t linux_stat_fd(int32_t fd, uint64_t statbuf_ptr)
{
    if (statbuf_ptr == 0u ||
        !process_user_buffer_is_valid((const void *)(uintptr_t)statbuf_ptr,
                                      sizeof(linux_stat64_t))) {
        return LINUX_EFAULT;
    }
    linux_stat64_t st;
    int64_t rc = linux_fill_stat_for_fd(fd, &st);
    if (rc != 0) {
        return rc;
    }
    if (copy_to_user_trusted((void *)(uintptr_t)statbuf_ptr, &st, sizeof(st)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

typedef struct {
    uint64_t d_ino;
    int64_t d_off;
    uint16_t d_reclen;
    uint8_t d_type;
    char d_name[256];
} linux_dirent64_t;

static int64_t linux_getdents64(int32_t fd, uint64_t buf_ptr, uint64_t count)
{
    if (buf_ptr == 0u || count == 0u ||
        !process_user_buffer_is_valid((const void *)(uintptr_t)buf_ptr,
                                      count)) {
        return LINUX_EFAULT;
    }
    uint64_t written = 0;
    for (;;) {
        vfs_dirent_t entry;
        int32_t rc = syscall_file_get_dir_dirent(fd, &entry);
        if (rc <= 0) {
            break;
        }
        uint64_t name_len = 0;
        while (name_len < sizeof(entry.name) && entry.name[name_len] != '\0') {
            ++name_len;
        }
        uint16_t reclen =
            (uint16_t)((offsetof(linux_dirent64_t, d_name) + name_len + 1u + 7u) &
                       ~7ULL);
        if (written + reclen > count) {
            break;
        }
        linux_dirent64_t d;
        memset(&d, 0, sizeof(d));
        d.d_ino = 1;
        d.d_reclen = reclen;
        d.d_type = entry.is_directory ? 4u : 8u;
        memcpy(d.d_name, entry.name, name_len);
        d.d_name[name_len] = '\0';
        d.d_off = (int64_t)written + reclen;
        if (copy_to_user_trusted((uint8_t *)(uintptr_t)buf_ptr + written,
                                 &d, reclen) != 0u) {
            return LINUX_EFAULT;
        }
        written += reclen;
    }
    return (int64_t)written;
}

static int64_t linux_rseq(uint64_t rseq, uint64_t length, uint64_t flags,
                          uint32_t sig)
{
    if ((flags & LINUX_RSEQ_FLAG_UNREGISTER) != 0u) {
        return (int64_t)process_rseq_unregister();
    }
    if (flags != 0u) {
        return LINUX_EINVAL;
    }
    if (rseq == 0u) {
        return LINUX_EINVAL;
    }
    if (length != (uint64_t)PROCESS_RSEQ_AREA_SIZE) {
        return LINUX_EINVAL;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)rseq,
                                      PROCESS_RSEQ_AREA_SIZE)) {
        return LINUX_EFAULT;
    }
    return (int64_t)process_rseq_register(rseq, sig);
}

/*
 * Namespaces, chroot and seccomp.
 *
 * Chromium refuses to start without --no-sandbox unless it can build a
 * "layer one" sandbox, and on Linux that means unprivileged user namespaces:
 * it probes /proc/self/ns/user, forks with CLONE_NEWUSER, writes the id maps,
 * chroots to an empty directory and drops its capabilities. Every one of
 * those steps has to succeed for the browser to get past
 * ZygoteHostImpl::Init.
 *
 * What is real here and what is not:
 *   - chroot(2) is real. It redirects every absolute path the process
 *     resolves (linux_apply_root), which is what actually removes the
 *     filesystem from a sandboxed renderer's reach, and it is what Chromium
 *     verifies by checking /proc has gone.
 *   - capabilities are real state: capset() records the set and capget()
 *     reports it back, so DropAllCapabilities() followed by HasAnyCapability()
 *     answers correctly.
 *   - the namespaces themselves are NOT isolation. This kernel has one pid
 *     space, one network stack and one mount table, and CLONE_NEWPID /
 *     CLONE_NEWNET / CLONE_NEWNS are accepted and ignored. A renderer is
 *     confined to the extent that it has no filesystem, no capabilities and
 *     no descriptors it was not given -- it is not confined from the other
 *     processes on the system.
 *   - seccomp-bpf ("layer two") is deliberately reported as unsupported
 *     rather than faked. A filter that is accepted and not enforced would
 *     make about:sandbox claim a syscall firewall that does not exist;
 *     Chromium handles an absent one by running without it and saying so.
 */
#define LINUX_CLONE_NEWNS     0x00020000u
#define LINUX_CLONE_NEWCGROUP 0x02000000u
#define LINUX_CLONE_NEWUTS    0x04000000u
#define LINUX_CLONE_NEWIPC    0x08000000u
#define LINUX_CLONE_NEWUSER   0x10000000u
#define LINUX_CLONE_NEWPID    0x20000000u
#define LINUX_CLONE_NEWNET    0x40000000u
#define LINUX_CLONE_NEW_ANY   (LINUX_CLONE_NEWNS | LINUX_CLONE_NEWCGROUP | \
                               LINUX_CLONE_NEWUTS | LINUX_CLONE_NEWIPC | \
                               LINUX_CLONE_NEWUSER | LINUX_CLONE_NEWPID | \
                               LINUX_CLONE_NEWNET)

/* Linux capability sets, per process.
 *
 * capget() used to answer "all bits set" unconditionally, which made
 * Credentials::DropAllCapabilities() followed by HasAnyCapability() report
 * that the drop had not worked. The three sets are stored as the 64-bit
 * values a v3 capget/capset exchanges; everything starts fully privileged
 * because this is a uid-0 system. */
typedef struct {
    uint64_t effective;
    uint64_t permitted;
    uint64_t inheritable;
    uint8_t  valid;
} linux_caps_t;

static linux_caps_t g_linux_caps[OS_CONFIG_PROCESS_MAX_COUNT];

static linux_caps_t *linux_caps_for_current(void)
{
    int32_t pid = process_memory_owner_pid_of(process_get_current_pid());
    if (pid < 0 || pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) {
        return NULL;
    }
    linux_caps_t *caps = &g_linux_caps[pid];
    if (!caps->valid) {
        caps->effective = ~0ULL;
        caps->permitted = ~0ULL;
        caps->inheritable = 0ULL;
        caps->valid = 1u;
    }
    return caps;
}

/* A capget/capset data block is two {effective, permitted, inheritable}
 * 32-bit triples for the v3 layout: index 0 carries bits 0..31, index 1 bits
 * 32..63, and the fields are interleaved per index. */
typedef struct {
    uint32_t effective;
    uint32_t permitted;
    uint32_t inheritable;
} linux_cap_data_t;

static int64_t linux_capget(uint64_t header_ptr, uint64_t data_ptr)
{
    (void)header_ptr;
    linux_caps_t *caps = linux_caps_for_current();
    if (caps == NULL) {
        return LINUX_EINVAL;
    }
    if (data_ptr == 0u) {
        return 0; /* header-only probe: "the version is fine" */
    }
    linux_cap_data_t data[2];
    data[0].effective   = (uint32_t)(caps->effective & 0xFFFFFFFFu);
    data[0].permitted   = (uint32_t)(caps->permitted & 0xFFFFFFFFu);
    data[0].inheritable = (uint32_t)(caps->inheritable & 0xFFFFFFFFu);
    data[1].effective   = (uint32_t)(caps->effective >> 32);
    data[1].permitted   = (uint32_t)(caps->permitted >> 32);
    data[1].inheritable = (uint32_t)(caps->inheritable >> 32);
    if (copy_to_user((void *)(uintptr_t)data_ptr, data, sizeof(data)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t linux_capset(uint64_t header_ptr, uint64_t data_ptr)
{
    (void)header_ptr;
    linux_caps_t *caps = linux_caps_for_current();
    if (caps == NULL) {
        return LINUX_EINVAL;
    }
    if (data_ptr == 0u) {
        return LINUX_EFAULT;
    }
    linux_cap_data_t data[2];
    if (copy_from_user(data, (const void *)(uintptr_t)data_ptr,
                       sizeof(data)) != 0u) {
        return LINUX_EFAULT;
    }
    caps->effective = (uint64_t)data[0].effective |
                      ((uint64_t)data[1].effective << 32);
    caps->permitted = (uint64_t)data[0].permitted |
                      ((uint64_t)data[1].permitted << 32);
    caps->inheritable = (uint64_t)data[0].inheritable |
                        ((uint64_t)data[1].inheritable << 32);
    return 0;
}

static int64_t linux_unshare(uint64_t flags)
{
    /* CLONE_FS/CLONE_FILES/CLONE_SYSVSEM detach shared state a thread was
     * created with; nothing here shares those per-thread to begin with. The
     * namespace bits are accepted as described above. */
    const uint64_t known = LINUX_CLONE_NEW_ANY | LINUX_CLONE_FS |
                           0x00000400u /* CLONE_FILES */ |
                           0x00040000u /* CLONE_SYSVSEM */;
    if ((flags & ~known) != 0u) {
        return LINUX_EINVAL;
    }
    return 0;
}

static int64_t linux_chroot(uint64_t path_ptr)
{
    char path[256];
    int64_t rc = linux_copy_cstring(path, sizeof(path),
                                    (const char *)(uintptr_t)path_ptr);
    if (rc < 0) {
        return rc;
    }
    /* Resolve against the root already in force, so a second chroot nests
     * the way Linux's does. */
    rc = linux_resolve_path(path, sizeof(path));
    if (rc < 0) {
        return rc;
    }
    /* "/proc/self/..." names the *caller*. Chromium chroots a short-lived
     * helper (clone(CLONE_FS)) to /proc/self/fdinfo/ precisely so that the
     * directory disappears when the helper exits, leaving the sandboxed
     * process rooted in nothing. Keeping the literal "self" would re-resolve
     * to whichever process looks later -- a live directory. Pin the pid. */
    if (strncmp(path, "/proc/self/", 11) == 0 || strcmp(path, "/proc/self") == 0) {
        char pinned[256];
        int n = snprintf(pinned, sizeof(pinned), "/proc/%d%s",
                         (int)process_memory_owner_pid_of(process_get_current_pid()),
                         path + 10);
        if (n <= 0 || (size_t)n >= sizeof(pinned)) {
            return LINUX_ENAMETOOLONG;
        }
        memcpy(path, pinned, (size_t)n + 1u);
    }
    /* It has to be a directory that exists: Chromium chroots to
     * /proc/self/fdinfo and would otherwise be confined to nothing at all. */
    int32_t handle = vfs_opendir(path);
    if (handle < 0) {
        return LINUX_ENOENT;
    }
    (void)vfs_closedir(handle);
    if (process_set_current_root(path) != 0) {
        return LINUX_EINVAL;
    }
    return 0;
}

static int64_t linux_seccomp(uint64_t operation)
{
    /* SECCOMP_GET_ACTION_AVAIL(2) / SECCOMP_GET_NOTIF_SIZES(3) are probes;
     * SET_MODE_STRICT(0) and SET_MODE_FILTER(1) would install a filter this
     * kernel cannot enforce. Report the feature as absent. Chromium's
     * KernelSupportsSeccompBPF() reads that and runs with layer one only. */
    (void)operation;
    return LINUX_ENOSYS;
}

static int64_t linux_prctl_ext(uint64_t option, uint64_t arg2, uint64_t arg3,
                               uint64_t arg4, uint64_t arg5);

static int64_t linux_prctl(uint64_t option, uint64_t arg2, uint64_t arg3,
                           uint64_t arg4, uint64_t arg5)
{
    (void)arg3;
    (void)arg4;
    (void)arg5;
    switch (option) {
        case LINUX_PR_SET_NAME: {
            /* Linux truncates at 16 bytes (15 + NUL) and never fails on a
             * long name. linux_copy_cstring() reports "no NUL within the
             * buffer" as EFAULT, which turned every Chromium thread name
             * longer than 15 characters ("ThreadPoolForegroundWorker", ...)
             * into a failed prctl. Copy byte by byte and stop at the cap. */
            char name[16];
            uint64_t i = 0;
            for (; i < sizeof(name) - 1u; ++i) {
                char ch = '\0';
                if (copy_from_user(&ch, (const char *)(uintptr_t)arg2 + i,
                                   1u) != 0u) {
                    return (i == 0u) ? LINUX_EFAULT : 0;
                }
                if (ch == '\0') {
                    break;
                }
                name[i] = ch;
            }
            name[i] = '\0';
            return (int64_t)process_set_current_name(name, 15u);
        }
        case LINUX_PR_GET_NAME: {
            char name[16];
            if (process_get_current_name(name, sizeof(name)) < 0) {
                return -1;
            }
            name[15] = '\0';
            if (!process_user_buffer_is_valid((void *)(uintptr_t)arg2, 16u)) {
                return LINUX_EFAULT;
            }
            return copy_to_user_trusted((void *)(uintptr_t)arg2,
                                        name, 16u) != 0u ? LINUX_EFAULT : 0;
        }
        default:
            return linux_prctl_ext(option, arg2, arg3, arg4, arg5);
    }
}

static int64_t linux_getcwd(uint64_t buf, uint64_t size)
{
    if (buf == 0u || size == 0u) {
        return LINUX_EINVAL;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)buf, size)) {
        return LINUX_EFAULT;
    }
    char cwd[256];
    if (process_get_current_cwd(cwd, sizeof(cwd)) < 0) {
        return -1;
    }
    uint64_t len = strlen(cwd) + 1u;
    if (len > size) {
        return LINUX_ENAMETOOLONG;
    }
    return copy_to_user_trusted((void *)(uintptr_t)buf, cwd, len) != 0u ?
        LINUX_EFAULT : (int64_t)len;
}

static int64_t linux_chdir(uint64_t path_ptr)
{
    char path[256];
    int64_t rc = linux_copy_cstring(path, sizeof(path),
                                    (const char *)(uintptr_t)path_ptr);
    if (rc < 0) {
        return rc;
    }
    rc = linux_resolve_path(path, sizeof(path));
    if (rc < 0) {
        return rc;
    }
    vfs_file_t vf;
    if (vfs_find_file(path, &vf)) {
        return LINUX_ENOTDIR;
    }
    int32_t dir_handle = vfs_opendir(path);
    if (dir_handle < 0) {
        return LINUX_ENOENT;
    }
    (void)vfs_closedir(dir_handle);
    return (int64_t)process_set_current_cwd(path);
}

typedef struct {
    uint16_t sin_family;
    uint16_t sin_port;
    uint32_t sin_addr;
    uint8_t sin_zero[8];
} linux_sockaddr_in_t;

static inline uint16_t linux_be16_to_host(uint16_t value)
{
    return (uint16_t)((value >> 8) | (uint16_t)(value << 8));
}

static inline uint32_t linux_be32_to_host(uint32_t value)
{
    return ((value & 0xFFu) << 24) | ((value & 0xFF00u) << 8) |
           ((value >> 8) & 0xFF00u) | ((value >> 24) & 0xFFu);
}

typedef struct {
    uint16_t sun_family;
    char sun_path[108];
} linux_sockaddr_un_t;

/* Peeks sa_family (identical offset/width in every Linux sockaddr_*
 * variant) without committing to a full sockaddr_in vs sockaddr_un copy. */
static int64_t linux_sockaddr_family(uint64_t addr_ptr, uint16_t *family_out)
{
    if (addr_ptr == 0u) {
        return LINUX_EFAULT;
    }
    if (copy_from_user(family_out, (const void *)(uintptr_t)addr_ptr,
                       sizeof(*family_out)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

/* Render a sockaddr_un as the flat string the AF_UNIX table keys on.
 *
 * Abstract addresses (sun_path[0] == '\0', the name running to addr_len) must
 * NOT be flattened with strlen(): that yields "" for every one of them, so
 * they all alias each other and any connect() matches whichever unrelated
 * socket happens to be bound to "". That is exactly what broke X clients here
 * -- libxcb tries the abstract socket first, so Doom's connect() "succeeded"
 * against the wrong endpoint and then hung on a handshake nobody would answer.
 * Encode them the way Linux itself displays them (ss, /proc/net/unix): a
 * leading '@' followed by the name. Names containing embedded NULs are
 * truncated at the first one; nothing in tree uses those. */
static int64_t linux_copy_sockaddr_un(uint64_t addr_ptr, uint64_t addr_len,
                                      char *path_out, uint64_t path_cap)
{
    if (addr_ptr == 0u) {
        return LINUX_EFAULT;
    }
    linux_sockaddr_un_t addr;
    memset(&addr, 0, sizeof(addr));
    if (copy_from_user(&addr, (const void *)(uintptr_t)addr_ptr,
                       sizeof(addr)) != 0u) {
        return LINUX_EFAULT;
    }
    if (addr.sun_family != LINUX_AF_UNIX) {
        return LINUX_EAFNOSUPPORT;
    }
    addr.sun_path[sizeof(addr.sun_path) - 1] = '\0';

    const uint64_t path_off = offsetof(linux_sockaddr_un_t, sun_path);

    if (addr.sun_path[0] == '\0') {
        /* Unnamed (addr_len covers only sun_family): autobind, unsupported. */
        if (addr_len <= path_off) {
            return LINUX_EINVAL;
        }
        uint64_t avail = addr_len - path_off;
        if (avail > sizeof(addr.sun_path) - 1u) {
            avail = sizeof(addr.sun_path) - 1u;
        }
        uint64_t len = strlen(addr.sun_path + 1); /* stops at an embedded NUL */
        if (len > avail - 1u) {
            len = avail - 1u;
        }
        if (len == 0u) {
            return LINUX_EINVAL;
        }
        if (len + 2u > path_cap) {
            return LINUX_ENAMETOOLONG;
        }
        path_out[0] = '@';
        memcpy(path_out + 1, addr.sun_path + 1, len);
        path_out[len + 1u] = '\0';
        return 0;
    }

    uint64_t len = strlen(addr.sun_path);
    if (len + 1u > path_cap) {
        return LINUX_ENAMETOOLONG;
    }
    memcpy(path_out, addr.sun_path, len + 1u);
    return 0;
}

static int64_t linux_copy_sockaddr_in(uint64_t addr_ptr, uint32_t *ip_out,
                                      uint16_t *port_out)
{
    if (addr_ptr == 0u) {
        return LINUX_EFAULT;
    }
    linux_sockaddr_in_t addr;
    if (copy_from_user(&addr, (const void *)(uintptr_t)addr_ptr,
                       sizeof(addr)) != 0u) {
        return LINUX_EFAULT;
    }
    if (addr.sin_family != LINUX_AF_INET) {
        return LINUX_EAFNOSUPPORT;
    }
    if (ip_out != NULL) *ip_out = linux_be32_to_host(addr.sin_addr);
    if (port_out != NULL) *port_out = linux_be16_to_host(addr.sin_port);
    return 0;
}

/* socketpair(2) - only AF_UNIX is meaningful here (Mojo IPC's primary
 * transport - TODO_Chromium_LinuxABI.md 3.7). */
static int64_t linux_socketpair(uint64_t domain, uint64_t type,
                                uint64_t protocol, uint64_t fds_ptr)
{
    (void)protocol;
    if (domain != LINUX_AF_UNIX) {
        return LINUX_EAFNOSUPPORT;
    }
    uint64_t base_type = type & 0xFu;
    if (base_type != LINUX_SOCK_STREAM && base_type != 5u && base_type != 2u) {
        return LINUX_EPROTONOSUPPORT;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)fds_ptr,
                                      sizeof(int32_t) * 2u)) {
        return LINUX_EFAULT;
    }
    int32_t fds[2];
    /* The type matters: SOCK_SEQPACKET keeps message boundaries, and
     * Chromium's zygote protocol depends on reading one request per call. */
    int64_t rc = unix_socket_pair_typed((int32_t)base_type, fds);
    if (rc < 0) {
        return LINUX_ENOMEM;
    }
    if (copy_to_user_trusted((void *)(uintptr_t)fds_ptr, fds, sizeof(fds)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t linux_socket_create(uint64_t domain, uint64_t type,
                                   uint64_t protocol)
{
    (void)protocol;
    uint64_t base_type = type & 0xFu;
    if (domain == LINUX_AF_UNIX) {
        /* SOCK_STREAM(1), or SOCK_SEQPACKET(5)/SOCK_DGRAM(2), which keep
         * message boundaries (UnixSocket.c, `seqpacket`). */
        if (base_type != LINUX_SOCK_STREAM && base_type != 5u && base_type != 2u) {
            return LINUX_EPROTONOSUPPORT;
        }
        int64_t fd = unix_socket_create((int32_t)base_type);
        return fd < 0 ? LINUX_ENOMEM : fd;
    }
    if (domain != LINUX_AF_INET) {
        return LINUX_EAFNOSUPPORT;
    }
    if (base_type != LINUX_SOCK_STREAM && base_type != LINUX_SOCK_DGRAM) {
        return LINUX_EPROTONOSUPPORT;
    }
    int32_t fd = syscall_socket_create((int32_t)base_type);
    if (fd < 0) {
        return LINUX_ENOMEM;
    }
    /* SOCK_NONBLOCK (0x800) is honored up front; SOCK_CLOEXEC (0x80000) is
     * accepted but sockets have no per-fd close-on-exec bit yet (they always
     * inherit across execve, the POSIX default). */
    if ((type & 0x800u) != 0u) {
        (void)syscall_socket_set_nonblocking(fd, 1);
    }
    return (int64_t)fd;
}

static int64_t linux_socket_bind(uint64_t fd, uint64_t addr_ptr,
                                 uint64_t addr_len)
{
    uint16_t family = 0;
    int64_t rc = linux_sockaddr_family(addr_ptr, &family);
    if (rc < 0) {
        return rc;
    }
    if (family == LINUX_AF_UNIX) {
        char path[110];
        rc = linux_copy_sockaddr_un(addr_ptr, addr_len, path, sizeof(path));
        if (rc < 0) return rc;
        rc = unix_socket_bind((int32_t)fd, path);
        return rc < 0 ? LINUX_EINVAL : rc;
    }
    uint32_t ip;
    uint16_t port;
    rc = linux_copy_sockaddr_in(addr_ptr, &ip, &port);
    if (rc < 0) {
        return rc;
    }
    (void)ip;
    return (int64_t)syscall_socket_bind((int32_t)fd, port);
}

static int64_t linux_socket_connect(uint64_t fd, uint64_t addr_ptr,
                                    uint64_t addr_len)
{
    uint16_t family = 0;
    int64_t rc = linux_sockaddr_family(addr_ptr, &family);
    if (rc < 0) {
        return rc;
    }
    if (family == LINUX_AF_UNIX) {
        char path[110];
        rc = linux_copy_sockaddr_un(addr_ptr, addr_len, path, sizeof(path));
        if (rc < 0) return rc;
        rc = unix_socket_connect((int32_t)fd, path);
        return rc < 0 ? LINUX_ENOENT : rc;
    }
    uint32_t ip;
    uint16_t port;
    rc = linux_copy_sockaddr_in(addr_ptr, &ip, &port);
    if (rc < 0) {
        return rc;
    }
    rc = (int64_t)syscall_socket_connect((int32_t)fd, ip, port);
    if (rc == 0 &&
        syscall_socket_get_type((int32_t)fd) == (int32_t)LINUX_SOCK_STREAM &&
        syscall_socket_is_nonblocking((int32_t)fd)) {
        /* The TCP stack only fires the SYN here; the 3-way handshake finishes
         * asynchronously. Non-blocking connect(2) must report EINPROGRESS so
         * the caller waits for POLLOUT and then reads SO_ERROR (which already
         * synthesises ECONNREFUSED/ETIMEDOUT once the connection dies). */
        return LINUX_EINPROGRESS;
    }
    return rc;
}

static int64_t linux_socket_accept(uint64_t fd, uint64_t addr_ptr,
                                   uint64_t addr_len_ptr)
{
    if (unix_socket_fd_in_range((int32_t)fd)) {
        int64_t accepted = unix_socket_accept((int32_t)fd);
        if (accepted < 0) return accepted;
        if (addr_len_ptr != 0u) {
            int32_t addr_len = 0;
            (void)copy_to_user((void *)(uintptr_t)addr_len_ptr, &addr_len,
                               sizeof(addr_len));
        }
        return accepted;
    }
    if (addr_ptr != 0u && addr_len_ptr != 0u) {
        int32_t addr_len = 0;
        if (copy_from_user(&addr_len, (const void *)(uintptr_t)addr_len_ptr,
                           sizeof(addr_len)) != 0u) {
            return LINUX_EFAULT;
        }
        if (addr_len < (int32_t)sizeof(linux_sockaddr_in_t)) {
            return LINUX_EINVAL;
        }
        int32_t accepted = syscall_socket_accept((int32_t)fd);
        if (accepted < 0) {
            return (int64_t)accepted;
        }
        syscall_socket_info_t info;
        if (syscall_socket_get_info(accepted, &info) != 0) {
            return LINUX_EBADF;
        }
        linux_sockaddr_in_t addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = LINUX_AF_INET;
        addr.sin_port = linux_be16_to_host(info.remote_port);
        addr.sin_addr = info.remote_ip;
        if (copy_to_user((void *)(uintptr_t)addr_ptr, &addr,
                         sizeof(addr)) != 0u) {
            return LINUX_EFAULT;
        }
        if (copy_to_user((void *)(uintptr_t)addr_len_ptr,
                         &addr_len, sizeof(addr_len)) != 0u) {
            return LINUX_EFAULT;
        }
        return accepted;
    }
    return (int64_t)syscall_socket_accept((int32_t)fd);
}

/* accept4(2) (syscall 288): accept() plus SOCK_NONBLOCK/SOCK_CLOEXEC applied to
 * the new fd. glibc's accept() is a thin wrapper over this. */
static int64_t linux_socket_accept4(uint64_t fd, uint64_t addr_ptr,
                                    uint64_t addr_len_ptr, uint64_t flags)
{
    int64_t accepted = linux_socket_accept(fd, addr_ptr, addr_len_ptr);
    if (accepted >= 0 && (flags & 0x800u) != 0u &&
        syscall_socket_fd_in_range((int32_t)accepted)) {
        (void)syscall_socket_set_nonblocking((int32_t)accepted, 1);
    }
    return accepted;
}

/* If a stream send failed because the peer is gone (EPIPE), apply POSIX
 * SIGPIPE semantics unless the caller passed MSG_NOSIGNAL. Returns the value
 * the syscall should yield. */
static int64_t linux_send_result_sigpipe(int64_t result, uint64_t flags)
{
    if (result == (int64_t)OS_STATUS_BROKEN_PIPE) {
        if ((flags & 0x4000u) == 0u) { /* MSG_NOSIGNAL */
            int32_t self = process_get_current_pid();
            if (self >= 0) {
                (void)process_signal_deliver(self, 13 /* SIGPIPE */);
            }
        }
        return -32; /* EPIPE */
    }
    return result;
}

static int64_t linux_socket_sendto(uint64_t fd, uint64_t buf, uint64_t len,
                                   uint64_t flags, uint64_t addr_ptr,
                                   uint64_t addr_len)
{
    (void)addr_len;
    if (len > 65535u) {
        len = 65535u;
    }
    if (len != 0u &&
        !process_user_buffer_is_valid((const void *)(uintptr_t)buf, len)) {
        return LINUX_EFAULT;
    }
    if (unix_socket_fd_in_range((int32_t)fd)) {
        /* AF_UNIX has no notion of a per-call destination address (Mojo
         * IPC always uses plain send()/recv() on an already-connected or
         * socketpair()-created endpoint), so addr_ptr is ignored here
         * exactly like Linux does for a connected/pair-created socket. */
        int64_t sent = unix_socket_send((int32_t)fd,
                                        (const void *)(uintptr_t)buf, len);
        /* As in recvfrom above: EAGAIN (ring full) has to stay EAGAIN so the
         * caller retries instead of treating the write as fatal. */
        return sent;
    }

    uint32_t dst_ip = 0;
    uint16_t dst_port = 0;
    if (addr_ptr != 0u) {
        int64_t rc = linux_copy_sockaddr_in(addr_ptr, &dst_ip, &dst_port);
        if (rc < 0) {
            return rc;
        }
    }

    if (syscall_socket_get_type((int32_t)fd) == (int32_t)LINUX_SOCK_STREAM) {
        /* TCP: an address argument only makes sense as an implicit
         * connect() on a not-yet-connected socket (matches historical
         * behavior here); once connected, sendto()'s address is ignored
         * like Linux does for connection-oriented sockets. */
        if (addr_ptr != 0u && dst_port != 0u && dst_ip != 0u) {
            int64_t rc2 = linux_socket_connect(fd, addr_ptr, addr_len);
            if (rc2 < 0 && rc2 != LINUX_EINVAL) {
                /* EINVAL here most likely means "already connected",
                 * which is fine for a subsequent sendto(). */
                return rc2;
            }
        }
        int64_t sent = (int64_t)syscall_socket_send(
            (int32_t)fd, (const void *)(uintptr_t)buf, (uint16_t)len);
        return linux_send_result_sigpipe(sent, flags);
    }

    /* UDP: each sendto() targets its own address independently (no
     * implicit connect()); dst_ip/dst_port are 0 when addr_ptr was NULL,
     * in which case syscall_socket_sendto() falls back to the
     * connect(2)-recorded default peer, if any. */
    return (int64_t)syscall_socket_sendto((int32_t)fd,
                                          (const void *)(uintptr_t)buf,
                                          (uint16_t)len, dst_ip, dst_port);
}

static int64_t linux_socket_recvfrom(uint64_t fd, uint64_t buf, uint64_t len,
                                     uint64_t flags, uint64_t addr_ptr,
                                     uint64_t addr_len_ptr)
{
    (void)flags;
    if (len > 65535u) {
        len = 65535u;
    }
    if (len != 0u &&
        !process_user_buffer_is_valid((const void *)(uintptr_t)buf, len)) {
        return LINUX_EFAULT;
    }
    if (unix_socket_fd_in_range((int32_t)fd)) {
        int64_t got = unix_socket_recv((int32_t)fd, (void *)(uintptr_t)buf, len);
        /* Report what actually happened. Collapsing every error to EBUSY threw
         * away the one distinction the caller cares about: EAGAIN means "not
         * yet, wait" and anything else means "give up". libxcb's read_block()
         * retries on EAGAIN and aborts the connection on anything else, so an
         * EBUSY here ended the X11 handshake after a single recv() -- the X
         * server had accepted the client and was still composing its reply.
         * See TODO_Doom_Xorg_MethodA.md M23. */
        if (got < 0) return got;
        if (addr_len_ptr != 0u) {
            int32_t addr_len = 0;
            (void)copy_to_user((void *)(uintptr_t)addr_len_ptr, &addr_len,
                               sizeof(addr_len));
        }
        return got;
    }

    uint32_t src_ip = 0;
    uint16_t src_port = 0;
    int need_addr = (addr_ptr != 0u);
    int64_t result = (int64_t)syscall_socket_recvfrom(
        (int32_t)fd, (void *)(uintptr_t)buf, (uint16_t)len,
        need_addr ? &src_ip : NULL, need_addr ? &src_port : NULL);

    if (result >= 0 && need_addr) {
        linux_sockaddr_in_t addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = LINUX_AF_INET;
        addr.sin_port = linux_be16_to_host(src_port);
        addr.sin_addr = src_ip;
        if (copy_to_user((void *)(uintptr_t)addr_ptr, &addr, sizeof(addr)) != 0u) {
            return LINUX_EFAULT;
        }
        if (addr_len_ptr != 0u) {
            int32_t addr_len = (int32_t)sizeof(addr);
            if (copy_to_user((void *)(uintptr_t)addr_len_ptr, &addr_len,
                             sizeof(addr_len)) != 0u) {
                return LINUX_EFAULT;
            }
        }
    }
    return result;
}

/* recvmsg(2)/sendmsg(2) on AF_INET sockets.
 *
 * Only AF_UNIX used to be routed here; an AF_INET socket got EOPNOTSUPP.
 * Chromium's DNS client sends its queries with sendto() but reads every reply
 * with recvmsg(), so each answer that arrived was left in the queue, the query
 * timed out and was re-sent, and pages failed with
 * DNS_PROBE_FINISHED_NO_INTERNET.
 *
 * The data goes through a staging buffer the size of one datagram: a UDP
 * payload always fits, and a stream socket is allowed to return (or accept)
 * less than was asked for. msg_control is not modelled -- no ancillary data
 * is ever delivered, so msg_controllen comes back 0. */
#define LINUX_INET_MSG_STAGING 1536u

typedef struct {
    uint64_t msg_name;
    uint32_t msg_namelen;
    uint32_t pad0;
    uint64_t msg_iov;
    uint64_t msg_iovlen;
    uint64_t msg_control;
    uint64_t msg_controllen;
    int32_t  msg_flags;
    uint32_t pad1;
} linux_msghdr_inet_t;

typedef struct {
    uint64_t iov_base;
    uint64_t iov_len;
} linux_iovec_inet_t;

static int64_t linux_inet_recvmsg(uint64_t fd, uint64_t msg_ptr, uint64_t flags)
{
    linux_msghdr_inet_t msg;
    if (copy_from_user(&msg, (const void *)(uintptr_t)msg_ptr, sizeof(msg)) != 0u) {
        return LINUX_EFAULT;
    }
    if (msg.msg_iovlen > 1024u) {
        return LINUX_EINVAL;
    }
    uint64_t want = 0u;
    for (uint64_t i = 0; i < msg.msg_iovlen && want < LINUX_INET_MSG_STAGING; ++i) {
        linux_iovec_inet_t iov;
        if (copy_from_user(&iov, (const void *)(uintptr_t)(msg.msg_iov + i * sizeof(iov)),
                           sizeof(iov)) != 0u) {
            return LINUX_EFAULT;
        }
        want += iov.iov_len;
    }
    if (want > LINUX_INET_MSG_STAGING) {
        want = LINUX_INET_MSG_STAGING;
    }

    uint8_t staging[LINUX_INET_MSG_STAGING];
    uint32_t src_ip = 0u;
    uint16_t src_port = 0u;
    int64_t got = (int64_t)syscall_socket_recvfrom((int32_t)fd, staging, (uint16_t)want,
                                                   &src_ip, &src_port);
    if (got == 0 && want != 0u && (flags & 0x40u /* MSG_DONTWAIT */) != 0u &&
        syscall_socket_get_type((int32_t)fd) == (int32_t)LINUX_SOCK_DGRAM) {
        return LINUX_EAGAIN;
    }
    if (got < 0) {
        return got;
    }

    uint64_t done = 0u;
    for (uint64_t i = 0; i < msg.msg_iovlen && done < (uint64_t)got; ++i) {
        linux_iovec_inet_t iov;
        if (copy_from_user(&iov, (const void *)(uintptr_t)(msg.msg_iov + i * sizeof(iov)),
                           sizeof(iov)) != 0u) {
            return LINUX_EFAULT;
        }
        uint64_t n = (uint64_t)got - done;
        if (n > iov.iov_len) n = iov.iov_len;
        if (n != 0u && copy_to_user((void *)(uintptr_t)iov.iov_base, staging + done, n) != 0u) {
            return LINUX_EFAULT;
        }
        done += n;
    }

    uint32_t namelen = 0u;
    if (msg.msg_name != 0u && msg.msg_namelen >= sizeof(linux_sockaddr_in_t)) {
        linux_sockaddr_in_t addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = LINUX_AF_INET;
        addr.sin_port = linux_be16_to_host(src_port);
        addr.sin_addr = src_ip;
        if (copy_to_user((void *)(uintptr_t)msg.msg_name, &addr, sizeof(addr)) != 0u) {
            return LINUX_EFAULT;
        }
        namelen = (uint32_t)sizeof(addr);
    }
    msg.msg_namelen = namelen;
    msg.msg_controllen = 0u;
    msg.msg_flags = 0;
    if (copy_to_user((void *)(uintptr_t)msg_ptr, &msg, sizeof(msg)) != 0u) {
        return LINUX_EFAULT;
    }
    return (int64_t)done;
}

static int64_t linux_inet_sendmsg(uint64_t fd, uint64_t msg_ptr, uint64_t flags)
{
    linux_msghdr_inet_t msg;
    if (copy_from_user(&msg, (const void *)(uintptr_t)msg_ptr, sizeof(msg)) != 0u) {
        return LINUX_EFAULT;
    }
    if (msg.msg_iovlen > 1024u) {
        return LINUX_EINVAL;
    }
    uint8_t staging[LINUX_INET_MSG_STAGING];
    uint64_t len = 0u;
    for (uint64_t i = 0; i < msg.msg_iovlen && len < LINUX_INET_MSG_STAGING; ++i) {
        linux_iovec_inet_t iov;
        if (copy_from_user(&iov, (const void *)(uintptr_t)(msg.msg_iov + i * sizeof(iov)),
                           sizeof(iov)) != 0u) {
            return LINUX_EFAULT;
        }
        uint64_t n = iov.iov_len;
        if (n > LINUX_INET_MSG_STAGING - len) n = LINUX_INET_MSG_STAGING - len;
        if (n != 0u && copy_from_user(staging + len, (const void *)(uintptr_t)iov.iov_base, n) != 0u) {
            return LINUX_EFAULT;
        }
        len += n;
    }
    uint32_t dst_ip = 0u;
    uint16_t dst_port = 0u;
    if (msg.msg_name != 0u && msg.msg_namelen != 0u) {
        int64_t rc = linux_copy_sockaddr_in(msg.msg_name, &dst_ip, &dst_port);
        if (rc < 0) {
            return rc;
        }
    }
    if (syscall_socket_get_type((int32_t)fd) == (int32_t)LINUX_SOCK_STREAM) {
        int64_t sent = (int64_t)syscall_socket_send((int32_t)fd, staging, (uint16_t)len);
        return linux_send_result_sigpipe(sent, flags);
    }
    return (int64_t)syscall_socket_sendto((int32_t)fd, staging, (uint16_t)len,
                                          dst_ip, dst_port);
}

static int64_t linux_socket_setsockopt(uint64_t fd, uint64_t level,
                                       uint64_t option, uint64_t value_ptr,
                                       uint64_t value_len)
{
    if (value_ptr == 0u || value_len == 0u) {
        return LINUX_EINVAL;
    }
    /* Options we do not model but that real Linux programs set and treat a
     * failure as fatal (crashpad wants SO_PASSCRED; libc / curl / node set
     * SO_SNDBUF/SO_RCVBUF/SO_REUSEPORT/timeouts). Accept as a no-op -- Linux
     * itself silently clamps most of these anyway. */
    if (level != LINUX_SOL_SOCKET) {
        return 0;
    }
    /* SO_PASSCRED on an AF_UNIX endpoint is real state, not a no-op: it is
     * what makes recvmsg() attach the SCM_CREDENTIALS control message that
     * Crashpad's socket protocol requires. */
    if (option == LINUX_SO_PASSCRED && unix_socket_fd_in_range((int32_t)fd)) {
        int32_t on = 0;
        if (value_len >= sizeof(int32_t) &&
            copy_from_user(&on, (const void *)(uintptr_t)value_ptr,
                           sizeof(on)) != 0u) {
            return LINUX_EFAULT;
        }
        (void)unix_socket_set_passcred((int32_t)fd, on != 0);
        return 0;
    }
    switch (option) {
        case LINUX_SO_PASSCRED:
        case LINUX_SO_PASSSEC:
        case LINUX_SO_SNDBUF:
        case LINUX_SO_RCVBUF:
        case LINUX_SO_BROADCAST:
        case LINUX_SO_REUSEPORT:
        case LINUX_SO_LINGER:
        case LINUX_SO_SNDTIMEO:
        case LINUX_SO_RCVTIMEO:
            return 0;
        default:
            break;
    }
    if (value_len < sizeof(int32_t)) {
        return LINUX_EINVAL;
    }
    int32_t value = 0;
    if (copy_from_user(&value, (const void *)(uintptr_t)value_ptr,
                       sizeof(value)) != 0u) {
        return LINUX_EFAULT;
    }
    int32_t mapped_option = 0;
    switch (option) {
        case LINUX_SO_REUSEADDR: mapped_option = 1; break;
        case LINUX_SO_KEEPALIVE: mapped_option = 2; break;
        default: return 0; /* accept-and-ignore rather than ENOPROTOOPT */
    }
    return (int64_t)syscall_socket_set_option((int32_t)fd, (int32_t)level,
                                              mapped_option, value);
}

static int64_t linux_socket_getsockopt(uint64_t fd, uint64_t level,
                                       uint64_t option, uint64_t value_ptr,
                                       uint64_t value_len_ptr)
{
    if (value_ptr == 0u || value_len_ptr == 0u) {
        return LINUX_EFAULT;
    }
    int32_t value_len = 0;
    if (copy_from_user(&value_len, (const void *)(uintptr_t)value_len_ptr,
                       sizeof(value_len)) != 0u) {
        return LINUX_EFAULT;
    }
    if (value_len < (int32_t)sizeof(int32_t)) {
        return LINUX_EINVAL;
    }
    if (level != LINUX_SOL_SOCKET) {
        return LINUX_ENOPROTOOPT;
    }

    /* SO_PEERCRED: struct ucred { int32 pid; uint32 uid; uint32 gid; }. We do
     * not track the peer, so report "pid 0, root:root" -- enough for callers
     * (crashpad, D-Bus-style libs) that only check the call succeeds. */
    if (option == LINUX_SO_PEERCRED) {
        int32_t ucred[3] = { 0, 0, 0 };
        /* Report the real pid on the other end when there is one: crashpad
         * learns the handler process it just spawned this way. uid/gid stay 0
         * -- this is a single-user system. */
        if (unix_socket_fd_in_range((int32_t)fd)) {
            (void)unix_socket_peer_pid((int32_t)fd, &ucred[0]);
            ucred[0] = process_pid_as_seen_by_current(ucred[0]);
        }
        int32_t want = value_len < (int32_t)sizeof(ucred) ? value_len
                                                          : (int32_t)sizeof(ucred);
        if (copy_to_user((void *)(uintptr_t)value_ptr, ucred, (uint64_t)want) != 0u) {
            return LINUX_EFAULT;
        }
        if (copy_to_user((void *)(uintptr_t)value_len_ptr, &want, sizeof(want)) != 0u) {
            return LINUX_EFAULT;
        }
        return 0;
    }

    /* Scalar options we synthesise a plausible answer for rather than failing. */
    {
        int32_t synth = 0;
        int have_synth = 1;
        switch (option) {
            case LINUX_SO_TYPE:     synth = 1;      break; /* SOCK_STREAM */
            case LINUX_SO_PROTOCOL: synth = 0;      break;
            case LINUX_SO_ERROR:    synth = 0;      break;
            case LINUX_SO_SNDBUF:   synth = 212992; break;
            case LINUX_SO_RCVBUF:   synth = 212992; break;
            case LINUX_SO_PASSCRED: synth = 0;      break;
            default:                have_synth = 0; break;
        }
        if (have_synth) {
            int32_t len = (int32_t)sizeof(synth);
            if (copy_to_user((void *)(uintptr_t)value_ptr, &synth, sizeof(synth)) != 0u ||
                copy_to_user((void *)(uintptr_t)value_len_ptr, &len, sizeof(len)) != 0u) {
                return LINUX_EFAULT;
            }
            return 0;
        }
    }

    int32_t mapped_option = 0;
    switch (option) {
        case LINUX_SO_REUSEADDR: mapped_option = 1; break;
        case LINUX_SO_KEEPALIVE: mapped_option = 2; break;
        default: return LINUX_ENOPROTOOPT;
    }
    int32_t value = 0;
    int32_t rc = syscall_socket_get_option((int32_t)fd, (int32_t)level,
                                           mapped_option, &value);
    if (rc < 0) {
        return (int64_t)rc;
    }
    if (copy_to_user((void *)(uintptr_t)value_ptr, &value,
                     sizeof(value)) != 0u) {
        return LINUX_EFAULT;
    }
    value_len = (int32_t)sizeof(value);
    if (copy_to_user((void *)(uintptr_t)value_len_ptr, &value_len,
                     sizeof(value_len)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Step 1 additions: resource introspection, procfs symlinks, affinity */
/* (TODO_Chromium_LinuxABI.md section 3.9)                            */
/* ------------------------------------------------------------------ */

/* Does `path` name anything at all -- file, device or directory? stat() has
 * always answered this by falling back to opendir(), because the pseudo
 * filesystems have no lookup that spans both. */
static bool linux_path_exists(const char *path)
{
    vfs_file_t vf;
    if (vfs_find_file(path, &vf)) {
        return true;
    }
    int32_t dir_handle = vfs_opendir(path);
    if (dir_handle < 0) {
        return false;
    }
    (void)vfs_closedir(dir_handle);
    return true;
}

static int64_t linux_readlink_common(const char *path, uint64_t buf,
                                     uint64_t bufsiz)
{
    if (bufsiz == 0u) {
        return LINUX_EINVAL;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)buf, bufsiz)) {
        return LINUX_EFAULT;
    }
    char target[256];
    uint64_t len = 0u;
    if (procfs_readlink(path, target, sizeof(target)) >= 0) {
        len = strlen(target);
    } else {
        /* Not a procfs link: ask the filesystems. Distinguishing the two
         * failure modes matters -- Chromium's ProcessSingleton reads
         * <user-data-dir>/SingletonLock and treats ENOENT as "no other
         * instance" (the normal first-run path) but any other errno as a
         * hard failure. */
        int32_t rc = vfs_readlink(path, target, (uint32_t)sizeof(target));
        if (rc < 0) {
            /* EINVAL ("exists, but is not a symlink") vs ENOENT is not a
             * detail: glibc's realpath() readlink()s every component of a
             * path and only tolerates EINVAL. Answering ENOENT for the
             * directory /tmp made realpath("/tmp/chromium") fail outright,
             * so Chromium decided --user-data-dir was unusable and fell back
             * to the default profile ("Failed To Create Data Directory"). */
            return linux_path_exists(path) ? LINUX_EINVAL : LINUX_ENOENT;
        }
        len = (uint64_t)rc;
    }
    if (len > bufsiz) {
        len = bufsiz;
    }
    if (copy_to_user_trusted((void *)(uintptr_t)buf, target, len) != 0u) {
        return LINUX_EFAULT;
    }
    return (int64_t)len; /* readlink() does NOT NUL-terminate the buffer. */
}

/* unlink(2)/unlinkat(2) on an absolute path. syscall_file_unlink() reports
 * every failure as EIO, including "there was nothing there" -- and callers do
 * read the errno: Chromium's ProcessSingleton removes stale lock files before
 * creating its own and logs anything but ENOENT as an error. */
static int64_t linux_unlink_resolved(const char *path)
{
    if (!linux_path_exists(path)) {
        return LINUX_ENOENT;
    }
    return (int64_t)syscall_file_unlink(path);
}

/* mkdir(2)/open(O_CREAT) carry a mode the VFS create hooks cannot take, so it
 * is applied to the finished node here. Filesystems without per-node modes
 * just report false and keep their fixed defaults. */
static void linux_apply_create_mode(const char *path, uint64_t mode)
{
    (void)vfs_set_mode(path, (uint32_t)(mode & 07777u));
}

/* symlink(2) / symlinkat(2). Only the writable tmpfs trees can hold links;
 * everywhere else this reports EPERM, which is what Linux returns for a
 * filesystem that does not support them. */
static int64_t linux_symlink_at(uint64_t target_ptr, uint64_t dirfd,
                                uint64_t linkpath_ptr)
{
    char target[256];
    char linkpath[256];
    int64_t rc = linux_copy_cstring(target, sizeof(target),
                                    (const char *)(uintptr_t)target_ptr);
    if (rc < 0) return rc;
    rc = linux_copy_cstring(linkpath, sizeof(linkpath),
                            (const char *)(uintptr_t)linkpath_ptr);
    if (rc < 0) return rc;
    rc = linux_resolve_at(dirfd, linkpath, sizeof(linkpath));
    if (rc < 0) return rc;

    vfs_file_t vf;
    if (vfs_find_file(linkpath, &vf)) {
        return LINUX_EEXIST;
    }
    if (!vfs_symlink(target, linkpath)) {
        return LINUX_EPERM;
    }
    return 0;
}

static int64_t linux_readlink(uint64_t path_ptr, uint64_t buf, uint64_t bufsiz)
{
    char path[256];
    int64_t rc = linux_copy_cstring(path, sizeof(path),
                                    (const char *)(uintptr_t)path_ptr);
    if (rc < 0) return rc;
    rc = linux_resolve_path(path, sizeof(path));
    if (rc < 0) return rc;
    return linux_readlink_common(path, buf, bufsiz);
}

static int64_t linux_readlinkat(uint64_t dirfd, uint64_t path_ptr,
                                uint64_t buf, uint64_t bufsiz)
{
    char path[256];
    int64_t rc = linux_copy_cstring(path, sizeof(path),
                                    (const char *)(uintptr_t)path_ptr);
    if (rc < 0) return rc;
    rc = linux_resolve_at(dirfd, path, sizeof(path));
    if (rc < 0) return rc;
    return linux_readlink_common(path, buf, bufsiz);
}

/* struct statx / struct statx_timestamp, matching the Linux uapi layout
 * (256 bytes total on x86_64). */
typedef struct {
    int64_t tv_sec;
    uint32_t tv_nsec;
    int32_t __reserved;
} linux_statx_timestamp_t;

typedef struct {
    uint32_t stx_mask;
    uint32_t stx_blksize;
    uint64_t stx_attributes;
    uint32_t stx_nlink;
    uint32_t stx_uid;
    uint32_t stx_gid;
    uint16_t stx_mode;
    uint16_t __spare0;
    uint64_t stx_ino;
    uint64_t stx_size;
    uint64_t stx_blocks;
    uint64_t stx_attributes_mask;
    linux_statx_timestamp_t stx_atime;
    linux_statx_timestamp_t stx_btime;
    linux_statx_timestamp_t stx_ctime;
    linux_statx_timestamp_t stx_mtime;
    uint32_t stx_rdev_major;
    uint32_t stx_rdev_minor;
    uint32_t stx_dev_major;
    uint32_t stx_dev_minor;
    uint64_t stx_mnt_id;
    uint64_t __spare2;
    uint64_t __spare3[12];
} linux_statx_t;

#define LINUX_STATX_BASIC_STATS 0x000007ffu
#define LINUX_AT_EMPTY_PATH      0x1000u

static int64_t linux_statx(uint64_t dirfd, uint64_t path_ptr, uint64_t flags,
                           uint64_t mask, uint64_t statxbuf_ptr)
{
    (void)mask;
    if (statxbuf_ptr == 0u ||
        !process_user_buffer_is_valid((void *)(uintptr_t)statxbuf_ptr,
                                      sizeof(linux_statx_t))) {
        return LINUX_EFAULT;
    }

    linux_stat64_t st;
    memset(&st, 0, sizeof(st));

    if ((flags & LINUX_AT_EMPTY_PATH) != 0u) {
        char empty[1];
        if (copy_from_user(empty, (const void *)(uintptr_t)path_ptr, 1u) == 0u &&
            empty[0] == '\0') {
            int64_t rc = linux_fill_stat_for_fd((int32_t)dirfd, &st);
            if (rc != 0) {
                return rc;
            }
        } else {
            return LINUX_ENOTSUP;
        }
    } else {
        char path[256];
        int64_t rc = linux_copy_cstring(path, sizeof(path),
                                        (const char *)(uintptr_t)path_ptr);
        if (rc < 0) return rc;
        rc = linux_resolve_at(dirfd, path, sizeof(path));
        if (rc < 0) return rc;
        rc = linux_resolve_path(path, sizeof(path));
        if (rc < 0) return rc;

        rc = linux_fill_stat_for_path(path, &st);
        if (rc != 0) {
            return rc;
        }
    }

    linux_statx_t sx;
    memset(&sx, 0, sizeof(sx));
    sx.stx_mask = LINUX_STATX_BASIC_STATS;
    sx.stx_blksize = (uint32_t)st.st_blksize;
    sx.stx_nlink = (uint32_t)st.st_nlink;
    sx.stx_uid = st.st_uid;
    sx.stx_gid = st.st_gid;
    sx.stx_mode = (uint16_t)st.st_mode;
    sx.stx_ino = st.st_ino;
    sx.stx_size = (uint64_t)st.st_size;
    sx.stx_blocks = (uint64_t)st.st_blocks;
    sx.stx_atime.tv_sec = st.st_atime;
    sx.stx_btime.tv_sec = st.st_ctime;
    sx.stx_ctime.tv_sec = st.st_ctime;
    sx.stx_mtime.tv_sec = st.st_mtime;

    if (copy_to_user_trusted((void *)(uintptr_t)statxbuf_ptr, &sx, sizeof(sx)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

typedef struct {
    int64_t uptime;
    uint64_t loads[3];
    uint64_t totalram;
    uint64_t freeram;
    uint64_t sharedram;
    uint64_t bufferram;
    uint64_t totalswap;
    uint64_t freeswap;
    uint16_t procs;
    uint16_t pad;
    uint64_t totalhigh;
    uint64_t freehigh;
    uint32_t mem_unit;
    uint8_t _f[8];
} linux_sysinfo_t;

static int64_t linux_sysinfo(uint64_t info_ptr)
{
    if (info_ptr == 0u ||
        !process_user_buffer_is_valid((void *)(uintptr_t)info_ptr,
                                      sizeof(linux_sysinfo_t))) {
        return LINUX_EFAULT;
    }
    linux_sysinfo_t info;
    memset(&info, 0, sizeof(info));
    info.uptime = (int64_t)(timer_ticks() / (timer_hz() != 0u ? timer_hz() : 60u));
    info.totalram = get_total_memory_pages() * PAGE_SIZE;
    uint64_t free_bytes = get_free_memory();
    info.freeram = free_bytes < info.totalram ? free_bytes : info.totalram;
    info.procs = (uint16_t)process_get_capacity();
    info.mem_unit = 1u;
    if (copy_to_user_trusted((void *)(uintptr_t)info_ptr, &info, sizeof(info)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

typedef struct {
    int64_t f_type;
    int64_t f_bsize;
    uint64_t f_blocks;
    uint64_t f_bfree;
    uint64_t f_bavail;
    uint64_t f_files;
    uint64_t f_ffree;
    struct { int32_t val[2]; } f_fsid;
    int64_t f_namelen;
    int64_t f_frsize;
    int64_t f_flags;
    int64_t f_spare[4];
} linux_statfs64_t;

#define LINUX_TMPFS_MAGIC 0x01021994L
#define LINUX_ISO9660_MAGIC 0x9660L

static void linux_statfs_fill(linux_statfs64_t *out)
{
    memset(out, 0, sizeof(*out));
    out->f_type = LINUX_TMPFS_MAGIC;
    out->f_bsize = 4096;
    uint64_t total_blocks = get_total_memory_pages() * (PAGE_SIZE / 4096u);
    uint64_t free_blocks = get_free_memory() / 4096u;
    out->f_blocks = total_blocks;
    out->f_bfree = free_blocks < total_blocks ? free_blocks : total_blocks;
    out->f_bavail = out->f_bfree;
    out->f_files = 65536u;
    out->f_ffree = 65536u;
    out->f_namelen = 255;
    out->f_frsize = 4096;
}

static int64_t linux_statfs(uint64_t path_ptr, uint64_t buf_ptr)
{
    (void)path_ptr;
    if (buf_ptr == 0u ||
        !process_user_buffer_is_valid((void *)(uintptr_t)buf_ptr,
                                      sizeof(linux_statfs64_t))) {
        return LINUX_EFAULT;
    }
    linux_statfs64_t out;
    linux_statfs_fill(&out);
    if (copy_to_user_trusted((void *)(uintptr_t)buf_ptr, &out, sizeof(out)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t linux_fstatfs(uint64_t fd, uint64_t buf_ptr)
{
    (void)fd;
    return linux_statfs(0u, buf_ptr);
}

/* Does `pid`, as the sched_*affinity and sched_getparam family use it, name a
 * task the caller may ask about?
 *
 * 0 means "me". Anything else is a *thread* id in Linux terms, and glibc leans
 * on that: pthread_getattr_np() calls __pthread_getaffinity_np(), which issues
 * sched_getaffinity(pd->tid, ...) for the thread being asked about -- never the
 * thread-group id. Comparing the argument against process_get_current_pid(),
 * which answers with the address-space owner, therefore rejected every call
 * made on a non-main thread with ESRCH.
 *
 * That one errno was enough to stop Chromium rendering a page. glibc's
 * pthread_getattr_np() returns the affinity error to its caller, V8's
 * base::Stack::GetStackStart() cannot then learn the thread's stack bounds,
 * and Heap::CollectGarbage() opens with
 * CHECK(isolate_->IsOnCentralStack()) -- so the first garbage collection on
 * any renderer thread aborted the browser:
 *
 *   Check failed: isolate_->IsOnCentralStack().
 *   #4 v8::internal::Heap::CollectGarbage(...)
 *   #9 v8::internal::Runtime_AllocateInYoungGeneration(...)
 *
 * See Docs/Others/TODO_Chromium_LinuxABI.md section 10.-5. */
static int linux_sched_pid_is_self(uint64_t pid)
{
    if (pid == 0u) {
        return 1;
    }
    int32_t target = (int32_t)pid;
    if (target < 0) {
        return 0;
    }
    if (target == process_get_current_tid() ||
        target == process_get_current_pid()) {
        return 1;
    }
    /* A sibling thread: same address-space owner. */
    int32_t owner = process_memory_owner_pid_of(target);
    return (owner >= 0 && owner == process_get_current_pid());
}

static int64_t linux_sched_getaffinity(uint64_t pid, uint64_t cpusetsize,
                                       uint64_t mask_ptr)
{
    if (!linux_sched_pid_is_self(pid)) {
        return LINUX_ESRCH;
    }
    if (cpusetsize == 0u || mask_ptr == 0u) {
        return LINUX_EINVAL;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)mask_ptr, cpusetsize)) {
        return LINUX_EFAULT;
    }
    uint32_t cpu_count = smp_get_cpu_count();
    if (cpu_count == 0u) {
        cpu_count = 1u;
    }
    uint8_t mask[128];
    uint64_t to_write = cpusetsize < sizeof(mask) ? cpusetsize : sizeof(mask);
    memset(mask, 0, to_write);
    for (uint32_t cpu = 0; cpu < cpu_count && (cpu / 8u) < to_write; ++cpu) {
        mask[cpu / 8u] |= (uint8_t)(1u << (cpu % 8u));
    }
    if (copy_to_user_trusted((void *)(uintptr_t)mask_ptr, mask, to_write) != 0u) {
        return LINUX_EFAULT;
    }
    return (int64_t)to_write;
}

static int64_t linux_sched_setaffinity(uint64_t pid, uint64_t cpusetsize,
                                       uint64_t mask_ptr)
{
    if (!linux_sched_pid_is_self(pid)) {
        return LINUX_ESRCH;
    }
    (void)cpusetsize;
    (void)mask_ptr;
    /* Scheduler does not support per-thread CPU pinning yet; accept the
     * request (this process still runs on whichever CPU the scheduler
     * picks) rather than failing callers that only want to *reduce* their
     * affinity mask. */
    return 0;
}

#define LINUX_MADV_DONTNEED 4u
#define LINUX_MADV_REMOVE    9u
#define LINUX_MADV_FREE      8u

/* A page of zeroes to copy out of. Static rather than a stack buffer: it is
 * read-only, so it is safe to share across CPUs, and 4 KiB is far too much
 * kernel stack. */

static int64_t linux_madvise(uint64_t addr, uint64_t length, uint64_t advice)
{
    /* MADV_NORMAL/WILLNEED/RANDOM/... really are hints: nothing to do.
     *
     * MADV_DONTNEED / MADV_FREE / MADV_REMOVE are not. On Linux the range
     * reads back as zeroes afterwards, and callers rely on that rather than
     * zeroing it themselves: PartitionAlloc decommits a span with madvise and
     * then treats the memory as already-zeroed on recommit
     * (DecommittedMemoryIsAlwaysZeroed() is true on Linux), so calloc() hands
     * out whatever was left behind. That is how a no-op madvise turned into
     * libxcb receiving a xcb_connection_t with a garbage ->setup pointer and
     * free()ing it.
     *
     * The pages are discarded rather than zeroed in place, which is both what
     * Linux does and the only safe thing to do: PartitionAlloc decommits a
     * span by mprotect(PROT_NONE) *and then* madvise(MADV_DONTNEED), so by the
     * time this runs the pages are frequently not writable, and zeroing them
     * through the user mapping faulted in kernel mode on a present read-only
     * page -- which used to panic the machine in the middle of Chromium's
     * startup. Dropping the mapping instead hands the frames back to the
     * allocator and lets the next access demand-zero, so the caller still
     * reads what Linux would have given it. Pages not currently present are
     * left alone; they already fault in as zero.
     *
     * One consequence worth naming: a discarded page is absent, and the
     * demand-zero path maps an absent user page writable without consulting
     * the protection the program last asked for. A span that PartitionAlloc
     * decommitted with mprotect(PROT_NONE) + madvise() therefore reads back as
     * accessible zeroes rather than faulting, so a use-after-free that
     * PROT_NONE would have caught goes unnoticed. That is already true of
     * every never-touched page in a PROT_NONE reservation here (see the
     * demand-paging comment in Arch/x86_64/cpu/IDT_Main.c); tracking
     * per-region protection is what would fix both at once.
     * See Docs/Others/TODO_Chromium_LinuxABI.md section 10.-6. */
    if (advice != LINUX_MADV_DONTNEED && advice != LINUX_MADV_FREE &&
        advice != LINUX_MADV_REMOVE) {
        (void)addr; (void)length;
        return 0;
    }
    if (length == 0u) {
        return 0;
    }
    if ((addr & (PAGE_SIZE - 1u)) != 0u) {
        return LINUX_EINVAL;
    }

    uint64_t cr3 = process_get_current_cr3();
    int32_t self = process_get_current_pid();
    if (cr3 == 0u || self < 0) {
        return 0;
    }
    uint64_t end = addr + ((length + PAGE_SIZE - 1u) & ~(uint64_t)(PAGE_SIZE - 1u));
    if (end <= addr) {
        return LINUX_EINVAL;
    }
    /* Only the mmap arena. That is where the mappings whose zeroing actually
     * matters live -- PartitionAlloc's pools and glibc's thread stacks both
     * come from mmap() -- and staying out of the code/heap/stack windows keeps
     * this away from eagerly-mapped library images, which carry no filemap
     * record to recognise them by and must not be blanked. Outside the arena
     * the call stays the hint it always was. */
    if (addr < USER_MMAP_BASE || end > USER_MMAP_LIMIT) {
        return 0;
    }
    /* Unmapped in runs, not page by page: paging_unmap_range() ends with a
     * TLB shootdown across every CPU on this address space, and a decommitted
     * PartitionAlloc span is hundreds of contiguous pages. One shootdown per
     * run instead of per page is the difference between a hint and a stall. */
    uint64_t run_start = 0u;
    uint64_t run_end = 0u;
    for (uint64_t page = addr; page <= end; page += PAGE_SIZE) {
        int eligible = 0;
        if (page < end && paging_virt_to_phys(cr3, page) != 0u) {
            /* Private anonymous pages only. A file-backed or shared page keeps
             * its contents on Linux -- MADV_DONTNEED just drops the cached
             * copy and the next access reads the file (or the shared object)
             * again. Discarding one of those would lose Chromium's mapped
             * .pak/ICU data and its shared-memory regions for good. */
            eligible = !filemap_addr_is_file_backed(self, page) &&
                       !shared_memory_addr_is_mapped(page);
        }

        if (eligible) {
            if (run_end == 0u) {
                run_start = page;
            }
            run_end = page + PAGE_SIZE;
            continue;
        }
        if (run_end != 0u) {
            (void)paging_unmap_range(cr3, run_start, run_end - run_start);
            run_end = 0u;
        }
    }
    return 0;
}

static int64_t linux_mincore(uint64_t addr, uint64_t length, uint64_t vec_ptr)
{
    if (length == 0u) {
        return 0;
    }
    uint64_t page_count = (length + PAGE_SIZE - 1u) / PAGE_SIZE;
    if (!process_user_buffer_is_valid((void *)(uintptr_t)vec_ptr, page_count)) {
        return LINUX_EFAULT;
    }
    uint8_t chunk[256];
    memset(chunk, 1, sizeof(chunk)); /* Report everything resident. */
    uint64_t written = 0;
    while (written < page_count) {
        uint64_t want = page_count - written;
        if (want > sizeof(chunk)) want = sizeof(chunk);
        if (copy_to_user_trusted((uint8_t *)(uintptr_t)vec_ptr + written,
                                 chunk, want) != 0u) {
            return LINUX_EFAULT;
        }
        written += want;
    }
    (void)addr;
    return 0;
}

static int64_t linux_sendfile(uint64_t out_fd, uint64_t in_fd, uint64_t offset_ptr,
                              uint64_t count)
{
    int64_t saved_offset = -1;
    if (offset_ptr != 0u) {
        int64_t requested = 0;
        if (copy_from_user(&requested, (const void *)(uintptr_t)offset_ptr,
                           sizeof(requested)) != 0u) {
            return LINUX_EFAULT;
        }
        saved_offset = syscall_file_seek((int32_t)in_fd, 0, LINUX_SEEK_CUR);
        if (saved_offset < 0 ||
            syscall_file_seek((int32_t)in_fd, requested, LINUX_SEEK_SET) < 0) {
            return LINUX_EBUSY;
        }
    }
    uint8_t chunk[4096];
    uint64_t total = 0;
    while (total < count) {
        uint64_t want = count - total;
        if (want > sizeof(chunk)) want = sizeof(chunk);
        int64_t got = syscall_file_read((int32_t)in_fd, chunk, want);
        if (got <= 0) break;
        /* out_fd is a kernel fd, not a userspace fd; use the fd-table
         * writer directly rather than the syscall_write() wrapper (which
         * validates `buf` as a *userspace* pointer and would reject this
         * kernel-side staging buffer). */
        int64_t put = syscall_file_write((int32_t)out_fd, chunk, (uint64_t)got);
        if (put < 0) {
            total = total != 0u ? total : (uint64_t)put;
            break;
        }
        total += (uint64_t)put;
        if (put < got) break;
    }
    if (offset_ptr != 0u) {
        int64_t new_pos = syscall_file_seek((int32_t)in_fd, 0, LINUX_SEEK_CUR);
        if (new_pos >= 0) {
            (void)copy_to_user_trusted((void *)(uintptr_t)offset_ptr, &new_pos,
                                       sizeof(new_pos));
        }
        if (saved_offset >= 0) {
            (void)syscall_file_seek((int32_t)in_fd, saved_offset, LINUX_SEEK_SET);
        }
    }
    return (int64_t)total;
}

typedef struct {
    int64_t sec;
    int64_t usec;
} linux_timeval_t;

typedef struct {
    linux_timeval_t it_interval;
    linux_timeval_t it_value;
} linux_itimerval_t;

static int64_t linux_getitimer(uint64_t which, uint64_t curr_value_ptr)
{
    (void)which;
    /* No per-process interval-timer/SIGALRM delivery yet; report
     * "disarmed" rather than failing outright. */
    linux_itimerval_t value;
    memset(&value, 0, sizeof(value));
    if (curr_value_ptr != 0u &&
        copy_to_user_trusted((void *)(uintptr_t)curr_value_ptr, &value,
                             sizeof(value)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t linux_setitimer(uint64_t which, uint64_t new_value_ptr,
                               uint64_t old_value_ptr)
{
    (void)which;
    (void)new_value_ptr;
    if (old_value_ptr != 0u) {
        linux_itimerval_t value;
        memset(&value, 0, sizeof(value));
        if (copy_to_user_trusted((void *)(uintptr_t)old_value_ptr, &value,
                                 sizeof(value)) != 0u) {
            return LINUX_EFAULT;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Section 3.8 / section 4 additions: self-contained syscalls that a    */
/* real glibc-linked binary (and Chromium) exercise at startup. None    */
/* of these touch the scheduler/paging fast paths - they are wrappers   */
/* over primitives that already exist, or well-defined no-ops.          */
/* ------------------------------------------------------------------ */

#define LINUX_EINTR (-4LL)

#define LINUX_O_CLOEXEC   0x80000u
#define LINUX_O_NONBLOCK  0x800u

/* pread64/pwrite64: like read/write but at an explicit offset and,
 * critically, WITHOUT disturbing the fd's current file position. glibc's
 * stdio, dynamic loader and Chromium's file layer all use these heavily. */
static int64_t linux_pread64(uint64_t fd, uint64_t buf, uint64_t count,
                             uint64_t offset)
{
    if (count == 0u) {
        return 0;
    }
    if (count > LINUX_MAX_IO_BYTES) {
        count = LINUX_MAX_IO_BYTES;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)buf, count)) {
        return LINUX_EFAULT;
    }
    int64_t saved = syscall_file_seek((int32_t)fd, 0, LINUX_SEEK_CUR);
    if (saved < 0) {
        return saved;
    }
    if (syscall_file_seek((int32_t)fd, (int64_t)offset, LINUX_SEEK_SET) < 0) {
        return LINUX_EINVAL;
    }
    int64_t rc = syscall_file_read((int32_t)fd, (uint8_t *)(uintptr_t)buf, count);
    (void)syscall_file_seek((int32_t)fd, saved, LINUX_SEEK_SET);
    return rc;
}

static int64_t linux_pwrite64(uint64_t fd, uint64_t buf, uint64_t count,
                              uint64_t offset)
{
    if (count == 0u) {
        return 0;
    }
    if (count > LINUX_MAX_IO_BYTES) {
        count = LINUX_MAX_IO_BYTES;
    }
    if (!process_user_buffer_is_valid((const void *)(uintptr_t)buf, count)) {
        return LINUX_EFAULT;
    }
    int64_t saved = syscall_file_seek((int32_t)fd, 0, LINUX_SEEK_CUR);
    if (saved < 0) {
        return saved;
    }
    if (syscall_file_seek((int32_t)fd, (int64_t)offset, LINUX_SEEK_SET) < 0) {
        return LINUX_EINVAL;
    }
    uint8_t chunk[4096];
    uint64_t total = 0;
    int64_t error = 0;
    while (total < count) {
        uint64_t want = count - total;
        if (want > sizeof(chunk)) want = sizeof(chunk);
        if (copy_from_user_trusted(chunk,
                                   (const uint8_t *)(uintptr_t)buf + total,
                                   want) != 0u) {
            error = LINUX_EFAULT;
            break;
        }
        int64_t put = syscall_file_write((int32_t)fd, chunk, want);
        if (put < 0) {
            error = put;
            break;
        }
        total += (uint64_t)put;
        if ((uint64_t)put < want) break;
    }
    (void)syscall_file_seek((int32_t)fd, saved, LINUX_SEEK_SET);
    return total != 0u ? (int64_t)total : error;
}

static int64_t linux_pipe2(uint64_t fds_ptr, uint64_t flags)
{
    if (!process_user_buffer_is_valid((void *)(uintptr_t)fds_ptr,
                                      sizeof(int32_t) * 2u)) {
        return LINUX_EFAULT;
    }
    int32_t fds[2];
    int32_t rc = syscall_file_pipe(fds);
    if (rc < 0) {
        return rc;
    }
    if ((flags & LINUX_O_CLOEXEC) != 0u) {
        (void)syscall_file_set_descriptor_flags(fds[0], LINUX_FD_CLOEXEC);
        (void)syscall_file_set_descriptor_flags(fds[1], LINUX_FD_CLOEXEC);
    }
    if ((flags & LINUX_O_NONBLOCK) != 0u) {
        int32_t f0 = syscall_file_get_status_flags(fds[0]);
        int32_t f1 = syscall_file_get_status_flags(fds[1]);
        if (f0 >= 0) (void)syscall_file_set_status_flags(fds[0], (uint32_t)f0 | 0x0800u);
        if (f1 >= 0) (void)syscall_file_set_status_flags(fds[1], (uint32_t)f1 | 0x0800u);
    }
    if (copy_to_user_trusted((void *)(uintptr_t)fds_ptr, fds, sizeof(fds)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t linux_dup3(uint64_t oldfd, uint64_t newfd, uint64_t flags)
{
    if (oldfd == newfd) {
        return LINUX_EINVAL;
    }
    int64_t rc = (int64_t)syscall_file_dup2((int32_t)oldfd, (int32_t)newfd);
    if (rc < 0) {
        return rc;
    }
    if ((flags & LINUX_O_CLOEXEC) != 0u) {
        (void)syscall_file_set_descriptor_flags((int32_t)newfd, LINUX_FD_CLOEXEC);
    }
    return rc;
}

static int64_t linux_faccessat(uint64_t dirfd, uint64_t path_ptr, uint64_t mode,
                               uint64_t flags)
{
    (void)flags;
    char path[256];
    int64_t rc = linux_copy_cstring(path, sizeof(path),
                                    (const char *)(uintptr_t)path_ptr);
    if (rc < 0) return rc;
    rc = linux_resolve_at(dirfd, path, sizeof(path));
    if (rc < 0) return rc;
    return syscall_access(path, (int32_t)mode);
}

/* getsockname/getpeername - fills a sockaddr_in from the socket table
 * (TODO_Chromium_LinuxABI.md section 4). AF_UNIX endpoints report an
 * unnamed address (family + zero-length path), matching Linux for
 * socketpair()/unbound Unix sockets. */
static int64_t linux_getsockname_common(uint64_t fd, uint64_t addr_ptr,
                                        uint64_t addr_len_ptr, int want_peer)
{
    if (addr_ptr == 0u || addr_len_ptr == 0u) {
        return LINUX_EFAULT;
    }
    int32_t caller_len = 0;
    if (copy_from_user(&caller_len, (const void *)(uintptr_t)addr_len_ptr,
                       sizeof(caller_len)) != 0u) {
        return LINUX_EFAULT;
    }
    if (caller_len < 0) {
        return LINUX_EINVAL;
    }

    if (unix_socket_fd_in_range((int32_t)fd)) {
        linux_sockaddr_un_t un;
        memset(&un, 0, sizeof(un));
        un.sun_family = LINUX_AF_UNIX;
        int32_t out_len = (int32_t)offsetof(linux_sockaddr_un_t, sun_path);
        uint32_t copy = (uint32_t)((caller_len < out_len) ? caller_len : out_len);
        if (copy != 0u &&
            copy_to_user((void *)(uintptr_t)addr_ptr, &un, copy) != 0u) {
            return LINUX_EFAULT;
        }
        if (copy_to_user((void *)(uintptr_t)addr_len_ptr, &out_len,
                         sizeof(out_len)) != 0u) {
            return LINUX_EFAULT;
        }
        return 0;
    }

    syscall_socket_info_t info;
    if (syscall_socket_get_info((int32_t)fd, &info) != 0) {
        return LINUX_EBADF;
    }
    linux_sockaddr_in_t addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = LINUX_AF_INET;
    if (want_peer) {
        addr.sin_port = linux_be16_to_host(info.remote_port);
        addr.sin_addr = info.remote_ip;
    } else {
        addr.sin_port = linux_be16_to_host(info.local_port);
        addr.sin_addr = info.local_ip;
    }
    int32_t out_len = (int32_t)sizeof(addr);
    uint32_t copy = (uint32_t)((caller_len < out_len) ? caller_len : out_len);
    if (copy != 0u &&
        copy_to_user((void *)(uintptr_t)addr_ptr, &addr, copy) != 0u) {
        return LINUX_EFAULT;
    }
    if (copy_to_user((void *)(uintptr_t)addr_len_ptr, &out_len,
                     sizeof(out_len)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t linux_clock_nanosleep(uint64_t clockid, uint64_t flags,
                                     uint64_t request_ptr, uint64_t remain_ptr,
                                     int *should_switch_out)
{
    (void)clockid;
    struct {
        int64_t sec;
        int64_t nsec;
    } req;
    if (copy_from_user(&req, (const void *)(uintptr_t)request_ptr,
                       sizeof(req)) != 0u) {
        return LINUX_EFAULT;
    }
    if (req.sec < 0 || req.nsec < 0 || req.nsec >= 1000000000LL) {
        return LINUX_EINVAL;
    }
    uint64_t target_ms = (uint64_t)req.sec * 1000u +
                         ((uint64_t)req.nsec + 999999u) / 1000000u;

    /* TIMER_ABSTIME (bit 0): `request` is an absolute time on `clockid`.
     * Convert to a relative delay using the same monotonic base the rest
     * of the compat layer uses. */
    if ((flags & 1u) != 0u) {
        uint32_t hz = timer_hz();
        if (hz == 0u) hz = 60u;
        uint64_t now_ms = (timer_ticks() * 1000ULL) / hz;
        target_ms = (target_ms > now_ms) ? (target_ms - now_ms) : 0u;
    }

    if (target_ms > 0u && process_sleep_current_ms(target_ms) == 0) {
        if (should_switch_out != NULL) {
            *should_switch_out = 1;
        }
    }
    /* We always sleep the whole interval (no early wakeup), so the
     * remaining-time output is zero when provided. */
    if (remain_ptr != 0u) {
        struct { int64_t sec; int64_t nsec; } zero = {0, 0};
        (void)copy_to_user_trusted((void *)(uintptr_t)remain_ptr, &zero,
                                   sizeof(zero));
    }
    return 0;
}

/* waitid(2) - Chromium's base::Process and glibc's posix_spawn helpers use
 * it in preference to wait4() (TODO_Chromium_LinuxABI.md section 4). */
static int64_t linux_waitid(uint64_t idtype, uint64_t id, uint64_t infop_ptr,
                            uint64_t options, uint64_t rusage_ptr,
                            int *should_switch_out, int *restart_out)
{
    if (rusage_ptr != 0u &&
        process_user_buffer_is_valid((void *)(uintptr_t)rusage_ptr, 144u)) {
        uint8_t zero[144];
        memset(zero, 0, sizeof(zero));
        (void)copy_to_user_trusted((void *)(uintptr_t)rusage_ptr, zero,
                                   sizeof(zero));
    }
    if (infop_ptr != 0u &&
        !process_user_buffer_is_valid((void *)(uintptr_t)infop_ptr, 128u)) {
        return LINUX_EFAULT;
    }

    int32_t wait_pid;
    switch (idtype) {
        case 0u: wait_pid = -1; break;             /* P_ALL  */
        case 1u: wait_pid = (int32_t)id; break;    /* P_PID  */
        case 2u: wait_pid = -1; break;             /* P_PGID - best effort */
        default: return LINUX_EINVAL;
    }

    int32_t exit_code = 0;
    int32_t term_signal = 0;
    int32_t nohang = (options & LINUX_WNOHANG) != 0u ? (int32_t)LINUX_WNOHANG : 0;
    int32_t child = process_waitpid_ex(wait_pid, &exit_code, nohang,
                                       &term_signal);
    if (child < 0) {
        return LINUX_ECHILD;
    }
    if (child == 0) {
        if ((options & LINUX_WNOHANG) == 0u) {
            /* Blocking waitid(): a child exists but has not exited. Park the
             * caller for a slice, then run the call again when it is next
             * scheduled. Returning 0 would report success with an untouched
             * siginfo_t, i.e. a child that has not exited read as one that
             * has -- the same defect fixed in linux_wait4(). */
            if (process_sleep_current_ms(LINUX_WAIT_POLL_SLICE_MS) == 0 &&
                should_switch_out != NULL) {
                *should_switch_out = 1;
            }
            if (restart_out != NULL) {
                *restart_out = 1;
            }
            return 0;
        }
        /* WNOHANG with no ready child: POSIX says zero-fill si_pid and
         * return success. */
        if (infop_ptr != 0u) {
            uint8_t zero[128];
            memset(zero, 0, sizeof(zero));
            if (copy_to_user_trusted((void *)(uintptr_t)infop_ptr, zero,
                                     sizeof(zero)) != 0u) {
                return LINUX_EFAULT;
            }
        }
        return 0;
    }

    if (infop_ptr != 0u) {
        uint8_t info[128];
        memset(info, 0, sizeof(info));
        int32_t si_signo = 17; /* SIGCHLD */
        int32_t si_code;
        int32_t si_status;
        if (term_signal != 0) {
            si_code = 2;                         /* CLD_KILLED */
            si_status = term_signal & 0x7f;
        } else {
            si_code = 1;                         /* CLD_EXITED */
            si_status = exit_code & 0xff;
        }
        memcpy(info + 0, &si_signo, sizeof(si_signo));
        memcpy(info + 8, &si_code, sizeof(si_code));
        memcpy(info + 16, &child, sizeof(child));   /* si_pid */
        /* si_uid at +20 stays 0 */
        memcpy(info + 24, &si_status, sizeof(si_status));
        if (copy_to_user_trusted((void *)(uintptr_t)infop_ptr, info,
                                 sizeof(info)) != 0u) {
            return LINUX_EFAULT;
        }
    }
    return 0;
}

static int64_t linux_getrusage(uint64_t who, uint64_t usage_ptr)
{
    (void)who;
    if (usage_ptr == 0u ||
        !process_user_buffer_is_valid((void *)(uintptr_t)usage_ptr, 144u)) {
        return LINUX_EFAULT;
    }
    uint8_t zero[144];
    memset(zero, 0, sizeof(zero));
    return copy_to_user_trusted((void *)(uintptr_t)usage_ptr, zero,
                                sizeof(zero)) != 0u ? LINUX_EFAULT : 0;
}

/* sendmmsg/recvmmsg - only AF_UNIX (Mojo IPC) like plain sendmsg/recvmsg;
 * iterate the mmsghdr[] array and fill msg_len. struct mmsghdr = 56-byte
 * struct msghdr + uint msg_len, padded to 64 bytes on x86_64. */
#define LINUX_MSGHDR_SIZE   56u
#define LINUX_MMSGHDR_SIZE  64u

static int64_t linux_sendmmsg(uint64_t fd, uint64_t msgvec, uint64_t vlen,
                              uint64_t flags)
{
    (void)flags;
    if (!unix_socket_fd_in_range((int32_t)fd)) {
        return LINUX_ENOTSUP;
    }
    if (vlen == 0u) {
        return 0;
    }
    if (vlen > 1024u) {
        vlen = 1024u;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)msgvec,
                                      vlen * LINUX_MMSGHDR_SIZE)) {
        return LINUX_EFAULT;
    }
    uint64_t sent = 0;
    for (uint64_t i = 0; i < vlen; ++i) {
        uint64_t hdr = msgvec + i * LINUX_MMSGHDR_SIZE;
        int64_t rc = unix_socket_sendmsg((int32_t)fd, hdr);
        if (rc < 0) {
            return sent != 0u ? (int64_t)sent : rc;
        }
        uint32_t msg_len = (uint32_t)rc;
        if (copy_to_user_trusted((void *)(uintptr_t)(hdr + LINUX_MSGHDR_SIZE),
                                 &msg_len, sizeof(msg_len)) != 0u) {
            return sent != 0u ? (int64_t)sent : LINUX_EFAULT;
        }
        ++sent;
    }
    return (int64_t)sent;
}

static int64_t linux_recvmmsg(uint64_t fd, uint64_t msgvec, uint64_t vlen,
                              uint64_t flags)
{
    (void)flags;
    if (!unix_socket_fd_in_range((int32_t)fd)) {
        return LINUX_ENOTSUP;
    }
    if (vlen == 0u) {
        return 0;
    }
    if (vlen > 1024u) {
        vlen = 1024u;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)msgvec,
                                      vlen * LINUX_MMSGHDR_SIZE)) {
        return LINUX_EFAULT;
    }
    uint64_t got = 0;
    for (uint64_t i = 0; i < vlen; ++i) {
        uint64_t hdr = msgvec + i * LINUX_MMSGHDR_SIZE;
        int64_t rc = unix_socket_recvmsg((int32_t)fd, hdr);
        if (rc < 0) {
            /* Return what we already have; a bare error only if nothing. */
            return got != 0u ? (int64_t)got : rc;
        }
        uint32_t msg_len = (uint32_t)rc;
        if (copy_to_user_trusted((void *)(uintptr_t)(hdr + LINUX_MSGHDR_SIZE),
                                 &msg_len, sizeof(msg_len)) != 0u) {
            return got != 0u ? (int64_t)got : LINUX_EFAULT;
        }
        ++got;
        if (rc == 0) {
            break;
        }
    }
    return (int64_t)got;
}

/* inotify: no real filesystem-change notification, but returning a valid
 * (never-ready) fd plus monotonically increasing watch descriptors keeps
 * Chromium's FilePathWatcher from treating the feature as fatally broken -
 * it simply never sees events and callers that need liveness fall back to
 * manual polling (TODO_Chromium_LinuxABI.md section 4). */
static int64_t linux_inotify_init(void)
{
    /* An empty-mask signalfd is a descriptor that is always readable=false
     * and integrates with the existing poll()/epoll() readiness plumbing. */
    return (int64_t)syscall_file_create_signalfd(0u);
}

static int64_t linux_inotify_add_watch(void)
{
    static volatile int32_t next_wd;
    return (int64_t)__sync_add_and_fetch(&next_wd, 1);
}

static int64_t linux_prctl_ext(uint64_t option, uint64_t arg2, uint64_t arg3,
                               uint64_t arg4, uint64_t arg5)
{
    (void)arg2;
    (void)arg3;
    (void)arg4;
    (void)arg5;
    switch (option) {
        case 1u:   /* PR_SET_PDEATHSIG   - no parent-death tracking; accept */
        case 2u:   /* PR_GET_PDEATHSIG   */
        case 3u:   /* PR_GET_DUMPABLE    */
        case 4u:   /* PR_SET_DUMPABLE    */
        case 7u:   /* PR_SET_KEEPCAPS    */
        case 8u:   /* PR_GET_KEEPCAPS    */
        case 15u:  /* PR_SET_NAME (handled by linux_prctl, here for safety) */
        case 22u:  /* PR_SET_SECCOMP - sandbox is out of scope; pretend OK  */
        case 23u:  /* PR_CAPBSET_READ   */
        case 35u:  /* PR_SET_TIMERSLACK */
        case 36u:  /* PR_GET_TIMERSLACK */
        case 38u:  /* PR_SET_NO_NEW_PRIVS */
        case 39u:  /* PR_GET_NO_NEW_PRIVS */
        case 41u:  /* PR_SET_THP_DISABLE  */
        case 42u:  /* PR_GET_THP_DISABLE  */
        case 24u:  /* PR_CAPBSET_DROP     */
        case 47u:  /* PR_CAP_AMBIENT      */
        case 45u:  /* PR_SET_PTRACER      */
        case 53u:  /* PR_SET_VMA (naming anon mappings) */
        case 59u:  /* PR_SET_SYSCALL_USER_DISPATCH */
        case 65u:  /* PR_GET_AUXV via prctl on newer kernels (accept, no-op) */
            return 0;
        case 21u:  /* PR_GET_SECCOMP - 0 == "not in seccomp mode" */
            return 0;
        default:
            return LINUX_ENOTSUP;
    }
}

/* Compile-time syscall trace (TODO_Chromium_LinuxABI.md section 6 / P3).
 * Build the kernel with -DLINUX_SYSCALL_TRACE to log every Linux-ABI
 * syscall (number, the six args, and the return value) to COM1. Zero cost
 * when the macro is undefined. Kept deliberately allocation-free and
 * lock-free so it is safe to call from the raw syscall path. */
#ifdef LINUX_SYSCALL_TRACE
/* Compact hex: serial_write_uint64() always emits 16 digits, and a full
 * trace of a Chromium startup is ~15k syscalls -- the leading zeros alone
 * are megabytes of COM1 time. */
static void linux_trace_hex(uint64_t v)
{
    char buf[17];
    int i = 16;
    buf[16] = '\0';
    if (v == 0u) {
        serial_write_char('0');
        return;
    }
    while (v != 0u && i > 0) {
        uint8_t d = (uint8_t)(v & 0xFu);
        buf[--i] = (char)(d < 10u ? (uint8_t)('0' + d)
                                  : (uint8_t)('a' + (d - 10u)));
        v >>= 4;
    }
    serial_write_string(&buf[i]);
}

/* A trace that starts at exec() is almost all dynamic linker: ~40k syscalls
 * of open/mmap before the program's own first instruction, and on COM1 that
 * costs minutes and changes the timing of whatever is being chased. So hold
 * the trace off until the process touches an AF_UNIX socket -- the display
 * connection is the point every graphical bring-up question is downstream
 * of -- and then stop again after LINUX_TRACE_MAX lines. */
#ifndef LINUX_TRACE_MAX
#define LINUX_TRACE_MAX 6000u
#endif
static bool     g_lx_trace_armed;
static uint32_t g_lx_trace_left = LINUX_TRACE_MAX;

bool linux_trace_is_armed(void) { return g_lx_trace_armed; }

static bool linux_trace_gate(uint64_t a1)
{
    if (!g_lx_trace_armed) {
        if (!unix_socket_fd_in_range((int32_t)a1)) return false;
        g_lx_trace_armed = true;
        serial_write_string("[lx] trace armed\n");
    }
    if (g_lx_trace_left == 0u) return false;
    if (--g_lx_trace_left == 0u) {
        serial_write_string("[lx] trace cap reached\n");
        return false;
    }
    return true;
}

static void linux_trace_enter(uint64_t num, uint64_t a1, uint64_t a2,
                              uint64_t a3, uint64_t a4, uint64_t a5,
                              uint64_t a6)
{
    if (!linux_trace_gate(a1)) return;
    serial_write_string("[lx]p");
    linux_trace_hex((uint64_t)(uint32_t)process_get_current_pid());
    serial_write_string(" #");
    linux_trace_hex(num);
    serial_write_char('(');
    linux_trace_hex(a1); serial_write_char(',');
    linux_trace_hex(a2); serial_write_char(',');
    linux_trace_hex(a3); serial_write_char(',');
    linux_trace_hex(a4); serial_write_char(',');
    linux_trace_hex(a5); serial_write_char(',');
    linux_trace_hex(a6);
    serial_write_string(")\n");
}

static void linux_trace_exit(uint64_t num, int64_t result)
{
    if (!g_lx_trace_armed || g_lx_trace_left == 0u) return;
    serial_write_string("[lx]p");
    linux_trace_hex((uint64_t)(uint32_t)process_get_current_pid());
    serial_write_string(" #");
    linux_trace_hex(num);
    serial_write_string("=");
    if (result < 0) {
        serial_write_char('-');
        linux_trace_hex((uint64_t)(-result));
    } else {
        linux_trace_hex((uint64_t)result);
    }
    serial_write_char('\n');
}
#define LINUX_TRACE_ENTER(n, a1, a2, a3, a4, a5, a6) \
    linux_trace_enter((n), (a1), (a2), (a3), (a4), (a5), (a6))
#define LINUX_TRACE_EXIT(n, r) linux_trace_exit((n), (r))
#else
#define LINUX_TRACE_ENTER(n, a1, a2, a3, a4, a5, a6) ((void)0)
#define LINUX_TRACE_EXIT(n, r) ((void)0)
#endif

/* Progress heartbeat for foreign processes.
 *
 * When Xorg goes silent there is no way to tell "still working" from "wedged",
 * and a full syscall trace drowns the shared COM1. Instead remember the last
 * syscall each pid entered plus how many it has made, and dump that table
 * whenever HEARTBEAT_PERIOD_MS has elapsed. Any process that is still running
 * drives the dump, so a stalled one shows up as a frozen count against the
 * syscall number it is parked in. */
#define LX_HEARTBEAT_MAX_PID     64u
#define LX_HEARTBEAT_PERIOD_MS   5000ull

static uint32_t g_lx_hb_last_num[LX_HEARTBEAT_MAX_PID];
static uint64_t g_lx_hb_last_arg[LX_HEARTBEAT_MAX_PID];
static uint64_t g_lx_hb_last_rip[LX_HEARTBEAT_MAX_PID];
static uint64_t g_lx_hb_count[LX_HEARTBEAT_MAX_PID];
static uint64_t g_lx_hb_next_dump_ns;

/* Read-only views of the per-pid heartbeat, for the timer-driven stall dump in
 * ProcessManager_Create.c. That dump has to be driven by the timer rather than
 * by the syscall path: the case it exists to diagnose is "every thread is
 * blocked and no syscalls are happening at all", which the syscall-driven
 * heartbeat below can never report. */
uint32_t linux_heartbeat_last_num(int32_t pid);
uint64_t linux_heartbeat_count(int32_t pid);
uint64_t linux_heartbeat_last_arg(int32_t pid);
uint64_t linux_heartbeat_last_rip(int32_t pid);

static void linux_syscall_heartbeat(uint64_t num, uint64_t arg1,
                                    uint64_t saved_rsp)
{
    /* Indexed by thread, not by memory owner: a stalled multithreaded process
     * is only diagnosable if each thread's last syscall is visible separately.
     * Keyed on the owner, every thread of Chromium wrote into one slot and the
     * dump said nothing more than "chrome is alive". */
    int32_t pid = process_get_current_tid();
    if (pid >= 0 && (uint32_t)pid < LX_HEARTBEAT_MAX_PID) {
        g_lx_hb_last_num[pid] = (uint32_t)num;
        g_lx_hb_last_arg[pid] = arg1;
        g_lx_hb_count[pid]++;
#if defined(__x86_64__)
        /* SYSCALL leaves the user return address in RCX, and the entry stub
         * saves it in the frame -- the same word linux_syscall_restart()
         * rewinds. It is the only handle the kernel has on where a stalled
         * foreign binary actually is. */
        g_lx_hb_last_rip[pid] =
            ((const uint64_t *)(uintptr_t)saved_rsp)[SYSCALL_FRAME_RCX];
#else
        (void)saved_rsp;
#endif
    }

    if (!OS_CONFIG_FOREIGN_TRACE) {
        return;
    }

    uint64_t now = timer_monotonic_ns();
    if (g_lx_hb_next_dump_ns == 0u) {
        g_lx_hb_next_dump_ns = now + LX_HEARTBEAT_PERIOD_MS * 1000000ull;
        return;
    }
    if (now < g_lx_hb_next_dump_ns) {
        return;
    }
    g_lx_hb_next_dump_ns = now + LX_HEARTBEAT_PERIOD_MS * 1000000ull;

    serial_write_string("[hb]");
    for (uint32_t i = 0; i < LX_HEARTBEAT_MAX_PID; ++i) {
        if (g_lx_hb_count[i] == 0u) continue;
        serial_write_string(" p");
        serial_write_uint32(i);
        serial_write_string("=#");
        serial_write_uint32(g_lx_hb_last_num[i]);
        serial_write_string("/a");
        serial_write_uint64(g_lx_hb_last_arg[i]);
        serial_write_string("/n");
        serial_write_uint64(g_lx_hb_count[i]);
    }
    serial_write_char('\n');
    /* Whose kernel-side reference is keeping a finished child unreapable is
     * only visible per CPU, so dump that alongside. */
    process_scheduler_debug_dump_cpus();
}

/* A blocking AF_UNIX read that found the ring empty. EAGAIN is not a value a
 * blocking recv() may return, and a client that gets one where it expected to
 * wait simply gives up: the X11 handshake died right here, with libxcb's first
 * read of the server's setup reply. Park the caller for a slice and run the
 * syscall again (same mechanism as linux_wait4/linux_epoll_wait). */
/* recv/send flags. Only MSG_DONTWAIT changes what the kernel must do here:
 * it makes a single call non-blocking regardless of the fd's O_NONBLOCK, so
 * EAGAIN is the answer the caller asked for and must not be turned into a
 * wait. libwayland reads the display socket with
 * MSG_DONTWAIT | MSG_CMSG_CLOEXEC on a *blocking* fd and relies on exactly
 * that: blocking there instead wedges the client the moment its event queue
 * runs dry -- which is one dispatch after the registry, so a GTK3 client
 * connected, received every global, and then never spoke again. */
#define LINUX_MSG_DONTWAIT 0x40u

static void linux_unix_block_retry(int32_t fd, int64_t *result,
                                   int *should_switch, int *restart_out)
{
    if (*result != LINUX_EAGAIN) {
        return;
    }
    if (unix_socket_fd_in_range(fd)) {
        if (unix_socket_is_nonblock(fd)) {
            /* Bring-up trace: the caller asked for a non-blocking read, so
             * EAGAIN is the right answer and it is on the caller to wait.
             * Printed because "the client gave up after one EAGAIN" looks
             * identical whether the socket really was non-blocking or we
             * simply failed to block. */
            unix_socket_trace_note("rx-NB", fd);
            return;
        }
    } else if (syscall_file_is_pipe(fd)) {
        /* A blocking read()/write() on a pipe must never come back EAGAIN:
         * syscall_pipe_read() waits inside the kernel but gives up after
         * PIPE_READ_WAIT_MAX_MS and reports EAGAIN even for a blocking fd,
         * and a reader that legitimately waits forever then sees an error
         * that cannot happen on Linux. Chromium's CrShutdownDetector thread
         * blocks on exactly such a pipe for the life of the browser and
         * turned the EAGAIN into "NOTREACHED hit. Unexpected error: Resource
         * temporarily unavailable" (shutdown_signal_handlers_posix.cc:137),
         * killing the process. Restart the syscall instead -- the same way
         * poll/epoll_wait/wait4 turn "no deadline" into a re-entry. */
        int32_t flags = syscall_file_get_status_flags(fd);
        if (flags < 0 || ((uint32_t)flags & LINUX_O_NONBLOCK) != 0u) {
            return;
        }
    } else {
        return;
    }
    if (process_sleep_current_ms(LINUX_WAIT_POLL_SLICE_MS) == 0 &&
        should_switch != NULL) {
        *should_switch = 1;
    }
    if (restart_out != NULL) {
        *restart_out = 1;
        *result = 0;
    }
}

/* select(2) / pselect6(2).
 *
 * Neither was implemented, so both returned ENOSYS -- and Xlib/libxcb wait for
 * the server's reply with exactly this call. An X client therefore issued one
 * recv(), got EAGAIN, tried to wait, was told the wait does not exist, and
 * reported "Couldn't connect to display!" without ever retrying. Built on the
 * same per-fd readiness probe poll(2) uses; a NULL timeout means "no
 * deadline", so it re-runs the syscall rather than reporting a timeout. */
#define LINUX_SELECT_MAX_FDS 1024u

static int64_t linux_select_common(uint64_t nfds, uint64_t rd_ptr,
                                   uint64_t wr_ptr, uint64_t ex_ptr,
                                   int64_t timeout_ms, int *should_switch_out,
                                   int *restart_out)
{
    if ((int64_t)nfds < 0 || nfds > LINUX_SELECT_MAX_FDS) {
        return LINUX_EINVAL;
    }
    /* Before the readiness scan, so a wakeup during it is not lost. */
    uint64_t generation = poll_wait_generation();
    uint64_t bytes = (nfds + 7u) / 8u;
    uint8_t rd[LINUX_SELECT_MAX_FDS / 8];
    uint8_t wr[LINUX_SELECT_MAX_FDS / 8];
    uint8_t ex[LINUX_SELECT_MAX_FDS / 8];
    memset(rd, 0, sizeof(rd));
    memset(wr, 0, sizeof(wr));
    memset(ex, 0, sizeof(ex));

    if (bytes != 0u) {
        if (rd_ptr != 0u &&
            copy_from_user(rd, (const void *)(uintptr_t)rd_ptr, bytes) != 0u) {
            return LINUX_EFAULT;
        }
        if (wr_ptr != 0u &&
            copy_from_user(wr, (const void *)(uintptr_t)wr_ptr, bytes) != 0u) {
            return LINUX_EFAULT;
        }
        if (ex_ptr != 0u &&
            copy_from_user(ex, (const void *)(uintptr_t)ex_ptr, bytes) != 0u) {
            return LINUX_EFAULT;
        }
    }

    uint8_t rd_out[LINUX_SELECT_MAX_FDS / 8];
    uint8_t wr_out[LINUX_SELECT_MAX_FDS / 8];
    memset(rd_out, 0, sizeof(rd_out));
    memset(wr_out, 0, sizeof(wr_out));

    int64_t ready = 0;
    for (uint64_t fd = 0; fd < nfds; ++fd) {
        uint32_t bit = 1u << (fd & 7u);
        uint64_t idx = fd >> 3;
        int want_rd = (rd_ptr != 0u) && ((rd[idx] & bit) != 0u);
        int want_wr = (wr_ptr != 0u) && ((wr[idx] & bit) != 0u);
        if (!want_rd && !want_wr) {
            continue;
        }
        uint32_t want = 0u;
        if (want_rd) want |= 0x1u;
        if (want_wr) want |= 0x4u;
        int32_t global = lxfd_get((int32_t)fd);
        if (global < 0) {
            return LINUX_EBADF; /* select() reports a bad descriptor */
        }
        uint32_t r = syscall_poll_one_fd(global, want);
        /* An error or hangup makes the fd readable as far as select() is
         * concerned - that is how a caller learns to go and collect it. */
        if (want_rd && (r & (0x1u | 0x8u | 0x10u | 0x20u)) != 0u) {
            rd_out[idx] |= (uint8_t)bit;
            ++ready;
        }
        if (want_wr && (r & (0x4u | 0x8u)) != 0u) {
            wr_out[idx] |= (uint8_t)bit;
            ++ready;
        }
    }

    if (ready == 0 && timeout_ms != 0) {
        uint32_t slice_ms = LINUX_POLL_SLICE_MS;
        if (timeout_ms > 0 && (uint64_t)timeout_ms < slice_ms) {
            slice_ms = (uint32_t)timeout_ms;
        }
        if (poll_wait_park(generation, slice_ms) != 0 &&
            should_switch_out != NULL) {
            *should_switch_out = 1;
        }
        (void)restart_out;
        /* Deliberately NOT restarted in-kernel the way epoll_wait() is, even
         * for a NULL timeout: measured, that hangs the X server somewhere in
         * its input-device init (it stopped after adding kbd0 and never
         * reached Dispatch(), so no client was ever accepted). A spurious
         * zero return is a wakeup the caller retries; an in-kernel wait that
         * never yields is a deadlock. See TODO_Doom_Xorg_MethodA.md M23.
         *
         * The caller may not observe a partially-updated set on timeout. */
        return 0;
    }

    if (bytes != 0u) {
        if (rd_ptr != 0u &&
            copy_to_user((void *)(uintptr_t)rd_ptr, rd_out, bytes) != 0u) {
            return LINUX_EFAULT;
        }
        if (wr_ptr != 0u &&
            copy_to_user((void *)(uintptr_t)wr_ptr, wr_out, bytes) != 0u) {
            return LINUX_EFAULT;
        }
        if (ex_ptr != 0u) {
            memset(ex, 0, sizeof(ex)); /* no exceptional conditions modelled */
            if (copy_to_user((void *)(uintptr_t)ex_ptr, ex, bytes) != 0u) {
                return LINUX_EFAULT;
            }
        }
    }
    return ready;
}

/* ===================================================================== */
/* Per-process descriptor numbering (Compat/Linux/Linux_FdTable.c)        */
/* ===================================================================== */

/*
 * Every Linux-ABI process names objects by numbers from its own table. The
 * handlers below this point keep working in the kernel's global numbers:
 * lx_fd_pre() rewrites descriptor arguments on the way in (and handles the
 * calls that are purely about numbering -- close, dup*, F_DUPFD, F_[GS]ETFD
 * -- itself), lx_fd_post() gives every descriptor a call created a number in
 * the caller's table on the way out.
 *
 * Close-on-exec lives in the per-process table only. The O_CLOEXEC-style
 * bits are recorded here and stripped before the handlers see them, so no
 * backend ever closes a global object behind the table's back at execve().
 */

#define LX_CLOEXEC_FLAG 0x80000u /* O_/SOCK_/EFD_/TFD_/SFD_/IN_/EPOLL_CLOEXEC */

/* The console is global 0/1/2 in every table and is never closed. */
static int lx_is_console_global(int32_t g)
{
    return g >= 0 && g <= 2;
}

/* close(2) of a kernel-global descriptor, in whichever backend owns it. */
static int64_t linux_close_global(int32_t g)
{
    if (lx_is_console_global(g)) {
        return 0;
    }
    int32_t owner = process_memory_owner_pid_of(process_get_current_pid());
    syscall_epoll_forget_fd_for(g, owner);
    if (unix_socket_fd_in_range(g)) {
        return unix_socket_close(g);
    }
    if (syscall_eventfd_is_valid(g)) {
        return (int64_t)syscall_eventfd_close(g);
    }
    if (syscall_epoll_is_valid(g)) {
        return (int64_t)syscall_epoll_close(g);
    }
    if (syscall_socket_get_type(g) >= 0) {
        return (int64_t)syscall_socket_close(g);
    }
    return (int64_t)syscall_file_close(g);
}

static void linux_close_global_void(int32_t g)
{
    (void)linux_close_global(g);
}

/* fork(): the child gets a copy of the table. epoll and eventfd objects are
 * refcounted per holding process rather than per pid, so the child's copy is
 * one more reference to each distinct one. */
static void lx_fork_hook(int32_t parent, int32_t child)
{
    lxfd_fork(parent, child);
    /* Capability sets are inherited across fork. */
    if (parent >= 0 && parent < (int32_t)OS_CONFIG_PROCESS_MAX_COUNT &&
        child >= 0 && child < (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) {
        g_linux_caps[child] = g_linux_caps[parent];
    }
    for (int32_t u = lxfd_next_open(child, -1); u >= 0;
         u = lxfd_next_open(child, u)) {
        int32_t g = lxfd_get_for(child, u);
        int seen = 0;
        for (int32_t v = lxfd_next_open(child, -1); v >= 0 && v < u;
             v = lxfd_next_open(child, v)) {
            if (lxfd_get_for(child, v) == g) {
                seen = 1;
                break;
            }
        }
        if (seen) {
            continue;
        }
        if (syscall_eventfd_is_valid(g)) {
            syscall_eventfd_addref(g);
        } else if (syscall_epoll_is_valid(g)) {
            syscall_epoll_addref(g);
        }
    }
}

/* Exit: files, pipes and sockets are released per pid by the exit path
 * itself; epoll and eventfd hold one reference per process. */
static void lx_exit_ref(int32_t g)
{
    if (syscall_eventfd_is_valid(g)) {
        (void)syscall_eventfd_close(g);
    } else if (syscall_epoll_is_valid(g)) {
        (void)syscall_epoll_close(g);
    }
}

static void lx_exit_hook(int32_t pid, int32_t unused)
{
    (void)unused;
    if (pid >= 0 && pid < (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) {
        g_linux_caps[pid].valid = 0u; /* the next process in this slot starts fresh */
    }
    lxfd_release_process(pid, lx_exit_ref);
}

static int lx_current_is_linux(void)
{
    return process_get_current_abi_mode() == PROCESS_ABI_LINUX;
}

/* SCM_RIGHTS: user number -> global on send, global -> user number on
 * receive. */
static int32_t lx_scm_resolve(int32_t u)
{
    return lx_current_is_linux() ? lxfd_get(u) : u;
}

static int32_t lx_scm_install(int32_t g)
{
    if (!lx_current_is_linux()) {
        return g;
    }
    int32_t u = lxfd_install(g, 0, 0);
    if (u < 0) {
        (void)linux_close_global(g);
    }
    return u;
}

/* /proc/<pid>/fd for a process with a table of its own. */
static int32_t lx_procfs_translate(int32_t pid, int32_t fd)
{
    if (lxfd_next_open(pid, -1) == -2) {
        return -2;
    }
    return lxfd_get_for(pid, fd);
}

static int32_t lx_procfs_next(int32_t pid, int32_t after)
{
    return lxfd_next_open(pid, after);
}

/* Rewrite one descriptor argument; EBADF if it names nothing. */
static int lx_xlate(uint64_t *arg)
{
    int32_t g = lxfd_get((int32_t)*arg);
    if (g < 0) {
        return -1;
    }
    *arg = (uint64_t)(uint32_t)g;
    return 0;
}

/* A directory descriptor argument: AT_FDCWD passes through untouched. */
static int lx_xlate_dirfd(uint64_t *arg)
{
    if ((int32_t)*arg == LINUX_AT_FDCWD) {
        return 0;
    }
    return lx_xlate(arg);
}

/* A pid argument the caller wrote in its own namespace's terms. */
static void lx_pid_in(uint64_t *arg)
{
    int32_t v = (int32_t)*arg;
    if (v > 0) {
        *arg = (uint64_t)(uint32_t)process_pid_from_current_view(v);
    }
}

/* Returns 1 when the call is finished (result in *res). */
static int lx_fd_pre(uint64_t num, uint64_t *a, int64_t *res, int32_t *cloexec)
{
    *cloexec = 0;
    switch (num) {
        /* ---- calls that are only about numbering ---- */
        case LINUX_SYS_CLOSE: {
            int32_t g = -1;
            int last = lxfd_remove((int32_t)a[0], &g);
            if (last < 0) {
                *res = LINUX_EBADF;
            } else {
                if (last == 1) {
                    (void)linux_close_global(g);
                }
                *res = 0;
            }
            return 1;
        }
        case LINUX_SYS_DUP: {
            int32_t g = lxfd_get((int32_t)a[0]);
            *res = (g < 0) ? LINUX_EBADF : (int64_t)lxfd_install(g, 0, 0);
            return 1;
        }
        case LINUX_SYS_DUP2:
        case LINUX_SYS_DUP3: {
            int32_t g = lxfd_get((int32_t)a[0]);
            int on = (num == LINUX_SYS_DUP3) && (a[2] & LX_CLOEXEC_FLAG) != 0u;
            if (g < 0) {
                *res = LINUX_EBADF;
                return 1;
            }
            if (a[0] == a[1]) {
                *res = (num == LINUX_SYS_DUP3) ? LINUX_EINVAL : (int64_t)a[1];
                return 1;
            }
            int32_t replaced = -1;
            int last = 0;
            int32_t rc = lxfd_set((int32_t)a[1], g, on, &replaced, &last);
            if (rc >= 0 && last) {
                (void)linux_close_global(replaced);
            }
            *res = rc;
            return 1;
        }
        case LINUX_SYS_FCNTL: {
            int32_t cmd = (int32_t)a[1];
            if (cmd == 0 /* F_DUPFD */ || cmd == 1030 /* F_DUPFD_CLOEXEC */) {
                int32_t g = lxfd_get((int32_t)a[0]);
                *res = (g < 0) ? LINUX_EBADF
                               : (int64_t)lxfd_install(g, cmd == 1030,
                                                       (int32_t)a[2]);
                return 1;
            }
            if (cmd == 1 /* F_GETFD */) {
                int rc = lxfd_get_cloexec((int32_t)a[0]);
                *res = (rc < 0) ? LINUX_EBADF : (rc ? 1 : 0);
                return 1;
            }
            if (cmd == 2 /* F_SETFD */) {
                int rc = lxfd_set_cloexec((int32_t)a[0], (a[2] & 1u) != 0u);
                *res = (rc < 0) ? LINUX_EBADF : 0;
                return 1;
            }
            if (lx_xlate(&a[0]) < 0) { *res = LINUX_EBADF; return 1; }
            return 0;
        }

        /* ---- one plain descriptor in arg1 ---- */
        case LINUX_SYS_READ: case LINUX_SYS_WRITE: case LINUX_SYS_FSTAT:
        case LINUX_SYS_LSEEK: case LINUX_SYS_IOCTL: case LINUX_SYS_READV:
        case LINUX_SYS_WRITEV: case LINUX_SYS_FTRUNCATE:
        case LINUX_SYS_FALLOCATE: case LINUX_SYS_FSTATFS:
        case LINUX_SYS_GETDENTS64: case LINUX_SYS_EPOLL_WAIT:
        case LINUX_SYS_EPOLL_PWAIT: case LINUX_SYS_EPOLL_PWAIT2:
        case LINUX_SYS_TIMERFD_SETTIME: case LINUX_SYS_TIMERFD_GETTIME:
        case LINUX_SYS_BIND: case LINUX_SYS_CONNECT: case LINUX_SYS_LISTEN:
        case LINUX_SYS_SENDTO: case LINUX_SYS_RECVFROM:
        case LINUX_SYS_SENDMSG: case LINUX_SYS_RECVMSG:
        case LINUX_SYS_SHUTDOWN: case LINUX_SYS_SETSOCKOPT:
        case LINUX_SYS_GETSOCKOPT: case LINUX_SYS_GETSOCKNAME:
        case LINUX_SYS_GETPEERNAME: case LINUX_SYS_PREAD64:
        case LINUX_SYS_PWRITE64: case LINUX_SYS_SENDMMSG:
        case LINUX_SYS_RECVMMSG: case LINUX_SYS_INOTIFY_ADD_WATCH:
        case LINUX_SYS_INOTIFY_RM_WATCH: case LINUX_SYS_FSYNC:
        case LINUX_SYS_FDATASYNC: case LINUX_SYS_SYNCFS: case LINUX_SYS_FLOCK:
        case LINUX_SYS_FADVISE64: case LINUX_SYS_FCHMOD:
        case LINUX_SYS_FCHOWN:
            if (lx_xlate(&a[0]) < 0) { *res = LINUX_EBADF; return 1; }
            return 0;

        case LINUX_SYS_ACCEPT:
            if (lx_xlate(&a[0]) < 0) { *res = LINUX_EBADF; return 1; }
            return 0;
        case 288u: /* accept4 */
            *cloexec = (a[3] & LX_CLOEXEC_FLAG) != 0u;
            if (lx_xlate(&a[0]) < 0) { *res = LINUX_EBADF; return 1; }
            return 0;

        case LINUX_SYS_SENDFILE:
            if (lx_xlate(&a[0]) < 0 || lx_xlate(&a[1]) < 0) {
                *res = LINUX_EBADF;
                return 1;
            }
            return 0;

        case LINUX_SYS_EPOLL_CTL:
            if (lx_xlate(&a[0]) < 0 || lx_xlate(&a[2]) < 0) {
                *res = LINUX_EBADF;
                return 1;
            }
            return 0;

        case LINUX_SYS_MMAP:
            if ((a[3] & LINUX_MAP_ANONYMOUS) == 0u && (int32_t)a[4] != -1 &&
                lx_xlate(&a[4]) < 0) {
                *res = LINUX_EBADF;
                return 1;
            }
            return 0;

        /* ---- a directory descriptor ---- */
        case LINUX_SYS_NEWFSTATAT: case LINUX_SYS_READLINKAT:
        case LINUX_SYS_FACCESSAT: case LINUX_SYS_FACCESSAT2:
        case LINUX_SYS_UTIMENSAT: case LINUX_SYS_FCHMODAT:
        case LINUX_SYS_FCHOWNAT: case LINUX_SYS_MKDIRAT:
        case LINUX_SYS_UNLINKAT: case LINUX_SYS_STATX:
        case LINUX_SYS_NAME_TO_HANDLE_AT:
            if (lx_xlate_dirfd(&a[0]) < 0) { *res = LINUX_EBADF; return 1; }
            return 0;
        case LINUX_SYS_OPENAT:
            *cloexec = (a[2] & LX_CLOEXEC_FLAG) != 0u;
            a[2] &= ~(uint64_t)LX_CLOEXEC_FLAG;
            if (lx_xlate_dirfd(&a[0]) < 0) { *res = LINUX_EBADF; return 1; }
            return 0;
        case LINUX_SYS_RENAMEAT: case LINUX_SYS_RENAMEAT2:
        case LINUX_SYS_LINKAT:
            if (lx_xlate_dirfd(&a[0]) < 0 || lx_xlate_dirfd(&a[2]) < 0) {
                *res = LINUX_EBADF;
                return 1;
            }
            return 0;
        case LINUX_SYS_SYMLINKAT:
            if (lx_xlate_dirfd(&a[1]) < 0) { *res = LINUX_EBADF; return 1; }
            return 0;

        /* ---- calls that create descriptors: note close-on-exec ---- */
        case LINUX_SYS_OPEN:
            *cloexec = (a[1] & LX_CLOEXEC_FLAG) != 0u;
            a[1] &= ~(uint64_t)LX_CLOEXEC_FLAG;
            return 0;
        case LINUX_SYS_PIPE2:
            *cloexec = (a[1] & LX_CLOEXEC_FLAG) != 0u;
            a[1] &= ~(uint64_t)LX_CLOEXEC_FLAG;
            return 0;
        case LINUX_SYS_MEMFD_CREATE:
            *cloexec = (a[1] & 0x1u) != 0u; /* MFD_CLOEXEC */
            a[1] &= ~(uint64_t)0x1u;
            return 0;
        case LINUX_SYS_SOCKET: case LINUX_SYS_SOCKETPAIR:
        case LINUX_SYS_EVENTFD2: case LINUX_SYS_TIMERFD_CREATE:
            *cloexec = (a[1] & LX_CLOEXEC_FLAG) != 0u;
            if (num == LINUX_SYS_TIMERFD_CREATE) {
                a[1] &= ~(uint64_t)LX_CLOEXEC_FLAG;
            }
            return 0;
        case LINUX_SYS_EPOLL_CREATE1: case LINUX_SYS_INOTIFY_INIT1:
            *cloexec = (a[0] & LX_CLOEXEC_FLAG) != 0u;
            a[0] &= ~(uint64_t)LX_CLOEXEC_FLAG;
            return 0;
        case LINUX_SYS_SIGNALFD4:
            *cloexec = (a[3] & LX_CLOEXEC_FLAG) != 0u;
            a[3] &= ~(uint64_t)LX_CLOEXEC_FLAG;
            if ((int32_t)a[0] != -1 && lx_xlate(&a[0]) < 0) {
                *res = LINUX_EBADF;
                return 1;
            }
            return 0;

        /* ---- pids, in the caller's namespace ---- */
        case LINUX_SYS_KILL: case LINUX_SYS_TKILL: case LINUX_SYS_WAIT4:
        case LINUX_SYS_GETPGID: case LINUX_SYS_SETPGID: case LINUX_SYS_GETSID:
        case LINUX_SYS_PRLIMIT64: case LINUX_SYS_SCHED_GETAFFINITY:
        case LINUX_SYS_SCHED_SETAFFINITY:
            lx_pid_in(&a[0]);
            return 0;
        case LINUX_SYS_TGKILL:
            lx_pid_in(&a[0]);
            lx_pid_in(&a[1]);
            return 0;

        default:
            return 0;
    }
}

/* Give each global descriptor a call produced a number of its own. */
static int64_t lx_install_or_close(int64_t global, int32_t cloexec)
{
    if (global < 0) {
        return global;
    }
    int32_t u = lxfd_install((int32_t)global, cloexec, 0);
    if (u < 0) {
        (void)linux_close_global((int32_t)global);
        return u;
    }
    return u;
}

/* pipe()/pipe2()/socketpair(): the handler wrote two globals into the
 * caller's array; replace them with the caller's own numbers. */
static int64_t lx_install_pair(uint64_t user_array, int32_t cloexec)
{
    int32_t pair[2];
    if (copy_from_user(pair, (const void *)(uintptr_t)user_array,
                       sizeof(pair)) != 0u) {
        return LINUX_EFAULT;
    }
    int32_t u0 = lxfd_install(pair[0], cloexec, 0);
    int32_t u1 = (u0 >= 0) ? lxfd_install(pair[1], cloexec, 0) : -24;
    if (u0 < 0 || u1 < 0) {
        if (u0 >= 0) {
            int32_t g;
            (void)lxfd_remove(u0, &g);
        }
        (void)linux_close_global(pair[0]);
        (void)linux_close_global(pair[1]);
        return -24;
    }
    int32_t out[2] = { u0, u1 };
    if (copy_to_user((void *)(uintptr_t)user_array, out, sizeof(out)) != 0u) {
        return LINUX_EFAULT;
    }
    return 0;
}

static int64_t lx_fd_post(uint64_t num, const uint64_t *orig, int64_t result,
                          int32_t cloexec)
{
    switch (num) {
        case LINUX_SYS_OPEN: case LINUX_SYS_OPENAT: case LINUX_SYS_CREAT:
        case LINUX_SYS_SOCKET: case LINUX_SYS_ACCEPT: case 288u:
        case LINUX_SYS_TIMERFD_CREATE: case LINUX_SYS_MEMFD_CREATE:
        case LINUX_SYS_EVENTFD: case LINUX_SYS_EVENTFD2:
        case LINUX_SYS_EPOLL_CREATE1: case LINUX_SYS_INOTIFY_INIT:
        case LINUX_SYS_INOTIFY_INIT1:
            return lx_install_or_close(result, cloexec);
        case LINUX_SYS_SIGNALFD4:
            /* Updating an existing signalfd returns the same descriptor. */
            if ((int32_t)orig[0] != -1) {
                return result < 0 ? result : (int64_t)(int32_t)orig[0];
            }
            return lx_install_or_close(result, cloexec);
        case LINUX_SYS_PIPE: case LINUX_SYS_PIPE2:
            return result < 0 ? result : lx_install_pair(orig[0], cloexec);
        case LINUX_SYS_SOCKETPAIR:
            return result < 0 ? result : lx_install_pair(orig[3], cloexec);
        case LINUX_SYS_EXECVE:
            if (result >= 0) {
                lxfd_exec_close_cloexec(linux_close_global_void);
            }
            return result;

        /* pids out, in the caller's namespace */
        case LINUX_SYS_GETPID: case LINUX_SYS_GETTID:
        case LINUX_SYS_SET_TID_ADDRESS: case LINUX_SYS_WAIT4:
        case LINUX_SYS_CLONE: case LINUX_SYS_FORK: case LINUX_SYS_VFORK:
            return result > 0 ? (int64_t)process_pid_as_seen_by_current((int32_t)result)
                              : result;
        case LINUX_SYS_GETPPID:
            if (process_current_is_pidns_init()) {
                return 0;
            }
            return result > 0 ? (int64_t)process_pid_as_seen_by_current((int32_t)result)
                              : result;
        default:
            return result;
    }
}

/* Registered once, with the compat layer: the fd table has to follow
 * processes through fork/exit and be consulted by SCM_RIGHTS and /proc. */
static void lx_fd_hooks_register(void)
{
    process_register_lifecycle_hooks(lx_fork_hook, lx_exit_hook);
    unix_socket_set_fd_hooks(lx_scm_resolve, lx_scm_install);
    procfs_set_fd_hooks(lx_procfs_translate, lx_procfs_next);
}

uint64_t linux_syscall_dispatch(uint64_t saved_rsp,
                                uint64_t num,
                                uint64_t arg1,
                                uint64_t arg2,
                                uint64_t arg3,
                                uint64_t arg4,
                                uint64_t arg5,
                                uint64_t arg6)
{
    int64_t result = LINUX_ENOSYS;
    int request_switch = 0;
    int request_restart = 0;

    flight_rec(FR_TAG_SYSCALL, num, (uint64_t)(uint32_t)process_get_current_pid());
    linux_syscall_heartbeat(num, arg1, saved_rsp);
#if LINUX_SYSCALL_PROFILE
    linux_syscall_profile(num);
#endif

    LINUX_TRACE_ENTER(num, arg1, arg2, arg3, arg4, arg5, arg6);

    /* Descriptor and pid numbers arrive in the caller's terms; see
     * lx_fd_pre(). The originals are kept for lx_fd_post(). */
    const uint64_t lx_orig[6] = { arg1, arg2, arg3, arg4, arg5, arg6 };
    int32_t lx_cloexec = 0;
    int lx_done = 0;
    {
        uint64_t a[6] = { arg1, arg2, arg3, arg4, arg5, arg6 };
        int64_t early = 0;
        if (lx_fd_pre(num, a, &early, &lx_cloexec)) {
            result = early;
            lx_done = 1;
        }
        arg1 = a[0]; arg2 = a[1]; arg3 = a[2];
        arg4 = a[3]; arg5 = a[4]; arg6 = a[5];
    }
    if (lx_done) {
        goto lx_fd_finished;
    }

    switch (num) {
        case LINUX_SYS_READ: {
            int should_switch = 0;
            result = linux_read(arg1, arg2, arg3);
            linux_unix_block_retry((int32_t)arg1, &result, &should_switch,
                                   &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_WRITE: {
            int should_switch = 0;
            result = write((int32_t)arg1, (const void *)(uintptr_t)arg2,
                           arg3);
            /* Same rule as read(): a blocking write to a full pipe waits, it
             * does not report EAGAIN. */
            linux_unix_block_retry((int32_t)arg1, &result, &should_switch,
                                   &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_OPEN:
            result = linux_open_path(arg1, arg2);
            break;

        case LINUX_SYS_CLOSE:
            /* fd ranges are disjoint (file table / sockets / epoll fds /
             * eventfd / AF_UNIX - see the range comments on each), so
             * exactly one of these actually owns any given fd. Sockets
             * were previously never routed to syscall_socket_close() at
             * all here, leaking the socket table slot and leaving the
             * underlying TCP connection open until process exit. */
            if (unix_socket_fd_in_range((int32_t)arg1)) {
                result = unix_socket_close((int32_t)arg1);
            } else if (syscall_eventfd_is_valid((int32_t)arg1)) {
                result = (int64_t)syscall_eventfd_close((int32_t)arg1);
            } else if (syscall_socket_get_type((int32_t)arg1) >= 0) {
                result = (int64_t)syscall_socket_close((int32_t)arg1);
            } else {
                result = (int64_t)syscall_file_close((int32_t)arg1);
            }
            break;

        case LINUX_SYS_STAT:
        case LINUX_SYS_LSTAT: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg1);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_path(path, sizeof(path));
            if (rc < 0) {
                result = rc;
                break;
            }
            result = linux_stat_path(path, arg2);
            break;
        }

        case LINUX_SYS_FSTAT:
            result = linux_stat_fd((int32_t)arg1, arg2);
            break;

        case LINUX_SYS_NEWFSTATAT: {
            /* fstatat(dirfd, pathname, statbuf, flags).
             *
             * glibc's dynamic linker fstat()s every shared object it opens as
             * fstatat(fd, "", statbuf, AT_EMPTY_PATH) -- rejecting that with
             * ENOTSUP breaks loading ANY .so (Chromium: "cannot open shared
             * object file: libdl.so.2: Error 95"). Also: a real openat/at
             * pathname that is absolute must be resolved regardless of dirfd. */
            char path[256];
            char first = '\0';
            if (arg2 != 0u) {
                (void)copy_from_user(&first, (const void *)(uintptr_t)arg2, 1u);
            }
            if ((arg4 & LINUX_AT_EMPTY_PATH) != 0u && first == '\0') {
                result = linux_stat_fd((int32_t)arg1, arg3);
                break;
            }
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_at(arg1, path, sizeof(path));
            if (rc < 0) {
                result = rc;
                break;
            }
            result = linux_stat_path(path, arg3);
            break;
        }

        case LINUX_SYS_LSEEK:
            result = (int64_t)syscall_file_seek((int32_t)arg1,
                                                (int64_t)arg2,
                                                (int32_t)arg3);
            break;

        case LINUX_SYS_MMAP:
            result = linux_mmap(arg1, arg2, arg3, arg4, arg5, arg6);
            break;

        case LINUX_SYS_MPROTECT:
#if PAGING_LOST_MAPPING_TRACE
            lostmap_note("mprotect", arg1, arg2, arg3);
#endif
            result = syscall_vm_mprotect(arg1, arg2, arg3);
            break;

        case LINUX_SYS_MUNMAP: {
#if PAGING_LOST_MAPPING_TRACE
            lostmap_note("munmap", arg1, arg2, 0);
#endif
            int32_t self = process_get_current_pid();
            if (self >= 0 && arg2 != 0u) {
                /* Flush + drop any MAP_SHARED file mapping in this range. */
                linux_mshared_flush_range(self, arg1, arg1 + arg2, 1);
                /* ...and release any demand-paged file mapping it covers, so
                 * the reference on the open file description goes away. */
                filemap_unregister_range(self, arg1, arg2);
            }
            /* A mapping of a shared-memory object (mmap of a memfd) is owned
             * by the shared-memory layer, which has to release it itself: the
             * pages are shared and the range has to leave the object's
             * mapping table, or a later map of the same object returns this
             * address after the allocator has reused it. */
            if (shared_memory_unmap_any((void *)(uintptr_t)arg1)) {
                result = 0;
                break;
            }
            result = (int64_t)process_user_munmap((void *)(uintptr_t)arg1,
                                                  arg2);
            break;
        }

        case 26u: { /* msync */
            int32_t self = process_get_current_pid();
            if (self < 0 || arg2 == 0u) {
                result = 0;
            } else {
                /* MS_INVALIDATE is a no-op here (no shared cache to drop);
                 * MS_SYNC/MS_ASYNC both just write the region back now. */
                linux_mshared_flush_range(self, arg1, arg1 + arg2, 0);
                result = 0;
            }
            break;
        }

        case LINUX_SYS_BRK:
            result = linux_brk(arg1);
            break;

        case LINUX_SYS_RT_SIGACTION:
            result = linux_rt_sigaction(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_RT_SIGPROCMASK:
            result = linux_rt_sigprocmask(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_RT_SIGRETURN:
            result = process_signal_rt_sigreturn(saved_rsp);
            break;

        case LINUX_SYS_IOCTL:
            result = syscall_ioctl_ex(arg1, arg2, arg3);
            break;

        case LINUX_SYS_READV:
            result = syscall_readv((int32_t)arg1, arg2, (int32_t)arg3);
            break;

        case LINUX_SYS_WRITEV:
            result = syscall_writev((int32_t)arg1, arg2, (int32_t)arg3);
            break;

        case LINUX_SYS_ACCESS: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg1);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_path(path, sizeof(path));
            if (rc < 0) {
                result = rc;
                break;
            }
            result = syscall_access(path, (int32_t)arg2);
            break;
        }

        case LINUX_SYS_PIPE: {
            if (!process_user_buffer_is_valid((const void *)(uintptr_t)arg1,
                                              sizeof(int32_t) * 2u)) {
                result = LINUX_EFAULT;
                break;
            }
            int32_t fds[2];
            int32_t rc = syscall_file_pipe(fds);
            if (rc >= 0 &&
                copy_to_user_trusted((void *)(uintptr_t)arg1, fds, sizeof(fds)) != 0u) {
                result = LINUX_EFAULT;
                break;
            }
            result = rc;
            break;
        }

        case LINUX_SYS_SCHED_YIELD:
            result = 0;
            request_switch = 1;
            break;

        case LINUX_SYS_MREMAP:
            result = syscall_vm_mremap5(arg1, arg2, arg3, arg4, arg5);
            break;

        case LINUX_SYS_MINCORE:
            result = linux_mincore(arg1, arg2, arg3);
            break;

        case LINUX_SYS_MADVISE:
            result = linux_madvise(arg1, arg2, arg3);
            break;

        case LINUX_SYS_DUP:
            result = (int64_t)syscall_file_dup((int32_t)arg1);
            break;

        case LINUX_SYS_DUP2:
            result = (int64_t)syscall_file_dup2((int32_t)arg1, (int32_t)arg2);
            break;

        case LINUX_SYS_NANOSLEEP: {
            struct {
                int64_t sec;
                int64_t nsec;
            } req;
            if (copy_from_user(&req, (const void *)(uintptr_t)arg1,
                               sizeof(req)) != 0u) {
                result = LINUX_EFAULT;
                break;
            }
            if (req.sec < 0 || req.nsec < 0 || req.nsec >= 1000000000LL) {
                result = LINUX_EINVAL;
                break;
            }
            uint64_t ms = (uint64_t)req.sec * 1000u +
                          ((uint64_t)req.nsec + 999999u) / 1000000u;
            if (ms > 0u && process_sleep_current_ms(ms) == 0) {
                request_switch = 1;
            }
            result = 0;
            break;
        }

        case LINUX_SYS_GETITIMER:
            result = linux_getitimer(arg1, arg2);
            break;

        case LINUX_SYS_SETITIMER:
            result = linux_setitimer(arg1, arg2, arg3);
            break;

        case LINUX_SYS_GETPID:
            result = (int64_t)process_get_current_pid();
            break;

        case LINUX_SYS_SENDFILE:
            result = linux_sendfile(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_CLONE:
        case LINUX_SYS_FORK:
        case LINUX_SYS_VFORK: {
            /* A new process: stop our other threads first (see
             * process_fork_hold_acquire()). Until they are all off-CPU, park
             * briefly and re-enter; the hold keeps them from being scheduled
             * again in the meantime. Threads (CLONE_THREAD) need none of it. */
            uint64_t cflags = (num == LINUX_SYS_CLONE) ? arg1 : 0x11u /* SIGCHLD */;
            int is_process = (cflags & LINUX_CLONE_THREAD) == 0u;
            if (is_process && !process_fork_hold_acquire()) {
                if (process_sleep_current_ms(1) == 0) {
                    request_switch = 1;
                }
                request_restart = 1;
                result = 0;
                break;
            }
            int should_switch = 0;
            if (num == LINUX_SYS_CLONE) {
                result = linux_clone(saved_rsp, arg1, arg2, arg3, arg4, arg5,
                                     &should_switch);
            } else {
                result = linux_clone(saved_rsp, 0x11u, 0u, 0u, 0u, 0u,
                                     &should_switch);
            }
            if (is_process) {
                process_fork_hold_release();
            }
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_EXECVE:
            LX_PROC_TRACE("execve", (uint64_t)(uint32_t)num, 0u);
            lx_newborn_arm(process_get_current_pid(), LINUX_NEWBORN_TRACE);
            result = linux_execve(arg1, arg2, arg3);
            request_switch = 1;
            break;

        case LINUX_SYS_EXIT:
            LX_PROC_TRACE("exit", arg1, (uint64_t)process_is_current_thread());
            if (process_is_current_thread()) {
                process_thread_exit_current((int32_t)arg1 & 0xFF);
            } else {
                process_exit_current_with_status((int32_t)arg1 & 0xFF);
            }
            result = 0;
            request_switch = 1;
            break;

        case LINUX_SYS_EXIT_GROUP:
            LX_PROC_TRACE("exit_group", arg1, 0u);
            process_exit_current_with_status((int32_t)arg1 & 0xFF);
            process_retire_current_thread();
            result = 0;
            request_switch = 1;
            break;

        case LINUX_SYS_WAIT4: {
            int should_switch = 0;
            result = linux_wait4(arg1, arg2, arg3, arg4, &should_switch,
                                 &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_KILL:
            result = (int64_t)process_signal_deliver_group((int32_t)arg1,
                                                           (int32_t)arg2);
            if (result == 0 &&
                ((int32_t)arg1 == process_get_current_pid() ||
                 (int32_t)arg1 == 0 || (int32_t)arg1 == -1) &&
                process_signal_maybe_self_terminate((int32_t)arg2)) {
                request_switch = 1;
            }
            break;

        case LINUX_SYS_UNAME:
            result = linux_uname(arg1);
            break;

        case LINUX_SYS_FCNTL:
            result = syscall_fcntl_ex((int32_t)arg1, (int32_t)arg2, arg3);
            break;

        case LINUX_SYS_FTRUNCATE:
            result = syscall_ftruncate((int32_t)arg1, (int64_t)arg2);
            /* A memfd seal refuses the resize with EPERM on Linux, not EACCES,
             * and Mojo's seal self-check only accepts EINVAL/ENOSYS/EPERM. */
            if (result == (int64_t)OS_STATUS_ACCESS_DENIED) {
                result = -1; /* EPERM */
            }
            break;

        case LINUX_SYS_FALLOCATE:
            result = linux_fallocate((int32_t)arg1, (uint32_t)arg2,
                                     (int64_t)arg3, (int64_t)arg4);
            break;

        case LINUX_SYS_GETCWD:
            result = linux_getcwd(arg1, arg2);
            break;

        case LINUX_SYS_CHDIR:
            result = linux_chdir(arg1);
            break;

        case LINUX_SYS_RENAME: {
            char old_path[256];
            char new_path[256];
            int64_t rc = linux_copy_cstring(old_path, sizeof(old_path),
                                            (const char *)(uintptr_t)arg1);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_copy_cstring(new_path, sizeof(new_path),
                                    (const char *)(uintptr_t)arg2);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_path(old_path, sizeof(old_path));
            if (rc >= 0) {
                rc = linux_resolve_path(new_path, sizeof(new_path));
            }
            if (rc < 0) {
                result = rc;
                break;
            }
            result = (int64_t)syscall_file_rename(old_path, new_path);
            break;
        }

        case LINUX_SYS_MKDIR: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg1);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_path(path, sizeof(path));
            if (rc < 0) {
                result = rc;
                break;
            }
            result = (int64_t)syscall_file_mkdir(path);
            if (result == 0) {
                linux_apply_create_mode(path, arg2);
            }
            break;
        }

        /* link(2) / linkat(2): emulated as a content copy in the pseudo
         * filesystems (no real hard links). linkat's dir fds are treated as
         * AT_FDCWD and flags ignored. The X server's LockServer() needs this
         * to place /tmp/.Xn-lock. See syscall_file_link(). */
        case LINUX_SYS_LINK:
        case LINUX_SYS_LINKAT: {
            uint64_t old_ptr = (num == LINUX_SYS_LINK) ? arg1 : arg2;
            uint64_t new_ptr = (num == LINUX_SYS_LINK) ? arg2 : arg4;
            char old_path[256];
            char new_path[256];
            int64_t rc = linux_copy_cstring(old_path, sizeof(old_path),
                                            (const char *)(uintptr_t)old_ptr);
            if (rc < 0) { result = rc; break; }
            rc = linux_copy_cstring(new_path, sizeof(new_path),
                                    (const char *)(uintptr_t)new_ptr);
            if (rc < 0) { result = rc; break; }
            rc = linux_resolve_path(old_path, sizeof(old_path));
            if (rc >= 0) rc = linux_resolve_path(new_path, sizeof(new_path));
            if (rc < 0) { result = rc; break; }
            result = (int64_t)syscall_file_link(old_path, new_path);
            break;
        }

        case LINUX_SYS_RMDIR: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg1);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_path(path, sizeof(path));
            if (rc < 0) {
                result = rc;
                break;
            }
            vfs_file_t vf;
            if (vfs_find_file(path, &vf)) {
                result = LINUX_ENOTDIR;
                break;
            }
            result = (int64_t)syscall_file_unlink(path);
            break;
        }

        case LINUX_SYS_CREAT: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg1);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_path(path, sizeof(path));
            if (rc < 0) {
                result = rc;
                break;
            }
            result = (int64_t)syscall_file_creat(path);
            break;
        }

        case LINUX_SYS_UNLINK: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg1);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_path(path, sizeof(path));
            if (rc < 0) {
                result = rc;
                break;
            }
            result = linux_unlink_resolved(path);
            break;
        }

        case LINUX_SYS_READLINK:
            result = linux_readlink(arg1, arg2, arg3);
            break;

        case LINUX_SYS_SYMLINK:
            result = linux_symlink_at(arg1, (uint64_t)LINUX_AT_FDCWD, arg2);
            break;

        case LINUX_SYS_SYMLINKAT:
            /* symlinkat(target, newdirfd, linkpath) */
            result = linux_symlink_at(arg1, arg2, arg3);
            break;

        case LINUX_SYS_GETTIMEOFDAY:
            result = linux_gettimeofday(arg1);
            break;

        case LINUX_SYS_GETRLIMIT:
            result = syscall_prlimit64(0u, arg2, 0u, arg1);
            break;

        case LINUX_SYS_SYSINFO:
            result = linux_sysinfo(arg1);
            break;

        case LINUX_SYS_GETUID:
        case LINUX_SYS_GETGID:
        case LINUX_SYS_GETEUID:
        case LINUX_SYS_GETEGID: {
            uint32_t uid = 0, gid = 0;
            (void)process_get_credentials(process_get_current_pid(), &uid, &gid);
            result = (num == LINUX_SYS_GETUID || num == LINUX_SYS_GETEUID)
                         ? (int64_t)uid : (int64_t)gid;
            break;
        }

        case LINUX_SYS_GETPPID:
            result = (int64_t)process_getppid();
            break;

        case LINUX_SYS_STATFS: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg1);
            if (rc < 0) {
                result = rc;
                break;
            }
            result = linux_statfs(arg1, arg2);
            break;
        }

        case LINUX_SYS_FSTATFS:
            result = linux_fstatfs(arg1, arg2);
            break;

        case LINUX_SYS_SIGALTSTACK:
            result = linux_sigaltstack(arg1, arg2);
            break;

        case LINUX_SYS_PRCTL:
            result = linux_prctl(arg1, arg2, arg3, arg4, arg5);
            break;

        case LINUX_SYS_ARCH_PRCTL:
            result = syscall_arch_prctl(arg1, arg2);
            break;

        case LINUX_SYS_SETRLIMIT:
            result = 0;
            break;

        case LINUX_SYS_GETTID:
            result = syscall_gettid();
            break;

        case LINUX_SYS_TKILL:
            result = (int64_t)process_signal_deliver((int32_t)arg1,
                                                     (int32_t)arg2);
            if (result == 0 && (int32_t)arg1 == process_get_current_pid() &&
                process_signal_maybe_self_terminate((int32_t)arg2)) {
                request_switch = 1;
            }
            break;

        case LINUX_SYS_TIME:
            result = linux_realtime_seconds();
            break;

        case LINUX_SYS_FUTEX: {
            /* For FUTEX_WAIT and FUTEX_WAIT_BITSET the fourth argument is a
             * pointer to a struct timespec -- relative for WAIT, an absolute
             * deadline on CLOCK_MONOTONIC (CLOCK_REALTIME with
             * FUTEX_CLOCK_REALTIME) for WAIT_BITSET. It used to reach
             * syscall_futex_wait() as the pointer value itself, read as
             * nanoseconds: every timed wait lasted ~20 minutes whatever it
             * asked for. Chromium's threads that recover from a missed signal
             * by timing out never did, and the browser sat on a blank page. */
            uint64_t futex_cmd = arg2 & 0x7fULL;
            uint64_t futex_timeout = arg4;
            int futex_done = 0;
            int futex_restart = 0;
            /* A wait already queued by an earlier pass of this syscall is
             * finished (or resumed) before anything else looks at the
             * arguments -- in particular before the timeout below would be
             * re-derived from them. See syscall_futex_linux_resume(). */
            if ((futex_cmd == 0u || futex_cmd == 9u) &&
                syscall_futex_linux_resume(&result, &futex_restart)) {
                futex_done = 1;
            }
            if (!futex_done && (futex_cmd == 0u || futex_cmd == 9u) && arg4 != 0u) {
                struct { int64_t tv_sec; int64_t tv_nsec; } ts;
                if (copy_from_user(&ts, (const void *)(uintptr_t)arg4, sizeof(ts)) != 0u) {
                    result = LINUX_EFAULT;
                    futex_done = 1;
                } else if (ts.tv_sec < 0 || ts.tv_nsec < 0 || ts.tv_nsec >= 1000000000LL) {
                    result = LINUX_EINVAL;
                    futex_done = 1;
                } else {
                    int64_t ns = ts.tv_sec * 1000000000LL + ts.tv_nsec;
                    if (futex_cmd == 9u) {
                        int64_t now = (arg2 & 0x100u) != 0u /* FUTEX_CLOCK_REALTIME */
                            ? clock_realtime_ns()
                            : (int64_t)timer_monotonic_ns();
                        ns -= now;
                    }
                    if (ns <= 0) {
                        /* Already due: Linux still checks the value first. */
                        int32_t cur = 0;
                        if (copy_from_user(&cur, (const void *)(uintptr_t)arg1, sizeof(cur)) != 0u) {
                            result = LINUX_EFAULT;
                        } else {
                            result = (cur != (int32_t)arg3) ? LINUX_EAGAIN : -110LL /* ETIMEDOUT */;
                        }
                        futex_done = 1;
                    } else {
                        futex_timeout = (uint64_t)ns;
                    }
                }
            }
            if (!futex_done && (futex_cmd == 0u || futex_cmd == 9u)) {
                result = syscall_futex_wait_linux(
                    arg1, (int32_t)arg3, futex_timeout,
                    futex_cmd == 9u ? (uint32_t)arg6 : 0xFFFFFFFFu,
                    &futex_restart);
            } else if (!futex_done) {
                result = syscall_futex(arg1, arg2, arg3, futex_timeout, arg5, arg6);
            }
            if (futex_restart) {
                request_restart = 1;
                request_switch = 1;
                break;
            }
            uint64_t command = arg2 & 0x7fULL;
            /* 0=WAIT, 9=WAIT_BITSET, 6/13=LOCK_PI[2]: these can park the
             * calling thread, so yield the CPU on the way out. */
            if (result == 0 &&
                (command == 0u || command == 9u ||
                 command == 6u || command == 13u)) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_SCHED_SETAFFINITY:
            result = linux_sched_setaffinity(arg1, arg2, arg3);
            break;

        case LINUX_SYS_SCHED_GETAFFINITY:
            result = linux_sched_getaffinity(arg1, arg2, arg3);
            break;

        case LINUX_SYS_GETDENTS64:
            result = linux_getdents64((int32_t)arg1, arg2, arg3);
            break;

        case LINUX_SYS_SET_TID_ADDRESS:
            result = syscall_set_tid_address(arg1);
            break;

        case LINUX_SYS_CLOCK_GETTIME:
            result = syscall_clock_gettime((int32_t)arg1, arg2);
            break;

        case LINUX_SYS_CLOCK_GETRES:
            result = syscall_clock_getres((int32_t)arg1, arg2);
            break;

        case LINUX_SYS_EPOLL_WAIT: {
            int should_switch = 0;
            result = linux_epoll_wait(arg1, arg2, arg3, arg4, &should_switch,
                                      &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_EPOLL_CTL:
            result = linux_epoll_ctl(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_TGKILL: {
            int32_t tgid = (int32_t)arg1;
            int32_t tid = (int32_t)arg2;
            int32_t signum = (int32_t)arg3;
            if (!process_signal_validate_group(tgid, tid)) {
                result = LINUX_ESRCH;
                break;
            }
            if (signum == 0) {
                result = 0;
                break;
            }
            if (signum < 0 || signum >= (int32_t)PROCESS_SIGNAL_MAX) {
                result = LINUX_EINVAL;
                break;
            }
            result = (int64_t)process_signal_deliver(tid, signum);
            if (result == 0 && tid == process_get_current_pid() &&
                process_signal_maybe_self_terminate(signum)) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_OPENAT: {
            /* dirfd is ignored for an absolute pathname (POSIX); a relative
             * one resolves against the cwd (AT_FDCWD) or the directory the
             * dirfd was opened on. */
            char oa_path[256];
            int64_t rc = linux_copy_cstring(oa_path, sizeof(oa_path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) { result = rc; break; }
            rc = linux_resolve_at(arg1, oa_path, sizeof(oa_path));
            if (rc < 0) { result = rc; break; }
            result = linux_open_resolved(oa_path, arg3);
            break;
        }

        case LINUX_SYS_SET_ROBUST_LIST:
            result = (int64_t)process_set_robust_list(arg1, arg2);
            break;

        case LINUX_SYS_READLINKAT:
            result = linux_readlinkat(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_TIMERFD_CREATE: {
            (void)arg1;
            (void)arg2;
            result = (int64_t)syscall_file_create_timerfd();
            break;
        }

        case LINUX_SYS_TIMERFD_SETTIME: {
            struct {
                int64_t sec;
                int64_t nsec;
            } it_value;
            struct {
                int64_t sec;
                int64_t nsec;
            } it_interval;
            if (copy_from_user(&it_value, (const void *)(uintptr_t)arg3,
                               sizeof(it_value)) != 0u ||
                copy_from_user(&it_interval, (const void *)(uintptr_t)arg4,
                               sizeof(it_interval)) != 0u) {
                result = LINUX_EFAULT;
                break;
            }
            if (it_value.nsec < 0 || it_value.nsec >= 1000000000LL ||
                it_interval.nsec < 0 || it_interval.nsec >= 1000000000LL) {
                result = LINUX_EINVAL;
                break;
            }
            result = (int64_t)syscall_file_timerfd_settime(
                (int32_t)arg1,
                (uint64_t)it_value.sec, (uint64_t)it_value.nsec,
                (uint64_t)it_interval.sec, (uint64_t)it_interval.nsec);
            break;
        }

        case LINUX_SYS_TIMERFD_GETTIME: {
            uint64_t value_sec = 0;
            uint64_t value_nsec = 0;
            uint64_t interval_sec = 0;
            uint64_t interval_nsec = 0;
            result = (int64_t)syscall_file_timerfd_gettime(
                (int32_t)arg1, &value_sec, &value_nsec,
                &interval_sec, &interval_nsec);
            if (result == 0) {
                struct {
                    int64_t sec;
                    int64_t nsec;
                } value;
                struct {
                    int64_t sec;
                    int64_t nsec;
                } interval;
                value.sec = (int64_t)value_sec;
                value.nsec = (int64_t)value_nsec;
                interval.sec = (int64_t)interval_sec;
                interval.nsec = (int64_t)interval_nsec;
                if (copy_to_user((void *)(uintptr_t)arg2, &value,
                                 sizeof(value)) != 0u ||
                    copy_to_user((void *)(uintptr_t)arg3, &interval,
                                 sizeof(interval)) != 0u) {
                    result = LINUX_EFAULT;
                }
            }
            break;
        }

        case LINUX_SYS_SIGNALFD4: {
            if (arg3 != sizeof(uint64_t)) {
                result = LINUX_EINVAL;
                break;
            }
            uint64_t mask = 0;
            if (copy_from_user(&mask, (const void *)(uintptr_t)arg2,
                               sizeof(mask)) != 0u) {
                result = LINUX_EFAULT;
                break;
            }
            result = (int64_t)syscall_file_create_signalfd(mask);
            break;
        }

        case LINUX_SYS_MEMFD_CREATE: {
            /* MFD_CLOEXEC | MFD_ALLOW_SEALING | MFD_HUGETLB | MFD_NOEXEC_SEAL
             * | MFD_EXEC. Unknown bits are EINVAL, and that has to be real:
             * Mojo probes the kernel by calling memfd_create() with every flag
             * bit set and CHECK-fails the process if it succeeds
             * (ChannelLinux::KernelSupportsUpgradeRequirements(),
             * channel_linux.cc:947). */
            const uint64_t mfd_known = 0x1u | 0x2u | 0x4u | 0x8u | 0x10u;
            if ((arg2 & ~mfd_known) != 0u) {
                result = LINUX_EINVAL;
                break;
            }
            result = (int64_t)syscall_file_create_memfd(
                (const char *)(uintptr_t)arg1);
            if (result >= 0 && (arg2 & 0x1u) != 0u) {
                (void)syscall_file_set_descriptor_flags((int32_t)result,
                                                        LINUX_FD_CLOEXEC);
            }
        }
#if CHROME_SHM_TRACE
            chrome_shm_trace3("memfd_create flags=", arg2, " -> ", (uint64_t)result, "", 0);
#endif
            break;

        case LINUX_SYS_STATX:
            result = linux_statx(arg1, arg2, arg3, arg4, arg5);
            break;

        case LINUX_SYS_RSEQ:
            result = linux_rseq(arg1, arg2, arg3, (uint32_t)arg4);
            break;

        case LINUX_SYS_EVENTFD:
            result = (int64_t)syscall_eventfd(arg1, 0u);
            break;

        case LINUX_SYS_EVENTFD2: {
            /* EFD_SEMAPHORE | EFD_NONBLOCK | EFD_CLOEXEC. As with
             * memfd_create() above, Mojo calls this with every bit set and
             * requires it to fail. */
            const uint64_t efd_known = 0x1u | 0x800u | 0x80000u;
            if ((arg2 & ~efd_known) != 0u) {
                result = LINUX_EINVAL;
                break;
            }
            result = (int64_t)syscall_eventfd(arg1, arg2);
            break;
        }

        case LINUX_SYS_EPOLL_CREATE1:
            result = (int64_t)syscall_epoll_create(arg1);
            break;

        case LINUX_SYS_PRLIMIT64:
            result = syscall_prlimit64(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_GETCPU:
            result = linux_getcpu(arg1, arg2, arg3);
            break;

        case LINUX_SYS_GETRANDOM:
            result = syscall_getrandom(arg1, arg2, arg3);
            break;

        case LINUX_SYS_SOCKET:
            result = linux_socket_create(arg1, arg2, arg3);
            break;

        case LINUX_SYS_SOCKETPAIR:
            result = linux_socketpair(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_BIND:
            result = linux_socket_bind(arg1, arg2, arg3);
            break;

        case LINUX_SYS_CONNECT:
            result = linux_socket_connect(arg1, arg2, arg3);
            break;

        case LINUX_SYS_LISTEN:
            if (unix_socket_fd_in_range((int32_t)arg1)) {
                result = unix_socket_listen((int32_t)arg1, (int32_t)arg2);
            } else {
                result = (int64_t)syscall_socket_listen((int32_t)arg1);
            }
            break;

        case LINUX_SYS_ACCEPT:
            result = linux_socket_accept(arg1, arg2, arg3);
            break;

        case 288u: /* accept4 */
            result = linux_socket_accept4(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_SENDTO:
            result = linux_socket_sendto(arg1, arg2, arg3, arg4, arg5, arg6);
            break;

        case LINUX_SYS_RECVFROM: {
            int should_switch = 0;
            result = linux_socket_recvfrom(arg1, arg2, arg3, arg4, arg5, arg6);
            if ((arg4 & LINUX_MSG_DONTWAIT) == 0u) {
                linux_unix_block_retry((int32_t)arg1, &result, &should_switch,
                                       &request_restart);
            }
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_SENDMSG:
            /* Only AF_UNIX (Mojo IPC's SCM_RIGHTS fd-passing transport)
             * is supported; sendmsg() on TCP/UDP sockets is not. The
             * 56-byte size matches glibc's struct msghdr (see the layout
             * comment in UnixSocket.c) - only the top-level struct is
             * validated here, not the msg_iov/msg_control buffers it
             * points to (a pre-existing gap in unix_socket_sendmsg()). */
            if (syscall_socket_fd_in_range((int32_t)arg1)) {
                result = linux_inet_sendmsg(arg1, arg2, arg3);
            } else if (!unix_socket_fd_in_range((int32_t)arg1)) {
                result = LINUX_ENOTSUP;
            } else if (!process_user_buffer_is_valid(
                           (const void *)(uintptr_t)arg2, 56u)) {
                result = LINUX_EFAULT;
            } else {
                result = unix_socket_sendmsg((int32_t)arg1, arg2);
            }
            break;

        case LINUX_SYS_RECVMSG: {
            int should_switch = 0;
            if (syscall_socket_fd_in_range((int32_t)arg1)) {
                result = linux_inet_recvmsg(arg1, arg2, arg3);
            } else if (!unix_socket_fd_in_range((int32_t)arg1)) {
                result = LINUX_ENOTSUP;
            } else if (!process_user_buffer_is_valid(
                           (const void *)(uintptr_t)arg2, 56u)) {
                result = LINUX_EFAULT;
            } else {
                result = unix_socket_recvmsg((int32_t)arg1, arg2);
                if ((arg3 & LINUX_MSG_DONTWAIT) == 0u) {
                    linux_unix_block_retry((int32_t)arg1, &result,
                                           &should_switch, &request_restart);
                }
            }
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_SHUTDOWN:
            if (unix_socket_fd_in_range((int32_t)arg1)) {
                /* AF_UNIX socketpair half-close. We don't model half-open
                 * AF_UNIX channels, but Chromium's sandbox host CHECKs that
                 * shutdown(SHUT_RD) on its broker socket returns 0
                 * (sandbox_host_linux.cc:41). Accept any valid `how`. */
                result = ((int32_t)arg2 >= LINUX_SHUT_RD &&
                          (int32_t)arg2 <= LINUX_SHUT_RDWR)
                             ? 0
                             : LINUX_EINVAL;
            } else {
                result = (int64_t)syscall_socket_shutdown((int32_t)arg1,
                                                          (int32_t)arg2);
            }
            break;

        case LINUX_SYS_SETSOCKOPT:
            result = linux_socket_setsockopt(arg1, arg2, arg3, arg4, arg5);
            break;

        case LINUX_SYS_GETSOCKOPT:
            result = linux_socket_getsockopt(arg1, arg2, arg3, arg4, arg5);
            break;

        case LINUX_SYS_GETSOCKNAME:
            result = linux_getsockname_common(arg1, arg2, arg3, 0);
            break;

        case LINUX_SYS_GETPEERNAME:
            result = linux_getsockname_common(arg1, arg2, arg3, 1);
            break;

        case LINUX_SYS_PREAD64:
            result = linux_pread64(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_PWRITE64:
            result = linux_pwrite64(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_PIPE2:
            result = linux_pipe2(arg1, arg2);
            break;

        case LINUX_SYS_DUP3:
            result = linux_dup3(arg1, arg2, arg3);
            break;

        case LINUX_SYS_FACCESSAT:
        case LINUX_SYS_FACCESSAT2:
            result = linux_faccessat(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_CLOCK_NANOSLEEP: {
            int should_switch = 0;
            result = linux_clock_nanosleep(arg1, arg2, arg3, arg4, &should_switch);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_EPOLL_PWAIT: {
            /* Identical to epoll_wait; the signal mask (arg5/arg6) is
             * ignored because signals are delivered synchronously here. */
            int should_switch = 0;
            result = linux_epoll_wait(arg1, arg2, arg3, arg4, &should_switch,
                                      &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_EPOLL_PWAIT2: {
            /* arg4 is a `const struct timespec *` here instead of a ms
             * count. Translate to milliseconds (NULL == block -> we treat
             * as 0, matching the non-blocking spirit of epoll here). */
            int should_switch = 0;
            uint64_t timeout_ms = 0;
            if (arg4 != 0u) {
                struct { int64_t sec; int64_t nsec; } ts;
                if (copy_from_user(&ts, (const void *)(uintptr_t)arg4,
                                   sizeof(ts)) == 0u && ts.sec >= 0 &&
                    ts.nsec >= 0) {
                    timeout_ms = (uint64_t)ts.sec * 1000u +
                                 ((uint64_t)ts.nsec + 999999u) / 1000000u;
                }
            }
            result = linux_epoll_wait(arg1, arg2, arg3, timeout_ms,
                                      &should_switch, &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_SELECT: {
            /* arg5 is `struct timeval *`; NULL means block. */
            int should_switch = 0;
            int64_t timeout_ms = -1;
            if (arg5 != 0u) {
                struct { int64_t sec; int64_t usec; } tv;
                if (copy_from_user(&tv, (const void *)(uintptr_t)arg5,
                                   sizeof(tv)) != 0u) {
                    result = LINUX_EFAULT;
                    break;
                }
                timeout_ms = (tv.sec >= 0 && tv.usec >= 0)
                                 ? (tv.sec * 1000 + (tv.usec + 999) / 1000)
                                 : 0;
            }
            result = linux_select_common(arg1, arg2, arg3, arg4, timeout_ms,
                                         &should_switch, &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_PSELECT6: {
            /* arg5 is `struct timespec *`; the arg6 sigmask is ignored, as it
             * is for ppoll/epoll_pwait (signals are delivered synchronously). */
            int should_switch = 0;
            int64_t timeout_ms = -1;
            if (arg5 != 0u) {
                struct { int64_t sec; int64_t nsec; } ts;
                if (copy_from_user(&ts, (const void *)(uintptr_t)arg5,
                                   sizeof(ts)) != 0u) {
                    result = LINUX_EFAULT;
                    break;
                }
                timeout_ms = (ts.sec >= 0 && ts.nsec >= 0)
                                 ? (ts.sec * 1000 + (ts.nsec + 999999) / 1000000)
                                 : 0;
            }
            result = linux_select_common(arg1, arg2, arg3, arg4, timeout_ms,
                                         &should_switch, &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_POLL: {
            int should_switch = 0;
            result = linux_poll_common(arg1, arg2, (int64_t)(int32_t)arg3,
                                       &should_switch, &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_PPOLL: {
            int should_switch = 0;
            result = linux_ppoll(arg1, arg2, arg3, &should_switch,
                                 &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_WAITID: {
            int should_switch = 0;
            result = linux_waitid(arg1, arg2, arg3, arg4, arg5, &should_switch,
                                  &request_restart);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_GETRUSAGE:
            result = linux_getrusage(arg1, arg2);
            break;

        case LINUX_SYS_SENDMMSG:
            result = linux_sendmmsg(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_RECVMMSG:
            result = linux_recvmmsg(arg1, arg2, arg3, arg4);
            break;

        case LINUX_SYS_INOTIFY_INIT:
        case LINUX_SYS_INOTIFY_INIT1:
            result = linux_inotify_init();
            break;

        case LINUX_SYS_INOTIFY_ADD_WATCH:
            result = linux_inotify_add_watch();
            break;

        case LINUX_SYS_INOTIFY_RM_WATCH:
            result = 0;
            break;

        case LINUX_SYS_PERSONALITY:
            /* Report/keep PER_LINUX (0). ADDR_NO_RANDOMIZE and friends are
             * accepted silently - there is no ASLR to toggle. */
            result = 0;
            break;

        case LINUX_SYS_GETPGID:
        case LINUX_SYS_GETPGRP:
        case LINUX_SYS_GETSID:
            /* No process-group/session tracking: model every process as
             * its own group and session leader. */
            result = (int64_t)process_get_current_pid();
            break;

        case LINUX_SYS_SETPGID:
            result = 0;
            break;

        case LINUX_SYS_SETSID:
            result = (int64_t)process_get_current_pid();
            break;

        case LINUX_SYS_UMASK:
            /* No per-process umask is stored; report the conventional 022
             * and accept (ignore) the new value. */
            result = 0x12;
            break;

        case LINUX_SYS_FSYNC:
        case LINUX_SYS_FDATASYNC:
        case LINUX_SYS_SYNCFS:
        case LINUX_SYS_FLOCK:
        case LINUX_SYS_FADVISE64:
        case LINUX_SYS_MLOCK:
        case LINUX_SYS_MUNLOCK:
        case LINUX_SYS_MLOCKALL:
        case LINUX_SYS_MUNLOCKALL:
        case LINUX_SYS_MLOCK2:
        case LINUX_SYS_UTIMENSAT:
        /* chmod(2)/fchmodat(2): store the bits on a filesystem that keeps
           them (tmpfs) and otherwise accept-and-ignore as before. fchmod(2)
           still has nothing to key on, so it stays a no-op success. */
        case LINUX_SYS_CHMOD:
        case LINUX_SYS_FCHMODAT: {
            uint64_t path_arg = (num == LINUX_SYS_CHMOD) ? arg1 : arg2;
            uint64_t mode_arg = (num == LINUX_SYS_CHMOD) ? arg2 : arg3;
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)path_arg);
            if (rc == 0 && linux_resolve_path(path, sizeof(path)) >= 0) {
                (void)vfs_set_mode(path, (uint32_t)(mode_arg & 07777u));
            }
            result = 0;
            break;
        }

        case LINUX_SYS_FCHMOD:
        case LINUX_SYS_FCHOWN:
        case LINUX_SYS_CHOWN:
        case LINUX_SYS_LCHOWN:
        case LINUX_SYS_FCHOWNAT:
        case LINUX_SYS_SETGROUPS:
        case LINUX_SYS_SYSLOG:
            result = 0;
            break;

        case LINUX_SYS_SYNC:
            result = 0;
            break;

        case LINUX_SYS_MEMBARRIER:
            /* cmd 0 == MEMBARRIER_CMD_QUERY: advertise the two portable
             * commands. Any actual barrier is a no-op on the single shared
             * address space model used here. */
            result = (arg1 == 0u) ? (int64_t)((1 << 1) | (1 << 3)) : 0;
            break;

        case LINUX_SYS_SETUID:
        case LINUX_SYS_SETGID:
        case LINUX_SYS_SETREUID:
        case LINUX_SYS_SETREGID:
        case LINUX_SYS_SETRESUID:
        case LINUX_SYS_SETRESGID: {
            /* Real, effective and saved ids are one value here; take the
             * effective one the call names (-1 keeps the current value).
             * Nothing enforces permissions, so every transition succeeds. */
            uint32_t want;
            if (num == LINUX_SYS_SETUID || num == LINUX_SYS_SETGID) {
                want = (uint32_t)arg1;
            } else {
                want = (uint32_t)arg2; /* euid/egid position */
                if (want == 0xFFFFFFFFu) want = (uint32_t)arg1;
            }
            int is_uid = (num == LINUX_SYS_SETUID || num == LINUX_SYS_SETREUID ||
                          num == LINUX_SYS_SETRESUID);
            (void)process_set_current_credentials(is_uid ? want : 0xFFFFFFFFu,
                                                  is_uid ? 0xFFFFFFFFu : want);
            result = 0;
            break;
        }

        case LINUX_SYS_GETRESUID:
        case LINUX_SYS_GETRESGID: {
            uint32_t cu = 0, cg = 0;
            (void)process_get_credentials(process_get_current_pid(), &cu, &cg);
            uint32_t v = (num == LINUX_SYS_GETRESUID) ? cu : cg;
            uint32_t ids[3] = {v, v, v};
            if ((arg1 != 0u &&
                 copy_to_user((void *)(uintptr_t)arg1, &ids[0], 4u) != 0u) ||
                (arg2 != 0u &&
                 copy_to_user((void *)(uintptr_t)arg2, &ids[1], 4u) != 0u) ||
                (arg3 != 0u &&
                 copy_to_user((void *)(uintptr_t)arg3, &ids[2], 4u) != 0u)) {
                result = LINUX_EFAULT;
            } else {
                result = 0;
            }
            break;
        }

        case LINUX_SYS_GETGROUPS:
            result = 0; /* No supplementary groups. */
            break;

        case LINUX_SYS_GETPRIORITY:
            result = 20; /* getpriority returns 20-nice; nice == 0. */
            break;

        case LINUX_SYS_SETPRIORITY:
            result = 0;
            break;

        case LINUX_SYS_SCHED_GETSCHEDULER:
            result = 0; /* SCHED_OTHER */
            break;

        case LINUX_SYS_SCHED_SETSCHEDULER:
        case LINUX_SYS_SCHED_SETPARAM:
            result = 0;
            break;

        case LINUX_SYS_SCHED_GETPARAM:
            if (arg2 != 0u) {
                int32_t prio = 0;
                result = copy_to_user((void *)(uintptr_t)arg2, &prio,
                                      sizeof(prio)) != 0u ? LINUX_EFAULT : 0;
            } else {
                result = 0;
            }
            break;

        case LINUX_SYS_SCHED_GET_PRIORITY_MAX:
            /* SCHED_FIFO(1)/SCHED_RR(2) -> 99, everything else -> 0. */
            result = (arg1 == 1u || arg1 == 2u) ? 99 : 0;
            break;

        case LINUX_SYS_SCHED_GET_PRIORITY_MIN:
            result = (arg1 == 1u || arg1 == 2u) ? 1 : 0;
            break;

        case LINUX_SYS_CAPGET:
            result = linux_capget(arg1, arg2);
            break;

        case LINUX_SYS_CAPSET:
            result = linux_capset(arg1, arg2);
            break;

        case LINUX_SYS_UNSHARE:
            result = linux_unshare(arg1);
            break;

        case LINUX_SYS_CHROOT:
            result = linux_chroot(arg1);
            break;

        case LINUX_SYS_SECCOMP:
            result = linux_seccomp(arg1);
            break;

        case LINUX_SYS_SETNS:
            /* There is nothing to join: namespaces are not modelled. */
            result = LINUX_EINVAL;
            break;

        case LINUX_SYS_PIVOT_ROOT:
            /* chroot(2) is the path Chromium actually takes; pivot_root
             * needs a real mount tree, which this kernel does not have. */
            result = LINUX_EPERM;
            break;

        case LINUX_SYS_MOUNT:
        case LINUX_SYS_UMOUNT2:
            /* The mount table is fixed at boot (VFS_Pseudo.c). Accepting a
             * mount silently would be a lie; refuse the way a container
             * without CAP_SYS_ADMIN does, which callers expect. */
            result = LINUX_EPERM;
            break;

        case LINUX_SYS_PTRACE:
            /* No debugger interface. Crashpad only needs this on the path it
             * takes *after* a crash, and reports the failure rather than
             * dying of it. */
            result = LINUX_EPERM;
            break;

        case LINUX_SYS_NAME_TO_HANDLE_AT:
            /* "This filesystem does not support file handles" is a normal
             * answer on Linux and the one callers have a fallback for. */
            result = LINUX_ENOTSUP;
            break;

        case LINUX_SYS_RT_SIGPENDING:
            if (arg1 != 0u && arg2 == sizeof(uint64_t)) {
                uint64_t none = 0;
                result = copy_to_user((void *)(uintptr_t)arg1, &none,
                                      sizeof(none)) != 0u ? LINUX_EFAULT : 0;
            } else {
                result = LINUX_EINVAL;
            }
            break;

        case LINUX_SYS_MKDIRAT: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) { result = rc; break; }
            rc = linux_resolve_at(arg1, path, sizeof(path));
            if (rc < 0) { result = rc; break; }
            result = (int64_t)syscall_file_mkdir(path);
            if (result == 0) {
                linux_apply_create_mode(path, arg3);
            }
            break;
        }

        case LINUX_SYS_UNLINKAT: {
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) { result = rc; break; }
            rc = linux_resolve_at(arg1, path, sizeof(path));
            if (rc < 0) { result = rc; break; }
            result = linux_unlink_resolved(path);
            break;
        }

        case LINUX_SYS_RENAMEAT:
        case LINUX_SYS_RENAMEAT2: {
            char old_path[256];
            char new_path[256];
            int64_t rc = linux_copy_cstring(old_path, sizeof(old_path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) { result = rc; break; }
            rc = linux_copy_cstring(new_path, sizeof(new_path),
                                    (const char *)(uintptr_t)arg4);
            if (rc < 0) { result = rc; break; }
            rc = linux_resolve_at(arg1, old_path, sizeof(old_path));
            if (rc >= 0) rc = linux_resolve_at(arg3, new_path, sizeof(new_path));
            if (rc < 0) { result = rc; break; }
            result = (int64_t)syscall_file_rename(old_path, new_path);
            break;
        }

        default:
            result = LINUX_ENOSYS;
            break;
    }

    if (!request_restart) {
        result = lx_fd_post(num, lx_orig, result, lx_cloexec);
    }
lx_fd_finished:

    /* Name every syscall this layer does not implement, once each. An
     * external Linux binary that dies for want of a syscall otherwise says
     * nothing useful: glibc turns ENOSYS into an ordinary errno and the
     * caller reports its own high-level failure (or none at all). One bit
     * per number keeps this allocation-free and silent after the first hit,
     * so it can stay on outside bring-up boots. Numbers above the bitmap are
     * rare enough to report every time. */
    if (result == LINUX_ENOSYS) {
        static uint8_t reported[512u / 8u];
        bool first = true;
        if (num < 512u) {
            uint8_t bit = (uint8_t)(1u << (num & 7u));
            if ((reported[num >> 3u] & bit) != 0u) {
                first = false;
            } else {
                reported[num >> 3u] |= bit;
            }
        }
        if (first) {
            serial_write_string("[lxsys] ENOSYS #");
            serial_write_uint64(num);
            serial_write_string(" (");
            serial_write_uint64(arg1);
            serial_write_string(",");
            serial_write_uint64(arg2);
            serial_write_string(",");
            serial_write_uint64(arg3);
            serial_write_string(")\n");
        }
    }

    /* Bring-up trace (TODO_Doom_Xorg_MethodA.md M8). Xorg can only run
     * xkbcomp through Popen(), i.e. pipe() + fork() + dup2() +
     * execve("/bin/sh"), and when any one of those fails it reports nothing
     * but "XKB: Could not invoke xkbcomp" -- which is then fatal. Name the
     * failing call here instead of guessing. Restricted to this handful of
     * process-spawn syscalls and to failures only, so it stays silent once
     * the path works. */
    if (result < 0 && !request_restart) {
        switch (num) {
        case LINUX_SYS_PIPE:
        case LINUX_SYS_PIPE2:
        case LINUX_SYS_CLONE:
        case LINUX_SYS_FORK:
        case LINUX_SYS_VFORK:
        case LINUX_SYS_EXECVE:
        case LINUX_SYS_DUP2:
        case LINUX_SYS_DUP3:
        case LINUX_SYS_SETITIMER:
        case LINUX_SYS_WAIT4:
            serial_write_string("[lxproc] syscall ");
            serial_write_uint64(num);
            serial_write_string(" failed -> ");
            serial_write_uint64((uint64_t)result);
            serial_write_char('\n');
            break;
        default:
            break;
        }
    }

    LINUX_TRACE_EXIT(num, result);
    /* Skip in-kernel restarts: a blocking wait4() or futex re-runs from the
     * top every slice, and logging each pass buries the one line that matters
     * under a hundred identical ones. User space only ever sees the last. */
    if (!request_restart) {
        lx_newborn_note(num, arg1, arg2, result);
    }

    if (request_restart) {
        linux_syscall_restart(saved_rsp, num);
    } else {
        linux_syscall_result(saved_rsp, result);
    }
    return (uint64_t)(uint32_t)request_switch;
}

static const compat_layer_t g_linux_compat_layer = {
    .abi = PROCESS_ABI_LINUX,
    .name = "Linux",
    .dispatch = linux_syscall_dispatch,
};

void linux_compat_layer_register(void)
{
    lx_fd_hooks_register();
    (void)compat_registry_register(&g_linux_compat_layer);
}

uint32_t linux_heartbeat_last_num(int32_t pid)
{
    if (pid < 0 || (uint32_t)pid >= LX_HEARTBEAT_MAX_PID) return 0xFFFFFFFFu;
    return g_lx_hb_last_num[pid];
}

uint64_t linux_heartbeat_count(int32_t pid)
{
    if (pid < 0 || (uint32_t)pid >= LX_HEARTBEAT_MAX_PID) return 0u;
    return g_lx_hb_count[pid];
}

uint64_t linux_heartbeat_last_arg(int32_t pid)
{
    if (pid < 0 || (uint32_t)pid >= LX_HEARTBEAT_MAX_PID) return 0u;
    return g_lx_hb_last_arg[pid];
}

uint64_t linux_heartbeat_last_rip(int32_t pid)
{
    if (pid < 0 || (uint32_t)pid >= LX_HEARTBEAT_MAX_PID) return 0u;
    return g_lx_hb_last_rip[pid];
}
