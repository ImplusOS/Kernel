#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Compat/Linux/Syscall_LinuxCompat.h"
#include "Compat/compat_registry.h"
#include "Core/process/ProcessManager.h"
#include "Core/process/ProcessScheduler.h"
#include "Core/syscall/Syscall_File.h"
#include "Core/syscall/Syscall_Socket.h"
#include "Core/syscall/Syscall_Epoll.h"
#include "Core/syscall/Syscall_Clock.h"
#include "Core/syscall/Syscall_Futex.h"
#include "Core/syscall/Syscall_VM.h"
#include "Core/syscall/Syscall_Main.h"
#include "Core/memory/SharedMemory.h"
#include "Core/memory/FileMap.h"
#include "Core/timer/Timer.h"
#include "Core/usercopy/Usercopy.h"
#include "Core/vfs/VFS.h"
#include "Core/vfs/DevFS.h"
#include "Core/vfs/ProcFS.h"
#include "IPC/UnixSocket.h"
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
#define LINUX_F_SETFL  4
#define LINUX_F_DUPFD_CLOEXEC 1030
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

int64_t syscall_prlimit64(uint64_t pid, uint64_t resource, uint64_t new_limit, uint64_t old_limit)
{
    int32_t current = process_get_current_pid();
    if (pid != 0u && (int32_t)pid != current) return LINUX_ESRCH;
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
        copy_to_user((void *)(uintptr_t)old_limit, &limit, sizeof(limit)) != 0u) {
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
            if (fd <= 2) {
                return (int64_t)total;
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
            if (fd <= 2) {
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

/* Terminal ioctls (TODO_Chromium_LinuxABI.md section 4). None of the fds
 * exposed here are real ttys, so TCGETS/TCSETS* must report ENOTTY - that
 * is exactly the signal isatty()/Chromium's base::IsTerminal() look for.
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
            for (uint64_t i = 0; i < n; ++i) {
                serial_write_char((char)chunk[i]);
            }
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
#define LINUX_SYS_GETCWD        79u
#define LINUX_SYS_CHDIR         80u
#define LINUX_SYS_RENAME        82u
#define LINUX_SYS_MKDIR         83u
#define LINUX_SYS_RMDIR         84u
#define LINUX_SYS_CREAT         85u
#define LINUX_SYS_LINK          86u
#define LINUX_SYS_UNLINK        87u
#define LINUX_SYS_READLINK      89u
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
#define LINUX_CLONE_PARENT_SETTID   0x00008000u
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
    return 0;
}

static int64_t linux_days_from_civil(int64_t year, unsigned month,
                                     unsigned day)
{
    year -= (month <= 2u) ? 1 : 0;
    int64_t era = (year >= 0 ? year : year - 399) / 400;
    unsigned yoe = (unsigned)(year - era * 400);
    unsigned doy =
        (153u * (month + (month > 2u ? 0u : 9u)) + 2u) / 5u + day - 1u;
    unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

static int64_t linux_realtime_seconds(void)
{
    rtc_time_t rtc;
    rtc_read_time(&rtc);
    int64_t days = linux_days_from_civil((int64_t)rtc.year,
                                         rtc.month, rtc.day);
    return days * 86400 + (int64_t)rtc.hour * 3600 +
           (int64_t)rtc.minute * 60 + (int64_t)rtc.second;
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
 * already coherent. */
#define LINUX_MSHARED_MAX 96
typedef struct {
    int32_t  in_use;
    int32_t  owner_pid;
    int32_t  fd;          /* our dup'd copy */
    uint64_t uaddr;
    uint64_t length;
    uint64_t file_offset;
    uint64_t writeback_len; /* min(length, file bytes from offset at map time) */
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

static void linux_mshared_register(int32_t owner_pid, int32_t src_fd,
                                   uint64_t uaddr, uint64_t length,
                                   uint64_t file_offset, uint64_t writeback_len)
{
    linux_mshared_init_once();
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

        spinlock_lock(&g_linux_mshared_lock);
        linux_mshared_t *e = &g_linux_mshared[i];
        if (e->in_use && e->owner_pid == owner_pid &&
            e->uaddr < hi && (e->uaddr + e->length) > lo) {
            fd = e->fd;
            uaddr = e->uaddr;
            wlen = e->writeback_len;
            foff = e->file_offset;
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
        while (done < wlen) {
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

static int32_t g_module_map_fds[64];
static char    g_module_map_names[64][64];

static void linux_module_map_note_open(int32_t fd, const char *path)
{
    if (fd < 0 || !linux_path_is_shared_object(path)) return;
    uint64_t n = strlen(path);
    uint64_t start = 0;
    for (uint64_t i = 0; i < n; ++i) if (path[i] == '/') start = i + 1u;
    for (uint32_t i = 0; i < 64u; ++i) {
        if (g_module_map_fds[i] == 0 || g_module_map_fds[i] == fd + 1) {
            g_module_map_fds[i] = fd + 1;
            uint64_t j = 0;
            for (; j < 63u && path[start + j]; ++j) {
                g_module_map_names[i][j] = path[start + j];
            }
            g_module_map_names[i][j] = '\0';
            return;
        }
    }
}

static const char *linux_module_map_name(int32_t fd)
{
    for (uint32_t i = 0; i < 64u; ++i) {
        if (g_module_map_fds[i] == fd + 1) return g_module_map_names[i];
    }
    return NULL;
}

static void linux_module_map_note_mmap(int32_t fd, uint64_t base, uint64_t len,
                                       uint64_t offset)
{
    const char *name = linux_module_map_name(fd);
    if (name == NULL) return;
    serial_write_string("[lxmap] ");
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

static int64_t linux_mmap(uint64_t addr, uint64_t length, uint64_t prot,
                          uint64_t flags, uint64_t fd, uint64_t offset)
{
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
            void *p = shared_memory_map(shm_handle);
            if (p == NULL) {
                return LINUX_ENOMEM;
            }
            return (int64_t)(uintptr_t)p;
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
                                   offset, read_len);
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
 * syscall_epoll_wait_ex() uses, since this path can't block internally. */
#define LINUX_POLLIN   0x0001
#define LINUX_POLLPRI  0x0002
#define LINUX_POLLOUT  0x0004
#define LINUX_POLLERR  0x0008
#define LINUX_POLLHUP  0x0010
#define LINUX_POLLNVAL 0x0020
#define LINUX_POLL_MAX_FDS  256u
#define LINUX_POLL_SLICE_MS 8u

typedef struct {
    int32_t fd;
    int16_t events;
    int16_t revents;
} linux_pollfd_t;

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

    if (nfds == 0u) {
        if (timeout_ms != 0 &&
            process_sleep_current_ms(slice_ms) == 0 &&
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
        uint32_t r = syscall_poll_one_fd(pfds[i].fd,
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
    if (process_sleep_current_ms(slice_ms) == 0 && should_switch_out != NULL) {
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
    int64_t tv[2];
    tv[0] = linux_realtime_seconds();
    tv[1] = 0;
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
    if ((flags & (LINUX_CLONE_VM | LINUX_CLONE_THREAD)) == 0u) {
        int32_t child_pid = process_fork();
        if (child_pid < 0) {
            return LINUX_EAGAIN;
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
        (void)process_set_clear_child_tid(child_tid);
    }
    return (int64_t)tid;
}

static int64_t linux_read(uint64_t fd, uint64_t buf, uint64_t count)
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
    return (int64_t)syscall_file_read((int32_t)fd,
                                      (uint8_t *)(uintptr_t)buf, count);
}

static int64_t linux_resolve_path(char *path, uint64_t capacity)
{
    if (path[0] == '/') {
        return 0;
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
    int64_t result;
    if ((flags & LINUX_O_DIRECTORY) != 0u) {
        result = (int64_t)syscall_file_register_dir(path);
    } else {
        result = (int64_t)syscall_file_open(path, flags);
        if (result == LINUX_ENOENT && (flags & LINUX_O_CREAT) != 0u) {
            result = (int64_t)syscall_file_creat(path);
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
    st->st_uid = 0;
    st->st_gid = 0;
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

static int64_t linux_stat_path(const char *path, uint64_t statbuf_ptr)
{
    if (statbuf_ptr == 0u ||
        !process_user_buffer_is_valid((const void *)(uintptr_t)statbuf_ptr,
                                      sizeof(linux_stat64_t))) {
        return LINUX_EFAULT;
    }
    linux_stat64_t st;
    memset(&st, 0, sizeof(st));
    vfs_file_t vf;
    if (vfs_find_file(path, &vf)) {
        linux_stat_fill_common_ino(&st, vf.size, vf.internal_id);
        if (devfs_path_is_device(path)) {
            st.st_mode = LINUX_S_IFCHR | 0x1B6u; /* crw-rw-rw- */
            st.st_size = 0;
            st.st_rdev = 0x0105u; /* arbitrary but stable device number */
        } else {
            st.st_mode = LINUX_S_IFREG | 0x1A4u;
        }
    } else {
        int32_t dir_handle = vfs_opendir(path);
        if (dir_handle < 0) {
            return LINUX_ENOENT;
        }
        (void)vfs_closedir(dir_handle);
        linux_stat_fill_common(&st, 0);
        st.st_mode = LINUX_S_IFDIR | 0x1EDu;
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
            char name[16];
            if (linux_copy_cstring(name, sizeof(name),
                                   (const char *)(uintptr_t)arg2) < 0) {
                return LINUX_EFAULT;
            }
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
    if (base_type != LINUX_SOCK_STREAM && base_type != 5u) {
        return LINUX_EPROTONOSUPPORT;
    }
    if (!process_user_buffer_is_valid((void *)(uintptr_t)fds_ptr,
                                      sizeof(int32_t) * 2u)) {
        return LINUX_EFAULT;
    }
    int32_t fds[2];
    int64_t rc = unix_socket_pair(fds);
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
        /* SOCK_STREAM(1) and SOCK_SEQPACKET(5) both map onto the
         * existing byte-stream Unix socket implementation - Mojo IPC
         * (TODO_Chromium_LinuxABI.md 3.7) mostly cares that framing is
         * preserved by the *application*, not the kernel. */
        if (base_type != LINUX_SOCK_STREAM && base_type != 5u) {
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
    if (procfs_readlink(path, target, sizeof(target)) < 0) {
        return LINUX_ENOENT;
    }
    uint64_t len = strlen(target);
    if (len > bufsiz) {
        len = bufsiz;
    }
    if (copy_to_user_trusted((void *)(uintptr_t)buf, target, len) != 0u) {
        return LINUX_EFAULT;
    }
    return (int64_t)len; /* readlink() does NOT NUL-terminate the buffer. */
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
    if ((int64_t)dirfd != LINUX_AT_FDCWD) {
        return LINUX_ENOTSUP;
    }
    return linux_readlink(path_ptr, buf, bufsiz);
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
        if ((int64_t)dirfd != LINUX_AT_FDCWD) {
            return LINUX_ENOTSUP;
        }
        char path[256];
        int64_t rc = linux_copy_cstring(path, sizeof(path),
                                        (const char *)(uintptr_t)path_ptr);
        if (rc < 0) return rc;
        rc = linux_resolve_path(path, sizeof(path));
        if (rc < 0) return rc;

        vfs_file_t vf;
        if (vfs_find_file(path, &vf)) {
            linux_stat_fill_common_ino(&st, vf.size, vf.internal_id);
            if (devfs_path_is_device(path)) {
                st.st_mode = LINUX_S_IFCHR | 0x1B6u;
                st.st_size = 0;
            } else {
                st.st_mode = LINUX_S_IFREG | 0x1A4u;
            }
        } else {
            int32_t dir_handle = vfs_opendir(path);
            if (dir_handle < 0) {
                return LINUX_ENOENT;
            }
            (void)vfs_closedir(dir_handle);
            linux_stat_fill_common(&st, 0);
            st.st_mode = LINUX_S_IFDIR | 0x1EDu;
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

static int64_t linux_sched_getaffinity(uint64_t pid, uint64_t cpusetsize,
                                       uint64_t mask_ptr)
{
    int32_t current = process_get_current_pid();
    if (pid != 0u && (int32_t)pid != current) {
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
    int32_t current = process_get_current_pid();
    if (pid != 0u && (int32_t)pid != current) {
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

static int64_t linux_madvise(uint64_t addr, uint64_t length, uint64_t advice)
{
    (void)addr;
    (void)length;
    /* No swap/reclaim subsystem to act on MADV_DONTNEED/MADV_FREE/etc; a
     * successful no-op is enough for glibc's malloc_trim()/PartitionAlloc
     * decommit hints to keep working correctly (they treat madvise purely
     * as an optimization). MADV_NORMAL/WILLNEED/... are all no-ops too. */
    (void)advice;
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
    int32_t rc = syscall_file_dup2((int32_t)oldfd, (int32_t)newfd);
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
    if ((int64_t)dirfd != LINUX_AT_FDCWD) {
        return LINUX_ENOTSUP;
    }
    char path[256];
    int64_t rc = linux_copy_cstring(path, sizeof(path),
                                    (const char *)(uintptr_t)path_ptr);
    if (rc < 0) return rc;
    rc = linux_resolve_path(path, sizeof(path));
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
static void linux_trace_enter(uint64_t num, uint64_t a1, uint64_t a2,
                              uint64_t a3, uint64_t a4, uint64_t a5,
                              uint64_t a6)
{
    serial_write_string("[lx] #");
    serial_write_uint64(num);
    serial_write_string(" (");
    serial_write_uint64(a1); serial_write_char(',');
    serial_write_uint64(a2); serial_write_char(',');
    serial_write_uint64(a3); serial_write_char(',');
    serial_write_uint64(a4); serial_write_char(',');
    serial_write_uint64(a5); serial_write_char(',');
    serial_write_uint64(a6);
    serial_write_string(")\n");
}

static void linux_trace_exit(uint64_t num, int64_t result)
{
    serial_write_string("[lx] #");
    serial_write_uint64(num);
    serial_write_string(" = ");
    serial_write_uint64((uint64_t)result);
    serial_write_char('\n');
}
#define LINUX_TRACE_ENTER(n, a1, a2, a3, a4, a5, a6) \
    linux_trace_enter((n), (a1), (a2), (a3), (a4), (a5), (a6))
#define LINUX_TRACE_EXIT(n, r) linux_trace_exit((n), (r))
#else
#define LINUX_TRACE_ENTER(n, a1, a2, a3, a4, a5, a6) ((void)0)
#define LINUX_TRACE_EXIT(n, r) ((void)0)
#endif

/* A blocking AF_UNIX read that found the ring empty. EAGAIN is not a value a
 * blocking recv() may return, and a client that gets one where it expected to
 * wait simply gives up: the X11 handshake died right here, with libxcb's first
 * read of the server's setup reply. Park the caller for a slice and run the
 * syscall again (same mechanism as linux_wait4/linux_epoll_wait). */
static void linux_unix_block_retry(int32_t fd, int64_t *result,
                                   int *should_switch, int *restart_out)
{
    if (*result != LINUX_EAGAIN || !unix_socket_fd_in_range(fd)) {
        return;
    }
    if (unix_socket_is_nonblock(fd)) {
        /* Bring-up trace: the caller asked for a non-blocking read, so EAGAIN
         * is the right answer and it is on the caller to wait. Printed because
         * "the client gave up after one EAGAIN" looks identical whether the
         * socket really was non-blocking or we simply failed to block. */
        unix_socket_trace_note("rx-NB", fd);
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
        uint32_t r = syscall_poll_one_fd((int32_t)fd, want);
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
        if (process_sleep_current_ms(slice_ms) == 0 &&
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

    LINUX_TRACE_ENTER(num, arg1, arg2, arg3, arg4, arg5, arg6);

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

        case LINUX_SYS_WRITE:
            result = write((int32_t)arg1, (const void *)(uintptr_t)arg2,
                           arg3);
            break;

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
            if (first != '/' && (int64_t)arg1 != LINUX_AT_FDCWD) {
                /* relative path against a non-CWD dirfd: not supported yet */
                result = LINUX_ENOTSUP;
                break;
            }
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) {
                result = rc;
                break;
            }
            rc = linux_resolve_path(path, sizeof(path));
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
            result = syscall_vm_mprotect(arg1, arg2, arg3);
            break;

        case LINUX_SYS_MUNMAP: {
            int32_t self = process_get_current_pid();
            if (self >= 0 && arg2 != 0u) {
                /* Flush + drop any MAP_SHARED file mapping in this range. */
                linux_mshared_flush_range(self, arg1, arg1 + arg2, 1);
                /* ...and release any demand-paged file mapping it covers, so
                 * the reference on the open file description goes away. */
                filemap_unregister_range(self, arg1, arg2);
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

        case LINUX_SYS_CLONE: {
            int should_switch = 0;
            result = linux_clone(saved_rsp, arg1, arg2, arg3, arg4, arg5,
                                 &should_switch);
            if (should_switch) {
                request_switch = 1;
            }
            break;
        }

        case LINUX_SYS_FORK:
        case LINUX_SYS_VFORK: {
            int32_t child_pid = process_fork();
            if (child_pid > 0) {
                request_switch = 1;
            }
            result = (int64_t)child_pid;
            break;
        }

        case LINUX_SYS_EXECVE:
            result = linux_execve(arg1, arg2, arg3);
            request_switch = 1;
            break;

        case LINUX_SYS_EXIT:
            if (process_is_current_thread()) {
                process_thread_exit_current((int32_t)arg1 & 0xFF);
            } else {
                process_exit_current_with_status((int32_t)arg1 & 0xFF);
            }
            result = 0;
            request_switch = 1;
            break;

        case LINUX_SYS_EXIT_GROUP:
            process_exit_current_with_status((int32_t)arg1 & 0xFF);
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
            result = (int64_t)syscall_file_unlink(path);
            break;
        }

        case LINUX_SYS_READLINK:
            result = linux_readlink(arg1, arg2, arg3);
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
        case LINUX_SYS_GETEGID:
            result = 0;
            break;

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
            result = syscall_futex(arg1, arg2, arg3, arg4, arg5, arg6);
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
            /* dirfd is ignored for an absolute pathname (POSIX); only a
             * relative path needs dirfd == AT_FDCWD here. */
            char oa_first = '\0';
            if (arg2 != 0u) {
                (void)copy_from_user(&oa_first, (const void *)(uintptr_t)arg2, 1u);
            }
            if (oa_first != '/' && (int64_t)arg1 != LINUX_AT_FDCWD) {
                result = LINUX_ENOTSUP;
                break;
            }
            result = linux_open_path(arg2, arg3);
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

        case LINUX_SYS_MEMFD_CREATE:
            result = (int64_t)syscall_file_create_memfd(
                (const char *)(uintptr_t)arg1);
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

        case LINUX_SYS_EVENTFD2:
            result = (int64_t)syscall_eventfd(arg1, arg2);
            break;

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
            linux_unix_block_retry((int32_t)arg1, &result, &should_switch,
                                   &request_restart);
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
            if (!unix_socket_fd_in_range((int32_t)arg1)) {
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
            if (!unix_socket_fd_in_range((int32_t)arg1)) {
                result = LINUX_ENOTSUP;
            } else if (!process_user_buffer_is_valid(
                           (const void *)(uintptr_t)arg2, 56u)) {
                result = LINUX_EFAULT;
            } else {
                result = unix_socket_recvmsg((int32_t)arg1, arg2);
                linux_unix_block_retry((int32_t)arg1, &result, &should_switch,
                                       &request_restart);
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
        case LINUX_SYS_FCHMOD:
        case LINUX_SYS_FCHMODAT:
        case LINUX_SYS_CHMOD:
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
        case LINUX_SYS_SETRESGID:
            /* Single-user system (uid 0); accept no-op transitions. */
            result = 0;
            break;

        case LINUX_SYS_GETRESUID:
        case LINUX_SYS_GETRESGID: {
            uint32_t ids[3] = {0, 0, 0};
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

        case LINUX_SYS_CAPGET: {
            /* Report an all-powerful, fully-permitted capability set
             * (uid 0, no sandbox). struct __user_cap_data is 3 x u32 pairs
             * for v3; write zeros for "effective/permitted/inheritable" is
             * wrong-way round, so advertise all bits set. */
            if (arg2 != 0u) {
                uint32_t data[6];
                for (int i = 0; i < 6; ++i) data[i] = 0xffffffffu;
                result = copy_to_user((void *)(uintptr_t)arg2, data,
                                      sizeof(data)) != 0u ? LINUX_EFAULT : 0;
            } else {
                result = 0;
            }
            break;
        }

        case LINUX_SYS_CAPSET:
            result = 0;
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
            if ((int64_t)arg1 != LINUX_AT_FDCWD) {
                result = LINUX_ENOTSUP;
                break;
            }
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) { result = rc; break; }
            rc = linux_resolve_path(path, sizeof(path));
            if (rc < 0) { result = rc; break; }
            result = (int64_t)syscall_file_mkdir(path);
            break;
        }

        case LINUX_SYS_UNLINKAT: {
            if ((int64_t)arg1 != LINUX_AT_FDCWD) {
                result = LINUX_ENOTSUP;
                break;
            }
            char path[256];
            int64_t rc = linux_copy_cstring(path, sizeof(path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) { result = rc; break; }
            rc = linux_resolve_path(path, sizeof(path));
            if (rc < 0) { result = rc; break; }
            result = (int64_t)syscall_file_unlink(path);
            break;
        }

        case LINUX_SYS_RENAMEAT:
        case LINUX_SYS_RENAMEAT2: {
            if ((int64_t)arg1 != LINUX_AT_FDCWD ||
                (int64_t)arg3 != LINUX_AT_FDCWD) {
                result = LINUX_ENOTSUP;
                break;
            }
            char old_path[256];
            char new_path[256];
            int64_t rc = linux_copy_cstring(old_path, sizeof(old_path),
                                            (const char *)(uintptr_t)arg2);
            if (rc < 0) { result = rc; break; }
            rc = linux_copy_cstring(new_path, sizeof(new_path),
                                    (const char *)(uintptr_t)arg4);
            if (rc < 0) { result = rc; break; }
            rc = linux_resolve_path(old_path, sizeof(old_path));
            if (rc >= 0) rc = linux_resolve_path(new_path, sizeof(new_path));
            if (rc < 0) { result = rc; break; }
            result = (int64_t)syscall_file_rename(old_path, new_path);
            break;
        }

        default:
            result = LINUX_ENOSYS;
            break;
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
    (void)compat_registry_register(&g_linux_compat_layer);
}
