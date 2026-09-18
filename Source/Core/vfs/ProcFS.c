#include "ProcFS.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Core/process/ProcessManager.h"
#include "Core/syscall/Syscall_File.h"
#include "Core/syscall/Syscall_Socket.h"
#include "IPC/UnixSocket.h"
#include "Core/timer/Timer.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "kernel/config.h"
#include "Debug/serial/Serial.h"

#define PROCFS_BUFFER_CAP 4096u

/* Linux-ABI processes see their own descriptor numbers, not the kernel's
 * global ones (Compat/Linux/Linux_FdTable.c). /proc/<pid>/fd has to list and
 * resolve those, so the Linux layer registers how. -2 from either hook means
 * "this process has no table of its own": fall back to the global tables. */
static int32_t (*g_procfs_fd_translate)(int32_t pid, int32_t fd);
static int32_t (*g_procfs_fd_next)(int32_t pid, int32_t after);

void procfs_set_fd_hooks(int32_t (*translate)(int32_t pid, int32_t fd),
                         int32_t (*next)(int32_t pid, int32_t after))
{
    g_procfs_fd_translate = translate;
    g_procfs_fd_next = next;
}

/* The kernel-global descriptor behind `fd` as `pid` sees it, or -1. */
static int32_t procfs_global_fd(int32_t pid, int32_t fd)
{
    if (g_procfs_fd_translate != NULL) {
        int32_t g = g_procfs_fd_translate(pid, fd);
        if (g != -2) {
            return g;
        }
    }
    return fd;
}

typedef struct {
    uint8_t *data;
    uint32_t size;
} procfs_open_t;

/* Returns the pid this /proc/<x>/... path refers to, or -1 if `x` is neither
 * "self"/"thread-self" nor the pid of a live process. *suffix_out points at
 * the path component after the pid/self segment (e.g. "maps", "fd/3").
 *
 * Other-process introspection used to be refused here. It cannot be: a
 * multi-process Chromium browser reads /proc/<child>/{stat,status,statm} for
 * every renderer it owns (ProcessMetrics), writes /proc/<pid>/oom_score_adj
 * for each one, and the zygote host resolves /proc/<pid>/exe. Refusing made
 * all of that look like a child that had already died. */
static int32_t procfs_resolve_pid(const char *path, const char **suffix_out)
{
    const char *prefix = "/proc/";
    size_t prefix_len = strlen(prefix);
    if (strncmp(path, prefix, prefix_len) != 0) {
        return -1;
    }
    const char *rest = path + prefix_len;
    int32_t current = process_get_current_pid();

    if (strncmp(rest, "self/", 5) == 0) {
        *suffix_out = rest + 5;
        return current;
    }
    if (strcmp(rest, "self") == 0) {
        *suffix_out = rest + 4;
        return current;
    }
    /* "thread-self" is what glibc and Chromium use when they mean the calling
     * thread rather than the thread group. Threads are scheduler slots here,
     * so the calling slot is already the right answer. */
    if (strncmp(rest, "thread-self/", 12) == 0) {
        *suffix_out = rest + 12;
        return current;
    }
    if (strcmp(rest, "thread-self") == 0) {
        *suffix_out = rest + 11;
        return current;
    }

    const char *cursor = rest;
    int32_t pid = 0;
    int had_digit = 0;
    while (*cursor >= '0' && *cursor <= '9') {
        pid = pid * 10 + (*cursor - '0');
        ++cursor;
        had_digit = 1;
    }
    if (!had_digit || (*cursor != '/' && *cursor != '\0')) {
        return -1;
    }
    if (pid != current && !process_is_alive(pid)) {
        return -1;
    }
    *suffix_out = (*cursor == '/') ? cursor + 1 : cursor;
    return current;
}

/* The command name of any live process. process_get_current_name() only ever
 * answers for the caller, and /proc/<pid>/{stat,status,comm} are now asked
 * about other processes too (see procfs_resolve_pid). */
static void procfs_pid_name(int32_t pid, char *out, uint32_t capacity)
{
    if (capacity == 0u) {
        return;
    }
    if (pid == process_get_current_pid() &&
        process_get_current_name(out, capacity) >= 0 && out[0] != '\0') {
        return;
    }
    process_perf_info_t info;
    if (process_get_perf_info(pid, &info) == 0 && info.name[0] != '\0') {
        strncpy(out, info.name, capacity - 1u);
        out[capacity - 1u] = '\0';
        return;
    }
    strncpy(out, "implusos", capacity - 1u);
    out[capacity - 1u] = '\0';
}

static uint32_t procfs_build_maps(int32_t pid, char *buf, uint32_t cap)
{
    (void)pid;
    uint64_t heap_cursor = process_get_heap_cursor();
    if (heap_cursor < USER_HEAP_BASE) {
        heap_cursor = USER_HEAP_BASE;
    }
    int n = 0;
    n += snprintf(buf + n, cap - (uint32_t)n,
                  "%016llx-%016llx r-xp 00000000 00:00 0 [code]\n",
                  (unsigned long long)USER_CODE_BASE,
                  (unsigned long long)USER_CODE_LIMIT);
    if (heap_cursor > USER_HEAP_BASE) {
        n += snprintf(buf + n, cap - (uint32_t)n,
                      "%016llx-%016llx rw-p 00000000 00:00 0 [heap]\n",
                      (unsigned long long)USER_HEAP_BASE,
                      (unsigned long long)heap_cursor);
    }
    n += snprintf(buf + n, cap - (uint32_t)n,
                  "%016llx-%016llx rw-p 00000000 00:00 0 [stack]\n",
                  (unsigned long long)USER_STACK_BASE,
                  (unsigned long long)USER_STACK_TOP);
    return (uint32_t)n;
}

/* /proc/<pid>/status with every field Linux 6.1 prints, in its order.
 *
 * Readers parse this by field name and treat a missing one as an error:
 * Chromium's renderers read it right after the zygote forks them and a
 * base::expected<> built from an absent field CHECK-failed ("state_ ==
 * State::kValue") before the renderer had done anything. Values this kernel
 * does not track are reported as the idle/zero Linux would show. */
static uint32_t procfs_build_status(int32_t pid, char *buf, uint32_t cap)
{
    char name[64];
    procfs_pid_name(pid, name, sizeof(name));
    int32_t threads = process_count_threads(pid);
    if (threads < 1) {
        threads = 1;
    }
    uint64_t rss_kb = 0;
    process_perf_info_t info;
    if (process_get_perf_info(pid, &info) == 0) {
        rss_kb = info.memory_usage / 1024u;
    }
    uint64_t text_kb = (USER_CODE_LIMIT - USER_CODE_BASE) / 1024u;
    uint64_t vm_kb = rss_kb + text_kb;
    int32_t ppid = process_get_parent_pid(pid);
    uint32_t uid = 0, gid = 0;
    (void)process_get_credentials(pid, &uid, &gid);
    int32_t seen = process_pid_as_seen_by_current(pid);
    int32_t seen_ppid = ppid > 0 ? process_pid_as_seen_by_current(ppid) : 0;
    /* NStgid/NSpid list the pid in every namespace from the root down: two
     * entries when this process is seen as a namespace init. */
    char nspid[32];
    if (seen != pid) {
        snprintf(nspid, sizeof(nspid), "%d\t%d", (int)pid, (int)seen);
    } else {
        snprintf(nspid, sizeof(nspid), "%d", (int)pid);
    }
    return (uint32_t)snprintf(buf, cap,
        "Name:\t%s\n"
        "Umask:\t0022\n"
        "State:\tR (running)\n"
        "Tgid:\t%d\n"
        "Ngid:\t0\n"
        "Pid:\t%d\n"
        "PPid:\t%d\n"
        "TracerPid:\t0\n"
        "Uid:\t%u\t%u\t%u\t%u\n"
        "Gid:\t%u\t%u\t%u\t%u\n"
        "FDSize:\t1024\n"
        "Groups:\t\n"
        "NStgid:\t%s\n"
        "NSpid:\t%s\n"
        "NSpgid:\t%s\n"
        "NSsid:\t%s\n"
        "VmPeak:\t%8llu kB\n"
        "VmSize:\t%8llu kB\n"
        "VmLck:\t       0 kB\n"
        "VmPin:\t       0 kB\n"
        "VmHWM:\t%8llu kB\n"
        "VmRSS:\t%8llu kB\n"
        "RssAnon:\t%8llu kB\n"
        "RssFile:\t       0 kB\n"
        "RssShmem:\t       0 kB\n"
        "VmData:\t%8llu kB\n"
        "VmStk:\t     132 kB\n"
        "VmExe:\t%8llu kB\n"
        "VmLib:\t       0 kB\n"
        "VmPTE:\t       0 kB\n"
        "VmSwap:\t       0 kB\n"
        "HugetlbPages:\t       0 kB\n"
        "CoreDumping:\t0\n"
        "THP_enabled:\t0\n"
        "Threads:\t%d\n"
        "SigQ:\t0/4096\n"
        "SigPnd:\t0000000000000000\n"
        "ShdPnd:\t0000000000000000\n"
        "SigBlk:\t0000000000000000\n"
        "SigIgn:\t0000000000000000\n"
        "SigCgt:\t0000000000000000\n"
        "CapInh:\t0000000000000000\n"
        "CapPrm:\t000001ffffffffff\n"
        "CapEff:\t000001ffffffffff\n"
        "CapBnd:\t000001ffffffffff\n"
        "CapAmb:\t0000000000000000\n"
        "NoNewPrivs:\t0\n"
        "Seccomp:\t0\n"
        "Seccomp_filters:\t0\n"
        "Speculation_Store_Bypass:\tthread vulnerable\n"
        "SpeculationIndirectBranch:\tconditional enabled\n"
        "Cpus_allowed:\tf\n"
        "Cpus_allowed_list:\t0-3\n"
        "Mems_allowed:\t1\n"
        "Mems_allowed_list:\t0\n"
        "voluntary_ctxt_switches:\t0\n"
        "nonvoluntary_ctxt_switches:\t0\n",
        name, seen, seen, seen_ppid, uid, uid, uid, uid, gid, gid, gid, gid,
        nspid, nspid, nspid, nspid,
        (unsigned long long)vm_kb, (unsigned long long)vm_kb,
        (unsigned long long)rss_kb, (unsigned long long)rss_kb,
        (unsigned long long)rss_kb, (unsigned long long)rss_kb,
        (unsigned long long)text_kb, threads);
}

static uint32_t procfs_build_stat(int32_t pid, char *buf, uint32_t cap)
{
    char name[64];
    procfs_pid_name(pid, name, sizeof(name));
    int32_t ppid = process_get_parent_pid(pid);
    uint64_t ticks = timer_ticks();
    return (uint32_t)snprintf(buf, cap,
        "%d (%s) R %d %d %d 0 -1 4194304 0 0 0 0 0 0 0 0 0 0 1 0 %llu "
        "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n",
        process_pid_as_seen_by_current(pid), name,
        ppid > 0 ? process_pid_as_seen_by_current(ppid) : 0,
        process_pid_as_seen_by_current(pid),
        process_pid_as_seen_by_current(pid), (unsigned long long)ticks);
}

static uint32_t procfs_build_cmdline(int32_t pid, char *buf, uint32_t cap)
{
    char arg[512];
    if (process_copy_launch_argument_of(pid, arg, sizeof(arg)) < 0 || arg[0] == '\0') {
        strncpy(arg, "/Userland/Userland.ELF", sizeof(arg) - 1u);
        arg[sizeof(arg) - 1u] = '\0';
    }
    uint32_t len = (uint32_t)strlen(arg);
    if (len + 1u > cap) {
        len = cap - 1u;
    }
    memcpy(buf, arg, len);
    buf[len] = '\0'; /* argv[0] NUL terminator; no further argv known. */
    return len + 1u;
}

/* The identity-map a single-user system has: every uid/gid inside maps to
 * itself outside. Chromium's namespace sandbox writes these files after
 * unshare(CLONE_NEWUSER) and reads them back to confirm the mapping took;
 * an absent file is reported as "no usable sandbox". */
static uint32_t procfs_build_id_map(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap, "%10u %10u %10u\n", 0u, 0u, 4294967295u);
}

static uint32_t procfs_build_setgroups(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap, "allow\n");
}

static uint32_t procfs_build_comm(int32_t pid, char *buf, uint32_t cap)
{
    char name[64];
    procfs_pid_name(pid, name, sizeof(name));
    return (uint32_t)snprintf(buf, cap, "%s\n", name);
}

/* statm is seven page counts: size resident shared text lib data dt.
 * base::ProcessMetrics reads it for every child process the browser owns. */
static uint32_t procfs_build_statm(int32_t pid, char *buf, uint32_t cap)
{
    uint64_t bytes = 0u;
    process_perf_info_t info;
    if (process_get_perf_info(pid, &info) == 0) {
        bytes = info.memory_usage;
    }
    uint64_t pages = bytes / PAGE_SIZE;
    uint64_t text = (USER_CODE_LIMIT - USER_CODE_BASE) / PAGE_SIZE;
    return (uint32_t)snprintf(buf, cap, "%llu %llu 0 %llu 0 %llu 0\n",
                              (unsigned long long)(pages + text),
                              (unsigned long long)pages,
                              (unsigned long long)text,
                              (unsigned long long)pages);
}

/* One entry per mounted pseudo/real filesystem is more than anything here
 * needs; a program that reads these is normally asking "is /dev/shm a tmpfs
 * and is it writable", and answering yes keeps it off its fallback paths. */
static uint32_t procfs_build_mounts(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap,
        "rootfs / rootfs rw 0 0\n"
        "proc /proc proc rw,nosuid,nodev,noexec 0 0\n"
        "devfs /dev devfs rw,nosuid 0 0\n"
        "tmpfs /dev/shm tmpfs rw,nosuid,nodev 0 0\n"
        "tmpfs /tmp tmpfs rw,nosuid,nodev 0 0\n"
        "tmpfs /run tmpfs rw,nosuid,nodev 0 0\n"
        "tmpfs /var tmpfs rw,nosuid,nodev 0 0\n");
}

static uint32_t procfs_build_meminfo(char *buf, uint32_t cap)
{
    uint64_t total_kb = get_total_memory_pages() * (PAGE_SIZE / 1024u);
    uint64_t free_kb = get_free_memory() / 1024u;
    if (free_kb > total_kb) {
        free_kb = total_kb;
    }
    return (uint32_t)snprintf(buf, cap,
        "MemTotal:       %llu kB\n"
        "MemFree:        %llu kB\n"
        "MemAvailable:   %llu kB\n"
        "SwapTotal:      0 kB\n"
        "SwapFree:       0 kB\n",
        (unsigned long long)total_kb,
        (unsigned long long)free_kb,
        (unsigned long long)free_kb);
}

static uint32_t procfs_build_cpuinfo(char *buf, uint32_t cap)
{
    int n = 0;
    n += snprintf(buf + n, cap - (uint32_t)n,
                  "processor\t: 0\n"
                  "vendor_id\t: GenuineIntel\n"
                  "model name\t: ImplusOS Virtual CPU\n"
                  "cpu MHz\t\t: 2000.000\n"
                  "cache size\t: 8192 KB\n"
                  "flags\t\t: fpu sse sse2\n\n");
    return (uint32_t)n;
}

static uint32_t procfs_build_stat_system(char *buf, uint32_t cap)
{
    uint64_t ticks = timer_ticks();
    return (uint32_t)snprintf(buf, cap,
        "cpu  %llu 0 0 0 0 0 0 0 0 0\n"
        "btime %llu\n"
        "processes %d\n",
        (unsigned long long)ticks, (unsigned long long)ticks,
        (int)process_get_capacity());
}

static uint32_t procfs_build_version(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap,
        "Linux version 6.1.0-implus (build@implusos) "
        "(gcc (ImplusOS x86_64-elf-gcc)) #1 SMP ImplusOS\n");
}

static uint32_t procfs_build_boot_id(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap,
        "00000000-0000-0000-0000-000000000000\n");
}

static uint32_t procfs_build_overcommit_memory(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap, "0\n");
}

/* Small static procfs scalars (under /proc/sys and /proc) that glibc,
 * libstdc++ and Chromium probe at startup. Failures here are tolerated, but
 * reporting plausible fixed values avoids fallback code paths that assume
 * a hostile/locked-down environment (TODO_Chromium_LinuxABI.md section 4:
 * "/proc/sys/kernel/threads-max, pid_max 等の静的報告"). */
static uint32_t procfs_build_uptime(char *buf, uint32_t cap)
{
    uint32_t hz = timer_hz();
    if (hz == 0u) {
        hz = 60u;
    }
    uint64_t ticks = timer_ticks();
    uint64_t whole = ticks / hz;
    uint64_t frac = ((ticks % hz) * 100u) / hz;
    return (uint32_t)snprintf(buf, cap, "%llu.%02llu %llu.%02llu\n",
                              (unsigned long long)whole, (unsigned long long)frac,
                              (unsigned long long)whole, (unsigned long long)frac);
}

static uint32_t procfs_build_loadavg(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap, "0.00 0.00 0.00 1/%d %d\n",
                              (int)process_get_capacity(),
                              (int)process_get_current_pid());
}

static uint32_t procfs_build_filesystems(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap,
        "nodev\ttmpfs\n"
        "nodev\tproc\n"
        "nodev\tdevfs\n"
        "\tiso9660\n"
        "\tvfat\n");
}

static uint32_t procfs_build_self_limits(char *buf, uint32_t cap)
{
    return (uint32_t)snprintf(buf, cap,
        "Limit                     Soft Limit           Hard Limit           Units\n"
        "Max cpu time              unlimited            unlimited            seconds\n"
        "Max file size             unlimited            unlimited            bytes\n"
        "Max data size             unlimited            unlimited            bytes\n"
        "Max stack size            8388608              unlimited            bytes\n"
        "Max core file size        0                    0                    bytes\n"
        "Max resident set          unlimited            unlimited            bytes\n"
        "Max processes             %-20d %-20d processes\n"
        "Max open files            %-20d %-20d files\n"
        "Max locked memory         unlimited            unlimited            bytes\n"
        "Max address space         unlimited            unlimited            bytes\n"
        "Max file locks            unlimited            unlimited            locks\n"
        "Max pending signals       %-20d %-20d signals\n"
        "Max msgqueue size         819200               819200               bytes\n"
        "Max nice priority         0                    0\n"
        "Max realtime priority     0                    0\n"
        "Max realtime timeout      unlimited            unlimited            us\n",
        (int)process_get_capacity(), (int)process_get_capacity(),
        (int)OS_CONFIG_FILE_MAX_FD, (int)OS_CONFIG_FILE_MAX_FD,
        4096, 4096);
}

typedef struct {
    const char *path;
    const char *value;
} procfs_static_scalar_t;

static const procfs_static_scalar_t g_procfs_static_scalars[] = {
    { "/proc/sys/kernel/threads-max",        "16384\n" },
    { "/proc/sys/kernel/pid_max",            "65536\n" },
    { "/proc/sys/kernel/osrelease",          "6.1.0-implus\n" },
    { "/proc/sys/kernel/ostype",             "Linux\n" },
    { "/proc/sys/kernel/hostname",           "implusos\n" },
    { "/proc/sys/kernel/cap_last_cap",       "40\n" },
    { "/proc/sys/kernel/ngroups_max",        "65536\n" },
    { "/proc/sys/kernel/yama/ptrace_scope",  "0\n" },
    { "/proc/sys/vm/max_map_count",          "1048576\n" },
    { "/proc/sys/vm/overcommit_ratio",       "50\n" },
    { "/proc/sys/vm/mmap_min_addr",          "65536\n" },
    { "/proc/sys/net/core/somaxconn",        "128\n" },
    { "/proc/sys/fs/pipe-max-size",          "1048576\n" },
    { "/proc/sys/fs/file-max",               "65536\n" },
    { "/proc/sys/fs/nr_open",                "1048576\n" },
};

/* Which /proc files a foreign program actually reads, and whether it got
 * anything. Enabled with -DPROCFS_TRACE=1. Chromium and glibc consult a dozen
 * of these and quietly change behaviour on what they find, so "is this file
 * even being read" is the first question worth answering before improving one.
 */
#ifndef PROCFS_TRACE
#define PROCFS_TRACE 0
#endif

static bool procfs_generate(const char *path, char *buf, uint32_t cap,
                            uint32_t *size_out)
{
#if PROCFS_TRACE
    serial_write_string("[procfs] ");
    serial_write_string(path);
    serial_write_char('\n');
#endif
    const char *suffix = NULL;
    int32_t pid = procfs_resolve_pid(path, &suffix);
    if (pid >= 0) {
        if (strcmp(suffix, "maps") == 0) {
            *size_out = procfs_build_maps(pid, buf, cap);
            return true;
        }
        if (strcmp(suffix, "status") == 0) {
            *size_out = procfs_build_status(pid, buf, cap);
            return true;
        }
        if (strcmp(suffix, "stat") == 0) {
            *size_out = procfs_build_stat(pid, buf, cap);
            return true;
        }
        if (strcmp(suffix, "cmdline") == 0) {
            *size_out = procfs_build_cmdline(pid, buf, cap);
            return true;
        }
        if (strcmp(suffix, "limits") == 0) {
            *size_out = procfs_build_self_limits(buf, cap);
            return true;
        }
        if (strcmp(suffix, "oom_score") == 0 ||
            strcmp(suffix, "oom_score_adj") == 0 ||
            strcmp(suffix, "oom_adj") == 0) {
            *size_out = (uint32_t)snprintf(buf, cap, "0\n");
            return true;
        }
        if (strcmp(suffix, "comm") == 0) {
            *size_out = procfs_build_comm(pid, buf, cap);
            return true;
        }
        if (strcmp(suffix, "statm") == 0) {
            *size_out = procfs_build_statm(pid, buf, cap);
            return true;
        }
        if (strcmp(suffix, "uid_map") == 0 || strcmp(suffix, "gid_map") == 0) {
            *size_out = procfs_build_id_map(buf, cap);
            return true;
        }
        if (strcmp(suffix, "setgroups") == 0) {
            *size_out = procfs_build_setgroups(buf, cap);
            return true;
        }
        if (strcmp(suffix, "cgroup") == 0) {
            *size_out = (uint32_t)snprintf(buf, cap, "0::/\n");
            return true;
        }
        if (strcmp(suffix, "mounts") == 0 || strcmp(suffix, "mountinfo") == 0) {
            *size_out = procfs_build_mounts(buf, cap);
            return true;
        }
        if (strncmp(suffix, "ns/", 3) == 0) {
            /* On Linux these are magic symlinks whose target names the
             * namespace instance. Chromium only ever stat()s them: the
             * presence of /proc/self/ns/user is its test for "this kernel
             * supports unprivileged user namespaces", and the answer decides
             * whether it can sandbox at all. A readable file with the same
             * "<type>:[<inode>]" text is enough, and the fixed inode is
             * honest -- there is exactly one of each namespace here. */
            const char *type = suffix + 3;
            if (strcmp(type, "user") == 0 || strcmp(type, "pid") == 0 ||
                strcmp(type, "net") == 0 || strcmp(type, "mnt") == 0 ||
                strcmp(type, "ipc") == 0 || strcmp(type, "uts") == 0 ||
                strcmp(type, "cgroup") == 0) {
                *size_out = (uint32_t)snprintf(buf, cap, "%s:[4026531836]\n",
                                               type);
                return true;
            }
            return false;
        }
        if (strncmp(suffix, "fdinfo/", 7) == 0 && suffix[7] >= '0' &&
            suffix[7] <= '9' &&
            strspn(suffix + 7, "0123456789") == strlen(suffix + 7)) {
            /* Only real descriptor numbers: fdinfo/ is Chromium's chroot of
             * choice precisely because nothing else can exist inside it. */
            /* Only the three fields anything actually parses. The offset is
             * reported as 0 rather than tracked: nothing here seeks a
             * descriptor by reading this file. */
            *size_out = (uint32_t)snprintf(buf, cap,
                                           "pos:\t0\nflags:\t02\nmnt_id:\t1\n");
            return true;
        }
        return false; /* "exe" and "fd/N" are symlinks: see procfs_readlink(). */
    }

    if (strcmp(path, "/proc/uptime") == 0) {
        *size_out = procfs_build_uptime(buf, cap);
        return true;
    }
    if (strcmp(path, "/proc/loadavg") == 0) {
        *size_out = procfs_build_loadavg(buf, cap);
        return true;
    }
    if (strcmp(path, "/proc/mounts") == 0 || strcmp(path, "/proc/self/mounts") == 0) {
        *size_out = procfs_build_mounts(buf, cap);
        return true;
    }
    if (strcmp(path, "/proc/filesystems") == 0) {
        *size_out = procfs_build_filesystems(buf, cap);
        return true;
    }
    for (size_t i = 0;
         i < sizeof(g_procfs_static_scalars) / sizeof(g_procfs_static_scalars[0]);
         ++i) {
        if (strcmp(path, g_procfs_static_scalars[i].path) == 0) {
            *size_out = (uint32_t)snprintf(buf, cap, "%s",
                                           g_procfs_static_scalars[i].value);
            return true;
        }
    }

    if (strcmp(path, "/proc/meminfo") == 0) {
        *size_out = procfs_build_meminfo(buf, cap);
        return true;
    }
    if (strcmp(path, "/proc/cpuinfo") == 0) {
        *size_out = procfs_build_cpuinfo(buf, cap);
        return true;
    }
    if (strcmp(path, "/proc/stat") == 0) {
        *size_out = procfs_build_stat_system(buf, cap);
        return true;
    }
    if (strcmp(path, "/proc/version") == 0) {
        *size_out = procfs_build_version(buf, cap);
        return true;
    }
    if (strcmp(path, "/proc/sys/kernel/random/boot_id") == 0) {
        *size_out = procfs_build_boot_id(buf, cap);
        return true;
    }
    if (strcmp(path, "/proc/sys/vm/overcommit_memory") == 0) {
        *size_out = procfs_build_overcommit_memory(buf, cap);
        return true;
    }
    return false;
}

/*
 * Directory enumeration.
 *
 * /proc used to have none ("opendir is not implemented"), and that is a hole
 * a real Linux program falls into immediately: Crashpad lists /proc/self/fd
 * to decide which descriptors to close in the handler process it execs, and
 * gives up on the whole crash reporter when the directory cannot be read.
 * Chromium's sandbox separately walks the same directory to assert no
 * directory descriptor is still open before it locks itself down.
 *
 * Three shapes are enumerable, which is all a Linux program asks for here:
 *   /proc                 -- "self" plus one entry per live process
 *   /proc/<pid>/fd        -- one entry per open descriptor (files, AF_UNIX
 *                            and AF_INET sockets each live in their own fd
 *                            range, so all three tables are walked)
 *   /proc/<pid>/fdinfo    -- the same names as fd/ (contents are per-fd files)
 *   /proc/<pid>/task      -- one entry per thread in the group
 */
#define PROCFS_DIR_HANDLE_MAX 16u

enum {
    PROCFS_DIR_ROOT = 1,
    PROCFS_DIR_FD,
    PROCFS_DIR_TASK,
};

/* Descriptor tables walked in order by a PROCFS_DIR_FD handle. */
enum {
    PROCFS_FD_STAGE_FILE = 0,
    PROCFS_FD_STAGE_UNIX,
    PROCFS_FD_STAGE_INET,
    PROCFS_FD_STAGE_DONE,
};

typedef struct {
    uint8_t used;
    uint8_t kind;
    uint8_t stage;    /* PROCFS_DIR_FD only: which descriptor table */
    uint8_t dots;     /* how many of ".", ".." have been produced */
    int32_t pid;
    int32_t cursor;   /* last entry produced, per kind; -1 == none yet */
} procfs_dir_t;

static procfs_dir_t g_procfs_dirs[PROCFS_DIR_HANDLE_MAX];

/* Strip a trailing '/' and return the directory kind for `path`, or 0 if it
 * is not an enumerable directory. *pid_out is the process the path names
 * (ignored for PROCFS_DIR_ROOT). */
static uint8_t procfs_dir_kind(const char *path, int32_t *pid_out)
{
    if (path == NULL) {
        return 0u;
    }
    char trimmed[128];
    size_t len = strlen(path);
    while (len > 1u && path[len - 1u] == '/') {
        --len;
    }
    if (len >= sizeof(trimmed)) {
        return 0u;
    }
    memcpy(trimmed, path, len);
    trimmed[len] = '\0';

    if (strcmp(trimmed, "/proc") == 0) {
        *pid_out = -1;
        return PROCFS_DIR_ROOT;
    }
    const char *suffix = NULL;
    int32_t pid = procfs_resolve_pid(trimmed, &suffix);
    if (pid < 0 || suffix == NULL) {
        return 0u;
    }
    *pid_out = pid;
    if (strcmp(suffix, "fd") == 0 || strcmp(suffix, "fdinfo") == 0) {
        return PROCFS_DIR_FD;
    }
    if (strcmp(suffix, "task") == 0) {
        return PROCFS_DIR_TASK;
    }
    return 0u;
}

/* The next descriptor after `dir->cursor`, walking the file table, then the
 * AF_UNIX range, then the AF_INET range. Returns -1 once all three are
 * exhausted. */
static int32_t procfs_dir_next_fd(procfs_dir_t *dir)
{
    while (dir->stage != PROCFS_FD_STAGE_DONE) {
        int32_t next = -1;
        switch (dir->stage) {
            case PROCFS_FD_STAGE_FILE:
                next = syscall_file_next_open_fd(dir->pid, dir->cursor);
                break;
            case PROCFS_FD_STAGE_UNIX:
                next = unix_socket_next_open_fd(dir->pid, dir->cursor);
                break;
            default:
                next = syscall_socket_next_open_fd(dir->pid, dir->cursor);
                break;
        }
        if (next >= 0) {
            dir->cursor = next;
            return next;
        }
        ++dir->stage;
        dir->cursor = -1;
    }
    return -1;
}

static bool procfs_vfs_find_file(const char *path, vfs_file_t *out_file)
{
    char *buffer = (char *)malloc(PROCFS_BUFFER_CAP);
    if (buffer == NULL) {
        return false;
    }
    uint32_t size = 0;
    if (!procfs_generate(path, buffer, PROCFS_BUFFER_CAP, &size)) {
        free(buffer);
        return false;
    }
    procfs_open_t *open_entry = (procfs_open_t *)malloc(sizeof(procfs_open_t));
    if (open_entry == NULL) {
        free(buffer);
        return false;
    }
    open_entry->data = (uint8_t *)buffer;
    open_entry->size = size;
    out_file->internal_id = (uint64_t)(uintptr_t)open_entry;
    out_file->size = size;
    out_file->driver_data = open_entry;
    return true;
}

static bool procfs_vfs_read_at(vfs_file_t *file, uint32_t offset,
                               uint8_t *buffer, uint32_t size)
{
    if (file == NULL || file->driver_data == NULL || buffer == NULL) {
        return false;
    }
    procfs_open_t *entry = (procfs_open_t *)file->driver_data;
    if (offset > entry->size || size > entry->size - offset) {
        return false;
    }
    memcpy(buffer, entry->data + offset, size);
    return true;
}

static bool procfs_vfs_read_file(vfs_file_t *file, uint8_t *buffer)
{
    return procfs_vfs_read_at(file, 0, buffer, file != NULL ? file->size : 0u);
}

static bool procfs_vfs_write_file(vfs_file_t *file, const uint8_t *buffer)
{
    (void)file;
    (void)buffer;
    return false;
}

static bool procfs_vfs_write_at(vfs_file_t *file, uint32_t offset,
                                const uint8_t *buffer, uint32_t size)
{
    (void)file;
    (void)offset;
    (void)buffer;
    (void)size;
    return true; /* Writes to e.g. /proc/sys/... are accepted and discarded. */
}

/* Writes to /proc/<pid>/oom_score_adj and friends are accepted and
 * discarded; a write longer than the generated text grows the "file" first,
 * and refusing that made the write fail with EIO ("Failed to adjust OOM score
 * of renderer"). */
static bool procfs_vfs_truncate(vfs_file_t *file, uint32_t new_size)
{
    (void)file;
    (void)new_size;
    return true;
}

static uint32_t procfs_vfs_get_file_size(vfs_file_t *file)
{
    return file != NULL ? file->size : 0u;
}

static bool procfs_vfs_creat(const char *path)
{
    (void)path;
    return false;
}

static bool procfs_vfs_mkdir(const char *path)
{
    (void)path;
    return false;
}

static int32_t procfs_vfs_opendir(const char *path)
{
    int32_t pid = -1;
    uint8_t kind = procfs_dir_kind(path, &pid);
    if (kind == 0u) {
        return -1;
    }
    for (uint32_t i = 0; i < PROCFS_DIR_HANDLE_MAX; ++i) {
        if (g_procfs_dirs[i].used != 0u) {
            continue;
        }
        g_procfs_dirs[i].used = 1u;
        g_procfs_dirs[i].kind = kind;
        g_procfs_dirs[i].stage = PROCFS_FD_STAGE_FILE;
        g_procfs_dirs[i].dots = 0u;
        g_procfs_dirs[i].pid = pid;
        g_procfs_dirs[i].cursor = -1;
        return (int32_t)i;
    }
    return -1;
}

static int32_t procfs_vfs_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    if (handle < 0 || (uint32_t)handle >= PROCFS_DIR_HANDLE_MAX ||
        out_entry == NULL) {
        return -1;
    }
    procfs_dir_t *dir = &g_procfs_dirs[handle];
    if (dir->used == 0u) {
        return -1;
    }
    out_entry->size = 0u;
    if (dir->dots < 2u) {
        const char *name = (dir->dots == 0u) ? "." : "..";
        ++dir->dots;
        strncpy(out_entry->name, name, sizeof(out_entry->name) - 1u);
        out_entry->name[sizeof(out_entry->name) - 1u] = '\0';
        out_entry->is_directory = true;
        return 0;
    }

    switch (dir->kind) {
        case PROCFS_DIR_ROOT: {
            if (dir->cursor == -1) {
                dir->cursor = 0;
                strncpy(out_entry->name, "self", sizeof(out_entry->name) - 1u);
                out_entry->name[sizeof(out_entry->name) - 1u] = '\0';
                out_entry->is_directory = true;
                return 0;
            }
            int32_t pid = process_next_live_pid(dir->pid);
            if (pid < 0) {
                return -1;
            }
            dir->pid = pid;
            snprintf(out_entry->name, sizeof(out_entry->name), "%d", (int)pid);
            out_entry->is_directory = true;
            return 0;
        }
        case PROCFS_DIR_FD: {
            int32_t fd = -1;
            int per_process = 0;
            if (g_procfs_fd_next != NULL) {
                int32_t next = g_procfs_fd_next(dir->pid, dir->cursor);
                if (next != -2) {
                    per_process = 1;
                    fd = next;
                    if (fd >= 0) {
                        dir->cursor = fd;
                    }
                }
            }
            if (!per_process) {
                fd = procfs_dir_next_fd(dir);
            }
            if (fd < 0) {
                return -1;
            }
            snprintf(out_entry->name, sizeof(out_entry->name), "%d", (int)fd);
            /* A descriptor opened on a directory is the one case a caller
             * actually branches on (Chromium refuses to enter its sandbox
             * while one is open), so report it truthfully. */
            out_entry->is_directory =
                syscall_file_is_dir(procfs_global_fd(dir->pid, fd)) > 0;
            return 0;
        }
        default: {
            int32_t tid = process_next_thread_of(dir->pid, dir->cursor);
            if (tid < 0) {
                return -1;
            }
            dir->cursor = tid;
            snprintf(out_entry->name, sizeof(out_entry->name), "%d", (int)tid);
            out_entry->is_directory = true;
            return 0;
        }
    }
}

static int32_t procfs_vfs_closedir(int32_t handle)
{
    if (handle < 0 || (uint32_t)handle >= PROCFS_DIR_HANDLE_MAX) {
        return -1;
    }
    g_procfs_dirs[handle].used = 0u;
    return 0;
}

static bool procfs_vfs_close_file(vfs_file_t *file)
{
    if (file == NULL || file->driver_data == NULL) {
        return true;
    }
    procfs_open_t *entry = (procfs_open_t *)file->driver_data;
    free(entry->data);
    free(entry);
    file->driver_data = NULL;
    return true;
}

static bool procfs_vfs_unlink(const char *path)
{
    (void)path;
    return false;
}

static void procfs_vfs_list_root(void)
{
}

static void procfs_vfs_set_case_sensitive(bool enabled)
{
    (void)enabled;
}

static bool procfs_vfs_get_case_sensitive(void)
{
    return true;
}

static const vfs_driver_t g_procfs_vfs_driver = {
    .fs_type = "procfs",
    .media_kind = VFS_MEDIA_KIND_PSEUDO,
    .prefix = NULL,
    .find_file = procfs_vfs_find_file,
    .read_file = procfs_vfs_read_file,
    .write_file = procfs_vfs_write_file,
    .read_at = procfs_vfs_read_at,
    .write_at = procfs_vfs_write_at,
    .truncate = procfs_vfs_truncate,
    .get_file_size = procfs_vfs_get_file_size,
    .creat = procfs_vfs_creat,
    .mkdir = procfs_vfs_mkdir,
    .opendir = procfs_vfs_opendir,
    .readdir = procfs_vfs_readdir,
    .closedir = procfs_vfs_closedir,
    .close_file = procfs_vfs_close_file,
    .unlink = procfs_vfs_unlink,
    .list_root = procfs_vfs_list_root,
    .set_case_sensitive = procfs_vfs_set_case_sensitive,
    .get_case_sensitive = procfs_vfs_get_case_sensitive,
};

void procfs_init(void)
{
}

const vfs_driver_t *procfs_vfs_get_driver(void)
{
    return &g_procfs_vfs_driver;
}

/* Shared by procfs_readlink() and procfs_parse_fd_path(): decode the "<n>"
 * of an "fd/<n>" suffix. Returns -1 unless the whole suffix is that. */
static int32_t procfs_fd_suffix_to_fd(const char *suffix)
{
    if (suffix == NULL || strncmp(suffix, "fd/", 3) != 0) {
        return -1;
    }
    const char *num = suffix + 3;
    int32_t fd = 0;
    int had_digit = 0;
    while (*num >= '0' && *num <= '9') {
        fd = fd * 10 + (*num - '0');
        ++num;
        had_digit = 1;
    }
    if (!had_digit || *num != '\0') {
        return -1;
    }
    return fd;
}

int32_t procfs_parse_fd_path(const char *path)
{
    if (path == NULL) {
        return -1;
    }
    const char *suffix = NULL;
    int32_t pid = procfs_resolve_pid(path, &suffix);
    if (pid < 0 || suffix == NULL) {
        return -1;
    }
    /* The descriptor tables are keyed by fd number alone, so a number only
     * means anything for the calling process: /proc/<other>/fd/<n> would
     * otherwise hand out this process's descriptor <n>. */
    if (pid != process_get_current_pid()) {
        return -1;
    }
    int32_t fd = procfs_fd_suffix_to_fd(suffix);
    if (fd < 0) {
        return -1;
    }
    /* Callers reopen or stat what this names, so hand back the kernel's
     * number for it. */
    return procfs_global_fd(pid, fd);
}

int procfs_readlink(const char *path, char *out, uint32_t capacity)
{
    const char *suffix = NULL;
    int32_t pid = procfs_resolve_pid(path, &suffix);
    if (pid < 0 || suffix == NULL) {
        return -1;
    }
    if (strcmp(suffix, "cwd") == 0 || strcmp(suffix, "root") == 0) {
        /* No per-process root, and the cwd of another process is not tracked
         * separately, so both resolve to "/" for anyone but the caller. */
        if (strcmp(suffix, "cwd") == 0 && pid == process_get_current_pid() &&
            process_get_current_cwd(out, capacity) == 0 && out[0] != '\0') {
            return 0;
        }
        strncpy(out, "/", capacity - 1u);
        out[capacity - 1u] = '\0';
        return 0;
    }
    if (strcmp(suffix, "exe") == 0) {
        /* The real executable path (glibc / Chromium readlink() this to find
         * their own asset directory: get it wrong and Chromium can't locate
         * icudtl.dat -> "Invalid file descriptor to ICU data received"). Fall
         * back to the launch argument, then to the init ELF, only if the exec
         * path was never recorded. */
        char arg[256];
        if (process_copy_exe_path_of(pid, arg, sizeof(arg)) <= 0 || arg[0] == '\0') {
            if (process_copy_launch_argument_of(pid, arg, sizeof(arg)) < 0 ||
                arg[0] == '\0') {
                strncpy(arg, "/Userland/Userland.ELF", sizeof(arg) - 1u);
                arg[sizeof(arg) - 1u] = '\0';
            }
        }
        strncpy(out, arg, capacity - 1u);
        out[capacity - 1u] = '\0';
        return 0;
    }
    {
        int32_t fd = procfs_fd_suffix_to_fd(suffix);
        if (fd < 0) {
            return -1;
        }
        int32_t global = procfs_global_fd(pid, fd);
        if (global < 0) {
            return -1;
        }
        {
            /* Sockets and pipes read back the way Linux names them; some
             * callers (Chromium's sandbox among them) look for "socket:". */
            extern int unix_socket_fd_in_range(int32_t fd);
            if (unix_socket_fd_in_range(global)) {
                snprintf(out, capacity, "socket:[%d]", (int)global);
                return 0;
            }
        }
        fd = global;
        vfs_file_t vf;
        if (syscall_file_get_file_info(fd, &vf, NULL) == 0) {
            /* The original open() path is not retained by the fd table,
             * so this is a best-effort placeholder rather than the real
             * path (matches what Linux shows for anonymous/unresolvable
             * descriptors). */
            snprintf(out, capacity, "anon_inode:[implusos-fd%d]", (int)fd);
            return 0;
        }
        return -1;
    }
}
