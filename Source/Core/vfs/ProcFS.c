#include "ProcFS.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Core/process/ProcessManager.h"
#include "Core/syscall/Syscall_File.h"
#include "Core/timer/Timer.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "kernel/config.h"

#define PROCFS_BUFFER_CAP 4096u

typedef struct {
    uint8_t *data;
    uint32_t size;
} procfs_open_t;

/* Returns the pid this /proc/<x>/... path refers to, or -1 if `x` is not
 * "self" and not the calling process's own pid (arbitrary-other-process
 * introspection is not supported). *suffix_out points at the path
 * component after the pid/self segment (e.g. "maps", "fd/3"). */
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
    if (pid != current) {
        return -1; /* Other-process introspection unsupported. */
    }
    *suffix_out = (*cursor == '/') ? cursor + 1 : cursor;
    return current;
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

static uint32_t procfs_build_status(int32_t pid, char *buf, uint32_t cap)
{
    char name[64];
    if (process_get_current_name(name, sizeof(name)) < 0) {
        strncpy(name, "implusos", sizeof(name) - 1u);
        name[sizeof(name) - 1u] = '\0';
    }
    int32_t threads = process_count_threads(pid);
    if (threads < 1) {
        threads = 1;
    }
    uint64_t heap_cursor = process_get_heap_cursor();
    uint64_t vm_size_kb =
        (heap_cursor > USER_HEAP_BASE ? (heap_cursor - USER_HEAP_BASE) : 0u) / 1024u +
        (USER_CODE_LIMIT - USER_CODE_BASE) / 1024u;
    int32_t ppid = process_get_parent_pid(pid);
    return (uint32_t)snprintf(buf, cap,
        "Name:\t%s\n"
        "State:\tR (running)\n"
        "Tgid:\t%d\n"
        "Pid:\t%d\n"
        "PPid:\t%d\n"
        "Threads:\t%d\n"
        "VmSize:\t%llu kB\n"
        "VmRSS:\t%llu kB\n"
        "Uid:\t0\t0\t0\t0\n"
        "Gid:\t0\t0\t0\t0\n",
        name, pid, pid, ppid, threads,
        (unsigned long long)vm_size_kb,
        (unsigned long long)vm_size_kb);
}

static uint32_t procfs_build_stat(int32_t pid, char *buf, uint32_t cap)
{
    char name[64];
    if (process_get_current_name(name, sizeof(name)) < 0) {
        strncpy(name, "implusos", sizeof(name) - 1u);
        name[sizeof(name) - 1u] = '\0';
    }
    int32_t ppid = process_get_parent_pid(pid);
    uint64_t ticks = timer_ticks();
    return (uint32_t)snprintf(buf, cap,
        "%d (%s) R %d %d %d 0 -1 4194304 0 0 0 0 0 0 0 0 0 0 1 0 %llu "
        "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n",
        pid, name, ppid, pid, pid, (unsigned long long)ticks);
}

static uint32_t procfs_build_cmdline(int32_t pid, char *buf, uint32_t cap)
{
    (void)pid;
    char arg[512];
    if (process_copy_launch_argument(arg, sizeof(arg)) < 0 || arg[0] == '\0') {
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

static bool procfs_generate(const char *path, char *buf, uint32_t cap,
                            uint32_t *size_out)
{
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

static bool procfs_vfs_truncate(vfs_file_t *file, uint32_t new_size)
{
    (void)file;
    (void)new_size;
    return false;
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
    (void)path;
    return -1; /* Directory enumeration of /proc is not implemented. */
}

static int32_t procfs_vfs_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    (void)handle;
    (void)out_entry;
    return -1;
}

static int32_t procfs_vfs_closedir(int32_t handle)
{
    (void)handle;
    return -1;
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

int procfs_readlink(const char *path, char *out, uint32_t capacity)
{
    const char *suffix = NULL;
    int32_t pid = procfs_resolve_pid(path, &suffix);
    if (pid < 0 || suffix == NULL) {
        return -1;
    }
    if (strcmp(suffix, "exe") == 0) {
        /* The real executable path (glibc / Chromium readlink() this to find
         * their own asset directory: get it wrong and Chromium can't locate
         * icudtl.dat -> "Invalid file descriptor to ICU data received"). Fall
         * back to the launch argument, then to the init ELF, only if the exec
         * path was never recorded. */
        char arg[256];
        if (process_copy_exe_path(arg, sizeof(arg)) <= 0 || arg[0] == '\0') {
            if (process_copy_launch_argument(arg, sizeof(arg)) < 0 || arg[0] == '\0') {
                strncpy(arg, "/Userland/Userland.ELF", sizeof(arg) - 1u);
                arg[sizeof(arg) - 1u] = '\0';
            }
        }
        strncpy(out, arg, capacity - 1u);
        out[capacity - 1u] = '\0';
        return 0;
    }
    if (strncmp(suffix, "fd/", 3) == 0) {
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
    return -1;
}
