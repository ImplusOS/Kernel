#include "Syscall_Main.h"
#include "Syscall_File.h"
#include "Syscall_Socket.h"
#include "Compat/compat_registry.h"
#include "kernel/boot_info.h"
#include "kernel/status.h"
#include "kernel/system_info.h"
#include "Drivers/Module/DriverManager.h"
#include "Drivers/Module/InputManager.h"
#include "Drivers/Module/AudioManager.h"
#include "Core/process/ProcessManager.h"
#include "Core/process/ProcessScheduler.h"
#include "smp/SMP_Main.h"
#include "Core/memory/SharedMemory.h"
#include "Core/sysinfo/SystemInfo.h"
#include "IPC/IPC_Main.h"
#include "Syscall_InputOwner.h"

typedef struct {
    int32_t pid;
    int32_t parent_pid;
    uint8_t state;
    uint8_t reserved[7];
    char    name[64];
    uint64_t total_ticks;
    uint64_t memory_usage;
} process_info_kernel_t;

typedef struct __attribute__((packed)) {
    uint32_t size;
    uint8_t is_dir;
    uint8_t exists;
} syscall_file_stat_t;
#include "mmu/Paging_Main.h"
#include "Drivers/Module/DriverBinary.h"
#include "Debug/serial/Serial.h"
#include "Core/sync/Spinlock.h"
#include "Network/network_main.h"
#include "Network/udp/UDP.h"
#include "Network/tcp/TCP.h"
#include "Network/dhcp/DHCP.h"
#include "Core/timer/Timer.h"
#include "Platform/rtc/RTC.h"
#include "Core/vfs/VFS.h"
#include "Core/usercopy/Usercopy.h"
#include "kernel/config.h"
#include "Platform/io/IO_Main.h"

#if defined(__aarch64__)
#include "cpu/Exception.h"
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SYSCALL_MAX_PATH_LEN    512U
#define SYSCALL_MAX_PUTS_LEN    1024U
#define SYSCALL_MAX_IO_BYTES    (32ULL * 1024ULL * 1024ULL)
#define SYSCALL_MAX_MEM_BYTES   (128ULL * 1024ULL * 1024ULL)
#define SYSCALL_MAX_ALLOC_BYTES (128ULL * 1024ULL * 1024ULL)
#define SYSCALL_UDP_MAX_RECV_BYTES (UDP_USER_HEADER_BYTES + 1472U)
#define SYSCALL_MAX_DISPLAY_RECTS 128U
#define SYSCALL_U32_MASK        0xFFFFFFFFULL
#define WM_FILL_RECT_TRACE      0

static int32_t g_audio_owner_pid = -1;

extern int32_t kernel_boot_profile_count(void);
extern int32_t kernel_boot_profile_get(int32_t index,
                                       boot_profile_entry_t *entry_out);

static const char k_decimal_digits[10] = "0123456789";

static inline void syscall_arch_enable_interrupts(void)
{
#if defined(__aarch64__)
    __asm__ volatile("msr daifclr, #0x2" ::: "memory");
#else
    __asm__ volatile("sti" ::: "memory");
#endif
}

static inline void syscall_arch_disable_interrupts(void)
{
#if defined(__aarch64__)
    __asm__ volatile("msr daifset, #0xf" ::: "memory");
#else
    __asm__ volatile("cli" ::: "memory");
#endif
}

static inline void syscall_arch_user_access_begin(void)
{
#if defined(__x86_64__)
    if ((hal_cpu_read_cr(4) & (1ULL << 21)) != 0) {
        __asm__ volatile("stac" ::: "memory");
    }
#endif
}

static inline void syscall_arch_user_access_end(void)
{
#if defined(__x86_64__)
    if ((hal_cpu_read_cr(4) & (1ULL << 21)) != 0) {
        __asm__ volatile("clac" ::: "memory");
    }
#endif
}

static void set_syscall_result(uint64_t saved_rsp, uint64_t value)
{
#if defined(__aarch64__)
    arm64_exception_frame_t *frame = (arm64_exception_frame_t *)(uintptr_t)saved_rsp;
    if (frame != NULL) {
        frame->x[0] = value;
    }
#else
    uint64_t *frame = (uint64_t *)(uintptr_t)saved_rsp;
    frame[SYSCALL_FRAME_RAX] = value;
#endif
}

static void set_syscall_status(uint64_t saved_rsp, os_status_t status)
{
    set_syscall_result(saved_rsp, os_status_to_u64(status));
}

static void set_syscall_i32(uint64_t saved_rsp, int32_t value)
{
    set_syscall_result(saved_rsp, os_status_to_u64(os_status_from_i32(value)));
}

static int user_buffer_ok(const void *ptr, uint64_t len)
{
    if (len == 0) {
        return 1;
    }
    if (ptr == NULL) {
        return 0;
    }
    return process_user_buffer_is_valid(ptr, len);
}

static int copy_user_cstring(char *dst, uint64_t dst_size, const char *src)
{
    return copy_user_cstring_s(dst, dst_size, src);
}

static int copy_user_to_user(void *dst, const void *src, uint64_t bytes)
{
    if (bytes != 0u &&
        (!process_user_buffer_is_valid(src, bytes) ||
         !process_user_buffer_is_valid(dst, bytes))) {
        return -1;
    }

    uint8_t chunk[512];
    uint64_t copied = 0;

    while (copied < bytes) {
        uint64_t n = bytes - copied;
        if (n > sizeof(chunk)) {
            n = sizeof(chunk);
        }
        if (copy_from_user_trusted(chunk, (const uint8_t *)src + copied, n) != 0u) {
            return -1;
        }
        if (copy_to_user_trusted((uint8_t *)dst + copied, chunk, n) != 0u) {
            return -1;
        }
        copied += n;
    }

    return 0;
}

static int memcmp_user_buffers(const void *lhs, const void *rhs, uint64_t bytes, int *result_out)
{
    uint8_t a[256];
    uint8_t b[256];
    uint64_t compared = 0;

    if (result_out == NULL) {
        return -1;
    }
    *result_out = 0;

    if (bytes != 0u &&
        (!process_user_buffer_is_valid(lhs, bytes) ||
         !process_user_buffer_is_valid(rhs, bytes))) {
        return -1;
    }

    while (compared < bytes) {
        uint64_t n = bytes - compared;
        if (n > sizeof(a)) {
            n = sizeof(a);
        }
        if (copy_from_user_trusted(a, (const uint8_t *)lhs + compared, n) != 0u ||
            copy_from_user_trusted(b, (const uint8_t *)rhs + compared, n) != 0u) {
            return -1;
        }
        for (uint64_t i = 0; i < n; ++i) {
            if (a[i] != b[i]) {
                *result_out = (int)a[i] - (int)b[i];
                return 0;
            }
        }
        compared += n;
    }

    return 0;
}

static int memset_user_buffer(void *dst, uint8_t value, uint64_t bytes)
{
    if (bytes != 0u && !process_user_buffer_is_valid(dst, bytes)) {
        return -1;
    }

    uint8_t chunk[512];
    memset(chunk, value, sizeof(chunk));

    uint64_t written = 0;
    while (written < bytes) {
        uint64_t n = bytes - written;
        if (n > sizeof(chunk)) {
            n = sizeof(chunk);
        }
        if (copy_to_user_trusted((uint8_t *)dst + written, chunk, n) != 0u) {
            return -1;
        }
        written += n;
    }
    return 0;
}

static int64_t syscall_file_read_to_user(int32_t fd, void *user_buffer, uint64_t len)
{
    if (len != 0u && !process_user_buffer_is_valid(user_buffer, len)) {
        return (int64_t)OS_STATUS_FAULT;
    }

    uint8_t chunk[4096];
    uint64_t total = 0;

    while (total < len) {
        uint64_t want = len - total;
        if (want > sizeof(chunk)) {
            want = sizeof(chunk);
        }

        int64_t n = syscall_file_read(fd, chunk, want);
        if (n < 0) {
            return (total != 0u) ? (int64_t)total : n;
        }
        if (n == 0) {
            break;
        }
        if (copy_to_user_trusted((uint8_t *)user_buffer + total, chunk, (uint64_t)n) != 0u) {
            return (total != 0u) ? (int64_t)total : (int64_t)OS_STATUS_FAULT;
        }
        total += (uint64_t)n;
        if ((uint64_t)n < want) {
            break;
        }
    }

    return (int64_t)total;
}

static int64_t syscall_file_write_from_user(int32_t fd, const void *user_buffer, uint64_t len)
{
    if (len != 0u && !process_user_buffer_is_valid(user_buffer, len)) {
        return (int64_t)OS_STATUS_FAULT;
    }

    uint8_t chunk[4096];
    uint64_t total = 0;

    while (total < len) {
        uint64_t want = len - total;
        if (want > sizeof(chunk)) {
            want = sizeof(chunk);
        }
        if (copy_from_user_trusted(chunk, (const uint8_t *)user_buffer + total, want) != 0u) {
            return (total != 0u) ? (int64_t)total : (int64_t)OS_STATUS_FAULT;
        }

        int64_t n = syscall_file_write(fd, chunk, want);
        if (n < 0) {
            return (total != 0u) ? (int64_t)total : n;
        }
        total += (uint64_t)n;
        if ((uint64_t)n < want) {
            break;
        }
    }

    return (int64_t)total;
}

static void syscall_fail(uint64_t saved_rsp,
                         uint64_t syscall_number,
                         os_status_t status,
                         const char *reason)
{
    set_syscall_status(saved_rsp, status);
}

static process_capability_mask_t syscall_required_capability(uint64_t syscall_number)
{
    switch (syscall_number) {
        case SYSCALL_SERIAL_PUTCHAR:
        case SYSCALL_SERIAL_PUTS:
        case SYSCALL_SERIAL_WRITE_U64:
        case SYSCALL_SERIAL_WRITE_U32:
        case SYSCALL_SERIAL_WRITE_U16:
            return PROCESS_CAP_SERIAL;

        case SYSCALL_PROCESS_CREATE:
        case SYSCALL_PROCESS_SPAWN_ELF:
        case SYSCALL_PROCESS_SPAWN_ELF_ARG:
        case SYSCALL_PROCESS_GET_LAUNCH_ARG:
        case SYSCALL_THREAD_CREATE:
        case SYSCALL_FORK:
        case SYSCALL_EXECVE:
        case SYSCALL_VFORK:
        case SYSCALL_CLONE:
            return PROCESS_CAP_PROCESS;
            
        case SYSCALL_FILE_OPEN:
        case SYSCALL_FILE_CREAT:
        case SYSCALL_FILE_READ:
        case SYSCALL_FILE_WRITE:
        case SYSCALL_FILE_CLOSE:
        case SYSCALL_FILE_SEEK:
        case SYSCALL_FILE_MKDIR:
        case SYSCALL_FILE_OPENDIR:
        case SYSCALL_FILE_READDIR:
        case SYSCALL_FILE_CLOSEDIR:
        case SYSCALL_FILE_UNLINK:
            return PROCESS_CAP_FILE;

        case SYSCALL_FILE_STAT:
        case SYSCALL_FILE_PIPE:
        case SYSCALL_FILE_DUP:
        case SYSCALL_FILE_DUP2:
        case SYSCALL_RAW_BLOCK_READ:
        case SYSCALL_RAW_BLOCK_WRITE:
            return PROCESS_CAP_FILE;

        case SYSCALL_USER_MMAP:
        case SYSCALL_USER_MALLOC:
        case SYSCALL_USER_FREE:
        case SYSCALL_USER_MEMCPY:
        case SYSCALL_USER_MEMCMP:
        case SYSCALL_USER_MEMSET:
        case SYSCALL_MPROTECT:
        case SYSCALL_MUNMAP:
        case SYSCALL_MREMAP:
            return PROCESS_CAP_MEMORY;

        case SYSCALL_INPUT_READ_KEYBOARD:
        case SYSCALL_INPUT_READ_MOUSE:
        case SYSCALL_WINDOW_REGISTER_SERVICE:
            return PROCESS_CAP_INPUT;

        case SYSCALL_GET_DISPLAY_WIDTH:
        case SYSCALL_GET_DISPLAY_HEIGHT:
        case SYSCALL_DISPLAY_DRAW_PIXEL:
        case SYSCALL_DISPLAY_FILL_RECT:
        case SYSCALL_DISPLAY_PRESENT:
        case SYSCALL_GET_DISPLAY_FRAMEBUFFER:
        case SYSCALL_DISPLAY_GET_PIXEL:
        case SYSCALL_DISPLAY_GET_TOPOLOGY:
        case SYSCALL_DISPLAY_GET_MONITOR_INFO:
        case SYSCALL_DISPLAY_GET_MONITOR_MODE_INFO:
        case SYSCALL_DISPLAY_SET_MONITOR_MODE:
        case SYSCALL_DISPLAY_PRESENT_RECTS:
            return PROCESS_CAP_DISPLAY;

        case SYSCALL_PROCESS_SIGNAL:
        case SYSCALL_TKILL:
        case SYSCALL_RT_SIGACTION:
        case SYSCALL_RT_SIGPROCMASK:
            return PROCESS_CAP_SIGNAL;

        case SYSCALL_UDP_SEND:
        case SYSCALL_TCP_CONNECT:
        case SYSCALL_TCP_LISTEN:
        case SYSCALL_TCP_ACCEPT:
        case SYSCALL_TCP_SEND:
        case SYSCALL_TCP_RECV:
        case SYSCALL_TCP_CLOSE:
        case SYSCALL_TCP_GET_STATE:
        case SYSCALL_SOCKET_CREATE:
        case SYSCALL_SOCKET_CONNECT:
        case SYSCALL_SOCKET_BIND:
        case SYSCALL_SOCKET_LISTEN:
        case SYSCALL_SOCKET_ACCEPT:
        case SYSCALL_SOCKET_SEND:
        case SYSCALL_SOCKET_RECV:
        case SYSCALL_SOCKET_CLOSE:
        case SYSCALL_SOCKET_GET_INFO:
        case SYSCALL_SOCKET_SET_OPTION:
        case SYSCALL_SOCKET_GET_OPTION:
        case SYSCALL_SOCKET_SHUTDOWN:
        case SYSCALL_SOCKET_LISTEN_EX:
        case SYSCALL_WIFI_SCAN_START:
        case SYSCALL_WIFI_GET_SCAN_RESULTS:
        case SYSCALL_WIFI_CONNECT:
        case SYSCALL_WIFI_DISCONNECT:
        case SYSCALL_WIFI_GET_STATUS:
        case SYSCALL_NET_GET_DHCP_DNS:
            return PROCESS_CAP_NETWORK;

        case SYSCALL_IPC_SEND_MESSAGE:
        case SYSCALL_IPC_RECEIVE_MESSAGE:
            return PROCESS_CAP_IPC;

        case SYSCALL_PROCESS_YIELD:
        case SYSCALL_PROCESS_EXIT:
        case SYSCALL_PROCESS_GET_PID:
        case SYSCALL_PROCESS_WAITPID:
        case SYSCALL_PROCESS_GETPPID:
        case SYSCALL_PROCESS_EXIT_STATUS:
        case SYSCALL_SLEEP:
        case SYSCALL_NANOSLEEP:
        case SYSCALL_GET_UPTIME_MS:
        case SYSCALL_GETCWD:
        case SYSCALL_GET_PROC_COUNT:
        case SYSCALL_GET_PROC_INFO:
        case SYSCALL_GET_PROC_PERF_INFO:
        case SYSCALL_GET_BOOT_PROFILE_COUNT:
        case SYSCALL_GET_BOOT_PROFILE_ENTRY:
        case SYSCALL_GET_DISK_COUNT:
        default:
            return 0;
    }
}

uint64_t syscall_dispatch(uint64_t saved_rsp,
                         uint64_t num,
                         uint64_t arg1,
                         uint64_t arg2,
                         uint64_t arg3,
                         uint64_t arg4,
                         uint64_t arg5,
                         uint64_t arg6)
{
    syscall_arch_enable_interrupts();

    (void)arg5;
    int32_t current_pid = current_pid_get();
    process_perf_note_syscall(current_pid);
    
    int request_switch = 0;
#if WM_FILL_RECT_TRACE
    int trace_wm_fill_rect = 0;
    if (num == SYSCALL_DISPLAY_FILL_RECT) {
        int32_t wm_pid = syscall_input_owner_get();
        trace_wm_fill_rect = (wm_pid >= 0 && current_pid == wm_pid);
    }
#endif

    if (input_manager_check_poll()) {
        uint64_t poll_flags = irq_save_disable();
        input_manager_poll();
        irq_restore(poll_flags);
    }

    if (network_stack_check_poll()) {
        network_stack_poll();
    }

    if (driver_manager_check_hotplug_poll()) {
        driver_manager_hotplug_poll();
    }

    if (num == SYSCALL_INPUT_READ_KEYBOARD ||
        num == SYSCALL_INPUT_READ_MOUSE) {
        uint64_t poll_flags = irq_save_disable();
        input_manager_poll();
        irq_restore(poll_flags);
    }

    /* Kernel-side syscall-ABI compat layer dispatch (Docs/Others/
     * TODO_OS_Refactor.md phase P6) -- looked up by registry instead of a
     * hardcoded "if (abi == PROCESS_ABI_LINUX) linux_syscall_dispatch(...)"
     * so a future second compat layer doesn't require editing this file.
     * See Kernel/Compat/compat_registry.h and Syscall_Init.c's
     * syscall_init() (where layers actually register themselves). */
    const compat_layer_t *compat = compat_registry_find(process_get_current_abi_mode());
    if (compat != NULL) {
        syscall_arch_user_access_begin();
        request_switch |= (int)compat->dispatch(saved_rsp, num,
                                                arg1, arg2, arg3, arg4,
                                                arg5, arg6);
        goto pre_schedule;
    }

    process_capability_mask_t required_capability = syscall_required_capability(num);
    if (required_capability != 0 &&
        !process_current_has_capability(required_capability)) {
        syscall_fail(saved_rsp, num, OS_STATUS_ACCESS_DENIED, "capability_denied");
        goto pre_schedule;
    }

    syscall_arch_user_access_begin();
    switch (num) {
        case SYSCALL_SERIAL_PUTCHAR:
            serial_write_char((char)arg1);
            set_syscall_result(saved_rsp, 0);
            break;

        case SYSCALL_SERIAL_PUTS: {
            char buffer[SYSCALL_MAX_PUTS_LEN];
            if (copy_user_cstring(buffer, sizeof(buffer),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_user_string");
                break;
            }
            serial_write_string(buffer);
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_AUDIO_OPEN: {
            int32_t pid = process_get_current_pid();
            if (pid < 0 || g_audio_owner_pid >= 0 ||
                !audio_manager_open()) {
                set_syscall_status(saved_rsp, OS_STATUS_ACCESS_DENIED);
                break;
            }
            g_audio_owner_pid = pid;
            set_syscall_result(saved_rsp, 1u);
            break;
        }

        case SYSCALL_AUDIO_GET_INFO: {
            if (process_get_current_pid() != g_audio_owner_pid) {
                set_syscall_status(saved_rsp, OS_STATUS_ACCESS_DENIED);
                break;
            }
            driver_audio_info_t info;
            if (!audio_manager_get_info(&info) ||
                copy_to_user((void *)(uintptr_t)arg1, &info,
                             sizeof(info)) != 0u) {
                set_syscall_status(saved_rsp, OS_STATUS_FAULT);
                break;
            }
            set_syscall_status(saved_rsp, OS_STATUS_OK);
            break;
        }

        case SYSCALL_AUDIO_WRITE: {
            if (process_get_current_pid() != g_audio_owner_pid ||
                arg2 == 0u || (arg2 & 3u) != 0u ||
                arg2 > SYSCALL_MAX_IO_BYTES ||
                !user_buffer_ok((const void *)(uintptr_t)arg1, arg2)) {
                set_syscall_status(saved_rsp, OS_STATUS_INVALID_ARG);
                break;
            }
            void *pcm = malloc((size_t)arg2);
            if (pcm == NULL) {
                set_syscall_status(saved_rsp, OS_STATUS_LIMIT_REACHED);
                break;
            }
            if (copy_from_user_trusted(pcm, (const void *)(uintptr_t)arg1,
                                       arg2) != 0u) {
                free(pcm);
                set_syscall_status(saved_rsp, OS_STATUS_FAULT);
                break;
            }
            int64_t written = audio_manager_write(pcm, arg2);
            free(pcm);
            set_syscall_result(saved_rsp, (uint64_t)written);
            break;
        }

        case SYSCALL_AUDIO_DRAIN:
            if (process_get_current_pid() != g_audio_owner_pid) {
                set_syscall_status(saved_rsp, OS_STATUS_ACCESS_DENIED);
            } else {
                set_syscall_status(saved_rsp,
                    audio_manager_drain((uint32_t)arg1) ?
                    OS_STATUS_OK : OS_STATUS_IO_ERROR);
            }
            break;

        case SYSCALL_AUDIO_CLOSE:
            if (process_get_current_pid() != g_audio_owner_pid) {
                set_syscall_status(saved_rsp, OS_STATUS_ACCESS_DENIED);
            } else {
                audio_manager_close();
                g_audio_owner_pid = -1;
                set_syscall_status(saved_rsp, OS_STATUS_OK);
            }
            break;

        case SYSCALL_SERIAL_WRITE_U64:
            serial_write_uint64(arg1);
            set_syscall_result(saved_rsp, 0);
            break;

        case SYSCALL_SERIAL_WRITE_U32:
            serial_write_uint32(arg1);
            set_syscall_result(saved_rsp, 0);
            break;

        case SYSCALL_SERIAL_WRITE_U16:
            serial_write_uint16(arg1);
            set_syscall_result(saved_rsp, 0);
            break;

        case SYSCALL_PROCESS_CREATE: {
            int32_t pid = process_create_user(arg1);
            set_syscall_i32(saved_rsp, pid);
            break;
        }

        case SYSCALL_PROCESS_SPAWN_ELF: {
            char path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_path");
                break;
            }
            int32_t pid = process_spawn_user_elf(path);
            set_syscall_i32(saved_rsp, pid);
            break;
        }

        case SYSCALL_PROCESS_SPAWN_ELF_ARG: {
            char path[SYSCALL_MAX_PATH_LEN];
            char argument[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0 ||
                copy_user_cstring(argument, sizeof(argument),
                                  (const char *)(uintptr_t)arg2) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_spawn_argument");
                break;
            }
            set_syscall_i32(saved_rsp,
                process_spawn_user_elf_with_arg(path, argument));
            break;
        }

        case SYSCALL_PROCESS_GET_LAUNCH_ARG: {
            char argument[SYSCALL_MAX_PATH_LEN];
            if (arg2 == 0u || arg2 > sizeof(argument) ||
                !user_buffer_ok((void *)(uintptr_t)arg1, arg2)) {
                syscall_fail(saved_rsp, num, OS_STATUS_INVALID_ARG,
                             "invalid_launch_argument_buffer");
                break;
            }
            int32_t length =
                process_copy_launch_argument(argument, (uint32_t)arg2);
            if (length < 0 ||
                copy_to_user_trusted((void *)(uintptr_t)arg1, argument,
                                     (uint64_t)length + 1u) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "launch_argument_copy_failed");
                break;
            }
            set_syscall_i32(saved_rsp, length);
            break;
        }

        case SYSCALL_PROCESS_YIELD:
            set_syscall_result(saved_rsp, 0);
            request_switch = 1;
            break;

        case SYSCALL_PROCESS_EXIT:
            process_exit_current();
            request_switch = 1;
            set_syscall_result(saved_rsp, 0);
            break;

        case SYSCALL_THREAD_CREATE: {
            int32_t tid = process_create_thread(arg1, arg2, arg3, arg4, arg5);
            set_syscall_i32(saved_rsp, tid);
            break;
        }

        case SYSCALL_THREAD_EXIT:
            process_thread_exit_current((int32_t)arg1);
            set_syscall_result(saved_rsp, 0);
            request_switch = 1;
            break;

        case SYSCALL_THREAD_JOIN:
            set_syscall_result(saved_rsp,
                               (uint64_t)(int64_t)process_thread_join(
                                   (int32_t)arg1));
            break;

        case SYSCALL_THREAD_DETACH:
            set_syscall_result(saved_rsp,
                               (uint64_t)(int64_t)process_thread_detach(
                                   (int32_t)arg1));
            break;

        case SYSCALL_FILE_OPEN: {
            char path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_path");
                break;
            }

            int32_t fd = syscall_file_open(path, arg2);
            set_syscall_i32(saved_rsp, fd);
            break;
        }

        case SYSCALL_FILE_CREAT: {
            char path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_path");
                break;
            }

            int32_t fd = syscall_file_creat(path);
            set_syscall_i32(saved_rsp, fd);
            break;
        }

        case SYSCALL_FILE_READ: {
            if (arg3 > SYSCALL_MAX_IO_BYTES ||
                !user_buffer_ok((const void *)(uintptr_t)arg2, arg3)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_read_buffer");
                break;
            }

            int64_t n = syscall_file_read_to_user((int32_t)arg1,
                                                  (void *)(uintptr_t)arg2,
                                                  arg3);
            set_syscall_result(saved_rsp, (uint64_t)n);
            break;
        }

        case SYSCALL_FILE_WRITE: {
            if (arg3 > SYSCALL_MAX_IO_BYTES ||
                !user_buffer_ok((const void *)(uintptr_t)arg2, arg3)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_write_buffer");
                break;
            }

            int64_t n = syscall_file_write_from_user((int32_t)arg1,
                                                     (const void *)(uintptr_t)arg2,
                                                     arg3);
            set_syscall_result(saved_rsp, (uint64_t)n);
            break;
        }

        case SYSCALL_FILE_CLOSE: {
            int32_t rc = syscall_file_close((int32_t)arg1);
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_FILE_SEEK: {
            int64_t pos = syscall_file_seek((int32_t)arg1, (int64_t)arg2, (int32_t)arg3);
            set_syscall_result(saved_rsp, (uint64_t)pos);
            break;
        }

        case SYSCALL_FILE_MKDIR: {
            char path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_path");
                break;
            }

            int32_t rc = syscall_file_mkdir(path);
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_FILE_OPENDIR: {
            char path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_path");
                break;
            }

            int32_t handle = syscall_file_opendir(path);
            set_syscall_i32(saved_rsp, handle);
            break;
        }

        case SYSCALL_FILE_READDIR: {
            vfs_dirent_t *entry_out = (vfs_dirent_t *)(uintptr_t)arg2;
            if (!user_buffer_ok(entry_out, sizeof(vfs_dirent_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_dirent_buffer");
                break;
            }

            vfs_dirent_t entry = {0};
            int32_t rc = syscall_file_readdir((int32_t)arg1, &entry);
            if (rc > 0 && copy_to_user_trusted(entry_out, &entry, sizeof(entry)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_dirent_buffer");
                break;
            }
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_FILE_CLOSEDIR: {
            int32_t rc = syscall_file_closedir((int32_t)arg1);
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_FILE_UNLINK: {
            char path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_path");
                break;
            }

            int32_t rc = syscall_file_unlink(path);
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_USER_MMAP: {
            void *mapped = process_user_mmap(arg1, arg2);
            set_syscall_result(saved_rsp, (uint64_t)(uintptr_t)mapped);
            break;
        }

        case SYSCALL_PROCESS_SIGNAL: {
            uint64_t previous = process_signal_set_handler((int32_t)arg1, arg2);
            set_syscall_result(saved_rsp, previous);
            break;
        }

        case SYSCALL_UDP_SEND: {
            uint32_t dst_ip = (uint32_t)arg1;
            uint16_t src_port = (uint16_t)(arg2 >> 16);
            uint16_t dst_port = (uint16_t)(arg2 & 0xFFFF);
            const void *payload = (const void *)(uintptr_t)arg3;
            uint16_t payload_len = (uint16_t)arg4;
            if ((uint64_t)payload_len != arg4 ||
                (payload_len != 0u && !user_buffer_ok(payload, payload_len))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_udp_send_buffer");
                break;
            }

            uint8_t *payload_copy = NULL;
            if (payload_len != 0u) {
                payload_copy = (uint8_t *)malloc(payload_len);
                if (payload_copy == NULL ||
                    copy_from_user_trusted(payload_copy, payload, payload_len) != 0u) {
                    if (payload_copy != NULL) {
                        free(payload_copy);
                    }
                    syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_udp_send_buffer");
                    break;
                }
            }

            bool success = udp_send(dst_ip, src_port, dst_port, payload_copy, payload_len);
            if (payload_copy != NULL) {
                free(payload_copy);
            }
            set_syscall_result(saved_rsp, success ? 1ULL : 0ULL);
            break;
        }

        case SYSCALL_UDP_BIND: {
            uint16_t port = (uint16_t)arg1;
            int32_t pid = process_get_current_pid();
            int32_t result = udp_user_bind(pid, port);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_UDP_UNBIND: {
            uint16_t port = (uint16_t)arg1;
            int32_t pid = process_get_current_pid();
            int32_t result = udp_user_unbind(pid, port);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_UDP_RECV: {
            uint16_t port = (uint16_t)arg1;
            void *buf = (void *)(uintptr_t)arg2;
            uint32_t buf_len = (uint32_t)arg3;
            uint32_t validate_len = buf_len;
            if (validate_len > SYSCALL_UDP_MAX_RECV_BYTES) {
                validate_len = SYSCALL_UDP_MAX_RECV_BYTES;
            }
            if (!user_buffer_ok(buf, validate_len)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_udp_recv_buffer");
                break;
            }
            if (buf_len < UDP_USER_HEADER_BYTES) {
                set_syscall_result(saved_rsp, (uint64_t)(int64_t)-1);
                break;
            }
            int32_t pid = process_get_current_pid();
            uint8_t recv_buf[SYSCALL_UDP_MAX_RECV_BYTES];
            int32_t result = udp_user_recv(pid, port, recv_buf,
                                           (uint32_t)sizeof(recv_buf));
            if (result > 0) {
                uint32_t copy_len = (uint32_t)result;
                if (copy_len > buf_len) {
                    copy_len = buf_len;
                }
                if (copy_len != 0u &&
                    copy_to_user_trusted(buf, recv_buf, copy_len) != 0u) {
                    syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_udp_recv_buffer");
                    break;
                }
                result = (int32_t)copy_len;
            }
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_TCP_CONNECT: {
            uint32_t remote_ip = (uint32_t)arg1;
            uint16_t remote_port = (uint16_t)(arg2 >> 16);
            uint16_t local_port = (uint16_t)(arg2 & 0xFFFF);
            int32_t result = tcp_connect(remote_ip, remote_port, local_port);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_TCP_LISTEN: {
            uint16_t port = (uint16_t)arg1;
            int32_t result = tcp_listen(port);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_TCP_ACCEPT: {
            int32_t listen_id = (int32_t)arg1;
            int32_t result = tcp_accept(listen_id);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_TCP_SEND: {
            int32_t conn_id = (int32_t)arg1;
            const void *payload = (const void *)(uintptr_t)arg2;
            uint16_t payload_len = (uint16_t)arg3;
            if ((uint64_t)payload_len != arg3 ||
                (payload_len != 0u && !user_buffer_ok(payload, payload_len))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_tcp_send_buffer");
                break;
            }

            uint8_t *payload_copy = NULL;
            if (payload_len != 0u) {
                payload_copy = (uint8_t *)malloc(payload_len);
                if (payload_copy == NULL ||
                    copy_from_user_trusted(payload_copy, payload, payload_len) != 0u) {
                    if (payload_copy != NULL) {
                        free(payload_copy);
                    }
                    syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_tcp_send_buffer");
                    break;
                }
            }

            int32_t result = tcp_send(conn_id, payload_copy, payload_len);
            if (payload_copy != NULL) {
                free(payload_copy);
            }
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_TCP_RECV: {
            int32_t conn_id = (int32_t)arg1;
            void *buf = (void *)(uintptr_t)arg2;
            uint16_t buf_len = (uint16_t)arg3;
            if ((uint64_t)buf_len != arg3 ||
                (buf_len != 0u && !user_buffer_ok(buf, buf_len))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_tcp_recv_buffer");
                break;
            }

            uint8_t *recv_buf = NULL;
            if (buf_len != 0u) {
                recv_buf = (uint8_t *)malloc(buf_len);
                if (recv_buf == NULL) {
                    syscall_fail(saved_rsp, num, OS_STATUS_LIMIT_REACHED, "tcp_recv_alloc_failed");
                    break;
                }
            }

            int32_t result = tcp_recv(conn_id, recv_buf, buf_len);
            if (result > 0 &&
                copy_to_user_trusted(buf, recv_buf, (uint64_t)result) != 0u) {
                if (recv_buf != NULL) {
                    free(recv_buf);
                }
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_tcp_recv_buffer");
                break;
            }
            if (recv_buf != NULL) {
                free(recv_buf);
            }
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_TCP_CLOSE: {
            int32_t conn_id = (int32_t)arg1;
            int32_t result = tcp_close(conn_id);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_TCP_GET_STATE: {
            int32_t conn_id = (int32_t)arg1;
            tcp_state_t state = tcp_get_state(conn_id);
            set_syscall_result(saved_rsp, (uint64_t)state);
            break;
        }

        case SYSCALL_USER_MALLOC: {
            uint32_t size = (uint32_t)arg1;
            if (size == 0 || size > SYSCALL_MAX_ALLOC_BYTES) {
                set_syscall_result(saved_rsp, 0);
                break;
            }

            void *ptr = process_user_alloc(size);
            set_syscall_result(saved_rsp, (uint64_t)(uintptr_t)ptr);
            break;
        }

        case SYSCALL_USER_FREE: {
            int rc = process_user_free((void *)(uintptr_t)arg1);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)rc);
            break;
        }

        case SYSCALL_USER_MEMCPY: {
            void *dst = (void *)(uintptr_t)arg1;
            const void *src = (const void *)(uintptr_t)arg2;
            uint64_t n = arg3;

            if (n > SYSCALL_MAX_MEM_BYTES ||
                !user_buffer_ok(dst, n) ||
                !user_buffer_ok(src, n)) {
                set_syscall_result(saved_rsp, 0);
                break;
            }

            if (copy_user_to_user(dst, src, n) < 0) {
                set_syscall_result(saved_rsp, 0);
                break;
            }

            set_syscall_result(saved_rsp, (uint64_t)(uintptr_t)dst);
            break;
        }

        case SYSCALL_USER_MEMCMP: {
            const uint8_t *s1 = (const uint8_t *)(uintptr_t)arg1;
            const uint8_t *s2 = (const uint8_t *)(uintptr_t)arg2;
            uint64_t n = arg3;

            if (n > SYSCALL_MAX_MEM_BYTES ||
                !user_buffer_ok(s1, n) ||
                !user_buffer_ok(s2, n)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_memcmp_buffer");
                break;
            }

            int result = 0;
            if (memcmp_user_buffers(s1, s2, n, &result) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_memcmp_buffer");
                break;
            }

            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_USER_MEMSET: {
            uint8_t *dst = (uint8_t *)(uintptr_t)arg1;
            uint8_t value = (uint8_t)arg2;
            uint64_t n = arg3;

            if (n > SYSCALL_MAX_MEM_BYTES || !user_buffer_ok(dst, n)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_memset_buffer");
                break;
            }
            if (memset_user_buffer(dst, value, n) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_memset_buffer");
                break;
            }

            set_syscall_result(saved_rsp, arg1);
            break;
        }
        case SYSCALL_INPUT_READ_KEYBOARD: {
            driver_keyboard_event_t *event_out = (driver_keyboard_event_t *)(uintptr_t)arg1;
            if (!user_buffer_ok(event_out, sizeof(driver_keyboard_event_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_keyboard_buffer");
                break;
            }
            int32_t rc = 0;
            int32_t owner_pid = syscall_input_owner_get();
            if (owner_pid < 0 || owner_pid == current_pid) {
                driver_keyboard_event_t event = {0};
                rc = input_manager_read_keyboard(&event);
                if (rc > 0 && copy_to_user_trusted(event_out, &event, sizeof(event)) != 0u) {
                    syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_keyboard_buffer");
                    break;
                }
            }
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_INPUT_READ_MOUSE: {
            driver_mouse_event_t *event_out = (driver_mouse_event_t *)(uintptr_t)arg1;
            if (!user_buffer_ok(event_out, sizeof(driver_mouse_event_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_mouse_buffer");
                break;
            }
            int32_t rc = 0;
            int32_t owner_pid = syscall_input_owner_get();
            if (owner_pid < 0 || owner_pid == current_pid) {
                driver_mouse_event_t event = {0};
                rc = input_manager_read_mouse(&event);
                if (rc > 0 && copy_to_user_trusted(event_out, &event, sizeof(event)) != 0u) {
                    syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_mouse_buffer");
                    break;
                }
            }
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_IPC_SEND_MESSAGE: {
            int32_t target_pid = (int32_t)arg1;
            const void *message = (const void *)(uintptr_t)arg2;
            uint32_t size = (uint32_t)arg3;

            if (size > IPC_MESSAGE_MAX_SIZE || !user_buffer_ok(message, size)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_message_buffer");
                break;
            }

            uint8_t message_copy[IPC_MESSAGE_MAX_SIZE];
            if (size > 0u &&
                copy_from_user_trusted(message_copy, message, size) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_message_buffer");
                break;
            }

            os_status_t status = ipc_send_message(target_pid, message_copy, size);
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_IPC_RECEIVE_MESSAGE: {
            ipc_message_t *out_message = (ipc_message_t *)(uintptr_t)arg1;
            if (!user_buffer_ok(out_message, sizeof(ipc_message_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_message_buffer");
                break;
            }

            ipc_message_t message = {0};
            os_status_t status = ipc_receive_message(&message);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(out_message, &message, sizeof(message)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_message_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_PROCESS_GET_PID: {
            int32_t pid = process_get_current_pid();
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)pid);
            break;
        }

        case SYSCALL_GET_DISPLAY_WIDTH: {
            uint32_t width = driver_manager_display_width();
            set_syscall_result(saved_rsp, (uint64_t)width);
            break;
        }

        case SYSCALL_GET_DISPLAY_HEIGHT: {
            uint32_t height = driver_manager_display_height();
            set_syscall_result(saved_rsp, (uint64_t)height);
            break;
        }

        case SYSCALL_WINDOW_REGISTER_SERVICE: {
            /* Only one process may hold exclusive raw-input ownership at a
             * time; a live holder cannot be displaced (see
             * Syscall_InputOwner.h). */
            bool claimed = syscall_input_owner_claim(current_pid);
            set_syscall_status(saved_rsp,
                               claimed ? OS_STATUS_OK : OS_STATUS_ACCESS_DENIED);
            break;
        }

        case SYSCALL_WINDOW_GET_WM_PID: {
            int32_t wm_pid = syscall_input_owner_get();
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)wm_pid);
            break;
        }

        case SYSCALL_DISPLAY_DRAW_PIXEL:
            driver_manager_display_draw_pixel((uint32_t)arg1,
                                              (uint32_t)arg2,
                                              (uint32_t)arg3);
            set_syscall_result(saved_rsp, 0);
            break;

        case SYSCALL_DISPLAY_FILL_RECT:
            driver_manager_display_fill_rect((uint32_t)arg1,
                                             (uint32_t)arg2,
                                             (uint32_t)arg3,
                                             (uint32_t)arg4,
                                             (uint32_t)arg5);
            set_syscall_result(saved_rsp, 0);
            break;

        case SYSCALL_DISPLAY_PRESENT:
            driver_manager_display_present();
            process_perf_note_display(
                current_pid,
                1u,
                (uint64_t)driver_manager_display_width() *
                (uint64_t)driver_manager_display_height() * 4ULL);
            set_syscall_result(saved_rsp, 0);
            break;

        case SYSCALL_DISPLAY_PRESENT_RECTS: {
            const display_rect_t *user_rects =
                (const display_rect_t *)(uintptr_t)arg1;
            uint32_t count = (uint32_t)arg2;
            if (count == 0u) {
                driver_manager_display_present();
                process_perf_note_display(
                    current_pid,
                    1u,
                    (uint64_t)driver_manager_display_width() *
                    (uint64_t)driver_manager_display_height() * 4ULL);
                set_syscall_result(saved_rsp, 0);
                break;
            }
            if (count > SYSCALL_MAX_DISPLAY_RECTS ||
                !user_buffer_ok(user_rects,
                                (uint64_t)count * sizeof(display_rect_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_INVALID_ARG,
                             "invalid_display_rects");
                break;
            }
            display_rect_t rects[SYSCALL_MAX_DISPLAY_RECTS];
            if (copy_from_user_trusted(rects, user_rects,
                                       (uint64_t)count * sizeof(display_rect_t)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_display_rects");
                break;
            }
            uint64_t bytes = 0u;
            uint32_t display_w = driver_manager_display_width();
            uint32_t display_h = driver_manager_display_height();
            for (uint32_t i = 0u; i < count; ++i) {
                int64_t x0 = rects[i].x;
                int64_t y0 = rects[i].y;
                int64_t x1 = x0 + (int64_t)rects[i].w;
                int64_t y1 = y0 + (int64_t)rects[i].h;
                if (x0 < 0) x0 = 0;
                if (y0 < 0) y0 = 0;
                if (x1 > (int64_t)display_w) x1 = (int64_t)display_w;
                if (y1 > (int64_t)display_h) y1 = (int64_t)display_h;
                if (x1 > x0 && y1 > y0) {
                    bytes += (uint64_t)(x1 - x0) *
                             (uint64_t)(y1 - y0) * 4ULL;
                }
            }
            driver_manager_display_present_rects(rects, count);
            process_perf_note_display(current_pid, count, bytes);
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_GET_DISPLAY_FRAMEBUFFER: {
            void *ptr = driver_manager_display_get_framebuffer();
            if (ptr == NULL) {
                set_syscall_result(saved_rsp, 0);
                break;
            }
            uint64_t cr3 = process_get_current_cr3();
            uint32_t w = driver_manager_display_width();
            uint32_t h = driver_manager_display_height();
            uint32_t stride = w;
            display_mode_info_t mode_info = {0};
            if (driver_manager_display_get_monitor_mode_info(0u, 0u,
                                                              &mode_info) &&
                mode_info.stride >= w) {
                stride = mode_info.stride;
            }
            uint64_t size = (uint64_t)stride * (uint64_t)h * 4ULL;
            
            static uint64_t g_fb_virt_next = 0x0000004800000000ULL;
            uint64_t user_virt = __atomic_add_fetch(&g_fb_virt_next, 
                (size + 4095ULL) & ~4095ULL, __ATOMIC_RELAXED);
            
            uint64_t start_virt = (uint64_t)(uintptr_t)ptr;
            uint64_t aligned_start_virt = start_virt & ~4095ULL;
            uint64_t aligned_end_virt = (start_virt + size + 4095ULL) & ~4095ULL;
            uint64_t num_pages = (aligned_end_virt - aligned_start_virt) / 4096ULL;

            int res = 0;
            for (uint64_t i = 0; i < num_pages; ++i) {
                uint64_t kvirt = aligned_start_virt + i * 4096ULL;

                uint64_t phys_page = paging_virt_to_phys(paging_get_kernel_cr3(), kvirt);
                if (phys_page == 0) {
                    res = -1;
                    break;
                }
                
                uint64_t virt_page = user_virt + i * 4096ULL;

                if (paging_map_user_page(cr3, virt_page, phys_page, PAGE_RW | PAGE_USER | PAGE_EXTERNAL) < 0) {
                    res = -1;
                    break;
                }
            }
            if (res < 0) {
                set_syscall_result(saved_rsp, 0);
            } else {
                set_syscall_result(saved_rsp, user_virt + (start_virt & 4095ULL));
            }
            break;
        }

        case SYSCALL_DISPLAY_GET_PIXEL: {
            uint32_t color = driver_manager_display_get_pixel((uint32_t)arg1, (uint32_t)arg2);
            set_syscall_result(saved_rsp, (uint64_t)color);
            break;
        }

        case SYSCALL_DISPLAY_GET_TOPOLOGY: {
            display_topology_t *topology_out =
                (display_topology_t *)(uintptr_t)arg1;
            if (!user_buffer_ok(topology_out, sizeof(display_topology_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_display_topology_buffer");
                break;
            }
            display_topology_t topology = {0};
            if (!driver_manager_display_get_topology(&topology) ||
                copy_to_user_trusted(topology_out, &topology, sizeof(topology)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_IO_ERROR,
                             "display_topology_unavailable");
                break;
            }
            set_syscall_status(saved_rsp, OS_STATUS_OK);
            break;
        }

        case SYSCALL_DISPLAY_GET_MONITOR_INFO: {
            display_monitor_info_t *info_out =
                (display_monitor_info_t *)(uintptr_t)arg2;
            if (!user_buffer_ok(info_out, sizeof(display_monitor_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_monitor_info_buffer");
                break;
            }
            display_monitor_info_t info = {0};
            if (!driver_manager_display_get_monitor_info((uint32_t)arg1,
                                                         &info)) {
                syscall_fail(saved_rsp, num, OS_STATUS_NOT_FOUND,
                             "monitor_not_found");
                break;
            }
            if (copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_monitor_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, OS_STATUS_OK);
            break;
        }

        case SYSCALL_DISPLAY_GET_MONITOR_MODE_INFO: {
            display_mode_info_t *info_out =
                (display_mode_info_t *)(uintptr_t)arg3;
            if (!user_buffer_ok(info_out, sizeof(display_mode_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_monitor_mode_info_buffer");
                break;
            }
            display_mode_info_t info = {0};
            if (!driver_manager_display_get_monitor_mode_info((uint32_t)arg1,
                                                              (uint32_t)arg2,
                                                              &info)) {
                syscall_fail(saved_rsp, num, OS_STATUS_NOT_FOUND,
                             "monitor_mode_not_found");
                break;
            }
            if (copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_monitor_mode_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, OS_STATUS_OK);
            break;
        }

        case SYSCALL_DISPLAY_SET_MONITOR_MODE:
            set_syscall_status(saved_rsp,
                driver_manager_display_set_monitor_mode((uint32_t)arg1,
                                                        (uint32_t)arg2) ?
                OS_STATUS_OK : OS_STATUS_NOT_SUPPORTED);
            break;

        case SYSCALL_SYSTEM_SHUTDOWN: {
            extern void acpi_shutdown(void);
            acpi_shutdown();
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_SYSTEM_SHUTDOWN_BROADCAST: {
            extern void process_broadcast_shutdown(void);
            process_broadcast_shutdown();
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_SYSTEM_REBOOT: {
            extern void acpi_reboot(void);
            acpi_reboot();
            set_syscall_result(saved_rsp, 0);
            break;
        }
        case SYSCALL_PROCESS_WAITPID: {
            int32_t *status_ptr = (int32_t *)(uintptr_t)arg2;
            if (status_ptr != NULL && !user_buffer_ok(status_ptr, sizeof(int32_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_status_buffer");
                break;
            }
            int32_t child_status = 0;
            int32_t result = process_waitpid((int32_t)arg1,
                                             status_ptr != NULL ? &child_status : NULL,
                                             (int32_t)arg3);
            if (result > 0 && status_ptr != NULL &&
                copy_to_user_trusted(status_ptr, &child_status, sizeof(child_status)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_status_buffer");
                break;
            }
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_GET_RTC_TIME: {
            rtc_time_t *out = (rtc_time_t *)(uintptr_t)arg1;
            if (!user_buffer_ok(out, sizeof(rtc_time_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_rtc_buffer");
                break;
            }
            rtc_time_t time = {0};
            rtc_read_time(&time);
            if (copy_to_user_trusted(out, &time, sizeof(time)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_rtc_buffer");
                break;
            }
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_PROCESS_GETPPID: {
            int32_t ppid = process_getppid();
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)ppid);
            break;
        }

        case SYSCALL_PROCESS_EXIT_STATUS: {
            process_exit_current_with_status((int32_t)arg1);
            request_switch = 1;
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_SLEEP: {
            uint64_t ms = arg1;
            if (ms > 0u && process_sleep_current_ms(ms) == 0) {
                request_switch |= PROCESS_SCHEDULE_REQUEST_SWITCH;
            }
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_NANOSLEEP: {
            uint64_t ns = arg1;
            uint64_t ms = (ns + 999999ULL) / 1000000ULL;
            if (ms > 0u && process_sleep_current_ms(ms) == 0) {
                request_switch |= PROCESS_SCHEDULE_REQUEST_SWITCH;
            }
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_GET_UPTIME_MS: {
            uint32_t hz = timer_hz();
            if (hz == 0) hz = 60;
            uint64_t ticks = timer_ticks();
            uint64_t ms = (ticks * 1000) / hz;
            set_syscall_result(saved_rsp, ms);
            break;
        }

        case SYSCALL_FILE_STAT: {
            char path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_path");
                break;
            }

            syscall_file_stat_t *stat_out = (void *)(uintptr_t)arg2;
            if (!user_buffer_ok(stat_out, sizeof(syscall_file_stat_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_stat_buffer");
                break;
            }
            vfs_file_t vf;
            syscall_file_stat_t stat = {0};
            if (vfs_find_file(path, &vf)) {
                stat.size = vf.size;
                stat.is_dir = 0;
                stat.exists = 1;
            } else {
                int32_t dir_handle = vfs_opendir(path);
                if (dir_handle >= 0) {
                    (void)vfs_closedir(dir_handle);
                    stat.size = 0;
                    stat.is_dir = 1;
                    stat.exists = 1;
                } else {
                    stat.size = 0;
                    stat.is_dir = 0;
                    stat.exists = 0;
                }
            }
            if (copy_to_user_trusted(stat_out, &stat, sizeof(stat)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_stat_buffer");
                break;
            }
            set_syscall_result(saved_rsp,
                               stat.exists ? 0u : (uint64_t)(int64_t)OS_STATUS_NOT_FOUND);
            break;
        }

        case SYSCALL_GET_PROC_COUNT: {
            set_syscall_result(saved_rsp, (uint64_t)process_get_capacity());
            break;
        }

        case SYSCALL_GET_PROC_INFO: {
            int32_t query_pid = (int32_t)arg1;
            void *info_out = (void *)(uintptr_t)arg2;
            if (!user_buffer_ok(info_out, sizeof(process_info_kernel_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_info_buffer");
                break;
            }
            process_info_kernel_t info = {0};
            int32_t rc = process_get_full_info(query_pid, &info);
            if (rc == 0 && copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_info_buffer");
                break;
            }
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_GET_PROC_PERF_INFO: {
            int32_t query_pid = (int32_t)arg1;
            process_perf_info_t *info_out =
                (process_perf_info_t *)(uintptr_t)arg2;
            if (!user_buffer_ok(info_out, sizeof(process_perf_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_perf_info_buffer");
                break;
            }
            process_perf_info_t info = {0};
            int32_t rc = process_get_perf_info(query_pid, &info);
            if (rc == 0 &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_perf_info_buffer");
                break;
            }
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_GET_BOOT_PROFILE_COUNT: {
            set_syscall_i32(saved_rsp, kernel_boot_profile_count());
            break;
        }

        case SYSCALL_GET_BOOT_PROFILE_ENTRY: {
            int32_t index = (int32_t)arg1;
            boot_profile_entry_t *entry_out =
                (boot_profile_entry_t *)(uintptr_t)arg2;
            if (!user_buffer_ok(entry_out, sizeof(boot_profile_entry_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_boot_profile_buffer");
                break;
            }
            boot_profile_entry_t entry = {0};
            int32_t rc = kernel_boot_profile_get(index, &entry);
            if (rc == 0 &&
                copy_to_user_trusted(entry_out, &entry, sizeof(entry)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_boot_profile_buffer");
                break;
            }
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_GET_TOTAL_MEMORY: {
            extern uint64_t get_total_memory_pages(void);
            set_syscall_result(saved_rsp, get_total_memory_pages() * 4096);
            break;
        }

        case SYSCALL_GET_USED_MEMORY: {
            extern uint64_t get_used_memory(void);
            set_syscall_result(saved_rsp, get_used_memory());
            break;
        }

        case SYSCALL_FILE_PIPE: {
            int32_t *fds_ptr = (int32_t *)(uintptr_t)arg1;
            if (!user_buffer_ok(fds_ptr, sizeof(int32_t) * 2)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_pipe_buffer");
                break;
            }
            int32_t fds[2] = {-1, -1};
            int32_t result = syscall_file_pipe(fds);
            if (result == 0 &&
                copy_to_user_trusted(fds_ptr, fds, sizeof(fds)) != 0u) {
                (void)syscall_file_close(fds[0]);
                (void)syscall_file_close(fds[1]);
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_pipe_buffer");
                break;
            }
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_FILE_DUP: {
            int32_t result = syscall_file_dup((int32_t)arg1);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_FILE_DUP2: {
            int32_t result = syscall_file_dup2((int32_t)arg1, (int32_t)arg2);
            set_syscall_result(saved_rsp, (uint64_t)(int64_t)result);
            break;
        }

        case SYSCALL_SOCKET_CREATE: {
            set_syscall_i32(saved_rsp, syscall_socket_create((int32_t)arg1));
            break;
        }

        case SYSCALL_SOCKET_CONNECT: {
            set_syscall_i32(saved_rsp, syscall_socket_connect(
                (int32_t)arg1, (uint32_t)arg2, (uint16_t)arg3));
            break;
        }

        case SYSCALL_SOCKET_BIND: {
            set_syscall_i32(saved_rsp, syscall_socket_bind(
                (int32_t)arg1, (uint16_t)arg2));
            break;
        }

        case SYSCALL_SOCKET_LISTEN: {
            set_syscall_i32(saved_rsp, syscall_socket_listen((int32_t)arg1));
            break;
        }

        case SYSCALL_SOCKET_ACCEPT: {
            set_syscall_i32(saved_rsp, syscall_socket_accept((int32_t)arg1));
            break;
        }

        case SYSCALL_SOCKET_SEND: {
            uint16_t length = (uint16_t)arg3;
            const void *user_data = (const void *)(uintptr_t)arg2;
            if ((uint64_t)length != arg3 || !user_buffer_ok(user_data, length)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_socket_send_buffer");
                break;
            }
            uint8_t *data = length != 0u ? (uint8_t *)malloc(length) : NULL;
            if (length != 0u &&
                (data == NULL ||
                 copy_from_user_trusted(data, user_data, length) != 0u)) {
                free(data);
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_socket_send_buffer");
                break;
            }
            int32_t sent = syscall_socket_send((int32_t)arg1, data, length);
            free(data);
            set_syscall_i32(saved_rsp, sent);
            break;
        }

        case SYSCALL_SOCKET_RECV: {
            uint16_t length = (uint16_t)arg3;
            void *user_data = (void *)(uintptr_t)arg2;
            if ((uint64_t)length != arg3 || !user_buffer_ok(user_data, length)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_socket_recv_buffer");
                break;
            }
            uint8_t *data = length != 0u ? (uint8_t *)malloc(length) : NULL;
            if (length != 0u && data == NULL) {
                syscall_fail(saved_rsp, num, OS_STATUS_LIMIT_REACHED,
                             "socket_recv_allocation_failed");
                break;
            }
            int32_t received = syscall_socket_recv((int32_t)arg1, data, length);
            if (received > 0 &&
                copy_to_user_trusted(user_data, data, (uint64_t)received) != 0u) {
                free(data);
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_socket_recv_buffer");
                break;
            }
            free(data);
            set_syscall_i32(saved_rsp, received);
            break;
        }

        case SYSCALL_SOCKET_CLOSE: {
            set_syscall_i32(saved_rsp, syscall_socket_close((int32_t)arg1));
            break;
        }

        case SYSCALL_SOCKET_GET_INFO: {
            syscall_socket_info_t info;
            int32_t result = syscall_socket_get_info((int32_t)arg1, &info);
            if (result == 0 &&
                copy_to_user((void *)(uintptr_t)arg2,
                             &info, sizeof(info)) != 0u) {
                result = (int32_t)OS_STATUS_FAULT;
            }
            set_syscall_i32(saved_rsp, result);
            break;
        }

        case SYSCALL_SOCKET_SET_OPTION:
            set_syscall_i32(saved_rsp, syscall_socket_set_option(
                (int32_t)arg1, (int32_t)arg2, (int32_t)arg3, (int32_t)arg4));
            break;

        case SYSCALL_SOCKET_GET_OPTION: {
            int32_t value = 0;
            int32_t result = syscall_socket_get_option(
                (int32_t)arg1, (int32_t)arg2, (int32_t)arg3, &value);
            if (result == 0 &&
                copy_to_user((void *)(uintptr_t)arg4,
                             &value, sizeof(value)) != 0u) {
                result = (int32_t)OS_STATUS_FAULT;
            }
            set_syscall_i32(saved_rsp, result);
            break;
        }

        case SYSCALL_SOCKET_SHUTDOWN:
            set_syscall_i32(saved_rsp, syscall_socket_shutdown(
                (int32_t)arg1, (int32_t)arg2));
            break;

        case SYSCALL_SOCKET_LISTEN_EX:
            set_syscall_i32(saved_rsp, syscall_socket_listen_with_backlog(
                (int32_t)arg1, (int32_t)arg2));
            break;

        case SYSCALL_MPROTECT: {
            extern int64_t syscall_vm_mprotect(uint64_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_vm_mprotect(arg1, arg2, arg3));
            break;
        }
        case SYSCALL_MUNMAP: {
            extern int64_t syscall_vm_munmap(uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_vm_munmap(arg1, arg2));
            break;
        }
        case SYSCALL_MREMAP: {
            extern int64_t syscall_vm_mremap(uint64_t, uint64_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_vm_mremap(arg1, arg2, arg3, arg4));
            break;
        }

        case SYSCALL_GETUID:
        case SYSCALL_GETEUID:
        case SYSCALL_GETGID:
        case SYSCALL_GETEGID:
            set_syscall_result(saved_rsp, 0);
            break;
        case SYSCALL_GETTID: {
            extern int64_t syscall_gettid(void);
            set_syscall_result(saved_rsp, (uint64_t)syscall_gettid());
            break;
        }
        case SYSCALL_SET_TID_ADDRESS: {
            extern int64_t syscall_set_tid_address(uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_set_tid_address(arg1));
            break;
        }

        case SYSCALL_EPOLL_CREATE: {
            extern int32_t syscall_epoll_create(uint64_t);
            set_syscall_i32(saved_rsp, syscall_epoll_create(arg1));
            break;
        }
        case SYSCALL_EPOLL_CTL: {
            extern int32_t syscall_epoll_ctl(int32_t, int32_t, int32_t, const void *);
            set_syscall_i32(saved_rsp, syscall_epoll_ctl((int32_t)arg1, (int32_t)arg2,
                            (int32_t)arg3, (const void *)(uintptr_t)arg4));
            break;
        }
        case SYSCALL_EPOLL_WAIT: {
            extern int32_t syscall_epoll_wait(int32_t, void *, int32_t, int32_t);
            set_syscall_i32(saved_rsp, syscall_epoll_wait((int32_t)arg1,
                            (void *)(uintptr_t)arg2, (int32_t)arg3, (int32_t)arg4));
            break;
        }
        case SYSCALL_EVENTFD: {
            extern int32_t syscall_eventfd(uint64_t, uint64_t);
            set_syscall_i32(saved_rsp, syscall_eventfd(arg1, arg2));
            break;
        }

        case SYSCALL_CLOCK_GETTIME: {
            extern int64_t syscall_clock_gettime(int32_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_clock_gettime((int32_t)arg1, arg2));
            break;
        }
        case SYSCALL_CLOCK_GETRES: {
            extern int64_t syscall_clock_getres(int32_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_clock_getres((int32_t)arg1, arg2));
            break;
        }

        case SYSCALL_ARCH_PRCTL: {
            extern int64_t syscall_arch_prctl(uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_arch_prctl(arg1, arg2));
            break;
        }
        case SYSCALL_PRLIMIT64: {
            extern int64_t syscall_prlimit64(uint64_t, uint64_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_prlimit64(arg1, arg2, arg3, arg4));
            break;
        }
        case SYSCALL_GETRANDOM: {
            extern int64_t syscall_getrandom(uint64_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_getrandom(arg1, arg2, arg3));
            break;
        }
        case SYSCALL_READV: {
            extern int64_t syscall_readv(int32_t, uint64_t, int32_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_readv((int32_t)arg1, arg2, (int32_t)arg3));
            break;
        }
        case SYSCALL_WRITEV: {
            extern int64_t syscall_writev(int32_t, uint64_t, int32_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_writev((int32_t)arg1, arg2, (int32_t)arg3));
            break;
        }
        case SYSCALL_FTRUNCATE: {
            extern int64_t syscall_ftruncate(int32_t, int64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_ftruncate((int32_t)arg1, (int64_t)arg2));
            break;
        }
        case SYSCALL_FCHMOD: {
            set_syscall_result(saved_rsp,
                               (uint64_t)(int64_t)OS_STATUS_NOT_SUPPORTED);
            break;
        }
        case SYSCALL_RENAME: {
            char old_path[SYSCALL_MAX_PATH_LEN];
            char new_path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(old_path, sizeof(old_path),
                                  (const char *)(uintptr_t)arg1) < 0 ||
                copy_user_cstring(new_path, sizeof(new_path),
                                  (const char *)(uintptr_t)arg2) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_rename_path");
                break;
            }
            set_syscall_result(saved_rsp,
                (uint64_t)(int64_t)syscall_file_rename(old_path, new_path));
            break;
        }
        case SYSCALL_IOCTL_EX: {
            extern int64_t syscall_ioctl_ex(int32_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_ioctl_ex((int32_t)arg1, arg2, arg3));
            break;
        }
        case SYSCALL_FCNTL_EX: {
            extern int64_t syscall_fcntl_ex(int32_t, int32_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_fcntl_ex((int32_t)arg1, (int32_t)arg2, arg3));
            break;
        }
        case SYSCALL_ACCESS: {
            char path[SYSCALL_MAX_PATH_LEN];
            if (copy_user_cstring(path, sizeof(path),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_access_path");
                break;
            }
            extern int64_t syscall_access(const char *, int32_t);
            set_syscall_result(saved_rsp,
                (uint64_t)syscall_access(path, (int32_t)arg2));
            break;
        }
        case SYSCALL_FD_POLL: {
            uint32_t events = (uint32_t)arg2;
            uint32_t ready = syscall_file_poll((int32_t)arg1, events);
            if ((ready & 0x0020u) != 0u)
                ready = syscall_socket_poll((int32_t)arg1, events);
            set_syscall_result(saved_rsp, ready);
            break;
        }
        case SYSCALL_SET_ROBUST_LIST: {
            set_syscall_result(saved_rsp,
                (uint64_t)(int64_t)process_set_robust_list(arg1, arg2));
            break;
        }

        case SYSCALL_FUTEX: {
            extern int64_t syscall_futex(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);
            int64_t result =
                syscall_futex(arg1, arg2, arg3, arg4, arg5, 0);
            set_syscall_result(saved_rsp, (uint64_t)result);
            uint64_t command = arg2 & 0x7fULL;
            if (result == 0 && (command == 0 || command == 9)) {
                request_switch = 1;
            }
            break;
        }

        case SYSCALL_CLONE: {
            int32_t tid = process_create_thread(arg1, arg2, arg3, arg4, arg5);
            set_syscall_i32(saved_rsp, tid);
            break;
        }

        case SYSCALL_RT_SIGACTION: {
            extern int64_t syscall_rt_sigaction(uint64_t, uint64_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_rt_sigaction(arg1, arg2, arg3, arg4));
            break;
        }
        case SYSCALL_RT_SIGPROCMASK: {
            extern int64_t syscall_rt_sigprocmask(uint64_t, uint64_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)syscall_rt_sigprocmask(arg1, arg2, arg3, arg4));
            break;
        }
        case SYSCALL_RT_SIGRETURN: {
            set_syscall_result(saved_rsp,
                               (uint64_t)(int64_t)OS_STATUS_NOT_SUPPORTED);
            break;
        }
        case SYSCALL_SIGALTSTACK: {
            set_syscall_result(saved_rsp,
                               (uint64_t)(int64_t)OS_STATUS_NOT_SUPPORTED);
            break;
        }
        case SYSCALL_TKILL: {
            int32_t target_pid = (int32_t)arg1;
            int32_t rc;
            if ((int32_t)arg2 == 0) {
                rc = process_is_alive(target_pid) ? 0 : -3;
            } else {
                rc = process_signal_deliver(target_pid, (int32_t)arg2);
                if (rc < 0) rc = -3;
            }
            set_syscall_i32(saved_rsp, rc);
            break;
        }
        case SYSCALL_SHM_CREATE:
            set_syscall_i32(saved_rsp, shared_memory_create((uint32_t)arg1));
            break;
        case SYSCALL_SHM_GRANT:
            set_syscall_i32(saved_rsp,
                            shared_memory_grant((int32_t)arg1, (int32_t)arg2));
            break;
        case SYSCALL_SHM_MAP:
            set_syscall_result(saved_rsp,
                (uint64_t)(uintptr_t)shared_memory_map((int32_t)arg1));
            break;
        case SYSCALL_SHM_UNMAP:
            set_syscall_i32(saved_rsp,
                shared_memory_unmap((int32_t)arg1, (void *)(uintptr_t)arg2));
            break;
        case SYSCALL_SHM_CLOSE:
            set_syscall_i32(saved_rsp, shared_memory_close((int32_t)arg1));
            break;
        case SYSCALL_MEMFD_SHM_HANDLE:
            set_syscall_i32(saved_rsp, syscall_memfd_shm_handle((int32_t)arg1));
            break;
        case SYSCALL_GET_MAIN_IMAGE_INFO: {
            struct {
                uint64_t phdr_vaddr;
                uint64_t phent;
                uint64_t phnum;
            } info;
            int64_t result = process_get_main_image_info(&info.phdr_vaddr,
                                                         &info.phent,
                                                         &info.phnum);
            if (result == 0 &&
                copy_to_user((void *)(uintptr_t)arg1,
                             &info, sizeof(info)) != 0u) {
                result = (int64_t)OS_STATUS_FAULT;
            }
            set_syscall_result(saved_rsp, (uint64_t)result);
            break;
        }
        
        case SYSCALL_FORK: {
            int32_t child_pid = process_fork();
            set_syscall_i32(saved_rsp, child_pid);
            if (child_pid > 0) request_switch = 1;
            break;
        }
        case SYSCALL_EXECVE: {
            int32_t result = process_execve(
                (const char *)arg1,
                (const char *const *)(uintptr_t)arg2,
                (const char *const *)(uintptr_t)arg3);
            set_syscall_i32(saved_rsp, result);
            request_switch = 1;
            break;
        }
        case SYSCALL_VFORK: {
            int32_t child_pid = process_fork();
            set_syscall_i32(saved_rsp, child_pid);
            if (child_pid > 0) request_switch = 1;
            break;
        }

        case SYSCALL_DRM_OPEN: {
            extern int64_t drm_client_open(void);
            set_syscall_result(saved_rsp, (uint64_t)drm_client_open());
            break;
        }
        case SYSCALL_DRM_IOCTL: {
            extern int64_t drm_client_ioctl(int32_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)drm_client_ioctl((int32_t)arg1, arg2, arg3));
            break;
        }
        case SYSCALL_DRM_CLOSE: {
            extern int64_t drm_client_close(int32_t);
            set_syscall_result(saved_rsp, (uint64_t)drm_client_close((int32_t)arg1));
            break;
        }
        case SYSCALL_DRM_MMAP: {
            extern void *drm_client_mmap(int32_t, uint64_t, uint64_t);
            void *p = drm_client_mmap((int32_t)arg1, arg2, arg3);
            set_syscall_result(saved_rsp, (uint64_t)(uintptr_t)p);
            break;
        }

        case SYSCALL_EVDEV_OPEN: {
            extern int64_t evdev_open(const char*);
            char kpath[256];
            if (arg1 == 0 || !copy_user_cstring(kpath, sizeof(kpath), (const char*)(uintptr_t)arg1)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_evdev_path");
                break;
            }
            set_syscall_result(saved_rsp, (uint64_t)evdev_open(kpath));
            break;
        }
        case SYSCALL_EVDEV_READ: {
            extern int64_t evdev_read(int32_t, void*, uint64_t);
            uint64_t len = arg3;
            void *user_buf = (void*)(uintptr_t)arg2;
            if (len == 0 || !user_buffer_ok(user_buf, len)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_evdev_read_buffer");
                break;
            }
            uint8_t *kbuf = (uint8_t *)malloc(len);
            if (kbuf == NULL) {
                syscall_fail(saved_rsp, num, OS_STATUS_LIMIT_REACHED, "evdev_read_alloc_failed");
                break;
            }
            int64_t result = evdev_read((int32_t)arg1, kbuf, len);
            if (result > 0 && copy_to_user_trusted(user_buf, kbuf, (uint64_t)result) != 0u) {
                free(kbuf);
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_evdev_read_buffer");
                break;
            }
            free(kbuf);
            set_syscall_result(saved_rsp, (uint64_t)result);
            break;
        }
        case SYSCALL_EVDEV_IOCTL: {
            extern int64_t evdev_ioctl(int32_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)evdev_ioctl((int32_t)arg1, arg2, arg3));
            break;
        }
        case SYSCALL_EVDEV_CLOSE: {
            extern int64_t evdev_close(int32_t);
            set_syscall_result(saved_rsp, (uint64_t)evdev_close((int32_t)arg1));
            break;
        }

        case SYSCALL_UNIX_SOCKET: {
            extern int64_t unix_socket_create(int32_t);
            set_syscall_result(saved_rsp, (uint64_t)unix_socket_create((int32_t)arg1));
            break;
        }
        case SYSCALL_UNIX_BIND: {
            extern int64_t unix_socket_bind(int32_t, const char*);
            char kpath[108];
            /* copy_user_cstring() returns 0 on success, <0 on fault. */
            if (arg2 == 0 || copy_user_cstring(kpath, sizeof(kpath), (const char*)(uintptr_t)arg2) != 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_bind_path");
                break;
            }
            set_syscall_result(saved_rsp, (uint64_t)unix_socket_bind((int32_t)arg1, kpath));
            break;
        }
        case SYSCALL_UNIX_LISTEN: {
            extern int64_t unix_socket_listen(int32_t, int32_t);
            set_syscall_result(saved_rsp, (uint64_t)unix_socket_listen((int32_t)arg1, (int32_t)arg2));
            break;
        }
        case SYSCALL_UNIX_ACCEPT: {
            extern int64_t unix_socket_accept(int32_t);
            set_syscall_result(saved_rsp, (uint64_t)unix_socket_accept((int32_t)arg1));
            break;
        }
        case SYSCALL_UNIX_CONNECT: {
            extern int64_t unix_socket_connect(int32_t, const char*);
            char kpath[108];
            /* copy_user_cstring() returns 0 on success, <0 on fault. */
            if (arg2 == 0 || copy_user_cstring(kpath, sizeof(kpath), (const char*)(uintptr_t)arg2) != 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_connect_path");
                break;
            }
            set_syscall_result(saved_rsp, (uint64_t)unix_socket_connect((int32_t)arg1, kpath));
            break;
        }
        case SYSCALL_UNIX_SEND: {
            extern int64_t unix_socket_send(int32_t, const void*, uint64_t);
            uint16_t length = (uint16_t)arg3;
            const void *user_data = (const void*)(uintptr_t)arg2;
            if ((uint64_t)length != arg3 || !user_buffer_ok(user_data, length)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_send_buffer");
                break;
            }
            uint8_t *data = length != 0u ? (uint8_t *)malloc(length) : NULL;
            if (length != 0u &&
                (data == NULL ||
                 copy_from_user_trusted(data, user_data, length) != 0u)) {
                free(data);
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_send_buffer");
                break;
            }
            int64_t sent = unix_socket_send((int32_t)arg1, data, length);
            free(data);
            set_syscall_result(saved_rsp, (uint64_t)sent);
            break;
        }
        case SYSCALL_UNIX_RECV: {
            extern int64_t unix_socket_recv(int32_t, void*, uint64_t);
            uint16_t length = (uint16_t)arg3;
            void *user_data = (void*)(uintptr_t)arg2;
            if ((uint64_t)length != arg3 || !user_buffer_ok(user_data, length)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_recv_buffer");
                break;
            }
            uint8_t *data = length != 0u ? (uint8_t *)malloc(length) : NULL;
            if (length != 0u && data == NULL) {
                syscall_fail(saved_rsp, num, OS_STATUS_LIMIT_REACHED, "unix_recv_alloc_failed");
                break;
            }
            int64_t received = unix_socket_recv((int32_t)arg1, data, length);
            if (received > 0 &&
                copy_to_user_trusted(user_data, data, (uint64_t)received) != 0u) {
                free(data);
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_recv_buffer");
                break;
            }
            free(data);
            set_syscall_result(saved_rsp, (uint64_t)received);
            break;
        }
        case SYSCALL_UNIX_SENDMSG: {
            extern int64_t unix_socket_sendmsg(int32_t, uint64_t);
            if (arg2 == 0 || !user_buffer_ok((const void*)(uintptr_t)arg2, 64)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_sendmsg_buffer");
                break;
            }
            uint8_t kmsg[64];
            if (copy_from_user_trusted(kmsg, (const void*)(uintptr_t)arg2, 64) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_sendmsg_buffer");
                break;
            }
            set_syscall_result(saved_rsp, (uint64_t)unix_socket_sendmsg((int32_t)arg1, (uint64_t)(uintptr_t)kmsg));
            break;
        }
        case SYSCALL_UNIX_RECVMSG: {
            extern int64_t unix_socket_recvmsg(int32_t, uint64_t);
            if (arg2 == 0 || !user_buffer_ok((const void*)(uintptr_t)arg2, 64)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_recvmsg_buffer");
                break;
            }
            uint8_t kmsg[64];
            if (copy_from_user_trusted(kmsg, (const void*)(uintptr_t)arg2, 64) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_recvmsg_buffer");
                break;
            }
            int64_t result = unix_socket_recvmsg((int32_t)arg1, (uint64_t)(uintptr_t)kmsg);
            if (result >= 0 &&
                copy_to_user_trusted((void*)(uintptr_t)arg2, kmsg, 64) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_unix_recvmsg_buffer");
                break;
            }
            set_syscall_result(saved_rsp, (uint64_t)result);
            break;
        }
        case SYSCALL_UNIX_CLOSE: {
            extern int64_t unix_socket_close(int32_t);
            set_syscall_result(saved_rsp, (uint64_t)unix_socket_close((int32_t)arg1));
            break;
        }

        case SYSCALL_KVM_OPEN: {
            extern int64_t kvm_client_open(void);
            set_syscall_result(saved_rsp, (uint64_t)kvm_client_open());
            break;
        }
        case SYSCALL_KVM_IOCTL: {
            extern int64_t kvm_client_ioctl(int32_t, uint64_t, uint64_t);
            set_syscall_result(saved_rsp, (uint64_t)kvm_client_ioctl((int32_t)arg1, arg2, arg3));
            break;
        }
        case SYSCALL_KVM_CLOSE: {
            extern int64_t kvm_client_close(int32_t);
            set_syscall_result(saved_rsp, (uint64_t)kvm_client_close((int32_t)arg1));
            break;
        }
        case SYSCALL_KVM_MMAP: {
            extern void *kvm_client_mmap(int32_t, uint64_t, uint64_t);
            void *p = kvm_client_mmap((int32_t)arg1, arg2, arg3);
            if (p != NULL) {
                uint64_t cr3 = process_get_current_cr3();
                uint64_t start = (uint64_t)(uintptr_t)p;
                uint64_t aligned_start = start & ~4095ULL;
                uint64_t aligned_size = (arg3 + 4095ULL) & ~4095ULL;
                if (aligned_size == 0) aligned_size = 4096;
                paging_set_user_access(cr3, aligned_start, aligned_size, 1);
            }
            set_syscall_result(saved_rsp, (uint64_t)(uintptr_t)p);
            break;
        }
        
        case SYSCALL_GET_CPU_INFO: {
            extern os_status_t sysinfo_get_cpu_info(system_cpu_info_t *);
            void *info_out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(info_out, sizeof(system_cpu_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_cpu_info_buffer");
                break;
            }
            system_cpu_info_t info = {0};
            os_status_t status = sysinfo_get_cpu_info(&info);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_cpu_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_GET_MEMORY_INFO: {
            extern os_status_t sysinfo_get_memory_info(system_memory_info_t *);
            void *info_out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(info_out, sizeof(system_memory_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_memory_info_buffer");
                break;
            }
            system_memory_info_t info = {0};
            os_status_t status = sysinfo_get_memory_info(&info);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_memory_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_GET_VMEM_INFO: {
            extern os_status_t sysinfo_get_vmem_info(system_vmem_info_t *);
            void *info_out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(info_out, sizeof(system_vmem_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_vmem_info_buffer");
                break;
            }
            system_vmem_info_t info = {0};
            os_status_t status = sysinfo_get_vmem_info(&info);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_vmem_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_GET_DISK_INFO: {
            extern os_status_t sysinfo_get_disk_info(uint32_t, system_disk_info_t *);
            uint32_t index = (uint32_t)arg1;
            void *info_out = (void *)(uintptr_t)arg2;
            if (!user_buffer_ok(info_out, sizeof(system_disk_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_disk_info_buffer");
                break;
            }
            system_disk_info_t info = {0};
            os_status_t status = sysinfo_get_disk_info(index, &info);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_disk_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_GET_DISK_COUNT: {
            extern os_status_t sysinfo_get_disk_count(uint32_t *);
            void *count_out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(count_out, sizeof(uint32_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_disk_count_buffer");
                break;
            }
            uint32_t count = 0;
            os_status_t status = sysinfo_get_disk_count(&count);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(count_out, &count, sizeof(count)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_disk_count_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_RAW_BLOCK_READ:
        case SYSCALL_RAW_BLOCK_WRITE: {
            uint32_t disk_index = (uint32_t)arg1;
            uint64_t lba = arg2;
            void *buffer = (void *)(uintptr_t)arg3;
            uint32_t sectors = (uint32_t)arg4;
            uint64_t byte_count = (uint64_t)sectors * 512ULL;

            if ((uint64_t)sectors != arg4 || byte_count > SYSCALL_MAX_IO_BYTES) {
                syscall_fail(saved_rsp, num, OS_STATUS_INVALID_ARG, "invalid_raw_block_size");
                break;
            }
            if (sectors != 0 && !user_buffer_ok(buffer, byte_count)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_raw_block_buffer");
                break;
            }

            bool ok = false;
            uint8_t *bounce = NULL;
            if (byte_count != 0u) {
                bounce = (uint8_t *)malloc((size_t)byte_count);
                if (bounce == NULL) {
                    syscall_fail(saved_rsp, num, OS_STATUS_LIMIT_REACHED, "raw_block_alloc_failed");
                    break;
                }
            }

            if (num == SYSCALL_RAW_BLOCK_READ) {
                ok = disk_raw_read(disk_index, lba, bounce, sectors);
                if (ok && byte_count != 0u &&
                    copy_to_user_trusted(buffer, bounce, byte_count) != 0u) {
                    free(bounce);
                    syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_raw_block_buffer");
                    break;
                }
            } else {
                if (byte_count != 0u &&
                    copy_from_user_trusted(bounce, buffer, byte_count) != 0u) {
                    free(bounce);
                    syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_raw_block_buffer");
                    break;
                }
                ok = disk_raw_write(disk_index, lba, bounce, sectors);
            }
            if (bounce != NULL) {
                free(bounce);
            }
            set_syscall_status(saved_rsp, ok ? OS_STATUS_OK : OS_STATUS_IO_ERROR);
            break;
        }

        case SYSCALL_GET_BOOT_FONT: {
            extern const BOOT_INFO *kernel_get_boot_info(void);
            const BOOT_INFO *boot_info = kernel_get_boot_info();
            uint64_t font_addr = boot_info ? boot_info->FontDataAddress : 0;
            uint64_t font_size = boot_info ? boot_info->FontDataSize : 0;

            if (font_addr == 0 || font_size == 0) {
                set_syscall_result(saved_rsp, 0);
                break;
            }

            void *buffer = (void *)(uintptr_t)arg1;
            uint64_t capacity = arg2;
            if (buffer == NULL || capacity == 0) {
                set_syscall_result(saved_rsp, font_size);
                break;
            }
            if (capacity < font_size) {
                syscall_fail(saved_rsp, num, OS_STATUS_INVALID_ARG, "boot_font_buffer_too_small");
                break;
            }
            if (!user_buffer_ok(buffer, font_size)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_boot_font_buffer");
                break;
            }

            if (copy_to_user_trusted(buffer,
                                    (const void *)(uintptr_t)font_addr,
                                    font_size) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_boot_font_buffer");
                break;
            }
            set_syscall_result(saved_rsp, font_size);
            break;
        }

        case SYSCALL_GET_DEVICE_INFO: {
            extern os_status_t sysinfo_get_device_info(uint32_t, system_device_t *);
            uint32_t index = (uint32_t)arg1;
            void *info_out = (void *)(uintptr_t)arg2;
            if (!user_buffer_ok(info_out, sizeof(system_device_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_device_info_buffer");
                break;
            }
            system_device_t info = {0};
            os_status_t status = sysinfo_get_device_info(index, &info);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_device_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_GET_GRAPHICS_INFO: {
            extern os_status_t sysinfo_get_graphics_info(system_graphics_info_t *);
            void *info_out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(info_out, sizeof(system_graphics_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_graphics_info_buffer");
                break;
            }
            system_graphics_info_t info = {0};
            os_status_t status = sysinfo_get_graphics_info(&info);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_graphics_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_GET_ARCH_INFO: {
            extern os_status_t sysinfo_get_arch_info(system_arch_info_t *);
            void *info_out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(info_out, sizeof(system_arch_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_arch_info_buffer");
                break;
            }
            system_arch_info_t info = {0};
            os_status_t status = sysinfo_get_arch_info(&info);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_arch_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_GET_SYSTEM_INFO: {
            extern os_status_t sysinfo_get_system_info(system_info_t *);
            void *info_out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(info_out, sizeof(system_info_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_system_info_buffer");
                break;
            }
            system_info_t info = {0};
            os_status_t status = sysinfo_get_system_info(&info);
            if (status == OS_STATUS_OK &&
                copy_to_user_trusted(info_out, &info, sizeof(info)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_system_info_buffer");
                break;
            }
            set_syscall_status(saved_rsp, status);
            break;
        }

        case SYSCALL_GET_CPU_USAGE: {
            system_cpu_usage_t *usage_out =
                (system_cpu_usage_t *)(uintptr_t)arg1;
            if (!user_buffer_ok(usage_out, sizeof(system_cpu_usage_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_cpu_usage_buffer");
                break;
            }
            system_cpu_usage_t usage = {0};
            uint64_t now_ns = timer_monotonic_ns();
            usage.cpu_count = smp_get_cpu_count();
            if (usage.cpu_count == 0) usage.cpu_count = 1;
            usage.timestamp_ns = now_ns;
            usage.wall_ns = now_ns;
            for (uint32_t i = 0; i < usage.cpu_count && i < process_scheduler_max_cpus(); i++) {
                usage.idle_ns[i] = process_scheduler_get_idle_ns(i);
            }
            if (copy_to_user_trusted(usage_out, &usage, sizeof(usage)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT,
                             "invalid_cpu_usage_buffer");
                break;
            }
            set_syscall_status(saved_rsp, OS_STATUS_OK);
            break;
        }

        case SYSCALL_SET_PROCESS_PRIORITY: {
            int32_t target_pid = (int32_t)arg1;
            uint32_t prio = (uint32_t)arg2;
            if (prio > 3) {
                syscall_fail(saved_rsp, num, OS_STATUS_INVALID_ARG,
                             "invalid_priority");
                break;
            }
            if (target_pid == 0) {
                target_pid = current_pid_get();
            }
            if (process_set_priority(target_pid, (uint8_t)prio) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_INVALID_ARG,
                             "invalid_pid");
                break;
            }
            set_syscall_status(saved_rsp, OS_STATUS_OK);
            break;
        }

        case SYSCALL_GET_PROC_DEBUG_INFO: {
            int32_t query_pid = (int32_t)arg1;
            void *info_out = (void *)(uintptr_t)arg2;
            if (!user_buffer_ok(info_out, 112)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_debug_info_buffer");
                break;
            }
            int32_t rc = process_get_debug_info(query_pid, info_out);
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_GET_OS_DEBUG_INFO: {
            void *info_out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(info_out, 40)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_os_debug_buffer");
                break;
            }
            int32_t rc = process_get_os_debug(info_out);
            set_syscall_i32(saved_rsp, rc);
            break;
        }

        case SYSCALL_READ_KERNEL_LOG: {
            /* Copy the tail of the kernel serial log into a userland buffer so a
             * userland viewer can show boot / driver / Linux-compat output
             * without a serial cable. arg1 = user buffer, arg2 = its size.
             * Returns the number of bytes written (excluding the NUL), which is
             * <= min(arg2 - 1, live log length). */
            void *ubuf = (void *)(uintptr_t)arg1;
            uint64_t ucap = arg2;
            if (ucap == 0u || !user_buffer_ok(ubuf, ucap)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_klog_buffer");
                break;
            }

            static char klog_stage[65536];
            static spinlock_t klog_stage_lock;
            uint32_t want = (ucap > sizeof(klog_stage)) ? (uint32_t)sizeof(klog_stage)
                                                        : (uint32_t)ucap;

            uint64_t f = irq_save_disable();
            spinlock_lock(&klog_stage_lock);
            uint32_t got = serial_copy_log(klog_stage, want);
            spinlock_unlock(&klog_stage_lock);
            irq_restore(f);

            uint64_t copy_bytes = (uint64_t)got + 1u; /* include NUL */
            if (copy_bytes > ucap) {
                copy_bytes = ucap;
            }
            if (copy_to_user(ubuf, klog_stage, copy_bytes) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_klog_buffer");
                break;
            }
            set_syscall_i32(saved_rsp, (int32_t)got);
            break;
        }

        case SYSCALL_WIFI_SCAN_START: {
            bool ok = driver_manager_nic_wifi_scan_start();
            set_syscall_result(saved_rsp, ok ? 1u : 0u);
            break;
        }

        case SYSCALL_WIFI_GET_SCAN_RESULTS: {
            void *out = (void *)(uintptr_t)arg1;
            uint32_t max_count = (uint32_t)arg2;
            /* Bounded so one call can't ask the kernel to stack-allocate an
             * unbounded copy buffer below. */
            if (max_count > 64u) {
                max_count = 64u;
            }
            uint64_t bytes = (uint64_t)max_count * sizeof(driver_wifi_scan_result_t);
            if (max_count != 0u && !user_buffer_ok(out, bytes)) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_wifi_scan_buffer");
                break;
            }
            driver_wifi_scan_result_t results[64];
            uint32_t count = driver_manager_nic_wifi_get_scan_results(results, max_count);
            if (count > max_count) {
                count = max_count;
            }
            if (count != 0u &&
                copy_to_user(out, results, (uint64_t)count * sizeof(driver_wifi_scan_result_t)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_wifi_scan_buffer");
                break;
            }
            set_syscall_result(saved_rsp, (uint64_t)count);
            break;
        }

        case SYSCALL_WIFI_CONNECT: {
            char ssid[DRIVER_WIFI_SSID_MAX + 1u];
            char psk[64];
            if (copy_user_cstring(ssid, sizeof(ssid),
                                  (const char *)(uintptr_t)arg1) < 0) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_wifi_ssid");
                break;
            }
            /* arg2 == 0 means "open network, no password" -- mirrors
             * ax900_connect(ssid, NULL). */
            const char *psk_ptr = NULL;
            if (arg2 != 0u) {
                if (copy_user_cstring(psk, sizeof(psk),
                                      (const char *)(uintptr_t)arg2) < 0) {
                    syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_wifi_psk");
                    break;
                }
                psk_ptr = psk;
            }
            bool ok = driver_manager_nic_wifi_connect(ssid, psk_ptr);
            set_syscall_result(saved_rsp, ok ? 1u : 0u);
            break;
        }

        case SYSCALL_WIFI_DISCONNECT: {
            driver_manager_nic_wifi_disconnect();
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_WIFI_GET_STATUS: {
            void *out = (void *)(uintptr_t)arg1;
            if (!user_buffer_ok(out, sizeof(driver_wifi_status_t))) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_wifi_status_buffer");
                break;
            }
            driver_wifi_status_t status;
            driver_manager_nic_wifi_get_status(&status);
            if (copy_to_user(out, &status, sizeof(status)) != 0u) {
                syscall_fail(saved_rsp, num, OS_STATUS_FAULT, "invalid_wifi_status_buffer");
                break;
            }
            set_syscall_result(saved_rsp, 0);
            break;
        }

        case SYSCALL_NET_GET_DHCP_DNS: {
            set_syscall_result(saved_rsp, (uint64_t)dhcp_get_dns_server());
            break;
        }

        default:
            syscall_fail(saved_rsp, num, OS_STATUS_NOT_SUPPORTED, "unknown_syscall");
            break;
    }

pre_schedule:
    syscall_arch_user_access_end();
    syscall_arch_disable_interrupts();

    if (process_timeslice_expired()) {
        request_switch |= PROCESS_SCHEDULE_REQUEST_SWITCH |
                          PROCESS_SCHEDULE_REQUEST_INVOLUNTARY;
    }

    {
        uint64_t current_user_rsp = syscall_get_user_rsp();
        uint64_t next_user_rsp = current_user_rsp;
        int32_t scheduled_from_pid = current_pid_get();
        uint64_t next_saved_rsp = process_schedule_on_syscall(saved_rsp,
                                                              current_user_rsp,
                                                              request_switch,
                                                              &next_user_rsp);
        if (!request_switch && current_pid_get() == scheduled_from_pid) {
            next_saved_rsp = saved_rsp;
            next_user_rsp = current_user_rsp;
        }

        syscall_set_user_rsp(next_user_rsp);

        return next_saved_rsp;
    }
}
