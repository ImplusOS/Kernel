#pragma once
#include <stdint.h>

#define SYSCALL_SERIAL_PUTCHAR    1
#define SYSCALL_SERIAL_PUTS       2
#define SYSCALL_SERIAL_WRITE_U64  3
#define SYSCALL_SERIAL_WRITE_U32  4
#define SYSCALL_SERIAL_WRITE_U16  5
#define SYSCALL_PROCESS_CREATE    6
#define SYSCALL_PROCESS_YIELD     7
#define SYSCALL_PROCESS_EXIT      8
#define SYSCALL_THREAD_CREATE     9
#define SYSCALL_THREAD_EXIT       10
#define SYSCALL_THREAD_JOIN       11
#define SYSCALL_THREAD_DETACH     12
#define SYSCALL_FILE_OPEN          23
#define SYSCALL_FILE_READ         24
#define SYSCALL_FILE_WRITE        25
#define SYSCALL_FILE_CLOSE        26
#define SYSCALL_FILE_SEEK         34
#define SYSCALL_USER_MALLOC       27
#define SYSCALL_USER_FREE         28
#define SYSCALL_USER_MEMCPY       29
#define SYSCALL_USER_MEMCMP       30
#define SYSCALL_USER_MEMSET       31
#define SYSCALL_INPUT_READ_KEYBOARD 32
#define SYSCALL_INPUT_READ_MOUSE  33
#define SYSCALL_PROCESS_SPAWN_ELF 36
#define SYSCALL_FILE_MKDIR        37
#define SYSCALL_FILE_OPENDIR      38
#define SYSCALL_FILE_READDIR      39
#define SYSCALL_FILE_CLOSEDIR     40
#define SYSCALL_FILE_UNLINK       41
#define SYSCALL_FILE_CREAT        42
#define SYSCALL_USER_MMAP         43
#define SYSCALL_PROCESS_SIGNAL    44
#define SYSCALL_IPC_SEND_MESSAGE  45
#define SYSCALL_IPC_RECEIVE_MESSAGE 46
#define SYSCALL_PROCESS_GET_PID   47
#define SYSCALL_GET_DISPLAY_WIDTH  48
#define SYSCALL_GET_DISPLAY_HEIGHT 49
#define SYSCALL_WINDOW_REGISTER_SERVICE 50
#define SYSCALL_WINDOW_GET_WM_PID 51
#define SYSCALL_DISPLAY_DRAW_PIXEL 52
#define SYSCALL_DISPLAY_FILL_RECT  53
#define SYSCALL_DISPLAY_PRESENT    54
#define SYSCALL_GET_DISPLAY_FRAMEBUFFER 55
#define SYSCALL_DISPLAY_GET_PIXEL  56
#define SYSCALL_DISPLAY_GET_TOPOLOGY 57
#define SYSCALL_DISPLAY_GET_MONITOR_INFO 58
#define SYSCALL_DISPLAY_GET_MONITOR_MODE_INFO 59
#define SYSCALL_DISPLAY_SET_MONITOR_MODE 60
#define SYSCALL_DISPLAY_PRESENT_RECTS 61
/* 62 (SYSCALL_WINDOW_GET_POINTER_STATE) retired: absolute pointer position
 * and screen clamping are now owned entirely by the userland WindowManager
 * (see Userland/Application/com.ImplusOS.windowmanager/Input/WM_Input.c),
 * which derives them from raw SYSCALL_INPUT_READ_MOUSE deltas. */
#define SYSCALL_SYSTEM_SHUTDOWN    250
#define SYSCALL_SYSTEM_REBOOT      251
#define SYSCALL_SYSTEM_SHUTDOWN_BROADCAST 252
#define SYSCALL_UDP_SEND         100
#define SYSCALL_TCP_CONNECT      101
#define SYSCALL_TCP_LISTEN       102
#define SYSCALL_TCP_ACCEPT       103
#define SYSCALL_TCP_SEND         104
#define SYSCALL_TCP_RECV         105
#define SYSCALL_TCP_CLOSE        106
#define SYSCALL_TCP_GET_STATE    107

#define SYSCALL_PROCESS_WAITPID   110
#define SYSCALL_PROCESS_GETPPID   111
#define SYSCALL_PROCESS_EXIT_STATUS 112
#define SYSCALL_SLEEP             113
#define SYSCALL_FILE_STAT         114
#define SYSCALL_FILE_PIPE         115
#define SYSCALL_FILE_DUP          116
#define SYSCALL_FILE_DUP2         117
#define SYSCALL_GETCWD            118
#define SYSCALL_GET_UPTIME_MS     119
#define SYSCALL_NANOSLEEP         120
#define SYSCALL_GET_PROC_COUNT    121
#define SYSCALL_GET_PROC_INFO     122
#define SYSCALL_PROCESS_SPAWN_ELF_ARG 123
#define SYSCALL_PROCESS_GET_LAUNCH_ARG 124
#define SYSCALL_GET_PROC_PERF_INFO 125
#define SYSCALL_GET_BOOT_PROFILE_COUNT 126
#define SYSCALL_GET_BOOT_PROFILE_ENTRY 127
#define SYSCALL_GET_RTC_TIME      140

#define SYSCALL_GET_TOTAL_MEMORY  253
#define SYSCALL_GET_USED_MEMORY   254

#define SYSCALL_MPROTECT          150
#define SYSCALL_MUNMAP            151
#define SYSCALL_MREMAP            152

#define SYSCALL_GETUID            154
#define SYSCALL_GETEUID           155
#define SYSCALL_GETGID            156
#define SYSCALL_GETEGID           157
#define SYSCALL_GETTID            158
#define SYSCALL_SET_TID_ADDRESS   159

#define SYSCALL_EPOLL_CREATE      160
#define SYSCALL_EPOLL_CTL         161
#define SYSCALL_EPOLL_WAIT        162
#define SYSCALL_EVENTFD           163

#define SYSCALL_CLOCK_GETTIME     164
#define SYSCALL_CLOCK_GETRES      165

#define SYSCALL_ARCH_PRCTL        168
#define SYSCALL_PRLIMIT64         169
#define SYSCALL_GETRANDOM         170
#define SYSCALL_READV             171
#define SYSCALL_WRITEV            172
#define SYSCALL_FTRUNCATE         173
#define SYSCALL_FCHMOD            174
#define SYSCALL_RENAME            175
#define SYSCALL_IOCTL_EX          176
#define SYSCALL_FCNTL_EX          177
#define SYSCALL_ACCESS            178
#define SYSCALL_FD_POLL           179

#define SYSCALL_FUTEX             180
#define SYSCALL_CLONE             181

#define SYSCALL_RT_SIGACTION      182
#define SYSCALL_RT_SIGPROCMASK    183
#define SYSCALL_RT_SIGRETURN      184
#define SYSCALL_SIGALTSTACK       185
#define SYSCALL_TKILL             186
#define SYSCALL_SHM_CREATE        187
#define SYSCALL_SHM_GRANT         188
#define SYSCALL_SHM_MAP           189

#define SYSCALL_FORK              190
#define SYSCALL_EXECVE            191
#define SYSCALL_VFORK             192
#define SYSCALL_SET_ROBUST_LIST   193
#define SYSCALL_SHM_UNMAP         194
#define SYSCALL_SHM_CLOSE         195
#define SYSCALL_GET_MAIN_IMAGE_INFO 196

#define SYSCALL_GET_CPU_INFO       200
#define SYSCALL_GET_MEMORY_INFO   201
#define SYSCALL_GET_VMEM_INFO     202
#define SYSCALL_GET_DISK_INFO     203
#define SYSCALL_GET_DEVICE_INFO   204
#define SYSCALL_GET_GRAPHICS_INFO 205
#define SYSCALL_GET_ARCH_INFO     206
#define SYSCALL_GET_SYSTEM_INFO   207

#define SYSCALL_DRM_OPEN          208
#define SYSCALL_DRM_IOCTL         209
#define SYSCALL_DRM_CLOSE         210
#define SYSCALL_DRM_MMAP          211

#define SYSCALL_EVDEV_OPEN        212
#define SYSCALL_EVDEV_READ        213
#define SYSCALL_EVDEV_IOCTL       214
#define SYSCALL_EVDEV_CLOSE       215
#define SYSCALL_GET_DISK_COUNT    216
#define SYSCALL_RAW_BLOCK_READ    217
#define SYSCALL_RAW_BLOCK_WRITE   218
#define SYSCALL_GET_BOOT_FONT     219

#define SYSCALL_UNIX_SOCKET       220
#define SYSCALL_UNIX_BIND         221
#define SYSCALL_UNIX_LISTEN       222
#define SYSCALL_UNIX_ACCEPT       223
#define SYSCALL_UNIX_CONNECT      224
#define SYSCALL_UNIX_SEND         225
#define SYSCALL_UNIX_RECV         226
#define SYSCALL_UNIX_SENDMSG      227
#define SYSCALL_UNIX_RECVMSG      228
#define SYSCALL_UNIX_CLOSE        229

#define SYSCALL_AUDIO_OPEN        230
#define SYSCALL_AUDIO_GET_INFO    231
#define SYSCALL_AUDIO_WRITE       232
#define SYSCALL_AUDIO_DRAIN       233
#define SYSCALL_AUDIO_CLOSE       234

#define SYSCALL_GET_CPU_USAGE     235
#define SYSCALL_SET_PROCESS_PRIORITY 236

#define SYSCALL_GET_PROC_DEBUG_INFO  260
#define SYSCALL_GET_OS_DEBUG_INFO    261

#define SYSCALL_WIFI_SCAN_START        262
#define SYSCALL_WIFI_GET_SCAN_RESULTS  263
#define SYSCALL_WIFI_CONNECT           264
#define SYSCALL_WIFI_DISCONNECT        265
#define SYSCALL_WIFI_GET_STATUS        266
#define SYSCALL_NET_GET_DHCP_DNS       267
#define SYSCALL_READ_KERNEL_LOG        268
/* Given a memfd fd, return the shared-memory handle backing it (or <0).
 * Lets a native process map a memfd received via SCM_RIGHTS with
 * os_shared_memory_map(). */
#define SYSCALL_MEMFD_SHM_HANDLE       269

#define SYSCALL_KVM_OPEN          240
#define SYSCALL_KVM_IOCTL         241
#define SYSCALL_KVM_CLOSE         242
#define SYSCALL_KVM_MMAP          243

#define SYSCALL_SOCKET_CREATE     130
#define SYSCALL_SOCKET_CONNECT    131
#define SYSCALL_SOCKET_BIND       132
#define SYSCALL_SOCKET_LISTEN     133
#define SYSCALL_SOCKET_ACCEPT     134
#define SYSCALL_SOCKET_SEND       135
#define SYSCALL_SOCKET_RECV       136
#define SYSCALL_SOCKET_CLOSE      137
#define SYSCALL_SOCKET_GET_INFO   141
#define SYSCALL_SOCKET_SET_OPTION 142
#define SYSCALL_SOCKET_GET_OPTION 143
#define SYSCALL_SOCKET_SHUTDOWN   144
#define SYSCALL_SOCKET_LISTEN_EX  145

#define SYSCALL_UDP_BIND          108
#define SYSCALL_UDP_UNBIND        109
#define SYSCALL_UDP_RECV          138

#define SYSCALL_FRAME_RAX  0
#define SYSCALL_FRAME_RDX  1
#define SYSCALL_FRAME_RSI  2
#define SYSCALL_FRAME_RDI  3
#define SYSCALL_FRAME_R8   4
#define SYSCALL_FRAME_R9   5
#define SYSCALL_FRAME_R10  6
#define SYSCALL_FRAME_R12  7
#define SYSCALL_FRAME_R13  8
#define SYSCALL_FRAME_R14  9
#define SYSCALL_FRAME_R15  10
#define SYSCALL_FRAME_RBX  11
#define SYSCALL_FRAME_RBP  12
#define SYSCALL_FRAME_RCX  13
#define SYSCALL_FRAME_R11  14
#define SYSCALL_FRAME_QWORDS 15

void     syscall_init(void);
void     syscall_init_per_cpu(void);
uint64_t syscall_get_user_rsp(void);
void     syscall_set_user_rsp(uint64_t user_rsp);
uint64_t syscall_get_kernel_rsp(void);
void     syscall_set_kernel_rsp(uint64_t kernel_rsp);

uint64_t syscall_dispatch(uint64_t saved_rsp,
                          uint64_t num,
                          uint64_t arg1,
                          uint64_t arg2,
                          uint64_t arg3,
                          uint64_t arg4,
                          uint64_t arg5,
                          uint64_t arg6);
