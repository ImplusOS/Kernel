#pragma once

#ifndef GDT_KERNEL_CODE
#define GDT_KERNEL_CODE      0x08
#endif
#ifndef GDT_KERNEL_DATA
#define GDT_KERNEL_DATA      0x10
#endif
#ifndef GDT_USER_COMPAT_CODE
#define GDT_USER_COMPAT_CODE 0x18
#endif
#ifndef GDT_USER_DATA
#define GDT_USER_DATA        0x20
#endif
#ifndef GDT_USER_CODE
#define GDT_USER_CODE        0x28
#endif
#ifndef GDT_TSS
#define GDT_TSS              0x30
#endif

/* Copy-on-write fork (TODO_Chromium_LinuxABI.md bucket B).
 *
 * When 1, process_fork() shares the parent's pages into the child read-only
 * and copies them lazily on the first write (paging_cow_clone_user_range /
 * paging_handle_cow_fault), backed by physical-page refcounts in
 * Memory_Main.c. This makes Chromium's zygote-style repeated forks cheap.
 *
 * Default 0: the eager full-copy fork is slower but known-good, whereas COW
 * touches PTE aliasing + SMP TLB coherence + the physical allocator at once
 * and has NOT been validated on real hardware / QEMU in this tree.
 *
 * (Was briefly flipped to 1 on 2026-08-29 for multiprocess Chromium -- see
 * TODO_glibc_Port.md G7 -- and reverted the same day: it made boot unstable
 * (intermittent pre-userland triple-fault reboots, __stack_chk_fail during
 * kernel init). Multiprocess Chromium must instead wait for COW to get a
 * dedicated QEMU boot-regression pass, or run eager-copy with more guest
 * RAM.) Flip to 1 (or -DKERNEL_COW_FORK=1) only after that validation. */
#ifndef KERNEL_COW_FORK
#define KERNEL_COW_FORK 0
#endif

#ifndef OS_CONFIG_PROCESS_MAX_COUNT
#ifdef PROCESS_MAX_COUNT_CONFIG
#define OS_CONFIG_PROCESS_MAX_COUNT PROCESS_MAX_COUNT_CONFIG
#else
#define OS_CONFIG_PROCESS_MAX_COUNT 256
#endif
#endif

#ifndef OS_CONFIG_FILE_MAX_FD
#ifdef FILE_MAX_FD_CONFIG
#define OS_CONFIG_FILE_MAX_FD FILE_MAX_FD_CONFIG
#else
/* NOTE: this is a *global*, system-wide fd table (Syscall_File.c), shared
 * by every process, not a per-process limit. Keep it comfortably below
 * Syscall_Socket.c's SOCKET_FD_BASE (socket fds live in a disjoint numeric
 * range starting there) - see OS_CONFIG_FILE_MAX_FD_MAX below.
 *
 * 192, not 256, because UnixSocket.h's UNIX_SOCK_FD_BASE follows immediately
 * after this table and the X server refuses a client whose fd is >= its
 * lastfdesc -- which is min(RLIMIT_NOFILE-1, MAXSELECT, MAXCLIENTS) and so is
 * pinned at the compile-time MAXCLIENTS of 256 no matter what -maxclients
 * says. With the table at 256 every AF_UNIX fd started at 256, so every X
 * client was accepted and instantly closed. 192 files + 64 AF_UNIX keeps the
 * whole range a client can land in under 256. See
 * Docs/Others/TODO_Doom_Xorg_MethodA.md M22. */
#define OS_CONFIG_FILE_MAX_FD 192
#endif
#endif

#ifndef OS_CONFIG_FILE_MAX_DIR_HANDLE
#ifdef FILE_MAX_DIR_HANDLE_CONFIG
#define OS_CONFIG_FILE_MAX_DIR_HANDLE FILE_MAX_DIR_HANDLE_CONFIG
#else
/* Kept <= OS_CONFIG_FILE_MAX_FD, which dropped to 192 so the AF_UNIX fd
 * range that follows it stays under the X server's 256-fd client limit. */
#define OS_CONFIG_FILE_MAX_DIR_HANDLE 192
#endif
#endif

#ifndef PROCESS_MAX_COUNT_CONFIG
#define PROCESS_MAX_COUNT_CONFIG OS_CONFIG_PROCESS_MAX_COUNT
#endif

#ifndef FILE_MAX_FD_CONFIG
#define FILE_MAX_FD_CONFIG OS_CONFIG_FILE_MAX_FD
#endif

#ifndef FILE_MAX_DIR_HANDLE_CONFIG
#define FILE_MAX_DIR_HANDLE_CONFIG OS_CONFIG_FILE_MAX_DIR_HANDLE
#endif

#define OS_CONFIG_PROCESS_MAX_COUNT_MIN   1
#define OS_CONFIG_PROCESS_MAX_COUNT_MAX   256
#define OS_CONFIG_FILE_MAX_FD_MIN         4
/* Must stay <= Syscall_Socket.c's SOCKET_FD_BASE (disjoint fd numeric
 * range for sockets) and <= Userland/POSIX/include/posix_fdtable.h's
 * POSIX_FD_TABLE_SIZE / posix_io.h's FD_SETSIZE (both 1024, indexed
 * directly by raw fd value with no indirection). */
#define OS_CONFIG_FILE_MAX_FD_MAX         256
#define OS_CONFIG_FILE_MAX_DIR_HANDLE_MIN 4
#define OS_CONFIG_FILE_MAX_DIR_HANDLE_MAX 256

#if (PROCESS_MAX_COUNT_CONFIG < OS_CONFIG_PROCESS_MAX_COUNT_MIN) || \
    (PROCESS_MAX_COUNT_CONFIG > OS_CONFIG_PROCESS_MAX_COUNT_MAX)
#error "PROCESS_MAX_COUNT_CONFIG is out of supported range"
#endif

#if (FILE_MAX_FD_CONFIG < OS_CONFIG_FILE_MAX_FD_MIN) || \
    (FILE_MAX_FD_CONFIG > OS_CONFIG_FILE_MAX_FD_MAX)
#error "FILE_MAX_FD_CONFIG is out of supported range"
#endif

#if (FILE_MAX_DIR_HANDLE_CONFIG < OS_CONFIG_FILE_MAX_DIR_HANDLE_MIN) || \
    (FILE_MAX_DIR_HANDLE_CONFIG > OS_CONFIG_FILE_MAX_DIR_HANDLE_MAX)
#error "FILE_MAX_DIR_HANDLE_CONFIG is out of supported range"
#endif

#if FILE_MAX_DIR_HANDLE_CONFIG > FILE_MAX_FD_CONFIG
#error "FILE_MAX_DIR_HANDLE_CONFIG must be <= FILE_MAX_FD_CONFIG"
#endif

#ifndef OS_CONFIG_SMP_MAX_CPUS
#define OS_CONFIG_SMP_MAX_CPUS 16
#endif

/* Kernel/Drivers/Module/DriverModule.c's g_modules[] table size. Decoupled
 * from BOOT_INFO's MAX_LOADED_FILES (kernel/boot_info.h, 16): that constant
 * bounds how many .ELF files the bootloader can preload before kernel_main
 * even runs, while this one also has to make room for driver modules
 * loaded post-boot (driver_module_manager_load_from_vfs()), so it must be
 * >= MAX_LOADED_FILES. */
#ifndef OS_CONFIG_DRIVER_MODULE_MAX_COUNT
#define OS_CONFIG_DRIVER_MODULE_MAX_COUNT 64
#endif

#ifndef OS_CONFIG_SMP_ENABLED
#define OS_CONFIG_SMP_ENABLED 1
#endif

#ifndef OS_CONFIG_LOG_FILE_MAX_BYTES
#define OS_CONFIG_LOG_FILE_MAX_BYTES (512 * 1024)
#endif

#ifndef OS_CONFIG_BOOT_FADE
#define OS_CONFIG_BOOT_FADE 0
#endif

#ifndef OS_CONFIG_DEBUG_PAGE_FAULT_DUMP
#define OS_CONFIG_DEBUG_PAGE_FAULT_DUMP 0
#endif

#ifndef OS_CONFIG_SIGNAL_HANDLER_MAX_PER_PROCESS
#define OS_CONFIG_SIGNAL_HANDLER_MAX_PER_PROCESS 32
#endif

#ifndef OS_CONFIG_PENDING_SIGNAL_MAX_PER_PROCESS
#define OS_CONFIG_PENDING_SIGNAL_MAX_PER_PROCESS 32
#endif

#ifndef OS_CONFIG_NET_IPV4_ADDR
#define OS_CONFIG_NET_IPV4_ADDR 0x0A00020FULL
#endif

#ifndef OS_CONFIG_NET_IPV4_MASK
#define OS_CONFIG_NET_IPV4_MASK 0xFFFFFF00ULL
#endif

#ifndef OS_CONFIG_NET_IPV4_GATEWAY
#define OS_CONFIG_NET_IPV4_GATEWAY 0x0A000202ULL
#endif
