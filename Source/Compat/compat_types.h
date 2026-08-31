#pragma once
#include <stdint.h>

/*
 * Kernel-side syscall-number-ABI compatibility layer -- NOT to be confused
 * with Userland/POSIX/ (a userland libc-level compat shim linked into
 * processes; see Userland/POSIX/README_POSIX.md). This one lives entirely
 * in kernel space and decides, per-process, which syscall-number table and
 * argument-marshalling convention Syscall_Dispatch.c's SYSCALL entry point
 * uses -- see Docs/Architecture/Compat_Layers.md (phase P7) for the full
 * writeup of the distinction between the two.
 *
 * Added under Docs/Others/TODO_OS_Refactor.md phase P6. process_abi_t
 * mirrors the PROCESS_ABI_IMPLUS/PROCESS_ABI_LINUX values
 * Kernel/Core/process/ProcessScheduler.h already defines as plain
 * `#define`s over a bare uint8_t (left as-is there deliberately -- the
 * process/scheduler subsystem is used far too pervasively to safely
 * introduce a new named type into it as part of this phase); this is
 * Kernel/Compat's own typed view of the same values for use in
 * compat_layer_t below.
 */
typedef uint8_t process_abi_t;

/* One compat layer's syscall dispatcher: given the raw saved user register
 * frame pointer, syscall number, and up to 6 arguments (the AMD64
 * SYSCALL/SYSRET ABI's argument-register slots -- see
 * Kernel/Core/syscall/Syscall_Main.h), handles the call and returns the
 * same request_switch bitmask Syscall_Dispatch.c's native switch produces.
 * Matches the real linux_syscall_dispatch() signature (Kernel/Compat/Linux/
 * Syscall_LinuxCompat.h) rather than an idealized generic shape, since
 * that is the one compat layer that actually exists today. */
typedef uint64_t (*compat_dispatch_fn_t)(uint64_t saved_rsp, uint64_t num,
                                         uint64_t arg1, uint64_t arg2,
                                         uint64_t arg3, uint64_t arg4,
                                         uint64_t arg5, uint64_t arg6);

typedef struct {
    process_abi_t abi;
    const char *name;
    compat_dispatch_fn_t dispatch;
} compat_layer_t;
