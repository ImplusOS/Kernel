#pragma once
#include <stdint.h>
#include "Compat/compat_types.h"

/* Linux syscall-number-ABI compat layer. See Kernel/Compat/compat_types.h
 * and compat_registry.h for how this plugs into Syscall_Dispatch.c
 * (Docs/Others/TODO_OS_Refactor.md phase P6) -- callers should go through
 * compat_registry_find(PROCESS_ABI_LINUX) rather than calling
 * linux_syscall_dispatch() directly. */

uint64_t linux_syscall_dispatch(uint64_t saved_rsp,
                                uint64_t num,
                                uint64_t arg1,
                                uint64_t arg2,
                                uint64_t arg3,
                                uint64_t arg4,
                                uint64_t arg5,
                                uint64_t arg6);

/* Registers this layer into Kernel/Compat's compat_registry -- called once
 * from Syscall_Init.c's syscall_init(). */
void linux_compat_layer_register(void);

/* Drop (without flushing - the address space is going away) any registered
 * file-backed MAP_SHARED write-back mappings owned by a terminating process.
 * Called from the process exit path. */
void linux_compat_mshared_release_pid(int32_t pid);
