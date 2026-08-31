#pragma once
#include <stdbool.h>
#include "compat_types.h"

/* Registration table for kernel-side syscall-ABI compat layers -- replaces
 * the single hardcoded `if (abi == PROCESS_ABI_LINUX)
 * linux_syscall_dispatch(...)` that used to live directly in
 * Syscall_Dispatch.c (Docs/Others/TODO_OS_Refactor.md phase P6). Adding a
 * second compat layer (e.g. a hypothetical future PROCESS_ABI_BSD) means
 * writing its dispatcher and calling compat_registry_register() for it
 * once at boot -- Syscall_Dispatch.c itself does not need to change. */

void compat_registry_init(void);
bool compat_registry_register(const compat_layer_t *layer);
const compat_layer_t *compat_registry_find(process_abi_t abi);
