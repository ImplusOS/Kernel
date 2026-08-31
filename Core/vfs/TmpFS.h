#pragma once

/*
 * TmpFS - minimal in-memory filesystem mounted at /dev/shm.
 *
 * Backs POSIX shm_open()/File-backed anonymous shared memory for
 * userland that does not have (or does not want to rely on)
 * memfd_create(). See TODO_Chromium_LinuxABI.md section 3.3.
 *
 * Content lives entirely in kernel heap memory (malloc/realloc) and is
 * lost on unmount/reboot; there is no persistence to a real disk.
 */

#include "kernel/interfaces/vfs_types.h"

void tmpfs_init(void);
const vfs_driver_t *tmpfs_vfs_get_driver(void);
