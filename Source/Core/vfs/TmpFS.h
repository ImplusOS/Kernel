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

/* mmap(MAP_SHARED) of a tmpfs file: the shared-memory object (Core/memory/
 * SharedMemory.c) holding the file's bytes, created on first use and used by
 * read/write/truncate from then on, so every mapping and every fd see the
 * same pages. Returns a handle > 0, or <= 0 when `file` is not a tmpfs file
 * (or is empty, or too large) and the caller should use its normal path. */
int32_t tmpfs_share_mapping(vfs_file_t *file, uint64_t length);
