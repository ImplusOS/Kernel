#pragma once

/*
 * ProcFS - minimal /proc pseudo-filesystem.
 *
 * Provides the handful of /proc entries that glibc, V8/PartitionAlloc,
 * Breakpad and base::SysInfo poke at: /proc/self/{maps,status,stat,
 * cmdline}, /proc/meminfo, /proc/cpuinfo, /proc/stat, /proc/version,
 * /proc/sys/kernel/random/boot_id, /proc/sys/vm/overcommit_memory.
 *
 * Content is generated once, at open() time, into a heap buffer (there is
 * no live "seek and re-read reflects new state" semantics like real Linux
 * procfs - each fresh open() sees current state though). Only the calling
 * process's own view ("self" or its own numeric pid) is supported; a
 * /proc/<other-pid>/... path resolves as not-found. See
 * TODO_Chromium_LinuxABI.md section 3.3.
 */

#include "kernel/interfaces/vfs_types.h"

void procfs_init(void);
const vfs_driver_t *procfs_vfs_get_driver(void);

/* Resolves /proc/self/exe and /proc/self/fd/<n> style paths to a
 * synthetic readlink() target. Returns 0 on success (writing a NUL
 * terminated string into `out`), <0 if `path` is not a procfs symlink. */
int procfs_readlink(const char *path, char *out, uint32_t capacity);
