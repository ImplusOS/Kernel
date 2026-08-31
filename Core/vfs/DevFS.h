#pragma once

/*
 * DevFS - minimal /dev pseudo-filesystem.
 *
 * Provides the handful of character device nodes that a Linux-ABI
 * userland (glibc, Chromium) expects to be able to open() directly:
 * /dev/null, /dev/zero, /dev/full, /dev/urandom, /dev/random, /dev/tty.
 *
 * Mounted at prefix "/dev" via vfs_mount() so ordinary open()/read()/
 * write()/stat() go through the regular VFS path. See TODO_Chromium_LinuxABI.md
 * section 3.3.
 */

#include "kernel/interfaces/vfs_types.h"

void devfs_init(void);
const vfs_driver_t *devfs_vfs_get_driver(void);

/* True if `path` names one of the fixed /dev character-device nodes
 * (used by stat()/fstat() in the Linux ABI layer to report S_IFCHR
 * instead of the generic S_IFREG mode). */
bool devfs_path_is_device(const char *path);
