#pragma once

/*
 * VFS_Pseudo -- the VFS layer's own list of in-kernel pseudo filesystems
 * (devfs, tmpfs, procfs, etcfs) and the DRM/KMS shim, plus the fixed mount
 * points they occupy.
 *
 * This used to be an open-coded run of devfs_init()/tmpfs_init()/... calls
 * and vfs_mount("/dev", ...) / vfs_mount("/tmp", ...) literals in
 * Kernel/Core/kernel_main.c's all_fs_initialize(). That put knowledge of
 * every specific pseudo-filesystem implementation -- and its mount path --
 * into the boot orchestrator. The boot code now calls
 * vfs_mount_pseudo_filesystems() and knows none of them; the list lives
 * here, in the VFS subsystem, which is where pseudo-filesystem wiring
 * belongs (cf. Linux keeping rootfs/devtmpfs setup in fs/ + init/do_mounts.c
 * rather than in start_kernel()).
 *
 * Adding a pseudo filesystem: add one row to g_vfs_pseudo_table[] in
 * VFS_Pseudo.c. No change to kernel_main.c.
 */

void vfs_mount_pseudo_filesystems(void);
