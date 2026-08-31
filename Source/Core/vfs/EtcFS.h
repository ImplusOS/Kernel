#pragma once

/*
 * EtcFS - minimal /etc pseudo-filesystem.
 *
 * Generates the handful of /etc files a Linux-ABI userland's own
 * resolver reads directly (Chromium included - it does not use the
 * kernel's DNS at all, it opens /etc/resolv.conf itself): /etc/hosts,
 * /etc/resolv.conf. See TODO_Chromium_LinuxABI.md section 3.10.
 *
 * Content reflects the compile-time network config in kernel/config.h
 * (OS_CONFIG_NET_IPV4_*) and is generated once per open(), same
 * simplification as ProcFS/DevFS.
 */

#include "kernel/interfaces/vfs_types.h"

void etcfs_init(void);
const vfs_driver_t *etcfs_vfs_get_driver(void);
