#pragma once

/*
 * SysFS - minimal /sys pseudo-filesystem.
 *
 * ImplusOS used to have no /sys at all, which is invisible right up until a
 * Linux-ABI program asks libdrm what a DRM node is. stat() on /dev/dri/card0
 * only ever yields a major/minor pair; from those two numbers libdrm rebuilds
 *
 *   /sys/dev/char/<maj>:<min>/uevent               DEVNAME=dri/card0
 *   /sys/dev/char/<maj>:<min>/device               realpath'd -> the PCI dir
 *   /sys/dev/char/<maj>:<min>/device/uevent        PCI_SLOT_NAME=0000:00:02.0
 *   /sys/dev/char/<maj>:<min>/device/{vendor,device,
 *                                      subsystem_vendor,subsystem_device,
 *                                      revision}
 *   /sys/dev/char/<maj>:<min>/device/subsystem     readlink -> ".../pci"
 *   /sys/dev/char/<maj>:<min>/device/drm           stat + readdir
 *
 * and refuses to produce a drmDevice until every one of them answers.
 * Mesa's loader_is_device_render_capable() is exactly one drmGetDevice2()
 * call, so without this filesystem glamor stopped at
 * "DRI2: failed to get compatible render device" and no GPU process ever
 * started. See Docs/Others/TODO_Chromium_LinuxABI.md.
 *
 * Table-driven rather than synthesised on demand: one GPU, two nodes, a fixed
 * shape, and every lookup is a plain strcmp -- which is all stat(), readdir()
 * and realpath() do with it.
 */

#include "kernel/interfaces/vfs_types.h"

void sysfs_init(void);
const vfs_driver_t *sysfs_vfs_get_driver(void);

/* Publish /sys/dev/char/226:{0,128} plus the PCI device both nodes hang off.
 * The minor numbers come from DevFS.h -- they are the same ones stat() reports
 * in st_rdev, and libdrm joins the two sides by formatting them back into a
 * path, so they must agree. Idempotent: vfs_mount_pseudo_filesystems() can run
 * twice during boot. Called from drm_kms_init(), after the PCI bus has been
 * enumerated (driver_module_critical runs well before fs_init). */
void sysfs_publish_drm_nodes(void);
