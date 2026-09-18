#pragma once

#include "DriverBinary.h"

#include <stdbool.h>

/*
 * BusRegistry -- matches a bus-enumerated device against every loaded
 * device driver module's declarative bus_matches[] table
 * (driver_module_descriptor_t, DriverBinary.h) and calls the winner's
 * probe(). Bus drivers (PCI_Main.c, USB_Main.c) call
 * bus_registry_report_device() once per device/interface they find instead
 * of hand-rolling VID/class dispatch themselves. Both bus drivers and the
 * device drivers it dispatches to are separately loaded driver modules, so
 * this lives in the kernel (reachable by all of them via
 * driver_binary_t.bus) rather than as a direct function call between two
 * modules that generally cannot see each other.
 */

void bus_registry_init(void);

/* Returns true if some loaded device driver's probe() claimed the device. A
 * false return with no loaded-module match is the hook point for dynamic
 * loading (driver_module_manager_load_from_vfs(), DriverModule.c). */
bool bus_registry_report_device(const bus_device_t *dev);

void bus_registry_report_device_removed(const bus_device_t *dev);

/* Re-runs matching for every device reported before the VFS was mounted.
 *
 * Bus enumeration happens in the driver_module_critical boot phase, which is
 * ahead of disk_io_init/fs_init (Kernel/Core/kernel_main.c) -- so the
 * on-demand half of bus_registry_report_device(), which has to read
 * DriverDB.txt off the boot medium, cannot work yet and every device needing
 * a not-yet-loaded module is dropped. The hotplug poll does not cover for
 * this either: it only enumerates ports that are *not* already connected, so
 * a device plugged in at boot is never reported a second time.
 *
 * Call once after the filesystems are up. Devices claimed here are forgotten;
 * anything still unclaimed is dropped, and from this point on report_device()
 * does its own manifest lookup inline (the VFS is live) and remembers
 * nothing. */
void bus_registry_retry_unclaimed(void);
