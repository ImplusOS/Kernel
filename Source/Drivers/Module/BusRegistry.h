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
