#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "DriverBinary.h"
#include "DriverManager.h"
#include "kernel/boot_info.h"

void driver_module_manager_init(const BOOT_INFO *boot_info);
bool driver_module_manager_register_from_memory(const char *name, const void *data, uint32_t size);
const driver_binary_t *driver_module_manager_kernel_api(void);

/* Read-only iteration over prepared (driver_module_init() already ran)
 * modules' descriptors, for BusRegistry.c to walk bus_matches[] without
 * needing direct access to g_modules[]. Skips modules that aren't prepared
 * yet (descriptor == NULL) -- see driver_module_prepare(). */
uint32_t driver_module_manager_count(void);
const driver_module_descriptor_t *driver_module_manager_descriptor_at(uint32_t index);

/* Post-boot dynamic load of a driver .ELF from a VFS path -- see the
 * implementation comment in DriverModule.c. */
bool driver_module_manager_load_from_vfs(const char *path);
void *driver_module_manager_get_driver(const char *name);
bool driver_module_init_all(void);
bool driver_module_init_critical(void);
bool driver_module_init_deferred(void);
bool driver_module_manager_unload_by_name(const char *name);
bool driver_module_manager_reload_by_name(const char *name);
