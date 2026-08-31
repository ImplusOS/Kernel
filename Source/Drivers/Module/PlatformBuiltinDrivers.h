#pragma once

/* Registers ACPI/Timer/LAPIC/IOAPIC (x86_64) or ACPI/Timer/GIC (arm64) as
 * DEVICE_TYPE_PLATFORM entries in DeviceRegistry, so they appear in driver
 * listings (OSDebug, /proc/bus/devices, ...) the same way a loadable driver
 * module does, without actually being one -- see Docs/Others/
 * TODO_OS_Refactor.md phase P5, 9.1 ("Plan A": built-in drivers).
 *
 * Call once, after driver_module_manager_init() has run (DeviceRegistry
 * does not exist before that -- see PlatformBuiltinDrivers.c's top comment
 * for why) and after the platform subsystems it describes have already
 * been initialized by kernel_main.c's normal boot sequence. This function
 * only adds registry visibility; it never calls acpi_init()/timer_init()/
 * lapic_init()/ioapic_init() itself and does not change when or whether
 * those run. */
void platform_builtin_drivers_register(void);
