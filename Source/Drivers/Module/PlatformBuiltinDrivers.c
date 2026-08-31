#include "PlatformBuiltinDrivers.h"
#include "DriverManager.h"
#include "kernel/interfaces/device.h"
#include "Platform/rtc/RTC.h"

#ifdef PLATFORM_X86_64
#include "Platform/interrupt/LAPIC.h"
#include "Platform/interrupt/IOAPIC.h"
#elif defined(PLATFORM_ARM64)
#include "Arch/arm64/interrupt/GIC.h"
#endif

/*
 * "Built-in driver" registration for ACPI/Timer/LAPIC/IOAPIC (x86_64) or
 * ACPI/Timer/GIC (arm64) -- Docs/Others/TODO_OS_Refactor.md phase P5, 9.1,
 * decision #3 ("Plan A"). These subsystems must initialize before
 * driver_module_manager_init() itself can run (it is what brings up
 * DeviceRegistry/BusRegistry and loads .ELF driver modules -- see that
 * function in Kernel/Drivers/Module/DriverModule.c and the boot order in
 * Kernel/Core/kernel_main.c's boot_profile phases), so they cannot be
 * loadable modules without a real chicken-and-egg problem. Plan A resolves
 * this by keeping them exactly as statically-linked, directly-called code
 * (acpi_init()/timer_init()/platform_interrupts_configure() are called by
 * kernel_main.c precisely as before this file existed, at precisely the
 * same point in boot) and merely registering a DeviceRegistry entry for
 * each *afterward*, once DeviceRegistry exists, so driver listings
 * (OSDebug, a future /proc/bus/devices) show them alongside real loaded
 * modules instead of leaving them invisible.
 *
 * platform_builtin_drivers_register() therefore does not (and must not)
 * call any of those subsystems' init functions -- only kernel_main.c does,
 * at its normal boot-sequence points, unchanged.
 */

typedef struct {
    const char *description;
} platform_builtin_driver_ops_t;

/* RTC gets a genuine callable vtable (not just a description) so a registry
 * consumer can read the wall clock without linking Platform/rtc directly. */
typedef struct {
    const char *description;
    void      (*read_time)(rtc_time_t *out);
} platform_rtc_ops_t;

static const platform_builtin_driver_ops_t g_acpi_ops = {
    "ACPI (RSDP/MADT discovery, SMP + interrupt routing input)"
};
static const platform_builtin_driver_ops_t g_timer_ops = {
    "System timer (see arch_ops_t.get_timer_hal for the per-arch backend)"
};
static const platform_rtc_ops_t g_rtc_ops = {
    "CMOS real-time clock (Platform/rtc)", rtc_read_time
};
#ifdef PLATFORM_X86_64
static const platform_builtin_driver_ops_t g_lapic_ops = { "Local APIC" };
static const platform_builtin_driver_ops_t g_ioapic_ops = { "I/O APIC" };
#elif defined(PLATFORM_ARM64)
static const platform_builtin_driver_ops_t g_gic_ops = {
    "Generic Interrupt Controller v3"
};
#endif

void platform_builtin_drivers_register(void)
{
    (void)driver_manager_attach("ACPI", DEVICE_TYPE_PLATFORM, &g_acpi_ops);
    (void)driver_manager_attach("Timer", DEVICE_TYPE_PLATFORM, &g_timer_ops);
    (void)driver_manager_attach("RTC", DEVICE_TYPE_PLATFORM, &g_rtc_ops);
#ifdef PLATFORM_X86_64
    if (lapic_is_present()) {
        (void)driver_manager_attach("LAPIC", DEVICE_TYPE_PLATFORM, &g_lapic_ops);
    }
    if (ioapic_is_present()) {
        (void)driver_manager_attach("IOAPIC", DEVICE_TYPE_PLATFORM, &g_ioapic_ops);
    }
#elif defined(PLATFORM_ARM64)
    (void)driver_manager_attach("GIC", DEVICE_TYPE_PLATFORM, &g_gic_ops);
#endif
}
