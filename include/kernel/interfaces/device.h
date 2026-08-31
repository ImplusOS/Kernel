#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum device_type {
    DEVICE_TYPE_UNKNOWN = 0,
    DEVICE_TYPE_PCI = 1,
    DEVICE_TYPE_DISPLAY = 2,
    DEVICE_TYPE_INPUT = 3,
    DEVICE_TYPE_USB = 4,
    DEVICE_TYPE_NIC = 5,
    DEVICE_TYPE_BLOCK = 6,
    DEVICE_TYPE_AUDIO = 7,
    DEVICE_TYPE_FILESYSTEM = 8,
    DEVICE_TYPE_SERIAL = 9,
    /* "Built-in driver" -- statically linked into the kernel rather than a
     * loadable .ELF module, but registered into DeviceRegistry the same
     * way so it shows up alongside real modules in driver listings (e.g.
     * OSDebug). Used for subsystems that must initialize before the
     * driver-module loader itself can run (ACPI, Timer, LAPIC, IOAPIC) --
     * see Docs/Others/TODO_OS_Refactor.md phase P5, 9.1 (decision #3,
     * "Plan A"), and Kernel/Drivers/Module/PlatformBuiltinDrivers.c. */
    DEVICE_TYPE_PLATFORM = 10,
    /* One network protocol layer (Ethernet/ARP/IPv4/ICMP/UDP/TCP/DHCP).
     * Registered the same "built-in driver" way as DEVICE_TYPE_PLATFORM,
     * NOT as separately loaded .ELF modules -- see Docs/Others/
     * TODO_OS_Refactor.md phase P5, 9.2 (decision #4's individual-ELF
     * request landed as this hybrid: each layer gets a real, individually
     * named DeviceRegistry entry with a genuine callable vtable and
     * dependency metadata, but cross-layer calls inside Kernel/Network/
     * remain direct C calls rather than driver_manager_find() lookups, to
     * avoid rewriting ~3200 lines of tightly cross-coupled,
     * timing-sensitive protocol code in one pass -- see
     * Kernel/Drivers/Module/NetworkBuiltinDrivers.c). */
    DEVICE_TYPE_NET_PROTOCOL = 11,
} device_type_t;

typedef struct device {
    device_type_t type;
    const void *ops;
    void *priv;
    const char *name;
} device_t;
