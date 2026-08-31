#include "BusRegistry.h"
#include "DriverModule.h"
#include "DriverDB.h"

#include "kernel/pnp.h"
#include "IPC/PnP_Notifications.h"

#include <stddef.h>
#include <string.h>

#define BUS_REGISTRY_ON_DEMAND_DIR "/Kernel/Driver/OnDemand/"

void bus_registry_init(void)
{
    /* Nothing to initialize today: matching walks DriverModule.c's module
     * table directly (driver_module_manager_count/_descriptor_at), which
     * owns its own state. Kept as an explicit entry point so kernel_main.c
     * has one obvious place to call, matching device_registry_init()'s
     * shape, and so Phase 4's manifest reader has a natural place to hook
     * its own one-time init later. */
}

static bool bus_match_hits(const driver_bus_match_t *m, const bus_device_t *dev)
{
    if (m->bus_type != dev->bus_type) {
        return false;
    }
    if ((m->match_flags & DRIVER_BUS_MATCH_VENDOR) != 0u && m->vendor_id != dev->vendor_id) {
        return false;
    }
    if ((m->match_flags & DRIVER_BUS_MATCH_DEVICE) != 0u && m->device_id != dev->device_id) {
        return false;
    }
    if ((m->match_flags & DRIVER_BUS_MATCH_CLASS) != 0u && m->class_code != dev->class_code) {
        return false;
    }
    if ((m->match_flags & DRIVER_BUS_MATCH_SUBCLASS) != 0u && m->subclass != dev->subclass) {
        return false;
    }
    if ((m->match_flags & DRIVER_BUS_MATCH_PROTOCOL) != 0u && m->protocol != dev->protocol) {
        return false;
    }
    return true;
}

static uint32_t bus_registry_pnp_bus(device_type_t bus_type)
{
    if (bus_type == DEVICE_TYPE_PCI) {
        return PNP_BUS_PCI;
    }
    if (bus_type == DEVICE_TYPE_USB) {
        return PNP_BUS_USB;
    }
    return PNP_BUS_UNKNOWN;
}

static uint32_t bus_registry_pnp_class(device_type_t bus_type)
{
    if (bus_type == DEVICE_TYPE_PCI) {
        return PNP_CLASS_PCI_DEVICE;
    }
    if (bus_type == DEVICE_TYPE_USB) {
        return PNP_CLASS_USB_DEVICE;
    }
    return PNP_CLASS_UNKNOWN;
}

static void bus_registry_publish(uint16_t action, const bus_device_t *dev)
{
    pnp_event_t event;
    pnp_event_init(&event,
                   action,
                   bus_registry_pnp_bus(dev->bus_type),
                   bus_registry_pnp_class(dev->bus_type),
                   "BusRegistry",
                   "Bus device",
                   action == PNP_EVENT_DEVICE_ADDED ? "claimed by a device driver's probe()"
                                                     : "removed");
    event.vendor_id = dev->vendor_id;
    event.device_id = dev->device_id;
    pnp_notifications_publish(&event);
}

static bool bus_registry_try_loaded_modules(const bus_device_t *dev)
{
    uint32_t count = driver_module_manager_count();
    for (uint32_t i = 0u; i < count; ++i) {
        const driver_module_descriptor_t *desc = driver_module_manager_descriptor_at(i);
        if (desc == NULL || desc->probe == NULL ||
            desc->bus_matches == NULL || desc->bus_match_count == 0u) {
            continue;
        }

        for (uint32_t m = 0u; m < desc->bus_match_count; ++m) {
            if (!bus_match_hits(&desc->bus_matches[m], dev)) {
                continue;
            }
            if (desc->probe(dev)) {
                return true;
            }
        }
    }
    return false;
}

bool bus_registry_report_device(const bus_device_t *dev)
{
    if (dev == NULL) {
        return false;
    }

    if (bus_registry_try_loaded_modules(dev)) {
        bus_registry_publish(PNP_EVENT_DEVICE_ADDED, dev);
        return true;
    }

    /* No already-loaded module claimed it -- consult the on-demand
     * manifest (DriverDB.c) for a driver .ELF that isn't loaded yet. A
     * miss here (no manifest, no matching entry, or the named file missing
     * under Kernel/Driver/OnDemand/) is not an error: most devices simply
     * have no driver at all, dynamically loadable or otherwise. */
    char module_name[DRIVER_DB_MODULE_NAME_MAX];
    if (!driver_db_lookup(dev, module_name, (uint32_t)sizeof(module_name))) {
        return false;
    }

    char path[sizeof(BUS_REGISTRY_ON_DEMAND_DIR) + DRIVER_DB_MODULE_NAME_MAX];
    size_t dir_len = strlen(BUS_REGISTRY_ON_DEMAND_DIR);
    memcpy(path, BUS_REGISTRY_ON_DEMAND_DIR, dir_len);
    size_t name_len = strlen(module_name);
    memcpy(path + dir_len, module_name, name_len + 1u);

    if (!driver_module_manager_load_from_vfs(path)) {
        return false;
    }

    /* Loaded (and activated -- driver_module_manager_load_from_vfs()
     * already ran driver_module_prepare()/_activate()) for the first time
     * just now; its bus_matches[] is only known after that, so retry the
     * match now that it's a "loaded module" too. */
    if (bus_registry_try_loaded_modules(dev)) {
        bus_registry_publish(PNP_EVENT_DEVICE_ADDED, dev);
        return true;
    }
    return false;
}

void bus_registry_report_device_removed(const bus_device_t *dev)
{
    if (dev == NULL) {
        return;
    }

    uint32_t count = driver_module_manager_count();
    for (uint32_t i = 0u; i < count; ++i) {
        const driver_module_descriptor_t *desc = driver_module_manager_descriptor_at(i);
        if (desc == NULL || desc->remove == NULL ||
            desc->bus_matches == NULL || desc->bus_match_count == 0u) {
            continue;
        }

        for (uint32_t m = 0u; m < desc->bus_match_count; ++m) {
            if (bus_match_hits(&desc->bus_matches[m], dev)) {
                desc->remove(dev);
                bus_registry_publish(PNP_EVENT_DEVICE_REMOVED, dev);
                return;
            }
        }
    }
}
