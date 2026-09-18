#include "BusRegistry.h"
#include "DriverModule.h"
#include "DriverDB.h"

#include "kernel/pnp.h"
#include "IPC/PnP_Notifications.h"
#include "Debug/serial/Serial.h"

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

/* ---- devices reported before the VFS was mounted ------------------------
 *
 * See bus_registry_retry_unclaimed() in BusRegistry.h for why these have to
 * be held at all. A pending entry owns a *copy* of the bus context, because
 * the reporting bus driver builds that on its own stack (USB_Main.c does) and
 * it is only valid for the duration of the report_device() call. */
#define BUS_REGISTRY_MAX_PENDING_PER_BUS 16u

typedef struct {
    bool         in_use;
    bool         has_ctx;
    bus_device_t dev;
    union {
        usb_device_context_t usb;
        driver_pci_device_t  pci;
    } ctx;
} bus_pending_t;

static bus_pending_t g_pending[BUS_REGISTRY_MAX_PENDING_PER_BUS * 2u];
static bool g_vfs_ready = false;

static bool bus_pending_same_device(const bus_pending_t *slot,
                                    const bus_device_t *dev)
{
    if (!slot->in_use ||
        slot->dev.bus_type != dev->bus_type ||
        slot->dev.vendor_id != dev->vendor_id ||
        slot->dev.device_id != dev->device_id) {
        return false;
    }
    /* One USB device can expose several vendor-specific interfaces and one
     * PCI id several functions, so the ids alone do not identify a device --
     * the bus context is what tells them apart. */
    if (slot->has_ctx && dev->bus_context != NULL) {
        if (dev->bus_type == DEVICE_TYPE_USB) {
            const usb_device_context_t *b =
                (const usb_device_context_t *)dev->bus_context;
            return slot->ctx.usb.addr == b->addr &&
                   slot->ctx.usb.interface == b->interface;
        }
        if (dev->bus_type == DEVICE_TYPE_PCI) {
            const driver_pci_device_t *b =
                (const driver_pci_device_t *)dev->bus_context;
            return slot->ctx.pci.bus == b->bus &&
                   slot->ctx.pci.device == b->device &&
                   slot->ctx.pci.function == b->function;
        }
    }
    return true;
}

static void bus_registry_remember_unclaimed(const bus_device_t *dev)
{
    if (g_vfs_ready) {
        return; /* the manifest is readable now -- nothing left to replay */
    }

    const uint32_t count = (uint32_t)(sizeof(g_pending) / sizeof(g_pending[0]));
    bus_pending_t *slot = NULL;
    uint32_t used_on_bus = 0u;

    for (uint32_t i = 0u; i < count; ++i) {
        if (bus_pending_same_device(&g_pending[i], dev)) {
            slot = &g_pending[i]; /* already held: refresh it in place */
            break;
        }
        if (g_pending[i].in_use && g_pending[i].dev.bus_type == dev->bus_type) {
            ++used_on_bus;
        }
    }

    if (slot == NULL) {
        /* Per-bus quota, not one shared pool: a real machine enumerates far
         * more PCI functions than there are slots here, and none of them may
         * crowd out the USB device this exists for (or the reverse). */
        if (used_on_bus >= BUS_REGISTRY_MAX_PENDING_PER_BUS) {
            return;
        }
        for (uint32_t i = 0u; i < count; ++i) {
            if (!g_pending[i].in_use) {
                slot = &g_pending[i];
                break;
            }
        }
    }
    if (slot == NULL) {
        return;
    }

    memset(slot, 0, sizeof(*slot));
    slot->dev = *dev;
    slot->dev.bus_context = NULL; /* re-pointed at our copy on replay */
    if (dev->bus_context != NULL) {
        if (dev->bus_type == DEVICE_TYPE_USB) {
            memcpy(&slot->ctx.usb, dev->bus_context, sizeof(slot->ctx.usb));
            slot->has_ctx = true;
        } else if (dev->bus_type == DEVICE_TYPE_PCI) {
            memcpy(&slot->ctx.pci, dev->bus_context, sizeof(slot->ctx.pci));
            slot->has_ctx = true;
        }
    }
    slot->in_use = true;
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

static bool bus_registry_dispatch(const bus_device_t *dev)
{
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

bool bus_registry_report_device(const bus_device_t *dev)
{
    if (dev == NULL) {
        return false;
    }

    if (bus_registry_dispatch(dev)) {
        return true;
    }

    /* Unclaimed. While the VFS is still down that verdict is not final: the
     * on-demand half of bus_registry_dispatch() could not read the manifest,
     * so hold the device for one replay once the filesystems are up. */
    bus_registry_remember_unclaimed(dev);
    return false;
}

void bus_registry_retry_unclaimed(void)
{
    /* Before the loop: from here on report_device() can read the manifest
     * itself, so nothing new needs holding (and a device claimed below must
     * not re-enter the pending table). */
    g_vfs_ready = true;

    const uint32_t count = (uint32_t)(sizeof(g_pending) / sizeof(g_pending[0]));
    for (uint32_t i = 0u; i < count; ++i) {
        bus_pending_t *slot = &g_pending[i];
        if (!slot->in_use) {
            continue;
        }

        bus_device_t dev = slot->dev;
        if (slot->has_ctx) {
            dev.bus_context = (dev.bus_type == DEVICE_TYPE_USB)
                                  ? (const void *)&slot->ctx.usb
                                  : (const void *)&slot->ctx.pci;
        }

        if (bus_registry_dispatch(&dev)) {
            serial_write_string("[BusRegistry] deferred match claimed vid ");
            serial_write_uint32(dev.vendor_id);
            serial_write_string(" pid ");
            serial_write_uint32(dev.device_id);
            serial_write_string("\n");
        }
        /* Claimed or not, this was its one replay: a device that still has no
         * driver now is one that has none at all. */
        slot->in_use = false;
    }
}

void bus_registry_report_device_removed(const bus_device_t *dev)
{
    if (dev == NULL) {
        return;
    }

    /* Gone before it was ever replayed: drop it, so a dongle unplugged during
     * boot is not probed after the fact. */
    const uint32_t pending_count =
        (uint32_t)(sizeof(g_pending) / sizeof(g_pending[0]));
    for (uint32_t i = 0u; i < pending_count; ++i) {
        if (bus_pending_same_device(&g_pending[i], dev)) {
            g_pending[i].in_use = false;
        }
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
