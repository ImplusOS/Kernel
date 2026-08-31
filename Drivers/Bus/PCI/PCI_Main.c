#include "Drivers/Module/PCI_Main.h"

#include <stdint.h>
#include <stddef.h>

#define PCI_MAX_SCANNED_DEVICES 128u
#define PCI_MAX_REGISTERED_DRIVERS 32u

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"
#else
#include "Platform/io/IO_Main.h"
#include "Core/sync/Spinlock.h"
#endif

#ifdef IMPLUS_DRIVER_MODULE
static const driver_binary_t *g_driver_api = NULL;

#define outl g_driver_api->outl
#define inl g_driver_api->inl

#define hal_cpu_pause               g_driver_api->hal.cpu_pause
#define hal_cpu_save_interrupts     g_driver_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts  g_driver_api->hal.cpu_restore_interrupts

typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_init(spinlock_t *l)   { l->locked = 0; }
static inline void spinlock_lock(spinlock_t *l)   {
    while (__sync_lock_test_and_set(&l->locked, 1)) {
        while (l->locked) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->locked); }

static inline uint64_t irq_save_disable(void) { return hal_cpu_save_interrupts(); }
static inline void irq_restore(uint64_t flags) { hal_cpu_restore_interrupts(flags); }
#endif

#if defined(__aarch64__)
#define ARM64_QEMU_VIRT_PCI_ECAM_BASE 0x4010000000ULL
#define ARM64_PCI_ECAM_BUS_SHIFT      20u
#define ARM64_PCI_ECAM_DEVICE_SHIFT   15u
#define ARM64_PCI_ECAM_FUNC_SHIFT     12u
#define ARM64_PCI_ECAM_BUS_SIZE       (1ULL << ARM64_PCI_ECAM_BUS_SHIFT)
#define ARM64_PCI_ECAM_BUS_COUNT      256u

static volatile uint8_t *g_arm64_pci_ecam = NULL;
static uint8_t g_arm64_pci_bus_mapped[ARM64_PCI_ECAM_BUS_COUNT];

static bool arm64_pci_ensure_bus_mapped(uint8_t bus)
{
    if (g_arm64_pci_bus_mapped[bus] != 0u) {
        return true;
    }
    if (g_driver_api == NULL || g_driver_api->map_mmio_virt == NULL) {
        return false;
    }

    uint64_t bus_base = ARM64_QEMU_VIRT_PCI_ECAM_BASE +
                        ((uint64_t)bus * ARM64_PCI_ECAM_BUS_SIZE);
    if (g_driver_api->map_mmio_virt(bus_base) == NULL) {
        return false;
    }

    g_arm64_pci_bus_mapped[bus] = 1u;
    if (g_arm64_pci_ecam == NULL) {
        g_arm64_pci_ecam = (volatile uint8_t *)(uintptr_t)ARM64_QEMU_VIRT_PCI_ECAM_BASE;
    }
    return true;
}

static volatile uint32_t *arm64_pci_config_addr(uint8_t bus,
                                                uint8_t device,
                                                uint8_t func,
                                                uint8_t offset)
{
    if (!arm64_pci_ensure_bus_mapped(bus) || g_arm64_pci_ecam == NULL) {
        return NULL;
    }

    uint64_t ecam_offset =
        ((uint64_t)bus << ARM64_PCI_ECAM_BUS_SHIFT) |
        ((uint64_t)device << ARM64_PCI_ECAM_DEVICE_SHIFT) |
        ((uint64_t)func << ARM64_PCI_ECAM_FUNC_SHIFT) |
        ((uint64_t)offset & 0xFCu);

    return (volatile uint32_t *)(g_arm64_pci_ecam + ecam_offset);
}
#else
static spinlock_t g_pci_lock = {0};
#endif

static pci_device_t g_pci_devices[PCI_MAX_SCANNED_DEVICES];
static uint32_t g_pci_device_count = 0;
static pci_bus_driver_t *g_pci_drivers[PCI_MAX_REGISTERED_DRIVERS];
static uint32_t g_pci_driver_count = 0;
static uint8_t g_pci_scan_done = 0;

uint32_t pci_read_config(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset)
{
#if defined(__aarch64__)
    volatile uint32_t *addr = arm64_pci_config_addr(bus, device, func, offset);
    if (addr == NULL) {
        return 0xFFFFFFFFu;
    }
    return *addr;
#else
    uint32_t address = (1u << 31) |
                       ((uint32_t)bus << 16) |
                       ((uint32_t)device << 11) |
                       ((uint32_t)func << 8) |
                       ((uint32_t)offset & 0xFCu);

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_pci_lock);
    outl(PCI_CONFIG_ADDRESS, address);
    uint32_t val = inl(PCI_CONFIG_DATA);
    spinlock_unlock(&g_pci_lock);
    irq_restore(flags);
    return val;
#endif
}

void pci_write_config(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset, uint32_t value)
{
#if defined(__aarch64__)
    volatile uint32_t *addr = arm64_pci_config_addr(bus, device, func, offset);
    if (addr == NULL) {
        return;
    }
    *addr = value;
#else
    uint32_t address = (1u << 31) |
                       ((uint32_t)bus << 16) |
                       ((uint32_t)device << 11) |
                       ((uint32_t)func << 8) |
                       ((uint32_t)offset & 0xFCu);

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_pci_lock);
    outl(PCI_CONFIG_ADDRESS, address);
    outl(PCI_CONFIG_DATA, value);
    spinlock_unlock(&g_pci_lock);
    irq_restore(flags);
#endif
}

uint32_t pci_read_bar(uint8_t bus, uint8_t device, uint8_t func, uint8_t bar_index)
{
    if (bar_index >= 6u) {
        return 0;
    }
    return pci_read_config(bus, device, func, (uint8_t)(0x10u + (bar_index * 4u)));
}

static void pci_read_bars(pci_device_t *dev)
{
    for (int i = 0; i < 6; i++) {
        dev->bar[i] = pci_read_bar(dev->bus, dev->device, dev->func, (uint8_t)i);
    }
}

bool pci_get_bar_info(uint8_t bus,
                      uint8_t device,
                      uint8_t func,
                      uint8_t bar_index,
                      pci_bar_info_t *out_bar)
{
    if (out_bar == NULL || bar_index >= 6u) {
        return false;
    }

    uint8_t offset = (uint8_t)(0x10u + bar_index * 4u);
    uint32_t original_low = pci_read_config(bus, device, func, offset);
    if (original_low == 0u || original_low == 0xFFFFFFFFu) {
        return false;
    }

    out_bar->is_io = (original_low & 1u) != 0u;
    out_bar->is_64bit = false;
    out_bar->prefetchable = false;
    out_bar->address = 0u;
    out_bar->size = 0u;

    uint32_t original_high = 0u;
    if (out_bar->is_io) {
        out_bar->address = (uint64_t)(original_low & ~0x3u);
    } else {
        uint32_t type = (original_low >> 1u) & 0x3u;
        out_bar->prefetchable = (original_low & 0x8u) != 0u;
        out_bar->is_64bit = type == 0x2u;
        if (out_bar->is_64bit) {
            if (bar_index >= 5u) {
                return false;
            }
            original_high = pci_read_config(bus, device, func,
                                            (uint8_t)(offset + 4u));
            out_bar->address = ((uint64_t)original_high << 32u) |
                               (uint64_t)(original_low & ~0xFu);
        } else {
            out_bar->address = (uint64_t)(original_low & ~0xFu);
        }
    }

    uint32_t command = pci_read_config(bus, device, func, 0x04u);
    pci_write_config(bus, device, func, 0x04u, command & ~0x3u);
    pci_write_config(bus, device, func, offset, 0xFFFFFFFFu);
    uint32_t size_low = pci_read_config(bus, device, func, offset);
    uint32_t size_high = 0u;
    if (out_bar->is_64bit) {
        pci_write_config(bus, device, func, (uint8_t)(offset + 4u),
                         0xFFFFFFFFu);
        size_high = pci_read_config(bus, device, func,
                                    (uint8_t)(offset + 4u));
    }
    pci_write_config(bus, device, func, offset, original_low);
    if (out_bar->is_64bit) {
        pci_write_config(bus, device, func, (uint8_t)(offset + 4u),
                         original_high);
    }
    pci_write_config(bus, device, func, 0x04u, command);

    if (out_bar->is_io) {
        uint32_t mask = size_low & ~0x3u;
        if (mask != 0u) {
            out_bar->size = (uint64_t)(~mask + 1u);
        }
    } else if (out_bar->is_64bit) {
        uint64_t mask = ((uint64_t)size_high << 32u) |
                        (uint64_t)(size_low & ~0xFu);
        if (mask != 0u) {
            out_bar->size = ~mask + 1u;
        }
    } else {
        uint32_t mask = size_low & ~0xFu;
        if (mask != 0u) {
            out_bar->size = (uint64_t)(~mask + 1u);
        }
    }
    return out_bar->address != 0u;
}

int32_t pci_find_capability(uint8_t bus,
                            uint8_t device,
                            uint8_t func,
                            uint8_t capability_id)
{
    uint32_t status_command = pci_read_config(bus, device, func, 0x04u);
    if ((status_command & (1u << 20u)) == 0u) {
        return -1;
    }

    uint8_t offset =
        (uint8_t)(pci_read_config(bus, device, func, 0x34u) & 0xFCu);
    for (uint32_t guard = 0u; offset >= 0x40u && guard < 64u; ++guard) {
        uint32_t header = pci_read_config(bus, device, func,
                                          (uint8_t)(offset & 0xFCu));
        if ((uint8_t)(header & 0xFFu) == capability_id) {
            return (int32_t)offset;
        }
        offset = (uint8_t)((header >> 8u) & 0xFCu);
    }
    return -1;
}

static uint32_t pci_device_class_key(const pci_device_t *dev)
{
    return ((uint32_t)dev->class_code << 16) |
           ((uint32_t)dev->subclass << 8) |
           (uint32_t)dev->prog_if;
}

static bool pci_id_matches(const pci_device_id_t *id, const pci_device_t *dev)
{
    if (id == NULL || dev == NULL) {
        return false;
    }
    if (id->vendor_id != PCI_ANY_ID && id->vendor_id != dev->vendor_id) {
        return false;
    }
    if (id->device_id != PCI_ANY_ID && id->device_id != dev->device_id) {
        return false;
    }
    if (id->class_code != PCI_ANY_CLASS &&
        id->class_code != pci_device_class_key(dev) &&
        id->class_code != (uint32_t)dev->class_code) {
        return false;
    }
    return true;
}

static void pci_probe_device_with_driver(const pci_device_t *dev, pci_bus_driver_t *driver)
{
    if (dev == NULL || driver == NULL ||
        driver->id_table == NULL || driver->probe == NULL) {
        return;
    }

    for (const pci_device_id_t *id = driver->id_table;
         id->vendor_id != 0u || id->device_id != 0u || id->class_code != 0u;
         ++id) {
        if (pci_id_matches(id, dev)) {
            (void)driver->probe(dev);
            break;
        }
    }
}

/* Reports `dev` to BusRegistry (Kernel/Drivers/Module/BusRegistry.c) so a
 * *loadable* device driver module (VirtIONet.c, ...) can claim it via a
 * declarative driver_bus_match_t + probe() in its own descriptor -- see
 * DriverBinary.h. This is additive, alongside (not instead of)
 * pci_register_driver()/g_pci_drivers[] above, which is for PCI device
 * drivers compiled into the kernel itself; loadable driver modules can't
 * reach that API (it's declared in a kernel-internal header, not exposed
 * through driver_binary_t), so they need this path instead. */
static void pci_report_bus_device(const pci_device_t *dev)
{
#ifdef IMPLUS_DRIVER_MODULE
    if (dev == NULL || g_driver_api == NULL || g_driver_api->bus.report_device == NULL) {
        return;
    }

    driver_pci_device_t pci_ctx;
    pci_ctx.bus = dev->bus;
    pci_ctx.device = dev->device;
    pci_ctx.function = dev->func;
    pci_ctx.class_code = dev->class_code;
    pci_ctx.subclass = dev->subclass;
    pci_ctx.prog_if = dev->prog_if;
    pci_ctx.revision = dev->revision;
    pci_ctx.vendor_id = dev->vendor_id;
    pci_ctx.device_id = dev->device_id;
    pci_ctx.interrupt_line = dev->interrupt_line;
    pci_ctx.interrupt_pin = dev->interrupt_pin;

    bus_device_t bus_dev;
    bus_dev.bus_type = DEVICE_TYPE_PCI;
    bus_dev.vendor_id = dev->vendor_id;
    bus_dev.device_id = dev->device_id;
    bus_dev.class_code = dev->class_code;
    bus_dev.subclass = dev->subclass;
    bus_dev.protocol = dev->prog_if;
    bus_dev.bus_context = &pci_ctx;

    (void)g_driver_api->bus.report_device(&bus_dev);
#else
    (void)dev;
#endif
}

static void pci_report_bus_device_removed(const pci_device_t *dev)
{
#ifdef IMPLUS_DRIVER_MODULE
    if (dev == NULL || g_driver_api == NULL || g_driver_api->bus.report_device_removed == NULL) {
        return;
    }

    driver_pci_device_t pci_ctx;
    pci_ctx.bus = dev->bus;
    pci_ctx.device = dev->device;
    pci_ctx.function = dev->func;
    pci_ctx.class_code = dev->class_code;
    pci_ctx.subclass = dev->subclass;
    pci_ctx.prog_if = dev->prog_if;
    pci_ctx.revision = dev->revision;
    pci_ctx.vendor_id = dev->vendor_id;
    pci_ctx.device_id = dev->device_id;
    pci_ctx.interrupt_line = dev->interrupt_line;
    pci_ctx.interrupt_pin = dev->interrupt_pin;

    bus_device_t bus_dev;
    bus_dev.bus_type = DEVICE_TYPE_PCI;
    bus_dev.vendor_id = dev->vendor_id;
    bus_dev.device_id = dev->device_id;
    bus_dev.class_code = dev->class_code;
    bus_dev.subclass = dev->subclass;
    bus_dev.protocol = dev->prog_if;
    bus_dev.bus_context = &pci_ctx;

    g_driver_api->bus.report_device_removed(&bus_dev);
#else
    (void)dev;
#endif
}

static void pci_probe_device(const pci_device_t *dev)
{
    for (uint32_t i = 0; i < g_pci_driver_count; ++i) {
        pci_probe_device_with_driver(dev, g_pci_drivers[i]);
    }
    pci_report_bus_device(dev);
}

static void pci_remove_device(const pci_device_t *dev)
{
    if (dev == NULL) {
        return;
    }

    pci_report_bus_device_removed(dev);

    for (uint32_t i = 0; i < g_pci_driver_count; ++i) {
        pci_bus_driver_t *driver = g_pci_drivers[i];
        if (driver == NULL ||
            driver->id_table == NULL ||
            driver->remove == NULL) {
            continue;
        }

        for (const pci_device_id_t *id = driver->id_table;
             id->vendor_id != 0u || id->device_id != 0u || id->class_code != 0u;
             ++id) {
            if (pci_id_matches(id, dev)) {
                driver->remove(dev);
                break;
            }
        }
    }
}

static void pci_publish_device_event(const pci_device_t *dev,
                                     uint16_t action,
                                     const char *detail)
{
#ifdef IMPLUS_DRIVER_MODULE
    if (dev == NULL || g_driver_api == NULL ||
        g_driver_api->pnp_notify == NULL) {
        return;
    }

    pnp_event_t event;
    pnp_event_init(&event,
                   action,
                   PNP_BUS_PCI,
                   PNP_CLASS_PCI_DEVICE,
                   "PCI_Driver.ELF",
                   "PCI device",
                   detail);
    event.vendor_id = dev->vendor_id;
    event.device_id = dev->device_id;
    event.location0 = ((uint32_t)dev->bus << 16u) |
                      ((uint32_t)dev->device << 8u) |
                      (uint32_t)dev->func;
    event.location1 = ((uint32_t)dev->class_code << 16u) |
                      ((uint32_t)dev->subclass << 8u) |
                      (uint32_t)dev->prog_if;
    g_driver_api->pnp_notify(&event);
#else
    (void)dev;
    (void)action;
    (void)detail;
#endif
}

static void pci_store_device(const pci_device_t *dev)
{
    if (dev == NULL || g_pci_device_count >= PCI_MAX_SCANNED_DEVICES) {
        return;
    }
    g_pci_devices[g_pci_device_count++] = *dev;
    pci_publish_device_event(dev,
                             PNP_EVENT_DEVICE_ADDED,
                             "PCI function enumerated");
    pci_probe_device(dev);
}

int pci_find_device(uint16_t vendor_id, uint16_t device_id, pci_device_t *out_device)
{
    if (out_device == NULL) {
        return 0;
    }

    for (uint16_t bus = 0; bus < 256; bus++) {
        for (uint8_t device = 0; device < 32; device++) {
            for (uint8_t func = 0; func < 8; func++) {
                uint32_t vendor_device = pci_read_config((uint8_t)bus, device, func, 0x00);
                uint16_t cur_vendor = (uint16_t)(vendor_device & 0xFFFFu);
                uint16_t cur_device = (uint16_t)((vendor_device >> 16) & 0xFFFFu);

                if (cur_vendor == 0xFFFFu) {
                    if (func == 0u) {
                        break;
                    }
                    continue;
                }

                if (cur_vendor == vendor_id && cur_device == device_id) {
                    uint32_t class_reg = pci_read_config((uint8_t)bus, device, func, 0x08);
                    out_device->bus = (uint8_t)bus;
                    out_device->device = device;
                    out_device->func = func;
                    out_device->vendor_id = cur_vendor;
                    out_device->device_id = cur_device;
                    out_device->class_code = (uint8_t)((class_reg >> 24) & 0xFFu);
                    out_device->subclass = (uint8_t)((class_reg >> 16) & 0xFFu);
                    out_device->prog_if = (uint8_t)((class_reg >> 8) & 0xFFu);
                    out_device->revision = (uint8_t)(class_reg & 0xFFu);
                    uint32_t irq_reg = pci_read_config((uint8_t)bus, device,
                                                       func, 0x3Cu);
                    out_device->interrupt_line = (uint8_t)(irq_reg & 0xFFu);
                    out_device->interrupt_pin =
                        (uint8_t)((irq_reg >> 8u) & 0xFFu);
                    pci_read_bars(out_device);
                    return 1;
                }

                if (func == 0u) {
                    uint32_t header_type = pci_read_config((uint8_t)bus, device, func, 0x0C);
                    if (((header_type >> 16) & 0x80u) == 0u) {
                        break;
                    }
                }
            }
        }
    }

    return 0;
}

static bool pci_same_slot(const pci_device_t *a, const pci_device_t *b)
{
    return a != NULL && b != NULL &&
           a->bus == b->bus &&
           a->device == b->device &&
           a->func == b->func;
}

static bool pci_same_config(const pci_device_t *a, const pci_device_t *b)
{
    if (!pci_same_slot(a, b)) {
        return false;
    }
    if (a->vendor_id != b->vendor_id ||
        a->device_id != b->device_id ||
        a->class_code != b->class_code ||
        a->subclass != b->subclass ||
        a->prog_if != b->prog_if ||
        a->revision != b->revision ||
        a->interrupt_line != b->interrupt_line ||
        a->interrupt_pin != b->interrupt_pin) {
        return false;
    }
    for (uint32_t i = 0u; i < 6u; ++i) {
        if (a->bar[i] != b->bar[i]) {
            return false;
        }
    }
    return true;
}

static int32_t pci_find_in_list(const pci_device_t *devices,
                                uint32_t count,
                                const pci_device_t *needle)
{
    if (devices == NULL || needle == NULL) {
        return -1;
    }
    for (uint32_t i = 0u; i < count; ++i) {
        if (pci_same_slot(&devices[i], needle)) {
            return (int32_t)i;
        }
    }
    return -1;
}

static uint32_t pci_collect_devices(pci_device_t *devices, uint32_t max_devices)
{
    uint32_t count = 0u;

    if (devices == NULL || max_devices == 0u) {
        return 0u;
    }

    for (uint16_t bus = 0; bus < 256 && count < max_devices; bus++) {
        for (uint8_t device = 0; device < 32 && count < max_devices; device++) {
            for (uint8_t func = 0; func < 8 && count < max_devices; func++) {
                uint32_t vendor_device = pci_read_config((uint8_t)bus, device, func, 0x00);
                uint16_t vendor_id = (uint16_t)(vendor_device & 0xFFFFu);
                uint16_t device_id = (uint16_t)((vendor_device >> 16) & 0xFFFFu);

                if (vendor_id == 0xFFFFu) {
                    if (func == 0u) {
                        break;
                    }
                    continue;
                }

                uint32_t class_reg = pci_read_config((uint8_t)bus, device, func, 0x08);
                pci_device_t *dev = &devices[count++];
                dev->bus = (uint8_t)bus;
                dev->device = device;
                dev->func = func;
                dev->vendor_id = vendor_id;
                dev->device_id = device_id;
                dev->class_code = (uint8_t)((class_reg >> 24) & 0xFFu);
                dev->subclass = (uint8_t)((class_reg >> 16) & 0xFFu);
                dev->prog_if = (uint8_t)((class_reg >> 8) & 0xFFu);
                dev->revision = (uint8_t)(class_reg & 0xFFu);
                uint32_t irq_reg = pci_read_config((uint8_t)bus, device,
                                                   func, 0x3Cu);
                dev->interrupt_line = (uint8_t)(irq_reg & 0xFFu);
                dev->interrupt_pin = (uint8_t)((irq_reg >> 8u) & 0xFFu);
                pci_read_bars(dev);

                if (func == 0u) {
                    uint32_t header_type = pci_read_config((uint8_t)bus, device, func, 0x0C);
                    if (((header_type >> 16) & 0x80u) == 0u) {
                        break;
                    }
                }
            }
        }
    }

    return count;
}

void pci_scan_bus(void)
{
    pci_device_t scanned[PCI_MAX_SCANNED_DEVICES];
    uint32_t scanned_count = pci_collect_devices(scanned,
                                                 PCI_MAX_SCANNED_DEVICES);

    if (g_pci_scan_done == 0u) {
        g_pci_device_count = 0u;
        for (uint32_t i = 0u; i < scanned_count; ++i) {
            pci_store_device(&scanned[i]);
        }
        g_pci_scan_done = 1u;
        return;
    }

    for (uint32_t i = 0u; i < g_pci_device_count; ++i) {
        const pci_device_t *old_dev = &g_pci_devices[i];
        int32_t new_index = pci_find_in_list(scanned, scanned_count, old_dev);
        if (new_index < 0) {
            pci_publish_device_event(old_dev,
                                     PNP_EVENT_DEVICE_REMOVED,
                                     "PCI function removed");
            pci_remove_device(old_dev);
            continue;
        }

        const pci_device_t *new_dev = &scanned[(uint32_t)new_index];
        if (!pci_same_config(old_dev, new_dev)) {
            if (old_dev->vendor_id != new_dev->vendor_id ||
                old_dev->device_id != new_dev->device_id ||
                old_dev->class_code != new_dev->class_code ||
                old_dev->subclass != new_dev->subclass ||
                old_dev->prog_if != new_dev->prog_if) {
                pci_publish_device_event(old_dev,
                                         PNP_EVENT_DEVICE_REMOVED,
                                         "PCI function removed");
                pci_remove_device(old_dev);
                pci_publish_device_event(new_dev,
                                         PNP_EVENT_DEVICE_ADDED,
                                         "PCI function hotplugged");
                pci_probe_device(new_dev);
            } else {
                pci_publish_device_event(new_dev,
                                         PNP_EVENT_DEVICE_CHANGED,
                                         "PCI function changed");
            }
        }
    }

    for (uint32_t i = 0u; i < scanned_count; ++i) {
        if (pci_find_in_list(g_pci_devices, g_pci_device_count,
                             &scanned[i]) < 0) {
            pci_publish_device_event(&scanned[i],
                                     PNP_EVENT_DEVICE_ADDED,
                                     "PCI function hotplugged");
            pci_probe_device(&scanned[i]);
        }
    }

    g_pci_device_count = scanned_count;
    for (uint32_t i = 0u; i < scanned_count; ++i) {
        g_pci_devices[i] = scanned[i];
    }
    g_pci_scan_done = 1u;
}

uint32_t pci_get_device_count(void)
{
    if (g_pci_scan_done == 0u) {
        pci_scan_bus();
    }
    return g_pci_device_count;
}

const pci_device_t *pci_get_device(uint32_t index)
{
    if (g_pci_scan_done == 0u) {
        pci_scan_bus();
    }
    if (index >= g_pci_device_count) {
        return NULL;
    }
    return &g_pci_devices[index];
}

int pci_register_driver(pci_bus_driver_t *driver)
{
    if (driver == NULL || driver->id_table == NULL || driver->probe == NULL) {
        return -1;
    }
    if (g_pci_driver_count >= PCI_MAX_REGISTERED_DRIVERS) {
        return -1;
    }

    g_pci_drivers[g_pci_driver_count++] = driver;
    if (g_pci_scan_done == 0u) {
        pci_scan_bus();
        return 0;
    }

    for (uint32_t i = 0; i < g_pci_device_count; ++i) {
        pci_probe_device_with_driver(&g_pci_devices[i], driver);
    }
    return 0;
}

#ifdef IMPLUS_DRIVER_MODULE
static const pci_driver_t g_pci_driver = {
    .read_config = pci_read_config,
    .write_config = pci_write_config,
    .scan_bus = pci_scan_bus,
    .find_device = pci_find_device,
    .read_bar = pci_read_bar,
    .get_device_count = pci_get_device_count,
    .get_device = pci_get_device,
    .get_bar_info = pci_get_bar_info,
    .find_capability = pci_find_capability,
};

static void pci_driver_shutdown(void)
{
    g_driver_api = NULL;
#if defined(__aarch64__)
    g_arm64_pci_ecam = NULL;
    for (uint32_t i = 0; i < ARM64_PCI_ECAM_BUS_COUNT; ++i) {
        g_arm64_pci_bus_mapped[i] = 0u;
    }
#endif
}

static const driver_module_descriptor_t g_pci_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_PCI,
    .load_priority = 10u,
    .deps = { NULL },
    .driver_api = &g_pci_driver,
    .shutdown = pci_driver_shutdown,
};

#undef outl
#undef inl

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL ||
        api->outl == NULL ||
        api->inl == NULL ||
        api->hal.cpu_pause == NULL ||
        api->hal.cpu_save_interrupts == NULL ||
        api->hal.cpu_restore_interrupts == NULL) {
        return NULL;
    }

    g_driver_api = api;
    return &g_pci_module;
}
#endif
