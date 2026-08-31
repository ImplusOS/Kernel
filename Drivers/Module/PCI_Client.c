#include "PCI_Main.h"

#include "Drivers/Module/DriverBinary.h"
#include "Drivers/Module/DriverManager.h"

#include <stdbool.h>
#include <stdint.h>

static const pci_driver_t *get_pci_driver(void)
{
    const device_t *device = driver_manager_find(DEVICE_TYPE_PCI, NULL);
    return device ? (const pci_driver_t *)device->ops : NULL;
}

uint32_t pci_read_config(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset)
{
    const pci_driver_t *drv = get_pci_driver();
    if (!drv) {
        return 0xFFFFFFFFu;
    }
    return drv->read_config(bus, device, func, offset);
}

void pci_write_config(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset, uint32_t value)
{
    const pci_driver_t *drv = get_pci_driver();
    if (!drv) {
        return;
    }
    drv->write_config(bus, device, func, offset, value);
}

void pci_scan_bus(void)
{
    const pci_driver_t *drv = get_pci_driver();
    if (!drv) {
        return;
    }
    if (drv->scan_bus != NULL) {
        drv->scan_bus();
    }
}

int pci_find_device(uint16_t vendor_id, uint16_t device_id, pci_device_t *out_device)
{
    if (out_device == NULL) {
        return 0;
    }
    const pci_driver_t *drv = get_pci_driver();
    if (!drv) {
        return 0;
    }
    if (drv->find_device == NULL) {
        return 0;
    }
    return drv->find_device(vendor_id, device_id, out_device);
}

uint32_t pci_read_bar(uint8_t bus, uint8_t device, uint8_t func, uint8_t bar_index)
{
    const pci_driver_t *drv = get_pci_driver();
    if (!drv) {
        return 0;
    }
    if (drv->read_bar != NULL) {
        return drv->read_bar(bus, device, func, bar_index);
    }
    if (bar_index >= 6u) {
        return 0;
    }
    return drv->read_config(bus, device, func, (uint8_t)(0x10u + (bar_index * 4u)));
}

uint32_t pci_get_device_count(void)
{
    const pci_driver_t *drv = get_pci_driver();
    if (drv == NULL || drv->get_device_count == NULL) {
        return 0u;
    }
    return drv->get_device_count();
}

const pci_device_t *pci_get_device(uint32_t index)
{
    const pci_driver_t *drv = get_pci_driver();
    if (drv == NULL || drv->get_device == NULL) {
        return NULL;
    }
    return drv->get_device(index);
}

bool pci_get_bar_info(uint8_t bus,
                      uint8_t device,
                      uint8_t func,
                      uint8_t bar_index,
                      pci_bar_info_t *out_bar)
{
    const pci_driver_t *drv = get_pci_driver();
    if (drv == NULL || drv->get_bar_info == NULL) {
        return false;
    }
    return drv->get_bar_info(bus, device, func, bar_index, out_bar);
}

int32_t pci_find_capability(uint8_t bus,
                            uint8_t device,
                            uint8_t func,
                            uint8_t capability_id)
{
    const pci_driver_t *drv = get_pci_driver();
    if (drv == NULL || drv->find_capability == NULL) {
        return -1;
    }
    return drv->find_capability(bus, device, func, capability_id);
}
