#ifndef PCI_H
#define PCI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCI_CONFIG_ADDRESS 0xCF8
#define PCI_CONFIG_DATA    0xCFC
#define PCI_ANY_ID         0xFFFFu
#define PCI_ANY_CLASS      0xFFFFFFFFu

typedef struct {
    uint8_t bus;
    uint8_t device;
    uint8_t func;
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t class_code;
    uint8_t subclass;
    uint8_t prog_if;
    uint32_t bar[6];
    uint8_t revision;
    uint8_t interrupt_line;
    uint8_t interrupt_pin;
} pci_device_t;

typedef struct {
    uint64_t address;
    uint64_t size;
    bool is_io;
    bool is_64bit;
    bool prefetchable;
} pci_bar_info_t;

typedef struct {
    uint16_t vendor_id;
    uint16_t device_id;
    uint32_t class_code;
} pci_device_id_t;

typedef struct pci_bus_driver {
    const char *name;
    const pci_device_id_t *id_table;
    bool (*probe)(const pci_device_t *dev);
    void (*remove)(const pci_device_t *dev);
    struct pci_bus_driver *next;
} pci_bus_driver_t;

typedef struct {
    uint32_t (*read_config)(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset);
    void (*write_config)(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset, uint32_t value);
    void (*scan_bus)(void);
    int (*find_device)(uint16_t vendor_id, uint16_t device_id, pci_device_t *out_device);
    uint32_t (*read_bar)(uint8_t bus, uint8_t device, uint8_t func, uint8_t bar_index);
    uint32_t (*get_device_count)(void);
    const pci_device_t *(*get_device)(uint32_t index);
    bool (*get_bar_info)(uint8_t bus,
                         uint8_t device,
                         uint8_t func,
                         uint8_t bar_index,
                         pci_bar_info_t *out_bar);
    int32_t (*find_capability)(uint8_t bus,
                               uint8_t device,
                               uint8_t func,
                               uint8_t capability_id);
} pci_driver_t;

uint32_t pci_read_config(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset);
void pci_write_config(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset, uint32_t value);

void pci_scan_bus(void);
int pci_find_device(uint16_t vendor_id, uint16_t device_id, pci_device_t *out_device);
uint32_t pci_get_device_count(void);
const pci_device_t *pci_get_device(uint32_t index);
int pci_register_driver(pci_bus_driver_t *driver);

uint32_t pci_read_bar(uint8_t bus, uint8_t device, uint8_t func, uint8_t bar_index);
bool pci_get_bar_info(uint8_t bus,
                      uint8_t device,
                      uint8_t func,
                      uint8_t bar_index,
                      pci_bar_info_t *out_bar);
int32_t pci_find_capability(uint8_t bus,
                            uint8_t device,
                            uint8_t func,
                            uint8_t capability_id);

#ifdef __cplusplus
}
#endif

#endif
