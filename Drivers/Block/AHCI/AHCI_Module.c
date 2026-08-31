#include "Drivers/Module/DriverBinary.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static const driver_binary_t *g_module_api;

static bool ahci_module_dma_init(void);
static void *ahci_module_dma_alloc(size_t bytes, uint64_t *phys);
static void ahci_module_dma_free(void *ptr, size_t bytes);
static uint32_t ahci_module_pci_read(uint8_t bus, uint8_t device,
                                     uint8_t function, uint8_t offset);
static void ahci_module_pci_write(uint8_t bus, uint8_t device,
                                  uint8_t function, uint8_t offset,
                                  uint32_t value);
static void *ahci_module_map(uint64_t phys);
static void *ahci_module_memset(void *dst, int value, size_t bytes);
static void *ahci_module_memcpy(void *dst, const void *src, size_t bytes);

#define dma_init        ahci_module_dma_init
#define dma_alloc       ahci_module_dma_alloc
#define dma_free        ahci_module_dma_free
#define pci_read_config ahci_module_pci_read
#define pci_write_config ahci_module_pci_write
#define map_mmio_virt   ahci_module_map
#define memset          ahci_module_memset
#define memcpy          ahci_module_memcpy

#include "../../../Platform/io/Protocol/AHCI/Protocol_AHCI.c"

#undef dma_init
#undef dma_alloc
#undef dma_free
#undef pci_read_config
#undef pci_write_config
#undef map_mmio_virt
#undef memset
#undef memcpy

static bool ahci_module_dma_init(void)
{
    return true;
}

static void *ahci_module_dma_alloc(size_t bytes, uint64_t *phys)
{
    return g_module_api->mem.dma_alloc_ex(bytes, 4096u, 0u, phys);
}

static void ahci_module_dma_free(void *ptr, size_t bytes)
{
    g_module_api->mem.dma_free(ptr, bytes);
}

static uint32_t ahci_module_pci_read(uint8_t bus, uint8_t device,
                                     uint8_t function, uint8_t offset)
{
    return g_module_api->pci.read_config(bus, device, function, offset);
}

static void ahci_module_pci_write(uint8_t bus, uint8_t device,
                                  uint8_t function, uint8_t offset,
                                  uint32_t value)
{
    g_module_api->pci.write_config(bus, device, function, offset, value);
}

static void *ahci_module_map(uint64_t phys)
{
    return g_module_api->hw.map_mmio_range(phys, 8192u);
}

static void *ahci_module_memset(void *dst, int value, size_t bytes)
{
    return g_module_api->mem.memset(dst, value, bytes);
}

static void *ahci_module_memcpy(void *dst, const void *src, size_t bytes)
{
    return g_module_api->mem.memcpy(dst, src, bytes);
}

static bool module_init(void)
{
    return ahci_init(0u);
}

static bool module_get_info(uint32_t index, driver_block_info_t *out)
{
    if (out == NULL || !ahci_select_device(index)) {
        return false;
    }
    uint64_t bytes = ahci_get_total_bytes();
    if (bytes < 512u) {
        return false;
    }
    g_module_api->mem.memset(out, 0, sizeof(*out));
    out->block_count = bytes / 512u;
    out->logical_block_size = 512u;
    out->physical_block_size = g_devices[index].atapi ? 2048u : 512u;
    out->flags = g_devices[index].atapi ? 0u : DRIVER_BLOCK_FLAG_WRITABLE;
    out->transport = DRIVER_BLOCK_TRANSPORT_AHCI;
    out->identity_flags = DRIVER_BLOCK_IDENTITY_PCI_VALID;
    out->pci_segment = 0u;
    out->pci_bus = g_controller_bus;
    out->pci_device = g_controller_device;
    out->pci_function = g_controller_function;
    out->controller_port = (uint16_t)g_devices[index].port;
    const char model[] = "AHCI SATA/ATAPI device";
    g_module_api->mem.memcpy(out->model, model, sizeof(model));
    return true;
}

static bool module_read(uint32_t index, uint64_t lba, void *buffer,
                        uint32_t count)
{
    return ahci_select_device(index) &&
           ahci_read(lba, (uint8_t *)buffer, count);
}

static bool module_write(uint32_t index, uint64_t lba, const void *buffer,
                         uint32_t count)
{
    return ahci_select_device(index) &&
           ahci_write(lba, (const uint8_t *)buffer, count);
}

static bool module_flush(uint32_t index)
{
    return ahci_select_device(index) && ahci_flush();
}

static const driver_storage_t g_storage = {
    .name = "ahci",
    .priority = 30u,
    .init = module_init,
    .is_ready = ahci_is_working,
    .get_device_count = ahci_get_device_count,
    .get_info = module_get_info,
    .read_blocks = module_read,
    .write_blocks = module_write,
    .flush = module_flush,
};

static void module_shutdown(void)
{
    ahci_free_dma_buffers();
    g_working = false;
}

static const driver_module_descriptor_t g_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_BLOCK,
    .load_priority = 42u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_storage,
    .shutdown = module_shutdown,
};

__attribute__((visibility("default")))
const driver_module_descriptor_t *driver_module_init(
    const driver_binary_t *api)
{
    if (api == NULL || api->version_major != DRIVER_API_VERSION_MAJOR ||
        api->pci.read_config == NULL || api->mem.dma_alloc_ex == NULL ||
        api->hw.map_mmio_range == NULL) {
        return NULL;
    }
    g_module_api = api;
    return &g_module;
}
