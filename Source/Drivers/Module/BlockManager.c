#include "BlockManager.h"

#include "DeviceRegistry.h"
#include "MemoryManagement/DMA_Memory.h"
#include "Debug/serial/Serial.h"
#include "Core/sync/Spinlock.h"

#include <stddef.h>
#include <string.h>

#define BLOCK_MANAGER_MAX_PROVIDERS 16u
#define BLOCK_MANAGER_MAX_DEVICES   64u

typedef struct {
    const driver_storage_t *storage;
    const char *module_name;
} block_provider_t;

typedef struct {
    const driver_storage_t *storage;
    const char *module_name;
    uint32_t local_index;
    driver_block_info_t info;
} block_device_entry_t;

static block_provider_t g_providers[BLOCK_MANAGER_MAX_PROVIDERS];
static block_device_entry_t g_devices[BLOCK_MANAGER_MAX_DEVICES];
static uint32_t g_provider_count;
static uint32_t g_device_count;
static uint32_t g_selected;
static bool g_initialized;
static BOOT_INFO g_boot_info;
static bool g_have_boot_identity;
static spinlock_t g_block_lock;

void block_manager_set_boot_identity(const BOOT_INFO *boot_info)
{
    g_have_boot_identity =
        boot_info != NULL &&
        boot_info->BootStorageIdentityVersion ==
            BOOT_STORAGE_IDENTITY_VERSION;
    if (g_have_boot_identity) {
        g_boot_info = *boot_info;
    }
    g_initialized = false;
}

static bool block_transport_matches_boot(const block_device_entry_t *entry)
{
    driver_block_transport_t boot_transport =
        (driver_block_transport_t)g_boot_info.BootStorageTransport;

    if (entry->info.transport == boot_transport) {
        return true;
    }

    /*
     * Some UEFI implementations report the second-stage image through a
     * controller-level path even though the first-stage loader booted from
     * removable USB media. In that case prefer the explicit boot-drive hint
     * and avoid pinning the kernel to the wrong storage transport.
     */
    if (g_boot_info.BootDriveType == BOOT_DRIVE_TYPE_USB &&
        entry->info.transport == DRIVER_BLOCK_TRANSPORT_USB) {
        return true;
    }

    return false;
}

static bool block_device_matches_boot(const block_device_entry_t *entry)
{
    if (!g_have_boot_identity || entry == NULL) {
        return false;
    }

    bool usb_boot_hint =
        g_boot_info.BootDriveType == BOOT_DRIVE_TYPE_USB &&
        entry->info.transport == DRIVER_BLOCK_TRANSPORT_USB &&
        entry->info.transport !=
            (driver_block_transport_t)g_boot_info.BootStorageTransport;

    if (!block_transport_matches_boot(entry)) {
        return false;
    }

    if (!usb_boot_hint &&
        (g_boot_info.BootStorageIdentityFlags &
         BOOT_STORAGE_FLAG_PCI_VALID) != 0u) {
        if ((entry->info.identity_flags &
             DRIVER_BLOCK_IDENTITY_PCI_VALID) == 0u) {
            return false;
        }
        if (entry->info.pci_segment != g_boot_info.BootStoragePciSegment ||
            entry->info.pci_bus != g_boot_info.BootStoragePciBus ||
            entry->info.pci_device != g_boot_info.BootStoragePciDevice ||
            entry->info.pci_function != g_boot_info.BootStoragePciFunction) {
            return false;
        }
    }

    if (entry->info.transport == DRIVER_BLOCK_TRANSPORT_NVME &&
        g_boot_info.BootStorageNamespace != 0u &&
        entry->info.namespace_id != g_boot_info.BootStorageNamespace) {
        return false;
    }
    if (entry->info.transport == DRIVER_BLOCK_TRANSPORT_AHCI &&
        g_boot_info.BootStoragePort != UINT16_MAX &&
        entry->info.controller_port != g_boot_info.BootStoragePort) {
        return false;
    }

    return true;
}

static void block_provider_add(const driver_storage_t *storage,
                               const char *module_name)
{
    if (storage == NULL || storage->get_device_count == NULL ||
        storage->get_info == NULL || storage->read_blocks == NULL ||
        g_provider_count >= BLOCK_MANAGER_MAX_PROVIDERS) {
        return;
    }
    g_providers[g_provider_count].storage = storage;
    g_providers[g_provider_count].module_name = module_name;
    ++g_provider_count;
}

static void block_provider_sort(void)
{
    for (uint32_t i = 1u; i < g_provider_count; ++i) {
        block_provider_t key = g_providers[i];
        uint32_t j = i;
        while (j > 0u &&
               g_providers[j - 1u].storage->priority >
                   key.storage->priority) {
            g_providers[j] = g_providers[j - 1u];
            --j;
        }
        g_providers[j] = key;
    }
}

bool block_manager_init(void)
{
    if (g_initialized) {
        return g_device_count != 0u;
    }

    g_provider_count = 0u;
    g_device_count = 0u;
    g_selected = 0u;
    spinlock_init(&g_block_lock);

    for (uint32_t i = 0u;; ++i) {
        const device_t *device =
            device_registry_find_by_index(DEVICE_TYPE_BLOCK, i);
        if (device == NULL) {
            break;
        }
        block_provider_add((const driver_storage_t *)device->ops,
                           device->name);
    }

    const device_t *usb = device_registry_find(DEVICE_TYPE_USB, NULL);
    if (usb != NULL) {
        const usb_master_vtable_t *vtable =
            (const usb_master_vtable_t *)usb->ops;
        block_provider_add(&vtable->storage, usb->name);
    }

    block_provider_sort();
    for (uint32_t p = 0u; p < g_provider_count; ++p) {
        const driver_storage_t *storage = g_providers[p].storage;
        if (storage->init != NULL && !storage->init()) {
            continue;
        }
        if (storage->is_ready != NULL && !storage->is_ready()) {
            continue;
        }
        uint32_t count = storage->get_device_count();
        for (uint32_t d = 0u;
             d < count && g_device_count < BLOCK_MANAGER_MAX_DEVICES; ++d) {
            block_device_entry_t *entry = &g_devices[g_device_count];
            memset(entry, 0, sizeof(*entry));
            if (!storage->get_info(d, &entry->info) ||
                entry->info.logical_block_size == 0u ||
                entry->info.block_count == 0u) {
                continue;
            }
            entry->info.flags &= ~DRIVER_BLOCK_FLAG_BOOT;
            entry->storage = storage;
            entry->module_name = g_providers[p].module_name;
            entry->local_index = d;
            ++g_device_count;
        }
    }
    for (uint32_t i = 0u; i < g_device_count; ++i) {
        if (block_device_matches_boot(&g_devices[i])) {
            block_device_entry_t boot = g_devices[i];
            g_devices[i] = g_devices[0];
            g_devices[0] = boot;
            g_devices[0].info.flags |= DRIVER_BLOCK_FLAG_BOOT;
            break;
        }
    }
    g_initialized = true;
    return g_device_count != 0u;
}

static bool block_manager_ensure_init(void)
{
    return g_initialized ? g_device_count != 0u : block_manager_init();
}

uint32_t block_manager_get_device_count(void)
{
    (void)block_manager_ensure_init();
    return g_device_count;
}

bool block_manager_select_device(uint32_t index)
{
    if (!block_manager_ensure_init() || index >= g_device_count) {
        return false;
    }
    spinlock_lock(&g_block_lock);
    g_selected = index;
    spinlock_unlock(&g_block_lock);
    return true;
}

uint32_t block_manager_selected_device(void)
{
    return g_selected;
}

bool block_manager_get_info(uint32_t index, driver_block_info_t *out_info)
{
    if (!block_manager_ensure_init() || out_info == NULL ||
        index >= g_device_count) {
        return false;
    }
    *out_info = g_devices[index].info;
    return true;
}

const char *block_manager_get_name(uint32_t index)
{
    if (!block_manager_ensure_init() || index >= g_device_count) {
        return NULL;
    }
    const char *name = g_devices[index].storage->name;
    return name != NULL ? name : g_devices[index].module_name;
}

bool block_manager_read_blocks(uint32_t index, uint64_t lba, void *buffer,
                               uint32_t block_count)
{
    if (!block_manager_ensure_init() || index >= g_device_count ||
        (block_count != 0u && buffer == NULL)) {
        return false;
    }
    block_device_entry_t *entry = &g_devices[index];
    if (lba > entry->info.block_count ||
        block_count > entry->info.block_count - lba) {
        return false;
    }
    return block_count == 0u ||
           entry->storage->read_blocks(entry->local_index, lba, buffer,
                                       block_count);
}

bool block_manager_write_blocks(uint32_t index, uint64_t lba,
                                const void *buffer, uint32_t block_count)
{
    if (!block_manager_ensure_init() || index >= g_device_count ||
        (block_count != 0u && buffer == NULL)) {
        return false;
    }
    block_device_entry_t *entry = &g_devices[index];
    if ((entry->info.flags & DRIVER_BLOCK_FLAG_WRITABLE) == 0u ||
        entry->storage->write_blocks == NULL ||
        lba > entry->info.block_count ||
        block_count > entry->info.block_count - lba) {
        return false;
    }
    return block_count == 0u ||
           entry->storage->write_blocks(entry->local_index, lba, buffer,
                                        block_count);
}

bool block_manager_flush(uint32_t index)
{
    if (!block_manager_ensure_init() || index >= g_device_count) {
        return false;
    }
    const block_device_entry_t *entry = &g_devices[index];
    return entry->storage->flush == NULL ||
           entry->storage->flush(entry->local_index);
}

static bool block_manager_sector_io_unlocked(uint32_t index,
                                             uint64_t sector_lba,
                                             uint8_t *read_buffer,
                                             const uint8_t *write_buffer,
                                             uint32_t sector_count)
{
    driver_block_info_t info;
    if (!block_manager_get_info(index, &info) ||
        info.logical_block_size < 512u ||
        (info.logical_block_size % 512u) != 0u) {
        return false;
    }
    uint32_t factor = info.logical_block_size / 512u;
    if (factor == 1u) {
        if (read_buffer != NULL) {
            return block_manager_read_blocks(index, sector_lba, read_buffer,
                                             sector_count);
        }
        return block_manager_write_blocks(index, sector_lba, write_buffer,
                                          sector_count);
    }

    uint8_t *bounce = NULL;
    bool ok = true;
    for (uint32_t done = 0u; done < sector_count;) {
        uint64_t sector = sector_lba + done;
        uint64_t block = sector / factor;
        uint32_t offset_sectors = (uint32_t)(sector % factor);
        uint32_t remaining = sector_count - done;

        if (offset_sectors == 0u && remaining >= factor) {
            uint32_t blocks = remaining / factor;
            if (read_buffer != NULL) {
                ok = block_manager_read_blocks(
                    index,
                    block,
                    read_buffer + (size_t)done * 512u,
                    blocks);
            } else {
                ok = block_manager_write_blocks(
                    index,
                    block,
                    write_buffer + (size_t)done * 512u,
                    blocks);
            }
            if (!ok) {
                break;
            }
            done += blocks * factor;
            continue;
        }

        if (bounce == NULL) {
            bounce = (uint8_t *)dma_alloc(info.logical_block_size, NULL);
            if (bounce == NULL) {
                return false;
            }
        }

        uint32_t chunk = factor - offset_sectors;
        if (chunk > remaining) {
            chunk = remaining;
        }
        if (!block_manager_read_blocks(index, block, bounce, 1u)) {
            ok = false;
            break;
        }
        size_t byte_offset = (size_t)offset_sectors * 512u;
        size_t byte_count = (size_t)chunk * 512u;
        if (read_buffer != NULL) {
            memcpy(read_buffer + (size_t)done * 512u,
                   bounce + byte_offset, byte_count);
        } else {
            memcpy(bounce + byte_offset,
                   write_buffer + (size_t)done * 512u, byte_count);
            if (!block_manager_write_blocks(index, block, bounce, 1u)) {
                ok = false;
                break;
            }
        }
        done += chunk;
    }
    if (bounce != NULL) {
        dma_free(bounce, info.logical_block_size);
    }
    return ok;
}

static bool block_manager_sector_io(uint32_t index, uint64_t sector_lba,
                                    uint8_t *read_buffer,
                                    const uint8_t *write_buffer,
                                    uint32_t sector_count)
{
    spinlock_lock(&g_block_lock);
    bool ok = block_manager_sector_io_unlocked(
        index, sector_lba, read_buffer, write_buffer, sector_count);
    spinlock_unlock(&g_block_lock);
    return ok;
}

bool block_manager_read_sectors(uint64_t lba, uint8_t *buffer,
                                uint32_t sectors)
{
    return block_manager_sector_io(g_selected, lba, buffer, NULL, sectors);
}

bool block_manager_write_sectors(uint64_t lba, const uint8_t *buffer,
                                 uint32_t sectors)
{
    return block_manager_sector_io(g_selected, lba, NULL, buffer, sectors);
}
