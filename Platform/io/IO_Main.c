#include "IO_Main.h"
#include "Protocol/ATA/Protocol_ATA.h"
#include "Protocol/USB_MassStorage/Protocol_USB_MassStorage.h"
#include "Drivers/Module/BlockManager.h"
#include "Core/sync/Spinlock.h"
#include "Debug/serial/Serial.h"
#include "Drivers/Module/DriverManager.h"
#include "Core/timer/Timer.h"
#include <stddef.h>
#include <string.h>

static bool module_block_init(uint64_t partition_lba)
{
    (void)partition_lba;
    return block_manager_init();
}

static bool module_block_read(uint64_t lba, uint8_t *buffer, uint32_t sectors)
{
    return block_manager_read_sectors(lba, buffer, sectors);
}

static bool module_block_write(uint64_t lba, const uint8_t *buffer,
                               uint32_t sectors)
{
    return block_manager_write_sectors(lba, buffer, sectors);
}

static bool module_block_is_working(void)
{
    return block_manager_get_device_count() != 0u;
}

static uint32_t module_block_get_device_count(void)
{
    return block_manager_get_device_count();
}

static bool module_block_select_device(uint32_t index)
{
    return block_manager_select_device(index);
}

static uint64_t module_block_get_total_bytes(void)
{
    driver_block_info_t info;
    if (!block_manager_get_info(block_manager_selected_device(), &info)) {
        return 0u;
    }
    return info.block_count * info.logical_block_size;
}

static const block_device_t g_block_devices[] = {
    { "block",  "Modular block device",  IO_PROTOCOL_TYPE_NONE,
      module_block_init, module_block_read, module_block_write,
      module_block_is_working, module_block_get_device_count,
      module_block_select_device, module_block_get_total_bytes },
    { "usb_ms", "USB Mass Storage",      IO_PROTOCOL_TYPE_USB_MASS_STORAGE,
      usb_ms_init, usb_ms_read, usb_ms_write,
      usb_ms_is_working, usb_ms_get_device_count,
      usb_ms_select_device, usb_ms_get_total_bytes },
    { "ata",    "ATA disk",              IO_PROTOCOL_TYPE_ATA,
      ata_init, ata_read, ata_write, ata_is_working,
      ata_get_device_count, ata_select_device, ata_get_total_bytes },
};

#define IO_MAX_DISKS 16

static const block_device_t *g_current_block_device = NULL;
static io_protocol_type_t    g_current_protocol     = IO_PROTOCOL_TYPE_NONE;
static uint32_t              g_current_device_index = 0;
static uint64_t              g_partition_lba        = 0;
static const block_device_t *g_detected_disks[IO_MAX_DISKS];
static uint32_t              g_detected_disks_indices[IO_MAX_DISKS];
static uint32_t              g_detected_disk_count  = 0;
static bool                  g_disk_scan_done       = false;
static spinlock_t            g_disk_lock;



static bool detected_disk_append(const block_device_t *device, uint32_t dev_index) {
    if (!device || g_detected_disk_count >= IO_MAX_DISKS) return false;
    g_detected_disks[g_detected_disk_count]         = device;
    g_detected_disks_indices[g_detected_disk_count] = dev_index;
    g_detected_disk_count++;
    return true;
}

static uint32_t block_device_detected_count(const block_device_t *device) {
    if (!device) return 0;

    uint32_t dev_count = 1;
    if (device->get_device_count)
        dev_count = device->get_device_count();

    if (dev_count == 0 && device->is_working && device->is_working())
        dev_count = 1;

    return dev_count;
}

static void copy_string(char *dst, uint32_t dst_size, const char *src) {
    if (!dst || dst_size == 0) return;
    uint32_t i = 0;
    if (src) {
        for (; i + 1 < dst_size && src[i]; ++i)
            dst[i] = src[i];
    }
    dst[i] = '\0';
}

static io_protocol_type_t block_transport_to_io(
        driver_block_transport_t transport)
{
    switch (transport) {
        case DRIVER_BLOCK_TRANSPORT_ATA:
            return IO_PROTOCOL_TYPE_ATA;
        case DRIVER_BLOCK_TRANSPORT_USB:
            return IO_PROTOCOL_TYPE_USB_MASS_STORAGE;
        case DRIVER_BLOCK_TRANSPORT_AHCI:
            return IO_PROTOCOL_TYPE_AHCI;
        case DRIVER_BLOCK_TRANSPORT_NVME:
            return IO_PROTOCOL_TYPE_NVME;
        case DRIVER_BLOCK_TRANSPORT_VIRTIO:
            return IO_PROTOCOL_TYPE_VIRTIO_BLOCK;
        default:
            return IO_PROTOCOL_TYPE_NONE;
    }
}

static const block_device_t *block_device_find_by_protocol(io_protocol_type_t protocol) {
    for (size_t i = 0; i < sizeof(g_block_devices) / sizeof(g_block_devices[0]); ++i) {
        if (g_block_devices[i].protocol == protocol)
            return &g_block_devices[i];
    }
    return NULL;
}

static uint32_t io_read_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t io_read_u64(const uint8_t *p) {
    return (uint64_t)p[0] | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) | ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}

static bool check_fat_boot_sector(const uint8_t *buffer) {
    uint16_t boot_sig = (uint16_t)buffer[510] | ((uint16_t)buffer[511] << 8);
    if (boot_sig == 0xAA55u) {
        if (memcmp(buffer + 82, "FAT32   ", 8) == 0) return true;
        if (memcmp(buffer + 54, "FAT16   ", 8) == 0) return true;
        if (memcmp(buffer + 54, "FAT12   ", 8) == 0) return true;
    }
    return false;
}

static bool check_iso_signature_at_lba(const block_device_t *device,
                                       uint64_t              partition_lba)
{
    uint8_t buffer[512];
    if (!device || !device->read) return false;

    return device->read(partition_lba + 16u * 4u, buffer, 1u) &&
           memcmp(buffer + 1, "CD001", 5) == 0;
}

static bool check_iso_signature(const block_device_t *device,
                                uint64_t              partition_lba,
                                bool                  prefer_whole_disk,
                                uint64_t             *out_partition_lba)
{
    if (!device || !device->read) return false;

    if (prefer_whole_disk &&
        check_iso_signature_at_lba(device, 0u)) {
        if (out_partition_lba != NULL) *out_partition_lba = 0u;
        return true;
    }

    if (check_iso_signature_at_lba(device, partition_lba)) {
        if (out_partition_lba != NULL) *out_partition_lba = partition_lba;
        return true;
    }

    if (partition_lba != 0u &&
        check_iso_signature_at_lba(device, 0u)) {
        if (out_partition_lba != NULL) *out_partition_lba = 0u;
        return true;
    }

    return false;
}

static bool check_fat_signature(const block_device_t *device,
                                uint64_t              partition_lba,
                                uint64_t             *out_partition_lba)
{
    uint8_t buffer[2048];
    if (!device || !device->read) return false;

    if (device->read(partition_lba, buffer, 1)) {
        if (check_fat_boot_sector(buffer)) {
            if (out_partition_lba != NULL) *out_partition_lba = partition_lba;
            return true;
        }
    }
    if (partition_lba != 0 && device->read(0, buffer, 1)) {
        if (check_fat_boot_sector(buffer)) {
            if (out_partition_lba != NULL) *out_partition_lba = 0u;
            return true;
        }
    }

    if (device->read(0, buffer, 1)) {
        uint16_t boot_sig = (uint16_t)buffer[510] | ((uint16_t)buffer[511] << 8);
        if (boot_sig == 0xAA55u) {
            bool protective_mbr = false;
            for (uint32_t i = 0u; i < 4u; ++i) {
                uint32_t offset = 446u + i * 16u;
                uint8_t type = buffer[offset + 4];
                uint32_t start_lba = io_read_u32(buffer + offset + 8);

                if (type == 0xEE) {
                    protective_mbr = true;
                    continue;
                }
                if ((type == 0x0B || type == 0x0C || type == 0x01 || type == 0x04 || type == 0x06 || type == 0x0E) && start_lba != 0) {
                    uint8_t pbuf[512];
                    if (device->read(start_lba, pbuf, 1)) {
                        if (check_fat_boot_sector(pbuf)) {
                            if (out_partition_lba != NULL) *out_partition_lba = start_lba;
                            return true;
                        }
                    }
                }
            }

            if (protective_mbr) {
                if (device->read(1, buffer, 1)) {
                    if (memcmp(buffer, "EFI PART", 8) == 0) {
                        uint32_t entries_lba = io_read_u32(buffer + 72);
                        uint32_t num_entries = io_read_u32(buffer + 80);
                        uint32_t entry_size  = io_read_u32(buffer + 84);

                        if (entry_size >= 128 && entry_size <= 4096) {
                            uint32_t entries_per_sector = 512 / entry_size;
                            if (entries_per_sector > 0) {
                                uint32_t sectors_to_read = (num_entries + entries_per_sector - 1) / entries_per_sector;
                                if (sectors_to_read > 4) sectors_to_read = 4;

                                for (uint32_t sector = 0; sector < sectors_to_read; ++sector) {
                                    if (device->read(entries_lba + sector, buffer, 1)) {
                                        for (uint32_t e = 0; e < entries_per_sector && e * entry_size + 40 <= 512; ++e) {
                                            uint8_t *entry = buffer + e * entry_size;
                                            bool is_zero = true;
                                            for (int j = 0; j < 16; j++) {
                                                if (entry[j] != 0) { is_zero = false; break; }
                                            }
                                            if (is_zero) continue;

                                            uint64_t first_lba = io_read_u64(entry + 32);
                                            if (first_lba != 0 && first_lba <= 0xFFFFFFFFULL) {
                                                uint8_t pbuf[512];
                                                if (device->read(first_lba, pbuf, 1)) {
                                                    if (check_fat_boot_sector(pbuf)) {
                                                        if (out_partition_lba != NULL) *out_partition_lba = first_lba;
                                                        return true;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}

static bool check_bootable_signature(const block_device_t *device,
                                     uint64_t              partition_lba,
                                     io_protocol_type_t    requested_protocol,
                                     uint64_t             *out_partition_lba)
{
    uint64_t effective_lba = partition_lba;
    bool prefer_whole_disk_iso =
        requested_protocol == IO_PROTOCOL_TYPE_USB_MASS_STORAGE;
    bool found = false;

    if (check_iso_signature(device, partition_lba, prefer_whole_disk_iso,
                            &effective_lba) ||
        check_fat_signature(device, partition_lba, &effective_lba)) {
        if (out_partition_lba != NULL) {
            *out_partition_lba = effective_lba;
        }
        found = true;
    }
    return found;
}

static const block_device_t *block_device_probe_one(
        const block_device_t *device,
        uint64_t              partition_lba,
        io_protocol_type_t    requested_protocol,
        uint32_t             *out_device_index,
        uint64_t             *out_partition_lba)
{
    const block_device_t *saved_device    = g_current_block_device;
    io_protocol_type_t    saved_protocol  = g_current_protocol;
    uint64_t              saved_lba       = g_partition_lba;
    uint32_t              saved_dev_index = g_current_device_index;

    if (!device || !device->init) {
        return NULL;
    }
    if (!device->init(0)) {
        return NULL;
    }

    uint32_t dev_count = 1;
    if (device->get_device_count)
        dev_count = device->get_device_count();

    for (uint32_t d = 0; d < dev_count; ++d) {
        if (device->select_device)
            device->select_device(d);

        uint64_t effective_lba = partition_lba;
        if (check_bootable_signature(device, partition_lba,
                                     requested_protocol, &effective_lba)) {
            if (out_device_index) *out_device_index = d;
            if (out_partition_lba) *out_partition_lba = effective_lba;
            if (saved_device) {
                g_current_block_device = saved_device;
                g_current_protocol     = saved_protocol;
                g_partition_lba        = saved_lba;
                g_current_device_index = saved_dev_index;
                if (saved_device->select_device)
                    saved_device->select_device(saved_dev_index);
            }
            return device;
        }
    }

    if (saved_device) {
        g_current_block_device = saved_device;
        g_current_protocol     = saved_protocol;
        g_partition_lba        = saved_lba;
        g_current_device_index = saved_dev_index;
        if (saved_device->select_device)
            saved_device->select_device(saved_dev_index);
    }
    return NULL;
}

static const block_device_t *block_device_select_probe(
        uint64_t  partition_lba,
        io_protocol_type_t requested_protocol,
        uint32_t *out_device_index,
        uint64_t *out_partition_lba)
{
    for (size_t i = 0; i < sizeof(g_block_devices) / sizeof(g_block_devices[0]); ++i) {
        if (requested_protocol != IO_PROTOCOL_TYPE_NONE) {
            if (g_block_devices[i].protocol == IO_PROTOCOL_TYPE_NONE ||
                g_block_devices[i].protocol != requested_protocol) {
                continue;
            }
        }
        const block_device_t *found =
            block_device_probe_one(&g_block_devices[i], partition_lba,
                                   requested_protocol, out_device_index,
                                   out_partition_lba);
        if (found) return found;
    }
    return NULL;
}

static bool module_block_find_boot(uint32_t *out_device_index)
{
    uint32_t count = block_manager_get_device_count();
    for (uint32_t i = 0u; i < count; ++i) {
        driver_block_info_t info;
        if (block_manager_get_info(i, &info) &&
            (info.flags & DRIVER_BLOCK_FLAG_BOOT) != 0u) {
            if (out_device_index != NULL) {
                *out_device_index = i;
            }
            return true;
        }
    }
    return false;
}

static bool module_block_probe_transport(
        io_protocol_type_t requested_protocol,
        uint64_t           partition_lba,
        uint32_t          *out_device_index,
        uint64_t          *out_partition_lba)
{
    uint32_t count = block_manager_get_device_count();
    for (uint32_t i = 0u; i < count; ++i) {
        driver_block_info_t info;
        if (!block_manager_get_info(i, &info) ||
            block_transport_to_io(info.transport) != requested_protocol ||
            !block_manager_select_device(i)) {
            continue;
        }
        uint64_t effective_lba = partition_lba;
        if (check_bootable_signature(&g_block_devices[0], partition_lba,
                                     requested_protocol, &effective_lba)) {
            if (out_device_index != NULL) {
                *out_device_index = i;
            }
            if (out_partition_lba != NULL) {
                *out_partition_lba = effective_lba;
            }
            return true;
        }
        if (requested_protocol == IO_PROTOCOL_TYPE_USB_MASS_STORAGE) {
            if (out_device_index != NULL) {
                *out_device_index = i;
            }
            if (out_partition_lba != NULL) {
                *out_partition_lba = 0u;
            }
            return true;
        }
    }
    return false;
}

static const block_device_t *block_device_select(
        io_protocol_type_t requested_protocol,
        uint64_t           partition_lba,
        uint32_t          *out_device_index,
        uint64_t          *out_partition_lba)
{
    const block_device_t *module_device = &g_block_devices[0];
    if (module_device->init != NULL && module_device->init(0u)) {
        uint32_t boot_index = 0u;
        if (module_block_find_boot(&boot_index)) {
            uint64_t effective_lba = partition_lba;
            if (!block_manager_select_device(boot_index)) {
                return NULL;
            }

            if (!check_bootable_signature(module_device, partition_lba,
                                          requested_protocol,
                                          &effective_lba)) {
                if (requested_protocol == IO_PROTOCOL_TYPE_USB_MASS_STORAGE) {
                    effective_lba = 0u;
                } else {
                    return NULL;
                }
            }
            if (out_device_index != NULL) {
                *out_device_index = boot_index;
            }
            if (out_partition_lba != NULL) {
                *out_partition_lba = effective_lba;
            }
            return module_device;
        }

        if (requested_protocol != IO_PROTOCOL_TYPE_NONE &&
            module_block_probe_transport(requested_protocol, partition_lba,
                                         out_device_index,
                                         out_partition_lba)) {
            return module_device;
        }
    }

    if (requested_protocol != IO_PROTOCOL_TYPE_NONE) {
        const block_device_t *device = block_device_find_by_protocol(requested_protocol);
        if (device != NULL && device != module_device) {
            const block_device_t *found =
                block_device_probe_one(device, partition_lba,
                                       requested_protocol, out_device_index,
                                       out_partition_lba);
            if (found) {
                return found;
            }
        }
    }

    const block_device_t *result =
        block_device_select_probe(partition_lba, requested_protocol,
                                  out_device_index, out_partition_lba);
    return result;
}

static void apply_boot_device(const block_device_t *device,
                               uint32_t              dev_idx,
                               uint64_t              partition_lba)
{
    g_current_block_device = device;
    g_current_protocol     = device->protocol;
    g_current_device_index = dev_idx;
    g_partition_lba        = partition_lba;
    if (device->select_device)
        device->select_device(dev_idx);
}

bool disk_io_init(uint64_t partition_lba, uint32_t boot_drive_type) {
    g_current_protocol     = IO_PROTOCOL_TYPE_NONE;
    g_partition_lba        = partition_lba;
    g_current_block_device = NULL;
    g_current_device_index = 0;
    g_disk_scan_done       = false;
    spinlock_init(&g_disk_lock);

    io_protocol_type_t requested_protocol = IO_PROTOCOL_TYPE_NONE;
    if      (boot_drive_type == BOOT_DRIVE_TYPE_IDE)
        requested_protocol = IO_PROTOCOL_TYPE_ATA;
    else if (boot_drive_type == BOOT_DRIVE_TYPE_USB)
        requested_protocol = IO_PROTOCOL_TYPE_USB_MASS_STORAGE;
    else if (boot_drive_type == BOOT_DRIVE_TYPE_AHCI)
        requested_protocol = IO_PROTOCOL_TYPE_AHCI;
    else if (boot_drive_type == BOOT_DRIVE_TYPE_NVME)
        requested_protocol = IO_PROTOCOL_TYPE_NVME;
    else if (boot_drive_type == BOOT_DRIVE_TYPE_VIRTIO)
        requested_protocol = IO_PROTOCOL_TYPE_VIRTIO_BLOCK;

    uint32_t found_index = 0;
    uint64_t effective_lba = partition_lba;
    const block_device_t *device =
        block_device_select(requested_protocol, partition_lba, &found_index,
                            &effective_lba);

    if (device) {
        apply_boot_device(device, found_index, effective_lba);

        if (device->select_device)
            device->select_device(found_index);
        return true;
    }

    return false;
}


bool disk_read(uint64_t lba, uint8_t *buffer, uint32_t sectors)
{
    if (!g_current_block_device) return false;
    spinlock_lock(&g_disk_lock);
    if (g_current_block_device->select_device)
        g_current_block_device->select_device(g_current_device_index);
    bool ok = false;
    if (g_current_block_device->read) {
        ok = g_current_block_device->read(lba, buffer, sectors);
    }
    spinlock_unlock(&g_disk_lock);
    return ok;
}

bool disk_write(uint64_t lba, const uint8_t *buffer, uint32_t sectors) {
    if (!g_current_block_device) return false;
    spinlock_lock(&g_disk_lock);
    bool ok = false;
    if (g_current_block_device->select_device)
        g_current_block_device->select_device(g_current_device_index);
    if (g_current_block_device->write)
        ok = g_current_block_device->write(lba, buffer, sectors);
    spinlock_unlock(&g_disk_lock);
    return ok;
}

bool disk_io_is_working(void) {
    if (g_current_block_device && g_current_block_device->is_working)
        return g_current_block_device->is_working();
    return false;
}

io_protocol_type_t disk_io_get_protocol(void) {
    return g_current_protocol;
}

uint64_t disk_get_partition_lba(void) {
    return g_partition_lba;
}


static void disk_scan_devices(void) {
    if (g_disk_scan_done) return;

    g_detected_disk_count = 0;

    for (size_t i = 0; i < sizeof(g_block_devices) / sizeof(g_block_devices[0]); ++i) {
        const block_device_t *device = &g_block_devices[i];

        if (device == g_current_block_device) {
            uint32_t dev_count = block_device_detected_count(device);

            for (uint32_t d = 0; d < dev_count; ++d) {
                if (!detected_disk_append(device, d)) break;
            }
            continue;
        }

        if (!device->init || !device->init(0)) continue;
        if (device->is_working && !device->is_working()) continue;

        uint32_t dev_count = block_device_detected_count(device);

        for (uint32_t d = 0; d < dev_count; ++d) {
            if (!detected_disk_append(device, d)) break;
        }
    }

    if (g_current_block_device) {
        if (g_current_block_device->select_device)
            g_current_block_device->select_device(g_current_device_index);
    }

    g_disk_scan_done = true;
}

uint32_t disk_get_count(void) {
    disk_scan_devices();
    return g_detected_disk_count;
}

bool disk_get_info(uint32_t index, io_disk_info_t *out_info) {
    if (!out_info) return false;
    disk_scan_devices();
    if (index >= g_detected_disk_count) return false;

    const block_device_t *device  = g_detected_disks[index];
    uint32_t              dev_idx = g_detected_disks_indices[index];
    memset(out_info, 0, sizeof(*out_info));

    if (device->select_device)
        device->select_device(dev_idx);

    copy_string(out_info->disk_name, sizeof(out_info->disk_name), device->name);
    uint32_t len = 0;
    while (out_info->disk_name[len]) len++;
    if (len < sizeof(out_info->disk_name) - 1) {
        out_info->disk_name[len]   = (char)('0' + dev_idx);
        out_info->disk_name[len+1] = '\0';
    }

    copy_string(out_info->manufacturer, sizeof(out_info->manufacturer), "ImplusOS");
    if (device == &g_block_devices[0]) {
        driver_block_info_t info;
        if (!block_manager_get_info(dev_idx, &info)) {
            return false;
        }
        copy_string(out_info->model, sizeof(out_info->model), info.model);
        out_info->protocol = block_transport_to_io(info.transport);
        out_info->total_bytes =
            info.block_count * (uint64_t)info.logical_block_size;
        out_info->sector_size = info.logical_block_size;
        if ((info.flags & DRIVER_BLOCK_FLAG_BOOT) != 0u) {
            out_info->flags |= IO_DISK_FLAG_BOOT;
        }
        if ((info.flags & DRIVER_BLOCK_FLAG_WRITABLE) != 0u) {
            out_info->flags |= IO_DISK_FLAG_WRITABLE;
        }
    } else {
        copy_string(out_info->model, sizeof(out_info->model), device->model);
        out_info->protocol = device->protocol;
        out_info->total_bytes =
            device->get_total_bytes ? device->get_total_bytes() : 0u;
        out_info->sector_size = 512u;
        if (device->write) {
            out_info->flags |= IO_DISK_FLAG_WRITABLE;
        }
    }

    if (device == g_current_block_device && dev_idx == g_current_device_index)
        out_info->flags |= IO_DISK_FLAG_BOOT;

    if (g_current_block_device && g_current_block_device->select_device)
        g_current_block_device->select_device(g_current_device_index);

    return true;
}

static bool disk_raw_io(uint32_t       index,
                        uint64_t       lba,
                        uint8_t       *read_buffer,
                        const uint8_t *write_buffer,
                        uint32_t       sectors)
{
    if (sectors == 0)                                    return true;
    if ((read_buffer == NULL) == (write_buffer == NULL)) return false;

    disk_scan_devices();
    if (index >= g_detected_disk_count) return false;

    const block_device_t *saved_device    = g_current_block_device;
    io_protocol_type_t    saved_protocol  = g_current_protocol;
    uint64_t              saved_lba       = g_partition_lba;
    uint32_t              saved_dev_index = g_current_device_index;

    const block_device_t *device  = g_detected_disks[index];
    uint32_t              dev_idx = g_detected_disks_indices[index];
    bool ok = false;

    if (device->select_device)
        device->select_device(dev_idx);

    bool already_up = device->is_working && device->is_working();
    if (already_up || (device->init && device->init(0))) {
        if (read_buffer  && device->read)  ok = device->read (lba, read_buffer,  sectors);
        else if (write_buffer && device->write) ok = device->write(lba, write_buffer, sectors);
    }

    if (saved_device) {
        g_current_block_device = saved_device;
        g_current_protocol     = saved_protocol;
        g_partition_lba        = saved_lba;
        g_current_device_index = saved_dev_index;
        if (saved_device->select_device)
            saved_device->select_device(saved_dev_index);
    }

    return ok;
}

bool disk_raw_read(uint32_t index, uint64_t lba, uint8_t *buffer, uint32_t sectors) {
    return disk_raw_io(index, lba, buffer, NULL, sectors);
}

bool disk_raw_write(uint32_t index, uint64_t lba, const uint8_t *buffer, uint32_t sectors) {
    return disk_raw_io(index, lba, NULL, buffer, sectors);
}
