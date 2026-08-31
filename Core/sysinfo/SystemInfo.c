#include "SystemInfo.h"
#include "Drivers/Module/PCI_Main.h"
#include "Drivers/Module/DriverManager.h"
#include "Platform/io/IO_Main.h"
#include "kernel/config.h"
#include "Core/sync/Spinlock.h"
#include <stddef.h>
#include <stdint.h>

extern uint64_t get_total_memory_pages(void);
extern uint64_t get_used_memory(void);
extern uint64_t get_free_memory(void);

typedef struct {
    uint16_t vendor_id;
    const char *vendor_name;
} pci_vendor_entry_t;

typedef struct {
    uint16_t vendor_id;
    uint16_t device_id;
    const char *device_name;
} pci_device_entry_t;

static const pci_vendor_entry_t g_pci_vendors[] = {
    {0x8086, "Intel"},
    {0x10DE, "NVIDIA"},
    {0x1022, "AMD"},
    {0x1002, "AMD"},
    {0x1414, "Microsoft"},
    {0x1B36, "Red Hat"},
    {0x1AF4, "VirtIO"},
    {0x0000, NULL}
};

static const pci_device_entry_t g_pci_devices[] = {
    {0x8086, 0x29C0, "Intel ICH9 ATA Controller"},
    {0x8086, 0x7A04, "Intel L700/M700 USB Host Controller"},
    {0x1022, 0x7902, "AMD FCH SATA Controller"},
    {0x1AF4, 0x1000, "VirtIO Network Device"},
    {0x1AF4, 0x1001, "VirtIO Block Device"},
    {0x1AF4, 0x1002, "VirtIO Console"},
    {0x1AF4, 0x1004, "VirtIO SCSI Device"},
    {0x1AF4, 0x1009, "VirtIO SCSI Device (transitional)"},
    {0xFFFF, 0xFFFF, NULL}
};

static pci_device_t g_pci_devices_list[256];
static uint32_t g_pci_device_count = 0;
static spinlock_t g_pci_scan_lock = {0};

static void sysinfo_scan_pci_devices(void)
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_pci_scan_lock);
    
    if (g_pci_device_count == 0) {
        g_pci_device_count = 0;
        
        for (uint16_t bus = 0; bus < 256 && g_pci_device_count < 256; bus++) {
            for (uint8_t device = 0; device < 32 && g_pci_device_count < 256; device++) {
                for (uint8_t func = 0; func < 8 && g_pci_device_count < 256; func++) {
                    uint32_t vendor_device = pci_read_config(bus, device, func, 0x00);
                    uint16_t vendor_id = (uint16_t)(vendor_device & 0xFFFFu);
                    uint16_t device_id = (uint16_t)((vendor_device >> 16) & 0xFFFFu);
                    
                    if (vendor_id == 0xFFFFu) {
                        if (func == 0u) break;
                        continue;
                    }
                    
                    uint32_t class_reg = pci_read_config(bus, device, func, 0x08);
                    uint8_t class_code = (uint8_t)((class_reg >> 24) & 0xFFu);
                    uint8_t subclass = (uint8_t)((class_reg >> 16) & 0xFFu);
                    
                    pci_device_t *pdev = &g_pci_devices_list[g_pci_device_count];
                    pdev->bus = bus;
                    pdev->device = device;
                    pdev->func = func;
                    pdev->vendor_id = vendor_id;
                    pdev->device_id = device_id;
                    pdev->class_code = class_code;
                    pdev->subclass = subclass;
                    
                    g_pci_device_count++;
                    
                    if (func == 0u) {
                        uint32_t header_type = pci_read_config(bus, device, func, 0x0C);
                        if (((header_type >> 16) & 0x80u) == 0u) {
                            break;
                        }
                    }
                }
            }
        }
    }
    
    spinlock_unlock(&g_pci_scan_lock);
    irq_restore(flags);
}

static const char *sysinfo_get_pci_vendor_name(uint16_t vendor_id)
{
    for (int i = 0; g_pci_vendors[i].vendor_name != NULL; i++) {
        if (g_pci_vendors[i].vendor_id == vendor_id) {
            return g_pci_vendors[i].vendor_name;
        }
    }
    return "Unknown";
}

static const char *sysinfo_get_pci_device_name(uint16_t vendor_id, uint16_t device_id)
{
    for (int i = 0; g_pci_devices[i].device_name != NULL; i++) {
        if (g_pci_devices[i].vendor_id == vendor_id && 
            g_pci_devices[i].device_id == device_id) {
            return g_pci_devices[i].device_name;
        }
    }
    return "Unknown PCI Device";
}

static system_device_type_t sysinfo_classify_pci_device(uint8_t class_code, uint8_t subclass)
{
    switch (class_code) {
        case 0x01:
            if (subclass == 0x01) return SYSTEM_DEVICE_ATA_CONTROLLER;
            if (subclass == 0x08) return SYSTEM_DEVICE_STORAGE_CONTROLLER;
            return SYSTEM_DEVICE_STORAGE_CONTROLLER;
        case 0x02:
            return SYSTEM_DEVICE_NETWORK_ADAPTER;
        case 0x03:
            return SYSTEM_DEVICE_GRAPHICS_ADAPTER;
        case 0x04:
            return SYSTEM_DEVICE_AUDIO_DEVICE;
        case 0x0C:
            if (subclass == 0x03) return SYSTEM_DEVICE_USB_CONTROLLER;
            return SYSTEM_DEVICE_UNKNOWN;
        case 0x06:
            return SYSTEM_DEVICE_PCI_BRIDGE;
        default:
            return SYSTEM_DEVICE_UNKNOWN;
    }
}

static void sysinfo_copy_string(char *dst, size_t dst_size, const char *src)
{
    if (dst == NULL || dst_size == 0) {
        return;
    }

    size_t i = 0;
    if (src != NULL) {
        for (; i + 1 < dst_size && src[i]; i++) {
            dst[i] = src[i];
        }
    }
    dst[i] = '\0';
}

os_status_t sysinfo_get_cpu_info(system_cpu_info_t *out_info)
{
    if (out_info == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
#if defined(__aarch64__)
    uint64_t midr;
    __asm__ volatile("mrs %0, MIDR_EL1" : "=r"(midr));
    out_info->vendor[0] = 'A';
    out_info->vendor[1] = 'R';
    out_info->vendor[2] = 'M';
    out_info->vendor[3] = '\0';
    out_info->stepping = (uint32_t)(midr & 0xFu);
    out_info->model = (uint32_t)((midr >> 4) & 0xFFFu);
    out_info->family = (uint32_t)((midr >> 16) & 0xFu);
    out_info->logical_cores = 1;
    out_info->physical_cores = 1;
    out_info->brand[0] = 'A';
    out_info->brand[1] = 'A';
    out_info->brand[2] = 'r';
    out_info->brand[3] = 'c';
    out_info->brand[4] = 'h';
    out_info->brand[5] = '6';
    out_info->brand[6] = '4';
    out_info->brand[7] = '\0';
#else
    uint32_t eax, ebx, ecx, edx;
    __asm__ volatile("cpuid" : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx) : "a"(0));
    
    ((uint32_t *)out_info->vendor)[0] = ebx;
    ((uint32_t *)out_info->vendor)[1] = edx;
    ((uint32_t *)out_info->vendor)[2] = ecx;
    ((uint32_t *)out_info->vendor)[3] = 0;
    
    __asm__ volatile("cpuid" : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx) : "a"(1));
    
    out_info->stepping = eax & 0xF;
    out_info->model = (eax >> 4) & 0xF;
    out_info->family = (eax >> 8) & 0xF;
    
    out_info->logical_cores = (ebx >> 16) & 0xFF;
    if (out_info->logical_cores == 0) out_info->logical_cores = 1;
    
    out_info->physical_cores = out_info->logical_cores / 2;
    if (out_info->physical_cores == 0) out_info->physical_cores = 1;
    
    out_info->brand[0] = '\0';
    eax = 0x80000000;
    __asm__ volatile("cpuid" : "=a"(eax) : "a"(eax) : "ebx", "ecx", "edx");
    
    if (eax >= 0x80000004) {
        uint32_t *brand_ptr = (uint32_t *)out_info->brand;
        for (uint32_t i = 0x80000002; i <= 0x80000004; i++) {
            __asm__ volatile("cpuid" 
                           : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx) 
                           : "a"(i));
            *brand_ptr++ = eax;
            *brand_ptr++ = ebx;
            *brand_ptr++ = ecx;
            *brand_ptr++ = edx;
        }
    }
#endif
    
    out_info->frequency_mhz = 2400;
    
    out_info->cache_l1d_kb = 32;
    out_info->cache_l1i_kb = 32;
    out_info->cache_l2_kb = 256;
    out_info->cache_l3_kb = 8192;
    
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_memory_info(system_memory_info_t *out_info)
{
    if (out_info == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    out_info->page_size = 4096;
    out_info->total_bytes = get_total_memory_pages() * 4096;
    out_info->used_bytes = get_used_memory();
    out_info->free_bytes = get_free_memory();
    out_info->cached_bytes = 0;
    out_info->buffers_bytes = 0;
    
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_vmem_info(system_vmem_info_t *out_info)
{
    if (out_info == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    out_info->page_size = 4096;
    out_info->total_pages = get_total_memory_pages();
    out_info->free_pages = get_total_memory_pages() - (get_used_memory() / 4096);
    out_info->mapped_pages = get_used_memory() / 4096;
    
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_disk_count(uint32_t *out_count)
{
    if (out_count == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    *out_count = disk_get_count();
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_disk_info(uint32_t index, system_disk_info_t *out_info)
{
    if (out_info == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    io_disk_info_t disk_info;
    if (!disk_get_info(index, &disk_info)) {
        return OS_STATUS_NOT_FOUND;
    }

    out_info->total_bytes = disk_info.total_bytes;
    out_info->used_bytes = 0;
    out_info->free_bytes = 0;
    out_info->sector_size = disk_info.sector_size;
    out_info->protocol = (uint32_t)disk_info.protocol;
    out_info->flags = disk_info.flags;

    sysinfo_copy_string(out_info->disk_name, sizeof(out_info->disk_name), disk_info.disk_name);
    sysinfo_copy_string(out_info->manufacturer, sizeof(out_info->manufacturer), disk_info.manufacturer);
    sysinfo_copy_string(out_info->model, sizeof(out_info->model), disk_info.model);
    
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_device_count(uint32_t *out_count)
{
    if (out_count == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    sysinfo_scan_pci_devices();
    *out_count = g_pci_device_count;
    
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_device_info(uint32_t index, system_device_t *out_info)
{
    if (out_info == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    sysinfo_scan_pci_devices();
    
    if (index >= g_pci_device_count) {
        return OS_STATUS_NOT_FOUND;
    }
    
    pci_device_t *pdev = &g_pci_devices_list[index];
    
    out_info->vendor_id = pdev->vendor_id;
    out_info->device_id = pdev->device_id;
    out_info->bus = pdev->bus;
    out_info->device = pdev->device;
    out_info->func = pdev->func;
    out_info->reserved = 0;
    
    out_info->type = sysinfo_classify_pci_device(pdev->class_code, pdev->subclass);
    
    const char *vendor_name = sysinfo_get_pci_vendor_name(pdev->vendor_id);
    const char *device_name = sysinfo_get_pci_device_name(pdev->vendor_id, pdev->device_id);
    
    for (size_t i = 0; i < sizeof(out_info->vendor_name) - 1 && vendor_name[i]; i++) {
        out_info->vendor_name[i] = vendor_name[i];
    }
    out_info->vendor_name[sizeof(out_info->vendor_name) - 1] = '\0';
    
    for (size_t i = 0; i < sizeof(out_info->device_name) - 1 && device_name[i]; i++) {
        out_info->device_name[i] = device_name[i];
    }
    out_info->device_name[sizeof(out_info->device_name) - 1] = '\0';
    
    out_info->irq = 0;
    out_info->flags = 0;
    
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_graphics_info(system_graphics_info_t *out_info)
{
    if (out_info == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    sysinfo_scan_pci_devices();
    
    const char *vendor = "Unknown";
    const char *model = "Framebuffer";
    uint32_t vram_mb = 16;
    
    for (uint32_t i = 0; i < g_pci_device_count; i++) {
        pci_device_t *pdev = &g_pci_devices_list[i];
        if (pdev->class_code == 0x03) {
            vendor = sysinfo_get_pci_vendor_name(pdev->vendor_id);
            model = sysinfo_get_pci_device_name(pdev->vendor_id, pdev->device_id);
            out_info->vendor_id = pdev->vendor_id;
            out_info->device_id = pdev->device_id;
            
            if (pdev->bar[0] != 0) {
                vram_mb = 64;
            }
            break;
        }
    }
    
    for (size_t i = 0; i < sizeof(out_info->vendor) - 1 && vendor[i]; i++) {
        out_info->vendor[i] = vendor[i];
    }
    out_info->vendor[sizeof(out_info->vendor) - 1] = '\0';
    
    for (size_t i = 0; i < sizeof(out_info->model) - 1 && model[i]; i++) {
        out_info->model[i] = model[i];
    }
    out_info->model[sizeof(out_info->model) - 1] = '\0';
    
    out_info->vram_mb = vram_mb;
    out_info->display_width = driver_manager_display_width();
    out_info->display_height = driver_manager_display_height();
    out_info->bits_per_pixel = 32;
    
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_arch_info(system_arch_info_t *out_info)
{
    if (out_info == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    out_info->bits = 64;
    out_info->endianness = 0;
    
    const char *arch_name = "x86_64";
    for (size_t i = 0; i < sizeof(out_info->name) - 1 && arch_name[i]; i++) {
        out_info->name[i] = arch_name[i];
    }
    out_info->name[sizeof(out_info->name) - 1] = '\0';
    
    return OS_STATUS_OK;
}

os_status_t sysinfo_get_system_info(system_info_t *out_info)
{
    if (out_info == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    
    os_status_t status;
    
    status = sysinfo_get_arch_info(&out_info->arch);
    if (os_status_is_error(status)) return status;
    
    status = sysinfo_get_cpu_info(&out_info->cpu);
    if (os_status_is_error(status)) return status;
    
    status = sysinfo_get_memory_info(&out_info->memory);
    if (os_status_is_error(status)) return status;
    
    status = sysinfo_get_vmem_info(&out_info->vmem);
    if (os_status_is_error(status)) return status;
    
    return OS_STATUS_OK;
}
