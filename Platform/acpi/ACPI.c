#include "ACPI.h"

#include <string.h>
#include "mmu/Paging_Main.h"
#include "Debug/serial/Serial.h"
#include "Drivers/Module/DriverManager.h"

typedef struct {
    char     signature[8];
    uint8_t  checksum;
    char     oem_id[6];
    uint8_t  revision;
    uint32_t rsdt_address;
} __attribute__((packed)) rsdp_v1_t;

typedef struct {
    rsdp_v1_t v1;
    uint32_t  length;
    uint64_t  xsdt_address;
    uint8_t   extended_checksum;
    uint8_t   reserved[3];
} __attribute__((packed)) rsdp_v2_t;

typedef struct {
    char     signature[4];
    uint32_t length;
    uint8_t  revision;
    uint8_t  checksum;
    char     oem_id[6];
    char     oem_table_id[8];
    uint32_t oem_revision;
    uint32_t creator_id;
    uint32_t creator_revision;
} __attribute__((packed)) sdt_header_t;

typedef struct {
    uint8_t address_space;
    uint8_t bit_width;
    uint8_t bit_offset;
    uint8_t access_size;
    uint64_t address;
} __attribute__((packed)) acpi_gas_t;

typedef struct {
    sdt_header_t header;
    uint32_t event_timer_block_id;
    acpi_gas_t base_address;
    uint8_t hpet_number;
    uint16_t minimum_tick;
    uint8_t page_protection;
} __attribute__((packed)) hpet_table_t;

typedef struct {
    sdt_header_t header;
    uint32_t     lapic_addr;
    uint32_t     flags;
    uint8_t      entries[];
} __attribute__((packed)) madt_t;

typedef struct {
    uint8_t type;
    uint8_t length;
} __attribute__((packed)) madt_entry_header_t;

typedef struct {
    madt_entry_header_t h;
    uint8_t acpi_processor_id;
    uint8_t apic_id;
    uint32_t flags;
} __attribute__((packed)) madt_local_apic_t;

typedef struct {
    madt_entry_header_t h;
    uint8_t  ioapic_id;
    uint8_t  reserved;
    uint32_t ioapic_addr;
    uint32_t gsi_base;
} __attribute__((packed)) madt_ioapic_t;

typedef struct {
    madt_entry_header_t h;
    uint16_t reserved;
    uint64_t lapic_addr;
} __attribute__((packed)) madt_lapic_override_t;

typedef struct {
    madt_entry_header_t h;
    uint8_t  bus;
    uint8_t  source_irq;
    uint32_t global_system_interrupt;
    uint16_t flags;
} __attribute__((packed)) madt_iso_t; 

typedef struct {
    madt_entry_header_t h;
    uint16_t reserved;
    uint32_t cpu_interface_number;
    uint32_t acpi_processor_uid;
    uint32_t flags;
    uint32_t parking_protocol_version;
    uint32_t performance_interrupt_gsiv;
    uint64_t parked_address;
    uint64_t physical_base_address;
    uint64_t gicv_base_address;
    uint64_t gich_base_address;
    uint32_t vgic_maintenance_interrupt;
    uint64_t gicr_base_address;
    uint64_t mpidr;
    uint8_t  processor_power_efficiency_class;
    uint8_t  reserved2;
    uint16_t spe_overflow_interrupt;
} __attribute__((packed)) madt_gicc_t;

typedef struct {
    madt_entry_header_t h;
    uint16_t reserved;
    uint32_t gic_id;
    uint64_t physical_base_address;
    uint32_t system_vector_base;
    uint8_t  gic_version;
    uint8_t  reserved2[3];
} __attribute__((packed)) madt_gicd_t;

typedef struct {
    madt_entry_header_t h;
    uint16_t reserved;
    uint64_t discovery_range_base_address;
    uint32_t discovery_range_length;
} __attribute__((packed)) madt_gicr_t;

typedef struct {
    sdt_header_t header;
    uint32_t firmware_ctrl;
    uint32_t dsdt;
    uint8_t  reserved;
    uint8_t  preferred_pm_profile;
    uint16_t sci_int;
    uint32_t smi_cmd;
    uint8_t  acpi_enable;
    uint8_t  acpi_disable;
    uint8_t  s4bios_req;
    uint8_t  pstate_cnt;
    uint32_t pm1a_evt_blk;
    uint32_t pm1b_evt_blk;
    uint32_t pm1a_cnt_blk;
    uint32_t pm1b_cnt_blk;
    uint32_t pm2_cnt_blk;
    uint32_t pm_tmr_blk;
    uint32_t gpe0_blk;
    uint32_t gpe1_blk;
    uint8_t  pm1_evt_len;
    uint8_t  pm1_cnt_len;
} __attribute__((packed)) fadt_t;

static acpi_info_t g_info;
static int g_ready = 0;

#include "Platform/io/IO_Main.h"

void acpi_reboot(void)
{
    
    outb(0x64, 0xFE);
    
#if defined(__aarch64__)
    while (1) {
        __asm__ volatile("wfi");
    }
#else
    __asm__ volatile ("lidt %0; int3" :: "m"((uint16_t[3]){0,0,0}));
#endif
    while(1);
}

void acpi_shutdown(void)
{
    
    const device_t *display_device = driver_manager_find(DEVICE_TYPE_DISPLAY, NULL);
    const driver_display_t *display = display_device ? (const driver_display_t *)display_device->ops : NULL;
    if (display) {
        uint32_t w = display->width();
        uint32_t h = display->height();
        display->fill_rect(0, 0, w, h, 0x000000); 
        
        for (uint32_t y = h/2 - 20; y < h/2 + 20; y++) {
            for (uint32_t x = w/2 - 100; x < w/2 + 100; x++) {
                display->draw_pixel(x, y, 0xFFFFFF); 
            }
        }
        display->present();
    }

    if (g_info.has_s5 && g_info.pm1a_cnt_blk != 0) {
        outw((uint16_t)g_info.pm1a_cnt_blk, (uint16_t)(g_info.slp_typ_s5 | (1 << 13)));
    }
    
    
    outw(0xB004, 0x2000);
    outw(0x604, 0x2000);
    outw(0x4004, 0x3400);
    
    while(1);
}

static void parse_dsdt(const sdt_header_t *dsdt)
{
    if (dsdt == NULL) return;
    
    const uint8_t *s5_ptr = (const uint8_t *)dsdt + sizeof(sdt_header_t);
    uint32_t len = dsdt->length - sizeof(sdt_header_t);
    
    
    for (uint32_t i = 0; i < len - 4; i++) {
        if (memcmp(s5_ptr + i, "_S5_", 4) == 0) {
            i += 4;
            
            
            if (s5_ptr[i] == 0x12) { 
                i += 3; 
                if (s5_ptr[i] == 0x0A) i++; 
                g_info.slp_typ_s5 = (uint16_t)((uint16_t)s5_ptr[i] << 10);
                g_info.has_s5 = true;
                return;
            }
        }
    }
}

static uint8_t checksum8(const uint8_t *p, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; ++i) {
        sum += p[i];
    }
    return (uint8_t)(sum & 0xFFu);
}

static const sdt_header_t *validate_sdt(const void *addr, const char sig[4])
{
    if (addr == NULL) return NULL;
    const sdt_header_t *hdr = (const sdt_header_t *)addr;
    if (memcmp(hdr->signature, sig, 4) != 0) {
        return NULL;
    }
    if (hdr->length < sizeof(sdt_header_t)) {
        return NULL;
    }
    if (checksum8((const uint8_t *)hdr, hdr->length) != 0) {
        return NULL;
    }
    return hdr;
}

static void parse_fadt(const fadt_t *fadt)
{
    if (fadt == NULL) return;
    
    g_info.pm1a_cnt_blk = fadt->pm1a_cnt_blk;
    
    const sdt_header_t *dsdt = (const sdt_header_t *)map_mmio_virt(fadt->dsdt);
    dsdt = validate_sdt(dsdt, "DSDT");
    parse_dsdt(dsdt);
}

static const sdt_header_t *find_sdt_entry(const sdt_header_t *xsdt,
                                          int use_xsdt,
                                          const char sig[4])
{
    if (xsdt == NULL) return NULL;
    uint32_t entry_bytes = xsdt->length - sizeof(sdt_header_t);
    if (entry_bytes == 0) return NULL;

    if (use_xsdt) {
        uint32_t entry_count = entry_bytes / 8U;
        const uint64_t *entries = (const uint64_t *)((const uint8_t *)xsdt + sizeof(sdt_header_t));
        for (uint32_t i = 0; i < entry_count; ++i) {
            const sdt_header_t *candidate = (const sdt_header_t *)map_mmio_virt(entries[i]);
            candidate = validate_sdt(candidate, sig);
            if (candidate != NULL) return candidate;
        }
    } else {
        uint32_t entry_count = entry_bytes / 4U;
        const uint32_t *entries = (const uint32_t *)((const uint8_t *)xsdt + sizeof(sdt_header_t));
        for (uint32_t i = 0; i < entry_count; ++i) {
            const sdt_header_t *candidate = (const sdt_header_t *)map_mmio_virt((uint64_t)entries[i]);
            candidate = validate_sdt(candidate, sig);
            if (candidate != NULL) return candidate;
        }
    }

    return NULL;
}

static void parse_madt(const madt_t *madt)
{
    if (madt == NULL) {
        return;
    }

    g_info.lapic_base = (uint64_t)madt->lapic_addr;

    uint32_t offset = 0;
    uint32_t payload_len = madt->header.length - sizeof(madt_t);

    while (offset + sizeof(madt_entry_header_t) <= payload_len) {
        const madt_entry_header_t *hdr = (const madt_entry_header_t *)(madt->entries + offset);
        if (hdr->length < sizeof(madt_entry_header_t)) {
            break;
        }

        switch (hdr->type) {
            case 0: {
                if (hdr->length >= sizeof(madt_local_apic_t) &&
                    g_info.cpu_count < ACPI_MAX_CPUS) {
                    const madt_local_apic_t *lapic = (const madt_local_apic_t *)hdr;
                    if (lapic->flags & 0x1u) {
                        g_info.cpu_apic_ids[g_info.cpu_count++] = lapic->apic_id;
                    }
                }
                break;
            }
            case 1: {
                if (hdr->length >= sizeof(madt_ioapic_t) &&
                    g_info.ioapic_base == 0) {
                    const madt_ioapic_t *io = (const madt_ioapic_t *)hdr;
                    g_info.ioapic_base = (uint64_t)io->ioapic_addr;
                    g_info.ioapic_gsi_base = io->gsi_base;
                }
                break;
            }
            case 2: {
                if (hdr->length >= sizeof(madt_iso_t)) {
                    const madt_iso_t *iso = (const madt_iso_t *)hdr;
                    if (iso->bus == 0 && iso->source_irq == 0) {
                        g_info.pit_gsi = iso->global_system_interrupt;
                        g_info.pit_active_low =
                            ((iso->flags & 0x3u) == 0x3u) ? 1u : 0u;
                        g_info.pit_level_trigger =
                            (((iso->flags >> 2) & 0x3u) == 0x3u) ? 1u : 0u;
                    }
                }
                break;
            }
            case 5: {
                if (hdr->length >= sizeof(madt_lapic_override_t)) {
                    const madt_lapic_override_t *ovr = (const madt_lapic_override_t *)hdr;
                    g_info.lapic_base = ovr->lapic_addr;
                }
                break;
            }
            case 11: {
                if (hdr->length >= offsetof(madt_gicc_t, gicr_base_address) &&
                    g_info.gicc_base == 0) {
                    const madt_gicc_t *gicc = (const madt_gicc_t *)hdr;
                    if ((gicc->flags & 0x1u) != 0u) {
                        g_info.gicc_base = gicc->physical_base_address;
                        if (hdr->length >= offsetof(madt_gicc_t, mpidr) &&
                            g_info.gicr_base == 0) {
                            g_info.gicr_base = gicc->gicr_base_address;
                        }
                    }
                }
                break;
            }
            case 12: {
                if (hdr->length >= sizeof(madt_gicd_t)) {
                    const madt_gicd_t *gicd = (const madt_gicd_t *)hdr;
                    g_info.gicd_base = gicd->physical_base_address;
                    g_info.gic_version = gicd->gic_version;
                }
                break;
            }
            case 14: {
                if (hdr->length >= sizeof(madt_gicr_t)) {
                    const madt_gicr_t *gicr = (const madt_gicr_t *)hdr;
                    g_info.gicr_base = gicr->discovery_range_base_address;
                    g_info.gicr_length = gicr->discovery_range_length;
                }
                break;
            }
            default:
                break;
        }

        offset += hdr->length;
    }
}

int acpi_init(const BOOT_INFO *boot_info)
{
    g_info.pit_gsi = 0;
    g_ready = 0;
    for (uint32_t i = 0; i < ACPI_MAX_CPUS; ++i) {
        g_info.cpu_apic_ids[i] = 0xFFu;
    }
    g_info.cpu_count = 0;
    g_info.lapic_base = 0;
    g_info.ioapic_base = 0;
    g_info.ioapic_gsi_base = 0;
    g_info.pit_level_trigger = 0;
    g_info.pit_active_low = 0;
    g_info.hpet_base = 0;
    g_info.gicd_base = 0;
    g_info.gicc_base = 0;
    g_info.gicr_base = 0;
    g_info.gicr_length = 0;
    g_info.gic_version = 0;
 
    if (boot_info == NULL || boot_info->AcpiRsdpAddress == 0) {
        return -1;
    }
 
    const rsdp_v1_t *rsdp1 = (const rsdp_v1_t *)(uintptr_t)boot_info->AcpiRsdpAddress;
    
    if (memcmp(rsdp1->signature, "RSD PTR ", 8) != 0) {
        return -1;
    }
 
    if (checksum8((const uint8_t *)rsdp1, sizeof(rsdp_v1_t)) != 0) {
        return -1;
    }
 
    uint8_t actual_revision = rsdp1->revision;
    
    int use_xsdt = 0;
    uint64_t sdt_addr = rsdp1->rsdt_address;
 
    if (actual_revision >= 2) {
        const rsdp_v2_t *rsdp2 = (const rsdp_v2_t *)(uintptr_t)boot_info->AcpiRsdpAddress;
        
        if (rsdp2->length >= sizeof(rsdp_v2_t) && rsdp2->length <= 64) {
            if (checksum8((const uint8_t *)rsdp2, rsdp2->length) == 0) {
                if (rsdp2->xsdt_address != 0) {
                    sdt_addr = rsdp2->xsdt_address;
                    use_xsdt = 1;
                }
            }
        }
    }
 
    const sdt_header_t *root = validate_sdt((const void *)map_mmio_virt(sdt_addr),
                                            use_xsdt ? "XSDT" : "RSDT");
    if (root == NULL) {
        return -1;
    }
 
    const sdt_header_t *madt_hdr = find_sdt_entry(root, use_xsdt, "APIC");
    if (madt_hdr == NULL) {
        return -1;
    }
 
    parse_madt((const madt_t *)madt_hdr);

    const sdt_header_t *fadt_hdr = find_sdt_entry(root, use_xsdt, "FACP");
    if (fadt_hdr != NULL) {
        parse_fadt((const fadt_t *)fadt_hdr);
    }

    const hpet_table_t *hpet =
        (const hpet_table_t *)find_sdt_entry(root, use_xsdt, "HPET");
    if (hpet != NULL && hpet->base_address.address_space == 0u) {
        g_info.hpet_base = hpet->base_address.address;
    }

    g_ready = 1;
    return 0;
}

const acpi_info_t *acpi_get_info(void)
{
    if (!g_ready) {
        return NULL;
    }
    return &g_info;
}
