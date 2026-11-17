#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "kernel/boot_info.h"

#define ACPI_MAX_CPUS 32

typedef struct {
    uint64_t lapic_base;
    uint64_t ioapic_base;
    uint32_t ioapic_gsi_base;
    uint8_t  cpu_count;
    uint8_t  cpu_apic_ids[ACPI_MAX_CPUS];
    uint32_t pit_gsi;
    uint8_t  pit_level_trigger;
    uint8_t  pit_active_low;
    uint64_t hpet_base;

    uint64_t gicd_base;
    uint64_t gicc_base;
    uint64_t gicr_base;
    uint32_t gicr_length;
    uint8_t  gic_version;

    /* AArch64: MPIDR_EL1 affinity of every enabled GICC (MADT type 11), in
     * MADT order -- cpu_count counts them -- and the FADT ARM_BOOT_ARCH PSCI
     * flags (bit 0 PSCI_COMPLIANT, bit 1 PSCI_USE_HVC). */
    uint64_t cpu_mpidr[ACPI_MAX_CPUS];
    uint8_t  psci_compliant;
    uint8_t  psci_use_hvc;
    
    uint32_t pm1a_cnt_blk;
    uint16_t slp_typ_s5;
    bool     has_s5;
} acpi_info_t;

int  acpi_init(const BOOT_INFO *boot_info);
const acpi_info_t *acpi_get_info(void);
void acpi_shutdown(void);
void acpi_reboot(void);
