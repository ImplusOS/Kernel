#include "Interrupts.h"

#include "Platform/io/IO_Main.h"
#include "Platform/interrupt/LAPIC.h"
#include "Platform/interrupt/IOAPIC.h"
#include "interfaces/interrupt_ops.h"

static int g_use_lapic = 0;

void platform_interrupts_init_legacy_pic(void)
{
    outb(0x21, 0xFF);
    outb(0xA1, 0xFF);
}

int platform_interrupts_configure(const acpi_info_t *info)
{
    if (info == NULL) {
        platform_interrupts_init_legacy_pic();
        return -1;
    }

    platform_interrupts_init_legacy_pic();

    if (info->lapic_base != 0) {
        if (lapic_init(info->lapic_base) == 0) {
            g_use_lapic = 1;
        }
    }

    if (info->ioapic_base != 0) {
        ioapic_init(info->ioapic_base, info->ioapic_gsi_base);
        ioapic_mask_all();
    }

    return 0;
}

void platform_interrupts_eoi(uint16_t vector)
{
    if (g_use_lapic) {
        lapic_eoi();
    } else {
        outb(0x20, 0x20);
        if (vector >= 40 && vector < 48) {
            outb(0xA0, 0x20);
        }
    }
}

int platform_interrupts_using_lapic(void)
{
    return g_use_lapic;
}

void platform_interrupts_route_pit(void)
{
    if (g_use_lapic) {
        const acpi_info_t *acpi = acpi_get_info();
        if (acpi != NULL) {
            ioapic_route_irq((uint8_t)acpi->pit_gsi,
                             VECTOR_TIMER,
                             0,
                             acpi->pit_level_trigger != 0,
                             acpi->pit_active_low != 0);
        }
    }
}

void platform_interrupts_mask_pit(void)
{
    if (g_use_lapic) {
        const acpi_info_t *acpi = acpi_get_info();
        if (acpi != NULL) {
            ioapic_mask_irq(acpi->pit_gsi);
        }
    }
}

static int interrupt_ops_configure(const void *firmware_info)
{
    return platform_interrupts_configure((const acpi_info_t *)firmware_info);
}

static void interrupt_ops_eoi(uint32_t vector)
{
    platform_interrupts_eoi((uint16_t)vector);
}

static int interrupt_ops_route_irq(uint32_t irq, uint32_t vector)
{
    if (!g_use_lapic) {
        return -1;
    }
    ioapic_route_irq((uint8_t)irq, (uint8_t)vector, 0, 0, 0);
    return 0;
}

static void interrupt_ops_mask_irq(uint32_t irq)
{
    if (g_use_lapic) {
        ioapic_mask_irq(irq);
    }
}

static void interrupt_ops_unmask_irq(uint32_t irq)
{
    if (g_use_lapic) {
        ioapic_unmask_irq(irq);
    }
}

static const interrupt_ops_t g_interrupt_ops = {
    .configure = interrupt_ops_configure,
    .eoi = interrupt_ops_eoi,
    .route_irq = interrupt_ops_route_irq,
    .mask_irq = interrupt_ops_mask_irq,
    .unmask_irq = interrupt_ops_unmask_irq,
    .using_local_timer = platform_interrupts_using_lapic,
};

const interrupt_ops_t *interrupt_ops_get(void)
{
    return &g_interrupt_ops;
}
