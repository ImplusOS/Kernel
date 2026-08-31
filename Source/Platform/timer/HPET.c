#include "HPET.h"

#include "Platform/acpi/ACPI.h"
#include "mmu/Paging_Main.h"

#include <stddef.h>

static volatile uint8_t *g_hpet;
static uint64_t g_period_fs;
static bool g_available;

static uint64_t hpet_read64(uint32_t offset)
{
    return *(volatile uint64_t *)(g_hpet + offset);
}

static void hpet_write64(uint32_t offset, uint64_t value)
{
    *(volatile uint64_t *)(g_hpet + offset) = value;
}

bool hpet_init(void)
{
#if !defined(PLATFORM_X86_64)
    return false;
#else
    const acpi_info_t *info = acpi_get_info();
    if (info == NULL || info->hpet_base == 0u) {
        return false;
    }
    g_hpet = map_mmio_virt(info->hpet_base);
    if (g_hpet == NULL) {
        return false;
    }
    uint64_t capabilities = hpet_read64(0x00u);
    g_period_fs = capabilities >> 32u;
    if (g_period_fs == 0u) {
        g_hpet = NULL;
        return false;
    }
    uint64_t config = hpet_read64(0x10u);
    hpet_write64(0x10u, config & ~1ULL);
    hpet_write64(0xF0u, 0u);
    hpet_write64(0x10u, (config & ~(1ULL << 1u)) | 1u);
    g_available = true;
    return true;
#endif
}

bool hpet_is_available(void)
{
    return g_available;
}

uint64_t hpet_monotonic_ns(void)
{
    if (!g_available) {
        return 0u;
    }
    uint64_t counter = hpet_read64(0xF0u);
    uint64_t whole = counter / 1000000ULL;
    uint64_t remainder = counter % 1000000ULL;
    return whole * g_period_fs + (remainder * g_period_fs) / 1000000ULL;
}
