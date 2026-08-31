#include "IOAPIC.h"
#include <stddef.h>

#include <stddef.h>
#include "mmu/Paging_Main.h"

#define IOAPIC_REG_ID     0x00
#define IOAPIC_REG_VER    0x01
#define IOAPIC_REG_REDIR  0x10

static volatile uint32_t *g_ioapic_base = NULL;
static uint32_t g_ioapic_gsi_base = 0;
static uint32_t g_ioapic_max_redir = 0;
static int g_ioapic_present = 0;

static inline void ioapic_write(uint8_t reg, uint32_t value)
{
    g_ioapic_base[0] = reg;
    g_ioapic_base[4] = value;
}

static inline uint32_t ioapic_read(uint8_t reg)
{
    g_ioapic_base[0] = reg;
    return g_ioapic_base[4];
}

int ioapic_init(uint64_t phys_base, uint32_t gsi_base)
{
    if (phys_base == 0) {
        return -1;
    }

    g_ioapic_base = (volatile uint32_t *)map_mmio_virt(phys_base);
    g_ioapic_gsi_base = gsi_base;

    uint32_t ver = ioapic_read(IOAPIC_REG_VER);
    g_ioapic_max_redir = ((ver >> 16) & 0xFFu) + 1u;

    g_ioapic_present = 1;
    return 0;
}

int ioapic_is_present(void)
{
    return g_ioapic_present;
}

static void ioapic_write_redir(uint8_t index, uint64_t value)
{
    ioapic_write((uint8_t)(IOAPIC_REG_REDIR + (index * 2u)), (uint32_t)(value & 0xFFFFFFFFu));
    ioapic_write((uint8_t)(IOAPIC_REG_REDIR + (index * 2u) + 1u), (uint32_t)(value >> 32));
}

void ioapic_mask_all(void)
{
    if (!g_ioapic_present) return;
    for (uint32_t i = 0; i < g_ioapic_max_redir; ++i) {
        ioapic_write_redir((uint8_t)i, 1ULL << 16);
    }
}

int ioapic_route_irq(uint8_t irq, uint8_t vector, uint8_t dest_apic_id, int level_trigger, int active_low)
{
    if (!g_ioapic_present) {
        return -1;
    }
    if (irq < g_ioapic_gsi_base) return -1;
    uint32_t redir_index = irq - g_ioapic_gsi_base;
    if (redir_index >= g_ioapic_max_redir) {
        return -1;
    }

    uint64_t entry = vector;
    if (level_trigger) entry |= (1ULL << 15);
    if (active_low)    entry |= (1ULL << 13);
    entry |= ((uint64_t)dest_apic_id) << 56;

    ioapic_write_redir((uint8_t)redir_index, entry);
    return 0;
}

int ioapic_mask_irq(uint32_t irq)
{
    if (!g_ioapic_present) return -1;
    if (irq < g_ioapic_gsi_base) return -1;
    
    uint32_t redir_index = irq - g_ioapic_gsi_base;
    if (redir_index >= g_ioapic_max_redir) return -1;

    ioapic_write((uint8_t)(IOAPIC_REG_REDIR + (redir_index * 2u)), 1ULL << 16);
    return 0;
}

int ioapic_unmask_irq(uint32_t irq)
{
    if (!g_ioapic_present) return -1;
    if (irq < g_ioapic_gsi_base) return -1;

    uint32_t redir_index = irq - g_ioapic_gsi_base;
    if (redir_index >= g_ioapic_max_redir) return -1;

    uint8_t reg = (uint8_t)(IOAPIC_REG_REDIR + (redir_index * 2u));
    uint32_t low = ioapic_read(reg);
    low &= ~(1u << 16);
    ioapic_write(reg, low);
    return 0;
}
