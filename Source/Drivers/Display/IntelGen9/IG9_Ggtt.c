/*
 * IG9_Ggtt.c - Global GTT access and a private-range allocator.
 *
 * Layout (i915 gen8_gmch_probe / ggtt_probe_common):
 *   - the GGTT is an array of 8-byte PTEs at BAR0 + BAR0_size/2;
 *   - its size is GMCH_CTRL.GGMS: N != 0 means (1 << N) MiB of PTEs;
 *   - a PTE is the page's physical address | _PAGE_PRESENT
 *     (gen8_ggtt_pte_encode; Gen8+ GGTT PTEs carry no cache bits).
 *
 * The firmware owns the low part of the table (the GOP framebuffer and
 * whatever else it pinned). This driver allocates only from a window at the
 * very top, and only entries that still hold the table's "empty" value: 0,
 * or - when the firmware pointed every unused entry at one scratch page, as
 * i915 itself does - that scratch PTE. Allocations get an unmapped guard
 * page on each side, every PTE write is read back, and failures roll back.
 */

#ifdef IMPLUS_DRIVER_MODULE

#include "IG9_Private.h"

/* 64 MiB of graphics address space: a 4K back buffer is ~32 MiB. */
#define IG9_GGTT_PRIVATE_ENTRIES 16384u
#define IG9_GGTT_MAX_MAPPINGS    64u

typedef struct {
    bool used;
    uint32_t slot_first;    /* first slot incl. leading guard */
    uint32_t slot_count;    /* pages + 2 guards */
    uint32_t first_entry;   /* first mapped entry */
    uint32_t pages;
} ig9_ggtt_mapping_t;

static uint32_t s_entries = 0u;
static uint32_t s_ggtt_offset = 0u;            /* within BAR0 */
static uint32_t s_private_first = 0u;
static uint64_t s_empty_pte = 0u;
static bool s_ready = false;
static uint32_t s_bitmap[IG9_GGTT_PRIVATE_ENTRIES / 32u];
static ig9_ggtt_mapping_t s_maps[IG9_GGTT_MAX_MAPPINGS];
static ig9_spinlock_t s_lock;

uint32_t ig9_ggtt_entry_count(void)
{
    return s_entries;
}

uint64_t ig9_ggtt_read_pte(uint32_t index)
{
    if (index >= s_entries) {
        return 0u;
    }
    uint32_t off = s_ggtt_offset + index * IG9_GGTT_PTE_BYTES;
    uint64_t lo = ig9_read32(off);
    uint64_t hi = ig9_read32(off + 4u);
    return lo | (hi << 32);
}

static void ig9_ggtt_write_pte(uint32_t index, uint64_t pte)
{
    if (index >= s_entries) {
        return;
    }
    uint32_t off = s_ggtt_offset + index * IG9_GGTT_PTE_BYTES;
    /* The entry is not live while it is being written (allocation and
     * teardown both happen with no GPU user), so two 32-bit stores are
     * as good as i915's writeq. */
    ig9_write32(off + 4u, (uint32_t)(pte >> 32));
    ig9_write32(off, (uint32_t)pte);
}

static void ig9_ggtt_invalidate(void)
{
    ig9_mb();
    ig9_write32(IG9_GFX_FLSH_CNTL_GEN6, IG9_GFX_FLSH_CNTL_EN);
    (void)ig9_read32(IG9_GFX_FLSH_CNTL_GEN6);
}

static uint32_t ig9_ggms_from(uint32_t gmch)
{
    return (gmch >> IG9_GMCH_GGMS_SHIFT) & IG9_GMCH_GGMS_MASK;
}

static uint32_t ig9_read_ggms(void)
{
    uint32_t ggms = ig9_ggms_from(ig9_pci_read(IG9_PCI_GMCH_CTRL));
    if (ggms != 0u) {
        return ggms;
    }

    /* Fall back to the host bridge's copy of GGC. */
    const driver_binary_t *api = g_ig9_api;
    if (api->pci.get_device_count == NULL || api->pci.get_device == NULL ||
        api->pci.read_config == NULL) {
        return 0u;
    }
    uint32_t count = api->pci.get_device_count();
    for (uint32_t i = 0; i < count; ++i) {
        driver_pci_device_t dev;
        if (!api->pci.get_device(i, &dev) ||
            dev.vendor_id != IG9_PCI_VENDOR_INTEL ||
            dev.class_code != IG9_PCI_CLASS_BRIDGE ||
            dev.subclass != IG9_PCI_SUBCLASS_HOST) {
            continue;
        }
        return ig9_ggms_from(api->pci.read_config(dev.bus, dev.device,
                                                  dev.function,
                                                  IG9_PCI_GMCH_CTRL));
    }
    return 0u;
}

bool ig9_ggtt_init(void)
{
    if (s_ready) {
        return true;
    }
    if (!ig9_uncore_ready()) {
        return false;
    }

    uint32_t ggms = ig9_read_ggms();
    if (ggms == 0u) {
        ig9_log("[IntelGen9] ggtt: GMCH_CTRL.GGMS is 0 (GTT disabled)\n");
        return false;
    }

    uint64_t table_bytes = (uint64_t)(1u << ggms) * 1024u * 1024u;
    uint64_t window = g_ig9_bars.mmio_size / 2u;
    if (table_bytes > window) {
        /* Never index past the half of BAR0 that holds the table. */
        table_bytes = window;
    }
    uint32_t entries = (uint32_t)(table_bytes / IG9_GGTT_PTE_BYTES);
    if (entries < IG9_GGTT_PRIVATE_ENTRIES * 4u) {
        ig9_log_val("ggtt: table too small, entries", entries);
        return false;
    }

    s_ggtt_offset = (uint32_t)window;
    s_entries = entries;
    s_private_first = entries - IG9_GGTT_PRIVATE_ENTRIES;

    /* What does an unused entry look like? Firmware normally leaves zeros;
     * i915-style setups point every unused entry at one scratch page. */
    uint64_t last = ig9_ggtt_read_pte(entries - 1u);
    s_empty_pte = 0u;
    if (last != 0u) {
        bool uniform = true;
        for (uint32_t i = 0; i < 256u; ++i) {
            if (ig9_ggtt_read_pte(entries - 1u - i) != last) {
                uniform = false;
                break;
            }
        }
        if (uniform) {
            s_empty_pte = last;
            ig9_log_val("ggtt: unused entries point at scratch pte",
                        (uint32_t)last);
        }
    }

    ig9_memset(s_bitmap, 0, sizeof(s_bitmap));
    ig9_memset(s_maps, 0, sizeof(s_maps));
    s_ready = true;

    ig9_log("[IntelGen9] ggtt: ");
    ig9_log_dec(entries);
    ig9_log(" entries (");
    ig9_log_dec((uint32_t)(((uint64_t)entries * IG9_PAGE_BYTES) >> 20));
    ig9_log(" MiB), private window at 0x");
    ig9_log_hex(s_private_first * IG9_PAGE_BYTES);
    ig9_log("\n");
    return true;
}

static bool ig9_ggtt_span(uint32_t ggtt, uint64_t bytes, uint32_t *first,
                          uint32_t *pages)
{
    if (!s_ready || bytes == 0u) {
        return false;
    }
    uint64_t start = ggtt & ~(uint64_t)(IG9_PAGE_BYTES - 1u);
    uint64_t end = (uint64_t)ggtt + bytes;
    uint64_t count = (end - start + IG9_PAGE_BYTES - 1u) / IG9_PAGE_BYTES;
    uint64_t first_entry = start / IG9_PAGE_BYTES;
    if (first_entry + count > s_entries) {
        return false;
    }
    *first = (uint32_t)first_entry;
    *pages = (uint32_t)count;
    return true;
}

bool ig9_ggtt_range_present(uint32_t ggtt, uint64_t bytes)
{
    uint32_t first = 0u;
    uint32_t pages = 0u;
    if (!ig9_ggtt_span(ggtt, bytes, &first, &pages)) {
        return false;
    }
    for (uint32_t i = 0; i < pages; ++i) {
        uint64_t pte = ig9_ggtt_read_pte(first + i);
        if ((pte & IG9_GGTT_PTE_PRESENT) == 0u ||
            (s_empty_pte != 0u && pte == s_empty_pte)) {
            return false;
        }
    }
    return true;
}

bool ig9_ggtt_range_maps(uint32_t ggtt, uint64_t phys, uint64_t bytes)
{
    uint32_t first = 0u;
    uint32_t pages = 0u;
    if (!ig9_ggtt_span(ggtt, bytes, &first, &pages)) {
        return false;
    }
    uint64_t base = phys & ~(uint64_t)(IG9_PAGE_BYTES - 1u);
    for (uint32_t i = 0; i < pages; ++i) {
        uint64_t pte = ig9_ggtt_read_pte(first + i);
        if ((pte & IG9_GGTT_PTE_PRESENT) == 0u ||
            (pte & IG9_GGTT_PTE_ADDR_MASK) !=
                base + (uint64_t)i * IG9_PAGE_BYTES) {
            return false;
        }
    }
    return true;
}

bool ig9_ggtt_find_phys(uint64_t phys, uint32_t *index_out)
{
    if (!s_ready) {
        return false;
    }
    uint64_t want = phys & IG9_GGTT_PTE_ADDR_MASK;
    for (uint32_t i = 0; i < s_entries; ++i) {
        uint64_t pte = ig9_ggtt_read_pte(i);
        if ((pte & IG9_GGTT_PTE_PRESENT) != 0u &&
            (pte & IG9_GGTT_PTE_ADDR_MASK) == want) {
            *index_out = i;
            return true;
        }
    }
    return false;
}

static bool ig9_slot_used(uint32_t slot)
{
    return (s_bitmap[slot / 32u] & (1u << (slot % 32u))) != 0u;
}

static void ig9_slots_set(uint32_t first, uint32_t count, bool used)
{
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t slot = first + i;
        if (used) {
            s_bitmap[slot / 32u] |= 1u << (slot % 32u);
        } else {
            s_bitmap[slot / 32u] &= ~(1u << (slot % 32u));
        }
    }
}

static uint64_t ig9_page_phys(const void *virt, uint64_t phys, uint32_t page)
{
    if (phys != 0u) {
        return phys + (uint64_t)page * IG9_PAGE_BYTES;
    }
    const uint8_t *p = (const uint8_t *)virt + (uint64_t)page * IG9_PAGE_BYTES;
    return g_ig9_api->virt_to_phys((void *)(uintptr_t)p);
}

bool ig9_ggtt_map(const void *virt, uint64_t phys, uint64_t bytes,
                  uint32_t *ggtt_out)
{
    if (!s_ready || ggtt_out == NULL || bytes == 0u ||
        (virt == NULL && phys == 0u) ||
        bytes > (uint64_t)(IG9_GGTT_PRIVATE_ENTRIES - 2u) * IG9_PAGE_BYTES) {
        return false;
    }
    if ((phys & (IG9_PAGE_BYTES - 1u)) != 0u ||
        ((uintptr_t)virt & (IG9_PAGE_BYTES - 1u)) != 0u) {
        return false;
    }

    uint32_t pages = (uint32_t)((bytes + IG9_PAGE_BYTES - 1u) / IG9_PAGE_BYTES);
    uint32_t needed = pages + 2u;

    /* Translate and validate every page before any PTE is written. */
    for (uint32_t i = 0; i < pages; ++i) {
        uint64_t p = ig9_page_phys(virt, phys, i);
        if (p == 0u || (p & (IG9_PAGE_BYTES - 1u)) != 0u ||
            (p & ~IG9_GGTT_PTE_ADDR_MASK) != 0u) {
            ig9_log_val("ggtt: unmappable page, phys lo", (uint32_t)p);
            return false;
        }
    }

    ig9_spin_lock(&s_lock);

    uint32_t map_idx = IG9_GGTT_MAX_MAPPINGS;
    for (uint32_t i = 0; i < IG9_GGTT_MAX_MAPPINGS; ++i) {
        if (!s_maps[i].used) {
            map_idx = i;
            break;
        }
    }

    uint32_t slot_first = UINT32_MAX;
    if (map_idx != IG9_GGTT_MAX_MAPPINGS) {
        uint32_t run = 0u;
        for (uint32_t slot = 0; slot < IG9_GGTT_PRIVATE_ENTRIES; ++slot) {
            bool free_slot = !ig9_slot_used(slot) &&
                ig9_ggtt_read_pte(s_private_first + slot) == s_empty_pte;
            run = free_slot ? run + 1u : 0u;
            if (run == needed) {
                slot_first = slot + 1u - needed;
                break;
            }
        }
    }
    if (slot_first == UINT32_MAX) {
        ig9_spin_unlock(&s_lock);
        ig9_log("[IntelGen9] ggtt: private window exhausted\n");
        return false;
    }

    uint32_t first_entry = s_private_first + slot_first + 1u;
    for (uint32_t i = 0; i < pages; ++i) {
        ig9_ggtt_write_pte(first_entry + i,
                           ig9_page_phys(virt, phys, i) | IG9_GGTT_PTE_PRESENT);
    }
    ig9_ggtt_invalidate();

    bool ok = true;
    for (uint32_t i = 0; i < pages; ++i) {
        uint64_t want = ig9_page_phys(virt, phys, i) | IG9_GGTT_PTE_PRESENT;
        if (ig9_ggtt_read_pte(first_entry + i) != want) {
            ok = false;
            break;
        }
    }
    if (!ok) {
        for (uint32_t i = 0; i < pages; ++i) {
            ig9_ggtt_write_pte(first_entry + i, s_empty_pte);
        }
        ig9_ggtt_invalidate();
        /* Keep the range reserved: its state could not be proven. */
        ig9_slots_set(slot_first, needed, true);
        ig9_spin_unlock(&s_lock);
        ig9_log("[IntelGen9] ggtt: PTE readback mismatch, range quarantined\n");
        return false;
    }

    ig9_slots_set(slot_first, needed, true);
    s_maps[map_idx].used = true;
    s_maps[map_idx].slot_first = slot_first;
    s_maps[map_idx].slot_count = needed;
    s_maps[map_idx].first_entry = first_entry;
    s_maps[map_idx].pages = pages;
    ig9_spin_unlock(&s_lock);

    *ggtt_out = first_entry * IG9_PAGE_BYTES;
    return true;
}

bool ig9_ggtt_unmap(uint32_t ggtt)
{
    if (!s_ready || (ggtt & (IG9_PAGE_BYTES - 1u)) != 0u) {
        return false;
    }
    ig9_spin_lock(&s_lock);
    for (uint32_t m = 0; m < IG9_GGTT_MAX_MAPPINGS; ++m) {
        ig9_ggtt_mapping_t *map = &s_maps[m];
        if (!map->used || map->first_entry * IG9_PAGE_BYTES != ggtt) {
            continue;
        }
        for (uint32_t i = 0; i < map->pages; ++i) {
            ig9_ggtt_write_pte(map->first_entry + i, s_empty_pte);
        }
        ig9_ggtt_invalidate();
        bool cleared = true;
        for (uint32_t i = 0; i < map->pages; ++i) {
            if (ig9_ggtt_read_pte(map->first_entry + i) != s_empty_pte) {
                cleared = false;
                break;
            }
        }
        if (cleared) {
            ig9_slots_set(map->slot_first, map->slot_count, false);
        }
        map->used = false;
        ig9_spin_unlock(&s_lock);
        if (!cleared) {
            ig9_log("[IntelGen9] ggtt: PTE clear failed, range quarantined\n");
        }
        return cleared;
    }
    ig9_spin_unlock(&s_lock);
    return false;
}

#endif /* IMPLUS_DRIVER_MODULE */
