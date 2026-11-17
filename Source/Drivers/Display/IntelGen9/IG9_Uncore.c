/*
 * IG9_Uncore.c - MMIO access, forcewake, PCI power state and the small
 * utility layer (logging, time, cache maintenance) of the IntelGen9 driver.
 *
 * Forcewake follows i915's Gen9 sequence (intel_uncore.c):
 *   wait ACK.KERNEL clear -> write MASKED_ENABLE(KERNEL) -> wait ACK.KERNEL
 * and, when the acknowledge does not arrive, the documented recovery kick
 * that toggles FORCEWAKE_KERNEL_FALLBACK (fw_domain_wait_ack_with_fallback).
 * Domains are reference counted so nested users cannot power a domain down
 * underneath one another.
 */

#ifdef IMPLUS_DRIVER_MODULE

#include "IG9_Private.h"

const driver_binary_t *g_ig9_api = NULL;
ig9_match_t g_ig9_match;
bool g_ig9_have_match = false;
ig9_bars_t g_ig9_bars;

static volatile uint8_t *s_mmio = NULL;

#ifdef IG9_HOST_SIM
/* The host-side simulator (Tools/IntelGen9Sim) models the device behind
 * these two hooks instead of a mapped BAR. */
uint32_t ig9_sim_mmio_read32(uint32_t offset);
void ig9_sim_mmio_write32(uint32_t offset, uint32_t value);
#endif

/* ------------------------------------------------------------------ *
 * Memory helpers
 * ------------------------------------------------------------------ */

void *ig9_memset(void *dst, int value, size_t n)
{
    if (g_ig9_api != NULL && g_ig9_api->memset != NULL) {
        return g_ig9_api->memset(dst, value, n);
    }
    volatile uint8_t *p = (volatile uint8_t *)dst;
    for (size_t i = 0; i < n; ++i) {
        p[i] = (uint8_t)value;
    }
    return dst;
}

void ig9_memcpy(void *dst, const void *src, size_t n)
{
    if (g_ig9_api != NULL && g_ig9_api->memcpy != NULL) {
        g_ig9_api->memcpy(dst, src, n);
        return;
    }
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    for (size_t i = 0; i < n; ++i) {
        d[i] = s[i];
    }
}

/* ------------------------------------------------------------------ *
 * Time
 * ------------------------------------------------------------------ */

uint64_t ig9_now_ns(void)
{
    if (g_ig9_api == NULL) {
        return 0u;
    }
    if (g_ig9_api->timer.monotonic_ns != NULL) {
        return g_ig9_api->timer.monotonic_ns();
    }
    return 0u;
}

void ig9_msleep(uint32_t ms)
{
    if (g_ig9_api == NULL) {
        return;
    }
    if (g_ig9_api->timer.msleep != NULL) {
        g_ig9_api->timer.msleep(ms);
    } else if (g_ig9_api->timer_msleep != NULL) {
        g_ig9_api->timer_msleep(ms);
    }
}

static void ig9_cpu_relax(void)
{
#if defined(__x86_64__)
    __asm__ __volatile__("pause" ::: "memory");
#elif defined(__aarch64__)
    __asm__ __volatile__("yield" ::: "memory");
#else
    __atomic_signal_fence(__ATOMIC_SEQ_CST);
#endif
}

void ig9_udelay(uint32_t us)
{
    uint64_t start = ig9_now_ns();
    if (start == 0u && ig9_now_ns() == 0u) {
        /* No clock: a coarse spin. Only used for the ~10-100 us gaps of
         * the forcewake recovery kick, where "at least" is what matters. */
        for (uint64_t i = 0; i < (uint64_t)us * 1000u; ++i) {
            ig9_cpu_relax();
        }
        return;
    }
    while (ig9_now_ns() - start < (uint64_t)us * 1000u) {
        ig9_cpu_relax();
    }
}

void ig9_deadline_start(ig9_deadline_t *d, uint32_t timeout_us)
{
    d->start_ns = ig9_now_ns();
    d->timeout_ns = (uint64_t)timeout_us * 1000u;
    d->polls = 0u;
    d->has_clock = d->start_ns != 0u;
}

bool ig9_deadline_expired(ig9_deadline_t *d)
{
    ++d->polls;
    if (!d->has_clock) {
        /* Each poll sleeps 1 ms; the first few spin so short waits stay
         * short even without a clock. */
        if (d->polls <= 64u) {
            for (uint32_t i = 0; i < 64u; ++i) {
                ig9_cpu_relax();
            }
            return false;
        }
        ig9_msleep(1u);
        return (uint64_t)(d->polls - 64u) * 1000000u >= d->timeout_ns;
    }

    uint64_t elapsed = ig9_now_ns() - d->start_ns;
    if (elapsed >= d->timeout_ns) {
        return true;
    }
    if (elapsed < 2000000u) {
        /* The common case (a blit finishing, an ack arriving) completes in
         * microseconds: spin rather than paying a 1 ms sleep granule. */
        for (uint32_t i = 0; i < 32u; ++i) {
            ig9_cpu_relax();
        }
    } else {
        ig9_msleep(1u);
    }
    return false;
}

/* ------------------------------------------------------------------ *
 * Cache maintenance / locking
 * ------------------------------------------------------------------ */

void ig9_mb(void)
{
#if defined(__x86_64__)
    __asm__ __volatile__("mfence" ::: "memory");
#elif defined(__aarch64__)
    __asm__ __volatile__("dsb sy" ::: "memory");
#else
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

void ig9_clflush_range(const void *base, uint64_t bytes)
{
    if (base == NULL || bytes == 0u) {
        return;
    }
#if defined(__x86_64__) && !defined(IG9_HOST_SIM)
    uintptr_t p = (uintptr_t)base & ~(uintptr_t)63u;
    uintptr_t end = (uintptr_t)base + (uintptr_t)bytes;
    __asm__ __volatile__("mfence" ::: "memory");
    for (; p < end; p += 64u) {
        __asm__ __volatile__("clflush (%0)" : : "r"((const void *)p) : "memory");
    }
    __asm__ __volatile__("mfence" ::: "memory");
#else
    (void)base;
    (void)bytes;
    ig9_mb();
#endif
}

void ig9_spin_lock(ig9_spinlock_t *l)
{
    while (__atomic_exchange_n(&l->lock, 1u, __ATOMIC_ACQUIRE) != 0u) {
        while (__atomic_load_n(&l->lock, __ATOMIC_RELAXED) != 0u) {
            ig9_cpu_relax();
        }
    }
}

void ig9_spin_unlock(ig9_spinlock_t *l)
{
    __atomic_store_n(&l->lock, 0u, __ATOMIC_RELEASE);
}

/* ------------------------------------------------------------------ *
 * MMIO
 * ------------------------------------------------------------------ */

bool ig9_uncore_ready(void)
{
    return s_mmio != NULL;
}

uint32_t ig9_read32(uint32_t offset)
{
    if (s_mmio == NULL || (uint64_t)offset + 4u > g_ig9_bars.mmio_size) {
        return 0u;
    }
#ifdef IG9_HOST_SIM
    return ig9_sim_mmio_read32(offset);
#else
    return *(volatile uint32_t *)(const volatile void *)(s_mmio + offset);
#endif
}

void ig9_write32(uint32_t offset, uint32_t value)
{
    if (s_mmio == NULL || (uint64_t)offset + 4u > g_ig9_bars.mmio_size) {
        return;
    }
#ifdef IG9_HOST_SIM
    ig9_sim_mmio_write32(offset, value);
#else
    *(volatile uint32_t *)(volatile void *)(s_mmio + offset) = value;
#endif
}

bool ig9_wait_reg(uint32_t offset, uint32_t mask, uint32_t value,
                  uint32_t timeout_us)
{
    ig9_deadline_t d;
    ig9_deadline_start(&d, timeout_us);
    for (;;) {
        if ((ig9_read32(offset) & mask) == value) {
            return true;
        }
        if (ig9_deadline_expired(&d)) {
            return (ig9_read32(offset) & mask) == value;
        }
    }
}

uint32_t ig9_pci_read(uint16_t offset)
{
    if (g_ig9_api == NULL || g_ig9_api->pci.read_config == NULL) {
        return 0xFFFFFFFFu;
    }
    return g_ig9_api->pci.read_config(g_ig9_match.bus, g_ig9_match.device,
                                      g_ig9_match.function, offset);
}

/*
 * A function parked in D3 can hang the bus when its BAR is read, so before
 * the first MMIO access: memory decode + bus mastering on, and D0 if the
 * function has a PM capability.
 */
static bool ig9_pci_prepare(void)
{
    const driver_binary_t *api = g_ig9_api;
    uint8_t bus = g_ig9_match.bus;
    uint8_t dev = g_ig9_match.device;
    uint8_t fn = g_ig9_match.function;

    if (api->pci.read_config == NULL || api->pci.write_config == NULL) {
        return false;
    }

    if (api->pci.find_capability != NULL) {
        int32_t pm_cap = api->pci.find_capability(bus, dev, fn, 0x01u);
        if (pm_cap > 0) {
            uint16_t pmcsr_off = (uint16_t)((uint32_t)pm_cap + 4u);
            uint32_t pmcsr = api->pci.read_config(bus, dev, fn, pmcsr_off);
            if ((pmcsr & 0x3u) != 0u) {
                api->pci.write_config(bus, dev, fn, pmcsr_off, pmcsr & ~0x3u);
                ig9_msleep(10u);   /* D3hot -> D0 needs 10 ms (PCI PM 1.2) */
                pmcsr = api->pci.read_config(bus, dev, fn, pmcsr_off);
                if ((pmcsr & 0x3u) != 0u) {
                    return false;
                }
            }
        }
    }

    uint32_t command = api->pci.read_config(bus, dev, fn, 0x04u);
    if (command == 0xFFFFFFFFu) {
        return false;
    }
    if ((command & 0x6u) != 0x6u) {
        api->pci.write_config(bus, dev, fn, 0x04u, command | 0x6u);
    }
    return true;
}

bool ig9_uncore_init(void)
{
    const driver_binary_t *api = g_ig9_api;
    if (s_mmio != NULL) {
        return true;
    }
    if (api == NULL || api->pci.get_bar == NULL ||
        api->hw.map_mmio_range == NULL) {
        return false;
    }

    driver_pci_bar_t bar0;
    ig9_memset(&bar0, 0, sizeof(bar0));
    if (!api->pci.get_bar(g_ig9_match.bus, g_ig9_match.device,
                          g_ig9_match.function, IG9_PCI_BAR_GTTMMADR, &bar0) ||
        bar0.address == 0u || bar0.is_io != 0u ||
        bar0.size < 4u * 1024u * 1024u || bar0.size > 64u * 1024u * 1024u ||
        (bar0.size & (bar0.size - 1u)) != 0u) {
        return false;
    }

    driver_pci_bar_t bar2;
    ig9_memset(&bar2, 0, sizeof(bar2));
    if (api->pci.get_bar(g_ig9_match.bus, g_ig9_match.device,
                         g_ig9_match.function, IG9_PCI_BAR_GMADR, &bar2) &&
        bar2.address != 0u && bar2.is_io == 0u && bar2.size != 0u) {
        g_ig9_bars.gmadr_phys = bar2.address;
        g_ig9_bars.gmadr_size = bar2.size;
    }

    if (!ig9_pci_prepare()) {
        return false;
    }

    volatile uint8_t *mmio =
        (volatile uint8_t *)api->hw.map_mmio_range(bar0.address,
                                                   (size_t)bar0.size);
    if (mmio == NULL) {
        return false;
    }

    g_ig9_bars.mmio_phys = bar0.address;
    g_ig9_bars.mmio_size = bar0.size;
    s_mmio = mmio;

    return true;
}

/* ------------------------------------------------------------------ *
 * Forcewake
 * ------------------------------------------------------------------ */

typedef struct {
    uint32_t set_reg;
    uint32_t ack_reg;
    uint32_t refs;
    const char *name;
} ig9_fw_domain_t;

static ig9_fw_domain_t s_fw_domains[2] = {
    { IG9_FORCEWAKE_GT,     IG9_FORCEWAKE_ACK_GT,     0u, "gt" },
    { IG9_FORCEWAKE_RENDER, IG9_FORCEWAKE_ACK_RENDER, 0u, "render" },
};

#define IG9_FW_ACK_TIMEOUT_US 50000u

static bool ig9_fw_wait_ack(const ig9_fw_domain_t *d, uint32_t bit, bool set)
{
    return ig9_wait_reg(d->ack_reg, bit, set ? bit : 0u, IG9_FW_ACK_TIMEOUT_US);
}

/* fw_domain_wait_ack_with_fallback(): kick the power-well state machine
 * by cycling the fallback bit until the kernel bit's ack shows up. */
static bool ig9_fw_ack_fallback(const ig9_fw_domain_t *d)
{
    for (uint32_t pass = 1u; pass <= 10u; ++pass) {
        (void)ig9_wait_reg(d->ack_reg, IG9_FORCEWAKE_KERNEL_FALLBACK, 0u, 1000u);
        ig9_write32(d->set_reg, IG9_MASKED_ENABLE(IG9_FORCEWAKE_KERNEL_FALLBACK));
        ig9_udelay(10u * pass);
        (void)ig9_wait_reg(d->ack_reg, IG9_FORCEWAKE_KERNEL_FALLBACK,
                           IG9_FORCEWAKE_KERNEL_FALLBACK, 1000u);
        bool acked = (ig9_read32(d->ack_reg) & IG9_FORCEWAKE_KERNEL) != 0u;
        ig9_write32(d->set_reg, IG9_MASKED_DISABLE(IG9_FORCEWAKE_KERNEL_FALLBACK));
        if (acked) {
            return true;
        }
    }
    return false;
}

static bool ig9_fw_domain_get(ig9_fw_domain_t *d)
{
    if (d->refs != 0u) {
        ++d->refs;
        return true;
    }

    /* A stale ack from a previous owner; i915 warns and carries on. */
    (void)ig9_fw_wait_ack(d, IG9_FORCEWAKE_KERNEL, false);
    ig9_write32(d->set_reg, IG9_MASKED_ENABLE(IG9_FORCEWAKE_KERNEL));
    if (!ig9_fw_wait_ack(d, IG9_FORCEWAKE_KERNEL, true) &&
        !ig9_fw_ack_fallback(d)) {
        ig9_write32(d->set_reg, IG9_MASKED_DISABLE(IG9_FORCEWAKE_KERNEL));
        return false;
    }
    d->refs = 1u;
    return true;
}

static void ig9_fw_domain_put(ig9_fw_domain_t *d)
{
    if (d->refs == 0u) {
        return;
    }
    if (--d->refs == 0u) {
        ig9_write32(d->set_reg, IG9_MASKED_DISABLE(IG9_FORCEWAKE_KERNEL));
    }
}

bool ig9_forcewake_get(uint32_t domains)
{
    if (s_mmio == NULL) {
        return false;
    }
    if ((domains & IG9_FW_GT) != 0u && !ig9_fw_domain_get(&s_fw_domains[0])) {
        return false;
    }
    if ((domains & IG9_FW_RENDER) != 0u &&
        !ig9_fw_domain_get(&s_fw_domains[1])) {
        if ((domains & IG9_FW_GT) != 0u) {
            ig9_fw_domain_put(&s_fw_domains[0]);
        }
        return false;
    }
    return true;
}

void ig9_forcewake_put(uint32_t domains)
{
    if ((domains & IG9_FW_RENDER) != 0u) {
        ig9_fw_domain_put(&s_fw_domains[1]);
    }
    if ((domains & IG9_FW_GT) != 0u) {
        ig9_fw_domain_put(&s_fw_domains[0]);
    }
}

#endif /* IMPLUS_DRIVER_MODULE */
