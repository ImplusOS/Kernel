#include "Drivers/Module/DriverBinary.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AC97_PERIOD_BYTES 2048u
#define AC97_PERIOD_COUNT 4u
#define AC97_TIMEOUT_MS   2000u

typedef struct __attribute__((packed)) {
    uint32_t address;
    uint16_t samples;
    uint16_t control;
} ac97_bdl_entry_t;

static const driver_binary_t *g_api;
static uint16_t g_nam;
static uint16_t g_nabm;
static ac97_bdl_entry_t *g_bdl;
static uint64_t g_bdl_phys;
static uint8_t *g_periods;
static uint64_t g_periods_phys;
static bool g_ready;
static bool g_open;
static uint32_t g_bdl_write_idx;

static bool ac97_find(void)
{
#if !defined(PLATFORM_X86_64)
    return false;
#else
    uint32_t count = g_api->pci.get_device_count();
    for (uint32_t i = 0u; i < count; ++i) {
        driver_pci_device_t device;
        driver_pci_bar_t nam;
        driver_pci_bar_t nabm;
        if (!g_api->pci.get_device(i, &device) ||
            device.class_code != 0x04u || device.subclass != 0x01u ||
            !g_api->pci.get_bar(device.bus, device.device,
                                device.function, 0u, &nam) ||
            !g_api->pci.get_bar(device.bus, device.device,
                                device.function, 1u, &nabm) ||
            !nam.is_io || !nabm.is_io ||
            nam.address > UINT16_MAX || nabm.address > UINT16_MAX) {
            continue;
        }
        g_nam = (uint16_t)nam.address;
        g_nabm = (uint16_t)nabm.address;
        g_api->pci.enable_bus_master(device.bus, device.device,
                                     device.function);
        return true;
    }
    return false;
#endif
}

static bool ac97_init(void)
{
    if (g_ready) {
        return true;
    }
    if (!ac97_find()) {
        return false;
    }
    g_bdl = g_api->mem.dma_alloc_ex(
        sizeof(ac97_bdl_entry_t) * 32u, 16u, UINT32_MAX, &g_bdl_phys);
    g_periods = g_api->mem.dma_alloc_ex(
        AC97_PERIOD_BYTES * AC97_PERIOD_COUNT, 16u, UINT32_MAX,
        &g_periods_phys);
    if (g_bdl == NULL || g_periods == NULL) {
        return false;
    }
    for (uint32_t i = 0u; i < AC97_PERIOD_COUNT; ++i) {
        g_bdl[i].address =
            (uint32_t)(g_periods_phys + (uint64_t)i * AC97_PERIOD_BYTES);
        g_bdl[i].samples = AC97_PERIOD_BYTES / 2u;
        g_bdl[i].control = 0x8000u;
    }
    g_api->hal.io_out32((uint16_t)(g_nabm + 0x2Cu), 0x02u);
    g_api->timer.msleep(20u);
    g_api->hal.io_out16((uint16_t)(g_nam + 0x00u), 0x0000u);
    g_api->hal.io_out16((uint16_t)(g_nam + 0x02u), 0x0000u);
    g_api->hal.io_out16((uint16_t)(g_nam + 0x18u), 0x0000u);
    g_api->hal.io_out32((uint16_t)(g_nabm + 0x10u),
                         (uint32_t)g_bdl_phys);
    g_bdl_write_idx = 0;
    g_ready = true;
    return true;
}

static bool ac97_get_info(driver_audio_info_t *info)
{
    if (info == NULL || !g_ready) {
        return false;
    }
    info->sample_rate = 48000u;
    info->channels = 2u;
    info->format = DRIVER_AUDIO_FORMAT_S16_LE;
    info->period_bytes = AC97_PERIOD_BYTES;
    info->period_count = AC97_PERIOD_COUNT;
    return true;
}

static bool ac97_open(void)
{
    if (!g_ready || g_open) {
        return false;
    }
    g_bdl_write_idx = 0;
    g_api->hal.io_out8((uint16_t)(g_nabm + 0x1Bu), 0x02u);
    g_api->timer.msleep(1u);
    g_api->hal.io_out16((uint16_t)(g_nabm + 0x16u), 0x001Cu);
    g_open = true;
    return true;
}

static int64_t ac97_write(const void *pcm, uint64_t bytes)
{
    if (!g_open || pcm == NULL || bytes == 0u || (bytes & 3u) != 0u) {
        return -1;
    }

    uint8_t ctrl = g_api->hal.io_in8((uint16_t)(g_nabm + 0x1Bu));
    if ((ctrl & 0x01u) != 0u) {
        uint16_t status = g_api->hal.io_in16((uint16_t)(g_nabm + 0x16u));
        if ((status & 0x08u) == 0u) {
            return 0;
        }
        g_api->hal.io_out16((uint16_t)(g_nabm + 0x16u), 0x001Cu);
    }

    const uint8_t *source = (const uint8_t *)pcm;
    uint64_t remaining = bytes;
    uint64_t done = 0u;

    while (remaining > 0u && g_bdl_write_idx < AC97_PERIOD_COUNT) {
        uint32_t chunk = (uint32_t)(remaining);
        if (chunk > AC97_PERIOD_BYTES) {
            chunk = AC97_PERIOD_BYTES;
        }

        g_api->mem.memcpy(
            g_periods + (uint64_t)g_bdl_write_idx * AC97_PERIOD_BYTES,
            source + done, chunk);
        if (chunk < AC97_PERIOD_BYTES) {
            g_api->mem.memset(
                g_periods + (uint64_t)g_bdl_write_idx * AC97_PERIOD_BYTES + chunk,
                0, AC97_PERIOD_BYTES - chunk);
        }

        g_bdl[g_bdl_write_idx].address =
            (uint32_t)(g_periods_phys +
                        (uint64_t)g_bdl_write_idx * AC97_PERIOD_BYTES);
        g_bdl[g_bdl_write_idx].samples = AC97_PERIOD_BYTES / 2u;
        g_bdl[g_bdl_write_idx].control = 0x8000u;

        ++g_bdl_write_idx;
        done += chunk;
        remaining -= chunk;

        if (chunk < AC97_PERIOD_BYTES) {
            break;
        }
    }

    if (g_bdl_write_idx > 0) {
        g_api->hal.io_out8((uint16_t)(g_nabm + 0x1Bu), 0x02u);
        g_api->hal.io_out32((uint16_t)(g_nabm + 0x10u),
                             (uint32_t)g_bdl_phys);
        g_api->hal.io_out8((uint16_t)(g_nabm + 0x15u),
                            (uint8_t)(g_bdl_write_idx - 1));
        g_api->hal.io_out16((uint16_t)(g_nabm + 0x16u), 0x001Cu);
        g_api->hal.io_out8((uint16_t)(g_nabm + 0x1Bu), 0x15u);
        g_bdl_write_idx = 0;
    }

    return (int64_t)done;
}

static bool ac97_drain(uint32_t timeout_ms)
{
    if (!g_open) return false;
    uint64_t deadline = g_api->timer.monotonic_ns() +
                         (uint64_t)timeout_ms * 1000000ULL;
    for (;;) {
        uint16_t status = g_api->hal.io_in16((uint16_t)(g_nabm + 0x16u));
        if ((status & 0x08u) != 0u) {
            g_api->hal.io_out16((uint16_t)(g_nabm + 0x16u), 0x001Cu);
            return true;
        }
        if (g_api->timer.monotonic_ns() >= deadline) break;
        g_api->hal.cpu_pause();
    }
    return false;
}

static void ac97_close(void)
{
    if (g_ready) {
        g_api->hal.io_out8((uint16_t)(g_nabm + 0x1Bu), 0x00u);
    }
    g_open = false;
}

static bool ac97_is_ready(void) { return g_ready; }

static const driver_audio_t g_audio = {
    .name = "ac97",
    .priority = 30u,
    .init = ac97_init,
    .is_ready = ac97_is_ready,
    .get_info = ac97_get_info,
    .open = ac97_open,
    .write = ac97_write,
    .drain = ac97_drain,
    .close = ac97_close,
};

static void ac97_shutdown(void)
{
    ac97_close();
    g_ready = false;
}

static const driver_module_descriptor_t g_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_AUDIO,
    .load_priority = 45u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_audio,
    .shutdown = ac97_shutdown,
};

__attribute__((visibility("default")))
const driver_module_descriptor_t *driver_module_init(
    const driver_binary_t *api)
{
    if (api == NULL || api->version_major != DRIVER_API_VERSION_MAJOR ||
        api->pci.get_device_count == NULL || api->mem.dma_alloc_ex == NULL) {
        return NULL;
    }
    g_api = api;
    return &g_module;
}
