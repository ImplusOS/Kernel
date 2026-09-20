#include "Drivers/Module/DriverBinary.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The output stream plays one cyclic DMA ring: HDA_RING_ENTRIES buffer
 * descriptors of HDA_ENTRY_BYTES each, looped by the controller for as long
 * as the stream runs. Software writes ahead of the hardware's link position
 * (LPIB) and silences what the hardware has already played, so a producer
 * that falls behind is heard as silence instead of the last lap of the ring.
 * 16 x 2 KiB = 32 KiB, about 170 ms of 48 kHz 16-bit stereo. */
#define HDA_ENTRY_BYTES  2048u
#define HDA_RING_ENTRIES 16u
#define HDA_RING_BYTES   (HDA_ENTRY_BYTES * HDA_RING_ENTRIES)
/* Never fill the last entry: keeps the write side from lapping the reader. */
#define HDA_RING_GUARD   HDA_ENTRY_BYTES
#define HDA_PERIOD_BYTES HDA_ENTRY_BYTES
#define HDA_PERIOD_COUNT HDA_RING_ENTRIES
#define HDA_BUFFER_BYTES HDA_RING_BYTES
#define HDA_TIMEOUT_MS   2000u

typedef struct __attribute__((packed)) {
    uint32_t address_low;
    uint32_t address_high;
    uint32_t length;
    uint32_t flags;
} hda_bdl_entry_t;

static const driver_binary_t *g_api;
static volatile uint8_t *g_regs;
static volatile uint8_t *g_stream;
static uint8_t g_codec;
static hda_bdl_entry_t *g_bdl;
static uint64_t g_bdl_phys;
static uint8_t *g_buffer;
static uint64_t g_buffer_phys;
static bool g_ready;
static bool g_open;

static uint16_t rd16(uint32_t off) { return *(volatile uint16_t *)(g_regs + off); }
static uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(g_regs + off); }
static void wr16(uint32_t off, uint16_t v) { *(volatile uint16_t *)(g_regs + off) = v; }
static void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(g_regs + off) = v; }

static uint32_t stream_ctl_read(void)
{
    return (uint32_t)g_stream[0] |
           ((uint32_t)g_stream[1] << 8u) |
           ((uint32_t)g_stream[2] << 16u);
}

static void stream_ctl_write(uint32_t value)
{
    *(volatile uint16_t *)(g_stream + 0x00u) = (uint16_t)value;
    *(volatile uint8_t *)(g_stream + 0x02u) = (uint8_t)(value >> 16u);
}

static bool stream_ctl_wait(uint32_t mask, uint32_t expected)
{
    uint64_t start = g_api->timer.monotonic_ns();
    while ((stream_ctl_read() & mask) != expected) {
        if (g_api->timer.monotonic_ns() - start >
            (uint64_t)HDA_TIMEOUT_MS * 1000000ULL) {
            return false;
        }
        g_api->hal.cpu_pause();
    }
    return true;
}

static bool hda_stream_reset(void)
{
    uint32_t ctl = stream_ctl_read() & ~(1u << 1u);
    stream_ctl_write(ctl);
    if (!stream_ctl_wait(1u << 1u, 0u)) {
        return false;
    }
    stream_ctl_write(ctl | 1u);
    if (!stream_ctl_wait(1u, 1u)) {
        return false;
    }
    stream_ctl_write(ctl & ~1u);
    return stream_ctl_wait(1u, 0u);
}

static bool wait32(uint32_t off, uint32_t mask, uint32_t expected)
{
    uint64_t start = g_api->timer.monotonic_ns();
    while ((rd32(off) & mask) != expected) {
        if (g_api->timer.monotonic_ns() - start >
            (uint64_t)HDA_TIMEOUT_MS * 1000000ULL) {
            return false;
        }
        g_api->hal.cpu_pause();
    }
    return true;
}

static bool hda_verb(uint8_t node, uint16_t verb, uint16_t payload,
                     uint32_t *response)
{
    if ((rd16(0x68u) & 1u) != 0u) {
        return false;
    }
    uint32_t command = ((uint32_t)g_codec << 28u) |
                       ((uint32_t)node << 20u) |
                       ((uint32_t)verb << 8u) |
                       payload;
    wr32(0x60u, command);
    wr16(0x68u, 1u);
    uint64_t start = g_api->timer.monotonic_ns();
    while ((rd16(0x68u) & 2u) == 0u) {
        if (g_api->timer.monotonic_ns() - start >
            (uint64_t)HDA_TIMEOUT_MS * 1000000ULL) {
            return false;
        }
    }
    if (response != NULL) {
        *response = rd32(0x64u);
    }
    wr16(0x68u, 2u);
    return true;
}

static bool hda_find(void)
{
#if !defined(PLATFORM_X86_64)
    return false;
#else
    uint32_t count = g_api->pci.get_device_count();
    for (uint32_t i = 0u; i < count; ++i) {
        driver_pci_device_t device;
        driver_pci_bar_t bar;
        if (!g_api->pci.get_device(i, &device) ||
            device.class_code != 0x04u || device.subclass != 0x03u ||
            !g_api->pci.get_bar(device.bus, device.device,
                                device.function, 0u, &bar) ||
            bar.is_io || bar.size < 0x1000u) {
            continue;
        }
        g_api->pci.enable_bus_master(device.bus, device.device,
                                     device.function);
        g_regs = g_api->hw.map_mmio_range(bar.address, (size_t)bar.size);
        return g_regs != NULL;
    }
    return false;
#endif
}

static bool hda_configure_codec(void)
{
    uint16_t codecs = rd16(0x0Eu);
    if (codecs == 0u) {
        return false;
    }
    for (g_codec = 0u; g_codec < 15u; ++g_codec) {
        if ((codecs & (1u << g_codec)) != 0u) {
            break;
        }
    }
    uint32_t response = 0u;
    if (!hda_verb(0u, 0xF00u, 0x00u, &response) ||
        response == 0u || response == 0xFFFFFFFFu) {
        return false;
    }
    (void)hda_verb(1u, 0x705u, 0u, NULL);
    (void)hda_verb(2u, 0x705u, 0u, NULL);
    (void)hda_verb(4u, 0x705u, 0u, NULL);
    (void)hda_verb(4u, 0x707u, 0x40u, NULL);
    (void)hda_verb(4u, 0x701u, 0u, NULL);
    (void)hda_verb(2u, 0x300u, 0x00u, NULL);
    (void)hda_verb(4u, 0x300u, 0x00u, NULL);
    return true;
}

static bool hda_init(void)
{
    if (g_ready) return true;
    if (!hda_find()) return false;
    wr32(0x08u, rd32(0x08u) & ~1u);
    if (!wait32(0x08u, 1u, 0u)) return false;
    wr32(0x08u, rd32(0x08u) | 1u);
    if (!wait32(0x08u, 1u, 1u)) return false;
    g_api->timer.msleep(10u);
    if (!hda_configure_codec()) return false;

    uint16_t gcap = rd16(0x00u);
    uint8_t input_streams = (uint8_t)((gcap >> 8u) & 0x0Fu);
    uint8_t output_streams = (uint8_t)((gcap >> 12u) & 0x0Fu);
    if (output_streams == 0u) return false;
    g_stream = g_regs + 0x80u + (uint32_t)input_streams * 0x20u;
    g_bdl = g_api->mem.dma_alloc_ex(
        sizeof(hda_bdl_entry_t) * HDA_RING_ENTRIES, 128u, 0u, &g_bdl_phys);
    g_buffer = g_api->mem.dma_alloc_ex(
        HDA_RING_BYTES, 128u, 0u, &g_buffer_phys);
    if (g_bdl == NULL || g_buffer == NULL) return false;
    g_ready = true;
    return true;
}

static bool hda_info(driver_audio_info_t *info)
{
    if (!g_ready || info == NULL) return false;
    info->sample_rate = 48000u;
    info->channels = 2u;
    info->format = DRIVER_AUDIO_FORMAT_S16_LE;
    info->period_bytes = HDA_PERIOD_BYTES;
    info->period_count = HDA_PERIOD_COUNT;
    return true;
}

/* Ring bookkeeping, all in absolute byte counts since hda_open(). */
static bool     g_running;
static uint64_t g_written;    /* bytes placed in the ring */
static uint64_t g_played;     /* bytes the hardware has consumed */
static uint32_t g_last_lpib;  /* LPIB at the last update */

static uint32_t hda_lpib(void)
{
    uint32_t lpib = *(volatile uint32_t *)(g_stream + 0x04u);
    return lpib < HDA_RING_BYTES ? lpib : 0u;
}

static void hda_ring_zero(uint64_t from_abs, uint64_t to_abs)
{
    while (from_abs < to_abs) {
        uint32_t off = (uint32_t)(from_abs % HDA_RING_BYTES);
        uint32_t n = HDA_RING_BYTES - off;
        if ((uint64_t)n > to_abs - from_abs) n = (uint32_t)(to_abs - from_abs);
        g_api->mem.memset(g_buffer + off, 0, n);
        from_abs += n;
    }
}

/* Fold the hardware's progress into g_played and silence what it has
 * played. Must run at least once per lap of the ring (~170 ms) while the
 * stream runs; the ALSA layer calls it from every timer tick. */
static void hda_update(void)
{
    if (!g_running) return;
    uint32_t lpib = hda_lpib();
    uint32_t delta = (lpib + HDA_RING_BYTES - g_last_lpib) % HDA_RING_BYTES;
    g_last_lpib = lpib;
    if (delta == 0u) return;
    uint64_t before = g_played;
    g_played += delta;
    hda_ring_zero(before, g_played);
    if (g_played > g_written) {
        /* Underrun: the hardware ran through silence. Restart the write
         * side where it is now. */
        g_written = g_played;
    }
}

static void hda_run(bool run)
{
    uint32_t ctl = stream_ctl_read();
    ctl = (ctl & ~(0xFu << 20u)) | (1u << 20u);
    if (run) ctl |= (1u << 1u); else ctl &= ~(1u << 1u);
    stream_ctl_write(ctl);
}

static bool hda_program_stream(void)
{
    if (!hda_stream_reset()) return false;
    for (uint32_t i = 0u; i < HDA_RING_ENTRIES; ++i) {
        uint64_t address = g_buffer_phys + (uint64_t)i * HDA_ENTRY_BYTES;
        g_bdl[i].address_low = (uint32_t)address;
        g_bdl[i].address_high = (uint32_t)(address >> 32u);
        g_bdl[i].length = HDA_ENTRY_BYTES;
        g_bdl[i].flags = 0u; /* no interrupt on completion: we poll LPIB */
    }
    g_api->mem.memset(g_buffer, 0, HDA_RING_BYTES);
    __sync_synchronize();
    *(volatile uint32_t *)(g_stream + 0x18u) = (uint32_t)g_bdl_phys;
    *(volatile uint32_t *)(g_stream + 0x1Cu) = (uint32_t)(g_bdl_phys >> 32u);
    *(volatile uint32_t *)(g_stream + 0x08u) = HDA_RING_BYTES;         /* CBL */
    *(volatile uint16_t *)(g_stream + 0x0Cu) = (uint16_t)(HDA_RING_ENTRIES - 1u); /* LVI */
    *(volatile uint16_t *)(g_stream + 0x12u) = 0x0011u; /* 48 kHz, 16-bit, 2 ch */
    *(volatile uint8_t *)(g_stream + 0x03u) = 0x1Cu;    /* clear status */
    hda_run(false);
    g_running = false;
    g_written = 0u;
    g_played = 0u;
    g_last_lpib = 0u;
    return true;
}

static bool hda_open(void)
{
    if (!g_ready || g_open) return false;
    if (!hda_program_stream()) return false;
    (void)hda_verb(2u, 0x706u, 0x10u, NULL);  /* DAC: stream tag 1, channel 0 */
    (void)hda_verb(2u, 0x200u, 0x11u, NULL);  /* DAC: 48 kHz 16-bit stereo */
    g_open = true;
    return true;
}

static uint64_t hda_stream_queued(void)
{
    if (!g_open) return 0u;
    hda_update();
    return g_written - g_played;
}

static uint64_t hda_stream_space(void)
{
    if (!g_open) return 0u;
    uint64_t queued = hda_stream_queued();
    uint64_t cap = HDA_RING_BYTES - HDA_RING_GUARD;
    return queued >= cap ? 0u : ((cap - queued) & ~3ull);
}

static uint64_t hda_stream_write(const void *pcm, uint64_t bytes)
{
    if (!g_open || pcm == NULL) return 0u;
    uint64_t space = hda_stream_space();
    if (bytes > space) bytes = space;
    bytes &= ~3ull;
    const uint8_t *src = pcm;
    uint64_t done = 0u;
    while (done < bytes) {
        uint32_t off = (uint32_t)(g_written % HDA_RING_BYTES);
        uint32_t n = HDA_RING_BYTES - off;
        if ((uint64_t)n > bytes - done) n = (uint32_t)(bytes - done);
        g_api->mem.memcpy(g_buffer + off, src + done, n);
        g_written += n;
        done += n;
    }
    __sync_synchronize();
    return done;
}

static bool hda_stream_start(void)
{
    if (!g_open) return false;
    if (!g_running) {
        g_last_lpib = hda_lpib();
        hda_run(true);
        g_running = true;
    }
    return true;
}

static void hda_stream_stop(void)
{
    if (!g_open) return;
    hda_run(false);
    g_running = false;
    /* Discard what is queued and restart the ring from the top. */
    (void)hda_program_stream();
}

/* The original blocking interface (native os_audio_write): feed the ring
 * until everything is queued. */
static int64_t hda_write(const void *pcm, uint64_t bytes)
{
    if (!g_open || pcm == NULL || bytes == 0u || (bytes & 3u) != 0u) return -1;
    const uint8_t *src = pcm;
    uint64_t done = 0u;
    uint64_t start = g_api->timer.monotonic_ns();
    while (done < bytes) {
        uint64_t n = hda_stream_write(src + done, bytes - done);
        done += n;
        if (!g_running && (done == bytes ||
                           hda_stream_queued() >= HDA_RING_BYTES / 2u)) {
            (void)hda_stream_start();
        }
        if (done < bytes) {
            if (g_api->timer.monotonic_ns() - start >
                (uint64_t)HDA_TIMEOUT_MS * 1000000ULL) {
                break;
            }
            g_api->timer.msleep(2u);
        }
    }
    return done == 0u ? -1 : (int64_t)done;
}

static bool hda_drain(uint32_t timeout)
{
    if (!g_open) return false;
    if (!g_running && hda_stream_queued() != 0u) (void)hda_stream_start();
    uint64_t start = g_api->timer.monotonic_ns();
    while (hda_stream_queued() != 0u) {
        if (g_api->timer.monotonic_ns() - start >
            (uint64_t)(timeout != 0u ? timeout : HDA_TIMEOUT_MS) * 1000000ULL) {
            return false;
        }
        g_api->timer.msleep(2u);
    }
    return true;
}

static void hda_close(void) {
    if (g_stream != NULL) {
        hda_run(false);
    }
    g_running = false;
    g_open = false;
}
static bool hda_is_ready(void) { return g_ready; }

static const driver_audio_t g_audio = {
    .name = "hda", .priority = 20u, .init = hda_init,
    .is_ready = hda_is_ready, .get_info = hda_info, .open = hda_open,
    .write = hda_write, .drain = hda_drain, .close = hda_close,
    .stream_write = hda_stream_write, .stream_queued = hda_stream_queued,
    .stream_space = hda_stream_space, .stream_start = hda_stream_start,
    .stream_stop = hda_stream_stop,
};

static const driver_module_descriptor_t g_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC, .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_AUDIO, .load_priority = 44u,
    .deps = { "PCI_Driver.ELF", NULL }, .driver_api = &g_audio,
    .shutdown = hda_close,
};

__attribute__((visibility("default")))
const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL || api->version_major != DRIVER_API_VERSION_MAJOR) return NULL;
    g_api = api;
    return &g_module;
}
