#include "Drivers/Module/DriverBinary.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VIRTIO_VENDOR_ID          0x1AF4u
#define VIRTIO_SOUND_MODERN_ID    0x1059u
#define PCI_CAP_VENDOR            0x09u

#define VIRTIO_CAP_COMMON         1u
#define VIRTIO_CAP_NOTIFY         2u
#define VIRTIO_CAP_DEVICE         4u

#define VIRTIO_STATUS_ACK         0x01u
#define VIRTIO_STATUS_DRIVER      0x02u
#define VIRTIO_STATUS_DRIVER_OK   0x04u
#define VIRTIO_STATUS_FEATURES_OK 0x08u
#define VIRTIO_STATUS_FAILED      0x80u

#define VIRTQ_DESC_NEXT           1u
#define VIRTQ_DESC_WRITE          2u
#define SOUND_QUEUE_COUNT         4u
#define SOUND_QUEUE_MAX           16u
#define SOUND_CONTROL_QUEUE       0u
#define SOUND_EVENT_QUEUE         1u
#define SOUND_TX_QUEUE            2u
#define SOUND_RX_QUEUE            3u

#define SOUND_PERIOD_BYTES        2048u
#define SOUND_PERIOD_COUNT        4u
#define SOUND_BUFFER_BYTES        (SOUND_PERIOD_BYTES * SOUND_PERIOD_COUNT)
#define SOUND_TIMEOUT_MS          3000u

#define VIRTIO_SND_R_PCM_INFO       0x0100u
#define VIRTIO_SND_R_PCM_SET_PARAMS 0x0101u
#define VIRTIO_SND_R_PCM_PREPARE    0x0102u
#define VIRTIO_SND_R_PCM_RELEASE    0x0103u
#define VIRTIO_SND_R_PCM_START      0x0104u
#define VIRTIO_SND_R_PCM_STOP       0x0105u
#define VIRTIO_SND_S_OK             0x8000u
#define VIRTIO_SND_D_OUTPUT         0u
#define VIRTIO_SND_PCM_FMT_S16      5u
#define VIRTIO_SND_PCM_RATE_48000   7u

#define COMMON_DEVICE_FEATURE_SELECT 0u
#define COMMON_DEVICE_FEATURE        4u
#define COMMON_DRIVER_FEATURE_SELECT 8u
#define COMMON_DRIVER_FEATURE       12u
#define COMMON_DEVICE_STATUS        20u
#define COMMON_QUEUE_SELECT         22u
#define COMMON_QUEUE_SIZE           24u
#define COMMON_QUEUE_MSIX_VECTOR    26u
#define COMMON_QUEUE_ENABLE         28u
#define COMMON_QUEUE_NOTIFY_OFF     30u
#define COMMON_QUEUE_DESC           32u
#define COMMON_QUEUE_DRIVER         40u
#define COMMON_QUEUE_DEVICE         48u

typedef struct __attribute__((packed)) {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
} virtq_desc_t;

typedef struct __attribute__((packed)) {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[SOUND_QUEUE_MAX];
} virtq_avail_t;

typedef struct __attribute__((packed)) {
    uint32_t id;
    uint32_t len;
} virtq_used_elem_t;

typedef struct __attribute__((packed)) {
    uint16_t flags;
    uint16_t idx;
    virtq_used_elem_t ring[SOUND_QUEUE_MAX];
} virtq_used_t;

typedef struct {
    virtq_desc_t *desc;
    virtq_avail_t *avail;
    virtq_used_t *used;
    uint64_t desc_phys;
    uint64_t avail_phys;
    uint64_t used_phys;
    volatile uint16_t *notify;
    uint16_t size;
    uint16_t avail_idx;
    uint16_t used_idx;
} sound_queue_t;

typedef struct __attribute__((packed)) {
    uint32_t code;
} sound_hdr_t;

typedef struct __attribute__((packed)) {
    sound_hdr_t hdr;
    uint32_t start_id;
    uint32_t count;
    uint32_t size;
} sound_query_info_t;

typedef struct __attribute__((packed)) {
    uint32_t hda_fn_nid;
    uint32_t features;
    uint64_t formats;
    uint64_t rates;
    uint8_t direction;
    uint8_t channels_min;
    uint8_t channels_max;
    uint8_t padding[5];
} sound_pcm_info_t;

typedef struct __attribute__((packed)) {
    sound_hdr_t hdr;
    uint32_t stream_id;
} sound_pcm_hdr_t;

typedef struct __attribute__((packed)) {
    sound_pcm_hdr_t hdr;
    uint32_t buffer_bytes;
    uint32_t period_bytes;
    uint32_t features;
    uint8_t channels;
    uint8_t format;
    uint8_t rate;
    uint8_t padding;
} sound_set_params_t;

typedef struct __attribute__((packed)) {
    uint32_t stream_id;
} sound_pcm_xfer_t;

typedef struct __attribute__((packed)) {
    uint32_t status;
    uint32_t latency_bytes;
} sound_pcm_status_t;

static const driver_binary_t *g_api;
static driver_pci_device_t g_pci;
static volatile uint8_t *g_common;
static volatile uint8_t *g_notify;
static volatile uint8_t *g_device;
static uint32_t g_notify_multiplier;
static sound_queue_t g_queues[SOUND_QUEUE_COUNT];
static uint8_t *g_control_request;
static uint64_t g_control_request_phys;
static uint8_t *g_control_response;
static uint64_t g_control_response_phys;
static uint8_t *g_pcm;
static uint64_t g_pcm_phys;
static sound_pcm_xfer_t *g_xfer;
static uint64_t g_xfer_phys;
static sound_pcm_status_t *g_pcm_status;
static uint64_t g_pcm_status_phys;
static uint8_t *g_event;
static uint64_t g_event_phys;
static uint32_t g_stream_id;
static bool g_ready;
static bool g_open;

static uint32_t pci_read32(uint16_t offset)
{
    return g_api->pci.read_config(g_pci.bus, g_pci.device, g_pci.function,
                                  (uint16_t)(offset & 0xFCu));
}

static uint8_t pci_read8(uint16_t offset)
{
    return (uint8_t)(pci_read32(offset) >> ((offset & 3u) * 8u));
}

static uint16_t common_read16(uint32_t offset)
{
    return *(volatile uint16_t *)(g_common + offset);
}

static uint32_t common_read32(uint32_t offset)
{
    return *(volatile uint32_t *)(g_common + offset);
}

static void common_write8(uint32_t offset, uint8_t value)
{
    *(volatile uint8_t *)(g_common + offset) = value;
}

static void common_write16(uint32_t offset, uint16_t value)
{
    *(volatile uint16_t *)(g_common + offset) = value;
}

static void common_write32(uint32_t offset, uint32_t value)
{
    *(volatile uint32_t *)(g_common + offset) = value;
}

static void common_write64(uint32_t offset, uint64_t value)
{
    *(volatile uint64_t *)(g_common + offset) = value;
}

static bool sound_find_device(void)
{
    uint32_t count = g_api->pci.get_device_count();
    for (uint32_t i = 0u; i < count; ++i) {
        driver_pci_device_t device;
        if (g_api->pci.get_device(i, &device) &&
            device.vendor_id == VIRTIO_VENDOR_ID &&
            device.device_id == VIRTIO_SOUND_MODERN_ID) {
            g_pci = device;
            return true;
        }
    }
    return false;
}

static bool sound_map_cap(uint16_t cap, uint8_t expected_type,
                          volatile uint8_t **out, uint32_t *notify_mult)
{
    if (pci_read8(cap + 3u) != expected_type) {
        return false;
    }
    uint8_t bar_index = pci_read8(cap + 4u);
    uint32_t offset = pci_read32(cap + 8u);
    uint32_t length = pci_read32(cap + 12u);
    driver_pci_bar_t bar;
    if (!g_api->pci.get_bar(g_pci.bus, g_pci.device, g_pci.function,
                            bar_index, &bar) ||
        bar.is_io || offset > bar.size || length > bar.size - offset) {
        return false;
    }
    *out = g_api->hw.map_mmio_range(bar.address + offset, length);
    if (*out == NULL) {
        return false;
    }
    if (notify_mult != NULL) {
        *notify_mult = pci_read32(cap + 16u);
    }
    return true;
}

static bool sound_find_caps(void)
{
    int32_t first = g_api->pci.find_capability(
        g_pci.bus, g_pci.device, g_pci.function, PCI_CAP_VENDOR);
    uint16_t cap = first >= 0 ? (uint16_t)first : 0u;
    for (uint32_t guard = 0u; cap >= 0x40u && guard < 96u; ++guard) {
        if (pci_read8(cap) == PCI_CAP_VENDOR) {
            uint8_t type = pci_read8(cap + 3u);
            if (type == VIRTIO_CAP_COMMON) {
                (void)sound_map_cap(cap, type, &g_common, NULL);
            } else if (type == VIRTIO_CAP_NOTIFY) {
                (void)sound_map_cap(cap, type, &g_notify,
                                    &g_notify_multiplier);
            } else if (type == VIRTIO_CAP_DEVICE) {
                (void)sound_map_cap(cap, type, &g_device, NULL);
            }
        }
        cap = pci_read8(cap + 1u);
    }
    return g_common != NULL && g_notify != NULL && g_device != NULL;
}

static bool sound_setup_queue(uint16_t index)
{
    sound_queue_t *queue = &g_queues[index];
    common_write16(COMMON_QUEUE_SELECT, index);
    uint16_t offered = common_read16(COMMON_QUEUE_SIZE);
    if (offered < 3u) {
        return false;
    }
    queue->size = offered > SOUND_QUEUE_MAX ? SOUND_QUEUE_MAX : offered;
    common_write16(COMMON_QUEUE_SIZE, queue->size);

    size_t desc_bytes = (size_t)queue->size * sizeof(virtq_desc_t);
    size_t avail_bytes = 6u + (size_t)queue->size * sizeof(uint16_t);
    size_t used_bytes = 6u + (size_t)queue->size * sizeof(virtq_used_elem_t);
    queue->desc = g_api->mem.dma_alloc_ex(
        desc_bytes, 16u, 0u, &queue->desc_phys);
    queue->avail = g_api->mem.dma_alloc_ex(
        avail_bytes, 2u, 0u, &queue->avail_phys);
    queue->used = g_api->mem.dma_alloc_ex(
        used_bytes, 4u, 0u, &queue->used_phys);
    if (queue->desc == NULL || queue->avail == NULL || queue->used == NULL) {
        return false;
    }
    common_write16(COMMON_QUEUE_MSIX_VECTOR, 0xFFFFu);
    common_write64(COMMON_QUEUE_DESC, queue->desc_phys);
    common_write64(COMMON_QUEUE_DRIVER, queue->avail_phys);
    common_write64(COMMON_QUEUE_DEVICE, queue->used_phys);
    common_write16(COMMON_QUEUE_ENABLE, 1u);
    uint16_t notify_off = common_read16(COMMON_QUEUE_NOTIFY_OFF);
    queue->notify = (volatile uint16_t *)(g_notify +
        (uint32_t)notify_off * g_notify_multiplier);
    return true;
}

static void sound_submit(sound_queue_t *queue, uint16_t head)
{
    queue->avail->ring[queue->avail_idx % queue->size] = head;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    ++queue->avail_idx;
    queue->avail->idx = queue->avail_idx;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    *queue->notify = (uint16_t)(queue - g_queues);
}

static bool sound_wait(sound_queue_t *queue, uint32_t timeout_ms)
{
    uint64_t start = g_api->timer.monotonic_ns();
    while (queue->used->idx == queue->used_idx) {
        if (g_api->timer.monotonic_ns() - start >
            (uint64_t)timeout_ms * 1000000ULL) {
            return false;
        }
        g_api->hal.cpu_pause();
    }
    ++queue->used_idx;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return true;
}

static bool sound_control(const void *request, uint32_t request_bytes,
                          uint32_t response_bytes)
{
    sound_queue_t *queue = &g_queues[SOUND_CONTROL_QUEUE];
    if (request_bytes > 64u || response_bytes > 64u) {
        return false;
    }
    g_api->mem.memset(g_control_request, 0, 64u);
    g_api->mem.memset(g_control_response, 0, 64u);
    g_api->mem.memcpy(g_control_request, request, request_bytes);
    queue->desc[0].addr = g_control_request_phys;
    queue->desc[0].len = request_bytes;
    queue->desc[0].flags = VIRTQ_DESC_NEXT;
    queue->desc[0].next = 1u;
    queue->desc[1].addr = g_control_response_phys;
    queue->desc[1].len = response_bytes;
    queue->desc[1].flags = VIRTQ_DESC_WRITE;
    queue->desc[1].next = 0u;
    sound_submit(queue, 0u);
    return sound_wait(queue, SOUND_TIMEOUT_MS) &&
           *(uint32_t *)g_control_response == VIRTIO_SND_S_OK;
}

static bool sound_pcm_command(uint32_t code)
{
    sound_pcm_hdr_t request = {
        .hdr = { .code = code },
        .stream_id = g_stream_id,
    };
    return sound_control(&request, sizeof(request), sizeof(sound_hdr_t));
}

static bool sound_find_stream(void)
{
    uint32_t stream_count = *(volatile uint32_t *)(g_device + 4u);
    for (uint32_t stream = 0u; stream < stream_count; ++stream) {
        sound_query_info_t query = {
            .hdr = { .code = VIRTIO_SND_R_PCM_INFO },
            .start_id = stream,
            .count = 1u,
            .size = sizeof(sound_pcm_info_t),
        };
        if (!sound_control(&query, sizeof(query),
                           sizeof(sound_hdr_t) + sizeof(sound_pcm_info_t))) {
            continue;
        }
        sound_pcm_info_t *info =
            (sound_pcm_info_t *)(g_control_response + sizeof(sound_hdr_t));
        if (info->direction == VIRTIO_SND_D_OUTPUT &&
            info->channels_min <= 2u && info->channels_max >= 2u &&
            (info->formats & (1ULL << VIRTIO_SND_PCM_FMT_S16)) != 0u &&
            (info->rates & (1ULL << VIRTIO_SND_PCM_RATE_48000)) != 0u) {
            g_stream_id = stream;
            return true;
        }
    }
    return false;
}

static bool sound_init(void)
{
    if (g_ready) {
        return true;
    }
    if (!sound_find_device() || !sound_find_caps() ||
        !g_api->pci.enable_bus_master(g_pci.bus, g_pci.device,
                                      g_pci.function)) {
        return false;
    }
    common_write8(COMMON_DEVICE_STATUS, 0u);
    common_write8(COMMON_DEVICE_STATUS,
                  VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER);
    common_write32(COMMON_DEVICE_FEATURE_SELECT, 1u);
    if ((common_read32(COMMON_DEVICE_FEATURE) & 1u) == 0u) {
        common_write8(COMMON_DEVICE_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }
    common_write32(COMMON_DRIVER_FEATURE_SELECT, 0u);
    common_write32(COMMON_DRIVER_FEATURE, 0u);
    common_write32(COMMON_DRIVER_FEATURE_SELECT, 1u);
    common_write32(COMMON_DRIVER_FEATURE, 1u);
    common_write8(COMMON_DEVICE_STATUS,
        VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER |
        VIRTIO_STATUS_FEATURES_OK);
    if ((*(volatile uint8_t *)(g_common + COMMON_DEVICE_STATUS) &
         VIRTIO_STATUS_FEATURES_OK) == 0u) {
        return false;
    }
    for (uint16_t i = 0u; i < SOUND_QUEUE_COUNT; ++i) {
        if (!sound_setup_queue(i)) {
            common_write8(COMMON_DEVICE_STATUS, VIRTIO_STATUS_FAILED);
            return false;
        }
    }
    g_control_request = g_api->mem.dma_alloc_ex(
        64u, 16u, 0u, &g_control_request_phys);
    g_control_response = g_api->mem.dma_alloc_ex(
        64u, 16u, 0u, &g_control_response_phys);
    g_pcm = g_api->mem.dma_alloc_ex(
        SOUND_PERIOD_BYTES, 16u, 0u, &g_pcm_phys);
    g_xfer = g_api->mem.dma_alloc_ex(
        sizeof(*g_xfer), 4u, 0u, &g_xfer_phys);
    g_pcm_status = g_api->mem.dma_alloc_ex(
        sizeof(*g_pcm_status), 4u, 0u, &g_pcm_status_phys);
    g_event = g_api->mem.dma_alloc_ex(8u, 4u, 0u, &g_event_phys);
    if (g_control_request == NULL || g_control_response == NULL ||
        g_pcm == NULL || g_xfer == NULL || g_pcm_status == NULL ||
        g_event == NULL) {
        return false;
    }
    common_write8(COMMON_DEVICE_STATUS,
        VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER |
        VIRTIO_STATUS_FEATURES_OK | VIRTIO_STATUS_DRIVER_OK);

    sound_queue_t *eventq = &g_queues[SOUND_EVENT_QUEUE];
    eventq->desc[0].addr = g_event_phys;
    eventq->desc[0].len = 8u;
    eventq->desc[0].flags = VIRTQ_DESC_WRITE;
    eventq->desc[0].next = 0u;
    sound_submit(eventq, 0u);
    if (!sound_find_stream()) {
        return false;
    }
    g_ready = true;
    return true;
}

static bool sound_get_info(driver_audio_info_t *info)
{
    if (!g_ready || info == NULL) {
        return false;
    }
    info->sample_rate = 48000u;
    info->channels = 2u;
    info->format = DRIVER_AUDIO_FORMAT_S16_LE;
    info->period_bytes = SOUND_PERIOD_BYTES;
    info->period_count = SOUND_PERIOD_COUNT;
    return true;
}

static bool sound_open(void)
{
    if (!g_ready || g_open) {
        return false;
    }
    sound_set_params_t params = {
        .hdr = {
            .hdr = { .code = VIRTIO_SND_R_PCM_SET_PARAMS },
            .stream_id = g_stream_id,
        },
        .buffer_bytes = SOUND_BUFFER_BYTES,
        .period_bytes = SOUND_PERIOD_BYTES,
        .features = 0u,
        .channels = 2u,
        .format = VIRTIO_SND_PCM_FMT_S16,
        .rate = VIRTIO_SND_PCM_RATE_48000,
        .padding = 0u,
    };
    if (!sound_control(&params, sizeof(params), sizeof(sound_hdr_t)) ||
        !sound_pcm_command(VIRTIO_SND_R_PCM_PREPARE) ||
        !sound_pcm_command(VIRTIO_SND_R_PCM_START)) {
        return false;
    }
    g_open = true;
    return true;
}

static bool sound_write_period(const uint8_t *pcm, uint32_t bytes)
{
    sound_queue_t *queue = &g_queues[SOUND_TX_QUEUE];
    g_api->mem.memset(g_pcm, 0, SOUND_PERIOD_BYTES);
    g_api->mem.memcpy(g_pcm, pcm, bytes);
    g_xfer->stream_id = g_stream_id;
    g_pcm_status->status = 0u;
    g_pcm_status->latency_bytes = 0u;

    queue->desc[0].addr = g_xfer_phys;
    queue->desc[0].len = sizeof(*g_xfer);
    queue->desc[0].flags = VIRTQ_DESC_NEXT;
    queue->desc[0].next = 1u;
    queue->desc[1].addr = g_pcm_phys;
    queue->desc[1].len = bytes;
    queue->desc[1].flags = VIRTQ_DESC_NEXT;
    queue->desc[1].next = 2u;
    queue->desc[2].addr = g_pcm_status_phys;
    queue->desc[2].len = sizeof(*g_pcm_status);
    queue->desc[2].flags = VIRTQ_DESC_WRITE;
    queue->desc[2].next = 0u;
    sound_submit(queue, 0u);
    return sound_wait(queue, SOUND_TIMEOUT_MS) &&
           g_pcm_status->status == VIRTIO_SND_S_OK;
}

static int64_t sound_write(const void *pcm, uint64_t bytes)
{
    if (!g_open || pcm == NULL || bytes == 0u || (bytes & 3u) != 0u) {
        return -1;
    }
    const uint8_t *source = pcm;
    uint64_t done = 0u;
    while (done < bytes) {
        uint32_t chunk = (uint32_t)(bytes - done);
        if (chunk > SOUND_PERIOD_BYTES) {
            chunk = SOUND_PERIOD_BYTES;
        }
        if (!sound_write_period(source + done, chunk)) {
            return done == 0u ? -1 : (int64_t)done;
        }
        done += chunk;
    }
    return (int64_t)done;
}

static bool sound_drain(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return g_open;
}

static void sound_close(void)
{
    if (g_open) {
        (void)sound_pcm_command(VIRTIO_SND_R_PCM_STOP);
        (void)sound_pcm_command(VIRTIO_SND_R_PCM_RELEASE);
    }
    g_open = false;
}

static bool sound_is_ready(void) { return g_ready; }

static const driver_audio_t g_audio = {
    .name = "virtio-sound",
    .priority = 10u,
    .init = sound_init,
    .is_ready = sound_is_ready,
    .get_info = sound_get_info,
    .open = sound_open,
    .write = sound_write,
    .drain = sound_drain,
    .close = sound_close,
};

static void sound_shutdown(void)
{
    sound_close();
    if (g_common != NULL) {
        common_write8(COMMON_DEVICE_STATUS, 0u);
    }
    g_ready = false;
}

static const driver_module_descriptor_t g_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_AUDIO,
    .load_priority = 43u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_audio,
    .shutdown = sound_shutdown,
};

__attribute__((visibility("default")))
const driver_module_descriptor_t *driver_module_init(
    const driver_binary_t *api)
{
    if (api == NULL || api->version_major != DRIVER_API_VERSION_MAJOR ||
        api->pci.get_device_count == NULL || api->mem.dma_alloc_ex == NULL ||
        api->hw.map_mmio_range == NULL) {
        return NULL;
    }
    g_api = api;
    return &g_module;
}
