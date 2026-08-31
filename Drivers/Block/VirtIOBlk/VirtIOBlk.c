#include "Drivers/Module/DriverBinary.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VIRTIO_VENDOR_ID        0x1AF4u
#define VIRTIO_BLK_MODERN_ID    0x1042u
#define PCI_CAP_VENDOR          0x09u

#define VIRTIO_CAP_COMMON       1u
#define VIRTIO_CAP_NOTIFY       2u
#define VIRTIO_CAP_DEVICE       4u

#define VIRTIO_STATUS_ACK       0x01u
#define VIRTIO_STATUS_DRIVER    0x02u
#define VIRTIO_STATUS_DRIVER_OK 0x04u
#define VIRTIO_STATUS_FEATURES  0x08u
#define VIRTIO_STATUS_FAILED    0x80u

#define VIRTIO_BLK_F_RO         (1u << 5u)
#define VIRTIO_BLK_F_BLK_SIZE   (1u << 6u)
#define VIRTIO_BLK_F_FLUSH      (1u << 9u)
#define VIRTIO_BLK_F_MQ         (1u << 12u)

#define VIRTQ_DESC_NEXT         1u
#define VIRTQ_DESC_WRITE        2u
#define VIRTIO_BLK_T_IN         0u
#define VIRTIO_BLK_T_OUT        1u
#define VIRTIO_BLK_T_FLUSH      4u

#define BLK_QUEUE_MAX           128u
#define BLK_IO_BYTES            (128u * 1024u)
#define BLK_TIMEOUT_MS          5000u

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
    uint16_t ring[BLK_QUEUE_MAX];
} virtq_avail_t;

typedef struct __attribute__((packed)) {
    uint32_t id;
    uint32_t len;
} virtq_used_elem_t;

typedef struct __attribute__((packed)) {
    uint16_t flags;
    uint16_t idx;
    virtq_used_elem_t ring[BLK_QUEUE_MAX];
} virtq_used_t;

typedef struct __attribute__((packed)) {
    uint32_t type;
    uint32_t reserved;
    uint64_t sector;
} virtio_blk_req_t;

static const driver_binary_t *g_api;
static driver_pci_device_t g_pci;
static volatile uint8_t *g_common;
static volatile uint8_t *g_notify;
static volatile uint8_t *g_device;
static uint32_t g_notify_multiplier;
static virtq_desc_t *g_desc;
static virtq_avail_t *g_avail;
static virtq_used_t *g_used;
static uint64_t g_desc_phys;
static uint64_t g_avail_phys;
static uint64_t g_used_phys;
static volatile uint16_t *g_notify_addr;
static uint16_t g_queue_size;
static uint16_t g_avail_idx;
static uint16_t g_used_idx;
static virtio_blk_req_t *g_request;
static uint64_t g_request_phys;
static uint8_t *g_status;
static uint64_t g_status_phys;
static uint8_t *g_buffer;
static uint64_t g_buffer_phys;
static uint64_t g_capacity_512;
static uint32_t g_block_size;
static uint32_t g_features;
static bool g_ready;

static uint32_t pci_read32(uint16_t offset)
{
    return g_api->pci.read_config(g_pci.bus, g_pci.device, g_pci.function,
                                  (uint16_t)(offset & 0xFCu));
}

static uint8_t pci_read8(uint16_t offset)
{
    uint32_t value = pci_read32(offset);
    return (uint8_t)(value >> ((offset & 3u) * 8u));
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

static uint64_t device_read64(uint32_t offset)
{
    uint64_t low = *(volatile uint32_t *)(g_device + offset);
    uint64_t high = *(volatile uint32_t *)(g_device + offset + 4u);
    return low | (high << 32u);
}

static bool virtio_find_device(void)
{
    uint32_t count = g_api->pci.get_device_count();
    for (uint32_t i = 0u; i < count; ++i) {
        driver_pci_device_t device;
        if (!g_api->pci.get_device(i, &device)) {
            continue;
        }
        if (device.vendor_id == VIRTIO_VENDOR_ID &&
            (device.device_id == VIRTIO_BLK_MODERN_ID ||
             (device.class_code == 0x01u && device.subclass == 0x00u))) {
            g_pci = device;
            return true;
        }
    }
    return false;
}

static bool virtio_map_cap(uint16_t cap, uint8_t expected_type,
                           volatile uint8_t **out, uint32_t *notify_mult)
{
    uint8_t cfg_type = pci_read8(cap + 3u);
    if (cfg_type != expected_type) {
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

static bool virtio_find_caps(void)
{
    int32_t cap_pos = g_api->pci.find_capability(
        g_pci.bus, g_pci.device, g_pci.function, PCI_CAP_VENDOR);
    uint16_t cap = cap_pos >= 0 ? (uint16_t)cap_pos : 0u;
    for (uint32_t guard = 0u; cap >= 0x40u && guard < 96u; ++guard) {
        if (pci_read8(cap) == PCI_CAP_VENDOR) {
            uint8_t type = pci_read8(cap + 3u);
            if (type == VIRTIO_CAP_COMMON) {
                (void)virtio_map_cap(cap, type, &g_common, NULL);
            } else if (type == VIRTIO_CAP_NOTIFY) {
                (void)virtio_map_cap(cap, type, &g_notify,
                                     &g_notify_multiplier);
            } else if (type == VIRTIO_CAP_DEVICE) {
                (void)virtio_map_cap(cap, type, &g_device, NULL);
            }
        }
        cap = pci_read8(cap + 1u);
    }
    return g_common != NULL && g_notify != NULL && g_device != NULL;
}

static bool virtio_setup_queue(void)
{
    common_write16(COMMON_QUEUE_SELECT, 0u);
    uint16_t offered = common_read16(COMMON_QUEUE_SIZE);
    if (offered < 3u) {
        return false;
    }
    g_queue_size = offered > BLK_QUEUE_MAX ? BLK_QUEUE_MAX : offered;
    common_write16(COMMON_QUEUE_SIZE, g_queue_size);

    size_t desc_bytes = (size_t)g_queue_size * sizeof(virtq_desc_t);
    size_t avail_bytes = 6u + (size_t)g_queue_size * sizeof(uint16_t);
    size_t used_bytes = 6u + (size_t)g_queue_size * sizeof(virtq_used_elem_t);
    g_desc = g_api->mem.dma_alloc_ex(desc_bytes, 16u, 0u, &g_desc_phys);
    g_avail = g_api->mem.dma_alloc_ex(avail_bytes, 2u, 0u, &g_avail_phys);
    g_used = g_api->mem.dma_alloc_ex(used_bytes, 4u, 0u, &g_used_phys);
    if (g_desc == NULL || g_avail == NULL || g_used == NULL) {
        return false;
    }
    common_write16(COMMON_QUEUE_MSIX_VECTOR, 0xFFFFu);
    common_write64(COMMON_QUEUE_DESC, g_desc_phys);
    common_write64(COMMON_QUEUE_DRIVER, g_avail_phys);
    common_write64(COMMON_QUEUE_DEVICE, g_used_phys);
    common_write16(COMMON_QUEUE_ENABLE, 1u);
    uint16_t notify_off = common_read16(COMMON_QUEUE_NOTIFY_OFF);
    g_notify_addr = (volatile uint16_t *)(g_notify +
        (uint32_t)notify_off * g_notify_multiplier);
    return true;
}

static bool virtio_request(uint32_t type, uint64_t sector,
                           uint32_t byte_count, bool device_writes)
{
    g_request->type = type;
    g_request->reserved = 0u;
    g_request->sector = sector;
    *g_status = 0xFFu;

    g_desc[0].addr = g_request_phys;
    g_desc[0].len = sizeof(*g_request);
    g_desc[0].flags = VIRTQ_DESC_NEXT;
    g_desc[0].next = type == VIRTIO_BLK_T_FLUSH ? 2u : 1u;
    if (type != VIRTIO_BLK_T_FLUSH) {
        g_desc[1].addr = g_buffer_phys;
        g_desc[1].len = byte_count;
        g_desc[1].flags = VIRTQ_DESC_NEXT |
                          (device_writes ? VIRTQ_DESC_WRITE : 0u);
        g_desc[1].next = 2u;
    }
    g_desc[2].addr = g_status_phys;
    g_desc[2].len = 1u;
    g_desc[2].flags = VIRTQ_DESC_WRITE;
    g_desc[2].next = 0u;

    g_avail->ring[g_avail_idx % g_queue_size] = 0u;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    ++g_avail_idx;
    g_avail->idx = g_avail_idx;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    *g_notify_addr = 0u;

    uint64_t start = g_api->timer.monotonic_ns();
    while (g_used->idx == g_used_idx) {
        if (g_api->timer.monotonic_ns() - start >
            (uint64_t)BLK_TIMEOUT_MS * 1000000ULL) {
            return false;
        }
        g_api->hal.cpu_pause();
    }
    ++g_used_idx;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return *g_status == 0u;
}

static bool virtio_rw(uint64_t lba, void *buffer, uint32_t block_count,
                      bool write)
{
    if (!g_ready || buffer == NULL || block_count == 0u) {
        return false;
    }
    uint32_t max_blocks = BLK_IO_BYTES / g_block_size;
    uint8_t *bytes = buffer;
    for (uint32_t done = 0u; done < block_count;) {
        uint32_t count = block_count - done;
        if (count > max_blocks) {
            count = max_blocks;
        }
        uint32_t byte_count = count * g_block_size;
        if (write) {
            g_api->memcpy(g_buffer,
                          bytes + (size_t)done * g_block_size, byte_count);
        }
        uint64_t sector =
            (lba + done) * ((uint64_t)g_block_size / 512u);
        if (!virtio_request(write ? VIRTIO_BLK_T_OUT : VIRTIO_BLK_T_IN,
                            sector, byte_count, !write)) {
            return false;
        }
        if (!write) {
            g_api->memcpy(bytes + (size_t)done * g_block_size,
                          g_buffer, byte_count);
        }
        done += count;
    }
    return true;
}

static bool blk_read(uint32_t index, uint64_t lba, void *buffer,
                     uint32_t count)
{
    return index == 0u && virtio_rw(lba, buffer, count, false);
}

static bool blk_write(uint32_t index, uint64_t lba, const void *buffer,
                      uint32_t count)
{
    return index == 0u && (g_features & VIRTIO_BLK_F_RO) == 0u &&
           virtio_rw(lba, (void *)buffer, count, true);
}

static bool blk_flush(uint32_t index)
{
    return index == 0u &&
           (((g_features & VIRTIO_BLK_F_FLUSH) == 0u) ||
            virtio_request(VIRTIO_BLK_T_FLUSH, 0u, 0u, false));
}

static bool blk_get_info(uint32_t index, driver_block_info_t *out)
{
    if (index != 0u || out == NULL || !g_ready) {
        return false;
    }
    g_api->memset(out, 0, sizeof(*out));
    out->block_count = (g_capacity_512 * 512u) / g_block_size;
    out->logical_block_size = g_block_size;
    out->physical_block_size = g_block_size;
    out->flags = (g_features & VIRTIO_BLK_F_RO) == 0u ?
                 DRIVER_BLOCK_FLAG_WRITABLE : 0u;
    out->transport = DRIVER_BLOCK_TRANSPORT_VIRTIO;
    out->identity_flags = DRIVER_BLOCK_IDENTITY_PCI_VALID;
    out->pci_segment = 0u;
    out->pci_bus = g_pci.bus;
    out->pci_device = g_pci.device;
    out->pci_function = g_pci.function;
    out->controller_port = UINT16_MAX;
    const char model[] = "VirtIO block device";
    g_api->memcpy(out->model, model, sizeof(model));
    return true;
}

static bool blk_init(void)
{
    if (g_ready) {
        return true;
    }
    if (!virtio_find_device() || !virtio_find_caps() ||
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
    common_write32(COMMON_DEVICE_FEATURE_SELECT, 0u);
    uint32_t offered = common_read32(COMMON_DEVICE_FEATURE);
    g_features = offered & (VIRTIO_BLK_F_RO | VIRTIO_BLK_F_BLK_SIZE |
                            VIRTIO_BLK_F_FLUSH | VIRTIO_BLK_F_MQ);
    common_write32(COMMON_DRIVER_FEATURE_SELECT, 0u);
    common_write32(COMMON_DRIVER_FEATURE, g_features);
    common_write32(COMMON_DRIVER_FEATURE_SELECT, 1u);
    common_write32(COMMON_DRIVER_FEATURE, 1u);
    common_write8(COMMON_DEVICE_STATUS,
        VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEATURES);
    if ((*(volatile uint8_t *)(g_common + COMMON_DEVICE_STATUS) &
         VIRTIO_STATUS_FEATURES) == 0u) {
        return false;
    }
    g_capacity_512 = device_read64(0u);
    g_block_size = (g_features & VIRTIO_BLK_F_BLK_SIZE) != 0u ?
        *(volatile uint32_t *)(g_device + 20u) : 512u;
    if (g_capacity_512 == 0u || g_block_size < 512u ||
        (g_block_size % 512u) != 0u || !virtio_setup_queue()) {
        return false;
    }
    g_request = g_api->mem.dma_alloc_ex(sizeof(*g_request), 16u, 0u,
                                         &g_request_phys);
    g_status = g_api->mem.dma_alloc_ex(1u, 1u, 0u, &g_status_phys);
    g_buffer = g_api->mem.dma_alloc_ex(BLK_IO_BYTES, 4096u, 0u,
                                        &g_buffer_phys);
    if (g_request == NULL || g_status == NULL || g_buffer == NULL) {
        return false;
    }
    common_write8(COMMON_DEVICE_STATUS,
        VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER |
        VIRTIO_STATUS_FEATURES | VIRTIO_STATUS_DRIVER_OK);
    g_ready = true;
    return true;
}

static bool blk_is_ready(void) { return g_ready; }
static uint32_t blk_count(void) { return g_ready ? 1u : 0u; }

static const driver_storage_t g_storage = {
    .name = "virtio-blk",
    .priority = 10u,
    .init = blk_init,
    .is_ready = blk_is_ready,
    .get_device_count = blk_count,
    .get_info = blk_get_info,
    .read_blocks = blk_read,
    .write_blocks = blk_write,
    .flush = blk_flush,
};

static void blk_shutdown(void)
{
    if (g_common != NULL) {
        common_write8(COMMON_DEVICE_STATUS, 0u);
    }
    g_ready = false;
}

static const driver_module_descriptor_t g_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_BLOCK,
    .load_priority = 40u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_storage,
    .shutdown = blk_shutdown,
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
