#include "VirtIONet.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"
#else
#include "Drivers/Module/PCI_Main.h"
#include "MemoryManagement/DMA_Memory.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "Core/sync/Spinlock.h"
#include "Debug/serial/Serial.h"
#endif

#ifdef IMPLUS_DRIVER_MODULE
static const driver_binary_t *g_api = NULL;

#define malloc             g_api->malloc
#define free               g_api->free
#define dma_alloc          g_api->dma_alloc
#define dma_free           g_api->dma_free
#define memset             g_api->memset
#define memcpy             g_api->memcpy
#define map_mmio_range     g_api->hw.map_mmio_range
#define pci_read_config    g_api->pci_read_config
#define pci_write_config   g_api->pci_write_config
#define serial_write_string g_api->serial_write_string

#define hal_cpu_save_interrupts    g_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts g_api->hal.cpu_restore_interrupts
#define hal_cpu_pause             g_api->hal.cpu_pause

typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_init(spinlock_t *l)   { l->locked = 0; }
static inline void spinlock_lock(spinlock_t *l)   {
    while (__sync_lock_test_and_set(&l->locked, 1)) {
        while (l->locked) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->locked); }

static inline uint64_t irq_save_disable(void) { return hal_cpu_save_interrupts(); }
static inline void irq_restore(uint64_t flags) { hal_cpu_restore_interrupts(flags); }

static inline bool dma_init(void) { return true; }
#endif

#define VIRTIO_VENDOR_ID              0x1AF4u
#define VIRTIO_NET_DEVICE_ID_MODERN   0x1041u
#define VIRTIO_NET_DEVICE_ID_LEGACY   0x1000u

#define PCI_CAP_ID_VENDOR             0x09u

#define VIRTIO_PCI_CAP_COMMON_CFG     1u
#define VIRTIO_PCI_CAP_NOTIFY_CFG     2u
#define VIRTIO_PCI_CAP_DEVICE_CFG     4u

#define VIRTIO_STATUS_ACKNOWLEDGE     0x01u
#define VIRTIO_STATUS_DRIVER          0x02u
#define VIRTIO_STATUS_DRIVER_OK       0x04u
#define VIRTIO_STATUS_FEATURES_OK     0x08u
#define VIRTIO_STATUS_FAILED          0x80u

#define VIRTIO_F_VERSION_1            (1ULL << 32)

#define VIRTIO_NET_F_MTU              (1ULL << 3)
#define VIRTIO_NET_F_MAC              (1ULL << 5)

#define VIRTQ_DESC_F_NEXT             1u
#define VIRTQ_DESC_F_WRITE            2u

#define VIRTIO_NET_RX_QUEUE_INDEX     0u
#define VIRTIO_NET_TX_QUEUE_INDEX     1u

#define VIRTIO_NET_HDR_BYTES          12u
#define VIRTIO_NET_MAX_FRAME_BYTES    1514u
#define VIRTIO_NET_BUF_BYTES          2048u
#define VIRTIO_NET_DEFAULT_MTU        1500u

#define COMMON_OFF_DEVICE_FEATURE_SELECT 0u
#define COMMON_OFF_DEVICE_FEATURE        4u
#define COMMON_OFF_DRIVER_FEATURE_SELECT 8u
#define COMMON_OFF_DRIVER_FEATURE       12u
#define COMMON_OFF_NUM_QUEUES           18u
#define COMMON_OFF_DEVICE_STATUS        20u
#define COMMON_OFF_QUEUE_SELECT         22u
#define COMMON_OFF_QUEUE_SIZE           24u
#define COMMON_OFF_QUEUE_MSIX_VECTOR    26u
#define COMMON_OFF_QUEUE_ENABLE         28u
#define COMMON_OFF_QUEUE_NOTIFY_OFF     30u
#define COMMON_OFF_QUEUE_DESC           32u
#define COMMON_OFF_QUEUE_DRIVER         40u
#define COMMON_OFF_QUEUE_DEVICE         48u

typedef struct __attribute__((packed)) {
    uint8_t cap_vndr;
    uint8_t cap_next;
    uint8_t cap_len;
    uint8_t cfg_type;
    uint8_t bar;
    uint8_t id;
    uint8_t padding[2];
    uint32_t offset;
    uint32_t length;
} virtio_pci_cap_t;

typedef struct __attribute__((packed)) {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
} virtq_desc_t;

typedef struct __attribute__((packed)) {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[];
} virtq_avail_t;

typedef struct __attribute__((packed)) {
    uint32_t id;
    uint32_t len;
} virtq_used_elem_t;

typedef struct __attribute__((packed)) {
    uint16_t flags;
    uint16_t idx;
    virtq_used_elem_t ring[];
} virtq_used_t;

typedef struct {
    uint8_t bus;
    uint8_t device;
    uint8_t func;
    uint64_t bar_addr[6];
    uint8_t bar_is_mem[6];
} virtio_net_pci_t;

typedef struct {
    volatile uint8_t *common_cfg;
    volatile uint8_t *notify_base;
    volatile uint8_t *device_cfg;
    uint32_t notify_off_multiplier;
} virtio_transport_t;

typedef struct {
    uint8_t *virt;
    uint64_t phys;
} dma_buffer_t;

typedef struct {
    uint16_t queue_index;
    uint16_t queue_size;
    volatile virtq_desc_t *desc;
    volatile virtq_avail_t *avail;
    volatile virtq_used_t *used;
    volatile uint16_t *notify_addr;
    uint16_t avail_idx;
    uint16_t used_idx_seen;
    uint8_t *ring_virt;
    uint64_t ring_phys;
    uint32_t ring_size_bytes;
} virtqueue_t;

static spinlock_t g_virtio_net_lock = {0};
static int g_virtio_net_ready = 0;
static uint16_t g_virtio_net_mtu = VIRTIO_NET_DEFAULT_MTU;
static uint8_t g_virtio_net_mac[6] = {0x52u, 0x54u, 0x00u, 0x12u, 0x34u, 0x56u};

static virtio_transport_t g_transport;
static virtqueue_t g_rx_queue;
static virtqueue_t g_tx_queue;

static dma_buffer_t *g_rx_buffers = NULL;
static dma_buffer_t *g_tx_buffers = NULL;
static uint8_t *g_tx_in_use = NULL;

static virtio_net_rx_callback_t g_rx_callback = NULL;

static inline uint32_t align_up_u32(uint32_t value, uint32_t align)
{
    return (value + align - 1u) & ~(align - 1u);
}

static inline void memory_barrier(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

static uint8_t pci_cfg_read8(uint8_t bus, uint8_t device, uint8_t func, uint16_t offset)
{
    uint32_t value = pci_read_config(bus, device, func, (uint16_t)(offset & 0xFFFCu));
    return (uint8_t)((value >> ((offset & 0x3u) * 8u)) & 0xFFu);
}

static uint16_t pci_cfg_read16(uint8_t bus, uint8_t device, uint8_t func, uint16_t offset)
{
    uint32_t value = pci_read_config(bus, device, func, (uint16_t)(offset & 0xFFFCu));
    return (uint16_t)((value >> ((offset & 0x2u) * 8u)) & 0xFFFFu);
}

static uint32_t pci_cfg_read32(uint8_t bus, uint8_t device, uint8_t func, uint16_t offset)
{
    return pci_read_config(bus, device, func, (uint16_t)(offset & 0xFFFCu));
}

static void pci_cfg_write16(uint8_t bus,
                            uint8_t device,
                            uint8_t func,
                            uint16_t offset,
                            uint16_t value)
{
    uint16_t aligned = (uint16_t)(offset & 0xFFFCu);
    uint32_t old_value = pci_read_config(bus, device, func, aligned);
    uint32_t shift = (uint32_t)((offset & 0x2u) * 8u);
    uint32_t mask = 0xFFFFu << shift;
    uint32_t merged = (old_value & ~mask) | ((uint32_t)value << shift);
    pci_write_config(bus, device, func, aligned, merged);
}

static inline uint8_t common_read8(volatile uint8_t *common, uint32_t offset)
{
    return *(volatile uint8_t *)(common + offset);
}

static inline uint16_t common_read16(volatile uint8_t *common, uint32_t offset)
{
    return *(volatile uint16_t *)(common + offset);
}

static inline uint32_t common_read32(volatile uint8_t *common, uint32_t offset)
{
    return *(volatile uint32_t *)(common + offset);
}

static inline void common_write8(volatile uint8_t *common, uint32_t offset, uint8_t value)
{
    *(volatile uint8_t *)(common + offset) = value;
}

static inline void common_write16(volatile uint8_t *common, uint32_t offset, uint16_t value)
{
    *(volatile uint16_t *)(common + offset) = value;
}

static inline void common_write32(volatile uint8_t *common, uint32_t offset, uint32_t value)
{
    *(volatile uint32_t *)(common + offset) = value;
}

static inline void common_write64(volatile uint8_t *common, uint32_t offset, uint64_t value)
{
    *(volatile uint64_t *)(common + offset) = value;
}

static void virtio_read_bar_addrs(uint8_t bus,
                                  uint8_t device,
                                  uint8_t func,
                                  uint64_t out_bar[6],
                                  uint8_t out_is_mem[6])
{
    for (uint8_t j = 0; j < 6u; j++) {
        out_bar[j] = 0;
        out_is_mem[j] = 0;
    }
    
    uint8_t i = 0;
    while (i < 6u) {
        uint32_t bar = pci_cfg_read32(bus, device, func, (uint16_t)(0x10u + i * 4u));

        if (bar == 0u || bar == 0xFFFFFFFFu) {
            i++;
            continue;
        }

        if ((bar & 0x1u) != 0u) {
            i++;
            continue;
        }

        out_is_mem[i] = 1;
        if (((bar >> 1) & 0x3u) == 0x2u && i < 5u) {
            uint32_t bar_hi = pci_cfg_read32(bus, device, func, (uint16_t)(0x10u + (i + 1u) * 4u));
            out_bar[i] = (((uint64_t)bar_hi) << 32) | (uint64_t)(bar & ~0xFu);
            i = (uint8_t)(i + 2u);
            continue;
        }

        out_bar[i] = (uint64_t)(bar & ~0xFu);
        i++;
    }
}

static int find_virtio_net(virtio_net_pci_t *out_dev)
{
    if (out_dev == NULL) {
        return 0;
    }

    for (uint16_t bus = 0; bus < 256u; ++bus) {
        for (uint8_t device = 0; device < 32u; ++device) {
            for (uint8_t func = 0; func < 8u; ++func) {
                uint32_t vd = pci_read_config((uint8_t)bus, device, func, 0x00u);
                uint16_t vendor_id = (uint16_t)(vd & 0xFFFFu);
                uint16_t device_id = (uint16_t)((vd >> 16) & 0xFFFFu);

                if (vendor_id == 0xFFFFu) {
                    if (func == 0u) {
                        break;
                    }
                    continue;
                }

                uint32_t class_reg = pci_read_config((uint8_t)bus, device, func, 0x08u);
                uint8_t class_code = (uint8_t)((class_reg >> 24) & 0xFFu);
                uint8_t subclass = (uint8_t)((class_reg >> 16) & 0xFFu);

                int device_match = 0;
                if (vendor_id == VIRTIO_VENDOR_ID) {
                    if (device_id == VIRTIO_NET_DEVICE_ID_MODERN ||
                        device_id == VIRTIO_NET_DEVICE_ID_LEGACY) {
                        device_match = 1;
                    } else if (class_code == 0x02u && subclass == 0x00u) {
                        device_match = 1;
                    }
                }

                if (device_match != 0) {
                    out_dev->bus = (uint8_t)bus;
                    out_dev->device = device;
                    out_dev->func = func;
                    
                    uint32_t cmd = pci_read_config((uint8_t)bus, device, func, 0x04u);
                    cmd |= (1u << 1) | (1u << 2);
                    pci_write_config((uint8_t)bus, device, func, 0x04u, cmd);

                    virtio_read_bar_addrs((uint8_t)bus,
                                          device,
                                          func,
                                          out_dev->bar_addr,
                                          out_dev->bar_is_mem);
                    return 1;
                }

                if (func == 0u) {
                    uint32_t header_type = pci_read_config((uint8_t)bus, device, func, 0x0Cu);
                    if (((header_type >> 16) & 0x80u) == 0u) {
                        break;
                    }
                }
            }
        }
    }

    return 0;
}

static int virtio_find_transport_caps(const virtio_net_pci_t *dev, virtio_transport_t *transport)
{
    if (dev == NULL || transport == NULL) {
        return 0;
    }

    transport->common_cfg = NULL;
    transport->notify_base = NULL;
    transport->device_cfg = NULL;
    transport->notify_off_multiplier = 0;

    uint16_t status = pci_cfg_read16(dev->bus, dev->device, dev->func, 0x06u);
    if ((status & (1u << 4)) == 0u) {
        return 0;
    }

    uint8_t cap = pci_cfg_read8(dev->bus, dev->device, dev->func, 0x34u);
    uint32_t guard = 0u;

    while (cap != 0u && cap >= 0x40u && guard++ < 96u) {
        uint8_t cap_id = pci_cfg_read8(dev->bus, dev->device, dev->func, cap);
        uint8_t cap_next = pci_cfg_read8(dev->bus, dev->device, dev->func, (uint16_t)(cap + 1u));

        if (cap_id == PCI_CAP_ID_VENDOR) {
            virtio_pci_cap_t vcap;
            vcap.cap_vndr = cap_id;
            vcap.cap_next = cap_next;
            vcap.cap_len = pci_cfg_read8(dev->bus, dev->device, dev->func, (uint16_t)(cap + 2u));
            vcap.cfg_type = pci_cfg_read8(dev->bus, dev->device, dev->func, (uint16_t)(cap + 3u));
            vcap.bar = pci_cfg_read8(dev->bus, dev->device, dev->func, (uint16_t)(cap + 4u));
            vcap.id = pci_cfg_read8(dev->bus, dev->device, dev->func, (uint16_t)(cap + 5u));
            vcap.offset = pci_cfg_read32(dev->bus, dev->device, dev->func, (uint16_t)(cap + 8u));
            vcap.length = pci_cfg_read32(dev->bus, dev->device, dev->func, (uint16_t)(cap + 12u));

            if (vcap.bar < 6u && dev->bar_is_mem[vcap.bar] != 0u && dev->bar_addr[vcap.bar] != 0u) {
                uint64_t phys = dev->bar_addr[vcap.bar] + (uint64_t)vcap.offset;
                volatile uint8_t *base = (volatile uint8_t *)map_mmio_range(phys, vcap.length);
                if (base == NULL) {
                    return 0;
                }

                if (vcap.cfg_type == VIRTIO_PCI_CAP_COMMON_CFG) {
                    transport->common_cfg = base;
                } else if (vcap.cfg_type == VIRTIO_PCI_CAP_NOTIFY_CFG) {
                    transport->notify_base = base;
                    transport->notify_off_multiplier =
                        pci_cfg_read32(dev->bus, dev->device, dev->func, (uint16_t)(cap + 16u));
                } else if (vcap.cfg_type == VIRTIO_PCI_CAP_DEVICE_CFG) {
                    transport->device_cfg = base;
                }
            }
        }

        cap = cap_next;
    }

    if (transport->common_cfg == NULL || transport->notify_base == NULL) {
        return 0;
    }

    return 1;
}

static void virtqueue_notify(const virtqueue_t *vq)
{
    if (vq == NULL || vq->notify_addr == NULL) {
        return;
    }
    *vq->notify_addr = vq->queue_index;
}

static int virtqueue_setup(virtio_transport_t *transport, virtqueue_t *vq, uint16_t queue_index)
{
    if (transport == NULL || vq == NULL || transport->common_cfg == NULL || transport->notify_base == NULL) {
        return 0;
    }

    common_write16(transport->common_cfg, COMMON_OFF_QUEUE_SELECT, queue_index);
    uint16_t queue_size = common_read16(transport->common_cfg, COMMON_OFF_QUEUE_SIZE);
    if (queue_size == 0u) {
        return 0;
    }

    uint32_t desc_bytes = (uint32_t)queue_size * (uint32_t)sizeof(virtq_desc_t);
    uint32_t avail_bytes = 6u + ((uint32_t)queue_size * 2u);
    uint32_t used_off = align_up_u32(desc_bytes + avail_bytes, 4u);
    uint32_t used_bytes = 6u + ((uint32_t)queue_size * 8u);
    uint32_t ring_bytes = used_off + used_bytes;

    uint64_t ring_phys = 0;
    uint8_t *ring_virt = (uint8_t *)dma_alloc((size_t)ring_bytes, &ring_phys);
    if (ring_virt == NULL || ring_phys == 0) {
        return 0;
    }

    memset(ring_virt, 0, ring_bytes);

    vq->queue_index = queue_index;
    vq->queue_size = queue_size;
    vq->ring_virt = ring_virt;
    vq->ring_phys = ring_phys;
    vq->ring_size_bytes = ring_bytes;
    vq->desc = (volatile virtq_desc_t *)ring_virt;
    vq->avail = (volatile virtq_avail_t *)(ring_virt + desc_bytes);
    vq->used = (volatile virtq_used_t *)(ring_virt + used_off);
    vq->avail_idx = 0;
    vq->used_idx_seen = 0;

    common_write16(transport->common_cfg, COMMON_OFF_QUEUE_MSIX_VECTOR, 0xFFFFu);
    common_write64(transport->common_cfg, COMMON_OFF_QUEUE_DESC, ring_phys);
    common_write64(transport->common_cfg, COMMON_OFF_QUEUE_DRIVER, ring_phys + (uint64_t)desc_bytes);
    common_write64(transport->common_cfg, COMMON_OFF_QUEUE_DEVICE, ring_phys + (uint64_t)used_off);
    common_write16(transport->common_cfg, COMMON_OFF_QUEUE_ENABLE, 1u);

    uint16_t notify_off = common_read16(transport->common_cfg, COMMON_OFF_QUEUE_NOTIFY_OFF);
    vq->notify_addr = (volatile uint16_t *)(transport->notify_base +
                                            ((uint32_t)notify_off * transport->notify_off_multiplier));

    common_write16(transport->common_cfg, COMMON_OFF_QUEUE_SELECT, queue_index);
    if (common_read16(transport->common_cfg, COMMON_OFF_QUEUE_ENABLE) == 0u) {
        return 0;
    }

    return 1;
}

static int virtio_net_device_init(const virtio_net_pci_t *dev,
                                  virtio_transport_t *transport,
                                  uint64_t *negotiated_features_out)
{
    if (dev == NULL || transport == NULL || transport->common_cfg == NULL || negotiated_features_out == NULL) {
        return 0;
    }

    uint16_t cmd = pci_cfg_read16(dev->bus, dev->device, dev->func, 0x04u);
    cmd |= (uint16_t)(1u << 1);
    cmd |= (uint16_t)(1u << 2);
    pci_cfg_write16(dev->bus, dev->device, dev->func, 0x04u, cmd);

    common_write8(transport->common_cfg, COMMON_OFF_DEVICE_STATUS, 0u);
    common_write8(transport->common_cfg,
                  COMMON_OFF_DEVICE_STATUS,
                  (uint8_t)(VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER));

    common_write32(transport->common_cfg, COMMON_OFF_DEVICE_FEATURE_SELECT, 0u);
    uint32_t host_features_low = common_read32(transport->common_cfg, COMMON_OFF_DEVICE_FEATURE);
    common_write32(transport->common_cfg, COMMON_OFF_DEVICE_FEATURE_SELECT, 1u);
    uint32_t host_features_high = common_read32(transport->common_cfg, COMMON_OFF_DEVICE_FEATURE);
    uint64_t host_features = ((uint64_t)host_features_high << 32) | (uint64_t)host_features_low;

    if ((host_features & VIRTIO_F_VERSION_1) == 0u) {
        common_write8(transport->common_cfg,
                      COMMON_OFF_DEVICE_STATUS,
                      (uint8_t)(VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FAILED));
        return 0;
    }

    uint64_t wanted_features = VIRTIO_F_VERSION_1;
    if ((host_features & VIRTIO_NET_F_MAC) != 0u) {
        wanted_features |= VIRTIO_NET_F_MAC;
    }
    if ((host_features & VIRTIO_NET_F_MTU) != 0u) {
        wanted_features |= VIRTIO_NET_F_MTU;
    }

    common_write32(transport->common_cfg, COMMON_OFF_DRIVER_FEATURE_SELECT, 0u);
    common_write32(transport->common_cfg,
                   COMMON_OFF_DRIVER_FEATURE,
                   (uint32_t)(wanted_features & 0xFFFFFFFFu));

    common_write32(transport->common_cfg, COMMON_OFF_DRIVER_FEATURE_SELECT, 1u);
    common_write32(transport->common_cfg,
                   COMMON_OFF_DRIVER_FEATURE,
                   (uint32_t)((wanted_features >> 32) & 0xFFFFFFFFu));

    common_write8(transport->common_cfg,
                  COMMON_OFF_DEVICE_STATUS,
                  (uint8_t)(VIRTIO_STATUS_ACKNOWLEDGE |
                            VIRTIO_STATUS_DRIVER |
                            VIRTIO_STATUS_FEATURES_OK));

    uint8_t status = common_read8(transport->common_cfg, COMMON_OFF_DEVICE_STATUS);
    if ((status & VIRTIO_STATUS_FEATURES_OK) == 0u) {
        common_write8(transport->common_cfg,
                      COMMON_OFF_DEVICE_STATUS,
                      (uint8_t)(status | VIRTIO_STATUS_FAILED));
        return 0;
    }

    *negotiated_features_out = wanted_features;
    return 1;
}

static int virtio_net_setup_rx_buffers(virtqueue_t *rxq)
{
    if (rxq == NULL || rxq->queue_size == 0u || rxq->desc == NULL || rxq->avail == NULL) {
        return 0;
    }

    uint16_t count = rxq->queue_size;
    g_rx_buffers = (dma_buffer_t *)malloc((uint64_t)count * (uint64_t)sizeof(dma_buffer_t));
    if (g_rx_buffers == NULL) {
        return 0;
    }
    memset(g_rx_buffers, 0, (size_t)count * sizeof(dma_buffer_t));

    for (uint16_t i = 0; i < count; ++i) {
        uint64_t phys = 0;
        uint8_t *buffer = (uint8_t *)dma_alloc(VIRTIO_NET_BUF_BYTES, &phys);
        if (buffer == NULL || phys == 0) {
            return 0;
        }

        g_rx_buffers[i].virt = buffer;
        g_rx_buffers[i].phys = phys;

        rxq->desc[i].addr = phys;
        rxq->desc[i].len = VIRTIO_NET_BUF_BYTES;
        rxq->desc[i].flags = VIRTQ_DESC_F_WRITE;
        rxq->desc[i].next = 0u;

        rxq->avail->ring[rxq->avail_idx % count] = i;
        rxq->avail_idx = (uint16_t)(rxq->avail_idx + 1u);
    }

    memory_barrier();
    rxq->avail->idx = rxq->avail_idx;
    memory_barrier();
    virtqueue_notify(rxq);

    return 1;
}

static int virtio_net_setup_tx_buffers(virtqueue_t *txq)
{
    if (txq == NULL || txq->queue_size == 0u || txq->desc == NULL || txq->avail == NULL) {
        return 0;
    }

    uint16_t count = txq->queue_size;

    g_tx_buffers = (dma_buffer_t *)malloc((uint64_t)count * (uint64_t)sizeof(dma_buffer_t));
    if (g_tx_buffers == NULL) {
        return 0;
    }
    memset(g_tx_buffers, 0, (size_t)count * sizeof(dma_buffer_t));

    g_tx_in_use = (uint8_t *)malloc((uint64_t)count);
    if (g_tx_in_use == NULL) {
        return 0;
    }
    memset(g_tx_in_use, 0, (size_t)count);

    for (uint16_t i = 0; i < count; ++i) {
        uint64_t phys = 0;
        uint8_t *buffer = (uint8_t *)dma_alloc(VIRTIO_NET_BUF_BYTES, &phys);
        if (buffer == NULL || phys == 0) {
            return 0;
        }
        g_tx_buffers[i].virt = buffer;
        g_tx_buffers[i].phys = phys;

        txq->desc[i].addr = phys;
        txq->desc[i].len = 0u;
        txq->desc[i].flags = 0u;
        txq->desc[i].next = 0u;
    }

    return 1;
}

static void virtio_net_reap_tx_locked(void)
{
    if (g_tx_in_use == NULL || g_tx_queue.queue_size == 0u || g_tx_queue.used == NULL) {
        return;
    }

    while ((uint16_t)(g_tx_queue.used->idx - g_tx_queue.used_idx_seen) != 0u) {
        uint16_t used_slot = (uint16_t)(g_tx_queue.used_idx_seen % g_tx_queue.queue_size);
        virtq_used_elem_t used_elem = g_tx_queue.used->ring[used_slot];
        uint16_t desc_id = (uint16_t)(used_elem.id & 0xFFFFu);
        if (desc_id < g_tx_queue.queue_size) {
            g_tx_in_use[desc_id] = 0u;
        }
        g_tx_queue.used_idx_seen = (uint16_t)(g_tx_queue.used_idx_seen + 1u);
    }
}

static int virtio_net_pop_rx_locked(uint8_t *out_frame, uint16_t *out_len)
{
    if (out_frame == NULL || out_len == NULL) {
        return 0;
    }
    if (g_rx_buffers == NULL || g_rx_queue.queue_size == 0u || g_rx_queue.used == NULL) {
        return 0;
    }
    if ((uint16_t)(g_rx_queue.used->idx - g_rx_queue.used_idx_seen) == 0u) {
        return 0;
    }

    uint16_t used_slot = (uint16_t)(g_rx_queue.used_idx_seen % g_rx_queue.queue_size);
    virtq_used_elem_t used_elem = g_rx_queue.used->ring[used_slot];
    uint16_t desc_id = (uint16_t)(used_elem.id & 0xFFFFu);
    uint32_t used_len = used_elem.len;

    g_rx_queue.used_idx_seen = (uint16_t)(g_rx_queue.used_idx_seen + 1u);

    uint16_t frame_len = 0u;
    if (desc_id < g_rx_queue.queue_size && used_len > VIRTIO_NET_HDR_BYTES) {
        uint32_t payload_len = used_len - VIRTIO_NET_HDR_BYTES;
        if (payload_len > VIRTIO_NET_MAX_FRAME_BYTES) {
            payload_len = VIRTIO_NET_MAX_FRAME_BYTES;
        }

        if (payload_len > 0u) {
            memcpy(out_frame,
                   g_rx_buffers[desc_id].virt + VIRTIO_NET_HDR_BYTES,
                   (size_t)payload_len);
            frame_len = (uint16_t)payload_len;
        }

        g_rx_queue.avail->ring[g_rx_queue.avail_idx % g_rx_queue.queue_size] = desc_id;
        memory_barrier();
        g_rx_queue.avail_idx = (uint16_t)(g_rx_queue.avail_idx + 1u);
        g_rx_queue.avail->idx = g_rx_queue.avail_idx;
        memory_barrier();
    }

    *out_len = frame_len;
    return 1;
}

bool virtio_net_init(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_virtio_net_lock);

    if (g_virtio_net_ready != 0) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return true;
    }

    if (!dma_init()) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    virtio_net_pci_t dev;
    if (!find_virtio_net(&dev)) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    virtio_transport_t transport;
    if (!virtio_find_transport_caps(&dev, &transport)) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    uint64_t negotiated_features = 0;
    if (!virtio_net_device_init(&dev, &transport, &negotiated_features)) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    uint16_t queue_count = common_read16(transport.common_cfg, COMMON_OFF_NUM_QUEUES);
    if (queue_count < 2u) {
        common_write8(transport.common_cfg,
                      COMMON_OFF_DEVICE_STATUS,
                      (uint8_t)(common_read8(transport.common_cfg, COMMON_OFF_DEVICE_STATUS) |
                                VIRTIO_STATUS_FAILED));
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    memset(&g_rx_queue, 0, sizeof(g_rx_queue));
    memset(&g_tx_queue, 0, sizeof(g_tx_queue));

    if (!virtqueue_setup(&transport, &g_rx_queue, VIRTIO_NET_RX_QUEUE_INDEX)) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    if (!virtqueue_setup(&transport, &g_tx_queue, VIRTIO_NET_TX_QUEUE_INDEX)) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    if (!virtio_net_setup_rx_buffers(&g_rx_queue)) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    if (!virtio_net_setup_tx_buffers(&g_tx_queue)) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    if ((negotiated_features & VIRTIO_NET_F_MAC) != 0u && transport.device_cfg != NULL) {
        for (uint8_t i = 0; i < 6u; ++i) {
            g_virtio_net_mac[i] = *(volatile uint8_t *)(transport.device_cfg + i);
        }
    }

    if ((negotiated_features & VIRTIO_NET_F_MTU) != 0u && transport.device_cfg != NULL) {
        uint16_t mtu = *(volatile uint16_t *)(transport.device_cfg + 10u);
        if (mtu >= 576u && mtu <= VIRTIO_NET_MAX_FRAME_BYTES) {
            g_virtio_net_mtu = mtu;
        }
    }

    g_transport = transport;

    uint8_t status = common_read8(g_transport.common_cfg, COMMON_OFF_DEVICE_STATUS);
    common_write8(g_transport.common_cfg,
                  COMMON_OFF_DEVICE_STATUS,
                  (uint8_t)(status | VIRTIO_STATUS_DRIVER_OK));

    g_virtio_net_ready = 1;

    spinlock_unlock(&g_virtio_net_lock);
    irq_restore(irq_flags);
    
    for (uint8_t i = 0; i < 6u; ++i) {
        uint8_t b = g_virtio_net_mac[i];
        char buf[4];
        const char *hex = "0123456789abcdef";
        buf[0] = hex[(b >> 4) & 0xFu];
        buf[1] = hex[b & 0xFu];
        buf[2] = (i == 5u) ? '\n' : ':';
        buf[3] = '\0';
        serial_write_string(buf);
    }
    return true;
}

bool virtio_net_is_ready(void)
{
    return g_virtio_net_ready != 0;
}

uint16_t virtio_net_mtu(void)
{
    return g_virtio_net_mtu;
}

void virtio_net_get_mac(uint8_t mac_out[6])
{
    if (mac_out == NULL) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_virtio_net_lock);

    for (uint8_t i = 0; i < 6u; ++i) {
        mac_out[i] = g_virtio_net_mac[i];
    }

    spinlock_unlock(&g_virtio_net_lock);
    irq_restore(irq_flags);
}

bool virtio_net_send(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len == 0u || frame_len > g_virtio_net_mtu + 14u) {
        return false;
    }

    int should_notify = 0;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_virtio_net_lock);

    if (g_virtio_net_ready == 0 ||
        g_tx_buffers == NULL ||
        g_tx_in_use == NULL ||
        g_tx_queue.queue_size == 0u) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    virtio_net_reap_tx_locked();

    uint16_t desc_id = g_tx_queue.queue_size;
    for (uint16_t i = 0; i < g_tx_queue.queue_size; ++i) {
        if (g_tx_in_use[i] == 0u) {
            desc_id = i;
            break;
        }
    }

    if (desc_id >= g_tx_queue.queue_size) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    uint8_t *tx_buffer = g_tx_buffers[desc_id].virt;
    if (tx_buffer == NULL) {
        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);
        return false;
    }

    memset(tx_buffer, 0, VIRTIO_NET_HDR_BYTES);
    memcpy(tx_buffer + VIRTIO_NET_HDR_BYTES, frame, (size_t)frame_len);

    g_tx_queue.desc[desc_id].addr = g_tx_buffers[desc_id].phys;
    g_tx_queue.desc[desc_id].len = (uint32_t)VIRTIO_NET_HDR_BYTES + (uint32_t)frame_len;
    g_tx_queue.desc[desc_id].flags = 0u;
    g_tx_queue.desc[desc_id].next = 0u;

    g_tx_in_use[desc_id] = 1u;

    g_tx_queue.avail->ring[g_tx_queue.avail_idx % g_tx_queue.queue_size] = desc_id;
    memory_barrier();
    g_tx_queue.avail_idx = (uint16_t)(g_tx_queue.avail_idx + 1u);
    g_tx_queue.avail->idx = g_tx_queue.avail_idx;
    memory_barrier();

    should_notify = 1;

    spinlock_unlock(&g_virtio_net_lock);
    irq_restore(irq_flags);

    if (should_notify != 0) {
        virtqueue_notify(&g_tx_queue);
    }

    return true;
}

void virtio_net_poll(void)
{
    uint8_t frame_buffer[VIRTIO_NET_MAX_FRAME_BYTES];
    uint32_t processed = 0u;

    for (;;) {
        uint16_t frame_len = 0u;
        virtio_net_rx_callback_t cb = NULL;
        int got_packet = 0;

        if (processed >= 16u) {
            break;
        }

        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_virtio_net_lock);

        if (g_virtio_net_ready == 0) {
            spinlock_unlock(&g_virtio_net_lock);
            irq_restore(irq_flags);
            return;
        }

        virtio_net_reap_tx_locked();
        got_packet = virtio_net_pop_rx_locked(frame_buffer, &frame_len);
        cb = g_rx_callback;

        spinlock_unlock(&g_virtio_net_lock);
        irq_restore(irq_flags);

        if (got_packet == 0) {
            return;
        }

        if (frame_len > 0u && cb != NULL) {
            cb(frame_buffer, frame_len);
        }

        virtqueue_notify(&g_rx_queue);
        processed++;
    }
}

void virtio_net_set_rx_callback(virtio_net_rx_callback_t cb)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_virtio_net_lock);

    g_rx_callback = cb;

    spinlock_unlock(&g_virtio_net_lock);
    irq_restore(irq_flags);
}

#ifdef IMPLUS_DRIVER_MODULE
static const driver_nic_t g_virtio_net_driver = {
    .init = virtio_net_init,
    .is_ready = virtio_net_is_ready,
    .mtu = virtio_net_mtu,
    .get_mac = virtio_net_get_mac,
    .send_frame = virtio_net_send,
    .poll = virtio_net_poll,
    .set_rx_callback = virtio_net_set_rx_callback,
};

static void virtio_net_shutdown(void)
{
    g_virtio_net_ready = 0;
    g_api = NULL;
}

/*
 * Worked example of the declarative bus_matches[]/probe() model
 * (Kernel/Drivers/Module/BusRegistry.h, DriverBinary.h) added alongside
 * this driver's pre-existing self-scan (find_virtio_net(), used by
 * virtio_net_init() above) -- both paths call the same idempotent
 * virtio_net_init() (it no-ops once g_virtio_net_ready is set), so this is
 * purely additive: probe() just gives PCI_Main.c's bus scan a way to
 * trigger bring-up as soon as a matching device is enumerated, instead of
 * only lazily via nic_init() (Drivers/Module/NIC.c) trying this
 * driver's driver_nic_t.init later. `dev` (a driver_pci_device_t via
 * bus_context) isn't consulted -- virtio_net_init() re-scans PCI itself
 * rather than trusting it, since that scan is what the pre-existing
 * self-contained design already relies on; threading bus_context through
 * instead of re-scanning is a reasonable follow-up, not needed for
 * correctness here.
 */
static const driver_bus_match_t g_virtio_net_bus_matches[] = {
    {
        .bus_type = DEVICE_TYPE_PCI,
        .vendor_id = (uint16_t)VIRTIO_VENDOR_ID,
        .device_id = (uint16_t)VIRTIO_NET_DEVICE_ID_MODERN,
        .match_flags = DRIVER_BUS_MATCH_VENDOR | DRIVER_BUS_MATCH_DEVICE,
    },
};

static bool virtio_net_probe(const bus_device_t *dev)
{
    (void)dev;
    return virtio_net_init();
}

/* No per-device teardown exists separate from the whole module's
 * shutdown() above -- virtio-net over PCI isn't exercised hot-removed in
 * practice (tied to the VM's lifetime in QEMU). Honest no-op rather than
 * fabricating a reset sequence. */
static void virtio_net_remove(const bus_device_t *dev)
{
    (void)dev;
}

static const driver_module_descriptor_t g_virtio_net_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_NIC,
    .load_priority = 50u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_virtio_net_driver,
    .shutdown = virtio_net_shutdown,
    .bus_matches = g_virtio_net_bus_matches,
    .bus_match_count = (uint32_t)(sizeof(g_virtio_net_bus_matches) / sizeof(g_virtio_net_bus_matches[0])),
    .probe = virtio_net_probe,
    .remove = virtio_net_remove,
};

#undef malloc
#undef free
#undef dma_alloc
#undef memset
#undef memcpy
#undef map_mmio_range
#undef pci_read_config
#undef pci_write_config
#undef cpu_save_interrupts
#undef cpu_restore_interrupts
#undef cpu_pause

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL ||
        api->malloc == NULL ||
        api->free == NULL ||
        api->dma_alloc == NULL ||
        api->memset == NULL ||
        api->memcpy == NULL ||
        api->hw.map_mmio_range == NULL ||
        api->pci_read_config == NULL ||
        api->pci_write_config == NULL ||
        api->hal.cpu_save_interrupts == NULL ||
        api->hal.cpu_restore_interrupts == NULL ||
        api->hal.cpu_pause == NULL) {
        return NULL;
    }

    g_api = api;
    return &g_virtio_net_module;
}
#endif