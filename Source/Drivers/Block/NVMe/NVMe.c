#include "Drivers/Module/DriverBinary.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NVME_MAX_NAMESPACES 16u
#define NVME_QUEUE_DEPTH    64u
#define NVME_PAGE_SIZE      4096u
#define NVME_IO_BYTES       (128u * 1024u)
#define NVME_TIMEOUT_MS     5000u

#define NVME_CC_ENABLE      1u
#define NVME_CSTS_READY     1u

typedef struct __attribute__((packed)) {
    uint8_t opcode;
    uint8_t flags;
    uint16_t cid;
    uint32_t nsid;
    uint64_t reserved;
    uint64_t mptr;
    uint64_t prp1;
    uint64_t prp2;
    uint32_t cdw10;
    uint32_t cdw11;
    uint32_t cdw12;
    uint32_t cdw13;
    uint32_t cdw14;
    uint32_t cdw15;
} nvme_command_t;

typedef struct __attribute__((packed)) {
    uint32_t result;
    uint32_t reserved;
    uint16_t sq_head;
    uint16_t sq_id;
    uint16_t cid;
    uint16_t status;
} nvme_completion_t;

typedef struct {
    nvme_command_t *sq;
    nvme_completion_t *cq;
    uint64_t sq_phys;
    uint64_t cq_phys;
    volatile uint32_t *sq_doorbell;
    volatile uint32_t *cq_doorbell;
    uint16_t qid;
    uint16_t tail;
    uint16_t head;
    uint16_t next_cid;
    uint8_t phase;
} nvme_queue_t;

typedef struct {
    uint32_t nsid;
    uint64_t block_count;
    uint32_t block_size;
} nvme_namespace_t;

static const driver_binary_t *g_api;
static volatile uint8_t *g_regs;
static driver_pci_device_t g_pci;
static nvme_queue_t g_admin;
static nvme_queue_t g_io;
static nvme_namespace_t g_namespaces[NVME_MAX_NAMESPACES];
static uint32_t g_namespace_count;
static uint8_t *g_identify;
static uint64_t g_identify_phys;
static uint8_t *g_io_buffer;
static uint64_t g_io_buffer_phys;
static uint64_t *g_prp_list;
static uint64_t g_prp_list_phys;
static uint32_t g_doorbell_stride;
static bool g_ready;

static uint32_t mmio_read32(uint32_t offset)
{
    return *(volatile uint32_t *)(g_regs + offset);
}

static uint64_t mmio_read64(uint32_t offset)
{
    uint64_t low = mmio_read32(offset);
    uint64_t high = mmio_read32(offset + 4u);
    return low | (high << 32u);
}

static void mmio_write32(uint32_t offset, uint32_t value)
{
    *(volatile uint32_t *)(g_regs + offset) = value;
}

static void mmio_write64(uint32_t offset, uint64_t value)
{
    mmio_write32(offset, (uint32_t)value);
    mmio_write32(offset + 4u, (uint32_t)(value >> 32u));
}

static bool nvme_wait_ready(bool ready)
{
    uint64_t start = g_api->timer.monotonic_ns();
    for (;;) {
        bool controller_ready =
            (mmio_read32(0x1Cu) & NVME_CSTS_READY) != 0u;
        if (controller_ready == ready) {
            return true;
        }
        if (g_api->timer.monotonic_ns() - start >
            (uint64_t)NVME_TIMEOUT_MS * 1000000ULL) {
            return false;
        }
        g_api->hal.cpu_pause();
    }
}

static bool nvme_queue_alloc(nvme_queue_t *queue, uint16_t qid,
                             uint32_t doorbell_stride)
{
    size_t sq_bytes = NVME_QUEUE_DEPTH * sizeof(nvme_command_t);
    size_t cq_bytes = NVME_QUEUE_DEPTH * sizeof(nvme_completion_t);
    queue->sq = g_api->mem.dma_alloc_ex(sq_bytes, NVME_PAGE_SIZE, 0u,
                                         &queue->sq_phys);
    queue->cq = g_api->mem.dma_alloc_ex(cq_bytes, NVME_PAGE_SIZE, 0u,
                                         &queue->cq_phys);
    if (queue->sq == NULL || queue->cq == NULL) {
        return false;
    }
    queue->qid = qid;
    queue->tail = 0u;
    queue->head = 0u;
    queue->phase = 1u;
    queue->next_cid = 1u;
    queue->sq_doorbell = (volatile uint32_t *)(g_regs + 0x1000u +
        (uint32_t)(2u * qid) * doorbell_stride);
    queue->cq_doorbell = (volatile uint32_t *)(g_regs + 0x1000u +
        (uint32_t)(2u * qid + 1u) * doorbell_stride);
    return true;
}

static bool nvme_submit(nvme_queue_t *queue, nvme_command_t *command,
                        uint32_t *result)
{
    command->cid = queue->next_cid++;
    queue->sq[queue->tail] = *command;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    queue->tail = (uint16_t)((queue->tail + 1u) % NVME_QUEUE_DEPTH);
    *queue->sq_doorbell = queue->tail;

    uint64_t start = g_api->timer.monotonic_ns();
    for (;;) {
        volatile nvme_completion_t *completion = &queue->cq[queue->head];
        uint16_t status = completion->status;
        if ((status & 1u) == queue->phase) {
            if (completion->cid != command->cid) {
                return false;
            }
            if (result != NULL) {
                *result = completion->result;
            }
            bool success = (status & 0xFFFEu) == 0u;
            queue->head =
                (uint16_t)((queue->head + 1u) % NVME_QUEUE_DEPTH);
            if (queue->head == 0u) {
                queue->phase ^= 1u;
            }
            *queue->cq_doorbell = queue->head;
            return success;
        }
        if (g_api->timer.monotonic_ns() - start >
            (uint64_t)NVME_TIMEOUT_MS * 1000000ULL) {
            return false;
        }
        g_api->hal.cpu_pause();
    }
}

static bool nvme_admin_identify(uint32_t nsid, uint32_t cns)
{
    nvme_command_t command;
    g_api->memset(&command, 0, sizeof(command));
    g_api->memset(g_identify, 0, NVME_PAGE_SIZE);
    command.opcode = 0x06u;
    command.nsid = nsid;
    command.prp1 = g_identify_phys;
    command.cdw10 = cns;
    return nvme_submit(&g_admin, &command, NULL);
}

static bool nvme_create_io_queue(void)
{
    nvme_command_t command;
    g_api->memset(&command, 0, sizeof(command));
    command.opcode = 0x05u;
    command.prp1 = g_io.cq_phys;
    command.cdw10 = g_io.qid | ((NVME_QUEUE_DEPTH - 1u) << 16u);
    command.cdw11 = 1u;
    if (!nvme_submit(&g_admin, &command, NULL)) {
        return false;
    }
    g_api->memset(&command, 0, sizeof(command));
    command.opcode = 0x01u;
    command.prp1 = g_io.sq_phys;
    command.cdw10 = g_io.qid | ((NVME_QUEUE_DEPTH - 1u) << 16u);
    command.cdw11 = 1u | ((uint32_t)g_io.qid << 16u);
    return nvme_submit(&g_admin, &command, NULL);
}

static bool nvme_prepare_prps(uint32_t bytes, uint64_t *prp1,
                              uint64_t *prp2)
{
    *prp1 = g_io_buffer_phys;
    *prp2 = 0u;
    if (bytes <= NVME_PAGE_SIZE) {
        return true;
    }
    uint32_t pages = (bytes + NVME_PAGE_SIZE - 1u) / NVME_PAGE_SIZE;
    if (pages == 2u) {
        *prp2 = g_io_buffer_phys + NVME_PAGE_SIZE;
        return true;
    }
    if (pages - 1u > NVME_PAGE_SIZE / sizeof(uint64_t)) {
        return false;
    }
    for (uint32_t i = 1u; i < pages; ++i) {
        g_prp_list[i - 1u] =
            g_io_buffer_phys + (uint64_t)i * NVME_PAGE_SIZE;
    }
    *prp2 = g_prp_list_phys;
    return true;
}

static bool nvme_rw(uint32_t device_index, uint64_t lba, void *buffer,
                    uint32_t block_count, bool write)
{
    if (!g_ready || device_index >= g_namespace_count ||
        buffer == NULL || block_count == 0u) {
        return false;
    }
    nvme_namespace_t *ns = &g_namespaces[device_index];
    uint32_t max_blocks = NVME_IO_BYTES / ns->block_size;
    uint8_t *bytes = (uint8_t *)buffer;
    for (uint32_t done = 0u; done < block_count;) {
        uint32_t count = block_count - done;
        if (count > max_blocks) {
            count = max_blocks;
        }
        uint32_t byte_count = count * ns->block_size;
        if (write) {
            g_api->memcpy(g_io_buffer, bytes + (size_t)done * ns->block_size,
                          byte_count);
        }
        nvme_command_t command;
        g_api->memset(&command, 0, sizeof(command));
        command.opcode = write ? 0x01u : 0x02u;
        command.nsid = ns->nsid;
        uint64_t prp1;
        uint64_t prp2;
        if (!nvme_prepare_prps(byte_count, &prp1, &prp2)) {
            return false;
        }
        command.prp1 = prp1;
        command.prp2 = prp2;
        uint64_t slba = lba + done;
        command.cdw10 = (uint32_t)slba;
        command.cdw11 = (uint32_t)(slba >> 32u);
        command.cdw12 = count - 1u;
        if (!nvme_submit(&g_io, &command, NULL)) {
            return false;
        }
        if (!write) {
            g_api->memcpy(bytes + (size_t)done * ns->block_size, g_io_buffer,
                          byte_count);
        }
        done += count;
    }
    return true;
}

static bool nvme_read(uint32_t device_index, uint64_t lba, void *buffer,
                      uint32_t block_count)
{
    return nvme_rw(device_index, lba, buffer, block_count, false);
}

static bool nvme_write(uint32_t device_index, uint64_t lba,
                       const void *buffer, uint32_t block_count)
{
    return nvme_rw(device_index, lba, (void *)buffer, block_count, true);
}

static bool nvme_flush(uint32_t device_index)
{
    if (device_index >= g_namespace_count) {
        return false;
    }
    nvme_command_t command;
    g_api->memset(&command, 0, sizeof(command));
    command.opcode = 0x00u;
    command.nsid = g_namespaces[device_index].nsid;
    return nvme_submit(&g_io, &command, NULL);
}

static bool nvme_get_info(uint32_t index, driver_block_info_t *out)
{
    if (index >= g_namespace_count || out == NULL) {
        return false;
    }
    g_api->memset(out, 0, sizeof(*out));
    out->block_count = g_namespaces[index].block_count;
    out->logical_block_size = g_namespaces[index].block_size;
    out->physical_block_size = g_namespaces[index].block_size;
    out->flags = DRIVER_BLOCK_FLAG_WRITABLE;
    out->transport = DRIVER_BLOCK_TRANSPORT_NVME;
    out->identity_flags = DRIVER_BLOCK_IDENTITY_PCI_VALID;
    out->pci_segment = 0u;
    out->pci_bus = g_pci.bus;
    out->pci_device = g_pci.device;
    out->pci_function = g_pci.function;
    out->controller_port = UINT16_MAX;
    out->namespace_id = g_namespaces[index].nsid;
    const char model[] = "NVMe namespace";
    g_api->memcpy(out->model, model, sizeof(model));
    return true;
}

static bool nvme_init(void)
{
    if (g_ready) {
        return true;
    }
    uint32_t device_count = g_api->pci.get_device_count();
    bool found = false;
    for (uint32_t i = 0u; i < device_count; ++i) {
        driver_pci_device_t device;
        if (g_api->pci.get_device(i, &device) &&
            device.class_code == 0x01u && device.subclass == 0x08u &&
            device.prog_if == 0x02u) {
            g_pci = device;
            found = true;
            break;
        }
    }
    driver_pci_bar_t bar;
    if (!found || !g_api->pci.get_bar(g_pci.bus, g_pci.device,
                                      g_pci.function, 0u, &bar) ||
        bar.is_io || bar.size < 0x2000u) {
        return false;
    }
    g_api->pci.enable_bus_master(g_pci.bus, g_pci.device, g_pci.function);
    g_regs = g_api->hw.map_mmio_range(bar.address, (size_t)bar.size);
    if (g_regs == NULL) {
        return false;
    }
    uint64_t cap = mmio_read64(0x00u);
    uint32_t mqes = (uint32_t)(cap & 0xFFFFu) + 1u;
    if (mqes < NVME_QUEUE_DEPTH || ((cap >> 37u) & 1u) == 0u) {
        return false;
    }
    g_doorbell_stride = 4u << ((cap >> 32u) & 0xFu);
    mmio_write32(0x14u, 0u);
    if (!nvme_wait_ready(false)) {
        return false;
    }
    if (!nvme_queue_alloc(&g_admin, 0u, g_doorbell_stride)) {
        return false;
    }
    g_identify = g_api->mem.dma_alloc_ex(NVME_PAGE_SIZE, NVME_PAGE_SIZE,
                                          0u, &g_identify_phys);
    g_io_buffer = g_api->mem.dma_alloc_ex(NVME_IO_BYTES, NVME_PAGE_SIZE,
                                           0u, &g_io_buffer_phys);
    g_prp_list = g_api->mem.dma_alloc_ex(NVME_PAGE_SIZE, NVME_PAGE_SIZE,
                                          0u, &g_prp_list_phys);
    if (g_identify == NULL || g_io_buffer == NULL || g_prp_list == NULL) {
        return false;
    }
    mmio_write32(0x24u, ((NVME_QUEUE_DEPTH - 1u) << 16u) |
                         (NVME_QUEUE_DEPTH - 1u));
    mmio_write64(0x28u, g_admin.sq_phys);
    mmio_write64(0x30u, g_admin.cq_phys);
    mmio_write32(0x14u, NVME_CC_ENABLE | (6u << 16u) | (4u << 20u));
    if (!nvme_wait_ready(true) ||
        !nvme_queue_alloc(&g_io, 1u, g_doorbell_stride) ||
        !nvme_create_io_queue()) {
        return false;
    }

    g_namespace_count = 0u;
    if (!nvme_admin_identify(0u, 2u)) {
        return false;
    }
    const uint32_t *nsids = (const uint32_t *)g_identify;
    for (uint32_t i = 0u; i < 1024u &&
         g_namespace_count < NVME_MAX_NAMESPACES; ++i) {
        uint32_t nsid = nsids[i];
        if (nsid == 0u) {
            break;
        }
        if (!nvme_admin_identify(nsid, 0u)) {
            continue;
        }
        uint64_t blocks = *(const uint64_t *)(g_identify + 0u);
        uint8_t format = g_identify[26u] & 0x0Fu;
        uint8_t lbads = g_identify[128u + (uint32_t)format * 4u + 2u];
        if (blocks == 0u || lbads < 9u || lbads > 16u) {
            continue;
        }
        nvme_namespace_t *ns = &g_namespaces[g_namespace_count++];
        ns->nsid = nsid;
        ns->block_count = blocks;
        ns->block_size = 1u << lbads;
    }
    g_ready = g_namespace_count != 0u;
    return g_ready;
}

static bool nvme_is_ready(void) { return g_ready; }
static uint32_t nvme_get_count(void) { return g_namespace_count; }

static const driver_storage_t g_storage = {
    .name = "nvme",
    .priority = 20u,
    .init = nvme_init,
    .is_ready = nvme_is_ready,
    .get_device_count = nvme_get_count,
    .get_info = nvme_get_info,
    .read_blocks = nvme_read,
    .write_blocks = nvme_write,
    .flush = nvme_flush,
};

static void nvme_shutdown(void)
{
    if (g_regs != NULL) {
        mmio_write32(0x14u, 0u);
        (void)nvme_wait_ready(false);
    }
    g_ready = false;
}

static const driver_module_descriptor_t g_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_BLOCK,
    .load_priority = 41u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_storage,
    .shutdown = nvme_shutdown,
};

__attribute__((visibility("default")))
const driver_module_descriptor_t *driver_module_init(
    const driver_binary_t *api)
{
    if (api == NULL || api->version_major != DRIVER_API_VERSION_MAJOR ||
        api->pci.get_device_count == NULL || api->mem.dma_alloc_ex == NULL ||
        api->hw.map_mmio_range == NULL || api->timer.monotonic_ns == NULL) {
        return NULL;
    }
    g_api = api;
    return &g_module;
}
