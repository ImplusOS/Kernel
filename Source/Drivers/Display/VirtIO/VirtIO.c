#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "Platform/io/IO_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "mmu/Paging_Main.h"
#include "../Display_Driver.h"
#include "Drivers/Module/DriverBinary.h"
#include "Drivers/Module/PCI_Main.h"

#define VIRTIO_VENDOR_ID 0x1AF4
#define VIRTIO_GPU_DEVICE_ID 0x1050
#define VIRTIO_GPU_DEVICE_NAME "VirtIO GPU"

#define PCI_CAP_ID_VENDOR 0x09

#define VIRTIO_PCI_CAP_COMMON_CFG 1
#define VIRTIO_PCI_CAP_NOTIFY_CFG 2
#define VIRTIO_PCI_CAP_DEVICE_CFG 4

#define VIRTIO_STATUS_ACKNOWLEDGE 0x01
#define VIRTIO_STATUS_DRIVER      0x02
#define VIRTIO_STATUS_DRIVER_OK   0x04
#define VIRTIO_STATUS_FEATURES_OK 0x08
#define VIRTIO_STATUS_FAILED      0x80

#define VIRTIO_F_VERSION_1 (1u << 0)

#define VIRTQ_DESC_F_NEXT  1
#define VIRTQ_DESC_F_WRITE 2

#define VIRTIO_GPU_CMD_GET_DISPLAY_INFO        0x0100
#define VIRTIO_GPU_CMD_RESOURCE_CREATE_2D      0x0101
#define VIRTIO_GPU_CMD_RESOURCE_UNREF          0x0102
#define VIRTIO_GPU_CMD_SET_SCANOUT             0x0103
#define VIRTIO_GPU_CMD_RESOURCE_FLUSH          0x0104
#define VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D     0x0105
#define VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING 0x0106
#define VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING 0x0107
#define VIRTIO_GPU_CMD_GET_CAPSET_INFO         0x0108
#define VIRTIO_GPU_CMD_GET_CAPSET              0x0109
#define VIRTIO_GPU_CMD_GET_EDID                0x010A
#define VIRTIO_GPU_CMD_RESOURCE_ASSIGN_UUID    0x010B
#define VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB    0x010C
#define VIRTIO_GPU_CMD_SET_SCANOUT_BLOB        0x010D

/* 3D (virgl) commands: keep these in sync with linux/virtio_gpu.h. */
#define VIRTIO_GPU_CMD_CTX_CREATE              0x0200
#define VIRTIO_GPU_CMD_CTX_DESTROY             0x0201
#define VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE     0x0202
#define VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE     0x0203
#define VIRTIO_GPU_CMD_RESOURCE_CREATE_3D      0x0204
#define VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D     0x0205
#define VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D   0x0206
#define VIRTIO_GPU_CMD_SUBMIT_3D               0x0207
#define VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB       0x0208
#define VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB     0x0209

#define VIRTIO_GPU_RESP_OK_NODATA        0x1100
#define VIRTIO_GPU_RESP_OK_DISPLAY_INFO  0x1101
#define VIRTIO_GPU_RESP_OK_CAPSET_INFO   0x1102
#define VIRTIO_GPU_RESP_OK_CAPSET        0x1103
#define VIRTIO_GPU_RESP_OK_EDID          0x1104

#define VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM 1
#define VIRTIO_GPU_CAPSET_VIRGL          1u

#define VIRTIO_GPU_EVENT_DISPLAY    (1u << 0)
#define VIRTIO_GPU_F_VIRGL          (1u << 0)
#define VIRTIO_GPU_F_EDID           (1u << 1)
#define VIRTIO_GPU_F_RESOURCE_BLOB  (1u << 3)
#define VIRTIO_GPU_F_CONTEXT_INIT   (1u << 4)
#define VIRTIO_GPU_MAX_SCANOUTS  16u
#define VIRTIO_GPU_MAX_MODES     8u
#define VIRTIO_GPU_MAX_PIXELS    (64ull * 1024ull * 1024ull)

/* How long a single control-queue command may take before the driver gives up.
 * The first command after DRIVER_OK can take a while: QEMU spawns the GPU
 * worker thread lazily, and a 10M-iteration spin (a few milliseconds) expired
 * before the device answered, which made the driver install a bogus 1024x768
 * fallback layout and re-layout -- moving the framebuffer -- once the real
 * mode arrived. */
#define VIRTIO_GPU_CMD_TIMEOUT_MS 1000u
/* Fallback busy-wait budget when the module API exposes no clock. */
#define VIRTIO_GPU_SPIN_BUDGET  400000000u
/* Mesa virgl capsets are returned as variable-size control-queue payloads;
 * reserve bounded space beyond the fixed EDID response. */
#define VIRTIO_GPU_RESP_BUF_BYTES (64u * 1024u)
#define VIRTIO_GPU_CMD_BUF_BYTES  (1024u * 1024u)
#define VIRTIO_GPU_MAX_CAPSETS 16u
/* The desktop framebuffer is allocated with headroom so a later mode change
 * usually reuses the same pages instead of moving them under user space. */
#define VIRTIO_GPU_FB_MIN_CAPACITY (1920u * 1080u * 4u)
/* Former framebuffers are retained, not freed: user space holds a direct
 * mapping of those pages and a stale write must land in dead memory rather
 * than in a reused kernel heap block. */
#define VIRTIO_GPU_RETIRED_FB_MAX 4u

#ifdef IMPLUS_DRIVER_MODULE
static const driver_binary_t *g_driver_api = NULL;

static void *driver_module_memset(void *dst, int value, size_t n) {
    uint8_t *p = (uint8_t *)dst;
    for (size_t i = 0; i < n; ++i) {
        p[i] = (uint8_t)value;
    }
    return dst;
}

static void *driver_module_memcpy(void *dst, const void *src, size_t n) {
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    for (size_t i = 0; i < n; ++i) {
        d[i] = s[i];
    }
    return dst;
}

static int driver_module_strcmp(const char *a, const char *b) {
    while (*a && *a == *b) {
        ++a;
        ++b;
    }
    return (int)(unsigned char)(*a) - (int)(unsigned char)(*b);
}

#define malloc g_driver_api->malloc
#define free g_driver_api->free
#define pci_read_config g_driver_api->pci_read_config
#define pci_write_config g_driver_api->pci_write_config
#define map_mmio_virt g_driver_api->map_mmio_virt
#define memset driver_module_memset
#define memcpy driver_module_memcpy
#define strcmp driver_module_strcmp
#endif

typedef struct {
    uint8_t bus;
    uint8_t device;
    uint8_t func;
    uint64_t bar_addr[6];
    uint8_t bar_is_mem[6];
    uint32_t irq;
} virtio_gpu_pci_t;

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

typedef struct {
    volatile uint8_t *common_cfg;
    volatile uint8_t *notify_base;
    volatile uint8_t *device_cfg;
    uint32_t notify_off_multiplier;
} virtio_pci_transport_t;

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
    uint16_t queue_index;
    uint16_t queue_size;
    volatile virtq_desc_t *desc;
    volatile virtq_avail_t *avail;
    volatile virtq_used_t *used;
    uint16_t avail_idx;
    uint16_t used_idx_seen;
    volatile uint16_t *notify_addr;
    void *raw_ring;
    size_t raw_ring_bytes;
    uint64_t ring_phys;
    /* DMA-owned command/response staging. A timed-out request may still be
     * in flight, so these buffers stay owned by the queue until completion is
     * drained or the device is reset. */
    uint8_t *cmd_buf;
    uint32_t cmd_cap;
    uint64_t cmd_phys;
    uint8_t *resp_buf;
    uint32_t resp_cap;
    uint64_t resp_phys;
} virtqueue_t;

typedef struct __attribute__((packed)) {
    uint32_t type;
    uint32_t flags;
    uint64_t fence_id;
    uint32_t ctx_id;
    uint32_t padding;
} virtio_gpu_ctrl_hdr_t;

typedef struct __attribute__((packed)) {
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
} virtio_gpu_rect_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t resource_id;
    uint32_t format;
    uint32_t width;
    uint32_t height;
} virtio_gpu_resource_create_2d_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t resource_id;
    uint32_t padding;
} virtio_gpu_resource_unref_t;

typedef struct __attribute__((packed)) {
    uint64_t addr;
    uint32_t length;
    uint32_t padding;
} virtio_gpu_mem_entry_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t resource_id;
    uint32_t nr_entries;
    virtio_gpu_mem_entry_t entry;
} virtio_gpu_resource_attach_backing_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t resource_id;
    uint32_t padding;
} virtio_gpu_resource_detach_backing_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    virtio_gpu_rect_t rect;
    uint32_t scanout_id;
    uint32_t resource_id;
} virtio_gpu_set_scanout_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    virtio_gpu_rect_t rect;
    uint64_t offset;
    uint32_t resource_id;
    uint32_t padding;
} virtio_gpu_transfer_to_host_2d_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    virtio_gpu_rect_t rect;
    uint32_t resource_id;
    uint32_t padding;
} virtio_gpu_resource_flush_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
} virtio_gpu_resp_nodata_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_rect_t r;
    uint32_t enabled;
    uint32_t flags;
} virtio_gpu_display_one_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    virtio_gpu_display_one_t pmodes[VIRTIO_GPU_MAX_SCANOUTS];
} virtio_gpu_resp_display_info_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t scanout;
    uint32_t padding;
} virtio_gpu_get_edid_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t size;
    uint32_t padding;
    uint8_t edid[1024];
} virtio_gpu_resp_edid_t;

/* ---- 3D (virgl) structures ---- */

typedef struct __attribute__((packed)) {
    uint32_t x, y, z;
    uint32_t w, h, d;
} virtio_gpu_box_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t nlen;
    uint32_t context_init;
    char debug_name[64];
} virtio_gpu_ctx_create_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
} virtio_gpu_ctx_destroy_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t resource_id;
    uint32_t padding;
} virtio_gpu_ctx_resource_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t size;
    uint32_t padding;
    /* DWORD-aligned virgl command stream follows. */
} virtio_gpu_submit_3d_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t resource_id;
    uint32_t target;
    uint32_t format;
    uint32_t bind;
    uint32_t width;
    uint32_t height;
    uint32_t depth;
    uint32_t array_size;
    uint32_t last_level;
    uint32_t nr_samples;
    uint32_t flags;
    uint32_t padding;
} virtio_gpu_resource_create_3d_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    virtio_gpu_box_t box;
    uint64_t offset;
    uint32_t resource_id;
    uint32_t level;
    uint32_t stride;
    uint32_t layer_stride;
} virtio_gpu_transfer_3d_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t capset_index;
    uint32_t padding;
} virtio_gpu_get_capset_info_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t capset_id;
    uint32_t max_version;
    uint32_t max_size;
    uint32_t padding;
} virtio_gpu_resp_capset_info_t;

typedef struct __attribute__((packed)) {
    virtio_gpu_ctrl_hdr_t hdr;
    uint32_t capset_id;
    uint32_t capset_version;
} virtio_gpu_get_capset_t;

/* Wire-layout guards: a drift here makes the device parse unrelated
 * bytes as virgl commands. Keep these sizes aligned with linux/virtio_gpu.h. */
_Static_assert(sizeof(virtio_gpu_ctrl_hdr_t) == 24u,
               "VirtIO GPU control header ABI mismatch");
_Static_assert(sizeof(virtio_gpu_ctx_create_t) == 96u,
               "VirtIO GPU context-create ABI mismatch");
_Static_assert(sizeof(virtio_gpu_ctx_destroy_t) == 24u,
               "VirtIO GPU context-destroy ABI mismatch");
_Static_assert(sizeof(virtio_gpu_ctx_resource_t) == 32u,
               "VirtIO GPU context-resource ABI mismatch");
_Static_assert(sizeof(virtio_gpu_submit_3d_t) == 32u,
               "VirtIO GPU submit-3D ABI mismatch");
_Static_assert(sizeof(virtio_gpu_resource_create_3d_t) == 72u,
               "VirtIO GPU resource-create-3D ABI mismatch");
_Static_assert(sizeof(virtio_gpu_transfer_3d_t) == 72u,
               "VirtIO GPU transfer-3D ABI mismatch");
_Static_assert(sizeof(virtio_gpu_get_capset_info_t) == 32u,
               "VirtIO GPU capset-info ABI mismatch");
_Static_assert(sizeof(virtio_gpu_resp_capset_info_t) == 40u,
               "VirtIO GPU capset-info response ABI mismatch");
_Static_assert(sizeof(virtio_gpu_get_capset_t) == 32u,
               "VirtIO GPU get-capset ABI mismatch");

typedef struct {
    uint32_t scanout_id;
    uint32_t resource_id;
    int32_t x;
    int32_t y;
    uint32_t width;
    uint32_t height;
    uint32_t physical_width_mm;
    uint32_t physical_height_mm;
    uint32_t refresh_millihz;
    uint32_t flags;
    uint32_t current_mode;
    uint32_t mode_count;
    display_mode_info_t modes[VIRTIO_GPU_MAX_MODES];
    uint32_t *fb;
    uint32_t fb_bytes;
    /* true when fb is the desktop framebuffer (g_gpu_fb) borrowed for direct
     * scanout; such a buffer must never be freed by the monitor cleanup. */
    bool fb_shared;
} virtio_gpu_monitor_t;

#ifdef IMPLUS_DRIVER_MODULE
static uint32_t g_vlog_budget = 16u;

static void vlog(const char *s) {
    if (g_vlog_budget != 0u && g_driver_api != NULL &&
        g_driver_api->serial_write_string != NULL) {
        g_driver_api->serial_write_string(s);
    }
}

static void vlog_num(uint64_t v) {
    if (g_vlog_budget != 0u && g_driver_api != NULL &&
        g_driver_api->serial_write_uint32 != NULL) {
        g_driver_api->serial_write_uint32((uint32_t)v);
    }
}

static void vlog_hex(uint64_t v) {
    static const char digits[] = "0123456789abcdef";
    char buf[19];
    if (g_vlog_budget == 0u || g_driver_api == NULL ||
        g_driver_api->serial_write_string == NULL) {
        return;
    }
    buf[0] = '0';
    buf[1] = 'x';
    for (uint32_t i = 0u; i < 16u; ++i) {
        buf[2u + i] = digits[(v >> (60u - i * 4u)) & 0xFu];
    }
    buf[18] = '\0';
    g_driver_api->serial_write_string(buf);
}

static void vlog_done(void) {
    if (g_vlog_budget != 0u) {
        --g_vlog_budget;
    }
}
#else
static void vlog(const char *s) { (void)s; }
static void vlog_num(uint64_t v) { (void)v; }
static void vlog_hex(uint64_t v) { (void)v; }
static void vlog_done(void) {}
#endif

/*
 * The virtio-gpu control queue is one shared ring for every command in this
 * module, and callers arrive from any CPU: the kernel log path calls
 * driver_manager_display_present() on every printf, userland syscalls poll the
 * config on every frames, and the driver module probe runs concurrently. With
 * no serialisation two CPUs rewrite desc[0]/desc[1] while the device is still
 * reading them, and the used-ring entry is consumed by the wrong caller -- the
 * device then DMAs at whatever address it read out of a half-updated chain.
 * That corrupts arbitrary kernel memory and shows up as random #UD / faults
 * under KVM (where the two CPUs genuinely interleave).
 *
 * Interrupts are masked while the lock is held: the display path is reachable
 * from an interrupt context through printf(), and taking the lock again on the
 * same CPU would deadlock.
 */
static volatile int g_gpu_lock = 0;

static uint64_t gpu_lock(void)
{
    uint64_t irq_state = 0u;

#ifdef IMPLUS_DRIVER_MODULE
    if (g_driver_api != NULL &&
        g_driver_api->hal.cpu_save_interrupts != NULL) {
        irq_state = g_driver_api->hal.cpu_save_interrupts();
    }
#endif

    while (__atomic_exchange_n(&g_gpu_lock, 1, __ATOMIC_ACQUIRE) != 0) {
        while (__atomic_load_n(&g_gpu_lock, __ATOMIC_RELAXED) != 0) {
#ifdef PLATFORM_X86_64
            __asm__ volatile ("pause" ::: "memory");
#else
            __asm__ volatile ("yield" ::: "memory");
#endif
        }
    }
    return irq_state;
}

static void gpu_unlock(uint64_t irq_state)
{
    __atomic_store_n(&g_gpu_lock, 0, __ATOMIC_RELEASE);

#ifdef IMPLUS_DRIVER_MODULE
    if (g_driver_api != NULL &&
        g_driver_api->hal.cpu_restore_interrupts != NULL) {
        g_driver_api->hal.cpu_restore_interrupts(irq_state);
    }
#else
    (void)irq_state;
#endif
}

/* Milliseconds since boot, or 0 when the driver API offers no clock (then the
 * wait helpers fall back to a spin budget). */
static uint64_t gpu_now_ms(void)
{
#ifdef IMPLUS_DRIVER_MODULE
    if (g_driver_api != NULL) {
        if (g_driver_api->timer.monotonic_ns != NULL) {
            return g_driver_api->timer.monotonic_ns() / 1000000ull;
        }
        if (g_driver_api->timer.hz != NULL &&
            g_driver_api->timer.ticks != NULL) {
            uint32_t hz = g_driver_api->timer.hz();
            if (hz != 0u) {
                return (g_driver_api->timer.ticks() * 1000ull) / (uint64_t)hz;
            }
        }
    }
#endif
    return 0ull;
}

static int g_gpu_ready = 0;
static uint32_t g_gpu_width = 0;
static uint32_t g_gpu_height = 0;
static uint32_t *g_gpu_fb = NULL;
static uint32_t g_gpu_fb_bytes = 0;
static uint32_t g_gpu_fb_capacity = 0;
static uint32_t g_gpu_generation = 1u;
static uint32_t g_gpu_monitor_count = 0;
static uint32_t g_gpu_next_resource_id = 1u;
static int32_t g_gpu_origin_x = 0;
static int32_t g_gpu_origin_y = 0;
static uint32_t g_gpu_features0 = 0;
static virtio_gpu_monitor_t g_gpu_monitors[VIRTIO_GPU_MAX_SCANOUTS];
static virtqueue_t g_gpu_controlq;
static virtio_pci_transport_t g_gpu_transport;

static inline uint32_t align_up_u32(uint32_t value, uint32_t align) {
    return (value + align - 1u) & ~(align - 1u);
}

static inline void memory_barrier(void) {
    __asm__ volatile ("" ::: "memory");
}

static uint8_t pci_cfg_read8(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset) {
    uint32_t v = pci_read_config(bus, device, func, (uint8_t)(offset & 0xFC));
    return (uint8_t)((v >> ((offset & 0x3u) * 8u)) & 0xFFu);
}

static uint16_t pci_cfg_read16(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset) {
    uint32_t v = pci_read_config(bus, device, func, (uint8_t)(offset & 0xFC));
    return (uint16_t)((v >> ((offset & 0x2u) * 8u)) & 0xFFFFu);
}

static uint32_t pci_cfg_read32(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset) {
    return pci_read_config(bus, device, func, (uint8_t)(offset & 0xFC));
}

static void pci_cfg_write16(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset, uint16_t value) {
    uint8_t aligned = (uint8_t)(offset & 0xFC);
    uint32_t old = pci_read_config(bus, device, func, aligned);
    uint32_t shift = (uint32_t)((offset & 0x2u) * 8u);
    uint32_t mask = 0xFFFFu << shift;
    uint32_t merged = (old & ~mask) | ((uint32_t)value << shift);
    pci_write_config(bus, device, func, aligned, merged);
}

static void pci_read_bar_addrs(uint8_t bus, uint8_t device, uint8_t func, uint64_t out_bar[6], uint8_t out_is_mem[6]) {
    uint8_t i = 0;
    while (i < 6) {
        uint32_t bar = pci_cfg_read32(bus, device, func, (uint8_t)(0x10 + i * 4));
        out_bar[i] = 0;
        out_is_mem[i] = 0;

        if (bar == 0 || bar == 0xFFFFFFFFu) {
            i++;
            continue;
        }

        if (bar & 0x1u) {
            i++;
            continue;
        }

        out_is_mem[i] = 1;
        if (((bar >> 1) & 0x3u) == 0x2u && i < 5) {
            uint32_t bar_hi = pci_cfg_read32(bus, device, func, (uint8_t)(0x10 + (i + 1) * 4));
            out_bar[i] = (((uint64_t)bar_hi) << 32) | (uint64_t)(bar & ~0xFu);
            i += 2;
            continue;
        }

        out_bar[i] = (uint64_t)(bar & ~0xFu);
        i++;
    }
}

static const char *pci_device_name(uint16_t vendor_id, uint16_t device_id) {
    if (vendor_id == VIRTIO_VENDOR_ID && device_id == VIRTIO_GPU_DEVICE_ID) {
        return VIRTIO_GPU_DEVICE_NAME;
    }
    return NULL;
}

static int find_virtio_gpu(virtio_gpu_pci_t *gpu) {
    for (uint16_t bus = 0; bus < 256; bus++) {
        for (uint8_t device = 0; device < 32; device++) {
            for (uint8_t func = 0; func < 8; func++) {
                uint32_t vd = pci_read_config((uint8_t)bus, device, func, 0x00);
                uint16_t vendor_id = (uint16_t)(vd & 0xFFFFu);
                uint16_t device_id = (uint16_t)((vd >> 16) & 0xFFFFu);

                if (vendor_id == 0xFFFFu) {
                    if (func == 0) {
                        break;
                    }
                    continue;
                }

                const char *device_name = pci_device_name(vendor_id, device_id);
                if (device_name != NULL && strcmp(device_name, VIRTIO_GPU_DEVICE_NAME) == 0) {
                    gpu->bus = (uint8_t)bus;
                    gpu->device = device;
                    gpu->func = func;
                    gpu->irq = pci_cfg_read8((uint8_t)bus, device, func, 0x3C);
                    pci_read_bar_addrs((uint8_t)bus, device, func, gpu->bar_addr, gpu->bar_is_mem);
                    return 1;
                }

                if (func == 0) {
                    uint32_t header_type = pci_read_config((uint8_t)bus, device, func, 0x0C);
                    if (((header_type >> 16) & 0x80u) == 0) {
                        break;
                    }
                }
            }
        }
    }
    return 0;
}

static int virtio_pci_find_caps(const virtio_gpu_pci_t *gpu, virtio_pci_transport_t *t) {
    t->common_cfg = NULL;
    t->notify_base = NULL;
    t->device_cfg = NULL;
    t->notify_off_multiplier = 0;

    uint16_t status = pci_cfg_read16(gpu->bus, gpu->device, gpu->func, 0x06);
    if ((status & (1u << 4)) == 0) {
        return 0;
    }

    uint8_t cap = pci_cfg_read8(gpu->bus, gpu->device, gpu->func, 0x34);
    uint32_t guard = 0;

    while (cap != 0 && cap >= 0x40 && guard++ < 64) {
        uint8_t cap_id = pci_cfg_read8(gpu->bus, gpu->device, gpu->func, cap);
        uint8_t cap_next = pci_cfg_read8(gpu->bus, gpu->device, gpu->func, (uint8_t)(cap + 1));

        if (cap_id == PCI_CAP_ID_VENDOR) {
            virtio_pci_cap_t vcap;
            vcap.cap_vndr = cap_id;
            vcap.cap_next = cap_next;
            vcap.cap_len = pci_cfg_read8(gpu->bus, gpu->device, gpu->func, (uint8_t)(cap + 2));
            vcap.cfg_type = pci_cfg_read8(gpu->bus, gpu->device, gpu->func, (uint8_t)(cap + 3));
            vcap.bar = pci_cfg_read8(gpu->bus, gpu->device, gpu->func, (uint8_t)(cap + 4));
            vcap.id = pci_cfg_read8(gpu->bus, gpu->device, gpu->func, (uint8_t)(cap + 5));
            vcap.offset = pci_cfg_read32(gpu->bus, gpu->device, gpu->func, (uint8_t)(cap + 8));
            vcap.length = pci_cfg_read32(gpu->bus, gpu->device, gpu->func, (uint8_t)(cap + 12));

            if (vcap.bar < 6 && gpu->bar_is_mem[vcap.bar] && gpu->bar_addr[vcap.bar] != 0) {
                uint64_t phys = gpu->bar_addr[vcap.bar] + (uint64_t)vcap.offset;
                volatile uint8_t *base = (volatile uint8_t *)map_mmio_virt(phys);
                if (!base) {
                    return 0;
                }

                if (vcap.cfg_type == VIRTIO_PCI_CAP_COMMON_CFG) {
                    t->common_cfg = base;
                } else if (vcap.cfg_type == VIRTIO_PCI_CAP_NOTIFY_CFG) {
                    t->notify_base = base;
                    t->notify_off_multiplier = pci_cfg_read32(gpu->bus, gpu->device, gpu->func, (uint8_t)(cap + 16));
                } else if (vcap.cfg_type == VIRTIO_PCI_CAP_DEVICE_CFG) {
                    t->device_cfg = base;
                }
            }
        }

        cap = cap_next;
    }

    return (t->common_cfg != NULL && t->notify_base != NULL) ? 1 : 0;
}

static inline uint8_t common_read8(volatile uint8_t *common, uint32_t off) {
    return *(volatile uint8_t *)(common + off);
}

static inline uint16_t common_read16(volatile uint8_t *common, uint32_t off) {
    return *(volatile uint16_t *)(common + off);
}

static inline uint32_t common_read32(volatile uint8_t *common, uint32_t off) {
    return *(volatile uint32_t *)(common + off);
}

static inline void common_write8(volatile uint8_t *common, uint32_t off, uint8_t v) {
    *(volatile uint8_t *)(common + off) = v;
}

static inline void common_write16(volatile uint8_t *common, uint32_t off, uint16_t v) {
    *(volatile uint16_t *)(common + off) = v;
}

static inline void common_write32(volatile uint8_t *common, uint32_t off, uint32_t v) {
    *(volatile uint32_t *)(common + off) = v;
}

static inline void common_write64(volatile uint8_t *common, uint32_t off, uint64_t v) {
    *(volatile uint64_t *)(common + off) = v;
}

static int virtio_pci_device_init(const virtio_gpu_pci_t *gpu, virtio_pci_transport_t *t) {
    uint16_t cmd = pci_cfg_read16(gpu->bus, gpu->device, gpu->func, 0x04);
    cmd |= (1u << 1);
    cmd |= (1u << 2);
    pci_cfg_write16(gpu->bus, gpu->device, gpu->func, 0x04, cmd);

    common_write8(t->common_cfg, 20, 0);
    common_write8(t->common_cfg, 20, VIRTIO_STATUS_ACKNOWLEDGE);
    common_write8(t->common_cfg, 20, VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER);

    common_write32(t->common_cfg, 0, 0);
    uint32_t device_features0 = common_read32(t->common_cfg, 4);
    common_write32(t->common_cfg, 0, 1);
    uint32_t device_features1 = common_read32(t->common_cfg, 4);
    if ((device_features1 & VIRTIO_F_VERSION_1) == 0u) {
        return 0;
    }

    g_gpu_features0 = 0u;
    if ((device_features0 & VIRTIO_GPU_F_EDID) != 0u) {
        g_gpu_features0 |= VIRTIO_GPU_F_EDID;
    }
    if ((device_features0 & VIRTIO_GPU_F_VIRGL) != 0u) {
        g_gpu_features0 |= VIRTIO_GPU_F_VIRGL;
    }
    if ((device_features0 & VIRTIO_GPU_F_RESOURCE_BLOB) != 0u) {
        g_gpu_features0 |= VIRTIO_GPU_F_RESOURCE_BLOB;
    }
    if ((device_features0 & VIRTIO_GPU_F_CONTEXT_INIT) != 0u) {
        g_gpu_features0 |= VIRTIO_GPU_F_CONTEXT_INIT;
    }

#ifdef IMPLUS_DRIVER_MODULE
    /* Record what the device actually offered. A 2D-ready VirtIO GPU may
     * still lack VIRGL when QEMU was started without a GL-capable device. */
    vlog("[vgpu] features dev0=");
    vlog_hex(device_features0);
    vlog(" dev1=");
    vlog_hex(device_features1);
    vlog(" negotiated=");
    vlog_hex(g_gpu_features0);
    vlog("\n");
#endif

    common_write32(t->common_cfg, 8, 0);
    common_write32(t->common_cfg, 12, g_gpu_features0);
    common_write32(t->common_cfg, 8, 1);
    common_write32(t->common_cfg, 12, VIRTIO_F_VERSION_1);

    uint8_t status = (uint8_t)(VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEATURES_OK);
    common_write8(t->common_cfg, 20, status);
    status = common_read8(t->common_cfg, 20);
    if ((status & VIRTIO_STATUS_FEATURES_OK) == 0) {
        common_write8(t->common_cfg, 20, status | VIRTIO_STATUS_FAILED);
        return 0;
    }

    return 1;
}

/* VirtIO descriptors contain bus/physical addresses, never kernel virtual
 * addresses. Use the module DMA pool so each region is physically contiguous. */
static void *gpu_dma_alloc(size_t bytes, size_t alignment, uint64_t *phys_out) {
#ifdef IMPLUS_DRIVER_MODULE
    if (g_driver_api == NULL || g_driver_api->mem.dma_alloc_ex == NULL) {
        return NULL;
    }
    return g_driver_api->mem.dma_alloc_ex(bytes, alignment, 0u, phys_out);
#else
    (void)alignment;
    void *ptr = malloc(bytes);
    if (phys_out != NULL) *phys_out = (uint64_t)(uintptr_t)ptr;
    return ptr;
#endif
}

static void gpu_dma_free(void *ptr, size_t bytes) {
    if (ptr == NULL) return;
#ifdef IMPLUS_DRIVER_MODULE
    if (g_driver_api != NULL && g_driver_api->mem.dma_free != NULL) {
        g_driver_api->mem.dma_free(ptr, bytes);
    }
#else
    (void)bytes;
    free(ptr);
#endif
}

static int virtqueue_init_ctrl(virtio_pci_transport_t *t, virtqueue_t *vq, uint16_t queue_index) {
    memset(vq, 0, sizeof(*vq));
    common_write16(t->common_cfg, 22, queue_index);
    uint16_t qsize = common_read16(t->common_cfg, 24);
    if (qsize < 2) {
        return 0;
    }

    uint32_t desc_bytes = (uint32_t)qsize * (uint32_t)sizeof(virtq_desc_t);
    uint32_t avail_bytes = 6u + (uint32_t)qsize * 2u;
    uint32_t used_off = align_up_u32(desc_bytes + avail_bytes, 4u);
    uint32_t used_bytes = 6u + (uint32_t)qsize * 8u;
    uint32_t total = used_off + used_bytes;

    uint64_t ring_phys = 0u;
    uint8_t *ring = (uint8_t *)gpu_dma_alloc(total, 4096u, &ring_phys);
    if (ring == NULL || ring_phys == 0u) {
        if (ring != NULL) gpu_dma_free(ring, total);
        return 0;
    }

    vq->queue_index = queue_index;
    vq->queue_size = qsize;
    vq->desc = (volatile virtq_desc_t *)ring;
    vq->avail = (volatile virtq_avail_t *)(ring + desc_bytes);
    vq->used = (volatile virtq_used_t *)(ring + used_off);
    vq->raw_ring = ring;
    vq->raw_ring_bytes = total;
    vq->ring_phys = ring_phys;

    vq->resp_buf = (uint8_t *)gpu_dma_alloc(VIRTIO_GPU_RESP_BUF_BYTES,
                                             4096u, &vq->resp_phys);
    if (vq->resp_buf == NULL || vq->resp_phys == 0u) {
        gpu_dma_free(vq->resp_buf, VIRTIO_GPU_RESP_BUF_BYTES);
        gpu_dma_free(vq->raw_ring, vq->raw_ring_bytes);
        memset(vq, 0, sizeof(*vq));
        return 0;
    }
    vq->resp_cap = VIRTIO_GPU_RESP_BUF_BYTES;

    vq->cmd_buf = (uint8_t *)gpu_dma_alloc(VIRTIO_GPU_CMD_BUF_BYTES,
                                            4096u, &vq->cmd_phys);
    if (vq->cmd_buf == NULL || vq->cmd_phys == 0u) {
        gpu_dma_free(vq->cmd_buf, VIRTIO_GPU_CMD_BUF_BYTES);
        gpu_dma_free(vq->resp_buf, VIRTIO_GPU_RESP_BUF_BYTES);
        gpu_dma_free(vq->raw_ring, vq->raw_ring_bytes);
        memset(vq, 0, sizeof(*vq));
        return 0;
    }
    vq->cmd_cap = VIRTIO_GPU_CMD_BUF_BYTES;

    common_write16(t->common_cfg, 26, 0xFFFFu);
    common_write64(t->common_cfg, 32, vq->ring_phys);
    common_write64(t->common_cfg, 40, vq->ring_phys + desc_bytes);
    common_write64(t->common_cfg, 48, vq->ring_phys + used_off);
    common_write16(t->common_cfg, 28, 1);

    uint16_t notify_off = common_read16(t->common_cfg, 30);
    vq->notify_addr = (volatile uint16_t *)(t->notify_base + ((uint32_t)notify_off * t->notify_off_multiplier));

    common_write16(t->common_cfg, 22, queue_index);
    if (common_read16(t->common_cfg, 28) == 0) {
        gpu_dma_free(vq->cmd_buf, VIRTIO_GPU_CMD_BUF_BYTES);
        gpu_dma_free(vq->resp_buf, VIRTIO_GPU_RESP_BUF_BYTES);
        gpu_dma_free(vq->raw_ring, vq->raw_ring_bytes);
        memset(vq, 0, sizeof(*vq));
        return 0;
    }

    return 1;
}

/*
 * Wait for the used ring to advance past `prev_used`. Returns false when the
 * device did not respond within VIRTIO_GPU_CMD_TIMEOUT_MS.
 */
static int virtqueue_wait_used(virtqueue_t *vq, uint16_t prev_used)
{
    uint64_t start_ms = gpu_now_ms();
    uint32_t spins = 0u;

    while ((uint16_t)(vq->used->idx - prev_used) == 0u) {
        memory_barrier();
        if (start_ms != 0ull) {
            if (gpu_now_ms() - start_ms >= VIRTIO_GPU_CMD_TIMEOUT_MS) {
                return 0;
            }
        } else if (++spins >= VIRTIO_GPU_SPIN_BUDGET) {
            return 0;
        }
    }
    return 1;
}

/*
 * Submit one command on the control queue and wait for its completion.
 *
 * The caller must hold g_gpu_lock. The device always writes its response into
 * vq->resp_buf, never into the caller's buffer, so a request that completes
 * late cannot corrupt a stack frame that has already been reused.
 */
static int virtqueue_submit_sync(virtqueue_t *vq, void *cmd, uint32_t cmd_len,
                                 void *resp, uint32_t resp_len) {
    if (vq->queue_size < 2 || cmd == NULL || resp == NULL ||
        vq->cmd_buf == NULL || vq->resp_buf == NULL ||
        cmd_len == 0u || cmd_len > vq->cmd_cap ||
        vq->cmd_phys == 0u || vq->resp_phys == 0u ||
        resp_len > vq->resp_cap) {
        return 0;
    }

    if (g_vlog_budget != 0u) {
        vlog("[vgpu] submit t=");
        vlog_num(cmd != NULL ? (uint64_t)(*(const uint32_t *)cmd) : 0u);
        vlog(" avail=");
        vlog_num(vq->avail_idx);
        vlog(" used=");
        vlog_num(vq->used->idx);
        vlog(" resp=");
        vlog_hex((uint64_t)(uintptr_t)resp);
        vlog("\n");
        vlog_done();
    }

    /* Retire a completion we never consumed (a previous request timed out).
     * The descriptors must not be rewritten while the device may still be
     * reading them. */
    if ((uint16_t)(vq->used->idx - vq->used_idx_seen) != 0u) {
        if (!virtqueue_wait_used(vq, vq->used_idx_seen)) {
            return 0;
        }
    }
    uint16_t prev_used = vq->used->idx;
    vq->used_idx_seen = prev_used;

    memset(vq->resp_buf, 0, resp_len);
    memcpy(vq->cmd_buf, cmd, cmd_len);
    memory_barrier();

    vq->desc[0].addr = vq->cmd_phys;
    vq->desc[0].len = cmd_len;
    vq->desc[0].flags = VIRTQ_DESC_F_NEXT;
    vq->desc[0].next = 1;

    vq->desc[1].addr = vq->resp_phys;
    vq->desc[1].len = resp_len;
    vq->desc[1].flags = VIRTQ_DESC_F_WRITE;
    vq->desc[1].next = 0;

    vq->avail->ring[vq->avail_idx % vq->queue_size] = 0;
    memory_barrier();
    vq->avail_idx++;
    vq->avail->idx = vq->avail_idx;
    memory_barrier();

    *vq->notify_addr = vq->queue_index;

    if (!virtqueue_wait_used(vq, prev_used)) {
        vlog("[vgpu] submit TIMEOUT t=");
        vlog_num(cmd != NULL ? (uint64_t)(*(const uint32_t *)cmd) : 0u);
        vlog(" avail=");
        vlog_num(vq->avail_idx);
        vlog(" used=");
        vlog_num(vq->used->idx);
        vlog("\n");
        vlog_done();
        /* Leave used_idx_seen behind: the next submit drains it first. The
         * response buffer stays owned by the queue, so a late completion is
         * harmless. */
        return 0;
    }

    /* Only consume the entry that belongs to this request (descriptor 0). */
    uint16_t slot = (uint16_t)(prev_used % vq->queue_size);
    if ((uint16_t)vq->used->ring[slot].id != 0u) {
        vq->used_idx_seen = vq->used->idx;
        return 0;
    }

    vq->used_idx_seen = vq->used->idx;
    memcpy(resp, vq->resp_buf, resp_len);
    return 1;
}

static int gpu_cmd_get_display_info(virtqueue_t *vq,
                                    virtio_gpu_resp_display_info_t *out_info) {
    virtio_gpu_ctrl_hdr_t cmd;
    virtio_gpu_resp_display_info_t resp;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    if (resp.hdr.type != VIRTIO_GPU_RESP_OK_DISPLAY_INFO) {
        return 0;
    }

    if (out_info != NULL) {
        memcpy(out_info, &resp, sizeof(resp));
    }
    return 1;
}

static int gpu_cmd_get_edid(virtqueue_t *vq, uint32_t scanout,
                            virtio_gpu_resp_edid_t *out_edid) {
    if ((g_gpu_features0 & VIRTIO_GPU_F_EDID) == 0u || out_edid == NULL) {
        return 0;
    }

    virtio_gpu_get_edid_t cmd;
    virtio_gpu_resp_edid_t resp;
    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_GET_EDID;
    cmd.scanout = scanout;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    if (resp.hdr.type != VIRTIO_GPU_RESP_OK_EDID || resp.size == 0u) {
        return 0;
    }

    memcpy(out_edid, &resp, sizeof(resp));
    return 1;
}

static int gpu_cmd_resource_create_2d(virtqueue_t *vq, uint32_t resource_id,
                                      uint32_t width, uint32_t height) {
    virtio_gpu_resource_create_2d_t cmd;
    virtio_gpu_resp_nodata_t resp;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
    cmd.resource_id = resource_id;
    cmd.format = VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM;
    cmd.width = width;
    cmd.height = height;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_resource_unref(virtqueue_t *vq, uint32_t resource_id) {
    if (resource_id == 0u) {
        return 1;
    }

    virtio_gpu_resource_unref_t cmd;
    virtio_gpu_resp_nodata_t resp;
    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_UNREF;
    cmd.resource_id = resource_id;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_resource_attach_backing(virtqueue_t *vq, uint32_t resource_id,
                                           void *fb, uint32_t bytes) {
    virtio_gpu_resource_attach_backing_t cmd;
    virtio_gpu_resp_nodata_t resp;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;
    cmd.resource_id = resource_id;
    cmd.nr_entries = 1;
    cmd.entry.addr = (uint64_t)(uintptr_t)fb;
    cmd.entry.length = bytes;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_resource_detach_backing(virtqueue_t *vq, uint32_t resource_id) {
    if (resource_id == 0u) {
        return 1;
    }

    virtio_gpu_resource_detach_backing_t cmd;
    virtio_gpu_resp_nodata_t resp;
    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING;
    cmd.resource_id = resource_id;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_set_scanout(virtqueue_t *vq, uint32_t scanout_id,
                               uint32_t resource_id,
                               uint32_t x, uint32_t y,
                               uint32_t width, uint32_t height) {
    virtio_gpu_set_scanout_t cmd;
    virtio_gpu_resp_nodata_t resp;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
    cmd.rect.x = x;
    cmd.rect.y = y;
    cmd.rect.width = width;
    cmd.rect.height = height;
    cmd.scanout_id = scanout_id;
    cmd.resource_id = resource_id;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

/* The 2D transfer unit is one rectangle of the resource. offset is the byte
 * offset of that rectangle's first pixel in the backing store; QEMU uses the
 * resource stride (width * 4) to advance between rows, so a partial update
 * must name its top-left pixel rather than assume offset 0. */
static int gpu_cmd_transfer_to_host_2d(virtqueue_t *vq, uint32_t resource_id,
                                       uint32_t x, uint32_t y,
                                       uint32_t width, uint32_t height,
                                       uint64_t offset) {
    virtio_gpu_transfer_to_host_2d_t cmd;
    virtio_gpu_resp_nodata_t resp;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
    cmd.rect.x = x;
    cmd.rect.y = y;
    cmd.rect.width = width;
    cmd.rect.height = height;
    cmd.offset = offset;
    cmd.resource_id = resource_id;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_resource_flush(virtqueue_t *vq, uint32_t resource_id,
                                  uint32_t x, uint32_t y,
                                  uint32_t width, uint32_t height) {
    virtio_gpu_resource_flush_t cmd;
    virtio_gpu_resp_nodata_t resp;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
    cmd.rect.x = x;
    cmd.rect.y = y;
    cmd.rect.width = width;
    cmd.rect.height = height;
    cmd.resource_id = resource_id;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

/* ---- 3D (virgl) commands ---- */

static int gpu_cmd_get_capset_info(virtqueue_t *vq, uint32_t index,
                                   virtio_gpu_resp_capset_info_t *out_info) {
    virtio_gpu_get_capset_info_t cmd;
    virtio_gpu_resp_capset_info_t resp;

    if (out_info == NULL) return 0;
    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_GET_CAPSET_INFO;
    cmd.capset_index = index;
    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd),
                               &resp, sizeof(resp))) {
        return 0;
    }
    if (resp.hdr.type != VIRTIO_GPU_RESP_OK_CAPSET_INFO ||
        resp.capset_id == 0u || resp.max_size == 0u) {
        return 0;
    }
    *out_info = resp;
    return 1;
}

static int gpu_cmd_get_capset(virtqueue_t *vq, uint32_t capset_id,
                              uint32_t capset_version, void *out_caps,
                              uint32_t out_size) {
    virtio_gpu_get_capset_t cmd;
    uint8_t *response = NULL;
    virtio_gpu_resp_capset_info_t info;
    uint32_t index;
    uint32_t response_size;
    int found = 0;
    int ok = 0;

    if (out_caps == NULL || out_size == 0u || capset_id == 0u) return 0;
    memset(&info, 0, sizeof(info));
    for (index = 0u; index < VIRTIO_GPU_MAX_CAPSETS; ++index) {
        virtio_gpu_resp_capset_info_t candidate;
        memset(&candidate, 0, sizeof(candidate));
        if (!gpu_cmd_get_capset_info(vq, index, &candidate)) break;
        if (candidate.capset_id == capset_id &&
            capset_version <= candidate.max_version) {
            info = candidate;
            found = 1;
            break;
        }
    }
    if (!found || info.max_size > VIRTIO_GPU_RESP_BUF_BYTES -
                                      (uint32_t)sizeof(virtio_gpu_ctrl_hdr_t)) {
        return 0;
    }

    response_size = (uint32_t)sizeof(virtio_gpu_ctrl_hdr_t) + info.max_size;
    response = (uint8_t *)malloc(response_size);
    if (response == NULL) return 0;

    memset(&cmd, 0, sizeof(cmd));
    cmd.hdr.type = VIRTIO_GPU_CMD_GET_CAPSET;
    cmd.capset_id = capset_id;
    cmd.capset_version = capset_version;
    memset(response, 0, response_size);
    if (virtqueue_submit_sync(vq, &cmd, sizeof(cmd),
                              response, response_size) &&
        ((virtio_gpu_ctrl_hdr_t *)response)->type ==
            VIRTIO_GPU_RESP_OK_CAPSET) {
        uint32_t copy_size = info.max_size;
        if (copy_size > out_size) copy_size = out_size;
        memcpy(out_caps, response + sizeof(virtio_gpu_ctrl_hdr_t), copy_size);
        ok = 1;
    }
    free(response);
    return ok;
}

static int gpu_cmd_ctx_create(virtqueue_t *vq, uint32_t context_id,
                              uint32_t context_init, const char *name) {
    virtio_gpu_ctx_create_t cmd;
    virtio_gpu_resp_nodata_t resp;

    if (context_id == 0u) return 0;
    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_CTX_CREATE;
    cmd.hdr.ctx_id = context_id;

    uint32_t name_len = 0u;
    if (name != NULL) {
        while (name_len < sizeof(cmd.debug_name) - 1u &&
               name[name_len] != '\0') {
            cmd.debug_name[name_len] = name[name_len];
            ++name_len;
        }
    }
    cmd.nlen = name_len;
    /* CONTEXT_INIT carries the capset id in its low byte. Older devices
     * implement the original virgl context-create request and require this
     * field to remain zero. */
    if ((g_gpu_features0 & VIRTIO_GPU_F_CONTEXT_INIT) != 0u) {
        cmd.context_init = context_init & 0xFFu;
    }

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_ctx_destroy(virtqueue_t *vq, uint32_t context_id) {
    virtio_gpu_ctx_destroy_t cmd;
    virtio_gpu_resp_nodata_t resp;

    if (context_id == 0u) return 0;
    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_CTX_DESTROY;
    cmd.hdr.ctx_id = context_id;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_submit_3d(virtqueue_t *vq, uint32_t context_id,
                             const void *blob, uint32_t blob_size) {
    /* Virgl command streams are DWORD streams. The protocol size is the
     * unpadded byte count, and the payload is DWORD-aligned. */
    if (context_id == 0u || blob == NULL || blob_size == 0u ||
        (blob_size & 3u) != 0u ||
        blob_size > UINT32_MAX - (uint32_t)sizeof(virtio_gpu_submit_3d_t)) {
        return 0;
    }

    uint32_t total = (uint32_t)sizeof(virtio_gpu_submit_3d_t) + blob_size;
    uint8_t *cmd = (uint8_t *)malloc(total);
    if (cmd == NULL) return 0;

    memset(cmd, 0, total);
    virtio_gpu_submit_3d_t *hdr = (virtio_gpu_submit_3d_t *)cmd;
    hdr->hdr.type = VIRTIO_GPU_CMD_SUBMIT_3D;
    hdr->hdr.ctx_id = context_id;
    hdr->size = blob_size;
    memcpy(cmd + sizeof(virtio_gpu_submit_3d_t), blob, blob_size);

    virtio_gpu_resp_nodata_t resp;
    memset(&resp, 0, sizeof(resp));
    int ok = virtqueue_submit_sync(vq, cmd, total, &resp, sizeof(resp));
    free(cmd);
    if (!ok) return 0;
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_resource_create_3d(virtqueue_t *vq, uint32_t resource_id,
                                      uint32_t target, uint32_t format,
                                      uint32_t bind, uint32_t width,
                                      uint32_t height, uint32_t depth,
                                      uint32_t array_size,
                                      uint32_t last_level,
                                      uint32_t nr_samples,
                                      uint32_t flags) {
    virtio_gpu_resource_create_3d_t cmd;
    virtio_gpu_resp_nodata_t resp;

    if (resource_id == 0u || width == 0u || height == 0u ||
        depth == 0u || target == 0u) return 0;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_3D;
    cmd.resource_id = resource_id;
    cmd.target = target;
    cmd.format = format;
    cmd.bind = bind;
    cmd.width = width;
    cmd.height = height;
    cmd.depth = depth;
    cmd.array_size = array_size;
    cmd.last_level = last_level;
    cmd.nr_samples = nr_samples;
    cmd.flags = flags;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_transfer_3d(virtqueue_t *vq, uint32_t command,
                               uint32_t context_id, uint32_t resource_id,
                               uint32_t x, uint32_t y, uint32_t z,
                               uint32_t w, uint32_t h, uint32_t d,
                               uint64_t offset, uint32_t level,
                               uint32_t stride, uint32_t layer_stride) {
    virtio_gpu_transfer_3d_t cmd;
    virtio_gpu_resp_nodata_t resp;

    if (context_id == 0u || resource_id == 0u ||
        (command != VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D &&
         command != VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D)) return 0;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = command;
    cmd.hdr.ctx_id = context_id;
    cmd.box.x = x;
    cmd.box.y = y;
    cmd.box.z = z;
    cmd.box.w = w;
    cmd.box.h = h;
    cmd.box.d = d;
    cmd.offset = offset;
    cmd.resource_id = resource_id;
    cmd.level = level;
    cmd.stride = stride;
    cmd.layer_stride = layer_stride;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static int gpu_cmd_ctx_resource(virtqueue_t *vq, uint32_t command,
                                uint32_t context_id, uint32_t resource_id) {
    virtio_gpu_ctx_resource_t cmd;
    virtio_gpu_resp_nodata_t resp;

    if (context_id == 0u || resource_id == 0u ||
        (command != VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE &&
         command != VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE)) return 0;

    memset(&cmd, 0, sizeof(cmd));
    memset(&resp, 0, sizeof(resp));
    cmd.hdr.type = command;
    cmd.hdr.ctx_id = context_id;
    cmd.resource_id = resource_id;

    if (!virtqueue_submit_sync(vq, &cmd, sizeof(cmd), &resp, sizeof(resp))) {
        return 0;
    }
    return resp.hdr.type == VIRTIO_GPU_RESP_OK_NODATA;
}

static void virtio_gpu_copy_string(char *dst, uint32_t dst_size, const char *src) {
    if (dst == NULL || dst_size == 0u) {
        return;
    }
    uint32_t i = 0u;
    if (src != NULL) {
        while (i + 1u < dst_size && src[i] != '\0') {
            dst[i] = src[i];
            ++i;
        }
    }
    dst[i] = '\0';
}

static void virtio_gpu_publish_monitor_event(const virtio_gpu_monitor_t *monitor,
                                             uint32_t monitor_index,
                                             uint16_t action) {
#ifdef IMPLUS_DRIVER_MODULE
    if (monitor == NULL || g_driver_api == NULL ||
        g_driver_api->pnp_notify == NULL) {
        return;
    }

    pnp_event_t event;
    const char *detail = "monitor configured";
    if (action == PNP_EVENT_DEVICE_REMOVED) {
        detail = "monitor disconnected";
    } else if (action == PNP_EVENT_DEVICE_CHANGED) {
        detail = "monitor configuration changed";
    }

    pnp_event_init(&event,
                   action,
                   PNP_BUS_DISPLAY,
                   PNP_CLASS_MONITOR,
                   "VirtIO_Driver.ELF",
                   "VirtIO GPU Scanout",
                   detail);
    event.location0 = ((monitor->scanout_id & 0xFFFFu) << 16u) |
                      (monitor_index & 0xFFFFu);
    event.location1 = ((monitor->width & 0xFFFFu) << 16u) |
                      (monitor->height & 0xFFFFu);
    g_driver_api->pnp_notify(&event);
#else
    (void)monitor;
    (void)monitor_index;
    (void)action;
#endif
}

static uint32_t virtio_gpu_next_resource_id(void) {
    uint32_t id = g_gpu_next_resource_id++;
    if (g_gpu_next_resource_id == 0u) {
        g_gpu_next_resource_id = 1u;
    }
    return id == 0u ? virtio_gpu_next_resource_id() : id;
}

static bool virtio_gpu_mode_exists(const virtio_gpu_monitor_t *monitor,
                                   uint32_t width, uint32_t height) {
    for (uint32_t i = 0u; i < monitor->mode_count; ++i) {
        if (monitor->modes[i].width == width &&
            monitor->modes[i].height == height) {
            return true;
        }
    }
    return false;
}

static void virtio_gpu_add_mode(virtio_gpu_monitor_t *monitor,
                                uint32_t width,
                                uint32_t height,
                                uint32_t refresh_millihz,
                                uint32_t flags,
                                const char *name) {
    if (monitor == NULL || width == 0u || height == 0u ||
        monitor->mode_count >= VIRTIO_GPU_MAX_MODES ||
        virtio_gpu_mode_exists(monitor, width, height)) {
        return;
    }

    uint32_t index = monitor->mode_count++;
    display_mode_info_t *mode = &monitor->modes[index];
    memset(mode, 0, sizeof(*mode));
    mode->monitor_index = 0u;
    mode->mode_index = index;
    mode->flags = flags;
    mode->width = width;
    mode->height = height;
    mode->stride = width;
    mode->bits_per_pixel = 32u;
    mode->refresh_millihz = refresh_millihz != 0u ? refresh_millihz : 60000u;
    virtio_gpu_copy_string(mode->name, sizeof(mode->name), name);
}

static uint32_t virtio_gpu_edid_u16(const uint8_t *edid, uint32_t offset) {
    return (uint32_t)edid[offset] | ((uint32_t)edid[offset + 1u] << 8u);
}

static void virtio_gpu_apply_edid(virtio_gpu_monitor_t *monitor,
                                  const virtio_gpu_resp_edid_t *edid_resp) {
    if (monitor == NULL || edid_resp == NULL || edid_resp->size < 128u) {
        return;
    }

    const uint8_t *edid = edid_resp->edid;
    if (edid[21] != 0u) {
        monitor->physical_width_mm = (uint32_t)edid[21] * 10u;
    }
    if (edid[22] != 0u) {
        monitor->physical_height_mm = (uint32_t)edid[22] * 10u;
    }

    uint32_t pixel_clock = virtio_gpu_edid_u16(edid, 54u);
    if (pixel_clock == 0u) {
        return;
    }

    uint32_t h_active = (uint32_t)edid[56] |
        (((uint32_t)edid[58] & 0xF0u) << 4u);
    uint32_t h_blank = (uint32_t)edid[57] |
        (((uint32_t)edid[58] & 0x0Fu) << 8u);
    uint32_t v_active = (uint32_t)edid[59] |
        (((uint32_t)edid[61] & 0xF0u) << 4u);
    uint32_t v_blank = (uint32_t)edid[60] |
        (((uint32_t)edid[61] & 0x0Fu) << 8u);
    uint32_t h_total = h_active + h_blank;
    uint32_t v_total = v_active + v_blank;
    uint32_t refresh = 60000u;
    if (h_total != 0u && v_total != 0u) {
        uint64_t pixel_clock_hz = (uint64_t)pixel_clock * 10000ull;
        refresh = (uint32_t)((pixel_clock_hz * 1000ull) /
                             ((uint64_t)h_total * (uint64_t)v_total));
    }
    if (h_active != 0u && v_active != 0u) {
        virtio_gpu_add_mode(monitor, h_active, v_active, refresh,
                            DISPLAY_MODE_FLAG_PREFERRED, "EDID preferred");
    }
}

static void virtio_gpu_build_modes(virtio_gpu_monitor_t *monitor,
                                   const virtio_gpu_resp_edid_t *edid_resp) {
    if (monitor == NULL) {
        return;
    }

    monitor->mode_count = 0u;
    monitor->current_mode = 0u;
    virtio_gpu_add_mode(monitor, monitor->width, monitor->height,
                        monitor->refresh_millihz,
                        DISPLAY_MODE_FLAG_CURRENT |
                        DISPLAY_MODE_FLAG_PREFERRED,
                        "Current");
    virtio_gpu_apply_edid(monitor, edid_resp);

    static const uint32_t fallback_modes[][2] = {
        {1920u, 1080u}, {1600u, 900u}, {1280u, 1024u},
        {1280u, 720u},  {1024u, 768u}, {800u, 600u},
    };
    for (uint32_t i = 0u;
         i < sizeof(fallback_modes) / sizeof(fallback_modes[0]);
         ++i) {
        virtio_gpu_add_mode(monitor,
                            fallback_modes[i][0],
                            fallback_modes[i][1],
                            60000u,
                            DISPLAY_MODE_FLAG_SYNTHETIC,
                            "Synthetic");
    }
}

static void virtio_gpu_fix_mode_metadata(virtio_gpu_monitor_t *monitor,
                                         uint32_t monitor_index) {
    if (monitor == NULL) {
        return;
    }
    for (uint32_t i = 0u; i < monitor->mode_count; ++i) {
        monitor->modes[i].monitor_index = monitor_index;
        monitor->modes[i].mode_index = i;
    }
}

static void virtio_gpu_release_monitor_resource(virtio_gpu_monitor_t *monitor) {
    if (monitor == NULL) {
        return;
    }
    if (monitor->resource_id != 0u && g_gpu_controlq.queue_size != 0u) {
        (void)gpu_cmd_resource_detach_backing(&g_gpu_controlq,
                                              monitor->resource_id);
        (void)gpu_cmd_resource_unref(&g_gpu_controlq, monitor->resource_id);
    }
    if (monitor->fb != NULL && !monitor->fb_shared) {
        free(monitor->fb);
    }
    monitor->resource_id = 0u;
    monitor->fb = NULL;
    monitor->fb_bytes = 0u;
    monitor->fb_shared = false;
}

/* Create the 2D resource a scanout samples. When the monitor covers the whole
 * desktop (the single-monitor case), shared_fb is the desktop framebuffer
 * itself and is attached directly: the compositor then draws straight into
 * the memory the device reads, so present() only has to transfer damage
 * instead of copying the entire desktop into a second monitor-sized buffer. */
static bool virtio_gpu_create_monitor_resource(virtio_gpu_monitor_t *monitor,
                                               uint32_t *shared_fb,
                                               uint32_t shared_width,
                                               uint32_t shared_bytes) {
    if (monitor == NULL || monitor->width == 0u || monitor->height == 0u) {
        return false;
    }
    uint64_t pixel_count = (uint64_t)monitor->width * (uint64_t)monitor->height;
    if (pixel_count == 0u || pixel_count > VIRTIO_GPU_MAX_PIXELS ||
        pixel_count > 0xFFFFFFFFull / sizeof(uint32_t)) {
        return false;
    }

    uint32_t fb_bytes = (uint32_t)(pixel_count * sizeof(uint32_t));
    bool shared = shared_fb != NULL &&
                  monitor->x == 0 && monitor->y == 0 &&
                  monitor->width == shared_width &&
                  shared_bytes >= fb_bytes;
    uint32_t *fb = shared ? shared_fb : (uint32_t *)malloc(fb_bytes);
    if (fb == NULL) {
        return false;
    }
    if (!shared) {
        memset(fb, 0, fb_bytes);
    }

    uint32_t resource_id = virtio_gpu_next_resource_id();
    if (!gpu_cmd_resource_create_2d(&g_gpu_controlq, resource_id,
                                    monitor->width, monitor->height) ||
        !gpu_cmd_resource_attach_backing(&g_gpu_controlq, resource_id,
                                         fb, fb_bytes) ||
        !gpu_cmd_set_scanout(&g_gpu_controlq, monitor->scanout_id,
                             resource_id, 0u, 0u,
                             monitor->width, monitor->height)) {
        (void)gpu_cmd_resource_detach_backing(&g_gpu_controlq, resource_id);
        (void)gpu_cmd_resource_unref(&g_gpu_controlq, resource_id);
        if (!shared) {
            free(fb);
        }
        return false;
    }

    monitor->resource_id = resource_id;
    monitor->fb = fb;
    monitor->fb_bytes = fb_bytes;
    monitor->fb_shared = shared;
    return true;
}

/* ---- desktop framebuffer lifetime ----
 *
 * SYSCALL_GET_DISPLAY_FRAMEBUFFER maps these very pages into the calling
 * process, so the buffer must not be freed or moved while a compositor is
 * drawing into it. A layout change therefore reuses the existing allocation
 * whenever it is large enough, and otherwise *retires* the old buffer instead
 * of freeing it: a stale user mapping then writes into dead memory rather than
 * into a kernel heap block that has since been handed to something else. */
static uint32_t *g_gpu_retired_fb[VIRTIO_GPU_RETIRED_FB_MAX];
static uint32_t g_gpu_retired_fb_next = 0u;

static void virtio_gpu_retire_framebuffer(uint32_t *fb)
{
    if (fb == NULL) {
        return;
    }
    uint32_t slot = g_gpu_retired_fb_next % VIRTIO_GPU_RETIRED_FB_MAX;
    if (g_gpu_retired_fb[slot] != NULL) {
        free(g_gpu_retired_fb[slot]);
    }
    g_gpu_retired_fb[slot] = fb;
    ++g_gpu_retired_fb_next;
}

static void virtio_gpu_release_resources(void) {
    for (uint32_t i = 0u; i < g_gpu_monitor_count; ++i) {
        virtio_gpu_release_monitor_resource(&g_gpu_monitors[i]);
    }
    g_gpu_monitor_count = 0u;
    virtio_gpu_retire_framebuffer(g_gpu_fb);
    g_gpu_fb = NULL;
    g_gpu_fb_bytes = 0u;
    g_gpu_fb_capacity = 0u;
    g_gpu_width = 0u;
    g_gpu_height = 0u;
}

static int32_t virtio_gpu_find_scanout(const virtio_gpu_monitor_t *monitors,
                                       uint32_t monitor_count,
                                       uint32_t scanout_id) {
    if (monitors == NULL) {
        return -1;
    }
    for (uint32_t i = 0u; i < monitor_count; ++i) {
        if (monitors[i].scanout_id == scanout_id) {
            return (int32_t)i;
        }
    }
    return -1;
}

static bool virtio_gpu_apply_layout(virtio_gpu_monitor_t *monitors,
                                    uint32_t monitor_count,
                                    int32_t origin_x,
                                    int32_t origin_y,
                                    uint32_t desktop_width,
                                    uint32_t desktop_height) {
    if (monitors == NULL || monitor_count == 0u ||
        monitor_count > VIRTIO_GPU_MAX_SCANOUTS ||
        desktop_width == 0u || desktop_height == 0u) {
        return false;
    }

    uint64_t desktop_pixels = (uint64_t)desktop_width * (uint64_t)desktop_height;
    if (desktop_pixels == 0u || desktop_pixels > VIRTIO_GPU_MAX_PIXELS ||
        desktop_pixels > 0xFFFFFFFFull / sizeof(uint32_t)) {
        return false;
    }

    uint32_t virtual_bytes = (uint32_t)(desktop_pixels * sizeof(uint32_t));

    /* Reuse the live framebuffer whenever it is large enough: user space maps
     * these pages directly, so a mode change must not move them. */
    bool fb_reused = (g_gpu_fb != NULL && g_gpu_fb_capacity >= virtual_bytes);
    uint32_t prev_capacity = g_gpu_fb_capacity;
    uint32_t *virtual_fb = g_gpu_fb;
    if (!fb_reused) {
        uint32_t capacity = virtual_bytes;
        if (capacity < VIRTIO_GPU_FB_MIN_CAPACITY) {
            capacity = VIRTIO_GPU_FB_MIN_CAPACITY;
        }
        virtual_fb = (uint32_t *)malloc(capacity);
        if (virtual_fb == NULL) {
            capacity = virtual_bytes;
            virtual_fb = (uint32_t *)malloc(capacity);
            if (virtual_fb == NULL) {
                return false;
            }
        }
        memset(virtual_fb, 0, virtual_bytes);
        g_gpu_fb_capacity = capacity;
    }

    uint16_t pnp_actions[VIRTIO_GPU_MAX_SCANOUTS];
    for (uint32_t i = 0u; i < monitor_count; ++i) {
        pnp_actions[i] =
            (g_gpu_monitor_count == 0u ||
             virtio_gpu_find_scanout(g_gpu_monitors,
                                     g_gpu_monitor_count,
                                     monitors[i].scanout_id) < 0) ?
            PNP_EVENT_DEVICE_ADDED :
            PNP_EVENT_DEVICE_CHANGED;
    }

    for (uint32_t i = 0u; i < monitor_count; ++i) {
        monitors[i].resource_id = 0u;
        monitors[i].fb = NULL;
        monitors[i].fb_bytes = 0u;
        monitors[i].fb_shared = false;
        monitors[i].x -= origin_x;
        monitors[i].y -= origin_y;
        virtio_gpu_fix_mode_metadata(&monitors[i], i);
        uint32_t *shared_fb = NULL;
        if (monitors[i].x == 0 && monitors[i].y == 0 &&
            monitors[i].width == desktop_width &&
            monitors[i].height == desktop_height) {
            shared_fb = virtual_fb;
        }
        if (!virtio_gpu_create_monitor_resource(&monitors[i], shared_fb,
                                                desktop_width,
                                                virtual_bytes)) {
            for (uint32_t j = 0u; j <= i; ++j) {
                virtio_gpu_release_monitor_resource(&monitors[j]);
            }
            if (!fb_reused) {
                free(virtual_fb);
                g_gpu_fb_capacity = prev_capacity;
            }
            return false;
        }
    }

    for (uint32_t i = 0u; i < g_gpu_monitor_count; ++i) {
        if (virtio_gpu_find_scanout(monitors,
                                    monitor_count,
                                    g_gpu_monitors[i].scanout_id) < 0) {
            virtio_gpu_publish_monitor_event(&g_gpu_monitors[i],
                                             i,
                                             PNP_EVENT_DEVICE_REMOVED);
        }
        virtio_gpu_release_monitor_resource(&g_gpu_monitors[i]);
    }
    g_gpu_monitor_count = 0u;

    /* Only the outgoing desktop buffer, and only when we actually switched
     * allocation, is retired (never freed -- user space may still map it). */
    if (!fb_reused) {
        virtio_gpu_retire_framebuffer(g_gpu_fb);
    }
    g_gpu_fb = virtual_fb;
    g_gpu_fb_bytes = virtual_bytes;
    g_gpu_width = desktop_width;
    g_gpu_height = desktop_height;
    g_gpu_origin_x = origin_x;
    g_gpu_origin_y = origin_y;
    g_gpu_monitor_count = monitor_count;
    vlog("[vgpu] apply layout w=");
    vlog_num(desktop_width);
    vlog(" h=");
    vlog_num(desktop_height);
    vlog(" fb=");
    vlog_hex((uint64_t)(uintptr_t)virtual_fb);
    vlog(" bytes=");
    vlog_num(virtual_bytes);
    vlog(" nmon=");
    vlog_num(monitor_count);
    vlog(" mon0fb=");
    vlog_hex((uint64_t)(uintptr_t)(monitor_count != 0u ? monitors[0].fb : NULL));
    vlog("\n");
    vlog_done();
    for (uint32_t i = 0u; i < monitor_count; ++i) {
        g_gpu_monitors[i] = monitors[i];
        virtio_gpu_publish_monitor_event(&g_gpu_monitors[i], i, pnp_actions[i]);
    }
    ++g_gpu_generation;
    return true;
}

static uint32_t virtio_gpu_build_layout_from_info(
    const virtio_gpu_resp_display_info_t *info,
    virtio_gpu_monitor_t *out_monitors,
    int32_t *out_origin_x,
    int32_t *out_origin_y,
    uint32_t *out_width,
    uint32_t *out_height) {
    if (info == NULL || out_monitors == NULL || out_origin_x == NULL ||
        out_origin_y == NULL || out_width == NULL || out_height == NULL) {
        return 0u;
    }

    uint32_t count = 0u;
    int32_t min_x = 0;
    int32_t min_y = 0;
    int32_t max_x = 0;
    int32_t max_y = 0;
    for (uint32_t i = 0u; i < VIRTIO_GPU_MAX_SCANOUTS; ++i) {
        const virtio_gpu_display_one_t *pmode = &info->pmodes[i];
        if (pmode->enabled == 0u ||
            pmode->r.width == 0u || pmode->r.height == 0u) {
            continue;
        }

        virtio_gpu_monitor_t *monitor = &out_monitors[count];
        memset(monitor, 0, sizeof(*monitor));
        monitor->scanout_id = i;
        monitor->x = (int32_t)pmode->r.x;
        monitor->y = (int32_t)pmode->r.y;
        monitor->width = pmode->r.width;
        monitor->height = pmode->r.height;
        monitor->refresh_millihz = 60000u;
        monitor->flags = DISPLAY_MONITOR_FLAG_CONNECTED |
                         DISPLAY_MONITOR_FLAG_MODESET |
                         DISPLAY_MONITOR_FLAG_HOTPLUG |
                         DISPLAY_MONITOR_FLAG_SYNTHETIC_MODE;
        if (count == 0u) {
            monitor->flags |= DISPLAY_MONITOR_FLAG_PRIMARY;
            min_x = monitor->x;
            min_y = monitor->y;
            max_x = monitor->x + (int32_t)monitor->width;
            max_y = monitor->y + (int32_t)monitor->height;
        } else {
            if (monitor->x < min_x) min_x = monitor->x;
            if (monitor->y < min_y) min_y = monitor->y;
            int32_t right = monitor->x + (int32_t)monitor->width;
            int32_t bottom = monitor->y + (int32_t)monitor->height;
            if (right > max_x) max_x = right;
            if (bottom > max_y) max_y = bottom;
        }

        virtio_gpu_resp_edid_t edid;
        memset(&edid, 0, sizeof(edid));
        virtio_gpu_build_modes(monitor,
            gpu_cmd_get_edid(&g_gpu_controlq, i, &edid) ? &edid : NULL);

        ++count;
        if (count >= VIRTIO_GPU_MAX_SCANOUTS) {
            break;
        }
    }

    if (count == 0u) {
        return 0u;
    }

    *out_origin_x = min_x;
    *out_origin_y = min_y;
    *out_width = (uint32_t)(max_x - min_x);
    *out_height = (uint32_t)(max_y - min_y);
    return count;
}

static bool virtio_gpu_layout_matches(const virtio_gpu_monitor_t *monitors,
                                      uint32_t monitor_count,
                                      int32_t origin_x,
                                      int32_t origin_y,
                                      uint32_t width,
                                      uint32_t height) {
    if (monitor_count != g_gpu_monitor_count ||
        origin_x != g_gpu_origin_x || origin_y != g_gpu_origin_y ||
        width != g_gpu_width || height != g_gpu_height) {
        return false;
    }
    for (uint32_t i = 0u; i < monitor_count; ++i) {
        const virtio_gpu_monitor_t *a = &monitors[i];
        const virtio_gpu_monitor_t *b = &g_gpu_monitors[i];
        if (a->scanout_id != b->scanout_id ||
            (a->x - origin_x) != b->x ||
            (a->y - origin_y) != b->y ||
            a->width != b->width ||
            a->height != b->height) {
            return false;
        }
    }
    return true;
}

static bool virtio_gpu_refresh_layout_from_device(void) {
    virtio_gpu_resp_display_info_t info;
    memset(&info, 0, sizeof(info));
    if (!gpu_cmd_get_display_info(&g_gpu_controlq, &info)) {
        return false;
    }

    virtio_gpu_monitor_t monitors[VIRTIO_GPU_MAX_SCANOUTS];
    memset(monitors, 0, sizeof(monitors));
    int32_t origin_x = 0;
    int32_t origin_y = 0;
    uint32_t width = 0u;
    uint32_t height = 0u;
    uint32_t count = virtio_gpu_build_layout_from_info(&info, monitors,
        &origin_x, &origin_y, &width, &height);
    if (count == 0u) {
        return false;
    }
    if (virtio_gpu_layout_matches(monitors, count, origin_x, origin_y,
                                  width, height)) {
        return false;
    }
    return virtio_gpu_apply_layout(monitors, count, origin_x, origin_y,
                                   width, height);
}

static bool virtio_gpu_apply_fallback_layout(uint32_t width, uint32_t height) {
    virtio_gpu_monitor_t monitor;
    memset(&monitor, 0, sizeof(monitor));
    monitor.scanout_id = 0u;
    monitor.x = 0;
    monitor.y = 0;
    monitor.width = width;
    monitor.height = height;
    monitor.refresh_millihz = 60000u;
    monitor.flags = DISPLAY_MONITOR_FLAG_CONNECTED |
                    DISPLAY_MONITOR_FLAG_PRIMARY |
                    DISPLAY_MONITOR_FLAG_MODESET |
                    DISPLAY_MONITOR_FLAG_HOTPLUG |
                    DISPLAY_MONITOR_FLAG_SYNTHETIC_MODE;
    virtio_gpu_build_modes(&monitor, NULL);
    return virtio_gpu_apply_layout(&monitor, 1u, 0, 0, width, height);
}

bool virtio_gpu_init(void) {
    virtio_gpu_pci_t gpu;
    virtio_pci_transport_t t;
    virtqueue_t controlq;

    vlog("[vgpu] init enter\n");
    vlog_done();

    g_gpu_ready = 0;
    gpu_dma_free(g_gpu_controlq.cmd_buf, VIRTIO_GPU_CMD_BUF_BYTES);
    gpu_dma_free(g_gpu_controlq.resp_buf, VIRTIO_GPU_RESP_BUF_BYTES);
    gpu_dma_free(g_gpu_controlq.raw_ring, g_gpu_controlq.raw_ring_bytes);
    memset(&g_gpu_controlq, 0, sizeof(g_gpu_controlq));
    virtio_gpu_release_resources();
    memset(&g_gpu_controlq, 0, sizeof(g_gpu_controlq));
    memset(&g_gpu_transport, 0, sizeof(g_gpu_transport));
    memset(g_gpu_monitors, 0, sizeof(g_gpu_monitors));

    if (!find_virtio_gpu(&gpu)) {
        vlog("[vgpu] no device\n");
        vlog_done();
        return false;
    }
    vlog("[vgpu] find bus=");
    vlog_num(gpu.bus);
    vlog(" dev=");
    vlog_num(gpu.device);
    vlog(" func=");
    vlog_num(gpu.func);
    vlog(" bar4=");
    vlog_hex(gpu.bar_addr[4]);
    vlog(" mem4=");
    vlog_num(gpu.bar_is_mem[4]);
    vlog("\n");
    vlog_done();


    if (!virtio_pci_find_caps(&gpu, &t)) {
        vlog("[vgpu] no caps\n");
        vlog_done();
        return false;
    }
    vlog("[vgpu] caps common=");
    vlog_hex((uint64_t)(uintptr_t)t.common_cfg);
    vlog(" notify=");
    vlog_hex((uint64_t)(uintptr_t)t.notify_base);
    vlog(" devcfg=");
    vlog_hex((uint64_t)(uintptr_t)t.device_cfg);
    vlog("\n");
    vlog_done();

    if (!virtio_pci_device_init(&gpu, &t)) {
        vlog("[vgpu] device init failed\n");
        vlog_done();
        return false;
    }

    if (!virtqueue_init_ctrl(&t, &controlq, 0)) {
        vlog("[vgpu] queue init failed\n");
        vlog_done();
        return false;
    }
    vlog("[vgpu] queue qsize=");
    vlog_num(controlq.queue_size);
    vlog(" ring=");
    vlog_hex((uint64_t)(uintptr_t)controlq.raw_ring);
    vlog(" desc=");
    vlog_hex((uint64_t)(uintptr_t)controlq.desc);
    vlog(" notify=");
    vlog_hex((uint64_t)(uintptr_t)controlq.notify_addr);
    vlog("\n");
    vlog_done();

    uint8_t status = common_read8(t.common_cfg, 20);
    common_write8(t.common_cfg, 20, (uint8_t)(status | VIRTIO_STATUS_DRIVER_OK));

    g_gpu_controlq = controlq;
    g_gpu_transport = t;

    /* The first command after DRIVER_OK can take longer than the steady-state
     * timeout while QEMU brings the GPU worker up. Retry a few times so the
     * real mode is installed on the first layout instead of a fallback that a
     * later poll would have to replace (which used to move the framebuffer out
     * from under a process that had already mapped it). */
    bool layout_ok = false;
    for (uint32_t attempt = 0u; attempt < 5u && !layout_ok; ++attempt) {
        layout_ok = virtio_gpu_refresh_layout_from_device();
#ifdef IMPLUS_DRIVER_MODULE
        if (!layout_ok && g_driver_api != NULL &&
            g_driver_api->timer.msleep != NULL) {
            g_driver_api->timer.msleep(20u);
        }
#endif
    }

    if (!layout_ok && !virtio_gpu_apply_fallback_layout(1024u, 768u)) {
        vlog("[vgpu] layout failed\n");
        vlog_done();
        gpu_dma_free(g_gpu_controlq.cmd_buf, VIRTIO_GPU_CMD_BUF_BYTES);
        gpu_dma_free(g_gpu_controlq.resp_buf, VIRTIO_GPU_RESP_BUF_BYTES);
        gpu_dma_free(g_gpu_controlq.raw_ring, g_gpu_controlq.raw_ring_bytes);
        memset(&g_gpu_controlq, 0, sizeof(g_gpu_controlq));
        g_gpu_fb_capacity = 0u;
        return false;
    }

    g_gpu_ready = 1;
    vlog("[vgpu] ready w=");
    vlog_num(g_gpu_width);
    vlog(" h=");
    vlog_num(g_gpu_height);
    vlog(" fb=");
    vlog_hex((uint64_t)(uintptr_t)g_gpu_fb);
    vlog(" bytes=");
    vlog_num(g_gpu_fb_bytes);
    vlog(" nmon=");
    vlog_num(g_gpu_monitor_count);
    vlog("\n");
    vlog_done();
    return true;
}

bool virtio_gpu_is_ready(void) {
    return g_gpu_ready != 0;
}

uint32_t virtio_gpu_width(void) {
    return g_gpu_width;
}

uint32_t virtio_gpu_height(void) {
    return g_gpu_height;
}

void virtio_gpu_draw_pixel(uint32_t x, uint32_t y, uint32_t color) {
    if (!g_gpu_ready || !g_gpu_fb || x >= g_gpu_width || y >= g_gpu_height) {
        return;
    }
    uint64_t irq = gpu_lock();
    if (g_gpu_ready && g_gpu_fb && x < g_gpu_width && y < g_gpu_height) {
        g_gpu_fb[(uint64_t)y * g_gpu_width + x] = color;
    }
    gpu_unlock(irq);
}

void virtio_gpu_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color) {
    if (!g_gpu_ready || !g_gpu_fb || w == 0 || h == 0) {
        return;
    }
    if (x >= g_gpu_width || y >= g_gpu_height) {
        return;
    }

    uint64_t irq = gpu_lock();
    if (!g_gpu_ready || !g_gpu_fb) {
        gpu_unlock(irq);
        return;
    }

    uint32_t x_end = x + w;
    uint32_t y_end = y + h;
    if (x_end > g_gpu_width || x_end < x) {
        x_end = g_gpu_width;
    }
    if (y_end > g_gpu_height || y_end < y) {
        y_end = g_gpu_height;
    }

    for (uint32_t py = y; py < y_end; ++py) {
        uint64_t row = (uint64_t)py * g_gpu_width;
        uint32_t *rp = &g_gpu_fb[row + x];
        uint32_t count = x_end - x;
        
        uint32_t i = 0;
        for (; i + 8 <= count; i += 8) {
            rp[i]   = color; rp[i+1] = color;
            rp[i+2] = color; rp[i+3] = color;
            rp[i+4] = color; rp[i+5] = color;
            rp[i+6] = color; rp[i+7] = color;
        }
        for (; i < count; ++i)
            rp[i] = color;
    }
}

static void virtio_gpu_present_rects_locked(const display_rect_t *rects,
                                            uint32_t count)
{
    if (g_vlog_budget != 0u) {
        vlog("[vgpu] present rects=");
        vlog_num(count);
        vlog(" fb=");
        vlog_hex((uint64_t)(uintptr_t)g_gpu_fb);
        vlog(" nmon=");
        vlog_num(g_gpu_monitor_count);
        vlog("\n");
        vlog_done();
    }

    for (uint32_t r = 0u; r < count; ++r) {
        int64_t x0 = rects[r].x;
        int64_t y0 = rects[r].y;
        int64_t x1 = x0 + (int64_t)rects[r].w;
        int64_t y1 = y0 + (int64_t)rects[r].h;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 > (int64_t)g_gpu_width) x1 = (int64_t)g_gpu_width;
        if (y1 > (int64_t)g_gpu_height) y1 = (int64_t)g_gpu_height;
        if (x1 <= x0 || y1 <= y0) {
            continue;
        }

        for (uint32_t i = 0u; i < g_gpu_monitor_count; ++i) {
            virtio_gpu_monitor_t *monitor = &g_gpu_monitors[i];
            if (monitor->fb == NULL || monitor->resource_id == 0u ||
                monitor->width == 0u || monitor->height == 0u ||
                g_gpu_fb == NULL) {
                continue;
            }

            int64_t mx0 = monitor->x;
            int64_t my0 = monitor->y;
            int64_t mx1 = mx0 + (int64_t)monitor->width;
            int64_t my1 = my0 + (int64_t)monitor->height;
            int64_t ix0 = x0 > mx0 ? x0 : mx0;
            int64_t iy0 = y0 > my0 ? y0 : my0;
            int64_t ix1 = x1 < mx1 ? x1 : mx1;
            int64_t iy1 = y1 < my1 ? y1 : my1;
            if (ix1 <= ix0 || iy1 <= iy0) {
                continue;
            }

            uint32_t w = (uint32_t)(ix1 - ix0);
            uint32_t h = (uint32_t)(iy1 - iy0);
            uint32_t local_x = (uint32_t)(ix0 - mx0);
            uint32_t local_y = (uint32_t)(iy0 - my0);

            /* A full-desktop monitor samples g_gpu_fb directly; every other
             * monitor keeps a compact, monitor-local copy and only the
             * damaged part of it is refreshed here. */
            if (!monitor->fb_shared) {
                for (uint32_t row = 0u; row < h; ++row) {
                    memcpy(&monitor->fb[(uint64_t)(local_y + row) *
                                            monitor->width + local_x],
                           &g_gpu_fb[(uint64_t)((uint32_t)iy0 + row) *
                                         g_gpu_width + (uint32_t)ix0],
                           (size_t)w * sizeof(uint32_t));
                }
            }

            uint64_t offset = ((uint64_t)local_y * monitor->width +
                               (uint64_t)local_x) * sizeof(uint32_t);
            if (!gpu_cmd_transfer_to_host_2d(&g_gpu_controlq,
                                             monitor->resource_id,
                                             local_x, local_y, w, h,
                                             offset)) {
                continue;
            }
            (void)gpu_cmd_resource_flush(&g_gpu_controlq,
                                         monitor->resource_id,
                                         local_x, local_y, w, h);
        }
    }
}

void virtio_gpu_present_rects(const display_rect_t *rects, uint32_t count) {
    if (!g_gpu_ready) {
        return;
    }

    uint64_t irq = gpu_lock();
    if (!g_gpu_ready) {
        gpu_unlock(irq);
        return;
    }

    if (rects == NULL || count == 0u) {
        display_rect_t full;
        full.x = 0;
        full.y = 0;
        full.w = g_gpu_width;
        full.h = g_gpu_height;
        virtio_gpu_present_rects_locked(&full, 1u);
    } else {
        virtio_gpu_present_rects_locked(rects, count);
    }

    gpu_unlock(irq);
}

void virtio_gpu_present(void) {
    virtio_gpu_present_rects(NULL, 0u);
}

/* ---- 3D (virgl) public API ---- */

static uint32_t g_gpu_next_context_id = 1u;

bool virtio_gpu_has_3d(void) {
    return g_gpu_ready && (g_gpu_features0 & VIRTIO_GPU_F_VIRGL) != 0u;
}

bool virtio_gpu_has_context_init(void) {
    return virtio_gpu_has_3d() &&
           (g_gpu_features0 & VIRTIO_GPU_F_CONTEXT_INIT) != 0u;
}

uint32_t virtio_gpu_resource_id_alloc(void) {
    if (!g_gpu_ready) return 0u;
    uint64_t irq_state = gpu_lock();
    uint32_t id = virtio_gpu_next_resource_id();
    gpu_unlock(irq_state);
    return id;
}

bool virtio_gpu_get_capset_info(uint32_t index, uint32_t *out_id,
                                uint32_t *out_version, uint32_t *out_size) {
    if (!virtio_gpu_has_3d() || out_id == NULL ||
        out_version == NULL || out_size == NULL ||
        index >= VIRTIO_GPU_MAX_CAPSETS) return false;
    virtio_gpu_resp_capset_info_t info;
    memset(&info, 0, sizeof(info));
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_get_capset_info(&g_gpu_controlq, index, &info);
    gpu_unlock(irq_state);
    if (!ok) return false;
    *out_id = info.capset_id;
    *out_version = info.max_version;
    *out_size = info.max_size;
    return true;
}

bool virtio_gpu_get_capset(uint32_t capset_id, uint32_t version,
                           void *out_caps, uint32_t out_size) {
    if (!virtio_gpu_has_3d() || out_caps == NULL || out_size == 0u) {
        return false;
    }
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_get_capset(&g_gpu_controlq, capset_id, version,
                                out_caps, out_size);
    gpu_unlock(irq_state);
    return ok != 0;
}

uint32_t virtio_gpu_ctx_create(uint32_t context_spec, const char *name) {
    if (!virtio_gpu_has_3d()) return 0u;
    uint64_t irq_state = gpu_lock();
    uint32_t id = g_gpu_next_context_id++;
    if (g_gpu_next_context_id == 0u) g_gpu_next_context_id = 1u;
    int ok = gpu_cmd_ctx_create(&g_gpu_controlq, id, context_spec, name);
    gpu_unlock(irq_state);
    return ok ? id : 0u;
}

bool virtio_gpu_ctx_destroy(uint32_t context_id) {
    if (!virtio_gpu_has_3d() || context_id == 0u) return false;
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_ctx_destroy(&g_gpu_controlq, context_id);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_submit_3d(uint32_t context_id, const void *blob, uint32_t blob_size) {
    if (!virtio_gpu_has_3d() || context_id == 0u || blob == NULL || blob_size == 0u) {
        return false;
    }
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_submit_3d(&g_gpu_controlq, context_id, blob, blob_size);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_resource_create_3d_ex(uint32_t resource_id,
                                      uint32_t target, uint32_t format,
                                      uint32_t bind, uint32_t width,
                                      uint32_t height, uint32_t depth,
                                      uint32_t array_size,
                                      uint32_t last_level,
                                      uint32_t nr_samples, uint32_t flags) {
    if (!virtio_gpu_has_3d() || resource_id == 0u || width == 0u ||
        height == 0u || depth == 0u || array_size == 0u || target == 0u) {
        return false;
    }
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_resource_create_3d(&g_gpu_controlq, resource_id,
                                        target, format, bind, width, height,
                                        depth, array_size, last_level,
                                        nr_samples, flags);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_resource_create_3d(uint32_t resource_id, uint32_t format,
                                   uint32_t width, uint32_t height,
                                   uint32_t depth, uint32_t target,
                                   uint32_t bind) {
    return virtio_gpu_resource_create_3d_ex(resource_id, target, format, bind,
                                            width, height, depth,
                                            1u, 0u, 0u, 0u);
}

bool virtio_gpu_resource_detach_backing(uint32_t resource_id) {
    if (!virtio_gpu_has_3d() || resource_id == 0u) return false;
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_resource_detach_backing(&g_gpu_controlq, resource_id);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_resource_unref(uint32_t resource_id) {
    if (!virtio_gpu_has_3d() || resource_id == 0u) return false;
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_resource_unref(&g_gpu_controlq, resource_id);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_resource_attach_backing(uint32_t resource_id,
                                        uint64_t physical_address,
                                        uint32_t bytes) {
    if (!virtio_gpu_has_3d() || resource_id == 0u ||
        physical_address == 0u || bytes == 0u) return false;
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_resource_attach_backing(&g_gpu_controlq, resource_id,
                                             (void *)(uintptr_t)physical_address,
                                             bytes);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_ctx_attach_resource(uint32_t context_id, uint32_t resource_id) {
    if (!virtio_gpu_has_3d()) return false;
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_ctx_resource(&g_gpu_controlq,
                                  VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE,
                                  context_id, resource_id);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_ctx_detach_resource(uint32_t context_id, uint32_t resource_id) {
    if (!virtio_gpu_has_3d()) return false;
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_ctx_resource(&g_gpu_controlq,
                                  VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE,
                                  context_id, resource_id);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_transfer_3d_ex(uint32_t context_id, uint32_t resource_id,
                               bool to_host, uint32_t x, uint32_t y,
                               uint32_t z, uint32_t width, uint32_t height,
                               uint32_t depth, uint64_t offset,
                               uint32_t level, uint32_t stride,
                               uint32_t layer_stride) {
    if (!virtio_gpu_has_3d() || context_id == 0u || resource_id == 0u ||
        width == 0u || height == 0u || depth == 0u) return false;
    uint64_t irq_state = gpu_lock();
    int ok = gpu_cmd_transfer_3d(&g_gpu_controlq,
                                 to_host ? VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D
                                         : VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D,
                                 context_id, resource_id, x, y, z,
                                 width, height, depth, offset, level,
                                 stride, layer_stride);
    gpu_unlock(irq_state);
    return ok != 0;
}

bool virtio_gpu_transfer_to_host_3d(uint32_t context_id, uint32_t resource_id,
                                    uint32_t x, uint32_t y,
                                    uint32_t w, uint32_t h) {
    if (!virtio_gpu_has_3d() || context_id == 0u || resource_id == 0u) return false;
    return virtio_gpu_transfer_3d_ex(context_id, resource_id, true,
                                     x, y, 0u, w, h, 1u,
                                     0u, 0u, 0u, 0u);
}

bool virtio_gpu_transfer_from_host_3d(uint32_t context_id, uint32_t resource_id,
                                      uint32_t x, uint32_t y,
                                      uint32_t w, uint32_t h) {
    if (!virtio_gpu_has_3d() || context_id == 0u || resource_id == 0u) return false;
    return virtio_gpu_transfer_3d_ex(context_id, resource_id, false,
                                     x, y, 0u, w, h, 1u,
                                     0u, 0u, 0u, 0u);
}

static bool virtio_gpu_probe(void) {
    virtio_gpu_pci_t gpu;
    return find_virtio_gpu(&gpu) != 0;
}

static void *virtio_gpu_get_framebuffer(void) {
    return g_gpu_fb;
}

static uint32_t virtio_gpu_generation(void) {
    return g_gpu_generation;
}

static bool virtio_gpu_config_event_pending(void) {
    if (g_gpu_transport.device_cfg == NULL) {
        return false;
    }
    uint32_t events = *(volatile uint32_t *)(g_gpu_transport.device_cfg + 0);
    if ((events & VIRTIO_GPU_EVENT_DISPLAY) == 0u) {
        return false;
    }
    *(volatile uint32_t *)(g_gpu_transport.device_cfg + 4) =
        VIRTIO_GPU_EVENT_DISPLAY;
    return true;
}

static bool virtio_gpu_poll_config(void) {
    if (!g_gpu_ready) {
        return false;
    }
    (void)virtio_gpu_config_event_pending();
    bool changed = virtio_gpu_refresh_layout_from_device();
    if (changed && g_vlog_budget != 0u) {
        vlog("[vgpu] poll relayout fb=");
        vlog_hex((uint64_t)(uintptr_t)g_gpu_fb);
        vlog("\n");
        vlog_done();
    }
    return changed;
}

static uint32_t virtio_gpu_monitor_count(void) {
    return g_gpu_ready ? g_gpu_monitor_count : 0u;
}

static bool virtio_gpu_get_topology(display_topology_t *out_topology) {
    if (!g_gpu_ready || out_topology == NULL) {
        return false;
    }
    memset(out_topology, 0, sizeof(*out_topology));
    out_topology->generation = g_gpu_generation;
    out_topology->monitor_count = g_gpu_monitor_count;
    out_topology->primary_monitor = 0u;
    out_topology->origin_x = g_gpu_origin_x;
    out_topology->origin_y = g_gpu_origin_y;
    out_topology->width = g_gpu_width;
    out_topology->height = g_gpu_height;
    return true;
}

static bool virtio_gpu_get_monitor_info(uint32_t monitor_index,
                                        display_monitor_info_t *out_info) {
    if (!g_gpu_ready || out_info == NULL ||
        monitor_index >= g_gpu_monitor_count) {
        return false;
    }

    const virtio_gpu_monitor_t *monitor = &g_gpu_monitors[monitor_index];
    memset(out_info, 0, sizeof(*out_info));
    out_info->index = monitor_index;
    out_info->id = monitor->scanout_id;
    out_info->flags = monitor->flags;
    out_info->output_type = DISPLAY_OUTPUT_VIRTIO_SCANOUT;
    out_info->x = monitor->x;
    out_info->y = monitor->y;
    out_info->width = monitor->width;
    out_info->height = monitor->height;
    out_info->physical_width_mm = monitor->physical_width_mm;
    out_info->physical_height_mm = monitor->physical_height_mm;
    out_info->refresh_millihz = monitor->refresh_millihz;
    out_info->current_mode = monitor->current_mode;
    out_info->mode_count = monitor->mode_count;
    out_info->generation = g_gpu_generation;
    virtio_gpu_copy_string(out_info->name, sizeof(out_info->name),
                           "VirtIO GPU Scanout 0");
    virtio_gpu_copy_string(out_info->output_name, sizeof(out_info->output_name),
                           "virtio-scanout0");
    if (monitor_index < 10u) {
        out_info->name[18] = (char)('0' + monitor_index);
        out_info->output_name[14] = (char)('0' + monitor_index);
    }
    return true;
}

static bool virtio_gpu_get_mode_info(uint32_t monitor_index,
                                     uint32_t mode_index,
                                     display_mode_info_t *out_info) {
    if (!g_gpu_ready || out_info == NULL ||
        monitor_index >= g_gpu_monitor_count ||
        mode_index >= g_gpu_monitors[monitor_index].mode_count) {
        return false;
    }
    *out_info = g_gpu_monitors[monitor_index].modes[mode_index];
    out_info->monitor_index = monitor_index;
    out_info->mode_index = mode_index;
    return true;
}

static bool virtio_gpu_set_mode(uint32_t monitor_index, uint32_t mode_index) {
    if (!g_gpu_ready || monitor_index >= g_gpu_monitor_count ||
        mode_index >= g_gpu_monitors[monitor_index].mode_count) {
        return false;
    }

    virtio_gpu_monitor_t monitors[VIRTIO_GPU_MAX_SCANOUTS];
    memset(monitors, 0, sizeof(monitors));
    for (uint32_t i = 0u; i < g_gpu_monitor_count; ++i) {
        monitors[i] = g_gpu_monitors[i];
        monitors[i].resource_id = 0u;
        monitors[i].fb = NULL;
        monitors[i].fb_bytes = 0u;
        monitors[i].fb_shared = false;
    }

    const display_mode_info_t *selected =
        &g_gpu_monitors[monitor_index].modes[mode_index];
    for (uint32_t mode = 0u; mode < monitors[monitor_index].mode_count; ++mode) {
        monitors[monitor_index].modes[mode].flags &= ~DISPLAY_MODE_FLAG_CURRENT;
    }
    monitors[monitor_index].width = selected->width;
    monitors[monitor_index].height = selected->height;
    monitors[monitor_index].refresh_millihz = selected->refresh_millihz;
    monitors[monitor_index].current_mode = mode_index;
    monitors[monitor_index].modes[mode_index].flags |= DISPLAY_MODE_FLAG_CURRENT;

    int32_t min_x = monitors[0].x;
    int32_t min_y = monitors[0].y;
    int32_t max_x = monitors[0].x + (int32_t)monitors[0].width;
    int32_t max_y = monitors[0].y + (int32_t)monitors[0].height;
    for (uint32_t i = 1u; i < g_gpu_monitor_count; ++i) {
        if (monitors[i].x < min_x) min_x = monitors[i].x;
        if (monitors[i].y < min_y) min_y = monitors[i].y;
        int32_t right = monitors[i].x + (int32_t)monitors[i].width;
        int32_t bottom = monitors[i].y + (int32_t)monitors[i].height;
        if (right > max_x) max_x = right;
        if (bottom > max_y) max_y = bottom;
    }

    return virtio_gpu_apply_layout(monitors, g_gpu_monitor_count,
                                   min_x, min_y,
                                   (uint32_t)(max_x - min_x),
                                   (uint32_t)(max_y - min_y));
}

static const driver_display_t g_virtio_display_driver = {
    .name = VIRTIO_GPU_DEVICE_NAME,
    .probe = virtio_gpu_probe,
    .init = virtio_gpu_init,
    .is_ready = virtio_gpu_is_ready,
    .width = virtio_gpu_width,
    .height = virtio_gpu_height,
    .draw_pixel = virtio_gpu_draw_pixel,
    .fill_rect = virtio_gpu_fill_rect,
    .present = virtio_gpu_present,
    .present_rects = virtio_gpu_present_rects,
    .get_framebuffer = virtio_gpu_get_framebuffer,
    .get_generation = virtio_gpu_generation,
    .poll_config = virtio_gpu_poll_config,
    .get_monitor_count = virtio_gpu_monitor_count,
    .get_topology = virtio_gpu_get_topology,
    .get_monitor_info = virtio_gpu_get_monitor_info,
    .get_mode_info = virtio_gpu_get_mode_info,
    .set_mode = virtio_gpu_set_mode,
    .gpu_has_3d = virtio_gpu_has_3d,
    .gpu_has_context_init = virtio_gpu_has_context_init,
    .gpu_resource_id_alloc = virtio_gpu_resource_id_alloc,
    .gpu_get_capset_info = virtio_gpu_get_capset_info,
    .gpu_get_capset = virtio_gpu_get_capset,
    .gpu_context_create = virtio_gpu_ctx_create,
    .gpu_context_destroy = virtio_gpu_ctx_destroy,
    .gpu_submit_3d = virtio_gpu_submit_3d,
    .gpu_resource_create_3d = virtio_gpu_resource_create_3d_ex,
    .gpu_resource_attach_backing = virtio_gpu_resource_attach_backing,
    .gpu_resource_detach_backing = virtio_gpu_resource_detach_backing,
    .gpu_resource_unref = virtio_gpu_resource_unref,
    .gpu_context_attach_resource = virtio_gpu_ctx_attach_resource,
    .gpu_context_detach_resource = virtio_gpu_ctx_detach_resource,
    .gpu_transfer_3d = virtio_gpu_transfer_3d_ex,
};

static void virtio_gpu_driver_shutdown(void)
{
    virtio_gpu_release_resources();
    gpu_dma_free(g_gpu_controlq.cmd_buf, VIRTIO_GPU_CMD_BUF_BYTES);
    gpu_dma_free(g_gpu_controlq.resp_buf, VIRTIO_GPU_RESP_BUF_BYTES);
    gpu_dma_free(g_gpu_controlq.raw_ring, g_gpu_controlq.raw_ring_bytes);
    memset(&g_gpu_controlq, 0, sizeof(g_gpu_controlq));
    g_gpu_ready = 0;
    memset(&g_gpu_transport, 0, sizeof(g_gpu_transport));
    g_driver_api = NULL;
}

static const driver_module_descriptor_t g_virtio_display_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_DISPLAY,
    .load_priority = 20u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_virtio_display_driver,
    .shutdown = virtio_gpu_driver_shutdown,
};

#ifdef IMPLUS_DRIVER_MODULE
#undef malloc
#undef free
#undef pci_read_config
#undef pci_write_config
#undef map_mmio_virt
#undef memset
#undef memcpy
#undef strcmp

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api) {
    if (!api ||
        !api->malloc || !api->free ||
        !api->mem.dma_alloc_ex || !api->mem.dma_free ||
        !api->pci_read_config || !api->pci_write_config ||
        !api->map_mmio_virt) {
        return NULL;
    }

    g_driver_api = api;
    return &g_virtio_display_module;
}

#endif
