#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "kernel/interfaces/device.h"
#include "kernel/pnp.h"
#include "kernel/interfaces/vfs_file.h"

#define DRIVER_API_VERSION_MAJOR 2u
#define DRIVER_API_VERSION_MINOR 2u

#define DRIVER_DESCRIPTOR_MAGIC 0x44525641u
#define DRIVER_DESCRIPTOR_VERSION 2u
#define DRIVER_MAX_DEPS 4u

typedef struct __attribute__((packed)) {
    uint16_t keycode;
    uint8_t pressed;
    uint8_t ascii;
    uint8_t modifiers;
    uint8_t reserved[3];
} driver_keyboard_event_t;

typedef struct __attribute__((packed)) {
    uint16_t x;
    uint16_t y;
    uint8_t buttons;
    int8_t wheel;
    uint8_t reserved[2];
} driver_mouse_event_t;

typedef struct {
    void (*msleep)(uint32_t ms);
    uint32_t (*hz)(void);
    uint64_t (*ticks)(void);
    uint64_t (*monotonic_ns)(void);
} driver_api_timer_t;

typedef struct {
    void *(*malloc)(uint64_t size);
    void (*free)(void *ptr);
    void *(*dma_alloc)(size_t size, uint64_t *phys_out);
    void (*dma_free)(void *ptr, size_t size);
    uint64_t (*virt_to_phys)(void *virt);
    void *(*memset)(void *s, int c, size_t n);
    void *(*memcpy)(void *dst, const void *src, size_t n);
    void *(*dma_alloc_ex)(size_t size,
                          size_t alignment,
                          uint64_t max_address,
                          uint64_t *phys_out);
} driver_api_mem_t;

typedef struct {
    uint8_t (*inb)(uint16_t port);
    void (*outb)(uint16_t port, uint8_t value);
    uint32_t (*inl)(uint16_t port);
    void (*outl)(uint16_t port, uint32_t value);
} driver_api_io_t;

typedef struct {
    bool (*disk_read)(uint64_t lba, uint8_t *buffer, uint32_t sector_count);
    bool (*disk_write)(uint64_t lba, const uint8_t *buffer, uint32_t sector_count);
    uint64_t (*disk_get_partition_lba)(void);
    uint32_t (*pci_read_config)(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset);
    void (*pci_write_config)(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset, uint32_t value);
    void *(*map_mmio_virt)(uint64_t phys_addr);
    void *(*map_mmio_range)(uint64_t phys_addr, size_t size);
} driver_api_hw_t;

/*
 * Filesystem access, added in API 2.2. A thin 1:1 mirror of Core/vfs/VFS.h's
 * find_file/read_at/get_file_size/close_file. Safe to call from any point in
 * a driver module's life EXCEPT its own driver_module_init() entry point:
 * VFS is not mounted yet that early in boot (kernel_main.c initializes
 * drivers before it initializes filesystems), so find_file() will simply
 * return false there. From any later callback (poll(), is_ready(), a
 * probe(), ...) it works normally. Intended use: loading firmware blobs by
 * path (see Kernel/Drivers/Wi-Fi/AX900/AX900.c for a worked example),
 * not general-purpose file access -- there is deliberately no write/creat/
 * mkdir surface here.
 */
typedef struct {
    bool (*find_file)(const char *path, vfs_file_t *out_file);
    bool (*read_at)(vfs_file_t *file, uint32_t offset, uint8_t *buffer, uint32_t size);
    uint32_t (*get_file_size)(vfs_file_t *file);
    bool (*close_file)(vfs_file_t *file);
} driver_api_fs_t;

/*
 * One bus-enumerated device, reported to BusRegistry
 * (Kernel/Drivers/Module/BusRegistry.h) by a bus driver (PCI_Main.c,
 * USB_Main.c) and handed to a matched device driver's probe()/remove() (see
 * driver_bus_match_t / driver_module_descriptor_t further down). Added in
 * API 2.2. `bus_context` is bus-specific and opaque to BusRegistry itself --
 * only the reporting bus driver and a device driver that understands that
 * bus's convention dereference it (e.g. a `driver_pci_device_t *` for PCI,
 * or a small addr/interface/endpoints struct for USB).
 */
typedef struct {
    device_type_t bus_type;
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t  class_code;
    uint8_t  subclass;
    uint8_t  protocol;
    const void *bus_context;
} bus_device_t;

typedef struct {
    uint8_t bus;
    uint8_t device;
    uint8_t function;
    uint8_t class_code;
    uint8_t subclass;
    uint8_t prog_if;
    uint8_t revision;
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t interrupt_line;
    uint8_t interrupt_pin;
} driver_pci_device_t;

typedef struct {
    uint64_t address;
    uint64_t size;
    uint8_t is_io;
    uint8_t is_64bit;
    uint8_t prefetchable;
    uint8_t reserved;
} driver_pci_bar_t;

typedef void (*driver_irq_handler_t)(void *context);

typedef struct {
    uint32_t (*get_device_count)(void);
    bool (*get_device)(uint32_t index, driver_pci_device_t *out_device);
    uint32_t (*read_config)(uint8_t bus,
                            uint8_t device,
                            uint8_t function,
                            uint16_t offset);
    void (*write_config)(uint8_t bus,
                         uint8_t device,
                         uint8_t function,
                         uint16_t offset,
                         uint32_t value);
    bool (*get_bar)(uint8_t bus,
                    uint8_t device,
                    uint8_t function,
                    uint8_t bar_index,
                    driver_pci_bar_t *out_bar);
    int32_t (*find_capability)(uint8_t bus,
                               uint8_t device,
                               uint8_t function,
                               uint8_t capability_id);
    bool (*enable_bus_master)(uint8_t bus,
                              uint8_t device,
                              uint8_t function);
    int32_t (*enable_msix)(uint8_t bus,
                           uint8_t device,
                           uint8_t function,
                           uint32_t requested_vectors,
                           uint32_t *out_vectors);
    void (*disable_msix)(uint8_t bus,
                         uint8_t device,
                         uint8_t function,
                         const uint32_t *vectors,
                         uint32_t vector_count);
    int32_t (*register_irq)(uint32_t vector,
                            driver_irq_handler_t handler,
                            void *context);
    void (*unregister_irq)(uint32_t vector);
} driver_api_pci_t;

typedef struct {
    void *(*create)(void);
    void (*destroy)(void *event);
    void (*reset)(void *event);
    void (*signal)(void *event);
    bool (*wait)(void *event, uint32_t timeout_ms);
} driver_api_event_t;

typedef struct {
    uint32_t (*cpu_count)(void);
    uint32_t (*current_cpu)(void);
} driver_api_system_t;

/*
 * BusRegistry access, added in API 2.2 alongside driver_bus_match_t /
 * bus_device_t above. A bus driver module (PCI_Main.c, USB_Main.c) calls
 * report_device() once per device/interface it enumerates instead of
 * hand-rolling dispatch to specific device driver modules itself -- see
 * Kernel/Drivers/Module/BusRegistry.h for the matching rules.
 *
 * register_device()/unregister_device() are the *dynamic* counterpart to
 * the one-shot registration a module's own driver_module_descriptor_t
 * already gets at boot/load time (driver_manager_attach(), called
 * automatically by DriverModule.c): they let an *already-loaded* module
 * register a device into DeviceRegistry from any callback, at any time --
 * e.g. a USB Wi-Fi driver that only knows it should present itself as a
 * DEVICE_TYPE_NIC once it has actually associated to a network, well after
 * its own module load. Same underlying registry as the automatic path
 * (wired to driver_manager_attach()/_detach() in DriverModule.c), so it
 * also publishes the same PNP_EVENT_DRIVER_READY notification.
 */
typedef struct {
    bool (*report_device)(const bus_device_t *dev);
    void (*report_device_removed)(const bus_device_t *dev);
    bool (*register_device)(const char *name, device_type_t kind, const void *driver_api);
    bool (*unregister_device)(const char *name);
} driver_api_bus_t;

typedef struct {
    void (*write_char)(char c);
    void (*write_string)(const char *str);
    void (*write_uint32)(uint32_t val);
} driver_api_debug_t;

typedef struct {
    void (*cpu_halt)(void);
    void (*cpu_pause)(void);
    void (*cpu_enable_interrupts)(void);
    void (*cpu_disable_interrupts)(void);
    uint64_t (*cpu_save_interrupts)(void);
    void (*cpu_restore_interrupts)(uint64_t state);
    void (*mmu_invalidate_tlb)(uintptr_t addr);
    uint64_t (*cpu_read_cr)(int reg);
    void (*cpu_write_cr)(int reg, uint64_t value);
    void (*cpu_memory_barrier)(void);
    void (*io_delay)(void);
    uint64_t (*cpu_read_msr)(uint32_t msr);
    void (*cpu_write_msr)(uint32_t msr, uint64_t value);
    void (*cpu_get_id)(uint32_t leaf, uint32_t subleaf, uint32_t *eax, uint32_t *ebx, uint32_t *ecx, uint32_t *edx);
    void (*cpu_get_gdt_ptr)(void *ptr);
    void (*cpu_invalidate_caches)(void);
    void (*arch_switch_stack)(uintptr_t sp);
    uint64_t (*cpu_get_current_el)(void);
    void (*cpu_set_vbar)(void *vbar);
    uint64_t (*cpu_read_fs_base)(void);
    void (*cpu_write_fs_base)(uint64_t val);
    void (*cpu_save_fpu)(uint8_t *state);
    void (*cpu_restore_fpu)(uint8_t *state);

    void (*io_out8)(uint16_t port, uint8_t value);
    uint8_t (*io_in8)(uint16_t port);
    void (*io_out16)(uint16_t port, uint16_t value);
    uint16_t (*io_in16)(uint16_t port);
    void (*io_out32)(uint16_t port, uint32_t value);
    uint32_t (*io_in32)(uint16_t port);
    void (*io_outsw)(uint16_t port, const void *addr, uint32_t count);
} driver_api_hal_t;

typedef struct {
    uint16_t version_major;
    uint16_t version_minor;
    uint32_t reserved;

    driver_api_timer_t timer;
    driver_api_mem_t mem;
    driver_api_io_t io;
    driver_api_hw_t hw;
    driver_api_debug_t dbg;
    driver_api_hal_t hal;
    driver_api_pci_t pci;
    driver_api_event_t event;
    driver_api_system_t system;
    driver_api_fs_t fs;
    driver_api_bus_t bus;

    void (*timer_msleep)(uint32_t ms);
    uint32_t (*timer_hz)(void);
    uint64_t (*timer_ticks)(void);
    void *(*malloc)(uint64_t size);
    void (*free)(void *ptr);
    void *(*dma_alloc)(size_t size, uint64_t *phys_out);
    void (*dma_free)(void *ptr, size_t size);
    uint64_t (*virt_to_phys)(void *virt);
    void *(*memset)(void *s, int c, size_t n);
    void *(*memcpy)(void *dst, const void *src, size_t n);
    uint8_t (*inb)(uint16_t port);
    void (*outb)(uint16_t port, uint8_t value);
    uint32_t (*inl)(uint16_t port);
    void (*outl)(uint16_t port, uint32_t value);
    bool (*disk_read)(uint64_t lba, uint8_t *buffer, uint32_t sector_count);
    bool (*disk_write)(uint64_t lba, const uint8_t *buffer, uint32_t sector_count);
    uint64_t (*disk_get_partition_lba)(void);
    uint32_t (*pci_read_config)(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset);
    void (*pci_write_config)(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset, uint32_t value);
    void *(*map_mmio_virt)(uint64_t phys_addr);
    void (*serial_write_char)(char c);
    void (*serial_write_string)(const char *str);
    void (*serial_write_uint32)(uint32_t val);
    void (*pnp_notify)(const pnp_event_t *event);
} driver_binary_t;

typedef struct {
    void (*init)(void);
    void (*poll)(void);
    int32_t (*read_keyboard)(driver_keyboard_event_t *out_event);
    int32_t (*read_mouse)(driver_mouse_event_t *out_event);
    void (*drain_keyboard)(driver_keyboard_event_t *tmp,
                           void (*forward)(driver_keyboard_event_t *));
    void (*drain_mouse)(driver_mouse_event_t *tmp,
                        void (*forward)(driver_mouse_event_t *));
    bool (*poll_hotplug)(void);
} driver_input_t;

#define DRIVER_BLOCK_FLAG_WRITABLE  (1u << 0)
#define DRIVER_BLOCK_FLAG_REMOVABLE (1u << 1)
#define DRIVER_BLOCK_FLAG_BOOT      (1u << 2)
#define DRIVER_BLOCK_IDENTITY_PCI_VALID (1u << 0)

typedef enum {
    DRIVER_BLOCK_TRANSPORT_UNKNOWN = 0,
    DRIVER_BLOCK_TRANSPORT_ATA,
    DRIVER_BLOCK_TRANSPORT_USB,
    DRIVER_BLOCK_TRANSPORT_AHCI,
    DRIVER_BLOCK_TRANSPORT_NVME,
    DRIVER_BLOCK_TRANSPORT_VIRTIO,
} driver_block_transport_t;

typedef struct {
    uint64_t block_count;
    uint32_t logical_block_size;
    uint32_t physical_block_size;
    uint32_t flags;
    driver_block_transport_t transport;
    uint32_t identity_flags;
    uint16_t pci_segment;
    uint8_t pci_bus;
    uint8_t pci_device;
    uint8_t pci_function;
    uint8_t reserved0;
    uint16_t controller_port;
    uint32_t namespace_id;
    char model[64];
    char serial[32];
} driver_block_info_t;

typedef struct {
    const char *name;
    uint32_t priority;
    bool (*init)(void);
    bool (*is_ready)(void);
    uint32_t (*get_device_count)(void);
    bool (*get_info)(uint32_t device_index, driver_block_info_t *out_info);
    bool (*read_blocks)(uint32_t device_index,
                        uint64_t lba,
                        void *buffer,
                        uint32_t block_count);
    bool (*write_blocks)(uint32_t device_index,
                         uint64_t lba,
                         const void *buffer,
                         uint32_t block_count);
    bool (*flush)(uint32_t device_index);
} driver_storage_t;

#define DRIVER_AUDIO_FORMAT_S16_LE 1u

typedef struct {
    uint32_t sample_rate;
    uint16_t channels;
    uint16_t format;
    uint32_t period_bytes;
    uint32_t period_count;
} driver_audio_info_t;

typedef struct {
    const char *name;
    uint32_t priority;
    bool (*init)(void);
    bool (*is_ready)(void);
    bool (*get_info)(driver_audio_info_t *out_info);
    bool (*open)(void);
    int64_t (*write)(const void *pcm, uint64_t bytes);
    bool (*drain)(uint32_t timeout_ms);
    void (*close)(void);
} driver_audio_t;

typedef struct {
    bool (*submit_interrupt_in_async)(uint8_t addr, uint8_t ep_num,
                                      uint16_t max_packet_size,
                                      void *dma_buf, uint64_t dma_phys,
                                      uint16_t length);
    int  (*check_interrupt_event)(uint8_t addr, uint8_t ep_num);
    bool (*submit_interrupt_in_sync)(uint8_t addr, uint8_t endpoint,
                                      uint16_t max_packet_size,
                                      void *data, uint16_t length);
    bool (*poll_hotplug)(void);
} driver_usb_t;

/*
 * bus_device_t.bus_context for a DEVICE_TYPE_USB bus_device_t (added in
 * API 2.2, alongside BusRegistry). USB_Main.c builds one of these on its
 * stack per enumerated interface and passes it (via bus_device_t) into
 * bus_registry_report_device() -> a matched device driver's probe(); the
 * driver copies out what it needs (submit_bulk included, since a
 * standalone USB device driver module is a separate .ELF from USB_Main.c
 * and cannot call its usb_submit_bulk() directly the way code compiled
 * into the same USB_Driver.ELF can) before probe() returns -- the pointer
 * itself is only valid for the duration of that call.
 */
typedef struct {
    uint8_t  addr;
    uint8_t  interface;
    uint8_t  ep_in;
    uint8_t  ep_out;
    uint16_t ep_in_mps;
    uint16_t ep_out_mps;
    bool (*submit_bulk)(uint8_t addr, uint8_t endpoint, uint16_t max_packet_size,
                        uint8_t pid, void *data, uint32_t length);
} usb_device_context_t;

typedef struct {
    void (*init)(uint32_t baud);
    void (*write_char)(char c);
    void (*write_string)(const char *str);
    int  (*read_char)(void);
    void (*write_uint32)(uint32_t val);
    void (*write_uint64)(uint64_t val);
    void (*write_dec16)(uint16_t val);
    uint32_t (*copy_log)(char *buffer, uint32_t buffer_size);
    void (*enable_file_logging)(const char *path);
} driver_serial_t;

typedef struct {
    driver_input_t   input;
    driver_storage_t storage;
    driver_usb_t     usb;
} usb_master_vtable_t;

typedef struct {
    void *addr;
    uint32_t size_bytes;
    uint32_t width;
    uint32_t height;
    uint32_t pixels_per_scan_line;
    uint32_t bytes_per_pixel;
} driver_boot_framebuffer_t;

#define DISPLAY_MAX_MONITORS 16u
#define DISPLAY_MAX_MONITOR_NAME 32u

#define DISPLAY_OUTPUT_UNKNOWN        0u
#define DISPLAY_OUTPUT_FRAMEBUFFER    1u
#define DISPLAY_OUTPUT_VIRTIO_SCANOUT 2u
#define DISPLAY_OUTPUT_HDMI           3u
#define DISPLAY_OUTPUT_DISPLAYPORT    4u
#define DISPLAY_OUTPUT_VGA            5u

#define DISPLAY_MONITOR_FLAG_CONNECTED      (1u << 0)
#define DISPLAY_MONITOR_FLAG_PRIMARY        (1u << 1)
#define DISPLAY_MONITOR_FLAG_MODESET        (1u << 2)
#define DISPLAY_MONITOR_FLAG_HOTPLUG        (1u << 3)
#define DISPLAY_MONITOR_FLAG_SYNTHETIC_MODE (1u << 4)

#define DISPLAY_MODE_FLAG_CURRENT   (1u << 0)
#define DISPLAY_MODE_FLAG_PREFERRED (1u << 1)
#define DISPLAY_MODE_FLAG_SYNTHETIC (1u << 2)

typedef struct __attribute__((packed)) {
    uint32_t generation;
    uint32_t monitor_count;
    uint32_t primary_monitor;
    uint32_t flags;
    int32_t  origin_x;
    int32_t  origin_y;
    uint32_t width;
    uint32_t height;
} display_topology_t;

typedef struct __attribute__((packed)) {
    uint32_t index;
    uint32_t id;
    uint32_t flags;
    uint32_t output_type;
    int32_t  x;
    int32_t  y;
    uint32_t width;
    uint32_t height;
    uint32_t physical_width_mm;
    uint32_t physical_height_mm;
    uint32_t refresh_millihz;
    uint32_t current_mode;
    uint32_t mode_count;
    uint32_t generation;
    char     name[DISPLAY_MAX_MONITOR_NAME];
    char     output_name[DISPLAY_MAX_MONITOR_NAME];
} display_monitor_info_t;

typedef struct __attribute__((packed)) {
    uint32_t monitor_index;
    uint32_t mode_index;
    uint32_t flags;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t bits_per_pixel;
    uint32_t refresh_millihz;
    char     name[DISPLAY_MAX_MONITOR_NAME];
} display_mode_info_t;

typedef struct __attribute__((packed)) {
    int32_t  x;
    int32_t  y;
    uint32_t w;
    uint32_t h;
} display_rect_t;

typedef struct {
    const char *name;
    bool (*probe)(void);
    bool (*init)(void);
    bool (*is_ready)(void);
    uint32_t (*width)(void);
    uint32_t (*height)(void);
    void (*draw_pixel)(uint32_t x, uint32_t y, uint32_t color);
    void (*fill_rect)(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color);
    void (*present)(void);
    void (*present_rects)(const display_rect_t *rects, uint32_t count);
    bool (*set_framebuffer)(const driver_boot_framebuffer_t *framebuffer);
    void *(*get_framebuffer)(void);
    uint32_t (*get_generation)(void);
    bool (*poll_config)(void);
    uint32_t (*get_monitor_count)(void);
    bool (*get_topology)(display_topology_t *out_topology);
    bool (*get_monitor_info)(uint32_t monitor_index,
                             display_monitor_info_t *out_info);
    bool (*get_mode_info)(uint32_t monitor_index,
                          uint32_t mode_index,
                          display_mode_info_t *out_info);
    bool (*set_mode)(uint32_t monitor_index, uint32_t mode_index);
} driver_display_t;

#define DRIVER_KBD_MOD_SHIFT (1u << 0)
#define DRIVER_KBD_MOD_CTRL  (1u << 1)
#define DRIVER_KBD_MOD_ALT   (1u << 2)
#define DRIVER_KBD_MOD_CAPS  (1u << 3)

typedef void (*driver_nic_rx_callback_t)(const uint8_t *frame, uint16_t frame_len);

/*
 * Wi-Fi management plane (added alongside AX900.c, the first driver
 * module that needs it). Deliberately kept optional/additive on
 * driver_nic_t rather than a separate descriptor kind: a NIC that only
 * does the generic send/recv/poll surface (VirtIONet, I219V, ...) simply
 * leaves these fields NULL, and NicManager/DriverManager below guard every
 * call with a NULL check exactly the way the rest of driver_nic_t already
 * does. This is what lets a wifi_* syscall reach a specific driver module
 * (its own .ELF/address space) through the same DeviceRegistry ->
 * DriverManager -> NicManager -> NIC.c chain the generic NIC calls use.
 */
typedef enum {
    DRIVER_WIFI_SECURITY_UNKNOWN = 0,
    DRIVER_WIFI_SECURITY_OPEN,
    DRIVER_WIFI_SECURITY_WPA_PSK,   /* WPA/WPA2/WPA3-personal, RSN or WPA1 IE seen */
} driver_wifi_security_t;

#define DRIVER_WIFI_SSID_MAX 32u

/* __attribute__((packed)) + a plain uint32_t (rather than the enum type
 * above) for `security`/`state` below: these two structs cross the
 * kernel/userland boundary as a raw byte copy (SYSCALL_WIFI_*, see
 * Syscall_Dispatch.c and Userland/API/WiFi.h's mirrored definitions) --
 * pinning the layout removes any dependence on the two independently
 * built freestanding binaries happening to choose the same enum
 * underlying type or struct padding. */
typedef struct __attribute__((packed)) {
    char ssid[DRIVER_WIFI_SSID_MAX + 1u];
    uint8_t bssid[6];
    int8_t  rssi_dbm;
    uint32_t security; /* driver_wifi_security_t */
} driver_wifi_scan_result_t;

typedef enum {
    DRIVER_WIFI_STATE_NO_ADAPTER = 0,
    DRIVER_WIFI_STATE_ADAPTER_ATTACHED,   /* USB device present, firmware not loaded yet */
    DRIVER_WIFI_STATE_FIRMWARE_LOADING,
    DRIVER_WIFI_STATE_FIRMWARE_FAILED,    /* fw files missing/rejected -- see AX900.c */
    DRIVER_WIFI_STATE_READY,              /* firmware up, idle */
    DRIVER_WIFI_STATE_SCANNING,
    DRIVER_WIFI_STATE_CONNECTING,
    DRIVER_WIFI_STATE_ASSOCIATED,
    DRIVER_WIFI_STATE_CONNECT_FAILED,
} driver_wifi_state_t;

typedef struct __attribute__((packed)) {
    uint32_t state; /* driver_wifi_state_t */
    char ssid[DRIVER_WIFI_SSID_MAX + 1u]; /* valid when state >= CONNECTING */
    uint8_t mac[6];
} driver_wifi_status_t;

typedef struct {
    bool (*init)(void);
    bool (*is_ready)(void);
    uint16_t (*mtu)(void);
    void (*get_mac)(uint8_t mac_out[6]);
    bool (*send_frame)(const uint8_t *frame, uint16_t frame_len);
    void (*poll)(void);
    void (*set_rx_callback)(driver_nic_rx_callback_t cb);

    /* ---- Optional Wi-Fi management plane (NULL on wired NICs) ---- */
    bool (*wifi_scan_start)(void);
    uint32_t (*wifi_get_scan_results)(driver_wifi_scan_result_t *out, uint32_t max_count);
    bool (*wifi_connect)(const char *ssid, const char *psk);
    void (*wifi_disconnect)(void);
    void (*wifi_get_status)(driver_wifi_status_t *out_status);
} driver_nic_t;

/*
 * Declarative bus-device binding, added in API 2.2 alongside
 * Kernel/Drivers/Module/BusRegistry.h. Lets a device driver module say "I
 * handle this bus device" instead of scanning its bus itself the way
 * VirtIONet.c/I219V.c still do -- see driver_module_descriptor_t.bus_matches
 * below. Entirely optional/additive: a descriptor with bus_match_count == 0
 * behaves exactly as before this field existed.
 */
#define DRIVER_BUS_MATCH_VENDOR   (1u << 0)
#define DRIVER_BUS_MATCH_DEVICE   (1u << 1)
#define DRIVER_BUS_MATCH_CLASS    (1u << 2)
#define DRIVER_BUS_MATCH_SUBCLASS (1u << 3)
#define DRIVER_BUS_MATCH_PROTOCOL (1u << 4)

typedef struct {
    device_type_t bus_type;    /* DEVICE_TYPE_PCI or DEVICE_TYPE_USB */
    uint16_t vendor_id;
    uint16_t device_id;        /* PCI device id, or USB PID */
    uint8_t  class_code;       /* PCI class, or USB interface class */
    uint8_t  subclass;
    uint8_t  protocol;
    uint8_t  match_flags;      /* DRIVER_BUS_MATCH_* bitmask: which fields above must match */
} driver_bus_match_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    device_type_t kind;
    uint32_t load_priority;
    const char *deps[DRIVER_MAX_DEPS];
    const void *driver_api;
    void (*shutdown)(void);

    /* Optional (API 2.2): see driver_bus_match_t above. All-zero
     * (bus_match_count == 0, probe == NULL) means "not using this" --
     * existing drivers that scan their bus themselves are unaffected. */
    const driver_bus_match_t *bus_matches;
    uint32_t bus_match_count;
    bool (*probe)(const bus_device_t *dev);
    void (*remove)(const bus_device_t *dev);
} driver_module_descriptor_t;

typedef const driver_module_descriptor_t *(*driver_module_init_fn_t)(const driver_binary_t *api);
