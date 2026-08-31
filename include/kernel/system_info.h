#pragma once

#include <stdint.h>
#include <stddef.h>
#include "kernel/config.h"

typedef struct __attribute__((packed)) {
    char vendor[16];
    char brand[64];
    uint32_t logical_cores;
    uint32_t physical_cores;
    uint32_t frequency_mhz;
    uint32_t cache_l1d_kb;
    uint32_t cache_l1i_kb;
    uint32_t cache_l2_kb;
    uint32_t cache_l3_kb;
    uint32_t stepping;
    uint32_t model;
    uint32_t family;
} system_cpu_info_t;

typedef struct __attribute__((packed)) {
    uint64_t total_bytes;
    uint64_t used_bytes;
    uint64_t free_bytes;
    uint64_t cached_bytes;
    uint64_t buffers_bytes;
    uint32_t page_size;
} system_memory_info_t;

typedef struct __attribute__((packed)) {
    uint32_t page_size;
    uint64_t total_pages;
    uint64_t free_pages;
    uint64_t mapped_pages;
} system_vmem_info_t;

typedef struct __attribute__((packed)) {
    char disk_name[32];
    uint64_t total_bytes;
    uint64_t used_bytes;
    uint64_t free_bytes;
    char manufacturer[64];
    char model[64];
    uint32_t sector_size;
    uint32_t protocol;
    uint32_t flags;
} system_disk_info_t;

typedef enum {
    SYSTEM_DEVICE_UNKNOWN = 0,
    SYSTEM_DEVICE_ATA_CONTROLLER,
    SYSTEM_DEVICE_USB_CONTROLLER,
    SYSTEM_DEVICE_USB_HUB,
    SYSTEM_DEVICE_USB_KEYBOARD,
    SYSTEM_DEVICE_USB_MOUSE,
    SYSTEM_DEVICE_USB_MASS_STORAGE,
    SYSTEM_DEVICE_PS2_KEYBOARD,
    SYSTEM_DEVICE_PS2_MOUSE,
    SYSTEM_DEVICE_NETWORK_ADAPTER,
    SYSTEM_DEVICE_AUDIO_DEVICE,
    SYSTEM_DEVICE_GRAPHICS_ADAPTER,
    SYSTEM_DEVICE_PCI_BRIDGE,
    SYSTEM_DEVICE_STORAGE_CONTROLLER,
} system_device_type_t;

typedef struct __attribute__((packed)) {
    system_device_type_t type;
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t bus;
    uint8_t device;
    uint8_t func;
    uint8_t reserved;
    char vendor_name[64];
    char device_name[128];
    uint32_t irq;
    uint32_t flags;
} system_device_t;

typedef struct __attribute__((packed)) {
    char vendor[64];
    char model[128];
    uint32_t vram_mb;
    uint16_t vendor_id;
    uint16_t device_id;
    uint32_t display_width;
    uint32_t display_height;
    uint32_t bits_per_pixel;
} system_graphics_info_t;

typedef struct __attribute__((packed)) {
    uint8_t bits;
    uint8_t endianness;
    uint8_t reserved[6];
    char name[32];
} system_arch_info_t;

typedef struct __attribute__((packed)) {
    system_arch_info_t arch;
    system_cpu_info_t cpu;
    system_memory_info_t memory;
    system_vmem_info_t vmem;
} system_info_t;

typedef struct __attribute__((packed)) {
    uint32_t cpu_count;
    uint32_t reserved;
    uint64_t timestamp_ns;
    uint64_t wall_ns;
    uint64_t idle_ns[OS_CONFIG_SMP_MAX_CPUS];
} system_cpu_usage_t;
