#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "interfaces/hal_io.h"

static inline void outsw(uint16_t port, const void *addr, int count) {
    hal_io_outsw(port, addr, (uint32_t)count);
}

static inline void outw(uint16_t port, uint16_t val) {
    hal_io_out16(port, val);
}

static inline void outb(uint16_t port, uint8_t val) {
    hal_io_out8(port, val);
}

static inline uint8_t inb(uint16_t port) {
    return hal_io_in8(port);
}

static inline uint16_t inw(uint16_t port) {
    return hal_io_in16(port);
}

static inline void outl(uint16_t port, uint32_t val) {
    hal_io_out32(port, val);
}

static inline uint32_t inl(uint16_t port) {
    return hal_io_in32(port);
}
 
typedef enum {
    IO_PROTOCOL_TYPE_NONE,
    IO_PROTOCOL_TYPE_ATA,
    IO_PROTOCOL_TYPE_AHCI,
    IO_PROTOCOL_TYPE_USB_MASS_STORAGE,
    IO_PROTOCOL_TYPE_NVME,
    IO_PROTOCOL_TYPE_VIRTIO_BLOCK
} io_protocol_type_t;

typedef struct {
    const char *name;
    const char *model;
    io_protocol_type_t protocol;
    bool (*init)(uint64_t partition_lba);
    bool (*read)(uint64_t lba, uint8_t *buffer, uint32_t sectors);
    bool (*write)(uint64_t lba, const uint8_t *buffer, uint32_t sectors);
    bool (*is_working)(void);
    uint32_t (*get_device_count)(void);
    bool (*select_device)(uint32_t index);
    uint64_t (*get_total_bytes)(void);
} block_device_t;

typedef struct {
    char disk_name[32];
    char manufacturer[64];
    char model[64];
    io_protocol_type_t protocol;
    uint64_t total_bytes;
    uint32_t sector_size;
    uint32_t flags;
} io_disk_info_t;

#define IO_DISK_FLAG_BOOT     (1u << 0)
#define IO_DISK_FLAG_WRITABLE (1u << 1)

bool disk_io_init(uint64_t partition_lba, uint32_t boot_drive_type);

bool disk_read(uint64_t lba, uint8_t *buffer, uint32_t sectors);
bool disk_write(uint64_t lba, const uint8_t *buffer, uint32_t sectors);
bool disk_io_is_working(void);
io_protocol_type_t disk_io_get_protocol(void);
uint64_t disk_get_partition_lba(void);
uint32_t disk_get_count(void);
bool disk_get_info(uint32_t index, io_disk_info_t *out_info);
bool disk_raw_read(uint32_t index, uint64_t lba, uint8_t *buffer, uint32_t sectors);
bool disk_raw_write(uint32_t index, uint64_t lba, const uint8_t *buffer, uint32_t sectors);
