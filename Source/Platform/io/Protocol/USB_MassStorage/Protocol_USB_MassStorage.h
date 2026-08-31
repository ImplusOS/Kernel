#pragma once
#include <stdbool.h>
#include <stdint.h>

bool usb_ms_init(uint64_t partition_lba);
bool usb_ms_read(uint64_t lba, uint8_t *buffer, uint32_t sectors);
bool usb_ms_write(uint64_t lba, const uint8_t *buffer, uint32_t sectors);
bool usb_ms_is_working(void);
uint32_t usb_ms_get_device_count(void);
bool usb_ms_select_device(uint32_t index);
uint64_t usb_ms_get_total_bytes(void);
