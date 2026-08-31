#pragma once

#include "DriverBinary.h"
#include "kernel/boot_info.h"

#include <stdbool.h>
#include <stdint.h>

bool block_manager_init(void);
void block_manager_set_boot_identity(const BOOT_INFO *boot_info);
uint32_t block_manager_get_device_count(void);
bool block_manager_select_device(uint32_t index);
uint32_t block_manager_selected_device(void);
bool block_manager_get_info(uint32_t index, driver_block_info_t *out_info);
const char *block_manager_get_name(uint32_t index);

bool block_manager_read_blocks(uint32_t index, uint64_t lba, void *buffer,
                               uint32_t block_count);
bool block_manager_write_blocks(uint32_t index, uint64_t lba,
                                const void *buffer, uint32_t block_count);
bool block_manager_flush(uint32_t index);

bool block_manager_read_sectors(uint64_t lba, uint8_t *buffer,
                                uint32_t sectors);
bool block_manager_write_sectors(uint64_t lba, const uint8_t *buffer,
                                 uint32_t sectors);
