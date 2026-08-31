#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "Drivers/Module/DriverBinary.h"

bool bot_init(void);
void bot_reset_devices(void);
bool bot_read_sectors(uint32_t lba, uint8_t *buffer, uint32_t sectors);
bool bot_write_sectors(uint32_t lba, const uint8_t *buffer, uint32_t sectors);
void bot_add_device(uint8_t addr, uint8_t interface, uint8_t ep_in, uint8_t ep_out, uint16_t ep_in_mps, uint16_t ep_out_mps);
uint32_t bot_get_device_count(void);
bool bot_select_device(uint32_t index);
uint64_t bot_get_total_bytes(void);
uint32_t bot_get_block_size(void);
bool bot_is_read_only(void);
bool bot_flush(void);
