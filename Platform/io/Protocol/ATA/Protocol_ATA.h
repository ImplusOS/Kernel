#pragma once
#include <stdbool.h>
#include <stdint.h>

bool ata_init(uint64_t partition_lba);
bool ata_read(uint64_t lba, uint8_t *buffer, uint32_t sectors);
bool ata_write(uint64_t lba, const uint8_t *buffer, uint32_t sectors);
bool ata_is_working(void);
uint32_t ata_get_device_count(void);
bool ata_select_device(uint32_t index);
uint64_t ata_get_total_bytes(void);
