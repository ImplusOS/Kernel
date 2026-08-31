#pragma once
#include <stdbool.h>
#include <stdint.h>

bool ahci_init(uint64_t partition_lba);
bool ahci_read(uint64_t lba, uint8_t *buffer, uint32_t sectors);
bool ahci_write(uint64_t lba, const uint8_t *buffer, uint32_t sectors);
bool ahci_flush(void);
bool ahci_is_working(void);
uint32_t ahci_get_device_count(void);
bool ahci_select_device(uint32_t index);
uint64_t ahci_get_total_bytes(void);
