#pragma once

#include "DriverBinary.h"

#include <stdbool.h>
#include <stdint.h>

void serial_manager_init(void);
void serial_manager_write_char(char c);
void serial_manager_write_string(const char *str);
void serial_manager_write_uint32(uint32_t val);
void serial_manager_write_uint64(uint64_t val);
void serial_manager_write_dec16(uint16_t val);
int serial_manager_read_char(void);
uint32_t serial_manager_copy_log(char *buffer, uint32_t buffer_size);
void serial_manager_enable_file_logging(const char *path);
