#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool (*init)(void);
    bool (*find_file)(const char *path, FAT32_FILE *file);
    bool (*read_file)(FAT32_FILE *file, uint8_t *buffer);
    bool (*write_file)(FAT32_FILE *file, const uint8_t *buffer);
} filesystem_ops_t;
