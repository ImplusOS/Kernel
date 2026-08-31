#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

typedef void (*serial_mirror_char_t)(char c);

typedef struct {
    void (*write_char)(char c);
    void (*write_string)(const char *str);
} serial_backend_t;

void serial_init(void);
void serial_set_screen_mirror(serial_mirror_char_t writer);
void serial_register_backend(const serial_backend_t *backend);
void serial_write_char(char c);
void serial_write_string(const char *str);
void serial_write_uint64(uint64_t value);
void serial_write_uint32(uint32_t value);
void serial_write_uint16(uint16_t value);
void serial_write_uint8(uint8_t value);
void serial_write_dec16(uint16_t v);
void serial_enable_file_logging(const char *path);
uint32_t serial_copy_log(char *buffer, uint32_t buffer_size);

#endif
