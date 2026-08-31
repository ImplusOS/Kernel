#include "Serial.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "interfaces/uart_hal.h"
#include "Core/vfs/VFS.h"

/* In-RAM scrollback for the on-screen log viewer / panic screen. 8 KiB was too
 * small to keep a verbose Chromium abort (the [FATAL:...] line plus its stack)
 * from scrolling out before it could be read. */
#define LOG_BUF_SIZE 65536

static const uart_hal_t *g_uart = NULL;
static const serial_backend_t *g_backend = NULL;
static serial_mirror_char_t g_screen_mirror = NULL;
static volatile uint8_t g_screen_mirror_active = 0;

static char g_log_buf[LOG_BUF_SIZE];
static uint32_t g_log_buf_pos = 0;
static bool g_log_buf_wrapped = false;
static bool g_log_buf_active = true;

static vfs_file_t g_log_file;
static bool g_log_file_open = false;
static uint32_t g_log_file_offset = 0;
static volatile uint8_t g_log_file_writing = 0;

static void serial_backend_init(void)
{
    if (g_uart == NULL) {
        g_uart = uart_hal_get();
    }
}

void serial_init(void)
{
    serial_backend_init();
    if (g_uart && g_uart->init) {
        g_uart->init(115200);
    }
}

void serial_register_backend(const serial_backend_t *backend)
{
    g_backend = backend;
}

void serial_set_screen_mirror(serial_mirror_char_t writer)
{
    g_screen_mirror = writer;
}

static void serial_mirror_char(char c)
{
    if (g_screen_mirror == NULL || g_screen_mirror_active != 0) {
        return;
    }

    g_screen_mirror_active = 1;
    g_screen_mirror(c);
    g_screen_mirror_active = 0;
}

static void serial_mirror_string(const char *str)
{
    while (*str) {
        serial_mirror_char(*str++);
    }
}

static void serial_write_char_to_uart(char c)
{
    serial_backend_init();
    if (g_uart && g_uart->write_char) {
        g_uart->write_char(c);
    }
}

static void serial_write_string_to_uart(const char *str)
{
    serial_backend_init();
    if (!g_uart) {
        return;
    }

    if (g_uart->write_string) {
        g_uart->write_string(str);
        return;
    }

    if (!g_uart->write_char) {
        return;
    }

    while (*str) {
        g_uart->write_char(*str++);
    }
}

static void serial_log_char(char c)
{
    if (g_log_buf_active) {
        g_log_buf[g_log_buf_pos] = c;
        g_log_buf_pos = (g_log_buf_pos + 1u) % LOG_BUF_SIZE;
        if (g_log_buf_pos == 0u) {
            g_log_buf_wrapped = true;
        }
    }

    if (g_log_file_open && g_log_file_writing == 0) {
        g_log_file_writing = 1;
        vfs_write_at(&g_log_file, g_log_file_offset, (const uint8_t *)&c, 1u);
        g_log_file_offset += 1u;
        g_log_file_writing = 0;
    }
}

void serial_write_char(char c)
{
    if (g_backend && g_backend->write_char) {
        g_backend->write_char(c);
    } else {
        serial_write_char_to_uart(c);
        serial_mirror_char(c);
        serial_log_char(c);
    }
}

void serial_write_string(const char *str)
{
    if (!str) {
        return;
    }

    if (g_backend && g_backend->write_string) {
        g_backend->write_string(str);
    } else {
        serial_write_string_to_uart(str);
        serial_mirror_string(str);

        if (g_log_buf_active) {
            const char *p = str;
            while (*p) {
                g_log_buf[g_log_buf_pos] = *p++;
                g_log_buf_pos = (g_log_buf_pos + 1u) % LOG_BUF_SIZE;
                if (g_log_buf_pos == 0u) {
                    g_log_buf_wrapped = true;
                }
            }
        }

        if (g_log_file_open && g_log_file_writing == 0) {
            g_log_file_writing = 1;
            uint32_t len = (uint32_t)strlen(str);
            vfs_write_at(&g_log_file, g_log_file_offset, (const uint8_t *)str, len);
            g_log_file_offset += len;
            g_log_file_writing = 0;
        }
    }
}

void serial_enable_file_logging(const char *path)
{
    if (!path) {
        return;
    }

    g_log_buf_active = false;

    vfs_creat(path);

    memset(&g_log_file, 0, sizeof(g_log_file));
    if (!vfs_find_file(path, &g_log_file)) {
        return;
    }

    uint32_t size = g_log_buf_wrapped ? LOG_BUF_SIZE : g_log_buf_pos;
    g_log_file_offset = 0u;
    if (size > 0u) {
        if (!g_log_buf_wrapped) {
            vfs_write_at(&g_log_file, 0u, (const uint8_t *)g_log_buf, size);
            g_log_file_offset = size;
        } else {
            uint32_t chunk1 = LOG_BUF_SIZE - g_log_buf_pos;
            vfs_write_at(&g_log_file, 0u,
                         (const uint8_t *)&g_log_buf[g_log_buf_pos], chunk1);
            vfs_write_at(&g_log_file, chunk1,
                         (const uint8_t *)g_log_buf, g_log_buf_pos);
            g_log_file_offset = chunk1 + g_log_buf_pos;
        }
    }

    g_log_file_open = true;
}

uint32_t serial_copy_log(char *buffer, uint32_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0u) {
        return 0u;
    }

    uint32_t available = g_log_buf_wrapped ? LOG_BUF_SIZE : g_log_buf_pos;
    uint32_t copy_len = available;
    if (copy_len + 1u > buffer_size) {
        copy_len = buffer_size - 1u;
    }

    if (copy_len == 0u) {
        buffer[0] = '\0';
        return 0u;
    }

    uint32_t start = 0u;
    if (g_log_buf_wrapped) {
        start = (g_log_buf_pos + (LOG_BUF_SIZE - copy_len)) % LOG_BUF_SIZE;
    } else if (available > copy_len) {
        start = available - copy_len;
    }

    for (uint32_t i = 0u; i < copy_len; ++i) {
        buffer[i] = g_log_buf[(start + i) % LOG_BUF_SIZE];
    }
    buffer[copy_len] = '\0';
    return copy_len;
}

static void serial_write_hex(uint64_t value, uint8_t nibbles)
{
    static const char hex[] = "0123456789ABCDEF";

    serial_write_string("0x");

    for (int i = (int)((nibbles - 1u) * 4u); i >= 0; i -= 4) {
        serial_write_char(hex[(value >> i) & 0xF]);
    }
}

void serial_write_uint64(uint64_t value)
{
    serial_write_hex(value, 16u);
}

void serial_write_uint32(uint32_t value)
{
    serial_write_hex(value, 8u);
}

void serial_write_uint16(uint16_t value)
{
    serial_write_hex(value, 4u);
}

void serial_write_uint8(uint8_t value)
{
    serial_write_hex(value, 2u);
}

void serial_write_dec16(uint16_t v)
{
    char buf[6];
    int i = (int)sizeof(buf);
    buf[--i] = '\0';

    if (v == 0) {
        buf[--i] = '0';
    } else {
        while (v > 0 && i > 0) {
            buf[--i] = (char)('0' + (v % 10));
            v /= 10;
        }
    }

    serial_write_string(&buf[i]);
}
