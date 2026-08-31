#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"
#endif

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Debug/serial/Serial.h"
#include "Core/vfs/VFS.h"

#define SERIAL_LOG_BUF_SIZE 8192

static const driver_binary_t *g_driver_api = NULL;

static serial_mirror_char_t g_screen_mirror = NULL;
static volatile uint8_t g_screen_mirror_active = 0;

static char g_log_buf[SERIAL_LOG_BUF_SIZE];
static uint32_t g_log_buf_pos = 0;
static bool g_log_buf_wrapped = false;
static bool g_log_buf_active = true;

static vfs_file_t g_log_file;
static bool g_log_file_open = false;
static uint32_t g_log_file_offset = 0;
static volatile uint8_t g_log_file_writing = 0;

static void module_mirror_char(char c)
{
    if (g_screen_mirror == NULL || g_screen_mirror_active != 0) {
        return;
    }
    g_screen_mirror_active = 1;
    g_screen_mirror(c);
    g_screen_mirror_active = 0;
}

static void module_mirror_string(const char *str)
{
    while (*str) {
        module_mirror_char(*str++);
    }
}

static void module_log_char(char c)
{
    if (g_log_buf_active) {
        g_log_buf[g_log_buf_pos] = c;
        g_log_buf_pos = (g_log_buf_pos + 1u) % SERIAL_LOG_BUF_SIZE;
        if (g_log_buf_pos == 0u) {
            g_log_buf_wrapped = true;
        }
    }

    if (g_log_file_open && g_log_file_writing == 0) {
        g_log_file_writing = 1;
        if (g_driver_api != NULL && g_driver_api->hw.disk_read != NULL) {
            vfs_write_at(&g_log_file, g_log_file_offset, (const uint8_t *)&c, 1u);
        }
        g_log_file_offset += 1u;
        g_log_file_writing = 0;
    }
}

static void module_backend_write_char(char c)
{
    serial_write_char(c);
    module_mirror_char(c);
    module_log_char(c);
}

static void module_backend_write_string(const char *str)
{
    if (!str) {
        return;
    }

    serial_write_string(str);
    module_mirror_string(str);

    if (g_log_buf_active) {
        const char *p = str;
        while (*p) {
            g_log_buf[g_log_buf_pos] = *p++;
            g_log_buf_pos = (g_log_buf_pos + 1u) % SERIAL_LOG_BUF_SIZE;
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

static void serial_drv_init(uint32_t baud)
{
    (void)baud;
}

static void serial_drv_write_char(char c)
{
    module_backend_write_char(c);
}

static void serial_drv_write_string(const char *str)
{
    if (!str) {
        return;
    }
    module_backend_write_string(str);
}

static int serial_drv_read_char(void)
{
    return -1;
}

static void serial_drv_write_hex(uint64_t value, uint8_t nibbles)
{
    static const char hex[] = "0123456789ABCDEF";
    serial_drv_write_string("0x");
    for (int i = (int)((nibbles - 1u) * 4u); i >= 0; i -= 4) {
        serial_drv_write_char(hex[(value >> i) & 0xF]);
    }
}

static void serial_drv_write_uint32(uint32_t val)
{
    serial_drv_write_hex(val, 8u);
}

static void serial_drv_write_uint64(uint64_t val)
{
    serial_drv_write_hex(val, 16u);
}

static void serial_drv_write_dec16(uint16_t v)
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
    serial_drv_write_string(&buf[i]);
}

static uint32_t serial_drv_copy_log(char *buffer, uint32_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0u) {
        return 0u;
    }

    uint32_t available = g_log_buf_wrapped ? SERIAL_LOG_BUF_SIZE : g_log_buf_pos;
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
        start = (g_log_buf_pos + (SERIAL_LOG_BUF_SIZE - copy_len)) % SERIAL_LOG_BUF_SIZE;
    } else if (available > copy_len) {
        start = available - copy_len;
    }

    for (uint32_t i = 0u; i < copy_len; ++i) {
        buffer[i] = g_log_buf[(start + i) % SERIAL_LOG_BUF_SIZE];
    }
    buffer[copy_len] = '\0';
    return copy_len;
}

static void serial_drv_enable_file_logging(const char *path)
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

    uint32_t size = g_log_buf_wrapped ? SERIAL_LOG_BUF_SIZE : g_log_buf_pos;
    g_log_file_offset = 0u;
    if (size > 0u) {
        if (!g_log_buf_wrapped) {
            vfs_write_at(&g_log_file, 0u, (const uint8_t *)g_log_buf, size);
            g_log_file_offset = size;
        } else {
            uint32_t chunk1 = SERIAL_LOG_BUF_SIZE - g_log_buf_pos;
            vfs_write_at(&g_log_file, 0u,
                         (const uint8_t *)&g_log_buf[g_log_buf_pos], chunk1);
            vfs_write_at(&g_log_file, chunk1,
                         (const uint8_t *)g_log_buf, g_log_buf_pos);
            g_log_file_offset = chunk1 + g_log_buf_pos;
        }
    }

    g_log_file_open = true;
}

#ifdef IMPLUS_DRIVER_MODULE

static const serial_backend_t g_serial_backend = {
    .write_char = module_backend_write_char,
    .write_string = module_backend_write_string,
};

static const driver_serial_t g_serial_driver = {
    .init = serial_drv_init,
    .write_char = serial_drv_write_char,
    .write_string = serial_drv_write_string,
    .read_char = serial_drv_read_char,
    .write_uint32 = serial_drv_write_uint32,
    .write_uint64 = serial_drv_write_uint64,
    .write_dec16 = serial_drv_write_dec16,
    .copy_log = serial_drv_copy_log,
    .enable_file_logging = serial_drv_enable_file_logging,
};

static void serial_driver_shutdown(void)
{
    serial_register_backend(NULL);
    g_driver_api = NULL;
    g_screen_mirror = NULL;
    g_screen_mirror_active = 0;
    g_log_buf_active = true;
    g_log_buf_pos = 0;
    g_log_buf_wrapped = false;
    g_log_file_open = false;
    g_log_file_offset = 0;
    g_log_file_writing = 0;
}

static const driver_module_descriptor_t g_serial_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_SERIAL,
    .load_priority = 5u,
    .deps = { NULL },
    .driver_api = &g_serial_driver,
    .shutdown = serial_driver_shutdown,
};

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL) {
        return NULL;
    }

    g_driver_api = api;

    serial_register_backend(&g_serial_backend);

    return &g_serial_module;
}

#endif
