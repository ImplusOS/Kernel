#include "SerialManager.h"
#include "DeviceRegistry.h"

static const driver_serial_t *g_driver = NULL;

void serial_manager_init(void)
{
    g_driver = NULL;

    const device_t *dev = device_registry_find(DEVICE_TYPE_SERIAL, NULL);
    if (dev != NULL && dev->ops != NULL) {
        g_driver = (const driver_serial_t *)dev->ops;
    }
}

void serial_manager_write_char(char c)
{
    if (g_driver != NULL && g_driver->write_char != NULL) {
        g_driver->write_char(c);
    }
}

void serial_manager_write_string(const char *str)
{
    if (g_driver != NULL && g_driver->write_string != NULL) {
        g_driver->write_string(str);
    }
}

void serial_manager_write_uint32(uint32_t val)
{
    if (g_driver != NULL && g_driver->write_uint32 != NULL) {
        g_driver->write_uint32(val);
    }
}

void serial_manager_write_uint64(uint64_t val)
{
    if (g_driver != NULL && g_driver->write_uint64 != NULL) {
        g_driver->write_uint64(val);
    }
}

void serial_manager_write_dec16(uint16_t val)
{
    if (g_driver != NULL && g_driver->write_dec16 != NULL) {
        g_driver->write_dec16(val);
    }
}

int serial_manager_read_char(void)
{
    if (g_driver != NULL && g_driver->read_char != NULL) {
        return g_driver->read_char();
    }
    return -1;
}

uint32_t serial_manager_copy_log(char *buffer, uint32_t buffer_size)
{
    if (g_driver != NULL && g_driver->copy_log != NULL) {
        return g_driver->copy_log(buffer, buffer_size);
    }
    return 0u;
}

void serial_manager_enable_file_logging(const char *path)
{
    if (g_driver != NULL && g_driver->enable_file_logging != NULL) {
        g_driver->enable_file_logging(path);
    }
}
