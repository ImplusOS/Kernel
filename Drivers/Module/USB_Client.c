#include "USB_Driver_API.h"
#include "Drivers/Module/DriverBinary.h"
#include "Drivers/Module/DriverManager.h"
#include "Debug/serial/Serial.h"

static const usb_master_vtable_t *g_usb_vtable = NULL;
static volatile uint32_t g_usb_poll_pending = 0;
static uint32_t g_usb_storage_index = 0;

static bool usb_driver_client_refresh(void)
{
    const device_t *device = driver_manager_find(DEVICE_TYPE_USB, NULL);
    g_usb_vtable = device ? (const usb_master_vtable_t *)device->ops : NULL;
    return (g_usb_vtable != NULL);
}

void usb_driver_client_init(void)
{
    if (!usb_driver_client_refresh()) {
        return;
    }

    if (g_usb_vtable->input.init) {
        g_usb_vtable->input.init();
    }
}

bool usb_driver_client_read_sectors(uint64_t lba, uint8_t *buffer, uint32_t sectors)
{
    if (!usb_driver_client_refresh()) {
        return false;
    }
    if (g_usb_vtable == NULL || g_usb_vtable->storage.read_blocks == NULL) {
        return false;
    }
    driver_block_info_t info;
    if (!g_usb_vtable->storage.get_info(g_usb_storage_index, &info) ||
        info.logical_block_size < 512u ||
        (info.logical_block_size % 512u) != 0u) {
        return false;
    }
    uint32_t factor = info.logical_block_size / 512u;
    if (factor != 1u || lba > UINT64_MAX / 512u) {
        return false;
    }
    return g_usb_vtable->storage.read_blocks(g_usb_storage_index, lba,
                                              buffer, sectors);
}

bool usb_driver_client_write_sectors(uint64_t lba, const uint8_t *buffer, uint32_t sectors)
{
    if (!usb_driver_client_refresh()) {
        return false;
    }
    if (g_usb_vtable == NULL || g_usb_vtable->storage.write_blocks == NULL) {
        return false;
    }
    driver_block_info_t info;
    if (!g_usb_vtable->storage.get_info(g_usb_storage_index, &info) ||
        info.logical_block_size != 512u) {
        return false;
    }
    return g_usb_vtable->storage.write_blocks(g_usb_storage_index, lba,
                                               buffer, sectors);
}

uint32_t usb_driver_client_get_device_count(void)
{
    if (!usb_driver_client_refresh()) return 0;
    if (g_usb_vtable == NULL || g_usb_vtable->storage.get_device_count == NULL) return 0;
    return g_usb_vtable->storage.get_device_count();
}

bool usb_driver_client_select_device(uint32_t index)
{
    if (!usb_driver_client_refresh()) return false;
    if (g_usb_vtable == NULL ||
        index >= g_usb_vtable->storage.get_device_count()) return false;
    g_usb_storage_index = index;
    return true;
}

uint64_t usb_driver_client_get_total_bytes(void)
{
    if (!usb_driver_client_refresh()) return 0;
    if (g_usb_vtable == NULL || g_usb_vtable->storage.get_info == NULL) return 0;
    driver_block_info_t info;
    if (!g_usb_vtable->storage.get_info(g_usb_storage_index, &info)) return 0;
    return info.block_count * info.logical_block_size;
}

uint32_t usb_driver_client_get_block_size(void)
{
    if (!usb_driver_client_refresh() ||
        g_usb_vtable->storage.get_info == NULL) return 0u;
    driver_block_info_t info;
    if (!g_usb_vtable->storage.get_info(g_usb_storage_index, &info)) return 0u;
    return info.logical_block_size;
}

int32_t usb_driver_client_read_keyboard(driver_keyboard_event_t *out_event)
{
    if (!usb_driver_client_refresh()) {
        return 0;
    }
    if (g_usb_vtable == NULL) {
        return 0;
    }
    if (g_usb_vtable->input.read_keyboard == NULL) {
        return 0;
    }
    return g_usb_vtable->input.read_keyboard(out_event);
}

int32_t usb_driver_client_read_mouse(driver_mouse_event_t *out_event)
{
    if (!usb_driver_client_refresh()) {
        return 0;
    }
    if (g_usb_vtable == NULL || g_usb_vtable->input.read_mouse == NULL) {
        return 0;
    }
    return g_usb_vtable->input.read_mouse(out_event);
}

void usb_driver_client_poll(void)
{
    if (!usb_driver_client_refresh()) {
        return;
    }
    if (g_usb_vtable == NULL || g_usb_vtable->input.poll == NULL) {
        return;
    }
    g_usb_vtable->input.poll();
}

void usb_driver_client_drain_keyboard(driver_keyboard_event_t *tmp,
                                      void (*forward)(driver_keyboard_event_t *))
{
    if (!usb_driver_client_refresh()) {
        return;
    }
    if (g_usb_vtable == NULL || g_usb_vtable->input.drain_keyboard == NULL) {
        return;
    }
    g_usb_vtable->input.drain_keyboard(tmp, forward);
}

void usb_driver_client_drain_mouse(driver_mouse_event_t *tmp,
                                   void (*forward)(driver_mouse_event_t *))
{
    if (!usb_driver_client_refresh()) {
        return;
    }
    if (g_usb_vtable == NULL || g_usb_vtable->input.drain_mouse == NULL) {
        return;
    }
    g_usb_vtable->input.drain_mouse(tmp, forward);
}

void usb_driver_client_schedule_poll(void)
{
    __atomic_store_n(&g_usb_poll_pending, 1u, __ATOMIC_RELEASE);
}

bool usb_driver_client_check_poll(void)
{
    if (__atomic_exchange_n(&g_usb_poll_pending, 0u, __ATOMIC_ACQ_REL)) {
        return true;
    }
    return false;
}
