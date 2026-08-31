#include "NIC.h"

#include "Drivers/Module/DriverManager.h"
#include "Drivers/Module/DeviceRegistry.h"

static volatile uint32_t g_poll_pending = 0;
static const driver_nic_t *g_active_driver = NULL;

bool nic_init(void)
{
    if (g_active_driver != NULL) {
        return true;
    }

    for (uint32_t i = 0;; ++i) {
        const device_t *dev = device_registry_find_by_index(DEVICE_TYPE_NIC, i);
        if (dev == NULL) {
            break;
        }
        const driver_nic_t *driver = (const driver_nic_t *)dev->ops;
        if (driver != NULL && driver->init != NULL && driver->init()) {
            g_active_driver = driver;
            return true;
        }
    }

    return false;
}

bool nic_is_ready(void)
{
    if (g_active_driver == NULL || g_active_driver->is_ready == NULL) {
        return false;
    }
    return g_active_driver->is_ready();
}

uint16_t nic_mtu(void)
{
    if (g_active_driver == NULL || g_active_driver->mtu == NULL) {
        return 0;
    }
    return g_active_driver->mtu();
}

void nic_get_mac(uint8_t mac_out[6])
{
    if (g_active_driver == NULL || g_active_driver->get_mac == NULL) {
        return;
    }
    g_active_driver->get_mac(mac_out);
}

bool nic_send_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (g_active_driver == NULL || g_active_driver->send_frame == NULL) {
        return false;
    }
    return g_active_driver->send_frame(frame, frame_len);
}

void nic_poll(void)
{
    if (g_active_driver == NULL || g_active_driver->poll == NULL) {
        return;
    }
    g_active_driver->poll();
}

void nic_set_rx_callback(nic_rx_callback_t cb)
{
    if (g_active_driver == NULL || g_active_driver->set_rx_callback == NULL) {
        return;
    }
    g_active_driver->set_rx_callback(cb);
}

void nic_schedule_poll(void)
{
    __atomic_store_n(&g_poll_pending, 1u, __ATOMIC_RELEASE);
}

bool nic_check_poll(void)
{
    if (__atomic_exchange_n(&g_poll_pending, 0u, __ATOMIC_ACQ_REL) != 0u) {
        return true;
    }
    return false;
}

bool nic_wifi_scan_start(void)
{
    if (g_active_driver == NULL || g_active_driver->wifi_scan_start == NULL) {
        return false;
    }
    return g_active_driver->wifi_scan_start();
}

uint32_t nic_wifi_get_scan_results(driver_wifi_scan_result_t *out, uint32_t max_count)
{
    if (g_active_driver == NULL || g_active_driver->wifi_get_scan_results == NULL) {
        return 0u;
    }
    return g_active_driver->wifi_get_scan_results(out, max_count);
}

bool nic_wifi_connect(const char *ssid, const char *psk)
{
    if (g_active_driver == NULL || g_active_driver->wifi_connect == NULL) {
        return false;
    }
    return g_active_driver->wifi_connect(ssid, psk);
}

void nic_wifi_disconnect(void)
{
    if (g_active_driver == NULL || g_active_driver->wifi_disconnect == NULL) {
        return;
    }
    g_active_driver->wifi_disconnect();
}

void nic_wifi_get_status(driver_wifi_status_t *out_status)
{
    if (out_status == NULL) {
        return;
    }
    if (g_active_driver == NULL || g_active_driver->wifi_get_status == NULL) {
        out_status->state = DRIVER_WIFI_STATE_NO_ADAPTER;
        out_status->ssid[0] = '\0';
        out_status->mac[0] = out_status->mac[1] = out_status->mac[2] = 0u;
        out_status->mac[3] = out_status->mac[4] = out_status->mac[5] = 0u;
        return;
    }
    g_active_driver->wifi_get_status(out_status);
}
