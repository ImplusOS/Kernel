#include "NicManager.h"

#include "Drivers/Module/NIC.h"

bool nic_manager_init(void)
{
    return nic_init();
}

bool nic_manager_is_ready(void)
{
    return nic_is_ready();
}

uint16_t nic_manager_mtu(void)
{
    return nic_mtu();
}

void nic_manager_get_mac(uint8_t mac_out[6])
{
    nic_get_mac(mac_out);
}

bool nic_manager_send_frame(const uint8_t *frame, uint16_t frame_len)
{
    return nic_send_frame(frame, frame_len);
}

void nic_manager_poll(void)
{
    nic_poll();
}

void nic_manager_set_rx_callback(driver_nic_rx_callback_t cb)
{
    nic_set_rx_callback(cb);
}

void nic_manager_schedule_poll(void)
{
    nic_schedule_poll();
}

bool nic_manager_check_poll(void)
{
    return nic_check_poll();
}

bool nic_manager_wifi_scan_start(void)
{
    return nic_wifi_scan_start();
}

uint32_t nic_manager_wifi_get_scan_results(driver_wifi_scan_result_t *out, uint32_t max_count)
{
    return nic_wifi_get_scan_results(out, max_count);
}

bool nic_manager_wifi_connect(const char *ssid, const char *psk)
{
    return nic_wifi_connect(ssid, psk);
}

void nic_manager_wifi_disconnect(void)
{
    nic_wifi_disconnect();
}

void nic_manager_wifi_get_status(driver_wifi_status_t *out_status)
{
    nic_wifi_get_status(out_status);
}
