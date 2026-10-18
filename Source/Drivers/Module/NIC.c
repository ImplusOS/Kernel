#include "NIC.h"

#include "Drivers/Module/DriverManager.h"
#include "Drivers/Module/DeviceRegistry.h"

static volatile uint32_t g_poll_pending = 0;
static const driver_nic_t *g_active_driver = NULL;

/* The wireless half of driver_nic_t (wifi_scan_start ... wifi_get_status) is
 * the Wi-Fi panel's interface. Those calls must not be resolved through
 * g_active_driver: on a machine that has both a wireless module and a wired
 * controller the wired one can be what nic_init() picked for the data path,
 * and the panel would then be talking to a driver whose wifi_get_status is
 * NULL. So look for a NIC that actually offers the Wi-Fi interface, trying
 * the active driver first (the usual case: the wireless module is also the
 * active NIC) and the registry after. */
static const driver_nic_t *nic_wifi_driver(void)
{
    if (g_active_driver != NULL && g_active_driver->wifi_get_status != NULL) {
        return g_active_driver;
    }

    for (uint32_t i = 0;; ++i) {
        const device_t *dev = device_registry_find_by_index(DEVICE_TYPE_NIC, i);
        if (dev == NULL) {
            return NULL;
        }
        const driver_nic_t *driver = (const driver_nic_t *)dev->ops;
        if (driver != NULL && driver->wifi_get_status != NULL) {
            return driver;
        }
    }
}

bool nic_init(void)
{
    if (g_active_driver != NULL) {
        return true;
    }

    /* A wireless NIC that is attached takes precedence over everything
     * else. Its is_ready() stays false until the user has connected it
     * through the Wi-Fi panel, so this is not about picking "the link that
     * works right now" -- it is about which driver this one-shot call
     * hands the data path to. On a machine with a wireless module and a
     * wired controller, the module is the adapter the user interacts with;
     * a wired controller that merely exists (no cable) would otherwise win
     * here and leave no way to get a network at all. If no wireless driver
     * answers, the historical rule applies unchanged: the first NIC of any
     * kind whose init() succeeds. */
    for (uint32_t pass = 0u; pass < 2u; ++pass) {
        for (uint32_t i = 0;; ++i) {
            const device_t *dev = device_registry_find_by_index(DEVICE_TYPE_NIC, i);
            if (dev == NULL) {
                break;
            }
            const driver_nic_t *driver = (const driver_nic_t *)dev->ops;
            if (driver == NULL || driver->init == NULL) {
                continue;
            }
            bool wireless = (driver->wifi_get_status != NULL);
            if (pass == 0u && !wireless) {
                continue;
            }
            if (driver->init()) {
                g_active_driver = driver;
                return true;
            }
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
    const driver_nic_t *driver = nic_wifi_driver();
    if (driver == NULL || driver->wifi_scan_start == NULL) {
        return false;
    }
    return driver->wifi_scan_start();
}

uint32_t nic_wifi_get_scan_results(driver_wifi_scan_result_t *out, uint32_t max_count)
{
    const driver_nic_t *driver = nic_wifi_driver();
    if (driver == NULL || driver->wifi_get_scan_results == NULL) {
        return 0u;
    }
    return driver->wifi_get_scan_results(out, max_count);
}

bool nic_wifi_connect(const char *ssid, const char *psk)
{
    const driver_nic_t *driver = nic_wifi_driver();
    if (driver == NULL || driver->wifi_connect == NULL) {
        return false;
    }
    return driver->wifi_connect(ssid, psk);
}

void nic_wifi_disconnect(void)
{
    const driver_nic_t *driver = nic_wifi_driver();
    if (driver == NULL || driver->wifi_disconnect == NULL) {
        return;
    }
    driver->wifi_disconnect();
}

void nic_wifi_get_status(driver_wifi_status_t *out_status)
{
    if (out_status == NULL) {
        return;
    }
    const driver_nic_t *driver = nic_wifi_driver();
    if (driver == NULL || driver->wifi_get_status == NULL) {
        out_status->state = DRIVER_WIFI_STATE_NO_ADAPTER;
        out_status->ssid[0] = '\0';
        out_status->mac[0] = out_status->mac[1] = out_status->mac[2] = 0u;
        out_status->mac[3] = out_status->mac[4] = out_status->mac[5] = 0u;
        return;
    }
    driver->wifi_get_status(out_status);
}
