#include "Protocol_USB_MassStorage.h"
#include "../../IO_Main.h"
#include "Drivers/Module/DriverManager.h"
#include "Drivers/Module/USB_Driver_API.h"
#include "Core/timer/Timer.h"

static bool g_usb_ms_working = false;

bool usb_ms_init(uint64_t partition_lba) {
    g_usb_ms_working = false;

    usb_driver_client_init();

    uint8_t probe_buf[512];
    
    for (int retry = 0; retry < 1000; retry++) {
        uint32_t count = usb_driver_client_get_device_count();
        if (count > 0) {
            for (uint32_t i = 0; i < count; i++) {
                if (usb_driver_client_select_device(i)) {
                    if (usb_driver_client_read_sectors(partition_lba, probe_buf, 1)) {
                        g_usb_ms_working = true;
                        return true;
                    }
                    if (partition_lba != 0 && usb_driver_client_read_sectors(0, probe_buf, 1)) {
                        g_usb_ms_working = true;
                        return true;
                    }
                }
            }
        }
        timer_apic_sleep_ms(10);
    }
    
    return false;
}

bool usb_ms_read(uint64_t lba, uint8_t *buffer, uint32_t sectors)
{
    if (!g_usb_ms_working) return false;
    if (sectors == 0) return true;
    return usb_driver_client_read_sectors(lba, buffer, sectors);
}

bool usb_ms_write(uint64_t lba, const uint8_t *buffer, uint32_t sectors)
{
    if (!g_usb_ms_working) return false;
    if (sectors == 0) return true;
    return usb_driver_client_write_sectors(lba, buffer, sectors);
}

bool usb_ms_is_working(void) {
    return g_usb_ms_working;
}

uint32_t usb_ms_get_device_count(void) {
    if (!g_usb_ms_working) return 0;
    return usb_driver_client_get_device_count();
}

bool usb_ms_select_device(uint32_t index) {
    if (!g_usb_ms_working) return false;
    return usb_driver_client_select_device(index);
}

uint64_t usb_ms_get_total_bytes(void) {
    if (!g_usb_ms_working) return 0;
    return usb_driver_client_get_total_bytes();
}
