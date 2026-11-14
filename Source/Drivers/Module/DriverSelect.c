#include <stddef.h>
#include <stdint.h>

#include "DeviceRegistry.h"
#include "DriverSelect.h"
#include "DriverBinary.h"

static driver_boot_framebuffer_t g_boot_framebuffer;

void driver_select_set_boot_framebuffer(const driver_boot_framebuffer_t *framebuffer) {
    if (!framebuffer) {
        g_boot_framebuffer.addr = NULL;
        g_boot_framebuffer.size_bytes = 0;
        g_boot_framebuffer.width = 0;
        g_boot_framebuffer.height = 0;
        g_boot_framebuffer.pixels_per_scan_line = 0;
        g_boot_framebuffer.bytes_per_pixel = 0;
        return;
    }

    g_boot_framebuffer = *framebuffer;
}

const driver_display_t *driver_select_pick_display_driver(void) {
    for (uint32_t i = 0;; ++i) {
        const device_t *device = device_registry_find_by_index(DEVICE_TYPE_DISPLAY, i);
        const driver_display_t *drv;
        if (device == NULL) {
            break;
        }

        drv = (const driver_display_t *)device->ops;
        if (drv == NULL || drv->init == NULL) {
            continue;
        }

        if (drv->set_framebuffer) {
            drv->set_framebuffer(&g_boot_framebuffer);
        }
        /* A driver that is already up (initialised by the module loader or by
         * the DRM render backend) is picked as-is. Running init() again would
         * reset a live device -- for VirtIO GPU that drops the scanout and
         * every in-flight command, which froze the screen whenever the boot
         * framebuffer could not be claimed by the generic GOP driver (OVMF's
         * virtio-gpu GOP is BltOnly, e.g. "-vga none -device virtio-gpu-gl-pci"). */
        if (drv->is_ready != NULL && drv->is_ready()) {
            return drv;
        }
        if (!drv->probe || drv->probe()) {
            if (drv->init()) {
                return drv;
            }
        }
    }

    return NULL;
}
