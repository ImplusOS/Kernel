#pragma once

#include <stdbool.h>
#include "DriverBinary.h"

void driver_select_set_boot_framebuffer(const driver_boot_framebuffer_t *framebuffer);
const driver_display_t *driver_select_pick_display_driver(void);
