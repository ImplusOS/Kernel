#ifndef PS2_INPUT_H
#define PS2_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "Drivers/Module/DriverBinary.h"

bool ps2_input_init(void);
void ps2_input_poll(void);
int32_t ps2_input_read_keyboard(driver_keyboard_event_t *out_event);
int32_t ps2_input_read_mouse(driver_mouse_event_t *out_event);

#endif
