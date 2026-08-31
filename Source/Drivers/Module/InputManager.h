#pragma once

#include "DriverBinary.h"

#include <stdbool.h>
#include <stdint.h>

void input_manager_init(void);
void input_manager_poll(void);
int32_t input_manager_read_keyboard(driver_keyboard_event_t *out_event);
int32_t input_manager_read_mouse(driver_mouse_event_t *out_event);
void input_manager_drain_keyboard(driver_keyboard_event_t *tmp,
                                  void (*forward)(driver_keyboard_event_t *));
void input_manager_drain_mouse(driver_mouse_event_t *tmp,
                               void (*forward)(driver_mouse_event_t *));
void input_manager_schedule_poll(void);
bool input_manager_check_poll(void);
