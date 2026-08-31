#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "Drivers/Module/DriverBinary.h"

void usb_hid_init(void);

void usb_hid_add_keyboard(uint8_t dev_addr, uint8_t interface, uint8_t ep_in, uint16_t mps);
void usb_hid_add_mouse(uint8_t dev_addr, uint8_t interface, uint8_t ep_in, uint16_t mps);
void usb_hid_remove_keyboard(uint8_t dev_addr);
void usb_hid_remove_mouse(uint8_t dev_addr);

void usb_hid_poll(void);

int32_t usb_hid_read_keyboard(driver_keyboard_event_t *out_event);
int32_t usb_hid_read_mouse(driver_mouse_event_t *out_event);
void usb_hid_drain_keyboard(driver_keyboard_event_t *tmp,
                            void (*forward)(driver_keyboard_event_t *));
void usb_hid_drain_mouse(driver_mouse_event_t *tmp,
                         void (*forward)(driver_mouse_event_t *));
