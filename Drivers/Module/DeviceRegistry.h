#pragma once

#include "kernel/interfaces/device.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef DEVICE_REGISTRY_MAX
#define DEVICE_REGISTRY_MAX 32u
#endif

typedef void (*device_detach_callback_t)(const char *name, device_type_t type);

void device_registry_init(void);
bool device_registry_add(const device_t *dev);
bool device_registry_remove(const char *name);
const device_t *device_registry_find(device_type_t type, const char *name);
const device_t *device_registry_find_by_index(device_type_t type, uint32_t index);
void device_registry_clear(void);
void device_registry_set_detach_callback(device_detach_callback_t callback);
