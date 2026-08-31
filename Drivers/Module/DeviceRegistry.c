#include "DeviceRegistry.h"

#include <string.h>

typedef struct {
    uint8_t used;
    device_t dev;
    char name_buf[64];
} registry_entry_t;

static registry_entry_t g_entries[DEVICE_REGISTRY_MAX];
static device_detach_callback_t g_detach_callback = 0;

void device_registry_init(void)
{
    device_registry_clear();
}

bool device_registry_add(const device_t *dev)
{
    if (dev == 0 || dev->name == 0 || dev->name[0] == '\0' ||
        dev->ops == 0 || dev->type == DEVICE_TYPE_UNKNOWN) {
        return false;
    }

    for (uint32_t i = 0; i < DEVICE_REGISTRY_MAX; ++i) {
        registry_entry_t *entry = &g_entries[i];
        if (entry->used != 0u && strcasecmp(entry->name_buf, dev->name) == 0) {
            entry->dev = *dev;
            entry->dev.name = entry->name_buf;
            return true;
        }
    }

    for (uint32_t i = 0; i < DEVICE_REGISTRY_MAX; ++i) {
        registry_entry_t *entry = &g_entries[i];
        if (entry->used != 0u) {
            continue;
        }

        memset(entry, 0, sizeof(*entry));
        entry->used = 1u;
        strncpy(entry->name_buf, dev->name, sizeof(entry->name_buf) - 1u);
        entry->name_buf[sizeof(entry->name_buf) - 1u] = '\0';
        entry->dev = *dev;
        entry->dev.name = entry->name_buf;
        return true;
    }

    return false;
}

bool device_registry_remove(const char *name)
{
    if (name == 0 || name[0] == '\0') {
        return false;
    }

    for (uint32_t i = 0; i < DEVICE_REGISTRY_MAX; ++i) {
        registry_entry_t *entry = &g_entries[i];
        if (entry->used == 0u || strcasecmp(entry->name_buf, name) != 0) {
            continue;
        }

        if (g_detach_callback != 0) {
            g_detach_callback(entry->name_buf, entry->dev.type);
        }
        memset(entry, 0, sizeof(*entry));
        return true;
    }

    return false;
}

const device_t *device_registry_find(device_type_t type, const char *name)
{
    if (type == DEVICE_TYPE_UNKNOWN) {
        return 0;
    }

    for (uint32_t i = 0; i < DEVICE_REGISTRY_MAX; ++i) {
        registry_entry_t *entry = &g_entries[i];
        if (entry->used == 0u || entry->dev.type != type) {
            continue;
        }
        if (name != 0 && strcasecmp(entry->name_buf, name) != 0) {
            continue;
        }
        return &entry->dev;
    }

    return 0;
}

const device_t *device_registry_find_by_index(device_type_t type, uint32_t index)
{
    uint32_t seen = 0;

    if (type == DEVICE_TYPE_UNKNOWN) {
        return 0;
    }

    for (uint32_t i = 0; i < DEVICE_REGISTRY_MAX; ++i) {
        registry_entry_t *entry = &g_entries[i];
        if (entry->used == 0u || entry->dev.type != type) {
            continue;
        }
        if (seen == index) {
            return &entry->dev;
        }
        ++seen;
    }

    return 0;
}

void device_registry_clear(void)
{
    memset(g_entries, 0, sizeof(g_entries));
}

void device_registry_set_detach_callback(device_detach_callback_t callback)
{
    g_detach_callback = callback;
}
