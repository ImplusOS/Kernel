#include "compat_registry.h"
#include <stddef.h>

#define COMPAT_REGISTRY_MAX 4u

static const compat_layer_t *g_layers[COMPAT_REGISTRY_MAX];
static uint32_t g_layer_count = 0u;

void compat_registry_init(void)
{
    g_layer_count = 0u;
}

bool compat_registry_register(const compat_layer_t *layer)
{
    if (!layer || !layer->dispatch || !layer->name) {
        return false;
    }
    for (uint32_t i = 0; i < g_layer_count; i++) {
        if (g_layers[i]->abi == layer->abi) {
            g_layers[i] = layer; /* re-registration replaces, doesn't duplicate */
            return true;
        }
    }
    if (g_layer_count >= COMPAT_REGISTRY_MAX) {
        return false;
    }
    g_layers[g_layer_count++] = layer;
    return true;
}

const compat_layer_t *compat_registry_find(process_abi_t abi)
{
    for (uint32_t i = 0; i < g_layer_count; i++) {
        if (g_layers[i]->abi == abi) {
            return g_layers[i];
        }
    }
    return NULL;
}
