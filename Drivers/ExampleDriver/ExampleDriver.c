#include "ExampleDriver.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"
#endif

#ifdef IMPLUS_DRIVER_MODULE
static const driver_binary_t *g_api = NULL;
#endif

void example_driver_init(void) {
    
}

#ifdef IMPLUS_DRIVER_MODULE
static void example_driver_shutdown(void)
{
    g_api = NULL;
}

static const driver_module_descriptor_t g_example_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_UNKNOWN,
    .load_priority = 100u,
    .deps = { NULL },
    .driver_api = NULL,
    .shutdown = example_driver_shutdown,
};

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL) {
        return NULL;
    }
    g_api = api;
    example_driver_init();
    return &g_example_module;
}
#endif
