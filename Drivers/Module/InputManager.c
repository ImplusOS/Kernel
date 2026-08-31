#include "InputManager.h"
#include "Debug/serial/Serial.h"

#include "DeviceRegistry.h"

#define INPUT_MAX_DRIVERS 8u

static const driver_input_t *g_drivers[INPUT_MAX_DRIVERS];
static uint32_t g_driver_count = 0;
static volatile uint32_t g_poll_pending = 0;
static volatile uint32_t g_poll_active = 0;
static uint32_t g_init_generation = 0;

static void input_manager_add_driver(const driver_input_t *drv)
{
    if (drv == 0) {
        return;
    }

    if (g_driver_count >= INPUT_MAX_DRIVERS) {
        return;
    }

    if (drv->init != 0) {
        drv->init();
    }

    g_drivers[g_driver_count++] = drv;
}

void input_manager_init(void)
{
    g_driver_count = 0;
    __atomic_store_n(&g_poll_active, 0u, __ATOMIC_RELEASE);
    ++g_init_generation;

    for (uint32_t i = 0; i < INPUT_MAX_DRIVERS; ++i) {
        g_drivers[i] = 0;
    }

    for (uint32_t i = 0;; ++i) {
        const device_t *dev = device_registry_find_by_index(DEVICE_TYPE_INPUT, i);

        if (dev == 0) {
            break;
        }

        input_manager_add_driver((const driver_input_t *)dev->ops);
    }

    for (uint32_t i = 0;; ++i) {
        const device_t *dev = device_registry_find_by_index(DEVICE_TYPE_USB, i);
        const usb_master_vtable_t *usb;

        if (dev == 0) {
            break;
        }

        usb = (const usb_master_vtable_t *)dev->ops;
        input_manager_add_driver(&usb->input);
    }
}

void input_manager_poll(void)
{
    __atomic_store_n(&g_poll_pending, 0u, __ATOMIC_RELEASE);
    if (__atomic_exchange_n(&g_poll_active, 1u, __ATOMIC_ACQUIRE) != 0u) {
        __atomic_store_n(&g_poll_pending, 1u, __ATOMIC_RELEASE);
        return;
    }

    for (uint32_t i = 0; i < g_driver_count; ++i) {
        if (g_drivers[i] != 0 && g_drivers[i]->poll != 0) {
            g_drivers[i]->poll();
        }
    }

    __atomic_store_n(&g_poll_active, 0u, __ATOMIC_RELEASE);
}

int32_t input_manager_read_keyboard(driver_keyboard_event_t *out_event)
{
    if (out_event == 0) {
        return -1;
    }

    for (uint32_t i = 0; i < g_driver_count; ++i) {
        if (g_drivers[i] != 0 && g_drivers[i]->read_keyboard != 0) {
            int32_t rc = g_drivers[i]->read_keyboard(out_event);
            if (rc > 0) {
                return rc;
            }
        }
    }
    return 0;
}

int32_t input_manager_read_mouse(driver_mouse_event_t *out_event)
{
    if (out_event == 0) {
        return -1;
    }

    for (uint32_t i = 0; i < g_driver_count; ++i) {
        if (g_drivers[i] != 0 && g_drivers[i]->read_mouse != 0) {
            int32_t rc = g_drivers[i]->read_mouse(out_event);
            if (rc > 0) {
                return rc;
            }
        }
    }
    return 0;
}

void input_manager_drain_keyboard(driver_keyboard_event_t *tmp,
                                  void (*forward)(driver_keyboard_event_t *))
{
    for (uint32_t i = 0; i < g_driver_count; ++i) {
        if (g_drivers[i] != 0 && g_drivers[i]->drain_keyboard != 0) {
            g_drivers[i]->drain_keyboard(tmp, forward);
            continue;
        }
        if (g_drivers[i] == 0 || g_drivers[i]->read_keyboard == 0) {
            continue;
        }
        while (g_drivers[i]->read_keyboard(tmp) > 0) {
            if (forward != 0) {
                forward(tmp);
            }
        }
    }
}

void input_manager_drain_mouse(driver_mouse_event_t *tmp,
                               void (*forward)(driver_mouse_event_t *))
{
    for (uint32_t i = 0; i < g_driver_count; ++i) {
        if (g_drivers[i] != 0 && g_drivers[i]->drain_mouse != 0) {
            g_drivers[i]->drain_mouse(tmp, forward);
            continue;
        }
        if (g_drivers[i] == 0 || g_drivers[i]->read_mouse == 0) {
            continue;
        }
        while (g_drivers[i]->read_mouse(tmp) > 0) {
            if (forward != 0) {
                forward(tmp);
            }
        }
    }
}

void input_manager_schedule_poll(void)
{
    __atomic_store_n(&g_poll_pending, 1u, __ATOMIC_RELEASE);
}

bool input_manager_check_poll(void)
{
    return __atomic_exchange_n(&g_poll_pending, 0u, __ATOMIC_ACQ_REL) != 0u;
}
