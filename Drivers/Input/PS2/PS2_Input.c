#include "Drivers/Module/PS2_Input.h"

#include "Platform/io/IO_Main.h"
#include "kernel/platform.h"
#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"
#endif

#include <stddef.h>
#include <stdint.h>
#include "kernel/keycodes.h"
#include "kernel/input_utils.h"

static const driver_binary_t *g_driver_api = NULL;

#define inb  g_driver_api->inb
#define outb g_driver_api->outb


#define PS2_DATA_PORT    0x60
#define PS2_STATUS_PORT  0x64
#define PS2_COMMAND_PORT 0x64

#define PS2_STATUS_OUTPUT_FULL 0x01
#define PS2_STATUS_INPUT_FULL  0x02
#define PS2_STATUS_AUX_DATA    0x20

#define PS2_CMD_READ_CONFIG   0x20
#define PS2_CMD_WRITE_CONFIG  0x60
#define PS2_CMD_DISABLE_PORT1 0xAD
#define PS2_CMD_ENABLE_PORT1  0xAE
#define PS2_CMD_DISABLE_PORT2 0xA7
#define PS2_CMD_ENABLE_PORT2  0xA8
#define PS2_CMD_TEST_PORT1    0xAB
#define PS2_CMD_TEST_PORT2    0xA9
#define PS2_CMD_SELF_TEST     0xAA
#define PS2_CMD_WRITE_MOUSE   0xD4

#define PS2_CONFIG_IRQ_PORT1   0x01
#define PS2_CONFIG_IRQ_PORT2   0x02
#define PS2_CONFIG_TRANSLATION 0x40

#define PS2_KBD_CMD_ENABLE_SCANNING   0xF4
#define PS2_MOUSE_CMD_SET_DEFAULTS    0xF6
#define PS2_MOUSE_CMD_ENABLE_REPORTING 0xF4

#define PS2_ACK       0xFA
#define PS2_RESEND    0xFE
#define PS2_TEST_OK   0x00
#define PS2_SELF_OK   0x55

#define PS2_COMMAND_TIMEOUT 100000u
#define PS2_MAX_POLL_READS  64u
#define PS2_QUEUE_SIZE      128u
#define PS2_SEND_RETRIES    3u
#define PS2_MOUSE_X_MAX     639
#define PS2_MOUSE_Y_MAX     479
#define PS2_MOUSE_DEFAULT_X 12
#define PS2_MOUSE_DEFAULT_Y 12

#include "Drivers/Module/Display_Main.h"

static uint8_t g_input_initialized = 0;
static uint8_t g_keyboard_available = 0;
static uint8_t g_mouse_available = 0;

static uint8_t g_keyboard_e0_pending = 0;
static uint8_t g_keyboard_modifiers = 0;
static uint8_t g_mouse_packet[3];
static uint8_t g_mouse_packet_index = 0;
static uint8_t g_last_mouse_buttons = 0;

static driver_keyboard_event_t g_keyboard_queue[PS2_QUEUE_SIZE];
static uint32_t g_keyboard_head = 0;
static uint32_t g_keyboard_tail = 0;
static uint32_t g_keyboard_count = 0;

static driver_mouse_event_t g_mouse_queue[PS2_QUEUE_SIZE];
static uint32_t g_mouse_head = 0;
static uint32_t g_mouse_tail = 0;
static uint32_t g_mouse_count = 0;

static bool ps2_platform_supported(void)
{
#if defined(PLATFORM_X86_64)
    return true;
#else
    return false;
#endif
}

static int controller_wait_input_clear(uint32_t spins)
{
    while (spins > 0u) {
        if ((inb(PS2_STATUS_PORT) & PS2_STATUS_INPUT_FULL) == 0u) {
            return 0;
        }
        --spins;
    }
    return -1;
}

static int controller_wait_output_full(uint32_t spins)
{
    while (spins > 0u) {
        if ((inb(PS2_STATUS_PORT) & PS2_STATUS_OUTPUT_FULL) != 0u) {
            return 0;
        }
        --spins;
    }
    return -1;
}

static int controller_write_command(uint8_t command)
{
    if (controller_wait_input_clear(PS2_COMMAND_TIMEOUT) < 0) {
        return -1;
    }
    outb(PS2_COMMAND_PORT, command);
    return 0;
}

static int controller_write_data(uint8_t data)
{
    if (controller_wait_input_clear(PS2_COMMAND_TIMEOUT) < 0) {
        return -1;
    }
    outb(PS2_DATA_PORT, data);
    return 0;
}

static int controller_read_data(uint8_t *value_out)
{
    if (value_out == NULL) {
        return -1;
    }
    if (controller_wait_output_full(PS2_COMMAND_TIMEOUT) < 0) {
        return -1;
    }
    *value_out = inb(PS2_DATA_PORT);
    return 0;
}

static void controller_flush_output(void)
{
    for (uint32_t i = 0; i < PS2_MAX_POLL_READS; ++i) {
        if ((inb(PS2_STATUS_PORT) & PS2_STATUS_OUTPUT_FULL) == 0u) {
            break;
        }
        (void)inb(PS2_DATA_PORT);
    }
}

static int controller_read_config(uint8_t *config_out)
{
    if (controller_write_command(PS2_CMD_READ_CONFIG) < 0) {
        return -1;
    }
    return controller_read_data(config_out);
}

static int controller_write_config(uint8_t config)
{
    if (controller_write_command(PS2_CMD_WRITE_CONFIG) < 0) {
        return -1;
    }
    return controller_write_data(config);
}

static int controller_test_port(uint8_t test_command)
{
    uint8_t status = 0;
    if (controller_write_command(test_command) < 0) {
        return -1;
    }
    if (controller_read_data(&status) < 0) {
        return -1;
    }
    return (status == PS2_TEST_OK) ? 0 : -1;
}

static int device_wait_ack(void)
{
    for (uint32_t spin = 0; spin < PS2_COMMAND_TIMEOUT; ++spin) {
        if ((inb(PS2_STATUS_PORT) & PS2_STATUS_OUTPUT_FULL) == 0u) {
            continue;
        }
        uint8_t value = inb(PS2_DATA_PORT);
        if (value == PS2_ACK) {
            return 0;
        }
        if (value == PS2_RESEND) {
            return 1;
        }
    }
    return -1;
}

static int send_keyboard_command(uint8_t command)
{
    for (uint32_t retry = 0; retry < PS2_SEND_RETRIES; ++retry) {
        if (controller_write_data(command) < 0) {
            return -1;
        }
        int ack = device_wait_ack();
        if (ack == 0) {
            return 0;
        }
        if (ack < 0) {
            return -1;
        }
    }
    return -1;
}

static int send_mouse_command(uint8_t command)
{
    for (uint32_t retry = 0; retry < PS2_SEND_RETRIES; ++retry) {
        if (controller_write_command(PS2_CMD_WRITE_MOUSE) < 0) {
            return -1;
        }
        if (controller_write_data(command) < 0) {
            return -1;
        }
        int ack = device_wait_ack();
        if (ack == 0) {
            return 0;
        }
        if (ack < 0) {
            return -1;
        }
    }
    return -1;
}

static void keyboard_queue_push(const driver_keyboard_event_t *event)
{
    if (event == NULL || g_keyboard_count >= PS2_QUEUE_SIZE) {
        return;
    }
    g_keyboard_queue[g_keyboard_head] = *event;
    g_keyboard_head = (g_keyboard_head + 1u) % PS2_QUEUE_SIZE;
    ++g_keyboard_count;
}

static int keyboard_queue_pop(driver_keyboard_event_t *event_out)
{
    if (event_out == NULL || g_keyboard_count == 0u) {
        return 0;
    }
    *event_out = g_keyboard_queue[g_keyboard_tail];
    g_keyboard_tail = (g_keyboard_tail + 1u) % PS2_QUEUE_SIZE;
    --g_keyboard_count;
    return 1;
}

static void mouse_queue_push(const driver_mouse_event_t *event)
{
    if (event == NULL) {
        return;
    }
    if (g_mouse_count >= PS2_QUEUE_SIZE) {
        g_mouse_tail = (g_mouse_tail + 1u) % PS2_QUEUE_SIZE;
        --g_mouse_count;
    }
    g_mouse_queue[g_mouse_head] = *event;
    g_mouse_head = (g_mouse_head + 1u) % PS2_QUEUE_SIZE;
    ++g_mouse_count;
}

static int mouse_queue_pop(driver_mouse_event_t *event_out)
{
    if (event_out == NULL || g_mouse_count == 0u) {
        return 0;
    }
    *event_out = g_mouse_queue[g_mouse_tail];
    g_mouse_tail = (g_mouse_tail + 1u) % PS2_QUEUE_SIZE;
    --g_mouse_count;
    return 1;
}

static void ps2_publish_device_event(uint32_t device_class,
                                     const char *device_name,
                                     uint32_t port)
{
    if (g_driver_api == NULL || g_driver_api->pnp_notify == NULL) {
        return;
    }

    pnp_event_t event;
    pnp_event_init(&event,
                   PNP_EVENT_DEVICE_ADDED,
                   PNP_BUS_PS2,
                   device_class,
                   "PS2_Driver.ELF",
                   device_name,
                   "PS/2 device ready");
    event.location0 = port;
    g_driver_api->pnp_notify(&event);
}

static void keyboard_update_modifiers(uint16_t keycode, int pressed)
{
    uint8_t base = (uint8_t)(keycode & 0x00FFu);
    int extended = (keycode & 0xFF00u) != 0;

    if (extended) {
        switch (keycode) {
        case KEY_RIGHTCTRL:
            if (pressed) g_keyboard_modifiers |= DRIVER_KBD_MOD_CTRL;
            else g_keyboard_modifiers &= (uint8_t)~DRIVER_KBD_MOD_CTRL;
            break;
        case KEY_RIGHTALT:
            if (pressed) g_keyboard_modifiers |= DRIVER_KBD_MOD_ALT;
            else g_keyboard_modifiers &= (uint8_t)~DRIVER_KBD_MOD_ALT;
            break;
        }
        return;
    }

    switch (base) {
    case KEY_LEFTSHIFT:
    case KEY_RIGHTSHIFT:
        if (pressed) g_keyboard_modifiers |= DRIVER_KBD_MOD_SHIFT;
        else g_keyboard_modifiers &= (uint8_t)~DRIVER_KBD_MOD_SHIFT;
        break;
    case KEY_LEFTCTRL:
        if (pressed) g_keyboard_modifiers |= DRIVER_KBD_MOD_CTRL;
        else g_keyboard_modifiers &= (uint8_t)~DRIVER_KBD_MOD_CTRL;
        break;
    case KEY_LEFTALT:
        if (pressed) g_keyboard_modifiers |= DRIVER_KBD_MOD_ALT;
        else g_keyboard_modifiers &= (uint8_t)~DRIVER_KBD_MOD_ALT;
        break;
    case KEY_CAPSLOCK:
        if (pressed) g_keyboard_modifiers ^= DRIVER_KBD_MOD_CAPS;
        break;
    }
}

static void handle_keyboard_byte(uint8_t value)
{
    if (value == PS2_ACK || value == PS2_RESEND) {
        return;
    }
    if (value == 0xE0u) {
        g_keyboard_e0_pending = 1;
        return;
    }
    if (value == 0xE1u) {
        g_keyboard_e0_pending = 0;
        return;
    }

    int pressed = ((value & 0x80u) == 0u) ? 1 : 0;
    uint16_t keycode = (uint16_t)(value & 0x7Fu);
    if (g_keyboard_e0_pending) {
        keycode |= 0xE000u;
        g_keyboard_e0_pending = 0;
    }

    keyboard_update_modifiers(keycode, pressed);

    driver_keyboard_event_t event = {0};
    event.keycode = keycode;
    event.pressed = (uint8_t)(pressed ? 1 : 0);
    event.modifiers = g_keyboard_modifiers;
    event.ascii = pressed ? (uint8_t)keycode_to_ascii(keycode, g_keyboard_modifiers) : 0u;
    keyboard_queue_push(&event);
}

static void handle_mouse_byte(uint8_t value)
{
    if (value == PS2_ACK || value == PS2_RESEND) {
        return;
    }

    if (g_mouse_packet_index == 0u && (value & 0x08u) == 0u) {
        return;
    }

    g_mouse_packet[g_mouse_packet_index++] = value;
    if (g_mouse_packet_index < 3u) {
        return;
    }

    g_mouse_packet_index = 0;

    uint8_t b0 = g_mouse_packet[0];
    uint8_t b1 = g_mouse_packet[1];
    uint8_t b2 = g_mouse_packet[2];

    if ((b0 & 0xC0u) != 0u) {
        return;
    }

    int16_t dx = (int16_t)(int8_t)b1;
    int16_t dy = (int16_t)(int8_t)b2;
    uint8_t buttons = (uint8_t)(b0 & 0x07u);

    if (dx == 0 && dy == 0 && buttons == g_last_mouse_buttons) {
        return;
    }

    driver_mouse_event_t event = {0};
    event.x = (uint16_t)(int16_t)dx;
    event.y = (uint16_t)(int16_t)dy;
    event.buttons = buttons;
    event.wheel = 0;
    mouse_queue_push(&event);
    g_last_mouse_buttons = buttons;
}

bool ps2_input_init(void)
{
    uint8_t config = 0;
    int port1_ok = 0;
    int port2_ok = 0;

    g_input_initialized = 0;
    g_keyboard_available = 0;
    g_mouse_available = 0;
    g_keyboard_e0_pending = 0;
    g_keyboard_modifiers = 0;
    g_mouse_packet_index = 0;
    g_last_mouse_buttons = 0;
    g_keyboard_head = g_keyboard_tail = g_keyboard_count = 0;
    g_mouse_head = g_mouse_tail = g_mouse_count = 0;

    if (!ps2_platform_supported()) {
        return false;
    }

    (void)controller_write_command(PS2_CMD_DISABLE_PORT1);
    (void)controller_write_command(PS2_CMD_DISABLE_PORT2);
    controller_flush_output();

    if (controller_read_config(&config) < 0) {
        return false;
    }

    config &= (uint8_t)~(PS2_CONFIG_IRQ_PORT1 | PS2_CONFIG_IRQ_PORT2);
    config |= PS2_CONFIG_TRANSLATION;
    if (controller_write_command(PS2_CMD_SELF_TEST) == 0) {
        uint8_t self_test = 0;
        if (controller_read_data(&self_test) == 0 && self_test != PS2_SELF_OK) {
            return false;
        }
        controller_write_config(config);
    }

    port1_ok = (controller_test_port(PS2_CMD_TEST_PORT1) == 0) ? 1 : 0;
    port2_ok = (controller_test_port(PS2_CMD_TEST_PORT2) == 0) ? 1 : 0;

    if (port1_ok) {
        (void)controller_write_command(PS2_CMD_ENABLE_PORT1);
        g_keyboard_available = 1;
    }
    if (port2_ok) {
        (void)controller_write_command(PS2_CMD_ENABLE_PORT2);
        g_mouse_available = 1;
    }

    if (port1_ok && send_keyboard_command(PS2_KBD_CMD_ENABLE_SCANNING) < 0) {
        return false;
    }

    if (g_mouse_available) {
        if (send_mouse_command(PS2_MOUSE_CMD_SET_DEFAULTS) < 0 ||
            send_mouse_command(PS2_MOUSE_CMD_ENABLE_REPORTING) < 0) {
            g_mouse_available = 0;
        }
    }

    controller_flush_output();

    if (!port1_ok && !g_mouse_available) {
        return false;
    }

    g_input_initialized = 1;
    if (port1_ok) {
        ps2_publish_device_event(PNP_CLASS_KEYBOARD,
                                 "PS/2 keyboard",
                                 1u);
    }
    if (g_mouse_available) {
        ps2_publish_device_event(PNP_CLASS_MOUSE,
                                 "PS/2 mouse",
                                 2u);
    }
    return true;
}

static bool ps2_try_enable_keyboard_hotplug(void)
{
    if (g_keyboard_available != 0u) {
        return false;
    }

    controller_flush_output();
    if (controller_test_port(PS2_CMD_TEST_PORT1) != 0) {
        return false;
    }
    if (controller_write_command(PS2_CMD_ENABLE_PORT1) < 0 ||
        send_keyboard_command(PS2_KBD_CMD_ENABLE_SCANNING) < 0) {
        return false;
    }

    g_keyboard_available = 1u;
    g_input_initialized = 1u;
    ps2_publish_device_event(PNP_CLASS_KEYBOARD,
                             "PS/2 keyboard",
                             1u);
    return true;
}

static bool ps2_try_enable_mouse_hotplug(void)
{
    if (g_mouse_available != 0u) {
        return false;
    }

    controller_flush_output();
    if (controller_test_port(PS2_CMD_TEST_PORT2) != 0) {
        return false;
    }
    if (controller_write_command(PS2_CMD_ENABLE_PORT2) < 0 ||
        send_mouse_command(PS2_MOUSE_CMD_SET_DEFAULTS) < 0 ||
        send_mouse_command(PS2_MOUSE_CMD_ENABLE_REPORTING) < 0) {
        return false;
    }

    g_mouse_available = 1u;
    g_input_initialized = 1u;
    g_mouse_packet_index = 0u;
    ps2_publish_device_event(PNP_CLASS_MOUSE,
                             "PS/2 mouse",
                             2u);
    return true;
}

bool ps2_input_poll_hotplug(void)
{
    if (!ps2_platform_supported()) {
        return false;
    }

    if (!g_input_initialized) {
        return ps2_input_init();
    }

    bool changed = false;
    if (ps2_try_enable_keyboard_hotplug()) {
        changed = true;
    }
    if (ps2_try_enable_mouse_hotplug()) {
        changed = true;
    }
    return changed;
}

void ps2_input_poll(void)
{
    if (!g_input_initialized) {
        return;
    }

    uint32_t poll_count = 0;
    const uint32_t max_polls = PS2_MAX_POLL_READS * 2;

    while ((inb(PS2_STATUS_PORT) & PS2_STATUS_OUTPUT_FULL) && poll_count < max_polls) {
        uint8_t status = inb(PS2_STATUS_PORT);
        uint8_t value = inb(PS2_DATA_PORT);
        poll_count++;

        if (value == PS2_ACK || value == PS2_RESEND) {
            g_mouse_packet_index = 0;
            continue;
        }

        if (status & PS2_STATUS_AUX_DATA) {
            if (!g_mouse_available) {
                continue;
            }

            if (g_mouse_packet_index == 0u && (value & 0x08u) == 0u) {
                continue;
            }

            g_mouse_packet[g_mouse_packet_index++] = value;

            if (g_mouse_packet_index < 3u) {
                continue;
            }

            g_mouse_packet_index = 0;

            uint8_t b0 = g_mouse_packet[0];
            uint8_t b1 = g_mouse_packet[1];
            uint8_t b2 = g_mouse_packet[2];

            if ((b0 & 0xC0u) != 0u) {
                continue;
            }

            int16_t dx = (int16_t)(int8_t)b1;
            int16_t dy = (int16_t)(int8_t)b2;
            uint8_t buttons = (uint8_t)(b0 & 0x07u);

            if (dx != 0 || dy != 0 || buttons != g_last_mouse_buttons) {
                driver_mouse_event_t event = {0};
                event.x = (uint16_t)(int16_t)dx;
                event.y = (uint16_t)(int16_t)dy;
                event.buttons = buttons;
                event.wheel = 0;

                if (g_mouse_count >= PS2_QUEUE_SIZE) {
                    g_mouse_tail = (g_mouse_tail + 1u) % PS2_QUEUE_SIZE;
                    --g_mouse_count;
                }

                g_mouse_queue[g_mouse_head] = event;
                g_mouse_head = (g_mouse_head + 1u) % PS2_QUEUE_SIZE;
                ++g_mouse_count;
                g_last_mouse_buttons = buttons;
            }
        } else {
            if (value == 0xE0u) {
                g_keyboard_e0_pending = 1;
                continue;
            }

            if (value == 0xE1u) {
                g_keyboard_e0_pending = 0;
                continue;
            }

            int pressed = ((value & 0x80u) == 0u) ? 1 : 0;
            uint16_t keycode = (uint16_t)(value & 0x7Fu);

            if (g_keyboard_e0_pending) {
                keycode |= 0xE000u;
                g_keyboard_e0_pending = 0;
            }

            keyboard_update_modifiers(keycode, pressed);

            driver_keyboard_event_t event = {0};
            event.keycode = keycode;
            event.pressed = (uint8_t)(pressed ? 1 : 0);
            event.modifiers = g_keyboard_modifiers;
            event.ascii = pressed ? (uint8_t)keycode_to_ascii(keycode, g_keyboard_modifiers) : 0u;

            if (g_keyboard_count >= PS2_QUEUE_SIZE) {
                g_keyboard_tail = (g_keyboard_tail + 1u) % PS2_QUEUE_SIZE;
                --g_keyboard_count;
            }

            g_keyboard_queue[g_keyboard_head] = event;
            g_keyboard_head = (g_keyboard_head + 1u) % PS2_QUEUE_SIZE;
            ++g_keyboard_count;
        }
    }
}

int32_t ps2_input_read_keyboard(driver_keyboard_event_t *out_event)
{
    if (out_event == NULL) {
        return -1;
    }
    ps2_input_poll();
    return keyboard_queue_pop(out_event);
}

int32_t ps2_input_read_mouse(driver_mouse_event_t *out_event)
{
    if (out_event == NULL) {
        return -1;
    }
    if (!g_mouse_available) {
        return 0;
    }
    ps2_input_poll();
    return mouse_queue_pop(out_event);
}

#ifdef IMPLUS_DRIVER_MODULE
static const driver_input_t g_ps2_input_driver = {
    .init = (void(*)(void))ps2_input_init,
    .poll = ps2_input_poll,
    .read_keyboard = ps2_input_read_keyboard,
    .read_mouse = ps2_input_read_mouse,
    .poll_hotplug = ps2_input_poll_hotplug,
};

static void ps2_driver_shutdown(void)
{
    g_input_initialized = 0;
    g_keyboard_available = 0;
    g_mouse_available = 0;
    g_keyboard_e0_pending = 0;
    g_keyboard_modifiers = 0;
    g_mouse_packet_index = 0;
    g_last_mouse_buttons = 0;
    g_keyboard_head = 0;
    g_keyboard_tail = 0;
    g_keyboard_count = 0;
    g_mouse_head = 0;
    g_mouse_tail = 0;
    g_mouse_count = 0;
    g_driver_api = NULL;
}

static const driver_module_descriptor_t g_ps2_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_INPUT,
    .load_priority = 40u,
    .deps = { NULL },
    .driver_api = &g_ps2_input_driver,
    .shutdown = ps2_driver_shutdown,
};

#undef inb
#undef outb

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (!ps2_platform_supported()) {
        return NULL;
    }

    if (api == NULL || api->inb == NULL || api->outb == NULL) {
        return NULL;
    }

    g_driver_api = api;

    return &g_ps2_module;
}
#endif
