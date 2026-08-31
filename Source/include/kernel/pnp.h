#pragma once

#include <stdint.h>

#define PNP_NOTIFICATION_ENDPOINT_PID (-4097)
#define PNP_IPC_MAGIC 0x504E5049u
#define PNP_IPC_VERSION 1u

#define PNP_EVENT_DRIVER_READY    1u
#define PNP_EVENT_DEVICE_ADDED    2u
#define PNP_EVENT_DEVICE_REMOVED  3u
#define PNP_EVENT_DEVICE_CHANGED  4u

#define PNP_BUS_UNKNOWN  0u
#define PNP_BUS_KERNEL   1u
#define PNP_BUS_PCI      2u
#define PNP_BUS_USB      3u
#define PNP_BUS_PS2      4u
#define PNP_BUS_DISPLAY  5u
#define PNP_BUS_PLATFORM 6u

#define PNP_CLASS_UNKNOWN    0u
#define PNP_CLASS_DRIVER     1u
#define PNP_CLASS_PCI_DEVICE 2u
#define PNP_CLASS_DISPLAY    3u
#define PNP_CLASS_MONITOR    4u
#define PNP_CLASS_INPUT      5u
#define PNP_CLASS_KEYBOARD   6u
#define PNP_CLASS_MOUSE      7u
#define PNP_CLASS_USB_DEVICE 8u
#define PNP_CLASS_STORAGE    9u

#define PNP_OP_SUBSCRIBE   1u
#define PNP_OP_UNSUBSCRIBE 2u
#define PNP_OP_DRAIN       3u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t action;
    uint64_t sequence;
    uint64_t timestamp_ms;
    uint32_t bus;
    uint32_t device_class;
    uint32_t flags;
    uint32_t location0;
    uint32_t location1;
    uint16_t vendor_id;
    uint16_t device_id;
    char driver[48];
    char device[64];
    char detail[96];
} pnp_event_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t opcode;
    uint32_t flags;
    uint32_t reserved;
} pnp_request_t;

typedef char pnp_event_must_fit_ipc[(sizeof(pnp_event_t) == 256u) ? 1 : -1];

static inline void pnp_zero_bytes(void *ptr, uint32_t size)
{
    uint8_t *bytes = (uint8_t *)ptr;
    for (uint32_t i = 0u; i < size; ++i) {
        bytes[i] = 0u;
    }
}

static inline void pnp_copy_string(char *dst, uint32_t dst_size, const char *src)
{
    if (dst == 0 || dst_size == 0u) {
        return;
    }

    uint32_t i = 0u;
    if (src != 0) {
        while (i + 1u < dst_size && src[i] != '\0') {
            dst[i] = src[i];
            ++i;
        }
    }
    dst[i] = '\0';
}

static inline void pnp_event_init(pnp_event_t *event,
                                  uint16_t action,
                                  uint32_t bus,
                                  uint32_t device_class,
                                  const char *driver,
                                  const char *device,
                                  const char *detail)
{
    if (event == 0) {
        return;
    }

    pnp_zero_bytes(event, (uint32_t)sizeof(*event));
    event->magic = PNP_IPC_MAGIC;
    event->version = PNP_IPC_VERSION;
    event->action = action;
    event->bus = bus;
    event->device_class = device_class;
    pnp_copy_string(event->driver, (uint32_t)sizeof(event->driver), driver);
    pnp_copy_string(event->device, (uint32_t)sizeof(event->device), device);
    pnp_copy_string(event->detail, (uint32_t)sizeof(event->detail), detail);
}

static inline void pnp_request_init(pnp_request_t *request, uint16_t opcode)
{
    if (request == 0) {
        return;
    }

    pnp_zero_bytes(request, (uint32_t)sizeof(*request));
    request->magic = PNP_IPC_MAGIC;
    request->version = PNP_IPC_VERSION;
    request->opcode = opcode;
}
