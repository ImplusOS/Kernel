#pragma once

#include <stdint.h>

#include "kernel/pnp.h"
#include "kernel/status.h"

void pnp_notifications_publish(const pnp_event_t *event);
int pnp_notifications_is_endpoint_pid(int32_t pid);
os_status_t pnp_notifications_handle_ipc(int32_t sender_pid,
                                         const void *message,
                                         uint32_t size);
void pnp_notifications_cleanup_process(int32_t pid);
