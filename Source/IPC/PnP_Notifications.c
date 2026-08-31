#include "PnP_Notifications.h"

#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "IPC/IPC_Main.h"

#include <string.h>

#define PNP_HISTORY_MAX 64u

static spinlock_t g_pnp_lock;
static pnp_event_t g_history[PNP_HISTORY_MAX];
static uint32_t g_history_head;
static uint32_t g_history_count;
static uint64_t g_next_sequence = 1u;
static int32_t g_subscriber_pid = -1;
static uint64_t g_subscriber_next_sequence = 1u;

static uint64_t pnp_timestamp_ms(void)
{
    return timer_monotonic_ns() / 1000000ULL;
}

static uint64_t pnp_oldest_sequence_locked(void)
{
    if (g_history_count == 0u) {
        return g_next_sequence;
    }
    return g_history[g_history_head].sequence;
}

static void pnp_store_locked(const pnp_event_t *event)
{
    uint32_t index;

    if (g_history_count < PNP_HISTORY_MAX) {
        index = (g_history_head + g_history_count) % PNP_HISTORY_MAX;
        ++g_history_count;
    } else {
        index = g_history_head;
        g_history_head = (g_history_head + 1u) % PNP_HISTORY_MAX;
    }

    g_history[index] = *event;
}

static int pnp_copy_next_for_subscriber(pnp_event_t *out_event,
                                        int32_t *out_pid)
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_pnp_lock);

    if (g_subscriber_pid < 0 || out_event == 0 || out_pid == 0) {
        spinlock_unlock(&g_pnp_lock);
        irq_restore(flags);
        return 0;
    }

    for (uint32_t i = 0u; i < g_history_count; ++i) {
        uint32_t index = (g_history_head + i) % PNP_HISTORY_MAX;
        if (g_history[index].sequence >= g_subscriber_next_sequence) {
            *out_event = g_history[index];
            *out_pid = g_subscriber_pid;
            spinlock_unlock(&g_pnp_lock);
            irq_restore(flags);
            return 1;
        }
    }

    spinlock_unlock(&g_pnp_lock);
    irq_restore(flags);
    return 0;
}

static void pnp_mark_delivered(uint64_t sequence)
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_pnp_lock);
    if (g_subscriber_next_sequence <= sequence) {
        g_subscriber_next_sequence = sequence + 1u;
    }
    spinlock_unlock(&g_pnp_lock);
    irq_restore(flags);
}

static void pnp_drop_subscriber(int32_t pid)
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_pnp_lock);
    if (g_subscriber_pid == pid) {
        g_subscriber_pid = -1;
    }
    spinlock_unlock(&g_pnp_lock);
    irq_restore(flags);
}

void pnp_notifications_cleanup_process(int32_t pid)
{
    if (pid < 0) {
        return;
    }
    pnp_drop_subscriber(pid);
}

static void pnp_flush_subscriber(void)
{
    for (;;) {
        pnp_event_t event;
        int32_t pid = -1;
        if (!pnp_copy_next_for_subscriber(&event, &pid)) {
            return;
        }

        os_status_t status =
            ipc_send_message_from_pid(PNP_NOTIFICATION_ENDPOINT_PID,
                                      pid,
                                      &event,
                                      (uint32_t)sizeof(event));
        if (status == OS_STATUS_OK) {
            pnp_mark_delivered(event.sequence);
            continue;
        }

        if (status == OS_STATUS_NOT_FOUND) {
            pnp_drop_subscriber(pid);
        }
        return;
    }
}

void pnp_notifications_publish(const pnp_event_t *event)
{
    if (event == 0) {
        return;
    }

    pnp_event_t copy = *event;
    copy.magic = PNP_IPC_MAGIC;
    copy.version = PNP_IPC_VERSION;
    copy.sequence = __atomic_fetch_add(&g_next_sequence, 1u,
                                       __ATOMIC_ACQ_REL);
    copy.timestamp_ms = pnp_timestamp_ms();
    copy.driver[sizeof(copy.driver) - 1u] = '\0';
    copy.device[sizeof(copy.device) - 1u] = '\0';
    copy.detail[sizeof(copy.detail) - 1u] = '\0';

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_pnp_lock);
    pnp_store_locked(&copy);
    spinlock_unlock(&g_pnp_lock);
    irq_restore(flags);

    pnp_flush_subscriber();
}

int pnp_notifications_is_endpoint_pid(int32_t pid)
{
    return pid == PNP_NOTIFICATION_ENDPOINT_PID ? 1 : 0;
}

os_status_t pnp_notifications_handle_ipc(int32_t sender_pid,
                                         const void *message,
                                         uint32_t size)
{
    if (sender_pid < 0 || message == 0 || size != sizeof(pnp_request_t)) {
        return OS_STATUS_INVALID_ARG;
    }

    const pnp_request_t *request = (const pnp_request_t *)message;
    if (request->magic != PNP_IPC_MAGIC ||
        request->version != PNP_IPC_VERSION) {
        return OS_STATUS_INVALID_ARG;
    }

    switch (request->opcode) {
    case PNP_OP_SUBSCRIBE: {
        uint64_t flags = irq_save_disable();
        spinlock_lock(&g_pnp_lock);
        g_subscriber_pid = sender_pid;
        g_subscriber_next_sequence = pnp_oldest_sequence_locked();
        spinlock_unlock(&g_pnp_lock);
        irq_restore(flags);
        pnp_flush_subscriber();
        return OS_STATUS_OK;
    }
    case PNP_OP_UNSUBSCRIBE: {
        uint64_t flags = irq_save_disable();
        spinlock_lock(&g_pnp_lock);
        if (g_subscriber_pid == sender_pid) {
            g_subscriber_pid = -1;
        }
        spinlock_unlock(&g_pnp_lock);
        irq_restore(flags);
        return OS_STATUS_OK;
    }
    case PNP_OP_DRAIN:
        pnp_flush_subscriber();
        return OS_STATUS_OK;
    default:
        return OS_STATUS_NOT_SUPPORTED;
    }
}
