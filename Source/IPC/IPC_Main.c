#include "IPC_Main.h"
#include "Core/process/ProcessManager.h"
#include "MemoryManagement/Memory_Main.h"
#include "Core/sync/Spinlock.h"
#include "IPC/PnP_Notifications.h"
#include "kernel/config.h"
#include <string.h>

#define MAX_PROCESSES OS_CONFIG_PROCESS_MAX_COUNT

static ipc_message_queue_t *g_message_queues[MAX_PROCESSES];
static spinlock_t g_ipc_lock;

void ipc_init(void)
{
    spinlock_init(&g_ipc_lock);
    for (int i = 0; i < MAX_PROCESSES; ++i) {
        g_message_queues[i] = NULL;
    }
}

void ipc_init_process_queue(int32_t pid)
{
    if (pid < 0 || pid >= MAX_PROCESSES) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipc_lock);

    if (g_message_queues[pid] == NULL) {
        g_message_queues[pid] = (ipc_message_queue_t *)malloc(sizeof(ipc_message_queue_t));
        if (g_message_queues[pid] != NULL) {
            memset(g_message_queues[pid], 0, sizeof(ipc_message_queue_t));
        }
    }

    spinlock_unlock(&g_ipc_lock);
    irq_restore(irq_flags);
}

os_status_t ipc_send_message_from_pid(int32_t sender_pid,
                                      int32_t target_pid,
                                      const void *message,
                                      uint32_t size)
{
    if (target_pid < 0 || target_pid >= MAX_PROCESSES || message == NULL || size > IPC_MESSAGE_MAX_SIZE) {
        return OS_STATUS_INVALID_ARG;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipc_lock);

    ipc_message_queue_t *queue = g_message_queues[target_pid];
    if (queue == NULL) {
        spinlock_unlock(&g_ipc_lock);
        irq_restore(irq_flags);
        return OS_STATUS_NOT_FOUND;
    }

    if (queue->count >= IPC_MAX_MESSAGES_PER_PROCESS) {
        spinlock_unlock(&g_ipc_lock);
        irq_restore(irq_flags);
        return OS_STATUS_LIMIT_REACHED;
    }

    ipc_message_t *msg = &queue->messages[queue->tail];
    msg->sender_pid = sender_pid;
    msg->size = size;
    memcpy(msg->data, message, size);

    queue->tail = (queue->tail + 1) % IPC_MAX_MESSAGES_PER_PROCESS;
    queue->count++;

    spinlock_unlock(&g_ipc_lock);
    irq_restore(irq_flags);

    process_perf_note_ipc_send(sender_pid);
    (void)process_wake_pid(target_pid);
    return OS_STATUS_OK;
}

os_status_t ipc_send_message(int32_t target_pid, const void *message, uint32_t size)
{
    int32_t sender_pid = process_get_current_pid();
    if (pnp_notifications_is_endpoint_pid(target_pid) != 0) {
        os_status_t status = pnp_notifications_handle_ipc(sender_pid, message, size);
        if (status == OS_STATUS_OK) {
            process_perf_note_ipc_send(sender_pid);
        }
        return status;
    }

    return ipc_send_message_from_pid(sender_pid, target_pid, message, size);
}

os_status_t ipc_receive_message(ipc_message_t *out_message)
{
    if (out_message == NULL) {
        return OS_STATUS_INVALID_ARG;
    }

    int32_t current_pid = process_get_current_pid();
    if (current_pid < 0 || current_pid >= MAX_PROCESSES) {
        return OS_STATUS_FAULT;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipc_lock);

    ipc_message_queue_t *queue = g_message_queues[current_pid];
    if (queue == NULL || queue->count == 0) {
        spinlock_unlock(&g_ipc_lock);
        irq_restore(irq_flags);
        return OS_STATUS_NOT_FOUND;
    }

    memcpy(out_message, &queue->messages[queue->head], sizeof(ipc_message_t));
    queue->head = (queue->head + 1) % IPC_MAX_MESSAGES_PER_PROCESS;
    queue->count--;

    spinlock_unlock(&g_ipc_lock);
    irq_restore(irq_flags);

    process_perf_note_ipc_recv(current_pid);
    return OS_STATUS_OK;
}

void ipc_cleanup_process_queue(int32_t pid)
{
    if (pid < 0 || pid >= MAX_PROCESSES) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_ipc_lock);

    if (g_message_queues[pid] != NULL) {
        free(g_message_queues[pid]);
        g_message_queues[pid] = NULL;
    }

    spinlock_unlock(&g_ipc_lock);
    irq_restore(irq_flags);
}
