#include "WaitQueue.h"

void wait_queue_init(wait_queue_t *queue)
{
    if (queue == 0) {
        return;
    }
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    spinlock_init(&queue->lock);
}

int wait_queue_push(wait_queue_t *queue, int32_t pid)
{
    if (queue == 0 || pid < 0) {
        return -1;
    }

    uint64_t flags = irq_save_disable();
    spinlock_lock(&queue->lock);
    if (queue->count >= OS_CONFIG_PROCESS_MAX_COUNT) {
        spinlock_unlock(&queue->lock);
        irq_restore(flags);
        return -1;
    }
    queue->pids[queue->tail] = pid;
    queue->tail = (queue->tail + 1u) % OS_CONFIG_PROCESS_MAX_COUNT;
    queue->count++;
    spinlock_unlock(&queue->lock);
    irq_restore(flags);
    return 0;
}

int32_t wait_queue_pop(wait_queue_t *queue)
{
    if (queue == 0) {
        return -1;
    }

    uint64_t flags = irq_save_disable();
    spinlock_lock(&queue->lock);
    if (queue->count == 0u) {
        spinlock_unlock(&queue->lock);
        irq_restore(flags);
        return -1;
    }
    int32_t pid = queue->pids[queue->head];
    queue->head = (queue->head + 1u) % OS_CONFIG_PROCESS_MAX_COUNT;
    queue->count--;
    spinlock_unlock(&queue->lock);
    irq_restore(flags);
    return pid;
}
