#pragma once

#include "Core/sync/Spinlock.h"
#include "kernel/config.h"
#include <stdint.h>

typedef struct {
    int32_t pids[OS_CONFIG_PROCESS_MAX_COUNT];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    spinlock_t lock;
} wait_queue_t;

void wait_queue_init(wait_queue_t *queue);
int wait_queue_push(wait_queue_t *queue, int32_t pid);
int32_t wait_queue_pop(wait_queue_t *queue);
