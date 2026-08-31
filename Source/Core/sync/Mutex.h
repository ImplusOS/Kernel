#pragma once

#include "Core/sync/WaitQueue.h"
#include <stdint.h>

typedef struct {
    volatile int32_t locked;
    int32_t owner_pid;
    wait_queue_t wait_queue;
} mutex_t;

typedef struct {
    volatile int32_t count;
    wait_queue_t wait_queue;
} semaphore_t;

void mutex_init(mutex_t *mutex);
void mutex_lock(mutex_t *mutex);
void mutex_unlock(mutex_t *mutex);

void semaphore_init(semaphore_t *sem, int32_t initial_count);
void semaphore_wait(semaphore_t *sem);
void semaphore_signal(semaphore_t *sem);
