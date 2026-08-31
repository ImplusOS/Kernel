#include "Mutex.h"

#include "Core/process/ProcessManager.h"
#include "interfaces/hal_cpu.h"

void mutex_init(mutex_t *mutex)
{
    if (mutex == 0) {
        return;
    }
    mutex->locked = 0;
    mutex->owner_pid = -1;
    wait_queue_init(&mutex->wait_queue);
}

void mutex_lock(mutex_t *mutex)
{
    if (mutex == 0) {
        return;
    }

    int32_t pid = process_get_current_tid();
    for (;;) {
        if (__atomic_exchange_n(&mutex->locked, 1, __ATOMIC_ACQUIRE) == 0) {
            break;
        }
        if (pid >= 0) {
            (void)wait_queue_push(&mutex->wait_queue, pid);
            (void)process_block_current();
        } else {
            while (__atomic_load_n(&mutex->locked, __ATOMIC_RELAXED) != 0) {
                hal_cpu_pause();
            }
        }
    }
    mutex->owner_pid = pid;
}

void mutex_unlock(mutex_t *mutex)
{
    if (mutex == 0) {
        return;
    }

    mutex->owner_pid = -1;
    __atomic_store_n(&mutex->locked, 0, __ATOMIC_RELEASE);

    int32_t pid = wait_queue_pop(&mutex->wait_queue);
    if (pid >= 0) {
        (void)process_wake_pid(pid);
    }
}

void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    if (sem == 0) {
        return;
    }
    sem->count = initial_count;
    wait_queue_init(&sem->wait_queue);
}

void semaphore_wait(semaphore_t *sem)
{
    if (sem == 0) {
        return;
    }

    int32_t pid = process_get_current_tid();
    for (;;) {
        int32_t old = __atomic_load_n(&sem->count, __ATOMIC_RELAXED);
        if (old > 0 &&
            __atomic_compare_exchange_n(&sem->count, &old, old - 1,
                                        0, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
            return;
        }
        if (pid >= 0) {
            (void)wait_queue_push(&sem->wait_queue, pid);
            (void)process_block_current();
        }
        hal_cpu_pause();
    }
}

void semaphore_signal(semaphore_t *sem)
{
    if (sem == 0) {
        return;
    }
    __atomic_add_fetch(&sem->count, 1, __ATOMIC_RELEASE);
    int32_t pid = wait_queue_pop(&sem->wait_queue);
    if (pid >= 0) {
        (void)process_wake_pid(pid);
    }
}
