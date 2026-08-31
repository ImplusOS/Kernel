#pragma once

#include <stdint.h>
#include "kernel/status.h"

#define IPC_MESSAGE_MAX_SIZE 256
#define IPC_MAX_MESSAGES_PER_PROCESS 128
#define IPC_SIGNAL_SHUTDOWN 0x80000001

typedef struct {
    int32_t sender_pid;
    uint32_t size;
    char data[IPC_MESSAGE_MAX_SIZE];
} __attribute__((aligned(16))) ipc_message_t;

typedef struct {
    ipc_message_t messages[IPC_MAX_MESSAGES_PER_PROCESS];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} ipc_message_queue_t;

void ipc_init(void);
void ipc_init_process_queue(int32_t pid);
os_status_t ipc_send_message(int32_t target_pid, const void *message, uint32_t size);
os_status_t ipc_send_message_from_pid(int32_t sender_pid,
                                      int32_t target_pid,
                                      const void *message,
                                      uint32_t size);
os_status_t ipc_receive_message(ipc_message_t *out_message);
void ipc_cleanup_process_queue(int32_t pid);
