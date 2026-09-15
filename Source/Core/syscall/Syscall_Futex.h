#pragma once
#include <stdint.h>

int64_t syscall_futex(uint64_t uaddr, uint64_t op, uint64_t val,
                      uint64_t timeout_or_val2, uint64_t uaddr2,
                      uint64_t val3);
void syscall_futex_on_timer_tick(void);

/* Linux FUTEX_WAIT / FUTEX_WAIT_BITSET, finished across a syscall restart.
 * See the comment on syscall_futex_linux_resume() in Syscall_Futex.c. */
int64_t syscall_futex_wait_linux(uint64_t uaddr, int32_t expected,
                                 uint64_t timeout_ns, uint32_t bitset,
                                 int *restart_out);
int syscall_futex_linux_resume(int64_t *result_out, int *restart_out);
