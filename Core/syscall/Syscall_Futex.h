#pragma once
#include <stdint.h>

int64_t syscall_futex(uint64_t uaddr, uint64_t op, uint64_t val,
                      uint64_t timeout_or_val2, uint64_t uaddr2,
                      uint64_t val3);
void syscall_futex_on_timer_tick(void);
