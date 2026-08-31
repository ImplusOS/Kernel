#pragma once
#include <stdint.h>

int64_t syscall_clock_gettime(int32_t clk_id, uint64_t tp_ptr);
int64_t syscall_clock_getres(int32_t clk_id, uint64_t res_ptr);
