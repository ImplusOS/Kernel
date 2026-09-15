#pragma once
#include <stdint.h>

int64_t syscall_clock_gettime(int32_t clk_id, uint64_t tp_ptr);
/* CLOCK_REALTIME in nanoseconds since the Unix epoch: the RTC read once,
 * advanced by the monotonic clock. Use this rather than rtc_read_time()
 * anywhere wall-clock time is handed to a program. */
int64_t clock_realtime_ns(void);
int64_t syscall_clock_getres(int32_t clk_id, uint64_t res_ptr);
