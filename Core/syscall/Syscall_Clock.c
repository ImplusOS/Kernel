#include "Syscall_Main.h"
#include "Core/process/ProcessManager.h"
#include "Core/usercopy/Usercopy.h"
#include "Core/timer/Timer.h"
#include "Platform/rtc/RTC.h"

#include <stddef.h>
#include <stdint.h>

/* POSIX clock ids (Linux uapi values). */
#define CLOCK_REALTIME           0
#define CLOCK_MONOTONIC          1
#define CLOCK_PROCESS_CPUTIME_ID 2
#define CLOCK_THREAD_CPUTIME_ID  3
#define CLOCK_MONOTONIC_RAW      4
#define CLOCK_REALTIME_COARSE    5
#define CLOCK_MONOTONIC_COARSE   6
#define CLOCK_BOOTTIME           7

typedef struct {
    int64_t tv_sec;
    int64_t tv_nsec;
} kernel_timespec_t;

/* Ticks since boot -> {sec,nsec}. This single monotonic base is shared by
 * every MONOTONIC-family clock so CLOCK_MONOTONIC, CLOCK_MONOTONIC_RAW and
 * CLOCK_BOOTTIME never disagree with each other or drift relative to the
 * timerfd/nanosleep code, which is driven off the same tick counter
 * (TODO_Chromium_LinuxABI.md section 3.8). There is no suspend state, so
 * BOOTTIME == MONOTONIC by construction. */
static void clock_monotonic_now(kernel_timespec_t *out)
{
    uint32_t hz = timer_hz();
    if (hz == 0u) {
        hz = 60u;
    }
    uint64_t ticks = timer_ticks();
    uint64_t whole = ticks / hz;
    uint64_t rem = ticks % hz;
    out->tv_sec = (int64_t)whole;
    out->tv_nsec = (int64_t)((rem * 1000000000ULL) / hz);
}

static void clock_realtime_now(kernel_timespec_t *out)
{
    rtc_time_t rtc;
    rtc_read_time(&rtc);

    uint64_t days = 0;
    for (uint16_t y = 1970; y < rtc.year; ++y) {
        days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    }
    static const uint16_t mdays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    for (uint8_t m = 1; m < rtc.month && m <= 12; ++m) {
        days += mdays[m - 1];
        if (m == 2 && rtc.year % 4 == 0 &&
            (rtc.year % 100 != 0 || rtc.year % 400 == 0)) {
            days += 1;
        }
    }
    days += (uint64_t)(rtc.day - 1);
    uint64_t secs = days * 86400ULL + (uint64_t)rtc.hour * 3600ULL +
                    (uint64_t)rtc.minute * 60ULL + (uint64_t)rtc.second;
    out->tv_sec = (int64_t)secs;
    out->tv_nsec = 0;
}

int64_t syscall_clock_gettime(int32_t clk_id, uint64_t tp_ptr)
{
    kernel_timespec_t *tp = (kernel_timespec_t *)(uintptr_t)tp_ptr;
    if (tp == NULL || !process_user_buffer_is_valid(tp, sizeof(*tp))) {
        return -14;
    }

    kernel_timespec_t value = {0};
    switch (clk_id) {
        case CLOCK_REALTIME:
        case CLOCK_REALTIME_COARSE:
            clock_realtime_now(&value);
            break;
        case CLOCK_MONOTONIC:
        case CLOCK_MONOTONIC_RAW:
        case CLOCK_MONOTONIC_COARSE:
        case CLOCK_BOOTTIME:
            clock_monotonic_now(&value);
            break;
        case CLOCK_PROCESS_CPUTIME_ID:
        case CLOCK_THREAD_CPUTIME_ID:
            /* No per-task CPU accounting yet; approximate CPU time with the
             * monotonic wall clock. glibc/V8 use these only for relative
             * deltas (profiling counters), so a consistent monotonic source
             * is acceptable and far better than ENOSYS. */
            clock_monotonic_now(&value);
            break;
        default:
            return -22;
    }
    return (copy_to_user(tp, &value, sizeof(value)) == 0u) ? 0 : -14;
}

int64_t syscall_clock_getres(int32_t clk_id, uint64_t res_ptr)
{
    kernel_timespec_t *res = (kernel_timespec_t *)(uintptr_t)res_ptr;
    if (res == NULL) return 0;
    if (!process_user_buffer_is_valid(res, sizeof(*res))) {
        return -14;
    }

    switch (clk_id) {
        case CLOCK_REALTIME:
        case CLOCK_MONOTONIC:
        case CLOCK_MONOTONIC_RAW:
        case CLOCK_BOOTTIME:
        case CLOCK_PROCESS_CPUTIME_ID:
        case CLOCK_THREAD_CPUTIME_ID:
        case CLOCK_REALTIME_COARSE:
        case CLOCK_MONOTONIC_COARSE: {
            uint32_t hz = timer_hz();
            if (hz == 0u) {
                hz = 60u;
            }
            kernel_timespec_t value = {
                .tv_sec = 0,
                .tv_nsec = (int64_t)(1000000000ULL / hz),
            };
            return (copy_to_user(res, &value, sizeof(value)) == 0u) ? 0 : -14;
        }
        default:
            return -22;
    }
}
