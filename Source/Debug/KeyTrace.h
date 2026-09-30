#pragma once

/* [ktr] -- keystroke stage trace.
 *
 * One line per stage a physical keystroke passes through, stamped with the
 * same monotonic millisecond clock every other kernel log line uses:
 *
 *   [ktr] RD  <ms> <keycode>   WM pulled the key out of the input driver
 *   [ktr] PS  <ms> <keycode>   the X session took it out of the window's IPC queue
 *   [ktr] IN  <ms> <keycode>   the X session injected it into /dev/input/event0
 *   [ktr] OUT <ms> <keycode>   Xorg read it back out of the evdev ring
 *   [ktr] MD  <ms> 0           the X server produced the frame that answers it
 *
 * The gap between two consecutive stamps is the hop between them, so a host
 * can split "the OS is slow" into the hop that is actually slow. Userland
 * stamps its own stages with get_uptime_ms(), which reads the same counter,
 * so kernel and userland lines are directly comparable.
 *
 * Compiled in only for a profiling build -- `make ... EXTRA_KERNEL_CFLAGS=
 * -DKEY_TRACE=1` -- and an empty inline otherwise, so a normal build carries
 * no serial traffic and no branch.
 *
 * The whole line is formatted here and handed to serial_write_string() as one
 * call. That is not cosmetic: the lock serial_write_string() takes covers the
 * string, not a sequence of calls, so the previous serial_write_uint32()
 * choreography let two CPUs' lines merge into "0x0000[ktr] PS 0x0" and the
 * merged line was unparseable. Every site below runs in process context with
 * IRQs on, because a polled UART costs ~1 ms per line.
 */

#include <stdint.h>

#include "Debug/serial/Serial.h"
#include "Core/timer/Timer.h"

#ifndef KEY_TRACE
#define KEY_TRACE 0
#endif

#if KEY_TRACE

static inline uint64_t key_trace_ms(void)
{
    uint32_t hz = timer_hz();
    if (hz == 0u) {
        return 0u;
    }
    return ((uint64_t)timer_ticks() * 1000u) / (uint64_t)hz;
}

static inline uint32_t key_trace_dec(char *dst, uint64_t value)
{
    char digits[20];
    uint32_t n = 0u;

    if (value == 0u) {
        dst[0] = '0';
        return 1u;
    }
    while (value != 0u) {
        digits[n++] = (char)('0' + (char)(value % 10u));
        value /= 10u;
    }
    for (uint32_t i = 0; i < n; ++i) {
        dst[i] = digits[n - 1u - i];
    }
    return n;
}

static inline void key_trace(const char *stage, uint32_t detail)
{
    char line[80];
    uint32_t n = 0u;

    line[n++] = '[';
    line[n++] = 'k';
    line[n++] = 't';
    line[n++] = 'r';
    line[n++] = ']';
    line[n++] = ' ';
    while (*stage != '\0' && n < sizeof(line) - 32u) {
        line[n++] = *stage++;
    }
    line[n++] = ' ';
    n += key_trace_dec(&line[n], key_trace_ms());
    line[n++] = ' ';
    n += key_trace_dec(&line[n], (uint64_t)detail);
    line[n++] = '\n';
    line[n] = '\0';
    serial_write_string(line);
}

#else

static inline void key_trace(const char *stage, uint32_t detail)
{
    (void)stage;
    (void)detail;
}

#endif
