#pragma once

#include <stdint.h>

/*
 * A flight recorder for faults that leave nothing on the serial line.
 *
 * A triple fault resets the machine without the double-fault handler ever
 * running, so the usual panic printout never happens and the log simply stops.
 * The only way to see what led up to one is to write breadcrumbs somewhere the
 * reset does not clear: QEMU (like real hardware on a warm reset) leaves RAM
 * intact, so this keeps a ring in a reserved low physical page and prints the
 * previous boot's trail on the next one.
 *
 * Enabled with -DFLIGHT_REC=1; every call compiles away otherwise.
 */

#define FR_TAG_SYSCALL   1u
#define FR_TAG_PF        2u
#define FR_TAG_GP        3u
#define FR_TAG_SWITCH    4u
#define FR_TAG_EXIT      5u
#define FR_TAG_DESTROY   6u
#define FR_TAG_THREAD    7u

#ifndef FLIGHT_REC
#define FLIGHT_REC 0
#endif

#if FLIGHT_REC
/* Prints whatever the previous boot left behind, then arms a fresh ring. */
void flight_rec_init(void);
void flight_rec(uint32_t tag, uint64_t a, uint64_t b);
#else
static inline void flight_rec_init(void) {}
static inline void flight_rec(uint32_t tag, uint64_t a, uint64_t b)
{
    (void)tag; (void)a; (void)b;
}
#endif
