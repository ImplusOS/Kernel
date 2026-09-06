#pragma once

#include <stdint.h>

/* Host stand-in for Core/syscall/Poll_Wait.h. The harness drives Pty.c with a
 * stub scheduler, so parking is just "no, we did not sleep" and notifying is a
 * no-op; the line discipline the tests cover does not depend on either. */
static inline uint64_t poll_wait_generation(void) { return 0u; }
static inline int poll_wait_park(uint64_t generation, uint32_t ms)
{
    (void)generation; (void)ms;
    return 0;
}
static inline void poll_wait_notify(void) { }
