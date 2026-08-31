#pragma once
#include <stdint.h>

/* Re-seeds __stack_chk_guard (see StackProtector.c) with the best entropy
 * available at the call site, XORed into the existing guard rather than
 * replacing it outright. Call as early as possible in boot once *some*
 * entropy source exists (TSC/CNTVCT_EL0 is enough; a cryptographic-grade
 * source is not required here -- the guard only needs to be
 * hard-to-predict-from-outside-the-kernel, not secret). Safe to call
 * multiple times as better entropy becomes available (e.g. once
 * Library/Crypto/CSPRNG is seeded) -- each call only strengthens the
 * guard, never weakens it. `extra_entropy` is any additional caller-known
 * value (a boot_info pointer, a timer tick count, ...) folded in alongside
 * the architecture's cycle counter. */
void stack_protector_reseed(uint64_t extra_entropy);
