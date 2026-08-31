#include "StackProtector.h"
#include "Debug/panic/Panic.h"

/*
 * -fstack-protector-strong (see Kernel/config/arch.mk, added under
 * Docs/Others/TODO_OS_Refactor.md phase P3 item 3) makes GCC emit, for any
 * function with a stack frame it judges worth protecting, a prologue that
 * copies the extern global `__stack_chk_guard` next to the frame's local
 * arrays, and an epilogue that compares it back and calls
 * `__stack_chk_fail()` on mismatch. Both symbols are plain C globals GCC
 * expects some translation unit to provide at link time -- normally libc
 * supplies them, but this kernel is `-nostdlib`, so this file is their
 * sole definition.
 *
 * The guard starts as a non-zero, non-trivial `.data` constant rather than
 * the default zero-initialized `.bss` value: stack protection is live from
 * the very first protected function's prologue, which runs long before any
 * entropy source (timer, RTC, RDRAND/CNTVCT) is initialized -- a
 * build-time constant is strictly better than 0 (which a classic
 * off-by-one NUL-terminator overflow can trivially reproduce) for that
 * earliest window. stack_protector_reseed() strengthens it with runtime
 * entropy as soon as the caller has any to offer; see its declaration in
 * StackProtector.h for when to call it.
 */
uintptr_t __stack_chk_guard = 0x595E9FBD94FDA766ULL;

void __attribute__((noreturn)) __stack_chk_fail(void)
{
    kernel_panic("stack_protector", "stack smashing detected");
    /* kernel_panic() halts forever internally and never returns; this is
     * just defensive belt-and-suspenders so a future change to panic()
     * can't accidentally turn a caught stack overflow into a silent
     * return-into-corrupted-frame. */
    for (;;) { }
}

static inline uint64_t stack_protector_read_cycles(void)
{
#if defined(PLATFORM_X86_64)
    uint32_t lo = 0, hi = 0;
    __asm__ volatile("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | (uint64_t)lo;
#elif defined(PLATFORM_ARM64)
    uint64_t cntvct = 0;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(cntvct));
    return cntvct;
#else
    return 0;
#endif
}

void stack_protector_reseed(uint64_t extra_entropy)
{
    uintptr_t mixed = (uintptr_t)stack_protector_read_cycles() ^
                       (uintptr_t)extra_entropy ^
                       (uintptr_t)&__stack_chk_guard;
    __stack_chk_guard ^= mixed;

    /* Never let a reseed collapse the guard to 0 or back to the build-time
     * constant -- both are values worth ruling out even if the entropy
     * mix happens to cancel out. */
    if (__stack_chk_guard == 0 || __stack_chk_guard == 0x595E9FBD94FDA766ULL) {
        __stack_chk_guard = mixed | 0x1u;
    }
}
