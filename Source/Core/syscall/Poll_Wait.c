#include "Poll_Wait.h"

#include <stdint.h>

#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "kernel/config.h"

#define POLL_WAIT_MASK_WORDS ((OS_CONFIG_PROCESS_MAX_COUNT + 63) / 64)

/* Consecutive times a process has been let off a sleep because the generation
 * moved under it. Bounded, because "the generation moved" is a global signal:
 * any other process making any fd ready trips it, so a busy system can trip it
 * every single time. A caller with an infinite timeout re-enters the syscall
 * from userspace (linux_syscall_restart rewinds RIP), so a park that never
 * sleeps becomes a 100% CPU spin that never yields -- which is exactly what
 * happened once a terminal was streaming the kernel log: every line written to
 * the pty bumped the generation, and Xorg and its client both stopped making
 * progress. After POLL_WAIT_MAX_DECLINES in a row the park sleeps regardless.
 * 0 disables the optimisation entirely (every park sleeps), which is the
 * setting to fall back to when deciding whether a stall is this code's
 * fault. */
#ifndef POLL_WAIT_MAX_DECLINES
#define POLL_WAIT_MAX_DECLINES 1u
#endif

static uint64_t  g_parked[POLL_WAIT_MASK_WORDS];
static uint8_t   g_declines[OS_CONFIG_PROCESS_MAX_COUNT];
static uint64_t  g_generation;
/* Zero is the unlocked state, so the static initialiser is the initialisation.
 * There is deliberately no lazy spinlock_init() here: the first two CPUs to
 * arrive would both see "not initialised yet" and both reset the lock word --
 * the second one clearing it out from under the first -- and let two CPUs into
 * the critical section at once. That is a hard race to see, because it only
 * needs to happen once and the damage (a corrupted parked-pid mask) shows up
 * later and somewhere else. */
static spinlock_t g_poll_lock;

/* Taken from process context (a parking poller) and from interrupt context
 * (a NIC or input IRQ that made an fd readable), so every acquisition masks
 * interrupts -- the same rule Syscall_Futex.c documents. */
static inline uint64_t poll_lock_irq(void)
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_poll_lock);
    return flags;
}

static inline void poll_unlock_irq(uint64_t flags)
{
    spinlock_unlock(&g_poll_lock);
    irq_restore(flags);
}

uint64_t poll_wait_generation(void)
{
#if !OS_CONFIG_POLL_WAIT_WAKEUPS
    return 0u;
#else
    uint64_t flags = poll_lock_irq();
    uint64_t generation = g_generation;
    poll_unlock_irq(flags);
    return generation;
#endif
}

int poll_wait_park(uint64_t generation, uint32_t ms)
{
#if !OS_CONFIG_POLL_WAIT_WAKEUPS
    (void)generation;
    return process_sleep_current_ms(ms) == 0 ? 1 : 0;
#else
    /* The calling THREAD's slot, not its process: process_sleep_current_ms()
     * blocks the slot that is running and process_wake_pid() wakes exactly
     * the slot it is given. Registering the thread-group id instead meant a
     * notify could only ever wake a multi-threaded program's main thread,
     * while every other thread parked in poll/epoll served out its whole
     * slice -- and all of them shared one parked bit and one run of declined
     * sleeps, clearing each other's registrations. Chromium, with ~60 threads
     * in epoll_wait/ppoll, spent tens of seconds at start-up with every
     * thread cycling through those syscalls and the window left blank. */
    int32_t pid = process_get_current_tid();
    if (pid < 0 || (uint32_t)pid >= (uint32_t)OS_CONFIG_PROCESS_MAX_COUNT ||
        ms == 0u) {
        return 0;
    }
    uint32_t word = (uint32_t)pid / 64u;
    uint64_t bit  = 1ULL << ((uint32_t)pid % 64u);

    uint64_t flags = poll_lock_irq();
    /* Drop any registration left over from last time first.
     *
     * A park ends one of two ways: poll_wait_notify() woke it, which clears
     * the bit, or the deadline simply expired, which does not -- nothing runs
     * on the timer path to clean up after it. A bit left set that way is not
     * harmless: every later notify calls process_wake_pid() on a process that
     * is not blocked, which banks a wake credit, and credits are what
     * process_sleep_current_ms() spends instead of sleeping. Clearing here
     * bounds the staleness to the window between waking and the next park. */
    g_parked[word] &= ~bit;

    /* Two separate things can talk this call out of sleeping: a generation
     * that moved (an event landed during the caller's scan) and a banked wake
     * credit. Both are useful once and ruinous in a loop -- a caller with an
     * infinite timeout re-runs the syscall from userspace, so a park that
     * never sleeps is a 100% CPU spin. Once this process has skipped
     * POLL_WAIT_MAX_DECLINES sleeps in a row, the next one is taken no matter
     * what either says. */
    int forced = (g_declines[pid] >= POLL_WAIT_MAX_DECLINES);
    if (!forced && g_generation != generation) {
        ++g_declines[pid];
        poll_unlock_irq(flags);
        return 1;   /* switch away and rescan, rather than spin */
    }
    g_parked[word] |= bit;
    poll_unlock_irq(flags);

    (void)forced;
    int slept = (process_sleep_current_ms(ms) == 0);

    int missed = 0;
    flags = poll_lock_irq();
    if (slept) {
        /* Registered and actually blocked: leave the bit set so a notify can
         * find us, and forget the run of skipped sleeps.
         *
         * Except if the bit is already gone, which means a notify ran in the
         * gap between unlocking above and reaching BLOCKED just now. It found
         * this process still runnable, so its process_wake_pid() had nothing
         * to wake, and the sleep that has only now started would serve out the
         * whole slice on top of an event that has already happened. Undo it.
         * This is the last lost-wakeup window, and closing it here -- rather
         * than by spending the process manager's wake_pending credit, which
         * belongs to process_block_current() and to futexes -- is what keeps
         * this mechanism from interfering with unrelated sleeps. */
        missed = ((g_parked[word] & bit) == 0u);
        g_declines[pid] = 0u;
    } else {
        g_parked[word] &= ~bit;
        if (g_declines[pid] < 0xFFu) {
            ++g_declines[pid];
        }
    }
    poll_unlock_irq(flags);

    if (missed) {
        (void)process_wake_pid(pid);
    }
    return 1;
#endif
}

/* Woken in batches so this never puts a kilobyte of pid array on the 32 KiB
 * kernel stack, and never holds g_poll_lock (with interrupts masked) across
 * an unbounded run of process_wake_pid() calls. */
#define POLL_WAIT_WAKE_BATCH 32u

void poll_wait_notify(void)
{
#if !OS_CONFIG_POLL_WAIT_WAKEUPS
    return;
#else
    int32_t  batch[POLL_WAIT_WAKE_BATCH];
    uint32_t count;

    uint64_t flags = poll_lock_irq();
    ++g_generation;
    poll_unlock_irq(flags);

    do {
        count = 0u;
        flags = poll_lock_irq();
        for (uint32_t word = 0;
             word < POLL_WAIT_MASK_WORDS && count < POLL_WAIT_WAKE_BATCH;
             ++word) {
            while (g_parked[word] != 0u && count < POLL_WAIT_WAKE_BATCH) {
                uint32_t bit = (uint32_t)__builtin_ctzll(g_parked[word]);
                g_parked[word] &= g_parked[word] - 1u;
                batch[count++] = (int32_t)(word * 64u + bit);
            }
        }
        poll_unlock_irq(flags);

        /* Outside the lock: process_wake_pid() takes the process table lock. */
        for (uint32_t i = 0; i < count; ++i) {
            (void)process_wake_pid(batch[i]);
        }
    } while (count == POLL_WAIT_WAKE_BATCH);
#endif
}
