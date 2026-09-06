#pragma once

#include <stdint.h>

/*
 * Wakeups for the poll(2)/ppoll(2)/select(2)/epoll_wait(2) family.
 *
 * This kernel cannot block inside a syscall: process_schedule_on_syscall()
 * only ever resumes a process at the instruction after its `syscall`, never
 * mid-C-function, so "wait for an fd" has to be spelled "sleep briefly, return
 * 0, let the caller ask again" (see Core/syscall/Syscall_Epoll.c). That works,
 * but the sleep was a flat 8 ms with nothing able to cut it short, which put a
 * hard floor of ~8 ms on every readiness event in the system.
 *
 * An X11 round trip crosses that floor twice -- once for the server to notice
 * the request, once for the client to notice the reply -- so it cost ~16 ms no
 * matter how fast the machine was. A GTK or Xorg startup makes thousands of
 * round trips, which is tens of seconds of pure sleeping, and that is why
 * enabling KVM (or moving to real hardware) changed nothing: the time was
 * never being spent executing instructions.
 *
 * So: a parked poller registers itself here first, and anything that makes an
 * fd readable or writable calls poll_wait_notify(), which wakes every parked
 * process immediately. Wakeups are deliberately global rather than per-fd --
 * a spurious wakeup just costs one more readiness scan, which every caller of
 * this family already has to tolerate, and it means a readiness source that
 * has not been taught about this yet degrades to the old timed poll rather
 * than hanging.
 *
 * Two lost-wakeup windows have to be closed, and they need different
 * mechanisms:
 *
 *  - between reading the fds and calling in here, an event may land. The
 *    generation counter catches that: callers read it *before* scanning and
 *    hand it back to poll_wait_park(), which refuses to sleep if it moved.
 *  - between registering here and actually reaching BLOCKED, the process is
 *    still runnable, so a notify's process_wake_pid() has nothing to wake.
 *    The process manager already handles that with its wake_pending credit,
 *    which process_sleep_current_ms() spends instead of sleeping.
 *
 * Both of those talk a park out of sleeping, and both are ruinous if they can
 * do it indefinitely: a caller with an infinite timeout re-runs the syscall
 * from userspace, so a park that never sleeps is a 100% CPU spin. Every
 * skipped sleep is therefore counted, and past POLL_WAIT_MAX_DECLINES in a row
 * the next sleep is taken regardless of what either mechanism says. See the
 * .c, and Docs/Others/TODO_Performance_LinuxApps.md section 9 for what this
 * looked like when it was wrong.
 */

/* Read before scanning fds for readiness. */
uint64_t poll_wait_generation(void);

/* Park the calling process for at most `ms`, unless an event has landed since
 * `generation` was taken -- in which case it returns without sleeping so the
 * caller rescans immediately.
 *
 * Returns non-zero when the caller should let the scheduler switch away, which
 * is every case except an unusable pid. That includes the "did not sleep"
 * path on purpose: a caller with an infinite timeout re-runs the syscall from
 * userspace, so not yielding there is a spin, not a fast path. */
int poll_wait_park(uint64_t generation, uint32_t ms);

/* Something became readable/writable. Wakes every parked poller. Cheap and
 * safe to call from any context, including a timer or device interrupt. */
void poll_wait_notify(void);
