#include "Syscall_Main.h"
#include "Syscall_Epoll.h"
#include "kernel/config.h"
#include "kernel/status.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "Poll_Wait.h"
#include "IPC/UnixSocket.h"
#include "interfaces/hal_cpu.h"
#include "Debug/serial/Serial.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * epoll/eventfd - TODO_Chromium_LinuxABI.md section 3.4.
 *
 * epoll_wait() readiness is now computed for real (each registered fd is
 * polled via the same syscall_file_poll()/syscall_socket_poll()/eventfd
 * readiness checks used elsewhere), instead of unconditionally reporting
 * every registered fd as ready every call.
 *
 * True "block inside this syscall until a wakeup or the timeout, then
 * resume this exact call" is not something this kernel's scheduler
 * supports, and it is worth being precise about why, because the obvious
 * fix -- scan, park, scan again, all inside the syscall -- was tried and
 * is much worse than what is here.
 *
 * process_sleep_current_ms() does not block. It marks the caller
 * PROCESS_STATE_BLOCKED, records a wake deadline and asks for a
 * reschedule, then *returns*; the switch happens on the way back out to
 * userspace, because process_run_next_on_current_cpu() reaches another
 * process by a one-way enter_user_mode() jump, never by a call that
 * returns. So a loop around it does not sleep between rounds -- it spins
 * at full speed with the process marked blocked, re-arming its own wake
 * deadline every iteration. Measured, that turned every X11 round trip
 * into seconds (the request sat in the socket while the server burned a
 * CPU pretending to wait for it) and a 20 s X server startup into 170 s.
 * See Docs/Others/TODO_Chromium_LinuxABI.md section 10.-5.
 *
 * So instead of busy-spinning with hal_cpu_pause() (100% CPU, the
 * pre-existing behavior) or claiming to honor the full timeout in one
 * shot (impossible here), epoll_wait sleeps for a short bounded slice and
 * returns 0 ("nothing ready yet") slightly before the caller's requested
 * timeout when nothing is ready, actually yielding the CPU via
 * process_sleep_current_ms()+request_switch in the meantime. This is
 * safe for any well-behaved event loop - including Chromium's - because
 * real epoll_wait() can already return early due to EINTR and all
 * production callers already loop on their own deadline rather than
 * trusting a single call to sleep the exact requested duration.
 *
 * That slice is a *ceiling*, not a fixed cost: the sleep is registered with
 * Core/syscall/Poll_Wait.h, so anything that makes an fd ready cuts it short
 * immediately. Without that the slice was a hard latency floor under every
 * readiness event in the system and X11 round trips cost ~16 ms each no
 * matter how fast the machine was.
 */

#ifndef PROCESS_STALL_DUMP
#define PROCESS_STALL_DUMP 0
#endif

/*
 * One epoll instance per thread that runs a message loop, and Chromium has
 * far more than sixteen of those once it is not being told to switch its
 * subsystems off: with the first-run flow, Sync, background networking and
 * the component updater all running, epoll_create1() started returning EMFILE
 * about six minutes in and the browser died on
 *   Check failed: epoll_.is_valid(). : Too many open files (24)
 * (message_pump_epoll.cc). The entry count per instance is what one of those
 * pumps watches at once; the browser's IO thread alone is well past 64.
 *
 * Cost is .bss: EPOLL_MAX_INSTANCES * EPOLL_MAX_ENTRIES * sizeof(epoll_entry_t).
 * The epfd numbering is 0x4000 + index and must stay below
 * EPOLL_EVENTFD_FD_BASE (0x5000), so the instance count has 4096 of headroom.
 */
#define EPOLL_MAX_INSTANCES   128
#define EPOLL_MAX_ENTRIES     192
/* eventfd is how Chromium signals between its threads; 32 is a handful of
 * WaitableEvents. */
#define EVENTFD_MAX_INSTANCES 128
#define EPOLL_CTL_ADD 1
#define EPOLL_CTL_DEL 2
#define EPOLL_CTL_MOD 3

#define EPOLLIN      0x001u
#define EPOLLOUT     0x004u
#define EPOLLERR     0x008u
#define EPOLLHUP     0x010u
#define EPOLLET      (1u << 31)

/* Bounded slice a single epoll_wait() call sleeps for when nothing is
 * ready and the caller allows blocking (timeout_ms != 0). Small enough
 * to stay responsive, large enough to not dominate scheduling overhead. */
#define EPOLL_POLL_SLICE_MS 1u



/* Must match Syscall_Socket.c's SOCKET_FD_BASE/SOCKET_TABLE_SIZE - kept
 * as a separate small constant here (rather than a shared header) the
 * same way Syscall_Socket.c already cross-references config.h in a
 * comment; there is no runtime dependency, just a documented invariant. */
#define EPOLL_SOCKET_FD_BASE  512
#define EPOLL_SOCKET_FD_COUNT 256
#define EPOLL_EVENTFD_FD_BASE 0x5000

#define LINUX_EFD_SEMAPHORE 1u

typedef struct {
    int      fd;
    uint32_t events;
    uint64_t data;
    /* EPOLLET bookkeeping: the readiness mask last reported for this fd.
     * In edge-triggered mode an entry is only returned when it has ready
     * bits that were not set at the previous report (a rising edge); the
     * mask is cleared again once the fd stops being ready. Unused for
     * level-triggered entries. */
    uint32_t last_ready;
    /* ...and the fd's arrival counter as of that report, where it has one.
     *
     * A rising edge in the sampled readiness mask is not the same event Linux
     * arms an edge-triggered fd on. Linux arms on arrival; this poller only
     * sees the level, and a second arrival while the fd is already readable
     * does not change it. The X server registers its client sockets
     * edge-triggered, so one short read left 20 bytes of a Chromium request
     * queued in the kernel with no further edge ever reported and both
     * processes asleep -- the machine idle and the browser hung. Comparing
     * the arrival counter recovers the missing event.
     * See TODO_Chromium_LinuxABI.md section 10.-5. */
    uint32_t last_seq;
} epoll_entry_t;

/* `refs` counts the processes holding a descriptor for the instance. There
 * was no close at all before: every epoll_create1() leaked its slot for the
 * life of the system, which is what actually exhausted the table in a long
 * Chromium session (one instance per message-loop thread, and threads come
 * and go). A fork() that hands the descriptor to the child is a second
 * reference; the instance goes away with the last one. */
typedef struct {
    uint8_t       used;
    uint32_t      refs;
    int32_t       owner_pid;   /* process that created it (memory owner) */
    uint32_t      count;
    epoll_entry_t entries[EPOLL_MAX_ENTRIES];
} epoll_instance_t;

typedef struct {
    uint8_t  used;
    uint32_t refs;
    uint64_t counter;
    int      flags;
} eventfd_instance_t;

static epoll_instance_t  g_epoll_instances[EPOLL_MAX_INSTANCES];
static eventfd_instance_t g_eventfd_instances[EVENTFD_MAX_INSTANCES];
static spinlock_t        g_epoll_lock;
static spinlock_t        g_eventfd_lock;
static int               g_epoll_initialized = 0;

extern uint32_t syscall_file_poll(int32_t fd, uint32_t events);
extern uint32_t syscall_socket_poll(int32_t fd, uint32_t events);

static void epoll_ensure_init(void)
{
    if (!g_epoll_initialized) {
        spinlock_init(&g_epoll_lock);
        spinlock_init(&g_eventfd_lock);
        for (int i = 0; i < EPOLL_MAX_INSTANCES; ++i) {
            g_epoll_instances[i].used  = 0;
            g_epoll_instances[i].count = 0;
        }
        for (int i = 0; i < EVENTFD_MAX_INSTANCES; ++i) {
            g_eventfd_instances[i].used = 0;
        }
        g_epoll_initialized = 1;
    }
}

int32_t syscall_epoll_create(uint64_t flags)
{
    (void)flags;
    epoll_ensure_init();
    spinlock_lock(&g_epoll_lock);
    for (int i = 0; i < EPOLL_MAX_INSTANCES; ++i) {
        if (!g_epoll_instances[i].used) {
            g_epoll_instances[i].used  = 1;
            g_epoll_instances[i].refs  = 1;
            g_epoll_instances[i].owner_pid =
                process_memory_owner_pid_of(process_get_current_pid());
            g_epoll_instances[i].count = 0;
            spinlock_unlock(&g_epoll_lock);
            return (int32_t)(0x4000 + i);
        }
    }
    spinlock_unlock(&g_epoll_lock);
    return -24;
}

int syscall_epoll_is_valid(int32_t epfd)
{
    int idx = epfd - 0x4000;
    if (idx < 0 || idx >= EPOLL_MAX_INSTANCES) return 0;
    epoll_ensure_init();
    spinlock_lock(&g_epoll_lock);
    int used = g_epoll_instances[idx].used;
    spinlock_unlock(&g_epoll_lock);
    return used;
}

/* One more process holds this epoll descriptor (fork). */
void syscall_epoll_addref(int32_t epfd)
{
    int idx = epfd - 0x4000;
    if (idx < 0 || idx >= EPOLL_MAX_INSTANCES) return;
    epoll_ensure_init();
    spinlock_lock(&g_epoll_lock);
    if (g_epoll_instances[idx].used) ++g_epoll_instances[idx].refs;
    spinlock_unlock(&g_epoll_lock);
}

/* close(2) of an epoll descriptor: drop one reference, free on the last. */
int32_t syscall_epoll_close(int32_t epfd)
{
    int idx = epfd - 0x4000;
    if (idx < 0 || idx >= EPOLL_MAX_INSTANCES) return -9;
    epoll_ensure_init();
    spinlock_lock(&g_epoll_lock);
    epoll_instance_t *inst = &g_epoll_instances[idx];
    if (!inst->used) {
        spinlock_unlock(&g_epoll_lock);
        return -9;
    }
    if (inst->refs > 1u) {
        --inst->refs;
    } else {
        inst->used = 0;
        inst->refs = 0;
        inst->count = 0;
    }
    spinlock_unlock(&g_epoll_lock);
    return 0;
}

/* A process has closed its last descriptor for `fd`: take it out of that
 * process's interest lists.
 * Linux does this implicitly when the open file description goes away. Here
 * descriptor numbers are recycled, so a stale entry would go on to watch
 * whatever object is handed that number next. */
void syscall_epoll_forget_fd_for(int32_t fd, int32_t owner_pid)
{
    epoll_ensure_init();
    spinlock_lock(&g_epoll_lock);
    for (int i = 0; i < EPOLL_MAX_INSTANCES; ++i) {
        epoll_instance_t *inst = &g_epoll_instances[i];
        if (!inst->used) continue;
        /* Only the closing process's own interest lists: another process
         * that still holds the object is still legitimately watching it. */
        if (inst->owner_pid != owner_pid) continue;
        uint32_t w = 0;
        for (uint32_t r = 0; r < inst->count; ++r) {
            if (inst->entries[r].fd == fd) continue;
            if (w != r) inst->entries[w] = inst->entries[r];
            ++w;
        }
        inst->count = w;
    }
    spinlock_unlock(&g_epoll_lock);
}

static epoll_instance_t *epoll_lookup(int32_t epfd)
{
    int idx = epfd - 0x4000;
    if (idx < 0 || idx >= EPOLL_MAX_INSTANCES) return NULL;
    return g_epoll_instances[idx].used ? &g_epoll_instances[idx] : NULL;
}

static uint32_t epoll_fd_ready_seq(int32_t fd);

/* Could `fd` ever name something one of the fd tables owns?
 *
 * epoll_ctl() on Linux rejects a bad fd with EBADF; this accepted anything.
 * That mattered because epoll_poll_fd() answers EPOLLERR for an fd it cannot
 * place, and EPOLLERR is always reported -- so one junk entry made
 * epoll_wait() return an event immediately, forever. Xorg's dbus-core module
 * does exactly this: its bus connection fails, it hands the failed call's
 * negative return to SetNotifyFd() as an fd, and the X server then spun a
 * whole CPU re-entering epoll_wait and starved every client on the machine
 * (Chromium among them). See TODO_Chromium_LinuxABI.md section 10.-5. */
static int epoll_fd_is_addressable(int32_t fd)
{
    if (fd < 0) {
        return 0;
    }
    if (fd < (int32_t)OS_CONFIG_FILE_MAX_FD) {
        return 1;
    }
    if (fd >= EPOLL_SOCKET_FD_BASE &&
        fd < EPOLL_SOCKET_FD_BASE + EPOLL_SOCKET_FD_COUNT) {
        return 1;
    }
    if (fd >= EPOLL_EVENTFD_FD_BASE &&
        fd < EPOLL_EVENTFD_FD_BASE + EVENTFD_MAX_INSTANCES) {
        return 1;
    }
    return unix_socket_fd_in_range(fd);
}

int32_t syscall_epoll_ctl(int32_t epfd, int32_t op, int32_t fd,
                          const epoll_event_t *event)
{
    epoll_ensure_init();
    spinlock_lock(&g_epoll_lock);
    epoll_instance_t *inst = epoll_lookup(epfd);
    if (!inst) {
        spinlock_unlock(&g_epoll_lock);
        return -9;
    }

    if (!epoll_fd_is_addressable(fd)) {
        spinlock_unlock(&g_epoll_lock);
        return -9; /* EBADF */
    }

    uint32_t found = inst->count;
    for (uint32_t i = 0; i < inst->count; ++i) {
        if (inst->entries[i].fd == fd) {
            found = i;
            break;
        }
    }

    if (op == EPOLL_CTL_ADD) {
        if (found != inst->count) {
            spinlock_unlock(&g_epoll_lock);
            return -17; /* EEXIST */
        }
        if (inst->count >= EPOLL_MAX_ENTRIES) {
            spinlock_unlock(&g_epoll_lock);
            return -12;
        }
        epoll_entry_t *e = &inst->entries[inst->count++];
        e->fd     = fd;
        e->events = event ? event->events : (EPOLLIN | EPOLLOUT);
        e->data   = event ? event->data : 0;
        e->last_ready = 0u;
        e->last_seq   = epoll_fd_ready_seq(fd);
    } else if (op == EPOLL_CTL_DEL) {
        if (found == inst->count) {
            spinlock_unlock(&g_epoll_lock);
            return -2; /* ENOENT */
        }
        inst->entries[found] = inst->entries[--inst->count];
    } else if (op == EPOLL_CTL_MOD) {
        if (found == inst->count) {
            spinlock_unlock(&g_epoll_lock);
            return -2; /* ENOENT */
        }
        inst->entries[found].events =
            event ? event->events : (EPOLLIN | EPOLLOUT);
        inst->entries[found].data = event ? event->data : 0;
        inst->entries[found].last_ready = 0u; /* re-arm the edge */
        inst->entries[found].last_seq = epoll_fd_ready_seq(fd);
    } else {
        spinlock_unlock(&g_epoll_lock);
        return -22; /* EINVAL */
    }

    spinlock_unlock(&g_epoll_lock);
    return 0;
}

/* EPOLLIN/EPOLLOUT readiness for an eventfd; caller already range-checked
 * `fd` against EPOLL_EVENTFD_FD_BASE..+EVENTFD_MAX_INSTANCES. */
static uint32_t eventfd_poll_locked(int32_t fd, uint32_t requested)
{
    int idx = fd - EPOLL_EVENTFD_FD_BASE;
    spinlock_lock(&g_eventfd_lock);
    eventfd_instance_t *e = &g_eventfd_instances[idx];
    uint32_t ready = 0;
    if (e->used) {
        if ((requested & EPOLLIN) != 0u && e->counter != 0u) ready |= EPOLLIN;
        /* Writes saturate rather than block (see syscall_eventfd_write),
         * so an eventfd is always writable in this implementation. */
        if ((requested & EPOLLOUT) != 0u) ready |= EPOLLOUT;
    } else {
        ready = EPOLLERR;
    }
    spinlock_unlock(&g_eventfd_lock);
    return ready;
}

/* Dispatches a readiness check to whichever fd-table `fd` actually lives
 * in (regular/pipe/timerfd/memfd/signalfd, socket, or eventfd - these are
 * disjoint numeric ranges, see kernel/config.h's OS_CONFIG_FILE_MAX_FD
 * comment and Syscall_Socket.c's SOCKET_FD_BASE comment). */
/* The fd's arrival counter, or 0 for a kind of fd that does not keep one.
 * An fd with no counter keeps the pre-existing level-transition behaviour:
 * its seq never changes, so it never forces an edge on its own. */
static uint32_t epoll_fd_ready_seq(int32_t fd)
{
    if (unix_socket_fd_in_range(fd)) {
        return unix_socket_rx_seq(fd);
    }
    return 0u;
}

static uint32_t epoll_poll_fd(int32_t fd, uint32_t requested)
{
    /* AF_UNIX first: its range sits inside the file table's numeric span. */
    if (unix_socket_fd_in_range(fd)) {
        return unix_socket_poll(fd, requested);
    }
    if (fd >= 0 && fd < (int32_t)OS_CONFIG_FILE_MAX_FD) {
        return syscall_file_poll(fd, requested);
    }
    if (fd >= EPOLL_SOCKET_FD_BASE &&
        fd < EPOLL_SOCKET_FD_BASE + EPOLL_SOCKET_FD_COUNT) {
        return syscall_socket_poll(fd, requested);
    }
    if (fd >= EPOLL_EVENTFD_FD_BASE &&
        fd < EPOLL_EVENTFD_FD_BASE + EVENTFD_MAX_INSTANCES) {
        return eventfd_poll_locked(fd, requested);
    }
    if (unix_socket_fd_in_range(fd)) {
        return unix_socket_poll(fd, requested);
    }
    return EPOLLERR;
}

/* Public one-shot readiness probe for a single fd, used by the Linux
 * compat layer's poll(2)/ppoll(2). `events`/return use the EPOLL* bit
 * values, which are numerically identical to the POLL* ones
 * (IN=0x1, OUT=0x4, ERR=0x8, HUP=0x10). Never blocks. */
uint32_t syscall_poll_one_fd(int32_t fd, uint32_t events)
{
    epoll_ensure_init();
    return epoll_poll_fd(fd, events);
}

/* One non-blocking readiness pass over every fd registered on `epfd`.
 * Returns the number of ready entries written to `events` (capped at
 * maxevents), or <0 on error. */
static int32_t epoll_check_once(int32_t epfd, epoll_event_t *events,
                                int32_t maxevents)
{
    spinlock_lock(&g_epoll_lock);
    epoll_instance_t *inst = epoll_lookup(epfd);
    if (!inst) {
        spinlock_unlock(&g_epoll_lock);
        return -9;
    }
    /* Snapshot under the lock (fd/events/data are small, count is
     * bounded), then poll each fd without the epoll table lock held -
     * syscall_file_poll()/syscall_socket_poll() take their own locks
     * and must not be called while holding this one. */
    epoll_entry_t snapshot[EPOLL_MAX_ENTRIES];
    uint32_t count = inst->count;
    memcpy(snapshot, inst->entries, count * sizeof(epoll_entry_t));
    spinlock_unlock(&g_epoll_lock);

    int32_t n = 0;
    /* Collected edge-trigger state updates, applied under the lock afterwards
     * (epoll_poll_fd() must not run while g_epoll_lock is held). */
    int      et_fd[EPOLL_MAX_ENTRIES];
    uint32_t et_ready[EPOLL_MAX_ENTRIES];
    uint32_t et_seq[EPOLL_MAX_ENTRIES];
    uint32_t et_n = 0;

    for (uint32_t i = 0; i < count && n < maxevents; ++i) {
        uint32_t requested = snapshot[i].events & (EPOLLIN | EPOLLOUT);
        if (requested == 0u) {
            requested = EPOLLIN | EPOLLOUT; /* Always watch for errors/hup. */
        }
        uint32_t ready = epoll_poll_fd(snapshot[i].fd, requested);
        ready &= (snapshot[i].events | EPOLLERR | EPOLLHUP);

        uint32_t deliver = ready;
        if ((snapshot[i].events & EPOLLET) != 0u) {
            /* Edge-triggered: surface bits that were not ready at the previous
             * report, plus everything still ready if something has arrived
             * since -- that arrival is the edge, and the level alone cannot
             * show it. EPOLLERR/EPOLLHUP are always surfaced (Linux delivers
             * them regardless; over-notification is safe for a correct
             * drain-until-EAGAIN consumer). Record both, so the fd must go
             * quiet, or take a fresh arrival, before it can edge again. */
            uint32_t seq = epoll_fd_ready_seq(snapshot[i].fd);
            uint32_t since_report = (seq != snapshot[i].last_seq) ? ready : 0u;
            deliver = (ready & ~snapshot[i].last_ready) | since_report |
                      (ready & (EPOLLERR | EPOLLHUP));
            et_fd[et_n]    = snapshot[i].fd;
            et_ready[et_n] = ready;
            et_seq[et_n]   = seq;
            ++et_n;
        }

        if (deliver != 0u) {
            events[n].events = deliver;
            events[n].data   = snapshot[i].data;
            ++n;
        }
    }

    if (et_n != 0u) {
        spinlock_lock(&g_epoll_lock);
        epoll_instance_t *inst2 = epoll_lookup(epfd);
        if (inst2) {
            for (uint32_t k = 0; k < et_n; ++k) {
                for (uint32_t j = 0; j < inst2->count; ++j) {
                    if (inst2->entries[j].fd == et_fd[k]) {
                        inst2->entries[j].last_ready = et_ready[k];
                        inst2->entries[j].last_seq   = et_seq[k];
                        break;
                    }
                }
            }
        }
        spinlock_unlock(&g_epoll_lock);
    }
    return n;
}

int32_t syscall_epoll_wait(int32_t epfd, epoll_event_t *events,
                           int32_t maxevents, int32_t timeout_ms)
{
    int should_switch = 0;
    return syscall_epoll_wait_ex(epfd, events, maxevents, timeout_ms,
                                 &should_switch);
}

int32_t syscall_epoll_wait_ex(int32_t epfd, epoll_event_t *events,
                              int32_t maxevents, int32_t timeout_ms,
                              int *should_switch_out)
{
    epoll_ensure_init();
    *should_switch_out = 0;

    if (!events || maxevents <= 0) {
        return -22;
    }

    /* Taken before the readiness scan so a wakeup that lands during the scan
     * is not lost -- see Poll_Wait.h. */
    uint64_t generation = poll_wait_generation();

    int32_t n = epoll_check_once(epfd, events, maxevents);
    if (n != 0) {
        return n; /* Ready (n>0) or an error (n<0): return immediately. */
    }
    if (timeout_ms == 0) {
        return 0; /* Pure poll: caller asked to never block. */
    }

    uint32_t slice_ms = EPOLL_POLL_SLICE_MS;
    if (timeout_ms > 0 && (uint32_t)timeout_ms < slice_ms) {
        slice_ms = (uint32_t)timeout_ms;
    }
    if (poll_wait_park(generation, slice_ms) != 0) {
        *should_switch_out = 1;
    }
    return 0;
}

#if PROCESS_STALL_DUMP
/* Every epoll set with its per-fd readiness, for the timer-driven stall dump.
 * "Which fd keeps reporting itself ready" is not answerable from the syscall
 * counts alone, and an fd that is wrongly always-ready turns a well-behaved
 * event loop into a spin that starves everything else on the machine. */
void epoll_debug_dump(void)
{
    /* Static, not automatic: this runs from the timer interrupt, and a
     * EPOLL_MAX_ENTRIES-wide snapshot is over a kilobyte. On the interrupt
     * stack that was enough to trip the kernel's own stack canary and panic
     * with "stack smashing detected" at SMP=4 -- a diagnostic that crashes
     * the thing it is measuring. Serialised by g_epoll_lock below, and the
     * dump is single-threaded by construction (CPU 0's timer tick). */
    static epoll_entry_t snapshot[EPOLL_MAX_ENTRIES];

    for (int i = 0; i < EPOLL_MAX_INSTANCES; ++i) {
        uint32_t count;

        spinlock_lock(&g_epoll_lock);
        if (!g_epoll_instances[i].used) {
            spinlock_unlock(&g_epoll_lock);
            continue;
        }
        count = g_epoll_instances[i].count;
        memcpy(snapshot, g_epoll_instances[i].entries,
               count * sizeof(epoll_entry_t));
        spinlock_unlock(&g_epoll_lock);

        serial_write_string("[epoll] ");
        serial_write_uint32((uint32_t)(0x4000 + i));
        serial_write_string(" n=");
        serial_write_uint32(count);
        for (uint32_t k = 0; k < count; ++k) {
            uint32_t requested = snapshot[k].events & (EPOLLIN | EPOLLOUT);
            if (requested == 0u) {
                requested = EPOLLIN | EPOLLOUT;
            }
            uint32_t ready = epoll_poll_fd(snapshot[k].fd, requested) &
                             (snapshot[k].events | EPOLLERR | EPOLLHUP);
            serial_write_string(" ");
            serial_write_uint32((uint32_t)snapshot[k].fd);
            serial_write_string("/w");
            serial_write_uint32(snapshot[k].events);
            serial_write_string("/r");
            serial_write_uint32(ready);
        }
        serial_write_char('\n');
    }
}
#endif

int32_t syscall_eventfd(uint64_t initval, uint64_t flags)
{
    epoll_ensure_init();
    spinlock_lock(&g_eventfd_lock);
    for (int i = 0; i < EVENTFD_MAX_INSTANCES; ++i) {
        if (!g_eventfd_instances[i].used) {
            g_eventfd_instances[i].used    = 1;
            g_eventfd_instances[i].refs    = 1;
            g_eventfd_instances[i].counter = initval;
            g_eventfd_instances[i].flags   = (int)flags;
            spinlock_unlock(&g_eventfd_lock);
            return (int32_t)(EPOLL_EVENTFD_FD_BASE + i);
        }
    }
    spinlock_unlock(&g_eventfd_lock);
    return -24;
}

int syscall_eventfd_is_valid(int32_t fd)
{
    int idx = fd - EPOLL_EVENTFD_FD_BASE;
    if (idx < 0 || idx >= EVENTFD_MAX_INSTANCES) return 0;
    spinlock_lock(&g_eventfd_lock);
    int used = g_eventfd_instances[idx].used;
    spinlock_unlock(&g_eventfd_lock);
    return used;
}

int64_t syscall_eventfd_read(int32_t fd, uint8_t *buffer, uint64_t len)
{
    if (len < 8u || buffer == NULL) return -22; /* EINVAL */
    int idx = fd - EPOLL_EVENTFD_FD_BASE;
    if (idx < 0 || idx >= EVENTFD_MAX_INSTANCES) return -9; /* EBADF */

    spinlock_lock(&g_eventfd_lock);
    eventfd_instance_t *e = &g_eventfd_instances[idx];
    if (!e->used) {
        spinlock_unlock(&g_eventfd_lock);
        return -9;
    }
    if (e->counter == 0u) {
        spinlock_unlock(&g_eventfd_lock);
        /* Real Linux would block here (or EAGAIN if O_NONBLOCK); this
         * implementation only supports the non-blocking style (see
         * header comment) since Chromium/glibc always pair eventfd with
         * epoll and only read after epoll reports EPOLLIN. */
        return -11; /* EAGAIN */
    }
    uint64_t value;
    if ((e->flags & (int)LINUX_EFD_SEMAPHORE) != 0) {
        value = 1u;
        e->counter -= 1u;
    } else {
        value = e->counter;
        e->counter = 0u;
    }
    spinlock_unlock(&g_eventfd_lock);

    memcpy(buffer, &value, sizeof(value));
    return (int64_t)sizeof(value);
}

int64_t syscall_eventfd_write(int32_t fd, const uint8_t *buffer, uint64_t len)
{
    if (len < 8u || buffer == NULL) return -22;
    int idx = fd - EPOLL_EVENTFD_FD_BASE;
    if (idx < 0 || idx >= EVENTFD_MAX_INSTANCES) return -9;

    uint64_t value;
    memcpy(&value, buffer, sizeof(value));
    if (value == 0xFFFFFFFFFFFFFFFFULL) return -22; /* Linux disallows this. */

    spinlock_lock(&g_eventfd_lock);
    eventfd_instance_t *e = &g_eventfd_instances[idx];
    if (!e->used) {
        spinlock_unlock(&g_eventfd_lock);
        return -9;
    }
    uint64_t new_counter = e->counter + value;
    if (new_counter < e->counter) {
        /* Would overflow: real Linux blocks the writer until there is
         * room. Saturate instead of blocking (see header comment on why
         * this kernel cannot suspend mid-syscall); a wraparound this
         * large in practice means something is already very wrong. */
        new_counter = 0xFFFFFFFFFFFFFFFEULL;
    }
    e->counter = new_counter;
    spinlock_unlock(&g_eventfd_lock);
    /* An eventfd write is the entire point of an eventfd: it exists to break
     * somebody out of an epoll_wait(). */
    poll_wait_notify();
    return (int64_t)sizeof(value);
}

int32_t syscall_eventfd_close(int32_t fd)
{
    int idx = fd - EPOLL_EVENTFD_FD_BASE;
    if (idx < 0 || idx >= EVENTFD_MAX_INSTANCES) return -9;
    spinlock_lock(&g_eventfd_lock);
    if (g_eventfd_instances[idx].refs > 1u) {
        --g_eventfd_instances[idx].refs;
    } else {
        g_eventfd_instances[idx].used = 0;
        g_eventfd_instances[idx].refs = 0;
    }
    spinlock_unlock(&g_eventfd_lock);
    return 0;
}

/* One more process holds this eventfd (fork). */
void syscall_eventfd_addref(int32_t fd)
{
    int idx = fd - EPOLL_EVENTFD_FD_BASE;
    if (idx < 0 || idx >= EVENTFD_MAX_INSTANCES) return;
    spinlock_lock(&g_eventfd_lock);
    if (g_eventfd_instances[idx].used) ++g_eventfd_instances[idx].refs;
    spinlock_unlock(&g_eventfd_lock);
}
