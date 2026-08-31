#include "Syscall_Main.h"
#include "Syscall_Epoll.h"
#include "kernel/config.h"
#include "kernel/status.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "IPC/UnixSocket.h"
#include "interfaces/hal_cpu.h"

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
 * supports: a syscall can only "block" by registering a sleep/blocked
 * state and *returning all the way to userspace* - see
 * process_block_current()/process_sleep_current_ms() and how
 * process_schedule_on_syscall() only ever resumes a process at "right
 * after the `syscall` instruction that invoked it", never mid-C-function
 * (process_run_next_on_current_cpu() confirms this: switching to another
 * process is a one-way jump via enter_user_mode(), not a call that
 * returns). So instead of busy-spinning with hal_cpu_pause() (100% CPU,
 * the pre-existing behavior) or claiming to honor the full timeout in one
 * shot (impossible here), epoll_wait sleeps for a short bounded slice and
 * returns 0 ("nothing ready yet") slightly before the caller's requested
 * timeout when nothing is ready, actually yielding the CPU via
 * process_sleep_current_ms()+request_switch in the meantime. This is
 * safe for any well-behaved event loop - including Chromium's - because
 * real epoll_wait() can already return early due to EINTR and all
 * production callers already loop on their own deadline rather than
 * trusting a single call to sleep the exact requested duration.
 */

#define EPOLL_MAX_INSTANCES   16
#define EPOLL_MAX_ENTRIES     64
#define EVENTFD_MAX_INSTANCES 32
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
#define EPOLL_POLL_SLICE_MS 8u

/* Must match Syscall_Socket.c's SOCKET_FD_BASE/SOCKET_TABLE_SIZE - kept
 * as a separate small constant here (rather than a shared header) the
 * same way Syscall_Socket.c already cross-references config.h in a
 * comment; there is no runtime dependency, just a documented invariant. */
#define EPOLL_SOCKET_FD_BASE  512
#define EPOLL_SOCKET_FD_COUNT 64
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
} epoll_entry_t;

typedef struct {
    uint8_t       used;
    uint32_t      count;
    epoll_entry_t entries[EPOLL_MAX_ENTRIES];
} epoll_instance_t;

typedef struct {
    uint8_t  used;
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
            g_epoll_instances[i].count = 0;
            spinlock_unlock(&g_epoll_lock);
            return (int32_t)(0x4000 + i);
        }
    }
    spinlock_unlock(&g_epoll_lock);
    return -24;
}

static epoll_instance_t *epoll_lookup(int32_t epfd)
{
    int idx = epfd - 0x4000;
    if (idx < 0 || idx >= EPOLL_MAX_INSTANCES) return NULL;
    return g_epoll_instances[idx].used ? &g_epoll_instances[idx] : NULL;
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

    if (op == EPOLL_CTL_ADD) {
        if (inst->count >= EPOLL_MAX_ENTRIES) {
            spinlock_unlock(&g_epoll_lock);
            return -12;
        }
        epoll_entry_t *e = &inst->entries[inst->count++];
        e->fd     = fd;
        e->events = event ? event->events : (EPOLLIN | EPOLLOUT);
        e->data   = event ? event->data : 0;
        e->last_ready = 0u;
    } else if (op == EPOLL_CTL_DEL) {
        for (uint32_t i = 0; i < inst->count; ++i) {
            if (inst->entries[i].fd == fd) {
                inst->entries[i] = inst->entries[--inst->count];
                break;
            }
        }
    } else if (op == EPOLL_CTL_MOD) {
        for (uint32_t i = 0; i < inst->count; ++i) {
            if (inst->entries[i].fd == fd) {
                inst->entries[i].events = event ? event->events : (EPOLLIN | EPOLLOUT);
                inst->entries[i].data   = event ? event->data : 0;
                inst->entries[i].last_ready = 0u; /* re-arm the edge */
                break;
            }
        }
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
static uint32_t epoll_poll_fd(int32_t fd, uint32_t requested)
{
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
            /* Edge-triggered: only surface bits that were not ready at the
             * previous report. EPOLLERR/EPOLLHUP are always surfaced (Linux
             * delivers them regardless; over-notification is safe for a
             * correct drain-until-EAGAIN consumer). Record the new mask so
             * the fd must go quiet before it can edge again. */
            deliver = (ready & ~snapshot[i].last_ready) |
                      (ready & (EPOLLERR | EPOLLHUP));
            et_fd[et_n]    = snapshot[i].fd;
            et_ready[et_n] = ready;
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
    if (process_sleep_current_ms(slice_ms) == 0) {
        *should_switch_out = 1;
    }
    return 0;
}

int32_t syscall_eventfd(uint64_t initval, uint64_t flags)
{
    epoll_ensure_init();
    spinlock_lock(&g_eventfd_lock);
    for (int i = 0; i < EVENTFD_MAX_INSTANCES; ++i) {
        if (!g_eventfd_instances[i].used) {
            g_eventfd_instances[i].used    = 1;
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
    return (int64_t)sizeof(value);
}

int32_t syscall_eventfd_close(int32_t fd)
{
    int idx = fd - EPOLL_EVENTFD_FD_BASE;
    if (idx < 0 || idx >= EVENTFD_MAX_INSTANCES) return -9;
    spinlock_lock(&g_eventfd_lock);
    g_eventfd_instances[idx].used = 0;
    spinlock_unlock(&g_eventfd_lock);
    return 0;
}
