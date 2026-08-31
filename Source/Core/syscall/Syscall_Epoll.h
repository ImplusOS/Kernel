#pragma once
#include <stdint.h>

/* Linux's x86_64 struct epoll_event is PACKED: 12 bytes, with `data` at
 * offset 4, not 16 bytes with `data` at offset 8. Getting this wrong makes
 * every epoll_wait() hand userland the `data` it registered shifted by four
 * bytes and the array stride wrong by four -- which is how the X server's
 * ospoll_wait() read a NULL `struct ospollfd *` out of events[i].data.ptr and
 * died dereferencing it (CR2=0x18) the moment it entered its main loop.
 * See TODO_Doom_Xorg_MethodA.md M21. */
#if defined(__x86_64__)
typedef struct __attribute__((packed)) {
    uint32_t events;
    uint64_t data;
} epoll_event_t;
#else
typedef struct {
    uint32_t events;
    uint64_t data;
} epoll_event_t;
#endif

int32_t syscall_epoll_create(uint64_t flags);
int32_t syscall_epoll_ctl(int32_t epfd, int32_t op, int32_t fd,
                          const epoll_event_t *event);
int32_t syscall_epoll_wait(int32_t epfd, epoll_event_t *events,
                           int32_t maxevents, int32_t timeout_ms);
/* Same as syscall_epoll_wait(), but also reports (via *should_switch_out)
 * whether the caller should request a scheduler switch - see the design
 * note at the top of Syscall_Epoll.c for why this can't just block
 * internally. */
int32_t syscall_epoll_wait_ex(int32_t epfd, epoll_event_t *events,
                              int32_t maxevents, int32_t timeout_ms,
                              int *should_switch_out);
/* One-shot readiness probe for a single fd (regular/pipe/timerfd/memfd/
 * signalfd/socket/eventfd), for the Linux compat poll(2)/ppoll(2).
 * `events` and the result use EPOLL* bits (== POLL* bits). Never blocks. */
uint32_t syscall_poll_one_fd(int32_t fd, uint32_t events);

int32_t syscall_eventfd(uint64_t initval, uint64_t flags);
int  syscall_eventfd_is_valid(int32_t fd);
int64_t syscall_eventfd_read(int32_t fd, uint8_t *buffer, uint64_t len);
int64_t syscall_eventfd_write(int32_t fd, const uint8_t *buffer, uint64_t len);
int32_t syscall_eventfd_close(int32_t fd);
