#pragma once
#include <stdint.h>

/* Global numbering of AF_UNIX endpoints: UNIX_SOCK_FD_BASE + slot.
 *
 * These used to sit at 192..255, squeezed under the X server's 256-client
 * limit and inside the file table's range, because Linux programs saw these
 * numbers directly. They no longer do: a Linux-ABI process has a descriptor
 * table of its own (Compat/Linux/Linux_FdTable.c) and never sees a global
 * number, so the range only has to stay below 1024 for native programs (the
 * POSIX layer and FD_SETSIZE index by raw fd). Layout: file table 0..511,
 * inet sockets 512..767, AF_UNIX 768..1023.
 *
 * 256 endpoints: multi-process Chromium alone holds a socketpair per child,
 * its zygotes' control and sandbox channels, a crash-handler channel per
 * process and one X connection per process that draws. */
#define UNIX_SOCK_FD_BASE 768
#define UNIX_SOCK_MAX 256
/* Receive ring per endpoint, allocated from the kernel heap when the
 * endpoint is created (256 of them inline would be 8 MiB of .bss). */
#define UNIX_SOCK_BUF_SIZE (32u * 1024u)
#define UNIX_SOCK_PATH_MAX 108
#define SCM_RIGHTS 1

/* O_NONBLOCK for an AF_UNIX endpoint. Sockets start blocking, so a recv with
 * an empty ring has to park the caller instead of reporting EAGAIN. */
int unix_socket_set_nonblock(int32_t fd, int on);
int unix_socket_is_nonblock(int32_t fd);
/* SO_PASSCRED / SO_PEERCRED for an AF_UNIX endpoint. See UnixSocket.c. */
int unix_socket_set_passcred(int32_t fd, int on);
int unix_socket_peer_pid(int32_t fd, int32_t *pid_out);
/* Bring-up trace hook, bounded by the same cap as the other [usock] lines. */
void unix_socket_trace_note(const char *tag, int32_t fd);
#define AF_UNIX 1
#define SOCK_STREAM_UNIX 1

void unix_socket_init(void);
int64_t unix_socket_create(int32_t type);
int64_t unix_socket_bind(int32_t fd, const char *path);
int64_t unix_socket_listen(int32_t fd, int32_t backlog);
int64_t unix_socket_accept(int32_t fd);
int64_t unix_socket_connect(int32_t fd, const char *path);
int64_t unix_socket_send(int32_t fd, const void *buf, uint64_t len);
int64_t unix_socket_recv(int32_t fd, void *buf, uint64_t len);
int64_t unix_socket_sendmsg(int32_t fd, uint64_t msg_ptr);
int64_t unix_socket_recvmsg(int32_t fd, uint64_t msg_ptr);
int64_t unix_socket_close(int32_t fd);
void unix_socket_close_all_for_pid(int32_t pid);
/* Hand every endpoint the parent holds to a newly forked child, so the two
 * hold independent references to the same socket. */
void unix_socket_fork_inherit(int32_t parent_pid, int32_t child_pid);
/* socketpair(AF_UNIX, ...) - creates two already-connected endpoints
 * without bind()/listen()/connect()/accept(). Writes the two new fds to
 * out_fds[0]/out_fds[1] and returns 0, or a negative error. */
int64_t unix_socket_pair(int32_t out_fds[2]);
/* socketpair() with a socket type: SOCK_SEQPACKET/SOCK_DGRAM keep message
 * boundaries, SOCK_STREAM does not. */
int64_t unix_socket_pair_typed(int32_t type, int32_t out_fds[2]);

/* How SCM_RIGHTS maps descriptors for the calling process. A Linux-ABI
 * process names objects by its own numbers (Compat/Linux/Linux_FdTable.c):
 * `resolve` turns one of those into a kernel-global descriptor on send, and
 * `install` gives a received global one a number in the receiver's table. -1
 * from resolve means "not open". Unset hooks mean identity. */
void unix_socket_set_fd_hooks(int32_t (*resolve)(int32_t fd),
                              int32_t (*install)(int32_t global));
/* True if `fd` falls in the Unix-domain-socket fd range (regardless of
 * whether it is currently in use) - lets callers route by fd alone. */
int unix_socket_fd_in_range(int32_t fd);

/* Ascending walk of a process's AF_UNIX fds; -1 to start, -1 when done.
 * Second half of what /proc/<pid>/fd lists (the first is the file table). */
int32_t unix_socket_next_open_fd(int32_t pid, int32_t after);



/* poll(2)/epoll readiness for an AF_UNIX fd. `events`/result use EPOLL*
 * (== POLL*) bits. */
uint32_t unix_socket_poll(int32_t fd, uint32_t events);

/* Count of appends to this endpoint's receive queues. Only equality across
 * two reads is meaningful: it tells epoll's edge-triggered mode that data
 * arrived, which sampling readiness alone cannot. See the field comment in
 * UnixSocket.c. */
uint32_t unix_socket_rx_seq(int32_t fd);
