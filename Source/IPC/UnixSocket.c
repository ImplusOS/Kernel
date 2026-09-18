#include "UnixSocket.h"
#include "Core/sync/Spinlock.h"
#include "Core/process/ProcessManager.h"
#include "Core/memory/SharedMemory.h"
#include "Core/syscall/Syscall_File.h"
#include "Debug/serial/Serial.h"
#include "Core/timer/Timer.h"
#include "Core/syscall/Poll_Wait.h"
#include "kernel/config.h"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Descriptors in flight per endpoint. Mojo attaches several per message and
 * a burst of messages can be queued before the reader runs. */
#define UNIX_SOCK_FD_MAX 64
/* Message records per endpoint for the boundary-preserving types. */
#define UNIX_SOCK_MSG_MAX 128
#define UNIX_CMSG_MAX_BYTES 4096u
#define UNIX_IOV_MAX 16u
#define FILE_O_RDWR_FLAG 0x0002u
#define SCM_CREDENTIALS 2
/* struct cmsghdr is 16 bytes on x86-64 and cmsg data is 8-byte aligned. */
#define UNIX_CMSG_ALIGN(n) (((n) + 7u) & ~7u)

struct _kernel_cmsghdr {
    uint32_t cmsg_len;
    uint32_t __pad1;
    int32_t cmsg_level;
    int32_t cmsg_type;
};

/* How many descriptors each process holds on this endpoint.
 *
 * A single owner_pid was wrong the moment anything forked. On Linux a
 * socketpair half handed to a child survives the parent's close(), because
 * each process has its own descriptor table and the endpoint dies with the
 * last reference. Here there is one global table, so the parent's close()
 * destroyed the endpoint outright and the child was left holding a socket
 * whose peer had vanished -- which is how Crashpad's client got EPIPE from
 * the very first sendmsg() to the handler it had just spawned, and reported
 * "Check failed: client.StartHandler(...)".
 *
 * A count rather than a bit, because one process can hold the same endpoint
 * under two descriptor numbers: dup2() of a socket onto a low fd (see the
 * alias table below) is a second reference, and closing either one must
 * leave the other working. */

/* One descriptor in flight over SCM_RIGHTS.
 *   SHM  -- a memfd's shared-memory object; `h` is its handle and the queue
 *           holds one reference to it.
 *   UNIX -- another AF_UNIX endpoint; `h` is its slot, kept alive by that
 *           endpoint's `inflight` count.
 *   FILE -- an entry of the file table (file, pipe, memfd not yet backed by
 *           shared memory); `h` is its global number, already granted to the
 *           processes that can receive it. */
enum { USOCK_PASS_SHM = 1, USOCK_PASS_UNIX = 2, USOCK_PASS_FILE = 3 };
struct usock_passed {
    uint8_t kind;
    int32_t h;
    /* SHM: the sender's descriptor status flags. Access mode travels with
     * the descriptor on Linux, and Chromium checks it: a read-only region
     * that arrives O_RDWR fails PlatformSharedMemoryRegion::Take() (the
     * pseudonymization salt, first thing every child process maps). */
    uint32_t status_flags;
};

typedef struct {
    uint8_t used;
    uint8_t listening;
    uint8_t connected;
    /* O_NONBLOCK, as set by fcntl(F_SETFL) or SOCK_NONBLOCK. Default 0, i.e.
     * blocking -- the caller of a blocking recv() must be made to wait rather
     * than handed EAGAIN, which is not a value a blocking recv may return. */
    uint8_t nonblock;
    /* SO_PASSCRED: recvmsg() on this endpoint attaches an SCM_CREDENTIALS
     * control message naming who sent the bytes. Crashpad's client and
     * handler talk over a socketpair with this set on both ends and treat a
     * message that arrives without credentials as a protocol violation
     * ("missing credentials"), so it is not decoration. */
    uint8_t passcred;
    /* Who last put bytes or descriptors into this endpoint's queues, which is
     * what SCM_CREDENTIALS reports. A stream carries no message boundaries
     * here, so the credentials are the last sender's rather than each byte's
     * -- the senders on a socketpair do not change identity mid-connection. */
    int32_t last_sender_pid;
    /* The process that created the endpoint. Kept for tracing and for
     * SO_PEERCRED -- who may use and close it is `refs`. */
    int32_t owner_pid;
    uint8_t refs[OS_CONFIG_PROCESS_MAX_COUNT];
    /* This endpoint queued inside a message on another socket (SCM_RIGHTS)
     * and not yet received. Keeps it alive after the sender closes its own
     * descriptor, which Mojo does as soon as sendmsg() returns. */
    uint16_t inflight;
    /* SOCK_SEQPACKET / SOCK_DGRAM: each send is one message, each receive
     * returns at most one, and descriptors travel with the message they were
     * sent with. Chromium's zygote protocol is SOCK_SEQPACKET and parses one
     * request per read; a byte stream would hand it two glued together. */
    uint8_t seqpacket;
    int32_t peer_fd;
    char path[UNIX_SOCK_PATH_MAX];
    uint8_t *buf;          /* UNIX_SOCK_BUF_SIZE bytes, heap */
    uint32_t buf_head, buf_tail;
    struct { uint32_t len; uint16_t nfds; } msgs[UNIX_SOCK_MSG_MAX];
    uint32_t msg_head, msg_tail;
    /* Bumped every time something is appended to this endpoint's queues.
     *
     * epoll's edge-triggered mode needs to know that data *arrived*, and a
     * poller that only samples readiness cannot see that: once the fd is
     * readable it stays readable, so a second arrival looks identical to the
     * first and no new edge is ever reported. A reader that drains less than
     * the whole ring then waits forever on bytes the kernel is holding -- see
     * Syscall_Epoll.c's epoll_check_once() and TODO_Chromium_LinuxABI.md
     * section 10.-5. This counter is the arrival event epoll compares against. */
    uint32_t rx_seq;
    /* Pending SCM_RIGHTS transfers, oldest first. See usock_passed_t. */
    struct usock_passed fd_queue[UNIX_SOCK_FD_MAX];
    uint32_t fds_head, fds_tail;
    spinlock_t lock;
} unix_sock_t;

#ifndef PROCESS_STALL_DUMP
#define PROCESS_STALL_DUMP 0
#endif
#if PROCESS_STALL_DUMP
static void usock_wire_note_dir(int32_t from, int32_t to, uint8_t dir,
                                const uint8_t *buf, uint64_t len);
#endif

static unix_sock_t g_usocks[UNIX_SOCK_MAX];
static int g_usock_init_done = 0;

void unix_socket_init(void) {
    memset(g_usocks, 0, sizeof(g_usocks));
    for (int i = 0; i < UNIX_SOCK_MAX; i++) spinlock_init(&g_usocks[i].lock);
    g_usock_init_done = 1;
}

static void usock_owner_set(unix_sock_t *s, int32_t pid) {
    if (pid < 0 || pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) return;
    if (s->refs[pid] < 255u) s->refs[pid] = (uint8_t)(s->refs[pid] + 1u);
}

static void usock_owner_clear(unix_sock_t *s, int32_t pid) {
    if (pid < 0 || pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) return;
    if (s->refs[pid] > 0u) s->refs[pid] = (uint8_t)(s->refs[pid] - 1u);
}

static int usock_owner_test(const unix_sock_t *s, int32_t pid) {
    if (pid < 0 || pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) return 0;
    return s->refs[pid] != 0u;
}

/* Alive while any process holds it or a message on another socket carries
 * it. */
static int usock_alive(const unix_sock_t *s) {
    if (s->inflight != 0u) return 1;
    for (uint32_t i = 0; i < OS_CONFIG_PROCESS_MAX_COUNT; ++i) {
        if (s->refs[i] != 0u) return 1;
    }
    return 0;
}

static unix_sock_t *usock_get(int32_t fd) {
    int idx = fd - UNIX_SOCK_FD_BASE;
    if (idx < 0 || idx >= UNIX_SOCK_MAX) return NULL;
    return g_usocks[idx].used ? &g_usocks[idx] : NULL;
}

/* SCM_RIGHTS descriptor mapping for the calling process; see UnixSocket.h. */
static int32_t (*g_usock_resolve)(int32_t fd);
static int32_t (*g_usock_install)(int32_t global);

void unix_socket_set_fd_hooks(int32_t (*resolve)(int32_t fd),
                              int32_t (*install)(int32_t global)) {
    g_usock_resolve = resolve;
    g_usock_install = install;
}

static void usock_destroy(unix_sock_t *s);

/* Let go of one descriptor that will never be received. */
static void usock_release_passed(struct usock_passed p) {
    if (p.kind == USOCK_PASS_SHM) {
        if (p.h >= 0) (void)shared_memory_release(p.h);
    } else if (p.kind == USOCK_PASS_UNIX) {
        if (p.h >= 0 && p.h < UNIX_SOCK_MAX) {
            unix_sock_t *o = &g_usocks[p.h];
            if (o->used && o->inflight > 0u) {
                --o->inflight;
                if (!usock_alive(o)) usock_destroy(o);
            }
        }
    }
    /* USOCK_PASS_FILE: the grant made at send time stays with the receiving
     * processes until they exit, like any descriptor they never closed. */
}

/* Tear the endpoint down for good: the last holder has let go. Descriptors
 * still queued on it are released outside its lock, since releasing one can
 * destroy another endpoint in turn. */
static void usock_destroy(unix_sock_t *s) {
    struct usock_passed pending[UNIX_SOCK_FD_MAX];
    uint32_t n = 0;
    spinlock_lock(&s->lock);
    if (!s->used) {
        spinlock_unlock(&s->lock);
        return;
    }
    while (s->fds_tail != s->fds_head && n < UNIX_SOCK_FD_MAX) {
        pending[n++] = s->fd_queue[s->fds_tail];
        s->fds_tail = (s->fds_tail + 1) % UNIX_SOCK_FD_MAX;
    }
    s->connected = 0;
    s->listening = 0;
    s->peer_fd = -1;
    s->used = 0;
    uint8_t *buf = s->buf;
    s->buf = NULL;
    spinlock_unlock(&s->lock);
    free(buf);
    for (uint32_t i = 0; i < n; ++i) {
        usock_release_passed(pending[i]);
    }
    /* The peer may be parked waiting for bytes that will now never come;
     * it has to wake up and see the hangup. */
    poll_wait_notify();
}

/* "Is anyone listening on `path`?" -- a read-only probe with no side effects
 * on either end, unlike connecting and hanging up, which the server sees as a
 * client that opened and vanished. */
int64_t unix_socket_path_listening(const char *path) {
    if (!g_usock_init_done) unix_socket_init();
    if (path == NULL) return 0;
    size_t clen = 0; while (path[clen]) clen++;
    for (int i = 0; i < UNIX_SOCK_MAX; i++) {
        if (!g_usocks[i].used || !g_usocks[i].listening) continue;
        size_t plen = 0; while (g_usocks[i].path[plen]) plen++;
        if (plen == clen && memcmp(g_usocks[i].path, path, plen) == 0) return 1;
    }
    return 0;
}

int64_t unix_socket_create(int32_t type) {
    if (!g_usock_init_done) unix_socket_init();
    uint8_t *buf = (uint8_t *)malloc(UNIX_SOCK_BUF_SIZE);
    if (buf == NULL) return -12;
    for (int i = 0; i < UNIX_SOCK_MAX; i++) {
        if (!g_usocks[i].used) {
            memset(&g_usocks[i], 0, sizeof(unix_sock_t));
            spinlock_init(&g_usocks[i].lock);
            g_usocks[i].used = 1;
            g_usocks[i].buf = buf;
            g_usocks[i].peer_fd = -1;
            g_usocks[i].last_sender_pid = -1;
            /* SOCK_DGRAM (2) and SOCK_SEQPACKET (5) keep message boundaries. */
            g_usocks[i].seqpacket = ((type & 0xF) == 2 || (type & 0xF) == 5) ? 1u : 0u;
            g_usocks[i].owner_pid = process_get_current_pid();
            usock_owner_set(&g_usocks[i], g_usocks[i].owner_pid);
            return UNIX_SOCK_FD_BASE + i;
        }
    }
    free(buf);
    return -24;
}

/* Ascending walk of a process's AF_UNIX endpoints, in the same "-1 to start,
 * -1 when done" form as syscall_file_next_open_fd(). AF_UNIX fds live in
 * their own numeric range (UNIX_SOCK_FD_BASE..), so /proc/<pid>/fd has to ask
 * both tables to produce the complete list a Linux program expects there. */
int32_t unix_socket_next_open_fd(int32_t pid, int32_t after) {
    if (!g_usock_init_done) unix_socket_init();
    int32_t start = (after < UNIX_SOCK_FD_BASE) ? 0 : (after - UNIX_SOCK_FD_BASE) + 1;
    if (start < 0) start = 0;
    for (int32_t i = start; i < UNIX_SOCK_MAX; i++) {
        if (g_usocks[i].used && usock_owner_test(&g_usocks[i], pid)) {
            return UNIX_SOCK_FD_BASE + i;
        }
    }
    return -1;
}

/* SO_PASSCRED. Set on both halves of a socketpair by anything that wants to
 * know who it is talking to; see the field comment. */
int unix_socket_set_passcred(int32_t fd, int on) {
    unix_sock_t *s = usock_get(fd);
    if (!s) return -1;
    s->passcred = on ? 1u : 0u;
    return 0;
}

/* SO_PEERCRED: the pid on the other end of a connected endpoint. Reporting a
 * real pid rather than 0 is what lets a caller identify the process it just
 * handed a socket to. */
int unix_socket_peer_pid(int32_t fd, int32_t *pid_out) {
    unix_sock_t *s = usock_get(fd);
    if (!s || !pid_out) return -1;
    unix_sock_t *peer = s->connected ? usock_get(s->peer_fd) : NULL;
    *pid_out = peer ? peer->owner_pid : 0;
    return 0;
}

int64_t unix_socket_bind(int32_t fd, const char *path) {
    unix_sock_t *s = usock_get(fd);
    if (!s || !path) return -14;
    size_t len = 0;
    while (path[len] && len < UNIX_SOCK_PATH_MAX - 1) len++;
    memcpy(s->path, path, len);
    s->path[len] = 0;
    return 0;
}

int64_t unix_socket_listen(int32_t fd, int32_t backlog) {
    (void)backlog;
    unix_sock_t *s = usock_get(fd);
    if (!s) return -9;
    s->listening = 1;
    /* Bring-up trace: one line per listening socket, so a client that cannot
     * find the server has something concrete to be compared against. */
    if (OS_CONFIG_FOREIGN_TRACE) {
        serial_write_string("[usock] listen '");
        serial_write_string(s->path);
        serial_write_string("'\n");
    }
    return 0;
}

/* Bring-up trace for the X11 handshake over AF_UNIX (TODO_Doom_Xorg_MethodA.md
 * M22). A client that connects, is accepted, and then goes away looks exactly
 * like an authorization refusal and exactly like a lost reply, so log the
 * connect/accept pairing and the first few payload sizes. Hard-capped so it
 * cannot flood the console. */
#define USOCK_TRACE_MAX 512
/* Distinct connect failures reported before going quiet; see below. */
#define USOCK_CONNECT_FAIL_MAX 8u
static uint32_t g_usock_trace_count;

/* "nothing buffered yet" is the overwhelming majority of the traffic on a
 * non-blocking socket -- a poll loop produces thousands of them a second --
 * and tracing it used up the whole budget before the first real byte moved.
 * Skip it: an EAGAIN is the absence of an event, and what a stalled
 * connection needs is the presence or absence of the tx/rx around it. */
/* "nothing buffered yet" is the overwhelming majority of the traffic on a
 * non-blocking socket -- a poll loop produces thousands of them a second --
 * and tracing it used up the whole budget before the first real byte moved.
 * Skip it: an EAGAIN is the absence of an event, and what a stalled
 * connection needs is the presence or absence of the tx/rx around it. */
static void usock_trace2(const char *tag, int32_t a, int64_t b)
{
    /* Off with the rest of the foreign trace. This is one COM1 line per
     * AF_UNIX packet and AF_UNIX *is* the X11 transport, so leaving it on put
     * hundreds of lines -- seconds of 115200 baud -- into the middle of every
     * client handshake. Worse, once a terminal streams the kernel log back
     * into a window, each line drawn produces more X traffic and so more
     * lines: a feedback loop with the console as the amplifier. */
    if (!OS_CONFIG_FOREIGN_TRACE) {
        return;
    }
    if (tag[0] == 'r' && tag[1] == 'x' && tag[2] == '-' && tag[3] == 'E' &&
        tag[4] == 'A') {
        return;
    }
    if (g_usock_trace_count >= USOCK_TRACE_MAX) return;
    ++g_usock_trace_count;
    serial_write_string("[usock] ");
    serial_write_string(tag);
    serial_write_string(" fd=");
    serial_write_uint64((uint64_t)(uint32_t)a);
    serial_write_string(" n=");
    serial_write_uint64((uint64_t)b);
    serial_write_char('\n');
}

int64_t unix_socket_accept(int32_t fd) {
    unix_sock_t *s = usock_get(fd);
    if (!s || !s->listening) return -22;
    for (int i = 0; i < UNIX_SOCK_MAX; i++) {
        if (g_usocks[i].used && g_usocks[i].connected && g_usocks[i].peer_fd == fd) {
            int64_t new_fd = unix_socket_create(s->seqpacket ? 5 : 1);
            if (new_fd < 0) return new_fd;
            unix_sock_t *ns = usock_get((int32_t)new_fd);
            if (ns) {
                ns->connected = 1;
                ns->peer_fd = UNIX_SOCK_FD_BASE + i;
                /* The client's first bytes (Wayland get_registry/sync) were
                 * written into the listener's ring before we got here, since
                 * its peer_fd still pointed at the listener. Move them, and
                 * any pending SCM_RIGHTS handles and message records, onto
                 * the accepted socket. */
                spinlock_lock(&s->lock);
                while (s->buf_tail != s->buf_head) {
                    uint32_t nx = (ns->buf_head + 1) % UNIX_SOCK_BUF_SIZE;
                    if (nx == ns->buf_tail) break;
                    ns->buf[ns->buf_head] = s->buf[s->buf_tail];
                    ns->buf_head = nx;
                    s->buf_tail = (s->buf_tail + 1) % UNIX_SOCK_BUF_SIZE;
                }
                while (s->fds_tail != s->fds_head) {
                    uint32_t nx = (ns->fds_head + 1) % UNIX_SOCK_FD_MAX;
                    if (nx == ns->fds_tail) break;
                    ns->fd_queue[ns->fds_head] = s->fd_queue[s->fds_tail];
                    ns->fds_head = nx;
                    s->fds_tail = (s->fds_tail + 1) % UNIX_SOCK_FD_MAX;
                }
                while (s->msg_tail != s->msg_head) {
                    uint32_t nx = (ns->msg_head + 1) % UNIX_SOCK_MSG_MAX;
                    if (nx == ns->msg_tail) break;
                    ns->msgs[ns->msg_head] = s->msgs[s->msg_tail];
                    ns->msg_head = nx;
                    s->msg_tail = (s->msg_tail + 1) % UNIX_SOCK_MSG_MAX;
                }
                ns->last_sender_pid = s->last_sender_pid;
                ++ns->rx_seq;
                spinlock_unlock(&s->lock);
                g_usocks[i].peer_fd = (int32_t)new_fd;
            }
            usock_trace2("accept", fd, new_fd);
            return new_fd;
        }
    }
    return -11;
}

int64_t unix_socket_connect(int32_t fd, const char *path) {
    unix_sock_t *s = usock_get(fd);
    if (!s || !path) return -14;
    /* An empty name matches nothing. A socket that was listen()ed without a
     * successful bind() keeps path[0] == '\0', and letting "" compare equal
     * made every such socket a catch-all that swallowed unrelated connects. */
    if (path[0] == '\0') return -111;
    for (int i = 0; i < UNIX_SOCK_MAX; i++) {
        if (g_usocks[i].used && g_usocks[i].listening) {
            size_t plen = 0; while (g_usocks[i].path[plen]) plen++;
            size_t clen = 0; while (path[clen]) clen++;
            if (plen == clen && memcmp(g_usocks[i].path, path, plen) == 0) {
                s->connected = 1;
                s->peer_fd = UNIX_SOCK_FD_BASE + i;
                usock_trace2("conn-ok", fd, UNIX_SOCK_FD_BASE + i);
                /* The listening socket is now readable: wake whoever is
                 * parked in accept()'s poll. */
                poll_wait_notify();
                return 0;
            }
        }
    }
    /* Only on failure, and capped. ECONNREFUSED here is what an X client sees
     * as "Couldn't connect to display!", with no further detail, so the first
     * few are worth having even in a quiet boot. The cap matters because a
     * failing connect is often on a retry timer that never gives up: Xorg's
     * dbus-core re-tries /run/dbus/system_bus_socket every 10 seconds for the
     * life of the server, and on a machine with no dbus that is two COM1
     * lines every 10 seconds forever. */
    {
        static uint32_t failed_reported;
        if (OS_CONFIG_FOREIGN_TRACE || failed_reported < USOCK_CONNECT_FAIL_MAX) {
            ++failed_reported;
            serial_write_string("[usock] connect FAILED '");
            serial_write_string(path);
            serial_write_string("'\n");
            if (!OS_CONFIG_FOREIGN_TRACE &&
                failed_reported == USOCK_CONNECT_FAIL_MAX) {
                serial_write_string("[usock] (further connect failures "
                                    "silenced)\n");
            }
        }
    }
    return -111;
}

/* Bytes queued in a ring. */
static uint32_t usock_ring_used(const unix_sock_t *s) {
    return (s->buf_head + UNIX_SOCK_BUF_SIZE - s->buf_tail) % UNIX_SOCK_BUF_SIZE;
}

/*
 * Append one send to the peer's queues: `len` bytes of `data` plus `nobjs`
 * descriptors. A boundary-preserving socket takes the whole message or
 * nothing; a stream takes what fits. Descriptors that cannot be queued are
 * released rather than leaked. Returns bytes accepted or a negative errno.
 */
static int64_t usock_enqueue(unix_sock_t *s, const uint8_t *data, uint64_t len,
                             struct usock_passed *objs, uint32_t nobjs) {
    unix_sock_t *peer = (s && s->connected) ? usock_get(s->peer_fd) : NULL;
    if (!peer) {
        for (uint32_t i = 0; i < nobjs; ++i) usock_release_passed(objs[i]);
        return -32; /* EPIPE */
    }
    spinlock_lock(&peer->lock);
    uint32_t space = (UNIX_SOCK_BUF_SIZE - 1u) - usock_ring_used(peer);
    uint32_t fd_space = (UNIX_SOCK_FD_MAX - 1u) -
                        (peer->fds_head + UNIX_SOCK_FD_MAX - peer->fds_tail) %
                            UNIX_SOCK_FD_MAX;
    if (peer->seqpacket) {
        uint32_t next_msg = (peer->msg_head + 1u) % UNIX_SOCK_MSG_MAX;
        if (len > UNIX_SOCK_BUF_SIZE - 1u) {
            spinlock_unlock(&peer->lock);
            for (uint32_t i = 0; i < nobjs; ++i) usock_release_passed(objs[i]);
            return -90; /* EMSGSIZE */
        }
        if (len > space || nobjs > fd_space || next_msg == peer->msg_tail) {
            spinlock_unlock(&peer->lock);
            return -11; /* EAGAIN: caller keeps its descriptors and retries */
        }
    }
    for (uint32_t i = 0; i < nobjs; ++i) {
        if (i < fd_space) {
            peer->fd_queue[peer->fds_head] = objs[i];
            peer->fds_head = (peer->fds_head + 1u) % UNIX_SOCK_FD_MAX;
        } else {
            usock_release_passed(objs[i]); /* stream with a full fd queue */
        }
    }
    uint64_t written = 0;
    while (written < len && written < space) {
        peer->buf[peer->buf_head] = data[written++];
        peer->buf_head = (peer->buf_head + 1u) % UNIX_SOCK_BUF_SIZE;
    }
    if (peer->seqpacket) {
        peer->msgs[peer->msg_head].len = (uint32_t)len;
        peer->msgs[peer->msg_head].nfds = (uint16_t)nobjs;
        peer->msg_head = (peer->msg_head + 1u) % UNIX_SOCK_MSG_MAX;
    }
    if (written != 0u || nobjs != 0u || peer->seqpacket) {
        ++peer->rx_seq;
        peer->last_sender_pid = process_get_current_pid();
    }
    spinlock_unlock(&peer->lock);
    if (written == 0 && len > 0 && !peer->seqpacket) {
        return -11; /* EAGAIN: ring full */
    }
#if PROCESS_STALL_DUMP
    usock_wire_note_dir((int32_t)(s - g_usocks) + UNIX_SOCK_FD_BASE, s->peer_fd,
                        0u, data, written);
#endif
    /* The peer is now readable. Cut short any poll()/select()/epoll_wait()
     * that is parked waiting for exactly this -- an X11 round trip crosses
     * that wait twice, so leaving it to time out cost ~16 ms per request. */
    poll_wait_notify();
    return (int64_t)written;
}

int64_t unix_socket_send(int32_t fd, const void *buf, uint64_t len) {
    unix_sock_t *s = usock_get(fd);
    if (!s || !s->connected || !buf) {
        usock_trace2("tx-BADF", fd, (int64_t)len);
        return -14;
    }
    int64_t rc = usock_enqueue(s, (const uint8_t *)buf, len, NULL, 0);
    usock_trace2(rc >= 0 ? "tx" : "tx-ERR", fd, rc);
    return rc;
}

/* The next message's size and descriptor count, for a boundary-preserving
 * endpoint. Caller holds the lock. Returns 0 if there is none. */
static int usock_front_msg(const unix_sock_t *s, uint32_t *len, uint32_t *nfds) {
    if (s->msg_tail == s->msg_head) return 0;
    *len = s->msgs[s->msg_tail].len;
    *nfds = s->msgs[s->msg_tail].nfds;
    return 1;
}

/*
 * Take up to `len` bytes into `dst` and, into `objs`, the descriptors that
 * belong with them: for a stream everything queued, for a boundary-preserving
 * socket exactly the front message's (whose unread tail is discarded, as
 * Linux does). Returns bytes read, -11 when empty, 0 at end of stream.
 */
static int64_t usock_dequeue(unix_sock_t *s, uint8_t *dst, uint64_t len,
                             struct usock_passed *objs, uint32_t *nobjs,
                             uint32_t objs_cap) {
    *nobjs = 0;
    spinlock_lock(&s->lock);
    uint64_t rd = 0;
    uint32_t take_fds;
    int have = 0;
    if (s->seqpacket) {
        uint32_t mlen = 0, mfds = 0;
        have = usock_front_msg(s, &mlen, &mfds);
        if (have) {
            uint64_t want = mlen < len ? mlen : len;
            while (rd < want) {
                dst[rd++] = s->buf[s->buf_tail];
                s->buf_tail = (s->buf_tail + 1u) % UNIX_SOCK_BUF_SIZE;
            }
            for (uint32_t skip = (uint32_t)rd; skip < mlen; ++skip) {
                s->buf_tail = (s->buf_tail + 1u) % UNIX_SOCK_BUF_SIZE;
            }
            s->msg_tail = (s->msg_tail + 1u) % UNIX_SOCK_MSG_MAX;
        }
        take_fds = have ? mfds : 0u;
    } else {
        while (rd < len && s->buf_tail != s->buf_head) {
            dst[rd++] = s->buf[s->buf_tail];
            s->buf_tail = (s->buf_tail + 1u) % UNIX_SOCK_BUF_SIZE;
        }
        have = (rd != 0u) || (s->fds_tail != s->fds_head);
        take_fds = UNIX_SOCK_FD_MAX;
    }
    struct usock_passed dropped[UNIX_SOCK_FD_MAX];
    uint32_t ndropped = 0;
    for (uint32_t i = 0; i < take_fds && s->fds_tail != s->fds_head; ++i) {
        struct usock_passed p = s->fd_queue[s->fds_tail];
        s->fds_tail = (s->fds_tail + 1u) % UNIX_SOCK_FD_MAX;
        if (*nobjs < objs_cap) {
            objs[(*nobjs)++] = p;
        } else {
            dropped[ndropped++] = p; /* no room in the caller's control buffer */
        }
    }
    spinlock_unlock(&s->lock);
    for (uint32_t i = 0; i < ndropped; ++i) usock_release_passed(dropped[i]);

    if (!have) {
        /* Distinguish "nothing yet" (EAGAIN) from "peer hung up" (EOF/0).
         * libwayland treats a 0-length recv as the compositor closing the
         * connection, so an empty-but-open socket must not return 0. */
        unix_sock_t *peer = (s->connected) ? usock_get(s->peer_fd) : NULL;
        int is_eof = (s->connected && !peer);
        return is_eof ? 0 : -11;
    }
    /* Draining the ring makes the peer writable again. */
    poll_wait_notify();
    return (int64_t)rd;
}

int64_t unix_socket_recv(int32_t fd, void *buf, uint64_t len) {
    unix_sock_t *s = usock_get(fd);
    if (!s || !buf) {
        usock_trace2("rx-BADF", fd, (int64_t)len);
        return -14;
    }
    /* The destination is user memory written without going through
     * copy_to_user(), so any page still shared with a fork parent has to be
     * unshared first. */
    process_user_break_cow(buf, len);
    struct usock_passed objs[UNIX_SOCK_FD_MAX];
    uint32_t nobjs = 0;
    int64_t rc = usock_dequeue(s, (uint8_t *)buf, len, objs, &nobjs,
                               UNIX_SOCK_FD_MAX);
    /* A plain read() has nowhere to put descriptors: Linux closes them. */
    for (uint32_t i = 0; i < nobjs; ++i) usock_release_passed(objs[i]);
    usock_trace2(rc > 0 ? "rx" : (rc == 0 ? "rx-EOF" : "rx-EAGAIN"), fd, rc);
    return rc;
}

/* msghdr layout matches glibc's struct msghdr on x86-64 (msg_iovlen /
 * msg_controllen are 8-byte). The top-level struct is dereferenced
 * directly (both callers guarantee it is mapped); msg_iov / msg_control
 * point at user memory and are bounds-limited before use. */
struct _kernel_msghdr {
    uint64_t name;
    uint32_t namelen;
    uint32_t __pad0;
    uint64_t iov;
    uint64_t iovlen;
    uint64_t control;
    uint64_t controllen;
    int32_t flags;
    int32_t __pad1;
};

/* Every process currently holding `s`: the ones that may receive what is
 * queued on it. */
static void usock_grant_to_holders(unix_sock_t *s, int32_t file_fd, int32_t shm) {
    for (int32_t pid = 0; pid < (int32_t)OS_CONFIG_PROCESS_MAX_COUNT; ++pid) {
        if (s->refs[pid] == 0u) continue;
        if (file_fd >= 0) (void)syscall_file_grant(file_fd, pid);
        if (shm >= 0) (void)shared_memory_grant(shm, pid);
    }
}

/* Turn one descriptor the sender named into something that can sit in a
 * queue. Returns 0 on success, -1 for a kind that cannot travel. */
static int usock_capture(int32_t global, unix_sock_t *peer,
                         struct usock_passed *out) {
    if (global >= UNIX_SOCK_FD_BASE && global < UNIX_SOCK_FD_BASE + UNIX_SOCK_MAX) {
        unix_sock_t *o = usock_get(global);
        if (!o) return -1;
        spinlock_lock(&o->lock);
        ++o->inflight;
        spinlock_unlock(&o->lock);
        out->kind = USOCK_PASS_UNIX;
        out->h = global - UNIX_SOCK_FD_BASE;
        out->status_flags = 0;
        return 0;
    }
    int32_t h = syscall_memfd_shm_handle(global);
    if (h >= 0) {
        usock_grant_to_holders(peer, -1, h);
        (void)shared_memory_addref(h);
        out->kind = USOCK_PASS_SHM;
        out->h = h;
        int32_t fl = syscall_file_get_status_flags(global);
        out->status_flags = (fl >= 0) ? (uint32_t)fl : FILE_O_RDWR_FLAG;
        return 0;
    }
    if (syscall_file_get_status_flags(global) >= 0 || syscall_file_is_dir(global)) {
        usock_grant_to_holders(peer, global, -1);
        out->kind = USOCK_PASS_FILE;
        out->h = global;
        out->status_flags = 0;
        return 0;
    }
    return -1;
}

/* One line per sendmsg/recvmsg: who, which endpoint, how many bytes and
 * descriptors. The zygote's fork handshake is a handful of these, and a
 * mismatch in either count is invisible from anywhere else. Off by default. */
#ifndef UNIX_SCM_TRACE
#define UNIX_SCM_TRACE 0
#endif
static void usock_scm_trace(const char *dir, int32_t fd, int64_t bytes,
                            uint32_t nobjs, int seqpacket) {
    if (!UNIX_SCM_TRACE) return;
    char line[128];
    int n = 0;
    const char *parts[] = { "[scm] ", dir, " pid=" };
    for (unsigned k = 0; k < 3; ++k) {
        for (const char *q = parts[k]; *q && n < 100; ++q) line[n++] = *q;
    }
    uint64_t vals[4] = { (uint64_t)(uint32_t)process_get_current_pid(),
                         (uint64_t)(uint32_t)fd, (uint64_t)bytes, nobjs };
    const char *names[4] = { "", " fd=", " bytes=", " fds=" };
    for (unsigned k = 0; k < 4; ++k) {
        for (const char *q = names[k]; *q && n < 110; ++q) line[n++] = *q;
        char dig[24]; int d = 0; uint64_t v = vals[k];
        if (k == 2 && bytes < 0) { line[n++] = '-'; v = (uint64_t)(-bytes); }
        do { dig[d++] = (char)('0' + v % 10u); v /= 10u; } while (v && d < 22);
        while (d > 0 && n < 120) line[n++] = dig[--d];
    }
    if (seqpacket && n < 124) { line[n++] = ' '; line[n++] = 'S'; }
    line[n++] = '\n';
    line[n] = '\0';
    serial_write_string(line);
}

int64_t unix_socket_sendmsg(int32_t fd, uint64_t msg_ptr) {
    struct _kernel_msghdr *msg = (void*)(uintptr_t)msg_ptr;
    if (!msg) return -14;
    unix_sock_t *s = usock_get(fd);
    if (!s || !s->connected) return -107; /* ENOTCONN */
    unix_sock_t *peer = usock_get(s->peer_fd);
    if (!peer) return -32;

    /* ---- SCM_RIGHTS: capture each named descriptor ---- */
    struct usock_passed objs[UNIX_SOCK_FD_MAX];
    uint32_t nobjs = 0;
    if (msg->control && msg->controllen >= sizeof(struct _kernel_cmsghdr) &&
        msg->controllen <= UNIX_CMSG_MAX_BYTES) {
        uint8_t *cdata = (uint8_t*)(uintptr_t)msg->control;
        uint32_t offset = 0;
        while (offset + sizeof(struct _kernel_cmsghdr) <= msg->controllen) {
            struct _kernel_cmsghdr *cmsg = (struct _kernel_cmsghdr *)(cdata + offset);
            uint32_t clen = cmsg->cmsg_len;
            if (clen < sizeof(struct _kernel_cmsghdr) ||
                offset + clen > msg->controllen) break;
            if (cmsg->cmsg_level == 1 && cmsg->cmsg_type == SCM_RIGHTS) {
                int32_t *fds = (int32_t *)(cdata + offset + sizeof(struct _kernel_cmsghdr));
                int num_fds = (int)((clen - sizeof(struct _kernel_cmsghdr)) / sizeof(int32_t));
                for (int i = 0; i < num_fds && nobjs < UNIX_SOCK_FD_MAX; i++) {
                    int32_t global = g_usock_resolve ? g_usock_resolve(fds[i]) : fds[i];
                    if (global < 0) {
                        for (uint32_t k = 0; k < nobjs; ++k) usock_release_passed(objs[k]);
                        return -9; /* EBADF, as Linux reports a bad fd here */
                    }
                    if (usock_capture(global, peer, &objs[nobjs]) == 0) {
                        if (UNIX_SCM_TRACE) {
                            char t[96];
                            snprintf(t, sizeof(t), "[scm]   capture #%u user=%d global=%d kind=%u h=%d\n",
                                     (unsigned)nobjs, (int)fds[i], (int)global,
                                     (unsigned)objs[nobjs].kind, (int)objs[nobjs].h);
                            serial_write_string(t);
                        }
                        ++nobjs;
                    } else {
                        serial_write_string("[unixsock] SCM_RIGHTS: descriptor kind cannot be passed, dropped\n");
                    }
                }
            }
            offset += (clen + 7u) & ~7u;
        }
    }

    /* ---- payload ---- */
    struct { uint64_t base; uint64_t len; } *iov = (void*)(uintptr_t)msg->iov;
    uint64_t iovn = (iov && msg->iovlen > 0) ?
                    (msg->iovlen > UNIX_IOV_MAX ? UNIX_IOV_MAX : msg->iovlen) : 0;

    if (peer->seqpacket) {
        /* One message: gather the iovecs first so it is queued whole. */
        uint64_t total = 0;
        for (uint32_t i = 0; i < iovn; i++) total += iov[i].len;
        if (total > UNIX_SOCK_BUF_SIZE) {
            for (uint32_t k = 0; k < nobjs; ++k) usock_release_passed(objs[k]);
            return -90;
        }
        uint8_t *gather = total ? (uint8_t *)malloc((size_t)total) : NULL;
        if (total && !gather) {
            for (uint32_t k = 0; k < nobjs; ++k) usock_release_passed(objs[k]);
            return -12;
        }
        uint64_t off = 0;
        for (uint32_t i = 0; i < iovn; i++) {
            if (iov[i].len == 0) continue;
            memcpy(gather + off, (const void *)(uintptr_t)iov[i].base, iov[i].len);
            off += iov[i].len;
        }
        int64_t rc = usock_enqueue(s, gather, total, objs, nobjs);
        usock_scm_trace("tx", fd, rc, nobjs, 1);
        free(gather);
        if (rc == -11) {
            /* Nothing was taken; the caller will retry with the same fds. */
            for (uint32_t k = 0; k < nobjs; ++k) usock_release_passed(objs[k]);
        }
        return rc;
    }

    /* Stream: descriptors ride with the first chunk. */
    int64_t total = 0;
    int first = 1;
    for (uint32_t i = 0; i < iovn; i++) {
        if (iov[i].len == 0) continue;
        int64_t r = usock_enqueue(s, (const uint8_t *)(uintptr_t)iov[i].base,
                                  iov[i].len, first ? objs : NULL,
                                  first ? nobjs : 0u);
        first = 0;
        if (r < 0) return total > 0 ? total : r;
        total += r;
        if ((uint64_t)r < iov[i].len) break; /* ring full: partial */
    }
    usock_scm_trace("tx", fd, total, nobjs, 0);
    if (first && nobjs != 0u) {
        /* Descriptors with no payload at all. */
        int64_t r = usock_enqueue(s, NULL, 0, objs, nobjs);
        if (r < 0) return r;
    }
    return total;
}

/* A received descriptor becomes an open descriptor of the receiver. Returns
 * the kernel-global number, or -1. */
static int32_t usock_adopt(struct usock_passed p) {
    int32_t self = process_get_current_pid();
    if (p.kind == USOCK_PASS_SHM) {
        int32_t g = syscall_memfd_install_shm(p.h, p.status_flags);
        if (g < 0) (void)shared_memory_release(p.h);
        return g;
    }
    if (p.kind == USOCK_PASS_UNIX) {
        if (p.h < 0 || p.h >= UNIX_SOCK_MAX) return -1;
        unix_sock_t *o = &g_usocks[p.h];
        spinlock_lock(&o->lock);
        int ok = o->used;
        if (ok) {
            usock_owner_set(o, self);
            if (o->inflight > 0u) --o->inflight;
        }
        spinlock_unlock(&o->lock);
        return ok ? UNIX_SOCK_FD_BASE + p.h : -1;
    }
    /* FILE: already granted at send time. */
    (void)syscall_file_grant(p.h, self);
    return p.h;
}

int64_t unix_socket_recvmsg(int32_t fd, uint64_t msg_ptr) {
    struct _kernel_msghdr *msg = (void*)(uintptr_t)msg_ptr;
    if (!msg) return -14;
    unix_sock_t *s = usock_get(fd);
    if (!s) return -9;

    struct { uint64_t base; uint64_t len; } *iov = (void*)(uintptr_t)msg->iov;
    uint64_t iovn = (iov && msg->iovlen > 0) ?
                    (msg->iovlen > UNIX_IOV_MAX ? UNIX_IOV_MAX : msg->iovlen) : 0;
    uint64_t want = 0;
    for (uint32_t i = 0; i < iovn; i++) want += iov[i].len;

    /* Read into a bounce buffer, then scatter: a message must be taken in one
     * dequeue so its boundary and its descriptors stay together. */
    uint64_t cap = want < UNIX_SOCK_BUF_SIZE ? want : UNIX_SOCK_BUF_SIZE;
    uint8_t *bounce = cap ? (uint8_t *)malloc((size_t)cap) : NULL;
    if (cap && !bounce) return -12;

    uint32_t ctl_room = 0;
    if (msg->control && msg->controllen >= sizeof(struct _kernel_cmsghdr) &&
        msg->controllen <= UNIX_CMSG_MAX_BYTES) {
        ctl_room = ((uint32_t)msg->controllen -
                    (uint32_t)sizeof(struct _kernel_cmsghdr)) / sizeof(int32_t);
    }
    struct usock_passed objs[UNIX_SOCK_FD_MAX];
    uint32_t nobjs = 0;
    int64_t rc = usock_dequeue(s, bounce, cap, objs, &nobjs,
                               ctl_room < UNIX_SOCK_FD_MAX ? ctl_room : UNIX_SOCK_FD_MAX);
    if (rc < 0) {
        free(bounce);
        return rc;
    }
    usock_scm_trace("rx", fd, rc, nobjs, s->seqpacket);
    if (UNIX_SCM_TRACE && s->seqpacket && rc > 0) {
        /* Printable bytes as themselves, the rest as \xx, whole message
         * up to 1 KiB: enough for the zygote's fork request. */
        char line[200];
        int n = 0;
        for (int64_t k = 0; k < rc && k < 1024; ++k) {
            uint8_t c = bounce[k];
            if (n > 180) {
                line[n++] = '\n'; line[n] = '\0';
                serial_write_string(line);
                n = 0;
            }
            if (n == 0) { line[n++] = '['; line[n++] = '='; line[n++] = ']'; line[n++] = ' '; }
            if (c >= 0x20 && c < 0x7f && c != '\\') {
                line[n++] = (char)c;
            } else {
                const char *digits = "0123456789abcdef";
                line[n++] = '\\'; line[n++] = digits[c >> 4]; line[n++] = digits[c & 0xF];
            }
        }
        line[n++] = '\n'; line[n] = '\0';
        serial_write_string(line);
    }
    uint64_t off = 0;
    for (uint32_t i = 0; i < iovn && off < (uint64_t)rc; i++) {
        uint64_t n = iov[i].len;
        if (n > (uint64_t)rc - off) n = (uint64_t)rc - off;
        if (n == 0) continue;
        process_user_break_cow((void *)(uintptr_t)iov[i].base, n);
        memcpy((void *)(uintptr_t)iov[i].base, bounce + off, n);
        off += n;
    }
    free(bounce);

    if (msg->control && msg->controllen >= sizeof(struct _kernel_cmsghdr) &&
        msg->controllen <= UNIX_CMSG_MAX_BYTES) {
        uint8_t *ctl = (uint8_t *)(uintptr_t)msg->control;
        uint32_t room = (uint32_t)msg->controllen;
        uint32_t used = 0;
        process_user_break_cow(ctl, room);

        /* SCM_CREDENTIALS first, when the receiver asked for it. Linux
         * synthesises this one itself rather than carrying it from the
         * sender's control buffer, and so do we: the identity reported is
         * whoever last wrote into this endpoint. */
        if (s->passcred) {
            uint32_t need = (uint32_t)sizeof(struct _kernel_cmsghdr) +
                            3u * (uint32_t)sizeof(uint32_t);
            if (room - used >= need) {
                struct _kernel_cmsghdr *cmsg =
                    (struct _kernel_cmsghdr *)(uintptr_t)(ctl + used);
                uint32_t *cred = (uint32_t *)(ctl + used +
                                              sizeof(struct _kernel_cmsghdr));
                int32_t sender = s->last_sender_pid;
                if (sender < 0) {
                    unix_sock_t *peer = s->connected ? usock_get(s->peer_fd) : NULL;
                    sender = peer ? peer->owner_pid : s->owner_pid;
                }
                cmsg->cmsg_len = need;
                cmsg->__pad1 = 0;
                cmsg->cmsg_level = 1; /* SOL_SOCKET */
                cmsg->cmsg_type = SCM_CREDENTIALS;
                uint32_t su = 0, sg = 0;
                (void)process_get_credentials(sender, &su, &sg);
                cred[0] = (uint32_t)process_pid_as_seen_by_current(sender);
                cred[1] = su;
                cred[2] = sg;
                used += UNIX_CMSG_ALIGN(need);
            }
        }

        uint32_t written = 0;
        if (nobjs != 0u && room > used + (uint32_t)sizeof(struct _kernel_cmsghdr)) {
            uint32_t max_fds = (room - used - (uint32_t)sizeof(struct _kernel_cmsghdr)) /
                               (uint32_t)sizeof(int32_t);
            int32_t *out = (int32_t *)(ctl + used + sizeof(struct _kernel_cmsghdr));
            for (uint32_t i = 0; i < nobjs; ++i) {
                if (written >= max_fds) {
                    usock_release_passed(objs[i]);
                    continue;
                }
                int32_t g = usock_adopt(objs[i]);
                if (g < 0) continue;
                int32_t u = g_usock_install ? g_usock_install(g) : g;
                if (UNIX_SCM_TRACE) {
                    char t[96];
                    snprintf(t, sizeof(t), "[scm]   adopt #%u kind=%u h=%d global=%d user=%d\n",
                             (unsigned)i, (unsigned)objs[i].kind, (int)objs[i].h,
                             (int)g, (int)u);
                    serial_write_string(t);
                }
                if (u < 0) continue;
                out[written++] = u;
            }
            if (written > 0) {
                struct _kernel_cmsghdr *cmsg =
                    (struct _kernel_cmsghdr *)(uintptr_t)(ctl + used);
                cmsg->cmsg_len = (uint32_t)sizeof(struct _kernel_cmsghdr) +
                                 written * (uint32_t)sizeof(int32_t);
                cmsg->__pad1 = 0;
                cmsg->cmsg_level = 1;
                cmsg->cmsg_type = SCM_RIGHTS;
                used += UNIX_CMSG_ALIGN(cmsg->cmsg_len);
            }
        } else {
            for (uint32_t i = 0; i < nobjs; ++i) usock_release_passed(objs[i]);
        }
        msg->controllen = used;
    } else {
        for (uint32_t i = 0; i < nobjs; ++i) usock_release_passed(objs[i]);
        if (msg->control) msg->controllen = 0;
    }
    return rc;
}

int64_t unix_socket_close(int32_t fd) {
    unix_sock_t *s = usock_get(fd);
    if (!s) return -9;
    /* Drop this process's reference; the endpoint only dies with the last
     * one. A caller that never held it closes nothing and is told so quietly
     * rather than with EBADF: glibc's closefrom() fallback walks every
     * descriptor number up to RLIMIT_NOFILE and closes it, and an error there
     * would be reported as a failure to spawn. */
    int32_t caller = process_get_current_pid();
    if (!usock_owner_test(s, caller)) {
        usock_trace2("close-NOTOWNER", fd, (int64_t)s->owner_pid);
        return 0;
    }
    usock_owner_clear(s, caller);
    if (!usock_alive(s)) {
        usock_destroy(s);
    }
    return 0;
}

/* POSIX: a forked child starts out holding every descriptor its parent held.
 * Called from process_fork() beside syscall_file_fork_inherit(). */
void unix_socket_fork_inherit(int32_t parent_pid, int32_t child_pid) {
    if (!g_usock_init_done) return;
    if (parent_pid < 0 || parent_pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT ||
        child_pid < 0 || child_pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) {
        return;
    }
    for (int i = 0; i < UNIX_SOCK_MAX; i++) {
        if (g_usocks[i].used) {
            g_usocks[i].refs[child_pid] = g_usocks[i].refs[parent_pid];
        }
    }
}

/* Release every AF_UNIX socket held by an exiting process. Without this the
 * global table leaks a slot per socket per exited process -- which is
 * exactly what made a later Xorg's socket() fail ("Unable to open socket").
 * Called from process exit alongside syscall_socket_close_all_for_pid(). */
void unix_socket_close_all_for_pid(int32_t pid) {
    if (!g_usock_init_done) return;
    if (pid < 0 || pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) return;
    for (int i = 0; i < UNIX_SOCK_MAX; i++) {
        if (g_usocks[i].used && g_usocks[i].refs[pid] != 0u) {
            g_usocks[i].refs[pid] = 0u;
            if (!usock_alive(&g_usocks[i])) {
                usock_destroy(&g_usocks[i]);
            }
        }
    }
}

int64_t unix_socket_pair_typed(int32_t type, int32_t out_fds[2]) {
    if (!out_fds) return -14;
    int64_t fd1 = unix_socket_create(type);
    if (fd1 < 0) return fd1;
    int64_t fd2 = unix_socket_create(type);
    if (fd2 < 0) {
        (void)unix_socket_close((int32_t)fd1);
        return fd2;
    }
    unix_sock_t *s1 = usock_get((int32_t)fd1);
    unix_sock_t *s2 = usock_get((int32_t)fd2);
    if (!s1 || !s2) {
        (void)unix_socket_close((int32_t)fd1);
        (void)unix_socket_close((int32_t)fd2);
        return -22;
    }
    s1->connected = 1;
    s1->peer_fd = (int32_t)fd2;
    s2->connected = 1;
    s2->peer_fd = (int32_t)fd1;
    out_fds[0] = (int32_t)fd1;
    out_fds[1] = (int32_t)fd2;
    return 0;
}

int64_t unix_socket_pair(int32_t out_fds[2]) {
    return unix_socket_pair_typed(1, out_fds);
}

int unix_socket_fd_in_range(int32_t fd) {
    return fd >= UNIX_SOCK_FD_BASE && fd < UNIX_SOCK_FD_BASE + UNIX_SOCK_MAX;
}

void unix_socket_trace_note(const char *tag, int32_t fd)
{
    usock_trace2(tag, fd, 0);
}

int unix_socket_set_nonblock(int32_t fd, int on)
{
    unix_sock_t *s = usock_get(fd);
    if (!s) return -14;
    s->nonblock = on ? 1u : 0u;
    return 0;
}

int unix_socket_is_nonblock(int32_t fd)
{
    unix_sock_t *s = usock_get(fd);
    return (s && s->nonblock) ? 1 : 0;
}

#ifndef PROCESS_STALL_DUMP
#define PROCESS_STALL_DUMP 0
#endif
#if PROCESS_STALL_DUMP
/* Every live endpoint with how much is queued on it, for the stall dump.
 * A client blocked in poll() on an X connection and a server blocked in
 * epoll_wait() on the same pair look identical from the syscall side; the
 * byte counts are what say whether a request is sitting undelivered. */
void unix_socket_debug_dump(void);
void unix_socket_debug_dump(void)
{
    for (int i = 0; i < UNIX_SOCK_MAX; ++i) {
        unix_sock_t *s = &g_usocks[i];
        if (!s->used) {
            continue;
        }
        spinlock_lock(&s->lock);
        uint32_t queued = (s->buf_head + UNIX_SOCK_BUF_SIZE - s->buf_tail) %
                          UNIX_SOCK_BUF_SIZE;
        uint32_t fds    = (s->fds_head + UNIX_SOCK_FD_MAX - s->fds_tail) %
                          UNIX_SOCK_FD_MAX;
        int32_t  peer   = s->peer_fd;
        int32_t  owner  = s->owner_pid;
        uint8_t  conn   = s->connected;
        uint8_t  lst    = s->listening;
        spinlock_unlock(&s->lock);

        serial_write_string("[usock] ");
        serial_write_uint32((uint32_t)(UNIX_SOCK_FD_BASE + i));
        serial_write_string(" own=");
        serial_write_uint32((uint32_t)owner);
        serial_write_string(" peer=");
        serial_write_uint32((uint32_t)peer);
        serial_write_string(conn ? " c" : (lst ? " l" : " -"));
        serial_write_string(" q=");
        serial_write_uint32(queued);
        serial_write_string(" fds=");
        serial_write_uint32(fds);
        serial_write_char('\n');
    }
}
#endif

#if PROCESS_STALL_DUMP
/* Ring of the most recent transfers, for the stall dump: which endpoint, how
 * many bytes, and the first four of them. An X client and the X server that
 * are both asleep look the same whether the client never sent its request or
 * the server never answered it; the wire trace is what tells them apart. */
#define USOCK_WIRE_MAX 48u
typedef struct {
    uint32_t ms;
    int32_t  from;
    int32_t  to;
    uint32_t len;
    uint8_t  dir;      /* 0 = send into the peer, 1 = recv out of this ring */
    uint8_t  head[4];
} usock_wire_t;
static usock_wire_t g_usock_wire[USOCK_WIRE_MAX];
static uint32_t     g_usock_wire_pos;
static spinlock_t   g_usock_wire_lock;

static void usock_wire_note_dir(int32_t from, int32_t to, uint8_t dir,
                                const uint8_t *buf, uint64_t len)
{
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_usock_wire_lock);
    usock_wire_t *w = &g_usock_wire[g_usock_wire_pos % USOCK_WIRE_MAX];
    ++g_usock_wire_pos;
    w->ms   = (uint32_t)(timer_monotonic_ns() / 1000000ull);
    w->from = from;
    w->to   = to;
    w->dir  = dir;
    w->len  = (uint32_t)len;
    for (uint32_t i = 0; i < 4u; ++i) {
        w->head[i] = (i < len) ? buf[i] : 0u;
    }
    spinlock_unlock(&g_usock_wire_lock);
    irq_restore(flags);
}

void unix_socket_wire_dump(void);
void unix_socket_wire_dump(void)
{
    uint32_t pos = g_usock_wire_pos;
    uint32_t n = (pos < USOCK_WIRE_MAX) ? pos : USOCK_WIRE_MAX;
    for (uint32_t k = 0; k < n; ++k) {
        const usock_wire_t *w = &g_usock_wire[(pos - n + k) % USOCK_WIRE_MAX];
        serial_write_string("[wire] t=");
        serial_write_uint32(w->ms);
        serial_write_string(w->dir ? " rx " : " tx ");
        serial_write_uint32((uint32_t)w->from);
        serial_write_string(">");
        serial_write_uint32((uint32_t)w->to);
        serial_write_string(" l=");
        serial_write_uint32(w->len);
        serial_write_string(" b=");
        for (uint32_t i = 0; i < 4u; ++i) {
            serial_write_uint32(w->head[i]);
            serial_write_string(",");
        }
        serial_write_char('\n');
    }
}
#endif

uint32_t unix_socket_rx_seq(int32_t fd)
{
    unix_sock_t *s = usock_get(fd);
    if (!s) {
        return 0u;
    }
    spinlock_lock(&s->lock);
    uint32_t seq = s->rx_seq;
    spinlock_unlock(&s->lock);
    return seq;
}

uint32_t unix_socket_poll(int32_t fd, uint32_t events) {
    unix_sock_t *s = usock_get(fd);
    if (!s) return 0x8u; /* EPOLLERR */
    uint32_t r = 0;
    if ((events & 0x1u) != 0u) {          /* POLLIN: bytes buffered, or a
                                           * pending SCM_RIGHTS transfer */
        spinlock_lock(&s->lock);
        int has = (s->buf_tail != s->buf_head) || (s->fds_tail != s->fds_head) ||
                  (s->seqpacket && s->msg_tail != s->msg_head);
        spinlock_unlock(&s->lock);
        if (has) r |= 0x1u;
    }
    if ((events & 0x4u) != 0u) {          /* POLLOUT: approximate as always
                                           * writable (ring is large) */
        unix_sock_t *peer = s->connected ? usock_get(s->peer_fd) : NULL;
        if (peer) r |= 0x4u;
        else if (!s->connected) r |= 0x4u; /* listening/unconnected: harmless */
    }
    if (s->connected) {
        unix_sock_t *peer = usock_get(s->peer_fd);
        if (!peer) r |= 0x10u;            /* EPOLLHUP: peer gone */
    }
    return r;
}
