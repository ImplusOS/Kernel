#include "UnixSocket.h"
#include "Core/sync/Spinlock.h"
#include "Core/process/ProcessManager.h"
#include "Core/memory/SharedMemory.h"
#include "Core/syscall/Syscall_File.h"
#include "Debug/serial/Serial.h"
#include <stddef.h>
#include <string.h>

#define UNIX_SOCK_FD_MAX 16
#define UNIX_CMSG_MAX_BYTES 4096u
#define UNIX_IOV_MAX 16u
#define FILE_O_RDWR_FLAG 0x0002u

struct _kernel_cmsghdr {
    uint32_t cmsg_len;
    uint32_t __pad1;
    int32_t cmsg_level;
    int32_t cmsg_type;
};

typedef struct {
    uint8_t used;
    uint8_t listening;
    uint8_t connected;
    /* O_NONBLOCK, as set by fcntl(F_SETFL) or SOCK_NONBLOCK. Default 0, i.e.
     * blocking -- the caller of a blocking recv() must be made to wait rather
     * than handed EAGAIN, which is not a value a blocking recv may return. */
    uint8_t nonblock;
    int32_t owner_pid;
    int32_t peer_fd;
    char path[UNIX_SOCK_PATH_MAX];
    uint8_t  buf[UNIX_SOCK_BUF_SIZE];
    uint32_t buf_head, buf_tail;
    /* Pending SCM_RIGHTS transfers: each entry is a shared-memory handle
     * already granted to this endpoint's owner and holding one reference.
     * recvmsg() adopts it into a fresh memfd fd; close() releases any left. */
    int32_t fd_queue[UNIX_SOCK_FD_MAX];
    uint32_t fds_head, fds_tail;
    spinlock_t lock;
} unix_sock_t;

static unix_sock_t g_usocks[UNIX_SOCK_MAX];
static int g_usock_init_done = 0;

void unix_socket_init(void) {
    memset(g_usocks, 0, sizeof(g_usocks));
    for (int i = 0; i < UNIX_SOCK_MAX; i++) spinlock_init(&g_usocks[i].lock);
    g_usock_init_done = 1;
}

static unix_sock_t *usock_get(int32_t fd) {
    int idx = fd - UNIX_SOCK_FD_BASE;
    if (idx < 0 || idx >= UNIX_SOCK_MAX) return NULL;
    return g_usocks[idx].used ? &g_usocks[idx] : NULL;
}

static void usock_drain_fd_queue(unix_sock_t *s) {
    while (s->fds_tail != s->fds_head) {
        int32_t h = s->fd_queue[s->fds_tail];
        s->fds_tail = (s->fds_tail + 1) % UNIX_SOCK_FD_MAX;
        if (h >= 0) (void)shared_memory_release(h);
    }
}

int64_t unix_socket_create(int32_t type) {
    (void)type;
    if (!g_usock_init_done) unix_socket_init();
    for (int i = 0; i < UNIX_SOCK_MAX; i++) {
        if (!g_usocks[i].used) {
            memset(&g_usocks[i], 0, sizeof(unix_sock_t));
            spinlock_init(&g_usocks[i].lock);
            g_usocks[i].used = 1;
            g_usocks[i].peer_fd = -1;
            g_usocks[i].owner_pid = process_get_current_pid();
            return UNIX_SOCK_FD_BASE + i;
        }
    }
    return -24;
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
    serial_write_string("[usock] listen '");
    serial_write_string(s->path);
    serial_write_string("'\n");
    return 0;
}

/* Bring-up trace for the X11 handshake over AF_UNIX (TODO_Doom_Xorg_MethodA.md
 * M22). A client that connects, is accepted, and then goes away looks exactly
 * like an authorization refusal and exactly like a lost reply, so log the
 * connect/accept pairing and the first few payload sizes. Hard-capped so it
 * cannot flood the console. */
#define USOCK_TRACE_MAX 48
static uint32_t g_usock_trace_count;

static void usock_trace2(const char *tag, int32_t a, int64_t b)
{
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
            int64_t new_fd = unix_socket_create(0);
            if (new_fd < 0) return new_fd;
            unix_sock_t *ns = usock_get((int32_t)new_fd);
            if (ns) {
                ns->connected = 1;
                ns->peer_fd = UNIX_SOCK_FD_BASE + i;
                /* The client's first bytes (Wayland get_registry/sync) were
                 * written into the listener's ring before we got here, since
                 * its peer_fd still pointed at the listener. Move them, and
                 * any pending SCM_RIGHTS handles, onto the accepted socket. */
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
                return 0;
            }
        }
    }
    /* Bring-up trace: only on failure. ECONNREFUSED here is what an X client
     * sees as "Couldn't connect to display!", with no further detail. */
    serial_write_string("[usock] connect FAILED '");
    serial_write_string(path);
    serial_write_string("'\n");
    return -111;
}

int64_t unix_socket_send(int32_t fd, const void *buf, uint64_t len) {
    unix_sock_t *s = usock_get(fd);
    if (!s || !s->connected || !buf) {
        usock_trace2("tx-BADF", fd, (int64_t)len);
        return -14;
    }
    unix_sock_t *peer = usock_get(s->peer_fd);
    if (!peer) {
        usock_trace2("tx-NOPEER", fd, (int64_t)len);
        return -32;
    }
    spinlock_lock(&peer->lock);
    uint64_t written = 0;
    const uint8_t *src = (const uint8_t*)buf;
    while (written < len) {
        uint32_t next = (peer->buf_head + 1) % UNIX_SOCK_BUF_SIZE;
        if (next == peer->buf_tail) break;
        peer->buf[peer->buf_head] = src[written++];
        peer->buf_head = next;
    }
    spinlock_unlock(&peer->lock);
    if (written == 0 && len > 0) {
        usock_trace2("tx-EAGAIN", fd, (int64_t)len);
        return -11; /* EAGAIN: ring full */
    }
    usock_trace2("tx", fd, (int64_t)written);
    return (int64_t)written;
}

int64_t unix_socket_recv(int32_t fd, void *buf, uint64_t len) {
    unix_sock_t *s = usock_get(fd);
    if (!s || !buf) {
        usock_trace2("rx-BADF", fd, (int64_t)len);
        return -14;
    }
    spinlock_lock(&s->lock);
    uint64_t rd = 0;
    uint8_t *dst = (uint8_t*)buf;
    while (rd < len && s->buf_tail != s->buf_head) {
        dst[rd++] = s->buf[s->buf_tail];
        s->buf_tail = (s->buf_tail + 1) % UNIX_SOCK_BUF_SIZE;
    }
    spinlock_unlock(&s->lock);
    if (rd == 0 && len > 0) {
        /* Distinguish "nothing yet" (EAGAIN) from "peer hung up" (EOF/0).
         * libwayland treats a 0-length recv as the compositor closing the
         * connection, so an empty-but-open socket must not return 0. */
        unix_sock_t *peer = (s->connected) ? usock_get(s->peer_fd) : NULL;
        int is_eof = (s->connected && !peer);
        usock_trace2(is_eof ? "rx-EOF" : "rx-EAGAIN", fd, (int64_t)len);
        return is_eof ? 0 : -11; /* 0 = EOF, -11 = EAGAIN */
    }
    usock_trace2("rx", fd, (int64_t)rd);
    return (int64_t)rd;
}

/* msghdr layout matches glibc's struct msghdr on x86-64 (msg_iovlen /
 * msg_controllen are 8-byte). The top-level struct is dereferenced
 * directly (both callers guarantee it is mapped); msg_iov / msg_control
 * point at user memory and are bounds-limited before use. */
struct _kernel_msghdr {
    uint64_t name; uint32_t namelen; uint32_t __pad0;
    uint64_t iov; uint64_t iovlen;
    uint64_t control; uint64_t controllen;
    int32_t flags; uint32_t __pad1;
};

int64_t unix_socket_sendmsg(int32_t fd, uint64_t msg_ptr) {
    struct _kernel_msghdr *msg = (void*)(uintptr_t)msg_ptr;
    if (!msg) return -14;

    /* ---- SCM_RIGHTS: translate each passed memfd into a shared-memory
     *      handle, grant it to the peer's owner, and queue it. ---- */
    if (msg->control && msg->controllen >= sizeof(struct _kernel_cmsghdr) &&
        msg->controllen <= UNIX_CMSG_MAX_BYTES) {
        unix_sock_t *s = usock_get(fd);
        unix_sock_t *peer = (s && s->connected) ? usock_get(s->peer_fd) : NULL;
        if (peer) {
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
                    for (int i = 0; i < num_fds; i++) {
                        int32_t h = syscall_memfd_shm_handle(fds[i]);
                        if (h < 0) {
                            serial_write_string("[unixsock] SCM_RIGHTS: fd is not an shm-backed memfd, dropped\n");
                            continue;
                        }
                        if (shared_memory_grant(h, peer->owner_pid) != 0) {
                            serial_write_string("[unixsock] SCM_RIGHTS: shared_memory_grant failed\n");
                            continue;
                        }
                        (void)shared_memory_addref(h);
                        spinlock_lock(&peer->lock);
                        uint32_t next = (peer->fds_head + 1) % UNIX_SOCK_FD_MAX;
                        if (next != peer->fds_tail) {
                            peer->fd_queue[peer->fds_head] = h;
                            peer->fds_head = next;
                            spinlock_unlock(&peer->lock);
                        } else {
                            spinlock_unlock(&peer->lock);
                            (void)shared_memory_release(h); /* queue full */
                        }
                    }
                }
                offset += (clen + 7u) & ~7u;
            }
        }
    }

    struct { uint64_t base; uint64_t len; } *iov = (void*)(uintptr_t)msg->iov;
    if (!iov || msg->iovlen == 0) return 0;
    uint64_t iovn = msg->iovlen > UNIX_IOV_MAX ? UNIX_IOV_MAX : msg->iovlen;
    int64_t total = 0;
    for (uint32_t i = 0; i < iovn; i++) {
        if (iov[i].len == 0) continue;
        int64_t r = unix_socket_send(fd, (void*)(uintptr_t)iov[i].base, iov[i].len);
        if (r < 0) return total > 0 ? total : r;
        total += r;
        if ((uint64_t)r < iov[i].len) break; /* ring full: partial */
    }
    return total;
}

int64_t unix_socket_recvmsg(int32_t fd, uint64_t msg_ptr) {
    struct _kernel_msghdr *msg = (void*)(uintptr_t)msg_ptr;
    if (!msg) return -14;

    struct { uint64_t base; uint64_t len; } *iov = (void*)(uintptr_t)msg->iov;
    int64_t total = 0;
    if (iov && msg->iovlen > 0) {
        uint64_t iovn = msg->iovlen > UNIX_IOV_MAX ? UNIX_IOV_MAX : msg->iovlen;
        for (uint32_t i = 0; i < iovn; i++) {
            if (iov[i].len == 0) continue;
            int64_t r = unix_socket_recv(fd, (void*)(uintptr_t)iov[i].base, iov[i].len);
            if (r < 0) return total > 0 ? total : r;
            total += r;
            if ((uint64_t)r < iov[i].len) break;
        }
    }

    unix_sock_t *s = usock_get(fd);
    if (s && msg->control &&
        msg->controllen >= sizeof(struct _kernel_cmsghdr) &&
        msg->controllen <= UNIX_CMSG_MAX_BYTES) {
        spinlock_lock(&s->lock);
        uint32_t avail_space = (uint32_t)msg->controllen - (uint32_t)sizeof(struct _kernel_cmsghdr);
        uint32_t max_fds = avail_space / (uint32_t)sizeof(int32_t);
        int32_t *out = (int32_t *)((uint8_t *)(uintptr_t)msg->control + sizeof(struct _kernel_cmsghdr));
        uint32_t written = 0;
        while (written < max_fds && s->fds_tail != s->fds_head) {
            int32_t h = s->fd_queue[s->fds_tail];
            s->fds_tail = (s->fds_tail + 1) % UNIX_SOCK_FD_MAX;
            spinlock_unlock(&s->lock);
            int32_t newfd = (h >= 0)
                ? syscall_memfd_install_shm(h, FILE_O_RDWR_FLAG)
                : -1;
            if (newfd < 0) {
                if (h >= 0) (void)shared_memory_release(h);
            } else {
                out[written++] = newfd; /* adopts the in-flight reference */
            }
            spinlock_lock(&s->lock);
        }
        if (written > 0) {
            struct _kernel_cmsghdr *cmsg = (struct _kernel_cmsghdr *)(uintptr_t)msg->control;
            cmsg->cmsg_len = (uint32_t)sizeof(struct _kernel_cmsghdr) + written * (uint32_t)sizeof(int32_t);
            cmsg->__pad1 = 0;
            cmsg->cmsg_level = 1;
            cmsg->cmsg_type = SCM_RIGHTS;
            msg->controllen = cmsg->cmsg_len;
        } else {
            msg->controllen = 0;
        }
        spinlock_unlock(&s->lock);
    } else if (msg->control) {
        msg->controllen = 0;
    }

    return total;
}

int64_t unix_socket_close(int32_t fd) {
    unix_sock_t *s = usock_get(fd);
    if (!s) return -9;
    spinlock_lock(&s->lock);
    usock_drain_fd_queue(s);
    spinlock_unlock(&s->lock);
    s->used = 0;
    return 0;
}

/* Release every AF_UNIX socket owned by an exiting process. Without this the
 * global g_usocks table leaks a slot per socket per exited process and fills
 * up after a few app launches -- which is exactly what makes a later Xorg's
 * socket() return ENOMEM ("Unable to open socket"). Called from process exit
 * alongside syscall_socket_close_all_for_pid(). */
void unix_socket_close_all_for_pid(int32_t pid) {
    if (!g_usock_init_done) return;
    for (int i = 0; i < UNIX_SOCK_MAX; i++) {
        if (g_usocks[i].used && g_usocks[i].owner_pid == pid) {
            spinlock_lock(&g_usocks[i].lock);
            usock_drain_fd_queue(&g_usocks[i]);
            g_usocks[i].connected = 0;
            g_usocks[i].listening = 0;
            g_usocks[i].peer_fd = -1;
            spinlock_unlock(&g_usocks[i].lock);
            g_usocks[i].used = 0;
        }
    }
}

int64_t unix_socket_pair(int32_t out_fds[2]) {
    if (!out_fds) return -14;
    int64_t fd1 = unix_socket_create(0);
    if (fd1 < 0) return fd1;
    int64_t fd2 = unix_socket_create(0);
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

int unix_socket_fd_in_range(int32_t fd) {
    return fd >= UNIX_SOCK_FD_BASE && fd < UNIX_SOCK_FD_BASE + UNIX_SOCK_MAX;
}

/* Readiness for poll(2)/ppoll(2)/epoll. `events` and the result use the
 * EPOLL / POLL bit values (IN=0x1, OUT=0x4, ERR=0x8, HUP=0x10). Without
 * this, AF_UNIX fds fell through poll's fd-type switch to EPOLLERR and any
 * client polling its Wayland display fd saw a dead connection. */
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

uint32_t unix_socket_poll(int32_t fd, uint32_t events) {
    unix_sock_t *s = usock_get(fd);
    if (!s) return 0x8u; /* EPOLLERR */
    uint32_t r = 0;
    if ((events & 0x1u) != 0u) {          /* POLLIN: bytes buffered, or a
                                           * pending SCM_RIGHTS transfer */
        spinlock_lock(&s->lock);
        int has = (s->buf_tail != s->buf_head) || (s->fds_tail != s->fds_head);
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
