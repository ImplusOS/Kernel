#include "Syscall_Socket.h"

#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Network/tcp/TCP.h"
#include "Network/udp/UDP.h"
#include "Core/timer/Timer.h"
#include "kernel/status.h"

#include <stddef.h>
#include <string.h>

#define SOCKET_TABLE_SIZE 64
/* Must stay >= OS_CONFIG_FILE_MAX_FD_MAX (kernel/config.h) so socket fds
 * (this disjoint numeric range) never collide with the regular file fd
 * table in Syscall_File.c, and SOCKET_FD_BASE + SOCKET_TABLE_SIZE must
 * stay <= POSIX_FD_TABLE_SIZE / FD_SETSIZE (both 1024) since the POSIX
 * layer indexes its per-fd tables directly by this raw kernel fd value. */
#define SOCKET_FD_BASE    512
/* Deliberately numerically identical to Linux's SOCK_STREAM/SOCK_DGRAM
 * (Syscall_LinuxCompat.c's LINUX_SOCK_STREAM/LINUX_SOCK_DGRAM) so that
 * syscall_socket_get_type()'s return value can be compared directly
 * against those without a separate translation table. */
#define SOCKET_TYPE_STREAM 1
#define SOCKET_TYPE_DGRAM  2
#define SOCKET_SOL_SOCKET 1
#define SOCKET_SO_REUSEADDR 2
#define SOCKET_SO_ERROR 4
#define SOCKET_ERR_ETIMEDOUT 110

#define SOCKET_POLL_IN   0x0001u
#define SOCKET_POLL_OUT  0x0004u
#define SOCKET_POLL_ERR  0x0008u
#define SOCKET_POLL_HUP  0x0010u

typedef struct {
    uint8_t used;
    uint8_t type;
    int32_t owner_pid;
    int32_t connection_id;   /* TCP only; always -1 for SOCKET_TYPE_DGRAM. */
    uint16_t bound_port;
    uint16_t backlog;
    uint8_t reuse_address;
    uint8_t shutdown_read;
    uint8_t shutdown_write;
    int32_t last_error;
    /* SOCKET_TYPE_DGRAM only: the connect(2)-recorded default peer, used
     * by plain send()/recv() (as opposed to sendto()/recvfrom(), which
     * pass an explicit destination/get the real source each call). */
    uint32_t udp_peer_ip;
    uint16_t udp_peer_port;
    uint8_t udp_connected;
    /* O_NONBLOCK / SOCK_NONBLOCK / FIONBIO: tracked here because socket fds
     * live outside the generic file-descriptor table (SOCKET_FD_BASE+). */
    uint8_t nonblocking;
} kernel_socket_t;

static kernel_socket_t g_sockets[SOCKET_TABLE_SIZE];
static spinlock_t g_socket_lock;
static int g_socket_initialized;

static void socket_ensure_initialized(void)
{
    if (g_socket_initialized) return;
    spinlock_init(&g_socket_lock);
    memset(g_sockets, 0, sizeof(g_sockets));
    g_socket_initialized = 1;
}

static int32_t socket_index(int32_t fd)
{
    int32_t index = fd - SOCKET_FD_BASE;
    return index >= 0 && index < SOCKET_TABLE_SIZE ? index : -1;
}

static kernel_socket_t *socket_owned_locked(int32_t fd)
{
    int32_t index = socket_index(fd);
    int32_t pid = process_get_current_pid();
    if (index < 0 || pid < 0 || g_sockets[index].used == 0u ||
        g_sockets[index].owner_pid != pid)
        return NULL;
    return &g_sockets[index];
}

static uint16_t allocate_ephemeral_port(void)
{
    uint32_t start = (uint32_t)(timer_ticks() % 16384u);
    for (uint32_t attempt = 0; attempt < 16384u; ++attempt) {
        uint16_t candidate =
            (uint16_t)(49152u + ((start + attempt) % 16384u));
        if (!tcp_local_port_in_use(candidate)) return candidate;
    }
    return 0u;
}

/* Binds `socket` to an ephemeral UDP port if it does not already have
 * one (Linux implicitly does this on the first send()/sendto() from an
 * unbound datagram socket). Caller must hold g_socket_lock. */
static int32_t socket_udp_auto_bind_locked(kernel_socket_t *socket)
{
    if (socket->bound_port != 0u) {
        return 0;
    }
    uint32_t start = (uint32_t)(timer_ticks() % 16384u);
    for (uint32_t attempt = 0; attempt < 4096u; ++attempt) {
        uint16_t candidate = (uint16_t)(49152u + ((start + attempt) % 16384u));
        if (udp_user_bind(socket->owner_pid, candidate) == 0) {
            socket->bound_port = candidate;
            return 0;
        }
    }
    return -1;
}

int32_t syscall_socket_create(int32_t type)
{
    if (type != SOCKET_TYPE_STREAM && type != SOCKET_TYPE_DGRAM) {
        return (int32_t)OS_STATUS_NOT_SUPPORTED;
    }
    int32_t pid = process_get_current_pid();
    if (pid < 0) return (int32_t)OS_STATUS_ACCESS_DENIED;

    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    for (int32_t index = 0; index < SOCKET_TABLE_SIZE; ++index) {
        if (g_sockets[index].used != 0u) continue;
        memset(&g_sockets[index], 0, sizeof(g_sockets[index]));
        g_sockets[index].used = 1u;
        g_sockets[index].type = (uint8_t)type;
        g_sockets[index].owner_pid = pid;
        g_sockets[index].connection_id = -1;
        spinlock_unlock(&g_socket_lock);
        return SOCKET_FD_BASE + index;
    }
    spinlock_unlock(&g_socket_lock);
    return (int32_t)OS_STATUS_LIMIT_REACHED;
}

int32_t syscall_socket_connect(int32_t fd, uint32_t ip, uint16_t port)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL || ip == 0u || port == 0u) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (socket->type == SOCKET_TYPE_DGRAM) {
        /* UDP "connect" just records a default peer for plain send()/
         * recv(); there is no handshake and it never fails for being
         * unreachable (matches Linux). */
        if (socket_udp_auto_bind_locked(socket) < 0) {
            spinlock_unlock(&g_socket_lock);
            return (int32_t)OS_STATUS_LIMIT_REACHED;
        }
        socket->udp_peer_ip = ip;
        socket->udp_peer_port = port;
        socket->udp_connected = 1u;
        spinlock_unlock(&g_socket_lock);
        return 0;
    }
    if (socket->connection_id >= 0) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    uint16_t local_port = socket->bound_port;
    spinlock_unlock(&g_socket_lock);

    if (local_port == 0u) local_port = allocate_ephemeral_port();
    if (local_port == 0u) return (int32_t)OS_STATUS_LIMIT_REACHED;
    int32_t connection_id = tcp_connect(ip, port, local_port);
    if (connection_id < 0) {
        spinlock_lock(&g_socket_lock);
        socket = socket_owned_locked(fd);
        if (socket != NULL) socket->last_error = 5;
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_IO_ERROR;
    }

    spinlock_lock(&g_socket_lock);
    socket = socket_owned_locked(fd);
    if (socket == NULL || socket->connection_id >= 0) {
        spinlock_unlock(&g_socket_lock);
        (void)tcp_close(connection_id);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    socket->connection_id = connection_id;
    socket->bound_port = local_port;
    spinlock_unlock(&g_socket_lock);
    return 0;
}

int32_t syscall_socket_bind(int32_t fd, uint16_t port)
{
    if (port == 0u) return (int32_t)OS_STATUS_INVALID_ARG;
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL || socket->connection_id >= 0 || socket->bound_port != 0u) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (socket->type == SOCKET_TYPE_DGRAM) {
        if (udp_user_bind(socket->owner_pid, port) < 0) {
            spinlock_unlock(&g_socket_lock);
            return (int32_t)OS_STATUS_ACCESS_DENIED;
        }
        socket->bound_port = port;
        spinlock_unlock(&g_socket_lock);
        return 0;
    }
    if (tcp_local_port_in_use(port)) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    for (int32_t index = 0; index < SOCKET_TABLE_SIZE; ++index) {
        kernel_socket_t *other = &g_sockets[index];
        if (other == socket || other->used == 0u ||
            other->bound_port != port) {
            continue;
        }
        if (socket->reuse_address == 0u || other->reuse_address == 0u) {
            spinlock_unlock(&g_socket_lock);
            return (int32_t)OS_STATUS_ACCESS_DENIED;
        }
    }
    socket->bound_port = port;
    spinlock_unlock(&g_socket_lock);
    return 0;
}

int32_t syscall_socket_listen(int32_t fd)
{
    return syscall_socket_listen_with_backlog(fd, 16);
}

int32_t syscall_socket_listen_with_backlog(int32_t fd, int32_t backlog)
{
    if (backlog <= 0) backlog = 1;
    if (backlog > (int32_t)TCP_MAX_CONNECTIONS) {
        backlog = (int32_t)TCP_MAX_CONNECTIONS;
    }
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL || socket->connection_id >= 0 || socket->bound_port == 0u) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    uint16_t port = socket->bound_port;
    spinlock_unlock(&g_socket_lock);

    int32_t connection_id = tcp_listen(port);
    if (connection_id < 0) return (int32_t)OS_STATUS_IO_ERROR;
    if (tcp_set_listen_backlog(connection_id, (uint16_t)backlog) < 0) {
        (void)tcp_close(connection_id);
        return (int32_t)OS_STATUS_IO_ERROR;
    }

    spinlock_lock(&g_socket_lock);
    socket = socket_owned_locked(fd);
    if (socket == NULL || socket->connection_id >= 0) {
        spinlock_unlock(&g_socket_lock);
        (void)tcp_close(connection_id);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    socket->connection_id = connection_id;
    socket->backlog = (uint16_t)backlog;
    spinlock_unlock(&g_socket_lock);
    return 0;
}

int32_t syscall_socket_accept(int32_t fd)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *listener = socket_owned_locked(fd);
    if (listener == NULL || listener->connection_id < 0) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    int32_t listener_id = listener->connection_id;
    int32_t owner_pid = listener->owner_pid;
    spinlock_unlock(&g_socket_lock);

    int32_t connection_id = tcp_accept(listener_id);
    if (connection_id < 0) return -11;

    spinlock_lock(&g_socket_lock);
    for (int32_t index = 0; index < SOCKET_TABLE_SIZE; ++index) {
        if (g_sockets[index].used != 0u) continue;
        memset(&g_sockets[index], 0, sizeof(g_sockets[index]));
        g_sockets[index].used = 1u;
        g_sockets[index].type = SOCKET_TYPE_STREAM;
        g_sockets[index].owner_pid = owner_pid;
        g_sockets[index].connection_id = connection_id;
        tcp_connection_info_t info;
        if (tcp_get_connection_info(connection_id, &info) == 0) {
            g_sockets[index].bound_port = info.local_port;
        }
        spinlock_unlock(&g_socket_lock);
        return SOCKET_FD_BASE + index;
    }
    spinlock_unlock(&g_socket_lock);
    (void)tcp_close(connection_id);
    return (int32_t)OS_STATUS_LIMIT_REACHED;
}

static int32_t socket_connection_id(int32_t fd)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    int32_t connection_id = socket != NULL ? socket->connection_id : -1;
    spinlock_unlock(&g_socket_lock);
    return connection_id;
}

int32_t syscall_socket_send(int32_t fd, const void *data, uint16_t length)
{
    return syscall_socket_sendto(fd, data, length, 0u, 0u);
}

int32_t syscall_socket_recv(int32_t fd, void *data, uint16_t length)
{
    return syscall_socket_recvfrom(fd, data, length, NULL, NULL);
}

/* sendto(2)/UDP: if dst_ip/dst_port are both 0, falls back to the
 * connect(2)-recorded default peer (matches plain send()). */
int32_t syscall_socket_sendto(int32_t fd, const void *data, uint16_t length,
                              uint32_t dst_ip, uint16_t dst_port)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (socket->type == SOCKET_TYPE_DGRAM) {
        uint32_t effective_ip = dst_ip;
        uint16_t effective_port = dst_port;
        if (effective_ip == 0u || effective_port == 0u) {
            if (!socket->udp_connected) {
                spinlock_unlock(&g_socket_lock);
                return (int32_t)OS_STATUS_INVALID_ARG;
            }
            effective_ip = socket->udp_peer_ip;
            effective_port = socket->udp_peer_port;
        }
        if (socket_udp_auto_bind_locked(socket) < 0) {
            spinlock_unlock(&g_socket_lock);
            return (int32_t)OS_STATUS_LIMIT_REACHED;
        }
        uint16_t local_port = socket->bound_port;
        spinlock_unlock(&g_socket_lock);
        return udp_syscall_send(effective_ip, local_port, effective_port,
                                data, length) ?
            (int32_t)length : (int32_t)OS_STATUS_IO_ERROR;
    }
    if (socket->shutdown_write != 0u) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_ACCESS_DENIED;
    }
    spinlock_unlock(&g_socket_lock);
    int32_t connection_id = socket_connection_id(fd);
    if (connection_id < 0) return (int32_t)OS_STATUS_INVALID_ARG;
    int32_t result = tcp_send(connection_id, data, length);
    if (result < 0) {
        /* Connection torn down (RST/FIN) -> EPIPE so the caller (and the
         * SIGPIPE path in the Linux layer) behaves like a POSIX write to a
         * broken stream; a transient failure is still EIO. */
        uint32_t pr = tcp_poll(connection_id, SOCKET_POLL_OUT);
        if ((pr & (SOCKET_POLL_HUP | SOCKET_POLL_ERR)) != 0u) {
            return (int32_t)OS_STATUS_BROKEN_PIPE;
        }
        return (int32_t)OS_STATUS_IO_ERROR;
    }
    return result;
}

/* recvfrom(2)/UDP: reports the datagram's real source via *src_ip_out /
 * *src_port_out (either may be NULL, matching plain recv()). For a
 * SOCKET_TYPE_STREAM socket these instead report the connected peer. */
int32_t syscall_socket_recvfrom(int32_t fd, void *data, uint16_t length,
                                uint32_t *src_ip_out, uint16_t *src_port_out)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (socket->type == SOCKET_TYPE_DGRAM) {
        int32_t owner_pid = socket->owner_pid;
        uint16_t port = socket->bound_port;
        spinlock_unlock(&g_socket_lock);
        if (port == 0u) {
            return 0; /* Nothing bound yet => nothing can have arrived. */
        }
        uint8_t staging[UDP_USER_HEADER_BYTES + 1472u];
        uint32_t want = (uint32_t)length + UDP_USER_HEADER_BYTES;
        if (want > sizeof(staging)) want = sizeof(staging);
        int32_t got = udp_user_recv(owner_pid, port, staging, want);
        if (got < (int32_t)UDP_USER_HEADER_BYTES) {
            return got < 0 ? (int32_t)OS_STATUS_IO_ERROR : 0;
        }
        uint32_t src_ip = (uint32_t)staging[0] | ((uint32_t)staging[1] << 8) |
            ((uint32_t)staging[2] << 16) | ((uint32_t)staging[3] << 24);
        uint16_t src_port = (uint16_t)(staging[4] | (uint16_t)(staging[5] << 8));
        uint16_t payload_len = (uint16_t)(staging[6] | (uint16_t)(staging[7] << 8));
        if (payload_len > length) payload_len = length;
        if (payload_len != 0u && data != NULL) {
            memcpy(data, staging + UDP_USER_HEADER_BYTES, payload_len);
        }
        if (src_ip_out != NULL) *src_ip_out = src_ip;
        if (src_port_out != NULL) *src_port_out = src_port;
        return (int32_t)payload_len;
    }
    if (socket->shutdown_read != 0u) {
        spinlock_unlock(&g_socket_lock);
        return 0;
    }
    int32_t connection_id = socket->connection_id;
    spinlock_unlock(&g_socket_lock);
    if (connection_id < 0) return (int32_t)OS_STATUS_INVALID_ARG;
    if (src_ip_out != NULL || src_port_out != NULL) {
        tcp_connection_info_t info;
        if (tcp_get_connection_info(connection_id, &info) == 0) {
            if (src_ip_out != NULL) *src_ip_out = info.remote_ip;
            if (src_port_out != NULL) *src_port_out = info.remote_port;
        }
    }
    int32_t result = tcp_recv(connection_id, data, length);
    if (result < 0) {
        return (int32_t)OS_STATUS_IO_ERROR;
    }
    if (result == 0 && length != 0u && syscall_socket_is_nonblocking(fd)) {
        /* Non-blocking read with an empty receive buffer. Distinguish a
         * genuine EOF (peer sent FIN -> return 0) from "no data has arrived
         * yet" (-> EAGAIN, so the reader keeps polling instead of treating
         * the socket as closed). tcp_poll reports POLLIN once a FIN is
         * pending even with an empty buffer, and POLLHUP for a fully-closed
         * connection. Blocking sockets keep the legacy "return 0" behaviour
         * (this kernel cannot actually block a syscall anyway). */
        uint32_t pr = tcp_poll(connection_id, SOCKET_POLL_IN);
        if ((pr & (SOCKET_POLL_IN | SOCKET_POLL_HUP | SOCKET_POLL_ERR)) == 0u) {
            return (int32_t)OS_STATUS_WOULD_BLOCK;
        }
    }
    return result;
}

int32_t syscall_socket_close(int32_t fd)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    int32_t connection_id = socket->connection_id;
    int32_t owner_pid = socket->owner_pid;
    uint16_t bound_port = socket->bound_port;
    uint8_t is_dgram = (socket->type == SOCKET_TYPE_DGRAM);
    memset(socket, 0, sizeof(*socket));
    spinlock_unlock(&g_socket_lock);
    if (connection_id >= 0) (void)tcp_close(connection_id);
    if (is_dgram && bound_port != 0u) (void)udp_user_unbind(owner_pid, bound_port);
    return 0;
}

int32_t syscall_socket_get_type(int32_t fd)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    int32_t type = socket != NULL ? (int32_t)socket->type : -1;
    spinlock_unlock(&g_socket_lock);
    return type;
}

int32_t syscall_socket_get_info(int32_t fd, syscall_socket_info_t *info_out)
{
    if (info_out == NULL) return (int32_t)OS_STATUS_INVALID_ARG;
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    int32_t connection_id = socket->connection_id;
    memset(info_out, 0, sizeof(*info_out));
    info_out->local_port = socket->bound_port;
    info_out->error = socket->last_error;
    spinlock_unlock(&g_socket_lock);

    if (connection_id >= 0) {
        tcp_connection_info_t tcp_info;
        if (tcp_get_connection_info(connection_id, &tcp_info) == 0) {
            info_out->local_ip = tcp_info.local_ip;
            info_out->remote_ip = tcp_info.remote_ip;
            info_out->local_port = tcp_info.local_port;
            info_out->remote_port = tcp_info.remote_port;
            info_out->state = (uint32_t)tcp_info.state;
        }
    }
    return 0;
}

int32_t syscall_socket_set_option(int32_t fd, int32_t level,
                                  int32_t option, int32_t value)
{
    if (level != SOCKET_SOL_SOCKET || option != SOCKET_SO_REUSEADDR) {
        return (int32_t)OS_STATUS_NOT_SUPPORTED;
    }
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL || socket->bound_port != 0u ||
        socket->connection_id >= 0) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    socket->reuse_address = value != 0 ? 1u : 0u;
    spinlock_unlock(&g_socket_lock);
    return 0;
}

int32_t syscall_socket_get_option(int32_t fd, int32_t level,
                                  int32_t option, int32_t *value_out)
{
    if (value_out == NULL || level != SOCKET_SOL_SOCKET) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    if (option == SOCKET_SO_REUSEADDR) {
        *value_out = socket->reuse_address != 0u ? 1 : 0;
        spinlock_unlock(&g_socket_lock);
        return 0;
    } else if (option == SOCKET_SO_ERROR) {
        int32_t error = socket->last_error;
        int32_t connection_id = socket->connection_id;
        socket->last_error = 0;
        spinlock_unlock(&g_socket_lock);

        if (error == 0 && connection_id >= 0) {
            tcp_connection_info_t info;
            if (tcp_get_connection_info(connection_id, &info) < 0 ||
                info.state == TCP_STATE_CLOSED) {
                error = SOCKET_ERR_ETIMEDOUT;
            }
        }
        *value_out = error;
        return 0;
    } else {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_NOT_SUPPORTED;
    }
}

int32_t syscall_socket_shutdown(int32_t fd, int32_t how)
{
    if (how < 0 || how > 2) return (int32_t)OS_STATUS_INVALID_ARG;
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL || socket->connection_id < 0) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    int32_t connection_id = socket->connection_id;
    if (how == 0 || how == 2) socket->shutdown_read = 1u;
    int close_write = 0;
    if ((how == 1 || how == 2) && socket->shutdown_write == 0u) {
        socket->shutdown_write = 1u;
        close_write = 1;
    }
    spinlock_unlock(&g_socket_lock);
    if (close_write && tcp_close(connection_id) < 0) {
        return (int32_t)OS_STATUS_IO_ERROR;
    }
    return 0;
}

int64_t syscall_socket_available(int32_t fd)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return (int64_t)OS_STATUS_INVALID_ARG;
    }
    if (socket->type == SOCKET_TYPE_DGRAM) {
        int32_t owner_pid = socket->owner_pid;
        uint16_t port = socket->bound_port;
        spinlock_unlock(&g_socket_lock);
        if (port == 0u) return 0;
        int32_t count = udp_user_available(owner_pid, port);
        /* Datagram count, not byte count, but callers (FIONREAD/select)
         * only use this as a "is there something to read" signal. */
        return count > 0 ? (int64_t)count : 0;
    }
    spinlock_unlock(&g_socket_lock);
    int32_t connection_id = socket_connection_id(fd);
    if (connection_id < 0) return (int64_t)OS_STATUS_INVALID_ARG;
    tcp_connection_info_t info;
    if (tcp_get_connection_info(connection_id, &info) < 0) {
        return (int64_t)OS_STATUS_IO_ERROR;
    }
    return (int64_t)info.receive_available;
}

uint32_t syscall_socket_poll(int32_t fd, uint32_t events)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return SOCKET_POLL_ERR;
    }
    if (socket->type == SOCKET_TYPE_DGRAM) {
        int32_t owner_pid = socket->owner_pid;
        uint16_t port = socket->bound_port;
        spinlock_unlock(&g_socket_lock);
        uint32_t ready = SOCKET_POLL_OUT; /* UDP send is never backpressured here. */
        if (port != 0u && udp_user_available(owner_pid, port) > 0) {
            ready |= SOCKET_POLL_IN;
        }
        return ready & events;
    }
    spinlock_unlock(&g_socket_lock);
    int32_t connection_id = socket_connection_id(fd);
    if (connection_id < 0) return SOCKET_POLL_ERR;
    return tcp_poll(connection_id, events);
}

int32_t syscall_socket_fd_in_range(int32_t fd)
{
    return socket_index(fd) >= 0 ? 1 : 0;
}

int32_t syscall_socket_set_nonblocking(int32_t fd, int32_t enabled)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    socket->nonblocking = enabled ? 1u : 0u;
    spinlock_unlock(&g_socket_lock);
    return 0;
}

int32_t syscall_socket_get_status_flags(int32_t fd)
{
    socket_ensure_initialized();
    spinlock_lock(&g_socket_lock);
    kernel_socket_t *socket = socket_owned_locked(fd);
    if (socket == NULL) {
        spinlock_unlock(&g_socket_lock);
        return (int32_t)OS_STATUS_INVALID_ARG;
    }
    int32_t flags = socket->nonblocking ? 0x0800 : 0; /* O_NONBLOCK */
    spinlock_unlock(&g_socket_lock);
    return flags;
}

int32_t syscall_socket_is_nonblocking(int32_t fd)
{
    int32_t flags = syscall_socket_get_status_flags(fd);
    return flags < 0 ? 0 : ((flags & 0x0800) != 0 ? 1 : 0);
}

void syscall_socket_close_all_for_pid(int32_t pid)
{
    if (pid < 0) return;
    socket_ensure_initialized();
    for (;;) {
        int32_t connection_id = -1;
        uint16_t bound_port = 0u;
        uint8_t is_dgram = 0u;
        int found = 0;
        spinlock_lock(&g_socket_lock);
        for (int32_t index = 0; index < SOCKET_TABLE_SIZE; ++index) {
            if (g_sockets[index].used == 0u ||
                g_sockets[index].owner_pid != pid)
                continue;
            connection_id = g_sockets[index].connection_id;
            bound_port = g_sockets[index].bound_port;
            is_dgram = (g_sockets[index].type == SOCKET_TYPE_DGRAM);
            memset(&g_sockets[index], 0, sizeof(g_sockets[index]));
            found = 1;
            break;
        }
        spinlock_unlock(&g_socket_lock);
        if (!found) break;
        if (connection_id >= 0) (void)tcp_close(connection_id);
        if (is_dgram && bound_port != 0u) (void)udp_user_unbind(pid, bound_port);
    }
    udp_user_release_all(pid); /* Belt-and-suspenders against any leaked binding. */
}
