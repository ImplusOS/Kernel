#pragma once

#include <stdint.h>

typedef struct {
    uint32_t local_ip;
    uint32_t remote_ip;
    uint16_t local_port;
    uint16_t remote_port;
    uint32_t state;
    int32_t error;
} syscall_socket_info_t;

int32_t syscall_socket_create(int32_t type);
int32_t syscall_socket_get_type(int32_t fd);
int32_t syscall_socket_connect(int32_t fd, uint32_t ip, uint16_t port);
int32_t syscall_socket_bind(int32_t fd, uint16_t port);
int32_t syscall_socket_listen(int32_t fd);
int32_t syscall_socket_listen_with_backlog(int32_t fd, int32_t backlog);
int32_t syscall_socket_accept(int32_t fd);
int32_t syscall_socket_send(int32_t fd, const void *data, uint16_t length);
int32_t syscall_socket_recv(int32_t fd, void *data, uint16_t length);
int32_t syscall_socket_sendto(int32_t fd, const void *data, uint16_t length,
                              uint32_t dst_ip, uint16_t dst_port);
int32_t syscall_socket_recvfrom(int32_t fd, void *data, uint16_t length,
                                uint32_t *src_ip_out, uint16_t *src_port_out);
int32_t syscall_socket_close(int32_t fd);
int32_t syscall_socket_get_info(int32_t fd, syscall_socket_info_t *info_out);
int32_t syscall_socket_set_option(int32_t fd, int32_t level,
                                  int32_t option, int32_t value);
int32_t syscall_socket_get_option(int32_t fd, int32_t level,
                                  int32_t option, int32_t *value_out);
int32_t syscall_socket_shutdown(int32_t fd, int32_t how);
int64_t syscall_socket_available(int32_t fd);
uint32_t syscall_socket_poll(int32_t fd, uint32_t events);
void syscall_socket_close_all_for_pid(int32_t pid);

/* Non-zero if fd is in the disjoint socket-fd numeric range (>= SOCKET_FD_BASE),
 * i.e. it belongs to this table rather than the generic file-descriptor table. */
int32_t syscall_socket_fd_in_range(int32_t fd);
/* O_NONBLOCK handling for socket fds (the generic file fd table does not cover
 * them). get returns the Linux O_NONBLOCK bit (0x800) if set, 0 if clear, and
 * a negative OS_STATUS on a bad/unowned fd. */
int32_t syscall_socket_set_nonblocking(int32_t fd, int32_t enabled);
int32_t syscall_socket_get_status_flags(int32_t fd);
int32_t syscall_socket_is_nonblocking(int32_t fd);
