#pragma once

#include <stdint.h>

/*
 * Per-process file descriptor tables for Linux-ABI processes.
 *
 * The kernel's descriptor tables (Syscall_File.c, UnixSocket.c,
 * Syscall_Socket.c, Syscall_Epoll.c) are global: a descriptor *number* names
 * one object system-wide, and each backend tracks which processes may use it.
 * That cannot express what every multi-process Linux program does between
 * fork and exec -- put an object on a number of its own choosing, typically
 * 0/1/2 for stdio and 3, 4, ... for inherited channels -- because two
 * processes cannot both have their own fd 0.
 *
 * This layer gives every Linux-ABI process (keyed by its memory owner, so
 * threads share one) a private table from the numbers it sees ("user fds")
 * to the kernel's global numbers ("global fds"). The Linux syscall
 * dispatcher translates on the way in and allocates on the way out; the
 * backends underneath keep working in global numbers and are unchanged.
 *
 * dup/dup2/dup3/F_DUPFD never reach a backend: they are two user numbers for
 * one global object, which is exactly POSIX's "two descriptors, one open file
 * description". A global object is closed in its backend only when the last
 * user number referring to it in this process goes away.
 */

#define LXFD_MAX 1024

/* Called from process_fork() before the child can run: the child starts with
 * a copy of the parent's table. */
void lxfd_fork(int32_t parent_pid, int32_t child_pid);

/* Called when a process exits: forget its table. Invokes `on_ref` once for
 * every global the table held (one call per distinct global), so refcounted
 * backends can drop this process's reference. */
void lxfd_release_process(int32_t pid, void (*on_ref)(int32_t global));

/* Global for a user fd of the calling process, or -1. */
int32_t lxfd_get(int32_t ufd);

/* Bind `global` to the lowest free user fd >= min_ufd. Returns the user fd or
 * -24 (EMFILE). */
int32_t lxfd_install(int32_t global, int cloexec, int32_t min_ufd);

/* dup2 onto `ufd`. *replaced receives the global previously bound there
 * (or -1), and *replaced_last is set when that was its last user fd in this
 * process -- the caller then closes it in its backend. Returns ufd or -9. */
int32_t lxfd_set(int32_t ufd, int32_t global, int cloexec,
                 int32_t *replaced, int *replaced_last);

/* close(). *global_out receives what it referred to; returns 1 when that was
 * the last user fd for it (backend close due), 0 when others remain, -9 when
 * `ufd` was not open. */
int lxfd_remove(int32_t ufd, int32_t *global_out);

int lxfd_get_cloexec(int32_t ufd);
int lxfd_set_cloexec(int32_t ufd, int on);

/* execve() succeeded: drop every close-on-exec user fd. `closer` is called for
 * each global that loses its last user fd. */
void lxfd_exec_close_cloexec(void (*closer)(int32_t global));

/* Ascending walk of a process's user fds (for /proc/<pid>/fd). -1 to start,
 * -1 when done. Returns -2 if the process has no table (not a Linux process
 * or not yet started), so the caller can fall back to the global tables. */
int32_t lxfd_next_open(int32_t pid, int32_t after);

/* Global number behind a user fd of an arbitrary process, or -1. */
int32_t lxfd_get_for(int32_t pid, int32_t ufd);
