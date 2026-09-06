#pragma once

#include <stdint.h>

/*
 * Pseudo-terminals (BSD-style master/slave pairs behind the Linux /dev/ptmx
 * + /dev/pts/N ABI), plus the line discipline that sits between them.
 *
 * A terminal emulator (xterm) holds the master, the shell it forks holds the
 * slave as its stdin/stdout/stderr and controlling terminal. Everything the
 * emulator writes to the master is keyboard input for the shell -- after the
 * line discipline has echoed it, assembled it into a line (ICANON) and turned
 * ^C into SIGINT (ISIG). Everything the shell writes to the slave comes back
 * out of the master with the output post-processing (ONLCR) applied.
 *
 * The nodes are published by DevFS (see DevFS.c). This file owns the pairs
 * themselves and knows nothing about fds; the fd layer reaches it through the
 * vfs_driver_t dev_* hooks.
 *
 * Blocking follows the pipe idiom in Syscall_File.c: poll the ring with
 * process_sleep_current_ms() rather than a wait queue. A read that would block
 * returns to the caller only on data, EOF, or a pending unmasked signal
 * (-EINTR), never on a timeout -- a shell legitimately sits in read() for
 * hours.
 *
 * Error returns are negative Linux errnos, matching the dev_read/dev_ioctl
 * contract in kernel/interfaces/vfs_types.h.
 */

#define PTY_ERR_AGAIN   (-11)  /* EAGAIN */
#define PTY_ERR_BADF     (-9)  /* EBADF  */
#define PTY_ERR_IO       (-5)  /* EIO    */
#define PTY_ERR_INTR     (-4)  /* EINTR  */
#define PTY_ERR_FAULT   (-14)  /* EFAULT */
#define PTY_ERR_INVAL   (-22)  /* EINVAL */
#define PTY_ERR_NOTTY   (-25)  /* ENOTTY */
#define PTY_ERR_NOSPC   (-28)  /* ENOSPC */
#define PTY_ERR_NOMEM   (-12)  /* ENOMEM */

void pty_init(void);

/* Master lifetime. pty_allocate() reserves a free pair and returns its index
 * (the N in /dev/pts/N); pty_master_release() drops the master's reference. */
int32_t pty_allocate(void);
void    pty_master_release(int32_t index);

/* Slave lifetime, one reference per open file description. */
int32_t pty_slave_acquire(int32_t index);
void    pty_slave_release(int32_t index);

/* "/dev/pts/N" -> N, or -1 when `path` is not a slave node. The pair does not
 * have to exist; pty_slave_is_openable() is the existence + unlocked test. */
int32_t pty_index_from_path(const char *path);
int     pty_slave_is_openable(int32_t index);

/* I/O. `is_master` picks the end. Read buffers are USER pointers (the dev_read
 * contract), write buffers are KERNEL pointers (the write_at contract). */
int64_t  pty_read(int32_t index, int is_master, uint8_t *user_buffer,
                  uint64_t length, uint32_t nonblock);
int64_t  pty_write(int32_t index, int is_master, const uint8_t *kernel_buffer,
                   uint64_t length, uint32_t nonblock);
uint32_t pty_poll(int32_t index, int is_master, uint32_t events);
int64_t  pty_ioctl(int32_t index, int is_master, uint64_t request, uint64_t arg);

/* Controlling terminal. pty_ctty_index_for_current() is what /dev/tty opens
 * resolve to; it walks the parent chain so a shell's children inherit it. */
int32_t pty_ctty_index_for_current(void);
void    pty_ctty_bind_current(int32_t index);
void    pty_forget_process(int32_t pid);

/* True once at least one process has this pair as its controlling terminal. */
int pty_is_valid_index(int32_t index);
