#pragma once

#include <stdint.h>

/*
 * SysV shared memory (shmget/shmat/shmdt/shmctl) for the Linux x86_64 ABI.
 *
 * Chromium's X11 backend uses MIT-SHM for its software output path: it calls
 * shmget(IPC_PRIVATE, ...) to allocate the image buffer, passes the shmid to
 * the X server over the wire, and both sides attach the same pages.  Without
 * these syscalls the probe returned ENOSYS and every frame went through
 * XPutImage over the Unix socket instead (a full-window copy per frame).
 *
 * Segments are backed by the kernel's shared-memory objects, so the pages are
 * genuinely shared between the client and Xorg.
 */

int64_t linux_sysv_shmget(uint64_t key, uint64_t size, uint64_t shmflg);
int64_t linux_sysv_shmat(uint64_t shmid, uint64_t shmaddr, uint64_t shmflg);
int64_t linux_sysv_shmdt(uint64_t shmaddr);
int64_t linux_sysv_shmctl(uint64_t shmid, uint64_t cmd, uint64_t buf);

/* Drop segments created by `pid` when its address space goes away.  Called
 * from the process teardown path; attached mappings keep the backing object
 * alive until they are unmapped. */
void linux_sysv_shm_cleanup_process(int32_t pid);
