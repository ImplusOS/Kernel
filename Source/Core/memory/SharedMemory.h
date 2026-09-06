#pragma once

#include <stdint.h>

int32_t shared_memory_create(uint32_t size);
int32_t shared_memory_grant(int32_t handle, int32_t pid);
void *shared_memory_map(int32_t handle);
int32_t shared_memory_unmap(int32_t handle, void *address);
int32_t shared_memory_close(int32_t handle);

/* Release whatever shared-memory mapping the calling address space holds at
 * `address`, whichever object it belongs to. Returns 1 if one was found and
 * released, 0 if `address` is not the start of a shared mapping.
 *
 * munmap(2) names an address, not a handle, so the Linux ABI has no handle to
 * pass to shared_memory_unmap(). Without this, munmap() of a shared region
 * freed the address range through the plain user allocator while the object
 * still recorded a mapping there -- and the next map of that object handed
 * back the stale address, by then owned by some other allocation. */
int shared_memory_unmap_any(void *address);

/* 1 if `addr` lies inside a shared-memory object this address space has
 * mapped. Shared pages are not private anonymous memory, so
 * madvise(MADV_DONTNEED) must leave them alone. */
int shared_memory_addr_is_mapped(uint64_t addr);
void shared_memory_cleanup_process(int32_t pid);

/* Bump the reference count on an existing object without mapping it. Used
 * when an fd that owns a share (e.g. a memfd backed by shared memory) is
 * duplicated, so the object outlives the first close(). Any caller. */
int32_t shared_memory_addref(int32_t handle);

/* Drop a reference taken by shared_memory_addref()/shared_memory_map()'s
 * bookkeeping from a process that is NOT the owner (the owner uses
 * shared_memory_close()). Destroys the object when the count hits zero.
 * Used by the SCM_RIGHTS receiver's fd close path. */
int32_t shared_memory_release(int32_t handle);

/* Size in bytes of the object behind `handle`, or 0 if unknown/invalid. */
uint32_t shared_memory_size(int32_t handle);
