#pragma once

#include <stdint.h>

int32_t shared_memory_create(uint32_t size);
int32_t shared_memory_grant(int32_t handle, int32_t pid);
void *shared_memory_map(int32_t handle);

/* As shared_memory_map(), but always returns a fresh mapping of the same
 * pages rather than aliasing one this address space already has. mmap(2)
 * semantics -- see the note on shared_memory_map_ex(). */
void *shared_memory_map_new(int32_t handle);
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

/* A process-independent identity for a user address inside a shared-memory
 * mapping of the calling process (object + offset), or 0 when the address is
 * not in one. Shared (non-PRIVATE) futexes are keyed on this so that a waiter
 * and a waker in different processes -- which map the object at different
 * addresses -- find each other. */
uint64_t shared_memory_addr_key(uint64_t addr);

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

/* Let any address space map the object (see shared_object_t.is_public). */
int32_t shared_memory_set_public(int32_t handle);

/* Byte-level access to the object's pages from kernel code: copy `len` bytes
 * at `offset` in or out. Fails if the range runs past the object. */
int32_t shared_memory_copy_in(int32_t handle, uint32_t offset,
                              const uint8_t *data, uint32_t len);
int32_t shared_memory_copy_out(int32_t handle, uint32_t offset,
                               uint8_t *out, uint32_t len);
