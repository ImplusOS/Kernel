#pragma once

#include <stdint.h>

int32_t shared_memory_create(uint32_t size);
int32_t shared_memory_grant(int32_t handle, int32_t pid);
void *shared_memory_map(int32_t handle);
int32_t shared_memory_unmap(int32_t handle, void *address);
int32_t shared_memory_close(int32_t handle);
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
