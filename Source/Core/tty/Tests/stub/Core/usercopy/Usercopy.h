#pragma once
#include <stdint.h>
#include <string.h>
/* No separate address space in the harness: a "user" pointer is just memory.
 * Both return 0 on success, matching the kernel contract. */
static inline uint64_t copy_from_user(void *dst, const void *src, uint64_t n)
{ if (!dst || !src) return 1u; memcpy(dst, src, (size_t)n); return 0u; }
static inline uint64_t copy_to_user(void *dst, const void *src, uint64_t n)
{ if (!dst || !src) return 1u; memcpy(dst, src, (size_t)n); return 0u; }
