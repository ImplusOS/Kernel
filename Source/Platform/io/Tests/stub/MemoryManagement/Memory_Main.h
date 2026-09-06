#pragma once
#include <stdint.h>
#include <stdlib.h>   /* calloc/free come straight from the host libc */

/* Host stand-ins for the kernel allocator. The harness sets
 * g_stub_total_pages to size the cache and g_stub_pages_left to exercise the
 * "out of pages" path. */
extern uint64_t g_stub_total_pages;
extern int64_t  g_stub_pages_left;

static inline uint64_t get_total_memory_pages(void) { return g_stub_total_pages; }
void *alloc_page(void);
void  free_page(void *addr);
