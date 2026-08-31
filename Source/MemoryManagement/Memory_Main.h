#ifndef MEMORY_MAIN_H
#define MEMORY_MAIN_H

#include <stdint.h>
#include <stddef.h>

void* malloc(uint64_t size);
void* malloc_sensitive(uint64_t size);
void  free(void* ptr);
void  free_sensitive(void* ptr);
void* calloc(uint64_t num, uint64_t size);
void* realloc(void* ptr, uint64_t new_size);



void memory_init(void);
void init_physical_memory(void *memory_map, size_t map_size, size_t desc_size, uint64_t detected_pages);
void physical_memory_reserve_region(uint64_t base, uint64_t size);

void* alloc_page(void);
void  free_page(void* addr);
void* alloc_contiguous_pages(uint32_t page_count, uint32_t align_pages);
void  free_contiguous_pages(void* addr, uint32_t page_count);

/* Physical-page reference counting for copy-on-write fork
 * (TODO_Chromium_LinuxABI.md bucket B). Must be armed once after the heap is
 * up via memory_init_page_refcounts(); until then every pmm_page_ref_* call
 * is a no-op and free_page() behaves exactly as before (single owner).
 *
 * Convention: a refcount of 0 or 1 means "one owner" - free_page() releases
 * the frame. pmm_page_ref_inc() takes a frame from 0/1 up to 2 (now shared);
 * pmm_page_ref_dec() drops it and returns the new count, and free_page()
 * only actually releases the frame once the count is back to <= 1. Counts
 * saturate at 255 (a frame shared that widely is simply never reclaimed). */
void     memory_init_page_refcounts(void);
void     pmm_page_ref_inc(uint64_t phys_addr);
uint32_t pmm_page_ref_dec(uint64_t phys_addr);
uint32_t pmm_page_ref_get(uint64_t phys_addr);

uint64_t get_free_memory(void);
uint64_t get_used_memory(void);
/* Physical pages currently free (not kernel-heap bytes -- see
 * get_free_memory(), which reports the heap). */
uint64_t memory_free_pages(void);
uint64_t get_total_memory_pages(void);
void     memory_dump_virtual(const void *addr, uint32_t bytes);
void     memory_dump_physical(uint64_t phys_addr, uint32_t bytes);

#endif
