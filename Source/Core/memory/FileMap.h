#pragma once

/*
 * FileMap -- demand-paged file-backed mmap().
 *
 * The Linux compat layer used to service a file-backed mmap() by allocating
 * physical pages for the whole mapping and reading the file into them up
 * front. glibc's loader maps each shared object twice over (one mapping
 * spanning the whole image, then MAP_FIXED segment mappings on top), so a
 * process like Xorg -- ~180 shared objects, libLLVM alone 129 MB -- ended up
 * with a multi-gigabyte resident set. fork() then had to copy all of it: one
 * measured fork ran 420 s and died with the machine out of physical memory,
 * which is what stopped Xorg from running xkbcomp.
 *
 * Instead, a large file mapping now reserves its address range with no
 * physical backing and registers here. The #PF handler calls
 * filemap_handle_fault() before the demand-zero path; a fault inside a
 * registered range allocates one page, reads that page's worth of the file
 * into it, and maps it. Pages the process never touches are never allocated
 * and never read.
 *
 * Each record holds a reference on the *open file description*, not on the
 * fd -- POSIX keeps a mapping valid after the fd is closed, and glibc's
 * loader closes it immediately after mapping.
 */

#include <stdint.h>

/* Register `length` bytes at `start` as backed by `file_handle` (from
 * syscall_file_mmap_acquire(), whose reference this takes over) beginning at
 * `file_offset`. `page_flags` are the PAGE_* bits to install on each page
 * faulted in. Returns 0, or -1 when the table is full -- in which case the
 * caller must fall back to an eager mapping. */
int  filemap_register(int32_t pid, uint64_t start, uint64_t length,
                      int32_t file_handle, uint64_t file_offset,
                      uint64_t page_flags);

/* Punch a zero-fill hole over [start, start+length): faults there stop being
 * served from a file and go back to the demand-zero path. Needed because an
 * anonymous MAP_FIXED can land on top of an existing file mapping -- which is
 * exactly how a shared object's .bss is laid over the mapping of its own
 * image. Without it those pages come back holding file bytes instead of
 * zeroes. Cheap no-op when nothing is mapped there. */
int  filemap_register_zero(int32_t pid, uint64_t start, uint64_t length);

/* #PF hook. Returns 1 when the fault was inside a registered mapping and the
 * page is now present, 0 otherwise (caller continues to its own handling).
 * Runs with interrupts disabled: the read below reaches the VFS and the block
 * drivers, which poll rather than sleep. */
int  filemap_handle_fault(int32_t pid, uint64_t cr3, uint64_t fault_addr);

/* munmap(): drops records wholly contained in [start, start+length). */
void filemap_unregister_range(int32_t pid, uint64_t start, uint64_t length);

/* 1 if `addr` falls in a mapping whose pages come from a file rather than from
 * the demand-zero path. madvise(MADV_DONTNEED) needs this: on Linux the range
 * reads back as zeroes only for private anonymous memory -- a file-backed or
 * shared mapping re-reads the underlying data instead -- so zeroing one of
 * those in place destroys it. */
int  filemap_addr_is_file_backed(int32_t pid, uint64_t addr);

/* Diagnostics: records in use, for `pid` (or for everyone when pid < 0),
 * and the size of the table. */
uint32_t filemap_debug_count(int32_t pid);
uint32_t filemap_debug_capacity(void);

/* Frees the shared page cache's frames that no mapping holds any more;
 * returns how many. For a fault about to fail for want of memory (and run
 * at every exit/exec). */
uint64_t filemap_reclaim_shared_pages(void);

/* Diagnostics: frames in the shared page cache, and in *unmapped_out how
 * many of them no mapping holds (reclaimable). */
uint32_t filemap_debug_shared_pages(uint32_t *unmapped_out);
/* hits, inserts, refused-when-full, sweeps, ns spent sweeping, frames freed */
void filemap_debug_cache_stats(uint64_t out[6]);
/* file reads by the fault path: count, pages, ns */
void filemap_debug_read_stats(uint64_t out[3]);

/* execve()/exit(): drops every record owned by `pid`. */
void filemap_release_pid(int32_t pid);

/* fork(): gives the child its own records (and its own file references) for
 * everything the parent has mapped. Returns 0, or -1 having left the child
 * with none. */
int  filemap_clone_for_fork(int32_t parent_pid, int32_t child_pid);
