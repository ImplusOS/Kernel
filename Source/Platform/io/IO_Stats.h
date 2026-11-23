#pragma once

/*
 * Per-disk I/O accounting for the block layer.
 *
 * IO_Main.c brackets every request that reaches a medium -- a block-cache
 * miss, a write, a raw read or write from the installer -- with
 * io_stats_begin() / io_stats_end(). A disk is named the way IO_Main.c names
 * it internally, by its transport (the block_device_t) and the unit index
 * within it, so the counters are kept before the disk list that
 * sysinfo_get_disk_info() numbers has even been scanned.
 *
 * Besides operation and byte counts, each disk keeps its busy time: the
 * wall-clock time during which at least one request was outstanding. That is
 * what a task manager shows as a disk's "active time"; it cannot exceed the
 * elapsed time however many requests overlap.
 *
 * The bytes are also charged to the process that issued the request
 * (process_t.disk_read_bytes / disk_write_bytes), so per-process disk rates
 * add up to the per-disk ones.
 */

#include <stdbool.h>
#include <stdint.h>

#include "kernel/system_info.h"

/* Opaque begin token: the start time of the request. */
typedef uint64_t io_stats_token_t;

io_stats_token_t io_stats_begin(const void *device, uint32_t unit);

/* `sectors` are 512-byte units, as every disk_* call counts them. */
void io_stats_end(const void *device, uint32_t unit, io_stats_token_t token,
                  bool write, uint32_t sectors, bool ok);

/* Counters of one disk, all zero for one that has seen no I/O yet. The
 * busy time includes the request in flight now, if any. A NULL `device`
 * reads every disk together, whose busy time is the time any disk was
 * busy. */
void io_stats_get(const void *device, uint32_t unit,
                  system_disk_stats_t *out_stats);
