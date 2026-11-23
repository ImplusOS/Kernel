/* Per-disk I/O accounting. See IO_Stats.h. */

#include "IO_Stats.h"

#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"

#include <stddef.h>
#include <string.h>

/* IO_Main.c tracks at most 16 disks; a few more slots cover a unit that
 * shows up through a transport the scan has not listed (a raw read of a
 * USB stick inserted after the first scan). */
#define IO_STATS_MAX_DISKS 24u

typedef struct {
    const void *device;     /* NULL: free slot */
    uint32_t    unit;
    uint32_t    in_flight;
    uint64_t    busy_since; /* valid while in_flight > 0 */
    uint64_t    busy_ns;
    uint64_t    read_ops, write_ops;
    uint64_t    read_bytes, write_bytes;
    uint64_t    read_ns, write_ns;
    uint64_t    errors;
} io_disk_counters_t;

static io_disk_counters_t g_disks[IO_STATS_MAX_DISKS];
/* Every disk together: its busy time is the time *any* disk was busy. */
static io_disk_counters_t g_total;
static spinlock_t         g_io_stats_lock;

/* Caller holds g_io_stats_lock. `create`: take a free slot when the disk
 * has none yet. */
static io_disk_counters_t *slot_for(const void *device, uint32_t unit, bool create)
{
    io_disk_counters_t *free_slot = NULL;
    for (uint32_t i = 0u; i < IO_STATS_MAX_DISKS; ++i) {
        io_disk_counters_t *d = &g_disks[i];
        if (d->device == device && d->unit == unit) {
            return d;
        }
        if (d->device == NULL && free_slot == NULL) {
            free_slot = d;
        }
    }
    if (!create || free_slot == NULL) {
        return NULL;
    }
    memset(free_slot, 0, sizeof(*free_slot));
    free_slot->device = device;
    free_slot->unit = unit;
    return free_slot;
}

io_stats_token_t io_stats_begin(const void *device, uint32_t unit)
{
    uint64_t now = timer_monotonic_ns();
    if (device == NULL) {
        return now;
    }
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_io_stats_lock);
    io_disk_counters_t *d = slot_for(device, unit, true);
    if (d != NULL) {
        if (d->in_flight++ == 0u) {
            d->busy_since = now;
        }
    }
    if (g_total.in_flight++ == 0u) {
        g_total.busy_since = now;
    }
    spinlock_unlock(&g_io_stats_lock);
    irq_restore(flags);
    return now;
}

void io_stats_end(const void *device, uint32_t unit, io_stats_token_t token,
                  bool write, uint32_t sectors, bool ok)
{
    if (device == NULL) {
        return;
    }
    uint64_t now = timer_monotonic_ns();
    uint64_t latency = now >= token ? now - token : 0u;
    uint64_t bytes = ok ? (uint64_t)sectors * 512u : 0u;

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_io_stats_lock);
    io_disk_counters_t *slots[2] = { slot_for(device, unit, false), &g_total };
    for (uint32_t i = 0u; i < 2u; ++i) {
        io_disk_counters_t *d = slots[i];
        if (d == NULL) {
            continue;
        }
        if (d->in_flight > 0u && --d->in_flight == 0u && now >= d->busy_since) {
            d->busy_ns += now - d->busy_since;
        }
        if (write) {
            d->write_ops++;
            d->write_bytes += bytes;
            d->write_ns += latency;
        } else {
            d->read_ops++;
            d->read_bytes += bytes;
            d->read_ns += latency;
        }
        if (!ok) {
            d->errors++;
        }
    }
    spinlock_unlock(&g_io_stats_lock);
    irq_restore(flags);

    if (bytes != 0u) {
        process_perf_note_disk(current_pid_get(), write, bytes);
    }
}

/* Caller holds g_io_stats_lock. */
static void copy_out(const io_disk_counters_t *d, uint64_t now,
                     system_disk_stats_t *out_stats)
{
    if (d != NULL) {
        out_stats->read_ops = d->read_ops;
        out_stats->write_ops = d->write_ops;
        out_stats->read_bytes = d->read_bytes;
        out_stats->write_bytes = d->write_bytes;
        out_stats->read_ns = d->read_ns;
        out_stats->write_ns = d->write_ns;
        out_stats->errors = d->errors;
        out_stats->in_flight = d->in_flight;
        out_stats->busy_ns = d->busy_ns;
        /* A long request still on the device counts up to now, so a disk
         * stuck on one slow read reads as busy, not idle. */
        if (d->in_flight > 0u && now >= d->busy_since) {
            out_stats->busy_ns += now - d->busy_since;
        }
    }
}

void io_stats_get(const void *device, uint32_t unit,
                  system_disk_stats_t *out_stats)
{
    if (out_stats == NULL) {
        return;
    }
    uint64_t now = timer_monotonic_ns();
    memset(out_stats, 0, sizeof(*out_stats));
    out_stats->timestamp_ns = now;

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_io_stats_lock);
    copy_out(device != NULL ? slot_for(device, unit, false) : &g_total,
             now, out_stats);
    spinlock_unlock(&g_io_stats_lock);
    irq_restore(flags);
}
