/* GPU usage accounting. See GpuStats.h. */

#include "GpuStats.h"

#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "Drivers/Module/Display_Main.h"
#include "Drivers/Module/DriverBinary.h"

#include <stddef.h>
#include <string.h>

/* Submissions waiting to retire. Mesa keeps a few frames in flight, so a
 * few hundred is plenty; when it does overflow, the oldest record is
 * dropped (its time goes uncharged), never the accounting as a whole. */
#define GPU_FENCE_LOG 256u

/* Fences retiring together share the batch's engine time; this caps the
 * per-batch scratch list (anything past it is charged to the last entry). */
#define GPU_RETIRE_BATCH 64u

typedef struct {
    uint64_t fence;
    uint64_t submit_ns;
    int32_t  pid;
} gpu_fence_rec_t;

static spinlock_t g_gpu_stats_lock;

static gpu_fence_rec_t g_fences[GPU_FENCE_LOG];
static uint32_t g_fence_head;           /* oldest record */
static uint32_t g_fence_count;
static uint64_t g_last_retire_ns;
static uint64_t g_fences_submitted;
static uint64_t g_fences_retired;

static uint64_t g_present_count;
static uint64_t g_present_bytes;
static uint64_t g_present_busy_ns;      /* union of present intervals */
static uint64_t g_present_busy_until;

static int64_t  g_gpu_memory_bytes;

void gpu_stats_note_present(int32_t pid, uint64_t start_ns, uint64_t end_ns,
                            uint64_t bytes)
{
    if (end_ns < start_ns) {
        end_ns = start_ns;
    }
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_gpu_stats_lock);
    g_present_count++;
    g_present_bytes += bytes;
    /* Two CPUs presenting at once keep the path busy once, not twice. */
    if (start_ns >= g_present_busy_until) {
        g_present_busy_ns += end_ns - start_ns;
        g_present_busy_until = end_ns;
    } else if (end_ns > g_present_busy_until) {
        g_present_busy_ns += end_ns - g_present_busy_until;
        g_present_busy_until = end_ns;
    }
    spinlock_unlock(&g_gpu_stats_lock);
    irq_restore(flags);

    process_perf_note_gpu(pid, PROCESS_GPU_ENGINE_COPY, end_ns - start_ns);
}

void gpu_stats_note_submit(int32_t pid, uint64_t fence)
{
    if (fence == 0u) {
        return;
    }
    uint64_t now = timer_monotonic_ns();
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_gpu_stats_lock);
    if (g_fence_count == GPU_FENCE_LOG) {
        g_fence_head = (g_fence_head + 1u) % GPU_FENCE_LOG;
        g_fence_count--;
    }
    gpu_fence_rec_t *r = &g_fences[(g_fence_head + g_fence_count) % GPU_FENCE_LOG];
    r->fence = fence;
    r->submit_ns = now;
    r->pid = pid;
    g_fence_count++;
    g_fences_submitted++;
    spinlock_unlock(&g_gpu_stats_lock);
    irq_restore(flags);
}

void gpu_stats_note_sync(int32_t pid, uint64_t busy_ns)
{
    process_perf_note_gpu(pid, PROCESS_GPU_ENGINE_3D, busy_ns);
}

void gpu_stats_note_retired(uint64_t done_fence)
{
    int32_t  pids[GPU_RETIRE_BATCH];
    uint32_t n = 0u;
    uint64_t span = 0u;

    uint64_t now = timer_monotonic_ns();
    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_gpu_stats_lock);
    if (g_fence_count != 0u && g_fences[g_fence_head].fence <= done_fence) {
        /* The engine has been on this batch since the later of the first
         * submission and the previous retirement. */
        uint64_t start = g_fences[g_fence_head].submit_ns;
        if (g_last_retire_ns > start) {
            start = g_last_retire_ns;
        }
        span = now > start ? now - start : 0u;
        while (g_fence_count != 0u && g_fences[g_fence_head].fence <= done_fence) {
            if (n < GPU_RETIRE_BATCH) {
                pids[n++] = g_fences[g_fence_head].pid;
            }
            g_fence_head = (g_fence_head + 1u) % GPU_FENCE_LOG;
            g_fence_count--;
            g_fences_retired++;
        }
        g_last_retire_ns = now;
    }
    spinlock_unlock(&g_gpu_stats_lock);
    irq_restore(flags);

    if (n == 0u || span == 0u) {
        return;
    }
    /* One process's whole batch is the common case (the compositor, or the
     * one game on screen): charge it in one go. */
    uint64_t share = span / n;
    for (uint32_t i = 0u; i < n; ++i) {
        uint64_t part = (i + 1u == n) ? span - share * (n - 1u) : share;
        process_perf_note_gpu(pids[i], PROCESS_GPU_ENGINE_3D, part);
    }
}

void gpu_stats_note_memory(int64_t delta_bytes)
{
    __atomic_fetch_add(&g_gpu_memory_bytes, delta_bytes, __ATOMIC_RELAXED);
}

static void take_engine(system_gpu_usage_t *out, uint32_t engine,
                        const driver_gpu_stats_t *ds, uint32_t driver_engine)
{
    out->engine_flags[engine] = SYSTEM_GPU_ENGINE_FLAG_DEVICE;
    out->engine_busy_ns[engine] = ds->busy_ns[driver_engine];
    out->engine_commands[engine] = ds->commands[driver_engine];
}

os_status_t sysinfo_get_gpu_usage(system_gpu_usage_t *out_usage)
{
    if (out_usage == NULL) {
        return OS_STATUS_INVALID_ARG;
    }
    memset(out_usage, 0, sizeof(*out_usage));
    out_usage->engine_count = 2u;

    /* Render and scanout are separate roles (Display_Main.c): a VirtIO GPU
     * may serve the render node while a firmware framebuffer owns the
     * panel. Each engine comes from the device that does that job. */
    const driver_display_t *render = display_get_3d_driver();
    const driver_display_t *scanout = display_get_active_driver();
    driver_gpu_stats_t ds;
    bool copy_done = false;

    if (render != NULL && render->gpu_get_stats != NULL &&
        render->gpu_get_stats(&ds)) {
        take_engine(out_usage, SYSTEM_GPU_ENGINE_3D, &ds, DRIVER_GPU_ENGINE_3D);
        out_usage->command_bytes = ds.command_bytes;
        out_usage->dedicated_memory_bytes += ds.dedicated_memory_bytes;
        if (render == scanout) {
            take_engine(out_usage, SYSTEM_GPU_ENGINE_COPY, &ds,
                        DRIVER_GPU_ENGINE_COPY);
            copy_done = true;
        }
    }
    if (!copy_done && scanout != NULL && scanout != render &&
        scanout->gpu_get_stats != NULL && scanout->gpu_get_stats(&ds)) {
        take_engine(out_usage, SYSTEM_GPU_ENGINE_COPY, &ds,
                    DRIVER_GPU_ENGINE_COPY);
        out_usage->dedicated_memory_bytes += ds.dedicated_memory_bytes;
        copy_done = true;
    }

    uint64_t flags = irq_save_disable();
    spinlock_lock(&g_gpu_stats_lock);
    if (!copy_done && scanout != NULL) {
        out_usage->engine_flags[SYSTEM_GPU_ENGINE_COPY] =
            SYSTEM_GPU_ENGINE_FLAG_SOFTWARE;
        out_usage->engine_busy_ns[SYSTEM_GPU_ENGINE_COPY] = g_present_busy_ns;
        out_usage->engine_commands[SYSTEM_GPU_ENGINE_COPY] = g_present_count;
    }
    out_usage->present_count = g_present_count;
    out_usage->present_bytes = g_present_bytes;
    out_usage->fences_submitted = g_fences_submitted;
    out_usage->fences_retired = g_fences_retired;
    spinlock_unlock(&g_gpu_stats_lock);
    irq_restore(flags);

    int64_t mem = __atomic_load_n(&g_gpu_memory_bytes, __ATOMIC_RELAXED);
    out_usage->shared_memory_bytes = mem > 0 ? (uint64_t)mem : 0u;
    /* Sampled last, so every busy total above is at or before it. */
    out_usage->timestamp_ns = timer_monotonic_ns();
    return OS_STATUS_OK;
}
