#pragma once

/*
 * GPU usage accounting -- the kernel half of system_gpu_usage_t.
 *
 * The machine-wide engine numbers come from the display drivers
 * (driver_display_t.gpu_get_stats: requests in flight on the device). What
 * only the kernel can see is collected here:
 *
 *   presents   Display_Main.c times every present. On a display with no
 *              engine of its own (a firmware framebuffer, ramfb, ...) that
 *              CPU copy *is* the scanout path, and its busy time stands in
 *              for the copy engine. Either way the time is charged to the
 *              presenting process as copy-engine time.
 *   fences     The DRM render node notes which process each fenced
 *              submission came from; when drm_timer_tick() sees fences
 *              retire, the engine time since the previous retirement is
 *              shared among the processes whose work just finished. That
 *              is what gives every process its own 3D-engine time.
 *   memory     Pages the DRM layer holds for GPU buffers.
 *
 * Every note_* function is safe from interrupt context.
 */

#include <stdbool.h>
#include <stdint.h>

#include "kernel/status.h"
#include "kernel/system_info.h"

/* A present that ran from start_ns to end_ns and pushed `bytes` of pixels. */
void gpu_stats_note_present(int32_t pid, uint64_t start_ns, uint64_t end_ns,
                            uint64_t bytes);

/* A fenced submission of `pid`'s was queued as `fence` (fence ids rise
 * monotonically and retire in order). */
void gpu_stats_note_submit(int32_t pid, uint64_t fence);

/* A synchronous (unfenced) GPU request of `pid`'s took `busy_ns`. */
void gpu_stats_note_sync(int32_t pid, uint64_t busy_ns);

/* Every fence up to and including `done_fence` has retired. */
void gpu_stats_note_retired(uint64_t done_fence);

/* GPU buffer memory came (`delta` > 0) or went. */
void gpu_stats_note_memory(int64_t delta_bytes);

os_status_t sysinfo_get_gpu_usage(system_gpu_usage_t *out_usage);
