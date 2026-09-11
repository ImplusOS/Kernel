#pragma once

#include <stdint.h>
#include "kernel/boot_info.h"

void load_bar_init(BOOT_INFO* boot_info);

void load_bar_set_target(uint32_t percent);
void load_bar_update(void);
void load_bar_tick(uint64_t tick);

/* Freeze the spinner where it stands: no further frames are drawn, but the
 * last one is left on the panel. Used just before the boot hand-off animation
 * captures the screen (Kernel/Source/Boot/BootAnim.c) so the spinner scales
 * and dissolves with the rest of the boot screen instead of blinking out. */
void load_bar_stop(void);

/* Stop the spinner and erase it from the panel. */
void load_bar_finish(void);
