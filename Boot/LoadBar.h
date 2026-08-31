#pragma once

#include <stdint.h>
#include "kernel/boot_info.h"

void load_bar_init(BOOT_INFO* boot_info);

void load_bar_set_target(uint32_t percent);
void load_bar_update(void);
void load_bar_tick(uint64_t tick);
void load_bar_finish(void);
