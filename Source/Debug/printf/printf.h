#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "kernel/boot_info.h"

void debugger_init(BOOT_INFO *boot_info);
bool debugger_display_init(void);
void debug_putchar(char c);
void debug_clear_screen(void);
