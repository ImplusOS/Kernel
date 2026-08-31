#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "kernel/boot_info.h"

void debugger_init(BOOT_INFO *boot_info);
bool debugger_display_init(void);
void debug_printf(const char *format, ...);
void debug_putchar(char c);
void debug_clear_screen(void);
void debug_reset_cursor(void);
void debug_present(void);