#ifndef IMPLUSOS_PANIC_H
#define IMPLUSOS_PANIC_H

#include "kernel/boot_info.h"

void kernel_panic_init(BOOT_INFO* boot_info);
void kernel_panic(const char* module_name, const char* message);

#endif