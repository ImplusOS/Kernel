#pragma once

#include <stdint.h>

#ifndef GDT_KERNEL_CODE
#define GDT_KERNEL_CODE 0
#endif
#ifndef GDT_KERNEL_DATA
#define GDT_KERNEL_DATA 0
#endif
#ifndef GDT_USER_CODE
#define GDT_USER_CODE   0
#endif
#ifndef GDT_USER_DATA
#define GDT_USER_DATA   0
#endif

void init_gdt(void);
void gdt_set_kernel_rsp0(uint64_t rsp0);
