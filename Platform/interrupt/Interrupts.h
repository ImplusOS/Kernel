#pragma once

#include <stdint.h>

#include "Platform/acpi/ACPI.h"

#define VECTOR_IRQ_BASE 32u
#define VECTOR_TIMER    (VECTOR_IRQ_BASE + 0u)
#define VECTOR_SPURIOUS 0xFFu

void platform_interrupts_init_legacy_pic(void);
int  platform_interrupts_configure(const acpi_info_t *info);
void platform_interrupts_eoi(uint16_t vector);
int  platform_interrupts_using_lapic(void);
void platform_interrupts_route_pit(void);
void platform_interrupts_mask_pit(void);