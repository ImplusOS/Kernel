#pragma once

#include <stdbool.h>
#include <stdint.h>

bool hpet_init(void);
bool hpet_is_available(void);
uint64_t hpet_monotonic_ns(void);
