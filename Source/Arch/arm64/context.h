#pragma once

#include <stdint.h>

typedef struct {
    uint64_t x19;
    uint64_t x20;
    uint64_t x21;
    uint64_t x22;
    uint64_t x23;
    uint64_t x24;
    uint64_t x25;
    uint64_t x26;
    uint64_t x27;
    uint64_t x28;
    uint64_t x29;
    uint64_t x30;
    uint64_t sp_el0;
    uint64_t sp_el1;
    uint64_t elr_el1;
    uint64_t spsr_el1;
} context_t;

