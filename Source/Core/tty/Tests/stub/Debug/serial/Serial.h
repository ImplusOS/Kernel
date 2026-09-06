#pragma once
#include <stdint.h>
#include <stdio.h>
/* The harness sends the kernel's bring-up trace to stderr so a failing case
 * shows it without polluting the pass/fail lines on stdout. */
static inline void serial_write_string(const char *s) { fputs(s ? s : "", stderr); }
static inline void serial_write_char(char c) { fputc(c, stderr); }
static inline void serial_write_uint32(uint32_t v) { fprintf(stderr, "0x%08X", v); }
static inline void serial_write_uint64(uint64_t v) { fprintf(stderr, "0x%016llX", (unsigned long long)v); }
