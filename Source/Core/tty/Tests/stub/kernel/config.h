#pragma once
/* Host-harness stand-in for Kernel/Source/include/kernel/config.h. Only the
 * knobs Pty.c actually reads. */
#define OS_CONFIG_PROCESS_MAX_COUNT 256
/* The bring-up trace is compiled in, so the harness exercises those paths too;
 * run.sh sends it to stderr. */
#define OS_CONFIG_FOREIGN_TRACE 1
