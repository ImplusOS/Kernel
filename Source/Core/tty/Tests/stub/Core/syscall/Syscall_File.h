#pragma once
#include <stdint.h>
/* Only the one entry point Pty.c calls (TIOCGPTPEER). */
int32_t syscall_file_open(const char *path, uint64_t flags);
