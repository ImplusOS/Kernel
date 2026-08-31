#pragma once

#include <stdint.h>

uint64_t copy_from_user(void *kernel_dst, const void *user_src, uint64_t bytes);
uint64_t copy_to_user(void *user_dst, const void *kernel_src, uint64_t bytes);
uint64_t copy_from_user_trusted(void *kernel_dst, const void *user_src, uint64_t bytes);
uint64_t copy_to_user_trusted(void *user_dst, const void *kernel_src, uint64_t bytes);
int copy_user_cstring_s(char *dst, uint64_t dst_size, const char *user_src);
