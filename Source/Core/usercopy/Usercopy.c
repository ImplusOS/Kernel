#include "Usercopy.h"

#include "Core/process/ProcessManager.h"
#include <stddef.h>
#include <string.h>

uint64_t copy_from_user(void *kernel_dst, const void *user_src, uint64_t bytes)
{
    if (bytes == 0) {
        return 0;
    }
    if (kernel_dst == 0 || user_src == 0 ||
        !process_user_buffer_is_valid(user_src, bytes)) {
        return bytes;
    }

    memcpy(kernel_dst, user_src, (size_t)bytes);
    return 0;
}

uint64_t copy_to_user(void *user_dst, const void *kernel_src, uint64_t bytes)
{
    if (bytes == 0) {
        return 0;
    }
    if (user_dst == 0 || kernel_src == 0 ||
        !process_user_buffer_is_valid(user_dst, bytes)) {
        return bytes;
    }

    memcpy(user_dst, kernel_src, (size_t)bytes);
    return 0;
}

uint64_t copy_from_user_trusted(void *kernel_dst, const void *user_src, uint64_t bytes)
{
    if (bytes == 0) {
        return 0;
    }
    if (kernel_dst == 0 || user_src == 0) {
        return bytes;
    }

    memcpy(kernel_dst, user_src, (size_t)bytes);
    return 0;
}

uint64_t copy_to_user_trusted(void *user_dst, const void *kernel_src, uint64_t bytes)
{
    if (bytes == 0) {
        return 0;
    }
    if (user_dst == 0 || kernel_src == 0) {
        return bytes;
    }

    memcpy(user_dst, kernel_src, (size_t)bytes);
    return 0;
}

int copy_user_cstring_s(char *dst, uint64_t dst_size, const char *user_src)
{
    if (dst == 0 || user_src == 0 || dst_size < 2u) {
        return -1;
    }

    if (process_user_buffer_is_valid(user_src, dst_size - 1u)) {
        uint64_t i = 0;
        while (i + 1u < dst_size) {
            char ch = user_src[i];
            dst[i] = ch;
            if (ch == '\0') {
                return 0;
            }
            ++i;
        }
        dst[0] = '\0';
        return -1;
    }

    uint64_t copied = 0;
    while (copied + 1u < dst_size) {
        uint64_t addr = (uint64_t)(uintptr_t)(user_src + copied);
        uint64_t page_end = (addr & ~(uint64_t)0xFFFu) + 0x1000u;
        uint64_t chunk = page_end - addr;
        uint64_t remaining = dst_size - 1u - copied;
        if (chunk > remaining) {
            chunk = remaining;
        }

        if (copy_from_user(dst + copied, user_src + copied, chunk) != 0u) {
            uint64_t fallback_limit = copied + chunk;
            if (fallback_limit > dst_size - 1u) {
                fallback_limit = dst_size - 1u;
            }
            for (uint64_t i = copied; i < fallback_limit; ++i) {
                char ch = '\0';
                if (copy_from_user(&ch, user_src + i, 1u) != 0u) {
                    dst[0] = '\0';
                    return -1;
                }
                dst[i] = ch;
                if (ch == '\0') {
                    return 0;
                }
            }
            dst[0] = '\0';
            return -1;
        }

        for (uint64_t i = copied; i < copied + chunk; ++i) {
            if (dst[i] == '\0') {
                return 0;
            }
        }
        copied += chunk;
    }

    dst[0] = '\0';
    return -1;
}
