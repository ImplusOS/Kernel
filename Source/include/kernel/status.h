#pragma once

#include <stdint.h>

typedef int64_t os_status_t;

enum {
    OS_STATUS_OK            = 0,
    OS_STATUS_INVALID_ARG   = -22,
    OS_STATUS_NOT_FOUND     = -2,
    OS_STATUS_ACCESS_DENIED = -13,
    OS_STATUS_LIMIT_REACHED = -24,
    OS_STATUS_IO_ERROR      = -5,
    OS_STATUS_FAULT         = -14,
    OS_STATUS_NOT_SUPPORTED = -95,
    OS_STATUS_WOULD_BLOCK   = -11,
    OS_STATUS_BROKEN_PIPE   = -32,   /* EPIPE: write with no reader */
    OS_STATUS_INTERNAL      = -255,
    OS_STATUS_INIT_FAILED   = -256
};

static inline int os_status_is_error(os_status_t status)
{
    return status < 0;
}

static inline os_status_t os_status_from_i32(int32_t value)
{
    return (os_status_t)value;
}

static inline uint64_t os_status_to_u64(os_status_t status)
{
    return (uint64_t)status;
}

static inline uint64_t os_status_abs_u64(os_status_t status)
{
    if (status < 0) {
        return (uint64_t)(-status);
    }
    return (uint64_t)status;
}

static inline const char *os_status_to_string(os_status_t status)
{
    switch (status) {
        case OS_STATUS_OK:
            return "OS_STATUS_OK";
        case OS_STATUS_INVALID_ARG:
            return "OS_STATUS_INVALID_ARG";
        case OS_STATUS_NOT_FOUND:
            return "OS_STATUS_NOT_FOUND";
        case OS_STATUS_ACCESS_DENIED:
            return "OS_STATUS_ACCESS_DENIED";
        case OS_STATUS_LIMIT_REACHED:
            return "OS_STATUS_LIMIT_REACHED";
        case OS_STATUS_IO_ERROR:
            return "OS_STATUS_IO_ERROR";
        case OS_STATUS_FAULT:
            return "OS_STATUS_FAULT";
        case OS_STATUS_NOT_SUPPORTED:
            return "OS_STATUS_NOT_SUPPORTED";
        case OS_STATUS_WOULD_BLOCK:
            return "OS_STATUS_WOULD_BLOCK";
        case OS_STATUS_BROKEN_PIPE:
            return "OS_STATUS_BROKEN_PIPE";
        case OS_STATUS_INTERNAL:
            return "OS_STATUS_INTERNAL";
        case OS_STATUS_INIT_FAILED:
            return "OS_STATUS_INIT_FAILED";
        default:
            return "OS_STATUS_UNKNOWN";
    }
}

static inline int32_t os_status_to_errno(os_status_t status)
{
    if (status >= 0) {
        return 0;
    }

    switch (status) {
        case OS_STATUS_INVALID_ARG:
            return 22;
        case OS_STATUS_NOT_FOUND:
            return 2;
        case OS_STATUS_ACCESS_DENIED:
            return 13;
        case OS_STATUS_LIMIT_REACHED:
            return 24;
        case OS_STATUS_IO_ERROR:
            return 5;
        case OS_STATUS_FAULT:
            return 14;
        case OS_STATUS_NOT_SUPPORTED:
            return 95;
        case OS_STATUS_WOULD_BLOCK:
            return 11;
        case OS_STATUS_BROKEN_PIPE:
            return 32;
        case OS_STATUS_INTERNAL:
            return 255;
        case OS_STATUS_INIT_FAILED:
            return 256;
        default: {
            uint64_t magnitude = os_status_abs_u64(status);
            if (magnitude > (uint64_t)INT32_MAX) {
                return 255;
            }
            return (int32_t)magnitude;
        }
    }
}
