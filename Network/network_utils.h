#pragma once

#include <stddef.h>
#include <stdint.h>

static inline uint16_t net_bswap16(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

static inline uint32_t net_bswap32(uint32_t v)
{
    return ((v & 0x000000FFu) << 24) |
           ((v & 0x0000FF00u) << 8) |
           ((v & 0x00FF0000u) >> 8) |
           ((v & 0xFF000000u) >> 24);
}

static inline uint16_t net_htons(uint16_t host)
{
    return net_bswap16(host);
}

static inline uint16_t net_ntohs(uint16_t net)
{
    return net_bswap16(net);
}

static inline uint32_t net_htonl(uint32_t host)
{
    return net_bswap32(host);
}

static inline uint32_t net_ntohl(uint32_t net)
{
    return net_bswap32(net);
}

static inline uint32_t net_ipv4_make(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    return ((uint32_t)a << 24) |
           ((uint32_t)b << 16) |
           ((uint32_t)c << 8) |
           (uint32_t)d;
}

static inline uint16_t net_checksum16(const void *data, size_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t sum = 0;

    for (size_t i = 0; i + 1 < len; i += 2) {
        uint16_t word = (uint16_t)(((uint16_t)bytes[i] << 8) | (uint16_t)bytes[i + 1]);
        sum += (uint32_t)word;
        if (sum > 0xFFFFu) {
            sum = (sum & 0xFFFFu) + 1u;
        }
    }

    if ((len & 1u) != 0u) {
        uint16_t word = (uint16_t)((uint16_t)bytes[len - 1] << 8);
        sum += (uint32_t)word;
        if (sum > 0xFFFFu) {
            sum = (sum & 0xFFFFu) + 1u;
        }
    }

    return (uint16_t)~(sum & 0xFFFFu);
}
