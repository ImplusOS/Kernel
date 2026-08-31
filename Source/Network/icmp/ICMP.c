#include "ICMP.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Network/ipv4.h"
#include "Network/network_utils.h"
#include "Debug/serial/Serial.h"

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  code;
    uint16_t checksum;
    uint16_t id;
    uint16_t sequence;
} icmp_header_t;

#define ICMP_MAX_PAYLOAD 1472u

static uint16_t icmp_checksum(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0;
    uint16_t i = 0;
    while ((uint16_t)(i + 1u) < len) {
        uint16_t word = (uint16_t)(((uint16_t)data[i] << 8) | (uint16_t)data[i + 1u]);
        sum += (uint32_t)word;
        i = (uint16_t)(i + 2u);
    }
    if ((len & 1u) != 0u) {
        sum += (uint32_t)((uint16_t)data[len - 1u] << 8);
    }
    while ((sum >> 16) != 0u) {
        sum = (sum & 0xFFFFu) + (sum >> 16);
    }
    return (uint16_t)~(sum & 0xFFFFu);
}

static void icmp_on_ipv4(uint32_t src_ip, uint32_t dst_ip,
                         const uint8_t *payload, uint16_t payload_len)
{
    (void)dst_ip;
    if (payload == NULL || payload_len < sizeof(icmp_header_t)) return;

    const icmp_header_t *hdr = (const icmp_header_t *)payload;

    if (hdr->type == ICMP_TYPE_ECHO_REQUEST && hdr->code == 0u) {
        uint16_t data_len = (uint16_t)(payload_len - sizeof(icmp_header_t));
        uint8_t reply[sizeof(icmp_header_t) + ICMP_MAX_PAYLOAD];
        if (data_len > ICMP_MAX_PAYLOAD) data_len = ICMP_MAX_PAYLOAD;

        icmp_header_t *rhdr = (icmp_header_t *)reply;
        rhdr->type = ICMP_TYPE_ECHO_REPLY;
        rhdr->code = 0;
        rhdr->checksum = 0;
        rhdr->id = hdr->id;
        rhdr->sequence = hdr->sequence;

        if (data_len > 0) {
            memcpy(reply + sizeof(icmp_header_t),
                   payload + sizeof(icmp_header_t), data_len);
        }

        uint16_t total = (uint16_t)(sizeof(icmp_header_t) + data_len);
        rhdr->checksum = icmp_checksum(reply, total);

        ipv4_send(src_ip, ICMP_PROTOCOL_NUMBER, reply, total);
    }
}

void icmp_init(void)
{
    (void)ipv4_register_protocol(ICMP_PROTOCOL_NUMBER, icmp_on_ipv4);
}

bool icmp_send_echo_request(uint32_t dst_ip, uint16_t id, uint16_t seq,
                            const void *data, uint16_t data_len)
{
    uint8_t packet[sizeof(icmp_header_t) + ICMP_MAX_PAYLOAD];
    if (data_len > ICMP_MAX_PAYLOAD) data_len = ICMP_MAX_PAYLOAD;

    icmp_header_t *hdr = (icmp_header_t *)packet;
    hdr->type = ICMP_TYPE_ECHO_REQUEST;
    hdr->code = 0;
    hdr->checksum = 0;
    hdr->id = net_htons(id);
    hdr->sequence = net_htons(seq);

    if (data_len > 0 && data != NULL) {
        memcpy(packet + sizeof(icmp_header_t), data, data_len);
    }

    uint16_t total = (uint16_t)(sizeof(icmp_header_t) + data_len);
    hdr->checksum = icmp_checksum(packet, total);

    return ipv4_send(dst_ip, ICMP_PROTOCOL_NUMBER, packet, total);
}
