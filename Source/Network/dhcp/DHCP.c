#include "DHCP.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Network/ipv4.h"
#include "Network/udp/UDP.h"
#include "Network/network_utils.h"
#include "Debug/serial/Serial.h"
#include "Core/timer/Timer.h"

#define DHCP_SERVER_PORT 67u
#define DHCP_CLIENT_PORT 68u

#define DHCP_MAGIC_COOKIE 0x63825363u

#define DHCP_MSG_DISCOVER 1u
#define DHCP_MSG_OFFER    2u
#define DHCP_MSG_REQUEST  3u
#define DHCP_MSG_ACK      5u

#define DHCP_OPTION_MSG_TYPE       53u
#define DHCP_OPTION_SUBNET_MASK    1u
#define DHCP_OPTION_GATEWAY        3u
#define DHCP_OPTION_DNS            6u
#define DHCP_OPTION_REQUESTED_IP   50u
#define DHCP_OPTION_SERVER_ID      54u
#define DHCP_OPTION_END            255u

typedef struct __attribute__((packed)) {
    uint8_t  op;
    uint8_t  htype;
    uint8_t  hlen;
    uint8_t  hops;
    uint32_t xid;
    uint16_t secs;
    uint16_t flags;
    uint32_t ciaddr;
    uint32_t yiaddr;
    uint32_t siaddr;
    uint32_t giaddr;
    uint8_t  chaddr[16];
    uint8_t  sname[64];
    uint8_t  file[128];
    uint32_t magic_cookie;
} dhcp_header_t;

static uint32_t g_dhcp_assigned_ip = 0;
static uint32_t g_dhcp_gateway = 0;
static uint32_t g_dhcp_subnet_mask = 0;
static uint32_t g_dhcp_dns = 0;
static uint32_t g_dhcp_xid = 0x12345678u;
static uint8_t  g_dhcp_state = 0;

static inline uint32_t dhcp_read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static inline void dhcp_write_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)((v >> 24) & 0xFFu);
    p[1] = (uint8_t)((v >> 16) & 0xFFu);
    p[2] = (uint8_t)((v >> 8)  & 0xFFu);
    p[3] = (uint8_t)(v & 0xFFu);
}

static void dhcp_parse_options(const uint8_t *options, uint16_t len,
                               uint32_t *out_msg_type, uint32_t *out_server_id)
{
    uint16_t i = 0;
    *out_msg_type = 0;
    *out_server_id = 0;
    while (i < len) {
        uint8_t opt = options[i++];
        if (opt == DHCP_OPTION_END) break;
        if (opt == 0) continue;
        if (i >= len) break;
        uint8_t opt_len = options[i++];
        if ((uint16_t)(i + opt_len) > len) break;

        if (opt == DHCP_OPTION_MSG_TYPE && opt_len == 1) {
            *out_msg_type = options[i];
        } else if (opt == DHCP_OPTION_SUBNET_MASK && opt_len == 4) {
            g_dhcp_subnet_mask = dhcp_read_be32(&options[i]);
        } else if (opt == DHCP_OPTION_GATEWAY && opt_len >= 4) {
            g_dhcp_gateway = dhcp_read_be32(&options[i]);
        } else if (opt == DHCP_OPTION_DNS && opt_len >= 4) {
            g_dhcp_dns = dhcp_read_be32(&options[i]);
        } else if (opt == DHCP_OPTION_SERVER_ID && opt_len == 4) {
            *out_server_id = dhcp_read_be32(&options[i]);
        }
        i += opt_len;
    }
}

static void dhcp_on_udp(uint32_t src_ip, uint16_t src_port,
                        uint32_t dst_ip, uint16_t dst_port,
                        const uint8_t *payload, uint16_t payload_len)
{
    (void)src_ip;
    (void)src_port;
    (void)dst_ip;
    if (dst_port != DHCP_CLIENT_PORT) return;
    if (payload_len < sizeof(dhcp_header_t)) return;

    const dhcp_header_t *hdr = (const dhcp_header_t *)payload;
    if (hdr->op != 2) return;
    if (hdr->xid != net_htonl(g_dhcp_xid)) return;

    uint16_t options_offset = sizeof(dhcp_header_t);
    uint16_t options_len = (payload_len > options_offset) ?
                           (uint16_t)(payload_len - options_offset) : 0;
    uint32_t msg_type = 0, server_id = 0;
    dhcp_parse_options(payload + options_offset, options_len, &msg_type, &server_id);

    if (msg_type == DHCP_MSG_OFFER && g_dhcp_state == 1) {
        g_dhcp_assigned_ip = net_ntohl(hdr->yiaddr);
        
        uint8_t buf[sizeof(dhcp_header_t) + 32];
        memset(buf, 0, sizeof(buf));
        dhcp_header_t *req = (dhcp_header_t *)buf;
        req->op = 1;
        req->htype = 1;
        req->hlen = 6;
        req->xid = net_htonl(g_dhcp_xid);
        req->flags = net_htons(0x8000u);
        req->magic_cookie = net_htonl(DHCP_MAGIC_COOKIE);

        uint8_t *opt = buf + sizeof(dhcp_header_t);
        *opt++ = DHCP_OPTION_MSG_TYPE; *opt++ = 1; *opt++ = DHCP_MSG_REQUEST;
        *opt++ = DHCP_OPTION_REQUESTED_IP; *opt++ = 4;
        dhcp_write_be32(opt, g_dhcp_assigned_ip); opt += 4;
        if (server_id != 0) {
            *opt++ = DHCP_OPTION_SERVER_ID; *opt++ = 4;
            dhcp_write_be32(opt, server_id); opt += 4;
        }
        *opt++ = DHCP_OPTION_END;

        uint16_t total = (uint16_t)(opt - buf);
        udp_send(0xFFFFFFFFu, DHCP_CLIENT_PORT, DHCP_SERVER_PORT, buf, total);
        g_dhcp_state = 2;
    } else if (msg_type == DHCP_MSG_ACK && g_dhcp_state == 2) {
        g_dhcp_assigned_ip = net_ntohl(hdr->yiaddr);
        g_dhcp_state = 3;

        ipv4_init(g_dhcp_assigned_ip, g_dhcp_subnet_mask, g_dhcp_gateway);
    }
}

void dhcp_init(void)
{
    g_dhcp_xid += (uint32_t)(timer_ticks() & 0xFFFFu);
    udp_bind(DHCP_CLIENT_PORT, dhcp_on_udp);
}

bool dhcp_discover(void)
{
    uint8_t buf[sizeof(dhcp_header_t) + 16];
    memset(buf, 0, sizeof(buf));
    dhcp_header_t *req = (dhcp_header_t *)buf;
    req->op = 1;
    req->htype = 1;
    req->hlen = 6;
    req->xid = net_htonl(g_dhcp_xid);
    req->flags = net_htons(0x8000u);
    req->magic_cookie = net_htonl(DHCP_MAGIC_COOKIE);

    uint8_t *opt = buf + sizeof(dhcp_header_t);
    *opt++ = DHCP_OPTION_MSG_TYPE; *opt++ = 1; *opt++ = DHCP_MSG_DISCOVER;
    *opt++ = DHCP_OPTION_END;

    uint16_t total = (uint16_t)(opt - buf);
    g_dhcp_state = 1;
    return udp_send(0xFFFFFFFFu, DHCP_CLIENT_PORT, DHCP_SERVER_PORT, buf, total);
}

uint32_t dhcp_get_assigned_ip(void) { return g_dhcp_assigned_ip; }
uint32_t dhcp_get_gateway(void) { return g_dhcp_gateway; }
uint32_t dhcp_get_subnet_mask(void) { return g_dhcp_subnet_mask; }
uint32_t dhcp_get_dns_server(void) { return g_dhcp_dns; }
