#include "DHCP.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Network/ipv4.h"
#include "Network/udp/UDP.h"
#include "Network/ethernet/Ethernet.h"
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
#define DHCP_MSG_NAK      6u

#define DHCP_OPTION_SUBNET_MASK    1u
#define DHCP_OPTION_GATEWAY        3u
#define DHCP_OPTION_DNS            6u
#define DHCP_OPTION_DOMAIN_NAME    15u
#define DHCP_OPTION_BROADCAST      28u
#define DHCP_OPTION_REQUESTED_IP   50u
#define DHCP_OPTION_MSG_TYPE       53u
#define DHCP_OPTION_SERVER_ID      54u
#define DHCP_OPTION_PARAM_REQUEST  55u
#define DHCP_OPTION_END            255u

/* BOOTP's fixed message size (RFC 951: 236-byte header + 64-byte vendor
 * area). RFC 1542 2.1 asks clients to send at least this much; servers and
 * relays built around struct bootp drop anything shorter. */
#define DHCP_MIN_MESSAGE 300u

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

static uint32_t g_dhcp_assigned_ip = 0;  /* set on ACK only */
static uint32_t g_dhcp_offered_ip = 0;   /* yiaddr of the OFFER being requested */
static uint32_t g_dhcp_gateway = 0;
static uint32_t g_dhcp_subnet_mask = 0;
static uint32_t g_dhcp_dns = 0;
static uint32_t g_dhcp_xid = 0x12345678u;
static uint8_t  g_dhcp_state = 0;        /* 0 idle, 1 DISCOVER sent, 2 REQUEST sent, 3 bound */

/* What every DISCOVER/REQUEST asks the server for. A server only has to
 * send the options a client lists in option 55, and macOS's bootpd -- the
 * DHCP server behind vmnet "shared" networking (UTM, Virtualization.framework)
 * -- sends nothing else: without this list its OFFER/ACK carried only the
 * message type, server identifier and lease time. The lease then came with
 * no netmask, router or DNS server, /etc/resolv.conf fell back to SLIRP's
 * 10.0.2.3, and every name lookup (Chromium: ERR_NAME_NOT_RESOLVED) failed.
 * QEMU's SLIRP and most routers send these unasked, which hid the bug. */
static const uint8_t g_dhcp_param_request[] = {
    DHCP_OPTION_SUBNET_MASK, DHCP_OPTION_GATEWAY, DHCP_OPTION_DNS,
    DHCP_OPTION_DOMAIN_NAME, DHCP_OPTION_BROADCAST,
};

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

/* Lay out a client message in `buf` (at least DHCP_MIN_MESSAGE bytes) and
 * return its length. */
static uint16_t dhcp_build(uint8_t *buf, uint8_t msg_type,
                           uint32_t requested_ip, uint32_t server_id)
{
    memset(buf, 0, DHCP_MIN_MESSAGE);
    dhcp_header_t *req = (dhcp_header_t *)buf;
    req->op = 1;
    req->htype = 1;
    req->hlen = 6;
    /* chaddr identifies the client to the server: a real DHCP server
     * (a home router) keys the lease on it, and some will not answer a
     * client whose hardware address is all zeros. */
    ethernet_get_local_mac(req->chaddr);
    req->xid = net_htonl(g_dhcp_xid);
    req->flags = net_htons(0x8000u); /* broadcast the reply: we have no address */
    req->magic_cookie = net_htonl(DHCP_MAGIC_COOKIE);

    uint8_t *opt = buf + sizeof(dhcp_header_t);
    *opt++ = DHCP_OPTION_MSG_TYPE; *opt++ = 1; *opt++ = msg_type;
    *opt++ = DHCP_OPTION_PARAM_REQUEST;
    *opt++ = (uint8_t)sizeof(g_dhcp_param_request);
    memcpy(opt, g_dhcp_param_request, sizeof(g_dhcp_param_request));
    opt += sizeof(g_dhcp_param_request);
    if (requested_ip != 0u) {
        *opt++ = DHCP_OPTION_REQUESTED_IP; *opt++ = 4;
        dhcp_write_be32(opt, requested_ip); opt += 4;
    }
    if (server_id != 0u) {
        *opt++ = DHCP_OPTION_SERVER_ID; *opt++ = 4;
        dhcp_write_be32(opt, server_id); opt += 4;
    }
    *opt++ = DHCP_OPTION_END;

    uint16_t len = (uint16_t)(opt - buf);
    return len < DHCP_MIN_MESSAGE ? (uint16_t)DHCP_MIN_MESSAGE : len;
}

/* Until a lease is bound the client has no address of its own, so the
 * request goes out from 0.0.0.0 (RFC 2131 4.1) -- not from the stack's
 * static fallback (OS_CONFIG_NET_IPV4_ADDR, QEMU SLIRP's 10.0.2.15), which
 * means nothing on any other network. */
static bool dhcp_send(const uint8_t *buf, uint16_t len)
{
    return udp_send_from(0u, 0xFFFFFFFFu, DHCP_CLIENT_PORT, DHCP_SERVER_PORT,
                         buf, len);
}

static void dhcp_log_ip(const char *label, uint32_t ip)
{
    serial_write_string(label);
    for (int shift = 24; shift >= 0; shift -= 8) {
        serial_write_dec16((uint16_t)((ip >> (uint32_t)shift) & 0xFFu));
        if (shift != 0) serial_write_char('.');
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
        g_dhcp_offered_ip = net_ntohl(hdr->yiaddr);
        uint8_t buf[DHCP_MIN_MESSAGE + 64];
        uint16_t total = dhcp_build(buf, DHCP_MSG_REQUEST, g_dhcp_offered_ip,
                                    server_id);
        (void)dhcp_send(buf, total);
        g_dhcp_state = 2;
    } else if (msg_type == DHCP_MSG_ACK && g_dhcp_state == 2) {
        uint32_t ip = net_ntohl(hdr->yiaddr);
        if (g_dhcp_subnet_mask == 0u) {
            /* No netmask in the ACK: take the address class's, rather than
             * 0.0.0.0, which would make every destination look on-link. */
            g_dhcp_subnet_mask = (ip >> 31) == 0u ? 0xFF000000u :
                                 (ip >> 30) == 2u ? 0xFFFF0000u : 0xFFFFFF00u;
        }
        if (g_dhcp_dns == 0u && g_dhcp_gateway != 0u) {
            /* No DNS server named: the router usually forwards DNS (home
             * routers, macOS Internet Sharing/vmnet, SLIRP). Better than
             * SLIRP's 10.0.2.3 on a network that is not SLIRP. */
            g_dhcp_dns = g_dhcp_gateway;
        }
        g_dhcp_assigned_ip = ip;
        g_dhcp_state = 3;

        ipv4_set_address(g_dhcp_assigned_ip, g_dhcp_subnet_mask, g_dhcp_gateway);

        dhcp_log_ip("[DHCP] lease ", g_dhcp_assigned_ip);
        dhcp_log_ip(" mask ", g_dhcp_subnet_mask);
        dhcp_log_ip(" gw ", g_dhcp_gateway);
        dhcp_log_ip(" dns ", g_dhcp_dns);
        dhcp_log_ip(" server ", server_id);
        serial_write_char('\n');
    } else if (msg_type == DHCP_MSG_NAK && g_dhcp_state == 2) {
        /* Start over; network_stack_on_timer_tick() sends the next DISCOVER
         * while no lease is bound. */
        g_dhcp_offered_ip = 0u;
        g_dhcp_state = 0;
        serial_write_string("[DHCP] NAK\n");
    }
}

void dhcp_init(void)
{
    g_dhcp_xid += (uint32_t)(timer_ticks() & 0xFFFFu);
    g_dhcp_assigned_ip = 0u;
    g_dhcp_offered_ip = 0u;
    g_dhcp_state = 0;
    udp_bind(DHCP_CLIENT_PORT, dhcp_on_udp);
}

bool dhcp_discover(void)
{
    /* A fresh exchange: forget the options of any earlier OFFER. */
    g_dhcp_subnet_mask = 0u;
    g_dhcp_gateway = 0u;
    g_dhcp_dns = 0u;
    g_dhcp_offered_ip = 0u;

    uint8_t buf[DHCP_MIN_MESSAGE + 64];
    uint16_t total = dhcp_build(buf, DHCP_MSG_DISCOVER, 0u, 0u);
    g_dhcp_state = 1;
    return dhcp_send(buf, total);
}

uint32_t dhcp_get_assigned_ip(void) { return g_dhcp_assigned_ip; }
uint32_t dhcp_get_gateway(void) { return g_dhcp_gateway; }
uint32_t dhcp_get_subnet_mask(void) { return g_dhcp_subnet_mask; }
uint32_t dhcp_get_dns_server(void) { return g_dhcp_dns; }
