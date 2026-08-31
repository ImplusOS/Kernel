#include "NetworkBuiltinDrivers.h"
#include "DriverManager.h"
#include "kernel/config.h"
#include "kernel/interfaces/device.h"

#include "Network/ethernet/Ethernet.h"
#include "Network/arp/ARP.h"
#include "Network/ipv4.h"
#include "Network/icmp/ICMP.h"
#include "Network/udp/UDP.h"
#include "Network/tcp/TCP.h"
#include "Network/dhcp/DHCP.h"

/*
 * See NetworkBuiltinDrivers.h for the hybrid design this implements
 * (Docs/Others/TODO_OS_Refactor.md phase P5, 9.2). Each layer gets its own
 * vtable type populated with
 * that layer's *actual* public entry points, so a lookup through
 * DeviceRegistry returns something genuinely callable, not just a
 * name/description pair.
 *
 * deps[] on each entry is real, accurate dependency information (derived
 * from grepping Kernel/Network's actual #include graph, not guessed). It is
 * now also load-bearing: network_builtin_drivers_init_all() walks it to
 * order the boot-time init calls, so Kernel/Network/network_main.c's
 * network_stack_init() no longer hard-codes the Ethernet -> ARP -> IPv4 ->
 * {ICMP, UDP, TCP} -> DHCP sequence itself. The kernel's network subsystem
 * asks this table to bring the layers up; it does not know their init
 * functions.
 */

typedef struct {
    bool     (*init)(void);
    bool     (*is_ready)(void);
    bool     (*send)(uint16_t ether_type, const uint8_t dst_mac[ETHERNET_ADDR_LEN],
                     const void *payload, uint16_t payload_len);
    bool     (*register_handler)(uint16_t ether_type, ethernet_type_handler_t handler);
    const char *deps[4];
} ethernet_builtin_driver_t;

static bool ethernet_init_void(void) { return ethernet_init(); }

static const ethernet_builtin_driver_t g_ethernet_driver = {
    .init = ethernet_init_void,
    .is_ready = ethernet_is_ready,
    .send = ethernet_send,
    .register_handler = ethernet_register_handler,
    .deps = { NULL },
};

typedef struct {
    bool     (*lookup)(uint32_t ipv4_addr, uint8_t mac_out[6]);
    bool     (*resolve)(uint32_t ipv4_addr, uint8_t mac_out[6], uint32_t timeout_ms);
    bool     (*send_request)(uint32_t target_ipv4_addr);
    const char *deps[4];
} arp_builtin_driver_t;

static const arp_builtin_driver_t g_arp_driver = {
    .lookup = arp_lookup,
    .resolve = arp_resolve,
    .send_request = arp_send_request,
    .deps = { "Ethernet_Driver", NULL },
};

typedef struct {
    uint32_t (*local_address)(void);
    bool     (*send)(uint32_t dst_ipv4_addr, uint8_t protocol,
                     const void *payload, uint16_t payload_len);
    bool     (*register_protocol)(uint8_t protocol, ipv4_protocol_handler_t handler);
    const char *deps[4];
} ipv4_builtin_driver_t;

static const ipv4_builtin_driver_t g_ipv4_driver = {
    .local_address = ipv4_local_address,
    .send = ipv4_send,
    .register_protocol = ipv4_register_protocol,
    .deps = { "Ethernet_Driver", "ARP_Driver", NULL },
};

typedef struct {
    bool (*send_echo_request)(uint32_t dst_ip, uint16_t id, uint16_t seq,
                              const void *data, uint16_t data_len);
    const char *deps[4];
} icmp_builtin_driver_t;

static const icmp_builtin_driver_t g_icmp_driver = {
    .send_echo_request = icmp_send_echo_request,
    .deps = { "IPv4_Driver", NULL },
};

typedef struct {
    bool    (*bind)(uint16_t port, udp_rx_handler_t handler);
    void    (*unbind)(uint16_t port);
    bool    (*send)(uint32_t dst_ipv4_addr, uint16_t src_port, uint16_t dst_port,
                    const void *payload, uint16_t payload_len);
    const char *deps[4];
} udp_builtin_driver_t;

static const udp_builtin_driver_t g_udp_driver = {
    .bind = udp_bind,
    .unbind = udp_unbind,
    .send = udp_send,
    .deps = { "IPv4_Driver", NULL },
};

typedef struct {
    int32_t  (*connect)(uint32_t remote_ip, uint16_t remote_port, uint16_t local_port);
    int32_t  (*listen)(uint16_t port);
    int32_t  (*send)(int32_t conn_id, const void *data, uint16_t len);
    int32_t  (*recv)(int32_t conn_id, void *buf, uint16_t buf_len);
    int32_t  (*close)(int32_t conn_id);
    const char *deps[4];
} tcp_builtin_driver_t;

static const tcp_builtin_driver_t g_tcp_driver = {
    .connect = tcp_connect,
    .listen = tcp_listen,
    .send = tcp_send,
    .recv = tcp_recv,
    .close = tcp_close,
    .deps = { "IPv4_Driver", NULL },
};

typedef struct {
    bool     (*discover)(void);
    uint32_t (*get_assigned_ip)(void);
    uint32_t (*get_gateway)(void);
    uint32_t (*get_subnet_mask)(void);
    const char *deps[4];
} dhcp_builtin_driver_t;

static const dhcp_builtin_driver_t g_dhcp_driver = {
    .discover = dhcp_discover,
    .get_assigned_ip = dhcp_get_assigned_ip,
    .get_gateway = dhcp_get_gateway,
    .get_subnet_mask = dhcp_get_subnet_mask,
    .deps = { "IPv4_Driver", "UDP_Driver", NULL },
};

/*
 * ---- one table, driven both for DeviceRegistry registration and for the
 * boot-time init sequence ----
 *
 * network_stack_init() (Kernel/Network/network_main.c) no longer contains a
 * hand-written "ethernet_init(); arp_init(...); ipv4_init(...); ..." list.
 * It calls network_builtin_drivers_init_all(), which walks g_net_layers[]
 * honouring each row's deps[] -- so the Ethernet -> ARP -> IPv4 ->
 * {ICMP,UDP,TCP} -> DHCP order is derived from the recorded dependencies
 * rather than open-coded in the kernel's network subsystem. The parameters
 * arp_init()/ipv4_init() need (the static OS_CONFIG_NET_IPV4_* address)
 * are supplied by the void adapters below, keeping that policy next to the
 * layer vtables instead of in network_main.c.
 */

#ifndef OS_CONFIG_NET_IPV4_ADDR
#define OS_CONFIG_NET_IPV4_ADDR 0x0A00020FUL
#endif
#ifndef OS_CONFIG_NET_IPV4_MASK
#define OS_CONFIG_NET_IPV4_MASK 0xFFFFFF00UL
#endif
#ifndef OS_CONFIG_NET_IPV4_GATEWAY
#define OS_CONFIG_NET_IPV4_GATEWAY 0x0A000202UL
#endif

static bool ethernet_init_adapter(void) { return ethernet_init(); }
static bool arp_init_adapter(void)
{
    arp_init((uint32_t)OS_CONFIG_NET_IPV4_ADDR,
             (uint32_t)OS_CONFIG_NET_IPV4_MASK,
             (uint32_t)OS_CONFIG_NET_IPV4_GATEWAY);
    return true;
}
static bool ipv4_init_adapter(void)
{
    ipv4_init((uint32_t)OS_CONFIG_NET_IPV4_ADDR,
              (uint32_t)OS_CONFIG_NET_IPV4_MASK,
              (uint32_t)OS_CONFIG_NET_IPV4_GATEWAY);
    return true;
}
static bool udp_init_adapter(void)  { udp_init();  return true; }
static bool tcp_init_adapter(void)  { tcp_init();  return true; }
static bool icmp_init_adapter(void) { icmp_init(); return true; }
static bool dhcp_init_adapter(void) { dhcp_init(); return true; }

typedef struct {
    const char  *name;
    const void  *ops;
    bool       (*init)(void);
    const char  *deps[4];
} net_layer_t;

static const net_layer_t g_net_layers[] = {
    { "Ethernet_Driver", &g_ethernet_driver, ethernet_init_adapter, { NULL } },
    { "ARP_Driver",      &g_arp_driver,      arp_init_adapter,      { "Ethernet_Driver", NULL } },
    { "IPv4_Driver",     &g_ipv4_driver,     ipv4_init_adapter,     { "Ethernet_Driver", "ARP_Driver", NULL } },
    { "ICMP_Driver",     &g_icmp_driver,     icmp_init_adapter,     { "IPv4_Driver", NULL } },
    { "UDP_Driver",      &g_udp_driver,      udp_init_adapter,      { "IPv4_Driver", NULL } },
    { "TCP_Driver",      &g_tcp_driver,      tcp_init_adapter,      { "IPv4_Driver", NULL } },
    { "DHCP_Driver",     &g_dhcp_driver,     dhcp_init_adapter,     { "IPv4_Driver", "UDP_Driver", NULL } },
};

#define NET_LAYER_COUNT ((uint32_t)(sizeof(g_net_layers) / sizeof(g_net_layers[0])))

void network_builtin_drivers_register(void)
{
    for (uint32_t i = 0; i < NET_LAYER_COUNT; ++i) {
        (void)driver_manager_attach(g_net_layers[i].name,
                                    DEVICE_TYPE_NET_PROTOCOL,
                                    g_net_layers[i].ops);
    }
}

static bool net_layer_index(const char *name, uint32_t *out_index)
{
    for (uint32_t i = 0; i < NET_LAYER_COUNT; ++i) {
        const char *a = g_net_layers[i].name;
        const char *b = name;
        while (*a != '\0' && *a == *b) { ++a; ++b; }
        if (*a == '\0' && *b == '\0') {
            *out_index = i;
            return true;
        }
    }
    return false;
}

bool network_builtin_drivers_init_all(void)
{
    network_builtin_drivers_register();   /* idempotent: DeviceRegistry adds by name */

    bool done[NET_LAYER_COUNT];
    for (uint32_t i = 0; i < NET_LAYER_COUNT; ++i) {
        done[i] = false;
    }

    /* Kahn-style: each sweep initialises every layer whose deps are already
     * initialised. NET_LAYER_COUNT sweeps is always enough for a DAG this
     * size; a sweep that makes no progress means an unresolved/cyclic dep. */
    for (uint32_t sweep = 0; sweep < NET_LAYER_COUNT; ++sweep) {
        bool progressed = false;

        for (uint32_t i = 0; i < NET_LAYER_COUNT; ++i) {
            if (done[i]) {
                continue;
            }

            bool deps_ready = true;
            for (uint32_t d = 0; d < 4u && g_net_layers[i].deps[d] != NULL; ++d) {
                uint32_t dep_idx = 0;
                if (!net_layer_index(g_net_layers[i].deps[d], &dep_idx) ||
                    !done[dep_idx]) {
                    deps_ready = false;
                    break;
                }
            }
            if (!deps_ready) {
                continue;
            }

            if (!g_net_layers[i].init()) {
                return false;
            }
            done[i] = true;
            progressed = true;
        }

        if (!progressed) {
            break;
        }
    }

    for (uint32_t i = 0; i < NET_LAYER_COUNT; ++i) {
        if (!done[i]) {
            return false;
        }
    }
    return true;
}
