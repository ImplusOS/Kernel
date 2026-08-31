#include "network_main.h"
#include "network_utils.h"

#include "kernel/config.h"
#include "Drivers/Module/DriverManager.h"
#include "Drivers/Module/NetworkBuiltinDrivers.h"
#include "Network/ethernet/Ethernet.h"
#include "ipv4.h"
#include "Network/tcp/TCP.h"
#include "Network/dhcp/DHCP.h"

static int g_network_ready = 0;
static uint32_t g_timer_ticks_for_aging = 0u;

/* DHCP is driven from here rather than from any specific NIC driver: no
 * caller anywhere kicked off dhcp_discover() before this (every NIC
 * driver -- wired or AX900 -- just left the stack on the static
 * OS_CONFIG_NET_IPV4_* address above, which happens to match QEMU's
 * user-mode NAT defaults but is meaningless on a real network/AP). Kick
 * a DHCP lease the moment the active NIC reports link-up, and keep
 * retrying at a fixed interval until one is actually assigned (dhcp_init()
 * lets a lost DISCOVER/OFFER/ACK just get retried with a fresh xid rather
 * than wedging forever). */
static bool g_dhcp_nic_was_ready = false;
static uint32_t g_dhcp_retry_ticks = 0u;
#define DHCP_RETRY_INTERVAL_TICKS 500u /* ~2s at the 250Hz system timer */

bool network_stack_init(void)
{
    if (g_network_ready != 0) {
        return true;
    }

    /* Ethernet -> ARP -> IPv4 -> {ICMP,UDP,TCP} -> DHCP, ordered from the
     * DEVICE_TYPE_NET_PROTOCOL layers' recorded deps[] rather than spelled
     * out here. The kernel's network subsystem does not name or call the
     * individual layer init functions any more. */
    if (!network_builtin_drivers_init_all()) {
        return false;
    }

    g_timer_ticks_for_aging = 0u;
    g_network_ready = 1;
    return true;
}

bool network_stack_is_ready(void)
{
    return g_network_ready != 0;
}

void network_stack_poll(void)
{
    if (g_network_ready == 0) {
        return;
    }

    ethernet_poll();
}

void network_stack_schedule_poll(void)
{
    if (g_network_ready == 0) {
        return;
    }
    driver_manager_nic_schedule_poll();
}

bool network_stack_check_poll(void)
{
    if (g_network_ready == 0) {
        return false;
    }
    return driver_manager_nic_check_poll();
}

void network_stack_on_timer_tick(void)
{
    if (g_network_ready == 0) {
        return;
    }

    driver_manager_nic_schedule_poll();

    g_timer_ticks_for_aging++;
    if (g_timer_ticks_for_aging >= 60u) {
        g_timer_ticks_for_aging = 0u;
        ipv4_process_timer();
        tcp_process_timer();
    }

    bool nic_ready = driver_manager_nic_is_ready();
    if (nic_ready && !g_dhcp_nic_was_ready) {
        /* Fresh link (including a just-completed AX900 association):
         * reset DHCP's own state machine so a lease held over from a
         * previous NIC/AP isn't reused against a network it was never
         * issued on. */
        dhcp_init();
        (void)dhcp_discover();
        g_dhcp_retry_ticks = 0u;
    }
    g_dhcp_nic_was_ready = nic_ready;

    if (nic_ready && dhcp_get_assigned_ip() == 0u) {
        g_dhcp_retry_ticks++;
        if (g_dhcp_retry_ticks >= DHCP_RETRY_INTERVAL_TICKS) {
            g_dhcp_retry_ticks = 0u;
            (void)dhcp_discover();
        }
    }
}

uint32_t network_stack_local_ipv4(void)
{
    return ipv4_local_address();
}
