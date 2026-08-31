#pragma once

/*
 * Registers each network protocol layer (Ethernet/ARP/IPv4/ICMP/UDP/TCP/
 * DHCP) as an individually named DEVICE_TYPE_NET_PROTOCOL entry in
 * DeviceRegistry, with a real callable vtable and recorded dependency
 * metadata -- see Docs/Others/TODO_OS_Refactor.md phase P5, 9.2, and the
 * per-layer vtable typedefs in NetworkBuiltinDrivers.c.
 *
 * Scope note (the hybrid approach decided for 9.2): this does NOT convert
 * Kernel/Network's internal cross-layer calls (e.g. ARP.c calling
 * ethernet_send() directly, DHCP.c calling ipv4_send()/udp_send()
 * directly) to go through these vtables via driver_manager_find() --
 * those stay exactly as direct C function calls, unchanged. What this
 * *does* provide: (1) each layer is visible in driver listings under its
 * own name, the same way a real loaded .ELF module is; (2) each layer's
 * public API is available as a genuine function-pointer vtable a new,
 * decoupled caller (e.g. a userland-facing syscall path added later, or a
 * future incremental "switch one layer's callers to the vtable" pass) can
 * reach via driver_manager_find(DEVICE_TYPE_NET_PROTOCOL, "..._Driver")
 * instead of an extern declaration; (3) the deps[] each entry records is
 * accurate, real dependency information a future full ELF-module
 * conversion (should the project pursue one) can use as-is.
 *
 * Update (this pass): the boot-time init sequence is no longer open-coded
 * in network_main.c either -- network_builtin_drivers_init_all() drives it
 * from the deps[] metadata below. Runtime paths (ethernet_poll(),
 * ipv4_process_timer(), the DHCP retry loop in network_stack_on_timer_tick())
 * are still direct calls.
 */
#include <stdbool.h>

void network_builtin_drivers_register(void);

/*
 * Bring every protocol layer up in dependency order (deps[] in
 * NetworkBuiltinDrivers.c decides the order; the void adapters there supply
 * the static-address parameters arp_init()/ipv4_init() take). Calls
 * network_builtin_drivers_register() first, so a separate registration call
 * is not required. Returns false if any layer's init fails or a dependency
 * cannot be resolved. Kernel/Network/network_main.c's network_stack_init()
 * calls this instead of open-coding the init sequence.
 */
bool network_builtin_drivers_init_all(void);
