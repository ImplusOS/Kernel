#pragma once

#include <stdint.h>

#define PSCI_SUCCESS          0
#define PSCI_NOT_SUPPORTED    (-1)
#define PSCI_INVALID_PARAMS   (-2)
#define PSCI_DENIED           (-3)
#define PSCI_ALREADY_ON       (-4)

#define PSCI_VERSION          0x84000000u
#define PSCI_SYSTEM_OFF       0x84000008u
#define PSCI_SYSTEM_RESET     0x84000009u
#define PSCI_CPU_ON_64        0xC4000003u

/* Pick the PSCI conduit: HVC when the FADT says PSCI_USE_HVC (QEMU virt
 * without EL3, the usual case), SMC otherwise. Until this is called the
 * conduit is HVC, which is also what a kernel entered at EL1 under a
 * hypervisor needs. */
void arm64_psci_set_conduit_hvc(int use_hvc);
int64_t arm64_psci_cpu_on(uint64_t mpidr, uint64_t entry, uint64_t context);
void arm64_psci_system_off(void);
void arm64_psci_system_reset(void);
