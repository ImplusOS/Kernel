#pragma once

#include <stdint.h>

#define PSCI_SUCCESS 0
#define PSCI_NOT_SUPPORTED (-1)
#define PSCI_CPU_ON_64 0xC4000003u

int64_t arm64_psci_cpu_on(uint64_t mpidr, uint64_t entry, uint64_t context);

