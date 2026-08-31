#include "PSCI.h"

static uint64_t psci_smc(uint64_t fid, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    register uint64_t x0 __asm__("x0") = fid;
    register uint64_t x1 __asm__("x1") = arg0;
    register uint64_t x2 __asm__("x2") = arg1;
    register uint64_t x3 __asm__("x3") = arg2;
    __asm__ volatile("smc #0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x3) : "memory");
    return x0;
}

int64_t arm64_psci_cpu_on(uint64_t mpidr, uint64_t entry, uint64_t context)
{
    return (int64_t)psci_smc(PSCI_CPU_ON_64, mpidr, entry, context);
}

