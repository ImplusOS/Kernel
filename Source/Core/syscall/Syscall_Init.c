#include "Syscall_Main.h"
#include "cpu/GDT_Main.h"
#include "kernel/config.h"
#include "smp/SMP_Main.h"
#include "Compat/compat_registry.h"
#include "Compat/Linux/Syscall_LinuxCompat.h"
#include <stdint.h>

#define SYSCALL_KERNEL_STACK_SIZE (64 * 1024)
#define SYSCALL_MAX_CPUS OS_CONFIG_SMP_MAX_CPUS

static inline void wrmsr(uint32_t msr, uint64_t value) {
#if defined(__aarch64__)
    (void)msr;
    (void)value;
#else
    uint32_t low = value & 0xFFFFFFFF;
    uint32_t high = value >> 32;
    __asm__ volatile ("wrmsr" :: "c"(msr), "a"(low), "d"(high));
#endif
}

#define IA32_EFER           0xC0000080
#define IA32_STAR           0xC0000081
#define IA32_LSTAR          0xC0000082
#define IA32_FMASK          0xC0000084
#define IA32_GS_BASE        0xC0000101
#define IA32_KERNEL_GS_BASE 0xC0000102
#define EFER_SCE            (1ULL << 0)
#define RFLAGS_TF           (1ULL << 8)
#define RFLAGS_IF           (1ULL << 9)
#define RFLAGS_DF           (1ULL << 10)
#define RFLAGS_IOPL         (3ULL << 12)
#define RFLAGS_NT           (1ULL << 14)
#define RFLAGS_AC           (1ULL << 18)

#define CR0_MP              (1ULL << 1)
#define CR0_EM              (1ULL << 2)
#define CR0_TS              (1ULL << 3)
#define CR0_NE              (1ULL << 5)
#define CR4_OSFXSR          (1ULL << 9)
#define CR4_OSXMMEXCPT      (1ULL << 10)

static void syscall_init_fpu_for_cpu(void)
{
#if defined(__aarch64__)
    uint64_t cpacr = (3ULL << 20) | (3ULL << 22);
    __asm__ volatile("msr CPACR_EL1, %0; isb" :: "r"(cpacr) : "memory");
#else
    uint64_t cr0 = 0;
    uint64_t cr4 = 0;
    uint32_t mxcsr = 0x1F80U;

    __asm__ volatile("mov %%cr0, %0" : "=r"(cr0));
    cr0 &= ~(CR0_EM | CR0_TS);
    cr0 |= (CR0_MP | CR0_NE);
    __asm__ volatile("mov %0, %%cr0" :: "r"(cr0) : "memory");

    __asm__ volatile("mov %%cr4, %0" : "=r"(cr4));
    cr4 |= (CR4_OSFXSR | CR4_OSXMMEXCPT);
    __asm__ volatile("mov %0, %%cr4" :: "r"(cr4) : "memory");

    __asm__ volatile("fninit");
    __asm__ volatile("ldmxcsr %0" :: "m"(mxcsr));
#endif
}

typedef struct {
    uint64_t user_rsp;
    uint64_t kernel_rsp;
} syscall_cpu_state_t;

static syscall_cpu_state_t g_syscall_cpu_state[SYSCALL_MAX_CPUS];
static uint8_t g_syscall_kernel_stack[SYSCALL_MAX_CPUS][SYSCALL_KERNEL_STACK_SIZE]
    __attribute__((aligned(16)));

static inline uint32_t syscall_current_cpu_id(void)
{
    return smp_get_current_cpu_id();
}

static inline syscall_cpu_state_t *syscall_cpu_state_current(void)
{
    uint32_t cpu_id = syscall_current_cpu_id();
    if (cpu_id >= SYSCALL_MAX_CPUS) {
        cpu_id = 0;
    }
    return &g_syscall_cpu_state[cpu_id];
}

static void syscall_init_cpu(uint32_t cpu_id)
{
    if (cpu_id >= SYSCALL_MAX_CPUS) {
        cpu_id = 0;
    }

    uint64_t kernel_rsp = (uint64_t)(g_syscall_kernel_stack[cpu_id] + SYSCALL_KERNEL_STACK_SIZE);
    g_syscall_cpu_state[cpu_id].user_rsp = 0;
    g_syscall_cpu_state[cpu_id].kernel_rsp = kernel_rsp & ~0xFULL;
    syscall_init_fpu_for_cpu();
}

extern void syscall_entry(void);

uint64_t syscall_get_user_rsp(void)
{
    return syscall_cpu_state_current()->user_rsp;
}

void syscall_set_user_rsp(uint64_t user_rsp)
{
    syscall_cpu_state_current()->user_rsp = user_rsp;
}

uint64_t syscall_get_kernel_rsp(void)
{
    return syscall_cpu_state_current()->kernel_rsp;
}

void syscall_set_kernel_rsp(uint64_t kernel_rsp)
{
    syscall_cpu_state_current()->kernel_rsp = kernel_rsp & ~0xFULL;
}

void syscall_init_per_cpu(void) {
    uint32_t cpu_id = syscall_current_cpu_id();
    if (cpu_id >= SYSCALL_MAX_CPUS) {
        cpu_id = 0;
    }
    syscall_init_cpu(cpu_id);

#if defined(__aarch64__)
    __asm__ volatile("msr TPIDR_EL1, %0" :: "r"(&g_syscall_cpu_state[cpu_id]) : "memory");
#else
    uint64_t efer_low, efer_high;
    __asm__ volatile ("rdmsr" : "=a"(efer_low), "=d"(efer_high) : "c"(IA32_EFER));
    uint64_t efer = (efer_high << 32) | efer_low;
    efer |= EFER_SCE;
    wrmsr(IA32_EFER, efer);

    wrmsr(IA32_LSTAR, (uint64_t)syscall_entry);
    uint64_t star =
        ((uint64_t)GDT_KERNEL_CODE << 32) |
        ((uint64_t)GDT_USER_COMPAT_CODE << 48);
    wrmsr(IA32_STAR, star);

    wrmsr(IA32_GS_BASE, (uint64_t)&g_syscall_cpu_state[cpu_id]);
    wrmsr(IA32_KERNEL_GS_BASE, 0);
    wrmsr(IA32_FMASK, RFLAGS_TF | RFLAGS_IF | RFLAGS_DF |
                      RFLAGS_IOPL | RFLAGS_NT | RFLAGS_AC);
#endif
}

void syscall_init(void) {
    syscall_init_per_cpu();

    /* Kernel-side syscall-ABI compat layer registration (Docs/Others/
     * TODO_OS_Refactor.md phase P6) -- Syscall_Dispatch.c looks these up
     * via compat_registry_find() instead of a hardcoded
     * "if (abi == PROCESS_ABI_LINUX)". A future second compat layer only
     * needs its own *_compat_layer_register() call added here. */
    compat_registry_init();
    linux_compat_layer_register();
}
