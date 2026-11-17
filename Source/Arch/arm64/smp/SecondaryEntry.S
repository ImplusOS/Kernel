/*
 * Secondary-CPU entry point, handed to PSCI CPU_ON.
 *
 * PSCI starts the CPU here with the MMU and caches off and x0 = the context
 * id we passed: the physical (= identity-mapped) address of this CPU's
 * arm64_ap_boot_t (SMP_Main.c). With the MMU off every data access is
 * Device-nGnRnE, which neither the exclusive-monitor atomics nor the
 * compiler's unaligned-access assumptions survive, so no C runs until the
 * kernel's own translation regime is on.
 *
 * arm64_ap_boot_t layout (keep in step with SMP_Main.c):
 *   0 stack_top  8 ttbr  16 tcr  24 mair  32 sctlr  40 entry  48 cpacr
 */
    .section .text
    .align 12
    .global arm64_secondary_entry
    .type arm64_secondary_entry, %function
arm64_secondary_entry:
    msr     daifset, #0xf
    mov     x19, x0

    /* Entered at EL2 (a kernel started at EL2 hands its secondaries the same
     * EL): drop to EL1 the way the boot manager's trampoline does for the
     * boot CPU. */
    mrs     x1, CurrentEL
    lsr     x1, x1, #2
    cmp     x1, #2
    b.ne    1f
    mrs     x2, hcr_el2
    orr     x2, x2, #(1 << 31)          /* RW: EL1 is AArch64 */
    msr     hcr_el2, x2
    mrs     x2, cptr_el2
    bic     x2, x2, #(1 << 10)          /* no FP/SIMD trap to EL2 */
    msr     cptr_el2, x2
    mrs     x2, cnthctl_el2
    orr     x2, x2, #3                  /* EL1 counter/timer access */
    msr     cnthctl_el2, x2
    msr     cntvoff_el2, xzr
    mov     x2, #0x0830
    movk    x2, #0x30C5, lsl #16        /* SCTLR_EL1 RES1, MMU off */
    msr     sctlr_el1, x2
    adr     x2, 1f
    msr     elr_el2, x2
    mov     x2, #0x3C5                  /* EL1h, DAIF masked */
    msr     spsr_el2, x2
    isb
    eret
1:
    ldr     x2, [x19, #48]
    msr     cpacr_el1, x2
    ldr     x1, [x19, #8]
    ldr     x2, [x19, #16]
    ldr     x3, [x19, #24]
    ldr     x4, [x19, #32]
    msr     mair_el1, x3
    msr     tcr_el1, x2
    msr     ttbr0_el1, x1
    msr     ttbr1_el1, x1
    isb
    tlbi    vmalle1
    dsb     nsh
    isb
    ic      iallu
    dsb     nsh
    isb
    msr     sctlr_el1, x4
    isb

    ldr     x1, [x19, #0]
    mov     sp, x1
    ldr     x2, [x19, #40]
    mov     x0, x19
    blr     x2
2:
    wfi
    b       2b
    .size arm64_secondary_entry, . - arm64_secondary_entry
