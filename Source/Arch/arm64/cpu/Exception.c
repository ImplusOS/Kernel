#include "Exception.h"
#include "CPU.h"
#include "IDT_Main.h"
#include "Arch/arm64/interrupt/GIC.h"
#include "Arch/arm64/mmu/Paging_Main.h"
#include "Core/memory/FileMap.h"
#include "Core/process/ProcessManager.h"
#include "Core/syscall/Syscall_Main.h"
#include "Debug/panic/Panic.h"
#include "Debug/serial/Serial.h"
#include "Drivers/Module/InterruptManager.h"
#include "interfaces/hal_cpu.h"
#include "kernel/config.h"
#include "smp/SMP_Main.h"

#define ESR_EC_SHIFT 26
#define ESR_EC_MASK  0x3FU

/* Exception classes (ARM DDI 0487, ESR_ELx.EC). */
#define ESR_EC_UNKNOWN      0x00U
#define ESR_EC_FP_ACCESS    0x07U
#define ESR_EC_ILLEGAL      0x0EU
#define ESR_EC_SVC64        0x15U
#define ESR_EC_SYSREG       0x18U
#define ESR_EC_IABT_LOWER   0x20U
#define ESR_EC_IABT_CUR     0x21U
#define ESR_EC_PC_ALIGN     0x22U
#define ESR_EC_DABT_LOWER   0x24U
#define ESR_EC_DABT_CUR     0x25U
#define ESR_EC_SP_ALIGN     0x26U
#define ESR_EC_FP_EXC64     0x2CU
#define ESR_EC_SERROR       0x2FU
#define ESR_EC_BRK64        0x3CU

#define ESR_ISS_WNR         (1ULL << 6)
#define ESR_ISS_FSC_MASK    0x3FULL

/* Vector table slots (ExceptionVectors.S): 0-3 current EL with SP_EL0,
 * 4-7 current EL with SP_ELx, 8-11 lower EL AArch64, 12-15 lower EL AArch32.
 * Within each group: sync, IRQ, FIQ, SError. */
#define VEC_GROUP(type)     ((type) >> 2)
#define VEC_KIND(type)      ((type) & 3u)
#define VEC_KIND_SYNC       0u
#define VEC_KIND_IRQ        1u
#define VEC_GROUP_LOWER64   2u

/* The x86 page-fault error-code bits the generic fault helpers take
 * (paging_access_is_now_permitted, process_record_page_fault). */
#define PF_ERR_PRESENT      (1ULL << 0)
#define PF_ERR_WRITE        (1ULL << 1)
#define PF_ERR_USER         (1ULL << 2)
#define PF_ERR_INSTR        (1ULL << 4)

static interrupt_handler_t g_interrupt_handlers[1024];

extern void process_preempt_from_user_irq_arm64(arm64_exception_frame_t *frame);
extern const char *process_get_current_name_str(void);
extern void process_record_page_fault(int32_t pid, uint64_t fault_addr,
                                      uint64_t rip, uint32_t error_code,
                                      int is_guard);

void arm64_exception_init(void)
{
    arm64_set_exception_vector((void *)arm64_exception_vector_table);
}

static void handle_irq(arm64_exception_frame_t *frame)
{
    (void)frame;
    uint32_t iar = arm64_gic_read_iar();
    uint32_t irq = iar & 0x3FFu;

    if (irq < 1020) {
        interrupt_manager_dispatch(irq);
        if (g_interrupt_handlers[irq]) {
            g_interrupt_handlers[irq]();
        }
    }

    arm64_gic_eoi(iar);
}

static const char *exception_class_name(uint32_t ec)
{
    switch (ec) {
    case ESR_EC_UNKNOWN:    return "undefined instruction";
    case ESR_EC_FP_ACCESS:  return "FP/SIMD access trap";
    case ESR_EC_ILLEGAL:    return "illegal execution state";
    case ESR_EC_SVC64:      return "svc";
    case ESR_EC_SYSREG:     return "system register access";
    case ESR_EC_IABT_LOWER:
    case ESR_EC_IABT_CUR:   return "instruction abort";
    case ESR_EC_PC_ALIGN:   return "PC alignment fault";
    case ESR_EC_DABT_LOWER:
    case ESR_EC_DABT_CUR:   return "data abort";
    case ESR_EC_SP_ALIGN:   return "SP alignment fault";
    case ESR_EC_FP_EXC64:   return "floating-point exception";
    case ESR_EC_SERROR:     return "SError";
    case ESR_EC_BRK64:      return "brk";
    default:                return "exception";
    }
}

static void dump_frame(const arm64_exception_frame_t *frame, uint64_t type)
{
    serial_write_string("[OS] [EXC] type=");
    serial_write_uint64(type);
    serial_write_string(" ESR=");
    serial_write_uint64(frame->esr_el1);
    serial_write_string(" FAR=");
    serial_write_uint64(frame->far_el1);
    serial_write_string(" ELR=");
    serial_write_uint64(frame->elr_el1);
    serial_write_string(" SPSR=");
    serial_write_uint64(frame->spsr_el1);
    serial_write_string(" SP_EL0=");
    serial_write_uint64(frame->sp_el0);
    /* The kernel is PIE and slid by the boot manager: this reference turns
     * an EL1 ELR back into an address `nm Kernel_Main.ELF.sym` resolves. */
    serial_write_string(" ref(serial_write_string)=");
    serial_write_uint64((uint64_t)(uintptr_t)&serial_write_string);
    serial_write_string("\n[OS] [EXC] regs:");
    for (uint32_t i = 0; i < 31u; ++i) {
        char name[5] = { 'x', 0, 0, '=', 0 };
        if (i < 10u) {
            name[1] = (char)('0' + i);
            name[2] = '=';
            name[3] = 0;
        } else {
            name[1] = (char)('0' + i / 10u);
            name[2] = (char)('0' + i % 10u);
        }
        serial_write_string(i % 4u == 0u ? "\n  " : " ");
        serial_write_string(name);
        serial_write_uint64(frame->x[i]);
    }
    serial_write_string("\n");
}

/* The current task is done for: drop it and run something else on this
 * CPU. Does not return while there is anything to run; the caller's stack
 * (the dead task's kernel stack) is abandoned. */
static void terminate_current_and_reschedule(int32_t signum)
{
    if (process_get_current_pid() >= 0 || process_get_current_tid() >= 0) {
        process_exit_current_signaled(signum);
    }
    while (!process_run_next_on_current_cpu()) {
        hal_cpu_enable_interrupts();
        hal_cpu_halt();
        hal_cpu_disable_interrupts();
    }
}

/* Data and instruction aborts: the arm64 counterpart of the x86
 * page_fault_handler(). Returns only when the faulting access should be
 * retried. */
static void handle_abort(arm64_exception_frame_t *frame, uint64_t type,
                         uint32_t ec)
{
    int from_user = (VEC_GROUP(type) == VEC_GROUP_LOWER64);
    int is_instr = (ec == ESR_EC_IABT_LOWER || ec == ESR_EC_IABT_CUR);
    uint64_t fsc = frame->esr_el1 & ESR_ISS_FSC_MASK;
    uint64_t far = frame->far_el1;

    uint64_t error_code = 0;
    /* Translation faults (0b0001LL) are "not present"; access-flag and
     * permission faults are on a present page. */
    if ((fsc & 0x3CULL) != 0x04ULL) {
        error_code |= PF_ERR_PRESENT;
    }
    if (!is_instr && (frame->esr_el1 & ESR_ISS_WNR) != 0) {
        error_code |= PF_ERR_WRITE;
    }
    if (from_user) {
        error_code |= PF_ERR_USER;
    }
    if (is_instr) {
        error_code |= PF_ERR_INSTR;
    }

    int32_t pid = process_get_current_pid();

    static volatile uint8_t g_abort_depth[OS_CONFIG_SMP_MAX_CPUS];
    uint32_t cpu = smp_get_current_cpu_id();
    int reentrant = 0;
    if (cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        reentrant = (g_abort_depth[cpu] != 0);
        g_abort_depth[cpu] = (uint8_t)(g_abort_depth[cpu] + 1u);
    }

    /* Same order as x86: a demand-paged file mapping first (it must be filled
     * from the file, not with zeroes), then "already permitted" (another CPU
     * mapped it while our TLB held the old entry), then demand-zero. Kernel-
     * mode faults at user addresses are copy_to/from_user into a lazily
     * committed buffer and get the same service. */
    int serviced = 0;
    if (pid >= 0 && !reentrant && paging_addr_is_user(far)) {
        uint64_t space = process_get_current_cr3();
        if (filemap_handle_fault(pid, space, far) > 0) {
            serviced = 1;
        }
        if (!serviced &&
            paging_access_is_now_permitted(space, far, error_code)) {
            paging_invalidate_page(far);
            serviced = 1;
        }
        if (!serviced && paging_handle_swap_fault(space, far) > 0) {
            process_record_page_fault(pid, far, frame->elr_el1,
                                      (uint32_t)error_code, 0);
            serviced = 1;
        }
    }

    if (cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS && g_abort_depth[cpu] != 0) {
        g_abort_depth[cpu]--;
    }
    if (serviced) {
        return;
    }

    /* The raw frame first: everything below may take the process table
     * lock, which a kernel-mode fault can be holding. */
    dump_frame(frame, type);
    if (!from_user) {
        /* A fault inside the kernel at a kernel address is the kernel's own
         * bug; do not go near locks it may hold. */
        if (!paging_addr_is_user(far)) {
            kernel_panic("PAGE_FAULT", "Abort in kernel mode");
        }
    }
    const char *name = process_get_current_name_str();
    serial_write_string("[OS] [PF] ");
    serial_write_string(is_instr ? "instruction" : "data");
    serial_write_string(" abort pid=");
    serial_write_uint64((uint64_t)(uint32_t)pid);
    serial_write_string(" tid=");
    serial_write_uint64((uint64_t)(uint32_t)process_get_current_tid());
    serial_write_string(" name=");
    serial_write_string(name ? name : "?");
    serial_write_string(" mode=");
    serial_write_string(from_user ? "user" : "kernel");
    serial_write_string(" access=");
    serial_write_string((error_code & PF_ERR_WRITE) ? "write" : "read");
    serial_write_string(" fsc=");
    serial_write_uint64(fsc);
    serial_write_string(" pte=");
    serial_write_uint64(paging_debug_leaf_pte(process_get_current_cr3(), far));
    serial_write_string("\n");

    if (from_user) {
        int guard = process_is_guard_page_fault(far);
        if (guard) {
            serial_write_string("[OS] [PF] guard page hit (stack overflow)\n");
        }
        process_record_page_fault(pid, far, frame->elr_el1,
                                  (uint32_t)error_code, guard);
        if (pid >= 0) {
            process_debug_dump_pid(pid);
        }
        serial_write_string("[OS] [PF] Terminating process (mode=user)\n");
        terminate_current_and_reschedule(11 /* SIGSEGV */);
        return;
    }

    /* A refused kernel access to a user buffer belongs to the process whose
     * buffer it is: kill it, keep the machine. */
    if (paging_addr_is_user(far) &&
        (pid >= 0 || process_get_current_tid() >= 0)) {
        serial_write_string("[OS] [PF] refused user buffer -> terminating process\n");
        terminate_current_and_reschedule(11 /* SIGSEGV */);
        return;
    }

    kernel_panic("PAGE_FAULT", "Abort in kernel mode");
}

/* A synchronous exception other than svc/abort taken from EL0: the Linux
 * mapping from arch/arm64/kernel/traps.c, then terminate the task. */
static void handle_user_trap(arm64_exception_frame_t *frame, uint64_t type,
                             uint32_t ec)
{
    int32_t signum;
    switch (ec) {
    case ESR_EC_UNKNOWN:
    case ESR_EC_ILLEGAL:
    case ESR_EC_SYSREG:    signum = 4;  break; /* SIGILL */
    case ESR_EC_PC_ALIGN:
    case ESR_EC_SP_ALIGN:  signum = 7;  break; /* SIGBUS */
    case ESR_EC_FP_EXC64:  signum = 8;  break; /* SIGFPE */
    case ESR_EC_BRK64:     signum = 5;  break; /* SIGTRAP */
    default:               signum = 11; break; /* SIGSEGV */
    }

    const char *name = process_get_current_name_str();
    serial_write_string("[OS] [EXC] user ");
    serial_write_string(exception_class_name(ec));
    serial_write_string(" -> signal ");
    serial_write_uint32((uint32_t)signum);
    serial_write_string(" pid=");
    serial_write_uint64((uint64_t)(uint32_t)process_get_current_pid());
    serial_write_string(" tid=");
    serial_write_uint64((uint64_t)(uint32_t)process_get_current_tid());
    serial_write_string(" name=");
    serial_write_string(name ? name : "?");
    serial_write_string("\n");
    dump_frame(frame, type);
    terminate_current_and_reschedule(signum);
}

arm64_exception_frame_t *arm64_exception_dispatch(arm64_exception_frame_t *frame, uint64_t type)
{
    uint32_t kind = VEC_KIND(type);
    uint32_t group = VEC_GROUP(type);

    if (kind == VEC_KIND_IRQ) {
        handle_irq(frame);
        if (group == VEC_GROUP_LOWER64) {
            /* Interrupted user code whose slice is up: switch now. Does not
             * return when it switches. */
            process_preempt_from_user_irq_arm64(frame);
        }
        return frame;
    }

    if (kind == VEC_KIND_SYNC) {
        uint32_t ec = (uint32_t)((frame->esr_el1 >> ESR_EC_SHIFT) & ESR_EC_MASK);

        if (ec == ESR_EC_SVC64 && group == VEC_GROUP_LOWER64) {
            uint64_t nr = frame->x[8];
            syscall_set_user_rsp(frame->sp_el0);
            uint64_t next_frame_addr =
                syscall_dispatch((uint64_t)(uintptr_t)frame, nr,
                                 frame->x[0], frame->x[1], frame->x[2],
                                 frame->x[3], frame->x[4], frame->x[5]);
            arm64_exception_frame_t *next_frame =
                (arm64_exception_frame_t *)(uintptr_t)next_frame_addr;
            if (next_frame == NULL) {
                next_frame = frame;
            }
            next_frame->sp_el0 = syscall_get_user_rsp();
            return next_frame;
        }

        if (ec == ESR_EC_DABT_LOWER || ec == ESR_EC_DABT_CUR ||
            ec == ESR_EC_IABT_LOWER || ec == ESR_EC_IABT_CUR) {
            handle_abort(frame, type, ec);
            return frame;
        }

        if (group == VEC_GROUP_LOWER64) {
            handle_user_trap(frame, type, ec);
            return frame;
        }

        serial_write_string("\n[OS] [EXC] kernel ");
        serial_write_string(exception_class_name(ec));
        serial_write_string("\n");
    } else {
        serial_write_string("\n[OS] [EXC] unexpected FIQ/SError\n");
    }

    dump_frame(frame, type);
    kernel_panic("Unhandled arm64 exception", "arm64_exception_dispatch");
    return frame;
}

void init_idt(void)
{
    arm64_exception_init();
}

void init_idt_per_cpu(void)
{
    arm64_exception_init();
}

void register_interrupt_handler(int vector, interrupt_handler_t handler)
{
    if (vector >= 0 && vector < 1024) {
        g_interrupt_handlers[vector] = handler;
    }
}

void init_gdt(void)
{
}

void gdt_set_kernel_rsp0(uint64_t rsp0)
{
    (void)rsp0;
}
