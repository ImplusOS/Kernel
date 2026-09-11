#include "IDT_Main.h"

#ifndef LINUX_SYSCALL_PROFILE
#define LINUX_SYSCALL_PROFILE 0
#endif
#include "interfaces/hal_cpu.h"
#include "kernel/config.h"

#include "Platform/io/IO_Main.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "Core/process/ProcessManager.h"
#include "Core/process/ProcessScheduler.h"
#include "Platform/interrupt/Interrupts.h"
#include "smp/SMP_Main.h"
#include "Debug/serial/Serial.h"
#include "Core/debug/FlightRec.h"
#include "Debug/panic/Panic.h"
#include "Drivers/Module/InterruptManager.h"

#include <stdint.h>

/* FS_BASE is not swapped on a kernel entry, so on the exception path it still
 * holds the faulting thread's pointer. Read directly: the process table copy
 * is only refreshed on a context switch. */
#define IDT_IA32_FS_BASE 0xC0000100U
static inline uint64_t rdmsr_fs_base_dbg(void)
{
    uint32_t lo, hi;
    __asm__ volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(IDT_IA32_FS_BASE));
    return ((uint64_t)hi << 32) | (uint64_t)lo;
}


#define MAX_IRQS 256
#define PANIC_STACK_DUMP_QWORDS 8

static IDT_Entry idt[IDT_ENTRIES];
static IDT_Ptr idt_ptr;
static isr_t irq_routines[MAX_IRQS] = {0};

extern void isr_default(void);
extern void isr_irq0(void);
extern void isr_page_fault(void);
extern void isr_double_fault(void);
extern void isr_nmi(void);
extern void isr_general_protection(void);
extern void isr_machine_check(void);
extern void isr_tlb_shootdown(void);
extern void (*isr_driver_table[])(void);
extern void load_idt(IDT_Ptr *idt_ptr);
extern void syscall_enter_user_from_frame(uint64_t next_saved_rsp,
                                          uint64_t next_user_rsp);

/* A panic must never itself fault -- a nested fault while the panic path runs
 * turns a clean BSOD/halt into a double -> triple fault, which QEMU services by
 * resetting the machine (the "boot loop"). The stack dumpers below walk pointers
 * (rsp, and a frame-pointer chain) that can be arbitrary garbage on a wild-jump
 * or corruption panic, so probe every address against the active page tables
 * before dereferencing it. */
static int panic_addr_readable(uint64_t addr)
{
    if (addr == 0 || (addr & 0x7ULL) != 0) {
        return 0;
    }
    uint64_t cr3 = hal_cpu_read_cr(3);
    if (paging_virt_to_phys(cr3, addr) == 0) {
        return 0;
    }
    /* Ensure the last byte of the qword is on a mapped page too. */
    if (((addr + 7u) & ~0xFFFULL) != (addr & ~0xFFFULL) &&
        paging_virt_to_phys(cr3, addr + 7u) == 0) {
        return 0;
    }
    return 1;
}

static void panic_dump_stack_words(uint64_t rsp)
{
    serial_write_string("[OS] [PANIC] Stack snapshot:\n");
    if (rsp == 0) {
        serial_write_string("[OS] [PANIC]  <invalid rsp>\n");
        return;
    }

    const uint64_t *stack = (const uint64_t *)(uintptr_t)rsp;
    for (uint32_t i = 0; i < PANIC_STACK_DUMP_QWORDS; ++i) {
        if (!panic_addr_readable(rsp + ((uint64_t)i * sizeof(uint64_t)))) {
            serial_write_string("[OS] [PANIC]  <unmapped, stopping>\n");
            return;
        }
        serial_write_string("[OS] [PANIC]  [");
        serial_write_uint32(i);
        serial_write_string("] @ ");
        serial_write_uint64(rsp + ((uint64_t)i * sizeof(uint64_t)));
        serial_write_string(" = ");
        serial_write_uint64(stack[i]);
        serial_write_string("\n");
    }
}

static void panic_dump_stack_trace(uint64_t rbp)
{
    serial_write_string("[OS] [PANIC] Stack trace:\n");
    if (rbp == 0 || (rbp & 0x7ULL) != 0) {
        serial_write_string("[OS] [PANIC]  <invalid rbp>\n");
        return;
    }

    const uint64_t max_depth = 16;
    uint64_t current_rbp = rbp;
    for (uint64_t depth = 0; depth < max_depth; ++depth) {
        if (!panic_addr_readable(current_rbp) ||
            !panic_addr_readable(current_rbp + sizeof(uint64_t))) {
            serial_write_string("[OS] [PANIC]  <unmapped rbp, stopping>\n");
            break;
        }
        const uint64_t *frame = (const uint64_t *)(uintptr_t)current_rbp;
        uint64_t next_rbp = frame[0];
        uint64_t return_rip = frame[1];

        serial_write_string("[OS] [PANIC]  #");
        serial_write_uint64(depth);
        serial_write_string(" rbp=");
        serial_write_uint64(current_rbp);
        serial_write_string(" rip=");
        serial_write_uint64(return_rip);
        serial_write_string("\n");
        if (next_rbp <= current_rbp || (next_rbp & 0x7ULL) != 0 ||
            (next_rbp - current_rbp) > 0x10000ULL) {
            break;
        }
        current_rbp = next_rbp;
    }
}

__attribute__((noreturn))
static void panic_exception(const char *name,
                            uint64_t vector,
                            uint64_t error_code,
                            uint64_t rip,
                            uint64_t rsp,
                            uint64_t rbp,
                            uint64_t cr2)
{
    uint64_t cr0 = hal_cpu_read_cr(0);
    uint64_t cr3 = hal_cpu_read_cr(3);
    uint64_t cr4 = hal_cpu_read_cr(4);

    {
        extern void serial_write_string(const char *str);
        extern void serial_write_uint64(uint64_t value);
        extern int32_t process_get_current_pid(void);
        extern const char *process_get_current_name_str(void);
        extern uint64_t process_get_current_kernel_stack_base(void);
        extern void process_debug_dump_slot_no_lock(int32_t pid);
        extern void process_scheduler_debug_dump_cpus(void);
        int32_t p = process_get_current_pid();
        serial_write_string("\n[OS] [PANIC] Fatal exception\n");
        process_scheduler_debug_dump_cpus();
        if (p >= 0) {
            const char *pn = process_get_current_name_str();
            serial_write_string("[OS] [PANIC] proc pid=");
            serial_write_uint64((uint64_t)(uint32_t)p);
            serial_write_string(" name=");
            serial_write_string(pn ? pn : "?");
            serial_write_string(" kstack=");
            serial_write_uint64(process_get_current_kernel_stack_base());
            serial_write_string("\n");
            process_debug_dump_slot_no_lock(p);
        }
    }
    serial_write_string("[OS] [PANIC] name: ");
    serial_write_string(name);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] vector: ");
    serial_write_uint64(vector);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] error: ");
    serial_write_uint64(error_code);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] RIP: ");
    serial_write_uint64(rip);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] RSP: ");
    serial_write_uint64(rsp);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] RBP: ");
    serial_write_uint64(rbp);
    serial_write_string("\n");
    uint64_t user_rsp = 0;
    uint64_t cs = 0;
    if (error_code == 0 || error_code <= 0x12) {
        // #GP, #PF, #DF, etc. - error code is pushed by CPU
        // Stack: error_code(0), RIP(8), CS(16), RFLAGS(24), user_RSP(32), SS(40)
        cs = *(uint64_t *)(uintptr_t)(rsp + 16);
        if ((cs & 3) == 3) {
            user_rsp = *(uint64_t *)(uintptr_t)(rsp + 32);
        }
    } else if (vector == 14) {
        // Page fault: CR2 is the faulting address
    }
    serial_write_string("[OS] [PANIC] User RSP: ");
    serial_write_uint64(user_rsp);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] CS: ");
    serial_write_uint64(cs);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] CR0: ");
    serial_write_uint64(cr0);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] CR3: ");
    serial_write_uint64(cr3);
    serial_write_string("\n");
    serial_write_string("[OS] [PANIC] CR4: ");
    serial_write_uint64(cr4);
    serial_write_string("\n");
    if (cr2 != 0) {
        serial_write_string("[OS] [PANIC] CR2: ");
        serial_write_uint64(cr2);
        serial_write_string("\n");
    }

    memory_dump_virtual((const void *)(uintptr_t)rsp,
                        PANIC_STACK_DUMP_QWORDS * (uint32_t)sizeof(uint64_t));
    panic_dump_stack_words(rsp);
    panic_dump_stack_trace(rbp);

    {
        extern void serial_write_string(const char *str);
        extern void serial_write_uint64(uint64_t value);
        serial_write_string("[OS] [PANIC] rip bytes:");
        if (paging_is_user_range_mapped(cr3, rip & ~0x7ULL, 16u)) {
            const uint8_t *p = (const uint8_t *)(uintptr_t)(rip & ~0x7ULL);
            for (uint32_t i = 0; i < 16; ++i) {
                serial_write_string(" ");
                serial_write_uint64(p[i]);
            }
        } else {
            serial_write_string(" unmapped");
        }
        serial_write_string("\n");
    }

    if ((cs & 3) == 3) {
        extern void serial_write_string(const char *str);
        extern void serial_write_uint64(uint64_t value);
        serial_write_string("[OS] [PANIC] user-rip bytes: ");
        if (paging_is_user_range_mapped(cr3, rip & ~0x7ULL, 16u)) {
            const uint8_t *p = (const uint8_t *)(uintptr_t)(rip & ~0x7ULL);
            for (uint32_t i = 0; i < 16; ++i) {
                uint32_t b = p[i];
                serial_write_string(" ");
                serial_write_uint64(b);
            }
        } else {
            serial_write_string(" unmapped");
        }
        serial_write_string("\n");
    }

    while (1) {
        hal_cpu_halt();
    }
}

IDT_Ptr *idt_get_ptr(void)
{
    return &idt_ptr;
}

void register_interrupt_handler(uint16_t irq, isr_t handler)
{
    if (irq < MAX_IRQS) {
        irq_routines[irq] = handler;
    }
}

static int pic_check_spurious(uint16_t irq_num)
{
    if (irq_num == 39) {
        outb(0x20, 0x0B);
        uint8_t isr = inb(0x20);
        if ((isr & 0x80) == 0) {
            return 1;
        }
    }
    if (irq_num == 47) {
        outb(0xA0, 0x0B);
        uint8_t isr = inb(0xA0);
        if ((isr & 0x80) == 0) {
            outb(0x20, 0x20);
            return 1;
        }
    }
    return 0;
}

void irq_handler(uint16_t irq_num)
{
    if (irq_num == VECTOR_TLB_SHOOTDOWN) {
        smp_tlb_shootdown_handler();
        return;
    }

    if (irq_num >= 32 && irq_num < 48) {
        if (pic_check_spurious(irq_num)) {
            return;
        }
    }

    if (irq_num < MAX_IRQS && irq_routines[irq_num]) {
        irq_routines[irq_num]();
    }
    interrupt_manager_dispatch(irq_num);

    if (irq_num >= 32) {
        platform_interrupts_eoi(irq_num);
    }
}

void double_fault_handler(uint64_t error_code, uint64_t rip, uint64_t rsp, uint64_t rbp)
{
    panic_exception("double_fault", 8, error_code, rip, rsp, rbp, 0);
}

void nmi_handler(uint64_t rip, uint64_t rsp, uint64_t rbp)
{
    panic_exception("nmi", 2, 0, rip, rsp, rbp, 0);
}

void general_protection_fault_handler(const uint64_t *gpregs,
                                      uint64_t error_code,
                                      uint64_t rip,
                                      const uint64_t *frame,
                                      uint64_t rbp)
{
    /* `frame` points at the CPU-pushed exception frame:
     * [0]=error_code [1]=rip [2]=cs [3]=rflags [4]=user_rsp [5]=user_ss.
     * `gpregs`, when non-NULL (fed by isr_general_protection), is the
     * SAVE_REGS window: [0]=r15 [1]=r14 [2]=r13 [3]=r12 [4]=r11 [5]=r10
     * [6]=r9 [7]=r8 [8]=rbp [9]=rdi [10]=rsi [11]=rdx [12]=rcx [13]=rbx
     * [14]=rax.
     *
     * A #GP that came from CPL 3 is the userland process's problem, not the
     * kernel's -- terminate that process and keep running. Foreign Linux
     * binaries reach here a lot: Chromium's IMMEDIATE_CRASH()/failed CHECK()
     * is `int3; ud2`, and a userland `int3` traps the DPL-0 #BP gate as #GP
     * with error = (3<<3)|2 = 0x1A; privileged instructions and non-canonical
     * accesses land here too. */
    int from_user = (frame != NULL) && ((frame[2] & 0x3ULL) == 0x3ULL);
    int32_t pid = process_get_current_pid();

    if (from_user && pid >= 0) {
        extern const char *process_get_current_name_str(void);
        extern void process_debug_dump_pid(int32_t pid);
        extern void process_exit_current_signaled(int32_t signum);

        serial_write_string("[OS] [#GP] user-mode #GP -> terminating pid=");
        serial_write_uint64((uint64_t)(uint32_t)pid);
        serial_write_string(" name=");
        const char *pn = process_get_current_name_str();
        serial_write_string(pn ? pn : "?");
        serial_write_string(" rip=");
        serial_write_uint64(rip);
        serial_write_string(" err=");
        serial_write_uint64(error_code);
        /* The exception frame itself, so a bad control transfer can be told
         * apart from a bad instruction: a non-canonical rip with a sane
         * cs/ss/rflags means the CPU was handed that rip on the way back to
         * user mode (sysret/iret) rather than jumping to it. user_rsp then
         * says which stack, and the words at it name the caller. */
        serial_write_string(" cs=");
        serial_write_uint64(frame[2]);
        serial_write_string(" rflags=");
        serial_write_uint64(frame[3]);
        serial_write_string(" user_rsp=");
        serial_write_uint64(frame[4]);
        serial_write_string(" user_ss=");
        serial_write_uint64(frame[5]);
        serial_write_string("\n");
        /* The saved GPR window names the faulting operand: JIT'd pixel loops
         * (llvmpipe swrast) crash as #GP(err=0) on a non-canonical
         * [base+idx] dereference, and the index registers (r14/r15) are not
         * in the CPU exception frame. */
        if (gpregs != NULL) {
            static const char *const gp_name[] = {
                "r15", "r14", "r13", "r12", "r11", "r10", "r9",
                "r8", "rbp", "rdi", "rsi", "rdx", "rcx", "rbx", "rax",
            };
            serial_write_string("[OS] [#GP] gpr:");
            for (int i = 0; i < 15; ++i) {
                if ((i & 1) == 0) serial_write_string("\n[OS] [#GP]   ");
                serial_write_string(gp_name[i]);
                serial_write_string("=");
                serial_write_uint64(gpregs[i]);
                serial_write_string(" ");
            }
            serial_write_string("\n");
        }
        /* The bytes at the faulting rip: a #GP with err=0 in user mode is
         * almost always either a privileged/undefined instruction or an
         * unaligned SSE access (movaps/movdqa), and those are only
         * distinguishable from the opcode. */
        if (rip >= 0x1000u &&
            process_user_buffer_is_valid((const void *)(uintptr_t)rip, 16u)) {
            const uint8_t *code = (const uint8_t *)(uintptr_t)rip;
            serial_write_string("[OS] [#GP] code:");
            for (int i = 0; i < 16; ++i) {
                serial_write_string(" ");
                serial_write_uint32((uint32_t)code[i]);
            }
            serial_write_string("\n");
        }
        {
            /* 64 qwords, not PANIC_STACK_DUMP_QWORDS: a #GP raised inside a
             * callee's own frame (an unaligned movaps in a variadic prologue,
             * say) puts the return address well above the first 8 slots, and
             * that return address is the only way to name the caller. */
            uint64_t urs = frame[4];
            if (urs >= 0x1000u && (urs & 7u) == 0u &&
                process_user_buffer_is_valid((const void *)(uintptr_t)urs,
                                             8u * 64u)) {
                const uint64_t *ustack = (const uint64_t *)(uintptr_t)urs;
                for (int i = 0; i < 64; ++i) {
                    if ((i & 7) == 0) {
                        serial_write_string("[OS] [#GP] ustack+");
                        serial_write_uint32((uint32_t)(i * 8));
                        serial_write_string(":");
                    }
                    serial_write_string(" ");
                    serial_write_uint64(ustack[i]);
                    if ((i & 7) == 7) serial_write_string("\n");
                }
            }
        }
        process_debug_dump_pid(pid);

        process_exit_current_signaled(4 /* SIGILL: int3/ud2/priv-insn */);

        while (!process_run_next_on_current_cpu()) {
            hal_cpu_enable_interrupts();
            hal_cpu_halt();
        }
        return;
    }

    panic_exception("general_protection", 13, error_code, rip,
                    (uint64_t)(uintptr_t)frame, rbp, 0);
}

void machine_check_handler(uint64_t rip, uint64_t rsp, uint64_t rbp)
{
    panic_exception("machine_check", 18, 0, rip, rsp, rbp, 0);
}

void set_interrupt_handler_with_ist(uint16_t n, void (*handler)(void), uint8_t ist)
{
    uint64_t addr = (uint64_t)handler;

    idt[n].offset_low  = (uint16_t)(addr & 0xFFFF);
    idt[n].selector    = 0x08;
    idt[n].ist         = (uint8_t)(ist & 0x7u);
    idt[n].type_attr   = 0x8E;
    idt[n].offset_mid  = (uint16_t)((addr >> 16) & 0xFFFF);
    idt[n].offset_high = (uint32_t)((addr >> 32) & 0xFFFFFFFF);
    idt[n].zero        = 0;
}

void set_interrupt_handler(uint16_t n, void (*handler)(void))
{
    set_interrupt_handler_with_ist(n, handler, 0);
}

static void pic_remap(void)
{
    outb(0x20, 0x11);
    outb(0xA0, 0x11);
    outb(0x21, 0x20);
    outb(0xA1, 0x28);
    outb(0x21, 0x04);
    outb(0xA1, 0x02);
    outb(0x21, 0x01);
    outb(0xA1, 0x01);
    outb(0x21, 0xFF);
    outb(0xA1, 0xFF);
}

void init_idt(void)
{
    static bool initialized = false;
    if (initialized) {
        load_idt(&idt_ptr);
        return;
    }

    pic_remap();

    for (int i = 0; i < IDT_ENTRIES; i++) {
        set_interrupt_handler((uint16_t)i, isr_default);
    }

    set_interrupt_handler(32, isr_irq0);
    for (uint16_t vector = 64u; vector <= 127u; ++vector) {
        set_interrupt_handler(vector, isr_driver_table[vector - 64u]);
    }
    set_interrupt_handler_with_ist(2, isr_nmi, 2);
    set_interrupt_handler_with_ist(8, isr_double_fault, 1);
    set_interrupt_handler(13, isr_general_protection);
    set_interrupt_handler(14, isr_page_fault);
    set_interrupt_handler(18, isr_machine_check);
    set_interrupt_handler(VECTOR_TLB_SHOOTDOWN, isr_tlb_shootdown);

    idt_ptr.limit = sizeof(idt) - 1;
    idt_ptr.base  = (uint64_t)&idt;

    load_idt(&idt_ptr);
    initialized = true;
}

void init_idt_per_cpu(void)
{
    load_idt(&idt_ptr);
}

int32_t page_fault_handler(uint64_t error_code,
                           uint64_t rip,
                           uint64_t frame_rsp,
                           uint64_t kernel_rsp,
                           uint64_t cr2,
                           uint64_t rbp)
{
    flight_rec(FR_TAG_PF, cr2, rip);
    const uint64_t PF_WRITE = (1ULL << 1);
    const uint64_t PF_USER  = (1ULL << 2);
    const uint64_t PF_RSVD  = (1ULL << 3);
    const uint64_t PF_INSTR = (1ULL << 4);
    uint64_t user_rsp = 0;

    if (frame_rsp != 0) {
        const uint64_t *frame = (const uint64_t *)(uintptr_t)frame_rsp;
        uint64_t cs = frame[2];
        if ((cs & 0x3ULL) == 0x3ULL) {
            user_rsp = frame[4];
        } else {
            user_rsp = frame_rsp;
        }
    }

    int32_t pid = process_get_current_pid();

    /* Re-entrancy guard: if we take another page fault while already handling
     * one on this CPU, the demand-paging path itself (or something it calls)
     * faulted -- a kernel bug. Recursing would grow the stack unbounded and can
     * end in a triple fault. Bail straight to the panic path instead. */
    static volatile uint8_t g_pf_depth[OS_CONFIG_SMP_MAX_CPUS];
    uint32_t pf_cpu = smp_get_current_cpu_id();
    int pf_reentrant = 0;
    if (pf_cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        pf_reentrant = (g_pf_depth[pf_cpu] != 0);
        g_pf_depth[pf_cpu] = (uint8_t)(g_pf_depth[pf_cpu] + 1u);
    }

#if OS_CONFIG_DEBUG_PAGE_FAULT_DUMP
    {
        extern void serial_write_string(const char *str);
        extern void serial_write_uint64(uint64_t value);
        extern const char *process_get_current_name_str(void);
        extern uint64_t process_get_current_kernel_stack_base(void);
        extern void process_scheduler_debug_dump_cpus(void);
        serial_write_string("[OS] [PF] pid=");
        serial_write_uint64((uint64_t)(uint32_t)pid);
        serial_write_string(" name=");
        const char *pn = process_get_current_name_str();
        serial_write_string(pn ? pn : "?");
        serial_write_string(" kstack=");
        serial_write_uint64(process_get_current_kernel_stack_base());
        serial_write_string("\n");
        process_scheduler_debug_dump_cpus();
    }
#endif
    /* Demand-fault user pages for BOTH user-mode faults and kernel-mode faults
     * at a user address -- the latter is copy_to/from_user() memcpy'ing into a
     * lazily-committed anonymous mmap() buffer (USER_MMAP arena). Chromium's
     * PartitionAlloc reserves huge PROT_NONE regions and only touches slivers
     * of them; without this a syscall that writes into such a buffer would
     * panic the kernel. paging_handle_swap_fault()/cow_fault() self-gate to
     * canonical user addresses, so a genuine kernel wild pointer still falls
     * through to the panic path below. */
#if LINUX_SYSCALL_PROFILE
    /* Page faults are not syscalls, so the syscall profile cannot see them --
     * and a demand-paged program spends its life here, not in syscalls. */
    {
        static volatile uint32_t pf_total;
        uint32_t n = __atomic_add_fetch(&pf_total, 1u, __ATOMIC_RELAXED);
        if ((n % 100000u) == 0u) {
            serial_write_string("[pfprof] faults=");
            serial_write_uint64((uint64_t)n);
            serial_write_string("\n");
        }
    }
#endif
    int pf_serviced = 0;
    if (pid >= 0 && !pf_reentrant) {
        uint64_t cr3 = process_get_current_cr3();
#if KERNEL_COW_FORK
        if ((error_code & PF_WRITE) != 0) {
            extern int paging_handle_cow_fault(uint64_t cr3, uint64_t fault_addr);
            if (paging_handle_cow_fault(cr3, cr2) > 0) {
                pf_serviced = 1;
            }
        }
#endif
        /* Before the demand-zero path: a fault inside a demand-paged file
         * mapping must be filled from the file, not with zeroes. The swap
         * handler below services ANY absent user page, so it would silently
         * hand back a blank page for a shared object's text. */
        if (!pf_serviced) {
            extern int filemap_handle_fault(int32_t pid, uint64_t cr3,
                                            uint64_t fault_addr);
            if (filemap_handle_fault(pid, cr3, cr2) > 0) {
                pf_serviced = 1;
            }
        }
        /* Ahead of the demand-paging handlers: if the tables already allow
         * this access, another CPU mapped (or widened) the page while this one
         * still held the old translation. Nothing to fill in -- drop the stale
         * entry and resume the instruction. See
         * paging_access_is_now_permitted(). */
        if (!pf_serviced &&
            paging_access_is_now_permitted(cr3, cr2, error_code)) {
            paging_invalidate_page(cr2);
            pf_serviced = 1;
        }
        if (!pf_serviced && paging_handle_swap_fault(cr3, cr2) > 0) {
            extern void process_record_page_fault(int32_t pid, uint64_t fault_addr,
                                                  uint64_t rip, uint32_t error_code,
                                                  int is_guard);
            process_record_page_fault(pid, cr2, rip, (uint32_t)error_code, 0);
            pf_serviced = 1;
        }
    }

    /* Past the demand-paging attempt: drop the re-entrancy guard now. Everything
     * below either resumes, terminates the faulting process (abandoning this
     * stack), or panics -- none of it re-enters the demand-paging path, and
     * leaving the counter raised would make the CPU's next real fault look
     * re-entrant. */
    if (pf_cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS && g_pf_depth[pf_cpu] != 0) {
        g_pf_depth[pf_cpu]--;
    }
    if (pf_serviced) {
        return 0;
    }

    /* Snapshot before any serial output: printing a dump line costs ~10 ms at
     * 115200 baud, which is ample time for a sibling thread to change the
     * memory we are trying to describe. Read the word the faulting indirect
     * jump went through first, and report it alongside the later re-read. */
    uint64_t nullcall_entry_addr = 0;
    uint64_t nullcall_entry_early = 0;
    /* Displacement of the indirect jump inside the dispatch trampoline, i.e.
     * which table slot the call was routed through. Captured with the rest of
     * the early snapshot so the TLS dump below can probe every candidate
     * table at the same index. */
    uint32_t nullcall_disp = 0;
    if ((error_code & PF_USER) != 0u && (error_code & PF_INSTR) != 0u &&
        cr2 < 0x1000u && pid >= 0 && user_rsp >= 0x1000u &&
        (user_rsp & 7u) == 0u &&
        process_user_buffer_is_valid((const void *)(uintptr_t)user_rsp, 8u)) {
        const uint64_t *ust0 = (const uint64_t *)(uintptr_t)user_rsp;
        const uint64_t *kr0 = (const uint64_t *)(uintptr_t)(kernel_rsp + 8u);
        uint64_t ret0 = ust0[0];
        const uint32_t win = 40u;
        uint64_t slot0 = 0;
        if (ret0 > win &&
            process_user_buffer_is_valid((const void *)(uintptr_t)(ret0 - win), win)) {
            const uint8_t *p0 = (const uint8_t *)(uintptr_t)(ret0 - win);
            for (uint32_t i = 0; i + 7u <= win; ++i) {
                if ((p0[i] & 0xF8u) != 0x48u || p0[i + 1] != 0x8Bu ||
                    (p0[i + 2] & 0xC7u) != 0x05u) {
                    continue;
                }
                uint32_t dd = (uint32_t)p0[i + 3] | ((uint32_t)p0[i + 4] << 8) |
                              ((uint32_t)p0[i + 5] << 16) | ((uint32_t)p0[i + 6] << 24);
                slot0 = (ret0 - win + i + 7u) + (uint64_t)(int64_t)(int32_t)dd;
            }
        }
        uint64_t tgt0 = (slot0 != 0u &&
                         process_user_buffer_is_valid((const void *)(uintptr_t)slot0, 8u))
                            ? *(const uint64_t *)(uintptr_t)slot0 : 0u;
        if (tgt0 != 0u &&
            process_user_buffer_is_valid((const void *)(uintptr_t)tgt0, 32u)) {
            const uint8_t *c0 = (const uint8_t *)(uintptr_t)tgt0;
            for (uint32_t i = 0; i + 7u <= 32u; ++i) {
                if (c0[i] != 0x41u || c0[i + 1] != 0xFFu || c0[i + 2] != 0xA3u) {
                    continue;
                }
                uint32_t dd = (uint32_t)c0[i + 3] | ((uint32_t)c0[i + 4] << 8) |
                              ((uint32_t)c0[i + 5] << 16) | ((uint32_t)c0[i + 6] << 24);
                nullcall_disp = dd;
                nullcall_entry_addr = kr0[4 /* r11 */] + (uint64_t)dd;
                if (process_user_buffer_is_valid(
                        (const void *)(uintptr_t)nullcall_entry_addr, 8u)) {
                    nullcall_entry_early =
                        *(const uint64_t *)(uintptr_t)nullcall_entry_addr;
                }
                break;
            }
        }
    }

    serial_write_string("[OS] [PF] Page fault pid=");
    serial_write_uint64((uint64_t)(uint32_t)pid);
    serial_write_string("\n");
    /* The address space the fault was taken in, next to the one the process
     * table says it should be: a mismatch means the CPU was executing user
     * code against someone else's page tables, and every address in this dump
     * would then mean something different to the faulting instruction than it
     * does to us. */
    serial_write_string("[OS] [PF] cr3=");
    serial_write_uint64(paging_get_active_cr3());
    serial_write_string(" proc_cr3=");
    serial_write_uint64(process_get_current_cr3());
    serial_write_string("\n");
    serial_write_string("[OS] [PF] CR2: ");
    serial_write_uint64(cr2);
    serial_write_string("\n");
    serial_write_string("[OS] [PF] RIP: ");
    serial_write_uint64(rip);
    /* The kernel is PIE and the boot manager slides it, so a raw RIP names
     * nothing on its own. Print the runtime address of a symbol from this
     * same image alongside it: the difference is the slide, and RIP minus the
     * slide is an address that `nm Kernel_Main.ELF.tmp` can resolve. */
    serial_write_string(" ref(serial_write_string)=");
    serial_write_uint64((uint64_t)(uintptr_t)&serial_write_string);
    serial_write_string("\n");
    serial_write_string("[OS] [PF] UserRSP: ");
    serial_write_uint64(user_rsp);
    serial_write_string("\n");
    serial_write_string("[OS] [PF] KernelRSP: ");
    serial_write_uint64(kernel_rsp);
    serial_write_string("\n");
    serial_write_string("[OS] [PF] RBP: ");
    serial_write_uint64(rbp);
    serial_write_string("\n");
    serial_write_string("[OS] [PF] Error: ");
    serial_write_uint64(error_code);
    serial_write_string("\n");

    serial_write_string("[OS] [PF] Access: ");
    serial_write_string((error_code & PF_WRITE) ? "write" : "read");
    serial_write_string(", mode: ");
    serial_write_string((error_code & PF_USER) ? "user" : "kernel");
    serial_write_string(", reserved: ");
    serial_write_string((error_code & PF_RSVD) ? "yes" : "no");
    serial_write_string(", exec: ");
    serial_write_string((error_code & PF_INSTR) ? "yes" : "no");
    serial_write_string("\n");

    if ((error_code & PF_USER) && pid >= 0) {
        if (process_is_guard_page_fault(cr2)) {
            serial_write_string("[OS][PF] Guard page hit (stack/heap overflow) pid=");
            serial_write_uint32((uint32_t)pid);
            serial_write_string("\n");
        } else {
            serial_write_string("[OS] [PF] Terminating process pid=");
            serial_write_uint32((uint32_t)pid);
            serial_write_string(" (mode=user)\n");
        }

        /* The words at the user stack pointer. A foreign binary that jumps
         * somewhere it should not reports only the landing address; the
         * return addresses still sitting on its stack name the caller, and
         * with the [lxmap] module bases those resolve to <module>+<offset>. */
        if (user_rsp >= 0x1000u && (user_rsp & 7u) == 0u &&
            process_user_buffer_is_valid((const void *)(uintptr_t)user_rsp,
                                         8u * PANIC_STACK_DUMP_QWORDS)) {
            const uint64_t *ustack = (const uint64_t *)(uintptr_t)user_rsp;
            serial_write_string("[OS] [PF] user stack:");
            for (int i = 0; i < PANIC_STACK_DUMP_QWORDS; ++i) {
                serial_write_string(" ");
                serial_write_uint64(ustack[i]);
            }
            serial_write_string("\n");

            /* An instruction fetch at (near) address zero is a call through a
             * NULL function pointer, and the interesting question is always
             * "which slot of which table, and what do its neighbours hold?" --
             * one NULL entry in an otherwise populated dispatch table means
             * something different from a table that is entirely zero. The
             * pointer was loaded RIP-relative right before the call, so scan
             * the bytes just below the return address for the last
             * `mov r64, [rip+disp32]` (REX.W, opcode 8B, mod=00 rm=101) and
             * dump the memory around the address it names. */
            if ((error_code & PF_INSTR) != 0u && cr2 < 0x1000u) {
                /* The register file the ISR pushed, so an indirect jump
                 * through a table (glvnd's `jmp *disp(%r11)` dispatch stubs,
                 * say) can be followed to the table itself. Layout matches
                 * SAVE_REGS in IDT.asm; kernel_rsp points one qword below it. */
                static const char *const regname[15] = {
                    "r15", "r14", "r13", "r12", "r11", "r10", "r9", "r8",
                    "rbp", "rdi", "rsi", "rdx", "rcx", "rbx", "rax"
                };
                const uint64_t *kregs =
                    (const uint64_t *)(uintptr_t)(kernel_rsp + 8u);
                /* The thread pointer and the static-TLS window below it.
                 * An initial-exec dispatch stub reaches its table with
                 * `mov tpoff(%rip),%rax; mov %fs:(%rax),%reg`, so when such a
                 * jump lands on NULL the two questions are "was the tpoff the
                 * right one for this module?" and "what does the slot it
                 * actually names hold?". Both are answered by the raw window:
                 * every loaded module's initial-exec block lives in it, and a
                 * table pointer sitting at the wrong offset is visible as a
                 * plausible pointer in a neighbouring slot. */
                {
                    uint64_t fsb = rdmsr_fs_base_dbg();
                    serial_write_string("[OS] [PF] fsbase=");
                    serial_write_uint64(fsb);
                    serial_write_string("\n");
                    /* Only the slots that hold something dereferenceable are
                     * interesting, and for each the useful follow-up is the
                     * word the faulting stub would have jumped through: two
                     * different modules' dispatch tables are told apart by
                     * what sits at the same index, not by the table address.
                     * `disp` below is that index's content, decoded from the
                     * trampoline earlier in this dump. */
                    if (fsb >= 1024u) {
                        for (int i = 0; i < 128; ++i) {
                            uint64_t slot_va = fsb - 1024u + (uint64_t)i * 8u;
                            if (!process_user_buffer_is_valid(
                                    (const void *)(uintptr_t)slot_va, 8u)) {
                                continue;
                            }
                            uint64_t v = *(const uint64_t *)(uintptr_t)slot_va;
                            if (v < 0x1000u) continue;
                            serial_write_string("[OS] [PF]  tp-");
                            serial_write_uint32((uint32_t)(1024 - i * 8));
                            serial_write_string(" = ");
                            serial_write_uint64(v);
                            if (nullcall_disp != 0u &&
                                process_user_buffer_is_valid(
                                    (const void *)(uintptr_t)(v + nullcall_disp),
                                    8u)) {
                                serial_write_string(" disp[");
                                serial_write_uint32(nullcall_disp);
                                serial_write_string("]=");
                                serial_write_uint64(
                                    *(const uint64_t *)(uintptr_t)(v + nullcall_disp));
                            }
                            serial_write_string("\n");
                        }
                    }
                }
                serial_write_string("[OS] [PF] regs:");
                for (int i = 0; i < 15; ++i) {
                    serial_write_string(" ");
                    serial_write_string(regname[i]);
                    serial_write_string("=");
                    serial_write_uint64(kregs[i]);
                }
                serial_write_string("\n");

                uint64_t ret = ustack[0];
                const uint32_t window = 40u;
                if (ret > window &&
                    process_user_buffer_is_valid(
                        (const void *)(uintptr_t)(ret - window), window)) {
                    const uint8_t *p = (const uint8_t *)(uintptr_t)(ret - window);
                    uint64_t slot = 0;
                    for (uint32_t i = 0; i + 7u <= window; ++i) {
                        if ((p[i] & 0xF8u) != 0x48u) continue; /* REX.W */
                        if (p[i + 1] != 0x8Bu) continue;       /* mov r64,r/m64 */
                        if ((p[i + 2] & 0xC7u) != 0x05u) continue; /* [rip+d32] */
                        uint32_t d = (uint32_t)p[i + 3] |
                                     ((uint32_t)p[i + 4] << 8) |
                                     ((uint32_t)p[i + 5] << 16) |
                                     ((uint32_t)p[i + 6] << 24);
                        slot = (ret - window + i + 7u) + (uint64_t)(int64_t)(int32_t)d;
                    }
                    uint64_t base = slot >= 0x40u ? ((slot - 0x40u) & ~7ULL) : 0u;
                    if (base != 0u &&
                        process_user_buffer_is_valid((const void *)(uintptr_t)base,
                                                     8u * 24u)) {
                        const uint64_t *tab = (const uint64_t *)(uintptr_t)base;
                        serial_write_string("[OS] [PF] null-call slot=");
                        serial_write_uint64(slot);
                        serial_write_string(" table@");
                        serial_write_uint64(base);
                        serial_write_string(":");
                        for (int i = 0; i < 24; ++i) {
                            serial_write_string(" ");
                            serial_write_uint64(tab[i]);
                        }
                        serial_write_string("\n");
                    }
                    /* One more hop: if the slot holds a trampoline that
                     * re-dispatches through a table in %r11 (`jmp *d32(%r11)`,
                     * how libGLdispatch reaches the current GL vendor), the
                     * NULL is in *that* table, not this one. Decode the
                     * displacement out of the trampoline and dump around it. */
                    uint64_t target =
                        (slot != 0u &&
                         process_user_buffer_is_valid((const void *)(uintptr_t)slot, 8u))
                            ? *(const uint64_t *)(uintptr_t)slot : 0u;
                    if (target != 0u &&
                        process_user_buffer_is_valid((const void *)(uintptr_t)target, 32u)) {
                        const uint8_t *c = (const uint8_t *)(uintptr_t)target;
                        /* The trampoline's own bytes. glvnd rewrites these in
                         * place at MakeCurrent, so "what the CPU executed" and
                         * "what the file says" can differ -- and a half-applied
                         * or reverted rewrite is itself a candidate cause. */
                        /* The trampoline's own GOTTPOFF operand and its
                         * neighbours. The word it loads has been seen holding
                         * another module's TLS offset; whether the words
                         * around it are also foreign is what separates a
                         * single stray store from a bulk copy over the .got. */
                        for (uint32_t i = 0; i + 7u <= 32u; ++i) {
                            if ((c[i] & 0xF8u) != 0x48u || c[i + 1] != 0x8Bu ||
                                (c[i + 2] & 0xC7u) != 0x05u) {
                                continue;
                            }
                            uint32_t gd = (uint32_t)c[i + 3] |
                                          ((uint32_t)c[i + 4] << 8) |
                                          ((uint32_t)c[i + 5] << 16) |
                                          ((uint32_t)c[i + 6] << 24);
                            uint64_t gslot = target + i + 7u +
                                             (uint64_t)(int64_t)(int32_t)gd;
                            uint64_t gbase = gslot >= 0x40u ? (gslot - 0x40u) : 0u;
                            if (gbase != 0u &&
                                process_user_buffer_is_valid(
                                    (const void *)(uintptr_t)gbase, 8u * 16u)) {
                                const uint64_t *g = (const uint64_t *)(uintptr_t)gbase;
                                serial_write_string("[OS] [PF] tpoff slot=");
                                serial_write_uint64(gslot);
                                serial_write_string(" got@");
                                serial_write_uint64(gbase);
                                serial_write_string(":");
                                for (int q = 0; q < 16; ++q) {
                                    serial_write_string(" ");
                                    serial_write_uint64(g[q]);
                                }
                                serial_write_string("\n");
                                /* The register holds one value, the memory may
                                 * hold another. Re-read after dropping this
                                 * CPU's translation and print the frame the
                                 * address resolves to: a value that only
                                 * changes across the invalidation, or a frame
                                 * that is not the one the loader wrote, means
                                 * the load was served from a stale mapping
                                 * rather than the word being corrupt. */
                                uint64_t gv1 = *(const uint64_t *)(uintptr_t)gslot;
                                hal_mmu_invalidate_tlb((uintptr_t)gslot);
                                uint64_t gv2 = *(const uint64_t *)(uintptr_t)gslot;
                                serial_write_string("[OS] [PF] tpoff pre-invlpg=");
                                serial_write_uint64(gv1);
                                serial_write_string(" post=");
                                serial_write_uint64(gv2);
                                serial_write_string(" phys=");
                                serial_write_uint64(paging_virt_to_phys(
                                    process_get_current_cr3(), gslot));
                                serial_write_string(" rax=");
                                serial_write_uint64(kregs[14]);
                                serial_write_string("\n");
                            }
                            break;
                        }
                        serial_write_string("[OS] [PF] trampoline@");
                        serial_write_uint64(target);
                        serial_write_string(":");
                        for (int q = 0; q < 32; ++q) {
                            serial_write_string(" ");
                            serial_write_uint32((uint32_t)c[q]);
                        }
                        serial_write_string("\n");
                        for (uint32_t i = 0; i + 7u <= 32u; ++i) {
                            if (c[i] != 0x41u || c[i + 1] != 0xFFu ||
                                c[i + 2] != 0xA3u) {
                                continue;
                            }
                            uint32_t d = (uint32_t)c[i + 3] |
                                         ((uint32_t)c[i + 4] << 8) |
                                         ((uint32_t)c[i + 5] << 16) |
                                         ((uint32_t)c[i + 6] << 24);
                            uint64_t tbl = kregs[4 /* r11 */] + (uint64_t)d;
                            /* Read the entry, drop the translation, read it
                             * again: a value that changes across the
                             * invalidation means this CPU was working from a
                             * stale mapping, which is a very different bug
                             * from a genuinely empty table slot. */
                            if (process_user_buffer_is_valid(
                                    (const void *)(uintptr_t)tbl, 8u)) {
                                uint64_t v1 = *(const uint64_t *)(uintptr_t)tbl;
                                hal_mmu_invalidate_tlb((uintptr_t)tbl);
                                uint64_t v2 = *(const uint64_t *)(uintptr_t)tbl;
                                serial_write_string("[OS] [PF] entry early=");
                                serial_write_uint64(nullcall_entry_early);
                                serial_write_string("@");
                                serial_write_uint64(nullcall_entry_addr);
                                serial_write_string(" pre-invlpg=");
                                serial_write_uint64(v1);
                                serial_write_string(" post=");
                                serial_write_uint64(v2);
                                serial_write_string("\n");
                                if (v2 != 0u &&
                                    process_user_buffer_is_valid(
                                        (const void *)(uintptr_t)v2, 16u)) {
                                    const uint8_t *tc =
                                        (const uint8_t *)(uintptr_t)v2;
                                    serial_write_string("[OS] [PF] entry code:");
                                    for (int q = 0; q < 16; ++q) {
                                        serial_write_string(" ");
                                        serial_write_uint32((uint32_t)tc[q]);
                                    }
                                    serial_write_string("\n");
                                }
                            }
                            /* The register file belongs to the LAST stub in
                             * the chain, not the first: a vendor's own
                             * entrypoint is itself `mov tpoff,%rax; mov
                             * %fs:(%rax),%r11; jmp *slot(%r11)`, so %r11 names
                             * the vendor's table while the displacement
                             * decoded above came from the caller's. Survey the
                             * whole table instead -- one NULL in an otherwise
                             * populated table and a table that is mostly NULL
                             * are very different bugs. */
                            {
                                uint64_t base_t = kregs[4 /* r11 */];
                                if (base_t >= 0x1000u &&
                                    process_user_buffer_is_valid(
                                        (const void *)(uintptr_t)base_t,
                                        8u * 2048u)) {
                                    const uint64_t *t =
                                        (const uint64_t *)(uintptr_t)base_t;
                                    uint32_t nulls = 0;
                                    serial_write_string("[OS] [PF] r11 table nulls:");
                                    for (uint32_t k = 0; k < 2048u; ++k) {
                                        if (t[k] != 0u) continue;
                                        if (nulls < 24u) {
                                            serial_write_string(" ");
                                            serial_write_uint32(k);
                                        }
                                        ++nulls;
                                    }
                                    serial_write_string(" total=");
                                    serial_write_uint32(nulls);
                                    serial_write_string("/2048\n");
                                }
                            }
                            uint64_t tb = tbl >= 0x20u ? (tbl - 0x20u) : 0u;
                            if (tb != 0u &&
                                process_user_buffer_is_valid(
                                    (const void *)(uintptr_t)tb, 8u * 12u)) {
                                const uint64_t *t2 = (const uint64_t *)(uintptr_t)tb;
                                serial_write_string("[OS] [PF] dispatch entry=");
                                serial_write_uint64(tbl);
                                serial_write_string(" around:");
                                for (int k = 0; k < 12; ++k) {
                                    serial_write_string(" ");
                                    serial_write_uint64(t2[k]);
                                }
                                serial_write_string("\n");
                            }
                            break;
                        }
                    }
                }
            }
        }

        {
            extern void process_record_page_fault(int32_t pid, uint64_t fault_addr,
                                                  uint64_t rip, uint32_t error_code,
                                                  int is_guard);
            process_record_page_fault(pid, cr2, rip, (uint32_t)error_code,
                                      process_is_guard_page_fault(cr2));
        }

        /* SIGSEGV (TODO_Chromium_LinuxABI.md 3.5): if the faulting
         * process installed a real handler (e.g. Chromium/Breakpad's
         * crash handler, or glibc's guard-page detector), deliver it
         * with an accurate ucontext/siginfo instead of unconditionally
         * killing the process - see process_signal_deliver_fault_now()
         * in ProcessManager_Create.c for why this needs the raw ISR
         * register frame rather than the usual syscall-boundary path.
         * kernel_rsp points at the ISR's post-alignment scratch qword;
         * the SAVE_REGS array itself starts 8 bytes further in. */
        {
            extern int process_signal_deliver_fault_now(
                int32_t pid, int32_t signum, uint64_t fault_addr,
                uint64_t *kernel_regs, uint64_t *cpu_frame);
            const int32_t sigsegv = 11;
            uint64_t *kernel_regs = (uint64_t *)(uintptr_t)(kernel_rsp + 8u);
            uint64_t *cpu_frame = (uint64_t *)(uintptr_t)frame_rsp;
            if (process_signal_deliver_fault_now(pid, sigsegv, cr2,
                                                 kernel_regs, cpu_frame)) {
                return 0; /* Resume via iretq, straight into the handler. */
            }
        }

        extern void process_debug_dump_pid(int32_t pid);
        process_debug_dump_pid(pid);

        /* Unhandled SIGSEGV: record the termination cause so wait4()/waitid()
         * in the Linux ABI report WIFSIGNALED / WTERMSIG == SIGSEGV. */
        extern void process_exit_current_signaled(int32_t signum);
        process_exit_current_signaled(11);

        serial_write_string("[OS] [PF] Idle-waiting for scheduler...\n");

        while (!process_run_next_on_current_cpu()) {
            hal_cpu_enable_interrupts();
            hal_cpu_halt();
        }

        return -1;
    } else {
        kernel_panic("PAGE_FAULT", "Page fault in kernel mode");
        serial_write_string("[OS] [PF] Page fault in kernel mode, halting\n");
    }

    panic_exception("page_fault", 14, error_code, rip, frame_rsp, rbp, cr2);
    return -1;
}

void unregister_interrupt_handler(uint16_t irq)
{
    if (irq < MAX_IRQS) {
        irq_routines[irq] = 0;
    }
}

void set_exception_handler(uint16_t exception_num, void (*handler)(void))
{
    if (exception_num < 32) {
        set_interrupt_handler(exception_num, handler);
    }
}

void set_irq_handler(uint16_t irq, void (*handler)(void))
{
    if (irq < 16) {
        set_interrupt_handler((uint16_t)(32 + irq), handler);
    }
}
