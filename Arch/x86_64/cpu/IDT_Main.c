#include "IDT_Main.h"
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
#include "Debug/panic/Panic.h"
#include "Drivers/Module/InterruptManager.h"

#include <stdint.h>

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

void general_protection_fault_handler(uint64_t error_code,
                                      uint64_t rip,
                                      uint64_t rsp,
                                      uint64_t rbp)
{
    /* `rsp` points at the CPU-pushed exception frame: [0]=error_code [1]=rip
     * [2]=cs [3]=rflags [4]=user_rsp [5]=user_ss. A #GP that came from CPL 3
     * is the userland process's problem, not the kernel's -- terminate that
     * process and keep running. Foreign Linux binaries reach here a lot:
     * Chromium's IMMEDIATE_CRASH()/failed CHECK() is `int3; ud2`, and a
     * userland `int3` traps the DPL-0 #BP gate as #GP with
     * error = (3<<3)|2 = 0x1A; privileged instructions and non-canonical
     * accesses land here too. */
    const uint64_t *frame = (const uint64_t *)(uintptr_t)rsp;
    int from_user = (rsp != 0) && ((frame[2] & 0x3ULL) == 0x3ULL);
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
        {
            uint64_t urs = frame[4];
            if (urs >= 0x1000u && (urs & 7u) == 0u &&
                process_user_buffer_is_valid((const void *)(uintptr_t)urs,
                                             8u * PANIC_STACK_DUMP_QWORDS)) {
                const uint64_t *ustack = (const uint64_t *)(uintptr_t)urs;
                serial_write_string("[OS] [#GP] user stack:");
                for (int i = 0; i < PANIC_STACK_DUMP_QWORDS; ++i) {
                    serial_write_string(" ");
                    serial_write_uint64(ustack[i]);
                }
                serial_write_string("\n");
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

    panic_exception("general_protection", 13, error_code, rip, rsp, rbp, 0);
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

    serial_write_string("[OS] [PF] Page fault\n");
    serial_write_string("[OS] [PF] CR2: ");
    serial_write_uint64(cr2);
    serial_write_string("\n");
    serial_write_string("[OS] [PF] RIP: ");
    serial_write_uint64(rip);
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
