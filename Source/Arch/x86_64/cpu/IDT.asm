BITS 64

global load_idt
global isr_default
global isr_irq0
global isr_page_fault
global isr_double_fault
global isr_nmi
global isr_general_protection
global isr_machine_check
global isr_tlb_shootdown
global isr_resched
global isr_driver_table
global isr_exc_0
global isr_exc_1
global isr_exc_3
global isr_exc_4
global isr_exc_5
global isr_exc_6
global isr_exc_7
global isr_exc_10
global isr_exc_11
global isr_exc_12
global isr_exc_16
global isr_exc_17
global isr_exc_19

extern double_fault_handler
extern nmi_handler
extern general_protection_fault_handler
extern machine_check_handler
extern page_fault_handler
extern irq_handler
extern timer_note_irq_rip
extern smp_tlb_shootdown_handler
extern process_preempt_from_user_irq
extern cpu_exception_handler
extern resched_ipi_handler

SECTION .text

%macro SAVE_REGS 0
    push rax
    push rbx
    push rcx
    push rdx
    push rsi
    push rdi
    push rbp
    push r8
    push r9
    push r10
    push r11
    push r12
    push r13
    push r14
    push r15
%endmacro

%macro RESTORE_REGS 0
    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop rbp
    pop rdi
    pop rsi
    pop rdx
    pop rcx
    pop rbx
    pop rax
%endmacro

%macro SWAPGS_IF_USER_ENTER 1
    test qword [rsp + %1], 3
    jz %%skip_swapgs_enter
    swapgs
%%skip_swapgs_enter:
%endmacro

%macro SWAPGS_IF_USER_EXIT 1
    test qword [rsp + %1], 3
    jz %%skip_swapgs_exit
    swapgs
%%skip_swapgs_exit:
%endmacro

SECTION .text

isr_default:
    SWAPGS_IF_USER_ENTER 8
    SAVE_REGS

    mov al, 0x20
    out 0x20, al

    RESTORE_REGS
    SWAPGS_IF_USER_EXIT 8
    iretq

isr_irq0:
    SWAPGS_IF_USER_ENTER 8
    SAVE_REGS

    ; Where the tick interrupted (latency diagnostics, Core/timer/Timer.c).
    mov rdi, [rsp + 15 * 8]
    call timer_note_irq_rip

    mov rdi, 32
    call irq_handler

    ; Interrupted user code whose slice is up: let the scheduler switch
    ; (process_preempt_from_user_irq does not return if it does).
    ; [rsp + 15*8] is the CPU frame: rip, then cs.
    test qword [rsp + 16 * 8], 3
    jz .irq0_no_preempt
    mov rdi, rsp
    call process_preempt_from_user_irq
.irq0_no_preempt:

    RESTORE_REGS
    SWAPGS_IF_USER_EXIT 8
    iretq

%macro DRIVER_ISR 1
isr_driver_%1:
    SWAPGS_IF_USER_ENTER 8
    SAVE_REGS

    mov rdi, %1
    call irq_handler

    RESTORE_REGS
    SWAPGS_IF_USER_EXIT 8
    iretq
%endmacro

%assign driver_vector 64
%rep 64
    DRIVER_ISR driver_vector
%assign driver_vector driver_vector + 1
%endrep

; Synchronous CPU exceptions that user code can raise (divide error, int3,
; ud2, SSE/x87 faults, alignment check, ...). They used to share isr_default,
; which EOIs the PIC and IRETQs straight back to the faulting instruction --
; so a user `ud2` spun forever and a divide by zero hung the thread. Each stub
; normalises the stack to [vector, error code, CPU frame] and hands it to
; cpu_exception_handler(), which turns a user-mode exception into the Linux
; signal for it and panics on a kernel-mode one. It returns only when the
; task is to resume (possibly into a signal handler).
%macro EXC_NOERR 1
isr_exc_%1:
    push qword 0
    push qword %1
    jmp isr_exc_common
%endmacro

%macro EXC_ERR 1
isr_exc_%1:
    push qword %1
    jmp isr_exc_common
%endmacro

EXC_NOERR 0
EXC_NOERR 1
EXC_NOERR 3
EXC_NOERR 4
EXC_NOERR 5
EXC_NOERR 6
EXC_NOERR 7
EXC_ERR 10
EXC_ERR 11
EXC_ERR 12
EXC_NOERR 16
EXC_ERR 17
EXC_NOERR 19

isr_exc_common:
    ; [rsp]=vector [rsp+8]=error code [rsp+16]=rip [rsp+24]=cs
    SWAPGS_IF_USER_ENTER 24
    SAVE_REGS
    ; [0..14]=r15..rax [15]=vector [16]=error [17]=rip [18]=cs [19]=rflags
    ; [20]=rsp [21]=ss. RSP is 16-byte aligned here (5+2+15 qwords below
    ; the CPU's aligned entry point is 176 bytes).
    mov rdi, rsp
    mov rsi, [rsp + 15 * 8]
    lea rdx, [rsp + 16 * 8]
    call cpu_exception_handler
    RESTORE_REGS
    SWAPGS_IF_USER_EXIT 24
    add rsp, 16
    iretq

; Reschedule IPI (process_scheduler_kick_idle_cpu): wakes a halted CPU,
; and if it interrupted user code lets the scheduler switch right away.
isr_resched:
    SWAPGS_IF_USER_ENTER 8
    SAVE_REGS
    call resched_ipi_handler
    test qword [rsp + 16 * 8], 3
    jz .resched_kernel
    mov rdi, rsp
    call process_preempt_from_user_irq
.resched_kernel:
    RESTORE_REGS
    SWAPGS_IF_USER_EXIT 8
    iretq

isr_tlb_shootdown:
    SWAPGS_IF_USER_ENTER 8
    SAVE_REGS

    call smp_tlb_shootdown_handler

    RESTORE_REGS
    SWAPGS_IF_USER_EXIT 8
    iretq

isr_page_fault:
    cli
    SWAPGS_IF_USER_ENTER 16
    SAVE_REGS
    sub rsp, 8

    mov rdi, [rsp + 16 * 8]
    mov rsi, [rsp + 17 * 8]
    lea rdx, [rsp + 16 * 8]
    mov rcx, rsp
    mov r8, cr2
    mov r9, rbp
    
    call page_fault_handler
    
    add rsp, 8
    test eax, eax
    jz .pf_resume
    
.pf_hang:
    sti
    hlt
    jmp .pf_hang
    
.pf_resume:
    RESTORE_REGS
    add rsp, 8
    SWAPGS_IF_USER_EXIT 8
    iretq

isr_double_fault:
    cli
    SWAPGS_IF_USER_ENTER 16
    mov rdi, [rsp]
    mov rsi, [rsp + 8]
    mov rdx, rsp
    mov rcx, rbp
    and rsp, ~0xF
    call double_fault_handler
.df_hang:
    sti
    hlt
    jmp .df_hang

isr_nmi:
    cli
    SWAPGS_IF_USER_ENTER 8
    mov rdi, [rsp]
    mov rsi, rsp
    mov rdx, rbp
    and rsp, ~0xF
    call nmi_handler
.nmi_hang:
    sti
    hlt
    jmp .nmi_hang

isr_general_protection:
    cli
    SWAPGS_IF_USER_ENTER 16
    SAVE_REGS
    sub rsp, 8

    ; gpregs (from rdi): [0]=r15 [1]=r14 ... [14]=rax
    ; CPU frame follows: [15]=err [16]=rip [17]=cs [18]=rflags
    ;                    [19]=user_rsp [20]=user_ss
    mov rdi, rsp
    add rdi, 8                       ; skip pad -> r15
    mov rsi, [rsp + 16 * 8]          ; error code
    mov rdx, [rsp + 17 * 8]          ; rip
    lea rcx, [rsp + 16 * 8]          ; CPU exception frame ptr
    mov r8, rbp
    call general_protection_fault_handler
    ; Returns only when the fault was turned into a signal: resume (into the
    ; handler the C side just set up).
    add rsp, 8
    RESTORE_REGS
    add rsp, 8                       ; error code
    SWAPGS_IF_USER_EXIT 8
    iretq

isr_machine_check:
    cli
    SWAPGS_IF_USER_ENTER 8
    mov rdi, [rsp]
    mov rsi, rsp
    mov rdx, rbp
    and rsp, ~0xF
    call machine_check_handler
.mce_hang:
    sti
    hlt
    jmp .mce_hang

load_idt:
    lidt [rdi]
    ret

SECTION .rodata
align 8
isr_driver_table:
%assign driver_vector 64
%rep 64
    dq isr_driver_%+driver_vector
%assign driver_vector driver_vector + 1
%endrep

section .note.GNU-stack noalloc noexec nowrite progbits
