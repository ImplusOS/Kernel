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
global isr_driver_table

extern double_fault_handler
extern nmi_handler
extern general_protection_fault_handler
extern machine_check_handler
extern page_fault_handler
extern irq_handler
extern smp_tlb_shootdown_handler

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
    
    mov rdi, 32
    call irq_handler

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
    mov rdi, [rsp]
    mov rsi,[rsp + 8]
    mov rdx, rsp
    mov rcx, rbp
    and rsp, ~0xF
    call general_protection_fault_handler
.gp_hang:
    sti
    hlt
    jmp .gp_hang

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
