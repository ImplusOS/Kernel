BITS 64
section .text

global vmx_vmlaunch_asm
global vmx_vmresume_asm
extern g_vmx_current_guest_regs

vmx_vmlaunch_asm:
    pushfq
    cli

    push rbx
    push rbp
    push r12
    push r13
    push r14
    push r15

    mov rax, rsp
    mov rdx, 0x6C14
    vmwrite rdx, rax

    mov rax, [rdi +   0]
    mov rbx, [rdi +   8]
    mov rcx, [rdi +  16]
    mov rdx, [rdi +  24]
    mov rsi, [rdi +  32]
    mov rbp, [rdi +  48]
    mov r8,  [rdi +  64]
    mov r9,  [rdi +  72]
    mov r10, [rdi +  80]
    mov r11, [rdi +  88]
    mov r12, [rdi +  96]
    mov r13, [rdi + 104]
    mov r14, [rdi + 112]
    mov r15, [rdi + 120]

    mov rdi, [rdi +  40]

    vmlaunch

    jmp vm_fail

vmx_vmresume_asm:
    pushfq
    cli

    push rbx
    push rbp
    push r12
    push r13
    push r14
    push r15

    mov rax, rsp
    mov rdx, 0x6C14
    vmwrite rdx, rax
    
    mov rax, [rdi +   0]
    mov rbx, [rdi +   8]
    mov rcx, [rdi +  16]
    mov rdx, [rdi +  24]
    mov rsi, [rdi +  32]
    mov rbp, [rdi +  48]
    mov r8,  [rdi +  64]
    mov r9,  [rdi +  72]
    mov r10, [rdi +  80]
    mov r11, [rdi +  88]
    mov r12, [rdi +  96]
    mov r13, [rdi + 104]
    mov r14, [rdi + 112]
    mov r15, [rdi + 120]
    mov rdi, [rdi +  40]

    vmresume

    jmp vm_fail

global vmx_vmexit_handler
vmx_vmexit_handler:
    push rdi

    mov rdi, [rel g_vmx_current_guest_regs]

    mov [rdi +   0], rax
    mov [rdi +   8], rbx
    mov [rdi +  16], rcx
    mov [rdi +  24], rdx
    mov [rdi +  32], rsi
    
    pop rax
    mov [rdi +  40], rax
    mov [rdi +  48], rbp

    mov [rdi +  64], r8
    mov [rdi +  72], r9
    mov [rdi +  80], r10
    mov [rdi +  88], r11
    mov [rdi +  96], r12
    mov [rdi + 104], r13
    mov [rdi + 112], r14
    mov [rdi + 120], r15

    pop r15
    pop r14
    pop r13
    pop r12
    pop rbp
    pop rbx
    popfq

    xor eax, eax
    ret

vm_fail:
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbp
    pop rbx
    popfq

    jc vm_fail_invalid

    mov eax, 2
    ret

vm_fail_invalid:
    mov eax, 1
    ret

section .note.GNU-stack noalloc noexec nowrite progbits
