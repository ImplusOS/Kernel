BITS 64
section .text
global syscall_entry
global syscall_enter_user_from_frame
extern syscall_dispatch

syscall_entry:
    swapgs
    
    mov [gs:0], rsp
    mov rsp, [gs:8]
    
    push r11
    push rcx
    push rbp
    push rbx
    push r15
    push r14
    push r13
    push r12
    push r10
    push r9
    push r8
    push rdi
    push rsi
    push rdx
    push rax
    
    ; Marshal to syscall_dispatch(saved_rsp, num, a1, a2, a3, a4, a5, a6).
    ; Frame offsets after the pushes above: [0]=rax [8]=rdx [16]=rsi [24]=rdi
    ; [32]=r8 [40]=r9 [48]=r10. Linux passes the 4th arg in r10, not rcx.
    mov rdi, rsp            ; p1 saved_rsp
    mov rsi, rax            ; p2 num
    mov rdx, [rsp + 24]     ; p3 a1  = user rdi
    mov rcx, [rsp + 16]     ; p4 a2  = user rsi
    mov r8,  [rsp + 8]      ; p5 a3  = user rdx
    mov r9,  [rsp + 48]     ; p6 a4  = user r10

    ; p7 (a5) and p8 (a6) go on the stack. The 8-byte alignment pad MUST sit
    ; below them (pushed first) or it shifts the args and the callee reads a5
    ; from the pad slot -- that bug fed garbage as mmap()'s fd (EBADF) and
    ; broke every dynamic .so load. SysV: at the call, rsp must be 16-aligned
    ; (it is 16k+8 here after 15 pushes), and the callee then finds a5 at
    ; [rsp+8] and a6 at [rsp+16].
    mov rax, [rsp + 40]     ; a6 = user r9
    mov r10, [rsp + 32]     ; a5 = user r8  (r10's own value already copied to r9)

    sub rsp, 8             ; realign to 16 for the call
    push rax               ; a6
    push r10               ; a5

    call syscall_dispatch
    
    mov rsp, rax
    
    pop rax
    pop rdx
    pop rsi
    pop rdi
    pop r8
    pop r9
    pop r10
    pop r12
    pop r13
    pop r14
    pop r15
    pop rbx
    pop rbp
    pop rcx
    pop r11

    mov rsp, [gs:0]
    
    swapgs
    o64 sysret

syscall_enter_user_from_frame:
    cli
    cld
    mov rbx, rdi

    mov rax, [rbx + (13 * 8)]
    mov rdx, [rbx + (14 * 8)]

    push qword (0x20 | 3)
    push rsi
    push rdx
    push qword (0x28 | 3)
    push rax

    mov rax, [rbx + (0 * 8)]
    mov rdx, [rbx + (1 * 8)]
    mov rsi, [rbx + (2 * 8)]
    mov rdi, [rbx + (3 * 8)]
    mov r8,  [rbx + (4 * 8)]
    mov r9,  [rbx + (5 * 8)]
    mov r10, [rbx + (6 * 8)]
    mov r12, [rbx + (7 * 8)]
    mov r13, [rbx + (8 * 8)]
    mov r14, [rbx + (9 * 8)]
    mov r15, [rbx + (10 * 8)]
    mov rbp, [rbx + (12 * 8)]
    mov rcx, [rbx + (13 * 8)]
    mov r11, [rbx + (14 * 8)]
    mov rbx, [rbx + (11 * 8)]
    
    swapgs

    iretq

section .note.GNU-stack noalloc noexec nowrite progbits