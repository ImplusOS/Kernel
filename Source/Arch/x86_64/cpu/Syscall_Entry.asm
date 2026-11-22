BITS 64
section .text
global syscall_entry
global syscall_enter_user_from_frame
global isr_int80
extern syscall_dispatch
extern process_scheduler_clear_leaving_pid

; Per-CPU syscall state (syscall_cpu_state_t in Core/syscall/Syscall_Init.c),
; reached through the kernel GS base.
%define PERCPU_USER_RSP      0
%define PERCPU_KERNEL_RSP    8
%define PERCPU_FULL_RESTORE  16
%define PERCPU_FULL_RCX      24
%define PERCPU_FULL_R11      32
%define PERCPU_USER_CS       48
%define PERCPU_USER_SS       56
%define PERCPU_INT80         64

; Selectors (kernel/config.h): 64-bit user code, and the user data segment a
; 32-bit (Linux i386) task also needs in DS/ES -- in compatibility mode a null
; data segment faults on the first memory access.
%define USER_CS64   (0x28 | 3)
%define USER_DS     (0x20 | 3)

syscall_entry:
    swapgs

    mov [gs:PERCPU_USER_RSP], rsp
    mov rsp, [gs:PERCPU_KERNEL_RSP]

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
    jmp syscall_exit_common

; Linux i386 system call: `int $0x80` from 32-bit compatibility mode (an
; interrupt gate with DPL 3, so IF is clear here). Number in EAX, arguments in
; EBX, ECX, EDX, ESI, EDI, EBP, result back in EAX, and every other register
; preserved -- ECX included, which is why this always leaves through the IRETQ
; ("full restore") exit with the real RCX/R11 handed back.
;
; The frame is built exactly where syscall_entry builds it, at the top of the
; task's kernel stack (TSS.RSP0 and PERCPU_KERNEL_RSP are the same address):
; fork, signal delivery and preemption all find a task's registers there. The
; CPU pushed its own frame into those same bytes, so it is read out first.
isr_int80:
    swapgs
    cld
    mov [gs:PERCPU_FULL_RCX], rcx
    mov [gs:PERCPU_FULL_R11], r11
    mov rcx, [rsp + 0]          ; user eip
    mov r11, [rsp + 16]         ; user eflags
    push rax
    mov rax, [rsp + 8 + 24]     ; user esp
    mov [gs:PERCPU_USER_RSP], rax
    pop rax
    mov rsp, [gs:PERCPU_KERNEL_RSP]

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

    mov qword [gs:PERCPU_FULL_RESTORE], 1
    mov qword [gs:PERCPU_INT80], 1

    ; syscall_dispatch(saved_rsp, eax, ebx, ecx, edx, esi, edi, ebp), every
    ; value zero-extended: the upper halves of the registers mean nothing to a
    ; 32-bit program.
    mov rdi, rsp
    mov esi, eax                    ; num
    mov edx, ebx                    ; a1 = ebx
    mov ecx, [gs:PERCPU_FULL_RCX]   ; a2 = ecx
    mov r8d, [rsp + 1 * 8]          ; a3 = edx
    mov r9d, [rsp + 2 * 8]          ; a4 = esi
    mov eax, [rsp + 12 * 8]         ; a6 = ebp
    mov r10d, [rsp + 3 * 8]         ; a5 = edi

    sub rsp, 8
    push rax                        ; a6
    push r10                        ; a5

    call syscall_dispatch
    ; fall through

syscall_exit_common:
    mov rsp, rax

    ; Only now is the previous task's kernel stack abandoned: everything up to
    ; here, including the return out of syscall_dispatch, ran on it. The
    ; scheduler marked that task "leaving" on this CPU so no other CPU picks it
    ; meanwhile (scheduler_pid_running_on_other_cpu); drop the marker now so it
    ; does not keep the task off every other CPU until this one schedules again.
    ; Safe to call C here: interrupts are off (IA32_FMASK clears IF), GS is the
    ; kernel's, nothing below RSP on this stack is in use, and every register
    ; the call may clobber is reloaded from the frame by the pops that follow.
    sub rsp, 8
    call process_scheduler_clear_leaving_pid
    add rsp, 8

    ; The task being resumed may need RCX and R11 back as well (it was
    ; preempted in its own code, or is returning from a signal handler):
    ; SYSRET cannot give it that, IRETQ can. See syscall_set_full_restore().
    ; A 32-bit task always goes that way: SYSRET would return it to 64-bit
    ; code, and its ECX is a preserved register.
    cmp qword [gs:PERCPU_FULL_RESTORE], 0
    jne .full_restore
    cmp qword [gs:PERCPU_USER_CS], USER_CS64
    jne .full_restore

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

    mov rsp, [gs:PERCPU_USER_RSP]

    swapgs
    o64 sysret

; Frame at RSP: [0]=rax [1]=rdx [2]=rsi [3]=rdi [4]=r8 [5]=r9 [6]=r10
; [7]=r12 [8]=r13 [9]=r14 [10]=r15 [11]=rbx [12]=rbp [13]=rip [14]=rflags.
; Slots 10..14 are reused in place as the IRETQ frame once their contents
; are in registers.
.full_restore:
    mov qword [gs:PERCPU_FULL_RESTORE], 0
    mov r15, [rsp + 10 * 8]
    mov rbx, [rsp + 11 * 8]
    mov rbp, [rsp + 12 * 8]
    mov rcx, [rsp + 13 * 8]         ; rip
    mov r11, [rsp + 14 * 8]         ; rflags
    mov rax, [gs:PERCPU_USER_RSP]   ; user rsp
    mov [rsp + 10 * 8], rcx
    mov rdx, [gs:PERCPU_USER_CS]
    mov [rsp + 11 * 8], rdx         ; user cs (64-bit or 32-bit, per task)
    mov [rsp + 12 * 8], r11
    mov [rsp + 13 * 8], rax
    mov rdx, [gs:PERCPU_USER_SS]
    mov [rsp + 14 * 8], rdx         ; user ss
    cmp qword [gs:PERCPU_USER_CS], USER_CS64
    je .full_restore_regs
    mov edx, USER_DS
    mov ds, dx
    mov es, dx
.full_restore_regs:
    mov rax, [rsp + 0 * 8]
    mov rdx, [rsp + 1 * 8]
    mov rsi, [rsp + 2 * 8]
    mov rdi, [rsp + 3 * 8]
    mov r8,  [rsp + 4 * 8]
    mov r9,  [rsp + 5 * 8]
    mov r10, [rsp + 6 * 8]
    mov r12, [rsp + 7 * 8]
    mov r13, [rsp + 8 * 8]
    mov r14, [rsp + 9 * 8]
    mov rcx, [gs:PERCPU_FULL_RCX]
    mov r11, [gs:PERCPU_FULL_R11]
    add rsp, 10 * 8
    swapgs
    iretq

syscall_enter_user_from_frame:
    cli
    cld
    mov rbx, rdi
    mov r12, rsi

    ; RSP is on the next task's kernel stack now; the previous task's stack
    ; (which the caller was running on) is no longer in use, so it may be
    ; picked by another CPU -- same as syscall_entry does after its switch.
    mov rsp, rbx
    and rsp, ~0xF
    call process_scheduler_clear_leaving_pid
    mov rsp, rbx
    mov rsi, r12

    mov rax, [rbx + (13 * 8)]
    mov rdx, [rbx + (14 * 8)]

    push qword [gs:PERCPU_USER_SS]
    push rsi
    push rdx
    push qword [gs:PERCPU_USER_CS]
    push rax

    cmp qword [gs:PERCPU_USER_CS], USER_CS64
    je .enter_regs
    mov eax, USER_DS
    mov ds, ax
    mov es, ax
.enter_regs:
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
    cmp qword [gs:PERCPU_FULL_RESTORE], 0
    je .enter_plain
    mov qword [gs:PERCPU_FULL_RESTORE], 0
    mov rcx, [gs:PERCPU_FULL_RCX]
    mov r11, [gs:PERCPU_FULL_R11]
.enter_plain:
    mov rbx, [rbx + (11 * 8)]

    swapgs

    iretq

section .note.GNU-stack noalloc noexec nowrite progbits
