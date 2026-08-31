section .smp_trampoline progbits alloc exec nowrite

BITS 16

AP_BASE   equ 0x8000
AP_SHARED equ 0x9000

AP_CR3    equ 0x9000
AP_ENTRY  equ 0x9008
AP_STACK  equ 0x9010
AP_GDTR   equ 0x9018
AP_IDTR   equ 0x9022

global smp_trampoline_start
global smp_trampoline_end

smp_trampoline_start:

entry16:
    cli
    cld
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov ss, ax
    xor sp, sp

    lgdt [AP_BASE + (gdt16_ptr - smp_trampoline_start)]

    mov eax, cr0
    or  al, 1
    mov cr0, eax

    jmp 0x08:(AP_BASE + (entry32 - smp_trampoline_start))

times (0x20 - ($ - smp_trampoline_start)) db 0x90

BITS 32

entry32:
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov fs, ax
    mov gs, ax

    mov eax, cr4
    or  eax, (1 << 5) | (1 << 9) | (1 << 10)
    mov cr4, eax

    mov eax, dword [AP_CR3]
    mov cr3, eax

    mov ecx, 0xC0000080
    rdmsr
    or  eax, (1 << 8)
    wrmsr

    mov eax, cr0
    or  eax, (1 << 31)
    mov cr0, eax

    jmp 0x18:(AP_BASE + (entry64 - smp_trampoline_start))

align 0x20, db 0x90

BITS 64

entry64:
    mov ax, 0x20
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov fs, ax
    mov gs, ax

    lgdt [AP_GDTR]

    mov rsp, qword [AP_STACK]

    push qword 0x08
    push qword (AP_BASE + (entry64_kgdt - smp_trampoline_start))
    retfq

entry64_kgdt:
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov ss, ax
    xor ax, ax
    mov fs, ax
    mov gs, ax

    lidt [AP_IDTR]

    mov rsp, qword [AP_STACK]
    xor rbp, rbp

    mov rax, qword [AP_ENTRY]
    jmp rax

.halt:
    cli
    hlt
    jmp .halt

align 8
gdt16_table:
    dq 0x0000000000000000
    dq 0x00CF9A000000FFFF
    dq 0x00CF92000000FFFF
    dq 0x00AF9A000000FFFF
    dq 0x00CF92000000FFFF

gdt16_ptr:
    dw (gdt16_ptr - gdt16_table - 1)
    dd (AP_BASE + (gdt16_table - smp_trampoline_start))

smp_trampoline_end:

section .note.GNU-stack noalloc noexec nowrite progbits