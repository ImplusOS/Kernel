BITS 64

global gdt_flush
global tss_flush

%define GDT_KERNEL_CODE 0x08
%define GDT_KERNEL_DATA 0x10

SECTION .text

gdt_flush:
    lgdt [rdi]

    mov ax, GDT_KERNEL_DATA
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax

    push GDT_KERNEL_CODE
    lea rax, [rel gdt_flush_ret]
    push rax
    retfq

gdt_flush_ret:
    ret

tss_flush:
    mov ax, di
    ltr ax
    ret
section .note.GNU-stack noalloc noexec nowrite progbits
