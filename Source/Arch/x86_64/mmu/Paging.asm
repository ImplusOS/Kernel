BITS 64
global paging_init_asm

SECTION .bss
align 4096
pml4:
    resq 512
pdpt:
    resq 512
pd:
    resq 512

SECTION .text
paging_init_asm:
    lea rax, [rel pdpt]
    or  rax, 0b111
    lea rdi, [rel pml4]
    mov [rdi], rax

    lea rax, [rel pd]
    or  rax, 0b111
    lea rdi, [rel pdpt]
    mov [rdi], rax

    mov rax, cr4
    or  rax, (1 << 5) | (1 << 9) | (1 << 10)
    mov cr4, rax

    xor rcx, rcx
    lea r8, [rel pd]
.fill_pd:
    mov rax, rcx
    shl rax, 21
    or  rax, (1 << 7) | 0b111
    mov [r8 + rcx*8], rax
    inc rcx
    cmp rcx, 512
    jne .fill_pd

    lea rax, [rel pml4]
    mov cr3, rax
    
    mov rax, cr0
    and al, 0xFB
    or  al, 0x02
    bts rax, 31
    mov cr0, rax

    ret

section .note.GNU-stack noalloc noexec nowrite progbits