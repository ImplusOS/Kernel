OUTPUT_FORMAT(elf64-x86-64)
ENTRY(kernel_main)

SECTIONS
{
    . = 0x01800000;

    _kernel_start = .;
    
    .text :
    {
        *(.text .text.*)
    }

    .rodata ALIGN(4K) : {
        *(.rodata*)
    }

    .data ALIGN(4K) : {
        *(.data*)
    }

    .got ALIGN(4K) : {
        *(.got .got.plt)
    }

    .rela.dyn ALIGN(4K) : {
        *(.rela.text .rela.text.*)
        *(.rela.rodata .rela.rodata.*)
        *(.rela.data .rela.data.*)
        *(.rela.got)
        *(.rela.bss .rela.bss.*)
        *(.rela.dyn)
    }

    .bss ALIGN(4K) : {
        *(COMMON)
        *(.bss*)
    }

    . = ALIGN(16);
    _stack_bottom = .;
    . += 0x8000;
    _stack_top = .;

    _kernel_end = .;

    /DISCARD/ : {
        *(.comment*)
        *(.note*)
        *(.debug*)
        *(.eh_frame*)
        *(.eh_frame_hdr*)
        *(.gcc_except_table*)
        *(.interp)
    }
}
