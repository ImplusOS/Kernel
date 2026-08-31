ENTRY(kernel_main)

SECTIONS
{
    . = 0x40080000;
    
    _kernel_start = .;
    
    .text : ALIGN(4096) {
        *(.text.arm64_vectors)
        *(.text*)
    }
    .rodata : ALIGN(4096) { *(.rodata*) }
    .data : ALIGN(4096) { *(.data*) }
    .bss : ALIGN(4096) {
        *(COMMON)
        *(.bss*)
    }
    
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