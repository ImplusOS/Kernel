.section .text.arm64_vectors, "ax"
.align 11
.global arm64_exception_vector_table
.type arm64_exception_vector_table, %function

.macro vector_entry label
    .align 7
    b \label
.endm

arm64_exception_vector_table:
    vector_entry arm64_handler_0
    vector_entry arm64_handler_1
    vector_entry arm64_handler_2
    vector_entry arm64_handler_3

    vector_entry arm64_handler_4
    vector_entry arm64_handler_5
    vector_entry arm64_handler_6
    vector_entry arm64_handler_7

    vector_entry arm64_handler_8
    vector_entry arm64_handler_9
    vector_entry arm64_handler_10
    vector_entry arm64_handler_11

    vector_entry arm64_handler_12
    vector_entry arm64_handler_13
    vector_entry arm64_handler_14
    vector_entry arm64_handler_15

.macro save_all
    sub sp, sp, #(36 * 8)
    stp x0, x1, [sp, #(0 * 8)]
    stp x2, x3, [sp, #(2 * 8)]
    stp x4, x5, [sp, #(4 * 8)]
    stp x6, x7, [sp, #(6 * 8)]
    stp x8, x9, [sp, #(8 * 8)]
    stp x10, x11, [sp, #(10 * 8)]
    stp x12, x13, [sp, #(12 * 8)]
    stp x14, x15, [sp, #(14 * 8)]
    stp x16, x17, [sp, #(16 * 8)]
    stp x18, x19, [sp, #(18 * 8)]
    stp x20, x21, [sp, #(20 * 8)]
    stp x22, x23, [sp, #(22 * 8)]
    stp x24, x25, [sp, #(24 * 8)]
    stp x26, x27, [sp, #(26 * 8)]
    stp x28, x29, [sp, #(28 * 8)]
    str x30, [sp, #(30 * 8)]
    
    mrs x0, sp_el0
    str x0, [sp, #(31 * 8)]
    mrs x0, elr_el1
    str x0, [sp, #(32 * 8)]
    mrs x0, spsr_el1
    str x0, [sp, #(33 * 8)]
    mrs x0, esr_el1
    str x0, [sp, #(34 * 8)]
    mrs x0, far_el1
    str x0, [sp, #(35 * 8)]
.endm

.macro restore_all
    ldr x0, [sp, #(33 * 8)]
    msr spsr_el1, x0
    ldr x0, [sp, #(32 * 8)]
    msr elr_el1, x0
    ldr x0, [sp, #(31 * 8)]
    msr sp_el0, x0
    
    ldp x0, x1, [sp, #(0 * 8)]
    ldp x2, x3, [sp, #(2 * 8)]
    ldp x4, x5, [sp, #(4 * 8)]
    ldp x6, x7, [sp, #(6 * 8)]
    ldp x8, x9, [sp, #(8 * 8)]
    ldp x10, x11, [sp, #(10 * 8)]
    ldp x12, x13, [sp, #(12 * 8)]
    ldp x14, x15, [sp, #(14 * 8)]
    ldp x16, x17, [sp, #(16 * 8)]
    ldp x18, x19, [sp, #(18 * 8)]
    ldp x20, x21, [sp, #(20 * 8)]
    ldp x22, x23, [sp, #(22 * 8)]
    ldp x24, x25, [sp, #(24 * 8)]
    ldp x26, x27, [sp, #(26 * 8)]
    ldp x28, x29, [sp, #(28 * 8)]
    ldr x30, [sp, #(30 * 8)]
    add sp, sp, #(36 * 8)
    eret
.endm

.global arm64_restore_exception_frame
.type arm64_restore_exception_frame, %function
arm64_restore_exception_frame:
    restore_all

.global arm64_enter_user_from_frame
.type arm64_enter_user_from_frame, %function
arm64_enter_user_from_frame:
    mov sp, x0
    b arm64_restore_exception_frame

.macro exception_handler id
arm64_handler_\id:
    save_all
    mov x19, sp
    mov x0, sp
    mov x1, #\id
    bl arm64_exception_dispatch
    cbnz x0, 1f
    mov x0, x19
1:
    mov sp, x0
    b arm64_restore_exception_frame
.endm

exception_handler 0
exception_handler 1
exception_handler 2
exception_handler 3
exception_handler 4
exception_handler 5
exception_handler 6
exception_handler 7
exception_handler 8
exception_handler 9
exception_handler 10
exception_handler 11
exception_handler 12
exception_handler 13
exception_handler 14
exception_handler 15
