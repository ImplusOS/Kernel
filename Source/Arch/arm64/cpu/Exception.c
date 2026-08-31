#include "Exception.h"
#include "CPU.h"
#include "IDT_Main.h"
#include "Arch/arm64/interrupt/GIC.h"
#include "Core/syscall/Syscall_Main.h"
#include "Debug/panic/Panic.h"
#include "Debug/serial/Serial.h"
#include "Drivers/Module/InterruptManager.h"

#define ESR_EC_SHIFT 26
#define ESR_EC_MASK  0x3FU
#define ESR_EC_SVC64 0x15U

static interrupt_handler_t g_interrupt_handlers[1024];

void arm64_exception_init(void)
{
    arm64_set_exception_vector((void *)arm64_exception_vector_table);
}

static void handle_irq(arm64_exception_frame_t *frame) {
    (void)frame;
    uint32_t iar = arm64_gic_read_iar();
    uint32_t irq = iar & 0x3FFu;

    if (irq < 1020) {
        interrupt_manager_dispatch(irq);
        if (irq < 1024 && g_interrupt_handlers[irq]) {
            g_interrupt_handlers[irq]();
        }
    }

    arm64_gic_eoi(iar);
}

arm64_exception_frame_t *arm64_exception_dispatch(arm64_exception_frame_t *frame, uint64_t type)
{
    if (type == 1 || type == 5 || type == 9 || type == 13) {
        handle_irq(frame);
        return frame;
    }

    if (type == 0 || type == 4 || type == 8 || type == 12) {
        uint32_t ec = (uint32_t)((frame->esr_el1 >> ESR_EC_SHIFT) & ESR_EC_MASK);
        if (ec == ESR_EC_SVC64) {
            uint64_t nr = frame->x[8];
            syscall_set_user_rsp(frame->sp_el0);
            uint64_t next_frame_addr =
                syscall_dispatch((uint64_t)(uintptr_t)frame, nr,
                                 frame->x[0], frame->x[1], frame->x[2],
                                 frame->x[3], frame->x[4], frame->x[5]);
            arm64_exception_frame_t *next_frame =
                (arm64_exception_frame_t *)(uintptr_t)next_frame_addr;
            if (next_frame == NULL) {
                next_frame = frame;
            }
            next_frame->sp_el0 = syscall_get_user_rsp();
            return next_frame;
        }
    }
    
    serial_write_string("\nUnhandled arm64 exception! Type: ");
    serial_write_uint64(type);
    serial_write_string(" ESR: ");
    serial_write_uint64(frame->esr_el1);
    serial_write_string(" FAR: ");
    serial_write_uint64(frame->far_el1);
    serial_write_string(" ELR: ");
    serial_write_uint64(frame->elr_el1);
    serial_write_string("\n");
    kernel_panic("Unhandled arm64 exception", "arm64_exception_dispatch");
    return frame;
}

void init_idt(void)
{
    arm64_exception_init();
}

void init_idt_per_cpu(void)
{
    arm64_exception_init();
}

void register_interrupt_handler(int vector, interrupt_handler_t handler)
{
    if (vector >= 0 && vector < 1024) {
        g_interrupt_handlers[vector] = handler;
    }
}

void init_gdt(void)
{
}

void gdt_set_kernel_rsp0(uint64_t rsp0)
{
    (void)rsp0;
}
