#pragma once

#define VECTOR_TIMER 30

typedef void (*interrupt_handler_t)(void);

void init_idt(void);
void init_idt_per_cpu(void);
void register_interrupt_handler(int vector, interrupt_handler_t handler);

