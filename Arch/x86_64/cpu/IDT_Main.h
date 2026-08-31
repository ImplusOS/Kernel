#ifndef IDT_MAIN_H
#define IDT_MAIN_H

#include <stdint.h>

#define IDT_ENTRIES 256

typedef struct {
    uint16_t offset_low;
    uint16_t selector;
    uint8_t ist;
    uint8_t type_attr;
    uint16_t offset_mid;
    uint32_t offset_high;
    uint32_t zero;
} __attribute__((packed)) IDT_Entry;

typedef struct {
    uint16_t limit;
    uint64_t base;
} __attribute__((packed)) IDT_Ptr;

typedef void(*isr_t)(void);

void register_interrupt_handler(uint16_t irq, isr_t handler);
void unregister_interrupt_handler(uint16_t irq);
void set_exception_handler(uint16_t exception_num, void (*handler)(void));
void set_irq_handler(uint16_t irq, void (*handler)(void));
void init_idt(void);
void init_idt_per_cpu(void);
void set_interrupt_handler(uint16_t n, void (*handler)(void));
void set_interrupt_handler_with_ist(uint16_t n, void (*handler)(void), uint8_t ist);
IDT_Ptr *idt_get_ptr(void);

extern void load_idt(IDT_Ptr*);

#endif
