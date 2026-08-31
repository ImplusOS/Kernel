#pragma once

#include "DriverBinary.h"

#include <stdint.h>

int32_t interrupt_manager_allocate_vector(void);
void interrupt_manager_free_vector(uint32_t vector);
int32_t interrupt_manager_register(uint32_t vector,
                                   driver_irq_handler_t handler,
                                   void *context);
void interrupt_manager_unregister(uint32_t vector);
void interrupt_manager_dispatch(uint32_t vector);
