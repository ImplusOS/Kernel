#include "InterruptManager.h"

#include "Core/sync/Spinlock.h"

#include <stddef.h>

#define DRIVER_VECTOR_FIRST 64u
#define DRIVER_VECTOR_LAST  127u
#define DRIVER_VECTOR_COUNT (DRIVER_VECTOR_LAST - DRIVER_VECTOR_FIRST + 1u)

typedef struct {
    driver_irq_handler_t handler;
    void *context;
    uint8_t allocated;
} interrupt_slot_t;

static interrupt_slot_t g_slots[DRIVER_VECTOR_COUNT];
static spinlock_t g_lock = {0};

static interrupt_slot_t *interrupt_slot(uint32_t vector)
{
    if (vector < DRIVER_VECTOR_FIRST || vector > DRIVER_VECTOR_LAST) {
        return NULL;
    }
    return &g_slots[vector - DRIVER_VECTOR_FIRST];
}

int32_t interrupt_manager_allocate_vector(void)
{
    spinlock_lock(&g_lock);
    for (uint32_t i = 0u; i < DRIVER_VECTOR_COUNT; ++i) {
        if (g_slots[i].allocated == 0u) {
            g_slots[i].allocated = 1u;
            spinlock_unlock(&g_lock);
            return (int32_t)(DRIVER_VECTOR_FIRST + i);
        }
    }
    spinlock_unlock(&g_lock);
    return -1;
}

void interrupt_manager_free_vector(uint32_t vector)
{
    interrupt_slot_t *slot = interrupt_slot(vector);
    if (slot == NULL) {
        return;
    }
    spinlock_lock(&g_lock);
    slot->allocated = 0u;
    slot->handler = NULL;
    slot->context = NULL;
    spinlock_unlock(&g_lock);
}

int32_t interrupt_manager_register(uint32_t vector,
                                   driver_irq_handler_t handler,
                                   void *context)
{
    interrupt_slot_t *slot = interrupt_slot(vector);
    if (slot == NULL || handler == NULL) {
        return -1;
    }
    spinlock_lock(&g_lock);
    if (slot->allocated == 0u || slot->handler != NULL) {
        spinlock_unlock(&g_lock);
        return -1;
    }
    slot->handler = handler;
    slot->context = context;
    spinlock_unlock(&g_lock);
    return 0;
}

void interrupt_manager_unregister(uint32_t vector)
{
    interrupt_slot_t *slot = interrupt_slot(vector);
    if (slot == NULL) {
        return;
    }
    spinlock_lock(&g_lock);
    slot->handler = NULL;
    slot->context = NULL;
    spinlock_unlock(&g_lock);
}

void interrupt_manager_dispatch(uint32_t vector)
{
    interrupt_slot_t *slot = interrupt_slot(vector);
    if (slot == NULL) {
        return;
    }
    driver_irq_handler_t handler =
        __atomic_load_n(&slot->handler, __ATOMIC_ACQUIRE);
    void *context = __atomic_load_n(&slot->context, __ATOMIC_ACQUIRE);
    if (handler != NULL) {
        handler(context);
    }
}
