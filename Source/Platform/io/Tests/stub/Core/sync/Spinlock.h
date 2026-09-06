#pragma once
#include <stdint.h>
typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_init(spinlock_t *l) { l->locked = 0; }
static inline void spinlock_lock(spinlock_t *l) { l->locked = 1; }
static inline void spinlock_unlock(spinlock_t *l) { l->locked = 0; }
static inline uint64_t irq_save_disable(void) { return 0u; }
static inline void irq_restore(uint64_t flags) { (void)flags; }
