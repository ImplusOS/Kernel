#pragma once
#include <stdint.h>
/* The harness is single-threaded, so the lock only has to keep the shape. */
typedef struct { volatile int locked; } spinlock_t;
static inline uint64_t irq_save_disable(void) { return 0u; }
static inline void irq_restore(uint64_t flags) { (void)flags; }
static inline void spinlock_init(spinlock_t *lock) { lock->locked = 0; }
static inline void spinlock_lock(spinlock_t *lock) { lock->locked = 1; }
static inline void spinlock_unlock(spinlock_t *lock) { lock->locked = 0; }
