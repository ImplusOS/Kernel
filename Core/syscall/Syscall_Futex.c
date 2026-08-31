#include "Syscall_Main.h"
#include "Core/process/ProcessManager.h"
#include "Core/usercopy/Usercopy.h"
#include "Core/timer/Timer.h"
#include "Core/sync/Spinlock.h"

#include <stddef.h>
#include <stdint.h>

#define FUTEX_WAIT_QUEUE_SIZE  128

#define FUTEX_WAIT        0
#define FUTEX_WAKE        1
#define FUTEX_REQUEUE     3
#define FUTEX_CMP_REQUEUE 4
#define FUTEX_WAKE_OP     5
#define FUTEX_LOCK_PI     6
#define FUTEX_UNLOCK_PI   7
#define FUTEX_TRYLOCK_PI  8
#define FUTEX_WAIT_BITSET 9
#define FUTEX_WAKE_BITSET 10
#define FUTEX_LOCK_PI2    13
#define FUTEX_PRIVATE     128
#define FUTEX_CMD_MASK    0x7f

/* PI-futex word layout (Linux include/uapi/linux/futex.h). */
#define FUTEX_TID_MASK    0x3FFFFFFFu
#define FUTEX_WAITERS     0x80000000u
#define FUTEX_OWNER_DIED  0x40000000u

#define FUTEX_EPERM       (-1LL)
#define FUTEX_EDEADLK     (-35LL)
#define FUTEX_EAGAIN_     (-11LL)
#define FUTEX_EOWNERDEAD  (-130LL)

/* FUTEX_WAKE_OP opcodes (see Linux include/uapi/linux/futex.h FUTEX_OP()). */
#define FUTEX_OP_SET  0
#define FUTEX_OP_ADD  1
#define FUTEX_OP_OR   2
#define FUTEX_OP_ANDN 3
#define FUTEX_OP_XOR  4
#define FUTEX_OP_OPARG_SHIFT 8
#define FUTEX_OP_CMP_EQ 0
#define FUTEX_OP_CMP_NE 1
#define FUTEX_OP_CMP_LT 2
#define FUTEX_OP_CMP_LE 3
#define FUTEX_OP_CMP_GT 4
#define FUTEX_OP_CMP_GE 5

typedef struct {
    uint8_t   used;
    int32_t   tid;
    int32_t   owner_pid;
    uint64_t  uaddr;
    uint32_t  bitset;
    uint64_t  deadline_ms;
} futex_waiter_t;

static futex_waiter_t g_futex_waiters[FUTEX_WAIT_QUEUE_SIZE];
static spinlock_t     g_futex_lock;
static int            g_futex_initialized = 0;

static void futex_ensure_init(void)
{
    if (!g_futex_initialized) {
        spinlock_init(&g_futex_lock);
        for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE; ++i) {
            g_futex_waiters[i].used = 0;
        }
        g_futex_initialized = 1;
    }
}

static uint64_t futex_uptime_ms(void)
{
    uint32_t hz = timer_hz();
    if (hz == 0) hz = 60;
    return (timer_ticks() * 1000ULL) / hz;
}

int64_t syscall_futex_wait(uint64_t uaddr, int32_t expected,
                           uint64_t timeout_ns, uint32_t bitset)
{
    futex_ensure_init();

    if (uaddr == 0) {
        return -14;
    }
    
    int32_t *ptr = (int32_t *)(uintptr_t)uaddr;
    if (!process_user_buffer_is_valid(ptr, sizeof(int32_t))) {
        return -14;
    }

    int32_t current_value = 0;
    if (copy_from_user_trusted(&current_value, ptr, sizeof(current_value)) != 0u) {
        return -14;
    }

    if (current_value != expected) {
        return -11;
    }

    int32_t tid = process_get_current_tid();
    int32_t owner_pid = process_get_current_pid();
    if (tid < 0 || owner_pid < 0) {
        return -3;
    }

    uint64_t deadline = 0;
    if (timeout_ns > 0) {
        uint64_t timeout_ms = (timeout_ns + 999999ULL) / 1000000ULL;
        deadline = futex_uptime_ms() + timeout_ms;
    }

    spinlock_lock(&g_futex_lock);
    current_value = 0;
    if (copy_from_user_trusted(&current_value, ptr, sizeof(current_value)) != 0u) {
        spinlock_unlock(&g_futex_lock);
        return -14;
    }
    if (current_value != expected) {
        spinlock_unlock(&g_futex_lock);
        return -11;
    }
    int slot = -1;
    for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE; ++i) {
        if (!g_futex_waiters[i].used) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        spinlock_unlock(&g_futex_lock);
        return -12;
    }
    g_futex_waiters[slot].used        = 1;
    g_futex_waiters[slot].tid         = tid;
    g_futex_waiters[slot].owner_pid   = owner_pid;
    g_futex_waiters[slot].uaddr       = uaddr;
    g_futex_waiters[slot].bitset      = bitset ? bitset : 0xFFFFFFFFu;
    g_futex_waiters[slot].deadline_ms = deadline;
    spinlock_unlock(&g_futex_lock);
    if (process_block_current() < 0) {
        spinlock_lock(&g_futex_lock);
        g_futex_waiters[slot].used = 0;
        spinlock_unlock(&g_futex_lock);
        return -3;
    }
    return 0;
}

int64_t syscall_futex_wake(uint64_t uaddr, int32_t count, uint32_t bitset)
{
    futex_ensure_init();

    if (count <= 0) {
        return 0;
    }
    if (bitset == 0) {
        bitset = 0xFFFFFFFFu;
    }
    if (!process_user_buffer_is_valid((const void *)(uintptr_t)uaddr,
                                      sizeof(int32_t))) {
        return -14;
    }

    int32_t owner_pid = process_get_current_pid();
    int32_t tids[FUTEX_WAIT_QUEUE_SIZE];
    int32_t woken = 0;
    spinlock_lock(&g_futex_lock);
    for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE && woken < count; ++i) {
        if (g_futex_waiters[i].used &&
            g_futex_waiters[i].owner_pid == owner_pid &&
            g_futex_waiters[i].uaddr == uaddr &&
            (g_futex_waiters[i].bitset & bitset) != 0) {
            tids[woken] = g_futex_waiters[i].tid;
            g_futex_waiters[i].used = 0;
            ++woken;
        }
    }
    spinlock_unlock(&g_futex_lock);
    for (int32_t i = 0; i < woken; ++i) {
        (void)process_wake_pid(tids[i]);
    }
    return (int64_t)woken;
}

void syscall_futex_on_timer_tick(void)
{
    if (!g_futex_initialized) {
        return;
    }

    uint64_t now = futex_uptime_ms();
    int32_t tids[FUTEX_WAIT_QUEUE_SIZE];
    int32_t count = 0;

    spinlock_lock(&g_futex_lock);
    for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE; ++i) {
        if (g_futex_waiters[i].used &&
            g_futex_waiters[i].deadline_ms != 0 &&
            g_futex_waiters[i].deadline_ms <= now) {
            tids[count++] = g_futex_waiters[i].tid;
            g_futex_waiters[i].used = 0;
        }
    }
    spinlock_unlock(&g_futex_lock);

    for (int32_t i = 0; i < count; ++i) {
        (void)process_wake_pid(tids[i]);
    }
}

/* FUTEX_REQUEUE / FUTEX_CMP_REQUEUE: wake up to `nr_wake` waiters on uaddr,
 * then move up to `nr_requeue` of the *remaining* waiters on uaddr over to
 * uaddr2's wait queue (without waking them - a later FUTEX_WAKE on uaddr2
 * will reach them). This is what glibc's pthread_cond_broadcast() relies
 * on to avoid a "thundering herd" waking every waiter just to have most of
 * them immediately re-block on the mutex. `expected` is non-NULL only for
 * CMP_REQUEUE, in which case *uaddr must equal *expected or the whole
 * operation is aborted with EAGAIN (matches Linux semantics). */
static int64_t syscall_futex_requeue(uint64_t uaddr, int32_t nr_wake,
                                     int32_t nr_requeue, uint64_t uaddr2,
                                     const int32_t *expected)
{
    futex_ensure_init();
    if (!process_user_buffer_is_valid((const void *)(uintptr_t)uaddr,
                                      sizeof(int32_t))) {
        return -14;
    }

    int32_t owner_pid = process_get_current_pid();
    int32_t tids[FUTEX_WAIT_QUEUE_SIZE];
    int32_t woken = 0;
    int32_t moved = 0;

    spinlock_lock(&g_futex_lock);
    if (expected != NULL) {
        int32_t current_value = 0;
        if (copy_from_user_trusted(&current_value,
                                   (const void *)(uintptr_t)uaddr,
                                   sizeof(current_value)) != 0u) {
            spinlock_unlock(&g_futex_lock);
            return -14;
        }
        if (current_value != *expected) {
            spinlock_unlock(&g_futex_lock);
            return -11;
        }
    }
    for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE && woken < nr_wake; ++i) {
        if (g_futex_waiters[i].used &&
            g_futex_waiters[i].owner_pid == owner_pid &&
            g_futex_waiters[i].uaddr == uaddr) {
            tids[woken] = g_futex_waiters[i].tid;
            g_futex_waiters[i].used = 0;
            ++woken;
        }
    }
    for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE && moved < nr_requeue; ++i) {
        if (g_futex_waiters[i].used &&
            g_futex_waiters[i].owner_pid == owner_pid &&
            g_futex_waiters[i].uaddr == uaddr) {
            g_futex_waiters[i].uaddr = uaddr2;
            ++moved;
        }
    }
    spinlock_unlock(&g_futex_lock);

    for (int32_t i = 0; i < woken; ++i) {
        (void)process_wake_pid(tids[i]);
    }
    return (int64_t)(woken + moved);
}

/* FUTEX_WAKE_OP: atomically apply an encoded op to *uaddr2, unconditionally
 * wake up to `nr_wake` waiters on uaddr, then - only if the pre-op value at
 * uaddr2 satisfies the encoded comparison - wake up to `nr_wake2` waiters
 * on uaddr2 too. Used by glibc's futex-based rwlocks/condvars. */
static int64_t syscall_futex_wake_op(uint64_t uaddr, int32_t nr_wake,
                                     int32_t nr_wake2, uint64_t uaddr2,
                                     uint32_t val3)
{
    futex_ensure_init();

    int32_t op = (int32_t)((val3 >> 28) & 0xFu);
    int32_t cmp = (int32_t)((val3 >> 24) & 0xFu);
    int32_t oparg = (int32_t)((val3 >> 12) & 0xFFFu);
    if ((oparg & 0x800) != 0) oparg -= 4096; /* sign-extend 12 bits */
    int32_t cmparg = (int32_t)(val3 & 0xFFFu);
    if ((cmparg & 0x800) != 0) cmparg -= 4096;
    int use_shift = (op & FUTEX_OP_OPARG_SHIFT) != 0;
    op &= 0x7;

    if (uaddr2 == 0u ||
        !process_user_buffer_is_valid((void *)(uintptr_t)uaddr2, sizeof(int32_t))) {
        return -14;
    }
    if (use_shift && (oparg < 0 || oparg > 31)) {
        return -22; /* EINVAL: shift count out of range */
    }

    int32_t *ptr2 = (int32_t *)(uintptr_t)uaddr2;
    spinlock_lock(&g_futex_lock);
    int32_t oldval = 0;
    if (copy_from_user_trusted(&oldval, ptr2, sizeof(oldval)) != 0u) {
        spinlock_unlock(&g_futex_lock);
        return -14;
    }
    int32_t arg = use_shift ? (int32_t)(1 << oparg) : oparg;
    int32_t newval;
    switch (op) {
        case FUTEX_OP_SET:  newval = arg; break;
        case FUTEX_OP_ADD:  newval = oldval + arg; break;
        case FUTEX_OP_OR:   newval = oldval | arg; break;
        case FUTEX_OP_ANDN: newval = oldval & ~arg; break;
        case FUTEX_OP_XOR:  newval = oldval ^ arg; break;
        default:             newval = oldval; break;
    }
    if (copy_to_user_trusted(ptr2, &newval, sizeof(newval)) != 0u) {
        spinlock_unlock(&g_futex_lock);
        return -14;
    }
    spinlock_unlock(&g_futex_lock);

    int cmp_result;
    switch (cmp) {
        case FUTEX_OP_CMP_EQ: cmp_result = (oldval == cmparg); break;
        case FUTEX_OP_CMP_NE: cmp_result = (oldval != cmparg); break;
        case FUTEX_OP_CMP_LT: cmp_result = (oldval < cmparg); break;
        case FUTEX_OP_CMP_LE: cmp_result = (oldval <= cmparg); break;
        case FUTEX_OP_CMP_GT: cmp_result = (oldval > cmparg); break;
        case FUTEX_OP_CMP_GE: cmp_result = (oldval >= cmparg); break;
        default: cmp_result = 0; break;
    }

    int64_t woken1 = syscall_futex_wake(uaddr, nr_wake, 0xFFFFFFFFu);
    if (woken1 < 0) {
        return woken1;
    }
    int64_t woken2 = 0;
    if (cmp_result && nr_wake2 > 0) {
        woken2 = syscall_futex_wake(uaddr2, nr_wake2, 0xFFFFFFFFu);
        if (woken2 < 0) {
            woken2 = 0;
        }
    }
    return woken1 + woken2;
}

/* -------------------------------------------------------------------- *
 * Priority-inheritance mutexes (FUTEX_LOCK_PI / UNLOCK_PI / TRYLOCK_PI) *
 * -------------------------------------------------------------------- *
 * TODO_Chromium_LinuxABI.md section 3.6. This scheduler has no real
 * priority inheritance (it is round-robin), so what is implemented here
 * is the PI *ownership protocol* only: the futex word carries the owner
 * TID in its low 30 bits plus FUTEX_WAITERS, LOCK_PI blocks until it
 * becomes the owner, and UNLOCK_PI hands ownership directly to one queued
 * waiter before waking it. No priority boosting happens - that is
 * acceptable here and is what glibc's PTHREAD_PRIO_INHERIT mutexes need
 * to be *functionally* correct. `timeout` (an absolute timespec for PI
 * ops) is not armed; a timed pthread_mutex_timedlock on a PI mutex will
 * block as if untimed (known limitation, mirrors the WAIT path's
 * best-effort timeout handling). */

static int futex_uaddr_has_waiter_locked(int32_t owner_pid, uint64_t uaddr,
                                         int32_t except_slot)
{
    for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE; ++i) {
        if (i == except_slot) {
            continue;
        }
        if (g_futex_waiters[i].used &&
            g_futex_waiters[i].owner_pid == owner_pid &&
            g_futex_waiters[i].uaddr == uaddr) {
            return 1;
        }
    }
    return 0;
}

static int64_t syscall_futex_lock_pi(uint64_t uaddr, int try_only)
{
    futex_ensure_init();

    int32_t *ptr = (int32_t *)(uintptr_t)uaddr;
    if (uaddr == 0u || !process_user_buffer_is_valid(ptr, sizeof(int32_t))) {
        return -14;
    }
    int32_t tid = process_get_current_tid();
    int32_t owner_pid = process_get_current_pid();
    if (tid < 0 || owner_pid < 0) {
        return -3;
    }

    spinlock_lock(&g_futex_lock);
    uint32_t word = 0;
    if (copy_from_user_trusted(&word, ptr, sizeof(word)) != 0u) {
        spinlock_unlock(&g_futex_lock);
        return -14;
    }
    uint32_t cur_owner = word & FUTEX_TID_MASK;

    if (cur_owner == 0u) {
        uint32_t newword = (uint32_t)tid;
        if (futex_uaddr_has_waiter_locked(owner_pid, uaddr, -1)) {
            newword |= FUTEX_WAITERS;
        }
        int rc = (copy_to_user_trusted(ptr, &newword, sizeof(newword)) != 0u);
        int died = (word & FUTEX_OWNER_DIED) != 0u;
        spinlock_unlock(&g_futex_lock);
        if (rc) return -14;
        return died ? FUTEX_EOWNERDEAD : 0;
    }
    if (cur_owner == (uint32_t)tid) {
        spinlock_unlock(&g_futex_lock);
        return FUTEX_EDEADLK;
    }
    if (try_only) {
        spinlock_unlock(&g_futex_lock);
        return FUTEX_EAGAIN_;
    }

    uint32_t newword = word | FUTEX_WAITERS;
    if (copy_to_user_trusted(ptr, &newword, sizeof(newword)) != 0u) {
        spinlock_unlock(&g_futex_lock);
        return -14;
    }
    int slot = -1;
    for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE; ++i) {
        if (!g_futex_waiters[i].used) { slot = i; break; }
    }
    if (slot < 0) {
        spinlock_unlock(&g_futex_lock);
        return -12;
    }
    g_futex_waiters[slot].used        = 1;
    g_futex_waiters[slot].tid         = tid;
    g_futex_waiters[slot].owner_pid   = owner_pid;
    g_futex_waiters[slot].uaddr       = uaddr;
    g_futex_waiters[slot].bitset      = 0xFFFFFFFFu;
    g_futex_waiters[slot].deadline_ms = 0;
    spinlock_unlock(&g_futex_lock);

    if (process_block_current() < 0) {
        spinlock_lock(&g_futex_lock);
        g_futex_waiters[slot].used = 0;
        spinlock_unlock(&g_futex_lock);
        return -3;
    }

    /* Woken. UNLOCK_PI hands ownership to us before the wake, so normally
     * the word already holds our TID. If not (spurious wake / a racing
     * take), report EAGAIN so glibc retries the syscall. */
    spinlock_lock(&g_futex_lock);
    g_futex_waiters[slot].used = 0;
    word = 0;
    if (copy_from_user_trusted(&word, ptr, sizeof(word)) != 0u) {
        spinlock_unlock(&g_futex_lock);
        return -14;
    }
    int owned = ((word & FUTEX_TID_MASK) == (uint32_t)tid);
    int died = (word & FUTEX_OWNER_DIED) != 0u;
    spinlock_unlock(&g_futex_lock);
    if (owned) {
        return died ? FUTEX_EOWNERDEAD : 0;
    }
    return FUTEX_EAGAIN_;
}

static int64_t syscall_futex_unlock_pi(uint64_t uaddr)
{
    futex_ensure_init();

    int32_t *ptr = (int32_t *)(uintptr_t)uaddr;
    if (uaddr == 0u || !process_user_buffer_is_valid(ptr, sizeof(int32_t))) {
        return -14;
    }
    int32_t tid = process_get_current_tid();
    int32_t owner_pid = process_get_current_pid();
    if (tid < 0 || owner_pid < 0) {
        return -3;
    }

    spinlock_lock(&g_futex_lock);
    uint32_t word = 0;
    if (copy_from_user_trusted(&word, ptr, sizeof(word)) != 0u) {
        spinlock_unlock(&g_futex_lock);
        return -14;
    }
    if ((word & FUTEX_TID_MASK) != (uint32_t)tid) {
        spinlock_unlock(&g_futex_lock);
        return FUTEX_EPERM;
    }

    int next = -1;
    for (int i = 0; i < FUTEX_WAIT_QUEUE_SIZE; ++i) {
        if (g_futex_waiters[i].used &&
            g_futex_waiters[i].owner_pid == owner_pid &&
            g_futex_waiters[i].uaddr == uaddr) {
            next = i;
            break;
        }
    }

    uint32_t newword;
    int32_t wake_tid = -1;
    if (next < 0) {
        newword = 0u;
    } else {
        wake_tid = g_futex_waiters[next].tid;
        g_futex_waiters[next].used = 0;
        newword = (uint32_t)wake_tid;
        if (futex_uaddr_has_waiter_locked(owner_pid, uaddr, next)) {
            newword |= FUTEX_WAITERS;
        }
    }
    if (copy_to_user_trusted(ptr, &newword, sizeof(newword)) != 0u) {
        spinlock_unlock(&g_futex_lock);
        return -14;
    }
    spinlock_unlock(&g_futex_lock);

    if (wake_tid >= 0) {
        (void)process_wake_pid(wake_tid);
    }
    return 0;
}

int64_t syscall_futex(uint64_t uaddr, uint64_t op, uint64_t val,
                      uint64_t timeout_or_val2, uint64_t uaddr2,
                      uint64_t val3)
{
    int cmd = (int)(op & FUTEX_CMD_MASK);

    switch (cmd) {
        case FUTEX_WAIT:
            return syscall_futex_wait(uaddr, (int32_t)val,
                                      timeout_or_val2, 0xFFFFFFFFu);
        case FUTEX_WAKE:
            return syscall_futex_wake(uaddr, (int32_t)val, 0xFFFFFFFFu);
        case FUTEX_WAIT_BITSET:
            return syscall_futex_wait(uaddr, (int32_t)val,
                                      timeout_or_val2, (uint32_t)val3);
        case FUTEX_WAKE_BITSET:
            return syscall_futex_wake(uaddr, (int32_t)val, (uint32_t)val3);
        case FUTEX_REQUEUE:
            return syscall_futex_requeue(uaddr, (int32_t)val,
                                         (int32_t)timeout_or_val2, uaddr2,
                                         NULL);
        case FUTEX_CMP_REQUEUE: {
            int32_t expected = (int32_t)val3;
            return syscall_futex_requeue(uaddr, (int32_t)val,
                                         (int32_t)timeout_or_val2, uaddr2,
                                         &expected);
        }
        case FUTEX_WAKE_OP:
            return syscall_futex_wake_op(uaddr, (int32_t)val,
                                         (int32_t)timeout_or_val2, uaddr2,
                                         (uint32_t)val3);
        case FUTEX_LOCK_PI:
        case FUTEX_LOCK_PI2:
            return syscall_futex_lock_pi(uaddr, 0);
        case FUTEX_TRYLOCK_PI:
            return syscall_futex_lock_pi(uaddr, 1);
        case FUTEX_UNLOCK_PI:
            return syscall_futex_unlock_pi(uaddr);
        default:
            return -38;
    }
}
