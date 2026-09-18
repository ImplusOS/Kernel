#include "Linux_FdTable.h"

#include <stdlib.h>
#include <string.h>

#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "kernel/config.h"

/* See Linux_FdTable.h for what this is and why. */

#define LXFD_EMPTY (-1)

typedef struct {
    int32_t global[LXFD_MAX];
    uint8_t cloexec[LXFD_MAX / 8];
} lxfd_table_t;

static lxfd_table_t *g_lxfd[OS_CONFIG_PROCESS_MAX_COUNT];
static spinlock_t g_lxfd_lock;
static volatile uint8_t g_lxfd_lock_ready;

static uint64_t lxfd_lock(void)
{
    uint64_t flags = irq_save_disable();
    if (!g_lxfd_lock_ready) {
        spinlock_init(&g_lxfd_lock);
        g_lxfd_lock_ready = 1u;
    }
    spinlock_lock(&g_lxfd_lock);
    return flags;
}

static void lxfd_unlock(uint64_t flags)
{
    spinlock_unlock(&g_lxfd_lock);
    irq_restore(flags);
}

static int32_t lxfd_owner_of(int32_t pid)
{
    int32_t owner = process_memory_owner_pid_of(pid);
    if (owner < 0 || owner >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) {
        return -1;
    }
    return owner;
}

/* A fresh table: fds 0/1/2 are the console, which is what every process here
 * starts with (the console is global 0/1/2, never allocated to anything). */
static lxfd_table_t *lxfd_new_table(void)
{
    lxfd_table_t *t = (lxfd_table_t *)malloc(sizeof(lxfd_table_t));
    if (t == NULL) {
        return NULL;
    }
    for (int32_t i = 0; i < LXFD_MAX; ++i) {
        t->global[i] = LXFD_EMPTY;
    }
    memset(t->cloexec, 0, sizeof(t->cloexec));
    t->global[0] = 0;
    t->global[1] = 1;
    t->global[2] = 2;
    return t;
}

/* The calling process's table, created on first use. Caller holds the lock.
 * malloc() under a spinlock is fine here: the kernel heap has its own lock and
 * never calls back into this module. */
static lxfd_table_t *lxfd_current_locked(void)
{
    int32_t owner = lxfd_owner_of(process_get_current_pid());
    if (owner < 0) {
        return NULL;
    }
    if (g_lxfd[owner] == NULL) {
        g_lxfd[owner] = lxfd_new_table();
    }
    return g_lxfd[owner];
}

static int lxfd_cloexec_test(const lxfd_table_t *t, int32_t ufd)
{
    return (t->cloexec[ufd >> 3] & (uint8_t)(1u << (ufd & 7))) != 0u;
}

static void lxfd_cloexec_put(lxfd_table_t *t, int32_t ufd, int on)
{
    uint8_t bit = (uint8_t)(1u << (ufd & 7));
    if (on) {
        t->cloexec[ufd >> 3] |= bit;
    } else {
        t->cloexec[ufd >> 3] &= (uint8_t)~bit;
    }
}

/* How many user fds of this table refer to `global`. */
static int32_t lxfd_refs_locked(const lxfd_table_t *t, int32_t global)
{
    int32_t n = 0;
    for (int32_t i = 0; i < LXFD_MAX; ++i) {
        if (t->global[i] == global) {
            ++n;
        }
    }
    return n;
}

void lxfd_fork(int32_t parent_pid, int32_t child_pid)
{
    int32_t parent = lxfd_owner_of(parent_pid);
    if (parent < 0 || child_pid < 0 ||
        child_pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) {
        return;
    }
    /* Allocate outside the lock, then copy under it. */
    lxfd_table_t *fresh = (lxfd_table_t *)malloc(sizeof(lxfd_table_t));
    uint64_t flags = lxfd_lock();
    lxfd_table_t *stale = g_lxfd[child_pid];
    g_lxfd[child_pid] = NULL;
    if (g_lxfd[parent] != NULL && fresh != NULL) {
        memcpy(fresh, g_lxfd[parent], sizeof(*fresh));
        g_lxfd[child_pid] = fresh;
        fresh = NULL;
    }
    lxfd_unlock(flags);
    /* No parent table (a process that never made a Linux syscall): the child
     * gets the default one lazily, same as its parent would have. */
    free(fresh);
    free(stale);
}

void lxfd_release_process(int32_t pid, void (*on_ref)(int32_t global))
{
    if (pid < 0 || pid >= (int32_t)OS_CONFIG_PROCESS_MAX_COUNT) {
        return;
    }
    uint64_t flags = lxfd_lock();
    lxfd_table_t *t = g_lxfd[pid];
    g_lxfd[pid] = NULL;
    lxfd_unlock(flags);
    if (t == NULL) {
        return;
    }
    if (on_ref != NULL) {
        for (int32_t i = 0; i < LXFD_MAX; ++i) {
            int32_t g = t->global[i];
            if (g == LXFD_EMPTY) {
                continue;
            }
            /* Once per distinct global: clear every later copy first. */
            for (int32_t j = i + 1; j < LXFD_MAX; ++j) {
                if (t->global[j] == g) {
                    t->global[j] = LXFD_EMPTY;
                }
            }
            on_ref(g);
        }
    }
    free(t);
}

int32_t lxfd_get(int32_t ufd)
{
    if (ufd < 0 || ufd >= LXFD_MAX) {
        return -1;
    }
    uint64_t flags = lxfd_lock();
    lxfd_table_t *t = lxfd_current_locked();
    int32_t g = (t != NULL) ? t->global[ufd] : -1;
    lxfd_unlock(flags);
    return g;
}

int32_t lxfd_get_for(int32_t pid, int32_t ufd)
{
    if (ufd < 0 || ufd >= LXFD_MAX) {
        return -1;
    }
    int32_t owner = lxfd_owner_of(pid);
    if (owner < 0) {
        return -1;
    }
    uint64_t flags = lxfd_lock();
    int32_t g = (g_lxfd[owner] != NULL) ? g_lxfd[owner]->global[ufd] : -1;
    lxfd_unlock(flags);
    return g;
}

int32_t lxfd_install(int32_t global, int cloexec, int32_t min_ufd)
{
    if (min_ufd < 0) {
        min_ufd = 0;
    }
    uint64_t flags = lxfd_lock();
    lxfd_table_t *t = lxfd_current_locked();
    int32_t result = -24; /* EMFILE */
    if (t != NULL) {
        for (int32_t i = min_ufd; i < LXFD_MAX; ++i) {
            if (t->global[i] == LXFD_EMPTY) {
                t->global[i] = global;
                lxfd_cloexec_put(t, i, cloexec);
                result = i;
                break;
            }
        }
    }
    lxfd_unlock(flags);
    return result;
}

int32_t lxfd_set(int32_t ufd, int32_t global, int cloexec,
                 int32_t *replaced, int *replaced_last)
{
    *replaced = -1;
    *replaced_last = 0;
    if (ufd < 0 || ufd >= LXFD_MAX) {
        return -9; /* EBADF */
    }
    uint64_t flags = lxfd_lock();
    lxfd_table_t *t = lxfd_current_locked();
    if (t == NULL) {
        lxfd_unlock(flags);
        return -12; /* ENOMEM */
    }
    int32_t old = t->global[ufd];
    t->global[ufd] = global;
    lxfd_cloexec_put(t, ufd, cloexec);
    if (old != LXFD_EMPTY && old != global) {
        *replaced = old;
        *replaced_last = (lxfd_refs_locked(t, old) == 0);
    }
    lxfd_unlock(flags);
    return ufd;
}

int lxfd_remove(int32_t ufd, int32_t *global_out)
{
    *global_out = -1;
    if (ufd < 0 || ufd >= LXFD_MAX) {
        return -9;
    }
    uint64_t flags = lxfd_lock();
    lxfd_table_t *t = lxfd_current_locked();
    int rc = -9;
    if (t != NULL && t->global[ufd] != LXFD_EMPTY) {
        int32_t g = t->global[ufd];
        t->global[ufd] = LXFD_EMPTY;
        lxfd_cloexec_put(t, ufd, 0);
        *global_out = g;
        rc = (lxfd_refs_locked(t, g) == 0) ? 1 : 0;
    }
    lxfd_unlock(flags);
    return rc;
}

int lxfd_get_cloexec(int32_t ufd)
{
    if (ufd < 0 || ufd >= LXFD_MAX) {
        return -9;
    }
    uint64_t flags = lxfd_lock();
    lxfd_table_t *t = lxfd_current_locked();
    int rc = -9;
    if (t != NULL && t->global[ufd] != LXFD_EMPTY) {
        rc = lxfd_cloexec_test(t, ufd);
    }
    lxfd_unlock(flags);
    return rc;
}

int lxfd_set_cloexec(int32_t ufd, int on)
{
    if (ufd < 0 || ufd >= LXFD_MAX) {
        return -9;
    }
    uint64_t flags = lxfd_lock();
    lxfd_table_t *t = lxfd_current_locked();
    int rc = -9;
    if (t != NULL && t->global[ufd] != LXFD_EMPTY) {
        lxfd_cloexec_put(t, ufd, on);
        rc = 0;
    }
    lxfd_unlock(flags);
    return rc;
}

void lxfd_exec_close_cloexec(void (*closer)(int32_t global))
{
    /* Collect under the lock, close outside it: backends take their own locks
     * and some of them log. */
    int32_t doomed[64];
    for (;;) {
        uint32_t count = 0;
        uint64_t flags = lxfd_lock();
        lxfd_table_t *t = lxfd_current_locked();
        if (t != NULL) {
            for (int32_t i = 0; i < LXFD_MAX && count < 64u; ++i) {
                if (t->global[i] == LXFD_EMPTY || !lxfd_cloexec_test(t, i)) {
                    continue;
                }
                int32_t g = t->global[i];
                t->global[i] = LXFD_EMPTY;
                lxfd_cloexec_put(t, i, 0);
                if (lxfd_refs_locked(t, g) == 0) {
                    doomed[count++] = g;
                }
            }
        }
        lxfd_unlock(flags);
        for (uint32_t i = 0; i < count; ++i) {
            if (closer != NULL) {
                closer(doomed[i]);
            }
        }
        if (count < 64u) {
            return;
        }
    }
}

int32_t lxfd_next_open(int32_t pid, int32_t after)
{
    int32_t owner = lxfd_owner_of(pid);
    if (owner < 0) {
        return -2;
    }
    int32_t start = (after < 0) ? 0 : after + 1;
    uint64_t flags = lxfd_lock();
    lxfd_table_t *t = g_lxfd[owner];
    int32_t found = -1;
    if (t == NULL) {
        found = -2;
    } else {
        for (int32_t i = start; i < LXFD_MAX; ++i) {
            if (t->global[i] != LXFD_EMPTY) {
                found = i;
                break;
            }
        }
    }
    lxfd_unlock(flags);
    return found;
}
