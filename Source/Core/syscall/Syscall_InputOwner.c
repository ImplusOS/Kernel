#include "Syscall_InputOwner.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"

static spinlock_t g_input_owner_lock;
static int32_t g_input_owner_pid = -1;

bool syscall_input_owner_claim(int32_t pid)
{
    if (pid < 0) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_input_owner_lock);

    bool ok = false;
    if (g_input_owner_pid < 0 ||
        g_input_owner_pid == pid ||
        !process_is_alive(g_input_owner_pid)) {
        g_input_owner_pid = pid;
        ok = true;
    }

    spinlock_unlock(&g_input_owner_lock);
    irq_restore(irq_flags);
    return ok;
}

void syscall_input_owner_release(int32_t pid)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_input_owner_lock);

    if (g_input_owner_pid == pid) {
        g_input_owner_pid = -1;
    }

    spinlock_unlock(&g_input_owner_lock);
    irq_restore(irq_flags);
}

int32_t syscall_input_owner_get(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_input_owner_lock);
    int32_t pid = g_input_owner_pid;
    spinlock_unlock(&g_input_owner_lock);
    irq_restore(irq_flags);
    return pid;
}

bool syscall_input_owner_is(int32_t pid)
{
    return pid >= 0 && syscall_input_owner_get() == pid;
}
