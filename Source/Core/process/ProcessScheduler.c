#include "ProcessScheduler.h"

#include <string.h>

#include "smp/SMP_Main.h"

static int32_t g_current_pid_per_cpu[OS_CONFIG_SMP_MAX_CPUS];
static int32_t g_last_pick_per_cpu[OS_CONFIG_SMP_MAX_CPUS];
static int32_t g_leaving_pid_per_cpu[OS_CONFIG_SMP_MAX_CPUS];
static uint8_t g_resched_per_cpu[OS_CONFIG_SMP_MAX_CPUS];
static uint32_t g_timeslice_ticks = 6;
static uint64_t g_cpu_idle_ns[OS_CONFIG_SMP_MAX_CPUS];

static uint32_t scheduler_cpu_id(void)
{
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu >= OS_CONFIG_SMP_MAX_CPUS) {
        cpu = 0;
    }
    return cpu;
}

static int scheduler_pid_running_on_other_cpu(const process_t *processes,
                                              int32_t capacity,
                                              int32_t pid,
                                              uint32_t current_cpu)
{
    if (processes == 0 || pid < 0 || pid >= capacity) {
        return 0;
    }
    uint32_t num_cpus = smp_get_cpu_count();
    if (num_cpus <= 1u) {
        return 0;
    }
    if (num_cpus > (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
        num_cpus = (uint32_t)OS_CONFIG_SMP_MAX_CPUS;
    }
    for (uint32_t cpu = 0; cpu < num_cpus; ++cpu) {
        if (cpu != current_cpu && g_current_pid_per_cpu[cpu] == pid) {
            return 1;
        }
    }
    return 0;
}

static uint32_t scheduler_timeslice_for_process(const process_t *proc)
{
    if (proc == NULL) {
        return g_timeslice_ticks;
    }
    uint32_t base = g_timeslice_ticks;
    switch (proc->priority) {
    case 0:  return base;
    case 1:  return base + base / 2;
    case 2:  return base * 2u;
    default: return base * 3u;
    }
}

void process_scheduler_init(uint32_t timeslice_ticks)
{
    g_timeslice_ticks = timeslice_ticks != 0u ? timeslice_ticks : 1u;
    for (uint32_t i = 0; i < OS_CONFIG_SMP_MAX_CPUS; ++i) {
        g_current_pid_per_cpu[i] = -1;
        g_last_pick_per_cpu[i] = -1;
        g_leaving_pid_per_cpu[i] = -1;
        g_resched_per_cpu[i] = 0;
        g_cpu_idle_ns[i] = 0;
    }
}

void process_scheduler_set_leaving_pid(int32_t pid)
{
    uint32_t cpu = scheduler_cpu_id();
    g_leaving_pid_per_cpu[cpu] = pid;
}

void process_scheduler_clear_leaving_pid(void)
{
    uint32_t cpu = scheduler_cpu_id();
    g_leaving_pid_per_cpu[cpu] = -1;
}

int process_scheduler_pid_in_use_on_any_cpu(int32_t pid)
{
    if (pid < 0) {
        return 0;
    }
    for (uint32_t cpu = 0; cpu < OS_CONFIG_SMP_MAX_CPUS; ++cpu) {
        if (g_current_pid_per_cpu[cpu] == pid ||
            g_leaving_pid_per_cpu[cpu] == pid) {
            return 1;
        }
    }
    return 0;
}

int32_t process_scheduler_current_pid(void)
{
    return g_current_pid_per_cpu[scheduler_cpu_id()];
}

void process_scheduler_set_current_pid(int32_t pid)
{
    uint32_t cpu = scheduler_cpu_id();
    g_current_pid_per_cpu[cpu] = pid;
    g_last_pick_per_cpu[cpu] = pid;
    smp_set_current_pid(pid);
}

int32_t process_scheduler_pick_next(process_t *processes,
                                    int32_t capacity,
                                    int32_t current_pid)
{
    if (processes == 0 || capacity <= 0) {
        return -1;
    }

    uint32_t cpu = scheduler_cpu_id();
    int32_t start = current_pid;
    if (start < 0 || start >= capacity) {
        start = g_last_pick_per_cpu[cpu];
    }
    if (start < 0 || start >= capacity) {
        uint32_t online = smp_get_cpu_count();
        if (online == 0u) {
            online = 1u;
        }
        start = (int32_t)(((uint64_t)capacity * cpu) / online);
        if (start >= capacity) {
            start = capacity - 1;
        }
    }

    for (int32_t step = 1; step <= capacity; ++step) {
        int32_t idx = (start + step) % capacity;
        if (processes[idx].state == PROCESS_STATE_READY &&
            !scheduler_pid_running_on_other_cpu(processes, capacity,
                                                idx, cpu)) {
            if (cpu != 0u && (idx == 0 || idx == 1)) {
                continue;
            }
            return idx;
        }
    }

    if (current_pid >= 0 && current_pid < capacity &&
        !scheduler_pid_running_on_other_cpu(processes, capacity,
                                            current_pid, cpu) &&
        (processes[current_pid].state == PROCESS_STATE_RUNNING ||
         processes[current_pid].state == PROCESS_STATE_READY)) {
        if (cpu != 0u && (current_pid == 0 || current_pid == 1)) {
            return -1;
        }
        return current_pid;
    }

    return -1;
}

void process_scheduler_on_tick(process_t *processes, int32_t capacity)
{
    int32_t pid = process_scheduler_current_pid();
    if (processes == 0 || pid < 0 || pid >= capacity) {
        return;
    }

    process_t *proc = &processes[pid];
    if (proc->state != PROCESS_STATE_RUNNING) {
        return;
    }

    proc->total_ticks++;
    if (proc->timeslice == 0u) {
        proc->timeslice = scheduler_timeslice_for_process(proc);
    }
    if (proc->timeslice > 0u) {
        proc->timeslice--;
        if (proc->timeslice == 0u) {
            process_scheduler_request_reschedule();
        }
    }
}

void process_scheduler_request_reschedule(void)
{
    g_resched_per_cpu[scheduler_cpu_id()] = 1u;
}

int process_scheduler_consume_reschedule(void)
{
    uint32_t cpu = scheduler_cpu_id();
    int pending = g_resched_per_cpu[cpu] != 0u;
    g_resched_per_cpu[cpu] = 0u;
    return pending;
}

void process_scheduler_prepare_run(process_t *proc)
{
    if (proc == 0) {
        return;
    }
    proc->state = PROCESS_STATE_RUNNING;
    proc->timeslice = scheduler_timeslice_for_process(proc);
    g_resched_per_cpu[scheduler_cpu_id()] = 0u;
}

void process_scheduler_add_idle_ns(uint64_t ns)
{
    uint32_t cpu = scheduler_cpu_id();
    g_cpu_idle_ns[cpu] += ns;
}

uint64_t process_scheduler_get_idle_ns(uint32_t cpu)
{
    if (cpu >= (uint32_t)OS_CONFIG_SMP_MAX_CPUS) return 0;
    return g_cpu_idle_ns[cpu];
}

uint32_t process_scheduler_max_cpus(void)
{
    return (uint32_t)OS_CONFIG_SMP_MAX_CPUS;
}

void process_scheduler_debug_dump_cpus(void)
{
    extern void serial_write_string(const char *str);
    extern void serial_write_uint64(uint64_t value);
    serial_write_string("[CPUS] ");
    for (uint32_t cpu = 0; cpu < OS_CONFIG_SMP_MAX_CPUS; ++cpu) {
        if (cpu != 0u) {
            serial_write_string(" ");
        }
        serial_write_uint64((uint64_t)(uint32_t)g_current_pid_per_cpu[cpu]);
        serial_write_string("/");
        serial_write_uint64((uint64_t)(uint32_t)g_leaving_pid_per_cpu[cpu]);
    }
    serial_write_string("\n");
}
