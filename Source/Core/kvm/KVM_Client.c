#include "KVM_Client.h"
#include "virt/VMX.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include <string.h>

#define KVM_MAX_VMS   4
#define KVM_MAX_VCPUS 4

typedef struct {
    uint8_t used;
    int32_t fd;
    vmx_vcpu_t vcpus[KVM_MAX_VCPUS];
} kvm_vm_t;

static kvm_vm_t g_vms[KVM_MAX_VMS];
static spinlock_t g_kvm_lock;
static int g_kvm_init_done = 0;
static int32_t g_next_fd = 0x7000;

void kvm_client_init(void) {
    spinlock_init(&g_kvm_lock);
    memset(g_vms, 0, sizeof(g_vms));
    g_kvm_init_done = 1;
}

int64_t kvm_client_open(void) {
    if (!g_kvm_init_done) kvm_client_init();
    
    spinlock_lock(&g_kvm_lock);
    for (int i = 0; i < KVM_MAX_VMS; i++) {
        if (!g_vms[i].used) {
            g_vms[i].used = 1;
            g_vms[i].fd = g_next_fd++;
            memset(g_vms[i].vcpus, 0, sizeof(g_vms[i].vcpus));
            spinlock_unlock(&g_kvm_lock);
            return g_vms[i].fd;
        }
    }
    spinlock_unlock(&g_kvm_lock);
    return -12;  
}

static kvm_vm_t *get_vm_by_fd(int32_t fd) {
    for (int i = 0; i < KVM_MAX_VMS; i++) {
        if (g_vms[i].used && g_vms[i].fd == fd) {
            return &g_vms[i];
        }
    }
    return NULL;
}

int64_t kvm_client_ioctl(int32_t fd, uint64_t request, uint64_t arg) {
    spinlock_lock(&g_kvm_lock);
    kvm_vm_t *vm = get_vm_by_fd(fd);
    if (!vm) {
        spinlock_unlock(&g_kvm_lock);
        return -9;  
    }

    int64_t ret = -22;  

    switch (request) {
        case KVM_CREATE_VM: {
            ret = 0;
            break;
        }
        case KVM_CREATE_VCPU: {
            uint32_t vcpu_id = (uint32_t)arg;
            if (vcpu_id < KVM_MAX_VCPUS) {
                if (!vm->vcpus[vcpu_id].active) {
                    if (vmx_vcpu_create(&vm->vcpus[vcpu_id]) == 0) {
                        ret = 0;
                    }
                }
            }
            break;
        }
        case KVM_SET_USER_MEMORY_REGION: {
            kvm_userspace_memory_region_t *region = (kvm_userspace_memory_region_t *)(uintptr_t)arg;
            if (region) {
                if (vm->vcpus[0].active) {
                    if (vmx_vcpu_add_mem_slot(&vm->vcpus[0], region) == 0) {
                        ret = 0;
                    }
                }
            }
            break;
        }
        case KVM_SET_REGS: {
            struct { uint32_t vcpu_id; vmx_regs_t regs; } *args = (void *)(uintptr_t)arg;
            if (args && args->vcpu_id < KVM_MAX_VCPUS && vm->vcpus[args->vcpu_id].active) {
                if (vmx_vcpu_set_regs(&vm->vcpus[args->vcpu_id], &args->regs) == 0) {
                    ret = 0;
                }
            }
            break;
        }
        case KVM_GET_REGS: {
            struct { uint32_t vcpu_id; vmx_regs_t regs; } *args = (void *)(uintptr_t)arg;
            if (args && args->vcpu_id < KVM_MAX_VCPUS && vm->vcpus[args->vcpu_id].active) {
                if (vmx_vcpu_get_regs(&vm->vcpus[args->vcpu_id], &args->regs) == 0) {
                    ret = 0;
                }
            }
            break;
        }
        case KVM_SET_SREGS: {
            struct { uint32_t vcpu_id; vmx_sregs_t sregs; } *args = (void *)(uintptr_t)arg;
            if (args && args->vcpu_id < KVM_MAX_VCPUS && vm->vcpus[args->vcpu_id].active) {
                if (vmx_vcpu_set_sregs(&vm->vcpus[args->vcpu_id], &args->sregs) == 0) {
                    ret = 0;
                }
            }
            break;
        }
        case KVM_GET_SREGS: {
            struct { uint32_t vcpu_id; vmx_sregs_t sregs; } *args = (void *)(uintptr_t)arg;
            if (args && args->vcpu_id < KVM_MAX_VCPUS && vm->vcpus[args->vcpu_id].active) {
                if (vmx_vcpu_get_sregs(&vm->vcpus[args->vcpu_id], &args->sregs) == 0) {
                    ret = 0;
                }
            }
            break;
        }
        case KVM_RUN: {
            uint32_t vcpu_id = (uint32_t)arg;
            if (vcpu_id < KVM_MAX_VCPUS && vm->vcpus[vcpu_id].active) {
                spinlock_unlock(&g_kvm_lock);
                ret = vmx_vcpu_run(&vm->vcpus[vcpu_id]);
                return ret; 
            }
            break;
        }
        default:
            break;
    }

    spinlock_unlock(&g_kvm_lock);
    return ret;
}

int64_t kvm_client_close(int32_t fd) {
    spinlock_lock(&g_kvm_lock);
    kvm_vm_t *vm = get_vm_by_fd(fd);
    if (vm) {
        for (int i = 0; i < KVM_MAX_VCPUS; i++) {
            if (vm->vcpus[i].active) {
                vmx_vcpu_destroy(&vm->vcpus[i]);
            }
        }
        vm->used = 0;
    }
    spinlock_unlock(&g_kvm_lock);
    return 0;
}

void *kvm_client_mmap(int32_t fd, uint64_t offset, uint64_t size) {
    (void)size;
    spinlock_lock(&g_kvm_lock);
    kvm_vm_t *vm = get_vm_by_fd(fd);
    void *ret = NULL;
    
    uint32_t vcpu_id = (uint32_t)offset;
    if (vm && vcpu_id < KVM_MAX_VCPUS && vm->vcpus[vcpu_id].active) {
        void *run_page = vm->vcpus[vcpu_id].run;
        if (run_page) {
            uint64_t cr3 = process_get_current_cr3();
            uint64_t start = (uint64_t)(uintptr_t)run_page;
            uint64_t aligned_start = start & ~4095ULL;
            paging_set_user_access(cr3, aligned_start, 4096, 1);
            ret = run_page;
        }
    }
    
    spinlock_unlock(&g_kvm_lock);
    return ret;
}
