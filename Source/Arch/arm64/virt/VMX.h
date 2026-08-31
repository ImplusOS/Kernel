#pragma once

#include <stdint.h>
#include "kernel/status.h"

#define KVM_MAX_VCPUS 4
#define VMX_MAX_MEM_SLOTS 16

#define KVM_CREATE_VM               1
#define KVM_CREATE_VCPU             2
#define KVM_SET_USER_MEMORY_REGION  3
#define KVM_RUN                     4
#define KVM_GET_REGS                5
#define KVM_SET_REGS                6
#define KVM_GET_SREGS               7
#define KVM_SET_SREGS               8
#define KVM_TRANSLATE               9
#define KVM_INTERRUPT               10

#define KVM_EXIT_UNKNOWN            0
#define KVM_EXIT_IO                 2
#define KVM_EXIT_MMIO               6
#define KVM_EXIT_HLT                5
#define KVM_EXIT_SHUTDOWN           8
#define KVM_EXIT_INTERNAL_ERROR     17
#define KVM_EXIT_DEBUG              4

typedef struct { uint64_t regs[31]; uint64_t pc; uint64_t pstate; uint64_t sp; } vmx_regs_t;
typedef struct { uint64_t base; uint16_t limit; } vmx_dtable_t;
typedef struct { uint16_t selector; uint64_t base; uint32_t limit; uint32_t access; } vmx_segment_t;
typedef struct {
    uint64_t cr0, cr2, cr3, cr4, efer;
    vmx_segment_t cs, ds, es, fs, gs, ss;
    vmx_segment_t tr, ldt;
    vmx_dtable_t gdt, idt;
} vmx_sregs_t;
typedef struct { uint64_t slot; uint64_t guest_phys_addr; uint64_t memory_size; uint64_t userspace_addr; uint32_t flags; } kvm_userspace_memory_region_t;
typedef struct {
    uint8_t request_interrupt_window;
    uint8_t immediate_exit;
    uint8_t _pad_in[6];
    uint32_t exit_reason;
    uint8_t ready_for_interrupt_injection;
    uint8_t _pad_out[3];
    uint8_t _pad_exit[256];
    uint8_t io_data[64];
} kvm_run_t;

#define KVM_IO_DATA_OFFSET ((uint64_t)__builtin_offsetof(kvm_run_t, io_data))

typedef struct {
    uint32_t slot;
    uint64_t guest_phys_addr;
    uint64_t memory_size;
    uint64_t host_virt_addr;
} vmx_mem_slot_t;

typedef struct {
    uint8_t active;
    uint8_t launched;
    uint8_t _pad[6];
    void *vmxon_region;
    void *vmcs_region;
    uint64_t ept_root_hpa;
    void *ept_root;
    vmx_regs_t guest_regs;
    kvm_run_t *run;
    vmx_mem_slot_t mem_slots[VMX_MAX_MEM_SLOTS];
    uint32_t mem_slot_count;
} vmx_vcpu_t;

static inline int vmx_init(void) { return -1; }
static inline void vmx_shutdown(void) {}
static inline int vmx_is_supported(void) { return 0; }
static inline int vmx_vcpu_create(vmx_vcpu_t *vcpu) { (void)vcpu; return -1; }
static inline void vmx_vcpu_destroy(vmx_vcpu_t *vcpu) { (void)vcpu; }
static inline int vmx_vcpu_run(vmx_vcpu_t *vcpu) { (void)vcpu; return -1; }
static inline int vmx_vcpu_set_regs(vmx_vcpu_t *vcpu, const vmx_regs_t *regs) { (void)vcpu; (void)regs; return -1; }
static inline int vmx_vcpu_get_regs(vmx_vcpu_t *vcpu, vmx_regs_t *regs) { (void)vcpu; (void)regs; return -1; }
static inline int vmx_vcpu_set_sregs(vmx_vcpu_t *vcpu, const vmx_sregs_t *sregs) { (void)vcpu; (void)sregs; return -1; }
static inline int vmx_vcpu_get_sregs(vmx_vcpu_t *vcpu, vmx_sregs_t *sregs) { (void)vcpu; (void)sregs; return -1; }
static inline int vmx_vcpu_add_mem_slot(vmx_vcpu_t *vcpu, const kvm_userspace_memory_region_t *region) { (void)vcpu; (void)region; return -1; }
