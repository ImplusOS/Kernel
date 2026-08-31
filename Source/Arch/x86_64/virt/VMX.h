#pragma once
#ifndef VMX_H
#define VMX_H

#include <stdint.h>
#include <stddef.h>

 
#define IA32_VMX_BASIC                  0x480
#define IA32_VMX_PINBASED_CTLS          0x481
#define IA32_VMX_PROCBASED_CTLS         0x482
#define IA32_VMX_EXIT_CTLS              0x483
#define IA32_VMX_ENTRY_CTLS             0x484
#define IA32_VMX_MISC                   0x485
#define IA32_VMX_CR0_FIXED0             0x486
#define IA32_VMX_CR0_FIXED1             0x487
#define IA32_VMX_CR4_FIXED0             0x488
#define IA32_VMX_CR4_FIXED1             0x489
#define IA32_VMX_PROCBASED_CTLS2        0x48B
#define IA32_VMX_EPT_VPID_CAP           0x48C
#define IA32_VMX_TRUE_PINBASED_CTLS     0x48D
#define IA32_VMX_TRUE_PROCBASED_CTLS    0x48E
#define IA32_VMX_TRUE_EXIT_CTLS         0x48F
#define IA32_VMX_TRUE_ENTRY_CTLS        0x490
#define IA32_FEATURE_CONTROL            0x3A
#define IA32_EFER_MSR                   0xC0000080
#define IA32_APIC_BASE_MSR              0x1B
#define IA32_MTRRCAP                    0xFE
#define IA32_MTRR_DEF_TYPE              0x2FF
#define IA32_PAT_MSR                    0x277
#define IA32_FS_BASE_MSR                0xC0000100
#define IA32_GS_BASE_MSR                0xC0000101
#define IA32_KERNEL_GS_BASE_MSR         0xC0000102
#define IA32_SYSENTER_CS                0x174
#define IA32_SYSENTER_ESP               0x175
#define IA32_SYSENTER_EIP               0x176
#define IA32_TSC_AUX                    0xC0000103

#define FEATURE_CONTROL_LOCKED          (1ULL << 0)
#define FEATURE_CONTROL_VMXON_OUTSIDE   (1ULL << 2)

 
#define CR0_PE (1ULL << 0)
#define CR0_MP (1ULL << 1)
#define CR0_EM (1ULL << 2)
#define CR0_TS (1ULL << 3)
#define CR0_ET (1ULL << 4)
#define CR0_NE (1ULL << 5)
#define CR0_WP (1ULL << 16)
#define CR0_NW (1ULL << 29)
#define CR0_CD (1ULL << 30)
#define CR0_PG (1ULL << 31)

#define CR4_VME        (1ULL << 0)
#define CR4_PVI        (1ULL << 1)
#define CR4_TSD        (1ULL << 2)
#define CR4_DE         (1ULL << 3)
#define CR4_PSE        (1ULL << 4)
#define CR4_PAE        (1ULL << 5)
#define CR4_MCE        (1ULL << 6)
#define CR4_PGE        (1ULL << 7)
#define CR4_OSFXSR     (1ULL << 9)
#define CR4_OSXMMEXCPT (1ULL << 10)
#define CR4_VMXE       (1ULL << 13)
#define CR4_SMXE       (1ULL << 14)
#define CR4_OSXSAVE    (1ULL << 18)

#define EFER_SCE  (1ULL << 0)
#define EFER_LME  (1ULL << 8)
#define EFER_LMA  (1ULL << 10)
#define EFER_NXE  (1ULL << 11)

 
#define PIN_BASED_EXT_INTR_MASK         (1U << 0)
#define PIN_BASED_NMI_EXITING           (1U << 3)
#define PIN_BASED_VIRTUAL_NMIS          (1U << 5)

 
#define CPU_BASED_HLT_EXITING           (1U << 7)
#define CPU_BASED_INVLPG_EXITING        (1U << 9)
#define CPU_BASED_MWAIT_EXITING         (1U << 10)
#define CPU_BASED_RDPMC_EXITING         (1U << 11)
#define CPU_BASED_RDTSC_EXITING         (1U << 12)
#define CPU_BASED_CR3_LOAD_EXITING      (1U << 15)
#define CPU_BASED_CR3_STORE_EXITING     (1U << 16)
#define CPU_BASED_CR8_LOAD_EXITING      (1U << 19)
#define CPU_BASED_CR8_STORE_EXITING     (1U << 20)
#define CPU_BASED_MOV_DR_EXITING        (1U << 23)
#define CPU_BASED_UNCOND_IO_EXITING     (1U << 24)
#define CPU_BASED_USE_IO_BITMAPS        (1U << 25)
#define CPU_BASED_USE_MSR_BITMAPS       (1U << 28)
#define CPU_BASED_MONITOR_EXITING       (1U << 29)
#define CPU_BASED_PAUSE_EXITING         (1U << 30)
#define CPU_BASED_ACTIVATE_SECONDARY    (1U << 31)

 
#define SECONDARY_EXEC_VIRTUALIZE_APIC  (1U << 0)
#define SECONDARY_EXEC_ENABLE_EPT       (1U << 1)
#define SECONDARY_EXEC_RDTSCP           (1U << 3)
#define SECONDARY_EXEC_ENABLE_VPID      (1U << 5)
#define SECONDARY_EXEC_WBINVD_EXITING   (1U << 6)
#define SECONDARY_EXEC_UNRESTRICTED     (1U << 7)
#define SECONDARY_EXEC_ENABLE_INVPCID   (1U << 12)
#define SECONDARY_EXEC_XSAVES           (1U << 20)

 
#define VM_EXIT_HOST_ADDR_SPACE_SIZE    (1U << 9)
#define VM_EXIT_ACK_INTR_ON_EXIT        (1U << 15)
#define VM_EXIT_SAVE_IA32_EFER          (1U << 20)
#define VM_EXIT_LOAD_IA32_EFER          (1U << 21)

 
#define VM_ENTRY_IA32E_MODE             (1U << 9)
#define VM_ENTRY_LOAD_IA32_EFER         (1U << 15)

 
#define EXIT_REASON_EXCEPTION_NMI       0
#define EXIT_REASON_EXTERNAL_INT        1
#define EXIT_REASON_TRIPLE_FAULT        2
#define EXIT_REASON_INIT_SIGNAL         3
#define EXIT_REASON_SIPI                4
#define EXIT_REASON_CPUID               10
#define EXIT_REASON_HLT                 12
#define EXIT_REASON_INVLPG              14
#define EXIT_REASON_RDPMC               15
#define EXIT_REASON_RDTSC               16
#define EXIT_REASON_VMCALL              18
#define EXIT_REASON_VMCLEAR             19
#define EXIT_REASON_VMLAUNCH            20
#define EXIT_REASON_CR_ACCESS           28
#define EXIT_REASON_DR_ACCESS           29
#define EXIT_REASON_IO_INSTRUCTION      30
#define EXIT_REASON_MSR_READ            31
#define EXIT_REASON_MSR_WRITE           32
#define EXIT_REASON_INVALID_GUEST_STATE 33
#define EXIT_REASON_MSR_LOADING         34
#define EXIT_REASON_MWAIT               36
#define EXIT_REASON_MONITOR             39
#define EXIT_REASON_PAUSE               40
#define EXIT_REASON_MCE_DURING_ENTRY    41
#define EXIT_REASON_EPT_VIOLATION       48
#define EXIT_REASON_EPT_MISCONFIG       49
#define EXIT_REASON_INVEPT              50
#define EXIT_REASON_RDTSCP              51
#define EXIT_REASON_PREEMPTION_TIMER    52
#define EXIT_REASON_INVVPID             53
#define EXIT_REASON_WBINVD              54
#define EXIT_REASON_XSETBV              55

 
 
#define VMCS_VPID                       0x0000

 
#define VMCS_GUEST_ES_SEL               0x0800
#define VMCS_GUEST_CS_SEL               0x0802
#define VMCS_GUEST_SS_SEL               0x0804
#define VMCS_GUEST_DS_SEL               0x0806
#define VMCS_GUEST_FS_SEL               0x0808
#define VMCS_GUEST_GS_SEL               0x080A
#define VMCS_GUEST_LDTR_SEL             0x080C
#define VMCS_GUEST_TR_SEL               0x080E

 
#define VMCS_HOST_ES_SEL                0x0C00
#define VMCS_HOST_CS_SEL                0x0C02
#define VMCS_HOST_SS_SEL                0x0C04
#define VMCS_HOST_DS_SEL                0x0C06
#define VMCS_HOST_FS_SEL                0x0C08
#define VMCS_HOST_GS_SEL                0x0C0A
#define VMCS_HOST_TR_SEL                0x0C0C

 
#define VMCS_IO_BITMAP_A                0x2000
#define VMCS_IO_BITMAP_B                0x2002
#define VMCS_MSR_BITMAP                 0x2004
#define VMCS_EPTP                       0x201A
#define VMCS_EPTP_HIGH                  0x201B

 
#define VMCS_GUEST_VMCS_LINK_PTR        0x2800
#define VMCS_GUEST_VMCS_LINK_PTR_HIGH   0x2801
#define VMCS_GUEST_IA32_DEBUGCTL        0x2802
#define VMCS_GUEST_IA32_EFER            0x2806
#define VMCS_GUEST_IA32_PAT             0x2804

 
#define VMCS_HOST_IA32_PAT              0x2C00
#define VMCS_HOST_IA32_EFER             0x2C02

 
#define VMCS_PIN_BASED_CONTROLS         0x4000
#define VMCS_CPU_BASED_CONTROLS         0x4002
#define VMCS_EXCEPTION_BITMAP           0x4004
#define VMCS_PF_ERROR_CODE_MASK         0x4006
#define VMCS_PF_ERROR_CODE_MATCH        0x4008
#define VMCS_CR3_TARGET_COUNT           0x400A
#define VMCS_EXIT_CONTROLS              0x400C
#define VMCS_EXIT_MSR_STORE_COUNT       0x400E
#define VMCS_EXIT_MSR_LOAD_COUNT        0x4010
#define VMCS_ENTRY_CONTROLS             0x4012
#define VMCS_ENTRY_MSR_LOAD_COUNT       0x4014
#define VMCS_ENTRY_INTR_INFO            0x4016
#define VMCS_ENTRY_EXCEPTION_ERRCODE    0x4018
#define VMCS_ENTRY_INSTR_LENGTH         0x401A
#define VMCS_SECONDARY_CPU_CONTROLS     0x401E

 
#define VMCS_GUEST_ES_LIMIT             0x4800
#define VMCS_GUEST_CS_LIMIT             0x4802
#define VMCS_GUEST_SS_LIMIT             0x4804
#define VMCS_GUEST_DS_LIMIT             0x4806
#define VMCS_GUEST_FS_LIMIT             0x4808
#define VMCS_GUEST_GS_LIMIT             0x480A
#define VMCS_GUEST_LDTR_LIMIT           0x480C
#define VMCS_GUEST_TR_LIMIT             0x480E
#define VMCS_GUEST_GDTR_LIMIT           0x4810
#define VMCS_GUEST_IDTR_LIMIT           0x4812
#define VMCS_GUEST_ES_ACCESS            0x4814
#define VMCS_GUEST_CS_ACCESS            0x4816
#define VMCS_GUEST_SS_ACCESS            0x4818
#define VMCS_GUEST_DS_ACCESS            0x481A
#define VMCS_GUEST_FS_ACCESS            0x481C
#define VMCS_GUEST_GS_ACCESS            0x481E
#define VMCS_GUEST_LDTR_ACCESS          0x4820
#define VMCS_GUEST_TR_ACCESS            0x4822
#define VMCS_GUEST_INTERRUPTIBILITY     0x4824
#define VMCS_GUEST_ACTIVITY_STATE       0x4826
#define VMCS_GUEST_SMBASE               0x4828
#define VMCS_GUEST_SYSENTER_CS          0x482A

 
#define VMCS_HOST_SYSENTER_CS           0x4C00

 
#define VMCS_CR0_GUEST_HOST_MASK        0x6000
#define VMCS_CR4_GUEST_HOST_MASK        0x6002
#define VMCS_CR0_READ_SHADOW            0x6004
#define VMCS_CR4_READ_SHADOW            0x6006

 
#define VMCS_GUEST_CR0                  0x6800
#define VMCS_GUEST_CR3                  0x6802
#define VMCS_GUEST_CR4                  0x6804
#define VMCS_GUEST_ES_BASE              0x6806
#define VMCS_GUEST_CS_BASE              0x6808
#define VMCS_GUEST_SS_BASE              0x680A
#define VMCS_GUEST_DS_BASE              0x680C
#define VMCS_GUEST_FS_BASE              0x680E
#define VMCS_GUEST_GS_BASE              0x6810
#define VMCS_GUEST_LDTR_BASE            0x6812
#define VMCS_GUEST_TR_BASE              0x6814
#define VMCS_GUEST_GDTR_BASE            0x6816
#define VMCS_GUEST_IDTR_BASE            0x6818
#define VMCS_GUEST_DR7                  0x681A
#define VMCS_GUEST_RSP                  0x681C
#define VMCS_GUEST_RIP                  0x681E
#define VMCS_GUEST_RFLAGS               0x6820
#define VMCS_GUEST_PENDING_DBG_EXCEPT   0x6822
#define VMCS_GUEST_SYSENTER_ESP         0x6824
#define VMCS_GUEST_SYSENTER_EIP         0x6826

 
#define VMCS_HOST_CR0                   0x6C00
#define VMCS_HOST_CR3                   0x6C02
#define VMCS_HOST_CR4                   0x6C04
#define VMCS_HOST_FS_BASE               0x6C06
#define VMCS_HOST_GS_BASE               0x6C08
#define VMCS_HOST_TR_BASE               0x6C0A
#define VMCS_HOST_GDTR_BASE             0x6C0C
#define VMCS_HOST_IDTR_BASE             0x6C0E
#define VMCS_HOST_SYSENTER_ESP          0x6C10
#define VMCS_HOST_SYSENTER_EIP          0x6C12
#define VMCS_HOST_RSP                   0x6C14
#define VMCS_HOST_RIP                   0x6C16

 
#define VMCS_EXIT_REASON                0x4402
#define VMCS_EXIT_QUALIFICATION         0x6400
#define VMCS_EXIT_INTR_INFO             0x4404
#define VMCS_EXIT_INTR_ERRCODE          0x4406
#define VMCS_IDT_VECTORING_INFO         0x4408
#define VMCS_IDT_VECTORING_ERRCODE      0x440A
#define VMCS_EXIT_INSTR_LENGTH          0x440C
#define VMCS_EXIT_INSTR_INFO            0x440E
#define VMCS_GUEST_PHYS_ADDR            0x2400
#define VMCS_GUEST_LINEAR_ADDR          0x640A

 
#define EPT_MT_UC       0ULL
#define EPT_MT_WC       1ULL
#define EPT_MT_WT       4ULL
#define EPT_MT_WP       5ULL
#define EPT_MT_WB       6ULL

#define EPT_READ        (1ULL << 0)
#define EPT_WRITE       (1ULL << 1)
#define EPT_EXECUTE     (1ULL << 2)
#define EPT_RWX         (EPT_READ | EPT_WRITE | EPT_EXECUTE)

 
#define EPTP_WB         (EPT_MT_WB)
#define EPTP_PAGE_WALK_4 (3ULL << 3)

 
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

 
typedef struct {
    uint64_t rax, rbx, rcx, rdx;
    uint64_t rsi, rdi, rbp, rsp;
    uint64_t r8, r9, r10, r11;
    uint64_t r12, r13, r14, r15;
    uint64_t rip, rflags;
} vmx_regs_t;

 
typedef struct {
    uint64_t base;
    uint32_t limit;
    uint16_t selector;
    uint8_t  type;
    uint8_t  present;
    uint8_t  dpl;
    uint8_t  db;
    uint8_t  s;
    uint8_t  l;
    uint8_t  g;
    uint8_t  avl;
    uint8_t  unusable;
    uint8_t  _pad;
} vmx_segment_t;

 
typedef struct {
    uint64_t base;
    uint16_t limit;
    uint16_t _pad[3];
} vmx_dtable_t;

 
typedef struct {
    vmx_segment_t cs, ds, es, fs, gs, ss;
    vmx_segment_t tr, ldt;
    vmx_dtable_t  gdt, idt;
    uint64_t cr0, cr2, cr3, cr4, cr8;
    uint64_t efer;
    uint64_t apic_base;
} vmx_sregs_t;

 
typedef struct {
    uint32_t slot;
    uint32_t flags;
    uint64_t guest_phys_addr;
    uint64_t memory_size;
    uint64_t userspace_addr;
} kvm_userspace_memory_region_t;

 
typedef struct {
     
    uint8_t  request_interrupt_window;
    uint8_t  immediate_exit;
    uint8_t  _pad_in[6];

     
    uint32_t exit_reason;
    uint8_t  ready_for_interrupt_injection;
    uint8_t  _pad_out[3];

    union {
         
        struct {
            uint8_t  direction;   
            uint8_t  size;        
            uint16_t port;
            uint32_t count;
            uint64_t data_offset;  
        } io;

         
        struct {
            uint64_t phys_addr;
            uint8_t  data[8];
            uint32_t len;
            uint8_t  is_write;
            uint8_t  _pad[3];
        } mmio;

         
        struct {
            uint32_t suberror;
            uint32_t ndata;
            uint64_t data[16];
        } internal;

         
        uint8_t _pad_exit[256];
    };

     
    uint8_t io_data[64];
} kvm_run_t;

#define KVM_IO_DATA_OFFSET  ((uint64_t)__builtin_offsetof(kvm_run_t, io_data))

 
#define VMX_MAX_MEM_SLOTS 16

typedef struct {
    uint32_t slot;
    uint64_t guest_phys_addr;
    uint64_t memory_size;
    uint64_t host_virt_addr;
} vmx_mem_slot_t;

typedef struct {
    uint8_t  active;
    uint8_t  launched;
    uint8_t  _pad[6];

     
    void    *vmxon_region;

     
    void    *vmcs_region;

     
    uint64_t ept_root_hpa;
    void    *ept_root;

     
    vmx_regs_t guest_regs;

     
    kvm_run_t *run;

     
    vmx_mem_slot_t mem_slots[VMX_MAX_MEM_SLOTS];
    uint32_t       mem_slot_count;
} vmx_vcpu_t;

 
int  vmx_init(void);
void vmx_shutdown(void);
int  vmx_is_supported(void);

int  vmx_vcpu_create(vmx_vcpu_t *vcpu);
void vmx_vcpu_destroy(vmx_vcpu_t *vcpu);
int  vmx_vcpu_run(vmx_vcpu_t *vcpu);
int  vmx_vcpu_set_regs(vmx_vcpu_t *vcpu, const vmx_regs_t *regs);
int  vmx_vcpu_get_regs(vmx_vcpu_t *vcpu, vmx_regs_t *regs);
int  vmx_vcpu_set_sregs(vmx_vcpu_t *vcpu, const vmx_sregs_t *sregs);
int  vmx_vcpu_get_sregs(vmx_vcpu_t *vcpu, vmx_sregs_t *sregs);
int  vmx_vcpu_add_mem_slot(vmx_vcpu_t *vcpu, const kvm_userspace_memory_region_t *region);

 
int      ept_create(vmx_vcpu_t *vcpu);
void     ept_destroy(vmx_vcpu_t *vcpu);
int      ept_map_page(vmx_vcpu_t *vcpu, uint64_t gpa, uint64_t hpa, uint64_t flags);
int      ept_map_range(vmx_vcpu_t *vcpu, uint64_t gpa, uint64_t hpa,
                       uint64_t size, uint64_t flags);
uint64_t ept_translate(vmx_vcpu_t *vcpu, uint64_t gpa);

 
extern int  vmx_vmlaunch_asm(vmx_regs_t *guest_regs);
extern int  vmx_vmresume_asm(vmx_regs_t *guest_regs);
extern vmx_regs_t *g_vmx_current_guest_regs;

 
static inline void vmx_vmwrite(uint64_t field, uint64_t value)
{
    uint8_t err;
    __asm__ volatile(
        "vmwrite %1, %2\n\t"
        "setna %0"
        : "=rm"(err)
        : "r"(value), "r"(field)
        : "cc", "memory"
    );
    (void)err;
}

static inline uint64_t vmx_vmread(uint64_t field)
{
    uint64_t value = 0;
    __asm__ volatile(
        "vmread %1, %0"
        : "=r"(value)
        : "r"(field)
        : "cc"
    );
    return value;
}

#include "interfaces/hal_cpu.h"

static inline uint64_t vmx_rdmsr(uint32_t msr)
{
    return hal_cpu_read_msr(msr);
}

static inline void vmx_wrmsr(uint32_t msr, uint64_t value)
{
    hal_cpu_write_msr(msr, value);
}

#endif  
