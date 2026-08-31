#include "VMX.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "Debug/serial/Serial.h"
#include "Core/sync/Spinlock.h"
#include <string.h>
#include <stddef.h>

static uint64_t ept_virt_to_phys(void *virt)
{
    return paging_virt_to_phys(paging_get_active_cr3(), (uint64_t)(uintptr_t)virt);
}

static void *ept_phys_to_virt(uint64_t phys)
{
     
    return (void *)(uintptr_t)phys;
}

static void *ept_alloc_table(void)
{
    void *page = alloc_page();
    if (page == NULL) {
        return NULL;
    }
    memset(page, 0, 4096);
    return page;
}

static void ept_free_table(void *table)
{
    if (table != NULL) {
        free_page(table);
    }
}

int ept_create(vmx_vcpu_t *vcpu)
{
    if (vcpu == NULL) {
        return -1;
    }

    void *root = ept_alloc_table();
    if (root == NULL) {
        return -1;
    }

    vcpu->ept_root = root;
    vcpu->ept_root_hpa = ept_virt_to_phys(root);

    return 0;
}

static void ept_free_level(uint64_t *table, int level)
{
    if (table == NULL || level < 1) {
        return;
    }

    for (int i = 0; i < 512; ++i) {
        uint64_t entry = table[i];
        if ((entry & EPT_READ) == 0) {
            continue;
        }
        
        if (level > 1) {
            uint64_t subtable_phys = entry & 0x000FFFFFFFFFF000ULL;
            uint64_t *subtable_virt = (uint64_t *)ept_phys_to_virt(subtable_phys);
            ept_free_level(subtable_virt, level - 1);
        }
    }

    ept_free_table(table);
}

void ept_destroy(vmx_vcpu_t *vcpu)
{
    if (vcpu == NULL || vcpu->ept_root == NULL) {
        return;
    }

    ept_free_level((uint64_t *)vcpu->ept_root, 4);
    vcpu->ept_root = NULL;
    vcpu->ept_root_hpa = 0;
}

 

int ept_map_page(vmx_vcpu_t *vcpu, uint64_t gpa, uint64_t hpa, uint64_t flags)
{
    if (vcpu == NULL || vcpu->ept_root == NULL) {
        return -1;
    }

    uint64_t *pml4 = (uint64_t *)vcpu->ept_root;

     
    uint32_t pml4_idx = (uint32_t)((gpa >> 39) & 0x1FF);
    if ((pml4[pml4_idx] & EPT_READ) == 0) {
        void *pdpt = ept_alloc_table();
        if (pdpt == NULL) return -1;
        pml4[pml4_idx] = ept_virt_to_phys(pdpt) | EPT_RWX;
    }
    uint64_t *pdpt = (uint64_t *)ept_phys_to_virt(pml4[pml4_idx] & 0x000FFFFFFFFFF000ULL);

     
    uint32_t pdpt_idx = (uint32_t)((gpa >> 30) & 0x1FF);
    if ((pdpt[pdpt_idx] & EPT_READ) == 0) {
        void *pd = ept_alloc_table();
        if (pd == NULL) return -1;
        pdpt[pdpt_idx] = ept_virt_to_phys(pd) | EPT_RWX;
    }
    uint64_t *pd = (uint64_t *)ept_phys_to_virt(pdpt[pdpt_idx] & 0x000FFFFFFFFFF000ULL);

     
    uint32_t pd_idx = (uint32_t)((gpa >> 21) & 0x1FF);
    if ((pd[pd_idx] & EPT_READ) == 0) {
        void *pt = ept_alloc_table();
        if (pt == NULL) return -1;
        pd[pd_idx] = ept_virt_to_phys(pt) | EPT_RWX;
    }
    uint64_t *pt = (uint64_t *)ept_phys_to_virt(pd[pd_idx] & 0x000FFFFFFFFFF000ULL);

     
    uint32_t pt_idx = (uint32_t)((gpa >> 12) & 0x1FF);
    pt[pt_idx] = (hpa & 0x000FFFFFFFFFF000ULL) | flags;

    return 0;
}

 

int ept_map_range(vmx_vcpu_t *vcpu, uint64_t gpa, uint64_t hpa,
                  uint64_t size, uint64_t flags)
{
    uint64_t offset = 0;
    while (offset < size) {
        int rc = ept_map_page(vcpu, gpa + offset, hpa + offset, flags);
        if (rc < 0) {
            return rc;
        }
        offset += 4096;
    }
    return 0;
}

 

uint64_t ept_translate(vmx_vcpu_t *vcpu, uint64_t gpa)
{
    if (vcpu == NULL || vcpu->ept_root == NULL) {
        return 0;
    }

    uint64_t *pml4 = (uint64_t *)vcpu->ept_root;
    uint32_t pml4_idx = (uint32_t)((gpa >> 39) & 0x1FF);
    if ((pml4[pml4_idx] & EPT_READ) == 0) return 0;

    uint64_t *pdpt = (uint64_t *)(uintptr_t)(pml4[pml4_idx] & 0x000FFFFFFFFFF000ULL);
    uint32_t pdpt_idx = (uint32_t)((gpa >> 30) & 0x1FF);
    if ((pdpt[pdpt_idx] & EPT_READ) == 0) return 0;

    uint64_t *pd = (uint64_t *)(uintptr_t)(pdpt[pdpt_idx] & 0x000FFFFFFFFFF000ULL);
    uint32_t pd_idx = (uint32_t)((gpa >> 21) & 0x1FF);
    if ((pd[pd_idx] & EPT_READ) == 0) return 0;

    uint64_t *pt = (uint64_t *)(uintptr_t)(pd[pd_idx] & 0x000FFFFFFFFFF000ULL);
    uint32_t pt_idx = (uint32_t)((gpa >> 12) & 0x1FF);
    if ((pt[pt_idx] & EPT_READ) == 0) return 0;

    return (pt[pt_idx] & 0x000FFFFFFFFFF000ULL) | (gpa & 0xFFF);
}
