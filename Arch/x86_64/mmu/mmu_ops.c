#include "interfaces/mmu_ops.h"

#include "mmu/Paging_Main.h"

static const mmu_ops_t g_x86_64_mmu_ops = {
    .init = init_paging,
    .map_mmio = map_mmio_virt,
    .kernel_address_space = paging_get_kernel_cr3,
    .active_address_space = paging_get_active_cr3,
    .switch_address_space = paging_switch_cr3,
    .create_address_space = paging_create_process_space,
    .destroy_address_space = paging_destroy_process_space,
    .map_user_page = paging_map_user_page,
    .map_user_range_alloc = paging_map_user_range_alloc,
    .unmap_range = paging_unmap_range,
    .virt_to_phys = paging_virt_to_phys,
};

const mmu_ops_t *mmu_ops_get(void)
{
    return &g_x86_64_mmu_ops;
}
