#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    void (*init)(void);
    void *(*map_mmio)(uint64_t phys_addr);
    uint64_t (*kernel_address_space)(void);
    uint64_t (*active_address_space)(void);
    void (*switch_address_space)(uint64_t address_space);
    uint64_t (*create_address_space)(void);
    void (*destroy_address_space)(uint64_t address_space);
    int (*map_user_page)(uint64_t address_space, uint64_t virt_addr, uint64_t phys_addr, uint64_t flags);
    int (*map_user_range_alloc)(uint64_t address_space, uint64_t start, uint64_t size, uint64_t flags);
    int (*unmap_range)(uint64_t address_space, uint64_t start, uint64_t size);
    uint64_t (*virt_to_phys)(uint64_t address_space, uint64_t virt_addr);
} mmu_ops_t;

const mmu_ops_t *mmu_ops_get(void);
