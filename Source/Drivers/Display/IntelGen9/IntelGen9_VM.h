#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Drivers/Module/DriverBinary.h"

/*
 * Gen9.5 software PPGTT/GEM foundation.
 *
 * This API builds and validates 48-bit, four-level Gen8+ GPU page tables in
 * DMA-addressable memory. It deliberately does not install a context, change
 * the GT page-table mode, or issue a GPU TLB invalidate. Those operations
 * belong to the later engine/context bring-up stage.
 */

#define IG9_VM_PAGE_SIZE       4096u
#define IG9_VM_ADDRESS_BITS    48u
#define IG9_VM_ADDRESS_LIMIT   (1ull << IG9_VM_ADDRESS_BITS)

#define IG9_VM_MAP_READ        (1u << 0)
#define IG9_VM_MAP_WRITE       (1u << 1)

typedef struct ig9_vm_space ig9_vm_space_t;
typedef struct ig9_gem_object ig9_gem_object_t;

bool ig9_vm_system_init(const driver_binary_t *api);
ig9_vm_space_t *ig9_vm_create(void);
bool ig9_vm_destroy(ig9_vm_space_t *vm);

bool ig9_vm_va_alloc(ig9_vm_space_t *vm,
                     uint64_t size,
                     uint64_t alignment,
                     uint32_t guard_pages,
                     uint64_t *gpu_va_out);
bool ig9_vm_va_free(ig9_vm_space_t *vm,
                    uint64_t gpu_va,
                    uint64_t size);

bool ig9_vm_map_contiguous(ig9_vm_space_t *vm,
                           uint64_t gpu_va,
                           uint64_t physical_base,
                           uint64_t size,
                           uint32_t flags);
bool ig9_vm_unmap(ig9_vm_space_t *vm,
                  uint64_t gpu_va,
                  uint64_t size);
bool ig9_vm_translate(ig9_vm_space_t *vm,
                      uint64_t gpu_va,
                      uint64_t *physical_out);

uint64_t ig9_vm_root_dma(const ig9_vm_space_t *vm);
uint64_t ig9_vm_generation(const ig9_vm_space_t *vm);
bool ig9_vm_tlb_invalidate_pending(const ig9_vm_space_t *vm);
/* Called only by a later hardware path after it has actually invalidated the
 * PPGTT TLB for the supplied generation. It is not a software acknowledgement. */
void ig9_vm_note_tlb_invalidate(ig9_vm_space_t *vm, uint64_t generation);

ig9_gem_object_t *ig9_gem_create(ig9_vm_space_t *vm,
                                uint64_t size,
                                uint32_t map_flags);
bool ig9_gem_get(ig9_gem_object_t *object);
/* A false return means the object was retained because the mapping could not
 * be proven removed. Callers must not assume its backing pages were freed. */
bool ig9_gem_put(ig9_gem_object_t *object);

void *ig9_gem_cpu_address(const ig9_gem_object_t *object);
uint64_t ig9_gem_gpu_address(const ig9_gem_object_t *object);
uint64_t ig9_gem_physical_address(const ig9_gem_object_t *object);
uint64_t ig9_gem_size(const ig9_gem_object_t *object);

/* Exercises the CPU-side page-table builder and GEM lifetime rules only.
 * It does not submit work to the GPU and does not validate hardware PPGTT. */
bool ig9_vm_software_selftest(const driver_binary_t *api);
