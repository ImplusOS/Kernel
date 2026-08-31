#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "kernel/boot_info.h"

void hal_cpu_halt(void);
void hal_cpu_pause(void);
void hal_cpu_enable_interrupts(void);
void hal_cpu_disable_interrupts(void);
uint64_t hal_cpu_save_interrupts(void);
void hal_cpu_restore_interrupts(uint64_t state);
void hal_mmu_invalidate_tlb(uintptr_t addr);
uint64_t hal_cpu_read_cr(int reg);
void hal_cpu_write_cr(int reg, uint64_t value);
void hal_cpu_memory_barrier(void);
void hal_io_delay(void);
uint64_t hal_cpu_read_msr(uint32_t msr);
void hal_cpu_write_msr(uint32_t msr, uint64_t value);
void hal_cpu_get_id(uint32_t leaf, uint32_t subleaf, uint32_t *eax, uint32_t *ebx, uint32_t *ecx, uint32_t *edx);
void hal_cpu_get_gdt_ptr(void *ptr);
void hal_cpu_invalidate_caches(void);
void hal_arch_switch_stack(uintptr_t sp);
__attribute__((noreturn))
void hal_arch_switch_stack_and_jump(uintptr_t sp,
                                    void (*entry)(BOOT_INFO *),
                                    BOOT_INFO *boot_info);
uint64_t hal_cpu_get_current_el(void);
void hal_cpu_set_vbar(void *vbar);
uint64_t hal_cpu_read_fs_base(void);
void hal_cpu_write_fs_base(uint64_t val);
uint64_t hal_cpu_read_gs_base(void);
void hal_cpu_write_gs_base(uint64_t val);
void hal_cpu_save_fpu(uint8_t *state);
void hal_cpu_restore_fpu(uint8_t *state);
