#pragma once
#include <stdint.h>

int64_t syscall_vm_mprotect(uint64_t addr, uint64_t len, uint64_t prot);
int64_t syscall_vm_munmap(uint64_t addr, uint64_t len);
int64_t syscall_vm_mremap(uint64_t old_addr, uint64_t old_size,
                          uint64_t new_size, uint64_t flags);
int64_t syscall_vm_mremap5(uint64_t old_addr, uint64_t old_size,
                           uint64_t new_size, uint64_t flags,
                           uint64_t new_addr);
