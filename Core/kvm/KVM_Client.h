#pragma once
#include <stdint.h>

void kvm_client_init(void);
int64_t kvm_client_open(void);
int64_t kvm_client_ioctl(int32_t fd, uint64_t request, uint64_t arg);
int64_t kvm_client_close(int32_t fd);
void *kvm_client_mmap(int32_t fd, uint64_t offset, uint64_t size);
