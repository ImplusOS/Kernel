#pragma once

#include "kernel/system_info.h"
#include "kernel/status.h"

os_status_t sysinfo_get_cpu_info(system_cpu_info_t *out_info);
os_status_t sysinfo_get_memory_info(system_memory_info_t *out_info);
os_status_t sysinfo_get_vmem_info(system_vmem_info_t *out_info);
os_status_t sysinfo_get_disk_count(uint32_t *out_count);
os_status_t sysinfo_get_disk_info(uint32_t index, system_disk_info_t *out_info);
os_status_t sysinfo_get_device_count(uint32_t *out_count);
os_status_t sysinfo_get_device_info(uint32_t index, system_device_t *out_info);
os_status_t sysinfo_get_graphics_info(system_graphics_info_t *out_info);
os_status_t sysinfo_get_arch_info(system_arch_info_t *out_info);
os_status_t sysinfo_get_system_info(system_info_t *out_info);