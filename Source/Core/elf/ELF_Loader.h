#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint64_t max_file_size;
    uint64_t min_vaddr;
    uint64_t max_vaddr;
} elf_load_policy_t;

typedef struct {
    uint64_t max_file_size;
    uint64_t max_image_size;
    uint32_t max_relocation_count;
} elf_module_load_policy_t;

typedef struct {
    void *load_base;
    uint32_t page_count;
    uint64_t image_span;
    uint64_t entry;
} elf_loaded_module_t;

typedef struct {
    uint64_t entry;        /* final entry: interpreter entry if PT_INTERP present, else biased e_entry */
    uint64_t main_entry;   /* main binary entry, load-bias applied (auxv AT_ENTRY) */
    uint64_t phdr_vaddr;   /* runtime address of the program headers, load-bias applied (AT_PHDR) */
    uint64_t phent;
    uint64_t phnum;
    uint64_t load_bias;    /* base the main executable was loaded at (0 for ET_EXEC) */
    uint64_t interp_base;  /* base the PT_INTERP interpreter was loaded at (0 if none; auxv AT_BASE) */
    char     interp_path[64]; /* PT_INTERP interpreter path ("" if none) */
    uint8_t  linux_abi;
} elf_loaded_image_info_t;

const char *elf_loader_last_error(void);

bool elf_loader_load_from_path(uint64_t target_cr3,
                               const char *path,
                               const elf_load_policy_t *policy,
                               elf_loaded_image_info_t *image_out);
bool elf_loader_load_from_memory(uint64_t target_cr3,
                                 const void *file_data,
                                 uint64_t file_size,
                                 const elf_load_policy_t *policy,
                                 elf_loaded_image_info_t *image_out);
bool elf_loader_load_module_from_memory(const void *file_data,
                                        uint64_t file_size,
                                        const elf_module_load_policy_t *policy,
                                        uint64_t *entry_out);
bool elf_loader_load_module_image_from_memory(const void *file_data,
                                              uint64_t file_size,
                                              const elf_module_load_policy_t *policy,
                                              elf_loaded_module_t *module_out);
