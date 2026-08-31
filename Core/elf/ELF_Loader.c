#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "ELF_Loader.h"
#include "mmu/Paging_Main.h"
#include "Core/process/ProcessManager.h" /* USER_CODE_BASE / USER_CODE_LIMIT */

#include "Drivers/Module/DriverManager.h"
#include "MemoryManagement/Memory_Main.h"
#include "Core/sync/Spinlock.h"
#include "Core/vfs/VFS.h"
#include "Debug/serial/Serial.h"

#include <stddef.h>
#include <stdint.h>

#define ELF_MAGIC_0 0x7F
#define ELF_MAGIC_1 'E'
#define ELF_MAGIC_2 'L'
#define ELF_MAGIC_3 'F'

#define ET_EXEC 2

#define EI_OSABI 7
#define ELFOSABI_LINUX 3
#define ELFOSABI_LINUX_ENTRY_HINT 0x4000000000ULL
#define ET_DYN  3

#define EM_X86_64  62
#define EM_AARCH64 183

#define PT_LOAD    1
#define PT_DYNAMIC 2
#define PT_INTERP  3

#define PF_X 0x1
#define PF_W 0x2
#define PF_R 0x4

#define SHT_RELA 4

#define SHN_UNDEF 0

#define R_X86_64_NONE      0
#define R_X86_64_64        1
#define R_X86_64_PC32      2
#define R_X86_64_COPY      5
#define R_X86_64_GLOB_DAT  6
#define R_X86_64_JUMP_SLOT 7
#define R_X86_64_RELATIVE  8
#define R_X86_64_PC64      24
#define R_X86_64_PLT32     4
#define R_X86_64_32        10
#define R_X86_64_32S       11

#define R_AARCH64_NONE           0
#define R_AARCH64_ABS64          257
#define R_AARCH64_ABS32          258
#define R_AARCH64_ABS16          259
#define R_AARCH64_PREL64         260
#define R_AARCH64_PREL32         261
#define R_AARCH64_PREL16         262

#define R_AARCH64_MOVW_UABS_G0        263
#define R_AARCH64_MOVW_UABS_G0_NC     264
#define R_AARCH64_MOVW_UABS_G1        265
#define R_AARCH64_MOVW_UABS_G1_NC     266
#define R_AARCH64_MOVW_UABS_G2        267
#define R_AARCH64_MOVW_UABS_G2_NC     268
#define R_AARCH64_MOVW_UABS_G3        269

#define R_AARCH64_ADR_PREL_PG_HI21    275
#define R_AARCH64_ADR_PREL_PG_HI21_NC 276
#define R_AARCH64_ADD_ABS_LO12_NC     277
#define R_AARCH64_LDST8_ABS_LO12_NC   278

#define R_AARCH64_JUMP26              282
#define R_AARCH64_CALL26              283

#define R_AARCH64_LDST16_ABS_LO12_NC  284
#define R_AARCH64_LDST32_ABS_LO12_NC  285
#define R_AARCH64_LDST64_ABS_LO12_NC  286
#define R_AARCH64_LDST128_ABS_LO12_NC 299

#define R_AARCH64_COPY       1024
#define R_AARCH64_GLOB_DAT   1025
#define R_AARCH64_JUMP_SLOT  1026
#define R_AARCH64_RELATIVE   1027

#define ELF_PAGE_SIZE 4096ULL

typedef struct {
    unsigned char e_ident[16];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint64_t e_entry;
    uint64_t e_phoff;
    uint64_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;
} Elf64_Ehdr;

typedef struct {
    uint32_t p_type;
    uint32_t p_flags;
    uint64_t p_offset;
    uint64_t p_vaddr;
    uint64_t p_paddr;
    uint64_t p_filesz;
    uint64_t p_memsz;
    uint64_t p_align;
} Elf64_Phdr;

typedef struct {
    uint32_t sh_name;
    uint32_t sh_type;
    uint64_t sh_flags;
    uint64_t sh_addr;
    uint64_t sh_offset;
    uint64_t sh_size;
    uint32_t sh_link;
    uint32_t sh_info;
    uint64_t sh_addralign;
    uint64_t sh_entsize;
} Elf64_Shdr;

typedef struct {
    uint64_t r_offset;
    uint64_t r_info;
    int64_t  r_addend;
} Elf64_Rela;

typedef struct {
    uint32_t st_name;
    uint8_t  st_info;
    uint8_t  st_other;
    uint16_t st_shndx;
    uint64_t st_value;
    uint64_t st_size;
} Elf64_Sym;

static inline uint32_t elf64_r_type(uint64_t info)
{
    return (uint32_t)(info & 0xFFFFFFFFu);
}

static inline uint32_t elf64_r_sym(uint64_t info)
{
    return (uint32_t)(info >> 32);
}

static uint64_t align_down_u64(uint64_t value, uint64_t align)
{
    if (align == 0) {
        return value;
    }
    return value & ~(align - 1u);
}

static uint64_t align_up_u64(uint64_t value, uint64_t align)
{
    if (align == 0) {
        return value;
    }
    return (value + align - 1u) & ~(align - 1u);
}

static bool range_in_file(uint64_t file_size, uint64_t offset, uint64_t size)
{
    if (offset > file_size) {
        return false;
    }
    return size <= (file_size - offset);
}

static bool in_vaddr_range(uint64_t start,
                           uint64_t size,
                           uint64_t min_vaddr,
                           uint64_t max_vaddr)
{
    if (size == 0) {
        return false;
    }
    if (start < min_vaddr) {
        return false;
    }
    if (start >= max_vaddr) {
        return false;
    }
    if (size > (max_vaddr - start)) {
        return false;
    }
    return true;
}

static bool runtime_range_ok(uint64_t image_start,
                             uint64_t image_span,
                             uint64_t addr,
                             uint64_t size)
{
    if (size == 0 || addr < image_start) {
        return false;
    }
    uint64_t offset = addr - image_start;
    if (offset > image_span) {
        return false;
    }
    return size <= (image_span - offset);
}

static const char *g_last_elf_error = NULL;

const char *elf_loader_last_error(void)
{
    return g_last_elf_error;
}

#define ELF_SET_ERROR(msg) do { g_last_elf_error = (msg); } while(0)

static bool resolve_kernel_symbol(const char *name, uint64_t *addr_out)
{
    if (name == NULL || addr_out == NULL) {
        return false;
    }

    if (strcmp(name, "memcpy") == 0) {
        *addr_out = (uint64_t)(uintptr_t)memcpy;
        return true;
    }
    if (strcmp(name, "memmove") == 0) {
        *addr_out = (uint64_t)(uintptr_t)memmove;
        return true;
    }
    if (strcmp(name, "memset") == 0) {
        *addr_out = (uint64_t)(uintptr_t)memset;
        return true;
    }
    if (strcmp(name, "memcmp") == 0) {
        *addr_out = (uint64_t)(uintptr_t)memcmp;
        return true;
    }
    if (strcmp(name, "strlen") == 0) {
        *addr_out = (uint64_t)(uintptr_t)strlen;
        return true;
    }
    if (strcmp(name, "strcmp") == 0) {
        *addr_out = (uint64_t)(uintptr_t)strcmp;
        return true;
    }
    if (strcmp(name, "strncmp") == 0) {
        *addr_out = (uint64_t)(uintptr_t)strncmp;
        return true;
    }
    if (strcmp(name, "strcasecmp") == 0) {
        *addr_out = (uint64_t)(uintptr_t)strcasecmp;
        return true;
    }
    if (strcmp(name, "strncpy") == 0) {
        *addr_out = (uint64_t)(uintptr_t)strncpy;
        return true;
    }
    if (strcmp(name, "malloc") == 0) {
        *addr_out = (uint64_t)(uintptr_t)malloc;
        return true;
    }
    if (strcmp(name, "free") == 0) {
        *addr_out = (uint64_t)(uintptr_t)free;
        return true;
    }

    return false;
}

static bool elf_validate_ehdr_arch(const Elf64_Ehdr *ehdr,
                                   uint64_t file_size,
                                   bool allow_exec)
{
    if (ehdr->e_ident[0] != ELF_MAGIC_0 ||
        ehdr->e_ident[1] != ELF_MAGIC_1 ||
        ehdr->e_ident[2] != ELF_MAGIC_2 ||
        ehdr->e_ident[3] != ELF_MAGIC_3) {
        return false;
    }

#if defined(PLATFORM_ARM64)
    if (ehdr->e_machine != EM_AARCH64) {
        return false;
    }
#elif defined(PLATFORM_X86_64)
    if (ehdr->e_machine != EM_X86_64) {
        return false;
    }
#else
#error "PLATFORM_ARM64 or PLATFORM_X86_64 must be defined"
#endif

    if (ehdr->e_type == ET_DYN) {
    } else if (allow_exec && ehdr->e_type == ET_EXEC) {
    } else {
        return false;
    }

    if (ehdr->e_phentsize != sizeof(Elf64_Phdr) || ehdr->e_phnum == 0) {
        return false;
    }

    uint64_t phdr_table_bytes = (uint64_t)ehdr->e_phnum *
                                (uint64_t)ehdr->e_phentsize;
    if (!range_in_file(file_size, ehdr->e_phoff, phdr_table_bytes)) {
        return false;
    }

    return true;
}

static void safe_memcpy(void *dest, const void *src, size_t n) {
    volatile uint8_t *d = (volatile uint8_t *)dest;
    const volatile uint8_t *s = (const volatile uint8_t *)src;
    for (size_t i = 0; i < n; i++) {
        d[i] = s[i];
    }
}

static void sync_instruction_cache_range(uintptr_t start, uintptr_t end)
{
#if defined(PLATFORM_ARM64)
    if (end <= start) {
        return;
    }

    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    uint64_t dcache_line = 4ULL << ((ctr >> 16) & 0xFULL);
    uint64_t icache_line = 4ULL << (ctr & 0xFULL);
    uintptr_t dstart = start & ~(uintptr_t)(dcache_line - 1ULL);
    uintptr_t istart = start & ~(uintptr_t)(icache_line - 1ULL);

    for (uintptr_t p = dstart; p < end; p += dcache_line) {
        __asm__ volatile("dc cvau, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb ish" ::: "memory");
    for (uintptr_t p = istart; p < end; p += icache_line) {
        __asm__ volatile("ic ivau, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb ish; isb" ::: "memory");
#else
    (void)start;
    (void)end;
#endif
}

static bool copy_to_address_space(uint64_t target_cr3,
                                  uint64_t dst_vaddr,
                                  const uint8_t *src,
                                  uint64_t size,
                                  bool executable)
{
    uint64_t copied = 0;
    while (copied < size) {
        uint64_t va = dst_vaddr + copied;
        uint64_t phys = paging_virt_to_phys(target_cr3, va);
        if (phys == 0) {
            return false;
        }

        uint64_t page_left = ELF_PAGE_SIZE - (va & (ELF_PAGE_SIZE - 1ULL));
        uint64_t chunk = size - copied;
        if (chunk > page_left) {
            chunk = page_left;
        }

        uint8_t *dst = (uint8_t *)(uintptr_t)phys;
        memcpy(dst, src + copied, (size_t)chunk);
        if (executable) {
            sync_instruction_cache_range((uintptr_t)dst,
                                         (uintptr_t)(dst + chunk));
        }
        copied += chunk;
    }
    return true;
}

static bool zero_address_space(uint64_t target_cr3,
                               uint64_t dst_vaddr,
                               uint64_t size,
                               bool executable)
{
    uint64_t cleared = 0;
    while (cleared < size) {
        uint64_t va = dst_vaddr + cleared;
        uint64_t phys = paging_virt_to_phys(target_cr3, va);
        if (phys == 0) {
            return false;
        }

        uint64_t page_left = ELF_PAGE_SIZE - (va & (ELF_PAGE_SIZE - 1ULL));
        uint64_t chunk = size - cleared;
        if (chunk > page_left) {
            chunk = page_left;
        }

        uint8_t *dst = (uint8_t *)(uintptr_t)phys;
        memset(dst, 0, (size_t)chunk);
        if (executable) {
            sync_instruction_cache_range((uintptr_t)dst,
                                         (uintptr_t)(dst + chunk));
        }
        cleared += chunk;
    }
    return true;
}

static bool sym_value_for_reloc(const Elf64_Sym *symbols,
                                uint32_t         symbol_count,
                                const char       *symbol_strings,
                                uint64_t         symbol_strings_size,
                                uint32_t         sym_index,
                                uint64_t         load_bias,
                                bool             must_have_symbol,
                                uint64_t        *sym_addr_out)
{
    if (sym_index == 0) {
        if (must_have_symbol) {
            return false;
        }
        *sym_addr_out = 0;
        return true;
    }
    if (symbols == NULL || sym_index >= symbol_count) {
        return false;
    }
    const Elf64_Sym *sym = &symbols[sym_index];
    if (sym->st_shndx == SHN_UNDEF) {
        const char *name = NULL;
        if (symbol_strings != NULL && sym->st_name < symbol_strings_size) {
            name = symbol_strings + sym->st_name;
        }
        if (resolve_kernel_symbol(name, sym_addr_out)) {
            return true;
        }
        if (must_have_symbol) {
            return false;
        }
        *sym_addr_out = 0;
        return true;
    }

    *sym_addr_out = load_bias + sym->st_value;
    return true;
}

static bool aarch64_patch_movzk(void *insn_ptr, uint16_t value)
{
    uint32_t insn;
    safe_memcpy(&insn, insn_ptr, sizeof(insn));
    insn = (insn & ~(0xFFFFu << 5)) | ((uint32_t)value << 5);
    safe_memcpy(insn_ptr, &insn, sizeof(insn));
    return true;
}

static bool aarch64_patch_adrp(void *insn_ptr, int64_t page_delta)
{
    if (page_delta < -(1LL << 20) || page_delta >= (1LL << 20)) {
        return false;
    }
    uint32_t imm21 = (uint32_t)(page_delta & 0x1FFFFF);
    uint32_t immlo = imm21 & 0x3u;
    uint32_t immhi = (imm21 >> 2) & 0x7FFFFu;

    uint32_t insn;
    safe_memcpy(&insn, insn_ptr, sizeof(insn));
    insn = (insn & ~((0x7FFFFu << 5) | (0x3u << 29)))
         | (immhi << 5)
         | (immlo << 29);
    safe_memcpy(insn_ptr, &insn, sizeof(insn));
    return true;
}

static bool aarch64_patch_add_imm12(void *insn_ptr, uint64_t value)
{
    uint32_t imm12 = (uint32_t)(value & 0xFFFu);
    uint32_t insn;
    safe_memcpy(&insn, insn_ptr, sizeof(insn));
    insn = (insn & ~(0xFFFu << 10)) | (imm12 << 10);
    safe_memcpy(insn_ptr, &insn, sizeof(insn));
    return true;
}

static bool aarch64_patch_ldst_imm12(void *insn_ptr,
                                     uint64_t value,
                                     unsigned shift_bits)
{
    uint64_t byte_off = value & 0xFFFu;
    if (shift_bits > 0 && (byte_off & ((1u << shift_bits) - 1u)) != 0) {
        return false;
    }
    uint32_t imm12 = (uint32_t)(byte_off >> shift_bits);
    uint32_t insn;
    safe_memcpy(&insn, insn_ptr, sizeof(insn));
    insn = (insn & ~(0xFFFu << 10)) | (imm12 << 10);
    safe_memcpy(insn_ptr, &insn, sizeof(insn));
    return true;
}

static bool aarch64_patch_branch26(void *insn_ptr, int64_t byte_delta)
{
    if (byte_delta & 3) {
        return false;
    }
    int64_t instr_delta = byte_delta >> 2;
    if (instr_delta < -(1LL << 25) || instr_delta >= (1LL << 25)) {
        return false;
    }
    uint32_t imm26 = (uint32_t)(instr_delta & 0x3FFFFFFu);
    uint32_t insn;
    safe_memcpy(&insn, insn_ptr, sizeof(insn));
    insn = (insn & ~0x3FFFFFFu) | imm26;
    safe_memcpy(insn_ptr, &insn, sizeof(insn));
    return true;
}

static bool apply_relocations(const uint8_t     *image,
                               uint64_t           file_size,
                               const Elf64_Shdr  *rel_sec,
                               const Elf64_Shdr  *shdrs,
                               uint16_t           shnum,
                               uint64_t           runtime_start,
                               uint64_t           image_span,
                               uint64_t           load_bias,
                               uint32_t          *reloc_budget)
{
    if (rel_sec->sh_entsize != sizeof(Elf64_Rela) ||
        (rel_sec->sh_size % rel_sec->sh_entsize) != 0) {
        return false;
    }

    if (!range_in_file(file_size, rel_sec->sh_offset, rel_sec->sh_size)) {
        return false;
    }

    uint64_t rel_count64 = rel_sec->sh_size / rel_sec->sh_entsize;
    if (rel_count64 > UINT32_MAX) {
        return false;
    }
    uint32_t rel_count = (uint32_t)rel_count64;

    if (rel_count > *reloc_budget) {
        return false;
    }
    *reloc_budget -= rel_count;

    const Elf64_Rela *rels = (const Elf64_Rela *)(image + rel_sec->sh_offset);

    const Elf64_Sym *symbols      = NULL;
    uint32_t         symbol_count = 0;
    const char      *symbol_strings = NULL;
    uint64_t         symbol_strings_size = 0;

    if (rel_sec->sh_link < shnum) {
        const Elf64_Shdr *sym_sec = &shdrs[rel_sec->sh_link];
        if (sym_sec->sh_entsize == sizeof(Elf64_Sym) &&
            range_in_file(file_size, sym_sec->sh_offset, sym_sec->sh_size)) {
            symbols      = (const Elf64_Sym *)(image + sym_sec->sh_offset);
            symbol_count = (uint32_t)(sym_sec->sh_size / sym_sec->sh_entsize);

            if (sym_sec->sh_link < shnum) {
                const Elf64_Shdr *str_sec = &shdrs[sym_sec->sh_link];
                if (range_in_file(file_size, str_sec->sh_offset, str_sec->sh_size)) {
                    symbol_strings = (const char *)(image + str_sec->sh_offset);
                    symbol_strings_size = str_sec->sh_size;
                }
            }
        }
    }

    for (uint32_t r = 0; r < rel_count; ++r) {
        const Elf64_Rela *rela = &rels[r];
        uint32_t type      = elf64_r_type(rela->r_info);
        uint32_t sym_index = elf64_r_sym(rela->r_info);

#if defined(PLATFORM_ARM64)
        switch (type) {

        case R_AARCH64_NONE:
            continue;

        case R_AARCH64_ABS64:
        case R_AARCH64_GLOB_DAT:
        case R_AARCH64_JUMP_SLOT:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint64_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias,
                                         true,
                                         &sym_addr)) {
                    return false;
                }
                uint64_t res64 = (sym_addr == 0 && type != R_AARCH64_ABS64)
                                 ? 0ULL
                                 : (uint64_t)((int64_t)sym_addr + rela->r_addend);
                safe_memcpy((void *)(uintptr_t)where_addr, &res64, sizeof(uint64_t));
            }
            continue;

        case R_AARCH64_ABS32:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                int64_t result = (int64_t)sym_addr + rela->r_addend;
                if (result < 0 || result > (int64_t)UINT32_MAX) {
                    return false;
                }
                uint32_t res32 = (uint32_t)result;
                safe_memcpy((void *)(uintptr_t)where_addr, &res32, sizeof(uint32_t));
            }
            continue;

        case R_AARCH64_ABS16:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint16_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                int64_t result = (int64_t)sym_addr + rela->r_addend;
                if (result < 0 || result > (int64_t)UINT16_MAX) {
                    return false;
                }
                uint16_t res16 = (uint16_t)result;
                safe_memcpy((void *)(uintptr_t)where_addr, &res16, sizeof(uint16_t));
            }
            continue;

        case R_AARCH64_PREL64:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint64_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                uint64_t res64 = (uint64_t)((int64_t)sym_addr
                                            + rela->r_addend
                                            - (int64_t)where_addr);
                safe_memcpy((void *)(uintptr_t)where_addr, &res64, sizeof(uint64_t));
            }
            continue;

        case R_AARCH64_PREL32:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                int64_t result = (int64_t)sym_addr
                               + rela->r_addend
                               - (int64_t)where_addr;
                if (result < INT32_MIN || result > INT32_MAX) {
                    return false;
                }
                uint32_t res32 = (uint32_t)(int32_t)result;
                safe_memcpy((void *)(uintptr_t)where_addr, &res32, sizeof(uint32_t));
            }
            continue;

        case R_AARCH64_PREL16:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint16_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                int64_t result = (int64_t)sym_addr
                               + rela->r_addend
                               - (int64_t)where_addr;
                if (result < INT16_MIN || result > INT16_MAX) {
                    return false;
                }
                uint16_t res16 = (uint16_t)(int16_t)result;
                safe_memcpy((void *)(uintptr_t)where_addr, &res16, sizeof(uint16_t));
            }
            continue;

        case R_AARCH64_RELATIVE:
            {
                if (sym_index != 0) {
                    return false;
                }
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint64_t))) {
                    return false;
                }
                uint64_t res64 = (uint64_t)((int64_t)load_bias + rela->r_addend);
                safe_memcpy((void *)(uintptr_t)where_addr, &res64, sizeof(uint64_t));
            }
            continue;

        case R_AARCH64_MOVW_UABS_G0:
        case R_AARCH64_MOVW_UABS_G0_NC:
        case R_AARCH64_MOVW_UABS_G1:
        case R_AARCH64_MOVW_UABS_G1_NC:
        case R_AARCH64_MOVW_UABS_G2:
        case R_AARCH64_MOVW_UABS_G2_NC:
        case R_AARCH64_MOVW_UABS_G3:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                uint64_t value = (uint64_t)((int64_t)sym_addr + rela->r_addend);

                unsigned shift;
                switch (type) {
                case R_AARCH64_MOVW_UABS_G0:
                case R_AARCH64_MOVW_UABS_G0_NC: shift =  0; break;
                case R_AARCH64_MOVW_UABS_G1:
                case R_AARCH64_MOVW_UABS_G1_NC: shift = 16; break;
                case R_AARCH64_MOVW_UABS_G2:
                case R_AARCH64_MOVW_UABS_G2_NC: shift = 32; break;
                default:                         shift = 48; break;
                }

                bool nc = (type == R_AARCH64_MOVW_UABS_G0_NC ||
                           type == R_AARCH64_MOVW_UABS_G1_NC ||
                           type == R_AARCH64_MOVW_UABS_G2_NC);
                if (!nc) {
                    uint64_t upper_mask = ~((uint64_t)0xFFFFu << shift)
                                         & (shift < 48 ? ~0ULL : 0ULL);
                    if (shift < 48 && (value & upper_mask) != 0) {
                        return false;
                    }
                }

                uint16_t imm16 = (uint16_t)((value >> shift) & 0xFFFFu);
                if (!aarch64_patch_movzk((void *)(uintptr_t)where_addr, imm16)) {
                    return false;
                }
            }
            continue;

        case R_AARCH64_ADR_PREL_PG_HI21:
        case R_AARCH64_ADR_PREL_PG_HI21_NC:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                uint64_t target = (uint64_t)((int64_t)sym_addr + rela->r_addend);

                int64_t page_delta = (int64_t)(target >> 12)
                                   - (int64_t)(where_addr >> 12);

                if (type == R_AARCH64_ADR_PREL_PG_HI21) {
                    if (page_delta < -(1LL << 20) || page_delta >= (1LL << 20)) {
                        return false;
                    }
                }

                if (!aarch64_patch_adrp((void *)(uintptr_t)where_addr, page_delta)) {
                    return false;
                }
            }
            continue;

        case R_AARCH64_ADD_ABS_LO12_NC:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                uint64_t value = (uint64_t)((int64_t)sym_addr + rela->r_addend);
                if (!aarch64_patch_add_imm12((void *)(uintptr_t)where_addr, value)) {
                    return false;
                }
            }
            continue;

        case R_AARCH64_LDST8_ABS_LO12_NC:
        case R_AARCH64_LDST16_ABS_LO12_NC:
        case R_AARCH64_LDST32_ABS_LO12_NC:
        case R_AARCH64_LDST64_ABS_LO12_NC:
        case R_AARCH64_LDST128_ABS_LO12_NC:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                uint64_t value = (uint64_t)((int64_t)sym_addr + rela->r_addend);

                unsigned shift;
                switch (type) {
                case R_AARCH64_LDST8_ABS_LO12_NC:   shift = 0; break;
                case R_AARCH64_LDST16_ABS_LO12_NC:  shift = 1; break;
                case R_AARCH64_LDST32_ABS_LO12_NC:  shift = 2; break;
                case R_AARCH64_LDST64_ABS_LO12_NC:  shift = 3; break;
                default:                             shift = 4; break;
                }

                if (!aarch64_patch_ldst_imm12((void *)(uintptr_t)where_addr,
                                              value, shift)) {
                    return false;
                }
            }
            continue;

        case R_AARCH64_JUMP26:
        case R_AARCH64_CALL26:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                int64_t byte_delta = (int64_t)sym_addr
                                   + rela->r_addend
                                   - (int64_t)where_addr;
                if (!aarch64_patch_branch26((void *)(uintptr_t)where_addr,
                                            byte_delta)) {
                    return false;
                }
            }
            continue;

        case R_AARCH64_COPY:
            return false;

        default:
            return false;
        }

#elif defined(PLATFORM_X86_64)
        switch (type) {

        case R_X86_64_NONE:
            continue;

        case R_X86_64_64:
        case R_X86_64_GLOB_DAT:
        case R_X86_64_JUMP_SLOT:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint64_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                if (sym_addr == 0 && type != R_X86_64_64) {
                    uint64_t zero = 0;
                    safe_memcpy((void *)(uintptr_t)where_addr, &zero, sizeof(uint64_t));
                    continue;
                }
                uint64_t res64 = (uint64_t)((int64_t)sym_addr + rela->r_addend);
                safe_memcpy((void *)(uintptr_t)where_addr, &res64, sizeof(uint64_t));
            }
            continue;

        case R_X86_64_RELATIVE:
            {
                if (sym_index != 0) {
                    return false;
                }
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint64_t))) {
                    return false;
                }
                uint64_t res64 = (uint64_t)((int64_t)load_bias + rela->r_addend);
                safe_memcpy((void *)(uintptr_t)where_addr, &res64, sizeof(uint64_t));
            }
            continue;

        case R_X86_64_PC32:
        case R_X86_64_PLT32:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                int64_t result = (int64_t)sym_addr
                               + rela->r_addend
                               - (int64_t)where_addr;
                if (result < INT32_MIN || result > INT32_MAX) {
                    return false;
                }
                uint32_t res32 = (uint32_t)(int32_t)result;
                safe_memcpy((void *)(uintptr_t)where_addr, &res32, sizeof(uint32_t));
            }
            continue;

        case R_X86_64_PC64:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint64_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                uint64_t res64 = (uint64_t)((int64_t)sym_addr
                                            + rela->r_addend
                                            - (int64_t)where_addr);
                safe_memcpy((void *)(uintptr_t)where_addr, &res64, sizeof(uint64_t));
            }
            continue;

        case R_X86_64_32:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                int64_t result = (int64_t)sym_addr + rela->r_addend;
                if (result < 0 || result > (int64_t)UINT32_MAX) {
                    return false;
                }
                uint32_t res32 = (uint32_t)result;
                safe_memcpy((void *)(uintptr_t)where_addr, &res32, sizeof(uint32_t));
            }
            continue;
            
        case R_X86_64_32S:
            {
                uint64_t where_addr = load_bias + rela->r_offset;
                if (!runtime_range_ok(runtime_start, image_span,
                                      where_addr, sizeof(uint32_t))) {
                    return false;
                }
                uint64_t sym_addr;
                if (!sym_value_for_reloc(symbols, symbol_count,
                                         symbol_strings, symbol_strings_size,
                                         sym_index, load_bias, true, &sym_addr)) {
                    return false;
                }
                int64_t result = (int64_t)sym_addr + rela->r_addend;
                if (result < INT32_MIN || result > INT32_MAX) {
                    return false;
                }
                uint32_t res32 = (uint32_t)(int32_t)result;
                safe_memcpy((void *)(uintptr_t)where_addr, &res32, sizeof(uint32_t));
            }
            continue;

        case R_X86_64_COPY:
            return false;

        default:
            return false;
        }

#endif

    }

    return true;
}

/* Window at the very top of the user code region reserved for the dynamic
 * linker (PT_INTERP) when the main image is a PIE. The main executable is
 * loaded at USER_CODE_BASE and must stay below this. */
#define ELF_INTERP_BIAS_RESERVE 0x08000000ULL /* 128 MiB */

static bool elf_load_image_biased(uint64_t target_cr3,
                                  const char *path,
                                  const elf_load_policy_t *policy,
                                  uint64_t bias_override,
                                  int is_interp,
                                  elf_loaded_image_info_t *image_out);

bool elf_loader_load_from_path(uint64_t target_cr3,
                               const char *path,
                               const elf_load_policy_t *policy,
                               elf_loaded_image_info_t *image_out)
{
    return elf_load_image_biased(target_cr3, path, policy, 0u, 0, image_out);
}

static bool elf_load_image_biased(uint64_t target_cr3,
                                  const char *path,
                                  const elf_load_policy_t *policy,
                                  uint64_t bias_override,
                                  int is_interp,
                                  elf_loaded_image_info_t *image_out)
{
    g_last_elf_error = NULL;

    if (target_cr3 == 0 || !path || !policy || !image_out) {
        ELF_SET_ERROR("invalid parameter");
        return false;
    }

    vfs_file_t file;
    memset(&file, 0, sizeof(file));
    if (!vfs_find_file(path, &file)) {
        ELF_SET_ERROR("file not found");
        return false;
    }
    
    Elf64_Phdr *phdrs = NULL;

    if (file.size == 0 || (uint64_t)file.size > policy->max_file_size) {
        ELF_SET_ERROR("invalid file size");
        goto fail;
    }

    if ((uint64_t)file.size < sizeof(Elf64_Ehdr)) {
        ELF_SET_ERROR("file too small for ELF header");
        goto fail;
    }

    Elf64_Ehdr ehdr;
    if (!vfs_read_at(&file, 0u, (uint8_t *)&ehdr, sizeof(ehdr))) {
        ELF_SET_ERROR("failed to read ELF header");
        goto fail;
    }

    if (!elf_validate_ehdr_arch(&ehdr, (uint64_t)file.size, true)) {
        ELF_SET_ERROR("ELF header validation failed (arch/type)");
        goto fail;
    }

    {
        uint64_t phdr_table_bytes = (uint64_t)ehdr.e_phnum *
                                    (uint64_t)ehdr.e_phentsize;
        if (ehdr.e_phoff > 0xFFFFFFFFULL ||
            phdr_table_bytes > 0xFFFFFFFFULL) {
            ELF_SET_ERROR("PHDR table overflow");
            goto fail;
        }

        phdrs = (Elf64_Phdr *)malloc((size_t)phdr_table_bytes);
        if (!phdrs) {
            ELF_SET_ERROR("PHDR table alloc failed (out of memory)");
            goto fail;
        }

        if (!vfs_read_at(&file,
                         (uint32_t)ehdr.e_phoff,
                         (uint8_t *)phdrs,
                         (uint32_t)phdr_table_bytes)) {
            ELF_SET_ERROR("failed to read PHDR table");
            goto fail;
        }
    }

    int    load_segments = 0;
    uint64_t phdr_vaddr  = 0;
    char   interp_path[64];
    interp_path[0] = '\0';

    /*
     * Load bias. ET_EXEC keeps its absolute p_vaddr (bias 0). A true PIE /
     * dynamic linker (ET_DYN whose lowest PT_LOAD starts below the user code
     * region, i.e. it was linked at base 0) is relocated: the main image goes
     * at USER_CODE_BASE, the interpreter at a caller-chosen base high in the
     * code region. An ET_DYN that is already linked into the user code region
     * (the in-tree custom ld / test .so's) keeps its addresses, bias 0.
     */
    uint64_t bias = 0u;
    if (ehdr.e_type == ET_DYN) {
        uint64_t lowest_load = UINT64_MAX;
        for (uint16_t i = 0; i < ehdr.e_phnum; ++i) {
            if (phdrs[i].p_type == PT_LOAD && phdrs[i].p_memsz != 0 &&
                phdrs[i].p_vaddr < lowest_load) {
                lowest_load = phdrs[i].p_vaddr;
            }
        }
        if (lowest_load != UINT64_MAX && lowest_load < USER_CODE_BASE) {
            bias = (bias_override != 0u) ? bias_override : USER_CODE_BASE;
        }
    }
    uint64_t max_seg_end = 0u; /* highest biased vaddr+memsz seen (for interp fit check) */

    for (uint16_t i = 0; i < ehdr.e_phnum; ++i) {
        Elf64_Phdr *ph = &phdrs[i];

        if (ph->p_type == PT_INTERP) {
            if (interp_path[0] == '\0' &&
                ph->p_filesz >= 1u && ph->p_filesz <= 63u &&
                ph->p_offset <= 0xFFFFFFFFULL &&
                range_in_file((uint64_t)file.size, ph->p_offset, ph->p_filesz)) {
                if (vfs_read_at(&file, (uint32_t)ph->p_offset,
                                (uint8_t *)interp_path,
                                (uint32_t)ph->p_filesz)) {
                    interp_path[sizeof(interp_path) - 1u] = '\0';
                    if (interp_path[0] != '/') {
                        interp_path[0] = '\0';
                    }
                }
            }
            continue;
        }

        if (ph->p_type != PT_LOAD || ph->p_memsz == 0) {
            continue;
        }

        if (ph->p_memsz < ph->p_filesz) {
            ELF_SET_ERROR("segment: memsz < filesz");
            goto fail;
        }

        uint64_t seg_vaddr = bias + ph->p_vaddr;

        if (!in_vaddr_range(seg_vaddr, ph->p_memsz,
                            policy->min_vaddr, policy->max_vaddr)) {
            ELF_SET_ERROR("segment: vaddr out of range");
            goto fail;
        }

        if (!range_in_file((uint64_t)file.size, ph->p_offset, ph->p_filesz)) {
            ELF_SET_ERROR("segment: offset out of file range");
            goto fail;
        }

        if (ph->p_offset > 0xFFFFFFFFULL || ph->p_filesz > 0xFFFFFFFFULL) {
            ELF_SET_ERROR("segment: offset/filesz overflow");
            goto fail;
        }

        if (seg_vaddr + ph->p_memsz > max_seg_end) {
            max_seg_end = seg_vaddr + ph->p_memsz;
        }

        if (phdr_vaddr == 0 &&
            ehdr.e_phoff >= ph->p_offset &&
            (ehdr.e_phoff - ph->p_offset) < ph->p_filesz) {
            phdr_vaddr = seg_vaddr + (ehdr.e_phoff - ph->p_offset);
        }

        uint64_t seg_flags = PAGE_RW | PAGE_USER;
        if ((ph->p_flags & PF_X) == 0) {
            seg_flags |= PAGE_NX;
        }

        if (paging_map_user_range_alloc(target_cr3,
                                        seg_vaddr,
                                        ph->p_memsz,
                                        seg_flags) < 0) {
            ELF_SET_ERROR("paging map failed");
            goto fail;
        }

        bool executable = ((ph->p_flags & PF_X) != 0);

        /*
         * Stream the segment's file data through a small fixed staging buffer
         * instead of malloc()'ing the whole p_filesz. Large binaries (a
         * dynamically-linked Chromium has ~150-250 MB PT_LOAD segments) would
         * otherwise instantly OOM the kernel heap.
         */
        if (ph->p_filesz > 0) {
            enum { ELF_SEG_CHUNK = 256u * 1024u };
            uint8_t *staging = (uint8_t *)malloc(ELF_SEG_CHUNK);
            if (staging == NULL) {
                ELF_SET_ERROR("segment staging alloc failed (out of memory)");
                goto fail;
            }
            uint64_t done = 0;
            while (done < ph->p_filesz) {
                uint64_t n = ph->p_filesz - done;
                if (n > ELF_SEG_CHUNK) {
                    n = ELF_SEG_CHUNK;
                }
                if (!vfs_read_at(&file,
                                 (uint32_t)(ph->p_offset + done),
                                 staging,
                                 (uint32_t)n)) {
                    free(staging);
                    ELF_SET_ERROR("failed to read segment data");
                    goto fail;
                }
                if (!copy_to_address_space(target_cr3,
                                           seg_vaddr + done,
                                           staging,
                                           n,
                                           executable)) {
                    free(staging);
                    ELF_SET_ERROR("copy to address space failed");
                    goto fail;
                }
                done += n;
            }
            free(staging);
        }
        if (ph->p_memsz > ph->p_filesz) {
            if (!zero_address_space(target_cr3,
                                    seg_vaddr + ph->p_filesz,
                                    ph->p_memsz - ph->p_filesz,
                                    executable)) {
                ELF_SET_ERROR("zero address space failed");
                goto fail;
            }
        }

        ++load_segments;
    }

    if (load_segments == 0) {
        ELF_SET_ERROR("no loadable segments");
        goto fail;
    }

    if (phdr_vaddr == 0) {
        uint64_t phoff = (uint64_t)ehdr.e_phoff;
        uint64_t ph_page = phoff & ~0xFFFULL;
        uint64_t first_vaddr = 0;
        uint64_t first_offset = 0;
        for (uint16_t i = 0; i < ehdr.e_phnum; ++i) {
            const Elf64_Phdr *ph = &phdrs[i];
            if (ph->p_type != PT_LOAD || ph->p_memsz == 0 ||
                ph->p_filesz == 0) {
                continue;
            }
            first_vaddr = ph->p_vaddr;
            first_offset = ph->p_offset;
            break;
        }
        if (ph_page + 0x1000ULL <= first_offset &&
            first_offset <= first_vaddr) {
            uint64_t map_vaddr = bias + first_vaddr - first_offset + ph_page;
            if (paging_map_user_range_alloc(
                    target_cr3, map_vaddr, 0x1000ULL,
                    PAGE_RW | PAGE_USER | PAGE_NX) == 0) {
                uint8_t *hdr_page = (uint8_t *)malloc(0x1000ULL);
                if (hdr_page != NULL) {
                    if (vfs_read_at(&file, (uint32_t)ph_page,
                                    hdr_page, 0x1000u)) {
                        (void)copy_to_address_space(target_cr3, map_vaddr,
                                                    hdr_page, 0x1000u, false);
                        phdr_vaddr = map_vaddr + (phoff & 0xFFFULL);
                    }
                    free(hdr_page);
                }
            }
        }
    }

    uint64_t final_entry = bias + ehdr.e_entry;
    if (final_entry < policy->min_vaddr ||
        final_entry >= policy->max_vaddr) {
        ELF_SET_ERROR("entry point out of valid range");
        goto fail;
    }

    image_out->entry       = final_entry;
    image_out->main_entry  = final_entry;
    image_out->phdr_vaddr  = phdr_vaddr;
    image_out->phent       = ehdr.e_phentsize;
    image_out->phnum       = ehdr.e_phnum;
    image_out->load_bias   = bias;
    image_out->interp_base = 0u;
    image_out->interp_path[0] = '\0';
    image_out->linux_abi   =
        (ehdr.e_ident[EI_OSABI] == ELFOSABI_LINUX) ||
        (ehdr.e_ident[EI_OSABI] == 0 &&
         ehdr.e_entry >= 0x1000 && ehdr.e_entry < ELFOSABI_LINUX_ENTRY_HINT) ? 1u : 0u;

    if (interp_path[0] != '\0' && !is_interp) {
        /* Load the dynamic linker high in the code region, clear of the main
         * image, and hand its (biased) entry back as the process entry. */
        uint64_t interp_bias =
            (policy->max_vaddr - ELF_INTERP_BIAS_RESERVE) & ~0xFFFULL;
        if (max_seg_end > interp_bias) {
            ELF_SET_ERROR("main image overlaps interpreter window");
            goto fail;
        }
        elf_loaded_image_info_t interp_info;
        memset(&interp_info, 0, sizeof(interp_info));
        if (!elf_load_image_biased(target_cr3, interp_path, policy,
                                   interp_bias, 1, &interp_info)) {
            ELF_SET_ERROR("interpreter load failed");
            goto fail;
        }
        if (interp_info.interp_path[0] != '\0') {
            ELF_SET_ERROR("nested interpreter not supported");
            goto fail;
        }
        image_out->entry = interp_info.entry;
        image_out->interp_base = interp_bias;
        memcpy(image_out->interp_path, interp_path, sizeof(interp_path));
    }

    free(phdrs);
    (void)vfs_close_file(&file);
    return true;

fail:
    if (phdrs != NULL) {
        free(phdrs);
    }
    (void)vfs_close_file(&file);
    return false;
}

bool elf_loader_load_from_memory(uint64_t target_cr3,
                                 const void *file_data,
                                 uint64_t file_size,
                                 const elf_load_policy_t *policy,
                                 elf_loaded_image_info_t *image_out)
{
    if (target_cr3 == 0 || !file_data || !policy || !image_out) {
        return false;
    }

    if (file_size == 0 || file_size > policy->max_file_size) {
        return false;
    }

    if (file_size < sizeof(Elf64_Ehdr)) {
        return false;
    }

    const uint8_t  *file_bytes = (const uint8_t *)file_data;
    const Elf64_Ehdr *ehdr     = (const Elf64_Ehdr *)file_bytes;

    if (!elf_validate_ehdr_arch(ehdr, file_size, true)) {
        return false;
    }

    const Elf64_Phdr *phdrs = (const Elf64_Phdr *)(file_bytes + ehdr->e_phoff);
    int    load_segments    = 0;
    uint64_t phdr_vaddr     = 0;
    bool   failed           = false;

    for (uint16_t i = 0; i < ehdr->e_phnum; ++i) {
        const Elf64_Phdr *ph = &phdrs[i];
        if (ph->p_type != PT_LOAD || ph->p_memsz == 0) {
            continue;
        }

        if (ph->p_memsz < ph->p_filesz) {
            failed = true; break;
        }

        if (!in_vaddr_range(ph->p_vaddr, ph->p_memsz,
                            policy->min_vaddr, policy->max_vaddr)) {
            failed = true; break;
        }

        if (!range_in_file(file_size, ph->p_offset, ph->p_filesz)) {
            failed = true; break;
        }

        if (phdr_vaddr == 0 &&
            ehdr->e_phoff >= ph->p_offset &&
            ehdr->e_phoff < (ph->p_offset + ph->p_filesz)) {
            phdr_vaddr = ph->p_vaddr + (ehdr->e_phoff - ph->p_offset);
        }

        uint64_t seg_flags = PAGE_RW | PAGE_USER;
        if ((ph->p_flags & PF_X) == 0) {
            seg_flags |= PAGE_NX;
        }

        if (paging_map_user_range_alloc(target_cr3,
                                        ph->p_vaddr,
                                        ph->p_memsz,
                                        seg_flags) < 0) {
            failed = true; break;
        }

        bool executable = ((ph->p_flags & PF_X) != 0);
        if (ph->p_filesz > 0) {
            if (!copy_to_address_space(target_cr3,
                                       ph->p_vaddr,
                                       file_bytes + ph->p_offset,
                                       ph->p_filesz,
                                       executable)) {
                failed = true; break;
            }
        }
        if (ph->p_memsz > ph->p_filesz) {
            if (!zero_address_space(target_cr3,
                                    ph->p_vaddr + ph->p_filesz,
                                    ph->p_memsz - ph->p_filesz,
                                    executable)) {
                failed = true; break;
            }
        }
        ++load_segments;
    }

    if (failed || load_segments == 0) {
        return false;
    }

    if (ehdr->e_entry < policy->min_vaddr ||
        ehdr->e_entry >= policy->max_vaddr) {
        return false;
    }

    image_out->entry      = ehdr->e_entry;
    image_out->main_entry = ehdr->e_entry;
    image_out->phdr_vaddr = phdr_vaddr;
    image_out->phent      = ehdr->e_phentsize;
    image_out->phnum      = ehdr->e_phnum;
    image_out->interp_path[0] = '\0';
    image_out->linux_abi  =
        (ehdr->e_ident[EI_OSABI] == ELFOSABI_LINUX) ||
        (ehdr->e_ident[EI_OSABI] == 0 &&
         ehdr->e_entry >= 0x1000 && ehdr->e_entry < ELFOSABI_LINUX_ENTRY_HINT) ? 1u : 0u;
    return true;
}

bool elf_loader_load_module_from_memory(const void *file_data,
                                        uint64_t file_size,
                                        const elf_module_load_policy_t *policy,
                                        uint64_t *entry_out)
{
    elf_loaded_module_t module;
    if (!elf_loader_load_module_image_from_memory(file_data, file_size,
                                                  policy, &module)) {
        return false;
    }
    *entry_out = module.entry;
    return true;
}

bool elf_loader_load_module_image_from_memory(const void *file_data,
                                              uint64_t file_size,
                                              const elf_module_load_policy_t *policy,
                                              elf_loaded_module_t *module_out)
{
    if (file_data == NULL || policy == NULL || module_out == NULL) {
        return false;
    }
    if (file_size == 0 || file_size > policy->max_file_size) {
        return false;
    }
    if (file_size < sizeof(Elf64_Ehdr)) {
        return false;
    }

    const uint8_t  *image = (const uint8_t *)file_data;
    const Elf64_Ehdr *ehdr = (const Elf64_Ehdr *)image;

    if (!elf_validate_ehdr_arch(ehdr, file_size, false)) {
        return false;
    }

    const Elf64_Phdr *phdrs =
        (const Elf64_Phdr *)(image + ehdr->e_phoff);

    uint64_t min_vaddr        = 0;
    uint64_t max_vaddr        = 0;
    uint32_t load_segment_count = 0;

    for (uint16_t i = 0; i < ehdr->e_phnum; ++i) {
        const Elf64_Phdr *ph = &phdrs[i];
        if (ph->p_type != PT_LOAD || ph->p_memsz == 0) {
            continue;
        }

        if (ph->p_memsz < ph->p_filesz) {
            return false;
        }
        if (!range_in_file(file_size, ph->p_offset, ph->p_filesz)) {
            return false;
        }

        uint64_t seg_end = ph->p_vaddr + ph->p_memsz;
        if (seg_end < ph->p_vaddr) {
            return false;
        }

        if (load_segment_count == 0) {
            min_vaddr = ph->p_vaddr;
            max_vaddr = seg_end;
        } else {
            if (ph->p_vaddr < min_vaddr) min_vaddr = ph->p_vaddr;
            if (seg_end     > max_vaddr) max_vaddr = seg_end;
        }
        ++load_segment_count;
    }

    if (load_segment_count == 0) {
        return false;
    }

    uint64_t aligned_min = align_down_u64(min_vaddr, ELF_PAGE_SIZE);
    uint64_t aligned_max = align_up_u64(max_vaddr,   ELF_PAGE_SIZE);
    if (aligned_max <= aligned_min) {
        return false;
    }

    uint64_t image_span = aligned_max - aligned_min;
    if (image_span == 0 || image_span > policy->max_image_size) {
        return false;
    }

    uint64_t page_count64 = image_span / ELF_PAGE_SIZE;
    if (page_count64 == 0 || page_count64 > UINT32_MAX) {
        return false;
    }
    uint32_t page_count = (uint32_t)page_count64;

    void *load_base_ptr = alloc_contiguous_pages(page_count, 1);
    if (load_base_ptr == NULL) {
        return false;
    }

    uint64_t runtime_start = (uint64_t)(uintptr_t)load_base_ptr;
    uint64_t load_bias     = runtime_start - aligned_min;

#if defined(PLATFORM_ARM64)
    if (paging_map_kernel_range(runtime_start, image_span, PAGE_RW) < 0) {
        free_contiguous_pages(load_base_ptr, page_count);
        return false;
    }
#endif

    memset(load_base_ptr, 0, (size_t)image_span);

    bool ok     = false;
    bool failed = false;

    for (uint16_t i = 0; i < ehdr->e_phnum && !failed; ++i) {
        const Elf64_Phdr *ph = &phdrs[i];
        if (ph->p_type != PT_LOAD || ph->p_memsz == 0) {
            continue;
        }

        uint64_t dst_addr = load_bias + ph->p_vaddr;
        if (!runtime_range_ok(runtime_start, image_span,
                              dst_addr, ph->p_memsz)) {
            failed = true;
            break;
        }

        uint8_t       *dst = (uint8_t *)(uintptr_t)dst_addr;
        const uint8_t *src = image + ph->p_offset;
        if (ph->p_filesz > 0) {
            memcpy(dst, src, (size_t)ph->p_filesz);
        }
        if (ph->p_memsz > ph->p_filesz) {
            memset(dst + ph->p_filesz, 0,
                   (size_t)(ph->p_memsz - ph->p_filesz));
        }
    }

    if (!failed && ehdr->e_shoff != 0 && ehdr->e_shnum != 0) {
        if (ehdr->e_shentsize != sizeof(Elf64_Shdr)) {
            failed = true;
        } else if (!range_in_file(file_size,
                                  ehdr->e_shoff,
                                  (uint64_t)ehdr->e_shnum *
                                  (uint64_t)ehdr->e_shentsize)) {
            failed = true;
        } else {
            const Elf64_Shdr *shdrs =
                (const Elf64_Shdr *)(image + ehdr->e_shoff);

            uint32_t reloc_budget = policy->max_relocation_count;

            for (uint16_t sec = 0; sec < ehdr->e_shnum && !failed; ++sec) {
                const Elf64_Shdr *rel_sec = &shdrs[sec];
                if (rel_sec->sh_type != SHT_RELA || rel_sec->sh_size == 0) {
                    continue;
                }

                if (!apply_relocations(image,
                                       file_size,
                                       rel_sec,
                                       shdrs,
                                       ehdr->e_shnum,
                                       runtime_start,
                                       image_span,
                                       load_bias,
                                       &reloc_budget)) {
                    failed = true;
                }
            }
        }
    }

    if (!failed) {
        sync_instruction_cache_range((uintptr_t)load_base_ptr,
                                     (uintptr_t)load_base_ptr + image_span);
    }

    if (!failed) {
        uint64_t entry = load_bias + ehdr->e_entry;
        if (runtime_range_ok(runtime_start, image_span, entry, 1)) {
            module_out->load_base  = load_base_ptr;
            module_out->page_count = page_count;
            module_out->image_span = image_span;
            module_out->entry      = entry;
            ok = true;
        }
    }

    if (!ok) {
        free_contiguous_pages(load_base_ptr, page_count);
    }

    return ok;
}
