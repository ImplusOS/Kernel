#include "Syscall_Main.h"
#include "Syscall_VM.h"
#include "mmu/Paging_Main.h"
#include "Core/process/ProcessManager.h"
#include "Core/usercopy/Usercopy.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define USER_VA_LIMIT 0x4000000000ULL

int64_t syscall_vm_mprotect(uint64_t addr, uint64_t len, uint64_t prot)
{
    enum { PROT_READ = 1u, PROT_WRITE = 2u, PROT_EXEC = 4u };
    if (len == 0 || (addr & (PAGE_SIZE - 1ULL)) != 0u ||
        (prot & ~(uint64_t)(PROT_READ | PROT_WRITE | PROT_EXEC)) != 0u)
        return -22;

    uint64_t cr3 = process_get_current_cr3();
    if (cr3 == 0) return -14;
    uint64_t flags = 0u;
    if ((prot & (PROT_READ | PROT_WRITE | PROT_EXEC)) != 0u)
        flags |= PAGE_USER;
    if ((prot & PROT_WRITE) != 0u) flags |= PAGE_RW;
    if ((prot & PROT_EXEC) == 0u) flags |= PAGE_NX;
    return paging_protect_user_range(cr3, addr, len, flags) < 0 ? -14 : 0;
}

int64_t syscall_vm_munmap(uint64_t addr, uint64_t len)
{
    if (len == 0) {
        return 0;
    }

    int rc = process_user_munmap((void *)(uintptr_t)addr, len);
    return (rc < 0) ? -14 : 0;
}

#define LINUX_MREMAP_MAYMOVE   1u
#define LINUX_MREMAP_FIXED     2u
#define LINUX_MREMAP_DONTUNMAP 4u

#define VM_STAGE_CHUNK (64u * 1024u)

/* mremap() - TODO_Chromium_LinuxABI.md section 3.2.
 *
 * Simplification: this always relocates the mapping (allocate a fresh
 * region of `new_size`, copy min(old_size,new_size) bytes, release the old
 * one) rather than attempting to grow the existing mapping in place first.
 * process_user_mmap()'s backing allocator (process_user_alloc(), a bump/
 * free-list allocator - see ProcessManager_Create.c) does not expose an
 * "extend this allocation if nothing follows it" primitive, and building
 * one safely (racing against concurrent allocations under the process
 * table lock) is significant additional risk for a benefit that rarely
 * matters in practice: glibc/PartitionAlloc only reach for mremap() on
 * large mmap-backed chunks and already pass MREMAP_MAYMOVE expecting the
 * pointer may change.
 *
 * MREMAP_FIXED / MREMAP_DONTUNMAP (TODO_Chromium_LinuxABI.md section 4):
 * MREMAP_FIXED honours an explicit target address by allocating page
 * frames directly at `new_addr` (same primitive as an mmap() MAP_FIXED),
 * copying the overlap, and then releasing the source unless
 * MREMAP_DONTUNMAP was also given. MREMAP_DONTUNMAP without MREMAP_FIXED
 * relocates via the normal allocator but keeps the old mapping intact.
 */
int64_t syscall_vm_mremap(uint64_t old_addr, uint64_t old_size,
                          uint64_t new_size, uint64_t flags)
{
    return syscall_vm_mremap5(old_addr, old_size, new_size, flags, 0u);
}

int64_t syscall_vm_mremap5(uint64_t old_addr, uint64_t old_size,
                           uint64_t new_size, uint64_t flags,
                           uint64_t new_addr)
{
    enum { EINVAL_ = -22, ENOMEM_ = -12, EFAULT_ = -14 };

    const int want_fixed = (flags & LINUX_MREMAP_FIXED) != 0u;
    const int want_dontunmap = (flags & LINUX_MREMAP_DONTUNMAP) != 0u;

    if (old_addr == 0u || (old_addr & (PAGE_SIZE - 1ULL)) != 0u ||
        new_size == 0u) {
        return EINVAL_;
    }
    /* MREMAP_FIXED and MREMAP_DONTUNMAP both imply MREMAP_MAYMOVE on Linux. */
    if (want_fixed) {
        if ((flags & LINUX_MREMAP_MAYMOVE) == 0u) {
            return EINVAL_;
        }
        if (new_addr == 0u || (new_addr & (PAGE_SIZE - 1ULL)) != 0u ||
            new_addr < 0x1000u || new_size > USER_VA_LIMIT - new_addr) {
            return EINVAL_;
        }
        uint64_t copy_len = old_size < new_size ? old_size : new_size;
        if (copy_len != 0u &&
            !process_user_buffer_is_valid((const void *)(uintptr_t)old_addr,
                                          copy_len)) {
            return EFAULT_;
        }
        uint64_t cr3 = process_get_current_cr3();
        if (cr3 == 0u ||
            paging_map_user_range_alloc(cr3, new_addr, new_size,
                                        PAGE_RW | PAGE_USER) < 0) {
            return ENOMEM_;
        }
        if (copy_len != 0u) {
            uint8_t *stage = (uint8_t *)malloc(VM_STAGE_CHUNK);
            if (stage == NULL) {
                (void)process_user_munmap((void *)(uintptr_t)new_addr, new_size);
                return ENOMEM_;
            }
            uint64_t done = 0;
            while (done < copy_len) {
                uint64_t chunk = copy_len - done;
                if (chunk > VM_STAGE_CHUNK) {
                    chunk = VM_STAGE_CHUNK;
                }
                if (copy_from_user_trusted(stage,
                        (const uint8_t *)(uintptr_t)old_addr + done, chunk) != 0u ||
                    copy_to_user_trusted((uint8_t *)(uintptr_t)new_addr + done,
                                         stage, chunk) != 0u) {
                    free(stage);
                    (void)process_user_munmap((void *)(uintptr_t)new_addr, new_size);
                    return EFAULT_;
                }
                done += chunk;
            }
            free(stage);
        }
        if (!want_dontunmap) {
            (void)process_user_munmap((void *)(uintptr_t)old_addr, old_size);
            (void)process_user_free((void *)(uintptr_t)old_addr);
        }
        return (int64_t)new_addr;
    }

    uint64_t old_aligned = (old_size + PAGE_SIZE - 1ULL) & ~(PAGE_SIZE - 1ULL);
    uint64_t new_aligned = (new_size + PAGE_SIZE - 1ULL) & ~(PAGE_SIZE - 1ULL);
    if (new_aligned == old_aligned && !want_dontunmap) {
        return (int64_t)old_addr;
    }
    if ((flags & LINUX_MREMAP_MAYMOVE) == 0u) {
        /* We never grow in place (see comment above), so a caller that
         * refuses relocation can only be satisfied when shrinking, which
         * is safe to just report as "moved to the same address". */
        if (new_aligned > old_aligned) {
            return ENOMEM_;
        }
        return (int64_t)old_addr;
    }

    uint64_t copy_len = old_size < new_size ? old_size : new_size;
    if (copy_len != 0u &&
        !process_user_buffer_is_valid((const void *)(uintptr_t)old_addr, copy_len)) {
        return EFAULT_;
    }

    /* Relocate into a lazily-committed reservation: the copy below faults in
     * only the pages actually carrying data (kernel-mode demand-zero), so a
     * large glibc/PartitionAlloc mremap() does not eagerly commit new_size
     * bytes of physical memory. */
    void *new_region = process_user_reserve(new_size);
    if (new_region == NULL) {
        return ENOMEM_;
    }

    if (copy_len != 0u) {
        uint8_t *stage = (uint8_t *)malloc(VM_STAGE_CHUNK);
        if (stage == NULL) {
            (void)process_user_free(new_region);
            return ENOMEM_;
        }
        uint64_t done = 0;
        while (done < copy_len) {
            uint64_t chunk = copy_len - done;
            if (chunk > VM_STAGE_CHUNK) {
                chunk = VM_STAGE_CHUNK;
            }
            if (copy_from_user_trusted(stage,
                                       (const uint8_t *)(uintptr_t)old_addr + done,
                                       chunk) != 0u ||
                copy_to_user_trusted((uint8_t *)new_region + done, stage,
                                     chunk) != 0u) {
                free(stage);
                (void)process_user_free(new_region);
                return EFAULT_;
            }
            done += chunk;
        }
        free(stage);
    }

    /* Release the source. process_user_munmap() drops the physical pages for
     * both tracked heap allocations and lazily-committed arena reservations
     * (which have no tracking slot); process_user_free() additionally frees a
     * tracked slot's address-space bookkeeping. MREMAP_DONTUNMAP keeps the
     * source in place per its contract. */
    if (!want_dontunmap) {
        (void)process_user_munmap((void *)(uintptr_t)old_addr, old_size);
        (void)process_user_free((void *)(uintptr_t)old_addr);
    }

    return (int64_t)(uintptr_t)new_region;
}
