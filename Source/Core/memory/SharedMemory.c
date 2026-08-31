#include "SharedMemory.h"

#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "MemoryManagement/Memory_Main.h"
#include "kernel/status.h"
#include "mmu/Paging_Main.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define SHARED_MEMORY_OBJECT_MAX 256u
#define SHARED_MEMORY_MAPPING_MAX 16u
#define SHARED_MEMORY_MAX_BYTES (64u * 1024u * 1024u)
#define SHARED_MEMORY_HANDLE_INDEX_BITS 8u
#define SHARED_MEMORY_HANDLE_INDEX_MASK 0xffu

typedef struct {
    int32_t pid;
    uint64_t address;
    void *allocation_base;
} shared_mapping_t;

typedef struct {
    uint8_t used;
    uint32_t generation;
    int32_t owner_pid;
    int32_t granted_pid;
    uint32_t size;
    uint32_t page_count;
    uint64_t **pages;
    uint32_t references;
    uint32_t mapping_count;
    shared_mapping_t mappings[SHARED_MEMORY_MAPPING_MAX];
} shared_object_t;

static shared_object_t g_shared_objects[SHARED_MEMORY_OBJECT_MAX];
static spinlock_t g_shared_memory_lock;
static int g_shared_memory_initialized;

static void shared_memory_init_once(void)
{
    if (g_shared_memory_initialized) return;
    spinlock_init(&g_shared_memory_lock);
    memset(g_shared_objects, 0, sizeof(g_shared_objects));
    g_shared_memory_initialized = 1;
}

static int32_t shared_memory_make_handle(uint32_t index, uint32_t generation)
{
    return (int32_t)((generation << SHARED_MEMORY_HANDLE_INDEX_BITS) |
                     (index + 1u));
}

static shared_object_t *shared_memory_find_locked(int32_t handle,
                                                  uint32_t *index_out)
{
    if (handle <= 0) return NULL;
    uint32_t raw = (uint32_t)handle;
    uint32_t encoded_index = raw & SHARED_MEMORY_HANDLE_INDEX_MASK;
    uint32_t generation = raw >> SHARED_MEMORY_HANDLE_INDEX_BITS;
    if (encoded_index == 0u || generation == 0u) return NULL;
    uint32_t index = encoded_index - 1u;
    if (index >= SHARED_MEMORY_OBJECT_MAX) return NULL;
    shared_object_t *object = &g_shared_objects[index];
    if (!object->used || object->generation != generation) return NULL;
    if (index_out) *index_out = index;
    return object;
}

static void shared_memory_destroy_locked(shared_object_t *object)
{
    if (!object || !object->used || object->references != 0u) return;
    for (uint32_t i = 0u; i < object->page_count; ++i) {
        if (object->pages[i]) free_page(object->pages[i]);
    }
    free(object->pages);
    uint32_t generation = object->generation;
    memset(object, 0, sizeof(*object));
    object->generation = generation;
}

int32_t shared_memory_create(uint32_t size)
{
    if (size == 0u || size > SHARED_MEMORY_MAX_BYTES) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }

    uint32_t page_count = (size + (uint32_t)PAGE_SIZE - 1u) /
                          (uint32_t)PAGE_SIZE;
    uint64_t **pages =
        (uint64_t **)malloc((size_t)page_count * sizeof(*pages));
    if (!pages) return (int32_t)OS_STATUS_LIMIT_REACHED;
    memset(pages, 0, (size_t)page_count * sizeof(*pages));

    for (uint32_t i = 0u; i < page_count; ++i) {
        pages[i] = (uint64_t *)alloc_page();
        if (!pages[i]) {
            for (uint32_t j = 0u; j < i; ++j) free_page(pages[j]);
            free(pages);
            return (int32_t)OS_STATUS_LIMIT_REACHED;
        }
        memset(pages[i], 0, PAGE_SIZE);
    }

    int32_t owner_pid = process_get_current_pid();
    if (owner_pid < 0) {
        for (uint32_t i = 0u; i < page_count; ++i) free_page(pages[i]);
        free(pages);
        return (int32_t)OS_STATUS_ACCESS_DENIED;
    }

    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    for (uint32_t index = 0u; index < SHARED_MEMORY_OBJECT_MAX; ++index) {
        shared_object_t *object = &g_shared_objects[index];
        if (object->used) continue;
        uint32_t generation = object->generation + 1u;
        if (generation == 0u || generation > 0x7fffffu) generation = 1u;
        memset(object, 0, sizeof(*object));
        object->used = 1u;
        object->generation = generation;
        object->owner_pid = owner_pid;
        object->granted_pid = -1;
        object->size = size;
        object->page_count = page_count;
        object->pages = pages;
        object->references = 1u;
        int32_t handle = shared_memory_make_handle(index, generation);
        spinlock_unlock(&g_shared_memory_lock);
        return handle;
    }
    spinlock_unlock(&g_shared_memory_lock);

    for (uint32_t i = 0u; i < page_count; ++i) free_page(pages[i]);
    free(pages);
    return (int32_t)OS_STATUS_LIMIT_REACHED;
}

int32_t shared_memory_grant(int32_t handle, int32_t pid)
{
    int32_t caller = process_get_current_pid();
    if (pid <= 0 || caller < 0) return (int32_t)OS_STATUS_INVALID_ARG;

    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    shared_object_t *object = shared_memory_find_locked(handle, NULL);
    if (!object || object->owner_pid != caller) {
        spinlock_unlock(&g_shared_memory_lock);
        return (int32_t)OS_STATUS_ACCESS_DENIED;
    }
    object->granted_pid = pid;
    spinlock_unlock(&g_shared_memory_lock);
    return 0;
}

void *shared_memory_map(int32_t handle)
{
    int32_t caller = process_get_current_pid();
    if (caller < 0) return NULL;

    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    shared_object_t *object = shared_memory_find_locked(handle, NULL);
    if (!object ||
        (object->owner_pid != caller && object->granted_pid != caller) ||
        object->mapping_count >= SHARED_MEMORY_MAPPING_MAX) {
        spinlock_unlock(&g_shared_memory_lock);
        return NULL;
    }
    for (uint32_t i = 0u; i < object->mapping_count; ++i) {
        if (object->mappings[i].pid == caller) {
            void *address = (void *)(uintptr_t)object->mappings[i].address;
            spinlock_unlock(&g_shared_memory_lock);
            return address;
        }
    }
    ++object->references;
    uint32_t size = object->size;
    uint32_t page_count = object->page_count;
    uint64_t **pages = object->pages;
    uint32_t generation = object->generation;
    spinlock_unlock(&g_shared_memory_lock);

    uint32_t allocation_size = size + (uint32_t)PAGE_SIZE - 1u;
    void *allocation_base = process_user_alloc(allocation_size);
    if (!allocation_base) {
        spinlock_lock(&g_shared_memory_lock);
        object = shared_memory_find_locked(handle, NULL);
        if (object && object->generation == generation) {
            --object->references;
            shared_memory_destroy_locked(object);
        }
        spinlock_unlock(&g_shared_memory_lock);
        return NULL;
    }

    uint64_t address =
        ((uint64_t)(uintptr_t)allocation_base + PAGE_SIZE - 1u) & PAGE_MASK;
    uint64_t cr3 = process_get_current_cr3();
    uint32_t mapped_pages = 0u;
    for (; mapped_pages < page_count; ++mapped_pages) {
        if (paging_map_user_page(
                cr3,
                address + (uint64_t)mapped_pages * PAGE_SIZE,
                (uint64_t)(uintptr_t)pages[mapped_pages],
                PAGE_RW | PAGE_USER | PAGE_EXTERNAL) < 0) {
            break;
        }
    }
    if (mapped_pages != page_count) {
        (void)process_user_free(allocation_base);
        spinlock_lock(&g_shared_memory_lock);
        object = shared_memory_find_locked(handle, NULL);
        if (object && object->generation == generation) {
            --object->references;
            shared_memory_destroy_locked(object);
        }
        spinlock_unlock(&g_shared_memory_lock);
        return NULL;
    }

    spinlock_lock(&g_shared_memory_lock);
    object = shared_memory_find_locked(handle, NULL);
    if (!object || object->generation != generation ||
        object->mapping_count >= SHARED_MEMORY_MAPPING_MAX) {
        spinlock_unlock(&g_shared_memory_lock);
        (void)process_user_free(allocation_base);
        spinlock_lock(&g_shared_memory_lock);
        object = shared_memory_find_locked(handle, NULL);
        if (object && object->generation == generation) {
            --object->references;
            shared_memory_destroy_locked(object);
        }
        spinlock_unlock(&g_shared_memory_lock);
        return NULL;
    }
    shared_mapping_t *mapping = &object->mappings[object->mapping_count++];
    mapping->pid = caller;
    mapping->address = address;
    mapping->allocation_base = allocation_base;
    spinlock_unlock(&g_shared_memory_lock);
    return (void *)(uintptr_t)address;
}

int32_t shared_memory_unmap(int32_t handle, void *address)
{
    int32_t caller = process_get_current_pid();
    if (caller < 0 || address == NULL) {
        return (int32_t)OS_STATUS_INVALID_ARG;
    }

    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    shared_object_t *object = shared_memory_find_locked(handle, NULL);
    if (!object) {
        spinlock_unlock(&g_shared_memory_lock);
        return (int32_t)OS_STATUS_NOT_FOUND;
    }

    void *allocation_base = NULL;
    uint32_t generation = object->generation;
    for (uint32_t i = 0u; i < object->mapping_count; ++i) {
        shared_mapping_t *mapping = &object->mappings[i];
        if (mapping->pid != caller ||
            mapping->address != (uint64_t)(uintptr_t)address) {
            continue;
        }
        allocation_base = mapping->allocation_base;
        object->mappings[i] = object->mappings[--object->mapping_count];
        break;
    }
    spinlock_unlock(&g_shared_memory_lock);
    if (!allocation_base) return (int32_t)OS_STATUS_NOT_FOUND;

    (void)process_user_free(allocation_base);

    spinlock_lock(&g_shared_memory_lock);
    object = shared_memory_find_locked(handle, NULL);
    if (object && object->generation == generation) {
        --object->references;
        shared_memory_destroy_locked(object);
    }
    spinlock_unlock(&g_shared_memory_lock);
    return 0;
}

int32_t shared_memory_close(int32_t handle)
{
    int32_t caller = process_get_current_pid();
    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    shared_object_t *object = shared_memory_find_locked(handle, NULL);
    if (!object || object->owner_pid != caller) {
        spinlock_unlock(&g_shared_memory_lock);
        return (int32_t)OS_STATUS_ACCESS_DENIED;
    }
    object->owner_pid = -1;
    object->granted_pid = -1;
    --object->references;
    shared_memory_destroy_locked(object);
    spinlock_unlock(&g_shared_memory_lock);
    return 0;
}

int32_t shared_memory_addref(int32_t handle)
{
    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    shared_object_t *object = shared_memory_find_locked(handle, NULL);
    if (!object) {
        spinlock_unlock(&g_shared_memory_lock);
        return (int32_t)OS_STATUS_NOT_FOUND;
    }
    ++object->references;
    spinlock_unlock(&g_shared_memory_lock);
    return 0;
}

int32_t shared_memory_release(int32_t handle)
{
    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    shared_object_t *object = shared_memory_find_locked(handle, NULL);
    if (!object) {
        spinlock_unlock(&g_shared_memory_lock);
        return (int32_t)OS_STATUS_NOT_FOUND;
    }
    int32_t caller = process_get_current_pid();
    if (object->granted_pid == caller) {
        object->granted_pid = -1;
    }
    if (object->references != 0u) {
        --object->references;
    }
    shared_memory_destroy_locked(object);
    spinlock_unlock(&g_shared_memory_lock);
    return 0;
}

uint32_t shared_memory_size(int32_t handle)
{
    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    shared_object_t *object = shared_memory_find_locked(handle, NULL);
    uint32_t size = object ? object->size : 0u;
    spinlock_unlock(&g_shared_memory_lock);
    return size;
}

void shared_memory_cleanup_process(int32_t pid)
{
    if (pid < 0) return;
    shared_memory_init_once();
    spinlock_lock(&g_shared_memory_lock);
    for (uint32_t index = 0u; index < SHARED_MEMORY_OBJECT_MAX; ++index) {
        shared_object_t *object = &g_shared_objects[index];
        if (!object->used) continue;

        for (uint32_t i = 0u; i < object->mapping_count;) {
            if (object->mappings[i].pid == pid) {
                object->mappings[i] =
                    object->mappings[--object->mapping_count];
                --object->references;
            } else {
                ++i;
            }
        }
        if (object->owner_pid == pid) {
            object->owner_pid = -1;
            object->granted_pid = -1;
            --object->references;
        } else if (object->granted_pid == pid) {
            object->granted_pid = -1;
        }
        shared_memory_destroy_locked(object);
    }
    spinlock_unlock(&g_shared_memory_lock);
}
