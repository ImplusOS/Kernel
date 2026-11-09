#include "Compat/Linux/SysvShm.h"

#include <stdbool.h>
#include <string.h>

#include "Core/memory/SharedMemory.h"
#include "Core/process/ProcessManager.h"
#include "Core/sync/Spinlock.h"
#include "Core/usercopy/Usercopy.h"
#include "Debug/serial/Serial.h"

/* Linux x86_64 errno values (negative returns from the compat layer). */
#define SHM_EINVAL   (-22)
#define SHM_EACCES   (-13)
#define SHM_EEXIST   (-17)
#define SHM_ENOENT   (-2)
#define SHM_ENOMEM   (-12)
#define SHM_ENOSPC   (-28)
#define SHM_EIDRM    (-43)
#define SHM_EFAULT   (-14)

/* SysV IPC constants (x86_64 Linux). */
#define SHM_IPC_PRIVATE 0u
#define SHM_IPC_CREAT   0x200u
#define SHM_IPC_EXCL    0x400u
#define SHM_IPC_RMID    0u
#define SHM_IPC_SET     1u
#define SHM_IPC_STAT    2u
#define SHM_SHM_LOCK    11u
#define SHM_SHM_UNLOCK  12u

#define SYSV_SHM_MAX 64u
#define SYSV_SHM_ATTACH_MAX 256u

typedef struct {
    uint8_t used;
    uint32_t generation;
    int32_t key;
    int32_t handle;      /* shared_memory object handle */
    uint32_t size;
    uint32_t mode;
    uint32_t uid;
    uint32_t gid;
    uint32_t cuid;
    uint32_t cgid;
    int32_t cpid;
    int32_t lpid;
    uint32_t nattch;
    /* IPC_RMID only marks a segment for deletion. Linux keeps it in the ID
     * table (and shmat still succeeds) while any attachment exists, because
     * Xlib's XShmAttach() rmids the segment on the client side *before* the
     * X server has processed the attach request. Removing the table entry
     * immediately made that attach fail with EIDRM, so the server never
     * shared the image and Chromium fell back to full-window PutImage. The
     * segment is destroyed with its last shmdt(). */
    uint8_t rmid;
} sysv_shm_seg_t;

/* One row per successful shmat(): which process attached which segment at
 * which address. nattch is decremented from here on shmdt() and on process
 * teardown, which is what lets an rmid'd segment be destroyed at the right
 * time instead of leaking or vanishing early. */
typedef struct {
    uint8_t used;
    int32_t pid;
    int32_t shmid;
    uint64_t address;
} sysv_shm_attach_t;

static sysv_shm_seg_t g_segs[SYSV_SHM_MAX];
static sysv_shm_attach_t g_atts[SYSV_SHM_ATTACH_MAX];
static spinlock_t g_shm_lock;
static int g_shm_initialized;

static void sysv_shm_init_once(void)
{
    if (g_shm_initialized) {
        return;
    }
    spinlock_init(&g_shm_lock);
    memset(g_segs, 0, sizeof(g_segs));
    memset(g_atts, 0, sizeof(g_atts));
    g_shm_initialized = 1;
}

/* Caller holds the lock. */
static sysv_shm_attach_t *find_attach_locked(int32_t pid, uint64_t address,
                                             uint32_t *index_out)
{
    for (uint32_t i = 0u; i < SYSV_SHM_ATTACH_MAX; ++i) {
        sysv_shm_attach_t *att = &g_atts[i];
        if (att->used && att->pid == pid && att->address == address) {
            if (index_out != NULL) {
                *index_out = i;
            }
            return att;
        }
    }
    return NULL;
}

/* Caller holds the lock. Returns false when the table is full. */
static bool add_attach_locked(int32_t pid, int32_t shmid, uint64_t address)
{
    for (uint32_t i = 0u; i < SYSV_SHM_ATTACH_MAX; ++i) {
        sysv_shm_attach_t *att = &g_atts[i];
        if (!att->used) {
            att->used = 1u;
            att->pid = pid;
            att->shmid = shmid;
            att->address = address;
            return true;
        }
    }
    return false;
}

/* Caller holds the lock. Frees the table entry of a segment whose last
 * attachment is gone and which was already marked IPC_RMID. */
static void reap_segment_locked(sysv_shm_seg_t *seg)
{
    if (!seg->used || !seg->rmid || seg->nattch != 0u) {
        return;
    }
    seg->used = 0u;
    memset(seg, 0, sizeof(*seg));
}

static int32_t make_shmid(uint32_t index, uint32_t generation)
{
    return (int32_t)((generation << 8) | (index + 1u));
}

/* Caller holds the lock. */
static sysv_shm_seg_t *find_shmid_locked(uint64_t shmid, uint32_t *index_out)
{
    uint32_t raw = (uint32_t)shmid;
    uint32_t encoded_index = raw & 0xffu;
    uint32_t generation = raw >> 8;
    if (encoded_index == 0u || generation == 0u) {
        return NULL;
    }
    uint32_t index = encoded_index - 1u;
    if (index >= SYSV_SHM_MAX) {
        return NULL;
    }
    sysv_shm_seg_t *seg = &g_segs[index];
    if (!seg->used || seg->generation != generation) {
        return NULL;
    }
    if (index_out != NULL) {
        *index_out = index;
    }
    return seg;
}

static void shm_trace(const char *what, uint64_t a, uint64_t b, uint64_t c)
{
    static uint32_t budget = 256u;
    if (budget == 0u) {
        return;
    }
    --budget;
    serial_write_string("[sysvshm] ");
    serial_write_string(what);
    serial_write_string(" a=");
    serial_write_uint64(a);
    serial_write_string(" b=");
    serial_write_uint64(b);
    serial_write_string(" c=");
    serial_write_uint64(c);
    serial_write_string("\n");
}

int64_t linux_sysv_shmget(uint64_t key, uint64_t size, uint64_t shmflg)
{
    if (size == 0u || size > 0x7fffffffu) {
        return SHM_EINVAL;
    }

    sysv_shm_init_once();
    spinlock_lock(&g_shm_lock);

    /* A named key returns the existing segment (the common X11 path uses
     * IPC_PRIVATE, so this loop usually does not find anything). */
    if (key != SHM_IPC_PRIVATE) {
        for (uint32_t i = 0u; i < SYSV_SHM_MAX; ++i) {
            sysv_shm_seg_t *seg = &g_segs[i];
            if (!seg->used || (uint32_t)seg->key != (uint32_t)key) {
                continue;
            }
            if ((shmflg & SHM_IPC_CREAT) != 0u &&
                (shmflg & SHM_IPC_EXCL) != 0u) {
                spinlock_unlock(&g_shm_lock);
                return SHM_EEXIST;
            }
            if (size > (uint64_t)seg->size) {
                spinlock_unlock(&g_shm_lock);
                return SHM_EINVAL;
            }
            int32_t id = make_shmid(i, seg->generation);
            spinlock_unlock(&g_shm_lock);
            shm_trace("shmget-existing", key, size, (uint64_t)(uint32_t)id);
            return id;
        }
        if ((shmflg & SHM_IPC_CREAT) == 0u) {
            spinlock_unlock(&g_shm_lock);
            return SHM_ENOENT;
        }
    }

    /* Allocate the backing object outside the table lock: it can take a
     * while (the pages are allocated and zeroed). */
    spinlock_unlock(&g_shm_lock);

    int32_t handle = shared_memory_create((uint32_t)size);
    if (handle <= 0) {
        return SHM_ENOMEM;
    }
    (void)shared_memory_set_public(handle);

    uint32_t uid = 0u;
    uint32_t gid = 0u;
    int32_t self = process_get_current_pid();
    (void)process_get_credentials(self, &uid, &gid);

    spinlock_lock(&g_shm_lock);
    for (uint32_t i = 0u; i < SYSV_SHM_MAX; ++i) {
        sysv_shm_seg_t *seg = &g_segs[i];
        if (seg->used) {
            continue;
        }
        uint32_t generation = seg->generation + 1u;
        if (generation == 0u || generation > 0x7fffffu) {
            generation = 1u;
        }
        memset(seg, 0, sizeof(*seg));
        seg->used = 1u;
        seg->generation = generation;
        seg->key = (int32_t)key;
        seg->handle = handle;
        seg->size = (uint32_t)size;
        seg->mode = (uint32_t)shmflg & 0777u;
        seg->uid = uid;
        seg->gid = gid;
        seg->cuid = uid;
        seg->cgid = gid;
        seg->cpid = self;
        int32_t id = make_shmid(i, generation);
        spinlock_unlock(&g_shm_lock);
        shm_trace("shmget-create", key, size, (uint64_t)(uint32_t)id);
        return id;
    }
    spinlock_unlock(&g_shm_lock);

    (void)shared_memory_release(handle);
    return SHM_ENOSPC;
}

int64_t linux_sysv_shmat(uint64_t shmid, uint64_t shmaddr, uint64_t shmflg)
{
    (void)shmaddr; /* Xlib passes NULL; a hint is ignored, the mapping is
                    * placed wherever the user allocator has room. */
    (void)shmflg;

    sysv_shm_init_once();
    spinlock_lock(&g_shm_lock);
    sysv_shm_seg_t *seg = find_shmid_locked(shmid, NULL);
    if (seg == NULL) {
        spinlock_unlock(&g_shm_lock);
        return SHM_EIDRM;
    }
    int32_t handle = seg->handle;
    int32_t self = process_get_current_pid();
    spinlock_unlock(&g_shm_lock);

    void *p = shared_memory_map_new(handle);
    if (p == NULL) {
        return SHM_ENOMEM;
    }

    /* Re-find the segment: an IPC_RMID (or the owner exiting) may have
     * reaped it while the mapping was being built. The mapping itself holds
     * a reference, so the pages are still valid; it just must be released. */
    spinlock_lock(&g_shm_lock);
    seg = find_shmid_locked(shmid, NULL);
    if (seg == NULL || !add_attach_locked(self, (int32_t)(uint32_t)shmid,
                                          (uint64_t)(uintptr_t)p)) {
        spinlock_unlock(&g_shm_lock);
        (void)shared_memory_unmap_any(p);
        return SHM_EIDRM;
    }
    seg->nattch++;
    seg->lpid = self;
    spinlock_unlock(&g_shm_lock);
    shm_trace("shmat", shmid, (uint64_t)(uintptr_t)p, 0u);
    return (int64_t)(uintptr_t)p;
}

int64_t linux_sysv_shmdt(uint64_t shmaddr)
{
    if (shmaddr == 0u) {
        return SHM_EINVAL;
    }

    sysv_shm_init_once();
    int32_t self = process_get_current_pid();

    spinlock_lock(&g_shm_lock);
    uint32_t att_index = 0u;
    sysv_shm_attach_t *att =
        find_attach_locked(self, shmaddr, &att_index);
    if (att == NULL) {
        spinlock_unlock(&g_shm_lock);
        return SHM_EINVAL;
    }
    int32_t shmid = att->shmid;
    att->used = 0u;
    spinlock_unlock(&g_shm_lock);

    /* The address space mapping belongs to the shared-memory layer. */
    if (!shared_memory_unmap_any((void *)(uintptr_t)shmaddr)) {
        return SHM_EINVAL;
    }

    spinlock_lock(&g_shm_lock);
    sysv_shm_seg_t *seg = find_shmid_locked((uint64_t)(uint32_t)shmid, NULL);
    if (seg != NULL) {
        if (seg->nattch > 0u) {
            seg->nattch--;
        }
        reap_segment_locked(seg);
    }
    spinlock_unlock(&g_shm_lock);
    shm_trace("shmdt", shmaddr, 0u, 0u);
    return 0;
}

/* Layout of glibc's struct shmid_ds on x86_64 (112 bytes):
 *   ipc_perm at 0 (key 0, uid 4, gid 8, cuid 12, cgid 16, mode 20, seq 24),
 *   shm_segsz 48, atime 56, dtime 64, ctime 72, cpid 80, lpid 84,
 *   nattch 88, 16 bytes of reserved padding. */
#define SHM_DS_SIZE 112u

int64_t linux_sysv_shmctl(uint64_t shmid, uint64_t cmd, uint64_t buf)
{
    sysv_shm_init_once();
    spinlock_lock(&g_shm_lock);
    uint32_t index = 0u;
    sysv_shm_seg_t *seg = find_shmid_locked(shmid, &index);
    if (seg == NULL) {
        spinlock_unlock(&g_shm_lock);
        return SHM_EIDRM;
    }

    switch (cmd) {
        case SHM_IPC_STAT: {
            uint8_t ds[SHM_DS_SIZE];
            memset(ds, 0, sizeof(ds));
            uint32_t key = (uint32_t)seg->key;
            uint32_t uid = seg->uid;
            uint32_t gid = seg->gid;
            uint32_t cuid = seg->cuid;
            uint32_t cgid = seg->cgid;
            uint16_t mode = (uint16_t)seg->mode;
            uint16_t seq = (uint16_t)(seg->generation & 0xffffu);
            uint64_t segsz = seg->size;
            int32_t cpid = seg->cpid;
            int32_t lpid = seg->lpid;
            uint64_t nattch = seg->nattch;
            memcpy(ds + 0, &key, 4);
            memcpy(ds + 4, &uid, 4);
            memcpy(ds + 8, &gid, 4);
            memcpy(ds + 12, &cuid, 4);
            memcpy(ds + 16, &cgid, 4);
            memcpy(ds + 20, &mode, 2);
            memcpy(ds + 24, &seq, 2);
            memcpy(ds + 48, &segsz, 8);
            memcpy(ds + 80, &cpid, 4);
            memcpy(ds + 84, &lpid, 4);
            memcpy(ds + 88, &nattch, 8);
            spinlock_unlock(&g_shm_lock);
            if (buf == 0u ||
                copy_to_user((void *)(uintptr_t)buf, ds, sizeof(ds)) != 0u) {
                return SHM_EFAULT;
            }
            return 0;
        }

        case SHM_IPC_SET: {
            uint8_t ds[SHM_DS_SIZE];
            spinlock_unlock(&g_shm_lock);
            if (buf == 0u ||
                copy_from_user(ds, (const void *)(uintptr_t)buf,
                               sizeof(ds)) != 0u) {
                return SHM_EFAULT;
            }
            uint16_t mode = 0u;
            uint32_t uid = 0u;
            uint32_t gid = 0u;
            memcpy(&mode, ds + 20, 2);
            memcpy(&uid, ds + 4, 4);
            memcpy(&gid, ds + 8, 4);
            spinlock_lock(&g_shm_lock);
            seg = find_shmid_locked(shmid, NULL);
            if (seg != NULL) {
                seg->mode = (uint32_t)mode & 0777u;
                seg->uid = uid;
                seg->gid = gid;
            }
            spinlock_unlock(&g_shm_lock);
            return seg != NULL ? 0 : SHM_EIDRM;
        }

        case SHM_IPC_RMID:
            /* Mark for deletion and drop the creation reference. The table
             * entry has to survive while any attachment remains: Xlib rmids
             * on the client before the X server has processed the attach,
             * so the server's shmat() must still find it. The last shmdt()
             * reaps it (see reap_segment_locked()). */
            if (!seg->rmid) {
                seg->rmid = 1u;
                (void)shared_memory_release(seg->handle);
            }
            reap_segment_locked(seg);
            spinlock_unlock(&g_shm_lock);
            shm_trace("shmctl-rmid", shmid, 0u, 0u);
            return 0;

        case SHM_SHM_LOCK:
        case SHM_SHM_UNLOCK:
            spinlock_unlock(&g_shm_lock);
            return 0;

        default:
            spinlock_unlock(&g_shm_lock);
            return SHM_EINVAL;
    }
}

void linux_sysv_shm_cleanup_process(int32_t pid)
{
    if (pid < 0) {
        return;
    }
    sysv_shm_init_once();
    spinlock_lock(&g_shm_lock);

    /* The address-space mappings were already dropped by
     * shared_memory_cleanup_process(); only the SysV bookkeeping is left.
     * Every attachment this process held stops counting now, which may let
     * an IPC_RMID'd segment be reaped. */
    for (uint32_t i = 0u; i < SYSV_SHM_ATTACH_MAX; ++i) {
        sysv_shm_attach_t *att = &g_atts[i];
        if (!att->used || att->pid != pid) {
            continue;
        }
        sysv_shm_seg_t *seg =
            find_shmid_locked((uint64_t)(uint32_t)att->shmid, NULL);
        if (seg != NULL && seg->nattch > 0u) {
            seg->nattch--;
        }
        att->used = 0u;
    }

    for (uint32_t i = 0u; i < SYSV_SHM_MAX; ++i) {
        sysv_shm_seg_t *seg = &g_segs[i];
        if (!seg->used) {
            continue;
        }
        /* SysV segments are kernel objects: they outlive the process that
         * created them. Only IPC_RMID (or the last detach of an rmid'd
         * segment) destroys one. Dropping every segment whose creator exited
         * made a client's segment vanish under the X server that had just
         * been handed its shmid -- the server's shmat then failed with
         * EIDRM and its MIT-SHM bookkeeping dereferenced a NULL descriptor.
         * Just forget which process created/last touched it. */
        if (seg->cpid == pid) {
            seg->cpid = -1;
        }
        if (seg->lpid == pid) {
            seg->lpid = -1;
        }
        reap_segment_locked(seg);
    }
    spinlock_unlock(&g_shm_lock);
}
