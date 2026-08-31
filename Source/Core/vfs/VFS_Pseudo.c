#include "VFS_Pseudo.h"

#include "VFS.h"
#include "DevFS.h"
#include "TmpFS.h"
#include "ProcFS.h"
#include "EtcFS.h"
#include "Core/drm/DRM_Kms.h"

#include <stddef.h>
#include <stdint.h>

/*
 * One row per (init step, optional mount point). Processed top to bottom:
 * `init` is called first when non-NULL, then, when `prefix` is non-NULL,
 * vfs_mount(prefix, get_driver()) is issued. A row may carry only an init
 * (drm_kms_init has no VFS mount of its own -- it is reached through the
 * /dev/dri/card0 node devfs publishes), only a mount, or both.
 *
 * Order is significant and matches the sequence kernel_main.c used before
 * this table existed:
 *   - devfs + drm init before "/dev" is mounted;
 *   - "/dev/shm" (tmpfs) mounts after "/dev" (devfs) so VFS.c's
 *     longest-prefix routing sends paths under /dev/shm to tmpfs;
 *   - each backing filesystem's init runs exactly once even though tmpfs
 *     backs four mount points (/dev/shm, /tmp, /run, /var).
 */
typedef struct {
    void                     (*init)(void);
    const char                *prefix;
    const vfs_driver_t *(*get_driver)(void);
} vfs_pseudo_fs_t;

static const vfs_pseudo_fs_t g_vfs_pseudo_table[] = {
    { devfs_init,   NULL,       NULL                 },
    { drm_kms_init, NULL,       NULL                 },
    { NULL,         "/dev",     devfs_vfs_get_driver },
    { tmpfs_init,   "/dev/shm", tmpfs_vfs_get_driver },
    { NULL,         "/tmp",     tmpfs_vfs_get_driver },
    { NULL,         "/run",     tmpfs_vfs_get_driver },
    { NULL,         "/var",     tmpfs_vfs_get_driver },
    { procfs_init,  "/proc",    procfs_vfs_get_driver },
    { etcfs_init,   "/etc",     etcfs_vfs_get_driver },
};

void vfs_mount_pseudo_filesystems(void)
{
    const uint32_t count =
        (uint32_t)(sizeof(g_vfs_pseudo_table) / sizeof(g_vfs_pseudo_table[0]));

    for (uint32_t i = 0; i < count; ++i) {
        const vfs_pseudo_fs_t *entry = &g_vfs_pseudo_table[i];

        if (entry->init != NULL) {
            entry->init();
        }
        if (entry->prefix != NULL && entry->get_driver != NULL) {
            (void)vfs_mount(entry->prefix, entry->get_driver());
        }
    }
}
