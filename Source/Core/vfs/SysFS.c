#include "SysFS.h"

#include <stdio.h>
#include <string.h>

#include "DevFS.h"
#include "Drivers/Module/PCI_Main.h"
#include "Drivers/Module/DisplayManager.h"

#define SYSFS_PATH_MAX 96u
#define SYSFS_TEXT_MAX 320u

typedef enum {
    SYSFS_EMPTY = 0, /* never written: g_sysfs_count bounds the live range */
    SYSFS_DIR,
    SYSFS_FILE,
    SYSFS_LINK,
} sysfs_kind_t;

typedef struct {
    sysfs_kind_t kind;
    char path[SYSFS_PATH_MAX];
    char text[SYSFS_TEXT_MAX]; /* file contents, or the link target */
} sysfs_node_t;

/* One GPU, two nodes: 3 skeleton dirs, 13 entries each (uevent, device/,
 * its uevent, vendor, device, subsystem_vendor, subsystem_device, revision,
 * the subsystem link, and device/drm with card0 + renderD128 in it) plus
 * /sys/bus[/pci] so the subsystem symlink is not dangling. 31 of 48 used,
 * with room for a second adapter without touching the layout. */
#define SYSFS_NODE_MAX 48u
static sysfs_node_t g_sysfs_nodes[SYSFS_NODE_MAX];
static uint32_t g_sysfs_count;

static sysfs_node_t *sysfs_find(const char *path)
{
    for (uint32_t i = 0; i < g_sysfs_count; ++i) {
        if (strcmp(g_sysfs_nodes[i].path, path) == 0) {
            return &g_sysfs_nodes[i];
        }
    }
    return NULL;
}

/* Insert (or, on a second boot-time init, re-find) a node. The whole table is
 * built this way so that sysfs_init() and sysfs_publish_drm_nodes() can be
 * called in either order, more than once, and never duplicate an entry. */
static sysfs_node_t *sysfs_add(sysfs_kind_t kind, const char *path,
                               const char *text)
{
    sysfs_node_t *node = sysfs_find(path);
    if (node != NULL) {
        return node;
    }
    if (g_sysfs_count >= SYSFS_NODE_MAX || strlen(path) >= SYSFS_PATH_MAX) {
        return NULL;
    }
    node = &g_sysfs_nodes[g_sysfs_count];
    memset(node, 0, sizeof(*node));
    ++g_sysfs_count;
    node->kind = kind;
    memcpy(node->path, path, strlen(path) + 1u);
    if (text != NULL) {
        memcpy(node->text, text, strlen(text) + 1u);
    }
    return node;
}

static void sysfs_add_dir(const char *path)
{
    (void)sysfs_add(SYSFS_DIR, path, NULL);
}

static void sysfs_add_file(const char *path, const char *text)
{
    (void)sysfs_add(SYSFS_FILE, path, text);
}

static void sysfs_add_link(const char *path, const char *target)
{
    (void)sysfs_add(SYSFS_LINK, path, target);
}

/* Strip a trailing '/' so "/sys/" and "/sys" name the same node. VFS already
 * normalises before dispatch, but the hooks below are also reached directly
 * from vfs_readlink(), and a strcmp that depends on the caller having
 * normalised is a strcmp waiting to fail. */
static bool sysfs_norm(const char *path, char *buf, uint32_t cap)
{
    if (path == NULL || cap == 0u) {
        return false;
    }
    uint32_t len = (uint32_t)strlen(path);
    while (len > 1u && path[len - 1u] == '/') {
        --len;
    }
    if (len >= cap) {
        return false;
    }
    memcpy(buf, path, len);
    buf[len] = '\0';
    return true;
}

/* ------------------------------------------------------------------ */
/* Content                                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    uint16_t vendor;
    uint16_t device;
    uint16_t subvendor;
    uint16_t subdevice;
    uint8_t revision;
    uint32_t domain;
    uint8_t bus;
    uint8_t dev;
    uint8_t func;
} sysfs_pci_t;

static void sysfs_slot_name(const sysfs_pci_t *pci, char *out, uint32_t cap)
{
    snprintf(out, cap, "%04x:%02x:%02x.%1x",
             (uint32_t)pci->domain, (uint32_t)pci->bus,
             (uint32_t)pci->dev, (uint32_t)pci->func);
}

/* Walk the PCI devices the bus layer enumerated (driver_module_critical, well
 * before fs_init) and pick the display controller -- base class 0x03, which is
 * what QEMU's virtio-gpu and stdvga both report. Its slot and IDs are what
 * goes into every /sys file below, so the answer is the machine's real GPU
 * rather than a hard-coded fiction. */
static bool sysfs_locate_display_pci(sysfs_pci_t *out)
{
    uint32_t count = pci_get_device_count();
    for (uint32_t i = 0; i < count; ++i) {
        const pci_device_t *dev = pci_get_device(i);
        if (dev == NULL || dev->class_code != 0x03u) {
            continue;
        }
        memset(out, 0, sizeof(*out));
        out->domain = 0u;
        out->bus = dev->bus;
        out->dev = dev->device;
        out->func = dev->func;
        out->vendor = dev->vendor_id;
        out->device = dev->device_id;
        out->revision = dev->revision;
        /* subsystem_vendor/device live at 0x2C; the bus layer's scan does not
         * keep them, and libdrm treats an absent pair as an error only if the
         * read itself fails -- fall back to the device IDs if the register
         * reads back blank (some devices do not implement it). */
        uint32_t sub = pci_read_config(dev->bus, dev->device, dev->func, 0x2Cu);
        out->subvendor = (uint16_t)(sub & 0xFFFFu);
        out->subdevice = (uint16_t)((sub >> 16) & 0xFFFFu);
        if (out->subvendor == 0u || out->subvendor == 0xFFFFu) {
            out->subvendor = dev->vendor_id;
            out->subdevice = dev->device_id;
        }
        return true;
    }
    return false;
}

/* Used when the bus layer has not enumerated anything (a build without the
 * PCI module, or a machine with no display class device). The identity is
 * unknown -- zeros so consumers do not mistake it for a real device. */
static void sysfs_default_pci(sysfs_pci_t *out)
{
    memset(out, 0, sizeof(*out));
    /* vendor/device = 0 = unknown; bus/dev/func = 0 = unspecified */
}

static void sysfs_publish_node(uint32_t major_num, uint32_t minor_num,
                               const char *devname, const sysfs_pci_t *pci,
                               const char *driver_name)
{
    char dir[SYSFS_PATH_MAX];
    char file[SYSFS_PATH_MAX];
    char text[SYSFS_TEXT_MAX];
    char slot[32];

    sysfs_slot_name(pci, slot, sizeof(slot));

    /* /sys/dev/char/226:0 -- drmGetDeviceNameFromFd2() reads the uevent here
     * for DEVNAME= and prefixes "/dev/" to it. */
    snprintf(dir, sizeof(dir), "/sys/dev/char/%u:%u",
             (uint32_t)major_num, (uint32_t)minor_num);
    sysfs_add_dir(dir);

    snprintf(text, sizeof(text),
             "MAJOR=%u\nMINOR=%u\nDEVNAME=%s\nDEVTYPE=drm\nPCI_SLOT_NAME=%s\nDRIVER=%s\n",
             (uint32_t)major_num, (uint32_t)minor_num, devname, slot, driver_name);
    snprintf(file, sizeof(file), "%s/uevent", dir);
    sysfs_add_file(file, text);

    /* .../device -- realpath() target of get_pci_path(), and the directory
     * every PCI attribute is read out of. A real directory, not a symlink, so
     * realpath() resolves it to itself and the literal-path fallback libdrm
     * keeps for a failed realpath lands on the same place either way. */
    snprintf(file, sizeof(file), "%s/device", dir);
    sysfs_add_dir(file);

    snprintf(text, sizeof(text),
             "MAJOR=%u\nMINOR=%u\nDEVNAME=%s\nDEVTYPE=drm\n"
             "PCI_SLOT_NAME=%s\nPCI_ID=%04X:%04X\nPCI_CLASS=038000\n"
             "PCI_SUBSYS_ID=%04X:%04X\nDRIVER=%s\n",
             (uint32_t)major_num, (uint32_t)minor_num, devname, slot,
             (uint32_t)pci->vendor, (uint32_t)pci->device,
             (uint32_t)pci->subvendor, (uint32_t)pci->subdevice,
             driver_name);
    snprintf(file, sizeof(file), "%s/device/uevent", dir);
    sysfs_add_file(file, text);

    /* parse_separate_sysfs_files() fopen()s each of these and fscanf()s "%x",
     * so "0x1af4\n" and not "1af4". Mesa's loader_get_linux_pci_field() does
     * strtoll(..., 16) on the same two, which reads the 0x fine. */
    snprintf(text, sizeof(text), "0x%04X\n", (uint32_t)pci->vendor);
    snprintf(file, sizeof(file), "%s/device/vendor", dir);
    sysfs_add_file(file, text);

    snprintf(text, sizeof(text), "0x%04X\n", (uint32_t)pci->device);
    snprintf(file, sizeof(file), "%s/device/device", dir);
    sysfs_add_file(file, text);

    snprintf(text, sizeof(text), "0x%04X\n", (uint32_t)pci->subvendor);
    snprintf(file, sizeof(file), "%s/device/subsystem_vendor", dir);
    sysfs_add_file(file, text);

    snprintf(text, sizeof(text), "0x%04X\n", (uint32_t)pci->subdevice);
    snprintf(file, sizeof(file), "%s/device/subsystem_device", dir);
    sysfs_add_file(file, text);

    /* Only read when DRM_DEVICE_GET_PCI_REVISION is passed; present so that
     * flag does not drop libdrm into parse_config_sysfs_file(), which wants a
     * 256-byte binary config space. */
    snprintf(text, sizeof(text), "0x%02X\n", (uint32_t)pci->revision);
    snprintf(file, sizeof(file), "%s/device/revision", dir);
    sysfs_add_file(file, text);

    /* get_subsystem_type() readlink()s this and matches the last path
     * component against "/pci". Reporting PCI directly (rather than the
     * virtio intermediate Linux has, whose realpath fallback libdrm then has
     * to walk back out of) means drmParseSubsystemType() answers DRM_BUS_PCI
     * first try, and drmProcessPciDevice() -- which always tags the result
     * DRM_BUS_PCI -- agrees with it. */
    snprintf(file, sizeof(file), "%s/device/subsystem", dir);
    sysfs_add_link(file, "../../../../bus/pci");

    /* .../device/drm -- drmNodeIsDRM() only stats it, but
     * drmGetMinorNameForFD() opendir()s it and looks for a "card"/"renderD"
     * prefixed entry, so it has to be a directory that lists both nodes. */
    snprintf(file, sizeof(file), "%s/device/drm", dir);
    sysfs_add_dir(file);

    snprintf(file, sizeof(file), "%s/device/drm/card0", dir);
    sysfs_add_dir(file);

    snprintf(file, sizeof(file), "%s/device/drm/renderD128", dir);
    sysfs_add_dir(file);
}

static void sysfs_publish_input_event_node(uint32_t major_num, uint32_t minor_num,
                                           const char *devname)
{
    char dir[SYSFS_PATH_MAX];
    char file[SYSFS_PATH_MAX];
    char text[SYSFS_TEXT_MAX];

    /* /sys/dev/char/13:64 -- libudev/evdev reads uevent for DEVNAME= */
    snprintf(dir, sizeof(dir), "/sys/dev/char/%u:%u",
             (uint32_t)major_num, (uint32_t)minor_num);
    sysfs_add_dir(dir);

    snprintf(text, sizeof(text),
             "MAJOR=%u\nMINOR=%u\nDEVNAME=%s\nDEVTYPE=input\n",
             (uint32_t)major_num, (uint32_t)minor_num, devname);
    snprintf(file, sizeof(file), "%s/uevent", dir);
    sysfs_add_file(file, text);
}

void sysfs_init(void)
{
    sysfs_add_dir("/sys");
    sysfs_add_dir("/sys/dev");
    sysfs_add_dir("/sys/dev/char");
    sysfs_add_dir("/sys/bus");
    sysfs_add_dir("/sys/bus/pci");

    /* Input event devices: event0 (keyboard), event1 (pointer) */
    sysfs_publish_input_event_node(DEVFS_INPUT_MAJOR, DEVFS_INPUT_EVENT0_MINOR, "input/event0");
    sysfs_publish_input_event_node(DEVFS_INPUT_MAJOR, DEVFS_INPUT_EVENT1_MINOR, "input/event1");
}

void sysfs_publish_drm_nodes(void)
{
    sysfs_init();

    sysfs_pci_t pci;
    if (!sysfs_locate_display_pci(&pci)) {
        sysfs_default_pci(&pci);
    }

    const char *driver_name = display_manager_get_active_driver_name();
    if (driver_name == NULL) {
        driver_name = "implus-display";
    }

    sysfs_publish_node(DEVFS_DRM_MAJOR, DEVFS_DRM_CARD_MINOR, "dri/card0",
                       &pci, driver_name);
    sysfs_publish_node(DEVFS_DRM_MAJOR, DEVFS_DRM_RENDER_MINOR,
                       "dri/renderD128", &pci, driver_name);
}

/* ------------------------------------------------------------------ */
/* vfs_driver_t hooks                                                  */
/* ------------------------------------------------------------------ */

static bool sysfs_vfs_find_file(const char *path, vfs_file_t *out_file)
{
    char norm[SYSFS_PATH_MAX];
    if (out_file == NULL || !sysfs_norm(path, norm, sizeof(norm))) {
        return false;
    }
    sysfs_node_t *node = sysfs_find(norm);
    /* Links deliberately answer "not a file": stat() on one should fall
     * through to opendir(), fail, and give ENOENT for the follow-target, and
     * nothing ever opens .../subsystem for reading -- libdrm readlink()s it. */
    if (node == NULL || node->kind != SYSFS_FILE) {
        return false;
    }
    memset(out_file, 0, sizeof(*out_file));
    out_file->internal_id = (uint64_t)(node - g_sysfs_nodes) + 1u;
    out_file->size = (uint32_t)strlen(node->text);
    out_file->driver_data = node;
    return true;
}

static bool sysfs_vfs_read_at(vfs_file_t *file, uint32_t offset,
                              uint8_t *buffer, uint32_t size)
{
    if (file == NULL || file->driver_data == NULL || buffer == NULL) {
        return false;
    }
    const sysfs_node_t *node = (const sysfs_node_t *)file->driver_data;
    uint32_t total = (uint32_t)strlen(node->text);
    if (offset > total || size > total - offset) {
        return false;
    }
    memcpy(buffer, node->text + offset, size);
    return true;
}

static bool sysfs_vfs_read_file(vfs_file_t *file, uint8_t *buffer)
{
    return sysfs_vfs_read_at(file, 0u, buffer,
                             file != NULL ? file->size : 0u);
}

static bool sysfs_vfs_write_file(vfs_file_t *file, const uint8_t *buffer)
{
    (void)file;
    (void)buffer;
    return false;
}

static bool sysfs_vfs_write_at(vfs_file_t *file, uint32_t offset,
                               const uint8_t *buffer, uint32_t size)
{
    (void)file;
    (void)offset;
    (void)buffer;
    (void)size;
    return false;
}

static bool sysfs_vfs_truncate(vfs_file_t *file, uint32_t new_size)
{
    (void)file;
    (void)new_size;
    return false;
}

static uint32_t sysfs_vfs_get_file_size(vfs_file_t *file)
{
    if (file == NULL) {
        return 0u;
    }
    if (file->driver_data != NULL) {
        const sysfs_node_t *node = (const sysfs_node_t *)file->driver_data;
        return (uint32_t)strlen(node->text);
    }
    return file->size;
}

static bool sysfs_vfs_creat(const char *path)
{
    (void)path;
    return false;
}

static bool sysfs_vfs_mkdir(const char *path)
{
    (void)path;
    return false;
}

static bool sysfs_vfs_unlink(const char *path)
{
    (void)path;
    return false;
}

static bool sysfs_vfs_close_file(vfs_file_t *file)
{
    /* No per-open state: driver_data points into the static table and is
     * cleared by vfs_close_file() itself. Must still exist so
     * vfs_close_file() reports success to its caller. */
    (void)file;
    return true;
}

#define SYSFS_DIR_HANDLE_MAX 8u

typedef struct {
    uint8_t in_use;
    uint32_t dir_index; /* node being enumerated */
    uint32_t cursor;    /* next table index to consider */
} sysfs_dir_t;

static sysfs_dir_t g_sysfs_dirs[SYSFS_DIR_HANDLE_MAX];

static int32_t sysfs_vfs_opendir(const char *path)
{
    char norm[SYSFS_PATH_MAX];
    if (!sysfs_norm(path, norm, sizeof(norm))) {
        return -1;
    }
    sysfs_node_t *node = sysfs_find(norm);
    if (node == NULL || node->kind != SYSFS_DIR) {
        return -1;
    }
    uint32_t index = (uint32_t)(node - g_sysfs_nodes);
    for (uint32_t i = 0; i < SYSFS_DIR_HANDLE_MAX; ++i) {
        if (!g_sysfs_dirs[i].in_use) {
            g_sysfs_dirs[i].in_use = 1u;
            g_sysfs_dirs[i].dir_index = index;
            g_sysfs_dirs[i].cursor = 0u;
            return (int32_t)i;
        }
    }
    return -1;
}

static int32_t sysfs_vfs_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    if (handle < 0 || (uint32_t)handle >= SYSFS_DIR_HANDLE_MAX ||
        !g_sysfs_dirs[handle].in_use || out_entry == NULL) {
        return -1;
    }
    sysfs_dir_t *dir = &g_sysfs_dirs[handle];
    const char *prefix = g_sysfs_nodes[dir->dir_index].path;
    uint32_t prefix_len = (uint32_t)strlen(prefix);

    while (dir->cursor < g_sysfs_count) {
        const sysfs_node_t *node = &g_sysfs_nodes[dir->cursor];
        ++dir->cursor;
        if (node->kind == SYSFS_EMPTY) {
            continue;
        }
        if (strncmp(node->path, prefix, prefix_len) != 0) {
            continue;
        }
        const char *rel = node->path + prefix_len;
        if (rel[0] != '/') {
            continue; /* the directory itself */
        }
        ++rel;
        if (strchr(rel, '/') != NULL) {
            continue; /* a descendant of a subdirectory, not an entry here */
        }
        strncpy(out_entry->name, rel, sizeof(out_entry->name) - 1u);
        out_entry->name[sizeof(out_entry->name) - 1u] = '\0';
        out_entry->size = (node->kind == SYSFS_FILE)
                              ? (uint32_t)strlen(node->text)
                              : 0u;
        out_entry->is_directory = (node->kind == SYSFS_DIR);
        return 1;
    }
    return 0;
}

static int32_t sysfs_vfs_closedir(int32_t handle)
{
    if (handle < 0 || (uint32_t)handle >= SYSFS_DIR_HANDLE_MAX ||
        !g_sysfs_dirs[handle].in_use) {
        return -1;
    }
    g_sysfs_dirs[handle].in_use = 0u;
    return 0;
}

static int32_t sysfs_vfs_readlink(const char *path, char *buf, uint32_t size)
{
    char norm[SYSFS_PATH_MAX];
    if (buf == NULL || size == 0u || !sysfs_norm(path, norm, sizeof(norm))) {
        return -1;
    }
    sysfs_node_t *node = sysfs_find(norm);
    if (node == NULL || node->kind != SYSFS_LINK) {
        return -1;
    }
    uint32_t len = (uint32_t)strlen(node->text);
    if (len >= size) {
        len = size;
    }
    memcpy(buf, node->text, len);
    return (int32_t)len;
}

static void sysfs_vfs_list_root(void)
{
}

static void sysfs_vfs_set_case_sensitive(bool enabled)
{
    (void)enabled;
}

static bool sysfs_vfs_get_case_sensitive(void)
{
    return true;
}

static const vfs_driver_t g_sysfs_vfs_driver = {
    .fs_type = "sysfs",
    .media_kind = VFS_MEDIA_KIND_PSEUDO,
    .prefix = NULL,
    .find_file = sysfs_vfs_find_file,
    .read_file = sysfs_vfs_read_file,
    .write_file = sysfs_vfs_write_file,
    .read_at = sysfs_vfs_read_at,
    .write_at = sysfs_vfs_write_at,
    .truncate = sysfs_vfs_truncate,
    .get_file_size = sysfs_vfs_get_file_size,
    .creat = sysfs_vfs_creat,
    .mkdir = sysfs_vfs_mkdir,
    .opendir = sysfs_vfs_opendir,
    .readdir = sysfs_vfs_readdir,
    .closedir = sysfs_vfs_closedir,
    .close_file = sysfs_vfs_close_file,
    .unlink = sysfs_vfs_unlink,
    .list_root = sysfs_vfs_list_root,
    .set_case_sensitive = sysfs_vfs_set_case_sensitive,
    .get_case_sensitive = sysfs_vfs_get_case_sensitive,
    .readlink = sysfs_vfs_readlink,
};

const vfs_driver_t *sysfs_vfs_get_driver(void)
{
    return &g_sysfs_vfs_driver;
}
