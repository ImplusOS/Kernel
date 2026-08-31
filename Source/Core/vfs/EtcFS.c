#include "EtcFS.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kernel/config.h"

#define ETCFS_BUFFER_CAP 4096u

typedef struct {
    uint8_t *data;
    uint32_t size;
} etcfs_open_t;

static uint32_t etcfs_format_ip(uint64_t addr_host_order, char *out, uint32_t cap)
{
    return (uint32_t)snprintf(out, cap, "%llu.%llu.%llu.%llu",
        (unsigned long long)((addr_host_order >> 24) & 0xFFu),
        (unsigned long long)((addr_host_order >> 16) & 0xFFu),
        (unsigned long long)((addr_host_order >> 8) & 0xFFu),
        (unsigned long long)(addr_host_order & 0xFFu));
}

static uint32_t etcfs_build_hosts(char *buf, uint32_t cap)
{
    char self_ip[16];
    etcfs_format_ip(OS_CONFIG_NET_IPV4_ADDR, self_ip, sizeof(self_ip));
    return (uint32_t)snprintf(buf, cap,
        "127.0.0.1\tlocalhost\n"
        "::1\t\tlocalhost ip6-localhost ip6-loopback\n"
        "%s\timplusos\n",
        self_ip);
}

static uint32_t etcfs_build_resolv_conf(char *buf, uint32_t cap)
{
    /* OS_CONFIG_NET_IPV4_GATEWAY is 10.0.2.2 by default, which is QEMU
     * user-mode networking's (SLIRP) built-in DNS proxy address - the
     * same host that already serves as the default gateway in this
     * config. If the gateway is overridden to a real router that isn't
     * also a DNS forwarder, this will need to change accordingly. */
    char gw_ip[16];
    etcfs_format_ip(OS_CONFIG_NET_IPV4_GATEWAY, gw_ip, sizeof(gw_ip));
    return (uint32_t)snprintf(buf, cap,
        "nameserver %s\n"
        "options edns0\n",
        gw_ip);
}

/* Static text files that glibc's NSS / dynamic loader / getaddrinfo read
 * at process start (TODO_Chromium_LinuxABI.md section 9.2). Keeping these
 * present - even as fixed content - stops glibc from falling back to
 * "assume a hostile/empty environment" code paths. */
typedef struct {
    const char *path;
    const char *content;
} etcfs_static_file_t;

/* Binary /etc files. /etc/localtime is a real IANA TZif for Etc/UTC (a byte
 * copy of /usr/share/zoneinfo/Etc/UTC, 114 bytes) - previously withheld
 * because a *malformed* TZif crashes glibc's tzset parser (section 4ter,
 * bucket C); a byte-exact valid one is safe and lets glibc/ICU resolve the
 * local zone to UTC instead of falling back to unset. */
typedef struct {
    const char    *path;
    const uint8_t *data;
    uint32_t       len;
} etcfs_binary_file_t;

static const uint8_t g_etcfs_localtime_utc[] = {
    0x54,0x5a,0x69,0x66,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,
    0x00,0x00,0x55,0x54,0x43,0x00,0x54,0x5a,0x69,0x66,0x32,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,
    0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x55,0x54,0x43,0x00,
    0x0a,0x55,0x54,0x43,0x30,0x0a,
};

static const etcfs_binary_file_t g_etcfs_binary_files[] = {
    { "/etc/localtime", g_etcfs_localtime_utc, sizeof(g_etcfs_localtime_utc) },
};

static const etcfs_static_file_t g_etcfs_static_files[] = {
    { "/etc/nsswitch.conf",
      "passwd:     files\n"
      "group:      files\n"
      "shadow:     files\n"
      "hosts:      files dns\n"
      "networks:   files\n"
      "protocols:  files\n"
      "services:   files\n"
      "ethers:     files\n"
      "rpc:        files\n" },
    { "/etc/ld.so.conf",
      "/lib64\n"
      "/usr/lib/x86_64-linux-gnu\n"   /* Debian multiarch: Vendor/LinuxRuntime stages the .so closure here */
      "/usr/lib\n"
      "/usr/local/lib\n" },
    /* dbus / GLib (g_get_...) read this; a fixed value is fine for a
     * single-image OS. 32 lowercase hex + newline, per machine-id(5). */
    { "/etc/machine-id",
      "0123456789abcdef0123456789abcdef\n" },
    { "/etc/passwd",
      "root:x:0:0:root:/root:/bin/sh\n"
      "nobody:x:65534:65534:nobody:/:/bin/false\n" },
    { "/etc/group",
      "root:x:0:\n"
      "nogroup:x:65534:\n" },
    { "/etc/host.conf",
      "multi on\n" },
    { "/etc/gai.conf",
      "# prefer IPv4 - ImplusOS has no IPv6 stack\n"
      "precedence ::ffff:0:0/96  100\n" },
    { "/etc/shells",
      "/bin/sh\n/bin/dash\n" },
    { "/etc/os-release",
      "NAME=ImplusOS\n"
      "ID=implusos\n"
      "PRETTY_NAME=\"ImplusOS\"\n"
      "VERSION_ID=\"1\"\n" },
    /* Minimal fontconfig config so Chromium/Skia (and any fontconfig client)
     * can resolve the generic families to the one bundled system TTF at
     * /usr/share/fonts (staged by the chrome app / WITH_CHROME image path).
     * The DTD reference is intentionally a URN: fontconfig treats a missing
     * DTD as a warning, not an error. */
    { "/etc/fonts/fonts.conf",
      "<?xml version=\"1.0\"?>\n"
      "<!DOCTYPE fontconfig SYSTEM \"urn:fontconfig:fonts.dtd\">\n"
      "<fontconfig>\n"
      "  <dir>/usr/share/fonts</dir>\n"
      "  <cachedir>/tmp/fontconfig</cachedir>\n"
      "  <match target=\"pattern\"><test name=\"family\"><string>sans-serif</string></test>"
      "<edit name=\"family\" mode=\"append_last\"><string>Noto Sans JP</string></edit></match>\n"
      "  <match target=\"pattern\"><test name=\"family\"><string>serif</string></test>"
      "<edit name=\"family\" mode=\"append_last\"><string>Noto Sans JP</string></edit></match>\n"
      "  <match target=\"pattern\"><test name=\"family\"><string>monospace</string></test>"
      "<edit name=\"family\" mode=\"append_last\"><string>Noto Sans JP</string></edit></match>\n"
      "  <config><rescan><int>30</int></rescan></config>\n"
      "</fontconfig>\n" },
};

static bool etcfs_generate(const char *path, char *buf, uint32_t cap,
                           uint32_t *size_out)
{
    if (strcmp(path, "/etc/hosts") == 0) {
        *size_out = etcfs_build_hosts(buf, cap);
        return true;
    }
    if (strcmp(path, "/etc/resolv.conf") == 0) {
        *size_out = etcfs_build_resolv_conf(buf, cap);
        return true;
    }
    for (size_t i = 0;
         i < sizeof(g_etcfs_binary_files) / sizeof(g_etcfs_binary_files[0]);
         ++i) {
        if (strcmp(path, g_etcfs_binary_files[i].path) == 0) {
            uint32_t n = g_etcfs_binary_files[i].len;
            if (n > cap) {
                n = cap;
            }
            memcpy(buf, g_etcfs_binary_files[i].data, n);
            *size_out = n;
            return true;
        }
    }
    for (size_t i = 0;
         i < sizeof(g_etcfs_static_files) / sizeof(g_etcfs_static_files[0]);
         ++i) {
        if (strcmp(path, g_etcfs_static_files[i].path) == 0) {
            *size_out = (uint32_t)snprintf(buf, cap, "%s",
                                           g_etcfs_static_files[i].content);
            return true;
        }
    }
    return false;
}

static bool etcfs_vfs_find_file(const char *path, vfs_file_t *out_file)
{
    char *buffer = (char *)malloc(ETCFS_BUFFER_CAP);
    if (buffer == NULL) {
        return false;
    }
    uint32_t size = 0;
    if (!etcfs_generate(path, buffer, ETCFS_BUFFER_CAP, &size)) {
        free(buffer);
        return false;
    }
    etcfs_open_t *entry = (etcfs_open_t *)malloc(sizeof(etcfs_open_t));
    if (entry == NULL) {
        free(buffer);
        return false;
    }
    entry->data = (uint8_t *)buffer;
    entry->size = size;
    out_file->internal_id = (uint64_t)(uintptr_t)entry;
    out_file->size = size;
    out_file->driver_data = entry;
    return true;
}

static bool etcfs_vfs_read_at(vfs_file_t *file, uint32_t offset,
                              uint8_t *buffer, uint32_t size)
{
    if (file == NULL || file->driver_data == NULL || buffer == NULL) {
        return false;
    }
    etcfs_open_t *entry = (etcfs_open_t *)file->driver_data;
    if (offset > entry->size || size > entry->size - offset) {
        return false;
    }
    memcpy(buffer, entry->data + offset, size);
    return true;
}

static bool etcfs_vfs_read_file(vfs_file_t *file, uint8_t *buffer)
{
    return etcfs_vfs_read_at(file, 0, buffer, file != NULL ? file->size : 0u);
}

static bool etcfs_vfs_write_file(vfs_file_t *file, const uint8_t *buffer)
{
    (void)file;
    (void)buffer;
    return false;
}

static bool etcfs_vfs_write_at(vfs_file_t *file, uint32_t offset,
                               const uint8_t *buffer, uint32_t size)
{
    (void)file;
    (void)offset;
    (void)buffer;
    (void)size;
    return false; /* Read-only: these files are generated, not stored. */
}

static bool etcfs_vfs_truncate(vfs_file_t *file, uint32_t new_size)
{
    (void)file;
    (void)new_size;
    return false;
}

static uint32_t etcfs_vfs_get_file_size(vfs_file_t *file)
{
    return file != NULL ? file->size : 0u;
}

static bool etcfs_vfs_creat(const char *path)
{
    (void)path;
    return false;
}

static bool etcfs_vfs_mkdir(const char *path)
{
    (void)path;
    return false;
}

typedef struct {
    uint8_t in_use;
    uint32_t cursor;
} etcfs_dir_handle_t;

#define ETCFS_DIR_HANDLE_MAX 4
static etcfs_dir_handle_t g_etcfs_dir_handles[ETCFS_DIR_HANDLE_MAX];
static const char *g_etcfs_entries[] = { "hosts", "resolv.conf" };
#define ETCFS_ENTRY_COUNT (sizeof(g_etcfs_entries) / sizeof(g_etcfs_entries[0]))

static int32_t etcfs_vfs_opendir(const char *path)
{
    if (path == NULL || (strcmp(path, "/etc") != 0 && strcmp(path, "/etc/") != 0)) {
        return -1;
    }
    for (int32_t i = 0; i < ETCFS_DIR_HANDLE_MAX; ++i) {
        if (!g_etcfs_dir_handles[i].in_use) {
            g_etcfs_dir_handles[i].in_use = 1;
            g_etcfs_dir_handles[i].cursor = 0;
            return i;
        }
    }
    return -1;
}

static int32_t etcfs_vfs_readdir(int32_t handle, vfs_dirent_t *out_entry)
{
    if (handle < 0 || handle >= ETCFS_DIR_HANDLE_MAX ||
        !g_etcfs_dir_handles[handle].in_use || out_entry == NULL) {
        return -1;
    }
    uint32_t cursor = g_etcfs_dir_handles[handle].cursor;
    if (cursor >= ETCFS_ENTRY_COUNT) {
        return 0;
    }
    strncpy(out_entry->name, g_etcfs_entries[cursor], sizeof(out_entry->name) - 1);
    out_entry->name[sizeof(out_entry->name) - 1] = '\0';
    out_entry->size = 0;
    out_entry->is_directory = false;
    g_etcfs_dir_handles[handle].cursor = cursor + 1u;
    return 1;
}

static int32_t etcfs_vfs_closedir(int32_t handle)
{
    if (handle < 0 || handle >= ETCFS_DIR_HANDLE_MAX) {
        return -1;
    }
    g_etcfs_dir_handles[handle].in_use = 0;
    return 0;
}

static bool etcfs_vfs_close_file(vfs_file_t *file)
{
    if (file == NULL || file->driver_data == NULL) {
        return true;
    }
    etcfs_open_t *entry = (etcfs_open_t *)file->driver_data;
    free(entry->data);
    free(entry);
    file->driver_data = NULL;
    return true;
}

static bool etcfs_vfs_unlink(const char *path)
{
    (void)path;
    return false;
}

static void etcfs_vfs_list_root(void)
{
}

static void etcfs_vfs_set_case_sensitive(bool enabled)
{
    (void)enabled;
}

static bool etcfs_vfs_get_case_sensitive(void)
{
    return true;
}

static const vfs_driver_t g_etcfs_vfs_driver = {
    .fs_type = "etcfs",
    .media_kind = VFS_MEDIA_KIND_PSEUDO,
    .prefix = NULL,
    .find_file = etcfs_vfs_find_file,
    .read_file = etcfs_vfs_read_file,
    .write_file = etcfs_vfs_write_file,
    .read_at = etcfs_vfs_read_at,
    .write_at = etcfs_vfs_write_at,
    .truncate = etcfs_vfs_truncate,
    .get_file_size = etcfs_vfs_get_file_size,
    .creat = etcfs_vfs_creat,
    .mkdir = etcfs_vfs_mkdir,
    .opendir = etcfs_vfs_opendir,
    .readdir = etcfs_vfs_readdir,
    .closedir = etcfs_vfs_closedir,
    .close_file = etcfs_vfs_close_file,
    .unlink = etcfs_vfs_unlink,
    .list_root = etcfs_vfs_list_root,
    .set_case_sensitive = etcfs_vfs_set_case_sensitive,
    .get_case_sensitive = etcfs_vfs_get_case_sensitive,
};

void etcfs_init(void)
{
    memset(g_etcfs_dir_handles, 0, sizeof(g_etcfs_dir_handles));
}

const vfs_driver_t *etcfs_vfs_get_driver(void)
{
    return &g_etcfs_vfs_driver;
}
