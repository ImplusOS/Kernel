#include "DriverDB.h"
#include "Core/vfs/VFS.h"

#include <stddef.h>
#include <string.h>

/*
 * DriverDB -- a small, hand-maintained manifest mapping a bus device id to
 * the driver .ELF that handles it, consulted by BusRegistry.c when
 * report_device() finds no match among already-loaded modules'
 * bus_matches[] tables. There is no build-time tooling in this repository
 * that can introspect a compiled descriptor's bus_matches[] to generate
 * this automatically (that information only exists after the module in
 * question has already been loaded and run once) -- it needs a line added
 * by hand whenever a new on-demand driver module is introduced.
 *
 * Format, one entry per line in DRIVER_DB_PATH:
 *   <bus>:vid=<hex>[,pid=<hex>][,class=<hex>]=<module filename>
 * `#` starts a comment (rest of the line ignored); blank lines are
 * skipped. `<bus>` is "usb" or "pci". Example:
 *   usb:vid=A69C,pid=8D80=AX900_Driver.ELF
 *   usb:vid=368B,pid=8D80=AX900_Driver.ELF
 */

#define DRIVER_DB_MAX_FILE_BYTES 4096u

static bool parse_hex16(const char *s, uint32_t len, uint16_t *out)
{
    if (len == 0u || len > 4u) {
        return false;
    }
    uint32_t value = 0u;
    for (uint32_t i = 0u; i < len; ++i) {
        char c = s[i];
        uint32_t digit;
        if (c >= '0' && c <= '9') {
            digit = (uint32_t)(c - '0');
        } else if (c >= 'a' && c <= 'f') {
            digit = (uint32_t)(c - 'a') + 10u;
        } else if (c >= 'A' && c <= 'F') {
            digit = (uint32_t)(c - 'A') + 10u;
        } else {
            return false;
        }
        value = (value << 4) | digit;
    }
    *out = (uint16_t)value;
    return true;
}

typedef struct {
    bool     has_vid;
    bool     has_pid;
    bool     has_class;
    uint16_t vid;
    uint16_t pid;
    uint16_t class_code;
    char     bus[8];
} driver_db_key_t;

static bool parse_line(const char *line, uint32_t line_len,
                       driver_db_key_t *out_key,
                       char *out_name, uint32_t out_name_cap)
{
    uint32_t pos = 0u;
    while (pos < line_len && line[pos] != ':') {
        ++pos;
    }
    if (pos >= line_len || pos == 0u || pos >= sizeof(out_key->bus)) {
        return false;
    }
    memset(out_key, 0, sizeof(*out_key));
    memcpy(out_key->bus, line, pos);
    out_key->bus[pos] = '\0';
    ++pos; /* skip ':' */

    /* the LAST '=' on the line separates the key list from the module
     * filename, since "vid=.." etc. also contain '=' */
    uint32_t name_eq = line_len;
    bool found_eq = false;
    for (uint32_t i = line_len; i > pos;) {
        --i;
        if (line[i] == '=') {
            name_eq = i;
            found_eq = true;
            break;
        }
    }
    if (!found_eq || name_eq <= pos) {
        return false;
    }

    uint32_t name_len = line_len - (name_eq + 1u);
    if (name_len == 0u || name_len >= out_name_cap) {
        return false;
    }
    memcpy(out_name, line + name_eq + 1u, name_len);
    out_name[name_len] = '\0';

    uint32_t seg_start = pos;
    while (seg_start < name_eq) {
        uint32_t seg_end = seg_start;
        while (seg_end < name_eq && line[seg_end] != ',') {
            ++seg_end;
        }

        uint32_t eq = seg_start;
        while (eq < seg_end && line[eq] != '=') {
            ++eq;
        }
        if (eq >= seg_end) {
            return false;
        }

        const char *key = line + seg_start;
        uint32_t key_len = eq - seg_start;
        const char *val = line + eq + 1u;
        uint32_t val_len = seg_end - (eq + 1u);

        uint16_t parsed = 0u;
        if (!parse_hex16(val, val_len, &parsed)) {
            return false;
        }

        if (key_len == 3u && strncmp(key, "vid", 3) == 0) {
            out_key->vid = parsed;
            out_key->has_vid = true;
        } else if (key_len == 3u && strncmp(key, "pid", 3) == 0) {
            out_key->pid = parsed;
            out_key->has_pid = true;
        } else if (key_len == 5u && strncmp(key, "class", 5) == 0) {
            out_key->class_code = parsed;
            out_key->has_class = true;
        } else {
            return false;
        }

        seg_start = seg_end + 1u;
    }

    return out_key->has_vid;
}

static bool key_matches(const driver_db_key_t *key, const bus_device_t *dev)
{
    const char *want_bus = NULL;
    if (dev->bus_type == DEVICE_TYPE_USB) {
        want_bus = "usb";
    } else if (dev->bus_type == DEVICE_TYPE_PCI) {
        want_bus = "pci";
    }
    if (want_bus == NULL || strcmp(key->bus, want_bus) != 0) {
        return false;
    }
    if (key->has_vid && key->vid != dev->vendor_id) {
        return false;
    }
    if (key->has_pid && key->pid != dev->device_id) {
        return false;
    }
    if (key->has_class && key->class_code != (uint16_t)dev->class_code) {
        return false;
    }
    return true;
}

bool driver_db_lookup(const bus_device_t *dev, char *out_name, uint32_t out_name_cap)
{
    if (dev == NULL || out_name == NULL || out_name_cap == 0u) {
        return false;
    }

    vfs_file_t file;
    memset(&file, 0, sizeof(file));
    if (!vfs_find_file(DRIVER_DB_PATH, &file)) {
        return false; /* manifest not present, or VFS not mounted yet -- not an error */
    }

    uint32_t size = vfs_get_file_size(&file);
    if (size == 0u || size > DRIVER_DB_MAX_FILE_BYTES) {
        vfs_close_file(&file);
        return false;
    }

    char buf[DRIVER_DB_MAX_FILE_BYTES];
    bool read_ok = vfs_read_at(&file, 0u, (uint8_t *)buf, size);
    vfs_close_file(&file);
    if (!read_ok) {
        return false;
    }

    bool found = false;
    uint32_t line_start = 0u;
    for (uint32_t i = 0u; i <= size && !found; ++i) {
        if (i == size || buf[i] == '\n') {
            uint32_t line_len = i - line_start;
            if (line_len > 0u && buf[line_start + line_len - 1u] == '\r') {
                --line_len;
            }

            const char *line = buf + line_start;
            if (line_len > 0u && line[0] != '#') {
                driver_db_key_t key;
                char name[DRIVER_DB_MODULE_NAME_MAX];
                if (parse_line(line, line_len, &key, name, sizeof(name)) &&
                    key_matches(&key, dev)) {
                    size_t copy_len = strlen(name);
                    if (copy_len < out_name_cap) {
                        memcpy(out_name, name, copy_len + 1u);
                        found = true;
                    }
                }
            }

            line_start = i + 1u;
        }
    }

    return found;
}
