#pragma once

#include "DriverBinary.h"

#include <stdbool.h>
#include <stdint.h>

#define DRIVER_DB_PATH "/Kernel/Driver/DriverDB.txt"
#define DRIVER_DB_MODULE_NAME_MAX 64u

/*
 * Looks up `dev` in the hand-maintained manifest at DRIVER_DB_PATH and, on
 * a match, copies the driver .ELF filename it names into `out_name`
 * (NUL-terminated, must fit in `out_name_cap`). Returns false if the
 * manifest can't be read (not present, or VFS not mounted yet -- neither
 * is an error) or no entry matches. See DriverDB.c for the file format.
 */
bool driver_db_lookup(const bus_device_t *dev, char *out_name, uint32_t out_name_cap);
