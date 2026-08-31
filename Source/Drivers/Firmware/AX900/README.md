# AX900 firmware

Firmware blobs for the UGREEN AX900 USB Wi-Fi 6 adapter (AICSemi AIC8800D80,
chip revision U02), read at runtime by
`Kernel/Drivers/Wi-Fi/AX900/AX900.c` via the `driver_binary_t.fs` API
from `/Kernel/Driver/Firmware/AX900/` on the booted system.

These files are AICSemi's proprietary property and are **not** distributed
with ImplusOS (same relationship Linux has with the separate
`linux-firmware` package) — this directory ships empty. To bring up a real
AX900 on real hardware, drop the following files here before running
`make image`/`make install_payload` (they are staged into the boot image
automatically, see the top-level `Makefile`):

- `fmacfw_8800d80_u02.bin`
- `lmacfw_rf_8800d80_u02.bin`
- `fw_patch_8800d80_u02.bin`

`fw_patch_table_8800d80_u02.bin` and `fw_adid_8800d80_u02.bin` are not
loaded by `AX900.c` yet — see the comment on `ax900_download_firmware()` in
`AX900.c` for why.

Note: this is `Kernel/Driver/` (singular), the on-disk boot-image layout
also used for `Kernel/Driver/*.ELF` — distinct from `Kernel/Drivers/`
(plural), this repository's driver *source* tree.
