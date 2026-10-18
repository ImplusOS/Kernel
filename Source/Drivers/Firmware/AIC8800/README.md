# AIC8800D80 firmware

Firmware blobs for AICSemi AIC8800D80 USB Wi-Fi 6 adapters (chip revision
U02), read at runtime by `Kernel/Drivers/Wi-Fi/AIC8800/` via the
`driver_binary_t.fs` API from `/Kernel/Driver/Firmware/AIC8800/` on the
booted system.

These files are AICSemi's proprietary property and are **not** distributed
with ImplusOS (the same relationship Linux has with the separate
`linux-firmware` package) — this directory ships empty, and the driver
fails gracefully ("firmware files missing") when a blob is absent. To bring
up an AIC8800D80 on real hardware, drop the following files here before
running `make image`/`make install_payload` (they are staged into the boot
image automatically, see the top-level `Makefile`):

- `fmacfw_8800d80_u02.bin` — the full-mac firmware, loaded at 0x00120000.
  `fmacfw_8800d80_h_u02.bin` is the variant for "H" chip revisions
  (`(chip_id & 0xC0) == 0xC0`); the driver picks between the two itself.
- `fw_patch_8800d80_u02.bin` — the ROM patch.
- `fw_patch_8800d80_u02_ext0.bin` — the external patch image the patch
  table's INF block points at (`ext0` is the id the D80 table lists; the
  driver appends each id it finds to `fw_patch_8800d80_u02_ext`).
- `fw_patch_table_8800d80_u02.bin` — the container that says where the
  patch and its extension go.
- `fw_adid_8800d80_u02.bin` — the ADID calibration blob.

The easiest source is AICSemi's own `aic8800_fdrv` release (the files sit
next to the driver source in `aic8800_fdrv/` and in the vendor's
`firmware/` directory for the D80/U02 combination). All of them must come
from a build for the **D80** chip and the **U02** board revision; the file
names above are exactly what `AIC8800.h` (`AIC_FW_FILE_*`) and
`AIC8800_Fw.c` ask the filesystem for.

Note: this is `Kernel/Driver/` (singular), the on-disk boot-image layout
also used for `Kernel/Driver/*.ELF` — distinct from `Kernel/Drivers/`
(plural), this repository's driver *source* tree.
