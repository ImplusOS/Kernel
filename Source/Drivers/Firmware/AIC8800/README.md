# AIC8800D80 firmware

Firmware blobs for AICSemi AIC8800D80 USB Wi-Fi 6 adapters (chip revision
U02), read at runtime by `Kernel/Source/Drivers/Wi-Fi/AIC8800/` via the
`driver_binary_t.fs` API from `/Kernel/Driver/Firmware/AIC8800/` on the
booted system.

These files are AICSemi's proprietary property (they come from the vendor's
`aic8800_fdrv` release and are copied here from its `firmware/` set). Do not
redistribute them on their own: treat this directory the same way Linux
treats `linux-firmware`. The driver only reads them at runtime and fails
gracefully ("firmware download: ... failed", "are the firmware blobs
installed in /Kernel/Driver/Firmware/AIC8800/?") when one is absent, so the
rest of the OS builds and boots without them.

The files staged into the boot image automatically by the top-level
`Makefile` (`make image` / `make install_payload`) are:

- `fmacfw_8800d80_u02.bin` — the full-mac firmware, loaded at 0x00120000.
- `fmacfw_8800d80_h_u02.bin` — the variant for "H" chip revisions
  (`(chip_id & 0xC0) == 0xC0`); the driver picks between the two itself.
  This image came from the same vendor release as the other H-capable D80
  blobs (`shenmintao/aic8800d80`, `fw/aic8800D80/`); the non-H files in
  this directory are from an older D80/U02 snapshot. A board that reports
  as H is best served by replacing the whole set with one vendor release.
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
