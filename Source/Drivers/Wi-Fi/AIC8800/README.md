# AIC8800D80 USB Wi-Fi driver (`AIC8800_Driver.ELF`)

Driver module for AICSemi AIC8800D80 USB Wi-Fi 6 adapters (the chip family
in a number of `a69c:`/`368b:` dongles). It is one module that handles the
whole life of the chip: the ROM's firmware-loader stage, downloading the
vendor firmware, the full-MAC station bring-up, scanning, WPA2-PSK
association and the 802.3 data path.

## Files

| File | Role |
|---|---|
| `AIC8800.c` | USB transport, message engine (CFG TX + CFG RX dispatch), RX sub-packet framing, `driver_nic_t` surface, probe/remove |
| `AIC8800_Protocol.h` | Every message id, struct and framing rule, with the vendor source each one came from |
| `AIC8800_Fw.c` | Loader stage: `DBG_MEM_*` reads/writes, patch table parse, firmware image download, `DBG_START_APP_REQ` |
| `AIC8800_Bring.c` | fdrv stage: TX power + RF calibration, MAC read, `MM_RESET`/`MM_VERSION`/`MM_START`/`MM_ADD_IF`, scan, connect |
| `AIC8800_Data.c` | 802.3 ↔ 802.11 conversion (`hostdesc` TX, `hw_rxhdr` RX, A-MSDU, 4-address, EAPOL detection) |
| `AIC8800_Wpa.c` | WPA2-PSK supplicant: EAPOL-Start, M1–M4, PTK/GTK derivation and install |
| `AIC8800_Crypto.c` | Self-contained SHA-1, HMAC-SHA1, PBKDF2, AES-128 (CCM building blocks not needed: CCMP stays in firmware), AES key unwrap |

The module is loaded **on demand**: `Manifest/OnDemand.txt` names it, and
`Manifest/DriverDB.txt` maps both AICSemi vendor ids (`a69c`, `368b`) to it.
`bus_matches[]` keys on vendor id + interface class 0xFF only; `probe()`
classifies the stage by product id (`0x8d80` = ROM loader, anything else =
already-running firmware, canonically `0x8d83` with `0x8d81` on some
revisions).

## Firmware

The firmware blobs are proprietary and live in
`Kernel/Source/Drivers/Firmware/AIC8800/` (they are staged into the boot
image under `/Kernel/Driver/Firmware/AIC8800/` by the top-level
`Makefile`). See that directory's `README.md` for the file list and where
to get the blobs, including the H-revision FMAC image. Without the blobs
the driver stops cleanly (it does not spin) and the Wi-Fi panel shows
"Adapter firmware failed to load".

## Watching it work

Everything the driver does is logged over the serial console with an
`[AIC8800]` prefix; a healthy first-time bring-up looks like this:

```
[AIC8800] loader-stage device at usb addr 2 iface 0 pid 0x8d80 cfg-in 0x00000081 cfg-out 0x00000001 data-in 0x00000000 data-out 0x00000001
[AIC8800] chip id 0x3 (mcu) H-variant
[AIC8800] patch table: adid 0x201940, patch 0x1e0000, ext images 1
[AIC8800] flashing fw_adid_8800d80_u02.bin at 0x201940 (…)
…
[AIC8800] firmware running; waiting for the device to come back with its firmware product id (8d83)
[AIC8800] firmware-stage device at usb addr 3 iface 0 pid 0x8d83 cfg-in 0x00000082 cfg-out 0x00000002 data-in 0x00000081 data-out 0x00000001
[AIC8800] bring-up: … (RF, MAC address, reset, version, start, add-if)
```

The two endpoint pairs matter: the ROM loader answers its `TASK_DBG`
messages on the *first* bulk pair, while the firmware stage uses the
*second* pair for commands and the first for 802.11 data. `cfg-in` /
`cfg-out` name the command pair the driver selected for the current stage;
`data-in` / `data-out` are the data pair (`data-in 0x00000000` means the
interface exposes only one pair, so the command pump also handles data).

then the panel shows `READY`, a scan produces results, and a connect logs
the association and the 4-way handshake. Failure modes are named where they
happen (`firmware download: <step> failed (attempt n/6)`, `bring-up: <step>
failed`, `4-way handshake timed out`, `key install kept failing`).

## Limits / not yet exercised

* The scan sweeps the 2.4 GHz channels 1–14 and the common 5 GHz set (42
  channels in one `SCANU_START_REQ`), active on every channel and with no
  regulatory-domain check. The connect request takes its channel, band and
  BSSID from that scan, so a network has to appear in a scan before the
  panel can join it.
* WPA2-PSK and open networks. No WPA3/SAE, no WPA/WPA1 (EAPOL v1 is
  dropped), no 802.1X/enterprise.
* No power save, no Bluetooth (the BT side of the chip is never started).
* The station MAC is generated locally at attach (the kernel caches it
  before the firmware is up, so the efuse MAC is only read and logged).
* `USB_Main.c` reports no remove for vendor-specific interfaces, so a
  re-plug is recognised only when the re-enumerated device probes with a
  new USB address; the driver then re-initialises the transport and runs
  the appropriate stage's ladder again.
