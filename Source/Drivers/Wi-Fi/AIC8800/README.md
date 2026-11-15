# AIC8800D80 USB Wi-Fi driver (`AIC8800_Driver.ELF`)

Driver module for AICSemi AIC8800D80 USB Wi-Fi 6 adapters (the chip family
in a number of `a69c:`/`368b:` dongles, plus TP-Link/Mercury `2357:014e`/
`014b` and Tenda U11 `2604:001f`/`0020`). It is one module that handles the
whole life of the chip: the ROM's firmware-loader stage, downloading the
vendor firmware, the full-MAC station bring-up, scanning, WPA2-PSK
association (including group/pairwise rekeys) and the 802.3 data path.

Every message id and structure layout was checked by compiling the vendor's
own `lmac_msg.h`/`rwnx_rx.h` on the host, and the protocol, ladder order and
default values follow the vendor's `aic_load_fw`/`aic8800_fdrv` sources
(the `shenmintao/aic8800d80` release the firmware blobs come from).

## Files

| File | Role |
|---|---|
| `AIC8800.c` | USB transport, message engine (CFG TX + CFG RX dispatch), RX sub-packet framing, `driver_nic_t` surface, probe/remove |
| `AIC8800_Protocol.h` | Every message id, struct and framing rule, with the vendor source each one came from |
| `AIC8800_Fw.c` | Loader stage: `DBG_MEM_*` reads/writes, patch table parse, firmware image download, `DBG_START_APP_REQ` |
| `AIC8800_Bring.c` | fdrv stage: warm-boot ROM reboot, `MM_SET_STACK_START`, TX power + RF calibration, MAC read, `MM_RESET`/`MM_VERSION`, `ME_CONFIG`/`ME_CHAN_CONFIG`, `MM_START`/`MM_SET_COEX`/`MM_ADD_IF`, scan, connect, `ME_SET_CONTROL_PORT` |
| `AIC8800_Data.c` | 802.3 ↔ 802.11 conversion (`hostdesc` TX, `hw_rxhdr` flags + security-header stripping on RX, A-MSDU, EAPOL routing) |
| `AIC8800_Wpa.c` | WPA2-PSK supplicant: EAPOL-Start, M1–M4, PTK/GTK derivation and install, group key handshake, PTK rekey, CCMP or TKIP group cipher |
| `AIC8800_Crypto.c` | Self-contained SHA-1, HMAC-SHA1, PBKDF2, AES-128 (CCMP itself stays in firmware), RFC 3394 AES key unwrap |

The module is loaded **on demand**: `Manifest/OnDemand.txt` names it, and
`Manifest/DriverDB.txt` maps both AICSemi vendor ids (`a69c`, `368b`) and the
branded firmware-stage ids to it. `bus_matches[]` keys on vendor id +
interface class 0xFF; `probe()` classifies the stage by vendor/product id
(`a69c:8d80`/`8d40` = ROM loader; the vendor fdrv's D81 set -- `8d81`, `8d41`,
`8d83`..`8d8d`, the TP-Link/Tenda ids -- = running firmware) and rejects every
other id, so other AICSemi chips (8800DC/DW/D80X2, ...) and unrelated
TP-Link/Tenda devices are left alone.

A firmware-stage unit seen before this boot downloaded firmware (a warm
reboot, or another OS ran it) is first sent back to its ROM with
`DBG_START_APP_REQ(REBOOT)`, exactly like the vendor's `aic_load_fw`, so the
session always runs firmware this driver configured.

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
[AIC8800] loader-stage device at usb addr 2 iface 0 id a69c8d80 cfg-in 0x1 cfg-out 0x1 data-in 0x0 data-out 0x1
[AIC8800] chip id 3 (rom) standard
[AIC8800] patch table: adid 201940, patch 1e0000, ext images 1
[AIC8800] flashing fw_adid_8800d80_u02.bin at 201940 (6ac bytes)
[AIC8800] flashing fw_patch_8800d80_u02.bin at 1e0000 (7fbc bytes)
[AIC8800] flashing fw_patch_8800d80_u02_ext0.bin at 20b43c (3f08 bytes)
[AIC8800] flashing fmacfw_8800d80_u02.bin at 120000 (576b8 bytes)
[AIC8800] firmware version 6090101, patch descriptor at 177c00
[AIC8800] firmware started; waiting for the adapter to re-enumerate with its firmware product id (8d81)
[AIC8800] firmware-stage device at usb addr 3 iface 0 id a69c8d81 cfg-in 0x2 cfg-out 0x2 data-in 0x1 data-out 0x1
[AIC8800] stack started (2.4 + 5 GHz)
[AIC8800] efuse mac 88 0 0 12 34 56 (this station uses its locally administered one)
[AIC8800] lmac version …
[AIC8800] station ready on vif 0
[AIC8800] scan complete: 7 networks
[AIC8800] connecting to HomeNet (WPA2-PSK)
[AIC8800] associated; running the 4-way handshake
[AIC8800] WPA2 keys installed, port open; link is up
```

The two endpoint pairs matter: the ROM loader answers its `TASK_DBG`
messages on the *first* bulk pair, while the firmware stage uses the
*second* pair for commands and the first for 802.11 data. `cfg-in` /
`cfg-out` name the command pair the driver selected for the current stage;
`data-in` / `data-out` are the data pair (`data-in 0x00000000` means the
interface exposes only one pair, so the command pump also handles data).

Failure modes are named where they happen (`firmware download: <step>
failed (attempt n/6)`, `bring-up: <step> failed`, `message 3 MIC mismatch:
wrong passphrase?`, `4-way handshake timed out`, `key install kept failing`,
`controlled port would not open`), and the panel shows the failed step.

## Transport notes

* The firmware stage aggregates RX: one bulk IN transfer carries up to ten
  packets. RX transfers are 20480 bytes (the vendor's
  `AICWF_USB_AGGR_MAX_PKT_SIZE`) and a packet cut by a full transfer is
  stitched together from the next one. There is no actual-length report from
  the bus, so the walk relies on per-packet length fields and on the landing
  buffer being zeroed between transfers.
* A DATA transfer's length field counts the whole transfer (header
  included); a CFG command's counts the bytes after the 4-byte header.
* The async IN API creates a missing transfer ring as an *interrupt*
  endpoint, so each pipe gets one sync bulk transfer first.
* Every kernel entry point takes a driver-wide try-lock; the network poll
  (run from the syscall path of any CPU) drives the steady state, the Wi-Fi
  syscalls drive firmware download and bring-up.

## Host test harness

There is no in-tree test (module.mk compiles every `.c` below this
directory into the module), but the driver was exercised on the host by
compiling its unmodified sources against a shim `driver_binary_t` and a
Python fake chip (ROM loader + firmware) with an independent WPA2 AP built
on the `cryptography` package: firmware download, bring-up order and
message layouts, scan parsing, WPA2 4-way handshake, group and PTK rekeys,
TKIP group cipher, CCMP/A-MSDU/aggregated/split RX, TX framing, open
networks and the warm-boot reboot, in both async and sync-only USB modes.

## Limits / not yet exercised

* The channel list (`ME_CHAN_CONFIG` and the scan) is 2.4 GHz channels
  1–14 plus, when `MM_SET_STACK_START` reports a 5 GHz radio, the vendor's
  28 5 GHz channels, all active-scan at a 20 dBm ceiling, with no
  regulatory-domain database. The connect request takes its channel and
  BSSID from the scan (strongest BSS of that name), so a network has to
  appear in a scan before the panel can join it.
* WPA2-PSK (pairwise CCMP, group CCMP or TKIP) and open networks. No
  WPA3/SAE, no WPA1/TKIP-pairwise, no 802.1X/enterprise, no mandatory PMF;
  such networks are refused up front with a log line.
* Capabilities are advertised as 802.11n (1x1, 20/40 MHz, SGI): VHT/HE
  capability blocks are left off, so an 802.11ac/ax AP associates in HT
  mode.
* No host-side A-MPDU reorder buffer and no defragmentation (fragments are
  dropped; APs essentially never fragment). TCP copes with the occasional
  out-of-order frame.
* No power save (`ME_CONFIG.ps_on = 0`), no Bluetooth.
* The station MAC is a locally administered address chosen once per boot
  (the kernel caches it before the firmware is up, so the efuse MAC is only
  read and logged).
* Everything runs from the kernel's network poll (and from the Wi-Fi
  syscalls, which step the driver too): the firmware download and the
  bring-up advance one bounded step per poll, every 10 ms or so, whether or
  not the Wi-Fi panel is open. A wireless module that is not the data-path
  NIC is polled as well. If the serial log stops right after the
  `loader-stage device` line, nothing is polling the driver. If it stops
  after `firmware download: starting`, the ROM loader is not answering its
  first command.
* `USB_Main.c` reports no remove for vendor-specific interfaces, so a
  re-plug is recognised only when the re-enumerated device probes with a
  new USB address; the driver then re-initialises the transport and runs
  the appropriate stage's ladder again.

## QEMU end-to-end test

`Tools/AIC8800Sim/` (top of the repository) contains a virtual AIC8800D80 that
QEMU attaches through `usb-redir`. It plays the ROM loader, the firmware
stage, a WPA2 access point and a small LAN (DHCP, ping, UDP/TCP echo).
`Userland/Application/WifiProbe` drives the Wi-Fi API headlessly. Run:

    make image_livecd AUTOSTART=/Userland/WifiProbe/WifiProbe.ELF
    Tools/AIC8800Sim/run_qemu.sh --warm

The run covers the firmware download, re-enumeration, bring-up, scan,
WPA2 handshake, DHCP, ICMP, UDP, TCP, group rekey and the warm-boot reboot
path, all through the real xHCI driver. See `Tools/AIC8800Sim/README.md`.
