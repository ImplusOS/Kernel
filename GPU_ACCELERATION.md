# GPU acceleration status

This document distinguishes display-copy acceleration from 3D rendering. A
working framebuffer blit does not by itself mean that OpenGL, Vulkan, WebGL, or
a browser GPU process can use the device.

## Current backends

| Backend | Display acceleration | 3D rendering |
| --- | --- | --- |
| VirtIO-GPU | 2D scanout and framebuffer transfer | virgl control protocol plus a Linux VirtIO-GPU DRM render-node path for Mesa's virgl Gallium winsys |
| Intel Gen9 / Gen9.5 | BCS blitter acceleration for validated framebuffer copies, with CPU fallback | Not implemented; this is not an i915-compatible 3D driver |

## VirtIO-GPU / Mesa virgl connection

The kernel exposes two distinct roles:

- `/dev/dri/card0` remains the KMS/display node and can continue to be driven by
  the currently selected display driver.
- `/dev/dri/renderD128` can discover a ready VirtIO-GPU 3D backend independently
  of the scanout driver. Its sysfs PCI identity and DRM driver name identify the
  VirtIO GPU so libdrm/Mesa can select the `virtio_gpu` winsys.

DevFS publishes the character node with Linux DRM major 226 and minor 128,
and its `/dev` directory enumeration exposes `dri` as a directory rather than
a synthetic entry containing a slash. SysFS publishes
`/sys/dev/char/226:128/uevent`, the PCI identity/driver attributes under
`device/`, and `device/drm/renderD128`; these are the paths libdrm uses to
validate the node and map it back to the VirtIO PCI device.

The Xorg log shows that the DRI3 extension is initialized. The
modesetting DDX's supported `PageFlip` option enables DRI3 page flipping and
defaults to on; the generated Xorg configuration keeps it enabled. The old
`Option "DRI3" "true"` entry was not a valid modesetting-driver option and
has been removed. The Xorg `[DRI2] ... kms_swrast` AIGLX provider line alone
does not prove which renderer a DRI3 client receives. For DRI3 clients, Xorg
uses libdrm to map the KMS fd to its associated render node. The published
`/sys/dev/char/226:0/device/drm` and
`/sys/dev/char/226:128/device/drm` entries expose that association; Mesa
then queries the render node's `virtio_gpu` driver name and selects
`virtio_gpu_dri.so`.

The render node implements the Linux VirtIO-GPU DRM ioctl subset used by the
Mesa virgl winsys:

- `GETPARAM`, including 3D support, capset-query fix, context-init capability,
  and supported capset ID mask;
- `GET_CAPS` and `CONTEXT_INIT`;
- 3D `RESOURCE_CREATE`, `RESOURCE_INFO`, `MAP`, `EXECBUFFER`,
  `TRANSFER_TO_HOST`, `TRANSFER_FROM_HOST`, and `WAIT`;
- GEM handle lifetime, mmap, and PRIME import/export with the virgl resource
  retained until the final GEM/dma-buf reference is released.

The VirtIO driver uses the Linux `virtio_gpu.h` wire command identifiers and
feature bits. 3D request layouts are guarded by compile-time size assertions.
Command submissions, transfers, and resource lifetime operations are
serialized. Render resources use contiguous guest physical backing; arbitrary
user pages and scatter/gather backing are not accepted.

### Deliberate limits

This is a Mesa virgl **legacy fence path**, not a full Linux DRM implementation.
The render node reports DRM version 0.0 so Mesa does not enable the sync_file
fence-FD path (Mesa gates that path on DRM version 0.1). Submissions and
transfers complete synchronously on the VirtIO control queue, and `WAIT`
therefore treats a live BO as completed.

The following features are not implemented and are not advertised as available:

- `RESOURCE_CREATE_BLOB`, host-visible/resource-blob memory, and cross-device
  resource sharing;
- syncobj, sync_file fence-FD import/export, multiple GPU command rings, and
  asynchronous fence events;
- modifiers and advanced dma-buf synchronization semantics.

Mesa Vulkan/Venus and gfxstream paths that require blob resources or multiple
rings are outside this implementation. The target here is Mesa's virgl OpenGL
winsys. The OS runtime stages Debian's `libgl1-mesa-dri` DRI modules,
including `virtio_gpu_dri.so` when supplied by the pinned package, as well as
the `libdrm.so.2` runtime dependency. The stage script reports a warning if
the virgl DRI module is absent; software `swrast_dri.so` remains available as
the fallback. Mesa's normal device discovery uses the render-node driver name
and PCI identity. For diagnosis only, a process can be launched with
`LIBGL_DEBUG=verbose MESA_LOADER_DRIVER_OVERRIDE=virtio_gpu`; do not force
this override on machines where the VirtIO 3D backend is unavailable.

The Mesa source/binaries themselves are not vendored by this change. The
pinned Debian Mesa package provides the userspace virgl implementation; the
kernel implements the matching legacy VirtIO-GPU DRM ABI subset.

## Intel Gen9 / Gen9.5

The Intel display module remains deliberately conservative. It only enables
its blitter after bounded ring, fence, GGTT, and scratch-memory self-tests. A
failure leaves the CPU framebuffer-copy path active. Its current acceleration
is a 2D presentation optimization; it does not implement the Intel 3D render
command streamer, i915 GEM execution ABI, or an OpenGL driver.

Full Intel 3D support requires a separate implementation of GPU virtual memory
and object pinning, render-engine initialization and reset, validated command
submission, fences and recovery, cache coherency, an i915-compatible DRM ABI,
and a matching Mesa i915 driver stack. The existing BLT path must remain
independent so that adding 3D support cannot regress display output.

## Validation

The source-level ioctl layouts and command IDs were compared with the Linux
VirtIO-GPU UAPI and the Mesa virgl winsys interface. Compile-time assertions
guard the wire structures and ioctl payload layouts.

A full kernel/module build, QEMU virgl boot, and real Mesa GL/WebGL test have not
yet been run in this environment. Before enabling this path on a target system:

1. Build the x86_64 kernel and VirtIO display module with the project toolchain.
2. Boot QEMU with a virgl-capable VirtIO GPU (for example `virtio-vga-gl`) and
   a virgl-capable host renderer.
3. Provide a compatible Mesa/libdrm runtime, open `/dev/dri/renderD128`, and
   verify capset discovery, context creation, BO map/resource creation,
   execbuffer, transfer, WAIT, and PRIME lifecycle.
4. Check `ls -l /dev/dri`, `stat /dev/dri/renderD128`, and
   `cat /sys/dev/char/226:128/uevent`. On boot the kernel should log
   `[drm] published ... renderD128 (226:128)`; when Mesa opens the render node
   it should log `[drm-render] VERSION driver=virtio_gpu version=0.0` and
   `[drm-render] 3D_FEATURES=1`.
5. Check `LIBGL_DEBUG=verbose MESA_LOADER_DRIVER_OVERRIDE=virtio_gpu glxinfo -B`,
   a GL conformance sample, and `chrome://gpu`. The renderer must be virgl,
   not `llvmpipe`. A browser may still choose software rendering if its own
   build or policy disables the available GL backend.
6. On Intel hardware, verify the existing BCS self-tests and CPU fallback. Do
   not interpret successful BLT presentation as Intel 3D acceleration.

### Desktop framebuffer and shell diagnostics

The framebuffer syscall now keeps one mapping per process address space and
returns the same user pointer on repeated calls. This prevents every redraw
from allocating another 1.2 MiB framebuffer VA range and another set of page
tables. The cache is keyed by PID and CR3, validates the cached first-page
physical mapping before reuse, and recycles entries belonging to exited
processes. Per-frame display and VirtIO serial diagnostics are rate-limited.

The uploaded boot trace also contained repeated `/bin/busybox` exec failures
with `elf_err=paging map failed` and exit status 127. That failure is in the
ELF PT_LOAD page-mapping path, not proof that `ls` or `rm` applets are
missing. On x86_64 the kernel now reserves physical `0x00400000-0x00800000`
before initializing the heap. This is the low identity-map window used by the
unchanged static BusyBox `ET_EXEC` image, whose entry segments start at
`0x00400000`. ELF pages mapped into that virtual window now use ordinary
private PMM frames, while a narrowly scoped page-table exception permits the
process CR3 to shadow only those reserved identity entries. All other low
identity mappings retain the strict same-physical-frame rule. This makes the
fixed-address mapping process-private without allowing the kernel heap or
normal PMM allocations to occupy the physical pages whose virtual addresses
are shadowed.

The kernel also logs the failing ELF segment range, requested mapping flags,
remaining free pages, and whether the lower-level failure came from
physical-page allocation, identity-page reservation, or a leaf PTE map. Use
those messages on the next boot to distinguish address-space leakage from
physical-page exhaustion or a page-table mapping error. Fork/COW, execve,
unmap, and teardown for this fixed-address window still require runtime
validation; do not treat the source change alone as a successful BusyBox boot.

A successful boot or framebuffer presentation alone is not evidence of 3D
acceleration.
