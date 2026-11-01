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
winsys. The Mesa source/binaries themselves are not vendored by this change;
the OS must provide a compatible Mesa build with the `virtio_gpu` DRI driver
and the corresponding libdrm runtime.

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
4. Check `glxinfo -B`, a GL conformance sample, and `chrome://gpu`. A browser
   may still choose software rendering if its own build or policy disables the
   available GL backend.
5. On Intel hardware, verify the existing BCS self-tests and CPU fallback. Do
   not interpret successful BLT presentation as Intel 3D acceleration.

A successful boot or framebuffer presentation alone is not evidence of 3D
acceleration.
