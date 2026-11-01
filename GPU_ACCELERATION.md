# GPU acceleration status

This document distinguishes display-copy acceleration from 3D rendering. A
working framebuffer blit does not by itself mean that OpenGL, Vulkan, WebGL, or
a browser GPU process can use the device.

## Current backends

| Backend | Existing display acceleration | 3D command execution |
| --- | --- | --- |
| VirtIO-GPU | 2D scanout and framebuffer transfer | virgl control-queue protocol support is being brought into line with the Linux UAPI; not yet wired to DRM render-node ioctls or Mesa |
| Intel Gen9 / Gen9.5 | BCS blitter acceleration for validated framebuffer copies, with CPU fallback | Not implemented; this driver is not an i915-compatible 3D driver |

## VirtIO-GPU / virgl protocol foundation

The VirtIO driver uses the modern `virtio_gpu.h` wire command identifiers and
feature bits. The 3D request layouts use the control header's `ctx_id` field,
the protocol's context-create name length and context-init fields, the
resource-create field order, and the transfer box/stride fields.

The module API now exposes the following building blocks:

- virgl context create/destroy and command submission;
- 3D resource create/unref;
- single contiguous DMA backing attachment;
- context resource attach/detach;
- host-to-guest and guest-to-host resource transfers.

The control queue is serialized while these public operations execute.
Backing passed to the attachment helper must be a contiguous,
DMA-addressable physical range. The helper does not pin arbitrary user pages,
build scatter/gather lists, or import dma-buf objects.

## Not yet implemented

The following are required before Linux applications can use virgl through
`/dev/dri/renderD128`:

1. VirtIO-specific DRM UAPI support (`GETPARAM`, `CONTEXT_INIT`,
   `RESOURCE_CREATE`, `EXECBUFFER`, `WAIT`, and the associated resource
   lifecycle operations).
2. A per-file DRM context and resource/handle table, including safe mapping of
   guest physical backing and validation of user-supplied command buffers.
3. dma-buf / PRIME interoperability between render resources and the KMS
   scanout path, with lifetime and synchronization guarantees.
4. Mesa virgl driver integration and end-to-end tests using a Linux ELF such as
   Chromium (`chrome://gpu`) and a GL/WebGL conformance sample.

Until these pieces exist, Mesa must not be told that the render node provides
working 3D acceleration. The software renderer remains the correct fallback.

## Intel Gen9 / Gen9.5

The Intel display module is deliberately conservative. It only enables its
blitter after bounded ring, fence, GGTT, and scratch-memory self-tests. A
failure leaves the CPU framebuffer-copy path active. Its current acceleration
is a 2D presentation optimization; it does not implement the Intel 3D render
command streamer, i915 GEM execution ABI, or an OpenGL driver.

Full Intel 3D support requires a separate, substantially larger implementation:
GPU virtual memory and object pinning, render-engine initialization and
reset, command submission validation, fences and preemption/recovery, cache
coherency, and a userspace driver stack. The existing BLT path must remain
independent so that adding 3D support cannot regress display output.

## Validation

Build the x86_64 kernel and display modules with the project toolchain, then
boot both configurations:

- QEMU: `-device virtio-vga-gl` (or the QEMU VirtIO-GPU configuration with
  virgl enabled) and a matching virgl-capable host.
- Physical hardware: an allowlisted Intel Gen9 / Gen9.5 device.

For VirtIO, verify device feature negotiation, context create/destroy, resource
create/attach/transfer/unref, and command completion before enabling the DRM
render-node capability. For Intel, verify the existing BLT self-tests and
compare the rendered output with the CPU fallback. A successful boot or
framebuffer presentation alone is not evidence of 3D acceleration.
