# ImplusOS Kernel

The ImplusOS monolithic kernel: process management, VFS + filesystem
drivers (FAT32/exFAT/ISO9660 + DevFS/TmpFS/ProcFS/EtcFS), the network stack
(Ethernet/ARP/IPv4/UDP/TCP/ICMP/DHCP), IPC (ring-buffer queues + AF_UNIX),
the loadable driver-module manager, the Linux syscall-ABI compat layer, and
per-architecture (`x86_64`/`arm64`) boot, paging, SMP, and interrupt
handling. See `Source/Core/kernel_main.c` for the 19-phase boot sequence.

This repository is a component of **[ImplusOS](https://github.com/ImplusOS)**,
a hobby operating system with a monolithic kernel, loadable driver modules,
a minimal freestanding C library, and a small graphical userland. It is not
meant to be built in isolation -- it is consumed as a checkout alongside
ImplusOS's other component repositories (see `Docs` for the full
architecture and `ImplusOS/Makefile` for how the pieces are wired together).

## Layout

```
Kernel/
├── Source/    All source for this component, structure preserved from ImplusOS
└── README.md  This file
```

## Build

`make` (delegates to `Source/Makefile`) compiles against sibling
checkouts of [I_libc](https://github.com/ImplusOS/I_libc) and
[Library](https://github.com/ImplusOS/Library); driver modules under
`Source/Drivers/` build independently via `Source/Drivers/module.mk`.
Pass `ARCH=arm64` for AArch64. Normally invoked from the top-level
ImplusOS Makefile's `kernel` and `driver_build` targets.

## License

MIT, matching the parent [ImplusOS](https://github.com/ImplusOS/ImplusOS) project.
