#pragma once

#ifndef GDT_KERNEL_CODE
#define GDT_KERNEL_CODE      0x08
#endif
#ifndef GDT_KERNEL_DATA
#define GDT_KERNEL_DATA      0x10
#endif
#ifndef GDT_USER_COMPAT_CODE
#define GDT_USER_COMPAT_CODE 0x18
#endif
#ifndef GDT_USER_DATA
#define GDT_USER_DATA        0x20
#endif
#ifndef GDT_USER_CODE
#define GDT_USER_CODE        0x28
#endif
#ifndef GDT_TSS
#define GDT_TSS              0x30
#endif

/* Copy-on-write fork (TODO_Chromium_LinuxABI.md bucket B, sections 11.3/12.3).
 *
 * process_fork() shares the parent's pages into the child read-only and
 * copies them lazily on the first write (paging_cow_clone_user_range /
 * paging_handle_cow_fault), backed by physical-page refcounts in
 * Memory_Main.c. The eager alternative duplicates the whole address space --
 * Chromium's is around 400 MB, a measured eager fork took ~5 s, and its
 * zygote forks once per renderer.
 *
 * What it took to make it correct:
 *   - clone both user ranges the eager path copies (the child used to get no
 *     stack and no mmap arena);
 *   - keep a page copy-on-write through a second fork, and keep "read-only"
 *     and "copy-on-write" distinct in mprotect();
 *   - unshare before any kernel write into user memory (CR0.WP is clear, so
 *     the kernel would otherwise write through into the shared frame);
 *   - stop the parent's other threads for the duration of the fork
 *     (process_fork_hold_acquire), since the clone rewrites the parent's page
 *     tables in place -- the equivalent of Linux holding mmap_lock.
 * -DKERNEL_COW_FORK=0 restores the eager copy. */
#ifndef KERNEL_COW_FORK
#define KERNEL_COW_FORK 1
#endif

/*
 * TLB shootdown is synchronous: smp_tlb_shootdown_cr3() does not return until
 * every CPU that could hold a stale translation has invalidated it. There is
 * no switch for it because there is no safe setting other than on -- a
 * fire-and-forget shootdown lets the sender free a page, narrow its protection
 * or hand its address range back to the allocator while another CPU is still
 * writing through the old translation, which is what made headless Chromium
 * die of memory corruption within a minute under `-smp >1` while surviving
 * indefinitely under `-smp 1`.
 *
 * What makes the wait affordable is that the request names an address space:
 * without PCID an x86 CR3 load flushes the whole TLB, so only the CPUs
 * currently running that CR3 have to answer. See
 * Arch/x86_64/smp/SMP_Main.c.
 */

#ifndef OS_CONFIG_PROCESS_MAX_COUNT
#ifdef PROCESS_MAX_COUNT_CONFIG
#define OS_CONFIG_PROCESS_MAX_COUNT PROCESS_MAX_COUNT_CONFIG
#else
/* Tasks, not processes: every thread takes a slot. 256 ran out while
 * browsing -- each Chromium renderer has ~25 threads, and a few tabs plus the
 * GPU/network/audio/storage processes crossed it; pthread_create() then
 * failed and the new process died on CHECK(io_thread_->StartWithOptions()). */
#define OS_CONFIG_PROCESS_MAX_COUNT 512
#endif
#endif

#ifndef OS_CONFIG_FILE_MAX_FD
#ifdef FILE_MAX_FD_CONFIG
#define OS_CONFIG_FILE_MAX_FD FILE_MAX_FD_CONFIG
#else
/* NOTE: this is a *global*, system-wide fd table (Syscall_File.c), shared
 * by every process, not a per-process limit. The socket ranges (inet
 * 512..767, AF_UNIX 768..1023) sit inside its numeric span and are skipped,
 * so files are 0..511 plus 1024..OS_CONFIG_FILE_MAX_FD-1; the upper part is
 * handed only to Linux-ABI processes, which see per-process numbers (native
 * programs and the POSIX layer index 1024-entry tables by the raw number).
 *
 * The AF_UNIX range used to live inside this table (192..255, under the X
 * server's 256-client limit), so the table skipped it. Linux programs now see
 * per-process descriptor numbers (Compat/Linux/Linux_FdTable.c) rather than
 * these global ones, and AF_UNIX moved to 768..1023 (UnixSocket.h); the whole
 * 0..511 range is files again. 192 slots were not enough for Chromium alone
 * -- ~130 open files plus ~40 shared-memory regions -- and once they ran out
 * it could not create the buffer for its next frame and terminated itself
 * ("Creating shared memory in /dev/shm/... failed: Too many open files").
 * 512 ran out the same way once Chromium was multi-process for real: a
 * YouTube tab plus the GPU, network and audio processes held ~490 -- every
 * child keeps its own descriptors for the resource packs and ICU data, on
 * top of its shared-memory regions -- and new renderers failed to start. */
#define OS_CONFIG_FILE_MAX_FD 2048
#endif
#endif

#ifndef OS_CONFIG_FILE_MAX_DIR_HANDLE
#ifdef FILE_MAX_DIR_HANDLE_CONFIG
#define OS_CONFIG_FILE_MAX_DIR_HANDLE FILE_MAX_DIR_HANDLE_CONFIG
#else
/* Kept <= OS_CONFIG_FILE_MAX_FD. */
#define OS_CONFIG_FILE_MAX_DIR_HANDLE 192
#endif
#endif

#ifndef PROCESS_MAX_COUNT_CONFIG
#define PROCESS_MAX_COUNT_CONFIG OS_CONFIG_PROCESS_MAX_COUNT
#endif

#ifndef FILE_MAX_FD_CONFIG
#define FILE_MAX_FD_CONFIG OS_CONFIG_FILE_MAX_FD
#endif

#ifndef FILE_MAX_DIR_HANDLE_CONFIG
#define FILE_MAX_DIR_HANDLE_CONFIG OS_CONFIG_FILE_MAX_DIR_HANDLE
#endif

#define OS_CONFIG_PROCESS_MAX_COUNT_MIN   1
#define OS_CONFIG_PROCESS_MAX_COUNT_MAX   1024
#define OS_CONFIG_FILE_MAX_FD_MIN         4
/* Must stay below Syscall_Epoll.c's epoll numbering (0x4000). Numbers from
 * 1024 up are given to Linux-ABI processes only (see above), so the POSIX
 * layer's 1024-entry tables never see them. */
#define OS_CONFIG_FILE_MAX_FD_MAX         4096
#define OS_CONFIG_FILE_MAX_DIR_HANDLE_MIN 4
#define OS_CONFIG_FILE_MAX_DIR_HANDLE_MAX 256

#if (PROCESS_MAX_COUNT_CONFIG < OS_CONFIG_PROCESS_MAX_COUNT_MIN) || \
    (PROCESS_MAX_COUNT_CONFIG > OS_CONFIG_PROCESS_MAX_COUNT_MAX)
#error "PROCESS_MAX_COUNT_CONFIG is out of supported range"
#endif

#if (FILE_MAX_FD_CONFIG < OS_CONFIG_FILE_MAX_FD_MIN) || \
    (FILE_MAX_FD_CONFIG > OS_CONFIG_FILE_MAX_FD_MAX)
#error "FILE_MAX_FD_CONFIG is out of supported range"
#endif

#if (FILE_MAX_DIR_HANDLE_CONFIG < OS_CONFIG_FILE_MAX_DIR_HANDLE_MIN) || \
    (FILE_MAX_DIR_HANDLE_CONFIG > OS_CONFIG_FILE_MAX_DIR_HANDLE_MAX)
#error "FILE_MAX_DIR_HANDLE_CONFIG is out of supported range"
#endif

#if FILE_MAX_DIR_HANDLE_CONFIG > FILE_MAX_FD_CONFIG
#error "FILE_MAX_DIR_HANDLE_CONFIG must be <= FILE_MAX_FD_CONFIG"
#endif

#ifndef OS_CONFIG_SMP_MAX_CPUS
#define OS_CONFIG_SMP_MAX_CPUS 16
#endif

/* Kernel/Drivers/Module/DriverModule.c's g_modules[] table size. Decoupled
 * from BOOT_INFO's MAX_LOADED_FILES (kernel/boot_info.h, 16): that constant
 * bounds how many .ELF files the bootloader can preload before kernel_main
 * even runs, while this one also has to make room for driver modules
 * loaded post-boot (driver_module_manager_load_from_vfs()), so it must be
 * >= MAX_LOADED_FILES. */
#ifndef OS_CONFIG_DRIVER_MODULE_MAX_COUNT
#define OS_CONFIG_DRIVER_MODULE_MAX_COUNT 64
#endif

#ifndef OS_CONFIG_SMP_ENABLED
#define OS_CONFIG_SMP_ENABLED 1
#endif

#ifndef OS_CONFIG_LOG_FILE_MAX_BYTES
#define OS_CONFIG_LOG_FILE_MAX_BYTES (512 * 1024)
#endif

/* Boot hand-off animation. With this set, the last thing the kernel does to
 * the boot screen is freeze it, scale it to 150% and dissolve it to black on
 * an ease-in-out curve (Kernel/Source/Boot/BootAnim.c); the init process then
 * brings its own first screen back in from 50% (Userland/Source/Userland.c).
 * Set to 0 for a boot that hands over on the bare boot screen -- the two
 * halves are independent, so the kernel half can be dropped on its own.
 * Costs OS_CONFIG_BOOT_FADE_MS of wall time plus one full-screen snapshot on
 * the kernel heap, both released before userland starts. */
#ifndef OS_CONFIG_BOOT_FADE
#define OS_CONFIG_BOOT_FADE 1
#endif

/* Wall-clock length of the kernel half of that transition, in milliseconds.
 * The loop is time-driven, so this is what the animation actually takes: a
 * panel too large to blit at 60 Hz loses frames, not time. */
#ifndef OS_CONFIG_BOOT_FADE_MS
#define OS_CONFIG_BOOT_FADE_MS 420u
#endif

/* Per-event tracing of the foreign (Linux-ABI) runtime: every shared-object
 * mmap, every AF_UNIX packet, every signal/exec/exit, and the periodic
 * syscall heartbeat. Invaluable while bringing a binary up, but it is written
 * to COM1 one character at a time from inside the syscall path, so an X
 * server plus a GL client -- which between them mmap ~400 segments and
 * exchange thousands of small packets before the first frame -- spend a large
 * part of their startup inside serial_write_string() -- at 115200 baud every
 * traced byte is ~87us of busy-wait, which no amount of KVM or faster silicon
 * makes cheaper. Off by default; set to 1 for a bring-up boot.
 * See Docs/Others/TODO_Doom_Xorg_MethodA.md. */
#ifndef OS_CONFIG_FOREIGN_TRACE
#define OS_CONFIG_FOREIGN_TRACE 0
#endif

/* Always-on, one-line-per-event log of foreign (Linux-ABI) program launches:
 * exec, the interpreter that will run it, and the exit status. This is what
 * OS_CONFIG_FOREIGN_TRACE's per-syscall firehose is not -- a handful of lines
 * per program rather than thousands, cheap enough to leave on in a release
 * boot, and enough to see *that* an app started and how long it took. Read it
 * from inside the OS at /dev/kmsg (Core/vfs/DevFS.c). */
#ifndef OS_CONFIG_FOREIGN_LAUNCH_LOG
#define OS_CONFIG_FOREIGN_LAUNCH_LOG 1
#endif

/* Early wakeups for poll(2)/select(2)/epoll_wait(2) and the pipe/pty waits
 * (Core/syscall/Poll_Wait.c). With this off, those waits serve out their full
 * sleep slice exactly as they did before the mechanism existed -- slower to
 * react, but with no wakeup bookkeeping at all. It is a switch rather than a
 * constant because the bookkeeping is subtle (see
 * Docs/Others/TODO_Performance_LinuxApps.md section 9): getting it wrong
 * costs a CPU spin, and being able to rule it out in one build is worth more
 * than the latency it buys. */
#ifndef OS_CONFIG_POLL_WAIT_WAKEUPS
#define OS_CONFIG_POLL_WAIT_WAKEUPS 1
#endif

/* Read cache in front of the block layer (Platform/io/Block_Cache.c). Set to
 * 0 to send every disk_read() straight at the medium again, which is useful
 * for measuring what the cache is worth on a given machine and for ruling it
 * out when chasing a filesystem bug. */
#ifndef OS_CONFIG_BLOCK_CACHE
#define OS_CONFIG_BLOCK_CACHE 1
#endif

#ifndef OS_CONFIG_DEBUG_PAGE_FAULT_DUMP
#define OS_CONFIG_DEBUG_PAGE_FAULT_DUMP 0
#endif

#ifndef OS_CONFIG_SIGNAL_HANDLER_MAX_PER_PROCESS
#define OS_CONFIG_SIGNAL_HANDLER_MAX_PER_PROCESS 32
#endif

#ifndef OS_CONFIG_PENDING_SIGNAL_MAX_PER_PROCESS
#define OS_CONFIG_PENDING_SIGNAL_MAX_PER_PROCESS 32
#endif

#ifndef OS_CONFIG_NET_IPV4_ADDR
#define OS_CONFIG_NET_IPV4_ADDR 0x0A00020FULL
#endif

#ifndef OS_CONFIG_NET_IPV4_MASK
#define OS_CONFIG_NET_IPV4_MASK 0xFFFFFF00ULL
#endif

#ifndef OS_CONFIG_NET_IPV4_GATEWAY
#define OS_CONFIG_NET_IPV4_GATEWAY 0x0A000202ULL
#endif
