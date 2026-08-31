#include "kernel/boot_info.h"
#include "kernel/platform.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "smp/SMP_Main.h"
#ifdef PLATFORM_X86_64
#include "cpu/IDT_Main.h"
#include "cpu/GDT_Main.h"
#include "virt/VMX.h"
#endif
#include "Platform/io/IO_Main.h"
#include "Drivers/Module/DriverModule.h"
#include "Drivers/Module/BlockManager.h"
#include "Drivers/Module/AudioManager.h"
#include "Drivers/Module/InputManager.h"
#include "Drivers/Module/DriverManager.h"
#include "Drivers/Module/DriverSelect.h"
#include "Drivers/Module/PlatformBuiltinDrivers.h"
#include "Core/elf/ELF_Loader.h"
#include "Core/syscall/Syscall_Main.h"
#include "Core/syscall/Syscall_File.h"
#include "Core/process/ProcessManager.h"
#include "kernel/config.h"
#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "Core/hardening/StackProtector.h"
#include "Boot/LoadBar.h"
#include "Platform/acpi/ACPI.h"
#include "Platform/interrupt/Interrupts.h"
#include "Core/vfs/VFS.h"
#include "Core/vfs/VFS_Pseudo.h"
#include "Drivers/Module/FS_VFS_Bridge.h"
#include <string.h>
#include "IPC/IPC_Main.h"
#include "Debug/printf/printf.h"
#include "Debug/panic/Panic.h"
#include "Network/network_main.h"
#include "Debug/serial/Serial.h"
#include <stdio.h>
#include "interfaces/arch_ops.h"
#include "interfaces/timer_hal.h"

static BOOT_INFO g_boot_info_copy;

#if OS_CONFIG_BOOT_FADE
static uint32_t *g_fb_snapshot = NULL;
static uint32_t g_fb_snapshot_pixels = 0;
#endif
static uint64_t g_boot_framebuffer_phys_base = 0;
static uint64_t g_boot_framebuffer_phys_size = 0;

#define KERNEL_BOOT_PROFILE_MAX 64u
static boot_profile_entry_t g_boot_profile[KERNEL_BOOT_PROFILE_MAX];
static uint32_t g_boot_profile_count = 0;

__attribute__((aligned(16))) static uint8_t kernel_stack[0x20000];

static uint64_t user_entry = 0;
extern const arch_ops_t *arch_ops_get(void);

static const char *kernel_boot_drive_type_name(uint32_t type)
{
    switch (type) {
        case BOOT_DRIVE_TYPE_IDE:
            return "IDE";
        case BOOT_DRIVE_TYPE_USB:
            return "USB";
        case BOOT_DRIVE_TYPE_AHCI:
            return "AHCI";
        case BOOT_DRIVE_TYPE_NVME:
            return "NVMe";
        case BOOT_DRIVE_TYPE_VIRTIO:
            return "VirtIO";
        case BOOT_DRIVE_TYPE_UNKNOWN:
        default:
            return "unknown";
    }
}

const BOOT_INFO *kernel_get_boot_info(void)
{
    return &g_boot_info_copy;
}

uint64_t kernel_get_boot_fb_phys_base(void)
{
    return g_boot_framebuffer_phys_base;
}

uint64_t kernel_get_boot_fb_phys_size(void)
{
    return g_boot_framebuffer_phys_size;
}

static uint64_t boot_profile_begin(void)
{
    return timer_monotonic_ns();
}

static void boot_profile_end(const char *name, uint64_t start_ns)
{
    if (name == NULL || g_boot_profile_count >= KERNEL_BOOT_PROFILE_MAX) {
        return;
    }
    uint64_t end_ns = timer_monotonic_ns();
    boot_profile_entry_t *entry = &g_boot_profile[g_boot_profile_count++];
    memset(entry, 0, sizeof(*entry));
    strncpy(entry->name, name, sizeof(entry->name) - 1u);
    entry->start_ns = start_ns;
    entry->duration_ns = end_ns >= start_ns ? end_ns - start_ns : 0u;
}

int32_t kernel_boot_profile_count(void)
{
    return (int32_t)g_boot_profile_count;
}

int32_t kernel_boot_profile_get(int32_t index, boot_profile_entry_t *entry_out)
{
    if (entry_out == NULL || index < 0 ||
        (uint32_t)index >= g_boot_profile_count) {
        return -1;
    }
    *entry_out = g_boot_profile[index];
    return 0;
}


static inline void kernel_arch_halt(void)
{
    hal_cpu_halt();
}

static inline void kernel_arch_switch_stack(uintptr_t sp)
{
    hal_arch_switch_stack(sp);
}

#define BOOT_FRAMEBUFFER_MAP_GRANULE (2ULL * 1024ULL * 1024ULL)

static uint64_t kernel_map_boot_framebuffer_range(uint64_t phys_base,
                                                  uint64_t size)
{
    if (phys_base == 0 || size == 0) {
        return phys_base;
    }

    uint64_t base = phys_base & ~(BOOT_FRAMEBUFFER_MAP_GRANULE - 1ULL);
    uint64_t offset = phys_base - base;
    uint64_t span = offset + size;
    if (span < offset) {
        return 0;
    }

    uint64_t chunks = (span + BOOT_FRAMEBUFFER_MAP_GRANULE - 1ULL) /
                      BOOT_FRAMEBUFFER_MAP_GRANULE;
    uint64_t first_virt = 0;

    for (uint64_t i = 0; i < chunks; ++i) {
        uint64_t chunk_phys = base + (i * BOOT_FRAMEBUFFER_MAP_GRANULE);
        void *mapped = map_mmio_virt(chunk_phys);
        if (mapped == NULL) {
            return 0;
        }

        uint64_t chunk_virt = (uint64_t)(uintptr_t)mapped;
        if (i == 0) {
            first_virt = chunk_virt;
        } else if (chunk_virt !=
                   first_virt + (i * BOOT_FRAMEBUFFER_MAP_GRANULE)) {
            return 0;
        }
    }

    return first_virt + offset;
}

static void kernel_rebind_boot_framebuffer_after_paging(BOOT_INFO *boot_info)
{
    if (boot_info == NULL ||
        g_boot_framebuffer_phys_base == 0 ||
        g_boot_framebuffer_phys_size == 0) {
        return;
    }

    uint64_t virt_base = kernel_map_boot_framebuffer_range(
        g_boot_framebuffer_phys_base,
        g_boot_framebuffer_phys_size
    );
    if (virt_base == 0) {
        return;
    }

    boot_info->FrameBufferBase = virt_base;
    g_boot_info_copy.FrameBufferBase = virt_base;

    debugger_init(boot_info);
    kernel_panic_init(boot_info);
    load_bar_init(boot_info);
    serial_set_screen_mirror(debug_putchar);
}

bool all_fs_initialize(void)
{
    if (!vfs_init()) {
        return false;
    }

    /* Real (media-backed) filesystems: every driver that registered itself
     * as DEVICE_TYPE_FILESYSTEM is discovered, mounted and ranked by media
     * kind inside FS_VFS_Bridge -- the boot path names no filesystem and
     * knows no filesystem module. */
    if (!fs_bridge_mount_all()) {
        return false;
    }

    /* In-kernel pseudo filesystems (devfs / tmpfs / procfs / etcfs) and the
     * DRM/KMS shim. The list and its mount points live in the VFS layer
     * (Kernel/Core/vfs/VFS_Pseudo.c), not here. */
    vfs_mount_pseudo_filesystems();

    return true;
}

#if OS_CONFIG_BOOT_FADE
static inline uint32_t alpha_blend(uint32_t dst, uint32_t src)
{
    uint8_t a  = (uint8_t)((src >> 24) & 0xFFu);

    if (a == 255) {
        return src;
    }

    if (a == 0) {
        return dst;
    }

    uint8_t sr = (uint8_t)((src >> 16) & 0xFFu);
    uint8_t sg = (uint8_t)((src >> 8)  & 0xFFu);
    uint8_t sb = (uint8_t)((src >> 0)  & 0xFFu);

    uint8_t dr = (uint8_t)((dst >> 16) & 0xFFu);
    uint8_t dg = (uint8_t)((dst >> 8)  & 0xFFu);
    uint8_t db = (uint8_t)((dst >> 0)  & 0xFFu);

    uint8_t r = (uint8_t)((sr * a + dr * (255 - a)) / 255);
    uint8_t g = (uint8_t)((sg * a + dg * (255 - a)) / 255);
    uint8_t b = (uint8_t)((sb * a + db * (255 - a)) / 255);

    return
        (0xFF << 24) |
        (r << 16) |
        (g << 8) |
        b;
}

static bool fb_snapshot_create(BOOT_INFO *bi)
{
    if (!bi || !bi->FrameBufferBase || bi->FrameBufferSize == 0) {
        return false;
    }

    g_fb_snapshot_pixels = (uint32_t)(bi->FrameBufferSize / 4);

    g_fb_snapshot = malloc(bi->FrameBufferSize);

    if (!g_fb_snapshot) {
        return false;
    }

    memcpy(
        g_fb_snapshot,
        (void *)bi->FrameBufferBase,
        bi->FrameBufferSize
    );

    return true;
}

static void fb_clear(BOOT_INFO *bi, uint32_t color)
{
    if (!g_fb_snapshot) {
        return;
    }

    uint32_t *fb = (uint32_t *)bi->FrameBufferBase;
    uint32_t pixels = (uint32_t)(bi->FrameBufferSize / 4);

    for (uint32_t i = 0; i < pixels; i++) {
        fb[i] = alpha_blend(g_fb_snapshot[i], color);
    }
}

static void kernel_boot_screen_color(uint32_t color)
{
    if (g_boot_info_copy.FrameBufferBase == 0 || g_boot_info_copy.FrameBufferSize < 4) {
        return;
    }
    fb_clear(&g_boot_info_copy, color);
}
#endif

static void load_spinner_timer(uint64_t tick)
{
    load_bar_tick(tick);
    load_bar_update();
}

static void kernel_main_after_stack_switch(BOOT_INFO *boot_info)
{
    uint64_t phase_ns = 0;

    /* Best-effort re-seed of the stack-protector canary (see
     * Kernel/Core/hardening/StackProtector.c) as early as possible with
     * whatever entropy is on hand -- the boot_info pointer's address is
     * not itself secret, but its low bits vary with the bootloader's
     * memory-map/ASLR-ish placement decisions in a way an attacker
     * targeting this specific build/boot can't fully predict in advance.
     * Reseeded again once the timer subsystem is up (see the "timer" boot
     * phase below) for a second, independent entropy source. */
    stack_protector_reseed((uint64_t)(uintptr_t)boot_info);

    load_bar_init(boot_info);
    timer_set_callback(load_spinner_timer);

    phase_ns = boot_profile_begin();
    {
        /* Every arch_ops_t implementation sets init_cpu_tables (see
         * Kernel/Arch/{x86_64,arm64}/arch_ops.c) -- the old x86_64-only
         * "else { init_gdt(); init_idt(); }" fallback here was dead code
         * once that became true and has been removed (Docs/Others/
         * TODO_OS_Refactor.md phase P4, 8.1 item 2). */
        const arch_ops_t *ops = arch_ops_get();
        if (ops && ops->init_cpu_tables) {
            ops->init_cpu_tables();
        }
    }
    boot_profile_end("cpu_tables", phase_ns);

    phase_ns = boot_profile_begin();
    init_physical_memory(
        (void *)boot_info->MemoryMap,
        boot_info->MemoryMapSize,
        boot_info->MemoryMapDescriptorSize,
        0
    );
    physical_memory_reserve_region(
        boot_info->FrameBufferBase,
        boot_info->FrameBufferSize
    );
    /* Reserve the low 1 MiB. The PMM builds its free list straight from the
     * UEFI memory map, which reports this region as ordinary conventional
     * RAM, and the page-allocator hint starts at frame 0 -- so the very
     * first allocations (the kernel heap in the "heap" phase below) land
     * here by default. The x86_64 SMP bring-up trampoline and its shared
     * hand-off struct are copied to fixed physical addresses in this range
     * (SMP_TRAMPOLINE_PHYS 0x8000 / SMP_SHARED_PHYS 0x9000, see
     * Kernel/Arch/x86_64/smp/SMP_Main.c). Without this reservation the heap
     * and a still-booting AP alias the same frames: an AP that is slow to
     * leave the real-mode trampoline executes whatever the heap has since
     * written there, which shows up as a rare triple-fault reboot late in
     * boot (kernel init done, screen already cleared, userland not yet up).
     * The legacy IVT/BDA/EBDA live down here too, so keep the whole 1 MiB. */
    physical_memory_reserve_region(0x0ULL, 0x100000ULL);
    boot_profile_end("pmm", phase_ns);

    serial_set_screen_mirror(debug_putchar);
    phase_ns = boot_profile_begin();
    init_paging();
    kernel_rebind_boot_framebuffer_after_paging(boot_info);
    boot_profile_end("paging", phase_ns);

    phase_ns = boot_profile_begin();
    memory_init();
#if KERNEL_COW_FORK
    /* Arm the physical-page refcount table now that the heap/PMM are up, so
     * copy-on-write fork can share frames between address spaces. */
    memory_init_page_refcounts();
#endif
    boot_profile_end("heap", phase_ns);

    phase_ns = boot_profile_begin();
    acpi_init(boot_info);
    platform_interrupts_configure(acpi_get_info());
    boot_profile_end("acpi_interrupts", phase_ns);

    /* arch_ops_t.get_timer_hal replaces the old "#ifdef PLATFORM_X86_64
     * timer = &lapic_timer_hal; #elif PLATFORM_ARM64 timer =
     * &generic_timer_hal" selection (Docs/Others/TODO_OS_Refactor.md
     * phase P4, 8.1 item 1). */
    const arch_ops_t *timer_ops = arch_ops_get();
    const timer_hal_t *timer = (timer_ops && timer_ops->get_timer_hal)
                                    ? timer_ops->get_timer_hal()
                                    : NULL;
    phase_ns = boot_profile_begin();
    timer_init(timer);
    boot_profile_end("timer", phase_ns);
    stack_protector_reseed(timer_ticks());

    phase_ns = boot_profile_begin();
    syscall_init();
    boot_profile_end("syscall", phase_ns);

    phase_ns = boot_profile_begin();
    smp_init();
    boot_profile_end("smp", phase_ns);

    {
        /* arm64's arch_ops_t.virtualization_init is an explicit "not
         * supported here" stub (returns -1, see Kernel/Arch/arm64/
         * arch_ops.c) rather than NULL, so the NULL check below is
         * sufficient on both arches and the old "#ifdef PLATFORM_X86_64"
         * around this whole block was dead weight (Docs/Others/
         * TODO_OS_Refactor.md phase P4, 8.1 item 3). */
        const arch_ops_t *ops = arch_ops_get();
        if (ops && ops->virtualization_init) {
            (void)ops->virtualization_init();
        }
    }

    {
        const arch_ops_t *ops = arch_ops_get();
        if (ops && ops->enable_interrupts) {
            ops->enable_interrupts();
        }
    }
    /* timer_switch_lapic() is itself a no-op when the active timer_hal_t
     * has no switch_to_local (see Kernel/Core/timer/Timer.c), so the old
     * "#if defined(PLATFORM_X86_64) || defined(PLATFORM_ARM64)" guard here
     * was a tautology (every supported ARCH build defines one or the
     * other) -- removed per 8.1 item 4. */
    timer_switch_lapic();
    timer_start_clock();
    phase_ns = boot_profile_begin();
    driver_module_manager_init(boot_info);
    boot_profile_end("driver_module_load", phase_ns);

    /* DeviceRegistry now exists (driver_module_manager_init() ->
     * driver_manager_init() -> device_registry_init() just ran) -- give
     * ACPI/Timer/LAPIC/IOAPIC (already initialized earlier in this very
     * function, at the same points as always) their DeviceRegistry
     * entries. See Kernel/Drivers/Module/PlatformBuiltinDrivers.c. */
    platform_builtin_drivers_register();

    phase_ns = boot_profile_begin();
    uint64_t driver_init_irq_flags = irq_save_disable();
    driver_module_init_critical();
    irq_restore(driver_init_irq_flags);
    boot_profile_end("driver_module_critical", phase_ns);

    driver_boot_framebuffer_t boot_fb = {
        .addr = (void *)(uintptr_t)g_boot_framebuffer_phys_base,
        .size_bytes = (uint32_t)g_boot_framebuffer_phys_size,
        .width = boot_info->HorizontalResolution,
        .height = boot_info->VerticalResolution,
        .pixels_per_scan_line = boot_info->PixelsPerScanLine,
        .bytes_per_pixel = 4
    };
    driver_select_set_boot_framebuffer(&boot_fb);
    phase_ns = boot_profile_begin();
    input_manager_init();
    boot_profile_end("input_init", phase_ns);

    block_manager_set_boot_identity(boot_info);
    phase_ns = boot_profile_begin();
    bool disk_ok = disk_io_init(boot_info->PartitionStartLBA, boot_info->BootDriveType);
    boot_profile_end("disk_io_init", phase_ns);
    if (!disk_ok) {
        kernel_panic("Disk Protocol initialization failed", "kernel_main");
    }

    bool fs_ready = false;
    phase_ns = boot_profile_begin();
    if (all_fs_initialize()) {
        fs_ready = true;
    }
    boot_profile_end("fs_init", phase_ns);

    if (!fs_ready) {
        kernel_panic("Filesystem initialization failed and diskless boot not enabled", "kernel_main");
    }

    serial_enable_file_logging("/Kernel.log");
    phase_ns = boot_profile_begin();
    bool display_ready = driver_manager_display_init();
    bool debug_display_ready = debugger_display_init();
    serial_set_screen_mirror(debug_putchar);
    boot_profile_end("display_init", phase_ns);

    phase_ns = boot_profile_begin();
    process_manager_init();
    boot_profile_end("process_manager", phase_ns);

    phase_ns = boot_profile_begin();
    ipc_init();
    syscall_file_init();
    boot_profile_end("kernel_services", phase_ns);

#if OS_CONFIG_BOOT_FADE
    bool fb_snapshot_ok = fb_snapshot_create(boot_info);
#else
    bool fb_snapshot_ok = false;
#endif
    (void)fb_snapshot_ok;
    load_bar_finish();
#if OS_CONFIG_BOOT_FADE
    for (int i = 0; i <= 10; i++) {
        timer_apic_sleep_ms(1);
        uint8_t alpha = (uint8_t)(i * 255 / 10);
        uint32_t color =
            (alpha << 24) |
            0x000000;
        kernel_boot_screen_color(color);
    }
#endif

    if (fs_ready) {
        phase_ns = boot_profile_begin();
        static const uint8_t userland_elf_magic[4] = {0x7Fu, 'E', 'L', 'F'};
        if (!vfs_set_default_fs_for_file("/Userland/Userland.ELF",
                                         64u,
                                         userland_elf_magic,
                                         sizeof(userland_elf_magic))) {
            vfs_init();
            if (all_fs_initialize() &&
                vfs_set_default_fs_for_file("/Userland/Userland.ELF",
                                           64u,
                                           userland_elf_magic,
                                           sizeof(userland_elf_magic))) {
                serial_write_string("[kernel:init] retry FS init: SUCCESS\n");
            } else {
                kernel_panic("Userland ELF not found or unreadable.", "kernel_main");
            }
        }

        if (process_register_boot_process("/Userland/Userland.ELF", &user_entry) < 0) {
            const char *elf_err = elf_loader_last_error();
            char panic_msg[128];
            if (elf_err) {
                snprintf(panic_msg, sizeof(panic_msg),
                         "Failed to start Userland (ELF: %s)", elf_err);
            } else {
                snprintf(panic_msg, sizeof(panic_msg),
                         "Failed to start Userland.");
            }
            kernel_panic(panic_msg, "kernel_main");
        }
        boot_profile_end("userland_elf", phase_ns);
    }

    phase_ns = boot_profile_begin();
    uint64_t deferred_irq_flags = irq_save_disable();
    driver_module_init_deferred();
    irq_restore(deferred_irq_flags);
    boot_profile_end("driver_module_deferred", phase_ns);

    phase_ns = boot_profile_begin();
    audio_manager_init();
    /* network_stack_init() -> network_builtin_drivers_init_all() both
     * registers the DEVICE_TYPE_NET_PROTOCOL layers and brings them up in
     * dependency order; no separate registration call is needed here. */
    network_stack_init();
    boot_profile_end("audio_network_init", phase_ns);

    timer_start_services();
    serial_set_screen_mirror(NULL);

    uint64_t user_rsp = process_get_current_user_rsp();
    uint64_t saved_rsp = process_get_current_saved_rsp();
    uint64_t user_cr3 = process_get_current_cr3();

    if (user_rsp == 0 || saved_rsp == 0 || user_cr3 == 0 || user_entry == 0) {
        const arch_ops_t *ops = arch_ops_get();
        if (ops && ops->disable_interrupts) {
            ops->disable_interrupts();
        }
        kernel_panic("Failed get RSP and CR3", "kernel_main");
    }

    {
        const arch_ops_t *ops = arch_ops_get();
        if (ops) {
            ops->enter_user_mode(saved_rsp, user_rsp, user_cr3);
        }
    }

    for (;;) { kernel_arch_halt(); }
}

__attribute__((noreturn))
void kernel_main(BOOT_INFO *boot_info)
{
    const arch_ops_t *ops = arch_ops_get();
    if (ops && ops->disable_interrupts) {
        ops->disable_interrupts();
    }
    if (ops && ops->early_init) {
        ops->early_init();
    }

    debugger_init(boot_info);

    serial_init();

    if (boot_info != NULL) {
        memcpy(&g_boot_info_copy, boot_info, sizeof(BOOT_INFO));
        boot_info = &g_boot_info_copy;
        g_boot_framebuffer_phys_base = boot_info->FrameBufferBase;
        g_boot_framebuffer_phys_size = boot_info->FrameBufferSize;
    }

    kernel_panic_init(boot_info);

    uintptr_t sp = (uintptr_t)(kernel_stack + sizeof(kernel_stack));
    sp &= ~0xFULL;

    #ifdef PLATFORM_X86_64
    {
        hal_arch_switch_stack_and_jump(sp, kernel_main_after_stack_switch, boot_info);
    }
    #elif defined(PLATFORM_ARM64)
        {
            hal_arch_switch_stack_and_jump(sp, kernel_main_after_stack_switch, boot_info);
        }
    #endif

    __builtin_unreachable();
}
