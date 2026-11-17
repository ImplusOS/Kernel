#include "GIC.h"
#include "interfaces/interrupt_ops.h"
#include "interfaces/mmio.h"
#include "kernel/config.h"
#include "Platform/acpi/ACPI.h"
#include "smp/SMP_Main.h"

#define GICD_CTLR       0x0000
#define GICD_TYPER      0x0004
#define GICD_IGROUPR    0x0080
#define GICD_ISENABLER  0x0100
#define GICD_ICENABLER  0x0180
#define GICD_ICPENDR    0x0280
#define GICD_ICACTIVER  0x0380
#define GICD_IPRIORITYR 0x0400
#define GICD_ITARGETSR  0x0800
#define GICD_ICFGR      0x0C00
#define GICD_IROUTER    0x6000

#define GICC_CTLR       0x0000
#define GICC_PMR        0x0004
#define GICC_BPR        0x0008
#define GICC_IAR        0x000C
#define GICC_EOIR       0x0010

#define GICD_SGIR       0x0F00

#define GICR_TYPER      0x0008
#define GICR_TYPER_VLPIS (1ULL << 1)
#define GICR_TYPER_LAST  (1ULL << 4)
#define GICR_FRAME_SIZE  0x20000ULL
#define GICR_WAKER      0x0014
#define GICR_SGI_BASE   0x10000
#define GICR_IGROUPR0   (GICR_SGI_BASE + 0x0080)
#define GICR_ISENABLER0 (GICR_SGI_BASE + 0x0100)
#define GICR_ICENABLER0 (GICR_SGI_BASE + 0x0180)
#define GICR_ICPENDR0   (GICR_SGI_BASE + 0x0280)
#define GICR_ICACTIVER0 (GICR_SGI_BASE + 0x0380)
#define GICR_IPRIORITYR (GICR_SGI_BASE + 0x0400)
#define GICR_ICFGR1     (GICR_SGI_BASE + 0x0C04)

#define GICD_CTLR_ENABLE_GRP0   (1u << 0)
#define GICD_CTLR_ENABLE_GRP1NS (1u << 1)
#define GICD_CTLR_ARE_NS        (1u << 4)
#define GICD_CTLR_RWP           (1u << 31)

#define GICC_CTLR_ENABLE        (1u << 0)

#define GICR_WAKER_PROCESSOR_SLEEP (1u << 1)
#define GICR_WAKER_CHILDREN_ASLEEP (1u << 2)

#define GIC_PRIORITY_DEFAULT 0xA0A0A0A0u
#define GIC_SPURIOUS_IRQ    0x3FFu
#define GIC_MAX_IRQ         1020u

#define GIC_VERSION_2 2u
#define GIC_VERSION_3 3u

#define ICC_CTLR_EOIMODE (1u << 1)

static volatile uint32_t *g_gicd;
static volatile uint32_t *g_gicc;
/* GICv3 has one redistributor per CPU; g_gicr is the first frame of the
 * redistributor region and g_gicr_cpu[] the frame each CPU found for itself
 * (gicv3_find_redistributor). Everything per-CPU -- SGIs and PPIs such as
 * the generic timer -- goes through the calling CPU's frame. */
static volatile uint32_t *g_gicr;
static volatile uint32_t *g_gicr_cpu[OS_CONFIG_SMP_MAX_CPUS];
static uint8_t g_gic_version;
static int g_gic_ready;

static inline uint32_t gic_read32(volatile uint32_t *base, uint64_t off)
{
    return mmio_read32((volatile uint8_t *)base + off);
}

static inline void gic_write32(volatile uint32_t *base, uint64_t off, uint32_t value)
{
    mmio_write32((volatile uint8_t *)base + off, value);
}

static inline void gic_write64(volatile uint32_t *base, uint64_t off, uint64_t value)
{
    mmio_write64((volatile uint8_t *)base + off, value);
}

static inline uint64_t read_mpidr_el1(void)
{
    uint64_t value;
    __asm__ volatile("mrs %0, mpidr_el1" : "=r"(value));
    return value;
}

static inline uint64_t read_current_el(void)
{
    uint64_t value;
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(value));
    return (value >> 2) & 0x3u;
}

static inline uint64_t gicv3_read_iar1(void)
{
    uint64_t value;
    __asm__ volatile("mrs %0, S3_0_C12_C12_0" : "=r"(value));
    return value;
}

static inline void gicv3_write_eoir1(uint64_t value)
{
    __asm__ volatile("msr S3_0_C12_C12_1, %0" :: "r"(value) : "memory");
    __asm__ volatile("isb" ::: "memory");
}

static inline uint64_t gicv3_read_sre_el1(void)
{
    uint64_t value;
    __asm__ volatile("mrs %0, S3_0_C12_C12_5" : "=r"(value));
    return value;
}

static inline void gicv3_write_sre_el1(uint64_t value)
{
    __asm__ volatile("msr S3_0_C12_C12_5, %0" :: "r"(value) : "memory");
    __asm__ volatile("isb" ::: "memory");
}

static inline uint64_t gicv3_read_sre_el2(void)
{
    uint64_t value;
    __asm__ volatile("mrs %0, S3_4_C12_C9_5" : "=r"(value));
    return value;
}

static inline void gicv3_write_sre_el2(uint64_t value)
{
    __asm__ volatile("msr S3_4_C12_C9_5, %0" :: "r"(value) : "memory");
    __asm__ volatile("isb" ::: "memory");
}

static inline void gicv3_write_pmr_el1(uint64_t value)
{
    __asm__ volatile("msr S3_0_C4_C6_0, %0" :: "r"(value) : "memory");
}

static inline void gicv3_write_bpr1_el1(uint64_t value)
{
    __asm__ volatile("msr S3_0_C12_C12_3, %0" :: "r"(value) : "memory");
}

static inline uint64_t gicv3_read_ctlr_el1(void)
{
    uint64_t value;
    __asm__ volatile("mrs %0, S3_0_C12_C12_4" : "=r"(value));
    return value;
}

static inline void gicv3_write_ctlr_el1(uint64_t value)
{
    __asm__ volatile("msr S3_0_C12_C12_4, %0" :: "r"(value) : "memory");
    __asm__ volatile("isb" ::: "memory");
}

static inline void gicv3_write_igrpen1_el1(uint64_t value)
{
    __asm__ volatile("msr S3_0_C12_C12_7, %0" :: "r"(value) : "memory");
    __asm__ volatile("isb" ::: "memory");
}

static inline void gic_barrier(void)
{
    __asm__ volatile("dsb sy; isb" ::: "memory");
}

/* This CPU's redistributor frame (GICv3), or NULL before it has one. */
static volatile uint32_t *gicr_this_cpu(void)
{
    uint32_t cpu = smp_get_current_cpu_id();
    if (cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS && g_gicr_cpu[cpu] != 0) {
        return g_gicr_cpu[cpu];
    }
    return (cpu == 0u) ? g_gicr : 0;
}

static void gicd_wait_rwp(void)
{
    for (uint32_t i = 0; i < 100000u; ++i) {
        if ((gic_read32(g_gicd, GICD_CTLR) & GICD_CTLR_RWP) == 0u) {
            break;
        }
    }
}

static uint32_t gicd_irq_count(void)
{
    uint32_t typer = gic_read32(g_gicd, GICD_TYPER);
    uint32_t count = ((typer & 0x1Fu) + 1u) * 32u;
    if (count > GIC_MAX_IRQ) {
        count = GIC_MAX_IRQ;
    }
    return count;
}

static uint64_t gicv3_current_affinity(void)
{
    uint64_t mpidr = read_mpidr_el1();
    return (mpidr & 0xFFu) |
           (mpidr & 0xFF00u) |
           (mpidr & 0xFF0000u) |
           ((mpidr & 0xFF00000000ULL) >> 8);
}

static void gicd_set_spi_target_v2(uint32_t irq)
{
    uint64_t off = GICD_ITARGETSR + (uint64_t)(irq & ~3u);
    uint32_t shift = (irq & 3u) * 8u;
    uint32_t value = gic_read32(g_gicd, off);
    value &= ~(0xFFu << shift);
    value |=  (0x01u << shift);
    gic_write32(g_gicd, off, value);
}

static void gicd_route_spi_v3(uint32_t irq)
{
    if (irq >= 32u) {
        gic_write64(g_gicd, GICD_IROUTER + ((uint64_t)irq * 8u),
                     gicv3_current_affinity());
    }
}

static void gicv3_configure_ppi_level(volatile uint32_t *gicr, uint32_t irq)
{
    if (gicr == 0 || irq < 16u || irq >= 32u) {
        return;
    }

    uint32_t shift = (irq - 16u) * 2u;
    uint32_t value = gic_read32(gicr, GICR_ICFGR1);
    value &= ~(0x3u << shift);
    gic_write32(gicr, GICR_ICFGR1, value);
}

static void gicd_configure_ppi_level(uint32_t irq)
{
    if (irq < 16u || irq >= 32u) {
        return;
    }

    uint64_t off = GICD_ICFGR + ((uint64_t)(irq / 16u) * 4u);
    uint32_t shift = (irq % 16u) * 2u;
    uint32_t value = gic_read32(g_gicd, off);
    value &= ~(0x3u << shift);
    gic_write32(g_gicd, off, value);
}

static void gic_clear_pending_irq(uint32_t irq)
{
    volatile uint32_t *gicr = gicr_this_cpu();
    if (g_gic_version >= GIC_VERSION_3 && irq < 32u && gicr != 0) {
        gic_write32(gicr, GICR_ICPENDR0, 1u << irq);
        return;
    }

    gic_write32(g_gicd, GICD_ICPENDR + ((irq / 32u) * 4u),
                 1u << (irq % 32u));
}

static void gic_clear_active_irq(uint32_t irq)
{
    volatile uint32_t *gicr = gicr_this_cpu();
    if (g_gic_version >= GIC_VERSION_3 && irq < 32u && gicr != 0) {
        gic_write32(gicr, GICR_ICACTIVER0, 1u << irq);
        return;
    }

    gic_write32(g_gicd, GICD_ICACTIVER + ((irq / 32u) * 4u),
                 1u << (irq % 32u));
}

static void gicv2_cpu_init(void)
{
    if (g_gicc) {
        gic_write32(g_gicc, GICC_PMR, 0xFFu);
        gic_write32(g_gicc, GICC_BPR, 0u);
        gic_write32(g_gicc, GICC_CTLR, GICC_CTLR_ENABLE);
    }
}

/* Walk the redistributor region for the frame whose GICR_TYPER affinity
 * is this CPU's. Frames are 128 KiB (256 KiB with VLPI support); the last
 * one has GICR_TYPER.Last set. */
static volatile uint32_t *gicv3_find_redistributor(void)
{
    if (g_gicr == 0) {
        return 0;
    }
    uint64_t mpidr = read_mpidr_el1();
    uint64_t aff = (mpidr & 0xFFFFFFULL) | ((mpidr >> 8) & 0xFF000000ULL);
    uintptr_t frame = (uintptr_t)g_gicr;
    for (uint32_t i = 0; i < 256u; ++i) {
        uint64_t typer = mmio_read64((volatile uint8_t *)frame + GICR_TYPER);
        if ((typer >> 32) == aff) {
            return (volatile uint32_t *)frame;
        }
        if ((typer & GICR_TYPER_LAST) != 0u) {
            break;
        }
        frame += (uintptr_t)(GICR_FRAME_SIZE *
                             (((typer & GICR_TYPER_VLPIS) != 0u) ? 2u : 1u));
    }
    return 0;
}

static void gicv3_redistributor_init(volatile uint32_t *gicr)
{
    if (gicr == 0) {
        return;
    }

    uint32_t waker = gic_read32(gicr, GICR_WAKER);
    waker &= ~GICR_WAKER_PROCESSOR_SLEEP;
    gic_write32(gicr, GICR_WAKER, waker);

    for (uint32_t i = 0; i < 100000u; ++i) {
        if ((gic_read32(gicr, GICR_WAKER) &
             GICR_WAKER_CHILDREN_ASLEEP) == 0u) {
            break;
        }
    }

    gic_write32(gicr, GICR_IGROUPR0, 0xFFFFFFFFu);
    gic_write32(gicr, GICR_ICENABLER0, 0xFFFFFFFFu);
    gic_write32(gicr, GICR_ICPENDR0, 0xFFFFFFFFu);
    gic_write32(gicr, GICR_ICACTIVER0, 0xFFFFFFFFu);
    gicv3_configure_ppi_level(gicr, 27u);
    gicv3_configure_ppi_level(gicr, 30u);
    for (uint32_t i = 0; i < 8u; ++i) {
        gic_write32(gicr, GICR_IPRIORITYR + i * 4u, GIC_PRIORITY_DEFAULT);
    }
    /* The SGIs this kernel sends between CPUs (arm64_gic_send_sgi). */
    gic_write32(gicr, GICR_ISENABLER0, 0x0000FFFFu);
    gic_barrier();
}

static void gicv3_cpu_init(void)
{
    if (read_current_el() >= 2u) {
        uint64_t sre_el2 = gicv3_read_sre_el2();
        sre_el2 |= 0xFu;
        gicv3_write_sre_el2(sre_el2);
    }

    uint64_t sre = gicv3_read_sre_el1();
    sre |= 0x1u;
    gicv3_write_sre_el1(sre);

    uint64_t ctlr = gicv3_read_ctlr_el1();
    ctlr &= ~(uint64_t)ICC_CTLR_EOIMODE;
    gicv3_write_ctlr_el1(ctlr);

    gicv3_write_pmr_el1(0xFFu);
    gicv3_write_bpr1_el1(0u);
    gicv3_write_igrpen1_el1(1u);
}

int arm64_gic_init(uint64_t gicd_base, uint64_t gicr_base, uint64_t gicc_base)
{
    if (gicd_base == 0) return -1;
    g_gicd = (volatile uint32_t *)(uintptr_t)gicd_base;
    g_gicc = (volatile uint32_t *)(uintptr_t)gicc_base;
    g_gicr = (volatile uint32_t *)(uintptr_t)gicr_base;
    g_gic_version = (gicr_base != 0) ? GIC_VERSION_3 : GIC_VERSION_2;

    gic_write32(g_gicd, GICD_CTLR, 0);
    gicd_wait_rwp();

    uint32_t irq_count = gicd_irq_count();
    uint32_t group_regs = (irq_count + 31u) / 32u;
    for (uint32_t i = 0; i < group_regs; ++i) {
        uint32_t group = (g_gic_version >= GIC_VERSION_3) ?
                         0xFFFFFFFFu : 0x00000000u;
        gic_write32(g_gicd, GICD_IGROUPR + i * 4u, group);
        gic_write32(g_gicd, GICD_ICENABLER + i * 4u, 0xFFFFFFFFu);
        gic_write32(g_gicd, GICD_ICPENDR + i * 4u, 0xFFFFFFFFu);
        gic_write32(g_gicd, GICD_ICACTIVER + i * 4u, 0xFFFFFFFFu);
    }
    for (uint32_t i = 0; i < (irq_count / 4u); ++i) {
        gic_write32(g_gicd, GICD_IPRIORITYR + i * 4u, GIC_PRIORITY_DEFAULT);
    }

    if (g_gic_version >= GIC_VERSION_3) {
        volatile uint32_t *own = gicv3_find_redistributor();
        if (own == 0) {
            own = g_gicr;
        }
        g_gicr_cpu[0] = own;
        gicv3_redistributor_init(own);
        gic_write32(g_gicd, GICD_CTLR,
                     GICD_CTLR_ARE_NS | GICD_CTLR_ENABLE_GRP1NS);
        gicd_wait_rwp();
        gicv3_cpu_init();
    } else {
        gicd_configure_ppi_level(27u);
        gicd_configure_ppi_level(30u);
        gic_write32(g_gicd, GICD_ISENABLER, 0x0000FFFFu); /* SGIs, banked */
        gic_write32(g_gicd, GICD_CTLR, GICD_CTLR_ENABLE_GRP0);
        gicd_wait_rwp();
        gicv2_cpu_init();
    }

    g_gic_ready = 1;
    return 0;
}

void arm64_gic_eoi(uint32_t irq)
{
    if ((irq & 0x3FFu) >= GIC_MAX_IRQ) {
        return;
    }

    if (g_gic_version >= GIC_VERSION_3) {
        gicv3_write_eoir1(irq);
    } else if (g_gicc) {
        gic_write32(g_gicc, GICC_EOIR, irq);
    }
}

uint32_t arm64_gic_read_iar(void)
{
    if (g_gic_version >= GIC_VERSION_3) {
        return (uint32_t)gicv3_read_iar1();
    }
    if (g_gicc) return gic_read32(g_gicc, GICC_IAR);
    return GIC_SPURIOUS_IRQ;
}

int arm64_gic_route_irq(uint32_t irq, uint32_t vector)
{
    (void)vector;
    if (!g_gic_ready || irq >= GIC_MAX_IRQ) return -1;
    if (g_gic_version >= GIC_VERSION_3) {
        gicd_route_spi_v3(irq);
    } else if (irq >= 32u) {
        gicd_set_spi_target_v2(irq);
    }
    gic_clear_pending_irq(irq);
    gic_clear_active_irq(irq);
    arm64_gic_unmask_irq(irq);
    gic_barrier();
    return 0;
}

void arm64_gic_mask_irq(uint32_t irq)
{
    if (!g_gic_ready || irq >= GIC_MAX_IRQ) return;
    volatile uint32_t *gicr = gicr_this_cpu();
    if (g_gic_version >= GIC_VERSION_3 && irq < 32u && gicr != 0) {
        gic_write32(gicr, GICR_ICENABLER0, 1u << irq);
        return;
    }
    gic_write32(g_gicd, GICD_ICENABLER + ((irq / 32u) * 4u),
                 1u << (irq % 32u));
}

void arm64_gic_unmask_irq(uint32_t irq)
{
    if (!g_gic_ready || irq >= GIC_MAX_IRQ) return;
    volatile uint32_t *gicr = gicr_this_cpu();
    if (g_gic_version >= GIC_VERSION_3 && irq < 32u && gicr != 0) {
        gic_write32(gicr, GICR_ISENABLER0, 1u << irq);
        return;
    }
    gic_write32(g_gicd, GICD_ISENABLER + ((irq / 32u) * 4u),
                 1u << (irq % 32u));
}

/* Per-CPU half of arm64_gic_init(), for a secondary CPU coming up: its
 * redistributor (GICv3) or its banked distributor registers (GICv2), and its
 * CPU interface. The distributor itself is shared and already enabled. */
int arm64_gic_init_secondary(void)
{
    if (!g_gic_ready) {
        return -1;
    }
    if (g_gic_version >= GIC_VERSION_3) {
        volatile uint32_t *own = gicv3_find_redistributor();
        if (own == 0) {
            return -1;
        }
        uint32_t cpu = smp_get_current_cpu_id();
        if (cpu < (uint32_t)OS_CONFIG_SMP_MAX_CPUS) {
            g_gicr_cpu[cpu] = own;
        }
        gicv3_redistributor_init(own);
        gicv3_cpu_init();
    } else {
        /* SGI/PPI enables, priorities and configuration are banked per CPU. */
        gic_write32(g_gicd, GICD_ICENABLER, 0xFFFFFFFFu);
        gic_write32(g_gicd, GICD_ICPENDR, 0xFFFFFFFFu);
        gic_write32(g_gicd, GICD_ICACTIVER, 0xFFFFFFFFu);
        for (uint32_t i = 0; i < 8u; ++i) {
            gic_write32(g_gicd, GICD_IPRIORITYR + i * 4u, GIC_PRIORITY_DEFAULT);
        }
        gicd_configure_ppi_level(27u);
        gicd_configure_ppi_level(30u);
        gic_write32(g_gicd, GICD_ISENABLER, 0x0000FFFFu);
        gicv2_cpu_init();
    }
    gic_barrier();
    return 0;
}

/* Raise SGI `sgi` (0-15) on the CPU whose MPIDR is `target_mpidr` (GICv3)
 * / whose GIC CPU interface number is `target_iface` (GICv2). */
void arm64_gic_send_sgi(uint64_t target_mpidr, uint32_t target_iface, uint32_t sgi)
{
    if (!g_gic_ready || sgi > 15u) {
        return;
    }
    __asm__ volatile("dsb ishst" ::: "memory");
    if (g_gic_version >= GIC_VERSION_3) {
        uint64_t aff0 = target_mpidr & 0xFFu;
        uint64_t aff1 = (target_mpidr >> 8) & 0xFFu;
        uint64_t aff2 = (target_mpidr >> 16) & 0xFFu;
        uint64_t aff3 = (target_mpidr >> 32) & 0xFFu;
        /* TargetList covers Aff0 values 0-15 within the Aff3.Aff2.Aff1
         * cluster; RS (bits 47:44) selects which group of 16. */
        uint64_t rs = aff0 >> 4;
        uint64_t value = (aff3 << 48) | (rs << 44) | (aff2 << 32) |
                         ((uint64_t)sgi << 24) | (aff1 << 16) |
                         (1ULL << (aff0 & 0xFu));
        __asm__ volatile("msr S3_0_C12_C11_5, %0; isb" :: "r"(value) : "memory");
    } else {
        gic_write32(g_gicd, GICD_SGIR,
                     ((1u << (target_iface & 7u)) << 16) | sgi);
    }
}

static int arm64_interrupt_configure(const void *firmware_info)
{
    const acpi_info_t *info = (const acpi_info_t *)firmware_info;
    uint64_t gicd_base = (info != 0 && info->gicd_base != 0) ?
                         info->gicd_base : 0x08000000ULL;
    uint64_t gicc_base = (info != 0) ? info->gicc_base : 0;
    uint64_t gicr_base = (info != 0) ? info->gicr_base : 0;

    if (gicr_base == 0 && info != 0 && info->gic_version >= GIC_VERSION_3) {
        gicr_base = 0x080A0000ULL;
    }
    if (gicc_base == 0 && gicr_base == 0) {
        gicc_base = 0x08010000ULL;
    }

    return arm64_gic_init(gicd_base, gicr_base, gicc_base);
}

static int arm64_using_local_timer(void)
{
    return 1;
}

static const interrupt_ops_t g_arm64_interrupt_ops = {
    .configure = arm64_interrupt_configure,
    .eoi = arm64_gic_eoi,
    .route_irq = arm64_gic_route_irq,
    .mask_irq = arm64_gic_mask_irq,
    .unmask_irq = arm64_gic_unmask_irq,
    .using_local_timer = arm64_using_local_timer,
};

const interrupt_ops_t *interrupt_ops_get(void)
{
    return &g_arm64_interrupt_ops;
}

void platform_interrupts_init_legacy_pic(void)
{
}

int platform_interrupts_configure(const acpi_info_t *info)
{
    return arm64_interrupt_configure(info);
}

void platform_interrupts_eoi(uint16_t vector)
{
    arm64_gic_eoi(vector);
}

int platform_interrupts_using_lapic(void)
{
    return 0;
}

void platform_interrupts_route_pit(void)
{
}

void platform_interrupts_mask_pit(void)
{
}

int lapic_init(uint64_t phys_base) { (void)phys_base; return -1; }
int lapic_is_present(void) { return 0; }
void lapic_eoi(void) {}
uint32_t lapic_get_id(void) { return 0; }
uint32_t lapic_timer_current(void) { return 0; }
int lapic_timer_start(uint8_t vector, uint32_t initial_count, int periodic, uint32_t divide)
{
    (void)vector; (void)initial_count; (void)periodic; (void)divide;
    return -1;
}
void lapic_timer_stop(void) {}
void lapic_timer_ap_init(void) {}
void lapic_send_ipi(uint8_t apic_id, uint32_t icr_low) { (void)apic_id; (void)icr_low; }
