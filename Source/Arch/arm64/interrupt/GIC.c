#include "GIC.h"
#include "interfaces/interrupt_ops.h"
#include "Platform/acpi/ACPI.h"

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
static volatile uint32_t *g_gicr;
static uint8_t g_gic_version;
static int g_gic_ready;

static inline uint32_t mmio_read32(volatile uint32_t *base, uint64_t off)
{
    return base[off / 4];
}

static inline void mmio_write32(volatile uint32_t *base, uint64_t off, uint32_t value)
{
    base[off / 4] = value;
}

static inline void mmio_write64(volatile uint32_t *base, uint64_t off, uint64_t value)
{
    base[off / 4] = (uint32_t)value;
    base[(off / 4) + 1u] = (uint32_t)(value >> 32);
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

static void gicd_wait_rwp(void)
{
    for (uint32_t i = 0; i < 100000u; ++i) {
        if ((mmio_read32(g_gicd, GICD_CTLR) & GICD_CTLR_RWP) == 0u) {
            break;
        }
    }
}

static uint32_t gicd_irq_count(void)
{
    uint32_t typer = mmio_read32(g_gicd, GICD_TYPER);
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
    uint32_t value = mmio_read32(g_gicd, off);
    value &= ~(0xFFu << shift);
    value |=  (0x01u << shift);
    mmio_write32(g_gicd, off, value);
}

static void gicd_route_spi_v3(uint32_t irq)
{
    if (irq >= 32u) {
        mmio_write64(g_gicd, GICD_IROUTER + ((uint64_t)irq * 8u),
                     gicv3_current_affinity());
    }
}

static void gicv3_configure_ppi_level(uint32_t irq)
{
    if (g_gicr == 0 || irq < 16u || irq >= 32u) {
        return;
    }

    uint32_t shift = (irq - 16u) * 2u;
    uint32_t value = mmio_read32(g_gicr, GICR_ICFGR1);
    value &= ~(0x3u << shift);
    mmio_write32(g_gicr, GICR_ICFGR1, value);
}

static void gicd_configure_ppi_level(uint32_t irq)
{
    if (irq < 16u || irq >= 32u) {
        return;
    }

    uint64_t off = GICD_ICFGR + ((uint64_t)(irq / 16u) * 4u);
    uint32_t shift = (irq % 16u) * 2u;
    uint32_t value = mmio_read32(g_gicd, off);
    value &= ~(0x3u << shift);
    mmio_write32(g_gicd, off, value);
}

static void gic_clear_pending_irq(uint32_t irq)
{
    if (g_gic_version >= GIC_VERSION_3 && irq < 32u && g_gicr != 0) {
        mmio_write32(g_gicr, GICR_ICPENDR0, 1u << irq);
        return;
    }

    mmio_write32(g_gicd, GICD_ICPENDR + ((irq / 32u) * 4u),
                 1u << (irq % 32u));
}

static void gic_clear_active_irq(uint32_t irq)
{
    if (g_gic_version >= GIC_VERSION_3 && irq < 32u && g_gicr != 0) {
        mmio_write32(g_gicr, GICR_ICACTIVER0, 1u << irq);
        return;
    }

    mmio_write32(g_gicd, GICD_ICACTIVER + ((irq / 32u) * 4u),
                 1u << (irq % 32u));
}

static void gicv2_cpu_init(void)
{
    if (g_gicc) {
        mmio_write32(g_gicc, GICC_PMR, 0xFFu);
        mmio_write32(g_gicc, GICC_BPR, 0u);
        mmio_write32(g_gicc, GICC_CTLR, GICC_CTLR_ENABLE);
    }
}

static void gicv3_redistributor_init(void)
{
    if (g_gicr == 0) {
        return;
    }

    uint32_t waker = mmio_read32(g_gicr, GICR_WAKER);
    waker &= ~GICR_WAKER_PROCESSOR_SLEEP;
    mmio_write32(g_gicr, GICR_WAKER, waker);

    for (uint32_t i = 0; i < 100000u; ++i) {
        if ((mmio_read32(g_gicr, GICR_WAKER) &
             GICR_WAKER_CHILDREN_ASLEEP) == 0u) {
            break;
        }
    }

    mmio_write32(g_gicr, GICR_IGROUPR0, 0xFFFFFFFFu);
    mmio_write32(g_gicr, GICR_ICENABLER0, 0xFFFFFFFFu);
    mmio_write32(g_gicr, GICR_ICPENDR0, 0xFFFFFFFFu);
    mmio_write32(g_gicr, GICR_ICACTIVER0, 0xFFFFFFFFu);
    gicv3_configure_ppi_level(27u);
    gicv3_configure_ppi_level(30u);
    for (uint32_t i = 0; i < 8u; ++i) {
        mmio_write32(g_gicr, GICR_IPRIORITYR + i * 4u, GIC_PRIORITY_DEFAULT);
    }
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

    mmio_write32(g_gicd, GICD_CTLR, 0);
    gicd_wait_rwp();

    uint32_t irq_count = gicd_irq_count();
    uint32_t group_regs = (irq_count + 31u) / 32u;
    for (uint32_t i = 0; i < group_regs; ++i) {
        uint32_t group = (g_gic_version >= GIC_VERSION_3) ?
                         0xFFFFFFFFu : 0x00000000u;
        mmio_write32(g_gicd, GICD_IGROUPR + i * 4u, group);
        mmio_write32(g_gicd, GICD_ICENABLER + i * 4u, 0xFFFFFFFFu);
        mmio_write32(g_gicd, GICD_ICPENDR + i * 4u, 0xFFFFFFFFu);
        mmio_write32(g_gicd, GICD_ICACTIVER + i * 4u, 0xFFFFFFFFu);
    }
    for (uint32_t i = 0; i < (irq_count / 4u); ++i) {
        mmio_write32(g_gicd, GICD_IPRIORITYR + i * 4u, GIC_PRIORITY_DEFAULT);
    }

    if (g_gic_version >= GIC_VERSION_3) {
        gicv3_redistributor_init();
        mmio_write32(g_gicd, GICD_CTLR,
                     GICD_CTLR_ARE_NS | GICD_CTLR_ENABLE_GRP1NS);
        gicd_wait_rwp();
        gicv3_cpu_init();
    } else {
        gicd_configure_ppi_level(27u);
        gicd_configure_ppi_level(30u);
        mmio_write32(g_gicd, GICD_CTLR, GICD_CTLR_ENABLE_GRP0);
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
        mmio_write32(g_gicc, GICC_EOIR, irq);
    }
}

uint32_t arm64_gic_read_iar(void)
{
    if (g_gic_version >= GIC_VERSION_3) {
        return (uint32_t)gicv3_read_iar1();
    }
    if (g_gicc) return mmio_read32(g_gicc, GICC_IAR);
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
    if (g_gic_version >= GIC_VERSION_3 && irq < 32u && g_gicr != 0) {
        mmio_write32(g_gicr, GICR_ICENABLER0, 1u << irq);
        return;
    }
    mmio_write32(g_gicd, GICD_ICENABLER + ((irq / 32u) * 4u),
                 1u << (irq % 32u));
}

void arm64_gic_unmask_irq(uint32_t irq)
{
    if (!g_gic_ready || irq >= GIC_MAX_IRQ) return;
    if (g_gic_version >= GIC_VERSION_3 && irq < 32u && g_gicr != 0) {
        mmio_write32(g_gicr, GICR_ISENABLER0, 1u << irq);
        return;
    }
    mmio_write32(g_gicd, GICD_ISENABLER + ((irq / 32u) * 4u),
                 1u << (irq % 32u));
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
