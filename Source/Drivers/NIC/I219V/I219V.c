#include "I219V.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Uncomment to try alternative RXDCTL offset (standard e1000e)
// #define I219V_RXDCTL_ALT_OFFSET 0x03028u

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"
#else
#include "Drivers/Module/PCI_Main.h"
#include "MemoryManagement/DMA_Memory.h"
#include "MemoryManagement/Memory_Main.h"
#include "mmu/Paging_Main.h"
#include "Core/sync/Spinlock.h"
#endif

#ifdef IMPLUS_DRIVER_MODULE
static const driver_binary_t *g_api = NULL;

#define malloc             g_api->malloc
#define free               g_api->free
#define dma_alloc          g_api->dma_alloc
#define dma_free           g_api->dma_free
#define dma_alloc_ex       g_api->mem.dma_alloc_ex
#define memset             g_api->memset
#define memcpy             g_api->memcpy
#define map_mmio_range     g_api->hw.map_mmio_range
#define pci_read_config    g_api->pci_read_config
#define pci_write_config   g_api->pci_write_config
#define timer_msleep       g_api->timer.msleep

#define hal_cpu_save_interrupts    g_api->hal.cpu_save_interrupts
#define hal_cpu_restore_interrupts g_api->hal.cpu_restore_interrupts
#define hal_cpu_pause             g_api->hal.cpu_pause

typedef struct { volatile int locked; } spinlock_t;
static inline void spinlock_init(spinlock_t *l)   { l->locked = 0; }
static inline void spinlock_lock(spinlock_t *l)   {
    while (__sync_lock_test_and_set(&l->locked, 1)) {
        while (l->locked) { hal_cpu_pause(); }
    }
}
static inline void spinlock_unlock(spinlock_t *l) { __sync_lock_release(&l->locked); }

static inline uint64_t irq_save_disable(void) { return hal_cpu_save_interrupts(); }
static inline void irq_restore(uint64_t flags) { hal_cpu_restore_interrupts(flags); }

static inline bool dma_init(void) { return true; }
#endif

#define INTEL_VENDOR 0x8086u

static const uint16_t i219_device_ids[] = {
    0x15B7, 0x15BC, 0x15BE, 0x15C0, 0x15C3,
    0x1559, 0x15B5, 0x15B6, 0x15B9, 0x15BA,
    0x15B8, 0x15BD, 0x15BF, 0x15C1, 0x15C2,
    0x155A, 0x15BB, 0x15C4, 0x15C5, 0x15C6,
    0x15C7, 0x15C8, 0x15C9, 0x15CA, 0x15CB,
    0x15CC, 0x15CD, 0x15CE, 0x15CF, 0x15D0,
    0x15D1, 0x15D2, 0x15D3, 0x15D4, 0x15D5,
    0x15D6, 0x15D7, 0x15D8, 0x15D9, 0x15DA,
    0x15DB, 0x15DC, 0x15DD, 0x15DE, 0x15DF,
    0x15E0, 0x15E1, 0x15E2,
};

#define REG_CTRL         0x0000u
#define REG_STATUS       0x0008u
#define REG_CTRL_EXT     0x0018u
#define REG_ICR          0x00C0u
#define REG_ICS          0x00C8u
#define REG_IMS          0x00D0u
#define REG_IMC          0x00D8u
#define REG_IAC          0x0100u
#define REG_RCTRL        0x0100u
#define REG_TCTRL        0x0400u
#define REG_MDIC         0x0020u
#define REG_RDBAL        0x2800u
#define REG_RDBAH        0x2804u
#define REG_RDLEN        0x2808u
#define REG_RDH          0x2810u
#define REG_RDT          0x2818u
#define REG_RDTR         0x2820u
#define REG_RDBAL_S      0x0C200u
#define REG_RDBAH_S      0x0C204u
#define REG_RDLEN_S      0x0C208u
#define REG_RDH_S        0x0C20Cu
#define REG_RDT_S        0x0C210u
#define REG_TDBAL        0x3800u
#define REG_TDBAH        0x3804u
#define REG_TDLEN        0x3808u
#define REG_TDH          0x3810u
#define REG_TDT          0x3818u
#define REG_RAL(n)       (0x5400u + (uint32_t)(n) * 8u)
#define REG_RAH(n)       (0x5404u + (uint32_t)(n) * 8u)
#define REG_MNGC         0x5220u
#define REG_EEPROM_STATUS 0x4010u
#define REG_EEC          0x1008u
#define REG_MTA(n)       (0x5200u + (uint32_t)(n) * 4u)
#define REG_RFCTL        0x5008u
#define RFCTL_ACK_DIS    (1u << 5)
#define RFCTL_IPV6_EX_DIS (1u << 0)
#define REG_RXDCTL       0x0C214u
#define REG_RXDCTL_STD   0x03028u
#define RXDCTL_ENABLE    (1u << 25)
#define REG_SRRCTL       0x0C218u
#define SRRCTL_DESCTYPE_LEGACY 0u
#define SRRCTL_DESCTYPE_ADVB   (1u << 1)
#define SRRCTL_BSIZE_MASK (0x3Fu << 25)
#define SRRCTL_BSIZE_2048 (0x12u << 25)
#define REG_RXPBS        0x07044u
#define RXPBS_SIZE_MASK  0x0000001Fu
#define RXPBS_SIZE_384KB 0x00000006u
#define RXPBS_SIZE_192KB 0x00000003u
#define REG_FEXTNVM3      0x0F034u
#define FEXTNVM3_DISABLE_RX_PB (1u << 11)
#define REG_IAM           0x00D4u
#define REG_SW_FW_SYNC    0x05B5Cu
#define SW_FW_SYNC_SW_MGMT (1u << 0)
#define SW_FW_SYNC_FW_MGMT (1u << 1)

#define CTRL_RST         (1u << 0)
#define CTRL_SLU         (1u << 6)
#define CTRL_ASDE        (1u << 5)
#define CTRL_FRCSPD      (1u << 11)
#define CTRL_FRCFDX      (1u << 10)
#define CTRL_FD          (1u << 0)
#define CTRL_SPEED_SHIFT 7u
#define CTRL_SPEED_MASK  (3u << CTRL_SPEED_SHIFT)
#define CTRL_SPEED_10    (0u << CTRL_SPEED_SHIFT)
#define CTRL_SPEED_100   (1u << CTRL_SPEED_SHIFT)
#define CTRL_SPEED_1000  (2u << CTRL_SPEED_SHIFT)
#define CTRL_RFCE        (1u << 5)
#define CTRL_TFCE        (1u << 6)
#define CTRL_LANPHYPC    (1u << 26)

#define CTRL_EXT_DRV_LOAD (1u << 27)

#define STATUS_LU        (1u << 1)
#define STATUS_FD        (1u << 0)
#define STATUS_SPEED_SHIFT 7u
#define STATUS_SPEED_MASK  (3u << STATUS_SPEED_SHIFT)
#define STATUS_LAN_INIT_DONE (1u << 9)
#define STATUS_PHYRA         (1u << 10)

#define RCTRL_EN         (1u << 0)
#define RCTRL_UPE        (1u << 3)
#define RCTRL_MPE        (1u << 4)
#define RCTRL_LPE        (1u << 5)
#define RCTRL_BSIZE_2048 (0u << 16)
#define RCTRL_SECRC      (1u << 26)
#define RCTRL_BAM        (1u << 15)

#define TCTRL_EN         (1u << 0)
#define TCTRL_PSP        (1u << 3)
#define TCTRL_CT_SHIFT   4u
#define TCTRL_COLD_SHIFT 12u
#define TCTRL_COLD_HD    (0x3Fu << TCTRL_COLD_SHIFT)

#define ICR_TXDW         (1u << 0)
#define ICR_TXQE         (1u << 1)
#define ICR_LSC          (1u << 2)
#define ICR_RXDMT0       (1u << 4)
#define ICR_RXT0         (1u << 7)
#define ICR_OTHER        (1u << 16)

#define RAH_AV           (1u << 31)

#define MDIC_DATA_SHIFT  0u
#define MDIC_REG_SHIFT   16u
#define MDIC_PHY_SHIFT   21u
#define MDIC_OP_WRITE    (1u << 26)
#define MDIC_OP_READ     (2u << 26)
#define MDIC_READY       (1u << 28)
#define MDIC_INT_EN      (1u << 29)
#define MDIC_ERROR       (1u << 30)

#define EEC_PRES         (1u << 8)
#define EEC_AUTO_RD      (1u << 9)

typedef struct __attribute__((packed)) {
    uint64_t addr;
    uint16_t length;
    uint16_t cksum;
    uint8_t  status;
    uint8_t  errors;
    uint16_t special;
} e1000_rx_desc_t;

#define RXD_STAT_DD     (1u << 0)
#define RXD_STAT_EOP    (1u << 1)
#define RXD_STAT_IXSM   (1u << 2)
#define RXD_STAT_VP     (1u << 3)

typedef struct __attribute__((packed)) {
    uint64_t addr;
    uint16_t length;
    uint8_t  cso;
    uint8_t  cmd;
    uint8_t  status;
    uint8_t  css;
    uint16_t special;
} e1000_tx_desc_t;

#define TXD_CMD_EOP     (1u << 0)
#define TXD_CMD_IFCS    (1u << 1)
#define TXD_CMD_IC      (1u << 2)
#define TXD_CMD_RS      (1u << 3)
#define TXD_CMD_RPS     (1u << 4)

#define TXD_STAT_DD     (1u << 0)
#define TXD_STAT_EC     (1u << 1)
#define TXD_STAT_LC     (1u << 2)

#define I219V_DESC_COUNT    256
#define I219V_BUF_SIZE      2048
#define I219V_DEFAULT_MTU   1500

typedef struct {
    uint8_t  *virt;
    uint64_t  phys;
} i219v_dma_buffer_t;

typedef struct {
    volatile e1000_rx_desc_t *desc;
    i219v_dma_buffer_t *buffers;
    uint16_t  count;
    uint16_t  next_to_clean;
    uint16_t  next_to_use;
} i219v_rx_ring_t;

typedef struct {
    volatile e1000_tx_desc_t *desc;
    i219v_dma_buffer_t *buffers;
    uint16_t  count;
    uint16_t  next_to_clean;
    uint16_t  next_to_use;
    bool     *in_use;
} i219v_tx_ring_t;

#define REG_GPRC         0x4074u
#define REG_TPT          0x40D4u

static struct {
    spinlock_t   lock;
    int          ready;
    uint16_t     mtu;
    uint8_t      mac[6];

    volatile uint8_t *mmio;
    uint8_t           bus, dev, func;

    i219v_rx_ring_t rx_ring;
    i219v_tx_ring_t tx_ring;

    uint32_t msix_vectors[3];
    driver_nic_rx_callback_t rx_callback;
} g_i219v;

static inline uint32_t mmio_read32(volatile uint8_t *base, uint32_t reg)
{
    return *(volatile uint32_t *)(base + reg);
}

static inline void mmio_write32(volatile uint8_t *base, uint32_t reg, uint32_t value)
{
    *(volatile uint32_t *)(base + reg) = value;
}

static inline void memory_barrier(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

static inline void clflush(volatile const void *addr)
{
    __asm__ volatile("clflush (%0)" :: "r"(addr) : "memory");
}

static inline void clflush_range(volatile const void *addr, size_t len)
{
    for (size_t off = 0; off < len; off += 64) {
        clflush((volatile const void *)((uintptr_t)addr + off));
    }
    memory_barrier();
}

static uint32_t pci_cfg_read32_raw(uint8_t bus, uint8_t device, uint8_t func, uint16_t offset)
{
    return pci_read_config(bus, device, func, (uint8_t)(offset & 0xFFu));
}

static void pci_cfg_write32_raw(uint8_t bus, uint8_t device, uint8_t func, uint16_t offset, uint32_t value)
{
    pci_write_config(bus, device, func, (uint8_t)(offset & 0xFFu), value);
}

static int find_i219v(uint8_t *out_bus, uint8_t *out_dev, uint8_t *out_func,
                      uint64_t *out_bar0_phys, uint32_t *out_bar0_size)
{
    if (out_bus == NULL || out_dev == NULL || out_func == NULL ||
        out_bar0_phys == NULL || out_bar0_size == NULL) {
        return 0;
    }

    for (uint16_t bus = 0; bus < 256u; ++bus) {
        for (uint8_t device = 0; device < 32u; ++device) {
            for (uint8_t func = 0; func < 8u; ++func) {
                uint32_t vd = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x00u);
                uint16_t vendor_id = (uint16_t)(vd & 0xFFFFu);
                uint16_t device_id = (uint16_t)((vd >> 16) & 0xFFFFu);

                if (vendor_id == 0xFFFFu) {
                    if (func == 0u) { break; }
                    continue;
                }

                if (vendor_id != INTEL_VENDOR) {
                    if (func == 0u) {
                        uint32_t hdr = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x0Cu);
                        if (((hdr >> 16) & 0x80u) == 0u) { break; }
                    }
                    continue;
                }

                int id_match = 0;
                uint32_t id_count = sizeof(i219_device_ids) / sizeof(i219_device_ids[0]);
                for (uint32_t i = 0; i < id_count; ++i) {
                    if (device_id == i219_device_ids[i]) {
                        id_match = 1;
                        break;
                    }
                }
                if (!id_match) {
                    if (func == 0u) {
                        uint32_t hdr = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x0Cu);
                        if (((hdr >> 16) & 0x80u) == 0u) { break; }
                    }
                    continue;
                }

                uint32_t class_reg = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x08u);
                uint8_t class_code = (uint8_t)((class_reg >> 24) & 0xFFu);
                uint8_t subclass = (uint8_t)((class_reg >> 16) & 0xFFu);
                if (class_code != 0x02u || subclass != 0x00u) {
                    if (func == 0u) {
                        uint32_t hdr = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x0Cu);
                        if (((hdr >> 16) & 0x80u) == 0u) { break; }
                    }
                    continue;
                }

                *out_bus = (uint8_t)bus;
                *out_dev = device;
                *out_func = func;

                uint32_t cmd = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x04u);
                cmd |= (1u << 1) | (1u << 2);
                pci_cfg_write32_raw((uint8_t)bus, device, func, 0x04u, cmd);

                uint32_t bar0 = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x10u);
                if ((bar0 & 0x1u) != 0u) {
                    return 0;
                }

                pci_cfg_write32_raw((uint8_t)bus, device, func, 0x10u, 0xFFFFFFFFu);
                uint32_t bar0_mask = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x10u);
                pci_cfg_write32_raw((uint8_t)bus, device, func, 0x10u, bar0);

                if (bar0_mask == 0u || bar0_mask == 0xFFFFFFFFu) {
                    return 0;
                }

                uint32_t size = (~(bar0_mask & ~0xFu)) + 1u;
                uint64_t phys = (uint64_t)(bar0 & ~0xFu);

                if ((bar0 >> 1) & 0x3u) {
                    uint32_t bar2 = pci_cfg_read32_raw((uint8_t)bus, device, func, 0x14u);
                    phys |= ((uint64_t)bar2 << 32);
                }

                *out_bar0_phys = phys;
                *out_bar0_size = size;

                return 1;
            }
        }
    }

    return 0;
}

static int i219v_read_mac_from_eec(void)
{
    uint32_t eec = mmio_read32(g_i219v.mmio, REG_EEC);
    if ((eec & EEC_PRES) == 0u) {
        return 0;
    }

    uint32_t timeout = 10000u;
    while ((mmio_read32(g_i219v.mmio, REG_EEC) & EEC_AUTO_RD) == 0u) {
        if (--timeout == 0u) { return 0; }
        hal_cpu_pause();
    }

    for (uint32_t i = 0; i < 3u; ++i) {
        uint32_t eec_val = mmio_read32(g_i219v.mmio, REG_EEC);
        eec_val &= ~0xFFFFu;
        eec_val |= (uint32_t)(((i << 8) | 0x01u) & 0xFFFFu);
        mmio_write32(g_i219v.mmio, REG_EEC, eec_val);

        timeout = 10000u;
        while ((mmio_read32(g_i219v.mmio, REG_EEC) & 0x04u) == 0u) {
            if (--timeout == 0u) { return 0; }
            hal_cpu_pause();
        }

        uint32_t data = (mmio_read32(g_i219v.mmio, REG_EEC) >> 16) & 0xFFFFu;
        g_i219v.mac[i * 2u] = (uint8_t)(data & 0xFFu);
        g_i219v.mac[i * 2u + 1u] = (uint8_t)((data >> 8) & 0xFFu);
    }

    return 1;
}

static void i219v_read_mac(void)
{
    uint32_t ral = mmio_read32(g_i219v.mmio, REG_RAL(0));
    uint32_t rah = mmio_read32(g_i219v.mmio, REG_RAH(0));

    if (ral == 0xFFFFFFFFu && rah == 0xFFFFFFFFu) {
        if (i219v_read_mac_from_eec()) {
            return;
        }
        g_i219v.mac[0] = 0x00u;
        g_i219v.mac[1] = 0x15u;
        g_i219v.mac[2] = 0x17u;
        g_i219v.mac[3] = 0x00u;
        g_i219v.mac[4] = 0x00u;
        g_i219v.mac[5] = 0x01u;
        return;
    }

    for (uint32_t i = 0; i < 4u; ++i) {
        g_i219v.mac[i] = (uint8_t)((ral >> (i * 8u)) & 0xFFu);
    }
    for (uint32_t i = 0; i < 2u; ++i) {
        g_i219v.mac[4u + i] = (uint8_t)((rah >> (i * 8u)) & 0xFFu);
    }
}

static void i219v_reap_tx(void)
{
    while (g_i219v.tx_ring.in_use[g_i219v.tx_ring.next_to_clean]) {
        volatile e1000_tx_desc_t *desc =
            &g_i219v.tx_ring.desc[g_i219v.tx_ring.next_to_clean];
        clflush((volatile const void *)desc);
        if (!(desc->status & TXD_STAT_DD))
            break;
        desc->status = 0;
        g_i219v.tx_ring.in_use[g_i219v.tx_ring.next_to_clean] = false;
        g_i219v.tx_ring.next_to_clean =
            (uint16_t)((g_i219v.tx_ring.next_to_clean + 1u) % g_i219v.tx_ring.count);
    }
}

static void i219v_rx_irq(void *context)
{
    (void)context;
    uint32_t icr = mmio_read32(g_i219v.mmio, REG_ICR);
    (void)icr;
}

bool i219v_init(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_i219v.lock);

    if (g_i219v.ready != 0) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return true;
    }

    if (!dma_init()) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }

    uint8_t bus = 0, dev = 0, func = 0;
    uint64_t bar0_phys = 0;
    uint32_t bar0_size = 0;
    if (!find_i219v(&bus, &dev, &func, &bar0_phys, &bar0_size)) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }

    g_i219v.bus = bus;
    g_i219v.dev = dev;
    g_i219v.func = func;

    volatile uint8_t *mmio = (volatile uint8_t *)map_mmio_range(bar0_phys, (size_t)bar0_size);
    if (mmio == NULL) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }
    g_i219v.mmio = mmio;

    mmio_write32(g_i219v.mmio, REG_CTRL, CTRL_RST);
    timer_msleep(1);
    uint32_t timeout = 100u;
    uint32_t rst_asserted = 0;
    while ((mmio_read32(g_i219v.mmio, REG_CTRL) & CTRL_RST) != 0u) {
        if (++rst_asserted == 1u) {
            mmio_write32(g_i219v.mmio, REG_RDBAL_S, 0xDEAD0000u);
            mmio_write32(g_i219v.mmio, REG_RDH_S, 0xBEEF0000u);
        }
        if (--timeout == 0u) { break; }
        timer_msleep(1);
    }
    mmio_write32(g_i219v.mmio, REG_IMC, 0xFFFFFFFFu);
    mmio_read32(g_i219v.mmio, REG_ICR);

    mmio_write32(g_i219v.mmio, REG_CTRL_EXT,
        mmio_read32(g_i219v.mmio, REG_CTRL_EXT) | CTRL_EXT_DRV_LOAD);
    {
        uint32_t swfw = mmio_read32(g_i219v.mmio, REG_SW_FW_SYNC);
        if (swfw & SW_FW_SYNC_FW_MGMT) {
            uint32_t wait = 1000u;
            while ((mmio_read32(g_i219v.mmio, REG_SW_FW_SYNC) & SW_FW_SYNC_FW_MGMT) != 0u) {
                if (--wait == 0u) { break; }
                timer_msleep(1);
            }
        }
        mmio_write32(g_i219v.mmio, REG_SW_FW_SYNC, swfw | SW_FW_SYNC_SW_MGMT);
    }

    {
        uint32_t ctrl_phy = mmio_read32(g_i219v.mmio, REG_CTRL);
        ctrl_phy |= CTRL_LANPHYPC;
        mmio_write32(g_i219v.mmio, REG_CTRL, ctrl_phy);
        mmio_write32(g_i219v.mmio, REG_CTRL, ctrl_phy);
        timer_msleep(1);
        ctrl_phy &= ~CTRL_LANPHYPC;
        mmio_write32(g_i219v.mmio, REG_CTRL, ctrl_phy);
        mmio_write32(g_i219v.mmio, REG_CTRL, ctrl_phy);
        timer_msleep(5);
    }
    {
        uint32_t phyra_wait = 100u;
        while (phyra_wait > 0u) {
            uint32_t s = mmio_read32(g_i219v.mmio, REG_STATUS);
            if (!(s & STATUS_PHYRA))
                break;
            timer_msleep(1);
            phyra_wait--;
        }
    }

    uint32_t link_wait = 3000u;
    int link_up = 0;
    while (link_wait > 0u) {
        uint32_t s = mmio_read32(g_i219v.mmio, REG_STATUS);
        if (s & STATUS_LU) {
            link_up = 1;
            break;
        }
        timer_msleep(1);
        link_wait--;
    }

    if (!link_up) {
        uint32_t ctrl = mmio_read32(g_i219v.mmio, REG_CTRL);
        ctrl |= CTRL_SLU | CTRL_FRCSPD | CTRL_FRCFDX | CTRL_FD;
        ctrl &= ~CTRL_SPEED_MASK;
        ctrl |= CTRL_SPEED_1000;
        mmio_write32(g_i219v.mmio, REG_CTRL, ctrl);
        timer_msleep(100);
    }

    if (g_api->pci.enable_msix(bus, dev, func, 3, g_i219v.msix_vectors) > 0) {
        g_api->pci.register_irq(g_i219v.msix_vectors[1], i219v_rx_irq, NULL);
    }

    i219v_read_mac();

    uint64_t rx_desc_phys = 0;
    uint32_t rx_desc_size = (uint32_t)I219V_DESC_COUNT * (uint32_t)sizeof(e1000_rx_desc_t);
    volatile e1000_rx_desc_t *rx_desc = (volatile e1000_rx_desc_t *)dma_alloc_ex(
        (size_t)rx_desc_size, 16u, 0xFFFFFFFFu, &rx_desc_phys);
    if (rx_desc == NULL || rx_desc_phys == 0) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }
    memset((void *)rx_desc, 0, (size_t)rx_desc_size);

    i219v_dma_buffer_t *rx_buffers = (i219v_dma_buffer_t *)malloc(
        (uint64_t)I219V_DESC_COUNT * (uint64_t)sizeof(i219v_dma_buffer_t));
    if (rx_buffers == NULL) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }
    memset(rx_buffers, 0, (size_t)I219V_DESC_COUNT * sizeof(i219v_dma_buffer_t));

    for (uint16_t i = 0; i < I219V_DESC_COUNT; ++i) {
        uint64_t buf_phys = 0;
        uint8_t *buf = (uint8_t *)dma_alloc(I219V_BUF_SIZE, &buf_phys);
        if (buf == NULL || buf_phys == 0) {
            for (uint16_t j = 0; j < i; ++j) {
                dma_free(rx_buffers[j].virt, I219V_BUF_SIZE);
            }
            free(rx_buffers);
            dma_free((void *)rx_desc, (size_t)rx_desc_size);
            spinlock_unlock(&g_i219v.lock);
            irq_restore(irq_flags);
            return false;
        }
        rx_buffers[i].virt = buf;
        rx_buffers[i].phys = buf_phys;
        rx_desc[i].addr = buf_phys;
        rx_desc[i].status = 0;
    }

    g_i219v.rx_ring.desc = rx_desc;
    g_i219v.rx_ring.buffers = rx_buffers;
    g_i219v.rx_ring.count = I219V_DESC_COUNT;
    g_i219v.rx_ring.next_to_clean = 0;
    g_i219v.rx_ring.next_to_use = 0;

    mmio_write32(g_i219v.mmio, REG_RDBAL, (uint32_t)(rx_desc_phys & 0xFFFFFFFFu));
    mmio_write32(g_i219v.mmio, REG_RDBAH, (uint32_t)((rx_desc_phys >> 32) & 0xFFFFFFFFu));
    mmio_write32(g_i219v.mmio, REG_RDLEN, rx_desc_size);
    mmio_write32(g_i219v.mmio, REG_RDH, 0);
    mmio_write32(g_i219v.mmio, REG_RDT, 0);
    mmio_write32(g_i219v.mmio, REG_RDBAL_S, (uint32_t)(rx_desc_phys & 0xFFFFFFFFu));
    mmio_write32(g_i219v.mmio, REG_RDBAH_S, (uint32_t)((rx_desc_phys >> 32) & 0xFFFFFFFFu));
    mmio_write32(g_i219v.mmio, REG_RDLEN_S, rx_desc_size);
    mmio_write32(g_i219v.mmio, REG_RDH_S, 0);
    mmio_write32(g_i219v.mmio, REG_RDT_S, 0);
    uint64_t tx_desc_phys = 0;
    uint32_t tx_desc_size = (uint32_t)I219V_DESC_COUNT * (uint32_t)sizeof(e1000_tx_desc_t);
    volatile e1000_tx_desc_t *tx_desc = (volatile e1000_tx_desc_t *)dma_alloc_ex(
        (size_t)tx_desc_size, 16u, 0xFFFFFFFFu, &tx_desc_phys);
    if (tx_desc == NULL || tx_desc_phys == 0) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }
    memset((void *)tx_desc, 0, (size_t)tx_desc_size);

    i219v_dma_buffer_t *tx_buffers = (i219v_dma_buffer_t *)malloc(
        (uint64_t)I219V_DESC_COUNT * (uint64_t)sizeof(i219v_dma_buffer_t));
    if (tx_buffers == NULL) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }
    memset(tx_buffers, 0, (size_t)I219V_DESC_COUNT * sizeof(i219v_dma_buffer_t));

    bool *tx_in_use = (bool *)malloc((uint64_t)I219V_DESC_COUNT);
    if (tx_in_use == NULL) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }
    memset(tx_in_use, 0, (size_t)I219V_DESC_COUNT);

    for (uint16_t i = 0; i < I219V_DESC_COUNT; ++i) {
        uint64_t buf_phys = 0;
        uint8_t *buf = (uint8_t *)dma_alloc(I219V_BUF_SIZE, &buf_phys);
        if (buf == NULL || buf_phys == 0) {
            spinlock_unlock(&g_i219v.lock);
            irq_restore(irq_flags);
            return false;
        }
        tx_buffers[i].virt = buf;
        tx_buffers[i].phys = buf_phys;
        tx_desc[i].addr = buf_phys;
    }

    g_i219v.tx_ring.desc = tx_desc;
    g_i219v.tx_ring.buffers = tx_buffers;
    g_i219v.tx_ring.count = I219V_DESC_COUNT;
    g_i219v.tx_ring.next_to_clean = 0;
    g_i219v.tx_ring.next_to_use = 0;
    g_i219v.tx_ring.in_use = tx_in_use;

    mmio_write32(g_i219v.mmio, REG_TDBAL, (uint32_t)(tx_desc_phys & 0xFFFFFFFFu));
    mmio_write32(g_i219v.mmio, REG_TDBAH, (uint32_t)((tx_desc_phys >> 32) & 0xFFFFFFFFu));
    mmio_write32(g_i219v.mmio, REG_TDLEN, tx_desc_size);
    mmio_write32(g_i219v.mmio, REG_TDH, 0);
    mmio_write32(g_i219v.mmio, REG_TDT, 0);
    for (uint32_t i = 0; i < 128u; ++i) {
        mmio_write32(g_i219v.mmio, REG_MTA(i), 0);
    }

    mmio_write32(g_i219v.mmio, REG_RFCTL, RFCTL_ACK_DIS | RFCTL_IPV6_EX_DIS);

    {
        uint32_t rxpbs = mmio_read32(g_i219v.mmio, REG_RXPBS);
        rxpbs = (rxpbs & ~RXPBS_SIZE_MASK) | RXPBS_SIZE_384KB;
        mmio_write32(g_i219v.mmio, REG_RXPBS, rxpbs);
    }

    mmio_write32(g_i219v.mmio, REG_SRRCTL, SRRCTL_DESCTYPE_LEGACY | SRRCTL_BSIZE_2048);
    {
        uint32_t rxdctl = mmio_read32(g_i219v.mmio, REG_RXDCTL);
        rxdctl |= RXDCTL_ENABLE;
        mmio_write32(g_i219v.mmio, REG_RXDCTL, rxdctl);
    }
    mmio_write32(g_i219v.mmio, REG_FEXTNVM3,
        mmio_read32(g_i219v.mmio, REG_FEXTNVM3) | FEXTNVM3_DISABLE_RX_PB);

    mmio_write32(g_i219v.mmio, REG_IAM, 0);

    mmio_write32(g_i219v.mmio, REG_RCTRL, RCTRL_EN | RCTRL_UPE | RCTRL_MPE | RCTRL_BAM | RCTRL_BSIZE_2048 | RCTRL_SECRC);

    uint32_t mac_low = (uint32_t)g_i219v.mac[0] |
                       ((uint32_t)g_i219v.mac[1] << 8) |
                       ((uint32_t)g_i219v.mac[2] << 16) |
                       ((uint32_t)g_i219v.mac[3] << 24);
    uint32_t mac_high = (uint32_t)g_i219v.mac[4] |
                        ((uint32_t)g_i219v.mac[5] << 8);
    mmio_write32(g_i219v.mmio, REG_RAL(0), mac_low);
    mmio_write32(g_i219v.mmio, REG_RAH(0), mac_high | RAH_AV);

    mmio_write32(g_i219v.mmio, REG_RDT, (uint32_t)(I219V_DESC_COUNT - 1u));
    mmio_write32(g_i219v.mmio, REG_RDT_S, (uint32_t)(I219V_DESC_COUNT - 1u));

    mmio_write32(g_i219v.mmio, REG_TCTRL, TCTRL_EN | TCTRL_PSP |
                 (0x0Fu << TCTRL_CT_SHIFT) | TCTRL_COLD_HD);

    mmio_write32(g_i219v.mmio, REG_IMS, ICR_RXT0 | ICR_RXDMT0 | ICR_LSC | ICR_TXQE);

    g_i219v.mtu = I219V_DEFAULT_MTU;
    g_i219v.ready = 1;

    spinlock_unlock(&g_i219v.lock);
    irq_restore(irq_flags);

    return true;
}

bool i219v_is_ready(void)
{
    return g_i219v.ready != 0;
}

uint16_t i219v_mtu(void)
{
    return g_i219v.mtu;
}

void i219v_get_mac(uint8_t mac_out[6])
{
    if (mac_out == NULL) {
        return;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_i219v.lock);

    for (uint8_t i = 0; i < 6u; ++i) {
        mac_out[i] = g_i219v.mac[i];
    }

    spinlock_unlock(&g_i219v.lock);
    irq_restore(irq_flags);
}

bool i219v_send(const uint8_t *frame, uint16_t frame_len)
{
    if (!g_i219v.ready || frame == NULL || frame_len == 0 ||
        frame_len > g_i219v.mtu + 14u) {
        return false;
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_i219v.lock);

    i219v_reap_tx();

    uint16_t idx = g_i219v.tx_ring.next_to_use;
    if (g_i219v.tx_ring.in_use[idx]) {
        spinlock_unlock(&g_i219v.lock);
        irq_restore(irq_flags);
        return false;
    }

    memcpy(g_i219v.tx_ring.buffers[idx].virt, frame, (size_t)frame_len);

    volatile e1000_tx_desc_t *desc = &g_i219v.tx_ring.desc[idx];
    desc->addr  = g_i219v.tx_ring.buffers[idx].phys;
    desc->length = frame_len;
    desc->cmd    = TXD_CMD_EOP | TXD_CMD_IFCS | TXD_CMD_RS;
    desc->status = 0;

    memory_barrier();

    g_i219v.tx_ring.in_use[idx] = true;
    g_i219v.tx_ring.next_to_use =
        (uint16_t)((idx + 1u) % g_i219v.tx_ring.count);

    memory_barrier();
    mmio_write32(g_i219v.mmio, REG_TDT, (uint32_t)g_i219v.tx_ring.next_to_use);

    spinlock_unlock(&g_i219v.lock);
    irq_restore(irq_flags);
    return true;
}

void i219v_poll(void)
{
    if (g_i219v.ready == 0) {
        return;
    }

    bool processed = false;
    uint16_t cleaned = g_i219v.rx_ring.next_to_clean;

    for (uint16_t i = 0; i < g_i219v.rx_ring.count; ++i) {
        volatile e1000_rx_desc_t *desc = &g_i219v.rx_ring.desc[cleaned];
        clflush((volatile const void *)desc);
        if (!(desc->status & RXD_STAT_DD))
            break;

        if ((desc->status & RXD_STAT_EOP) && desc->length > 0 &&
            desc->length <= I219V_BUF_SIZE) {
            clflush_range(g_i219v.rx_ring.buffers[cleaned].virt, desc->length);
            if (g_i219v.rx_callback)
                g_i219v.rx_callback(g_i219v.rx_ring.buffers[cleaned].virt, desc->length);
        }

        desc->status = 0;
        cleaned = (uint16_t)((cleaned + 1u) % g_i219v.rx_ring.count);
        processed = true;
    }

    if (processed) {
        memory_barrier();
        g_i219v.rx_ring.next_to_clean = cleaned;
        uint16_t rdt = (cleaned == 0u) ?
            (uint16_t)(g_i219v.rx_ring.count - 1u) :
            (uint16_t)(cleaned - 1u);
        mmio_write32(g_i219v.mmio, REG_RDT, (uint32_t)rdt);
        mmio_write32(g_i219v.mmio, REG_RDT_S, (uint32_t)rdt);
    }

    i219v_reap_tx();
}

void i219v_set_rx_callback(i219v_rx_callback_t cb)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_i219v.lock);

    g_i219v.rx_callback = cb;

    spinlock_unlock(&g_i219v.lock);
    irq_restore(irq_flags);
}

#ifdef IMPLUS_DRIVER_MODULE
static const driver_nic_t g_i219v_driver = {
    .init = i219v_init,
    .is_ready = i219v_is_ready,
    .mtu = i219v_mtu,
    .get_mac = i219v_get_mac,
    .send_frame = i219v_send,
    .poll = i219v_poll,
    .set_rx_callback = i219v_set_rx_callback,
};

static void i219v_shutdown(void)
{
    mmio_write32(g_i219v.mmio, REG_IMC, 0xFFFFFFFFu);
    mmio_write32(g_i219v.mmio, REG_RCTRL, 0);
    mmio_write32(g_i219v.mmio, REG_TCTRL, 0);
    g_i219v.ready = 0;
    g_api = NULL;
}

static const driver_module_descriptor_t g_i219v_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_NIC,
    .load_priority = 50u,
    .deps = { "PCI_Driver.ELF", NULL },
    .driver_api = &g_i219v_driver,
    .shutdown = i219v_shutdown,
};

#undef malloc
#undef free
#undef dma_alloc
#undef dma_free
#undef dma_alloc_ex
#undef memset
#undef memcpy
#undef map_mmio_range
#undef pci_read_config
#undef pci_write_config
#undef timer_msleep
#undef hal_cpu_save_interrupts
#undef hal_cpu_restore_interrupts
#undef hal_cpu_pause

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL ||
        api->malloc == NULL ||
        api->free == NULL ||
        api->dma_alloc == NULL ||
        api->memset == NULL ||
        api->memcpy == NULL ||
        api->mem.dma_alloc_ex == NULL ||
        api->hw.map_mmio_range == NULL ||
        api->pci_read_config == NULL ||
        api->pci_write_config == NULL ||
        api->hal.cpu_save_interrupts == NULL ||
        api->hal.cpu_restore_interrupts == NULL ||
        api->hal.cpu_pause == NULL ||
        api->pci.enable_msix == NULL ||
        api->pci.register_irq == NULL) {
        return NULL;
    }

    g_api = api;
    return &g_i219v_module;
}
#endif
