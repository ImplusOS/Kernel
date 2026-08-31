#include "GDT_Main.h"
#include "smp/SMP_Main.h"
#include "Core/sync/Spinlock.h"
#include "Debug/serial/Serial.h"

#define GDT_MAX_CPUS OS_CONFIG_SMP_MAX_CPUS

typedef struct {
    struct GDTEntry  null;
    struct GDTEntry  kernel_code;
    struct GDTEntry  kernel_data;
    struct GDTEntry  user_compat_code;
    struct GDTEntry  user_data;
    struct GDTEntry  user_code;
    struct GDTEntry64 tss;
} __attribute__((packed)) GDT_Table;

static GDT_Table g_gdt[GDT_MAX_CPUS];
static struct GDTR g_gdtr[GDT_MAX_CPUS];
static struct TSS g_tss[GDT_MAX_CPUS];

static uint8_t kernel_stack[GDT_MAX_CPUS][64 * 1024];
static uint8_t df_ist_stack[GDT_MAX_CPUS][64 * 1024];
static uint8_t nmi_ist_stack[GDT_MAX_CPUS][64 * 1024];
extern void gdt_flush(uint64_t);
extern void tss_flush(uint16_t);

static inline uint32_t gdt_current_cpu_id(void)
{
    return smp_get_current_cpu_id();
}

static struct GDTEntry make_gdt_entry(uint32_t limit, uint8_t access, uint8_t flags) {
    struct GDTEntry e = {0};
    e.limit_low = limit & 0xFFFF;
    e.access = access;
    e.gran   = ((limit >> 16) & 0x0F) | (flags & 0xF0);
    return e;
}

void init_gdt(void) {
    uint32_t cpu_id = gdt_current_cpu_id();
    if (cpu_id >= GDT_MAX_CPUS) {
        cpu_id = GDT_MAX_CPUS - 1;
    }
    GDT_Table *gdt = &g_gdt[cpu_id];
    struct TSS *tss = &g_tss[cpu_id];

    gdt->null = make_gdt_entry(0, 0, 0);

    gdt->kernel_code = make_gdt_entry(0xFFFFF, 0x9A, 0xA0);
    gdt->kernel_data = make_gdt_entry(0xFFFFF, 0x92, 0x80);

    gdt->user_compat_code = make_gdt_entry(0xFFFFF, 0xFA, 0xC0);
    gdt->user_data = make_gdt_entry(0xFFFFF, 0xF2, 0x80);
    gdt->user_code = make_gdt_entry(0xFFFFF, 0xFA, 0xA0);

    uint64_t tss_base = (uint64_t)tss;
    uint32_t tss_limit = sizeof(struct TSS) - 1;

    gdt->tss.limit_low  = tss_limit & 0xFFFF;
    gdt->tss.base_low   = tss_base & 0xFFFF;
    gdt->tss.base_mid   = (tss_base >> 16) & 0xFF;
    gdt->tss.access     = 0x89;
    gdt->tss.gran       = (tss_limit >> 16) & 0x0F;
    gdt->tss.base_high  = (tss_base >> 24) & 0xFF;
    gdt->tss.base_upper = (tss_base >> 32);
    gdt->tss.reserved   = 0;

    tss->reserved0 = 0;
    tss->rsp0 = 0;
    tss->rsp1 = 0;
    tss->rsp2 = 0;
    tss->reserved1 = 0;
    tss->reserved2 = 0;
    tss->reserved3 = 0;
    for (int i = 0; i < 7; i++) {
        tss->ist[i] = 0;
    }
    tss->rsp0 = (uint64_t)(kernel_stack[cpu_id] + sizeof(kernel_stack[cpu_id]));
    uint64_t ist_flags = irq_save_disable();
    gdt_set_ist(1, (uint64_t)(uintptr_t)(df_ist_stack[cpu_id] + sizeof(df_ist_stack[cpu_id])));
    gdt_set_ist(2, (uint64_t)(uintptr_t)(nmi_ist_stack[cpu_id] + sizeof(nmi_ist_stack[cpu_id])));
    irq_restore(ist_flags);
    
    tss->io_map_base = sizeof(struct TSS);

    g_gdtr[cpu_id].limit = sizeof(GDT_Table) - 1;
    g_gdtr[cpu_id].base  = (uint64_t)gdt;

    gdt_flush((uint64_t)&g_gdtr[cpu_id]);
    tss_flush(GDT_TSS);
}

void gdt_set_kernel_rsp0(uint64_t rsp0)
{
    uint32_t cpu_id = gdt_current_cpu_id();
    if (cpu_id >= GDT_MAX_CPUS) {
        cpu_id = GDT_MAX_CPUS - 1;
    }
    g_tss[cpu_id].rsp0 = rsp0 & ~0xFULL;
}

void gdt_set_ist(uint8_t ist_index, uint64_t rsp)
{
    if (ist_index == 0 || ist_index > 7) {
        return;
    }
    uint32_t cpu_id = gdt_current_cpu_id();
    if (cpu_id >= GDT_MAX_CPUS) {
        cpu_id = GDT_MAX_CPUS - 1;
    }
    g_tss[cpu_id].ist[ist_index - 1] = rsp & ~0xFULL;
}
