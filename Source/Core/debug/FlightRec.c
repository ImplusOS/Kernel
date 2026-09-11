#include "Core/debug/FlightRec.h"

#if FLIGHT_REC

#include "Debug/serial/Serial.h"
#include "smp/SMP_Main.h"
#include "kernel/config.h"

/* One page of low memory, just past the SMP trampoline (0x8000) and its shared
 * hand-off block (0x9000). Reserved from the PMM in memory init. */
#define FR_PHYS   0xA000ULL
#define FR_MAGIC  0x52464D49u   /* "IMFR" */
#define FR_SLOTS  100u

typedef struct {
    uint32_t cpu;
    uint32_t tag;
    uint64_t a;
    uint64_t b;
} fr_entry_t;

typedef struct {
    uint32_t   magic;
    uint32_t   head;      /* next slot to write */
    uint32_t   wrapped;
    uint32_t   pad;
    fr_entry_t e[FR_SLOTS];
} fr_ring_t;

static volatile fr_ring_t *ring(void)
{
    return (volatile fr_ring_t *)(uintptr_t)FR_PHYS;
}

static const char *fr_tag_name(uint32_t tag)
{
    switch (tag) {
        case FR_TAG_SYSCALL: return "sys";
        case FR_TAG_PF:      return "pf";
        case FR_TAG_GP:      return "gp";
        case FR_TAG_SWITCH:  return "sw";
        case FR_TAG_EXIT:    return "exit";
        case FR_TAG_DESTROY: return "destroy";
        case FR_TAG_THREAD:  return "thread";
        default:             return "?";
    }
}

void flight_rec_init(void)
{
    volatile fr_ring_t *r = ring();

    if (r->magic == FR_MAGIC) {
        uint32_t head = r->head % FR_SLOTS;
        uint32_t count = r->wrapped ? FR_SLOTS : head;
        serial_write_string("[flightrec] previous boot left ");
        serial_write_uint32(count);
        serial_write_string(" entries, oldest first:\n");
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t idx = (head + FR_SLOTS - count + i) % FR_SLOTS;
            serial_write_string("[fr] cpu");
            serial_write_uint32(r->e[idx].cpu);
            serial_write_string(" ");
            serial_write_string(fr_tag_name(r->e[idx].tag));
            serial_write_string(" a=");
            serial_write_uint64(r->e[idx].a);
            serial_write_string(" b=");
            serial_write_uint64(r->e[idx].b);
            serial_write_char('\n');
        }
    }

    r->magic = FR_MAGIC;
    r->head = 0;
    r->wrapped = 0;
    for (uint32_t i = 0; i < FR_SLOTS; ++i) {
        r->e[i].cpu = 0; r->e[i].tag = 0; r->e[i].a = 0; r->e[i].b = 0;
    }
}

void flight_rec(uint32_t tag, uint64_t a, uint64_t b)
{
    volatile fr_ring_t *r = ring();
    if (r->magic != FR_MAGIC) {
        return;
    }
    uint32_t slot = __atomic_fetch_add(&r->head, 1u, __ATOMIC_RELAXED);
    if (slot >= FR_SLOTS) {
        r->wrapped = 1u;
    }
    slot %= FR_SLOTS;
    r->e[slot].cpu = smp_get_current_cpu_id();
    r->e[slot].tag = tag;
    r->e[slot].a = a;
    r->e[slot].b = b;
}

#endif /* FLIGHT_REC */
