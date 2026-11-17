/*
 * IG9_Private.h - state and helpers shared by the IntelGen9 driver module's
 * translation units. Not a public interface.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "Drivers/Module/DriverBinary.h"
#include "IG9_Regs.h"

/* ------------------------------------------------------------------ *
 * Device identity
 * ------------------------------------------------------------------ */

typedef enum {
    IG9_GEN_SKYLAKE,
    IG9_GEN_KABYLAKE,
    IG9_GEN_COFFEELAKE,
    IG9_GEN_COMETLAKE,
    IG9_GEN_AMBERLAKE,
    IG9_GEN_BROXTON,
    IG9_GEN_GEMINILAKE,
} ig9_gen_t;

typedef struct {
    uint16_t device_id;
    const char *name;
    ig9_gen_t gen;
} ig9_device_t;

typedef struct {
    uint8_t bus;
    uint8_t device;
    uint8_t function;
    uint8_t revision;
    uint16_t device_id;
    const char *name;
    ig9_gen_t gen;
    bool has_llc;   /* big-core parts share the LLC with the CPU; LP parts do not */
} ig9_match_t;

extern const driver_binary_t *g_ig9_api;
extern ig9_match_t g_ig9_match;
extern bool g_ig9_have_match;

/* ------------------------------------------------------------------ *
 * Utilities (IG9_Uncore.c)
 * ------------------------------------------------------------------ */

void *ig9_memset(void *dst, int value, size_t n);
void ig9_memcpy(void *dst, const void *src, size_t n);

uint64_t ig9_now_ns(void);          /* 0 when the kernel exposes no clock */
void ig9_msleep(uint32_t ms);
void ig9_udelay(uint32_t us);

/* Bounded waiting: spin briefly, then back off to 1 ms sleeps. Works with
 * or without a monotonic clock (without one, each poll costs 1 ms). */
typedef struct {
    uint64_t start_ns;
    uint64_t timeout_ns;
    uint32_t polls;
    bool has_clock;
} ig9_deadline_t;

void ig9_deadline_start(ig9_deadline_t *d, uint32_t timeout_us);
/* Returns true once the deadline has passed; otherwise backs off a bit. */
bool ig9_deadline_expired(ig9_deadline_t *d);

/* CPU cache maintenance for memory the GPU reads or writes. */
void ig9_clflush_range(const void *base, uint64_t bytes);
void ig9_mb(void);

typedef struct {
    volatile uint32_t lock;
} ig9_spinlock_t;

void ig9_spin_lock(ig9_spinlock_t *l);
void ig9_spin_unlock(ig9_spinlock_t *l);

/* ------------------------------------------------------------------ *
 * Uncore: BARs, MMIO, forcewake, PCI power (IG9_Uncore.c)
 * ------------------------------------------------------------------ */

typedef struct {
    uint64_t mmio_phys;
    uint64_t mmio_size;     /* BAR0 size; GGTT lives in its upper half */
    uint64_t gmadr_phys;    /* BAR2 aperture (0 if absent) */
    uint64_t gmadr_size;
} ig9_bars_t;

extern ig9_bars_t g_ig9_bars;

bool ig9_uncore_init(void);
bool ig9_uncore_ready(void);
uint32_t ig9_read32(uint32_t offset);
void ig9_write32(uint32_t offset, uint32_t value);
bool ig9_wait_reg(uint32_t offset, uint32_t mask, uint32_t value,
                  uint32_t timeout_us);

#define IG9_FW_GT     (1u << 0)
#define IG9_FW_RENDER (1u << 1)

bool ig9_forcewake_get(uint32_t domains);
void ig9_forcewake_put(uint32_t domains);

uint32_t ig9_pci_read(uint16_t offset);

/* ------------------------------------------------------------------ *
 * GGTT (IG9_Ggtt.c)
 * ------------------------------------------------------------------ */

bool ig9_ggtt_init(void);
uint32_t ig9_ggtt_entry_count(void);
uint64_t ig9_ggtt_read_pte(uint32_t index);
/* True when every page of [ggtt, ggtt+bytes) has a present PTE. */
bool ig9_ggtt_range_present(uint32_t ggtt, uint64_t bytes);
/* True when [ggtt, ggtt+bytes) maps physically contiguous pages starting
 * at phys (page granularity). */
bool ig9_ggtt_range_maps(uint32_t ggtt, uint64_t phys, uint64_t bytes);
/* Search the table for the entry that maps `phys` (read-only). */
bool ig9_ggtt_find_phys(uint64_t phys, uint32_t *index_out);
/* Map `bytes` of memory into a private GGTT range with guard pages on both
 * sides. `phys` (if non-zero) is the authoritative base of a physically
 * contiguous allocation; otherwise each page of `virt` is translated. */
bool ig9_ggtt_map(const void *virt, uint64_t phys, uint64_t bytes,
                  uint32_t *ggtt_out);
/* Unmap; the caller guarantees the GPU no longer references the range. */
bool ig9_ggtt_unmap(uint32_t ggtt);

/* ------------------------------------------------------------------ *
 * Blitter engine (IG9_Engine.c)
 * ------------------------------------------------------------------ */

typedef enum {
    IG9_SUBMIT_NONE = 0,
    IG9_SUBMIT_EXECLISTS,
    IG9_SUBMIT_LEGACY_RING,
} ig9_submit_mode_t;

typedef struct {
    uint32_t ggtt;      /* graphics address of pixel (0,0) */
    uint32_t pitch;     /* bytes per row */
} ig9_surface_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} ig9_blit_rect_t;

/* Bring the BCS up (execlists first, legacy ring as a fallback), run the
 * self-tests, and report whether the engine may be used. */
bool ig9_engine_init(void);
bool ig9_engine_ready(void);
ig9_submit_mode_t ig9_engine_mode(void);
/* Copy the same rectangles from src to dst (same x/y in both surfaces).
 * Synchronous: returns once the GPU has finished writing, or false after
 * a timeout/hang (in which case the engine has been reset, and may have
 * been disabled for the rest of the boot). */
bool ig9_engine_copy_rects(const ig9_surface_t *dst, const ig9_surface_t *src,
                           const ig9_blit_rect_t *rects, uint32_t count);
/* Stop using the engine (module shutdown). */
void ig9_engine_shutdown(void);
