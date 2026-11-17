/*
 * IG9_Engine.c - the blitter command streamer (BCS) of a Gen9 GPU.
 *
 * SUBMISSION
 *
 * Gen9 is an execlists part: i915 forces execlists on GRAPHICS_VER >= 9 and
 * never drives these engines through the legacy ring buffer. So the primary
 * path here is the one i915 uses:
 *
 *   - one logical ring context (LRC) whose register-state page carries the
 *     ring's START/HEAD/TAIL/CTL in the MI_LOAD_REGISTER_IMM layout of
 *     intel_lrc_reg.h, legacy-32-bit addressing, PDPs pointing at a scratch
 *     page directory (so the context has a well-formed address space even
 *     though nothing here uses PPGTT addresses);
 *   - RING_MODE.GFX_RUN_LIST_ENABLE, HWS_PGA -> our status page;
 *   - submission = write the new TAIL into the context image, then the two
 *     ELSP elements (element 1 empty, element 0 our descriptor);
 *   - completion = the MI_FLUSH_DW post-sync seqno write (the breadcrumb
 *     i915 emits on xcs engines), followed by the ACTIVE_IDLE event in the
 *     context status buffer.
 *
 * If execlists bring-up or its self-tests fail, the engine is reset and the
 * legacy ring-buffer path (RING_START/CTL/TAIL MMIO) is tried instead, with
 * the same self-tests. If that fails too the engine is left reset and idle,
 * and the display stays on the CPU copy path.
 *
 * All blitter work is placed in a batch buffer started with
 * MI_BATCH_BUFFER_START_GEN8 *without* the PPGTT bit: a privileged GGTT
 * batch. XY_* blit commands carry no address-space selector of their own,
 * so this makes "the addresses in these commands are GGTT addresses"
 * explicit instead of depending on how the ring itself is mapped.
 *
 * Every request is synchronous (submit, wait for its breadcrumb, wait for
 * the engine to go idle). That keeps the context image, the ring and the
 * batch buffer single-owner, and it means a lite-restore never happens
 * (WaIdleLiteRestore does not apply).
 *
 * HANG HANDLING
 *
 * A breadcrumb that does not land within its deadline triggers an engine
 * reset (RESET_CTL ready-to-reset handshake, then GDRST.BLT, as in
 * intel_reset.c), the submission mode is restarted from scratch, and the
 * caller falls back to a CPU copy for that frame. Three hangs disable the
 * engine for the rest of the boot.
 */

#ifdef IMPLUS_DRIVER_MODULE

#include "IG9_Private.h"

#define IG9_RING_BYTES            (32u * 1024u)
#define IG9_BATCH_BYTES           (16u * 1024u)
#define IG9_FENCE_OFFSET          0x100u          /* qword-aligned seqno slot */
#define IG9_REQUEST_TIMEOUT_US    200000u         /* a 4K full-screen copy is ~2 ms */
#define IG9_IDLE_TIMEOUT_US       20000u
#define IG9_RESET_TIMEOUT_US      500000u
#define IG9_MAX_HANGS             3u
#define IG9_CTX_HW_ID             1u
/* (batch dwords - BB_END - pad) / 10 */
#define IG9_MAX_RECTS_PER_BATCH   ((IG9_BATCH_BYTES / 4u - 2u) / IG9_XY_SRC_COPY_BLT_DWORDS)

typedef struct {
    uint8_t *virt;
    uint64_t phys;
    uint32_t bytes;
    uint32_t ggtt;
    bool ggtt_mapped;
} ig9_bo_t;

typedef struct {
    ig9_submit_mode_t mode;
    bool ready;
    bool buffers;
    bool fw_held;
    ig9_bo_t ring;
    ig9_bo_t batch;
    ig9_bo_t hwsp;      /* engine hardware status page */
    ig9_bo_t ctx;       /* PPHWSP + register state */
    ig9_bo_t pd;        /* scratch page directory (physical) */
    ig9_bo_t pt;        /* scratch page table (physical) */
    ig9_bo_t scratch;   /* page the scratch PTEs point at */
    ig9_bo_t fence;     /* breadcrumb page */
    ig9_bo_t test;      /* 2 pages: self-test dst (0) and src (1) */
    uint32_t tail;
    uint32_t seqno;
    uint32_t csb_head;
    uint64_t desc;
    uint32_t hangs;
    bool idle_event_missing_logged;
    ig9_spinlock_t lock;
} ig9_engine_t;

static ig9_engine_t s_eng;

#define BCS IG9_BCS_BASE

/* ------------------------------------------------------------------ *
 * Buffer objects
 * ------------------------------------------------------------------ */

static bool ig9_bo_alloc(ig9_bo_t *bo, uint32_t bytes, bool map_ggtt)
{
    const driver_binary_t *api = g_ig9_api;
    uint64_t phys = 0u;
    void *virt = NULL;

    ig9_memset(bo, 0, sizeof(*bo));
    if (api->mem.dma_alloc_ex != NULL) {
        /* PTEs/PDPs hold 39-bit physical addresses on Gen9. */
        virt = api->mem.dma_alloc_ex(bytes, IG9_PAGE_BYTES,
                                     (1ull << 39) - 1u, &phys);
    }
    if (virt == NULL) {
        virt = api->dma_alloc(bytes, &phys);
    }
    if (virt == NULL || phys == 0u) {
        return false;
    }
    if ((phys & (IG9_PAGE_BYTES - 1u)) != 0u ||
        ((uintptr_t)virt & (IG9_PAGE_BYTES - 1u)) != 0u ||
        (phys & ~IG9_GGTT_PTE_ADDR_MASK) != 0u) {
        api->dma_free(virt, bytes);
        return false;
    }

    bo->virt = (uint8_t *)virt;
    bo->phys = phys;
    bo->bytes = bytes;
    ig9_memset(bo->virt, 0, bytes);
    ig9_clflush_range(bo->virt, bytes);

    if (map_ggtt) {
        if (!ig9_ggtt_map(bo->virt, bo->phys, bytes, &bo->ggtt)) {
            api->dma_free(virt, bytes);
            ig9_memset(bo, 0, sizeof(*bo));
            return false;
        }
        bo->ggtt_mapped = true;
    }
    return true;
}

static void ig9_bo_free(ig9_bo_t *bo)
{
    if (bo->virt == NULL) {
        return;
    }
    if (bo->ggtt_mapped && !ig9_ggtt_unmap(bo->ggtt)) {
        /* The GPU might still reach these pages through a PTE we could not
         * clear: leak them rather than hand them back to the allocator. */
        ig9_memset(bo, 0, sizeof(*bo));
        return;
    }
    g_ig9_api->dma_free(bo->virt, bo->bytes);
    ig9_memset(bo, 0, sizeof(*bo));
}

static void ig9_buffers_free(void)
{
    ig9_bo_free(&s_eng.test);
    ig9_bo_free(&s_eng.fence);
    ig9_bo_free(&s_eng.scratch);
    ig9_bo_free(&s_eng.pt);
    ig9_bo_free(&s_eng.pd);
    ig9_bo_free(&s_eng.ctx);
    ig9_bo_free(&s_eng.hwsp);
    ig9_bo_free(&s_eng.batch);
    ig9_bo_free(&s_eng.ring);
    s_eng.buffers = false;
}

static bool ig9_buffers_alloc(void)
{
    if (s_eng.buffers) {
        return true;
    }
    if (!ig9_bo_alloc(&s_eng.ring, IG9_RING_BYTES, true) ||
        !ig9_bo_alloc(&s_eng.batch, IG9_BATCH_BYTES, true) ||
        !ig9_bo_alloc(&s_eng.hwsp, IG9_PAGE_BYTES, true) ||
        !ig9_bo_alloc(&s_eng.ctx, IG9_CTX_OBJECT_PAGES * IG9_PAGE_BYTES, true) ||
        !ig9_bo_alloc(&s_eng.pd, IG9_PAGE_BYTES, false) ||
        !ig9_bo_alloc(&s_eng.pt, IG9_PAGE_BYTES, false) ||
        !ig9_bo_alloc(&s_eng.scratch, IG9_PAGE_BYTES, false) ||
        !ig9_bo_alloc(&s_eng.fence, IG9_PAGE_BYTES, true) ||
        !ig9_bo_alloc(&s_eng.test, 2u * IG9_PAGE_BYTES, true)) {
        ig9_buffers_free();
        return false;
    }

    /* Scratch PPGTT: every PTE -> scratch page, every PDE -> that PT.
     * gen8_pte_encode / gen8_pde_encode: addr | PRESENT | RW (PAT index 0). */
    uint64_t *pt = (uint64_t *)(void *)s_eng.pt.virt;
    uint64_t *pd = (uint64_t *)(void *)s_eng.pd.virt;
    for (uint32_t i = 0; i < IG9_PAGE_BYTES / 8u; ++i) {
        pt[i] = s_eng.scratch.phys | IG9_PPGTT_PRESENT | IG9_PPGTT_RW;
        pd[i] = s_eng.pt.phys | IG9_PPGTT_PRESENT | IG9_PPGTT_RW;
    }
    ig9_clflush_range(pt, IG9_PAGE_BYTES);
    ig9_clflush_range(pd, IG9_PAGE_BYTES);

    s_eng.buffers = true;
    return true;
}

static volatile uint32_t *ig9_fence_slot(void)
{
    return (volatile uint32_t *)(void *)(s_eng.fence.virt + IG9_FENCE_OFFSET);
}

/* ------------------------------------------------------------------ *
 * Reset
 * ------------------------------------------------------------------ */

static bool ig9_engine_reset(void)
{
    /* i915 holds every forcewake domain across a reset. */
    bool render = ig9_forcewake_get(IG9_FW_RENDER);

    ig9_write32(IG9_RING_RESET_CTL(BCS),
                IG9_MASKED_ENABLE(IG9_RESET_CTL_REQUEST_RESET));
    /* On a timeout, do the same as i915 on a retry: reset anyway, the
     * engine is stuck. */
    (void)ig9_wait_reg(IG9_RING_RESET_CTL(BCS), IG9_RESET_CTL_READY_TO_RESET,
                       IG9_RESET_CTL_READY_TO_RESET, 700u);

    ig9_write32(IG9_GEN6_GDRST, IG9_GDRST_BLT);
    bool done = ig9_wait_reg(IG9_GEN6_GDRST, IG9_GDRST_BLT, 0u,
                             IG9_RESET_TIMEOUT_US);

    ig9_write32(IG9_RING_RESET_CTL(BCS),
                IG9_MASKED_DISABLE(IG9_RESET_CTL_REQUEST_RESET));
    if (render) {
        ig9_forcewake_put(IG9_FW_RENDER);
    }

    return done;
}

/* ------------------------------------------------------------------ *
 * Execlists
 * ------------------------------------------------------------------ */

static uint32_t *ig9_ctx_regs(void)
{
    return (uint32_t *)(void *)(s_eng.ctx.virt +
                                IG9_CTX_STATE_PAGE * IG9_PAGE_BYTES);
}

static void ig9_ctx_reg(uint32_t *regs, uint32_t pos, uint32_t reg,
                        uint32_t value)
{
    regs[pos] = reg;
    regs[pos + 1u] = value;
}

/* execlists_init_reg_state() for a Gen9 non-render engine. */
static void ig9_ctx_build(void)
{
    ig9_memset(s_eng.ctx.virt, 0, s_eng.ctx.bytes);
    uint32_t *regs = ig9_ctx_regs();

    regs[IG9_CTX_LRI_HEADER_0] = IG9_MI_LOAD_REGISTER_IMM(11u) |
                                 IG9_MI_LRI_FORCE_POSTED;
    ig9_ctx_reg(regs, IG9_CTX_CONTEXT_CONTROL, IG9_RING_CONTEXT_CONTROL(BCS),
                IG9_MASKED_ENABLE(IG9_CTX_CTRL_INHIBIT_SYN_SWITCH |
                                  IG9_CTX_CTRL_RESTORE_INHIBIT) |
                IG9_MASKED_DISABLE(IG9_CTX_CTRL_SAVE_INHIBIT));
    ig9_ctx_reg(regs, IG9_CTX_RING_HEAD, IG9_RING_HEAD(BCS), 0u);
    ig9_ctx_reg(regs, IG9_CTX_RING_TAIL, IG9_RING_TAIL(BCS), 0u);
    ig9_ctx_reg(regs, IG9_CTX_RING_START, IG9_RING_START(BCS), s_eng.ring.ggtt);
    ig9_ctx_reg(regs, IG9_CTX_RING_CTL, IG9_RING_CTL(BCS),
                IG9_RING_CTL_SIZE(IG9_RING_BYTES) | IG9_RING_CTL_VALID);
    ig9_ctx_reg(regs, IG9_CTX_BB_HEAD_U, IG9_RING_BBADDR_UDW(BCS), 0u);
    ig9_ctx_reg(regs, IG9_CTX_BB_HEAD_L, IG9_RING_BBADDR(BCS), 0u);
    ig9_ctx_reg(regs, IG9_CTX_BB_STATE, IG9_RING_BBSTATE(BCS), 1u << 5);
    ig9_ctx_reg(regs, IG9_CTX_SECOND_BB_HEAD_U, IG9_RING_SBBADDR_UDW(BCS), 0u);
    ig9_ctx_reg(regs, IG9_CTX_SECOND_BB_HEAD_L, IG9_RING_SBBADDR(BCS), 0u);
    ig9_ctx_reg(regs, IG9_CTX_SECOND_BB_STATE, IG9_RING_SBBSTATE(BCS), 0u);

    regs[IG9_CTX_LRI_HEADER_1] = IG9_MI_LOAD_REGISTER_IMM(9u) |
                                 IG9_MI_LRI_FORCE_POSTED;
    ig9_ctx_reg(regs, IG9_CTX_TIMESTAMP, IG9_RING_CTX_TIMESTAMP(BCS), 0u);
    uint32_t pd_hi = (uint32_t)(s_eng.pd.phys >> 32);
    uint32_t pd_lo = (uint32_t)s_eng.pd.phys;
    ig9_ctx_reg(regs, IG9_CTX_PDP3_UDW, IG9_RING_PDP_UDW(BCS, 3u), pd_hi);
    ig9_ctx_reg(regs, IG9_CTX_PDP3_LDW, IG9_RING_PDP_LDW(BCS, 3u), pd_lo);
    ig9_ctx_reg(regs, IG9_CTX_PDP2_UDW, IG9_RING_PDP_UDW(BCS, 2u), pd_hi);
    ig9_ctx_reg(regs, IG9_CTX_PDP2_LDW, IG9_RING_PDP_LDW(BCS, 2u), pd_lo);
    ig9_ctx_reg(regs, IG9_CTX_PDP1_UDW, IG9_RING_PDP_UDW(BCS, 1u), pd_hi);
    ig9_ctx_reg(regs, IG9_CTX_PDP1_LDW, IG9_RING_PDP_LDW(BCS, 1u), pd_lo);
    ig9_ctx_reg(regs, IG9_CTX_PDP0_UDW, IG9_RING_PDP_UDW(BCS, 0u), pd_hi);
    ig9_ctx_reg(regs, IG9_CTX_PDP0_LDW, IG9_RING_PDP_LDW(BCS, 0u), pd_lo);
    regs[IG9_CTX_END] = IG9_MI_BATCH_BUFFER_END;

    ig9_clflush_range(s_eng.ctx.virt, s_eng.ctx.bytes);

    s_eng.desc = IG9_CTX_VALID | IG9_CTX_PRIVILEGE |
                 (IG9_CTX_LEGACY_32B << IG9_CTX_ADDRESSING_SHIFT) |
                 (uint64_t)s_eng.ctx.ggtt |
                 ((uint64_t)IG9_CTX_HW_ID << IG9_CTX_ID_SHIFT);
}

static void ig9_csb_reset_head(void)
{
    uint32_t wp = IG9_CSB_WRITE_PTR(ig9_read32(IG9_RING_CSB_PTR(BCS)));
    /* After reset the write pointer reads 7 ("nothing yet"); the first
     * event will land in entry 0. */
    s_eng.csb_head = (wp < IG9_CSB_ENTRIES) ? wp : IG9_CSB_ENTRIES - 1u;
}

/* Consume pending context-status events; returns the OR of their status. */
static uint32_t ig9_csb_process(void)
{
    uint32_t wp = IG9_CSB_WRITE_PTR(ig9_read32(IG9_RING_CSB_PTR(BCS)));
    if (wp >= IG9_CSB_ENTRIES) {
        return 0u;
    }
    uint32_t seen = 0u;
    uint32_t head = s_eng.csb_head;
    while (head != wp) {
        head = (head + 1u) % IG9_CSB_ENTRIES;
        seen |= ig9_read32(IG9_RING_CSB_BUF_LO(BCS, head));
    }
    if (head != s_eng.csb_head) {
        s_eng.csb_head = head;
        ig9_write32(IG9_RING_CSB_PTR(BCS),
                    IG9_MASKED_FIELD(IG9_CSB_PTR_MASK << 8, head << 8));
    }
    return seen;
}

static bool ig9_execlists_enable(void)
{
    ig9_ctx_build();

    ig9_write32(IG9_RING_HWSTAM(BCS), 0xFFFFFFFFu);
    ig9_write32(IG9_RING_MODE(BCS),
                IG9_MASKED_FIELD(IG9_GFX_REPLAY_MODE | IG9_GFX_RUN_LIST_ENABLE,
                                 IG9_GFX_RUN_LIST_ENABLE));
    ig9_write32(IG9_RING_MI_MODE(BCS), IG9_MASKED_DISABLE(IG9_MI_MODE_STOP_RING));
    ig9_write32(IG9_RING_HWS_PGA(BCS), s_eng.hwsp.ggtt);
    (void)ig9_read32(IG9_RING_HWS_PGA(BCS));

    if ((ig9_read32(IG9_RING_MODE(BCS)) & IG9_GFX_RUN_LIST_ENABLE) == 0u) {
        return false;
    }
    ig9_csb_reset_head();
    return true;
}

static void ig9_execlists_submit(void)
{
    uint32_t *regs = ig9_ctx_regs();

    /* The engine saved HEAD into this cache line when the context went
     * idle; drop any stale CPU copy before touching TAIL next to it. */
    ig9_clflush_range(&regs[IG9_CTX_RING_HEAD], 16u);
    regs[IG9_CTX_RING_TAIL + 1u] = s_eng.tail;
    ig9_clflush_range(&regs[IG9_CTX_RING_TAIL + 1u], 4u);

    (void)ig9_csb_process();

    /* ELSP: element 1 (unused) then element 0, upper dword first; the
     * final write triggers the submission. */
    ig9_write32(IG9_RING_ELSP(BCS), 0u);
    ig9_write32(IG9_RING_ELSP(BCS), 0u);
    ig9_write32(IG9_RING_ELSP(BCS), (uint32_t)(s_eng.desc >> 32));
    ig9_write32(IG9_RING_ELSP(BCS), (uint32_t)s_eng.desc);
}

/* ------------------------------------------------------------------ *
 * Legacy ring buffer
 * ------------------------------------------------------------------ */

static bool ig9_legacy_enable(void)
{
    ig9_write32(IG9_RING_MODE(BCS),
                IG9_MASKED_DISABLE(IG9_GFX_RUN_LIST_ENABLE |
                                   IG9_GFX_PPGTT_ENABLE));
    ig9_write32(IG9_RING_HWSTAM(BCS), 0xFFFFFFFFu);
    ig9_write32(IG9_RING_HWS_PGA(BCS), s_eng.hwsp.ggtt);
    (void)ig9_read32(IG9_RING_HWS_PGA(BCS));

    ig9_write32(IG9_RING_CTL(BCS), 0u);
    ig9_write32(IG9_RING_HEAD(BCS), 0u);
    ig9_write32(IG9_RING_TAIL(BCS), 0u);
    ig9_write32(IG9_RING_START(BCS), s_eng.ring.ggtt);
    (void)ig9_read32(IG9_RING_START(BCS));
    ig9_write32(IG9_RING_CTL(BCS),
                IG9_RING_CTL_SIZE(IG9_RING_BYTES) | IG9_RING_CTL_VALID);

    if (!ig9_wait_reg(IG9_RING_CTL(BCS), IG9_RING_CTL_VALID,
                      IG9_RING_CTL_VALID, 50000u) ||
        ig9_read32(IG9_RING_START(BCS)) != s_eng.ring.ggtt ||
        (ig9_read32(IG9_RING_HEAD(BCS)) & IG9_RING_HEAD_ADDR_MASK) != 0u) {
        return false;
    }
    ig9_write32(IG9_RING_MI_MODE(BCS), IG9_MASKED_DISABLE(IG9_MI_MODE_STOP_RING));

    if ((ig9_read32(IG9_RING_MODE(BCS)) &
         (IG9_GFX_RUN_LIST_ENABLE | IG9_GFX_PPGTT_ENABLE)) != 0u) {
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ *
 * Common submission
 * ------------------------------------------------------------------ */

static bool ig9_mode_start(ig9_submit_mode_t mode)
{
    s_eng.mode = mode;
    s_eng.tail = 0u;
    ig9_memset(s_eng.ring.virt, 0, s_eng.ring.bytes);
    ig9_clflush_range(s_eng.ring.virt, s_eng.ring.bytes);
    ig9_memset(s_eng.hwsp.virt, 0, s_eng.hwsp.bytes);
    ig9_clflush_range(s_eng.hwsp.virt, s_eng.hwsp.bytes);

    bool ok = (mode == IG9_SUBMIT_EXECLISTS) ? ig9_execlists_enable()
                                             : ig9_legacy_enable();
    if (!ok) {
        s_eng.mode = IG9_SUBMIT_NONE;
    }
    return ok;
}

/* Append to the ring; packets never straddle the end of the ring (the
 * remainder is padded with MI_NOOP, as intel_ring_begin() does). */
static bool ig9_ring_emit(const uint32_t *cmds, uint32_t count)
{
    uint32_t padded = (count + 1u) & ~1u;
    uint32_t bytes = padded * 4u;
    uint32_t *ring = (uint32_t *)(void *)s_eng.ring.virt;

    if (bytes == 0u || bytes > IG9_RING_BYTES / 2u) {
        return false;
    }
    if (s_eng.tail + bytes > IG9_RING_BYTES) {
        uint32_t start = s_eng.tail;
        for (uint32_t off = start; off < IG9_RING_BYTES; off += 4u) {
            ring[off / 4u] = IG9_MI_NOOP;
        }
        ig9_clflush_range(s_eng.ring.virt + start, IG9_RING_BYTES - start);
        s_eng.tail = 0u;
    }

    uint32_t start = s_eng.tail;
    for (uint32_t i = 0; i < padded; ++i) {
        ring[start / 4u + i] = (i < count) ? cmds[i] : IG9_MI_NOOP;
    }
    ig9_clflush_range(s_eng.ring.virt + start, bytes);
    s_eng.tail = (start + bytes) % IG9_RING_BYTES;
    return true;
}

static void ig9_ring_kick(void)
{
    ig9_mb();
    if (s_eng.mode == IG9_SUBMIT_EXECLISTS) {
        ig9_execlists_submit();
    } else {
        ig9_write32(IG9_RING_TAIL(BCS), s_eng.tail & IG9_RING_TAIL_ADDR_MASK);
        (void)ig9_read32(IG9_RING_TAIL(BCS));
    }
}

/* Run the batch already written to s_eng.batch (`dwords` long, ending in
 * MI_BATCH_BUFFER_END) and wait for it. */
static bool ig9_run_batch(uint32_t dwords, uint32_t timeout_us)
{
    uint32_t *batch = (uint32_t *)(void *)s_eng.batch.virt;
    if ((dwords & 1u) != 0u) {
        batch[dwords++] = IG9_MI_NOOP;
    }
    ig9_clflush_range(batch, (uint64_t)dwords * 4u);

    uint32_t seqno = s_eng.seqno + 1u;
    if (seqno == 0u) {
        seqno = 1u;
    }
    s_eng.seqno = seqno;

    volatile uint32_t *slot = ig9_fence_slot();
    *slot = ~seqno;
    ig9_clflush_range((const void *)slot, 4u);

    /* gen8_emit_bb_start (secure: GGTT) + gen8_emit_fini_breadcrumb (xcs). */
    uint32_t cmds[10];
    uint32_t n = 0u;
    cmds[n++] = IG9_MI_BATCH_BUFFER_START_GEN8;
    cmds[n++] = s_eng.batch.ggtt;
    cmds[n++] = 0u;
    cmds[n++] = IG9_MI_NOOP;
    cmds[n++] = (IG9_MI_FLUSH_DW + 1u) | IG9_MI_FLUSH_DW_OP_STOREDW;
    cmds[n++] = (s_eng.fence.ggtt + IG9_FENCE_OFFSET) | IG9_MI_FLUSH_DW_USE_GTT;
    cmds[n++] = 0u;
    cmds[n++] = seqno;
    cmds[n++] = IG9_MI_USER_INTERRUPT;
    cmds[n++] = IG9_MI_NOOP;

    if (!ig9_ring_emit(cmds, n)) {
        return false;
    }
    ig9_ring_kick();

    ig9_deadline_t d;
    ig9_deadline_start(&d, timeout_us);
    for (;;) {
        ig9_clflush_range((const void *)slot, 4u);
        if (*slot == seqno) {
            break;
        }
        if (ig9_deadline_expired(&d)) {
            ig9_clflush_range((const void *)slot, 4u);
            if (*slot == seqno) {
                break;
            }
            return false;
        }
    }

    /* The breadcrumb is the last command; wait for the engine to retire
     * the context (or drain the ring) so the next request starts from an
     * idle engine and the batch buffer may be rewritten. */
    if (s_eng.mode == IG9_SUBMIT_EXECLISTS) {
        ig9_deadline_start(&d, IG9_IDLE_TIMEOUT_US);
        for (;;) {
            if ((ig9_csb_process() & IG9_CSB_STATUS_ACTIVE_IDLE) != 0u) {
                break;
            }
            if (ig9_deadline_expired(&d)) {
                if (!s_eng.idle_event_missing_logged) {
                    s_eng.idle_event_missing_logged = true;
                }
                break;
            }
        }
    } else if (!ig9_wait_reg(IG9_RING_HEAD(BCS), IG9_RING_HEAD_ADDR_MASK,
                             s_eng.tail & IG9_RING_HEAD_ADDR_MASK,
                             IG9_IDLE_TIMEOUT_US)) {
    }
    return true;
}

/* ------------------------------------------------------------------ *
 * Blit encoders (Gen8+ layouts)
 * ------------------------------------------------------------------ */

static uint32_t ig9_emit_color_blt(uint32_t *b, uint32_t dst_ggtt,
                                   uint32_t pitch, uint32_t x, uint32_t y,
                                   uint32_t w, uint32_t h, uint32_t color)
{
    b[0] = IG9_XY_COLOR_BLT_CMD | IG9_BLT_WRITE_ALPHA | IG9_BLT_WRITE_RGB |
           IG9_BLT_LEN(IG9_XY_COLOR_BLT_DWORDS);
    b[1] = IG9_BLT_DEPTH_32 | IG9_BLT_ROP_PAT_COPY | (pitch & 0xFFFFu);
    b[2] = (y << 16) | x;
    b[3] = ((y + h) << 16) | (x + w);
    b[4] = dst_ggtt;
    b[5] = 0u;
    b[6] = color;
    return IG9_XY_COLOR_BLT_DWORDS;
}

static uint32_t ig9_emit_copy_blt(uint32_t *b, const ig9_surface_t *dst,
                                  const ig9_surface_t *src, uint32_t x,
                                  uint32_t y, uint32_t w, uint32_t h)
{
    b[0] = IG9_XY_SRC_COPY_BLT_CMD | IG9_BLT_WRITE_ALPHA | IG9_BLT_WRITE_RGB |
           IG9_BLT_LEN(IG9_XY_SRC_COPY_BLT_DWORDS);
    b[1] = IG9_BLT_DEPTH_32 | IG9_BLT_ROP_SRC_COPY | (dst->pitch & 0xFFFFu);
    b[2] = (y << 16) | x;
    b[3] = ((y + h) << 16) | (x + w);
    b[4] = dst->ggtt;
    b[5] = 0u;
    b[6] = (y << 16) | x;
    b[7] = src->pitch & 0xFFFFu;
    b[8] = src->ggtt;
    b[9] = 0u;
    return IG9_XY_SRC_COPY_BLT_DWORDS;
}

/* ------------------------------------------------------------------ *
 * Self-tests (on driver-owned pages only)
 * ------------------------------------------------------------------ */

#define IG9_TEST_PITCH 256u     /* 64 pixels per row, 16 rows per page */

static bool ig9_selftest_store(void)
{
    volatile uint32_t *dst = (volatile uint32_t *)(void *)s_eng.test.virt;
    const uint32_t marker = 0xC0FFEE01u;
    dst[0] = 0u;
    ig9_clflush_range(s_eng.test.virt, 64u);

    uint32_t *b = (uint32_t *)(void *)s_eng.batch.virt;
    uint32_t n = 0u;
    b[n++] = IG9_MI_STORE_DWORD_IMM_GEN4 | IG9_MI_USE_GGTT;
    b[n++] = s_eng.test.ggtt;
    b[n++] = 0u;
    b[n++] = marker;
    b[n++] = IG9_MI_BATCH_BUFFER_END;
    if (!ig9_run_batch(n, IG9_REQUEST_TIMEOUT_US)) {
        return false;
    }
    ig9_clflush_range(s_eng.test.virt, 64u);
    if (dst[0] != marker) {
        return false;
    }
    return true;
}

static bool ig9_selftest_fill(void)
{
    volatile uint32_t *px = (volatile uint32_t *)(void *)s_eng.test.virt;
    const uint32_t color = 0xA5C3E1F0u;
    const uint32_t bg = 0x5A5A5A5Au;
    const uint32_t stride = IG9_TEST_PITCH / 4u;
    for (uint32_t i = 0; i < IG9_PAGE_BYTES / 4u; ++i) {
        px[i] = bg;
    }
    ig9_clflush_range(s_eng.test.virt, IG9_PAGE_BYTES);

    /* 20x5 rectangle at (7,2). */
    uint32_t *b = (uint32_t *)(void *)s_eng.batch.virt;
    uint32_t n = ig9_emit_color_blt(b, s_eng.test.ggtt, IG9_TEST_PITCH,
                                    7u, 2u, 20u, 5u, color);
    b[n++] = IG9_MI_BATCH_BUFFER_END;
    if (!ig9_run_batch(n, IG9_REQUEST_TIMEOUT_US)) {
        return false;
    }
    ig9_clflush_range(s_eng.test.virt, IG9_PAGE_BYTES);
    for (uint32_t y = 0; y < 16u; ++y) {
        for (uint32_t x = 0; x < stride; ++x) {
            bool inside = x >= 7u && x < 27u && y >= 2u && y < 7u;
            uint32_t want = inside ? color : bg;
            if (px[y * stride + x] != want) {
                return false;
            }
        }
    }
    return true;
}

static bool ig9_selftest_copy(void)
{
    volatile uint32_t *dst = (volatile uint32_t *)(void *)s_eng.test.virt;
    volatile uint32_t *src =
        (volatile uint32_t *)(void *)(s_eng.test.virt + IG9_PAGE_BYTES);
    const uint32_t bg = 0x00DEAD00u;
    const uint32_t stride = IG9_TEST_PITCH / 4u;
    for (uint32_t i = 0; i < IG9_PAGE_BYTES / 4u; ++i) {
        dst[i] = bg;
        src[i] = 0x10000000u + i * 0x01010101u;
    }
    ig9_clflush_range(s_eng.test.virt, 2u * IG9_PAGE_BYTES);

    ig9_surface_t d = { s_eng.test.ggtt, IG9_TEST_PITCH };
    ig9_surface_t s = { s_eng.test.ggtt + IG9_PAGE_BYTES, IG9_TEST_PITCH };
    /* 21x3 rectangle at (3,9) - exactly what present uses. */
    uint32_t *b = (uint32_t *)(void *)s_eng.batch.virt;
    uint32_t n = ig9_emit_copy_blt(b, &d, &s, 3u, 9u, 21u, 3u);
    b[n++] = IG9_MI_BATCH_BUFFER_END;
    if (!ig9_run_batch(n, IG9_REQUEST_TIMEOUT_US)) {
        return false;
    }
    ig9_clflush_range(s_eng.test.virt, IG9_PAGE_BYTES);
    for (uint32_t y = 0; y < 16u; ++y) {
        for (uint32_t x = 0; x < stride; ++x) {
            uint32_t i = y * stride + x;
            bool inside = x >= 3u && x < 24u && y >= 9u && y < 12u;
            uint32_t want = inside ? (0x10000000u + i * 0x01010101u) : bg;
            if (dst[i] != want) {
                return false;
            }
        }
    }
    return true;
}

static bool ig9_selftests(void)
{
    if (!ig9_selftest_store()) {
        return false;
    }
    if (!ig9_selftest_fill()) {
        return false;
    }
    if (!ig9_selftest_copy()) {
        return false;
    }
    /* A second round exercises a resubmission of the same context (and,
     * for execlists, a context that has been saved once). */
    return ig9_selftest_copy();
}

static bool ig9_try_mode(ig9_submit_mode_t mode)
{
    if (!ig9_mode_start(mode)) {
        return false;
    }
    if (!ig9_selftests()) {
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ *
 * Public interface
 * ------------------------------------------------------------------ */

bool ig9_engine_ready(void)
{
    return s_eng.ready;
}

ig9_submit_mode_t ig9_engine_mode(void)
{
    return s_eng.ready ? s_eng.mode : IG9_SUBMIT_NONE;
}

bool ig9_engine_init(void)
{
    if (s_eng.ready) {
        return true;
    }
    if (!s_eng.fw_held) {
        if (!ig9_forcewake_get(IG9_FW_GT)) {
            return false;
        }
        s_eng.fw_held = true;
    }
    if (!ig9_buffers_alloc()) {
        ig9_forcewake_put(IG9_FW_GT);
        s_eng.fw_held = false;
        return false;
    }

    /* i915 sanitizes the engines at load (intel_gt_sanitize); start from a
     * freshly reset BCS whatever the firmware left behind. */
    bool reset_ok = ig9_engine_reset();
    if (reset_ok && ig9_try_mode(IG9_SUBMIT_EXECLISTS)) {
        s_eng.ready = true;
        return true;
    }
    reset_ok = ig9_engine_reset();
    if (reset_ok && ig9_try_mode(IG9_SUBMIT_LEGACY_RING)) {
        s_eng.ready = true;
        return true;
    }

    s_eng.mode = IG9_SUBMIT_NONE;
    if (ig9_engine_reset()) {
        /* Engine verifiably idle: nothing can reach our buffers any more. */
        ig9_buffers_free();
    }
    ig9_forcewake_put(IG9_FW_GT);
    s_eng.fw_held = false;
    return false;
}

static void ig9_engine_recover(void)
{
    ++s_eng.hangs;
    ig9_submit_mode_t mode = s_eng.mode;
    if (s_eng.hangs >= IG9_MAX_HANGS || !ig9_engine_reset() ||
        !ig9_mode_start(mode)) {
        s_eng.ready = false;
        s_eng.mode = IG9_SUBMIT_NONE;
    }
}

bool ig9_engine_copy_rects(const ig9_surface_t *dst, const ig9_surface_t *src,
                           const ig9_blit_rect_t *rects, uint32_t count)
{
    if (!s_eng.ready || dst == NULL || src == NULL || rects == NULL) {
        return false;
    }
    if (count == 0u) {
        return true;
    }

    ig9_spin_lock(&s_eng.lock);
    uint32_t done = 0u;
    bool ok = true;
    while (done < count) {
        uint32_t *b = (uint32_t *)(void *)s_eng.batch.virt;
        uint32_t n = 0u;
        uint32_t take = count - done;
        if (take > IG9_MAX_RECTS_PER_BATCH) {
            take = IG9_MAX_RECTS_PER_BATCH;
        }
        for (uint32_t i = 0; i < take; ++i) {
            const ig9_blit_rect_t *r = &rects[done + i];
            n += ig9_emit_copy_blt(&b[n], dst, src, r->x, r->y, r->w, r->h);
        }
        b[n++] = IG9_MI_BATCH_BUFFER_END;
        if (!ig9_run_batch(n, IG9_REQUEST_TIMEOUT_US)) {
            ig9_engine_recover();
            ok = false;
            break;
        }
        done += take;
    }
    ig9_spin_unlock(&s_eng.lock);
    return ok;
}

void ig9_engine_shutdown(void)
{
    ig9_spin_lock(&s_eng.lock);
    if (s_eng.ready) {
        s_eng.ready = false;
        (void)ig9_engine_reset();
        s_eng.mode = IG9_SUBMIT_NONE;
    }
    if (s_eng.fw_held) {
        ig9_forcewake_put(IG9_FW_GT);
        s_eng.fw_held = false;
    }
    ig9_spin_unlock(&s_eng.lock);
}

#endif /* IMPLUS_DRIVER_MODULE */
