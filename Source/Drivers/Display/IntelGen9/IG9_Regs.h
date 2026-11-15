/*
 * IG9_Regs.h - register offsets, bit fields and command encodings for the
 * Intel Gen9 / Gen9.5 (Skylake, Kaby Lake, Coffee Lake, Whiskey Lake,
 * Comet Lake, Broxton/Apollo Lake, Gemini Lake) integrated GPU.
 *
 * Every value here is cross-checked against the Linux i915 driver
 * (i915_reg.h, intel_engine_regs.h, intel_lrc_reg.h, intel_gpu_commands.h,
 * intel_uncore.c, intel_reset.c) and the Gen9 PRM. Nothing in this header
 * has side effects; it is shared with the host-side simulator test
 * (Tools/IntelGen9Sim), which decodes commands with the same constants.
 */
#pragma once

#include <stdint.h>

/* ------------------------------------------------------------------ *
 * Masked registers: bits [31:16] select which of bits [15:0] change.
 * ------------------------------------------------------------------ */
#define IG9_MASKED_ENABLE(bits)          ((uint32_t)(((bits) << 16) | (bits)))
#define IG9_MASKED_DISABLE(bits)         ((uint32_t)((bits) << 16))
#define IG9_MASKED_FIELD(mask, value)    ((uint32_t)(((mask) << 16) | (value)))

/* ------------------------------------------------------------------ *
 * PCI
 * ------------------------------------------------------------------ */
#define IG9_PCI_VENDOR_INTEL             0x8086u
#define IG9_PCI_CLASS_DISPLAY            0x03u
#define IG9_PCI_CLASS_BRIDGE             0x06u
#define IG9_PCI_SUBCLASS_HOST            0x00u
#define IG9_PCI_BAR_GTTMMADR             0u      /* 64-bit BAR0: MMIO + GGTT    */
#define IG9_PCI_BAR_GMADR                2u      /* 64-bit BAR2: CPU aperture   */
/* SNB_GMCH_CTRL (a.k.a. GGC). i915 reads it from the GPU function itself;
 * the host bridge (00:00.0) carries the same value at the same offset. */
#define IG9_PCI_GMCH_CTRL                0x50u
#define IG9_GMCH_GGMS_SHIFT              6u      /* BDW_GMCH_GGMS_SHIFT */
#define IG9_GMCH_GGMS_MASK               0x3u

/* ------------------------------------------------------------------ *
 * Forcewake (Gen9: intel_uncore.c fw_domain_init for GRAPHICS_VER 9)
 * ------------------------------------------------------------------ */
#define IG9_FORCEWAKE_GT                 0xA188u   /* FORCEWAKE_GT_GEN9       */
#define IG9_FORCEWAKE_ACK_GT             0x130044u /* FORCEWAKE_ACK_GT_GEN9   */
#define IG9_FORCEWAKE_RENDER             0xA278u   /* FORCEWAKE_RENDER_GEN9   */
#define IG9_FORCEWAKE_ACK_RENDER         0x0D84u   /* FORCEWAKE_ACK_RENDER_GEN9 */
#define IG9_FORCEWAKE_KERNEL             (1u << 0)
#define IG9_FORCEWAKE_KERNEL_FALLBACK    (1u << 15)

/* ------------------------------------------------------------------ *
 * Reset (intel_reset.c gen8_engine_reset_prepare / gen6_hw_domain_reset)
 * ------------------------------------------------------------------ */
#define IG9_GEN6_GDRST                   0x941Cu
#define IG9_GDRST_FULL                   (1u << 0)
#define IG9_GDRST_RENDER                 (1u << 1)
#define IG9_GDRST_MEDIA                  (1u << 2)
#define IG9_GDRST_BLT                    (1u << 3)
#define IG9_RESET_CTL_REQUEST_RESET      (1u << 0)
#define IG9_RESET_CTL_READY_TO_RESET     (1u << 1)

/* GGTT invalidate (gen8_ggtt_invalidate -> GFX_FLSH_CNTL_GEN6). */
#define IG9_GFX_FLSH_CNTL_GEN6           0x101008u
#define IG9_GFX_FLSH_CNTL_EN             (1u << 0)

/* Fence (detiling) registers: 32 on Gen9, 64-bit each. */
#define IG9_FENCE_REG_GEN6_LO(i)         (0x100000u + (uint32_t)(i) * 8u)
#define IG9_FENCE_REG_GEN6_HI(i)         (0x100004u + (uint32_t)(i) * 8u)
#define IG9_FENCE_COUNT                  32u
#define IG9_FENCE_VALID                  (1u << 0)

/* ------------------------------------------------------------------ *
 * Blitter engine (BCS) registers, relative to the engine base.
 * ------------------------------------------------------------------ */
#define IG9_BCS_BASE                     0x22000u
#define IG9_RING_TAIL(b)                 ((b) + 0x030u)
#define IG9_RING_HEAD(b)                 ((b) + 0x034u)
#define IG9_RING_START(b)                ((b) + 0x038u)
#define IG9_RING_CTL(b)                  ((b) + 0x03Cu)
#define IG9_RING_IPEIR(b)                ((b) + 0x064u)
#define IG9_RING_IPEHR(b)                ((b) + 0x068u)
#define IG9_RING_INSTDONE(b)             ((b) + 0x06Cu)
#define IG9_RING_ACTHD(b)                ((b) + 0x074u)
#define IG9_RING_HWS_PGA(b)              ((b) + 0x080u)
#define IG9_RING_HWSTAM(b)               ((b) + 0x098u)
#define IG9_RING_MI_MODE(b)              ((b) + 0x09Cu)
#define IG9_RING_IMR(b)                  ((b) + 0x0A8u)
#define IG9_RING_RESET_CTL(b)            ((b) + 0x0D0u)
#define IG9_RING_BBSTATE(b)              ((b) + 0x110u)
#define IG9_RING_SBBADDR(b)              ((b) + 0x114u)
#define IG9_RING_SBBSTATE(b)             ((b) + 0x118u)
#define IG9_RING_SBBADDR_UDW(b)          ((b) + 0x11Cu)
#define IG9_RING_BBADDR(b)               ((b) + 0x140u)
#define IG9_RING_BBADDR_UDW(b)           ((b) + 0x168u)
#define IG9_RING_ELSP(b)                 ((b) + 0x230u)
#define IG9_RING_EXECLIST_STATUS_LO(b)   ((b) + 0x234u)
#define IG9_RING_EXECLIST_STATUS_HI(b)   ((b) + 0x238u)
#define IG9_RING_CONTEXT_CONTROL(b)      ((b) + 0x244u)
#define IG9_RING_PDP_LDW(b, n)           ((b) + 0x270u + (uint32_t)(n) * 8u)
#define IG9_RING_PDP_UDW(b, n)           ((b) + 0x274u + (uint32_t)(n) * 8u)
#define IG9_RING_MODE(b)                 ((b) + 0x29Cu)   /* RING_MODE_GEN7 */
#define IG9_RING_CSB_BUF_LO(b, i)        ((b) + 0x370u + (uint32_t)(i) * 8u)
#define IG9_RING_CSB_BUF_HI(b, i)        ((b) + 0x374u + (uint32_t)(i) * 8u)
#define IG9_RING_CSB_PTR(b)              ((b) + 0x3A0u)
#define IG9_RING_CTX_TIMESTAMP(b)        ((b) + 0x3A8u)

#define IG9_RING_CTL_VALID               (1u << 0)
#define IG9_RING_CTL_SIZE(bytes)         (((uint32_t)(bytes) - 4096u) & 0x001FF000u)
#define IG9_RING_HEAD_ADDR_MASK          0x001FFFFCu
#define IG9_RING_TAIL_ADDR_MASK          0x001FFFF8u

#define IG9_MI_MODE_STOP_RING            (1u << 8)
#define IG9_MI_MODE_IDLE                 (1u << 9)

#define IG9_GFX_PPGTT_ENABLE             (1u << 9)
#define IG9_GFX_REPLAY_MODE              (1u << 11)
#define IG9_GFX_RUN_LIST_ENABLE          (1u << 15)

/* Context status buffer (Gen8-Gen10: 6 entries, 3-bit pointers). */
#define IG9_CSB_ENTRIES                  6u
#define IG9_CSB_PTR_MASK                 0x7u
#define IG9_CSB_WRITE_PTR(v)             ((v) & IG9_CSB_PTR_MASK)
#define IG9_CSB_READ_PTR(v)              (((v) >> 8) & IG9_CSB_PTR_MASK)
#define IG9_CSB_STATUS_IDLE_ACTIVE       (1u << 0)
#define IG9_CSB_STATUS_PREEMPTED         (1u << 1)
#define IG9_CSB_STATUS_ELEMENT_SWITCH    (1u << 2)
#define IG9_CSB_STATUS_ACTIVE_IDLE       (1u << 3)
#define IG9_CSB_STATUS_COMPLETE          (1u << 4)
#define IG9_CSB_STATUS_LITE_RESTORE      (1u << 15)

/* Context descriptor (intel_lrc.c default_desc_template, Gen9). */
#define IG9_CTX_VALID                    (1ull << 0)
#define IG9_CTX_FORCE_PD_RESTORE         (1ull << 1)
#define IG9_CTX_FORCE_RESTORE            (1ull << 2)
#define IG9_CTX_ADDRESSING_SHIFT         3u
#define IG9_CTX_LEGACY_32B               1ull
#define IG9_CTX_PRIVILEGE                (1ull << 8)
#define IG9_CTX_ID_SHIFT                 32u

/* CONTEXT_CONTROL bits (masked). */
#define IG9_CTX_CTRL_RESTORE_INHIBIT     (1u << 0)
#define IG9_CTX_CTRL_SAVE_INHIBIT        (1u << 2)
#define IG9_CTX_CTRL_INHIBIT_SYN_SWITCH  (1u << 3)

/* Logical ring context image, dword offsets inside the register-state page
 * (intel_lrc_reg.h, Gen8/Gen9 non-render engines). Each CTX_* names the
 * register-offset dword; the value lives at CTX_* + 1. */
#define IG9_CTX_LRI_HEADER_0             0x01u
#define IG9_CTX_CONTEXT_CONTROL          0x02u
#define IG9_CTX_RING_HEAD                0x04u
#define IG9_CTX_RING_TAIL                0x06u
#define IG9_CTX_RING_START               0x08u
#define IG9_CTX_RING_CTL                 0x0Au
#define IG9_CTX_BB_HEAD_U                0x0Cu
#define IG9_CTX_BB_HEAD_L                0x0Eu
#define IG9_CTX_BB_STATE                 0x10u
#define IG9_CTX_SECOND_BB_HEAD_U         0x12u
#define IG9_CTX_SECOND_BB_HEAD_L         0x14u
#define IG9_CTX_SECOND_BB_STATE          0x16u
#define IG9_CTX_LRI_HEADER_1             0x21u
#define IG9_CTX_TIMESTAMP                0x22u
#define IG9_CTX_PDP3_UDW                 0x24u
#define IG9_CTX_PDP3_LDW                 0x26u
#define IG9_CTX_PDP2_UDW                 0x28u
#define IG9_CTX_PDP2_LDW                 0x2Au
#define IG9_CTX_PDP1_UDW                 0x2Cu
#define IG9_CTX_PDP1_LDW                 0x2Eu
#define IG9_CTX_PDP0_UDW                 0x30u
#define IG9_CTX_PDP0_LDW                 0x32u
#define IG9_CTX_END                      0x34u
/* Context object layout: page 0 is the per-process HW status page, the
 * register state starts on page 1 (LRC_STATE_PN). Gen9 non-render engines
 * need 2 state pages (GEN8_LR_CONTEXT_OTHER_SIZE); 2 spare pages follow. */
#define IG9_CTX_STATE_PAGE               1u
#define IG9_CTX_OBJECT_PAGES             5u

/* Gen8 PPGTT entries (legacy 32-bit, 4 PDPs). */
#define IG9_PPGTT_PRESENT                (1ull << 0)
#define IG9_PPGTT_RW                     (1ull << 1)

/* ------------------------------------------------------------------ *
 * GGTT (Gen8+): 8-byte PTEs at BAR0 + (BAR0 size / 2).
 * gen8_ggtt_pte_encode(): addr | _PAGE_PRESENT.
 * ------------------------------------------------------------------ */
#define IG9_GGTT_PTE_PRESENT             (1ull << 0)
#define IG9_GGTT_PTE_ADDR_MASK           0x0000007FFFFFF000ull  /* bits 38:12 */
#define IG9_GGTT_PTE_BYTES               8u
#define IG9_PAGE_BYTES                   4096u

/* ------------------------------------------------------------------ *
 * Display engine (Skylake+ "universal planes"). Pipe stride 0x1000.
 * ------------------------------------------------------------------ */
#define IG9_PIPE_COUNT                   3u
#define IG9_PIPE_STRIDE                  0x1000u
#define IG9_PIPEDSL(p)                   (0x70000u + (uint32_t)(p) * IG9_PIPE_STRIDE)
#define IG9_PIPECONF(p)                  (0x70008u + (uint32_t)(p) * IG9_PIPE_STRIDE)
#define IG9_PIPECONF_ENABLE              (1u << 31)
#define IG9_PIPE_FRMCOUNT(p)             (0x70040u + (uint32_t)(p) * IG9_PIPE_STRIDE)
#define IG9_PIPESRC(p)                   (0x6001Cu + (uint32_t)(p) * IG9_PIPE_STRIDE)

/* Universal planes: plane n (1-based) of pipe p at 0x70180 + (n-1)*0x100. */
#define IG9_PLANE_MAX                    4u
#define IG9_PLANE_REG(p, n, off)         (0x70180u + (uint32_t)(p) * IG9_PIPE_STRIDE + \
                                          ((uint32_t)(n) - 1u) * 0x100u + (off))
#define IG9_PLANE_CTL(p, n)              IG9_PLANE_REG(p, n, 0x00u)
#define IG9_PLANE_STRIDE(p, n)           IG9_PLANE_REG(p, n, 0x08u)
#define IG9_PLANE_POS(p, n)              IG9_PLANE_REG(p, n, 0x0Cu)
#define IG9_PLANE_SIZE(p, n)             IG9_PLANE_REG(p, n, 0x10u)
#define IG9_PLANE_SURF(p, n)             IG9_PLANE_REG(p, n, 0x1Cu)
#define IG9_PLANE_OFFSET(p, n)           IG9_PLANE_REG(p, n, 0x24u)
#define IG9_PLANE_CTL_ENABLE             (1u << 31)
#define IG9_PLANE_CTL_FORMAT_MASK        (0xFu << 24)
#define IG9_PLANE_CTL_FORMAT_XRGB8888    (0x4u << 24)
#define IG9_PLANE_CTL_TILED_MASK         (0x7u << 10)
#define IG9_PLANE_CTL_TILED_LINEAR       (0x0u << 10)
#define IG9_PLANE_CTL_TILED_X            (0x1u << 10)
#define IG9_PLANE_SURF_ADDR_MASK         0xFFFFF000u

/* DDI transcoders. */
#define IG9_TRANS_DDI_FUNC_CTL(t)        (0x60400u + (uint32_t)(t) * 0x1000u)
#define IG9_TRANS_DDI_FUNC_CTL_EDP       0x6F400u
#define IG9_TRANS_DDI_ENABLE             (1u << 31)
#define IG9_TRANS_DDI_PORT_SHIFT         28u
#define IG9_TRANS_DDI_PORT_MASK          0x7u
#define IG9_TRANS_DDI_MODE_SHIFT         24u
#define IG9_TRANS_DDI_MODE_MASK          0x7u
#define IG9_TRANS_DDI_MODE_HDMI          0u
#define IG9_TRANS_DDI_MODE_DVI           1u
#define IG9_TRANS_DDI_MODE_DP_SST        2u
#define IG9_TRANS_DDI_MODE_DP_MST        3u
#define IG9_TRANS_DDI_EDP_INPUT_SHIFT    12u
#define IG9_TRANS_DDI_EDP_INPUT_MASK     0x7u

/* ------------------------------------------------------------------ *
 * Command encodings (intel_gpu_commands.h)
 * ------------------------------------------------------------------ */
#define IG9_MI_INSTR(op, flags)          (((uint32_t)(op) << 23) | (uint32_t)(flags))
#define IG9_MI_NOOP                      IG9_MI_INSTR(0x00u, 0u)
#define IG9_MI_USER_INTERRUPT            IG9_MI_INSTR(0x02u, 0u)
#define IG9_MI_BATCH_BUFFER_END          IG9_MI_INSTR(0x0Au, 0u)
#define IG9_MI_STORE_DWORD_IMM_GEN4      IG9_MI_INSTR(0x20u, 2u)   /* 4 dwords */
#define IG9_MI_USE_GGTT                  (1u << 22)
#define IG9_MI_FLUSH_DW                  IG9_MI_INSTR(0x26u, 1u)
#define IG9_MI_FLUSH_DW_OP_STOREDW       (1u << 14)
#define IG9_MI_FLUSH_DW_USE_GTT          (1u << 2)                 /* in address dword */
#define IG9_MI_BATCH_BUFFER_START_GEN8   IG9_MI_INSTR(0x31u, 1u)   /* 3 dwords */
#define IG9_MI_BATCH_PPGTT               (1u << 8)                 /* clear = GGTT batch */
#define IG9_MI_LOAD_REGISTER_IMM(n)      IG9_MI_INSTR(0x22u, 2u * (n) - 1u)
#define IG9_MI_LRI_FORCE_POSTED          (1u << 12)
#define IG9_MI_OPCODE(dw)                (((dw) >> 23) & 0x3Fu)

/* 2D blitter. On Gen8+ every surface address is 64-bit, which makes
 * XY_COLOR_BLT 7 dwords and XY_SRC_COPY_BLT 10 dwords (Mesa intel_blit.c,
 * xf86-video-intel sna: "gen >= 0100 ? 10 : 8"). */
#define IG9_BLT_CLIENT                   (2u << 29)
#define IG9_XY_COLOR_BLT_CMD             (IG9_BLT_CLIENT | (0x50u << 22))
#define IG9_XY_SRC_COPY_BLT_CMD          (IG9_BLT_CLIENT | (0x53u << 22))
#define IG9_BLT_OPCODE(dw)               (((dw) >> 22) & 0x7Fu)
#define IG9_XY_COLOR_BLT_DWORDS          7u
#define IG9_XY_SRC_COPY_BLT_DWORDS       10u
#define IG9_BLT_WRITE_ALPHA              (1u << 21)
#define IG9_BLT_WRITE_RGB                (1u << 20)
#define IG9_BLT_SRC_TILED                (1u << 15)
#define IG9_BLT_DST_TILED                (1u << 11)
#define IG9_BLT_DEPTH_32                 (3u << 24)
#define IG9_BLT_ROP_SRC_COPY             (0xCCu << 16)
#define IG9_BLT_ROP_PAT_COPY             (0xF0u << 16)
#define IG9_BLT_LEN(dwords)              ((uint32_t)(dwords) - 2u)
#define IG9_BLT_MAX_COORD                0x7FFFu
#define IG9_BLT_MAX_PITCH                0x7FFFu
