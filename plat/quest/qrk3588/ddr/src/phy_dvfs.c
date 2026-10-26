/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* DDR PHY DVFS transaction orchestration.
 *
 * Builds the per-frequency workspace, switches active channels, executes the
 * training/configuration passes, restores channel state and exports a profile
 * snapshot into the shared layout storage.
 */
#include "qrk3588_ddr_internal.h"
#include "qrk3588_ddr_runtime.h"

#include <common/debug.h>


#define DVFS_EINVAL    (-22)
#define DVFS_ETIMEDOUT (-110)
#define DVFS_CONTEXT_SLOT_ADDR ((uint32_t)QRK3588_DDR_CONTEXT_SLOT_VA)
#define DVFS_ACTIVE_CFG_ADDR   ((uint32_t)QRK3588_DDR_ACTIVE_CONFIG_SLOT_VA)
#define DVFS_DRAM_SRC_ADDR     ((uint32_t)QRK3588_DDR_DRAM_SOURCE_SLOT_VA)
#define DVFS_WORK_ADDR         ((uint32_t)QRK3588_DDR_DVFS_WORK_SLOT_VA)
#define DVFS_FREQ_AUX_ADDR     ((uint32_t)QRK3588_DDR_FREQUENCY_AUX_SLOT_VA)
#define DVFS_SNAPSHOT_ADDR     ((uint32_t)QRK3588_DDR_DVFS_IMAGE_VA)
#define DVFS_DESC_TABLE_ADDR   ((uint32_t)QRK3588_DDR_DESCRIPTOR_TABLE_VA)
#define DVFS_PATCH_TABLE_ADDR  ((uint32_t)QRK3588_DDR_TYPE_CONFIG_TABLE_VA)

static inline uintptr_t dv_addr(const void *base, size_t off)
{
    return qrk3588_ddr_resolve_addr((uintptr_t)base + off);
}
static inline uint8_t dv_r8(const void *base, size_t off)
{
    return *(volatile const uint8_t *)(uintptr_t)dv_addr(base, off);
}
static inline uint32_t dv_r32(const void *base, size_t off)
{
    return *(volatile const uint32_t *)(uintptr_t)dv_addr(base, off);
}
static inline uint64_t dv_r64(const void *base, size_t off)
{
    return *(volatile const uint64_t *)(uintptr_t)dv_addr(base, off);
}
static inline void dv_w32(void *base, size_t off, uint32_t v)
{
    *(volatile uint32_t *)(uintptr_t)dv_addr(base, off) = v;
}
static inline uint8_t *dv_ptr_slot(uint32_t absolute)
{
    return (uint8_t *)(uintptr_t)DDR_LAYOUT_REF(uintptr_t, absolute);
}
static inline uint8_t *dv_ch_slot(uint8_t *ctx, uint32_t ch)
{
    return qrk3588_ddr_ctx_channel_slot(ctx, ch);
}
static inline uint8_t *dv_phy(uint8_t *ctx, uint32_t ch)
{
    return (uint8_t *)(uintptr_t)dv_r64(dv_ch_slot(ctx, ch), 0u);
}
static inline uint8_t *dv_ctrl(uint8_t *ctx, uint32_t ch)
{
    return (uint8_t *)(uintptr_t)dv_r64(dv_ch_slot(ctx, ch), 8u);
}
static inline uint8_t *dv_aux(uint8_t *ctx, uint32_t ch)
{
    return (uint8_t *)(uintptr_t)dv_r64(dv_ch_slot(ctx, ch), 0x10u);
}
static inline uint8_t *dv_lane_root(uint8_t *ctx, uint32_t ch)
{
    return (uint8_t *)qrk3588_ddr_ctx_shared_bank(ctx, ch);
}
static inline size_t dv_lane_off(uint32_t ch)
{
    return ((size_t)(ch & 1u) + 0x4cu) * 4u;
}
static int dv_wait(uint8_t *base, size_t off, uint32_t mask, uint32_t value)
{
    return qrk3588_ddr_poll32((uintptr_t)(base + off), mask, value, 0u);
}

static uint32_t dv_selector_from_profile(uint8_t *ctx, uint32_t profile, int *ok)
{
    uint8_t *ctrl0 = (uint8_t *)(uintptr_t)dv_r64(ctx, 8u);
    uint32_t top = dv_r32(ctrl0, 0xb8u) >> 30;
	*ok = 1;
	if (profile == 0u)
		return 0u;
	if ((top & 1u) == 0u)
		return 1u;
	if (top == 1u)
		return 2u;
	*ok = 0;
    return 0u;
}

/* LP5 ordering workaround for the channel-switch sequence.  It may
 * touch a physical alias solely to force an ordered read before DVFS. */
static void dv_lp5_ordering_sync(uint8_t *ctx, uint8_t *active)
{
    uint8_t *probe = (uint8_t *)(uintptr_t)dv_r64(ctx, 0x98u);
    uint64_t stride = 0u;
    uint32_t selector = 0xffu;
    uint32_t shift = 0u;

    if (probe != NULL && (dv_r32(probe, 0x300u) & 1u) != 0u) {
        selector = (dv_r32(probe, 0x300u) >> 4) & 0xfu;
        shift = selector <= 7u ? selector + 7u : selector - 1u;
    } else {
        uint8_t *map = (uint8_t *)(uintptr_t)dv_r64(ctx, 0x90u);
        uint32_t code = map != NULL ? ((dv_r32(map, 0x10u) >> 4) & 0x1fu) : 0u;
        switch (code) {
        case 7u: stride = UINT64_C(0x10000000); break;
        case 4u: stride = UINT64_C(0x20000000); break;
        case 3u: stride = UINT64_C(0x40000000); break;
        case 8u: stride = UINT64_C(0x60000000); break;
        case 2u: stride = UINT64_C(0x80000000); break;
        case 6u: stride = UINT64_C(0xc0000000); break;
        case 1u: stride = UINT64_C(0x100000000); break;
        case 5u: stride = UINT64_C(0x180000000); break;
        default: stride = 0u; break;
        }
    }

    for (unsigned i = 0; i < 4u; ++i) {
        uint32_t ch = qrk3588_ddr_ctx_route(ctx, i);
        if (ch == 0xfu)
            continue;
        /* Any non-rank1 channel makes the DVFS flow leave this special
         * preamble immediately. */
        if (qrk3588_ddr_state_channel_rank_count(active, ch) != 1u)
            return;
        uint8_t *pctl = dv_phy(ctx, ch) + 0x10000u;
        uint32_t v = dv_r32(pctl, 0x100u);
        if ((v & 1u) == 0u || (v & 4u) != 0u)
            return;
        dv_w32(pctl, 0x100u, v | 4u);
        uintptr_t pa = selector != 0xffu ? ((uintptr_t)ch << shift)
                                        : (uintptr_t)((uint64_t)ch * stride);
        if (pa != 0u) {
            volatile uint64_t sink = *(volatile uint64_t *)(uintptr_t)qrk3588_ddr_resolve_addr(pa);
            (void)sink;
        }
#if defined(__aarch64__)
        __asm__ volatile("isb" ::: "memory");
        __asm__ volatile("dsb sy" ::: "memory");
#endif
    }
}

static int dv_wait_stage(uint32_t stage,
			 uint8_t *base, uint32_t offset,
			 uint32_t mask, uint32_t expected)
{
	int rc;

	rc = dv_wait(base, offset, mask, expected);
	if (rc != 0) {
		ERROR("DDR: DVFS wait failed stage=%u "
		       "addr=%p+0x%x val=%08x mask=%08x exp=%08x rc=%d\n",
		       stage,
		       (void *)base, offset,
		       dv_r32(base, offset),
		       mask, expected, rc);
	}

	return rc;
}

static int dv_sticky_error(void)
{
    return qrk3588_ddr_get_error();
}

static int dv_update_complete_checked(uint8_t *phy)
{
    ddr_phy_update_complete((uint32_t)(uintptr_t)phy);
    return dv_sticky_error();
}

static int dv_phy_reg_write_checked(uint8_t *phy, uint32_t rank,
                                    uint32_t reg, uint32_t value)
{
    (void)ddr_phy_reg_write((uint32_t)(uintptr_t)phy, rank, reg, value);
    return dv_sticky_error();
}

static int dv_switch_channel(uint8_t *ctx, uint8_t *active, uint8_t *work,
                             const uint8_t *freq_cfg, uint32_t frequency,
                             uint32_t profile, uint32_t selector,
                             uint32_t inverted, uint32_t dram_type,
                             uint32_t row_index)
{
    uint32_t ch = qrk3588_ddr_ctx_route(ctx, row_index);
    if (ch == 0xfu)
        return 0;

    uint8_t *slot = dv_ch_slot(ctx, ch);
    uint8_t *phy = dv_phy(ctx, ch);
    uint8_t *ctrl = dv_ctrl(ctx, ch);
    uint8_t *aux = dv_aux(ctx, ch);
    uint8_t *pctl = phy + 0x10000u;
    uint8_t *phy_hi = phy + 0x20000u;
    uint8_t *lane = dv_lane_root(ctx, ch);
    size_t lane_off = dv_lane_off(ch);
    size_t bank = (size_t)profile << 20;
    uint32_t saved_af;
    uint32_t saved_cb;
    uint32_t saved_184;
    uint32_t saved_500;
    uint32_t flag_100 = 0u, flag_508_31 = 0u, flag_518 = 0u;
    uint32_t flag_banka80 = 0u, flag_508_15 = 0u;
    uint32_t lane_bits = (inverted << 7) | (inverted << 6);

    dv_w32(ctrl, 0x04u, dv_r32(ctrl, 0x04u) & ~8u);

    (void)ddr_training_mirror_entry(1u, ch, 1u, 3u);
    (void)ddr_training_mirror_entry(2u, ch, 1u, 3u);
    dv_w32(ctrl, 0xb8u, dv_r32(ctrl, 0xb8u) & ~0x2000u);
    fout_mhz_setup(ctx, ch, inverted, qrk3588_ddr_state_frequency(work) << 1);
    if (dv_sticky_error() != 0)
        return dv_sticky_error();

    /* Copy only the two proven geometry fields required before rebuilding the
     * frequency images. */
    struct qrk3588_ddr_channel_geometry *work_geometry =
        qrk3588_ddr_state_channel_geometry(work, row_index);
    const struct qrk3588_ddr_channel_geometry *active_geometry =
        qrk3588_ddr_state_channel_geometry_const(active, row_index);
    work_geometry->rank_count = active_geometry->rank_count;
    work_geometry->bus_width_code = active_geometry->bus_width_code;

    dv_w32(pctl, 0xc80u, 0u);
    ddr_pctl_frequency_register_image_build(ctx, ch, work, profile);

    {
        size_t o = bank + 0xd0cu;
        uint32_t v = dv_r32(phy, o) & 0xff00ffe0u;
        v |= ((qrk3588_ddr_ctx_descriptor_state_const(ctx)->word4_hi16 & 0xffu) << 16);
        v |= qrk3588_ddr_ctx_descriptor_state_const(ctx)->word4_lo16 & 0x1fu;
        if (profile == 1u && qrk3588_ddr_ctx_descriptor_state_const(ctx)->word4_hi16 == 0u) {
            v |= 0x00600000u;
            dv_w32(phy, o, v);
            if (dv_update_complete_checked(phy) != 0)
                return dv_sticky_error();
        } else {
            dv_w32(phy, o, v);
            if (dv_update_complete_checked(phy) != 0)
                return dv_sticky_error();
        }
    }

    ddr_phy_timing_select(ctx, ch, work, selector);
    ddr_phy_frequency_register_image_build(ctx, ch, work, selector, profile, 1u);

    if (dram_type == 7u || dram_type == 8u) {
        uint32_t v = dv_r32(phy, bank + 0x504u) & 0xffffff3fu;
        /*
         * MR13[7:6] selects the LPDDR4 FSP.  The frozen DVFS path derives
         * this from the inverted descriptor bit (0x80 >> inverted); the
         * selector belongs only to DDRPHY_DVFS_CON[31:30] below.
         */
        uint32_t reg = v | (UINT32_C(0x80) >> inverted);
        if (dv_phy_reg_write_checked(phy, 3u, 13u, reg) != 0)
            return dv_sticky_error();
        uint32_t a84 = dv_r32(ctrl, 0xa84u) & 0xffffff00u;
        a84 |= lane_bits | v;
        dv_w32(ctrl, 0xa84u, a84);
    } else if (dram_type == 9u) {
        const struct qrk3588_ddr_lp5_channel_image *image =
            qrk3588_ddr_state_lp5_channel_image_const(work, ch);
        uint32_t base = image->byte[9] & 0xf0u;
        uint32_t reg = base | inverted | ((inverted ? 4u : 0u));
        if (dv_phy_reg_write_checked(phy, 3u, 16u, reg) != 0)
            return dv_sticky_error();
        uint32_t a88 = dv_r32(ctrl, 0xa88u) & 0x00ffffffu;
        a88 |= ((reg & ~0x0cu) | (inverted << 2)) << 24;
        dv_w32(ctrl, 0xa88u, a88);
    }

    ddr_phy_frequency_parameter_stage(ctx, ch, work, profile, selector, 1u);
    if (dv_sticky_error() != 0)
        return dv_sticky_error();
    if (dram_type == 7u || dram_type == 8u)
        ddr_phy_apply_profile_registers((uint32_t)(uintptr_t)phy, profile, dram_type);
    else if (dram_type == 9u)
        ddr_phy_apply_byte_config((uint32_t)(uintptr_t)phy,
                            qrk3588_ddr_state_lp5_channel_image_const(work, ch)->byte);
    if (dv_sticky_error() != 0)
        return dv_sticky_error();

    ddr_channel_timing_pack(ctx, ch, work);
    ddr_phy_timing_program((uint32_t)(uintptr_t)phy, ch,
                           qrk3588_ddr_state_channel_geometry_const(active, ch),
                           dram_type, frequency);
    if (dv_sticky_error() != 0)
        return dv_sticky_error();
    ddr_phy_state_capture_and_clear(ctx, ch, work);
    if (dv_sticky_error() != 0)
        return dv_sticky_error();

    if (selector == 1u) saved_af = dv_r32(ctrl, 0xaf4u);
    else if (selector == 2u) saved_af = dv_r32(ctrl, 0xaf8u);
    else saved_af = dv_r32(ctrl, 0xadcu);

    dv_w32(phy_hi, 0x90u, dv_r32(phy_hi, 0x90u) & ~1u);
    if (dv_wait_stage(1, phy_hi, 0x114u, 0x10001u, 0u) != 0) return DVFS_ETIMEDOUT;
    saved_cb = ddr_phy_request_pause(phy);
    if (dv_sticky_error() != 0)
        return dv_sticky_error();
    dv_w32(pctl, 0xb84u, dv_r32(pctl, 0xb84u) | 2u);
    if (dv_wait_stage(2, pctl, 0xb88u, 0x36000000u, 0x36000000u) != 0) return DVFS_ETIMEDOUT;

    if ((dv_r32(pctl, 0x100u) & 1u) != 0u) {
        if ((dv_r32(pctl, 0x200u) & 0x100u) == 0u) {
            dv_w32(pctl, 0x100u, dv_r32(pctl, 0x100u) | 4u);
            flag_100 = 2u;
        } else {
            dv_w32(pctl, 0x100u, dv_r32(pctl, 0x100u) & ~1u);
            flag_100 = 1u;
        }
    }

    dv_w32(pctl, 0xc80u, 0u);
    if ((dv_r32(pctl, 0x508u) & 0x80000000u) == 0u) {
        dv_w32(pctl, 0x508u, dv_r32(pctl, 0x508u) | 0x80000000u);
        flag_508_31 = 1u;
    }
    /* The update call is unconditional; bit31 only controls later cleanup. */
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    dv_w32(pctl, 0xb8cu, dv_r32(pctl, 0xb8cu) | 0x20000u);
    if (dv_wait_stage(3, pctl, 0xb90u, 0x20000u, 0u) != 0) return DVFS_ETIMEDOUT;

    dv_w32(pctl, 0xc80u, 0u);
    if ((dv_r32(pctl, 0x518u) & 1u) != 0u) {
        dv_w32(pctl, 0x518u, dv_r32(pctl, 0x518u) & ~1u);
        flag_518 = 1u;
    }
    /* Update is unconditional; the branch only controls whether bit0 was saved. */
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    if ((dv_r32(phy, bank + 0xa80u) & 1u) != 0u) {
        dv_w32(phy, bank + 0xa80u, dv_r32(phy, bank + 0xa80u) & ~1u);
        flag_banka80 = 1u;
    }

    dv_w32(pctl, 0xc80u, 0u);
    saved_184 = dv_r32(pctl, 0x184u);
    dv_w32(pctl, 0x184u, saved_184 & ~1u);
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    if (dv_wait_stage(4, pctl, 0x14u, 7u, 1u) != 0) return DVFS_ETIMEDOUT;
    saved_500 = dv_r32(pctl, 0x500u);
    dv_w32(pctl, 0x500u, 0u);
    if (dv_wait_stage(5, pctl, 0x514u, 6u, 0u) != 0) return DVFS_ETIMEDOUT;
    dv_w32(pctl, 0x180u, (dv_r32(pctl, 0x180u) & ~0x200u) | 0x800u);
    if (dv_wait_stage(6, pctl, 0x14u, 7u, 3u) != 0) return DVFS_ETIMEDOUT;
    if (dv_wait_stage(7, pctl, 0x14u, 0x30u, 0x20u) != 0) return DVFS_ETIMEDOUT;
    dv_w32(pctl, 0xb84u, dv_r32(pctl, 0xb84u) | 1u);
    if (dv_wait_stage(8, pctl, 0xb88u, 0x30000000u, 0x30000000u) != 0) return DVFS_ETIMEDOUT;
    dv_w32(pctl, 0xc80u, 0u);
    dv_w32(pctl, 0x510u, dv_r32(pctl, 0x510u) & ~1u);
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    if ((dv_r32(pctl, 0x508u) & 0x8000u) != 0u) {
        dv_w32(pctl, 0x508u, dv_r32(pctl, 0x508u) & ~0x8000u);
        flag_508_15 = 1u;
        if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    }

    dv_w32(ctrl, 0xb8u, (dv_r32(ctrl, 0xb8u) & 0x3fffffffu) | (selector << 30));
    dv_w32(ctrl, 0xe04u, dv_r32(ctrl, 0xe04u) & 0x7fffffffu);
    dv_w32(pctl, 0xc80u, 0u);
    dv_w32(pctl, 0x510u, dv_r32(pctl, 0x510u) | 0x20u);
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    if (dv_wait_stage(9, pctl, 0x514u, 1u, 0u) != 0) return DVFS_ETIMEDOUT;

    dv_w32(lane, lane_off, 0x00080000u);
    dv_w32(aux, 0xa00u, 0x00020002u);
    dv_w32(lane, lane_off, selector | 0x00030000u);
    dv_w32(aux, 0x300u, inverted | 0x00010000u);
    dv_w32(aux, 0xa00u, 0x00020000u);
    dv_w32(lane, lane_off, 0x00080008u);
    dv_w32(pctl, 0xc80u, 0u);
    dv_w32(pctl, 0x08u, (dv_r32(pctl, 0x08u) & ~3u) | profile);
    if (dram_type == 9u)
        dv_w32(pctl, 0x10u, (dv_r32(pctl, 0x10u) & ~1u) | (saved_af & 1u));
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    dv_w32(pctl, 0x208u, dv_r32(pctl, 0x208u) | 0x10u);
    dv_w32(pctl, 0x208u, dv_r32(pctl, 0x208u) & ~0x10u);
    dv_w32(pctl, 0xc80u, 0u);
    dv_w32(pctl, 0x510u, dv_r32(pctl, 0x510u) & ~0x20u);
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();

    if (flag_508_15) {
        dv_w32(pctl, 0xc80u, 0u);
        dv_w32(pctl, 0x508u, dv_r32(pctl, 0x508u) | 0x8000u);
        if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    }
    if (dv_wait_stage(10, pctl, 0x514u, 1u, 1u) != 0) return DVFS_ETIMEDOUT;
    dv_w32(ctrl, 0xe04u, dv_r32(ctrl, 0xe04u) | 0x80000000u);
    dv_w32(pctl, 0xc80u, 0u);
    dv_w32(pctl, 0x510u, dv_r32(pctl, 0x510u) | 1u);
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    dv_w32(pctl, 0x180u, dv_r32(pctl, 0x180u) & ~0x800u);
    /* Vendor waits until state leaves 3. */
    if ((dv_r32(pctl, 0x14u) & 7u) == 3u) {
        if (dv_wait_stage(11, pctl, 0x14u, 7u, 1u) != 0) return DVFS_ETIMEDOUT;
    }
    dv_w32(pctl, 0x500u, saved_500);
    dv_w32(pctl, 0xb84u, dv_r32(pctl, 0xb84u) & ~1u);

    if (dram_type == 7u || dram_type == 8u) {
        uint32_t reg = (dv_r32(phy, bank + 0x504u) & 0xffffff7fu) | lane_bits;
        if (dv_phy_reg_write_checked(phy, 3u, 13u, reg | 8u) != 0) return dv_sticky_error();
        udelay(1u);
        if (dv_phy_reg_write_checked(phy, 3u, 13u, reg) != 0) return dv_sticky_error();
    } else if (dram_type == 9u) {
        uint8_t *r = work + (size_t)ch * 20u;
        uint32_t reg = (dv_r8(r, 0x1ddu) & 0xf0u) | (inverted | (inverted << 2));
        if (dv_phy_reg_write_checked(phy, 3u, 16u, reg | 0x40u) != 0) return dv_sticky_error();
        udelay(1u);
        if (dv_phy_reg_write_checked(phy, 3u, 16u, reg) != 0) return dv_sticky_error();
    }
    dv_w32(pctl, 0xb84u, dv_r32(pctl, 0xb84u) & ~2u);
    if (flag_100 == 1u) {
        dv_w32(pctl, 0x100u, dv_r32(pctl, 0x100u) | 1u);
        dv_w32(pctl, 0x100u, dv_r32(pctl, 0x100u) | 4u);
        dv_w32(pctl, 0x100u, dv_r32(pctl, 0x100u) & ~4u);
    } else if (flag_100 == 2u) {
        dv_w32(pctl, 0x100u, dv_r32(pctl, 0x100u) & ~4u);
    }
    if (flag_508_31) {
        dv_w32(pctl, 0xc80u, 0u);
        dv_w32(pctl, 0x508u, dv_r32(pctl, 0x508u) & 0x7fffffffu);
        if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    }
    dv_w32(pctl, 0xc80u, 0u);
    if (flag_518)
        dv_w32(pctl, 0x518u, dv_r32(pctl, 0x518u) | 1u);
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    if (flag_banka80)
        dv_w32(phy, bank + 0xa80u, dv_r32(phy, bank + 0xa80u) | 1u);
    dv_w32(pctl, 0xc80u, 0u);
    dv_w32(pctl, 0x184u, saved_184);
    if (dv_update_complete_checked(phy) != 0) return dv_sticky_error();
    ddr_phy_request_restore(phy, saved_cb);
    dv_w32(phy_hi, 0x90u, dv_r32(phy_hi, 0x90u) | 1u);

    dv_w32(lane, lane_off, 0x00200020u);
    dv_w32(lane, lane_off, 0x00100010u);
    dv_w32(ctrl, 0xb0u, (dv_r32(ctrl, 0xb0u) & ~0x20u) | ((uint32_t)freq_cfg[1] << 5));
    dv_w32(ctrl, 0xb0u, dv_r32(ctrl, 0xb0u) & ~0x40u);
    dv_w32(ctrl, 0xb0u, dv_r32(ctrl, 0xb0u) | 0x40u);
    if (dv_wait_stage(12, ctrl, 0xb4u, 1u << 18, 1u << 18) != 0) return DVFS_ETIMEDOUT;
    dv_w32(lane, lane_off, 0x00100000u);
    dv_w32(lane, lane_off, 0x00200000u);
    (void)slot;
    return 0;
}

static int dv_run_config_passes(uint8_t *ctx, uint8_t *work,
                                uint32_t profile, uint32_t selector)
{
    static const struct {
        uint32_t flags;
        uint32_t control;
    } pass[4] = {
        { 0x10u, 0x100u },
        { 0x10u, 0x001u },
        { 0x08u, 0x000u },
        { 0x08u, 0x201u },
    };

    for (unsigned p = 0; p < 4u; ++p) {
        for (unsigned i = 0; i < 4u; ++i) {
            uint32_t ch = qrk3588_ddr_ctx_route(ctx, i);

            if (ch == 0xfu)
                continue;

            uint32_t mode =
                qrk3588_ddr_state_channel_rank_count(work, ch) == 2u ?
                3u : 0u;

            (void)ddr_config_write_seq(ctx, ch, mode, work,
                                       selector, profile,
                                       pass[p].flags, 1u,
                                       pass[p].control);
            if (dv_sticky_error() != 0)
                return dv_sticky_error();

            /* Complete all passes even when an individual training command
             * reports failure; later passes restore the controller state. */
        }
    }
    return 0;
}

static void dv_patch_post_training_channel(uint8_t *ctx, uint8_t *work,
                                           uint32_t frequency,
                                           uint32_t selector, uint32_t ch)
{
    const uint8_t *tbl = DDR_LAYOUT_CPTR(uint8_t, DVFS_PATCH_TABLE_ADDR);
    uint8_t index = tbl[0x0au];
    uint32_t field = (dv_r32(tbl, 0x14u + (size_t)index * 4u) >> 11) & 0x7fu;
    if (field == 0u) return;
    uint32_t scaled = field;

    uint8_t *ctrl = dv_ctrl(ctx, ch);
    uint32_t row = qrk3588_ddr_state_channel_rank_count(work, ch);
    uint32_t rank_bits = 0u, valid = 0u;
    if (frequency > 1065u) {
        scaled = field * UINT32_C(0x249f00);
        rank_bits = row == 1u ? 1u : 3u;
        valid = 1u;
    }
    size_t reg = selector == 1u ? 0xa48u : (selector == 2u ? 0xa58u : 0xa24u);
    uint32_t shift = selector == 1u || selector == 2u ? 14u : 22u;
    uint32_t mask = selector == 1u || selector == 2u ? 0xffff3fffu : 0xff3fffffu;
    dv_w32(ctrl, reg, (dv_r32(ctrl, reg) & mask) | (rank_bits << shift));
    if (valid) {
        dv_w32(ctrl, 0xa28u, (dv_r32(ctrl, 0xa28u) & 0x80000000u) | scaled);
    }
    if (selector == 1u || selector == 2u)
        dv_w32(ctrl, reg, (dv_r32(ctrl, reg) & ~0x20000u) | (valid << 17));
    else
        dv_w32(ctrl, 0xa28u, (dv_r32(ctrl, 0xa28u) & 0x7fffffffu) | (valid << 31));
}

static int dv_restore_training(uint8_t *ctx, uint8_t *work,
                               uint32_t frequency, uint32_t profile,
                               uint32_t selector)
{
    if (qrk3588_ddr_ctx_descriptor_state_const(ctx)->wordc_bit19_inverted == 1u)
        print_mode_registers(work, 0u);

    for (unsigned i = 0; i < 4u; ++i) {
        uint32_t ch = qrk3588_ddr_ctx_route(ctx, i);

        if (ch == 0xfu) continue;
        uint8_t *ctrl = dv_ctrl(ctx, ch);
        if (frequency >= 533u && frequency <= 1065u) {
            dv_w32(ctrl, 0x04u, dv_r32(ctrl, 0x04u) | 0x20u);
            dv_w32(ctrl, 0x10u, dv_r32(ctrl, 0x10u) | 0x200u);
            ddr_phy_update_toggle((uintptr_t)ctrl);
            dv_w32(ctrl, 0x10u, dv_r32(ctrl, 0x10u) & ~0x200u);
        }
        if (qrk3588_ddr_ctx_descriptor_state_const(ctx)->wordc_bit19_inverted == 1u)
            print_train_result(ch, work);
        (void)phy_update_seq(work, ctx, ch,
                             qrk3588_ddr_state_channel_rank_count(work, ch),
                             NULL, NULL);
        if (dv_sticky_error() != 0)
            return dv_sticky_error();
        dv_patch_post_training_channel(ctx, work, frequency, selector, ch);
        ddr_phy_state_restore(ctx, ch, work);
    }
    return 0;
}

static void dv_export_descriptor_window(uint8_t *ctx, uint8_t *work,
                                        uint8_t *snapshot, uint32_t selector)
{
    uint8_t *controller0 = dv_ctrl(ctx, 0u);
    uint8_t *clock = (uint8_t *)(uintptr_t)dv_r64(ctx, 0x88u);
    uint8_t *desc = (uint8_t *)(uintptr_t)dv_r64(ctx, 0x10u);
    uint32_t ref_clock_mhz = ddr_reference_clock_hz() / 1000000u;
    uint32_t old594 = dv_r32(clock, 0x594u);
    uint32_t extended = dv_r32(desc, 0x300u) & 1u;
    size_t base = extended ? 0x20u : 0u;
    uint32_t d0 = dv_r32(desc, base + 0u);
    uint32_t d1 = dv_r32(desc, base + 4u);
    uint32_t d2 = dv_r32(desc, base + 8u);
    uint32_t low = d0 & 0x3ffu;
    uint32_t shift = (d1 >> 6) & 7u;
    uint32_t div = d1 & 0x3fu;
    uint32_t num = d2 & 0xffffu;
    uint32_t denom = UINT32_C(0xfff5);
    uint32_t scaled = div ? (((num / denom) + low) * (ref_clock_mhz << 1)) / (div << shift) : 0u;

    dv_w32(clock, 0x594u, 0x00c00040u);
    dv_w32(snapshot, 4u, scaled >> 1);
    const uint32_t *table = DDR_LAYOUT_CPTR(uint32_t, DVFS_DESC_TABLE_ADDR);
    uint32_t lane_mask = dv_r32(work, 0u) == 1u ? 0xc00u : 0x400u;
    for (unsigned i = 0; i < 11u; ++i) {
        uint8_t *out = snapshot + 8u + i * 0x10u;
        uint32_t a=table[i*3u], b=table[i*3u+1u], c=table[i*3u+2u];
        dv_w32(out,0u,a); dv_w32(out,4u,b); dv_w32(out,8u,c);
        uint32_t off = selector == 1u ? a : (selector == 2u ? b : c);
        uint32_t value = off == UINT32_MAX ? 0u : dv_r32(controller0, off);
        if (scaled <= 1065u && (off & ~0x10u) == 0xa48u)
            value |= lane_mask;
        dv_w32(out,0x0cu,value);
    }
    dv_w32(snapshot,0xb8u,UINT32_MAX);

    dv_w32(clock,0x594u,0x00c00040u);
    for (unsigned i=0;i<7u;i++) dv_w32(snapshot,0xf8u+i*4u,dv_r32(desc,base+i*4u));
    dv_w32(clock,0x594u,(old594 & 0xc0u) | 0x00c00000u);
    dv_w32(snapshot,0x124u,dv_r32(controller0,0x04u)&0x00400000u);
    dv_w32(snapshot, 0x128u,
           qrk3588_ddr_state_phy_saved_const(work, 0u)->shared_18_value);
    {
        const struct qrk3588_ddr_phy_divider_state *divider_state =
            qrk3588_ddr_state_phy_dividers_const(work);
        dv_w32(snapshot, 0x140u, divider_state->lane2);
        dv_w32(snapshot, 0x144u, divider_state->lane1);
        dv_w32(snapshot, 0x148u, divider_state->lane0);
        dv_w32(snapshot, 0x14cu, divider_state->delta);
        (void)ddr_memcpy(snapshot + 0x150u,
                         qrk3588_ddr_state_global_const(work)->lp5_channel, 0x50u);
    }
}

static void dv_export_training_snapshot(uint8_t *ctx, uint8_t *work,
                                        uint32_t profile, uint32_t selector)
{
    uint8_t *all = DDR_LAYOUT_PTR(uint8_t, DVFS_SNAPSHOT_ADDR);
    uint8_t *snap = all + (size_t)profile * 0x850u;
    dv_export_descriptor_window(ctx, work, snap, selector);

    for (unsigned i=0;i<4u;i++) {
        uint32_t ch=qrk3588_ddr_ctx_route(ctx, i); if(ch==0xfu)continue;
        dv_w32(snap,0x114u+(size_t)ch*4u,
               ((*qrk3588_ddr_state_private_status_const(work, ch) & 0xffu) << 8) |
               (*qrk3588_ddr_state_limit_code_const(work, ch) & 0xffu));
        if(profile==0u) {
            uint32_t q=dv_r32(dv_ctrl(ctx,ch),0xb4u)>>21;
            uint8_t *p=all+(size_t)ch*0x17cu;
            dv_w32(p,0x1a0u,q); dv_w32(p,0x9f0u,q);
            dv_w32(p,0x1240u,q); dv_w32(p,0x1a90u,q);
        }
        ddr_capture_channel_training_state(ctx,ch,snap,work);
    }
    for(unsigned i=0;i<4u;i++) {
        uint32_t ch=qrk3588_ddr_ctx_route(ctx, i); if(ch==0xfu)continue;
        struct qrk3588_ddr_dvfs_channel_snapshot *dst =
            (struct qrk3588_ddr_dvfs_channel_snapshot *)(void *)
                (snap + 0x790u + (size_t)ch *
                 sizeof(struct qrk3588_ddr_dvfs_channel_snapshot));
        const struct qrk3588_ddr_channel_state *src =
            qrk3588_ddr_state_channel_view_const(work, ch);

        for (uint32_t word = 0u;
             word != QRK3588_DDR_STATE_PACKED_PROFILE_WORDS; ++word)
            dst->packed_profile[word] = src->packed_profile.word[word];
        for (uint32_t word = 0u;
             word != QRK3588_DDR_DVFS_SNAPSHOT_TAIL_WORDS; ++word)
            dst->opaque_tail[word] = src->private_tail[word];
        /* reserved_2c / channel private_tail[5] are intentionally not copied. */
    }
    dv_w32(snap,0u,UINT32_C(0xfead0004));
}

int phy_dvfs_config(uint32_t frequency_mhz, uint32_t profile)
{
    uint8_t *ctx = dv_ptr_slot(DVFS_CONTEXT_SLOT_ADDR);
    uint8_t *active = dv_ptr_slot(DVFS_ACTIVE_CFG_ADDR);
    uint8_t *work = dv_ptr_slot(DVFS_WORK_ADDR);
    uint8_t *dram_src = dv_ptr_slot(DVFS_DRAM_SRC_ADDR);
    uint8_t *freq_aux = dv_ptr_slot(DVFS_FREQ_AUX_ADDR);
    if (!ctx || !active || !work || !dram_src || profile > 3u)
        return DVFS_EINVAL;

    uint32_t dram_type = qrk3588_ddr_state_dram_type(active);
    uint8_t *descriptor=(uint8_t *)(uintptr_t)dv_r64(ctx,0x10u);
    uint32_t inverted=((dv_r32(descriptor,0x300u)&1u)^1u);
    int ok; uint32_t selector=dv_selector_from_profile(ctx,profile,&ok);
    if(!ok) return DVFS_EINVAL;
    if(dram_type==9u) dv_lp5_ordering_sync(ctx,active);

    ddr_memset(work, 0, sizeof(struct qrk3588_ddr_state_image));
    (void)ddr_build_frequency_params(work,dram_src,freq_aux,frequency_mhz,dram_type,0u);
    (void)ddr_memcpy(qrk3588_ddr_state_training_result(work, 0u),
                     qrk3588_ddr_state_training_result_const(active, 0u),
                     4u * sizeof(struct qrk3588_ddr_training_result));
    (void)ddr_memcpy(qrk3588_ddr_state_private_copy_region(work),
                     qrk3588_ddr_state_private_copy_region_const(active),
                     QRK3588_DDR_STATE_PRIVATE_COPY_SIZE);
    uint8_t *dram = (uint8_t *)qrk3588_ddr_state_dram_params(work);
    const uint8_t *freq_cfg = (const uint8_t *)freq_cfg_ptr(
        qrk3588_ddr_state_dram_type(work), dv_r32(dram, 0x2cu));

    for(unsigned i=0;i<4u;i++) {
        int rc=dv_switch_channel(ctx,active,work,freq_cfg,frequency_mhz,profile,selector,inverted,dram_type,i);
        if(rc!=0) return rc;
    }

    {
        int rc;

        rc = dv_run_config_passes(ctx, work, profile, selector);
        if (rc != 0)
            return rc;
        rc = dv_restore_training(ctx, work, frequency_mhz, profile, selector);
        if (rc != 0)
            return rc;
    }
    if(dram_type==9u) ddr_publish_phy_channel_status();
    dv_export_training_snapshot(ctx, work, profile, selector);
    return (int)UINT32_C(0xfead0004);
}
