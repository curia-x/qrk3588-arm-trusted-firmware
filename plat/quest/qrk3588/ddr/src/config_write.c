/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* DDR command, training-window and PHY deskew programming. */
#include "qrk3588_ddr_internal.h"


#define CW_EINVAL (-22)
#define CW_EIO    (-5)
#define CW_ETIMEDOUT (-110)

static inline uintptr_t cw_addr(const void *base, size_t off)
{
    return qrk3588_ddr_resolve_addr((uintptr_t)base + off);
}
static inline uint32_t cw_rd32(const void *base, size_t off)
{
    return *(volatile uint32_t *)(uintptr_t)cw_addr(base, off);
}
static inline uint64_t cw_rd64(const void *base, size_t off)
{
    return *(volatile uint64_t *)(uintptr_t)cw_addr(base, off);
}
static inline uint8_t cw_rd8(const void *base, size_t off)
{
    return *(volatile uint8_t *)(uintptr_t)cw_addr(base, off);
}
static inline void cw_wr32(void *base, size_t off, uint32_t value)
{
    *(volatile uint32_t *)(uintptr_t)cw_addr(base, off) = value;
}
static inline uint8_t *cw_slot(void *ctx, uint32_t channel)
{
    return qrk3588_ddr_ctx_channel_slot(ctx, channel);
}
static inline uint8_t *cw_primary(void *ctx, uint32_t channel)
{
    return (uint8_t *)(uintptr_t)cw_rd64(cw_slot(ctx, channel), 0u);
}
static inline uint8_t *cw_phy(void *ctx, uint32_t channel)
{
    return (uint8_t *)(uintptr_t)cw_rd64(cw_slot(ctx, channel), 8u);
}

static uint32_t cw_prepare_saved_trigger(void *ctx, uint32_t channel,
                                         uint8_t *state, uint32_t control,
                                         int *valid)
{
    uint8_t *primary = cw_primary(ctx, channel);
    uint8_t *phy = cw_phy(ctx, channel);
    uint32_t token = control & 0xffu;
    uint32_t freq;
    uint32_t v;

    *valid = 0;
    if (token != 0xffu && (control & 0x100u) == 0u)
        return 0u;

    uint32_t saved = ddr_phy_trigger_begin(primary);
    *valid = 1;
    freq = qrk3588_ddr_state_frequency(state);

    v = cw_rd32(phy, 0x04u) & ~0x00400000u;
    cw_wr32(phy, 0x04u, v);
    v = cw_rd32(phy, 0x50u) & ~0x10000000u;
    cw_wr32(phy, 0x50u, v);
    cw_wr32(phy, 0xb0u, cw_rd32(phy, 0xb0u) | 0x00800000u);

    v = cw_rd32(phy, 0x08u) & 0xf80fffffu;
    if (freq <= 1199u)
        v |= 0x00800000u;
    else if (freq <= 1599u)
        v |= 0x00400000u;
    else
        v |= 0x00100000u;
    cw_wr32(phy, 0x08u, v);
    return saved;
}

static int cw_wait_training_done(uint8_t *phy)
{
    return qrk3588_ddr_poll32((uintptr_t)(phy + 0xa24u), 2u, 2u, 0u);
}

static void cw_write_training_patterns(uint8_t *phy, uint32_t mode,
                                       uint32_t timing_freq,
                                       uint32_t dram_type)
{
    uint32_t v;

    if (timing_freq <= 532u) {
        cw_wr32(phy, 0x04u, cw_rd32(phy, 0x04u) | 0x04000000u);
        cw_wr32(phy, 0x14u, cw_rd32(phy, 0x14u) & 0xffffffc0u);
        cw_wr32(phy, 0x160u, cw_rd32(phy, 0x160u) & ~2u);
    } else {
        cw_wr32(phy, 0x04u, cw_rd32(phy, 0x04u) | 0x00020000u);
        v = cw_rd32(phy, 0x160u);
        if (timing_freq <= 1065u)
            v |= 3u;
        else
            v = (v & 0xfffffc00u) | 0x7eu;
        cw_wr32(phy, 0x160u, v);

        if (dram_type == 9u && timing_freq > 1065u) {
            cw_wr32(phy, 0xb0u, cw_rd32(phy, 0xb0u) | 0x00800000u);
            v = cw_rd32(phy, 0xb4u) & 0x7fcu;
            cw_wr32(phy, 0x160u, (cw_rd32(phy, 0x160u) & 0xfffffc03u) | v);
            cw_wr32(phy, 0xb0u, cw_rd32(phy, 0xb0u) & ~0x00800000u);
        }

        cw_wr32(phy, 0x14u, (cw_rd32(phy, 0x14u) & 0xfffff03fu) | 0x0a80u);
        cw_wr32(phy, 0x93cu, 0xaa55aa55u);
        cw_wr32(phy, 0x940u, 0xaa55aa55u);
        cw_wr32(phy, 0x944u, 0xaa55aa55u);
        cw_wr32(phy, 0x948u, 0xaa55aa55u);
        cw_wr32(phy, 0x94cu, 0xaa55aa55u);
        cw_wr32(phy, 0x950u, 0xaaaa5555u);
        cw_wr32(phy, 0x954u, 0xaaaa5555u);
        cw_wr32(phy, 0x958u, 0x5555aaaau);
        cw_wr32(phy, 0x95cu, 0xaa55aa55u);
        cw_wr32(phy, 0x960u, 0x55aa55aau);
        cw_wr32(phy, 0x964u, 0x55aa55aau);
        cw_wr32(phy, 0x968u, 0xaa55aa55u);
        cw_wr32(phy, 0x96cu, 0x00005555u);
        cw_wr32(phy, 0x970u, 0x0000aaaau);
        cw_wr32(phy, 0x974u, 0x00005aa5u);
    }

    cw_wr32(phy, 0x98u, 0x00ff55aau);
    cw_wr32(phy, 0x9cu, 0xff00aa55u);
    cw_wr32(phy, 0xa0u, 0x00ff55aau);
    cw_wr32(phy, 0xa4u, 0xff00aa55u);
    cw_wr32(phy, 0xa8u, 0x00005aa5u);

    v = cw_rd32(phy, 0xa24u) | 0x40u;
    v &= ~0xc000u;
    if (mode == 3u)
        v |= 0xc000u;
    else
        v |= 0x4000u << mode;
    cw_wr32(phy, 0xa24u, v | 1u);
}

static void cw_program_deskew_from_capture(uint8_t *phy, uint32_t timing_freq)
{
    volatile uint32_t *saved = DDR_LAYOUT_PTR(volatile uint32_t, QRK3588_DDR_CONFIG_WRITE_SAVED_VA);
    uint32_t lane_scale;
    uint32_t half_scale;
    uint32_t bitpos = 0u;

    if (timing_freq > 532u)
        return;

    /* Save ten raw lane taps using the hardware field layout. */
    uint32_t base = cw_rd32(phy, 0xb0u) & 1u;
    uint32_t tap = cw_rd32(phy, 0xb4u) & 0x7ffu;
    half_scale = tap >> 1;
    lane_scale = base == 0u ? half_scale : tap;
    for (uint32_t lane = 0; lane < 10u; ++lane) {
        size_t src = lane == 9u ? 0x254u : 0x1f0u + lane * 12u;
        saved[lane] = cw_rd32(phy, src);
    }

    cw_wr32(phy, 0x04u, cw_rd32(phy, 0x04u) | 0x20u);
    cw_wr32(phy, 0x10u, cw_rd32(phy, 0x10u) | 0x200u);

    for (uint32_t lane = 0; lane < 10u; ++lane) {
        uint32_t raw = saved[lane];
        uint32_t lo = (raw & 0x3ffu) + half_scale;
        uint32_t hi = ((raw >> 16) & 0x3ffu) + half_scale;
        uint32_t qlo = lane_scale ? lo / lane_scale : 0u;
        uint32_t qhi = lane_scale ? hi / lane_scale : 0u;
        uint32_t rlo = lane_scale ? lo % lane_scale : lo;
        uint32_t rhi = lane_scale ? hi % lane_scale : hi;
        size_t dst = 0x8b0u + lane * 4u;
        cw_wr32(phy, dst, raw);
        cw_wr32(phy, dst + 0x60u, raw);
        cw_wr32(phy, dst - 0x30u, rlo | (rhi << 16));
        cw_wr32(phy, dst + 0x30u, rlo | (rhi << 16));

        uint32_t mask = ~(7u << bitpos);
        cw_wr32(phy, 0xdd0u, (cw_rd32(phy, 0xdd0u) & mask) | (qlo << bitpos));
        cw_wr32(phy, 0xdd8u, (cw_rd32(phy, 0xdd8u) & mask) | (qlo << bitpos));
        cw_wr32(phy, 0xdd4u, (cw_rd32(phy, 0xdd4u) & mask) | (qhi << bitpos));
        cw_wr32(phy, 0xddcu, (cw_rd32(phy, 0xddcu) & mask) | (qhi << bitpos));
        bitpos += 3u;
    }
    ddr_phy_update_toggle((uintptr_t)phy);
    cw_wr32(phy, 0x10u, cw_rd32(phy, 0x10u) & ~0x200u);
    cw_wr32(phy, 0x14u, (cw_rd32(phy, 0x14u) & 0xffffffc0u) | 0x2fu);
    cw_wr32(phy, 0x04u, cw_rd32(phy, 0x04u) & ~0x04000000u);
    for (uint32_t lane = 0; lane < 10u; ++lane) {
        size_t dst = 0x8b0u + lane * 4u;
        cw_wr32(phy, dst, 0u);
        cw_wr32(phy, dst + 0x60u, 0u);
        cw_wr32(phy, dst - 0x30u, 0u);
        cw_wr32(phy, dst + 0x30u, 0u);
    }
    cw_wr32(phy, 0xdd0u, 0u);
    cw_wr32(phy, 0xdd4u, 0u);
    cw_wr32(phy, 0xdd8u, 0u);
    cw_wr32(phy, 0xddcu, 0u);
    ddr_phy_update_toggle((uintptr_t)phy);

}

static int cw_bit0_register_capture_stage(void *ctx, uint32_t channel,
                                          uint8_t *state)
{
    uint8_t *primary = cw_primary(ctx, channel);
    uint8_t *phy = cw_phy(ctx, channel);
    uint8_t *dram = (uint8_t *)qrk3588_ddr_state_dram_params(state);
    uint8_t *timing = (uint8_t *)qrk3588_ddr_state_timing(state);
    uint32_t *dram_type = qrk3588_ddr_state_dram_type_ptr(state);
    uint32_t type = qrk3588_ddr_state_dram_type(state);
    uint32_t frequency = qrk3588_ddr_state_frequency(state);

    if (type == 9u) {
        uint32_t scale = cw_rd32(dram, 0x2cu);
        uint32_t width = cw_rd32(dram, 0x60u);
        uint32_t tap = cw_rd32(phy, 0xb4u) & 0x7ffu;
        volatile uint32_t *table = DDR_LAYOUT_PTR(
            volatile uint32_t, QRK3588_DDR_MODE_SCRATCH_TABLE_VA);

        scale >>= width == 0x80u ? 1u : 2u;
        scale = (scale * 5u + 999u) / 1000u;
        cw_wr32(phy, 0xa70u, (cw_rd32(phy, 0xa70u) & 0xfc3fffffu) | 0x00c00000u);
        cw_wr32(phy, 0x608u,
                (cw_rd32(phy, 0x608u) & 0xff00003fu) | 0x00800000u |
                (scale << 6) | (scale << 11));
        cw_wr32(phy, 0x600u, cw_rd32(phy, 0x600u) | 0x80000000u);
        cw_wr32(phy, 0x18u, cw_rd32(phy, 0x18u) | 0x2000u);

        (void)phy_reg_rw_seq(ctx, channel, 0u, state);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        (void)phy_reg_rw_seq(ctx, channel, 1u, state);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();

        {
            const uint32_t half = tap >> 1;
            const uint32_t doubled = tap << 1;
            for (uint32_t i = 0u; i != 7u; ++i) {
                uint32_t reference = table[i];
                uint32_t measured = table[7u + i];
                uint32_t base = width == 0x80u ? tap : doubled;
                uint32_t center = width == 0x80u ? half : tap;
                uint32_t delta;
                uint32_t value;

                delta = base - measured;
                if (delta > reference)
                    value = center - ((delta - reference) >> 1);
                else
                    value = center + ((reference - delta) >> 1);
                table[14u + i] = value;
            }
        }

        cw_wr32(phy, 0x7cu, table[14u] & 0x3ffu);
        cw_wr32(phy, 0x80u, table[16u] & 0x3ffu);
        cw_wr32(phy, 0x84u, table[18u] & 0x3ffu);
        cw_wr32(phy, 0x8cu, table[20u] & 0x3ffu);
        cw_wr32(phy, 0x60cu, cw_rd32(phy, 0x60cu) | 1u);
        ddr_phy_update_toggle((uintptr_t)phy);
        cw_wr32(phy, 0x18u, cw_rd32(phy, 0x18u) & ~0x2000u);
        cw_wr32(phy, 0x600u, cw_rd32(phy, 0x600u) & 0x7fffffffu);
        ddr_phy_reg_read(ctx, channel, dram_type, 0u, frequency << 1);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        return 0;
    }

    {
        uint32_t tap = cw_rd32(phy, 0xb4u) & 0x7ffu;
        uint32_t tap_scale = (cw_rd32(phy, 0xb0u) & 1u) != 0u ? tap << 1 : tap;
        uint32_t cycle = cw_rd32(timing, 0u);
        uint32_t denominator;
        uint32_t code;
        uint32_t pair;
        uint32_t mr13;

        /* A zero divisor yields a zero quotient in the firmware arithmetic
         * contract; model that edge explicitly instead of evaluating C division
         * by zero or inventing an error return. */
        denominator = cycle != 0u ? UINT32_C(1000000000) / cycle : 0u;
        denominator = tap_scale != 0u ? denominator / tap_scale : 0u;
        code = denominator != 0u ? UINT32_C(91000) / denominator : 0u;
        pair = code | (code << 16);

        ddr_phy_reg_read(ctx, channel, dram_type, 1u, 24u);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        cw_wr32(phy, 0x18u, cw_rd32(phy, 0x18u) & ~0x4000u);
        cw_wr32(phy, 0x04u, cw_rd32(phy, 0x04u) | 0x00200000u);
        cw_wr32(phy, 0x10u, cw_rd32(phy, 0x10u) | 0x80u);
        cw_wr32(phy, 0x6cu, pair);
        ddr_phy_update_toggle((uintptr_t)phy);
        cw_wr32(phy, 0x10u, cw_rd32(phy, 0x10u) & ~0x80u);

        mr13 = cw_rd32(primary, 0x504u) & 0xffffu;
        (void)ddr_phy_reg_write((uint32_t)(uintptr_t)primary, 1u, 13u, mr13 | 1u);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        cw_wr32(phy, 0x04u, cw_rd32(phy, 0x04u) | 4u);
        cw_wr32(phy, 0x600u, (cw_rd32(phy, 0x600u) & 0xf9ffffffu) | 0x02000000u);
        udelay(1u);
        cw_wr32(phy, 0x60cu, cw_rd32(phy, 0x60cu) & 0xff3fffffu);
        udelay(1u);
        ddr_phy_reg_read(ctx, channel, dram_type, 0u, frequency << 1);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();

        cw_wr32(phy, 0x600u,
                (cw_rd32(phy, 0x600u) & 0xffffff00u) |
                (cw_rd32(primary, 0x508u) & 0xffffu));
        cw_wr32(phy, 0x10u, cw_rd32(phy, 0x10u) | 0x200u);
        cw_wr32(phy, 0x74u, (cw_rd32(phy, 0x74u) & ~3u) | 1u);
        cw_wr32(phy, 0x600u,
                (cw_rd32(phy, 0x600u) & 0xff9fffffu) | 0x00200000u);
        udelay(1u);
        cw_wr32(phy, 0x600u, cw_rd32(phy, 0x600u) & 0xff9fffffu);
        cw_wr32(phy, 0x74u, cw_rd32(phy, 0x74u) & ~3u);
        cw_wr32(phy, 0x10u, cw_rd32(phy, 0x10u) & ~0x200u);
        udelay(1u);

        cw_wr32(phy, 0x600u, cw_rd32(phy, 0x600u) | 0x80000000u);
        cw_wr32(phy, 0x8cu, code << 16);
        cw_wr32(phy, 0x90u, pair);
        ddr_phy_update_toggle((uintptr_t)phy);
        cw_wr32(phy, 0x600u, cw_rd32(phy, 0x600u) & 0x7fffffffu);
        cw_wr32(phy, 0x18u, cw_rd32(phy, 0x18u) | 0x2040u);
        cw_wr32(phy, 0x600u, cw_rd32(phy, 0x600u) | 0x800u);
        cw_wr32(phy, 0x60cu, cw_rd32(phy, 0x60cu) | 2u);

        for (uint32_t wait = 0u; wait != 100000u; ++wait) {
            if ((cw_rd32(phy, 0xbc4u) & (1u << 25)) != 0u)
                break;
            udelay(1u);
        }

        ddr_phy_reg_read(ctx, channel, dram_type, 1u, 24u);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        cw_wr32(phy, 0x18u, cw_rd32(phy, 0x18u) & ~0x2040u);
        cw_wr32(phy, 0x60cu, cw_rd32(phy, 0x60cu) | 0x00c00000u);
        udelay(1u);
        cw_wr32(phy, 0x600u, cw_rd32(phy, 0x600u) & 0xf9ffffffu);
        cw_wr32(phy, 0x04u, cw_rd32(phy, 0x04u) & ~4u);
        cw_wr32(phy, 0x60cu, cw_rd32(phy, 0x60cu) & ~2u);
        cw_wr32(phy, 0x18u, cw_rd32(phy, 0x18u) | 0x4040u);
        (void)ddr_phy_reg_write((uint32_t)(uintptr_t)primary, 1u, 13u, code);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        ddr_phy_reg_read(ctx, channel, dram_type, 0u, frequency << 1);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();

        if ((cw_rd32(phy, 0xbc4u) & 0x0001fe01u) != 0u)
            return -1;
    }
    return 0;
}

static int cw_bit1_lane_calibration_stage(void *ctx, uint32_t channel,
                                          uint32_t mode)
{
    return ddr_phy_lane_calibrate(ctx, channel, mode);
}

static int cw_bit2_phy_setup_stage(void *ctx, uint32_t channel, uint32_t mode,
                                   uint8_t *state)
{
    uint8_t *phy = cw_phy(ctx, channel);
    uint32_t frequency = qrk3588_ddr_state_frequency(state);
    uint32_t dram_type = qrk3588_ddr_state_dram_type(state);
    uint32_t v;
    uint32_t delay_code;

    v = cw_rd32(phy, 0xa24u) | 8u;
    v &= ~0x300u;
    v |= mode == 3u ? 0x300u : (0x100u << (mode & 31u));
    cw_wr32(phy, 0xa24u, v);

    v = (cw_rd32(phy, 0x70u) & 0xfc0fffffu) | 0x02800000u;
    delay_code = (frequency * 20u + 999u) / 1000u;
    if (delay_code < 2u)
        delay_code = 2u;
    v = (v & 0xfff03fffu) | (delay_code << 14);
    cw_wr32(phy, 0x70u, v);

    cw_wr32(phy, 0xa24u, cw_rd32(phy, 0xa24u) | 1u);
    if (qrk3588_ddr_poll32((uintptr_t)(phy + 0xa24u), 2u, 2u, 0u) != 0)
        return CW_ETIMEDOUT;
    cw_wr32(phy, 0xa24u, cw_rd32(phy, 0xa24u) & ~1u);
    cw_wr32(phy, 0xa24u, cw_rd32(phy, 0xa24u) & ~0x300u);
    cw_wr32(phy, 0xa24u, cw_rd32(phy, 0xa24u) & ~8u);
    if ((cw_rd32(phy, 0x74u) & 0x30u) != 0u)
        return -1;
    (void)dram_type;
    return 0;
}
__attribute__((noinline))
static int cw_bit3_pattern_stage(void *ctx, uint32_t channel, uint32_t mode,
                                 uint8_t *state, uint32_t arg4,
                                 uint32_t token)
{
    uint8_t *phy = cw_phy(ctx, channel);
    uint8_t *dram = (uint8_t *)qrk3588_ddr_state_dram_params(state);
    uint8_t *timing = (uint8_t *)qrk3588_ddr_state_timing(state);
    uint32_t timing_freq = cw_rd32(timing, 0u);
    uint32_t dram_type = qrk3588_ddr_state_dram_type(state);
    uint32_t select = cw_rd32(phy, 0xb8u) >> 30;
    uint32_t feature;

    (void)arg4;
    if (select == 0u)
        feature = (cw_rd32(phy, 0x04u) >> 29) & 1u;
    else
        feature = (cw_rd32(phy, 0xbcu + (size_t)(select - 1u) * 4u) >> 11) & 1u;

    if (timing_freq <= 532u || feature == 0u) {
        if (mode == 1u)
            return 0;
        if (mode == 3u)
            mode = 0u;
    }

    if (token != 1u) {
        cw_write_training_patterns(phy, mode, timing_freq, dram_type);
        if (token == 0u)
            return 0;
    }

    if (cw_wait_training_done(phy) != 0)
        return CW_ETIMEDOUT;
    cw_wr32(phy, 0xa24u, cw_rd32(phy, 0xa24u) & ~1u);
    cw_wr32(phy, 0xa24u, cw_rd32(phy, 0xa24u) & ~0xc000u);
    cw_wr32(phy, 0xa24u, cw_rd32(phy, 0xa24u) & ~0x40u);

    if (timing_freq <= 532u) {
        cw_program_deskew_from_capture(phy, timing_freq);
    } else if (timing_freq <= 1065u) {
        uint32_t mask = mode == 3u ? 3u : (1u << mode);
        (void)ddr_training_mirror_entry(1u, channel, 0u, mask);
    }

    (void)dram;
    if ((cw_rd32(phy, 0xe8u) & 0xffffu) != 0u)
        return -1;
    if ((cw_rd32(phy, 0xecu) & 3u) != 0u)
        return -1;
    return 0;
}
#include <common/debug.h>
static int cw_bit4_training_search(void *ctx, uint32_t channel, uint32_t mode,
                                   uint8_t *state, uint32_t arg4, uint32_t profile,
                                   uint32_t arg7, uint32_t token)
{
    uint8_t *timing = (uint8_t *)qrk3588_ddr_state_timing(state);
    uint8_t *dram = (uint8_t *)qrk3588_ddr_state_dram_params(state);
    uint8_t *primary = cw_primary(ctx, channel);
    uint8_t *phy = cw_phy(ctx, channel);
    uint32_t dram_type = qrk3588_ddr_state_dram_type(state);
    uint32_t freq = cw_rd32(timing, 0u);
    uint32_t width = cw_rd32(dram, 0x60u);

    if (mode != 0u && mode != 3u) {
        int rc = ddr_phy_command((const uint32_t *)timing, ctx, channel, mode, token);
        return (rc != 0 || token != 0u) ? rc : 0;
    }
    if (arg7 == 0u || token == 1u)
        return ddr_phy_command((const uint32_t *)timing, ctx, channel, mode, token);

    uint32_t freq_unit = freq;
    if (dram_type == 9u)
        freq_unit = width == 128u ? (freq >> 1) : (freq >> 2);

    uint32_t tap_word;
    uint32_t reg_off;
    if (arg4 != 0u) {
        tap_word = (cw_rd32(phy, 0xbcu + (size_t)(arg4 - 1u) * 4u) >> 14) & 0x3fu;
        reg_off = 0xa2cu + arg4 * 16u;
    } else {
        tap_word = cw_rd32(phy, 0u) & 0x3fu;
        reg_off = 0xa2cu;
    }

    uint32_t global_seed = DDR_LAYOUT_REF(uint32_t, QRK3588_DDR_CONFIG_WRITE_SEED_VA);
    uint32_t current;
    uint32_t minimum;
    uint32_t maximum;

    if (global_seed != 0u) {
        uint32_t q = freq ? (0x0040d990u / freq) : 0u;
        /* Arithmetic is intentionally 32-bit; wraparound is part of the
         * seeded hardware-programming path, which skips the live sweep. */
        uint32_t scaled = (uint32_t)((q + global_seed) * freq_unit);
        current = scaled / 0x000f4240u;
        minimum = current;
        maximum = current;
    } else {
        current = (tap_word & 1u) ? 1u : 2u;
        minimum = 0xffu;
        maximum = 0u;

        uint32_t shift6 = current << 6;
        uint32_t lane_bias = current + tap_word;
        uint32_t enc2 = lane_bias << 1;
        uint32_t enc4 = lane_bias << 2;
        uint8_t *state_row = state + (size_t)channel * 20u;

        for (; current <= 18u; current += 2u, shift6 += 0x80u,
             enc2 += 4u, enc4 += 8u) {
            uint32_t v = cw_rd32(phy, reg_off);
            cw_wr32(phy, reg_off, (v & 0xfffff83fu) | shift6);
            int ok = 1;
            if (freq > 799u) {
                ok = ddr_phy_command((const uint32_t *)timing, ctx, channel,
                                     0u, 0xffu) == 0;
                /* A Native bounded-wait timeout is sticky and represents a
                 * point where frozen firmware would still be blocked.  Do not
                 * continue sweeping taps and issuing MMIO after that point;
                 * ordinary command/training failures remain nonfatal samples. */
                if (qrk3588_ddr_get_error() != 0)
                    return qrk3588_ddr_get_error();
            } else {
                uint32_t expected;
                uint32_t encoded;

                if (dram_type == 9u) {
                    expected = cw_rd8(state_row, 0x1d9u);
                    encoded = width == 128u ? enc2 : enc4;
                } else {
                    expected = cw_rd32(primary,
                            (size_t)profile * 0x100000u + 0x508u) & 0xffffu;
                    encoded = tap_word - 3u + current;
                }

                cw_wr32(primary, 0x10c80u, 0u);
                size_t image_off = 0x580u + (size_t)profile * 0x100000u;
                uint32_t image = cw_rd32(primary, image_off);
                cw_wr32(primary, image_off,
                        (image & 0xffc0ffffu) | (encoded << 16));
                ddr_phy_update_complete((uint32_t)(uintptr_t)primary);
                if (qrk3588_ddr_get_error() != 0)
                    return qrk3588_ddr_get_error();
                {
                    uint32_t probe = ddr_phy_config_block(primary, 1u, 12u);
                    if (qrk3588_ddr_get_error() != 0)
                        return qrk3588_ddr_get_error();
                    ok = probe == expected;
                }
            }

            if (ok) {
                if (current < minimum) minimum = current;
                if (current > maximum) maximum = current;
            }
        }
    }

    if (minimum == 0xffu || maximum == 0u) {
        return -1;
    }

    uint32_t selected = minimum + 2u;
    uint32_t upper = maximum <= 18u ? maximum : 18u;
    if (upper <= selected)
        selected = upper;
    if (((tap_word + selected) & 1u) != 0u)
        --selected;

    uint32_t v = cw_rd32(phy, reg_off);
    cw_wr32(phy, reg_off, (v & 0xfffff83fu) | (selected << 6));
    cw_wr32(phy, 0x688u, (cw_rd32(phy, 0x688u) & 0xfffffe1fu) | 0x100u);

    {
        uint32_t encoded;
        if (dram_type == 9u) {
            uint32_t merged = tap_word + selected;
            encoded = width == 128u ? (merged << 1) : (merged << 2);
        } else {
            encoded = tap_word - 3u + selected;
        }
        cw_wr32(primary, 0x10c80u, 0u);
        size_t image_off = 0x580u + (size_t)profile * 0x100000u;
        cw_wr32(primary, image_off,
                (cw_rd32(primary, image_off) & 0xffc0ffffu) | (encoded << 16));
        ddr_phy_update_complete((uint32_t)(uintptr_t)primary);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
    }

    return ddr_phy_command((const uint32_t *)timing, ctx, channel, mode, token);
}

static void cw_finish_cleanup(void *ctx, uint32_t channel, uint8_t *state,
                              uint32_t arg4, int saved_valid,
                              uint32_t saved_trigger)
{
    uint8_t *primary = cw_primary(ctx, channel);
    uint8_t *phy = cw_phy(ctx, channel);
    uint32_t timing_freq = cw_rd32(qrk3588_ddr_state_timing(state), 0u);

    cw_wr32(phy, 0xb0u, cw_rd32(phy, 0xb0u) & ~0x00800000u);
    cw_wr32(phy, 0x160u, cw_rd32(phy, 0x160u) & ~1u);
    if (timing_freq > 532u || arg4 != 0u)
        cw_wr32(phy, 0x04u, cw_rd32(phy, 0x04u) | 0x00400000u);
    ddr_phy_update_toggle((uintptr_t)phy);
    /* Cleanup always performs the restore call.  Without a save preamble the
     * saved slot remains zero, so the same call contract is preserved. */
    (void)saved_valid;
    ddr_phy_trigger_end(primary, saved_trigger);
}

int ddr_config_write_seq(void *ctx, uint32_t channel, uint32_t mode,
                                uint8_t *state, uint32_t arg4, uint32_t arg5,
                                uint32_t operation_flags, uint32_t arg7,
                                uint32_t control)
{
    uint32_t token = control & 0xffu;
    int saved_valid = 0;
    uint32_t saved_trigger = 0u;
    int rc = 0;

    if (channel >= 4u || ctx == NULL || state == NULL)
        return CW_EINVAL;

    saved_trigger = cw_prepare_saved_trigger(ctx, channel, state, control,
                                             &saved_valid);

    if ((operation_flags & 0x01u) != 0u) {
        rc = cw_bit0_register_capture_stage(ctx, channel, state);
        if (rc != 0)
            return rc;
    }

    if ((operation_flags & 0x02u) != 0u) {
        rc = cw_bit1_lane_calibration_stage(ctx, channel, mode);
        if (rc != 0)
            return rc;
    }

    if ((operation_flags & 0x04u) != 0u) {
        rc = cw_bit2_phy_setup_stage(ctx, channel, mode, state);
        if (rc != 0)
            return rc;
    }

    if ((operation_flags & 0x10u) != 0u) {
        rc = cw_bit4_training_search(ctx, channel, mode, state, arg4,
                                     arg5, arg7, token);
        if (rc != 0)
            return rc;
        if (token == 0u)
            return 0;
    }

    if ((operation_flags & 0x08u) != 0u) {
        rc = cw_bit3_pattern_stage(ctx, channel, mode, state, arg4, token);
        if (rc != 0)
            return rc;
    }

    if (token == 0xffu || (control & 0x200u) != 0u)
        cw_finish_cleanup(ctx, channel, state, arg4, saved_valid, saved_trigger);

    return rc;
}
