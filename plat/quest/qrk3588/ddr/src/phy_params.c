/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* DRAM-frequency dependent PHY parameter staging.
 *
 * Selects frequency profiles, normalizes per-channel limits, programs live or
 * banked controller timing fields, encodes PHY dividers and builds the active
 * PHY image.  Unnamed layout fields remain offset-based where no stable public
 * field name is available.
 */
#include <stddef.h>
#include <stdint.h>

#include "qrk3588_ddr_internal.h"
#include "qrk3588_ddr_runtime.h"

#define LPDDR4   7u
#define LPDDR4X  8u
#define LPDDR5   9u

static inline uintptr_t pa(const void *base, size_t off)
{
    return qrk3588_ddr_resolve_addr((uintptr_t)base + off);
}
static inline uint8_t pr8(const void *base, size_t off)
{
    return *(volatile const uint8_t *)(uintptr_t)pa(base, off);
}
static inline uint16_t pr16(const void *base, size_t off)
{
    return *(volatile const uint16_t *)(uintptr_t)pa(base, off);
}
static inline uint32_t pr32(const void *base, size_t off)
{
    return *(volatile const uint32_t *)(uintptr_t)pa(base, off);
}
static inline void pw32(void *base, size_t off, uint32_t value)
{
    *(volatile uint32_t *)(uintptr_t)pa(base, off) = value;
}
static inline uint32_t udiv0(uint32_t num, uint32_t den)
{
    return den != 0u ? num / den : 0u;
}
static inline uint32_t clamp_min(uint32_t v, uint32_t lo)
{
    return v < lo ? lo : v;
}
static inline uint32_t clamp_range(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint32_t limit_to_table_index(uint32_t limit)
{
    const uint16_t *lookup = DDR_LAYOUT_REF(const uint16_t *, QRK3588_DDR_LOOKUP_SLOT_VA);
    if (lookup == 0)
        return limit;
    for (uint32_t i = 0u; i != 64u; ++i) {
        if (limit <= pr16(lookup, (size_t)i * 2u))
            return i;
    }
    /* Retain the pre-scan value if no table entry matches. */
    return limit;
}

static void program_live_controller_fields(uint8_t *controller,
                                           uint8_t *state,
                                           uint32_t dram_type,
                                           uint32_t direct_delta,
                                           uint32_t table_index)
{
    const struct qrk3588_ddr_phy_divider_state *div =
        qrk3588_ddr_state_phy_dividers_const(state);
    const uint32_t div_lane2 = div->lane2;
    const uint32_t div_lane1 = div->lane1;
    const uint32_t div_delta = div->delta;
    const uint32_t div_lane0 = div->lane0;
    const uint32_t div_delta_live = div->delta;
    uint32_t v;

    (void)div_delta;

    v = pr32(controller, 0x3c8u) & UINT32_C(0x08ffffff);
    v |= (div_lane1 << 24) | (div_lane1 << 28);
    if (dram_type != LPDDR5)
        v |= UINT32_C(0x80000000);
    pw32(controller, 0x3c8u, v);

    v = pr32(controller, 0x3d4u) & UINT32_C(0xffc0c0c0);
    v |= div_lane0 | (div_lane0 << 3) | (div_lane0 << 8) | (div_lane0 << 11);
    v |= (div_lane2 << 16) | (div_lane2 << 19);
    pw32(controller, 0x3d4u, v);

    pw32(controller, 0x3dcu, pr32(controller, 0x3dcu) | UINT32_C(0x073f0000));

    /* Frozen 032a0 is a two-stage live-divider transaction:
     *   3e0 base image write -> 3e0 reread -> 0x202 update -> 3e0 write
     *   -> 0x0c bit30 update.
     * Collapsing the two 3e0 writes is final-value equivalent in a memory
     * model but not equivalent to the controller's transaction protocol. */
    v = pr32(controller, 0x3e0u) & UINT32_C(0xffffc7c7);
    v |= (div_delta_live << 3) | (div_delta_live << 11);
    pw32(controller, 0x3e0u, v);

    v = pr32(controller, 0x3e0u);
    if (direct_delta != 0u) {
        v &= UINT32_C(0xfffffdfd);
        pw32(controller, 0x3e0u, v);
        pw32(controller, 0x0cu,
             pr32(controller, 0x0cu) & UINT32_C(0xbfffffff));
    } else {
        v |= UINT32_C(0x202);
        pw32(controller, 0x3e0u, v);
        pw32(controller, 0x0cu,
             pr32(controller, 0x0cu) | UINT32_C(0x40000000));
    }

    v = pr32(controller, 0x3ecu) & UINT32_C(0xffffc0c0);
    v |= table_index | (table_index << 8);
    pw32(controller, 0x3ecu, v);
}

static void program_banked_controller_fields(uint8_t *controller,
                                             uint8_t *state,
                                             uint32_t mode,
                                             uint32_t direct_delta,
                                             uint32_t table_index)
{
    const uint32_t bank = mode * 4u - 4u;
    const struct qrk3588_ddr_phy_divider_state *div =
        qrk3588_ddr_state_phy_dividers_const(state);
    const uint32_t div_lane2 = div->lane2;
    const uint32_t div_lane1 = div->lane1;
    const uint32_t div_delta = div->delta;
    uint32_t v;

    {
        const uint32_t div_lane0 = div->lane0;
        v = div_lane0 | (div_lane0 << 3) | (div_lane0 << 6) | (div_lane0 << 9);
    }
    v |= (table_index << 20) | (table_index << 26);
    v |= (div_lane1 << 12) | (div_lane1 << 15);
    pw32(controller, (size_t)bank + 0xd4u, v);

    v = pr32(controller, (size_t)bank + 0xdcu) & UINT32_C(0xffc0ffff);
    v |= (div_delta << 16) | (div_delta << 19);
    pw32(controller, (size_t)bank + 0xdcu, v);

    v = pr32(controller, (size_t)bank + 0xa00u) & UINT32_C(0xffc0ffff);
    v |= (div_lane2 << 16) | (div_lane2 << 19);
    pw32(controller, (size_t)bank + 0xa00u, v);

    v = pr32(controller, (size_t)bank + 0xccu);
    if (direct_delta != 0u)
        v &= UINT32_C(0xdfffffff);
    else
        v |= UINT32_C(0x20000000);
    pw32(controller, (size_t)bank + 0xccu, v);

    v = pr32(controller, 0x3e0u);
    if (direct_delta != 0u)
        v &= UINT32_C(0xfffffdfd);
    else
        v |= UINT32_C(0x202);
    pw32(controller, 0x3e0u, v);
}

void ddr_phy_frequency_parameter_stage(void *context, uint32_t channel,
                                       uint8_t *state, uint32_t profile,
                                       uint32_t mode,
                                       uint32_t use_override)
{
    uint8_t *controller = (uint8_t *)qrk3588_ddr_ctx_channel_base1(context, channel);
    uint8_t *phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(context, channel);
    const uint32_t dram_type = qrk3588_ddr_state_dram_type(state);
    const uint32_t frequency = qrk3588_ddr_state_frequency(state);
    const uint8_t *type_cfg = (const uint8_t *)dram_type_cfg_ptr(dram_type);
    const uint8_t *freq_cfg;
    uint32_t packed;
    uint32_t type14;
    uint32_t type24;
    uint32_t lane0, lane1, lane2, period_base;
    uint32_t direct_delta;
    uint32_t live_flag;
    uint32_t dynamic_period;
    uint32_t high_period;
    uint32_t lp5_threshold_div = 0u;
    uint32_t lp5_aux_div = 0u;
    uint32_t final_aux;
    uint32_t table_index;
    uint32_t mhz_a, mhz_b;
    uint32_t enc_a, enc_b;
    uint32_t v;

    if (type_cfg == 0)
        return;
    freq_cfg = (const uint8_t *)freq_cfg_ptr(dram_type, frequency);

    type14 = pr32(type_cfg, 0x14u);
    type24 = pr32(type_cfg, 0x24u);
    if (frequency > (pr32(type_cfg, 0x18u) & 0xfffu)) {
        packed = pr32(type_cfg, 0x0cu);
        dynamic_period = type14 & 0xffu;
    } else {
        packed = pr32(type_cfg, 0x10u);
        dynamic_period = 0u;
    }

    lane0 = packed & 0xffu;
    lane1 = (packed >> 8) & 0xffu;
    lane2 = (packed >> 16) & 0xffu;
    if (pr8(freq_cfg, 2u) != 0u) {
        period_base = pr32(type_cfg, 0x10u) >> 24;
        live_flag = (type14 >> 27) & 1u;
        direct_delta = 0u;
    } else {
        period_base = pr32(type_cfg, 0x0cu) >> 24;
        live_flag = (type14 >> 26) & 1u;
        direct_delta = (type14 >> 8) & 0x3ffu;
    }
    high_period = frequency > (type24 & 0xfffu)
                    ? ((type14 >> 18) & 0xffu) : 0u;

    if (direct_delta != 0u) {
        uint32_t override = 0u;
        if (use_override != 0u)
            override = qrk3588_ddr_state_training_result_const(state, channel)
                ->direct0_rank0;
        v = override != 0u ? override : (pr32(type_cfg, 0x2cu) & 0x3ffu);
    } else {
        v = pr32(type_cfg, 0x30u) & 0x3ffu;
    }
    v = clamp_range(v, 0xa6u, 0x2bbu);
    *qrk3588_ddr_state_limit_code(state, channel) = v;

    final_aux = direct_delta;
    if (dram_type == LPDDR5) {
        uint32_t pre_aux = 0u;
        uint32_t threshold_period = 0u;
        const uint32_t channel_mode = qrk3588_ddr_state_channel_rank_count(state, channel);

        if (frequency > ((type24 >> 12) & 0xfffu))
            threshold_period = type24 >> 24;
        if (dynamic_period != 0u)
            pre_aux = pr8(type_cfg, 0x2bu);

        if (threshold_period != 0u) {
            uint32_t scaled = threshold_period;
            if (channel_mode == 2u)
                scaled <<= 1;
            scaled = clamp_range(scaled, 40u, 240u);
            lp5_threshold_div = udiv0(240u, scaled);
        }
        if (pre_aux != 0u) {
            const uint32_t scaled = clamp_min(pre_aux, 40u);
            lp5_aux_div = udiv0(240u, scaled);
        }

        if (channel_mode == 2u && pre_aux != 0u) {
            if (dynamic_period != 0u)
                dynamic_period = udiv0(dynamic_period * pre_aux,
                                       pre_aux - dynamic_period);
            if (direct_delta != 0u) {
                final_aux = direct_delta;
                direct_delta = udiv0(pre_aux * direct_delta,
                                     pre_aux - direct_delta);
            } else {
                final_aux = pre_aux;
            }
        } else {
            final_aux = direct_delta;
        }
    }

    {
        struct qrk3588_ddr_phy_divider_state *div =
            qrk3588_ddr_state_phy_dividers(state);
        div->lane2 = (udiv0(240u, lane2) - 1u) & 7u;
        div->lane1 = (udiv0(240u, lane1) - 1u) & 7u;
        div->lane0 = (udiv0(240u, lane0) - 1u) & 7u;
        div->delta = direct_delta != 0u ? (udiv0(240u, direct_delta) & 7u) : 0u;
    }

    period_base = clamp_range(period_base, 40u, 240u);
    const uint32_t period_base_div = udiv0(240u, period_base);
    const uint32_t dynamic_div = dynamic_period != 0u
        ? udiv0(240u, clamp_min(dynamic_period, 40u)) : 0u;
    const uint32_t high_div = high_period != 0u
        ? udiv0(240u, clamp_min(high_period, 40u)) : 0u;

    table_index = limit_to_table_index(
        *qrk3588_ddr_state_limit_code_const(state, channel));
    *qrk3588_ddr_state_limit_code(state, channel) = table_index;

    if (mode == 0u)
        program_live_controller_fields(controller, state, dram_type, direct_delta, table_index);
    else
        program_banked_controller_fields(controller, state, mode, direct_delta, table_index);
    *qrk3588_ddr_state_private_status(state, channel) = 0u;

    if (use_override != 0u)
        mhz_b = qrk3588_ddr_state_training_result_const(state, channel)
            ->direct1_rank0;
    else
        mhz_b = 0u;

    mhz_a = frequency > (type24 & 0xfffu)
        ? ((pr32(type_cfg, 0x2cu) >> 20) & 0x3ffu)
        : ((pr32(type_cfg, 0x30u) >> 20) & 0x3ffu);
    if (frequency <= (pr32(type_cfg, 0x18u) & 0xfffu))
        mhz_b = (pr32(type_cfg, 0x30u) >> 10) & 0x3ffu;
    else if (mhz_b == 0u)
        mhz_b = (pr32(type_cfg, 0x2cu) >> 10) & 0x3ffu;

    enc_a = ddr_freq_divider_encode(&mhz_a, dram_type);
    enc_b = ddr_freq_divider_encode(&mhz_b, dram_type);

    /*
     * The firmware reuses its auxiliary period slot for the final packed
     * divider after the two primary divider encodings.  LP5 with no dynamic
     * period falls back to the clamped base period; other paths keep zero or
     * convert the selected auxiliary period directly.
     */
    uint32_t packed_aux = final_aux;
    if (dram_type == LPDDR5 && dynamic_period == 0u)
        packed_aux = packed_aux != 0u ? packed_aux : period_base;
    if (packed_aux != 0u)
        packed_aux = udiv0(240u, clamp_min(packed_aux, 40u));

    if (dram_type == LPDDR5) {
        struct qrk3588_ddr_lp5_channel_image *image =
            qrk3588_ddr_state_lp5_channel_image(state, channel);
        uint32_t byte_1d6 = (image->byte[2] & ~7u) | period_base_div;
        uint32_t byte_1d8 = dynamic_div | (high_div << 4);
        uint32_t byte_1de = (((pr32(type_cfg, 0x28u) >> 16) & 1u) << 3)
                           | (((pr32(type_cfg, 0x28u) >> 17) & 1u) << 4)
                           | (((pr32(type_cfg, 0x28u) >> 18) & 1u) << 5)
                           | packed_aux;

        image->byte[7] = (uint8_t)enc_b;
        image->byte[8] = (uint8_t)enc_b;
        image->byte[5] = (uint8_t)enc_a;
        image->byte[2] = (uint8_t)byte_1d6;
        if (lp5_aux_div != 0u)
            byte_1d8 |= 8u;
        image->byte[4] = (uint8_t)byte_1d8;
        if (high_div == 0u)
            byte_1de |= 0x10u;
        image->byte[10] = (uint8_t)byte_1de;
        if (qrk3588_ddr_state_channel_device_width_code(state, channel) == 0u)
            image->byte[10] |= 0x80u;
        image->byte[11] = (uint8_t)((image->byte[11] & ~7u) | lp5_threshold_div);
        image->byte[14] = (uint8_t)((image->byte[14] & 0x1fu) | (lp5_aux_div << 5));
        image->byte[13] |= 0x30u;
        return;
    }

    {
        const size_t bank = (size_t)profile << 20;
        uint32_t high_504;
        uint32_t high_508 = dynamic_div | (high_div << 4);
        uint32_t high_50c;

        pw32(phy, 0x10c80u, 0u);
        v = pr32(phy, bank + 0x508u);
        pw32(phy, bank + 0x508u, (v & 0xffff0000u) | enc_a);
        v = pr32(phy, bank + 0x50cu);
        pw32(phy, bank + 0x50cu, (v & 0xffff0000u) | enc_b);
        ddr_phy_update_complete((uint32_t)(uintptr_t)phy);
        /* Native fail-fast: the frozen helper cannot advance until this PHY
         * update completes.  Stop this void stage when the bounded wait fails. */
        if (qrk3588_ddr_get_error() != 0)
            return;

        high_504 = ((pr32(phy, bank + 0x504u) >> 16) & UINT32_C(0xffffffc6));
        high_504 |= live_flag | (period_base_div << 3);

        high_50c = (pr32(phy, bank + 0x50cu) >> 16) & UINT32_C(0xfffffff8);
        high_50c |= (((pr32(type_cfg, 0x28u) >> 16) & 1u) << 3)
                  | (((pr32(type_cfg, 0x28u) >> 17) & 1u) << 4)
                  | (((pr32(type_cfg, 0x28u) >> 18) & 1u) << 5)
                  | packed_aux;
        if (qrk3588_ddr_state_channel_device_width_code(state, channel) == 0u)
            high_50c |= 0x80u;

        pw32(phy, 0x10c80u, 0u);
        v = pr32(phy, bank + 0x504u);
        pw32(phy, bank + 0x504u, (v & 0xffffu) | (high_504 << 16));
        v = pr32(phy, bank + 0x508u);
        pw32(phy, bank + 0x508u, (v & 0xffffu) | (high_508 << 16));
        v = pr32(phy, bank + 0x50cu);
        pw32(phy, bank + 0x50cu, (v & 0xffffu) | (high_50c << 16));
        ddr_phy_update_complete((uint32_t)(uintptr_t)phy);
        if (qrk3588_ddr_get_error() != 0)
            return;
    }
}
