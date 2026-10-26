/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Build the PCTL and PHY register images for each operating point. */
#include <stddef.h>
#include <stdint.h>

#include "qrk3588_ddr_internal.h"
#include "qrk3588_ddr_runtime.h"

#define LPDDR4   7u
#define LPDDR4X  8u
#define LPDDR5   9u

static inline uintptr_t naddr(const void *base, size_t off)
{
    return qrk3588_ddr_resolve_addr((uintptr_t)base + off);
}

static inline uint8_t nrd8(const void *base, size_t off)
{
    return *(volatile const uint8_t *)(uintptr_t)naddr(base, off);
}

static inline uint32_t nrd32(const void *base, size_t off)
{
    return *(volatile const uint32_t *)(uintptr_t)naddr(base, off);
}


static inline void nwr8(void *base, size_t off, uint8_t value)
{
    *(volatile uint8_t *)(uintptr_t)naddr(base, off) = value;
}

static inline void nwr32(void *base, size_t off, uint32_t value)
{
    *(volatile uint32_t *)(uintptr_t)naddr(base, off) = value;
}

static inline uint32_t umax32(uint32_t a, uint32_t b)
{
    return a >= b ? a : b;
}

static inline uint32_t div1000_round_up_u32(uint32_t value)
{
    return (value + 999u) / 1000u;
}

static inline uint32_t scaled_7p5_over_1000(uint32_t value)
{
    return (value * 7u + (value >> 1) + 999u) / 1000u;
}

/*
 * Proposed role: build the PCTL frequency-register image for one channel and
 * one frequency slot.
 *
 * Contract:
 *   context + channel*0x20 + 0x00 -> channel PCTL image base
 *   config DRAM-parameter slot    -> DRAM parameter block
 *   config timing slot            -> synthesized timing block
 *   profile                       -> 1 MiB PCTL image selector
 */
void ddr_pctl_frequency_register_image_build(void *context, uint32_t channel,
                                             uint8_t *config,
                                             uint32_t profile)
{
    uint8_t *dram = (uint8_t *)qrk3588_ddr_state_dram_params(config);
    uint8_t *timing = (uint8_t *)qrk3588_ddr_state_timing(config);
    uint8_t *pctl = (uint8_t *)qrk3588_ddr_ctx_channel_base0(context, channel);
    const uint32_t dram_type = nrd32(dram, 0x28u);
    const uint32_t frequency = nrd32(dram, 0x2cu);
    const uint8_t *freq_cfg = (const uint8_t *)freq_cfg_ptr(dram_type, frequency);
    const size_t slot = (size_t)profile << 20;
    const uint32_t t0 = nrd32(timing, 0x00u);
    const uint32_t t1e8 = nrd32(timing, 0x1e8u);
    const uint32_t t1ec = nrd32(timing, 0x1ecu);
    uint32_t scale_hint = 0u;
    uint32_t value;

    /* Type-specific first control word and LP5 controller-width selector. */
    if (dram_type == LPDDR5) {
        if (nrd32(dram, 0x60u) == 0x80u) {
            scale_hint = t0 >> 1;
            nwr32(pctl, slot + 0xd00u, 0u);
        } else {
            scale_hint = t0 >> 2;
            nwr32(pctl, slot + 0xd00u, 1u);
        }

        value = ((t1ec + nrd32(timing, 0x208u) + nrd32(timing, 0x30u) + 1u) << 24)
              | ((nrd32(timing, 0x54u) >> 10) << 8)
              | nrd32(timing, 0x58u)
              | (nrd32(timing, 0x5cu) << 16);
    } else {
        value = ((t1ec + (nrd32(timing, 0x1f0u) >> 1)
                  + nrd32(timing, 0x30u) + 1u) << 24)
              | ((nrd32(timing, 0x54u) >> 10) << 8)
              | nrd32(timing, 0x58u)
              | (nrd32(timing, 0x5cu) << 16);
    }
    nwr32(pctl, slot + 0x00u, value);
    nwr32(pctl, slot + 0x18u, nrd32(timing, 0x78u) + 2u);

    /* Common refresh/DFI image programming. */
    const struct qrk3588_ddr_context_descriptor_state *descriptor_state =
        qrk3588_ddr_ctx_descriptor_state_const(context);

    if (descriptor_state->wordc_bit27 != 0u)
        value = UINT32_C(0x82100000) | (nrd32(timing, 0x20u) >> 3);
    else
        value = UINT32_C(0x02100000) | ((nrd32(timing, 0x20u) + 31u) >> 5);
    nwr32(pctl, slot + 0x600u, value);
    nwr32(pctl, slot + 0x604u, nrd32(timing, 0x60u));
    nwr32(pctl, slot + 0xb80u, UINT32_C(0x00020000));
    nwr32(pctl, slot + 0x588u, (t1ec - 2u) | UINT32_C(0x200));
    value = nrd32(pctl, slot + 0x5a4u);
    nwr32(pctl, slot + 0x5a4u, (value & UINT32_C(0xffffe0e0)) | UINT32_C(0x801));
    nwr32(pctl, slot + 0x5a8u, UINT32_C(0x00500003));

    if (descriptor_state->wordc_bit26 != 0u) {
        const uint32_t q0 = (t0 + 999u + ((t0 * 7u) >> 3)) / 1000u;
        const uint32_t three_t0 = t0 * 3u;
        const uint32_t q1 = (three_t0 + 999u + (three_t0 >> 2)) / 1000u;

        value = ((nrd32(timing, 0x24u) + q0) << 24)
              | ((nrd32(timing, 0x58u) + q0) << 16)
              | ((nrd32(timing, 0x2cu) + q0) << 8)
              |  (nrd32(timing, 0x44u) + q0);
        nwr32(pctl, slot + 0xb04u, value);
        nwr32(pctl, slot + 0xb08u, nrd32(timing, 0x3cu) + q1);
        nwr32(pctl, slot + 0xb00u, t0 * UINT32_C(0x28c58));
    }

    if (dram_type == LPDDR4 || dram_type == LPDDR4X) {
        const uint32_t half_1f0 = nrd32(timing, 0x1f0u) >> 1;
        const uint32_t freq_q = scaled_7p5_over_1000(frequency);
        const uint32_t min_q = umax32(nrd32(timing, 0x78u), 3u);
        const uint32_t q = umax32(freq_q, min_q);
        uint32_t delay_limit = umax32((uint32_t)nrd8(freq_cfg, 0x0du) + 4u,
                                      nrd32(timing, 0x6cu));
        uint32_t field_08_mid;
        uint32_t q78;

        nwr32(pctl, slot + 0xd04u, UINT32_C(0x808));
        nwr32(pctl, slot + 0x04u,
              nrd32(timing, 0x40u)
              | (nrd32(timing, 0x38u) << 8)
              | (q << 16));

        if (nrd32(dram, 0x40u) != 0u) {
            field_08_mid = half_1f0 - nrd32(timing, 0xa8u)
                         + t1e8 + 2u
                         + delay_limit - nrd32(timing, 0xc8u);
        } else {
            field_08_mid = half_1f0 - t1ec + t1e8 + 3u + delay_limit;
        }
        nwr32(pctl, slot + 0x08u,
              (t1ec << 24) | (t1e8 << 16)
              | (field_08_mid << 8)
              | (half_1f0 + 1u + t1ec + nrd32(timing, 0x4cu)));
        nwr32(pctl, slot + 0xd08u,
              ((half_1f0 + 9u + t1ec + nrd32(timing, 0x4cu)))
              | ((field_08_mid + 8u) << 8));

        q78 = umax32(scaled_7p5_over_1000(t0), 8u);
        nwr32(pctl, slot + 0x0cu,
              (t1ec + nrd32(timing, 0x30u) + half_1f0 + 1u + q78)
              | ((t1e8 + nrd32(timing, 0x6cu)
                  + nrd32(timing, 0x38u) - 8u + half_1f0 + q78) << 8)
              | (nrd32(timing, 0x9cu) << 16));

        nwr32(pctl, slot + 0x10u,
              nrd32(timing, 0x28u)
              | ((nrd32(timing, 0x24u) - nrd32(timing, 0x1e4u)) << 24)
              | (nrd32(timing, 0x44u) << 8)
              | (nrd32(timing, 0x48u) << 16));
        nwr32(pctl, slot + 0x14u,
              nrd32(timing, 0x84u)
              | (umax32(nrd32(timing, 0x17cu), nrd32(timing, 0x84u)) << 8)
              | (nrd32(timing, 0x1c4u) << 16)
              | (umax32(nrd32(timing, 0x1c0u), 8u) << 24));
        nwr32(pctl, slot + 0x30u,
              umax32(nrd32(timing, 0x178u), nrd32(timing, 0x180u)) << 16);
        nwr32(pctl, slot + 0x34u,
              nrd32(timing, 0x174u)
              | (nrd32(timing, 0x16cu) << 16)
              | (nrd32(timing, 0xacu) << 24));
        nwr32(pctl, slot + 0x38u,
              nrd32(timing, 0x70u)
              | (umax32(div1000_round_up_u32(t0 * 40u), 8u) << 16));
        nwr32(pctl, slot + 0x64u,
              (nrd32(timing, 0x38u) | UINT32_C(0x40000))
              | ((half_1f0 + 1u + t1ec + nrd32(timing, 0x30u)) << 8));
        nwr32(pctl, slot + 0x608u, nrd32(timing, 0x170u) << 16);

        value = nrd32(timing, 0x134u);
        value = (value & ~UINT32_C(0x8)) | ((uint32_t)nrd8(freq_cfg, 0x09u) << 3);
        nwr32(timing, 0x134u, value);
        nwr32(pctl, slot + 0x500u, nrd32(timing, 0x138u) | (value << 16));
        nwr32(pctl, slot + 0x504u,
              nrd32(timing, 0x154u) | (nrd32(timing, 0x13cu) << 16));
        nwr32(pctl, slot + 0x508u,
              nrd32(timing, 0x150u) | (nrd32(timing, 0x14cu) << 16));
        nwr32(pctl, slot + 0x50cu,
              nrd32(timing, 0x158u) | (nrd32(timing, 0x168u) << 16));
        nwr32(pctl, slot + 0x580u,
              UINT32_C(0x07000200) | (t1ec - 2u) | ((t1e8 + 15u) << 16));
        nwr32(pctl, slot + 0x800u,
              nrd32(timing, 0x198u) | (nrd32(timing, 0x19cu) << 16));
        return;
    }

    if (dram_type != LPDDR5)
        return;

    /* LPDDR5-only image fields. */
    {
        uint8_t *phy = pctl + 0x10000u;
        int32_t encoded;
        int32_t negative_adjust;
        int32_t delay_raw;
        uint32_t delay_candidate;
        uint32_t max_delay;
        uint32_t positive_limit;
        uint32_t selected_204_208;
        uint32_t field_low;
        uint32_t q;
        uint32_t old24;
        uint32_t lane_mode;

        value = nrd32(phy, 0x10u);
        nwr32(phy, 0x10u, (value & ~UINT32_C(1)) | nrd32(dram, 0x70u));
        nwr32(pctl, slot + 0xd04u,
              (nrd32(timing, 0x298u) - 1u)
              | ((nrd32(timing, 0x29cu) - 1u) << 8));

        selected_204_208 = nrd32(dram, 0x64u) != 0u
                         ? nrd32(timing, 0x204u)
                         : nrd32(timing, 0x208u);
        nwr32(pctl, slot + 0x04u,
              nrd32(timing, 0x40u)
              | (nrd32(timing, 0x78u) << 16)
              | ((selected_204_208 + nrd32(timing, 0x23cu)) << 8));

        encoded = (int32_t)nrd8(freq_cfg, 0x0cu);
        if (encoded > 7)
            encoded = -(encoded & 7);
        delay_raw = (int32_t)nrd8(freq_cfg, 0x0du) + encoded + 7;
        if (nrd32(dram, 0x60u) == 0x80u) {
            delay_candidate = (uint32_t)(delay_raw >> 1);
            negative_adjust = encoded < 0 ? (encoded - 2) / 2 : 0;
        } else {
            delay_candidate = (uint32_t)(delay_raw >> 2);
            negative_adjust = encoded < 0 ? (encoded - 6) / 4 : 0;
        }

        delay_candidate = delay_candidate - t1ec + t1e8 + nrd32(timing, 0x20cu);
        positive_limit = t1ec + nrd32(timing, 0x208u) - t1e8 + 1u;
        positive_limit = (int32_t)positive_limit - negative_adjust > 0
                       ? (uint32_t)((int32_t)positive_limit - negative_adjust) : 0u;
        max_delay = umax32(nrd32(timing, 0x50u), delay_candidate);

        if (nrd32(dram, 0x64u) != 0u)
            field_low = t1ec + nrd32(timing, 0x204u) + nrd32(timing, 0x4cu);
        else
            field_low = nrd32(timing, 0x1d0u) + nrd32(timing, 0x20cu) + t1ec;

        nwr32(pctl, slot + 0x08u,
              field_low | (max_delay << 8) | (t1e8 << 16) | (t1ec << 24));
        max_delay = umax32(max_delay, nrd32(timing, 0x2a0u));
        positive_limit = umax32(positive_limit, nrd32(timing, 0x2a4u));
        nwr32(pctl, slot + 0xd08u, positive_limit | (max_delay << 8));

        q = umax32(scaled_7p5_over_1000(scale_hint), 4u);
        nwr32(pctl, slot + 0x0cu,
              (q + nrd32(timing, 0x30u) + nrd32(timing, 0x20cu) + t1ec)
              | ((t1e8 + nrd32(timing, 0x26cu) + nrd32(timing, 0x20cu)
                  + nrd32(timing, 0x23cu) + q) << 8)
              | (nrd32(timing, 0x9cu) << 16));

        field_low = nrd32(dram, 0x64u) != 0u
                  ? nrd32(timing, 0x44u) : nrd32(timing, 0x1ccu);
        nwr32(pctl, slot + 0x10u,
              nrd32(timing, 0x28u)
              | (field_low << 8)
              | (nrd32(timing, 0x204u) << 16)
              | ((nrd32(timing, 0x24u) - nrd32(timing, 0x1e4u)) << 24));
        nwr32(pctl, slot + 0x14u,
              nrd32(timing, 0x244u)
              | (nrd32(timing, 0x17cu) << 8)
              | (nrd32(timing, 0x24cu) << 16)
              | (umax32(nrd32(timing, 0x250u), 8u) << 24));
        nwr32(pctl, slot + 0x1cu, nrd32(timing, 0x258u));

        old24 = nrd32(pctl, slot + 0x24u) & UINT32_C(0xffe0ff00);
        nwr32(pctl, slot + 0x24u,
              old24 | (t1ec + nrd32(timing, 0x208u) + nrd32(timing, 0x238u))
              | (nrd32(timing, 0x240u) << 16));
        nwr32(pctl, slot + 0x30u,
              umax32(nrd32(timing, 0x248u), nrd32(timing, 0x234u)) << 16);
        nwr32(pctl, slot + 0x34u,
              nrd32(timing, 0x174u)
              | (nrd32(timing, 0xacu) << 24)
              | ((nrd32(dram, 0x64u) != 0u ? nrd32(timing, 0x204u)
                                             : nrd32(timing, 0x20cu)) << 18));
        nwr32(pctl, slot + 0x38u, nrd32(timing, 0x70u) | UINT32_C(0x80000));
        nwr32(pctl, slot + 0x5cu,
              nrd32(timing, 0x278u)
              | ((((nrd32(timing, 0x27cu) + 0x3ffu) >> 10) + 1u) << 16));

        /* csel w9,w9,#2,eq: zero stays zero; non-zero selects code 2. */
        lane_mode = nrd32(dram, 0x64u) == 0u ? 0u : 2u;
        value = (nrd32(timing, 0x20cu) + nrd32(timing, 0x284u) + t1ec)
              | ((nrd32(timing, 0x20cu) + t1e8 + nrd32(timing, 0x284u)) << 8)
              | (umax32(nrd32(timing, 0x2a8u), delay_candidate) << 16)
              | (lane_mode << 24);
        nwr32(pctl, slot + 0x60u, value);
        nwr32(pctl, slot + 0x64u,
              (nrd32(timing, 0x208u) + nrd32(timing, 0x23cu))
              | ((nrd32(timing, 0x1f4u) + 1u + nrd32(timing, 0x208u) + t1ec) << 8));
        nwr32(pctl, slot + 0x78u, UINT32_C(0x000c0c15));
        nwr32(pctl, slot + 0x608u,
              (nrd32(timing, 0x170u) << 16) | (nrd32(timing, 0x230u) << 24));

        if (nrd32(dram, 0x60u) == 0x80u) {
            nwr32(pctl, slot + 0x580u,
                  UINT32_C(0x07000200) | ((t1ec << 1) - 2u) | (t1e8 << 17));
            nwr32(pctl, slot + 0x590u,
                  (nrd32(timing, 0x210u) << 17)
                  | (nrd32(timing, 0x220u) << 25)
                  | (nrd32(timing, 0x22cu) << 9));
        } else {
            nwr32(pctl, slot + 0x580u,
                  UINT32_C(0x07000400) | ((t1ec << 2) - 2u) | (t1e8 << 18));
            nwr32(pctl, slot + 0x590u,
                  (nrd32(timing, 0x210u) << 18)
                  | (nrd32(timing, 0x220u) << 26)
                  | (nrd32(timing, 0x22cu) << 10));
        }
        nwr32(pctl, slot + 0x800u,
              (nrd32(timing, 0x19cu) << 16) | (nrd32(timing, 0x28cu) + 10u));

        /* Per-channel mode-register image bytes stored in the config object. */
        {
            uint8_t *mr = config + (size_t)channel * 20u;
            nwr8(mr, 0x1d4u, (uint8_t)nrd32(timing, 0x134u));
            nwr8(mr, 0x1d5u, (uint8_t)nrd32(timing, 0x138u));
            nwr8(mr, 0x1d6u, (uint8_t)nrd32(timing, 0x13cu));
            nwr8(mr, 0x1d7u, (uint8_t)nrd32(timing, 0x1f8u));
            value = nrd32(timing, 0x154u) | UINT32_C(0x10);
            nwr32(timing, 0x154u, value);
            nwr8(mr, 0x1dau, (uint8_t)value);
            nwr8(mr, 0x1ddu, (uint8_t)nrd32(timing, 0x15cu));
            nwr8(mr, 0x1dfu, (uint8_t)nrd32(timing, 0x1fcu));
            nwr8(mr, 0x1e0u, (uint8_t)nrd32(timing, 0x164u));
            nwr8(mr, 0x1e1u, UINT8_C(0x80));
        }
    }
}

static inline size_t phy_selector_offset(uint32_t selector,
                                         size_t slot0, size_t slot1,
                                         size_t slot2)
{
    if (selector == 1u)
        return slot1;
    if (selector == 2u)
        return slot2;
    return slot0;
}

static uint32_t lp5_training_threshold_word(const uint8_t *dram,
                                            const uint8_t *timing,
                                            uint32_t *mode_is_zero)
{
    const uint32_t half_rate_mode = nrd32(dram, 0x60u);
    const struct qrk3588_ddr_training_threshold *table;
    size_t count;
    uint32_t data_rate = nrd32(timing, 0x00u) << 1;

    if (half_rate_mode == 0x80u) {
        table = qrk3588_lp5_training_low();
        count = 6u;
    } else {
        table = qrk3588_lp5_training_high();
        count = 10u;
    }

    for (size_t i = 0; i < count; ++i) {
        const struct qrk3588_ddr_training_threshold *row = &table[i];
        if (data_rate <= row->max_data_rate_mhz) {
            *mode_is_zero = row->mode_flag == 0u ? 1u : 0u;
            return row->param2
                 | (row->mode_flag << 8)
                 | (row->param4 << 16)
                 | (row->param5 << 24)
                 | (row->param3 << 28);
        }
    }

    *mode_is_zero = 0u;
    return 0u;
}

static void phy_copy_controller_image_fields(uint8_t *pctl, uint8_t *phy,
                                             uint32_t profile)
{
    const size_t slot = (size_t)profile << 20;
    uint32_t source;
    uint32_t value;

    source = nrd32(pctl, slot + 0x500u);
    value = nrd32(phy, 0xa80u);
    value = (value & UINT32_C(0xffffff00)) | (source >> 16);
    nwr32(phy, 0xa80u, value);

    value = nrd32(phy, 0xa80u);
    value = (value & UINT32_C(0xffff00ff)) | ((source & UINT32_C(0xffff)) << 8);
    nwr32(phy, 0xa80u, value);

    source = nrd32(pctl, slot + 0x508u);
    value = nrd32(phy, 0xa80u);
    value = (value & UINT32_C(0xff00ffff)) | (source & UINT32_C(0xffff0000));
    nwr32(phy, 0xa80u, value);

    value = nrd32(phy, 0xa80u);
    value = (value & UINT32_C(0x00ffffff)) | (source << 24);
    nwr32(phy, 0xa80u, value);

    source = nrd32(pctl, slot + 0x504u) & UINT32_C(0xffff);
    value = nrd32(phy, 0xa84u);
    value = (value & UINT32_C(0xffffff00)) | source;
    nwr32(phy, 0xa84u, value);
}

static void phy_program_selector_extension(uint8_t *phy,
                                           const uint8_t *dram,
                                           const uint8_t *timing,
                                           const uint8_t *freq_cfg,
                                           uint32_t dram_type,
                                           uint32_t selector,
                                           uint32_t flag_a,
                                           uint32_t flag_b)
{
    const uint32_t subslot = selector == 1u ? 0u : 4u;
    const size_t timing_slot = selector == 1u ? 0xa48u : 0xa58u;
    const uint32_t selector_timing = dram_type == LPDDR5
                                   ? nrd32(timing, 0x204u) : 8u;
    uint32_t value;

    value = nrd32(phy, timing_slot);
    value = (value & UINT32_C(0xe07fffff)) | (selector_timing << 23);
    nwr32(phy, timing_slot, value);

    nwr32(phy, (size_t)subslot + 0xc4u,
          ((uint32_t)nrd8(freq_cfg, 0x00u) << 30)
        | ((uint32_t)nrd8(freq_cfg, 0x01u) << 31)
        | UINT32_C(0x30800000));

    value = nrd32(phy, (size_t)subslot + 0xbcu) & UINT32_C(0xff);
    value |= (uint32_t)nrd8(freq_cfg, 0x09u) << 12;
    value |= (uint32_t)nrd8(freq_cfg, 0x0au) << 13;
    value |= nrd32(timing, 0x1e8u) << 14;
    value |= nrd32(timing, 0x00u) << 20;
    value |= (uint32_t)nrd8(freq_cfg, 0x0fu) << 8;
    value |= flag_a << 11;
    value |= flag_b << 10;

    /*
     * The frozen flow publishes the common selector-bank value, then reads
     * it back before applying the DRAM-specific mode bits.  Keep the two
     * writes distinct: the final word is the same, but a reused bank can
     * observe the intermediate transaction.
     */
    nwr32(phy, (size_t)subslot + 0xbcu, value);
    value = nrd32(phy, (size_t)subslot + 0xbcu);

    if (dram_type == LPDDR5) {
        value &= ~UINT32_C(0x61);
        value |= nrd32(dram, 0x64u) != 0u ? 1u : 0u;
        value |= (uint32_t)nrd8(freq_cfg, 0x13u) << 5;
    } else {
        value &= ~UINT32_C(0x80);
        value |= ((uint32_t)nrd8(freq_cfg, 0x0fu) & 1u) << 7;
    }
    nwr32(phy, (size_t)subslot + 0xbcu, value);

    /* Selector-extension format is common to LP4/LP5; only the preceding
     * bit-7 / LP5 mode-bit patch differs by DRAM type. */
    value = ((uint32_t)nrd8(freq_cfg, 0x0eu) << 30)
          | ((uint32_t)nrd8(freq_cfg, 0x07u) << 31)
          | ((uint32_t)nrd8(freq_cfg, 0x0bu) << 24)
          | ((uint32_t)nrd8(freq_cfg, 0x0cu) | UINT32_C(0x60000))
          | ((uint32_t)nrd8(freq_cfg, 0x02u) << 29)
          | ((uint32_t)nrd8(freq_cfg, 0x06u) << 12)
          | ((uint32_t)nrd8(freq_cfg, 0x08u) << 16)
          | ((uint32_t)nrd8(freq_cfg, 0x0du) << 4)
          | ((uint32_t)nrd8(freq_cfg, 0x05u) << 8);
    nwr32(phy, (size_t)subslot + 0xccu, value);

    value = nrd32(phy, (size_t)subslot + 0xa00u) & UINT32_C(0xffffff00);
    value |= (uint32_t)nrd8(freq_cfg, 0x10u);
    value |= (uint32_t)nrd8(freq_cfg, 0x10u) << 4;
    nwr32(phy, (size_t)subslot + 0xa00u, value);
}

/*
 * Semantic role: build/program one PHY frequency image for a channel.
 *
 * selector selects one of the three PHY image banks; profile selects
 * the controller image that is mirrored into PHY fields on LP4/LP4X.
 */
void ddr_phy_frequency_register_image_build(void *context, uint32_t channel,
                                             uint8_t *state,
                                             uint32_t selector,
                                             uint32_t profile,
                                             uint32_t use_freq_cfg_flag)
{
    uint8_t *dram = (uint8_t *)qrk3588_ddr_state_dram_params(state);
    uint8_t *timing = (uint8_t *)qrk3588_ddr_state_timing(state);
    uint8_t *pctl = (uint8_t *)qrk3588_ddr_ctx_channel_base0(context, channel);
    uint8_t *phy = (uint8_t *)qrk3588_ddr_ctx_channel_base1(context, channel);
    const uint32_t dram_type = nrd32(dram, 0x28u);
    const uint32_t frequency = nrd32(dram, 0x2cu);
    const uint8_t *freq_cfg = (const uint8_t *)freq_cfg_ptr(dram_type, frequency);
    const uint32_t timing0 = nrd32(timing, 0x00u);
    const uint32_t scaled_timing = dram_type == LPDDR5
                                 ? (timing0 >> (nrd32(dram, 0x60u) == 0x80u ? 1u : 2u))
                                 : timing0;
    const uint32_t flag_a = qrk3588_ddr_state_channel_rank_count(state, channel) == 1u
                          ? 0u : ((uint32_t)nrd8(freq_cfg, 0x03u) & 1u);
    const uint32_t flag_b = use_freq_cfg_flag == 0u
                          ? 1u : ((uint32_t)nrd8(freq_cfg, 0x04u) & 1u);
    uint32_t delay_cycles;
    uint32_t value;

    if (dram_type == LPDDR5) {
        delay_cycles = (scaled_timing * 6u + (scaled_timing * 117u) / 1000u + 999u) / 1000u;
    } else {
        delay_cycles = (scaled_timing * 8u + (frequency * 46u) / 1000u + 999u) / 1000u;
    }

    value = (nrd32(timing, 0xa0u) << 14)
          | (nrd32(timing, 0x274u) << 18)
          | nrd32(timing, 0x1ecu)
          | (delay_cycles << 6);
    nwr32(phy, phy_selector_offset(selector, 0xa2cu, 0xa3cu, 0xa4cu), value);

    value = (nrd32(timing, 0x24u) << 14)
          | (nrd32(timing, 0x38u) << 8)
          | (dram_type == LPDDR5
                ? (nrd32(timing, 0x48u) | UINT32_C(0x67c00000))
                : UINT32_C(0x67c00008));
    nwr32(phy, phy_selector_offset(selector, 0xa30u, 0xa40u, 0xa50u), value);

    if (dram_type == LPDDR5) {
        value = (nrd32(timing, 0x9cu) << 6)
              | (nrd32(timing, 0x2cu) << 13)
              | UINT32_C(0x03c00000);
    } else {
        const uint32_t phase = (nrd32(timing, 0x1f0u) >> 1)
                             - nrd32(timing, 0x1ecu)
                             + nrd32(timing, 0x1e8u)
                             + nrd32(timing, 0x50u);
        value = (nrd32(timing, 0x9cu) << 6)
              | (nrd32(timing, 0x2cu) << 13)
              | (phase << 21);
    }
    nwr32(phy, phy_selector_offset(selector, 0xa34u, 0xa44u, 0xa54u), value);

    if (dram_type == LPDDR5) {
        const size_t word_slot = phy_selector_offset(selector, 0xae0u, 0xae4u, 0xae8u);
        const size_t flag_slot = phy_selector_offset(selector, 0xadcu, 0xaf4u, 0xaf8u);
        const size_t enable_slot = phy_selector_offset(selector, 0xa70u, 0xa48u, 0xa58u);
        uint32_t mode_is_zero;
        uint8_t *lane = state + (size_t)channel * 0x14u;
        uint32_t packed = lp5_training_threshold_word(dram, timing, &mode_is_zero);

        nwr32(phy, word_slot, packed);
        nwr32(phy, flag_slot, mode_is_zero);

        value = nrd32(phy, enable_slot);
        value = (value & ~UINT32_C(1)) | (mode_is_zero ^ 1u);
        nwr32(phy, enable_slot, value);

        nwr8(lane, 0x1dau, (uint8_t)(nrd8(lane, 0x1dau) | 0x40u));
        value = (uint32_t)nrd8(lane, 0x1d7u)
              | ((uint32_t)nrd8(lane, 0x1d8u) << 8)
              | ((uint32_t)nrd8(lane, 0x1d9u) << 16)
              | ((uint32_t)nrd8(lane, 0x1ddu) << 24);
        nwr32(phy, 0xa88u, value);
        nwr32(phy, 0xa8cu, (uint32_t)*(volatile const uint16_t *)(uintptr_t)naddr(lane, 0x1deu));

        value = (uint32_t)nrd8(lane, 0x1d5u)
              | ((uint32_t)nrd8(lane, 0x1d6u) << 8)
              | ((uint32_t)nrd8(lane, 0x1d5u) << 16)
              | ((uint32_t)nrd8(lane, 0x1d6u) << 24);
        nwr32(phy, phy_selector_offset(selector, 0xe34u, 0xe38u, 0xe3cu), value);
    } else {
        phy_copy_controller_image_fields(pctl, phy, profile);
    }

    if (selector != 0u) {
        phy_program_selector_extension(phy, dram, timing, freq_cfg,
                                       dram_type, selector, flag_a, flag_b);
        return;
    }

    nwr32(phy, 0xa38u,
          dram_type == LPDDR5 ? (nrd32(timing, 0x204u) << 4) : UINT32_C(0x80));

    value = nrd32(phy, 0x00u) & UINT32_C(0xffffc0c0);
    value |= nrd32(timing, 0x1e8u);
    value |= nrd32(timing, 0x1f0u) == 0x10u ? UINT32_C(0x1000) : UINT32_C(0x2000);
    nwr32(phy, 0x00u, value);

    value = nrd32(phy, 0xa1cu);
    value = (value & UINT32_C(0xffffffc0)) | nrd32(timing, 0x1e8u);
    nwr32(phy, 0xa1cu, value);

    if (dram_type == LPDDR5) {
        nwr32(phy, 0x00u, nrd32(phy, 0x00u) & UINT32_C(0xffffbfff));
    } else {
        const uint32_t image = nrd32(pctl, ((size_t)profile << 20) + 0x504u);
        value = nrd32(phy, 0x00u) & UINT32_C(0xf7ffffff);
        value |= ((image >> 17) & 1u) << 27;
        nwr32(phy, 0x00u, value);
    }

    value = nrd32(phy, 0xb8u);
    value = (value & UINT32_C(0xfffff000)) | (timing0 & UINT32_C(0xfff));
    nwr32(phy, 0xb8u, value);

    value = nrd32(phy, 0xb0u) & UINT32_C(0xfff800fe);
    value |= UINT32_C(0x0002ef00) | (uint32_t)nrd8(freq_cfg, 0x00u);
    nwr32(phy, 0xb0u, value);

    value = nrd32(phy, 0x04u) & UINT32_C(0xcfffffff);
    value |= flag_a << 29;
    value |= flag_b << 28;
    if (dram_type == LPDDR5) {
        value &= ~UINT32_C(0x00008000);
        value |= (nrd32(dram, 0x64u) != 0u ? 1u : 0u) << 15;
    }
    nwr32(phy, 0x04u, value);

    if (dram_type == LPDDR5) {
        value = ((uint32_t)nrd8(freq_cfg, 0x06u) << 24)
              | ((uint32_t)nrd8(freq_cfg, 0x08u) << 29)
              | ((uint32_t)nrd8(freq_cfg, 0x02u) << 30)
              | ((uint32_t)nrd8(freq_cfg, 0x09u) << 31)
              | ((uint32_t)nrd8(freq_cfg, 0x0du) << 16)
              | ((uint32_t)nrd8(freq_cfg, 0x05u) << 20)
              | ((uint32_t)nrd8(freq_cfg, 0x0cu) << 12);
    } else {
        value = ((uint32_t)nrd8(freq_cfg, 0x02u) << 30)
              | ((uint32_t)nrd8(freq_cfg, 0x09u) << 31)
              | ((uint32_t)nrd8(freq_cfg, 0x06u) << 24)
              | ((uint32_t)nrd8(freq_cfg, 0x08u) << 29)
              | ((uint32_t)nrd8(freq_cfg, 0x0du) << 16)
              | ((uint32_t)nrd8(freq_cfg, 0x05u) << 20)
              | ((uint32_t)nrd8(freq_cfg, 0x0cu) << 12)
              | UINT32_C(0x300);
    }
    nwr32(phy, 0x0cu, value);

    if (dram_type != LPDDR5) {
        value = nrd32(phy, 0x14u) & UINT32_C(0xffbfffff);
        value |= ((uint32_t)nrd8(freq_cfg, 0x0fu) & 1u) << 22;
        nwr32(phy, 0x14u, value);

        if (qrk3588_ddr_state_channel_rank_row_bits(state, channel, 0u) > 0x11u
            || (qrk3588_ddr_state_channel_rank_count(state, channel) > 1u
                && qrk3588_ddr_state_channel_rank_row_bits(state, channel, 1u) > 0x11u)) {
            nwr32(phy, 0x14u, nrd32(phy, 0x14u) | UINT32_C(0x80000000));
        }
    }

    value = nrd32(phy, 0x00u) & UINT32_C(0xfbff7fbf);
    value |= (uint32_t)nrd8(freq_cfg, 0x0au) << 15;
    value |= (uint32_t)nrd8(freq_cfg, 0x0eu) << 26;
    value |= (uint32_t)nrd8(freq_cfg, 0x07u) << 6;
    nwr32(phy, 0x00u, value);

    value = nrd32(phy, 0x18u) & UINT32_C(0xfff3b9bc);
    value |= (uint32_t)nrd8(freq_cfg, 0x0bu);
    value |= dram_type == LPDDR5 ? UINT32_C(0x000c4740) : UINT32_C(0x000c4440);
    nwr32(phy, 0x18u, value);

    value = nrd32(phy, 0x10u) & UINT32_C(0xfffff7ff);
    value |= (uint32_t)nrd8(freq_cfg, 0x0fu) << 11;
    nwr32(phy, 0x10u, value);

    value = nrd32(phy, 0x400u) & UINT32_C(0xffffff00);
    value |= (uint32_t)nrd8(freq_cfg, 0x10u);
    value |= (uint32_t)nrd8(freq_cfg, 0x10u) << 4;
    nwr32(phy, 0x400u, value);

    if (dram_type == LPDDR5) {
        value = nrd32(phy, 0x08u) & UINT32_C(0xffff3fff);
        value |= (uint32_t)nrd8(freq_cfg, 0x13u) << 14;
        nwr32(phy, 0x08u, value);
    }
}
