/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* LPDDR4, LPDDR4X and channel timing construction. */
#include <stddef.h>
#include <stdint.h>
#include "qrk3588_ddr_internal.h"
#include "qrk3588_ddr_runtime.h"

#define LPDDR4   7u
#define LPDDR4X  8u
#define LPDDR5   9u

static inline uintptr_t timing_addr(const void *base, size_t off)
{
    return qrk3588_ddr_resolve_addr((uintptr_t)base + off);
}

static inline uint8_t data_u8(const void *base, size_t off)
{
    return *(volatile const uint8_t *)(uintptr_t)timing_addr(base, off);
}

static inline uint32_t data_u32(const void *base, size_t off)
{
    return *(volatile const uint32_t *)(uintptr_t)timing_addr(base, off);
}

static inline uint64_t data_u64(const void *base, size_t off)
{
    return *(volatile const uint64_t *)(uintptr_t)timing_addr(base, off);
}

static inline void data_set_u8(void *base, size_t off, uint8_t value)
{
    *(volatile uint8_t *)(uintptr_t)timing_addr(base, off) = value;
}

static inline void data_set_u32(void *base, size_t off, uint32_t value)
{
    *(volatile uint32_t *)(uintptr_t)timing_addr(base, off) = value;
}

static inline uint8_t replace_u8_field(uint8_t old_value, uint32_t value,
                                       unsigned lsb, unsigned width)
{
    const uint32_t field_mask = ((UINT32_C(1) << width) - 1u) << lsb;
    return (uint8_t)(((uint32_t)old_value & ~field_mask) |
                     ((value << lsb) & field_mask));
}

static inline uint32_t max_u32(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}

static inline int32_t sdiv32(int32_t numerator, int32_t denominator)
{
    if (denominator == 0)
        return 0;
    if (numerator == INT32_MIN && denominator == -1)
        return INT32_MIN;
    return numerator / denominator;
}

/*
 *
 * Builds the per-channel compact timing/profile image consumed by the PHY and
 * copies the final six 32-bit profile words into the channel output block.
 *
 * Inputs:
 *   context[channel * 0x20 + 0x00] -> PHY base
 *   context[channel * 0x20 + 0x18] -> output image
 *   config DRAM-parameter slot      -> DRAM parameter block
 *   config timing slot              -> synthesized timing block
 *   config + channel * 0x6c        -> channel profile
 */
void ddr_channel_timing_pack(void *context, uint32_t channel, uint8_t *config)
{
    uint8_t *dram_params = (uint8_t *)qrk3588_ddr_state_dram_params(config);
    uint8_t *timing = (uint8_t *)qrk3588_ddr_state_timing(config);
    const struct qrk3588_ddr_channel_geometry *geometry =
        qrk3588_ddr_state_channel_geometry_const(config, channel);
    struct qrk3588_ddr_channel_packed_profile *profile =
        qrk3588_ddr_state_channel_packed_profile(config, channel);
    uint8_t *profile_bytes = (uint8_t *)(void *)profile;
    uint8_t *channel_ctx = qrk3588_ddr_ctx_channel_slot(context, channel);
    const uintptr_t phy_base = (uintptr_t)data_u64(channel_ctx, 0x00);
    uint8_t *profile_out = (uint8_t *)(uintptr_t)data_u64(channel_ctx, 0x18);
    const uint32_t dram_type = qrk3588_ddr_state_dram_type(config);
    const uint32_t dram_rate = data_u32(dram_params, 0x28);
    const uint32_t dram_mode = data_u32(dram_params, 0x2c);
    const uint8_t *freq_cfg = (const uint8_t *)freq_cfg_ptr(dram_rate, dram_mode);
    const uint32_t phy_lane_code = (data_u32((const void *)(phy_base + 0x10000u), 0) >> 16) & 0xfu;
    const uint32_t channel_shift = geometry->bus_width_code & 31u;
    const uint32_t bus_width_metric = (UINT32_C(2) << channel_shift) * phy_lane_code;
    uint32_t divisor;
    uint32_t timing_24_28;
    uint32_t t_1e8;
    uint32_t t_1ec;
    uint32_t profile_40_value;
    uint32_t profile_41_value;
    uint32_t freq13_limit;
    int32_t encoded_offset;
    uint8_t value8;

    /* LP5 x16 uses a doubled controller timing quantum; all LP4/4X paths do. */
    if (dram_type == LPDDR5)
        divisor = data_u32(dram_params, 0x60) == 0x80u ? 2u : 1u;
    else
        divisor = 2u;

    timing_24_28 = data_u32(timing, 0x24) + data_u32(timing, 0x28);
    t_1e8 = data_u32(timing, 0x1e8);
    t_1ec = data_u32(timing, 0x1ec);

    value8 = data_u8(profile_bytes, 0x00);
    value8 = replace_u8_field(value8, data_u32(timing, 0x40) / divisor, 0, 6);
    data_set_u8(profile_bytes, 0x00, value8);

    value8 = data_u8(profile_bytes, 0x02);
    value8 = replace_u8_field(value8,
                              (t_1ec + data_u32(timing, 0x30) + timing_24_28) / divisor,
                              0, 6);
    data_set_u8(profile_bytes, 0x02, value8);

    if (dram_type == LPDDR4 || dram_type == LPDDR4X) {
        const uint32_t half_t_1f0 = data_u32(timing, 0x1f0) >> 1;
        uint32_t t;

        value8 = data_u8(profile_bytes, 0x01);
        t = timing_24_28 + data_u32(timing, 0x38) - half_t_1f0;
        value8 = replace_u8_field(value8, t / divisor, 0, 6);
        data_set_u8(profile_bytes, 0x01, value8);

        data_set_u8(profile_bytes, 0x03,
                    (uint8_t)(((half_t_1f0 + timing_24_28 + t_1e8) / divisor) + 60u));

        freq13_limit = max_u32((uint32_t)data_u8(freq_cfg, 0x0d) + 4u,
                               data_u32(timing, 0x6c));

        if (data_u32(dram_params, 0x40) == 1u) {
            profile_40_value = t_1e8 + 2u + freq13_limit;
            profile_40_value -= data_u32(timing, 0xa8) + data_u32(timing, 0xc8);
        } else {
            profile_40_value = freq13_limit - t_1ec + t_1e8 + 3u;
        }

        profile_41_value = t_1ec + data_u32(timing, 0x4c) + 1u;
    } else {
        uint32_t freq12 = data_u8(freq_cfg, 0x0c);
        int32_t signed_delay;
        uint32_t candidate;

        value8 = data_u8(profile_bytes, 0x01);
        value8 = replace_u8_field(value8,
                                  (timing_24_28 + data_u32(timing, 0x23c)) / divisor,
                                  0, 6);
        data_set_u8(profile_bytes, 0x01, value8);

        data_set_u8(profile_bytes, 0x03,
                    (uint8_t)(((t_1e8 + data_u32(timing, 0x20c) + timing_24_28) /
                               divisor) + 54u));

        encoded_offset = freq12 <= 7u ? (int32_t)freq12 : -(int32_t)(freq12 & 7u);
        signed_delay = (int32_t)data_u8(freq_cfg, 0x0d) + encoded_offset;
        if (data_u32(dram_params, 0x60) == 0x80u)
            signed_delay = sdiv32(signed_delay + 6, 2);
        else
            signed_delay = sdiv32(signed_delay + 4, 4);

        candidate = (uint32_t)signed_delay;
        candidate = max_u32(data_u32(timing, 0x26c), candidate);

        if (data_u32(dram_params, 0x40) == 1u) {
            profile_40_value = t_1e8 + data_u32(timing, 0x260);
            profile_40_value -= data_u32(timing, 0xa8) + data_u32(timing, 0xc8);
            profile_40_value += candidate + 1u;
        } else {
            profile_40_value = t_1e8 - t_1ec + candidate;
        }

        if (data_u32(dram_params, 0x64) == 0u) {
            profile_41_value = t_1ec + data_u32(timing, 0x238);
        } else {
            profile_41_value = t_1ec + data_u32(timing, 0x204);
            profile_41_value += data_u32(timing, 0x4c) - data_u32(timing, 0x208);
        }
    }

    value8 = data_u8(profile_bytes, 0x04);
    value8 = replace_u8_field(value8, profile_40_value / divisor, 0, 5);
    data_set_u8(profile_bytes, 0x04, value8);

    value8 = data_u8(profile_bytes, 0x05);
    value8 = replace_u8_field(value8, profile_41_value / divisor, 0, 5);
    data_set_u8(profile_bytes, 0x05, value8);

    value8 = data_u8(profile_bytes, 0x06);
    value8 = replace_u8_field(value8, data_u32(timing, 0x44) / divisor, 0, 4);
    data_set_u8(profile_bytes, 0x06, value8);

    value8 = data_u8(profile_bytes, 0x07);
    value8 = replace_u8_field(value8, data_u32(timing, 0x5c) / divisor, 0, 6);
    data_set_u8(profile_bytes, 0x07, value8);

    value8 = data_u8(profile_bytes, 0x08);
    value8 = replace_u8_field(value8, divisor << 1, 0, 4);
    data_set_u8(profile_bytes, 0x08, value8);

    value8 = data_u8(profile_bytes, 0x09);
    value8 = replace_u8_field(value8, divisor * 6u, 0, 6);
    data_set_u8(profile_bytes, 0x09, value8);

    value8 = data_u8(profile_bytes, 0x10);
    value8 = replace_u8_field(value8, 1u, 0, 3);
    value8 = replace_u8_field(value8, 1u, 4, 4);
    data_set_u8(profile_bytes, 0x10, value8);

    value8 = data_u8(profile_bytes, 0x11);
    value8 = replace_u8_field(value8, 1u, 0, 4);
    value8 = replace_u8_field(value8, 1u, 4, 3);
    data_set_u8(profile_bytes, 0x11, value8);

    value8 = data_u8(profile_bytes, 0x14);
    value8 = (uint8_t)((value8 & ~UINT8_C(0x03)) | UINT8_C(0x04));
    switch (bus_width_metric) {
    case 16u:
        value8 &= (uint8_t)~UINT8_C(0x18);
        break;
    case 32u:
        value8 = replace_u8_field(value8, 1u, 3, 2);
        break;
    case 64u:
        value8 = replace_u8_field(value8, 2u, 3, 2);
        break;
    default:
        value8 |= UINT8_C(0x1c);
        break;
    }
    data_set_u8(profile_bytes, 0x14, value8);

    value8 = data_u8(profile_bytes, 0x14);
    value8 = replace_u8_field(value8, 1u, 5, 2);
    data_set_u8(profile_bytes, 0x14, value8);

    value8 = data_u8(profile_bytes, 0x0c);
    value8 = replace_u8_field(value8,
                              data_u32(timing,
                                       (dram_type == LPDDR4 || dram_type == LPDDR4X) ?
                                       0x48u : 0x240u) / divisor,
                              0, 3);
    value8 = replace_u8_field(value8, data_u8(profile_bytes, 0x05), 3, 5);
    data_set_u8(profile_bytes, 0x0c, value8);

    value8 = data_u8(profile_bytes, 0x0d);
    value8 = replace_u8_field(value8, data_u8(profile_bytes, 0x06), 0, 4);
    data_set_u8(profile_bytes, 0x0d, value8);

    if (profile_out != NULL) {
        for (uint32_t i = 0u; i != QRK3588_DDR_STATE_PACKED_PROFILE_WORDS; ++i)
            data_set_u32(profile_out, 0x10u + (size_t)i * 4u, profile->word[i]);
    }
}


static inline uint32_t ceil_div1000_u64(uint64_t value)
{
    return (uint32_t)((value + UINT64_C(999)) / UINT64_C(1000));
}

static inline uint32_t cycles_from_ns_x1000(uint32_t frequency, uint32_t time_ns)
{
    return ceil_div1000_u64((uint64_t)frequency * time_ns);
}

static inline uint32_t timing_u32(const uint8_t *timing, size_t off)
{
    return data_u32(timing, off);
}

static inline void timing_set(uint8_t *timing, size_t off, uint32_t value)
{
    data_set_u32(timing, off, value);
}

struct lp4_base_profile {
    uint32_t t38;
    uint32_t ta8;
    uint32_t tac;
    uint32_t t138;
    uint32_t t1ec;
    uint32_t mr134_base;
};

static unsigned lp4_frequency_bin(uint32_t mhz)
{
    if (mhz <= 266u)  return 0u;
    if (mhz <= 533u)  return 1u;
    if (mhz <= 800u)  return 2u;
    if (mhz <= 1066u) return 3u;
    if (mhz <= 1333u) return 4u;
    if (mhz <= 1600u) return 5u;
    if (mhz <= 1866u) return 6u;
    return 7u;
}

static unsigned lp4_work_flag_bin(uint32_t flag)
{
    if (flag <= 4u)  return 0u;
    if (flag <= 6u)  return 1u;
    if (flag <= 8u)  return 2u;
    if (flag <= 10u) return 3u;
    if (flag <= 12u) return 4u;
    if (flag <= 14u) return 5u;
    if (flag <= 16u) return 6u;
    return 7u;
}

static void lp4_apply_base_profile(uint8_t *timing, unsigned bin,
                                   uint32_t x8_adjust, uint32_t is_32bit,
                                   uint32_t extended_mode)
{
    static const struct lp4_base_profile profiles[8] = {
        {  8u, 0u,  0u,  0u,  4u,   4u },
        {  8u, 0u,  0u,  9u,  6u,  20u },
        {  8u, 0u,  0u, 18u,  8u,  36u },
        {  8u, 4u, 20u, 27u, 10u,  52u },
        { 10u, 4u, 22u, 36u, 12u,  68u },
        { 12u, 6u, 24u, 45u, 14u,  84u },
        { 14u, 6u, 26u, 54u, 16u, 100u },
        { 16u, 8u, 28u, 63u, 18u, 116u },
    };
    const struct lp4_base_profile *p = &profiles[bin];
    uint32_t t1e8, t1f4;

    switch (bin) {
    case 0:
        t1e8 = 6u;
        t1f4 = 6u;
        break;
    case 1:
        t1e8 = extended_mode ? 12u : 10u;
        t1f4 = (x8_adjust + 5u) * 2u;
        break;
    case 2:
        t1e8 = (x8_adjust + (extended_mode ? 8u : 7u)) * 2u;
        t1f4 = 16u;
        break;
    case 3:
        t1e8 = (x8_adjust + (extended_mode ? 11u : 10u)) * 2u;
        t1f4 = (x8_adjust + 10u) * 2u;
        break;
    case 4:
        t1e8 = (x8_adjust + (extended_mode ? 14u : 12u)) * 2u;
        t1f4 = (x8_adjust + 6u) * 4u;
        break;
    case 5:
        t1e8 = (x8_adjust + (extended_mode ? 8u : 7u)) * 4u;
        t1f4 = (x8_adjust + 15u) * 2u;
        break;
    case 6:
        t1e8 = extended_mode ? (x8_adjust + 9u) * 4u
                             : (x8_adjust + 8u) * 4u;
        t1f4 = x8_adjust * 4u + 34u;
        break;
    default:
        t1e8 = extended_mode ? (x8_adjust + 10u) * 4u
                             : (x8_adjust + 9u) * 4u;
        t1f4 = (x8_adjust + 10u) * 4u;
        break;
    }

    timing_set(timing, 0x038, p->t38);
    timing_set(timing, 0x0a8, p->ta8);
    timing_set(timing, 0x0ac, p->tac);
    timing_set(timing, 0x134, is_32bit | p->mr134_base);
    timing_set(timing, 0x138, p->t138);
    timing_set(timing, 0x1e8, t1e8);
    timing_set(timing, 0x1ec, p->t1ec);
    timing_set(timing, 0x1f4, t1f4);
}

static uint32_t lp4_encode_mr13c(const uint8_t *dram, uint32_t dram_type)
{
    uint32_t left = data_u32(dram, 0x44);
    uint32_t right = data_u32(dram, 0x48);
    uint32_t value;

    left = left != 0u ? 0x40u : 0u;
    right = right != 0u ? 0x80u : 0u;
    value = left | right;
    if (dram_type != LPDDR4X)
        value |= 1u;

    switch (data_u32(dram, 0x4c)) {
    case 0x30u: value |= 0x28u; break;
    case 0x3cu: value |= 0x20u; break;
    case 0x50u: value |= 0x18u; break;
    case 0x78u: value |= 0x10u; break;
    case 0xf0u: value |= 0x08u; break;
    default:    value |= 0x30u; break;
    }
    return value;
}

static uint32_t lp4_drive_code(uint32_t value)
{
    switch (value) {
    case 0u:    return 0u;
    case 0x30u: return 5u;
    case 0x3cu: return 4u;
    case 0x50u: return 3u;
    case 0x78u: return 2u;
    case 0xf0u: return 1u;
    default:    return 6u;
    }
}

static uint32_t lp4_encode_mr14c(const uint8_t *dram, uint32_t base)
{
    switch (data_u32(dram, 0x54)) {
    case 0u:    return base;
    case 0x30u: return base | 0x50u;
    case 0x3cu: return base | 0x40u;
    case 0x50u: return base | 0x30u;
    case 0x78u: return base | 0x20u;
    case 0xf0u: return base | 0x10u;
    default:    return base | 0x60u;
    }
}

/*
 *
 * Semantic LPDDR4/LPDDR4X timing synthesis.  The output is the  * 0x2ac-byte timing ABI consumed by the controller/PHY image builders.  Field
 * offsets are retained where the stripped image carries no unique source name.
 */
void ddr_timing_base_calculate(uint8_t *dram, uint8_t *timing,
                               uint32_t dram_type)
{
    const uint32_t frequency = data_u32(dram, 0x2c);
    const uint32_t geometry_count = data_u32(dram, 0x30);
    const uint32_t x8_adjust = data_u32(dram, 0x10) == 8u ? 1u : 0u;
    const uint32_t is_32bit = data_u32(dram, 0x34) == 0x20u ? 1u : 0u;
    const uint32_t extended_mode = data_u32(dram, 0x44) != 0u;
    uint32_t max_geometry = 0u;
    unsigned freq_bin, work_bin, profile_bin;
    uint32_t t5, t10, t18, t21, t30, t40, t42;
    uint32_t t3900, t200000, t2000000;
    uint32_t density_coeff;
    uint32_t i;

    for (i = 0; i < geometry_count; ++i) {
        const size_t off = 4u + (size_t)i * 0x14u;
        const uint32_t ranks = data_u32(dram, off + 0x00u);
        if (ranks == 0u)
            continue;
        max_geometry = max_u32(max_geometry, data_u32(dram, off + 0x04u));
        if (ranks > 1u)
            max_geometry = max_u32(max_geometry, data_u32(dram, off + 0x08u));
    }

    ddr_memset(timing, 0, 0x2acu);
    timing_set(timing, 0x000, frequency);
    timing_set(timing, 0x1e4, 0u);
    timing_set(timing, 0x1f0, data_u32(dram, 0x34));

    freq_bin = lp4_frequency_bin(frequency);
    work_bin = lp4_work_flag_bin(qrk3588_ddr_timing_work_flag());
    profile_bin = freq_bin > work_bin ? freq_bin : work_bin;
    lp4_apply_base_profile(timing, profile_bin, x8_adjust, is_32bit, extended_mode);

    timing_set(timing, 0x13c, lp4_encode_mr13c(dram, dram_type));
    timing_set(timing, 0x130, 0u);
    timing_set(timing, 0x14c,
               lp4_encode_mr14c(dram,
                   data_u32(dram, 0x40) ? lp4_drive_code(data_u32(dram, 0x50))
                                        : 0u));

    t5       = cycles_from_ns_x1000(frequency, 5u);
    t10      = cycles_from_ns_x1000(frequency, 10u);
    t18      = cycles_from_ns_x1000(frequency, 18u);
    t21      = cycles_from_ns_x1000(frequency, 21u);
    t30      = cycles_from_ns_x1000(frequency, 30u);
    t40      = cycles_from_ns_x1000(frequency, 40u);
    t42      = cycles_from_ns_x1000(frequency, 42u);
    t3900    = cycles_from_ns_x1000(frequency, 3900u);
    t200000  = cycles_from_ns_x1000(frequency, 200000u);
    t2000000 = cycles_from_ns_x1000(frequency, 2000000u);

    t18 = max_u32(t18, 4u);
    t21 = max_u32(t21, 4u);
    t42 = max_u32(t42, 3u);
    t10 = max_u32(t10, 4u);

    timing_set(timing, 0x004, t200000);
    timing_set(timing, 0x008, cycles_from_ns_x1000(frequency, 10u));
    timing_set(timing, 0x00c, t2000000);
    timing_set(timing, 0x010, t5);
    timing_set(timing, 0x014, cycles_from_ns_x1000(frequency, 2000u));
    timing_set(timing, 0x018, t200000);
    timing_set(timing, 0x01c, t2000000);
    timing_set(timing, 0x020, t3900);
    timing_set(timing, 0x024, t18);
    timing_set(timing, 0x028, t18);
    timing_set(timing, 0x02c, t21);
    timing_set(timing, 0x030,
               max_u32(cycles_from_ns_x1000(frequency,
                   2u * (9u + x8_adjust)), 6u));
    timing_set(timing, 0x038, timing_u32(timing, 0x038));
    timing_set(timing, 0x03c, t21 + t42);
    timing_set(timing, 0x040, t18 + t42);
    timing_set(timing, 0x044, t10);
    timing_set(timing, 0x048, data_u32(dram, 0x34) == 0x20u ? 16u : 8u);
    timing_set(timing, 0x04c,
               max_u32(cycles_from_ns_x1000(frequency,
                   2u * (9u + x8_adjust) - 8u), 8u));
    timing_set(timing, 0x050, 0u);
    timing_set(timing, 0x16c, timing_u32(timing, 0x048) * 4u);
    timing_set(timing, 0x054,
               (uint32_t)(((uint64_t)frequency * UINT64_C(70200) + 999u) / 1000u) <
               t3900 * 9u ?
               (uint32_t)(((uint64_t)frequency * UINT64_C(70200) + 999u) / 1000u) :
               t3900 * 9u);
    timing_set(timing, 0x058, t42);
    timing_set(timing, 0x05c, t40);

    if (max_geometry > UINT32_C(0x40000000)) {
        timing_set(timing, 0x170, cycles_from_ns_x1000(frequency, 90u));
        timing_set(timing, 0x064, cycles_from_ns_x1000(frequency, 190u));
        timing_set(timing, 0x060, cycles_from_ns_x1000(frequency, 380u));
        density_coeff = 387u;
    } else if (max_geometry > UINT32_C(0x20000000)) {
        timing_set(timing, 0x170, cycles_from_ns_x1000(frequency, 90u));
        timing_set(timing, 0x064, cycles_from_ns_x1000(frequency, 140u));
        timing_set(timing, 0x060, cycles_from_ns_x1000(frequency, 280u));
        density_coeff = 287u;
    } else if (max_geometry > UINT32_C(0x10000000)) {
        timing_set(timing, 0x170, cycles_from_ns_x1000(frequency, 90u));
        timing_set(timing, 0x064, cycles_from_ns_x1000(frequency, 900u));
        timing_set(timing, 0x060, cycles_from_ns_x1000(frequency, 180u));
        density_coeff = 187u;
    } else {
        const uint32_t t60 = cycles_from_ns_x1000(frequency, 60u);
        timing_set(timing, 0x170, t60);
        timing_set(timing, 0x064, t60);
        timing_set(timing, 0x060, cycles_from_ns_x1000(frequency, 130u));
        density_coeff = 137u;
    }

    {
        const uint32_t half_freq = frequency >> 1;
        const uint32_t three_freq = frequency * 3u;
        const uint32_t trfc_like = ceil_div1000_u64((uint64_t)frequency * density_coeff + half_freq);
        const uint32_t t1750 = (frequency + 999u + (three_freq >> 2)) / 1000u;
        const uint32_t t7 = max_u32(ceil_div1000_u64((uint64_t)frequency * 7u + half_freq), 4u);
        const uint32_t t7_min5 = max_u32(t7, 5u);
        const uint32_t t15 = max_u32(cycles_from_ns_x1000(frequency, 15u), 3u);
        const uint32_t t14 = max_u32(cycles_from_ns_x1000(frequency, 14u), 10u);
        const uint32_t t30_min8 = max_u32(t30, 8u);
        const uint32_t t1750b = (frequency + 999u + half_freq) / 1000u;
        const uint32_t t3500 = (half_freq + three_freq + 999u) / 1000u;
        const uint32_t t3over5 = (three_freq / 5u + three_freq + 999u) / 1000u;
        const uint32_t t1000 = cycles_from_ns_x1000(frequency, 1000u);
        const uint32_t t50 = max_u32(cycles_from_ns_x1000(frequency, 50u), 3u);
        const uint32_t t20 = cycles_from_ns_x1000(frequency, 20u);
        const uint32_t t250 = cycles_from_ns_x1000(frequency, 250u);
        const uint32_t t100 = cycles_from_ns_x1000(frequency, 100u);
        const uint32_t t200 = cycles_from_ns_x1000(frequency, 200u);

        timing_set(timing, 0x068, half_freq / 1000u);
        timing_set(timing, 0x06c, t3over5);
        timing_set(timing, 0x070, max_u32(trfc_like, 2u));
        timing_set(timing, 0x074, max_u32(trfc_like, 2u));
        timing_set(timing, 0x078, t7_min5);
        timing_set(timing, 0x084, t7);
        timing_set(timing, 0x09c, t14);
        timing_set(timing, 0x0a0, 8u);
        timing_set(timing, 0x0a4, t18 + 3u);
        timing_set(timing, 0x0c8, t1750b);
        timing_set(timing, 0x0d0, t3500);
        timing_set(timing, 0x0d4, t1750b);
        timing_set(timing, 0x0d8, t3500);
        timing_set(timing, 0x100, t50);
        timing_set(timing, 0x104, 40u);
        timing_set(timing, 0x108, t20);
        timing_set(timing, 0x10c, 20u);
        timing_set(timing, 0x114, t250);
        timing_set(timing, 0x124, t20);
        timing_set(timing, 0x128, t1750b);
        timing_set(timing, 0x174, 4u);
        timing_set(timing, 0x178, max_u32(t1750, 3u));
        timing_set(timing, 0x17c, t15);
        timing_set(timing, 0x180, max_u32(t1750, 3u));
        timing_set(timing, 0x184, t1750);
        timing_set(timing, 0x188, max_u32(t5, 5u));
        timing_set(timing, 0x18c, t1750);
        timing_set(timing, 0x190, t7_min5);
        timing_set(timing, 0x194, t14);
        timing_set(timing, 0x198, t1000);
        timing_set(timing, 0x19c, t30_min8);
        timing_set(timing, 0x1a0, max_u32(t1750, 3u));
        timing_set(timing, 0x1a4, t250);
        timing_set(timing, 0x1a8, t100);
        timing_set(timing, 0x1ac, t200);
        timing_set(timing, 0x1b0, t100);
        timing_set(timing, 0x1b4, t250);
        timing_set(timing, 0x1b8, t7);
        timing_set(timing, 0x1bc, t7);
        timing_set(timing, 0x1c0, max_u32(t1750, 3u));
        timing_set(timing, 0x1c4, t7_min5);
    }
}
