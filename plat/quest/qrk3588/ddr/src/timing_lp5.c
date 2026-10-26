/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* LPDDR5 timing-parameter synthesis.
 *
 * Produces the 0x2ac-byte timing image from DRAM geometry, frequency, ratio and
 * mode parameters using the immutable timing tables in the layout image.
 */
#include <stddef.h>
#include <stdint.h>
#include "qrk3588_ddr_internal.h"
#include "qrk3588_ddr_runtime.h"

#define LPDDR4   7u
#define LPDDR4X  8u
#define LPDDR5   9u

static inline uintptr_t lp5_addr(const void *base, size_t off)
{
    return qrk3588_ddr_resolve_addr((uintptr_t)base + off);
}
static inline uint32_t rd32(const void *base, size_t off)
{
    return *(volatile const uint32_t *)(uintptr_t)lp5_addr(base, off);
}
static inline void wr32(void *base, size_t off, uint32_t value)
{
    *(volatile uint32_t *)(uintptr_t)lp5_addr(base, off) = value;
}
static inline uint32_t umax32(uint32_t a, uint32_t b) { return a > b ? a : b; }
static inline uint32_t umin32(uint32_t a, uint32_t b) { return a < b ? a : b; }
static inline uint32_t ceil1000(uint64_t v) { return (uint32_t)((v + 999u) / 1000u); }
static inline uint32_t cyc(uint32_t f, uint32_t ns) { return ceil1000((uint64_t)f * ns); }

/* Immutable LP5 timing rows from the fixed parameter tables. */
static const uint32_t lp5_mr_2to1[6][7] = {
    {5u, 5u, 0u, 0u, 6u, 6u, 0u},
    {10u, 10u, 0u, 0u, 11u, 12u, 16u},
    {14u, 15u, 0u, 0u, 17u, 18u, 32u},
    {19u, 20u, 0u, 0u, 0u, 0u, 48u},
    {24u, 25u, 0u, 0u, 0u, 0u, 64u},
    {28u, 29u, 0u, 0u, 0u, 0u, 80u},
};

static const uint32_t lp5_mr_4to1[14][7] = {
    {3u, 3u, 0u, 0u, 3u, 3u, 0u},
    {5u, 5u, 0u, 0u, 6u, 6u, 16u},
    {7u, 8u, 0u, 0u, 9u, 9u, 32u},
    {10u, 10u, 0u, 0u, 0u, 0u, 48u},
    {12u, 13u, 0u, 0u, 0u, 0u, 64u},
    {14u, 15u, 0u, 0u, 0u, 0u, 80u},
    {16u, 17u, 18u, 19u, 0u, 0u, 96u},
    {19u, 20u, 21u, 22u, 0u, 0u, 112u},
    {21u, 22u, 23u, 24u, 0u, 0u, 128u},
    {24u, 25u, 27u, 28u, 0u, 0u, 144u},
    {26u, 28u, 29u, 31u, 0u, 0u, 160u},
    {28u, 29u, 31u, 32u, 0u, 0u, 176u},
    {32u, 34u, 36u, 38u, 0u, 0u, 192u},
    {37u, 39u, 41u, 43u, 0u, 0u, 208u},
};

static const uint32_t lp5_timing_2to1[6][18] = {
    {6u, 6u, 6u, 4u, 4u, 0u, 0u, 1u, 1u, 1u, 3u, 4u, 0u, 0u, 0u, 6u, 7u, 0u},
    {8u, 8u, 8u, 4u, 6u, 1u, 0u, 0u, 2u, 2u, 3u, 5u, 0u, 0u, 0u, 7u, 9u, 0u},
    {10u, 10u, 12u, 6u, 8u, 2u, 32u, 1u, 3u, 2u, 4u, 6u, 1u, 1u, 3u, 8u, 10u, 1u},
    {12u, 14u, 14u, 8u, 10u, 3u, 48u, 2u, 4u, 3u, 4u, 7u, 2u, 4u, 4u, 8u, 11u, 1u},
    {16u, 16u, 18u, 8u, 14u, 4u, 48u, 1u, 7u, 4u, 4u, 8u, 3u, 3u, 5u, 10u, 14u, 1u},
    {18u, 20u, 20u, 10u, 16u, 5u, 80u, 3u, 9u, 4u, 4u, 8u, 5u, 7u, 7u, 10u, 141u, 2u},
};

static const uint32_t lp5_timing_4to1[14][18] = {
    {3u, 3u, 3u, 2u, 2u, 0u, 0u, 0u, 0u, 1u, 2u, 3u, 0u, 0u, 0u, 3u, 4u, 0u},
    {4u, 4u, 4u, 2u, 3u, 1u, 0u, 0u, 1u, 1u, 2u, 3u, 0u, 0u, 0u, 4u, 5u, 0u},
    {5u, 5u, 6u, 3u, 4u, 2u, 32u, 1u, 2u, 1u, 2u, 3u, 1u, 1u, 2u, 4u, 5u, 1u},
    {6u, 7u, 7u, 4u, 5u, 3u, 48u, 1u, 2u, 2u, 2u, 4u, 1u, 2u, 2u, 4u, 6u, 1u},
    {8u, 8u, 9u, 4u, 7u, 4u, 48u, 1u, 4u, 2u, 2u, 4u, 2u, 2u, 3u, 5u, 7u, 1u},
    {9u, 10u, 10u, 5u, 8u, 5u, 80u, 2u, 5u, 2u, 2u, 4u, 3u, 4u, 4u, 5u, 7u, 1u},
    {10u, 11u, 12u, 6u, 9u, 6u, 96u, 2u, 5u, 3u, 2u, 5u, 3u, 4u, 5u, 5u, 8u, 1u},
    {12u, 13u, 14u, 6u, 11u, 7u, 96u, 2u, 7u, 3u, 2u, 5u, 4u, 5u, 6u, 6u, 9u, 1u},
    {13u, 14u, 15u, 7u, 12u, 8u, 128u, 3u, 8u, 3u, 2u, 5u, 5u, 6u, 7u, 6u, 9u, 2u},
    {15u, 16u, 17u, 8u, 14u, 9u, 144u, 3u, 9u, 4u, 2u, 6u, 6u, 7u, 8u, 6u, 10u, 2u},
    {16u, 17u, 19u, 9u, 15u, 10u, 160u, 4u, 10u, 4u, 2u, 6u, 6u, 7u, 9u, 7u, 11u, 2u},
    {17u, 18u, 20u, 9u, 16u, 11u, 160u, 4u, 11u, 4u, 2u, 6u, 7u, 8u, 10u, 7u, 11u, 2u},
    {20u, 22u, 24u, 11u, 19u, 12u, 192u, 5u, 13u, 5u, 2u, 7u, 7u, 9u, 11u, 9u, 14u, 3u},
    {23u, 25u, 26u, 12u, 22u, 13u, 208u, 5u, 15u, 6u, 2u, 8u, 8u, 10u, 11u, 10u, 16u, 3u},
};

static const uint32_t lp5_timing_dbi[8][18] = {
    {12u, 13u, 0u, 6u, 9u, 6u, 96u, 2u, 5u, 3u, 2u, 5u, 5u, 6u, 0u, 5u, 6u, 1u},
    {13u, 14u, 0u, 6u, 11u, 7u, 96u, 2u, 7u, 3u, 2u, 5u, 5u, 6u, 0u, 6u, 9u, 1u},
    {15u, 16u, 0u, 7u, 12u, 8u, 128u, 3u, 8u, 3u, 2u, 5u, 7u, 8u, 0u, 6u, 9u, 2u},
    {17u, 18u, 0u, 8u, 14u, 9u, 144u, 3u, 9u, 4u, 2u, 6u, 8u, 9u, 0u, 6u, 10u, 2u},
    {18u, 20u, 0u, 9u, 15u, 10u, 160u, 4u, 10u, 4u, 2u, 6u, 8u, 10u, 0u, 7u, 11u, 2u},
    {19u, 21u, 0u, 9u, 16u, 11u, 160u, 4u, 11u, 4u, 2u, 6u, 9u, 11u, 0u, 7u, 11u, 2u},
    {23u, 24u, 0u, 11u, 19u, 12u, 192u, 5u, 13u, 5u, 2u, 7u, 10u, 11u, 0u, 9u, 14u, 3u},
    {26u, 28u, 0u, 12u, 22u, 13u, 208u, 5u, 15u, 6u, 2u, 8u, 11u, 13u, 0u, 10u, 16u, 3u},
};


static int lp5_frequency_bin(uint32_t data_rate, uint32_t ratio_mode)
{
    static const uint16_t limit_2to1[6] = {533u, 1067u, 1600u, 2133u, 2750u, 3200u};
    static const uint16_t limit_4to1[14] = {
        533u, 1067u, 1600u, 2133u, 2750u, 3200u, 3733u,
        4267u, 4800u, 5500u, 6000u, 6400u, 7500u, 8533u
    };
    unsigned i, n = ratio_mode == 0x80u ? 6u : 14u;
    const uint16_t *limit = ratio_mode == 0x80u ? limit_2to1 : limit_4to1;
    for (i = 0; i < n; ++i)
        if (data_rate <= limit[i]) return (int)i;
    return -1;
}

static uint32_t encode_drive_base(uint32_t v)
{
    switch (v) {
    case 0x30u: return 6u;
    case 0x3cu: return 4u;
    case 0x50u: return 3u;
    case 0x78u: return 2u;
    case 0xf0u: return 1u;
    default:    return 0u;
    }
}
static uint32_t encode_drive_upper(uint32_t v)
{
    switch (v) {
    case 0x30u: return 0x50u;
    case 0x3cu: return 0x40u;
    case 0x50u: return 0x30u;
    case 0x78u: return 0x20u;
    case 0xf0u: return 0x10u;
    default:    return 0x60u;
    }
}
static uint32_t encode_drive_200(uint32_t v)
{
    switch (v) {
    case 0x30u: return 0xa0u;
    case 0x3cu: return 0x80u;
    case 0x50u: return 0x60u;
    case 0x78u: return 0x40u;
    case 0xf0u: return 0x20u;
    default:    return 0xc0u;
    }
}

static uint32_t lp5_max_rank_geometry(const uint8_t *dram)
{
    uint32_t count = rd32(dram, 0x30u), maxv = 0u;
    for (uint32_t i = 0; i < count; ++i) {
        size_t o = 4u + (size_t)i * 0x14u;
        uint32_t ranks = rd32(dram, o);
        if (!ranks) continue;
        maxv = umax32(maxv, rd32(dram, o + 4u));
        if (ranks > 1u) maxv = umax32(maxv, rd32(dram, o + 8u));
    }
    return maxv;
}

static void lp5_select_mode_indices(const uint8_t *dram, uint32_t clock,
                                    uint32_t *row_variant, uint32_t *mr_variant)
{
    const uint32_t width = rd32(dram, 0x10u);
    const uint32_t high_mode = rd32(dram, 0x6cu);
    const uint32_t selector = rd32(dram, 0x44u);

    *row_variant = 0u;
    *mr_variant = 0u;
    if (high_mode != 0u) {
        if (clock > 1599u && width == 8u) {
            *row_variant = 1u;
            *mr_variant = 1u;
        }
        return;
    }
    if (width == 8u) {
        *mr_variant = 1u;
        *row_variant = selector ? 2u : 1u;
    } else if (selector) {
        *row_variant = 1u;
    }
}

int ddr_timing_parameter_synthesize(uint8_t *dram, uint8_t *timing)
{
    const uint32_t type = rd32(dram, 0x28u);
    uint32_t clock, rate, tck, gear;
    uint32_t row_variant, mr_variant, max_geometry;
    const uint32_t *mrrow, *tbl;
    int bin;
    uint32_t p34, p40, p44, p48, p50, p54, p58, p60, p64, p68, p6c;
    uint32_t p70, p74, p78, p7c, p80, p84, p8c, p90;
    uint32_t local68_nonzero;
    uint32_t t1e8, t1ec, t1f0, t204, t208, t20c, t210, t220;
    uint32_t t23c, t264, t26c, tcc, td8;
    uint32_t normalized_8c, correction5, correction6;

    if (type == LPDDR4 || type == LPDDR4X) {
        ddr_timing_base_calculate(dram, timing, type);
        return 0;
    }
    if (type != LPDDR5) return -1;

    clock = rd32(dram, 0x2cu);
    rate = clock << 1;
    p34 = rd32(dram, 0x34u);
    p40 = rd32(dram, 0x40u); p44 = rd32(dram, 0x44u);
    p48 = rd32(dram, 0x48u); p50 = rd32(dram, 0x50u);
    p54 = rd32(dram, 0x54u); p58 = rd32(dram, 0x58u);
    p60 = rd32(dram, 0x60u); p64 = rd32(dram, 0x64u);
    p68 = rd32(dram, 0x68u); p6c = rd32(dram, 0x6cu);
    p70 = rd32(dram, 0x70u); p74 = rd32(dram, 0x74u);
    p78 = rd32(dram, 0x78u); p7c = rd32(dram, 0x7cu);
    p80 = rd32(dram, 0x80u); p84 = rd32(dram, 0x84u);
    p8c = rd32(dram, 0x8cu); p90 = rd32(dram, 0x90u);
    local68_nonzero = p68 != 0u;
    max_geometry = lp5_max_rank_geometry(dram);

    if (p60 == 0x80u) { tck = clock >> 1; gear = 2u; }
    else               { tck = clock >> 2; gear = 4u; }

    if (p6c != 0u && clock <= 1599u) return -1;
    lp5_select_mode_indices(dram, clock, &row_variant, &mr_variant);

    bin = lp5_frequency_bin(rate, p60);
    if (bin < 0) return -1;
    if (p60 == 0x80u) {
        mrrow = lp5_mr_2to1[bin];
        tbl = lp5_timing_2to1[bin];
    } else {
        mrrow = lp5_mr_4to1[bin];
        if (p6c != 0u && bin >= 6) tbl = lp5_timing_dbi[bin - 6];
        else                       tbl = lp5_timing_4to1[bin];
    }

    ddr_memset(timing, 0, 0x2acu);
    wr32(timing, 0x000u, clock);
    wr32(timing, 0x004u, ceil1000((uint64_t)tck * 200000u));
    wr32(timing, 0x008u, ceil1000((uint64_t)tck * 10u));
    wr32(timing, 0x00cu, ceil1000((uint64_t)tck * 2000000u));
    wr32(timing, 0x010u, 5u);
    wr32(timing, 0x014u, ceil1000((uint64_t)tck * 2000u));
    wr32(timing, 0x018u, ceil1000((uint64_t)tck * 200000u));
    wr32(timing, 0x01cu, ceil1000((uint64_t)tck * 2000000u));
    wr32(timing, 0x168u, (p6c << 6) | (p68 << 4));
    wr32(timing, 0x1e4u, 0u);
    wr32(timing, 0x1f0u, p34);

    wr32(timing, 0x1ecu, tbl[3]);
    wr32(timing, 0x210u, tbl[7]);
    wr32(timing, 0x214u, tbl[9]);
    wr32(timing, 0x218u, tbl[10]);
    wr32(timing, 0x21cu, tbl[11]);
    wr32(timing, 0x1e8u, tbl[row_variant]);
    wr32(timing, 0x220u, tbl[12u + row_variant]);
    wr32(timing, 0x224u, tbl[15]);
    wr32(timing, 0x228u, tbl[16]);
    wr32(timing, 0x22cu, tbl[17]);
    wr32(timing, 0x1f4u, mrrow[(local68_nonzero ? 2u : 0u) + mr_variant]);
    wr32(timing, 0x134u, tbl[6]);
    wr32(timing, 0x138u, tbl[5] | mrrow[6]);
    wr32(timing, 0x1f8u, (p7c << 1) | (p78 << 4) | (p8c << 2) | (p80 << 6) | p84);

    if (p60 == 0x80u) {
        uint32_t q = p34 >> 2;
        t204 = t208 = t20c = q;
        t264 = p80;
        t23c = umax32(ceil1000((uint64_t)tck * 7u + (tck >> 1)), 4u) - 4u;
        correction6 = p80 == 3u ? 4u : 2u;
        wr32(timing, 0x260u, p80 + 1u);
        wr32(timing, 0x280u, p8c + 2u);
        wr32(timing, 0x284u, p8c + 1u);
        correction5 = p80;
    } else {
        uint32_t q = p34 >> 2;
        uint32_t aux = 0u;
        if (p64 != 0u) {
            q = p34 >> 3;
            t204 = t208 = t20c = q;
        } else {
            aux = (p34 == 16u) ? 2u : (p34 >> 4);
            t204 = t20c = q;
            t208 = p34 == 16u ? 2u : aux * 3u;
        }
        t264 = p80 == 2u ? 1u : 0u;
        wr32(timing, 0x260u, t264 + 1u);
        correction6 = p80 == 3u ? 2u : 1u;
        correction5 = p80 != 0u ? 1u : 0u;
        wr32(timing, 0x280u, p8c != 0u ? 2u : 1u);
        wr32(timing, 0x284u, p8c != 0u ? 1u : 0u);
        t23c = umax32(ceil1000((uint64_t)tck * 7u + (tck >> 1)), 2u) - 2u;
        wr32(timing, 0x240u, aux);
    }
    wr32(timing, 0x204u, t204);
    wr32(timing, 0x208u, t208);
    wr32(timing, 0x20cu, t20c);
    wr32(timing, 0x23cu, t23c);
    wr32(timing, 0x264u, t264);
    wr32(timing, 0x0a0u, p60 == 0x80u ? 8u : 4u);

    {
        uint32_t mr13c = (p48 << 7) | (p44 << 6) | (p64 | 6u);
        uint32_t mr14c = 0u;
        uint32_t mr200 = 0u;
        if (p40)
            mr14c = encode_drive_base(p50) | 0x08u;
        if (p54 != 0u)
            mr14c |= encode_drive_upper(p54);
        if (p40 && p58 != 0u)
            mr200 = encode_drive_200(p58);
        wr32(timing, 0x13cu, mr13c);
        wr32(timing, 0x14cu, mr14c);
        wr32(timing, 0x200u, mr200);
    }
    wr32(timing, 0x154u, 0u);
    wr32(timing, 0x1fcu, p60 | (rate > 3200u ? 8u : 0u) | (p70 ? 0x10u : 0u));
    wr32(timing, 0x164u, p74);

    wr32(timing, 0x038u, (p64 ? t204 : t208) + t23c);
    wr32(timing, 0x048u, t204);
    wr32(timing, 0x020u, cyc(tck, 3900u));

    if (max_geometry > UINT32_C(0x80000000)) {
        wr32(timing, 0x060u, cyc(tck, 380u));
        wr32(timing, 0x064u, cyc(tck, 190u));
    } else if (max_geometry > UINT32_C(0x40000000)) {
        wr32(timing, 0x060u, cyc(tck, 280u));
        wr32(timing, 0x064u, cyc(tck, 140u));
    } else if (max_geometry > UINT32_C(0x20000000)) {
        wr32(timing, 0x060u, cyc(tck, 210u));
        wr32(timing, 0x064u, cyc(tck, 120u));
    } else if (max_geometry > UINT32_C(0x10000000)) {
        wr32(timing, 0x060u, cyc(tck, 180u));
        wr32(timing, 0x064u, cyc(tck, 90u));
    } else {
        wr32(timing, 0x060u, cyc(tck, 130u));
        wr32(timing, 0x064u, cyc(tck, 60u));
    }
    wr32(timing, 0x170u, cyc(tck, max_geometry > UINT32_C(0x10000000) ? 90u : 60u));

    {
        uint32_t t20 = rd32(timing, 0x020u);
        uint32_t t8 = rd32(timing, 0x008u);
        uint32_t t24 = umax32(cyc(tck, 18u), 2u);
        uint32_t t2c = umax32(cyc(tck, 21u), 2u);
        uint32_t t58 = umax32(cyc(tck, 42u), 3u);
        uint32_t t30 = umax32(cyc(tck, 34u), 3u);
        uint32_t t44 = umax32(ceil1000((uint64_t)tck * 5u), 2u);
        uint32_t t5c = ceil1000((uint64_t)tck * 20u);
        uint32_t t238 = umax32(ceil1000((uint64_t)tck * 6u + (tck >> 2)), 4u);
        uint32_t t1d0 = umax32(cyc(tck, 12u), 4u);
        uint32_t t54 = umin32(ceil1000((uint64_t)tck * 70200u), t20 * 9u);

        wr32(timing, 0x174u, 2u);
        wr32(timing, 0x024u, t24); wr32(timing, 0x028u, t24);
        wr32(timing, 0x02cu, t2c); wr32(timing, 0x030u, t30);
        wr32(timing, 0x03cu, t2c + t58); wr32(timing, 0x040u, t24 + t58);
        wr32(timing, 0x044u, t44); wr32(timing, 0x054u, t54);
        wr32(timing, 0x058u, t58); wr32(timing, 0x05cu, t5c);
        wr32(timing, 0x238u, t238); wr32(timing, 0x1d0u, t1d0);
        wr32(timing, 0x230u, p64 == 8u ? t8 : umax32(ceil1000((uint64_t)tck * 7u + (tck >> 1)), 0u));

        if (rd32(dram, 0x10u) == 16u) {
            if (p64 == 16u) wr32(timing, 0x04cu, t1d0);
            else if (p64 == 8u) {
                wr32(timing, 0x044u, umax32(t8, 2u));
                wr32(timing, 0x05cu, ceil1000((uint64_t)tck * 40u));
                wr32(timing, 0x04cu, t1d0);
            }
        } else if (rd32(dram, 0x10u) == 8u) {
            if (p64 == 0u) {
                wr32(timing, 0x030u, umax32(cyc(tck,36u),3u));
                wr32(timing, 0x238u, umax32(ceil1000((uint64_t)tck * 8u + (tck >> 2)),4u));
                wr32(timing, 0x1d0u, umax32(cyc(tck,14u),4u));
            } else if (p64 == 16u) {
                wr32(timing, 0x030u, umax32(cyc(tck,36u),3u));
                wr32(timing, 0x04cu, umax32(cyc(tck,14u),4u));
            } else if (p64 == 8u) {
                wr32(timing, 0x044u, umax32(t8,2u));
                wr32(timing, 0x05cu, ceil1000((uint64_t)tck * 40u));
                wr32(timing, 0x030u, umax32(cyc(tck,36u),3u));
                wr32(timing, 0x04cu, umax32(cyc(tck,14u),4u));
            }
        }

        wr32(timing, 0x274u, umax32(t8,5u));
        wr32(timing, 0x0a4u, t24 + 2u);
        wr32(timing, 0x234u, 2u); wr32(timing, 0x250u, 2u);
        wr32(timing, 0x09cu, umax32(cyc(tck,14u),5u));
        wr32(timing, 0x070u,
             umax32(ceil1000((uint64_t)tck * 7u + (tck >> 1)),2u) + rd32(timing,0x060u));
        wr32(timing, 0x17cu, umax32(cyc(tck,15u),2u));
        wr32(timing, 0x244u, t8 + 1u);
        wr32(timing, 0x248u, 3u);
        wr32(timing, 0x24cu, umax32(ceil1000((uint64_t)tck * 5u),3u));
        wr32(timing, 0x078u, umax32(cyc(tck,7u),3u));
        wr32(timing, 0x25cu, umax32(rd32(timing,0x09cu),6u));
        wr32(timing, 0x254u, (clock + 999u + ((rate + clock) >> 2)) / 1000u);
        wr32(timing, 0x258u, ceil1000((uint64_t)tck * 3u));
        wr32(timing, 0x268u, (uint32_t)(((uint64_t)tck * 6u + 9999u) / 10000u));
        wr32(timing, 0x26cu, (uint32_t)(((uint64_t)tck * (rate <= 3200u ? 19u : 16u) + 9999u) / 10000u));
        wr32(timing, 0x270u, (uint32_t)(((uint64_t)tck * (rate <= 3200u ? 1900u : 1600u)) / 1000000u));
        wr32(timing, 0x278u, t8 + 1u);
        wr32(timing, 0x27cu, ceil1000((uint64_t)tck * 200000u));
        {
            uint32_t half = tck >> 1;
            uint32_t floor15 = (tck + half) / 1000u;
            uint32_t ceil15 = (tck + half + 999u) / 1000u;
            uint32_t threehalf = (half + 3u*tck + 999u) / 1000u;
            wr32(timing, 0x0ccu, floor15);
            wr32(timing, 0x0d0u, threehalf);
            wr32(timing, 0x0d8u, threehalf);
            wr32(timing, 0x0c8u, ceil15);
            wr32(timing, 0x0d4u, ceil15);
        }
        wr32(timing, 0x294u, cyc(tck,30u));
        wr32(timing, 0x288u, cyc(tck,1500u));
        wr32(timing, 0x28cu, cyc(tck,3000u));
        wr32(timing, 0x290u, cyc(tck,6000u));
        wr32(timing, 0x19cu, umax32(rd32(timing,0x294u),4u));
        wr32(timing, 0x100u, umax32(cyc(tck,50u),3u));
        wr32(timing, 0x104u, umax32(cyc(tck,14u),14u));
        wr32(timing, 0x108u, umax32(t5c,14u));
    }

    t1e8 = rd32(timing,0x1e8u); t1ec=rd32(timing,0x1ecu); t1f0=rd32(timing,0x1f0u);
    t204=rd32(timing,0x204u); t208=rd32(timing,0x208u); t20c=rd32(timing,0x20cu);
    t210=rd32(timing,0x210u); t220=rd32(timing,0x220u); t264=rd32(timing,0x264u);
    t26c=rd32(timing,0x26cu); tcc=rd32(timing,0x0ccu); td8=rd32(timing,0x0d8u);
    normalized_8c = (p60 == 0x80u) ? (p8c + 1u) : (p8c == 2u ? 2u : 1u);

    if (p60 == 0u) {
        uint32_t sub = rate <= 1600u ? 1u : (rate <= 3733u ? 2u : 3u);
        wr32(timing,0x0a8u,t1ec-sub);
        if (p64 == 16u || p64 == 8u)
            wr32(timing,0x0acu,t1ec + (t1f0 != 16u ? 5u : 3u));
        else if (p64 == 0u)
            wr32(timing,0x0acu,t1ec + (t1f0 != 16u ? 7u : 3u));
        sub = rate <= 1600u ? 1u : (rate <= 3733u ? 2u : 3u);
        wr32(timing,0x0b4u,t1e8-sub);
        if (rate <= 3200u) {
            uint32_t b0=t1e8+(t1f0!=16u?5u:3u);
            wr32(timing,0x0b0u,b0);
        } else {
            uint32_t b0=t1e8+(rate>3733u?4u:3u)+(p64==8u?2u:4u);
            wr32(timing,0x0b0u,b0);
        }
    } else {
        uint32_t sub = rate <= 533u ? 1u : (rate <= 1600u ? 2u : (rate <= 2750u ? 3u : 4u));
        wr32(timing,0x0a8u,t1ec-sub);
        if (p64 == 16u || p64 == 8u)
            wr32(timing,0x0acu,t1ec + (t1f0 != 16u ? 9u : 5u));
        else if (p64 == 0u)
            wr32(timing,0x0acu,t1ec + (t1f0 != 16u ? 7u : 3u));
        wr32(timing,0x0b4u,t1e8-sub);
        {
            uint32_t b0=t1e8+(t1f0!=16u?9u:5u);
            if (rate>1600u) b0++;
            wr32(timing,0x0b0u,b0);
        }
    }

    /* Vendor code snapshots the unadjusted pair to B8/BC first. */
    wr32(timing,0x0b8u,rd32(timing,0x0b0u));
    wr32(timing,0x0bcu,rd32(timing,0x0b4u));
    if (p74 != 0u) {
        uint32_t b0=rd32(timing,0x0b0u), b4=rd32(timing,0x0b4u);
        if (p78 == 3u) {
            if (p60 == 0u) { b0 += 1u; b4 -= 2u; }
        } else if (p60 == 0x80u) { b0 += 2u; b4 -= 2u; }
        else { b0 += 1u; b4 -= 1u; }
        wr32(timing,0x0b0u,b0); wr32(timing,0x0b4u,b4);
    }
    wr32(timing,0x0c0u,rd32(timing,0x0b0u));
    wr32(timing,0x0c4u,rd32(timing,0x0b4u));

    {
        uint32_t t50;
        if (p64 == 0u) {
            if (p50 != 0u)
                t50 = t26c + t264 + t1e8 + t20c - (tcc + rd32(timing,0x0a8u)) + 1u;
            else
                t50 = t26c - t1ec + t1e8 + t20c;
            if (p50 != 0u && p58 != 0u && p68 == 0u && p90 == 0u)
                t50 += 1u;
        } else {
            if (p50 != 0u)
                t50 = t26c + t264 + t1e8 + t204 - rd32(timing,0x0a8u) + 1u - tcc;
            else
                t50 = t26c - t1ec + t1e8 + t204;
            if (p50 != 0u && p74 && p58 != 0u && p68 == 0u && p90 == 0u)
                t50 += (p60 == 0x80u) ? 2u : 1u;
        }
        wr32(timing,0x050u,t50);
        wr32(timing,0x2a8u,t50 + t208 - t20c);
        if (p68) wr32(timing,0x050u,t50+1u);
    }

    if (p70 != 0u) {
        uint32_t v298,v29c,v2a0,v2a4;
        if (p50 != 0u) {
            uint32_t sum = rd32(timing,0x0acu) + td8;
            if (p58 != 0u) {
                v298 = rd32(timing,0x0b0u) - rd32(timing,0x0b4u);
                v29c = sum - rd32(timing,0x0a8u);
                v2a4 = sum - t1e8;
                v2a0 = t26c + 1u + t1e8 + t208 - (rd32(timing,0x0a8u)+tcc);
            } else {
                v29c = sum - rd32(timing,0x0a8u);
                v2a4 = sum - t1e8;
                v2a0 = t26c + t1e8 - (rd32(timing,0x0a8u)+tcc) + 1u;
                v298 = 4u / gear + correction6 + (t208+1u);
            }
        } else {
            v29c = t208 + 1u;
            v298 = 4u / gear + (t208+1u) + correction6;
            v2a0 = t26c - t1ec + (t208+t1e8) + correction5;
            v2a4 = 1u - t1e8 + (t208+t1ec);
        }
        /* In the timing flow w2 still carries p90 only on the p50!=0 path.
         * The p50==0 path reuses w2 for (t208+t1ec), so the final correction
         * is controlled only by p6c/p68 there. */
        if ((p50 != 0u && p58 != 0u && p90 == 0u) || p6c != 0u || p68 != 0u) {
            v2a0 += correction5;
            v2a4 += correction6;
        }
        wr32(timing,0x298u,v298); wr32(timing,0x29cu,v29c);
        wr32(timing,0x2a0u,v2a0); wr32(timing,0x2a4u,v2a4);
    } else {
        uint32_t v298,v29c,v2a0,v2a4;
        if (p50 != 0u) {
            uint32_t a=t20c+t1e8+normalized_8c;
            uint32_t b=t20c+t1ec+umax32(td8,normalized_8c);
            v298=a-t220; v29c=b-t210;
            v2a0=a-umin32(t1ec+tcc,t210); v2a4=b-t220;
        } else {
            uint32_t a=t20c+t1e8+normalized_8c, b=t20c+t1ec+normalized_8c;
            v298=a-t220; v2a0=a-t210; v29c=b-t210; v2a4=b-t220;
        }
        wr32(timing,0x298u,v298); wr32(timing,0x29cu,v29c);
        wr32(timing,0x2a0u,v2a0); wr32(timing,0x2a4u,v2a4);
    }
    wr32(timing,0x1ccu,rd32(timing,0x044u));
    return 0;
}
