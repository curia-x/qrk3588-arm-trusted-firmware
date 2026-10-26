/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* RK3588 DDR PHY programming, state capture/restore and update helpers. */

#include "qrk3588_ddr_internal.h"


void ddr_phy_channel_init(void *ctx)
{
    volatile uint32_t *ctrl = (volatile uint32_t *)qrk3588_ddr_ctx_clock_table(ctx);

    ctrl[0x850u / 4u] = 0x22012201u;
    ctrl[0x854u / 4u] = 0x00200020u;
    ctrl[0x85cu / 4u] = 0x22012201u;
    ctrl[0x860u / 4u] = 0x00200020u;
    ctrl[0x850u / 4u] = 0x88048804u;
    ctrl[0x854u / 4u] = 0x00800080u;
    ctrl[0x85cu / 4u] = 0x88048804u;
    ctrl[0x860u / 4u] = 0x00800080u;
    ctrl[0x858u / 4u] = 0x00630063u;
    ctrl[0x864u / 4u] = 0x00630063u;

    for (uint32_t route = 0u; route < QRK3588_DDR_CHANNEL_COUNT; ++route) {
        uint32_t channel = qrk3588_ddr_ctx_route(ctx, route);
        if (channel != QRK3588_DDR_CHANNEL_UNUSED) {
            volatile uint32_t *phy = (volatile uint32_t *)
                qrk3588_ddr_ctx_channel_base1(ctx, channel);
            volatile uint32_t *table = (volatile uint32_t *)
                qrk3588_ddr_ctx_shared_bank(ctx, channel);
            uint32_t table_offset = ((channel & 1u) * 0x30u) + 0x1cu;

            phy[0x18u / 4u] |= 0x1000u;
            table[table_offset / 4u] = 0x000f000fu;
        }
    }
}

/* PHY command programming */

/* Issue/finish the per-channel PHY command. */
int ddr_phy_command(const uint32_t *freq_mhz, void *ctx,
                    uint32_t channel, uint32_t mode,
                    uint32_t completion_token)
{
    volatile uint32_t *phy;
    uint32_t status_after;
    uint32_t errors_after;
    uint32_t status_sel;
    uint32_t command_ready;
    uint32_t reg;

    if (*freq_mhz <= 799u)
        return 0;

    phy = (volatile uint32_t *)qrk3588_ddr_ctx_channel_base1(ctx, channel);
    status_sel = phy[0xb8u / 4u] >> 30;
    if (status_sel == 0u) {
        command_ready = (phy[4u / 4u] >> 28) & 1u;
    } else {
        uintptr_t offset = 0xbcu + ((uintptr_t)(status_sel - 1u) << 2);
        command_ready = (phy[offset / 4u] >> 10) & 1u;
    }

    if (!command_ready) {
        if (mode == 1u)
            return 0;

        if (mode == 3u)
            mode = 0u;
    }

    /* Token 1 means finish an already-issued command.  Other nonzero
     * tokens issue a new command first and then wait for completion. */
    if (completion_token == 1u) {
        goto finish;
    }

    phy[0xacu / 4u] = 0x000055aau;
    phy[0xa7cu / 4u] = 0xaa550000u;
    phy[0xa24u / 4u] |= 0x20u;
    reg = phy[0xa24u / 4u] & ~0x3000u;
    reg |= (mode == 3u) ? 0x3000u : (0x1000u << mode);
    phy[0xa24u / 4u] = reg;
    phy[0xa24u / 4u] |= 1u;

    if (completion_token == 0u)
        return 0;

finish:
    if (qrk3588_ddr_poll32((uintptr_t)&phy[0xa24u / 4u], 2u, 2u, 0u) != 0)
        return QRK3588_DDR_ETIMEDOUT;
    phy[0xa24u / 4u] &= ~1u;
    phy[0xa24u / 4u] &= ~0x3000u;
    phy[0xa24u / 4u] &= ~0x20u;
    status_after = phy[0xe8u / 4u];
    errors_after = phy[0xecu / 4u];
    if ((status_after & 0xffffu) != 0u)
        return -1;
    return (errors_after & 3u) ? -1 : 0;
}

/* PHY lane calibration */

/*
 * Program one PHY lane calibration handshake and report its completion/error field.
 */
int ddr_phy_lane_calibrate(void *lane_table, uint32_t lane,
                                       uint32_t select)
{
    volatile uint32_t *phy = *(volatile uint32_t **)
        ((uint8_t *)lane_table + ((uintptr_t)lane << 5) + 8u);
    uint32_t cmd;

    phy[0xacu / 4u] = 0xeeee5555u;
    phy[0xa7cu / 4u] = 0x5555eeeeu;

    phy[0xa24u / 4u] |= 0x10u;
    cmd = phy[0xa24u / 4u] & ~0x0c00u;
    if (select == 3u)
        cmd |= 0x0c00u;
    else
        cmd |= 0x400u << select;
    phy[0xa24u / 4u] = cmd;

    phy[0xa24u / 4u] |= 1u;
    if (qrk3588_ddr_poll32((uintptr_t)&phy[0xa24u / 4u], 2u, 2u, 0u) != 0)
        return QRK3588_DDR_ETIMEDOUT;
    phy[0xa24u / 4u] &= ~1u;
    phy[0xa24u / 4u] &= ~0x0c00u;
    phy[0xa24u / 4u] &= ~0x10u;

    return (phy[0x594u / 4u] & 0x0c0c0000u) ? -1 : 0;
}

/* PHY mode configuration */

/* Select the short or full PHY timing programming path. */
void ddr_phy_mode_config(void *ctx, uint32_t channel,
                                      void *state, uint32_t short_path,
                                      uint32_t extra)
{
    volatile uint32_t *phy = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base1(ctx, channel);

    if (short_path == 1u) {
        const uint8_t *cfg = (const uint8_t *)qrk3588_ddr_state_dram_params(state);
        uint32_t dram_type = *(const uint32_t *)(cfg + 0x28u);
        uint32_t reg;

        ddr_phy_timing_select(ctx, channel, state, 0u);
        reg = phy[0] & 0xfcf7ffffu;
        if (dram_type == 9u)
            reg |= 0x02080000u;
        phy[0] = reg;
    } else {
        ddr_phy_frequency_register_image_build(ctx, channel, state, 0u, 0u, extra);
        phy[0xa1cu / 4u] &= ~0x00040000u;
    }
}

/* PHY register programming sequence A */

/* Native bounded-wait adapter for legacy void register-image helpers.
 * Frozen code cannot advance after an indirect PHY transaction stalls; stop
 * the void sequence immediately once the Native runtime latches an error. */
static int phy_reg_write_native_failed(uint32_t phy_base, uint32_t rank,
                                       uint32_t reg, uint32_t value)
{
    (void)ddr_phy_reg_write(phy_base, rank, reg, value);
    return qrk3588_ddr_get_error() != 0;
}

/* Apply the packed PHY register set selected by frequency index. */
void ddr_phy_apply_profile_registers(uint32_t phy_base, uint32_t selector,
                                      uint32_t dram_type)
{
    uintptr_t block = (uintptr_t)phy_base + ((uintptr_t)selector << 20);
    uint32_t word500 = *(volatile const uint32_t *)(block + 0x500u);
    uint32_t word504 = *(volatile const uint32_t *)(block + 0x504u);
    uint32_t word508 = *(volatile const uint32_t *)(block + 0x508u);
    uint32_t word50c = *(volatile const uint32_t *)(block + 0x50cu);
    uint32_t high = word50c >> 16;

    if (phy_reg_write_native_failed(phy_base, 3u, 3u, word504 >> 16)) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 1u, word500 >> 16)) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 2u, word500 & 0xffffu)) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 11u, word508 >> 16)) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 12u, word508 & 0xffffu)) return;

    if (dram_type == 7u) {
        if (phy_reg_write_native_failed(phy_base, 3u, 22u, high)) return;
    } else {
        if (phy_reg_write_native_failed(phy_base, 1u, 22u, high)) return;
        if (phy_reg_write_native_failed(phy_base, 2u, 22u,
                                        (high & 0xc7u) | 0x28u)) return;
    }
    (void)phy_reg_write_native_failed(phy_base, 3u, 14u, word50c & 0xffffu);
}

/* PHY register programming sequence B */

/* Apply a packed PHY byte configuration block. */
void ddr_phy_apply_byte_config(uint32_t phy_base, const uint8_t *cfg)
{
    if (phy_reg_write_native_failed(phy_base, 3u, 1u, cfg[0])) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 2u, cfg[1])) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 3u, cfg[2])) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 10u, cfg[3])) return;

    if (phy_reg_write_native_failed(phy_base, 1u, 11u, cfg[4])) return;
    if (phy_reg_write_native_failed(phy_base, 2u, 11u, cfg[4] & 0x8fu)) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 12u, cfg[5])) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 13u, cfg[6])) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 14u, cfg[7])) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 15u, cfg[8])) return;

    if (phy_reg_write_native_failed(phy_base, 1u, 17u, cfg[10])) return;
    if (phy_reg_write_native_failed(phy_base, 2u, 17u, cfg[10] | 0x28u)) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 18u, cfg[11])) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 20u, cfg[12])) return;
    if (phy_reg_write_native_failed(phy_base, 3u, 25u, cfg[13])) return;
    (void)phy_reg_write_native_failed(phy_base, 3u, 41u, cfg[14]);
}

/* PHY margin restore */

/* Derive the PHY restore margin used by ddr_phy_state_restore. */
uint32_t ddr_phy_restore_margin(void *ctx, uint32_t channel,
                                             void *snapshot)
{
    volatile uint32_t *phy = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base0(ctx, channel);
    volatile uint32_t *bank = (volatile uint32_t *)
        ((uintptr_t)phy + 0x10000u);
    uint32_t dram_type = qrk3588_ddr_state_dram_type(snapshot);
    uint32_t slice_offset = (bank[2] & 3u) << 20;
    uint32_t delay_word = *(volatile uint32_t *)
        ((uintptr_t)phy + slice_offset + 8u);
    uint32_t delay_hi = (delay_word >> 24) & 0x7fu;
    uint32_t delay_lo = (delay_word >> 16) & 0x7fu;
    uint32_t limit_hi = 0u;
    uint32_t limit_mid = 0u;
    uint32_t bias = 2u;
    uint32_t wide_mode = 0u;
    uint32_t scaled_delay;

    if (dram_type == 9u) {
        const uint8_t *config = (const uint8_t *)qrk3588_ddr_state_dram_params(snapshot);

        if (*(const uint32_t *)(config + 0x60u) == 0u) {
            scaled_delay = (delay_hi << 2) - 2u;
            bias = 4u;
            wide_mode = 1u;
        } else {
            scaled_delay = (delay_hi << 1) - 2u;
        }

        uint32_t limit_word = *(volatile uint32_t *)
            ((uintptr_t)phy + slice_offset + 0x590u);
        limit_hi = limit_word >> 24;
        limit_mid = (limit_word >> 16) & 0xffu;
    } else {
        scaled_delay = delay_hi - 2u;
    }

    uint32_t shift = bias + scaled_delay;
    shift >>= wide_mode ? 2u : 1u;
    uint32_t correction = (shift <= 3u) ? 3u - shift : 0u;
    uint32_t margin = (*(volatile uint32_t *)
        ((uintptr_t)phy + slice_offset + 0x580u) >> 16) & 0x3fu;

    if (dram_type == 9u) {
        if (margin > limit_hi)
            margin = limit_hi;
        if (scaled_delay > limit_mid)
            scaled_delay = limit_mid;
    }

    if (wide_mode) {
        margin >>= 2;
        scaled_delay >>= 2;
    } else {
        margin >>= 1;
        scaled_delay >>= 1;
    }

    uint32_t minimum = (margin < scaled_delay) ? margin : scaled_delay;
    uint32_t fallback = (minimum <= 3u) ? 3u - minimum : 0u;
    if (correction < fallback)
        correction = fallback;

    uint32_t half_delta = (delay_lo - 7u) >> 1;
    uint32_t total = correction + margin;
    if (half_delta + 4u > total)
        return 0u;
    return total - half_delta - 4u;
}

/* PHY state capture */

/* Capture and clear per-channel PHY state. */
void ddr_phy_state_capture_and_clear(void *ctx, uint32_t channel,
                                                  void *snapshot)
{
    struct qrk3588_ddr_phy_saved_state *saved =
        qrk3588_ddr_state_phy_saved(snapshot, channel);
    volatile uint32_t *phy = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base0(ctx, channel);
    volatile uint32_t *bank = (volatile uint32_t *)((uintptr_t)phy + 0x10000u);
    volatile uint32_t *table;
    uint32_t table_offset;
    uint32_t pair = channel & 1u;
    uint32_t value;

    /* ctx+0xc8 and ctx+0xd0 select the controller table by channel group. */
    table = (volatile uint32_t *)qrk3588_ddr_ctx_shared_bank(ctx, channel);

    saved->bank_180 = bank[0x180u / 4u];
    bank[0x180u / 4u] &= 0xfffbfd00u;
    if (qrk3588_ddr_poll32((uintptr_t)&bank[0x14u / 4u], 7u, 1u, 0u) != 0)
        return;

    table_offset = pair * 0x30u;
    saved->shared_18_value = table[(table_offset + 0x18u) / 4u] & 0xffffu;

    saved->shared_04_low5 = table[(table_offset + 4u) / 4u] & 0x1fu;
    table[(table_offset + 4u) / 4u] = 0x001f0000u;

    saved->shared_24_low13 = table[(table_offset + 0x24u) / 4u] & 0x2fffu;
    table[(table_offset + 0x24u) / 4u] = 0x2fff0fffu;

    value = table[pair + 0x4cu] & 0x4000u;
    saved->shared_4c_bit14 = value;
    table[pair + 0x4cu] = 0x40000000u;
}

/* PHY state restore */

/* Restore per-channel PHY state captured by ddr_phy_state_capture_and_clear. */
void ddr_phy_state_restore(void *ctx, uint32_t channel,
                                        void *snapshot)
{
    struct qrk3588_ddr_phy_saved_state *saved =
        qrk3588_ddr_state_phy_saved(snapshot, channel);
    volatile uint32_t *phy = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base0(ctx, channel);
    volatile uint32_t *bank = (volatile uint32_t *)
        ((uintptr_t)phy + 0x10000u);
    volatile uint32_t *table = (volatile uint32_t *)
        qrk3588_ddr_ctx_shared_bank(ctx, channel);
    uint32_t pair = channel & 1u;
    uint32_t table_offset = pair * 0x30u;
    uint32_t value = ddr_phy_restore_margin(ctx, channel, snapshot);

    saved->shared_18_value = value;
    table[(table_offset + 0x18u) / 4u] = value | 0xffff0000u;

    value = saved->shared_04_low5;
    table[(table_offset + 4u) / 4u] = value | 0x001f0000u;

    value = saved->shared_24_low13;
    table[(table_offset + 0x24u) / 4u] = value | 0x2fff0000u;

    value = saved->shared_4c_bit14;
    table[pair + 0x4cu] = value | 0x40000000u;

    bank[0x180u / 4u] = saved->bank_180;
}

/* PHY timing selection */

/* Configure one PHY timing selector. */
void ddr_phy_timing_select(void *ctx, uint32_t channel,
                                        const void *state, uint32_t selector)
{
    volatile uint32_t *phy = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base1(ctx, channel);
    const uint8_t *cfg = (const uint8_t *)qrk3588_ddr_state_dram_params(state);
    uint32_t value;
    uint32_t offset;

    if (*(const uint32_t *)(cfg + 0x28u) == 9u) {
        const uint8_t *timing = (const uint8_t *)freq_cfg_ptr(9u,
            *(const uint32_t *)(cfg + 0x2cu));
        value = (uint32_t)timing[0x12u] << 4;
        value |= (*(const uint32_t *)(cfg + 0x60u) == 0x80u) ? 12u : 10u;
    } else {
        value = 9u;
    }

    offset = (selector == 1u) ? 0xbf0u :
             (selector == 2u) ? 0xa94u : 0xbf4u;
    *(volatile uint32_t *)((uintptr_t)phy + offset) = value;
}

/* PHY training-margin capture/update sequence */

static inline uint32_t phy_sem_rd32(volatile uint8_t *base, uintptr_t off)
{
    return *(volatile uint32_t *)(base + off);
}

static inline void phy_sem_wr32(volatile uint8_t *base, uintptr_t off, uint32_t value)
{
    *(volatile uint32_t *)(base + off) = value;
}

static inline uint32_t phy_sem_adjust_10bit(uint32_t value, uint32_t encoded_delta)
{
    uint32_t magnitude = encoded_delta & 0x1ffu;
    return (encoded_delta & 0x200u) ? value - magnitude : value + magnitude;
}

static int phy_capture_margin_sample(volatile uint8_t *phy, uint32_t dram_type,
                                     unsigned pass, unsigned sample_index,
                                     uint32_t *status_word,
                                     uint16_t *margin_lo, uint16_t *margin_hi)
{
    uint32_t status;
    uint32_t a, b;
    uint32_t fail_bits;

    /* Select the requested rank/sample slot in bits [25:24]. */
    a = phy_sem_rd32(phy, 0x688u);
    a = (a & 0xfcffffffu) | (0x01000000u << sample_index);
    phy_sem_wr32(phy, 0x688u, a);

    if (dram_type == 9u)
        phy_sem_wr32(phy, 0xa24u, phy_sem_rd32(phy, 0xa24u) | 1u);

    a = phy_sem_rd32(phy, 0x684u);
    a |= (pass == 0u) ? 2u : 4u;
    phy_sem_wr32(phy, 0x684u, a);

    if (qrk3588_ddr_poll32((uintptr_t)(phy + 0x684u), 1u, 1u, 0u) != 0)
        return QRK3588_DDR_ETIMEDOUT;

    a = phy_sem_rd32(phy, 0x684u);
    if (pass == 0u) {
        a &= ~2u;
        status = phy_sem_rd32(phy, 0x68cu);
    } else {
        a &= ~4u;
        status = phy_sem_rd32(phy, 0x690u);
    }
    phy_sem_wr32(phy, 0x684u, a);

    a = phy_sem_rd32(phy, 0x69cu);
    b = phy_sem_rd32(phy, 0x6a0u);

    if (dram_type == 9u)
        phy_sem_wr32(phy, 0xa24u, phy_sem_rd32(phy, 0xa24u) & ~1u);

    *status_word = status;

    /* Four status bits are packed as two 2-bit fields.  The selected bit is
     * rank/sample_index dependent.  Pass 1 has two additional fatal bits. */
    fail_bits = ((status >> 26) & 3u) | ((status >> 10) & 3u);
    if (((fail_bits >> sample_index) & 1u) != 0u ||
        (pass != 0u && (status & 0x00004000u) != 0u))
        return QRK3588_DDR_EIO;

    if ((a & 0x3ffu) == 0x3ffu || (b & 0x3ffu) == 0x3ffu)
        return QRK3588_DDR_EIO;

    fail_bits = ((status >> 28) & 3u) | ((status >> 12) & 3u);
    if (((fail_bits >> sample_index) & 1u) != 0u ||
        (pass != 0u && (status & 0x40000000u) != 0u))
        return QRK3588_DDR_EIO;

    if (((a >> 16) & 0x3ffu) == 0x3ffu ||
        ((b >> 16) & 0x3ffu) == 0x3ffu)
        return QRK3588_DDR_EIO;

    ddr_phy_update_toggle((uintptr_t)phy);

    *margin_lo = (uint16_t)((a & 0x3ffu) + (b & 0x3ffu));
    *margin_hi = (uint16_t)(((a >> 16) & 0x3ffu) + ((b >> 16) & 0x3ffu));
    return 0;
}

/* Apply the two captured signed margin deltas to the internal PHY tables
 * during the no-output-buffer pass-1 path. */
static void phy_apply_margin_to_lane_tables(volatile uint8_t *phy,
                                            const uint32_t samples[2],
                                            uint32_t count)
{
    uint32_t sample_index;

    for (sample_index = 0u; sample_index < count; ++sample_index) {
        uint32_t sample = samples[sample_index];
        uint32_t dlo = sample & 0x3ffu;
        uint32_t dhi = (sample >> 16) & 0x3ffu;
        uintptr_t src_pair0 = sample_index ? 0xdc8u : 0xdc0u;
        uintptr_t dst_pair0 = sample_index ? 0xdd8u : 0xdd0u;
        uintptr_t src_pair1 = sample_index ? 0xdccu : 0xdc4u;
        uintptr_t dst_pair1 = sample_index ? 0xddcu : 0xdd4u;
        uintptr_t lane_src = sample_index ? 0x410u : 0x1f0u;
        uintptr_t lane_dst = sample_index ? 0x8e0u : 0x880u;
        uintptr_t copy_src = sample_index ? 0x500u : 0x490u;
        uintptr_t copy_dst = sample_index ? 0x910u : 0x8b0u;
        unsigned lane;

        phy_sem_wr32(phy, dst_pair0, phy_sem_rd32(phy, src_pair0));
        phy_sem_wr32(phy, dst_pair1, phy_sem_rd32(phy, src_pair1));

        for (lane = 0u; lane < 10u; ++lane) {
            uintptr_t src = lane_src + (uintptr_t)lane * 0x0cu;
            uint32_t value;
            uint32_t lo, hi;

            if (lane == 9u)
                src -= 8u;
            value = phy_sem_rd32(phy, src);
            lo = phy_sem_adjust_10bit(value & 0x3ffu, dlo);
            hi = phy_sem_adjust_10bit((value >> 16) & 0x3ffu, dhi);
            phy_sem_wr32(phy, lane_dst + (uintptr_t)lane * 4u,
                         lo | (hi << 16));
        }

        for (lane = 0u; lane < 10u; ++lane) {
            uintptr_t dst = copy_dst + (uintptr_t)lane * 4u;
            if (sample_index == 1u && lane == 9u)
                dst = 0x8e8u + (uintptr_t)lane * 4u;
            phy_sem_wr32(phy, dst,
                         phy_sem_rd32(phy, copy_src + (uintptr_t)lane * 0x0cu -
                                      (lane == 9u ? 8u : 0u)));
        }
    }

    phy_sem_wr32(phy, 0x4u, phy_sem_rd32(phy, 0x4u) | 0x20u);
    phy_sem_wr32(phy, 0x10u, phy_sem_rd32(phy, 0x10u) | 0x200u);
    udelay(1u);
    phy_sem_wr32(phy, 0x10u, phy_sem_rd32(phy, 0x10u) & ~0x200u);
}

/* Fold captured margins into the PHY's packed per-rank training image during
 * the no-output-buffer pass-0 path. */
static void phy_apply_margin_to_training_image(volatile uint8_t *phy,
                                               const uint32_t samples[2],
                                               uint32_t count)
{
    uint32_t sample_index;

    for (sample_index = 0u; sample_index < count; ++sample_index) {
        uintptr_t src0 = sample_index ? 0x770u : 0x190u;
        uintptr_t dst0 = sample_index ? 0x824u : 0x7ccu;
        uintptr_t src1 = sample_index ? 0x794u : 0x614u;
        uintptr_t dst1 = sample_index ? 0x854u : 0x7f4u;
        uint32_t off;
        uint32_t sample = samples[sample_index];
        uint32_t word;
        uint32_t lo, hi;

        for (off = 0u; off < 0x20u; off += 4u) {
            uintptr_t source_delta = sample_index ? off : (uintptr_t)off * 3u;
            phy_sem_wr32(phy, dst0 + off, phy_sem_rd32(phy, src0 + source_delta));
        }
        phy_sem_wr32(phy, dst0 - 4u, phy_sem_rd32(phy, src0 - 4u));

        for (off = 0u; off < 0x20u; off += 4u) {
            uintptr_t source_delta = sample_index ? off : (uintptr_t)off * 3u;
            phy_sem_wr32(phy, dst1 + off, phy_sem_rd32(phy, src1 + source_delta));
        }
        phy_sem_wr32(phy, dst1 - 4u, phy_sem_rd32(phy, src1 - 4u));

        if (sample_index != 0u)
            phy_sem_wr32(phy, 0x814u, phy_sem_rd32(phy, 0x764u));
        else
            phy_sem_wr32(phy, 0x7c0u, phy_sem_rd32(phy, 0x574u));

        word = phy_sem_rd32(phy, 0x580u + (uintptr_t)sample_index * 0x1e8u);
        lo = word & 0x1ffu;
        hi = (word >> 16) & 0x1ffu;
        lo = phy_sem_adjust_10bit(lo, sample & 0x3ffu);
        hi = phy_sem_adjust_10bit(hi, (sample >> 16) & 0x3ffu);
        phy_sem_wr32(phy, 0x7c4u + (uintptr_t)sample_index * 0x54u,
                     (lo & 0x1ffu) | (hi << 16));
    }

    phy_sem_wr32(phy, 0x4u, phy_sem_rd32(phy, 0x4u) | 0x8u);
    phy_sem_wr32(phy, 0x10u, phy_sem_rd32(phy, 0x10u) | 0x100u);
    udelay(1u);
    phy_sem_wr32(phy, 0x10u, phy_sem_rd32(phy, 0x10u) & ~0x100u);
}

int phy_update_seq(void *state_ptr, void *context_ptr,
                                      uint32_t channel, uint32_t count,
                                      void *output_a, void *output_b)
{
    uint8_t *state = (uint8_t *)state_ptr;
    volatile uint8_t *guard = (volatile uint8_t *)
        qrk3588_ddr_ctx_channel_base0(context_ptr, channel);
    volatile uint8_t *phy = (volatile uint8_t *)
        qrk3588_ddr_ctx_channel_base1(context_ptr, channel);
    const uint8_t *timing = (const uint8_t *)qrk3588_ddr_state_timing(state);
    uint32_t dram_type = qrk3588_ddr_state_dram_type(state);
    uint32_t saved_phy4_bit22;
    uint32_t trigger_token;
    unsigned pass;

    if (count > 2u)
        return QRK3588_DDR_EINVAL;

    if (output_a == NULL && *(const uint32_t *)timing <= 1599u && output_b == NULL)
        return 0;

    trigger_token = ddr_phy_trigger_begin((volatile void *)guard);
    saved_phy4_bit22 = phy_sem_rd32(phy, 0x4u) & 0x00400000u;
    if (saved_phy4_bit22 != 0u)
        phy_sem_wr32(phy, 0x4u, phy_sem_rd32(phy, 0x4u) & ~0x00400000u);

    phy_sem_wr32(phy, 0xb0u, phy_sem_rd32(phy, 0xb0u) | 0x00800000u);

    if (dram_type == 9u) {
        phy_sem_wr32(phy, 0xa24u, phy_sem_rd32(phy, 0xa24u) & ~0x78u);
        phy_sem_wr32(phy, 0x684u, phy_sem_rd32(phy, 0x684u) | 0x10u);
    } else {
        uint32_t v;
        phy_sem_wr32(phy, 0xaecu, phy_sem_rd32(phy, 0xaecu) | 0x0008d000u);
        v = phy_sem_rd32(phy, 0x6a4u);
        v = (v & 0xffe0ffffu) | (*(const uint32_t *)(timing + 0x44u) << 16);
        phy_sem_wr32(phy, 0x6a4u, v & 0x7fffffffu);
        v = phy_sem_rd32(phy, 0x688u);
        v = (v & 0xff03ffffu) | (*(const uint32_t *)(timing + 0x1ecu) << 18);
        phy_sem_wr32(phy, 0x688u, v);
        phy_sem_wr32(phy, 0x684u, phy_sem_rd32(phy, 0x684u) & ~0x80u);
    }

    for (pass = 0u; pass < 2u; ++pass) {
        uint32_t samples[2] = { 0u, 0u };
        int has_a = output_a != NULL;
        int has_b = output_b != NULL;
        int write_a = (pass == 0u) && has_a;
        uint32_t i;
        int capture_failed = 0;

        if ((pass == 0u && !has_a && has_b) ||
            (pass == 1u && has_a && !has_b))
            continue;

        for (i = 0u; i < count; ++i) {
            uint16_t lo, hi;
            int rc = phy_capture_margin_sample(phy, dram_type, pass, i,
                                               &samples[i], &lo, &hi);
            if (rc != 0) {
                if (rc == QRK3588_DDR_ETIMEDOUT) {
                    ddr_phy_trigger_end((volatile void *)guard, trigger_token);
                    return rc;
                }
                capture_failed = 1;
                break;
            }

            if (write_a) {
                uint16_t *dst = (uint16_t *)((uint8_t *)output_a + 4u + (uintptr_t)i * 4u);
                dst[0] = lo;
                dst[1] = hi;
            } else if (pass == 1u && has_b) {
                uint16_t *dst = (uint16_t *)((uint8_t *)output_b + 4u + (uintptr_t)i * 4u);
                dst[0] = lo;
                dst[1] = hi;
            }
        }

        if (!capture_failed && !has_a && !has_b) {
            if (pass == 0u)
                phy_apply_margin_to_training_image(phy, samples, count);
            else
                phy_apply_margin_to_lane_tables(phy, samples, count);
        }
    }

    phy_sem_wr32(phy, 0x4u, phy_sem_rd32(phy, 0x4u) | 5u);
    udelay(1u);
    phy_sem_wr32(phy, 0x4u, phy_sem_rd32(phy, 0x4u) & ~5u);
    phy_sem_wr32(phy, 0xb0u, phy_sem_rd32(phy, 0xb0u) & ~0x00800000u);
    if (saved_phy4_bit22 != 0u)
        phy_sem_wr32(phy, 0x4u, phy_sem_rd32(phy, 0x4u) | 0x00400000u);

    ddr_phy_update_toggle((uintptr_t)phy);
    phy_sem_wr32(phy, 0x10u, phy_sem_rd32(phy, 0x10u) | 0x40u);
    phy_sem_wr32(phy, 0x694u, 0u);
    phy_sem_wr32(phy, 0x698u, 0u);
    phy_sem_wr32(phy, 0x10u, phy_sem_rd32(phy, 0x10u) & ~0x40u);
    ddr_phy_update_toggle((uintptr_t)phy);
    ddr_phy_trigger_end((volatile void *)guard, trigger_token);
    return 0;
}
