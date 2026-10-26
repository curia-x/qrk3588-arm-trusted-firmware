/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* DDR frequency-switch programming sequence.
 *
 * Stages channel clocks and controller/PHY images, performs update handshakes,
 * commits the selected timing profile and restores shared PHY state.
 */
#include "qrk3588_ddr_internal.h"

/*
 *
 * The sequence is a frequency-switch programming orchestration pass.  It
 * walks the four channel selectors in ctx+0xd8..ctx+0xe4, stages the PLL/PCTL
 * and PHY banks, commits the selected timing image, performs the PHY update
 * handshakes, then restores the shared PHY save area.  Unknown private
 * layout fields intentionally remain offset-named; CPU register state and
 * basic-block control flow are not part of this implementation.
 */

#ifdef RK3588_DDR_MMIO_TRACE
extern void rk3588_ddr_mmio_trace(uintptr_t address, uint32_t value,
                                  uint32_t is_write);
#endif

static inline uint32_t fs_rd32(const void *base, size_t off)
{
    const uintptr_t address = (uintptr_t)((const uint8_t *)base + off);
    const uint32_t value = *(const volatile uint32_t *)address;
#ifdef RK3588_DDR_MMIO_TRACE
    rk3588_ddr_mmio_trace(address, value, 0u);
#endif
    return value;
}


static inline void fs_wr32(void *base, size_t off, uint32_t value)
{
    const uintptr_t address = (uintptr_t)((uint8_t *)base + off);
#ifdef RK3588_DDR_MMIO_TRACE
    rk3588_ddr_mmio_trace(address, value, 1u);
#endif
    *(volatile uint32_t *)address = value;
}

static inline uint8_t *fs_channel_phy(void *context, uint32_t channel)
{
    return (uint8_t *)qrk3588_ddr_ctx_channel_base0(context, channel);
}

static inline uint8_t *fs_channel_ctrl(void *context, uint32_t channel)
{
    return (uint8_t *)qrk3588_ddr_ctx_channel_base1(context, channel);
}

static inline uint8_t *fs_channel_aux(void *context, uint32_t channel)
{
    return (uint8_t *)qrk3588_ddr_ctx_channel_base2(context, channel);
}

static inline uint8_t *fs_shared_lane_root(void *context, uint32_t channel)
{
    return (uint8_t *)qrk3588_ddr_ctx_shared_bank(context, channel);
}

static inline volatile uint32_t *fs_clock_words(void *context)
{
    return (volatile uint32_t *)qrk3588_ddr_ctx_clock_table(context);
}

static void fs_assert_channel_resets(void *context, uint32_t channel,
                                     uint8_t *aux)
{
    volatile uint32_t *clock = fs_clock_words(context);
    uint8_t *lane_root = fs_shared_lane_root(context, channel);
    const uint32_t pair = channel & 1u;
    const size_t lane_off = 0x130u + (size_t)pair * 4u;
    const size_t clock_word = 0x280u + channel + 0x14u + (channel >> 1);
    const uint32_t bit = 1u << (6u - pair * 4u);
    const uint32_t paired_bit = bit | (bit << 16);

    fs_wr32(aux, 0xa00u, 0x00160016u);
    udelay(10u);
    fs_wr32(aux, 0xa00u, 0x00100000u);
    clock[clock_word] = paired_bit;
    fs_wr32(lane_root, lane_off, 0x00080000u);
    fs_wr32(aux, 0xa00u, 0x00040000u);
    udelay(5u);
    fs_wr32(lane_root, lane_off, 0x00080008u);
    udelay(5u);
    fs_wr32(aux, 0xa00u, 0x00020000u);
    ddr_clock_triplet_write(clock, channel, 1u);
    udelay(5u);
    /* CRU uses hiword write-mask semantics.  The frozen blob writes only
     * the mask bit here, so the corresponding low-half reset bit is cleared
     * and the per-channel DFI soft reset is released. */
    clock[clock_word] = bit << 16;
    udelay(5u);
    fs_wr32(lane_root, lane_off, 0x00080000u);
    fs_wr32(aux, 0xa00u, 0x00020002u);
}

static void fs_release_channel_resets(void *context, uint32_t channel,
                                      uint8_t *aux)
{
    volatile uint32_t *clock = fs_clock_words(context);
    uint8_t *lane_root = fs_shared_lane_root(context, channel);
    const uint32_t pair = channel & 1u;
    const size_t lane_off = 0x130u + (size_t)pair * 4u;
    const size_t clock_word = 0x280u + channel + 0x14u + (channel >> 1);
    const uint32_t lane_idle = pair == 0u ? 0x80000u : 0x10000u;
    uint32_t staged;

    fs_wr32(aux, 0xa00u, 0x00020000u);
    fs_wr32(lane_root, lane_off, 0x00080008u);
    staged = (1u << (pair == 0u ? 3u : 0u)) | lane_idle;
    clock[clock_word] = staged;
    udelay(10u);
    clock[clock_word] = lane_idle;
    udelay(50u);
}

static int fs_phy_reg_write_checked(uint8_t *phy, uint32_t rank,
                                    uint32_t reg, uint32_t value)
{
    (void)ddr_phy_reg_write((uint32_t)(uintptr_t)phy, rank, reg, value);
    return qrk3588_ddr_get_error();
}

static void fs_stage_frequency_profile(void *context, uint8_t *config,
                                       uint32_t channel, uint8_t *phy,
                                       uint32_t dram_type)
{
    const uint8_t *table = DDR_LAYOUT_CPTR(uint8_t, QRK3588_DDR_FREQUENCY_PROFILE_TABLE_VA);
    uint8_t *bank = phy + 0x30000u;
    uint32_t row18 = qrk3588_ddr_state_channel_rank_row_bits(config, channel, 0u);
    uint32_t row1c = qrk3588_ddr_state_channel_rank_row_bits(config, channel, 1u);
    const uint32_t special = (row18 != row1c ||
                              qrk3588_ddr_state_channel_rank_count(config, channel) == 1u);
    uint32_t selector;

    if (dram_type == 9u)
        selector = special ? 2u : 6u;
    else
        selector = special ? 0u : 4u;
    uint32_t maximum = row18 >= row1c ? row18 : row1c;
    uint32_t v;

    (void)context;
    qrk3588_ddr_state_set_channel_frequency_profile_selector(config, channel,
                                                            selector);
    fs_wr32(bank, 0x04u, fs_rd32(table, (size_t)selector * 40u));
    ddr_word_copy(bank + 0x0cu, table + 4u + (size_t)selector * 40u, 36u);

    if (maximum > 17u) {
        v = fs_rd32(bank, 0x1cu);
        v = (v & 0xe0ffffffu) | ((v & 0x1fu) << 24);
        fs_wr32(bank, 0x1cu, v);
    }

    /* Walk down from 18 and mark absent profile levels in the two packed
     * 32-bit words at 0x1c/0x20. */
    for (uint32_t level = 18u; maximum < level; --level) {
        const uint32_t delta = 18u - level;
        const size_t off = 0x1cu + (size_t)(delta & 4u);
        const uint32_t shift = ((level - 19u) & 3u) << 3;
        fs_wr32(bank, off, fs_rd32(bank, off) | (0x1fu << shift));
    }

    if (qrk3588_ddr_state_channel_rank_count(config, channel) == 1u)
        fs_wr32(bank, 0x04u, fs_rd32(bank, 0x04u) | 0x3fu);

    if (qrk3588_ddr_state_channel_row_3_4(config, channel) != 0u && row18 > 13u)
        fs_wr32(bank, 0x30u, row18 - 13u);
}

static int fs_prepare_channel(void *context, uint8_t *config,
                              uint32_t profile, uint32_t channel)
{
    uint8_t *phy = fs_channel_phy(context, channel);
    uint8_t *aux = fs_channel_aux(context, channel);
    uint8_t *pctl = phy + 0x10000u;
    uint8_t *phy_hi = phy + 0x20000u;
    uint8_t *lane_root = fs_shared_lane_root(context, channel);
    const uint32_t dram_type = qrk3588_ddr_state_dram_type(config);
    const uint32_t pair = channel & 1u;
    uint32_t v;

    qrk3588_ddr_active_context->error_channel = channel;
    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_CLOCK_SETUP;
    fout_mhz_setup(context, channel, qrk3588_ddr_ctx_handoff_state_const(context)->topology_word88_bit26, 24u);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_RESET_ASSERT;
    fs_assert_channel_resets(context, channel, aux);

    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_PHY_BOOT_CONFIG;
    ddr_phy_mode_config(context, channel, config, 1u, 0u);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_RESET_RELEASE;
    fs_release_channel_resets(context, channel, aux);
    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_PCTL_CONFIG;
    config_pctl_regs_init(context, channel, config, profile);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();

    v = fs_rd32(phy, 0xd0cu) & 0xff00ffe0u;
    v |= ((qrk3588_ddr_ctx_descriptor_state_const(context)->word4_hi16 & 0xffu) << 16);
    v |= qrk3588_ddr_ctx_descriptor_state_const(context)->word4_lo16 & 0x1fu;
    fs_wr32(phy, 0xd0cu, v);

    fs_wr32(pctl, 0x280u, fs_rd32(pctl, 0x280u) | 0x80000000u);
    fs_wr32(phy_hi, 0x04u, fs_rd32(phy_hi, 0x04u) | 0x00010000u);

    if (dram_type == 9u) {
        fs_wr32(phy_hi, 0x00u, fs_rd32(phy_hi, 0x00u) | 1u);
        fs_wr32(phy_hi, 0x04u, fs_rd32(phy_hi, 0x04u) | 0x2000u);
        fs_wr32(lane_root, (size_t)pair * 48u, 0x10001000u);
    }

    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_PHY_PARAM_CONFIG;
    fs_wr32(pctl, 0x08u, fs_rd32(pctl, 0x08u) & ~3u);
    ddr_phy_frequency_parameter_stage(context, channel, config, 0u, 0u, 0u);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();

    fs_stage_frequency_profile(context, config, channel, phy, dram_type);

    ddr_clock_triplet_write(fs_clock_words(context), channel, 0u);
    fs_wr32(pctl, 0xb84u, fs_rd32(pctl, 0xb84u) | 2u);
    fs_wr32(phy_hi, 0x90u, fs_rd32(phy_hi, 0x90u) & ~1u);
    fs_wr32(pctl, 0xc80u, 0u);
    fs_wr32(pctl, 0x510u, fs_rd32(pctl, 0x510u) & ~1u);
    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_PHY_UPDATE;
    ddr_phy_update_complete((uint32_t)(uintptr_t)phy);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    fs_wr32(phy_hi, 0x90u, fs_rd32(phy_hi, 0x90u) | 1u);
    fs_wr32(pctl, 0xb84u, fs_rd32(pctl, 0xb84u) & ~2u);
    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_PHY_IMAGE_CONFIG;
    ddr_phy_mode_config(context, channel, config, 0u, profile);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();

    if (dram_type == 9u)
        fs_wr32(pctl, 0x80u, fs_rd32(pctl, 0x80u) | 8u);

    return 0;
}

static int fs_enable_phy_update(void *context, uint32_t channel)
{
    uint8_t *phy = fs_channel_phy(context, channel);
    uint8_t *ctrl = fs_channel_ctrl(context, channel);
    uint8_t *pctl = phy + 0x10000u;

    fs_wr32(ctrl, 0xb0u, fs_rd32(ctrl, 0xb0u) & ~0x20u);
    fs_wr32(pctl, 0xc80u, 0u);
    fs_wr32(pctl, 0x510u, fs_rd32(pctl, 0x510u) | 0x20u);
    ddr_phy_update_complete((uint32_t)(uintptr_t)phy);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    if (qrk3588_ddr_poll32((uintptr_t)(pctl + 0x514u), 1u, 1u, 0u) != 0)
        return QRK3588_DDR_ETIMEDOUT;
    return 0;
}

static int fs_trigger_phy_update(void *context, uint32_t channel)
{
    uint8_t *phy = fs_channel_phy(context, channel);
    uint8_t *pctl = phy + 0x10000u;

    fs_wr32(pctl, 0xc80u, 0u);
    fs_wr32(pctl, 0x510u, fs_rd32(pctl, 0x510u) & ~0x20u);
    fs_wr32(pctl, 0x510u, fs_rd32(pctl, 0x510u) | 1u);
    ddr_phy_update_complete((uint32_t)(uintptr_t)phy);
    return qrk3588_ddr_get_error();
}

static int fs_commit_channel(void *context, uint8_t *config,
                             uint32_t profile, uint32_t channel)
{
    uint8_t *phy = fs_channel_phy(context, channel);
    uint8_t *ctrl = fs_channel_ctrl(context, channel);
    uint8_t *pctl = phy + 0x10000u;
    uint8_t *lane_root = fs_shared_lane_root(context, channel);
    const uint32_t dram_type = qrk3588_ddr_state_dram_type(config);
    const uint32_t pair = channel & 1u;
    uint32_t v;

    if (dram_type == 9u) {
        ddr_phy_update_toggle((uintptr_t)ctrl);
        fs_wr32(lane_root, (size_t)pair * 48u + 8u, 0x00ff00ffu);
        fs_wr32(lane_root, (size_t)pair * 48u + 8u, 0x00ff0000u);
        udelay(2u);
        ddr_phy_apply_byte_config((uint32_t)(uintptr_t)phy,
                            qrk3588_ddr_state_lp5_channel_image_const(config, channel)->byte);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        fs_wr32(pctl, 0x80u, fs_rd32(pctl, 0x80u) & ~8u);
        fs_wr32(pctl, 0x280u, fs_rd32(pctl, 0x280u) | 0x80000000u);
        ddr_pctl_trigger_handshake(phy);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        fs_wr32(pctl, 0x280u, fs_rd32(pctl, 0x280u) & 0x7fffffffu);
    }

    if (qrk3588_ddr_poll32((uintptr_t)(pctl + 0x14u), 7u, 1u, 0u) != 0)
        return QRK3588_DDR_ETIMEDOUT;

    if ((dram_type == 7u || dram_type == 8u) && qrk3588_ddr_ctx_handoff_state_const(context)->valid == 0u) {
        const uint32_t word508 = fs_rd32(phy, 0x508u);
        const uint32_t word50c = fs_rd32(phy, 0x50cu);
        const uint32_t high508 = word508 >> 16;

        if (fs_phy_reg_write_checked(phy, 3u, 11u, high508) != 0)
            return qrk3588_ddr_get_error();
        if (fs_phy_reg_write_checked(phy, 2u, 11u,
                                     high508 & 0xffffff8fu) != 0)
            return qrk3588_ddr_get_error();
        if (fs_phy_reg_write_checked(phy, 3u, 12u,
                                     word508 & 0xffffu) != 0)
            return qrk3588_ddr_get_error();
        if (profile != 0u &&
            fs_phy_reg_write_checked(phy, 3u, 14u,
                                     word50c & 0xffffu) != 0)
            return qrk3588_ddr_get_error();
        if (fs_phy_reg_write_checked(phy, 3u, 22u,
                                     word50c >> 16) != 0)
            return qrk3588_ddr_get_error();
        fs_wr32(pctl, 0x280u, fs_rd32(pctl, 0x280u) | 0x80000000u);
        ddr_pctl_trigger_handshake(phy);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        fs_wr32(pctl, 0x280u, fs_rd32(pctl, 0x280u) & 0x7fffffffu);
    }

    v = fs_rd32(ctrl, 0x3d0u);
    fs_wr32(ctrl, 0x3d0u, (v & 0xffff0000u) | 7u);
    /* Frozen 089dc commits CTRL+0x3c8 in two distinct transactions:
     * write the base image first, reread it, then set trigger bit1. */
    v = fs_rd32(ctrl, 0x3c8u);
    v = (v & 0xfffbfff3u) | 0x00040004u;
    fs_wr32(ctrl, 0x3c8u, v);
    v = fs_rd32(ctrl, 0x3c8u);
    fs_wr32(ctrl, 0x3c8u, v | 2u);
    if (qrk3588_ddr_poll32((uintptr_t)(ctrl + 0x3ccu), 1u, 1u, 0u) != 0)
        return QRK3588_DDR_ETIMEDOUT;
    fs_wr32(ctrl, 0x3c8u, fs_rd32(ctrl, 0x3c8u) & 0xfffbfffdu);

    ddr_phy_reg_read(context, channel, qrk3588_ddr_state_dram_type_ptr_const(config),
                     qrk3588_ddr_ctx_handoff_state_const(context)->topology_word88_bit26, qrk3588_ddr_state_frequency(config) << 1);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    return 0;
}

static int fs_finalize_channel(void *context, uint8_t *config, uint32_t channel)
{
    uint8_t *ctrl = fs_channel_ctrl(context, channel);
    uint8_t *lane_root = fs_shared_lane_root(context, channel);
    const uint8_t *dram = (const uint8_t *)qrk3588_ddr_state_dram_params(config);
    const uint32_t dram_type = qrk3588_ddr_state_dram_type(config);
    const uint8_t *freq_cfg = (const uint8_t *)freq_cfg_ptr(dram_type, fs_rd32(dram, 0x2cu));
    const size_t lane_off = ((size_t)(channel & 1u) + 0x4cu) * 4u;
    uint32_t v;

    fs_wr32(lane_root, lane_off, 0x00200020u);
    fs_wr32(lane_root, lane_off, 0x00100010u);

    /* Frozen 089dc likewise requires a B0 bit6 0->1 edge as two MMIO
     * writes with a reread between them. */
    v = fs_rd32(ctrl, 0xb0u);
    v = (v & 0xffffff9fu) | ((uint32_t)freq_cfg[1] << 5);
    fs_wr32(ctrl, 0xb0u, v);
    v = fs_rd32(ctrl, 0xb0u);
    fs_wr32(ctrl, 0xb0u, v | 0x40u);
    if (qrk3588_ddr_poll32((uintptr_t)(ctrl + 0xb4u), 1u << 18, 1u << 18, 0u) != 0)
        return QRK3588_DDR_ETIMEDOUT;

    fs_wr32(lane_root, lane_off, 0x00100000u);
    fs_wr32(lane_root, lane_off, 0x00200000u);
    return 0;
}

int ddr_frequency_switch_program_sequence(void *context, uint8_t *config,
                                          uint32_t profile)
{
    const uint32_t dram_type = qrk3588_ddr_state_dram_type(config);
    const uint8_t *context_bytes = (const uint8_t *)context;
    const size_t channel_slots = 4u;
    int rc = 0;

    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();

    if (qrk3588_ddr_active_context != NULL)
        qrk3588_ddr_active_context->error_dram_type = dram_type;
    ddr_clock_regs_save_restore(context, config, 1u);

    for (size_t i = 0; i < channel_slots; ++i) {
        const uint32_t channel = qrk3588_ddr_ctx_route(context_bytes, (uint32_t)i);
        if (channel == 0x0fu)
            continue;
        rc = fs_prepare_channel(context, config, profile, channel);
        if (rc != 0)
            goto out;
    }

    for (size_t i = 0; i < channel_slots; ++i) {
        const uint32_t channel = qrk3588_ddr_ctx_route(context_bytes, (uint32_t)i);
        if (channel == 0x0fu)
            continue;
        qrk3588_ddr_active_context->error_channel = channel;
        qrk3588_ddr_active_context->error_phase =
            QRK3588_DDR_PHASE_DFI_START;
        rc = fs_enable_phy_update(context, channel);
        if (rc != 0)
            goto out;
    }

    for (size_t i = 0; i < channel_slots; ++i) {
        const uint32_t channel = qrk3588_ddr_ctx_route(context_bytes, (uint32_t)i);
        if (channel == 0x0fu)
            continue;
        rc = fs_trigger_phy_update(context, channel);
        if (rc != 0)
            goto out;
    }

    if (dram_type == 9u)
        udelay(2210u);

    for (size_t i = 0; i < channel_slots; ++i) {
        const uint32_t channel = qrk3588_ddr_ctx_route(context_bytes, (uint32_t)i);
        if (channel == 0x0fu)
            continue;
        qrk3588_ddr_active_context->error_channel = channel;
        qrk3588_ddr_active_context->error_phase =
            QRK3588_DDR_PHASE_CHANNEL_COMMIT;
        rc = fs_commit_channel(context, config, profile, channel);
        if (rc != 0)
            goto out;
    }

    ddr_clock_regs_save_restore(context, config, 0u);

    for (size_t i = 0; i < channel_slots; ++i) {
        const uint32_t channel = qrk3588_ddr_ctx_route(context_bytes, (uint32_t)i);
        if (channel == 0x0fu)
            continue;
        qrk3588_ddr_active_context->error_channel = channel;
        qrk3588_ddr_active_context->error_phase =
            QRK3588_DDR_PHASE_CHANNEL_FINALIZE;
        rc = fs_finalize_channel(context, config, channel);
        if (rc != 0)
            return rc;
    }

    return 0;

out:
    if (qrk3588_ddr_active_context != NULL) {
        volatile uint32_t *clock = fs_clock_words(context);

        qrk3588_ddr_active_context->error_cru_mode =
            clock[0x280u / 4u];
        qrk3588_ddr_active_context->error_cru_clksel1 =
            clock[0x304u / 4u];
    }
    ddr_clock_regs_save_restore(context, config, 0u);
    return rc;
}
