/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Top-level RK3588 DDR training orchestration.
 *
 * Selects the DRAM candidate, probes active channels, performs calibration and
 * training sweeps, publishes parameters and commits the final DVFS profile.
 * Proven private report/eye-scan side effects are preserved.  Unsafe vendor
 * service continuations fail closed instead of performing absolute MMIO/calls
 * or spinning forever.
 */
#include <stddef.h>
#include <stdint.h>

#include <arch_helpers.h>
#include <common/debug.h>

#include <platform_def.h>
#include "qrk3588_ddr_internal.h"
#include "qrk3588_ddr_runtime.h"

#define TRAIN_DESC_ADDR             ((uint32_t)QRK3588_DDR_TYPE_CONFIG_TABLE_VA)
#define TRAIN_HASH_TEMPLATE_ADDR    ((uint32_t)QRK3588_DDR_HASH_TEMPLATE_VA)
#define TRAIN_CONTEXT_SLOT          ((uint32_t)QRK3588_DDR_CONTEXT_SLOT_VA)
#define TRAIN_ACTIVE_SLOT           ((uint32_t)QRK3588_DDR_ACTIVE_CONFIG_SLOT_VA)
#define TRAIN_TIMING_SLOT           ((uint32_t)QRK3588_DDR_TIMING_SLOT_VA)
#define TRAIN_LOOKUP_SLOT           ((uint32_t)QRK3588_DDR_LOOKUP_SLOT_VA)
#define TRAIN_DRAM_SLOT             ((uint32_t)QRK3588_DDR_DRAM_CONFIG_SLOT_VA)
#define TRAIN_SCRATCH_EDGE          ((uint32_t)QRK3588_DDR_TRAIN_EDGE_SCRATCH_VA)
#define TRAIN_SCRATCH_DIRECT0       ((uint32_t)QRK3588_DDR_TRAIN_DIRECT0_SCRATCH_VA)
#define TRAIN_SCRATCH_DIRECT1       ((uint32_t)QRK3588_DDR_TRAIN_DIRECT1_SCRATCH_VA)
#define TRAIN_DVFS_IMAGE            ((uint32_t)QRK3588_DDR_DVFS_IMAGE_VA)
#define TRAIN_DVFS_IMAGE_SIZE       UINT32_C(0x2140)
#define TRAIN_PUBLISH_DST           UINT32_C(0x00100000)
#define TRAIN_DESCRIPTOR_COPY_DST   UINT32_C(0x00104000)

#define TRAIN_CHANNELS 4u
#define TRAIN_LOOKUP_COUNT 64u
#define TRAIN_RECORD_SIZE 12u
#define TRAIN_DIRECT1_MAX_RECORDS 32u

struct train_record {
    uint32_t value;
    uint16_t rank0_low;
    uint16_t rank0_high;
    uint16_t rank1_low;
    uint16_t rank1_high;
};

/* Frozen descriptor word0c bit23 uses this 0x70-byte service/eye-scan config.
 * Only fields consumed by the eye scanner receive
 * names; the remaining stable words intentionally stay private. */
struct qrk3588_ddr_private_eye_config {
    uint64_t io_base[4];
    uint32_t io_selector;
    uint32_t width_mode;
    uint32_t channel;
    uint32_t private_2c;
    uint32_t private_30;
    uint32_t private_34;
    uint32_t private_38;
    uint32_t private_3c;
    uint32_t private_40;
    uint32_t private_44;
    uint32_t private_48;
    uint32_t private_4c;
    uint32_t private_50;
    uint32_t private_54;
    uint32_t private_58;
    uint32_t private_5c;
    uint32_t private_60;
    uint32_t private_64;
    uint32_t private_68;
    uint32_t private_6c;
};

_Static_assert(offsetof(struct qrk3588_ddr_private_eye_config, io_selector) == 0x20u,
               "private eye selector offset");
_Static_assert(offsetof(struct qrk3588_ddr_private_eye_config, width_mode) == 0x24u,
               "private eye width offset");
_Static_assert(offsetof(struct qrk3588_ddr_private_eye_config, channel) == 0x28u,
               "private eye channel offset");
_Static_assert(sizeof(struct qrk3588_ddr_private_eye_config) == 0x70u,
               "private eye config size");

static inline uint32_t tm_rd32(const void *p, size_t off)
{
    return *(volatile const uint32_t *)((const uint8_t *)p + off);
}
static inline void tm_wr32(void *p, size_t off, uint32_t value)
{
    *(volatile uint32_t *)((uint8_t *)p + off) = value;
}
static inline uint8_t *tm_context(void)
{
    return DDR_LAYOUT_REF(uint8_t *, TRAIN_CONTEXT_SLOT);
}
static inline uint8_t *tm_active(void)
{
    return DDR_LAYOUT_REF(uint8_t *, TRAIN_ACTIVE_SLOT);
}
static inline uint8_t *tm_timing(void)
{
    return DDR_LAYOUT_REF(uint8_t *, TRAIN_TIMING_SLOT);
}
static inline uint8_t *tm_dram(void)
{
    return DDR_LAYOUT_REF(uint8_t *, TRAIN_DRAM_SLOT);
}
static inline uint32_t tm_channel_selector(const uint8_t *ctx, unsigned slot)
{
    return qrk3588_ddr_ctx_route(ctx, slot);
}
static inline unsigned tm_rank_count(const uint8_t *active, unsigned channel)
{
    uint32_t n = qrk3588_ddr_state_channel_rank_count(active, channel);
    if (n == 0u) n = 1u;
    if (n > 2u) n = 2u;
    return (unsigned)n;
}

static int tm_descriptor_validate(const uint8_t **selected_out,
                                  uint32_t order[3], uint32_t *channel_mask,
                                  uint32_t *active_profile_out,
                                  uint32_t *special_publish_out)
{
    const uint8_t *desc = DDR_LAYOUT_CPTR(uint8_t, TRAIN_DESC_ADDR);
    const uint8_t *sel;
    uint32_t word14;
    uint32_t mode;

    if (tm_rd32(desc, 4u) != 5u || desc[0x0bu] != 8u ||
        desc[0x19u] != 13u || desc[0x21u] != 13u || desc[0x1bu] != 13u ||
        desc[0x0au] == 0u || desc[0x18u] == 0u || desc[0x20u] == 0u ||
        desc[0x1au] == 0u || desc[0x22u] == 0u)
        return -1;

    /* Frozen aa00 validates byte +0x24 in the descriptor base before the
     * active profile record is selected. */
    if (desc[0x24u] == 0u)
        return -1;

    sel = desc + (size_t)desc[0x0au] * 4u;
    word14 = tm_rd32(sel, 0x14u);
    mode = (word14 >> 5) & 0xfu;
    *special_publish_out = (word14 >> 4) & 1u;
    *active_profile_out = word14 & 0x7u;
    if (mode == 8u) {
        order[0] = 8u; order[1] = 7u; order[2] = 9u;
    } else if (mode == 9u) {
        order[0] = 9u; order[1] = 7u; order[2] = 8u;
    } else {
        order[0] = 7u; order[1] = 8u; order[2] = 9u;
    }
    *channel_mask = sel[0x0bu] & 0x0fu;
    *selected_out = sel;
    return 0;
}

static void tm_init_context_from_descriptor(uint8_t *ctx, const uint8_t *sel,
                                            uint32_t channel_mask)
{
    uint32_t w4 = tm_rd32(sel, 4u);
    uint32_t wc = tm_rd32(sel, 0x0cu);

    struct qrk3588_ddr_context_descriptor_state *descriptor_state =
        qrk3588_ddr_ctx_descriptor_state(ctx);
    descriptor_state->word4_hi16 = w4 >> 16;
    descriptor_state->word4_lo16 = w4 & 0xffffu;
    descriptor_state->wordc_bit26 = (wc >> 26) & 1u;
    descriptor_state->wordc_bit27 = (wc >> 27) & 1u;
    descriptor_state->wordc_bit19_inverted =
        ((wc ^ UINT32_C(0x80000)) >> 19) & 1u;

    for (unsigned i = 0; i < TRAIN_CHANNELS; ++i)
        qrk3588_ddr_ctx_store_route(ctx, i,
                (channel_mask & (1u << i)) ? i : QRK3588_DDR_CHANNEL_UNUSED);
}

static uint32_t tm_type_target_frequency(uint32_t type)
{
    const uint32_t *type_cfg = (const uint32_t *)dram_type_cfg_ptr(type);

    return type_cfg != NULL ? (type_cfg[0] & 0xfffu) : 0u;
}

static void tm_sync_handoff_preset(const uint8_t *active, uint32_t type)
{
    const struct qrk3588_ddr_channel_geometry *g =
        qrk3588_ddr_state_channel_geometry_const(active, 0u);
    uint32_t *preset = (uint32_t *)(uintptr_t)
        (type == 9u ? qrk3588_lp5_2400_preset() : qrk3588_lp4_2112_preset());

    preset[0] = 8u << g->bus_width_code;
    preset[1] = g->rank_count;
    preset[2] = g->cs_row_bits[0];
    preset[3] = g->cs_row_bits[1];
    preset[4] = g->cs_high16_row_bits[0];
    preset[5] = g->cs_high16_row_bits[1];
    preset[6] = 1u << g->bank_bits;
    preset[7] = g->column_bits;
    preset[8] = 8u << g->device_width_code;
    preset[9] = g->row_3_4;
}

/* Restore channel geometry from the retained topology snapshot encoded in
 * topology words 0x88/0x8c.  This is used only after the handoff magic has
 * been acknowledged. */
static int tm_restore_handoff_geometry(uint8_t *ctx, uint8_t *active,
                                       uint8_t *dram, uint8_t *timing,
                                       uint32_t *type_out)
{
    volatile uint32_t *topology =
        (volatile uint32_t *)(uintptr_t)qrk3588_ddr_ctx_topology_regs(ctx);
    uint32_t word88;
    uint32_t word8c;
    uint32_t type;

    if (topology == NULL || qrk3588_ddr_ctx_handoff_state_const(ctx)->valid == 0u)
        return 0;

    word88 = topology[0x88u / 4u];
    word8c = topology[0x8cu / 4u];
    type = word88 >> 27;
    if (type < 7u || type > 9u || dram_type_cfg_ptr(type) == NULL)
        return -1;

    if (ddr_build_frequency_params(active, dram, timing, 528u, type, 0u) != 0)
        return -1;

    for (uint32_t channel = 0u; channel < TRAIN_CHANNELS; ++channel) {
        const uint32_t packed = (word8c >> (channel * 8u)) & 0xffu;
        struct qrk3588_ddr_channel_geometry *g;
        uint32_t cs0_row;

        if (packed == 0u) {
            qrk3588_ddr_ctx_store_route(ctx, channel, QRK3588_DDR_CHANNEL_UNUSED);
            continue;
        }

        qrk3588_ddr_ctx_store_route(ctx, channel, channel);
        g = qrk3588_ddr_state_channel_geometry(active, channel);
        g->rank_count = ((word88 >> channel) & 1u) + 1u;
        g->column_bits = 10u;
        g->bank_bits = type == 9u ? 4u : 3u;
        g->bus_width_code = 1u;
        g->device_width_code = (word88 >> (channel + 12u)) & 1u;
        g->row_3_4 = (packed >> 2) & 1u;

        cs0_row = ((((packed >> 2) & 0xfu) + 1u) >> 1)
                + (type == 9u ? 13u : 14u);
        if ((packed >> 6) == 1u)
            ++cs0_row;
        g->cs_row_bits[0] = cs0_row;
        g->cs_high16_row_bits[0] = cs0_row;

        if (g->rank_count == 2u) {
            uint32_t cs1_row = cs0_row;
            if (((word88 >> (channel + 4u)) & 1u) != 0u)
                cs1_row = cs0_row - 1u;
            else if (((word88 >> (channel + 8u)) & 1u) != 0u)
                cs1_row = cs0_row + 1u;
            g->cs_row_bits[1] = cs1_row;
            g->cs_high16_row_bits[1] = cs1_row;
        } else {
            g->cs_row_bits[1] = 0u;
            g->cs_high16_row_bits[1] = 0u;
        }
    }

    tm_sync_handoff_preset(active, type);
    *type_out = type;
    return 1;
}

static int tm_prepare_handoff_channels(uint8_t *ctx, uint32_t type)
{
    volatile uint32_t *reset_regs;
    uint8_t *last_phy = NULL;

    if (qrk3588_ddr_ctx_handoff_state_const(ctx)->valid == 0u)
        return 0;

    for (uint32_t slot = 0u; slot < TRAIN_CHANNELS; ++slot) {
        uint32_t channel = qrk3588_ddr_ctx_route(ctx, slot);
        uint8_t *phy;
        uint8_t *pctl;
        if (channel == QRK3588_DDR_CHANNEL_UNUSED)
            continue;
        phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(ctx, channel);
        pctl = phy + 0x10000u;
        tm_wr32(pctl, 0x180u, tm_rd32(pctl, 0x180u) | 0x800u);
        if (qrk3588_ddr_poll32((uintptr_t)(pctl + 0x14u), 0x23u, 0x23u, 0u) != 0)
            return QRK3588_DDR_ETIMEDOUT;
    }

    reset_regs = (volatile uint32_t *)(uintptr_t)qrk3588_ddr_ctx_reset_regs(ctx);
    if (reset_regs != NULL) {
        uint32_t v = reset_regs[0x20u / 4u] & 0xffu;
        if (v != 0u)
            reset_regs[0x24u / 4u] = (v << 8) | (v << 24);
    }

    for (uint32_t slot = 0u; slot < TRAIN_CHANNELS; ++slot) {
        uint32_t channel = qrk3588_ddr_ctx_route(ctx, slot);
        uint8_t *phy;
        uint8_t *pctl;
        if (channel == QRK3588_DDR_CHANNEL_UNUSED)
            continue;
        phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(ctx, channel);
        pctl = phy + 0x10000u;
        last_phy = phy;
        tm_wr32(pctl, 0x180u, tm_rd32(pctl, 0x180u) & ~0x800u);
        if (qrk3588_ddr_poll32((uintptr_t)(pctl + 0x14u), 0x3u, 0x1u, 0u) != 0)
            return QRK3588_DDR_ETIMEDOUT;
    }

    if (last_phy == NULL)
        return -1;
    if (type == 9u) {
        (void)ddr_phy_reg_write((uint32_t)(uintptr_t)last_phy, 3u, 28u, 4u);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        udelay(1u);
    }
    {
        uint8_t *pctl = last_phy + 0x10000u;
        tm_wr32(pctl, 0x280u, tm_rd32(pctl, 0x280u) | 0x80000000u);
        ddr_pctl_trigger_handshake(last_phy);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        tm_wr32(pctl, 0x280u, tm_rd32(pctl, 0x280u) & 0x7fffffffu);
    }
    return 0;
}

static uint32_t tm_geometry_row_from_probe(uint32_t type, uint32_t value)
{
    const uint32_t code = (value >> 2) & 0xfu;
    return ((code + 1u) >> 1) + (type == 9u ? 13u : 14u);
}

static int tm_discover_channel_geometry(uint8_t *ctx, uint8_t *active,
                                        uint32_t channel, uint32_t type)
{
    static const uint8_t rank_probe_mr[3] = { 12u, 5u, 14u };
    struct qrk3588_ddr_channel_geometry *g =
        qrk3588_ddr_state_channel_geometry(active, channel);
    uint8_t *phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(ctx, channel);
    uint8_t *pctl = phy + 0x10000u;
    uint32_t saved_180;
    uint32_t rank0_probe;
    uint32_t rank1_probe;
    int dual_rank = 1;

    /* Frozen 0992c @ 0xff00b01c..0xff00b0f0 reconstructs geometry from
     * MR8 before probing rank presence.  These fields deliberately overwrite
     * the provisional preset geometry produced by the 528-MHz build. */
    g->bus_width_code = 1u;
    rank0_probe = ddr_phy_config_block(phy, 1u, 8u);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    g->bank_bits = type == 9u ? 4u : 3u;
    g->cs_row_bits[0] = tm_geometry_row_from_probe(type, rank0_probe);
    g->column_bits = 10u;
    g->device_width_code = 1u;
    if (((rank0_probe >> 6) & 3u) == 1u) {
        g->device_width_code = 0u;
        ++g->cs_row_bits[0];
    }
    g->row_3_4 = (rank0_probe >> 2) & 1u;

    saved_180 = tm_rd32(pctl, 0x180u);
    tm_wr32(pctl, 0x180u, 0u);

    if (qrk3588_ddr_state_frequency(active) > 799u) {
        int rc;

        /* Frozen has this branch, although the cold-discovery caller has just
         * built 528 MHz.  Preserve it so the recovered helper is faithful if
         * reused independently. */
        rc = ddr_config_write_seq(ctx, channel, 1u, active,
                                  0u, 0u, 0x10u, 1u, 0xffu);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        if (rc != 0)
            dual_rank = 0;
    } else {
        for (unsigned i = 0u; i < 3u; ++i) {
            uint32_t r0 = ddr_phy_config_block(phy, 1u, rank_probe_mr[i]);
            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();
            uint32_t r1 = ddr_phy_config_block(phy, 2u, rank_probe_mr[i]);
            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();
            if (((r0 ^ r1) & 0xffu) != 0u) {
                dual_rank = 0;
                break;
            }
        }
    }

    if (!dual_rank) {
        g->rank_count = 1u;
        g->cs_row_bits[1] = 0u;
        g->cs_high16_row_bits[1] = 0u;
        tm_wr32(pctl, 0x180u, saved_180);
        g->cs_high16_row_bits[0] = g->cs_row_bits[0];
        return 0;
    }

    rank1_probe = ddr_phy_config_block(phy, 2u, 8u);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    g->cs_row_bits[1] = tm_geometry_row_from_probe(type, rank1_probe);

    if (((rank1_probe >> 6) & 3u) == 1u) {
        if (g->device_width_code == 1u) {
            /* Frozen keeps w1==1 from the device-width comparison, so the
             * MR0 discriminator is read from rank0 here. */
            uint32_t mr0 = ddr_phy_config_block(phy, 1u, 0u);
            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();
            if ((mr0 & 2u) == 0u) {
                tm_wr32(pctl, 0x180u, saved_180);
                return -1;
            }
        }
        g->device_width_code = 0u;
        ++g->cs_row_bits[1];
    } else if (g->device_width_code == 0u) {
        uint32_t mr0 = ddr_phy_config_block(phy, 2u, 0u);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        if ((mr0 & 2u) == 0u) {
            tm_wr32(pctl, 0x180u, saved_180);
            return -1;
        }
    }

    g->rank_count = 2u;
    g->cs_high16_row_bits[1] = g->cs_row_bits[1];
    tm_wr32(pctl, 0x180u, saved_180);
    g->cs_high16_row_bits[0] = g->cs_row_bits[0];
    return 0;
}

enum tm_probe_result {
    TM_PROBE_RETRY = -1,
    TM_PROBE_FATAL = -2,
    TM_PROBE_OK = 0,
};

static int tm_probe_candidate(uint8_t *ctx, uint8_t *active, uint8_t *dram,
                              uint8_t *timing, uint32_t type)
{
    if (dram_type_cfg_ptr(type) == NULL)
        return TM_PROBE_RETRY;
    /* Frozen ac18/ac1c treats a failed 528-MHz parameter build as a
     * terminal setup error, unlike later candidate probe failures. */
    if (ddr_build_frequency_params(active, dram, timing, 528u, type, 0u) != 0)
        return TM_PROBE_FATAL;
    if (ddr_frequency_switch_program_sequence(ctx, active, 0u) != 0)
        return -1;

    for (unsigned slot = 0u; slot < TRAIN_CHANNELS; ++slot) {
        uint32_t ch = tm_channel_selector(ctx, slot);
        uint8_t *phy;
        int rc;

        if (ch == QRK3588_DDR_CHANNEL_UNUSED)
            continue;
        phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(ctx, ch);

        rc = ddr_config_write_seq(ctx, ch, 0u, active, 0u, 0u,
                                  type == 9u ? 0x18u : 0x10u,
                                  1u, 0xffu);
        if (qrk3588_ddr_get_error() != 0)
            return TM_PROBE_FATAL;
        if (rc != 0)
            return TM_PROBE_RETRY;

        if (type == 7u || type == 8u) {
            /* Frozen probes MR14 only on physical channel 0 to discriminate
             * LPDDR4 from LPDDR4X. */
            if (ch == 0u) {
                uint32_t mr14 = ddr_phy_config_block(phy, 1u, 14u);
                if (qrk3588_ddr_get_error() != 0)
                    return TM_PROBE_FATAL;
                if (mr14 != (type == 7u ? 77u : 93u))
                    return -1;
            }

            /* 0xff00af28..0xff00af6c: mirror the currently selected divider
             * into PHY MR14 machinery, then issue the second config phase. */
            (void)ddr_phy_reg_write((uint32_t)(uintptr_t)phy, 3u, 14u,
                                    tm_rd32(phy, 0x50cu) & 0xffffu);
            /* Native fail-fast: frozen waits forever on an indirect-PHY
             * timeout; do not issue the following config phase after the
             * bounded Native helper has latched an error. */
            if (qrk3588_ddr_get_error() != 0)
                return TM_PROBE_FATAL;
            rc = ddr_config_write_seq(ctx, ch, 0u, active, 0u, 0u,
                                      0x8u, 1u, 0xffu);
            if (qrk3588_ddr_get_error() != 0)
                return TM_PROBE_FATAL;
            if (rc != 0)
                return TM_PROBE_RETRY;
        }

        if (tm_discover_channel_geometry(ctx, active, ch, type) != 0)
            return -1;
    }
    return 0;
}

static void tm_edge_program(uint8_t *ctx, uint8_t *active, unsigned ch,
                            uint32_t flags, unsigned emit_ranks)
{
    volatile uint32_t *ctrl = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base1(ctx, ch);

    /* 0xb00..0xb44: the training flow clears the same packed training selectors before
     * walking the 8x8 edge matrix. */
    for (unsigned i = 0; i < 18u; ++i)
        ctrl[(0xb00u >> 2) + i] =
            (ctrl[(0xb00u >> 2) + i] & UINT32_C(0xfffc45ff)) | 0x200u;

    for (unsigned outer = 0; outer < 8u; ++outer) {
        for (unsigned inner = 0; inner < 8u; ++inner) {
            uint32_t bits;
            if (outer != 0u && inner != 0u)
                continue;
            bits = (outer << 11) | (inner << 15);
            ctrl[0xb48u >> 2] =
                (ctrl[0xb48u >> 2] & UINT32_C(0xfffc47ff)) | 0x200u | bits;
            ctrl[0xb4cu >> 2] =
                (ctrl[0xb4cu >> 2] & UINT32_C(0xfffc47ff)) | 0x200u | bits;
            ddr_phy_update_toggle((uintptr_t)ctrl);
            (void)ddr_config_write_seq(ctx, ch, 0u, active,
                                              0u, 0u, flags, 0u, 0xffu);
            /* Native fail-fast: stop the search at the first sticky timeout. */
            if (qrk3588_ddr_get_error() != 0)
                return;
        }
    }

    ddr_phy_update_toggle((uintptr_t)ctrl);
    for (unsigned rank = 0u; rank < emit_ranks; ++rank) {
        (void)ddr_config_write_seq(ctx, ch, rank, active,
                                  0u, 0u, flags, 0u, 0xffu);
        /* Native fail-fast: do not synthesize later-rank traffic after a
         * bounded helper has failed. */
        if (qrk3588_ddr_get_error() != 0)
            return;
    }
}

static int tm_record_valid(const struct train_record *r, unsigned ranks)
{
    if (r->rank0_low == 0u || r->rank0_high == 0u)
        return 0;
    if (ranks > 1u && (r->rank1_low == 0u || r->rank1_high == 0u))
        return 0;
    return 1;
}

static int tm_record_meets_direct0_threshold(const struct train_record *r,
                                             unsigned ranks,
                                             uint32_t threshold)
{
    if (r->rank0_low < threshold || r->rank0_high < threshold)
        return 0;
    if (ranks > 1u &&
        (r->rank1_low < threshold || r->rank1_high < threshold))
        return 0;
    return 1;
}

static int tm_record_has_zero(const struct train_record *r, unsigned ranks)
{
    if (r->rank0_low == 0u || r->rank0_high == 0u)
        return 1;
    if (ranks > 1u && (r->rank1_low == 0u || r->rank1_high == 0u))
        return 1;
    return 0;
}

static uint32_t tm_train_direct0(uint8_t *ctx, uint8_t *active, unsigned ch,
                                 unsigned ranks)
{
    const uint16_t *lookup = DDR_LAYOUT_REF(const uint16_t *, TRAIN_LOOKUP_SLOT);
    struct train_record *record = DDR_LAYOUT_PTR(struct train_record, TRAIN_SCRATCH_DIRECT0);
    const uint32_t threshold = qrk3588_ddr_state_dram_type(active) == 9u ? 25u : 33u;
    uint32_t lo, hi, ref, first = 0u, last = 0u;
    unsigned invalid_streak = 0u;
    int have = 0;

    (void)ddr_training_limits(&lo, &hi, &ref, 0u, ch);
    for (unsigned i = 0; i < TRAIN_LOOKUP_COUNT; ++i) {
        uint32_t actual = ddr_training_apply_limit(lookup[i], 0u, 0u, ch);
        if (qrk3588_ddr_get_error() != 0)
            return ref;
        ddr_memset(record, 0, sizeof(*record));
        (void)phy_update_seq(active, ctx, ch, ranks, record, NULL);
        /* Native fail-fast: the returned training value is irrelevant once a
         * bounded helper has latched an error; the caller checks the sticky
         * status immediately after this function returns. */
        if (qrk3588_ddr_get_error() != 0)
            return ref;

        /* The training path gives a zero-valued capture one immediate retry at
         * the same programmed limit before evaluating the threshold window. */
        if (tm_record_has_zero(record, ranks)) {
            (void)phy_update_seq(active, ctx, ch, ranks, record, NULL);
            if (qrk3588_ddr_get_error() != 0)
                return ref;
        }

        if (tm_record_meets_direct0_threshold(record, ranks, threshold)) {
            if (!have)
                first = actual;
            if (actual > last)
                last = actual;
            have = 1;
            invalid_streak = 0u;
        } else if (have) {
            ++invalid_streak;
            if (invalid_streak > 2u)
                break;
        }
    }

    (void)ddr_training_apply_limit(ref, 0u, 0u, ch);
    if (qrk3588_ddr_get_error() != 0)
        return ref;
    if (!have)
        return ref;
    return (first + last) >> 1;
}

static uint32_t tm_window_score(const struct train_record *records,
                                unsigned center, unsigned rank)
{
    uint32_t score = 0u;
    for (unsigned j = center - 3u; j <= center + 3u; ++j) {
        uint16_t low = rank ? records[j].rank1_low : records[j].rank0_low;
        uint16_t high = rank ? records[j].rank1_high : records[j].rank0_high;
        /* Vendor score is the sum of both eye edges, not eye width.
         * This preserves its bias/tie-break behavior across the 7-point
         * moving window. */
        score += (uint32_t)low + (uint32_t)high;
    }
    return score;
}

static uint32_t tm_train_direct1(uint8_t *ctx, uint8_t *active, unsigned ch,
                                 unsigned ranks, uint32_t *rank1_out)
{
    struct train_record *records = DDR_LAYOUT_PTR(struct train_record, TRAIN_SCRATCH_DIRECT1);
    uint32_t low, high, ref, count = 0u;
    uint32_t selected[2] = {0u, 0u};

    (void)ddr_training_limits(&low, &high, &ref, 1u, ch);
    /* Vendor grid is anchored to ref (same decimal residue), not to 0. */
    {
        uint32_t lower100 = ref >= 100u ? ref - 100u : 0u;
        uint32_t upper100 = ref + 100u;
        if (lower100 >= low) {
            low = lower100;
        } else if (low < ref) {
            low = ref - ((ref - low) / 10u) * 10u;
        }
        if (upper100 <= high) {
            high = upper100;
        } else if (high > ref) {
            high = ref + ((high - ref) / 10u) * 10u;
        }
    }

    if (high >= low) {
        for (uint32_t value = high;; value -= 10u) {
            struct train_record *r;
            (void)ddr_training_apply_limit(value, 3u, 1u, ch);
            if (qrk3588_ddr_get_error() != 0) {
                *rank1_out = 0u;
                return ref;
            }
            if (count >= TRAIN_DIRECT1_MAX_RECORDS)
                break;
            r = &records[count];
            ddr_memset(r, 0, sizeof(*r));
            /* Frozen be84 consumes the populated record and never branches
             * on phy_update_seq() w0. */
            (void)phy_update_seq(active, ctx, ch, ranks, NULL, r);
            /* Native fail-fast: do not continue the 10-unit sweep after the
             * first bounded-wait failure. */
            if (qrk3588_ddr_get_error() != 0) {
                *rank1_out = 0u;
                return ref;
            }
            if (tm_record_valid(r, ranks)) {
                r->value = value;
                ++count;
            }
            if (value <= low || value < 10u)
                break;
        }
    }
    (void)ddr_training_apply_limit(ref, 3u, 1u, ch);
    if (qrk3588_ddr_get_error() != 0) {
        *rank1_out = 0u;
        return ref;
    }

    for (unsigned rank = 0; rank < ranks; ++rank) {
        uint32_t best_score = 0u;
        unsigned best = 0u;
        if (count >= 7u) {
            best = 3u;
            best_score = tm_window_score(records, best, rank);
            for (unsigned center = 4u; center + 3u < count; ++center) {
                uint32_t score = tm_window_score(records, center, rank);
                if (score > best_score) {
                    best_score = score;
                    best = center;
                }
            }
            selected[rank] = records[best].value;
        } else {
            selected[rank] = ref;
        }
    }
    /* Frozen keeps the unused rank1 result zero on single-rank channels.
     * Do not mirror rank0 into the second slot: that changes the published
     * training image even though rank1 is absent. */
    *rank1_out = selected[1];
    return selected[0];
}

static int tm_train_channel(uint8_t *ctx, uint8_t *active, unsigned ch,
                            int direct0_enabled, int direct1_enabled)
{
    unsigned ranks = tm_rank_count(active, ch);
    uint32_t midpoint = 0u, direct1_rank0 = 0u, direct1_rank1 = 0u;
    struct qrk3588_ddr_training_result *result =
        qrk3588_ddr_state_training_result(active, ch);

    ddr_phy_state_capture_and_clear(ctx, ch, active);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    ddr_memset(DDR_LAYOUT_PTR(void, TRAIN_SCRATCH_EDGE), 0, 0x40u);
    tm_edge_program(ctx, active, ch, 0x10u, ranks);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    ddr_memset(DDR_LAYOUT_PTR(void, TRAIN_SCRATCH_DIRECT0), 0, 0xfcu);
    ddr_memset(DDR_LAYOUT_PTR(void, TRAIN_SCRATCH_DIRECT1), 0, 0xfcu);

    /* Frozen 0992c derives these two enables from descriptor word0c bits
     * 21/22 only when active frequency is >1065 MHz.  At <=1065 it clears
     * both local enables but still performs the surrounding edge/finalization
     * stages.  Keep the frozen direct0->direct1->apply ordering when both are
     * enabled; that ordering is observable through the helper event stream. */
    if (direct0_enabled) {
        midpoint = tm_train_direct0(ctx, active, ch, ranks);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
    }
    if (direct1_enabled) {
        direct1_rank0 =
            tm_train_direct1(ctx, active, ch, ranks, &direct1_rank1);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
    }

    if (direct0_enabled) {
        result->direct0_rank0 = midpoint;
        result->direct0_rank1 = midpoint;
        (void)ddr_training_apply_limit(midpoint, 3u, 0u, ch);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
    }
    if (direct1_enabled) {
        result->direct1_rank0 = direct1_rank0;
        result->direct1_rank1 = direct1_rank1;
        (void)ddr_training_apply_limit(direct1_rank0, 3u, 1u, ch);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();

        {
            const uint32_t dram_type = qrk3588_ddr_state_dram_type(active);
            uint32_t encoded = result->direct1_rank0;
            const uint32_t divider_code =
                ddr_freq_divider_encode(&encoded, dram_type) & 0xffffu;
            result->direct1_rank0 = encoded;

            if (dram_type == 9u) {
                struct qrk3588_ddr_lp5_channel_image *image =
                    qrk3588_ddr_state_lp5_channel_image(active, ch);
                image->byte[7] = (uint8_t)divider_code;
                image->byte[8] = (uint8_t)divider_code;
            } else {
                /* Frozen c5a4..c5c8 LP4/LP4X commit: clear the PHY-bank
                 * service word, replace only PHY+0x50c.low16 with the encoded
                 * divider, then complete the PHY update. */
                uint8_t *const phy = (uint8_t *)(uintptr_t)
                    qrk3588_ddr_ctx_channel_base0(ctx, ch);
                uint8_t *const bank = phy + UINT32_C(0x10000);
                tm_wr32(bank, 0xc80u, 0u);
                tm_wr32(phy, 0x50cu,
                        (tm_rd32(phy, 0x50cu) & UINT32_C(0xffff0000)) |
                        divider_code);
                ddr_phy_update_complete((uint32_t)(uintptr_t)phy);
                if (qrk3588_ddr_get_error() != 0)
                    return qrk3588_ddr_get_error();
            }
        }
    }

    ddr_memset(DDR_LAYOUT_PTR(void, TRAIN_SCRATCH_EDGE), 0, 0x40u);
    tm_edge_program(ctx, active, ch, 0x10u, 0u);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    /* Frozen c12c/c6b0 emits one full-operation config transaction per
     * populated rank.  Single-rank channels must not synthesize rank1. */
    for (unsigned rank = 0u; rank < ranks; ++rank) {
        (void)ddr_config_write_seq(ctx, ch, rank, active,
                                  0u, 0u, 0x18u, 0u, 0xffu);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
    }
    (void)phy_update_seq(active, ctx, ch, ranks, NULL, NULL);
    /* Native fail-fast: frozen cannot pass this point if the helper's wait
     * never completes, so avoid restore MMIO after a bounded timeout. */
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    ddr_phy_state_restore(ctx, ch, active);
    return 0;
}

static int tm_private_eye_report_prefix(uint8_t *ctx, uint8_t *active)
{
    struct qrk3588_ddr_private_eye_config *config =
        DDR_LAYOUT_REF(struct qrk3588_ddr_private_eye_config *,
                       QRK3588_DDR_PRIVATE_EYE_CONFIG_SLOT_VA);
    volatile uint8_t *cache_id =
        DDR_LAYOUT_PTR(volatile uint8_t, QRK3588_DDR_CHANNEL_CACHE_ID_VA);
    volatile uint8_t *cache_set =
        DDR_LAYOUT_PTR(volatile uint8_t, QRK3588_DDR_CHANNEL_CACHE_SET_VA);
    volatile uint8_t *cache_width =
        DDR_LAYOUT_PTR(volatile uint8_t, QRK3588_DDR_CHANNEL_CACHE_WIDTH_VA);
    volatile uint16_t *cache =
        DDR_LAYOUT_PTR(volatile uint16_t, QRK3588_DDR_CHANNEL_CACHE_DATA_VA);
    const uint32_t dram_type = qrk3588_ddr_state_dram_type(active);
    const uint32_t frequency = qrk3588_ddr_state_frequency(active);
    uint32_t result[TRAIN_CHANNELS] = {0u, 0u, 0u, 0u};
    uintptr_t primary;
    uint32_t divider;
    uint32_t phase_count;
    uint32_t base_tick;
    uint32_t half_tick;
    uint32_t step;

    if (config == NULL)
        return -1;

    /* Frozen preamble normalizes the per-channel controller service bit and
     * mirrors the training state for every populated slot. */
    for (uint32_t channel = 0u; channel != TRAIN_CHANNELS; ++channel) {
        if (qrk3588_ddr_ctx_route(ctx, channel) == QRK3588_DDR_CHANNEL_UNUSED)
            continue;
        uintptr_t base0 = qrk3588_ddr_ctx_channel_base0(ctx, channel);
        if (base0 == 0u)
            return -1;
        volatile uint32_t *control = (volatile uint32_t *)(uintptr_t)
            (base0 + UINT32_C(0x10100));
        uint32_t value = *control;
        if ((value & 1u) != 0u) {
            if (dram_type == 9u)
                value &= ~1u;
            else
                value |= 4u;
            *control = value;
        }
        (void)ddr_training_mirror_entry(1u, channel, 0u, 3u);
    }

    /* Frozen code clears two contiguous 90-byte halfword tables directly,
     * then clears the three byte-sized cache selectors. */
    for (uint32_t i = 0u; i != 90u; ++i)
        cache[i] = 0u;
    *cache_set = 0u;
    *cache_width = 0u;
    *cache_id = 0u;

    primary = qrk3588_ddr_ctx_channel_base1(ctx, 0u);
    if (primary == 0u || frequency == 0u)
        return -1;
    divider = tm_rd32((const void *)(uintptr_t)primary, 0xb4u) & 0x7ffu;
    phase_count = (tm_rd32((const void *)(uintptr_t)primary, 0xb0u) & 1u) + 1u;
    if (divider == 0u)
        return -1;

    step = (divider * phase_count) / 6u + 1u;
    base_tick = UINT32_C(1000000000) / frequency;
    base_tick /= divider;
    base_tick /= phase_count;
    if (base_tick == 0u)
        return -1;
    half_tick = base_tick >> 1;

    config->private_2c = step;
    config->private_30 = step;
    config->io_selector = 0x4000u;
    config->width_mode =
        qrk3588_ddr_state_channel_geometry_const(active, 0u)->bus_width_code;
    config->private_64 = 20u;
    config->private_68 = 20u;
    config->private_38 = 1u;
    config->private_48 = 1u;
    config->private_4c = 1u;
    config->private_50 = 1u;
    config->io_base[2] = 0u;
    config->io_base[3] = 0u;
    config->private_34 = 0u;
    config->private_58 = (half_tick + 16000u) / base_tick;
    config->private_5c = (half_tick + 37000u) / base_tick;
    config->private_54 = (half_tick + 43000u) / base_tick;
    config->private_60 = (half_tick + 35000u) / base_tick;
    config->private_6c = 0u;

    for (uint32_t channel = 0u; channel != TRAIN_CHANNELS; ++channel) {
        if (qrk3588_ddr_ctx_route(ctx, channel) == QRK3588_DDR_CHANNEL_UNUSED)
            continue;
        const uint64_t base = (uint64_t)channel << 33;
        config->io_base[0] = base;
        if (qrk3588_ddr_state_channel_rank_count(active, channel) == 2u) {
            uintptr_t base0 = qrk3588_ddr_ctx_channel_base0(ctx, channel);
            if (base0 == 0u)
                return -1;
            uint32_t shift =
                (tm_rd32((const void *)(uintptr_t)(base0 + UINT32_C(0x30000)),
                         0x04u) & 0x3fu) + 7u;
            config->io_base[1] = base + (UINT64_C(1) << (shift & 63u));
        } else {
            config->io_base[1] = 0u;
        }
        config->io_selector = 0x4000u;
        config->channel = channel;

        ddr_phy_state_capture_and_clear(ctx, channel, active);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        result[channel] = (uint32_t)ddr_eye_scan_report(config);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        ddr_phy_state_restore(ctx, channel, active);
    }

    /* Frozen code only reports result[] here.  It then enters a vendor service
     * teardown, performs absolute GRF/UART writes and spins forever.  All
     * the required scratch and hardware state is retained; BL2 stops
     * before that unsafe continuation regardless of eye-scan result. */
    (void)result;
    return -1;
}

static int tm_verify_cold_memory(uint8_t *ctx)
{
    int failed = 0;

    if (qrk3588_ddr_ctx_handoff_state_const(ctx)->valid != 0u)
        return 0;

    for (uint32_t slot = 0u; slot != TRAIN_CHANNELS; ++slot) {
        const uint32_t channel = qrk3588_ddr_ctx_route(ctx, slot);
        if (channel == QRK3588_DDR_CHANNEL_UNUSED)
            continue;

        qrk3588_ddr_active_context->error_channel = channel;
        const uintptr_t base = (uintptr_t)channel << 33;
        for (uint32_t pass = 0u; pass != 2u; ++pass) {
            for (uint32_t word = 0u; word != 16u; ++word) {
                uint32_t expected = UINT32_C(1) << word;
                if (pass != 0u)
                    expected = ~expected;
                volatile uint32_t *addr = (volatile uint32_t *)(uintptr_t)
                    qrk3588_ddr_resolve_addr(base + (uintptr_t)word * sizeof(uint32_t));
                *addr = expected;
            }
            for (uint32_t word = 0u; word != 16u; ++word) {
                uint32_t expected = UINT32_C(1) << word;
                if (pass != 0u)
                    expected = ~expected;
                volatile const uint32_t *addr = (volatile const uint32_t *)(uintptr_t)
                    qrk3588_ddr_resolve_addr(base + (uintptr_t)word * sizeof(uint32_t));
                uint32_t observed = *addr;
                if (channel == 0u && pass == 0u && word < 4u)
                    qrk3588_ddr_active_context->memory_trace.first_words[word] =
                        observed;
                if (observed != expected) {
                    struct qrk3588_ddr_memory_trace *trace =
                        &qrk3588_ddr_active_context->memory_trace;

                    if (trace->valid == 0u) {
                        trace->address = base +
                            (uintptr_t)word * sizeof(uint32_t);
                        trace->expected = expected;
                        trace->observed = observed;
                        trace->channel = channel;
                        trace->pass = pass;
                        trace->word = word;
                        trace->valid = 1u;
                    }
                    failed = -1;
                }
            }
        }
    }
    return failed;
}

static void tm_patch_high_frequency_phy(uint8_t *ctx, const uint8_t *active,
                                        uint32_t profile0_frequency)
{
    if (profile0_frequency <= 1065u)
        return;

    for (uint32_t slot = 0u; slot != TRAIN_CHANNELS; ++slot) {
        const uint32_t channel = qrk3588_ddr_ctx_route(ctx, slot);
        if (channel == QRK3588_DDR_CHANNEL_UNUSED)
            continue;
        uint8_t *phy = (uint8_t *)(uintptr_t)
            qrk3588_ddr_ctx_channel_base1(ctx, channel);
        const uint32_t rank_bits =
            qrk3588_ddr_state_channel_rank_count(active, channel) == 1u ? 1u : 3u;

        tm_wr32(phy, 0xa24u, tm_rd32(phy, 0xa24u) | (rank_bits << 18));
        tm_wr32(phy, 0xa48u, tm_rd32(phy, 0xa48u) | (rank_bits << 10));
        tm_wr32(phy, 0xa58u, tm_rd32(phy, 0xa58u) | (rank_bits << 10));
        tm_wr32(phy, 0x004u, tm_rd32(phy, 0x004u) | 0x04000000u);
    }
}

static int tm_publish(uint8_t *ctx, uint8_t *active,
                      const uint8_t *selected,
                      uint32_t active_profile,
                      uint32_t special_publish)
{
    const uint8_t *desc = DDR_LAYOUT_CPTR(uint8_t, TRAIN_DESC_ADDR);
    const uint32_t type = qrk3588_ddr_state_dram_type(active);
    const uint32_t hash_selector = desc[type == 9u ? 0x24u : 0x22u];
    const uint32_t *hash = (const uint32_t *)(const void *)
        (desc + (size_t)hash_selector * sizeof(uint32_t));
    const uint32_t grf_profile = selected[0x0au];
    const int grf_result = grf_ddr_config(ctx, active, hash, grf_profile);
    const uint32_t *type_cfg;
    uint32_t frequency[4];

    qrk3588_ddr_state_set_grf_config_result(active, (uint32_t)grf_result);

    if (special_publish != 0u) {
        for (uint32_t channel = 0u; channel != TRAIN_CHANNELS; ++channel) {
            const uintptr_t base2 = qrk3588_ddr_ctx_channel_base2(ctx, channel);
            if (base2 == 0u)
                return -1;
            tm_wr32((void *)(uintptr_t)base2, 0x24u, UINT32_C(0x20002000));
        }
        /* Frozen retained-handoff service calls an absolute private entry
         * point and then spins forever.  Native BL2 preserves the safe prefix
         * and fails closed before that non-relocatable continuation. */
        if (qrk3588_ddr_ctx_handoff_state_const(ctx)->valid == 1u)
            return -1;
        goto finalize;
    }

    type_cfg = (const uint32_t *)dram_type_cfg_ptr(type);
    if (type_cfg == NULL)
        return -1;

    frequency[0] = type_cfg[0] & 0xfffu;
    frequency[1] = (type_cfg[0] >> 12) & 0xfffu;
    frequency[2] = type_cfg[1] & 0xfffu;
    frequency[3] = (type_cfg[1] >> 12) & 0xfffu;

    if (qrk3588_ddr_ctx_handoff_state_const(ctx)->valid == 0u) {
        ddr_memset(DDR_LAYOUT_PTR(void, TRAIN_PUBLISH_DST), 0,
                   TRAIN_DVFS_IMAGE_SIZE);
        ddr_memset(DDR_LAYOUT_PTR(void, TRAIN_DVFS_IMAGE), 0,
                   TRAIN_DVFS_IMAGE_SIZE);
    }

    /* Frozen scheduling programs every non-active profile first. */
    for (uint32_t profile = 1u; profile != 4u; ++profile) {
        if (profile != active_profile) {
            (void)phy_dvfs_config(frequency[profile], profile);
            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();
        }
    }
    if (active_profile != 0u) {
        (void)phy_dvfs_config(frequency[0], 0u);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
    }
    (void)phy_dvfs_config(active_profile < 4u ? frequency[active_profile] : 0u,
                          active_profile);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();

    tm_patch_high_frequency_phy(ctx, active, frequency[0]);

    if (qrk3588_ddr_ctx_handoff_state_const(ctx)->valid == 0u) {
        (void)ddr_memcpy(DDR_LAYOUT_PTR(void, TRAIN_PUBLISH_DST),
                         DDR_LAYOUT_CPTR(void, TRAIN_DVFS_IMAGE),
                         TRAIN_DVFS_IMAGE_SIZE);
    } else {
        return -1;
    }
finalize:
    sdram_params_init();
    ddr_phy_channel_init(ctx);
    (void)ddr_copy_descriptor_block(DDR_LAYOUT_PTR(void, TRAIN_DESCRIPTOR_COPY_DST),
                      DDR_LAYOUT_CPTR(void, TRAIN_DESC_ADDR), 0x130u, type);
    ddr_debug_snapshot(tm_rd32(selected, 0x10u));
    return 0;
}

__attribute__((noinline, noclone))
int ddr_training_main(void)
{
    uint8_t *ctx = tm_context();
    uint8_t *active = tm_active();
    uint8_t *timing = tm_timing();
    uint8_t *dram = tm_dram();
    const uint8_t *selected;
    uint32_t order[3], mask;
    uint32_t selected_type = 0u;
    uint32_t active_profile = 0u;
    uint32_t special_publish = 0u;

    if (ctx == NULL || active == NULL || timing == NULL || dram == NULL)
        return -1;

    ddr_context_base_table_init((uint64_t *)ctx);
    if (tm_descriptor_validate(&selected, order, &mask, &active_profile,
                               &special_publish) != 0)
        return -1;
    tm_init_context_from_descriptor(ctx, selected, mask);
    const int handoff_valid = qrk3588_ddr_ctx_capture_topology_handoff(ctx);

    if (ddr_controller_reset_required(ctx))
        ddr_controller_reset_handshake(ctx);
    if (qrk3588_ddr_get_error() != 0)
        return -1;
    ddr_clear_aux_status_words(ctx);

    /* Frozen common attempt entry ff00ab2c. */
    ddr_memset(active, 0, sizeof(struct qrk3588_ddr_state_image));

    if (handoff_valid != 0) {
        int target_ready = 0;

        for (unsigned i = 0u; i < 3u; ++i) {
            int restored;

            if (i != 0u)
                ddr_memset(active, 0,
                           sizeof(struct qrk3588_ddr_state_image));

            restored = tm_restore_handoff_geometry(ctx, active, dram, timing,
                                                   &selected_type);
            if (restored <= 0)
                return -1;

            {
                const uint32_t target_frequency =
                    tm_type_target_frequency(selected_type);
                if (target_frequency == 0u)
                    return -1;
                if (ddr_build_frequency_params(active, dram, timing,
                                               target_frequency,
                                               selected_type, 1u) != 0)
                    continue;
            }
            if (ddr_frequency_switch_program_sequence(ctx, active, 1u) != 0) {
                if (qrk3588_ddr_get_error() != 0)
                    return qrk3588_ddr_get_error();
                continue;
            }
            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();

            target_ready = 1;
            break;
        }
        if (target_ready == 0)
            return -1;
    } else {
        int target_ready = 0;

        for (unsigned i = 0u; i < 3u; ++i) {
            const uint32_t type = order[i];
            int probe_rc;

            if (i != 0u)
                ddr_memset(active, 0,
                           sizeof(struct qrk3588_ddr_state_image));
            probe_rc = tm_probe_candidate(ctx, active, dram, timing, type);

            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();
            if (probe_rc == TM_PROBE_FATAL)
                return -1;
            if (probe_rc != TM_PROBE_OK)
                continue;

            {
                const uint32_t target_frequency =
                    tm_type_target_frequency(type);
                if (target_frequency == 0u)
                    continue;
                if (ddr_build_frequency_params(active, dram, timing,
                                               target_frequency, type, 1u) != 0)
                    continue;
            }
            if (ddr_frequency_switch_program_sequence(ctx, active, 1u) != 0) {
                if (qrk3588_ddr_get_error() != 0)
                    return qrk3588_ddr_get_error();
                continue;
            }
            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();

            selected_type = type;
            target_ready = 1;
            break;
        }
        if (target_ready == 0)
            return -1;
    }
    if (tm_prepare_handoff_channels(ctx, selected_type) != 0)
        return -1;

    for (unsigned slot = 0; slot < TRAIN_CHANNELS; ++slot) {
        uint32_t ch = tm_channel_selector(ctx, slot);
        uint32_t rank_count;
        uint32_t training_mode;
        int training_result;
        int update_result;

        if (ch == 0x0fu)
            continue;

        rank_count = tm_rank_count(active, ch);
        /*
         * PHY mode 3 trains both chip selects and is valid only for dual-rank
         * channels.
         */
        training_mode = rank_count == 2u ? 3u : 0u;
        qrk3588_ddr_active_context->error_dram_type = selected_type;
        qrk3588_ddr_active_context->error_channel = ch;
        qrk3588_ddr_active_context->error_rank_count = rank_count;
        qrk3588_ddr_active_context->error_phase =
            QRK3588_DDR_PHASE_INITIAL_TRAINING;
        training_result = ddr_config_write_seq(ctx, ch, training_mode,
                                               active, 0u, 0u, 0x18u,
                                               1u, 0xffu);
        qrk3588_ddr_active_context->initial_training_result[ch] =
            training_result;
        /* Frozen ba04 does not branch on ddr_config_write_seq() w0.  Keep
         * the result for diagnostics; only the native sticky hardware error
         * (for bounded-wait safety adaptation) is fatal here. */
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();

        /* Frozen 0992c ba08..ba3c performs this low-frequency transaction
         * between the initial config write and phy_update_seq. */
        if (qrk3588_ddr_state_frequency(active) <= 1065u) {
            uint8_t *const base1 = (uint8_t *)(uintptr_t)
                qrk3588_ddr_ctx_channel_base1(ctx, ch);
            tm_wr32(base1, 0x04u, tm_rd32(base1, 0x04u) | 0x20u);
            tm_wr32(base1, 0x10u, tm_rd32(base1, 0x10u) | 0x200u);
            ddr_phy_update_toggle((uintptr_t)base1);
            tm_wr32(base1, 0x10u, tm_rd32(base1, 0x10u) & ~0x200u);
        }

        /*
         * This pass establishes provisional PHY margins. A training-result
         * miss is validated by the following memory test and normal channel
         * training; transport and programming errors remain fatal here.
         */
        update_result = phy_update_seq(active, ctx, ch, rank_count,
                                       NULL, NULL);
        qrk3588_ddr_active_context->initial_phy_update_result[ch] =
            update_result;
        /* Frozen ba68 ignores phy_update_seq() w0 and consumes hardware
         * state.  The bounded native poll layer reports real transport/timeouts
         * through qrk3588_ddr_get_error(). */
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        ddr_sync_controller_b4_high11(ctx, active, ch);
        print_dram_info_chan(ctx, ch, active);
    }

    for (unsigned slot = 0; slot < TRAIN_CHANNELS; ++slot) {
        uint32_t ch = tm_channel_selector(ctx, slot);
        if (ch != 0x0fu) {
            uint8_t *phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(ctx, ch);
            (void)ddr_phy_config_block(phy, 1u, 5u);
            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();
            break;
        }
    }
    ddr_channel_topology_and_timing_init(ctx, active);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    if (selected_type == 9u)
        ddr_publish_phy_channel_status();

    qrk3588_ddr_active_context->error_phase =
        QRK3588_DDR_PHASE_MEMORY_TEST;
    if (handoff_valid == 0 && tm_verify_cold_memory(ctx) != 0) {
        ddr_controller_reset_handshake(ctx);
        return -1;
    }

    {
        const uint32_t training_word = tm_rd32(selected, 0x0cu);
        const int high_frequency =
            qrk3588_ddr_state_frequency(active) > 1065u;
        const int direct0_enabled =
            high_frequency && ((training_word & (1u << 21)) != 0u);
        const int direct1_enabled =
            high_frequency && ((training_word & (1u << 22)) != 0u);

        for (unsigned slot = 0; slot < TRAIN_CHANNELS; ++slot) {
            uint32_t ch = tm_channel_selector(ctx, slot);

            if (ch == 0x0fu)
                continue;
            qrk3588_ddr_active_context->error_channel = ch;
            qrk3588_ddr_active_context->error_rank_count =
                tm_rank_count(active, ch);
            qrk3588_ddr_active_context->error_phase =
                QRK3588_DDR_PHASE_CHANNEL_TRAINING;
            if (tm_train_channel(ctx, active, ch,
                                 direct0_enabled, direct1_enabled) != 0)
                return -1;
            if (qrk3588_ddr_get_error() != 0)
                return qrk3588_ddr_get_error();
        }
    }

    /* Descriptor bit23 selects a private eye/report service mode after ordinary
     * per-channel training. Preserve the required scratch and MMIO setup,
     * then stop before the terminal vendor UART/GRF service loop. */
    if ((tm_rd32(selected, 0x0cu) & (1u << 23)) != 0u)
        return tm_private_eye_report_prefix(ctx, active);

    qrk3588_ddr_active_context->error_phase = QRK3588_DDR_PHASE_PUBLISH;
    return tm_publish(ctx, active, selected, active_profile, special_publish);
}
