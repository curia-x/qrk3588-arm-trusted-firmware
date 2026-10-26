/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* DDR training and parameter construction. */
#include "qrk3588_ddr_internal.h"

int ddr_training_mirror_invalid(void)
{
    return -1;
}

/* Program the PHY timing ratios selected by the encoded operating point. */
void ddr_phy_timing_program(
    uint32_t phy_base, uint32_t channel,
    const struct qrk3588_ddr_channel_geometry *geometry,
    uint32_t dram_type, uint32_t setting)
{
    uint64_t encoded = ddr_geometry_capacity_component(geometry, 0u, dram_type);
    if (geometry->rank_count == 2u) {
        uint64_t alternate =
            ddr_geometry_capacity_component(geometry, 1u, dram_type);
        if (encoded <= alternate)
            encoded = alternate;
    }

    uint32_t shift = geometry->bus_width_code + 0x14u - geometry->device_width_code;
    uint32_t point = (uint32_t)(encoded >> shift);
    if (dram_type != 7u && dram_type != 8u && dram_type != 9u)
        return;

    if (dram_type == 9u) {
        uintptr_t channel_base = (uintptr_t)channel << 20;
        uint32_t select = *(volatile uint32_t *)(uintptr_t)
            (phy_base + channel_base + 0xd00u);
        setting = (select & 1u) ? setting >> 2 : setting >> 1;
    }

    uint32_t fine;
    uint32_t coarse;
    uint32_t ratio;
    if (point <= 0x100u) {
        ratio = 0x3cu;
        coarse = 0x82u;
        fine = ratio;
    } else if (point <= 0x200u) {
        ratio = 0x5au;
        coarse = 0xb4u;
        fine = ratio;
    } else if (point <= 0x400u) {
        ratio = 0x5au;
        fine = 0x78u;
        coarse = 0xd2u;
    } else {
        ratio = 0x5au;
        fine = point <= 0x800u ? 0xbeu : 0x8cu;
        coarse = point <= 0x800u ? 0x17cu : 0x118u;
    }

    volatile uint32_t *bank = (volatile uint32_t *)(uintptr_t)
        (phy_base + 0x10000u);
    uint32_t encoded_fine;
    if (bank[0x200u / 4u] & (1u << 8))
        encoded_fine = (fine * setting + 999u) / 1000u;
    else
        encoded_fine = (coarse * setting + 999u) / 1000u;

    uint32_t duration = (coarse + 7u) * setting + ((setting >> 1) + 999u);
    if (duration <= 0x7cfu)
        duration = 2u;
    else
        duration /= 1000u;

    uintptr_t channel_base = (uintptr_t)channel << 20;
    bank[0xc80u / 4u] = 0u;
    volatile uint32_t *reg604 = (volatile uint32_t *)(uintptr_t)
        (phy_base + channel_base + 0x604u);
    *reg604 = (*reg604 & 0xfffff000u) | encoded_fine;
    volatile uint32_t *reg038 = (volatile uint32_t *)(uintptr_t)
        (phy_base + channel_base + 0x38u);
    *reg038 = (*reg038 & 0xffffff00u) | duration;
    volatile uint32_t *reg608 = (volatile uint32_t *)(uintptr_t)
        (phy_base + channel_base + 0x608u);
    *reg608 = (*reg608 & 0xff00ffffu) |
        (((ratio * setting + 999u) / 1000u) << 16);
    ddr_phy_update_complete(phy_base);
}

/* Execute the PHY indirect-register read handshake for one channel. */
void ddr_phy_reg_read(void *context, uint32_t channel,
                                 const uint32_t *config, uint32_t command,
                                 uint32_t frequency)
{
    uint32_t phy_base = (uint32_t)qrk3588_ddr_ctx_channel_base0(context, channel);
    volatile uint32_t *bank = (volatile uint32_t *)(uintptr_t)
        (phy_base + 0x10000u);
    volatile uint32_t *controller = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base2(context, channel);
    volatile uint32_t *table = (volatile uint32_t *)
        qrk3588_ddr_ctx_shared_bank(context, channel);
    volatile uint32_t *request = table + ((channel & 1u) + 0x4cu);

    bank[0xc80u / 4u] = 0u;
    bank[0x510u / 4u] &= ~1u;
    bank[0x510u / 4u] |= 0x20u;
    ddr_phy_update_complete(phy_base);
    if (qrk3588_ddr_get_error() != 0)
        return;
    if (qrk3588_ddr_poll32((uintptr_t)&bank[0x514u / 4u], 1u, 0u, 0u) != 0)
        return;

    *request = 0x80000u;
    udelay(1u);
    if (config[0] == 9u)
        controller[0xa00u / 4u] = 0x20002u;
    *request = 0x30000u;
    fout_mhz_setup(context, channel, command, frequency);
    controller[0x300u / 4u] = command | 0x10000u;
    controller[0xa00u / 4u] = 0x20000u;
    *request = 0x80008u;

    bank[0xc80u / 4u] = 0u;
    bank[0x510u / 4u] &= ~0x20u;
    ddr_phy_update_complete(phy_base);
    if (qrk3588_ddr_get_error() != 0)
        return;
    if (qrk3588_ddr_poll32((uintptr_t)&bank[0x514u / 4u], 1u, 1u, 0u) != 0)
        return;
    bank[0xc80u / 4u] = 0u;
    bank[0x510u / 4u] |= 1u;
    ddr_phy_update_complete(phy_base);
}

/* Extract a signed training window and clamp it to the PHY delay range. */
int ddr_training_window(int32_t *low, int32_t *high,
                                   uint32_t read_set, int32_t sample,
                                   uint32_t ten_bit, uint32_t channel)
{
    if (sample < -1 || sample > 16)
        return -1;

    uint8_t *context = DDR_LAYOUT_REF(uint8_t *, QRK3588_DDR_CONTEXT_SLOT_VA);
    volatile uint8_t *results = (volatile uint8_t *)
        qrk3588_ddr_ctx_channel_base1(context, channel);
    volatile uint8_t *reference = (volatile uint8_t *)
        qrk3588_ddr_ctx_channel_base1(context, 0u);
    uint32_t delay = *(volatile uint32_t *)(reference + 0xb4u) & 0x7ffu;
    uint32_t scale = *(volatile uint32_t *)(reference + 0xb0u);
    int32_t limit = (int32_t)((delay << ((scale & 1u) ? 3u : 2u)) / 10u);

    uint32_t minimum = 0x3ffu;
    uint32_t maximum = 0u;
    if (sample == -1) {
        uint32_t count = ten_bit ? 18u : 16u;
        for (uint32_t i = 0; i != count; ++i) {
            uint32_t value;
            if (ten_bit) {
                uint32_t group = i / 9u;
                uint32_t lane = i - group * 9u;
                uintptr_t offset = lane * 12u + (read_set ? 0x410u : 0x1f0u);
                uint32_t word = *(volatile uint32_t *)(results + offset);
                value = (word >> (group * 16u)) & 0x3ffu;
            } else {
                uintptr_t offset = (i & 4u) + 0x978u + (i >> 3) * 12u;
                uint32_t word = *(volatile uint32_t *)(results + offset);
                value = (word >> ((i & 3u) * 8u)) & 0x7fu;
            }
            if (value < minimum) minimum = value;
            if (value > maximum) maximum = value;
        }
        *low = -(int32_t)minimum;
        *high = (int32_t)(ten_bit ? 0x3ffu : 0x6fu) - (int32_t)maximum;
    } else {
        uint32_t value;
        if (ten_bit) {
            uintptr_t offset = (uint32_t)(sample & 7) * 12u +
                (read_set ? 0x410u : 0x1f0u);
            uint32_t word = *(volatile uint32_t *)(results + offset);
            value = (word >> (((uint32_t)sample >> 3) * 16u)) & 0x3ffu;
            *high = (int32_t)(0x3ffu - value);
        } else {
            uintptr_t offset = ((uint32_t)sample & 4u) + 0x978u +
                ((uint32_t)sample >> 3) * 12u;
            uint32_t word = *(volatile uint32_t *)(results + offset);
            value = (word >> (((uint32_t)sample & 3u) * 8u)) & 0x7fu;
            *high = (int32_t)(0x6fu - value);
        }
        *low = -(int32_t)value;
    }

    if (*low < -limit) *low = -limit;
    if (*high > limit) *high = limit;
    return 0;
}

/* Apply a delta to cached PHY training samples, preserving the baseline. */
int ddr_training_adjust(int32_t delta, uint32_t read_set,
                                   int32_t sample, uint32_t ten_bit,
                                   uint32_t channel)
{
    uint8_t *context = DDR_LAYOUT_REF(uint8_t *, QRK3588_DDR_CONTEXT_SLOT_VA);
    volatile uint32_t *phy = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base1(context, channel);
    volatile uint8_t *cache = DDR_LAYOUT_PTR(volatile uint8_t, QRK3588_DDR_CHANNEL_CACHE_DATA_VA);
    volatile uint8_t *cache_channel = DDR_LAYOUT_PTR(volatile uint8_t, QRK3588_DDR_CHANNEL_CACHE_ID_VA);
    volatile uint8_t *cache_set = DDR_LAYOUT_PTR(volatile uint8_t, QRK3588_DDR_CHANNEL_CACHE_SET_VA);
    volatile uint8_t *cache_width = DDR_LAYOUT_PTR(volatile uint8_t, QRK3588_DDR_CHANNEL_CACHE_WIDTH_VA);

    if (*cache_set != read_set || *cache_width != ten_bit ||
        *cache_channel != channel) {
        for (uint32_t i = 0; i != 180u; i += 2u)
            *(volatile uint16_t *)(cache + i) = 0u;
        *cache_set = (uint8_t)read_set;
        *cache_width = (uint8_t)ten_bit;
        *cache_channel = (uint8_t)channel;
    }

    if (ten_bit)
        phy[4u / 4u] |= 0x20u;

    uint32_t first = sample == -1 ? 0u : (uint32_t)sample;
    uint32_t count = sample == -1 ? (ten_bit ? 18u : 16u) : first + 1u;
    int failed = 0;
    for (uint32_t i = first; i < count; ++i) {
        uintptr_t reg_offset;
        uint32_t shift;
        uint32_t mask;
        if (ten_bit) {
            reg_offset = 0x880u + (i % 9u) * 4u +
                (read_set ? 0x60u : 0u);
            shift = (i / 9u) * 16u;
            mask = 0x3ffu;
        } else {
            reg_offset = 0x978u + (i & 4u) + (i >> 3) * 12u;
            shift = (i & 3u) * 8u;
            mask = 0x7fu;
        }

        volatile int16_t *saved_bank = (volatile int16_t *)cache +
            read_set * 45u;
        volatile int16_t *saved = sample == -1 ? saved_bank : saved_bank + i;
        uint32_t word = *(volatile uint32_t *)((uintptr_t)phy + reg_offset);
        int32_t value = (int32_t)((word >> shift) & mask) - *saved + delta;

        if (sample == -1) {
            if (value < 0 || value > (int32_t)mask)
                failed = -1;
        } else {
            /* A single-sample negative result saturates to zero and succeeds;
             * only upper overflow is rejected before the PHY write. */
            *saved = (int16_t)delta;
            if (value > (int32_t)mask)
                return -1;
        }

        if (value < 0) value = 0;
        if (value > (int32_t)mask) value = (int32_t)mask;
        word = (word & ~(mask << shift)) | ((uint32_t)value << shift);
        *(volatile uint32_t *)((uintptr_t)phy + reg_offset) = word;
    }

    if (sample == -1) {
        volatile int16_t *saved_bank = (volatile int16_t *)cache + read_set * 45u;
        *saved_bank = (int16_t)delta;
    }
    if (failed)
        return -1;
    if (ten_bit) {
        phy[0x10u / 4u] |= 0x200u;
        ddr_phy_update_toggle((uintptr_t)phy);
        phy[0x10u / 4u] &= ~0x200u;
    } else {
        ddr_phy_update_toggle((uintptr_t)phy);
    }
    return 0;
}



/* Report mode-register values for every populated channel and rank. */
void print_mode_registers(uint8_t *config, uint32_t verbose)
{
    static const uint32_t lp4_short[] = {0, 4, 5, 6, 7, 8, 18, 19, 24, 25};
    static const uint32_t lp4_full[] =
        {0, 4, 5, 6, 7, 8, 21, 26, 27, 29, 43, 44, 45, 57};
    static const uint32_t other_short[] = {12, 14};
    static const uint32_t other_full[] = {1, 3, 11, 12, 14, 15, 17, 19, 41};
    uint8_t *context = DDR_LAYOUT_REF(uint8_t *, QRK3588_DDR_CONTEXT_SLOT_VA);
    uint32_t dram_type = qrk3588_ddr_state_dram_type(config);

    for (uint32_t slot = 0; slot != 4u; ++slot) {
        uint32_t channel = qrk3588_ddr_ctx_route(context, slot);
        if (channel == 15u)
            continue;
        uint8_t *phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(context, channel);
        uint32_t ranks = qrk3588_ddr_state_channel_rank_count(config, channel);
        uint8_t *channel_cfg = config + (uintptr_t)channel * 0x14u;
        for (uint32_t rank = 0; rank < ranks; ++rank) {
            const uint32_t *mrs;
            uint32_t count;
            if (dram_type == 7u || dram_type == 8u) {
                mrs = verbose ? lp4_full : lp4_short;
                count = verbose ? 14u : 10u;
            } else {
                mrs = verbose ? other_full : other_short;
                count = verbose ? 9u : 2u;
            }

            ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR_CHANNEL_RANK_VA), channel, rank);
            for (uint32_t i = 0; i != count; ++i) {
                uint32_t value = ddr_phy_config_block(phy, 1u << rank, mrs[i]);
                if (qrk3588_ddr_get_error() != 0)
                    return;
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR_GENERIC_VA), mrs[i], value);
            }

            if (verbose && (dram_type == 7u || dram_type == 8u)) {
                volatile uint32_t *bank = (volatile uint32_t *)(phy + 0x10000u);
                uintptr_t offset = (uintptr_t)(bank[8u / 4u] & 3u) << 20;
                uint32_t mr = *(volatile uint32_t *)(phy + offset + 0x500u);
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR1_VA), mr >> 16);
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR2_VA), mr & 0xffffu);
                mr = *(volatile uint32_t *)(phy + offset + 0x504u);
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR3_VA), mr >> 16);
                mr = *(volatile uint32_t *)(phy + offset + 0x508u);
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR11_VA), mr >> 16);
                mr = *(volatile uint32_t *)(phy + offset + 0x50cu) >> 16;
                if (dram_type == 8u && rank == 1u)
                    mr = (mr & 0xc7u) | 0x28u;
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR22_VA), mr);
            } else if (verbose && dram_type == 9u) {
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR2_VA),
                           channel_cfg[0x1d5u]);
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR10_VA),
                           channel_cfg[0x1d7u]);
                ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_MR18_VA),
                           channel_cfg[0x1dfu]);
            }
            ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_NEWLINE_VA));
        }
    }
}

void ddr_phy_channel_restore_and_program(void *ctx,
                                                     uint32_t channel,
                                                     void *snapshot)
{
    volatile uint32_t *phy = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base0(ctx, channel);
    volatile uint32_t *controller = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base1(ctx, channel);
    volatile uint32_t *table = (volatile uint32_t *)
        qrk3588_ddr_ctx_shared_bank(ctx, channel);
    volatile uint32_t *bank = (volatile uint32_t *)
        ((uintptr_t)phy + 0x10000u);
    uint32_t pair = channel & 1u;
    uint32_t table_offset = pair * 0x30u;
    uint32_t restore = ddr_phy_restore_margin(ctx, channel, snapshot);

    table[(table_offset + 0x18u) / 4u] = restore | 0xffff0000u;
    table[(table_offset + 4u) / 4u] = 0x001f0017u;
    table[(table_offset + 0x24u) / 4u] =
        (bank[0x100u / 4u] & 1u) ? 0x2fff2fffu : 0x2fff2000u;
    table[pair + 0x4cu] = 0x40004000u;

    controller[0xe04u / 4u] |= 0x80000000u;
    if (qrk3588_ddr_ctx_descriptor_state_const(ctx)->word4_hi16 != 0u)
        bank[0x180u / 4u] |= 0x200u;

    uint32_t channel_mode = *(const uint32_t *)
        qrk3588_ddr_state_channel_const(snapshot, channel);
    uint32_t selector = channel_mode == 2u ? 3u : 1u;
    uint32_t value = bank[0x180u / 4u] & 0xffffff0fu;
    if (qrk3588_ddr_ctx_descriptor_state_const(ctx)->word4_lo16 != 0u)
        value |= selector << 4;
    bank[0x180u / 4u] = value;

    value = bank[0x180u / 4u] & 0xfffffff0u;
    if (qrk3588_ddr_ctx_descriptor_state_const(ctx)->word4_hi16 != 0u)
        value |= selector;
    bank[0x180u / 4u] = value;
}

/* Select the three training limits for one PHY channel. */
uint32_t ddr_training_limits(uint32_t *out0, uint32_t *out1,
                                        uint32_t *out2, uint32_t scaled,
                                        uint32_t channel)
{
    /*
     * QRK3588_DDR_RUNTIME_VA(0x610) is a pointer SLOT, not the channel array itself:
     *   context = *(void **)QRK3588_DDR_RUNTIME_VA(0x610);
     *   phy     = *(void **)(context + channel * 0x20);
     * QRK3588_DDR_RUNTIME_VA(0x5e0) is another pointer slot whose value points at a slot
     * containing the active configuration pointer (two loads in the ROM).
     */
    uint8_t *context = DDR_LAYOUT_REF(uint8_t *, QRK3588_DDR_CONTEXT_SLOT_VA);
    uint8_t *phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(context, channel);
    volatile uint32_t *bank = (volatile uint32_t *)((uintptr_t)phy + 0x10000u);
    uint8_t **config_slot = DDR_LAYOUT_REF(uint8_t **, QRK3588_DDR_CONFIG_POINTER_SLOT_VA);
    uint8_t *config = *config_slot;
    uint32_t dram_type = qrk3588_ddr_state_dram_type(config);

    if (scaled == 0u) {
        volatile uint32_t *controller = (volatile uint32_t *)
            qrk3588_ddr_ctx_channel_base1(context, channel);
        const uint16_t *lookup = DDR_LAYOUT_REF(const uint16_t *, QRK3588_DDR_LOOKUP_SLOT_VA);
        uint32_t index = (controller[0x3ecu / 4u] & 0x3fu);
        *out2 = lookup[index];
        if (dram_type == 7u) {
            *out1 = 0x2bbu;
        } else if (dram_type == 8u) {
            *out1 = 0x258u;
        } else {
            *out1 = 0x1f4u;
        }
        *out0 = 0xa6u;
        return 0u;
    }

    if (dram_type == 9u) {
        const struct qrk3588_ddr_lp5_channel_image *image =
            qrk3588_ddr_state_lp5_channel_image_const(config, channel);
        uint32_t byte = image->byte[7];
        *out2 = byte * 5u + 0x64u;
        *out0 = 0x64u;
        *out1 = 0x2dfu;
        return 0u;
    }

    uint32_t bank_select = (bank[2] & 3u) << 20;
    uint32_t word = *(volatile uint32_t *)
        ((uintptr_t)phy + bank_select + 0x50cu);
    uint32_t bit = (word >> 6) & 1u;
    uint32_t low = word & 0x3fu;
    if (dram_type == 7u) {
        *out2 = low * 4u + 0x64u + bit * 0xb3u;
        *out0 = 0x64u;
        *out1 = 0x1a4u;
    } else {
        *out2 = low * 6u + 0x96u + bit * 0x78u;
        *out0 = 0x96u;
        *out1 = 0x275u;
    }
    return 0u;
}

uint32_t ddr_training_apply_limit(uint32_t limit, uint32_t selector,
                                              uint32_t direct, uint32_t channel)
{
    uint8_t *context = DDR_LAYOUT_REF(uint8_t *, QRK3588_DDR_CONTEXT_SLOT_VA);
    if (direct == 0u) {
        volatile uint32_t *controller = (volatile uint32_t *)
            qrk3588_ddr_ctx_channel_base1(context, channel);
        const uint16_t *lookup = DDR_LAYOUT_REF(const uint16_t *, QRK3588_DDR_LOOKUP_SLOT_VA);
        if (limit > 0x2bbu) limit = 0x2bbu;
        if (limit < 0xa6u) limit = 0xa6u;
        uint32_t index;
        for (index = 0u; index != 0x40u; ++index)
            if (limit <= lookup[index]) break;
        if (index == 0x40u) index = 0u;
        controller[0x3ecu / 4u] =
            (controller[0x3ecu / 4u] & ~0x3f3fu) | index | (index << 8);
        ddr_phy_update_toggle((uintptr_t)controller);
        return lookup[index];
    }

    uint8_t *phy = (uint8_t *)qrk3588_ddr_ctx_channel_base0(context, channel);
    uint8_t **config_slot = DDR_LAYOUT_REF(uint8_t **, QRK3588_DDR_CONFIG_POINTER_SLOT_VA);
    uint8_t *config = *config_slot;
    uint32_t encoded = limit;
    ddr_freq_divider_encode(&encoded, qrk3588_ddr_state_dram_type(config));
    uint32_t mask = selector == 3u ? 3u : 1u << selector;
    (void)ddr_phy_reg_write((uint32_t)(uintptr_t)phy, mask, 14u, encoded);
    if (qrk3588_ddr_get_error() != 0)
        return encoded;
    (void)ddr_phy_reg_write((uint32_t)(uintptr_t)phy, mask, 15u, encoded);
    return encoded;
}

/* Resolve per-channel state before entering the shared mirror operation. */
int ddr_training_mirror_entry(uint32_t mode, uint32_t channel,
                                         uint32_t clear, uint32_t channel_mask)
{
    if (mode < 1u || mode > 2u)
        return ddr_training_mirror_invalid();

    uint8_t *context = DDR_LAYOUT_REF(uint8_t *, QRK3588_DDR_CONTEXT_SLOT_VA);
    void *state = (void *)qrk3588_ddr_ctx_channel_base1(context, channel);
    return ddr_training_mirror(mode, state, clear, channel_mask);
}

/* Copy or clear the selected per-channel training-result mirror. */
int ddr_training_mirror(uint32_t mode, void *state,
                                   uint32_t clear, uint32_t channel_mask)
{
    uint8_t *p = (uint8_t *)state;
    for (uint32_t lane = 0; lane != 2; ++lane) {
        if (((channel_mask >> lane) & 1u) == 0u)
            continue;
        if (mode == 2u) {
            uint32_t first_dst = lane ? 0x814u : 0x7c0u;
            uint32_t second_dst = lane ? 0x818u : 0x7c4u;
            if (clear) {
                *(uint32_t *)(p + first_dst) = 0;
                *(uint32_t *)(p + second_dst) = 0;
            } else {
                *(uint32_t *)(p + second_dst) = *(uint32_t *)(p + 0x580u);
                *(uint32_t *)(p + first_dst) = *(uint32_t *)(p + (lane ? 0x574u : 0x764u));
            }
            uint32_t dst_a = lane ? 0x7ccu : 0x824u;
            uint32_t dst_b = lane ? 0x854u : 0x7f4u;
            uint32_t src_a = lane ? 0x190u : 0x770u;
            uint32_t src_b = lane ? 0x614u : 0x794u;
            for (uint32_t i = 0; i != 8; ++i) {
                if (clear) {
                    *(uint32_t *)(p + dst_a + i * 4u) = 0;
                    *(uint32_t *)(p + dst_b + i * 4u) = 0;
                } else {
                    uint32_t source_index = lane ? i * 3u : i;
                    *(uint32_t *)(p + dst_a + i * 4u) = *(uint32_t *)(p + src_a + source_index * 4u);
                    *(uint32_t *)(p + dst_b + i * 4u) = *(uint32_t *)(p + src_b + source_index * 4u);
                }
            }
            if (clear) {
                *(uint32_t *)(p + dst_a + 0x1cu) = 0;
                *(uint32_t *)(p + dst_b + 0x1cu) = 0;
            } else {
                *(uint32_t *)(p + dst_a + 0x1cu) = *(uint32_t *)(p + src_a - 4u);
                *(uint32_t *)(p + dst_b + 0x1cu) = *(uint32_t *)(p + src_b - 4u);
            }
        } else {
            uint32_t first_dst = lane ? 0xdd8u : 0xdd0u;
            uint32_t second_dst = lane ? 0xddcu : 0xdd4u;
            if (clear) {
                *(uint32_t *)(p + first_dst) = 0;
                *(uint32_t *)(p + second_dst) = 0;
            } else {
                *(uint32_t *)(p + first_dst) = *(uint32_t *)(p + 0xdc0u);
                *(uint32_t *)(p + second_dst) = *(uint32_t *)(p + (lane ? 0xdccu : 0xdd4u));
            }
            uint32_t dst_a = lane ? 0x880u : 0x8e0u;
            uint32_t dst_b = lane ? 0x8b0u : 0x910u;
            uint32_t src_a = lane ? 0x1f0u : 0x410u;
            uint32_t src_b = lane ? 0x2a0u : 0x0f0u;
            for (uint32_t i = 0; i != 10; ++i) {
                uint32_t tail = i == 9u ? (uint32_t)-8 : 0u;
                if (clear) {
                    *(uint32_t *)(p + dst_a + i * 4u) = 0;
                    *(uint32_t *)(p + dst_b + i * 4u) = 0;
                } else {
                    *(uint32_t *)(p + dst_a + i * 4u) = *(uint32_t *)(p + src_a + i * 12u + tail);
                    *(uint32_t *)(p + dst_b + i * 4u) = *(uint32_t *)(p + src_b + i * 12u + tail);
                }
            }
        }
    }
    return 0;
}

/*
 * Capture per-rank PHY training/timing registers from channel state into the
 * 0x17c-byte-per-channel snapshot/configuration record.  Rank 0 uses one
 * source-offset set and later ranks use another;
 * preserving that asymmetry is important (it is not a simple memcpy).
 */
void ddr_capture_channel_training_state(void *context_ptr,
                                                    uint32_t channel,
                                                    uint8_t *snapshot,
                                                    const uint8_t *rank_info)
{
    uint8_t *context = (uint8_t *)context_ptr;
    volatile uint8_t *phy_state = (volatile uint8_t *)
        qrk3588_ddr_ctx_channel_base1(context, channel);
    uint8_t *ch = snapshot + (uintptr_t)channel * 0x17cu;
    uint32_t rank_count = *(const uint32_t *)
        qrk3588_ddr_state_channel_const(rank_info, channel);

    /* Fixed five-word prefix: phy_state[0xbb0..0xbc0] -> ch[0x1a4..0x1b4]. */
    for (uint32_t i = 0; i != 5u; ++i)
        *(uint32_t *)(ch + 0x1a4u + i * 4u) =
            *(volatile uint32_t *)(phy_state + 0xbb0u + i * 4u);

    /* Temporarily select mode 4 in state+0x10 while taking state+0x6c. */
    uint32_t mode = *(volatile uint32_t *)(phy_state + 0x10u);
    *(volatile uint32_t *)(phy_state + 0x10u) = (mode & ~7u) | 4u;
    *(uint32_t *)(ch + 0x1c8u) = *(volatile uint32_t *)(phy_state + 0x6cu);
    *(volatile uint32_t *)(phy_state + 0x10u) &= ~7u;

    static const uint16_t grp_a[2][9] = {
        {0x18c, 0x190, 0x19c, 0x1a8, 0x1b4, 0x1c0, 0x1cc, 0x1d8, 0x1e4},
        {0x76c, 0x770, 0x774, 0x778, 0x77c, 0x780, 0x784, 0x788, 0x78c},
    };
    static const uint16_t grp_b[2][9] = {
        {0x610, 0x614, 0x620, 0x62c, 0x638, 0x644, 0x650, 0x65c, 0x668},
        {0x790, 0x794, 0x798, 0x79c, 0x7a0, 0x7a4, 0x7a8, 0x7ac, 0x7b0},
    };
    static const uint16_t grp_c[2][10] = {
        {0x1f0, 0x1fc, 0x208, 0x214, 0x220, 0x22c, 0x238, 0x244, 0x250, 0x254},
        {0x410, 0x41c, 0x428, 0x434, 0x440, 0x44c, 0x458, 0x464, 0x470, 0x474},
    };
    static const uint16_t grp_d[2][10] = {
        {0x490, 0x49c, 0x4a8, 0x4b4, 0x4c0, 0x4cc, 0x4d8, 0x4e4, 0x4f0, 0x4f4},
        {0x500, 0x50c, 0x518, 0x524, 0x530, 0x53c, 0x548, 0x554, 0x560, 0x564},
    };
    static const uint16_t scalar0[2] = {0x0f0, 0x5e4};
    static const uint16_t scalar1[2] = {0x0fc, 0x5f0};
    static const uint16_t mirror0[2] = {0x574, 0x764};
    static const uint16_t mirror1[2] = {0x580, 0x768};
    static const uint16_t tail0[2] = {0xdc0, 0xdc8};
    static const uint16_t tail1[2] = {0xdc4, 0xdcc};

    uint8_t *pair = ch + 0x1b8u;
    uint8_t *group9 = ch + 0x1ccu;
    uint8_t *group10 = ch + 0x26cu;

    for (uint32_t rank = 0; rank != rank_count; ++rank) {
        uint32_t set = rank == 0u ? 0u : 1u;

        *(uint32_t *)(pair + 0u) = *(volatile uint32_t *)(phy_state + scalar0[set]);
        *(uint32_t *)(pair + 4u) = *(volatile uint32_t *)(phy_state + scalar1[set]);

        for (uint32_t i = 0; i != 9u; ++i)
            *(uint32_t *)(group9 + i * 4u) =
                *(volatile uint32_t *)(phy_state + grp_a[set][i]);

        *(uint32_t *)(pair + 0xa4u) = *(volatile uint32_t *)(phy_state + mirror0[set]);
        *(uint32_t *)(pair + 0xa8u) = *(volatile uint32_t *)(phy_state + mirror1[set]);

        for (uint32_t i = 0; i != 9u; ++i)
            *(uint32_t *)(group9 + 0x48u + i * 4u) =
                *(volatile uint32_t *)(phy_state + grp_b[set][i]);

        for (uint32_t i = 0; i != 10u; ++i)
            *(uint32_t *)(group10 + i * 4u) =
                *(volatile uint32_t *)(phy_state + grp_c[set][i]);
        for (uint32_t i = 0; i != 10u; ++i)
            *(uint32_t *)(group10 + 0x50u + i * 4u) =
                *(volatile uint32_t *)(phy_state + grp_d[set][i]);

        *(uint32_t *)(pair + 0x154u) = *(volatile uint32_t *)(phy_state + tail0[set]);
        *(uint32_t *)(pair + 0x158u) = *(volatile uint32_t *)(phy_state + tail1[set]);

        pair += 8u;
        group9 += 0x24u;
        group10 += 0x28u;
    }
}

/*
 * Normalize per-channel/rank capacity geometry into the compact tag52
 * region-list format consumed by the configuration parser.  A bounded
 * 160-byte scratch list handles the 4/8/16-GiB address-hole cases as explicit
 * region operations.
 */
typedef struct {
    uint64_t start;
    uint64_t size;
} ddr_region_t;

typedef struct {
    uint32_t count;
    ddr_region_t region[10];
} ddr_region_list_t;

static void tag52_store_regions(uint8_t *out, const ddr_region_list_t *list)
{
    *(uint32_t *)(out + 0x00u) = list->count;
    for (uint32_t i = 0; i < list->count; ++i)
        *(uint64_t *)(out + 0x08u + (uintptr_t)i * 8u) = list->region[i].start;
    for (uint32_t i = 0; i < list->count; ++i)
        *(uint64_t *)(out + 0x08u + (uintptr_t)(list->count + i) * 8u) =
            list->region[i].size;
}

static void region_append(ddr_region_list_t *list, uint64_t start, uint64_t size)
{
    if (list->count < (uint32_t)(sizeof(list->region) / sizeof(list->region[0]))) {
        list->region[list->count].start = start;
        list->region[list->count].size = size;
        ++list->count;
    }
}

static void normalize_tag52_regions(ddr_region_list_t *dst,
                                    const ddr_region_list_t *src,
                                    uint32_t special_low_hole,
                                    uint32_t insert_extra,
                                    uint64_t extra_start,
                                    uint64_t extra_size)
{
    const uint64_t low_limit = special_low_hole ? UINT64_C(0xf6ffffff)
                                                 : UINT64_C(0xefffffff);
    const uint64_t low_size = special_low_hole ? UINT64_C(0xf7000000)
                                                : UINT64_C(0xf0000000);
    const uint64_t four_gib = UINT64_C(0x100000000);
    const uint64_t special_hi_start = UINT64_C(0xff140000);
    const uint64_t special_hi_size = UINT64_C(0x00eb0000);

    dst->count = 0u;
    for (uint32_t i = 0; i < src->count; ++i) {
        const uint64_t start = src->region[i].start;
        const uint64_t size = src->region[i].size;
        const uint64_t end = start + size;

        if (start > UINT64_C(0xffffffff)) {
            region_append(dst, start, size);
        } else {
            region_append(dst, start, end <= low_limit ? size : low_size);
            /* Emit the 4-GiB continuation only when end is strictly above
             * 4 GiB.  The low-hole path also emits the ff140000 bridge once
             * end crosses 0xffffffff, including the exact-4-GiB case. */
            if (special_low_hole && end > UINT64_C(0xffffffff))
                region_append(dst, special_hi_start, special_hi_size);
            if (end > four_gib)
                region_append(dst, four_gib, end - four_gib);
        }

        if (insert_extra != 0u) {
            const uint64_t extra_end = extra_start + extra_size;
            const uint32_t next = i + 1u;
            if (next >= src->count ||
                (extra_start > end && extra_end < src->region[next].start)) {
                region_append(dst, extra_start, extra_size);
                insert_extra = 0u;
            }
        }
    }
}

static void patch_channel_map_byte(void *context, uint32_t active_count,
                                   uint32_t value)
{
    uint8_t *ctx = (uint8_t *)context;
    for (uint32_t i = 0; i < active_count; ++i) {
        volatile uint32_t *reg = (volatile uint32_t *)
            qrk3588_ddr_ctx_channel_output(ctx, i);
        uint32_t old = reg[3];
        reg[3] = (old & UINT32_C(0xffff00ff)) | value;
    }
}

static void set_channel_map_word(void *context, uint32_t active_count,
                                 uint32_t value)
{
    uint8_t *ctx = (uint8_t *)context;
    for (uint32_t i = 0; i < active_count; ++i) {
        volatile uint32_t *reg = (volatile uint32_t *)
            qrk3588_ddr_ctx_channel_output(ctx, i);
        reg[3] = value;
    }
}

void ddr_topology_region_normalize(void *context, void *aux_ptr,
                                   void *out_ptr, uint32_t flag)
{
    uint8_t *ctx = (uint8_t *)context;
    uint8_t *aux = (uint8_t *)aux_ptr;
    uint8_t *out = (uint8_t *)out_ptr;
    uint64_t rank_size[4][2];
    uint64_t channel_size[4];
    uint64_t raw_total = 0u;
    uint32_t active_count = 0u;
    uint32_t equal_channels = 1u;
    const uint32_t dram_type = qrk3588_ddr_state_dram_type(aux);
    volatile uint32_t *global = (volatile uint32_t *)qrk3588_ddr_ctx_grf_regs(ctx);
    const uint32_t status = global[0x300u / 4u];
    const uint32_t remap_enabled = status & 1u;
    const uint32_t map_selector = (status >> 4) & 0xfu;
    const uint32_t topology_mode =
        qrk3588_ddr_state_channel_frequency_profile_selector(aux, 0u);
    ddr_region_list_t initial;
    ddr_region_list_t normalized;

    ddr_memset(rank_size, 0, sizeof(rank_size));
    ddr_memset(channel_size, 0, sizeof(channel_size));
    ddr_memset(&initial, 0, sizeof(initial));
    ddr_memset(&normalized, 0, sizeof(normalized));

    for (uint32_t slot = 0; slot < 4u; ++slot) {
        const uint32_t route = qrk3588_ddr_ctx_route(ctx, slot);
        uint8_t *ccfg = qrk3588_ddr_state_channel(aux, slot);
        if (route != 0x0fu) {
            uint64_t r0 = ddr_geometry_capacity_component((const struct qrk3588_ddr_channel_geometry *)(const void *)ccfg, 0u, dram_type);
            uint64_t r1 = ddr_geometry_capacity_component((const struct qrk3588_ddr_channel_geometry *)(const void *)ccfg, 1u, dram_type);
            ++active_count;
            raw_total += r0 + r1;
            if (qrk3588_ddr_state_channel_row_3_4(aux, slot) != 0u) {
                r0 = (r0 * 3u) >> 2;
                r1 = (r1 * 3u) >> 2;
            }
            rank_size[slot][0] = r0;
            rank_size[slot][1] = r1;
            channel_size[slot] = r0 + r1;
            /* The ROM compares channel sizes only for active routes.  An
             * inactive slot is zeroed but does not by itself clear the
             * uniform-channel flag. */
            if (slot != 0u && channel_size[slot] != channel_size[0])
                equal_channels = 0u;
        }
    }

    ddr_memset(out, 0, 0xb8u);

    if (remap_enabled != 0u) {
        if (topology_mode > 3u) {
            const uint64_t visible_total = channel_size[0] + channel_size[1] +
                                           channel_size[2] + channel_size[3];
            /* The selector chooses the remap window: <=7 uses the 16-GiB
             * window, >7 uses the 8-GiB window. */
            if (map_selector <= 7u && visible_total > UINT64_C(0x3fc000000)) {
                if (visible_total > UINT64_C(0x400000000)) {
                    region_append(&initial, 0u, UINT64_C(0x3fc000000));
                    region_append(&initial, UINT64_C(0x3fc500000), UINT64_C(0x03a00000));
                    region_append(&initial, UINT64_C(0x400000000),
                                  visible_total - UINT64_C(0x400000000));
                } else {
                    region_append(&initial, 0u, UINT64_C(0x3fc000000));
                    region_append(&initial, UINT64_C(0x3fc500000), UINT64_C(0x03a00000));
                }
            } else if (map_selector > 7u && visible_total > UINT64_C(0x1fe000000)) {
                if (visible_total > UINT64_C(0x200000000)) {
                    region_append(&initial, 0u, UINT64_C(0x1fe000000));
                    region_append(&initial, UINT64_C(0x1fe300000), UINT64_C(0x01c00000));
                    /* Preserve the firmware arithmetic exactly: this
                     * branch subtracts 0x20000000 (512 MiB), not the 8-GiB
                     * remap base 0x200000000.  The asymmetric 512-MiB
                     * subtraction is intentional and covered by differential
                     * tests. */
                    region_append(&initial, UINT64_C(0x200000000),
                                  visible_total - UINT64_C(0x20000000));
                } else {
                    region_append(&initial, 0u, UINT64_C(0x1fe000000));
                    region_append(&initial, UINT64_C(0x1fe300000), UINT64_C(0x01c00000));
                }
            } else {
                region_append(&initial, 0u, visible_total);
            }
        } else {
            const uint64_t rank0_total = rank_size[0][0] + rank_size[1][0] +
                                         rank_size[2][0] + rank_size[3][0];
            const uint64_t rank1_total = rank_size[0][1] + rank_size[1][1] +
                                         rank_size[2][1] + rank_size[3][1];
            if (map_selector <= 7u && rank0_total > UINT64_C(0x3fc000000)) {
                region_append(&initial, 0u, UINT64_C(0x3fc000000));
                region_append(&initial, UINT64_C(0x3fc500000), UINT64_C(0x03a00000));
                region_append(&initial, UINT64_C(0x400000000), rank1_total);
            } else if (map_selector > 7u && rank0_total > UINT64_C(0x1fe000000)) {
                region_append(&initial, 0u, UINT64_C(0x1fe000000));
                region_append(&initial, UINT64_C(0x1fe300000), UINT64_C(0x01c00000));
                region_append(&initial, UINT64_C(0x200000000), rank1_total);
            } else if (rank_size[0][1] == 0u) {
                /* Channel-0 rank1 presence gates exposure of the aggregated
                 * rank1 payload as a second remapped region. */
                region_append(&initial, 0u, rank0_total);
            } else {
                region_append(&initial, 0u, rank0_total);
                region_append(&initial,
                              map_selector > 7u ? UINT64_C(0x200000000)
                                                : UINT64_C(0x400000000),
                              rank1_total);
            }
        }
    } else if (topology_mode <= 3u && *(uint32_t *)aux == 2u) {
        if (active_count == 4u) {
            for (uint32_t i = 0; i < 8u; ++i)
                region_append(&initial, (uint64_t)i << 32,
                              rank_size[i >> 1][i & 1u]);
        } else {
            /* With fewer than four active channels the ROM still emits a
             * fixed four-region 0/4/8/12-GiB topology, populated from the
             * first two channel rank pairs. */
            region_append(&initial, 0u, rank_size[0][0]);
            region_append(&initial, UINT64_C(0x100000000), rank_size[0][1]);
            region_append(&initial, UINT64_C(0x200000000), rank_size[1][0]);
            region_append(&initial, UINT64_C(0x300000000), rank_size[1][1]);
        }
    } else if (equal_channels != 0u) {
        region_append(&initial, 0u,
                      channel_size[0] + channel_size[1] +
                      channel_size[2] + channel_size[3]);
    } else if (active_count == 4u) {
        uint64_t start = 0u;
        for (uint32_t i = 0; i < 4u; ++i) {
            region_append(&initial, start, channel_size[i]);
            start += channel_size[0];
        }
    } else {
        region_append(&initial, 0u, channel_size[0]);
        region_append(&initial, channel_size[0], channel_size[1]);
    }

    /* Side effects that the ROM couples to the first region's >4-GiB case. */
    uint32_t special_low_hole = 0u;
    uint32_t insert_extra = 0u;
    uint64_t extra_start = 0u;
    uint64_t extra_size = 0u;
    if (flag != 0u && initial.count != 0u && initial.region[0].size > UINT64_C(0xffffffff)) {
        if (remap_enabled != 0u) {
            const uint64_t r0 = rank_size[0][0];
            const uint64_t r1 = rank_size[0][1];
            if (r1 == 0u) {
                if (r0 <= UINT64_C(0x100000000)) {
                    extra_start = raw_total + UINT64_C(0xf0000000);
                    if (r0 == UINT64_C(0x100000000) && active_count != 0u)
                        patch_channel_map_byte(context, active_count,
                                               (uint32_t)(r0 / active_count));
                    insert_extra = 1u;
                    extra_size = UINT64_C(0x10000000);
                }
            } else if (r0 <= UINT64_C(0x80000000)) {
                if (topology_mode > 3u) {
                    const uint32_t encoded = (uint32_t)((r1 >> 25) & 0xffu) |
                                             ((uint32_t)(r0 >> 25) << 8);
                    set_channel_map_word(context, active_count, encoded);
                    extra_start = raw_total + UINT64_C(0xf0000000);
                } else {
                    uint64_t scaled = r0;
                    if (qrk3588_ddr_state_channel_row_3_4(aux, 3u) != 0u)
                        scaled = (r0 << 2) / 3u;
                    extra_start = UINT64_C(0xf0000000) +
                                  scaled * (uint64_t)active_count;
                }
                insert_extra = 1u;
                extra_size = UINT64_C(0x10000000);
            }
        } else {
            global[2] = UINT32_C(0x01000000);
            special_low_hole = 1u;
        }
    }

    normalize_tag52_regions(&normalized, &initial, special_low_hole,
                            insert_extra, extra_start, extra_size);
    tag52_store_regions(out, &normalized);
    *(uint32_t *)(out + 0xa8u) |= 2u;
}

/*
 * Build the three tag payloads used by the DDR parameter parser, derive the
 * optional UART descriptor from the selected DRAM config, normalize the
 * per-channel parameter block, then run the version/tag pass.
 */
void sdram_params_init(void)
{
    uint8_t tag50[0x28] __attribute__((aligned(8)));
    uint8_t tag52[0xb8] __attribute__((aligned(8)));
    uint8_t tag58[0x38] __attribute__((aligned(8)));
    const uint8_t *cfg_base = DDR_LAYOUT_CPTR(uint8_t, QRK3588_DDR_TYPE_CONFIG_TABLE_VA);

    ddr_memset(tag50, 0, sizeof(tag50));

    uint32_t cfg_index = cfg_base[0x0au];
    const uint8_t *selected = cfg_base + (uintptr_t)cfg_index * 4u;
    uint32_t descriptor = *(const uint32_t *)selected;
    uint32_t uart_index = descriptor >> 28;

    if (uart_index <= 9u) {
        static const uintptr_t uart_bases[10] = {
            0xfd890000u,
            0xfeb40000u, 0xfeb50000u, 0xfeb60000u, 0xfeb70000u,
            0xfeb80000u, 0xfeb90000u, 0xfeba0000u, 0xfebb0000u,
            0xfebc0000u,
        };
        *(uint32_t *)(tag50 + 0x04u) = 1u;
        *(uintptr_t *)(tag50 + 0x08u) = uart_bases[uart_index];
        *(uint32_t *)(tag50 + 0x10u) = descriptor & 0x00ffffffu;
        *(uint32_t *)(tag50 + 0x14u) = (descriptor >> 24) & 0x0fu;
        *(uint32_t *)(tag50 + 0x18u) = uart_index;
    }

    ddr_sram_workspace_reset_if_tagged();
    (void)config_tag_parse(0x54410050u, tag50);

    ddr_memset(tag52, 0, sizeof(tag52));
    void *context = DDR_LAYOUT_REF(void *, QRK3588_DDR_CONTEXT_SLOT_VA);
    void *aux = DDR_LAYOUT_REF(void *, QRK3588_DDR_ACTIVE_CONFIG_SLOT_VA);
    uint32_t flag = selected[0x0fu] & 1u;
    ddr_topology_region_normalize(context, aux, tag52, flag);
    (void)config_tag_parse(0x54410052u, tag52);

    (void)config_reparse(*(const uint32_t *)(selected + 0x10u));

    ddr_memset(tag58, 0, sizeof(tag58));
    const uint32_t *sdram = DDR_LAYOUT_REF(const uint32_t *, QRK3588_DDR_SDRAM_PARAMS_SLOT_VA);
    for (uint32_t i = 0; i != 4u; ++i)
        *(uint32_t *)(tag58 + 4u + i * 4u) = sdram[i];
    (void)config_tag_parse(0x54410058u, tag58);
    (void)config_check_version();
}

/*
 * Sweep the PHY training code field and capture the first transition point for
 * each of seven status bits.  mode==0 records the first clear sample;
 * mode!=0 records the first set sample.  Results are written to the ROM's
 * per-mode scratch table at QRK3588_DDR_RUNTIME_VA(0x678) + mode*0x1c.
 */
int phy_reg_rw_seq(void *context, uint32_t channel, uint32_t mode,
                              uint8_t *config)
{
    volatile uint32_t *phy = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base1(context, channel);
    uint32_t phy_base = (uint32_t)(uintptr_t)phy;
    uint32_t *reg_cfg = qrk3588_ddr_state_dram_type_ptr(config);

    if (mode == 0u) {
        ddr_phy_reg_read(context, channel, reg_cfg, 0u, 0x64u);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
    }

    phy[0xa20u / 4u] |= 1u;
    phy[4u / 4u] |= 4u;
    phy[0x600u / 4u] = (phy[0x600u / 4u] & 0xf9ffffffu) | 0x02000000u;
    phy[0xadcu / 4u] = (phy[0xadcu / 4u] & 0xfffffff3u) | 4u;
    phy[0xa78u / 4u] = 0x00010100u | (mode << 19);
    phy[0xa68u / 4u] |= 0x1000u;
    udelay(1u);
    phy[0xa68u / 4u] &= ~0x1000u;
    phy[0x600u / 4u] = (phy[0x600u / 4u] & 0xff9fffffu) | 0x00200000u;
    phy[0x608u / 4u] |= 0x20000000u;

    uint32_t frequency = qrk3588_ddr_state_frequency(config) << 1;
    ddr_phy_reg_read(context, channel, reg_cfg, 0u, frequency);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();

    phy[0x608u / 4u] |= 0x08000000u;
    phy[0x608u / 4u] |= 0x80000000u;
    udelay(1u);
    phy[0x608u / 4u] &= 0x7fffffffu;
    phy[0x608u / 4u] |= 0x40000000u;
    phy[0x60cu / 4u] |= 1u;
    ddr_phy_update_toggle(phy_base);

    const uint8_t *timing = (const uint8_t *)qrk3588_ddr_state_dram_params(config);
    uint32_t tap = phy[0xb4u / 4u] & 0x7ffu;
    if (*(const uint32_t *)(timing + 0x60u) == 0x80u)
        tap >>= 1;
    uint32_t tap_hi = tap << 16;
    phy[0x8cu / 4u] = (phy[0x8cu / 4u] & 0xf800ffffu) | tap_hi;
    phy[0x90u / 4u] = tap | tap_hi;
    ddr_phy_update_toggle(phy_base);

    volatile uint32_t *result = (volatile uint32_t *)(uintptr_t)
        DDR_LAYOUT_ADDR(QRK3588_DDR_MODE_SCRATCH_TABLE_VA + (uintptr_t)mode * 0x1cu);
    uint32_t found_mask = 0u;

    for (uint32_t code = 0u; code != 0x2ffu && found_mask != 0x7fu; ++code) {
        uint32_t pair = code | (code << 16);
        phy[0x7cu / 4u] = pair;
        phy[0x80u / 4u] = pair;
        phy[0x84u / 4u] = pair;
        phy[0x8cu / 4u] = (phy[0x8cu / 4u] & 0xfffff800u) | code;
        ddr_phy_update_toggle(phy_base);

        phy[0xa60u / 4u] = 0x10007f00u;
        phy[0xa5cu / 4u] = 1u;
        __asm__ volatile("nop");
        phy[0xa5cu / 4u] = 0u;

        uint32_t status = phy[0x3a0u / 4u];
        for (uint32_t bit = 0u; bit != 7u; ++bit) {
            uint32_t sample = (status >> bit) & 1u;
            if (sample != (mode != 0u ? 1u : 0u))
                continue;
            uint32_t mask = 1u << bit;
            if (found_mask & mask)
                continue;
            result[bit] = code;
            found_mask |= mask;
        }
    }

    phy[0x8cu / 4u] = 0u;
    phy[0x90u / 4u] = 0u;
    ddr_phy_update_toggle(phy_base);
    phy[0x60cu / 4u] &= ~1u;
    ddr_phy_update_toggle(phy_base);
    ddr_phy_reg_read(context, channel, reg_cfg, 0u, 0x64u);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();

    phy[0x608u / 4u] &= 0x9fffffffu;
    udelay(1u);
    phy[0x608u / 4u] &= 0xf7ffffffu;
    phy[0xa78u / 4u] = 0x100u;
    phy[0xa68u / 4u] |= 0x1000u;
    udelay(1u);
    phy[0xa68u / 4u] &= ~0x1000u;
    phy[0x600u / 4u] &= 0xff9fffffu;
    phy[0xadcu / 4u] = (phy[0xadcu / 4u] & 0xfffffff3u) | 4u;
    phy[0x600u / 4u] = (phy[0x600u / 4u] & 0xf9ffffffu) | 0x02000000u;
    phy[0x608u / 4u] |= 0x10000000u;
    udelay(1u);
    phy[0x608u / 4u] &= 0xefffffffu;
    phy[4u / 4u] &= ~4u;
    phy[0xa20u / 4u] &= ~1u;
    return 0;
}

void config_pctl_regs_init(void *context, uint32_t channel,
                                      uint8_t *config, uint32_t profile)
{
    const uint8_t *cfg_base = DDR_LAYOUT_CPTR(uint8_t, QRK3588_DDR_TYPE_CONFIG_TABLE_VA);
    const uint8_t *selected = cfg_base + (uintptr_t)cfg_base[0x0au] * 4u;
    uint8_t *channel_root = (uint8_t *)qrk3588_ddr_ctx_channel_base0(context, channel);
    volatile uint32_t *pctl = (volatile uint32_t *)(channel_root + 0x10000u);
    const uint8_t *dram = (const uint8_t *)qrk3588_ddr_state_dram_params(config);
    const uint8_t *timing = (const uint8_t *)qrk3588_ddr_state_timing(config);
    uint32_t freq_div = qrk3588_ddr_state_frequency(config) / 24u;
    uint32_t selected_bit = ((*(const uint32_t *)(selected + 0x14u) >> 3) & 1u) << 2;

    ddr_pctl_frequency_register_image_build(context, channel, config, 0u);

    if ((profile == 0u ||
         qrk3588_ddr_state_channel_device_width_code(config, 0u) == 0u) &&
        *(const uint32_t *)(dram + 0x28u) != 9u)
        pctl[0x280u / 4u] |= 0x20000000u;

    const struct qrk3588_ddr_channel_geometry *geometry =
        qrk3588_ddr_state_channel_geometry_const(config, channel);
    uint32_t mstr = (geometry->rank_count == 2u) ? 0x03000000u : 0x01000000u;
    if (*(const uint32_t *)(dram + 0x34u) == 0x10u)
        mstr |= 0x00080000u;
    else
        mstr |= 0x00040000u;
    if (geometry->bus_width_code == 0u)
        mstr |= 0x1000u;

    uint32_t dram_type = *(const uint32_t *)(dram + 0x28u);
    if (dram_type == 7u || dram_type == 8u) {
        mstr |= 2u;
    } else if (dram_type == 9u) {
        mstr |= 8u;
    } else {
        ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_DRAM_TYPE_UNSUPPORTED_VA),
                   DDR_LAYOUT_CPTR(char, QRK3588_DDR_DEBUG_PCTL_INIT_NAME_VA), 0xfe5u);
    }
    pctl[0] = mstr;

    if (qrk3588_ddr_ctx_descriptor_state_const(context)->wordc_bit27 != 0u)
        pctl[0x200u / 4u] = (dram_type == 9u) ? 0x10fu : 0x136u;
    else
        pctl[0x200u / 4u] = 0u;

    pctl[0x288u / 4u] = 1u;
    pctl[0x508u / 4u] = 0x80008000u;

    uint32_t sched = (*(const uint32_t *)(dram + 0x44u) == 0u) ? 1u : 5u;
    if (*(const uint32_t *)(dram + 0x48u) != 0u)
        sched |= 2u;
    pctl[0xc94u / 4u] = sched;

    uint32_t t0 = *(const uint32_t *)(timing + 0x14u) / freq_div;
    uint32_t t1 = *(const uint32_t *)(timing + 0x0cu) / freq_div;
    t0 = ((t0 + 0x3ffu) >> 10) + 1u;
    t1 = ((t1 + 0x3ffu) >> 10) + 1u;
    pctl[0xd00u / 4u] = t1 | (t0 << 16);
    uint32_t t2 = *(const uint32_t *)(timing + 0x18u) / freq_div;
    t2 = ((t2 + 0x3ffu) >> 10) + 1u;
    pctl[0xd04u / 4u] = t2 << 16;

    pctl[0x380u / 4u] = (pctl[0x380u / 4u] & ~4u) | selected_bit;
    if (dram_type == 9u) {
        pctl[0x380u / 4u] = (pctl[0x380u / 4u] & 0xffff80ffu) | 0x1f00u;
        pctl[0x384u / 4u] = (pctl[0x384u / 4u] & 0xffff0fffu) | 0x3000u;
        pctl[0x38cu / 4u] = (pctl[0x38cu / 4u] & 0xffff00ffu) | 0x0200u;
        pctl[0x38cu / 4u] = (pctl[0x38cu / 4u] & 0xffffff00u) | 0x10u;
    }

    pctl[0x394u / 4u] &= 0xcfffffffu;
    pctl[0xc90u / 4u] = (pctl[0xc90u / 4u] & 0xff000fffu) | 0xf000u;
    pctl[0x184u / 4u] |= 1u;
    pctl[0x500u / 4u] = (pctl[0x500u / 4u] & 0xffeefeeeu) | 0x00100011u;
    pctl[0x18cu / 4u] = 0u;
    pctl[0x510u / 4u] |= 4u;

    if (qrk3588_ddr_ctx_descriptor_state_const(context)->wordc_bit26 != 0u) {
        pctl[0x104u / 4u] = 1u;
        pctl[0x108u / 4u] = 1u;
        pctl[0x118u / 4u] &= ~1u;
    }
    pctl[0x518u / 4u] |= 1u;

    for (uint32_t i = 0u; i != 4u; ++i) {
        volatile uint32_t *bank = (volatile uint32_t *)(channel_root +
                                                        (uintptr_t)i * 0x100000u);
        bank[0x804u / 4u] = (bank[0x804u / 4u] & 0xfff00000u) | 0x000f0000u;
    }
    pctl[0x510u / 4u] |= 4u;
}

/* Build per-channel topology/rank fields, then program each populated channel. */

void ddr_channel_topology_and_timing_init(void *context,
                                                     uint8_t *config)
{
    uint32_t dram_type = qrk3588_ddr_state_dram_type(config);
    uint32_t topology = dram_type << 27;
    uint32_t lane_map = 0u;
    uint8_t *ctx = (uint8_t *)context;

    /* Four logical channel slots are encoded at context+0xd8..0xe4. */
    for (uint32_t slot = 0u; slot != 4u; ++slot) {
        uint32_t ch = qrk3588_ddr_ctx_route(ctx, slot);
        if (ch == 0x0fu) {
            topology |= 0xffff8000u;
            continue;
        }

        volatile void *phy = (volatile void *)qrk3588_ddr_ctx_channel_base0(ctx, ch);
        uint32_t probe = ddr_phy_config_block(phy, 1u, 8u);
        if (qrk3588_ddr_get_error() != 0)
            return;

        topology |= (qrk3588_ddr_state_channel_device_width_code(config, ch) <<
                     (ch + 12u));
        topology |= ((qrk3588_ddr_state_channel_rank_count(config, ch) - 1u) << ch);
        lane_map |= probe << (ch * 8u);

        uint32_t last = qrk3588_ddr_state_channel_rank_row_bits(config, ch, 1u);
        uint32_t peer = qrk3588_ddr_state_channel_rank_row_bits(config, ch, 0u);
        if (last - 1u == peer)
            topology |= 1u << (ch + 8u);
        if (peer == last + 1u)
            topology |= 1u << (ch + 4u);
    }

    volatile uint32_t *topology_regs = (volatile uint32_t *)qrk3588_ddr_ctx_topology_regs(ctx);
    topology_regs[0x88u / 4u] = topology;
    topology_regs[0x8cu / 4u] = lane_map;

    uint32_t packed[4] = {0u, 0u, 0u, 0u};
    uint32_t active_count = 0u;

    for (uint32_t slot = 0u; slot != 4u; ++slot) {
        uint32_t ch = qrk3588_ddr_ctx_route(ctx, slot);
        if (ch == 0x0fu)
            continue;
        ++active_count;

        const struct qrk3588_ddr_channel_geometry *geometry =
            qrk3588_ddr_state_channel_geometry_const(config, ch);
        if (geometry->column_bits == 0u)
            continue;

        volatile uint32_t *chan_aux = (volatile uint32_t *)
            qrk3588_ddr_ctx_channel_output(ctx, ch);
        chan_aux[2] = qrk3588_ddr_state_channel_frequency_profile_selector(config, ch);

        const uint32_t *global_timing = qrk3588_ddr_state_frequency_ptr(config);
        if (ch <= 1u) {
            ddr_program_rank_geometry_fields(geometry, global_timing,
                                             &packed[0], &packed[1], ch);
            volatile uint32_t *dst = (volatile uint32_t *)qrk3588_ddr_ctx_geometry_regs(ctx);
            dst[0x208u / 4u] = packed[0];
            dst[0x20cu / 4u] = packed[1];
            dst[0x210u / 4u] = 0u;
            dst[0x214u / 4u] = 0u;
        } else {
            ddr_program_rank_geometry_fields(geometry, global_timing,
                                             &packed[2], &packed[3], ch);
            volatile uint32_t *dst = (volatile uint32_t *)qrk3588_ddr_ctx_geometry_regs(ctx);
            dst[0x210u / 4u] = packed[2];
            dst[0x214u / 4u] = packed[3];
        }

        uint64_t v0 = ddr_geometry_capacity_component(geometry, 0u, dram_type);
        uint64_t v1 = ddr_geometry_capacity_component(geometry, 1u, dram_type);
        if (qrk3588_ddr_state_channel_row_3_4(config, ch) != 0u) {
            v0 = (v0 * 3u) >> 2;
            v1 = (v1 * 3u) >> 2;
        }

        if (qrk3588_ddr_state_channel_rank_count(config, ch) == 1u) {
            if (v0 == 0x200000000ull) {
                v0 = 0x100000000ull;
                v1 = 0x100000000ull;
            } else {
                v0 = 0x100000000ull;
                v1 = 0u;
            }
        } else if (qrk3588_ddr_state_channel_frequency_profile_selector(config, ch) <= 3u) {
            v0 = 0x100000000ull;
        }

        chan_aux[3] = (uint32_t)(((v1 >> 26) & 0xffu) << 8) |
                      (uint32_t)((v0 >> 26) & 0xffu);

        ddr_channel_timing_pack(context, ch, config);
        uint32_t phy_base = (uint32_t)qrk3588_ddr_ctx_channel_base0(ctx, ch);
        ddr_phy_timing_program(
            phy_base, 0u,
            geometry, dram_type, qrk3588_ddr_state_frequency(config));
        if (qrk3588_ddr_get_error() != 0)
            return;

        if (qrk3588_ddr_ctx_descriptor_state_const(ctx)->wordc_bit26 != 0u) {
            volatile uint32_t *pctl = (volatile uint32_t *)(uintptr_t)
                (phy_base + 0x10000u);
            pctl[0x100u / 4u] |= 1u;
        }
        ddr_phy_channel_restore_and_program(context, ch, config);
    }

    if (active_count == 2u) {
        volatile uint32_t *reg = (volatile uint32_t *)qrk3588_ddr_ctx_address_map_regs(ctx);
        reg[0x10u / 4u] = 0x80000000u;
    }
}



/* Build the DRAM-type/frequency parameter pair consumed by the PHY/PCTL code. */
int ddr_build_frequency_params(uint8_t *config,
                                          uint8_t *dram_params,
                                          uint8_t *timing,
                                          uint32_t frequency,
                                          uint32_t dram_type,
                                          uint32_t preserve_geometry)
{
    const uint8_t *preset;
    if (dram_type == 7u || dram_type == 8u)
        preset = qrk3588_lp4_2112_preset();
    else if (dram_type == 9u)
        preset = qrk3588_lp5_2400_preset();
    else {
        dbg_puts(DDR_LAYOUT_CPTR(char, QRK3588_DDR_ERR_DRAM_TYPE_UNSUPPORTED_VA));
        return -1;
    }

    *(uint32_t *)(preset == qrk3588_lp4_2112_preset()
                      ? DDR_LAYOUT_ADDR(QRK3588_DDR_PARAMETER_VA(0x814u))
                      : DDR_LAYOUT_ADDR(QRK3588_DDR_PARAMETER_VA(0x854u))) = dram_type;
    /* The preset word is frequency-sensitive. */
    *(volatile uint32_t *)(uintptr_t)(preset + 0x38u) = frequency;

    if (!preserve_geometry) {
        const uint32_t p0 = *(const uint32_t *)(preset + 0x00u);
        const uint32_t p1 = *(const uint32_t *)(preset + 0x04u);
        const uint32_t p2 = *(const uint32_t *)(preset + 0x08u);
        const uint32_t p3 = *(const uint32_t *)(preset + 0x0cu);
        const uint32_t p4 = *(const uint32_t *)(preset + 0x10u);
        const uint32_t p5 = *(const uint32_t *)(preset + 0x14u);
        const uint32_t p6 = *(const uint32_t *)(preset + 0x18u);
        const uint32_t p7 = *(const uint32_t *)(preset + 0x1cu);
        const uint32_t p8 = *(const uint32_t *)(preset + 0x20u);
        const uint32_t p9 = *(const uint32_t *)(preset + 0x24u);
        uint32_t rank_bits = (p6 == 0x10u) ? 4u : ((p6 >> 2) + 1u);

        for (uint32_t i = 0u; i != 4u; ++i) {
            struct qrk3588_ddr_channel_geometry *geometry =
                qrk3588_ddr_state_channel_geometry(config, i);

            geometry->rank_count = p1;
            geometry->column_bits = p7;
            geometry->bank_bits = rank_bits;
            geometry->bus_width_code = p0 >> 4;
            geometry->device_width_code = p8 >> 4;
            geometry->row_3_4 = p9;
            geometry->cs_row_bits[0] = p2;
            geometry->cs_row_bits[1] = p3;
            geometry->cs_row_bits[2] = 0u;
            geometry->cs_row_bits[3] = 0u;
            geometry->cs_high16_row_bits[0] = p4;
            geometry->cs_high16_row_bits[1] = p5;
            geometry->cs_high16_row_bits[2] = 0u;
            geometry->cs_high16_row_bits[3] = 0u;
            qrk3588_ddr_state_set_channel_frequency_profile_selector(config, i, 0u);
        }
        qrk3588_ddr_state_set_dram_type(config, *(const uint32_t *)(preset + 0x2cu));
        qrk3588_ddr_state_set_frequency(config, frequency);
        {
            struct qrk3588_ddr_global_state *global = qrk3588_ddr_state_global(config);
            global->private_1b8 = 4u;
            global->grf_config_result = 0u;
            global->private_1c0 = 0u;
        }
    } else {
        qrk3588_ddr_state_set_frequency(config, frequency);
    }

    qrk3588_ddr_state_set_dram_params(config, dram_params);
    qrk3588_ddr_state_set_timing(config, timing);

    ddr_memset(dram_params, 0, 0x94u);
    ddr_preset_to_dram_params((const uint32_t *)preset, (uint32_t *)dram_params, 0u);

    const uint8_t *type_cfg = (const uint8_t *)dram_type_cfg_ptr(dram_type);
    const uint8_t *freq_cfg = (const uint8_t *)freq_cfg_ptr(dram_type, frequency);

    if (type_cfg != 0) {
        *(uint32_t *)(dram_params + 0x4cu) =
            freq_cfg[2] ? type_cfg[0x13] : type_cfg[0x0f];

        if (frequency > (*(const uint32_t *)(type_cfg + 0x18u) & 0xfffu)) {
            *(uint32_t *)(dram_params + 0x40u) = 1u;
            *(uint32_t *)(dram_params + 0x50u) = type_cfg[0x14];
            *(uint32_t *)(dram_params + 0x58u) =
                (*(uint32_t *)config == 1u) ? 0u : type_cfg[0x2b];
        } else {
            *(uint32_t *)(dram_params + 0x40u) = 0u;
            *(uint32_t *)(dram_params + 0x50u) = 0u;
            *(uint32_t *)(dram_params + 0x58u) = 0u;
        }

        if (frequency > (*(const uint32_t *)(type_cfg + 0x24u) & 0xfffu))
            *(uint32_t *)(dram_params + 0x54u) =
                (*(const uint32_t *)(type_cfg + 0x14u) >> 18) & 0xffu;
        else
            *(uint32_t *)(dram_params + 0x54u) = 0u;
    }

    *(uint32_t *)(dram_params + 0x78u) = freq_cfg[9];

    if (dram_type == 9u) {
        *(uint32_t *)(dram_params + 0x7cu) = 0u;
        *(uint32_t *)(dram_params + 0x80u) = freq_cfg[0x13];
        *(uint32_t *)(dram_params + 0x84u) = freq_cfg[0x14];
        *(uint32_t *)(dram_params + 0x88u) = 0u;
        *(uint32_t *)(dram_params + 0x90u) = 0u;
        *(uint32_t *)(dram_params + 0x74u) = 2u;
        *(uint32_t *)(dram_params + 0x8cu) = 2u;

        uint32_t doubled = frequency << 1;
        const uint32_t *tbl;
        uint32_t count;
        if (frequency <= 0x640u) {
            *(uint32_t *)(dram_params + 0x60u) = 0x80u;
            *(uint32_t *)(dram_params + 0x64u) = 0x10u;
            tbl = (const uint32_t *)qrk3588_lp5_training_low();
            count = 6u;
        } else {
            *(uint32_t *)(dram_params + 0x60u) = 0u;
            *(uint32_t *)(dram_params + 0x64u) = 0u;
            tbl = (const uint32_t *)qrk3588_lp5_training_high();
            count = 10u;
        }
        for (uint32_t i = 0u; i != count; ++i) {
            const uint32_t *e = (const uint32_t *)((const uint8_t *)tbl + i * 0x18u);
            if (doubled <= e[0]) {
                if (e[1] == 0u)
                    *(uint32_t *)(dram_params + 0x70u) = 1u;
                break;
            }
        }
    } else {
        *(uint32_t *)(dram_params + 0x80u) = freq_cfg[0x13];
        *(uint32_t *)(dram_params + 0x8cu) = 0u;
        *(uint32_t *)(dram_params + 0x88u) = 1u;
    }

    ddr_memset(timing, 0, 0x2acu);
    qrk3588_ddr_set_timing_work_flag(0u);
    int rc = ddr_timing_parameter_synthesize(dram_params, timing);
    if (rc != 0)
        return rc;

    if (*(uint32_t *)(dram_params + 0x28u) == 9u) {
        uint32_t base = (frequency < 0x641u) ? (frequency >> 1) : (frequency >> 2);
        *(uint32_t *)(timing + 0x84u) = *(uint32_t *)(timing + 0x244u);
        *(uint32_t *)(timing + 0x88u) = *(uint32_t *)(timing + 0x17cu);
        *(uint32_t *)(timing + 0x8cu) = *(uint32_t *)(timing + 0x24cu);
        *(uint32_t *)(timing + 0x90u) = *(uint32_t *)(timing + 0x250u);
        *(uint32_t *)(timing + 0x98u) = 0u;
        *(uint32_t *)(timing + 0x1dcu) = 0u;
        *(uint32_t *)(timing + 0x180u) =
            (*(uint32_t *)(timing + 0x248u) >= *(uint32_t *)(timing + 0x234u))
                ? *(uint32_t *)(timing + 0x248u)
                : *(uint32_t *)(timing + 0x234u);
        *(uint32_t *)(timing + 0x1e0u) = 0u;
        uint32_t tmp = base * 3u + (base >> 1);
        *(uint32_t *)(timing + 0x6cu) = (tmp + 0x3e7u) / 0x3e8u;
        tmp = base * 14u;
        tmp = (tmp + 0x3e7u) / 0x3e8u;
        if (tmp < 10u)
            tmp = 10u;
        *(uint32_t *)(timing + 0x194u) = tmp;
    } else {
        const uint32_t offs[] = {0x1c4u, 0x78u, 0x188u, 0x190u, 0x84u};
        for (uint32_t i = 0; i != sizeof(offs)/sizeof(offs[0]); ++i)
            *(uint32_t *)(timing + offs[i]) += 2u;
    }
    return 0;
}

/*
 * Search one training axis and return the stable [low, high] window.
 * The search uses a compact two-direction state machine.  Keep control-flow
 * joins explicit so signed boundary tests and retry transitions remain easy
 * to audit.
 */
int train_channels_loop(void *context, uint32_t selector,
                                   int32_t mode, uint32_t direct,
                                   uint32_t *result, uint32_t channel)
{
    uint8_t *ctx = (uint8_t *)context;
    int32_t win_low = 0, win_high = 0;
    uint32_t w19, w22, w25, w24, w9, w8, w13;
    int32_t w26, w27, w28, w0s;
    uint32_t w15;
    uintptr_t x10;
    uint32_t special, baseline_failed;

    if (mode == -1)
        w19 = 0u;
    else
        w19 = ~(1u << ((uint32_t)mode & 31u));

    (void)ddr_training_window(&win_low, &win_high, selector, mode,
                              direct, channel);
    (void)ddr_training_adjust(0, selector, mode, direct, channel);

    w22 = (uint32_t)io_config_dispatch(
        *(uint64_t *)(ctx + 0x00u), *(uint32_t *)(ctx + 0x08u), 1u,
        *(uint32_t *)(ctx + 0x0cu), w19, (uint32_t)mode);

    special = (mode == -1 && selector == 0u);
    if (special) {
        for (uint32_t off = 0x10u; off != 0x28u; off += 8u) {
            uint64_t base = *(uint64_t *)(ctx + off);
            if (base != 0u && base != *(uint64_t *)(ctx + 0x00u)) {
                w22 |= (uint32_t)io_config_dispatch(
                    base, *(uint32_t *)(ctx + 0x08u), 1u,
                    *(uint32_t *)(ctx + 0x0cu), w19, 0xffffffffu);
            }
        }
    }

    baseline_failed = (w22 != 0u);
    w15 = 1u;
    x10 = 0u;
    w28 = 0;
    w27 = 0;

L_ee34:
    w25 = ((uint32_t)x10 ^ 1u) & 1u;
    w24 = ((w15 & 1u) ^ 1u) & 1u;
    w26 = w28;
    w9 = 0u;
    w28 = 0;
    w8 = 1u;

L_ee54:
    if (w8 == 0u)
        goto L_ef9c;

L_ee58:
    w0s = w28 + (x10 ? 8 : -8);

L_ee64:
    if (w22 != 0u) {
        uint32_t gt = (w0s > 0) ? 1u : 0u;
        if ((gt & w25) != 0u)
            goto L_f05c;
        if ((w24 & ((uint32_t)w0s >> 31)) != 0u)
            goto L_f05c;
    }

    w28 = win_low;
    if (w0s >= w28)
        goto L_efcc;
    if (w8 == 0u)
        goto L_efb8;

L_ee90:
    if ((baseline_failed & (w9 ^ 1u)) != 0u)
        goto L_f05c;
    w8 = 0u;

L_eea4:
    (void)ddr_training_adjust(w28, selector, mode, direct, channel);
    w13 = (uint32_t)io_config_dispatch(
        *(uint64_t *)(ctx + 0x00u), *(uint32_t *)(ctx + 0x08u), 1u,
        *(uint32_t *)(ctx + 0x0cu), w19, (uint32_t)mode);

    if (special) {
        for (uint32_t off = 0x10u; off != 0x28u; off += 8u) {
            uint64_t base = *(uint64_t *)(ctx + off);
            if (base != 0u && base != *(uint64_t *)(ctx + 0x00u)) {
                w13 |= (uint32_t)io_config_dispatch(
                    base, *(uint32_t *)(ctx + 0x08u), 1u,
                    *(uint32_t *)(ctx + 0x0cu), w19, 0xffffffffu);
            }
        }
    }

    if (w22 != 0u)
        goto L_eff4;

    /* 0xef54: collapse the probe bit only on a failed sample. */
    if (w8 != 0u && w13 != 0u)
        w8 = 0u;
    if (w13 != 0u || w8 != 0u)
        goto L_ee54;
    if (x10 != 0u)
        goto L_f068;
    w27 = w28;
    w28 = w26;
    goto L_f068;

L_ef94:
    w9 = 0u;
L_ef98:
    w8 = 0u;
L_ef9c:
    w0s = x10 ? (w28 - 1) : (w28 + 1);
    goto L_ee64;

L_efa8:
    w9 = 1u;
    goto L_ef98;

L_efb8:
    if (x10 == 0u)
        goto L_f0d4;
    w28 = w26;
L_efc0:
    if (w22 == 0u)
        goto L_f068;
    w26 = w28;
    goto L_f05c;

L_efcc:
    w28 = win_high;
    if (w0s <= w28) {
        w28 = w0s;
        goto L_eea4;
    }
    if (w8 != 0u)
        goto L_ee90;
    if (x10 != 0u)
        goto L_efc0;

L_efe0:
    if (w22 != 0u)
        goto L_f05c;
    w28 = w26;
    goto L_f070;

L_eff4:
    if (w8 != 0u) {
        if ((w9 | w13) == 0u)
            goto L_ef94;
        if (w13 != 0u && w9 != 0u)
            goto L_efa8;
    }

    {
        uint32_t a = (w8 ^ 1u) & 1u;
        uint32_t b = w9 ^ 1u;
        if ((a & b) != 0u && w13 != 0u) {
            if (x10 == 0u)
                w26 = w28 - 1;
            else
                w27 = w28 + 1;
            w9 = 1u;
            w8 = 1u;
            goto L_ee58;
        }
        if ((a & w9) == 0u)
            goto L_ee54;
        if (w13 != 0u)
            goto L_efa8;
        if (x10 == 0u) {
            /* csel w26,w26,w28,eq ; csel w27,w28,w27,eq */
            w27 = w28;
        } else {
            w26 = w28;
        }
    }

L_f05c:
    w28 = w27 | w26;
    if (w28 != 0) {
        w28 = w26;
        goto L_f080;
    }
    w27 = 0;

L_f068:
    if (w15 == 2u)
        goto L_f080;

L_f070:
    ++x10;
    ++w15;
    goto L_ee34;

L_f080:
    (void)ddr_training_adjust(0, selector, mode, direct, channel);
    if ((w27 | w28) != 0 && w27 < w28) {
        result[1] = (uint32_t)w27;
        result[2] = (uint32_t)w28;
        return 0;
    }
    return -1;

L_f0d4:
    w27 = w28;
    goto L_efe0;
}

static inline uint32_t rk_lsl32(uint32_t value, uint32_t shift)
{
    return value << (shift & 31u);
}

/* Configure the RK3588 DDR hash/stride fields in PMU-GRF/DDR-GRF. */
int grf_ddr_config(void *context, uint8_t *config,
                              const uint32_t hash_template[7],
                              uint32_t profile)
{
    uint8_t *ctx = (uint8_t *)context;
    uint32_t active_plus_one = 1u;
    uint32_t global0 = qrk3588_ddr_state_channel_rank_row_bits(config, 0u, 0u);
    uint32_t global1 = qrk3588_ddr_state_channel_rank_row_bits(config, 0u, 1u);
    uint32_t stride = qrk3588_ddr_state_channel_frequency_profile_selector(config, 0u);

    /* Channel 0 supplies the reference geometry.  The firmware compares only
     * populated channels 1..3 against it; context route slot 0 is therefore
     * intentionally not part of this validation loop. */
    for (uint32_t channel = 1u; channel != QRK3588_DDR_CHANNEL_COUNT; ++channel) {
        uint32_t route = qrk3588_ddr_ctx_route(ctx, channel);
        if (route == QRK3588_DDR_CHANNEL_UNUSED)
            continue;
        ++active_plus_one;
        if (qrk3588_ddr_state_channel_frequency_profile_selector(config, channel) != stride ||
            qrk3588_ddr_state_channel_rank_row_bits(config, channel, 0u) != global0 ||
            qrk3588_ddr_state_channel_rank_row_bits(config, channel, 1u) != global1) {
            profile = 0xffu;
            break;
        }
    }

    uint32_t preserve_mask;
    if (stride == 0u)
        preserve_mask = 0x0ffffc00u;
    else if (stride > 4u)
        preserve_mask = 0x1ffff000u;
    else
        preserve_mask = 0x0ffff800u;

    /* With no populated channel the profile is deliberately forced invalid. */
    if (active_plus_one == 1u)
        profile = 0xffu;

    volatile uint32_t *grf = (volatile uint32_t *)qrk3588_ddr_ctx_grf_regs(ctx);
    volatile uint32_t *mirror = (volatile uint32_t *)qrk3588_ddr_ctx_grf_mirror_regs(ctx);
    uint32_t stride_class = stride & ~4u;

    if (profile > 5u)
        goto fallback_stride_mode;

    /* Profile 5 is only retained for stride classes 2/6; otherwise use 4. */
    if (profile == 5u && stride != 2u && stride != 6u)
        profile = 4u;

    uint32_t upper_width = (stride_class == 2u) ? 5u : 4u;
    uint32_t gap = upper_width - profile;
    if (gap > 5u)
        gap = 5u;

    uint32_t base_profile = profile;
    if (active_plus_one == 2u)
        profile += 8u;

    uint32_t ddr_cfg = (stride << 8) | 1u |
                       rk_lsl32(profile, 4u) |
                       rk_lsl32(gap, 12u);
    grf[0x300u / 4u] = ddr_cfg;
    mirror[0x240u / 4u] = ddr_cfg;

    /* The following seven registers describe channel/bank/rank hash masks. */
    uint32_t shift = base_profile;
    grf[0x314u / 4u] = (hash_template[0] & preserve_mask) |
                       rk_lsl32(8u, shift);

    uint32_t extra = 0u;
    if (active_plus_one == 2u) {
        grf[0x318u / 4u] = 0u;
    } else {
        grf[0x318u / 4u] = (hash_template[1] & preserve_mask) |
                           rk_lsl32(0x10u, shift);
        extra = 1u;
    }

    shift += gap;
    if (stride_class == 2u) {
        grf[0x304u / 4u] = 0u;
        shift += extra;
        grf[0x308u / 4u] = (hash_template[3] & preserve_mask) |
                           rk_lsl32(0x10u, shift);
        grf[0x30cu / 4u] = (hash_template[4] & preserve_mask) |
                           rk_lsl32(0x20u, shift);
        grf[0x310u / 4u] = (hash_template[5] & preserve_mask) |
                           rk_lsl32(0x40u, shift);
        if (stride == 2u)
            grf[0x31cu / 4u] = 0u;
        else
            grf[0x31cu / 4u] = (hash_template[6] & preserve_mask) |
                               rk_lsl32(0x80u, shift);
    } else {
        shift += extra;
        grf[0x304u / 4u] = (hash_template[2] & preserve_mask) |
                           rk_lsl32(0x10u, shift);
        grf[0x308u / 4u] = (hash_template[3] & preserve_mask) |
                           rk_lsl32(0x20u, shift);
        grf[0x30cu / 4u] = (hash_template[4] & preserve_mask) |
                           rk_lsl32(0x40u, shift);
        grf[0x310u / 4u] = (stride_class == 0u) ? 0u :
                           ((hash_template[5] & preserve_mask) |
                            rk_lsl32(0x80u, shift));
        if (stride <= 3u) {
            grf[0x31cu / 4u] = 0u;
        } else {
            uint32_t bit = (stride == 4u) ? 0x80u : 0x100u;
            grf[0x31cu / 4u] = (hash_template[6] & preserve_mask) |
                               rk_lsl32(bit, shift);
        }
    }

    mirror[0x244u / 4u] = grf[0x304u / 4u];
    mirror[0x248u / 4u] = grf[0x308u / 4u];
    mirror[0x24cu / 4u] = grf[0x30cu / 4u];
    mirror[0x250u / 4u] = grf[0x310u / 4u];
    mirror[0x254u / 4u] = grf[0x314u / 4u];
    mirror[0x258u / 4u] = grf[0x318u / 4u];
    mirror[0x25cu / 4u] = grf[0x31cu / 4u];

    ((volatile uint32_t *)qrk3588_ddr_ctx_address_map_regs(ctx))[0x10u / 4u] = 0x01f00000u;
    *(volatile uint32_t *)(uintptr_t)0xfd59800cu = 0x001f0000u;
    goto report;

fallback_stride_mode:
    grf[0x300u / 4u] = 0u;
    mirror[0x240u / 4u] = 0u;

    uint64_t encoded;
    if (qrk3588_ddr_state_channel_rank_count(config, 0u) == 2u && stride <= 3u) {
        encoded = 0x200000000ull;
    } else {
        encoded = ddr_geometry_capacity_component((const struct qrk3588_ddr_channel_geometry *)(const void *)config, 0x0fu,
                                   qrk3588_ddr_state_dram_type(config));
        if (qrk3588_ddr_state_channel_row_3_4(config, 0u) != 0u)
            encoded = (encoded * 3u) >> 2;
    }

    uint32_t code;
    switch ((uint32_t)(encoded >> 20)) {
    case 0x100u:  code = 7u; break;
    case 0x200u:  code = 4u; break;
    case 0x400u:  code = 3u; break;
    case 0x600u:  code = 8u; break;
    case 0x800u:  code = 2u; break;
    case 0xc00u:  code = 6u; break;
    case 0x1000u: code = 1u; break;
    case 0x1800u: code = 5u; break;
    case 0x2000u: code = 0u; break;
    default:
        dbg_puts(DDR_LAYOUT_CPTR(char, QRK3588_DDR_ERR_GRF_SIZE_UNSUPPORTED_VA));
        code = 0u;
        break;
    }
    ((volatile uint32_t *)qrk3588_ddr_ctx_address_map_regs(ctx))[0x10u / 4u] =
        0x01f00000u | (code << 4);
    *(volatile uint32_t *)(uintptr_t)0xfd59800cu = 0x001f0000u | code;

report:
    ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_GRF_STRIDE_CONFIG_VA),
               (grf[0x300u / 4u] >> 4) & 0x0fu,
               (grf[0x300u / 4u] >> 8) & 0x0fu);
    ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_GRF_HASH_VA),
               grf[0x314u / 4u], grf[0x318u / 4u],
               grf[0x304u / 4u], grf[0x308u / 4u],
               grf[0x30cu / 4u], grf[0x310u / 4u],
               grf[0x31cu / 4u]);
    return 0;
}
