/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_DDR_STATE_H
#define QRK3588_DDR_STATE_H

#include <stddef.h>
#include <stdint.h>

/* Shared DDR state/image layout.  Four channel blocks precede global fields. */
#define QRK3588_DDR_STATE_CHANNEL_STRIDE      0x6cu
#define QRK3588_DDR_STATE_FREQUENCY_OFFSET   0x1b0u
#define QRK3588_DDR_STATE_DRAM_TYPE_OFFSET   0x1b4u
#define QRK3588_DDR_STATE_DRAM_PTR_OFFSET    0x228u
#define QRK3588_DDR_STATE_TIMING_PTR_OFFSET  0x230u

_Static_assert(4u * QRK3588_DDR_STATE_CHANNEL_STRIDE == QRK3588_DDR_STATE_FREQUENCY_OFFSET,
               "four channel blocks must end at frequency field");

/* Proven per-channel geometry prefix.  These ten words are consumed by the
 * capacity model and DDR-GRF geometry packer. */
struct qrk3588_ddr_channel_geometry {
    uint32_t rank_count;
    uint32_t column_bits;
    uint32_t bank_bits;
    uint32_t bus_width_code;
    uint32_t device_width_code;
    uint32_t row_3_4;
    uint32_t cs_row_bits[4];
    uint32_t cs_high16_row_bits[4];
};

_Static_assert(offsetof(struct qrk3588_ddr_channel_geometry, rank_count) == 0x00u,
               "rank_count offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_geometry, column_bits) == 0x04u,
               "column_bits offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_geometry, bank_bits) == 0x08u,
               "bank_bits offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_geometry, bus_width_code) == 0x0cu,
               "bus_width_code offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_geometry, device_width_code) == 0x10u,
               "device_width_code offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_geometry, row_3_4) == 0x14u,
               "row_3_4 offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_geometry, cs_row_bits) == 0x18u,
               "cs row offsets");
_Static_assert(offsetof(struct qrk3588_ddr_channel_geometry, cs_high16_row_bits) == 0x28u,
               "high16 row offsets");
_Static_assert(sizeof(struct qrk3588_ddr_channel_geometry) == 0x38u,
               "geometry block size");

#define QRK3588_DDR_STATE_PROFILE_SELECTOR_OFFSET  0x38u
#define QRK3588_DDR_STATE_PACKED_PROFILE_OFFSET    0x3cu
#define QRK3588_DDR_STATE_PACKED_PROFILE_WORDS     6u

/* Six packed controller/PHY timing words produced per channel.  The individual
 * bitfields are hardware-private, so the type deliberately names the stable
 * word layout without inventing unsupported field names. */
struct qrk3588_ddr_channel_packed_profile {
    uint32_t word[QRK3588_DDR_STATE_PACKED_PROFILE_WORDS];
};

_Static_assert(sizeof(struct qrk3588_ddr_channel_packed_profile) == 0x18u,
               "packed profile size");
_Static_assert(QRK3588_DDR_STATE_PACKED_PROFILE_OFFSET +
               sizeof(struct qrk3588_ddr_channel_packed_profile) == 0x54u,
               "packed profile end offset");

/* Complete 0x6c per-channel state row.  The final six words are deliberately
 * opaque because the hardware does not expose individual field definitions. */
struct qrk3588_ddr_channel_state {
    struct qrk3588_ddr_channel_geometry geometry;
    uint32_t frequency_profile_selector;
    struct qrk3588_ddr_channel_packed_profile packed_profile;
    uint32_t private_tail[6];
};

_Static_assert(offsetof(struct qrk3588_ddr_channel_state, geometry) == 0x00u,
               "channel geometry offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_state,
                        frequency_profile_selector) == 0x38u,
               "channel profile selector offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_state, packed_profile) == 0x3cu,
               "channel packed profile offset");
_Static_assert(offsetof(struct qrk3588_ddr_channel_state, private_tail) == 0x54u,
               "channel opaque tail offset");
_Static_assert(sizeof(struct qrk3588_ddr_channel_state) ==
               QRK3588_DDR_STATE_CHANNEL_STRIDE,
               "channel state row size");

#define QRK3588_DDR_DVFS_SNAPSHOT_TAIL_WORDS 5u

/* Per-channel DVFS snapshot ABI.  Frozen 075d8 writes six packed-profile
 * words followed by the first five opaque tail words into a 0x30-byte slot;
 * the final word at +0x2c is intentionally left untouched. */
struct qrk3588_ddr_dvfs_channel_snapshot {
    uint32_t packed_profile[QRK3588_DDR_STATE_PACKED_PROFILE_WORDS];
    uint32_t opaque_tail[QRK3588_DDR_DVFS_SNAPSHOT_TAIL_WORDS];
    uint32_t reserved_2c;
};

_Static_assert(offsetof(struct qrk3588_ddr_dvfs_channel_snapshot,
                        packed_profile) == 0x00u,
               "DVFS snapshot packed-profile offset");
_Static_assert(offsetof(struct qrk3588_ddr_dvfs_channel_snapshot,
                        opaque_tail) == 0x18u,
               "DVFS snapshot opaque-tail offset");
_Static_assert(offsetof(struct qrk3588_ddr_dvfs_channel_snapshot,
                        reserved_2c) == 0x2cu,
               "DVFS snapshot reserved offset");
_Static_assert(sizeof(struct qrk3588_ddr_dvfs_channel_snapshot) == 0x30u,
               "DVFS snapshot channel slot size");

static inline uint8_t *qrk3588_ddr_state_channel(void *state, uint32_t channel)
{
    return (uint8_t *)state + (size_t)channel * QRK3588_DDR_STATE_CHANNEL_STRIDE;
}

static inline const uint8_t *qrk3588_ddr_state_channel_const(const void *state,
                                                             uint32_t channel)
{
    return (const uint8_t *)state + (size_t)channel * QRK3588_DDR_STATE_CHANNEL_STRIDE;
}

static inline struct qrk3588_ddr_channel_state *qrk3588_ddr_state_channel_view(
    void *state, uint32_t channel)
{
    return (struct qrk3588_ddr_channel_state *)(void *)
        qrk3588_ddr_state_channel(state, channel);
}

static inline const struct qrk3588_ddr_channel_state *
qrk3588_ddr_state_channel_view_const(const void *state, uint32_t channel)
{
    return (const struct qrk3588_ddr_channel_state *)(const void *)
        qrk3588_ddr_state_channel_const(state, channel);
}

/* Global state begins immediately after the four 0x6c channel rows.  Fields
 * with repeatedly demonstrated behavior are typed below; three words whose
 * storage role is stable but whose unique hardware meaning is not established
 * remain explicitly private instead of receiving speculative names. */
struct qrk3588_ddr_phy_divider_state {
    uint32_t lane2;
    uint32_t lane1;
    uint32_t lane0;
    uint32_t delta;
};

struct qrk3588_ddr_lp5_channel_image {
    uint8_t byte[20];
};

struct qrk3588_ddr_global_state {
    uint32_t frequency;
    uint32_t dram_type;
    uint32_t private_1b8;
    uint32_t grf_config_result;
    uint32_t private_1c0;
    struct qrk3588_ddr_phy_divider_state phy_dividers;
    struct qrk3588_ddr_lp5_channel_image lp5_channel[4];
    uint32_t reserved_224;
    uint64_t dram_params;
    uint64_t timing;
};

_Static_assert(offsetof(struct qrk3588_ddr_global_state, frequency) == 0x00u,
               "global frequency offset");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, dram_type) == 0x04u,
               "global dram_type offset");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, private_1b8) == 0x08u,
               "global private_1b8 offset");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, grf_config_result) == 0x0cu,
               "global grf_config_result offset");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, private_1c0) == 0x10u,
               "global private_1c0 offset");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, phy_dividers) == 0x14u,
               "global divider offset");
_Static_assert(sizeof(struct qrk3588_ddr_phy_divider_state) == 0x10u,
               "divider state size");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, lp5_channel) == 0x24u,
               "global LP5 image offset");
_Static_assert(sizeof(struct qrk3588_ddr_lp5_channel_image) == 0x14u,
               "LP5 channel image size");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, reserved_224) == 0x74u,
               "global reserved_224 offset");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, dram_params) == 0x78u,
               "global dram pointer offset");
_Static_assert(offsetof(struct qrk3588_ddr_global_state, timing) == 0x80u,
               "global timing pointer offset");
_Static_assert(sizeof(struct qrk3588_ddr_global_state) == 0x88u,
               "global state size");

static inline struct qrk3588_ddr_global_state *qrk3588_ddr_state_global(void *state)
{
    return (struct qrk3588_ddr_global_state *)((uint8_t *)state +
                                              QRK3588_DDR_STATE_FREQUENCY_OFFSET);
}

static inline const struct qrk3588_ddr_global_state *
qrk3588_ddr_state_global_const(const void *state)
{
    return (const struct qrk3588_ddr_global_state *)((const uint8_t *)state +
                                                     QRK3588_DDR_STATE_FREQUENCY_OFFSET);
}

static inline uint32_t qrk3588_ddr_state_grf_config_result(const void *state)
{
    return qrk3588_ddr_state_global_const(state)->grf_config_result;
}

static inline void qrk3588_ddr_state_set_grf_config_result(void *state, uint32_t result)
{
    qrk3588_ddr_state_global(state)->grf_config_result = result;
}

static inline struct qrk3588_ddr_phy_divider_state *
qrk3588_ddr_state_phy_dividers(void *state)
{
    return &qrk3588_ddr_state_global(state)->phy_dividers;
}

static inline const struct qrk3588_ddr_phy_divider_state *
qrk3588_ddr_state_phy_dividers_const(const void *state)
{
    return &qrk3588_ddr_state_global_const(state)->phy_dividers;
}

static inline struct qrk3588_ddr_lp5_channel_image *
qrk3588_ddr_state_lp5_channel_image(void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_global(state)->lp5_channel[channel] : NULL;
}

static inline const struct qrk3588_ddr_lp5_channel_image *
qrk3588_ddr_state_lp5_channel_image_const(const void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_global_const(state)->lp5_channel[channel] : NULL;
}

static inline uint32_t qrk3588_ddr_state_frequency(const void *state)
{
    return qrk3588_ddr_state_global_const(state)->frequency;
}

static inline void qrk3588_ddr_state_set_frequency(void *state, uint32_t frequency)
{
    qrk3588_ddr_state_global(state)->frequency = frequency;
}

static inline const uint32_t *qrk3588_ddr_state_frequency_ptr(const void *state)
{
    return &qrk3588_ddr_state_global_const(state)->frequency;
}

static inline uint32_t qrk3588_ddr_state_dram_type(const void *state)
{
    return qrk3588_ddr_state_global_const(state)->dram_type;
}

static inline void qrk3588_ddr_state_set_dram_type(void *state, uint32_t dram_type)
{
    qrk3588_ddr_state_global(state)->dram_type = dram_type;
}

static inline uint32_t *qrk3588_ddr_state_dram_type_ptr(void *state)
{
    return &qrk3588_ddr_state_global(state)->dram_type;
}

static inline const uint32_t *qrk3588_ddr_state_dram_type_ptr_const(const void *state)
{
    return &qrk3588_ddr_state_global_const(state)->dram_type;
}

static inline void *qrk3588_ddr_state_dram_params(const void *state)
{
    return (void *)(uintptr_t)qrk3588_ddr_state_global_const(state)->dram_params;
}

static inline void qrk3588_ddr_state_set_dram_params(void *state, const void *dram)
{
    qrk3588_ddr_state_global(state)->dram_params = (uint64_t)(uintptr_t)dram;
}

static inline void *qrk3588_ddr_state_timing(const void *state)
{
    return (void *)(uintptr_t)qrk3588_ddr_state_global_const(state)->timing;
}

static inline void qrk3588_ddr_state_set_timing(void *state, const void *timing)
{
    qrk3588_ddr_state_global(state)->timing = (uint64_t)(uintptr_t)timing;
}

/* Typed access to the proven geometry prefix of each channel row. */
static inline struct qrk3588_ddr_channel_geometry *qrk3588_ddr_state_channel_geometry(
    void *state, uint32_t channel)
{
    return &qrk3588_ddr_state_channel_view(state, channel)->geometry;
}

static inline const struct qrk3588_ddr_channel_geometry *
qrk3588_ddr_state_channel_geometry_const(const void *state, uint32_t channel)
{
    return &qrk3588_ddr_state_channel_view_const(state, channel)->geometry;
}

static inline uint32_t qrk3588_ddr_state_channel_rank_count(
    const void *state, uint32_t channel)
{
    return qrk3588_ddr_state_channel_geometry_const(state, channel)->rank_count;
}

static inline void qrk3588_ddr_state_set_channel_rank_count(
    void *state, uint32_t channel, uint32_t count)
{
    qrk3588_ddr_state_channel_geometry(state, channel)->rank_count = count;
}

static inline uint32_t qrk3588_ddr_state_channel_column_bits(
    const void *state, uint32_t channel)
{
    return qrk3588_ddr_state_channel_geometry_const(state, channel)->column_bits;
}

static inline uint32_t qrk3588_ddr_state_channel_device_width_code(
    const void *state, uint32_t channel)
{
    return qrk3588_ddr_state_channel_geometry_const(state, channel)->device_width_code;
}

static inline uint32_t qrk3588_ddr_state_channel_row_3_4(
    const void *state, uint32_t channel)
{
    return qrk3588_ddr_state_channel_geometry_const(state, channel)->row_3_4;
}

static inline uint32_t qrk3588_ddr_state_channel_rank_row_bits(
    const void *state, uint32_t channel, uint32_t rank)
{
    const struct qrk3588_ddr_channel_geometry *g =
        qrk3588_ddr_state_channel_geometry_const(state, channel);
    return rank < 4u ? g->cs_row_bits[rank] : 0u;
}

static inline void qrk3588_ddr_state_set_channel_rank_row_bits(
    void *state, uint32_t channel, uint32_t rank, uint32_t bits)
{
    struct qrk3588_ddr_channel_geometry *g =
        qrk3588_ddr_state_channel_geometry(state, channel);
    if (rank < 4u)
        g->cs_row_bits[rank] = bits;
}

static inline uint32_t qrk3588_ddr_state_channel_high16_row_bits(
    const void *state, uint32_t channel, uint32_t rank)
{
    const struct qrk3588_ddr_channel_geometry *g =
        qrk3588_ddr_state_channel_geometry_const(state, channel);
    return rank < 4u ? g->cs_high16_row_bits[rank] : 0u;
}

/* Training/capture state following the 0x88-byte global block.  The capture
 * record uses a fixed 0x20-byte stride per channel.  Unknown padding words are
 * retained explicitly instead of receiving speculative names. */
#define QRK3588_DDR_STATE_PHY_SAVED_BASE_OFFSET       0x238u
#define QRK3588_DDR_STATE_PHY_SAVED_STRIDE            0x20u
#define QRK3588_DDR_STATE_LIMIT_CODE_BASE_OFFSET      0x2c8u
#define QRK3588_DDR_STATE_PRIVATE_STATUS_BASE_OFFSET  0x2d8u
#define QRK3588_DDR_STATE_TRAINING_RESULT_BASE_OFFSET 0x2e8u
#define QRK3588_DDR_STATE_PRIVATE_COPY_BASE_OFFSET    0x328u
#define QRK3588_DDR_STATE_PRIVATE_COPY_SIZE           0x20u
#define QRK3588_DDR_STATE_TOTAL_SIZE                  0x358u

struct qrk3588_ddr_phy_saved_state {
    uint32_t bank_180;
    uint32_t reserved_04;
    uint32_t shared_04_low5;
    uint32_t shared_18_value;
    uint32_t shared_24_low13;
    uint32_t shared_4c_bit14;
    uint32_t reserved_18;
    uint32_t reserved_1c;
};

struct qrk3588_ddr_training_result {
    uint32_t direct0_rank0;
    uint32_t direct1_rank0;
    uint32_t direct0_rank1;
    uint32_t direct1_rank1;
};

/* Frequency-switch clock register save area at state+0x348. */
struct qrk3588_ddr_clock_saved_state {
    uint32_t reserved_348;
    uint32_t reg_280;
    uint32_t reg_304;
    uint32_t reserved_354;
};

_Static_assert(sizeof(struct qrk3588_ddr_phy_saved_state) == 0x20u,
               "per-channel PHY saved-state size");
_Static_assert(offsetof(struct qrk3588_ddr_phy_saved_state, bank_180) == 0x00u,
               "saved bank_180 offset");
_Static_assert(offsetof(struct qrk3588_ddr_phy_saved_state, reserved_04) == 0x04u,
               "saved reserved_04 offset");
_Static_assert(offsetof(struct qrk3588_ddr_phy_saved_state, shared_04_low5) == 0x08u,
               "saved shared_04 offset");
_Static_assert(offsetof(struct qrk3588_ddr_phy_saved_state, shared_18_value) == 0x0cu,
               "saved shared_18 offset");
_Static_assert(offsetof(struct qrk3588_ddr_phy_saved_state, shared_24_low13) == 0x10u,
               "saved shared_24 offset");
_Static_assert(offsetof(struct qrk3588_ddr_phy_saved_state, shared_4c_bit14) == 0x14u,
               "saved shared_4c offset");
_Static_assert(offsetof(struct qrk3588_ddr_phy_saved_state, reserved_18) == 0x18u,
               "saved reserved_18 offset");
_Static_assert(offsetof(struct qrk3588_ddr_phy_saved_state, reserved_1c) == 0x1cu,
               "saved reserved_1c offset");
_Static_assert(sizeof(struct qrk3588_ddr_training_result) == 0x10u,
               "per-channel training-result size");

_Static_assert(sizeof(struct qrk3588_ddr_clock_saved_state) == 0x10u,
               "clock saved-state size");
_Static_assert(offsetof(struct qrk3588_ddr_clock_saved_state, reserved_348) == 0x00u,
               "clock saved reserved_348 offset");
_Static_assert(offsetof(struct qrk3588_ddr_clock_saved_state, reg_280) == 0x04u,
               "clock saved reg_280 offset");
_Static_assert(offsetof(struct qrk3588_ddr_clock_saved_state, reg_304) == 0x08u,
               "clock saved reg_304 offset");
_Static_assert(offsetof(struct qrk3588_ddr_clock_saved_state, reserved_354) == 0x0cu,
               "clock saved reserved_354 offset");

/* Complete 0x358-byte working DDR state image.  Unknown storage remains
 * explicitly private, but every stable region boundary is represented in the
 * type so callers no longer depend on ad-hoc base offsets or a magic total
 * size. */
struct qrk3588_ddr_state_image {
    struct qrk3588_ddr_channel_state channel[4];
    struct qrk3588_ddr_global_state global;
    struct qrk3588_ddr_phy_saved_state phy_saved[4];
    uint32_t controller_b4_high11[4];
    uint32_t limit_code[4];
    uint32_t private_status[4];
    struct qrk3588_ddr_training_result training_result[4];
    uint8_t private_copy[QRK3588_DDR_STATE_PRIVATE_COPY_SIZE];
    struct qrk3588_ddr_clock_saved_state clock_saved;
};

_Static_assert(offsetof(struct qrk3588_ddr_state_image, channel) == 0x000u,
               "state channel block offset");
_Static_assert(offsetof(struct qrk3588_ddr_state_image, global) == 0x1b0u,
               "state global block offset");
_Static_assert(offsetof(struct qrk3588_ddr_state_image, phy_saved) == 0x238u,
               "state PHY-save block offset");
_Static_assert(offsetof(struct qrk3588_ddr_state_image, controller_b4_high11) == 0x2b8u,
               "state controller_b4_high11 offset");
_Static_assert(offsetof(struct qrk3588_ddr_state_image, limit_code) == 0x2c8u,
               "state limit-code offset");
_Static_assert(offsetof(struct qrk3588_ddr_state_image, private_status) == 0x2d8u,
               "state private-status offset");
_Static_assert(offsetof(struct qrk3588_ddr_state_image, training_result) == 0x2e8u,
               "state training-result offset");
_Static_assert(offsetof(struct qrk3588_ddr_state_image, private_copy) == 0x328u,
               "state private-copy offset");
_Static_assert(offsetof(struct qrk3588_ddr_state_image, clock_saved) == 0x348u,
               "state clock-saved offset");
_Static_assert(sizeof(struct qrk3588_ddr_state_image) == QRK3588_DDR_STATE_TOTAL_SIZE,
               "complete DDR state size");

static inline struct qrk3588_ddr_state_image *qrk3588_ddr_state_image(void *state)
{
    return (struct qrk3588_ddr_state_image *)state;
}

static inline const struct qrk3588_ddr_state_image *
qrk3588_ddr_state_image_const(const void *state)
{
    return (const struct qrk3588_ddr_state_image *)state;
}

static inline struct qrk3588_ddr_phy_saved_state *
qrk3588_ddr_state_phy_saved(void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image(state)->phy_saved[channel] : NULL;
}

static inline const struct qrk3588_ddr_phy_saved_state *
qrk3588_ddr_state_phy_saved_const(const void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image_const(state)->phy_saved[channel] : NULL;
}

static inline uint32_t *qrk3588_ddr_state_controller_b4_high11(
    void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image(state)->controller_b4_high11[channel] : NULL;
}

static inline const uint32_t *qrk3588_ddr_state_controller_b4_high11_const(
    const void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image_const(state)->controller_b4_high11[channel] : NULL;
}

static inline struct qrk3588_ddr_clock_saved_state *
qrk3588_ddr_state_clock_saved(void *state)
{
    return &qrk3588_ddr_state_image(state)->clock_saved;
}

static inline const struct qrk3588_ddr_clock_saved_state *
qrk3588_ddr_state_clock_saved_const(const void *state)
{
    return &qrk3588_ddr_state_image_const(state)->clock_saved;
}

static inline uint32_t *qrk3588_ddr_state_limit_code(void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image(state)->limit_code[channel] : NULL;
}

static inline const uint32_t *
qrk3588_ddr_state_limit_code_const(const void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image_const(state)->limit_code[channel] : NULL;
}

static inline uint32_t *qrk3588_ddr_state_private_status(void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image(state)->private_status[channel] : NULL;
}

static inline const uint32_t *
qrk3588_ddr_state_private_status_const(const void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image_const(state)->private_status[channel] : NULL;
}

static inline struct qrk3588_ddr_training_result *
qrk3588_ddr_state_training_result(void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image(state)->training_result[channel] : NULL;
}

static inline const struct qrk3588_ddr_training_result *
qrk3588_ddr_state_training_result_const(const void *state, uint32_t channel)
{
    return channel < 4u ? &qrk3588_ddr_state_image_const(state)->training_result[channel] : NULL;
}

static inline void *qrk3588_ddr_state_private_copy_region(void *state)
{
    return qrk3588_ddr_state_image(state)->private_copy;
}

static inline const void *qrk3588_ddr_state_private_copy_region_const(const void *state)
{
    return qrk3588_ddr_state_image_const(state)->private_copy;
}

static inline uint32_t qrk3588_ddr_state_channel_frequency_profile_selector(
    const void *state, uint32_t channel)
{
    return qrk3588_ddr_state_channel_view_const(state, channel)
        ->frequency_profile_selector;
}

static inline void qrk3588_ddr_state_set_channel_frequency_profile_selector(
    void *state, uint32_t channel, uint32_t selector)
{
    qrk3588_ddr_state_channel_view(state, channel)
        ->frequency_profile_selector = selector;
}

static inline struct qrk3588_ddr_channel_packed_profile *
qrk3588_ddr_state_channel_packed_profile(void *state, uint32_t channel)
{
    return &qrk3588_ddr_state_channel_view(state, channel)->packed_profile;
}

static inline const struct qrk3588_ddr_channel_packed_profile *
qrk3588_ddr_state_channel_packed_profile_const(const void *state, uint32_t channel)
{
    return &qrk3588_ddr_state_channel_view_const(state, channel)->packed_profile;
}

#endif
