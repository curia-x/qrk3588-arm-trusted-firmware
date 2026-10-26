/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_DDR_CONTEXT_H
#define QRK3588_DDR_CONTEXT_H

#include <stddef.h>
#include <stdint.h>

/* Shared hardware-context layout used by the DDR controller/PHY routines. */
#define QRK3588_DDR_CHANNEL_COUNT              4u
#define QRK3588_DDR_CTX_CHANNEL_STRIDE         0x20u
#define QRK3588_DDR_CTX_CHANNEL_BASE0_OFFSET   0x00u
#define QRK3588_DDR_CTX_CHANNEL_BASE1_OFFSET   0x08u
#define QRK3588_DDR_CTX_CHANNEL_BASE2_OFFSET   0x10u
#define QRK3588_DDR_CTX_CHANNEL_OUTPUT_OFFSET  0x18u

#define QRK3588_DDR_CTX_GEOMETRY_REGS_OFFSET   0x80u
#define QRK3588_DDR_CTX_CLOCK_TABLE_OFFSET     0x88u
#define QRK3588_DDR_CTX_ADDRESS_MAP_REGS_OFFSET 0x90u
#define QRK3588_DDR_CTX_GRF_REGS_OFFSET        0x98u
#define QRK3588_DDR_CTX_GRF_MIRROR_REGS_OFFSET 0xa0u
#define QRK3588_DDR_CTX_RESET_REGS_OFFSET      0xb8u
#define QRK3588_DDR_CTX_TOPOLOGY_REGS_OFFSET   0xc0u
#define QRK3588_DDR_CTX_SHARED_BANK0_OFFSET    0xc8u
#define QRK3588_DDR_CTX_SHARED_BANK1_OFFSET    0xd0u
#define QRK3588_DDR_CTX_ROUTE_TABLE_OFFSET     0xd8u
#define QRK3588_DDR_CTX_DESCRIPTOR_STATE_OFFSET 0xe8u
#define QRK3588_DDR_CTX_HANDOFF_STATE_OFFSET    0xfcu

#define QRK3588_DDR_TOPOLOGY_HANDOFF_MAGIC      UINT32_C(0x02468ace)
#define QRK3588_DDR_TOPOLOGY_HANDOFF_ACK        UINT32_C(0x13579bdf)

struct qrk3588_ddr_context_descriptor_state {
    uint32_t word4_hi16;
    uint32_t word4_lo16;
    uint32_t wordc_bit26;
    uint32_t wordc_bit27;
    uint32_t wordc_bit19_inverted;
};

/* Retained-topology handoff state is independent of descriptor decoding.
 * It is populated only when the topology register block carries the handoff
 * magic and is consumed by frequency-switch/training paths. */
struct qrk3588_ddr_context_handoff_state {
    uint32_t topology_word88_bit26;
    uint32_t valid;
};

_Static_assert(offsetof(struct qrk3588_ddr_context_descriptor_state, word4_hi16) == 0x00u,
               "descriptor word4 high-half offset");
_Static_assert(offsetof(struct qrk3588_ddr_context_descriptor_state, word4_lo16) == 0x04u,
               "descriptor word4 low-half offset");
_Static_assert(offsetof(struct qrk3588_ddr_context_descriptor_state, wordc_bit26) == 0x08u,
               "descriptor wordc bit26 offset");
_Static_assert(offsetof(struct qrk3588_ddr_context_descriptor_state, wordc_bit27) == 0x0cu,
               "descriptor wordc bit27 offset");
_Static_assert(offsetof(struct qrk3588_ddr_context_descriptor_state, wordc_bit19_inverted) == 0x10u,
               "descriptor wordc bit19-inverted offset");
_Static_assert(sizeof(struct qrk3588_ddr_context_descriptor_state) == 0x14u,
               "descriptor-derived context state size");
_Static_assert(offsetof(struct qrk3588_ddr_context_handoff_state, topology_word88_bit26) == 0x00u,
               "handoff word88 bit26 offset");
_Static_assert(offsetof(struct qrk3588_ddr_context_handoff_state, valid) == 0x04u,
               "handoff valid offset");
_Static_assert(sizeof(struct qrk3588_ddr_context_handoff_state) == 0x08u,
               "handoff state size");
#define QRK3588_DDR_CHANNEL_UNUSED             0x0fu

static inline uint8_t *qrk3588_ddr_ctx_channel_slot(void *ctx, uint32_t channel)
{
    return (uint8_t *)ctx + (size_t)channel * QRK3588_DDR_CTX_CHANNEL_STRIDE;
}

static inline const uint8_t *qrk3588_ddr_ctx_channel_slot_const(const void *ctx,
                                                                uint32_t channel)
{
    return (const uint8_t *)ctx + (size_t)channel * QRK3588_DDR_CTX_CHANNEL_STRIDE;
}

static inline uintptr_t qrk3588_ddr_ctx_load_ptr(const void *ctx, size_t offset)
{
    return (uintptr_t)*(const uint64_t *)((const uint8_t *)ctx + offset);
}

static inline void qrk3588_ddr_ctx_store_ptr(void *ctx, size_t offset, uintptr_t value)
{
    *(uint64_t *)((uint8_t *)ctx + offset) = (uint64_t)value;
}

static inline uintptr_t qrk3588_ddr_ctx_channel_base0(const void *ctx, uint32_t channel)
{
    return qrk3588_ddr_ctx_load_ptr(qrk3588_ddr_ctx_channel_slot_const(ctx, channel),
                                   QRK3588_DDR_CTX_CHANNEL_BASE0_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_channel_base1(const void *ctx, uint32_t channel)
{
    return qrk3588_ddr_ctx_load_ptr(qrk3588_ddr_ctx_channel_slot_const(ctx, channel),
                                   QRK3588_DDR_CTX_CHANNEL_BASE1_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_channel_base2(const void *ctx, uint32_t channel)
{
    return qrk3588_ddr_ctx_load_ptr(qrk3588_ddr_ctx_channel_slot_const(ctx, channel),
                                   QRK3588_DDR_CTX_CHANNEL_BASE2_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_channel_output(const void *ctx, uint32_t channel)
{
    return qrk3588_ddr_ctx_load_ptr(qrk3588_ddr_ctx_channel_slot_const(ctx, channel),
                                   QRK3588_DDR_CTX_CHANNEL_OUTPUT_OFFSET);
}

static inline void qrk3588_ddr_ctx_store_channel_base0(void *ctx, uint32_t channel,
                                                       uintptr_t value)
{
    qrk3588_ddr_ctx_store_ptr(qrk3588_ddr_ctx_channel_slot(ctx, channel),
                             QRK3588_DDR_CTX_CHANNEL_BASE0_OFFSET, value);
}

static inline void qrk3588_ddr_ctx_store_channel_base1(void *ctx, uint32_t channel,
                                                       uintptr_t value)
{
    qrk3588_ddr_ctx_store_ptr(qrk3588_ddr_ctx_channel_slot(ctx, channel),
                             QRK3588_DDR_CTX_CHANNEL_BASE1_OFFSET, value);
}

static inline void qrk3588_ddr_ctx_store_channel_base2(void *ctx, uint32_t channel,
                                                       uintptr_t value)
{
    qrk3588_ddr_ctx_store_ptr(qrk3588_ddr_ctx_channel_slot(ctx, channel),
                             QRK3588_DDR_CTX_CHANNEL_BASE2_OFFSET, value);
}

static inline void qrk3588_ddr_ctx_store_channel_output(void *ctx, uint32_t channel,
                                                        uintptr_t value)
{
    qrk3588_ddr_ctx_store_ptr(qrk3588_ddr_ctx_channel_slot(ctx, channel),
                             QRK3588_DDR_CTX_CHANNEL_OUTPUT_OFFSET, value);
}

static inline uintptr_t qrk3588_ddr_ctx_geometry_regs(const void *ctx)
{
    return qrk3588_ddr_ctx_load_ptr(ctx, QRK3588_DDR_CTX_GEOMETRY_REGS_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_clock_table(const void *ctx)
{
    return qrk3588_ddr_ctx_load_ptr(ctx, QRK3588_DDR_CTX_CLOCK_TABLE_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_address_map_regs(const void *ctx)
{
    return qrk3588_ddr_ctx_load_ptr(ctx, QRK3588_DDR_CTX_ADDRESS_MAP_REGS_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_grf_regs(const void *ctx)
{
    return qrk3588_ddr_ctx_load_ptr(ctx, QRK3588_DDR_CTX_GRF_REGS_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_grf_mirror_regs(const void *ctx)
{
    return qrk3588_ddr_ctx_load_ptr(ctx, QRK3588_DDR_CTX_GRF_MIRROR_REGS_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_reset_regs(const void *ctx)
{
    return qrk3588_ddr_ctx_load_ptr(ctx, QRK3588_DDR_CTX_RESET_REGS_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_topology_regs(const void *ctx)
{
    return qrk3588_ddr_ctx_load_ptr(ctx, QRK3588_DDR_CTX_TOPOLOGY_REGS_OFFSET);
}

static inline uintptr_t qrk3588_ddr_ctx_shared_bank(const void *ctx, uint32_t channel)
{
    return qrk3588_ddr_ctx_load_ptr(ctx, channel < 2u ? QRK3588_DDR_CTX_SHARED_BANK0_OFFSET
                                                     : QRK3588_DDR_CTX_SHARED_BANK1_OFFSET);
}

static inline uint32_t qrk3588_ddr_ctx_route(const void *ctx, uint32_t slot)
{
    return *(const uint32_t *)((const uint8_t *)ctx + QRK3588_DDR_CTX_ROUTE_TABLE_OFFSET
                               + (size_t)slot * sizeof(uint32_t));
}

static inline void qrk3588_ddr_ctx_store_route(void *ctx, uint32_t slot, uint32_t channel)
{
    *(uint32_t *)((uint8_t *)ctx + QRK3588_DDR_CTX_ROUTE_TABLE_OFFSET
                  + (size_t)slot * sizeof(uint32_t)) = channel;
}

static inline struct qrk3588_ddr_context_descriptor_state *
qrk3588_ddr_ctx_descriptor_state(void *ctx)
{
    return (struct qrk3588_ddr_context_descriptor_state *)
        ((uint8_t *)ctx + QRK3588_DDR_CTX_DESCRIPTOR_STATE_OFFSET);
}

static inline const struct qrk3588_ddr_context_descriptor_state *
qrk3588_ddr_ctx_descriptor_state_const(const void *ctx)
{
    return (const struct qrk3588_ddr_context_descriptor_state *)
        ((const uint8_t *)ctx + QRK3588_DDR_CTX_DESCRIPTOR_STATE_OFFSET);
}

static inline struct qrk3588_ddr_context_handoff_state *
qrk3588_ddr_ctx_handoff_state(void *ctx)
{
    return (struct qrk3588_ddr_context_handoff_state *)
        ((uint8_t *)ctx + QRK3588_DDR_CTX_HANDOFF_STATE_OFFSET);
}

static inline const struct qrk3588_ddr_context_handoff_state *
qrk3588_ddr_ctx_handoff_state_const(const void *ctx)
{
    return (const struct qrk3588_ddr_context_handoff_state *)
        ((const uint8_t *)ctx + QRK3588_DDR_CTX_HANDOFF_STATE_OFFSET);
}

/* Capture/acknowledge the retained topology handoff marker.  Returns 1 when a
 * valid handoff was consumed, 0 for the ordinary cold/normal path. */
static inline int qrk3588_ddr_ctx_capture_topology_handoff(void *ctx)
{
    struct qrk3588_ddr_context_handoff_state *handoff =
        qrk3588_ddr_ctx_handoff_state(ctx);
    volatile uint32_t *topology =
        (volatile uint32_t *)(uintptr_t)qrk3588_ddr_ctx_topology_regs(ctx);

    handoff->topology_word88_bit26 = 0u;
    handoff->valid = 0u;
    if (topology == NULL)
        return 0;
    if (topology[0x84u / 4u] != QRK3588_DDR_TOPOLOGY_HANDOFF_MAGIC)
        return 0;

    handoff->valid = 1u;
    handoff->topology_word88_bit26 = (topology[0x88u / 4u] >> 26) & 1u;
    topology[0x84u / 4u] = QRK3588_DDR_TOPOLOGY_HANDOFF_ACK;
    return 1;
}

#endif
