/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Shared DDR-state / MMIO transition helpers. */

#include "qrk3588_ddr_internal.h"

void ddr_sync_controller_b4_high11(void *context, void *state, uint32_t channel)
{
    volatile uint32_t *controller = (volatile uint32_t *)
        qrk3588_ddr_ctx_channel_base1(context, channel);
    uint32_t value = controller[0xb4u / 4u];
    uint32_t code = value >> 21;
    uint32_t *saved = qrk3588_ddr_state_controller_b4_high11(state, channel);

    if (saved != NULL)
        *saved = code;

    value = controller[0xafcu / 4u];
    controller[0xafcu / 4u] = (value & UINT32_C(0xfffff800)) | code;

    value = controller[0xb4u / 4u];
    controller[0xb4u / 4u] = value | UINT32_C(0x00020000);
    value = controller[0xb4u / 4u];
    controller[0xb4u / 4u] = value & ~UINT32_C(0x00020000);
}

void ddr_clock_regs_save_restore(void *context, void *state, uint32_t save)
{
    volatile uint32_t *clock = (volatile uint32_t *)qrk3588_ddr_ctx_clock_table(context);
    struct qrk3588_ddr_clock_saved_state *saved = qrk3588_ddr_state_clock_saved(state);

    if (save == 1u) {
        saved->reg_280 = clock[0x280u / 4u];
        saved->reg_304 = clock[0x304u / 4u];
        clock[0x280u / 4u] = UINT32_C(0x03000000);
        clock[0x304u / 4u] = UINT32_C(0x0fc009c0);
    } else {
        clock[0x280u / 4u] = saved->reg_280 | UINT32_C(0xffff0000);
        clock[0x304u / 4u] = saved->reg_304 | UINT32_C(0xffff0000);
    }
}
