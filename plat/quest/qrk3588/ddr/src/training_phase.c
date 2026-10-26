/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "qrk3588_ddr_internal.h"

#define TRAINING_PHASE_RESULTS_ADDR QRK3588_DDR_TRAIN_PHASE_RECORDS_VA

static uint32_t *training_phase_results_ptr(void)
{
    return DDR_LAYOUT_PTR(uint32_t, TRAINING_PHASE_RESULTS_ADDR);
}

/* Sweep one training parameter, retaining up to twenty result records. */
int ddr_training_phase_exec(void *context, uint32_t limits[3],
                                       uint32_t selector, int32_t mode,
                                       uint32_t direct, uint32_t channel,
                                       uint32_t report)
{
    uint32_t *training_phase_results = training_phase_results_ptr();
    if (mode == -1 || mode == 0) {
        ddr_training_limits(&limits[0], &limits[1], &limits[2],
                            direct, channel);
        uint32_t step = (limits[1] - limits[0]) / 20u;
        if (step == 0u)
            return -1;
        for (uint32_t direction = 0; direction != 2u; ++direction) {
            uint32_t value = limits[2];
            for (uint32_t attempt = 0; attempt != 21U; ++attempt) {
                ddr_training_apply_limit(value, selector, direct, channel);
                if (qrk3588_ddr_get_error() != 0)
                    return qrk3588_ddr_get_error();
                int status = train_channels_loop(context, selector, 0u, direct,
                                                 training_phase_results, channel);
                if (status != 0) {
                    limits[direction ? 0u : 1u] = value;
                    break;
                }
                value = direction ? value - step : value + step;
                if (value >= limits[1] || value <= limits[0])
                    break;
            }
        }

        if (report) {
            const char *kind = direct ? DDR_LAYOUT_CPTR(char, QRK3588_DDR_LABEL_TX_VA)
                                      : DDR_LAYOUT_CPTR(char, QRK3588_DDR_LABEL_RX_VA);
            ddr_printf(DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_VREF_VA), kind,
                       limits[0] / 10u, limits[0] % 10u,
                       limits[1] / 10u, limits[1] % 10u,
                       limits[2] / 10u, limits[2] % 10u);
        }
    }

    for (uint32_t i = 0; i != 60u; i += 3u)
        training_phase_results[i] = 0u;

    uint32_t value = limits[1];
    uint32_t decrement = (limits[1] + 19u - limits[0]) / 20u;
    uint32_t used = 0u;
    if (decrement == 0u) {
        ddr_training_apply_limit(limits[2], selector, direct, channel);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        return -1;
    }
    while ((int32_t)value >= (int32_t)limits[0]) {
        uint32_t actual = ddr_training_apply_limit(value, selector,
                                                   direct, channel);
        if (qrk3588_ddr_get_error() != 0)
            return qrk3588_ddr_get_error();
        uint32_t previous = used == 0u
            ? DDR_LAYOUT_REF(uint32_t, QRK3588_DDR_VREF_SCAN_UPPER_VA)
            : training_phase_results[(used - 1u) * 3u];
        if (previous != actual) {
            if (train_channels_loop(context, selector, (uint32_t)mode, direct,
                                    &training_phase_results[used * 3u], channel) == 0) {
                training_phase_results[used * 3u] = actual;
                if (++used == 20u)
                    break;
            }
        }
        value -= decrement;
    }
    ddr_training_apply_limit(limits[2], selector, direct, channel);
    if (qrk3588_ddr_get_error() != 0)
        return qrk3588_ddr_get_error();
    return used == 0u ? -1 : 0;
}
