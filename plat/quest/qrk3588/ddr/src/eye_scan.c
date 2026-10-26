/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "qrk3588_ddr_internal.h"

#define EYE_SCAN_STATE_ADDR        QRK3588_DDR_VREF_SCAN_VA
#define EYE_CURRENT_RECORDS_ADDR   QRK3588_DDR_TRAIN_PHASE_RECORDS_VA
#define EYE_BASELINE_RECORDS_ADDR  QRK3588_DDR_EYE_BASELINE_RECORDS_VA
#define EYE_RECORD_COUNT           20u
#define EYE_RECORD_BYTES           (EYE_RECORD_COUNT * 12u)

struct eye_window_record {
    uint32_t code;
    int32_t low;
    int32_t high;
};

static uint32_t eye_load_u32(const uint8_t *p, size_t off)
{
    return *(const uint32_t *)(const void *)(p + off);
}

static uintptr_t eye_load_ptr(const uint8_t *p, unsigned index)
{
    return (uintptr_t)((const uint64_t *)(const void *)p)[index];
}

/* Intersect the current mode's windows with the first successful mode used as
 * the eye-report baseline.  A code absent from the baseline is made an
 * explicitly invalid [1024,-1024] interval, matching the invalid-window encoding. */
static void eye_intersect_baseline_with_current(struct eye_window_record *baseline,
                                                const struct eye_window_record *current)
{
    unsigned i;

    for (i = 0u; i < EYE_RECORD_COUNT; ++i) {
        unsigned j;
        if (baseline[i].code == 0u)
            break;

        for (j = 0u; j < EYE_RECORD_COUNT; ++j) {
            if (current[j].code == baseline[i].code) {
                if (baseline[i].low < current[j].low)
                    baseline[i].low = current[j].low;
                if (baseline[i].high > current[j].high)
                    baseline[i].high = current[j].high;
                break;
            }
        }

        if (j == EYE_RECORD_COUNT) {
            baseline[i].low = 1024;
            baseline[i].high = -1024;
        }
    }
}

/* Run the two-pass Vref scan and merge the resulting eye windows. */
int ddr_eye_scan_report(void *config_ptr)
{
    uint8_t *config = (uint8_t *)config_ptr;
    int32_t *scan = DDR_LAYOUT_PTR(int32_t, EYE_SCAN_STATE_ADDR);
    struct eye_window_record *current =
        DDR_LAYOUT_PTR(struct eye_window_record, EYE_CURRENT_RECORDS_ADDR);
    struct eye_window_record *baseline =
        DDR_LAYOUT_PTR(struct eye_window_record, EYE_BASELINE_RECORDS_ADDR);
    uint32_t io_selector = eye_load_u32(config, 0x20u);
    uint32_t width_mode = eye_load_u32(config, 0x24u);
    uint32_t channel = eye_load_u32(config, 0x28u);
    uint32_t mode_count = width_mode == 0u ? 8u :
                          width_mode == 1u ? 16u : 32u;
    uint32_t selector;
    int failed = 0;

    scan[0] = 0;

    for (selector = 0u; selector < 4u; ++selector) {
        uintptr_t io_base = eye_load_ptr(config, selector);
        void *training_context = (void *)io_base;
        uint32_t direct;

        /* Vendor special case: selector 0 is configured even for a null base;
         * later selectors are skipped when their context pointer is absent. */
        if (selector != 0u && io_base == 0u)
            continue;

        if (io_config_dispatch((uint64_t)io_base, io_selector, 1u,
                               width_mode, 0u, UINT32_MAX) != 0) {
            failed = 1;
            continue;
        }

        for (direct = 0u; direct != 2u; ++direct) {
            uint32_t limits[3] = {0u, 0u, 0u};
            uint32_t failure_mask = 0u;
            uint32_t mode;

            scan[0] = 0;
            ddr_memset(baseline, 0, EYE_RECORD_BYTES);

            for (mode = 0u; mode < mode_count; ++mode) {
                int vref_status;

                if (ddr_training_phase_exec(&training_context, limits,
                                            selector, (int32_t)mode,
                                            direct, channel, 1u) != 0)
                    continue;

                vref_status = ddr_vref_training_scan(selector, (int32_t)mode,
                                                     direct, channel,
                                                     &limits[2], config,
                                                     current);

                if (baseline[0].code == 0u) {
                    (void)ddr_memcpy(baseline, current, EYE_RECORD_BYTES);
                } else {
                    eye_intersect_baseline_with_current(baseline, current);
                }

                if (vref_status != 0)
                    failure_mask |= 1u << mode;
            }

            /* mode=-1 renders the accumulated eye report.  Non-empty
             * mode=-1 caches are outside the safe ABI contract; this return is
             * diagnostic-only and intentionally ignored. */
            (void)ddr_vref_training_scan(selector, -1, direct, channel,
                                         &limits[2], config, baseline);

            if (failure_mask != 0u)
                failed = 1;
        }
    }

    return failed;
}
