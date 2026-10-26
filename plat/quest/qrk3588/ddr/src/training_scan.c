/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "qrk3588_ddr_internal.h"

struct ddr_vref_window_record {
    uint32_t code;
    int32_t low;
    int32_t high;
};

struct ddr_vref_limits {
    uint8_t reserved0[0x54];
    int32_t normal_positive;
    int32_t normal_negative_magnitude;
    int32_t direct_positive;
    int32_t direct_negative_magnitude;
    uint32_t normal_tolerance;
    uint32_t direct_tolerance;
    uint32_t minimum_selected_code;
};

#define VREF_WORK_LOW_ADDR       QRK3588_DDR_VREF_WORK_LOW_VA
#define VREF_SELECTED_ADDR       QRK3588_DDR_VREF_SELECTED_VA
#define VREF_SCAN_ADDR           QRK3588_DDR_VREF_SCAN_VA
#define VREF_REFERENCE_REC_ADDR  QRK3588_DDR_VREF_REFERENCE_RECORD_VA
#define VREF_BEST_REC_ADDR       QRK3588_DDR_VREF_BEST_RECORD_VA
#define VREF_WORK_HIGH_ADDR      QRK3588_DDR_VREF_WORK_HIGH_VA

static void vref_copy_record(uintptr_t base, int32_t mode,
                             const struct ddr_vref_window_record *record)
{
    uintptr_t address = base + (intptr_t)mode * 12;
    struct ddr_vref_window_record *dst =
        (struct ddr_vref_window_record *)(uintptr_t)DDR_LAYOUT_ADDR(address);
    *dst = *record;
}

static void vref_store_selected(int32_t mode, uint16_t code)
{
    uintptr_t address = VREF_SELECTED_ADDR + (intptr_t)mode * 2;
    *DDR_LAYOUT_PTR(uint16_t, address) = code;
}

static uint16_t vref_load_selected(int32_t mode)
{
    uintptr_t address = VREF_SELECTED_ADDR + (intptr_t)mode * 2;
    return *DDR_LAYOUT_PTR(uint16_t, address);
}

/*
 * Evaluate Vref scan data and update shared training state.  ASCII rendering
 * is diagnostic-only and does not participate in window selection or state
 * updates.
 */
int ddr_vref_training_scan(uint32_t selector, int32_t mode,
                           uint32_t direct, uint32_t channel,
                           const uint32_t *reference_code,
                           const void *limits_ptr, void *records_ptr)
{
    const struct ddr_vref_limits *limits =
        (const struct ddr_vref_limits *)limits_ptr;
    struct ddr_vref_window_record *records =
        (struct ddr_vref_window_record *)records_ptr;
    int32_t *window_low = DDR_LAYOUT_PTR(int32_t, VREF_WORK_LOW_ADDR);
    int32_t *window_high = DDR_LAYOUT_PTR(int32_t, VREF_WORK_HIGH_ADDR);
    int32_t *scan = DDR_LAYOUT_PTR(int32_t, VREF_SCAN_ADDR);
    int32_t minimum_low;
    int32_t maximum_high;
    int32_t best_width = 0;
    uint32_t best_code = 0;
    int32_t required_low;
    int32_t required_high;
    uint32_t tolerance;
    int status = 0;
    unsigned i;

    if (mode < -1 || mode >= 8)
        return -1;

    (void)ddr_training_window(window_low, window_high, selector, mode,
                              direct, channel);

    tolerance = direct ? limits->direct_tolerance : limits->normal_tolerance;
    required_high = direct ? limits->direct_positive : limits->normal_positive;
    required_low = -(direct ? limits->direct_negative_magnitude
                            : limits->normal_negative_magnitude);

    minimum_low = *window_high;
    maximum_high = *window_low;
    for (i = 0u; i < 20u; ++i) {
        const struct ddr_vref_window_record *r = &records[i];
        int32_t width;

        if (r->code == 0u)
            continue;
        if (r->low < minimum_low)
            minimum_low = r->low;
        if (r->high > maximum_high)
            maximum_high = r->high;
        width = r->high - r->low;
        if (width > best_width) {
            best_width = width;
            best_code = r->code;
        }
    }

    /* mode == -1 is a print/empty-scan call.  Its caller clears all 20
     * records first; a non-empty table would index the per-mode cache with
     * UINT32_MAX and is outside the valid ABI domain. */
    if (mode == -1 && records[0].code != 0u)
        return -1;

    if (scan[0] == 0) {
        int32_t lower = minimum_low - 20;
        int32_t raw_upper = maximum_high + 20;
        int32_t upper = raw_upper;
        if (lower < *window_low)
            lower = *window_low;
        if (upper > *window_high)
            upper = *window_high;
        scan[0] = lower;
        scan[1] = upper;
        /* The scan logic intentionally derives the printable step from the
         * unclamped high edge even when scan[1] itself is clamped. */
        scan[2] = (raw_upper - lower + 63) / 64;
    }

    for (i = 0u; i < 20u; ++i) {
        struct ddr_vref_window_record *r = &records[i];

        if (r->code == 0u)
            break;
        if (r->low > r->high)
            continue;

        if (i == 0u)
            vref_store_selected(mode, (uint16_t)r->code);

        if (r->code == best_code)
            vref_copy_record(VREF_BEST_REC_ADDR, mode, r);

        if (i != 19u) {
            uint32_t reference = *reference_code;
            if (r->code >= reference && reference > records[i + 1u].code)
                vref_copy_record(VREF_REFERENCE_REC_ADDR, mode, r);
        }

        if (tolerance != 0u) {
            uint32_t reference = *reference_code;
            uint32_t lower = reference - tolerance;
            uint32_t upper = reference + tolerance;

            if (r->code >= lower && r->code <= upper) {
                if (r->low > required_low || r->high < required_high)
                    status = -1;
            }
        }
    }

    if (mode != -1 && limits->minimum_selected_code != 0u &&
        vref_load_selected(mode) < limits->minimum_selected_code)
        status = -1;

    return status;
}
