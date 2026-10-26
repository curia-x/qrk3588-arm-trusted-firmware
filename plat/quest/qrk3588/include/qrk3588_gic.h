/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_GIC_H
#define QRK3588_GIC_H

#include <lib/utils_def.h>

#define QRK3588_GICD_BASE	UL(0xfe600000)
#define QRK3588_GICR_BASE	UL(0xfe680000)

/* Register the GICR frame layout before BL31's generic GIC setup runs. */
void qrk3588_gic_setup(void);

#endif /* QRK3588_GIC_H */
