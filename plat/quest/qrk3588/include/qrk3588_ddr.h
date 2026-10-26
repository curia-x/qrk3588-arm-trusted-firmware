/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_DDR_H
#define QRK3588_DDR_H

#include <stdint.h>

#define QRK3588_DDR_MAX_REGIONS 10U

struct qrk3588_ddr_region {
	uint64_t start;
	uint64_t size;
};

struct qrk3588_ddr_info {
	uint32_t count;
	struct qrk3588_ddr_region regions[QRK3588_DDR_MAX_REGIONS];
};

int qrk3588_ddr_init(void *param);

#endif /* QRK3588_DDR_H */
