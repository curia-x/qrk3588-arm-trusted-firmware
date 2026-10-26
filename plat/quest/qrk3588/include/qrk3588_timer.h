/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_TIMER_H
#define QRK3588_TIMER_H

#include <lib/utils_def.h>

#define QRK3588_HPTIMER_BASE		UL(0xfd8c8000)
#define QRK3588_HPTIMER_CTRL		U(0x04)
#define QRK3588_HPTIMER_LOAD_COUNT0	U(0x14)
#define QRK3588_HPTIMER_LOAD_COUNT1	U(0x18)
#define QRK3588_HPTIMER_CTRL_ENABLE	BIT(0)
#define QRK3588_HPTIMER_LOAD_MAX	UINT32_MAX
#define QRK3588_COUNTER_START_RETRIES	U(100000)
#define QRK3588_SYS_COUNTER_FREQ	U(24000000)

#endif /* QRK3588_TIMER_H */
