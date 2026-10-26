/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#if QRK3588_GICT_DIAGNOSTICS
#include <lib/mmio.h>
#endif
#include <plat/arm/common/plat_arm.h>
#include <plat/common/platform.h>

#include <platform_def.h>
#include <qrk3588_gic.h>

#if QRK3588_GICT_DIAGNOSTICS
#define QRK3588_GICD_SAC		(QRK3588_GICD_BASE + UL(0x24))
#define QRK3588_GICD_SAC_GICTNS		BIT(1)
#endif

static const uintptr_t qrk3588_gicr_frames[] = {
	QRK3588_GICR_BASE,
	0U,
};

void qrk3588_gic_setup(void)
{
#if QRK3588_GICT_DIAGNOSTICS
	/* Debug-only: expose GIC-600 RAS records to Non-secure software. */
	mmio_setbits_32(QRK3588_GICD_SAC, QRK3588_GICD_SAC_GICTNS);
#endif

	gic_set_gicr_frames(qrk3588_gicr_frames);
}

void plat_gic_pre_pcpu_init(unsigned int cpu_idx)
{
	(void)cpu_idx;
}
