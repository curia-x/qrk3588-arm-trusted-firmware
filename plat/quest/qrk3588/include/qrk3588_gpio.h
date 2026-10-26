/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_GPIO_H
#define QRK3588_GPIO_H

#include <lib/utils_def.h>

/* IOC register domains used by GPIO banks 1 through 4. */
#define QRK3588_BUS_IOC_BASE		UL(0xfd5f8000)
#define QRK3588_VCCIO3_5_IOC_BASE	UL(0xfd5fa000)

/* GPIO controller base addresses. */
#define QRK3588_GPIO0_BASE		UL(0xfd8a0000)
#define QRK3588_GPIO1_BASE		UL(0xfec20000)
#define QRK3588_GPIO2_BASE		UL(0xfec30000)
#define QRK3588_GPIO3_BASE		UL(0xfec40000)
#define QRK3588_GPIO4_BASE		UL(0xfec50000)

/* GPIO v2 controller register offsets. */
#define QRK3588_GPIO_SWPORT_DR		UL(0x00)
#define QRK3588_GPIO_SWPORT_DDR	UL(0x08)
#define QRK3588_GPIO_EXT_PORT		UL(0x70)

/* GPIO3B controls in the BUS_IOC and VCCIO3_5_IOC register domains. */
#define QRK3588_GPIO3B_IOMUX_SEL_L	(QRK3588_BUS_IOC_BASE + UL(0x68))
#define QRK3588_GPIO3B_PULL		(QRK3588_VCCIO3_5_IOC_BASE + UL(0x134))

/* GPIO3_B2 fields in the GPIO3 controller and GPIO3B IOC registers. */
#define QRK3588_GPIO3_B2_BIT		BIT_32(10)
#define QRK3588_GPIO3_B2_DDR_WRITE_MASK	(QRK3588_GPIO3_B2_BIT << 16)
#define QRK3588_GPIO3_B2_IOMUX_SHIFT	U(8)
#define QRK3588_GPIO3_B2_PULL_SHIFT	U(4)

/* Common IOC field widths and PULL_TYPE_IO_1V8_ONLY encodings. */
#define QRK3588_GPIO_IOMUX_MASK	U(0xf)
#define QRK3588_GPIO_PULL_MASK		U(0x3)
#define QRK3588_GPIO_PULL_IO_1V8_ONLY_UP	U(0x3)

#endif /* QRK3588_GPIO_H */
