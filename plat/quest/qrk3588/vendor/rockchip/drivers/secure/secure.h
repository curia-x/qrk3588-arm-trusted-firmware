/*
 * Copyright (c) 2024, Rockchip, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Derived from Trusted Firmware-A Rockchip RK3588 platform code.
 * Original: plat/rockchip/rk3588/drivers/secure/secure.h
 * Upstream revision: ffb5fa55dc8e2d3d5923cf94d6beef6f40561d9c
 */

#ifndef SECURE_H
#define SECURE_H

/* PMUSGRF */
#define PMU1SGRF_SOC_CON(n)		((n) * 4)

/* SGRF */
#define SGRF_SOC_CON(i)			((i) * 4)
#define SGRF_FIREWALL_CON(i)		(0x240 + (i) * 4)
#define SGRF_FIREWALL_CON_CNT		32

/* ddr firewall */
#define FIREWALL_DDR_RGN(i)		((i) * 0x4)
#define FIREWALL_DDR_RGN_CNT		16
#define FIREWALL_DDR_MST(i)		(0x40 + (i) * 0x4)
#define FIREWALL_DDR_MST_CNT		42
#define FIREWALL_DDR_CON		0xf0

#define FIREWALL_SYSMEM_RGN(i)		((i) * 0x4)
#define FIREWALL_SYSMEM_RGN_CNT		8
#define FIREWALL_SYSMEM_MST(i)		(0x40 + (i) * 0x4)
#define FIREWALL_SYSMEM_MST_CNT		43
#define FIREWALL_SYSMEM_CON		0xf0

#define FIREWALL_DSU_RGN(i)		((i) * 0x4)
#define FIREWALL_DSU_RGN_CNT		16
#define FIREWALL_DSU_MST(i)		(0x40 + (i) * 0x4)
#define FIREWALL_DSU_MST_CNT		2
#define FIREWALL_DSU_CON(i)		(0xf0 + (i) * 4)
#define FIREWALL_DSU_CON_CNT		4

#define FIREWALL_DDR_ADDR_LIMIT_MB	0x8000	/* 15-bit MiB address field */
#define RG_MAP_SECURE(top, base)	\
	(((((top) - 1) & 0x7fff) << 16) | ((base) & 0x7fff))
#define RG_MAP_SRAM_SECURE(top_kb, base_kb)	\
	(((((top_kb) / 4 - 1) & 0xff) << 16) | ((base_kb) / 4 & 0xff))

void sgrf_init(void);

#endif /* SECURE_H */
