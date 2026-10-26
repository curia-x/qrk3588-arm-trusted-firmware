/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLATFORM_DEF_H
#define PLATFORM_DEF_H

#include <plat/common/common_def.h>
#include "qrk3588_memory.h"

/*
 * Defines the normal stack memory available to each CPU. This
 * constant is used by plat/common/aarch64/platform_mp_stack.S
 * and plat/common/aarch64/platform_up_stack.S.
 */
#define PLATFORM_STACK_SIZE U(0xA000)

#define CACHE_WRITEBACK_SHIFT	U(6)
/*
 * Defines the size in bytes of the largest cache line across
 * all the cache levels in the platform.
 */
#define CACHE_WRITEBACK_GRANULE			(U(1) << CACHE_WRITEBACK_SHIFT)

/* Current SMP scope: four A55 CPUs and four A76 CPUs (affinities 0 through 0x700). */
#define PLATFORM_CORE_COUNT		U(8)
#define PLATFORM_CLUSTER_COUNT		U(1)

/********************** Definitions for PSCI. **********************/

/*
 * RK3588's eight CPUs share one DSU/L3 (Datasheet v1.7, section 1.2.1).
 * Expose eight CPU domains at level 0 beneath that shared cluster at level 1.
 * Count the exposed PSCI nodes, not all SoC power or voltage domains.
 * Cluster power switching is unsupported; its target state stays RUN.
 */
#define PLAT_NUM_PWR_DOMAINS	(PLATFORM_CLUSTER_COUNT + PLATFORM_CORE_COUNT)
#define PLAT_MAX_PWR_LVL		U(1)

/*
 * Defines the local power state corresponding to the deepest power
 * down possible at every power domain level in the platform. The local
 * power states for each level may be sparsely allocated between 0 and
 * this value with 0 being reserved for the RUN state. The PSCI
 * implementation uses this value to initialize the local power states
 * of the power domain nodes and to specify the requested power state
 * for a PSCI_CPU_OFF call.
 */
// TODO: Change to real number when bringup psci.
#define PLAT_MAX_OFF_STATE	2

/*
 * Defines the local power state corresponding to the deepest retention
 * state possible at every power domain level in the platform. This macro
 * should be a value less than PLAT_MAX_OFF_STATE and greater than 0.
 * It is used by the PSCI implementation to distinguish between retention
 * and power down local power states within PSCI_CPU_SUSPEND call.
 */
// TODO: Change to real number when bringup psci.
#define PLAT_MAX_RET_STATE 1

/*
 * Defines the maximum number of local power states per power domain
 * level that the platform supports. The default value of this macro is
 * 2 since most platforms just support a maximum of two local power
 * states at each power domain level (power-down and retention). If the
 * platform needs to account for more local power states, then it must
 * redefine this macro.
 *
 * Currently, this macro is used by the Generic PSCI implementation to
 * size the array used for PSCI_STAT_COUNT/RESIDENCY accounting.
 */
// TODO: Change to real number when bringup psci. (Default to 2)
// #define PLAT_MAX_PWR_LVL_STATES

/********************* Definitions for memory map. ********************/

/*
 * Defines the base address in secure ROM where BL1 originally lives.
 * Must be aligned on a page-size boundary.
 */
#define BL1_RO_BASE	QRK3588_BROM_STAGE0_LOAD_BASE

/*
 * Defines the maximum address in secure ROM that BL1’s actual content
 * (i.e. excluding any data section allocated at runtime) can occupy.
 */
#define BL1_RO_LIMIT QRK3588_BROM_STAGE0_LOAD_LIMIT

/**
 * Defines the base address in secure RAM where BL1’s read-write data
 * live at runtime.
 * Must be aligned on a page-size boundary.
 */
#define BL1_RW_BASE	BL1_RO_LIMIT

/*
 * Defines the maximum address in secure RAM that BL1’s read-write data
 * can occupy at runtime.
 */
#define BL1_RW_LIMIT (QRK3588_SYSTEM_SRAM_BASE + UL(0x80000))

/*
 * Defines the base address in secure RAM where BL1 loads the BL2 binary image.
 * Must be aligned on a page-size boundary.
 * This constant is not applicable when BL2_IN_XIP_MEM is set to ‘1’.
 */
#define BL2_BASE BL1_RW_LIMIT

/*
 * Defines the maximum address in secure RAM that the BL2 image can occupy.
 * This constant is not applicable when BL2_IN_XIP_MEM is set to ‘1’.
 */
#define BL2_LIMIT QRK3588_SYSTEM_SRAM_LIMIT

/*
 * Defines the base address in secure XIP memory where BL2 RO section originally lives.
 * Must be aligned on a page-size boundary.
 * This constant is only needed when BL2_IN_XIP_MEM is set to ‘1’.
 */
// #define BL2_RO_BASE

/*
 * Defines the maximum address in secure XIP memory that BL2’s actual content
 * (i.e. excluding any data section allocated at runtime) can occupy.
 * This constant is only needed when BL2_IN_XIP_MEM is set to ‘1’.
 */
// #define BL2_RO_LIMIT

/*
 * Defines the base address in secure RAM where BL2’s read-write data will live at runtime.
 * Must be aligned on a page-size boundary.
 * This constant is only needed when BL2_IN_XIP_MEM is set to ‘1’.
 */
// #define BL2_RW_BASE

/*
 * Defines the maximum address in secure RAM that BL2’s read-write data can occupy at runtime.
 * This constant is only needed when BL2_IN_XIP_MEM is set to ‘1’.
 */
// #define BL2_RW_LIMIT

/* Reserve the upper 2 MiB of the high DDR region for the EL3 runtime. */
#define BL31_LIMIT	QRK3588_DDR_HIGH_LIMIT
#define BL31_BASE	(BL31_LIMIT - SZ_2M)

/* Keep the non-secure payload in low DDR, independent of the BL31 carveout. */
#define NS_IMAGE_OFFSET	QRK3588_BL33_BASE
#define NS_IMAGE_MAX_SIZE	(QRK3588_DDR_LIMIT - NS_IMAGE_OFFSET)

/*
 * Defines the maximum message size between AP and RSE. Need to define if
 * platform supports RSE.
 */
// #define PLAT_RSE_COMMS_PAYLOAD_MAX_SIZE 1024

/* I/O device configuration */
#define MAX_IO_DEVICES			3
#define MAX_IO_HANDLES			4
#define MAX_IO_BLOCK_DEVICES		1

#define PLAT_VIRT_ADDR_SPACE_SIZE	(ULL(1) << 48)
#define PLAT_PHY_ADDR_SPACE_SIZE	(ULL(1) << 40)

#define MAX_MMAP_REGIONS		12
#define MAX_XLAT_TABLES			6

/* GIC */
#define PLAT_ARM_GICD_BASE		UL(0xFE600000)

#endif /* PLATFORM_DEF_H */
