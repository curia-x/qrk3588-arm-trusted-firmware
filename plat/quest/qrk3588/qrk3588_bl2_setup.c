/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stddef.h>

#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/desc_image_load.h>
#include <drivers/console.h>
#include <drivers/delay_timer.h>
#include <drivers/generic_delay_timer.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <platform_def.h>
#include <qrk3588_ddr.h>
#include <qrk3588_io_storage.h>
#include <qrk3588_mmio.h>
#include <qrk3588_timer.h>

#define MAP_BL2_TOTAL		MAP_REGION_FLAT(\
					BL2_BASE,\
					BL2_LIMIT - BL2_BASE,\
					MT_MEMORY | MT_RW | EL3_PAS)

#define MAP_BL2_RO		MAP_REGION_FLAT(\
					BL_CODE_BASE,\
					BL_CODE_END - BL_CODE_BASE,\
					MT_CODE | EL3_PAS),\
				MAP_REGION_FLAT(\
					BL_RO_DATA_BASE,\
					BL_RO_DATA_END - BL_RO_DATA_BASE,\
					MT_RO_DATA | EL3_PAS)

#define MAP_DEVICE		MAP_REGION_FLAT(\
					QRK3588_DEVICE_BASE,\
					QRK3588_DEVICE_SIZE,\
					MT_DEVICE | MT_RW | EL3_PAS)

/* BL2 must map every destination to which the image loader writes. */
#define MAP_BL31_LOAD		MAP_REGION_FLAT(\
					BL31_BASE,\
					BL31_LIMIT - BL31_BASE,\
					MT_MEMORY | MT_RW | EL3_PAS)

#define MAP_BL33_LOAD		MAP_REGION_FLAT(\
					NS_IMAGE_OFFSET,\
					NS_IMAGE_MAX_SIZE,\
					MT_MEMORY | MT_RW | MT_NS)

bl_mem_params_node_t *bl_mem_params_desc_ptr __attribute__((weak));
unsigned int bl_mem_params_desc_num __attribute__((weak));

unsigned int plat_get_syscnt_freq2(void)
{
	return QRK3588_SYS_COUNTER_FREQ;
}

void bl2_early_platform_setup2(u_register_t arg0, u_register_t arg1,
			       u_register_t arg2, u_register_t arg3)
{
	int ret;

	generic_delay_timer_init();

#if QRK3588_BL2_DEBUG
	/*
	 * XMODEM completes before tio can normally reconnect to UART2.  Pause at
	 * the BL2 entry point so the debug terminal can reconnect before later
	 * boot messages are emitted.  Keep release boot timing unchanged.
	 */
	NOTICE("BL2: DEBUG: delaying 1500 ms for tio reconnect\n");
	udelay(1500000U);
#endif

	ret = qrk3588_ddr_init();
	if (ret != 0) {
		ERROR("BL2: DDR initialization failed (%d)\n", ret);
		panic();
	}

	ret = qrk3588_bl2_io_setup((uint64_t)arg2);
	if (ret != 0) {
		ERROR("BL2: storage handoff failed (%d)\n", ret);
		panic();
	}

#if QRK3588_BOOT_EMMC
	{
		bl_mem_params_node_t *bl31 = get_bl_mem_params_node(BL31_IMAGE_ID);

		if (bl31 == NULL)
			panic();
		bl31->ep_info.args.arg2 = arg2;
	}
#endif
}

void bl2_plat_arch_setup(void)
{
	const mmap_region_t bl_regions[] = {
		MAP_BL2_TOTAL,
		MAP_BL2_RO,
		{0}
	};
	const mmap_region_t plat_regions[] = {
		MAP_BL31_LOAD,
		MAP_BL33_LOAD,
		MAP_DEVICE,
		{0}
	};

	setup_page_tables(bl_regions, plat_regions);
	enable_mmu_el1(0);
}

void bl2_platform_setup(void)
{
}

/*******************************************************************************
 * This function is a wrapper of a common function which returns the list of
 * loadable images.
 ******************************************************************************/
struct bl_load_info *plat_get_bl_image_load_info(void)
{
	return get_bl_load_info_from_mem_params_desc();
}

/*******************************************************************************
 * This function returns the list of executable images.
 ******************************************************************************/
struct bl_params *plat_get_next_bl_params(void)
{
	return get_next_bl_params_from_mem_params_desc();
}

/*******************************************************************************
 * This function is a wrapper of a common function which flushes the data
 * structures so that they are visible in memory for the next BL image.
 ******************************************************************************/
void plat_flush_next_bl_params(void)
{
	flush_bl_params_desc();
}
