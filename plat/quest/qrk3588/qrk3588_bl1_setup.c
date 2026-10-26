/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <arch_helpers.h>
#include <bl1/bl1.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <drivers/generic_delay_timer.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <platform_def.h>
#include <qrk3588_io_storage.h>
#include <qrk3588_mmc.h>
#include <qrk3588_mmio.h>
#include <qrk3588_bootstrap.h>
#include <qrk3588_clock.h>
#include <qrk3588_fwu.h>
#include <qrk3588_timer.h>

#define MAP_BL1_TOTAL		MAP_REGION_FLAT(\
					bl1_tzram_layout.total_base,\
					bl1_tzram_layout.total_size,\
					MT_MEMORY | MT_RW | EL3_PAS)

#define MAP_BL1_RO		MAP_REGION_FLAT(\
					BL_CODE_BASE,\
					BL1_CODE_END - BL_CODE_BASE,\
					MT_CODE | EL3_PAS),\
				MAP_REGION_FLAT(\
					BL1_RO_DATA_BASE,\
					BL1_RO_DATA_END\
						- BL1_RO_DATA_BASE,\
					MT_RO_DATA | EL3_PAS)

#define MAP_DEVICE		MAP_REGION_FLAT(\
					QRK3588_DEVICE_BASE,\
					QRK3588_DEVICE_SIZE,\
					MT_DEVICE | MT_RW | EL3_PAS)

/* Data structure which holds the extents of the trusted RAM for BL1 */
static meminfo_t bl1_tzram_layout;

meminfo_t *bl1_plat_sec_mem_layout(void)
{
	return &bl1_tzram_layout;
}

void bl1_early_platform_setup(void)
{
	uint64_t midr, mpidr;

	asm volatile("mrs %0, midr_el1"  : "=r"(midr));
	asm volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
	NOTICE("BL1: MIDR = 0x%lx, MPIDR = 0x%lx\n", midr, mpidr);

	/* Allow BL1 to see the whole Trusted RAM */
	bl1_tzram_layout.total_base = QRK3588_SYSTEM_SRAM_BASE;
	bl1_tzram_layout.total_size = QRK3588_SYSTEM_SRAM_SIZE;
}

int bl1_plat_handle_pre_image_load(unsigned int image_id)
{
	if ((image_id != BL2_IMAGE_ID) || !qrk3588_fwu_requested())
		return 0;

	NOTICE("BL1: GPIO3_B2 requests firmware upgrade\n");
	return qrk3588_fwu_receive_fip();
}

void bl1_plat_arch_setup(void)
{
	/* 1. Setup up identity mapping for all addresses, (4KB granule size only) */
	const mmap_region_t bl_regions[] = {
		MAP_BL1_TOTAL,
		MAP_BL1_RO,
		{0}
	};

	/* Setup a mapping for mmio devices */
	const mmap_region_t plat_qrk3588_mmap[] = {
		MAP_DEVICE,
		{0}
	};

	setup_page_tables(bl_regions, plat_qrk3588_mmap);
	/* 2. Enable the mmu as well as the icache and dcache.  */
	enable_mmu_el3(0);
}

static int qrk3588_clk_init(void)
{
	int ret;

	/* PLL public MODE muxes output XIN24M; core0 is now exactly 24 MHz. */
	qrk3588_clock_enter_osc_safe_state();

	/*
	 * Install final proven downstream mux/dividers before touching PLL M/P/S.
	 * LPLL MODE is still OSC, so core0 /2 temporarily becomes 12 MHz.
	 */
	qrk3588_clock_preconfigure_boot_tree();

	/* Program and lock all PLL primitives while consumers still see OSC paths. */
	ret = qrk3588_clock_program_boot_plls();
	if (ret)
		return ret;

	/* LPLL MODE is released last here: core0 changes 12 -> 450 MHz. */
	qrk3588_clock_release_boot_plls();

	/* Secure matrix upstream is not fully documented, so its roots switch now. */
	qrk3588_clock_activate_secure_roots();

	return 0;
}

unsigned int plat_get_syscnt_freq2(void)
{
	return QRK3588_SYS_COUNTER_FREQ;
}

static int qrk3588_timer_init(void)
{
	uint64_t start;

	/* Early delays use a CPU loop; the system counter may still be off. */
	write_cntfrq_el0(QRK3588_SYS_COUNTER_FREQ);
	if ((mmio_read_32(QRK3588_HPTIMER_BASE + QRK3588_HPTIMER_CTRL) &
	     QRK3588_HPTIMER_CTRL_ENABLE) == 0U) {
		mmio_write_32(QRK3588_HPTIMER_BASE + QRK3588_HPTIMER_LOAD_COUNT0,
			      QRK3588_HPTIMER_LOAD_MAX);
		mmio_write_32(QRK3588_HPTIMER_BASE + QRK3588_HPTIMER_LOAD_COUNT1,
			      QRK3588_HPTIMER_LOAD_MAX);
		mmio_write_32(QRK3588_HPTIMER_BASE + QRK3588_HPTIMER_CTRL,
			      QRK3588_HPTIMER_CTRL_ENABLE);
	}
	isb();
	start = read_cntpct_el0();
	/* Do not enter udelay() if a disabled counter would hang it forever. */
	for (unsigned int i = 0; i < QRK3588_COUNTER_START_RETRIES; i++) {
		if (read_cntpct_el0() != start) {
			generic_delay_timer_init();
			return 0;
		}
	}
	return -ETIMEDOUT;
}

void bl1_platform_setup(void)
{
	int ret;

	ret = qrk3588_clk_init();
	if (ret != 0) {
		ERROR("BL1: clock initialization failed (%d)\n", ret);
		plat_error_handler(ret);
	}

	ret = qrk3588_timer_init();
	if (ret != 0) {
		ERROR("BL1: system counter is not running (%d)\n", ret);
		plat_error_handler(ret);
	}

	/* GPIO/IOC clocks are now live; sample before storage and image loading. */
	qrk3588_fwu_sample_request();

	ret = plat_io_setup();
	if (ret != 0) {
		plat_error_handler(ret);
	}

	ret = qrk3588_bootstrap_emmc_bl1();
	if (ret != 0) {
		ERROR("BL1: eMMC BL1 bootstrap failed (%d)\n", ret);
		plat_error_handler(ret);
	}
}

int bl1_plat_handle_post_image_load(unsigned int image_id)
{
	image_desc_t *desc;

	if (image_id != BL2_IMAGE_ID) {
		return 0;
	}
	desc = bl1_plat_get_image_desc(BL2_IMAGE_ID);
	assert(desc != NULL);

	/* BL2 uses a static memory layout and does not consume meminfo. */
	desc->ep_info.args.arg1 = 0U;
#if QRK3588_BOOT_EMMC
	desc->ep_info.args.arg2 = qrk3588_emmc_handoff_token();
#endif
	return 0;
}

unsigned int bl1_plat_get_next_image_id(void)
{
	/* BL2 load will be done by default. */
	return BL2_IMAGE_ID;
}
