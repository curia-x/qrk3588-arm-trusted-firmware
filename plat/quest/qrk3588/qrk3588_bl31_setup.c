/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stddef.h>
#include <assert.h>

#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <drivers/generic_delay_timer.h>
#if QRK3588_BOOT_EMMC
#include <qrk3588_mmc.h>
#endif

#include <lib/psci/psci.h>
#include <lib/utils_def.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <qrk3588_gic.h>
#include <qrk3588_mmio.h>
#include <qrk3588_timer.h>

#define MAP_BL31_TOTAL		MAP_REGION_FLAT(\
					BL31_BASE,\
					BL31_LIMIT - BL31_BASE,\
					MT_MEMORY | MT_RW | EL3_PAS)

#define MAP_BL31_RO		MAP_REGION_FLAT(\
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

void sgrf_init(void);

static entry_point_info_t bl32_image_ep_info;
static entry_point_info_t bl33_image_ep_info;
extern const plat_psci_ops_t qrk3588_plat_psci_ops;
void qrk3588_psci_setup(uintptr_t sec_entrypoint);

void bl31_early_platform_setup2(u_register_t arg0, u_register_t arg1,
		u_register_t arg2, u_register_t arg3)
{
	bl_params_t *params_from_bl2;
	bl_params_node_t *bl_params;
#if QRK3588_BOOT_EMMC
	int ret;
#endif

	(void)arg1;
	(void)arg3;

	params_from_bl2 = (bl_params_t *)arg0;
	if (params_from_bl2 != NULL) {
		assert(params_from_bl2->h.type == PARAM_BL_PARAMS);
		assert(params_from_bl2->h.version >= VERSION_2);
		bl_params = params_from_bl2->head;
		while (bl_params != NULL) {
			if (bl_params->image_id == BL32_IMAGE_ID &&
			    bl_params->ep_info != NULL)
				bl32_image_ep_info = *bl_params->ep_info;

			if (bl_params->image_id == BL33_IMAGE_ID &&
			    bl_params->ep_info != NULL) {
				bl33_image_ep_info = *bl_params->ep_info;
				if (bl33_image_ep_info.spsr == 0U)
					bl33_image_ep_info.spsr =
						SPSR_64(MODE_EL2, MODE_SP_ELX,
							DISABLE_ALL_EXCEPTIONS);
			}

			bl_params = bl_params->next_params_info;
		}
	}

	qrk3588_gic_setup();

	generic_delay_timer_init();
#if QRK3588_BOOT_EMMC
	ret = qrk3588_emmc_handoff((uint64_t)arg2);
	if (ret != 0)
		WARN("BL31: eMMC handoff unavailable (%d)\n", ret);
#else
	(void)arg2;
#endif
}

void bl31_plat_arch_setup(void)
{
	const mmap_region_t bl_regions[] = {
		MAP_BL31_TOTAL,
		MAP_BL31_RO,
		{0}
	};
	const mmap_region_t plat_regions[] = {
		MAP_DEVICE,
		/*
		 * Reserve the first SRAM page for the BootROM secondary-release
		 * mailbox and the low-address trampoline used by PSCI CPU_ON.
		 * Normal NC shares control data with MMU-off parked CPUs.
		 * BL1/BL2 must not use this page for memory-layout handoff.
		 */
		MAP_REGION_FLAT(QRK3588_SYSTEM_SRAM_BASE, PAGE_SIZE,
				MT_NON_CACHEABLE | MT_RW | EL3_PAS),
		{0}
	};

	/* Retire any earlier-stage cacheable copy before using the NC page. */
	flush_dcache_range(QRK3588_SYSTEM_SRAM_BASE, PAGE_SIZE);
	setup_page_tables(bl_regions, plat_regions);
	enable_mmu_el3(0);
}

void bl31_platform_setup(void)
{
	/* Rockchip implement. */
	sgrf_init();
}

void bl31_plat_runtime_setup(void)
{
}

unsigned int plat_get_syscnt_freq2(void)
{
	return QRK3588_SYS_COUNTER_FREQ;
}

entry_point_info_t *bl31_plat_get_next_image_ep_info(uint32_t type)
{
	entry_point_info_t *next_image_info;

	assert(sec_state_is_valid(type));
	if (type == NON_SECURE) {
		next_image_info = &bl33_image_ep_info;
	} else {
		next_image_info = &bl32_image_ep_info;
	}

	/*
	 * None of the images on the ARM development platforms can have 0x0
	 * as the entrypoint
	 */
	if (next_image_info->pc)
		return next_image_info;
	else
		return NULL;
}

int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const struct plat_psci_ops **psci_ops)
{
	qrk3588_psci_setup(sec_entrypoint);
	*psci_ops = &qrk3588_plat_psci_ops;

	return 0;
}

const unsigned char *plat_get_power_domain_tree_desc(void)
{
	/*
	 * PSCI encodes the topology as a root count followed by child counts
	 * for each non-CPU node in breadth-first order.
	 * Expose one shared DSU cluster with PLATFORM_CORE_COUNT CPU children.
	 */
	static const unsigned char power_domain_tree[] = {
		PLATFORM_CLUSTER_COUNT,
		PLATFORM_CORE_COUNT
	};

	return power_domain_tree;
}

plat_local_state_t plat_get_target_pwr_state(unsigned int lvl,
			const plat_local_state_t *states, unsigned int ncpu)
{
	(void)lvl;
	(void)states;
	(void)ncpu;
	return PSCI_LOCAL_STATE_RUN;
}
