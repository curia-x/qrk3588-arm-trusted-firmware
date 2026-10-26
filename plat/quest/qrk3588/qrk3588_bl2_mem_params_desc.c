/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <common/desc_image_load.h>
#include <plat/common/platform.h>

#include <platform_def.h>

/* BL2 loads the EL3 runtime before the non-secure payload. */
static bl_mem_params_node_t qrk3588_bl2_mem_params_descs[] = {
	{
		.image_id = BL31_IMAGE_ID,
		SET_STATIC_PARAM_HEAD(ep_info, PARAM_EP, VERSION_2,
			entry_point_info_t, SECURE | EXECUTABLE | EP_FIRST_EXE),
		.ep_info.pc = BL31_BASE,
		.ep_info.spsr = SPSR_64(MODE_EL3, MODE_SP_ELX,
					DISABLE_ALL_EXCEPTIONS),
		SET_STATIC_PARAM_HEAD(image_info, PARAM_EP, VERSION_2,
			image_info_t, IMAGE_ATTRIB_PLAT_SETUP),
		.image_info.image_base = BL31_BASE,
		.image_info.image_max_size = BL31_LIMIT - BL31_BASE,
		.next_handoff_image_id = BL33_IMAGE_ID,
	},
	{
		.image_id = BL33_IMAGE_ID,
		SET_STATIC_PARAM_HEAD(ep_info, PARAM_EP, VERSION_2,
			entry_point_info_t, NON_SECURE | EXECUTABLE),
		/* Keep EL2 available to Linux KVM through the BL33 handoff. */
		.ep_info.pc = NS_IMAGE_OFFSET,
		.ep_info.spsr = SPSR_64(MODE_EL2, MODE_SP_ELX,
				DISABLE_ALL_EXCEPTIONS),
		SET_STATIC_PARAM_HEAD(image_info, PARAM_EP, VERSION_2,
			image_info_t, 0),
		.image_info.image_base = NS_IMAGE_OFFSET,
		.image_info.image_max_size = NS_IMAGE_MAX_SIZE,
		.next_handoff_image_id = INVALID_IMAGE_ID,
	},
};

REGISTER_BL_IMAGE_DESCS(qrk3588_bl2_mem_params_descs)
