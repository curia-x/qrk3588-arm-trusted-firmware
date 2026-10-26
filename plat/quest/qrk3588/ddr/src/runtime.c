/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <lib/mmio.h>
#include <lib/utils_def.h>

#include <qrk3588_clock.h>

#include "qrk3588_ddr_internal.h"

uint32_t ddr_phy_reg_write(uint32_t phy_base, uint32_t reg_addr,
			   uint32_t data_hi, uint32_t data_lo)
{
	uintptr_t base = phy_base + PHY_OFFSET;

	if (qrk3588_ddr_poll32(base + PHY_STATUS_REG, BIT(0), 0U, 0U) != 0)
		return (uint32_t)QRK3588_DDR_ETIMEDOUT;

	mmio_write_32(base + PHY_CTRL_REG, reg_addr << 4);
	mmio_write_32(base + PHY_DATA_REG,
		      (data_lo & U(0xff)) | (data_hi << 8));
	mmio_setbits_32(base + PHY_CTRL_REG, BIT(31));

	if (qrk3588_ddr_poll32(base + PHY_CTRL_REG, BIT(31), 0U, 0U) != 0)
		return (uint32_t)QRK3588_DDR_ETIMEDOUT;
	if (qrk3588_ddr_poll32(base + PHY_STATUS_REG, BIT(0), 0U, 0U) != 0)
		return (uint32_t)QRK3588_DDR_ETIMEDOUT;

	return 0U;
}

void ddr_phy_update_complete(uint32_t phy_base)
{
	uintptr_t base = phy_base + PHY_OFFSET;

	mmio_write_32(base + PHY_COMPLETE_TRIGGER, 1U);
	(void)qrk3588_ddr_poll32(base + PHY_COMPLETE_STATUS, BIT(0),
				 BIT(0), 0U);
}

void ddr_phy_update_toggle(uintptr_t ctrl_base)
{
	mmio_setbits_32(ctrl_base + PHY_UPDATE_CTRL_REG,
			  PHY_UPDATE_START_BIT);
	udelay(1U);
	mmio_clrbits_32(ctrl_base + PHY_UPDATE_CTRL_REG,
			  PHY_UPDATE_START_BIT);
}

int ddr_config_check(const void *config)
{
	void *sdram_params;

	if (config == NULL)
		return QRK3588_DDR_EINVAL;

	sdram_params = DDR_LAYOUT_REF(void *,
				       QRK3588_DDR_SDRAM_PARAMS_SLOT_VA);
	(void)ddr_memcpy(sdram_params, config,
			 QRK3588_DDR_BOARD_CONFIG_SIZE);

	return 0;
}


/* Initialize and train DRAM for the BL2 cold-boot path. */
int qrk3588_ddr_main(void *config)
{
	int ret;

	ret = ddr_config_check(config);
	if (ret != 0)
		return ret;

	ret = dram_type_detect();
	if (ret != 0)
		goto error;

	ret = ddr_training_prepare();
	if (ret != 0 || qrk3588_ddr_get_error() != 0)
		goto error;

	ret = ddr_training_main();
	if (ret != 0 || qrk3588_ddr_get_error() != 0)
		goto error;

	dbg_puts(DDR_LAYOUT_CPTR(char, QRK3588_DDR_BOOT_BANNER_VA));

	__asm__ volatile("ic ialluis");
	__asm__ volatile("isb");
	return 0;

error:
	{
		int error = qrk3588_ddr_get_error();

		return error != 0 ? error : ret;
	}
}


void ddr_memset(void *dst, int c, size_t n)
{
	uint8_t *d = (void *)qrk3588_ddr_resolve_addr((uintptr_t)dst);

	while (n-- != 0U)
		*d++ = (uint8_t)c;
}

uintptr_t ddr_memcpy(void *dst, const void *src, size_t n)
{
	uintptr_t original = (uintptr_t)dst;
	uint8_t *d = (void *)qrk3588_ddr_resolve_addr((uintptr_t)dst);
	const uint8_t *s = (const void *)qrk3588_ddr_resolve_addr((uintptr_t)src);

	if (d == s)
		return original;

	if ((((uintptr_t)d | (uintptr_t)s) & U(7)) == 0U) {
		size_t qwords = n >> 3;

		for (size_t i = 0; i < qwords; ++i)
			((uint64_t *)d)[i] = ((const uint64_t *)s)[i];
		d += qwords << 3;
		s += qwords << 3;
		n -= qwords << 3;
	}

	while (n-- != 0U)
		*d++ = *s++;

	return original;
}

uint32_t ddr_reference_clock_hz(void)
{
	return QRK3588_OSC_HZ;
}
