/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <common/bl_common.h>
#include <common/debug.h>
#include <lib/utils_def.h>

#include <platform_def.h>
#include <qrk3588_mmc.h>

#include <qrk3588_bootstrap.h>

#if QRK3588_BOOT_EMMC

#define RK3588_IDBLOCK_LBA		U(64)
#define RK3588_IDBLOCK_SECTOR_SIZE	U(512)
#define RK3588_IDBLOCK_MAGIC_RKNS	U(0x534e4b52)
#define RK3588_IDBLOCK_MEDIA_EMMC	U(0x434d4d45)
#define RK3588_IDBLOCK_SKIP_IMAGE_SHA	BIT_32(14)

/* The first stage image follows the one-sector IDBlock header. */
#define RK3588_IDBLOCK_IMAGE0_OFFSET	U(1)

/* BootROM's fixed SRAM boot_source_id field and its MaskROM value. */
#define RK3588_BROM_BOOT_SOURCE_ADDR	(QRK3588_SYSTEM_SRAM_BASE + U(0x10))
#define RK3588_BROM_BOOT_SOURCE_MASKROM	U(10)

/* Fields consumed by the RK3588 BootROM in its 512-byte IDBlock header. */
#define RK3588_IDBLOCK_BOOT_FLAG_OFF	U(0x0c)
#define RK3588_IDBLOCK_MEDIA_MAGIC_OFF	U(0x30)
#define RK3588_IDBLOCK_IMAGE0_OFF	U(0x78)

static uint8_t idblock_header[RK3588_IDBLOCK_SECTOR_SIZE] __aligned(512);
static uint8_t idblock_tail[RK3588_IDBLOCK_SECTOR_SIZE] __aligned(512);

static bool booted_from_maskrom(void)
{
	return *(volatile const uint32_t *)(uintptr_t)RK3588_BROM_BOOT_SOURCE_ADDR ==
	       RK3588_BROM_BOOT_SOURCE_MASKROM;
}

static void idblock_write_u32(size_t offset, uint32_t value)
{
	memcpy(idblock_header + offset, &value, sizeof(value));
}

static int write_bl1_payload(size_t image_size, size_t image_sectors)
{
	const uintptr_t image = BL1_RO_BASE;
	size_t whole_sectors = image_size / RK3588_IDBLOCK_SECTOR_SIZE;
	size_t tail_size = image_size % RK3588_IDBLOCK_SECTOR_SIZE;
	int lba = RK3588_IDBLOCK_LBA + RK3588_IDBLOCK_IMAGE0_OFFSET;

	if (whole_sectors != 0U) {
		size_t whole_size = whole_sectors * RK3588_IDBLOCK_SECTOR_SIZE;

		if (qrk3588_emmc_write(lba, image, whole_size) != whole_size)
			return -EIO;
		lba += (int)whole_sectors;
	}

	if (tail_size != 0U) {
		memset(idblock_tail, 0, sizeof(idblock_tail));
		memcpy(idblock_tail, (const void *)(image + whole_sectors *
			       RK3588_IDBLOCK_SECTOR_SIZE), tail_size);
		if (qrk3588_emmc_write(lba, (uintptr_t)idblock_tail,
				       sizeof(idblock_tail)) != sizeof(idblock_tail))
			return -EIO;
	}

	return image_sectors == whole_sectors + (tail_size != 0U) ? 0 : -EINVAL;
}

int qrk3588_bootstrap_emmc_bl1(void)
{
	size_t image_size;
	size_t image_sectors;
	uint32_t size_and_offset;

	if (!booted_from_maskrom())
		return 0;

	if (BL1_ROM_END <= BL1_RO_BASE)
		return -EINVAL;

	image_size = BL1_ROM_END - BL1_RO_BASE;
	image_sectors = round_up(image_size, RK3588_IDBLOCK_SECTOR_SIZE) /
			RK3588_IDBLOCK_SECTOR_SIZE;

	if (image_sectors > UINT16_MAX)
		return -EFBIG;

	NOTICE("BL1: MaskROM download detected; installing BL1 to eMMC\n");
	if (write_bl1_payload(image_size, image_sectors) != 0)
		return -EIO;

	/*
	 * Publish the header only after the complete payload is on eMMC.  A reset
	 * during the payload write therefore leaves no valid IDBlock magic behind.
	 */
	memset(idblock_header, 0, sizeof(idblock_header));
	idblock_write_u32(0U, RK3588_IDBLOCK_MAGIC_RKNS);
	idblock_write_u32(RK3588_IDBLOCK_BOOT_FLAG_OFF,
			  RK3588_IDBLOCK_SKIP_IMAGE_SHA);
	idblock_write_u32(RK3588_IDBLOCK_MEDIA_MAGIC_OFF,
			  RK3588_IDBLOCK_MEDIA_EMMC);
	size_and_offset = ((uint32_t)image_sectors << 16) |
			  RK3588_IDBLOCK_IMAGE0_OFFSET;
	idblock_write_u32(RK3588_IDBLOCK_IMAGE0_OFF, size_and_offset);
	if (qrk3588_emmc_write(RK3588_IDBLOCK_LBA, (uintptr_t)idblock_header,
			       sizeof(idblock_header)) != sizeof(idblock_header))
		return -EIO;

	NOTICE("BL1: installed %lu-byte IDBlock payload at LBA %u\n",
	       (unsigned long)image_size,
	       RK3588_IDBLOCK_LBA + RK3588_IDBLOCK_IMAGE0_OFFSET);
	return 0;
}

#else

int qrk3588_bootstrap_emmc_bl1(void)
{
	return 0;
}

#endif /* QRK3588_BOOT_EMMC */
