/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdbool.h>

#include <common/debug.h>
#include <common/tbbr/tbbr_img_def.h>
#include <drivers/io/io_driver.h>
#include <drivers/io/io_block.h>
#include <drivers/io/io_fip.h>
#include <drivers/io/io_storage.h>
#include <lib/utils_def.h>
#include <plat/common/platform.h>
#include <tools_share/firmware_image_package.h>

#include <platform_def.h>
#include <qrk3588_io_storage.h>
#if QRK3588_BOOT_EMMC || QRK3588_BOOT_SDMMC
#include <qrk3588_mmc.h>
#elif QRK3588_BOOT_SPI_NOR
#include <qrk3588_fspi.h>
#endif

static uintptr_t storage_dev_handle;
static uintptr_t fip_dev_handle;
static bool storage_ready;

/* io_block also uses this buffer for unaligned FIP headers and payloads. */
static uint8_t block_buffer[4096] __aligned(512);

static io_block_dev_spec_t storage_dev_spec = {
	.buffer = {
		.offset = (uintptr_t)block_buffer,
		.length = sizeof(block_buffer),
	},
	.block_size = 512,
};

static const io_block_spec_t fip_block_spec = {
	.offset = QRK3588_FIP_IMAGE_OFFSET,
	.length = QRK3588_FIP_IMAGE_MAX_SIZE,
};

CASSERT((QRK3588_FIP_IMAGE_OFFSET % 512) == 0, fip_offset_block_aligned);
CASSERT((QRK3588_FIP_IMAGE_MAX_SIZE % 512) == 0, fip_size_block_aligned);
CASSERT(QRK3588_FIP_IMAGE_MAX_SIZE > 0, fip_size_nonzero);
/* io_block passes the absolute sector number to the driver as an int. */
CASSERT(QRK3588_FIP_IMAGE_MAX_SIZE <= 0x10000000000ULL,
	fip_size_addressable);
CASSERT(QRK3588_FIP_IMAGE_OFFSET <= 0x10000000000ULL -
	QRK3588_FIP_IMAGE_MAX_SIZE, fip_region_addressable);

static const io_uuid_spec_t bl2_uuid_spec = {
	.uuid = UUID_TRUSTED_BOOT_FIRMWARE_BL2,
};

static const io_uuid_spec_t bl31_uuid_spec = {
	.uuid = UUID_EL3_RUNTIME_FIRMWARE_BL31,
};

static const io_uuid_spec_t bl33_uuid_spec = {
	.uuid = UUID_NON_TRUSTED_FIRMWARE_BL33,
};

#if TRUSTED_BOARD_BOOT
static const io_uuid_spec_t trusted_key_uuid_spec = {
	.uuid = UUID_TRUSTED_KEY_CERT,
};

static const io_uuid_spec_t soc_key_uuid_spec = {
	.uuid = UUID_SOC_FW_KEY_CERT,
};

static const io_uuid_spec_t soc_content_uuid_spec = {
	.uuid = UUID_SOC_FW_CONTENT_CERT,
};

static const io_uuid_spec_t nt_key_uuid_spec = {
	.uuid = UUID_NON_TRUSTED_FW_KEY_CERT,
};

static const io_uuid_spec_t nt_content_uuid_spec = {
	.uuid = UUID_NON_TRUSTED_FW_CONTENT_CERT,
};

static const io_uuid_spec_t tb_fw_cert_uuid_spec = {
	.uuid = UUID_TRUSTED_BOOT_FW_CERT,
};
#endif

int plat_get_image_source(unsigned int image_id, uintptr_t *dev_handle,
			  uintptr_t *image_spec)
{
	const io_uuid_spec_t *uuid_spec;
	int ret;

	if ((dev_handle == NULL) || (image_spec == NULL)) {
		return -EINVAL;
	}
	if (!storage_ready) {
		return -ENODEV;
	}

	/* The FIP driver calls back here to obtain its physical backing store. */
	if (image_id == FIP_IMAGE_ID) {
		*dev_handle = storage_dev_handle;
		*image_spec = (uintptr_t)&fip_block_spec;
		return 0;
	}

	switch (image_id) {
	case BL2_IMAGE_ID:
		uuid_spec = &bl2_uuid_spec;
		break;
	case BL31_IMAGE_ID:
		uuid_spec = &bl31_uuid_spec;
		break;
	case BL33_IMAGE_ID:
		uuid_spec = &bl33_uuid_spec;
		break;
#if TRUSTED_BOARD_BOOT
	case TRUSTED_KEY_CERT_ID:
		uuid_spec = &trusted_key_uuid_spec;
		break;
	case SOC_FW_KEY_CERT_ID:
		uuid_spec = &soc_key_uuid_spec;
		break;
	case SOC_FW_CONTENT_CERT_ID:
		uuid_spec = &soc_content_uuid_spec;
		break;
	case NON_TRUSTED_FW_KEY_CERT_ID:
		uuid_spec = &nt_key_uuid_spec;
		break;
	case NON_TRUSTED_FW_CONTENT_CERT_ID:
		uuid_spec = &nt_content_uuid_spec;
		break;
	case TRUSTED_BOOT_FW_CERT_ID:
		uuid_spec = &tb_fw_cert_uuid_spec;
		break;
#endif
	default:
		return -ENOENT;
	}

	ret = io_dev_init(fip_dev_handle, FIP_IMAGE_ID);
	if (ret != 0) {
		ERROR("QRK3588: cannot open FIP at 0x%lx (%d)\n",
		      (unsigned long)QRK3588_FIP_IMAGE_OFFSET, ret);
		return ret;
	}
	*dev_handle = fip_dev_handle;
	*image_spec = (uintptr_t)uuid_spec;
	return 0;
}

static int qrk3588_register_io_devices(void)
{
	const io_dev_connector_t *block_connector;
	const io_dev_connector_t *fip_connector;
	int ret;

	ret = register_io_dev_block(&block_connector);
	if (ret != 0) {
		return ret;
	}
	ret = io_dev_open(block_connector, (uintptr_t)&storage_dev_spec,
			  &storage_dev_handle);
	if (ret != 0) {
		return ret;
	}
	ret = register_io_dev_fip(&fip_connector);
	if (ret == 0) {
		ret = io_dev_open(fip_connector, 0, &fip_dev_handle);
	}
	if (ret != 0) {
		(void)io_dev_close(storage_dev_handle);
		return ret;
	}

	storage_ready = true;
	NOTICE("QRK3588: FIP offset 0x%lx, window 0x%lx bytes\n",
	       (unsigned long)QRK3588_FIP_IMAGE_OFFSET,
	       (unsigned long)QRK3588_FIP_IMAGE_MAX_SIZE);
	return 0;
}

int plat_io_setup(void)
{
	int ret;

	if (storage_ready) {
		return 0;
	}

#if QRK3588_BOOT_EMMC
	ret = qrk3588_emmc_init();
	storage_dev_spec.ops.read = qrk3588_emmc_read;
#elif QRK3588_BOOT_SDMMC
	ret = qrk3588_sdmmc_init();
	storage_dev_spec.ops.read = qrk3588_sdmmc_read;
#elif QRK3588_BOOT_SPI_NOR
	ret = qrk3588_fspi_nor_init();
	storage_dev_spec.ops.read = qrk3588_fspi_nor_read;
#else
#error "Select a QRK3588 boot device"
#endif
	if (ret != 0) {
		ERROR("QRK3588: storage initialization failed (%d)\n", ret);
		return ret;
	}

	return qrk3588_register_io_devices();
}

int qrk3588_bl2_io_setup(uint64_t emmc_handoff_token)
{
	int ret;

	if (storage_ready) {
		return 0;
	}

#if QRK3588_BOOT_EMMC
	ret = qrk3588_emmc_handoff(emmc_handoff_token);
	storage_dev_spec.ops.read = qrk3588_emmc_read;
#else
	(void)emmc_handoff_token;
	ret = -ENOTSUP;
#endif
	if (ret != 0) {
		ERROR("QRK3588: eMMC handoff failed (%d)\n", ret);
		return ret;
	}

	return qrk3588_register_io_devices();
}
