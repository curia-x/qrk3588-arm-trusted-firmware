/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>

#include <platform_def.h>
#include <qrk3588_console.h>
#include <qrk3588_fwu.h>
#include <qrk3588_gpio.h>
#include <qrk3588_io_storage.h>
#include <qrk3588_mmc.h>
#include <quest_xmodem.h>

#define FWU_GPIO_SETTLE_US		U(100)

/* A 4 KiB SRAM staging buffer bounds RAM use regardless of FIP size. */
#define FWU_STAGING_SIZE		U(4096)

static bool fwu_requested;
static uint8_t fwu_staging[FWU_STAGING_SIZE] __aligned(512);
static uint8_t fwu_packet[QUEST_XMODEM_MAX_PACKET_SIZE];

struct fwu_writer {
	size_t used;
	size_t written;
};

static void fwu_uart_putc(void *cookie, uint8_t value)
{
	(void)cookie;
	(void)qrk3588_console_putc(value);
}

static int fwu_uart_getc(void *cookie, unsigned int timeout_us)
{
	(void)cookie;
	while (timeout_us-- != 0U) {
		int value = qrk3588_console_getc();

		if (value >= 0)
			return value;
		udelay(1);
	}
	return -ETIMEDOUT;
}

void qrk3588_fwu_sample_request(void)
{
	uint32_t mask = QRK3588_GPIO_IOMUX_MASK <<
			QRK3588_GPIO3_B2_IOMUX_SHIFT;

	/*
	 * Select GPIO input and enable its weak pull-up.  GPIO3 uses the RK3588
	 * PULL_TYPE_IO_1V8_ONLY encoding: 0b01 is pull-down, 0b11 is pull-up.
	 */
	mmio_write_32(QRK3588_GPIO3B_IOMUX_SEL_L, (mask << 16));
	mask = QRK3588_GPIO_PULL_MASK << QRK3588_GPIO3_B2_PULL_SHIFT;
	mmio_write_32(QRK3588_GPIO3B_PULL, (mask << 16) |
		      (QRK3588_GPIO_PULL_IO_1V8_ONLY_UP <<
		       QRK3588_GPIO3_B2_PULL_SHIFT));
	/* Clear B2's direction bit through the GPIO high-word write mask. */
	mmio_write_32(QRK3588_GPIO3_BASE + QRK3588_GPIO_SWPORT_DDR,
		      QRK3588_GPIO3_B2_DDR_WRITE_MASK);

	/* Reject a transient while the pad settles after reset. */
	fwu_requested = true;
	for (unsigned int i = 0U; i < 3U; i++) {
		if ((mmio_read_32(QRK3588_GPIO3_BASE + QRK3588_GPIO_EXT_PORT) &
		     QRK3588_GPIO3_B2_BIT) != 0U) {
			fwu_requested = false;
			break;
		}
		udelay(FWU_GPIO_SETTLE_US);
	}
	if (fwu_requested) {
		WARN("BL1: GPIO3_B2 low: iomux=0x%x pull=0x%x dr=0x%x ddr=0x%x ext=0x%x\n",
		     mmio_read_32(QRK3588_GPIO3B_IOMUX_SEL_L),
		     mmio_read_32(QRK3588_GPIO3B_PULL),
		     mmio_read_32(QRK3588_GPIO3_BASE + QRK3588_GPIO_SWPORT_DR),
		     mmio_read_32(QRK3588_GPIO3_BASE + QRK3588_GPIO_SWPORT_DDR),
		     mmio_read_32(QRK3588_GPIO3_BASE + QRK3588_GPIO_EXT_PORT));
	}
}

bool qrk3588_fwu_requested(void)
{
	return fwu_requested;
}

static int fwu_flush(struct fwu_writer *writer)
{
	int lba = (int)(QRK3588_FIP_IMAGE_OFFSET / 512U + writer->written / 512U);
	size_t write_size;

	if (writer->used == 0U)
		return 0;
	write_size = (writer->used + 511U) & ~((size_t)511U);
	if (write_size > writer->used)
		memset(fwu_staging + writer->used, 0, write_size - writer->used);
	if (qrk3588_emmc_write(lba, (uintptr_t)fwu_staging,
			write_size) != write_size)
		return -EIO;
	writer->written += writer->used;
	writer->used = 0U;
	return 0;
}

static int fwu_consume(void *cookie, const uint8_t *data, size_t size)
{
	struct fwu_writer *writer = cookie;

	if ((writer->used + size > FWU_STAGING_SIZE) &&
	    (fwu_flush(writer) != 0))
		return -EIO;
	if (writer->written + writer->used + size > QRK3588_FIP_IMAGE_MAX_SIZE)
		return -EFBIG;
	memcpy(fwu_staging + writer->used, data, size);
	writer->used += size;
	return 0;
}

int qrk3588_fwu_receive_fip(void)
{
#if !QRK3588_BOOT_EMMC
	return -ENOTSUP;
#else
	static const struct quest_xmodem_ops xmodem_ops = {
		.getc = fwu_uart_getc,
		.putc = fwu_uart_putc,
		.consume = fwu_consume,
	};
	static const struct quest_xmodem_config xmodem_config = {
		/* Send 'C' once per second while waiting for the PC-side sender. */
		.start_timeout_us = U(1000000),

		/* Allow five minutes for manually selecting the firmware file. */
		.start_max_timeouts = U(300),

		/* Once transfer starts, 200 ms is already very generous at 1.5 Mbaud. */
		.byte_timeout_us = U(200000),

		/* Abort after ten consecutive transfer failures. */
		.max_retries = U(10),
	};
	struct fwu_writer writer = { 0 };
	int ret;

	NOTICE("BL1: waiting for XMODEM FIP on UART2\n");
	ret = quest_xmodem_receive_crc(&xmodem_ops, &writer, fwu_packet,
					 sizeof(fwu_packet), &xmodem_config);
	if (ret != 0)
		return ret;
	ret = fwu_flush(&writer);
	if (ret != 0)
		return ret;
	NOTICE("BL1: received XMODEM FIP (%lu bytes)\n",
	       (unsigned long)writer.written);
	return writer.written == 0U ? -EINVAL : 0;
#endif
}
