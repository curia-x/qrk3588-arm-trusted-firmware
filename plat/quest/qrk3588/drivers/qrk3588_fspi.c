/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>

#include <qrk3588_fspi.h>

/*
 * This is a native TF-A SPI NOR reader. It uses the RK3588 SFC register
 * interface and standard JEDEC NOR commands to supply 512-byte blocks to the
 * TF-A I/O layer. Register addresses, bit definitions and command values are
 * defined by the hardware and protocol interfaces.
 *
 * PIO keeps the reader independent of DMA address-width and cache handling
 * during the early SRAM-only boot stage.
 */
#define SFC_BASE			UL(0xfe2b0000)
#define SFC_CTRL			U(0x000)
#define SFC_IMR			U(0x004)
#define SFC_ICLR			U(0x008)
#define SFC_RCVR			U(0x010)
#define SFC_FSR			U(0x020)
#define SFC_SR			U(0x024)
#define SFC_RISR			U(0x028)
#define SFC_DLL_CTRL		U(0x03c)
#define SFC_LEN_CTRL		U(0x088)
#define SFC_CMD			U(0x100)
#define SFC_ADDR			U(0x104)
#define SFC_DATA			U(0x108)

#define SFC_PHASE_NEGATIVE	BIT(1)
#define SFC_FIFO_EMPTY		(BIT(1) | BIT(2))
#define SFC_BUSY			BIT(0)
#define SFC_RX_LEVEL_SHIFT	U(16)
#define SFC_RX_LEVEL_MASK	U(0x1f)
/* FIFO underflow/overflow, bus error and serial-interface error. */
#define SFC_ERRORS		(BIT(1) | BIT(2) | BIT(5) | BIT(6))
#define SFC_CMD_ADDR_24		BIT(14)
#define SFC_CMD_LENGTH_SHIFT	U(16)
#define SFC_MAX_TRANSFER		U(0x2000)
#define SFC_RESET_TIMEOUT_US	U(1000)
#define SFC_FIFO_TIMEOUT_US	U(10000)
#define SFC_BUSY_TIMEOUT_US	U(100000)

#define FSPI_CLKSEL		UL(0xfd7c0438)
#define FSPI_CLKGATE		UL(0xfd7c087c)
#define FSPI_IOC_BASE		UL(0xfd5f8040)

#define NOR_READ		U(0x03)
#define NOR_READ_STATUS		U(0x05)
#define NOR_RESET_ENABLE		U(0x66)
#define NOR_RESET		U(0x99)
#define NOR_READ_ID		U(0x9f)
#define NOR_STATUS_BUSY		BIT(0)
#define NOR_READY_TIMEOUT_US	U(10000)
#define NOR_SECTOR_SIZE		U(512)
#define NOR_MAX_CAPACITY_LOG2	U(24)

static uint32_t nor_capacity;

static int fspi_wait_clear(unsigned int reg, uint32_t mask,
			   unsigned int timeout_us)
{
	uint64_t timeout = timeout_init_us(timeout_us);

	while ((mmio_read_32(SFC_BASE + reg) & mask) != 0U) {
		if (timeout_elapsed(timeout)) {
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	return 0;
}

static int fspi_reset(void)
{
	int ret;

	mmio_write_32(SFC_BASE + SFC_RCVR, 1U);
	ret = fspi_wait_clear(SFC_RCVR, BIT(0), SFC_RESET_TIMEOUT_US);
	if (ret != 0) {
		return ret;
	}

	mmio_write_32(SFC_BASE + SFC_ICLR, UINT32_MAX);
	mmio_write_32(SFC_BASE + SFC_CTRL, SFC_PHASE_NEGATIVE);
	/* Always use the command's 14-bit length, including after warm entry. */
	mmio_write_32(SFC_BASE + SFC_LEN_CTRL, 0U);
	if (((mmio_read_32(SFC_BASE + SFC_FSR) & SFC_FIFO_EMPTY) !=
	     SFC_FIFO_EMPTY) ||
	    ((mmio_read_32(SFC_BASE + SFC_SR) & SFC_BUSY) != 0U)) {
		return -EIO;
	}

	return 0;
}

static int fspi_controller_init(void)
{
	/* Select the 24 MHz oscillator with divider 1 for conservative operation. */
	mmio_write_32(FSPI_CLKSEL, U(0x3fc02000));
	/* Enable NVM bus roots, HCLK_NVM, SCLK_SFC and HCLK_SFC. */
	mmio_write_32(FSPI_CLKGATE, U(0x06070000));

#if QRK3588_FSPI_IOMUX == 2
	mmio_write_32(FSPI_IOC_BASE + U(0x20), U(0xffff5555));
	mmio_write_32(FSPI_IOC_BASE + U(0x24), U(0x00f00050));
	mmio_write_32(FSPI_IOC_BASE + U(0x34), U(0x000f0002));
#elif QRK3588_FSPI_IOMUX == 1
	mmio_write_32(FSPI_IOC_BASE + U(0x04), U(0xff003300));
	mmio_write_32(FSPI_IOC_BASE + U(0x08), U(0xf0ff3033));
	mmio_write_32(FSPI_IOC_BASE + U(0x0c), U(0x000f0003));
#else
	mmio_write_32(FSPI_IOC_BASE + U(0x00), U(0x000f0002));
	mmio_write_32(FSPI_IOC_BASE + U(0x18), U(0xffff2222));
	mmio_write_32(FSPI_IOC_BASE + U(0x1c), U(0x0f000200));
#endif
	udelay(1);
	/* PIO does not require any DMA firewall configuration. */

	mmio_write_32(SFC_BASE + SFC_IMR, UINT32_MAX);
	/* No high-frequency sampling delay at the conservative 24 MHz rate. */
	mmio_write_32(SFC_BASE + SFC_DLL_CTRL, 0U);
	return fspi_reset();
}

/* Single-line commands, with an optional 24-bit address and receive data. */
static int fspi_exec(uint32_t command, uint32_t address, void *buffer,
		     size_t length)
{
	uint8_t *out = buffer;
	size_t remaining = length;
	uint64_t timeout;
	int ret;

	if ((length > SFC_MAX_TRANSFER) ||
	    ((length != 0U) && (buffer == NULL))) {
		return -EINVAL;
	}

	if (((mmio_read_32(SFC_BASE + SFC_FSR) & SFC_FIFO_EMPTY) !=
	     SFC_FIFO_EMPTY) ||
	    ((mmio_read_32(SFC_BASE + SFC_SR) & SFC_BUSY) != 0U)) {
		ret = fspi_reset();
		if (ret != 0) {
			return ret;
		}
	}

	mmio_write_32(SFC_BASE + SFC_ICLR, UINT32_MAX);
	mmio_write_32(SFC_BASE + SFC_CTRL, SFC_PHASE_NEGATIVE);
	mmio_write_32(SFC_BASE + SFC_CMD,
		      command | ((uint32_t)length << SFC_CMD_LENGTH_SHIFT));
	if ((command & SFC_CMD_ADDR_24) != 0U) {
		mmio_write_32(SFC_BASE + SFC_ADDR, address);
	}

	timeout = timeout_init_us(SFC_FIFO_TIMEOUT_US);
	while (remaining != 0U) {
		uint32_t level;

		if ((mmio_read_32(SFC_BASE + SFC_RISR) & SFC_ERRORS) != 0U) {
			ret = -EIO;
			goto fail;
		}
		level = (mmio_read_32(SFC_BASE + SFC_FSR) >>
			 SFC_RX_LEVEL_SHIFT) & SFC_RX_LEVEL_MASK;
		if (level == 0U) {
			if (timeout_elapsed(timeout)) {
				ret = -ETIMEDOUT;
				goto fail;
			}
			udelay(1);
			continue;
		}

		while ((level-- != 0U) && (remaining != 0U)) {
			uint32_t word = mmio_read_32(SFC_BASE + SFC_DATA);
			size_t count = MIN(remaining, sizeof(word));

			/* Support unaligned destinations and a final 1..3 bytes. */
			for (size_t i = 0U; i < count; i++) {
				*out++ = (uint8_t)(word >> (i * 8U));
			}
			remaining -= count;
		}
		timeout = timeout_init_us(SFC_FIFO_TIMEOUT_US);
	}

	ret = fspi_wait_clear(SFC_SR, SFC_BUSY, SFC_BUSY_TIMEOUT_US);
	if (ret != 0) {
		goto fail;
	}
	if ((mmio_read_32(SFC_BASE + SFC_RISR) & SFC_ERRORS) != 0U) {
		ret = -EIO;
		goto fail;
	}
	udelay(1);
	return 0;

fail:
	(void)fspi_reset();
	return ret;
}

static int nor_wait_ready(void)
{
	uint64_t timeout = timeout_init_us(NOR_READY_TIMEOUT_US);
	uint8_t status;
	int ret;

	do {
		ret = fspi_exec(NOR_READ_STATUS, 0U, &status, sizeof(status));
		if (ret != 0) {
			return ret;
		}
		if ((status & NOR_STATUS_BUSY) == 0U) {
			return 0;
		}
		udelay(1);
	} while (!timeout_elapsed(timeout));

	return -ETIMEDOUT;
}

int qrk3588_fspi_nor_init(void)
{
	uint8_t id[3];
	int ret;

	nor_capacity = 0U;
	ret = fspi_controller_init();
	if (ret != 0) {
		return ret;
	}
	ret = fspi_exec(NOR_RESET_ENABLE, 0U, NULL, 0U);
	if (ret != 0) {
		return ret;
	}
	ret = fspi_exec(NOR_RESET, 0U, NULL, 0U);
	if (ret != 0) {
		return ret;
	}
	udelay(200);

	ret = fspi_exec(NOR_READ_ID, 0U, id, sizeof(id));
	if (ret != 0) {
		return ret;
	}
	if ((id[0] == 0U) || (id[0] == U(0xff)) ||
	    (id[1] == 0U) || (id[1] == U(0xff))) {
		return -ENODEV;
	}
	/* Reject continuation IDs and nonstandard density/addressing schemes. */
	if ((id[0] == U(0x7f)) || (id[2] < U(16)) ||
	    (id[2] > NOR_MAX_CAPACITY_LOG2)) {
		WARN("FSPI NOR: unsupported JEDEC ID %02x %02x %02x\n",
		     id[0], id[1], id[2]);
		return -EOPNOTSUPP;
	}
	ret = nor_wait_ready();
	if (ret != 0) {
		return ret;
	}
	nor_capacity = U(1) << id[2];
	INFO("FSPI NOR: JEDEC %02x %02x %02x, %u bytes, M%u\n",
	     id[0], id[1], id[2], nor_capacity, QRK3588_FSPI_IOMUX);
	return 0;
}

size_t qrk3588_fspi_nor_read(int lba, uintptr_t buffer, size_t size)
{
	uint64_t address;
	size_t remaining = size;
	uint8_t *out = (uint8_t *)buffer;

	if ((nor_capacity == 0U) || (lba < 0) || (buffer == 0U) ||
	    (size > UINTPTR_MAX - buffer)) {
		return 0U;
	}
	address = (uint64_t)(unsigned int)lba * NOR_SECTOR_SIZE;
	if ((address > nor_capacity) || (size > nor_capacity - address)) {
		return 0U;
	}

	while (remaining != 0U) {
		size_t count = MIN(remaining, (size_t)SFC_MAX_TRANSFER);
		int ret = fspi_exec(NOR_READ | SFC_CMD_ADDR_24,
				    (uint32_t)address, out, count);

		if (ret != 0) {
			ERROR("FSPI NOR: read at 0x%x failed (%d)\n",
			      (uint32_t)address, ret);
			return 0U;
		}
		address += count;
		out += count;
		remaining -= count;
	}

	return size;
}
