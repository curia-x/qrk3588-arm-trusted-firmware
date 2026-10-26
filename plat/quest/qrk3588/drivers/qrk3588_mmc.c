/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/mmc.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>

#include <qrk3588_mmc.h>

#define EMMC_BASE			UL(0xfe2e0000)
#define SDMMC_BASE			UL(0xfe2c0000)
#define EMMC_CLKSEL			UL(0xfd7c0434)
#define SDMMC_CLKSEL			UL(0xfd7d030c)
#define EMMC_CLKGATE			UL(0xfd7c087c)
#define EMMC_SOFTRST			UL(0xfd7c0a7c)

#define CMD_TIMEOUT_US			U(100000)
#define DATA_TIMEOUT_US			U(500000)
#define RESET_TIMEOUT_US			U(20000)
#define CARD_INIT_RETRIES		U(2000)

#define CARD_R1_ERRORS			(GENMASK_32(31U, 19U) | \
					 BIT_32(16U) | STATUS_SWITCH_ERROR)

#define SDHCI_BLOCK_SIZE			U(0x04)
#define SDHCI_BLOCK_COUNT		U(0x06)
#define SDHCI_ARGUMENT			U(0x08)
#define SDHCI_TRANSFER_MODE		U(0x0c)
#define SDHCI_COMMAND			U(0x0e)
#define SDHCI_RESPONSE			U(0x10)
#define SDHCI_BUFFER			U(0x20)
#define SDHCI_PRESENT			U(0x24)
#define SDHCI_HOST_CONTROL		U(0x28)
#define SDHCI_POWER_CONTROL		U(0x29)
#define SDHCI_CLOCK_CONTROL		U(0x2c)
#define SDHCI_TIMEOUT_CONTROL		U(0x2e)
#define SDHCI_RESET			U(0x2f)
#define SDHCI_INT_STATUS			U(0x30)
#define SDHCI_INT_ENABLE			U(0x34)
#define SDHCI_SIGNAL_ENABLE		U(0x38)
#define SDHCI_HOST_CONTROL2		U(0x3e)
#define SDHCI_CMD_INHIBIT			BIT_32(0)
#define SDHCI_DATA_INHIBIT		BIT_32(1)
#define SDHCI_DATA_AVAILABLE		BIT_32(11)
#define SDHCI_SPACE_AVAILABLE		BIT_32(10)
#define SDHCI_INT_CMD_DONE		BIT_32(0)
#define SDHCI_INT_DATA_DONE		BIT_32(1)
#define SDHCI_INT_READ_READY		BIT_32(5)
#define SDHCI_INT_WRITE_READY		BIT_32(4)
#define SDHCI_INT_ERRORS			U(0xffff8000)
#define SDHCI_INT_TIMEOUT		BIT_32(16)
#define SDHCI_CMD_DATA			BIT_32(5)
#define SDHCI_CMD_R1			U(0x1a)
#define SDHCI_CMD_R1B			U(0x1b)
#define SDHCI_CMD_R2			U(0x09)
#define SDHCI_CMD_R3			U(0x02)
#define SDHCI_CMD(index, flags)		(((index) << 8) | (flags))

#define DW_CTRL				U(0x00)
#define DW_PWREN				U(0x04)
#define DW_CLKDIV			U(0x08)
#define DW_CLKSRC			U(0x0c)
#define DW_CLKENA			U(0x10)
#define DW_TMOUT				U(0x14)
#define DW_CTYPE				U(0x18)
#define DW_BLKSIZ			U(0x1c)
#define DW_BYTCNT			U(0x20)
#define DW_INTMASK			U(0x24)
#define DW_CMDARG			U(0x28)
#define DW_CMD				U(0x2c)
#define DW_RESP0				U(0x30)
#define DW_RINTSTS			U(0x44)
#define DW_STATUS			U(0x48)
#define DW_FIFOTH			U(0x4c)
#define DW_BMOD				U(0x80)
#define DW_FIFO				U(0x200)
#define DW_FIFO_RESET			BIT_32(1)
#define DW_FIFO_EMPTY			BIT_32(2)
#define DW_CARD_BUSY			BIT_32(9)
#define DW_CMD_START			BIT_32(31)
#define DW_CMD_HOLD			BIT_32(29)
#define DW_CMD_INIT			BIT_32(15)
#define DW_CMD_WAIT_DATA			BIT_32(13)
#define DW_CMD_DATA			BIT_32(9)
#define DW_CMD_CRC			BIT_32(8)
#define DW_CMD_LONG_RESPONSE		BIT_32(7)
#define DW_CMD_RESPONSE			BIT_32(6)
#define DW_CMD_R1			(DW_CMD_RESPONSE | DW_CMD_CRC)
#define DW_INT_CMD_DONE			BIT_32(2)
#define DW_INT_DATA_DONE			BIT_32(3)
#define DW_INT_RX_READY			BIT_32(5)
#define DW_INT_RESPONSE_TIMEOUT		BIT_32(8)
#define DW_INT_CMD_ERRORS		(BIT_32(1) | BIT_32(6) | BIT_32(8) | \
					 BIT_32(12))
#define DW_INT_DATA_ERRORS		(BIT_32(7) | BIT_32(9) | BIT_32(11) | \
					 BIT_32(13) | BIT_32(15))
#define DW_UPDATE_CLOCK			U(0x80202000)

#define EMMC_HANDOFF_VALID		(UINT64_C(1) << 63)
#define EMMC_HANDOFF_SECTOR		(UINT64_C(1) << 16)
#define EMMC_HANDOFF_RCA_MASK		UINT64_C(0xffff)
#define EMMC_HANDOFF_ALLOWED		(EMMC_HANDOFF_VALID | \
					 EMMC_HANDOFF_SECTOR | \
					 EMMC_HANDOFF_RCA_MASK)

struct qrk3588_card {
	bool ready;
	bool sector_addressed;
	uint16_t rca;
};

static struct qrk3588_card emmc_card;
static struct qrk3588_card sd_card;
static uint8_t emmc_ext_csd[MMC_BLOCK_SIZE];
static bool sdhci_recovery_enabled = true;

static int wait32(uintptr_t address, uint32_t mask, uint32_t value,
		  unsigned int timeout)
{
	while (timeout-- != 0U) {
		if ((mmio_read_32(address) & mask) == value) {
			return 0;
		}
		udelay(1);
	}

	return -ETIMEDOUT;
}

static int sdhci_reset(uint8_t mask)
{
	mmio_write_8(EMMC_BASE + SDHCI_RESET, mask);
	for (unsigned int i = 0U; i < RESET_TIMEOUT_US; i++) {
		if ((mmio_read_8(EMMC_BASE + SDHCI_RESET) & mask) == 0U) {
			return 0;
		}
		udelay(1);
	}

	return -ETIMEDOUT;
}

static int sdhci_clock(bool identify)
{
	mmio_write_16(EMMC_BASE + SDHCI_CLOCK_CONTROL, 0U);
	udelay(10);
	/* CRU CLKSEL_CON77: OSC24M /64 for identification, /1 for data. */
	mmio_write_32(EMMC_CLKSEL, identify ? U(0xff00bf00) : U(0xff008000));
	mmio_write_16(EMMC_BASE + SDHCI_CLOCK_CONTROL, 1U);
	for (unsigned int i = 0U; i < RESET_TIMEOUT_US; i++) {
		uint16_t clock = mmio_read_16(EMMC_BASE + SDHCI_CLOCK_CONTROL);

		if ((clock & BIT(1)) != 0U) {
			mmio_write_16(EMMC_BASE + SDHCI_CLOCK_CONTROL,
				      clock | BIT(2));
			udelay(500);
			return 0;
		}
		udelay(1);
	}

	return -ETIMEDOUT;
}

static int sdhci_wait_irq(uint32_t mask)
{
	for (unsigned int i = 0U; i < DATA_TIMEOUT_US; i++) {
		uint32_t status = mmio_read_32(EMMC_BASE + SDHCI_INT_STATUS);

		if ((status & SDHCI_INT_ERRORS) != 0U) {
			return (status & SDHCI_INT_TIMEOUT) != 0U ?
				-ETIMEDOUT : -EIO;
		}
		if ((status & mask) != 0U) {
			return 0;
		}
		udelay(1);
	}

	return -ETIMEDOUT;
}

/* data is either NULL or exactly one 512-byte receive block. */
static int sdhci_command(uint16_t command, uint32_t argument,
			 uint32_t *response, uint8_t *data)
{
	uint32_t inhibit = SDHCI_CMD_INHIBIT;
	uint32_t reply;
	int ret;

	if ((data != NULL) || ((command & 3U) == 3U)) {
		inhibit |= SDHCI_DATA_INHIBIT;
	}
	ret = wait32(EMMC_BASE + SDHCI_PRESENT, inhibit, 0U, DATA_TIMEOUT_US);
	if (ret != 0) {
		goto fail;
	}

	mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, UINT32_MAX);
	if (data != NULL) {
		mmio_write_16(EMMC_BASE + SDHCI_BLOCK_SIZE, MMC_BLOCK_SIZE);
		mmio_write_16(EMMC_BASE + SDHCI_BLOCK_COUNT, 1U);
		/* Read direction, block-count enable; DMA and multi-block disabled. */
		mmio_write_16(EMMC_BASE + SDHCI_TRANSFER_MODE, U(0x12));
	} else {
		mmio_write_16(EMMC_BASE + SDHCI_TRANSFER_MODE, 0U);
	}
	mmio_write_32(EMMC_BASE + SDHCI_ARGUMENT, argument);
	mmio_write_16(EMMC_BASE + SDHCI_COMMAND, command);
	ret = sdhci_wait_irq(SDHCI_INT_CMD_DONE);
	if (ret != 0) {
		goto fail;
	}
	mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, SDHCI_INT_CMD_DONE);
	reply = mmio_read_32(EMMC_BASE + SDHCI_RESPONSE);
	if (response != NULL) {
		*response = reply;
	}
	/* R1/R1b only: OCR, CID and RCA responses have different layouts. */
	if (((command & U(0x1a)) == SDHCI_CMD_R1) &&
	    ((reply & CARD_R1_ERRORS) != 0U)) {
		ret = -EIO;
		goto fail;
	}

	if (data != NULL) {
		ret = sdhci_wait_irq(SDHCI_INT_READ_READY);
		if (ret != 0) {
			goto fail;
		}
		ret = wait32(EMMC_BASE + SDHCI_PRESENT, SDHCI_DATA_AVAILABLE,
			     SDHCI_DATA_AVAILABLE, DATA_TIMEOUT_US);
		if (ret != 0) {
			goto fail;
		}
		mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, SDHCI_INT_READ_READY);
		for (size_t i = 0U; i < MMC_BLOCK_SIZE; i += sizeof(uint32_t)) {
			uint32_t word = mmio_read_32(EMMC_BASE + SDHCI_BUFFER);

			memcpy(data + i, &word, sizeof(word));
		}
		ret = sdhci_wait_irq(SDHCI_INT_DATA_DONE);
		if (ret != 0) {
			goto fail;
		}
	}
	if ((data != NULL) || ((command & 3U) == 3U)) {
		ret = wait32(EMMC_BASE + SDHCI_PRESENT, SDHCI_DATA_INHIBIT,
			     0U, DATA_TIMEOUT_US);
		if (ret != 0) {
			goto fail;
		}
	}
	mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, UINT32_MAX);
	return 0;

fail:
	VERBOSE("eMMC: CMD%u failed (%d), irq=0x%x\n", command >> 8,
		ret, mmio_read_32(EMMC_BASE + SDHCI_INT_STATUS));
	mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, UINT32_MAX);
	if (sdhci_recovery_enabled) {
		(void)sdhci_reset(6U);
	}
	return ret;
}

static int emmc_select_user_area(void)
{
	uint32_t status;
	uint32_t argument;
	int ret;

	ret = sdhci_command(SDHCI_CMD(8U, SDHCI_CMD_R1 | SDHCI_CMD_DATA),
			    0U, NULL, emmc_ext_csd);
	if (ret != 0) {
		return ret;
	}
	/* Preserve BOOT_ACK and BOOT_PARTITION_ENABLE; change only access. */
	argument = EXTCSD_WRITE_BYTES | EXTCSD_CMD(CMD_EXTCSD_PARTITION_CONFIG) |
		   EXTCSD_VALUE(emmc_ext_csd[CMD_EXTCSD_PARTITION_CONFIG] &
				~EXT_CSD_PART_CONFIG_ACC_MASK);
	ret = sdhci_command(SDHCI_CMD(6U, SDHCI_CMD_R1B), argument, NULL, NULL);
	if (ret != 0) {
		return ret;
	}
	for (unsigned int i = 0U; i < CARD_INIT_RETRIES; i++) {
		ret = sdhci_command(SDHCI_CMD(13U, SDHCI_CMD_R1),
				    (uint32_t)emmc_card.rca << 16, &status, NULL);
		if (ret != 0) {
			return ret;
		}
		if (((status & STATUS_READY_FOR_DATA) != 0U) &&
		    (MMC_GET_STATE(status) == MMC_STATE_TRAN)) {
			return 0;
		}
		udelay(1000);
	}
	return -ETIMEDOUT;
}

int qrk3588_emmc_init(void)
{
	uint32_t ocr = 0U;
	int ret;

	emmc_card.ready = false;
	emmc_card.rca = 2U;
	/* Enable the NVM/eMMC clocks and release the eMMC reset domain. */
	mmio_write_32(EMMC_CLKGATE, U(0x01f70000));
	mmio_write_32(EMMC_SOFTRST, U(0x01f001f0));
	udelay(2);
	mmio_write_32(EMMC_SOFTRST, U(0x01f00000));
	mmio_write_32(EMMC_CLKSEL, U(0xff00bf00));
	mmio_write_32(UL(0xfd5f8040), U(0xffff1111));
	mmio_write_32(UL(0xfd5f8058), U(0xffff1111));
	mmio_write_32(UL(0xfd5f805c), U(0xffff1111));
	mmio_clrbits_32(UL(0xfe0380a8), U(0xffff0000));
	mmio_clrbits_32(UL(0xfe0300a8), U(0xffff0000));
	ret = sdhci_reset(1U);
	if (ret != 0) {
		return ret;
	}
	/* Bypass the sampling DLL while the controller uses the legacy clock. */
	mmio_write_32(EMMC_BASE + U(0x800), BIT_32(24) | BIT_32(0));
	mmio_write_32(EMMC_BASE + U(0x804), BIT_32(31));
	mmio_write_32(EMMC_BASE + U(0x808), 0U);
	mmio_write_32(EMMC_BASE + U(0x810), 0U);
	mmio_write_32(EMMC_BASE + U(0x80c), U(0x0c100000));
	mmio_write_8(EMMC_BASE + SDHCI_HOST_CONTROL, 0U);
	mmio_write_16(EMMC_BASE + SDHCI_HOST_CONTROL2, 0U);
	mmio_write_8(EMMC_BASE + SDHCI_POWER_CONTROL, U(0x0b));
	udelay(1000);
	ret = sdhci_clock(true);
	if (ret != 0) {
		return ret;
	}
	mmio_write_8(EMMC_BASE + SDHCI_TIMEOUT_CONTROL, U(0x0e));
	mmio_write_32(EMMC_BASE + SDHCI_INT_ENABLE, U(0x03ff8023));
	mmio_write_32(EMMC_BASE + SDHCI_SIGNAL_ENABLE, 0U);
	ret = sdhci_command(SDHCI_CMD(0U, 0U), 0U, NULL, NULL);
	if (ret != 0) {
		return ret;
	}
	udelay(100);
	for (unsigned int i = 0U; i < CARD_INIT_RETRIES; i++) {
		ret = sdhci_command(SDHCI_CMD(1U, SDHCI_CMD_R3),
				    OCR_HCS | OCR_VDD_MIN_2V7, &ocr, NULL);
		if (ret != 0) {
			return ret;
		}
		if ((ocr & OCR_POWERUP) != 0U) {
			break;
		}
		udelay(1000);
	}
	if ((ocr & OCR_POWERUP) == 0U) {
		return -ETIMEDOUT;
	}
	if (((ocr & OCR_ACCESS_MODE_MASK) != OCR_BYTE_MODE) &&
	    ((ocr & OCR_ACCESS_MODE_MASK) != OCR_SECTOR_MODE)) {
		return -EIO;
	}
	emmc_card.sector_addressed = (ocr & OCR_ACCESS_MODE_MASK) == OCR_SECTOR_MODE;
	ret = sdhci_command(SDHCI_CMD(2U, SDHCI_CMD_R2), 0U, NULL, NULL);
	if (ret == 0) {
		ret = sdhci_command(SDHCI_CMD(3U, SDHCI_CMD_R1),
				    (uint32_t)emmc_card.rca << 16, NULL, NULL);
	}
	if (ret == 0) {
		ret = sdhci_clock(false);
	}
	if (ret == 0) {
		ret = sdhci_command(SDHCI_CMD(7U, SDHCI_CMD_R1B),
				    (uint32_t)emmc_card.rca << 16, NULL, NULL);
	}
	if (ret == 0) {
		ret = sdhci_command(SDHCI_CMD(16U, SDHCI_CMD_R1),
				    MMC_BLOCK_SIZE, NULL, NULL);
	}
	if (ret == 0) {
		ret = emmc_select_user_area();
	}
	emmc_card.ready = ret == 0;
	return ret;
}

uint64_t qrk3588_emmc_handoff_token(void)
{
	if (!emmc_card.ready)
		return 0U;

	return EMMC_HANDOFF_VALID |
		(uint64_t)emmc_card.rca |
		(emmc_card.sector_addressed ? EMMC_HANDOFF_SECTOR : 0U);
}

int qrk3588_emmc_handoff(uint64_t token)
{
	uint32_t status;
	int ret;

	if (((token & EMMC_HANDOFF_VALID) == 0U) ||
	    ((token & ~EMMC_HANDOFF_ALLOWED) != 0U) ||
	    ((token & EMMC_HANDOFF_RCA_MASK) == 0U)) {
		return -ENODEV;
	}

	emmc_card.ready = false;
	emmc_card.rca = (uint16_t)(token & EMMC_HANDOFF_RCA_MASK);
	emmc_card.sector_addressed = (token & EMMC_HANDOFF_SECTOR) != 0U;

	/* BL1 has already selected the user area and configured 512-byte blocks. */
	sdhci_recovery_enabled = false;
	ret = sdhci_command(SDHCI_CMD(13U, SDHCI_CMD_R1),
			    (uint32_t)emmc_card.rca << 16, &status, NULL);
	sdhci_recovery_enabled = true;
	if ((ret != 0) || ((status & STATUS_READY_FOR_DATA) == 0U) ||
	    (MMC_GET_STATE(status) != MMC_STATE_TRAN)) {
		return ret != 0 ? ret : -EIO;
	}

	emmc_card.ready = true;
	return 0;
}

static int dw_update_clock(void)
{
	mmio_write_32(SDMMC_BASE + DW_CMD, DW_UPDATE_CLOCK);
	return wait32(SDMMC_BASE + DW_CMD, DW_CMD_START, 0U, CMD_TIMEOUT_US);
}

static int dw_reset_fifo(void)
{
	mmio_setbits_32(SDMMC_BASE + DW_CTRL, DW_FIFO_RESET);
	return wait32(SDMMC_BASE + DW_CTRL, DW_FIFO_RESET, 0U, RESET_TIMEOUT_US);
}

static int dw_command(uint32_t command, uint32_t argument,
		      uint32_t *response, uint8_t *data, bool busy)
{
	unsigned int remaining;
	uint32_t status;
	int ret;

	ret = wait32(SDMMC_BASE + DW_STATUS, DW_CARD_BUSY, 0U, DATA_TIMEOUT_US);
	if (ret == 0) {
		ret = wait32(SDMMC_BASE + DW_CMD, DW_CMD_START, 0U, CMD_TIMEOUT_US);
	}
	if ((ret == 0) && (data != NULL)) {
		ret = dw_reset_fifo();
	}
	if (ret != 0) {
		return ret;
	}
	mmio_write_32(SDMMC_BASE + DW_RINTSTS, UINT32_MAX);
	if (data != NULL) {
		mmio_write_32(SDMMC_BASE + DW_BLKSIZ, MMC_BLOCK_SIZE);
		mmio_write_32(SDMMC_BASE + DW_BYTCNT, MMC_BLOCK_SIZE);
	}
	mmio_write_32(SDMMC_BASE + DW_CMDARG, argument);
	mmio_write_32(SDMMC_BASE + DW_CMD, command | DW_CMD_START | DW_CMD_HOLD);
	remaining = CMD_TIMEOUT_US;
	do {
		status = mmio_read_32(SDMMC_BASE + DW_RINTSTS);
		if ((status & DW_INT_CMD_ERRORS) != 0U) {
			ret = (status & DW_INT_RESPONSE_TIMEOUT) != 0U ?
				-ETIMEDOUT : -EIO;
			goto fail;
		}
		if ((status & DW_INT_CMD_DONE) != 0U) {
			break;
		}
		udelay(1);
	} while (--remaining != 0U);
	if (remaining == 0U) {
		ret = -ETIMEDOUT;
		goto fail;
	}
	mmio_write_32(SDMMC_BASE + DW_RINTSTS, DW_INT_CMD_DONE);
	status = mmio_read_32(SDMMC_BASE + DW_RESP0);
	if (response != NULL) {
		*response = status;
	}
	if ((data != NULL) && ((status & CARD_R1_ERRORS) != 0U)) {
		ret = -EIO;
		goto fail;
	}
	if (data != NULL) {
		size_t moved = 0U;

		remaining = DATA_TIMEOUT_US;
		do {
			status = mmio_read_32(SDMMC_BASE + DW_RINTSTS);
			if ((status & (DW_INT_CMD_ERRORS | DW_INT_DATA_ERRORS)) != 0U) {
				ret = -EIO;
				goto fail;
			}
			if ((moved < MMC_BLOCK_SIZE) &&
			    ((mmio_read_32(SDMMC_BASE + DW_STATUS) & DW_FIFO_EMPTY) == 0U)) {
				uint32_t word = mmio_read_32(SDMMC_BASE + DW_FIFO);

				memcpy(data + moved, &word, sizeof(word));
				moved += sizeof(word);
				mmio_write_32(SDMMC_BASE + DW_RINTSTS, DW_INT_RX_READY);
				continue;
			}
			if ((moved == MMC_BLOCK_SIZE) &&
			    ((status & DW_INT_DATA_DONE) != 0U)) {
				break;
			}
			udelay(1);
		} while (--remaining != 0U);
		if (remaining == 0U) {
			ret = -ETIMEDOUT;
			goto fail;
		}
	}
	if (busy || (data != NULL)) {
		ret = wait32(SDMMC_BASE + DW_STATUS, DW_CARD_BUSY,
			     0U, DATA_TIMEOUT_US);
		if (ret != 0) {
			goto fail;
		}
	}
	mmio_write_32(SDMMC_BASE + DW_RINTSTS, UINT32_MAX);
	return 0;

fail:
	VERBOSE("SD: CMD%u failed (%d), irq=0x%x\n", command & U(0x3f),
		ret, mmio_read_32(SDMMC_BASE + DW_RINTSTS));
	mmio_write_32(SDMMC_BASE + DW_RINTSTS, UINT32_MAX);
	(void)dw_reset_fifo();
	return ret;
}

static int sd_acmd41(uint32_t argument, uint32_t *response)
{
	uint32_t status;
	int ret;

	ret = dw_command(55U | DW_CMD_R1 | DW_CMD_WAIT_DATA,
			 (uint32_t)sd_card.rca << 16, &status, NULL, false);
	if (ret != 0) {
		return ret;
	}
	if (((status & CARD_R1_ERRORS) != 0U) || ((status & BIT(5)) == 0U)) {
		return -EIO;
	}
	return dw_command(41U | DW_CMD_RESPONSE | DW_CMD_WAIT_DATA,
			  argument, response, NULL, false);
}

int qrk3588_sdmmc_init(void)
{
	uint32_t response = 0U;
	uint32_t ocr = 0U;
	bool v2;
	int ret;

	sd_card.ready = false;
	sd_card.rca = 0U;
	mmio_write_32(SDMMC_CLKSEL, U(0x3fc027c0));
	mmio_write_32(UL(0xfd58a3b0), 2U);
	mmio_write_32(UL(0xfd5f0004), U(0x00ff0021));
	mmio_write_32(UL(0xfd5f8098), U(0xffff1111));
	mmio_write_32(UL(0xfd5f809c), U(0x00ff0011));
	udelay(2);
	/* Board power is external; only enable the controller-side power state. */
	mmio_write_32(SDMMC_BASE + DW_PWREN, 0U);
	udelay(1000);
	mmio_write_32(SDMMC_BASE + DW_CTRL, 7U);
	ret = wait32(SDMMC_BASE + DW_CTRL, 7U, 0U, RESET_TIMEOUT_US);
	if (ret != 0) {
		return ret;
	}
	mmio_write_32(SDMMC_BASE + DW_CTRL, 0U);
	mmio_write_32(SDMMC_BASE + DW_BMOD, 0U);
	mmio_write_32(SDMMC_BASE + DW_INTMASK, 0U);
	mmio_write_32(SDMMC_BASE + DW_CTYPE, 0U);
	mmio_write_32(SDMMC_BASE + DW_CLKSRC, 0U);
	mmio_write_32(SDMMC_BASE + DW_CLKDIV, 0U);
	mmio_write_32(SDMMC_BASE + DW_CLKENA, 1U);
	ret = dw_update_clock();
	if (ret != 0) {
		return ret;
	}
	udelay(500);
	mmio_write_32(SDMMC_BASE + DW_TMOUT, U(0xffffff40));
	mmio_write_32(SDMMC_BASE + DW_FIFOTH, U(0x307f0080));
	ret = dw_command(DW_CMD_INIT, 0U, NULL, NULL, false);
	if (ret != 0) {
		return ret;
	}
	udelay(100);
	ret = dw_command(8U | DW_CMD_R1 | DW_CMD_WAIT_DATA,
			 VHS_2_7_3_6_V | CMD8_CHECK_PATTERN, &response, NULL, false);
	v2 = ret == 0;
	if (v2 && ((response & U(0xfff)) != (VHS_2_7_3_6_V | CMD8_CHECK_PATTERN))) {
		return -EIO;
	}
	if (!v2 && (ret != -ETIMEDOUT)) {
		return ret;
	}
	for (unsigned int i = 0U; i < CARD_INIT_RETRIES; i++) {
		ret = sd_acmd41(OCR_VDD_MIN_2V7 | (v2 ? OCR_HCS : 0U), &ocr);
		if (ret != 0) {
			return ret;
		}
		if ((ocr & OCR_POWERUP) != 0U) {
			break;
		}
		udelay(1000);
	}
	if ((ocr & OCR_POWERUP) == 0U) {
		return -ETIMEDOUT;
	}
	sd_card.sector_addressed = v2 && ((ocr & OCR_HCS) != 0U);
	ret = dw_command(2U | DW_CMD_R1 | DW_CMD_LONG_RESPONSE | DW_CMD_WAIT_DATA,
			 0U, NULL, NULL, false);
	if (ret == 0) {
		ret = dw_command(3U | DW_CMD_R1 | DW_CMD_WAIT_DATA,
				 0U, &response, NULL, false);
	}
	if (ret != 0) {
		return ret;
	}
	if ((response & U(0xe000)) != 0U) {
		return -EIO;
	}
	sd_card.rca = (uint16_t)(response >> 16);
	if (sd_card.rca == 0U) {
		return -EIO;
	}
	mmio_write_32(SDMMC_BASE + DW_CLKENA, 0U);
	ret = dw_update_clock();
	if (ret != 0) {
		return ret;
	}
	mmio_write_32(SDMMC_CLKSEL, U(0x3fc02000));
	mmio_write_32(SDMMC_BASE + DW_CLKDIV, 0U);
	ret = dw_update_clock();
	if (ret != 0) {
		return ret;
	}
	mmio_write_32(SDMMC_BASE + DW_CLKENA, 1U);
	ret = dw_update_clock();
	if (ret == 0) {
		ret = dw_command(7U | DW_CMD_R1 | DW_CMD_WAIT_DATA,
				 (uint32_t)sd_card.rca << 16, &response, NULL, true);
	}
	if ((ret == 0) && ((response & CARD_R1_ERRORS) != 0U)) {
		ret = -EIO;
	}
	if (ret == 0) {
		ret = dw_command(16U | DW_CMD_R1 | DW_CMD_WAIT_DATA,
				 MMC_BLOCK_SIZE, &response, NULL, false);
	}
	if ((ret == 0) && ((response & CARD_R1_ERRORS) != 0U)) {
		ret = -EIO;
	}
	sd_card.ready = ret == 0;
	return ret;
}

static size_t card_read(struct qrk3588_card *card, int lba,
			uintptr_t buffer, size_t size, bool emmc)
{
	uint64_t last_lba;

	if (!card->ready || (lba < 0) || (buffer == 0U) || (size == 0U) ||
	    ((size & MMC_BLOCK_MASK) != 0U) || (buffer > UINTPTR_MAX - size)) {
		return 0U;
	}
	last_lba = (uint64_t)(unsigned int)lba + size / MMC_BLOCK_SIZE - 1U;
	if (last_lba > (card->sector_addressed ? UINT32_MAX :
		       UINT32_MAX / MMC_BLOCK_SIZE)) {
		return 0U;
	}
	for (size_t offset = 0U; offset < size; offset += MMC_BLOCK_SIZE) {
		uint32_t argument = (uint32_t)lba + offset / MMC_BLOCK_SIZE;
		int ret;

		if (!card->sector_addressed) {
			argument *= MMC_BLOCK_SIZE;
		}
		if (emmc) {
			ret = sdhci_command(SDHCI_CMD(17U, SDHCI_CMD_R1 | SDHCI_CMD_DATA),
					    argument, NULL, (uint8_t *)(buffer + offset));
		} else {
			ret = dw_command(17U | DW_CMD_R1 | DW_CMD_DATA | DW_CMD_WAIT_DATA,
					 argument, NULL, (uint8_t *)(buffer + offset), false);
		}
		if (ret != 0) {
			ERROR("%s: read LBA %u failed (%d)\n", emmc ? "eMMC" : "SD",
			      (unsigned int)((uint32_t)lba + offset / MMC_BLOCK_SIZE), ret);
			card->ready = false;
			return 0U;
		}
	}
	return size;
}

/*
 * The boot path only needs single-block PIO writes.  Keeping this separate
 * from the read helper avoids changing the established read command path.
 */
static int emmc_write_block(uint32_t argument, const uint8_t *data)
{
	const char *stage = "idle";
	uint32_t response;
	int ret;

	stage = "wait-idle";
	ret = wait32(EMMC_BASE + SDHCI_PRESENT,
		     SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT, 0U, DATA_TIMEOUT_US);
	if (ret != 0)
		goto fail;

	stage = "command";
	mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, UINT32_MAX);
	mmio_write_16(EMMC_BASE + SDHCI_BLOCK_SIZE, MMC_BLOCK_SIZE);
	mmio_write_16(EMMC_BASE + SDHCI_BLOCK_COUNT, 1U);
	/* Block-count enable, PIO write direction. */
	mmio_write_16(EMMC_BASE + SDHCI_TRANSFER_MODE, U(0x02));
	mmio_write_32(EMMC_BASE + SDHCI_ARGUMENT, argument);
	mmio_write_16(EMMC_BASE + SDHCI_COMMAND,
		      SDHCI_CMD(24U, SDHCI_CMD_R1 | SDHCI_CMD_DATA));
	ret = sdhci_wait_irq(SDHCI_INT_CMD_DONE);
	if (ret != 0)
		goto fail;
	mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, SDHCI_INT_CMD_DONE);
	response = mmio_read_32(EMMC_BASE + SDHCI_RESPONSE);
	if ((response & CARD_R1_ERRORS) != 0U) {
		ret = -EIO;
		goto fail;
	}

	/* RK3588 exposes write-buffer readiness through PRESENT, but does not
	 * raise INT_WRITE_READY for this PIO CMD24 transfer. */
	stage = "write-space";
	ret = wait32(EMMC_BASE + SDHCI_PRESENT, SDHCI_SPACE_AVAILABLE,
		     SDHCI_SPACE_AVAILABLE, DATA_TIMEOUT_US);
	if (ret != 0)
		goto fail;
	for (size_t i = 0U; i < MMC_BLOCK_SIZE; i += sizeof(uint32_t)) {
		uint32_t word;

		memcpy(&word, data + i, sizeof(word));
		mmio_write_32(EMMC_BASE + SDHCI_BUFFER, word);
	}
	stage = "data-done";
	ret = sdhci_wait_irq(SDHCI_INT_DATA_DONE);
	if (ret != 0)
		goto fail;
	stage = "wait-data-idle";
	ret = wait32(EMMC_BASE + SDHCI_PRESENT, SDHCI_DATA_INHIBIT, 0U,
		     DATA_TIMEOUT_US);
	if (ret != 0)
		goto fail;
	mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, UINT32_MAX);
	return 0;

fail:
	ERROR("eMMC: CMD24 %s failed (%d), present=0x%x irq=0x%x\n", stage,
	      ret, mmio_read_32(EMMC_BASE + SDHCI_PRESENT),
	      mmio_read_32(EMMC_BASE + SDHCI_INT_STATUS));
	mmio_write_32(EMMC_BASE + SDHCI_INT_STATUS, UINT32_MAX);
	(void)sdhci_reset(6U);
	return ret;
}

size_t qrk3588_emmc_write(int lba, uintptr_t buffer, size_t size)
{
	uint64_t last_lba;

	if (!emmc_card.ready || (lba < 0) || (buffer == 0U) || (size == 0U) ||
	    ((size & MMC_BLOCK_MASK) != 0U) || (buffer > UINTPTR_MAX - size))
		return 0U;
	last_lba = (uint64_t)(unsigned int)lba + size / MMC_BLOCK_SIZE - 1U;
	if (last_lba > (emmc_card.sector_addressed ? UINT32_MAX :
		       UINT32_MAX / MMC_BLOCK_SIZE))
		return 0U;

	for (size_t offset = 0U; offset < size; offset += MMC_BLOCK_SIZE) {
		uint32_t argument = (uint32_t)lba + offset / MMC_BLOCK_SIZE;

		if (!emmc_card.sector_addressed)
			argument *= MMC_BLOCK_SIZE;
		if (emmc_write_block(argument,
			(const uint8_t *)(buffer + offset)) != 0) {
			ERROR("eMMC: write LBA %u failed\n",
			      (unsigned int)((uint32_t)lba +
			      offset / MMC_BLOCK_SIZE));
			emmc_card.ready = false;
			return 0U;
		}
	}
	return size;
}

size_t qrk3588_emmc_read(int lba, uintptr_t buffer, size_t size)
{
	return card_read(&emmc_card, lba, buffer, size, true);
}

size_t qrk3588_sdmmc_read(int lba, uintptr_t buffer, size_t size)
{
	return card_read(&sd_card, lba, buffer, size, false);
}
