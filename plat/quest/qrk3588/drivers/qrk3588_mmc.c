/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <drivers/delay_timer.h>
#include <drivers/mmc.h>
#include <lib/mmio.h>
#include <qrk3588_mmc.h>

#include "qrk3588_emmc_regs.h"

/* One-bit legacy mode: oscillator only, no PLL or delay-line calibration. */
#define EMMC_OSC_HZ			U(24000000)
#define EMMC_IDENT_HZ			U(400000)
#define EMMC_POLL_US			U(10)
/* Software policy budgets, not undocumented register programming values. */
#define EMMC_HOST_TIMEOUT_US		U(100000)
#define EMMC_COMMAND_TIMEOUT_US		U(100000)
#define EMMC_DATA_TIMEOUT_US		U(3000000)
#define EMMC_POWERUP_TIMEOUT_US		U(1000000)
#define EMMC_INITIAL_CLOCKS_US		U(1000) /* > 74 clocks at 400 kHz */
#define EMMC_OCR_POLL_US			U(1000)
#define EMMC_RELATIVE_ADDRESS		U(1) /* Host-assigned, single device */

/* JEDEC commands and R1 fields; shared protocol fields come from mmc.h. */
enum emmc_opcode {
	EMMC_GO_IDLE = 0,
	EMMC_SEND_OP_COND = 1,
	EMMC_ALL_SEND_CID = 2,
	EMMC_SET_RELATIVE_ADDR = 3,
	EMMC_SWITCH = 6,
	EMMC_SELECT_CARD = 7,
	EMMC_SEND_EXT_CSD = 8,
	EMMC_SEND_CSD = 9,
	EMMC_SEND_STATUS = 13,
	EMMC_SET_BLOCKLEN = 16,
	EMMC_READ_SINGLE_BLOCK = 17,
	EMMC_WRITE_BLOCK = 24,
};
#define EMMC_R1_OUT_OF_RANGE		BIT_32(31)
#define EMMC_R1_ADDRESS_ERROR		BIT_32(30)
#define EMMC_R1_BLOCK_LEN_ERROR		BIT_32(29)
#define EMMC_R1_ERASE_SEQ_ERROR		BIT_32(28)
#define EMMC_R1_ERASE_PARAM		BIT_32(27)
#define EMMC_R1_WP_VIOLATION		BIT_32(26)
#define EMMC_R1_CARD_LOCKED		BIT_32(25)
#define EMMC_R1_LOCK_FAILED		BIT_32(24)
#define EMMC_R1_COM_CRC_ERROR		BIT_32(23)
#define EMMC_R1_ILLEGAL_COMMAND		BIT_32(22)
#define EMMC_R1_ECC_FAILED		BIT_32(21)
#define EMMC_R1_CC_ERROR			BIT_32(20)
#define EMMC_R1_ERROR			BIT_32(19)
#define EMMC_R1_CID_CSD_OVERWRITE		BIT_32(16)
#define EMMC_R1_WP_ERASE_SKIP		BIT_32(15)
#define EMMC_R1_ERASE_RESET		BIT_32(13)
#define EMMC_R1_ERRORS			(EMMC_R1_OUT_OF_RANGE | EMMC_R1_ADDRESS_ERROR | \
	EMMC_R1_BLOCK_LEN_ERROR | EMMC_R1_ERASE_SEQ_ERROR | EMMC_R1_ERASE_PARAM | \
	EMMC_R1_WP_VIOLATION | EMMC_R1_CARD_LOCKED | EMMC_R1_LOCK_FAILED | \
	EMMC_R1_COM_CRC_ERROR | EMMC_R1_ILLEGAL_COMMAND | EMMC_R1_ECC_FAILED | \
	EMMC_R1_CC_ERROR | EMMC_R1_ERROR | EMMC_R1_CID_CSD_OVERWRITE | \
	EMMC_R1_WP_ERASE_SKIP | EMMC_R1_ERASE_RESET | STATUS_SWITCH_ERROR)

/* Versioned project ABI, not a pointer into another boot stage's memory. */
#define EMMC_HANDOFF_TAG			ULL(0x51454d3100000000) /* "QEM1" */
#define EMMC_HANDOFF_RCA_MASK		ULL(0xffff)
#define EMMC_HANDOFF_SECTOR_MODE		BIT_64(16)
#define EMMC_HANDOFF_PAYLOAD_MASK	(EMMC_HANDOFF_RCA_MASK | EMMC_HANDOFF_SECTOR_MODE)

static struct {
	uint16_t rca;
	bool sector_addressing;
	bool available;
} device;

static void masked_write(uintptr_t address, uint32_t mask, uint32_t value)
{
	mmio_write_32(address, (mask << EMMC_WRITE_ENABLE_SHIFT) | (value & mask));
}

static int reset_host(uint8_t lines)
{
	uint64_t deadline = timeout_init_us(EMMC_HOST_TIMEOUT_US);

	mmio_write_8(EMMC_BASE + EMMC_SOFTWARE_RESET, lines);
	do {
		if ((mmio_read_8(EMMC_BASE + EMMC_SOFTWARE_RESET) & lines) == 0U) {
			return 0;
		}
		udelay(EMMC_POLL_US);
	} while (!timeout_elapsed(deadline));
	return -ETIMEDOUT;
}

static int wait_idle(uint32_t mask)
{
	uint64_t deadline = timeout_init_us(EMMC_COMMAND_TIMEOUT_US);

	do {
		if ((mmio_read_32(EMMC_BASE + EMMC_PRESENT_STATE) & mask) == 0U) {
			return 0;
		}
		udelay(EMMC_POLL_US);
	} while (!timeout_elapsed(deadline));
	return -ETIMEDOUT;
}

static int wait_event(uint16_t event, uint32_t timeout_us)
{
	uint64_t deadline = timeout_init_us(timeout_us);

	do {
		uint16_t status = mmio_read_16(EMMC_BASE + EMMC_NORMAL_STATUS);
		uint16_t errors = mmio_read_16(EMMC_BASE + EMMC_ERROR_STATUS);

		/* A completion and an error can arrive in the same register read. */
		if ((errors != 0U) || ((status & EMMC_EVENT_ERROR) != 0U)) {
			return ((errors & (EMMC_ERROR_COMMAND_TIMEOUT |
					  EMMC_ERROR_DATA_TIMEOUT)) != 0U) ?
				-ETIMEDOUT : -EIO;
		}
		if ((status & event) != 0U) {
			mmio_write_16(EMMC_BASE + EMMC_NORMAL_STATUS, event);
			return 0;
		}
		udelay(EMMC_POLL_US);
	} while (!timeout_elapsed(deadline));
	return -ETIMEDOUT;
}

static void enable_polling(void)
{
	mmio_write_16(EMMC_BASE + EMMC_NORMAL_SIGNAL, 0U);
	mmio_write_16(EMMC_BASE + EMMC_ERROR_SIGNAL, 0U);
	mmio_write_16(EMMC_BASE + EMMC_NORMAL_ENABLE, EMMC_PIO_EVENTS);
	mmio_write_16(EMMC_BASE + EMMC_ERROR_ENABLE, EMMC_PIO_ERRORS);
}

/* One command owns its status bits until its data/busy phase has finished. */
static int execute(enum emmc_opcode opcode, uint32_t argument,
		   uint16_t format, void *block, bool write, uint32_t *response)
{
	uint32_t inhibit = EMMC_INHIBIT_COMMAND;
	uint32_t status;
	int ret;

	if ((block != NULL) ||
	    ((format & EMMC_RESPONSE_MASK) == EMMC_RESPONSE_BUSY)) {
		inhibit |= EMMC_INHIBIT_DATA;
	}
	ret = wait_idle(inhibit);
	if (ret != 0) {
		return ret;
	}
	mmio_write_16(EMMC_BASE + EMMC_NORMAL_STATUS, EMMC_PIO_EVENTS);
	mmio_write_16(EMMC_BASE + EMMC_ERROR_STATUS, EMMC_PIO_ERRORS);
	if (block != NULL) {
		mmio_write_16(EMMC_BASE + EMMC_BLOCK_SIZE, MMC_BLOCK_SIZE);
		mmio_write_16(EMMC_BASE + EMMC_BLOCK_COUNT, 1U);
		format |= EMMC_COMMAND_DATA;
	}
	mmio_write_16(EMMC_BASE + EMMC_TRANSFER_MODE,
		     (block != NULL && !write) ? EMMC_TRANSFER_READ : EMMC_TRANSFER_WRITE);
	mmio_write_32(EMMC_BASE + EMMC_ARGUMENT, argument);
	mmio_write_16(EMMC_BASE + EMMC_COMMAND,
		     ((uint16_t)opcode << EMMC_COMMAND_INDEX_SHIFT) | format);
	ret = wait_event(EMMC_EVENT_COMMAND, EMMC_COMMAND_TIMEOUT_US);
	if (ret != 0) {
		return ret;
	}
	status = mmio_read_32(EMMC_BASE + EMMC_RESPONSE);
	if (response != NULL) {
		*response = status;
	}
	/* R3 is OCR, R2 is CID/CSD: neither contains an R1 status word. */
	if (((format & EMMC_COMMAND_INDEX_CHECK) != 0U) &&
	    ((status & EMMC_R1_ERRORS) != 0U)) {
		return -EIO;
	}
	if (block != NULL) {
		ret = wait_event(write ? EMMC_EVENT_WRITE_READY : EMMC_EVENT_READ_READY,
				 EMMC_DATA_TIMEOUT_US);
		if (ret != 0) {
			return ret;
		}
		for (size_t offset = 0U; offset < MMC_BLOCK_SIZE; offset += sizeof(uint32_t)) {
			uint32_t word;

			/* Callers may supply unaligned buffers. The CPU is little-endian. */
			if (write) {
				memcpy(&word, (uint8_t *)block + offset, sizeof(word));
				mmio_write_32(EMMC_BASE + EMMC_BUFFER, word);
			} else {
				word = mmio_read_32(EMMC_BASE + EMMC_BUFFER);
				memcpy((uint8_t *)block + offset, &word, sizeof(word));
			}
		}
	}
	if ((block != NULL) ||
	    ((format & EMMC_RESPONSE_MASK) == EMMC_RESPONSE_BUSY)) {
		return wait_event(EMMC_EVENT_TRANSFER, EMMC_DATA_TIMEOUT_US);
	}
	return 0;
}

static int card_ready(void)
{
	uint64_t deadline = timeout_init_us(EMMC_DATA_TIMEOUT_US);

	do {
		uint32_t status;
		int ret = execute(EMMC_SEND_STATUS,
			(uint32_t)device.rca << RCA_SHIFT_OFFSET,
			EMMC_RESPONSE_R1, NULL, false, &status);

		if (ret != 0) {
			return ret;
		}
		if (((status & STATUS_READY_FOR_DATA) != 0U) &&
		    (MMC_GET_STATE(status) == MMC_STATE_TRAN)) {
			return 0;
		}
		udelay(EMMC_POLL_US);
	} while (!timeout_elapsed(deadline));
	return -ETIMEDOUT;
}

static int switch_byte(unsigned int index, uint8_t value)
{
	int ret = execute(EMMC_SWITCH, EXTCSD_WRITE_BYTES | EXTCSD_CMD(index) |
		EXTCSD_VALUE(value), EMMC_RESPONSE_R1B, NULL, false, NULL);

	return (ret == 0) ? card_ready() : ret;
}

static int set_clock(unsigned int hz)
{
	unsigned int divisor = EMMC_OSC_HZ / hz;
	uint64_t deadline;

	/* Change the physical input clock at CRU, without a PLL dependency. */
	mmio_write_16(EMMC_BASE + EMMC_CLOCK_CONTROL, 0U);
	masked_write(EMMC_CLKSEL, EMMC_CLK_SOURCE_MASK | EMMC_CLK_DIV_MASK,
		     EMMC_CLK_SOURCE_OSC | ((divisor - 1U) << EMMC_CLK_DIV_SHIFT));
	mmio_write_16(EMMC_BASE + EMMC_CLOCK_CONTROL, EMMC_INTERNAL_CLOCK_ON);
	deadline = timeout_init_us(EMMC_HOST_TIMEOUT_US);
	do {
		if ((mmio_read_16(EMMC_BASE + EMMC_CLOCK_CONTROL) &
		     EMMC_INTERNAL_CLOCK_STABLE) != 0U) {
			mmio_write_16(EMMC_BASE + EMMC_CLOCK_CONTROL,
				     EMMC_INTERNAL_CLOCK_ON | EMMC_CARD_CLOCK_ON);
			return 0;
		}
		udelay(EMMC_POLL_US);
	} while (!timeout_elapsed(deadline));
	return -ETIMEDOUT;
}

static int prepare_host(void)
{
	int ret;

	masked_write(EMMC_CLKGATE, EMMC_CLOCK_GATES, 0U);
	masked_write(EMMC_IOC_CMD_CLK, EMMC_CMD_MUX_MASK | EMMC_CLK_MUX_MASK,
		EMMC_PIN_FUNCTION | (EMMC_PIN_FUNCTION << EMMC_CLK_MUX_SHIFT));
	masked_write(EMMC_IOC_DATA0, EMMC_DATA0_MUX_MASK, EMMC_PIN_FUNCTION);
	ret = reset_host(EMMC_RESET_ALL);
	if (ret != 0) {
		return ret;
	}
	masked_write(EMMC_SYS_GRF_CON10, EMMC_GRF_CLOCK_BYPASS | EMMC_GRF_CLOCK_STABLE,
		     EMMC_GRF_CLOCK_STABLE);
	mmio_write_32(EMMC_BASE + EMMC_DLL_CONTROL, EMMC_DLL_BYPASS);
	mmio_write_32(EMMC_BASE + EMMC_DLL_RECEIVE, EMMC_RX_ORIGINAL_CLOCK);
	mmio_write_32(EMMC_BASE + EMMC_DLL_TRANSMIT, EMMC_TX_INVERTED_CLOCK);
	mmio_write_32(EMMC_BASE + EMMC_DLL_COMMAND_OUT, EMMC_CMD_RISING_EDGE);
	mmio_write_16(EMMC_BASE + EMMC_CARD_CONTROL,
		     EMMC_CARD_IS_EMMC | EMMC_CARD_RESET_RELEASED);
	mmio_write_8(EMMC_BASE + EMMC_HOST_CONTROL, EMMC_HOST_LEGACY_1BIT);
	mmio_write_16(EMMC_BASE + EMMC_HOST_CONTROL2, EMMC_HOST_NO_PRESET);
	mmio_write_8(EMMC_BASE + EMMC_POWER_CONTROL, EMMC_BUS_POWER_ON);
	mmio_write_8(EMMC_BASE + EMMC_TIMEOUT_CONTROL, EMMC_TIMEOUT_MAX);
	enable_polling();
	return set_clock(EMMC_IDENT_HZ);
}

/* After an uncertain transfer, require explicit reinitialization, never replay a write. */
static int invalidate(int error)
{
	device.available = false;
	(void)reset_host(EMMC_RESET_COMMAND | EMMC_RESET_DATA);
	return error;
}

int qrk3588_emmc_init(void)
{
	uint8_t ext_csd[MMC_BLOCK_SIZE];
	uint64_t deadline;
	uint32_t ocr = 0U;
	uint8_t user_partition;
	int ret;

	device.available = false;
	device.rca = EMMC_RELATIVE_ADDRESS;
	device.sector_addressing = false;
	ret = prepare_host();
	if (ret != 0) {
		return ret;
	}
	udelay(EMMC_INITIAL_CLOCKS_US);
	/*
	 * This controller completes CMD0 with NORMAL type. On the tested board,
	 * ABORT type never raised Command Complete, despite the TRM CMD note.
	 * A same-state NORMAL/ABORT comparison confirmed the distinction.
	 */
	ret = execute(EMMC_GO_IDLE, 0U, EMMC_RESPONSE_NONE | EMMC_COMMAND_NORMAL,
		      NULL, false, NULL);
	if (ret != 0) {
		return invalidate(ret);
	}
	deadline = timeout_init_us(EMMC_POWERUP_TIMEOUT_US);
	do {
		ret = execute(EMMC_SEND_OP_COND, OCR_HCS | OCR_VDD_MIN_2V7,
			      EMMC_RESPONSE_R3, NULL, false, &ocr);
		if (ret != 0) {
			return invalidate(ret);
		}
		if ((ocr & OCR_POWERUP) != 0U) {
			break;
		}
		udelay(EMMC_OCR_POLL_US);
	} while (!timeout_elapsed(deadline));
	if ((ocr & OCR_POWERUP) == 0U) {
		return invalidate(-ETIMEDOUT);
	}
	if (((ocr & OCR_VDD_MIN_2V7) == 0U) ||
	    (((ocr & OCR_ACCESS_MODE_MASK) != OCR_BYTE_MODE) &&
	     ((ocr & OCR_ACCESS_MODE_MASK) != OCR_SECTOR_MODE))) {
		return invalidate(-EIO);
	}
	device.sector_addressing = (ocr & OCR_ACCESS_MODE_MASK) == OCR_SECTOR_MODE;
	ret = execute(EMMC_ALL_SEND_CID, 0U, EMMC_RESPONSE_R2, NULL, false, NULL);
	if (ret == 0) {
		ret = execute(EMMC_SET_RELATIVE_ADDR,
			(uint32_t)device.rca << RCA_SHIFT_OFFSET, EMMC_RESPONSE_R1,
			NULL, false, NULL);
	}
	if (ret == 0) {
		ret = execute(EMMC_SEND_CSD, (uint32_t)device.rca << RCA_SHIFT_OFFSET,
			      EMMC_RESPONSE_R2, NULL, false, NULL);
	}
	if (ret == 0) {
		ret = execute(EMMC_SELECT_CARD, (uint32_t)device.rca << RCA_SHIFT_OFFSET,
			      EMMC_RESPONSE_R1B, NULL, false, NULL);
	}
	if (ret == 0) {
		ret = execute(EMMC_SET_BLOCKLEN, MMC_BLOCK_SIZE, EMMC_RESPONSE_R1,
			      NULL, false, NULL);
	}
	if (ret == 0) {
		ret = execute(EMMC_SEND_EXT_CSD, 0U, EMMC_RESPONSE_R1, ext_csd, false, NULL);
	}
	if (ret != 0) {
		return invalidate(ret);
	}
	user_partition = ext_csd[CMD_EXTCSD_PARTITION_CONFIG] & ~EXT_CSD_PART_CONFIG_ACC_MASK;
	if (user_partition != ext_csd[CMD_EXTCSD_PARTITION_CONFIG]) {
		ret = switch_byte(CMD_EXTCSD_PARTITION_CONFIG, user_partition);
		if (ret == 0) {
			ret = execute(EMMC_SEND_EXT_CSD, 0U, EMMC_RESPONSE_R1,
				      ext_csd, false, NULL);
		}
		if ((ret == 0) && (ext_csd[CMD_EXTCSD_PARTITION_CONFIG] != user_partition)) {
			ret = -EIO;
		}
	}
	if (ret == 0) {
		ret = set_clock(EMMC_OSC_HZ);
	}
	if (ret == 0) {
		ret = card_ready();
	}
	if (ret != 0) {
		return invalidate(ret);
	}
	device.available = true;
	return 0;
}

uint64_t qrk3588_emmc_handoff_token(void)
{
	if (!device.available) {
		return 0U;
	}
	return EMMC_HANDOFF_TAG | device.rca |
		(device.sector_addressing ? EMMC_HANDOFF_SECTOR_MODE : 0U);
}

int qrk3588_emmc_handoff(uint64_t token)
{
	int ret;

	device.available = false;
	if (((token & ~EMMC_HANDOFF_PAYLOAD_MASK) != EMMC_HANDOFF_TAG) ||
	    ((token & EMMC_HANDOFF_RCA_MASK) == 0U)) {
		return -EINVAL;
	}
	device.rca = token & EMMC_HANDOFF_RCA_MASK;
	device.sector_addressing = (token & EMMC_HANDOFF_SECTOR_MODE) != 0U;
	/* The producer leaves a selected card and configured host. Do not reset either. */
	enable_polling();
	ret = card_ready();
	if (ret != 0) {
		return invalidate(ret);
	}
	device.available = true;
	return 0;
}

static size_t transfer_blocks(int lba, uintptr_t buffer, size_t size, bool write)
{
	uint64_t last_block;
	uint64_t max_block = device.sector_addressing ? UINT32_MAX : UINT32_MAX / MMC_BLOCK_SIZE;
	size_t blocks = size / MMC_BLOCK_SIZE;

	if (!device.available || (lba < 0) || (buffer == 0U) || (size == 0U) ||
	    ((size & MMC_BLOCK_MASK) != 0U) || (size - 1U > UINTPTR_MAX - buffer)) {
		return 0U;
	}
	last_block = (uint64_t)(unsigned int)lba + blocks - 1U;
	if (last_block > max_block) {
		return 0U;
	}
	for (size_t i = 0U; i < blocks; ++i) {
		uint64_t block = (uint64_t)(unsigned int)lba + i;
		uint32_t argument = device.sector_addressing ? block : block * MMC_BLOCK_SIZE;
		int ret = execute(write ? EMMC_WRITE_BLOCK : EMMC_READ_SINGLE_BLOCK,
			argument, EMMC_RESPONSE_R1,
			(void *)(buffer + i * MMC_BLOCK_SIZE), write, NULL);

		if ((ret == 0) && write) {
			ret = card_ready();
		}
		if (ret != 0) {
			(void)invalidate(ret);
			return 0U;
		}
	}
	return size;
}

size_t qrk3588_emmc_read(int lba, uintptr_t buffer, size_t size)
{
	return transfer_blocks(lba, buffer, size, false);
}

size_t qrk3588_emmc_write(int lba, uintptr_t buffer, size_t size)
{
	return transfer_blocks(lba, buffer, size, true);
}
