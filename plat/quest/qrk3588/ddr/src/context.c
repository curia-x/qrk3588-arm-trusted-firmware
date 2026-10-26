/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <lib/mmio.h>

#include "qrk3588_ddr_internal.h"

#define QRK3588_DDR_TIMING_WORK_FLAG_OFFSET \
	(QRK3588_DDR_TIMING_WORK_FLAG_VA - QRK3588_DDR_LAYOUT_VA_BASE)

struct qrk3588_ddr_context *qrk3588_ddr_active_context;

uintptr_t qrk3588_ddr_resolve_addr(uintptr_t address)
{
	struct qrk3588_ddr_context *ctx = qrk3588_ddr_active_context;

	if (ctx == NULL)
		return address;

	if ((address >= QRK3588_DDR_LAYOUT_VA_BASE) &&
	    (address < QRK3588_DDR_LAYOUT_VA_END))
		return (uintptr_t)&ctx->layout_storage[
			address - QRK3588_DDR_LAYOUT_VA_BASE];

	if ((address >= QRK3588_DDR_TAG_VA_BASE) &&
	    (address < QRK3588_DDR_TAG_VA_BASE + QRK3588_DDR_TAG_AREA_SIZE))
		return (uintptr_t)&ctx->tag_area[
			address - QRK3588_DDR_TAG_VA_BASE];

	return address;
}

void *qrk3588_ddr_resolve_layout(uintptr_t address)
{
	return (void *)qrk3588_ddr_resolve_addr(address);
}

static void qrk3588_ddr_rebase_parameter_pointers(
	struct qrk3588_ddr_context *ctx)
{
	const uintptr_t first = QRK3588_DDR_FIXED_DATA_VA -
				QRK3588_DDR_LAYOUT_VA_BASE;
	const uintptr_t last = first + QRK3588_DDR_FIXED_DATA_SIZE;

	for (uintptr_t offset = first;
	     offset + sizeof(uint64_t) <= last;
	     offset += sizeof(uint64_t)) {
		uint64_t value;

		memcpy(&value, &ctx->layout_storage[offset], sizeof(value));
		if ((value < QRK3588_DDR_LAYOUT_VA_BASE) ||
		    (value >= QRK3588_DDR_LAYOUT_VA_END))
			continue;

		value = (uintptr_t)&ctx->layout_storage[
			(uintptr_t)value - QRK3588_DDR_LAYOUT_VA_BASE];
		memcpy(&ctx->layout_storage[offset], &value, sizeof(value));
	}
}

void qrk3588_ddr_context_prepare(struct qrk3588_ddr_context *ctx)
{
	if (ctx == NULL)
		return;

	memset(ctx, 0, sizeof(*ctx));
	ctx->poll_timeout_us = QRK3588_DDR_DEFAULT_POLL_TIMEOUT_US;
	qrk3588_ddr_active_context = ctx;

	memcpy(&ctx->layout_storage[QRK3588_DDR_FIXED_DATA_VA -
				    QRK3588_DDR_LAYOUT_VA_BASE],
	       qrk3588_ddr_fixed_data_template(),
	       QRK3588_DDR_FIXED_DATA_SIZE);
	qrk3588_ddr_rebase_parameter_pointers(ctx);
}

void qrk3588_ddr_set_error(int error)
{
	if ((qrk3588_ddr_active_context != NULL) &&
	    (qrk3588_ddr_active_context->error == 0))
		qrk3588_ddr_active_context->error = error;
}

int qrk3588_ddr_get_error(void)
{
	if (qrk3588_ddr_active_context == NULL)
		return QRK3588_DDR_EINVAL;

	return qrk3588_ddr_active_context->error;
}

uint32_t qrk3588_ddr_timing_work_flag(void)
{
	if (qrk3588_ddr_active_context == NULL)
		return 0U;

	return *(const uint32_t *)(const void *)&qrk3588_ddr_active_context->
		layout_storage[QRK3588_DDR_TIMING_WORK_FLAG_OFFSET];
}

void qrk3588_ddr_set_timing_work_flag(uint32_t value)
{
	if (qrk3588_ddr_active_context != NULL)
		*(uint32_t *)(void *)&qrk3588_ddr_active_context->
			layout_storage[QRK3588_DDR_TIMING_WORK_FLAG_OFFSET] = value;
}

uint64_t qrk3588_ddr_geometry_capacity_bytes(
	const struct qrk3588_ddr_channel_geometry *geometry,
	uint32_t dram_type, uint32_t mode)
{
	uint32_t width_adjust;
	uint32_t address_bits;
	uint64_t size;

	if (geometry == NULL)
		return 0U;

	width_adjust = (dram_type != 0U) ? 0U :
		(geometry->device_width_code == 0U ? 2U : 1U);
	address_bits = geometry->bus_width_code + geometry->column_bits +
		       width_adjust + geometry->bank_bits;
	size = UINT64_C(1) << ((address_bits + geometry->cs_row_bits[0]) & 63U);

	if (geometry->rank_count > 1U) {
		size += UINT64_C(1) <<
			((address_bits + geometry->cs_row_bits[1]) & 63U);
		if (geometry->rank_count == 4U) {
			size += UINT64_C(1) <<
				((address_bits + geometry->cs_row_bits[2]) & 63U);
			size += UINT64_C(1) <<
				((address_bits + geometry->cs_row_bits[3]) & 63U);
		}
	}

	if (geometry->row_3_4 != 0U)
		size = (size * 3U) >> 2;
	else if (mode != 0U)
		size = ((uint64_t)(mode & 0xffU) << 23) + (size >> 1);

	return size;
}

static uint32_t qrk3588_ddr_poll_timeout(uint32_t timeout_us)
{
	if (timeout_us != 0U)
		return timeout_us;

	if ((qrk3588_ddr_active_context != NULL) &&
	    (qrk3588_ddr_active_context->poll_timeout_us != 0U))
		return qrk3588_ddr_active_context->poll_timeout_us;

	return QRK3588_DDR_DEFAULT_POLL_TIMEOUT_US;
}

int qrk3588_ddr_try_poll32(uintptr_t address, uint32_t mask,
			   uint32_t expected, uint32_t timeout_us)
{
	uint32_t timeout = qrk3588_ddr_poll_timeout(timeout_us);
	uint32_t value = 0U;

	for (uint32_t elapsed = 0U; elapsed < timeout; elapsed++) {
		value = mmio_read_32(qrk3588_ddr_resolve_addr(address));
		if ((value & mask) == expected)
			return 0;
		udelay(1U);
	}

	if ((qrk3588_ddr_active_context != NULL) &&
	    (qrk3588_ddr_active_context->error == 0)) {
		qrk3588_ddr_active_context->poll_address = address;
		qrk3588_ddr_active_context->poll_caller =
			(uintptr_t)__builtin_return_address(0);
		qrk3588_ddr_active_context->poll_value = value;
		qrk3588_ddr_active_context->poll_mask = mask;
		qrk3588_ddr_active_context->poll_expected = expected;
	}

	return QRK3588_DDR_ETIMEDOUT;
}

int qrk3588_ddr_poll32(uintptr_t address, uint32_t mask, uint32_t expected,
		       uint32_t timeout_us)
{
	int ret = qrk3588_ddr_try_poll32(address, mask, expected, timeout_us);

	if (ret != 0) {
		if ((qrk3588_ddr_active_context != NULL) &&
		    (qrk3588_ddr_active_context->error == 0))
			qrk3588_ddr_active_context->poll_caller =
				(uintptr_t)__builtin_return_address(0);
		qrk3588_ddr_set_error(ret);
	}

	return ret;
}

int qrk3588_ddr_poll_any32(uintptr_t address, uint32_t mask,
			   uint32_t timeout_us)
{
	uint32_t timeout = qrk3588_ddr_poll_timeout(timeout_us);
	uint32_t value = 0U;

	for (uint32_t elapsed = 0U; elapsed < timeout; elapsed++) {
		value = mmio_read_32(qrk3588_ddr_resolve_addr(address));
		if ((value & mask) != 0U)
			return 0;
		udelay(1U);
	}

	if ((qrk3588_ddr_active_context != NULL) &&
	    (qrk3588_ddr_active_context->error == 0)) {
		qrk3588_ddr_active_context->poll_address = address;
		qrk3588_ddr_active_context->poll_caller =
			(uintptr_t)__builtin_return_address(0);
		qrk3588_ddr_active_context->poll_value = value;
		qrk3588_ddr_active_context->poll_mask = mask;
		qrk3588_ddr_active_context->poll_expected = mask;
	}

	qrk3588_ddr_set_error(QRK3588_DDR_ETIMEDOUT);
	return QRK3588_DDR_ETIMEDOUT;
}

const uint8_t *qrk3588_ddr_type_config_table(void)
{
	return DDR_LAYOUT_CPTR(uint8_t, QRK3588_DDR_TYPE_CONFIG_TABLE_VA);
}

const struct qrk3588_ddr_freq_bin *qrk3588_lp4_freq_bins(void)
{
	return DDR_LAYOUT_CPTR(struct qrk3588_ddr_freq_bin,
			       QRK3588_DDR_LP4_FREQ_BINS_VA);
}

const struct qrk3588_ddr_freq_bin *qrk3588_lp5_freq_bins(void)
{
	return DDR_LAYOUT_CPTR(struct qrk3588_ddr_freq_bin,
			       QRK3588_DDR_LP5_FREQ_BINS_VA);
}

const uint8_t *qrk3588_lp4_2112_preset(void)
{
	return DDR_LAYOUT_CPTR(uint8_t, QRK3588_DDR_LP4_2112_PRESET_VA);
}

const uint8_t *qrk3588_lp5_2400_preset(void)
{
	return DDR_LAYOUT_CPTR(uint8_t, QRK3588_DDR_LP5_2400_PRESET_VA);
}

const struct qrk3588_ddr_training_threshold *qrk3588_lp5_training_low(void)
{
	return DDR_LAYOUT_CPTR(struct qrk3588_ddr_training_threshold,
			       QRK3588_DDR_LP5_TRAINING_LOW_VA);
}

const struct qrk3588_ddr_training_threshold *qrk3588_lp5_training_high(void)
{
	return DDR_LAYOUT_CPTR(struct qrk3588_ddr_training_threshold,
			       QRK3588_DDR_LP5_TRAINING_HIGH_VA);
}

int qrk3588_ddr_bl2_init(struct qrk3588_ddr_context *ctx,
			 const void *config)
{
	int ret;

	if ((ctx == NULL) || (config == NULL))
		return QRK3588_DDR_EINVAL;

	qrk3588_ddr_context_prepare(ctx);
	ret = qrk3588_ddr_main((void *)config);
	if (qrk3588_ddr_get_error() != 0)
		return qrk3588_ddr_get_error();

	return ret;
}
