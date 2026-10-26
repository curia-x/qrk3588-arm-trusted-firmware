/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>

#include <common/debug.h>
#include <platform_def.h>
#include <qrk3588_ddr.h>
#include <qrk3588_ddr_runtime.h>

static struct qrk3588_ddr_context qrk3588_ddr_context
	__aligned(CACHE_WRITEBACK_GRANULE);

/* Four channel records plus global timing and native pointer fields. */
#define QRK3588_DDR_CONFIG_SIZE	U(0x238)

static uint8_t qrk3588_ddr_config[QRK3588_DDR_CONFIG_SIZE]
	__aligned(sizeof(uint64_t));

static uint64_t qrk3588_ddr_tag_read_u64(const uint8_t *address)
{
	const volatile uint32_t *words =
		(const volatile uint32_t *)(const void *)address;
	uint32_t low = words[0];
	uint32_t high = words[1];

	/* Tag records are word-aligned, but are not necessarily 8-byte aligned. */
	return (uint64_t)low | ((uint64_t)high << 32);
}

/*
 * The DDR runtime owns tag_area, but it remains valid until BL2 leaves the
 * DDR bring-up path.  Decode only the bounded tag52 region-list ABI here.
 */
static void qrk3588_ddr_report_regions(void)
{
	const uint32_t region_tag = UINT32_C(0x54410052);
	const uint32_t *entry = DDR_LAYOUT_CPTR(uint32_t,
					    QRK3588_DDR_TAG_VA_BASE);
	size_t words_left = QRK3588_DDR_TAG_AREA_SIZE / sizeof(*entry);
	uint32_t count = 0U;
	uint32_t reported = 0U;
	uint64_t total = 0U;
	const uint8_t *region_starts = NULL;
	const uint8_t *region_sizes = NULL;
	while (words_left >= 2U) {
		uint32_t entry_words = entry[0];

		if ((entry_words == 0U) || (entry_words > words_left))
			break;

		if (entry[1] == region_tag) {
			const uint8_t *payload = (const uint8_t *)(const void *)(entry + 2);
			const uint8_t *starts;
			const uint8_t *sizes;

			if (entry_words < 4U)
				break;

			count = *(const uint32_t *)(const void *)payload;
			if ((count > 10U) ||
			    (8U + (size_t)count * 2U * sizeof(uint64_t) >
			     (size_t)(entry_words - 2U) * sizeof(*entry)))
				break;

			starts = payload + 8U;
			sizes = starts + (size_t)count * sizeof(uint64_t);
			region_starts = starts;
			region_sizes = sizes;
			for (uint32_t i = 0U; i < count; ++i) {
				uint64_t size = qrk3588_ddr_tag_read_u64(
					sizes + (size_t)i * sizeof(uint64_t));

				if (size != 0U) {
					total += size;
					++reported;
				}
			}
			break;
		}

		entry += entry_words;
		words_left -= entry_words;
	}

	NOTICE("DDR: %u accessible region(s), %llu MiB total\n",
	       reported, (unsigned long long)(total >> 20));
	for (uint32_t i = 0U; i < count; ++i) {
		uint64_t start = qrk3588_ddr_tag_read_u64(
			region_starts + (size_t)i * sizeof(uint64_t));
		uint64_t size = qrk3588_ddr_tag_read_u64(
			region_sizes + (size_t)i * sizeof(uint64_t));
		uint64_t end;

		if (size == 0U)
			continue;

		end = start + size - 1U;
		NOTICE("DDR: [0x%016llx - 0x%016llx] (%llu MiB)\n",
		       (unsigned long long)start,
		       (unsigned long long)end,
		       (unsigned long long)(size >> 20));
	}
}

int qrk3588_ddr_init(void)
{
	int ret = qrk3588_ddr_bl2_init(&qrk3588_ddr_context,
					qrk3588_ddr_config);

	if (ret == 0)
		qrk3588_ddr_report_regions();

	return ret;
}
