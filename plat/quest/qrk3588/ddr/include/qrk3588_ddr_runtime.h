/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_DDR_RUNTIME_H
#define QRK3588_DDR_RUNTIME_H

#include "qrk3588_ddr_context.h"
#include "qrk3588_ddr_layout.h"
#include "qrk3588_ddr_state.h"

#include <stddef.h>
#include <stdint.h>

#include <lib/libc/cdefs.h>
#include <lib/utils_def.h>

#define QRK3588_DDR_FIXED_DATA_SIZE      0x1fc8u
#define QRK3588_DDR_TAG_VA_BASE          ((uintptr_t)0x001fe000u)
#define QRK3588_DDR_TAG_AREA_SIZE        0x2000u

#define QRK3588_DDR_DEFAULT_POLL_TIMEOUT_US  10000u
#define QRK3588_DDR_BOARD_CONFIG_SIZE         32u

#define QRK3588_DDR_EINVAL     (-22)
#define QRK3588_DDR_ETIMEDOUT  (-110)
#define QRK3588_DDR_EIO        (-5)

enum qrk3588_ddr_init_phase {
	QRK3588_DDR_PHASE_NONE = 0,
	QRK3588_DDR_PHASE_CLOCK_SETUP,
	QRK3588_DDR_PHASE_RESET_ASSERT,
	QRK3588_DDR_PHASE_PHY_BOOT_CONFIG,
	QRK3588_DDR_PHASE_RESET_RELEASE,
	QRK3588_DDR_PHASE_PCTL_CONFIG,
	QRK3588_DDR_PHASE_PHY_PARAM_CONFIG,
	QRK3588_DDR_PHASE_PHY_UPDATE,
	QRK3588_DDR_PHASE_PHY_IMAGE_CONFIG,
	QRK3588_DDR_PHASE_DFI_START,
	QRK3588_DDR_PHASE_CHANNEL_COMMIT,
	QRK3588_DDR_PHASE_CHANNEL_FINALIZE,
	QRK3588_DDR_PHASE_INITIAL_TRAINING,
	QRK3588_DDR_PHASE_MEMORY_TEST,
	QRK3588_DDR_PHASE_CHANNEL_TRAINING,
	QRK3588_DDR_PHASE_PUBLISH,
};

struct qrk3588_ddr_memory_trace {
	uintptr_t address;
	uint32_t expected;
	uint32_t observed;
	uint32_t first_words[4];
	uint32_t channel;
	uint32_t pass;
	uint32_t word;
	uint32_t valid;
};

struct qrk3588_ddr_context {
	/*
	 * Parameter tables use stable layout tokens. Resolve those tokens into
	 * BL2-owned storage at the driver boundary.
	 */
	uint8_t layout_storage[QRK3588_DDR_LAYOUT_STORAGE_SIZE]
		__aligned(16);
	uint8_t tag_area[QRK3588_DDR_TAG_AREA_SIZE]
		__aligned(16);
	uint8_t debug_log[0x2000] __aligned(16);

	uintptr_t poll_address;
	uintptr_t poll_caller;
	uint32_t poll_value;
	uint32_t poll_mask;
	uint32_t poll_expected;
	uint32_t poll_timeout_us;
	uint32_t error_cru_mode;
	uint32_t error_cru_clksel1;
	uint32_t error_dram_type;
	uint32_t error_channel;
	uint32_t error_rank_count;
	uint32_t error_phase;
	int32_t initial_training_result[QRK3588_DDR_CHANNEL_COUNT];
	int32_t initial_phy_update_result[QRK3588_DDR_CHANNEL_COUNT];
	struct qrk3588_ddr_memory_trace memory_trace;
	int32_t error;
};

/* Named views over LPDDR parameter tables. */
struct qrk3588_ddr_freq_bin {
	uint8_t raw[0x15];
};

struct qrk3588_ddr_training_threshold {
	uint32_t max_data_rate_mhz;
	uint32_t mode_flag;
	uint32_t param2;
	uint32_t param3;
	uint32_t param4;
	uint32_t param5;
};

extern struct qrk3588_ddr_context *qrk3588_ddr_active_context;

void qrk3588_ddr_context_prepare(struct qrk3588_ddr_context *ctx);
void *qrk3588_ddr_resolve_layout(uintptr_t address);
uintptr_t qrk3588_ddr_resolve_addr(uintptr_t address);
void qrk3588_ddr_set_error(int error);
int qrk3588_ddr_get_error(void);
uint32_t qrk3588_ddr_timing_work_flag(void);
void qrk3588_ddr_set_timing_work_flag(uint32_t value);

/* Geometry helper shared by discovery and GRF publication. */
uint64_t qrk3588_ddr_geometry_capacity_bytes(
    const struct qrk3588_ddr_channel_geometry *geom,
    uint32_t dram_type, uint32_t mode);

int qrk3588_ddr_poll32(uintptr_t address, uint32_t mask, uint32_t expected,
                      uint32_t timeout_us);
int qrk3588_ddr_poll_any32(uintptr_t address, uint32_t mask,
                          uint32_t timeout_us);
int qrk3588_ddr_try_poll32(uintptr_t address, uint32_t mask, uint32_t expected,
                          uint32_t timeout_us);

const uint8_t *qrk3588_ddr_fixed_data_template(void);
const uint8_t *qrk3588_ddr_type_config_table(void);
const struct qrk3588_ddr_freq_bin *qrk3588_lp4_freq_bins(void);
const struct qrk3588_ddr_freq_bin *qrk3588_lp5_freq_bins(void);
const uint8_t *qrk3588_lp4_2112_preset(void);
const uint8_t *qrk3588_lp5_2400_preset(void);
const struct qrk3588_ddr_training_threshold *qrk3588_lp5_training_low(void);
const struct qrk3588_ddr_training_threshold *qrk3588_lp5_training_high(void);

/* BL2-facing entry point. Returns 0 on success or a negative errno-style code. */
int qrk3588_ddr_bl2_init(struct qrk3588_ddr_context *ctx,
			 const void *config);

#define DDR_LAYOUT_ADDR(addr) qrk3588_ddr_resolve_addr((uintptr_t)(addr))
#define DDR_LAYOUT_PTR(type, addr) ((type *)(uintptr_t)DDR_LAYOUT_ADDR(addr))
#define DDR_LAYOUT_CPTR(type, addr) ((const type *)(uintptr_t)DDR_LAYOUT_ADDR(addr))
#define DDR_LAYOUT_REF(type, addr) (*DDR_LAYOUT_PTR(type, addr))

#endif
