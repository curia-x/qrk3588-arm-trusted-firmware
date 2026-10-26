/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdint.h>

#include <arch_helpers.h>
#include <lib/mmio.h>

#include <qrk3588_clock.h>
#include <qrk3588_cru.h>

/* --------------------------------------------------------------------------
 * Boot profile
 * -------------------------------------------------------------------------- */

#define QRK3588_OSC_SAFE_DIV                 U(1)

#define QRK3588_BOOT_BUS_DIV                 U(3)
#define QRK3588_BOOT_DSU_DF_DIV              U(2)
#define QRK3588_BOOT_DSU_PCLK_DIV            U(3)
#define QRK3588_BOOT_CORE0_DIV               U(2)
#define QRK3588_BOOT_SBUS_MATRIX_DIV         U(7)
#define QRK3588_BOOT_SBUS_PCLK_DIV           U(7)

#define QRK3588_BOOT_MATRIX_100_DIV          U(15)
#define QRK3588_BOOT_MATRIX_150_DIV          U(6)
#define QRK3588_BOOT_MATRIX_200_DIV          U(6)
#define QRK3588_BOOT_MATRIX_350_DIV          U(2)
#define QRK3588_BOOT_MATRIX_400_DIV          U(3)
#define QRK3588_BOOT_MATRIX_500_DIV          U(3)

/* SECURECRU matrix divider values are made explicit instead of inherited. */
#define QRK3588_BOOT_SEC_58_DIV              U(4)
#define QRK3588_BOOT_SEC_116_DIV             U(2)
#define QRK3588_BOOT_SEC_175_DIV             U(2)
#define QRK3588_BOOT_SEC_233_DIV             U(3)
#define QRK3588_BOOT_SEC_350_DIV             U(2)

/*
 * PLL rate = 24 MHz * M / P / 2^S.  CPLL/GPLL use K=0 deliberately so the
 * early profile stays integer and easy to audit.
 */
#define QRK3588_CPLL_M                       U(125)
#define QRK3588_CPLL_P                       U(1)
#define QRK3588_CPLL_S                       U(2)

#define QRK3588_GPLL_M                       U(100)
#define QRK3588_GPLL_P                       U(1)
#define QRK3588_GPLL_S                       U(2)

#define QRK3588_LPLL_M                       U(300)
#define QRK3588_LPLL_P                       U(2)
#define QRK3588_LPLL_S                       U(2)

#define QRK3588_SPLL_M                       U(234)
#define QRK3588_SPLL_P                       U(2)
#define QRK3588_SPLL_S                       U(3)

#define QRK3588_PLL_INT_RATE(m, p, s) \
	((QRK3588_OSC_HZ * (m)) / (p) / (UL(1) << (s)))

_Static_assert(QRK3588_PLL_INT_RATE(QRK3588_CPLL_M, QRK3588_CPLL_P,
				    QRK3588_CPLL_S) == QRK3588_BOOT_CPLL_HZ,
	       "CPLL boot rate mismatch");
_Static_assert(QRK3588_PLL_INT_RATE(QRK3588_GPLL_M, QRK3588_GPLL_P,
				    QRK3588_GPLL_S) == QRK3588_BOOT_GPLL_HZ,
	       "GPLL boot rate mismatch");
_Static_assert(QRK3588_PLL_INT_RATE(QRK3588_LPLL_M, QRK3588_LPLL_P,
				    QRK3588_LPLL_S) == QRK3588_BOOT_LPLL_HZ,
	       "LPLL boot rate mismatch");
_Static_assert(QRK3588_PLL_INT_RATE(QRK3588_SPLL_M, QRK3588_SPLL_P,
				    QRK3588_SPLL_S) == QRK3588_BOOT_SPLL_HZ,
	       "SPLL boot rate mismatch");
_Static_assert(QRK3588_OSC_HZ / QRK3588_BOOT_CORE0_DIV ==
		QRK3588_BOOT_CPU_PREPLL_HZ, "A55 pre-PLL rate mismatch");
_Static_assert(QRK3588_BOOT_LPLL_HZ / QRK3588_BOOT_CORE0_DIV ==
		QRK3588_BOOT_CPU_L0_HZ, "A55 boot rate mismatch");

/* Lock timeout is a bring-up fault guard, not a PLL timing specification. */
#define QRK3588_PLL_LOCK_POLL_MAX            U(100000)

struct qrk3588_pll_hw {
	uintptr_t con0;
	uintptr_t con1;
	uintptr_t con2;
	uintptr_t con6;
	bool has_fraction;
};

struct qrk3588_pll_cfg {
	uint16_t m;
	uint8_t p;
	uint8_t s;
	uint16_t k;
};

static const struct qrk3588_pll_hw cpll_hw = {
	.con0 = QRK3588_CPLL_CON(0),
	.con1 = QRK3588_CPLL_CON(1),
	.con2 = QRK3588_CPLL_CON(2),
	.con6 = QRK3588_CPLL_CON(6),
	.has_fraction = true,
};

static const struct qrk3588_pll_hw gpll_hw = {
	.con0 = QRK3588_GPLL_CON(0),
	.con1 = QRK3588_GPLL_CON(1),
	.con2 = QRK3588_GPLL_CON(2),
	.con6 = QRK3588_GPLL_CON(6),
	.has_fraction = true,
};

static const struct qrk3588_pll_hw lpll_hw = {
	.con0 = QRK3588_LPLL_CON(0),
	.con1 = QRK3588_LPLL_CON(1),
	.con2 = 0U,
	.con6 = QRK3588_LPLL_CON(6),
	.has_fraction = false,
};

static const struct qrk3588_pll_hw spll_hw = {
	.con0 = QRK3588_SPLL_CON(0),
	.con1 = QRK3588_SPLL_CON(1),
	.con2 = 0U,
	.con6 = QRK3588_SPLL_CON(6),
	.has_fraction = false,
};

static const struct qrk3588_pll_cfg cpll_cfg = {
	.m = QRK3588_CPLL_M,
	.p = QRK3588_CPLL_P,
	.s = QRK3588_CPLL_S,
	.k = 0U,
};

static const struct qrk3588_pll_cfg gpll_cfg = {
	.m = QRK3588_GPLL_M,
	.p = QRK3588_GPLL_P,
	.s = QRK3588_GPLL_S,
	.k = 0U,
};

static const struct qrk3588_pll_cfg lpll_cfg = {
	.m = QRK3588_LPLL_M,
	.p = QRK3588_LPLL_P,
	.s = QRK3588_LPLL_S,
	.k = 0U,
};

static const struct qrk3588_pll_cfg spll_cfg = {
	.m = QRK3588_SPLL_M,
	.p = QRK3588_SPLL_P,
	.s = QRK3588_SPLL_S,
	.k = 0U,
};

/* --------------------------------------------------------------------------
 * Register helpers
 * -------------------------------------------------------------------------- */

static inline void qrk3588_hiword_write(uintptr_t reg, uint16_t mask,
					uint16_t value)
{
	mmio_write_32(reg, ((uint32_t)mask << 16) | ((uint32_t)value & mask));
}

static inline void qrk3588_write_fields(uintptr_t reg, uint32_t fields)
{
	mmio_write_32(reg, fields);
}

static inline void qrk3588_gate_enable(uintptr_t reg, uint16_t mask)
{
	/* Gate bits are active-high disable. */
	qrk3588_hiword_write(reg, mask, 0U);
}

static inline void qrk3588_gate_disable(uintptr_t reg, uint16_t mask)
{
	qrk3588_hiword_write(reg, mask, mask);
}

static bool qrk3588_pll_wait_lock(const struct qrk3588_pll_hw *pll)
{
	const uint32_t lock = U(1) << QRK3588_PLL_LOCK_SHIFT;

	for (uint32_t i = 0U; i < QRK3588_PLL_LOCK_POLL_MAX; ++i) {
		if ((mmio_read_32(pll->con6) & lock) != 0U)
			return true;
	}

	return false;
}

static bool qrk3588_pll_program(const struct qrk3588_pll_hw *pll,
				const struct qrk3588_pll_cfg *cfg)
{
	const uint16_t con0_mask = (uint16_t)(QRK3588_FIELD_MASK(PLL_M) |
					       QRK3588_FIELD_MASK(PLL_BYPASS));
	const uint16_t con0_value = (uint16_t)(QRK3588_FIELD_VALUE(PLL_M, cfg->m) |
						 QRK3588_FIELD_VALUE(PLL_BYPASS,
							QRK3588_PLL_BYPASS_DISABLE));
	const uint16_t con1_cfg_mask = (uint16_t)(QRK3588_FIELD_MASK(PLL_P) |
						   QRK3588_FIELD_MASK(PLL_S));
	const uint16_t con1_cfg_value = (uint16_t)(QRK3588_FIELD_VALUE(PLL_P, cfg->p) |
						    QRK3588_FIELD_VALUE(PLL_S, cfg->s));
	const uint16_t pd_mask = (uint16_t)QRK3588_FIELD_MASK(PLL_POWERDOWN);

	/* Public MODE muxes stay on OSC, so changing the primitive cannot reach users. */
	qrk3588_hiword_write(pll->con1, pd_mask,
		(uint16_t)QRK3588_FIELD_VALUE(PLL_POWERDOWN,
						 QRK3588_PLL_POWER_DOWN));

	qrk3588_hiword_write(pll->con0, con0_mask, con0_value);
	qrk3588_hiword_write(pll->con1, con1_cfg_mask, con1_cfg_value);

	/* FRACPLL K occupies the ordinary lower 16 bits of CON2. */
	if (pll->has_fraction)
		mmio_write_32(pll->con2, cfg->k);

	/* CON1[13]: 1 = power-down, 0 = run. */
	qrk3588_hiword_write(pll->con1, pd_mask,
		(uint16_t)QRK3588_FIELD_VALUE(PLL_POWERDOWN,
						 QRK3588_PLL_POWER_UP));

	return qrk3588_pll_wait_lock(pll);
}

/* --------------------------------------------------------------------------
 * Phase 1: move live paths onto a deterministic 24 MHz oscillator baseline
 * -------------------------------------------------------------------------- */

static void qrk3588_enable_live_infrastructure(void)
{
	/*
	 * These paths are already needed while clock setup itself is executing;
	 * they cannot be gated off and then repaired later.
	 */
	qrk3588_gate_enable(QRK3588_CRU_GATE_MATRIX,
		QRK3588_GATE_MATRIX_100M | QRK3588_GATE_MATRIX_150M |
		QRK3588_GATE_MATRIX_200M | QRK3588_GATE_MATRIX_350M |
		QRK3588_GATE_MATRIX_400M | QRK3588_GATE_MATRIX_500M);

	/* TOP APB carries UART/I2C/SPI/GPIO/PMU/CRU register accesses. */
	qrk3588_gate_enable(QRK3588_CRU_GATE_TOP_ROOTS,
		QRK3588_GATE_PCLK_TOP_ROOT | QRK3588_GATE_PCLK_TOP_BIU);

	/* BUS AXI carries GIC/system DMA/decompressor/spinlock traffic. */
	qrk3588_gate_enable(QRK3588_CRU_GATE_BUS,
		QRK3588_GATE_ACLK_BUS_ROOT | QRK3588_GATE_ACLK_BUS_BIU);

	/* SBUS carries system-control, JTAG/DAP and timer control traffic. */
	qrk3588_gate_enable(QRK3588_SBUSCRU_GATE_INFRA,
		QRK3588_GATE_PCLK_SBUS_ROOT | QRK3588_GATE_PCLK_SBUS_BIU |
		QRK3588_GATE_PCLK_SBUS_CRU | QRK3588_GATE_PCLK_SBUS_SGRF |
		QRK3588_GATE_PCLK_JDBCK_DAP | QRK3588_GATE_CLK_JDBCK_DAP |
		QRK3588_GATE_MATRIX_SBUS | QRK3588_GATE_PCLK_STIMER1 |
		QRK3588_GATE_CLK_SBUS_TIMER);

	/* DSU coherent/function plane, control/debug plane and executing A55 path. */
	qrk3588_gate_enable(QRK3588_DSUCRU_GATE_FUNCTIONAL,
		QRK3588_GATE_SCLK_DSU_DF_SRC | QRK3588_GATE_SCLK_DSU_SRC |
		QRK3588_GATE_SCLK_DSU_SRC_T | QRK3588_GATE_SCLK_DSU |
		QRK3588_GATE_ACLKM_DSU | QRK3588_GATE_ACLKS_DSU |
		QRK3588_GATE_ACLKMP_DSU | QRK3588_GATE_PERIPHCLK_DSU |
		QRK3588_GATE_CNTCLK_DSU | QRK3588_GATE_TSCLK_DSU);
	qrk3588_gate_enable(QRK3588_DSUCRU_GATE_CONTROL,
		QRK3588_GATE_PCLK_DSU_ROOT | QRK3588_GATE_PCLK_DSU_NS_ROOT |
		QRK3588_GATE_PCLK_DSU_BIU | QRK3588_GATE_PCLK_DSU |
		QRK3588_GATE_PCLK_DBG | QRK3588_GATE_PCLK_S_DAPLITE |
		QRK3588_GATE_PCLK_M_DAPLITE | QRK3588_GATE_PCLK_M_DAPLITE_BIU |
		QRK3588_GATE_PCLK_DSU_GRF);
	qrk3588_gate_enable(QRK3588_DSUCRU_GATE_CORE,
		QRK3588_GATE_CLK_CORE_L | QRK3588_GATE_CLK_CORE_L0_UC |
		QRK3588_GATE_ACLK_MP_DSU_BIU | QRK3588_GATE_GICCLK_DSU);

	/* SecureCRU itself is programmed below, so its bus fabric must remain live. */
	qrk3588_gate_enable(QRK3588_SECURECRU_GATE_INFRA,
		QRK3588_GATE_SEC_MATRIX_58 | QRK3588_GATE_SEC_MATRIX_116 |
		QRK3588_GATE_SEC_MATRIX_175 | QRK3588_GATE_SEC_MATRIX_233 |
		QRK3588_GATE_SEC_MATRIX_350 | QRK3588_GATE_ACLK_SECURE_NS_ROOT |
		QRK3588_GATE_HCLK_SECURE_NS_ROOT | QRK3588_GATE_ACLK_SECURE_S_ROOT |
		QRK3588_GATE_HCLK_SECURE_S_ROOT | QRK3588_GATE_PCLK_SECURE_S_ROOT |
		QRK3588_GATE_ACLK_SECURE_NS_BIU | QRK3588_GATE_HCLK_SECURE_NS_BIU |
		QRK3588_GATE_ACLK_SECURE_S_BIU | QRK3588_GATE_HCLK_SECURE_S_BIU |
		QRK3588_GATE_PCLK_SECURE_S_BIU);
}

void qrk3588_clock_enter_osc_safe_state(void)
{
	/*
	 * MODE muxes are the isolation boundary.  Force them to OSC first so every
	 * later M/P/S change is hidden from the live clock tree.
	 */
	qrk3588_write_fields(QRK3588_CRU_MODE_CON,
		QRK3588_FIELD(CLK_CPLL_MODE, QRK3588_PLL_MODE_OSC) |
		QRK3588_FIELD(CLK_GPLL_MODE, QRK3588_PLL_MODE_OSC));
	qrk3588_write_fields(QRK3588_DSUCRU_MODE_CON,
		QRK3588_FIELD(CLK_LPLL_MODE, QRK3588_PLL_MODE_OSC));
	qrk3588_write_fields(QRK3588_SBUSCRU_MODE_CON,
		QRK3588_FIELD(CLK_SPLL_MODE, QRK3588_PLL_MODE_OSC));

	qrk3588_enable_live_infrastructure();

	/*
	 * A55 boot core: use the LPLL public path while LPLL MODE still outputs
	 * XIN24M.  core0 UC /1 therefore gives a known 24 MHz execution clock.
	 */
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_CORE_L_SRC,
		QRK3588_FIELD(CLK_CORE_L_SRC_SEL, QRK3588_CLK_CORE_L_LPLL));
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_CORE_L0,
		QRK3588_FIELD(CLK_CORE_L0_UC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_OSC_SAFE_DIV)) |
		QRK3588_FIELD(CLK_CORE_L0_SEL, QRK3588_CLK_CORE_L0_UC));

	/* Keep the currently used control/data buses on direct or OSC-derived paths. */
	qrk3588_write_fields(QRK3588_CRU_CLKSEL_BUS_ROOT,
		QRK3588_FIELD(ACLK_BUS_ROOT_SEL, QRK3588_ACLK_BUS_CPLL) |
		QRK3588_FIELD(ACLK_BUS_ROOT_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_OSC_SAFE_DIV)));
	qrk3588_write_fields(QRK3588_CRU_CLKSEL_TOP_PCLK,
		QRK3588_FIELD(PCLK_TOP_ROOT_SEL, QRK3588_PCLK_TOP_OSC));

	qrk3588_write_fields(QRK3588_SBUSCRU_CLKSEL_ROOTS,
		QRK3588_FIELD(PCLK_SBUS_ROOT_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_OSC_SAFE_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_SBUS_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_OSC_SAFE_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_SBUS_SEL, QRK3588_MATRIX_SBUS_SPLL) |
		QRK3588_FIELD(CLK_SBUS_TIMER_SEL, QRK3588_SBUS_TIMER_OSC));

	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_DSU_SRC,
		QRK3588_FIELD(SCLK_DSU_DF_SRC_SEL, QRK3588_SCLK_DSU_GPLL) |
		QRK3588_FIELD(SCLK_DSU_DF_SRC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_OSC_SAFE_DIV)));
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_DSU_PATH,
		QRK3588_FIELD(SCLK_DSU_SRC_T_SEL, QRK3588_SCLK_DSU_SRC_T_NORMAL));
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_PCLK,
		QRK3588_FIELD(PCLK_DSU_ROOT_SEL, QRK3588_PCLK_DSU_LPLL) |
		QRK3588_FIELD(PCLK_DSU_ROOT_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_OSC_SAFE_DIV)) |
		QRK3588_FIELD(PCLK_DSU_NS_ROOT_SEL, QRK3588_PCLK_DSU_NS_OSC));

	/* Secure AXI/AHB/APB roots remain on direct OSC during the safe phase. */
	qrk3588_write_fields(QRK3588_SECURECRU_CLKSEL_ROOTS,
		QRK3588_FIELD(ACLK_SECURE_NS_ROOT_SEL, QRK3588_ACLK_SECURE_NS_OSC) |
		QRK3588_FIELD(HCLK_SECURE_NS_ROOT_SEL, QRK3588_HCLK_SECURE_NS_OSC) |
		QRK3588_FIELD(ACLK_SECURE_S_ROOT_SEL, QRK3588_ACLK_SECURE_S_OSC) |
		QRK3588_FIELD(HCLK_SECURE_S_ROOT_SEL, QRK3588_HCLK_SECURE_S_OSC) |
		QRK3588_FIELD(PCLK_SECURE_S_ROOT_SEL, QRK3588_PCLK_SECURE_S_OSC));
}

/* --------------------------------------------------------------------------
 * Phase 2: preconfigure the final mux/divider tree while every PLL MODE is OSC
 * -------------------------------------------------------------------------- */

static void qrk3588_preconfigure_main_matrix(void)
{
	/*
	 * The parent muxes already point at CPLL/GPLL/SPLL public outputs, but those
	 * public outputs are still XIN24M.  Divider changes can therefore only slow
	 * the temporary clock; they cannot overspeed a consumer.
	 */
	qrk3588_write_fields(QRK3588_CRU_CLKSEL_MATRIX_100_50,
		QRK3588_FIELD(CLK_MATRIX_100M_SRC_SEL, QRK3588_MATRIX_SRC_CPLL) |
		QRK3588_FIELD(CLK_MATRIX_100M_SRC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_MATRIX_100_DIV)));

	qrk3588_write_fields(QRK3588_CRU_CLKSEL_MATRIX_200_150,
		QRK3588_FIELD(CLK_MATRIX_200M_SRC_SEL, QRK3588_MATRIX_SRC_GPLL) |
		QRK3588_FIELD(CLK_MATRIX_200M_SRC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_MATRIX_200_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_150M_SRC_SEL, QRK3588_MATRIX_SRC_CPLL) |
		QRK3588_FIELD(CLK_MATRIX_150M_SRC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_MATRIX_150_DIV)));

	qrk3588_write_fields(QRK3588_CRU_CLKSEL_MATRIX_400_350,
		QRK3588_FIELD(CLK_MATRIX_400M_SRC_SEL, QRK3588_MATRIX_SRC_GPLL) |
		QRK3588_FIELD(CLK_MATRIX_400M_SRC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_MATRIX_400_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_350M_SRC_SEL, QRK3588_MATRIX_350_SRC_SPLL) |
		QRK3588_FIELD(CLK_MATRIX_350M_SRC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_MATRIX_350_DIV)));

	qrk3588_write_fields(QRK3588_CRU_CLKSEL_MATRIX_500_450,
		QRK3588_FIELD(CLK_MATRIX_500M_SRC_SEL, QRK3588_MATRIX_SRC_CPLL) |
		QRK3588_FIELD(CLK_MATRIX_500M_SRC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_MATRIX_500_DIV)));

	/* Secure matrix dividers are explicit even though their upstream is fixed. */
	qrk3588_write_fields(QRK3588_SECURECRU_CLKSEL_MATRIX,
		QRK3588_FIELD(CLK_MATRIX_SEC_58M_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_SEC_58_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_SEC_116M_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_SEC_116_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_SEC_175M_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_SEC_175_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_SEC_233M_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_SEC_233_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_SEC_350M_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_SEC_350_DIV)));
}

void qrk3588_clock_preconfigure_boot_tree(void)
{
	qrk3588_preconfigure_main_matrix();

	/*
	 * A55 core0 is already on the LPLL public path.  Since LPLL MODE is OSC,
	 * installing the final /2 now changes only 24 -> 12 MHz.  When LPLL MODE
	 * is released later, the same path becomes 900 /2 = 450 MHz with no
	 * additional core mux/divider write.
	 */
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_CORE_L0,
		QRK3588_FIELD(CLK_CORE_L0_UC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_CORE0_DIV)) |
		QRK3588_FIELD(CLK_CORE_L0_SEL, QRK3588_CLK_CORE_L0_UC));

	/* BUS AXI: DECOM, DMAC0/1/2, GIC system AXI and hardware spinlock. */
	qrk3588_write_fields(QRK3588_CRU_CLKSEL_BUS_ROOT,
		QRK3588_FIELD(ACLK_BUS_ROOT_SEL, QRK3588_ACLK_BUS_CPLL) |
		QRK3588_FIELD(ACLK_BUS_ROOT_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_BUS_DIV)));

	/* DSU coherent/GIC/trace/counter source and DSU/debug control PCLK. */
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_DSU_SRC,
		QRK3588_FIELD(SCLK_DSU_DF_SRC_SEL, QRK3588_SCLK_DSU_GPLL) |
		QRK3588_FIELD(SCLK_DSU_DF_SRC_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_DSU_DF_DIV)));
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_DSU_PATH,
		QRK3588_FIELD(SCLK_DSU_SRC_T_SEL, QRK3588_SCLK_DSU_SRC_T_NORMAL));
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_PCLK,
		QRK3588_FIELD(PCLK_DSU_ROOT_SEL, QRK3588_PCLK_DSU_LPLL) |
		QRK3588_FIELD(PCLK_DSU_ROOT_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_DSU_PCLK_DIV)) |
		QRK3588_FIELD(PCLK_DSU_NS_ROOT_SEL,
			       QRK3588_PCLK_DSU_NS_MATRIX_100M));

	/*
	 * SBUS system-control/debug: SBUS BIU/CRU/SGRF, JDBCK-DAP and system-timer
	 * control. Program both divider fields before SPLL
	 * release; do not assume a compounded /49 relationship because the
	 * inspected TRM does not explicitly close PCLK_SBUS_ROOT's upstream edge.
	 */
	qrk3588_write_fields(QRK3588_SBUSCRU_CLKSEL_ROOTS,
		QRK3588_FIELD(PCLK_SBUS_ROOT_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_SBUS_PCLK_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_SBUS_DIV,
			       QRK3588_DIV_ENCODE(QRK3588_BOOT_SBUS_MATRIX_DIV)) |
		QRK3588_FIELD(CLK_MATRIX_SBUS_SEL, QRK3588_MATRIX_SBUS_SPLL) |
		QRK3588_FIELD(CLK_SBUS_TIMER_SEL, QRK3588_SBUS_TIMER_OSC));

	/*
	 * TOP APB/control plane: UART/I2C/SPI/GPIO/PWM/CAN/ADC/WDT, mailbox/PMU,
	 * CRU and PHY configuration interfaces.  Peripheral functional clocks are
	 * separate; only register/control access follows this root.
	 */
	qrk3588_write_fields(QRK3588_CRU_CLKSEL_TOP_PCLK,
		QRK3588_FIELD(PCLK_TOP_ROOT_SEL, QRK3588_PCLK_TOP_MATRIX_100M));

	/* TOP S200/S400 AXI/BIU branches are gated off until the profile is live. */
	qrk3588_write_fields(QRK3588_CRU_CLKSEL_TOP_S,
		QRK3588_FIELD(ACLK_TOP_S200_ROOT_SEL,
			       QRK3588_ACLK_TOP_S200_MATRIX_200M) |
		QRK3588_FIELD(ACLK_TOP_S400_ROOT_SEL,
			       QRK3588_ACLK_TOP_S400_MATRIX_400M));

	/*
	 * CENTER memory-side AXI fabric, also gated off here:
	 *   LOW  -> DDR shared-memory AXI path
	 *   S200 -> CENTER S200 BIU/interconnect branch
	 *   S400 -> CENTER S400 BIU/interconnect branch
	 * DDR controller/PHY functional clocks are separate.
	 */
	qrk3588_write_fields(QRK3588_CRU_CLKSEL_CENTER_ROOTS,
		QRK3588_FIELD(ACLK_CENTER_LOW_ROOT_SEL,
			       QRK3588_ACLK_CENTER_LOW_MATRIX_500M) |
		QRK3588_FIELD(ACLK_CENTER_S200_ROOT_SEL,
			       QRK3588_ACLK_CENTER_S200_MATRIX_200M) |
		QRK3588_FIELD(ACLK_CENTER_S400_ROOT_SEL,
			       QRK3588_ACLK_CENTER_S400_MATRIX_400M));
}

static bool qrk3588_cpll_program(void)
{
	bool locked;
	const struct qrk3588_pll_cfg * const cfg = &cpll_cfg;

	/*
	 * RK3588 CPLL quirk:
	 * clk_bisrintf_pllsrc must be gated while the CPLL primitive is
	 * powered down and restarted.
	 */
	qrk3588_gate_disable(QRK3588_CRU_GATE_CON19,
			     QRK3588_GATE_BISRINTF_PLLSRC);

	locked = qrk3588_pll_program(&cpll_hw, cfg);

	if (locked) {
		qrk3588_gate_enable(QRK3588_CRU_GATE_CON19,
				    QRK3588_GATE_BISRINTF_PLLSRC);
	}

	return locked;
}

static bool qrk3588_lpll_program(void)
{
	const struct qrk3588_pll_cfg * const cfg = &lpll_cfg;
	/*
	 * RK3588 LPLL quirk:
	 * explicitly move the A55 core clock away from LPLL before
	 * restarting the LPLL primitive.
	 */
	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_CORE_L_SRC,
		QRK3588_FIELD(CLK_CORE_L_SRC_SEL,
			      QRK3588_CLK_CORE_L_SLOW));

	if (!qrk3588_pll_program(&lpll_hw, cfg))
		return false;

	qrk3588_write_fields(QRK3588_DSUCRU_CLKSEL_CORE_L_SRC,
		QRK3588_FIELD(CLK_CORE_L_SRC_SEL,
			      QRK3588_CLK_CORE_L_LPLL));

	return true;
}

/* --------------------------------------------------------------------------
 * Phase 3: program and lock PLL primitives; MODE muxes still output OSC
 * -------------------------------------------------------------------------- */

int qrk3588_clock_program_boot_plls(void)
{
	if (!qrk3588_cpll_program())
		return QRK3588_CLOCK_ERR_CPLL_LOCK;

	if (!qrk3588_pll_program(&gpll_hw, &gpll_cfg))
		return QRK3588_CLOCK_ERR_GPLL_LOCK;

	if (!qrk3588_lpll_program())
		return QRK3588_CLOCK_ERR_LPLL_LOCK;

	if (!qrk3588_pll_program(&spll_hw, &spll_cfg))
		return QRK3588_CLOCK_ERR_SPLL_LOCK;
	return 0;
}

/* --------------------------------------------------------------------------
 * Phase 4: release locked PLLs into the already prepared clock tree
 * -------------------------------------------------------------------------- */

void qrk3588_clock_release_boot_plls(void)
{
	/*
	 * Downstream parent/divider state is already final.  CPLL/GPLL/SPLL are
	 * released first; LPLL is last because its MODE transition is the exact
	 * point where the executing A55 changes from 12 MHz to 450 MHz.
	 */
	qrk3588_write_fields(QRK3588_CRU_MODE_CON,
		QRK3588_FIELD(CLK_CPLL_MODE, QRK3588_PLL_MODE_PLL) |
		QRK3588_FIELD(CLK_GPLL_MODE, QRK3588_PLL_MODE_PLL));
	qrk3588_write_fields(QRK3588_SBUSCRU_MODE_CON,
		QRK3588_FIELD(CLK_SPLL_MODE, QRK3588_PLL_MODE_PLL));

	qrk3588_write_fields(QRK3588_DSUCRU_MODE_CON,
		QRK3588_FIELD(CLK_LPLL_MODE, QRK3588_PLL_MODE_PLL));
}

void qrk3588_clock_activate_secure_roots(void)
{
	/*
	 * Secure S/NS bus fabric is switched only after every boot PLL is locked
	 * and its public MODE mux has been released:
	 *   NS AXI -> general matrix350
	 *   NS AHB -> general matrix150
	 *   S  AXI -> secure matrix350
	 *   S  AHB -> secure matrix175
	 *   S  APB -> secure matrix116
	 * This affects the secure/OTP/crypto/key-ladder/timer/WDT bus/control
	 * fabric; the individual IP functional clocks remain separate.
	 */
	qrk3588_write_fields(QRK3588_SECURECRU_CLKSEL_ROOTS,
		QRK3588_FIELD(ACLK_SECURE_NS_ROOT_SEL,
			       QRK3588_ACLK_SECURE_NS_MATRIX_350M) |
		QRK3588_FIELD(HCLK_SECURE_NS_ROOT_SEL,
			       QRK3588_HCLK_SECURE_NS_MATRIX_150M) |
		QRK3588_FIELD(ACLK_SECURE_S_ROOT_SEL,
			       QRK3588_ACLK_SECURE_S_MATRIX_350M) |
		QRK3588_FIELD(HCLK_SECURE_S_ROOT_SEL,
			       QRK3588_HCLK_SECURE_S_MATRIX_175M) |
		QRK3588_FIELD(PCLK_SECURE_S_ROOT_SEL,
			       QRK3588_PCLK_SECURE_S_MATRIX_116M));
}
