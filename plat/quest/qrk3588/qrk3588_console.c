/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdint.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/console.h>
#include <drivers/ti/uart/uart_16550.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>
#include <plat/common/platform.h>

#define QRK3588_UART2_BASE			UL(0xfeb50000)
#define QRK3588_OSC_FREQ_HZ			U(24000000)

#define EARLY_UART_CLK_HZ			QRK3588_OSC_FREQ_HZ
#define EARLY_CONSOLE_BAUDRATE		U(1500000)

#define QRK3588_CRU_BASE			UL(0xfd7c0000)
#define CRU_CLKSEL_CON45			(QRK3588_CRU_BASE + UL(0x03b4))
#define CRU_CLKGATE_CON12			(QRK3588_CRU_BASE + UL(0x0830))
#define CRU_CLKGATE_CON13			(QRK3588_CRU_BASE + UL(0x0834))
#define CRU_SOFTRST_CON12			(QRK3588_CRU_BASE + UL(0x0a30))

#define PCLK_UART2_GATE_SHIFT			U(3)
#define PCLK_UART2_GATE_MASK			U(0x1)

#define SCLK_UART2_GATE_SHIFT			U(0)
#define SCLK_UART2_GATE_MASK			U(0x1)

#define SCLK_UART2_SEL_SHIFT			U(0)
#define SCLK_UART2_SEL_MASK			U(0x3)
#define SCLK_UART2_SEL_XIN24M			U(0x2)

#define UART2_RESET_SHIFT			U(3)
#define UART2_RESET_MASK			U(0x1)

#define QRK3588_PMU2_IOC_BASE			UL(0xfd5f4000)
#define QRK3588_BUS_IOC_BASE			UL(0xfd5f8000)

#define PMU2_IOC_GPIO0B_IOMUX_SEL_H		(QRK3588_PMU2_IOC_BASE + UL(0x0000))
#define BUS_IOC_GPIO0B_IOMUX_SEL_H		(QRK3588_BUS_IOC_BASE + UL(0x000c))

#define GPIO0B5_IOMUX_SHIFT			U(4)
#define GPIO0B6_IOMUX_SHIFT			U(8)
#define GPIO_IOMUX_MASK				U(0xf)

#define GPIO0B5_REFER_BUS_IOC			U(0x8)
#define GPIO0B6_REFER_BUS_IOC			U(0x8)

#define GPIO0B5_UART2_TX_M0			U(0xa)
#define GPIO0B6_UART2_RX_M0			U(0xa)

static console_t early_console;

/* These are existing TI 16550 low-level assembly entry points. */
extern int console_16550_core_putc(int character, uintptr_t baseaddr);
extern int console_16550_core_getc(uintptr_t baseaddr);

int qrk3588_console_putc(int character)
{
	return console_16550_core_putc(character, QRK3588_UART2_BASE);
}

int qrk3588_console_getc(void)
{
	return console_16550_core_getc(QRK3588_UART2_BASE);
}

/*
 * RK3588 CRU/IOC registers use hiword write-mask semantics:
 *
 * [31:16] write mask
 * [15:0]  value
 */
static inline void qrk3588_field_write(uintptr_t reg, uint32_t shift,
				       uint32_t mask, uint32_t value)
{
	uint32_t field_mask = mask << shift;
	uint32_t field_value = (value & mask) << shift;

	mmio_write_32(reg, (field_mask << 16) | field_value);
}

static void uart2_pinmux_configure(void)
{
	uint32_t mask;
	uint32_t value;

	mask = (GPIO_IOMUX_MASK << GPIO0B5_IOMUX_SHIFT) |
	       (GPIO_IOMUX_MASK << GPIO0B6_IOMUX_SHIFT);

	/*
	 * GPIO0_B5/B6 hand mux control from PMU2_IOC to BUS_IOC.
	 */
	value = (GPIO0B5_REFER_BUS_IOC << GPIO0B5_IOMUX_SHIFT) |
		(GPIO0B6_REFER_BUS_IOC << GPIO0B6_IOMUX_SHIFT);

	mmio_write_32(PMU2_IOC_GPIO0B_IOMUX_SEL_H,
		      (mask << 16) | value);

	/* GPIO0_B5 -> UART2_TX_M0, GPIO0_B6 -> UART2_RX_M0. */
	value = (GPIO0B5_UART2_TX_M0 << GPIO0B5_IOMUX_SHIFT) |
		(GPIO0B6_UART2_RX_M0 << GPIO0B6_IOMUX_SHIFT);

	mmio_write_32(BUS_IOC_GPIO0B_IOMUX_SEL_H,
		      (mask << 16) | value);
}

static inline void pclk_uart2_enable(void)
{
	qrk3588_field_write(CRU_CLKGATE_CON12,
			    PCLK_UART2_GATE_SHIFT,
			    PCLK_UART2_GATE_MASK,
			    U(0));
}

static inline void sclk_uart2_enable(bool enable)
{
	qrk3588_field_write(CRU_CLKGATE_CON13,
			    SCLK_UART2_GATE_SHIFT,
			    SCLK_UART2_GATE_MASK,
			    enable ? U(0) : U(1));
}

static inline void sclk_uart2_select_xin24m(void)
{
	qrk3588_field_write(CRU_CLKSEL_CON45,
			    SCLK_UART2_SEL_SHIFT,
			    SCLK_UART2_SEL_MASK,
			    SCLK_UART2_SEL_XIN24M);
}

static inline void uart2_reset_deassert(void)
{
	qrk3588_field_write(CRU_SOFTRST_CON12,
			    UART2_RESET_SHIFT,
			    UART2_RESET_MASK,
			    U(0));
}

static void uart2_clock_configure(void)
{
	pclk_uart2_enable();

	/*
	 * Keep the early console independent of PLL configuration by
	 * running SCLK_UART2 directly from xin24m.
	 */
	sclk_uart2_enable(false);
	sclk_uart2_select_xin24m();
	sclk_uart2_enable(true);

	uart2_reset_deassert();
}

static int qrk3588_early_console_register(console_t *console)
{
	if (!console)
		return 0;

	uart2_clock_configure();
	uart2_pinmux_configure();

	return console_16550_register(QRK3588_UART2_BASE,
				      EARLY_UART_CLK_HZ,
				      EARLY_CONSOLE_BAUDRATE,
				      console);
}

void plat_setup_early_console(void)
{
	qrk3588_early_console_register(&early_console);
}

void __dead2 plat_error_handler(int err)
{
	(void)err;

	console_flush();

	while (true)
		wfi();
}
