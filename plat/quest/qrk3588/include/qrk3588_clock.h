/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_CLOCK_H
#define QRK3588_CLOCK_H

#include <lib/utils_def.h>

#define QRK3588_OSC_HZ                     UL(24000000)
#define QRK3588_OSC_MHZ                    U(24)

#define QRK3588_BOOT_CPLL_HZ               UL(750000000)
#define QRK3588_BOOT_GPLL_HZ               UL(600000000)
#define QRK3588_BOOT_LPLL_HZ               UL(900000000)
#define QRK3588_BOOT_SPLL_HZ               UL(351000000)

/* core0 uses the LPLL public path /2 before and after LPLL MODE release. */
#define QRK3588_BOOT_CPU_PREPLL_HZ          UL(12000000)
#define QRK3588_BOOT_CPU_PREPLL_MHZ         U(12)
#define QRK3588_BOOT_CPU_L0_HZ             UL(450000000)
#define QRK3588_BOOT_CPU_L0_MHZ             U(450)

/*
 * Required order:
 *   1) enter_osc_safe_state()      : PLL MODE -> OSC; boot core = 24 MHz
 *   2) preconfigure_boot_tree()    : final proven mux/div; boot core = 12 MHz
 *   3) program_boot_plls()         : M/P/S/K + LOCK; MODE still OSC
 *   4) release_boot_plls()         : public MODE -> PLL; boot core = 450 MHz
 *   5) activate_secure_roots()     : Secure roots leave direct OSC
 *   6) enable_boot_branches()      : ungate inactive TOP/CENTER data paths
 */
void qrk3588_clock_enter_osc_safe_state(void);
void qrk3588_clock_preconfigure_boot_tree(void);
int qrk3588_clock_program_boot_plls(void);
void qrk3588_clock_release_boot_plls(void);
void qrk3588_clock_activate_secure_roots(void);
void qrk3588_clock_enable_boot_branches(void);

enum qrk3588_clock_result {
	QRK3588_CLOCK_OK = 0,
	QRK3588_CLOCK_ERR_CPLL_LOCK = -1,
	QRK3588_CLOCK_ERR_GPLL_LOCK = -2,
	QRK3588_CLOCK_ERR_LPLL_LOCK = -3,
	QRK3588_CLOCK_ERR_SPLL_LOCK = -4,
};

#endif /* QRK3588_CLOCK_H */
