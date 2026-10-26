/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

# This macro allows the crash reporting routine to print relevant
# platform registers in case of an unhandled exception in BL31. This
# aids in debugging and this macro can be defined to be empty in case
# register reporting is not desired.
# For instance, GIC or interconnect registers may be helpful for troubleshooting.
.macro plat_crash_print_regs
      /* TODO */
.endm
