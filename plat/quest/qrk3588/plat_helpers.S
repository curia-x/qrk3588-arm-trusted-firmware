/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <asm_macros.S>
#include <assert_macros.S>
#include <platform_def.h>

.globl platform_mem_init
.globl plat_reset_handler
.globl plat_my_core_pos
.globl plat_core_pos_by_mpidr
.globl plat_arm_calc_core_pos

/* ---------------------------------------------------------------------
 * We don't need to carry out any memory initialization on ARM
 * platforms. The Secure RAM is accessible straight away.
 * ---------------------------------------------------------------------
 */
func platform_mem_init
	ret
endfunc platform_mem_init

func plat_reset_handler
	ret
endfunc plat_reset_handler

/*
 * Map MPIDR in x0 to a linear CPU index.
 * Branches to \invalid on failure.
 *
 * \tmp must be a 64-bit general-purpose register other than x0
 */
.macro mpidr_to_core_pos tmp, invalid
	/* Only Aff1 identifies a core; Aff0, Aff2 and Aff3 must be zero. */
	ubfx	\tmp, x0, #32, #MPIDR_AFFINITY_BITS
	cbnz	\tmp, \invalid
	tst	 w0, #0x00ff00ff
	b.ne	\invalid
	ubfx	x0, x0, #MPIDR_AFFINITY_BITS, #MPIDR_AFFINITY_BITS
	cmp	 x0, #PLATFORM_CORE_COUNT
	b.hs	\invalid
.endm

/* -------------------------------------------------
 * unsigned int plat_my_core_pos(void)
 * Out: x0 - index of the calling CPU
 *
 * Called before a stack is available, including BL31 warm entry.
 * -------------------------------------------------
 */
func plat_my_core_pos
	mrs	x0, mpidr_el1

	mpidr_to_core_pos x1, 1f
	ret
1:
	/* No stack exists yet: never return an invalid per-CPU index. */
	wfe
	b	1b
endfunc plat_my_core_pos

/* -------------------------------------------------
 * int plat_core_pos_by_mpidr(u_register_t mpidr)
 * Out: x0 - index of the calling CPU or -1 if the MPIDR is invalid
 * -------------------------------------------------
 */
func plat_core_pos_by_mpidr
	mpidr_to_core_pos x1, 1f
	ret
1:
	mov	 w0, #-1
	ret
endfunc plat_core_pos_by_mpidr

/* -------------------------------------------------
 * unsigned int plat_arm_calc_core_pos(u_register_t mpidr)
 * Out: x0 - index of the calling CPU or PLATFORM_CORE_COUNT if the MPIDR is invalid
 * -------------------------------------------------
 */
func plat_arm_calc_core_pos
	mpidr_to_core_pos x1, 1f
	ret
1:
	mov	 w0, #PLATFORM_CORE_COUNT
	ret
endfunc plat_arm_calc_core_pos
