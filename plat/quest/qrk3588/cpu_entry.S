/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <asm_macros.S>
#include <platform_def.h>

/* Copied once to the reserved first SRAM page at offset 0x100. Released CPUs
 * report READY and wait for their entry slot. CPU_ON never rewrites code.
 * Position independent, with no stack or initialized registers required.
 */
.section .rodata.qrk3588_cpu, "a"
.balign 8
.global qrk3588_cpu_start
.global qrk3588_cpu_entrypoints
.global qrk3588_cpu_ready
.global qrk3588_cpu_end
qrk3588_cpu_start:
	mrs	x0, mpidr_el1

	/* Aff3 must be zero. */
	ubfx	x1, x0, #32, #8
	cbnz	x1, 3f

	/* Aff0 and Aff2 must both be zero. */
	tst	w0, #0x00ff00ff
	b.ne	3f

	/* Accept exposed single-threaded affinities 0x100 through 0x700. */
	ubfx	x0, x0, #8, #8
	cbz	x0, 3f
	cmp	x0, #PLATFORM_CORE_COUNT
	b.hs	3f

	/* x1 = &slot[x0] */
	adr	x1, qrk3588_cpu_entrypoints
	add	x1, x1, x0, lsl #3

	/* Report this CPU's arrival before waiting for its entry slot. */
	adr	x2, qrk3588_cpu_ready
	add	x2, x2, x0, lsl #2
	mov	w3, #1
	str	w3, [x2]
	dsb	sy
1:
	/* if (*x1) {jump(2f);} else {wait} */
	ldr	x0, [x1]
	cbnz	x0, 2f
	wfe
	b	1b
2:
	dsb	sy
	isb
	br	x0
3:
	wfe
	b	3b
/* Entry slots require natural alignment for 64-bit loads/stores. */
.balign 8
qrk3588_cpu_entrypoints:
	.fill	PLATFORM_CORE_COUNT, 8, 0
/* One aligned 32-bit READY word per CPU; the publisher maps it Normal NC. */
.balign 4
qrk3588_cpu_ready:
	.fill	PLATFORM_CORE_COUNT, 4, 0
qrk3588_cpu_end:
