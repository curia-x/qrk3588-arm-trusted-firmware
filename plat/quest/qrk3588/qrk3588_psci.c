/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <plat/common/platform.h>
#include <platform_def.h>
#include <qrk3588_memory.h>

/* RK3588 TRM Part1 v1.0, sections 2.4.2 and 2.17.3. */
#define CRU_GLB_SRST_FST_VALUE	UL(0xfd7c0c08)
#define CRU_GLB_RST_CON		UL(0xfd7c0c10)
#define GLB_SRST_FST_TRIGGER	U(0xfdb9)
#define GLBRST_TRIG_PMU_EN	BIT(3)
#define GLBRST_TRIG_PMU_SEL	BIT(2)

/*
 * TRM Part1 v1.0, PMU_CPU[0:7]_AUTO_PWR_CON, pp.1142-1149.
 * Keep shared DSU/BIUs and voltage domains running during CPU hotplug.
 */
#define CPU_AUTO_PWR_CON(cpu)	(UL(0xfd8d8010) + (cpu) * U(4))
#define CPU_AUTO_PWRDN_EN	BIT(0)
#define CPU_SW_WAKEUP_EN		BIT(3)
/* All implemented low-power mode/wakeup enables; exclude debug reset policy. */
#define CPU_AUTO_MODE_MASK	(BIT(0) | BIT(1) | BIT(3) | BIT(4) | \
				 BIT(6) | BIT(7) | BIT(8) | BIT(9))
#define CPU_POWER_TIMEOUT_US	U(10000)

/* The ROM mailbox broadcasts the trampoline; only per-CPU READY proves arrival. */
#define CLUSTER_STS		UL(0xfd8d8080)
#define A55_RESET_REG		UL(0xfd818a0c)
#define A76_RESET_REG		UL(0xfd810a00)
#define A76_RESET_REG_STRIDE	UL(0x2000)
#define A76_FIRST_CPU		U(4)
#define BOOT_MAILBOX		(QRK3588_SYSTEM_SRAM_BASE + U(4))
#define BOOT_ENTRY		(QRK3588_SYSTEM_SRAM_BASE + U(8))
#define BOOT_RELEASE		U(0xdeadbeaf)
#define CPU_READY		U(1)
#define CPU_READY_TIMEOUT_US U(10000)
#define TRAMPOLINE_BASE		(QRK3588_SYSTEM_SRAM_BASE + QRK3588_TRAMPOLINE_OFFSET)

extern const unsigned char qrk3588_cpu_start[];
extern const unsigned char qrk3588_cpu_entrypoints[];
extern const unsigned char qrk3588_cpu_ready[];
extern const unsigned char qrk3588_cpu_end[];
static uintptr_t cpu_warm_entrypoint;
static uintptr_t entry_slots;
static uintptr_t ready_flags;

/* Concurrent power-on callers publish the same ROM entry and release token. */
static void qrk3588_publish_boot_mailbox(void)
{
	mmio_write_32(BOOT_ENTRY, (uint32_t)TRAMPOLINE_BASE);
	dmbst();
	mmio_write_32(BOOT_MAILBOX, BOOT_RELEASE);
	dsbsy();
}

static bool qrk3588_cpu_is_ready(unsigned int cpu)
{
	uintptr_t flag = ready_flags + cpu * sizeof(uint32_t);

	/* The Normal NC mapping observes each MMU-off writer directly. */
	return mmio_read_32(flag) == CPU_READY;
}

/* Notify the ROM wait loop until the CPU reaches the trampoline. */
static int qrk3588_ready_cpu(unsigned int cpu)
{
	uint64_t deadline;

	if (qrk3588_cpu_is_ready(cpu))
		return PSCI_E_SUCCESS;

	deadline = timeout_init_us(CPU_READY_TIMEOUT_US);
	for (;;) {
		if (qrk3588_cpu_is_ready(cpu))
			return PSCI_E_SUCCESS;
		if (timeout_elapsed(deadline))
			break;
		/* A late ROM entrant can overwrite the shared release token. */
		qrk3588_publish_boot_mailbox();
		sev();
		udelay(1);
	}

	ERROR("CPU%u: SRAM trampoline arrival timed out\n", cpu);
	return PSCI_E_INTERN_FAIL;
}

void qrk3588_psci_setup(uintptr_t sec_entrypoint)
{
	size_t size = (uintptr_t)qrk3588_cpu_end -
		      (uintptr_t)qrk3588_cpu_start;
	size_t entry_offset = (uintptr_t)qrk3588_cpu_entrypoints -
			    (uintptr_t)qrk3588_cpu_start;
	size_t ready_offset = (uintptr_t)qrk3588_cpu_ready -
			    (uintptr_t)qrk3588_cpu_start;

	cpu_warm_entrypoint = sec_entrypoint;
	entry_slots = TRAMPOLINE_BASE + entry_offset;
	ready_flags = TRAMPOLINE_BASE + ready_offset;
	memcpy((void *)TRAMPOLINE_BASE, qrk3588_cpu_start, size);
	/* Complete NC stores before instruction-cache maintenance/publication. */
	dsbsy();
	/* This reaches the Inner Shareable domain, not a parked CPU's frontend. */
	__asm__ volatile ("ic ialluis" : : : "memory");
	dsbsy();
}

/* CPU_ON may observe AFF_STATE_OFF before the final power-down WFI. */
static int qrk3588_wait_cpu_power(unsigned int cpu, bool on)
{
	uint64_t deadline = timeout_init_us(CPU_POWER_TIMEOUT_US);

	while (((mmio_read_32(CLUSTER_STS) & BIT(cpu)) == 0U) != on) {
		if (timeout_elapsed(deadline)) {
			ERROR("CPU%u: power-%s timed out\n", cpu, on ? "on" : "off");
			return PSCI_E_INTERN_FAIL;
		}
		udelay(1);
	}
	return PSCI_E_SUCCESS;
}

static inline int qrk3588_wait_cpu_power_on(unsigned int cpu)
{
	return qrk3588_wait_cpu_power(cpu, true);
}

static inline int qrk3588_wait_cpu_power_off(unsigned int cpu)
{
	return qrk3588_wait_cpu_power(cpu, false);
}

static int qrk3588_wake_cpu(unsigned int cpu)
{
	uintptr_t reg = CPU_AUTO_PWR_CON(cpu);
	uint32_t control = mmio_read_32(reg);
	int ret;

	/* An armed CPU_OFF must reach OFF before its wakeup is asserted. */
	if ((control & CPU_AUTO_PWRDN_EN) != 0U) {
		ret = qrk3588_wait_cpu_power_off(cpu);
		if (ret != PSCI_E_SUCCESS)
			return ret;
	} else if ((mmio_read_32(CLUSTER_STS) & BIT(cpu)) == 0U) {
		return PSCI_E_SUCCESS;
	}

	/* Invalidate stale SRAM state before the CPU can execute the trampoline. */
	mmio_write_64(entry_slots + cpu * sizeof(uint64_t), 0U);
	mmio_write_32(ready_flags + cpu * sizeof(uint32_t), 0U);
	qrk3588_publish_boot_mailbox();
	/* Software wakeup is a level, not a self-clearing trigger. */
	mmio_write_32(reg, (CPU_SW_WAKEUP_EN << 16) | CPU_SW_WAKEUP_EN);
	ret = qrk3588_wait_cpu_power_on(cpu);
	/* Preserve the OFF request on failure so a retry still waits for OFF. */
	control = CPU_SW_WAKEUP_EN;
	if (ret == PSCI_E_SUCCESS)
		control |= CPU_AUTO_PWRDN_EN;
	mmio_write_32(reg, control << 16);
	dsbsy();
	return ret;
}

/* CPU0's cold reset/ROM path is not yet established for hotplug. */
static int qrk3588_cpu_off_early(const psci_power_state_t *target_state)
{
	(void)target_state;
	return plat_my_core_pos() == 0U ? PSCI_E_DENIED : PSCI_E_SUCCESS;
}

static void qrk3588_cpu_off(const psci_power_state_t *target_state)
{
	unsigned int cpu = plat_my_core_pos();

	(void)target_state;
	assert(cpu < PLATFORM_CORE_COUNT);
	/*
	 * Generic PSCI has already disabled the CPU's GIC and prepared caches.
	 * Clear both words before arming OFF; never reuse a previous warm entry.
	 */
	mmio_write_64(entry_slots + cpu * sizeof(uint64_t), 0U);
	mmio_write_32(ready_flags + cpu * sizeof(uint32_t), 0U);
	dsbsy();
	/*
	 * Only explicit software wakeup may restart an offlined CPU.
	 * Generic PSCI's terminal WFI completes the core power-down handshake.
	 */
	mmio_write_32(CPU_AUTO_PWR_CON(cpu),
		     (CPU_AUTO_MODE_MASK << 16) | CPU_AUTO_PWRDN_EN);
	dsbsy();
}

static int qrk3588_deassert_cpu_reset(unsigned int cpu)
{
	uintptr_t reset_reg;
	uint32_t reset_mask;

	if (cpu >= A76_FIRST_CPU) {
		unsigned int a76 = cpu - A76_FIRST_CPU;

		reset_reg = A76_RESET_REG + (a76 / 2U) * A76_RESET_REG_STRIDE;
		reset_mask = (BIT(4) | BIT(5)) << (4 * (a76 % 2U));
	} else {
		reset_reg = A55_RESET_REG;
		reset_mask = BIT(2 + 3 * cpu) | BIT(3 + 3 * cpu);
	}

	/* Masked write preserves the sibling core's reset state. */
	mmio_write_32(reset_reg, reset_mask << 16);
	dsbsy();
	return PSCI_E_SUCCESS;
}

/* Power-on completes when the CPU is waiting in the trampoline. */
static int qrk3588_power_on_cpu(unsigned int cpu)
{
	int ret;

	/* Generic PSCI locking serializes power-on for the same target CPU. */
	ret = qrk3588_wake_cpu(cpu);
	if (ret != PSCI_E_SUCCESS)
		return ret;

	ret = qrk3588_deassert_cpu_reset(cpu);
	if (ret != PSCI_E_SUCCESS) {
		ERROR("CPU%d: deassert reset failed\n", cpu);
		return ret;
	}

	return qrk3588_ready_cpu(cpu);
}

static int qrk3588_cpu_on(u_register_t mpidr)
{
	int cpu = plat_core_pos_by_mpidr(mpidr);
	uintptr_t slot;
	int ret;

	if (cpu <= 0 || cpu >= PLATFORM_CORE_COUNT || cpu_warm_entrypoint == 0U)
		return PSCI_E_INTERN_FAIL;

	ret = qrk3588_power_on_cpu(cpu);
	if (ret != PSCI_E_SUCCESS)
		return ret;

	/* Release the target CPU via its dedicated entry slot. */
	slot = entry_slots + cpu * sizeof(uint64_t);
	mmio_write_64(slot, cpu_warm_entrypoint);
	dsbsy();
	sev();
	NOTICE("CPU%d: release to warm entry 0x%lx\n", cpu, cpu_warm_entrypoint);
	return PSCI_E_SUCCESS;
}

static void qrk3588_cpu_on_finish(const psci_power_state_t *target_state)
{
	(void)target_state;
	/* TF-A common PSCI code initializes the per-CPU GIC after this callback. */
	NOTICE("CPU: EL3 warm entry, MPIDR 0x%lx\n", read_mpidr_el1());
}

static int qrk3588_validate_ns_entrypoint(uintptr_t entrypoint)
{
	/*
	 * Initial SMP payload is in low non-secure DDR; exclude SRAM/MMIO/BL31.
	 * AArch64 instructions are 4 bytes, so require a 4-byte-aligned entry.
	 */
	if (entrypoint >= QRK3588_DDR_LIMIT || (entrypoint & U(3)) != 0U)
		return PSCI_E_INVALID_ADDRESS;
	return PSCI_E_SUCCESS;
}

static void __dead2 qrk3588_system_reset(void)
{
	/* Include PMU in the first global reset, preserving other sources. */
	mmio_setbits_32(CRU_GLB_RST_CON,
			GLBRST_TRIG_PMU_EN | GLBRST_TRIG_PMU_SEL);
	mmio_write_32(CRU_GLB_SRST_FST_VALUE, GLB_SRST_FST_TRIGGER);
	dsbsy();

	/* Never resume the caller while hardware completes the reset. */
	for (;;)
		wfi();
}

const plat_psci_ops_t qrk3588_plat_psci_ops = {
	.system_reset = qrk3588_system_reset,
	.pwr_domain_off_early = qrk3588_cpu_off_early,
	.pwr_domain_off = qrk3588_cpu_off,
	.pwr_domain_on = qrk3588_cpu_on,
	.pwr_domain_on_finish = qrk3588_cpu_on_finish,
	.validate_ns_entrypoint = qrk3588_validate_ns_entrypoint,
};
