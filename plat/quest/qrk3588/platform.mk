#
# Copyright (c) 2026, ninozhang001@gmail.com and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

# QRK3588 authenticates firmware by default. Command-line assignments retain
# GNU make precedence, so TRUSTED_BOARD_BOOT=0 selects the non-authenticated
# build without changing U-Boot, Linux or rootfs authentication policy.
TRUSTED_BOARD_BOOT := 1
ifeq (${TRUSTED_BOARD_BOOT},1)
GENERATE_COT := 1
KEY_SIZE := 3072

# Development fixtures belong to the project build repository. Supply a
# different directory or explicit native key arguments for another trust root;
# missing keys fail the build and are never silently generated or bypassed.
QRK3588_KEY_DIR ?= $(abspath ../build/keys/qrk3588/development)
ROT_KEY ?= ${QRK3588_KEY_DIR}/rot.pem
TRUSTED_WORLD_KEY ?= ${QRK3588_KEY_DIR}/trusted-world.pem
NON_TRUSTED_WORLD_KEY ?= ${QRK3588_KEY_DIR}/non-trusted-world.pem
BL31_KEY ?= ${QRK3588_KEY_DIR}/bl31.pem
BL33_KEY ?= ${QRK3588_KEY_DIR}/bl33.pem

# Keep native make invocations isolated just like the project build driver.
# An explicit BUILD_PLAT retains the caller's chosen path.
BUILD_PLAT := $(abspath ${BUILD_BASE})/qrk3588-secure/${BUILD_TYPE}
endif

include plat/quest/common/quest_common.mk
include lib/xlat_tables_v2/xlat_tables.mk

# Native DDR is split across lifted C translation units.  LTO can discard or
# internalize the BL2 entrypoint before the final cross-unit reference is
# resolved, producing an undefined qrk3588_ddr_init on clean builds.
ENABLE_LTO := 0

PLAT_QRK3588_PATH := $(QUEST_PATH)/qrk3588

QRK3588_BOOT_DEVICE ?= emmc
QRK3588_FSPI_IOMUX ?= 0
QRK3588_GICT_DIAGNOSTICS ?= 0

ifneq ($(QRK3588_GICT_DIAGNOSTICS),0)
ifneq ($(QRK3588_GICT_DIAGNOSTICS),1)
$(error QRK3588_GICT_DIAGNOSTICS must be 0 or 1)
endif
endif

QRK3588_BOOT_EMMC := 0
QRK3588_BOOT_SDMMC := 0
QRK3588_BOOT_SPI_NOR := 0
ifeq (${QRK3588_BOOT_DEVICE},emmc)
QRK3588_BOOT_EMMC := 1
QRK3588_STORAGE_SOURCE := ${PLAT_QRK3588_PATH}/drivers/qrk3588_mmc.c
else ifeq (${QRK3588_BOOT_DEVICE},sd)
QRK3588_BOOT_SDMMC := 1
QRK3588_STORAGE_SOURCE := ${PLAT_QRK3588_PATH}/drivers/qrk3588_mmc.c
else ifeq (${QRK3588_BOOT_DEVICE},spinor)
QRK3588_BOOT_SPI_NOR := 1
QRK3588_STORAGE_SOURCE := ${PLAT_QRK3588_PATH}/drivers/qrk3588_fspi.c
else
$(error QRK3588_BOOT_DEVICE must be emmc, sd or spinor)
endif

$(eval $(call add_defines,\
	QRK3588_BOOT_EMMC QRK3588_BOOT_SDMMC QRK3588_BOOT_SPI_NOR \
	QRK3588_FSPI_IOMUX))
$(eval $(call add_define,QRK3588_GICT_DIAGNOSTICS))

PLAT_INCLUDES += \
	-I. \
	-Iplat/quest/qrk3588/include \
	-Iplat/quest/qrk3588/ddr/include \
	-Iplat/quest/common/include

PLAT_BL_COMMON_SOURCES += \
	lib/cpus/aarch64/cortex_a55.S   \
	lib/cpus/aarch64/cortex_a76.S   \
	drivers/ti/uart/aarch64/16550_console.S \
	plat/quest/qrk3588/qrk3588_console.c \
	${XLAT_TABLES_LIB_SRCS} \
	plat/common/aarch64/crash_console_helpers.S     \

BL1_SOURCES += \
	plat/quest/common/quest_xmodem.c \
	plat/quest/qrk3588/qrk3588_bl1_setup.c \
	plat/quest/qrk3588/qrk3588_bootstrap.c \
	plat/quest/qrk3588/qrk3588_fwu.c \
	plat/quest/qrk3588/plat_helpers.S \
	plat/quest/qrk3588/qrk3588_clock.c \
	drivers/delay_timer/delay_timer.c \
	drivers/delay_timer/generic_delay_timer.c \
	drivers/io/io_storage.c \
	drivers/io/io_block.c \
	drivers/io/io_fip.c \
	${QRK3588_STORAGE_SOURCE} \
	plat/quest/qrk3588/qrk3588_io_storage.c

BL2_SOURCES += \
	common/desc_image_load.c \
	drivers/io/io_storage.c \
	drivers/io/io_block.c \
	drivers/io/io_fip.c \
	drivers/delay_timer/delay_timer.c \
	drivers/delay_timer/generic_delay_timer.c \
	${QRK3588_STORAGE_SOURCE} \
	plat/quest/qrk3588/qrk3588_io_storage.c \
	plat/quest/qrk3588/qrk3588_ddr.c \
	plat/quest/qrk3588/ddr/src/config.c \
	plat/quest/qrk3588/ddr/src/config_write.c \
	plat/quest/qrk3588/ddr/src/context.c \
	plat/quest/qrk3588/ddr/src/diagnostics.c \
	plat/quest/qrk3588/ddr/src/eye_scan.c \
	plat/quest/qrk3588/ddr/src/fixed_data.c \
	plat/quest/qrk3588/ddr/src/frequency_switch.c \
	plat/quest/qrk3588/ddr/src/image.c \
	plat/quest/qrk3588/ddr/src/lowlevel.c \
	plat/quest/qrk3588/ddr/src/phy.c \
	plat/quest/qrk3588/ddr/src/phy_dvfs.c \
	plat/quest/qrk3588/ddr/src/phy_params.c \
	plat/quest/qrk3588/ddr/src/platform.c \
	plat/quest/qrk3588/ddr/src/runtime.c \
	plat/quest/qrk3588/ddr/src/state_mmio.c \
	plat/quest/qrk3588/ddr/src/timing.c \
	plat/quest/qrk3588/ddr/src/timing_lp5.c \
	plat/quest/qrk3588/ddr/src/training.c \
	plat/quest/qrk3588/ddr/src/training_main.c \
	plat/quest/qrk3588/ddr/src/training_phase.c \
	plat/quest/qrk3588/ddr/src/training_scan.c \
	plat/quest/qrk3588/qrk3588_bl2_setup.c \
	plat/quest/qrk3588/qrk3588_bl2_mem_params_desc.c \
	plat/quest/qrk3588/plat_helpers.S

PROGRAMMABLE_RESET_ADDRESS      := 1
COLD_BOOT_SINGLE_CPU            := 1
EARLY_CONSOLE                   := 1
SEPARATE_CODE_AND_RODATA        := 1
HW_ASSISTED_COHERENCY           := 1    # Cortex-A55 needed
CTX_INCLUDE_AARCH32_REGS        := 0    # Cortex-A76 supports only AArch64

BL31_SOURCES += \
	drivers/delay_timer/delay_timer.c \
	drivers/delay_timer/generic_delay_timer.c \
	lib/cpus/aarch64/cortex_a55.S \
	plat/quest/qrk3588/qrk3588_bl31_setup.c \
	plat/quest/qrk3588/qrk3588_psci.c \
	plat/quest/qrk3588/cpu_entry.S \
	plat/quest/qrk3588/plat_helpers.S \
	plat/quest/qrk3588/qrk3588_gic.c \
	plat/quest/qrk3588/vendor/rockchip/drivers/secure/secure.c

# BL31 may issue eMMC requests after BL2 has loaded it. Reuse the card
# selection state retained by BL1 instead of repeating card discovery.
ifeq (${QRK3588_BOOT_EMMC},1)
BL31_SOURCES += \
	plat/quest/qrk3588/drivers/qrk3588_mmc.c
endif

# Enable workarounds for selected Cortex-A55 errata.
ERRATA_A55_1530923 := 1

USE_GIC_DRIVER          := 3
# Allow detection of GIC-600
GICV3_SUPPORT_GIC600    := 1

INIT_UNUSED_NS_EL2  := 1

# Software trust anchor: BL1 must be provisioned and protected separately.
ifeq (${TRUSTED_BOARD_BOOT},1)
ifneq (${RESET_TO_BL2}${COT_DESC_IN_DTB},00)
$(error QRK3588 secure boot requires BL1 and the static BL2 CoT)
endif
ifneq (${USE_TBBR_DEFS},1)
$(error QRK3588 secure boot requires USE_TBBR_DEFS=1)
endif
ifneq ($(strip ${SCP_BL2} ${TB_FW_CONFIG} ${FW_CONFIG} ${HW_CONFIG} ${SOC_FW_CONFIG} ${TOS_FW_CONFIG} ${NT_FW_CONFIG}),)
$(error QRK3588 secure boot does not load SCP firmware or firmware configuration payloads)
endif
ifneq (${COT},tbbr)
$(error QRK3588 secure boot requires COT=tbbr)
endif
ifneq (${DYN_DISABLE_AUTH},0)
$(error QRK3588 secure boot forbids DYN_DISABLE_AUTH)
endif
ifneq (${HASH_ALG},sha256)
$(error QRK3588 secure boot requires HASH_ALG=sha256)
endif
ifneq (${KEY_ALG},rsa)
$(error QRK3588 secure boot requires KEY_ALG=rsa)
endif
ifneq (${KEY_SIZE},3072)
$(error QRK3588 secure boot requires KEY_SIZE=3072)
endif
ifeq (${ROT_KEY},)
$(error QRK3588 secure boot requires an explicit ROT_KEY)
endif
ifneq (${BL32},)
$(error QRK3588 BL32 loading and secure payload dispatch are not supported yet)
endif
TFW_NVCTR_VAL ?= 0
NTFW_NVCTR_VAL ?= 0
ifneq (${TFW_NVCTR_VAL}${NTFW_NVCTR_VAL},00)
$(error QRK3588 software counters require TFW_NVCTR_VAL=0 NTFW_NVCTR_VAL=0)
endif

include drivers/auth/auth.mk
include drivers/auth/mbedtls/mbedtls_crypto.mk
include drivers/auth/mbedtls/mbedtls_x509.mk

QRK3588_AUTH_SOURCES := ${AUTH_SOURCES} \
	${PLAT_QRK3588_PATH}/qrk3588_trusted_boot.c \
	drivers/auth/tbbr/tbbr_cot_common.c
BL1_SOURCES += ${QRK3588_AUTH_SOURCES} drivers/auth/tbbr/tbbr_cot_bl1.c
BL2_SOURCES += ${QRK3588_AUTH_SOURCES} drivers/auth/tbbr/tbbr_cot_bl2.c

PLAT_INCLUDES += -I${BUILD_PLAT}
QRK3588_ROTPK_HEADER := ${BUILD_PLAT}/qrk3588_rotpk.h
# Re-evaluate the key on every invocation, including a different key path or
# content with an older timestamp. Unchanged public anchors keep their mtime.
.PHONY: qrk3588_rotpk_check
${QRK3588_ROTPK_HEADER}: qrk3588_rotpk_check ${PLAT_QRK3588_PATH}/gen_rotpk.py
	${Q}${PYTHON} ${PLAT_QRK3588_PATH}/gen_rotpk.py --key "${ROT_KEY}" --output "$@"

${BUILD_PLAT}/bl1/qrk3588_trusted_boot.o: ${QRK3588_ROTPK_HEADER}
${BUILD_PLAT}/bl2/qrk3588_trusted_boot.o: ${QRK3588_ROTPK_HEADER}
endif
