/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_MMC_H
#define QRK3588_MMC_H

#include <stddef.h>
#include <stdint.h>

int qrk3588_emmc_init(void);
/*
 * QEM1 token ABI: bits 63:32 = 0x51454d31, bit 16 = sector addressing,
 * bits 15:0 = nonzero RCA, bits 31:17 = zero. Zero means unavailable.
 * Producer guarantees user partition, legacy 1-bit mode, idle host/card.
 * BL1, BL2 and BL31 must use this same ABI.
 */
uint64_t qrk3588_emmc_handoff_token(void);
int qrk3588_emmc_handoff(uint64_t token);

/* io_block callbacks: 512-byte geometry, return size on success or zero. */
size_t qrk3588_emmc_read(int lba, uintptr_t buffer, size_t size);
size_t qrk3588_emmc_write(int lba, uintptr_t buffer, size_t size);

#endif /* QRK3588_MMC_H */
