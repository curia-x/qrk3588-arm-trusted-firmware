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
uint64_t qrk3588_emmc_handoff_token(void);
int qrk3588_emmc_handoff(uint64_t token);
int qrk3588_sdmmc_init(void);

/* io_block callbacks: 512-byte geometry, return size on success or zero. */
size_t qrk3588_emmc_read(int lba, uintptr_t buffer, size_t size);
size_t qrk3588_emmc_write(int lba, uintptr_t buffer, size_t size);
size_t qrk3588_sdmmc_read(int lba, uintptr_t buffer, size_t size);

#endif /* QRK3588_MMC_H */
