/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_IO_STORAGE_H
#define QRK3588_IO_STORAGE_H

#include <lib/utils_def.h>

/* eMMC user-area layout, with 512-byte blocks:
 *   0..63       fixed hole
 *   64          IDBlock header
 *   65..2104    BL1
 *   2105..43064 FIP (20 MiB)
 */
#define QRK3588_FIP_IMAGE_OFFSET	UL(0x00107200)
#define QRK3588_FIP_IMAGE_MAX_SIZE	UL(0x01400000)

int plat_io_setup(void);
int qrk3588_bl2_io_setup(uint64_t emmc_handoff_token);

#endif /* QRK3588_IO_STORAGE_H */
