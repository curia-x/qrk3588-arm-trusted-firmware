/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_FSPI_H
#define QRK3588_FSPI_H

#include <stddef.h>
#include <stdint.h>

/* Board wiring: FSPI M0, M1 or M2, on chip select 0. */
#ifndef QRK3588_FSPI_IOMUX
#define QRK3588_FSPI_IOMUX	0
#endif

#if QRK3588_FSPI_IOMUX < 0 || QRK3588_FSPI_IOMUX > 2
#error "QRK3588_FSPI_IOMUX must be 0, 1 or 2"
#endif

/*
 * Requires an initialized delay timer and mapped SFC/CRU/IOC registers.
 * Supports single-line, 24-bit-addressed JEDEC NOR, up to 16 MiB. NAND needs
 * device-specific geometry, ECC and bad-block handling and is not supported.
 */
int qrk3588_fspi_nor_init(void);

/* io_block callback: LBA is in 512-byte sectors; errors return zero bytes. */
size_t qrk3588_fspi_nor_read(int lba, uintptr_t buffer, size_t size);

#endif /* QRK3588_FSPI_H */
