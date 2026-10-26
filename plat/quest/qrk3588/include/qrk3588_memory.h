/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_MEMORY_H
#define QRK3588_MEMORY_H

#include <lib/utils_def.h>

/*
 * BL1 takes ownership of System SRAM as follows:
 *
 * 0xff000000 +--------------------------+
 *            | Reserved SRAM (4 KiB)    |
 * 0xff001000 +--------------------------+
 *            | BL1 RO                   |
 * 0xff040000 +--------------------------+
 *            | BL1 RW                   |
 * 0xff080000 +--------------------------+
 *            | BL2                      |
 * 0xff100000 +--------------------------+
 */
#define QRK3588_SYSTEM_SRAM_BASE		UL(0xff000000)
#define QRK3588_SYSTEM_SRAM_SIZE		UL(0x00100000)
#define QRK3588_SYSTEM_SRAM_LIMIT		(QRK3588_SYSTEM_SRAM_BASE + \
						 QRK3588_SYSTEM_SRAM_SIZE)

#define QRK3588_BROM_STAGE0_LOAD_BASE	(QRK3588_SYSTEM_SRAM_BASE + UL(0x1000))
#define QRK3588_BROM_STAGE0_LOAD_LIMIT	(QRK3588_SYSTEM_SRAM_BASE + UL(0x40000))

/*
 * 4-GiB DDR layout:
 *
 * 0x000000000 +---------------------------+
 *             | Boot-time reserved (2 MiB)|
 * 0x00200000  +---------------------------+
 *             | BL33 initial image        |
 *             |                           |
 * 0x01000000  +---------------------------+
 *             | U-Boot boot workspace     |
 * 0x01200000  +---------------------------+
 *             | Linux DTB                 |
 * 0x01400000  +---------------------------+
 *             | Linux Image               |
 *             |                           |
 * 0xf0000000  +---------------------------+
 *
 *             ... address hole ...
 *
 * 0x1f0000000 +---------------------------+
 *             | Remapped high DDR         |
 *             |                           |
 * 0x1ffe00000 +---------------------------+
 *             | BL31 / EL3 runtime        |
 * 0x200000000 +---------------------------+
 */
#define QRK3588_DDR_BASE		UL(0x00000000)
#define QRK3588_DDR_LIMIT		UL(0xf0000000)
#define QRK3588_BL33_BASE		UL(0x00200000)

/* The 4-GiB configuration remaps its upper 256 MiB above the 4-GiB hole. */
#define QRK3588_DDR_HIGH_BASE		ULL(0x1f0000000)
#define QRK3588_DDR_HIGH_LIMIT		ULL(0x200000000)

#endif /* QRK3588_MEMORY_H */
