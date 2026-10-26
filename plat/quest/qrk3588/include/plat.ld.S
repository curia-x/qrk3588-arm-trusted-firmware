/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <qrk3588_memory.h>

/* The copied code and per-CPU slots must fit in the first SRAM page. */
ASSERT(QRK3588_TRAMPOLINE_OFFSET +
       (qrk3588_cpu_end - qrk3588_cpu_start) <= PAGE_SIZE,
       "QRK3588 CPU trampoline exceeds reserved SRAM page")
