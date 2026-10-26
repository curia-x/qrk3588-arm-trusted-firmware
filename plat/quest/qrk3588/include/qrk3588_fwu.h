/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QRK3588_FWU_H
#define QRK3588_FWU_H

#include <stdbool.h>

void qrk3588_fwu_sample_request(void);
bool qrk3588_fwu_requested(void);
int qrk3588_fwu_receive_fip(void);

#endif /* QRK3588_FWU_H */
