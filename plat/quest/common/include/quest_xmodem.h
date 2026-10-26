/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef QUEST_XMODEM_H
#define QUEST_XMODEM_H

#include <stddef.h>
#include <stdint.h>

#include <lib/utils_def.h>

#define QUEST_XMODEM_MAX_PACKET_SIZE	U(1024)

struct quest_xmodem_ops {
	int (*getc)(void *cookie, unsigned int timeout_us);
	void (*putc)(void *cookie, uint8_t value);
	int (*consume)(void *cookie, const uint8_t *data, size_t size);
};

struct quest_xmodem_config {
	unsigned int start_timeout_us;
	unsigned int byte_timeout_us;

	/*
	 * Number of startup timeout periods before giving up.
	 * 0 means wait indefinitely.
	 */
	unsigned int start_max_timeouts;

	/*
	 * Maximum consecutive transfer errors.
	 * Reset after a valid packet is received.
	 */
	unsigned int max_retries;
};

int quest_xmodem_receive_crc(const struct quest_xmodem_ops *ops,
			     void *cookie, uint8_t *packet_buffer,
			     size_t packet_buffer_size,
			     const struct quest_xmodem_config *config);

#endif /* QUEST_XMODEM_H */
