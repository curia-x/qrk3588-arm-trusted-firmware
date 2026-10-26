/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <lib/utils_def.h>

#include <quest_xmodem.h>

#define XMODEM_SOH			U(0x01)
#define XMODEM_STX			U(0x02)
#define XMODEM_EOT			U(0x04)
#define XMODEM_ACK			U(0x06)
#define XMODEM_NAK			U(0x15)
#define XMODEM_CAN			U(0x18)

#define XMODEM_CRC_REQUEST		'C'

#define XMODEM_PACKET_128		U(128)

static uint16_t xmodem_crc16(const uint8_t *data, size_t length)
{
	uint16_t crc = 0U;

	while (length-- != 0U) {
		crc ^= (uint16_t)*data++ << 8;

		for (unsigned int bit = 0U; bit < 8U; bit++) {
			if ((crc & U(0x8000)) != 0U)
				crc = (uint16_t)((crc << 1) ^ U(0x1021));
			else
				crc = (uint16_t)(crc << 1);
		}
	}

	return crc;
}

/*
 * Discard bytes belonging to an incomplete packet.
 *
 * XMODEM has no explicit framing delimiter between packets. If reception
 * fails halfway through a packet, blindly treating the next received byte
 * as SOH/STX could make remaining payload bytes look like a new packet.
 *
 * Wait until the UART becomes idle before asking the sender to retransmit.
 */
static void xmodem_drain_until_idle(const struct quest_xmodem_ops *ops,
				    void *cookie,
				    unsigned int timeout_us)
{
	while (ops->getc(cookie, timeout_us) >= 0)
		;
}

/*
 * Cancel the transfer.
 *
 * Sending CAN twice makes cancellation less likely to be confused with
 * an isolated corrupted byte on the serial line.
 */
static void xmodem_cancel(const struct quest_xmodem_ops *ops, void *cookie)
{
	ops->putc(cookie, XMODEM_CAN);
	ops->putc(cookie, XMODEM_CAN);
}

int quest_xmodem_receive_crc(const struct quest_xmodem_ops *ops,
			     void *cookie,
			     uint8_t *packet_buffer,
			     size_t packet_buffer_size,
			     const struct quest_xmodem_config *config)
{
	uint8_t expected = 1U;
	unsigned int start_timeouts = 0U;
	unsigned int retries = 0U;
	bool started = false;
	bool have_packet = false;

	if ((ops == NULL) ||
	    (ops->getc == NULL) ||
	    (ops->putc == NULL) ||
	    (ops->consume == NULL) ||
	    (packet_buffer == NULL) ||
	    (packet_buffer_size < QUEST_XMODEM_MAX_PACKET_SIZE) ||
	    (config == NULL) ||
	    (config->start_timeout_us == 0U) ||
	    (config->byte_timeout_us == 0U) ||
	    (config->max_retries == 0U))
		return -EINVAL;

	/*
	 * Ask the sender to use CRC16 mode.
	 *
	 * The request is repeated periodically while waiting for the user to
	 * select a file and start the sender.
	 */
	ops->putc(cookie, XMODEM_CRC_REQUEST);

	for (;;) {
		unsigned int packet_size;
		uint8_t number;
		uint8_t number_inv;
		uint16_t received_crc;
		uint16_t calculated_crc;
		int marker;
		int ret;

		/*
		 * Startup and transfer use deliberately different timeouts:
		 *
		 *   startup:
		 *       Human interaction may take minutes.
		 *
		 *   transfer:
		 *       Bytes should arrive continuously, so a much shorter
		 *       timeout is appropriate.
		 */
		marker = ops->getc(cookie,
				  started ?
				  config->byte_timeout_us :
				  config->start_timeout_us);

		if (marker < 0) {
			if (!started) {
				/*
				 * A startup timeout is not a protocol failure.
				 * It merely means the sender has not started yet.
				 */
				if ((config->start_max_timeouts != 0U) &&
				    (++start_timeouts >=
				     config->start_max_timeouts)) {
					xmodem_cancel(ops, cookie);
					return -ETIMEDOUT;
				}

				ops->putc(cookie, XMODEM_CRC_REQUEST);
				continue;
			}

			goto transfer_retry;
		}

		if (marker == XMODEM_CAN)
			return -ECANCELED;

		if (marker == XMODEM_EOT) {
			/*
			 * Do not accept an EOT before the transfer actually
			 * started. This avoids treating random startup input as
			 * a successful zero-byte transfer.
			 */
			if (!started) {
				ops->putc(cookie, XMODEM_CRC_REQUEST);
				continue;
			}

			ops->putc(cookie, XMODEM_ACK);
			return 0;
		}

		if ((marker != XMODEM_SOH) &&
		    (marker != XMODEM_STX)) {
			/*
			 * Before transfer begins, unrelated serial input is
			 * simply ignored. During transfer it means framing has
			 * been lost and a retransmission is required.
			 */
			if (!started)
				continue;

			goto transfer_resync;
		}

		/*
		 * SOH/STX means the sender has entered packet transfer mode.
		 * From this point onward packet errors are answered with NAK,
		 * not another CRC negotiation character.
		 */
		started = true;

		packet_size = (marker == XMODEM_SOH) ?
			      XMODEM_PACKET_128 :
			      QUEST_XMODEM_MAX_PACKET_SIZE;

		/*
		 * Always consume the complete packet before validating its
		 * header.
		 *
		 * In particular, do not immediately reject an invalid
		 * block-number complement: the sender is still transmitting
		 * the payload and CRC, and leaving those bytes in the UART
		 * would destroy framing for the next receive attempt.
		 */
		ret = ops->getc(cookie, config->byte_timeout_us);
		if (ret < 0)
			goto transfer_resync;
		number = (uint8_t)ret;

		ret = ops->getc(cookie, config->byte_timeout_us);
		if (ret < 0)
			goto transfer_resync;
		number_inv = (uint8_t)ret;

		for (unsigned int i = 0U; i < packet_size; i++) {
			ret = ops->getc(cookie, config->byte_timeout_us);
			if (ret < 0)
				goto transfer_resync;

			packet_buffer[i] = (uint8_t)ret;
		}

		ret = ops->getc(cookie, config->byte_timeout_us);
		if (ret < 0)
			goto transfer_resync;

		received_crc = (uint16_t)ret << 8;

		ret = ops->getc(cookie, config->byte_timeout_us);
		if (ret < 0)
			goto transfer_resync;

		received_crc |= (uint8_t)ret;

		/*
		 * At this point the entire packet has been consumed, so any
		 * validation failure can safely request a retransmission
		 * without draining the UART.
		 */
		if ((uint8_t)~number != number_inv)
			goto transfer_retry;

		calculated_crc = xmodem_crc16(packet_buffer, packet_size);

		if (received_crc != calculated_crc)
			goto transfer_retry;

		/*
		 * The sender may retransmit the previous packet if our ACK
		 * was lost. ACK it again, but never pass duplicate data to
		 * the consumer.
		 *
		 * have_packet prevents block 0 from being interpreted as a
		 * duplicate before block 1 has ever been accepted.
		 */
		if (have_packet &&
		    (number == (uint8_t)(expected - 1U))) {
			retries = 0U;
			ops->putc(cookie, XMODEM_ACK);
			continue;
		}

		if (number != expected)
			goto transfer_retry;

		ret = ops->consume(cookie, packet_buffer, packet_size);
		if (ret != 0) {
			xmodem_cancel(ops, cookie);
			return ret;
		}

		expected++;
		have_packet = true;
		retries = 0U;

		ops->putc(cookie, XMODEM_ACK);
		continue;


transfer_resync:
		/*
		 * Reception stopped in the middle of a packet, or framing was
		 * otherwise lost. Drain the unfinished transmission before
		 * sending NAK so stale payload bytes cannot masquerade as the
		 * next packet marker.
		 */
		xmodem_drain_until_idle(ops, cookie,
				       config->byte_timeout_us);

transfer_retry:
		if (++retries >= config->max_retries) {
			xmodem_cancel(ops, cookie);
			return -ETIMEDOUT;
		}

		ops->putc(cookie, XMODEM_NAK);
	}
}
