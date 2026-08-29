/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal transparent bridge:
 *   complete 12-byte SPI frame -> unchanged ESB frame -> hardware ACK.
 *
 * The SPI slave layer uses the Zephyr spi_transceive driver with a fresh
 * reverse-ACK snapshot armed before every transaction. This is the reverse
 * (MISO) mechanism hardware-proven during the August 19 OTA sessions; the
 * direct-nrfx double-buffered rewrite never clocked a single byte onto MISO
 * (see the RP2040 [MISO] witness capture, 2026-08-22).
 */

#include <errno.h>
#include <string.h>

#include <esb.h>
#include <hal/nrf_gpio.h>
#include <nrfx.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(transmitter, LOG_LEVEL_INF);

#define LINK_MAGIC           0xA5U
#define LINK_VERSION         0x03U
#define LINK_FRAME_SIZE      12U
#define LINK_TYPE_CONTROL    0x03U
#define LINK_CONTROL_SYSTEM_OFF 0x01U
#define LINK_CONTROL_POLL_ACK   0x02U
#define LINK_ACK_MAGIC       0x5AU
#define LINK_RF_CHANNEL      90U
#define REPORT_QUEUE_DEPTH   256U
#define ESB_EVENT_TIMEOUT_US 15000U
#define RETRY_BACKOFF_US     50U

struct link_frame {
	uint8_t magic;
	uint8_t version;
	uint8_t type;
	uint8_t sequence;
	uint8_t data[8];
} __packed;

BUILD_ASSERT(sizeof(struct link_frame) == LINK_FRAME_SIZE);

K_MSGQ_DEFINE(report_queue, sizeof(struct link_frame), REPORT_QUEUE_DEPTH,
	      sizeof(uint32_t));
static K_SEM_DEFINE(esb_tx_done, 0, 1);
static K_SEM_DEFINE(esb_started, 0, 1);
static K_SEM_DEFINE(poweroff_requested, 0, 1);

static struct esb_config esb_config = ESB_DEFAULT_CONFIG;
static struct esb_payload esb_tx_payload;
static struct esb_payload esb_rx_payload;
static atomic_t esb_last_tx_succeeded;

static struct k_spinlock spi_ack_lock;
static struct link_frame spi_ack_response = {
	.magic = LINK_ACK_MAGIC,
	.version = LINK_VERSION,
};

static atomic_t spi_frames;
static atomic_t spi_errors;
static atomic_t spi_duplicates;
static atomic_t report_queue_overruns;
static atomic_t esb_tx_successes;
static atomic_t esb_tx_failures;
static atomic_t esb_tx_timeouts;
static atomic_t radio_frame_in_flight;
static atomic_t poweroff_pending;

static struct link_frame last_spi_frame;
static bool last_spi_frame_valid;

static const struct device *const spi_device = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct spi_config spi_slave_config = {
	.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = 0,
};

static void transmitter_esb_event_handler(const struct esb_evt *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		atomic_set(&esb_last_tx_succeeded, 1);
		atomic_inc(&esb_tx_successes);
		k_sem_give(&esb_tx_done);
		break;
	case ESB_EVENT_TX_FAILED:
		atomic_set(&esb_last_tx_succeeded, 0);
		atomic_inc(&esb_tx_failures);
		k_sem_give(&esb_tx_done);
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (esb_read_rx_payload(&esb_rx_payload) == 0) {
			struct link_frame ack;

			if (esb_rx_payload.length != sizeof(ack)) {
				continue;
			}

			memcpy(&ack, esb_rx_payload.data, sizeof(ack));
			if (ack.magic != LINK_ACK_MAGIC ||
			    ack.version != LINK_VERSION) {
				continue;
			}

			k_spinlock_key_t key = k_spin_lock(&spi_ack_lock);
			memcpy(&spi_ack_response, &ack, sizeof(ack));
			k_spin_unlock(&spi_ack_lock, key);
		}
		break;
	default:
		break;
	}
}

static void spi_ack_snapshot(struct link_frame *output)
{
	k_spinlock_key_t key = k_spin_lock(&spi_ack_lock);
	memcpy(output, &spi_ack_response, sizeof(*output));
	k_spin_unlock(&spi_ack_lock, key);
}

static int esb_initialize(void)
{
	static const uint8_t base_address_0[4] = {
		0xE7, 0xE7, 0xE7, 0xE7
	};
	static const uint8_t base_address_1[4] = {
		0xC2, 0xC2, 0xC2, 0xC2
	};
	static const uint8_t address_prefixes[8] = {
		0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8
	};
	int err;

	esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
	esb_config.mode = ESB_MODE_PTX;
	esb_config.bitrate = ESB_BITRATE_2MBPS;
	esb_config.tx_output_power = ESB_TX_POWER_8DBM;
	esb_config.retransmit_delay = 450;
	esb_config.retransmit_count = 6;
	esb_config.payload_length = sizeof(struct link_frame);
	esb_config.selective_auto_ack = true;
	esb_config.use_fast_ramp_up = true;
	esb_config.event_handler = transmitter_esb_event_handler;

	err = esb_init(&esb_config);
	if (err != 0) return err;
	err = esb_set_base_address_0(base_address_0);
	if (err != 0) return err;
	err = esb_set_base_address_1(base_address_1);
	if (err != 0) return err;
	err = esb_set_prefixes(address_prefixes, ARRAY_SIZE(address_prefixes));
	if (err != 0) return err;
	return esb_set_rf_channel(LINK_RF_CHANNEL);
}

static int esb_send_once(const struct link_frame *frame)
{
	k_sem_reset(&esb_tx_done);
	atomic_set(&esb_last_tx_succeeded, 0);
	esb_flush_tx();

	esb_tx_payload.length = sizeof(*frame);
	esb_tx_payload.pipe = 0;
	esb_tx_payload.noack = false;
	memcpy(esb_tx_payload.data, frame, sizeof(*frame));

	int err = esb_write_payload(&esb_tx_payload);
	if (err != 0) return err;

	err = k_sem_take(&esb_tx_done, K_USEC(ESB_EVENT_TIMEOUT_US));
	if (err != 0) {
		atomic_inc(&esb_tx_timeouts);
		esb_flush_tx();
		return -ETIMEDOUT;
	}
	return atomic_get(&esb_last_tx_succeeded) != 0 ? 0 : -EIO;
}

static void radio_thread(void)
{
	struct link_frame frame;

	k_sem_take(&esb_started, K_FOREVER);
	for (;;) {
		k_msgq_get(&report_queue, &frame, K_FOREVER);
		atomic_set(&radio_frame_in_flight, 1);

		/* Do not dequeue the next SPI report until this exact frame received
		 * a hardware ESB ACK. This guarantees zero lost keys and zero stuck keys. */
		while (esb_send_once(&frame) != 0) {
			k_busy_wait(RETRY_BACKOFF_US);
		}
		atomic_set(&radio_frame_in_flight, 0);
	}
}

K_THREAD_DEFINE(radio_thread_id, 1536, radio_thread,
		NULL, NULL, NULL, 5, 0, 0);

/* SPI slave thread: one blocking transaction at a time, arming a FRESH
 * reverse-ACK snapshot before each one so the RP2040 reads the newest LED
 * or DFU state on the very next transfer. This mirrors the hardware-proven
 * August 19 mechanism. */
static void spi_slave_thread(void)
{
	struct link_frame spi_tx;
	struct link_frame spi_rx;

	for (;;) {
		spi_ack_snapshot(&spi_tx);

		struct spi_buf tx_buffer = {
			.buf = &spi_tx,
			.len = sizeof(spi_tx),
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buffer,
			.count = 1,
		};
		struct spi_buf rx_buffer = {
			.buf = &spi_rx,
			.len = sizeof(spi_rx),
		};
		const struct spi_buf_set rx = {
			.buffers = &rx_buffer,
			.count = 1,
		};

		memset(&spi_rx, 0, sizeof(spi_rx));
		int err = spi_transceive(spi_device, &spi_slave_config, &tx, &rx);
		if (err < 0) {
			atomic_inc(&spi_errors);
			k_yield();
			continue;
		}

		/* Zephyr slave mode returns the number of received 8-bit frames. */
		if (err != sizeof(spi_rx)) {
			atomic_inc(&spi_errors);
			continue;
		}

		if (spi_rx.magic != LINK_MAGIC ||
		    spi_rx.version != LINK_VERSION) {
			atomic_inc(&spi_errors);
			continue;
		}

		if (last_spi_frame_valid &&
		    memcmp(&spi_rx, &last_spi_frame, sizeof(spi_rx)) == 0) {
			/* RP2040 safety copy with the same sequence: suppress
			 * before ESB; HID semantics remain untouched. */
			atomic_inc(&spi_duplicates);
			continue;
		}

		if (spi_rx.type == LINK_TYPE_CONTROL &&
		    spi_rx.data[0] == LINK_CONTROL_SYSTEM_OFF) {
			memcpy(&last_spi_frame, &spi_rx, sizeof(spi_rx));
			last_spi_frame_valid = true;
			atomic_inc(&spi_frames);
			atomic_set(&poweroff_pending, 1);
			k_sem_give(&poweroff_requested);
			continue;
		}

		if (k_msgq_put(&report_queue, &spi_rx, K_NO_WAIT) != 0) {
			atomic_inc(&report_queue_overruns);
			continue;
		}
		memcpy(&last_spi_frame, &spi_rx, sizeof(spi_rx));
		last_spi_frame_valid = true;
		atomic_inc(&spi_frames);
	}
}

K_THREAD_DEFINE(spi_slave_thread_id, 2048, spi_slave_thread,
		NULL, NULL, NULL, 4, 0, 0);

#if CONFIG_LOG
static void status_thread(void)
{
	for (;;) {
		k_sleep(K_SECONDS(5));
		LOG_INF("SPI=%ld err=%ld duplicates=%ld queue_full=%ld ESB_ok=%ld fail=%ld timeout=%ld",
			(long)atomic_get(&spi_frames),
			(long)atomic_get(&spi_errors),
			(long)atomic_get(&spi_duplicates),
			(long)atomic_get(&report_queue_overruns),
			(long)atomic_get(&esb_tx_successes),
			(long)atomic_get(&esb_tx_failures),
			(long)atomic_get(&esb_tx_timeouts));
	}
}

K_THREAD_DEFINE(status_thread_id, 1024, status_thread,
		NULL, NULL, NULL, 7, 0, 0);
#endif

int main(void)
{
	int err = esb_initialize();
	if (err != 0) return err;

	if (!device_is_ready(spi_device)) {
		LOG_ERR("SPI slave device is not ready");
		return -ENODEV;
	}
	k_sem_give(&esb_started);

	for (;;) {
		k_sem_take(&poweroff_requested, K_FOREVER);
		if (atomic_get(&poweroff_pending) == 0) {
			continue;
		}

		/* Give RP2040's same-sequence SPI safety copy time to complete, then
		 * wait only for already-accepted urgent ESB traffic. */
		k_sleep(K_MSEC(2));
		while (k_msgq_num_used_get(&report_queue) != 0U ||
		       atomic_get(&radio_frame_in_flight) != 0) {
			k_sleep(K_MSEC(1));
		}

		esb_disable();
		nrf_gpio_cfg_sense_input(NRF_GPIO_PIN_MAP(0, 22),
					 NRF_GPIO_PIN_PULLUP,
					 NRF_GPIO_PIN_SENSE_LOW);
		sys_poweroff();
	}
}
