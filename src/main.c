/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal transparent bridge:
 *   complete 12-byte SPI frame -> unchanged ESB frame -> hardware ACK.
 *
 * This component does not interpret HID state, deduplicate reports, generate
 * keepalives, rewrite sequence numbers, or create autonomous radio traffic.
 */

#include <errno.h>
#include <string.h>

#include <esb.h>
#include <nrfx.h>
#include <nrfx_spis.h>
#include <hal/nrf_gpio.h>
#include <zephyr/irq.h>
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
#define LINK_ACK_MAGIC       0x5AU
#define LINK_RF_CHANNEL      80U
#define REPORT_QUEUE_DEPTH   256U
#define ESB_EVENT_TIMEOUT_US 2000U
#define RETRY_BACKOFF_US     100U
#define SPIS_IRQ_PRIORITY    1U
#define SPIS_SCK_PIN         NRF_GPIO_PIN_MAP(0, 17)
#define SPIS_MOSI_PIN        NRF_GPIO_PIN_MAP(0, 20)
#define SPIS_MISO_PIN        NRF_GPIO_PIN_MAP(0, 8)
#define SPIS_CSN_PIN         NRF_GPIO_PIN_MAP(0, 22)
#define SPIS_BUFFER_COUNT    2U

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
static atomic_t spi_rearm_errors;
static atomic_t spi_duplicates;
static atomic_t report_queue_overruns;
static atomic_t esb_tx_successes;
static atomic_t esb_tx_failures;
static atomic_t esb_tx_timeouts;
static atomic_t radio_frame_in_flight;
static atomic_t poweroff_pending;

static nrfx_spis_t spis_instance = NRFX_SPIS_INSTANCE(NRF_SPIS1);
static struct link_frame spis_rx_buffers[SPIS_BUFFER_COUNT]
	__aligned(sizeof(uint32_t));
static struct link_frame spis_tx_buffers[SPIS_BUFFER_COUNT]
	__aligned(sizeof(uint32_t));
static struct link_frame last_spi_frame;
static bool last_spi_frame_valid;

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
			spi_ack_response = ack;
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
	*output = spi_ack_response;
	k_spin_unlock(&spi_ack_lock, key);
}

static int spis_arm_buffer(uint8_t index)
{
	memset(&spis_rx_buffers[index], 0, sizeof(spis_rx_buffers[index]));
	return nrfx_spis_buffers_set(&spis_instance,
		(uint8_t const *)&spis_tx_buffers[index],
		sizeof(spis_tx_buffers[index]),
		(uint8_t *)&spis_rx_buffers[index],
		sizeof(spis_rx_buffers[index]));
}

static void spis_event_handler(nrfx_spis_event_t const *event,
			       void *context)
{
	ARG_UNUSED(context);

	if (event->evt_type != NRFX_SPIS_XFER_DONE) {
		return;
	}

	uint8_t completed_index;
	if (event->p_rx_buf == &spis_rx_buffers[0]) {
		completed_index = 0U;
	} else if (event->p_rx_buf == &spis_rx_buffers[1]) {
		completed_index = 1U;
	} else {
		atomic_inc(&spi_errors);
		return;
	}

	/* Rearm EasyDMA first, directly from the SPIS IRQ. Validation, queueing
	 * and radio work happen only after the other buffer is already requested. */
	uint8_t const next_index = completed_index ^ 1U;
	if (spis_arm_buffer(next_index) != 0) {
		atomic_inc(&spi_rearm_errors);
	}

	/* Refresh the completed slot's reverse status while the other slot owns
	 * the next transaction. A slightly stale LED status is safe; an unarmed
	 * keyboard input transaction is not. */
	spi_ack_snapshot(&spis_tx_buffers[completed_index]);

	if (event->rx_amount != sizeof(struct link_frame)) {
		atomic_inc(&spi_errors);
		return;
	}

	struct link_frame frame;
	memcpy(&frame, event->p_rx_buf, sizeof(frame));
	if (frame.magic != LINK_MAGIC || frame.version != LINK_VERSION) {
		atomic_inc(&spi_errors);
		return;
	}
	if (last_spi_frame_valid &&
	    memcmp(&frame, &last_spi_frame, sizeof(frame)) == 0) {
		/* RP2040 sends one transport-level safety copy with the same
		 * sequence. Suppress it before ESB; HID semantics remain untouched. */
		atomic_inc(&spi_duplicates);
		return;
	}
	if (frame.type == LINK_TYPE_CONTROL &&
	    frame.data[0] == LINK_CONTROL_SYSTEM_OFF) {
		last_spi_frame = frame;
		last_spi_frame_valid = true;
		atomic_inc(&spi_frames);
		atomic_set(&poweroff_pending, 1);
		k_sem_give(&poweroff_requested);
		return;
	}

	if (k_msgq_put(&report_queue, &frame, K_NO_WAIT) != 0) {
		atomic_inc(&report_queue_overruns);
		return;
	}
	last_spi_frame = frame;
	last_spi_frame_valid = true;
	atomic_inc(&spi_frames);
}

static int spis_initialize(void)
{
	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIS1), SPIS_IRQ_PRIORITY,
		    nrfx_spis_irq_handler, &spis_instance, 0);

	nrfx_spis_config_t config = NRFX_SPIS_DEFAULT_CONFIG(
		SPIS_SCK_PIN, SPIS_MOSI_PIN, SPIS_MISO_PIN, SPIS_CSN_PIN);
	config.irq_priority = SPIS_IRQ_PRIORITY;
	config.def = 0x00U;
	config.orc = 0x00U;

	int err = nrfx_spis_init(&spis_instance, &config,
				 spis_event_handler, NULL);
	if (err != 0) {
		return err;
	}

	for (uint8_t i = 0U; i < SPIS_BUFFER_COUNT; ++i) {
		spi_ack_snapshot(&spis_tx_buffers[i]);
	}
	return spis_arm_buffer(0U);
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
	esb_config.retransmit_count = 1;
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
		 * a hardware ESB ACK. This preserves order including key releases. */
		while (esb_send_once(&frame) != 0) {
			k_busy_wait(RETRY_BACKOFF_US);
		}
		atomic_set(&radio_frame_in_flight, 0);
	}
}

K_THREAD_DEFINE(radio_thread_id, 1536, radio_thread,
		NULL, NULL, NULL, 5, 0, 0);

#if CONFIG_LOG
static void status_thread(void)
{
	for (;;) {
		k_sleep(K_SECONDS(5));
		LOG_INF("SPI=%ld err=%ld rearm_err=%ld duplicates=%ld queue_full=%ld ESB_ok=%ld fail=%ld timeout=%ld",
			(long)atomic_get(&spi_frames),
			(long)atomic_get(&spi_errors),
			(long)atomic_get(&spi_rearm_errors),
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
	err = spis_initialize();
	if (err != 0) return err;
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
		nrfx_spis_uninit(&spis_instance);
		nrf_gpio_cfg_sense_input(SPIS_CSN_PIN,
					 NRF_GPIO_PIN_PULLUP,
					 NRF_GPIO_PIN_SENSE_LOW);
		sys_poweroff();
	}
}
