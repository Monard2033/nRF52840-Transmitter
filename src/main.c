/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wireless keyboard transmitter:
 *   12-byte typed SPI input frame -> reliable 12-byte ESB PTX packet.
 */

#include <errno.h>
#include <string.h>

#include <esb.h>
#include <nrfx.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_gpio.h>

LOG_MODULE_REGISTER(transmitter, LOG_LEVEL_INF);

#define INPUT_DATA_SIZE         8U
#define LINK_MAGIC              0xA5U
#define LINK_VERSION            0x02U
#define LINK_TYPE_KEYBOARD      0x01U
#define LINK_TYPE_CONSUMER      0x02U
#define LINK_TYPE_CONTROL       0x03U
#define LINK_CONTROL_SYSTEM_OFF 0x01U
#define LINK_RF_CHANNEL         80U
#define REPORT_QUEUE_DEPTH      32U
#define REPORT_KEEPALIVE_MS     8
#define APP_TX_RETRY_COUNT      2U
#define ESB_EVENT_TIMEOUT_US    2000
#define WAKE_CSN_PIN            NRF_GPIO_PIN_MAP(0, 22)

struct link_input_packet {
	uint8_t magic;
	uint8_t version;
	uint8_t type;
	uint8_t sequence;
	uint8_t data[INPUT_DATA_SIZE];
} __packed;

BUILD_ASSERT(sizeof(struct link_input_packet) == 12U);

static const struct device *const spi_device =
	DEVICE_DT_GET(DT_NODELABEL(spi1));

static const struct spi_config spi_config = {
	.frequency = 8000000U,
	.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = 0,
};

K_MSGQ_DEFINE(report_queue, sizeof(struct link_input_packet),
	      REPORT_QUEUE_DEPTH, sizeof(uint32_t));
static K_SEM_DEFINE(esb_tx_done, 0, 1);
static K_SEM_DEFINE(esb_started, 0, 1);

static struct esb_config esb_config = ESB_DEFAULT_CONFIG;
static struct esb_payload esb_tx_payload;
static atomic_t esb_last_tx_succeeded;

static atomic_t spi_frames;
static atomic_t spi_errors;
static atomic_t report_queue_overruns;
static atomic_t esb_tx_successes;
static atomic_t esb_tx_failures;
static atomic_t esb_tx_timeouts;
static atomic_t esb_link_probes;

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
		/* This application does not use acknowledgement payloads. */
		break;
	default:
		break;
	}
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
	esb_config.payload_length = sizeof(struct link_input_packet);
	esb_config.selective_auto_ack = true;
	esb_config.use_fast_ramp_up = true;
	esb_config.event_handler = transmitter_esb_event_handler;

	err = esb_init(&esb_config);
	if (err != 0) {
		return err;
	}

	err = esb_set_base_address_0(base_address_0);
	if (err != 0) {
		return err;
	}

	err = esb_set_base_address_1(base_address_1);
	if (err != 0) {
		return err;
	}

	err = esb_set_prefixes(address_prefixes, ARRAY_SIZE(address_prefixes));
	if (err != 0) {
		return err;
	}

	return esb_set_rf_channel(LINK_RF_CHANNEL);
}

static bool queue_packet(const struct link_input_packet *packet)
{
	if (k_msgq_put(&report_queue, packet, K_NO_WAIT) == 0) {
		return true;
	}

	/*
	 * Frames are complete typed input states. When saturated, replace the
	 * oldest frame with the newest so releases are not stuck behind stale data.
	 */
	struct link_input_packet discarded;

	atomic_inc(&report_queue_overruns);
	if (k_msgq_get(&report_queue, &discarded, K_NO_WAIT) == 0) {
		return k_msgq_put(&report_queue, packet, K_NO_WAIT) == 0;
	}
	return false;
}

static int esb_send_packet(const struct link_input_packet *packet)
{
	int err = -EIO;

	esb_tx_payload.length = sizeof(*packet);
	esb_tx_payload.pipe = 0;
	esb_tx_payload.noack = false;
	memcpy(esb_tx_payload.data, packet, sizeof(*packet));

	for (uint32_t attempt = 0; attempt <= APP_TX_RETRY_COUNT; ++attempt) {
		k_sem_reset(&esb_tx_done);
		atomic_set(&esb_last_tx_succeeded, 0);
		esb_flush_tx();

		err = esb_write_payload(&esb_tx_payload);
		if (err != 0) {
			LOG_WRN("ESB queue failed: %d", err);
			continue;
		}

		err = k_sem_take(&esb_tx_done, K_USEC(ESB_EVENT_TIMEOUT_US));
		if (err != 0) {
			atomic_inc(&esb_tx_timeouts);
			LOG_ERR("ESB event timeout");
			esb_flush_tx();
			return -ETIMEDOUT;
		}

		if (atomic_get(&esb_last_tx_succeeded) != 0) {
			return 0;
		}

		err = -EIO;
	}

	return err;
}

static bool keyboard_packet_is_released(const struct link_input_packet *packet)
{
	static const uint8_t released[INPUT_DATA_SIZE] = { 0 };

	return memcmp(packet->data, released, sizeof(released)) == 0;
}

static void transmitter_system_off(void)
{
	LOG_INF("Control request: entering System OFF; wake source is CSN/P0.22 LOW");
	esb_disable();

	/* SPIS no longer needs the CSN function after the complete control frame.
	 * Reconfigure the same physical pin as an active-low System OFF source. */
	nrf_gpio_cfg_sense_input(WAKE_CSN_PIN,
				 NRF_GPIO_PIN_PULLUP,
				 NRF_GPIO_PIN_SENSE_LOW);
	sys_poweroff();
}

static void radio_thread(void)
{
	struct link_input_packet packet;
	struct link_input_packet latest_keyboard;
	bool latest_keyboard_valid = false;
	bool keyboard_delivery_pending = false;

	k_sem_take(&esb_started, K_FOREVER);
	for (;;) {
		/* Consumer frames are transition-only. Keyboard keepalive runs only
		 * while at least one normal key/modifier remains pressed. */
		k_timeout_t const timeout = latest_keyboard_valid &&
			(!keyboard_packet_is_released(&latest_keyboard) ||
			 keyboard_delivery_pending) ?
			K_MSEC(REPORT_KEEPALIVE_MS) : K_FOREVER;
		if (k_msgq_get(&report_queue, &packet, timeout) == 0) {
			if (packet.type == LINK_TYPE_KEYBOARD) {
				latest_keyboard = packet;
				latest_keyboard_valid = true;
				keyboard_delivery_pending = true;
			}
		} else {
			packet = latest_keyboard;
			atomic_inc(&esb_link_probes);
		}

		if (esb_send_packet(&packet) != 0) {
			LOG_WRN_RATELIMIT("Report was not acknowledged");
		} else if (packet.type == LINK_TYPE_KEYBOARD) {
			keyboard_delivery_pending = false;
		}
	}
}

K_THREAD_DEFINE(radio_thread_id, 1536, radio_thread,
		NULL, NULL, NULL, 5, 0, 0);

#if CONFIG_LOG
static void status_thread(void)
{
	for (;;) {
		k_sleep(K_SECONDS(5));
		LOG_INF("SPI=%ld err=%ld queue_drop=%ld keepalive=%ld ESB_ok=%ld fail=%ld "
			"timeout=%ld RF[state=%lu cfg_ch=%u reg_ch=%lu mode=%lu txpipe=%lu] "
			"HF=0x%08lx",
			(long)atomic_get(&spi_frames),
			(long)atomic_get(&spi_errors),
			(long)atomic_get(&report_queue_overruns),
			(long)atomic_get(&esb_link_probes),
			(long)atomic_get(&esb_tx_successes),
			(long)atomic_get(&esb_tx_failures),
			(long)atomic_get(&esb_tx_timeouts),
			(unsigned long)NRF_RADIO->STATE,
			LINK_RF_CHANNEL,
			(unsigned long)NRF_RADIO->FREQUENCY,
			(unsigned long)NRF_RADIO->MODE,
			(unsigned long)NRF_RADIO->TXADDRESS,
			(unsigned long)NRF_CLOCK->HFCLKSTAT);
	}
}

K_THREAD_DEFINE(status_thread_id, 1024, status_thread,
		NULL, NULL, NULL, 7, 0, 0);
#endif

int main(void)
{
	struct link_input_packet spi_rx __aligned(sizeof(uint32_t));
	struct link_input_packet last_spi_keyboard;
	bool last_spi_keyboard_valid = false;
	int err;

	LOG_INF("Starting typed SPI-to-ESB keyboard/consumer transmitter");

	if (!device_is_ready(spi_device)) {
		LOG_ERR("SPI slave device is not ready");
		return -ENODEV;
	}

	err = esb_initialize();
	if (err != 0) {
		LOG_ERR("ESB initialization failed: %d", err);
		return err;
	}
	k_sem_give(&esb_started);

	LOG_INF("Ready: SPI slave on %s at 8 MHz, ESB PTX 2 Mbps channel %u, "
		"pipe 0 address E7:E7E7E7E7",
		spi_device->name, LINK_RF_CHANNEL);

	for (;;) {
		struct spi_buf rx_buffer = {
			.buf = &spi_rx,
			.len = sizeof(spi_rx),
		};
		const struct spi_buf_set rx = {
			.buffers = &rx_buffer,
			.count = 1,
		};

		memset(&spi_rx, 0, sizeof(spi_rx));
		err = spi_transceive(spi_device, &spi_config, NULL, &rx);
		if (err < 0) {
			atomic_inc(&spi_errors);
			LOG_ERR("SPI slave transfer failed: %d", err);
			k_msleep(1);
			continue;
		}

		/*
		 * In Zephyr slave mode, the positive return value is the number
		 * of received 8-bit frames. Reject incomplete typed frames.
		 */
		if (err != sizeof(spi_rx)) {
			atomic_inc(&spi_errors);
			LOG_WRN("Ignoring short SPI frame: %d/%u bytes", err,
				(unsigned int)sizeof(spi_rx));
			continue;
		}

		atomic_inc(&spi_frames);
		if (spi_rx.magic != LINK_MAGIC || spi_rx.version != LINK_VERSION ||
		    (spi_rx.type != LINK_TYPE_KEYBOARD &&
		     spi_rx.type != LINK_TYPE_CONSUMER &&
		     spi_rx.type != LINK_TYPE_CONTROL)) {
			atomic_inc(&spi_errors);
			LOG_WRN("Ignoring invalid SPI frame: magic=%02x version=%u type=%u",
				spi_rx.magic, spi_rx.version, spi_rx.type);
			continue;
		}

		if (spi_rx.type == LINK_TYPE_CONTROL) {
			if (spi_rx.data[0] != LINK_CONTROL_SYSTEM_OFF) {
				atomic_inc(&spi_errors);
				LOG_WRN("Ignoring unknown control command: %u", spi_rx.data[0]);
				continue;
			}
			transmitter_system_off();
			continue;
		}

		if (spi_rx.type == LINK_TYPE_KEYBOARD) {
			if (last_spi_keyboard_valid &&
			    memcmp(spi_rx.data, last_spi_keyboard.data,
				   sizeof(spi_rx.data)) == 0) {
				continue;
			}
			if (queue_packet(&spi_rx)) {
				last_spi_keyboard = spi_rx;
				last_spi_keyboard_valid = true;
			}
			continue;
		}

		(void)queue_packet(&spi_rx);
	}
}
