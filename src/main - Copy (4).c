/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <esb.h>
#include <zephyr/sys_clock.h>
#include <nrfx_clock.h>
#include <string.h>
#include <nrfx.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/drivers/spi.h>

#define ERROR_PAYLOAD_FLAG 0x80
#define BINDING_ID 0x0BADF00D
#define VECTOR_SIZE 10 // Vector size for GPIO data

LOG_MODULE_REGISTER(transmitter, LOG_LEVEL_INF);

// ESB configuration
struct esb_config esb_config = ESB_DEFAULT_CONFIG;
static struct esb_payload tx_payload;
static volatile bool tx_failed = false;

// USB HID configuration
static volatile bool usb_configured = false;
static const struct device *hid_dev;
// SPI configuration
//static const struct device *spi3_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));
#define SPI_BUFFER_SIZE 8

// ADC configuration
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(adc_input));

// LED definitions
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(DT_N_NODELABEL_red_led, gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(DT_N_NODELABEL_green_led, gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(DT_N_NODELABEL_blue_led, gpios);

enum led_color {
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_OFF,
    LED_YELLOW,
    LED_ORANGE
};

// Packet types
enum packet_type {
    PACKET_TYPE_BIND_REQUEST = 0x01,
    PACKET_TYPE_BIND_ACK = 0x04,
    PACKET_TYPE_BATTERY_VOLTAGE = 0x06
};

// Vectors for GPIO data (retained for potential future use)
static uint8_t data_minus_vector[VECTOR_SIZE];
static uint8_t data_plus_vector[VECTOR_SIZE];
static uint8_t data_minus_count = 0;
static uint8_t data_plus_count = 0;

// Hysteresis sleep configuration
#define BASE_TIMEOUT K_SECONDS(60)
#define EXTENDED_TIMEOUT K_SECONDS(120)
#define GRACE_PERIOD_MS 2000
static k_timeout_t current_timeout = BASE_TIMEOUT;
static int wakeup_count = 0;
static int64_t last_sleep_time = 0;

#define BATTERY_CRITICAL_THRESHOLD 15
#define BATTERY_MAX_VOLTAGE 4200
#define BATTERY_MIN_VOLTAGE 3300
#define CHARGING_THRESHOLD_DELTA 50
#define FULL_CHARGE_THRESHOLD 4190
static uint32_t battery_level = 0;
static bool battery_warning_active = false;
static int pulse_count = 0;
static int64_t last_voltage_time = 0;
static uint32_t last_voltage_mv = 0;
static uint32_t max_voltage_mv = 0;
static bool is_charging = false;
static enum led_color battery_led_color = LED_OFF;

// Function declarations
void event_handler(struct esb_evt const *event);
void sample_and_transmit(struct esb_evt const *event);
void send_battery_level(uint32_t voltage);
void enter_low_power(struct k_timer *dummy);
void update_sleep_timeout(void);
void led_timer_handler(struct k_timer *timer);
void check_battery_status(void);
void battery_warning_timer_handler(struct k_timer *timer);
void battery_pulse_timer_handler(struct k_timer *timer);
void charging_pulse_timer_handler(struct k_timer *timer);
void battery_check_timer_handler(struct k_timer *timer);
void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param);
void set_led_status(enum led_color color);
void blink_led(enum led_color color, int duration_ms, int blink_count);
void read_spi_payload(void);

// Timer definitions
static K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
static K_TIMER_DEFINE(inactivity_timer, NULL, enter_low_power);
static K_TIMER_DEFINE(battery_warning_timer, battery_warning_timer_handler, NULL);
static K_TIMER_DEFINE(battery_pulse_timer, battery_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(charging_pulse_timer, charging_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(battery_check_timer, battery_check_timer_handler, NULL);

// HID report descriptor (retained for potential future use)
static const uint8_t hid_report_desc[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x75, 0x01,
    0x95, 0x08, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
    0x15, 0x00, 0x25, 0x01, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 0x75, 0x08,
    0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00,
    0x29, 0x65, 0x81, 0x00, 0xC0
};

// LED control
void set_led_status(enum led_color color) {
    gpio_pin_set_dt(&red_led, (color == LED_RED) ? 1 : 0);
    gpio_pin_set_dt(&green_led, (color == LED_GREEN) ? 1 : 0);
    gpio_pin_set_dt(&blue_led, (color == LED_BLUE) ? 1 : 0);
}

void blink_led(enum led_color color, int duration_ms, int blink_count) {
    for (int i = 0; i < blink_count; i++) {
        set_led_status(color);
        k_sleep(K_MSEC(duration_ms));
        set_led_status(LED_OFF);
        k_sleep(K_MSEC(duration_ms));
    }
}

// Timer handlers
void led_timer_handler(struct k_timer *timer) {
    if (!is_charging) {
        set_led_status(LED_OFF);
        LOG_INF("LED timer: All LEDs off");
    }
}

void battery_warning_timer_handler(struct k_timer *timer) {
    if (!is_charging && battery_warning_active) {
        k_timer_start(&battery_pulse_timer, K_NO_WAIT, K_NO_WAIT);
    }
}

void battery_pulse_timer_handler(struct k_timer *timer) {
    if (pulse_count < 8 && !is_charging) {
        set_led_status(LED_RED);
        k_sleep(K_MSEC(500));
        set_led_status(LED_OFF);
        k_sleep(K_MSEC(500));
        pulse_count++;
    } else {
        pulse_count = 0;
        k_timer_stop(&battery_pulse_timer);
        if (battery_warning_active && !is_charging) {
            k_timer_start(&battery_warning_timer, K_MINUTES(5), K_NO_WAIT);
        }
        set_led_status(LED_OFF);
    }
}

void charging_pulse_timer_handler(struct k_timer *timer) {
    static bool led_on = true;
    if (is_charging) {
        set_led_status(led_on ? battery_led_color : LED_OFF);
        led_on = !led_on;
        k_timer_start(&charging_pulse_timer, K_MSEC(500), K_NO_WAIT);
    } else {
        k_timer_stop(&charging_pulse_timer);
        set_led_status(battery_led_color);
        LOG_INF("Charging pulse stopped, restoring battery level color %d", battery_led_color);
    }
}

void battery_check_timer_handler(struct k_timer *timer) {
    LOG_INF("Battery check timer: Triggering battery status check");
    check_battery_status();
}

// Power management
void enter_low_power(struct k_timer *dummy) {
    LOG_INF("Entering System OFF mode");
    k_timer_stop(&led_timer);
    k_timer_stop(&battery_warning_timer);
    k_timer_stop(&battery_pulse_timer);
    k_timer_stop(&charging_pulse_timer);
    k_timer_stop(&battery_check_timer);
    esb_disable();
    set_led_status(LED_OFF);
    //pm_state_set(PM_STATE_SOFT_OFF, 0);
}

// Sleep timeout management
void update_sleep_timeout(void) {
    int64_t now_ms = k_uptime_get();
    k_ticks_t now_ticks = k_ms_to_ticks_ceil32(now_ms);
    k_ticks_t last_ticks = k_ms_to_ticks_ceil32(last_sleep_time);
    k_ticks_t grace_ticks = k_ms_to_ticks_ceil32(GRACE_PERIOD_MS);
    if (last_sleep_time != 0 && (now_ticks - last_ticks) < grace_ticks) {
        wakeup_count++;
        if (wakeup_count == 1) {
            current_timeout = EXTENDED_TIMEOUT;
            LOG_INF("Quick wakeup, sleep timeout 120s");
        }
    } else {
        wakeup_count = 0;
        current_timeout = BASE_TIMEOUT;
        LOG_INF("Resetting sleep timeout to 60s");
    }
    last_sleep_time = now_ms;
    k_timer_start(&inactivity_timer, current_timeout, K_NO_WAIT);
    LOG_INF("Inactivity timer started, timeout=%lld ms", Z_TIMEOUT_MS_TICKS(current_timeout.ticks));
}

// Battery monitoring
void check_battery_status(void) {
    int16_t buf;
    struct adc_sequence sequence = {
        .channels = BIT(adc_channel.channel_id),
        .buffer = &buf,
        .buffer_size = sizeof(buf),
        .resolution = BIT(adc_channel.resolution),
    };

    if (!device_is_ready(adc_channel.dev)) {
        LOG_ERR("ADC device not ready: %s", adc_channel.dev->name);
        return;
    }

    int err = adc_channel_setup_dt(&adc_channel);
    if (err < 0) {
        LOG_ERR("ADC channel setup failed (ID: %u, err: %d)", adc_channel.channel_id, err);
        return;
    }

    err = adc_sequence_init_dt(&adc_channel, &sequence);
    if (err < 0) {
        LOG_ERR("ADC sequence init failed (channel ID: %u, resolution: %u, err: %d)",
                adc_channel.channel_id, sequence.resolution, err);
        return;
    }

    err = adc_read(adc_channel.dev, &sequence);
    if (err < 0) {
        LOG_ERR("ADC read failed (channel ID: %u, buffer size: %u, err: %d)",
                adc_channel.channel_id, sequence.buffer_size, err);
        return;
    }

    uint32_t adc_value = buf;
    uint32_t voltage_mv = (adc_value * BATTERY_MAX_VOLTAGE) * 10 / (1 << 12); // 12-bit ADC
    battery_level = ((voltage_mv - BATTERY_MIN_VOLTAGE) * 100) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);
    if (battery_level < 0) battery_level = 0;
    if (battery_level > 100) battery_level = 100;
    LOG_INF("Battery level: %d%%, Voltage: %d mV", battery_level, voltage_mv);

    // Set LED color
    if (battery_level >= 75) {
        battery_led_color = LED_GREEN;
    } else if (battery_level >= 50) {
        battery_led_color = LED_YELLOW;
    } else if (battery_level >= 25) {
        battery_led_color = LED_ORANGE;
    } else {
        battery_led_color = LED_RED;
    }

    if (k_uptime_get() < Z_TIMEOUT_MS_TICKS(5000)) {
        LOG_INF("System just started, setting battery LED color %d", battery_led_color);
        set_led_status(battery_led_color);
        k_timer_start(&led_timer, K_SECONDS(10), K_NO_WAIT);
    }

    int64_t now_ms = k_uptime_get();
    if (now_ms - last_voltage_time >= 1000) {
        if (last_voltage_mv > 0) {
            if ((voltage_mv - last_voltage_mv) > CHARGING_THRESHOLD_DELTA) {
                is_charging = true;
                k_timer_start(&charging_pulse_timer, K_MSEC(500), K_NO_WAIT);
                battery_warning_active = false;
                k_timer_stop(&battery_warning_timer);
                k_timer_stop(&battery_pulse_timer);
                LOG_INF("Charging detected, starting %d pulse", battery_led_color);
            } else if (is_charging && voltage_mv >= FULL_CHARGE_THRESHOLD) {
                k_timer_stop(&charging_pulse_timer);
                set_led_status(LED_GREEN);
                max_voltage_mv = voltage_mv;
                LOG_INF("Full charge reached, static green");
            }
        }
        last_voltage_time = now_ms;
        last_voltage_mv = voltage_mv;
        if (voltage_mv > max_voltage_mv) {
            max_voltage_mv = voltage_mv;
        }
    }

    if (battery_level < BATTERY_CRITICAL_THRESHOLD && !is_charging && !battery_warning_active) {
        battery_warning_active = true;
        k_timer_start(&battery_warning_timer, K_NO_WAIT, K_NO_WAIT);
    }

    // Send battery level via ESB
    send_battery_level(voltage_mv);
}

// ESB transmission
void send_battery_level(uint32_t voltage) {
   // Prepare 8-byte payload
    // Prepare 8-byte payload with packet type 0x06 for battery voltage
    uint8_t payload_data[8] = {0x06}; // Packet type 0x06 for voltage
    memcpy(payload_data + 1, &voltage, sizeof(voltage)); // Offset by 1 byte for packet type

    // Set payload length and copy to tx_payload
    tx_payload.length = sizeof(payload_data);
    memcpy(tx_payload.data, payload_data, sizeof(payload_data));

    // Send via ESB
    int err = esb_write_payload(&tx_payload);
    if (err == 0) {
        LOG_INF("TX -> Voltage: %d mV", voltage);
        blink_led(LED_GREEN, 100, 2);
    } else {
        LOG_ERR("Failed to send voltage, err %d", err);
        blink_led(LED_RED, 100, 3);
    }
    set_led_status(LED_OFF);
}

void sample_and_transmit(struct esb_evt const *event) {
    static uint8_t last_payload_data[2] = {0};
    uint8_t payload_data[8] = {0};
    int err = esb_write_payload(&tx_payload);
    if (err == 0) {
        LOG_INF("TX -> D+: %d, D-: %d", payload_data[1], payload_data[2]);
        blink_led(LED_GREEN, 100, 1);
    } else {
        LOG_ERR("ESB write payload failed, err %d", err);
        blink_led(LED_RED, 100, 3);
    }
    set_led_status(LED_OFF);
}

// USB callback
void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param) {
    ARG_UNUSED(param);
    switch (status) {
    case USB_DC_CONFIGURED:
        LOG_INF("USB_DC_CONFIGURED received, setting usb_configured = true");
        usb_configured = true;
        blink_led(LED_GREEN, 200, 2);
        break;
    case USB_DC_DISCONNECTED:
        LOG_INF("USB_DC_DISCONNECTED received, setting usb_configured = false");
        usb_configured = false;
        blink_led(LED_RED, 200, 2);
        break;
    default:
        break;
    }
    set_led_status(LED_OFF);
}

/* void read_spi_payload(void) {
    uint8_t tx_buffer[SPI_BUFFER_SIZE] = {0}; // Data to send via MOSI (data_minus, P0.17)
    uint8_t rx_buffer[SPI_BUFFER_SIZE] = {0}; // Buffer for 8-byte payload from MISO (data_plus, P0.20)
    struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = SPI_BUFFER_SIZE
    };
    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = SPI_BUFFER_SIZE
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };
    struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1
    };
    static struct spi_config spi_cfg = {
        .frequency = 32000000U, // 32 MHz, adjustable
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA, // Adjust mode as per slave
        .slave = 0,
        .cs = { .gpio = NULL }, // Using gpio0 22 as CSN from dts, properly initialize as struct spi_cs_control
    };
    int err = spi_transceive(spi3_dev, &spi_cfg, &tx, &rx);
    if (err) {
        LOG_ERR("SPI transceive failed, err %d", err);
        blink_led(LED_RED, 100, 3);
        return;
    } else {
        LOG_INF("8-byte SPI payload received:");
        LOG_HEXDUMP_INF(rx_buffer, SPI_BUFFER_SIZE, "Raw Payload");
    }
} */

int main(void) {
    int err;
    LOG_INF("Starting 2.4GHz Transmitter");

    // Initialize USB HID
    hid_dev = device_get_binding("HID_0");
    if (!hid_dev) {
        LOG_ERR("Failed to get HID_0 device");
        blink_led(LED_RED, 100, 5);
    }
    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), NULL);
    err = usb_hid_init(hid_dev);
    if (err) {
        LOG_ERR("Failed to init USB HID, err %d", err);
        blink_led(LED_RED, 100, 5);
    }
    LOG_INF("Attempting to enable USB...");
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        err = usb_enable(&usb_status_cb);
        if (err) {
            LOG_ERR("Failed to enable USB, err %d", err);
            blink_led(LED_RED, 100, 5);
        }
    }
    // Initialize LEDs
    if (!device_is_ready(red_led.port) || !device_is_ready(green_led.port) || !device_is_ready(blue_led.port)) {
        LOG_ERR("RGB LED GPIOs not ready");
        blink_led(LED_RED, 100, 5);
    } 
    LOG_INF("RGB LED GPIOs ready: Red pin %d, Green pin %d, Blue pin P1.0%d", red_led.pin, green_led.pin, blue_led.pin);
    gpio_pin_configure_dt(&red_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&green_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT);

    k_sleep(K_MSEC(100)); // Wait for clock stabilization
    esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
    esb_config.mode = ESB_MODE_PTX;
    esb_config.bitrate = ESB_BITRATE_2MBPS;
    esb_config.retransmit_delay = 100; // 100 ms retransmit delay
    esb_config.tx_output_power = ESB_TX_POWER_8DBM;
    esb_config.payload_length = 8;
    esb_config.retransmit_count = 3;
    esb_config.event_handler = sample_and_transmit;
    err = esb_init(&esb_config);
    if (err) {
        LOG_ERR("ESB init failed, err %d", err);
        blink_led(LED_RED, 100, 5);
    }

    // Set ESB address
    uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
    err = esb_set_base_address_0(base_addr_0);
    if (err) {
        LOG_ERR("Failed to set base address 0, err %d", err);
    }
    uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
    err = esb_set_prefixes(prefixes, 8);
    if (err) {
        LOG_ERR("Failed to set prefixes, err %d", err);
    }
    err = esb_start_tx();

    LOG_INF("Transmitter initialized successfully");
    blink_led(LED_BLUE, 200, 3);

    // Check battery status at startup
    LOG_INF("Checking initial battery status");
    check_battery_status();
    //k_timer_start(&battery_check_timer, K_MINUTES(5), K_MINUTES(5));

    /* // Initialize SPI
    if (!device_is_ready(spi3_dev)) {
        LOG_ERR("SPI3 device not ready");
        blink_led(LED_RED, 100, 5);
        return -ENODEV;
    }
    LOG_INF("SPI initialized: P0.17 (MOSI), P0.20 (MISO)"); */

    // Main loop to read SPI payload periodically
    //read_spi_payload();
    k_sleep(K_SECONDS(1)); // Adjust polling rate as needed
    
    LOG_INF("Main loop completed, exiting.");
    return 0;
}
