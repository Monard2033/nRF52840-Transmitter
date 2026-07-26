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
#include <string.h>
#include <nrfx.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#define ERROR_PAYLOAD_FLAG 0x80

LOG_MODULE_REGISTER(transmitter, LOG_LEVEL_INF);
void sample_and_transmit(struct esb_evt const *event);
void enter_low_power(struct k_timer *dummy);
void update_sleep_timeout(void);
void led_timer_handler(struct k_timer *timer);
void check_battery_status(void);
void battery_warning_timer_handler(struct k_timer *timer);
void battery_pulse_timer_handler(struct k_timer *timer);
static void charging_pulse_timer_handler(struct k_timer *timer);
static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param);
static const struct device *hid_dev;

// State machine states for main task
enum system_state {
    STATE_INIT,
    STATE_CHECK_BATTERY,
    STATE_SEND_WINDOWS_KEY,
    STATE_TEST_PINS,
    STATE_CYCLE_LEDS,
    STATE_IDLE,
};

// Global state variable
static enum system_state current_state = STATE_INIT;
static bool main_task_completed = false; // Flag to stop main_task_thread after one cycle

// ESB configuration
struct esb_config esb_config = ESB_DEFAULT_CONFIG;
static struct esb_payload tx_payload;

// USB HID configuration
static K_SEM_DEFINE(usb_configured_sem, 0, 1);
static volatile bool usb_configured = false;
static const struct device *hid_dev;

// Pin definitions from device tree
static const struct gpio_dt_spec data_plus = GPIO_DT_SPEC_GET(DT_N_NODELABEL_data_plus_pin, gpios);
static const struct gpio_dt_spec data_minus = GPIO_DT_SPEC_GET(DT_N_NODELABEL_data_minus_pin, gpios);
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

// Hysteresis sleep configuration
#define BASE_TIMEOUT K_SECONDS(60)
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

// Timer definitions
// Forward declaration for led_timer_handler
static K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
static K_TIMER_DEFINE(inactivity_timer, NULL, enter_low_power);
static K_TIMER_DEFINE(battery_warning_timer, battery_warning_timer_handler, NULL);
static K_TIMER_DEFINE(battery_pulse_timer, battery_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(charging_pulse_timer, charging_pulse_timer_handler, NULL);

// HID report descriptor for a standard keyboard
static const uint8_t hid_report_desc[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x75, 0x01,
    0x95, 0x08, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
    0x15, 0x00, 0x25, 0x01, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 0x75, 0x08,
    0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00,
    0x29, 0x65, 0x81, 0x00, 0xC0
};

// Function declarations
void sample_and_transmit(struct esb_evt const *event);
void enter_low_power(struct k_timer *dummy);
void update_sleep_timeout(void);
void led_timer_handler(struct k_timer *timer);
void check_battery_status(void);
void battery_warning_timer_handler(struct k_timer *timer);
void battery_pulse_timer_handler(struct k_timer *timer);
static void charging_pulse_timer_handler(struct k_timer *timer);
static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param);
void set_led_status(enum led_color color);
void blink_led(enum led_color color, int duration_ms, int blink_count);
void cycle_leds(void);
void pin_activity_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void send_windows_key(void);
void test_pins(void);

// LED control functions
void set_led_status(enum led_color color) {
    gpio_pin_set_dt(&red_led, (color == LED_RED || color == LED_YELLOW || color == LED_ORANGE) ? 1 : 0);
    gpio_pin_set_dt(&green_led, (color == LED_GREEN || color == LED_YELLOW) ? 1 : 0);
    gpio_pin_set_dt(&blue_led, (color == LED_BLUE) ? 1 : 0);
}

void blink_led(enum led_color color, int duration_ms, int blink_count) {
    for (int i = 0; i < blink_count; i++) {
        LOG_INF("Blinking LED %d: ON", color);
        set_led_status(color);
        k_sleep(K_MSEC(duration_ms));
        LOG_INF("Blinking LED %d: OFF", color);
        set_led_status(LED_OFF);
        k_sleep(K_MSEC(duration_ms));
    }
}

void cycle_leds(void) {
    const enum led_color colors[] = {LED_RED, LED_GREEN, LED_BLUE};
    for (int j = 0; j < 3; j++) { // Reduced to 3 colors for brevity
        LOG_INF("Testing LED %d (%s)", colors[j],
                colors[j] == LED_RED ? "RED" :
                colors[j] == LED_GREEN ? "GREEN" :
                colors[j] == LED_BLUE ? "BLUE" : "OFF");
        blink_led(colors[j], 500, 2); // 2 blinks, 500ms on/off
        k_sleep(K_MSEC(200)); // Short pause between LEDs
    }
    LOG_INF("LED test completed");
}

// Timer handlers
void led_timer_handler(struct k_timer *timer) {
    if (!is_charging) {
        set_led_status(LED_OFF);
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
    }
}

void charging_pulse_timer_handler(struct k_timer *timer) {
    static bool led_on = true;
    if (is_charging) {
        set_led_status(led_on ? LED_GREEN : LED_OFF);
        led_on = !led_on;
        k_timer_start(&charging_pulse_timer, K_MSEC(1000), K_NO_WAIT);
    } else {
        k_timer_stop(&charging_pulse_timer);
        set_led_status(LED_OFF);
    }
}

// Power management
void enter_low_power(struct k_timer *dummy) {
    LOG_INF("Entering System OFF mode");
    k_timer_stop(&led_timer);
    k_timer_stop(&battery_warning_timer);
    k_timer_stop(&battery_pulse_timer);
    k_timer_stop(&charging_pulse_timer);
    esb_disable();
    set_led_status(LED_OFF);
}

void update_sleep_timeout(void) {
    int64_t now_ms = k_uptime_get();
    k_ticks_t now_ticks = k_ms_to_ticks_ceil32(now_ms);
    k_ticks_t last_ticks = k_ms_to_ticks_ceil32(last_sleep_time);
    k_ticks_t grace_ticks = k_ms_to_ticks_ceil32(GRACE_PERIOD_MS);
    if (last_sleep_time != 0 && (now_ticks - last_ticks) < grace_ticks) {
        wakeup_count++;
        if (wakeup_count == 1) {
            current_timeout = K_SECONDS(30);
            LOG_INF("Quick wakeup, sleep timeout 30s");
        }
    } else {
        wakeup_count = 0;
        current_timeout = BASE_TIMEOUT;
        LOG_INF("Resetting sleep timeout to 60s");
    }
    last_sleep_time = now_ms;
    k_timer_start(&inactivity_timer, current_timeout, K_NO_WAIT);
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
    uint32_t voltage_mv = ((adc_value * BATTERY_MAX_VOLTAGE) * 10 / (1 << 12));
    battery_level = ((voltage_mv - BATTERY_MIN_VOLTAGE) * 100) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);
    if (battery_level < 0) battery_level = 0;
    if (battery_level > 100) battery_level = 100;
    LOG_INF("Battery level: %d%%, Voltage: %d mV", battery_level, voltage_mv);

    if (k_uptime_get() < Z_TIMEOUT_MS_TICKS(5000)) {
        LOG_INF("System just started, updating LED based on battery.");
        if (battery_level >= 75) {
            set_led_status(LED_GREEN);
        } else if (battery_level >= 50) {
            set_led_status(LED_YELLOW);
        } else if (battery_level >= 25) {
            set_led_status(LED_ORANGE);
        } else {
            set_led_status(LED_RED);
        }
        k_timer_start(&led_timer, K_SECONDS(5), K_NO_WAIT);
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
                LOG_INF("Charging detected, starting green pulse.");
            } else if (is_charging && voltage_mv >= FULL_CHARGE_THRESHOLD) {
                k_timer_stop(&charging_pulse_timer);
                set_led_status(LED_GREEN);
                max_voltage_mv = voltage_mv;
                LOG_INF("Full charge reached, static green.");
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
}

// USB callback
static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param) {
    ARG_UNUSED(param);
    switch (status) {
    case USB_DC_CONFIGURED:
        LOG_INF("USB_DC_CONFIGURED received, setting usb_configured = true");
        usb_configured = true;
        k_sem_give(&usb_configured_sem);
        blink_led(LED_GREEN, 200, 2); // Indicate USB connection success
        break;
    case USB_DC_DISCONNECTED:
        LOG_INF("USB_DC_DISCONNECTED received, setting usb_configured = false");
        usb_configured = false;
        blink_led(LED_RED, 200, 2); // Indicate USB disconnection
        break;
    default:
        break;
    }
}

// ESB transmission
void sample_and_transmit(struct esb_evt const *event) {
    static uint8_t last_payload_data[2] = {0};
    uint8_t payload_data[2] = {0}; // Placeholder for data_plus/data_minus
    if (payload_data[0] != last_payload_data[0] || payload_data[1] != last_payload_data[1]) {
        tx_payload.length = sizeof(payload_data);
        memcpy(tx_payload.data, payload_data, sizeof(payload_data));
        last_payload_data[0] = payload_data[0];
        last_payload_data[1] = payload_data[1];
        if (esb_write_payload(&tx_payload) == 0) {
            LOG_INF("TX -> D+: %d, D-: %d", payload_data[0], payload_data[1]);
            blink_led(LED_GREEN, 100, 1); // Indicate successful transmission
        } else {
            LOG_ERR("ESB write payload failed");
            blink_led(LED_RED, 100, 3); // Indicate failure
        }
    }
}

void send_windows_key(void) {
    uint8_t hid_report[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    tx_payload.length = 8;
    memcpy(tx_payload.data, hid_report, 8);
    if (esb_write_payload(&tx_payload) == 0) {
        LOG_INF("TX -> Windows key press sent");
        blink_led(LED_GREEN, 200, 3); // Success
    } else {
        LOG_ERR("Failed to send Windows key press");
        blink_led(LED_RED, 100, 3); // Failure
    }
    k_sleep(K_MSEC(100));
    memset(hid_report, 0, 8);
    tx_payload.length = 8;
    memcpy(tx_payload.data, hid_report, 8);
    if (esb_write_payload(&tx_payload) == 0) {
        LOG_INF("TX -> Windows key release sent");
        blink_led(LED_GREEN, 200, 1); // Success
    } else {
        LOG_ERR("Failed to send Windows key release");
        blink_led(LED_RED, 500, 3); // Failure
    }
}

void test_pins(void) {
    int ret;

    LOG_INF("Starting test for P0.%d (DATA_PLUS) and P0.%d (DATA_MINUS)", 
            data_plus.pin, data_minus.pin);

    for (int i = 0; i < 3; i++) {
        LOG_INF("Cycle %d: Setting DATA_PLUS and DATA_MINUS to HIGH", i + 1);
        ret = gpio_pin_set_dt(&data_plus, 1);
        if (ret != 0) {
            LOG_ERR("Failed to set DATA_PLUS (P0.%d) to HIGH: %d", data_plus.pin, ret);
            blink_led(LED_RED, 100, 3); // Failure
        } else {
            blink_led(LED_GREEN, 100, 1); // Success
        }
        ret = gpio_pin_set_dt(&data_minus, 1);
        if (ret != 0) {
            LOG_ERR("Failed to set DATA_MINUS (P0.%d) to HIGH: %d", data_minus.pin, ret);
            blink_led(LED_RED, 100, 3); // Failure
        } else {
            blink_led(LED_GREEN, 100, 1); // Success
        }
        k_sleep(K_MSEC(500));

        LOG_INF("Cycle %d: Setting DATA_PLUS and DATA_MINUS to LOW", i + 1);
        ret = gpio_pin_set_dt(&data_plus, 0);
        if (ret != 0) {
            LOG_ERR("Failed to set DATA_PLUS (P0.%d) to LOW: %d", data_plus.pin, ret);
            blink_led(LED_RED, 100, 3); // Failure
        } else {
            blink_led(LED_GREEN, 100, 1); // Success
        }
        ret = gpio_pin_set_dt(&data_minus, 0);
        if (ret != 0) {
            LOG_ERR("Failed to set DATA_MINUS (P0.%d) to LOW: %d", data_minus.pin, ret);
            blink_led(LED_RED, 100, 3); // Failure
        } else {
            blink_led(LED_GREEN, 100, 1); // Success
        }
        k_sleep(K_MSEC(500));
    }

    LOG_INF("Configuring pins as inputs with pull-up and active-low");
    ret = gpio_pin_configure_dt(&data_plus, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
    if (ret != 0) {
        LOG_ERR("Failed to configure DATA_PLUS (P0.%d) as input: %d", data_plus.pin, ret);
        blink_led(LED_RED, 100, 3); // Failure
        return;
    }
    ret = gpio_pin_configure_dt(&data_minus, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
    if (ret != 0) {
        LOG_ERR("Failed to configure DATA_MINUS (P0.%d) as input: %d", data_minus.pin, ret);
        blink_led(LED_RED, 100, 3); // Failure
        return;
    }

    int plus_state = gpio_pin_get_dt(&data_plus);
    int minus_state = gpio_pin_get_dt(&data_minus);
    if (plus_state < 0) {
        LOG_ERR("Failed to read DATA_PLUS (P0.%d) state: %d", data_plus.pin, plus_state);
        blink_led(LED_RED, 100, 3); // Failure
    } else {
        LOG_INF("DATA_PLUS (P0.%d) input state: %d (expected 1 due to pull-up)", 
                data_plus.pin, plus_state);
        blink_led(LED_GREEN, 100, 1); // Success
    }
    if (minus_state < 0) {
        LOG_ERR("Failed to read DATA_MINUS (P0.%d) state: %d", data_minus.pin, minus_state);
        blink_led(LED_RED, 100, 3); // Failure
    } else {
        LOG_INF("DATA_MINUS (P0.%d) input state: %d (expected 1 due to pull-up)", 
                data_minus.pin, minus_state);
        blink_led(LED_GREEN, 100, 1); // Success
    }

    LOG_INF("Pin test completed");
}

// GPIO interrupt handler
void pin_activity_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (pins & BIT(data_plus.pin)) {
        LOG_INF("Pin activity detected P%d on DATA_PLUS, resetting inactivity timer", gpio_pin_get_dt(&data_plus));
        blink_led(LED_BLUE, 100, 1); // Indicate activity
    }
    if (pins & BIT(data_minus.pin)) {
        LOG_INF("Pin activity detected P%d on DATA_MINUS, resetting inactivity timer", gpio_pin_get_dt(&data_minus));
        blink_led(LED_BLUE, 100, 1); // Indicate activity
    }
    update_sleep_timeout();
    k_timer_start(&led_timer, K_SECONDS(4), K_NO_WAIT);
    if (esb_start_tx() == 0) {
        blink_led(LED_GREEN, 100, 1); // Success
    } else {
        blink_led(LED_RED, 100, 3); // Failure
    }
}

// Main task thread for state machine (runs once)
void main_task(void *arg1, void *arg2, void *arg3) {
    while (!main_task_completed) {
        switch (current_state) {
            case STATE_INIT:
                LOG_INF("State: INIT");
                set_led_status(LED_OFF); // Ensure LEDs off at start
                current_state = STATE_CHECK_BATTERY;
                break;

            case STATE_CHECK_BATTERY:
                LOG_INF("State: CHECK_BATTERY");
                check_battery_status();
                k_timer_start(&inactivity_timer, current_timeout, K_NO_WAIT);
                current_state = STATE_SEND_WINDOWS_KEY;
                k_sleep(K_MSEC(100)); // Short delay for battery status
                break;

            case STATE_SEND_WINDOWS_KEY:
                LOG_INF("State: SEND_WINDOWS_KEY");
                send_windows_key();
                k_sleep(K_MSEC(1000)); // Wait for key press/release and blinks (~600ms)
                current_state = STATE_TEST_PINS;
                set_led_status(LED_OFF); // Ensure LEDs off
                break;

            case STATE_TEST_PINS:
                LOG_INF("State: TEST_PINS");
                test_pins();
                k_sleep(K_MSEC(3000)); // Wait for 3 cycles (~3s)
                current_state = STATE_CYCLE_LEDS;
                set_led_status(LED_OFF); // Ensure LEDs off
                break;

            case STATE_CYCLE_LEDS:
                LOG_INF("State: CYCLE_LEDS");
                cycle_leds();
                k_sleep(K_MSEC(3200)); // Wait for 3 colors (~3.2s)
                current_state = STATE_IDLE;
                set_led_status(LED_OFF); // Ensure LEDs off
                break;

            case STATE_IDLE:
                LOG_INF("State: IDLE");
                set_led_status(LED_OFF); // Ensure LEDs off in idle
                main_task_completed = true; // Stop after one cycle
                LOG_INF("Main task completed, waiting for inactivity timer");
                break;
        }
    }
    // Sleep indefinitely to allow inactivity timer to trigger
    k_sleep(K_FOREVER);
}

// Monitor thread for battery and GPIO
void monitor_thread_func(void *arg1, void *arg2, void *arg3) {
    static bool monitor_completed = false;
    if (!monitor_completed) {
        check_battery_status();
       monitor_completed = true; // Stop after one check
    }
        k_sleep(K_FOREVER);
}

// Thread definitions
K_THREAD_DEFINE(main_task_thread, 2048, main_task, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(monitor_thread, 1024, monitor_thread_func, NULL, NULL, NULL, 6, 0, 0);

int main(void) {
    int err;
    LOG_INF("Starting 2.4GHz HID Keyboard Transmitter");

    // Initialize USB HID
    hid_dev = device_get_binding("HID_0");
    if (!hid_dev) {
        LOG_ERR("Failed to get HID_0 device");
        return -ENODEV;
    }
    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), NULL);
    err = usb_hid_init(hid_dev);
    if (err) {
        LOG_ERR("Failed to init USB HID, err %d", err);
        return err;
    }
    LOG_INF("Attempting to enable USB...");
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        err = usb_enable(usb_status_cb);
        if (err) {
            LOG_ERR("Failed to enable USB, err %d", err);
            return err;
        }
    }
    // Initialize ESB with retries
    esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
    esb_config.mode = ESB_MODE_PTX;
    esb_config.bitrate = ESB_BITRATE_2MBPS;
    esb_config.payload_length = 8;
    esb_config.retransmit_count = 3;
    esb_config.event_handler = sample_and_transmit; 

    err = esb_init(&esb_config);
    if(err) {
        LOG_ERR("Failed to initialize ESB, err %d", err);
        blink_led(LED_RED, 500, 5); // Indicate ESB failure
    }
    LOG_INF("Transmitter initialized successfully");

    uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
    err = esb_set_base_address_0(base_addr_0);
    if (err) {
        LOG_ERR("Failed to set base address 0, err %d", err);
        return err;
    }
    uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
    err = esb_set_prefixes(prefixes, 8);
    if (err) {
        LOG_ERR("Failed to set prefixes, err %d", err);
        return err;
    }

    if (!esb_is_idle()) {
        LOG_WRN("ESB not idle, flushing buffers");
        esb_flush_tx();
        esb_flush_rx();
    }
    err = esb_start_tx();
    if (err) 
    LOG_ERR("ESB start_tx failed, err %d, ", err);
     blink_led(LED_RED, 500, 3); // Indicate ESB failure
    k_sleep(K_MSEC(100));

    // Initialize GPIOs
    if (!device_is_ready(data_plus.port) || !device_is_ready(data_minus.port)) {
        LOG_ERR("Data GPIOs not ready");
        return -ENODEV;
    }
    LOG_INF("Data GPIOs: data_plus=P0.%d, data_minus=P0.%d",
            data_plus.pin, data_minus.pin);
    err = gpio_pin_configure_dt(&data_plus, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
    if (err) {
        LOG_ERR("Failed to configure data_plus (P0.20), err %d", err);
        return err;
    }
    err = gpio_pin_configure_dt(&data_minus, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
    if (err) {
        LOG_ERR("Failed to configure data_minus (P0.22), err %d", err);
        return err;
    }

    if (!device_is_ready(red_led.port) || !device_is_ready(green_led.port) || !device_is_ready(blue_led.port)) {
        LOG_ERR("RGB LED GPIOs not ready");
        return -ENODEV;
    }
    LOG_INF("RGB LED GPIOs ready: Red pin %d, Green pin %d, Blue pin 1.0%d", red_led.pin, green_led.pin, blue_led.pin);
    gpio_pin_configure_dt(&red_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&green_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT);

    // Configure GPIO interrupts
    static struct gpio_callback pin_cb_data;
    gpio_init_callback(&pin_cb_data, pin_activity_handler, BIT(data_plus.pin) | BIT(data_minus.pin));
    err = gpio_add_callback(data_plus.port, &pin_cb_data);
    if (err) {
        LOG_ERR("Failed to add GPIO callback, err %d", err);
        return err;
    }
    gpio_pin_interrupt_configure_dt(&data_plus, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&data_minus, GPIO_INT_EDGE_BOTH);

    // Main thread yields to allow main_task_thread and monitor_thread to run
    while (1) {
        k_sleep(K_SECONDS(1));
    }

    LOG_INF("Main loop completed, exiting.");
    return 0;
}