/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
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
#include <zephyr/drivers/adc.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

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

/* --- LED and Timer Definitions --- */


/* --- LED Definitions --- */

static const struct gpio_dt_spec data_plus = GPIO_DT_SPEC_GET(DT_N_NODELABEL_data_plus_pin,gpios);
static const struct gpio_dt_spec data_minus = GPIO_DT_SPEC_GET(DT_N_NODELABEL_data_minus_pin,gpios);
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(adc_input));
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(DT_N_NODELABEL_red_led,gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(DT_N_NODELABEL_green_led,gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(DT_N_NODELABEL_blue_led,gpios);

enum led_color
{
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_OFF,
    LED_YELLOW,
    LED_ORANGE
};
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

static K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
static K_TIMER_DEFINE(inactivity_timer, NULL, enter_low_power);
static K_TIMER_DEFINE(battery_warning_timer, battery_warning_timer_handler, NULL);
static K_TIMER_DEFINE(battery_pulse_timer, battery_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(charging_pulse_timer, charging_pulse_timer_handler, NULL);

/* --- ESB and Power Management --- */
static struct esb_payload tx_payload;
static struct esb_config esb_config = ESB_DEFAULT_CONFIG;
#define ERROR_PAYLOAD_FLAG 0x80

/* --- USB HID Configuration --- */
static K_SEM_DEFINE(usb_configured_sem, 0, 1);
static volatile bool usb_configured = false;
#define REPORT_ID 1
#define KEY_CTRL_CODE_MIN 224
#define KEY_CTRL_CODE_MAX 231
#define KEY_CODE_MIN 0
#define KEY_CODE_MAX 101
#define KEY_PRESS_MAX 6
#define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)

/* --- Hysteresis Sleep Configuration --- */
#define BASE_TIMEOUT K_SECONDS(60)
#define GRACE_PERIOD_MS 2000
static k_timeout_t current_timeout = BASE_TIMEOUT;
static int wakeup_count = 0;
static int64_t last_sleep_time = 0;

/* --- Battery Configuration --- */
#define BATTERY_CRITICAL_THRESHOLD 15
#define BATTERY_MAX_VOLTAGE 4200
#define BATTERY_MIN_VOLTAGE 3300
#define CHARGING_THRESHOLD_DELTA 50
#define FULL_CHARGE_THRESHOLD 4190
static int battery_level = 100;
static bool battery_warning_active = false;
static int pulse_count = 0;
static int64_t last_voltage_time = 0;
static uint32_t last_voltage_mv = 0;
static uint32_t max_voltage_mv = 0;
static bool is_charging = false;

/* --- Function Definitions --- */

 /* HID Report Descriptor for a standard keyboard */
 static const uint8_t hid_report_desc[] = {
     0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x75, 0x01,
     0x95, 0x08, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
     0x15, 0x00, 0x25, 0x01, 0x81, 0x02, 0x95, 0x01,
     0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 0x75, 0x08,
     0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00,
     0x29, 0x65, 0x81, 0x00, 0xC0
 };

 struct usb_desc_strings {
    const uint8_t *product;
};



void led_timer_handler(struct k_timer *timer)
{
    if (!is_charging)
    {
        set_led_status(LED_OFF);
    }
}

void battery_warning_timer_handler(struct k_timer *timer)
{
    if (!is_charging && battery_warning_active)
    {
        k_timer_start(&battery_pulse_timer, K_NO_WAIT, K_NO_WAIT);
    }
}

void battery_pulse_timer_handler(struct k_timer *timer)
{
    if (pulse_count < 8 && !is_charging)
    {
        set_led_status(LED_RED);
        k_sleep(K_MSEC(500));
        set_led_status(LED_OFF);
        k_sleep(K_MSEC(500));
        pulse_count++;
    }
    else
    {
        pulse_count = 0;
        k_timer_stop(&battery_pulse_timer);
        if (battery_warning_active && !is_charging)
        {
            k_timer_start(&battery_warning_timer, K_MINUTES(5), K_NO_WAIT);
        }
    }
}

static void charging_pulse_timer_handler(struct k_timer *timer)
{
    static bool led_on = true;
    if (is_charging)
    {
        set_led_status(led_on ? LED_GREEN : LED_OFF);
        led_on = !led_on;
        k_timer_start(&charging_pulse_timer, K_MSEC(1000), K_NO_WAIT);
    }
    else
    {
        blink_led(LED_RED, 500, 3);
        k_timer_stop(&charging_pulse_timer);
        set_led_status(LED_OFF);
    }
}

void enter_low_power(struct k_timer *dummy)
{
    LOG_INF("Inactivity detected. Preparing to enter System OFF mode.");
    int retry_count = 0;
    const int max_retries = 3;
    while (retry_count < max_retries && !esb_is_idle())
    {
        LOG_INF("ESB is busy, waiting %d/%d...", retry_count + 1, max_retries);
        k_sleep(K_MSEC(20));
        retry_count++;
    }
    if (!esb_is_idle())
    {
        LOG_INF("ESB busy after %d retries", max_retries);
    }
    set_led_status(LED_BLUE);
    k_timer_start(&led_timer, K_SECONDS(4), K_NO_WAIT);
}

void update_sleep_timeout(void)
{
    int64_t now_ms = k_uptime_get();
    k_ticks_t now_ticks = k_ms_to_ticks_ceil32(now_ms);
    k_ticks_t last_ticks = k_ms_to_ticks_ceil32(last_sleep_time);
    k_ticks_t grace_ticks = k_ms_to_ticks_ceil32(GRACE_PERIOD_MS);
    if (last_sleep_time != 0 && (now_ticks - last_ticks) < grace_ticks)
    {
        wakeup_count++;
        if (wakeup_count == 1)
        {
            current_timeout = K_SECONDS(30); //original 2 minutes
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 2 minutes.");
        }
        else if (wakeup_count >= 2)
        {
            current_timeout = K_SECONDS(180);
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 3 minutes.");
            wakeup_count = 2;
        }
        else if (wakeup_count >= 3)
        {
            current_timeout = K_SECONDS(240);
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 4 minutes.");
            wakeup_count = 3;
        }
        else if (wakeup_count >= 4)
        {
            current_timeout = K_SECONDS(300);
            LOG_INF("Quick wakeup again, capping sleep timeout at 5 minutes.");
            wakeup_count = 4;
        }
    }
    else
    {
        wakeup_count = 0;
        current_timeout = BASE_TIMEOUT;
        LOG_INF("Long inactivity or fresh start, resetting sleep timeout to 1 minute.");
    }
    last_sleep_time = now_ms;
}

void pin_activity_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {

    if (pins & BIT(data_plus.pin)) {
        LOG_INF("Pin activity detected P%d on DATA_PLUS, resetting inactivity timer", gpio_pin_get_dt(&data_plus));
    }
    if (pins & BIT(data_minus.pin)) {
        LOG_INF("Pin activity detected P%d on DATA_MINUS, resetting inactivity timer", gpio_pin_get_dt(&data_minus));
    }
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
    uint32_t voltage_mv = ((adc_value * BATTERY_MAX_VOLTAGE) * 10 / (1 << 12)); // 12-bit ADC
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
    send_battery_level();
}

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    ARG_UNUSED(param);
    switch (status)
    {
    case USB_DC_CONFIGURED:
        LOG_INF("USB_DC_CONFIGURED received, setting usb_configured = true");
        usb_configured = true;
        k_sem_give(&usb_configured_sem);
        break;
    case USB_DC_DISCONNECTED:
        LOG_INF("USB_DC_DISCONNECTED received, setting usb_configured = false");
        usb_configured = false;
        break;
    default:
        break;
    }
}

void sample_and_transmit(struct esb_evt const *event) {
    static uint8_t last_payload_data[2] = {0};
    uint8_t payload_data[2] = {0}; // Example test payload
    if (payload_data[0] != last_payload_data[0] || payload_data[1] != last_payload_data[1]) {
        tx_payload.length = sizeof(payload_data);
        memcpy(tx_payload.data, payload_data, sizeof(payload_data));
        last_payload_data[0] = payload_data[0];
        last_payload_data[1] = payload_data[1];
        if (esb_write_payload(&tx_payload) == 0) {
            LOG_INF("TX -> D+: %d, D-: %d", payload_data[0], payload_data[1]);
        } else {
            LOG_ERR("ESB write payload failed");
        }
    }
}

void send_windows_key(void) {
    uint8_t hid_report[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Left GUI only
    tx_payload.length = 8;
    memcpy(tx_payload.data, hid_report, 8);
    if (esb_write_payload(&tx_payload) == 0) {
        LOG_INF("TX -> Windows key press sent");
        blink_led(LED_RED, 200, 3);
    } else {
        LOG_ERR("Failed to send Windows key press, ESB error");
    }

    k_sleep(K_MSEC(100)); // Delay to ensure key is registered

    memset(hid_report, 0, 8); // Clear all bytes for key release
    tx_payload.length = 8;
    memcpy(tx_payload.data, hid_report, 8);
    if (esb_write_payload(&tx_payload) == 0) {
        LOG_INF("TX -> Windows key release sent");
    } else {
        LOG_ERR("Failed to send Windows key release, ESB error");
    }
}
int main(void)
{
    int err;
    LOG_INF("Starting 2.4GHz HID Keyboard Transmitter");
    
    hid_dev = device_get_binding("HID_0");
    LOG_INF("USB device found and ready.");

    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), NULL);
    LOG_INF("HID registered.");

    err = usb_hid_init(hid_dev);
    if (err)
    {
        LOG_ERR("Failed to init USB HID, err %d", err);
    }
    LOG_INF("HID initialized.");

    LOG_INF("Attempting to enable USB...");
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK))
    {
        err = usb_enable(NULL);
        if (err)
        {
            LOG_ERR("Failed to enable USB, err %d", err);
        }
        LOG_INF("USB enabled successfully.");
    }
   
    esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
    esb_config.mode = ESB_MODE_PTX;
    esb_config.bitrate = ESB_BITRATE_2MBPS;
    esb_config.payload_length = 8;
    esb_config.retransmit_count = 3;
    esb_config.event_handler = sample_and_transmit;
    err = esb_init(&esb_config);
    if (err)
    {
        LOG_ERR("ESB initialization failed, err %d", err);
    }
    LOG_INF("Transmitter initialized successfully");

    /* Initialize GPIOs */
    if (!device_is_ready(data_plus.port) || !device_is_ready(data_minus.port))
    {
        LOG_ERR("Data GPIOs not ready");
    }
    else 
    {
        LOG_INF("Data GPIOs ready: D+ pin %d, D- pin %d", data_plus.pin, data_minus.pin);
    }
    gpio_pin_configure_dt(&data_plus, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&data_minus, GPIO_INPUT | GPIO_PULL_UP);

    /* Initialize RGB LED */
    if (!device_is_ready(red_led.port) || !device_is_ready(green_led.port) || !device_is_ready(blue_led.port))
    {
        LOG_ERR("RGB LED GPIOs not ready");
    }
    else 
    {
        LOG_INF("RGB LED GPIOs ready: Red pin %d, Green pin %d, Blue pin %d", red_led.pin, green_led.pin, blue_led.pin);
    }
    gpio_pin_configure_dt(&red_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&green_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT);

    k_timer_start(&led_timer, K_SECONDS(5), K_NO_WAIT);
    check_battery_status();

    /* Configure GPIO Interrupt for Wake-up */
    static struct gpio_callback pin_cb_data;
    gpio_init_callback(&pin_cb_data, pin_activity_handler, BIT(data_plus.pin) | BIT(data_minus.pin));
    err = gpio_add_callback(data_plus.port, &pin_cb_data);
    if (err)
    {
        LOG_ERR("Failed to add callback, err %d", err);
    }
    gpio_pin_interrupt_configure_dt(&data_plus, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&data_minus, GPIO_INT_EDGE_BOTH);

    uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
    err = esb_set_base_address_0(base_addr_0);
    if (err)
    {
        LOG_ERR("Failed to set base address 0");
    }

    uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
    err = esb_set_prefixes(prefixes, 8);
    if (err)
    {
        LOG_ERR("Failed to set prefixes");
    }
    k_sleep(K_MSEC(100));
    if (!esb_is_idle()) {
        LOG_WRN("ESB not idle, flushing buffers");
        esb_flush_tx();
        esb_flush_rx();
    }
    err = esb_start_tx();
    if (err)
    {
        LOG_ERR("ESB start_tx failed: %d", err);
    }
    charging_pulse_timer_handler(&charging_pulse_timer); // Start charging pulse timer

    LOG_INF("Blinking blue LED to indicate ESB connect attempt...");
    blink_led(LED_BLUE, 500, 5);
    

    /* Send Windows key sequence (for testing) */
    send_windows_key();
    k_sleep(K_MSEC(200)); // Delay to ensure key is registered
    send_windows_key();
    LOG_INF("Windows key sequence sent");

    return 0;
}