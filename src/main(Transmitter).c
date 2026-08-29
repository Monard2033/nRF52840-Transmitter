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
static void int_in_ready_cb(const struct device *hid_dev);
static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param);

/* --- Pin Definitions from Device Tree --- */
// Replace with direct GPIO pin definitions if device tree nodes are not available
#define DATA_PLUS_NODE  DT_ALIAS(data_plus)
#define DATA_MINUS_NODE DT_ALIAS(data_minus)

#if DT_NODE_HAS_STATUS(DATA_PLUS_NODE, okay)
static const struct gpio_dt_spec data_plus = GPIO_DT_SPEC_GET(DATA_PLUS_NODE, gpios);
#else
static const struct gpio_dt_spec data_plus = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 17, // <-- Set to your actual pin number for data_plus
    .dt_flags = GPIO_INPUT,
};
#endif

#if DT_NODE_HAS_STATUS(DATA_MINUS_NODE, okay)
static const struct gpio_dt_spec data_minus = GPIO_DT_SPEC_GET(DATA_MINUS_NODE, gpios);
#else
static const struct gpio_dt_spec data_minus = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 20, // <-- Set to your actual pin number for data_minus
    .dt_flags = GPIO_INPUT,
};
#endif
// Replace with your actual GPIO port and pin numbers for the RGB LED
static const struct gpio_dt_spec red_led = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 22, // <-- Set to your actual pin number for red LED
    .dt_flags = GPIO_OUTPUT,
};
static const struct gpio_dt_spec green_led = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 24, // <-- Set to your actual pin number for green LED
    .dt_flags = GPIO_OUTPUT,
};
static const struct gpio_dt_spec blue_led = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 11, // <-- Set to your actual pin number for blue LED
    .dt_flags = GPIO_OUTPUT,
};

#if DT_NODE_HAS_STATUS(ADC_NODE, okay)
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET();
#else
static const struct adc_dt_spec adc_channel = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(adc)),
    .channel_id = 0, // P0.04 for battery ADC
};
#endif

/* --- LED and Timer Definitions --- */
enum led_color
{
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_OFF,
    LED_YELLOW,
    LED_ORANGE
};
static K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
static K_TIMER_DEFINE(inactivity_timer, NULL, enter_low_power);
static K_TIMER_DEFINE(battery_warning_timer, battery_warning_timer_handler, NULL);
static K_TIMER_DEFINE(battery_pulse_timer, battery_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(charging_pulse_timer, charging_pulse_timer_handler, NULL);

/* --- ESB and Power Management --- */
static struct esb_payload tx_payload;
static struct esb_config esb_config = ESB_DEFAULT_CONFIG;
static uint8_t last_payload_data[2] = {0xFF, 0xFF};
#define ERROR_PAYLOAD_FLAG 0x80 // High bit indicates error message

/* --- USB HID Configuration --- */
static const struct device *hid_dev;
static K_SEM_DEFINE(usb_configured_sem, 0, 1);
static volatile bool usb_configured = false;
#define REPORT_ID 1
#define KEY_CTRL_CODE_MIN 224
#define KEY_CTRL_CODE_MAX 231
#define KEY_CODE_MIN 0
#define KEY_CODE_MAX 101
#define KEY_PRESS_MAX 6
#define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)
static const uint8_t hid_report_desc[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x75, 0x01,
    0x95, 0x08, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
    0x15, 0x00, 0x25, 0x01, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 0x75, 0x08,
    0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00,
    0x29, 0x65, 0x81, 0x00, 0xC0};
static void int_in_ready_cb(const struct device *hid_dev)
{
    LOG_INF("IN endpoint ready to send data. Sending empty report.");
    static uint8_t report[2] = {REPORT_ID, 0};
    if (hid_int_ep_write(hid_dev, report, sizeof(report), NULL))
    {
        LOG_ERR("Failed to submit report");
    }
    else
    {
        report[1]++;
    }
}

// Define your HID operations structure
static const struct hid_ops my_hid_ops = {
    .int_in_ready = int_in_ready_cb,
};

/* --- Hysteresis Sleep Configuration --- */
#define BASE_TIMEOUT K_SECONDS(60) // 1 minute default
#define GRACE_PERIOD_MS 2000       // 2 seconds in milliseconds
static k_timeout_t current_timeout = BASE_TIMEOUT;
static int wakeup_count = 0;
static int64_t last_sleep_time = 0;

/* --- Battery Configuration --- */
#define BATTERY_CRITICAL_THRESHOLD 15 // 15% critical level
#define BATTERY_MAX_VOLTAGE 4200      // mV (fully charged, e.g., 4.2V)
#define BATTERY_MIN_VOLTAGE 3300      // mV (fully discharged, e.g., 3.3V)
#define CHARGING_THRESHOLD_DELTA 50   // 50mV increase indicates charging
#define FULL_CHARGE_THRESHOLD 4190    // 10mV below max to account for noise
static int battery_level = 100;       // Placeholder, updated by ADC
static bool battery_warning_active = false;
static int pulse_count = 0;
static int64_t last_voltage_time = 0;
static uint32_t last_voltage_mv = 0;
static uint32_t max_voltage_mv = 0;
static bool is_charging = false;

/* --- Function Definitions --- */

void set_led_status(enum led_color color)
{
    if (!device_is_ready(red_led.port) || !device_is_ready(green_led.port) || !device_is_ready(blue_led.port))
    {
        LOG_INF("RGB LED GPIOs not ready");
    }
    gpio_pin_configure_dt(&red_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&green_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT);
    switch (color)
    {
    case LED_RED:
        gpio_pin_set_dt(&red_led, 1);
        gpio_pin_set_dt(&green_led, 0);
        gpio_pin_set_dt(&blue_led, 0);
        break;
    case LED_GREEN:
        gpio_pin_set_dt(&red_led, 0);
        gpio_pin_set_dt(&green_led, 1);
        gpio_pin_set_dt(&blue_led, 0);
        break;
    case LED_BLUE:
        gpio_pin_set_dt(&red_led, 0);
        gpio_pin_set_dt(&green_led, 0);
        gpio_pin_set_dt(&blue_led, 1);
        break;
    case LED_YELLOW:
        gpio_pin_set_dt(&red_led, 1);
        gpio_pin_set_dt(&green_led, 1);
        gpio_pin_set_dt(&blue_led, 0);
        break;
    case LED_ORANGE:
        gpio_pin_set_dt(&red_led, 1);
        gpio_pin_set_dt(&green_led, 1);
        gpio_pin_set_dt(&blue_led, 0); // Approximate orange with red+green
        break;
    case LED_OFF:
        gpio_pin_set_dt(&red_led, 0);
        gpio_pin_set_dt(&green_led, 0);
        gpio_pin_set_dt(&blue_led, 0);
        break;
    }
}

void led_timer_handler(struct k_timer *timer)
{
    if (!is_charging)
    { // Only turn off if not charging
        set_led_status(LED_OFF);
    }
}

void battery_warning_timer_handler(struct k_timer *timer)
{
    if (!is_charging && battery_warning_active)
    {
        k_timer_start(&battery_pulse_timer, K_NO_WAIT, K_NO_WAIT); // Start pulsing sequence
    }
}

void battery_pulse_timer_handler(struct k_timer *timer)
{
    if (pulse_count < 8 && !is_charging)
    { // 4 pulses, 2 states each (on/off)
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
            k_timer_start(&battery_warning_timer, K_MINUTES(5), K_NO_WAIT); // Repeat after 5 minutes
        }
    }
}

static void charging_pulse_timer_handler(struct k_timer *timer)
{
    static bool led_on = true;
    if (is_charging)
    {
        set_led_status(led_on ? LED_GREEN : LED_OFF); // Green pulse for charging
        led_on = !led_on;
        k_timer_start(&charging_pulse_timer, K_MSEC(500), K_NO_WAIT); // Pulse every 500ms
    }
    else
    {
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
        k_sleep(K_MSEC(20)); // 20ms delay per retry
        retry_count++;
    }
    if (!esb_is_idle())
    {

        LOG_INF("ESB busy after %d retries", max_retries);
    }
    // esb_disable();  // Disable ESB to save power
    set_led_status(LED_BLUE);                           // Indicate power-saving mode
    k_timer_start(&led_timer, K_SECONDS(4), K_NO_WAIT); // Blue LED for 4 seconds
}

void update_sleep_timeout(void)
{
    int64_t now_ms = k_uptime_get();                               // Current time in milliseconds
    k_ticks_t now_ticks = k_ms_to_ticks_ceil32(now_ms);            // Convert to ticks
    k_ticks_t last_ticks = k_ms_to_ticks_ceil32(last_sleep_time);  // Convert last time to ticks
    k_ticks_t grace_ticks = k_ms_to_ticks_ceil32(GRACE_PERIOD_MS); // Convert grace period to ticks
    if (last_sleep_time != 0 && (now_ticks - last_ticks) < grace_ticks)
    {
        wakeup_count++;
        if (wakeup_count == 1)
        {
            current_timeout = K_SECONDS(120); // 2 minutes
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 2 minutes.");
        }
        else if (wakeup_count >= 2)
        {
            current_timeout = K_SECONDS(180); // 3 minutes
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 3 minutes.");
            wakeup_count = 2;
        }
        else if (wakeup_count >= 3)
        {
            current_timeout = K_SECONDS(240); // 4 minutes
            LOG_INF("Quick wakeup detected, increasing sleep timeout to 4 minutes.");
            wakeup_count = 3;
        }
        else if (wakeup_count >= 4)
        {
            current_timeout = K_SECONDS(300); // 5 minutes
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

void pin_activity_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    update_sleep_timeout();
    k_timer_start(&inactivity_timer, K_NO_WAIT, current_timeout);
    set_led_status(LED_BLUE); // Blue LED during ESB connect attempt
    k_timer_start(&led_timer, K_SECONDS(4), K_NO_WAIT);
    esb_start_tx(); // Re-enable ESB on GPIO interrupt
    LOG_INF("Pin activity detected, resetting inactivity timer");
}

void check_battery_status(void)
{
    int16_t buf;
    struct adc_sequence sequence = {
        .channels = BIT(adc_channel.channel_id),
        .buffer = &buf,
        .buffer_size = sizeof(buf),
        .resolution = 12,
    };

    if (adc_channel.dev && device_is_ready(adc_channel.dev))
    {
        if (adc_sequence_init_dt(&adc_channel, &sequence) == 0)
        {
            if (adc_read(adc_channel.dev, &sequence) == 0)
            {
                uint32_t adc_value = buf;
                uint32_t voltage_mv = (adc_value * BATTERY_MAX_VOLTAGE) / (1 << 12); // 12-bit ADC
                battery_level = ((voltage_mv - BATTERY_MIN_VOLTAGE) * 100) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);
                if (battery_level < 0)
                    battery_level = 0;
                if (battery_level > 100)
                    battery_level = 100;

                LOG_INF("Battery level: %d%%, Voltage: %d mV", battery_level, voltage_mv);

                // Power-on battery status
                if (k_uptime_get() < 5000)
                {
                    if (battery_level > 75)
                    {
                        set_led_status(LED_GREEN);
                    }
                    else if (battery_level > 50)
                    {
                        set_led_status(LED_YELLOW);
                    }
                    else if (battery_level > 25)
                    {
                        set_led_status(LED_ORANGE);
                    }
                    else
                    {
                        set_led_status(LED_RED);
                    }
                    k_timer_start(&led_timer, K_SECONDS(5), K_NO_WAIT);
                }

                // Charging detection
                int64_t now_ms = k_uptime_get();
                if (now_ms - last_voltage_time >= 1000)
                { // Check every 1 second
                    if (last_voltage_mv > 0)
                    {
                        if ((voltage_mv - last_voltage_mv) > CHARGING_THRESHOLD_DELTA)
                        {
                            is_charging = true;
                            k_timer_start(&charging_pulse_timer, K_MSEC(500), K_NO_WAIT); // Start pulsating
                            battery_warning_active = false;                               // Disable warning during charging
                            k_timer_stop(&battery_warning_timer);
                            k_timer_stop(&battery_pulse_timer);
                            LOG_INF("Charging detected, starting green pulse.");
                        }
                        else if (is_charging && voltage_mv >= FULL_CHARGE_THRESHOLD)
                        {
                            k_timer_stop(&charging_pulse_timer);
                            set_led_status(LED_GREEN); // Static green at full charge
                            max_voltage_mv = voltage_mv;
                            LOG_INF("Full charge reached, static green.");
                        }
                        else if (!is_charging && last_voltage_mv > 0 && (voltage_mv - last_voltage_mv) <= -CHARGING_THRESHOLD_DELTA)
                        {
                            // Voltage drop without charging, do nothing (LED off)
                        }
                    }
                    last_voltage_time = now_ms;
                    last_voltage_mv = voltage_mv;
                    if (voltage_mv > max_voltage_mv)
                    {
                        max_voltage_mv = voltage_mv; // Update max voltage
                    }
                }

                // Critical battery warning (only if not charging)
                if (battery_level < BATTERY_CRITICAL_THRESHOLD && !is_charging && !battery_warning_active)
                {
                    battery_warning_active = true;
                    k_timer_start(&battery_warning_timer, K_NO_WAIT, K_NO_WAIT);
                }
            }
            else
            {
                LOG_INF("ADC read failed");
            }
        }
        else
        {
            LOG_ERR("ADC sequence init failed");
        }
    }
    else
    {
        LOG_ERR("ADC device not ready");
    }
}

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    ARG_UNUSED(param);
    LOG_INF("USB status callback: status=%d", status);
    switch (status)
    {
    case USB_DC_CONFIGURED:
        LOG_INF("USB_DC_CONFIGURED received, setting usb_configured = true");
        usb_configured = true;
        k_sem_give(&usb_configured_sem);
        break;
    default:
        LOG_INF("Unknown USB status code: %d", status);
        break;
    }
}

void sample_and_transmit(struct esb_evt const *event)
{
    uint8_t payload_data[2];
    payload_data[0] = gpio_pin_get_dt(&data_plus);
    payload_data[1] = gpio_pin_get_dt(&data_minus);

    if (payload_data[0] != last_payload_data[0] || payload_data[1] != last_payload_data[1])
    {
        k_timer_start(&inactivity_timer, K_NO_WAIT, current_timeout);
        tx_payload.length = sizeof(payload_data);
        memcpy(tx_payload.data, payload_data, sizeof(payload_data));

        last_payload_data[0] = payload_data[0];
        last_payload_data[1] = battery_warning_active ? 1 : payload_data[1]; // Example data tweak for warning

        if (esb_write_payload(&tx_payload) == 0)
        {
            LOG_INF("TX -> D+: %d, D-: %d", payload_data[0], payload_data[1]);
        }
        else
        {
            LOG_INF("ESB write payload failed");
        }
    }
}
/* New Function for Blinking LED */
void blink_led(enum led_color color, int duration_ms, int blink_count)
{
    for (int i = 0; i < blink_count; i++)
    {
        set_led_status(color); // Turn LED on
        k_sleep(K_MSEC(duration_ms));
        set_led_status(LED_OFF); // Turn LED off
        k_sleep(K_MSEC(duration_ms));
    }
}

int main(void)
{
    int err;
    LOG_INF("Starting 2.4GHz HID Keyboard Transmitter");
    /* Initialize USB HID */
    hid_dev = device_get_binding("HID_0");
    LOG_INF("USB device found and ready.");

    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), &my_hid_ops);
    LOG_INF("HID registered.");

    err = usb_hid_init(hid_dev);
    if (err)
    {
        LOG_ERR("Failed to init USB HID, err %d", err);
        return 0;
    }
    LOG_INF("HID initialized.");

    LOG_INF("Attempting to enable USB...");
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK))
    {
        usb_dc_set_status_callback(&usb_status_cb);
        err = usb_enable(&usb_status_cb);
        if (err)
        {
            LOG_ERR("Failed to enable USB, err %d", err);
            return 0;
        }
        LOG_INF("USB enabled successfully.");
    }

    /* Initialize ESB */
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

    /* --- Initialize GPIOs --- */
    if (!device_is_ready(data_plus.port) || !device_is_ready(data_minus.port))
    {
        LOG_ERR("Data GPIOs not ready");
    }
    gpio_pin_configure_dt(&data_plus, GPIO_INPUT);
    gpio_pin_configure_dt(&data_minus, GPIO_INPUT);

    /* --- Initialize RGB LED --- */
    if (!device_is_ready(red_led.port) || !device_is_ready(green_led.port) || !device_is_ready(blue_led.port))
    {
        LOG_ERR("RGB LED GPIOs not ready");
    }

    /* --- Power-On LED Status --- */
    blink_led(LED_GREEN, 500, 5);
    // set_led_status(LED_GREEN); // Initial green for 5 seconds
    k_timer_start(&led_timer, K_SECONDS(5), K_NO_WAIT);
    check_battery_status(); // Update LED based on battery level

    /* --- Configure GPIO Interrupt for Wake-up --- */
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
    err = esb_start_tx();
    if (err)
    {
        LOG_ERR("ESB start_tx failed: %d", err);
    }

    /* Modified: Blink blue LED to indicate ESB connect attempt */
    LOG_INF("Blinking blue LED to indicate ESB connect attempt...");
    blink_led(LED_BLUE, 500, 5); // Blink 5 times, 0.5s on/off
    // k_timer_start(&led_timer, K_SECONDS(4), K_NO_WAIT);
   

    LOG_INF("Transmitter initialized successfully");

    /* --- Thread Exits, Idle Thread Takes Over --- */
    return 0;
}