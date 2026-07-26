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
#include <zephyr/drivers/pwm.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

LOG_MODULE_REGISTER(transmitter, LOG_LEVEL_INF);
enum led_color
{
    LED_RED,
    LED_GREEN,
    LED_GREEN_OFF, // New state for green LED off
    LED_BLUE,
    LED_YELLOW,
    LED_ORANGE,
    LED_WHITE,
    LED_OFF
};
enum pwm_clock {
    PWM_CLOCK_16MHZ = 0,
    PWM_CLOCK_8MHZ,
    PWM_CLOCK_4MHZ,
    PWM_CLOCK_2MHZ,
    PWM_CLOCK_1MHZ,
    PWM_CLOCK_500KHZ,
    PWM_CLOCK_250KHZ,
    PWM_CLOCK_125KHZ
};

// Packet types
enum packet_type {
    PACKET_TYPE_BIND_REQUEST = 0x01,
    PACKET_TYPE_BIND_ACK = 0x04,
    PACKET_TYPE_BATTERY_VOLTAGE = 0x06
};

void sample_and_transmit(struct esb_evt const *event);
void enter_low_power(struct k_timer *dummy);
void update_sleep_timeout(void);
void led_timer_handler(struct k_timer *timer);
int check_battery_status(void);
void send_battery_level(uint32_t voltage);
void battery_warning_timer_handler(struct k_timer *timer);
void battery_check_timer_handler(struct k_timer *timer);
void battery_pulse_timer_handler(struct k_timer *timer);
void pulse_timer_handler(struct k_timer *timer);
static void charging_pulse_timer_handler(struct k_timer *timer);
void post_init_callback(struct k_timer *timer);
K_TIMER_DEFINE(post_init_timer, post_init_callback, NULL);
static bool initialization_complete = false;
static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param);
static const struct device *hid_dev;

/* --- LED and Timer Definitions --- */
#define MIN_PERIOD PWM_SEC(1U) / 128U // Minimum period (1/128th of a second ~ 0.0078125s)
#define MAX_PERIOD PWM_MSEC(50U)    // Start with 0.05 seconds (50,000,000 ns) to reduce    // Maximum period (1 second)
static uint32_t max_period = MAX_PERIOD;
static volatile bool pulsing = false;
static enum led_color pulse_color = LED_OFF;
static uint8_t pulse_step = 0;
static const uint8_t max_steps = 20; // Number of fade steps
static const uint32_t step_delay = 50; // 50ms per step for smooth fade

/* --- Pin Definitions from Device Tree --- */
/* static const struct pwm_dt_spec data_plus = GPIO_DT_SPEC_GET(DT_N_NODELABEL_data_plus_pin,gpios);
static const struct gpio_dt_spec data_minus = GPIO_DT_SPEC_GET(DT_N_NODELABEL_data_minus_pin,gpios); */
static const struct adc_dt_spec adc = ADC_DT_SPEC_GET(DT_PATH(adc_input));

/* --- LED Definitions --- */
static const struct pwm_dt_spec pwm_red = PWM_DT_SPEC_GET(DT_ALIAS(red_pwm_led));
static const struct pwm_dt_spec pwm_green = PWM_DT_SPEC_GET(DT_ALIAS(green_pwm_led));
static const struct pwm_dt_spec pwm_blue = PWM_DT_SPEC_GET(DT_ALIAS(blue_pwm_led));


/* void set_led_status(enum led_color color) {
    switch (color) {
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
            gpio_pin_set_dt(&green_led, 1); // Simplified orange as Red + Green
            gpio_pin_set_dt(&blue_led, 0);
            break;
        case LED_OFF:
            gpio_pin_set_dt(&red_led, 0);
            gpio_pin_set_dt(&green_led, 0);
            gpio_pin_set_dt(&blue_led, 0);
            break;
        default:
            // Handle unexpected enum values by turning off all LEDs
            gpio_pin_set_dt(&red_led, 0);
            gpio_pin_set_dt(&green_led, 0);
            gpio_pin_set_dt(&blue_led, 0);
            break;
    }
} */

void set_led_status(enum led_color color) {
    if (!device_is_ready(pwm_blue.dev)) {
        LOG_ERR("PWM device not ready");
        return;
    }

    uint32_t red_pulse = max_period;   // 0% duty (off)
    uint32_t green_pulse = max_period; // 0% duty (off)
    uint32_t blue_pulse = max_period;  // 0% duty (off)

    switch (color) {
        case LED_OFF:
            red_pulse = max_period;   // 100% duty (on)
            green_pulse = max_period; // 100% duty (on)
            blue_pulse = max_period;  // 100% duty (on)
            LOG_INF("Setting OFF: red=%d, green=%d, blue=%d", red_pulse, green_pulse, blue_pulse);
            break;
        case LED_RED:
            red_pulse = 0; // 100% duty (on)
            LOG_INF("Setting RED: red=%d, green=%d, blue=%d", red_pulse, green_pulse, blue_pulse);
            break;
        case LED_GREEN:
            green_pulse = 0; // 100% duty (on)
            LOG_INF("Setting GREEN: red=%d, green=%d, blue=%d", red_pulse, green_pulse, blue_pulse);
            break;
        case LED_BLUE:
            blue_pulse = 0; // 100% duty (on)
            LOG_INF("Setting BLUE: red=%d, green=%d, blue=%d", red_pulse, green_pulse, blue_pulse);
            break;
        case LED_YELLOW:
            red_pulse = 0;   // 100% duty (on)
            green_pulse = 0; // 100% duty (on)
            LOG_INF("Setting YELLOW: red=%d, green=%d, blue=%d", red_pulse, green_pulse, blue_pulse);
            break;
        case LED_ORANGE:
            red_pulse = 0;      // 100% duty (on)
            green_pulse = max_period / 2; // 50% duty (dimmed on)
            LOG_INF("Setting ORANGE: red=%d, green=%d, blue=%d", red_pulse, green_pulse, blue_pulse);
            break;
        case LED_WHITE:
            red_pulse = 0;   // 100% duty (on)
            green_pulse = 0; // 100% duty (on)
            blue_pulse = 0;  // 100% duty (on)
            LOG_INF("Setting WHITE: red=%d, green=%d, blue=%d", red_pulse, green_pulse, blue_pulse);
            break;
        default:
            LOG_INF("Setting OFF: red=%d, green=%d, blue=%d", red_pulse, green_pulse, blue_pulse);
            break;
    }

    // Apply PWM settings with debug
    LOG_INF("Applying PWM: red=%lu, green=%lu, blue=%lu, period=%lu", red_pulse, green_pulse, blue_pulse, max_period);
    pwm_set_dt(&pwm_red, max_period, red_pulse);
    pwm_set_dt(&pwm_green, max_period, green_pulse);
    pwm_set_dt(&pwm_blue, max_period, blue_pulse);
}

static K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
static K_TIMER_DEFINE(inactivity_timer, NULL, enter_low_power);
static K_TIMER_DEFINE(battery_warning_timer, battery_warning_timer_handler, NULL);
static K_TIMER_DEFINE(battery_pulse_timer, battery_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(pulse_timer, pulse_timer_handler, NULL); // Timer for pulsing
static K_TIMER_DEFINE(charging_pulse_timer, charging_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(battery_check_timer, battery_check_timer_handler, NULL);

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
#define BATTERY_MIN_VOLTAGE 3430
#define CHARGING_THRESHOLD_DELTA 50
#define FULL_CHARGE_THRESHOLD 4190
static bool battery_warning_active = false;
static uint32_t pulse_count = 0;
static uint32_t battery_level = 0;
static int64_t last_voltage_time = 0;
static uint32_t last_voltage_mv = 0;
static uint32_t max_voltage_mv = 0;
static bool is_charging = false;

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

/* --- INIT PERIPHERALS --- */
void init_peripherals(void)
{
   
} 
void pulse_led(enum led_color color, uint32_t duration_ms, uint8_t pulse_count) {
    if (pulsing) return; // Avoid overlapping pulses
    LOG_INF("Starting pulse: color=%d, duration=%d ms, count=%d", color, duration_ms, pulse_count);
    pulsing = true;
    pulse_color = color;
    pulse_step = 0;
    k_timer_start(&pulse_timer, K_NO_WAIT, K_MSEC(step_delay));
    // Total duration = (max_steps * 2 * step_delay) * pulse_count
}
void pulse_timer_handler(struct k_timer *timer) {
//LOG_INF("Pulse timer handler called, pulsing=%d, step=%d, color=%d", pulsing, pulse_step, pulse_color);
    uint32_t red_pulse = max_period, green_pulse = max_period, blue_pulse = max_period; // Default to off (0% duty)
    if (pulse_step < max_steps) {
        // Fade in: Decrease duty cycle from max_period (full on) to 0 (off)
        uint32_t duty = pulse_step * (max_period / max_steps);
        switch (pulse_color) {
            case LED_RED: red_pulse = duty; break;
            case LED_GREEN: green_pulse = duty; break;
            case LED_BLUE: blue_pulse = duty; break;
            case LED_YELLOW:
                red_pulse = duty;
                green_pulse = duty;
                break;
            case LED_ORANGE:
                red_pulse = duty;
                green_pulse = duty / 2; // 50% dimming
                break;
            case LED_WHITE:
                red_pulse = duty;
                green_pulse = duty;
                blue_pulse = duty;
                break;
            default: break;
        }
        //LOG_INF("Pulse fade in, step %d, color=%d", pulse_step, pulse_color);
    } else if (pulse_step < max_steps * 2) {
        // Fade out: Increase duty cycle from 0 (off) to max_period (full on)
        uint32_t duty = max_period - ((pulse_step - max_steps) * (max_period / max_steps));
        switch (pulse_color) {
            case LED_RED: red_pulse = duty; break;
            case LED_GREEN: green_pulse = duty; break;
            case LED_BLUE: blue_pulse = duty; break;
            case LED_YELLOW:
                red_pulse = duty;
                green_pulse = duty;
                break;
            case LED_ORANGE:
                red_pulse = duty;
                green_pulse = duty / 2;
                break;
            case LED_WHITE:
                red_pulse = duty;
                green_pulse = duty;
                blue_pulse = duty;
                break;
            default: break;
        }
        //LOG_INF("Pulse fade out, step %d, color=%d", pulse_step, pulse_color);
    } else {
        // End of one pulse cycle
        pulsing = false;
        pulse_step = 0;
        pwm_set_dt(&pwm_red, max_period, max_period); // Ensure off
        pwm_set_dt(&pwm_green, max_period, max_period);
        pwm_set_dt(&pwm_blue, max_period, max_period);
        k_timer_stop(&pulse_timer);
        LOG_INF("Pulse cycle complete");
        return;
    }

    pwm_set_dt(&pwm_red, max_period, red_pulse);
    pwm_set_dt(&pwm_green, max_period, green_pulse);
    pwm_set_dt(&pwm_blue, max_period, blue_pulse);
    pulse_step++;
    k_timer_start(&pulse_timer, K_MSEC(step_delay), K_NO_WAIT);
}

void led_timer_handler(struct k_timer *timer)
{
    if (!is_charging){
        pulse_led(LED_RED, 1000, 1);
    }
}

void battery_warning_timer_handler(struct k_timer *timer)
{
    if (!is_charging && battery_warning_active){
        k_timer_start(&battery_pulse_timer, K_NO_WAIT, K_NO_WAIT);
    }
}

void battery_pulse_timer_handler(struct k_timer *timer) {
    if (pulse_count < 8 && !is_charging) {
        pulse_led(LED_RED, 1000, 1); // Fade in/out over 1s
        pulse_count++;
    } else {
        pulse_count = 0;
        k_timer_stop(&battery_pulse_timer);
        if (battery_warning_active && !is_charging) {
            k_timer_start(&battery_warning_timer, K_MINUTES(5), K_NO_WAIT);
        }
    }
}

static void charging_pulse_timer_handler(struct k_timer *timer) {
    if (is_charging) {
        pulse_led(LED_GREEN, 2000, 1); // 2s fade cycle
        k_timer_start(&charging_pulse_timer, K_MSEC(2000), K_NO_WAIT);
    } else {
        pulse_led(LED_RED, 1500, 3); // 3 pulses of 1.5s each
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
    LOG_INF("Entering System OFF mode");
    k_timer_stop(&led_timer);
    k_timer_stop(&battery_warning_timer);
    k_timer_stop(&battery_pulse_timer);
    k_timer_stop(&charging_pulse_timer);
    k_timer_stop(&battery_check_timer);
    esb_disable();
    pulse_led(LED_RED, 1500, 3); // Replace blink
    set_led_status(LED_OFF);
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

void pin_activity_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    update_sleep_timeout();
    k_timer_start(&inactivity_timer, K_NO_WAIT, current_timeout);
    set_led_status(LED_BLUE);
    k_timer_start(&led_timer, K_SECONDS(4), K_NO_WAIT);
    esb_start_tx();
    LOG_INF("Pin activity detected, resetting inactivity timer");
}
/* // GPIO interrupt handler
void pin_activity_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (pins & BIT(data_plus.pin)) {
        LOG_INF("Pin activity detected P%d on DATA_PLUS, resetting inactivity timer", gpio_port_get_raw(data_plus.port, &pins));
        pulse_led(LED_BLUE, 100, 1); // Indicate activity
    }
    if (pins & BIT(data_minus.pin)) {
        LOG_INF("Pin activity detected P%d on DATA_MINUS, resetting inactivity timer", gpio_port_get_raw(data_minus.port, &pins));
        pulse_led(LED_BLUE, 100, 1); // Indicate activity
    }
    update_sleep_timeout();
    k_timer_start(&led_timer, K_SECONDS(4), K_NO_WAIT);
    if (esb_start_tx() == 0) {
        pulse_led(LED_GREEN, 100, 1); // Success
    } else {
        pulse_led(LED_RED, 100, 3); // Failure
    }
} */

int check_battery_status(void) {
    int16_t buf;
    int battery_led_color = LED_OFF;
    struct adc_sequence sequence = {
        .channels = adc.channel_id,
        .buffer = &buf,
        .buffer_size = sizeof(buf),
        .oversampling = adc.oversampling,
        .resolution = adc.resolution,
    };
        if (!device_is_ready(adc.dev)) {
            LOG_ERR("ADC device not ready: %s", adc.dev->name);
            return 0;
        }

        int err = adc_channel_setup_dt(&adc);
        if (err < 0) {
            LOG_ERR("ADC channel setup failed (ID: %u, err: %d)", adc.channel_id, err);
            return 0;
        }
        err = adc_sequence_init_dt(&adc, &sequence);
        if (err < 0) {
            LOG_ERR("ADC sequence init failed (channel ID: %u, resolution: %u, err: %d)",
                    adc.channel_id, sequence.resolution, err);
            return 0;
        }
        err = adc_read(adc.dev, &sequence);
        if (err < 0) {
            LOG_ERR("ADC read failed (channel ID: %u, buffer size: %u, err: %d)",
                    adc.channel_id, sequence.buffer_size, err);
            return 0;
        }

    uint32_t adc_value = buf;
    LOG_INF("ADC READ VALUE: %d", adc_value);
    uint32_t voltage_mv = (adc_value * 1.45);
    LOG_INF("VOLTAGE mV: %d mV", voltage_mv);
    battery_level = (voltage_mv - BATTERY_MIN_VOLTAGE) * 100 / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);
    if (battery_level < 0) battery_level = 0;
    if (battery_level > 100) battery_level = 100;

    LOG_INF("Battery level: %d%%, Voltage: %d mV", battery_level, voltage_mv);

        // Set LED color based on battery level
    if (battery_level >= 75 && battery_level <= 100) {
        battery_led_color = LED_GREEN;
    } else if (battery_level >= 50 && battery_level < 75) {
        battery_led_color = LED_YELLOW;
    } else if (battery_level >= 25 && battery_level < 50) {
        battery_led_color = LED_ORANGE;
    } else if (battery_level >= 0 && battery_level < 25) {
        battery_led_color = LED_RED;
    } else {
        battery_led_color = LED_OFF; // Or a warning color like LED_ORANGE
    }
    set_led_status(battery_led_color);

    /// First 5 seconds override logic
    if (k_uptime_get() < Z_TIMEOUT_MS_TICKS(5000)) {
        LOG_INF("System just started, setting battery LED color based on level %d%%", battery_level);
        set_led_status(battery_led_color); // Use the determined color
        k_timer_start(&led_timer, K_SECONDS(5), K_NO_WAIT);
        set_led_status(LED_OFF); // Turn off after 5 seconds
    }

    // Detect charging or full battery
    int64_t now_ms = k_uptime_get();
    if (now_ms - last_voltage_time >= 1000) {
        if (last_voltage_mv > 0) {
            if ((voltage_mv - last_voltage_mv) > CHARGING_THRESHOLD_DELTA) {
                is_charging = true;
                k_timer_start(&charging_pulse_timer, K_MSEC(500), K_NO_WAIT);
                battery_warning_active = false;
                k_timer_stop(&battery_warning_timer);
                k_timer_stop(&battery_pulse_timer);
                LOG_INF("Charging detected, starting green led");
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

    // Low battery alert
    if (battery_level < BATTERY_CRITICAL_THRESHOLD && !is_charging && !battery_warning_active) {
        battery_warning_active = true;
        k_timer_start(&battery_warning_timer, K_NO_WAIT, K_NO_WAIT);
    }

    // Send via ESB
    //send_battery_level(voltage_mv);
    return voltage_mv;
}
void battery_check_timer_handler(struct k_timer *dummy) {
   check_battery_status();
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
        pulse_led(LED_GREEN, 200, 2); // Replace blink
    } else {
        pulse_led(LED_RED, 300, 3); // Replace blink
    }
    set_led_status(LED_OFF);
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
        pulse_led(LED_GREEN, 200, 3);
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

void post_init_callback(struct k_timer *timer)
{
    if (!initialization_complete) {
        LOG_WRN("Post-init attempted before peripherals ready. Retrying...");
        k_timer_start(&post_init_timer, K_MSEC(100), K_NO_WAIT);
        return;
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
        return 0;
    }
    LOG_INF("HID initialized.");

    LOG_INF("Attempting to enable USB...");
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK))
    {
        err = usb_enable(NULL);
        if (err)
        {
            LOG_ERR("Failed to enable USB, err %d", err);
            return 0;
        }
        LOG_INF("USB enabled successfully.");
    }
    esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
    esb_config.mode = ESB_MODE_PTX;
    esb_config.bitrate = ESB_BITRATE_2MBPS;
    esb_config.payload_length = 8;
    esb_config.retransmit_count = 5;
    esb_config.retransmit_delay = 100;
    esb_config.tx_output_power = ESB_TX_POWER_6DBM;
    esb_config.event_handler = sample_and_transmit;
    err = esb_init(&esb_config);
    if (err)
    {LOG_ERR("ESB initialization failed, err %d", err);}
    uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
    err = esb_set_base_address_0(base_addr_0);
    if (err)
    {LOG_ERR("Failed to set base address 0");}
    uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
    err = esb_set_prefixes(prefixes, 8);
    if (err)
    {LOG_ERR("Failed to set prefixes");}
    LOG_INF("Transmitter initialized successfully");

    /* // Initialize GPIOs 
    if (!device_is_ready(data_plus.port) || !device_is_ready(data_minus.port))
    {LOG_ERR("Data GPIOs not ready");}
    else {LOG_INF("Data GPIOs ready: D+ pin %d, D- pin %d", data_plus.pin, data_minus.pin);}

    gpio_pin_configure_dt(&data_plus, GPIO_INPUT);
    gpio_pin_configure_dt(&data_minus, GPIO_INPUT | GPIO_PULL_UP);

    * //Initialize RGB LED 
    if (!device_is_ready(red_led.port) || !device_is_ready(green_led.port) || !device_is_ready(blue_led.port))
    {LOG_ERR("RGB LED GPIOs not ready");}
    else { LOG_INF("RGB LED GPIOs ready: Red pin %d, Green pin %d, Blue pin P1.0%d", red_led.pin, green_led.pin, blue_led.pin);}

    gpio_pin_configure_dt(&red_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&green_led, GPIO_OUTPUT);
    gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT);
 */// PWM Calibration
   // PWM Calibration
    if (!device_is_ready(pwm_red.dev)) {
        LOG_ERR("PWM device not ready");
        return 0;
    }
    LOG_INF("Calibrating PWM for RGB LEDs...");
    max_period = MAX_PERIOD;
    while (pwm_set_dt(&pwm_red, max_period, max_period / 2U)) {
        max_period /= 2U;
        if (max_period < (4U * MIN_PERIOD)) {
            LOG_ERR("Error: PWM device does not support a period at least %lu nsec", 4U * MIN_PERIOD);
            return 0;
        }
    }
    LOG_INF("Done calibrating; maximum/minimum periods %u/%lu nsec", max_period, MIN_PERIOD);
    uint32_t red_duty = 0;   // 0% duty (off)
    uint32_t green_duty = 0; // 0% duty (off)
    uint32_t blue_duty = 0;  // 0% duty (off)
    // Initialize all PWM channels to off
    pwm_set_dt(&pwm_red, max_period, red_duty);
    pwm_set_dt(&pwm_green, max_period, green_duty);
    pwm_set_dt(&pwm_blue, max_period, blue_duty);
    LOG_INF("ALL LEDS INITIALIZED OFF RED: %d, GREEN: %d, BLUE: %d (0 duty cycle)", red_duty, green_duty, blue_duty);
    //k_timer_start(&led_timer, K_SECONDS(5), K_NO_WAIT);
    /* Configure GPIO Interrupt for Wake-up */
    static struct gpio_callback pin_cb_data;
    //gpio_init_callback(&pin_cb_data, pin_activity_handler, BIT(data_plus.pin) | BIT(data_minus.pin));
    /* err = gpio_add_callback(data_plus.port, &pin_cb_data);
    if (err)
    {LOG_ERR("Failed to add callback, err %d", err);} */

 /*    gpio_pin_interrupt_configure_dt(&data_plus, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&data_minus, GPIO_INT_EDGE_BOTH); */

    if (!esb_is_idle()) {
        LOG_WRN("ESB not idle, flushing buffers");
        esb_flush_tx();
        esb_flush_rx();
    }
    err = esb_start_tx();
    if (err)
    {LOG_ERR("ESB start_tx failed: %d", err);}
    LOG_INF("ESB started, waiting for data...");
    LOG_INF("Pulsing blue LED to indicate ESB connect attempt...");
    //pulse_led(LED_BLUE, 250, 3);

    //check_battery_status();
    //pulse_led(LED_OFF, 2000, 8);
    LOG_INF("ALL LEDS SHOULD BE OFF (0 duty cycle)");
    set_led_status(LED_OFF);
    k_sleep(K_SECONDS(1));
    LOG_INF("GREEN LED should be ON (100 duty cycle)");
    set_led_status(LED_GREEN);
    k_sleep(K_SECONDS(1));
    LOG_INF("GREEN LED should be OFF (0 duty cycle)");
    set_led_status(LED_OFF);
    set_led_status(LED_BLUE);
    LOG_INF("GREEN LED should pulse ON for 1 second");
    pulse_led(LED_GREEN, 1000, 3);
    k_sleep(K_SECONDS(2));
    
    //for(int i = 0; i < 20; i++) {check_battery_status(); k_sleep(K_SECONDS(1));}
    //k_timer_start(&battery_check_timer, K_MINUTES(1), K_MINUTES(1));

    /* Send Windows key sequence (for testing) */
    //send_windows_key();
    //k_sleep(K_MSEC(200)); // Delay to ensure key is registered
    //LOG_INF("Windows key sequence sent");

    return 0;
}