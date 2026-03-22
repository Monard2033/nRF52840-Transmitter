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
#include <string.h>
#include <nrfx.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/types.h>
#include <zephyr/irq.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

/* --- Configuration Defines --- */
#define BATTERY_CRITICAL_THRESHOLD 15
#define BATTERY_MAX_VOLTAGE 4200
#define BATTERY_MIN_VOLTAGE 3430
#define CHARGING_THRESHOLD_DELTA 150
#define FULL_CHARGE_THRESHOLD 4190
#define BASE_TIMEOUT K_SECONDS(60)
#define GRACE_PERIOD_MS 2000
#define REPORT_ID 1
#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_MSEC(5U)
#define SPI_BUFFER_SIZE 8
#define ESB_START_TX_RETRIES 3
#define ESB_START_TX_DELAY_MS 10

LOG_MODULE_REGISTER(TRANSMITTER, LOG_LEVEL_INF);

/* --- Enumerations --- */
enum led_color {
    LED_RED,
    LED_GREEN,
    LED_GREEN_OFF,
    LED_BLUE,
    LED_YELLOW,
    LED_ORANGE,
    LED_MAGENTA,
    LED_WHITE,
    LED_OFF
};

enum packet_type {
    PACKET_TYPE_BIND_REQUEST = 0x01,
    PACKET_TYPE_BIND_ACK = 0x04,
    PACKET_TYPE_BATTERY_VOLTAGE = 0x06
};

/* --- Global Variables --- */
static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi3));
static const struct gpio_dt_spec wake_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(spi3), wake_gpios);
static const struct gpio_dt_spec cs_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(spi3), cs_gpios);
static struct spi_config spi_cfg = {
    .frequency = 32000000,
    .operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
    .slave = 0,
};
static uint8_t spi_rx_buf[SPI_BUFFER_SIZE];
static struct esb_payload tx_payload;
static struct esb_config esb_config = ESB_DEFAULT_CONFIG;
static struct gpio_callback wake_pin_cb_data;
static struct gpio_callback cs_pin_cb_data;

static const struct adc_dt_spec adc = ADC_DT_SPEC_GET(DT_NODELABEL(vbatt));
static const struct pwm_dt_spec pwm_red = PWM_DT_SPEC_GET(DT_NODELABEL(red_pwm));
static const struct pwm_dt_spec pwm_green = PWM_DT_SPEC_GET(DT_NODELABEL(green_pwm));
static const struct pwm_dt_spec pwm_blue = PWM_DT_SPEC_GET(DT_NODELABEL(blue_pwm));

static uint32_t max_period = MAX_PERIOD;
static volatile bool pulsing = false;
static enum led_color pulse_color = LED_OFF;
static uint32_t pulse_duration_ms;
static uint32_t total_steps;
static uint8_t current_step;
static uint32_t total_execution_ms;
static uint8_t remaining_pulses;
static const uint32_t step_delay = 50;
static const uint32_t grace_period_ms = 500;

static struct {
    enum led_color color;
    uint32_t duration_ms;
    uint8_t count;
    bool active;
} pulse_queue[7];
static uint8_t queue_index = 0;

static volatile bool usb_configured = false;
static K_SEM_DEFINE(usb_configured_sem, 0, 1);
static k_timeout_t current_timeout = BASE_TIMEOUT;
static int64_t last_sleep_time = 0;
static int wakeup_count = 0;
static bool battery_warning_active = false;
static uint32_t battery_level = 0;
static int64_t last_voltage_time = 0;
static uint32_t last_voltage_mv = 0;
static uint32_t max_voltage_mv = 0;
static bool is_charging = false;
static uint32_t pulses = 0;
static volatile bool spi_activity_detected = false;
static const struct device *hid_dev;

/* Workqueue for battery status */
static struct k_work battery_work;

/* --- Function Prototypes --- */
void initialize_usb_hid(void);
void initialize_clock(void);
void initialize_peripherals(void);
void display_and_transmit_battery(struct k_timer *timer);
void battery_work_handler(struct k_work *work);
void wake_pin_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void cs_pin_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void handle_wakeup(void);
void enter_low_power(struct k_timer *timer);
void process_spi_activity(struct esb_evt const *event);
void update_sleep_timeout(void);
void send_windows_key(void);
void set_led_status(enum led_color color);
void pulse_led(enum led_color color, uint32_t duration_ms, uint8_t pulse_count);
void pulse_timer_handler(struct k_timer *timer);
int check_battery_status(void);
void send_battery_level(uint32_t voltage);
void detect_charging_status(uint32_t voltage_mv);
void battery_check_timer_handler(struct k_timer *timer);
void battery_warning_timer_handler(struct k_timer *timer);
void battery_pulse_timer_handler(struct k_timer *timer);
void charging_pulse_timer_handler(struct k_timer *timer);
static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param);

/* --- Timers --- */
static K_TIMER_DEFINE(pulse_timer, pulse_timer_handler, NULL);
static K_TIMER_DEFINE(inactivity_timer, enter_low_power, NULL);
static K_TIMER_DEFINE(charging_pulse_timer, charging_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(battery_warning_timer, battery_warning_timer_handler, NULL);
static K_TIMER_DEFINE(battery_pulse_timer, battery_pulse_timer_handler, NULL);
static K_TIMER_DEFINE(battery_display_timer, display_and_transmit_battery, NULL);

/* HID Report Descriptor */
static const uint8_t hid_report_desc[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x75, 0x01,
    0x95, 0x08, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
    0x15, 0x00, 0x25, 0x01, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 0x75, 0x08,
    0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00,
    0x29, 0x65, 0x81, 0x00, 0xC0
};

/* --- Function Definitions --- */

void battery_work_handler(struct k_work *work) {
    uint32_t voltage_mv = check_battery_status();
    if (voltage_mv == 0) {
        LOG_ERR("Failed to read battery status, skipping LED and transmission");}
    enum led_color battery_led_color = LED_OFF;
    if (battery_level >= 75) battery_led_color = LED_GREEN;
    else if (battery_level >= 50) battery_led_color = LED_YELLOW;
    else if (battery_level >= 25) battery_led_color = LED_ORANGE;
    else if (battery_level > 0) battery_led_color = LED_RED;
    pulse_led(battery_led_color, 3000, 1);
    send_battery_level(voltage_mv);
}

void display_and_transmit_battery(struct k_timer *timer) {
    LOG_INF("Displaying and transmitting battery status");
    k_work_submit(&battery_work);  // Offload to system workqueue
}

void wake_pin_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    spi_activity_detected = true;
    LOG_INF("Wake pin interrupt triggered (P0.06)");
    handle_wakeup();
}

void cs_pin_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    spi_activity_detected = true;
    LOG_INF("CS pin interrupt triggered (P0.08)");
    handle_wakeup();
}

void handle_wakeup(void) {
    LOG_INF("Waking up due to SPI activity");
    k_timer_start(&battery_display_timer, K_NO_WAIT, K_MINUTES(1));
    update_sleep_timeout();
    spi_activity_detected = false;
}

void enter_low_power(struct k_timer *timer) {
    LOG_INF("Inactivity detected. Preparing to enter System OFF mode.");

    int retry_count = 0;
    const int max_retries = 3;
    while (retry_count < max_retries && !esb_is_idle()) {
        LOG_INF("ESB is busy, waiting %d/%d...", retry_count + 1, max_retries);
        k_sleep(K_MSEC(20));
        retry_count++;
    }
    if (!esb_is_idle()) {
        LOG_INF("ESB busy after %d retries", max_retries);
    }

    k_timer_stop(&battery_display_timer);
    k_timer_stop(&battery_warning_timer);
    k_timer_stop(&battery_pulse_timer);
    k_timer_stop(&charging_pulse_timer);
    k_timer_stop(&pulse_timer);
    k_timer_stop(&inactivity_timer);

    esb_disable();
    spi_release(spi_dev, &spi_cfg);
    set_led_status(LED_OFF);
    LOG_INF("System OFF mode entered");
}

void process_spi_activity(struct esb_evt const *event) {
    static uint8_t last_payload_data[2] = {0};
    struct spi_buf rx_buf = { .buf = spi_rx_buf, .len = SPI_BUFFER_SIZE };
    struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    int err = spi_read(spi_dev, &spi_cfg, &rx);
    if (err == 0) {
        if (spi_rx_buf[0] != last_payload_data[0] || spi_rx_buf[1] != last_payload_data[1]) {
            tx_payload.length = 2;
            memcpy(tx_payload.data, spi_rx_buf, 2);
            last_payload_data[0] = spi_rx_buf[0];
            last_payload_data[1] = spi_rx_buf[1];
            if (esb_write_payload(&tx_payload) == 0) {
                LOG_INF("TX -> D+: %d, D-: %d", spi_rx_buf[0], spi_rx_buf[1]);
            } else {
                LOG_ERR("ESB write payload failed");
            }
            update_sleep_timeout();
        }
    } else {
        LOG_ERR("SPI read failed, err: %d", err);
    }
}

void update_sleep_timeout(void) {
    int64_t now_ms = k_uptime_get();
    k_ticks_t now_ticks = k_ms_to_ticks_ceil32(now_ms);
    k_ticks_t last_ticks = k_ms_to_ticks_ceil32(last_sleep_time);
    k_ticks_t grace_ticks = k_ms_to_ticks_ceil32(GRACE_PERIOD_MS);
    if (last_sleep_time != 0 && (now_ticks - last_ticks) < grace_ticks) {
        wakeup_count++;
        current_timeout = (wakeup_count == 1) ? K_SECONDS(30) :
                         (wakeup_count == 2) ? K_SECONDS(180) :
                         (wakeup_count >= 3) ? K_SECONDS(240) : K_SECONDS(300);
        LOG_INF("Quick wakeup detected, sleep timeout increased");
    } else {
        wakeup_count = 0;
        current_timeout = BASE_TIMEOUT;
        LOG_INF("Long inactivity, resetting sleep timeout to 60 seconds");
    }
    last_sleep_time = now_ms;
    k_timer_start(&inactivity_timer, K_NO_WAIT, current_timeout);
}

void send_windows_key(void) {
    uint8_t hid_report[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    tx_payload.length = 8;
    memcpy(tx_payload.data, hid_report, 8);
    if (esb_write_payload(&tx_payload) == 0) {
        LOG_INF("TX -> Windows key press sent");
        pulse_led(LED_GREEN, 600, 2);
    } else {
        LOG_ERR("Failed to send Windows key press, ESB error");
    }

    k_sleep(K_MSEC(100));
    memset(hid_report, 0, 8);
    tx_payload.length = 8;
    memcpy(tx_payload.data, hid_report, 8);
    if (esb_write_payload(&tx_payload) == 0) {
        LOG_INF("TX -> Windows key release sent");
    } else {
        LOG_ERR("Failed to send Windows key release, ESB error");
    }
}

void set_led_status(enum led_color color) {
    uint32_t red_pulse = 0, green_pulse = 0, blue_pulse = 0;
    switch (color) {
        case LED_OFF: break;
        case LED_RED: red_pulse = max_period; break;
        case LED_GREEN: green_pulse = max_period; break;
        case LED_BLUE: blue_pulse = max_period; break;
        case LED_YELLOW: red_pulse = max_period; green_pulse = max_period; break;
        case LED_ORANGE: red_pulse = max_period; green_pulse = max_period / 2; break;
        case LED_MAGENTA: red_pulse = max_period; blue_pulse = max_period; break;
        case LED_WHITE: red_pulse = max_period; green_pulse = max_period; blue_pulse = max_period; break;
        default: break;
    }
    pwm_set_dt(&pwm_red, max_period, red_pulse);
    pwm_set_dt(&pwm_green, max_period, green_pulse);
    pwm_set_dt(&pwm_blue, max_period, blue_pulse);
}

void pulse_led(enum led_color color, uint32_t duration_ms, uint8_t pulse_count) {
    if (pulsing && queue_index >= 7 && pulse_count >= 1) return;
    if (pulse_count == 0) {
        if (pulsing && pulse_color == LED_OFF) return;
        pulsing = true;
        pulse_color = color;
        pulse_duration_ms = duration_ms;
        pulse_count = 0;
        current_step = 0;
        total_steps = (pulse_duration_ms + (2 * step_delay - 1)) / (2 * step_delay);
        if (total_steps < 10) total_steps = 10;
        k_timer_start(&pulse_timer, K_NO_WAIT, K_MSEC(step_delay));
        return;
    }

    pulse_queue[queue_index].color = color;
    pulse_queue[queue_index].duration_ms = duration_ms;
    pulse_queue[queue_index].count = pulse_count;
    pulse_queue[queue_index].active = false;
    queue_index++;

    if (!pulsing) {
        pulsing = true;
        pulse_color = pulse_queue[0].color;
        pulse_duration_ms = pulse_queue[0].duration_ms;
        pulses = pulse_queue[0].count;
        remaining_pulses = pulse_count;
        current_step = 0;
        total_steps = (pulse_duration_ms + (2 * step_delay - 1)) / (2 * step_delay);
        if (total_steps < 10) total_steps = 10;
        total_execution_ms = pulse_duration_ms * pulses + grace_period_ms;
        pulse_queue[0].active = true;
        k_timer_start(&pulse_timer, K_NO_WAIT, K_MSEC(step_delay));
    }
}

void pulse_timer_handler(struct k_timer *timer) {
    if (!pulsing) {
        k_timer_stop(&pulse_timer);
        return;
    }

    uint32_t red_pulse = 0, green_pulse = 0, blue_pulse = 0;
    uint32_t total_pulse_steps = total_steps * 2;
    uint32_t current_pulse = current_step / total_pulse_steps;
    uint32_t step_in_pulse = current_step % total_pulse_steps;
    bool is_infinite_pulse = (pulses == 0);

    if (is_infinite_pulse || (!is_infinite_pulse && current_pulse < pulses)) {
        if (step_in_pulse < total_steps) {
            uint32_t duty = (step_in_pulse * max_period) / total_steps;
            switch (pulse_color) {
                case LED_RED: red_pulse = duty; break;
                case LED_GREEN: green_pulse = duty; break;
                case LED_BLUE: blue_pulse = duty; break;
                case LED_YELLOW: red_pulse = duty; green_pulse = duty; break;
                case LED_ORANGE: red_pulse = duty; green_pulse = duty / 2; break;
                case LED_MAGENTA: red_pulse = duty; blue_pulse = duty; break;
                case LED_WHITE: red_pulse = duty; green_pulse = duty; blue_pulse = duty; break;
                case LED_OFF:
                    red_pulse = 0; green_pulse = 0; blue_pulse = 0;
                    if (is_infinite_pulse) {
                        pulsing = false;
                        current_step = 0;
                        k_timer_stop(&pulse_timer);
                        LOG_INF("Infinite pulse stopped by LED_OFF");
                        return;
                    }
                    break;
                default: break;
            }
        } else if (step_in_pulse < total_pulse_steps) {
            uint32_t duty = max_period - ((step_in_pulse - total_steps) * max_period) / total_steps;
            switch (pulse_color) {
                case LED_RED: red_pulse = duty; break;
                case LED_GREEN: green_pulse = duty; break;
                case LED_BLUE: blue_pulse = duty; break;
                case LED_YELLOW: red_pulse = duty; green_pulse = duty; break;
                case LED_ORANGE: red_pulse = duty; green_pulse = duty / 2; break;
                case LED_MAGENTA: red_pulse = duty; blue_pulse = duty; break;
                case LED_WHITE: red_pulse = duty; green_pulse = duty; blue_pulse = duty; break;
                case LED_OFF:
                    red_pulse = 0; green_pulse = 0; blue_pulse = 0;
                    if (is_infinite_pulse) {
                        pulsing = false;
                        current_step = 0;
                        k_timer_stop(&pulse_timer);
                        LOG_INF("Infinite pulse stopped by LED_OFF");
                        return;
                    }
                    break;
                default: break;
            }
        }
    } else if (!is_infinite_pulse && (uint32_t)(current_step - (pulses * total_pulse_steps)) * step_delay < grace_period_ms) {
        red_pulse = 0; green_pulse = 0; blue_pulse = 0;
    } else {
        pulsing = false;
        current_step = 0;
        pwm_set_dt(&pwm_red, max_period, 0);
        pwm_set_dt(&pwm_green, max_period, 0);
        pwm_set_dt(&pwm_blue, max_period, 0);
        k_timer_stop(&pulse_timer);
        if (queue_index > 0) {
            for (uint8_t i = 0; i < queue_index - 1; i++) {pulse_queue[i] = pulse_queue[i + 1];}
            queue_index--;
            if (queue_index > 0) {
                pulsing = true;
                pulse_color = pulse_queue[0].color;
                pulse_duration_ms = pulse_queue[0].duration_ms;
                pulses = pulse_queue[0].count;
                remaining_pulses = pulses;
                current_step = 0;
                total_steps = (pulse_duration_ms + (2 * step_delay - 1)) / (2 * step_delay);
                if (total_steps < 10) total_steps = 10;
                total_execution_ms = pulse_duration_ms * pulses + grace_period_ms;
                pulse_queue[0].active = true;
                k_timer_start(&pulse_timer, K_NO_WAIT, K_MSEC(step_delay));
            }
        }
        return;
    }
    pwm_set_dt(&pwm_red, max_period, red_pulse);
    pwm_set_dt(&pwm_green, max_period, green_pulse);
    pwm_set_dt(&pwm_blue, max_period, blue_pulse);
    current_step++;
    k_timer_start(&pulse_timer, K_MSEC(step_delay), K_NO_WAIT);
}

int check_battery_status(void) {
    int16_t buf = 0;  // Initialize to avoid using uninitialized data
    struct adc_sequence sequence = {
        .channels = adc.channel_id,
        .buffer = &buf,
        .buffer_size = sizeof(buf),
        .oversampling = adc.oversampling,
        .resolution = adc.resolution,
    };

    int err = adc_sequence_init_dt(&adc, &sequence);
    if (err < 0) {
        LOG_ERR("ADC sequence init failed (err: %d)", err);
        return 0;
    }

    err = adc_read(adc.dev, &sequence);
    if (err < 0) {
        LOG_ERR("ADC read failed (err: %d)", err);
        return 0;
    }

    LOG_INF("Initialized ADC sequence for channel %u with resolution %u and oversampling %u",
            adc.channel_id, sequence.resolution, sequence.oversampling);

    uint32_t adc_value = buf;
    LOG_INF("RAW ADC VALUE: %d ", adc_value);
    uint32_t voltage_mv = (adc_value * (3.3 / 1023)) / 0.6666666667;
    LOG_INF("Voltage = %d mV", voltage_mv);
    battery_level = (voltage_mv - BATTERY_MIN_VOLTAGE) * 100 / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);
    if (battery_level < 0) battery_level = 0;
    if (battery_level > 100) battery_level = 100;

    LOG_INF("Battery level: %d%%, Voltage: %d mV", battery_level, voltage_mv);

    if (battery_level < BATTERY_CRITICAL_THRESHOLD && !is_charging && !battery_warning_active) {
        battery_warning_active = true;
        k_timer_start(&battery_warning_timer, K_NO_WAIT, K_NO_WAIT);
        LOG_INF("Low battery warning activated");
    }
    detect_charging_status(voltage_mv);
    return voltage_mv;
}

void detect_charging_status(uint32_t voltage_mv) {
    int64_t now_ms = k_uptime_get();
    if (now_ms - last_voltage_time >= 1000) {
        if (last_voltage_mv > 0) {
            if ((voltage_mv - last_voltage_mv) > CHARGING_THRESHOLD_DELTA) {
                is_charging = true;
                k_timer_start(&charging_pulse_timer, K_MSEC(500), K_NO_WAIT);
                battery_warning_active = false;
                k_timer_stop(&battery_warning_timer);
                k_timer_stop(&battery_pulse_timer);
                LOG_INF("Charging state triggered: Voltage delta %d mV", voltage_mv - last_voltage_mv);
                pulse_led(LED_GREEN, 2500, 0);
            } else if (is_charging && voltage_mv >= FULL_CHARGE_THRESHOLD) {
                k_timer_stop(&charging_pulse_timer);
                //set_led_status(LED_GREEN);
                max_voltage_mv = voltage_mv;
                is_charging = false;
                LOG_INF("Full charge state triggered: %d mV", voltage_mv);
                pulse_led(LED_OFF, 0, 0);
            }
        }
        last_voltage_time = now_ms;
        last_voltage_mv = voltage_mv;
        if (voltage_mv > max_voltage_mv) {
            max_voltage_mv = voltage_mv;
        }
    }
}

void battery_warning_timer_handler(struct k_timer *timer) {
    if (!is_charging && battery_warning_active) {
        k_timer_start(&battery_pulse_timer, K_NO_WAIT, K_NO_WAIT);
    }
}

void battery_pulse_timer_handler(struct k_timer *timer) {
    if (pulses < 8 && !is_charging) {
        pulse_led(LED_RED, 1000, 3);
        pulses++;
    } else {
        pulses = 0;
        k_timer_stop(&battery_pulse_timer);
        if (battery_warning_active && !is_charging) {
            k_timer_start(&battery_warning_timer, K_MINUTES(5), K_NO_WAIT);
        }
    }
}

void charging_pulse_timer_handler(struct k_timer *timer) {
    if (is_charging) {
        pulse_led(LED_GREEN, 2000, 1);
        k_timer_start(&charging_pulse_timer, K_MSEC(2000), K_NO_WAIT);
    } else {
        pulse_led(LED_RED, 3000, 1);
        k_timer_stop(&charging_pulse_timer);
        set_led_status(LED_OFF);
    }
}

void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param) {
    ARG_UNUSED(param);
    switch (status) {
        case USB_DC_CONFIGURED:
            usb_configured = true;
            k_sem_give(&usb_configured_sem);
            break;
        case USB_DC_DISCONNECTED:
            usb_configured = false;
            break;
        default:
            break;
    }
}

void send_battery_level(uint32_t voltage) {
    if(esb_start_tx()==0) { LOG_INF("ESB TX started successfully"); }
    uint8_t payload_data[8] = {PACKET_TYPE_BATTERY_VOLTAGE};
    memcpy(payload_data + 1, &voltage, sizeof(voltage));
    tx_payload.length = sizeof(payload_data);
    memcpy(tx_payload.data, payload_data, sizeof(payload_data));

    int err = esb_write_payload(&tx_payload);
    if (err == 0) {
        LOG_INF("TX -> Voltage: %d mV", voltage);
        pulse_led(LED_GREEN, 200, 2);
    } else {
        pulse_led(LED_RED, 300, 2);
    }
    set_led_status(LED_OFF);
}

int main(void) {
    LOG_INF("Starting 2.4GHz HID Keyboard Transmitter");

    hid_dev = device_get_binding("HID_0");
    if (!hid_dev) { LOG_ERR("USB HID device not found"); }
    LOG_INF("USB device found and ready.");

    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), NULL);
    LOG_INF("HID registered.");

    int err = usb_hid_init(hid_dev);
    if (err) { LOG_ERR("Failed to init USB HID, err %d", err); }
    LOG_INF("HID initialized.");

    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        err = usb_enable(usb_status_cb);
        if (err) { LOG_ERR("Failed to enable USB, err %d", err); }
        LOG_INF("USB enabled successfully.");
    }

    // Initialize high-frequency clock
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
    k_sleep(K_MSEC(1)); // Ensure clock stabilizes
    LOG_INF("High-frequency clock started");

    // Ensure radio peripheral is powered
    nrf_radio_power_set(NRF_RADIO, true);
    if (NRF_RADIO->POWER == 1) {
        LOG_INF("Radio powered on"); }
    else {
    LOG_ERR("Radio power-on timeout exceeded, err %d", NRF_RADIO->POWER);
    }
     // Initialize ESB
    esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
    esb_config.mode = ESB_MODE_PTX;
    esb_config.bitrate = ESB_BITRATE_2MBPS;
    esb_config.use_fast_ramp_up = 1;
    esb_config.payload_length = 8;
    esb_config.retransmit_count = 5;
    esb_config.retransmit_delay = 100;
    esb_config.tx_output_power = ESB_TX_POWER_4DBM;
    esb_config.selective_auto_ack = true;
    esb_config.event_handler = process_spi_activity;
    err = esb_init(&esb_config);
    if (err) {
        LOG_ERR("ESB init failed, err %d", err);
    }
    uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
    err = esb_set_base_address_0(base_addr_0);
    if (err) {
        LOG_ERR("Failed to set base address 0");
    }

    uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
    err = esb_set_prefixes(prefixes, 8);
    if (err) {
        LOG_ERR("Failed to set prefixes");
    }
    err = esb_set_rf_channel(2);
    if (err) {
        LOG_ERR("Failed to set RF channel, err %d", err);
    }
    err = esb_start_tx();
    if (err != 0) {LOG_ERR("ESB TX failed to start, err %d", err); }

    // Initialize PWM
    pwm_set_dt(&pwm_red, max_period, 0);
    pwm_set_dt(&pwm_green, max_period, 0);
    pwm_set_dt(&pwm_blue, max_period, 0);
    LOG_INF("PWM initialized");

    // Initialize ADC
    if (!device_is_ready(adc.dev)) {
        LOG_ERR("ADC device not ready: %s", adc.dev->name);
    }
    err = adc_channel_setup_dt(&adc);
    if (err < 0) {
        LOG_ERR("ADC channel setup failed (ID: %u, err: %d)", adc.channel_id, err);
    }

    // Initialize workqueue for battery status
    k_work_init(&battery_work, battery_work_handler);

    if (!device_is_ready(wake_pin.port)) { LOG_ERR("Wake GPIO not ready"); }
    err = gpio_pin_configure_dt(&wake_pin, GPIO_INPUT | GPIO_PULL_DOWN);
    if (err) { LOG_ERR("Failed to configure wake pin (P0.06), err %d", err); }
    err = gpio_pin_interrupt_configure_dt(&wake_pin, GPIO_INT_EDGE_RISING);
    if (err) { LOG_ERR("Failed to configure wake pin interrupt (P0.06), err %d", err); }
    gpio_init_callback(&wake_pin_cb_data, wake_pin_interrupt_handler, BIT(wake_pin.pin));
    err = gpio_add_callback(wake_pin.port, &wake_pin_cb_data);
    if (err) { LOG_ERR("Failed to add wake pin callback (P0.06), err %d", err); }
    if (!device_is_ready(cs_pin.port)) { LOG_ERR("CS GPIO not ready"); }
    err = gpio_pin_configure_dt(&cs_pin, GPIO_INPUT | GPIO_PULL_UP);
    if (err) { LOG_ERR("Failed to configure CS pin (P0.08), err %d", err); }
    err = gpio_pin_interrupt_configure_dt(&cs_pin, GPIO_INT_EDGE_FALLING);
    if (err) { LOG_ERR("Failed to configure CS pin interrupt (P0.08), err %d", err); }
    gpio_init_callback(&cs_pin_cb_data, cs_pin_interrupt_handler, BIT(cs_pin.pin));
    err = gpio_add_callback(cs_pin.port, &cs_pin_cb_data);
    if (err) { LOG_ERR("Failed to add CS pin callback (P0.08), err %d", err); }
    LOG_INF("GPIO pins configured for wake and CS");
    LOG_INF("System initialized, entering main loop");

    k_timer_start(&battery_display_timer, K_NO_WAIT, K_SECONDS(5));
    return 0;
}