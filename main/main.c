#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "lcd_i2c.h"
#include "dht20.h"
#include "piezo.h"

static const char *TAG = "MAIN";

// ─── Pin Assignments ─────────────────────────────────────────
#define PIN_I2C_SDA       8
#define PIN_I2C_SCL       9
#define PIN_PIEZO_PWM     5
#define PIN_PIEZO_WATER   6
#define PIN_FAN_PWM       7
#define PIN_BUTTON        4
#define PIN_STATUS_LED    2

// ─── I2C Config ──────────────────────────────────────────────
#define I2C_PORT          I2C_NUM_0
#define I2C_FREQ_HZ       100000    // 100 kHz standard mode

// ─── Fan PWM (LEDC channel 1, timer 1) ───────────────────────
#define FAN_LEDC_TIMER    LEDC_TIMER_1
#define FAN_LEDC_CHANNEL  LEDC_CHANNEL_1
#define FAN_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define FAN_DUTY_RES      LEDC_TIMER_10_BIT
#define FAN_FREQ_HZ       25000     // 25 kHz — standard PC fan PWM
#define FAN_DUTY_FULL     1023      // 100%
#define FAN_DUTY_OFF      0

// ─── Humidity Control ────────────────────────────────────────
#define HUMIDITY_DEFAULT  40        // % RH
#define HUMIDITY_MIN      20
#define HUMIDITY_MAX      70
#define HUMIDITY_STEP     5

// ─── Timing ──────────────────────────────────────────────────
#define SENSOR_POLL_MS    2000
#define LCD_REFRESH_MS    1000
#define DEBOUNCE_MS       200
#define FAN_RAMP_MS       500       // Fan lead time before atomizer

// ─── System State ────────────────────────────────────────────
static float humidity_pct      = 0.0f;
static float temperature_c     = 0.0f;
static bool  water_present     = false;
static bool  humidifier_on     = false;
static int   humidity_threshold = HUMIDITY_DEFAULT;

static int64_t last_sensor_us   = 0;
static int64_t last_lcd_us      = 0;
static int64_t last_button_us   = 0;

// ─────────────────────────────────────────────────────────────
//  IYENEOBONG — Initialization
// ─────────────────────────────────────────────────────────────

static void init_i2c(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    ESP_LOGI(TAG, "I2C ready (SDA=%d SCL=%d)", PIN_I2C_SDA, PIN_I2C_SCL);
}

static void init_fan(void) {
    ledc_timer_config_t t = {
        .speed_mode      = FAN_LEDC_MODE,
        .timer_num       = FAN_LEDC_TIMER,
        .duty_resolution = FAN_DUTY_RES,
        .freq_hz         = FAN_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num   = PIN_FAN_PWM,
        .speed_mode = FAN_LEDC_MODE,
        .channel    = FAN_LEDC_CHANNEL,
        .timer_sel  = FAN_LEDC_TIMER,
        .duty       = FAN_DUTY_OFF,
        .hpoint     = 0,
    };
    ledc_channel_config(&c);
    ESP_LOGI(TAG, "Fan PWM ready on GPIO%d", PIN_FAN_PWM);
}

static void init_button_and_led(void) {
    // Button — active LOW
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

    // Status LED
    gpio_config_t led = {
        .pin_bit_mask = (1ULL << PIN_STATUS_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    gpio_set_level(PIN_STATUS_LED, 1); // LED on = system alive
}

// ─────────────────────────────────────────────────────────────
//  Fan helpers
// ─────────────────────────────────────────────────────────────

static void fan_set(bool on) {
    uint32_t duty = on ? FAN_DUTY_FULL : FAN_DUTY_OFF;
    ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty);
    ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL);
}

// ─────────────────────────────────────────────────────────────
//  KIEN — Humidifier control logic
// ─────────────────────────────────────────────────────────────

static void start_humidifier(void) {
    if (humidifier_on) return;
    ESP_LOGI(TAG, "Starting humidifier...");
    fan_set(true);
    vTaskDelay(pdMS_TO_TICKS(FAN_RAMP_MS));  // Fan first, then mist
    piezo_set_active(true);
    humidifier_on = true;
}

static void stop_humidifier(void) {
    if (!humidifier_on) return;
    ESP_LOGI(TAG, "Stopping humidifier...");
    piezo_set_active(false);
    vTaskDelay(pdMS_TO_TICKS(FAN_RAMP_MS));  // Clear residual mist
    fan_set(false);
    humidifier_on = false;
}

static void run_control_logic(void) {
    // SAFETY: tank empty → shut down immediately
    if (!water_present && humidifier_on) {
        ESP_LOGW(TAG, "Water empty — emergency stop");
        stop_humidifier();
        return;
    }

    bool below_threshold = (humidity_pct < (float)humidity_threshold);

    if (below_threshold && water_present) {
        start_humidifier();
    } else if (!below_threshold) {
        stop_humidifier();
    }
}

// ─────────────────────────────────────────────────────────────
//  KIEN — LCD display
// ─────────────────────────────────────────────────────────────

/*
 * Normal display layout (16x2):
 *   Row 0:  "H:65.2%  T:22.1C"
 *   Row 1:  "Set:40% ON  FULL"
 */
static void update_lcd_normal(void) {
    char line[17];

    // Row 0 — sensor readings
    lcd_set_cursor(0, 0);
    snprintf(line, sizeof(line), "H:%4.1f%% T:%4.1fC", humidity_pct, temperature_c);
    lcd_print(line);

    // Row 1 — threshold, state, water
    lcd_set_cursor(0, 1);
    snprintf(line, sizeof(line), "Set:%2d%% %-3s %s",
             humidity_threshold,
             humidifier_on  ? "ON"  : "OFF",
             water_present  ? "FULL" : "EMPT");
    lcd_print(line);
}

static void update_lcd_low_water(void) {
    lcd_set_cursor(0, 0);
    lcd_print("!! REFILL WATER!");
    lcd_set_cursor(0, 1);
    lcd_print("  Tank is empty ");
}

static void update_lcd_threshold_confirm(void) {
    char line[17];
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print(" Threshold set: ");
    lcd_set_cursor(0, 1);
    snprintf(line, sizeof(line), "     %d%% RH     ", humidity_threshold);
    lcd_print(line);
    vTaskDelay(pdMS_TO_TICKS(800));
}

// ─────────────────────────────────────────────────────────────
//  Button handler (cycles threshold)
// ─────────────────────────────────────────────────────────────

static void handle_button(void) {
    if (gpio_get_level(PIN_BUTTON) == 0) {  // Active LOW
        int64_t now = esp_timer_get_time();
        if ((now - last_button_us) > (DEBOUNCE_MS * 1000LL)) {
            last_button_us = now;
            humidity_threshold += HUMIDITY_STEP;
            if (humidity_threshold > HUMIDITY_MAX) {
                humidity_threshold = HUMIDITY_MIN;
            }
            ESP_LOGI(TAG, "Threshold → %d%%", humidity_threshold);
            update_lcd_threshold_confirm();
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  ESP-IDF entry point
// ─────────────────────────────────────────────────────────────

void app_main(void) {
    ESP_LOGI(TAG, "=== Humidifier System Starting ===");

    //intialize hardware
    init_i2c();
    init_fan();
    init_button_and_led();
    piezo_init(PIN_PIEZO_PWM, PIN_PIEZO_WATER);
    lcd_init(I2C_PORT, LCD_I2C_ADDR_DEFAULT, LCD_COLS, LCD_ROWS);
    dht20_init(I2C_PORT);

    // Splash screen
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("  Humidifier v1 ");
    lcd_set_cursor(0, 1);
    lcd_print(" System  Ready! ");
    vTaskDelay(pdMS_TO_TICKS(1500));
    lcd_clear();

    ESP_LOGI(TAG, "=== System Ready — target %d%% RH ===", humidity_threshold);

    // ── Main loop ─────────────────────────────────────────────
    while (1) {
        int64_t now_us = esp_timer_get_time();

        // 1. Read sensors every SENSOR_POLL_MS
        if ((now_us - last_sensor_us) >= (SENSOR_POLL_MS * 1000LL)) {
            last_sensor_us = now_us;

            // Water level (GPIO)
            water_present = piezo_water_present();

            // Humidity + temperature (I2C)
            dht20_data_t reading;
            if (dht20_read(&reading) && reading.valid) {
                humidity_pct    = reading.humidity;
                temperature_c   = reading.temperature;
            } else {
                ESP_LOGW(TAG, "DHT20 read failed");
            }

            // Run control decision
            run_control_logic();
        }

        // 2. Refresh LCD every LCD_REFRESH_MS
        if ((now_us - last_lcd_us) >= (LCD_REFRESH_MS * 1000LL)) {
            last_lcd_us = now_us;
            if (!water_present && !humidifier_on) {
                update_lcd_low_water();
            } else {
                update_lcd_normal();
            }
        }

        // 3. Poll button every loop iteration
        handle_button();

        // Yield to FreeRTOS scheduler
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}