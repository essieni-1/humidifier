/*
 *  PIN ASSIGNMENTS
 *  ───────────────────────────────────────────────────────────
 *  DHT20 Sensor (I2C)
 *    SDA          → GPIO 20
 *    SCL          → GPIO 21
 *
 *  LCD (direct 4-bit parallel via hd44780)
 *    RS           → GPIO 38
 *    E            → GPIO 37
 *    D4           → GPIO 36
 *    D5           → GPIO 35
 *    D6           → GPIO 48
 *    D7           → GPIO 47
 *    Backlight    → not used (tie to 3.3V directly)
 *
 *  Piezo module (UART2)
 *    ESP TX       → GPIO 17  (→ module RX)
 *    ESP RX       → GPIO 18  (← module TX)
 *
 *  Button         → GPIO 4   (other leg to GND)
 *  Status LED     → GPIO 2   (~330Ω series resistor)
 * ============================================================
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hd44780.h"
#include "piezo.h"

static const char *TAG = "HUMIDIFIER";

// ─── I2C (DHT20 sensor) ───────────────────────────────────────
#define I2C_MASTER_SCL_IO    21
#define I2C_MASTER_SDA_IO    20
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000
#define DHT20_ADDR           0x38

// ─── Piezo UART ───────────────────────────────────────────────
#define PIN_PIEZO_TX   17
#define PIN_PIEZO_RX   18

// ─── Button / LED ─────────────────────────────────────────────
#define PIN_BUTTON     4
#define PIN_STATUS_LED 2

// ─── Humidity control ─────────────────────────────────────────
#define HUMIDITY_DEFAULT  40    // % RH starting threshold
#define HUMIDITY_MIN      20
#define HUMIDITY_MAX      70
#define HUMIDITY_STEP     5

// ─── Timing ──────────────────────────────────────────────────
#define SENSOR_POLL_MS   2000
#define LCD_REFRESH_MS   1000
#define DEBOUNCE_MS      200

// ─────────────────────────────────────────────────────────────
//  LCD — direct 4-bit parallel via hd44780 component
//  Add to your idf_component_manager or copy the component from:
//  https://github.com/UncleRus/esp-idf-lib/tree/master/components/hd44780
// ─────────────────────────────────────────────────────────────
static hd44780_t lcd = {
    .write_cb = NULL,
    .font     = HD44780_FONT_5X8,
    .lines    = 2,
    .pins = {
        .rs = GPIO_NUM_38,
        .e  = GPIO_NUM_37,
        .d4 = GPIO_NUM_36,
        .d5 = GPIO_NUM_35,
        .d6 = GPIO_NUM_48,
        .d7 = GPIO_NUM_47,
        .bl = HD44780_NOT_USED
    }
};

// ─── System state ─────────────────────────────────────────────
static float humidity_pct       = 0.0f;
static float temperature_c      = 0.0f;
static bool  water_present      = false;
static bool  humidifier_on      = false;
static int   humidity_threshold = HUMIDITY_DEFAULT;

static int64_t last_sensor_us = 0;
static int64_t last_lcd_us    = 0;
static int64_t last_button_us = 0;

//initialization functions for I2C and GPIOs

static void init_i2c(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "I2C ready (SDA=GPIO%d SCL=GPIO%d)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
}

static void init_button_and_led(void) {
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

    gpio_config_t led = {
        .pin_bit_mask = (1ULL << PIN_STATUS_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    gpio_set_level(PIN_STATUS_LED, 1); // ON = system alive
}

// ─────────────────────────────────────────────────────────────
//  DHT20 reading (your exact logic from the provided code)
// ─────────────────────────────────────────────────────────────

/*
 * read_dht20()
 * Triggers a measurement and reads 7 bytes from the DHT20.
 * Writes results into humidity_pct and temperature_c globals.
 * Returns true on success.
 */
static bool read_dht20(void) {
    uint8_t trigger_cmd[] = {0xAC, 0x33, 0x00};
    uint8_t data[7]       = {0};

    // Trigger measurement
    i2c_master_write_to_device(I2C_MASTER_NUM, DHT20_ADDR,
                               trigger_cmd, 3, pdMS_TO_TICKS(100));

    // Datasheet: wait 80ms for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(80));

    // Read 7 bytes back
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, DHT20_ADDR,
                                                data, 7, pdMS_TO_TICKS(100));

    // Bit 7 of status byte = busy flag; must be 0 to accept data
    if (err != ESP_OK || (data[0] & 0x80) != 0) {
        ESP_LOGE(TAG, "DHT20 read failed (err=0x%x busy=%d)",
                 err, (data[0] & 0x80) != 0);
        return false;
    }

    // Humidity (datasheet page 10)
    uint32_t raw_hum  = ((uint32_t)data[1] << 12)
                      | ((uint32_t)data[2] << 4)
                      |  (data[3] >> 4);
    humidity_pct = (float)raw_hum * 100.0f / 1048576.0f;

    // Temperature (datasheet page 10)
    uint32_t raw_temp = ((uint32_t)(data[3] & 0x0F) << 16)
                      | ((uint32_t)data[4] << 8)
                      |  data[5];
    temperature_c = (float)raw_temp * 200.0f / 1048576.0f - 50.0f;

    ESP_LOGI(TAG, "DHT20: T=%.1f C  H=%.1f %%", temperature_c, humidity_pct);
    return true;
}

// ─────────────────────────────────────────────────────────────
// Humidifier control logic
// ─────────────────────────────────────────────────────────────

static void start_humidifier(void) {
    if (humidifier_on) return;
    if (!piezo_start()) {
        ESP_LOGW(TAG, "Atomizer refused to start — tank empty");
        return;
    }
    humidifier_on = true;
    ESP_LOGI(TAG, "Humidifier ON");
}

static void stop_humidifier(void) {
    if (!humidifier_on) return;
    piezo_stop();
    humidifier_on = false;
    ESP_LOGI(TAG, "Humidifier OFF");
}

static void run_control_logic(void) {
    // Safety: tank ran dry while running — stop immediately
    if (!water_present && humidifier_on) {
        ESP_LOGW(TAG, "Water empty — emergency stop");
        stop_humidifier();
        return;
    }

    if (humidity_pct < (float)humidity_threshold && water_present) {
        start_humidifier();
    } else if (humidity_pct >= (float)humidity_threshold) {
        stop_humidifier();
    }
}

// ─────────────────────────────────────────────────────────────
//  LCD display (using hd44780 component)
// ─────────────────────────────────────────────────────────────

/*
 * Normal display (16x2):
 *   Row 0:  "Temp: 22.1 C    "
 *   Row 1:  "Hum:  65.2 %    "
 *
 * When humidifier is running, row 1 appends state:
 *   Row 0:  "Temp: 22.1 C    "
 *   Row 1:  "Hum:65% ON  SET:40"  (truncated to 16 chars)
 */
static void update_lcd_normal(void) {
    char line[17];

    // Row 0: temperature
    hd44780_gotoxy(&lcd, 0, 0);
    snprintf(line, sizeof(line), "Temp: %.1f C    ", temperature_c);
    hd44780_puts(&lcd, line);

    // Row 1: humidity + humidifier state
    hd44780_gotoxy(&lcd, 0, 1);
    snprintf(line, sizeof(line), "Hum:%.1f%% %s %d%%",
             humidity_pct,
             humidifier_on ? "ON " : "OFF",
             humidity_threshold);
    hd44780_puts(&lcd, line);
}

static void update_lcd_low_water(void) {
    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "!! REFILL WATER!");
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, "  Tank is empty ");
}

static void update_lcd_sensor_error(void) {
    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "Sensor Error    ");
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, "Check I2C wiring");
}

static void update_lcd_threshold_confirm(void) {
    char line[17];
    hd44780_clear(&lcd);
    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, " Threshold set: ");
    hd44780_gotoxy(&lcd, 0, 1);
    snprintf(line, sizeof(line), "     %d%% RH     ", humidity_threshold);
    hd44780_puts(&lcd, line);
    vTaskDelay(pdMS_TO_TICKS(800));
}

// ─────────────────────────────────────────────────────────────
//  Button — cycles humidity threshold up in 5% steps
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
            ESP_LOGI(TAG, "Threshold -> %d%%", humidity_threshold);
            update_lcd_threshold_confirm();
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  ESP-IDF entry point
// ─────────────────────────────────────────────────────────────

void app_main(void) {
    ESP_LOGI(TAG, "=== Humidifier Starting ===");

    // ── Iyeneobong: init all hardware ────────────────────────
    init_i2c();
    init_button_and_led();
    piezo_init(PIN_PIEZO_TX, PIN_PIEZO_RX);

    // Init LCD (direct GPIO hd44780)
    // Datasheet: wait >100ms after power-up
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_ERROR_CHECK(hd44780_init(&lcd));
    hd44780_clear(&lcd);
    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "  Humidifier v1 ");
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, " System  Ready! ");
    vTaskDelay(pdMS_TO_TICKS(1500));
    hd44780_clear(&lcd);

    ESP_LOGI(TAG, "=== Ready — target %d%% RH ===", humidity_threshold);

    // ── Main loop ─────────────────────────────────────────────
    while (1) {
        int64_t now_us = esp_timer_get_time();

        // 1. Read sensors every SENSOR_POLL_MS
        if ((now_us - last_sensor_us) >= (SENSOR_POLL_MS * 1000LL)) {
            last_sensor_us = now_us;

            // Water level MUST come first — module ignores all
            // commands if it detects no water at the hardware level
            water_present = piezo_water_present();

            // DHT20 read (trigger + 80ms wait + read is inside)
            bool sensor_ok = read_dht20();

            if (sensor_ok) {
                run_control_logic();
            } else {
                // Can't trust humidity reading — stop to be safe
                stop_humidifier();
            }
        }

        // 2. Refresh LCD every LCD_REFRESH_MS
        if ((now_us - last_lcd_us) >= (LCD_REFRESH_MS * 1000LL)) {
            last_lcd_us = now_us;

            if (!water_present && !humidifier_on) {
                update_lcd_low_water();
            } else if (humidity_pct == 0.0f && temperature_c == 0.0f) {
                update_lcd_sensor_error();
            } else {
                update_lcd_normal();
            }
        }

        // 3. Check button every loop cycle
        handle_button();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
