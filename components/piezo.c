#include "piezo.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "PIEZO";

// LEDC channel/timer assignments (use timer 0 for piezo)
#define PIEZO_LEDC_TIMER    LEDC_TIMER_0
#define PIEZO_LEDC_CHANNEL  LEDC_CHANNEL_0
#define PIEZO_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define PIEZO_DUTY_RES      LEDC_TIMER_10_BIT   // 0–1023
#define PIEZO_DUTY_50PCT    512                  // 50% of 1023

static int _water_gpio = -1;

void piezo_init(int pwm_gpio, int water_level_gpio) {
    // ── Configure LEDC timer ──────────────────────────────────
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = PIEZO_LEDC_MODE,
        .timer_num       = PIEZO_LEDC_TIMER,
        .duty_resolution = PIEZO_DUTY_RES,
        .freq_hz         = PIEZO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    // ── Configure LEDC channel (output starts at 0% duty = OFF) ─
    ledc_channel_config_t chan_cfg = {
        .gpio_num   = pwm_gpio,
        .speed_mode = PIEZO_LEDC_MODE,
        .channel    = PIEZO_LEDC_CHANNEL,
        .timer_sel  = PIEZO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&chan_cfg);

    // ── Configure water-level GPIO as input with pull-down ───
    _water_gpio = water_level_gpio;
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << water_level_gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    ESP_LOGI(TAG, "Piezo PWM on GPIO%d @ %d Hz, water sensor on GPIO%d",
             pwm_gpio, PIEZO_PWM_FREQ_HZ, water_level_gpio);
}

void piezo_set_active(bool active) {
    uint32_t duty = active ? PIEZO_DUTY_50PCT : 0;
    ledc_set_duty(PIEZO_LEDC_MODE, PIEZO_LEDC_CHANNEL, duty);
    ledc_update_duty(PIEZO_LEDC_MODE, PIEZO_LEDC_CHANNEL);
    ESP_LOGI(TAG, "Atomizer %s", active ? "ON" : "OFF");
}

bool piezo_water_present(void) {
    if (_water_gpio < 0) return false;
    return gpio_get_level(_water_gpio) == 1;
}
