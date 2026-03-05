#include "dht20.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG  = "DHT20";
static int         _port = 0;

// ── I2C helpers ───────────────────────────────────────────────

static esp_err_t i2c_write_bytes(uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ── CRC-8 check (polynomial 0x31) ────────────────────────────
static uint8_t crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

// ── Public API ────────────────────────────────────────────────

bool dht20_init(int i2c_port) {
    _port = i2c_port;
    vTaskDelay(pdMS_TO_TICKS(100)); // Sensor needs 100ms after power-on

    // Read status byte
    uint8_t status = 0;
    if (i2c_read_bytes(&status, 1) != ESP_OK) {
        ESP_LOGE(TAG, "DHT20 not found on I2C bus");
        return false;
    }

    // If bit 3 is not set, sensor needs calibration command
    if ((status & 0x18) != 0x18) {
        ESP_LOGW(TAG, "DHT20 calibrating...");
        uint8_t cal_cmd[] = {0xBE, 0x08, 0x00};
        i2c_write_bytes(cal_cmd, 3);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "DHT20 ready");
    return true;
}

bool dht20_read(dht20_data_t *out) {
    out->valid = false;

    // 1. Send measurement trigger
    uint8_t trigger[] = {0xAC, 0x33, 0x00};
    if (i2c_write_bytes(trigger, 3) != ESP_OK) {
        ESP_LOGE(TAG, "Trigger write failed");
        return false;
    }

    // 2. Wait 80ms for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(80));

    // 3. Read 7 bytes: status + 5 data + crc
    uint8_t buf[7] = {0};
    if (i2c_read_bytes(buf, 7) != ESP_OK) {
        ESP_LOGE(TAG, "Data read failed");
        return false;
    }

    // 4. Check busy bit (bit 7 of status byte)
    if (buf[0] & 0x80) {
        ESP_LOGW(TAG, "Sensor busy");
        return false;
    }

    // 5. Verify CRC
    if (crc8(buf, 6) != buf[6]) {
        ESP_LOGE(TAG, "CRC mismatch");
        return false;
    }

    // 6. Parse raw 20-bit humidity and temperature values
    //    Humidity:    buf[1..3] upper 20 bits
    //    Temperature: buf[3..5] lower 20 bits
    uint32_t raw_hum  = ((uint32_t)buf[1] << 12) |
                        ((uint32_t)buf[2] << 4)  |
                        ((uint32_t)buf[3] >> 4);

    uint32_t raw_temp = ((uint32_t)(buf[3] & 0x0F) << 16) |
                        ((uint32_t)buf[4] << 8)            |
                        (uint32_t)buf[5];

    out->humidity    = (float)raw_hum  / 1048576.0f * 100.0f;
    out->temperature = (float)raw_temp / 1048576.0f * 200.0f - 50.0f;
    out->valid       = true;

    ESP_LOGI(TAG, "H: %.1f%%  T: %.1f°C", out->humidity, out->temperature);
    return true;
}
