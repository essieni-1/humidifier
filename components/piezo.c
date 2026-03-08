#include "piezo.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PIEZO";

// ── UART config ───────────────────────────────────────────────
#define UART_NUM          UART_NUM_2
#define UART_BAUD         9600
#define UART_BUF_SIZE     256
#define UART_WAIT_TIMEOUT 300   // ms to wait for a response

// ── Protocol constants ────────────────────────────────────────
#define HEADER          0x55
#define MODULE_ID_HIGH  0x31
#define MODULE_ID_LOW   0x03

// Parameter reading commands (include module ID in frame)
#define READ_OUTPUT_DUTY       0x11
#define GET_ATOMIZE_TIME       0x12
#define GET_WATER_LEVEL_STATUS 0x13

// Parameter setting commands (no module ID in frame)
#define SET_PWM_DUTY       0x01
#define SET_ATOMIZE_TIME   0x02

// ── Internal: build and send a frame ─────────────────────────
//   is_read=1  → reading command  (header + module_id + cmd + len + data + chk)
//   is_read=0  → setting command  (header + cmd + len + data + chk)
static void send_command(uint8_t cmd, uint8_t *data, uint8_t len, uint8_t is_read) {
    uint8_t frame[16];
    int idx = 0;

    frame[idx++] = HEADER;
    if (is_read) {
        frame[idx++] = MODULE_ID_HIGH;
        frame[idx++] = MODULE_ID_LOW;
    }
    frame[idx++] = cmd;
    frame[idx++] = len;

    uint8_t checksum = HEADER
                     + (is_read ? (MODULE_ID_HIGH + MODULE_ID_LOW) : 0)
                     + cmd + len;

    for (int i = 0; i < len; i++) {
        frame[idx++] = data[i];
        checksum    += data[i];
    }
    frame[idx++] = checksum & 0xFF;

    uart_write_bytes(UART_NUM, (const char *)frame, idx);
}

// ── Internal: receive and validate a response frame ───────────
// Returns number of data bytes on success, negative on error:
//   -1 = timeout / frame too short
//   -2 = bad header
//   -3 = data longer than buffer
//   -4 = checksum mismatch
static int receive_response(uint8_t *data, uint8_t max_len, uint8_t *status) {
    uint8_t rxbuf[16] = {0};
    int len = uart_read_bytes(UART_NUM, rxbuf, sizeof(rxbuf),
                              pdMS_TO_TICKS(UART_WAIT_TIMEOUT));

    if (len < 6)              return -1;
    if (rxbuf[0] != HEADER)   return -2;

    *status = rxbuf[3];
    uint8_t data_len = rxbuf[4];
    if (data_len > max_len)   return -3;

    // Verify checksum over header + module_id bytes + status + data_len + data
    uint8_t checksum = 0;
    for (int i = 0; i < 5 + data_len; i++) checksum += rxbuf[i];
    checksum &= 0xFF;
    if (checksum != rxbuf[5 + data_len]) return -4;

    for (int i = 0; i < data_len; i++) data[i] = rxbuf[5 + i];
    return data_len;
}

// ── Public API ────────────────────────────────────────────────

void piezo_init(int tx_gpio, int rx_gpio) {
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM, &cfg);
    uart_set_pin(UART_NUM, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);

    ESP_LOGI(TAG, "Piezo UART ready (TX=GPIO%d RX=GPIO%d @ %d baud)",
             tx_gpio, rx_gpio, UART_BAUD);
}

bool piezo_water_present(void) {
    send_command(GET_WATER_LEVEL_STATUS, NULL, 0, 1);
    uint8_t status = 0, level = 0;
    int ret = receive_response(&level, 1, &status);
    if (ret < 0) {
        ESP_LOGW(TAG, "Water level read failed (err %d)", ret);
        return false;
    }
    bool present = (level != 0);
    ESP_LOGI(TAG, "Water level: %s", present ? "FULL" : "EMPTY");
    return present;
}

bool piezo_start(void) {
    // Module will silently ignore us if tank is empty — check first
    if (!piezo_water_present()) {
        ESP_LOGW(TAG, "Cannot start — tank is empty");
        return false;
    }
    uint8_t duty = 255;
    send_command(SET_PWM_DUTY, &duty, 1, 0);
    ESP_LOGI(TAG, "Atomizer ON (duty=255)");
    return true;
}

void piezo_stop(void) {
    uint8_t duty = 0;
    send_command(SET_PWM_DUTY, &duty, 1, 0);
    ESP_LOGI(TAG, "Atomizer OFF (duty=0)");
}

bool piezo_read_duty(uint8_t *duty) {
    send_command(READ_OUTPUT_DUTY, NULL, 0, 1);
    uint8_t status = 0;
    int ret = receive_response(duty, 1, &status);
    if (ret < 0) {
        ESP_LOGW(TAG, "Read duty failed (err %d)", ret);
        return false;
    }
    ESP_LOGI(TAG, "Current duty: %d", *duty);
    return true;
}

void piezo_set_timer(uint8_t minutes) {
    send_command(SET_ATOMIZE_TIME, &minutes, 1, 0);
    ESP_LOGI(TAG, "Atomizer timer set to %d min", minutes);
}
