#include "lcd_i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "LCD";

// ── PCF8574 bit positions ────────────────────────────────────
#define LCD_RS   0x01   // P0
#define LCD_RW   0x02   // P1  (tied LOW, always write)
#define LCD_EN   0x04   // P2
#define LCD_BL   0x08   // P3  backlight
#define LCD_D4   0x10   // P4
#define LCD_D5   0x20   // P5
#define LCD_D6   0x40   // P6
#define LCD_D7   0x80   // P7

// ── HD44780 commands ─────────────────────────────────────────
#define LCD_CMD_CLEAR        0x01
#define LCD_CMD_HOME         0x02
#define LCD_CMD_ENTRY_MODE   0x06   // Increment, no shift
#define LCD_CMD_DISPLAY_ON   0x0C   // Display on, cursor off, blink off
#define LCD_CMD_FUNC_4BIT    0x28   // 4-bit, 2-line, 5x8 font
#define LCD_CMD_DDRAM        0x80   // Base address for DDRAM set

// Row offsets for up to 4-row displays
static const uint8_t ROW_OFFSETS[4] = {0x00, 0x40, 0x14, 0x54};

// ── State ─────────────────────────────────────────────────────
static int     _port;
static uint8_t _addr;
static uint8_t _cols;
static uint8_t _rows;
static uint8_t _backlight = LCD_BL;  // default: backlight ON

// ── Low-level I2C write ───────────────────────────────────────
static void pcf8574_write(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data | _backlight, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
}

// Pulse the EN pin to clock data into the HD44780
static void lcd_pulse_enable(uint8_t data) {
    pcf8574_write(data | LCD_EN);
    esp_rom_delay_us(1);
    pcf8574_write(data & ~LCD_EN);
    esp_rom_delay_us(50);
}

// Send 4-bit nibble
static void lcd_write4bits(uint8_t nibble, uint8_t rs) {
    uint8_t data = rs;
    data |= (nibble & 0x01) ? LCD_D4 : 0;
    data |= (nibble & 0x02) ? LCD_D5 : 0;
    data |= (nibble & 0x04) ? LCD_D6 : 0;
    data |= (nibble & 0x08) ? LCD_D7 : 0;
    lcd_pulse_enable(data);
}

// Send full byte (2 nibbles) — rs=0 for command, rs=LCD_RS for data
static void lcd_send(uint8_t value, uint8_t rs) {
    lcd_write4bits(value >> 4, rs);   // High nibble first
    lcd_write4bits(value & 0x0F, rs); // Low nibble
}

// ── Public API ────────────────────────────────────────────────

void lcd_init(int i2c_port, uint8_t addr, uint8_t cols, uint8_t rows) {
    _port = i2c_port;
    _addr = addr;
    _cols = cols;
    _rows = rows;

    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for LCD power-up

    // HD44780 4-bit initialization sequence (per datasheet)
    pcf8574_write(0x00);
    vTaskDelay(pdMS_TO_TICKS(20));

    lcd_write4bits(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4bits(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4bits(0x03, 0); esp_rom_delay_us(150);
    lcd_write4bits(0x02, 0); // Switch to 4-bit mode

    lcd_send(LCD_CMD_FUNC_4BIT,   0);
    lcd_send(LCD_CMD_DISPLAY_ON,  0);
    lcd_send(LCD_CMD_CLEAR,       0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send(LCD_CMD_ENTRY_MODE,  0);

    ESP_LOGI(TAG, "LCD initialized (%dx%d) at I2C addr 0x%02X", cols, rows, addr);
}

void lcd_clear(void) {
    lcd_send(LCD_CMD_CLEAR, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    if (row >= _rows) row = _rows - 1;
    if (col >= _cols) col = _cols - 1;
    lcd_send(LCD_CMD_DDRAM | (col + ROW_OFFSETS[row]), 0);
}

void lcd_print_char(char c) {
    lcd_send((uint8_t)c, LCD_RS);
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_print_char(*str++);
    }
}

void lcd_backlight(uint8_t on) {
    _backlight = on ? LCD_BL : 0;
    pcf8574_write(0);  // Refresh backlight state
}
