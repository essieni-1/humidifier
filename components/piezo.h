#pragma once
#include <stdbool.h>
#include <stdint.h>

/*
 * piezo.h
 * UART driver for BM52O5221-1 piezo atomizer module
 *
 * IMPORTANT from module datasheet:
 *   If the module detects NO water at the hardware level it will
 *   silently ignore ALL commands. Always call piezo_water_present()
 *   before any action to avoid the module going unresponsive.
 *
 * Wiring:
 *   ESP32-S3 TX (PIN_PIEZO_TX) → Module RX
 *   ESP32-S3 RX (PIN_PIEZO_RX) → Module TX
 *   GND → GND
 */

/**
 * @brief Initialize UART2 for the piezo module.
 * @param tx_gpio  GPIO for ESP TX → module RX
 * @param rx_gpio  GPIO for module TX → ESP RX
 */
void piezo_init(int tx_gpio, int rx_gpio);

/**
 * @brief Query the module's water level sensor over UART.
 * @return true if water is present, false if tank is empty.
 *         Always check this before calling piezo_start().
 */
bool piezo_water_present(void);

/**
 * @brief Start atomizing at full duty (255).
 *        Checks water level first — returns false if tank empty.
 * @return true if atomizer started, false if refused (no water)
 */
bool piezo_start(void);

/** @brief Stop atomizing (set duty to 0). */
void piezo_stop(void);

/**
 * @brief Read the current PWM duty reported by the module (0-255).
 * @param duty  Output pointer
 * @return true on success
 */
bool piezo_read_duty(uint8_t *duty);

/**
 * @brief Set atomizer run timer in minutes. 0 = run indefinitely.
 */
void piezo_set_timer(uint8_t minutes);
