#pragma once

/*
 * AHT20 temperature & humidity sensor — I2C master driver
 *
 * Wiring (ESP32-LyraT v4.3):
 *   AHT20 VIN  -> 3.3V
 *   AHT20 GND  -> GND
 *   AHT20 SDA  -> GPIO 18   (+ 4.7kΩ pull-up to 3.3V)
 *   AHT20 SCL  -> GPIO 23   (+ 4.7kΩ pull-up to 3.3V)
 *
 * Uses I2C_NUM_0 (GPIO 18/23). De custom codec driver in dit project is een stub
 * en gebruikt I2C niet, dus er is geen busconflict.
 *
 * Starts a background FreeRTOS task that reads temperature + humidity every 2 seconds
 * and logs the result to the serial monitor.
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the AHT20 I2C bus and start the periodic read task.
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if the sensor did not respond,
 *         or another esp_err_t on lower-level failure.
 */
esp_err_t aht20_start(void);

/**
 * @brief Haal de laatste gemeten waarden op (thread-safe).
 *
 * @param temp_c   Pointer voor temperatuur in °C (mag NULL zijn)
 * @param hum_pct  Pointer voor relatieve vochtigheid in % (mag NULL zijn)
 * @return ESP_OK als er een geldige meting beschikbaar is,
 *         ESP_ERR_INVALID_STATE als nog geen meting is gedaan.
 */
esp_err_t aht20_get_last(float *temp_c, float *hum_pct);

#ifdef __cplusplus
}
#endif
