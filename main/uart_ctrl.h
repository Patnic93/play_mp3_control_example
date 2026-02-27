#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise UART RX control interface.
 *
 * Protocol (binary, length-prefixed):
 *   [len:uint8][payload:len bytes]
 * Where payload matches the same opcode frames used by the I2C control interface.
 */
esp_err_t uart_ctrl_init(QueueHandle_t cmd_queue);

#ifdef __cplusplus
}
#endif
