#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise UART control interface.
 *
 * Protocol (binary, length-prefixed):
 *   [len:uint8][payload:len bytes]
 * Where payload matches the same opcode frames used by the I2C control interface.
 *
 * Reply (device -> host):
 *   On CMD_STATUS, device sends: [len=3][0x06][status:uint8][volume:uint8]
 */
esp_err_t uart_ctrl_init(QueueHandle_t cmd_queue);

/**
 * @brief Update status snapshot used for CMD_STATUS replies.
 */
void uart_ctrl_set_status(uint8_t status, uint8_t volume);

/**
 * @brief Send an unsolicited AUDIO_LEVEL frame to the host.
 *        Frame: [len=3][0x08][peak_L][peak_R]
 */
void uart_ctrl_send_audio_level(uint8_t left, uint8_t right);

#ifdef __cplusplus
}
#endif
