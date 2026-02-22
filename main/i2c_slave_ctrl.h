/*
 * I2C Slave Controller - LyraT audio player front-end interface
 *
 * The LyraT acts as I2C SLAVE at address 0x42.
 * A front-end microcontroller (master) sends commands to control playback.
 *
 * ============================================================
 *  COMMAND SET  (master -> slave write transactions)
 * ============================================================
 *
 *  CMD_PLAY      0x01  [0x01]
 *      Start or resume playback.
 *
 *  CMD_STOP      0x02  [0x02]
 *      Stop playback.
 *
 *  CMD_PAUSE     0x03  [0x03]
 *      Pause playback.
 *
 *  CMD_VOLUME    0x04  [0x04][vol:uint8]
 *      Set volume. vol = 0 (mute) .. 100 (max).
 *
 *  CMD_SET_URL   0x05  [0x05][len:uint8][url:len bytes]
 *      Set HTTP stream URL and immediately start playing.
 *      len = number of URL bytes, max 255.
 *      Example: https://icecast.omroep.nl/radio2-bb-mp3
 *
 *  CMD_STATUS    0x06  [0x06]
 *      Request status. Master must follow with a 2-byte READ:
 *        Byte 0: player status  (0=stopped, 1=playing, 2=paused, 0xFF=error)
 *        Byte 1: current volume (0-100)
 *
 * ============================================================
 *  WIRING
 * ============================================================
 *   LyraT SDA  ->  GPIO 21  ->  front-end SDA
 *   LyraT SCL  ->  GPIO 22  ->  front-end SCL
 *   GND        ->  GND
 *   (Add 4.7kOhm pull-ups to 3.3 V on SDA and SCL)
 *
 *  I2C slave address: 0x42 (7-bit)
 *  Speed: up to 400 kHz
 * ============================================================
 */

#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -- Hardware config --------------------------------------- */
#define I2C_SLAVE_ADDR      0x42
#define I2C_SLAVE_SDA_IO    21
#define I2C_SLAVE_SCL_IO    22
#define I2C_SLAVE_PORT      I2C_NUM_1

/* -- Limits ------------------------------------------------ */
#define MAX_URL_LEN         255
#define I2C_RX_BUF_SIZE     (MAX_URL_LEN + 4)   /* opcode + len + url + margin */

/* -- Command opcodes --------------------------------------- */
typedef enum {
    CMD_PLAY    = 0x01,
    CMD_STOP    = 0x02,
    CMD_PAUSE   = 0x03,
    CMD_VOLUME  = 0x04,
    CMD_SET_URL = 0x05,
    CMD_STATUS  = 0x06,
} i2c_cmd_opcode_t;

/* -- Player status codes (returned on CMD_STATUS read) ----- */
typedef enum {
    PLAYER_STOPPED = 0x00,
    PLAYER_PLAYING = 0x01,
    PLAYER_PAUSED  = 0x02,
    PLAYER_ERROR   = 0xFF,
} i2c_player_status_t;

/* -- Command struct passed via queue to audio task ---------- */
typedef struct {
    i2c_cmd_opcode_t opcode;
    uint8_t          volume;            /* valid for CMD_VOLUME  */
    char             url[MAX_URL_LEN + 1]; /* valid for CMD_SET_URL */
} i2c_command_t;

/* -- Public API -------------------------------------------- */

/**
 * @brief Initialise I2C slave and attach it to cmd_queue.
 *        Commands received over I2C are parsed and pushed onto
 *        cmd_queue as i2c_command_t structs.
 *
 * @param cmd_queue  FreeRTOS queue (item size = sizeof(i2c_command_t))
 * @return ESP_OK on success
 */
esp_err_t i2c_slave_ctrl_init(QueueHandle_t cmd_queue);

/**
 * @brief Update the status word returned when the master issues CMD_STATUS.
 *        Call this from the audio task whenever playback state changes.
 *
 * @param status  Current player status
 * @param volume  Current volume 0-100
 */
void i2c_slave_ctrl_set_status(i2c_player_status_t status, uint8_t volume);

#ifdef __cplusplus
}
#endif
