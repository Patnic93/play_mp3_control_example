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
 *   Pins are configurable via sdkconfig (CONFIG_PLAY_MP3_I2C_SLAVE_SDA_GPIO / SCL).
 *   Recommended (ESP32-LyraT v4.3, avoid codec I2C pins 18/23):
 *     LyraT SDA  ->  GPIO 15 (JP7 pin 1 / TDO)  ->  front-end SDA
 *     LyraT SCL  ->  GPIO 14 (JP7 pin 4 / TMS)  ->  front-end SCL
 *   Avoid:
 *     GPIO 13 is often used for I2S WS/LRCLK on LyraT-style boards (audio codec), so it may be driven and break I2C.
 *     GPIO 12 is a strapping pin on ESP32; external pull-ups can change boot mode / flash voltage.
 *
 *   Physical note (LyraT v4.3): GPIO32/GPIO33 are commonly wired to the MODE/REC buttons.
 *   They can work electrically, but the button circuitry may load SDA/SCL.
 *
 *   Notes:
 *   - Many LyraT-class boards use GPIO18/23 for the on-board audio codec I2C master bus.
 *     If the codec/ADF creates an I2C master on those pins, running the external control
 *     interface as an I2C SLAVE on the same pins/port can become unreliable.
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

/* Optional: config comes from Kconfig.projbuild (sdkconfig.h). */
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -- Hardware config --------------------------------------- */
#if defined(CONFIG_PLAY_MP3_I2C_SLAVE_ADDR)
#define I2C_SLAVE_ADDR      CONFIG_PLAY_MP3_I2C_SLAVE_ADDR
#else
#define I2C_SLAVE_ADDR      0x42
#endif

#if defined(CONFIG_PLAY_MP3_I2C_SLAVE_SDA_GPIO)
#define I2C_SLAVE_SDA_IO    CONFIG_PLAY_MP3_I2C_SLAVE_SDA_GPIO
#else
#define I2C_SLAVE_SDA_IO    15
#endif

#if defined(CONFIG_PLAY_MP3_I2C_SLAVE_SCL_GPIO)
#define I2C_SLAVE_SCL_IO    CONFIG_PLAY_MP3_I2C_SLAVE_SCL_GPIO
#else
#define I2C_SLAVE_SCL_IO    14
#endif

#if defined(CONFIG_PLAY_MP3_I2C_SLAVE_PORT_0) && (CONFIG_PLAY_MP3_I2C_SLAVE_PORT_0)
#define I2C_SLAVE_PORT      I2C_NUM_0
#else
/* Default to I2C_NUM_1 to avoid clashing with codec I2C on many boards. */
#define I2C_SLAVE_PORT      I2C_NUM_1
#endif

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
