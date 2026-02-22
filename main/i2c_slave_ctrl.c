/*
 * I2C Slave Controller - implementation  (ESP-IDF I2C Slave Driver V2)
 *
 * Receives command frames from a front-end I2C master and pushes
 * parsed i2c_command_t structs onto the audio-task command queue.
 *
 * ISR path  : i2c_rx_cb (on_receive) -> xStreamBufferSendFromISR
 * Task path : i2c_parse_task -> xQueueSend (cmd_queue)
 * Read path : i2c_request_cb (on_request) -> i2c_slave_write (status reply)
 *
 * The master can read the 2-byte status at ANY time (no CMD_STATUS write
 * needed), but sending 0x06 first is recommended for protocol clarity.
 *
 * NOTE: master should insert a >=5 ms delay between writing 0x06 and
 *       reading 2 bytes, to give the slave time to process.
 */

#include "i2c_slave_ctrl.h"

#include "driver/i2c_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>

static const char *TAG = "I2C_SLAVE";

/* Internal state */
static i2c_slave_dev_handle_t s_handle;
static QueueHandle_t          s_cmd_queue;

/*
 * Stream buffer: ISR writes frames as [len:1 byte][data:len bytes].
 * Sized for 4 maximum-length frames.
 */
static StreamBufferHandle_t s_stream;

/*
 * Status reply bytes: [0]=player_status, [1]=volume
 * Updated by audio task via i2c_slave_ctrl_set_status().
 * Read by on_request ISR.
 */
static volatile uint8_t  s_status_reply[2] = {PLAYER_STOPPED, 50};
static SemaphoreHandle_t s_status_mutex;

/* on_request ISR: master wants to read -> send status */
static bool IRAM_ATTR i2c_request_cb(i2c_slave_dev_handle_t handle,
                                      const i2c_slave_request_event_data_t *evt,
                                      void *arg)
{
    (void)evt;
    BaseType_t woken = pdFALSE;

    /* Copy snapshot of status (no mutex in ISR - reads are atomic on 8-bit) */
    uint8_t reply[2] = {s_status_reply[0], s_status_reply[1]};
    uint32_t written = 0;
    i2c_slave_write(handle, reply, sizeof(reply), &written, 0);

    return woken == pdTRUE;
}

/* on_receive ISR: master finished writing */
static bool IRAM_ATTR i2c_rx_cb(i2c_slave_dev_handle_t handle,
                                  const i2c_slave_rx_done_event_data_t *evt,
                                  void *arg)
{
    BaseType_t woken = pdFALSE;

    uint8_t len = (uint8_t)((evt->length > I2C_RX_BUF_SIZE)
                                ? I2C_RX_BUF_SIZE
                                : evt->length);

    /* Write [len][data...] into stream buffer for parse task */
    xStreamBufferSendFromISR(s_stream, &len,        1,   &woken);
    xStreamBufferSendFromISR(s_stream, evt->buffer, len, &woken);

    return woken == pdTRUE;
}

/* Command parse task (normal task context) */
static void i2c_parse_task(void *arg)
{
    uint8_t len_byte;
    uint8_t data[I2C_RX_BUF_SIZE];

    for (;;) {
        /* Block until at least one byte arrives (the length prefix) */
        if (xStreamBufferReceive(s_stream, &len_byte, 1, portMAX_DELAY) != 1
                || len_byte == 0) {
            continue;
        }

        /* Read the payload */
        size_t got = xStreamBufferReceive(s_stream, data, len_byte,
                                          pdMS_TO_TICKS(200));
        if (got != len_byte) {
            ESP_LOGW(TAG, "Incomplete frame: expected %u got %u", len_byte, (unsigned)got);
            continue;
        }

        i2c_cmd_opcode_t op = (i2c_cmd_opcode_t)data[0];
        i2c_command_t    cmd = { .opcode = op };

        switch (op) {

            /* Single-byte commands */
            case CMD_PLAY:
            case CMD_STOP:
            case CMD_PAUSE:
                ESP_LOGI(TAG, "CMD 0x%02X received", op);
                xQueueSend(s_cmd_queue, &cmd, 0);
                break;

            /* Volume: [0x04][vol 0-100] */
            case CMD_VOLUME:
                if (got < 2) { ESP_LOGW(TAG, "CMD_VOLUME: short frame"); break; }
                cmd.volume = (data[1] > 100) ? 100 : data[1];
                ESP_LOGI(TAG, "CMD_VOLUME %u", cmd.volume);
                xQueueSend(s_cmd_queue, &cmd, 0);
                break;

            /* Set URL: [0x05][len][url...] */
            case CMD_SET_URL: {
                if (got < 3) { ESP_LOGW(TAG, "CMD_SET_URL: short frame"); break; }
                uint8_t url_len = data[1];
                if (url_len == 0 || got < (size_t)(2 + url_len)) {
                    ESP_LOGW(TAG, "CMD_SET_URL: bad url_len=%u got=%u", url_len, (unsigned)got);
                    break;
                }
                /* url_len is uint8_t so always fits in MAX_URL_LEN (255) */
                memcpy(cmd.url, &data[2], url_len);
                cmd.url[url_len] = '\0';
                ESP_LOGI(TAG, "CMD_SET_URL: %s", cmd.url);
                xQueueSend(s_cmd_queue, &cmd, 0);
                break;
            }

            /* Status hint (reply is handled by on_request) */
            case CMD_STATUS:
                ESP_LOGD(TAG, "CMD_STATUS hint received - reply on next master read");
                break;

            default:
                ESP_LOGW(TAG, "Unknown opcode 0x%02X", op);
                break;
        }
    }
}

/* Public API */

esp_err_t i2c_slave_ctrl_init(QueueHandle_t cmd_queue)
{
    s_cmd_queue = cmd_queue;

    s_status_mutex = xSemaphoreCreateMutex();
    if (!s_status_mutex) return ESP_ERR_NO_MEM;

    /* Stream buffer big enough for 4 max-size frames */
    s_stream = xStreamBufferCreate((size_t)(I2C_RX_BUF_SIZE + 1) * 4, 1);
    if (!s_stream) return ESP_ERR_NO_MEM;

    /* Configure I2C slave (V2 driver) */
    i2c_slave_config_t cfg = {
        .i2c_port         = I2C_SLAVE_PORT,
        .sda_io_num       = I2C_SLAVE_SDA_IO,
        .scl_io_num       = I2C_SLAVE_SCL_IO,
        .clk_source       = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth   = 32,                      /* TX ring buffer depth */
        .receive_buf_depth = (uint32_t)(I2C_RX_BUF_SIZE * 4), /* RX ring buffer */
        .slave_addr       = I2C_SLAVE_ADDR,
        .addr_bit_len     = I2C_ADDR_BIT_LEN_7,
        .intr_priority    = 0,
    };
    ESP_RETURN_ON_ERROR(i2c_new_slave_device(&cfg, &s_handle),
                        TAG, "i2c_new_slave_device failed");

    /* Register callbacks */
    i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_rx_cb,
        .on_request = i2c_request_cb,
    };
    ESP_RETURN_ON_ERROR(i2c_slave_register_event_callbacks(s_handle, &cbs, NULL),
                        TAG, "register_event_callbacks failed");

    /* Start parse task on core 0 (audio pipeline runs on core 1 by ADF default) */
    xTaskCreatePinnedToCore(i2c_parse_task, "i2c_parse", 4096, NULL,
                            configMAX_PRIORITIES - 2, NULL, 0);

    ESP_LOGI(TAG, "I2C slave ready  addr=0x%02X  SDA=GPIO%d  SCL=GPIO%d",
             I2C_SLAVE_ADDR, I2C_SLAVE_SDA_IO, I2C_SLAVE_SCL_IO);
    return ESP_OK;
}

void i2c_slave_ctrl_set_status(i2c_player_status_t status, uint8_t volume)
{
    /* Atomic byte writes - no mutex needed for reads in ISR */
    s_status_reply[0] = (uint8_t)status;
    s_status_reply[1] = volume;
}

