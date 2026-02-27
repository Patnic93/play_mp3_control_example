/*
 * I2C Slave Controller - implementation (ESP-IDF I2C Slave Driver V2)
 *
 * Receives command frames from a front-end I2C master and pushes parsed
 * i2c_command_t structs onto the audio-task command queue.
 *
 * ISR path  : i2c_rx_cb (on_receive) -> xStreamBufferSendFromISR
 * Task path : i2c_parse_task -> xQueueSend (cmd_queue)
 * Read path : i2c_request_cb (on_request) -> i2c_slave_write (status reply)
 */

#include "i2c_slave_ctrl.h"

#include "driver/gpio.h"
#include "driver/i2c_slave.h"

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <string.h>

static const char *TAG = "I2C_SLAVE";

/* Internal state */
static i2c_slave_dev_handle_t s_handle;
static QueueHandle_t s_cmd_queue;
static StreamBufferHandle_t s_stream;

/* Debug counters to verify bus activity */
static volatile uint32_t s_isr_write_count;
static volatile uint32_t s_isr_read_count;

/* Status reply bytes: [0]=player_status, [1]=volume */
static volatile uint8_t s_status_reply[2] = {PLAYER_STOPPED, 50};
static SemaphoreHandle_t s_status_mutex;

static void i2c_activity_task(void *arg);
static void i2c_parse_task(void *arg);

static bool i2c_request_cb(i2c_slave_dev_handle_t handle,
                           const i2c_slave_request_event_data_t *evt,
                           void *arg);

static bool i2c_rx_cb(i2c_slave_dev_handle_t handle,
                      const i2c_slave_rx_done_event_data_t *evt,
                      void *arg);

static void i2c_activity_task(void *arg)
{
    (void)arg;
    uint32_t last_writes = 0;
    uint32_t last_reads = 0;
    uint32_t idle_ticks = 0;

    for (;;) {
        uint32_t writes = s_isr_write_count;
        uint32_t reads = s_isr_read_count;
        if (writes != last_writes || reads != last_reads) {
            ESP_LOGI(TAG, "I2C activity: writes=%" PRIu32 " reads=%" PRIu32, writes, reads);
            last_writes = writes;
            last_reads = reads;
            idle_ticks = 0;
        } else {
            idle_ticks++;
            /* Every ~5s print a heartbeat + line levels to confirm wiring/pull-ups. */
            if (idle_ticks >= 25) {
                int sda = gpio_get_level(I2C_SLAVE_SDA_IO);
                int scl = gpio_get_level(I2C_SLAVE_SCL_IO);
                ESP_LOGI(TAG,
                         "No I2C traffic yet. GPIOs: SDA=%d(SDA=%d) SCL=%d(SCL=%d) (expect both 1 w/ pull-ups). addr=0x%02X port=%d",
                         I2C_SLAVE_SDA_IO,
                         sda,
                         I2C_SLAVE_SCL_IO,
                         scl,
                         I2C_SLAVE_ADDR,
                         (int)I2C_SLAVE_PORT);
                idle_ticks = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* on_request ISR: master wants to read -> send status */
static bool IRAM_ATTR i2c_request_cb(i2c_slave_dev_handle_t handle,
                                     const i2c_slave_request_event_data_t *evt,
                                     void *arg)
{
    (void)evt;
    (void)arg;
    BaseType_t woken = pdFALSE;

    s_isr_read_count++;

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
    (void)handle;
    (void)arg;
    BaseType_t woken = pdFALSE;

    s_isr_write_count++;

    uint8_t len = (uint8_t)((evt->length > I2C_RX_BUF_SIZE) ? I2C_RX_BUF_SIZE : evt->length);

    /* Write [len][data...] into stream buffer for parse task */
    (void)xStreamBufferSendFromISR(s_stream, &len, 1, &woken);
    (void)xStreamBufferSendFromISR(s_stream, evt->buffer, len, &woken);

    return woken == pdTRUE;
}

static void i2c_parse_task(void *arg)
{
    (void)arg;

    uint8_t len_byte;
    uint8_t data[I2C_RX_BUF_SIZE];

    for (;;) {
        /* Block until at least one byte arrives (the length prefix) */
        if (xStreamBufferReceive(s_stream, &len_byte, 1, portMAX_DELAY) != 1 || len_byte == 0) {
            continue;
        }

        /* Read the payload */
        size_t got = xStreamBufferReceive(s_stream, data, len_byte, pdMS_TO_TICKS(200));
        if (got != len_byte) {
            ESP_LOGW(TAG, "Incomplete frame: expected %u got %u", (unsigned)len_byte, (unsigned)got);
            continue;
        }

        /* Always log opcode so we can see traffic from the host */
        ESP_LOGI(TAG, "RX frame: len=%u opcode=0x%02X", (unsigned)got, (unsigned)data[0]);

        i2c_cmd_opcode_t op = (i2c_cmd_opcode_t)data[0];
        i2c_command_t cmd = {.opcode = op};

        switch (op) {
        case CMD_PLAY:
        case CMD_STOP:
        case CMD_PAUSE:
            (void)xQueueSend(s_cmd_queue, &cmd, 0);
            break;

        case CMD_VOLUME:
            if (got < 2) {
                ESP_LOGW(TAG, "CMD_VOLUME: short frame");
                break;
            }
            cmd.volume = (data[1] > 100) ? 100 : data[1];
            (void)xQueueSend(s_cmd_queue, &cmd, 0);
            break;

        case CMD_SET_URL: {
            if (got < 3) {
                ESP_LOGW(TAG, "CMD_SET_URL: short frame");
                break;
            }
            uint8_t url_len = data[1];
            if (url_len == 0 || got < (size_t)(2 + url_len)) {
                ESP_LOGW(TAG, "CMD_SET_URL: bad url_len=%u got=%u", url_len, (unsigned)got);
                break;
            }
            memcpy(cmd.url, &data[2], url_len);
            cmd.url[url_len] = '\0';
            (void)xQueueSend(s_cmd_queue, &cmd, 0);
            break;
        }

        case CMD_STATUS:
            /* Status reply is handled by on_request callback */
            break;

        default:
            ESP_LOGW(TAG, "Unknown opcode 0x%02X", (unsigned)op);
            break;
        }
    }
}

esp_err_t i2c_slave_ctrl_init(QueueHandle_t cmd_queue)
{
    if (!cmd_queue) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cmd_queue = cmd_queue;

    /* Ensure pull-ups are enabled (external 4.7k preferred; internal helps for quick bring-up).
     * Note: the I2C driver may reconfigure GPIOs, so we also set this again after init. */
    gpio_set_pull_mode(I2C_SLAVE_SDA_IO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(I2C_SLAVE_SCL_IO, GPIO_PULLUP_ONLY);

    s_status_mutex = xSemaphoreCreateMutex();
    if (!s_status_mutex) {
        return ESP_ERR_NO_MEM;
    }

    /* Stream buffer big enough for 4 max-size frames */
    s_stream = xStreamBufferCreate((size_t)(I2C_RX_BUF_SIZE + 1) * 4, 1);
    if (!s_stream) {
        return ESP_ERR_NO_MEM;
    }

    /* Configure I2C slave (V2 driver) */
    i2c_slave_config_t cfg = {
        .i2c_port = I2C_SLAVE_PORT,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth = 32,                           /* TX ring buffer depth */
        .receive_buf_depth = (uint32_t)(I2C_RX_BUF_SIZE * 4), /* RX ring buffer */
        .slave_addr = I2C_SLAVE_ADDR,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .intr_priority = 0,
    };

    ESP_RETURN_ON_ERROR(i2c_new_slave_device(&cfg, &s_handle), TAG, "i2c_new_slave_device failed");

    /* Re-apply pull-ups after driver GPIO config (important if there are no external pull-ups). */
    gpio_set_pull_mode(I2C_SLAVE_SDA_IO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(I2C_SLAVE_SCL_IO, GPIO_PULLUP_ONLY);

    int sda0 = gpio_get_level(I2C_SLAVE_SDA_IO);
    int scl0 = gpio_get_level(I2C_SLAVE_SCL_IO);
    ESP_LOGI(TAG, "Line levels right after init: SDA=%d SCL=%d (expect both 1)", sda0, scl0);

    i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_rx_cb,
        .on_request = i2c_request_cb,
    };
    ESP_RETURN_ON_ERROR(i2c_slave_register_event_callbacks(s_handle, &cbs, NULL),
                        TAG,
                        "i2c_slave_register_event_callbacks failed");

    if (xTaskCreate(i2c_parse_task, "i2c_parse", 4096, NULL, 10, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    /* Optional debug task: prints a heartbeat until I2C traffic appears */
    if (xTaskCreate(i2c_activity_task, "i2c_act", 2048, NULL, 1, NULL) != pdPASS) {
        ESP_LOGW(TAG, "Failed to create i2c_act task");
    }

    ESP_LOGI(TAG, "I2C slave ready: addr=0x%02X port=%d SDA=%d SCL=%d",
             I2C_SLAVE_ADDR,
             (int)I2C_SLAVE_PORT,
             I2C_SLAVE_SDA_IO,
             I2C_SLAVE_SCL_IO);

    return ESP_OK;
}

void i2c_slave_ctrl_set_status(i2c_player_status_t status, uint8_t volume)
{
    if (volume > 100) {
        volume = 100;
    }

    if (s_status_mutex) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status_reply[0] = (uint8_t)status;
        s_status_reply[1] = volume;
        xSemaphoreGive(s_status_mutex);
    } else {
        s_status_reply[0] = (uint8_t)status;
        s_status_reply[1] = volume;
    }
}
