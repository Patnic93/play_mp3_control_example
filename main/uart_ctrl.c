/*
 * UART Control - bidirectional command interface
 *
 * Protocol (binary, length-prefixed):
 *   [len:uint8][payload:len bytes]
 * Where payload matches the same opcode frames used by the I2C control interface.
 *
 * Reply (device -> host):
 *   On CMD_STATUS, device sends: [len=3][0x06][status:uint8][volume:uint8]
 */

#include "uart_ctrl.h"

#include "i2c_slave_ctrl.h" /* reuse i2c_command_t + opcodes */

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#include "freertos/semphr.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "UART_CTRL";

#include "sdkconfig.h"

// Bring-up helper: periodically transmit a STATUS frame so you can identify the UART TX pin
// with a logic analyzer / USB-TTL adapter even if the host isn't sending commands yet.
// Set to 0 to disable.
#ifndef UART_CTRL_STATUS_BEACON_ENABLED
#define UART_CTRL_STATUS_BEACON_ENABLED 0
#endif

// Set to 1 for verbose UART logs during wiring/debug.
#ifndef UART_CTRL_VERBOSE_LOGS
#define UART_CTRL_VERBOSE_LOGS 0
#endif

#if defined(CONFIG_PLAY_MP3_UART_PORT)
#define UART_CTRL_PORT           ((uart_port_t)CONFIG_PLAY_MP3_UART_PORT)
#else
#define UART_CTRL_PORT           UART_NUM_1
#endif

#if defined(CONFIG_PLAY_MP3_UART_BAUD)
#define UART_CTRL_BAUD           (CONFIG_PLAY_MP3_UART_BAUD)
#else
#define UART_CTRL_BAUD           (115200)
#endif

#if defined(CONFIG_PLAY_MP3_UART_TX_GPIO)
#define UART_CTRL_TX_GPIO        (CONFIG_PLAY_MP3_UART_TX_GPIO)
#else
#define UART_CTRL_TX_GPIO        (13)
#endif

#if defined(CONFIG_PLAY_MP3_UART_RX_GPIO)
#define UART_CTRL_RX_GPIO        (CONFIG_PLAY_MP3_UART_RX_GPIO)
#else
#define UART_CTRL_RX_GPIO        (15)
#endif

#if defined(CONFIG_PLAY_MP3_UART_FRAME_TIMEOUT_MS)
#define UART_CTRL_FRAME_TIMEOUT_MS (CONFIG_PLAY_MP3_UART_FRAME_TIMEOUT_MS)
#else
#define UART_CTRL_FRAME_TIMEOUT_MS (50)
#endif

static QueueHandle_t s_cmd_queue;
static SemaphoreHandle_t s_tx_mutex;

/* Status reply bytes: [0]=player_status, [1]=volume */
static volatile uint8_t s_status_reply[2] = {PLAYER_STOPPED, 50};

static void uart_rx_task(void *arg);
#if UART_CTRL_STATUS_BEACON_ENABLED
static void uart_status_beacon_task(void *arg);
#endif

static esp_err_t uart_ctrl_send_frame(const uint8_t *payload, size_t len)
{
    if (!payload || len == 0 || len > 255) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_tx_mutex) {
        xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    }

    uint8_t len_byte = (uint8_t)len;
    int w0 = uart_write_bytes(UART_CTRL_PORT, (const char *)&len_byte, 1);
    int w1 = uart_write_bytes(UART_CTRL_PORT, (const char *)payload, len);

    if (s_tx_mutex) {
        xSemaphoreGive(s_tx_mutex);
    }

    if (w0 != 1 || w1 != (int)len) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void uart_ctrl_set_status(uint8_t status, uint8_t volume)
{
    s_status_reply[0] = status;
    s_status_reply[1] = volume;
}

static void parse_and_enqueue(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }

    i2c_cmd_opcode_t op = (i2c_cmd_opcode_t)data[0];
    i2c_command_t cmd = {.opcode = op};

    switch (op) {
    case CMD_PLAY:
    case CMD_STOP:
    case CMD_PAUSE:
        (void)xQueueSend(s_cmd_queue, &cmd, 0);
        break;

    case CMD_VOLUME:
        if (len < 2) {
            ESP_LOGW(TAG, "CMD_VOLUME: short frame");
            break;
        }
        cmd.volume = (data[1] > 100) ? 100 : data[1];
        (void)xQueueSend(s_cmd_queue, &cmd, 0);
        break;

    case CMD_SET_URL: {
        if (len < 3) {
            ESP_LOGW(TAG, "CMD_SET_URL: short frame");
            break;
        }
        uint8_t url_len = data[1];
        if (url_len == 0 || len < (size_t)(2 + url_len)) {
            ESP_LOGW(TAG, "CMD_SET_URL: bad url_len=%u len=%u", (unsigned)url_len, (unsigned)len);
            break;
        }
        memcpy(cmd.url, &data[2], url_len);
        cmd.url[url_len] = '\0';
        (void)xQueueSend(s_cmd_queue, &cmd, 0);
        break;
    }

    case CMD_STATUS:
        {
            uint8_t reply[3] = {CMD_STATUS, s_status_reply[0], s_status_reply[1]};
            ESP_LOGD(TAG,
                     "CMD_STATUS rx -> reply: status=%u volume=%u (TX_GPIO=%d RX_GPIO=%d)",
                     (unsigned)reply[1],
                     (unsigned)reply[2],
                     (int)UART_CTRL_TX_GPIO,
                     (int)UART_CTRL_RX_GPIO);
            esp_err_t tx = uart_ctrl_send_frame(reply, sizeof(reply));
            if (tx != ESP_OK) {
                ESP_LOGD(TAG, "CMD_STATUS reply send failed: %s", esp_err_to_name(tx));
            }
        }
        break;

    default:
        ESP_LOGW(TAG, "Unknown opcode 0x%02X", (unsigned)op);
        break;
    }
}

esp_err_t uart_ctrl_init(QueueHandle_t cmd_queue)
{
    if (!cmd_queue) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cmd_queue = cmd_queue;

    ESP_LOGW(TAG, "UART control init starting...");

    if (!s_tx_mutex) {
        s_tx_mutex = xSemaphoreCreateMutex();
        if (!s_tx_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }

    /* Force-release RX pin from JTAG/other muxing so we can receive data. */
    gpio_reset_pin((gpio_num_t)UART_CTRL_RX_GPIO);
    gpio_set_direction((gpio_num_t)UART_CTRL_RX_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)UART_CTRL_RX_GPIO, GPIO_PULLUP_ONLY);

    gpio_reset_pin((gpio_num_t)UART_CTRL_TX_GPIO);
    gpio_set_direction((gpio_num_t)UART_CTRL_TX_GPIO, GPIO_MODE_OUTPUT);



    uart_config_t cfg = {
        .baud_rate = UART_CTRL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(UART_CTRL_PORT, 2048, 1024, 0, NULL, 0), TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_CTRL_PORT, &cfg), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_CTRL_PORT, UART_CTRL_TX_GPIO, UART_CTRL_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG,
                        "uart_set_pin failed");

    ESP_LOGW(TAG,
             "UART control ready: port=%d baud=%d TX_GPIO=%d RX_GPIO=%d",
             (int)UART_CTRL_PORT,
             UART_CTRL_BAUD,
             UART_CTRL_TX_GPIO,
             UART_CTRL_RX_GPIO);

    xTaskCreate(uart_rx_task, "uart_ctrl_rx", 4096, NULL, 10, NULL);

#if UART_CTRL_STATUS_BEACON_ENABLED
    xTaskCreate(uart_status_beacon_task, "uart_status_beacon", 2048, NULL, 5, NULL);
    ESP_LOGW(TAG, "UART debug STATUS beacon enabled (1 Hz)");
#endif
    return ESP_OK;
}

#if UART_CTRL_STATUS_BEACON_ENABLED
static void uart_status_beacon_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint8_t reply[3] = {CMD_STATUS, s_status_reply[0], s_status_reply[1]};
        (void)uart_ctrl_send_frame(reply, sizeof(reply));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

static void uart_rx_task(void *arg)
{
    (void)arg;

    uint8_t len_byte = 0;
    uint8_t buf[I2C_RX_BUF_SIZE];

    for (;;) {
        int n = uart_read_bytes(UART_CTRL_PORT, &len_byte, 1, portMAX_DELAY);
        if (n != 1) {
            continue;
        }
        if (len_byte == 0) {
            continue;
        }

        size_t want = len_byte;
        if (want > sizeof(buf)) {
            /* Drain and resync. */
            ESP_LOGW(TAG, "Frame too long (%u), draining", (unsigned)want);
            uint8_t drain[32];
            size_t remaining = want;
            while (remaining > 0) {
                int d = uart_read_bytes(UART_CTRL_PORT, drain, (remaining > sizeof(drain)) ? sizeof(drain) : remaining,
                                        pdMS_TO_TICKS(UART_CTRL_FRAME_TIMEOUT_MS));
                if (d <= 0) {
                    break;
                }
                remaining -= (size_t)d;
            }
            continue;
        }

        int got = uart_read_bytes(UART_CTRL_PORT, buf, want, pdMS_TO_TICKS(UART_CTRL_FRAME_TIMEOUT_MS));
        if (got != (int)want) {
            ESP_LOGW(TAG, "Incomplete frame: expected %u got %d", (unsigned)want, got);
            continue;
        }

        if (UART_CTRL_VERBOSE_LOGS) {
            ESP_LOGW(TAG, "RX frame: len=%u opcode=0x%02X", (unsigned)want, (unsigned)buf[0]);
        } else {
            ESP_LOGD(TAG, "RX frame: len=%u opcode=0x%02X", (unsigned)want, (unsigned)buf[0]);
        }
        parse_and_enqueue(buf, want);
    }
}
