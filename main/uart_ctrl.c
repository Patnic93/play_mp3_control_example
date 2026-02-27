/*
 * UART Control - receive-only command interface
 *
 * This provides a simple, robust alternative to I2C when only JP7/JTAG pins are accessible
 * or when the I2C bus is unreliable due to board pin muxing.
 */

#include "uart_ctrl.h"

#include "i2c_slave_ctrl.h" /* reuse i2c_command_t + opcodes */

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "UART_CTRL";

#include "sdkconfig.h"

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

#if defined(CONFIG_PLAY_MP3_UART_RX_GPIO)
#define UART_CTRL_RX_GPIO        (CONFIG_PLAY_MP3_UART_RX_GPIO)
#else
#define UART_CTRL_RX_GPIO        (14)
#endif

#if defined(CONFIG_PLAY_MP3_UART_FRAME_TIMEOUT_MS)
#define UART_CTRL_FRAME_TIMEOUT_MS (CONFIG_PLAY_MP3_UART_FRAME_TIMEOUT_MS)
#else
#define UART_CTRL_FRAME_TIMEOUT_MS (50)
#endif

static QueueHandle_t s_cmd_queue;

static void uart_rx_task(void *arg);

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
        if (url_len > MAX_URL_LEN) {
            url_len = MAX_URL_LEN;
        }
        memcpy(cmd.url, &data[2], url_len);
        cmd.url[url_len] = '\0';
        (void)xQueueSend(s_cmd_queue, &cmd, 0);
        break;
    }

    case CMD_STATUS:
        /* No reply channel in 1-wire RX mode. */
        ESP_LOGI(TAG, "CMD_STATUS received (ignored in UART RX mode)");
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

    /*
     * Force-release GPIO14 from JTAG (TMS) or any other peripheral that might
     * still claim it after boot.  Without this, uart_set_pin() may succeed but
     * the IO-MUX remains routed to JTAG and we never see RX data.
     */
    gpio_reset_pin((gpio_num_t)UART_CTRL_RX_GPIO);
    gpio_set_direction((gpio_num_t)UART_CTRL_RX_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)UART_CTRL_RX_GPIO, GPIO_PULLUP_ONLY);



    uart_config_t cfg = {
        .baud_rate = UART_CTRL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(UART_CTRL_PORT, 2048, 0, 0, NULL, 0), TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_CTRL_PORT, &cfg), TAG, "uart_param_config failed");
    /* RX only: TX pin disabled (-1). */
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_CTRL_PORT, UART_PIN_NO_CHANGE, UART_CTRL_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG,
                        "uart_set_pin failed");

    ESP_LOGW(TAG, "UART control ready: port=%d baud=%d RX_GPIO=%d", (int)UART_CTRL_PORT, UART_CTRL_BAUD, UART_CTRL_RX_GPIO);

    xTaskCreate(uart_rx_task, "uart_ctrl_rx", 4096, NULL, 10, NULL);
    return ESP_OK;
}

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

        ESP_LOGW(TAG, "RX frame: len=%u opcode=0x%02X", (unsigned)want, (unsigned)buf[0]);
        parse_and_enqueue(buf, want);
    }
}
