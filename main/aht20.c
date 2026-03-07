/*
 * AHT20 temperature & humidity sensor — I2C master driver
 *
 * Datasheet: https://asairsensors.com/wp-content/uploads/2021/09/Data-Sheet-AHT20-ASAIR-V1.0.03.pdf
 *
 * I2C address : 0x38 (fixed)
 * Bus         : I2C_NUM_1, GPIO 21 (SDA) / GPIO 22 (SCL)
 *
 * Startup sequence
 *   1. Power-on delay ≥ 20 ms
 *   2. Read status byte; if bit 3 (CAL) = 0 → send initialise command 0xBE 0x08 0x00
 *   3. Loop every 2 s:
 *      a. Trigger measurement 0xAC 0x33 0x00
 *      b. Wait ≥ 80 ms
 *      c. Read 6 bytes → parse humidity + temperature
 *      d. Log to serial
 */

#include "aht20.h"

#include "esp_check.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* Forward declaration: uart_ctrl_send_sensor_data is defined in uart_ctrl.c.
 * We include the header here to avoid a circular dependency via i2c_slave_ctrl.h. */
extern void uart_ctrl_send_sensor_data(float temp_c, float hum_pct);

#define AHT20_I2C_PORT      I2C_NUM_0
#define AHT20_SDA_GPIO      18
#define AHT20_SCL_GPIO      23
#define AHT20_I2C_ADDR      0x38
#define AHT20_I2C_FREQ_HZ   100000

#define AHT20_CMD_INIT      0xBE
#define AHT20_CMD_TRIGGER   0xAC
#define AHT20_CMD_SOFTRESET 0xBA
#define AHT20_STATUS_BUSY   (1 << 7)
#define AHT20_STATUS_CAL    (1 << 3)

static const char *TAG = "AHT20";

/* Laatste geldige meting (beschermd door mutex) */
static float s_last_temp = 0.0f;
static float s_last_hum  = 0.0f;
static bool  s_has_data  = false;
static SemaphoreHandle_t s_data_mutex;

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

/* Write bytes to the sensor */
static esp_err_t aht20_write(const uint8_t *data, size_t len)
{
    return i2c_master_transmit(s_dev, data, len, pdMS_TO_TICKS(100));
}

/* Read bytes from the sensor */
static esp_err_t aht20_read(uint8_t *data, size_t len)
{
    return i2c_master_receive(s_dev, data, len, pdMS_TO_TICKS(100));
}

/* Initialise the sensor (calibration check + init command if needed) */
static esp_err_t aht20_init_sensor(void)
{
    vTaskDelay(pdMS_TO_TICKS(40));  /* ≥20 ms after power-on */

    uint8_t status = 0;
    esp_err_t err = aht20_read(&status, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor niet gevonden op 0x%02X (SDA=GPIO%d, SCL=GPIO%d): %s",
                 AHT20_I2C_ADDR, AHT20_SDA_GPIO, AHT20_SCL_GPIO, esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }

    if ((status & AHT20_STATUS_CAL) == 0) {
        ESP_LOGI(TAG, "Kalibratie bit niet gezet, initialiseer sensor...");
        uint8_t init_cmd[] = {AHT20_CMD_INIT, 0x08, 0x00};
        ESP_RETURN_ON_ERROR(aht20_write(init_cmd, sizeof(init_cmd)), TAG, "Init command mislukt");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "AHT20 gevonden en gereed. Status=0x%02X", status);
    return ESP_OK;
}

/* Read one measurement; stores results in out-params (can be NULL) */
static esp_err_t aht20_measure(float *temp_c, float *humidity_pct)
{
    /* Trigger */
    uint8_t trig[] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
    ESP_RETURN_ON_ERROR(aht20_write(trig, sizeof(trig)), TAG, "Trigger mislukt");

    /* Wait for conversion (≥80 ms; poll busy bit) */
    uint8_t status = AHT20_STATUS_BUSY;
    for (int i = 0; i < 10 && (status & AHT20_STATUS_BUSY); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (aht20_read(&status, 1) != ESP_OK) {
            return ESP_FAIL;
        }
    }
    if (status & AHT20_STATUS_BUSY) {
        ESP_LOGW(TAG, "Sensor blijft busy na 100 ms");
        return ESP_ERR_TIMEOUT;
    }

    /* Read 6 data bytes */
    uint8_t buf[6] = {0};
    ESP_RETURN_ON_ERROR(aht20_read(buf, sizeof(buf)), TAG, "Lezen mislukt");

    /* Parse 20-bit raw values */
    uint32_t raw_hum  = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t raw_temp = ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    if (humidity_pct) {
        *humidity_pct = (float)raw_hum  * 100.0f / 1048576.0f;
    }
    if (temp_c) {
        *temp_c = (float)raw_temp * 200.0f / 1048576.0f - 50.0f;
    }
    return ESP_OK;
}

/* Background task: leest elke 2 seconden, logt naar serial.
 * Stuurt de eerste meting direct unsolicited via UART naar de Waveshare,
 * en daarna elke 60 seconden — timing-onafhankelijk van de Waveshare boot. */
static void aht20_task(void *arg)
{
    (void)arg;
    bool first_push_done = false;
    unsigned long last_push_s = 0;
    unsigned long uptime_s = 0;

    for (;;) {
        float temp = 0.0f, hum = 0.0f;
        esp_err_t err = aht20_measure(&temp, &hum);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Temp: %.1f °C   Vochtigheid: %.1f %%", temp, hum);
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            s_last_temp = temp;
            s_last_hum  = hum;
            s_has_data  = true;
            xSemaphoreGive(s_data_mutex);

            /* Stuur direct als dit de eerste geldige meting is,
             * of als er 60 seconden zijn verstreken sinds de laatste push. */
            bool do_push = !first_push_done || (uptime_s - last_push_s >= 60);
            if (do_push) {
                uart_ctrl_send_sensor_data(temp, hum);
                ESP_LOGI(TAG, "Sensor data verstuurd via UART (unsolicited)");
                first_push_done = true;
                last_push_s = uptime_s;
            }
        } else {
            ESP_LOGW(TAG, "Meting mislukt: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
        uptime_s += 2;
    }
}

esp_err_t aht20_get_last(float *temp_c, float *hum_pct)
{
    if (!s_data_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    bool ok = s_has_data;
    if (ok) {
        if (temp_c)  *temp_c  = s_last_temp;
        if (hum_pct) *hum_pct = s_last_hum;
    }
    xSemaphoreGive(s_data_mutex);
    return ok ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t aht20_start(void)
{
    /* Probeer de bestaande bus handle te hergebruiken (ADF init die bus vóór ons).
     * Als die er nog niet is (geen ADF i2c init), maken we hem zelf aan. */
    esp_err_t err = i2c_master_get_bus_handle(AHT20_I2C_PORT, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Geen bestaande I2C bus gevonden op poort %d, nieuwe bus aanmaken.", AHT20_I2C_PORT);
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port            = AHT20_I2C_PORT,
            .sda_io_num          = AHT20_SDA_GPIO,
            .scl_io_num          = AHT20_SCL_GPIO,
            .clk_source          = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt   = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "Bus aanmaken mislukt");
    } else {
        ESP_LOGI(TAG, "Bestaande I2C bus hergebruikt (poort %d, SDA=%d, SCL=%d).",
                 AHT20_I2C_PORT, AHT20_SDA_GPIO, AHT20_SCL_GPIO);
    }

    /* Voeg AHT20 device toe */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AHT20_I2C_ADDR,
        .scl_speed_hz    = AHT20_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev), TAG, "Device toevoegen mislukt");

    /* Initialiseer sensor */
    err = aht20_init_sensor();
    if (err != ESP_OK) {
        return err;
    }

    /* Start lees-taak */
    s_data_mutex = xSemaphoreCreateMutex();
    if (!s_data_mutex) {
        return ESP_ERR_NO_MEM;
    }
    xTaskCreate(aht20_task, "aht20", 4096, NULL, 5, NULL);
    return ESP_OK;
}
