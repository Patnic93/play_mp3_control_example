/* Play internet radio stream via audio pipeline
   NPO Radio 2 - https://icecast.omroep.nl/radio2-bb-mp3

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "i2c_slave_ctrl.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "http_stream.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"
#include "esp_peripherals.h"
#include "periph_touch.h"
#include "periph_adc_button.h"
#include "periph_button.h"
#include "board.h"

#define WIFI_SSID      "PATNIC-IOT"
#define WIFI_PASSWORD  "Parelmoer_4904@50"
#define RADIO_STREAM_URL "http://icecast.omroep.nl/radio2-bb-mp3"

static const char *TAG = "NPO_RADIO2_STREAM";

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    static int retry_count = 0;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < 10) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGW(TAG, "Retrying WiFi connection (%d/10)", retry_count);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi: %s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Failed to connect to WiFi: %s", WIFI_SSID);
    }
}

/* ── I2C command queue (shared between i2c_slave_ctrl and app_main) ── */
static QueueHandle_t s_i2c_cmd_queue;

void app_main(void)
{
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t http_stream_reader, i2s_stream_writer, mp3_decoder;

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "[ 0 ] Initialize NVS, WiFi and I2C slave");
    /* Create I2C command queue before init so slave can start receiving */
    s_i2c_cmd_queue = xQueueCreate(8, sizeof(i2c_command_t));
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_sta();

    /* Start I2C slave – front-end can now send commands at any time */
    ESP_ERROR_CHECK(i2c_slave_ctrl_init(s_i2c_cmd_queue));

    ESP_LOGI(TAG, "[ 1 ] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    int player_volume;
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);

    ESP_LOGI(TAG, "[ 2 ] Create audio pipeline, add all elements to pipeline, and subscribe pipeline event");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[2.1] Create http stream to read from radio URL");
    http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
    http_cfg.type = AUDIO_STREAM_READER;
    http_cfg.enable_playlist_parser = true;
    http_stream_reader = http_stream_init(&http_cfg);

    ESP_LOGI(TAG, "[2.2] Create mp3 decoder");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_decoder = mp3_decoder_init(&mp3_cfg);

    ESP_LOGI(TAG, "[2.3] Create i2s stream to write data to codec chip");
#if defined CONFIG_ESP32_C3_LYRA_V2_BOARD
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_PDM_TX_CFG_DEFAULT();
#else
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
#endif
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[2.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, http_stream_reader, "http");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG, "[2.5] Link it together [http_stream]-->mp3_decoder-->i2s_stream-->[codec_chip]");
    const char *link_tag[3] = {"http", "mp3", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

    ESP_LOGI(TAG, "[2.6] Set radio stream URL");
    audio_element_set_uri(http_stream_reader, RADIO_STREAM_URL);

    ESP_LOGI(TAG, "[ 3 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[3.1] Initialize keys on board");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGW(TAG, "[ 5 ] Tap touch buttons to control music player:");
    ESP_LOGW(TAG, "      [Play] to start, pause and resume, [Set] to stop.");
    ESP_LOGW(TAG, "      [Vol-] or [Vol+] to adjust volume.");

    ESP_LOGI(TAG, "[ 5.1 ] Start audio_pipeline - streaming NPO Radio 2");
    i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
    audio_pipeline_run(pipeline);

    while (1) {
        /* ── Check I2C command queue first (non-blocking) ── */
        i2c_command_t i2c_cmd;
        while (xQueueReceive(s_i2c_cmd_queue, &i2c_cmd, 0) == pdTRUE) {
            switch (i2c_cmd.opcode) {
                case CMD_PLAY: {
                    audio_element_state_t st = audio_element_get_state(i2s_stream_writer);
                    if (st == AEL_STATE_PAUSED) {
                        audio_pipeline_resume(pipeline);
                        i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        ESP_LOGI(TAG, "[I2C] PLAY – resumed");
                    } else if (st == AEL_STATE_INIT || st == AEL_STATE_FINISHED) {
                        audio_pipeline_run(pipeline);
                        i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        ESP_LOGI(TAG, "[I2C] PLAY – started");
                    }
                    break;
                }
                case CMD_STOP:
                    audio_pipeline_stop(pipeline);
                    audio_pipeline_wait_for_stop(pipeline);
                    i2c_slave_ctrl_set_status(PLAYER_STOPPED, (uint8_t)player_volume);
                    ESP_LOGI(TAG, "[I2C] STOP");
                    break;

                case CMD_PAUSE:
                    audio_pipeline_pause(pipeline);
                    i2c_slave_ctrl_set_status(PLAYER_PAUSED, (uint8_t)player_volume);
                    ESP_LOGI(TAG, "[I2C] PAUSE");
                    break;

                case CMD_VOLUME:
                    player_volume = i2c_cmd.volume;
                    audio_hal_set_volume(board_handle->audio_hal, player_volume);
                    i2c_slave_ctrl_set_status(
                        (audio_element_get_state(i2s_stream_writer) == AEL_STATE_RUNNING)
                            ? PLAYER_PLAYING : PLAYER_STOPPED,
                        (uint8_t)player_volume);
                    ESP_LOGI(TAG, "[I2C] VOLUME %d", player_volume);
                    break;

                case CMD_SET_URL:
                    ESP_LOGI(TAG, "[I2C] SET_URL %s", i2c_cmd.url);
                    audio_pipeline_stop(pipeline);
                    audio_pipeline_wait_for_stop(pipeline);
                    audio_pipeline_terminate(pipeline);
                    audio_pipeline_reset_ringbuffer(pipeline);
                    audio_pipeline_reset_elements(pipeline);
                    audio_element_set_uri(http_stream_reader, i2c_cmd.url);
                    audio_pipeline_run(pipeline);
                    i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                    break;

                default:
                    break;
            }
        }

        /* ── Wait for audio pipeline / peripheral event (50 ms timeout) ── */
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, pdMS_TO_TICKS(50));
        if (ret != ESP_OK) {
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) mp3_decoder
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {

            audio_element_info_t music_info = {0};
            audio_element_getinfo(mp3_decoder, &music_info);
            ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
            continue;
        }

        if ((msg.source_type == PERIPH_ID_TOUCH || msg.source_type == PERIPH_ID_BUTTON || msg.source_type == PERIPH_ID_ADC_BTN)
            && (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_PRESSED || msg.cmd == PERIPH_ADC_BUTTON_PRESSED)) {
            if ((int) msg.data == get_input_play_id()) {
                ESP_LOGI(TAG, "[ * ] [Play] touch tap event");
                audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
                switch (el_state) {
                    case AEL_STATE_INIT :
                        ESP_LOGI(TAG, "[ * ] Starting audio pipeline");
                        audio_pipeline_run(pipeline);
                        i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        break;
                    case AEL_STATE_RUNNING :
                        ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
                        audio_pipeline_pause(pipeline);
                        i2c_slave_ctrl_set_status(PLAYER_PAUSED, (uint8_t)player_volume);
                        break;
                    case AEL_STATE_PAUSED :
                        ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
                        audio_pipeline_resume(pipeline);
                        i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        break;
                    case AEL_STATE_FINISHED :
                        ESP_LOGI(TAG, "[ * ] Restarting radio stream");
                        audio_pipeline_reset_ringbuffer(pipeline);
                        audio_pipeline_reset_elements(pipeline);
                        audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
                        audio_element_set_uri(http_stream_reader, RADIO_STREAM_URL);
                        audio_pipeline_run(pipeline);
                        i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        break;
                    default :
                        ESP_LOGI(TAG, "[ * ] Not supported state %d", el_state);
                }
            } else if ((int) msg.data == get_input_set_id()) {
                ESP_LOGI(TAG, "[ * ] [Set] touch tap event");
                ESP_LOGI(TAG, "[ * ] Stopping audio pipeline");
                i2c_slave_ctrl_set_status(PLAYER_STOPPED, (uint8_t)player_volume);
                break;
            } else if ((int) msg.data == get_input_mode_id()) {
                ESP_LOGI(TAG, "[ * ] [mode] tap event - restart stream");
                audio_pipeline_stop(pipeline);
                audio_pipeline_wait_for_stop(pipeline);
                audio_pipeline_terminate(pipeline);
                audio_pipeline_reset_ringbuffer(pipeline);
                audio_pipeline_reset_elements(pipeline);
                audio_element_set_uri(http_stream_reader, RADIO_STREAM_URL);
                audio_pipeline_run(pipeline);
                i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
            } else if ((int) msg.data == get_input_volup_id()) {
                ESP_LOGI(TAG, "[ * ] [Vol+] touch tap event");
                player_volume += 10;
                if (player_volume > 100) {
                    player_volume = 100;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
            } else if ((int) msg.data == get_input_voldown_id()) {
                ESP_LOGI(TAG, "[ * ] [Vol-] touch tap event");
                player_volume -= 10;
                if (player_volume < 0) {
                    player_volume = 0;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                i2c_slave_ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
            }
        }
    }

    ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);
    audio_pipeline_unregister(pipeline, http_stream_reader);
    audio_pipeline_unregister(pipeline, mp3_decoder);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Make sure audio_pipeline_remove_listener is called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(http_stream_reader);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(mp3_decoder);
}
