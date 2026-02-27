/* Play internet radio stream via audio pipeline
   NPO Radio 2 - https://icecast.omroep.nl/radio2-bb-mp3

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "i2c_slave_ctrl.h"
#include "uart_ctrl.h"
#include "sdkconfig.h"
#include "esp_http_server.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
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

#define RADIO_STREAM_URL "http://icecast.omroep.nl/radio2-bb-mp3"

static const char *TAG = "NPO_RADIO2_STREAM";

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_PORTAL_CRED_BIT BIT2

static httpd_handle_t s_portal_httpd;
static TaskHandle_t s_dns_task;
static volatile bool s_dns_task_stop;
static wifi_config_t s_portal_sta_cfg;
static int s_wifi_retry_count;

static void url_decode_inplace(char *s)
{
    if (!s) {
        return;
    }
    char *src = s;
    char *dst = s;
    while (*src) {
        if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else if (*src == '%' && src[1] && src[2]) {
            char hex[3] = {src[1], src[2], 0};
            *dst++ = (char)strtol(hex, NULL, 16);
            src += 3;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = 0;
}

static bool form_get_value(char *body, const char *key, char *out, size_t out_len)
{
    if (!body || !key || !out || out_len == 0) {
        return false;
    }
    out[0] = 0;

    size_t key_len = strlen(key);
    char *p = body;
    while (p && *p) {
        char *amp = strchr(p, '&');
        if (amp) {
            *amp = 0;
        }
        char *eq = strchr(p, '=');
        if (eq) {
            *eq = 0;
            const char *k = p;
            char *v = eq + 1;
            if (strlen(k) == key_len && strcmp(k, key) == 0) {
                url_decode_inplace(v);
                strlcpy(out, v, out_len);
                *eq = '=';
                if (amp) {
                    *amp = '&';
                }
                return true;
            }
            *eq = '=';
        }
        if (amp) {
            *amp = '&';
            p = amp + 1;
        } else {
            break;
        }
    }
    return false;
}

static void dns_server_task(void *arg)
{
    uint32_t ip = (uint32_t)(uintptr_t)arg; /* network byte order */

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS socket create failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "DNS bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    struct timeval tv = {.tv_sec = 0, .tv_usec = 200 * 1000};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t rx[256];
    uint8_t tx[256];

    while (!s_dns_task_stop) {
        struct sockaddr_in from = {0};
        socklen_t fromlen = sizeof(from);
        int len = recvfrom(sock, rx, sizeof(rx), 0, (struct sockaddr *)&from, &fromlen);
        if (len <= 0) {
            continue;
        }
        if (len < 12) {
            continue;
        }

        /* Find end of question section */
        int q = 12;
        while (q < len && rx[q] != 0) {
            q += rx[q] + 1;
        }
        q += 1; /* zero */
        if (q + 4 > len) {
            continue;
        }
        int question_len = q + 4;

        memcpy(tx, rx, (size_t)question_len);

        /* Flags: response + recursion available */
        tx[2] = 0x81;
        tx[3] = 0x80;
        /* QDCOUNT unchanged, ANCOUNT = 1 */
        tx[6] = 0x00;
        tx[7] = 0x01;
        /* NSCOUNT, ARCOUNT = 0 */
        tx[8] = 0x00;
        tx[9] = 0x00;
        tx[10] = 0x00;
        tx[11] = 0x00;

        int a = question_len;
        /* NAME: pointer to QNAME at offset 12 */
        tx[a++] = 0xC0;
        tx[a++] = 0x0C;
        /* TYPE A */
        tx[a++] = 0x00;
        tx[a++] = 0x01;
        /* CLASS IN */
        tx[a++] = 0x00;
        tx[a++] = 0x01;
        /* TTL */
        tx[a++] = 0x00;
        tx[a++] = 0x00;
        tx[a++] = 0x00;
        tx[a++] = 0x00;
        /* RDLENGTH */
        tx[a++] = 0x00;
        tx[a++] = 0x04;
        /* RDATA = AP IP */
        memcpy(&tx[a], &ip, 4);
        a += 4;

        (void)sendto(sock, tx, a, 0, (struct sockaddr *)&from, fromlen);
    }

    close(sock);
    vTaskDelete(NULL);
}

static const char *PORTAL_HTML =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>WiFi Setup</title></head><body>"
    "<h2>WiFi instellen</h2>"
    "<form method='POST' action='/wifi'>"
    "<label>SSID</label><br><input name='ssid' maxlength='32' required><br><br>"
    "<label>Wachtwoord</label><br><input name='pass' maxlength='64' type='password'><br><br>"
    "<button type='submit'>Opslaan</button>"
    "</form>"
    "<p>Na opslaan probeert het device te verbinden.</p>"
    "</body></html>";

static esp_err_t portal_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PORTAL_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t portal_post_wifi_handler(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0 || total > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad content length");
        return ESP_FAIL;
    }

    char body[513];
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r < 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_FAIL;
        }
        if (r == 0) {
            break;
        }
        received += r;
    }
    body[received] = 0;

    char ssid[33];
    char pass[65];
    if (!form_get_value(body, "ssid", ssid, sizeof(ssid))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing ssid");
        return ESP_FAIL;
    }
    (void)form_get_value(body, "pass", pass, sizeof(pass));

    memset(&s_portal_sta_cfg, 0, sizeof(s_portal_sta_cfg));
    strlcpy((char *)s_portal_sta_cfg.sta.ssid, ssid, sizeof(s_portal_sta_cfg.sta.ssid));
    strlcpy((char *)s_portal_sta_cfg.sta.password, pass, sizeof(s_portal_sta_cfg.sta.password));
    /* Be permissive: allow WPA/WPA2 mixed networks; OPEN when no password. */
    s_portal_sta_cfg.sta.threshold.authmode = (pass[0] == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA_PSK;

    size_t pass_len = strlen(pass);
    if (pass_len) {
        ESP_LOGW(TAG, "Portal received credentials: SSID='%s' (password length=%u)", ssid, (unsigned)pass_len);
    } else {
        ESP_LOGW(TAG, "Portal received credentials: SSID='%s' (empty password)", ssid);
    }
    xEventGroupSetBits(wifi_event_group, WIFI_PORTAL_CRED_BIT);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req,
                    "<html><body><h3>Opgeslagen</h3><p>Device gaat nu verbinden...</p></body></html>",
                    HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t portal_start_http(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.stack_size = 4096;

    esp_err_t err = httpd_start(&s_portal_httpd, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t wifi_post = {
        .uri = "/wifi",
        .method = HTTP_POST,
        .handler = portal_post_wifi_handler,
        .user_ctx = NULL,
    };
    err = httpd_register_uri_handler(s_portal_httpd, &wifi_post);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register POST failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t any_get = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = portal_get_handler,
        .user_ctx = NULL,
    };
    err = httpd_register_uri_handler(s_portal_httpd, &any_get);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register GET failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static void portal_stop_http(void)
{
    if (s_portal_httpd) {
        httpd_stop(s_portal_httpd);
        s_portal_httpd = NULL;
    }
}

static void captive_portal_start(void)
{
    uint8_t mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "playmp3_setup_%02X%02X%02X", mac[3], mac[4], mac[5]);

    wifi_config_t ap_cfg = {0};
    strlcpy((char *)ap_cfg.ap.ssid, ap_ssid, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = (uint8_t)strlen(ap_ssid);
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif && esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
        ESP_LOGW(TAG, "Captive portal AP '%s' IP=" IPSTR, ap_ssid, IP2STR(&ip_info.ip));
    } else {
        ESP_LOGW(TAG, "Captive portal AP '%s' started", ap_ssid);
    }

    /* DNS catch-all -> AP IP */
    s_dns_task_stop = false;
    uint32_t ip_nbo = (ip_info.ip.addr == 0) ? inet_addr("192.168.4.1") : ip_info.ip.addr;
    xTaskCreate(dns_server_task, "dns_captive", 3072, (void *)(uintptr_t)ip_nbo, 4, &s_dns_task);

    ESP_ERROR_CHECK(portal_start_http());

    ESP_LOGW(TAG, "Open browser to http://192.168.4.1/ (or it should pop up as a captive portal)");
}

static void captive_portal_stop(void)
{
    portal_stop_http();

    if (s_dns_task) {
        s_dns_task_stop = true;
        /* give task time to exit */
        vTaskDelay(pdMS_TO_TICKS(250));
        s_dns_task = NULL;
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_wifi_retry_count = 0;
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *disc = (const wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected (reason=%d)", disc ? disc->reason : -1);

        if (s_wifi_retry_count < 10) {
            esp_wifi_connect();
            s_wifi_retry_count++;
            ESP_LOGW(TAG, "Retrying WiFi connection (%d/10)", s_wifi_retry_count);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_has_saved_creds(void)
{
    wifi_config_t cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK) {
        return false;
    }
    return cfg.sta.ssid[0] != 0;
}

static void wifi_init_sta_or_portal(void)
{
    wifi_event_group = xEventGroupCreate();

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    /* Captive portal needs AP netif for DHCP/IP; keep STA netif too. */
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    for (;;) {
        if (wifi_has_saved_creds()) {
            ESP_LOGI(TAG, "WiFi credentials found in flash. Connecting...");
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            s_wifi_retry_count = 0;
            ESP_ERROR_CHECK(esp_wifi_start());
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
            EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                   pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
            if (bits & WIFI_CONNECTED_BIT) {
                ESP_LOGI(TAG, "WiFi connected");
                return;
            }
            ESP_LOGW(TAG, "WiFi connect failed. Starting captive portal...");
            ESP_ERROR_CHECK(esp_wifi_stop());
        } else {
            ESP_LOGW(TAG, "No WiFi credentials found. Starting captive portal...");
        }

        xEventGroupClearBits(wifi_event_group, WIFI_PORTAL_CRED_BIT);
        captive_portal_start();

        /* Wait until we receive credentials via HTTP. */
        (void)xEventGroupWaitBits(wifi_event_group, WIFI_PORTAL_CRED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        captive_portal_stop();
        ESP_ERROR_CHECK(esp_wifi_stop());

        /* Save and try connect as STA */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &s_portal_sta_cfg));
        s_wifi_retry_count = 0;
        ESP_ERROR_CHECK(esp_wifi_start());

        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                               pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi connected after portal");
            return;
        }

        ESP_LOGW(TAG, "WiFi failed after portal. Restarting portal...");
        ESP_ERROR_CHECK(esp_wifi_stop());
    }
}

/* -- Command queue (shared between control interface and app_main) -- */
static QueueHandle_t s_cmd_queue;

static inline void ctrl_set_status(uint8_t status, uint8_t volume)
{
#if defined(CONFIG_PLAY_MP3_CTRL_IFACE_I2C) && CONFIG_PLAY_MP3_CTRL_IFACE_I2C
    i2c_slave_ctrl_set_status((i2c_player_status_t)status, volume);
#elif defined(CONFIG_PLAY_MP3_CTRL_IFACE_UART) && CONFIG_PLAY_MP3_CTRL_IFACE_UART
    uart_ctrl_set_status(status, volume);
#else
    (void)status;
    (void)volume;
#endif
}

void app_main(void)
{
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t http_stream_reader, i2s_stream_writer, mp3_decoder;

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("I2C_SLAVE", ESP_LOG_INFO);

    ESP_LOGI(TAG, "[ 0 ] Initialize control interface, NVS and WiFi");
    /* Create command queue before init so control task can start receiving */
    s_cmd_queue = xQueueCreate(8, sizeof(i2c_command_t));

#if defined(CONFIG_PLAY_MP3_CTRL_IFACE_I2C) && CONFIG_PLAY_MP3_CTRL_IFACE_I2C
    /* Start I2C slave early so the front-end can communicate even while WiFi connects */
    ESP_ERROR_CHECK(i2c_slave_ctrl_init(s_cmd_queue));
#elif defined(CONFIG_PLAY_MP3_CTRL_IFACE_UART) && CONFIG_PLAY_MP3_CTRL_IFACE_UART
    /* Start UART control early so the front-end can communicate even while WiFi connects */
    ESP_ERROR_CHECK(uart_ctrl_init(s_cmd_queue));
#else
    ESP_LOGW(TAG, "No control interface selected in sdkconfig");
#endif

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_sta_or_portal();

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
    /* Large ring buffer absorbs network jitter (~4 s at 128 kbps).
     * Higher task priority keeps the buffer filled ahead of the decoder. */
    http_cfg.out_rb_size = 64 * 1024;
    http_cfg.task_stack  = 6 * 1024;
    http_cfg.task_prio   = 7;
    http_stream_reader = http_stream_init(&http_cfg);

    ESP_LOGI(TAG, "[2.2] Create mp3 decoder");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    /* Bigger output buffer smooths the hand-off to the I2S DMA. */
    mp3_cfg.out_rb_size = 16 * 1024;
    mp3_cfg.task_prio   = 6;
    mp3_decoder = mp3_decoder_init(&mp3_cfg);

    ESP_LOGI(TAG, "[2.3] Create i2s stream to write data to codec chip");
#if defined CONFIG_ESP32_C3_LYRA_V2_BOARD
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_PDM_TX_CFG_DEFAULT();
#else
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
#endif
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    /* Higher I2S task priority prevents audio underruns under CPU load. */
    i2s_cfg.task_prio = 23;
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

    // Belangrijk: start NIET automatisch met de default stream.
    // Dit voorkomt dat de LyraT na een reset spontaan weer Radio 2 gaat spelen.
    ESP_LOGI(TAG, "[ 5.1 ] Ready (idle): default URL set, waiting for PLAY or SET_URL");
    ctrl_set_status(PLAYER_STOPPED, (uint8_t)player_volume);

    while (1) {
        /* -- Check control command queue first (non-blocking) -- */
        i2c_command_t i2c_cmd;
        while (xQueueReceive(s_cmd_queue, &i2c_cmd, 0) == pdTRUE) {
            switch (i2c_cmd.opcode) {
                case CMD_PLAY: {
                    audio_element_state_t st = audio_element_get_state(i2s_stream_writer);
                    if (st == AEL_STATE_PAUSED) {
                        audio_pipeline_resume(pipeline);
                        ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        ESP_LOGI(TAG, "[I2C] PLAY -- resumed");
                    } else if (st == AEL_STATE_INIT || st == AEL_STATE_FINISHED) {
                        audio_pipeline_run(pipeline);
                        ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        ESP_LOGI(TAG, "[I2C] PLAY -- started");
                    }
                    break;
                }
                case CMD_STOP:
                    {
                        audio_element_state_t st = audio_element_get_state(i2s_stream_writer);
                        if (st == AEL_STATE_RUNNING || st == AEL_STATE_PAUSED || st == AEL_STATE_FINISHED) {
                            audio_pipeline_stop(pipeline);
                            audio_pipeline_wait_for_stop(pipeline);
                        }
                    }
                    ctrl_set_status(PLAYER_STOPPED, (uint8_t)player_volume);
                    ESP_LOGI(TAG, "[I2C] STOP");
                    break;

                case CMD_PAUSE:
                    {
                        audio_element_state_t st = audio_element_get_state(i2s_stream_writer);
                        if (st == AEL_STATE_RUNNING) {
                            audio_pipeline_pause(pipeline);
                            ctrl_set_status(PLAYER_PAUSED, (uint8_t)player_volume);
                            ESP_LOGI(TAG, "[I2C] PAUSE");
                        } else {
                            ESP_LOGI(TAG, "[I2C] PAUSE ignored (state=%d)", (int)st);
                        }
                    }
                    break;

                case CMD_VOLUME:
                    player_volume = i2c_cmd.volume;
                    audio_hal_set_volume(board_handle->audio_hal, player_volume);
                    ctrl_set_status(
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
                    ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                    break;

                default:
                    break;
            }
        }

        /* -- Wait for audio pipeline / peripheral event (50 ms timeout) -- */
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
                        ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        break;
                    case AEL_STATE_RUNNING :
                        ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
                        audio_pipeline_pause(pipeline);
                        ctrl_set_status(PLAYER_PAUSED, (uint8_t)player_volume);
                        break;
                    case AEL_STATE_PAUSED :
                        ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
                        audio_pipeline_resume(pipeline);
                        ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        break;
                    case AEL_STATE_FINISHED :
                        ESP_LOGI(TAG, "[ * ] Restarting radio stream");
                        audio_pipeline_reset_ringbuffer(pipeline);
                        audio_pipeline_reset_elements(pipeline);
                        audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
                        audio_element_set_uri(http_stream_reader, RADIO_STREAM_URL);
                        audio_pipeline_run(pipeline);
                        ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
                        break;
                    default :
                        ESP_LOGI(TAG, "[ * ] Not supported state %d", el_state);
                }
            } else if ((int) msg.data == get_input_set_id()) {
                ESP_LOGI(TAG, "[ * ] [Set] touch tap event");
                ESP_LOGI(TAG, "[ * ] Stopping audio pipeline");
                audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
                if (el_state == AEL_STATE_RUNNING || el_state == AEL_STATE_PAUSED || el_state == AEL_STATE_FINISHED) {
                    audio_pipeline_stop(pipeline);
                    audio_pipeline_wait_for_stop(pipeline);
                }
                ctrl_set_status(PLAYER_STOPPED, (uint8_t)player_volume);
                continue;
            } else if ((int) msg.data == get_input_mode_id()) {
                ESP_LOGI(TAG, "[ * ] [mode] tap event - restart stream");
                audio_pipeline_stop(pipeline);
                audio_pipeline_wait_for_stop(pipeline);
                audio_pipeline_terminate(pipeline);
                audio_pipeline_reset_ringbuffer(pipeline);
                audio_pipeline_reset_elements(pipeline);
                audio_element_set_uri(http_stream_reader, RADIO_STREAM_URL);
                audio_pipeline_run(pipeline);
                ctrl_set_status(PLAYER_PLAYING, (uint8_t)player_volume);
            } else if ((int) msg.data == get_input_volup_id()) {
                ESP_LOGI(TAG, "[ * ] [Vol+] touch tap event");
                player_volume += 10;
                if (player_volume > 100) {
                    player_volume = 100;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ctrl_set_status(
                    (audio_element_get_state(i2s_stream_writer) == AEL_STATE_RUNNING)
                        ? PLAYER_PLAYING : PLAYER_STOPPED,
                    (uint8_t)player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
            } else if ((int) msg.data == get_input_voldown_id()) {
                ESP_LOGI(TAG, "[ * ] [Vol-] touch tap event");
                player_volume -= 10;
                if (player_volume < 0) {
                    player_volume = 0;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ctrl_set_status(
                    (audio_element_get_state(i2s_stream_writer) == AEL_STATE_RUNNING)
                        ? PLAYER_PLAYING : PLAYER_STOPPED,
                    (uint8_t)player_volume);
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
