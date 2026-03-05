/*
 * audio_level_tap.c
 *
 * Pass-through audio element that calculates peak audio levels (L/R)
 * from 16-bit interleaved stereo PCM data.
 */

#include "audio_level_tap.h"
#include "audio_element.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "LEVEL_TAP";

static volatile uint8_t s_peak_l = 0;
static volatile uint8_t s_peak_r = 0;

uint8_t audio_level_get_l(void) { return s_peak_l; }
uint8_t audio_level_get_r(void) { return s_peak_r; }
void    audio_level_reset(void) { s_peak_l = 0; s_peak_r = 0; }

static esp_err_t level_tap_open(audio_element_handle_t self)
{
    ESP_LOGI(TAG, "open");
    return ESP_OK;
}

static esp_err_t level_tap_close(audio_element_handle_t self)
{
    ESP_LOGI(TAG, "close");
    s_peak_l = 0;
    s_peak_r = 0;
    return ESP_OK;
}

static audio_element_err_t level_tap_process(audio_element_handle_t self,
                                             char *buf, int len)
{
    int bytes_read = audio_element_input(self, buf, len);
    if (bytes_read <= 0) {
        return (audio_element_err_t)bytes_read;
    }

    /* Calculate peak levels from 16-bit interleaved stereo PCM */
    int16_t *samples = (int16_t *)buf;
    int n = bytes_read / (int)sizeof(int16_t);
    int32_t peak_l = 0, peak_r = 0;

    for (int i = 0; i + 1 < n; i += 2) {
        int32_t al = abs((int)samples[i]);
        int32_t ar = abs((int)samples[i + 1]);
        if (al > peak_l) peak_l = al;
        if (ar > peak_r) peak_r = ar;
    }

    /* Scale 0..32767 -> 0..255 */
    s_peak_l = (uint8_t)((peak_l * 255) / 32768);
    s_peak_r = (uint8_t)((peak_r * 255) / 32768);

    return (audio_element_err_t)audio_element_output(self, buf, bytes_read);
}

audio_element_handle_t audio_level_tap_init(void)
{
    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    cfg.open         = level_tap_open;
    cfg.close        = level_tap_close;
    cfg.process      = level_tap_process;
    cfg.buffer_len   = 4 * 1024;
    cfg.task_stack   = 3 * 1024;
    cfg.task_prio    = 5;
    cfg.tag          = "level_tap";

    audio_element_handle_t el = audio_element_init(&cfg);
    if (!el) {
        ESP_LOGE(TAG, "audio_element_init failed");
    }
    return el;
}
