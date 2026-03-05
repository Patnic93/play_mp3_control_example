/*
 * audio_level_tap.h
 *
 * Pass-through audio element that calculates peak audio levels (L/R)
 * from 16-bit interleaved stereo PCM data flowing through the pipeline.
 * Insert between equalizer and i2s_stream.
 */

#pragma once

#include "audio_element.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create and return the level-tap audio element.
 */
audio_element_handle_t audio_level_tap_init(void);

/**
 * @brief Get latest peak level for left channel (0-255).
 */
uint8_t audio_level_get_l(void);

/**
 * @brief Get latest peak level for right channel (0-255).
 */
uint8_t audio_level_get_r(void);

/**
 * @brief Reset levels to zero (call on stop/pause).
 */
void audio_level_reset(void);

#ifdef __cplusplus
}
#endif
