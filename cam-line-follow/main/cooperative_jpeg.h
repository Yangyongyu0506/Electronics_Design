#pragma once

#include "jpeg_decoder.h"

/*
 * JPEG decode wrapper which gives FreeRTOS a scheduling point while TJpgDec
 * works through a frame.  It intentionally has the same input/output API as
 * esp_jpeg_decode(), so camera negotiation and capture settings are untouched.
 */
esp_err_t cooperative_jpeg_decode(esp_jpeg_image_cfg_t *cfg,
                                  esp_jpeg_image_output_t *img);
