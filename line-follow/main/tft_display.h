#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

/*
 * LQ_TFT18SPIV33 (ST7735-compatible) wiring. Change pins only here.
 * GPIO_NUM_NC is supported for CS (tie TFT CS low) and RST (tie to EN/reset).
 * The defaults use currently unassigned, non-flash, non-USB GPIOs.
 */
#define TFT_MOSI_GPIO GPIO_NUM_47
#define TFT_SCLK_GPIO GPIO_NUM_21
#define TFT_CS_GPIO GPIO_NUM_NC
#define TFT_DC_GPIO GPIO_NUM_39
#define TFT_RST_GPIO GPIO_NUM_48

#define TFT_SPI_HOST SPI2_HOST
#define TFT_SPI_CLOCK_HZ (20 * 1000 * 1000)
#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define TFT_UPDATE_PERIOD_MS 200

esp_err_t tft_display_start(void);
void tft_display_set_motor_speeds(float m1_cm_s, float m2_cm_s, float m3_cm_s);
void tft_display_set_distance(bool valid, float distance_cm);
