#include "tft_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "tft";
static spi_device_handle_t tft_spi;
static uint16_t frame[TFT_WIDTH * TFT_HEIGHT];
static portMUX_TYPE data_lock = portMUX_INITIALIZER_UNLOCKED;
static float shown_speed[3];
static float shown_distance;
static bool shown_distance_valid;

/* 5x7 glyphs needed by the dashboard: space, '-', '.', ':', 0-9, A-Z. */
static const uint8_t font[][5] = {
    [' ' - ' ']={0,0,0,0,0}, ['-' - ' ']={0x08,0x08,0x08,0x08,0x08},
    ['.' - ' ']={0,0x60,0x60,0,0}, [':' - ' ']={0,0x36,0x36,0,0},
    ['0' - ' ']={0x3e,0x51,0x49,0x45,0x3e}, ['1' - ' ']={0,0x42,0x7f,0x40,0},
    ['2' - ' ']={0x42,0x61,0x51,0x49,0x46}, ['3' - ' ']={0x21,0x41,0x45,0x4b,0x31},
    ['4' - ' ']={0x18,0x14,0x12,0x7f,0x10}, ['5' - ' ']={0x27,0x45,0x45,0x45,0x39},
    ['6' - ' ']={0x3c,0x4a,0x49,0x49,0x30}, ['7' - ' ']={1,0x71,9,5,3},
    ['8' - ' ']={0x36,0x49,0x49,0x49,0x36}, ['9' - ' ']={6,0x49,0x49,0x29,0x1e},
    ['A' - ' ']={0x7e,0x11,0x11,0x11,0x7e}, ['C' - ' ']={0x3e,0x41,0x41,0x41,0x22},
    ['D' - ' ']={0x7f,0x41,0x41,0x22,0x1c}, ['E' - ' ']={0x7f,0x49,0x49,0x49,0x41},
    ['I' - ' ']={0,0x41,0x7f,0x41,0}, ['M' - ' ']={0x7f,2,0x0c,2,0x7f},
    ['O' - ' ']={0x3e,0x41,0x41,0x41,0x3e}, ['P' - ' ']={0x7f,9,9,9,6},
    ['R' - ' ']={0x7f,9,0x19,0x29,0x46}, ['S' - ' ']={0x26,0x49,0x49,0x49,0x32},
    ['T' - ' ']={1,1,0x7f,1,1},
};

static esp_err_t write_bytes(bool data, const void *bytes, size_t length)
{
    gpio_set_level(TFT_DC_GPIO, data);
    spi_transaction_t transaction = {.length = length * 8, .tx_buffer = bytes};
    return spi_device_polling_transmit(tft_spi, &transaction);
}

static esp_err_t command(uint8_t cmd, const uint8_t *data, size_t length)
{
    ESP_RETURN_ON_ERROR(write_bytes(false, &cmd, 1), TAG, "command");
    return length ? write_bytes(true, data, length) : ESP_OK;
}

static void fill_rect(int x, int y, int width, int height, uint16_t color)
{
    for (int py = y; py < y + height && py < TFT_HEIGHT; py++)
        for (int px = x; px < x + width && px < TFT_WIDTH; px++)
            if (px >= 0 && py >= 0) frame[py * TFT_WIDTH + px] = __builtin_bswap16(color);
}

static void draw_text(int x, int y, const char *text, uint16_t color, int scale)
{
    while (*text) {
        unsigned char ch = (unsigned char)*text++;
        const uint8_t *glyph = (ch >= ' ' && ch <= 'Z') ? font[ch - ' '] : font[0];
        for (int col = 0; col < 5; col++)
            for (int row = 0; row < 7; row++)
                if (glyph[col] & (1U << row))
                    fill_rect(x + col * scale, y + row * scale, scale, scale, color);
        x += 6 * scale;
    }
}

static esp_err_t flush_frame(void)
{
    const uint8_t col[] = {0, 0, 0, TFT_WIDTH - 1};
    ESP_RETURN_ON_ERROR(command(0x2a, col, sizeof(col)), TAG, "column address");

    for (int y = 0; y < TFT_HEIGHT; y += 8) {
        int rows = (TFT_HEIGHT - y < 8) ? TFT_HEIGHT - y : 8;
        const uint8_t row[] = {0, (uint8_t)y, 0, (uint8_t)(y + rows - 1)};
        ESP_RETURN_ON_ERROR(command(0x2b, row, sizeof(row)), TAG, "row address");
        ESP_RETURN_ON_ERROR(command(0x2c, NULL, 0), TAG, "memory write");
        ESP_RETURN_ON_ERROR(write_bytes(true, &frame[y * TFT_WIDTH],
                                        (size_t)rows * TFT_WIDTH * sizeof(frame[0])),
                            TAG, "pixel data");
    }
    return ESP_OK;
}

static void display_task(void *arg)
{
    char line[24];
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        float speed[3], distance;
        bool distance_valid;
        portENTER_CRITICAL(&data_lock);
        memcpy(speed, shown_speed, sizeof(speed));
        distance = shown_distance;
        distance_valid = shown_distance_valid;
        portEXIT_CRITICAL(&data_lock);

        memset(frame, 0, sizeof(frame));
        draw_text(31, 10, "MOTOR SPEED", 0xffff, 1);
        for (int i = 0; i < 3; i++) {
            snprintf(line, sizeof(line), "M%d: %6.1f", i + 1, speed[i]);
            draw_text(8, 38 + i * 25, line, 0x07ff, 2);
        }
        draw_text(8, 116, "DIST:", 0xffe0, 2);
        if (distance_valid) snprintf(line, sizeof(line), "%5.1f CM", distance);
        else snprintf(line, sizeof(line), "--.- CM");
        draw_text(8, 138, line, distance_valid ? 0x07e0 : 0xf800, 2);

        esp_err_t err = flush_frame();
        if (err != ESP_OK) ESP_LOGW(TAG, "display update failed: %s", esp_err_to_name(err));
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TFT_UPDATE_PERIOD_MS));
    }
}

esp_err_t tft_display_start(void)
{
    gpio_config_t dc = {.pin_bit_mask = 1ULL << TFT_DC_GPIO, .mode = GPIO_MODE_OUTPUT};
    ESP_RETURN_ON_ERROR(gpio_config(&dc), TAG, "DC GPIO");
    if (TFT_RST_GPIO != GPIO_NUM_NC) {
        gpio_config_t rst = {.pin_bit_mask = 1ULL << TFT_RST_GPIO, .mode = GPIO_MODE_OUTPUT};
        ESP_RETURN_ON_ERROR(gpio_config(&rst), TAG, "RST GPIO");
        gpio_set_level(TFT_RST_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(TFT_RST_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(120));
    }
    spi_bus_config_t bus = {
        .mosi_io_num = TFT_MOSI_GPIO, .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = TFT_SCLK_GPIO, .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC, .max_transfer_sz = sizeof(frame),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(TFT_SPI_HOST, &bus, SPI_DMA_CH_AUTO), TAG, "SPI bus");
    spi_device_interface_config_t dev = {
        .clock_speed_hz = TFT_SPI_CLOCK_HZ, .mode = 0,
        .spics_io_num = TFT_CS_GPIO, .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TFT_SPI_HOST, &dev, &tft_spi), TAG, "SPI device");

    ESP_RETURN_ON_ERROR(command(0x01, NULL, 0), TAG, "software reset");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(command(0x11, NULL, 0), TAG, "sleep out");
    vTaskDelay(pdMS_TO_TICKS(120));
    const uint8_t color_mode = 0x05;
    ESP_RETURN_ON_ERROR(command(0x3a, &color_mode, 1), TAG, "RGB565");
    const uint8_t madctl = 0x00;
    ESP_RETURN_ON_ERROR(command(0x36, &madctl, 1), TAG, "orientation");
    ESP_RETURN_ON_ERROR(command(0x21, NULL, 0), TAG, "inversion on");
    ESP_RETURN_ON_ERROR(command(0x29, NULL, 0), TAG, "display on");

    if (xTaskCreate(display_task, "tft_display", 3072, NULL, 2, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "TFT started on MOSI=%d SCLK=%d CS=%d DC=%d RST=%d",
             TFT_MOSI_GPIO, TFT_SCLK_GPIO, TFT_CS_GPIO, TFT_DC_GPIO, TFT_RST_GPIO);
    return ESP_OK;
}

void tft_display_set_motor_speeds(float m1, float m2, float m3)
{
    portENTER_CRITICAL(&data_lock);
    shown_speed[0] = m1; shown_speed[1] = m2; shown_speed[2] = m3;
    portEXIT_CRITICAL(&data_lock);
}

void tft_display_set_distance(bool valid, float distance_cm)
{
    portENTER_CRITICAL(&data_lock);
    shown_distance_valid = valid;
    shown_distance = distance_cm;
    portEXIT_CRITICAL(&data_lock);
}
