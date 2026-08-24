#include <stdint.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#define LED_GPIO              GPIO_NUM_38
#define LED_NUM               1
#define UPDATE_INTERVAL_MS    20
#define BREATH_STEP           3
#define COLOR_TRANSITION_STEPS 255

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_color_t;

static const rgb_color_t color_palette[] = {
    {255,   0,   0}, // Red
    {255,   0, 255}, // Purple
    {  0,   0, 255}, // Blue
    {  0, 255, 255}, // Cyan
    {  0, 255,   0}, // Green
    {255, 255,   0}, // Yellow
};

static uint8_t interpolate(uint8_t start, uint8_t end, uint16_t position)
{
    int32_t value = (int32_t)start * COLOR_TRANSITION_STEPS
                    + ((int32_t)end - start) * position;
    return (uint8_t)(value / COLOR_TRANSITION_STEPS);
}

static uint8_t apply_brightness(uint8_t component, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)component * brightness + 127) / 255);
}

void app_main(void)
{
    led_strip_handle_t led_strip;

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_NUM,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config,
                                              &led_strip));

    uint16_t breath_phase = 0;
    uint16_t color_phase = 0;
    const uint16_t color_cycle_steps =
        (sizeof(color_palette) / sizeof(color_palette[0]))
        * COLOR_TRANSITION_STEPS;

    while (1) {
        // Triangle wave: 0 -> 255 -> 0.
        uint8_t brightness = breath_phase <= 255
                                 ? breath_phase
                                 : 510 - breath_phase;

        uint8_t color_index = color_phase / COLOR_TRANSITION_STEPS;
        uint8_t next_index = (color_index + 1)
                             % (sizeof(color_palette) / sizeof(color_palette[0]));
        uint16_t transition_position = color_phase % COLOR_TRANSITION_STEPS;

        uint8_t red = interpolate(color_palette[color_index].red,
                                  color_palette[next_index].red,
                                  transition_position);
        uint8_t green = interpolate(color_palette[color_index].green,
                                    color_palette[next_index].green,
                                    transition_position);
        uint8_t blue = interpolate(color_palette[color_index].blue,
                                   color_palette[next_index].blue,
                                   transition_position);

        ESP_ERROR_CHECK(led_strip_set_pixel(
            led_strip, 0,
            apply_brightness(red, brightness),
            apply_brightness(green, brightness),
            apply_brightness(blue, brightness)));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

        breath_phase = (breath_phase + BREATH_STEP) % 510;
        color_phase = (color_phase + 1) % color_cycle_steps;
        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }
}
