#include <stdio.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// LQ-R4CHVB digital outputs. Change these definitions to match the wiring.
#define TRACK_SENSOR_CH1_GPIO GPIO_NUM_11
#define TRACK_SENSOR_CH2_GPIO GPIO_NUM_12
#define TRACK_SENSOR_CH3_GPIO GPIO_NUM_13
#define TRACK_SENSOR_CH4_GPIO GPIO_NUM_14

#define SENSOR_READ_INTERVAL_MS 100

void app_main(void)
{
    const gpio_config_t sensor_gpio_config = {
        .pin_bit_mask = (1ULL << TRACK_SENSOR_CH1_GPIO) |
                        (1ULL << TRACK_SENSOR_CH2_GPIO) |
                        (1ULL << TRACK_SENSOR_CH3_GPIO) |
                        (1ULL << TRACK_SENSOR_CH4_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&sensor_gpio_config));

    while (1) {
        const int ch1 = gpio_get_level(TRACK_SENSOR_CH1_GPIO);
        const int ch2 = gpio_get_level(TRACK_SENSOR_CH2_GPIO);
        const int ch3 = gpio_get_level(TRACK_SENSOR_CH3_GPIO);
        const int ch4 = gpio_get_level(TRACK_SENSOR_CH4_GPIO);

        printf("CH1=%d CH2=%d CH3=%d CH4=%d\n", ch1, ch2, ch3, ch4);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}
