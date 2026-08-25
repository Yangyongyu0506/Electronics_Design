#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "encoder";

#define ENCODER_COUNT   3
#define ENC_HIGH_LIMIT  1000
#define ENC_LOW_LIMIT   -1000
#define PRINT_INTERVAL_MS 100

#define ENC_A_GPIO  GPIO_NUM_41
#define ENC_A_B_GPIO GPIO_NUM_40
#define ENC_B_GPIO  GPIO_NUM_8
#define ENC_B_B_GPIO GPIO_NUM_3
#define ENC_D_GPIO  GPIO_NUM_17
#define ENC_D_B_GPIO GPIO_NUM_18

typedef struct {
    int a_gpio;
    int b_gpio;
    pcnt_unit_handle_t unit;
    int last_count;
} encoder_t;

static encoder_t encoders[ENCODER_COUNT] = {
    {ENC_A_GPIO, ENC_A_B_GPIO, NULL, 0},
    {ENC_B_GPIO, ENC_B_B_GPIO, NULL, 0},
    {ENC_D_GPIO, ENC_D_B_GPIO, NULL, 0},
};

static void encoders_init(void)
{
    for (int i = 0; i < ENCODER_COUNT; i++) {
        pcnt_unit_config_t unit_conf = {
            .low_limit = ENC_LOW_LIMIT,
            .high_limit = ENC_HIGH_LIMIT,
            .flags.accum_count = 1,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_conf, &encoders[i].unit));

        pcnt_glitch_filter_config_t filter_conf = {
            .max_glitch_ns = 1000,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoders[i].unit, &filter_conf));

        pcnt_chan_config_t chan_a_conf = {
            .edge_gpio_num = encoders[i].a_gpio,
            .level_gpio_num = encoders[i].b_gpio,
        };
        pcnt_channel_handle_t chan_a = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(encoders[i].unit, &chan_a_conf, &chan_a));

        pcnt_chan_config_t chan_b_conf = {
            .edge_gpio_num = encoders[i].b_gpio,
            .level_gpio_num = encoders[i].a_gpio,
        };
        pcnt_channel_handle_t chan_b = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(encoders[i].unit, &chan_b_conf, &chan_b));

        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        ESP_ERROR_CHECK(pcnt_unit_enable(encoders[i].unit));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(encoders[i].unit));
        ESP_ERROR_CHECK(pcnt_unit_start(encoders[i].unit));
    }
}

void app_main(void)
{
    encoders_init();
    ESP_LOGI(TAG, "hand-rotate each wheel: 1 revolution = 512 counts");

    while (1) {
        int counts[ENCODER_COUNT];
        int deltas[ENCODER_COUNT];

        for (int i = 0; i < ENCODER_COUNT; i++) {
            ESP_ERROR_CHECK(pcnt_unit_get_count(encoders[i].unit, &counts[i]));
            deltas[i] = counts[i] - encoders[i].last_count;
            encoders[i].last_count = counts[i];
        }

        ESP_LOGI(TAG, "A: count=%8d d=%6d/s | B: count=%8d d=%6d/s | D: count=%8d d=%6d/s",
                 counts[0], deltas[0] * (1000 / PRINT_INTERVAL_MS),
                 counts[1], deltas[1] * (1000 / PRINT_INTERVAL_MS),
                 counts[2], deltas[2] * (1000 / PRINT_INTERVAL_MS));

        vTaskDelay(pdMS_TO_TICKS(PRINT_INTERVAL_MS));
    }
}
