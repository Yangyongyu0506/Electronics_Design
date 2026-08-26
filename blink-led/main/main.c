#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// LQ-R4CHVB digital outputs. Change these definitions to match the wiring.
#define TRACK_SENSOR_CH1_GPIO GPIO_NUM_11
#define TRACK_SENSOR_CH2_GPIO GPIO_NUM_12
#define TRACK_SENSOR_CH3_GPIO GPIO_NUM_13
#define TRACK_SENSOR_CH4_GPIO GPIO_NUM_14

#define HC_SR04_TRIG_GPIO GPIO_NUM_9
#define HC_SR04_ECHO_GPIO GPIO_NUM_10

#define SENSOR_READ_INTERVAL_MS 100
#define HC_SR04_TIMEOUT_US 30000

_Static_assert(HC_SR04_TRIG_GPIO != TRACK_SENSOR_CH1_GPIO &&
               HC_SR04_TRIG_GPIO != TRACK_SENSOR_CH2_GPIO &&
               HC_SR04_TRIG_GPIO != TRACK_SENSOR_CH3_GPIO &&
               HC_SR04_TRIG_GPIO != TRACK_SENSOR_CH4_GPIO &&
               HC_SR04_ECHO_GPIO != TRACK_SENSOR_CH1_GPIO &&
               HC_SR04_ECHO_GPIO != TRACK_SENSOR_CH2_GPIO &&
               HC_SR04_ECHO_GPIO != TRACK_SENSOR_CH3_GPIO &&
               HC_SR04_ECHO_GPIO != TRACK_SENSOR_CH4_GPIO &&
               HC_SR04_TRIG_GPIO != HC_SR04_ECHO_GPIO,
               "HC-SR04 GPIO conflicts with another sensor GPIO");

static bool hc_sr04_read_distance(float *distance_cm)
{
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(HC_SR04_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);

    const int64_t deadline_us = esp_timer_get_time() + HC_SR04_TIMEOUT_US;

    while (gpio_get_level(HC_SR04_ECHO_GPIO) == 0) {
        if (esp_timer_get_time() >= deadline_us) {
            return false;
        }
    }

    const int64_t echo_start_us = esp_timer_get_time();
    while (gpio_get_level(HC_SR04_ECHO_GPIO) == 1) {
        if (esp_timer_get_time() >= deadline_us) {
            return false;
        }
    }

    const int64_t echo_duration_us = esp_timer_get_time() - echo_start_us;
    *distance_cm = echo_duration_us / 58.0f;
    return true;
}

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

    const gpio_config_t ultrasonic_gpio_config = {
        .pin_bit_mask = (1ULL << HC_SR04_TRIG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ultrasonic_gpio_config));

    const gpio_config_t echo_gpio_config = {
        .pin_bit_mask = (1ULL << HC_SR04_ECHO_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_gpio_config));
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        const int ch1 = gpio_get_level(TRACK_SENSOR_CH1_GPIO);
        const int ch2 = gpio_get_level(TRACK_SENSOR_CH2_GPIO);
        const int ch3 = gpio_get_level(TRACK_SENSOR_CH3_GPIO);
        const int ch4 = gpio_get_level(TRACK_SENSOR_CH4_GPIO);
        float distance_cm;

        if (hc_sr04_read_distance(&distance_cm)) {
            printf("CH1=%d CH2=%d CH3=%d CH4=%d | Distance=%.1f cm\n",
                   ch1, ch2, ch3, ch4, distance_cm);
        } else {
            printf("CH1=%d CH2=%d CH3=%d CH4=%d | Distance=timeout\n",
                   ch1, ch2, ch3, ch4);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}
