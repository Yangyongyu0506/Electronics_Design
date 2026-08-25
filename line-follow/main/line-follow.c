#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "line-follow";

#define MOTOR_COUNT 3
#define PWM_FREQ_HZ 20000
#define MAX_DUTY 1023

#define MOTOR_A_PWM_GPIO GPIO_NUM_1
#define MOTOR_A_IN1_GPIO GPIO_NUM_42
#define MOTOR_A_IN2_GPIO GPIO_NUM_2

#define MOTOR_B_PWM_GPIO GPIO_NUM_4
#define MOTOR_B_IN1_GPIO GPIO_NUM_5
#define MOTOR_B_IN2_GPIO GPIO_NUM_6

#define MOTOR_D_PWM_GPIO GPIO_NUM_7
#define MOTOR_D_IN1_GPIO GPIO_NUM_16
#define MOTOR_D_IN2_GPIO GPIO_NUM_15

#define SENSOR_COUNT 4
#define SENSOR_CH1_GPIO GPIO_NUM_11
#define SENSOR_CH2_GPIO GPIO_NUM_12
#define SENSOR_CH3_GPIO GPIO_NUM_13
#define SENSOR_CH4_GPIO GPIO_NUM_14

#define CONTROL_PERIOD_MS 10
#define BASE_VX 40
#define KP 12
#define MAX_WZ 75

typedef struct {
    int pwm_gpio;
    int in1_gpio;
    int in2_gpio;
    ledc_channel_t pwm_channel;
} motor_t;

static motor_t motors[MOTOR_COUNT] = {
    {MOTOR_A_PWM_GPIO, MOTOR_A_IN1_GPIO, MOTOR_A_IN2_GPIO, LEDC_CHANNEL_0},
    {MOTOR_B_PWM_GPIO, MOTOR_B_IN1_GPIO, MOTOR_B_IN2_GPIO, LEDC_CHANNEL_1},
    {MOTOR_D_PWM_GPIO, MOTOR_D_IN1_GPIO, MOTOR_D_IN2_GPIO, LEDC_CHANNEL_2},
};

static const int sensor_gpios[SENSOR_COUNT] = {
    SENSOR_CH1_GPIO,
    SENSOR_CH2_GPIO,
    SENSOR_CH3_GPIO,
    SENSOR_CH4_GPIO,
};

static const int sensor_weights[SENSOR_COUNT] = {-3, -1, 1, 3};

static void motors_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    uint64_t in_pin_mask = 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        in_pin_mask |= (1ULL << motors[i].in1_gpio) | (1ULL << motors[i].in2_gpio);
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = in_pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        gpio_set_level(motors[i].in1_gpio, 0);
        gpio_set_level(motors[i].in2_gpio, 0);
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t chan_conf = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i].pwm_channel,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = motors[i].pwm_gpio,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&chan_conf));
    }
}

static void sensor_init(void)
{
    uint64_t sensor_pin_mask = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_pin_mask |= (1ULL << sensor_gpios[i]);
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = sensor_pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void motor_set_speed(int index, int speed_percent)
{
    if (speed_percent > 100) {
        speed_percent = 100;
    }
    if (speed_percent < -100) {
        speed_percent = -100;
    }

    if (speed_percent == 0) {
        gpio_set_level(motors[index].in1_gpio, 0);
        gpio_set_level(motors[index].in2_gpio, 0);
        return;
    }

    uint32_t duty = (speed_percent < 0 ? -speed_percent : speed_percent) * MAX_DUTY / 100;
    gpio_set_level(motors[index].in1_gpio, speed_percent > 0);
    gpio_set_level(motors[index].in2_gpio, speed_percent < 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[index].pwm_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[index].pwm_channel));
}

static void set_2d_speed(int vx, int wz)
{
    motor_set_speed(0, (int)(vx * 0.5) + wz);
    motor_set_speed(2, -(int)(vx * 0.5) + wz);
    motor_set_speed(1, wz);
}

static bool read_line_error(int *error_scaled)
{
    int error_sum = 0;
    int black_count = 0;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        int level = gpio_get_level(sensor_gpios[i]);
        if (level == 0) {
            error_sum += sensor_weights[i];
            black_count++;
        }
    }

    if (black_count == 0) {
        return false;
    }

    *error_scaled = error_sum * 100 / black_count;
    return true;
}

void app_main(void)
{
    motors_init();
    sensor_init();
    ESP_LOGI(TAG, "motors and line sensor initialized");

    set_2d_speed(0, 0);

    bool line_ok = false;
    uint32_t tick = 0;

    while (1) {
        int error = 0;
        bool ok = read_line_error(&error);

        if (ok) {
            if (!line_ok) {
                ESP_LOGI(TAG, "line detected");
            }
            line_ok = true;

            int wz = KP * error / 100;
            if (wz > MAX_WZ) {
                wz = MAX_WZ;
            }
            if (wz < -MAX_WZ) {
                wz = -MAX_WZ;
            }

            set_2d_speed(BASE_VX, wz);

            if (++tick % 100 == 0) {
                ESP_LOGI(TAG, "error=%d wz=%d", error, wz);
            }
        } else {
            if (line_ok) {
                ESP_LOGW(TAG, "line lost, stop");
            }
            line_ok = false;
            set_2d_speed(0, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
