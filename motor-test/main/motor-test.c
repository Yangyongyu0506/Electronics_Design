#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor";

#define MOTOR_COUNT   3
#define PWM_FREQ_HZ   20000
#define MAX_DUTY      1023
#define ENC_HIGH_LIMIT  1000
#define ENC_LOW_LIMIT   -1000

#define MOTOR1_PWM_GPIO   GPIO_NUM_1
#define MOTOR1_IN1_GPIO   GPIO_NUM_42
#define MOTOR1_IN2_GPIO   GPIO_NUM_2
#define MOTOR1_ENC_A_GPIO GPIO_NUM_41
#define MOTOR1_ENC_B_GPIO GPIO_NUM_40

#define MOTOR2_PWM_GPIO   GPIO_NUM_4
#define MOTOR2_IN1_GPIO   GPIO_NUM_5
#define MOTOR2_IN2_GPIO   GPIO_NUM_6
#define MOTOR2_ENC_A_GPIO GPIO_NUM_38
#define MOTOR2_ENC_B_GPIO GPIO_NUM_39

#define MOTOR3_PWM_GPIO   GPIO_NUM_7
#define MOTOR3_IN1_GPIO   GPIO_NUM_16
#define MOTOR3_IN2_GPIO   GPIO_NUM_15
#define MOTOR3_ENC_A_GPIO GPIO_NUM_21
#define MOTOR3_ENC_B_GPIO GPIO_NUM_20

typedef struct {
    int pwm_gpio;
    int in1_gpio;
    int in2_gpio;
    int enc_a_gpio;
    int enc_b_gpio;
    ledc_channel_t pwm_channel;
    pcnt_unit_handle_t enc_unit;
} motor_t;

static motor_t motors[MOTOR_COUNT] = {
    {MOTOR1_PWM_GPIO, MOTOR1_IN1_GPIO, MOTOR1_IN2_GPIO,
     MOTOR1_ENC_A_GPIO, MOTOR1_ENC_B_GPIO, LEDC_CHANNEL_0, NULL},
    {MOTOR2_PWM_GPIO, MOTOR2_IN1_GPIO, MOTOR2_IN2_GPIO,
     MOTOR2_ENC_A_GPIO, MOTOR2_ENC_B_GPIO, LEDC_CHANNEL_1, NULL},
    {MOTOR3_PWM_GPIO, MOTOR3_IN1_GPIO, MOTOR3_IN2_GPIO,
     MOTOR3_ENC_A_GPIO, MOTOR3_ENC_B_GPIO, LEDC_CHANNEL_2, NULL},
};

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

        pcnt_unit_config_t unit_conf = {
            .low_limit = ENC_LOW_LIMIT,
            .high_limit = ENC_HIGH_LIMIT,
            .flags.accum_count = 1,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_conf, &motors[i].enc_unit));

        pcnt_glitch_filter_config_t filter_conf = {
            .max_glitch_ns = 1000,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motors[i].enc_unit, &filter_conf));

        pcnt_chan_config_t chan_a_conf = {
            .edge_gpio_num = motors[i].enc_a_gpio,
            .level_gpio_num = motors[i].enc_b_gpio,
        };
        pcnt_channel_handle_t chan_a = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(motors[i].enc_unit, &chan_a_conf, &chan_a));

        pcnt_chan_config_t chan_b_conf = {
            .edge_gpio_num = motors[i].enc_b_gpio,
            .level_gpio_num = motors[i].enc_a_gpio,
        };
        pcnt_channel_handle_t chan_b = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(motors[i].enc_unit, &chan_b_conf, &chan_b));

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

        ESP_ERROR_CHECK(pcnt_unit_enable(motors[i].enc_unit));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(motors[i].enc_unit));
        ESP_ERROR_CHECK(pcnt_unit_start(motors[i].enc_unit));
    }
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

static void motor_brake(int index)
{
    gpio_set_level(motors[index].in1_gpio, 1);
    gpio_set_level(motors[index].in2_gpio, 1);
}

static void motor_coast(int index)
{
    gpio_set_level(motors[index].in1_gpio, 0);
    gpio_set_level(motors[index].in2_gpio, 0);
}

static int encoder_get_count(int index)
{
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(motors[index].enc_unit, &count));
    return count;
}

static void log_encoder_counts(void)
{
    int counts[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) {
        counts[i] = encoder_get_count(i);
    }
    ESP_LOGI(TAG, "encoder counts: m1=%d m2=%d m3=%d", counts[0], counts[1], counts[2]);
}

static void set_2d_speed(int vx, int wz)
{
    motor_set_speed(0, (int)(vx * 0.5 + wz));
    motor_set_speed(2, (int)(-vx * 0.5 + wz));
    motor_set_speed(1, (int)(wz));
}

void app_main(void)
{
    motors_init();
    ESP_LOGI(TAG, "motors initialized");

    set_2d_speed(50, 0);

    // while (1) {
    //     for (int m = 0; m < MOTOR_COUNT; m++) {
    //         ESP_LOGI(TAG, "motor %d: forward 60%% for 2 s", m + 1);
    //         motor_set_speed(m, 60);
    //         for (int t = 0; t < 4; t++) {
    //             vTaskDelay(pdMS_TO_TICKS(500));
    //             log_encoder_counts();
    //         }

    //         ESP_LOGI(TAG, "motor %d: brake for 1 s", m + 1);
    //         motor_brake(m);
    //         vTaskDelay(pdMS_TO_TICKS(1000));

    //         ESP_LOGI(TAG, "motor %d: reverse 60%% for 2 s", m + 1);
    //         motor_set_speed(m, -60);
    //         for (int t = 0; t < 4; t++) {
    //             vTaskDelay(pdMS_TO_TICKS(500));
    //             log_encoder_counts();
    //         }

    //         ESP_LOGI(TAG, "motor %d: coast for 1 s", m + 1);
    //         motor_coast(m);
    //         vTaskDelay(pdMS_TO_TICKS(1000));
    //     }
    // }
}
