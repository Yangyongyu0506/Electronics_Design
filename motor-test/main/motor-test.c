#include <math.h>
#include <stdbool.h>
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

#define SPEED_CTRL_PERIOD_MS      20
#define COUNTS_PER_REV            512
#define WHEEL_DIAMETER_CM         5.5f
#define WHEEL_BASE_CM             9.0f
#define DEG_TO_RAD                (3.14159265f / 180.0f)
#define COUNTS_PER_CM             ((float)COUNTS_PER_REV / (3.14159265f * WHEEL_DIAMETER_CM))
#define PID_KP            2.0f
#define PID_KI            0.5f
#define PID_KD            0.0f
#define PID_INTEGRAL_LIMIT 400.0f
#define SPEED_FILTER_ALPHA 0.3f

#define MOTOR1_PWM_GPIO   GPIO_NUM_1
#define MOTOR1_IN1_GPIO   GPIO_NUM_42
#define MOTOR1_IN2_GPIO   GPIO_NUM_2
#define MOTOR1_ENC_A_GPIO GPIO_NUM_41
#define MOTOR1_ENC_B_GPIO GPIO_NUM_40

#define MOTOR2_PWM_GPIO   GPIO_NUM_4
#define MOTOR2_IN1_GPIO   GPIO_NUM_5
#define MOTOR2_IN2_GPIO   GPIO_NUM_6
#define MOTOR2_ENC_A_GPIO GPIO_NUM_8
#define MOTOR2_ENC_B_GPIO GPIO_NUM_3

#define MOTOR3_PWM_GPIO   GPIO_NUM_7
#define MOTOR3_IN1_GPIO   GPIO_NUM_16
#define MOTOR3_IN2_GPIO   GPIO_NUM_15
#define MOTOR3_ENC_A_GPIO GPIO_NUM_17
#define MOTOR3_ENC_B_GPIO GPIO_NUM_18

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_measured;
} pid_t;

static float pid_update(pid_t *pid, float error, float measured, bool saturated)
{
    float p_term = pid->kp * error;

    if (!saturated) {
        pid->integral += error;
        if (pid->integral > PID_INTEGRAL_LIMIT) {
            pid->integral = PID_INTEGRAL_LIMIT;
        } else if (pid->integral < -PID_INTEGRAL_LIMIT) {
            pid->integral = -PID_INTEGRAL_LIMIT;
        }
    }

    float d_term = pid->kd * (pid->last_measured - measured);
    pid->last_measured = measured;

    return p_term + pid->ki * pid->integral + d_term;
}

static void pid_reset(pid_t *pid)
{
    pid->integral = 0;
    pid->last_measured = 0;
}

typedef struct {
    int pwm_gpio;
    int in1_gpio;
    int in2_gpio;
    int enc_a_gpio;
    int enc_b_gpio;
    ledc_channel_t pwm_channel;
    pcnt_unit_handle_t enc_unit;
    int target_speed;
    int last_count;
    float measured_speed;
    pid_t pid;
    bool saturated;
} motor_t;

static motor_t motors[MOTOR_COUNT] = {
    {MOTOR1_PWM_GPIO, MOTOR1_IN1_GPIO, MOTOR1_IN2_GPIO,
     MOTOR1_ENC_A_GPIO, MOTOR1_ENC_B_GPIO, LEDC_CHANNEL_0, NULL,
     0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
    {MOTOR2_PWM_GPIO, MOTOR2_IN1_GPIO, MOTOR2_IN2_GPIO,
     MOTOR2_ENC_A_GPIO, MOTOR2_ENC_B_GPIO, LEDC_CHANNEL_1, NULL,
     0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
    {MOTOR3_PWM_GPIO, MOTOR3_IN1_GPIO, MOTOR3_IN2_GPIO,
     MOTOR3_ENC_A_GPIO, MOTOR3_ENC_B_GPIO, LEDC_CHANNEL_2, NULL,
     0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
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

static void motor_set_target_cm_s(int index, float speed_cm_s)
{
    int new_target = (int)(speed_cm_s * COUNTS_PER_CM);

    if (new_target == 0) {
        motors[index].target_speed = 0;
        motor_coast(index);
        pid_reset(&motors[index].pid);
        return;
    }

    if ((new_target > 0) != (motors[index].target_speed > 0)) {
        pid_reset(&motors[index].pid);
    }
    motors[index].target_speed = new_target;
}

static int encoder_get_count(int index)
{
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(motors[index].enc_unit, &count));
    return count;
}

static void speed_ctrl_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            int count = encoder_get_count(i);
            int delta = count - motors[i].last_count;
            motors[i].last_count = count;

            float measured_raw = (float)delta * 1000.0f / SPEED_CTRL_PERIOD_MS;
            motors[i].measured_speed += (measured_raw - motors[i].measured_speed) * SPEED_FILTER_ALPHA;
            float measured = motors[i].measured_speed;

            float target = (float)motors[i].target_speed;
            if (target == 0) {
                continue;
            }

            float error = fabsf(target) - fabsf(measured);
            float output = pid_update(&motors[i].pid, error, fabsf(measured), motors[i].saturated);

            uint32_t duty;
            motors[i].saturated = false;
            if (output < 0) {
                duty = 0;
                motors[i].saturated = true;
            } else if (output > MAX_DUTY) {
                duty = MAX_DUTY;
                motors[i].saturated = true;
            } else {
                duty = (uint32_t)output;
            }

            gpio_set_level(motors[i].in1_gpio, target > 0);
            gpio_set_level(motors[i].in2_gpio, target < 0);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[i].pwm_channel, duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[i].pwm_channel));
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SPEED_CTRL_PERIOD_MS));
    }
}

static void log_measured_speeds(void)
{
    ESP_LOGI(TAG, "m1 t=%6.1f m=%6.1f | m2 t=%6.1f m=%6.1f | m3 t=%6.1f m=%6.1f (cm/s)",
             (float)motors[0].target_speed / COUNTS_PER_CM, motors[0].measured_speed / COUNTS_PER_CM,
             (float)motors[1].target_speed / COUNTS_PER_CM, motors[1].measured_speed / COUNTS_PER_CM,
             (float)motors[2].target_speed / COUNTS_PER_CM, motors[2].measured_speed / COUNTS_PER_CM);
}

#define WHEEL_A_DRIVE_DEG 30.0f
#define WHEEL_B_DRIVE_DEG 270.0f
#define WHEEL_D_DRIVE_DEG 150.0f

static void set_speed(float vx, float vy, float wz)
{
    float va = 0.86602540378 * vx + 0.5 * vy + WHEEL_BASE_CM * wz;
    float vd = -0.86602540378 * vx + 0.5 * vy + WHEEL_BASE_CM * wz;
    float vb = -vy + WHEEL_BASE_CM * wz;

    motor_set_target_cm_s(0, va);
    motor_set_target_cm_s(1, vb);
    motor_set_target_cm_s(2, vd);
}

void app_main(void)
{
    motors_init();
    ESP_LOGI(TAG, "motors initialized");

    xTaskCreate(speed_ctrl_task, "speed_ctrl", 4096, NULL, 5, NULL);

    while (1) {
        ESP_LOGI(TAG, "phase: forward 10 cm/s for 3 s");
        set_speed(20.0f, 0.0f, 0.0f);
        for (int t = 0; t < 3; t++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            log_measured_speeds();
        }

        ESP_LOGI(TAG, "phase: left 10 cm/s for 3 s");
        set_speed(0.0f, 20.0f, 0.0f);
        for (int t = 0; t < 3; t++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            log_measured_speeds();
        }

        ESP_LOGI(TAG, "phase: rotate +0.8 rad/s for 3 s");
        set_speed(0.0f, 0.0f, - 0.8f);
        for (int t = 0; t < 3; t++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            log_measured_speeds();
        }

        ESP_LOGI(TAG, "phase: backward 10 cm/s for 3 s");
        set_speed(-20.0f, 0.0f, 0.0f);
        for (int t = 0; t < 3; t++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            log_measured_speeds();
        }

        ESP_LOGI(TAG, "phase: stop for 2 s");
        set_speed(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motor_brake(i);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
