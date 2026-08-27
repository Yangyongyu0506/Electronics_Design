#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tft_display.h"

static const char *TAG = "line-follow";

#define MOTOR_COUNT 3
#define PWM_FREQ_HZ 20000
#define MAX_DUTY 1023
#define ENC_HIGH_LIMIT 1000
#define ENC_LOW_LIMIT -1000

#define SPEED_CTRL_PERIOD_MS 20
#define COUNTS_PER_REV 512
#define WHEEL_DIAMETER_CM 5.5f
#define WHEEL_BASE_CM 9.0f
#define COUNTS_PER_CM ((float)COUNTS_PER_REV / (3.14159265f * WHEEL_DIAMETER_CM))
#define PID_KP 2.0f
#define PID_KI 0.1f
#define PID_KD 0.3f
#define PID_INTEGRAL_LIMIT 400.0f
#define SPEED_FILTER_ALPHA 0.3f

#define MOTOR_A_PWM_GPIO GPIO_NUM_1
#define MOTOR_A_IN1_GPIO GPIO_NUM_42
#define MOTOR_A_IN2_GPIO GPIO_NUM_2
#define MOTOR_A_ENC_A_GPIO GPIO_NUM_41
#define MOTOR_A_ENC_B_GPIO GPIO_NUM_40

#define MOTOR_B_PWM_GPIO GPIO_NUM_4
#define MOTOR_B_IN1_GPIO GPIO_NUM_5
#define MOTOR_B_IN2_GPIO GPIO_NUM_6
#define MOTOR_B_ENC_A_GPIO GPIO_NUM_8
#define MOTOR_B_ENC_B_GPIO GPIO_NUM_3

#define MOTOR_D_PWM_GPIO GPIO_NUM_7
#define MOTOR_D_IN1_GPIO GPIO_NUM_16
#define MOTOR_D_IN2_GPIO GPIO_NUM_15
#define MOTOR_D_ENC_A_GPIO GPIO_NUM_17
#define MOTOR_D_ENC_B_GPIO GPIO_NUM_18

#define SENSOR_COUNT 4
#define SENSOR_CH1_GPIO GPIO_NUM_11
#define SENSOR_CH2_GPIO GPIO_NUM_12
#define SENSOR_CH3_GPIO GPIO_NUM_13
#define SENSOR_CH4_GPIO GPIO_NUM_14

#define HC_SR04_TRIG_GPIO GPIO_NUM_9
#define HC_SR04_ECHO_GPIO GPIO_NUM_10
#define HC_SR04_TIMEOUT_US 25000
#define ULTRASONIC_PERIOD_MS 60
#define ULTRASONIC_MIN_VALID_CM 2.0f
#define ULTRASONIC_MAX_VALID_CM 400.0f
#define OBSTACLE_DISTANCE_CM 6.0f
#define OBSTACLE_CONFIRM_COUNT 3

#define CONTROL_PERIOD_MS 10
#define BASE_VX_CM_S 10.0f
#define LINE_KP 0.4f
#define MAX_WZ_RAD_S 1.5f

#define AVOID_STOP_MS 200
#define AVOID_SIDE_SPEED_CM_S 15.0f
#define AVOID_FORWARD_SPEED_CM_S 18.0f
#define AVOID_CLEAR_CONFIRM_COUNT 3
#define AVOID_EXTRA_LEFT_MS 700
#define AVOID_FORWARD_MS 2000
#define LINE_REACQUIRE_CONFIRM_COUNT 3

#define LOST_LINE_TURN_SPEED_RAD_S 0.8f
#define LOST_LINE_SEARCH_RIGHT_ANGLE_RAD (70.0f * 3.14159265f / 180.0f)
#define LOST_LINE_SEARCH_LEFT_ANGLE_RAD (140.0f * 3.14159265f / 180.0f)
#define LOST_LINE_SEARCH_RIGHT_TIMEOUT_MS 3000
#define LOST_LINE_SEARCH_LEFT_TIMEOUT_MS 5000
#define LOST_LINE_CONFIRM_COUNT 3

_Static_assert(HC_SR04_TRIG_GPIO != HC_SR04_ECHO_GPIO &&
                   HC_SR04_TRIG_GPIO != SENSOR_CH1_GPIO &&
                   HC_SR04_TRIG_GPIO != SENSOR_CH2_GPIO &&
                   HC_SR04_TRIG_GPIO != SENSOR_CH3_GPIO &&
                   HC_SR04_TRIG_GPIO != SENSOR_CH4_GPIO &&
                   HC_SR04_ECHO_GPIO != SENSOR_CH1_GPIO &&
                   HC_SR04_ECHO_GPIO != SENSOR_CH2_GPIO &&
                   HC_SR04_ECHO_GPIO != SENSOR_CH3_GPIO &&
                   HC_SR04_ECHO_GPIO != SENSOR_CH4_GPIO,
               "HC-SR04 GPIO conflicts with line sensor GPIO");

typedef enum
{
    STATE_LINE_FOLLOW,
    STATE_AVOID_STOP,
    STATE_AVOID_LEFT,
    STATE_AVOID_EXTRA_LEFT,
    STATE_AVOID_FORWARD,
    STATE_SEARCH_LINE,
    STATE_LOST_LINE_SEARCH_RIGHT,
    STATE_LOST_LINE_SEARCH_LEFT,
    STATE_SEARCH_FAILED,
} robot_state_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_measured;
} pid_t;

static float pid_update(pid_t *pid, float error, float measured, bool saturated)
{
    float p_term = pid->kp * error;

    if (!saturated)
    {
        pid->integral += error;
        if (pid->integral > PID_INTEGRAL_LIMIT)
        {
            pid->integral = PID_INTEGRAL_LIMIT;
        }
        else if (pid->integral < -PID_INTEGRAL_LIMIT)
        {
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

typedef struct
{
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
    {MOTOR_A_PWM_GPIO, MOTOR_A_IN1_GPIO, MOTOR_A_IN2_GPIO, MOTOR_A_ENC_A_GPIO, MOTOR_A_ENC_B_GPIO, LEDC_CHANNEL_0, NULL, 0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
    {MOTOR_B_PWM_GPIO, MOTOR_B_IN1_GPIO, MOTOR_B_IN2_GPIO, MOTOR_B_ENC_A_GPIO, MOTOR_B_ENC_B_GPIO, LEDC_CHANNEL_1, NULL, 0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
    {MOTOR_D_PWM_GPIO, MOTOR_D_IN1_GPIO, MOTOR_D_IN2_GPIO, MOTOR_D_ENC_A_GPIO, MOTOR_D_ENC_B_GPIO, LEDC_CHANNEL_2, NULL, 0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
};

static const int sensor_gpios[SENSOR_COUNT] = {
    SENSOR_CH1_GPIO,
    SENSOR_CH2_GPIO,
    SENSOR_CH3_GPIO,
    SENSOR_CH4_GPIO,
};

static const int sensor_weights[SENSOR_COUNT] = {-3, -1, 1, 3};

typedef struct
{
    float distance_cm;
    uint32_t sample_id;
    bool valid;
} ultrasonic_snapshot_t;

static portMUX_TYPE ultrasonic_lock = portMUX_INITIALIZER_UNLOCKED;
static ultrasonic_snapshot_t ultrasonic_snapshot;

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
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
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
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        gpio_set_level(motors[i].in1_gpio, 0);
        gpio_set_level(motors[i].in2_gpio, 0);
    }

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
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

static void sensor_init(void)
{
    uint64_t sensor_pin_mask = 0;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
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

static void ultrasonic_init(void)
{
    gpio_config_t trig_conf = {
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_conf));

    gpio_config_t echo_conf = {
        .pin_bit_mask = 1ULL << HC_SR04_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_conf));
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);
}

static bool ultrasonic_read_cm(float *distance_cm)
{
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(HC_SR04_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);

    int64_t deadline_us = esp_timer_get_time() + HC_SR04_TIMEOUT_US;
    while (gpio_get_level(HC_SR04_ECHO_GPIO) == 0)
    {
        if (esp_timer_get_time() >= deadline_us)
        {
            return false;
        }
    }

    int64_t echo_start_us = esp_timer_get_time();
    while (gpio_get_level(HC_SR04_ECHO_GPIO) == 1)
    {
        if (esp_timer_get_time() >= deadline_us)
        {
            return false;
        }
    }

    *distance_cm = (float)(esp_timer_get_time() - echo_start_us) / 58.0f;
    return *distance_cm >= ULTRASONIC_MIN_VALID_CM &&
           *distance_cm <= ULTRASONIC_MAX_VALID_CM;
}

static void ultrasonic_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (true)
    {
        float distance_cm = 0.0f;
        bool valid = ultrasonic_read_cm(&distance_cm);

        portENTER_CRITICAL(&ultrasonic_lock);
        ultrasonic_snapshot.distance_cm = distance_cm;
        ultrasonic_snapshot.valid = valid;
        ultrasonic_snapshot.sample_id++;
        portEXIT_CRITICAL(&ultrasonic_lock);

        tft_display_set_distance(valid, distance_cm);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
    }
}

static ultrasonic_snapshot_t ultrasonic_get_snapshot(void)
{
    ultrasonic_snapshot_t snapshot;
    portENTER_CRITICAL(&ultrasonic_lock);
    snapshot = ultrasonic_snapshot;
    portEXIT_CRITICAL(&ultrasonic_lock);
    return snapshot;
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

    if (new_target == 0)
    {
        motors[index].target_speed = 0;
        motor_coast(index);
        pid_reset(&motors[index].pid);
        return;
    }

    if ((new_target > 0) != (motors[index].target_speed > 0))
    {
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

static int motion_start_counts[MOTOR_COUNT];

static void motion_reset(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motion_start_counts[i] = encoder_get_count(i);
    }
}

static float motion_get_rotation(void)
{
    float da = (encoder_get_count(0) - motion_start_counts[0]) / COUNTS_PER_CM;
    float db = (encoder_get_count(1) - motion_start_counts[1]) / COUNTS_PER_CM;
    float dd = (encoder_get_count(2) - motion_start_counts[2]) / COUNTS_PER_CM;

    return (da + db + dd) / (3.0f * WHEEL_BASE_CM);
}

static void speed_ctrl_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            int count = encoder_get_count(i);
            int delta = count - motors[i].last_count;
            motors[i].last_count = count;

            float measured_raw = (float)delta * 1000.0f / SPEED_CTRL_PERIOD_MS;
            motors[i].measured_speed += (measured_raw - motors[i].measured_speed) * SPEED_FILTER_ALPHA;
            float measured = motors[i].measured_speed;

            float target = (float)motors[i].target_speed;
            if (target == 0)
            {
                continue;
            }

            float error = fabsf(target) - fabsf(measured);
            float output = pid_update(&motors[i].pid, error, fabsf(measured), motors[i].saturated);

            uint32_t duty;
            motors[i].saturated = false;
            if (output < 0)
            {
                duty = 0;
                motors[i].saturated = true;
            }
            else if (output > MAX_DUTY)
            {
                duty = MAX_DUTY;
                motors[i].saturated = true;
            }
            else
            {
                duty = (uint32_t)output;
            }

            gpio_set_level(motors[i].in1_gpio, target > 0);
            gpio_set_level(motors[i].in2_gpio, target < 0);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[i].pwm_channel, duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[i].pwm_channel));
        }

        tft_display_set_motor_speeds(motors[0].measured_speed / COUNTS_PER_CM,
                                     motors[1].measured_speed / COUNTS_PER_CM,
                                     motors[2].measured_speed / COUNTS_PER_CM);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SPEED_CTRL_PERIOD_MS));
    }
}

static void set_speed(float vx, float vy, float wz)
{
    float va = 0.86602540378f * vx + 0.5f * vy + WHEEL_BASE_CM * wz;
    float vd = -0.86602540378f * vx + 0.5f * vy + WHEEL_BASE_CM * wz;
    float vb = -vy + WHEEL_BASE_CM * wz;

    motor_set_target_cm_s(0, va);
    motor_set_target_cm_s(1, vb);
    motor_set_target_cm_s(2, vd);
}

static void stop_and_brake(void)
{
    set_speed(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motor_brake(i);
    }
}

static bool read_line_error(int *error_scaled)
{
    int error_sum = 0;
    int black_count = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        int level = gpio_get_level(sensor_gpios[i]);
        if (level == 0)
        {
            error_sum += sensor_weights[i];
            black_count++;
        }
    }

    if (black_count == 0)
    {
        return false;
    }

    *error_scaled = error_sum * 100 / black_count;
    return true;
}

static void enter_state(robot_state_t *state, robot_state_t next, int64_t *entered_ms)
{
    *state = next;
    *entered_ms = esp_timer_get_time() / 1000;
    motion_reset();
    ESP_LOGI(TAG, "state -> %d", next);
}

void app_main(void)
{
    motors_init();
    sensor_init();
    ultrasonic_init();
    ESP_ERROR_CHECK(tft_display_start());
    ESP_LOGI(TAG, "motors, line sensor, HC-SR04 and TFT initialized");

    xTaskCreate(speed_ctrl_task, "speed_ctrl", 4096, NULL, 5, NULL);
    xTaskCreate(ultrasonic_task, "ultrasonic", 2048, NULL, 3, NULL);

    robot_state_t state = STATE_LINE_FOLLOW;
    int64_t state_entered_ms = esp_timer_get_time() / 1000;
    uint32_t last_ultrasonic_sample_id = 0;
    bool line_ok = false;
    int obstacle_count = 0;
    int obstacle_clear_count = 0;
    int reacquire_count = 0;
    int lost_line_count = 0;
    bool stop_on_line_loss_after_avoidance = false;
    uint32_t tick = 0;

    while (1)
    {
        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t state_ms = now_ms - state_entered_ms;
        int error = 0;
        bool ok = read_line_error(&error);

        ultrasonic_snapshot_t distance_sample = ultrasonic_get_snapshot();
        bool new_distance_sample = distance_sample.sample_id != last_ultrasonic_sample_id;

        if (state == STATE_LINE_FOLLOW && new_distance_sample)
        {
            last_ultrasonic_sample_id = distance_sample.sample_id;
            if (distance_sample.valid && distance_sample.distance_cm < OBSTACLE_DISTANCE_CM)
            {
                obstacle_count++;
                if (obstacle_count >= OBSTACLE_CONFIRM_COUNT)
                {
                    ESP_LOGW(TAG, "obstacle at %.1f cm", distance_sample.distance_cm);
                    enter_state(&state, STATE_AVOID_STOP, &state_entered_ms);
                    state_ms = 0;
                    obstacle_count = 0;
                    set_speed(0.0f, 0.0f, 0.0f);
                    for (int i = 0; i < MOTOR_COUNT; i++)
                    {
                        motor_brake(i);
                    }
                }
            }
            else
            {
                obstacle_count = 0;
            }
        }

        if (state == STATE_LINE_FOLLOW && ok)
        {
            if (!line_ok)
            {
                ESP_LOGI(TAG, "line detected");
            }
            line_ok = true;
            lost_line_count = 0;

            float wz = LINE_KP * (float)error / 100.0f;
            if (wz > MAX_WZ_RAD_S)
            {
                wz = MAX_WZ_RAD_S;
            }
            if (wz < -MAX_WZ_RAD_S)
            {
                wz = -MAX_WZ_RAD_S;
            }

            set_speed(BASE_VX_CM_S, 0.0f, wz);

            if (++tick % 100 == 0)
            {
                ESP_LOGI(TAG, "error=%d wz=%.2f | m1=%.1f m2=%.1f m3=%.1f cm/s",
                         error, wz,
                         motors[0].measured_speed / COUNTS_PER_CM,
                         motors[1].measured_speed / COUNTS_PER_CM,
                         motors[2].measured_speed / COUNTS_PER_CM);
            }
        }
        else if (state == STATE_LINE_FOLLOW)
        {
            lost_line_count++;
            if (lost_line_count >= LOST_LINE_CONFIRM_COUNT)
            {
                line_ok = false;
                reacquire_count = 0;
                if (stop_on_line_loss_after_avoidance)
                {
                    ESP_LOGW(TAG, "line lost after avoidance, stop without turning");
                    enter_state(&state, STATE_SEARCH_FAILED, &state_entered_ms);
                    stop_and_brake();
                }
                else
                {
                    ESP_LOGW(TAG, "line lost, search up to 70 degrees right");
                    enter_state(&state, STATE_LOST_LINE_SEARCH_RIGHT, &state_entered_ms);
                    set_speed(0.0f, 0.0f, -LOST_LINE_TURN_SPEED_RAD_S);
                }
            }
        }
        else if (state == STATE_AVOID_STOP)
        {
            if (state_ms >= AVOID_STOP_MS)
            {
                obstacle_clear_count = 0;
                enter_state(&state, STATE_AVOID_LEFT, &state_entered_ms);
            }
        }
        else if (state == STATE_AVOID_LEFT)
        {
            set_speed(0.0f, AVOID_SIDE_SPEED_CM_S, 0.0f);

            if (new_distance_sample)
            {
                last_ultrasonic_sample_id = distance_sample.sample_id;
                if (distance_sample.valid && distance_sample.distance_cm < OBSTACLE_DISTANCE_CM)
                {
                    obstacle_clear_count = 0;
                }
                else if (++obstacle_clear_count >= AVOID_CLEAR_CONFIRM_COUNT)
                {
                    if (distance_sample.valid)
                    {
                        ESP_LOGI(TAG, "obstacle cleared at %.1f cm, continue left for 0.7 seconds",
                                 distance_sample.distance_cm);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "obstacle cleared (no echo), continue left for 0.7 seconds");
                    }
                    enter_state(&state, STATE_AVOID_EXTRA_LEFT, &state_entered_ms);
                }
            }
        }
        else if (state == STATE_AVOID_EXTRA_LEFT)
        {
            set_speed(0.0f, AVOID_SIDE_SPEED_CM_S, 0.0f);

            if (new_distance_sample)
            {
                last_ultrasonic_sample_id = distance_sample.sample_id;
                if (distance_sample.valid && distance_sample.distance_cm < OBSTACLE_DISTANCE_CM)
                {
                    ESP_LOGW(TAG, "obstacle detected again at %.1f cm while moving left",
                             distance_sample.distance_cm);
                    obstacle_clear_count = 0;
                    enter_state(&state, STATE_AVOID_LEFT, &state_entered_ms);
                }
            }

            if (state == STATE_AVOID_EXTRA_LEFT && state_ms >= AVOID_EXTRA_LEFT_MS)
            {
                enter_state(&state, STATE_AVOID_FORWARD, &state_entered_ms);
            }
        }
        else if (state == STATE_AVOID_FORWARD)
        {
            set_speed(AVOID_FORWARD_SPEED_CM_S, 0.0f, 0.0f);

            if (new_distance_sample)
            {
                last_ultrasonic_sample_id = distance_sample.sample_id;
                if (distance_sample.valid && distance_sample.distance_cm < OBSTACLE_DISTANCE_CM)
                {
                    ESP_LOGW(TAG, "obstacle detected again at %.1f cm while moving forward",
                             distance_sample.distance_cm);
                    obstacle_clear_count = 0;
                    enter_state(&state, STATE_AVOID_LEFT, &state_entered_ms);
                }
            }

            if (state == STATE_AVOID_FORWARD && state_ms >= AVOID_FORWARD_MS)
            {
                reacquire_count = 0;
                enter_state(&state, STATE_SEARCH_LINE, &state_entered_ms);
            }
        }
        else if (state == STATE_SEARCH_LINE)
        {
            set_speed(0.0f, -AVOID_SIDE_SPEED_CM_S, 0.0f);

            if (ok)
            {
                reacquire_count++;
                if (reacquire_count >= LINE_REACQUIRE_CONFIRM_COUNT)
                {
                    ESP_LOGI(TAG, "line reacquired, resume line following");
                    line_ok = true;
                    lost_line_count = 0;
                    stop_on_line_loss_after_avoidance = true;
                    enter_state(&state, STATE_LINE_FOLLOW, &state_entered_ms);
                }
            }
            else
            {
                reacquire_count = 0;
            }
        }
        else if (state == STATE_LOST_LINE_SEARCH_RIGHT)
        {
            float turned_angle = fabsf(motion_get_rotation());
            set_speed(0.0f, 0.0f, -LOST_LINE_TURN_SPEED_RAD_S);

            if (ok)
            {
                reacquire_count++;
                if (reacquire_count >= LINE_REACQUIRE_CONFIRM_COUNT)
                {
                    ESP_LOGI(TAG, "line found while turning right at %.1f degrees",
                             turned_angle * 180.0f / 3.14159265f);
                    line_ok = true;
                    lost_line_count = 0;
                    enter_state(&state, STATE_LINE_FOLLOW, &state_entered_ms);
                }
            }
            else
            {
                reacquire_count = 0;
            }

            if (state == STATE_LOST_LINE_SEARCH_RIGHT &&
                (turned_angle >= LOST_LINE_SEARCH_RIGHT_ANGLE_RAD ||
                 state_ms >= LOST_LINE_SEARCH_RIGHT_TIMEOUT_MS))
            {
                ESP_LOGW(TAG, "line not found to the right, search up to 140 degrees left");
                reacquire_count = 0;
                enter_state(&state, STATE_LOST_LINE_SEARCH_LEFT, &state_entered_ms);
                set_speed(0.0f, 0.0f, LOST_LINE_TURN_SPEED_RAD_S);
            }
        }
        else if (state == STATE_LOST_LINE_SEARCH_LEFT)
        {
            float turned_angle = fabsf(motion_get_rotation());
            set_speed(0.0f, 0.0f, LOST_LINE_TURN_SPEED_RAD_S);

            if (ok)
            {
                reacquire_count++;
                if (reacquire_count >= LINE_REACQUIRE_CONFIRM_COUNT)
                {
                    ESP_LOGI(TAG, "line found while turning left at %.1f degrees",
                             turned_angle * 180.0f / 3.14159265f);
                    line_ok = true;
                    lost_line_count = 0;
                    enter_state(&state, STATE_LINE_FOLLOW, &state_entered_ms);
                }
            }
            else
            {
                reacquire_count = 0;
            }

            if (state == STATE_LOST_LINE_SEARCH_LEFT &&
                (turned_angle >= LOST_LINE_SEARCH_LEFT_ANGLE_RAD ||
                 state_ms >= LOST_LINE_SEARCH_LEFT_TIMEOUT_MS))
            {
                ESP_LOGE(TAG, "line not found after one right and one left search, stop");
                enter_state(&state, STATE_SEARCH_FAILED, &state_entered_ms);
                stop_and_brake();
            }
        }
        else if (state == STATE_SEARCH_FAILED)
        {
            // Terminal state: stay stopped after the single right/left search cycle.
            stop_and_brake();
        }
        else
        {
            set_speed(0.0f, 0.0f, 0.0f);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
