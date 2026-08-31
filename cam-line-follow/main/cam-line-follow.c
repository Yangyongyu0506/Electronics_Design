#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "cooperative_jpeg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "libuvc/libuvc.h"
#include "libuvc_adapter.h"
#include "usb/usb_host.h"

static const char *TAG = "line-follow";

#define MAX_MODE_CANDIDATES          32
#define PREFERRED_WIDTH              320
#define PREFERRED_HEIGHT             240
#define PREFERRED_FPS                10
#define MAX_SOURCE_WIDTH             640
#define MAX_SOURCE_HEIGHT            480
#define DECODE_SCALE_DIV             8
#define DECODE_BUF_BYTES             ((PREFERRED_WIDTH / 2) * (PREFERRED_HEIGHT / 2) * 2)
#define JPEG_FRAME_MAX_BYTES         (512 * 1024)
#define FRAME_SLOT_COUNT             2
#define LINE_LUM_THRESHOLD           100
#define LINE_MIN_BLACK_PIXELS        4
#define LINE_ROI_FRACTION            6

#define CAMERA_FLIPPED               1

#define LINE_KP                      1.8f
#define LINE_KD                      0.035f
#define LINE_MAX_WZ                  1.2f
#define LINE_DEADBAND                0.06f
#define LINE_SPEED_MAX_CM_S          10.0f
#define LINE_SPEED_MIN_CM_S          1.5f
#define LINE_PIVOT_ERROR             0.72f
#define LINE_LOST_STOP_DELAY_MS      500
#define LINE_FRAME_TIMEOUT_MS        300
#define LINE_STEER_SIGN              1.0f
#define LINE_PERIOD_MS               20

#define HC_SR04_TRIG_GPIO            GPIO_NUM_9
#define HC_SR04_ECHO_GPIO            GPIO_NUM_10
#define HC_SR04_TIMEOUT_US           25000
#define ULTRASONIC_PERIOD_MS         60
#define ULTRASONIC_MIN_VALID_CM      2.0f
#define ULTRASONIC_MAX_VALID_CM      400.0f
#define OBSTACLE_DISTANCE_CM         6.0f
#define OBSTACLE_CONFIRM_COUNT       3
#define AVOID_STOP_MS                200
#define AVOID_SIDE_SPEED_CM_S        15.0f
#define AVOID_FORWARD_SPEED_CM_S     18.0f
#define AVOID_CLEAR_CONFIRM_COUNT    3
#define AVOID_EXTRA_LEFT_MS          700
#define AVOID_FORWARD_MS             1200
#define LINE_REACQUIRE_CONFIRM_COUNT 3
#define LINE_LOSS_CONFIRM_COUNT      3

#define MOTOR_COUNT   3
#define PWM_FREQ_HZ   20000
#define MAX_DUTY      1023
#define ENC_HIGH_LIMIT  1000
#define ENC_LOW_LIMIT   -1000

#define SPEED_CTRL_PERIOD_MS      20
#define COUNTS_PER_REV            512
#define WHEEL_DIAMETER_CM         5.5f
#define WHEEL_BASE_CM             9.0f
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

#define WHEEL_A_DRIVE_DEG 30.0f
#define WHEEL_B_DRIVE_DEG 270.0f
#define WHEEL_D_DRIVE_DEG 150.0f

#define UVC_CHECK(expression) do {                                      \
    uvc_error_t error_ = (expression);                                  \
    if (error_ < 0) {                                                   \
        ESP_LOGE(TAG, "UVC error: %s", uvc_strerror(error_));       \
        assert(false);                                                  \
    }                                                                   \
} while (0)

static EventGroupHandle_t s_app_flags;
static uint16_t s_frame_width;
static uint16_t s_frame_height;
static uint8_t s_decode_scale_divisor;
static uint8_t s_decode_buf[DECODE_BUF_BYTES];

typedef struct {
    uint8_t *data;
    size_t len;
} frame_slot_t;

static frame_slot_t s_frame_slots[FRAME_SLOT_COUNT];
static QueueHandle_t s_free_frames;
static QueueHandle_t s_ready_frames;
static volatile uint32_t s_callback_drops;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_measured;
} pid_ctrl_t;

static float pid_update(pid_ctrl_t *pid, float error, float measured, bool saturated)
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

static void pid_reset(pid_ctrl_t *pid)
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
    pid_ctrl_t pid;
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

typedef struct {
    int32_t black_left;
    int32_t black_right;
    int32_t lateral_q1000;
    int32_t heading_q1000;
    int64_t updated_us;
} line_state_t;

static portMUX_TYPE s_line_mux = portMUX_INITIALIZER_UNLOCKED;
static line_state_t s_line = {0};
static volatile bool s_avoid_active;

typedef enum {
    AVOID_IDLE,
    AVOID_STOP,
    AVOID_LEFT,
    AVOID_EXTRA_LEFT,
    AVOID_FORWARD,
    AVOID_SEARCH_LINE,
    AVOID_DRIVE_UNTIL_LINE_LOST,
    AVOID_FINISHED_STOP,
} avoid_state_t;

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

static void set_speed(float vx, float vy, float wz)
{
    float va = 0.86602540378 * vx + 0.5 * vy + WHEEL_BASE_CM * wz;
    float vd = -0.86602540378 * vx + 0.5 * vy + WHEEL_BASE_CM * wz;
    float vb = -vy + WHEEL_BASE_CM * wz;

    motor_set_target_cm_s(0, va);
    motor_set_target_cm_s(1, vb);
    motor_set_target_cm_s(2, vd);
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
    while (gpio_get_level(HC_SR04_ECHO_GPIO) == 0) {
        if (esp_timer_get_time() >= deadline_us) {
            return false;
        }
    }

    int64_t echo_start_us = esp_timer_get_time();
    while (gpio_get_level(HC_SR04_ECHO_GPIO) == 1) {
        if (esp_timer_get_time() >= deadline_us) {
            return false;
        }
    }

    *distance_cm = (float)(esp_timer_get_time() - echo_start_us) / 58.0f;
    return *distance_cm >= ULTRASONIC_MIN_VALID_CM &&
           *distance_cm <= ULTRASONIC_MAX_VALID_CM;
}

static void avoid_enter_state(avoid_state_t *state, avoid_state_t next,
                              int64_t *entered_ms)
{
    *state = next;
    *entered_ms = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "avoid state -> %d", (int)next);
}

static void ultrasonic_avoid_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    avoid_state_t state = AVOID_IDLE;
    int64_t state_entered_ms = esp_timer_get_time() / 1000;
    int obstacle_count = 0;
    int clear_count = 0;
    int reacquire_count = 0;
    int line_lost_count = 0;
    int64_t last_line_frame_us = 0;

    while (true) {
        float distance_cm = 0.0f;
        bool distance_valid = ultrasonic_read_cm(&distance_cm);
        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t state_ms = now_ms - state_entered_ms;

        if (state == AVOID_IDLE) {
            if (distance_valid && distance_cm < OBSTACLE_DISTANCE_CM) {
                if (++obstacle_count >= OBSTACLE_CONFIRM_COUNT) {
                    ESP_LOGW(TAG, "obstacle at %.1f cm", (double)distance_cm);
                    s_avoid_active = true;
                    set_speed(0.0f, 0.0f, 0.0f);
                    avoid_enter_state(&state, AVOID_STOP, &state_entered_ms);
                    obstacle_count = 0;
                }
            } else {
                obstacle_count = 0;
            }
        } else if (state == AVOID_STOP) {
            set_speed(0.0f, 0.0f, 0.0f);
            if (state_ms >= AVOID_STOP_MS) {
                clear_count = 0;
                avoid_enter_state(&state, AVOID_LEFT, &state_entered_ms);
            }
        } else if (state == AVOID_LEFT) {
            set_speed(0.0f, AVOID_SIDE_SPEED_CM_S, 0.0f);
            if (distance_valid && distance_cm < OBSTACLE_DISTANCE_CM) {
                clear_count = 0;
            } else if (++clear_count >= AVOID_CLEAR_CONFIRM_COUNT) {
                ESP_LOGI(TAG, "obstacle cleared, continue left for %d ms",
                         AVOID_EXTRA_LEFT_MS);
                avoid_enter_state(&state, AVOID_EXTRA_LEFT, &state_entered_ms);
            }
        } else if (state == AVOID_EXTRA_LEFT) {
            set_speed(0.0f, AVOID_SIDE_SPEED_CM_S, 0.0f);
            if (distance_valid && distance_cm < OBSTACLE_DISTANCE_CM) {
                clear_count = 0;
                avoid_enter_state(&state, AVOID_LEFT, &state_entered_ms);
            } else if (state_ms >= AVOID_EXTRA_LEFT_MS) {
                avoid_enter_state(&state, AVOID_FORWARD, &state_entered_ms);
            }
        } else if (state == AVOID_FORWARD) {
            set_speed(AVOID_FORWARD_SPEED_CM_S, 0.0f, 0.0f);
            if (distance_valid && distance_cm < OBSTACLE_DISTANCE_CM) {
                clear_count = 0;
                avoid_enter_state(&state, AVOID_LEFT, &state_entered_ms);
            } else if (state_ms >= AVOID_FORWARD_MS) {
                reacquire_count = 0;
                last_line_frame_us = 0;
                avoid_enter_state(&state, AVOID_SEARCH_LINE, &state_entered_ms);
            }
        } else if (state == AVOID_SEARCH_LINE) {
            set_speed(0.0f, -AVOID_SIDE_SPEED_CM_S, 0.0f);

            int32_t total = 0;
            int64_t updated_us = 0;
            portENTER_CRITICAL(&s_line_mux);
            total = s_line.black_left + s_line.black_right;
            updated_us = s_line.updated_us;
            portEXIT_CRITICAL(&s_line_mux);

            if (updated_us != 0 && updated_us != last_line_frame_us) {
                last_line_frame_us = updated_us;
                if (total >= LINE_MIN_BLACK_PIXELS) {
                    if (++reacquire_count >= LINE_REACQUIRE_CONFIRM_COUNT) {
                        ESP_LOGI(TAG, "line reacquired, drive straight until line ends");
                        line_lost_count = 0;
                        avoid_enter_state(&state, AVOID_DRIVE_UNTIL_LINE_LOST,
                                          &state_entered_ms);
                    }
                } else {
                    reacquire_count = 0;
                }
            }
        } else if (state == AVOID_DRIVE_UNTIL_LINE_LOST) {
            /* Do not invoke camera steering after avoidance: travel straight
               over the reacquired line and stop once it ends. */
            set_speed(AVOID_FORWARD_SPEED_CM_S, 0.0f, 0.0f);

            int32_t total = 0;
            int64_t updated_us = 0;
            portENTER_CRITICAL(&s_line_mux);
            total = s_line.black_left + s_line.black_right;
            updated_us = s_line.updated_us;
            portEXIT_CRITICAL(&s_line_mux);

            if (updated_us != 0 && updated_us != last_line_frame_us) {
                last_line_frame_us = updated_us;
                if (total < LINE_MIN_BLACK_PIXELS) {
                    if (++line_lost_count >= LINE_LOSS_CONFIRM_COUNT) {
                        ESP_LOGI(TAG, "line ended after avoidance, stopping");
                        set_speed(0.0f, 0.0f, 0.0f);
                        avoid_enter_state(&state, AVOID_FINISHED_STOP,
                                          &state_entered_ms);
                    }
                } else {
                    line_lost_count = 0;
                }
            }
        } else if (state == AVOID_FINISHED_STOP) {
            set_speed(0.0f, 0.0f, 0.0f);
        }

        static uint32_t log_skip;
        if ((log_skip++ % 10) == 0) {
            if (distance_valid) {
                ESP_LOGI(TAG, "ultrasonic: %.1f cm avoid=%d",
                         (double)distance_cm, (int)state);
            } else {
                ESP_LOGI(TAG, "ultrasonic: no echo avoid=%d", (int)state);
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
    }
}

static void line_ctrl_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    float last_err = 0.0f;
    int64_t last_t_us = 0;
    int64_t line_lost_since_us = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LINE_PERIOD_MS));

        if (s_avoid_active) {
            last_err = 0.0f;
            last_t_us = 0;
            line_lost_since_us = 0;
            continue;
        }

        int32_t left = 0;
        int32_t right = 0;
        int32_t lateral_q1000 = 0;
        int32_t heading_q1000 = 0;
        int64_t updated_us = 0;
        portENTER_CRITICAL(&s_line_mux);
        left = s_line.black_left;
        right = s_line.black_right;
        lateral_q1000 = s_line.lateral_q1000;
        heading_q1000 = s_line.heading_q1000;
        updated_us = s_line.updated_us;
        portEXIT_CRITICAL(&s_line_mux);

        int64_t now_us = esp_timer_get_time();
        int32_t total = left + right;
        bool frame_stale = updated_us == 0 ||
                           now_us - updated_us > LINE_FRAME_TIMEOUT_MS * 1000LL;
        if (frame_stale) {
            set_speed(0.0f, 0.0f, 0.0f);
            last_err = 0.0f;
            last_t_us = 0;
            line_lost_since_us = 0;
            continue;
        }

        if (total < LINE_MIN_BLACK_PIXELS) {
            if (line_lost_since_us == 0) {
                line_lost_since_us = now_us;
            }
            /* Keep the last motor command briefly so small gaps in the line
               do not stop the car. Never extend this delay for stale frames. */
            if (now_us - line_lost_since_us >=
                LINE_LOST_STOP_DELAY_MS * 1000LL) {
                set_speed(0.0f, 0.0f, 0.0f);
                last_err = 0.0f;
                last_t_us = 0;
            }
            continue;
        }
        line_lost_since_us = 0;

        float lateral = lateral_q1000 / 1000.0f;
        float heading = heading_q1000 / 1000.0f;
        /* Deliberately do not use heading/look-ahead here. With a 90-degree
           corner it makes the car cut across as soon as the outgoing line is
           visible. Steering must come only from the line under the nose. */
        float err = lateral;

        float dt = (last_t_us == 0) ? (LINE_PERIOD_MS / 1000.0f)
                                    : (float)(now_us - last_t_us) / 1000000.0f;
        if (dt <= 0.0f) {
            dt = LINE_PERIOD_MS / 1000.0f;
        }

        float wz = 0.0f;
        if (fabsf(err) > LINE_DEADBAND) {
            float derr = (err - last_err) / dt;
            wz = LINE_STEER_SIGN * (LINE_KP * err + LINE_KD * derr);
            if (wz > LINE_MAX_WZ) {
                wz = LINE_MAX_WZ;
            } else if (wz < -LINE_MAX_WZ) {
                wz = -LINE_MAX_WZ;
            }
        }

        float abs_err = fminf(fabsf(err), 1.0f);
        float vx = LINE_SPEED_MAX_CM_S -
                   (LINE_SPEED_MAX_CM_S - LINE_SPEED_MIN_CM_S) * abs_err;
        if (abs_err >= LINE_PIVOT_ERROR) {
            vx = 0.0f;
        }
        set_speed(vx, 0.0f, wz);

        static uint32_t s_line_log_skip;
        if ((s_line_log_skip++ % 10) == 0) {
            ESP_LOGI(TAG, "follow: lateral=%.2f heading=%.2f total=%d vx=%.1f wz=%.2f",
                     (double)lateral, (double)heading, (int)total,
                     (double)vx, (double)wz);
        }

        last_err = err;
        last_t_us = now_us;
    }
}

static bool is_mjpeg_format(const uvc_format_desc_t *format)
{
    return format->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG;
}

static bool is_black_pixel(int r, int g, int b)
{
    const int lum = (r + 2 * g + b) >> 2;
    return lum < LINE_LUM_THRESHOLD;
}

static void detect_line_mjpeg(const uint8_t *jpeg, size_t jpeg_len)
{
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)jpeg,
        .indata_size = jpeg_len,
        .outbuf = s_decode_buf,
        .outbuf_size = sizeof(s_decode_buf),
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_1_8,
        .flags = {
            .swap_color_bytes = 0,
        },
    };
    esp_jpeg_image_output_t image = {0};
    esp_err_t err = cooperative_jpeg_decode(&jpeg_cfg, &image);
    if (err != ESP_OK || image.width == 0 || image.height == 0) {
        ESP_LOGW(TAG, "JPEG decode failed: %s", esp_err_to_name(err));
        return;
    }

    static uint32_t s_log_skip;
    const bool log_now = (s_log_skip++ % 5) == 0;

    const uint16_t *decoded = (const uint16_t *)s_decode_buf;
    const uint32_t half = image.width / 2;
    const uint32_t y_start = CAMERA_FLIPPED ? 0 : (image.height * (LINE_ROI_FRACTION - 1)) / LINE_ROI_FRACTION;
    const uint32_t y_end = CAMERA_FLIPPED ? image.height / LINE_ROI_FRACTION : image.height;
    uint32_t black_left = 0;
    uint32_t black_right = 0;
    uint32_t black_count = 0;
    uint32_t black_x_sum = 0;

    for (uint32_t y = y_start; y < y_end; ++y) {
        for (uint32_t x = 0; x < image.width; ++x) {
            const uint16_t p = decoded[(size_t)y * image.width + x];
            const int r = ((p >> 11) & 0x1f) * 255 / 31;
            const int g = ((p >> 5) & 0x3f) * 255 / 63;
            const int b = (p & 0x1f) * 255 / 31;
            if (is_black_pixel(r, g, b)) {
                /* Express x in the car's view, not the upside-down sensor view. */
                uint32_t car_x = CAMERA_FLIPPED ? image.width - 1 - x : x;
                if (car_x < half) {
                    ++black_left;
                } else {
                    ++black_right;
                }

                black_x_sum += car_x;
                ++black_count;
            }
        }
    }

    const float center = (image.width - 1) * 0.5f;
    const float half_width = center > 0.0f ? center : 1.0f;
    float line_pos = 0.0f;
    if (black_count > 0) {
        line_pos = ((float)black_x_sum / black_count - center) / half_width;
    }

    /* Only the nearest eighth contributes. Positive control error means turn
       CCW, so a line to the car's right produces a negative error. */
    float lateral = -line_pos;
    float heading = 0.0f;

    portENTER_CRITICAL(&s_line_mux);
    s_line.black_left = (int32_t)black_left;
    s_line.black_right = (int32_t)black_right;
    s_line.lateral_q1000 = (int32_t)lroundf(lateral * 1000.0f);
    s_line.heading_q1000 = (int32_t)lroundf(heading * 1000.0f);
    s_line.updated_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_line_mux);

    if (log_now) {
        ESP_LOGI(TAG, "line: left=%u right=%u total=%u lateral=%.2f heading=%.2f",
                 (unsigned)black_left, (unsigned)black_right,
                 (unsigned)(black_left + black_right),
                 (double)lateral, (double)heading);
    }
}

static bool jpeg_has_valid_markers(const uint8_t *data, size_t len)
{
    return data != NULL && len >= 4 &&
           data[0] == 0xff && data[1] == 0xd8 &&
           data[len - 2] == 0xff && data[len - 1] == 0xd9;
}

static void frame_callback(uvc_frame_t *frame, void *user_ptr)
{
    (void)user_ptr;
    if (frame == NULL || frame->data == NULL || frame->data_bytes < 4 ||
        frame->data_bytes > JPEG_FRAME_MAX_BYTES ||
        frame->frame_format != UVC_FRAME_FORMAT_MJPEG ||
        frame->width != s_frame_width || frame->height != s_frame_height) {
        ++s_callback_drops;
        return;
    }

    uint8_t slot_index;
    if (xQueueReceive(s_free_frames, &slot_index, 0) != pdTRUE) {
        /* Processing is behind: replace the oldest frame that is still queued. */
        if (xQueueReceive(s_ready_frames, &slot_index, 0) != pdTRUE) {
            ++s_callback_drops;
            return;
        }
        ++s_callback_drops;
    }

    frame_slot_t *slot = &s_frame_slots[slot_index];
    memcpy(slot->data, frame->data, frame->data_bytes);
    slot->len = frame->data_bytes;

    if (xQueueSend(s_ready_frames, &slot_index, 0) != pdTRUE) {
        slot->len = 0;
        (void)xQueueSend(s_free_frames, &slot_index, 0);
        ++s_callback_drops;
    }
}

static void frame_processing_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    uint32_t last_reported_drops = 0;

    while (true) {
        uint8_t slot_index;
        if (xQueueReceive(s_ready_frames, &slot_index, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_ERROR_CHECK(esp_task_wdt_reset());
            continue;
        }

        frame_slot_t *slot = &s_frame_slots[slot_index];
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        if (slot->len <= JPEG_FRAME_MAX_BYTES &&
            jpeg_has_valid_markers(slot->data, slot->len)) {
            detect_line_mjpeg(slot->data, slot->len);
        } else {
            ESP_LOGW(TAG, "dropping incomplete/corrupt JPEG (%u bytes)",
                     (unsigned)slot->len);
        }
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        slot->len = 0;
        if (xQueueSend(s_free_frames, &slot_index, 0) != pdTRUE) {
            ESP_LOGE(TAG, "frame-pool queue corruption");
        }

        uint32_t drops = s_callback_drops;
        if (drops != last_reported_drops) {
            if (drops - last_reported_drops > FRAME_SLOT_COUNT * 2) {
                ESP_LOGW(TAG, "capture overloaded: %u frame(s) dropped; camera target is 10 FPS",
                         (unsigned)(drops - last_reported_drops));
            }
            last_reported_drops = drops;
        }

        /* JPEG decoding can keep CPU0 busy continuously when frames arrive
           back-to-back. Always yield so the watched IDLE0 task can run;
           wait a little longer when capture is already backed up. */
        if (uxQueueMessagesWaiting(s_ready_frames) > 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            vTaskDelay(1);
        }
    }
}

static esp_err_t initialize_frame_pipeline(void)
{
    s_free_frames = xQueueCreate(FRAME_SLOT_COUNT, sizeof(uint8_t));
    s_ready_frames = xQueueCreate(FRAME_SLOT_COUNT, sizeof(uint8_t));
    if (s_free_frames == NULL || s_ready_frames == NULL) {
        return ESP_ERR_NO_MEM;
    }

    for (uint8_t i = 0; i < FRAME_SLOT_COUNT; ++i) {
        s_frame_slots[i].data = heap_caps_malloc(JPEG_FRAME_MAX_BYTES,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_frame_slots[i].data == NULL) {
            ESP_LOGW(TAG, "PSRAM frame slot %u unavailable; using internal RAM",
                     (unsigned)i);
            s_frame_slots[i].data = heap_caps_malloc(JPEG_FRAME_MAX_BYTES,
                                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        if (s_frame_slots[i].data == NULL ||
            xQueueSend(s_free_frames, &i, 0) != pdTRUE) {
            ESP_LOGE(TAG, "unable to allocate frame slot %u", (unsigned)i);
            return ESP_ERR_NO_MEM;
        }
    }

    if (xTaskCreate(frame_processing_task, "frame_process", 6144,
                    NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static uvc_error_t negotiate_camera_mode(uvc_device_handle_t *devh,
                                         uvc_stream_ctrl_t *ctrl)
{
    const uvc_frame_desc_t *candidates[MAX_MODE_CANDIDATES];
    size_t count = 0;

    for (const uvc_format_desc_t *format = uvc_get_format_descs(devh);
         format != NULL; format = format->next) {
        if (!is_mjpeg_format(format)) {
            continue;
        }
        for (const uvc_frame_desc_t *frame = format->frame_descs;
             frame != NULL && count < MAX_MODE_CANDIDATES; frame = frame->next) {
            if (frame->wWidth <= MAX_SOURCE_WIDTH &&
                frame->wHeight <= MAX_SOURCE_HEIGHT) {
                candidates[count++] = frame;
            }
        }
    }

    ESP_LOGI(TAG, "trying preferred mode MJPEG %dx%d @ %d FPS",
             PREFERRED_WIDTH, PREFERRED_HEIGHT, PREFERRED_FPS);
    uvc_error_t preferred = uvc_get_stream_ctrl_format_size(
        devh, ctrl, UVC_FRAME_FORMAT_MJPEG,
        PREFERRED_WIDTH, PREFERRED_HEIGHT, PREFERRED_FPS);
    if (preferred == UVC_SUCCESS) {
        s_frame_width = PREFERRED_WIDTH;
        s_frame_height = PREFERRED_HEIGHT;
        s_decode_scale_divisor = DECODE_SCALE_DIV;
        return UVC_SUCCESS;
    }

    /* Prefer fallback resolutions closest to 320x240. */
    for (size_t i = 1; i < count; ++i) {
        const uvc_frame_desc_t *item = candidates[i];
        size_t j = i;
        uint32_t item_distance = (uint32_t)abs((int)item->wWidth - PREFERRED_WIDTH) +
                                 (uint32_t)abs((int)item->wHeight - PREFERRED_HEIGHT);
        while (j > 0) {
            uint32_t prev_distance =
                (uint32_t)abs((int)candidates[j - 1]->wWidth - PREFERRED_WIDTH) +
                (uint32_t)abs((int)candidates[j - 1]->wHeight - PREFERRED_HEIGHT);
            if (prev_distance <= item_distance) {
                break;
            }
            candidates[j] = candidates[j - 1];
            --j;
        }
        candidates[j] = item;
    }

    for (size_t i = 0; i < count; ++i) {
        const uvc_frame_desc_t *frame = candidates[i];
        int default_fps = frame->dwDefaultFrameInterval == 0 ? 0 :
                          (int)(10000000U / frame->dwDefaultFrameInterval);
        const int fps_attempts[] = {PREFERRED_FPS, default_fps};
        for (size_t attempt = 0; attempt < 2; ++attempt) {
            int fps = fps_attempts[attempt];
            if (attempt == 1 && fps == PREFERRED_FPS) {
                continue;
            }
            ESP_LOGI(TAG, "trying fallback MJPEG %ux%u @ %d FPS",
                     (unsigned)frame->wWidth, (unsigned)frame->wHeight, fps);

            uvc_error_t result = uvc_get_stream_ctrl_format_size(
                devh, ctrl, UVC_FRAME_FORMAT_MJPEG,
                frame->wWidth, frame->wHeight, fps);
            if (result == UVC_SUCCESS) {
                s_frame_width = frame->wWidth;
                s_frame_height = frame->wHeight;
                s_decode_scale_divisor = DECODE_SCALE_DIV;
                return UVC_SUCCESS;
            }
        }
    }

    return UVC_ERROR_INVALID_MODE;
}

static void libuvc_adapter_cb(libuvc_adapter_event_t event)
{
    xEventGroupSetBits(s_app_flags, event);
}

static EventBits_t wait_for_event(EventBits_t event)
{
    return xEventGroupWaitBits(s_app_flags, event, pdTRUE, pdFALSE,
                               portMAX_DELAY) & event;
}

static void usb_lib_handler_task(void *args)
{
    (void)args;
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static void uvc_stats_task(void *args)
{
    (void)args;
    uvc_stream_counters_t previous = {0};

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uvc_stream_counters_t current;
        uvc_get_stream_counters(&current);

        ESP_LOGI(TAG,
                 "uvc/s bulk=%u starts=%u short=%u jpeg=%u fid_drop=%u invalid=%u",
                 (unsigned)(current.bulk_transfers - previous.bulk_transfers),
                 (unsigned)(current.payload_starts - previous.payload_starts),
                 (unsigned)(current.short_packets - previous.short_packets),
                 (unsigned)(current.jpeg_frames - previous.jpeg_frames),
                 (unsigned)(current.fid_incomplete_drops - previous.fid_incomplete_drops),
                 (unsigned)(current.invalid_payload_drops - previous.invalid_payload_drops));
        previous = current;
    }
}

static esp_err_t initialize_usb_host_lib(void)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    return xTaskCreate(usb_lib_handler_task, "usb_events", 4096, NULL, 5, NULL) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

static void camera_task(void *arg)
{
    (void)arg;
    uvc_context_t *ctx = NULL;
    UVC_CHECK(uvc_init(&ctx, NULL));

    while (true) {
        ESP_LOGI(TAG, "waiting for ESP-Claw UVC camera");
        wait_for_event(UVC_DEVICE_CONNECTED);
        vTaskDelay(pdMS_TO_TICKS(2000));

        uvc_device_t *dev = NULL;
        for (int retries = 5; retries > 0 && dev == NULL; --retries) {
            if (uvc_find_device(ctx, &dev, 0, 0, NULL) != UVC_SUCCESS) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        if (dev == NULL) {
            ESP_LOGW(TAG, "UVC camera not found");
            continue;
        }

        uvc_device_handle_t *devh = NULL;
        UVC_CHECK(uvc_open(dev, &devh));

        uvc_stream_ctrl_t ctrl;
        if (negotiate_camera_mode(devh, &ctrl) == UVC_SUCCESS) {
            if (ctrl.dwMaxVideoFrameSize == 0 ||
                ctrl.dwMaxVideoFrameSize > JPEG_FRAME_MAX_BYTES) {
                ctrl.dwMaxVideoFrameSize = JPEG_FRAME_MAX_BYTES;
            }
            ctrl.dwMaxPayloadTransferSize = 512;
            UVC_CHECK(uvc_start_streaming(devh, &ctrl, frame_callback, NULL, 0));
            ESP_LOGI(TAG, "using negotiated MJPEG mode %ux%u @ %.2f FPS",
                     (unsigned)s_frame_width, (unsigned)s_frame_height,
                     ctrl.dwFrameInterval == 0 ? 0.0 :
                     10000000.0 / ctrl.dwFrameInterval);
            wait_for_event(UVC_DEVICE_DISCONNECTED);
            uvc_stop_streaming(devh);
        } else {
            ESP_LOGE(TAG, "camera has no negotiable MJPEG mode up to 640x480");
            wait_for_event(UVC_DEVICE_DISCONNECTED);
        }
        uvc_close(devh);
    }
}

void app_main(void)
{
    s_app_flags = xEventGroupCreate();
    assert(s_app_flags != NULL);
    ESP_ERROR_CHECK(initialize_frame_pipeline());
    ESP_ERROR_CHECK(initialize_usb_host_lib());
    assert(xTaskCreate(uvc_stats_task, "uvc_stats", 3072, NULL, 2, NULL) == pdPASS);

    motors_init();
    ultrasonic_init();
    ESP_LOGI(TAG, "motors and HC-SR04 initialized");
    assert(xTaskCreate(speed_ctrl_task, "speed_ctrl", 4096, NULL, 5, NULL) == pdPASS);
    assert(xTaskCreate(line_ctrl_task, "line_ctrl", 4096, NULL, 4, NULL) == pdPASS);
    assert(xTaskCreate(ultrasonic_avoid_task, "ultrasonic", 3072, NULL, 3, NULL) == pdPASS);

    libuvc_adapter_config_t adapter_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = libuvc_adapter_cb,
    };
    libuvc_adapter_set_config(&adapter_config);
    assert(xTaskCreate(camera_task, "camera", 4096, NULL, 5, NULL) == pdPASS);
}
