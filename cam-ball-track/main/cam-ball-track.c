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
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "jpeg_decoder.h"
#include "cooperative_jpeg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "libuvc/libuvc.h"
#include "libuvc_adapter.h"
#include "usb/usb_host.h"

static const char *TAG = "ball-track";

#define MAX_MODE_CANDIDATES          32
#define PREFERRED_WIDTH              320
#define PREFERRED_HEIGHT             240
#define PREFERRED_FPS                10
#define MAX_SOURCE_WIDTH             640
#define MAX_SOURCE_HEIGHT            480
#define DECODE_SCALE_DIV             8
#define DECODE_BUF_BYTES             ((PREFERRED_WIDTH / 2) * (PREFERRED_HEIGHT / 2) * 2)
#define JPEG_FRAME_MAX_BYTES         (512 * 1024)
#define FRAME_SLOT_COUNT              2
#define RED_MIN_VALUE                80
#define RED_MIN_SATURATION           25
#define RED_HUE_MAX_ANGLE            12
#define RED_HUE_MIN_ANGLE            300
#define BALL_MIN_PIXELS               18
#define GREEN_MIN_VALUE               129
#define GREEN_MIN_SATURATION          19
#define GREEN_HUE_MIN_ANGLE           167
#define GREEN_HUE_MAX_ANGLE           187
#define GREEN_DETECT_RATIO_PERCENT    0.2f

#define CAMERA_FLIPPED               0

#define BALL_STALE_US                450000
#define BALL_TASK_PERIOD_MS          30
#define SEARCH_WZ_RAD_S              0.65f
#define ALIGN_KP                     0.012f
#define ALIGN_MAX_WZ_RAD_S           1.25f
#define ALIGN_DEADBAND_PX            8
#define APPROACH_SPEED_CM_S          50.0f
/* 0.5 m at 50 cm/s = 1.0 s. */
#define APPROACH_TIME_MS              1000
#define LOCK_DIRECTION_STOP_MS        1000
#define STOP_AFTER_APPROACH_MS         300
#define RETURN_SPEED_CM_S            (-10.0f)
#define RETURN_TIME_MS               4000

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

typedef enum {
    BALL_RED,
    BALL_GREEN,
    BALL_COLOR_COUNT,
} ball_color_t;

typedef struct {
    int32_t cx;
    int32_t cy;
    uint32_t pixels;
    int64_t last_seen_us;
} ball_state_t;

static portMUX_TYPE s_ball_mux = portMUX_INITIALIZER_UNLOCKED;
static ball_state_t s_balls[BALL_COLOR_COUNT];
/* The camera only publishes the colour currently being handled by the task. */
static ball_color_t s_detection_target = BALL_RED;

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

typedef enum {
    TASK_SEARCH,
    TASK_ALIGN_DIRECTION,
    TASK_LOCK_DIRECTION,
    TASK_APPROACH_BALL,
    TASK_STOP_AFTER_APPROACH,
    TASK_RETURN_TO_WAIT,
    TASK_COMPLETE,
} task_phase_t;

static const char *ball_name(ball_color_t color)
{
    return color == BALL_RED ? "red" : "green";
}

static bool get_fresh_ball(ball_color_t color, ball_state_t *ball, int64_t now_us)
{
    portENTER_CRITICAL(&s_ball_mux);
    *ball = s_balls[color];
    portEXIT_CRITICAL(&s_ball_mux);
    return ball->last_seen_us != 0 && now_us - ball->last_seen_us <= BALL_STALE_US;
}

static float clampf(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

/* For each colour, drive for two seconds from its first camera sighting. */
static void ball_task_ctrl(void *arg)
{
    (void)arg;
    task_phase_t phase = TASK_SEARCH;
    ball_color_t wanted = BALL_RED;
    int64_t phase_started_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BALL_TASK_PERIOD_MS));
        const int64_t now_us = esp_timer_get_time();
        ball_state_t ball;
        const bool visible = get_fresh_ball(wanted, &ball, now_us);
        const float image_center_x = s_frame_width / 2.0f;

        if (phase == TASK_COMPLETE) {
            set_speed(0.0f, 0.0f, 0.0f);
            continue;
        }

        if (!visible && phase != TASK_SEARCH && phase != TASK_LOCK_DIRECTION &&
            phase != TASK_APPROACH_BALL &&
            phase != TASK_STOP_AFTER_APPROACH &&
            phase != TASK_RETURN_TO_WAIT) {
            ESP_LOGW(TAG, "%s ball lost; returning to search", ball_name(wanted));
            phase = TASK_SEARCH;
            phase_started_us = now_us;
        }

        switch (phase) {
        case TASK_SEARCH:
            if (!visible) {
                /* Red is searched by turning left (CCW+); green by turning right. */
                const float search_wz = wanted == BALL_RED ? SEARCH_WZ_RAD_S
                                                            : -SEARCH_WZ_RAD_S;
                set_speed(0.0f, 0.0f, search_wz);
                break;
            }
            ESP_LOGI(TAG, "%s ball found at (%ld,%ld); aligning direction", ball_name(wanted),
                     (long)ball.cx, (long)ball.cy);
            set_speed(0.0f, 0.0f, 0.0f);
            phase = TASK_ALIGN_DIRECTION;
            phase_started_us = now_us;
            break;

        case TASK_ALIGN_DIRECTION: {
            const float error_x = ball.cx - image_center_x;
            const float wz = clampf(ALIGN_KP * error_x,
                                    -ALIGN_MAX_WZ_RAD_S, ALIGN_MAX_WZ_RAD_S);
            if (fabsf(error_x) <= ALIGN_DEADBAND_PX) {
                set_speed(0.0f, 0.0f, 0.0f);
                ESP_LOGI(TAG, "%s direction aligned; waiting one second", ball_name(wanted));
                phase = TASK_LOCK_DIRECTION;
                phase_started_us = now_us;
            } else {
                set_speed(0.0f, 0.0f, wz);
            }
            break;
        }

        case TASK_LOCK_DIRECTION:
            /* Keep still for one second after the initial ball direction is captured. */
            set_speed(0.0f, 0.0f, 0.0f);
            if (now_us - phase_started_us >= (int64_t)LOCK_DIRECTION_STOP_MS * 1000) {
                ESP_LOGI(TAG, "%s direction locked; starting five-second approach",
                         ball_name(wanted));
                phase = TASK_APPROACH_BALL;
                phase_started_us = now_us;
            }
            break;

        case TASK_APPROACH_BALL: {
            /* Direction is already aligned, so travel straight forward. */
            set_speed(APPROACH_SPEED_CM_S, 0.0f, 0.0f);
            if (now_us - phase_started_us >= (int64_t)APPROACH_TIME_MS * 1000) {
                set_speed(0.0f, 0.0f, 0.0f);
                ESP_LOGI(TAG, "%s five-second approach complete; stopping", ball_name(wanted));
                phase = TASK_STOP_AFTER_APPROACH;
                phase_started_us = now_us;
            }
            break;
        }

        case TASK_STOP_AFTER_APPROACH:
            set_speed(0.0f, 0.0f, 0.0f);
            if (now_us - phase_started_us >= (int64_t)STOP_AFTER_APPROACH_MS * 1000) {
                phase = TASK_RETURN_TO_WAIT;
                phase_started_us = now_us;
            }
            break;

        case TASK_RETURN_TO_WAIT:
            set_speed(RETURN_SPEED_CM_S, 0.0f, 0.0f);
            if (now_us - phase_started_us >= (int64_t)RETURN_TIME_MS * 1000) {
                set_speed(0.0f, 0.0f, 0.0f);
                if (wanted == BALL_RED) {
                    wanted = BALL_GREEN;
                    portENTER_CRITICAL(&s_ball_mux);
                    s_detection_target = BALL_GREEN;
                    portEXIT_CRITICAL(&s_ball_mux);
                    phase = TASK_SEARCH;
                    phase_started_us = now_us;
                    ESP_LOGI(TAG, "return complete; searching for green ball");
                } else {
                    phase = TASK_COMPLETE;
                    ESP_LOGI(TAG, "green ball complete; task finished and motors stopped");
                }
            }
            break;

        case TASK_COMPLETE:
            break;
        }
    }
}

static bool is_mjpeg_format(const uvc_format_desc_t *format)
{
    return format->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG;
}

static bool is_ball_rgb(ball_color_t color, int r, int g, int b,
                        int mx, int mn, int delta)
{
    const int min_value = color == BALL_GREEN ? GREEN_MIN_VALUE : RED_MIN_VALUE;
    const int min_saturation = color == BALL_GREEN ? GREEN_MIN_SATURATION
                                                    : RED_MIN_SATURATION;
    if (mx < min_value || delta < min_saturation) {
        return false;
    }

    int h6;
    if (mx == r) {
        h6 = (g - b) * 60 / delta;
    } else if (mx == g) {
        h6 = (b - r) * 60 / delta + 120;
    } else {
        h6 = (r - g) * 60 / delta + 240;
    }
    if (h6 < 0) {
        h6 += 360;
    }

    if (color == BALL_RED) {
        return h6 <= RED_HUE_MAX_ANGLE || h6 >= RED_HUE_MIN_ANGLE;
    }
    return h6 >= GREEN_HUE_MIN_ANGLE && h6 <= GREEN_HUE_MAX_ANGLE;
}

static void detect_balls_mjpeg(const uint8_t *jpeg, size_t jpeg_len)
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
    uint32_t count[BALL_COLOR_COUNT] = {0};
    uint64_t sum_x[BALL_COLOR_COUNT] = {0};
    uint64_t sum_y[BALL_COLOR_COUNT] = {0};
    ball_color_t detection_target;
    portENTER_CRITICAL(&s_ball_mux);
    detection_target = s_detection_target;
    portEXIT_CRITICAL(&s_ball_mux);

    for (uint32_t y = 0; y < image.height; ++y) {
        for (uint32_t x = 0; x < image.width; ++x) {
            const uint16_t p = decoded[(size_t)y * image.width + x];
            const int r = ((p >> 11) & 0x1f) * 255 / 31;
            const int g = ((p >> 5) & 0x3f) * 255 / 63;
            const int b = (p & 0x1f) * 255 / 31;

            int mx = r > g ? r : g;
            mx = mx > b ? mx : b;
            int mn = r < g ? r : g;
            mn = mn < b ? mn : b;
            const int delta = mx - mn;

            for (ball_color_t color = BALL_RED; color < BALL_COLOR_COUNT; ++color) {
                if (color == detection_target &&
                    is_ball_rgb(color, r, g, b, mx, mn, delta)) {
                    ++count[color];
                    sum_x[color] += x;
                    sum_y[color] += y;
                }
            }

        }
    }

    for (ball_color_t color = BALL_RED; color < BALL_COLOR_COUNT; ++color) {
        if (color != detection_target) {
            continue;
        }
        const uint32_t min_pixels = color == BALL_GREEN
            ? (uint32_t)ceilf((float)image.width * image.height *
                              GREEN_DETECT_RATIO_PERCENT / 100.0f)
            : BALL_MIN_PIXELS;
        if (count[color] < min_pixels) {
            continue;
        }
        int32_t cx = (int32_t)(sum_x[color] * s_decode_scale_divisor / count[color]);
        int32_t cy = (int32_t)(sum_y[color] * s_decode_scale_divisor / count[color]);
        if (CAMERA_FLIPPED) {
            cx = (int32_t)s_frame_width - 1 - cx;
            cy = (int32_t)s_frame_height - 1 - cy;
        }
        portENTER_CRITICAL(&s_ball_mux);
        s_balls[color].cx = cx;
        s_balls[color].cy = cy;
        s_balls[color].pixels = count[color];
        s_balls[color].last_seen_us = esp_timer_get_time();
        portEXIT_CRITICAL(&s_ball_mux);
        if (log_now) {
            ESP_LOGI(TAG, "%s detected: pixels=%u center=(%ld,%ld)",
                     ball_name(color), (unsigned)count[color], (long)cx, (long)cy);
        }
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
            detect_balls_mjpeg(slot->data, slot->len);
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

        if (uxQueueMessagesWaiting(s_ready_frames) > 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
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
    ESP_LOGI(TAG, "motors initialized");
    assert(xTaskCreate(speed_ctrl_task, "speed_ctrl", 4096, NULL, 5, NULL) == pdPASS);
    assert(xTaskCreate(ball_task_ctrl, "ball_task", 4096, NULL, 4, NULL) == pdPASS);

    libuvc_adapter_config_t adapter_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = libuvc_adapter_cb,
    };
    libuvc_adapter_set_config(&adapter_config);
    assert(xTaskCreate(camera_task, "camera", 4096, NULL, 5, NULL) == pdPASS);
}
