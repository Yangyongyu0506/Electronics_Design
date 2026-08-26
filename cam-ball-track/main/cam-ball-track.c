#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "libuvc/libuvc.h"
#include "libuvc_helper.h"
#include "libuvc_adapter.h"
#include "usb/usb_host.h"

static const char *TAG = "ball-track";

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
#define PID_KI            0.1f
#define PID_KD            0.3f
#define PID_INTEGRAL_LIMIT 400.0f
#define SPEED_FILTER_ALPHA 0.3f

#define MOTOR_A_PWM_GPIO   GPIO_NUM_1
#define MOTOR_A_IN1_GPIO   GPIO_NUM_42
#define MOTOR_A_IN2_GPIO   GPIO_NUM_2
#define MOTOR_A_ENC_A_GPIO GPIO_NUM_41
#define MOTOR_A_ENC_B_GPIO GPIO_NUM_40

#define MOTOR_B_PWM_GPIO   GPIO_NUM_4
#define MOTOR_B_IN1_GPIO   GPIO_NUM_5
#define MOTOR_B_IN2_GPIO   GPIO_NUM_6
#define MOTOR_B_ENC_A_GPIO GPIO_NUM_8
#define MOTOR_B_ENC_B_GPIO GPIO_NUM_3

#define MOTOR_D_PWM_GPIO   GPIO_NUM_7
#define MOTOR_D_IN1_GPIO   GPIO_NUM_16
#define MOTOR_D_IN2_GPIO   GPIO_NUM_15
#define MOTOR_D_ENC_A_GPIO GPIO_NUM_17
#define MOTOR_D_ENC_B_GPIO GPIO_NUM_18

#define FRAME_MAX          (64 * 1024)
#define LIBUVC_BUF_CAP     (16 * 1024)
#define NEGOTIATION_ATTEMPTS 3

#define MAX_BLOCKS_X 64
#define MAX_BLOCKS_Y 64

#define BALL_KP          0.003f
#define BALL_KD          0.0015f
#define BALL_MAX_WZ      1.5f
#define BALL_MIN_BLOCKS  2
#define BALL_MIN_SIGNAL  4
#define BALL_RED_FRAC    0.6f
#define BALL_DEADBAND_PX 8

#define CAMERA_FLIPPED   0

#define UVC_CHECK(exp) do {                 \
    uvc_error_t _err_ = (exp);              \
    if (_err_ < 0) {                        \
        ESP_LOGE(TAG, "UVC error: %s",      \
                 uvc_error_string(_err_));  \
        assert(0);                          \
    }                                       \
} while (0)

typedef struct {
    enum uvc_frame_format format;
    int width;
    int height;
    int fps;
} stream_profile_t;

static const stream_profile_t stream_profiles[] = {
    {UVC_FRAME_FORMAT_MJPEG, 640, 480, 10},
    {UVC_FRAME_FORMAT_MJPEG, 640, 480, 0},
    {UVC_FRAME_FORMAT_MJPEG, 480, 320, 10},
    {UVC_FRAME_FORMAT_MJPEG, 480, 320, 0},
};

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
    {MOTOR_A_PWM_GPIO, MOTOR_A_IN1_GPIO, MOTOR_A_IN2_GPIO,
     MOTOR_A_ENC_A_GPIO, MOTOR_A_ENC_B_GPIO, LEDC_CHANNEL_0, NULL,
     0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
    {MOTOR_B_PWM_GPIO, MOTOR_B_IN1_GPIO, MOTOR_B_IN2_GPIO,
     MOTOR_B_ENC_A_GPIO, MOTOR_B_ENC_B_GPIO, LEDC_CHANNEL_1, NULL,
     0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
    {MOTOR_D_PWM_GPIO, MOTOR_D_IN1_GPIO, MOTOR_D_IN2_GPIO,
     MOTOR_D_ENC_A_GPIO, MOTOR_D_ENC_B_GPIO, LEDC_CHANNEL_2, NULL,
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
    float va = 0.86602540378f * vx + 0.5f * vy + WHEEL_BASE_CM * wz;
    float vd = -0.86602540378f * vx + 0.5f * vy + WHEEL_BASE_CM * wz;
    float vb = -vy + WHEEL_BASE_CM * wz;

    motor_set_target_cm_s(0, va);
    motor_set_target_cm_s(1, vb);
    motor_set_target_cm_s(2, vd);
}

typedef struct {
    const uint8_t *data;
    size_t len;
    size_t pos;
    uint32_t bitbuf;
    int bitcnt;
    int restart_pending;
    int eof;
} bitreader_t;

typedef struct {
    uint8_t huffval[256];
    int valcount;
    int mincode[17];
    int maxcode[17];
    int valptr[17];
} huff_table_t;

static huff_table_t s_dc_tables[4];
static huff_table_t s_ac_tables[4];

static int s_comp_h[4];
static int s_comp_v[4];
static int s_comp_count;

static int s_scan_comp_id[4];
static int s_scan_comp_dc[4];
static int s_scan_comp_ac[4];
static int s_scan_count;

static int s_width;
static int s_height;

static int16_t s_block_cb[MAX_BLOCKS_X * MAX_BLOCKS_Y];
static int16_t s_block_cr[MAX_BLOCKS_X * MAX_BLOCKS_Y];
static int s_blocks_x;
static int s_blocks_y;

static void huff_build(huff_table_t *t, const uint8_t *bits, const uint8_t *vals, int nvals)
{
    int code = 0;
    int k = 0;
    for (int l = 1; l <= 16; l++) {
        t->valptr[l] = k;
        t->mincode[l] = code;
        code += bits[l - 1];
        t->maxcode[l] = code - 1;
        k += bits[l - 1];
        code <<= 1;
    }
    t->valcount = nvals;
    for (int i = 0; i < nvals && i < 256; i++) {
        t->huffval[i] = vals[i];
    }
}

static int br_fill(bitreader_t *br)
{
    if (br->pos >= br->len) {
        return -1;
    }
    uint8_t b = br->data[br->pos++];
    if (b == 0xFF) {
        while (br->pos < br->len && br->data[br->pos] == 0xFF) {
            br->pos++;
        }
        if (br->pos >= br->len) {
            return -1;
        }
        uint8_t m = br->data[br->pos];
        if (m == 0x00) {
            br->pos++;
        } else if (m >= 0xD0 && m <= 0xD7) {
            br->pos++;
            br->restart_pending = 1;
            return -2;
        } else {
            return -1;
        }
    }
    br->bitbuf = b;
    return 0;
}

static int br_read_bit(bitreader_t *br)
{
    if (br->bitcnt == 0) {
        if (br_fill(br) != 0) {
            br->eof = 1;
            return 0;
        }
        br->bitcnt = 8;
    }
    br->bitcnt--;
    return (int)((br->bitbuf >> br->bitcnt) & 1);
}

static void br_skip_bits(bitreader_t *br, int n)
{
    for (int i = 0; i < n; i++) {
        br_read_bit(br);
    }
}

static int receive_extend(bitreader_t *br, int s)
{
    int v = 0;
    for (int i = 0; i < s; i++) {
        v = (v << 1) | br_read_bit(br);
    }
    if (s > 0 && v < (1 << (s - 1))) {
        v -= (1 << s) - 1;
    }
    return v;
}

static int huff_decode(bitreader_t *br, huff_table_t *t)
{
    int code = 0;
    for (int l = 1; l <= 16; l++) {
        code = (code << 1) | br_read_bit(br);
        if (br->eof) {
            return -1;
        }
        if (code <= t->maxcode[l]) {
            int idx = t->valptr[l] + code - t->mincode[l];
            if (idx >= 0 && idx < t->valcount) {
                return t->huffval[idx];
            }
            return -1;
        }
    }
    return -1;
}

static int scan_decode(bitreader_t *br)
{
    int dc_prev[4] = {0, 0, 0, 0};
    int mcu_count = 0;
    int total = s_blocks_x * s_blocks_y;

    while (mcu_count < total) {
        if (br->restart_pending) {
            br->restart_pending = 0;
            br->bitcnt = 0;
            for (int i = 0; i < 4; i++) {
                dc_prev[i] = 0;
            }
        }

        int16_t cb = 0;
        int16_t cr = 0;
        for (int ci = 0; ci < s_scan_count; ci++) {
            int comp = s_scan_comp_id[ci];
            int blocks = s_comp_h[comp] * s_comp_v[comp];
            for (int b = 0; b < blocks; b++) {
                int s = huff_decode(br, &s_dc_tables[s_scan_comp_dc[ci]]);
                if (s < 0) {
                    return mcu_count;
                }
                int diff = receive_extend(br, s);
                int dc = dc_prev[ci] + diff;
                dc_prev[ci] = dc;

                if (b == 0) {
                    if (comp == 2) {
                        cb = (int16_t)dc;
                    } else if (comp == 3) {
                        cr = (int16_t)dc;
                    }
                }

                for (;;) {
                    int rs = huff_decode(br, &s_ac_tables[s_scan_comp_ac[ci]]);
                    if (rs < 0) {
                        return mcu_count;
                    }
                    int r = rs >> 4;
                    int s2 = rs & 15;
                    if (s2 == 0) {
                        if (r == 15) {
                            continue;
                        }
                        break;
                    }
                    br_skip_bits(br, s2);
                }
            }
        }

        s_block_cb[mcu_count] = cb;
        s_block_cr[mcu_count] = cr;
        mcu_count++;
    }

    return mcu_count;
}

static bool jpeg_dc_decode(const uint8_t *data, size_t len)
{
    size_t pos = 0;
    bool sof_seen = false;
    bool sos_seen = false;
    static char marker_trace[128];
    int trace_len = 0;
    static bool trace_logged = false;
    static const uint8_t *entropy_ptr;
    static size_t entropy_len;
    static int entropy_scan_count;
    static int entropy_comp[3];
    static int entropy_dc[3];
    static int entropy_ac[3];

    if (len < 4 || data[0] != 0xFF || data[1] != 0xD8) {
        return false;
    }
    pos = 2;

    while (pos + 4 <= len) {
        if (data[pos] != 0xFF) {
            pos++;
            continue;
        }
        while (pos < len && data[pos] == 0xFF) {
            pos++;
        }
        if (pos >= len) {
            break;
        }
        uint8_t marker = data[pos++];
        if (marker == 0xD9 || marker == 0xD8) {
            continue;
        }
        if (marker >= 0xD0 && marker <= 0xD7) {
            continue;
        }
        if (marker == 0x01) {
            continue;
        }
        if (pos + 2 > len) {
            return false;
        }
        uint16_t seg_len = (uint16_t)((data[pos] << 8) | data[pos + 1]);
        if (seg_len < 2 || pos + seg_len > len) {
            return false;
        }
        const uint8_t *seg = data + pos + 2;
        size_t seg_data = seg_len - 2;

        if (trace_len < (int)sizeof(marker_trace) - 16) {
            trace_len += snprintf(marker_trace + trace_len, sizeof(marker_trace) - trace_len,
                                  "%02x(%u) ", marker, (unsigned)seg_len);
        }

        if (marker == 0xC0 || marker == 0xC1) {
            if (seg_data < 6) {
                return false;
            }
            s_height = (seg[1] << 8) | seg[2];
            s_width = (seg[3] << 8) | seg[4];
            s_comp_count = seg[5];
            if (s_comp_count < 1 || s_comp_count > 3) {
                return false;
            }
            int hmax = 1;
            int vmax = 1;
            for (int i = 0; i < s_comp_count; i++) {
                int cid = seg[6 + i * 3];
                int h = seg[7 + i * 3] >> 4;
                int v = seg[7 + i * 3] & 0x0F;
                if (cid >= 1 && cid <= 3) {
                    s_comp_h[cid] = h;
                    s_comp_v[cid] = v;
                }
                if (h > hmax) {
                    hmax = h;
                }
                if (v > vmax) {
                    vmax = v;
                }
            }
            s_blocks_x = (s_width + 8 * hmax - 1) / (8 * hmax);
            s_blocks_y = (s_height + 8 * vmax - 1) / (8 * vmax);
            if (s_blocks_x > MAX_BLOCKS_X || s_blocks_y > MAX_BLOCKS_Y) {
                return false;
            }
            sof_seen = true;
        } else if (marker == 0xC4) {
            size_t p = 0;
            while (p < seg_data) {
                uint8_t tc = seg[p] >> 4;
                uint8_t th = seg[p] & 0x0F;
                p++;
                if (p + 16 > seg_data) {
                    return false;
                }
                const uint8_t *bits = seg + p;
                p += 16;
                int nvals = 0;
                for (int i = 0; i < 16; i++) {
                    nvals += bits[i];
                }
                if (p + nvals > seg_data) {
                    return false;
                }
                if (tc == 0 && th < 4) {
                    huff_build(&s_dc_tables[th], bits, seg + p, nvals);
                } else if (tc == 1 && th < 4) {
                    huff_build(&s_ac_tables[th], bits, seg + p, nvals);
                }
                p += nvals;
            }
        } else if (marker == 0xDA) {
            s_scan_count = seg[0];
            if (s_scan_count < 1 || s_scan_count > 3) {
                return false;
            }
            for (int i = 0; i < s_scan_count; i++) {
                s_scan_comp_id[i] = seg[1 + i * 2];
                s_scan_comp_dc[i] = seg[2 + i * 2] >> 4;
                s_scan_comp_ac[i] = seg[2 + i * 2] & 0x0F;
            }
            pos += seg_len;
            entropy_ptr = data + pos;
            entropy_len = len - pos;
            entropy_scan_count = s_scan_count;
            for (int i = 0; i < s_scan_count; i++) {
                entropy_comp[i] = s_scan_comp_id[i];
                entropy_dc[i] = s_scan_comp_dc[i];
                entropy_ac[i] = s_scan_comp_ac[i];
            }
            bitreader_t br = {
                .data = data + pos,
                .len = len - pos,
                .pos = 0,
                .bitbuf = 0,
                .bitcnt = 0,
                .restart_pending = 0,
                .eof = 0,
            };
            int decoded = scan_decode(&br);
            if (decoded <= 0) {
                if (!trace_logged) {
                    trace_logged = true;
                    ESP_LOGI(TAG, "markers: %s", marker_trace);
                    char hex[128];
                    int o = 0;
                    size_t n = entropy_len < 40 ? entropy_len : 40;
                    for (size_t i = 0; i < n && o < (int)sizeof(hex) - 8; i++) {
                        o += snprintf(hex + o, sizeof(hex) - o, "%02x ", entropy_ptr[i]);
                    }
                    ESP_LOGI(TAG, "entropy: %s", hex);
                    for (int i = 0; i < entropy_scan_count; i++) {
                        ESP_LOGI(TAG, "scan comp %d: dc=%d ac=%d h=%d v=%d",
                                 entropy_comp[i], entropy_dc[i], entropy_ac[i],
                                 s_comp_h[entropy_comp[i]], s_comp_v[entropy_comp[i]]);
                    }
                    ESP_LOGI(TAG, "dims: %dx%d comps=%d", s_width, s_height, s_comp_count);
                }
                return false;
            }
            sos_seen = true;
            break;
        }

        pos += seg_len;
    }

    if (!trace_logged) {
        trace_logged = true;
        ESP_LOGI(TAG, "jpeg markers: %s", marker_trace);
    }

    return sof_seen && sos_seen;
}

static float s_last_error;
static bool s_ball_seen;

static void track_ball(const uint8_t *jpeg, size_t len)
{
    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();

    if (!jpeg_dc_decode(jpeg, len)) {
        if (now - last_log > 2000000) {
            last_log = now;
            ESP_LOGW(TAG, "jpeg decode failed, len=%u", (unsigned)len);
        }
        return;
    }

    int total = s_blocks_x * s_blocks_y;
    int max_diff = 0;
    for (int i = 0; i < total; i++) {
        int diff = (int)s_block_cr[i] - (int)s_block_cb[i];
        if (diff > max_diff) {
            max_diff = diff;
        }
    }

    int threshold = BALL_MIN_SIGNAL;
    if (max_diff > threshold) {
        int frac_thr = (int)((float)max_diff * BALL_RED_FRAC);
        if (frac_thr > threshold) {
            threshold = frac_thr;
        }
    }

    int red_count = 0;
    int sum_x = 0;
    int sum_y = 0;
    for (int i = 0; i < total; i++) {
        int diff = (int)s_block_cr[i] - (int)s_block_cb[i];
        if (diff >= threshold && s_block_cr[i] > 0) {
            int bx = i % s_blocks_x;
            int by = i / s_blocks_x;
            sum_x += bx;
            sum_y += by;
            red_count++;
        }
    }

    if (now - last_log > 1000000) {
        last_log = now;
        ESP_LOGI(TAG, "dbg: %dx%d blocks=%dx%d maxdiff=%d thr=%d red=%d",
                 s_width, s_height, s_blocks_x, s_blocks_y, max_diff, threshold, red_count);
    }

    if (red_count >= BALL_MIN_BLOCKS) {
        int cx_block = sum_x / red_count;
        int cy_block = sum_y / red_count;
        int px = cx_block * s_width / s_blocks_x + s_width / s_blocks_x / 2;
        int py = cy_block * s_height / s_blocks_y + s_height / s_blocks_y / 2;

        float error = (float)(px - s_width / 2);
        float de = error - s_last_error;
        s_last_error = error;

        float wz = BALL_KP * error + BALL_KD * de;
        if (CAMERA_FLIPPED) {
            wz = -wz;
        }
        if (error > -BALL_DEADBAND_PX && error < BALL_DEADBAND_PX) {
            wz = 0;
        }
        if (wz > BALL_MAX_WZ) {
            wz = BALL_MAX_WZ;
        } else if (wz < -BALL_MAX_WZ) {
            wz = -BALL_MAX_WZ;
        }

        set_speed(0.0f, 0.0f, wz);
        s_ball_seen = true;

        ESP_LOGI(TAG, "ball: blocks=%d cx=%d cy=%d err=%d wz=%.2f",
                 red_count, px, py, (int)error, (double)wz);
    } else {
        if (s_ball_seen) {
            ESP_LOGI(TAG, "ball lost");
        }
        s_ball_seen = false;
        s_last_error = 0;
        set_speed(0.0f, 0.0f, 0.0f);
    }
}

static EventGroupHandle_t s_app_flags;

static uint8_t s_jpeg_buf[FRAME_MAX];
static size_t s_jpeg_len;
static bool s_jpeg_in_frame;
static uint8_t s_jpeg_last_byte;

static void frame_callback(uvc_frame_t *frame, void *ptr)
{
    static size_t fps;
    static int64_t start_time;
    int64_t current_time = esp_timer_get_time();

    fps++;
    if (!start_time) {
        start_time = current_time;
    }
    if (current_time > start_time + 1000000) {
        ESP_LOGI(TAG, "camera fps: %u", fps);
        start_time = current_time;
        fps = 0;
    }

    const uint8_t *src = (const uint8_t *)frame->data;
    size_t remain = frame->data_bytes;
    bool chunk_boundary = true;

    while (remain > 0) {
        if (!s_jpeg_in_frame) {
            size_t skip = 0;
            bool found = false;
            if (chunk_boundary && s_jpeg_last_byte == 0xFF && src[0] == 0xD8) {
                found = true;
                s_jpeg_buf[0] = 0xFF;
                s_jpeg_len = 1;
            } else {
                for (size_t i = 0; i + 1 < remain; i++) {
                    if (src[i] == 0xFF && src[i + 1] == 0xD8) {
                        skip = i;
                        found = true;
                        break;
                    }
                }
            }
            if (!found) {
                break;
            }
            src += skip;
            remain -= skip;
            s_jpeg_in_frame = true;
        }

        size_t eoi = 0;
        bool eoi_found = false;
        if (chunk_boundary && s_jpeg_last_byte == 0xFF && src[0] == 0xD9) {
            eoi = 1;
            eoi_found = true;
        } else {
            for (size_t i = 0; i + 1 < remain; i++) {
                if (src[i] == 0xFF && src[i + 1] == 0xD9) {
                    eoi = i + 2;
                    eoi_found = true;
                    break;
                }
            }
        }
        chunk_boundary = false;

        size_t copy = remain;
        if (eoi_found) {
            copy = eoi;
        }
        if (s_jpeg_len + copy > FRAME_MAX) {
            copy = FRAME_MAX - s_jpeg_len;
            eoi_found = false;
        }

        if (copy > 0) {
            memcpy(s_jpeg_buf + s_jpeg_len, src, copy);
            s_jpeg_len += copy;
            src += copy;
            remain -= copy;
        }

        if (eoi_found) {
            track_ball(s_jpeg_buf, s_jpeg_len);
            s_jpeg_in_frame = false;
            s_jpeg_len = 0;
        } else if (s_jpeg_len >= FRAME_MAX) {
            s_jpeg_in_frame = false;
            s_jpeg_len = 0;
        }
    }

    if (frame->data_bytes > 0) {
        const uint8_t *fdata = (const uint8_t *)frame->data;
        s_jpeg_last_byte = fdata[frame->data_bytes - 1];
    }
}

static void libuvc_adapter_cb(libuvc_adapter_event_t event)
{
    xEventGroupSetBits(s_app_flags, event);
}

static EventBits_t wait_for_event(EventBits_t event)
{
    return xEventGroupWaitBits(s_app_flags, event, pdTRUE, pdFALSE, portMAX_DELAY) & event;
}

static void usb_lib_handler_task(void *args)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static esp_err_t initialize_usb_host_lib(void)
{
    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        return err;
    }

    TaskHandle_t task_handle = NULL;
    if (xTaskCreate(usb_lib_handler_task, "usb_events", 4096, NULL, 2, &task_handle) != pdPASS) {
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static uvc_error_t negotiate_stream_profile(uvc_device_handle_t *devh, uvc_stream_ctrl_t *ctrl)
{
    uvc_error_t res = UVC_ERROR_NO_DEVICE;

    for (size_t i = 0; i < sizeof(stream_profiles) / sizeof(stream_profiles[0]); i++) {
        int attempt = NEGOTIATION_ATTEMPTS;
        do {
            ESP_LOGI(TAG, "negotiate %dx%d, %d fps ...",
                     stream_profiles[i].width, stream_profiles[i].height, stream_profiles[i].fps);
            res = uvc_get_stream_ctrl_format_size(devh,
                                                  ctrl,
                                                  stream_profiles[i].format,
                                                  stream_profiles[i].width,
                                                  stream_profiles[i].height,
                                                  stream_profiles[i].fps);
        } while (--attempt && !(UVC_SUCCESS == res));
        if (UVC_SUCCESS == res) {
            ESP_LOGI(TAG, "negotiation complete: profile %d, format idx %d, frame idx %d",
                     i, ctrl->bFormatIndex, ctrl->bFrameIndex);
            break;
        }
    }

    return res;
}

static void camera_task(void *arg)
{
    uvc_context_t *ctx = NULL;
    uvc_device_handle_t *devh = NULL;
    uvc_stream_ctrl_t ctrl;

    UVC_CHECK(uvc_init(&ctx, NULL));

    while (1) {
        ESP_LOGI(TAG, "waiting for USB UVC device ...");
        wait_for_event(UVC_DEVICE_CONNECTED);

        vTaskDelay(pdMS_TO_TICKS(2000));

        uvc_device_t *dev = NULL;
        int retries = 5;
        while (retries-- && uvc_find_device(ctx, &dev, 0, 0, NULL) != UVC_SUCCESS) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (dev == NULL) {
            ESP_LOGW(TAG, "UVC device not found");
            continue;
        }
        ESP_LOGI(TAG, "UVC device found");

        UVC_CHECK(uvc_open(dev, &devh));

        if (UVC_SUCCESS == negotiate_stream_profile(devh, &ctrl)) {
            ctrl.dwMaxPayloadTransferSize = 512;
            if (ctrl.dwMaxVideoFrameSize > LIBUVC_BUF_CAP) {
                ctrl.dwMaxVideoFrameSize = LIBUVC_BUF_CAP;
            }

            UVC_CHECK(uvc_start_streaming(devh, &ctrl, frame_callback, NULL, 0));
            ESP_LOGI(TAG, "streaming started");

            wait_for_event(UVC_DEVICE_DISCONNECTED);

            uvc_stop_streaming(devh);
            ESP_LOGI(TAG, "streaming stopped");
        } else {
            ESP_LOGE(TAG, "no supported stream profile found");
            wait_for_event(UVC_DEVICE_DISCONNECTED);
        }

        uvc_close(devh);
        devh = NULL;
    }
}

void app_main(void)
{
    motors_init();
    ESP_LOGI(TAG, "motors initialized");

    s_app_flags = xEventGroupCreate();
    assert(s_app_flags);

    xTaskCreate(speed_ctrl_task, "speed_ctrl", 4096, NULL, 5, NULL);

    ESP_ERROR_CHECK(initialize_usb_host_lib());

    libuvc_adapter_config_t config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = libuvc_adapter_cb,
    };
    libuvc_adapter_set_config(&config);

    xTaskCreate(camera_task, "camera", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
