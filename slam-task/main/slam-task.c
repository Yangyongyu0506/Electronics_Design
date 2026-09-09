#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

static const char *TAG = "slam";

#define PC_IP_ADDR          "192.168.0.198"
#define PC_PORT             34567
#define NET_PERIOD_MS       20

#define WIFI_SSID           "iarc-thu5G"
#define WIFI_PASSWORD       "12345678"

#define ODOM_PERIOD_MS      10
#define LOG_EVERY_N         20

#define LIDAR_UART          UART_NUM_1
#define LIDAR_TX_GPIO       GPIO_NUM_10
#define LIDAR_RX_GPIO       GPIO_NUM_9
#define LIDAR_BAUD          230400
#define LIDAR_POINTS_PER_PKT 16
#define LIDAR_MAX_SCAN_POINTS 800

#define ENC_HIGH_LIMIT      10000
#define ENC_LOW_LIMIT       -10000
#define COUNTS_PER_REV      512
#define WHEEL_DIAMETER_CM   5.5f
#define WHEEL_BASE_CM       9.0f
#define COUNTS_PER_CM       ((float)COUNTS_PER_REV / (3.14159265f * WHEEL_DIAMETER_CM))
#define WHEEL_A_SIGN        1.0f
#define WHEEL_B_SIGN        1.0f
#define WHEEL_D_SIGN        1.0f
#define ODOM_SCALE          0.01f

#define SPEED_CTRL_PERIOD_MS 20
#define MAX_DUTY            1023
#define PID_KP              2.0f
#define PID_KI              0.5f
#define PID_KD              0.0f
#define PID_INTEGRAL_LIMIT  400.0f
#define SPEED_FILTER_ALPHA  0.3f
#define CMD_MAGIC           0x0D0D0003
#define CMD_TIMEOUT_MS      500

#define RAD_TO_DEG          (180.0f / 3.14159265f)

#define MOTOR_COUNT         3
#define PWM_FREQ_HZ         20000

static const int s_motor_pwm[MOTOR_COUNT] = {GPIO_NUM_1, GPIO_NUM_4, GPIO_NUM_7};
static const int s_motor_in1[MOTOR_COUNT] = {GPIO_NUM_42, GPIO_NUM_5, GPIO_NUM_16};
static const int s_motor_in2[MOTOR_COUNT] = {GPIO_NUM_2, GPIO_NUM_6, GPIO_NUM_15};
static const ledc_channel_t s_motor_chan[MOTOR_COUNT] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

static pcnt_unit_handle_t s_enc_units[MOTOR_COUNT];
static const int s_enc_pins_a[MOTOR_COUNT] = {GPIO_NUM_41, GPIO_NUM_8, GPIO_NUM_17};
static const int s_enc_pins_b[MOTOR_COUNT] = {GPIO_NUM_40, GPIO_NUM_3, GPIO_NUM_18};

static float s_px, s_py;
static float s_yaw;
static float s_vx, s_vy;
static float s_wz_wheel;
static volatile uint64_t s_last_cmd_us;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_measured;
} pid_ctrl_t;

typedef struct {
    int target_speed;
    int last_count;
    float measured_speed;
    pid_ctrl_t pid;
    bool saturated;
} motor_ctrl_t;

static motor_ctrl_t motors[MOTOR_COUNT] = {
    {0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
    {0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
    {0, 0, 0, {PID_KP, PID_KI, PID_KD, 0, 0}, false},
};

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t seq;
    float vx, vy, wz;
} cmd_vel_packet_t;

typedef struct {
    float px, py, pz;
    float yaw;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wz;
    uint64_t stamp_us;
} odom_state_t;

static portMUX_TYPE s_odom_mux = portMUX_INITIALIZER_UNLOCKED;
static odom_state_t s_odom;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint64_t stamp_us;
    float px, py, pz;
    float yaw;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wz;
    uint8_t seq;
} odom_packet_t;

#define SCAN_MAGIC          0x0D0D0002

static uint16_t s_scan_angle_mdeg[LIDAR_MAX_SCAN_POINTS];
static uint16_t s_scan_dist_mm[LIDAR_MAX_SCAN_POINTS];
static uint8_t s_scan_intensity[LIDAR_MAX_SCAN_POINTS];
static uint16_t s_scan_tx_angle[LIDAR_MAX_SCAN_POINTS];
static uint16_t s_scan_tx_dist[LIDAR_MAX_SCAN_POINTS];
static uint8_t s_scan_tx_intensity[LIDAR_MAX_SCAN_POINTS];
static uint16_t s_scan_tx_len;
static uint8_t s_scan_tx_seq;
static bool s_scan_tx_ready;
static portMUX_TYPE s_scan_mux = portMUX_INITIALIZER_UNLOCKED;

static void motor_brake_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        gpio_set_level(s_motor_in1[i], 1);
        gpio_set_level(s_motor_in2[i], 1);
    }
}

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

static void motor_set_target_cm_s(int index, float speed_cm_s)
{
    int new_target = (int)(speed_cm_s * COUNTS_PER_CM);

    if (new_target == 0) {
        motors[index].target_speed = 0;
        gpio_set_level(s_motor_in1[index], 0);
        gpio_set_level(s_motor_in2[index], 0);
        pid_reset(&motors[index].pid);
        return;
    }

    if ((new_target > 0) != (motors[index].target_speed > 0)) {
        pid_reset(&motors[index].pid);
    }
    motors[index].target_speed = new_target;
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

static void speed_ctrl_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t iter = 0;
    set_speed(0.0f, 0.0f, 0.0f);

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SPEED_CTRL_PERIOD_MS));

        if (esp_timer_get_time() - s_last_cmd_us >
            (uint64_t)CMD_TIMEOUT_MS * 1000) {
            set_speed(0.0f, 0.0f, 0.0f);
        }

        for (int i = 0; i < MOTOR_COUNT; i++) {
            int count = 0;
            if (pcnt_unit_get_count(s_enc_units[i], &count) != ESP_OK) {
                continue;
            }
            int delta = count - motors[i].last_count;
            motors[i].last_count = count;

            float measured_raw = (float)delta * 1000.0f / SPEED_CTRL_PERIOD_MS;
            motors[i].measured_speed +=
                (measured_raw - motors[i].measured_speed) * SPEED_FILTER_ALPHA;

            float target = (float)motors[i].target_speed;
            if (target == 0) {
                continue;
            }

            float error = fabsf(target) - fabsf(motors[i].measured_speed);
            float output = pid_update(&motors[i].pid, error,
                                      fabsf(motors[i].measured_speed),
                                      motors[i].saturated);

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

            gpio_set_level(s_motor_in1[i], target > 0);
            gpio_set_level(s_motor_in2[i], target < 0);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                          s_motor_chan[i], duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                             s_motor_chan[i]));
        }

        if (++iter % 50 == 0) {
            ESP_LOGI(TAG,
                     "motors: t=[%d %d %d] m=[%.0f %.0f %.0f] c/s",
                     motors[0].target_speed, motors[1].target_speed,
                     motors[2].target_speed,
                     (double)motors[0].measured_speed,
                     (double)motors[1].measured_speed,
                     (double)motors[2].measured_speed);
        }
    }
}

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

    for (int i = 0; i < MOTOR_COUNT; i++) {
        gpio_set_direction(s_motor_in1[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(s_motor_in2[i], GPIO_MODE_OUTPUT);

        ledc_channel_config_t chan_conf = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_motor_chan[i],
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = s_motor_pwm[i],
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&chan_conf));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, s_motor_chan[i], 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, s_motor_chan[i]));
    }

    motor_brake_all();
    ESP_LOGI(TAG, "motors forced to zero speed (braked)");
}

static void encoders_init(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pcnt_unit_config_t unit_conf = {
            .low_limit = ENC_LOW_LIMIT,
            .high_limit = ENC_HIGH_LIMIT,
            .flags.accum_count = 1,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_conf, &s_enc_units[i]));

        pcnt_glitch_filter_config_t filter_conf = {
            .max_glitch_ns = 1000,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_enc_units[i], &filter_conf));

        pcnt_chan_config_t chan_a_conf = {
            .edge_gpio_num = s_enc_pins_a[i],
            .level_gpio_num = s_enc_pins_b[i],
        };
        pcnt_channel_handle_t chan_a = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(s_enc_units[i], &chan_a_conf, &chan_a));

        pcnt_chan_config_t chan_b_conf = {
            .edge_gpio_num = s_enc_pins_b[i],
            .level_gpio_num = s_enc_pins_a[i],
        };
        pcnt_channel_handle_t chan_b = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(s_enc_units[i], &chan_b_conf, &chan_b));

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

        ESP_ERROR_CHECK(pcnt_unit_enable(s_enc_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_enc_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_enc_units[i]));
    }
    ESP_LOGI(TAG, "wheel encoders initialized");
}

static void wheel_kinematics(float *vx, float *vy, float *wz)
{
    static int s_last_counts[MOTOR_COUNT];
    float wheel_cm_s[MOTOR_COUNT];

    for (int i = 0; i < MOTOR_COUNT; i++) {
        int count = 0;
        if (pcnt_unit_get_count(s_enc_units[i], &count) != ESP_OK) {
            count = s_last_counts[i];
        }
        const int delta = count - s_last_counts[i];
        s_last_counts[i] = count;
        wheel_cm_s[i] = (float)delta / COUNTS_PER_CM * 1000.0f /
                        (float)ODOM_PERIOD_MS * ODOM_SCALE;
    }

    const float va = WHEEL_A_SIGN * wheel_cm_s[0];
    const float vb = WHEEL_B_SIGN * wheel_cm_s[1];
    const float vd = WHEEL_D_SIGN * wheel_cm_s[2];

    *wz = (va + vb + vd) / (3.0f * WHEEL_BASE_CM);
    *vy = WHEEL_BASE_CM * (*wz) - vb;
    *vx = (va - vd) / (2.0f * 0.86602540378f);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconn =
            (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "wifi disconnected, reason=%d; reconnecting",
                 (int)disconn->reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid));
    strncpy((char *)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void odom_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    uint32_t iter = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ODOM_PERIOD_MS));
        const int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = ODOM_PERIOD_MS / 1000.0f;
        }

        float vx_w, vy_w, wz_w;
        wheel_kinematics(&vx_w, &vy_w, &wz_w);
        s_wz_wheel = wz_w;

        s_yaw += wz_w * dt;
        const float c = cosf(s_yaw);
        const float s = sinf(s_yaw);
        s_px += (vx_w * c - vy_w * s) * dt;
        s_py += (vx_w * s + vy_w * c) * dt;
        s_vx = vx_w;
        s_vy = vy_w;

        portENTER_CRITICAL(&s_odom_mux);
        s_odom.px = s_px;
        s_odom.py = s_py;
        s_odom.pz = 0.0f;
        s_odom.yaw = s_yaw;
        s_odom.qw = cosf(s_yaw * 0.5f);
        s_odom.qx = 0.0f;
        s_odom.qy = 0.0f;
        s_odom.qz = sinf(s_yaw * 0.5f);
        s_odom.vx = s_vx;
        s_odom.vy = s_vy;
        s_odom.vz = 0.0f;
        s_odom.wz = wz_w;
        s_odom.stamp_us = (uint64_t)now_us;
        portEXIT_CRITICAL(&s_odom_mux);

        if (++iter % LOG_EVERY_N == 0) {
            ESP_LOGI(TAG,
                     "odom: x=%7.3f y=%7.3f yaw=%6.1f deg | "
                     "vx=%5.1f vy=%5.1f wz=%5.1f",
                     (double)s_px, (double)s_py,
                     (double)(s_yaw * RAD_TO_DEG),
                     (double)s_vx, (double)s_vy,
                     (double)(wz_w * RAD_TO_DEG));
        }
    }
}

static void lidar_publish_scan(uint16_t count)
{
    if (count == 0) {
        return;
    }
    portENTER_CRITICAL(&s_scan_mux);
    memcpy(s_scan_tx_angle, s_scan_angle_mdeg, count * sizeof(uint16_t));
    memcpy(s_scan_tx_dist, s_scan_dist_mm, count * sizeof(uint16_t));
    memcpy(s_scan_tx_intensity, s_scan_intensity, count);
    s_scan_tx_len = count;
    s_scan_tx_seq++;
    s_scan_tx_ready = true;
    portEXIT_CRITICAL(&s_scan_mux);
}

static void lidar_task(void *arg)
{
    (void)arg;

    uart_config_t uc = {
        .baud_rate = LIDAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, LIDAR_TX_GPIO, LIDAR_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "N10 lidar UART ready (baud %d)", LIDAR_BAUD);

    uint8_t pkt[108];
    uint16_t idx = 0;
    float last_deg = 0.0f;

    while (1) {
        while (uart_read_bytes(LIDAR_UART, pkt, 1, portMAX_DELAY) != 1) {
        }
        if (pkt[0] != 0xA5) {
            continue;
        }
        while (uart_read_bytes(LIDAR_UART, pkt + 1, 1, portMAX_DELAY) != 1) {
        }
        if (pkt[1] != 0x5A) {
            continue;
        }
        while (uart_read_bytes(LIDAR_UART, pkt + 2, 1, portMAX_DELAY) != 1) {
        }
        const int pkt_len = pkt[2];
        if (pkt_len != 58 && pkt_len != 108) {
            continue;
        }
        int got = 3;
        while (got < pkt_len) {
            int n = uart_read_bytes(LIDAR_UART, pkt + got, pkt_len - got,
                                    pdMS_TO_TICKS(50));
            if (n <= 0) {
                break;
            }
            got += n;
        }
        if (got < pkt_len) {
            continue;
        }

        uint16_t sum = 0;
        for (int i = 0; i < pkt_len - 1; i++) {
            sum += pkt[i];
        }
        if ((uint8_t)(sum & 0xFF) != pkt[pkt_len - 1]) {
            continue;
        }

        const float start_deg = (float)((pkt[5] << 8) | pkt[6]) / 100.0f;
        const int end_off = pkt_len - 3;
        float end_deg = (float)((pkt[end_off] << 8) | pkt[end_off + 1]) / 100.0f;
        if (end_deg > 360.0f) {
            end_deg -= 360.0f;
        }
        float interval = end_deg - start_deg;
        if (interval < 0.0f) {
            interval += 360.0f;
        }

        const float step_deg = interval / (float)(LIDAR_POINTS_PER_PKT - 1);

        for (int n = 0; n < LIDAR_POINTS_PER_PKT; n++) {
            const uint16_t d = (uint16_t)((pkt[7 + 3 * n] << 8) | pkt[8 + 3 * n]);
            if (d == 0xFFFF) {
                continue;
            }
            float deg = start_deg + step_deg * (float)n;
            if (deg >= 360.0f) {
                deg -= 360.0f;
            }

            if (idx > 0 && deg < last_deg && deg < 5.0f && last_deg > 355.0f) {
                lidar_publish_scan(idx);
                idx = 0;
            }

            if (idx < LIDAR_MAX_SCAN_POINTS) {
                s_scan_angle_mdeg[idx] = (uint16_t)lroundf(deg * 100.0f);
                s_scan_dist_mm[idx] = d;
                s_scan_intensity[idx] = pkt[9 + 3 * n];
                idx++;
            }
            last_deg = deg;
        }
    }
}

static void net_task(void *arg)
{
    (void)arg;
    odom_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = 0x0D0D0001;
    uint8_t seq = 0;
    int sock = -1;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(NET_PERIOD_MS));

        if (sock < 0) {
            sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            struct sockaddr_in addr = {
                .sin_family = AF_INET,
                .sin_port = htons(PC_PORT),
            };
            inet_pton(AF_INET, PC_IP_ADDR, &addr.sin_addr);
            if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
                ESP_LOGW(TAG, "cannot connect to %s:%d; retrying",
                         PC_IP_ADDR, PC_PORT);
                close(sock);
                sock = -1;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            ESP_LOGI(TAG, "streaming odometry to %s:%d @ %d Hz",
                     PC_IP_ADDR, PC_PORT, 1000 / NET_PERIOD_MS);
        }

        odom_state_t s;
        portENTER_CRITICAL(&s_odom_mux);
        s = s_odom;
        portEXIT_CRITICAL(&s_odom_mux);

        pkt.stamp_us = s.stamp_us;
        pkt.px = s.px;
        pkt.py = s.py;
        pkt.pz = s.pz;
        pkt.yaw = s.yaw;
        pkt.qw = s.qw;
        pkt.qx = s.qx;
        pkt.qy = s.qy;
        pkt.qz = s.qz;
        pkt.vx = s.vx;
        pkt.vy = s.vy;
        pkt.vz = s.vz;
        pkt.wz = s.wz;
        pkt.seq = seq++;

        int sent = send(sock, &pkt, sizeof(pkt), 0);
        if (sent < 0) {
            ESP_LOGW(TAG, "send failed; reconnecting");
            close(sock);
            sock = -1;
            continue;
        }

        uint16_t scan_len = 0;
        portENTER_CRITICAL(&s_scan_mux);
        if (s_scan_tx_ready) {
            scan_len = s_scan_tx_len;
            s_scan_tx_ready = false;
        }
        portEXIT_CRITICAL(&s_scan_mux);

        if (scan_len > 0) {
            uint8_t hdr[7];
            uint32_t magic = SCAN_MAGIC;
            memcpy(hdr, &magic, 4);
            hdr[4] = s_scan_tx_seq;
            hdr[5] = (uint8_t)(scan_len & 0xFF);
            hdr[6] = (uint8_t)(scan_len >> 8);
            int sent1 = send(sock, hdr, sizeof(hdr), 0);
            int sent2 = send(sock, s_scan_tx_angle, scan_len * sizeof(uint16_t), 0);
            int sent3 = send(sock, s_scan_tx_dist, scan_len * sizeof(uint16_t), 0);
            int sent4 = send(sock, s_scan_tx_intensity, scan_len, 0);
            if (sent1 < 0 || sent2 < 0 || sent3 < 0 || sent4 < 0) {
                ESP_LOGW(TAG, "scan send failed; reconnecting");
                close(sock);
                sock = -1;
            }
        }

        static uint8_t s_cmd_buf[128];
        static int s_cmd_len;
        uint8_t rxbuf[128];
        int n = recv(sock, rxbuf, sizeof(rxbuf), MSG_DONTWAIT);
        if (n > 0) {
            for (int i = 0; i < n && s_cmd_len < (int)sizeof(s_cmd_buf); i++) {
                s_cmd_buf[s_cmd_len++] = rxbuf[i];
            }
            while (s_cmd_len >= (int)sizeof(cmd_vel_packet_t)) {
                cmd_vel_packet_t cmd;
                memcpy(&cmd, s_cmd_buf, sizeof(cmd));
                if (cmd.magic != CMD_MAGIC) {
                    memmove(s_cmd_buf, s_cmd_buf + 1, sizeof(s_cmd_buf) - 1);
                    s_cmd_len--;
                    continue;
                }
                set_speed(cmd.vx * 100.0f,
                          cmd.vy * 100.0f,
                          cmd.wz);
                s_last_cmd_us = (uint64_t)esp_timer_get_time();
                static uint32_t s_cmd_log_skip;
                if (++s_cmd_log_skip % 25 == 0) {
                    ESP_LOGI(TAG, "cmd_vel: vx=%.2f vy=%.2f wz=%.2f",
                             (double)cmd.vx, (double)cmd.vy, (double)cmd.wz);
                }
                memmove(s_cmd_buf, s_cmd_buf + sizeof(cmd),
                        sizeof(s_cmd_buf) - sizeof(cmd));
                s_cmd_len -= sizeof(cmd);
            }
        } else if (n == 0) {
            ESP_LOGW(TAG, "connection closed by host; reconnecting");
            close(sock);
            sock = -1;
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    motors_init();
    encoders_init();
    assert(xTaskCreate(odom_task, "odom", 4096, NULL, 5, NULL) == pdPASS);
    assert(xTaskCreate(lidar_task, "lidar", 4096, NULL, 5, NULL) == pdPASS);
    assert(xTaskCreate(speed_ctrl_task, "speed_ctrl", 4096, NULL, 5, NULL) == pdPASS);
    xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 4, NULL, 1);
}
