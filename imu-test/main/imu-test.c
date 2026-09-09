#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

static const char *TAG = "imu";

#define PIN_I2C_SCL         GPIO_NUM_21
#define PIN_I2C_SDA         GPIO_NUM_47
#define I2C_CLK_HZ          400000
#define MPU_ADDR            0x68

#define PC_IP_ADDR          "192.168.0.200"
#define PC_PORT             34567
#define NET_PERIOD_MS       20

#define WIFI_SSID           "iarc-thu5G"
#define WIFI_PASSWORD       "12345678"

#define MOTOR_COUNT         3
#define PWM_FREQ_HZ         20000

static const int s_motor_pwm[MOTOR_COUNT] = {GPIO_NUM_1, GPIO_NUM_4, GPIO_NUM_7};
static const int s_motor_in1[MOTOR_COUNT] = {GPIO_NUM_42, GPIO_NUM_5, GPIO_NUM_16};
static const int s_motor_in2[MOTOR_COUNT] = {GPIO_NUM_2, GPIO_NUM_6, GPIO_NUM_15};
static const ledc_channel_t s_motor_chan[MOTOR_COUNT] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

#define IMU_PERIOD_MS       10
#define LOG_EVERY_N         20
#define GYRO_CAL_SAMPLES    500

#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    131.0f
#define DEG_TO_RAD          (3.14159265f / 180.0f)
#define RAD_TO_DEG          (180.0f / 3.14159265f)
#define GRAVITY_M_S2        9.81f

#define ENC_HIGH_LIMIT      10000
#define ENC_LOW_LIMIT       -10000
#define COUNTS_PER_REV      512
#define WHEEL_DIAMETER_CM   5.5f
#define WHEEL_BASE_CM       9.0f
#define COUNTS_PER_CM       ((float)COUNTS_PER_REV / (3.14159265f * WHEEL_DIAMETER_CM))
#define TILT_FREEZE_RAD     0.2618f
#define SIGN_CAL_TIME_MS    3000
#define SIGN_CAL_MIN_GZ     0.8f

#define ACC_AX_SIGN         1.0f
#define ACC_AY_SIGN         1.0f
#define ACC_AZ_SIGN         1.0f
#define GYR_GX_SIGN         1.0f
#define GYR_GY_SIGN         1.0f
#define GYR_GZ_SIGN         1.0f

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static float s_q0 = 1.0f, s_q1 = 0.0f, s_q2 = 0.0f, s_q3 = 0.0f;
static float s_roll, s_pitch, s_yaw;
static float s_heading;
static float s_vx, s_vy, s_vz;
static float s_px, s_py, s_pz;
static float s_gx_bias, s_gy_bias, s_gz_bias;
static float s_wheel_sign[MOTOR_COUNT] = {1.0f, 1.0f, 1.0f};
static float s_wz_wheel;
static float s_wheel_cm_s_log[MOTOR_COUNT];

typedef struct {
    float px, py, pz;
    float vx, vy, vz;
    float q0, q1, q2, q3;
    float gx, gy, gz;
    float ax, ay, az;
    float roll, pitch, yaw;
    uint64_t stamp_us;
} odom_state_t;

static portMUX_TYPE s_odom_mux = portMUX_INITIALIZER_UNLOCKED;
static odom_state_t s_odom;

static void mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t cmd[2] = {reg, val};
    ESP_ERROR_CHECK(i2c_master_transmit(s_dev, cmd, sizeof(cmd), -1));
}

static void mpu_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    ESP_ERROR_CHECK(i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, -1));
}

static esp_err_t mpu_read_raw(int16_t raw[6])
{
    uint8_t reg = 0x3B;
    uint8_t buf[14];
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, buf,
                                                sizeof(buf), 20);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < 6; i++) {
        raw[i] = (int16_t)((buf[i * 2] << 8) | buf[i * 2 + 1]);
    }
    return ESP_OK;
}

static void mpu_init(void)
{
    uint8_t who = 0;
    mpu_read_regs(0x75, &who, 1);
    ESP_LOGI(TAG, "IMU WHO_AM_I=0x%02X (MPU6500=0x70, MPU6050=0x68)", who);

    mpu_write_reg(0x6B, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write_reg(0x6B, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    mpu_write_reg(0x19, 0x00);
    mpu_write_reg(0x1A, 0x03);
    mpu_write_reg(0x1B, 0x00);
    mpu_write_reg(0x1C, 0x00);
    mpu_write_reg(0x1D, 0x03);
    ESP_LOGI(TAG, "MPU6500 initialized: gyro 250dps, accel 2g, DLPF 41Hz");
}

static int cmp_i16(const void *a, const void *b)
{
    return (int)*(const int16_t *)a - (int)*(const int16_t *)b;
}

static void gyro_calibrate(void)
{
    static int16_t gx_s[GYRO_CAL_SAMPLES];
    static int16_t gy_s[GYRO_CAL_SAMPLES];
    static int16_t gz_s[GYRO_CAL_SAMPLES];

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        int16_t raw[6];
        int retries = 10;
        while (mpu_read_raw(raw) != ESP_OK && retries-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
        }
        gx_s[i] = raw[3];
        gy_s[i] = raw[4];
        gz_s[i] = raw[5];
        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
    }

    qsort(gx_s, GYRO_CAL_SAMPLES, sizeof(int16_t), cmp_i16);
    qsort(gy_s, GYRO_CAL_SAMPLES, sizeof(int16_t), cmp_i16);
    qsort(gz_s, GYRO_CAL_SAMPLES, sizeof(int16_t), cmp_i16);
    s_gx_bias = (float)gx_s[GYRO_CAL_SAMPLES / 2];
    s_gy_bias = (float)gy_s[GYRO_CAL_SAMPLES / 2];
    s_gz_bias = (float)gz_s[GYRO_CAL_SAMPLES / 2];

    ESP_LOGI(TAG, "gyro bias: %.1f %.1f %.1f LSB",
             (double)s_gx_bias, (double)s_gy_bias, (double)s_gz_bias);
}

static void mahony_imu_update(float gx, float gy, float gz,
                              float ax, float ay, float az, float dt)
{
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        return;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;

    const float q0 = s_q0, q1 = s_q1, q2 = s_q2, q3 = s_q3;

    const float vx = 2.0f * (q1 * q3 - q0 * q2);
    const float vy = 2.0f * (q0 * q1 + q2 * q3);
    const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    const float ex = ay * vz - az * vy;
    const float ey = az * vx - ax * vz;
    const float ez = ax * vy - ay * vx;

    const float kp = 1.0f;
    gx += kp * ex;
    gy += kp * ey;
    gz += kp * ez;

    const float qa = q0, qb = q1, qc = q2;
    const float qn0 = q0 + 0.5f * (-qb * gx - qc * gy - q3 * gz) * dt;
    const float qn1 = q1 + 0.5f * (qa * gx + qc * gz - q3 * gy) * dt;
    const float qn2 = q2 + 0.5f * (qa * gy - qb * gz + q3 * gx) * dt;
    const float qn3 = q3 + 0.5f * (qa * gz + qb * gy - qc * gx) * dt;

    norm = sqrtf(qn0 * qn0 + qn1 * qn1 + qn2 * qn2 + qn3 * qn3);
    s_q0 = qn0 / norm;
    s_q1 = qn1 / norm;
    s_q2 = qn2 / norm;
    s_q3 = qn3 / norm;
}

static void quat_to_rpy(void)
{
    const float q0 = s_q0, q1 = s_q1, q2 = s_q2, q3 = s_q3;
    s_roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                    1.0f - 2.0f * (q1 * q1 + q2 * q2));
    s_pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    s_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                   1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

static pcnt_unit_handle_t s_enc_units[MOTOR_COUNT];
static const int s_enc_pins_a[MOTOR_COUNT] = {GPIO_NUM_41, GPIO_NUM_8, GPIO_NUM_17};
static const int s_enc_pins_b[MOTOR_COUNT] = {GPIO_NUM_40, GPIO_NUM_3, GPIO_NUM_18};

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

static void wheel_kinematics(float gz_rad_s, float *vx, float *vy, float *wz)
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
                        (float)IMU_PERIOD_MS;
    }

    const float va = s_wheel_sign[0] * wheel_cm_s[0];
    const float vb = s_wheel_sign[1] * wheel_cm_s[1];
    const float vd = s_wheel_sign[2] * wheel_cm_s[2];
    s_wheel_cm_s_log[0] = va;
    s_wheel_cm_s_log[1] = vb;
    s_wheel_cm_s_log[2] = vd;

    s_wz_wheel = (va + vb + vd) / (3.0f * WHEEL_BASE_CM);

    *wz = gz_rad_s;
    const float va_r = va - WHEEL_BASE_CM * gz_rad_s;
    const float vb_r = vb - WHEEL_BASE_CM * gz_rad_s;
    const float vd_r = vd - WHEEL_BASE_CM * gz_rad_s;
    *vx = (va_r - vd_r) / (2.0f * 0.86602540378f);
    *vy = -vb_r;
}

static void encoder_sign_calibrate(void)
{
    ESP_LOGI(TAG, "rotate the car in place CCW (counter-clockwise) for %d s "
             "to calibrate encoder signs", SIGN_CAL_TIME_MS / 1000);
    vTaskDelay(pdMS_TO_TICKS(2000));

    double gz_sum = 0.0;
    double cnt_sum[MOTOR_COUNT] = {0};
    int last_counts[MOTOR_COUNT] = {0};
    const int64_t start_us = esp_timer_get_time();

    while (esp_timer_get_time() - start_us < (int64_t)SIGN_CAL_TIME_MS * 1000) {
        int16_t raw[6];
        if (mpu_read_raw(raw) == ESP_OK) {
            gz_sum += GYR_GZ_SIGN * ((float)raw[5] - s_gz_bias) /
                      GYRO_LSB_PER_DPS;
        }
        for (int i = 0; i < MOTOR_COUNT; i++) {
            int count = 0;
            if (pcnt_unit_get_count(s_enc_units[i], &count) == ESP_OK) {
                cnt_sum[i] += count - last_counts[i];
                last_counts[i] = count;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
    }

    if (fabsf((float)gz_sum) < SIGN_CAL_MIN_GZ) {
        ESP_LOGW(TAG, "no significant rotation detected; "
                 "keeping default encoder signs");
        return;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (cnt_sum[i] * gz_sum < 0.0) {
            s_wheel_sign[i] = -1.0f;
        }
        ESP_LOGI(TAG, "wheel %d: sign=%+d (counts=%d, gz=%d)",
                 i, s_wheel_sign[i] > 0 ? 1 : -1,
                 (int)cnt_sum[i], (int)gz_sum);
    }
}

static void imu_task(void *arg)
{
    (void)arg;
    gyro_calibrate();
    encoder_sign_calibrate();

    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    uint32_t iter = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
        const int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = IMU_PERIOD_MS / 1000.0f;
        }

        int16_t raw[6];
        esp_err_t read_err = mpu_read_raw(raw);
        if (read_err != ESP_OK) {
            static uint32_t s_i2c_errors;
            static uint32_t s_reported;
            s_i2c_errors++;
            if (s_i2c_errors - s_reported >= 100) {
                ESP_LOGW(TAG, "i2c read failing: %s (%lu total)",
                         esp_err_to_name(read_err), (unsigned long)s_i2c_errors);
                s_reported = s_i2c_errors;
                i2c_master_bus_reset(s_bus);
            }
            continue;
        }

        const float ax = ACC_AX_SIGN * (float)raw[0] / ACCEL_LSB_PER_G * GRAVITY_M_S2;
        const float ay = ACC_AY_SIGN * (float)raw[1] / ACCEL_LSB_PER_G * GRAVITY_M_S2;
        const float az = ACC_AZ_SIGN * (float)raw[2] / ACCEL_LSB_PER_G * GRAVITY_M_S2;
        const float gx = GYR_GX_SIGN * ((float)raw[3] - s_gx_bias) /
                         GYRO_LSB_PER_DPS * DEG_TO_RAD;
        const float gy = GYR_GY_SIGN * ((float)raw[4] - s_gy_bias) /
                         GYRO_LSB_PER_DPS * DEG_TO_RAD;
        const float gz = GYR_GZ_SIGN * ((float)raw[5] - s_gz_bias) /
                         GYRO_LSB_PER_DPS * DEG_TO_RAD;

        float vx_w, vy_w;
        wheel_kinematics(gz, &vx_w, &vy_w, &s_wz_wheel);

        const bool level = fabsf(s_roll) < TILT_FREEZE_RAD &&
                           fabsf(s_pitch) < TILT_FREEZE_RAD;
        if (level) {
            s_heading += gz * dt;

            const float ch = cosf(s_heading);
            const float sh = sinf(s_heading);
            s_px += (vx_w * ch - vy_w * sh) * dt;
            s_py += (vx_w * sh + vy_w * ch) * dt;
            s_vx = vx_w;
            s_vy = vy_w;
        } else {
            s_vx = 0.0f;
            s_vy = 0.0f;
        }
        s_pz = 0.0f;
        s_vz = 0.0f;

        mahony_imu_update(gx, gy, gz, ax, ay, az, dt);
        quat_to_rpy();

        const float cr = cosf(s_roll * 0.5f), sr = sinf(s_roll * 0.5f);
        const float cp = cosf(s_pitch * 0.5f), sp = sinf(s_pitch * 0.5f);
        const float ch = cosf(s_heading * 0.5f), sh = sinf(s_heading * 0.5f);
        const float qf_w = ch * cp * cr + sh * sp * sr;
        const float qf_x = ch * cp * sr - sh * sp * cr;
        const float qf_y = ch * sp * cr + sh * cp * sr;
        const float qf_z = sh * cp * cr - ch * sp * sr;

        portENTER_CRITICAL(&s_odom_mux);
        s_odom.px = s_px;
        s_odom.py = s_py;
        s_odom.pz = s_pz;
        s_odom.vx = s_vx;
        s_odom.vy = s_vy;
        s_odom.vz = s_vz;
        s_odom.q0 = qf_w;
        s_odom.q1 = qf_x;
        s_odom.q2 = qf_y;
        s_odom.q3 = qf_z;
        s_odom.gx = gx;
        s_odom.gy = gy;
        s_odom.gz = gz;
        s_odom.ax = ax;
        s_odom.ay = ay;
        s_odom.az = az;
        s_odom.roll = s_roll;
        s_odom.pitch = s_pitch;
        s_odom.yaw = s_heading;
        s_odom.stamp_us = (uint64_t)now_us;
        portEXIT_CRITICAL(&s_odom_mux);

        if (++iter % LOG_EVERY_N == 0) {
            ESP_LOGI(TAG,
                     "rpy(deg): R=%6.1f P=%6.1f Y=%6.1f | "
                     "pos(m): x=%7.3f y=%7.3f | "
                     "gz=%5.1f dps | va=%5.1f vb=%5.1f vd=%5.1f cm/s",
                     (double)(s_roll * RAD_TO_DEG),
                     (double)(s_pitch * RAD_TO_DEG),
                     (double)(s_heading * RAD_TO_DEG),
                     (double)s_px, (double)s_py,
                     (double)(gz * RAD_TO_DEG),
                     (double)s_wheel_cm_s_log[0],
                     (double)s_wheel_cm_s_log[1],
                     (double)s_wheel_cm_s_log[2]);
        }
    }
}

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint64_t stamp_us;
    float roll, pitch, yaw;
    float q0, q1, q2, q3;
    float px, py, pz;
    float vx, vy, vz;
    float gx, gy, gz;
    float ax, ay, az;
    uint8_t seq;
} imu_packet_t;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)data;
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
    strncpy((char *)wc.sta.password, WIFI_PASSWORD,
            sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void net_task(void *arg)
{
    (void)arg;
    imu_packet_t pkt;
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
            ESP_LOGI(TAG, "streaming IMU/odometry to %s:%d @ %d Hz",
                     PC_IP_ADDR, PC_PORT, 1000 / NET_PERIOD_MS);
        }

        odom_state_t s;
        portENTER_CRITICAL(&s_odom_mux);
        s = s_odom;
        portEXIT_CRITICAL(&s_odom_mux);

        pkt.stamp_us = s.stamp_us;
        pkt.roll = s.roll;
        pkt.pitch = s.pitch;
        pkt.yaw = s.yaw;
        pkt.q0 = s.q0;
        pkt.q1 = s.q1;
        pkt.q2 = s.q2;
        pkt.q3 = s.q3;
        pkt.px = s.px;
        pkt.py = s.py;
        pkt.pz = s.pz;
        pkt.vx = s.vx;
        pkt.vy = s.vy;
        pkt.vz = s.vz;
        pkt.gx = s.gx;
        pkt.gy = s.gy;
        pkt.gz = s.gz;
        pkt.ax = s.ax;
        pkt.ay = s.ay;
        pkt.az = s.az;
        pkt.seq = seq++;

        int sent = send(sock, &pkt, sizeof(pkt), 0);
        if (sent < 0) {
            ESP_LOGW(TAG, "send failed; reconnecting");
            close(sock);
            sock = -1;
        }
    }
}

static void motor_brake_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        gpio_set_level(s_motor_in1[i], 1);
        gpio_set_level(s_motor_in2[i], 1);
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

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = PIN_I2C_SCL,
        .sda_io_num = PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = I2C_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    mpu_init();
    motors_init();
    encoders_init();
    assert(xTaskCreate(imu_task, "imu", 6144, NULL, 5, NULL) == pdPASS);
    xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 4, NULL, 1);
}
