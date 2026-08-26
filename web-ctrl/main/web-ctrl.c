#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "web-ctrl";

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

#define WIFI_SSID      "ESP32-CAR"
#define WIFI_PASSWORD  "12345678"

#define CMD_TIMEOUT_US      600000
#define MAX_VXY_CM_S        25.0f
#define MAX_WZ_RAD_S        2.0f

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

static void set_speed(float vx, float vy, float wz)
{
    float va = 0.86602540378f * vx + 0.5f * vy + WHEEL_BASE_CM * wz;
    float vd = -0.86602540378f * vx + 0.5f * vy + WHEEL_BASE_CM * wz;
    float vb = -vy + WHEEL_BASE_CM * wz;

    motor_set_target_cm_s(0, va);
    motor_set_target_cm_s(1, vb);
    motor_set_target_cm_s(2, vd);
}

static volatile int64_t g_last_cmd_time = 0;

static const char PAGE_HTML[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32 Car</title>"
"<style>"
"body{font-family:sans-serif;background:#111;color:#eee;text-align:center;"
"-webkit-user-select:none;user-select:none;touch-action:none}"
"h2{margin:15px 0}"
".grid{display:grid;grid-template-columns:repeat(3,84px);gap:8px;justify-content:center}"
"button{padding:20px 0;font-size:22px;border-radius:10px;border:none;background:#333;color:#fff}"
"button:active{background:#0a6}"
"#stop{background:#933}#stop:active{background:#0a6}"
".row{display:flex;gap:8px;justify-content:center;margin-top:8px}"
"</style></head><body>"
"<h2>ESP32-S3 Car</h2>"
"<div class='grid'>"
"<button onmouseover=\"start(14,14,0)\" ontouchstart=\"evt(event,14,14,0)\" ontouchend=\"evt(event)\">&#8598;</button>"
"<button onmouseover=\"start(20,0,0)\" ontouchstart=\"evt(event,20,0,0)\" ontouchend=\"evt(event)\">&#8593;</button>"
"<button onmouseover=\"start(14,-14,0)\" ontouchstart=\"evt(event,14,-14,0)\" ontouchend=\"evt(event)\">&#8599;</button>"
"<button onmouseover=\"start(0,20,0)\" ontouchstart=\"evt(event,0,20,0)\" ontouchend=\"evt(event)\">&#8592;</button>"
"<button id='stop' onclick=\"stop()\">STOP</button>"
"<button onmouseover=\"start(0,-20,0)\" ontouchstart=\"evt(event,0,-20,0)\" ontouchend=\"evt(event)\">&#8594;</button>"
"<button onmouseover=\"start(-14,14,0)\" ontouchstart=\"evt(event,-14,14,0)\" ontouchend=\"evt(event)\">&#8601;</button>"
"<button onmouseover=\"start(-20,0,0)\" ontouchstart=\"evt(event,-20,0,0)\" ontouchend=\"evt(event)\">&#8595;</button>"
"<button onmouseover=\"start(-14,-14,0)\" ontouchstart=\"evt(event,-14,-14,0)\" ontouchend=\"evt(event)\">&#8600;</button>"
"</div>"
"<div class='row'>"
"<button onmouseover=\"start(0,0,1.5)\" ontouchstart=\"evt(event,0,0,1.5)\" ontouchend=\"evt(event)\">&#8634;</button>"
"<button onmouseover=\"start(0,0,-1.5)\" ontouchstart=\"evt(event,0,0,-1.5)\" ontouchend=\"evt(event)\">&#8635;</button>"
"</div>"
"<p id='st'>last: -</p>"
"<script>"
"var t=null;"
"function send(vx,vy,wz){document.getElementById('st').textContent=vx+','+vy+','+wz;fetch('/cmd?vx='+vx+'&vy='+vy+'&wz='+wz)}"
"function start(vx,vy,wz){stopT();send(vx,vy,wz);t=setInterval(function(){send(vx,vy,wz)},250)}"
"function stopT(){if(t){clearInterval(t);t=null}}"
"function stop(){stopT();send(0,0,0)}"
"function evt(e,a,b,c){e.preventDefault();if(e.type==='touchstart'){if(a!==undefined)start(a,b,c)}else stopT()}"
"</script></body></html>";

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char query[64] = {0};
    char val[16] = {0};
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "vx", val, sizeof(val)) == ESP_OK) {
            vx = atof(val);
        }
        if (httpd_query_key_value(query, "vy", val, sizeof(val)) == ESP_OK) {
            vy = atof(val);
        }
        if (httpd_query_key_value(query, "wz", val, sizeof(val)) == ESP_OK) {
            wz = atof(val);
        }
    }

    if (vx > MAX_VXY_CM_S) {
        vx = MAX_VXY_CM_S;
    } else if (vx < -MAX_VXY_CM_S) {
        vx = -MAX_VXY_CM_S;
    }
    if (vy > MAX_VXY_CM_S) {
        vy = MAX_VXY_CM_S;
    } else if (vy < -MAX_VXY_CM_S) {
        vy = -MAX_VXY_CM_S;
    }
    if (wz > MAX_WZ_RAD_S) {
        wz = MAX_WZ_RAD_S;
    } else if (wz < -MAX_WZ_RAD_S) {
        wz = -MAX_WZ_RAD_S;
    }

    set_speed(vx, vy, wz);
    g_last_cmd_time = esp_timer_get_time();

    httpd_resp_sendstr(req, "ok");
    return ESP_OK;
}

static void start_http_server(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t cmd_uri = {
        .uri = "/cmd",
        .method = HTTP_GET,
        .handler = cmd_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &cmd_uri);

    ESP_LOGI(TAG, "http server started on http://192.168.4.1/");
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " connected", MAC2STR(event->mac));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " disconnected", MAC2STR(event->mac));
    }
}

static void wifi_init_softap(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASSWORD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi ap started: ssid=%s password=%s", WIFI_SSID, WIFI_PASSWORD);
}

void app_main(void)
{
    motors_init();
    ESP_LOGI(TAG, "motors initialized");

    wifi_init_softap();
    start_http_server();

    xTaskCreate(speed_ctrl_task, "speed_ctrl", 4096, NULL, 5, NULL);

    while (1) {
        if (g_last_cmd_time != 0 && esp_timer_get_time() - g_last_cmd_time > CMD_TIMEOUT_US) {
            g_last_cmd_time = 0;
            set_speed(0.0f, 0.0f, 0.0f);
            for (int i = 0; i < MOTOR_COUNT; i++) {
                motor_brake(i);
            }
            ESP_LOGW(TAG, "cmd timeout, stop");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
