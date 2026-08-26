#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "libuvc/libuvc.h"
#include "libuvc_helper.h"
#include "libuvc_adapter.h"
#include "nvs_flash.h"
#include "usb/usb_host.h"

static const char *TAG = "camera-test";

#define WIFI_SSID      "ESP32-CAM"
#define WIFI_PASSWORD  "12345678"

#define FRAME_MAX          (64 * 1024)
#define LIBUVC_BUF_CAP     (16 * 1024)
#define MAX_CLIENTS        1
#define NEGOTIATION_ATTEMPTS 3

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
    {UVC_FRAME_FORMAT_MJPEG, 320, 240, 15},
    {UVC_FRAME_FORMAT_MJPEG, 320, 240, 0},
    {UVC_FRAME_FORMAT_MJPEG, 640, 480, 10},
    {UVC_FRAME_FORMAT_MJPEG, 640, 480, 0},
    {UVC_FRAME_FORMAT_MJPEG, 160, 120, 0},
    {UVC_FRAME_FORMAT_ANY, 0, 0, 0},
};

static uint8_t s_last_frame[FRAME_MAX];
static size_t s_last_frame_len;
static SemaphoreHandle_t s_frame_mutex;
static EventGroupHandle_t s_app_flags;

static const char PAGE_HTML[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32 Cam</title>"
"<style>"
"body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0}"
"h2{margin:12px 0}"
"img{max-width:100%;height:auto;border:2px solid #444;border-radius:8px;transform:rotate(180deg)}"
"p{color:#888;font-size:13px}"
"</style></head><body>"
"<h2>ESP32-S3 USB Camera</h2>"
"<img id='cam' src='/frame'>"
"<p id='st'>waiting for first frame...</p>"
"<p>connect PC to hotspot ESP32-CAM, open http://192.168.4.1</p>"
"<script>"
"var im=document.getElementById('cam'),n=0;"
"function next(){im.src='/frame?_t='+Date.now()}"
"im.onload=function(){n++;document.getElementById('st').textContent='frames loaded: '+n;next()};"
"im.onerror=function(){document.getElementById('st').textContent='load error, retrying...';setTimeout(next,500)}"
"</script>"
"</body></html>";

static uint8_t s_jpeg_buf[FRAME_MAX];
static size_t s_jpeg_len;
static bool s_jpeg_in_frame;
static uint8_t s_jpeg_last_byte;

static void publish_frame(const uint8_t *data, size_t len)
{
    static size_t fps;
    static int64_t start_time;

    int64_t current_time = esp_timer_get_time();
    fps++;
    if (!start_time) {
        start_time = current_time;
    }
    if (current_time > start_time + 1000000) {
        ESP_LOGI(TAG, "camera fps: %u, frame bytes: %u", fps, (unsigned)len);
        start_time = current_time;
        fps = 0;
    }

    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    memcpy(s_last_frame, data, len);
    s_last_frame_len = len;
    xSemaphoreGive(s_frame_mutex);
}

static void frame_callback(uvc_frame_t *frame, void *ptr)
{
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
            publish_frame(s_jpeg_buf, s_jpeg_len);
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

        libuvc_adapter_print_descriptors(devh);
        uvc_print_diag(devh, stderr);

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

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t frame_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    size_t len = s_last_frame_len;
    if (len == 0) {
        xSemaphoreGive(s_frame_mutex);
        ESP_LOGW(TAG, "frame request but no frame yet");
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no frame yet");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "frame request: sending %u bytes", (unsigned)len);
    esp_err_t err = httpd_resp_send(req, (const char *)s_last_frame, len);
    xSemaphoreGive(s_frame_mutex);
    ESP_LOGI(TAG, "frame send result: %d", err);
    return err;
}

static esp_err_t dump_handler(httpd_req_t *req)
{
    char buf[512];
    int off = 0;

    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    size_t len = s_last_frame_len;
    off += snprintf(buf + off, sizeof(buf) - off, "len=%u\n", (unsigned)len);
    size_t n = len < 32 ? len : 32;
    for (size_t i = 0; i < n && off < (int)sizeof(buf) - 8; i++) {
        off += snprintf(buf + off, sizeof(buf) - off, "%02x ", s_last_frame[i]);
    }
    xSemaphoreGive(s_frame_mutex);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, buf, off);
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

    httpd_uri_t frame_uri = {
        .uri = "/frame",
        .method = HTTP_GET,
        .handler = frame_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &frame_uri);

    httpd_uri_t dump_uri = {
        .uri = "/dump",
        .method = HTTP_GET,
        .handler = dump_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &dump_uri);

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
    s_app_flags = xEventGroupCreate();
    assert(s_app_flags);

    s_frame_mutex = xSemaphoreCreateMutex();
    assert(s_frame_mutex);

    wifi_init_softap();
    start_http_server();
    ESP_LOGI(TAG, "free heap after httpd: %lu", esp_get_free_heap_size());

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
