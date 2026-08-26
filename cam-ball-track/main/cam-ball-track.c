#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "jpeg_decoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "libuvc/libuvc.h"
#include "libuvc_adapter.h"
#include "usb/usb_host.h"

static const char *TAG = "red-detect";

#define MAX_MODE_CANDIDATES          32
#define PREFERRED_WIDTH              320
#define PREFERRED_HEIGHT             240
#define PREFERRED_FPS                15
#define MAX_SOURCE_WIDTH             640
#define MAX_SOURCE_HEIGHT            480
#define DECODE_BUF_BYTES             ((PREFERRED_WIDTH / 2) * (PREFERRED_HEIGHT / 2) * 2)
#define JPEG_FRAME_MAX_BYTES         (512 * 1024)
#define FRAME_SLOT_COUNT              2
#define RED_MIN_R                    120
#define RED_DOMINANCE                40
#define RED_DETECT_RATIO_PERCENT     2.0f

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

static bool is_mjpeg_format(const uvc_format_desc_t *format)
{
    return format->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG;
}

static bool is_red_rgb565(uint16_t pixel)
{
    const int r = ((pixel >> 11) & 0x1f) * 255 / 31;
    const int g = ((pixel >> 5) & 0x3f) * 255 / 63;
    const int b = (pixel & 0x1f) * 255 / 31;
    return r >= RED_MIN_R && r >= g + RED_DOMINANCE && r >= b + RED_DOMINANCE;
}

static void detect_red_mjpeg(const uint8_t *jpeg, size_t jpeg_len)
{
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)jpeg,
        .indata_size = jpeg_len,
        .outbuf = s_decode_buf,
        .outbuf_size = sizeof(s_decode_buf),
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = s_decode_scale_divisor == 2 ?
                     JPEG_IMAGE_SCALE_1_4 : JPEG_IMAGE_SCALE_1_4,
        .flags = {
            .swap_color_bytes = 0,
        },
    };
    esp_jpeg_image_output_t image = {0};
    esp_err_t err = esp_jpeg_decode(&jpeg_cfg, &image);
    if (err != ESP_OK || image.width == 0 || image.height == 0) {
        ESP_LOGW(TAG, "JPEG decode failed: %s", esp_err_to_name(err));
        return;
    }

    const uint16_t *decoded = (const uint16_t *)s_decode_buf;
    uint32_t red_count = 0;
    uint64_t sum_x = 0;
    uint64_t sum_y = 0;

    for (uint32_t y = 0; y < image.height; ++y) {
        for (uint32_t x = 0; x < image.width; ++x) {
            if (is_red_rgb565(decoded[(size_t)y * image.width + x])) {
                ++red_count;
                sum_x += x;
                sum_y += y;
            }
        }
    }

    const uint32_t pixel_count = (uint32_t)image.width * image.height;
    const float ratio = 100.0f * red_count / pixel_count;
    if (red_count != 0 && ratio >= RED_DETECT_RATIO_PERCENT) {
        ESP_LOGI(TAG, "RED DETECTED red_pixels=%u center=(%u,%u) red=%.2f%%",
                 (unsigned)red_count,
                 (unsigned)(sum_x * s_decode_scale_divisor / red_count),
                 (unsigned)(sum_y * s_decode_scale_divisor / red_count),
                 (double)ratio);
    } else {
        ESP_LOGI(TAG, "red_pixels=%u center=(-1,-1) red=%.2f%%",
                 (unsigned)red_count, (double)ratio);
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
            detect_red_mjpeg(slot->data, slot->len);
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
            ESP_LOGW(TAG, "capture overloaded: %u frame(s) dropped; camera target is 15 FPS",
                     (unsigned)(drops - last_reported_drops));
            last_reported_drops = drops;
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
        s_decode_scale_divisor = 2;
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
                s_decode_scale_divisor =
                    frame->wWidth <= PREFERRED_WIDTH && frame->wHeight <= PREFERRED_HEIGHT ? 2 : 4;
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

    libuvc_adapter_config_t adapter_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = libuvc_adapter_cb,
    };
    libuvc_adapter_set_config(&adapter_config);
    assert(xTaskCreate(camera_task, "camera", 4096, NULL, 5, NULL) == pdPASS);
}
