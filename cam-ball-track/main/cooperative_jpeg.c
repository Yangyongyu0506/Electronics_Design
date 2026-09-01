#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tjpgd.h"

#include "cooperative_jpeg.h"

#if JD_FORMAT != 0
#error "cooperative_jpeg requires CONFIG_JD_FORMAT_RGB888=y"
#endif

#define JPEG_WORK_BUF_SIZE 8192
/* Yield after a bounded amount of TJpgDec work.  One output callback handles
 * one MCU, so the decoder can neither monopolise a core nor starve its IDLE
 * task, even when a UVC frame is slow to decode. */
#define JPEG_YIELD_EVERY_MCUS 128

typedef struct {
    esp_jpeg_image_cfg_t *cfg;
    uint32_t output_calls;
} cooperative_jpeg_ctx_t;

static const char *jpeg_result_name(JRESULT result)
{
    static const char *const names[] = {
        "OK", "INTR", "INP", "MEM1", "MEM2", "PAR", "FMT1", "FMT2", "FMT3",
    };
    return result <= JDR_FMT3 ? names[result] : "unknown";
}

static uint8_t scale_divisor(esp_jpeg_image_scale_t scale)
{
    switch (scale) {
    case JPEG_IMAGE_SCALE_0:   return 1;
    case JPEG_IMAGE_SCALE_1_2: return 2;
    case JPEG_IMAGE_SCALE_1_4: return 4;
    case JPEG_IMAGE_SCALE_1_8: return 8;
    default:                   return 0;
    }
}

static size_t jpeg_input(JDEC *jd, uint8_t *buffer, size_t requested)
{
    cooperative_jpeg_ctx_t *ctx = jd->device;
    esp_jpeg_image_cfg_t *cfg = ctx->cfg;
    if (cfg->priv.read > cfg->indata_size) {
        return 0;
    }

    const size_t available = cfg->indata_size - cfg->priv.read;
    const size_t count = requested < available ? requested : available;
    if (buffer != NULL && count != 0) {
        memcpy(buffer, cfg->indata + cfg->priv.read, count);
    }
    cfg->priv.read += count;
    return count;
}

static int jpeg_output(JDEC *jd, void *bitmap, JRECT *rect)
{
    cooperative_jpeg_ctx_t *ctx = jd->device;
    esp_jpeg_image_cfg_t *cfg = ctx->cfg;
    const uint8_t *source = bitmap;
    uint16_t *destination = (uint16_t *)cfg->outbuf;
    const uint32_t line_pixels = jd->width / scale_divisor(cfg->out_scale);
    const uint32_t output_width = rect->right - rect->left + 1;

    for (uint32_t y = rect->top; y <= rect->bottom; ++y) {
        uint16_t *row = destination + y * line_pixels + rect->left;
        for (uint32_t x = 0; x < output_width; ++x) {
            const uint16_t pixel = ((source[0] & 0xf8) << 8) |
                                   ((source[1] & 0xfc) << 3) |
                                   (source[2] >> 3);
            row[x] = cfg->flags.swap_color_bytes ?
                         (uint16_t)((pixel << 8) | (pixel >> 8)) : pixel;
            source += 3;
        }
    }

    if (++ctx->output_calls % JPEG_YIELD_EVERY_MCUS == 0) {
        /* This task is subscribed by frame_processing_task().  Resetting it
         * here covers a long but progressing decode; delaying lets IDLE run. */
        (void)esp_task_wdt_reset();
        vTaskDelay(1);
    }
    return 1;
}

esp_err_t cooperative_jpeg_decode(esp_jpeg_image_cfg_t *cfg,
                                  esp_jpeg_image_output_t *img)
{
    if (cfg == NULL || img == NULL || cfg->indata == NULL || cfg->outbuf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->out_format != JPEG_IMAGE_FORMAT_RGB565) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    const uint8_t divisor = scale_divisor(cfg->out_scale);
    if (divisor == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *work = cfg->advanced.working_buffer;
    bool owns_work = false;
    size_t work_size = cfg->advanced.working_buffer_size;
    if (work == NULL) {
        work = heap_caps_malloc(JPEG_WORK_BUF_SIZE, MALLOC_CAP_8BIT);
        if (work == NULL) {
            return ESP_ERR_NO_MEM;
        }
        work_size = JPEG_WORK_BUF_SIZE;
        owns_work = true;
    }

    cfg->priv.read = 0;
    cooperative_jpeg_ctx_t ctx = {.cfg = cfg, .output_calls = 0};
    JDEC decoder;
    JRESULT result = jd_prepare(&decoder, jpeg_input, work, work_size, &ctx);
    if (result == JDR_OK) {
        const size_t output_size = (size_t)(decoder.width / divisor) *
                                   (decoder.height / divisor) * sizeof(uint16_t);
        if (output_size > cfg->outbuf_size) {
            result = JDR_MEM1;
        } else {
            img->width = decoder.width / divisor;
            img->height = decoder.height / divisor;
            img->output_len = output_size;
            result = jd_decomp(&decoder, jpeg_output, cfg->out_scale);
        }
    }

    if (result != JDR_OK) {
        static int64_t last_error_log_us;
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_error_log_us >= 1000000) {
            ESP_LOGW("coop_jpeg", "TinyJPG %s (%d), frame=%u bytes",
                     jpeg_result_name(result), (int)result,
                     (unsigned)cfg->indata_size);
            last_error_log_us = now_us;
        }
    }

    if (owns_work) {
        free(work);
    }
    return result == JDR_OK ? ESP_OK : ESP_FAIL;
}
