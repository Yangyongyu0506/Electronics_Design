#include "esp_err.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cooperative_jpeg.h"

/*
 * esp_jpeg deliberately exposes only jpeg_decoder.h.  TJpgDec's tjpgd.h is
 * an implementation detail of the managed component, so do not include it
 * from this application.  The public decoder call is blocking; placing WDT
 * service and a scheduler yield on both sides keeps this frame task friendly
 * to FreeRTOS without depending on private component headers.
 */
esp_err_t cooperative_jpeg_decode(esp_jpeg_image_cfg_t *cfg,
                                  esp_jpeg_image_output_t *img)
{
    if (cfg == NULL || img == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)esp_task_wdt_reset();
    vTaskDelay(1);
    const esp_err_t result = esp_jpeg_decode(cfg, img);
    (void)esp_task_wdt_reset();
    vTaskDelay(1);
    return result;
}
