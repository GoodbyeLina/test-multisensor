#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sc7a20h.h"

#define TAG "SC7A20H"

void app_main(void)
{
    // 初始化I2C和SC7A20H
    ESP_ERROR_CHECK(sc7a20h_i2c_init(I2C_NUM_0));
    
    // 初始化带重试
    for (int i = 0; i < 3; i++) {
        esp_err_t err = sc7a20h_init();
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "Init attempt %d failed", i+1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    int16_t x, y, z;
    while (1) {
        if (sc7a20h_read_accel(&x, &y, &z) == ESP_OK) {
            ESP_LOGI(TAG, "Accel X:%.2fg Y:%.2fg Z:%.2fg",
                    x * 0.004f, y * 0.004f, z * 0.004f);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
