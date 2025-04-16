
#include "cam_ov2640.h"


#define OV_2640_I2C_ADDR 0x3C
#define OV_2640_I2C_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0
#define OV_2640_I2C_SDA_PIN 21
#define OV_2640_I2C_SCL_PIN 22

#define OV_2640_VSYNC_PIN 4
#define OV_2640_HREF_PIN 35
#define OV_2640_PCLK_PIN 39
#define OV_2640_XCLK_PIN 32
#define OV_2640_RST_PIN 5
#define OV_2640_PWDN_PIN 34
#define OV_2640_D0_PIN 36
#define OV_2640_D1_PIN 37
#define OV_2640_D2_PIN 38
#define OV_2640_D3_PIN 35
#define OV_2640_D4_PIN 41
#define OV_2640_D5_PIN 42
#define OV_2640_D6_PIN 13
#define OV_2640_D7_PIN 12

static const char *TAG = "example:take_picture";



void app_main(void)
{
    #if ESP_CAMERA_SUPPORTED
    if(ESP_OK != init_camera()) {
        return;
    }

    while (1)
    {
        ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *pic = esp_camera_fb_get();

        // use pic->buf to access the image
        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        esp_camera_fb_return(pic);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    return;
#endif

}