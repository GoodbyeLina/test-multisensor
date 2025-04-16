#include "cam_ov2640.h"
#include "esp_log.h"
#include "esp_camera.h"

static const char *TAG = "ov2640-driver";

static camera_config_t camera_config = {
    
    // 基本引脚配置
    .pin_pwdn = OV_2640_PWDN_PIN,
    .pin_reset = OV_2640_RST_PIN,
    .pin_xclk = OV_2640_XCLK_PIN,
    .pin_sccb_sda = OV_2640_I2C_SDA_PIN,
    .pin_sccb_scl = OV_2640_I2C_SCL_PIN,
    
    // 数据引脚配置
    .pin_d7 = OV_2640_D7_PIN,
    .pin_d6 = OV_2640_D6_PIN,
    .pin_d5 = OV_2640_D5_PIN,
    .pin_d4 = OV_2640_D4_PIN,
    .pin_d3 = OV_2640_D3_PIN,
    .pin_d2 = OV_2640_D2_PIN,
    .pin_d1 = OV_2640_D1_PIN,
    .pin_d0 = OV_2640_D0_PIN,

    // 控制信号引脚
    .pin_vsync = OV_2640_VSYNC_PIN,
    .pin_href = OV_2640_HREF_PIN,
    .pin_pclk = OV_2640_PCLK_PIN,
    
    // XCLK配置
    .xclk_freq_hz = 20000000, // 20MHz
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    
    // 图像格式和分辨率（根据PSRAM调整）
    .pixel_format = PIXFORMAT_JPEG, // 必须使用JPEG格式
    .frame_size = FRAMESIZE_UXGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY//CAMERA_GRAB_LATEST. Sets when buffers should be filled

};

esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

