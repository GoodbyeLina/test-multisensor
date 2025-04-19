#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "unity.h"

#include <inttypes.h>
#include <math.h>

#include "nvs_flash.h"
#include "nvs.h"

#include "hc_sr04.h"
#include "sc7a20h.h"
#include "hmc5883l.h"
#include "my_mpu6050.h"
#include "my_ST7735.h"
#include "cam_ov2640.h"

#include "driver/gpio.h"

#define INTERVAL 400
#define WAIT vTaskDelay(INTERVAL)

#define CALIBRATION_SAMPLES 100

static const char *TAG = "main";
static mpu6050_handle_t my_mpu6050 = NULL;

#define HC_SR04 0
#define MPU6050 0
#define ST7735  0
#define SC7A20H 0
#define HCM5883L 1
#define ov2640 0


void app_main(void)
{
#if HC_SR04


    // 初始化HC-SR04、MPU6050
    hc_sr04_config_t config = {
        .trig_pin = GPIO_NUM_15,
        .echo_pin = GPIO_NUM_16
    };
    esp_err_t ret_hcsr04 = hc_sr04_init(&config);

	if (ret_hcsr04 != ESP_OK) {
        printf("HC-SR04 init failed: %d\n", ret_hcsr04);
        return;
    }

	while (1)
	{
        float distance = hc_sr04_measure_distance();
        if (distance >= 0) {
            printf("Distance: %.2f cm\n", distance);
        } else {
            printf("Measurement failed\n");
        }
	}
#endif

#if MPU6050

	esp_err_t ret;
	uint8_t mpu6050_deviceid;
	mpu6050_acce_value_t acce;
	mpu6050_gyro_value_t gyro;
	mpu6050_temp_value_t temp;

	i2c_sensor_mpu6050_init();

	ret = mpu6050_get_deviceid(my_mpu6050, &mpu6050_deviceid);
	TEST_ASSERT_EQUAL(ESP_OK, ret);
	TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

	ret = mpu6050_get_acce(my_mpu6050, &acce);
	TEST_ASSERT_EQUAL(ESP_OK, ret);
	ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

	ret = mpu6050_get_gyro(my_mpu6050, &gyro);
	TEST_ASSERT_EQUAL(ESP_OK, ret);
	ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

	ret = mpu6050_get_temp(my_mpu6050, &temp);
	TEST_ASSERT_EQUAL(ESP_OK, ret);
	ESP_LOGI(TAG, "t:%.2f \n", temp.temp);

	mpu6050_delete(my_mpu6050);
	ret = i2c_driver_delete(I2C_MASTER_NUM);
	TEST_ASSERT_EQUAL(ESP_OK, ret);

#endif


#if ST7735
    // Initialize NVS
	// NVS saves the touch position calibration.
	ESP_LOGI(TAG, "Initialize NVS");
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );

	ESP_LOGI(TAG, "Initializing SPIFFS");
	// Maximum files that could be open at the same time is 10.
	ESP_ERROR_CHECK(mountSPIFFS("/fonts", "storage0", 10));
	listSPIFFS("/fonts/");

	// Image file borrowed from here
	// https://www.flaticon.com/packs/social-media-343
	// Maximum files that could be open at the same time is 1.
	ESP_ERROR_CHECK(mountSPIFFS("/icons", "storage1", 1));
	listSPIFFS("/icons/");

	// Maximum files that could be open at the same time is 1.
	ESP_ERROR_CHECK(mountSPIFFS("/images", "storage2", 1));
	listSPIFFS("/images/");

	xTaskCreate(TFT, "TFT", 1024*6, NULL, 2, NULL);

#endif

#if SC7A20H
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

#endif

#if HCM5883L
    // 初始化HMC5883L(使用自定义配置)
    hmc5883l_config_t hmc_config = {
        .i2c_port = HMC5883L_DEFAULT_I2C_PORT,
        .i2c_addr = HMC5883L_DEFAULT_ADDR,
        .sda_pin = 11,  // 根据实际硬件连接修改
        .scl_pin = 12,  // 根据实际硬件连接修改
        .i2c_freq = HMC5883L_DEFAULT_I2C_FREQ
    };

    // 带重试的初始化
    esp_err_t hmc_init_ret = ESP_FAIL;
    for (int i = 0; i < 3; i++) {
        hmc_init_ret = hmc5883l_init(&hmc_config);
        if (hmc_init_ret == ESP_OK) {
            ESP_LOGI(TAG, "HMC5883L initialized successfully (attempt %d)", i+1);
            break;
        }
        ESP_LOGE(TAG, "HMC5883L init attempt %d failed: %s",
                i+1, esp_err_to_name(hmc_init_ret));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (hmc_init_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HMC5883L after 3 attempts");
        return;
    }

    hmc5883l_data_t data;
    
    while (1) {
        if (hmc5883l_read(&data) == ESP_OK) {
            ESP_LOGI(TAG, "X:%.2fG Y:%.2fG Z:%.2fG Heading:%.1f°",
                    data.x, data.y, data.z, data.heading);
        } else {
            ESP_LOGE(TAG, "Failed to read data");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }

#endif

#if ov2640
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

			vTaskDelay(2000 / portTICK_PERIOD_MS);
		}
	#else
		ESP_LOGE(TAG, "Camera support is not available for this chip");
		return;
	#endif
#endif

}