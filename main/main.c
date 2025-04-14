#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hc_sr04.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"
#include "unity.h"

#define I2C_MASTER_SCL_IO 39      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 38      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define CALIBRATION_SAMPLES 100


static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;


/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/**
 * @brief mpu6050 gyro calibration
 */
void calibrate_gyro(mpu6050_handle_t mpu, float *offsets) {
   mpu6050_gyro_value_t gyro;
   float sum_x = 0, sum_y = 0, sum_z = 0;
   
   for(int i=0; i<CALIBRATION_SAMPLES; i++) {
       mpu6050_get_gyro(mpu, &gyro);
       sum_x += gyro.gyro_x;
       sum_y += gyro.gyro_y; 
       sum_z += gyro.gyro_z;
       vTaskDelay(50 / portTICK_PERIOD_MS);
   }
   
   offsets[0] = sum_x / CALIBRATION_SAMPLES;
   offsets[1] = sum_y / CALIBRATION_SAMPLES;
   offsets[2] = sum_z / CALIBRATION_SAMPLES;
}
void app_main(void)
{
    // 初始化HC-SR04
    hc_sr04_config_t config = {
        .trig_pin = GPIO_NUM_15,
        .echo_pin = GPIO_NUM_16
    };

    esp_err_t ret_mpu6050;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    esp_err_t ret_hcsr04 = hc_sr04_init(&config);
    if (ret_hcsr04 != ESP_OK) {
        printf("HC-SR04 init failed: %d\n", ret_hcsr04);
        return;
    }

    i2c_sensor_mpu6050_init();

    while(1) {
        float distance = hc_sr04_measure_distance();
        if (distance >= 0) {
            printf("Distance: %.2f cm\n", distance);
        } else {
            printf("Measurement failed\n");
        }
        ret_mpu6050 = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
        TEST_ASSERT_EQUAL(ESP_OK, ret_mpu6050);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");
    
        ret_mpu6050 = mpu6050_get_acce(mpu6050, &acce);
        TEST_ASSERT_EQUAL(ESP_OK, ret_mpu6050);
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
    
        ret_mpu6050 = mpu6050_get_gyro(mpu6050, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret_mpu6050);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
    
        ret_mpu6050 = mpu6050_get_temp(mpu6050, &temp);
        TEST_ASSERT_EQUAL(ESP_OK, ret_mpu6050);
        ESP_LOGI(TAG, "t:%.2f \n", temp.temp);
    
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    mpu6050_delete(mpu6050);
    ret_mpu6050 = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret_mpu6050);

}