#include <stdio.h>
#include "driver/i2c.h"
#include "esp/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

void app_main(void)
{
    // Include necessary headers (already added above)

    // Initialize I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_19,
        .scl_io_num = GPIO_NUM_18,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // Initialize MPU6050
    mpu6050_init(I2C_NUM_0, MPU6050_ADDR);

    while (1) {
        int16_t ax, ay, az;
        mpu6050_get_acceleration(&ax, &ay, &az);
        printf("Accel: X=%d, Y=%d, Z=%d\n", ax, ay, az);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
