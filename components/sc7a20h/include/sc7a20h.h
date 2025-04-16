#ifndef SC7A20H_H
#define SC7A20H_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "sdkconfig.h"

#define SC7A20H_ADDR 0x32
#define SC7A20H_READ_ADDR 0x33

/**
 * @brief 初始化SC7A20H I2C接口
 * @param i2c_num I2C端口号
 * @return esp_err_t ESP_OK表示成功
 */
esp_err_t sc7a20h_i2c_init(i2c_port_t i2c_num);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化SC7A20H加速度计
 * @return esp_err_t ESP_OK表示成功
 */
esp_err_t sc7a20h_init(void);

/**
 * @brief 读取加速度数据
 * @param x X轴加速度输出
 * @param y Y轴加速度输出
 * @param z Z轴加速度输出
 * @return esp_err_t ESP_OK表示成功
 */
esp_err_t sc7a20h_read_accel(int16_t *x, int16_t *y, int16_t *z);

#ifdef __cplusplus
}
#endif

#endif // SC7A20H_H
