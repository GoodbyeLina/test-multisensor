#ifndef HMC5883L_H
#define HMC5883L_H

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 默认I2C配置
#define HMC5883L_DEFAULT_I2C_PORT    I2C_NUM_0
#define HMC5883L_DEFAULT_I2C_FREQ    100000  // 100kHz
#define HMC5883L_DEFAULT_ADDR       0x1E

// 传感器配置结构体
typedef struct {
    i2c_port_t i2c_port;      // I2C端口号
    uint8_t i2c_addr;         // I2C地址
    int sda_pin;              // SDA引脚
    int scl_pin;              // SCL引脚
    uint32_t i2c_freq;        // I2C频率
} hmc5883l_config_t;

// 磁力计数据
typedef struct {
    float x;  // X轴磁场强度(Gauss)
    float y;  // Y轴磁场强度(Gauss) 
    float z;  // Z轴磁场强度(Gauss)
    float heading;  // 航向角(度)
} hmc5883l_data_t;

/**
 * @brief 初始化HMC5883L驱动
 * @param config 配置参数，NULL使用默认配置
 * @return ESP_OK成功，其他失败
 */
esp_err_t hmc5883l_init(const hmc5883l_config_t *config);

/**
 * @brief 读取磁力计数据
 * @param data 存储读取的数据
 * @return ESP_OK成功，其他失败
 */
esp_err_t hmc5883l_read(hmc5883l_data_t *data);

/**
 * @brief 设置传感器增益
 * @param gain 增益值(参见数据手册)
 * @return ESP_OK成功，其他失败
 */
esp_err_t hmc5883l_set_gain(uint8_t gain);

/**
 * @brief 设置测量模式
 * @param mode 模式(0-连续测量, 1-单次测量, 2-空闲)
 * @return ESP_OK成功，其他失败
 */
esp_err_t hmc5883l_set_mode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif // HMC5883L_H