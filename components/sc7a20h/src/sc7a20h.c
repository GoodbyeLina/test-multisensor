#include "sc7a20h.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define TAG "SC7A20H"

// I2C配置
static i2c_port_t g_i2c_port = I2C_NUM_0;
#define SC7A20H_I2C_SDA_IO 11
#define SC7A20H_I2C_SCL_IO 12
#define SC7A20H_I2C_FREQ_HZ 100000

// 寄存器地址
#define WHO_AM_I_REG     0x0F
#define CTRL_REG1        0x20
#define CTRL_REG4        0x23
#define OUT_X_L_REG      0x28
#define OUT_X_H_REG      0x29
#define OUT_Y_L_REG      0x2A
#define OUT_Y_H_REG      0x2B
#define OUT_Z_L_REG      0x2C
#define OUT_Z_H_REG      0x2D
// 默认配置
static const sc7a20h_config_t default_config = {
    .i2c_port = I2C_NUM_0,
    .i2c_addr = SC7A20H_ADDR,
    .sda_pin = SC7A20H_I2C_SDA_IO,
    .scl_pin = SC7A20H_I2C_SCL_IO,
    .i2c_freq = SC7A20H_I2C_FREQ_HZ
};
// 扫描I2C设备
static void i2c_scanner(void)
{
    ESP_LOGI(TAG, "开始I2C扫描...");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(default_config.i2c_port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "发现设备 at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C扫描完成");
}

// 初始化I2C接口
esp_err_t sc7a20h_i2c_init(i2c_port_t i2c_num)
{
    g_i2c_port = i2c_num;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SC7A20H_I2C_SDA_IO,
        .scl_io_num = SC7A20H_I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SC7A20H_I2C_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(i2c_num, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: 0x%x", err);
        return err;
    }
    
    err = i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C初始化成功");
        i2c_scanner(); // 扫描I2C总线上的设备
    } else {
        ESP_LOGE(TAG, "I2C驱动安装失败: 0x%x", err);
    }
    return err;
}

// 写入寄存器
static esp_err_t sc7a20h_write_byte(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(default_config.i2c_port,
                                     default_config.i2c_addr,
                                     write_buf, sizeof(write_buf),
                                     pdMS_TO_TICKS(1000));
}

// 读取寄存器
static esp_err_t sc7a20h_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t reg_addr = reg;
    esp_err_t ret = i2c_master_write_read_device(default_config.i2c_port,
                                                default_config.i2c_addr,
                                                &reg_addr, 1,
                                                data, len,
                                                pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

// 初始化SC7A20H
esp_err_t sc7a20h_init(void)
{
    uint8_t who_am_i;
    esp_err_t err = sc7a20h_read_bytes(WHO_AM_I_REG, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取WHO_AM_I寄存器失败: 0x%x", err);
        return err;
    }
    
    if (who_am_i != 0x11) {
        ESP_LOGE(TAG, "无效的WHO_AM_I值: 0x%02X (期望值: 0x11)", who_am_i);
        return ESP_FAIL;
    }

    // 配置加速度计: 100Hz输出数据率, ±4g量程
    err = sc7a20h_write_byte(CTRL_REG1, 0x57); // CTRL_REG1
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写入CTRL_REG1失败: 0x%x", err);
        return err;
    }
    err = sc7a20h_write_byte(CTRL_REG4, 0x01); // CTRL_REG4
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写入CTRL_REG4失败: 0x%x", err);
    }
    return err;
}

/**
 * @brief 读取加速度数据(12位分辨率)
 * @param x X轴加速度输出(单位: mg, 范围: ±2000mg)
 * @param y Y轴加速度输出(单位: mg, 范围: ±2000mg)
 * @param z Z轴加速度输出(单位: mg, 范围: ±2000mg)
 * @return esp_err_t ESP_OK表示成功
 *
 * 该函数一次性读取OUT_X_L_REG(0x28)到OUT_Z_H_REG(0x2D)共6个寄存器，
 * 将高低字节组合成16位数据后右移4位(12位有效数据)，
 * 根据CTRL_REG4配置的±4g量程，1LSB对应1mg
 */
esp_err_t sc7a20h_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    if (x == NULL || y == NULL || z == NULL) {
        ESP_LOGE(TAG, "无效的输出指针");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[6] = {0};
    esp_err_t err_XL = sc7a20h_read_bytes(OUT_X_L_REG, &data[0], 1);
    if (err_XL != ESP_OK) {
        ESP_LOGE(TAG, "读取加速度数据失败: 0x%x", err_XL);
        return err_XL;
    }
    esp_err_t err_XH = sc7a20h_read_bytes(OUT_X_H_REG, &data[1], 1);
    if (err_XH != ESP_OK) {
        ESP_LOGE(TAG, "读取加速度数据失败: 0x%x", err_XH);
        return err_XH;
    }
    esp_err_t err_YL = sc7a20h_read_bytes(OUT_Y_L_REG, &data[2], 1);
    if (err_YL != ESP_OK) {
        ESP_LOGE(TAG, "读取加速度数据失败: 0x%x", err_YL);
        return err_YL;
    }
    esp_err_t err_YH = sc7a20h_read_bytes(OUT_Y_H_REG, &data[3], 1);
    if (err_YH != ESP_OK) {
        ESP_LOGE(TAG, "读取加速度数据失败: 0x%x", err_YH);
        return err_YH;
    }
    esp_err_t err_ZL = sc7a20h_read_bytes(OUT_Z_L_REG, &data[4], 1);
    if (err_ZL != ESP_OK) {
        ESP_LOGE(TAG, "读取加速度数据失败: 0x%x", err_ZL);
        return err_ZL;
    }
    esp_err_t err_ZH = sc7a20h_read_bytes(OUT_Z_H_REG, &data[5], 1);
    if (err_ZH != ESP_OK) {
        ESP_LOGE(TAG, "读取加速度数据失败: 0x%x", err_ZH);
        return err_ZH;
    }

    // 组合高低字节(16位有符号数)
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    // ±4g量程时 1LSB = 1mg
    *x = raw_x;
    *y = raw_y;
    *z = raw_z;
    
    // // 转换为g值(1g = 1000mg)
    // float g_x = raw_x / 1000.0f;
    // float g_y = raw_y / 1000.0f;
    // float g_z = raw_z / 1000.0f;
    
    // ESP_LOGI(TAG, "加速度数据: X=%.2fmg(0x%04X) Y=%.2fmg(0x%04X) Z=%.2fmg(0x%04X)",
    //          (float)raw_x, (uint16_t)raw_x,
    //          (float)raw_y, (uint16_t)raw_y,
    //          (float)raw_z, (uint16_t)raw_z);
    // ESP_LOGI(TAG, "Accel X:%.2fg Y:%.2fg Z:%.2fg", g_x, g_y, g_z);

    return ESP_OK;
}
