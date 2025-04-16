#include "sc7a20h.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define TAG "SC7A20H"

// I2C配置
static i2c_port_t g_i2c_port = I2C_NUM_0;
#define SC7A20H_I2C_SDA_IO 21
#define SC7A20H_I2C_SCL_IO 22
#define SC7A20H_I2C_FREQ_HZ 100000
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
        return err;
    }
    return i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
}

// 写入寄存器
static esp_err_t sc7a20h_write_byte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SC7A20H_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// 读取寄存器
static esp_err_t sc7a20h_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SC7A20H_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SC7A20H_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// 初始化SC7A20H
esp_err_t sc7a20h_init(void)
{
    uint8_t who_am_i;
    esp_err_t err = sc7a20h_read_bytes(0x0F, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return err;
    }
    
    if (who_am_i != 0x11) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I value: 0x%02X", who_am_i);
        return ESP_FAIL;
    }

    // 配置加速度计: 100Hz输出数据率, ±4g量程
    err = sc7a20h_write_byte(0x20, 0x57); // CTRL_REG1
    if (err != ESP_OK) return err;
    err = sc7a20h_write_byte(0x23, 0x01); // CTRL_REG4
    return err;
}

// 读取加速度数据
esp_err_t sc7a20h_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    esp_err_t err = sc7a20h_read_bytes(0x28, data, 6);
    if (err != ESP_OK) return err;
    
    *x = (int16_t)((data[1] << 8) | data[0]) >> 4;
    *y = (int16_t)((data[3] << 8) | data[2]) >> 4;
    *z = (int16_t)((data[5] << 8) | data[4]) >> 4;
    return ESP_OK;
}
