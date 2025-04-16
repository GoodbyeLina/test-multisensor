#pragma once
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

// OV2640引脚配置结构体
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

/**
 * @brief 初始化OV2640相机
 * @param config 相机配置参数
 * @return esp_err_t 初始化结果
 */
esp_err_t init_camera(void);

#ifdef __cplusplus
}
#endif