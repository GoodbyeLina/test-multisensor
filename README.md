# ESP32 多传感器项目

本项目演示如何使用ESP32与多种传感器和显示模块交互，包括：
- HC-SR04超声波传感器
- MPU6050加速度计/陀螺仪
- ILI9340 LCD显示屏
- OV2640摄像头模块
- HMC5883L磁力计
- SC7A20H加速度计
- ST7735 LCD显示屏

## 功能特性
- 使用HC-SR04进行距离测量
- 使用MPU6050进行运动检测
- 使用OV2640摄像头捕获图像
- 使用HMC5883L进行磁场检测
- 使用SC7A20H进行加速度测量
- 使用ST7735 LCD显示传感器数据和图片
- MPU6050的I2C通信
- HC-SR04的GPIO接口
- SPI接口驱动ILI9340/ST7735显示屏

## 未来计划
- 实现多传感器数据融合算法
- 添加摄像头图像处理功能

## 硬件需求
- ESP32开发板
- HC-SR04超声波传感器
- MPU6050六轴运动传感器
- OV2640摄像头模块
- HMC5883L磁力计
- SC7A20H加速度计
- ST7735 SPI LCD显示屏
- 面包板和跳线

## 硬件连接与配置
### ILI9340 配置
配置文件: `components/ili9340/ili9340.h`

```c
#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22
#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5

spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096
};

spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 40*1000*1000,
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 7,
    .pre_cb = NULL,
    .post_cb = NULL,
};
```

硬件连接：
- VCC: 3.3V
- GND: GND
- CS: GPIO22
- RESET: GPIO18
- DC: GPIO21
- MOSI: GPIO23
- SCK: GPIO19
- LED: GPIO5 (背光控制)
- MISO: GPIO25

### HC-SR04 配置
配置文件: `main/main.c` 和 `components/hc_sr04/include/hc_sr04.h`

```c
hc_sr04_config_t config = {
    .trig_pin = GPIO_NUM_15,  // 触发引脚
    .echo_pin = GPIO_NUM_16   // 回波引脚
};
esp_err_t ret = hc_sr04_init(&config);
```

硬件连接：
- VCC: 5V
- GND: GND
- TRIG: GPIO15
- ECHO: GPIO16

### MPU6050 配置
配置文件: `main/main.c` 和 `managed_components/espressif__mpu6050/include/mpu6050.h`

```c
#define I2C_MASTER_SCL_IO 39  // I2C时钟线
#define I2C_MASTER_SDA_IO 38  // I2C数据线
#define I2C_MASTER_NUM I2C_NUM_0  // I2C端口号
#define I2C_MASTER_FREQ_HZ 100000 // I2C时钟频率

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ
};
i2c_param_config(I2C_MASTER_NUM, &conf);
i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
```

硬件连接：
- VCC: 3.3V
- GND: GND
- SCL: GPIO39
- SDA: GPIO38

### OV2640 配置
配置文件: `components/cam_ov2640/include/cam_ov2640.h`

```c
#define CAM_PIN_PWDN    -1  // 电源控制引脚(未使用)
#define CAM_PIN_RESET   -1  // 硬件复位引脚(未使用)
#define CAM_PIN_XCLK    4   // XCLK信号引脚
#define CAM_PIN_SIOD    18  // I2C SDA
#define CAM_PIN_SIOC    23  // I2C SCL

#define CAM_PIN_D7      36  // 数据引脚7
#define CAM_PIN_D6      37  // 数据引脚6
#define CAM_PIN_D5      38  // 数据引脚5
#define CAM_PIN_D4      39  // 数据引脚4
#define CAM_PIN_D3      35  // 数据引脚3
#define CAM_PIN_D2      14  // 数据引脚2
#define CAM_PIN_D1      13  // 数据引脚1
#define CAM_PIN_D0      34  // 数据引脚0
#define CAM_PIN_VSYNC   5   // VSYNC信号
#define CAM_PIN_HREF    27  // HREF信号
#define CAM_PIN_PCLK    25  // PCLK信号
```

硬件连接：
- 3.3V: 3.3V电源
- GND: 地线
- SIOD: GPIO18 (I2C SDA)
- SIOC: GPIO23 (I2C SCL)
- VSYNC: GPIO5
- HREF: GPIO27
- PCLK: GPIO25
- XCLK: GPIO4
- D0-D7: GPIO34,13,14,35,39,38,37,36

### HMC5883L 配置
配置文件: `components/HMC5883L/include/HMC5883L.h`

```c
#define I2C_MASTER_SCL_IO 39  // I2C时钟线
#define I2C_MASTER_SDA_IO 38  // I2C数据线
#define I2C_MASTER_NUM I2C_NUM_0  // I2C端口号
#define I2C_MASTER_FREQ_HZ 100000 // I2C时钟频率

hmc5883l_config_t config = {
    .i2c_port = I2C_MASTER_NUM,
    .i2c_addr = HMC5883L_ADDR,
    .sda_pin = I2C_MASTER_SDA_IO,
    .scl_pin = I2C_MASTER_SCL_IO,
    .mode = HMC5883L_MODE_CONTINUOUS,
    .range = HMC5883L_RANGE_1_3GA,
    .data_rate = HMC5883L_DATARATE_15HZ
};
```

硬件连接：
- VCC: 3.3V
- GND: GND
- SCL: GPIO39
- SDA: GPIO38

### SC7A20H 配置
配置文件: `components/sc7a20h/include/sc7a20h.h`

```c
#define I2C_MASTER_SCL_IO 39  // I2C时钟线
#define I2C_MASTER_SDA_IO 38  // I2C数据线
#define I2C_MASTER_NUM I2C_NUM_0  // I2C端口号

sc7a20h_config_t config = {
    .i2c_port = I2C_MASTER_NUM,
    .i2c_addr = SC7A20H_ADDR,
    .sda_pin = I2C_MASTER_SDA_IO,
    .scl_pin = I2C_MASTER_SCL_IO,
    .range = SC7A20H_RANGE_2G,
    .data_rate = SC7A20H_DATARATE_100HZ
};
```

硬件连接：
- VCC: 3.3V
- GND: GND
- SCL: GPIO39
- SDA: GPIO38

### ST7735 配置
配置文件: `components/ST7735_TEST/include/ST7735.h`

```c
#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22
#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5

spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096
};

spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 40*1000*1000,
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 7,
    .pre_cb = NULL,
    .post_cb = NULL,
};
```

硬件连接：
- VCC: 3.3V
- GND: GND
- CS: GPIO22
- RESET: GPIO18
- DC: GPIO21
- MOSI: GPIO23
- SCK: GPIO19
- LED: GPIO5 (背光控制)
- MISO: GPIO25

## 软件需求
- ESP-IDF v5.0+
- CMake 3.5+
- Python 3.7+

## 构建与烧录
1. 克隆本仓库：
   ```bash
   git clone --recursive https://github.com/GoodbyeLina/test-multisensor.git
   ```
2. 设置ESP-IDF环境
3. 配置显示屏参数：
   ```bash
   idf.py menuconfig
   ```
   在配置界面中：
   - 进入"Component config" → "ILI9340 Configuration"
   - 设置SPI总线号(默认为SPI2_HOST)
   - 确认或修改SPI引脚配置(与硬件连接一致)
   - 设置显示方向(0-3对应不同旋转角度)
   - 设置背光控制引脚(默认为GPIO5)
4. 保存配置并退出
5. 构建并烧录：
   ```bash
   idf.py build flash monitor
   ```

## 项目结构
```
├── components
│   ├── cam_ov2640
│   │   ├── include
│   │   │   └── cam_ov2640.h
│   │   └── src
│   │       └── cam_ov2640.c
│   ├── hc_sr04
│   │   ├── include
│   │   │   └── hc_sr04.h
│   │   └── src
│   │       └── hc_sr04.c
│   ├── HMC5883L
│   │   ├── include
│   │   │   └── HMC5883L.h
│   │   └── src
│   │       └── HMC5883L.c
│   ├── ili9340
│   │   ├── include
│   │   │   └── ili9340.h
│   │   └── src
│   │       └── ili9340.c
│   ├── sc7a20h
│   │   ├── include
│   │   │   └── sc7a20h.h
│   │   └── src
│   │       └── sc7a20h.c
│   └── ST7735_TEST
│       ├── include
│       │   └── ST7735.h
│       └── src
│           └── ST7735.c
├── main
│   ├── CMakeLists.txt
│   └── main.c
├── managed_components
│   ├── espressif__esp_jpeg
│   │   ├── include
│   │   │   └── jpeg_decoder.h
│   │   └── src
│   │       └── jpeg_decoder.c
│   └── espressif__mpu6050
│       ├── include
│       │   └── mpu6050.h
│       └── src
│           └── mpu6050.c
├── font
│   └── (各种字体文件)
├── icons
│   └── (各种图标文件)
├── images
│   └── (各种图片文件)
└── README.md
```

## 示例输出
```
距离: 25.34 cm
陀螺仪 X: 0.12, Y: -0.05, Z: 0.03
加速度计 X: 0.98, Y: 0.02, Z: 0.01
磁场强度 X: 12.3, Y: 5.6, Z: -8.7 μT
摄像头: 图像捕获成功
LCD显示: 传感器数据和图片
```

## 许可证
MIT 许可证

## 更新日志
### 2025-04-18
- 更新摄像头驱动(components/cam_ov2640)
  - 优化图像捕获性能
  - 添加自动曝光控制
- 更新超声波传感器驱动(components/hc_sr04)
  - 提高测量精度
  - 添加温度补偿功能
- 更新主程序(main/main.c)
  - 添加多传感器数据融合算法
  - 优化显示刷新率
- 更新构建配置(main/CMakeLists.txt)
  - 添加对新组件的支持
- 新增ST7735 LCD组件(components/my_ST7735)
  - 支持160x128分辨率
  - 添加图形绘制API
- 新增MPU6050组件(components/my_mpu6050)
  - 优化I2C通信
  - 添加运动检测功能
- 更新VSCode开发环境配置(.vscode/settings.json)
