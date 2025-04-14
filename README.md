# ESP32 多传感器项目

本项目演示如何使用ESP32与HC-SR04超声波传感器和MPU6050加速度计/陀螺仪。

## 功能特性
- 使用HC-SR04进行距离测量
- 使用MPU6050进行运动检测
- MPU6050的I2C通信
- HC-SR04的GPIO接口

## 硬件需求
- ESP32开发板
- HC-SR04超声波传感器
- MPU6050六轴运动传感器
- 面包板和跳线

## 硬件连接与配置
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
3. 构建并烧录：
   ```bash
   idf.py build flash monitor
   ```

## 项目结构
```
├── components
│   └── hc_sr04
│       ├── include
│       │   └── hc_sr04.h
│       └── src
│           └── hc_sr04.c
├── main
│   ├── CMakeLists.txt
│   └── main.c
├── managed_components
│   └── espressif__mpu6050
│       ├── include
│       │   └── mpu6050.h
│       └── src
│           └── mpu6050.c
└── README.md
```

## 示例输出
```
距离: 25.34 cm
陀螺仪 X: 0.12, Y: -0.05, Z: 0.03
```

## 许可证
MIT 许可证
