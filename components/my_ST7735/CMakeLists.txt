# 组件CMake配置
idf_component_register(
    SRCS "my_ST7735.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash vfs spiffs decode ili9340
)