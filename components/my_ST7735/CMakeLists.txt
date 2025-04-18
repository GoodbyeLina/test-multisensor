# 组件CMake配置
idf_component_register(
    SRCS "my_ST7735.c"
    INCLUDE_DIRS "." "../ili9340" "../ili9340/decode"
    REQUIRES driver nvs_flash vfs spiffs
)