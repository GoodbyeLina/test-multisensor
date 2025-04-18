
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        cam_ov2640
        esp32-camera
        esp_jpeg
        hc_sr04
        HMC5883L
        ili9340
        decode
        my_mpu6050
        my_ST7735
        sc7a20h
        unity
        vfs
        spiffs
        nvs_flash

)
