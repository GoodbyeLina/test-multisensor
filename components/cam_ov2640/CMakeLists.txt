idf_component_register(
    SRCS "cam_ov2640.c"
    INCLUDE_DIRS "."
    REQUIRES esp32-camera nvs_flash esp_psram driver
)