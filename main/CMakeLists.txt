
idf_component_register(SRCS "main.c" 
                    REQUIRES hc_sr04 unity vfs spiffs nvs_flash ili9340
                    INCLUDE_DIRS ".")
