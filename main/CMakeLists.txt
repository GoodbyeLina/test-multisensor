
idf_component_register(SRCS "main.c" 
                    REQUIRES 
                        hc_sr04 
                        ili9340
                        sc7a20h
                        HMC5883L
                        cam_ov2640
                        unity vfs spiffs nvs_flash 
                        
                    INCLUDE_DIRS ".")
