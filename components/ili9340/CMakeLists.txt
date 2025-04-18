set(srcs 
    "ili9340.c" 
    "fontx.c"
)

idf_component_register(SRCS "${srcs}"
                       REQUIRES driver freertos
                       INCLUDE_DIRS "." 
                       )
