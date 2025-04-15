set(srcs 
    "ili9340.c" 
    "fontx.c"
    "decode/decode_jpeg.c"
    "decode/decode_png.c"
    "decode/pngle.c"
)

idf_component_register(SRCS "${srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "." "decode")
