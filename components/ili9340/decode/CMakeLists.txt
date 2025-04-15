set(COMPONENT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(srcs
    "${COMPONENT_DIR}/decode_png.c"
    "${COMPONENT_DIR}/decode_jpeg_v5.c"
    "${COMPONENT_DIR}/pngle.c"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "${COMPONENT_DIR}"  # 头文件搜索路径
    REQUIRES hc_sr04                 # 依赖 hc_sr04（超声波驱动）
    # PRIV_REQUIRES ...             # 可选的私有依赖
)