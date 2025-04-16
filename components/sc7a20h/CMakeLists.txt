# 组件CMake配置
idf_component_register(
    SRCS "src/sc7a20h.c"
    INCLUDE_DIRS "include"
    REQUIRES driver 
)
