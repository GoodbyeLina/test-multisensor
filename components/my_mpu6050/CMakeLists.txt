# 重点：组件名仍是mpu6050（和目录名一致）
set(COMPONENT_NAME my_mpu6050)

# 组件CMake配置
idf_component_register(
    SRCS "my_mpu6050.c"
    INCLUDE_DIRS "."
    REQUIRES driver mpu6050 unity
)