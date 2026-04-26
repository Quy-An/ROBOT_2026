set(srcs 
    "src/driver_mpu6050.c"
    "src/driver_mpu6050_dmp.c"
    "src/driver_mpu6050_interface.c"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES "driver" "esp_timer"
)