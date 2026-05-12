set(srcs 
    "src/driver_pca9685.c"
    "src/driver_pca9685_interface.c"
    "src/driver_pca9685_basic.c"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES "driver" "esp_timer"
)