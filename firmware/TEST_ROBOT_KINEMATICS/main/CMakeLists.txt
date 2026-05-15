set(srcs 
    "main.c" 
    "robot_controller.c" 
    "sensor_mpu6050.c" 
    "pid_control.c"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES 
        esp_driver_i2c 
        esp_adc 
        driver 
        PCA9685
        MPU6050
)
