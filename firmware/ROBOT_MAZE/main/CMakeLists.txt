idf_component_register(
    SRCS "robot_kinematics.c" "robot_controller.c" "robot_sensor.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_driver_i2c 
        esp_adc 
        driver 
        PCA9685
        MPU6050
)
