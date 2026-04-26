idf_component_register(
    SRCS
        "src/vl53l0x_platform_espidf.c"
        "src/vl53l0x_i2c_platform_espidf.c"
        "src/vl53l0x_espidf.c"
        "st_api/core/src/vl53l0x_api.c"
        "st_api/core/src/vl53l0x_api_core.c"
        "st_api/core/src/vl53l0x_api_calibration.c"
        "st_api/core/src/vl53l0x_api_strings.c"
        "st_api/core/src/vl53l0x_api_ranging.c"
    INCLUDE_DIRS
        "include"
        "st_api/core/inc"
        "st_api/platform/inc"
    REQUIRES
        driver
        freertos
)

# Optional: if you enable VL53L0X_LOG_ENABLE, provide your own log implementation.
# target_compile_definitions(${COMPONENT_LIB} PRIVATE VL53L0X_LOG_ENABLE=1)
