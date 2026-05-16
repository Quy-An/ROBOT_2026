#pragma once

#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#include "vl53l0x_platform.h"  // ST: defines VL53L0X_Dev_t / VL53L0X_DEV

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Register an I2C device (7-bit address) with ESP-IDF new I2C master driver.
 * You must call this once before using any ST VL53L0X API calls.
 */
esp_err_t vl53l0x_espidf_i2c_register(i2c_master_bus_handle_t bus,
                                     uint8_t i2c_addr_7bit,
                                     uint32_t scl_speed_hz);

/**
 * Unregister a previously-registered device.
 */
esp_err_t vl53l0x_espidf_i2c_unregister(uint8_t i2c_addr_7bit);

/**
 * Initialize an ST device struct to use with the ST VL53L0X API.
 * Note: i2c_addr_7bit is the 7-bit address (default 0x29).
 */
static inline void vl53l0x_espidf_init_st_device(VL53L0X_Dev_t *dev,
                                                uint8_t i2c_addr_7bit,
                                                uint16_t comms_speed_khz)
{
    if (!dev) return;
    memset(dev, 0, sizeof(*dev));
    dev->I2cDevAddr = i2c_addr_7bit;
    dev->comms_type = I2C;
    dev->comms_speed_khz = comms_speed_khz;
}

/**
 * Convenience: returns pointer type expected by ST API.
 */
static inline VL53L0X_DEV vl53l0x_espidf_as_st_dev(VL53L0X_Dev_t *dev)
{
    return (VL53L0X_DEV)dev;
}

#ifdef __cplusplus
}
#endif
