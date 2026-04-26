#include "vl53l0x_espidf.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "vl53l0x";

// Implementation is in vl53l0x_i2c_platform_espidf.c
esp_err_t vl53l0x_espidf_i2c_register(i2c_master_bus_handle_t bus,
                                     uint8_t i2c_addr_7bit,
                                     uint32_t scl_speed_hz);

esp_err_t vl53l0x_espidf_i2c_unregister(uint8_t i2c_addr_7bit);

// Just keep this TU to host any future convenience helpers.
// For now, the public API is declared in vl53l0x_espidf.h
