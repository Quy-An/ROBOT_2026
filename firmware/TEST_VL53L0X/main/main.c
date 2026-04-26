#include <stdio.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
/**
 * @brief user component include
 */
#include "vl53l0x_espidf.h"
#include "vl53l0x_api.h"

/**
 * @brief Tag for logging
 */
static const char *TAG = "Main_App";

/**
 * @brief I2C master bus handle
 * This handle is used for all device communications on the I2C bus.
 * It is initialized in the i2c_bus_init() function and used in the i2c_scanner() function to probe for devices on the bus.
 */
i2c_master_bus_handle_t bus_handle;

/**
 * @brief Initialize the I2C master bus
 * This function sets up the I2C master bus 
 * with the specified configuration parameters defined in config.h. 
 * It creates a new I2C master bus handle that 
 * will be used for all subsequent I2C communications.
 */
static void i2c_bus_init(void){
    ESP_LOGI(TAG, "Initializing I2C Master Bus...");
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,   
        .i2c_port = I2C_MASTER_PORT,                      
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,              
        .flags.enable_internal_pullup = true,
    };

    // Check if initializing bus handle correctly
    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C Master Bus initialized successfully.");
    } else {
        ESP_LOGE(TAG, "I2C Master Bus initialization failed: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Scan I2C bus for devices to debug NACK issues
 * This function iterates through all possible I2C addresses (1 to 126) 
 * and uses the i2c_master_probe() function to check 
 * if a device responds at each address.
 */
static void i2c_scanner(void) {
    ESP_LOGW(TAG, "Scanning I2C bus...");
    int devices_found = 0;
    for (uint8_t i = 1; i < 127; i++) {
        esp_err_t ret = i2c_master_probe(bus_handle, i, 100);
        if (ret == ESP_OK) {
            ESP_LOGW(TAG, "=> FOUND DEVICE AT ADDRESS: 0x%02X", i);
            devices_found++;
        }
    }
    if (devices_found == 0) {
        ESP_LOGE(TAG, "=> NO I2C DEVICES FOUND! Check wiring (SDA/SCL swapped?), power, or pull-up resistors.");
    }
    ESP_LOGW(TAG, "Scan completed.");
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{

}
