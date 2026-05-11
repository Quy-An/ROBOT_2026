#include <stdio.h>
#include <stdbool.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
/**
 * @brief user include header file
 */
#include "driver_pca9685_basic.h"
#include "robot_kinematics.h"
#include "robot_controller.h"

/**
 * @brief  TAG for logging
 */
static const char *TAG = "Main_App";

/**
 * @brief PCA9685 handle and I2C bus handle;
 */
i2c_master_bus_handle_t i2c_bus_handle = NULL;

/**
 * @brief Initialize the I2C master bus
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
    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C Master Bus initialized successfully.");
    } else {
        ESP_LOGE(TAG, "I2C Master Bus initialization failed: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Scan I2C bus for devices to debug NACK issues
 */
static void i2c_scanner(void) {
    ESP_LOGW(TAG, "Scanning I2C bus...");
    int devices_found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, 100);
        if (ret == ESP_OK) {
            ESP_LOGW(TAG, "=> FOUND DEVICE AT ADDRESS: 0x%02X", addr);
            devices_found++;
        }
    }
    if (devices_found == 0) {
        ESP_LOGE(TAG, "=> NO I2C DEVICES FOUND! Check wiring (SDA/SCL swapped?), power, or pull-up resistors.");
    }
    ESP_LOGW(TAG, "Scan completed.");
}


void app_main(void)
{
    ESP_LOGI(TAG, "PCA9685 basic demo starting...");

    /* 1) Initialize the shared I2C bus (ESP-IDF v5.x i2c_master) */
    i2c_bus_init();
    if (i2c_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C init failed, cannot continue.");
        return;
    }
    
    // // Scan for devices before initializing drivers to see hardware reality
    // i2c_scanner();

    /* 2) Initialize robot kinematics */
    kinematics_init();

    /* 3) Start robot control task */
    robot_controller_task_start();

    // Example sequence
    while (1) {
        // // Move forward
        // ESP_LOGI(TAG, "Moving forward");
        // robot_controller_set_velocity(0.8f, 0.0f);
        // vTaskDelay(pdMS_TO_TICKS(2000));

        // Stop
        ESP_LOGI(TAG, "Stopping");
        robot_controller_set_velocity(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // // Spin
        // ESP_LOGI(TAG, "Spinning");
        // robot_controller_set_velocity(0.0f, 10.0f);
        // vTaskDelay(pdMS_TO_TICKS(2000));
    }
}