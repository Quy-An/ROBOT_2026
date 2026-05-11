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


/**
 * @brief Control the speed of the left and right motors
 * @param left_speed Speed of left motor (-100.0 to 100.0). Positive is forward, negative is backward.
 * @param right_speed Speed of right motor (-100.0 to 100.0). Positive is forward, negative is backward.
 */
static void motor_control(float left_speed, float right_speed) {
    if (left_speed > 0) {
        pca9685_basic_write(PWM_RPWM_LEFT, 0.0f, left_speed);
        pca9685_basic_write(PWM_LPWM_LEFT, 0.0f, 0.0f);
    } else if (left_speed < 0) {
        pca9685_basic_write(PWM_RPWM_LEFT, 0.0f, 0.0f);
        pca9685_basic_write(PWM_LPWM_LEFT, 0.0f, -left_speed);
    } else {
        pca9685_basic_write(PWM_RPWM_LEFT, 0.0f, 0.0f);
        pca9685_basic_write(PWM_LPWM_LEFT, 0.0f, 0.0f);
    }

    if (right_speed > 0) {
        pca9685_basic_write(PWM_RPWM_RIGHT, 0.0f, right_speed);
        pca9685_basic_write(PWM_LPWM_RIGHT, 0.0f, 0.0f);
    } else if (right_speed < 0) {
        pca9685_basic_write(PWM_RPWM_RIGHT, 0.0f, 0.0f);
        pca9685_basic_write(PWM_LPWM_RIGHT, 0.0f, -right_speed);
    } else {
        pca9685_basic_write(PWM_RPWM_RIGHT, 0.0f, 0.0f);
        pca9685_basic_write(PWM_LPWM_RIGHT, 0.0f, 0.0f);
    }
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
    
    // Scan for devices before initializing drivers to see hardware reality
    i2c_scanner();

    /* 2) Initialize the PCA9685 driver */
    ESP_LOGI(TAG, "Initializing PCA9685...");
    uint8_t res = pca9685_basic_init(PCA9685_ADDRESS_A000000, 1000); // 1000Hz PWM
    if (res != 0) {
        ESP_LOGE(TAG, "PCA9685 initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "PCA9685 initialized successfully.");

    /* 3) Motor control sequence */
    while (1) {
        // 1. Forward (Tiến)
        ESP_LOGI(TAG, "Action: FORWARD (Tiến)");
        motor_control(95.0f, 95.0f); // 100% max speed
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Stop
        ESP_LOGI(TAG, "Action: STOP");
        motor_control(0.0f, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(1000));

        // 2. Backward (Lùi)
        ESP_LOGI(TAG, "Action: BACKWARD (Lùi)");
        motor_control(-95.0f, -95.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Stop
        ESP_LOGI(TAG, "Action: STOP");
        motor_control(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 3. Pivot Left (Xoay trái, quay tại chỗ)
        ESP_LOGI(TAG, "Action: PIVOT LEFT (Xoay trái)");
        motor_control(-40.0f, 40.0f); // Trái lùi, phải tiến
        vTaskDelay(pdMS_TO_TICKS(1500));

        // Stop
        ESP_LOGI(TAG, "Action: STOP");
        motor_control(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 4. Pivot Right (Xoay phải, quay tại chỗ)
        ESP_LOGI(TAG, "Action: PIVOT RIGHT (Xoay phải)");
        motor_control(40.0f, -40.0f); // Trái tiến, phải lùi
        vTaskDelay(pdMS_TO_TICKS(1500));

        // Stop
        ESP_LOGI(TAG, "Action: STOP");
        motor_control(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 5. Turn Left (Rẽ trái khi tiến)
        ESP_LOGI(TAG, "Action: TURN LEFT (Rẽ trái)");
        motor_control(20.0f, 60.0f); // Bánh trái chậm, phải nhanh
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Stop
        ESP_LOGI(TAG, "Action: STOP");
        motor_control(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 6. Turn Right (Rẽ phải khi tiến)
        ESP_LOGI(TAG, "Action: TURN RIGHT (Rẽ phải)");
        motor_control(60.0f, 20.0f); // Bánh trái nhanh, phải chậm
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Stop
        ESP_LOGI(TAG, "Action: STOP");
        motor_control(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}