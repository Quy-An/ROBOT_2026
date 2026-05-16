#include <stdio.h>
#include <stdbool.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver_pca9685_basic.h"
#include "robot_controller.h"
#include "sensor_mpu6050.h"
#include "sensor_task.h"

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
    ESP_LOGI(TAG, "Robot Maze Solver - PID Turning Test");

    /* 1) Khởi tạo I2C */
    i2c_bus_init();
    if (i2c_bus_handle == NULL) return;

    /* 2) Khởi tạo động học */
    kinematics_init();

    /* 3) Khởi tạo MPU6050 DMP */
    if (mpu6050_controller_init() != 0) {
        ESP_LOGE(TAG, "MPU6050 initialization failed.");
        return;
    }

    /* 4) Chạy task điều khiển */
    robot_controller_task_start();

    // Đợi 5 giây để cảm biến DMP ổn định hoàn toàn và triệt tiêu trôi (drift) ban đầu
    ESP_LOGW(TAG, "Waiting for DMP stabilization (5s)...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Starting Test Sequence.");

    while (1) {
        float p, r, y;
        mpu6050_get_euler_angles(&p, &r, &y);
        ESP_LOGI(TAG, "Current Yaw: %.2f", y);
        vTaskDelay(pdMS_TO_TICKS(200));

        // // --- TEST XOAY TRÁI 90 ĐỘ ---
        // ESP_LOGW(TAG, ">>>>> COMMAND: TURN LEFT 90 DEGREES <<<<<");
        // robot_turn_relative(90.0f);
        
        // // In góc liên tục trong 3 giây để quan sát quá trình xoay
        // for(int i = 0; i < 30; i++) {
        //     mpu6050_get_euler_angles(&p, &r, &y);
        //     ESP_LOGI(TAG, "Current Yaw: %.2f", y);
        //     vTaskDelay(pdMS_TO_TICKS(100));
        // }

        // // Dừng lại 2 giây để quan sát kết quả cuối cùng
        // ESP_LOGI(TAG, "Pause...");
        // vTaskDelay(pdMS_TO_TICKS(2000));

        // // --- TEST XOAY PHẢI 90 ĐỘ ---
        // ESP_LOGW(TAG, ">>>>> COMMAND: TURN RIGHT 90 DEGREES <<<<<");
        // robot_turn_relative(-90.0f);
        
        // for(int i = 0; i < 30; i++) {
        //     mpu6050_get_euler_angles(&p, &r, &y);
        //     ESP_LOGI(TAG, "Current Yaw: %.2f", y);
        //     vTaskDelay(pdMS_TO_TICKS(100));
        // }

        // ESP_LOGI(TAG, "Pause...");
        // vTaskDelay(pdMS_TO_TICKS(2000));
        
        // // --- TEST ĐI THẲNG 2 GIÂY (GIỮ HƯỚNG) ---
        // ESP_LOGW(TAG, ">>>>> COMMAND: DRIVE STRAIGHT <<<<<");
        // robot_drive_straight(0.3f); // Đi chậm 0.3 m/s để test PID giữ hướng
        // for(int i = 0; i < 20; i++) {
        //     mpu6050_get_euler_angles(&p, &r, &y);
        //     ESP_LOGI(TAG, "Straight Path - Yaw: %.2f", y);
        //     vTaskDelay(pdMS_TO_TICKS(100));
        // }
        
        // robot_controller_set_velocity(0.0f, 0.0f); // Dừng robot
        // vTaskDelay(pdMS_TO_TICKS(2000));
    }
}