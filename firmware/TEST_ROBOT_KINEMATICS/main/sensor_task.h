#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "vl53l0x_espidf.h"
#include "vl53l0x_api.h"

#define NUM_SENSORS 5

#define ADDR_FRONT        0x30
#define ADDR_FRONT_LEFT   0x32
#define ADDR_FRONT_RIGHT  0x31
#define ADDR_LEFT         0x33
#define ADDR_RIGHT        0x34

// -------------------------------------------------------
// Struct cấu hình mỗi sensor
// -------------------------------------------------------
typedef struct {
    uint32_t id;
    const char* name;
    gpio_num_t xshut_io;
    gpio_num_t int_io;
    uint8_t target_addr;
    VL53L0X_Dev_t dev_struct;
    VL53L0X_DEV dev;
    QueueHandle_t sensor_queue;
} sensor_cfg_t;

// -------------------------------------------------------
// Dữ liệu chia sẻ giữa các task
// -------------------------------------------------------
extern uint16_t    sensor_distances[NUM_SENSORS];
extern uint8_t     sensor_status[NUM_SENSORS];
extern sensor_cfg_t sensors[NUM_SENSORS];

// -------------------------------------------------------
// API public
// -------------------------------------------------------

/**
 * @brief Khởi tạo toàn bộ cảm biến (XSHUT, I2C addr, calibration, ISR/poll task)
 * @param bus_handle  I2C master bus đã được khởi tạo từ ngoài
 */
void sensors_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Task in tổng hợp khoảng cách 500ms/lần — truyền NULL khi tạo task
 */
void print_summary_task(void *pvParameters);
