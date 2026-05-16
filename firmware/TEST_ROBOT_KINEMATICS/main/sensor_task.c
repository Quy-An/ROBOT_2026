#include "sensor_task.h"

#include <stdio.h>
#include <string.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "sensor_task";

// -------------------------------------------------------
// Dữ liệu chia sẻ
// -------------------------------------------------------
uint16_t sensor_distances[NUM_SENSORS] = {0};
uint8_t  sensor_status[NUM_SENSORS]    = {255}; // 255 = chưa có data

sensor_cfg_t sensors[NUM_SENSORS] = {
    { .id = 0, .name = "FRONT",       .xshut_io = XSHUT_FRONT_IO,       .int_io = INTERUPT_FRONT_IO,       .target_addr = ADDR_FRONT       },
    { .id = 1, .name = "FRONT_LEFT",  .xshut_io = XSHUT_FRONT_LEFT_IO,  .int_io = INTERUPT_FRONT_LEFT_IO,  .target_addr = ADDR_FRONT_LEFT  },
    { .id = 2, .name = "FRONT_RIGHT", .xshut_io = XSHUT_FRONT_RIGHT_IO, .int_io = INTERUPT_FRONT_RIGHT_IO, .target_addr = ADDR_FRONT_RIGHT },
    { .id = 3, .name = "LEFT",        .xshut_io = XSHUT_LEFT_IO,        .int_io = INTERUPT_LEFT_IO,        .target_addr = ADDR_LEFT        },
    { .id = 4, .name = "RIGHT",       .xshut_io = XSHUT_RIGHT_IO,       .int_io = INTERUPT_RIGHT_IO,       .target_addr = ADDR_RIGHT       }
};

// -------------------------------------------------------
// ISR Handler
// -------------------------------------------------------
static void IRAM_ATTR vl53l0x_isr_handler(void *arg) {
    sensor_cfg_t *s = (sensor_cfg_t *)arg;
    xQueueSendFromISR(s->sensor_queue, &s->id, NULL);
}

// -------------------------------------------------------
// Task ISR-driven (FRONT, FRONT_LEFT, FRONT_RIGHT, RIGHT)
// -------------------------------------------------------
static void vl53l0x_sensor_task(void *pvParameters) {
    sensor_cfg_t *s = (sensor_cfg_t *)pvParameters;
    VL53L0X_RangingMeasurementData_t data;
    uint32_t dummy_id;

    while (1) {
        if (xQueueReceive(s->sensor_queue, &dummy_id, portMAX_DELAY)) {
            VL53L0X_Error status = VL53L0X_GetRangingMeasurementData(s->dev, &data);
            if (status == VL53L0X_ERROR_NONE && data.RangeStatus == 0) {
                sensor_distances[s->id] = data.RangeMilliMeter;
                sensor_status[s->id]    = data.RangeStatus;
            }
            VL53L0X_ClearInterruptMask(s->dev, 0);
        }
    }
}


static void vl53l0x_poll_task(void *pvParameters) {
    sensor_cfg_t *s = (sensor_cfg_t *)pvParameters;
    VL53L0X_RangingMeasurementData_t data;
    uint8_t ready = 0;

    while (1) {
        VL53L0X_GetMeasurementDataReady(s->dev, &ready);
        if (ready) {
            VL53L0X_Error status = VL53L0X_GetRangingMeasurementData(s->dev, &data);
            if (status == VL53L0X_ERROR_NONE && data.RangeStatus == 0) {
                sensor_distances[s->id] = data.RangeMilliMeter;
                sensor_status[s->id]    = data.RangeStatus;
            }
            VL53L0X_ClearInterruptMask(s->dev, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// -------------------------------------------------------
// Task in bảng tổng hợp
// -------------------------------------------------------
void print_summary_task(void *pvParameters) {
    while (1) {
        printf("[FRONT:%4umm | FL:%4umm | FR:%4umm | LEFT:%4umm | RIGHT:%4umm]\n",
            sensor_distances[0],
            sensor_distances[1],
            sensor_distances[2],
            sensor_distances[3],
            sensor_distances[4]
        );
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// -------------------------------------------------------
// Khởi tạo toàn bộ cảm biến
// -------------------------------------------------------
void sensors_init(i2c_master_bus_handle_t bus_handle) {
    // Cấu hình GPIO interrupt cho tất cả sensor
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_NEGEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = 1,
        .pin_bit_mask = (1ULL << INTERUPT_FRONT_IO)       |
                        (1ULL << INTERUPT_FRONT_LEFT_IO)  |
                        (1ULL << INTERUPT_FRONT_RIGHT_IO) |
                        (1ULL << INTERUPT_LEFT_IO)        |
                        (1ULL << INTERUPT_RIGHT_IO)
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);

    // Tắt tất cả sensor trước
    for (int i = 0; i < NUM_SENSORS; i++) {
        gpio_set_direction(sensors[i].xshut_io, GPIO_MODE_OUTPUT);
        gpio_set_level(sensors[i].xshut_io, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Khởi tạo từng sensor một
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_cfg_t *s = &sensors[i];

        s->sensor_queue = xQueueCreate(5, sizeof(uint32_t));

        gpio_set_level(s->xshut_io, 1);
        vTaskDelay(pdMS_TO_TICKS(200));

        vl53l0x_espidf_i2c_register(bus_handle, 0x29, I2C_MASTER_FREQ_HZ);
        vl53l0x_espidf_init_st_device(&s->dev_struct, 0x29, 400);
        s->dev = vl53l0x_espidf_as_st_dev(&s->dev_struct);

        VL53L0X_DataInit(s->dev);
        VL53L0X_StaticInit(s->dev);

        // Đổi địa chỉ I2C nếu khác 0x29
        if (s->target_addr != 0x29) {
            VL53L0X_SetDeviceAddress(s->dev, (s->target_addr << 1));
            vl53l0x_espidf_i2c_unregister(0x29);
            vl53l0x_espidf_i2c_register(bus_handle, s->target_addr, I2C_MASTER_FREQ_HZ);
            s->dev_struct.I2cDevAddr = s->target_addr;
        }

        // Calibration
        uint8_t vhv, phase;
        VL53L0X_PerformRefCalibration(s->dev, &vhv, &phase);
        uint32_t count;
        uint8_t is_aperture;
        VL53L0X_PerformRefSpadManagement(s->dev, &count, &is_aperture);

        // Cấu hình GPIO interrupt của sensor
        VL53L0X_SetGpioConfig(s->dev, 0,
            VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING,
            VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
            VL53L0X_INTERRUPTPOLARITY_LOW);

        // Chế độ đo liên tục
        VL53L0X_SetDeviceMode(s->dev, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);
        VL53L0X_SetInterMeasurementPeriodMilliSeconds(s->dev, 2000);

        VL53L0X_StartMeasurement(s->dev);
        VL53L0X_ClearInterruptMask(s->dev, 0);

        // Gắn ISR cho tất cả cảm biến
        gpio_isr_handler_add(s->int_io, vl53l0x_isr_handler, (void *)s);

        // Tạo task tương ứng
        char task_name[16];
        snprintf(task_name, sizeof(task_name), "ToF_%s", s->name);
        xTaskCreate(vl53l0x_sensor_task, task_name, 3072, (void *)s, 5, NULL);

        ESP_LOGI(TAG, "[%s] OK tại 0x%02x", s->name, s->target_addr);
    }
}
