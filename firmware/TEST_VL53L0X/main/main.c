#include <stdio.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"

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
 * @brief Địa chỉ I2C 7-bit mặc định của VL53L0X
 */
#define VL53L0X_DEFAULT_I2C_ADDR 0x29

/**
 * @brief I2C master bus handle
 */
i2c_master_bus_handle_t bus_handle;

/**
 * @brief Cấu trúc quản lý cảm biến VL53L0X
 */
typedef struct {
    const char *name;
    gpio_num_t xshut_pin;
    gpio_num_t intr_pin;
    uint8_t target_i2c_addr;
    VL53L0X_Dev_t st_dev;
    SemaphoreHandle_t data_ready_sem;
} vl53l0x_sensor_t;

/**
 * @brief Khai báo danh sách cảm biến cần khởi tạo.
 * Dễ dàng mở rộng thêm bằng cách thêm phần tử vào mảng này.
 */
static vl53l0x_sensor_t sensors[] = {
    {
        .name = "LEFT",
        .xshut_pin = XSHUT_LEFT_IO,
        .intr_pin = INTERUPT_LEFT_IO,
        .target_i2c_addr = 0x2A,
        .data_ready_sem = NULL
    },
    {
        .name = "RIGHT",
        .xshut_pin = XSHUT_RIGHT_IO,
        .intr_pin = INTERUPT_RIGHT_IO,
        .target_i2c_addr = 0x2B,
        .data_ready_sem = NULL
    }
};

#define NUM_SENSORS (sizeof(sensors) / sizeof(sensors[0]))

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

    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C Master Bus initialized successfully.");
    } else {
        ESP_LOGE(TAG, "I2C Master Bus initialization failed: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Trình phục vụ ngắt (ISR) cho tất cả các cảm biến
 */
static void IRAM_ATTR vl53l0x_isr_handler(void* arg) {
    vl53l0x_sensor_t *sensor = (vl53l0x_sensor_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    xSemaphoreGiveFromISR(sensor->data_ready_sem, &high_task_wakeup);
    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Khởi tạo ngắt GPIO cho một cảm biến
 */
static void vl53l0x_setup_interrupt(vl53l0x_sensor_t *sensor) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,       // Ngắt cạnh xuống (Interrupt Polarity Low)
        .mode = GPIO_MODE_INPUT,             
        .pin_bit_mask = (1ULL << sensor->intr_pin),
        .pull_down_en = 0,
        .pull_up_en = 1                       // Trở kéo lên nội bộ
    };
    gpio_config(&io_conf);

    // Gắn ISR handler
    gpio_isr_handler_add(sensor->intr_pin, vl53l0x_isr_handler, (void*)sensor);
}

/**
 * @brief Khởi tạo hệ thống nhiều cảm biến: Đổi nguồn qua XSHUT, đổi địa chỉ, calib, setup_ngắt.
 */
static bool vl53l0x_multi_sensor_init(void) {
    VL53L0X_Error st_err;
    
        // 1. Tắt toàn bộ cảm biến (Kéo XSHUT xuống LOW) và tạo semaphores
    for (int i = 0; i < NUM_SENSORS; i++) {
        gpio_config_t out_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT, // Dùng Output bình thường (Push-Pull) để chắc chắn cấp đủ điện thế
            .pin_bit_mask = (1ULL << sensors[i].xshut_pin),
            .pull_down_en = 0,
            .pull_up_en = 0
        };
        gpio_config(&out_conf);
        gpio_set_level(sensors[i].xshut_pin, 0);
        sensors[i].data_ready_sem = xSemaphoreCreateBinary();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Cài đặt dịch vụ ngắt toàn cục một lần
    gpio_install_isr_service(0);

    // 2. Khởi động lần lượt từng cảm biến
    for (int i = 0; i < NUM_SENSORS; i++) {
        vl53l0x_sensor_t *s = &sensors[i];
        ESP_LOGI(TAG, "Initializing Sensor %s...", s->name);

        // Bật cảm biến hiện tại (Kéo XSHUT lên HIGH)
        gpio_set_level(s->xshut_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); // Mất vài ms để thiết bị khởi động lại (nên để 50ms)
        
        // Đăng ký địa chỉ mặc định để cấu hình
        esp_err_t err = vl53l0x_espidf_i2c_register(bus_handle, VL53L0X_DEFAULT_I2C_ADDR, 400000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[%s] Init failed to register default adddress", s->name);
            continue;
        }

        vl53l0x_espidf_init_st_device(&s->st_dev, VL53L0X_DEFAULT_I2C_ADDR, 400);
        VL53L0X_DEV dev = vl53l0x_espidf_as_st_dev(&s->st_dev);

        // 1. PHẢI DataInit và StaticInit TRƯỚC (Theo README Mục 7)
        st_err = VL53L0X_DataInit(dev);
        if (st_err != VL53L0X_ERROR_NONE) {
            ESP_LOGE(TAG, "[%s] DataInit failed", s->name);
            continue;
        }

        st_err = VL53L0X_StaticInit(dev);
        if (st_err != VL53L0X_ERROR_NONE) {
            ESP_LOGE(TAG, "[%s] StaticInit failed", s->name);
            continue;
        }

        // Đổi địa chỉ qua I2C. ST API yêu cầu địa chỉ gốc (target x 2)
        st_err = VL53L0X_SetDeviceAddress(dev, s->target_i2c_addr * 2);
        if (st_err != VL53L0X_ERROR_NONE) {
            ESP_LOGE(TAG, "[%s] Failed to change address", s->name);
            continue;
        }

        // Đăng ký lại bằng địa chỉ mới sử dụng driver I2C của ESP-IDF 
        vl53l0x_espidf_i2c_unregister(VL53L0X_DEFAULT_I2C_ADDR);
        err = vl53l0x_espidf_i2c_register(bus_handle, s->target_i2c_addr, 400000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[%s] Init failed to register NEW adddress", s->name);
            continue;
        }

        // Cập nhật Struct thiết bị sau khi đã đổi địa chỉ
        s->st_dev.I2cDevAddr = s->target_i2c_addr;

        // Theo flow khuyến nghị của ST: Chạy DataInit và StaticInit SAU KHI đổi địa chỉ
        VL53L0X_DataInit(dev);
        VL53L0X_StaticInit(dev);

        // Hiệu chuẩn (Ref Calibration)
        uint32_t refSpadCount;
        uint8_t isApertureSpads, VhvSettings, PhaseCal;
        VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
        VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);

        // Lập cấu hình ngắt phần cứng bắn ra báo hiệu đo xong
        VL53L0X_SetGpioConfig(dev, 
            0, 
            VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, 
            VL53L0X_INTERRUPTPOLARITY_LOW, 
            VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY
        );

        // Cài đặt chân ngắt trên ESP32 lắng nghe sườn xuống
        vl53l0x_setup_interrupt(s);

        // Đặt mode và ra lệnh bắt đầu đo liên tục 
        VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        VL53L0X_StartMeasurement(dev);

        // Xóa cờ rác nếu có
        VL53L0X_ClearInterruptMask(dev, 0);

        ESP_LOGI(TAG, "Sensor %s initialized at I2C Addr: 0x%02X", s->name, s->target_i2c_addr);
    }
    return true;
}

/**
 * @brief Task để chờ và in ra dữ liệu từ một cảm biến khi quá trình đo kết thúc (có ngắt báo)
 */
void vl53l0x_reading_task(void *pvParameter) {
    vl53l0x_sensor_t *sensor = (vl53l0x_sensor_t *)pvParameter;
    VL53L0X_DEV dev = vl53l0x_espidf_as_st_dev(&sensor->st_dev);
    VL53L0X_RangingMeasurementData_t measure;

    ESP_LOGI(TAG, "Task started for sensor %s", sensor->name);

    while (1) {
        // Sleep task chờ xử lý ngắt Semaphore
        if (xSemaphoreTake(sensor->data_ready_sem, portMAX_DELAY) == pdTRUE) {
            
            // Tiến hành nhận kết quả I2C
            VL53L0X_GetRangingMeasurementData(dev, &measure);
            
            if (measure.RangeStatus == 0) {
                ESP_LOGI(TAG, "[%s] Distance: %u mm", sensor->name, measure.RangeMilliMeter);
            } 
            
            // Quan trọng: Làm sạch cờ báo ngắt nội bộ mới có thể thực hiện phép đo tiếp
            VL53L0X_ClearInterruptMask(dev, 0);
        }
    }
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    i2c_bus_init();
    
    // Bật tất cả các bộ điều khiển và tùy biến thiết lập mạng lưới
    if (!vl53l0x_multi_sensor_init()) {
        ESP_LOGE(TAG, "Failed to initialize sensors.");
    }
    
    // Gửi mỗi cảm biến một Task riêng cho việc bắt Data song song
    for (int i = 0; i < NUM_SENSORS; i++) {
        char task_name[32];
        snprintf(task_name, sizeof(task_name), "vl53_tsk_%s", sensors[i].name);
        
        xTaskCreate(vl53l0x_reading_task, task_name, 4096, &sensors[i], 5, NULL);
    }
}
