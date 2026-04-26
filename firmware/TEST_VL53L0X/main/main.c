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
 * @brief Địa chỉ I2C 7-bit của VL53L0X
 * Theo Datasheet (Section 4), địa chỉ byte ghi là 0x52 (8-bit),
 * tương đương với địa chỉ 7-bit là 0x29 mà thư viện I2C ESP-IDF sử dụng.
 */
#define VL53L0X_I2C_ADDR 0x29

/**
 * @brief I2C master bus handle
 */
i2c_master_bus_handle_t bus_handle;

/**
 * @brief Cấu trúc thiết bị VL53L0X dùng cho ST API
 */
static VL53L0X_Dev_t vl53_dev;

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
 * @brief Scan I2C bus for devices to debug NACK issues
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
        ESP_LOGE(TAG, "=> NO I2C DEVICES FOUND! Check wiring, power, or pull-up resistors.");
    }
    ESP_LOGW(TAG, "Scan completed.");
}

/**
 * @brief Initialize VL53L0X sensor and start continuous measurement
 * Tuân thủ quy trình khuyến nghị trong thư mục README.md
 */
static bool vl53l0x_sensor_init_and_start(void)
{
    VL53L0X_Error st_err;
    
    ESP_LOGI(TAG, "Registering VL53L0X device on I2C bus at address 0x%02X...", VL53L0X_I2C_ADDR);
    
    // 1. Đăng ký I2C device vào ESP-IDF bus
    esp_err_t err = vl53l0x_espidf_i2c_register(bus_handle, VL53L0X_I2C_ADDR, 400000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register VL53L0X device: %s", esp_err_to_name(err));
        return false;
    }
    
    // 2. Khởi tạo cấu trúc dữ liệu ST API
    vl53l0x_espidf_init_st_device(&vl53_dev, VL53L0X_I2C_ADDR, 400);
    VL53L0X_DEV dev = vl53l0x_espidf_as_st_dev(&vl53_dev);
    
    // 3. DataInit và StaticInit
    ESP_LOGI(TAG, "Running VL53L0X_DataInit...");
    st_err = VL53L0X_DataInit(dev);
    if (st_err != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53L0X_DataInit failed: %d", st_err);
        return false;
    }
    
    ESP_LOGI(TAG, "Running VL53L0X_StaticInit...");
    st_err = VL53L0X_StaticInit(dev);
    if (st_err != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53L0X_StaticInit failed: %d", st_err);
        return false;
    }

    // 4. (Tùy chọn) Perform Calibration (Tham khảo từ README.md)
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    ESP_LOGI(TAG, "Performing Ref Calibration...");
    VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);

    // 5. Cấu hình chế độ đo liên tục (Continuous Ranging)
    ESP_LOGI(TAG, "Setting Device Mode to Continuous Ranging...");
    st_err = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (st_err != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53L0X_SetDeviceMode failed: %d", st_err);
        return false;
    }

    // Tùy chỉnh inter-measurement period (VD: đo mỗi 50ms)
    // VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev, 50);

    // 6. Bắt đầu đo
    ESP_LOGI(TAG, "Starting Measurement...");
    st_err = VL53L0X_StartMeasurement(dev);
    if (st_err != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53L0X_StartMeasurement failed: %d", st_err);
        return false;
    }

    ESP_LOGI(TAG, "VL53L0X sensor initialized and continuous ranging started successfully!");
    return true;
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    // Khởi tạo I2C bus
    i2c_bus_init();
    
    // Scan I2C bus (hỗ trợ debug phần cứng)
    i2c_scanner();
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Khởi tạo và thiết lập cảm biến đo liên tục
    if (!vl53l0x_sensor_init_and_start()) {
        ESP_LOGE(TAG, "Sensor initialization failed. Halting application.");
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    VL53L0X_DEV dev = vl53l0x_espidf_as_st_dev(&vl53_dev);
    VL53L0X_RangingMeasurementData_t measure;
    uint8_t ready = 0;
    
    ESP_LOGI(TAG, "Entering continuous polling loop...");

    while (1) {
        // Kiểm tra xem đã có dữ liệu đo mới chưa
        VL53L0X_GetMeasurementDataReady(dev, &ready);
        
        if (ready) {
            // Lấy dữ liệu khoảng cách
            VL53L0X_GetRangingMeasurementData(dev, &measure);
            
            if (measure.RangeStatus == 0) {
                // Status = 0 (VL53L0X_ERROR_NONE): Dữ liệu tin cậy
                ESP_LOGI(TAG, "Distance: %u mm", measure.RangeMilliMeter);
            } 
           else {
               // Đã đo xong nhưng tín hiệu ngoài tầm, nhiễu sáng, tín hiệu yếu...
                ESP_LOGW(TAG, "Invalid/Out of range: %u mm, status=%u",
             measure.RangeMilliMeter, measure.RangeStatus);
           }
            
            // QUAN TRỌNG: Phải xóa cờ ngắt để cảm biến có thể thực hiện phép đo tiếp theo
            VL53L0X_ClearInterruptMask(dev, 0);
        }
        
        // Delay nhỏ để nhường CPU cho các task khác (tránh Watchdog timeout)
        VL53L0X_PollingDelay(dev);
    }
}