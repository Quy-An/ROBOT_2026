#include <stdio.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver_mpu6050.h"
#include "driver_mpu6050_interface.h"

static const char *TAG = "Main_App";

i2c_master_bus_handle_t bus_handle;
mpu6050_handle_t mpu6050_handle;

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
        ESP_LOGE(TAG, "=> NO I2C DEVICES FOUND! Check wiring (SDA/SCL swapped?), power, or pull-up resistors.");
    }
    ESP_LOGW(TAG, "Scan completed.");
}

/**
 * @brief Initialize MPU6050
 */
static void imu_init(void) {
    uint8_t res;
    
    // Link interface functions
    DRIVER_MPU6050_LINK_INIT(&mpu6050_handle, mpu6050_handle_t);
    DRIVER_MPU6050_LINK_IIC_INIT(&mpu6050_handle, mpu6050_interface_iic_init);
    DRIVER_MPU6050_LINK_IIC_DEINIT(&mpu6050_handle, mpu6050_interface_iic_deinit);
    DRIVER_MPU6050_LINK_IIC_READ(&mpu6050_handle, mpu6050_interface_iic_read);
    DRIVER_MPU6050_LINK_IIC_WRITE(&mpu6050_handle, mpu6050_interface_iic_write);
    DRIVER_MPU6050_LINK_DELAY_MS(&mpu6050_handle, mpu6050_interface_delay_ms);
    DRIVER_MPU6050_LINK_DEBUG_PRINT(&mpu6050_handle, mpu6050_interface_debug_print);
    DRIVER_MPU6050_LINK_RECEIVE_CALLBACK(&mpu6050_handle, mpu6050_interface_receive_callback);

    // Initialize MPU6050 AD0 Address (0xD0 for ad0 low, 0xD2 for ad0 high)
    // We will default to LOW (0x68). If the scanner finds 0x69, you should change this to MPU6050_ADDRESS_AD0_HIGH
    res = mpu6050_set_addr_pin(&mpu6050_handle, MPU6050_ADDRESS_AD0_LOW);
    if (res != 0) {
        ESP_LOGE(TAG, "MPU6050 set address failed");
        return;
    }

    // Initialize MPU6050 hardware
    res = mpu6050_init(&mpu6050_handle);
    if (res != 0) {
        ESP_LOGE(TAG, "MPU6050 init failed");
        return;
    }

    /* Follow LibDriver read_test init sequence */
    vTaskDelay(pdMS_TO_TICKS(100));

    res = mpu6050_set_sleep(&mpu6050_handle, MPU6050_BOOL_FALSE);
    if (res != 0) {
        ESP_LOGE(TAG, "MPU6050 disable sleep failed");
        (void)mpu6050_deinit(&mpu6050_handle);
        return;
    }

    res = mpu6050_set_low_pass_filter(&mpu6050_handle, MPU6050_LOW_PASS_FILTER_3);
    if (res != 0) {
        ESP_LOGE(TAG, "MPU6050 set low pass filter failed");
        (void)mpu6050_deinit(&mpu6050_handle);
        return;
    }

    res = mpu6050_set_temperature_sensor(&mpu6050_handle, MPU6050_BOOL_TRUE);
    if (res != 0) {
        ESP_LOGE(TAG, "MPU6050 enable temperature sensor failed");
        (void)mpu6050_deinit(&mpu6050_handle);
        return;
    }

    res = mpu6050_set_cycle_wake_up(&mpu6050_handle, MPU6050_BOOL_FALSE);
    if (res != 0) {
        ESP_LOGE(TAG, "MPU6050 disable cycle wake up failed");
        (void)mpu6050_deinit(&mpu6050_handle);
        return;
    }

    /* read_test sets wake-up frequency even though cycle is disabled; keep it */
    res = mpu6050_set_wake_up_frequency(&mpu6050_handle, MPU6050_WAKE_UP_FREQUENCY_1P25_HZ);
    if (res != 0) {
        ESP_LOGE(TAG, "MPU6050 set wake up frequency failed");
        (void)mpu6050_deinit(&mpu6050_handle);
        return;
    }

    /* Enable all accel/gyro axes */
    if (mpu6050_set_standby_mode(&mpu6050_handle, MPU6050_SOURCE_ACC_X, MPU6050_BOOL_FALSE) != 0 ||
        mpu6050_set_standby_mode(&mpu6050_handle, MPU6050_SOURCE_ACC_Y, MPU6050_BOOL_FALSE) != 0 ||
        mpu6050_set_standby_mode(&mpu6050_handle, MPU6050_SOURCE_ACC_Z, MPU6050_BOOL_FALSE) != 0 ||
        mpu6050_set_standby_mode(&mpu6050_handle, MPU6050_SOURCE_GYRO_X, MPU6050_BOOL_FALSE) != 0 ||
        mpu6050_set_standby_mode(&mpu6050_handle, MPU6050_SOURCE_GYRO_Y, MPU6050_BOOL_FALSE) != 0 ||
        mpu6050_set_standby_mode(&mpu6050_handle, MPU6050_SOURCE_GYRO_Z, MPU6050_BOOL_FALSE) != 0) {
        ESP_LOGE(TAG, "MPU6050 set standby mode failed");
        (void)mpu6050_deinit(&mpu6050_handle);
        return;
    }

    /* Disable self-test */
    (void)mpu6050_set_gyroscope_test(&mpu6050_handle, MPU6050_AXIS_X, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_gyroscope_test(&mpu6050_handle, MPU6050_AXIS_Y, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_gyroscope_test(&mpu6050_handle, MPU6050_AXIS_Z, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_accelerometer_test(&mpu6050_handle, MPU6050_AXIS_X, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_accelerometer_test(&mpu6050_handle, MPU6050_AXIS_Y, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_accelerometer_test(&mpu6050_handle, MPU6050_AXIS_Z, MPU6050_BOOL_FALSE);

    /* Disable FIFO and FIFO sources */
    (void)mpu6050_set_fifo(&mpu6050_handle, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_fifo_enable(&mpu6050_handle, MPU6050_FIFO_TEMP, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_fifo_enable(&mpu6050_handle, MPU6050_FIFO_XG, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_fifo_enable(&mpu6050_handle, MPU6050_FIFO_YG, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_fifo_enable(&mpu6050_handle, MPU6050_FIFO_ZG, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_fifo_enable(&mpu6050_handle, MPU6050_FIFO_ACCEL, MPU6050_BOOL_FALSE);

    /* Disable I2C master/bypass */
    (void)mpu6050_set_iic_master(&mpu6050_handle, MPU6050_BOOL_FALSE);
    (void)mpu6050_set_iic_bypass(&mpu6050_handle, MPU6050_BOOL_FALSE);

    /* Set configuration: ±2000 dps and ±8g are good for robotics */
    (void)mpu6050_set_gyroscope_range(&mpu6050_handle, MPU6050_GYROSCOPE_RANGE_2000DPS);
    (void)mpu6050_set_accelerometer_range(&mpu6050_handle, MPU6050_ACCELEROMETER_RANGE_8G);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Post-init sanity checks */
    // {
    //     mpu6050_bool_t sleep_en = MPU6050_BOOL_TRUE;
    //     mpu6050_bool_t cycle_en = MPU6050_BOOL_FALSE;
    //     mpu6050_clock_source_t clk = MPU6050_CLOCK_SOURCE_INTERNAL_8MHZ;

    //     if (mpu6050_get_sleep(&mpu6050_handle, &sleep_en) == 0 &&
    //         mpu6050_get_cycle_wake_up(&mpu6050_handle, &cycle_en) == 0 &&
    //         mpu6050_get_clock_source(&mpu6050_handle, &clk) == 0) {
    //         ESP_LOGI(TAG, "MPU6050 state: sleep=%d cycle=%d clock_src=%d", (int)sleep_en, (int)cycle_en, (int)clk);
    //     }

    //     mpu6050_bool_t st = MPU6050_BOOL_FALSE;
    //     if (mpu6050_get_standby_mode(&mpu6050_handle, MPU6050_SOURCE_ACC_X, &st) == 0) ESP_LOGI(TAG, "Standby ACC_X=%d", (int)st);
    //     if (mpu6050_get_standby_mode(&mpu6050_handle, MPU6050_SOURCE_ACC_Y, &st) == 0) ESP_LOGI(TAG, "Standby ACC_Y=%d", (int)st);
    //     if (mpu6050_get_standby_mode(&mpu6050_handle, MPU6050_SOURCE_ACC_Z, &st) == 0) ESP_LOGI(TAG, "Standby ACC_Z=%d", (int)st);
    //     if (mpu6050_get_standby_mode(&mpu6050_handle, MPU6050_SOURCE_GYRO_X, &st) == 0) ESP_LOGI(TAG, "Standby GYRO_X=%d", (int)st);
    //     if (mpu6050_get_standby_mode(&mpu6050_handle, MPU6050_SOURCE_GYRO_Y, &st) == 0) ESP_LOGI(TAG, "Standby GYRO_Y=%d", (int)st);
    //     if (mpu6050_get_standby_mode(&mpu6050_handle, MPU6050_SOURCE_GYRO_Z, &st) == 0) ESP_LOGI(TAG, "Standby GYRO_Z=%d", (int)st);
    // }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully.");
}

void app_main(void)
{
    // 1. Initialize the shared I2C bus
    i2c_bus_init();
    
    // // Scan for devices before initializing drivers to see hardware reality
    // i2c_scanner();

    // 2. Initialize the MPU6050 sensor
    imu_init();

    // Data buffers
    int16_t accel_raw[1][3] = {0}, gyro_raw[1][3] = {0};
    float accel_g[1][3] = {0}, gyro_dps[1][3] = {0};
    int16_t temp_raw = 0;
    float temp_c = 0.0f;
    uint16_t len = 1;

    while (1) {
        /* mpu6050_read uses len as input/output; keep it at 1 sample for this loop. */
        len = 1;
        // Read accel and gyro data from sensor
        if (mpu6050_read(&mpu6050_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len) == 0) {
            ESP_LOGI(TAG, "Raw: Accel=%d,%d,%d Gyro=%d,%d,%d (len=%u)",
                     (int)accel_raw[0][0], (int)accel_raw[0][1], (int)accel_raw[0][2],
                     (int)gyro_raw[0][0], (int)gyro_raw[0][1], (int)gyro_raw[0][2],
                     (unsigned)len);

            ESP_LOGI(TAG, "Accel(g): X=%0.2f Y=%0.2f Z=%0.2f | Gyro(dps): X=%0.2f Y=%0.2f Z=%0.2f",
                     accel_g[0][0], accel_g[0][1], accel_g[0][2],
                     gyro_dps[0][0], gyro_dps[0][1], gyro_dps[0][2]);
        } else {
            ESP_LOGW(TAG, "Failed to read MPU6050");
        }

        // Read temperature data from sensor
        if (mpu6050_read_temperature(&mpu6050_handle, &temp_raw, &temp_c) == 0) {
                ESP_LOGI(TAG, "Temp: raw=%d C=%0.2f", (int)temp_raw, (double)temp_c);
        } else {
            ESP_LOGW(TAG, "Failed to read MPU6050 temperature");
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay between reads (10 Hz)
    }

}