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
#include "driver_mpu6050_dmp.h"

/**
 * @brief  TAG for logging
 */
static const char *TAG = "Main_App";

/**
 * @brief MPU6050 handle and I2C bus handle;
 */
/* NOTE: LibDriver MPU6050 interface expects a global symbol named `bus_handle`. */
i2c_master_bus_handle_t bus_handle = NULL;

/**
 * @brief variable to store read data
 */
static uint8_t (*g_gpio_irq)(void) = NULL;
static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];      
static float gs_gyro_dps[128][3];        
static int32_t gs_quat[128][4];          
static float gs_pitch[128];              
static float gs_roll[128];                
static float gs_yaw[128];                     

static bool s_gpio_isr_service_installed = false;
static TaskHandle_t s_mpu6050_irq_task_handle = NULL;

static void mpu6050_irq_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        /* Wait until ISR notifies us that MPU6050 asserted INT. */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (g_gpio_irq != NULL)
        {
            /* Drain latched/queued interrupts while INT is active (active-low). */
            while (gpio_get_level(INTERRUPT_MPU6050_IO) == 0)
            {
                (void)g_gpio_irq();
            }
        }

        /* Re-enable GPIO interrupt for next event. */
        (void)gpio_intr_enable(INTERRUPT_MPU6050_IO);
    }
}

static void mpu6050_gpio_isr_handler(void *arg)
{
    (void)arg;

    /* IMPORTANT: do NOT call I2C/driver functions here.
     * Just notify a task; otherwise we can hit ISR WDT.
     */
    (void)gpio_intr_disable(INTERRUPT_MPU6050_IO);

    if (s_mpu6050_irq_task_handle != NULL)
    {
        BaseType_t high_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_mpu6050_irq_task_handle, &high_task_woken);
        if (high_task_woken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
}

static int gpio_interrupt_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INTERRUPT_MPU6050_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return 1;
    }

    if (!s_gpio_isr_service_installed)
    {
        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
            return 1;
        }
        s_gpio_isr_service_installed = true;
    }

    err = gpio_isr_handler_add(INTERRUPT_MPU6050_IO, mpu6050_gpio_isr_handler, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        return 1;
    }

    /* Keep IRQ disabled until g_gpio_irq is assigned and DMP is initialized. */
    (void)gpio_intr_disable(INTERRUPT_MPU6050_IO);
    return 0;
}

static int gpio_interrupt_deinit(void)
{
    (void)gpio_intr_disable(INTERRUPT_MPU6050_IO);

    esp_err_t err = gpio_isr_handler_remove(INTERRUPT_MPU6050_IO);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "gpio_isr_handler_remove failed: %s", esp_err_to_name(err));
        return 1;
    }

    return 0;
}

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
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t ret = i2c_master_probe(bus_handle, addr, 100);
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
 * @brief dmp funtion
 */
static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6050_INTERRUPT_MOTION :
        {
            mpu6050_interface_debug_print("mpu6050: irq motion.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_I2C_MAST :
        {
            mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DMP :
        {
            mpu6050_interface_debug_print("mpu6050: irq dmp\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DATA_READY :
        {
            mpu6050_interface_debug_print("mpu6050: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");
            
            break;
        }
    }
}

static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6050_DMP_TAP_X_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_X_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");
            
            break;
        }
    }
}

static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6050_DMP_ORIENT_PORTRAIT :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_LANDSCAPE :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");
            
            break;
        }
    }
}

void app_main(void)
{
    uint8_t res;
    uint16_t len;
    uint32_t cnt;

    ESP_LOGI(TAG, "MPU6050 DMP demo starting...");

    /* 1) Initialize the shared I2C bus (ESP-IDF v5.x i2c_master) */
    i2c_bus_init();
    if (bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C init failed, cannot continue.");
        return;
    }
    
    // // Scan for devices before initializing drivers to see hardware reality
    i2c_scanner();

    /* 2) Init GPIO interrupt line from MPU6050 INT pin */
    if (gpio_interrupt_init() != 0)
    {
        ESP_LOGE(TAG, "GPIO interrupt init failed.");
        return;
    }

    /* Create a task to process MPU6050 interrupts in task context (safe for I2C). */
    if (xTaskCreatePinnedToCore(mpu6050_irq_task,
                               "mpu6050_irq",
                               4096,
                               NULL,
                               10,
                               &s_mpu6050_irq_task_handle,
                               0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create mpu6050_irq task.");
        (void)gpio_interrupt_deinit();
        return;
    }

    /* Link ISR -> LibDriver IRQ handler */
    g_gpio_irq = mpu6050_dmp_irq_handler;

    /* 3) Init DMP example (LibDriver) */
    res = mpu6050_dmp_init(MPU6050_ADDRESS_AD0_LOW, a_receive_callback,
                           a_dmp_tap_callback, a_dmp_orient_callback);
    if (res != 0)
    {
        ESP_LOGE(TAG, "mpu6050_dmp_init failed.");
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
        return;
    }

    (void)gpio_intr_enable(INTERRUPT_MPU6050_IO);

    /* Give sensor/DMP some time after init */
    mpu6050_interface_delay_ms(500);

    while (1)
    {
        len = 128;

        res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                  gs_gyro_raw, gs_gyro_dps,
                                  gs_quat,
                                  gs_pitch, gs_roll, gs_yaw,
                                  &len);
        if (res != 0)
        {
            ESP_LOGE(TAG, "mpu6050_dmp_read_all failed.");
            break;
        }

        /* Main DMP outputs (degrees) */
        ESP_LOGI(TAG, "DMP(len=%u): pitch=%0.2f roll=%0.2f yaw=%0.2f",
                 (unsigned)len, (double)gs_pitch[0], (double)gs_roll[0], (double)gs_yaw[0]);

        /* Optional: uncomment if you also want accel/gyro in the same loop */
        // ESP_LOGI(TAG, "Accel(g): %0.2f %0.2f %0.2f | Gyro(dps): %0.2f %0.2f %0.2f",
        //          (double)gs_accel_g[0][0], (double)gs_accel_g[0][1], (double)gs_accel_g[0][2],
        //          (double)gs_gyro_dps[0][0], (double)gs_gyro_dps[0][1], (double)gs_gyro_dps[0][2]);

        cnt = 0;
        res = mpu6050_dmp_get_pedometer_counter(&cnt);
        if (res == 0)
        {
            mpu6050_interface_debug_print("mpu6050: steps %lu.\n", (unsigned long)cnt);
        }

        mpu6050_interface_delay_ms(500);
    }

    /* cleanup on error */
    (void)mpu6050_dmp_deinit();
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();
}