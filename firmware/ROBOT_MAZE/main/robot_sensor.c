#include "robot_sensor.h"
#include <stdio.h>
#include <stdbool.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver_mpu6050_dmp.h"

static const char *TAG = "Robot_Sensor";

/* Note: LibDriver MPU6050 interface expects a global symbol named `bus_handle`.
   If it's defined in main.c, we use it as extern here. But if we define it here, 
   we need to make sure we assign the initialized bus to it. */
extern i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_bus_handle_t bus_handle = NULL;

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

static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6050_INTERRUPT_MOTION :
            mpu6050_interface_debug_print("mpu6050: irq motion.\n");
            break;
        case MPU6050_INTERRUPT_FIFO_OVERFLOW :
            mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");
            break;
        case MPU6050_INTERRUPT_I2C_MAST :
            mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");
            break;
        case MPU6050_INTERRUPT_DMP :
            mpu6050_interface_debug_print("mpu6050: irq dmp\n");
            break;
        case MPU6050_INTERRUPT_DATA_READY :
            mpu6050_interface_debug_print("mpu6050: irq data ready\n");
            break;
        default :
            mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");
            break;
    }
}

static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    /* Handle tap callback */
}

static void a_dmp_orient_callback(uint8_t orientation)
{
    /* Handle orient callback */
}

static void mpu_sensor_task(void *arg)
{
    uint8_t res;
    uint16_t len;
    
    while(1)
    {
        len = 128;

        res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                  gs_gyro_raw, gs_gyro_dps,
                                  gs_quat,
                                  gs_pitch, gs_roll, gs_yaw,
                                  &len);
        if (res == 0)
        {
            // Here you can integrate with the controller task. E.g. save the yaw data globally.
            // ESP_LOGD(TAG, "DMP(len=%u): pitch=%0.2f roll=%0.2f yaw=%0.2f",
            //          (unsigned)len, (double)gs_pitch[0], (double)gs_roll[0], (double)gs_yaw[0]);
        }
        else if (res != 7 && res != 8 && res != 6 && res != 1) 
        {
            ESP_LOGE(TAG, "mpu6050_dmp_read_all failed (code %d).", res);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void robot_sensor_init(void)
{
    ESP_LOGI(TAG, "MPU6050 Initialization starting...");

    // Make sure we have the bus_handle correctly pointing to out main I2C bus config.
    if (i2c_bus_handle != NULL) {
        bus_handle = i2c_bus_handle;
    } else {
        ESP_LOGE(TAG, "I2C bus handle is NULL. Initialization stopped.");
        return;
    }

    if (gpio_interrupt_init() != 0)
    {
        ESP_LOGE(TAG, "GPIO interrupt init failed.");
        return;
    }

    if (xTaskCreatePinnedToCore(mpu6050_irq_task,
                               "mpu6050_irq",
                               4096,
                               NULL,
                               10, // Higher priority than user task
                               &s_mpu6050_irq_task_handle,
                               1) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create mpu6050_irq task.");
        (void)gpio_interrupt_deinit();
        return;
    }

    g_gpio_irq = mpu6050_dmp_irq_handler;

    uint8_t res = mpu6050_dmp_init(MPU6050_ADDRESS_AD0_LOW, a_receive_callback,
                           a_dmp_tap_callback, a_dmp_orient_callback);
    if (res != 0)
    {
        ESP_LOGE(TAG, "mpu6050_dmp_init failed.");
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
        return;
    }

    (void)gpio_intr_enable(INTERRUPT_MPU6050_IO);

    xTaskCreatePinnedToCore(mpu_sensor_task, "mpu_sensor_task", 4096, NULL, 5, NULL, 1);
    
    ESP_LOGI(TAG, "MPU6050 Initialization complete.");
}

float robot_sensor_get_yaw(void)
{
    return gs_yaw[0];
}
