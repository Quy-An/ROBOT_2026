#include "sensor_mpu6050.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver_mpu6050_dmp.h"

static const char *TAG = "MPU6050_CTRL";

#ifndef MPU6050_READ_USE_INTERRUPT
#define MPU6050_READ_USE_INTERRUPT 1
#endif

#ifndef MPU6050_POLL_PERIOD_MS
#define MPU6050_POLL_PERIOD_MS 10
#endif

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
static TaskHandle_t s_mpu6050_read_task_handle = NULL;

// Data protection
static portMUX_TYPE mpu_data_mutex = portMUX_INITIALIZER_UNLOCKED;
static float curr_pitch = 0.0f;
static float curr_roll = 0.0f;
static float curr_yaw = 0.0f;
static float curr_accel_g[3] = {0.0f, 0.0f, 0.0f};
static float curr_gyro_dps[3] = {0.0f, 0.0f, 0.0f};

/* LibDriver callbacks */
static void a_receive_callback(uint8_t type) { }
static void a_dmp_tap_callback(uint8_t count, uint8_t direction) { }
static void a_dmp_orient_callback(uint8_t orientation) { }

static void mpu6050_irq_task(void *arg) {
    (void)arg;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (g_gpio_irq != NULL) {
            while (gpio_get_level(INTERRUPT_MPU6050_IO) == 0) {
                (void)g_gpio_irq();
            }
        }
        (void)gpio_intr_enable(INTERRUPT_MPU6050_IO);

        /* Wake the reading task to pull FIFO data in task-context. */
        if (s_mpu6050_read_task_handle != NULL) {
            xTaskNotifyGive(s_mpu6050_read_task_handle);
        }
    }
}

static void IRAM_ATTR mpu6050_gpio_isr_handler(void *arg) {
    (void)arg;
    (void)gpio_intr_disable(INTERRUPT_MPU6050_IO);
    if (s_mpu6050_irq_task_handle != NULL) {
        BaseType_t high_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_mpu6050_irq_task_handle, &high_task_woken);
        if (high_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static int gpio_interrupt_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INTERRUPT_MPU6050_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return 1;

    if (!s_gpio_isr_service_installed) {
        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return 1;
        s_gpio_isr_service_installed = true;
    }

    err = gpio_isr_handler_add(INTERRUPT_MPU6050_IO, mpu6050_gpio_isr_handler, NULL);
    if (err != ESP_OK) return 1;

    (void)gpio_intr_disable(INTERRUPT_MPU6050_IO);
    return 0;
}

static void mpu_reading_task(void *pvParameters) {
    ESP_LOGI(TAG, "MPU6050 reading task started");
    uint16_t len;
    uint8_t res;

    while (1) {
    #if (MPU6050_READ_USE_INTERRUPT != 0)
        /* Only read when IRQ task indicates new data/event. */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#else
        /* Polling fallback (ms). */
        vTaskDelay(pdMS_TO_TICKS(MPU6050_POLL_PERIOD_MS));
#endif

        len = 128;
        res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                  gs_gyro_raw, gs_gyro_dps,
                                  gs_quat,
                                  gs_pitch, gs_roll, gs_yaw,
                                  &len);
        if (res == 0 && len > 0) {
            uint16_t idx = (uint16_t)(len - 1); /* use newest sample */
            // Update cached values safely
            portENTER_CRITICAL(&mpu_data_mutex);
            curr_pitch = gs_pitch[idx];
            curr_roll = gs_roll[idx];
            curr_yaw = gs_yaw[idx];
            curr_accel_g[0] = gs_accel_g[idx][0];
            curr_accel_g[1] = gs_accel_g[idx][1];
            curr_accel_g[2] = gs_accel_g[idx][2];
            curr_gyro_dps[0] = gs_gyro_dps[idx][0];
            curr_gyro_dps[1] = gs_gyro_dps[idx][1];
            curr_gyro_dps[2] = gs_gyro_dps[idx][2];
            portEXIT_CRITICAL(&mpu_data_mutex);

            // Optional debug print
            // ESP_LOGD(TAG, "Yaw: %.2f", curr_yaw);
        }

    }
}

int mpu6050_controller_init(void) {
    if (gpio_interrupt_init() != 0) {
        ESP_LOGE(TAG, "GPIO interrupt init failed.");
        return 1;
    }

    if (xTaskCreatePinnedToCore(mpu6050_irq_task, "mpu6050_irq", 4096, NULL, 10, &s_mpu6050_irq_task_handle, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create mpu6050_irq task.");
        return 1;
    }

    g_gpio_irq = mpu6050_dmp_irq_handler;

    uint8_t res = mpu6050_dmp_init(MPU6050_ADDRESS_AD0_LOW, a_receive_callback, a_dmp_tap_callback, a_dmp_orient_callback);
    if (res != 0) {
        ESP_LOGE(TAG, "mpu6050_dmp_init failed.");
        g_gpio_irq = NULL;
        return 1;
    }

    (void)gpio_intr_enable(INTERRUPT_MPU6050_IO);

    // Give sensor/DMP some time after init
    vTaskDelay(pdMS_TO_TICKS(500));

    // Spawn task to read DMP (IRQ-driven by default)
    xTaskCreate(mpu_reading_task, "mpu6050_read", 4096, NULL, 4, &s_mpu6050_read_task_handle);

    ESP_LOGI(TAG, "MPU6050 initialized successfully.");
    return 0;
}

void mpu6050_get_euler_angles(float *pitch, float *roll, float *yaw) {
    portENTER_CRITICAL(&mpu_data_mutex);
    if (pitch) *pitch = curr_pitch;
    if (roll)  *roll  = curr_roll;
    if (yaw)   *yaw   = curr_yaw;
    portEXIT_CRITICAL(&mpu_data_mutex);
}

void mpu6050_get_accel_gyro(float accel_g_out[3], float gyro_dps_out[3]) {
    portENTER_CRITICAL(&mpu_data_mutex);
    if (accel_g_out) {
        accel_g_out[0] = curr_accel_g[0];
        accel_g_out[1] = curr_accel_g[1];
        accel_g_out[2] = curr_accel_g[2];
    }
    if (gyro_dps_out) {
        gyro_dps_out[0] = curr_gyro_dps[0];
        gyro_dps_out[1] = curr_gyro_dps[1];
        gyro_dps_out[2] = curr_gyro_dps[2];
    }
    portEXIT_CRITICAL(&mpu_data_mutex);
}
