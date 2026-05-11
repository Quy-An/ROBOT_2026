#include "robot_controller.h"
#include "robot_kinematics.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char *TAG = "ROBOT_CTRL";

static float target_vx = 0.0f;
static float target_wz = 0.0f;
static portMUX_TYPE velocity_mutex = portMUX_INITIALIZER_UNLOCKED;

static void robot_control_task(void *pvParameters) {
    ESP_LOGI(TAG, "Robot control task started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz loop

    while (1) {
        float vx, wz;

        // Safely fetch target velocity
        portENTER_CRITICAL(&velocity_mutex);
        vx = target_vx;
        wz = target_wz;
        portEXIT_CRITICAL(&velocity_mutex);

        // Output to motors
        move_robot(vx, wz);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void robot_controller_set_velocity(float vx, float wz) {
    portENTER_CRITICAL(&velocity_mutex);
    target_vx = vx;
    target_wz = wz;
    portEXIT_CRITICAL(&velocity_mutex);
}

void robot_controller_task_start(void) {
    xTaskCreate(robot_control_task, "robot_ctrl_task", 4096, NULL, 5, NULL);
}
