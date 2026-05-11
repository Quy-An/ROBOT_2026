#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BTS7960.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "ROBOT_CONTROL";

/**
 * @brief Hàm delay không block task (chuẩn FreeRTOS)
 */
static void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Xe đi tiến
 */
void robot_move_forward(int speed, int duration_ms) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    ESP_LOGI(TAG, "Move forward with speed %d%%", speed);
    BTS7960_set_left(speed);
    BTS7960_set_right(speed);
    
    if (duration_ms > 0) {
        delay_ms(duration_ms);
        BTS7960_stop();
    }
}

/**
 * @brief Xe đi lùi
 */
void robot_move_backward(int speed, int duration_ms) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    ESP_LOGI(TAG, "Move backward with speed %d%%", speed);
    BTS7960_set_left(-speed);
    BTS7960_set_right(-speed);
    
    if (duration_ms > 0) {
        delay_ms(duration_ms);
        BTS7960_stop();
    }
}

/**
 * @brief Xe rẽ trái (trong khi di chuyển - rẽ cung rộng)
 */
void robot_turn_left(int speed, int duration_ms) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    ESP_LOGI(TAG, "Turn left with speed %d%%", speed);
    BTS7960_set_left(speed / 2);      // Bánh trái quay chậm lại
    BTS7960_set_right(speed);         // Bánh phải giữ nguyên
    
    if (duration_ms > 0) {
        delay_ms(duration_ms);
        BTS7960_stop();
    }
}

/**
 * @brief Xe rẽ phải (trong khi di chuyển - rẽ cung rộng)
 */
void robot_turn_right(int speed, int duration_ms) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    ESP_LOGI(TAG, "Turn right with speed %d%%", speed);
    BTS7960_set_left(speed);          // Bánh trái giữ nguyên
    BTS7960_set_right(speed / 2);     // Bánh phải quay chậm lại
    
    if (duration_ms > 0) {
        delay_ms(duration_ms);
        BTS7960_stop();
    }
}

/**
 * @brief Xe xoay tại chỗ sang trái (Spin left)
 */
void robot_spin_left(int speed, int duration_ms) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    ESP_LOGI(TAG, "Spin left with speed %d%%", speed);
    BTS7960_set_left(-speed);         // Bánh trái lùi
    BTS7960_set_right(speed);         // Bánh phải tiến
    
    if (duration_ms > 0) {
        delay_ms(duration_ms);
        BTS7960_stop();
    }
}

/**
 * @brief Xe xoay tại chỗ sang phải (Spin right)
 */
void robot_spin_right(int speed, int duration_ms) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    ESP_LOGI(TAG, "Spin right with speed %d%%", speed);
    BTS7960_set_left(speed);          // Bánh trái tiến
    BTS7960_set_right(-speed);        // Bánh phải lùi
    
    if (duration_ms > 0) {
        delay_ms(duration_ms);
        BTS7960_stop();
    }
}

/**
 * @brief Dừng xe
 */
void robot_stop(void) {
    ESP_LOGI(TAG, "Stop");
    BTS7960_stop();
}

void app_main(void)
{
    ESP_LOGI(TAG, "Robot Control System Started");
    
    // Khởi tạo PWM cho động cơ
    BTS7960_init();
    ESP_LOGI(TAG, "BTS7960 initialized");
    
    // Đợi phần cứng ổn định
    delay_ms(500); 
    
    ESP_LOGI(TAG, "=== Demo Mode ===");
    
    ESP_LOGI(TAG, "1. Tiến 2 giây...");
    robot_move_forward(100, 2000);
    delay_ms(500);
    
    ESP_LOGI(TAG, "2. Lùi 2 giây...");
    robot_move_backward(100, 2000);
    delay_ms(500);
    
    ESP_LOGI(TAG, "3. Rẽ trái 2 giây...");
    robot_turn_left(100, 2000);
    delay_ms(500);
    
    ESP_LOGI(TAG, "4. Rẽ phải 2 giây...");
    robot_turn_right(100, 2000);
    delay_ms(500);
    
    ESP_LOGI(TAG, "5. Xoay trái tại chỗ 2 giây...");
    robot_spin_left(100, 2000);
    delay_ms(500);
    
    ESP_LOGI(TAG, "6. Xoay phải tại chỗ 2 giây...");
    robot_spin_right(100, 2000);
    delay_ms(500);
    
    ESP_LOGI(TAG, "7. Dừng hẳn.");
    robot_stop();
    
    ESP_LOGI(TAG, "Demo hoàn tất. Đưa xe về chế độ nghỉ.");
    
    while (1) {
        delay_ms(1000); // Giữ cho task không bị hủy và không ăn 100% CPU
    }
}