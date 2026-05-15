#include "robot_controller.h"
#include "driver_pca9685_basic.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "pid_control.h"
#include "sensor_mpu6050.h"
#include <math.h>

static const char *TAG = "ROBOT_CTRL";

// Macro chuyển đổi
#define DEG_TO_RAD(deg) ((deg) * (float)M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / (float)M_PI)

// Các hằng số vật lý
static const float R = 0.0375f;      // Bán kính bánh xe
static const float B = 0.1075f;       // Khoảng cách 2 bánh

static float inv_R;
static float B_half;

static float target_vx = 0.0f;
static float target_wz = 0.0f;
static portMUX_TYPE velocity_mutex = portMUX_INITIALIZER_UNLOCKED;

typedef enum {
    ROBOT_STATE_IDLE,
    ROBOT_STATE_DRIVE_STRAIGHT,
    ROBOT_STATE_TURN
} robot_mode_t;

static robot_mode_t current_mode = ROBOT_STATE_IDLE;
static float target_yaw_deg = 0.0f; // Lưu đơn vị độ để dễ debug
static pid_const_t yaw_pid;

// Cấu hình PID
#define YAW_KP  2.5f  // Cần tăng Kp vì sai số Radian nhỏ hơn Độ rất nhiều (1 rad ~ 57 độ)
#define YAW_KI  0.0f
#define YAW_KD  0.0f
#define MAX_WZ_RAD_S  15.0f // Giới hạn vận tốc góc đầu ra (rad/s)

static float map_speed_to_pwm(float rad_s) {
    float pwm = (rad_s / MAX_RAD_S) * 100.0f;
    if (pwm > 99.9f) pwm = 99.9f;
    if (pwm < -99.9f) pwm = -99.9f;
    return pwm;
}

void robot_controller_init_pid(void) {
    // Lưu ý: MAX_WZ_RAD_S ở đây là limit cho output của PID
    pid_init(&yaw_pid, YAW_KP, YAW_KI, YAW_KD, MAX_WZ_RAD_S);
}

// API: Đi thẳng, khóa hướng hiện tại
void robot_drive_straight(float vx) {
    float p, r, y;
    mpu6050_get_euler_angles(&p, &r, &y);
    
    portENTER_CRITICAL(&velocity_mutex);
    target_vx = vx;
    target_yaw_deg = y; 
    current_mode = ROBOT_STATE_DRIVE_STRAIGHT;
    portEXIT_CRITICAL(&velocity_mutex);
    
    pid_reset(&yaw_pid);
    ESP_LOGI(TAG, "Drive Straight: Speed=%.2f, Lock Yaw=%.2f", vx, y);
}

// API: Xoay tương đối (ví dụ +90.0)
void robot_turn_relative(float delta_yaw_deg) {
    float p, r, y;
    mpu6050_get_euler_angles(&p, &r, &y);

    portENTER_CRITICAL(&velocity_mutex);
    target_vx = 0;
    // Sử dụng normalize_angle (đơn vị Độ) để tính đích đến
    target_yaw_deg = normalize_angle(y + delta_yaw_deg);
    current_mode = ROBOT_STATE_TURN;
    portEXIT_CRITICAL(&velocity_mutex);
    
    pid_reset(&yaw_pid);
    ESP_LOGI(TAG, "Turn Relative: Target Yaw=%.2f", target_yaw_deg);
}

void motor_control(float left_speed, float right_speed) {
    if (left_speed > 0) {
        pca9685_basic_write(PWM_RPWM_LEFT, 0.0f, left_speed);
        pca9685_basic_write(PWM_LPWM_LEFT, 0.0f, 0.0f);
    } else if (left_speed < 0) {
        pca9685_basic_write(PWM_RPWM_LEFT, 0.0f, 0.0f);
        pca9685_basic_write(PWM_LPWM_LEFT, 0.0f, -left_speed);
    } else {
        pca9685_basic_write(PWM_RPWM_LEFT, 0.0f, 0.0f);
        pca9685_basic_write(PWM_LPWM_LEFT, 0.0f, 0.0f);
    }

    if (right_speed > 0) {
        pca9685_basic_write(PWM_RPWM_RIGHT, 0.0f, right_speed);
        pca9685_basic_write(PWM_LPWM_RIGHT, 0.0f, 0.0f);
    } else if (right_speed < 0) {
        pca9685_basic_write(PWM_RPWM_RIGHT, 0.0f, 0.0f);
        pca9685_basic_write(PWM_LPWM_RIGHT, 0.0f, -right_speed);
    } else {
        pca9685_basic_write(PWM_RPWM_RIGHT, 0.0f, 0.0f);
        pca9685_basic_write(PWM_LPWM_RIGHT, 0.0f, 0.0f);
    }
}

void kinematics_init(void) {
    inv_R = 1.0f / R;
    B_half = B / 2.0f;
    uint8_t res = pca9685_basic_init(PCA9685_ADDRESS_A000000, 1000);
    if (res != 0) ESP_LOGE(TAG, "PCA9685 Fail");
}

wheel_velocity_t inverse_kinematics(robot_velocity_t robot) {
    wheel_velocity_t wheel;
    float v_left_mps  = robot.vx - (robot.wz * B_half);
    float v_right_mps = robot.vx + (robot.wz * B_half);
    wheel.left  = v_left_mps * inv_R;
    wheel.right = -(v_right_mps * inv_R); 
    return wheel;
}

void move_robot(float vx, float wz) {
    robot_velocity_t target = { .vx = vx, .wz = wz };
    wheel_velocity_t wheels = inverse_kinematics(target);
    motor_control(map_speed_to_pwm(wheels.left), map_speed_to_pwm(wheels.right));
}

void robot_controller_set_velocity(float vx, float wz) {
    portENTER_CRITICAL(&velocity_mutex);
    target_vx = vx;
    target_wz = wz;
    current_mode = ROBOT_STATE_IDLE; // Chế độ điều khiển trực tiếp
    portEXIT_CRITICAL(&velocity_mutex);
    
    ESP_LOGI(TAG, "Set Velocity: vx=%.2f m/s, wz=%.2f rad/s", vx, wz);
}

// --- CẬP NHẬT CHÍNH TRONG TASK ĐIỀU KHIỂN ---
static void robot_control_task(void *pvParameters) {
    ESP_LOGI(TAG, "Robot control task started");
    robot_controller_init_pid();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float dt = 0.01f; // 10ms

    while (1) {
        float p, r, cur_y_deg;
        mpu6050_get_euler_angles(&p, &r, &cur_y_deg);

        float vx_out = 0, wz_out = 0;

        portENTER_CRITICAL(&velocity_mutex);
        if (current_mode != ROBOT_STATE_IDLE) {
            // Chuyển đổi sang Radian để đưa vào bộ PID
            float setpoint_rad = DEG_TO_RAD(target_yaw_deg);
            float measure_rad  = DEG_TO_RAD(cur_y_deg);

            vx_out = target_vx;
            // pid_compute nhận rad, trả về rad/s
            wz_out = pid_compute(&yaw_pid, setpoint_rad, measure_rad, dt);

            if (current_mode == ROBOT_STATE_TURN) {
                // Nếu sai số < 1 độ, coi như xoay xong
                if (fabsf(normalize_angle(target_yaw_deg - cur_y_deg)) < 1.0f) {
                    // Bạn có thể chuyển về IDLE hoặc giữ nguyên để khóa góc
                    // current_mode = ROBOT_STATE_IDLE; 
                }
            }
        } else {
            vx_out = target_vx;
            wz_out = target_wz;
        }
        portEXIT_CRITICAL(&velocity_mutex);

        move_robot(vx_out, wz_out);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void robot_controller_task_start(void) {
    xTaskCreate(robot_control_task, "robot_ctrl_task", 4096, NULL, 5, NULL);
}