#include "robot_kinematics.h"
#include "driver_pca9685_basic.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "ROBOT_KINEMATICS";

// Các hằng số vật lý
static const float R = 0.0375f;      // Bán kính bánh xe
static const float B = 0.1075f;       // Khoảng cách 2 bánh (0.05375 * 2)

// Các hằng số tính toán sẵn (Pre-computed constants) để giảm phép chia/nhân trong vòng lặp
static float inv_R;
static float B_half;

static /**
 * @brief Quy đổi vận tốc rad/s sang PWM 0-100
 */
float map_speed_to_pwm(float rad_s) {
    // Tính phần trăm dựa trên vận tốc tối đa
    float pwm = (rad_s / MAX_RAD_S) * 100.0f;

    // Giới hạn trong khoảng 0-100 (Sử dụng 99.9f do thư viện PCA9685 giới hạn tổng delay_percent + duty <= 100)
    // Thư viện có bug kiểm tra ">= 100.0f" nên 100.0f sẽ bị báo lỗi.
    if (pwm > 99.9f) pwm = 99.9f;
    if (pwm < -99.9f) pwm = -99.9f;

    return pwm;
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

    /* 2) Initialize the PCA9685 driver */
    ESP_LOGI(TAG, "Initializing PCA9685...");
    uint8_t res = pca9685_basic_init(PCA9685_ADDRESS_A000000, 1000); // 1000Hz PWM
    if (res != 0) {
        ESP_LOGE(TAG, "PCA9685 initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "PCA9685 initialized successfully.");
}

/**
 * ĐỘNG HỌC NGHỊCH: Từ vận tốc robot tính ra tốc độ bánh xe
 * Theo yêu cầu: Tiến (+vx) -> Bánh trái (+), Bánh phải (-)
 */
wheel_velocity_t inverse_kinematics(robot_velocity_t robot) {
    wheel_velocity_t wheel;

    // Vận tốc tuyến tính của từng bánh xe (m/s)
    float v_left_mps  = robot.vx - (robot.wz * B_half);
    float v_right_mps = robot.vx + (robot.wz * B_half);

    // Chuyển sang rad/s và áp dụng quy ước chiều quay
    wheel.left  = v_left_mps * inv_R;           // Tiến -> rad/s dương
    wheel.right = -(v_right_mps * inv_R);       // Tiến -> rad/s âm (theo yêu cầu)

    return wheel;
}

/**
 * ĐỘNG HỌC THUẬN: Từ tốc độ bánh xe tính ra vận tốc robot
 */
robot_velocity_t forward_kinematics(wheel_velocity_t wheel) {
    robot_velocity_t robot;

    // Chuyển về vận tốc m/s (nhớ đảo lại dấu bánh phải theo quy ước)
    float v_l = wheel.left * R;
    float v_r = -wheel.right * R; 

    // Công thức ROS 2 Differential Drive
    robot.vx = (v_l + v_r) / 2.0f;
    robot.wz = (v_r - v_l) / B;

    return robot;
}

void move_robot(float vx, float wz) {
    robot_velocity_t target = { .vx = vx, .wz = wz };
    wheel_velocity_t wheels = inverse_kinematics(target);

    // Chuyển tốc độ bánh xe sang PWM
    float left_pwm  = map_speed_to_pwm(wheels.left);
    float right_pwm = map_speed_to_pwm(wheels.right);

    // ESP_LOGI(TAG, "left_pwm: %.2f, right_pwm %.2f", left_pwm, right_pwm);

    // Gọi hàm điều khiển motor bạn đã viết (nó sẽ tự xử lý chiều dựa vào dấu +/-)
    motor_control(left_pwm, right_pwm);
}