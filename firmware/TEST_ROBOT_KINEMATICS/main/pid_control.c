#include "pid_control.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void pid_init(pid_const_t *pid, float kp, float ki, float kd, float limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = limit;
    pid->integral_limit = limit * 0.5f; // Chống bão hòa tích phân
    pid->error_prev = 0;
    pid->integral = 0;
}

float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief Chuẩn hóa góc về khoảng [-PI, PI] để tính toán sai số góc chính xác
 */
float normalize_angle_rad(float angle_rad) {
    while (angle_rad > M_PI) angle_rad -= 2.0f * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0f * M_PI;
    return angle_rad;
}

/**
 * @brief Tính toán PID cho đơn vị Radian
 * @param setpoint Góc đích (radian)
 * @param measure  Góc hiện tại đọc từ cảm biến (radian)
 * @param dt       Thời gian lấy mẫu (giây)
 */
float pid_compute(pid_const_t *pid, float setpoint, float measure, float dt) {
    // 1. Tính sai số góc và chuẩn hóa về [-PI, PI]
    float error = normalize_angle_rad(setpoint - measure);
    
    // 2. Thành phần Tích phân (Integral)
    pid->integral += error * dt;
    
    // Chống bão hòa tích phân (Anti-windup)
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // 3. Thành phần Đạo hàm (Derivative)
    float derivative = (error - pid->error_prev) / dt;
    
    // 4. Tính toán đầu ra (Đơn vị: rad/s)
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // 5. Giới hạn đầu ra theo MAX_RAD_S đã định nghĩa trong robot_controller.h
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    pid->error_prev = error;
    return output;
}

void pid_reset(pid_const_t *pid) {
    pid->error_prev = 0;
    pid->integral = 0;
}